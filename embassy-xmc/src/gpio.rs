//! GPIO API foundation.
//!
//! Pin routing is intentionally not guessed. The XMC4700 has per-pin
//! alternate-function selections and the Relax Kit board exposes several
//! possible LED/display routes. The concrete register backend is the next
//! hardware implementation step.

use core::ptr::{read_volatile, write_volatile};

const PORT0_BASE: usize = 0x4802_8000;
const PORT1_BASE: usize = 0x4802_8100;
const PORT2_BASE: usize = 0x4802_8200;
const PORT3_BASE: usize = 0x4802_8300;
const PORT4_BASE: usize = 0x4802_8400;
const PORT5_BASE: usize = 0x4802_8500;
const PORT6_BASE: usize = 0x4802_8600;
const PORT7_BASE: usize = 0x4802_8700;
const PORT8_BASE: usize = 0x4802_8800;
const PORT9_BASE: usize = 0x4802_8900;
// Port 10-13 existieren nicht
const PORT14_BASE: usize = 0x4802_8E00;
const PORT15_BASE: usize = 0x4802_8F00;

/// Offsets der Port-Register (für alle Ports P0..P9 identisch).
const OUT: usize = 0x00; // Output Register (Soll-Pegel)
const OMR: usize = 0x04; // Output Modification Register
const IOCR0: usize = 0x10; // Input/Output Control Register für Pins 0..3 (IOCR4 = 0x14, ...)
const IN: usize = 0x24; // Input Register (tatsächlicher Pegel am Pin)
const PDR0: usize = 0x40; // Pad Driver Mode Register für Pins 0..7 (PDR1 = 0x44 für Pins 8..15)
const PDISC: usize = 0x60; // Pin Function Decision Control (nur P14/P15 relevant)

/// PCx-Feld in IOCRn: General-Purpose-Ausgang, Push-Pull.
const IOCR_PC_OUTPUT_PUSH_PULL: u32 = 0b1_0000;
/// PCx-Feld in IOCRn für Eingänge: Tristate / Pull-Down / Pull-Up.
const IOCR_PC_INPUT_TRISTATE: u32 = 0b0_0000;
const IOCR_PC_INPUT_PULL_DOWN: u32 = 0b0_0001;
const IOCR_PC_INPUT_PULL_UP: u32 = 0b0_0010;
/// Höchster Port mit Ausgangsregistern. P14/P15 sind reine Analog-/Digital-Eingänge.
const MAX_OUTPUT_PORT: u8 = 9;
/// P14/P15 haben kein IOCR; der Digitaleingang wird über PDISC freigeschaltet.
const FIRST_INPUT_ONLY_PORT: u8 = 14;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Level {
    Low,
    High,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    Input,
    Output,
}

/// Interner Pull-Widerstand eines Eingangs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Pull {
    None,
    Down,
    Up,
}

/// Treiberstärke und Flankensteilheit eines Ausgangs (Wert für das PDx-Feld in PDR0/PDR1).
///
/// Welche Modi ein Pin unterstützt, hängt von seiner Pad-Klasse ab (siehe Datenblatt);
/// "Strong" gibt es nur bei Pads der Klasse A1+/A2.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum DriveStrength {
    StrongSharpEdge = 0,
    StrongMediumEdge = 1,
    StrongSoftEdge = 2,
    StrongSlowEdge = 3,
    Medium = 4,
    Weak = 7,
}

/// Describes a GPIO pin after the board-specific pin mux has been configured.
pub struct Pin {
    port: u8,
    pin: u8,
}

impl Pin {
    /// Construct a pin descriptor. This is an internal low-level escape hatch;
    /// application code should use board pin constants when those are added.
    pub const unsafe fn new_unchecked(port: u8, pin: u8) -> Self {
        assert!(pin < 16, "Invalid pin");
        Self { port, pin }
    }

    pub const fn port(&self) -> u8 {
        self.port
    }

    pub const fn number(&self) -> u8 {
        self.pin
    }

    /// Basisadresse des Ports als Zahl (Byte-Adresse).
    const fn port_base(&self) -> usize {
        match self.port {
            0 => PORT0_BASE,
            1 => PORT1_BASE,
            2 => PORT2_BASE,
            3 => PORT3_BASE,
            4 => PORT4_BASE,
            5 => PORT5_BASE,
            6 => PORT6_BASE,
            7 => PORT7_BASE,
            8 => PORT8_BASE,
            9 => PORT9_BASE,
            14 => PORT14_BASE,
            15 => PORT15_BASE,
            _ => panic!("Invalid port"),
        }
    }

    pub const fn port_adr(&self) -> *mut u32 {
        self.port_base() as *mut u32
    }

    /// Adresse eines Port-Registers. Der Offset ist in **Bytes**.
    ///
    /// Achtung: `*mut u32::add(n)` rechnet in Elementen (n * 4 Bytes). Daher wird
    /// hier auf `usize` addiert und erst danach zum Pointer gecastet.
    const fn reg(&self, offset: usize) -> *mut u32 {
        (self.port_base() + offset) as *mut u32
    }

    /// Adresse des Output Modification Register (OMR) dieses Ports.
    pub const fn omr_adr(&self) -> *mut u32 {
        self.reg(OMR)
    }

    /// Pin als Push-Pull-Ausgang der alternativen Funktion `alt` (1..=4) schalten,
    /// z. B. ALT2 = U0C0.DOUT0 an P1.5. Wird von den Peripherie-Treibern benutzt.
    pub(crate) fn set_alt_output(&self, alt: u8) {
        assert!((1..=4).contains(&alt), "ALT-Funktion muss 1..=4 sein");
        assert!(self.port <= MAX_OUTPUT_PORT, "Port hat keine Ausgangsregister");
        self.set_pc(IOCR_PC_OUTPUT_PUSH_PULL | alt as u32);
    }

    /// Pin als Eingang mit Pull-Up schalten (z. B. UART-RX, Ruhepegel High).
    pub(crate) fn set_input_pull_up(&self) {
        self.set_pc(IOCR_PC_INPUT_PULL_UP);
    }

    /// Schreibt das PCx-Feld dieses Pins im passenden IOCRn-Register.
    fn set_pc(&self, pc: u32) {
        // 4 Pins pro IOCR-Register, je 8 Bit; PCx liegt in den Bits [7:3] des Bytes.
        let reg = self.reg(IOCR0 + 4 * (self.pin as usize / 4));
        let shift = 8 * (self.pin as u32 % 4) + 3;
        // IOCR deckt vier Pins ab: Read-Modify-Write gegen Unterbrechungen absichern.
        critical_section::with(|_| unsafe {
            let v = read_volatile(reg);
            write_volatile(reg, (v & !(0x1F << shift)) | (pc << shift));
        });
    }

    /// Schreibt das PDx-Feld (3 Bit) dieses Pins in PDR0 (Pins 0..7) bzw. PDR1 (Pins 8..15).
    fn set_drive(&self, strength: DriveStrength) {
        let reg = self.reg(PDR0 + 4 * (self.pin as usize / 8));
        let shift = 4 * (self.pin as u32 % 8);
        critical_section::with(|_| unsafe {
            let v = read_volatile(reg);
            write_volatile(reg, (v & !(0x7 << shift)) | ((strength as u32) << shift));
        });
    }

    /// Schaltet bei P14/P15 den Digitaleingang frei (PDISC-Bit löschen).
    fn enable_digital_input(&self) {
        let reg = self.reg(PDISC);
        critical_section::with(|_| unsafe {
            let v = read_volatile(reg);
            write_volatile(reg, v & !(1u32 << self.pin));
        });
    }

    /// Liest den tatsächlichen Pegel am Pin (IN-Register).
    fn read_level(&self) -> Level {
        let v = unsafe { read_volatile(self.reg(IN)) };
        if (v >> self.pin) & 1 != 0 {
            Level::High
        } else {
            Level::Low
        }
    }
}

/// Output pin handle (Push-Pull).
pub struct Output {
    pin: Pin,
}

impl Output {
    pub fn new(pin: Pin, initial: Level) -> Self {
        assert!(
            pin.port <= MAX_OUTPUT_PORT,
            "Port hat keine Ausgangsregister"
        );
        let mut out = Self { pin };
        // Erst den Pegel setzen, dann den Ausgang aktivieren: kein Glitch beim Start.
        out.set_level(initial);
        out.pin.set_pc(IOCR_PC_OUTPUT_PUSH_PULL);
        out
    }

    pub fn set_level(&mut self, level: Level) {
        let pin = self.pin.pin as u32;
        // OMR: Bits 0..15 = PSn (setzen), Bits 16..31 = PRn (löschen).
        // Beides liegt im selben Register; ein einzelner Schreibzugriff genügt.
        let value = match level {
            Level::High => 1u32 << pin,
            Level::Low => 1u32 << (pin + 16),
        };
        unsafe { write_volatile(self.pin.omr_adr(), value) }
    }

    pub fn toggle(&mut self) {
        let pin = self.pin.pin as u32;
        // PSn und PRn gleichzeitig gesetzt => Toggle, atomar in Hardware.
        unsafe { write_volatile(self.pin.omr_adr(), (1u32 << pin) | (1u32 << (pin + 16))) }
    }

    /// Soll-Pegel laut OUT-Register (nicht der gemessene Pegel am Pin).
    pub fn is_set_high(&self) -> bool {
        let v = unsafe { read_volatile(self.pin.reg(OUT)) };
        (v >> self.pin.pin) & 1 != 0
    }

    /// Tatsächlicher Pegel am Pin (IN-Register), z. B. bei Überlast oder externem Treiber.
    pub fn level(&self) -> Level {
        self.pin.read_level()
    }

    /// Treiberstärke / Flankensteilheit einstellen.
    pub fn set_drive_strength(&mut self, strength: DriveStrength) {
        self.pin.set_drive(strength);
    }

    pub const fn pin(&self) -> &Pin {
        &self.pin
    }
}

/// Input pin handle.
pub struct Input {
    pin: Pin,
}

impl Input {
    pub fn new(pin: Pin, pull: Pull) -> Self {
        if pin.port >= FIRST_INPUT_ONLY_PORT {
            // P14/P15: keine Pull-Widerstände, nur Freischalten des Digitaleingangs.
            assert!(pull == Pull::None, "P14/P15 haben keine Pull-Widerstände");
            pin.enable_digital_input();
        } else {
            let pc = match pull {
                Pull::None => IOCR_PC_INPUT_TRISTATE,
                Pull::Down => IOCR_PC_INPUT_PULL_DOWN,
                Pull::Up => IOCR_PC_INPUT_PULL_UP,
            };
            pin.set_pc(pc);
        }
        Self { pin }
    }

    pub fn level(&self) -> Level {
        self.pin.read_level()
    }

    pub fn is_high(&self) -> bool {
        self.level() == Level::High
    }

    pub fn is_low(&self) -> bool {
        self.level() == Level::Low
    }

    pub const fn pin(&self) -> &Pin {
        &self.pin
    }
}
