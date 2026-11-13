//! UART (ASC-Modus des USIC) für den XMC4700, blockierend, 8N1.
//!
//! # Relax Kit: Virtueller COM-Port
//!
//! Der On-Board-Debugger (J-Link) stellt einen virtuellen COM-Port bereit
//! (unter Linux z. B. `/dev/ttyACM0`). Er hängt an **USIC0 Kanal 0**:
//!
//! | Signal | Pin  | Funktion                    |
//! |--------|------|-----------------------------|
//! | TX     | P1.5 | ALT2 = U0C0.DOUT0           |
//! | RX     | P1.4 | Eingang U0C0.DX0B (DSEL = 1) |
//!
//! [`Uart::new_debug`] richtet genau diese Verbindung ein.
//!
//! # print! / println!
//!
//! Nach [`init_print`] schreiben die Makros [`print!`](crate::print) und
//! [`println!`](crate::println) auf diesen UART (`\n` wird zu `\r\n`).
//!
//! Gesendet wird blockierend, aber **ohne** Critical Section: Interrupts (und damit der
//! Zeitgeber) laufen während der Ausgabe weiter. Der Executor ist kooperativ, Ausgaben
//! verschiedener Tasks mischen sich daher nicht. Nur ein `println!` aus einem
//! Interrupt-Handler kann sich zeichenweise in eine laufende Ausgabe schieben.

use core::fmt;
use core::sync::atomic::{AtomicBool, Ordering};

use crate::gpio::Pin;
use crate::usic::{self, rd, wr};

// ---------------------------------------------------------------------------
// Register (Adressen und Bits gegen die SVD des xmc4700-PAC geprüft)
// ---------------------------------------------------------------------------

const USIC0_CH0: usize = 0x4003_0000;

// Kanal-Register
const KSCFG: usize = 0x0C;
const FDR: usize = 0x10;
const BRG: usize = 0x14;
const DX0CR: usize = 0x1C;
const SCTR: usize = 0x34;
const TCSR: usize = 0x38;
const PCR: usize = 0x3C;
const CCR: usize = 0x40;
const PSR: usize = 0x48;
const PSCR: usize = 0x4C;
const RBUFSR: usize = 0x50;
const RBUF: usize = 0x54;
const TBUF0: usize = 0x80;

const KSCFG_MODEN: u32 = 1 << 0;
const KSCFG_BPMODEN: u32 = 1 << 1;
const FDR_DM_FRACTIONAL: u32 = 0b10 << 14;
/// Überabtastung: 16 Zeitquanten pro Bit (BRG.DCTQ = 15).
const OVERSAMPLING: u32 = 16;
const SCTR_PDL: u32 = 1 << 1; // Ruhepegel High
const SCTR_TRM: u32 = 0b01 << 8; // Schieben aktiv bei Pegel 1
const SCTR_FLE_8: u32 = 7 << 16; // Rahmenlänge 8 Bit
const SCTR_WLE_8: u32 = 7 << 24; // Wortlänge 8 Bit
const TCSR_TDV: u32 = 1 << 7; // TBUF enthält noch ungesendete Daten
const TCSR_TDSSM: u32 = 1 << 8; // Single-Shot: TDV wird nach dem Übernehmen gelöscht
const TCSR_TDEN: u32 = 0b01 << 10; // Senden starten, sobald TDV = 1
const PCR_SMD: u32 = 1 << 0; // 3 Abtastungen pro Bit, Mehrheitsentscheid
const CCR_MODE_ASC: u32 = 2;
const PSR_BUSY: u32 = 1 << 9;
const PSR_RIF: u32 = 1 << 14;
const PSR_AIF: u32 = 1 << 15;
const PSR_ALL: u32 = 0x1_FFFF;
const RBUFSR_RDV0: u32 = 1 << 13;
const RBUFSR_RDV1: u32 = 1 << 14;
/// DX0CR.DSEL: Eingang DX0B
const DX0_DSEL_B: u32 = 1;

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    /// Baudrate in Bit/s
    pub baudrate: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self { baudrate: 115_200 }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// Baudrate mit dem Peripherie-Takt nicht erreichbar (Abweichung > 2 %)
    InvalidBaudrate,
}

// ---------------------------------------------------------------------------
// Treiber
// ---------------------------------------------------------------------------

/// UART auf USIC0 Kanal 0 (8 Datenbits, keine Parität, 1 Stoppbit).
pub struct Uart {
    base: usize,
}

impl Uart {
    /// UART für den virtuellen COM-Port des Relax Kits: TX = P1.5, RX = P1.4.
    ///
    /// `f_periph_hz` ist der Peripherie-Takt (siehe [`crate::clocks::peripheral_clock_hz`]).
    pub fn new_debug(f_periph_hz: u32, config: Config) -> Result<Self, Error> {
        let div = usic::calc_divider(f_periph_hz, config.baudrate, OVERSAMPLING, false)
            // Mehr als 2 % Abweichung verträgt UART nicht zuverlässig.
            .filter(|d| d.actual.abs_diff(config.baudrate) as u64 * 50 <= config.baudrate as u64)
            .ok_or(Error::InvalidBaudrate)?;
        let base = USIC0_CH0;
        // Ohne das stürzt die Formatierung von Zahlen in `println!` ab (siehe `crate::init`).
        crate::init();

        // 1. USIC0 takten und aus dem Reset holen.
        usic::enable_module(usic::Module::Usic0);

        // 2. Kanal einschalten (Schreibzugriff mit Bit-Schutz), danach Kanal deaktivieren,
        //    solange konfiguriert wird.
        wr(base + KSCFG, KSCFG_MODEN | KSCFG_BPMODEN);
        while rd(base + KSCFG) & KSCFG_MODEN == 0 {}
        wr(base + CCR, 0);

        // 3. Baudrate
        wr(base + FDR, FDR_DM_FRACTIONAL | div.step);
        wr(base + BRG, ((div.pdiv - 1) << 16) | ((OVERSAMPLING - 1) << 10));

        // 4. Rahmen: 8 Bit, LSB zuerst, Ruhepegel High
        wr(base + SCTR, SCTR_PDL | SCTR_TRM | SCTR_FLE_8 | SCTR_WLE_8);
        wr(base + TCSR, TCSR_TDSSM | TCSR_TDEN);
        // Abtastpunkt in der Bitmitte (Quantum 9 von 16), 1 Stoppbit
        let sample_point = OVERSAMPLING / 2 + 1;
        wr(base + PCR, PCR_SMD | (sample_point << 8));

        // 5. RX-Eingang: P1.4 als DX0B
        wr(base + DX0CR, DX0_DSEL_B);

        // 6. Statusflags löschen und ASC-Modus aktivieren
        wr(base + PSCR, PSR_ALL);
        wr(base + CCR, CCR_MODE_ASC);

        // 7. Pins erst jetzt umschalten, damit TX gleich mit Ruhepegel High startet.
        unsafe {
            Pin::new_unchecked(1, 4).set_input_pull_up();
            Pin::new_unchecked(1, 5).set_alt_output(2);
        }

        Ok(Self { base })
    }

    /// Ein Byte senden (wartet, bis der Sendepuffer frei ist).
    pub fn write_byte(&mut self, byte: u8) {
        write_byte_raw(self.base, byte);
    }

    /// Bytes senden.
    pub fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_byte(b);
        }
    }

    /// Wartet, bis das letzte Byte vollständig gesendet ist.
    pub fn flush(&mut self) {
        while rd(self.base + TCSR) & TCSR_TDV != 0 {}
        while rd(self.base + PSR) & PSR_BUSY != 0 {}
    }

    /// Ein empfangenes Byte abholen, falls vorhanden.
    pub fn try_read_byte(&mut self) -> Option<u8> {
        if rd(self.base + RBUFSR) & (RBUFSR_RDV0 | RBUFSR_RDV1) == 0 {
            return None;
        }
        wr(self.base + PSCR, PSR_RIF | PSR_AIF);
        Some(rd(self.base + RBUF) as u8)
    }

    /// Blockierend auf ein Byte warten.
    pub fn read_byte(&mut self) -> u8 {
        loop {
            if let Some(b) = self.try_read_byte() {
                return b;
            }
        }
    }
}

fn write_byte_raw(base: usize, byte: u8) {
    while rd(base + TCSR) & TCSR_TDV != 0 {}
    wr(base + TBUF0, byte as u32);
}

impl fmt::Write for Uart {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        self.write(s.as_bytes());
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// print! / println!
// ---------------------------------------------------------------------------

/// `true`, sobald [`init_print`] gelaufen ist. Vorher verwerfen die Makros ihre Ausgabe.
static PRINT_READY: AtomicBool = AtomicBool::new(false);

/// Macht `uart` zur Ausgabe für [`print!`](crate::print) / [`println!`](crate::println).
pub fn init_print(uart: Uart) {
    // Der Kanal ist fest (USIC0 CH0); der Handle selbst hält keinen weiteren Zustand.
    debug_assert_eq!(uart.base, USIC0_CH0);
    core::mem::forget(uart);
    PRINT_READY.store(true, Ordering::Release);
}

/// Schreiber hinter den Makros: wandelt `\n` in `\r\n` für Serial-Monitore.
struct PrintWriter;

impl fmt::Write for PrintWriter {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        for b in s.bytes() {
            if b == b'\n' {
                write_byte_raw(USIC0_CH0, b'\r');
            }
            write_byte_raw(USIC0_CH0, b);
        }
        Ok(())
    }
}

#[doc(hidden)]
pub fn _print(args: fmt::Arguments<'_>) {
    if PRINT_READY.load(Ordering::Acquire) {
        let _ = fmt::Write::write_fmt(&mut PrintWriter, args);
    }
}

/// Wie `std::print!`, Ausgabe auf den mit [`uart::init_print`](crate::uart::init_print)
/// gewählten UART.
#[macro_export]
macro_rules! print {
    ($($arg:tt)*) => {
        $crate::uart::_print(core::format_args!($($arg)*))
    };
}

/// Wie `std::println!`, Ausgabe auf den mit [`uart::init_print`](crate::uart::init_print)
/// gewählten UART.
#[macro_export]
macro_rules! println {
    () => {
        $crate::print!("\n")
    };
    ($($arg:tt)*) => {
        $crate::uart::_print(core::format_args!("{}\n", core::format_args!($($arg)*)))
    };
}
