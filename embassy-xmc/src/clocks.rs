//! XMC4700 Takt-Einrichtung (SCU: OSC_HP, Haupt-PLL, Taktteiler, Flash-Wartezyklen).
//!
//! [`init_144mhz`] stellt den Takt wie der Infineon-Startup-Code (`SystemInit`) ein:
//!
//! ```text
//! Quarz 12 MHz ─ OSC_HP ─ P=2 ─ VCO ×48 = 288 MHz ─ K2=2 ─ fPLL = 144 MHz = fSYS
//! fCPU = fPERIPH = fCCU = 144 MHz
//! ```
//!
//! Ohne diese Einrichtung läuft der Chip auf dem internen Backup-Oszillator fOFI. Dessen
//! Frequenz ist ungenau (auf dem Test-Board gemessen ca. 36 MHz statt nominal 24 MHz),
//! daher stimmen dann weder Baudraten noch Embassy-Zeiten.
//!
//! Die Abfragefunktionen ([`sys_clock_hz`] usw.) lesen die Register und rechnen die
//! Frequenzen aus; sie gelten also auch für einen von anderer Stelle eingestellten Takt.

use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{AtomicU32, Ordering};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Clocks {
    pub f_sys_hz: u32,
    pub f_cpu_hz: u32,
    pub f_peripheral_hz: u32,
    pub f_ccu_hz: u32,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// Der Quarz-Oszillator schwingt nicht an (Quarz fehlt oder falsche Frequenz).
    OscillatorNotUsable,
    /// Die PLL rastet nicht ein.
    PllNoLock,
}

// ---------------------------------------------------------------------------
// Register (Adressen und Bits gegen die SVD des xmc4700-PAC geprüft)
// ---------------------------------------------------------------------------

const SCU_CLK_SYSCLKCR: usize = 0x5000_460C;
const SCU_CLK_CPUCLKCR: usize = 0x5000_4610;
const SCU_CLK_PBCLKCR: usize = 0x5000_4614;
const SCU_CLK_CCUCLKCR: usize = 0x5000_4620;
const SYSCLKCR_SYSSEL_PLL: u32 = 1 << 16;

const SCU_OSC_OSCHPCTRL: usize = 0x5000_4704;
const OSCHPCTRL_MODE: u32 = 0b11 << 4; // 00 = Quarzbetrieb
const OSCHPCTRL_OSCVAL: u32 = 0xF << 16;

const SCU_PLL_PLLSTAT: usize = 0x5000_4710;
const SCU_PLL_PLLCON0: usize = 0x5000_4714;
const SCU_PLL_PLLCON1: usize = 0x5000_4718;
const SCU_PLL_PLLCON2: usize = 0x5000_471C;
const PLLSTAT_VCOBYST: u32 = 1 << 0;
const PLLSTAT_VCOLOCK: u32 = 1 << 2;
/// PLLLV | PLLHV | PLLSP: Oszillator für die PLL brauchbar
const PLLSTAT_OSC_USABLE: u32 = 0b111 << 7;
const PLLCON0_VCOBYP: u32 = 1 << 0;
const PLLCON0_VCOPWD: u32 = 1 << 1;
const PLLCON0_FINDIS: u32 = 1 << 4;
const PLLCON0_OSCDISCDIS: u32 = 1 << 6;
const PLLCON0_PLLPWD: u32 = 1 << 16;
const PLLCON0_OSCRES: u32 = 1 << 17;
const PLLCON0_RESLD: u32 = 1 << 18;
const PLLCON2_PINSEL: u32 = 1 << 0; // 1 = fOFI, 0 = OSC_HP
const PLLCON2_K1INSEL: u32 = 1 << 8;

const FLASH0_FCON: usize = 0x5800_2014;
const FCON_WSPFLASH: u32 = 0xF;

/// Interner Backup-Oszillator fOFI (nominal).
const F_OFI_HZ: u32 = 24_000_000;
/// Referenz für OSCHPCTRL.OSCVAL
const F_OSCREF_HZ: u32 = 2_500_000;

/// Frequenz des Quarzes an OSC_HP, sobald [`init_144mhz`] gelaufen ist (0 = unbekannt).
static F_OSC_HZ: AtomicU32 = AtomicU32::new(0);

fn rd(addr: usize) -> u32 {
    // Feste SCU-/Flash-Adressen, siehe Konstanten oben.
    unsafe { read_volatile(addr as *const u32) }
}

fn wr(addr: usize, value: u32) {
    unsafe { write_volatile(addr as *mut u32, value) }
}

fn modify(addr: usize, clear: u32, set: u32) {
    wr(addr, (rd(addr) & !clear) | set);
}

/// Wartet, bis `rd(addr) & mask == want`; `false` nach `max_loops` Versuchen.
fn wait_for(addr: usize, mask: u32, want: u32, max_loops: u32) -> bool {
    for _ in 0..max_loops {
        if rd(addr) & mask == want {
            return true;
        }
    }
    false
}

// ---------------------------------------------------------------------------
// Einrichtung
// ---------------------------------------------------------------------------

/// Quarz des Relax Kits
pub const RELAX_KIT_XTAL_HZ: u32 = 12_000_000;

/// PLL-Teiler: fVCO = fOSC / P * N = 288 MHz (zulässig 260..520 MHz)
const PLL_P: u32 = 2;
const PLL_N: u32 = 48;
/// K2-Stufen: 24, 48, 72, 96 und zuletzt 144 MHz. Der Systemtakt wird in Schritten
/// angehoben, damit der Stromsprung die Kernspannung nicht einbrechen lässt.
const PLL_K2_STEPS: [u32; 5] = [12, 6, 4, 3, 2];
/// PFLASH-Wartezyklen für 144 MHz (Zugriffszeit 22 ns ⇒ mindestens 4 Takte)
const FLASH_WAIT_STATES_144MHZ: u32 = 4;

fn pllcon1(k2: u32) -> u32 {
    ((PLL_P - 1) << 24) | ((k2 - 1) << 16) | ((PLL_N - 1) << 8)
}

/// Systemtakt 144 MHz aus dem 12-MHz-Quarz des Relax Kits einstellen.
///
/// Muss **vor** [`crate::uart::Uart`] und [`crate::time_drivers::init`] aufgerufen werden,
/// weil beide ihre Teiler aus dem aktuellen Takt berechnen. Schlägt der Quarz fehl, bleibt
/// der Chip auf fOFI und der Fehler wird zurückgegeben.
pub fn init_144mhz() -> Result<Clocks, Error> {
    init_pll(RELAX_KIT_XTAL_HZ)
}

fn init_pll(xtal_hz: u32) -> Result<Clocks, Error> {
    crate::init();

    // Läuft das System schon auf der PLL (z. B. nach einem Warmstart ohne Reset der SCU),
    // erst auf fOFI zurückschalten: Die PLL wird gleich umkonfiguriert.
    if rd(SCU_CLK_SYSCLKCR) & SYSCLKCR_SYSSEL_PLL != 0 {
        wr(SCU_CLK_SYSCLKCR, 0);
        cortex_m::asm::delay(1_000);
    }

    // 1. Flash-Wartezyklen vor dem Hochtakten setzen.
    modify(FLASH0_FCON, FCON_WSPFLASH, FLASH_WAIT_STATES_144MHZ);

    // 2. PLL einschalten.
    modify(SCU_PLL_PLLCON0, PLLCON0_VCOPWD | PLLCON0_PLLPWD, 0);

    // 3. OSC_HP im Quarzbetrieb starten und als PLL-Eingang wählen.
    let oscval = (xtal_hz / F_OSCREF_HZ) - 1;
    modify(SCU_OSC_OSCHPCTRL, OSCHPCTRL_MODE | OSCHPCTRL_OSCVAL, oscval << 16);
    modify(SCU_PLL_PLLCON2, PLLCON2_PINSEL | PLLCON2_K1INSEL, 0);
    // Oszillator-Watchdog neu starten
    modify(SCU_PLL_PLLCON0, PLLCON0_OSCRES, 0);
    // Anschwingen dauert einige ms (bei 24..36 MHz fOFI reichen 10^6 Durchläufe gut aus).
    if !wait_for(SCU_PLL_PLLSTAT, PLLSTAT_OSC_USABLE, PLLSTAT_OSC_USABLE, 1_000_000) {
        return Err(Error::OscillatorNotUsable);
    }

    // 4. PLL im Bypass auf 24 MHz konfigurieren und einrasten lassen.
    modify(SCU_PLL_PLLCON0, 0, PLLCON0_VCOBYP);
    modify(SCU_PLL_PLLCON0, 0, PLLCON0_FINDIS);
    wr(SCU_PLL_PLLCON1, pllcon1(PLL_K2_STEPS[0]));
    modify(SCU_PLL_PLLCON0, 0, PLLCON0_OSCDISCDIS);
    modify(SCU_PLL_PLLCON0, PLLCON0_FINDIS, 0);
    modify(SCU_PLL_PLLCON0, 0, PLLCON0_RESLD);
    if !wait_for(SCU_PLL_PLLSTAT, PLLSTAT_VCOLOCK, PLLSTAT_VCOLOCK, 1_000_000) {
        return Err(Error::PllNoLock);
    }

    // 5. Bypass aus: fPLL = 24 MHz
    modify(SCU_PLL_PLLCON0, PLLCON0_VCOBYP, 0);
    if !wait_for(SCU_PLL_PLLSTAT, PLLSTAT_VCOBYST, 0, 1_000_000) {
        return Err(Error::PllNoLock);
    }

    // 6. Teiler: fCPU = fPERIPH = fCCU = fSYS, dann System auf die PLL umschalten.
    wr(SCU_CLK_CPUCLKCR, 0);
    wr(SCU_CLK_PBCLKCR, 0);
    wr(SCU_CLK_CCUCLKCR, 0);
    wr(SCU_CLK_SYSCLKCR, SYSCLKCR_SYSSEL_PLL);
    F_OSC_HZ.store(xtal_hz, Ordering::Relaxed);

    // 7. In Stufen auf 144 MHz. Je Stufe ca. 50 µs warten (bei bis zu 96 MHz < 10_000 Takte).
    for &k2 in &PLL_K2_STEPS[1..] {
        cortex_m::asm::delay(10_000);
        wr(SCU_PLL_PLLCON1, pllcon1(k2));
    }
    cortex_m::asm::delay(10_000);

    current().ok_or(Error::PllNoLock)
}

// ---------------------------------------------------------------------------
// Takt aus den SCU-Registern ablesen
// ---------------------------------------------------------------------------

/// Ausgangsfrequenz der Haupt-PLL. `None`, wenn der Quarz-Eingang nicht bekannt ist.
fn pll_clock_hz() -> Option<u32> {
    let f_in = if rd(SCU_PLL_PLLCON2) & PLLCON2_PINSEL != 0 {
        F_OFI_HZ
    } else {
        match F_OSC_HZ.load(Ordering::Relaxed) {
            0 => return None,
            f => f,
        }
    };
    let con1 = rd(SCU_PLL_PLLCON1);
    let k1 = (con1 & 0x7F) + 1;
    let n = ((con1 >> 8) & 0x7F) + 1;
    let k2 = ((con1 >> 16) & 0x7F) + 1;
    let p = ((con1 >> 24) & 0xF) + 1;
    if rd(SCU_PLL_PLLSTAT) & PLLSTAT_VCOBYST != 0 {
        Some(f_in / k1) // Prescaler-Modus (VCO umgangen)
    } else {
        Some((f_in as u64 * n as u64 / (p as u64 * k2 as u64)) as u32)
    }
}

/// Systemtakt fSYS. `None`, wenn die PLL gewählt ist, ihr Quarz-Eingang aber nicht über
/// [`init_144mhz`] eingestellt wurde.
pub fn sys_clock_hz() -> Option<u32> {
    let sysclkcr = rd(SCU_CLK_SYSCLKCR);
    let src = if sysclkcr & SYSCLKCR_SYSSEL_PLL != 0 {
        pll_clock_hz()?
    } else {
        F_OFI_HZ
    };
    Some(src / ((sysclkcr & 0xFF) + 1))
}

/// CPU-Takt: `fCPU = fSYS / (CPUDIV + 1)`
pub fn cpu_clock_hz() -> Option<u32> {
    let cpudiv = (rd(SCU_CLK_CPUCLKCR) & 1) + 1;
    sys_clock_hz().map(|f| f / cpudiv)
}

/// Peripherie-Takt fPERIPH (u. a. für USIC/UART): `fCPU / (PBDIV + 1)`
pub fn peripheral_clock_hz() -> Option<u32> {
    let pbdiv = (rd(SCU_CLK_PBCLKCR) & 1) + 1;
    cpu_clock_hz().map(|f| f / pbdiv)
}

/// Takt der CCU-Module: `fCCU = fSYS / (CCUDIV + 1)`
pub fn ccu_clock_hz() -> Option<u32> {
    let ccudiv = (rd(SCU_CLK_CCUCLKCR) & 1) + 1;
    sys_clock_hz().map(|f| f / ccudiv)
}

/// Alle Takte auf einmal.
pub fn current() -> Option<Clocks> {
    Some(Clocks {
        f_sys_hz: sys_clock_hz()?,
        f_cpu_hz: cpu_clock_hz()?,
        f_peripheral_hz: peripheral_clock_hz()?,
        f_ccu_hz: ccu_clock_hz()?,
    })
}
