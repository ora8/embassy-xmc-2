//! XMC4700 clock-tree configuration.
//!
//! Do not assume a clock frequency until the board oscillator and PLL setup
//! are known. The Relax Kit's startup configuration must be verified against
//! the exact kit revision and schematic.

#[derive(Clone, Copy, Debug)]
pub struct Clocks {
    pub f_cpu_hz: u32,
    pub f_peripheral_hz: u32,
}

/// Conservative descriptor only; it does not configure hardware.
pub const UNCONFIGURED: Clocks = Clocks {
    f_cpu_hz: 0,
    f_peripheral_hz: 0,
};

// ---------------------------------------------------------------------------
// Takt aus den SCU-Registern ablesen
// ---------------------------------------------------------------------------

use core::ptr::read_volatile;

const SCU_CLK_SYSCLKCR: usize = 0x5000_460C;
const SCU_CLK_CCUCLKCR: usize = 0x5000_4620;
/// Interner 24-MHz-Backup-Takt fOFI, mit dem der Chip nach dem Reset läuft.
const F_OFI_HZ: u32 = 24_000_000;

fn read_reg(addr: usize) -> u32 {
    // Feste SCU-Adresse, siehe Konstanten oben.
    unsafe { read_volatile(addr as *const u32) }
}

/// Systemtakt fSYS, solange der 24-MHz-Backup-Takt als Quelle gewählt ist.
///
/// Gibt `None` zurück, wenn `SYSCLKCR.SYSSEL` die PLL auswählt: Deren Frequenz hängt von der
/// Board-spezifischen PLL-Konfiguration ab, die dieses Crate noch nicht vornimmt.
pub fn sys_clock_hz() -> Option<u32> {
    let sysclkcr = read_reg(SCU_CLK_SYSCLKCR);
    if sysclkcr & (1 << 16) != 0 {
        return None; // SYSSEL = fPLL
    }
    Some(F_OFI_HZ / ((sysclkcr & 0xFF) + 1))
}

/// Takt der CCU-Module: `fCCU = fSYS / (CCUDIV + 1)`. `None` wie bei [`sys_clock_hz`].
pub fn ccu_clock_hz() -> Option<u32> {
    let ccudiv = (read_reg(SCU_CLK_CCUCLKCR) & 1) + 1;
    sys_clock_hz().map(|f| f / ccudiv)
}
