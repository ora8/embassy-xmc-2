//! Gemeinsame Teile der USIC-Kanäle (UART, SPI): Takt/Reset und Baudraten-Teiler.

use core::ptr::{read_volatile, write_volatile};

// SCU: Takt-Gating und Reset. USIC0 hängt an den Registern 0, USIC1/USIC2 an den Registern 1.
const SCU_CLK_CGATSTAT0: usize = 0x5000_4640;
const SCU_CLK_CGATCLR0: usize = 0x5000_4648;
const SCU_CLK_CGATSTAT1: usize = 0x5000_464C;
const SCU_CLK_CGATCLR1: usize = 0x5000_4654;
const SCU_RESET_PRSTAT0: usize = 0x5000_440C;
const SCU_RESET_PRCLR0: usize = 0x5000_4414;
const SCU_RESET_PRSTAT1: usize = 0x5000_4418;
const SCU_RESET_PRCLR1: usize = 0x5000_4420;

/// USIC-Module des XMC4700
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum Module {
    Usic0,
    #[allow(dead_code)] // noch kein Treiber auf USIC1
    Usic1,
    Usic2,
}

#[inline(always)]
pub(crate) fn rd(addr: usize) -> u32 {
    // Nur mit festen MMIO-Adressen aus diesem Crate aufgerufen.
    unsafe { read_volatile(addr as *const u32) }
}

#[inline(always)]
pub(crate) fn wr(addr: usize, value: u32) {
    unsafe { write_volatile(addr as *mut u32, value) }
}

fn wait_clear(addr: usize, mask: u32) {
    for _ in 0..10_000 {
        if rd(addr) & mask == 0 {
            return;
        }
    }
}

/// Modul takten und aus dem Reset holen.
pub(crate) fn enable_module(module: Module) {
    let (cgat_clr, cgat_stat, pr_clr, pr_stat, mask) = match module {
        Module::Usic0 => (
            SCU_CLK_CGATCLR0,
            SCU_CLK_CGATSTAT0,
            SCU_RESET_PRCLR0,
            SCU_RESET_PRSTAT0,
            1 << 11,
        ),
        Module::Usic1 => (
            SCU_CLK_CGATCLR1,
            SCU_CLK_CGATSTAT1,
            SCU_RESET_PRCLR1,
            SCU_RESET_PRSTAT1,
            1 << 7,
        ),
        Module::Usic2 => (
            SCU_CLK_CGATCLR1,
            SCU_CLK_CGATSTAT1,
            SCU_RESET_PRCLR1,
            SCU_RESET_PRSTAT1,
            1 << 8,
        ),
    };
    wr(cgat_clr, mask);
    wait_clear(cgat_stat, mask);
    wr(pr_clr, mask);
    wait_clear(pr_stat, mask);
}

/// Ergebnis der Teiler-Berechnung
#[derive(Clone, Copy, Debug)]
pub(crate) struct Divider {
    /// FDR.STEP (Fractional Divider)
    pub step: u32,
    /// PDIV + 1 (Wert für BRG.PDIV ist `pdiv - 1`)
    pub pdiv: u32,
    /// Tatsächliche Rate in Bit/s
    pub actual: u32,
}

/// Teiler für `f = fPERIPH * STEP / 1024 / (PDIV + 1) / oversampling`.
///
/// Sucht die Einstellung mit der kleinsten Abweichung. Mit `not_above` werden nur Raten
/// zugelassen, die höchstens `rate` betragen (für SPI: das Gerät darf nicht übertaktet werden).
pub(crate) fn calc_divider(
    f_periph: u32,
    rate: u32,
    oversampling: u32,
    not_above: bool,
) -> Option<Divider> {
    if rate == 0 || oversampling == 0 {
        return None;
    }
    let f = f_periph as u64;
    let r = rate as u64;
    let os = oversampling as u64;
    let den = 1024 * os * r;
    let mut best: Option<(Divider, u64)> = None;
    for step in 1..=1023u64 {
        let p_round = (f * step + den / 2) / den;
        // Beim Verbot zu hoher Raten aufrunden, sonst runden
        let p_ceil = (f * step).div_ceil(den);
        let p = if not_above { p_ceil } else { p_round };
        if !(1..=1024).contains(&p) {
            continue;
        }
        let actual = f * step / (1024 * os * p);
        if actual == 0 || (not_above && actual > r) {
            continue;
        }
        let err = actual.abs_diff(r);
        if best.as_ref().map_or(true, |&(_, e)| err < e) {
            best = Some((
                Divider {
                    step: step as u32,
                    pdiv: p as u32,
                    actual: actual as u32,
                },
                err,
            ));
        }
    }
    best.map(|(d, _)| d)
}
