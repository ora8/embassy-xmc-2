//! 16-Bit-Parallelbus im 8080-Stil (Intel-Schreibzyklus) für Displays wie das ILI9341.
//!
//! Die Datenleitungen DB0..DB15 liegen auf **P0.0..P0.15**. Ein Datenwort wird mit einem
//! einzigen Schreibzugriff auf `PORT0.OMR` ausgegeben (Setzen und Löschen gleichzeitig),
//! danach erzeugt WR einen Low-Puls; das Display übernimmt die Daten mit der steigenden
//! Flanke.
//!
//! ```text
//!        ┌───────── Daten gültig ─────────┐
//! DB  ───┘                                └───
//! WR  ──────────┐            ┌────────────────
//!               └── ≥ 15 ns ─┘ ↑ Übernahme
//! ```
//!
//! Zeitbedingungen des ILI9341 beim Schreiben: Zyklus ≥ 66 ns, WR low/high ≥ 15 ns,
//! Setup/Hold ≥ 10 ns. Lesezugriffe auf den Port zwischen WR-low und WR-high warten, bis
//! der Schreibpuffer der CPU geleert ist; so ist der Puls auch bei gepufferten
//! Schreibzugriffen sicher breit genug.
//!
//! Gemessen bei 144 MHz: Mit nur einem Lesezugriff dauert ein Wort 39 ns, also zu kurz
//! (ca. 5–6 Takte, ein Lesezugriff ≈ 2–3 Takte). Mit `LOW_READS` = 3 und
//! `HIGH_READS` = 2 ergibt die Abschätzung ca. 95–120 ns pro Wort mit Reserve;
//! am Board noch nachzumessen (die Demo gibt „ns pro Wort“ aus).
//!
//! CS und RD werden nur gehalten (CS dauerhaft aktiv, RD inaktiv): Der Bus gehört einem
//! einzigen Display, gelesen wird nicht.
//!
//! Mit dem Feature `mipidsi` implementiert [`Parallel16`] `mipidsi::interface::Interface`.

use core::ptr::{read_volatile, write_volatile};

use crate::gpio::{DriveStrength, Level, Output, Pin, IOCR_PC_OUTPUT_PUSH_PULL};

const PORT0_OUT: usize = 0x4802_8000;
const PORT0_OMR: usize = 0x4802_8004;

/// Lesezugriffe auf den Port während WR = Low (Pulsbreite ≥ 15 ns)
const LOW_READS: u32 = 3;
/// Lesezugriffe nach WR = High (High-Zeit und Zyklus ≥ 66 ns)
const HIGH_READS: u32 = 2;

#[inline(always)]
fn port0_write(word: u16) {
    let w = word as u32;
    // Bits 0..15 setzen, Bits 16..31 löschen; ein Zugriff schaltet alle 16 Pins.
    unsafe { write_volatile(PORT0_OMR as *mut u32, w | ((!w & 0xFFFF) << 16)) }
}

/// 16-Bit-8080-Bus auf Port 0 mit den Steuerleitungen WR, DC (RS), CS und RD.
pub struct Parallel16 {
    wr_omr: *mut u32,
    wr_low: u32,
    wr_high: u32,
    dc: Output,
    _cs: Option<Output>,
    _rd: Option<Output>,
}

impl Parallel16 {
    /// Bus einrichten. `wr`, `dc`, `cs`, `rd` sind beliebige Ausgänge; sie werden auf
    /// WR = High, DC = High, CS = Low (aktiv), RD = High gesetzt.
    ///
    /// Alle 16 Pins von Port 0 werden zu Push-Pull-Ausgängen.
    pub fn new_port0(
        mut wr: Output,
        mut dc: Output,
        cs: Option<Output>,
        rd: Option<Output>,
    ) -> Self {
        port0_write(0);
        for n in 0..16 {
            // Port 0 hat 16 Pins; die Konfiguration ist pinweise.
            let pin = unsafe { Pin::new_unchecked(0, n) };
            pin.set_pc(IOCR_PC_OUTPUT_PUSH_PULL);
            pin.set_drive(DriveStrength::StrongMediumEdge);
        }
        wr.set_level(Level::High);
        wr.set_drive_strength(DriveStrength::StrongMediumEdge);
        dc.set_level(Level::High);
        let cs = cs.map(|mut c| {
            c.set_level(Level::Low);
            c
        });
        let rd = rd.map(|mut r| {
            r.set_level(Level::High);
            r
        });
        let bit = wr.pin().number() as u32;
        Self {
            wr_omr: wr.pin().omr_adr(),
            wr_low: 1 << (bit + 16),
            wr_high: 1 << bit,
            dc,
            _cs: cs,
            _rd: rd,
        }
    }

    /// WR-Puls: fällt, wartet bis die Flanke am Pin ist, steigt.
    #[inline(always)]
    fn strobe(&mut self) {
        unsafe {
            write_volatile(self.wr_omr, self.wr_low);
            // Ein Lesezugriff ist erst fertig, wenn WR-low wirklich geschrieben ist (Device-
            // Speicher, Programmreihenfolge), und kostet selbst Buszeit. Anders als NOPs, die
            // die CPU verwerfen darf, verlängert er den Puls also verlässlich.
            for _ in 0..LOW_READS {
                let _ = read_volatile(PORT0_OUT as *const u32);
            }
            write_volatile(self.wr_omr, self.wr_high);
            for _ in 0..HIGH_READS {
                let _ = read_volatile(PORT0_OUT as *const u32);
            }
        }
    }

    /// Ein Wort schreiben.
    #[inline(always)]
    pub fn write_word(&mut self, word: u16) {
        port0_write(word);
        self.strobe();
    }

    /// Kommando (DC = Low) mit Parameter-Bytes (DC = High) senden.
    pub fn write_command(&mut self, cmd: u8, args: &[u8]) {
        self.dc.set_level(Level::Low);
        self.write_word(cmd as u16);
        self.dc.set_level(Level::High);
        for &a in args {
            self.write_word(a as u16);
        }
    }

    /// Datenwörter (DC = High) senden, z. B. RGB565-Pixel.
    pub fn write_data(&mut self, words: &[u16]) {
        for &w in words {
            self.write_word(w);
        }
    }

    /// Dasselbe Datenwort `count`-mal senden: Daten einmal anlegen, dann nur WR pulsen.
    pub fn write_repeated(&mut self, word: u16, count: u32) {
        port0_write(word);
        for _ in 0..count {
            self.strobe();
        }
    }
}

#[cfg(feature = "mipidsi")]
mod mipidsi_impl {
    use super::Parallel16;
    use core::convert::Infallible;
    use mipidsi::interface::{Interface, InterfaceKind};

    impl Interface for Parallel16 {
        type Word = u16;
        type Error = Infallible;

        const KIND: InterfaceKind = InterfaceKind::Parallel16Bit;

        fn send_command(&mut self, command: u8, args: &[u8]) -> Result<(), Infallible> {
            self.write_command(command, args);
            Ok(())
        }

        fn send_pixels<const N: usize>(
            &mut self,
            pixels: impl IntoIterator<Item = [u16; N]>,
        ) -> Result<(), Infallible> {
            for pixel in pixels {
                for w in pixel {
                    self.write_word(w);
                }
            }
            Ok(())
        }

        fn send_repeated_pixel<const N: usize>(
            &mut self,
            pixel: [u16; N],
            count: u32,
        ) -> Result<(), Infallible> {
            if N == 1 {
                self.write_repeated(pixel[0], count);
            } else {
                for _ in 0..count {
                    for w in pixel {
                        self.write_word(w);
                    }
                }
            }
            Ok(())
        }
    }
}
