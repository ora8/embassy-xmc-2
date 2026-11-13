//! SPI-Master (SSC-Modus des USIC), blockierend, 8 Bit, MSB zuerst.
//!
//! # Relax Kit: SPI am Arduino-Header
//!
//! Wie in Infineons XMC-for-Arduino (`XMC_SPI_0`) liegt der SPI des Arduino-Headers auf
//! **USIC2 Kanal 0**:
//!
//! | Arduino | Signal | Pin   | Funktion           |
//! |---------|--------|-------|--------------------|
//! | D13     | SCK    | P3.9  | ALT1 = U2C0.SCLKOUT |
//! | D11     | MOSI   | P3.8  | ALT1 = U2C0.DOUT0  |
//! | D12     | MISO   | P3.7  | Eingang U2C0.DX0C  |
//!
//! Chip-Select (z. B. D10 = P3.10) ist ein normaler GPIO-Ausgang. Für `embedded-hal`
//! ergibt `embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs, delay)` daraus ein
//! `SpiDevice`, das Treiber wie `mipidsi` erwarten.
//!
//! Die Konfiguration (Überabtastung 2, Rahmen, Taktphasen der SPI-Modi) entspricht
//! `XMC_SPI_CH_Init` aus Infineons XMCLib.

use core::convert::Infallible;

use embedded_hal::spi::{self as hal_spi, Mode, Phase, Polarity};

use crate::gpio::Pin;
use crate::usic::{self, rd, wr};

// ---------------------------------------------------------------------------
// Register (Adressen und Bits gegen die SVD des xmc4700-PAC geprüft)
// ---------------------------------------------------------------------------

const USIC2_CH0: usize = 0x4802_4000;

const KSCFG: usize = 0x0C;
const FDR: usize = 0x10;
const BRG: usize = 0x14;
const DX0CR: usize = 0x1C;
const SCTR: usize = 0x34;
const TCSR: usize = 0x38;
const PCR: usize = 0x3C;
const CCR: usize = 0x40;
const PSCR: usize = 0x4C;
const RBUFSR: usize = 0x50;
const RBUF: usize = 0x54;
const TBUF0: usize = 0x80;

const KSCFG_MODEN: u32 = 1 << 0;
const KSCFG_BPMODEN: u32 = 1 << 1;
const FDR_DM_FRACTIONAL: u32 = 0b10 << 14;
/// Überabtastung im SSC-Modus: 2 Zeitquanten pro Bit (BRG.DCTQ = 1)
const OVERSAMPLING: u32 = 2;
const SCTR_SDIR_MSB: u32 = 1 << 0;
const SCTR_PDL: u32 = 1 << 1;
const SCTR_TRM: u32 = 0b01 << 8;
/// Rahmenlänge 64 = unbegrenzt (Rahmenende über FEM, wie in der XMCLib)
const SCTR_FLE_UNLIMITED: u32 = 0x3F << 16;
const SCTR_WLE_8: u32 = 7 << 24;
const TCSR_TDV: u32 = 1 << 7;
const TCSR_HPCMD: u32 = 1 << 4;
const TCSR_TDSSM: u32 = 1 << 8;
const TCSR_TDEN: u32 = 0b01 << 10;
const PCR_MSLSEN: u32 = 1 << 0;
const PCR_SELCTR: u32 = 1 << 1;
const PCR_SELINV: u32 = 1 << 2;
const PCR_FEM: u32 = 1 << 3;
const CCR_MODE_SPI: u32 = 1;
const PSR_ALL: u32 = 0x1_FFFF;
const RBUFSR_RDV: u32 = (1 << 13) | (1 << 14);
/// DX0CR.DSEL: DX0C = P3.7, DX0G = eigener DOUT0 (interne Rückschleife)
const DX0_DSEL_P3_7: u32 = 2;
const DX0_DSEL_LOOPBACK: u32 = 6;
const DX0CR_INSW: u32 = 1 << 4;
/// BRG.SCLKCFG (Bits 31:30): Ruhepegel und Verzögerung des Schiebetakts
const BRG_SCLKCFG_SHIFT: u32 = 30;

// ---------------------------------------------------------------------------
// Konfiguration
// ---------------------------------------------------------------------------

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    /// Höchste Taktfrequenz in Hz; eingestellt wird die nächste erreichbare darunter.
    pub frequency: u32,
    /// SPI-Modus (Taktpolarität und -phase)
    pub mode: Mode,
    /// MOSI intern auf den Empfänger zurückführen (Selbsttest ohne Verdrahtung).
    /// Die Pins werden trotzdem angesteuert.
    pub internal_loopback: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            frequency: 20_000_000,
            mode: hal_spi::MODE_0,
            internal_loopback: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    /// Frequenz mit dem Peripherie-Takt nicht erreichbar (höchstens fPERIPH / 2)
    InvalidFrequency,
}

/// SCLKCFG für die vier SPI-Modi (Zuordnung wie in XMC-for-Arduino)
fn sclkcfg(mode: Mode) -> u32 {
    match (mode.polarity, mode.phase) {
        (Polarity::IdleLow, Phase::CaptureOnFirstTransition) => 0b10, // Modus 0
        (Polarity::IdleLow, Phase::CaptureOnSecondTransition) => 0b00, // Modus 1
        (Polarity::IdleHigh, Phase::CaptureOnFirstTransition) => 0b11, // Modus 2
        (Polarity::IdleHigh, Phase::CaptureOnSecondTransition) => 0b01, // Modus 3
    }
}

// ---------------------------------------------------------------------------
// Treiber
// ---------------------------------------------------------------------------

/// SPI-Master auf USIC2 Kanal 0
pub struct Spi {
    base: usize,
    frequency: u32,
}

impl Spi {
    /// SPI am Arduino-Header des Relax Kits: SCK = P3.9, MOSI = P3.8, MISO = P3.7.
    ///
    /// `f_periph_hz` ist der Peripherie-Takt (siehe [`crate::clocks::peripheral_clock_hz`]).
    pub fn new_arduino(f_periph_hz: u32, config: Config) -> Result<Self, Error> {
        let div = usic::calc_divider(f_periph_hz, config.frequency, OVERSAMPLING, true)
            .ok_or(Error::InvalidFrequency)?;
        let base = USIC2_CH0;
        crate::init();

        // 1. USIC2 takten, Kanal einschalten und für die Konfiguration anhalten
        usic::enable_module(usic::Module::Usic2);
        wr(base + KSCFG, KSCFG_MODEN | KSCFG_BPMODEN);
        while rd(base + KSCFG) & KSCFG_MODEN == 0 {}
        wr(base + CCR, 0);

        // 2. Takt: fSCLK = fPERIPH * STEP / 1024 / (PDIV + 1) / 2, Modus über SCLKCFG
        wr(base + FDR, FDR_DM_FRACTIONAL | div.step);
        wr(
            base + BRG,
            (sclkcfg(config.mode) << BRG_SCLKCFG_SHIFT)
                | ((div.pdiv - 1) << 16)
                | ((OVERSAMPLING - 1) << 10),
        );

        // 3. Rahmen: 8-Bit-Wörter, MSB zuerst, Rahmenende wenn TBUF leer läuft
        wr(
            base + SCTR,
            SCTR_SDIR_MSB | SCTR_PDL | SCTR_TRM | SCTR_FLE_UNLIMITED | SCTR_WLE_8,
        );
        wr(base + TCSR, TCSR_HPCMD | TCSR_TDEN | TCSR_TDSSM);
        // Master mit internem Slave-Select-Signal (MSLS); es ist auf keinen Pin geführt,
        // Chip-Select übernimmt ein GPIO.
        wr(base + PCR, PCR_MSLSEN | PCR_SELCTR | PCR_SELINV | PCR_FEM);

        // 4. Dateneingang
        let dsel = if config.internal_loopback {
            DX0_DSEL_LOOPBACK
        } else {
            DX0_DSEL_P3_7
        };
        // INSW: Eingang direkt auf die Schiebeeinheit. Ohne das Bit liest der Empfänger im
        // SSC-Modus nur Nullen (so auch `XMC_SPI_CH_SetInputSource` in der XMCLib).
        wr(base + DX0CR, DX0CR_INSW | dsel);

        // 5. Status löschen, SPI-Modus starten
        wr(base + PSCR, PSR_ALL);
        wr(base + CCR, CCR_MODE_SPI);

        // 6. Pins umschalten
        unsafe {
            Pin::new_unchecked(3, 7).set_input_pull_up();
            Pin::new_unchecked(3, 8).set_alt_output(1);
            Pin::new_unchecked(3, 9).set_alt_output(1);
        }

        Ok(Self {
            base,
            frequency: div.actual,
        })
    }

    /// Tatsächlich eingestellte Taktfrequenz in Hz
    pub fn frequency(&self) -> u32 {
        self.frequency
    }

    /// Kern aller Übertragungen (Vollduplex).
    ///
    /// Es sind höchstens zwei Bytes unterwegs (Sendepuffer + Schieberegister). So hält der
    /// Bus ohne Pause durch, und der doppelte Empfangspuffer (RBUF0/RBUF1) läuft nie über.
    /// `out(i)` liefert das i-te Sendebyte, `inp(i, b)` nimmt das i-te Empfangsbyte.
    #[inline(always)]
    fn run(&mut self, len: usize, out: impl Fn(usize) -> u8, mut inp: impl FnMut(usize, u8)) {
        let base = self.base;
        // Reste aus früheren Übertragungen verwerfen
        while rd(base + RBUFSR) & RBUFSR_RDV != 0 {
            let _ = rd(base + RBUF);
        }
        let mut sent = 0;
        let mut received = 0;
        while received < len {
            if sent < len && sent - received < 2 && rd(base + TCSR) & TCSR_TDV == 0 {
                wr(base + TBUF0, out(sent) as u32);
                sent += 1;
            }
            if rd(base + RBUFSR) & RBUFSR_RDV != 0 {
                inp(received, rd(base + RBUF) as u8);
                received += 1;
            }
        }
    }

    /// Bytes senden, Empfangenes verwerfen. Kehrt erst zurück, wenn alles draußen ist.
    pub fn write(&mut self, words: &[u8]) {
        self.run(words.len(), |i| words[i], |_, _| {});
    }

    /// `read.len()` Bytes lesen; dabei werden 0x00 gesendet.
    pub fn read(&mut self, words: &mut [u8]) {
        self.run(words.len(), |_| 0, |i, b| words[i] = b);
    }

    /// Gleichzeitig senden und empfangen; ungleich lange Puffer werden mit 0x00 aufgefüllt
    /// bzw. abgeschnitten (wie in `embedded_hal::spi::SpiBus::transfer` verlangt).
    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) {
        let len = read.len().max(write.len());
        self.run(
            len,
            |i| write.get(i).copied().unwrap_or(0),
            |i, b| {
                if let Some(r) = read.get_mut(i) {
                    *r = b;
                }
            },
        );
    }

    /// Puffer senden und mit den empfangenen Bytes überschreiben.
    pub fn transfer_in_place(&mut self, words: &mut [u8]) {
        let len = words.len();
        let ptr = words.as_mut_ptr();
        // `out(i)` liest Byte i, bevor `inp(i, _)` es überschreibt (Byte i wird immer vor
        // seiner Antwort gesendet), daher genügen rohe Zugriffe ohne Kopie.
        self.run(
            len,
            |i| unsafe { *ptr.add(i) },
            |i, b| unsafe { *ptr.add(i) = b },
        );
    }
}

// ---------------------------------------------------------------------------
// embedded-hal 1.0
// ---------------------------------------------------------------------------

impl hal_spi::ErrorType for Spi {
    type Error = Infallible;
}

impl hal_spi::SpiBus<u8> for Spi {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        Spi::read(self, words);
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Infallible> {
        Spi::write(self, words);
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Infallible> {
        Spi::transfer(self, read, write);
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Infallible> {
        Spi::transfer_in_place(self, words);
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Infallible> {
        // Alle Methoden warten, bis das letzte Byte empfangen (= vollständig gesendet) ist.
        Ok(())
    }
}
