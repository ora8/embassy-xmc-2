//! SPI-Selbsttest ohne Verdrahtung: MOSI wird im USIC intern auf den Empfänger
//! zurückgeführt. Ergebnis auf dem Serial Monitor (115200 Baud).
//!
//!     cargo run --release --bin spi_loopback

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use embassy_xmc::spi::{self, Spi};
use embassy_xmc::uart::{self, Uart};
use embassy_xmc::{clocks, println, time_drivers};
use embedded_hal::spi::SpiBus;
use panic_halt as _;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    embassy_xmc::init();
    let clk = clocks::init_144mhz().unwrap();
    uart::init_print(Uart::new_debug(clk.f_peripheral_hz, uart::Config::default()).unwrap());
    time_drivers::init(clk.f_ccu_hz);
    println!();
    println!("SPI-Selbsttest (interne Rückschleife), fPERIPH = {} Hz", clk.f_peripheral_hz);

    {
        let cfg = spi::Config { frequency: 1_000_000, ..Default::default() };
        let mut spi = Spi::new_arduino(clk.f_peripheral_hz, cfg).unwrap();
        let mut rx = [0u8; 8];
        SpiBus::transfer(&mut spi, &mut rx, &[0x12, 0x34, 0x56, 0x78, 0, 0, 0, 0]).unwrap();
        println!("ohne Rückschleife (P3.7 offen, Pull-up): {:02x?}", rx);
    }
    for &mode in &[
        embedded_hal::spi::MODE_0,
        embedded_hal::spi::MODE_1,
        embedded_hal::spi::MODE_2,
        embedded_hal::spi::MODE_3,
    ] {
        for &freq in &[1_000_000u32, 10_000_000, 20_000_000, 40_000_000] {
            let config = spi::Config {
                frequency: freq,
                mode,
                internal_loopback: true,
            };
            let mut spi = Spi::new_arduino(clk.f_peripheral_hz, config).unwrap();

            let mut tx = [0u8; 256];
            for (i, b) in tx.iter_mut().enumerate() {
                *b = (i as u8).wrapping_mul(37) ^ 0xA5;
            }
            let mut rx = [0u8; 256];
            SpiBus::transfer(&mut spi, &mut rx, &tx).unwrap();
            let ok_transfer = rx == tx;
            if !ok_transfer {
                println!("  gesendet  {:02x?}", &tx[..8]);
                println!("  empfangen {:02x?}", &rx[..8]);
            }

            let mut buf = tx;
            SpiBus::transfer_in_place(&mut spi, &mut buf).unwrap();
            let ok_in_place = buf == tx;

            // Durchsatz: 16 KiB schreiben
            let block = [0x55u8; 1024];
            let t0 = Instant::now();
            for _ in 0..16 {
                SpiBus::write(&mut spi, &block).unwrap();
            }
            let us = t0.elapsed().as_micros().max(1);
            let kib_s = 16 * 1024 * 1_000_000 / us / 1024;

            println!(
                "Modus {:?}/{:?}, Soll {} Hz, Ist {} Hz: transfer {}, in_place {}, {} KiB/s",
                mode.polarity,
                mode.phase,
                freq,
                spi.frequency(),
                if ok_transfer { "OK" } else { "FEHLER" },
                if ok_in_place { "OK" } else { "FEHLER" },
                kib_s
            );
        }
    }
    println!("Fertig.");
    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
