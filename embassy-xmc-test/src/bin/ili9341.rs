//! ILI9341-Display (240×320, SPI) mit `mipidsi` und `embedded-graphics`.
//!
//!     cargo run --release --bin ili9341
//!
//! # Verdrahtung (Arduino-Header des Relax Kits)
//!
//! | Display | Arduino | XMC4700 |
//! |---------|---------|---------|
//! | SCK     | D13     | P3.9    |
//! | MOSI/SDI| D11     | P3.8    |
//! | MISO/SDO| D12     | P3.7 (optional) |
//! | CS      | D10     | P3.10   |
//! | DC/RS   | D9      | P1.11   |
//! | RESET   | D8      | P1.10   |
//! | VCC/LED | 3V3     |         |
//! | GND     | GND     |         |
//!
//! Andere Pins für CS/DC/RESET: Konstanten unten anpassen.
//! Sind Rot und Blau vertauscht, `ColorOrder::Bgr` durch `ColorOrder::Rgb` ersetzen.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Ticker};
use embassy_xmc::gpio::{Level, Output, Pin};
use embassy_xmc::spi::{self, Spi};
use embassy_xmc::uart::{self, Uart};
use embassy_xmc::{clocks, println, time_drivers};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle, Triangle};
use embedded_graphics::text::Text;
use embedded_hal_bus::spi::ExclusiveDevice;
use mipidsi::interface::SpiInterface;
use mipidsi::models::ILI9341Rgb565;
use mipidsi::options::{ColorOrder, Orientation, Rotation};
use mipidsi::Builder;
use panic_halt as _;

/// (Port, Pin) der Steuerleitungen
const PIN_CS: (u8, u8) = (3, 10); // D10
const PIN_DC: (u8, u8) = (1, 11); // D9
const PIN_RST: (u8, u8) = (1, 10); // D8

/// SPI-Takt. Das Datenblatt nennt 10 MHz für Schreibzugriffe; die meisten Module
/// laufen problemlos mit 20..40 MHz. Bei Bildfehlern verringern.
const SPI_HZ: u32 = 20_000_000;

fn out(p: (u8, u8), level: Level) -> Output {
    // Die Pins sind nur hier in Gebrauch.
    Output::new(unsafe { Pin::new_unchecked(p.0, p.1) }, level)
}

/// Kleiner Text-Puffer für `write!` ohne Heap
struct Buf<const N: usize> {
    data: [u8; N],
    len: usize,
}

impl<const N: usize> Buf<N> {
    fn new() -> Self {
        Self { data: [0; N], len: 0 }
    }
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.data[..self.len]).unwrap_or("")
    }
}

impl<const N: usize> core::fmt::Write for Buf<N> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let n = s.len().min(N - self.len);
        self.data[self.len..self.len + n].copy_from_slice(&s.as_bytes()[..n]);
        self.len += n;
        Ok(())
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    embassy_xmc::init();
    let clk = clocks::init_144mhz().unwrap();
    uart::init_print(Uart::new_debug(clk.f_peripheral_hz, uart::Config::default()).unwrap());
    time_drivers::init(clk.f_ccu_hz);
    println!();
    println!("ILI9341-Demo, fSYS = {} Hz", clk.f_sys_hz);

    // SPI-Bus + Chip-Select = SpiDevice
    let spi_config = spi::Config {
        frequency: SPI_HZ,
        ..Default::default()
    };
    let spi = Spi::new_arduino(clk.f_peripheral_hz, spi_config).unwrap();
    println!("SPI-Takt {} Hz", spi.frequency());
    let cs = out(PIN_CS, Level::High);
    let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();

    // Display-Schnittstelle (DC-Pin + Sammelpuffer) und Treiber
    let dc = out(PIN_DC, Level::Low);
    let rst = out(PIN_RST, Level::High);
    let mut buffer = [0u8; 512];
    let di = SpiInterface::new(spi_dev, dc, &mut buffer);
    let mut display = Builder::new(ILI9341Rgb565, di)
        .reset_pin(rst)
        .color_order(ColorOrder::Bgr)
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
        .init(&mut Delay)
        .unwrap();
    println!("Display initialisiert");

    // Statisches Bild
    let t0 = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let t_clear = t0.elapsed().as_millis();

    let size = display.bounding_box().size;
    let title = MonoTextStyle::new(&FONT_10X20, Rgb565::WHITE);
    Text::new("Hallo XMC4700 + Embassy!", Point::new(10, 25), title)
        .draw(&mut display)
        .unwrap();

    let colors = [
        Rgb565::RED,
        Rgb565::GREEN,
        Rgb565::BLUE,
        Rgb565::YELLOW,
        Rgb565::CYAN,
        Rgb565::MAGENTA,
    ];
    let w = size.width / colors.len() as u32;
    for (i, &c) in colors.iter().enumerate() {
        Rectangle::new(Point::new(i as i32 * w as i32, 40), Size::new(w, 30))
            .into_styled(PrimitiveStyle::with_fill(c))
            .draw(&mut display)
            .unwrap();
    }
    Circle::new(Point::new(30, 90), 80)
        .into_styled(PrimitiveStyle::with_stroke(Rgb565::RED, 3))
        .draw(&mut display)
        .unwrap();
    Triangle::new(Point::new(150, 170), Point::new(210, 90), Point::new(270, 170))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::GREEN))
        .draw(&mut display)
        .unwrap();
    println!("Bild gezeichnet (clear: {} ms)", t_clear);

    // Laufende Anzeige: Uhrzeit seit Start, jede Sekunde neu
    let small = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();
    let info = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
    let mut line = Buf::<48>::new();
    let _ = write!(line, "SPI {} MHz, clear {} ms", SPI_HZ / 1_000_000, t_clear);
    Text::new(line.as_str(), Point::new(10, size.height as i32 - 8), info)
        .draw(&mut display)
        .unwrap();

    let mut ticker = Ticker::every(Duration::from_secs(1));
    loop {
        let s = Instant::now().as_secs();
        let mut line = Buf::<32>::new();
        let _ = write!(line, "Laufzeit {:02}:{:02}:{:02}", s / 3600, s / 60 % 60, s % 60);
        Text::new(line.as_str(), Point::new(10, 205), small)
            .draw(&mut display)
            .unwrap();
        ticker.next().await;
    }
}
