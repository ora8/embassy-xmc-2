//! ILI9341-Display mit 16-Bit-Parallelbus (z. B. Modul TFT_320QDT_9341), `mipidsi` und
//! `embedded-graphics`.
//!
//!     cargo run --release --bin ili9341_parallel
//!
//! # Verdrahtung
//!
//! | Display     | XMC4700        | Relax Kit                  |
//! |-------------|----------------|----------------------------|
//! | GND         | GND            | GND                        |
//! | VCC, LEDA   | 3,3 V          | 3V3                        |
//! | F_CS        | 3,3 V          | (Flash des Moduls aus)     |
//! | RS (DC)     | P1.11          | D9                         |
//! | WR          | P1.10          | D8                         |
//! | RD          | P1.9           | D7 (oder fest an 3,3 V)    |
//! | CS          | P1.8           | D4                         |
//! | REST        | P1.0           | D2                         |
//! | DB0..DB15   | P0.0..P0.15    | X1/X2, DB13 = D15/SCL      |
//!
//! DB-Leitungen im Einzelnen: DB0 X1-10, DB1 X1-7, DB2 X1-8, DB3 X1-5, DB4 X1-6, DB5 X1-3,
//! DB6 X1-4, DB7 X2-9, DB8 X2-10, DB9 X1-12, DB10 X1-9, DB11 X1-2, DB12 X2-4,
//! DB13 D15 (SCL), DB14 X2-5, DB15 X2-6.
//!
//! Sind Rot und Blau vertauscht, `ColorOrder::Bgr` durch `ColorOrder::Rgb` ersetzen.

#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Instant, Ticker};
use embassy_xmc::gpio::{Level, Output, Pin};
use embassy_xmc::parallel::Parallel16;
use embassy_xmc::uart::{self, Uart};
use embassy_xmc::{clocks, println, time_drivers};
use embedded_graphics::mono_font::ascii::{FONT_10X20, FONT_6X10};
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{Circle, PrimitiveStyle, Rectangle, Triangle};
use embedded_graphics::text::Text;
use mipidsi::models::ILI9341Rgb565;
use mipidsi::options::{ColorOrder, Orientation, Rotation};
use mipidsi::Builder;
use panic_halt as _;

/// (Port, Pin) der Steuerleitungen
const PIN_RS: (u8, u8) = (1, 11); // D9
const PIN_WR: (u8, u8) = (1, 10); // D8
const PIN_RD: (u8, u8) = (1, 9); // D7
const PIN_CS: (u8, u8) = (1, 8); // D4
const PIN_RST: (u8, u8) = (1, 0); // D2

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
    println!("ILI9341-Demo (16 Bit parallel), fSYS = {} Hz", clk.f_sys_hz);

    // 16-Bit-Bus auf Port 0, Steuerleitungen auf Port 1
    let bus = Parallel16::new_port0(
        out(PIN_WR, Level::High),
        out(PIN_RS, Level::High),
        Some(out(PIN_CS, Level::Low)),
        Some(out(PIN_RD, Level::High)),
    );
    let rst = out(PIN_RST, Level::High);
    let mut display = Builder::new(ILI9341Rgb565, bus)
        .reset_pin(rst)
        .color_order(ColorOrder::Bgr)
        .orientation(Orientation::new().rotate(Rotation::Deg90).flip_horizontal())
        .init(&mut Delay)
        .unwrap();
    println!("Display initialisiert");

    // Statisches Bild
    let t0 = Instant::now();
    display.clear(Rgb565::BLACK).unwrap();
    let t_clear_us = t0.elapsed().as_micros();
    let t_clear = t_clear_us / 1000;

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
    // 320 × 240 Pixel = 76 800 Wörter
    println!(
        "Bild gezeichnet (clear: {} us = {} ns pro Wort)",
        t_clear_us,
        t_clear_us * 1000 / 76_800
    );

    // Laufende Anzeige: Uhrzeit seit Start, jede Sekunde neu
    let small = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();
    let info = MonoTextStyle::new(&FONT_6X10, Rgb565::CSS_GRAY);
    let mut line = Buf::<48>::new();
    let _ = write!(line, "16 Bit parallel, clear {} ms", t_clear);
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
