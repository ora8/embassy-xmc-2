#![no_std]
#![no_main]

use panic_halt as _;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_xmc::gpio::{Level, Output, Pin};
use embassy_xmc::uart::{self, Uart};
use embassy_xmc::println;
use embassy_time::{Duration, Instant, Ticker};

use embassy_xmc::clocks;
use embassy_xmc::time_drivers as time_driver;

/// Takt der CCU-Module, falls die PLL gewählt ist (hier nicht konfiguriert, siehe `clocks`)
const FALLBACK_CCU_HZ: u32 = 24_000_000;
/// Peripherie-Takt, falls die PLL gewählt ist (ebenso)
const FALLBACK_PERIPH_HZ: u32 = 24_000_000;

/// Signal von led1_task an counter_task (trägt den Zählerstand)
static BLINK_EVENT: Signal<CriticalSectionRawMutex, u32> = Signal::new();

#[embassy_executor::task]
async fn led1_task() {
    // Ticker statt Timer::after: kein Drift, die Periode bleibt exakt
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut n: u32 = 0;
    let led_pin = unsafe { Pin::new_unchecked(5, 8) }; // Beispiel für LED Pin auf Port 5, Pin 8
    let mut led = Output::new(led_pin, Level::Low);
    loop {
        led.toggle();
        n = n.wrapping_add(1);
        BLINK_EVENT.signal(n);
        ticker.next().await;
    }
}

/// Gibt jeden Blink-Zählerstand auf dem Serial Monitor aus.
#[embassy_executor::task]
async fn counter_task() {
    loop {
        let n = BLINK_EVENT.wait().await;
        println!("Blink {} bei {} ms", n, Instant::now().as_millis());
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // UART auf dem virtuellen COM-Port des J-Link (USIC0 CH0, TX P1.5 / RX P1.4), 115200 8N1
    let f_periph = clocks::peripheral_clock_hz().unwrap_or(FALLBACK_PERIPH_HZ);
    let serial = Uart::new_debug(f_periph, uart::Config::default()).unwrap();
    uart::init_print(serial);
    println!();
    println!("embassy-xmc: XMC4700 gestartet, fPERIPH = {} Hz", f_periph);

    // CCU40 als Zeitbasis starten: 24 MHz / 2^3 = 3 MHz (Feature `tick-hz-3_000_000`)
    time_driver::init(clocks::ccu_clock_hz().unwrap_or(FALLBACK_CCU_HZ));
    spawner.spawn(led1_task().unwrap());
    spawner.spawn(counter_task().unwrap());
}
