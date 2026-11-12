#![no_std]
#![no_main]

use panic_halt as _;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_xmc::gpio::{Level, Output, Pin};
use embassy_time::{Duration, Ticker};

use embassy_xmc::clocks;
use embassy_xmc::time_drivers as time_driver;

/// Takt der CCU-Module, falls die PLL gewählt ist (hier nicht konfiguriert, siehe `clocks`)
const FALLBACK_CCU_HZ: u32 = 24_000_000;

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

/// Minimal executor bring-up.
///
/// This proves the upstream Embassy executor is selected and entered.
/// Replace `pending()` with spawned application tasks once GPIO and the
/// XMC hardware time driver are implemented.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // CCU40 als Zeitbasis starten: 24 MHz / 2^3 = 3 MHz (Feature `tick-hz-3_000_000`)
    time_driver::init(clocks::ccu_clock_hz().unwrap_or(FALLBACK_CCU_HZ));
    spawner.spawn(led1_task().unwrap());
}
