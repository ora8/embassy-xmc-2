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

/// Nur falls `clocks` den Takt nicht bestimmen kann (sollte nach `init_144mhz` nie passieren)
const FALLBACK_CCU_HZ: u32 = 144_000_000;
const FALLBACK_PERIPH_HZ: u32 = 144_000_000;

/// Signal von led1_task an counter_task (trägt den Zählerstand)
static BLINK_EVENT: Signal<CriticalSectionRawMutex, u32> = Signal::new();

#[embassy_executor::task]
async fn led1_task() {
    // Ticker statt Timer::after: kein Drift, die Periode bleibt exakt
    let mut ticker = Ticker::every(Duration::from_millis(500));
    let mut n: u32 = 0;
    let led1_pin = unsafe { Pin::new_unchecked(5, 8) }; // Beispiel für LED Pin auf Port 5, Pin 8
    let mut led1 = Output::new(led1_pin, Level::Low);
    let led2_pin = unsafe { Pin::new_unchecked(5, 9) }; // Beispiel für LED Pin auf Port 5, Pin 8
    let mut led2 = Output::new(led2_pin, Level::High);
    loop {
        led1.toggle();
        led2.toggle();
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
    embassy_xmc::init();

    // Zuerst der Takt: 144 MHz aus dem 12-MHz-Quarz. UART und Zeitgeber rechnen ihre
    // Teiler aus dem Takt, der beim Aufruf gerade eingestellt ist.
    let pll = clocks::init_144mhz();

    // UART auf dem virtuellen COM-Port des J-Link (USIC0 CH0, TX P1.5 / RX P1.4), 115200 8N1
    let f_periph = clocks::peripheral_clock_hz().unwrap_or(FALLBACK_PERIPH_HZ);
    let serial = Uart::new_debug(f_periph, uart::Config::default()).unwrap();
    uart::init_print(serial);
    println!();
    match pll {
        Ok(c) => println!(
            "embassy-xmc: XMC4700 gestartet, fSYS = {} Hz, fPERIPH = {} Hz, fCCU = {} Hz",
            c.f_sys_hz, c.f_peripheral_hz, c.f_ccu_hz
        ),
        Err(e) => println!("embassy-xmc: PLL-Fehler {:?}, laufe auf fOFI", e),
    }

    // CCU40 als Zeitbasis starten: 144 MHz / 2^4 = 9 MHz (Feature `tick-hz-9_000_000`)
    time_driver::init(clocks::ccu_clock_hz().unwrap_or(FALLBACK_CCU_HZ));
    spawner.spawn(led1_task().unwrap());
    spawner.spawn(counter_task().unwrap());
}
