//! Embassy-Zeittreiber für den XMC4700 auf Basis des Moduls CCU40 (CCU4).
//!
//! # Aufbau
//!
//! Zwei Slices von CCU40 teilen sich den Modul-Prescaler und laufen mit derselben
//! Tick-Rate (`fCCU / 2^PSIV`, muss gleich `embassy_time_driver::TICK_HZ` sein):
//!
//! * **CC40 – freilaufender Zähler.** 16 Bit, Edge-aligned, Periode `0xFFFF`. Zwei
//!   Interrupts pro Runde (Period-Match bei `0xFFFF`, Compare-Match bei `0x7FFF`) zählen
//!   Halbperioden. Daraus entsteht der 64-Bit-Zeitstempel für `now()` (gleiches Verfahren
//!   wie bei `embassy-stm32` mit 16-Bit-Timern).
//! * **CC41 – One-Shot-Alarm.** Compare-/Period-Register der CCU4-Slices sind
//!   Shadow-Register: Bei laufendem Timer wirkt ein neuer Wert erst beim nächsten
//!   Period-Match, bei *gestopptem* Timer sofort. Deshalb steht der Alarm-Slice still,
//!   während er umprogrammiert wird, und läuft danach genau einmal bis zum Alarm.
//!   Liegt der Alarm weiter als 65535 Ticks entfernt, wird in Etappen gewartet.
//!
//! # Tick-Rate und Takt
//!
//! Der CCU4-Prescaler teilt nur durch Zweierpotenzen (`PSIV` = 0..=15). Der CCU-Takt muss
//! daher `TICK_HZ * 2^n` sein. Beispiele:
//!
//! | fCCU       | Feature in `embassy-time`  | PSIV |
//! |------------|----------------------------|------|
//! | 24 MHz     | `tick-hz-3_000_000`        | 3    |
//! | 24 MHz     | `tick-hz-375_000`          | 6    |
//! | 144 MHz    | `tick-hz-9_000_000`        | 4    |
//!
//! [`init`] bricht mit Panic ab, wenn kein passender Prescaler existiert.
//!
//! # Interrupts
//!
//! Benutzt `CCU40_0` (IRQ 44, Zähler) und `CCU40_1` (IRQ 45, Alarm). Die Einträge stehen
//! in der Vektortabelle in [`crate::interrupt`].

use core::cell::{Cell, RefCell};
use core::ptr::{read_volatile, write_volatile};
use core::sync::atomic::{compiler_fence, AtomicBool, Ordering};
use core::task::Waker;

use cortex_m::peripheral::NVIC;
use critical_section::{CriticalSection, Mutex};
use embassy_time_driver::{time_driver_impl, Driver};
use embassy_time_queue_utils::Queue;

use crate::interrupt::Interrupt;

/// Tick-Rate der Embassy-Zeitbasis (aus dem gewählten `tick-hz-*` Feature).
pub const TICK_HZ: u64 = embassy_time_driver::TICK_HZ;

// ---------------------------------------------------------------------------
// Register (Adressen und Bits gegen die SVD des xmc4700-PAC geprüft)
// ---------------------------------------------------------------------------

// SCU
const SCU_CLK_CLKSET: usize = 0x5000_4604; // CCUCEN = Bit 4
const SCU_CLK_CGATSTAT0: usize = 0x5000_4640; // CCU40 = Bit 2 (1 = gesperrt)
const SCU_CLK_CGATCLR0: usize = 0x5000_4648; // CCU40 = Bit 2
const SCU_RESET_PRSTAT0: usize = 0x5000_440C; // CCU40RS = Bit 2 (1 = im Reset)
const SCU_RESET_PRCLR0: usize = 0x5000_4414; // CCU40RS = Bit 2
const SCU_CLK_SYSCLKCR: usize = 0x5000_460C; // SYSSEL = Bit 16
const SCU_CLK_SLEEPCR: usize = 0x5000_4630; // SYSSEL = Bit 0, CCUCR = Bit 20
const SCU_CCUEN: u32 = 1 << 4;
const SYSCLKCR_SYSSEL: u32 = 1 << 16;
const SLEEPCR_SYSSEL: u32 = 1 << 0;
const SLEEPCR_CCUCR: u32 = 1 << 20;
const SCU_CCU40: u32 = 1 << 2;

// Modul CCU40 und seine Slices
const CCU40: usize = 0x4000_C000;
const CC40: usize = CCU40 + 0x100; // Slice 0: freilaufender Zähler
const CC41: usize = CCU40 + 0x200; // Slice 1: One-Shot-Alarm

// Modul-Register
const GIDLC: usize = 0x0C; // Idle-Modus löschen / Prescaler starten
const GCSS: usize = 0x10; // Shadow-Transfer anfordern
const GCST: usize = 0x18; // Shadow-Transfer-Status
const GIDLC_CS0I: u32 = 1 << 0;
const GIDLC_CS1I: u32 = 1 << 1;
const GIDLC_SPRB: u32 = 1 << 8;
const GCSS_S0SE: u32 = 1 << 0;
const GCSS_S1SE: u32 = 1 << 4;
const GCST_S0SS: u32 = 1 << 0;
const GCST_S1SS: u32 = 1 << 4;

// Slice-Register
const TCSET: usize = 0x0C;
const TCCLR: usize = 0x10;
const TC: usize = 0x14;
const PSC: usize = 0x24;
const PRS: usize = 0x34; // Periode (Shadow)
const CRS: usize = 0x3C; // Compare (Shadow)
const TIMER: usize = 0x70;
const INTS: usize = 0xA0;
const INTE: usize = 0xA4;
const SRS: usize = 0xA8;
const SWR: usize = 0xB0;

const TCSET_TRBS: u32 = 1 << 0; // Timer starten
const TCCLR_TRBC: u32 = 1 << 0; // Timer stoppen
const TCCLR_TCC: u32 = 1 << 1; // Timer auf 0 setzen
const TC_TSSM: u32 = 1 << 1; // Single-Shot-Modus
const INT_PM: u32 = 1 << 0; // Period-Match (aufwärts)
const INT_CMU: u32 = 1 << 2; // Compare-Match (aufwärts)
const SWR_ALL: u32 = 0xF0F; // alle Statusflags löschen

// ---------------------------------------------------------------------------
// Zeitbasis
// ---------------------------------------------------------------------------

/// Der Zähler läuft von 0 bis 0xFFFF. Alle Interrupt-Ereignisse treten bei
/// `TIMER == 0x7FFF` (Compare) bzw. `0xFFFF` (Period-Match) auf. Verschoben um `PHASE`
/// liegen sie bei `s = 0x8003` bzw. `s = 3`, sodass ein um bis zu drei Ticks "zu frühes"
/// Flag unschädlich bleibt. Der Versatz macht `now()` konstant um `PHASE` Ticks größer.
const PHASE: u16 = 4;
const COMPARE_AT: u32 = 0x7FFF;
const PERIOD: u32 = 0xFFFF;
/// Größte Wartezeit, die der 16-Bit-Alarm-Slice in einer Etappe schafft.
const MAX_ALARM_TICKS: u64 = 0xFFFF;

/// 64-Bit-Zeit aus Halbperioden-Zähler und 16-Bit-Zählerstand.
///
/// Das Halbperiodenbit wird in den Zählerstand eingeblendet: So bleibt das Ergebnis auch
/// dann korrekt, wenn der Interrupt, der `period` erhöht, noch nicht gelaufen ist
/// (Latenz darf bis zu einer halben Runde = 0x8000 Ticks betragen).
fn calc_now(period: u64, counter: u16) -> u64 {
    let s = counter.wrapping_add(PHASE);
    (period << 15) + (s ^ (((period & 1) as u16) << 15)) as u64
}

// ---------------------------------------------------------------------------
// Registerzugriff
// ---------------------------------------------------------------------------

// Die Adressen stammen ausschließlich aus den Konstanten oben (feste MMIO-Register).
#[inline(always)]
fn rd(addr: usize) -> u32 {
    unsafe { read_volatile(addr as *const u32) }
}

#[inline(always)]
fn wr(addr: usize, value: u32) {
    unsafe { write_volatile(addr as *mut u32, value) }
}

/// Wartet (begrenzt), bis alle Bits aus `mask` in `addr` gelöscht sind.
fn wait_clear(addr: usize, mask: u32) {
    for _ in 0..10_000 {
        if rd(addr) & mask == 0 {
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// Treiber
// ---------------------------------------------------------------------------

struct XmcTimeDriver {
    /// Anzahl überschrittener Halbperioden des 16-Bit-Zählers. `u64`, damit die Zeit auch
    /// bei MHz-Tick-Raten praktisch nie überläuft (ein `u32` wäre bei 3 MHz nach ~1,5 Jahren
    /// voll). Cortex-M4 hat keine 64-Bit-Atomics, daher der Zugriff per Critical Section.
    period: Mutex<Cell<u64>>,
    /// `true`, sobald die Hardware läuft. Vorher liefert `now()` 0 (siehe `Driver::now`).
    initialized: AtomicBool,
    started: AtomicBool,
    queue: Mutex<RefCell<Queue>>,
}

time_driver_impl!(static DRIVER: XmcTimeDriver = XmcTimeDriver {
    period: Mutex::new(Cell::new(0)),
    initialized: AtomicBool::new(false),
    started: AtomicBool::new(false),
    queue: Mutex::new(RefCell::new(Queue::new())),
});

impl Driver for XmcTimeDriver {
    fn now(&self) -> u64 {
        if !self.initialized.load(Ordering::Relaxed) {
            return 0;
        }
        critical_section::with(|cs| {
            // Erst `period`, dann der Zähler: Ist ein Halbperioden-Interrupt noch offen, ist
            // `period` um eins zu klein; `calc_now` gleicht das aus.
            let period = self.period.borrow(cs).get();
            compiler_fence(Ordering::Acquire);
            let counter = rd(CC40 + TIMER) as u16;
            calc_now(period, counter)
        })
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        critical_section::with(|cs| {
            let mut queue = self.queue.borrow(cs).borrow_mut();
            if queue.schedule_wake(at, waker) {
                let mut next = queue.next_expiration(self.now());
                while !self.set_alarm(cs, next) {
                    next = queue.next_expiration(self.now());
                }
            }
        })
    }
}

impl XmcTimeDriver {
    /// Alarm-Slice auf `timestamp` einstellen. `u64::MAX` schaltet den Alarm aus.
    ///
    /// Gibt `false` zurück, wenn `timestamp` schon erreicht ist. Der Aufrufer muss dann
    /// die abgelaufenen Einträge der Queue abarbeiten und es mit dem nächsten versuchen.
    fn set_alarm(&self, _cs: CriticalSection<'_>, timestamp: u64) -> bool {
        if !self.initialized.load(Ordering::Relaxed) {
            // `init` stellt den Alarm später anhand der Queue ein.
            return true;
        }

        // Alarm-Slice anhalten und Flag löschen: Shadow-Transfer wirkt nun sofort.
        wr(CC41 + TCCLR, TCCLR_TRBC | TCCLR_TCC);
        wr(CC41 + SWR, INT_PM);

        if timestamp == u64::MAX {
            return true;
        }
        let now = self.now();
        if timestamp <= now {
            return false;
        }

        // Die Etappe endet frühestens nach `ticks` Ticks (die Startlatenz kommt dazu).
        let ticks = (timestamp - now).min(MAX_ALARM_TICKS) as u32;
        wr(CC41 + PRS, ticks);
        wr(CCU40 + GCSS, GCSS_S1SE);
        wait_clear(CCU40 + GCST, GCST_S1SS);
        wr(CC41 + TCSET, TCSET_TRBS);

        // Der Zeitpunkt kann während der Einstellung verstrichen sein.
        if timestamp <= self.now() {
            wr(CC41 + TCCLR, TCCLR_TRBC | TCCLR_TCC);
            wr(CC41 + SWR, INT_PM);
            return false;
        }
        true
    }

    /// Abgelaufene Timer wecken und den Alarm für den nächsten einstellen.
    fn trigger_alarm(&self, cs: CriticalSection<'_>) {
        let mut queue = self.queue.borrow(cs).borrow_mut();
        let mut next = queue.next_expiration(self.now());
        while !self.set_alarm(cs, next) {
            next = queue.next_expiration(self.now());
        }
    }

    /// Interrupt von CC40: eine Halbperiode des Zählers ist vorbei.
    fn on_counter_irq(&self) {
        let status = rd(CC40 + INTS) & (INT_PM | INT_CMU);
        // Nur die gesehenen Flags löschen, damit kein neues Ereignis verloren geht.
        wr(CC40 + SWR, status);
        let halves = status.count_ones() as u64;
        critical_section::with(|cs| {
            let period = self.period.borrow(cs);
            period.set(period.get() + halves);
        });
    }

    /// Interrupt von CC41: Alarm-Etappe abgelaufen.
    fn on_alarm_irq(&self) {
        wr(CC41 + SWR, INT_PM);
        critical_section::with(|cs| self.trigger_alarm(cs));
    }
}

/// Interrupt-Handler `CCU40_0` (IRQ 44). Eingetragen in [`crate::interrupt`].
pub(crate) unsafe extern "C" fn ccu40_0_handler() {
    DRIVER.on_counter_irq();
}

/// Interrupt-Handler `CCU40_1` (IRQ 45). Eingetragen in [`crate::interrupt`].
pub(crate) unsafe extern "C" fn ccu40_1_handler() {
    DRIVER.on_alarm_irq();
}

/// Zeitgeber starten. Muss vor der ersten Verwendung von `embassy_time` aufgerufen werden.
///
/// `ccu_hz` ist der Takt der CCU-Module (siehe [`crate::clocks::ccu_clock_hz`]). Er muss
/// `TICK_HZ * 2^n` mit `n` in `0..=15` sein.
///
/// # Panics
///
/// Bei einem zweiten Aufruf oder wenn `ccu_hz` nicht zur Tick-Rate passt.
pub fn init(ccu_hz: u32) {
    let psiv = (0u32..16)
        .find(|&n| TICK_HZ << n == ccu_hz as u64)
        .expect("CCU-Takt ist kein 2^n-Vielfaches von TICK_HZ: passendes tick-hz-Feature wählen");
    assert!(
        !DRIVER.started.swap(true, Ordering::SeqCst),
        "time_drivers::init darf nur einmal aufgerufen werden"
    );

    // 1. Takt freigeben, Modul entsperren und aus dem Reset holen.
    wr(SCU_CLK_CLKSET, SCU_CCUEN);
    wr(SCU_CLK_CGATCLR0, SCU_CCU40);
    wait_clear(SCU_CLK_CGATSTAT0, SCU_CCU40);
    wr(SCU_RESET_PRCLR0, SCU_CCU40);
    wait_clear(SCU_RESET_PRSTAT0, SCU_CCU40);

    // 1b. CCU-Takt auch im Sleep-Modus weiterlaufen lassen. Nach dem Reset ist
    // SLEEPCR.CCUCR = 0: Sobald der Executor im Leerlauf `WFE` ausführt, bleibt der
    // CCU-Takt stehen, der Zeitgeber friert ein und kein Interrupt weckt die CPU mehr.
    // Die Taktquelle im Sleep (SLEEPCR.SYSSEL) muss der im Betrieb entsprechen,
    // sonst ändert sich die Tick-Rate im Leerlauf.
    let syssel = if rd(SCU_CLK_SYSCLKCR) & SYSCLKCR_SYSSEL != 0 {
        SLEEPCR_SYSSEL
    } else {
        0
    };
    wr(
        SCU_CLK_SLEEPCR,
        (rd(SCU_CLK_SLEEPCR) & !SLEEPCR_SYSSEL) | syssel | SLEEPCR_CCUCR,
    );

    // 2. Slices aus dem Idle-Modus holen und den Modul-Prescaler starten.
    wr(CCU40 + GIDLC, GIDLC_CS0I | GIDLC_CS1I | GIDLC_SPRB);

    // 3. CC40: freilaufender 16-Bit-Zähler. Die Slice steht noch, Shadow-Transfer wirkt sofort.
    wr(CC40 + PSC, psiv);
    wr(CC40 + TC, 0); // Edge-aligned, kontinuierlich, Compare-Modus
    wr(CC40 + PRS, PERIOD);
    wr(CC40 + CRS, COMPARE_AT);
    wr(CCU40 + GCSS, GCSS_S0SE);
    wait_clear(CCU40 + GCST, GCST_S0SS);
    wr(CC40 + SRS, 0); // Period- und Compare-Match auf Service-Request 0 (IRQ 44)
    wr(CC40 + SWR, SWR_ALL);
    wr(CC40 + INTE, INT_PM | INT_CMU);
    wr(CC40 + TCCLR, TCCLR_TCC);

    // 4. CC41: One-Shot-Alarm. Period-Match auf Service-Request 1 (IRQ 45).
    wr(CC41 + PSC, psiv);
    wr(CC41 + TC, TC_TSSM);
    wr(CC41 + SRS, 1);
    wr(CC41 + SWR, SWR_ALL);
    wr(CC41 + INTE, INT_PM);

    // 5. Interrupts im NVIC freigeben (Priorität bleibt auf dem Reset-Wert 0).
    unsafe {
        NVIC::unmask(Interrupt::CCU40_0);
        NVIC::unmask(Interrupt::CCU40_1);
    }

    // 6. Zähler starten und Zeitgeber freigeben.
    wr(CC40 + TCSET, TCSET_TRBS);
    DRIVER.initialized.store(true, Ordering::Release);

    // Timer, die schon vor `init` eingereiht wurden, bekommen jetzt ihren Alarm.
    critical_section::with(|cs| DRIVER.trigger_alarm(cs));
}
