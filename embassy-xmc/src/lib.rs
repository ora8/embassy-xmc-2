#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

//! HAL foundation for the Infineon XMC4700.
//!
//! This crate deliberately keeps the hardware-specific layer separate from
//! Embassy's executor and time APIs. Clock-tree and peripheral pin routing
//! must be configured for the exact Relax Kit board revision.

pub mod gpio;
pub mod interrupt;
pub mod clocks;
pub mod parallel;
pub mod spi;
pub mod time_drivers;
pub mod uart;
mod usic;

/// Grundinitialisierung des Cortex-M4; als Erstes in `main` aufrufen.
///
/// Das Boot-ROM (SSW) des XMC4700 setzt `SCB.CCR.UNALIGN_TRP`. Rust geht beim Target
/// `thumbv7em` aber davon aus, dass unausgerichtete Zugriffe erlaubt sind (u. a. die
/// Zahlenformatierung in `core::fmt` nutzt sie). Mit gesetztem Bit endet z. B. jedes
/// `println!("{}", zahl)` in einem UsageFault. Die Funktion löscht das Bit wieder.
///
/// Mehrfaches Aufrufen ist unschädlich; [`uart::Uart`] und [`time_drivers::init`] rufen
/// sie zur Sicherheit ebenfalls auf.
pub fn init() {
    const SCB_CCR: *mut u32 = 0xE000_ED14 as *mut u32;
    const CCR_UNALIGN_TRP: u32 = 1 << 3;
    // Feste Adresse des Configuration and Control Register; Read-Modify-Write ohne
    // Konkurrenz (nur dieses eine Bit wird verändert).
    unsafe {
        let v = core::ptr::read_volatile(SCB_CCR);
        if v & CCR_UNALIGN_TRP != 0 {
            core::ptr::write_volatile(SCB_CCR, v & !CCR_UNALIGN_TRP);
            cortex_m::asm::dsb();
            cortex_m::asm::isb();
        }
    }
}

/// Board/peripheral ownership token.
///
/// The final `Peripherals::take()` implementation will enforce singleton
/// ownership for each peripheral. This initial scaffold does not yet claim
/// ownership of PAC peripherals.
pub struct Peripherals {
    _private: (),
}

impl Peripherals {
    /// Initialize the XMC4700 HAL.
    ///
    /// TODO: implement clock tree, flash wait states, peripheral clock gates,
    /// and board-specific pin mux setup after confirming the board revision.
    pub fn take() -> Self {
        init();
        Self { _private: () }
    }
}
