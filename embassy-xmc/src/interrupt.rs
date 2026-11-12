use cortex_m::asm;

use crate::time_drivers::{ccu40_0_handler, ccu40_1_handler};

/// Interrupt-Nummern (IRQ-Index im NVIC). Nur die vom HAL genutzten Quellen sind aufgeführt.
/// Die Nummern stammen aus der SVD des xmc4700-PAC.
#[allow(non_camel_case_types)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u16)]
pub enum Interrupt {
    /// CCU40 Service Request 0 (Zähler-Halbperioden des Zeitgebers)
    CCU40_0 = 44,
    /// CCU40 Service Request 1 (Alarm des Zeitgebers)
    CCU40_1 = 45,
}

unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
    fn number(self) -> u16 {
        self as u16
    }
}

#[derive(Clone, Copy)]
#[repr(C)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    reserved: usize,
}

// Gemeinsamer Handler für noch nicht implementierte Interrupts
unsafe extern "C" fn default_handler() {
    loop {
        asm::wfi();
    }
}

// Interrupt-Tabelle für den XMC4700
const fn make_interrupts() -> [Vector; 112] {
    let mut vectors = [
        Vector {
            handler: default_handler,
        };
        112
    ];

    // Reservierte Interrupts laut Infineon-Referenzhandbuch
    vectors[9] = Vector { reserved: 0 };
    vectors[10] = Vector { reserved: 0 };
    vectors[11] = Vector { reserved: 0 };
    vectors[13] = Vector { reserved: 0 };

    // Zeitgeber (siehe time_drivers.rs)
    vectors[Interrupt::CCU40_0 as usize] = Vector {
        handler: ccu40_0_handler,
    };
    vectors[Interrupt::CCU40_1 as usize] = Vector {
        handler: ccu40_1_handler,
    };

    vectors
}

// Die eigentliche Interrupt-Vektortabelle. Der Linker-Abschnitt `.vector_table.interrupts`
// liegt in cortex-m-rt direkt hinter den 16 Core-Exceptions, Eintrag n gehört also zu IRQ n.
#[used]
#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
pub static __INTERRUPTS: [Vector; 112] = make_interrupts();
