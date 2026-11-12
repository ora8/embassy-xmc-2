# embassy-xmc4700

Embassy-based Rust project foundation for the **Infineon XMC4700 Relax Light Kit**.

## Architecture

- `embassy-executor`: original Embassy async executor, Cortex-M thread mode.
- `embassy-time`: Embassy timer API.
- `xmc4700`: generated PAC for register-level access.
- `cortex-m` / `cortex-m-rt`: Cortex-M runtime and core support.
- `embassy-xmc4700`: board-specific HAL layer being developed.

Target: `thumbv7em-none-eabihf` (Cortex-M4F).

## Important status

This is an initial **HAL project scaffold**, not yet a ready-to-flash board-support package.
It intentionally does not guess XMC4700 register addresses, clock frequencies, pin mux
settings, or LED polarity. In particular:

1. GPIO register backend is not implemented yet.
2. XMC clock-tree and peripheral clock setup are not implemented yet.
3. The `embassy-time-driver` (CCU40, see below) is implemented but has not been verified
   on hardware yet. It needs a CCU clock of `TICK_HZ * 2^n` (n = 0..=15); with the
   unconfigured 24 MHz clock that is `tick-hz-3_000_000` (the workspace default).
4. Startup/linker memory layout and the exact Relax Kit board revision still need
   verification against the board documentation.

The executor dependency is the real upstream Embassy executor. The project does not
contain a replacement executor.

## Build environment

Install the Rust target:

```sh
rustup target add thumbv7em-none-eabihf
rustup component add rust-src
```

Build the workspace:

```sh
cargo check --workspace
```

The target linker script and memory layout may need to be added for the selected
board startup/runtime arrangement before producing a flashable ELF.

## Planned implementation order

1. Confirm Relax Kit revision, oscillator, and linker memory map.
2. Implement `SystemInit`/clock tree and flash wait states.
3. Implement safe GPIO pin ownership and LED 5.8 output.
4. ~~Implement a monotonic 64-bit Embassy time driver using a CCU timer.~~ Done (CCU40), hardware test pending.
5. Add USIC SPI, I2C and UART drivers.
6. Add board examples: LED heartbeat, I2C scan, SPI TFT (ILI9341/ST7796).

## Time driver (CCU40)

`embassy-xmc/src/time_drivers.rs` uses two slices of the CCU40 module, both clocked with
`fCCU / 2^PSIV` (= `TICK_HZ`):

| Slice | Role | Interrupt |
|-------|------|-----------|
| CC40  | free-running 16-bit counter; period match (`0xFFFF`) and compare match (`0x7FFF`) count half periods, giving the 64-bit `now()` | `CCU40_0` (IRQ 44) |
| CC41  | one-shot alarm; it is stopped while being reprogrammed because period/compare registers are shadowed and only update immediately while the slice is stopped | `CCU40_1` (IRQ 45) |

Alarms further away than 65535 ticks are reached in several legs. The vector table is
`__INTERRUPTS` in `interrupt.rs` (cortex-m-rt runs with its `device` feature, `build.rs`
provides an empty `device.x`).

`embassy-time` must use a tick rate that the CCU4 prescaler can produce exactly:

| fCCU    | `embassy-time` feature | PSIV |
|---------|------------------------|------|
| 24 MHz  | `tick-hz-3_000_000`    | 3    |
| 24 MHz  | `tick-hz-375_000`      | 6    |
| 144 MHz | `tick-hz-9_000_000`    | 4    |

`time_drivers::init(clocks::ccu_clock_hz().unwrap())` panics if no prescaler fits.
`ccu_clock_hz()` only knows the 24 MHz backup clock; with the PLL selected it returns `None`.

## Why a CCU timer for Embassy time?

`embassy-time` requires a monotonic timestamp and a scheduled wake-up callback.
A free-running hardware timer plus compare interrupt is a better fit than a
busy-loop delay. The driver will extend the hardware counter to 64 bits and
maintain the Embassy timer queue.
