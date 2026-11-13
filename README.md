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
2. Clock tree: `clocks::init_144mhz()` runs the system at 144 MHz from the 12 MHz crystal
   (PLL, flash wait states); verified on hardware.
3. The `embassy-time-driver` (CCU40, see below) is implemented and verified
   on hardware. It needs a CCU clock of `TICK_HZ * 2^n` (n = 0..=15); with 144 MHz
   that is `tick-hz-9_000_000` (the workspace default).
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
2. ~~Implement `SystemInit`/clock tree and flash wait states.~~ Done (144 MHz).
3. Implement safe GPIO pin ownership and LED 5.8 output.
4. ~~Implement a monotonic 64-bit Embassy time driver using a CCU timer.~~ Done (CCU40), verified on hardware.
5. Add USIC SPI, I2C and UART drivers. (UART and SPI done, I2C open)
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
`ccu_clock_hz()` computes the clock from the SCU/PLL registers. Call `time_drivers::init`
only after the final clock setup.

## UART / Serial Monitor

`embassy-xmc/src/uart.rs` drives USIC0 channel 0 in ASC mode (blocking, 8N1). On the
Relax Kit this channel is wired to the on-board J-Link's virtual COM port
(`/dev/ttyACM0` on Linux): TX = P1.5 (ALT2, U0C0.DOUT0), RX = P1.4 (U0C0.DX0B).

```rust
embassy_xmc::init(); // clears SCB.CCR.UNALIGN_TRP, set by the boot ROM
clocks::init_144mhz().unwrap(); // first: UART and timer derive their dividers from it
let f_periph = clocks::peripheral_clock_hz().unwrap();
let serial = Uart::new_debug(f_periph, uart::Config { baudrate: 115_200 }).unwrap();
uart::init_print(serial);
println!("Hallo {}", 42);
```

`print!`/`println!` translate `\n` to `\r\n`. `Uart` also offers `write`, `flush`,
`read_byte`, `try_read_byte` and `core::fmt::Write`.

Notes from the hardware test:

- The boot ROM sets `SCB.CCR.UNALIGN_TRP`. Rust's `core::fmt` uses unaligned accesses on
  `thumbv7em`, so formatting a number caused a UsageFault. `embassy_xmc::init()` clears the bit.
- **Clock:** the internal backup oscillator fOFI is imprecise (measured about 36 MHz instead
  of the nominal 24 MHz on the test board). Always call `clocks::init_144mhz()` first; with
  it, UART at 115200 baud and Embassy time are exact (board vs. host time over 12 s: 1.0000).

## SPI / ILI9341 display

`embassy-xmc/src/spi.rs` is a blocking SPI master on USIC2 channel 0 (SSC mode), wired to
the Relax Kit's Arduino header like Infineon's XMC-for-Arduino: SCK = P3.9 (D13),
MOSI = P3.8 (D11), MISO = P3.7 (D12). It implements `embedded_hal::spi::SpiBus`; GPIO
`Output`/`Input` implement the `embedded-hal` digital traits, so `embedded-hal-bus` and
display drivers such as `mipidsi` work directly.

- All four SPI modes, up to fPERIPH/2; `spi::Config::frequency` is an upper limit.
- Verified on hardware with the internal loopback (`cargo run --release --bin spi_loopback`):
  modes 0..3 at 1/10/20/40 MHz correct; up to 20 MHz the bus runs gap-free (2436 KiB/s).
- `cargo run --release --bin ili9341`: ILI9341 demo with `mipidsi` + `embedded-graphics`,
  CS = P3.10 (D10), DC = P1.11 (D9), RESET = P1.10 (D8), SPI 20 MHz. A full-screen clear
  takes 61 ms. Wiring table and settings are at the top of
  `embassy-xmc-test/src/bin/ili9341.rs`.

### Parallel ILI9341 (e.g. TFT_320QDT_9341)

Modules with a 40-pin header (DB0..DB15, RS, WR, RD, CS) use the ILI9341's 16-bit 8080
parallel bus, not SPI. `embassy-xmc/src/parallel.rs` (`Parallel16`) drives DB0..DB15 on
P0.0..P0.15 with one `OMR` write per word; with the `mipidsi` feature it implements
`mipidsi::interface::Interface`. Demo: `cargo run --release --bin ili9341_parallel`;
the wiring table is at the top of `embassy-xmc-test/src/bin/ili9341_parallel.rs`
(RS = P1.11/D9, WR = P1.10/D8, RD = P1.9/D7, CS = P1.8/D4, REST = P1.0/D2).
Touch (XPT2046) and the SD slot of such modules use SPI.

Timing: without padding a word took 39 ns (ILI9341 needs ≥ 66 ns), so the WR strobe is
padded with port reads (`LOW_READS` = 3, `HIGH_READS` = 2); the resulting time per word is
estimated, not yet measured on hardware. The demo prints "ns pro Wort".

## Why a CCU timer for Embassy time?

`embassy-time` requires a monotonic timestamp and a scheduled wake-up callback.
A free-running hardware timer plus compare interrupt is a better fit than a
busy-loop delay. The driver will extend the hardware counter to 64 bits and
maintain the Embassy timer queue.
