# ch585-emulator

[![CI](https://github.com/mcskny/ch585-emulator/actions/workflows/ci.yml/badge.svg)](https://github.com/mcskny/ch585-emulator/actions/workflows/ci.yml)
[![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

A fast, **zero-dependency** Rust emulator for the **WCH CH585M** (`QingKe V3C` RV32IMAC + Zba/Zbb/Zbc/Zbs + WCH XW) microcontroller, designed to execute unmodified firmware ELFs faster than real time (`~3–4x+` real time at `62.4 MHz`, including live external board circuits), with a CLI runner, embeddable Rust core, multi-chip RF cluster, and WebAssembly browser UI.

---

## Features

- **QingKe V3C CPU & Memory**
  - RV32IMAC + Zba/Zbb/Zbc/Zbs bitmanip + WCH `XW` 16-bit compressed instructions and `mcpy`.
  - Machine/User privilege transitions (`MachineMode_Call`), `mtvec` mode-3 absolute vector table, 3-level hardware stacking (HPE), `PFIC` level-driven IRQs, and `SysTick`.
  - 512 KB Flash (with `ROM_CFG` Chip ID / MAC / temperature calibration + JEDEC SPI Flash controller for `FLASH_ROM_*` / `EEPROM_*`, timing `WIP`, and mid-operation power-cut corruption) and 128 KB SRAM.
  - Direct-mapped cached basic-block execution engine (`run_fast_batch`) with two-level SRAM self-modifying code tracking and `--exact` single-cycle lockstep fallback.
- **On-Chip Peripherals**
  - **Clocks, Reset & Power**: PLL/HSI/HSE/LSE/LSI (including `--lse-fail`), `WDOG`/`IWDG`, `VDD` BOR + hysteresis, `BAT_DET`, `Active`/`Idle`/`Halt`/`Sleep`/`Shutdown` power modes, GPIO/RTC wakeup, `O(1)` idle/sleep time-skipping, and per-peripheral `PowerReport`.
  - **GPIO (`PA0..15`, `PB0..23`)**: bitwise effective level calculation, internal/external pull-up/pull-down, contention logging, edge/level interrupts, and resistive Millman pin-voltage estimation.
  - **`TMR0..3` & `PWMX` (`PWM4..11`)**: timers, capture/compare, PWM + DMA FIFO with chronological short-pulse edge preservation.
  - **14-Channel 12-bit ADC**: single, continuous, multi-channel scan & DMA, PGA gain stages, internal VBAT & temp sensor, deterministic noise, piecewise calibration curves, and RC hold-capacitor settling (`tau`).
  - **`UART0..3`, `SPI0..1` (with DMA), and `RTC`**.
  - **USB 2.0 HS (`USB2`)**: virtual USB HS host supporting automatic Bus Reset, full descriptor enumeration (Device/Config/String/HID), EP1–EP7 IN/OUT DMA transfers, hot-plug/VBUS re-enumeration, suspend & remote wakeup, and NKRO/vendor decoding.
- **High-Level BLE & 2.4 GHz RF Bridges**
  - Symbol/callback trampoline bridges `wchrf.h` (`RF_Tx`, `RF_Rx`, Fast/Bound descriptor rings) and `libCH58xBLE.a` (GAP/GATT/TMOS/LowPower/SNV bonding in DataFlash) for end-to-end host interactions (`ble_scan`, `ble_connect`, `ble_pair_and_bond`, GATT read/write/notifications, reconnect) and `MultiChipCluster` simulation over `GpioWire`, `UartWire`, and `RfMedium`.
- **Live External Board Models at Real-Time Speed**
  - `ExternalDevice` trait + zero-allocation `Ch585Machine::run_us_with` / `step_with` and `attach_device`:
    - `PinWatchMask` to filter watched MCU pins without unnecessary event allocation.
    - `InputReadMask` + `on_before_input_read` (`PA_PIN`, `PB_PIN`, and exact `Adc::step` pre-sample hooks).
    - Dynamic device deadlines (`next_event_ns` / `on_time_reached`) honored by both active block budgets and `Sleep` fast-forward.
  - `ch585-boards` includes `MacropadV2Board` (`PB16` ratiometric analog rail, Li-ion divider, `74HC165`, quadrature encoder, `WS2812`/`SK6812` decoder, `BQ25300` charger).
- **Tooling & Diagnostics**
  - Timed CLI stimuli (`--at`, `--senaryo`), VCD waveform export (`--vcd`), ELF symbol/ISR/stack profiler (`--profile`), JSON calibration override (`--calibration`), deterministic differential JSON reports (`--json`), and WebAssembly + browser UI (`web/`).

---

## Workspace Structure

| Path | Role |
|---|---|
| [`crates/ch585-core`](crates/ch585-core) | Chip-generic CH585M emulator core, tests, and Rust library (`ch585_core::prelude`) |
| [`crates/ch585-boards`](crates/ch585-boards) | Decoupled external board profiles (`MacropadV2Board`) built on `ExternalDevice` |
| [`crates/ch585-cli`](crates/ch585-cli) | CLI emulator runner, scenario injector, VCD/JSON/profiler output |
| [`crates/ch585-wasm`](crates/ch585-wasm) | `cdylib` WebAssembly bindings (`ch585_wasm_abi_version() == 2`) |
| [`web`](web) | Browser interactive dashboard (QFN48 pinout, logic analyzer, UART, USB, flash/power views) |
| [`scripts/build_wasm.sh`](scripts/build_wasm.sh) | Builds `web/ch585_wasm.wasm` targeting `wasm32-unknown-unknown` |
| [`ASSUMPTIONS.md`](ASSUMPTIONS.md) | Hardware modeling notes and design assumptions |

---

## Quick Start

### 1. CLI Runner

```bash
cargo build --release --bin ch585-cli

# Run a firmware ELF for 500 ms with live USB attach & performance stats
target/release/ch585-cli path/to/firmware.elf --usb-plug 0 --pin PB9:high --ms 500 --stats
```

Common CLI options:

- `--ms <N>` / `--cycles <N>`: simulation duration
- `--board macropad_v2`: attach `MacropadV2Board` external hardware model
- `--pin PB9:high` / `--pull PB0:up` / `--adc 0=1650` / `--adc-noise 5mV`: initial pin & ADC setup
- `--at 200ms:adc:0=2800` / `--senaryo scenario.json`: timed external stimuli
- `--usb-full`, `--usb-decode`, `--ble`, `--uart 2`, `--gpio`: inspect decoded output
- `--vcd trace.vcd` / `--vcd-pins PB10,PB5,PB6`: export IEEE 1364 VCD waveform
- `--profile`, `--json`, `--exact`, `--calibration cal.json`, `--lse-fail`, `--vdd 3000mV`

### 2. Embedding `ch585-core` in Rust

```rust
use ch585_core::prelude::*;

struct MyBoard { /* switches, LEDs, sensors */ }

impl ExternalDevice for MyBoard {
    fn watched_pins(&self) -> PinWatchMask {
        PinWatchMask::NONE.with_pb(10)
    }

    fn watched_input_reads(&self) -> InputReadMask {
        InputReadMask::GPIO_B
    }

    fn on_before_input_read(&mut self, _kind: InputReadKind, ctx: &mut PinContext<'_>) {
        ctx.set_pin_drive('B', 3, ExtPinDrive::High);
    }

    fn on_pin_change(&mut self, ev: PinChangeEvent, _ctx: &mut PinContext<'_>) {
        // React to MCU/peripheral edge transitions with exact ev.time_ns
        let _ = (ev.time_ns, ev.level);
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let elf = std::fs::read("firmware.elf")?;
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf)?;

    let mut board = MyBoard {};
    mach.run_us_with(50_000, &mut board);
    Ok(())
}
```

### 3. WebAssembly UI

Requires the `wasm32-unknown-unknown` Rust target (`rustup target add wasm32-unknown-unknown`):

```bash
./scripts/build_wasm.sh
python3 -m http.server -d web 8080
```

Then open `http://localhost:8080` in your browser and load a compiled CH585 `.elf` firmware.

---

## Testing

```bash
cargo test --workspace --release
RUSTDOCFLAGS="-D warnings" cargo doc --workspace --no-deps
```

Optional proprietary WCH/board ELF fixtures are not redistributed in this repository; tests that reference them skip cleanly when absent. You can optionally set `CH585_KLAVYE_ELF=/path/to/klavye.elf` to run the external Rev 2.1 hardware parity & real-time speed acceptance test.

---

## Scope & Limitations

- **`USB FS` (`USB1`), `I2C`, and real capacitive `TouchKey`** are outside scope (`TouchKey` provides a minimal stub returning a fixed baseline count).
- **BLE & 2.4 GHz RF baseband hardware registers** (`0x4000_C000..0x4000_DFFF`) are undocumented/proprietary and are bridged at ELF `.symtab` / callback ABI boundaries rather than bit-level RF PHY registers.
- **Default cycle timing is ideal (1 cycle/instruction)** unless configured with a `TimingModel` calibration JSON, and average power numbers are based on typical datasheet currents rather than PCB-specific measurements.

See [`ASSUMPTIONS.md`](ASSUMPTIONS.md) for the complete technical reference.

---

## License

Dual-licensed under either of:

- MIT License ([`LICENSE-MIT`](LICENSE-MIT))
- Apache License, Version 2.0 ([`LICENSE-APACHE`](LICENSE-APACHE))

at your option.
