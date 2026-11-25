mod scenario;

use std::env;
use std::fs;
use std::process;
use std::time::Instant;

use ch585_boards::{MacropadV2Board, MacropadV2Handle};
use ch585_core::calibration::{current_table_from_json_str, AdcCalibrationModel, TimingModel};
use ch585_core::cluster::{GpioPinRef, MultiChipCluster};
use ch585_core::json::JsonValue;
use ch585_core::peripherals::sys::PowerMode;
use ch585_core::report::{decode_nkro_usages, MachineReport, PerfStats};
use ch585_core::rf::RfMediumConfig;
use ch585_core::soc::Ch585Machine;
use ch585_core::vcd::VcdConfig;
use scenario::{
    apply_event_action, parse_action, parse_at_arg, parse_pin_drive_val, parse_pin_name,
    parse_pull_val, parse_scenario_json, EventAction, TimedEvent,
};

fn print_usage() {
    eprintln!(
        "Usage: ch585-cli <firmware.elf> [options]\n\
         Options:\n\
         \x20 --cycles <N>             Run N CPU cycles (default: 2000000 if no --ms/--at)\n\
         \x20 --ms <N>                 Run N milliseconds of emulated time\n\
         \x20 --pin <PAx|PBx>:<0|1|z>  Set initial external digital pin drive (repeatable)\n\
         \x20 --pull <PAx|PBx>:<up|down|none> Set external pull resistor (repeatable)\n\
         \x20 --adc <ch>:<mv>          Set ADC channel DC voltage in mV (repeatable)\n\
         \x20 --adc-noise [<ch>:]<mv>  Set deterministic ADC noise amplitude in mV\n\
         \x20 --at <time>:<kind>:<spec> Schedule timed event (e.g. 200ms:adc:1=2800, 300ms:reset:pin)\n\
         \x20 --scenario <file.json>   Load timed event list from JSON file\n\
         \x20 --flash-in <file.bin>    Load persistent DataFlash (32KB or 512KB image)\n\
         \x20 --flash-out <file.bin>   Save full 512KB flash image after run\n\
         \x20 --vdd <mv>               Set initial MCU VDD supply voltage in mV (default 3300)\n\
         \x20 --power-cut-flash <pct>  Fault inject: cut power at <pct>% of next flash write/erase\n\
         \x20 --usb-unplug <ms>        Detach virtual USB host at <ms>\n\
         \x20 --usb-plug <ms>          Re-attach virtual USB host at <ms>\n\
         \x20 --lse-fail               Simulate 32.768 kHz external crystal failure\n\
         \x20 --exact                  Disable peripheral batching (1-cycle exact peripheral stepping)\n\
         \x20 --board macropad_v2      Enable ratiometric rail, 74HC165, encoder, WS2812, BQ25300\n\
         \x20 --bat <mv>               Set initial Li-ion battery voltage in mV (default 3700 on board)\n\
         \x20 --calibration <file.json> Load combined timing/ADC/current calibration JSON\n\
         \x20 --timing <file.json>     Load CPU/bus timing calibration JSON\n\
         \x20 --adc-cal <file.json>    Load ADC curve/offset/settling calibration JSON\n\
         \x20 --current-table <file>   Load current table override JSON\n\
         \x20 --vcd <out.vcd>          Record and export IEEE 1364 VCD waveform\n\
         \x20 --vcd-pins <PA0,PB10..>  Comma-separated GPIO pins included in VCD export\n\
         \x20 --profile                Enable ELF symbol, ISR latency, and stack depth profiler\n\
         \x20 --stack-limit <addr>     Override stack overflow threshold (hex or decimal)\n\
         \x20 --json                   Emit full machine-readable JSON output to stdout\n\
         \x20 --uart <0..3>            Print UART TX output (default: all non-empty UARTs)\n\
         \x20 --usb                    Print USB HS enumeration & packet summary\n\
         \x20 --usb-full               Print all USB packets with full hex payload\n\
         \x20 --usb-decode             Decode HID NKRO reports and EP4 vendor telemetry text\n\
         \x20 --ble                    Print BLE/RF virtual bridge & Virtual Central summary\n\
         \x20 --gpio                   Print GPIO PA/PB & contention summary\n\
         \x20 --stats                  Print CPU/Clock/Power/IRQ/Unknown-MMIO summary\n\
         \x20 --peer <elf>             Add peer MCU to run in MultiChipCluster (repeatable)\n\
         \x20 --wire-gpio <pA>-<pB>    Wire GPIO pin A (chip 0) to pin B (chip 1) (or c0:pin-c1:pin)\n\
         \x20 --wire-uart <uA>-<uB>    Wire duplex UART between chip 0 & chip 1 (or c0:uA-c1:uB)\n\
         \x20 --rf-loss <pct>          2.4G RF packet loss percentage (0..100)\n\
         \x20 --rf-delay-us <us>       2.4G RF propagation latency in us (default 25)\n\
         \x20 --rf-jitter-us <us>      2.4G RF jitter in us (default 0)\n\
         \x20 --rf-drop-ms <ms>        Inject 2.4G RF link drop at <ms>\n\
         \x20 --quantum-us <us>        Multi-chip sync quantum in us (default 50)"
    );
}

fn parse_chip_pin_endpoint(s: &str, default_chip: usize) -> Result<GpioPinRef, String> {
    if let Some((chip_s, pin_s)) = s.split_once(':') {
        let chip = chip_s.trim().parse::<usize>().map_err(|_| format!("Bad chip index '{chip_s}'"))?;
        let (port, idx) = parse_pin_name(pin_s).ok_or_else(|| format!("Bad pin '{pin_s}'"))?;
        Ok(GpioPinRef::new(chip, port, idx))
    } else {
        let (port, idx) = parse_pin_name(s).ok_or_else(|| format!("Bad pin '{s}'"))?;
        Ok(GpioPinRef::new(default_chip, port, idx))
    }
}

fn parse_chip_uart_endpoint(s: &str, default_chip: usize) -> Result<(usize, usize), String> {
    if let Some((chip_s, u_s)) = s.split_once(':') {
        let chip = chip_s.trim().parse::<usize>().map_err(|_| format!("Bad chip index '{chip_s}'"))?;
        let u = u_s.trim().parse::<usize>().map_err(|_| format!("Bad UART index '{u_s}'"))?;
        Ok((chip, u))
    } else {
        let u = s.trim().parse::<usize>().map_err(|_| format!("Bad UART index '{s}'"))?;
        Ok((default_chip, u))
    }
}

fn parse_u32_hex_or_dec(s: &str) -> Result<u32, String> {
    let t = s.trim();
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16).map_err(|_| format!("Invalid hex u32 '{s}'"))
    } else {
        t.parse::<u32>().map_err(|_| format!("Invalid decimal u32 '{s}'"))
    }
}

fn main() {
    let args: Vec<String> = env::args().skip(1).collect();
    if args.is_empty() || args.iter().any(|a| a == "-h" || a == "--help") {
        print_usage();
        process::exit(if args.is_empty() { 1 } else { 0 });
    }

    let elf_path = &args[0];
    let elf_bytes = fs::read(elf_path).unwrap_or_else(|e| {
        eprintln!("Failed to read {elf_path}: {e}");
        process::exit(1);
    });

    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf_bytes).unwrap_or_else(|e| {
        eprintln!("Failed to parse ELF {elf_path}: {e}");
        process::exit(1);
    });

    let mut run_cycles: Option<u64> = None;
    let mut run_ms: Option<u64> = None;
    let mut uart_filter: Option<usize> = None;
    let mut show_usb = false;
    let mut usb_full = false;
    let mut usb_decode = false;
    let mut show_ble = false;
    let mut show_gpio = false;
    let mut show_stats = false;
    let mut json_out = false;
    let mut profile_en = false;
    let mut stack_limit_override: Option<u32> = None;
    let mut vcd_out_path: Option<String> = None;
    let mut vcd_pins: Vec<(char, u8)> = Vec::new();
    let mut board_handle: Option<MacropadV2Handle> = None;

    let mut init_adc_overrides: Vec<(usize, i32)> = Vec::new();
    let mut init_bat_mv: Option<i32> = None;
    let mut init_vdd_mv: Option<i32> = None;
    let mut flash_out_path: Option<String> = None;
    let mut timed_events: Vec<TimedEvent> = Vec::new();
    let mut peer_elfs: Vec<String> = Vec::new();
    let mut gpio_wire_specs: Vec<(GpioPinRef, GpioPinRef)> = Vec::new();
    let mut uart_wire_specs: Vec<((usize, usize), (usize, usize))> = Vec::new();
    let mut rf_cfg = RfMediumConfig::default();
    let mut rf_drop_ms: Option<u64> = None;
    let mut quantum_us: u64 = 50;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--cycles" => {
                i += 1;
                run_cycles = Some(args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --cycles value");
                    process::exit(1);
                }));
            }
            "--ms" => {
                i += 1;
                run_ms = Some(args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --ms value");
                    process::exit(1);
                }));
            }
            "--pin" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                let (pin_s, val_s) = spec.split_once(':').unwrap_or_else(|| {
                    eprintln!("Invalid --pin spec: {spec}");
                    process::exit(1);
                });
                let (port, idx) = parse_pin_name(pin_s).unwrap_or_else(|| process::exit(1));
                let drv = parse_pin_drive_val(val_s).unwrap_or_else(|| process::exit(1));
                mach.set_pin_drive(port, idx, drv);
            }
            "--pull" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                let (pin_s, val_s) = spec.split_once(':').unwrap_or_else(|| {
                    eprintln!("Invalid --pull spec: {spec}");
                    process::exit(1);
                });
                let (port, idx) = parse_pin_name(pin_s).unwrap_or_else(|| process::exit(1));
                let pull = parse_pull_val(val_s).unwrap_or_else(|| process::exit(1));
                mach.set_pin_pull(port, idx, pull);
            }
            "--adc" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                match parse_action("adc", spec) {
                    Ok(EventAction::Adc { ch, mv }) => init_adc_overrides.push((ch, mv)),
                    Ok(_) => unreachable!(),
                    Err(e) => {
                        eprintln!("Invalid --adc: {e}");
                        process::exit(1);
                    }
                }
            }
            "--adc-noise" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                match parse_action("adc-noise", spec) {
                    Ok(act) => apply_event_action(&mut mach, &mut board_handle, &act),
                    Err(e) => {
                        eprintln!("Invalid --adc-noise: {e}");
                        process::exit(1);
                    }
                }
            }
            "--at" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                match parse_at_arg(spec) {
                    Ok(ev) => timed_events.push(ev),
                    Err(e) => {
                        eprintln!("{e}");
                        process::exit(1);
                    }
                }
            }
            "--scenario" | "--senaryo" => {
                i += 1;
                let path = args.get(i).unwrap_or_else(|| process::exit(1));
                let txt = fs::read_to_string(path).unwrap_or_else(|e| {
                    eprintln!("Failed to read scenario {path}: {e}");
                    process::exit(1);
                });
                match parse_scenario_json(&txt) {
                    Ok(mut evs) => timed_events.append(&mut evs),
                    Err(e) => {
                        eprintln!("Scenario error in {path}: {e}");
                        process::exit(1);
                    }
                }
            }
            "--usb-unplug" => {
                i += 1;
                let ms: u64 = args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --usb-unplug ms");
                    process::exit(1);
                });
                timed_events.push(TimedEvent {
                    at_us: ms * 1000,
                    action: EventAction::UsbDetach,
                });
            }
            "--usb-plug" => {
                i += 1;
                let ms: u64 = args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --usb-plug ms");
                    process::exit(1);
                });
                timed_events.push(TimedEvent {
                    at_us: ms * 1000,
                    action: EventAction::UsbAttach,
                });
            }
            "--lse-fail" => mach.bus.sys.lse_failed = true,
            "--flash-in" => {
                i += 1;
                let path = args.get(i).unwrap_or_else(|| process::exit(1));
                let data = fs::read(path).unwrap_or_else(|e| {
                    eprintln!("Failed to read --flash-in {path}: {e}");
                    process::exit(1);
                });
                mach.load_flash_overlay_dataflash(&data);
            }
            "--flash-out" => {
                i += 1;
                let path = args.get(i).unwrap_or_else(|| process::exit(1)).clone();
                flash_out_path = Some(path);
            }
            "--vdd" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                match parse_action("vdd", spec) {
                    Ok(EventAction::VddMv(mv)) => init_vdd_mv = Some(mv),
                    Ok(_) => unreachable!(),
                    Err(e) => {
                        eprintln!("Invalid --vdd: {e}");
                        process::exit(1);
                    }
                }
            }
            "--power-cut-flash" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                match parse_action("flash-cut", spec) {
                    Ok(act) => apply_event_action(&mut mach, &mut board_handle, &act),
                    Err(e) => {
                        eprintln!("Invalid --power-cut-flash: {e}");
                        process::exit(1);
                    }
                }
            }
            "--board" => {
                i += 1;
                let name = args.get(i).unwrap_or_else(|| process::exit(1)).clone();
                if name != "macropad_v2" {
                    eprintln!("Unknown board profile '{name}' (supported: macropad_v2)");
                    process::exit(1);
                }
                board_handle = Some(MacropadV2Board::attach(&mut mach));
            }
            "--bat" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                match parse_action("bat", spec) {
                    Ok(EventAction::BatMv(mv)) => init_bat_mv = Some(mv),
                    Ok(_) => unreachable!(),
                    Err(e) => {
                        eprintln!("Invalid --bat: {e}");
                        process::exit(1);
                    }
                }
            }
            "--uart" => {
                i += 1;
                uart_filter = Some(args.get(i).and_then(|s| s.parse().ok()).unwrap_or(0));
            }
            "--usb" => show_usb = true,
            "--usb-full" => {
                show_usb = true;
                usb_full = true;
            }
            "--usb-decode" => {
                show_usb = true;
                usb_decode = true;
            }
            "--ble" => show_ble = true,
            "--gpio" => show_gpio = true,
            "--stats" => show_stats = true,
            "--exact" => mach.exact_periph_step = true,
            "--json" => json_out = true,
            "--profile" => profile_en = true,
            "--stack-limit" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                let lim = parse_u32_hex_or_dec(spec).unwrap_or_else(|e| {
                    eprintln!("{e}");
                    process::exit(1);
                });
                stack_limit_override = Some(lim);
            }
            "--vcd" => {
                i += 1;
                vcd_out_path = Some(args.get(i).unwrap_or_else(|| process::exit(1)).clone());
            }
            "--vcd-pins" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                for part in spec.split(',') {
                    if let Some((p, idx)) = parse_pin_name(part) {
                        vcd_pins.push((p, idx));
                    } else if !part.trim().is_empty() {
                        eprintln!("Invalid pin in --vcd-pins: '{part}'");
                        process::exit(1);
                    }
                }
            }
            "--calibration" => {
                i += 1;
                let path = args.get(i).unwrap_or_else(|| process::exit(1));
                let txt = fs::read_to_string(path).unwrap_or_else(|e| {
                    eprintln!("Failed to read --calibration {path}: {e}");
                    process::exit(1);
                });
                if let Err(e) = mach.load_calibration_json(&txt) {
                    eprintln!("Calibration error in {path}: {e}");
                    process::exit(1);
                }
            }
            "--timing" => {
                i += 1;
                let path = args.get(i).unwrap_or_else(|| process::exit(1));
                let txt = fs::read_to_string(path).unwrap_or_else(|e| {
                    eprintln!("Failed to read --timing {path}: {e}");
                    process::exit(1);
                });
                match TimingModel::from_json_str(&txt) {
                    Ok(t) => mach.set_timing_model(t),
                    Err(e) => {
                        eprintln!("Timing JSON error in {path}: {e}");
                        process::exit(1);
                    }
                }
            }
            "--adc-cal" => {
                i += 1;
                let path = args.get(i).unwrap_or_else(|| process::exit(1));
                let txt = fs::read_to_string(path).unwrap_or_else(|e| {
                    eprintln!("Failed to read --adc-cal {path}: {e}");
                    process::exit(1);
                });
                match AdcCalibrationModel::from_json_str(&txt) {
                    Ok(cal) => mach.set_adc_calibration(cal),
                    Err(e) => {
                        eprintln!("ADC calibration JSON error in {path}: {e}");
                        process::exit(1);
                    }
                }
            }
            "--current-table" => {
                i += 1;
                let path = args.get(i).unwrap_or_else(|| process::exit(1));
                let txt = fs::read_to_string(path).unwrap_or_else(|e| {
                    eprintln!("Failed to read --current-table {path}: {e}");
                    process::exit(1);
                });
                match current_table_from_json_str(&txt) {
                    Ok(tbl) => mach.set_current_table(tbl),
                    Err(e) => {
                        eprintln!("Current table JSON error in {path}: {e}");
                        process::exit(1);
                    }
                }
            }
            "--peer" => {
                i += 1;
                let path = args.get(i).unwrap_or_else(|| process::exit(1)).clone();
                peer_elfs.push(path);
            }
            "--wire-gpio" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                let (a_s, b_s) = spec.split_once('-').unwrap_or_else(|| {
                    eprintln!("Invalid --wire-gpio '{spec}' (expected PA4-PB0)");
                    process::exit(1);
                });
                let a = parse_chip_pin_endpoint(a_s, 0).unwrap_or_else(|e| {
                    eprintln!("{e}");
                    process::exit(1);
                });
                let b = parse_chip_pin_endpoint(b_s, 1).unwrap_or_else(|e| {
                    eprintln!("{e}");
                    process::exit(1);
                });
                gpio_wire_specs.push((a, b));
            }
            "--wire-uart" => {
                i += 1;
                let spec = args.get(i).unwrap_or_else(|| process::exit(1));
                let (a_s, b_s) = spec.split_once('-').unwrap_or_else(|| {
                    eprintln!("Invalid --wire-uart '{spec}' (expected 1-1)");
                    process::exit(1);
                });
                let a = parse_chip_uart_endpoint(a_s, 0).unwrap_or_else(|e| {
                    eprintln!("{e}");
                    process::exit(1);
                });
                let b = parse_chip_uart_endpoint(b_s, 1).unwrap_or_else(|e| {
                    eprintln!("{e}");
                    process::exit(1);
                });
                uart_wire_specs.push((a, b));
            }
            "--rf-loss" => {
                i += 1;
                rf_cfg.packet_loss_pct = args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --rf-loss value");
                    process::exit(1);
                });
            }
            "--rf-delay-us" => {
                i += 1;
                rf_cfg.delay_us = args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --rf-delay-us value");
                    process::exit(1);
                });
            }
            "--rf-jitter-us" => {
                i += 1;
                rf_cfg.jitter_us = args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --rf-jitter-us value");
                    process::exit(1);
                });
            }
            "--rf-drop-ms" => {
                i += 1;
                rf_drop_ms = Some(args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --rf-drop-ms value");
                    process::exit(1);
                }));
            }
            "--quantum-us" => {
                i += 1;
                quantum_us = args.get(i).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
                    eprintln!("Invalid --quantum-us value");
                    process::exit(1);
                });
            }
            other => {
                eprintln!("Unknown option: {other}");
                process::exit(1);
            }
        }
        i += 1;
    }

    // Apply initial battery & ADC settings AFTER --board so dividers/ratios match overrides
    if let Some(mv) = init_bat_mv {
        apply_event_action(&mut mach, &mut board_handle, &EventAction::BatMv(mv));
    }
    for (ch, mv) in init_adc_overrides {
        apply_event_action(&mut mach, &mut board_handle, &EventAction::Adc { ch, mv });
    }
    if let Some(mv) = init_vdd_mv {
        mach.set_vdd_mv(mv);
    }
    if let Some(b) = board_handle.as_ref() {
        b.sync(&mut mach);
    } else {
        mach.poll_ext();
    }

    if vcd_out_path.is_some() {
        mach.enable_vcd_trace(131_072);
    }
    if profile_en || stack_limit_override.is_some() {
        mach.enable_profiler(true);
        if let Some(lim) = stack_limit_override {
            let top = mach.profiler_report().stack.top;
            mach.set_stack_bounds(top, lim);
        }
    }

    timed_events.sort_by_key(|e| e.at_us);

    let print_opts = PrintOpts {
        uart_filter,
        show_usb,
        usb_full,
        usb_decode,
        show_ble,
        show_gpio,
        show_stats,
    };

    if !peer_elfs.is_empty() {
        let mut cluster = MultiChipCluster::with_rf_config(rf_cfg);
        cluster.quantum_us = quantum_us.max(1);
        cluster.rf_medium.drop_link_at_ns = rf_drop_ms.map(|ms| ms.saturating_mul(1_000_000));
        cluster.add_chip(elf_path.clone(), mach);
        for peer_path in &peer_elfs {
            let p_bytes = fs::read(peer_path).unwrap_or_else(|e| {
                eprintln!("Failed to read peer ELF {peer_path}: {e}");
                process::exit(1);
            });
            let mut peer_mach = Ch585Machine::new();
            peer_mach.load_elf(&p_bytes).unwrap_or_else(|e| {
                eprintln!("Failed to parse peer ELF {peer_path}: {e}");
                process::exit(1);
            });
            cluster.add_chip(peer_path.clone(), peer_mach);
        }
        for (a, b) in gpio_wire_specs {
            cluster.connect_gpio(a, b);
        }
        for ((c0, u0), (c1, u1)) in uart_wire_specs {
            cluster.connect_uart_duplex(c0, u0, c1, u1);
        }

        let wall_start = Instant::now();
        let max_ev_us = timed_events.last().map(|e| e.at_us).unwrap_or(0);
        let default_us = run_cycles.map(|c| c / 60).unwrap_or(if max_ev_us > 0 { max_ev_us + 20_000 } else { 50_000 });
        let total_us = run_ms.map(|ms| ms * 1000).unwrap_or(default_us);
        let mut cur_us = 0u64;
        for ev in &timed_events {
            if ev.at_us > total_us {
                break;
            }
            if ev.at_us > cur_us {
                cluster.run_us(ev.at_us - cur_us);
                cur_us = ev.at_us;
            }
            apply_event_action(cluster.chip_mut(0), &mut board_handle, &ev.action);
        }
        if total_us > cur_us {
            cluster.run_us(total_us - cur_us);
        }
        let wall_elapsed = wall_start.elapsed();
        let wall_s = wall_elapsed.as_secs_f64().max(1e-9);
        let total_instret: u64 = cluster.chips.iter().map(|c| c.cpu.instret).sum();
        let sim_s = (cluster.sim_time_ns as f64) * 1e-9;
        let perf = PerfStats {
            wall_ms: wall_s * 1000.0,
            speed_x: sim_s / wall_s,
            mips: (total_instret as f64) / (wall_s * 1e6),
        };

        if let Some(path) = flash_out_path {
            if let Err(e) = fs::write(&path, cluster.chip(0).flash_bytes()) {
                eprintln!("Failed to write --flash-out {path}: {e}");
                process::exit(1);
            }
        }
        if let Some(path) = vcd_out_path {
            let vcd = cluster.chip(0).export_vcd(&VcdConfig {
                pins: vcd_pins,
                include_internal: true,
            });
            if let Err(e) = fs::write(&path, vcd) {
                eprintln!("Failed to write --vcd {path}: {e}");
                process::exit(1);
            }
        }

        if json_out {
            let chip_vals: Vec<JsonValue> = cluster
                .chips
                .iter()
                .enumerate()
                .map(|(idx, c)| {
                    let rep = MachineReport::from_machine(c, uart_filter, false);
                    let bj = if idx == 0 {
                        board_json_value(board_handle.as_ref())
                    } else {
                        None
                    };
                    let sim_c_s = (c.bus.total_time_ns as f64) * 1e-9;
                    let c_perf = PerfStats {
                        wall_ms: perf.wall_ms,
                        speed_x: sim_c_s / wall_s,
                        mips: (c.cpu.instret as f64) / (wall_s * 1e6),
                    };
                    rep.to_json_value(bj, Some(c_perf))
                })
                .collect();
            let rf_med = JsonValue::Object(vec![
                ("link_dropped".into(), JsonValue::Bool(cluster.rf_medium.link_dropped)),
                ("total_sent".into(), JsonValue::Number(cluster.rf_medium.total_sent as f64)),
                ("total_delivered".into(), JsonValue::Number(cluster.rf_medium.total_delivered as f64)),
                ("total_dropped_loss".into(), JsonValue::Number(cluster.rf_medium.total_dropped_loss as f64)),
                ("total_dropped_link".into(), JsonValue::Number(cluster.rf_medium.total_dropped_link as f64)),
                ("total_crc_errors".into(), JsonValue::Number(cluster.rf_medium.total_crc_errors as f64)),
            ]);
            let root = JsonValue::Object(vec![
                ("schema_version".into(), JsonValue::Number(1.0)),
                ("sim_time_ns".into(), JsonValue::Number(cluster.sim_time_ns as f64)),
                ("quantum_us".into(), JsonValue::Number(cluster.quantum_us as f64)),
                ("rf_medium".into(), rf_med),
                ("chips".into(), JsonValue::Array(chip_vals)),
                (
                    "perf".into(),
                    JsonValue::Object(vec![
                        ("wall_ms".into(), JsonValue::Number(perf.wall_ms)),
                        ("speed_x".into(), JsonValue::Number(perf.speed_x)),
                        ("mips".into(), JsonValue::Number(perf.mips)),
                    ]),
                ),
            ]);
            println!("{}", root.to_json_string(true));
            return;
        }

        for (idx, c) in cluster.chips.iter().enumerate() {
            println!("==================== CHIP #{idx}: {} ====================", cluster.names[idx]);
            let c_sim_s = (c.bus.total_time_ns as f64) * 1e-9;
            let c_perf = PerfStats {
                wall_ms: perf.wall_ms,
                speed_x: c_sim_s / wall_s,
                mips: (c.cpu.instret as f64) / (wall_s * 1e6),
            };
            print_chip_text(c, if idx == 0 { board_handle.as_ref() } else { None }, c_perf, &print_opts);
        }
        if show_ble || show_stats {
            let rm = &cluster.rf_medium;
            println!(
                "=== RF MEDIUM ===\nSimTimeUs={} QuantumUs={} LinkDropped={} Sent={} Delivered={} DroppedLoss={} DroppedLink={} CrcErrors={} WallMs={:.1} TotalMips={:.1}",
                cluster.sim_time_ns / 1000,
                cluster.quantum_us,
                rm.link_dropped,
                rm.total_sent,
                rm.total_delivered,
                rm.total_dropped_loss,
                rm.total_dropped_link,
                rm.total_crc_errors,
                perf.wall_ms,
                perf.mips,
            );
        }
        return;
    }

    let wall_start = Instant::now();
    if !timed_events.is_empty() || run_ms.is_some() {
        let max_ev_us = timed_events.last().map(|e| e.at_us).unwrap_or(0);
        let total_us = run_ms.map(|ms| ms * 1000).unwrap_or(max_ev_us + 20_000);
        let mut cur_us = 0u64;
        for ev in &timed_events {
            if ev.at_us > total_us {
                break;
            }
            if ev.at_us > cur_us {
                mach.run_us(ev.at_us - cur_us);
                cur_us = ev.at_us;
            }
            apply_event_action(&mut mach, &mut board_handle, &ev.action);
        }
        if total_us > cur_us {
            mach.run_us(total_us - cur_us);
        }
    } else {
        mach.run_cycles(run_cycles.unwrap_or(2_000_000));
    }
    let wall_elapsed = wall_start.elapsed();

    if let Some(path) = flash_out_path {
        if let Err(e) = fs::write(&path, mach.flash_bytes()) {
            eprintln!("Failed to write --flash-out {path}: {e}");
            process::exit(1);
        }
    }

    if let Some(path) = vcd_out_path {
        let vcd = mach.export_vcd(&VcdConfig {
            pins: vcd_pins,
            include_internal: true,
        });
        if let Err(e) = fs::write(&path, vcd) {
            eprintln!("Failed to write --vcd {path}: {e}");
            process::exit(1);
        }
    }

    let sim_s = (mach.bus.total_time_ns as f64) * 1e-9;
    let wall_s = wall_elapsed.as_secs_f64().max(1e-9);
    let perf = PerfStats {
        wall_ms: wall_s * 1000.0,
        speed_x: sim_s / wall_s,
        mips: (mach.cpu.instret as f64) / (wall_s * 1e6),
    };

    if json_out {
        let board_json = board_json_value(board_handle.as_ref());
        let report = MachineReport::from_machine(&mach, uart_filter, false);
        println!("{}", report.to_json_string(board_json, Some(perf), true));
        return;
    }

    print_chip_text(&mach, board_handle.as_ref(), perf, &print_opts);
}

struct PrintOpts {
    uart_filter: Option<usize>,
    show_usb: bool,
    usb_full: bool,
    usb_decode: bool,
    show_ble: bool,
    show_gpio: bool,
    show_stats: bool,
}

fn board_json_value(board_handle: Option<&MacropadV2Handle>) -> Option<JsonValue> {
    board_handle.map(|b| {
        let st = b.state();
        JsonValue::Object(vec![
            ("profile".into(), JsonValue::Str("macropad_v2".to_string())),
            ("charging".into(), JsonValue::Bool(st.charger.charging)),
            ("fault".into(), JsonValue::Bool(st.charger.fault)),
            ("vbat_mv".into(), JsonValue::Number(st.battery.mv as f64)),
            ("vbat_ch".into(), JsonValue::Number(st.battery.ch as f64)),
            ("vbat_pin_mv".into(), JsonValue::Number(st.battery.pin_mv() as f64)),
            ("encoder_pending".into(), JsonValue::Number(st.encoder.pending() as f64)),
            (
                "shift165_bits".into(),
                JsonValue::Str(format!("0x{:04X}", st.shift165.input_bits)),
            ),
            (
                "ws2812_frames".into(),
                JsonValue::Number(st.ws2812.frame_count as f64),
            ),
            (
                "ws2812_leds".into(),
                JsonValue::Array(
                    st.ws2812
                        .leds
                        .iter()
                        .map(|rgb| {
                            JsonValue::Array(vec![
                                JsonValue::Number(rgb[0] as f64),
                                JsonValue::Number(rgb[1] as f64),
                                JsonValue::Number(rgb[2] as f64),
                            ])
                        })
                        .collect(),
                ),
            ),
        ])
    })
}

fn print_chip_text(
    mach: &Ch585Machine,
    board_handle: Option<&MacropadV2Handle>,
    perf: PerfStats,
    opts: &PrintOpts,
) {
    for u in 0..4 {
        if opts.uart_filter.is_some() && opts.uart_filter != Some(u) {
            continue;
        }
        let tx = &mach.bus.uarts[u].tx_log;
        if !tx.is_empty() || opts.uart_filter == Some(u) {
            println!("=== UART{u} TX ({} bytes) ===", tx.len());
            println!("{}", String::from_utf8_lossy(tx));
        }
    }

    if opts.show_usb {
        let u = &mach.bus.usbhs;
        let vid = if u.device_desc.len() >= 10 {
            u16::from_le_bytes([u.device_desc[8], u.device_desc[9]])
        } else {
            0
        };
        let pid = if u.device_desc.len() >= 12 {
            u16::from_le_bytes([u.device_desc[10], u.device_desc[11]])
        } else {
            0
        };
        println!(
            "=== USB HS ===\nAttached={} Enumerated={} Suspended={} RemoteWakeups={} DevAddr={} VID={:04X} PID={:04X} DevDesc={}B CfgDesc={}B Strings={} HidReportDescs={} Packets={}",
            u.host_attached,
            u.enumerated,
            u.suspended,
            u.remote_wakeup_count,
            u.dev_ad,
            vid,
            pid,
            u.device_desc.len(),
            u.config_desc.len(),
            u.string_descs.len(),
            u.hid_report_descs.len(),
            u.in_packets.len(),
        );
        let limit = if opts.usb_full { u.in_packets.len() } else { 8 };
        for pkt in u.in_packets.iter().take(limit) {
            let show_bytes = if opts.usb_full { pkt.data.len() } else { pkt.data.len().min(16) };
            println!(
                "  EP{} {} ({}B): {:02X?}",
                pkt.ep,
                if pkt.is_in { "IN" } else { "OUT" },
                pkt.data.len(),
                &pkt.data[..show_bytes]
            );
        }

        if opts.usb_decode {
            let mut kbd_events = Vec::new();
            let mut telem = String::new();
            let mut vendor_pkts = Vec::new();
            for pkt in &u.in_packets {
                if pkt.ep == 1 && pkt.is_in {
                    if let Some((mods, keys)) = decode_nkro_usages(&pkt.data) {
                        kbd_events.push(format!("mods=0x{mods:02X} keys={keys:?}"));
                    }
                } else if pkt.ep == 4 && pkt.is_in && pkt.data.len() >= 6 && pkt.data[0] == 0x57 && pkt.data[1] == 0x01 {
                    if pkt.data[2] == 0x02 {
                        let n = ((pkt.data[5] & 0x7F) as usize).min(pkt.data.len().saturating_sub(6));
                        telem.push_str(&String::from_utf8_lossy(&pkt.data[6..6 + n]));
                    } else {
                        vendor_pkts.push(format!("type=0x{:02X} seq=0x{:02X} data={:02X?}", pkt.data[2], pkt.data[3], &pkt.data[4..]));
                    }
                }
            }
            if !kbd_events.is_empty() {
                println!("=== USB DECODE (EP1 NKRO, {} reports) ===", kbd_events.len());
                for line in kbd_events {
                    println!("  {line}");
                }
            }
            if !telem.is_empty() {
                println!("=== USB DECODE (EP4 Telemetry) ===");
                for line in telem.lines() {
                    if !line.is_empty() {
                        println!("  {line}");
                    }
                }
            }
            if !vendor_pkts.is_empty() {
                println!("=== USB DECODE (EP4 Responses, {}) ===", vendor_pkts.len());
                for line in vendor_pkts {
                    println!("  {line}");
                }
            }
        }
    }

    if opts.show_ble {
        let b = &mach.bus.ble;
        println!(
            "=== BLE BRIDGE ===\nHooks={} Central={} Advertising={} Bonds={} Conn=[int={},lat={},to={}] TMOSStarts={} LowPower={} SleepHooks={}\nAdvData={}B ScanRsp={}B Events={} HidReports={}\nRF Mode={} Bound={} Tx={} Rx={} CrcErr={} Timeouts={}",
            b.hooks.len(),
            b.central_state.as_str(),
            b.advertising_enabled,
            b.bonds.len(),
            b.conn_params.interval,
            b.conn_params.latency,
            b.conn_params.timeout,
            b.tmos_start_calls,
            b.low_power_calls,
            b.sleep_hook_calls,
            b.adv_data.len(),
            b.scan_rsp_data.len(),
            b.timestamped_events.len(),
            b.timestamped_hid_reports.len(),
            b.rf.mode.as_str(),
            b.rf.bound_connected,
            b.rf.tx_count,
            b.rf.rx_count,
            b.rf.rx_crc_err_count,
            b.rf.timeout_count,
        );
        for ev in b.timestamped_events.iter().take(24) {
            println!("  [{}us] {}: {}", ev.time_ns / 1000, ev.kind, ev.detail);
        }
        for rpt in b.timestamped_hid_reports.iter().take(16) {
            println!(
                "  [{}us] HID id={} attr=0x{:04X} data={:02X?}",
                rpt.time_ns / 1000,
                rpt.report_id,
                rpt.attr_handle,
                rpt.data
            );
        }
    }

    if opts.show_gpio {
        println!(
            "=== GPIO ===\nPA DIR=0x{:04X} OUT=0x{:04X} PIN=0x{:04X}\nPB DIR=0x{:06X} OUT=0x{:06X} PIN=0x{:06X}\nContention={:?}",
            mach.bus.gpio.pa_dir,
            mach.bus.gpio.pa_out,
            mach.bus.gpio.effective_pa_pin(),
            mach.bus.gpio.pb_dir,
            mach.bus.gpio.pb_out,
            mach.bus.gpio.effective_pb_pin(),
            mach.bus.gpio.contention_log,
        );
    }

    if let Some(b) = board_handle {
        let st = b.state();
        println!(
            "=== BOARD (macropad_v2) ===\nBQ25300 Charging={} Fault={} | VBAT={}mV (AIN{}={}mV) | EncoderPending={} | 74HC165=0x{:04X} | WS2812 Frames={} LEDs={:?}",
            st.charger.charging,
            st.charger.fault,
            st.battery.mv,
            st.battery.ch,
            st.battery.pin_mv(),
            st.encoder.pending(),
            st.shift165.input_bits,
            st.ws2812.frame_count,
            st.ws2812.leds,
        );
    }

    if mach.profiler.enabled {
        print!("{}", mach.profiler_report().format_text(16));
    }

    if opts.show_stats {
        let tm = &mach.bus.time_ns_by_mode;
        let pr = mach.bus.power_report();
        let speed_x = perf.speed_x;
        let mips = perf.mips;
        let wall_ms = perf.wall_ms;
        let rst_name = match mach.bus.sys.reset_flag & 0x07 {
            0x00 => "SW",
            0x01 => "RPOR",
            0x02 => "WTR",
            0x03 => "MR",
            0x05 => "GPWSM",
            _ => "OTHER",
        };
        println!(
            "=== STATS ===\nPC=0x{:08X} Cycles={} TimeUs={} SysClk={}Hz Instret={} LogicEvents={} Contention={} UnknownMMIO={:08X?}\nLastReset=0x{:02X} ({rst_name}) Powered={} BOR={} VDD={}mV SleepSkippedUs={} FlashInterruptedOps={}\nPowerModesUs=[Active={}, Idle={}, Halt={}, Sleep={}, Shutdown={}] EstCurrent={:.1}uA\nPowerReport: Core={:.1}uA ADC={:.1}uA USBHS={:.1}uA BLE={:.1}uA UART={:.1}uA SPI={:.1}uA TMR={:.1}uA PWM={:.1}uA BatMon={:.1}uA Total={:.1}uA\nWallMs={:.1} Speed={:.2}x ({:.1} MIPS) Exact={}",
            mach.cpu.pc,
            mach.cpu.cycles,
            mach.bus.total_time_ns / 1000,
            mach.bus.sys.sys_clk_hz(),
            mach.cpu.instret,
            mach.bus.logic.samples.len(),
            mach.bus.gpio.contention_log.len(),
            mach.bus.unknown_mmio,
            mach.bus.sys.reset_flag & 0x07,
            mach.bus.powered,
            mach.bus.bor_active,
            mach.bus.vdd_mv,
            mach.bus.sleep_skipped_ns / 1000,
            mach.bus.flash_ctrl.interrupted_ops,
            tm[PowerMode::Active as usize] / 1000,
            tm[PowerMode::Idle as usize] / 1000,
            tm[PowerMode::Halt as usize] / 1000,
            tm[PowerMode::Sleep as usize] / 1000,
            tm[PowerMode::Shutdown as usize] / 1000,
            pr.total_avg_ua,
            pr.core_avg_ua,
            pr.adc_avg_ua,
            pr.usbhs_avg_ua,
            pr.ble_rf_avg_ua,
            pr.uart_avg_ua,
            pr.spi_avg_ua,
            pr.tmr_avg_ua,
            pr.pwm_avg_ua,
            pr.bat_mon_avg_ua,
            pr.total_avg_ua,
            wall_ms,
            speed_x,
            mips,
            mach.exact_periph_step,
        );
        for w in &mach.bus.gpio.contention_log {
            println!("  WARN: {w}");
        }
    }
}
