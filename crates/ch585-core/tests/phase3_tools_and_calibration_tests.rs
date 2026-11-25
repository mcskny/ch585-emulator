use ch585_core::ext::PinAnalogModel;
use ch585_core::json::parse_json;
use ch585_core::peripherals::adc::AdcWaveform;
use ch585_core::peripherals::gpio::ExtPinDrive;
use ch585_core::peripherals::irq;
use ch585_core::report::MachineReport;
use ch585_core::soc::Ch585Machine;
use ch585_core::vcd::VcdConfig;

fn root_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

fn write_u32_slice(buf: &mut [u8], off: usize, word: u32) {
    buf[off..off + 4].copy_from_slice(&word.to_le_bytes());
}

#[test]
fn test_timing_model_flash_vs_highcode_branch_load_irq_dma() {
    // Program:
    //   0x00: addi x5, x0, 4       (00400293)
    //   0x04: addi x5, x5, -1      (fff28293)
    //   0x08: bne  x5, x0, -4      (fe029ee3) -> taken 3x, not taken 1x
    //   0x0C: lw   x6, 0(x7)       (0003a303) -> loads from x7 (SRAM or MMIO)
    //   0x10: nop                  (00000013)
    //   0x14: jal  x0, 0           (0000006f) -> tight idle loop
    let prog = [
        0x0040_0293u32,
        0xFFF2_8293,
        0xFE02_9EE3,
        0x0003_A303,
        0x0000_0013,
        0x0000_006F,
    ];

    // 1. Ideal model: Flash == SRAM (1 cycle / insn)
    let mut ideal = Ch585Machine::new();
    for (i, &w) in prog.iter().enumerate() {
        write_u32_slice(&mut ideal.bus.flash, i * 4, w);
    }
    ideal.cpu.regs[7] = 0x2000_0100;
    ideal.run_cycles(11);
    assert_eq!(ideal.cpu.instret, 11);
    assert_eq!(ideal.cpu.cycles, 11);

    // 2. Calibrated timing model: Flash slower than SRAM (__HIGH_CODE), branch/load/MMIO/IRQ/DMA penalties
    let cal_json = r#"{
        "timing": {
            "flash_fetch_wait": 2,
            "sram_fetch_wait": 0,
            "branch_taken_penalty": 1,
            "jump_penalty": 1,
            "load_extra_cycles": 1,
            "store_extra_cycles": 1,
            "mmio_extra_cycles": 2,
            "irq_entry_cycles": 4,
            "irq_exit_cycles": 3,
            "dma_bus_penalty_cycles": 2
        }
    }"#;

    let run_11_insns = |in_sram: bool, load_addr: u32| -> u64 {
        let mut m = Ch585Machine::new();
        m.load_calibration_json(cal_json).unwrap();
        if in_sram {
            for (i, &w) in prog.iter().enumerate() {
                write_u32_slice(&mut m.bus.sram, 0x200 + i * 4, w);
            }
            m.cpu.pc = 0x2000_0200;
        } else {
            for (i, &w) in prog.iter().enumerate() {
                write_u32_slice(&mut m.bus.flash, i * 4, w);
            }
            m.cpu.pc = 0;
        }
        m.cpu.regs[7] = load_addr;
        while m.cpu.instret < 11 {
            m.step();
        }
        m.cpu.cycles
    };

    // Execute same 11 instructions (init + 4 loop iters + lw + nop):
    // In SRAM: 11 base + 3 taken branches (3) + 1 load (1) = 15 cycles
    let cyc_sram_mem = run_11_insns(true, 0x2000_0100);
    assert_eq!(cyc_sram_mem, 15);

    // In Flash (+2 wait state * 11 fetches = +22): 15 + 22 = 37 cycles
    let cyc_flash_mem = run_11_insns(false, 0x2000_0100);
    assert_eq!(cyc_flash_mem, 37);
    assert!(cyc_flash_mem > cyc_sram_mem);

    // SRAM reading MMIO (0x4000_2000) adds mmio_extra_cycles (+2)
    let cyc_sram_mmio = run_11_insns(true, 0x4000_2000);
    assert_eq!(cyc_sram_mmio, 17);

    // Verify IRQ entry/exit + DMA bus penalty
    let mut irq_m = Ch585Machine::new();
    irq_m.load_calibration_json(cal_json).unwrap();
    // Trap handler at 0x100: mret (0x3020_0073)
    write_u32_slice(&mut irq_m.bus.flash, 0x100, 0x3020_0073);
    // Vector table entry 14 (SWI) -> 0x100 in mode 3
    write_u32_slice(&mut irq_m.bus.flash, 14 * 4, 0x100);
    // Main code at 0x2000_0300: j .
    write_u32_slice(&mut irq_m.bus.sram, 0x300, 0x0000_006F);
    irq_m.cpu.pc = 0x2000_0300;
    irq_m.cpu.mtvec = 0x0000_0003; // WCH mode 3
    irq_m.cpu.mstatus |= 1 << 3;   // MIE
    irq_m.bus.pfic.write_u32(0x100, 1 << irq::SWI);
    irq_m.bus.pfic.write_u32(0x200, 1 << irq::SWI); // pend SWI

    // Step 1: enters IRQ -> cost 1 + irq_entry_cycles(4) = 5 cycles (instret stays 0)
    irq_m.step();
    assert_eq!(irq_m.cpu.pc, 0x100);
    assert_eq!(irq_m.cpu.cycles, 4); // extra 4 cycles recorded for entry
    // Step 2: executes mret in Flash -> 1 + flash_wait(2) + irq_exit(3) = 6 cycles
    irq_m.step();
    assert_eq!(irq_m.cpu.pc, 0x2000_0300);
    assert_eq!(irq_m.cpu.cycles, 10);

    // Step 3: enable Timer1 DMA burst and verify dma_bus_penalty_cycles (+2) stalls CPU
    let cyc_before_dma = irq_m.cpu.cycles;
    write_u32_slice(&mut irq_m.bus.sram, 0x400, 22);
    write_u32_slice(&mut irq_m.bus.sram, 0x404, 50);
    irq_m.bus.timers[1].write_u32(0x0C, 78);
    irq_m.bus.timers[1].write_u32(0x18, 0x2000_0400);
    irq_m.bus.timers[1].write_u32(0x1C, 0x2000_0408);
    irq_m.bus.timers[1].write_u8(0x01, 0x01); // RB_TMR_DMA_EN
    irq_m.bus.timers[1].write_u8(0x00, 0x0C); // count + PWM out
    irq_m.step(); // runs `j .` in SRAM (1 + jump_penalty(1) = 2) + 2 DMA stall = 4
    assert_eq!(irq_m.cpu.cycles - cyc_before_dma, 4);
}

#[test]
fn test_adc_calibration_curve_offset_gain_and_rc_settling() {
    let mut mach = Ch585Machine::new();
    let cal_json = r#"{
        "adc": {
            "settling": {
                "enabled": true,
                "sample_cap_pf": 10.0,
                "internal_r_ohm": 1000.0,
                "base_sample_ns": 100.0
            },
            "entries": [
                {
                    "channel": 0,
                    "gain": "0dB",
                    "offset_mv": 10.0,
                    "offset_counts": 5.0,
                    "gain_scale": 1.0,
                    "curve_mv_to_counts": [[0, 0], [500, 1000], [1000, 2200], [2000, 4000]]
                },
                {
                    "channel": 1,
                    "gain": "-6dB",
                    "offset_mv": 0.0,
                    "offset_counts": 0.0,
                    "gain_scale": 1.0
                }
            ]
        }
    }"#;
    mach.load_calibration_json(cal_json).unwrap();
    write_u32_slice(&mut mach.bus.flash, 0, 0x0000_006F); // j .

    // Part A: piecewise linear curve + offset_mv + offset_counts on ch0 (0dB, low R_src=0 -> ~10ns tau -> ~100% settled)
    mach.bus.adc.channels[0] = AdcWaveform::Dc(740); // +10 offset_mv -> 750mV -> interp(500..1000)=1600 + 5 = 1605
    mach.bus.adc.write_u8(0x58, 0); // ch0
    mach.bus.adc.write_u8(0x59, 0x01 | (2 << 4)); // ON, GA=2 (0dB), div=0
    mach.bus.adc.write_u8(0x5A, 0x01); // start
    mach.run_cycles(40);
    assert!((mach.bus.adc.data as i32 - 1605).abs() <= 2, "data={}", mach.bus.adc.data);

    // Part B: high source impedance on PA5 (ch1) causes partial RC settling from hold_cap (~740mV) toward 3000mV
    mach.set_pin_analog_model(
        'A',
        5,
        PinAnalogModel {
            source_ohm: 50_000,
            load_ohm: None,
            load_to_vdd: false,
            ext_pull_ohm: 100_000,
        },
    );
    mach.bus.adc.channels[1] = AdcWaveform::Dc(3000);
    mach.bus.adc.write_u8(0x58, 1); // ch1
    mach.bus.adc.write_u8(0x59, 0x01 | (1 << 4)); // ON, GA=1 (-6dB), shortest sample time (div=0)
    mach.bus.adc.write_u8(0x5A, 0x01);
    mach.run_cycles(40);
    let first_sample = mach.bus.adc.data;

    // Repeated conversions with longest sample time (div=3) converge upward toward settled 3000mV (~3950 counts)
    mach.bus.adc.write_u8(0x59, 0x01 | (1 << 4) | (3 << 6));
    for _ in 0..12 {
        mach.bus.adc.write_u8(0x5A, 0x01);
        mach.run_cycles(60);
    }
    let settled_sample = mach.bus.adc.data;
    assert!(
        first_sample < 2200 && settled_sample > first_sample + 1200,
        "Expected RC ramp from first={first_sample} to settled={settled_sample}"
    );
}

#[test]
fn test_current_table_json_override_and_power_report() {
    let mut mach = Ch585Machine::new();
    write_u32_slice(&mut mach.bus.flash, 0, 0x0000_006F);
    mach.load_calibration_json(r#"{ "current": { "active_ldo_ua": 7500.0, "adc_ua": 600.0 } }"#)
        .unwrap();
    mach.bus.adc.write_u8(0x59, 0x01); // power on ADC
    mach.run_us(10_000);

    let rep = mach.bus.power_report();
    assert!((rep.core_avg_ua - 7500.0).abs() < 5.0, "core={}", rep.core_avg_ua);
    assert!((rep.adc_avg_ua - 600.0).abs() < 5.0, "adc={}", rep.adc_avg_ua);
    assert!((rep.total_avg_ua - 8100.0).abs() < 20.0, "total={}", rep.total_avg_ua);
}

#[test]
fn test_vcd_export_pins_irq_dma_usb() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/uygulama.elf")) else {
        eprintln!("Skipping test_vcd_export_pins_irq_dma_usb: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();
    mach.set_pin_drive('B', 9, ExtPinDrive::High);
    mach.enable_vcd_trace(65_536);

    mach.run_us(80_000);
    // Toggle external pin PB9 (VBUS, digital GPIO input) late in trace
    mach.set_pin_drive('B', 9, ExtPinDrive::Low);
    mach.run_us(5_000);
    mach.set_pin_drive('B', 9, ExtPinDrive::High);
    mach.run_us(5_000);

    let vcd = mach.export_vcd(&VcdConfig {
        pins: vec![('B', 9)],
        include_internal: true,
    });

    assert!(vcd.contains("$timescale 1ns $end"));
    assert!(vcd.contains("$scope module gpio $end"));
    assert!(vcd.contains("$var wire 1 ! PB9 $end"));
    assert!(vcd.contains("$scope module irq $end"));
    assert!(vcd.contains("$scope module dma $end"));
    assert!(vcd.contains("$scope module usb $end"));
    assert!(vcd.contains("$enddefinitions $end"));
    assert!(vcd.contains("#0\n$dumpvars\n"));
    assert!(vcd.contains("1!\n") && vcd.contains("0!\n"));
    // Verify strictly non-decreasing timestamps
    let mut prev_t = 0u64;
    for line in vcd.lines() {
        if let Some(rest) = line.strip_prefix('#') {
            let t: u64 = rest.parse().unwrap();
            assert!(t >= prev_t, "Non-monotonic VCD timestamp {t} < {prev_t}");
            prev_t = t;
        }
    }
    assert!(prev_t > 0);
}

#[test]
fn test_profiler_elf_symbols_isr_max_and_stack_overflow() {
    // 1. Real ELF symbol & stack depth check on uygulama.elf (when local fixture exists)
    if let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/uygulama.elf")) {
        let mut mach = Ch585Machine::new();
        mach.load_elf(&elf).unwrap();
        mach.set_pin_drive('B', 9, ExtPinDrive::High);
        mach.enable_profiler(true);
        mach.run_us(60_000);

        let rep = mach.profiler_report();
        assert!(rep.total_cycles > 100_000);
        assert!(!rep.functions.is_empty());
        assert!(
            rep.functions.iter().any(|f| !f.name.starts_with("0x")),
            "Expected resolved ELF symbol names in profiler: {:?}",
            rep.functions.iter().take(5).map(|f| &f.name).collect::<Vec<_>>()
        );
        assert!(!rep.stack.overflow);
        assert!(rep.stack.min_sp >= rep.stack.limit && rep.stack.min_sp < rep.stack.top);
    } else {
        eprintln!("Skipping Part 1 of test_profiler_elf_symbols_isr_max_and_stack_overflow: optional ELF fixture not found");
    }

    // 2. Synthetic ISR latency + stack overflow trigger test
    let mut syn = Ch585Machine::new();
    // 0x00: li sp, 0x20020000 (lui sp, 0x20020 = 20020137)
    // 0x04: addi sp, sp, -512 (e0010113) -> drops SP below 0x2001FF00 limit!
    // 0x08: nop (00000013)
    // 0x0C: j . (0000006f)
    // 0x100 (ISR): addi sp, sp, -64 (fc010113); nop; nop; mret (30200073)
    write_u32_slice(&mut syn.bus.flash, 0x00, 0x2002_0137);
    write_u32_slice(&mut syn.bus.flash, 0x04, 0xE001_0113);
    write_u32_slice(&mut syn.bus.flash, 0x08, 0x0000_0013);
    write_u32_slice(&mut syn.bus.flash, 0x0C, 0x0000_006F);
    write_u32_slice(&mut syn.bus.flash, 0x100, 0xFC01_0113);
    write_u32_slice(&mut syn.bus.flash, 0x104, 0x0000_0013);
    write_u32_slice(&mut syn.bus.flash, 0x108, 0x0401_0113); // restore sp
    write_u32_slice(&mut syn.bus.flash, 0x10C, 0x3020_0073); // mret
    write_u32_slice(&mut syn.bus.flash, 14 * 4, 0x100);      // SWI vector

    syn.enable_profiler(true);
    syn.set_stack_bounds(0x2002_0000, 0x2001_FF00);
    syn.cpu.mtvec = 0x0000_0003;
    syn.cpu.mstatus |= 1 << 3;
    syn.run_cycles(6);
    syn.bus.pfic.write_u32(0x100, 1 << irq::SWI);
    syn.bus.pfic.write_u32(0x200, 1 << irq::SWI);
    syn.run_cycles(20);

    let srep = syn.profiler_report();
    assert!(srep.stack.overflow, "Expected stack overflow warning");
    assert!(!srep.stack.overflow_events.is_empty());
    assert!(srep.longest_isr.is_some());
    let longest = srep.longest_isr.unwrap();
    assert_eq!(longest.irqn, irq::SWI);
    assert!(longest.cycles >= 4);
    assert_eq!(srep.isrs.len(), 1);
}

#[test]
fn test_json_report_parse_roundtrip_and_deterministic_diff() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/uygulama.elf")) else {
        eprintln!("Skipping test_json_report_parse_roundtrip_and_deterministic_diff: optional ELF fixture not found");
        return;
    };

    let run_once = || -> String {
        let mut m = Ch585Machine::new();
        m.load_elf(&elf).unwrap();
        m.set_pin_drive('B', 9, ExtPinDrive::High);
        m.enable_profiler(true);
        m.run_us(50_000);
        MachineReport::from_machine(&m, Some(2), false).to_diff_json_string()
    };

    let j1 = run_once();
    let j2 = run_once();
    assert_eq!(j1, j2, "Differential JSON output must be byte-identical across runs");

    let parsed = parse_json(&j1).expect("Valid JSON");
    assert_eq!(parsed.get("schema_version").and_then(|v| v.as_u32()), Some(1));
    assert_eq!(
        parsed
            .get("usbhs")
            .and_then(|u| u.get("enumerated"))
            .and_then(|v| v.as_bool()),
        Some(true)
    );
    let uart_txt = parsed
        .get("uart")
        .and_then(|a| a.as_array())
        .and_then(|arr| arr.first())
        .and_then(|u| u.get("tx_text"))
        .and_then(|s| s.as_str())
        .unwrap_or("");
    assert!(uart_txt.contains("Macropad v2"));
    assert_eq!(
        parsed
            .get("unknown_mmio")
            .and_then(|a| a.as_array())
            .map(|a| a.len()),
        Some(0)
    );
    assert!(parsed
        .get("profile")
        .and_then(|p| p.get("functions"))
        .and_then(|a| a.as_array())
        .is_some_and(|a| !a.is_empty()));
}
