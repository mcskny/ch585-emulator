use ch585_core::peripherals::gpio::{ExtPinDrive, ExtPull, Gpio};
use ch585_core::peripherals::irq;
use ch585_core::pfic::Pfic;
use ch585_core::soc::Ch585Machine;

fn root_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

#[test]
fn test_gpio_bitmask_and_pin_level_equivalence() {
    let mut gpio = Gpio::new();
    let mut pfic = Pfic::new();

    // Configure mixed MCU outputs, pulls, peripheral overrides, external drives/pulls
    gpio.pa_dir = 0x00FF;
    gpio.pa_out = 0x00AA;
    gpio.pa_pu = 0x5500;
    gpio.pa_pd_drv = 0x0F00;

    gpio.pb_dir = 0x000F_00FF;
    gpio.pb_out = 0x000A_0055;
    gpio.pb_pu = 0x0030_FF00;
    gpio.pb_pd_drv = 0x0010_0F00;

    gpio.pin_in_dis = (1 << 12) | (1 << (16 + 15));
    gpio.set_periph_out(1 << 4, 1 << 4, 1 << 10, 1 << 10);

    for i in 0..16u8 {
        match i % 4 {
            0 => gpio.set_ext_drive('A', i, ExtPinDrive::High),
            1 => gpio.set_ext_drive('A', i, ExtPinDrive::Low),
            2 => gpio.set_ext_drive('A', i, ExtPinDrive::AnalogMv(2000)),
            _ => gpio.set_ext_drive('A', i, ExtPinDrive::Floating),
        }
        gpio.set_ext_pull(
            'A',
            i,
            if i % 3 == 0 {
                ExtPull::PullUp
            } else if i % 3 == 1 {
                ExtPull::PullDown
            } else {
                ExtPull::None
            },
        );
    }
    for i in 0..24u8 {
        match i % 5 {
            0 => gpio.set_ext_drive('B', i, ExtPinDrive::High),
            1 => gpio.set_ext_drive('B', i, ExtPinDrive::Low),
            2 => gpio.set_ext_drive('B', i, ExtPinDrive::AnalogMv(900)),
            _ => gpio.set_ext_drive('B', i, ExtPinDrive::Floating),
        }
        gpio.set_ext_pull(
            'B',
            i,
            if i % 2 == 0 {
                ExtPull::PullUp
            } else {
                ExtPull::None
            },
        );
    }

    let pa_eff = gpio.effective_pa_pin();
    let pb_eff = gpio.effective_pb_pin();
    for i in 0..16u8 {
        assert_eq!(
            gpio.pin_level('A', i),
            ((pa_eff >> i) & 1) != 0,
            "PA{i} mismatch"
        );
    }
    for i in 0..24u8 {
        assert_eq!(
            gpio.pin_level('B', i),
            ((pb_eff >> i) & 1) != 0,
            "PB{i} mismatch"
        );
    }
    assert!(!gpio.pin_level('A', 16));
    assert!(!gpio.pin_level('B', 24));
    assert!(!gpio.pin_level('C', 0));

    // Verify rebuild_ext_masks matches incremental external drive updates
    let v_before = gpio.version;
    gpio.rebuild_ext_masks();
    assert_eq!(gpio.version, v_before);

    // Contention: PA0 is MCU dir=1, out=0 while ext_pa[0]=High
    gpio.check_contention();
    assert!(
        gpio.contention_log.iter().any(|s| s.contains("PA0")),
        "Expected contention on PA0: {:?}",
        gpio.contention_log
    );

    // Edge & level interrupt evaluation
    gpio.pa_int_mode = 1 << 11; // rising-edge on PA11
    gpio.pa_out |= 1 << 11;
    gpio.pa_int_en = 1 << 11;
    gpio.pb_int_mode = 0; // active-high level on PB10
    gpio.pb_out |= 1 << 10;
    gpio.pb_int_en = 1 << 10; // PB10 is forced HIGH by periph_out
    pfic.write_u32(0x100, (1 << irq::GPIO_A) | (1 << irq::GPIO_B));
    gpio.evaluate_interrupts(&mut pfic);
    assert!(gpio.has_level_int_enabled());
    assert!(pfic.is_pending(irq::GPIO_B));

    // Toggle PA11 Low -> High via external drive after edge arm
    gpio.pa_dir &= !(1 << 11);
    gpio.pa_pu &= !(1 << 11);
    gpio.pa_pd_drv &= !(1 << 11);
    gpio.set_ext_drive('A', 11, ExtPinDrive::Low);
    gpio.evaluate_interrupts(&mut pfic);
    gpio.set_ext_drive('A', 11, ExtPinDrive::High);
    gpio.evaluate_interrupts(&mut pfic);
    assert_ne!(gpio.pa_int_if & (1 << 11), 0);
    assert!(pfic.is_pending(irq::GPIO_A));
}

#[test]
fn test_batched_vs_exact_equivalence_on_uygulama() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/uygulama.elf")) else {
        eprintln!("Skipping test_batched_vs_exact_equivalence_on_uygulama: optional ELF fixture not found");
        return;
    };

    let mut batched = Ch585Machine::new();
    batched.load_elf(&elf).unwrap();
    batched.set_pin_drive('B', 9, ExtPinDrive::High);
    batched.exact_periph_step = false;
    batched.periph_batch_cycles = 256;
    batched.run_us(100_000);

    let mut exact = Ch585Machine::new();
    exact.load_elf(&elf).unwrap();
    exact.set_pin_drive('B', 9, ExtPinDrive::High);
    exact.exact_periph_step = true;
    exact.run_us(100_000);

    let uart_batched = String::from_utf8_lossy(&batched.bus.uarts[2].tx_log).to_string();
    let uart_exact = String::from_utf8_lossy(&exact.bus.uarts[2].tx_log).to_string();
    assert_eq!(uart_batched, uart_exact);
    assert!(uart_batched.contains("Macropad v2"));
    assert!(uart_batched.contains("SYSCLK=62400000"));
    assert!(uart_batched.contains("REST_CAL"));

    assert!(batched.bus.unknown_mmio.is_empty());
    assert!(exact.bus.unknown_mmio.is_empty());
    assert_eq!(batched.bus.gpio.contention_log.len(), 0);
    assert_eq!(exact.bus.gpio.contention_log.len(), 0);

    let time_diff = batched.bus.total_time_ns.abs_diff(exact.bus.total_time_ns);
    assert!(
        (time_diff as f64) / (exact.bus.total_time_ns as f64) < 0.001,
        "Time drift too high: batched={} exact={}",
        batched.bus.total_time_ns,
        exact.bus.total_time_ns
    );

    let cur_batched = batched.bus.estimated_avg_current_ua();
    let cur_exact = exact.bus.estimated_avg_current_ua();
    assert!(
        (cur_batched - cur_exact).abs() / cur_exact.max(1.0) < 0.05,
        "Current mismatch: batched={cur_batched} exact={cur_exact}"
    );
}

#[cfg(not(debug_assertions))]
#[test]
fn test_realtime_speed_exceeds_1x_release() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/uygulama.elf")) else {
        eprintln!("Skipping test_realtime_speed_exceeds_1x_release: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();
    mach.set_pin_drive('B', 9, ExtPinDrive::High);

    let t0 = std::time::Instant::now();
    mach.run_us(500_000);
    let elapsed = t0.elapsed().as_secs_f64();
    let ratio = 0.5 / elapsed;
    assert!(
        ratio >= 1.0,
        "Expected >= 1.0x realtime speed in release build, got {ratio:.2}x ({elapsed:.3}s for 500ms)"
    );
}
