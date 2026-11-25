use ch585_core::peripherals::adc::AdcWaveform;
use ch585_core::peripherals::gpio::ExtPinDrive;
use ch585_core::soc::Ch585Machine;

fn root_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

fn read_u32_sram(mach: &Ch585Machine, vaddr: u32) -> u32 {
    let idx = (vaddr - 0x2000_0000) as usize;
    u32::from_le_bytes([
        mach.bus.sram[idx],
        mach.bus.sram[idx + 1],
        mach.bus.sram[idx + 2],
        mach.bus.sram[idx + 3],
    ])
}

#[test]
fn test_sdk_uart1_tx_and_rx_echo() {
    let Ok(elf) = std::fs::read(root_dir().join("build_tests/uart1_exam.elf")) else {
        eprintln!("Skipping test_sdk_uart1_tx_and_rx_echo: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();

    mach.run_cycles(300_000);
    let tx = String::from_utf8_lossy(&mach.bus.uarts[1].tx_log).to_string();
    assert!(tx.contains("This is a tx exam"), "UART1 TX was: {tx:?}");

    mach.bus.uarts[1].tx_log.clear();
    mach.bus.uarts[1].inject_rx(b"HELLO");
    mach.run_cycles(100_000);
    assert_eq!(&mach.bus.uarts[1].tx_log, b"HELLO");
    assert!(mach.bus.unknown_mmio.is_empty());
}

#[test]
fn test_sdk_adc_exam_all_modes() {
    let Ok(elf) = std::fs::read(root_dir().join("build_tests/adc_exam.elf")) else {
        eprintln!("Skipping test_sdk_adc_exam_all_modes: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();
    mach.bus.adc.channels[0] = AdcWaveform::Dc(1650);
    mach.bus.adc.channels[1] = AdcWaveform::Dc(825);

    mach.run_cycles(5_000_000);
    let out = String::from_utf8_lossy(&mach.bus.uarts[0].tx_log).to_string();
    assert!(out.contains("Start @ChipID=93"), "out={out}");
    assert!(out.contains("1.Temperature sampling...\n25 2048"));
    assert!(out.contains("RoughCalib_Value =0"));
    assert!(out.contains("3.Single channel DMA sampling...\n3218"));
    assert!(out.contains("4.Diff channel sampling...\n2048"));
    assert!(out.contains("6.Single channel sampling in interrupt mode...\n1609"));
    assert!(mach.bus.unknown_mmio.is_empty());
}

#[test]
fn test_sdk_tmr_exam_pwm_and_timer0_toggle() {
    let Ok(elf) = std::fs::read(root_dir().join("build_tests/tmr_exam.elf")) else {
        eprintln!("Skipping test_sdk_tmr_exam_pwm_and_timer0_toggle: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();

    // Run 250ms so TMR0 (100ms) fires at least twice and toggles PB15
    mach.run_us(250_000);
    let out = String::from_utf8_lossy(&mach.bus.uarts[0].tx_log).to_string();
    assert!(out.contains("Start @ChipID=93"));

    // Verify PB15 (bit 15) toggled in logic analyzer
    let pb15_toggles = mach
        .bus
        .logic
        .samples
        .windows(2)
        .filter(|w| ((w[0].pb ^ w[1].pb) & (1 << 15)) != 0)
        .count();
    assert!(pb15_toggles >= 2, "Expected >=2 PB15 toggles, got {pb15_toggles}");

    // Verify PB22 (bit 22) TMR3 100us PWM toggled many times
    let pb22_toggles = mach
        .bus
        .logic
        .samples
        .windows(2)
        .filter(|w| ((w[0].pb ^ w[1].pb) & (1 << 22)) != 0)
        .count();
    assert!(pb22_toggles >= 200, "Expected PWM toggles on PB22, got {pb22_toggles}");
}

#[test]
fn test_periph_smoke_gpio_irq_spi_pwm_adc_scan_rtc() {
    let Ok(elf) = std::fs::read(root_dir().join("build_tests/periph_smoke.elf")) else {
        eprintln!("Skipping test_periph_smoke_gpio_irq_spi_pwm_adc_scan_rtc: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();
    mach.bus.adc.channels[0] = AdcWaveform::Dc(0);
    mach.bus.adc.channels[1] = AdcWaveform::Dc(1050);
    mach.bus.adc.channels[2] = AdcWaveform::Dc(2100);
    mach.bus.adc.channels[3] = AdcWaveform::Dc(3150);

    // Run 10 ms first
    mach.run_us(10_000);
    let out = String::from_utf8_lossy(&mach.bus.uarts[0].tx_log).to_string();
    assert!(out.contains("SMOKE OK scan=1024,2048,3072,4095"), "out={out}");
    assert_eq!(&mach.bus.spis[0].tx_log, &[0xAA, 0x55, 0x93]);

    // Check TMR0 IRQ counter incremented (> 40 times in 10ms @ 160us period)
    let tmr_addr = *mach.addr_by_sym.get("g_tmr0_irqs").unwrap();
    assert!(read_u32_sram(&mach, tmr_addr) > 40);

    // Trigger PA0 falling edge (pull-up was high, drive Low) -> toggles PB0
    let pb0_before = (mach.bus.gpio.effective_pb_pin() & 1) != 0;
    mach.set_pin_drive('A', 0, ExtPinDrive::Low);
    mach.run_cycles(500);
    let gpioa_addr = *mach.addr_by_sym.get("g_gpioa_irqs").unwrap();
    assert_eq!(read_u32_sram(&mach, gpioa_addr), 1);
    let pb0_after = (mach.bus.gpio.effective_pb_pin() & 1) != 0;
    assert_ne!(pb0_before, pb0_after);

    // Trigger PB22 remapped INTX falling edge
    mach.set_pin_drive('B', 22, ExtPinDrive::Low);
    mach.run_cycles(500);
    let gpiob_addr = *mach.addr_by_sym.get("g_gpiob_irqs").unwrap();
    assert_eq!(read_u32_sram(&mach, gpiob_addr), 1);

    // Verify PWM4 output on PA12 toggled
    let pa12_toggles = mach
        .bus
        .logic
        .samples
        .windows(2)
        .filter(|w| ((w[0].pa ^ w[1].pa) & (1 << 12)) != 0)
        .count();
    assert!(pa12_toggles > 20);

    // Run 140 ms more -> RTC 0.125s timer IRQ must fire!
    mach.run_us(140_000);
    let rtc_addr = *mach.addr_by_sym.get("g_rtc_irqs").unwrap();
    assert!(read_u32_sram(&mach, rtc_addr) >= 1);
}
