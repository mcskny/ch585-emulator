use std::cell::RefCell;
use std::rc::Rc;

use ch585_core::ext::{ExternalDevice, PinAnalogModel, PinChangeEvent, PinContext};
use ch585_core::peripherals::adc::AdcWaveform;
use ch585_core::peripherals::gpio::{ExtPinDrive, ExtPull, PinElectricState};
use ch585_core::peripherals::irq;
use ch585_core::peripherals::sys::PowerMode;
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
fn test_usb_auto_detach_and_re_enumerate_on_vbus_cycle_without_manual_trigger() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/uygulama.elf")) else {
        eprintln!("Skipping test_usb_auto_detach_and_re_enumerate_on_vbus_cycle_without_manual_trigger: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();
    mach.set_pin_drive('B', 9, ExtPinDrive::High);
    for ch in 0..14 {
        mach.bus.adc.channels[ch] = AdcWaveform::Dc(1650);
    }

    mach.run_us(120_000);
    assert!(mach.bus.usbhs.enumerated);

    // Unplug VBUS -> firmware calls usb_hs_hid_deinit -> host auto-detaches
    mach.set_pin_drive('B', 9, ExtPinDrive::Low);
    mach.run_us(80_000);
    assert!(
        !mach.bus.usbhs.enumerated,
        "Expected enumerated=false after VBUS drop"
    );

    // Plug VBUS back -> host automatically detects pull-up, issues Bus Reset, and re-enumerates
    mach.bus.usbhs.in_packets.clear();
    mach.set_pin_drive('B', 9, ExtPinDrive::High);
    mach.run_us(120_000);
    assert!(
        mach.bus.usbhs.enumerated,
        "Expected automatic re-enumeration without calling trigger_enumerate()"
    );

    // Key press after auto re-enumeration works on EP1 IN
    mach.bus.usbhs.in_packets.clear();
    mach.bus.adc.channels[0] = AdcWaveform::Dc(2800);
    mach.run_us(30_000);
    assert!(
        mach.bus
            .usbhs
            .in_packets
            .iter()
            .any(|p| p.ep == 1 && p.is_in && p.data.iter().any(|&b| b != 0)),
        "Expected EP1 IN report after automatic USB re-enumeration"
    );
}

#[test]
fn test_usb_detach_attach_suspend_and_remote_wakeup() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/usb_hid.elf")) else {
        eprintln!("Skipping test_usb_detach_attach_suspend_and_remote_wakeup: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();

    mach.run_cycles(500_000);
    assert!(mach.bus.usbhs.enumerated);

    mach.bus.usbhs.detach();
    assert!(!mach.bus.usbhs.enumerated);
    assert!(!mach.bus.usbhs.host_attached);

    mach.bus.usbhs.attach();
    mach.run_cycles(500_000);
    assert!(mach.bus.usbhs.enumerated);

    mach.bus.usbhs.suspend();
    assert!(mach.bus.usbhs.suspended);
    mach.bus.usbhs.write_u8(0x04, 0x01); // USBHS_UD_UD_REMOTE_WKUP
    assert!(!mach.bus.usbhs.suspended);
    assert_eq!(mach.bus.usbhs.remote_wakeup_count, 1);
    assert_eq!(mach.bus.usbhs.wake_ctrl & 0x01, 0);
}

#[test]
fn test_gpio_external_pulls_and_drive_contention() {
    let mut mach = Ch585Machine::new();
    mach.set_pin_pull('B', 7, ExtPull::PullDown);
    mach.set_pin_pull('B', 0, ExtPull::PullUp);

    assert_eq!(mach.bus.gpio.pb_electric(7), PinElectricState::Low);
    assert_eq!(mach.bus.gpio.pb_electric(0), PinElectricState::High);
    assert_eq!(mach.bus.gpio.effective_pb_pin() & (1 << 7), 0);
    assert_ne!(mach.bus.gpio.effective_pb_pin() & (1 << 0), 0);

    // Create strong drive contention: MCU drives PB7 HIGH while external drives PB7 LOW
    mach.bus.gpio.pb_dir |= 1 << 7;
    mach.bus.gpio.pb_out |= 1 << 7;
    mach.set_pin_drive('B', 7, ExtPinDrive::Low);
    mach.bus.step_peripherals(1);

    assert_eq!(mach.bus.gpio.contention_log.len(), 1);
    assert!(mach.bus.gpio.contention_log[0].contains("PB7"));
}

#[test]
fn test_adc_deterministic_noise() {
    let mut mach1 = Ch585Machine::new();
    let mut mach2 = Ch585Machine::new();

    for m in [&mut mach1, &mut mach2] {
        m.bus.adc.channels[0] = AdcWaveform::Dc(1650);
        m.bus.adc.set_noise_seed(0x1234_5678);
        m.bus.adc.noise_mv[0] = 30;
        m.bus.adc.write_u8(0x58, 0); // CH0
        m.bus.adc.write_u8(0x59, 0x11); // Power on + PGA=-6dB
    }

    let mut samples1 = Vec::new();
    let mut samples2 = Vec::new();
    for _ in 0..8 {
        mach1.bus.adc.write_u8(0x5A, 0x01); // Start
        mach2.bus.adc.write_u8(0x5A, 0x01);
        for _ in 0..40 {
            mach1.bus.step_peripherals(1);
            mach2.bus.step_peripherals(1);
        }
        samples1.push(mach1.bus.adc.data);
        samples2.push(mach2.bus.adc.data);
    }

    assert_eq!(samples1, samples2, "Seeded ADC noise should be deterministic");
    let min_v = *samples1.iter().min().unwrap();
    let max_v = *samples1.iter().max().unwrap();
    assert!(max_v > min_v, "Expected ADC noise spread across samples: {samples1:?}");
    // Nominal -6dB at 1650mV is 2633; +/-30mV is ~ +/-29 counts
    assert!(min_v >= 2590 && max_v <= 2675, "Out of +/-30mV bounds: {samples1:?}");
}

#[test]
fn test_lse_failure_freezes_ck32k_pin_and_rtc() {
    let mut mach = Ch585Machine::new();
    // Unlock safe access and switch 32k clock to LSE (bit2 = RB_CLK_OSC32K_XT, bit0 = XT32K_PON)
    mach.bus.sys.write_u8(0x40, 0x57);
    mach.bus.sys.write_u8(0x40, 0xA8);
    mach.bus.sys.write_u8(0x2F, 0x05);

    // Verify pin toggles & RTC counts when LSE is healthy
    let mut saw_high = false;
    let mut saw_low = false;
    for _ in 0..4000 {
        mach.bus.step_peripherals(100);
        if (mach.bus.sys.read_u8(0x2F, mach.bus.total_cycles) & 0x80) != 0 {
            saw_high = true;
        } else {
            saw_low = true;
        }
    }
    assert!(saw_high && saw_low, "Healthy LSE should toggle RB_32K_CLK_PIN");
    let rtc_before_fail = mach.bus.rtc.cnt_32k;
    assert!(rtc_before_fail > 0);

    // Inject LSE failure -> pin freezes 0 and RTC stops counting
    mach.bus.sys.lse_failed = true;
    let mut any_high = false;
    for _ in 0..2000 {
        mach.bus.step_peripherals(100);
        if (mach.bus.sys.read_u8(0x2F, mach.bus.total_cycles) & 0x80) != 0 {
            any_high = true;
        }
    }
    assert!(!any_high, "Failed LSE must stop RB_32K_CLK_PIN");
    assert_eq!(mach.bus.rtc.cnt_32k, rtc_before_fail, "Failed LSE must stop RTC");
}

#[test]
fn test_low_power_sleep_mode_wakeup_and_current_estimation() {
    let mut mach = Ch585Machine::new();
    mach.cpu.pc = 0;
    // Place `wfi` (0x10500073) followed by `j .` (0x0000006f) in Flash at 0x0
    mach.bus.flash[0..4].copy_from_slice(&0x1050_0073u32.to_le_bytes());
    mach.bus.flash[4..8].copy_from_slice(&0x0000_006Fu32.to_le_bytes());
    mach.cpu.mtvec = 0; // direct mode trap to 0

    // Setup PB0 falling-edge wakeup + GPIO_B IRQ enabled in PFIC
    mach.set_pin_drive('B', 0, ExtPinDrive::High);
    mach.bus.step_peripherals(1);
    mach.bus.gpio.pb_out &= !(1 << 0);
    mach.bus.gpio.pb_int_mode |= 1 << 0;
    mach.bus.gpio.pb_int_en |= 1 << 0;
    mach.bus.pfic.write_u32(0x100, 1 << irq::GPIO_B);

    // Enter LowPower_Sleep: SCTLR SLEEPDEEP (bit 2), POWER_PLAN = RB_PWR_PLAN_EN | RB_PWR_CORE | RB_PWR_RAM32K
    mach.bus.pfic.sctlr |= 1 << 2;
    mach.bus.sys.power_plan = 0x8000 | 0x04 | 0x02;
    mach.bus.sys.slp_wake_ctrl = 0x10; // RB_SLP_GPIO_WAKE

    // Step past `wfi` and stay asleep for 50k cycles
    mach.run_cycles(50_000);
    assert!(mach.cpu.halted_wfi);
    assert_eq!(mach.bus.power_mode, PowerMode::Sleep);
    assert!(mach.bus.time_ns_by_mode[PowerMode::Sleep as usize] > 0);
    assert!(
        mach.bus.estimated_avg_current_ua() < 15.0,
        "Expected sleep avg uA < 15uA, got {}",
        mach.bus.estimated_avg_current_ua()
    );

    // Wake via PB0 falling edge
    mach.set_pin_drive('B', 0, ExtPinDrive::Low);
    mach.run_cycles(10);
    assert!(!mach.cpu.halted_wfi);
    assert_eq!(mach.bus.power_mode, PowerMode::Active);
    assert_eq!(mach.bus.sys.power_plan & 0x8000, 0, "RB_PWR_PLAN_EN should auto-clear on Sleep wake");
}

#[test]
fn test_ext_pin_change_callback_and_bitbang_response() {
    let mut mach = Ch585Machine::new();
    let log: Rc<RefCell<Vec<PinChangeEvent>>> = Rc::new(RefCell::new(Vec::new()));
    let log_cb = Rc::clone(&log);

    mach.on_pin_change(move |ev, ctx| {
        if ev.port == 'B' && ev.idx == 6 {
            log_cb.borrow_mut().push(ev);
            // Invert PB6 onto PB3 immediately inside callback (simulating external logic/shift reg)
            ctx.set_pin_drive(
                'B',
                3,
                if ev.level {
                    ExtPinDrive::Low
                } else {
                    ExtPinDrive::High
                },
            );
        }
    });

    // Enable 20mA output HIGH on PB6 and check timestamp, mode flags, and loopback response on PB3
    mach.bus.gpio.pb_dir |= 1 << 6;
    mach.bus.gpio.pb_pd_drv |= 1 << 6;
    mach.bus.gpio.pb_out |= 1 << 6;
    mach.bus.step_peripherals(5);
    mach.poll_ext();

    let events = log.borrow();
    assert_eq!(events.len(), 1);
    let ev = events[0];
    assert_eq!(ev.port, 'B');
    assert_eq!(ev.idx, 6);
    assert!(ev.level);
    assert!(ev.is_output);
    assert!(ev.output_val);
    assert!(ev.high_drive_20ma);
    assert_eq!(ev.time_ns, mach.bus.total_time_ns);
    assert_eq!(mach.bus.gpio.effective_pb_pin() & (1 << 3), 0);

    drop(events);
    mach.bus.gpio.pb_out &= !(1 << 6);
    mach.bus.step_peripherals(5);
    mach.poll_ext();
    assert_ne!(mach.bus.gpio.effective_pb_pin() & (1 << 3), 0);
}

#[test]
fn test_ext_timer_callback_and_rescheduling() {
    struct PulseTrainDev {
        toggles: Rc<RefCell<Vec<u64>>>,
        state: bool,
    }
    impl ExternalDevice for PulseTrainDev {
        fn on_attach(&mut self, ctx: &mut PinContext<'_>) {
            ctx.schedule_after_us(100, 1);
        }
        fn on_timer(&mut self, token: u64, ctx: &mut PinContext<'_>) {
            if token == 1 {
                self.state = !self.state;
                self.toggles.borrow_mut().push(ctx.time_ns());
                ctx.set_pin_drive(
                    'A',
                    0,
                    if self.state {
                        ExtPinDrive::High
                    } else {
                        ExtPinDrive::Low
                    },
                );
                if self.toggles.borrow().len() < 3 {
                    ctx.schedule_after_us(100, 1);
                }
            }
        }
    }

    let mut mach = Ch585Machine::new();
    // Put `j .` in flash and run at 62.4MHz
    mach.bus.flash[0..4].copy_from_slice(&0x0000_006Fu32.to_le_bytes());
    let toggles = Rc::new(RefCell::new(Vec::new()));
    mach.attach_device(Box::new(PulseTrainDev {
        toggles: Rc::clone(&toggles),
        state: false,
    }));

    assert_eq!(mach.next_ext_event_ns(), Some(100_000));
    mach.run_us(350);

    let fired = toggles.borrow();
    assert_eq!(fired.len(), 3);
    assert_eq!(*fired, vec![100_000, 200_000, 300_000]);
    assert_ne!(mach.bus.gpio.effective_pa_pin() & 1, 0);
}

#[test]
fn test_pin_analog_model_drive_strength_5ma_vs_20ma() {
    let mut mach = Ch585Machine::new();
    // Attach a 100 Ohm load to GND on PA1
    mach.set_pin_analog_model(
        'A',
        1,
        PinAnalogModel {
            load_ohm: Some(100),
            load_to_vdd: false,
            ..Default::default()
        },
    );

    // Output HIGH at 5mA (R_out = 100 Ohm) -> 3300 * 100 / (100 + 100) = 1650 mV
    mach.bus.gpio.pa_dir |= 1 << 1;
    mach.bus.gpio.pa_out |= 1 << 1;
    mach.bus.gpio.pa_pd_drv &= !(1 << 1);
    assert_eq!(mach.pin_voltage_mv('A', 1), 1650);

    // Switch to 20mA drive (R_out = 25 Ohm) -> 3300 * 100 / (100 + 25) = 2640 mV
    mach.bus.gpio.pa_pd_drv |= 1 << 1;
    assert_eq!(mach.pin_voltage_mv('A', 1), 2640);
}

#[test]
fn test_run_us_exceeds_one_second_without_truncation() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/uygulama.elf")) else {
        eprintln!("Skipping test_run_us_exceeds_one_second_without_truncation: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();
    mach.set_pin_drive('B', 9, ExtPinDrive::High);
    for ch in 0..14 {
        mach.bus.adc.channels[ch] = AdcWaveform::Dc(1650);
    }

    mach.run_us(1_200_000);
    let total_us = mach.bus.total_time_ns / 1_000;
    assert!(
        total_us >= 1_200_000,
        "Expected run_us(1_200_000) to reach >= 1.2s without 50M-step cutoff, got {total_us} us"
    );
}
