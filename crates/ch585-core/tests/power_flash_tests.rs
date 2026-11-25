use std::time::Instant;

use ch585_core::peripherals::gpio::ExtPinDrive;
use ch585_core::peripherals::irq;
use ch585_core::peripherals::sys::{
    PowerMode, RST_FLAG_GPWSM, RST_FLAG_MR, RST_FLAG_RPOR, RST_FLAG_SW, RST_FLAG_WTR,
};
use ch585_core::soc::{Ch585Machine, SysBus};

fn flash_spi_tx(bus: &mut SysBus, bytes: &[u8]) {
    bus.flash_ctrl.write_u8(0x06, 0x05, &mut bus.flash); // CS low
    for &b in bytes {
        bus.flash_ctrl.write_u8(0x04, b, &mut bus.flash);
    }
    bus.flash_ctrl.write_u8(0x06, 0x00, &mut bus.flash); // CS high
}

fn flash_rdsr(bus: &mut SysBus) -> u8 {
    bus.flash_ctrl.write_u8(0x06, 0x05, &mut bus.flash);
    bus.flash_ctrl.write_u8(0x04, 0x05, &mut bus.flash);
    let st = bus.flash_ctrl.read_u8(0x04, &bus.flash);
    bus.flash_ctrl.write_u8(0x06, 0x00, &mut bus.flash);
    st
}

fn enc_addi(rd: u32, rs1: u32, imm: i32) -> u32 {
    (((imm as u32) & 0xFFF) << 20) | (rs1 << 15) | (rd << 7) | 0x13
}

fn enc_lui(rd: u32, imm20: u32) -> u32 {
    (imm20 << 12) | (rd << 7) | 0x37
}

fn enc_sb(rs2: u32, rs1: u32, imm: i32) -> u32 {
    let u = (imm as u32) & 0xFFF;
    let hi = (u >> 5) & 0x7F;
    let lo = u & 0x1F;
    (hi << 25) | (rs2 << 20) | (rs1 << 15) | (0b000 << 12) | (lo << 7) | 0x23
}

fn enc_sh(rs2: u32, rs1: u32, imm: i32) -> u32 {
    let u = (imm as u32) & 0xFFF;
    let hi = (u >> 5) & 0x7F;
    let lo = u & 0x1F;
    (hi << 25) | (rs2 << 20) | (rs1 << 15) | (0b001 << 12) | (lo << 7) | 0x23
}

fn enc_sw(rs2: u32, rs1: u32, imm: i32) -> u32 {
    let u = (imm as u32) & 0xFFF;
    let hi = (u >> 5) & 0x7F;
    let lo = u & 0x1F;
    (hi << 25) | (rs2 << 20) | (rs1 << 15) | (0b010 << 12) | (lo << 7) | 0x23
}

fn enc_j(offset_bytes: i32) -> u32 {
    let imm = offset_bytes as u32;
    let b20 = (imm >> 20) & 1;
    let b10_1 = (imm >> 1) & 0x3FF;
    let b11 = (imm >> 11) & 1;
    let b19_12 = (imm >> 12) & 0xFF;
    (b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12) | 0x6F
}

#[test]
fn test_reset_flags_and_sram_keep_behavior() {
    let mut mach = Ch585Machine::new();
    assert_eq!(mach.bus.sys.reset_flag, RST_FLAG_RPOR);

    // Populate SRAM sentinel and R8_GLOB_RESET_KEEP
    mach.bus.sram[0x100..0x104].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
    mach.bus.sys.write_u8(0x47, 0xA5);

    // Pin reset clears SRAM, preserves R8_GLOB_RESET_KEEP, and sets RST_FLAG_MR (0x03)
    mach.pin_reset();
    assert_eq!(mach.bus.sys.reset_flag, RST_FLAG_MR);
    assert_eq!(
        mach.bus.sys.read_u8(0x44, mach.bus.total_cycles) & 0x07,
        RST_FLAG_MR
    );
    assert_eq!(&mach.bus.sram[0x100..0x104], &[0, 0, 0, 0]);
    assert_eq!(mach.bus.sys.glob_reset_keep, 0xA5);

    // Software reset clears SRAM, preserves R8_GLOB_RESET_KEEP, and sets RST_FLAG_SW (0x00)
    mach.bus.sram[0x100..0x104].copy_from_slice(&0x1234_5678u32.to_le_bytes());
    mach.soft_reset(RST_FLAG_SW);
    assert_eq!(mach.bus.sys.reset_flag, RST_FLAG_SW);
    assert_eq!(&mach.bus.sram[0x100..0x104], &[0, 0, 0, 0]);
    assert_eq!(mach.bus.sys.glob_reset_keep, 0xA5);

    // Power cycle clears SRAM AND clears R8_GLOB_RESET_KEEP, setting RST_FLAG_RPOR (0x01)
    mach.bus.sram[0x100..0x104].copy_from_slice(&0xCAFE_BABEu32.to_le_bytes());
    mach.power_cycle();
    assert_eq!(mach.bus.sys.reset_flag, RST_FLAG_RPOR);
    assert_eq!(&mach.bus.sram[0x100..0x104], &[0, 0, 0, 0]);
    assert_eq!(mach.bus.sys.glob_reset_keep, 0x00);

    // Watchdog timeout reset -> RST_FLAG_WTR (0x02)
    mach.bus.flash[0..4].copy_from_slice(&0x0000_006Fu32.to_le_bytes()); // j .
    mach.bus.sys.write_u8(0x40, 0x57);
    mach.bus.sys.write_u8(0x40, 0xA8);
    mach.bus.sys.write_u8(0x46, 0x02); // RB_WDOG_RST_EN
    mach.bus.sys.write_u8(0x43, 0xFF);
    mach.run_cycles(200_000);
    assert_eq!(mach.bus.sys.reset_flag, RST_FLAG_WTR);

    // Shutdown wakeup with RAM32K retention -> RST_FLAG_GPWSM (0x05)
    mach.cpu.pc = 0;
    mach.bus.flash[0..4].copy_from_slice(&0x1050_0073u32.to_le_bytes()); // wfi
    mach.bus.flash[4..8].copy_from_slice(&0x0000_006Fu32.to_le_bytes()); // j .
    mach.set_pin_drive('B', 0, ExtPinDrive::High);
    mach.bus.step_peripherals(1);
    mach.bus.gpio.pb_int_mode |= 1 << 0;
    mach.bus.gpio.pb_int_en |= 1 << 0;
    mach.bus.pfic.write_u32(0x100, 1 << irq::GPIO_B);
    mach.bus.pfic.sctlr |= 1 << 2; // SLEEPDEEP
    mach.bus.sys.power_plan = 0x8000 | 0x02; // RB_PWR_PLAN_EN | RB_PWR_RAM32K (Shutdown)
    mach.bus.sys.slp_wake_ctrl = 0x10; // RB_SLP_GPIO_WAKE
    mach.run_cycles(200);
    assert_eq!(mach.bus.power_mode, PowerMode::Shutdown);
    mach.bus.sram[0x200] = 0x5A;
    mach.set_pin_drive('B', 0, ExtPinDrive::Low);
    mach.run_cycles(10);
    assert_eq!(mach.bus.sys.reset_flag, RST_FLAG_GPWSM);
    assert_eq!(mach.bus.sram[0x200], 0x5A);
}

#[test]
fn test_vdd_bor_lvr_and_bat_det_monitor() {
    let mut mach = Ch585Machine::new();
    mach.bus.flash[0..4].copy_from_slice(&0x0000_006Fu32.to_le_bytes()); // j .
    mach.bus.sram[0x50] = 0x77;

    // Configure LPLevel_2V2 (low=2200mV, lower=2100mV) with RB_BAT_LOW_IE
    mach.bus.sys.write_u8(0x40, 0x57);
    mach.bus.sys.write_u8(0x40, 0xA8);
    mach.bus.sys.write_u8(0x25, 0x01); // LPLevel_2V2 cfg
    mach.bus.sys.write_u8(0x24, 0x02 | 0x01 | 0x08); // RB_BAT_MON_EN | RB_BAT_LOW_VTHX | RB_BAT_LOW_IE

    mach.set_vdd_mv(2300);
    assert_eq!(mach.bus.sys.bat_status, 0);
    assert!(!mach.bus.pfic.is_pending(irq::WDOG_BAT));

    // Drop below 2200mV -> LOW flag + WDOG_BAT IRQ, MCU still powered
    mach.set_vdd_mv(2150);
    assert_eq!(mach.bus.sys.bat_status & 0x02, 0x02);
    assert_eq!(mach.bus.sys.bat_status & 0x01, 0x00);
    assert!(mach.bus.pfic.is_pending(irq::WDOG_BAT));
    assert!(mach.bus.powered);

    // Drop below 2100mV -> LOWER + LOW
    mach.set_vdd_mv(2050);
    assert_eq!(mach.bus.sys.bat_status, 0x03);
    assert!(mach.bus.powered);

    // Drop below BOR (1750mV) -> BOR holds MCU unpowered and wipes SRAM
    mach.set_vdd_mv(1600);
    assert!(mach.bus.bor_active);
    assert!(!mach.bus.powered);
    assert_eq!(mach.bus.sram[0x50], 0);

    // While unpowered, time advances without executing CPU instructions
    let instret_before = mach.cpu.instret;
    mach.run_us(5_000);
    assert_eq!(mach.cpu.instret, instret_before);
    assert!(mach.bus.unpowered_ns >= 5_000_000);

    // Rise into hysteresis deadband (1780mV < 1750+50) -> stays held in BOR
    mach.set_vdd_mv(1780);
    assert!(mach.bus.bor_active);
    assert!(!mach.bus.powered);

    // Recover above hysteresis -> reboots with RST_FLAG_RPOR
    mach.set_vdd_mv(3300);
    assert!(!mach.bus.bor_active);
    assert!(mach.bus.powered);
    assert_eq!(mach.bus.sys.reset_flag, RST_FLAG_RPOR);
}

#[test]
fn test_flash_persistence_and_mid_write_mid_erase_power_cut() {
    let mut mach = Ch585Machine::new();
    mach.bus.flash[0..4].copy_from_slice(&0x0000_006Fu32.to_le_bytes()); // j .

    // A) Normal DataFlash write at 0x70000: WIP=1 while in flight, WIP=0 after 100us
    let payload: Vec<u8> = (0..16).map(|i| 0x10 + i).collect();
    flash_spi_tx(&mut mach.bus, &[0x06]); // WREN
    let mut prog_cmd = vec![0x02, 0x07, 0x00, 0x00];
    prog_cmd.extend_from_slice(&payload);
    flash_spi_tx(&mut mach.bus, &prog_cmd);

    assert_eq!(flash_rdsr(&mut mach.bus) & 0x01, 0x01, "WIP should be 1 right after CS high");
    mach.run_us(100);
    assert_eq!(flash_rdsr(&mut mach.bus) & 0x01, 0x00, "WIP should clear once program finishes");
    assert_eq!(&mach.dataflash_bytes()[..16], &payload[..]);

    // B) Flash persistence round-trip into new machines (both 32KB DataFlash and 512KB overlay)
    let saved_df = mach.dataflash_bytes().to_vec();
    let saved_full = mach.flash_bytes().to_vec();

    let mut mach_df = Ch585Machine::new();
    mach_df.load_flash_image(&saved_df);
    assert_eq!(&mach_df.dataflash_bytes()[..16], &payload[..]);

    let mut mach_ov = Ch585Machine::new();
    mach_ov.load_flash_overlay_dataflash(&saved_full);
    assert_eq!(&mach_ov.dataflash_bytes()[..16], &payload[..]);

    // C) Mid-write power cut at 50% of 16-byte program at 0x70100
    mach.bus.flash_ctrl.power_cut_next_op_pct = Some(50);
    flash_spi_tx(&mut mach.bus, &[0x06]);
    let mut prog_cut = vec![0x02, 0x07, 0x01, 0x00];
    prog_cut.extend_from_slice(&[0x00u8; 16]);
    flash_spi_tx(&mut mach.bus, &prog_cut);

    mach.run_us(100);
    assert!(!mach.bus.powered, "Flash cut should power off MCU");
    assert_eq!(mach.bus.flash_ctrl.interrupted_ops, 1);
    assert_eq!(&mach.dataflash_bytes()[0x100..0x108], &[0x00u8; 8]);
    assert_ne!(mach.dataflash_bytes()[0x108], 0x00);
    assert_ne!(mach.dataflash_bytes()[0x108], 0xFF);
    assert_eq!(&mach.dataflash_bytes()[0x109..0x110], &[0xFFu8; 7]);

    // D) Mid-erase power cut at 50% of 256B page erase at 0x70200
    mach.power_on();
    flash_spi_tx(&mut mach.bus, &[0x06]);
    let mut prog_zeros = vec![0x02, 0x07, 0x02, 0x00];
    prog_zeros.extend_from_slice(&[0x00u8; 256]);
    flash_spi_tx(&mut mach.bus, &prog_zeros);
    mach.run_us(100);
    assert_eq!(&mach.dataflash_bytes()[0x200..0x300], &[0x00u8; 256]);

    mach.bus.flash_ctrl.power_cut_next_op_pct = Some(50);
    flash_spi_tx(&mut mach.bus, &[0x06]);
    flash_spi_tx(&mut mach.bus, &[0x81, 0x07, 0x02, 0x00]); // 256B Page Erase
    mach.run_us(300);

    assert!(!mach.bus.powered, "Flash erase cut should power off MCU");
    assert_eq!(mach.bus.flash_ctrl.interrupted_ops, 2);
    assert_eq!(&mach.dataflash_bytes()[0x200..0x280], &[0xFFu8; 128]);
    assert_eq!(mach.dataflash_bytes()[0x280], 0xAA);
    assert_eq!(&mach.dataflash_bytes()[0x281..0x300], &[0x00u8; 127]);
}

#[test]
fn test_per_peripheral_current_breakdown() {
    let mut mach = Ch585Machine::new();
    mach.bus.flash[0..4].copy_from_slice(&0x0000_006Fu32.to_le_bytes()); // j .

    // Enable DCDC + ADC + USBHS active + UART2 + TMR0 + PWM4 + HA BAT_DET
    mach.bus.sys.power_plan |= 0x0200;
    mach.bus.adc.cfg |= 0x01;
    mach.bus.usbhs.ctrl = 0x20;
    mach.bus.usbhs.pll_ctrl = 0x04;
    mach.bus.uarts[2].fcr = 0x01;
    mach.bus.timers[0].ctrl_mod = 0x04;
    mach.bus.pwm.out_en = 0x01;
    mach.bus.sys.bat_det_ctrl = 0x01;

    mach.run_us(10_000);
    let pr = mach.bus.power_report();
    assert!(pr.core_avg_ua > 3_900.0 && pr.core_avg_ua < 4_100.0, "core={}", pr.core_avg_ua);
    assert!(pr.adc_avg_ua > 350.0, "adc={}", pr.adc_avg_ua);
    assert!(pr.usbhs_avg_ua > 11_000.0, "usbhs={}", pr.usbhs_avg_ua);
    assert!(pr.uart_avg_ua > 90.0, "uart={}", pr.uart_avg_ua);
    assert!(pr.tmr_avg_ua > 70.0, "tmr={}", pr.tmr_avg_ua);
    assert!(pr.pwm_avg_ua > 70.0, "pwm={}", pr.pwm_avg_ua);
    assert!(pr.bat_mon_avg_ua > 200.0, "bat={}", pr.bat_mon_avg_ua);
    assert!(pr.total_avg_ua > 16_000.0, "total={}", pr.total_avg_ua);
    assert!((pr.total_avg_ua - mach.bus.estimated_avg_current_ua()).abs() < 1e-6);
}

#[test]
fn test_one_hour_sleep_time_skipping_with_rtc_wakeup() {
    let mut mach = Ch585Machine::new();

    // Build tiny RV32I Sleep loop:
    //   x5 = 0x4000_1000 (SYS/RTC base)
    //   x6 = 0x2000_0000 (SRAM base)
    //   x7 = 0x10        (RB_RTC_TMR_CLR)
    //   x8 = 0           (wakeup count)
    //   x10 = 0x8006     (RB_PWR_PLAN_EN | RB_PWR_CORE | RB_PWR_RAM32K -> Sleep)
    //   x11 = 0x57, x12 = 0xA8 (Safe Access keys)
    // loop:
    //   sb x11, 0x40(x5)
    //   sb x12, 0x40(x5)
    //   sh x10, 0x20(x5)
    //   wfi
    //   sb x7, 0x30(x5)
    //   addi x8, x8, 1
    //   sw x8, 0x400(x6)
    //   j loop
    let prog = [
        enc_lui(5, 0x40001),
        enc_lui(6, 0x20000),
        enc_addi(7, 0, 0x10),
        enc_addi(8, 0, 0),
        enc_lui(10, 0x8),
        enc_addi(10, 10, 6),
        enc_addi(11, 0, 0x57),
        enc_addi(12, 0, 0xA8),
        // loop offset = 8 * 4 = 32
        enc_sb(11, 5, 0x40),
        enc_sb(12, 5, 0x40),
        enc_sh(10, 5, 0x20),
        0x1050_0073u32, // wfi
        enc_sb(7, 5, 0x30),
        enc_addi(8, 8, 1),
        enc_sw(8, 6, 0x400),
        enc_j(-28),
    ];
    for (i, &insn) in prog.iter().enumerate() {
        let off = i * 4;
        mach.bus.flash[off..off + 4].copy_from_slice(&insn.to_le_bytes());
    }

    mach.cpu.pc = 0;
    mach.bus.pfic.write_u32(0x100, 1 << irq::RTC);
    mach.bus.pfic.sctlr |= 1 << 2; // SLEEPDEEP
    mach.bus.sys.slp_wake_ctrl = 0x08; // RB_SLP_RTC_WAKE
    mach.bus.rtc.mode_ctrl = 0x13; // Periodic timer 32768 ticks = 1.024s at 32kHz

    let wall_start = Instant::now();
    // Run 1 full simulated hour (3,600,000,000 us)
    mach.run_us(3_600_000_000);
    let wall_elapsed = wall_start.elapsed();

    assert!(
        wall_elapsed.as_secs_f64() < 2.0,
        "1h sleep took too long: {:.3}s",
        wall_elapsed.as_secs_f64()
    );
    assert!(mach.bus.total_time_ns >= 3_600_000_000_000);
    assert!(
        mach.bus.sleep_skipped_ns > 3_500_000_000_000,
        "sleep_skipped_ns={}",
        mach.bus.sleep_skipped_ns
    );

    let wake_count = u32::from_le_bytes([
        mach.bus.sram[0x400],
        mach.bus.sram[0x401],
        mach.bus.sram[0x402],
        mach.bus.sram[0x403],
    ]);
    // 3600s / 1.024s = 3515.625 -> 3515 RTC timer ticks/wakeups
    assert_eq!(wake_count, 3515, "wake_count={wake_count}");
    assert_eq!(mach.bus.rtc.cnt_2s, 1757, "cnt_2s={}", mach.bus.rtc.cnt_2s);
    assert!(
        mach.bus.estimated_avg_current_ua() < 10.0,
        "Expected <10uA sleep avg, got {:.2}uA",
        mach.bus.estimated_avg_current_ua()
    );
}
