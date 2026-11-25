use ch585_boards::MacropadV2Board;
use ch585_core::peripherals::gpio::ExtPinDrive;
use ch585_core::soc::Ch585Machine;

#[test]
fn test_macropad_board_ratiometric_battery_bq25300_165_encoder_ws2812() {
    let mut mach = Ch585Machine::new();
    // Place `j .` in flash so `mach.step()` / `run_us()` run peripherals + ExtHost cleanly
    mach.bus.flash[0..4].copy_from_slice(&0x0000_006Fu32.to_le_bytes());

    let board = MacropadV2Board::attach(&mut mach);

    // 1. Ratiometric analog rail controlled by PB16
    mach.bus.gpio.pb_dir |= 1 << 16;
    mach.bus.gpio.pb_out |= 1 << 16;
    mach.step();
    assert_eq!(mach.bus.adc.sample_channel_mv(4, 0.0, &mach.bus.gpio), 1650);
    assert_eq!(mach.bus.adc.sample_channel_mv(1, 0.0, &mach.bus.gpio), 1650);

    board.set_supply_mv(&mut mach, 3000);
    assert_eq!(mach.bus.adc.sample_channel_mv(4, 0.0, &mach.bus.gpio), 1500);
    assert_eq!(mach.bus.adc.sample_channel_mv(1, 0.0, &mach.bus.gpio), 1500);

    mach.bus.gpio.pb_out &= !(1 << 16);
    mach.step();
    assert_eq!(mach.bus.adc.sample_channel_mv(4, 0.0, &mach.bus.gpio), 0);
    assert_eq!(mach.bus.adc.sample_channel_mv(1, 0.0, &mach.bus.gpio), 0);

    // 2. BQ25300 charger STAT on PB8 (active-LOW CHG_EN on PB7 with R66 100k pull-down)
    mach.bus.gpio.pb_pu |= 1 << 8;
    mach.set_pin_drive('B', 9, ExtPinDrive::High); // VBUS
    mach.step();
    assert!(board.state().charger.charging);
    assert_eq!(mach.bus.gpio.effective_pb_pin() & (1 << 8), 0); // STAT pulled low

    mach.bus.gpio.pb_dir |= 1 << 7;
    mach.bus.gpio.pb_out |= 1 << 7; // Drive CHG_EN HIGH -> charging disabled
    mach.step();
    assert!(!board.state().charger.charging);
    assert_ne!(mach.bus.gpio.effective_pb_pin() & (1 << 8), 0); // Released -> pull-up high
    mach.bus.gpio.pb_dir &= !(1 << 7);
    mach.bus.gpio.pb_out &= !(1 << 7);

    // 3. Battery divider model (3700mV -> AIN13=1183mV; 3600mV -> 1151mV)
    assert_eq!(board.state().battery.mv, 3700);
    assert_eq!(mach.bus.adc.sample_channel_mv(13, 0.0, &mach.bus.gpio), 1183);
    board.set_battery_mv(&mut mach, 3600);
    assert_eq!(mach.bus.adc.sample_channel_mv(13, 0.0, &mach.bus.gpio), 1151);

    // 4. 74HC165 16-bit parallel load & shift (PB3 data, PB5 PL#, PB6 CLK)
    board.set_shift165_bits(&mut mach, 0xA55A);
    mach.bus.gpio.pb_dir |= (1 << 5) | (1 << 6);
    mach.bus.gpio.pb_out &= !((1 << 5) | (1 << 6)); // PL#=0 load
    mach.step();
    mach.bus.gpio.pb_out |= 1 << 5; // PL#=1 shift enable
    mach.step();
    let mut shifted = 0u16;
    for _ in 0..16 {
        let bit = if (mach.bus.gpio.effective_pb_pin() & (1 << 3)) != 0 {
            1
        } else {
            0
        };
        shifted = (shifted << 1) | bit;
        mach.bus.gpio.pb_out |= 1 << 6;
        mach.step();
        mach.bus.gpio.pb_out &= !(1 << 6);
        mach.step();
    }
    assert_eq!(shifted, 0xA55A);

    // 5. Quadrature encoder (PB1/PB2) driven via ExtHost device timer
    board.queue_encoder_steps(&mut mach, 2);
    mach.run_us(5_000);
    assert_eq!(board.state().encoder.pending(), 0);
    let pb = mach.bus.gpio.effective_pb_pin();
    assert_eq!((pb >> 1) & 0x3, 0x0, "2 steps from (1,1) -> (0,1) -> (0,0)");

    // 6. WS2812 decoder on PB10 driven by real TMR1 DMA PWM via pin-change + latch timer events
    mach.bus.sys.write_u8(0x40, 0x57);
    mach.bus.sys.write_u8(0x40, 0xA8);
    mach.bus.sys.write_u8(0x08, 0x45); // PLL/5 = 62.4MHz
    mach.bus.sys.write_u8(0x09, 0x01);
    mach.bus.gpio.pin_alternate |= 0x02; // RB_PIN_TMR1 -> PB10
    mach.bus.gpio.pb_dir |= 1 << 10;

    let grb: [u8; 3] = [0x11, 0x22, 0x33];
    let dma_base = 0x2000_4000u32;
    let mut words = Vec::new();
    for byte in grb {
        for bit in (0..8).rev() {
            let duty = if ((byte >> bit) & 1) != 0 { 50u32 } else { 22u32 };
            words.push(duty);
        }
    }
    for (idx, w) in words.iter().enumerate() {
        let addr = (dma_base as usize - 0x2000_0000) + idx * 4;
        mach.bus.sram[addr..addr + 4].copy_from_slice(&w.to_le_bytes());
    }
    mach.bus.timers[1].write_u32(0x0C, 78);
    mach.bus.timers[1].write_u32(0x18, dma_base);
    mach.bus.timers[1].write_u32(0x1C, dma_base + (words.len() as u32) * 4);
    mach.bus.timers[1].write_u8(0x01, 0x01);
    mach.bus.timers[1].write_u8(0x00, 0x0C);

    mach.run_cycles(8_000);
    assert_eq!(board.state().ws2812.frame_count, 1);
    assert_eq!(board.state().ws2812.leds, vec![[0x22, 0x11, 0x33]]);
}
