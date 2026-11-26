use ch585_core::cluster::MultiChipCluster;
use ch585_core::peripherals::irq;
use ch585_core::rf::{RfMediumConfig, RfMode, RfOutgoingFrame, RfPacketKind, STA_DMA_ENABLE};
use ch585_core::soc::Ch585Machine;

fn lui(rd: u8, imm20: u32) -> u32 {
    ((imm20 & 0xFFFFF) << 12) | ((rd as u32) << 7) | 0x37
}

fn addi(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20) | ((rs1 as u32) << 15) | ((rd as u32) << 7) | 0x13
}

fn bne(rs1: u8, rs2: u8, offset: i32) -> u32 {
    let imm = offset as u32;
    let b12 = (imm >> 12) & 1;
    let b11 = (imm >> 11) & 1;
    let b10_5 = (imm >> 5) & 0x3F;
    let b4_1 = (imm >> 1) & 0x0F;
    (b12 << 31)
        | (b10_5 << 25)
        | ((rs2 as u32) << 20)
        | ((rs1 as u32) << 15)
        | (1 << 12)
        | (b4_1 << 8)
        | (b11 << 7)
        | 0x63
}

fn jal(rd: u8, offset: i32) -> u32 {
    let imm = offset as u32;
    let b20 = (imm >> 20) & 1;
    let b10_1 = (imm >> 1) & 0x3FF;
    let b11 = (imm >> 11) & 1;
    let b19_12 = (imm >> 12) & 0xFF;
    (b20 << 31)
        | (b10_1 << 21)
        | (b11 << 20)
        | (b19_12 << 12)
        | ((rd as u32) << 7)
        | 0x6F
}

fn jalr(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20) | ((rs1 as u32) << 15) | ((rd as u32) << 7) | 0x67
}

fn write_prog(mach: &mut Ch585Machine, base: u32, words: &[u32]) {
    for (i, &w) in words.iter().enumerate() {
        let addr = (base as usize) + i * 4;
        if (0x2000_0000..0x2002_0000).contains(&base) {
            let off = addr - 0x2000_0000;
            mach.bus.sram[off..off + 4].copy_from_slice(&w.to_le_bytes());
        } else {
            mach.bus.flash[addr..addr + 4].copy_from_slice(&w.to_le_bytes());
        }
    }
}

fn setup_spanning_hook_machine(exact: bool) -> Ch585Machine {
    let mut mach = Ch585Machine::new();
    // Flash entry at 0x0000_0000 -> call CH58x_BLEInit at 0x0080, run 200-iter loop at 0x0100,
    // call stubbed RFRole_Stop in highcode SRAM at 0x2000_0100, then spin in WFI.
    write_prog(
        &mut mach,
        0x0000_0000,
        &[
            jal(1, 0x80),                // 0x00: call 0x80 (observer hook)
            addi(10, 0, 0),              // 0x04: a0 = 0 (accumulator)
            addi(11, 0, 200),            // 0x08: t1/x11 = 200
            jal(0, 0x0100 - 0x000C),     // 0x0C: jump to loop at 0x0100
        ],
    );
    // Observer hook at 0x0080: sets a5 = 7 and returns
    write_prog(
        &mut mach,
        0x0000_0080,
        &[
            addi(15, 0, 7),
            jalr(0, 1, 0),
        ],
    );
    // Hot loop at 0x0100 (inside [0x0080 .. 0x2000_0100] hook span)
    write_prog(
        &mut mach,
        0x0000_0100,
        &[
            addi(10, 10, 3),             // 0x100: x10 += 3
            addi(11, 11, -1),            // 0x104: x11 -= 1
            bne(11, 0, -8),              // 0x108: loop to 0x100
            lui(5, 0x20000),             // 0x10C: x5 = 0x2000_0000
            addi(5, 5, 0x100),           // 0x110: x5 = 0x2000_0100
            jalr(1, 5, 0),               // 0x114: call stubbed RFRole_Stop (returns a0=0? wait: check hook)
            jal(0, 0),                   // 0x118: halt loop
        ],
    );
    // Fallback code at 0x2000_0100 (overridden by RFRole_Stop stub)
    write_prog(&mut mach, 0x2000_0100, &[addi(15, 15, 99), jalr(0, 1, 0)]);

    mach.addr_by_sym.insert("CH58x_BLEInit".into(), 0x0000_0080);
    mach.addr_by_sym.insert("RFRole_Stop".into(), 0x2000_0100);
    mach.bus.ble.resolve_symbols(&mach.addr_by_sym);
    mach.bus.ble.rf.mode = RfMode::FastHost;
    mach.bus.flash_version = mach.bus.flash_version.wrapping_add(1);
    mach.exact_periph_step = exact;
    mach
}

#[test]
fn spanning_hooks_use_exact_pc_lookup_and_match_exact_execution() {
    let mut fast = setup_spanning_hook_machine(false);
    let mut exact = setup_spanning_hook_machine(true);

    // Span covers 0x0080..=0x2000_0100, so in_hook_range is true inside the loop,
    // while has_hook_at must be true ONLY at the exact registered hook PCs.
    assert!(fast.bus.ble.in_hook_range(0x0000_0100));
    assert!(!fast.bus.ble.has_hook_at(0x0000_0100));
    assert!(!fast.bus.ble.has_hook_at(0x0000_0104));
    assert!(!fast.bus.ble.has_hook_at(0x0000_0108));
    assert!(fast.bus.ble.has_hook_at(0x0000_0080));
    assert!(fast.bus.ble.has_hook_at(0x2000_0100));

    fast.run_us(500);
    exact.run_us(500);

    // x15 == 7 proves observer hook's guest instructions executed (and SRAM fallback += 99 did NOT,
    // because RFRole_Stop was stubbed at 0x2000_0100 and transitioned rf.mode to Idle).
    assert_eq!(fast.cpu.regs[15], 7);
    assert_eq!(exact.cpu.regs[15], 7);
    assert_eq!(fast.cpu.regs[11], 0);
    assert_eq!(exact.cpu.regs[11], 0);
    assert_eq!(fast.bus.ble.rf.mode, RfMode::Idle);
    assert_eq!(exact.bus.ble.rf.mode, RfMode::Idle);
    assert_eq!(fast.bus.ble.event_log.len(), 1);
    assert_eq!(fast.bus.ble.event_log, exact.bus.ble.event_log);
}

#[test]
fn rf_medium_next_event_horizon_covers_idle_tx_inflight_and_timeouts() {
    let mut cluster = MultiChipCluster::with_rf_config(RfMediumConfig {
        delay_us: 25,
        ..Default::default()
    });
    let host = Ch585Machine::new();
    let dev = Ch585Machine::new();
    let hi = cluster.add_chip("host", host);
    let di = cluster.add_chip("dev", dev);

    // Idle cluster has no pending RF horizon
    assert_eq!(cluster.next_rf_event_ns(), None);

    // Unpaired compatible FastHost + FastDevice triggers immediate pairing horizon
    cluster.chips[hi].bus.ble.rf.mode = RfMode::FastHost;
    cluster.chips[hi].bus.ble.rf.bound_active = true;
    cluster.chips[di].bus.ble.rf.mode = RfMode::FastDevice;
    cluster.chips[di].bus.ble.rf.bound_active = true;
    assert_eq!(cluster.next_rf_event_ns(), Some(0));

    cluster.run_us(50);
    assert!(cluster.chips[hi].bus.ble.rf.bound_connected);
    assert!(cluster.chips[di].bus.ble.rf.bound_connected);
    // Connected & idle with healthy peer has no RF timer pending
    assert_eq!(cluster.next_rf_event_ns(), None);

    // Pending TX DMA descriptor triggers immediate horizon
    let desc_addr: u32 = 0x2000_0400;
    let buf_addr: u32 = 0x2000_0420;
    cluster.chips[di].bus.ble.rf.tx_ring_head = desc_addr;
    cluster.chips[di].bus.ble.rf.tx_cur_desc = desc_addr;
    let d_off = (desc_addr - 0x2000_0000) as usize;
    cluster.chips[di].bus.sram[d_off..d_off + 4].copy_from_slice(&(STA_DMA_ENABLE | 4).to_le_bytes());
    cluster.chips[di].bus.sram[d_off + 4..d_off + 8].copy_from_slice(&64u32.to_le_bytes());
    cluster.chips[di].bus.sram[d_off + 8..d_off + 12].copy_from_slice(&buf_addr.to_le_bytes());
    cluster.chips[di].bus.sram[d_off + 12..d_off + 16].copy_from_slice(&desc_addr.to_le_bytes());
    assert_eq!(cluster.next_rf_event_ns(), Some(cluster.sim_time_ns));

    // Clear DMA, push outbox frame, step slightly to put it in flight -> horizon at deliver_at_ns
    cluster.chips[di].bus.sram[d_off..d_off + 4].copy_from_slice(&0u32.to_le_bytes());
    let now = cluster.sim_time_ns;
    let dev_id = cluster.chips[di].bus.ble.rf.dev_id;
    cluster.chips[di].bus.ble.rf.outbox.push_back(RfOutgoingFrame {
        kind: RfPacketKind::FastData,
        frequency_khz: 2_402_000,
        access_address: 0x7176_2345,
        crc_init: 0x0055_5555,
        dev_id,
        tx_power_dbm: 0,
        payload: vec![1, 2, 3, 4],
        sent_ns: now,
    });
    assert_eq!(cluster.next_rf_event_ns(), Some(now));

    let chips_len = cluster.chips.len();
    assert_eq!(chips_len, 2);
    cluster.rf_medium.step(now, &mut cluster.chips);
    assert_eq!(cluster.next_rf_event_ns(), Some(now + 25_000));

    // Deliver in-flight frame, then drop link -> timeout horizon matches last_link_ok + timeout_ms
    cluster.rf_medium.step(now + 25_000, &mut cluster.chips);
    assert_eq!(cluster.next_rf_event_ns(), None);

    cluster.rf_medium.link_dropped = true;
    cluster.chips[hi].bus.ble.rf.timeout_ms = 50;
    cluster.chips[di].bus.ble.rf.timeout_ms = 50;
    let ok_ns = cluster.chips[hi].bus.ble.rf.last_link_ok_ns;
    assert_eq!(cluster.next_rf_event_ns(), Some(ok_ns + 50_000_000));

    // BasicRx timeout horizon on a single powered chip
    let mut basic_cluster = MultiChipCluster::new();
    let mut rx_mach = Ch585Machine::new();
    rx_mach.bus.ble.rf.mode = RfMode::BasicRx;
    rx_mach.bus.ble.rf.basic_rx_start_ns = 10_000;
    rx_mach.bus.ble.rf.basic_rx_timeout_us = 250;
    basic_cluster.add_chip("rx", rx_mach);
    assert_eq!(basic_cluster.next_rf_event_ns(), Some(10_000 + 250_000));
}

fn sb(rs2: u8, rs1: u8, imm12: i32) -> u32 {
    let imm = (imm12 as u32) & 0xFFF;
    ((imm >> 5) << 25) | ((rs2 as u32) << 20) | ((rs1 as u32) << 15) | ((imm & 0x1F) << 7) | 0x23
}

#[test]
fn timer_and_adc_irq_dirty_fast_path_preserves_level_irqs() {
    let mut mach = Ch585Machine::new();
    // Test TMR0 stopped -> period match -> RW1C clear while stopped via MMIO,
    // plus ADC single conversion IRQ -> EOC clear while idle.
    mach.bus.timers[0].write_u32(0x0C, 5);      // cnt_end = 5
    mach.bus.timers[0].write_u8(0x02, 0x01);    // inter_en = CYC_END
    mach.bus.timers[0].write_u8(0x00, 0x04);    // enable timer
    mach.bus.step_peripherals(10);
    assert_ne!(mach.bus.timers[0].int_flag & 0x01, 0);
    assert!(mach.bus.pfic.is_pending(irq::TMR0));

    // Stop timer, step once (syncs with flag still set), then clear flag via RW1C while stopped
    mach.bus.timers[0].write_u8(0x00, 0x00);
    mach.bus.step_peripherals(4);
    assert!(mach.bus.pfic.is_pending(irq::TMR0));
    mach.bus.timers[0].write_u8(0x06, 0x01);
    mach.bus.step_peripherals(4);
    assert!(!mach.bus.pfic.is_pending(irq::TMR0));

    // Verify guest MMIO write path triggers ADC EOC IRQ and clears cleanly when idle
    let mut guest = Ch585Machine::new();
    write_prog(
        &mut guest,
        0x0000_0000,
        &[
            lui(5, 0x40001),          // 0x00: x5 = SYS/ADC base 0x4000_1000
            addi(6, 0, 0x01),
            sb(6, 5, 0x5A),           // 0x08: R8_ADC_CONVERT = 1 (start single conversion)
            jal(0, 0),                // 0x0C: spin
        ],
    );
    guest.run_us(50);
    assert!(guest.bus.pfic.is_pending(irq::ADC));
    guest.bus.adc.write_u8(0x5A, 0x00); // clears RB_ADC_IF_EOC while idle/stopped
    guest.bus.step_peripherals(4);
    assert!(!guest.bus.pfic.is_pending(irq::ADC));
}
