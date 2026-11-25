use ch585_core::peripherals::sys::PowerMode;
use ch585_core::soc::Ch585Machine;

fn lui(rd: u8, imm20: u32) -> u32 {
    ((imm20 & 0xFFFFF) << 12) | ((rd as u32) << 7) | 0x37
}

fn addi(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20) | ((rs1 as u32) << 15) | ((rd as u32) << 7) | 0x13
}

fn andi(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20)
        | ((rs1 as u32) << 15)
        | (7 << 12)
        | ((rd as u32) << 7)
        | 0x13
}

fn xori(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20)
        | ((rs1 as u32) << 15)
        | (4 << 12)
        | ((rd as u32) << 7)
        | 0x13
}

fn lw(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20)
        | ((rs1 as u32) << 15)
        | (2 << 12)
        | ((rd as u32) << 7)
        | 0x03
}

fn sb(rs2: u8, rs1: u8, imm12: i32) -> u32 {
    let imm = (imm12 as u32) & 0xFFF;
    ((imm >> 5) << 25) | ((rs2 as u32) << 20) | ((rs1 as u32) << 15) | ((imm & 0x1F) << 7) | 0x23
}

fn sw(rs2: u8, rs1: u8, imm12: i32) -> u32 {
    let imm = (imm12 as u32) & 0xFFF;
    ((imm >> 5) << 25)
        | ((rs2 as u32) << 20)
        | ((rs1 as u32) << 15)
        | (2 << 12)
        | ((imm & 0x1F) << 7)
        | 0x23
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

fn csrrs(rd: u8, csr: u16, rs1: u8) -> u32 {
    ((csr as u32) << 20) | ((rs1 as u32) << 15) | (2 << 12) | ((rd as u32) << 7) | 0x73
}

fn csrrw(rd: u8, csr: u16, rs1: u8) -> u32 {
    ((csr as u32) << 20) | ((rs1 as u32) << 15) | (1 << 12) | ((rd as u32) << 7) | 0x73
}

fn csrrsi(rd: u8, csr: u16, uimm: u8) -> u32 {
    ((csr as u32) << 20) | (((uimm as u32) & 31) << 15) | (6 << 12) | ((rd as u32) << 7) | 0x73
}

fn mret() -> u32 {
    0x3020_0073
}

fn wfi() -> u32 {
    0x1050_0073
}

fn write_flash_prog(mach: &mut Ch585Machine, base: u32, prog: &[u32]) {
    for (i, &w) in prog.iter().enumerate() {
        let addr = (base as usize) + i * 4;
        mach.bus.flash[addr..addr + 4].copy_from_slice(&w.to_le_bytes());
    }
    mach.bus.flash_version = mach.bus.flash_version.wrapping_add(1);
}

fn read_sram_u32(mach: &Ch585Machine, vaddr: u32) -> u32 {
    let i = (vaddr - 0x2000_0000) as usize;
    u32::from_le_bytes([
        mach.bus.sram[i],
        mach.bus.sram[i + 1],
        mach.bus.sram[i + 2],
        mach.bus.sram[i + 3],
    ])
}

#[test]
fn test_batch_vs_exact_irq_timing_parity_systick_tmr0_usb() {
    let run_case = |batch_cycles: u32| -> (Vec<(u64, u16)>, u32, u32, usize, u16) {
        let mut m = Ch585Machine::new();
        m.periph_batch_cycles = batch_cycles;

        // Main program at 0x0:
        //   mtvec = 0x100 (direct trap vector)
        //   R32_PA_DIR (0x4000_10A0) |= 0x03 (PA0, PA1 outputs)
        //   enable MIE in mstatus
        //   busy spin incrementing x10 (stays inside cached basic-block loop!)
        let main_prog = [
            addi(5, 0, 0x100),      // li t0, 0x100
            csrrw(0, 0x305, 5),     // csrw mtvec, t0
            lui(5, 0x40001),        // lui t0, 0x40001
            addi(6, 0, 0x03),       // li t1, 3
            sw(6, 5, 0x0A0),        // sw t1, 0x0A0(t0) -> R32_PA_DIR = 3
            csrrsi(0, 0x300, 8),    // csrsi mstatus, MIE
            addi(10, 10, 1),        // spin: addi a0, a0, 1
            jal(0, -4),             // j spin
        ];
        write_flash_prog(&mut m, 0x0, &main_prog);

        // Trap handler at 0x100:
        // Uses t0..t3 (x5..x7, x28) without clobbering main loop's a0 (x10).
        // 0x100: csrr t0, mcause
        // 0x104: andi t0, t0, 0x1F
        // 0x108: addi t1, x0, 12 (SysTick)
        // 0x10C: bne  t0, t1, check_tmr0 (+36 -> 0x130)
        // 0x110: lui  t1, 0xE000F
        // 0x114: sw   x0, 4(t1)          // SysTick->SR = 0
        // 0x118: lui  t1, 0x40001
        // 0x11C: lw   t2, 0x0AC(t1)      // R32_PA_OUT
        // 0x120: xori t2, t2, 1          // toggle PA0
        // 0x124: sw   t2, 0x0AC(t1)
        // 0x128: jal  x0, inc_done (+32 -> 0x148)
        // check_tmr0 (0x130):
        // 0x130: lui  t1, 0x40002
        // 0x134: sb   x0, 6(t1)          // wait: RW1C! write 0x01 to 0x4000_2006
        // 0x134 should write 1: addi t2, x0, 1; sb t2, 6(t1)
        let isr_prog = [
            csrrs(5, 0x342, 0),     // 0x100: csrr t0, mcause
            andi(5, 5, 0x1F),       // 0x104: andi t0, t0, 0x1F
            addi(6, 0, 12),         // 0x108: li t1, 12
            bne(5, 6, 36),          // 0x10C -> 0x130 (check_tmr0)
            lui(6, 0xE000F),        // 0x110
            sw(0, 6, 4),            // 0x114: SysTick->SR = 0
            lui(6, 0x40001),        // 0x118
            lw(7, 6, 0x0A8),        // 0x11C: R32_PA_OUT
            xori(7, 7, 1),          // 0x120: toggle PA0
            sw(7, 6, 0x0A8),        // 0x124
            jal(0, 40),             // 0x128 -> 0x150 (inc_done)
            addi(0, 0, 0),          // 0x12C: nop
            // check_tmr0 @ 0x130:
            lui(6, 0x40002),        // 0x130
            addi(7, 0, 1),          // 0x134
            sb(7, 6, 6),            // 0x138: R8_TMR0_INT_FLAG = 1 (RW1C)
            lui(6, 0x40001),        // 0x13C
            lw(7, 6, 0x0A8),        // 0x140: R32_PA_OUT
            xori(7, 7, 2),          // 0x144: toggle PA1
            sw(7, 6, 0x0A8),        // 0x148
            addi(0, 0, 0),          // 0x14C: nop
            // inc_done @ 0x150:
            lui(6, 0x20000),        // 0x150
            lw(7, 6, 0x100),        // 0x154
            addi(7, 7, 1),          // 0x158
            sw(7, 6, 0x100),        // 0x15C
            mret(),                 // 0x160
        ];
        write_flash_prog(&mut m, 0x100, &isr_prog);

        // Configure PFIC for SysTick(12) + TMR0(16)
        m.bus.pfic.write_u32(0x100, (1 << 12) | (1 << 16));
        // Configure SysTick: STE | STIE | STCLK(HCLK) | STRE = 0x0F, CMP = 3333
        m.bus.systick.cmp = 3333;
        m.bus.systick.ctlr = 0x0F;
        // Configure TMR0: COUNT_EN(0x04), CNT_END = 7777, INTER_EN = 0x01
        m.bus.timers[0].cnt_end = 7777;
        m.bus.timers[0].inter_en = 0x01;
        m.bus.timers[0].ctrl_mod = 0x04;

        // Enable USBHS attached + enumerated and arm an EP1 IN packet mid-way
        m.bus.usbhs.ctrl = 0x20;
        m.bus.usbhs.pll_ctrl = 0x04;
        m.bus.usbhs.host_attached = true;
        m.bus.usbhs.attach();
        m.bus.usbhs.dev_ad = 1;
        m.bus.usbhs.enumerated = true;
        m.bus.usbhs.ep_tx_en = 0x02;
        m.bus.usbhs.ep_tx_dma[1] = 0x2000_0200;
        m.bus.usbhs.ep_t_len[1] = 4;
        m.bus.sram[0x200..0x204].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);
        m.bus.usbhs.ep_tx_ctrl[1] = 0x02; // ACK ready

        m.bus.logic.max_samples = 10_000;
        m.run_us(15_000);

        let transitions: Vec<(u64, u16)> = m
            .bus
            .logic
            .samples
            .windows(2)
            .filter(|w| w[0].pa != w[1].pa)
            .map(|w| (w[1].time_ns, w[1].pa))
            .collect();
        let isr_cnt = read_sram_u32(&m, 0x2000_0100);
        (
            transitions,
            isr_cnt,
            m.bus.systick.cnt as u32,
            m.bus.usbhs.in_packets.len(),
            m.bus.usbhs.frame_no,
        )
    };

    let exact = run_case(1);
    let batched = run_case(256);

    assert!(exact.1 >= 30, "expected >=30 IRQs, got {}", exact.1);
    assert!(
        exact.0.len() >= 30,
        "expected >=30 GPIO toggles, got {}",
        exact.0.len()
    );
    assert_eq!(batched.0, exact.0, "GPIO IRQ edge timestamps must match 1-cycle ref");
    assert_eq!(batched.1, exact.1, "total ISR count mismatch");
    assert_eq!(batched.2, exact.2, "SysTick cnt mismatch");
    assert_eq!(batched.3, exact.3, "USB packet count mismatch");
    assert_eq!(batched.4, exact.4, "USB frame_no mismatch");
}

#[test]
fn test_idle_fast_forward_with_active_usbhs_and_blocker_stats() {
    // Part A: WFI loop with active, host-attached, unsuspended USBHS still fast-forwards
    // Idle windows while preserving SysTick IRQs, SOF frame counting, and EP1/EP4 transfers.
    let mut m = Ch585Machine::new();
    let main_prog = [
        addi(5, 0, 0x100),      // 0x00: li t0, 0x100
        csrrw(0, 0x305, 5),     // 0x04: csrw mtvec, t0
        csrrsi(0, 0x300, 8),    // 0x08: csrsi mstatus, MIE
        wfi(),                  // 0x0C: loop: wfi
        jal(0, -4),             // 0x10: j loop
    ];
    write_flash_prog(&mut m, 0x0, &main_prog);

    let isr_prog = [
        lui(6, 0xE000F),        // 0x100
        sw(0, 6, 4),            // 0x104: clear SysTick->SR
        lui(6, 0x20000),        // 0x108
        lw(7, 6, 0x100),        // 0x10C
        addi(7, 7, 1),          // 0x110
        sw(7, 6, 0x100),        // 0x114
        mret(),                 // 0x118
    ];
    write_flash_prog(&mut m, 0x100, &isr_prog);

    // 62.4 MHz PLL sysclk (0x0105 -> 312 MHz / 5)
    m.bus.sys.clk_sys_cfg = 0x0105;
    m.bus.pfic.write_u32(0x100, 1 << 12); // enable SysTick IRQ
    m.bus.systick.cmp = 62_399; // 1 ms auto-reload period (62,400 cycles) @ 62.4 MHz
    m.bus.systick.ctlr = 0x0F;

    // USBHS active, attached, enumerate cleanly through EP0, NOT suspended
    m.bus.usbhs.ctrl = 0x20;
    m.bus.usbhs.pll_ctrl = 0x04;
    m.bus.usbhs.host_attached = true;
    m.bus.usbhs.ep0_dma = 0x2000_0400;
    m.bus.usbhs.ep_rx_en = 0x01;
    m.bus.usbhs.ep_tx_en = 0x01;
    let mut pending_addr = 0u8;
    let mut expect_data_in = false;
    for _ in 0..20_000 {
        m.bus.usbhs.step(1, &mut m.bus.pfic, &mut m.bus.sram);
        if (m.bus.usbhs.int_fg & 0x41) != 0 {
            m.bus.usbhs.write_u8(0x08, 0x41);
        }
        if (m.bus.usbhs.ep_rx_ctrl[0] & 0x80) != 0 {
            let setup_is = (m.bus.usbhs.ep_rx_ctrl[0] & 0x08) != 0;
            m.bus.usbhs.ep_rx_ctrl[0] &= !0x98;
            if setup_is {
                let setup = &m.bus.sram[0x400..0x408];
                let bm_req = setup[0];
                let b_req = setup[1];
                let w_val_lo = setup[2];
                let w_val_hi = setup[3];
                let w_len = u16::from_le_bytes([setup[6], setup[7]]) as usize;
                if (bm_req & 0x80) != 0 && w_len > 0 {
                    let resp: Vec<u8> = match (b_req, w_val_hi) {
                        (0x06, 1) => vec![
                            18, 1, 0x00, 0x02, 0, 0, 0, 64, 0x86, 0x1A, 0x10, 0xFE, 0x00, 0x01, 1,
                            2, 3, 1,
                        ],
                        (0x06, 2) => vec![9, 2, 9, 0, 1, 1, 0, 0x80, 50],
                        _ => vec![4, 3, b'X', 0],
                    };
                    let n = resp.len().min(w_len);
                    m.bus.sram[0x400..0x400 + n].copy_from_slice(&resp[..n]);
                    m.bus.usbhs.ep_t_len[0] = n as u16;
                    m.bus.usbhs.ep_tx_ctrl[0] = 0x02;
                    expect_data_in = true;
                } else {
                    if b_req == 0x05 {
                        pending_addr = w_val_lo;
                    }
                    m.bus.usbhs.ep_t_len[0] = 0;
                    m.bus.usbhs.ep_tx_ctrl[0] = 0x02;
                    expect_data_in = false;
                }
            }
        }
        if (m.bus.usbhs.ep_tx_ctrl[0] & 0x80) != 0 {
            m.bus.usbhs.ep_tx_ctrl[0] &= !0x80;
            if expect_data_in {
                expect_data_in = false;
                m.bus.usbhs.ep_rx_ctrl[0] = 0x02; // StatusOut
            } else if pending_addr != 0 {
                m.bus.usbhs.write_u8(0x03, pending_addr);
                pending_addr = 0;
            }
        }
        if m.bus.usbhs.enumerated {
            break;
        }
    }
    assert!(m.bus.usbhs.enumerated);
    let frame_before = m.bus.usbhs.frame_no;
    m.bus.usbhs.ep_tx_en = 0x03; // EP1 IN enabled (NAK)
    m.bus.usbhs.ep_tx_ctrl[1] = 0x00;
    m.bus.usbhs.ep_rx_en = 0x11; // EP4 OUT enabled (ACK ready)
    m.bus.usbhs.ep_rx_dma[4] = 0x2000_0300;
    m.bus.usbhs.ep_max_len[4] = 64;
    m.bus.usbhs.ep_rx_ctrl[4] = 0x02;

    // Run 10.1 ms idle so 10th SysTick ISR finishes cleanly inside the window
    m.run_us(10_100);

    assert_eq!(m.bus.power_mode, PowerMode::Idle);
    assert!(
        m.bus.sleep_skipped_ns > 8_000_000,
        "expected most of 10 ms WFI with active USBHS to fast-forward, skipped_ns={}",
        m.bus.sleep_skipped_ns
    );
    assert!(m.bus.idle_skip_stats.jumps >= 10);
    assert_eq!(read_sram_u32(&m, 0x2000_0100), 10);
    // 10.1 ms at 62.4 MHz -> 630,240 cycles -> ~80 SOF frames added
    let frame_delta = m.bus.usbhs.frame_no.wrapping_sub(frame_before);
    assert!(
        frame_delta >= 79 && frame_delta <= 82,
        "USBHS frame_delta={} should track ~80 microframes across Idle skip",
        frame_delta
    );

    // Queue EP4 OUT packet across USB-active Idle, then arm EP1 IN after clearing EP4 DONE
    m.bus.usbhs.queue_out_packet(4, vec![0x57, 0x01, 0x10, 0x00]);
    m.run_us(2_000);
    assert_eq!(
        &m.bus.sram[0x300..0x304],
        &[0x57, 0x01, 0x10, 0x00],
        "EP4 OUT packet was not written to SRAM across USB-active Idle"
    );
    m.bus.usbhs.ep_rx_ctrl[4] &= !0x80;

    m.bus.sram[0x200..0x204].copy_from_slice(&[1, 2, 3, 4]);
    m.bus.usbhs.ep_tx_dma[1] = 0x2000_0200;
    m.bus.usbhs.ep_t_len[1] = 4;
    m.bus.usbhs.ep_tx_ctrl[1] = 0x02;
    m.run_us(3_000);

    assert!(
        m.bus.usbhs.in_packets.iter().any(|p| p.ep == 1 && p.is_in && p.data == [1, 2, 3, 4]),
        "EP1 IN packet was not delivered across USB-active Idle"
    );
    assert_eq!(read_sram_u32(&m, 0x2000_0100), 15);

    // Part B: Simultaneous USB + ADC blockers must both increment independently
    let mut blk_mach = Ch585Machine::new();
    let wfi_prog = [wfi(), jal(0, -4)];
    write_flash_prog(&mut blk_mach, 0x0, &wfi_prog);
    blk_mach.bus.sys.clk_sys_cfg = 0x0105;
    blk_mach.cpu.halted_wfi = true;
    blk_mach.bus.power_mode = PowerMode::Idle;
    blk_mach.bus.usbhs.ctrl = 0x20;
    blk_mach.bus.usbhs.pll_ctrl = 0x04;
    blk_mach.bus.usbhs.host_attached = true;
    // Clear bus_reset_sent via trigger_enumerate then set attach_delay to 1 so usb_c == Some(1)
    blk_mach.bus.usbhs.trigger_enumerate();
    blk_mach.bus.usbhs.write_u8(0x00, 0x20);
    // Start a single ADC conversion (conv_cycles_rem = 16; run 15 cycles so remaining == 1)
    blk_mach.bus.adc.write_u8(0x59, 0x01);
    blk_mach.bus.adc.write_u8(0x5A, 0x01);
    blk_mach.bus.adc.advance_idle_cycles(15);
    assert_eq!(blk_mach.bus.adc.conv_cycles_remaining(), 1);
    // Set USBHS next_event_cycles to >1 vs ==1: after trigger_enumerate attach_delay=100;
    // advance 99 cycles so next cycle both USB bus reset AND ADC conversion fire!
    blk_mach.bus.usbhs.advance_idle_cycles(99, &mut blk_mach.bus.pfic);
    assert_eq!(blk_mach.bus.usbhs.next_event_cycles(true), Some(1));

    blk_mach.run_us(1);
    assert!(
        blk_mach.bus.idle_skip_stats.blocked_usb_count >= 1
            && blk_mach.bus.idle_skip_stats.blocked_usb_ns > 0,
        "expected USB blocker recorded, stats={:?}",
        blk_mach.bus.idle_skip_stats
    );
    assert!(
        blk_mach.bus.idle_skip_stats.blocked_adc_count >= 1
            && blk_mach.bus.idle_skip_stats.blocked_adc_ns > 0,
        "expected simultaneous ADC blocker recorded, stats={:?}",
        blk_mach.bus.idle_skip_stats
    );
}
