use std::cell::RefCell;
use std::path::PathBuf;
use std::rc::Rc;
use std::time::Instant;

use ch585_core::ext::{
    ExternalDevice, InputReadKind, InputReadMask, PinChangeEvent, PinContext, PinWatchMask,
};
use ch585_core::peripherals::gpio::{ExtPinDrive, ExtPull};
use ch585_core::peripherals::irq;
use ch585_core::peripherals::sys::PowerMode;
use ch585_core::soc::Ch585Machine;

// Minimal RV32I assembler helpers for deterministic core-level parity tests.
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

fn slli(rd: u8, rs1: u8, shamt: u8) -> u32 {
    ((shamt as u32 & 31) << 20) | ((rs1 as u32) << 15) | (1 << 12) | ((rd as u32) << 7) | 0x13
}

fn srli(rd: u8, rs1: u8, shamt: u8) -> u32 {
    ((shamt as u32 & 31) << 20) | ((rs1 as u32) << 15) | (5 << 12) | ((rd as u32) << 7) | 0x13
}

fn or_reg(rd: u8, rs1: u8, rs2: u8) -> u32 {
    ((rs2 as u32) << 20) | ((rs1 as u32) << 15) | (6 << 12) | ((rd as u32) << 7) | 0x33
}

fn lw(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20)
        | ((rs1 as u32) << 15)
        | (2 << 12)
        | ((rd as u32) << 7)
        | 0x03
}

fn lhu(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20)
        | ((rs1 as u32) << 15)
        | (5 << 12)
        | ((rd as u32) << 7)
        | 0x03
}

fn lbu(rd: u8, rs1: u8, imm12: i32) -> u32 {
    (((imm12 as u32) & 0xFFF) << 20)
        | ((rs1 as u32) << 15)
        | (4 << 12)
        | ((rd as u32) << 7)
        | 0x03
}

fn sb(rs2: u8, rs1: u8, imm12: i32) -> u32 {
    let imm = (imm12 as u32) & 0xFFF;
    ((imm >> 5) << 25) | ((rs2 as u32) << 20) | ((rs1 as u32) << 15) | ((imm & 0x1F) << 7) | 0x23
}

fn sh(rs2: u8, rs1: u8, imm12: i32) -> u32 {
    let imm = (imm12 as u32) & 0xFFF;
    ((imm >> 5) << 25)
        | ((rs2 as u32) << 20)
        | ((rs1 as u32) << 15)
        | (1 << 12)
        | ((imm & 0x1F) << 7)
        | 0x23
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

fn branch(funct3: u32, rs1: u8, rs2: u8, offset: i32) -> u32 {
    let imm = offset as u32;
    let b12 = (imm >> 12) & 1;
    let b11 = (imm >> 11) & 1;
    let b10_5 = (imm >> 5) & 0x3F;
    let b4_1 = (imm >> 1) & 0x0F;
    (b12 << 31)
        | (b10_5 << 25)
        | ((rs2 as u32) << 20)
        | ((rs1 as u32) << 15)
        | (funct3 << 12)
        | (b4_1 << 8)
        | (b11 << 7)
        | 0x63
}

fn beq(rs1: u8, rs2: u8, offset: i32) -> u32 {
    branch(0, rs1, rs2, offset)
}

fn bne(rs1: u8, rs2: u8, offset: i32) -> u32 {
    branch(1, rs1, rs2, offset)
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

fn read_sram_u16(mach: &Ch585Machine, vaddr: u32) -> u16 {
    let i = (vaddr - 0x2000_0000) as usize;
    u16::from_le_bytes([mach.bus.sram[i], mach.bus.sram[i + 1]])
}

// ============================================================================
// Test 1: Short back-to-back GPIO pulses inside one basic block (74HC165)
// ============================================================================

#[derive(Default, Clone)]
struct Sr74hc165Dev {
    input_bits: u16,
    shift_reg: u16,
    prev_clk: bool,
    edges: Vec<(u64, u8, bool)>,
    shared_edges: Option<Rc<RefCell<Vec<(u64, u8, bool)>>>>,
}

impl Sr74hc165Dev {
    fn new(input_bits: u16) -> Self {
        Self {
            input_bits,
            shift_reg: input_bits,
            prev_clk: false,
            edges: Vec::new(),
            shared_edges: None,
        }
    }

    fn update(&mut self, ctx: &mut PinContext<'_>) {
        let latch = ctx.read_pin('B', 5);
        let clk = ctx.read_pin('B', 6);
        if !latch {
            self.shift_reg = self.input_bits;
        } else if !self.prev_clk && clk {
            self.shift_reg = (self.shift_reg << 1) | 1;
        }
        self.prev_clk = clk;
        let drive = if (self.shift_reg & 0x8000) != 0 {
            ExtPinDrive::High
        } else {
            ExtPinDrive::Low
        };
        ctx.set_pin_drive('B', 3, drive);
    }
}

impl ExternalDevice for Sr74hc165Dev {
    fn watched_pins(&self) -> PinWatchMask {
        PinWatchMask::NONE.with_pb(5).with_pb(6)
    }

    fn on_attach(&mut self, ctx: &mut PinContext<'_>) {
        self.update(ctx);
    }

    fn on_pin_change(&mut self, ev: PinChangeEvent, ctx: &mut PinContext<'_>) {
        if ev.port == 'B' && (ev.idx == 5 || ev.idx == 6) {
            let item = (ev.time_ns, ev.idx, ev.level);
            self.edges.push(item);
            if let Some(shared) = &self.shared_edges {
                shared.borrow_mut().push(item);
            }
            self.update(ctx);
        }
    }
}

#[test]
fn test_short_back_to_back_gpio_pulses_74hc165_parity() {
    let mut p = Vec::new();
    p.push(lui(3, 0x40001));
    p.push(lui(4, 0x20000));
    p.push(addi(5, 0, 1 << 5));
    p.push(addi(6, 0, 1 << 6));
    p.push(addi(7, 0, (1 << 5) | (1 << 6)));
    p.push(sw(5, 3, 0xD8)); // PB_SET = PL#
    p.push(sw(6, 3, 0xCC)); // PB_CLR = CLK
    p.push(sw(7, 3, 0xC0)); // PB_DIR = PB5 | PB6
    p.push(sw(5, 3, 0xCC)); // PB_CLR = PL#  (1-cycle falling edge)
    p.push(sw(5, 3, 0xD8)); // PB_SET = PL#  (1-cycle rising edge)
    p.push(addi(8, 0, 0));
    p.push(addi(9, 0, 16));
    let loop_idx = p.len();
    p.push(lw(10, 3, 0xC4)); // R32_PB_PIN
    p.push(srli(10, 10, 3));
    p.push(andi(10, 10, 1));
    p.push(slli(8, 8, 1));
    p.push(or_reg(8, 8, 10));
    p.push(sw(6, 3, 0xD8)); // PB_SET = CLK (1-cycle rising edge)
    p.push(sw(6, 3, 0xCC)); // PB_CLR = CLK (1-cycle falling edge)
    p.push(addi(9, 9, -1));
    let bne_off = ((loop_idx as i32) - (p.len() as i32)) * 4;
    p.push(bne(9, 0, bne_off));
    p.push(sw(8, 4, 0x100));
    p.push(addi(11, 0, 1));
    p.push(sw(11, 4, 0x104));
    p.push(jal(0, 0));

    let setup_mach = || {
        let mut m = Ch585Machine::new();
        write_flash_prog(&mut m, 0, &p);
        m.cpu.pc = 0;
        m
    };

    // Path A: single-step (`step_with`)
    let mut m_step = setup_mach();
    let mut dev_step = Sr74hc165Dev::new(0xA55A);
    while read_sram_u32(&m_step, 0x2000_0104) == 0 && m_step.bus.total_cycles < 2_000 {
        m_step.step_with(&mut dev_step);
    }
    let val_step = read_sram_u32(&m_step, 0x2000_0100);

    // Path B: batched (`run_us_with`)
    let mut m_fast_scoped = setup_mach();
    let mut dev_scoped = Sr74hc165Dev::new(0xA55A);
    m_fast_scoped.run_us_with(50, &mut dev_scoped);
    let val_fast_scoped = read_sram_u32(&m_fast_scoped, 0x2000_0100);

    // Path C: attached device + `run_us`
    let shared_edges = Rc::new(RefCell::new(Vec::new()));
    let mut m_fast_att = setup_mach();
    let mut dev_att = Sr74hc165Dev::new(0xA55A);
    dev_att.shared_edges = Some(Rc::clone(&shared_edges));
    m_fast_att.attach_device(Box::new(dev_att));
    m_fast_att.run_us(50);
    let val_fast_att = read_sram_u32(&m_fast_att, 0x2000_0100);
    let att_edges = shared_edges.borrow().clone();

    assert_eq!(val_step, 0xA55A);
    assert_eq!(val_fast_scoped, 0xA55A);
    assert_eq!(val_fast_att, 0xA55A);

    let pl_edges: Vec<_> = dev_step.edges.iter().filter(|e| e.1 == 5).cloned().collect();
    let clk_edges: Vec<_> = dev_step.edges.iter().filter(|e| e.1 == 6).skip(1).cloned().collect();
    assert!(pl_edges.len() >= 2, "pl_edges={pl_edges:?}");
    assert_eq!(clk_edges.len(), 32, "Expected 32 CLK edges");
    assert_eq!(dev_scoped.edges, dev_step.edges);
    assert_eq!(att_edges, dev_step.edges);
}

// ============================================================================
// Test 2: Short peripheral PWM pulses (TMR1 non-DMA + TMR1 DMA WS2812 + PWMX)
// ============================================================================

#[derive(Default, Clone)]
struct PwmEdgeRecorder {
    edges: Vec<(u64, u8, bool, bool)>,
    shared: Option<Rc<RefCell<Vec<(u64, u8, bool, bool)>>>>,
}

impl ExternalDevice for PwmEdgeRecorder {
    fn watched_pins(&self) -> PinWatchMask {
        PinWatchMask::NONE.with_pb(10).with_pb(6)
    }

    fn on_pin_change(&mut self, ev: PinChangeEvent, _ctx: &mut PinContext<'_>) {
        if ev.port == 'B' && (ev.idx == 10 || ev.idx == 6) {
            let item = (ev.time_ns, ev.idx, ev.level, ev.is_output);
            self.edges.push(item);
            if let Some(shared) = &self.shared {
                shared.borrow_mut().push(item);
            }
        }
    }
}

#[test]
fn test_short_peripheral_pwm_edges_tmr1_and_pwmx_parity() {
    // Program exercises:
    //  1) TMR1 non-DMA narrow initial pulse (cnt_end=100, duty=5) on PB10
    //  2) TMR1 DMA 3-bit WS2812 burst ([22, 44, 22], cnt_end=78) on PB10
    //  3) PWMX PWM8 narrow pulse (reg_cycle=40, duty=3, div=1) mid-batch on PB6
    let mut p = Vec::new();
    p.push(lui(3, 0x40001)); // x3 = SYS/GPIO 0x4000_1000
    p.push(lui(4, 0x20000)); // x4 = SRAM 0x2000_0000
    p.push(lui(5, 0x40002));
    p.push(addi(5, 5, 0x400)); // x5 = TMR1 0x4000_2400
    p.push(lui(6, 0x40005)); // x6 = PWMX 0x4000_5000

    // Pin alternate bit1 = RB_PIN_TMR1 -> PB10; PB_DIR = (1<<10)|(1<<6)
    p.push(addi(7, 0, 2));
    p.push(sh(7, 3, 0x18));
    p.push(addi(7, 0, (1 << 10) | (1 << 6)));
    p.push(sw(7, 3, 0xC0));

    // Part A: Non-DMA TMR1 initial narrow pulse (duty=5, cnt_end=100)
    p.push(addi(7, 0, 0x02)); // ALL_CLEAR
    p.push(sb(7, 5, 0x00));
    p.push(addi(7, 0, 100));
    p.push(sw(7, 5, 0x0C)); // cnt_end
    p.push(addi(7, 0, 5));
    p.push(sw(7, 5, 0x10)); // fifo = 5
    p.push(addi(7, 0, 0x0C)); // RB_TMR_OUT_EN | RB_TMR_COUNT_EN
    p.push(sb(7, 5, 0x00));
    p.push(addi(8, 0, 65));
    let l1 = p.len();
    p.push(addi(8, 8, -1));
    p.push(bne(8, 0, ((l1 as i32) - (p.len() as i32)) * 4));
    p.push(addi(7, 0, 0x02));
    p.push(sb(7, 5, 0x00));

    // Part B: TMR1 DMA 3-bit WS2812 burst in SRAM [0x200..0x20C]
    p.push(addi(7, 0, 22));
    p.push(sw(7, 4, 0x200));
    p.push(addi(7, 0, 44));
    p.push(sw(7, 4, 0x204));
    p.push(addi(7, 0, 22));
    p.push(sw(7, 4, 0x208));
    p.push(addi(7, 0, 78));
    p.push(sw(7, 5, 0x0C)); // cnt_end = 78
    p.push(addi(7, 4, 0x200));
    p.push(sw(7, 5, 0x18)); // dma_beg
    p.push(addi(7, 4, 0x20C));
    p.push(sw(7, 5, 0x1C)); // dma_end
    p.push(addi(7, 0, 0x01));
    p.push(sb(7, 5, 0x01)); // ctrl_dma = ENABLE
    p.push(addi(7, 0, 0x0C));
    p.push(sb(7, 5, 0x00)); // ctrl_mod = OUT_EN | COUNT_EN
    p.push(addi(8, 0, 150));
    let l2 = p.len();
    p.push(addi(8, 8, -1));
    p.push(bne(8, 0, ((l2 as i32) - (p.len() as i32)) * 4));
    p.push(addi(7, 0, 0x02));
    p.push(sb(7, 5, 0x00));

    // Part C: PWMX PWM8 (ch 4) on PB6 (reg_cycle=40, mode 3 16-bit, data8_9_16[0]=3)
    p.push(addi(7, 0, 40));
    p.push(sw(7, 6, 0x14));
    p.push(addi(7, 0, 1));
    p.push(sb(7, 6, 0x03)); // clock_div = 1
    p.push(addi(7, 0, 0x0C));
    p.push(sb(7, 6, 0x02)); // mode 3
    p.push(addi(7, 0, 3));
    p.push(sh(7, 6, 0x10)); // PWM8 duty = 3
    p.push(addi(7, 0, 1 << 4));
    p.push(sb(7, 6, 0x00)); // out_en = PWM8
    p.push(addi(8, 0, 30));
    let l3 = p.len();
    p.push(addi(8, 8, -1));
    p.push(bne(8, 0, ((l3 as i32) - (p.len() as i32)) * 4));
    p.push(sb(0, 6, 0x00)); // out_en = 0
    p.push(addi(7, 0, 1));
    p.push(sw(7, 4, 0x100)); // done flag
    p.push(jal(0, 0));

    let setup_mach = || {
        let mut m = Ch585Machine::new();
        write_flash_prog(&mut m, 0, &p);
        m.cpu.pc = 0;
        m
    };

    // Path A: single-step (`step_with`)
    let mut m_step = setup_mach();
    let mut rec_step = PwmEdgeRecorder::default();
    while read_sram_u32(&m_step, 0x2000_0100) == 0 && m_step.bus.total_cycles < 3_000 {
        m_step.step_with(&mut rec_step);
    }
    for _ in 0..10 {
        m_step.step_with(&mut rec_step);
    }

    // Path B: batched (`run_us_with`)
    let mut m_scoped = setup_mach();
    let mut rec_scoped = PwmEdgeRecorder::default();
    m_scoped.run_us_with(110, &mut rec_scoped);

    // Path C: attached device + `run_us`
    let shared = Rc::new(RefCell::new(Vec::new()));
    let mut m_att = setup_mach();
    let mut rec_att = PwmEdgeRecorder::default();
    rec_att.shared = Some(Rc::clone(&shared));
    m_att.attach_device(Box::new(rec_att));
    m_att.run_us(110);
    let edges_att = shared.borrow().clone();

    let pb10_edges: Vec<_> = rec_step
        .edges
        .iter()
        .filter(|e| e.1 == 10 && e.3)
        .collect();
    let pb6_edges: Vec<_> = rec_step
        .edges
        .iter()
        .filter(|e| e.1 == 6 && e.3)
        .collect();

    // PB10: initial DIR=1 low edge, then at least 1 non-DMA pulse (2 edges) + 3 DMA pulses (6 edges)
    let pb10_pulses: Vec<_> = pb10_edges.windows(2).filter(|w| w[0].2 && !w[1].2).collect();
    assert!(
        pb10_pulses.len() >= 4,
        "Expected >= 4 high->low PWM pulses on PB10, got {} (edges={:?})",
        pb10_pulses.len(),
        pb10_edges
    );
    // PWMX PB6: high->low narrow pulse of 3 cycles
    let pb6_pulses: Vec<_> = pb6_edges.windows(2).filter(|w| w[0].2 && !w[1].2).collect();
    assert!(
        !pb6_pulses.is_empty(),
        "Expected PWM8 pulse on PB6, edges={:?}",
        pb6_edges
    );

    assert_eq!(rec_scoped.edges, rec_step.edges, "run_us_with vs step_with PWM edges mismatch");
    assert_eq!(edges_att, rec_step.edges, "attach_device+run_us vs step_with PWM edges mismatch");
}

// ============================================================================
// Test 3: Pre-read input sync (GPIOA/B + ADC + ADC DMA) & Sleep/Active next_event_ns
// ============================================================================

#[derive(Default)]
struct LazyInputSyncState {
    pa_calls: usize,
    pb_calls: usize,
    adc_calls: usize,
}

struct LazyInputSyncDev {
    shared: Rc<RefCell<LazyInputSyncState>>,
}

impl ExternalDevice for LazyInputSyncDev {
    fn watched_pins(&self) -> PinWatchMask {
        PinWatchMask::NONE
    }

    fn watched_input_reads(&self) -> InputReadMask {
        InputReadMask::ALL
    }

    fn on_before_input_read(&mut self, kind: InputReadKind, ctx: &mut PinContext<'_>) {
        let mut st = self.shared.borrow_mut();
        match kind {
            InputReadKind::GpioA => {
                let high = (st.pa_calls % 2) == 0;
                st.pa_calls += 1;
                ctx.set_pin_drive(
                    'A',
                    2,
                    if high {
                        ExtPinDrive::High
                    } else {
                        ExtPinDrive::Low
                    },
                );
            }
            InputReadKind::GpioB => {
                let high = st.pb_calls == 0;
                st.pb_calls += 1;
                ctx.set_pin_drive(
                    'B',
                    4,
                    if high {
                        ExtPinDrive::High
                    } else {
                        ExtPinDrive::Low
                    },
                );
            }
            InputReadKind::Adc => {
                let mv = match st.adc_calls {
                    0 => 2500,
                    1 => 900,
                    2 => 2100,
                    _ => 1650,
                };
                st.adc_calls += 1;
                ctx.set_adc_mv(0, mv);
            }
        }
    }
}

#[test]
fn test_pre_read_input_sync_gpio_and_adc_dma_parity() {
    // Program reads PA_PIN twice, PB_PIN twice, runs single ADC conv + read ADC_DATA,
    // and runs 2-sample ADC DMA continuous scan into SRAM 0x2000_0200.
    let mut p = Vec::new();
    p.push(lui(3, 0x40001));
    p.push(lui(4, 0x20000));
    // Read PA_PIN x2 -> SRAM 0x100, 0x104
    p.push(lw(5, 3, 0xA4));
    p.push(sw(5, 4, 0x100));
    p.push(lw(5, 3, 0xA4));
    p.push(sw(5, 4, 0x104));
    // Read PB_PIN x2 -> SRAM 0x108, 0x10C
    p.push(lw(5, 3, 0xC4));
    p.push(sw(5, 4, 0x108));
    p.push(addi(6, 0, 1));
    p.push(addi(6, 6, 1));
    p.push(lw(5, 3, 0xC4));
    p.push(sw(5, 4, 0x10C));

    // Single-shot ADC ch 0 (PA4)
    p.push(sb(0, 3, 0x58)); // channel = 0
    p.push(addi(5, 0, 0x01));
    p.push(sb(5, 3, 0x59)); // cfg = power on, -12dB gain
    p.push(sb(5, 3, 0x5A)); // convert = START
    let poll_single = p.len();
    p.push(lbu(6, 3, 0x5A));
    p.push(andi(6, 6, 1));
    p.push(bne(6, 0, ((poll_single as i32) - (p.len() as i32)) * 4));
    p.push(lhu(5, 3, 0x5C)); // read R16_ADC_DATA
    p.push(sw(5, 4, 0x110));

    // 2-sample ADC DMA scan into 0x2000_0200..0x2000_0204
    p.push(addi(5, 4, 0x200));
    p.push(sw(5, 3, 0x68)); // dma_beg
    p.push(addi(5, 4, 0x204));
    p.push(sw(5, 3, 0x6C)); // dma_end
    p.push(addi(5, 0, 0x18));
    p.push(sb(5, 3, 0x62)); // clear dma_if
    p.push(addi(5, 0, 0x41)); // DMA_EN | AUTO_LOOP? no, bit0 DMA_EN | bit6 CONT
    p.push(sb(5, 3, 0x61)); // ctrl_dma
    p.push(addi(5, 0, 0x01));
    p.push(sb(5, 3, 0x5A)); // start continuous conversions
    let poll_dma = p.len();
    p.push(lbu(6, 3, 0x62)); // R8_ADC_DMA_IF
    p.push(andi(6, 6, 0x08)); // IF_DMA_END
    p.push(beq(6, 0, ((poll_dma as i32) - (p.len() as i32)) * 4));
    p.push(sb(0, 3, 0x5A)); // stop ADC
    p.push(addi(5, 0, 1));
    p.push(sw(5, 4, 0x114)); // done
    p.push(jal(0, 0));

    let setup_mach = || {
        let mut m = Ch585Machine::new();
        write_flash_prog(&mut m, 0, &p);
        m.cpu.pc = 0;
        m
    };

    // Path A: step_with
    let st_step = Rc::new(RefCell::new(LazyInputSyncState::default()));
    let mut m_step = setup_mach();
    {
        let mut dev = LazyInputSyncDev {
            shared: Rc::clone(&st_step),
        };
        while read_sram_u32(&m_step, 0x2000_0114) == 0 && m_step.bus.total_cycles < 2_000 {
            m_step.step_with(&mut dev);
        }
    }

    // Path B: run_us_with
    let st_scoped = Rc::new(RefCell::new(LazyInputSyncState::default()));
    let mut m_scoped = setup_mach();
    {
        let mut dev = LazyInputSyncDev {
            shared: Rc::clone(&st_scoped),
        };
        m_scoped.run_us_with(50, &mut dev);
    }

    // Path C: attach_device + run_us
    let st_att = Rc::new(RefCell::new(LazyInputSyncState::default()));
    let mut m_att = setup_mach();
    m_att.attach_device(Box::new(LazyInputSyncDev {
        shared: Rc::clone(&st_att),
    }));
    m_att.run_us(50);

    let capture = |m: &Ch585Machine| {
        (
            (read_sram_u32(m, 0x2000_0100) >> 2) & 1,
            (read_sram_u32(m, 0x2000_0104) >> 2) & 1,
            (read_sram_u32(m, 0x2000_0108) >> 4) & 1,
            (read_sram_u32(m, 0x2000_010C) >> 4) & 1,
            read_sram_u32(m, 0x2000_0110),
            read_sram_u16(m, 0x2000_0200),
            read_sram_u16(m, 0x2000_0202),
        )
    };

    let c_step = capture(&m_step);
    let c_scoped = capture(&m_scoped);
    let c_att = capture(&m_att);

    assert_eq!(c_step.0, 1);
    assert_eq!(c_step.1, 0);
    assert_eq!(c_step.2, 1);
    assert_eq!(c_step.3, 0);
    // Single-shot 2500mV -> (2500+3150)*512/1050 = 2755
    assert_eq!(c_step.4, 2755);
    // DMA scan: 900mV -> 1974, 2100mV -> 2560
    assert_eq!(c_step.5, 1974);
    assert_eq!(c_step.6, 2560);

    assert_eq!(c_scoped, c_step, "run_us_with vs step_with input sync mismatch");
    assert_eq!(c_att, c_step, "attach_device+run_us vs step_with input sync mismatch");
}

struct DynamicScheduleDev {
    wake_ns: u64,
    active_ev_ns: u64,
    phase: u8,
    reached_log: Vec<(u8, u64)>,
}

impl DynamicScheduleDev {
    fn new(wake_ns: u64, active_ev_ns: u64) -> Self {
        Self {
            wake_ns,
            active_ev_ns,
            phase: 0,
            reached_log: Vec::new(),
        }
    }
}

impl ExternalDevice for DynamicScheduleDev {
    fn watched_pins(&self) -> PinWatchMask {
        PinWatchMask::NONE
    }

    fn next_event_ns(&self) -> Option<u64> {
        match self.phase {
            0 => Some(self.wake_ns),
            1 => Some(self.active_ev_ns),
            _ => None,
        }
    }

    fn on_time_reached(&mut self, ctx: &mut PinContext<'_>) {
        match self.phase {
            0 if ctx.time_ns() >= self.wake_ns => {
                self.reached_log.push((0, ctx.time_ns()));
                self.phase = 1;
                ctx.set_pin_drive('B', 0, ExtPinDrive::Low);
            }
            1 if ctx.time_ns() >= self.active_ev_ns => {
                self.reached_log.push((1, ctx.time_ns()));
                self.phase = 2;
                ctx.set_pin_drive('A', 1, ExtPinDrive::High);
            }
            _ => {}
        }
    }
}

#[test]
fn test_dynamic_next_event_ns_clamps_sleep_wfi_and_active_batch() {
    // Program puts MCU into Sleep WFI until PB0 drops, then wakes (MIE=0, RISC-V WFI wakes on
    // enabled pending IRQ), logs wakeup marker 1, then active-polls PA1 rising from next_event_ns.
    let mut p = Vec::new();
    p.push(lui(3, 0x40001));
    p.push(lui(4, 0x20000));
    p.push(wfi()); // Enter Sleep (PFIC/PWR configured on host)
    p.push(addi(5, 0, 1));
    p.push(sh(5, 3, 0x9E)); // clear PB0 int_if
    p.push(sw(5, 4, 0x100)); // wakeup marker = 1
    let poll_pa1 = p.len();
    p.push(lw(6, 3, 0xA4)); // R32_PA_PIN
    p.push(andi(6, 6, 1 << 1));
    p.push(beq(6, 0, ((poll_pa1 as i32) - (p.len() as i32)) * 4));
    p.push(addi(5, 0, 2));
    p.push(sw(5, 4, 0x104)); // active event marker = 2
    p.push(jal(0, 0));

    let setup = || {
        let mut m = Ch585Machine::new();
        write_flash_prog(&mut m, 0, &p);
        m.cpu.pc = 0;
        m.set_pin_pull('B', 0, ExtPull::PullUp);
        m.set_pin_drive('A', 1, ExtPinDrive::Low);
        m.bus.gpio.pb_int_mode = 1 << 0; // edge trigger
        m.bus.gpio.pb_out &= !(1 << 0); // falling edge
        m.bus.gpio.pb_int_en = 1 << 0;
        m.bus.pfic.write_u32(0x100, 1 << irq::GPIO_B);
        m.bus.pfic.sctlr |= 1 << 2; // SLEEPDEEP
        m.bus.sys.slp_wake_ctrl = 0x10; // RB_SLP_GPIO_WAKE
        m.bus.sys.power_plan = 0x8006; // Sleep + RAM retain
        m.bus.gpio.evaluate_interrupts(&mut m.bus.pfic);
        m
    };

    let mut m = setup();
    let mut dev = DynamicScheduleDev::new(5_000_000, 5_500_000);
    m.run_us_with(7_000, &mut dev);

    assert_eq!(read_sram_u32(&m, 0x2000_0100), 1);
    assert_eq!(read_sram_u32(&m, 0x2000_0104), 2);
    assert!(
        m.bus.sleep_skipped_ns >= 4_500_000,
        "Expected Sleep fast-forward to wake deadline, sleep_skipped_ns={}",
        m.bus.sleep_skipped_ns
    );
    assert_eq!(dev.reached_log.len(), 2);
    assert_eq!(dev.reached_log[0], (0, 5_000_000));
    assert!(dev.reached_log[1].0 == 1 && dev.reached_log[1].1 >= 5_500_000);
    assert_eq!(m.bus.power_mode, PowerMode::Active);
}

// ============================================================================
// Test 4: Rev 2.1 klavye.elf Acceptance Test (Parity + Median Speed >= 1x)
// ============================================================================

const TUS_COUNT: usize = 12;
const TUS_ADC_CH: [usize; TUS_COUNT] = [1, 0, 10, 9, 8, 7, 6, 5, 3, 2, 11, 12];

#[derive(Clone, Debug, PartialEq)]
enum BoardStimulus {
    KeyTravel(usize, f32),
    EncoderSteps(i32),
    EncButton(bool),
}

#[derive(Clone)]
struct Rev21BoardModel {
    tus_mm: [f32; TUS_COUNT],
    ktm_aktif: [bool; TUS_COUNT],
    enc_btn: bool,
    layer_btn: bool,
    vbus: bool,
    pil_mv: f32,
    analog_open: bool,
    dig_open: bool,
    v5_open: bool,

    // 74HC165 state & parity log
    sr: u16,
    son_clk: bool,
    son_latch: bool,
    sr_loads: Vec<u16>,
    sr_reads: Vec<u16>,
    sr_cur_word: u16,
    sr_bit_cnt: u8,

    // Encoder state
    enc_kuyruk: i32,
    enc_faz: u8,
    enc_next_ns: Option<u64>,

    // WS2812 edge recorder & SK6812MINI decoder
    ws_power: bool,
    ws_high: bool,
    ws_kenar_ns: u64,
    ws_bit_cnt: usize,
    ws_grb: [[u8; 3]; TUS_COUNT],
    ws_gecersiz: bool,
    ws_son_bit: bool,
    ws_latch_ns: Option<u64>,
    ws_pencere_disi: u64,
    rgb_edges: Vec<(u64, bool)>,
    rgb_frames: Vec<[[u8; 3]; TUS_COUNT]>,

    // Wakeup / PB0 ordering log
    prev_pb0_low: Option<bool>,
    wakeup_events: Vec<(u64, bool)>,

    // Timestamped scripted stimuli
    schedule: Vec<(u64, BoardStimulus)>,
    next_sched_idx: usize,
    record_logs: bool,
    adc_dirty: bool,
}

impl Rev21BoardModel {
    fn new(schedule: Vec<(u64, BoardStimulus)>, record_logs: bool) -> Self {
        Self {
            tus_mm: [0.0; TUS_COUNT],
            ktm_aktif: [false; TUS_COUNT],
            enc_btn: false,
            layer_btn: false,
            vbus: true,
            pil_mv: 3700.0,
            analog_open: false,
            dig_open: true,
            v5_open: false,
            sr: 0,
            son_clk: false,
            son_latch: true,
            sr_loads: Vec::new(),
            sr_reads: Vec::new(),
            sr_cur_word: 0,
            sr_bit_cnt: 0,
            enc_kuyruk: 0,
            enc_faz: 0,
            enc_next_ns: None,
            ws_power: false,
            ws_high: false,
            ws_kenar_ns: 0,
            ws_bit_cnt: 0,
            ws_grb: [[0; 3]; TUS_COUNT],
            ws_gecersiz: false,
            ws_son_bit: false,
            ws_latch_ns: None,
            ws_pencere_disi: 0,
            rgb_edges: Vec::new(),
            rgb_frames: Vec::new(),
            prev_pb0_low: None,
            wakeup_events: Vec::new(),
            schedule,
            next_sched_idx: 0,
            record_logs,
            adc_dirty: true,
        }
    }

    fn drv_mv(&self, idx: usize, vcc: f32) -> i32 {
        if vcc <= 0.0 {
            return 0;
        }
        let d_alt = 2.36f32;
        let yol = 3.9f32;
        let mm = self.tus_mm[idx].clamp(0.0, yol);
        let d = d_alt + (yol - mm);
        let b = (189.0f32 * (d_alt / d).powi(3)).min(169.0);
        let v = vcc / 2.0 + 12.5 * b * (vcc / 3300.0);
        v.clamp(30.0, vcc - 30.0) as i32
    }

    fn ktm_low(&self, i: usize) -> bool {
        !self.dig_open || self.ktm_aktif[i]
    }

    fn sr_paralel(&self) -> u16 {
        let lvl = |low: bool| if low { 0u16 } else { 1u16 };
        let mut u29 = 0u16;
        for i in 0..8 {
            u29 |= lvl(self.ktm_low(i)) << i;
        }
        let mut u30 = 0u16;
        for i in 0..4 {
            u30 |= lvl(self.ktm_low(8 + i)) << i;
        }
        u30 |= lvl(self.enc_btn) << 4;
        u30 |= lvl(self.layer_btn) << 5;
        (u29 << 8) | u30
    }

    fn sync_adc(&mut self, ctx: &mut PinContext<'_>) {
        if !self.adc_dirty {
            return;
        }
        self.adc_dirty = false;
        let vcc = if self.analog_open { 3300.0 } else { 0.0 };
        for i in 0..TUS_COUNT {
            ctx.set_adc_mv(TUS_ADC_CH[i], self.drv_mv(i, vcc));
        }
        ctx.set_adc_mv(4, if self.analog_open { 1650 } else { 0 });
        ctx.set_adc_mv(13, ((self.pil_mv * 470.0) / 1470.0) as i32);
    }

    fn sync_digital(&mut self, time_ns: u64, ctx: &mut PinContext<'_>) {
        for i in 0..TUS_COUNT {
            let mm = self.tus_mm[i];
            if mm >= 1.66 {
                self.ktm_aktif[i] = true;
            } else if mm <= 1.16 {
                self.ktm_aktif[i] = false;
            }
        }
        let any_low = (0..TUS_COUNT).any(|i| self.ktm_low(i));
        if self.record_logs && self.prev_pb0_low != Some(any_low) {
            self.wakeup_events.push((time_ns, any_low));
            self.prev_pb0_low = Some(any_low);
        }
        ctx.set_pin_drive(
            'B',
            0,
            if any_low {
                ExtPinDrive::Low
            } else {
                ExtPinDrive::Floating
            },
        );
        let chg_en_high = ctx.mcu_output_level('B', 7);
        let charging = self.vbus && !chg_en_high;
        ctx.set_pin_drive(
            'B',
            8,
            if charging {
                ExtPinDrive::Low
            } else {
                ExtPinDrive::Floating
            },
        );
    }

    fn sync_sr(&mut self, ctx: &mut PinContext<'_>) {
        let latch = ctx.mcu_output_level('B', 5);
        let clk = ctx.mcu_output_level('B', 6);
        if !latch {
            let p = self.sr_paralel();
            if self.record_logs && self.son_latch {
                self.sr_loads.push(p);
                self.sr_cur_word = 0;
                self.sr_bit_cnt = 0;
            }
            self.sr = p;
        } else if clk && !self.son_clk {
            if self.record_logs && self.sr_bit_cnt < 16 {
                let bit = u16::from((self.sr & 0x8000) != 0);
                self.sr_cur_word = (self.sr_cur_word << 1) | bit;
                self.sr_bit_cnt += 1;
                if self.sr_bit_cnt == 16 {
                    self.sr_reads.push(self.sr_cur_word);
                }
            }
            self.sr <<= 1;
        }
        self.son_latch = latch;
        self.son_clk = clk;
        let qh = (self.sr & 0x8000) != 0;
        ctx.set_pin_drive('B', 3, if qh { ExtPinDrive::High } else { ExtPinDrive::Low });
    }

    fn ws_kilitle(&mut self) {
        if self.ws_bit_cnt == 0 {
            return;
        }
        if !self.ws_gecersiz && self.ws_bit_cnt % 24 == 0 && self.record_logs {
            let mut frame = [[0u8; 3]; TUS_COUNT];
            for i in 0..(self.ws_bit_cnt / 24).min(TUS_COUNT) {
                let [g, r, b] = self.ws_grb[i];
                frame[i] = [r, g, b];
            }
            self.rgb_frames.push(frame);
        }
        self.ws_bit_cnt = 0;
        self.ws_gecersiz = false;
    }

    fn sync_ws(&mut self, ns: u64, high: bool, guc: bool) {
        if !guc {
            if self.ws_power {
                self.ws_bit_cnt = 0;
                self.ws_gecersiz = false;
                self.ws_latch_ns = None;
            }
            self.ws_power = false;
            return;
        }
        if !self.ws_power {
            self.ws_power = true;
            self.ws_high = high;
            self.ws_kenar_ns = ns;
            return;
        }
        let sure = ns.saturating_sub(self.ws_kenar_ns);
        if !self.ws_high && sure >= 80_000 {
            self.ws_kilitle();
            self.ws_latch_ns = None;
        }
        if self.ws_high == high {
            return;
        }
        if self.record_logs && self.rgb_edges.len() < 40_000 {
            self.rgb_edges.push((ns, high));
        }
        if self.ws_high {
            self.ws_son_bit = sure >= 450;
            if !(150..=750).contains(&sure) {
                self.ws_pencere_disi += 1;
                self.ws_gecersiz = true;
            }
            let i = self.ws_bit_cnt;
            if i < 24 * TUS_COUNT {
                let byte = &mut self.ws_grb[i / 24][i % 24 / 8];
                *byte = (*byte << 1) | u8::from(self.ws_son_bit);
            }
            self.ws_bit_cnt += 1;
            self.ws_latch_ns = Some(ns.saturating_add(80_000));
        } else {
            if self.ws_bit_cnt > 0 {
                let win = if self.ws_son_bit { 350..=900 } else { 650..=1200 };
                if !win.contains(&sure) {
                    self.ws_pencere_disi += 1;
                    self.ws_gecersiz = true;
                }
            }
            self.ws_latch_ns = None;
        }
        self.ws_high = high;
        self.ws_kenar_ns = ns;
    }

    fn init_on_machine(&mut self, m: &mut Ch585Machine) {
        m.set_pin_pull('B', 0, ExtPull::PullUp);
        m.set_pin_pull('B', 4, ExtPull::PullUp);
        m.set_pin_pull('B', 7, ExtPull::PullDown);
        m.set_pin_pull('B', 16, ExtPull::PullDown);
        m.set_pin_pull('B', 17, ExtPull::PullDown);
        m.set_pin_drive('B', 9, ExtPinDrive::High);
        m.bus.usbhs.attach();

        self.analog_open = m.bus.gpio.ext_pull_pb[16] == ExtPull::PullUp;
        self.dig_open = m.bus.gpio.ext_pull_pb[4] != ExtPull::PullUp; // PB4 PullUp => false initially
        self.v5_open = false;
        let vcc = if self.analog_open { 3300.0 } else { 0.0 };
        for i in 0..TUS_COUNT {
            m.bus.adc.channels[TUS_ADC_CH[i]] =
                ch585_core::peripherals::adc::AdcWaveform::Dc(self.drv_mv(i, vcc));
        }
        m.bus.adc.channels[4] =
            ch585_core::peripherals::adc::AdcWaveform::Dc(if self.analog_open { 1650 } else { 0 });
        m.bus.adc.channels[13] = ch585_core::peripherals::adc::AdcWaveform::Dc(
            ((self.pil_mv * 470.0) / 1470.0) as i32,
        );
        self.adc_dirty = false;
        m.set_pin_drive('B', 1, ExtPinDrive::High);
        m.set_pin_drive('B', 2, ExtPinDrive::High);
        let any_low = (0..TUS_COUNT).any(|i| self.ktm_low(i));
        if self.record_logs {
            self.prev_pb0_low = Some(any_low);
        }
        m.set_pin_drive(
            'B',
            0,
            if any_low {
                ExtPinDrive::Low
            } else {
                ExtPinDrive::Floating
            },
        );
        m.poll_ext();
    }
}

impl ExternalDevice for Rev21BoardModel {
    fn watched_pins(&self) -> PinWatchMask {
        PinWatchMask::NONE
            .with_pb(4) // PB_DIG_EN_N
            .with_pb(5) // PB_SR_LATCH
            .with_pb(6) // PB_SR_SCK
            .with_pb(7) // PB_CHG_EN
            .with_pb(9) // PB_VBUS
            .with_pb(10) // PB_NEOPIXEL
            .with_pb(16) // PB_ANALOG_EN
            .with_pb(17) // PB_TPS_EN
    }

    fn next_event_ns(&self) -> Option<u64> {
        let mut best = self
            .schedule
            .get(self.next_sched_idx)
            .map(|(at, _)| *at);
        if let Some(t) = self.enc_next_ns {
            best = Some(best.map_or(t, |b| b.min(t)));
        }
        if let Some(t) = self.ws_latch_ns {
            best = Some(best.map_or(t, |b| b.min(t)));
        }
        best
    }

    fn on_pin_change(&mut self, ev: PinChangeEvent, ctx: &mut PinContext<'_>) {
        if ev.port != 'B' {
            return;
        }
        match ev.idx {
            4 => {
                self.dig_open = !ev.level;
                self.sync_digital(ev.time_ns, ctx);
            }
            16 => {
                self.analog_open = ev.level;
                self.adc_dirty = true;
                self.sync_adc(ctx);
            }
            17 => {
                self.v5_open = ev.level && self.pil_mv > 3000.0;
                let neo = ctx.mcu_output_level('B', 10);
                self.sync_ws(ev.time_ns, neo, self.v5_open);
            }
            5 | 6 => {
                self.sync_sr(ctx);
            }
            7 | 9 => {
                self.sync_digital(ev.time_ns, ctx);
            }
            10 => {
                self.sync_ws(ev.time_ns, ev.level, self.v5_open);
            }
            _ => {}
        }
    }

    fn on_time_reached(&mut self, ctx: &mut PinContext<'_>) {
        let now = ctx.time_ns();
        let mut changed = false;
        while let Some((at_ns, stim)) = self.schedule.get(self.next_sched_idx).cloned() {
            if at_ns > now {
                break;
            }
            self.next_sched_idx += 1;
            match stim {
                BoardStimulus::KeyTravel(idx, mm) => {
                    self.tus_mm[idx] = mm;
                    self.adc_dirty = true;
                    changed = true;
                }
                BoardStimulus::EncoderSteps(steps) => {
                    self.enc_kuyruk += steps;
                    if self.enc_kuyruk != 0 && self.enc_next_ns.is_none() {
                        self.enc_next_ns = Some(now.saturating_add(1_000_000));
                    }
                }
                BoardStimulus::EncButton(pressed) => {
                    self.enc_btn = pressed;
                    changed = true;
                }
            }
        }
        if changed {
            self.sync_adc(ctx);
            self.sync_digital(now, ctx);
        }

        if let Some(enc_due) = self.enc_next_ns {
            if now >= enc_due && self.enc_kuyruk != 0 {
                if self.enc_kuyruk > 0 {
                    self.enc_faz = (self.enc_faz + 1) & 3;
                    self.enc_kuyruk -= 1;
                } else {
                    self.enc_faz = (self.enc_faz + 3) & 3;
                    self.enc_kuyruk += 1;
                }
                let (a, b) = [(false, false), (true, false), (true, true), (false, true)]
                    [self.enc_faz as usize];
                ctx.set_pin_drive('B', 1, if a { ExtPinDrive::Low } else { ExtPinDrive::High });
                ctx.set_pin_drive('B', 2, if b { ExtPinDrive::Low } else { ExtPinDrive::High });
                self.enc_next_ns = if self.enc_kuyruk != 0 {
                    Some(now.saturating_add(1_000_000))
                } else {
                    None
                };
            }
        }

        if let Some(ws_due) = self.ws_latch_ns {
            if now >= ws_due {
                let neo = ctx.mcu_output_level('B', 10);
                self.sync_ws(now, neo, self.v5_open);
            }
        }
    }
}

#[test]
fn test_phase5_klavye_rev21_acceptance_parity_and_speed() {
    let Some(klavye_path) = std::env::var_os("CH585_KLAVYE_ELF").map(PathBuf::from) else {
        eprintln!(
            "Skipping klavye.elf acceptance test: set CH585_KLAVYE_ELF to run with external firmware"
        );
        return;
    };
    if !klavye_path.exists() {
        eprintln!(
            "Skipping klavye.elf acceptance test: optional ELF fixture not found at {}",
            klavye_path.display()
        );
        return;
    }
    let elf = std::fs::read(&klavye_path).expect("read klavye.elf");

    let stimuli = vec![
        (125_000_000, BoardStimulus::KeyTravel(0, 3.0)),
        (135_000_000, BoardStimulus::EncoderSteps(2)),
        (145_000_000, BoardStimulus::KeyTravel(3, 3.0)),
        (160_000_000, BoardStimulus::KeyTravel(0, 0.0)),
        (180_000_000, BoardStimulus::KeyTravel(3, 0.0)),
        (195_000_000, BoardStimulus::EncButton(true)),
        (195_000_000, BoardStimulus::KeyTravel(5, 3.0)),
        (215_000_000, BoardStimulus::KeyTravel(5, 0.0)),
        (215_000_000, BoardStimulus::EncButton(false)),
    ];

    // 1) Reference single-step path (`step_with`)
    let mut m_step = Ch585Machine::new();
    m_step.load_elf(&elf).unwrap();
    let mut dev_step = Rev21BoardModel::new(stimuli.clone(), true);
    dev_step.init_on_machine(&mut m_step);
    let target_ns = 230_000_000u64;
    while m_step.bus.total_time_ns < target_ns {
        m_step.step_with(&mut dev_step);
    }

    // 2) Fast block path (`run_us_with`)
    let mut m_fast = Ch585Machine::new();
    m_fast.load_elf(&elf).unwrap();
    let mut dev_fast = Rev21BoardModel::new(stimuli.clone(), true);
    dev_fast.init_on_machine(&mut m_fast);
    m_fast.run_us_with(230_000, &mut dev_fast);

    // Compare HID reports (EP1 IN)
    let hid_step: Vec<Vec<u8>> = m_step
        .bus
        .usbhs
        .in_packets
        .iter()
        .filter(|p| p.is_in && p.ep == 1)
        .map(|p| p.data.clone())
        .collect();
    let hid_fast: Vec<Vec<u8>> = m_fast
        .bus
        .usbhs
        .in_packets
        .iter()
        .filter(|p| p.is_in && p.ep == 1)
        .map(|p| p.data.clone())
        .collect();
    assert!(!hid_step.is_empty(), "Expected HID reports from klavye.elf");
    assert_eq!(hid_fast, hid_step, "HID report mismatch between step and run_us_with");

    // Compare 74HC165 read words & loads
    assert!(!dev_step.sr_loads.is_empty(), "Expected 74HC165 reads from klavye.elf");
    assert_eq!(dev_fast.sr_loads, dev_step.sr_loads, "74HC165 load sequence mismatch");
    assert_eq!(dev_fast.sr_reads, dev_step.sr_reads, "74HC165 shifted bits mismatch");

    // Compare WS2812 / RGB edges & decoded frames
    assert_eq!(dev_step.ws_pencere_disi, 0, "step WS2812 timing violation");
    assert_eq!(dev_fast.ws_pencere_disi, 0, "run_us_with WS2812 timing violation");
    assert!(!dev_step.rgb_frames.is_empty(), "Expected RGB frames from klavye.elf");
    assert_eq!(dev_fast.rgb_frames, dev_step.rgb_frames, "RGB frames mismatch");
    assert_eq!(dev_fast.rgb_edges, dev_step.rgb_edges, "RGB edge timestamps mismatch");

    // Compare wakeup ordering (PB0 transitions)
    assert!(!dev_step.wakeup_events.is_empty(), "Expected wakeup events");
    assert_eq!(
        dev_fast.wakeup_events, dev_step.wakeup_events,
        "Wakeup ordering/timestamps mismatch"
    );
    assert!(m_fast.bus.unknown_mmio.is_empty(), "UnknownMMIO={:08X?}", m_fast.bus.unknown_mmio);
    assert!(m_fast.bus.gpio.contention_log.is_empty(), "Contention={:?}", m_fast.bus.gpio.contention_log);

    // 3) Measure median real-time speed over 3 x 200ms active runs after 120ms boot
    let mut speeds = Vec::with_capacity(3);
    for run_idx in 0..3 {
        let sched = vec![
            (130_000_000, BoardStimulus::KeyTravel(0, 3.0)),
            (150_000_000, BoardStimulus::EncoderSteps(2)),
            (170_000_000, BoardStimulus::KeyTravel(0, 0.0)),
            (220_000_000, BoardStimulus::KeyTravel(2, 3.0)),
            (260_000_000, BoardStimulus::KeyTravel(2, 0.0)),
        ];
        let mut m = Ch585Machine::new();
        m.load_elf(&elf).unwrap();
        let mut dev = Rev21BoardModel::new(sched, false);
        dev.init_on_machine(&mut m);
        m.run_us_with(120_000, &mut dev);

        let t0 = Instant::now();
        m.run_us_with(200_000, &mut dev);
        let wall = t0.elapsed().as_secs_f64();
        let ratio = 0.200 / wall.max(1e-9);
        eprintln!(
            "Phase5 klavye.elf + live Rev21Board run #{run_idx}: {ratio:.3}x ({wall:.4}s for 200ms)"
        );
        speeds.push(ratio);
    }
    speeds.sort_by(|a, b| a.total_cmp(b));
    let median_speed = speeds[1];
    eprintln!("Phase5 klavye.elf + live Rev21Board MEDIAN speed: {median_speed:.3}x");
    #[cfg(not(debug_assertions))]
    assert!(
        median_speed >= 1.0,
        "Expected median speed >= 1.0x real time with external board model active, got {median_speed:.3}x"
    );
}
