pub mod decode;

use crate::bus::Bus;
use crate::calibration::TimingModel;
use crate::profiler::Profiler;
use crate::soc::SysBus;
use decode::{decode_16, decode_32, Insn};

const BLOCK_CACHE_SIZE: usize = 4096;
const BLOCK_CACHE_MASK: usize = BLOCK_CACHE_SIZE - 1;
const MAX_BLOCK_OPS: usize = 16;

/// Synthetic return address pushed into `ra` when dispatching a guest callback from the host.
pub const CALLBACK_RET_SENTINEL: u32 = 0xFFFF_FFE0;

#[derive(Debug, Clone)]
pub struct PendingCallback {
    pub entry_pc: u32,
    pub args: [u32; 8],
    pub stack_blob: Vec<u8>,
    pub blob_arg_patches: Vec<(u8, u32)>,
    pub read_back: Option<(u32, u32, usize)>,
    pub tag: u32,
    pub wake_from_wfi: bool,
}

#[derive(Debug, Clone)]
pub struct CallbackResult {
    pub tag: u32,
    pub ret_a0: u32,
    pub out_data: Vec<u8>,
}

#[derive(Debug, Clone)]
pub struct CallbackFrame {
    saved_regs: [u32; 32],
    saved_pc: u32,
    saved_mstatus: u32,
    saved_halted_wfi: bool,
    blob_base: u32,
    read_back: Option<(u32, u32, usize)>,
    tag: u32,
}

const OP_NOP: u8 = 0;
const OP_LUI: u8 = 1;
const OP_ADDI: u8 = 2;
const OP_SLTI: u8 = 3;
const OP_SLTIU: u8 = 4;
const OP_XORI: u8 = 5;
const OP_ORI: u8 = 6;
const OP_ANDI: u8 = 7;
const OP_SLLI: u8 = 8;
const OP_SRLI: u8 = 9;
const OP_SRAI: u8 = 10;
const OP_ADD: u8 = 11;
const OP_SUB: u8 = 12;
const OP_SLL: u8 = 13;
const OP_SLT: u8 = 14;
const OP_SLTU: u8 = 15;
const OP_XOR: u8 = 16;
const OP_SRL: u8 = 17;
const OP_SRA: u8 = 18;
const OP_OR: u8 = 19;
const OP_AND: u8 = 20;
const OP_MUL: u8 = 21;
const OP_MULH: u8 = 22;
const OP_MULHSU: u8 = 23;
const OP_MULHU: u8 = 24;
const OP_DIV: u8 = 25;
const OP_DIVU: u8 = 26;
const OP_REM: u8 = 27;
const OP_REMU: u8 = 28;

const OP_LB: u8 = 29;
const OP_LH: u8 = 30;
const OP_LW: u8 = 31;
const OP_LBU: u8 = 32;
const OP_LHU: u8 = 33;

const OP_SB: u8 = 34;
const OP_SH: u8 = 35;
const OP_SW: u8 = 36;

const OP_JAL: u8 = 37;
const OP_JALR: u8 = 38;
const OP_BEQ: u8 = 39;
const OP_BNE: u8 = 40;
const OP_BLT: u8 = 41;
const OP_BGE: u8 = 42;
const OP_BLTU: u8 = 43;
const OP_BGEU: u8 = 44;

const OP_SEXTB: u8 = 45;
const OP_SEXTH: u8 = 46;
const OP_ZEXTH: u8 = 47;
const OP_SH1ADD: u8 = 48;
const OP_SH2ADD: u8 = 49;
const OP_SH3ADD: u8 = 50;
const OP_MIN: u8 = 51;
const OP_MINU: u8 = 52;
const OP_MAX: u8 = 53;
const OP_MAXU: u8 = 54;
const OP_ANDN: u8 = 55;
const OP_ORN: u8 = 56;
const OP_XNOR: u8 = 57;
const OP_ROL: u8 = 58;
const OP_ROR: u8 = 59;
const OP_RORI: u8 = 60;
const OP_CLZ: u8 = 61;
const OP_CTZ: u8 = 62;
const OP_CPOP: u8 = 63;
const OP_REV8: u8 = 64;

const OP_SLOW: u8 = 65;

#[repr(C)]
#[derive(Debug, Clone, Copy)]
struct FastOp {
    op: u8,
    a: u8,
    b: u8,
    c: u8,
    imm: i32,
}

impl FastOp {
    const NOP: Self = Self {
        op: OP_NOP,
        a: 0,
        b: 0,
        c: 0,
        imm: 0,
    };
}

#[derive(Debug, Clone, Copy)]
struct CachedBlock {
    tag_pc: u32,
    end_pc: u32,
    raw_head: u64,
    ops: [FastOp; MAX_BLOCK_OPS],
    len4_mask: u16,
    len: u8,
    is_self_loop: bool,
}

impl CachedBlock {
    const EMPTY: Self = Self {
        tag_pc: u32::MAX,
        end_pc: 0,
        raw_head: 0,
        ops: [FastOp::NOP; MAX_BLOCK_OPS],
        len4_mask: 0,
        len: 0,
        is_self_loop: false,
    };

    #[inline(always)]
    fn pc_at(&self, op_count: usize) -> u32 {
        let mask = if op_count >= 16 {
            self.len4_mask
        } else {
            self.len4_mask & ((1u16 << op_count) - 1)
        };
        let bytes = ((op_count as u32) + mask.count_ones()) << 1;
        self.tag_pc.wrapping_add(bytes)
    }
}

#[inline(always)]
fn read_head_u64(bus: &SysBus, pc: u32) -> u64 {
    let f = pc as usize;
    if f + 8 <= bus.flash.len() {
        return u64::from_le_bytes([
            bus.flash[f],
            bus.flash[f + 1],
            bus.flash[f + 2],
            bus.flash[f + 3],
            bus.flash[f + 4],
            bus.flash[f + 5],
            bus.flash[f + 6],
            bus.flash[f + 7],
        ]);
    }
    let so = pc.wrapping_sub(0x2000_0000) as usize;
    if so + 8 <= bus.sram.len() {
        return u64::from_le_bytes([
            bus.sram[so],
            bus.sram[so + 1],
            bus.sram[so + 2],
            bus.sram[so + 3],
            bus.sram[so + 4],
            bus.sram[so + 5],
            bus.sram[so + 6],
            bus.sram[so + 7],
        ]);
    }
    let mem = if f < bus.flash.len() {
        &bus.flash[f..]
    } else if so < bus.sram.len() {
        &bus.sram[so..]
    } else {
        return 0;
    };
    let mut buf = [0u8; 8];
    let n = mem.len().min(8);
    buf[..n].copy_from_slice(&mem[..n]);
    u64::from_le_bytes(buf)
}

#[inline(always)]
fn block_cache_idx(pc: u32) -> usize {
    (((pc >> 1) ^ (pc >> 13) ^ (pc >> 18)) as usize) & BLOCK_CACHE_MASK
}

#[inline]
fn decode_slice_at(mem: &[u8], off: usize) -> Option<(Insn, u8)> {
    if off + 2 <= mem.len() {
        let lo = u16::from_le_bytes([mem[off], mem[off + 1]]);
        if (lo & 0b11) != 0b11 {
            Some((decode_16(lo), 2))
        } else if off + 4 <= mem.len() {
            let hi = u16::from_le_bytes([mem[off + 2], mem[off + 3]]);
            Some((decode_32((lo as u32) | ((hi as u32) << 16)), 4))
        } else {
            Some((Insn::Illegal(lo as u32), 2))
        }
    } else {
        None
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Privilege {
    User = 0,
    Machine = 3,
}

#[derive(Debug, Clone)]
pub struct HpeFrame {
    pub regs: [u32; 16], // x1, x5..x7, x10..x17, x28..x31
    /// Outer interrupt's `(mepc, mstatus, mcause)` saved when this frame nested inside
    /// an already-active interrupt; `None` for depth-1 (non-nested) interrupts.
    pub nested_trap_ctx: Option<(u32, u32, u32)>,
    pub irqn: u8,
}

const HPE_REG_MAP: [usize; 16] = [
    1, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 28, 29, 30, 31,
];

#[derive(Debug, Clone)]
pub struct Cpu {
    pub regs: [u32; 32],
    pub pc: u32,
    pub priv_mode: Privilege,
    pub halted_wfi: bool,
    pub ebreak_hit: bool,

    // Standard Machine CSRs
    pub mstatus: u32,
    pub misa: u32,
    pub mtvec: u32,
    pub mscratch: u32,
    pub mepc: u32,
    pub mcause: u32,
    pub mtval: u32,

    // WCH QingKe custom CSRs
    pub intsyscr: u32, // 0x804: bit0=HWSTKEN, bit1=INESTEN, bit5=single-exit HPE pop bypass (auto-cleared on mret)
    pub corecfgr: u32, // 0xBC0
    pub corecfgr2: u32, // 0xBC1
    pub unknown_csrs: Vec<(u16, u32)>,

    // Counters
    pub cycles: u64,
    pub instret: u64,

    // Hardware Stacking Engine (HPE) stack (up to 3 levels)
    pub hpe_stack: Vec<HpeFrame>,
    /// Tracks active IRQs even when HPE is disabled (for IACTR clearing on mret)
    pub active_irqs: Vec<u8>,

    // Fast basic-block cache (invalidated on FENCE.I / reset / flash & SRAM SMC epoch)
    block_cache: Vec<CachedBlock>,
    cached_flash_version: u64,
    /// IRQ number popped by the most recent `Mret` (for profiling/timing)
    pub last_popped_irq: Option<u8>,
    /// Nested guest callback trampoline frames
    pub callback_stack: Vec<CallbackFrame>,
    /// Recent guest callback return values & captured buffers
    pub last_callback_results: Vec<CallbackResult>,
}

impl Default for Cpu {
    fn default() -> Self {
        Self::new()
    }
}

impl Cpu {
    pub fn new() -> Self {
        Self {
            regs: [0; 32],
            pc: 0,
            priv_mode: Privilege::Machine,
            halted_wfi: false,
            ebreak_hit: false,
            mstatus: 0x0000_1800, // MPP=3 initially
            misa: 0x4000_1104,    // RV32IMC
            mtvec: 0,
            mscratch: 0,
            mepc: 0,
            mcause: 0,
            mtval: 0,
            intsyscr: 0,
            corecfgr: 0,
            corecfgr2: 0,
            unknown_csrs: Vec::new(),
            cycles: 0,
            instret: 0,
            hpe_stack: Vec::new(),
            active_irqs: Vec::new(),
            block_cache: Vec::new(),
            cached_flash_version: u64::MAX,
            last_popped_irq: None,
            callback_stack: Vec::new(),
            last_callback_results: Vec::new(),
        }
    }

    pub fn reset(&mut self) {
        let mut cache = std::mem::take(&mut self.block_cache);
        if !cache.is_empty() {
            cache.fill(CachedBlock::EMPTY);
        }
        *self = Self::new();
        self.block_cache = cache;
    }

    #[inline]
    pub fn in_callback(&self) -> bool {
        !self.callback_stack.is_empty()
    }

    /// Enter a guest RISC-V callback with `ra = CALLBACK_RET_SENTINEL`.
    /// Optional `stack_blob` is placed at `[sp .. sp + aligned_len)` with pointers patched into `a0..a7`.
    pub fn enter_callback(&mut self, sram: &mut [u8], cb: PendingCallback) -> bool {
        let entry = cb.entry_pc & !1;
        if entry == 0 || self.callback_stack.len() >= 8 {
            return false;
        }
        let sp_base = if (0x2000_0200..=0x2002_0000).contains(&self.regs[2]) {
            self.regs[2] & !0xF
        } else {
            0x2001_FE00
        };
        let raw_len = cb.stack_blob.len();
        let aligned_len = if raw_len == 0 {
            0
        } else {
            (raw_len + 15) & !15
        };
        let blob_base = sp_base.saturating_sub(aligned_len as u32);
        if blob_base < 0x2000_0100 {
            return false;
        }
        if raw_len > 0 {
            let sram_off = (blob_base - 0x2000_0000) as usize;
            if sram_off + raw_len > sram.len() {
                return false;
            }
            sram[sram_off..sram_off + raw_len].copy_from_slice(&cb.stack_blob);
        }
        self.callback_stack.push(CallbackFrame {
            saved_regs: self.regs,
            saved_pc: self.pc,
            saved_mstatus: self.mstatus,
            saved_halted_wfi: if cb.wake_from_wfi {
                false
            } else {
                self.halted_wfi
            },
            blob_base,
            read_back: cb.read_back,
            tag: cb.tag,
        });
        self.regs[2] = blob_base;
        self.regs[1] = CALLBACK_RET_SENTINEL;
        for (i, &val) in cb.args.iter().enumerate() {
            self.regs[10 + i] = val;
        }
        for &(arg_idx, off) in &cb.blob_arg_patches {
            if arg_idx < 8 {
                self.regs[10 + arg_idx as usize] = blob_base.wrapping_add(off);
            }
        }
        self.pc = entry;
        self.halted_wfi = false;
        true
    }

    fn push_callback_result(&mut self, res: CallbackResult) {
        if self.last_callback_results.len() >= 64 {
            self.last_callback_results.drain(0..16);
        }
        self.last_callback_results.push(res);
    }

    /// Finish the innermost guest callback when execution returns to `CALLBACK_RET_SENTINEL`.
    pub fn complete_callback_frame(&mut self, sram: &[u8]) -> Option<CallbackResult> {
        let frame = self.callback_stack.pop()?;
        let ret_a0 = self.regs[10];
        let mut out_data = Vec::new();
        if let Some((len_off, buf_off, max_len)) = frame.read_back {
            let len_so = frame
                .blob_base
                .wrapping_add(len_off)
                .wrapping_sub(0x2000_0000) as usize;
            if len_so + 2 <= sram.len() {
                let n =
                    (u16::from_le_bytes([sram[len_so], sram[len_so + 1]]) as usize).min(max_len);
                let buf_so = frame
                    .blob_base
                    .wrapping_add(buf_off)
                    .wrapping_sub(0x2000_0000) as usize;
                if buf_so + n <= sram.len() {
                    out_data.extend_from_slice(&sram[buf_so..buf_so + n]);
                }
            }
        }
        self.regs = frame.saved_regs;
        self.pc = frame.saved_pc;
        self.mstatus = frame.saved_mstatus;
        self.halted_wfi = frame.saved_halted_wfi;
        let res = CallbackResult {
            tag: frame.tag,
            ret_a0,
            out_data,
        };
        self.push_callback_result(res.clone());
        Some(res)
    }

    fn complete_callback_frame_bus(&mut self, bus: &mut impl Bus) -> Option<CallbackResult> {
        let frame = self.callback_stack.pop()?;
        let ret_a0 = self.regs[10];
        let mut out_data = Vec::new();
        if let Some((len_off, buf_off, max_len)) = frame.read_back {
            let len_addr = frame.blob_base.wrapping_add(len_off);
            let n = (bus.read_u16(len_addr) as usize).min(max_len);
            let buf_addr = frame.blob_base.wrapping_add(buf_off);
            for i in 0..n {
                out_data.push(bus.read_u8(buf_addr.wrapping_add(i as u32)));
            }
        }
        self.regs = frame.saved_regs;
        self.pc = frame.saved_pc;
        self.mstatus = frame.saved_mstatus;
        self.halted_wfi = frame.saved_halted_wfi;
        let res = CallbackResult {
            tag: frame.tag,
            ret_a0,
            out_data,
        };
        self.push_callback_result(res.clone());
        Some(res)
    }

    #[inline]
    pub fn invalidate_decode_cache(&mut self) {
        if !self.block_cache.is_empty() {
            self.block_cache.fill(CachedBlock::EMPTY);
        }
    }

    #[inline(always)]
    pub fn set_reg(&mut self, idx: u8, val: u32) {
        if idx != 0 {
            self.regs[idx as usize] = val;
        }
    }

    #[inline]
    pub fn mie(&self) -> bool {
        (self.mstatus & (1 << 3)) != 0
    }

    #[inline]
    pub fn hpe_enabled(&self) -> bool {
        (self.intsyscr & 1) != 0
    }

    #[inline]
    pub fn nesting_enabled(&self) -> bool {
        (self.intsyscr & 2) != 0
    }

    /// Check PFIC for deliverable interrupt and enter it if allowed.
    #[inline]
    pub fn service_interrupts(&mut self, bus: &mut impl Bus) -> bool {
        let p = bus.pfic_ref();
        if p.pending_words == 0 && (!p.event_pending || (p.sctlr & (1 << 3)) == 0) {
            return false;
        }
        let nesting = self.nesting_enabled();
        let pending = bus.pfic_ref().pick_pending_irq(nesting);

        if let Some((irqn, vtf_addr)) = pending {
            // Wake from WFI on any enabled+pending interrupt regardless of MIE (RISC-V spec)
            self.halted_wfi = false;

            if irqn == 2 || self.mie() {
                self.enter_interrupt(bus, irqn, vtf_addr);
                return true;
            }
        } else if bus.pfic_ref().event_pending {
            // WFE wake event (via _SEV / PFIC->SCTLR)
            if (bus.pfic_ref().sctlr & (1 << 3)) != 0 {
                self.halted_wfi = false;
                bus.pfic().event_pending = false;
            }
        }
        false
    }

    pub fn enter_interrupt(&mut self, bus: &mut impl Bus, irqn: u8, vtf_addr: Option<u32>) {
        // Save current state before mutating active_irqs or trap CSRs
        let was_nested = !self.active_irqs.is_empty();
        let nested_trap_ctx = if was_nested {
            Some((self.mepc, self.mstatus, self.mcause))
        } else {
            None
        };
        let saved_mepc = self.pc;

        // Clear pending if edge/SW? Note: hardware clears pending or leaves peripheral flag?
        // On PFIC, IPR for IPSR-triggered or edge IRQs clears on entry (like NVIC pending->active).
        // Level-sensitive peripherals re-assert if flag isn't cleared inside ISR.
        bus.pfic().clear_pending(irqn);
        bus.pfic().set_active(irqn, true);
        bus.pfic().active_stack.push(irqn);
        self.active_irqs.push(irqn);

        if self.hpe_enabled() && self.hpe_stack.len() < 3 {
            let mut saved_regs = [0u32; 16];
            for (slot, &reg_idx) in HPE_REG_MAP.iter().enumerate() {
                saved_regs[slot] = self.regs[reg_idx];
            }
            self.hpe_stack.push(HpeFrame {
                regs: saved_regs,
                nested_trap_ctx,
                irqn,
            });
        }

        // Update mstatus: MPIE <- MIE, MIE <- 0 (or kept if hardware nesting? keep standard: MIE<-0 unless user/HW re-enables; wait: let's check!)
        let mie_bit = (self.mstatus >> 3) & 1;
        self.mstatus = (self.mstatus & !((1 << 7) | (1 << 3) | (3 << 11)))
            | (mie_bit << 7)
            | ((self.priv_mode as u32) << 11);
        self.priv_mode = Privilege::Machine;
        self.mepc = saved_mepc;
        self.mcause = 0x8000_0000 | (irqn as u32);

        self.pc = if let Some(addr) = vtf_addr {
            addr
        } else {
            self.resolve_trap_vector(irqn as u32, true, bus)
        };
    }

    pub fn enter_exception(&mut self, bus: &mut impl Bus, cause: u32, tval: u32) {
        let mie_bit = (self.mstatus >> 3) & 1;
        self.mstatus = (self.mstatus & !((1 << 7) | (1 << 3) | (3 << 11)))
            | (mie_bit << 7)
            | ((self.priv_mode as u32) << 11);
        self.priv_mode = Privilege::Machine;
        self.mepc = self.pc;
        self.mcause = cause;
        self.mtval = tval;
        // On QingKe V3C, synchronous exceptions go to EXC_IRQn(3) or mtvec base depending on mode 3
        self.pc = self.resolve_trap_vector(cause, false, bus);
    }

    fn resolve_trap_vector(&self, cause: u32, is_interrupt: bool, bus: &mut impl Bus) -> u32 {
        let mode = self.mtvec & 3;
        let base = self.mtvec & !3;
        match mode {
            0 => base,
            1 => {
                if is_interrupt {
                    base.wrapping_add(cause * 4)
                } else {
                    base
                }
            }
            3 => {
                // WCH QingKe Mode 3: Absolute Address Vector Table
                // Startup table has Reset/NMI/EXC/Ecall_M(5)/Ecall_U(8)/Break(9)/SysTick(12)/SWI(14)/external(16..)
                let table_idx = if is_interrupt {
                    cause
                } else {
                    match cause {
                        8 => 8,  // Ecall U-mode
                        11 => 5, // Ecall M-mode
                        3 => 9,  // Breakpoint
                        _ => 3,  // EXC_IRQn / HardFault
                    }
                };
                bus.read_u32(base.wrapping_add(table_idx * 4))
            }
            _ => base,
        }
    }

    fn read_csr(&self, csr: u16) -> u32 {
        match csr {
            0x300 => self.mstatus,
            0x301 => self.misa,
            0x305 => self.mtvec,
            0x340 => self.mscratch,
            0x341 => self.mepc,
            0x342 => self.mcause,
            0x343 => self.mtval,
            0x800 => self.mstatus & 0x88, // WCH gintenr: aliases MIE(3) & MPIE(7)
            0x804 => self.intsyscr,
            0xBC0 => self.corecfgr,
            0xBC1 => self.corecfgr2,
            0xB00 | 0xC00 => self.cycles as u32,
            0xB80 | 0xC80 => (self.cycles >> 32) as u32,
            0xB02 | 0xC02 => self.instret as u32,
            0xB82 | 0xC82 => (self.instret >> 32) as u32,
            0xF11 => 0x5743_4800, // mvendorid ("WCH\0")
            0xF12 => 0x0000_0000, // marchid
            0xF13 => 0x0000_0000, // mimpid
            0xF14 => 0x0000_0000, // mhartid
            _ => self
                .unknown_csrs
                .iter()
                .find(|(id, _)| *id == csr)
                .map(|(_, v)| *v)
                .unwrap_or(0),
        }
    }

    fn write_csr(&mut self, csr: u16, val: u32) {
        match csr {
            0x300 => self.mstatus = val,
            0x301 => {} // misa read-only
            0x305 => self.mtvec = val,
            0x340 => self.mscratch = val,
            0x341 => self.mepc = val,
            0x342 => self.mcause = val,
            0x343 => self.mtval = val,
            0x800 => {
                // WCH gintenr: modifies bits 3 (MIE) and 7 (MPIE) of mstatus, accessible in User mode
                self.mstatus = (self.mstatus & !0x88) | (val & 0x88);
            }
            0x804 => self.intsyscr = val,
            0xBC0 => self.corecfgr = val,
            0xBC1 => self.corecfgr2 = val,
            _ => {
                if let Some(entry) = self.unknown_csrs.iter_mut().find(|(id, _)| *id == csr) {
                    entry.1 = val;
                } else {
                    self.unknown_csrs.push((csr, val));
                }
            }
        }
    }

    /// Execute one instruction (or advance 1 WFI idle cycle). Returns cycles elapsed (typically 1).
    pub fn step(&mut self, bus: &mut impl Bus) -> u32 {
        if self.pc == CALLBACK_RET_SENTINEL && !self.callback_stack.is_empty() {
            self.complete_callback_frame_bus(bus);
            self.cycles += 1;
            self.instret += 1;
            return 1;
        }
        if self.service_interrupts(bus) {
            return 1;
        }
        if self.halted_wfi {
            self.cycles += 1;
            bus.on_wfi(1);
            return 1;
        }

        let pc = self.pc;
        let lo = bus.read_u16(pc);
        let (insn, len) = if (lo & 0b11) != 0b11 {
            (decode_16(lo), 2u32)
        } else {
            let hi = bus.read_u16(pc.wrapping_add(2));
            let word = (lo as u32) | ((hi as u32) << 16);
            (decode_32(word), 4u32)
        };

        let mut next_pc = pc.wrapping_add(len);
        self.exec_insn(insn, pc, &mut next_pc, bus);
        self.pc = next_pc;
        self.cycles += 1;
        self.instret += 1;
        1
    }

    #[inline]
    pub(crate) fn insn_load_addr(insn: &Insn, regs: &[u32; 32]) -> Option<u32> {
        match *insn {
            Insn::Lb { rs1, imm, .. }
            | Insn::Lh { rs1, imm, .. }
            | Insn::Lw { rs1, imm, .. }
            | Insn::Lbu { rs1, imm, .. }
            | Insn::Lhu { rs1, imm, .. } => Some(regs[rs1 as usize].wrapping_add(imm as u32)),
            _ => None,
        }
    }

    #[inline]
    pub(crate) fn fetch_decode_sys(&mut self, bus: &mut SysBus, pc: u32) -> (Insn, u32) {
        if let Some((insn, len)) = decode_slice_at(&bus.flash, pc as usize) {
            return (insn, len as u32);
        }
        let so = pc.wrapping_sub(0x2000_0000) as usize;
        if let Some((insn, len)) = decode_slice_at(&bus.sram, so) {
            return (insn, len as u32);
        }
        let lo = bus.read_u16(pc);
        if (lo & 0b11) != 0b11 {
            (decode_16(lo), 2u32)
        } else {
            let hi = bus.read_u16(pc.wrapping_add(2));
            let word = (lo as u32) | ((hi as u32) << 16);
            (decode_32(word), 4u32)
        }
    }

    /// Execute one instruction honoring a configurable `TimingModel` (flash/RAM wait, branch/load/IRQ penalties).
    pub fn step_sys(&mut self, bus: &mut SysBus) -> u32 {
        if self.pc == CALLBACK_RET_SENTINEL && !self.callback_stack.is_empty() {
            self.complete_callback_frame(&bus.sram);
            self.cycles += 1;
            self.instret += 1;
            return 1;
        }
        if self.service_interrupts(bus) {
            let entry_extra = bus.timing.irq_entry_cycles as u32;
            self.cycles += entry_extra as u64;
            return 1 + entry_extra;
        }
        if self.halted_wfi {
            self.cycles += 1;
            bus.on_wfi(1);
            return 1;
        }

        let pc = self.pc;
        let (insn, len) = self.fetch_decode_sys(bus, pc);
        let mem_extra = pre_insn_mem_extra(&insn, &self.regs, &bus.timing);
        let mut next_pc = pc.wrapping_add(len);
        self.exec_insn(insn, pc, &mut next_pc, bus);
        let cost = insn_timing_cycles(
            &insn,
            pc,
            next_pc,
            len,
            mem_extra,
            self.last_popped_irq.is_some(),
            &bus.timing,
        );
        self.pc = next_pc;
        self.cycles += cost as u64;
        self.instret += 1;
        cost
    }

    #[inline]
    fn eval_ble_hook(&mut self, bus: &mut SysBus, extra_cycles: u32) -> (bool, bool) {
        let approx_cycles = bus.total_cycles + extra_cycles as u64;
        let sys_hz = bus.sys.sys_clk_hz().max(32_000) as u64;
        let approx_ns = bus.total_time_ns
            + ((extra_cycles as u128).saturating_mul(1_000_000_000) / (sys_hz as u128)) as u64;
        let mut flash_dirty = false;
        let stubbed = bus.ble.on_pc(
            self,
            &mut bus.flash,
            &mut bus.sram,
            approx_ns,
            approx_cycles,
            &mut flash_dirty,
        );
        if flash_dirty {
            bus.flash_version = bus.flash_version.wrapping_add(1);
        }
        (stubbed, flash_dirty)
    }

    /// Batch execution path used when a non-ideal `TimingModel` is active.
    pub fn run_timed_batch(&mut self, bus: &mut SysBus, budget: u32, ble_has_hooks: bool) -> u32 {
        if self.halted_wfi {
            self.cycles += 1;
            bus.on_wfi(1);
            return 1;
        }
        let mut ran = 0u32;
        while ran < budget {
            if self.pc == CALLBACK_RET_SENTINEL && !self.callback_stack.is_empty() {
                self.complete_callback_frame(&bus.sram);
                ran += 1;
                bus.pending_periph_cycles += 1;
                if self.halted_wfi {
                    break;
                }
                continue;
            }
            if ble_has_hooks && bus.ble.in_hook_range(self.pc) {
                let extra = bus.pending_periph_cycles;
                let (stubbed, _) = self.eval_ble_hook(bus, extra);
                if stubbed {
                    ran += 1;
                    bus.pending_periph_cycles += 1;
                    if bus.ble.has_pending_callbacks() && self.active_irqs.is_empty() && !self.in_callback() {
                        break;
                    }
                    continue;
                }
                if bus.ble.has_pending_callbacks() && self.active_irqs.is_empty() && !self.in_callback() {
                    break;
                }
            }
            let pc = self.pc;
            let (insn, len) = self.fetch_decode_sys(bus, pc);
            if bus.ext_input_read_mask != 0 {
                if let Some(addr) = Self::insn_load_addr(&insn, &self.regs) {
                    if let Some(kind) = bus.needs_input_read_sync(addr) {
                        if bus.skip_input_sync_once {
                            bus.skip_input_sync_once = false;
                        } else {
                            bus.pending_input_read_sync = Some(kind);
                            break;
                        }
                    }
                }
            }
            let mem_extra = pre_insn_mem_extra(&insn, &self.regs, &bus.timing);
            let mut next_pc = pc.wrapping_add(len);
            self.exec_insn(insn, pc, &mut next_pc, bus);
            let cost = insn_timing_cycles(
                &insn,
                pc,
                next_pc,
                len,
                mem_extra,
                self.last_popped_irq.is_some(),
                &bus.timing,
            );
            self.pc = next_pc;
            self.cycles += cost as u64;
            self.instret += 1;
            ran += cost;
            bus.pending_periph_cycles += cost;
            if self.halted_wfi
                || bus.mmio_sync_requested
                || self.last_popped_irq.is_some()
                || (bus.pfic.pending_words != 0 && self.mie())
                || matches!(
                    insn,
                    Insn::Ecall
                        | Insn::Ebreak
                        | Insn::Wfi
                        | Insn::Mret
                        | Insn::Csrrw { .. }
                        | Insn::Csrrs { .. }
                        | Insn::Csrrc { .. }
                        | Insn::Csrrwi { .. }
                        | Insn::Csrrsi { .. }
                        | Insn::Csrrci { .. }
                )
            {
                bus.mmio_sync_requested = false;
                break;
            }
        }
        ran
    }

    /// Profiled execution path recording function calls/cycles, ISR durations, and stack depth.
    pub fn run_profiled_batch(
        &mut self,
        bus: &mut SysBus,
        profiler: &mut Profiler,
        budget: u32,
        ble_has_hooks: bool,
    ) -> u32 {
        if self.halted_wfi {
            self.cycles += 1;
            bus.on_wfi(1);
            return 1;
        }
        let sys_hz = bus.sys.sys_clk_hz().max(32_000) as u128;
        let mut ran = 0u32;
        while ran < budget {
            if self.pc == CALLBACK_RET_SENTINEL && !self.callback_stack.is_empty() {
                self.complete_callback_frame(&bus.sram);
                ran += 1;
                bus.pending_periph_cycles += 1;
                if self.halted_wfi {
                    break;
                }
                continue;
            }
            if ble_has_hooks && bus.ble.in_hook_range(self.pc) {
                let extra = bus.pending_periph_cycles;
                let (stubbed, _) = self.eval_ble_hook(bus, extra);
                if stubbed {
                    ran += 1;
                    bus.pending_periph_cycles += 1;
                    if bus.ble.has_pending_callbacks() && self.active_irqs.is_empty() && !self.in_callback() {
                        break;
                    }
                    continue;
                }
                if bus.ble.has_pending_callbacks() && self.active_irqs.is_empty() && !self.in_callback() {
                    break;
                }
            }
            let pc = self.pc;
            let (insn, len) = self.fetch_decode_sys(bus, pc);
            if bus.ext_input_read_mask != 0 {
                if let Some(addr) = Self::insn_load_addr(&insn, &self.regs) {
                    if let Some(kind) = bus.needs_input_read_sync(addr) {
                        if bus.skip_input_sync_once {
                            bus.skip_input_sync_once = false;
                        } else {
                            bus.pending_input_read_sync = Some(kind);
                            break;
                        }
                    }
                }
            }
            let is_call_entry = profiler.take_pending_call();
            let mem_extra = pre_insn_mem_extra(&insn, &self.regs, &bus.timing);
            let mut next_pc = pc.wrapping_add(len);
            self.exec_insn(insn, pc, &mut next_pc, bus);
            let cost = if bus.timing.is_ideal() {
                1
            } else {
                insn_timing_cycles(
                    &insn,
                    pc,
                    next_pc,
                    len,
                    mem_extra,
                    self.last_popped_irq.is_some(),
                    &bus.timing,
                )
            };
            self.pc = next_pc;
            self.cycles += cost as u64;
            self.instret += 1;
            ran += cost;
            bus.pending_periph_cycles += cost;

            let ns_delta = ((cost as u128) * 1_000_000_000u128 / sys_hz).max(1) as u64;
            profiler.on_block(pc, self.regs[2], cost as u64, ns_delta, is_call_entry);

            match insn {
                Insn::Jal { rd, .. } | Insn::Jalr { rd, .. } if rd == 1 || rd == 5 => {
                    profiler.mark_pending_call();
                }
                _ => {}
            }
            if let Some(popped) = self.last_popped_irq {
                let approx_offset_ns =
                    ((bus.pending_periph_cycles as u128) * 1_000_000_000u128 / sys_hz) as u64;
                profiler.on_irq_exit(popped, self.cycles, bus.total_time_ns + approx_offset_ns);
            }

            if self.halted_wfi
                || bus.mmio_sync_requested
                || self.last_popped_irq.is_some()
                || (bus.pfic.pending_words != 0 && self.mie())
                || matches!(
                    insn,
                    Insn::Ecall
                        | Insn::Ebreak
                        | Insn::Wfi
                        | Insn::Mret
                        | Insn::Csrrw { .. }
                        | Insn::Csrrs { .. }
                        | Insn::Csrrc { .. }
                        | Insn::Csrrwi { .. }
                        | Insn::Csrrsi { .. }
                        | Insn::Csrrci { .. }
                )
            {
                bus.mmio_sync_requested = false;
                break;
            }
        }
        ran
    }

    fn compile_block(
        &mut self,
        bus: &mut SysBus,
        start_pc: u32,
        ble_has_hooks: bool,
        idx: usize,
    ) -> bool {
        let in_flash = (start_pc as usize) + 2 <= bus.flash.len();
        let in_sram = if !in_flash {
            let so = start_pc.wrapping_sub(0x2000_0000) as usize;
            so + 2 <= bus.sram.len()
        } else {
            false
        };
        if !in_flash && !in_sram {
            return false;
        }

        let mut blk = CachedBlock::EMPTY;
        blk.tag_pc = start_pc;
        let mut cur_pc = start_pc;
        let mut len = 0usize;
        let single_ble_step = ble_has_hooks && bus.ble.in_hook_range(start_pc);

        while len < MAX_BLOCK_OPS {
            if len > 0 && ble_has_hooks && bus.ble.in_hook_range(cur_pc) {
                break;
            }
            let decoded = if in_flash {
                decode_slice_at(&bus.flash, cur_pc as usize)
            } else {
                let so = cur_pc.wrapping_sub(0x2000_0000) as usize;
                decode_slice_at(&bus.sram, so)
            };
            let Some((insn, ilen)) = decoded else {
                break;
            };
            if ilen == 4 {
                blk.len4_mask |= 1u16 << len;
            }
            let next_pc = cur_pc.wrapping_add(ilen as u32);
            let mut is_term = false;

            let alu_imm = |op: u8, rd: u8, rs1: u8, imm: i32| -> FastOp {
                if rd == 0 {
                    FastOp::NOP
                } else {
                    FastOp {
                        op,
                        a: rd,
                        b: rs1,
                        c: 0,
                        imm,
                    }
                }
            };
            let alu_reg = |op: u8, rd: u8, rs1: u8, rs2: u8| -> FastOp {
                if rd == 0 {
                    FastOp::NOP
                } else {
                    FastOp {
                        op,
                        a: rd,
                        b: rs1,
                        c: rs2,
                        imm: 0,
                    }
                }
            };

            let fop = match insn {
                Insn::Fence | Insn::FenceI => FastOp::NOP,
                Insn::Lui { rd, imm } => alu_imm(OP_LUI, rd, 0, imm),
                Insn::Auipc { rd, imm } => {
                    alu_imm(OP_LUI, rd, 0, cur_pc.wrapping_add(imm as u32) as i32)
                }
                Insn::Addi { rd, rs1, imm } => alu_imm(OP_ADDI, rd, rs1, imm),
                Insn::Slti { rd, rs1, imm } => alu_imm(OP_SLTI, rd, rs1, imm),
                Insn::Sltiu { rd, rs1, imm } => alu_imm(OP_SLTIU, rd, rs1, imm),
                Insn::Xori { rd, rs1, imm } => alu_imm(OP_XORI, rd, rs1, imm),
                Insn::Ori { rd, rs1, imm } => alu_imm(OP_ORI, rd, rs1, imm),
                Insn::Andi { rd, rs1, imm } => alu_imm(OP_ANDI, rd, rs1, imm),
                Insn::Slli { rd, rs1, shamt } => alu_reg(OP_SLLI, rd, rs1, shamt),
                Insn::Srli { rd, rs1, shamt } => alu_reg(OP_SRLI, rd, rs1, shamt),
                Insn::Srai { rd, rs1, shamt } => alu_reg(OP_SRAI, rd, rs1, shamt),

                Insn::Add { rd, rs1, rs2 } => alu_reg(OP_ADD, rd, rs1, rs2),
                Insn::Sub { rd, rs1, rs2 } => alu_reg(OP_SUB, rd, rs1, rs2),
                Insn::Sll { rd, rs1, rs2 } => alu_reg(OP_SLL, rd, rs1, rs2),
                Insn::Slt { rd, rs1, rs2 } => alu_reg(OP_SLT, rd, rs1, rs2),
                Insn::Sltu { rd, rs1, rs2 } => alu_reg(OP_SLTU, rd, rs1, rs2),
                Insn::Xor { rd, rs1, rs2 } => alu_reg(OP_XOR, rd, rs1, rs2),
                Insn::Srl { rd, rs1, rs2 } => alu_reg(OP_SRL, rd, rs1, rs2),
                Insn::Sra { rd, rs1, rs2 } => alu_reg(OP_SRA, rd, rs1, rs2),
                Insn::Or { rd, rs1, rs2 } => alu_reg(OP_OR, rd, rs1, rs2),
                Insn::And { rd, rs1, rs2 } => alu_reg(OP_AND, rd, rs1, rs2),

                Insn::Mul { rd, rs1, rs2 } => alu_reg(OP_MUL, rd, rs1, rs2),
                Insn::Mulh { rd, rs1, rs2 } => alu_reg(OP_MULH, rd, rs1, rs2),
                Insn::Mulhsu { rd, rs1, rs2 } => alu_reg(OP_MULHSU, rd, rs1, rs2),
                Insn::Mulhu { rd, rs1, rs2 } => alu_reg(OP_MULHU, rd, rs1, rs2),
                Insn::Div { rd, rs1, rs2 } => alu_reg(OP_DIV, rd, rs1, rs2),
                Insn::Divu { rd, rs1, rs2 } => alu_reg(OP_DIVU, rd, rs1, rs2),
                Insn::Rem { rd, rs1, rs2 } => alu_reg(OP_REM, rd, rs1, rs2),
                Insn::Remu { rd, rs1, rs2 } => alu_reg(OP_REMU, rd, rs1, rs2),

                // Zba / Zbb
                Insn::Sh1add { rd, rs1, rs2 } => alu_reg(OP_SH1ADD, rd, rs1, rs2),
                Insn::Sh2add { rd, rs1, rs2 } => alu_reg(OP_SH2ADD, rd, rs1, rs2),
                Insn::Sh3add { rd, rs1, rs2 } => alu_reg(OP_SH3ADD, rd, rs1, rs2),
                Insn::Andn { rd, rs1, rs2 } => alu_reg(OP_ANDN, rd, rs1, rs2),
                Insn::Orn { rd, rs1, rs2 } => alu_reg(OP_ORN, rd, rs1, rs2),
                Insn::Xnor { rd, rs1, rs2 } => alu_reg(OP_XNOR, rd, rs1, rs2),
                Insn::Clz { rd, rs1 } => alu_reg(OP_CLZ, rd, rs1, 0),
                Insn::Ctz { rd, rs1 } => alu_reg(OP_CTZ, rd, rs1, 0),
                Insn::Cpop { rd, rs1 } => alu_reg(OP_CPOP, rd, rs1, 0),
                Insn::Max { rd, rs1, rs2 } => alu_reg(OP_MAX, rd, rs1, rs2),
                Insn::Maxu { rd, rs1, rs2 } => alu_reg(OP_MAXU, rd, rs1, rs2),
                Insn::Min { rd, rs1, rs2 } => alu_reg(OP_MIN, rd, rs1, rs2),
                Insn::Minu { rd, rs1, rs2 } => alu_reg(OP_MINU, rd, rs1, rs2),
                Insn::SextB { rd, rs1 } => alu_reg(OP_SEXTB, rd, rs1, 0),
                Insn::SextH { rd, rs1 } => alu_reg(OP_SEXTH, rd, rs1, 0),
                Insn::ZextH { rd, rs1 } => alu_reg(OP_ZEXTH, rd, rs1, 0),
                Insn::Rol { rd, rs1, rs2 } => alu_reg(OP_ROL, rd, rs1, rs2),
                Insn::Ror { rd, rs1, rs2 } => alu_reg(OP_ROR, rd, rs1, rs2),
                Insn::Rori { rd, rs1, shamt } => alu_reg(OP_RORI, rd, rs1, shamt),
                Insn::Rev8 { rd, rs1 } => alu_reg(OP_REV8, rd, rs1, 0),

                Insn::Lb { rd, rs1, imm } => FastOp {
                    op: OP_LB,
                    a: rd,
                    b: rs1,
                    c: 0,
                    imm,
                },
                Insn::Lh { rd, rs1, imm } => FastOp {
                    op: OP_LH,
                    a: rd,
                    b: rs1,
                    c: 0,
                    imm,
                },
                Insn::Lw { rd, rs1, imm } => FastOp {
                    op: OP_LW,
                    a: rd,
                    b: rs1,
                    c: 0,
                    imm,
                },
                Insn::Lbu { rd, rs1, imm } => FastOp {
                    op: OP_LBU,
                    a: rd,
                    b: rs1,
                    c: 0,
                    imm,
                },
                Insn::Lhu { rd, rs1, imm } => FastOp {
                    op: OP_LHU,
                    a: rd,
                    b: rs1,
                    c: 0,
                    imm,
                },

                Insn::Sb { rs1, rs2, imm } => FastOp {
                    op: OP_SB,
                    a: 0,
                    b: rs1,
                    c: rs2,
                    imm,
                },
                Insn::Sh { rs1, rs2, imm } => FastOp {
                    op: OP_SH,
                    a: 0,
                    b: rs1,
                    c: rs2,
                    imm,
                },
                Insn::Sw { rs1, rs2, imm } => FastOp {
                    op: OP_SW,
                    a: 0,
                    b: rs1,
                    c: rs2,
                    imm,
                },

                Insn::Jal { rd, imm } => {
                    is_term = true;
                    let target = cur_pc.wrapping_add(imm as u32);
                    if target == start_pc && rd == 0 {
                        blk.is_self_loop = true;
                    }
                    FastOp {
                        op: OP_JAL,
                        a: rd,
                        b: 0,
                        c: 0,
                        imm: target as i32,
                    }
                }
                Insn::Jalr { rd, rs1, imm } => {
                    is_term = true;
                    FastOp {
                        op: OP_JALR,
                        a: rd,
                        b: rs1,
                        c: 0,
                        imm,
                    }
                }
                Insn::Beq { rs1, rs2, imm } => {
                    is_term = true;
                    let target = cur_pc.wrapping_add(imm as u32);
                    if target == start_pc {
                        blk.is_self_loop = true;
                    }
                    FastOp {
                        op: OP_BEQ,
                        a: 0,
                        b: rs1,
                        c: rs2,
                        imm: target as i32,
                    }
                }
                Insn::Bne { rs1, rs2, imm } => {
                    is_term = true;
                    let target = cur_pc.wrapping_add(imm as u32);
                    if target == start_pc {
                        blk.is_self_loop = true;
                    }
                    FastOp {
                        op: OP_BNE,
                        a: 0,
                        b: rs1,
                        c: rs2,
                        imm: target as i32,
                    }
                }
                Insn::Blt { rs1, rs2, imm } => {
                    is_term = true;
                    let target = cur_pc.wrapping_add(imm as u32);
                    if target == start_pc {
                        blk.is_self_loop = true;
                    }
                    FastOp {
                        op: OP_BLT,
                        a: 0,
                        b: rs1,
                        c: rs2,
                        imm: target as i32,
                    }
                }
                Insn::Bge { rs1, rs2, imm } => {
                    is_term = true;
                    let target = cur_pc.wrapping_add(imm as u32);
                    if target == start_pc {
                        blk.is_self_loop = true;
                    }
                    FastOp {
                        op: OP_BGE,
                        a: 0,
                        b: rs1,
                        c: rs2,
                        imm: target as i32,
                    }
                }
                Insn::Bltu { rs1, rs2, imm } => {
                    is_term = true;
                    let target = cur_pc.wrapping_add(imm as u32);
                    if target == start_pc {
                        blk.is_self_loop = true;
                    }
                    FastOp {
                        op: OP_BLTU,
                        a: 0,
                        b: rs1,
                        c: rs2,
                        imm: target as i32,
                    }
                }
                Insn::Bgeu { rs1, rs2, imm } => {
                    is_term = true;
                    let target = cur_pc.wrapping_add(imm as u32);
                    if target == start_pc {
                        blk.is_self_loop = true;
                    }
                    FastOp {
                        op: OP_BGEU,
                        a: 0,
                        b: rs1,
                        c: rs2,
                        imm: target as i32,
                    }
                }
                _ => {
                    is_term = true;
                    FastOp {
                        op: OP_SLOW,
                        a: ilen,
                        b: 0,
                        c: 0,
                        imm: 0,
                    }
                }
            };

            blk.ops[len] = fop;
            len += 1;
            cur_pc = next_pc;
            if is_term || single_ble_step {
                break;
            }
        }

        if len == 0 {
            return false;
        }
        blk.end_pc = cur_pc;
        blk.len = len as u8;
        blk.raw_head = read_head_u64(bus, start_pc);
        if in_sram {
            let sram_start = start_pc.wrapping_sub(0x2000_0000) as usize;
            let byte_len = cur_pc.wrapping_sub(start_pc) as usize;
            bus.mark_sram_code_range(sram_start, byte_len);
        }
        self.block_cache[idx] = blk;
        true
    }

    /// Execute up to `budget` cycles via cached basic blocks, flushing peripherals on MMIO/sync boundaries.
    pub fn run_fast_batch(&mut self, bus: &mut SysBus, budget: u32, ble_has_hooks: bool) -> u32 {
        if self.halted_wfi {
            self.cycles += 1;
            bus.on_wfi(1);
            return 1;
        }

        if self.block_cache.is_empty() {
            self.block_cache = vec![CachedBlock::EMPTY; BLOCK_CACHE_SIZE];
        }
        if bus.flash_version != self.cached_flash_version {
            self.block_cache.fill(CachedBlock::EMPTY);
            self.cached_flash_version = bus.flash_version;
        }

        let mut regs = self.regs;
        let mut pc = self.pc;
        let mut ran = 0u32;
        let mut flushed = 0u32;
        let mut stub_cycles = 0u32;
        let mut slow_pc: Option<u32> = None;
        let cur_flash_ver = self.cached_flash_version;

        const LOAD_CONT: u8 = 0;
        const LOAD_BREAK_AFTER: u8 = 1;
        const LOAD_BREAK_BEFORE: u8 = 2;

        macro_rules! fast_load {
            ($j:expr, $fop:expr, $size:expr, $read_sram:expr, $read_flash:expr, $read_bus:expr) => {{
                let addr = regs[$fop.b as usize].wrapping_add($fop.imm as u32);
                let so = addr.wrapping_sub(0x2000_0000) as usize;
                let mut status = LOAD_CONT;
                let v = if so <= SysBus::SRAM_SIZE - $size {
                    $read_sram(&bus.sram[so..so + $size])
                } else if (addr as usize) <= SysBus::FLASH_SIZE - $size {
                    let f = addr as usize;
                    $read_flash(&bus.flash[f..f + $size])
                } else {
                    let done = ran + ($j as u32);
                    if done > flushed {
                        bus.pending_periph_cycles += done - flushed;
                        flushed = done;
                    }
                    if let Some(kind) = bus.needs_input_read_sync(addr) {
                        if bus.skip_input_sync_once {
                            bus.skip_input_sync_once = false;
                            let v = $read_bus(bus, addr);
                            if bus.mmio_sync_requested {
                                status = LOAD_BREAK_AFTER;
                            }
                            v
                        } else {
                            bus.pending_input_read_sync = Some(kind);
                            status = LOAD_BREAK_BEFORE;
                            0
                        }
                    } else {
                        let v = $read_bus(bus, addr);
                        if bus.mmio_sync_requested {
                            status = LOAD_BREAK_AFTER;
                        }
                        v
                    }
                };
                if status != LOAD_BREAK_BEFORE && $fop.a != 0 {
                    regs[$fop.a as usize] = v;
                }
                status
            }};
        }

        macro_rules! fast_store {
            ($j:expr, $fop:expr, $align_mask:expr, $size:expr, $write_sram:expr, $write_bus:expr) => {{
                let addr = regs[$fop.b as usize].wrapping_add($fop.imm as u32);
                let val = regs[$fop.c as usize];
                let so = addr.wrapping_sub(0x2000_0000) as usize;
                if (addr & $align_mask) == 0 && so <= SysBus::SRAM_SIZE - $size {
                    $write_sram(&mut bus.sram[so..so + $size], val);
                    bus.on_sram_write(so, $size)
                } else {
                    let done = ran + ($j as u32);
                    if done > flushed {
                        bus.pending_periph_cycles += done - flushed;
                        flushed = done;
                    }
                    $write_bus(bus, addr, val);
                    bus.mmio_sync_requested || bus.flash_version != cur_flash_ver
                }
            }};
        }

        'outer: while ran < budget {
            if ble_has_hooks && bus.ble.in_hook_range(pc) {
                self.regs = regs;
                self.pc = pc;
                let saved_instret = self.instret;
                self.instret = saved_instret + (ran - stub_cycles) as u64;
                let extra_cyc = bus.pending_periph_cycles + (ran - flushed);
                let (stubbed, flash_dirty) = self.eval_ble_hook(bus, extra_cyc);
                self.instret = saved_instret;
                if stubbed {
                    regs = self.regs;
                    pc = self.pc;
                    ran += 1;
                    stub_cycles += 1;
                    bus.pending_periph_cycles += ran - flushed;
                    flushed = ran;
                    if flash_dirty
                        || (bus.ble.has_pending_callbacks()
                            && self.active_irqs.is_empty()
                            && !self.in_callback())
                    {
                        break 'outer;
                    }
                    continue 'outer;
                }
                if flash_dirty
                    || (bus.ble.has_pending_callbacks()
                        && self.active_irqs.is_empty()
                        && !self.in_callback())
                {
                    regs = self.regs;
                    pc = self.pc;
                    break 'outer;
                }
            }

            let idx = block_cache_idx(pc);
            let head = read_head_u64(bus, pc);
            if self.block_cache[idx].tag_pc != pc
                || self.block_cache[idx].len == 0
                || self.block_cache[idx].raw_head != head
            {
                if pc == CALLBACK_RET_SENTINEL && !self.callback_stack.is_empty() {
                    self.regs = regs;
                    self.pc = pc;
                    self.complete_callback_frame(&bus.sram);
                    regs = self.regs;
                    pc = self.pc;
                    ran += 1;
                    stub_cycles += 1;
                    if self.halted_wfi {
                        break 'outer;
                    }
                    continue 'outer;
                }
                if !self.compile_block(bus, pc, ble_has_hooks, idx) {
                    if ran > flushed {
                        bus.pending_periph_cycles += ran - flushed;
                        flushed = ran;
                    }
                    slow_pc = Some(pc);
                    break 'outer;
                }
            }

            let blk = &self.block_cache[idx];
            let blk_len = blk.len as usize;
            let blk_end = blk.end_pc;
            let is_self_loop = blk.is_self_loop;

            'self_loop: loop {
                let avail = (budget - ran) as usize;
                if avail == 0 {
                    break 'outer;
                }
                let ops_to_run = blk_len.min(avail);

                for j in 0..ops_to_run {
                    let fop = blk.ops[j];
                    match fop.op {
                        OP_NOP => {}
                        OP_LUI => regs[fop.a as usize] = fop.imm as u32,
                        OP_ADDI => {
                            regs[fop.a as usize] = regs[fop.b as usize].wrapping_add(fop.imm as u32);
                        }
                        OP_SLTI => {
                            regs[fop.a as usize] = u32::from((regs[fop.b as usize] as i32) < fop.imm);
                        }
                        OP_SLTIU => {
                            regs[fop.a as usize] = u32::from(regs[fop.b as usize] < (fop.imm as u32));
                        }
                        OP_XORI => regs[fop.a as usize] = regs[fop.b as usize] ^ (fop.imm as u32),
                        OP_ORI => regs[fop.a as usize] = regs[fop.b as usize] | (fop.imm as u32),
                        OP_ANDI => regs[fop.a as usize] = regs[fop.b as usize] & (fop.imm as u32),
                        OP_SLLI => {
                            regs[fop.a as usize] = regs[fop.b as usize].wrapping_shl(fop.c as u32);
                        }
                        OP_SRLI => {
                            regs[fop.a as usize] = regs[fop.b as usize].wrapping_shr(fop.c as u32);
                        }
                        OP_SRAI => {
                            regs[fop.a as usize] =
                                ((regs[fop.b as usize] as i32).wrapping_shr(fop.c as u32)) as u32;
                        }
                        OP_ADD => {
                            regs[fop.a as usize] =
                                regs[fop.b as usize].wrapping_add(regs[fop.c as usize]);
                        }
                        OP_SUB => {
                            regs[fop.a as usize] =
                                regs[fop.b as usize].wrapping_sub(regs[fop.c as usize]);
                        }
                        OP_SLL => {
                            regs[fop.a as usize] =
                                regs[fop.b as usize].wrapping_shl(regs[fop.c as usize] & 31);
                        }
                        OP_SLT => {
                            regs[fop.a as usize] = u32::from(
                                (regs[fop.b as usize] as i32) < (regs[fop.c as usize] as i32),
                            );
                        }
                        OP_SLTU => {
                            regs[fop.a as usize] =
                                u32::from(regs[fop.b as usize] < regs[fop.c as usize]);
                        }
                        OP_XOR => {
                            regs[fop.a as usize] = regs[fop.b as usize] ^ regs[fop.c as usize];
                        }
                        OP_SRL => {
                            regs[fop.a as usize] =
                                regs[fop.b as usize].wrapping_shr(regs[fop.c as usize] & 31);
                        }
                        OP_SRA => {
                            regs[fop.a as usize] = ((regs[fop.b as usize] as i32)
                                .wrapping_shr(regs[fop.c as usize] & 31))
                                as u32;
                        }
                        OP_OR => {
                            regs[fop.a as usize] = regs[fop.b as usize] | regs[fop.c as usize];
                        }
                        OP_AND => {
                            regs[fop.a as usize] = regs[fop.b as usize] & regs[fop.c as usize];
                        }
                        OP_MUL => {
                            regs[fop.a as usize] =
                                regs[fop.b as usize].wrapping_mul(regs[fop.c as usize]);
                        }
                        OP_MULH => {
                            let a = regs[fop.b as usize] as i32 as i64;
                            let b = regs[fop.c as usize] as i32 as i64;
                            regs[fop.a as usize] = ((a * b) >> 32) as u32;
                        }
                        OP_MULHSU => {
                            let a = regs[fop.b as usize] as i32 as i64;
                            let b = regs[fop.c as usize] as u64 as i64;
                            regs[fop.a as usize] = ((a.wrapping_mul(b)) >> 32) as u32;
                        }
                        OP_MULHU => {
                            let a = regs[fop.b as usize] as u64;
                            let b = regs[fop.c as usize] as u64;
                            regs[fop.a as usize] = ((a * b) >> 32) as u32;
                        }
                        OP_DIV => {
                            let a = regs[fop.b as usize] as i32;
                            let b = regs[fop.c as usize] as i32;
                            let res = if b == 0 {
                                -1i32
                            } else if a == i32::MIN && b == -1 {
                                i32::MIN
                            } else {
                                a.wrapping_div(b)
                            };
                            regs[fop.a as usize] = res as u32;
                        }
                        OP_DIVU => {
                            let a = regs[fop.b as usize];
                            let b = regs[fop.c as usize];
                            regs[fop.a as usize] = if b == 0 { u32::MAX } else { a / b };
                        }
                        OP_REM => {
                            let a = regs[fop.b as usize] as i32;
                            let b = regs[fop.c as usize] as i32;
                            let res = if b == 0 {
                                a
                            } else if a == i32::MIN && b == -1 {
                                0
                            } else {
                                a.wrapping_rem(b)
                            };
                            regs[fop.a as usize] = res as u32;
                        }
                        OP_REMU => {
                            let a = regs[fop.b as usize];
                            let b = regs[fop.c as usize];
                            regs[fop.a as usize] = if b == 0 { a } else { a % b };
                        }
                        OP_SH1ADD => {
                            regs[fop.a as usize] =
                                (regs[fop.b as usize] << 1).wrapping_add(regs[fop.c as usize]);
                        }
                        OP_SH2ADD => {
                            regs[fop.a as usize] =
                                (regs[fop.b as usize] << 2).wrapping_add(regs[fop.c as usize]);
                        }
                        OP_SH3ADD => {
                            regs[fop.a as usize] =
                                (regs[fop.b as usize] << 3).wrapping_add(regs[fop.c as usize]);
                        }
                        OP_ANDN => {
                            regs[fop.a as usize] = regs[fop.b as usize] & !regs[fop.c as usize];
                        }
                        OP_ORN => {
                            regs[fop.a as usize] = regs[fop.b as usize] | !regs[fop.c as usize];
                        }
                        OP_XNOR => {
                            regs[fop.a as usize] = !(regs[fop.b as usize] ^ regs[fop.c as usize]);
                        }
                        OP_CLZ => regs[fop.a as usize] = regs[fop.b as usize].leading_zeros(),
                        OP_CTZ => regs[fop.a as usize] = regs[fop.b as usize].trailing_zeros(),
                        OP_CPOP => regs[fop.a as usize] = regs[fop.b as usize].count_ones(),
                        OP_MAX => {
                            let a = regs[fop.b as usize] as i32;
                            let b = regs[fop.c as usize] as i32;
                            regs[fop.a as usize] = a.max(b) as u32;
                        }
                        OP_MAXU => {
                            regs[fop.a as usize] = regs[fop.b as usize].max(regs[fop.c as usize]);
                        }
                        OP_MIN => {
                            let a = regs[fop.b as usize] as i32;
                            let b = regs[fop.c as usize] as i32;
                            regs[fop.a as usize] = a.min(b) as u32;
                        }
                        OP_MINU => {
                            regs[fop.a as usize] = regs[fop.b as usize].min(regs[fop.c as usize]);
                        }
                        OP_SEXTB => {
                            regs[fop.a as usize] = regs[fop.b as usize] as u8 as i8 as i32 as u32;
                        }
                        OP_SEXTH => {
                            regs[fop.a as usize] = regs[fop.b as usize] as u16 as i16 as i32 as u32;
                        }
                        OP_ZEXTH => {
                            regs[fop.a as usize] = (regs[fop.b as usize] & 0xFFFF) as u32;
                        }
                        OP_ROL => {
                            regs[fop.a as usize] =
                                regs[fop.b as usize].rotate_left(regs[fop.c as usize] & 31);
                        }
                        OP_ROR => {
                            regs[fop.a as usize] =
                                regs[fop.b as usize].rotate_right(regs[fop.c as usize] & 31);
                        }
                        OP_RORI => {
                            regs[fop.a as usize] =
                                regs[fop.b as usize].rotate_right(fop.c as u32);
                        }
                        OP_REV8 => regs[fop.a as usize] = regs[fop.b as usize].swap_bytes(),

                        OP_LB => {
                            match fast_load!(
                                j,
                                fop,
                                1,
                                |s: &[u8]| s[0] as i8 as i32 as u32,
                                |s: &[u8]| s[0] as i8 as i32 as u32,
                                |b: &mut SysBus, a: u32| b.read_u8(a) as i8 as i32 as u32
                            ) {
                                LOAD_BREAK_AFTER => {
                                    ran += (j + 1) as u32;
                                    pc = blk.pc_at(j + 1);
                                    break 'outer;
                                }
                                LOAD_BREAK_BEFORE => {
                                    ran += j as u32;
                                    pc = blk.pc_at(j);
                                    break 'outer;
                                }
                                _ => {}
                            }
                        }
                        OP_LH => {
                            match fast_load!(
                                j,
                                fop,
                                2,
                                |s: &[u8]| u16::from_le_bytes([s[0], s[1]]) as i16 as i32 as u32,
                                |s: &[u8]| u16::from_le_bytes([s[0], s[1]]) as i16 as i32 as u32,
                                |b: &mut SysBus, a: u32| b.read_u16(a) as i16 as i32 as u32
                            ) {
                                LOAD_BREAK_AFTER => {
                                    ran += (j + 1) as u32;
                                    pc = blk.pc_at(j + 1);
                                    break 'outer;
                                }
                                LOAD_BREAK_BEFORE => {
                                    ran += j as u32;
                                    pc = blk.pc_at(j);
                                    break 'outer;
                                }
                                _ => {}
                            }
                        }
                        OP_LW => {
                            match fast_load!(
                                j,
                                fop,
                                4,
                                |s: &[u8]| u32::from_le_bytes([s[0], s[1], s[2], s[3]]),
                                |s: &[u8]| u32::from_le_bytes([s[0], s[1], s[2], s[3]]),
                                |b: &mut SysBus, a: u32| b.read_u32(a)
                            ) {
                                LOAD_BREAK_AFTER => {
                                    ran += (j + 1) as u32;
                                    pc = blk.pc_at(j + 1);
                                    break 'outer;
                                }
                                LOAD_BREAK_BEFORE => {
                                    ran += j as u32;
                                    pc = blk.pc_at(j);
                                    break 'outer;
                                }
                                _ => {}
                            }
                        }
                        OP_LBU => {
                            match fast_load!(
                                j,
                                fop,
                                1,
                                |s: &[u8]| s[0] as u32,
                                |s: &[u8]| s[0] as u32,
                                |b: &mut SysBus, a: u32| b.read_u8(a) as u32
                            ) {
                                LOAD_BREAK_AFTER => {
                                    ran += (j + 1) as u32;
                                    pc = blk.pc_at(j + 1);
                                    break 'outer;
                                }
                                LOAD_BREAK_BEFORE => {
                                    ran += j as u32;
                                    pc = blk.pc_at(j);
                                    break 'outer;
                                }
                                _ => {}
                            }
                        }
                        OP_LHU => {
                            match fast_load!(
                                j,
                                fop,
                                2,
                                |s: &[u8]| u16::from_le_bytes([s[0], s[1]]) as u32,
                                |s: &[u8]| u16::from_le_bytes([s[0], s[1]]) as u32,
                                |b: &mut SysBus, a: u32| b.read_u16(a) as u32
                            ) {
                                LOAD_BREAK_AFTER => {
                                    ran += (j + 1) as u32;
                                    pc = blk.pc_at(j + 1);
                                    break 'outer;
                                }
                                LOAD_BREAK_BEFORE => {
                                    ran += j as u32;
                                    pc = blk.pc_at(j);
                                    break 'outer;
                                }
                                _ => {}
                            }
                        }

                        OP_SB => {
                            if fast_store!(
                                j,
                                fop,
                                0,
                                1,
                                |s: &mut [u8], v: u32| s[0] = v as u8,
                                |b: &mut SysBus, a: u32, v: u32| b.write_u8(a, v as u8)
                            ) {
                                ran += (j + 1) as u32;
                                pc = blk.pc_at(j + 1);
                                break 'outer;
                            }
                        }
                        OP_SH => {
                            if fast_store!(
                                j,
                                fop,
                                1,
                                2,
                                |s: &mut [u8], v: u32| s.copy_from_slice(&(v as u16).to_le_bytes()),
                                |b: &mut SysBus, a: u32, v: u32| b.write_u16(a, v as u16)
                            ) {
                                ran += (j + 1) as u32;
                                pc = blk.pc_at(j + 1);
                                break 'outer;
                            }
                        }
                        OP_SW => {
                            if fast_store!(
                                j,
                                fop,
                                3,
                                4,
                                |s: &mut [u8], v: u32| s.copy_from_slice(&v.to_le_bytes()),
                                |b: &mut SysBus, a: u32, v: u32| b.write_u32(a, v)
                            ) {
                                ran += (j + 1) as u32;
                                pc = blk.pc_at(j + 1);
                                break 'outer;
                            }
                        }

                        OP_JAL => {
                            if fop.a != 0 {
                                regs[fop.a as usize] = blk_end;
                            }
                            ran += (j + 1) as u32;
                            if is_self_loop && ran < budget {
                                continue 'self_loop;
                            }
                            pc = fop.imm as u32;
                            continue 'outer;
                        }
                        OP_JALR => {
                            let target = regs[fop.b as usize].wrapping_add(fop.imm as u32) & !1;
                            if fop.a != 0 {
                                regs[fop.a as usize] = blk_end;
                            }
                            ran += (j + 1) as u32;
                            pc = target;
                            continue 'outer;
                        }
                        OP_BEQ => {
                            ran += (j + 1) as u32;
                            if regs[fop.b as usize] == regs[fop.c as usize] {
                                if is_self_loop && ran < budget {
                                    continue 'self_loop;
                                }
                                pc = fop.imm as u32;
                            } else {
                                pc = blk_end;
                            }
                            continue 'outer;
                        }
                        OP_BNE => {
                            ran += (j + 1) as u32;
                            if regs[fop.b as usize] != regs[fop.c as usize] {
                                if is_self_loop && ran < budget {
                                    continue 'self_loop;
                                }
                                pc = fop.imm as u32;
                            } else {
                                pc = blk_end;
                            }
                            continue 'outer;
                        }
                        OP_BLT => {
                            ran += (j + 1) as u32;
                            if (regs[fop.b as usize] as i32) < (regs[fop.c as usize] as i32) {
                                if is_self_loop && ran < budget {
                                    continue 'self_loop;
                                }
                                pc = fop.imm as u32;
                            } else {
                                pc = blk_end;
                            }
                            continue 'outer;
                        }
                        OP_BGE => {
                            ran += (j + 1) as u32;
                            if (regs[fop.b as usize] as i32) >= (regs[fop.c as usize] as i32) {
                                if is_self_loop && ran < budget {
                                    continue 'self_loop;
                                }
                                pc = fop.imm as u32;
                            } else {
                                pc = blk_end;
                            }
                            continue 'outer;
                        }
                        OP_BLTU => {
                            ran += (j + 1) as u32;
                            if regs[fop.b as usize] < regs[fop.c as usize] {
                                if is_self_loop && ran < budget {
                                    continue 'self_loop;
                                }
                                pc = fop.imm as u32;
                            } else {
                                pc = blk_end;
                            }
                            continue 'outer;
                        }
                        OP_BGEU => {
                            ran += (j + 1) as u32;
                            if regs[fop.b as usize] >= regs[fop.c as usize] {
                                if is_self_loop && ran < budget {
                                    continue 'self_loop;
                                }
                                pc = fop.imm as u32;
                            } else {
                                pc = blk_end;
                            }
                            continue 'outer;
                        }
                        _ => {
                            ran += j as u32;
                            slow_pc = Some(blk.pc_at(j));
                            break 'outer;
                        }
                    }
                }

                ran += ops_to_run as u32;
                if ops_to_run == blk_len {
                    pc = blk_end;
                    continue 'outer;
                } else {
                    pc = blk.pc_at(ops_to_run);
                    break 'outer;
                }
            }
        }

        self.regs = regs;
        self.pc = pc;
        self.cycles += (ran - stub_cycles) as u64;
        self.instret += (ran - stub_cycles) as u64;
        if ran > flushed {
            bus.pending_periph_cycles += ran - flushed;
        }

        if let Some(spc) = slow_pc {
            let lo = bus.read_u16(spc);
            let (insn, ilen) = if (lo & 0b11) != 0b11 {
                (decode_16(lo), 2u32)
            } else {
                let hi = bus.read_u16(spc.wrapping_add(2));
                (decode_32((lo as u32) | ((hi as u32) << 16)), 4u32)
            };
            let mut next_pc = spc.wrapping_add(ilen);
            self.exec_insn(insn, spc, &mut next_pc, bus);
            self.pc = next_pc;
            self.cycles += 1;
            self.instret += 1;
            bus.pending_periph_cycles += 1;
            return ran + 1;
        }

        ran
    }

    #[inline]
    fn exec_insn(&mut self, insn: Insn, pc: u32, next_pc: &mut u32, bus: &mut impl Bus) {
        self.last_popped_irq = None;
        let r = &self.regs;
        match insn {
            Insn::Lui { rd, imm } => self.set_reg(rd, imm as u32),
            Insn::Auipc { rd, imm } => self.set_reg(rd, pc.wrapping_add(imm as u32)),
            Insn::Jal { rd, imm } => {
                self.set_reg(rd, *next_pc);
                *next_pc = pc.wrapping_add(imm as u32);
            }
            Insn::Jalr { rd, rs1, imm } => {
                let target = r[rs1 as usize].wrapping_add(imm as u32) & !1;
                self.set_reg(rd, *next_pc);
                *next_pc = target;
            }
            Insn::Beq { rs1, rs2, imm } => {
                if r[rs1 as usize] == r[rs2 as usize] {
                    *next_pc = pc.wrapping_add(imm as u32);
                }
            }
            Insn::Bne { rs1, rs2, imm } => {
                if r[rs1 as usize] != r[rs2 as usize] {
                    *next_pc = pc.wrapping_add(imm as u32);
                }
            }
            Insn::Blt { rs1, rs2, imm } => {
                if (r[rs1 as usize] as i32) < (r[rs2 as usize] as i32) {
                    *next_pc = pc.wrapping_add(imm as u32);
                }
            }
            Insn::Bge { rs1, rs2, imm } => {
                if (r[rs1 as usize] as i32) >= (r[rs2 as usize] as i32) {
                    *next_pc = pc.wrapping_add(imm as u32);
                }
            }
            Insn::Bltu { rs1, rs2, imm } => {
                if r[rs1 as usize] < r[rs2 as usize] {
                    *next_pc = pc.wrapping_add(imm as u32);
                }
            }
            Insn::Bgeu { rs1, rs2, imm } => {
                if r[rs1 as usize] >= r[rs2 as usize] {
                    *next_pc = pc.wrapping_add(imm as u32);
                }
            }
            Insn::Lb { rd, rs1, imm } => {
                let addr = r[rs1 as usize].wrapping_add(imm as u32);
                let v = bus.read_u8(addr) as i8 as i32 as u32;
                self.set_reg(rd, v);
            }
            Insn::Lh { rd, rs1, imm } => {
                let addr = r[rs1 as usize].wrapping_add(imm as u32);
                let v = bus.read_u16(addr) as i16 as i32 as u32;
                self.set_reg(rd, v);
            }
            Insn::Lw { rd, rs1, imm } => {
                let addr = r[rs1 as usize].wrapping_add(imm as u32);
                let v = bus.read_u32(addr);
                self.set_reg(rd, v);
            }
            Insn::Lbu { rd, rs1, imm } => {
                let addr = r[rs1 as usize].wrapping_add(imm as u32);
                let v = bus.read_u8(addr) as u32;
                self.set_reg(rd, v);
            }
            Insn::Lhu { rd, rs1, imm } => {
                let addr = r[rs1 as usize].wrapping_add(imm as u32);
                let v = bus.read_u16(addr) as u32;
                self.set_reg(rd, v);
            }
            Insn::Sb { rs1, rs2, imm } => {
                let addr = r[rs1 as usize].wrapping_add(imm as u32);
                bus.write_u8(addr, r[rs2 as usize] as u8);
            }
            Insn::Sh { rs1, rs2, imm } => {
                let addr = r[rs1 as usize].wrapping_add(imm as u32);
                bus.write_u16(addr, r[rs2 as usize] as u16);
            }
            Insn::Sw { rs1, rs2, imm } => {
                let addr = r[rs1 as usize].wrapping_add(imm as u32);
                bus.write_u32(addr, r[rs2 as usize]);
            }
            Insn::Addi { rd, rs1, imm } => {
                self.set_reg(rd, r[rs1 as usize].wrapping_add(imm as u32));
            }
            Insn::Slti { rd, rs1, imm } => {
                self.set_reg(rd, u32::from((r[rs1 as usize] as i32) < imm));
            }
            Insn::Sltiu { rd, rs1, imm } => {
                self.set_reg(rd, u32::from(r[rs1 as usize] < (imm as u32)));
            }
            Insn::Xori { rd, rs1, imm } => {
                self.set_reg(rd, r[rs1 as usize] ^ (imm as u32));
            }
            Insn::Ori { rd, rs1, imm } => {
                self.set_reg(rd, r[rs1 as usize] | (imm as u32));
            }
            Insn::Andi { rd, rs1, imm } => {
                self.set_reg(rd, r[rs1 as usize] & (imm as u32));
            }
            Insn::Slli { rd, rs1, shamt } => {
                self.set_reg(rd, r[rs1 as usize].wrapping_shl(shamt as u32));
            }
            Insn::Srli { rd, rs1, shamt } => {
                self.set_reg(rd, r[rs1 as usize].wrapping_shr(shamt as u32));
            }
            Insn::Srai { rd, rs1, shamt } => {
                self.set_reg(rd, ((r[rs1 as usize] as i32).wrapping_shr(shamt as u32)) as u32);
            }
            Insn::Add { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].wrapping_add(r[rs2 as usize]));
            }
            Insn::Sub { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].wrapping_sub(r[rs2 as usize]));
            }
            Insn::Sll { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].wrapping_shl(r[rs2 as usize] & 31));
            }
            Insn::Slt { rd, rs1, rs2 } => {
                self.set_reg(
                    rd,
                    u32::from((r[rs1 as usize] as i32) < (r[rs2 as usize] as i32)),
                );
            }
            Insn::Sltu { rd, rs1, rs2 } => {
                self.set_reg(rd, u32::from(r[rs1 as usize] < r[rs2 as usize]));
            }
            Insn::Xor { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize] ^ r[rs2 as usize]);
            }
            Insn::Srl { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].wrapping_shr(r[rs2 as usize] & 31));
            }
            Insn::Sra { rd, rs1, rs2 } => {
                self.set_reg(
                    rd,
                    ((r[rs1 as usize] as i32).wrapping_shr(r[rs2 as usize] & 31)) as u32,
                );
            }
            Insn::Or { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize] | r[rs2 as usize]);
            }
            Insn::And { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize] & r[rs2 as usize]);
            }
            Insn::Fence => {}
            Insn::FenceI => {
                self.invalidate_decode_cache();
            }
            Insn::Ecall => {
                let cause = match self.priv_mode {
                    Privilege::User => 8,
                    Privilege::Machine => 11,
                };
                self.enter_exception(bus, cause, 0);
                *next_pc = self.pc;
            }
            Insn::Ebreak => {
                self.ebreak_hit = true;
                self.enter_exception(bus, 3, pc);
                *next_pc = self.pc;
            }
            Insn::Wfi => {
                self.halted_wfi = true;
            }
            Insn::Mret => {
                let suppress_hpe_pop = (self.intsyscr & (1 << 5)) != 0;
                let ret_pc = self.mepc;
                let mpie = (self.mstatus >> 7) & 1;
                let mpp = (self.mstatus >> 11) & 3;
                let ret_priv = if mpp == 3 {
                    Privilege::Machine
                } else {
                    Privilege::User
                };

                // Pop PFIC active IRQ and matching HPE frame (if any)
                let (popped_irq, nested_ctx) = if let Some(frame) = self.hpe_stack.last() {
                    if self.active_irqs.last().copied() == Some(frame.irqn) {
                        let f = self.hpe_stack.pop().unwrap();
                        if !suppress_hpe_pop {
                            for (slot, &reg_idx) in HPE_REG_MAP.iter().enumerate() {
                                self.regs[reg_idx] = f.regs[slot];
                            }
                        }
                        self.active_irqs.pop();
                        (Some(f.irqn), f.nested_trap_ctx)
                    } else {
                        (self.active_irqs.pop(), None)
                    }
                } else {
                    (self.active_irqs.pop(), None)
                };

                if suppress_hpe_pop {
                    self.intsyscr &= !(1 << 5);
                }

                if let Some(irqn) = popped_irq {
                    bus.pfic().set_active(irqn, false);
                    if bus.pfic_ref().active_stack.last().copied() == Some(irqn) {
                        bus.pfic().active_stack.pop();
                    }
                }
                self.last_popped_irq = popped_irq;
                self.priv_mode = ret_priv;

                if !suppress_hpe_pop {
                    if let Some((outer_mepc, outer_mstatus, outer_mcause)) = nested_ctx {
                        self.mepc = outer_mepc;
                        self.mcause = outer_mcause;
                        self.mstatus = (outer_mstatus & !(1 << 3)) | (mpie << 3);
                    } else {
                        self.mstatus = (self.mstatus & !((1 << 3) | (3 << 11)))
                            | (mpie << 3)
                            | (1 << 7);
                    }
                } else {
                    self.mstatus = (self.mstatus & !((1 << 3) | (3 << 11)))
                        | (mpie << 3)
                        | (1 << 7);
                }
                *next_pc = ret_pc;
            }
            Insn::Csrrw { rd, rs1, csr } => {
                let old = self.read_csr(csr);
                let new = r[rs1 as usize];
                self.write_csr(csr, new);
                self.set_reg(rd, old);
            }
            Insn::Csrrs { rd, rs1, csr } => {
                let old = self.read_csr(csr);
                if rs1 != 0 {
                    self.write_csr(csr, old | r[rs1 as usize]);
                }
                self.set_reg(rd, old);
            }
            Insn::Csrrc { rd, rs1, csr } => {
                let old = self.read_csr(csr);
                if rs1 != 0 {
                    self.write_csr(csr, old & !r[rs1 as usize]);
                }
                self.set_reg(rd, old);
            }
            Insn::Csrrwi { rd, zimm, csr } => {
                let old = self.read_csr(csr);
                self.write_csr(csr, zimm as u32);
                self.set_reg(rd, old);
            }
            Insn::Csrrsi { rd, zimm, csr } => {
                let old = self.read_csr(csr);
                if zimm != 0 {
                    self.write_csr(csr, old | (zimm as u32));
                }
                self.set_reg(rd, old);
            }
            Insn::Csrrci { rd, zimm, csr } => {
                let old = self.read_csr(csr);
                if zimm != 0 {
                    self.write_csr(csr, old & !(zimm as u32));
                }
                self.set_reg(rd, old);
            }

            // M extension
            Insn::Mul { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].wrapping_mul(r[rs2 as usize]));
            }
            Insn::Mulh { rd, rs1, rs2 } => {
                let a = r[rs1 as usize] as i32 as i64;
                let b = r[rs2 as usize] as i32 as i64;
                self.set_reg(rd, ((a * b) >> 32) as u32);
            }
            Insn::Mulhsu { rd, rs1, rs2 } => {
                let a = r[rs1 as usize] as i32 as i64;
                let b = r[rs2 as usize] as u64 as i64;
                self.set_reg(rd, ((a.wrapping_mul(b)) >> 32) as u32);
            }
            Insn::Mulhu { rd, rs1, rs2 } => {
                let a = r[rs1 as usize] as u64;
                let b = r[rs2 as usize] as u64;
                self.set_reg(rd, ((a * b) >> 32) as u32);
            }
            Insn::Div { rd, rs1, rs2 } => {
                let a = r[rs1 as usize] as i32;
                let b = r[rs2 as usize] as i32;
                let res = if b == 0 {
                    -1i32
                } else if a == i32::MIN && b == -1 {
                    i32::MIN
                } else {
                    a.wrapping_div(b)
                };
                self.set_reg(rd, res as u32);
            }
            Insn::Divu { rd, rs1, rs2 } => {
                let a = r[rs1 as usize];
                let b = r[rs2 as usize];
                self.set_reg(rd, if b == 0 { u32::MAX } else { a / b });
            }
            Insn::Rem { rd, rs1, rs2 } => {
                let a = r[rs1 as usize] as i32;
                let b = r[rs2 as usize] as i32;
                let res = if b == 0 {
                    a
                } else if a == i32::MIN && b == -1 {
                    0
                } else {
                    a.wrapping_rem(b)
                };
                self.set_reg(rd, res as u32);
            }
            Insn::Remu { rd, rs1, rs2 } => {
                let a = r[rs1 as usize];
                let b = r[rs2 as usize];
                self.set_reg(rd, if b == 0 { a } else { a % b });
            }

            // Zba
            Insn::Sh1add { rd, rs1, rs2 } => {
                self.set_reg(rd, (r[rs1 as usize] << 1).wrapping_add(r[rs2 as usize]));
            }
            Insn::Sh2add { rd, rs1, rs2 } => {
                self.set_reg(rd, (r[rs1 as usize] << 2).wrapping_add(r[rs2 as usize]));
            }
            Insn::Sh3add { rd, rs1, rs2 } => {
                self.set_reg(rd, (r[rs1 as usize] << 3).wrapping_add(r[rs2 as usize]));
            }

            // Zbb
            Insn::Andn { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize] & !r[rs2 as usize]);
            }
            Insn::Orn { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize] | !r[rs2 as usize]);
            }
            Insn::Xnor { rd, rs1, rs2 } => {
                self.set_reg(rd, !(r[rs1 as usize] ^ r[rs2 as usize]));
            }
            Insn::Clz { rd, rs1 } => {
                self.set_reg(rd, r[rs1 as usize].leading_zeros());
            }
            Insn::Ctz { rd, rs1 } => {
                self.set_reg(rd, r[rs1 as usize].trailing_zeros());
            }
            Insn::Cpop { rd, rs1 } => {
                self.set_reg(rd, r[rs1 as usize].count_ones());
            }
            Insn::Max { rd, rs1, rs2 } => {
                let a = r[rs1 as usize] as i32;
                let b = r[rs2 as usize] as i32;
                self.set_reg(rd, a.max(b) as u32);
            }
            Insn::Maxu { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].max(r[rs2 as usize]));
            }
            Insn::Min { rd, rs1, rs2 } => {
                let a = r[rs1 as usize] as i32;
                let b = r[rs2 as usize] as i32;
                self.set_reg(rd, a.min(b) as u32);
            }
            Insn::Minu { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].min(r[rs2 as usize]));
            }
            Insn::SextB { rd, rs1 } => {
                self.set_reg(rd, r[rs1 as usize] as u8 as i8 as i32 as u32);
            }
            Insn::SextH { rd, rs1 } => {
                self.set_reg(rd, r[rs1 as usize] as u16 as i16 as i32 as u32);
            }
            Insn::ZextH { rd, rs1 } => {
                self.set_reg(rd, (r[rs1 as usize] & 0xFFFF) as u32);
            }
            Insn::Rol { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].rotate_left(r[rs2 as usize] & 31));
            }
            Insn::Ror { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize].rotate_right(r[rs2 as usize] & 31));
            }
            Insn::Rori { rd, rs1, shamt } => {
                self.set_reg(rd, r[rs1 as usize].rotate_right(shamt as u32));
            }
            Insn::OrcB { rd, rs1 } => {
                let x = r[rs1 as usize];
                let mut res = 0u32;
                for i in 0..4 {
                    if ((x >> (i * 8)) & 0xFF) != 0 {
                        res |= 0xFF << (i * 8);
                    }
                }
                self.set_reg(rd, res);
            }
            Insn::Rev8 { rd, rs1 } => {
                self.set_reg(rd, r[rs1 as usize].swap_bytes());
            }

            // Zbc
            Insn::Clmul { rd, rs1, rs2 } => {
                let a = r[rs1 as usize];
                let b = r[rs2 as usize];
                let mut out = 0u32;
                for i in 0..32 {
                    if ((b >> i) & 1) != 0 {
                        out ^= a << i;
                    }
                }
                self.set_reg(rd, out);
            }
            Insn::Clmulh { rd, rs1, rs2 } => {
                let a = r[rs1 as usize];
                let b = r[rs2 as usize];
                let mut out = 0u32;
                for i in 1..32 {
                    if ((b >> i) & 1) != 0 {
                        out ^= a >> (32 - i);
                    }
                }
                self.set_reg(rd, out);
            }
            Insn::Clmulr { rd, rs1, rs2 } => {
                let a = r[rs1 as usize];
                let b = r[rs2 as usize];
                let mut out = 0u32;
                for i in 0..32 {
                    if ((b >> i) & 1) != 0 {
                        out ^= a >> (31 - i);
                    }
                }
                self.set_reg(rd, out);
            }

            // Zbs
            Insn::Bclr { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize] & !(1u32 << (r[rs2 as usize] & 31)));
            }
            Insn::Bclri { rd, rs1, shamt } => {
                self.set_reg(rd, r[rs1 as usize] & !(1u32 << (shamt & 31)));
            }
            Insn::Bext { rd, rs1, rs2 } => {
                self.set_reg(rd, (r[rs1 as usize] >> (r[rs2 as usize] & 31)) & 1);
            }
            Insn::Bexti { rd, rs1, shamt } => {
                self.set_reg(rd, (r[rs1 as usize] >> (shamt & 31)) & 1);
            }
            Insn::Binv { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize] ^ (1u32 << (r[rs2 as usize] & 31)));
            }
            Insn::Binvi { rd, rs1, shamt } => {
                self.set_reg(rd, r[rs1 as usize] ^ (1u32 << (shamt & 31)));
            }
            Insn::Bset { rd, rs1, rs2 } => {
                self.set_reg(rd, r[rs1 as usize] | (1u32 << (r[rs2 as usize] & 31)));
            }
            Insn::Bseti { rd, rs1, shamt } => {
                self.set_reg(rd, r[rs1 as usize] | (1u32 << (shamt & 31)));
            }

            // WCH mcpy rd(end), rs1(start), rs2(dst)
            Insn::Mcpy { rd, rs1, rs2 } => {
                let end = r[rd as usize];
                let mut start = r[rs1 as usize];
                let mut dst = r[rs2 as usize];
                while start < end {
                    let b = bus.read_u8(start);
                    bus.write_u8(dst, b);
                    start = start.wrapping_add(1);
                    dst = dst.wrapping_add(1);
                }
                self.set_reg(rs1, start);
                self.set_reg(rs2, dst);
            }

            Insn::Illegal(raw) => {
                self.enter_exception(bus, 2, raw);
                *next_pc = self.pc;
            }
        }
    }
}

#[inline]
fn is_mmio_or_core_addr(a: u32) -> bool {
    a.wrapping_sub(0x4000_0000) < 0x1_0000 || a.wrapping_sub(0xE000_E000) < 0x2000
}

#[inline]
fn pre_insn_mem_extra(insn: &Insn, regs: &[u32; 32], timing: &TimingModel) -> u32 {
    match *insn {
        Insn::Lb { rs1, imm, .. }
        | Insn::Lh { rs1, imm, .. }
        | Insn::Lw { rs1, imm, .. }
        | Insn::Lbu { rs1, imm, .. }
        | Insn::Lhu { rs1, imm, .. } => {
            let a = regs[rs1 as usize].wrapping_add(imm as u32);
            let mmio = if is_mmio_or_core_addr(a) {
                timing.mmio_extra_cycles as u32
            } else {
                0
            };
            (timing.load_extra_cycles as u32) + mmio
        }
        Insn::Sb { rs1, imm, .. }
        | Insn::Sh { rs1, imm, .. }
        | Insn::Sw { rs1, imm, .. } => {
            let a = regs[rs1 as usize].wrapping_add(imm as u32);
            let mmio = if is_mmio_or_core_addr(a) {
                timing.mmio_extra_cycles as u32
            } else {
                0
            };
            (timing.store_extra_cycles as u32) + mmio
        }
        _ => 0,
    }
}

#[inline]
fn insn_timing_cycles(
    insn: &Insn,
    pc: u32,
    next_pc: u32,
    len: u32,
    mem_extra: u32,
    popped_irq: bool,
    timing: &TimingModel,
) -> u32 {
    let fetch_wait = if (pc as usize) < SysBus::FLASH_SIZE {
        timing.flash_fetch_wait as u32
    } else if (0x2000_0000..0x2002_0000).contains(&pc) {
        timing.sram_fetch_wait as u32
    } else {
        0
    };
    let flow_extra = match *insn {
        Insn::Beq { .. }
        | Insn::Bne { .. }
        | Insn::Blt { .. }
        | Insn::Bge { .. }
        | Insn::Bltu { .. }
        | Insn::Bgeu { .. } => {
            if next_pc != pc.wrapping_add(len) {
                timing.branch_taken_penalty as u32
            } else {
                0
            }
        }
        Insn::Jal { .. } | Insn::Jalr { .. } => timing.jump_penalty as u32,
        Insn::Mret if popped_irq => timing.irq_exit_cycles as u32,
        _ => 0,
    };
    1 + fetch_wait + flow_extra + mem_extra
}
