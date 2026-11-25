/// WCH QingKe V3C PFIC (0xE000_E000) and SysTick (0xE000_F000).
/// Offsets verified against `core_riscv.h`.

#[derive(Debug, Clone)]
pub struct Pfic {
    /// Interrupt Enable Status Register (256 IRQs -> 8x u32)
    pub isr: [u32; 8],
    /// Software/edge-latched Interrupt Pending Register (256 IRQs -> 8x u32)
    pub ipr: [u32; 8],
    /// Hardware level-driven pending lines (synced from peripherals)
    pub hw_ipr: [u32; 8],
    /// Interrupt Active Register (256 IRQs -> 8x u32)
    pub iactr: [u32; 8],
    /// Interrupt Priority Threshold (8-bit effective, bit7 = preempt priority)
    pub ithresdr: u32,
    /// VTF ID Register (4 channels)
    pub vtfidr: [u8; 4],
    /// VTF Address Register (bit 0 = enable)
    pub vtfaddr: [u32; 4],
    /// Interrupt Priority (256 x u8)
    pub iprior: [u8; 256],
    /// System Control Register (0xD10)
    pub sctlr: u32,
    /// Set by writing 0x8000_0000 to SCTLR
    pub reset_requested: bool,
    /// WFE event latch (set via SCTLR bit 5 `SEVONPEND`/event)
    pub event_pending: bool,
    /// Global status summary (cached or computed)
    pub active_stack: Vec<u8>,
    /// Bitmask of words `0..8` where `isr[w] & (ipr[w] | hw_ipr[w]) & !iactr[w] != 0`
    pub pending_words: u8,
}

impl Default for Pfic {
    fn default() -> Self {
        Self::new()
    }
}

impl Pfic {
    pub fn new() -> Self {
        let mut isr = [0u32; 8];
        // Reset/NMI/EXC are always considered enabled by default
        isr[0] = 0x0000_000E;
        Self {
            isr,
            ipr: [0; 8],
            hw_ipr: [0; 8],
            iactr: [0; 8],
            ithresdr: 0,
            vtfidr: [0; 4],
            vtfaddr: [0; 4],
            iprior: [0; 256],
            sctlr: 0,
            reset_requested: false,
            event_pending: false,
            active_stack: Vec::new(),
            pending_words: 0,
        }
    }

    #[inline]
    fn refresh_word(&mut self, idx: usize) {
        let cand = self.isr[idx] & (self.ipr[idx] | self.hw_ipr[idx]) & !self.iactr[idx];
        let mask = 1u8 << idx;
        if cand != 0 {
            self.pending_words |= mask;
        } else {
            self.pending_words &= !mask;
        }
    }

    pub fn set_pending(&mut self, irqn: u8) {
        let idx = (irqn as usize) >> 5;
        let bit = 1u32 << (irqn & 31);
        self.ipr[idx] |= bit;
        self.event_pending = true;
        self.refresh_word(idx);
    }

    /// Synchronize a level-sensitive peripheral interrupt line.
    #[inline]
    pub fn sync_level_irq(&mut self, irqn: u8, asserted: bool) {
        let idx = (irqn as usize) >> 5;
        let bit = 1u32 << (irqn & 31);
        let prev = (self.hw_ipr[idx] & bit) != 0;
        if prev == asserted {
            return;
        }
        if asserted {
            self.event_pending = true;
            self.hw_ipr[idx] |= bit;
        } else {
            self.hw_ipr[idx] &= !bit;
        }
        self.refresh_word(idx);
    }

    pub fn clear_pending(&mut self, irqn: u8) {
        let idx = (irqn as usize) >> 5;
        let bit = 1u32 << (irqn & 31);
        self.ipr[idx] &= !bit;
        self.refresh_word(idx);
    }

    pub fn is_enabled(&self, irqn: u8) -> bool {
        let idx = (irqn as usize) >> 5;
        let bit = 1u32 << (irqn & 31);
        (self.isr[idx] & bit) != 0
    }

    pub fn is_pending(&self, irqn: u8) -> bool {
        let idx = (irqn as usize) >> 5;
        let bit = 1u32 << (irqn & 31);
        ((self.ipr[idx] | self.hw_ipr[idx]) & bit) != 0
    }

    pub fn is_active(&self, irqn: u8) -> bool {
        let idx = (irqn as usize) >> 5;
        let bit = 1u32 << (irqn & 31);
        (self.iactr[idx] & bit) != 0
    }

    pub fn set_active(&mut self, irqn: u8, active: bool) {
        let idx = (irqn as usize) >> 5;
        let bit = 1u32 << (irqn & 31);
        if active {
            self.iactr[idx] |= bit;
        } else {
            self.iactr[idx] &= !bit;
        }
        self.refresh_word(idx);
    }

    /// Returns the highest-priority deliverable IRQ (if any) considering nesting/threshold.
    /// Returns `(irqn, optional_vtf_addr)`.
    #[inline]
    pub fn pick_pending_irq(&self, nesting_enabled: bool) -> Option<(u8, Option<u32>)> {
        if self.pending_words == 0 {
            return None;
        }
        let current_prio = self.active_stack.last().map(|&iq| self.iprior[iq as usize]);
        let thresh = (self.ithresdr & 0xFF) as u8;

        let mut best: Option<(u8, u8)> = None; // (irqn, prio)
        let mut words = self.pending_words;
        while words != 0 {
            let word_idx = words.trailing_zeros() as usize;
            words &= words - 1;
            let mut candidates =
                self.isr[word_idx] & (self.ipr[word_idx] | self.hw_ipr[word_idx]) & !self.iactr[word_idx];
            while candidates != 0 {
                let bit = candidates.trailing_zeros() as usize;
                candidates &= candidates - 1;
                let irqn = ((word_idx << 5) | bit) as u8;
                // Skip 0..11 unless NMI(2)
                if irqn < 12 && irqn != 2 {
                    continue;
                }
                let prio = self.iprior[irqn as usize];
                if thresh != 0 && prio >= thresh {
                    continue;
                }
                if let Some(cur_p) = current_prio {
                    if !nesting_enabled {
                        continue;
                    }
                    // Lower numeric priority = higher urgency; bit 7 is pre-emption bit (0 = high, 0x80 = low)
                    if prio >= cur_p {
                        continue;
                    }
                }
                match best {
                    None => best = Some((irqn, prio)),
                    Some((best_irq, best_p)) => {
                        if prio < best_p || (prio == best_p && irqn < best_irq) {
                            best = Some((irqn, prio));
                        }
                    }
                }
            }
        }

        let (irqn, _) = best?;
        let mut vtf_target = None;
        for ch in 0..4 {
            if (self.vtfaddr[ch] & 1) != 0 && self.vtfidr[ch] == irqn {
                vtf_target = Some(self.vtfaddr[ch] & !1);
                break;
            }
        }
        Some((irqn, vtf_target))
    }

    pub fn gisr(&self) -> u32 {
        let active_id = self.active_stack.last().copied().unwrap_or(0) as u32;
        let nest_sta = match self.active_stack.len() {
            0 => 0,
            1 => 1,
            2 => 3,
            _ => 7,
        };
        let mut gpend = 0u32;
        for i in 0..8 {
            if (self.isr[i] & (self.ipr[i] | self.hw_ipr[i])) != 0 {
                gpend = 1;
                break;
            }
        }
        let gact = u32::from(!self.active_stack.is_empty());
        active_id | (nest_sta << 8) | (gact << 16) | (gpend << 17)
    }

    pub fn read_u32(&self, offset: u32) -> u32 {
        match offset {
            0x00..=0x1C => self.isr[((offset - 0x00) >> 2) as usize],
            0x20..=0x3C => {
                let i = ((offset - 0x20) >> 2) as usize;
                self.ipr[i] | self.hw_ipr[i]
            }
            0x40 => self.ithresdr,
            0x4C => self.gisr(),
            0x50 => u32::from_le_bytes(self.vtfidr),
            0x60..=0x6C => self.vtfaddr[((offset - 0x60) >> 2) as usize],
            0x300..=0x31C => self.iactr[((offset - 0x300) >> 2) as usize],
            0x400..=0x4FC => {
                let idx = (offset - 0x400) as usize;
                u32::from_le_bytes([
                    self.iprior[idx],
                    self.iprior[idx + 1],
                    self.iprior[idx + 2],
                    self.iprior[idx + 3],
                ])
            }
            0xD10 => self.sctlr,
            _ => 0,
        }
    }

    pub fn write_u32(&mut self, offset: u32, val: u32) {
        match offset {
            0x40 => self.ithresdr = val & 0xFF,
            0x50 => self.vtfidr = val.to_le_bytes(),
            0x60..=0x6C => self.vtfaddr[((offset - 0x60) >> 2) as usize] = val,
            0x100..=0x11C => {
                let idx = ((offset - 0x100) >> 2) as usize;
                self.isr[idx] |= val;
                self.refresh_word(idx);
            }
            0x180..=0x19C => {
                let idx = ((offset - 0x180) >> 2) as usize;
                self.isr[idx] &= !val;
                self.refresh_word(idx);
            }
            0x200..=0x21C => {
                let idx = ((offset - 0x200) >> 2) as usize;
                self.ipr[idx] |= val;
                if val != 0 {
                    self.event_pending = true;
                }
                self.refresh_word(idx);
            }
            0x280..=0x29C => {
                let idx = ((offset - 0x280) >> 2) as usize;
                self.ipr[idx] &= !val;
                self.refresh_word(idx);
            }
            0x300..=0x31C => {
                let idx = ((offset - 0x300) >> 2) as usize;
                self.iactr[idx] = val;
                self.refresh_word(idx);
            }
            0x400..=0x4FC => {
                let idx = (offset - 0x400) as usize;
                let b = val.to_le_bytes();
                self.iprior[idx..idx + 4].copy_from_slice(&b);
            }
            0xD10 => {
                // KEY3 = 0xBEEF0000 or direct bit31 per core_riscv.h PFIC_SystemReset
                if (val & 0x8000_0000) != 0 {
                    self.reset_requested = true;
                }
                if (val & (1 << 5)) != 0 {
                    self.event_pending = true;
                }
                self.sctlr = val & 0x7FFF_FFFF;
            }
            _ => {}
        }
    }

    pub fn read_u8(&self, offset: u32) -> u8 {
        if (0x400..0x500).contains(&offset) {
            return self.iprior[(offset - 0x400) as usize];
        }
        if (0x50..0x54).contains(&offset) {
            return self.vtfidr[(offset - 0x50) as usize];
        }
        let word = self.read_u32(offset & !3);
        ((word >> ((offset & 3) * 8)) & 0xFF) as u8
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        if (0x400..0x500).contains(&offset) {
            self.iprior[(offset - 0x400) as usize] = val;
            return;
        }
        if (0x50..0x54).contains(&offset) {
            self.vtfidr[(offset - 0x50) as usize] = val;
            return;
        }
        let aligned = offset & !3;
        let shift = (offset & 3) * 8;
        let mut word = self.read_u32(aligned);
        word &= !(0xFF << shift);
        word |= (val as u32) << shift;
        self.write_u32(aligned, word);
    }
}

/// QingKe V3C SysTick (`0xE000_F000`).
/// Supports 32-bit/64-bit counting, up/down mode, auto-reload, HCLK or HCLK/8 clock.
#[derive(Debug, Clone)]
pub struct SysTick {
    pub ctlr: u32,
    pub sr: u32,
    pub cnt: u64,
    pub cmp: u64,
    prescale_rem: u32,
}

impl Default for SysTick {
    fn default() -> Self {
        Self::new()
    }
}

impl SysTick {
    pub const IRQN: u8 = 12;

    pub fn new() -> Self {
        Self {
            ctlr: 0,
            sr: 0,
            cnt: 0,
            cmp: 0xFFFF_FFFF_FFFF_FFFF,
            prescale_rem: 0,
        }
    }

    pub fn next_irq_cycles(&self) -> Option<u64> {
        if (self.ctlr & 1) == 0 || (self.ctlr & (1 << 1)) == 0 {
            return None;
        }
        if (self.sr & 1) != 0 {
            return Some(0);
        }
        let down_mode = (self.ctlr & (1 << 4)) != 0;
        let ticks_needed = if !down_mode {
            if self.cnt < self.cmp {
                self.cmp - self.cnt
            } else {
                return None;
            }
        } else {
            self.cnt.max(1)
        };
        if (self.ctlr & (1 << 2)) != 0 {
            Some(ticks_needed)
        } else {
            let cyc = ticks_needed
                .saturating_mul(8)
                .saturating_sub(self.prescale_rem as u64);
            Some(cyc.max(1))
        }
    }

    pub fn step(&mut self, hclk_cycles: u32, pfic: &mut Pfic) {
        self.advance_cycles(hclk_cycles as u64, pfic);
    }

    pub fn advance_cycles(&mut self, hclk_cycles: u64, pfic: &mut Pfic) {
        if (self.ctlr & 1) == 0 {
            return; // STE=0
        }
        let ticks = if (self.ctlr & (1 << 2)) != 0 {
            // STCLK = 1 -> HCLK
            hclk_cycles
        } else {
            // STCLK = 0 -> HCLK / 8
            let total = (self.prescale_rem as u64).saturating_add(hclk_cycles);
            self.prescale_rem = (total & 7) as u32;
            total >> 3
        };
        if ticks == 0 {
            return;
        }

        let down_mode = (self.ctlr & (1 << 4)) != 0;
        let auto_reload = (self.ctlr & (1 << 3)) != 0;
        let irq_en = (self.ctlr & (1 << 1)) != 0;

        if !down_mode {
            // Up-counting: compare against cmp (32-bit if only CMPL used; WCH writes CMPH=0 usually)
            let prev = self.cnt;
            let next = prev.wrapping_add(ticks);
            if prev < self.cmp && (ticks >= self.cmp - prev) {
                self.sr |= 1; // CNTIF
                if irq_en {
                    pfic.set_pending(Self::IRQN);
                }
                if auto_reload {
                    let period = self.cmp.saturating_add(1);
                    self.cnt = if period > 0 {
                        (ticks - (self.cmp - prev)) % period
                    } else {
                        0
                    };
                    return;
                }
            }
            self.cnt = next;
        } else {
            // Down-counting from cmp to 0
            if self.cnt <= ticks {
                self.sr |= 1;
                if irq_en {
                    pfic.set_pending(Self::IRQN);
                }
                if auto_reload {
                    let period = self.cmp.saturating_add(1);
                    let over = ticks - self.cnt;
                    self.cnt = if over == 0 || period == 0 {
                        self.cmp
                    } else {
                        self.cmp.saturating_sub((over - 1) % period)
                    };
                } else {
                    self.cnt = 0;
                }
            } else {
                self.cnt -= ticks;
            }
        }
    }

    pub fn read_u32(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.ctlr,
            0x04 => self.sr,
            0x08 => self.cnt as u32,
            0x0C => (self.cnt >> 32) as u32,
            0x10 => self.cmp as u32,
            0x14 => (self.cmp >> 32) as u32,
            _ => 0,
        }
    }

    pub fn write_u32(&mut self, offset: u32, val: u32, pfic: &mut Pfic) {
        match offset {
            0x00 => {
                // bit 5 (INIT): update counter to 0 (up) or CMP (down)
                if (val & (1 << 5)) != 0 {
                    self.cnt = if (val & (1 << 4)) != 0 { self.cmp } else { 0 };
                }
                if (val & (1 << 31)) != 0 {
                    pfic.set_pending(Self::IRQN);
                }
                self.ctlr = val & !(1 << 5);
            }
            0x04 => {
                // Writing 0 clears CNTIF; SWIE bit 31 triggers SWI or clears?
                if (val & 1) == 0 {
                    self.sr &= !1;
                }
                if (val & (1 << 31)) != 0 {
                    self.sr |= 1 << 31;
                    pfic.set_pending(14); // SWI_IRQn
                } else {
                    self.sr &= !(1 << 31);
                }
            }
            0x08 => self.cnt = (self.cnt & 0xFFFF_FFFF_0000_0000) | (val as u64),
            0x0C => self.cnt = (self.cnt & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32),
            0x10 => self.cmp = val as u64, // Writing CMPL resets upper 32 bits for 32-bit compatibility
            0x14 => self.cmp = (self.cmp & 0x0000_0000_FFFF_FFFF) | ((val as u64) << 32),
            _ => {}
        }
    }
}
