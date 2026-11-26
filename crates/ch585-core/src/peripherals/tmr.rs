use std::collections::VecDeque;

use crate::pfic::Pfic;

/// 26-bit General Purpose Timer / PWM / Capture (`TMR0..TMR3`).
#[derive(Debug, Clone)]
pub struct Timer {
    pub idx: u8,
    pub irqn: u8,
    pub ctrl_mod: u8,
    pub ctrl_dma: u8,
    pub inter_en: u8,
    pub int_flag: u8,
    pub count: u32,
    pub cnt_end: u32,
    pub fifo: VecDeque<u32>,
    pub dma_now: u32,
    pub dma_beg: u32,
    pub dma_end: u32,
    /// Current PWM output state
    pub pwm_out: bool,
    /// Fires true for 1 cycle on period match (can trigger ADC)
    pub cycle_pulse: bool,
    pub dma_active_step: bool,
    pwm_rep: u8,
    irq_dirty: bool,
}

impl Timer {
    pub fn new(idx: u8, irqn: u8) -> Self {
        Self {
            idx,
            irqn,
            ctrl_mod: 0,
            ctrl_dma: 0,
            inter_en: 0,
            int_flag: 0,
            count: 0,
            cnt_end: 0x03FF_FFFF,
            fifo: VecDeque::with_capacity(8),
            dma_now: 0,
            dma_beg: 0,
            dma_end: 0,
            pwm_out: false,
            cycle_pulse: false,
            dma_active_step: false,
            pwm_rep: 0,
            irq_dirty: true,
        }
    }

    fn pwm_repeat_target(&self) -> u8 {
        match (self.ctrl_mod >> 6) & 3 {
            0 => 1,
            1 => 4,
            2 => 8,
            _ => 16,
        }
    }

    fn refill_dma_fifo(&mut self, sram: &[u8]) {
        if (self.ctrl_dma & 0x01) == 0 || self.dma_end <= self.dma_beg {
            return;
        }
        while self.fifo.len() < 8 && self.dma_now < self.dma_end {
            self.dma_active_step = true;
            let addr = (self.dma_now & 0x1FFFF) as usize;
            if addr + 4 <= sram.len() {
                let word = u32::from_le_bytes([
                    sram[addr],
                    sram[addr + 1],
                    sram[addr + 2],
                    sram[addr + 3],
                ]);
                self.fifo.push_back(word & 0x03FF_FFFF);
            }
            self.dma_now += 4;
            if self.dma_now >= self.dma_end {
                self.int_flag |= 0x08; // RB_TMR_IF_DMA_END
                if (self.ctrl_dma & 0x04) != 0 {
                    self.dma_now = self.dma_beg;
                } else {
                    self.ctrl_dma &= !0x01;
                    break;
                }
            }
        }
    }

    pub fn is_pwm_or_dma_active(&self) -> bool {
        (self.ctrl_mod & 0x04) != 0 && ((self.ctrl_mod & 0x08) != 0 || (self.ctrl_dma & 0x01) != 0)
    }

    pub fn next_edge_cycles(&self) -> Option<u32> {
        if !self.is_pwm_or_dma_active() || (self.ctrl_mod & 0x01) != 0 {
            return None;
        }
        let end = (self.cnt_end & 0x03FF_FFFF).max(1);
        let dma_pwm = (self.ctrl_mod & 0x08) != 0 && self.dma_end > self.dma_beg;
        if (self.ctrl_dma & 0x01) != 0 && self.fifo.is_empty() {
            return Some(1);
        }
        if (self.ctrl_mod & 0x08) != 0 {
            let polar = (self.ctrl_mod & 0x10) != 0;
            let duty = if dma_pwm && self.fifo.is_empty() {
                if polar { end } else { 0 }
            } else {
                self.fifo.front().copied().unwrap_or(end / 2)
            };
            let expected_now = (self.count < duty) ^ polar;
            if self.pwm_out != expected_now {
                return Some(1);
            }
            if dma_pwm && self.fifo.is_empty() && (self.ctrl_dma & 0x01) == 0 {
                return if self.inter_en == 0 {
                    None
                } else {
                    Some(end.saturating_sub(self.count).max(1))
                };
            }
            if self.count < duty && duty < end {
                Some((duty - self.count).max(1))
            } else {
                Some(end.saturating_sub(self.count).max(1))
            }
        } else {
            Some(end.saturating_sub(self.count).max(1))
        }
    }

    pub fn next_irq_cycles(&self) -> Option<u64> {
        if (self.ctrl_mod & 0x04) == 0 || (self.ctrl_mod & 0x01) != 0 || self.inter_en == 0 {
            return None;
        }
        let end = (self.cnt_end & 0x03FF_FFFF).max(1);
        Some(end.saturating_sub(self.count).max(1) as u64)
    }

    pub fn advance_cycles(&mut self, cycles: u64, pfic: &mut Pfic) {
        self.cycle_pulse = false;
        if (self.ctrl_mod & 0x04) == 0 || (self.ctrl_mod & 0x01) != 0 {
            if self.irq_dirty {
                pfic.sync_level_irq(self.irqn, (self.int_flag & self.inter_en) != 0);
                self.irq_dirty = false;
            }
            return;
        }
        let end = (self.cnt_end & 0x03FF_FFFF).max(1) as u64;
        let next = (self.count as u64).saturating_add(cycles);
        if next >= end {
            self.count = (next % end) as u32;
            self.int_flag |= 0x01; // RB_TMR_IF_CYC_END
            self.cycle_pulse = true;
        } else {
            self.count = next as u32;
        }
        pfic.sync_level_irq(self.irqn, (self.int_flag & self.inter_en) != 0);
        self.irq_dirty = false;
    }

    pub fn step(&mut self, cycles: u32, pfic: &mut Pfic, sram: &mut [u8]) {
        self.cycle_pulse = false;
        self.dma_active_step = false;
        if (self.ctrl_mod & 0x04) == 0 {
            if self.irq_dirty {
                pfic.sync_level_irq(self.irqn, (self.int_flag & self.inter_en) != 0);
                self.irq_dirty = false;
            }
            return; // RB_TMR_COUNT_EN = 0
        }
        let mode_in = (self.ctrl_mod & 0x01) != 0;
        if mode_in {
            if self.irq_dirty {
                pfic.sync_level_irq(self.irqn, (self.int_flag & self.inter_en) != 0);
                self.irq_dirty = false;
            }
            // Capture/count mode increments on external edges
            return;
        }

        let end = (self.cnt_end & 0x03FF_FFFF).max(1);
        let dma_pwm = (self.ctrl_mod & 0x08) != 0 && self.dma_end > self.dma_beg;
        if (self.ctrl_dma & 0x01) != 0 && self.fifo.is_empty() {
            self.refill_dma_fifo(sram);
        }

        let next = self.count + cycles;
        if next >= end {
            self.count = next % end;
            self.int_flag |= 0x01; // RB_TMR_IF_CYC_END
            self.cycle_pulse = true;

            if dma_pwm || (self.ctrl_dma & 0x01) != 0 {
                self.pwm_rep += 1;
                if self.pwm_rep >= self.pwm_repeat_target() {
                    self.pwm_rep = 0;
                    if !self.fifo.is_empty() {
                        self.fifo.pop_front();
                    }
                    self.refill_dma_fifo(sram);
                }
            }
        } else {
            self.count = next;
        }

        // Update PWM output if RB_TMR_OUT_EN (bit 3) is set
        if (self.ctrl_mod & 0x08) != 0 {
            let polar = (self.ctrl_mod & 0x10) != 0;
            let duty = if dma_pwm && self.fifo.is_empty() {
                // Idle between DMA bursts: stay inactive
                if polar { end } else { 0 }
            } else {
                self.fifo.front().copied().unwrap_or(end / 2)
            };
            let active = self.count < duty;
            self.pwm_out = active ^ polar;
        }

        pfic.sync_level_irq(self.irqn, (self.int_flag & self.inter_en) != 0);
        self.irq_dirty = false;
    }

    pub fn read_u8(&self, offset: u32) -> u8 {
        match offset {
            0x00 => self.ctrl_mod,
            0x01 => self.ctrl_dma,
            0x02 => self.inter_en,
            0x06 => self.int_flag,
            0x07 => self.fifo.len() as u8,
            0x08..=0x0B => (self.count >> ((offset - 0x08) * 8)) as u8,
            0x0C..=0x0F => (self.cnt_end >> ((offset - 0x0C) * 8)) as u8,
            0x10..=0x13 => {
                let v = self.fifo.front().copied().unwrap_or(0);
                (v >> ((offset - 0x10) * 8)) as u8
            }
            0x14..=0x17 => (self.dma_now >> ((offset - 0x14) * 8)) as u8,
            0x18..=0x1B => (self.dma_beg >> ((offset - 0x18) * 8)) as u8,
            0x1C..=0x1F => (self.dma_end >> ((offset - 0x1C) * 8)) as u8,
            _ => 0,
        }
    }

    pub fn read_u32(&self, offset: u32) -> u32 {
        let mut copy = self.clone();
        copy.read_u32_mut(offset)
    }

    pub fn read_u32_mut(&mut self, offset: u32) -> u32 {
        match offset {
            0x00 => u32::from_le_bytes([self.ctrl_mod, self.ctrl_dma, self.inter_en, 0]),
            0x04 => u32::from_le_bytes([0, 0, self.int_flag, self.fifo.len() as u8]),
            0x08 => self.count,
            0x0C => self.cnt_end,
            0x10 => self.fifo.pop_front().unwrap_or(0),
            0x14 => self.dma_now,
            0x18 => self.dma_beg,
            0x1C => self.dma_end,
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        self.irq_dirty = true;
        match offset {
            0x00 => {
                self.ctrl_mod = val & !0x02;
                if (val & 0x02) != 0 {
                    // RB_TMR_ALL_CLEAR
                    self.count = 0;
                    self.fifo.clear();
                    self.int_flag = 0;
                    self.pwm_rep = 0;
                }
            }
            0x01 => self.ctrl_dma = val,
            0x02 => self.inter_en = val,
            0x06 => self.int_flag &= !val, // RW1C
            0x0C..=0x0F => {
                let s = (offset - 0x0C) * 8;
                self.cnt_end = (self.cnt_end & !(0xFF << s)) | ((val as u32) << s);
                self.cnt_end &= 0x03FF_FFFF;
            }
            0x10 => {
                if self.fifo.len() < 8 {
                    self.fifo.push_back(val as u32);
                }
            }
            0x14..=0x17 => {
                let s = (offset - 0x14) * 8;
                self.dma_now = (self.dma_now & !(0xFF << s)) | ((val as u32) << s);
            }
            0x18..=0x1B => {
                let s = (offset - 0x18) * 8;
                self.dma_beg = (self.dma_beg & !(0xFF << s)) | ((val as u32) << s);
                self.dma_now = self.dma_beg;
            }
            0x1C..=0x1F => {
                let s = (offset - 0x1C) * 8;
                self.dma_end = (self.dma_end & !(0xFF << s)) | ((val as u32) << s);
            }
            _ => {}
        }
    }

    pub fn write_u32(&mut self, offset: u32, val: u32) {
        self.irq_dirty = true;
        match offset {
            0x00 => {
                let b = val.to_le_bytes();
                self.write_u8(0x00, b[0]);
                self.write_u8(0x01, b[1]);
                self.write_u8(0x02, b[2]);
            }
            0x04 => {
                let b = val.to_le_bytes();
                self.write_u8(0x06, b[2]);
            }
            0x0C => self.cnt_end = val & 0x03FF_FFFF,
            0x10 => {
                if self.fifo.len() < 8 {
                    self.fifo.push_back(val & 0x03FF_FFFF);
                }
            }
            0x14 => self.dma_now = val,
            0x18 => {
                self.dma_beg = val;
                self.dma_now = val;
            }
            0x1C => self.dma_end = val,
            _ => {}
        }
    }
}
