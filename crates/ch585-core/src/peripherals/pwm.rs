use crate::peripherals::irq;
use crate::pfic::Pfic;

/// PWM4..PWM11 peripheral (`0x4000_5000..0x4000_5017`).
#[derive(Debug, Clone)]
pub struct PwmX {
    pub out_en: u8,    // 0x00
    pub polar: u8,     // 0x01
    pub config: u8,    // 0x02
    pub clock_div: u8, // 0x03
    pub data_8: [u8; 8],     // 0x04..0x0B (also PWM4..7 16-bit at 0x04..0x0B)
    pub int_ctrl: u8,        // 0x0C
    pub data8_9_16: [u16; 2],// 0x10..0x13 (PWM8..9 16-bit)
    pub reg_cycle: u32,      // 0x14 (16-bit cycle)

    counter: u32,
    div_acc: u32,
    pub outputs: [bool; 8],
}

impl Default for PwmX {
    fn default() -> Self {
        Self::new()
    }
}

impl PwmX {
    pub fn new() -> Self {
        Self {
            out_en: 0,
            polar: 0,
            config: 0,
            clock_div: 1,
            data_8: [0; 8],
            int_ctrl: 0,
            data8_9_16: [0; 2],
            reg_cycle: 256,
            counter: 0,
            div_acc: 0,
            outputs: [false; 8],
        }
    }

    pub fn period(&self) -> u32 {
        let mode = (self.config >> 2) & 0x03;
        let sub1 = (self.config & 0x01) as u32;
        match mode {
            0 => 256 - sub1,
            1 => 128 - sub1,
            2 => 64 - sub1,
            _ => (self.reg_cycle & 0xFFFF).max(1),
        }
    }

    pub fn channel_duty(&self, ch_idx: usize) -> u32 {
        let mode = (self.config >> 2) & 0x03;
        if mode == 3 {
            match ch_idx {
                0..=3 => u16::from_le_bytes([self.data_8[ch_idx * 2], self.data_8[ch_idx * 2 + 1]]) as u32,
                4 => self.data8_9_16[0] as u32,
                5 => self.data8_9_16[1] as u32,
                _ => 0,
            }
        } else {
            self.data_8[ch_idx] as u32
        }
    }

    #[inline]
    pub fn next_edge_cycles(&self) -> Option<u32> {
        if self.out_en == 0 && (self.int_ctrl & 0x01) == 0 {
            return None;
        }
        let div = (self.clock_div as u32).max(1);
        let first_tick_cyc = div.saturating_sub(self.div_acc).max(1);
        let per = self.period();
        let mut min_ticks = per.saturating_sub(self.counter).max(1);

        for ch in 0..8 {
            if (self.out_en & (1 << ch)) == 0 {
                if self.outputs[ch] {
                    return Some(first_tick_cyc);
                }
                continue;
            }
            let duty = self.channel_duty(ch);
            let polar = ((self.polar >> ch) & 1) != 0;
            let expected_now = (self.counter < duty) ^ polar;
            if self.outputs[ch] != expected_now {
                return Some(first_tick_cyc);
            }
            if self.counter < duty && duty < per {
                min_ticks = min_ticks.min(duty - self.counter);
            }
        }

        Some(first_tick_cyc.saturating_add(min_ticks.saturating_sub(1).saturating_mul(div)))
    }

    pub fn step(&mut self, cycles: u32, pfic: &mut Pfic) {
        if self.out_en == 0 && (self.int_ctrl & 0x01) == 0 {
            pfic.sync_level_irq(irq::PWMX, (self.int_ctrl & 0x81) == 0x81);
            return;
        }
        let div = (self.clock_div as u32).max(1);
        self.div_acc += cycles;
        let ticks = self.div_acc / div;
        self.div_acc %= div;
        if ticks == 0 {
            pfic.sync_level_irq(irq::PWMX, (self.int_ctrl & 0x81) == 0x81);
            return;
        }

        let per = self.period();
        let next = self.counter + ticks;
        if next >= per {
            self.counter = next % per;
            self.int_ctrl |= 0x80; // RB_PWM_IF_CYC
        } else {
            self.counter = next;
        }
        pfic.sync_level_irq(irq::PWMX, (self.int_ctrl & 0x81) == 0x81);

        for ch in 0..8 {
            if (self.out_en & (1 << ch)) == 0 {
                self.outputs[ch] = false;
                continue;
            }
            let duty = self.channel_duty(ch);
            let polar = ((self.polar >> ch) & 1) != 0;
            let active = self.counter < duty;
            self.outputs[ch] = active ^ polar;
        }
    }

    pub fn read_u8(&self, offset: u32) -> u8 {
        match offset {
            0x00 => self.out_en,
            0x01 => self.polar,
            0x02 => self.config,
            0x03 => self.clock_div,
            0x04..=0x0B => self.data_8[(offset - 0x04) as usize],
            0x0C => self.int_ctrl,
            0x10..=0x11 => (self.data8_9_16[0] >> ((offset - 0x10) * 8)) as u8,
            0x12..=0x13 => (self.data8_9_16[1] >> ((offset - 0x12) * 8)) as u8,
            0x14..=0x17 => (self.reg_cycle >> ((offset - 0x14) * 8)) as u8,
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        match offset {
            0x00 => self.out_en = val,
            0x01 => self.polar = val,
            0x02 => self.config = val,
            0x03 => self.clock_div = val,
            0x04..=0x0B => self.data_8[(offset - 0x04) as usize] = val,
            0x0C => {
                // Bit 7 is RW1C
                let old_if = self.int_ctrl & 0x80;
                self.int_ctrl = (val & 0x7F) | if (val & 0x80) != 0 { 0 } else { old_if };
            }
            0x10 => self.data8_9_16[0] = (self.data8_9_16[0] & 0xFF00) | (val as u16),
            0x11 => self.data8_9_16[0] = (self.data8_9_16[0] & 0x00FF) | ((val as u16) << 8),
            0x12 => self.data8_9_16[1] = (self.data8_9_16[1] & 0xFF00) | (val as u16),
            0x13 => self.data8_9_16[1] = (self.data8_9_16[1] & 0x00FF) | ((val as u16) << 8),
            0x14..=0x17 => {
                let s = (offset - 0x14) * 8;
                self.reg_cycle = (self.reg_cycle & !(0xFF << s)) | ((val as u32) << s);
            }
            _ => {}
        }
    }

    pub fn read_u32(&self, offset: u32) -> u32 {
        let b0 = self.read_u8(offset) as u32;
        let b1 = self.read_u8(offset + 1) as u32;
        let b2 = self.read_u8(offset + 2) as u32;
        let b3 = self.read_u8(offset + 3) as u32;
        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    }

    pub fn write_u32(&mut self, offset: u32, val: u32) {
        let b = val.to_le_bytes();
        for i in 0..4 {
            self.write_u8(offset + i as u32, b[i]);
        }
    }
}
