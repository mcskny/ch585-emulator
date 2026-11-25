use crate::peripherals::irq;
use crate::pfic::Pfic;

const DAY_TICKS: u64 = 43_200 * 65_536;

/// Real-Time Clock (`0x4000_1030..0x4000_103F`), driven by 32768 Hz / 32000 Hz clock.
#[derive(Debug, Clone)]
pub struct Rtc {
    pub flag_ctrl: u8, // 0x30: bit6=TMR_FLAG, bit7=TRIG_FLAG
    pub mode_ctrl: u8, // 0x31
    pub trig: u32,     // 0x34
    pub cnt_32k: u16,  // 0x38 (0..0xFFFF -> 2 seconds)
    pub cnt_2s: u16,   // 0x3A (0..43199 -> 1 day)
    pub cnt_day: u16,  // 0x3C (14-bit)

    cycle_acc: u64,
    ns_rem: u64,
    tmr_ticks: u32,
}

impl Default for Rtc {
    fn default() -> Self {
        Self::new()
    }
}

impl Rtc {
    pub fn new() -> Self {
        Self {
            flag_ctrl: 0,
            mode_ctrl: 0,
            trig: 0,
            cnt_32k: 0,
            cnt_2s: 0,
            cnt_day: 0,
            cycle_acc: 0,
            ns_rem: 0,
            tmr_ticks: 0,
        }
    }

    #[inline]
    pub fn cnt_lo_32(&self) -> u32 {
        (self.cnt_32k as u32) | ((self.cnt_2s as u32) << 16)
    }

    #[inline]
    pub fn irq_asserted(&self) -> bool {
        ((self.flag_ctrl & 0x40) != 0 && (self.mode_ctrl & 0x10) != 0)
            || ((self.flag_ctrl & 0x80) != 0 && (self.mode_ctrl & 0x20) != 0)
    }

    fn ticks_until_trig(&self) -> Option<u64> {
        if (self.mode_ctrl & 0x20) == 0 {
            return None;
        }
        let trig_2s = (self.trig >> 16) as u64;
        if trig_2s >= 43_200 {
            return None;
        }
        let cur = (self.cnt_2s as u64) * 65_536 + (self.cnt_32k as u64);
        let trig_lo = (self.trig & 0xFFFF) as u64;
        let target_base = trig_2s * 65_536 + trig_lo;
        let ignore_bit0 = (self.mode_ctrl & 0x08) != 0;
        let dist = |target: u64| -> u64 {
            if target > cur {
                target - cur
            } else {
                DAY_TICKS - cur + target
            }
        };
        if ignore_bit0 {
            let t0 = target_base & !1;
            let t1 = t0 + 1;
            Some(dist(t0).min(dist(t1)).max(1))
        } else {
            Some(dist(target_base).max(1))
        }
    }

    /// Number of 32 kHz ticks until the next enabled RTC wakeup event (0 if already asserted).
    pub fn next_wakeup_ticks(&self) -> Option<u64> {
        if self.irq_asserted() {
            return Some(0);
        }
        let tmr = if (self.mode_ctrl & 0x10) != 0 {
            let period = 4096u64 << (self.mode_ctrl & 0x07);
            Some(period.saturating_sub(self.tmr_ticks as u64).max(1))
        } else {
            None
        };
        let trig = self.ticks_until_trig();
        match (tmr, trig) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        }
    }

    /// Nanoseconds until next RTC wakeup event at `clk_32k_hz` (rounded up so fast-forward lands on tick edge).
    pub fn next_wakeup_ns(&self, clk_32k_hz: u32) -> Option<u64> {
        let ticks = self.next_wakeup_ticks()?;
        if ticks == 0 {
            return Some(0);
        }
        let hz = clk_32k_hz.max(1) as u128;
        let needed_scaled = (ticks as u128) * 1_000_000_000u128;
        let rem = needed_scaled.saturating_sub(self.ns_rem as u128);
        Some(rem.div_ceil(hz).max(1) as u64)
    }

    /// Advance RTC by `ticks` 32 kHz clock ticks in O(1).
    pub fn advance_ticks(&mut self, ticks: u64, pfic: &mut Pfic) -> bool {
        let mut woke = false;
        if ticks > 0 {
            if let Some(dist) = self.ticks_until_trig() {
                if ticks >= dist {
                    self.flag_ctrl |= 0x80; // RB_RTC_TRIG_FLAG
                    woke = true;
                }
            }
            let cur = (self.cnt_2s as u64) * 65_536 + (self.cnt_32k as u64);
            let next_total = cur + ticks;
            let days = next_total / DAY_TICKS;
            let rem = next_total % DAY_TICKS;
            if days > 0 {
                self.cnt_day = ((self.cnt_day as u64 + days) & 0x3FFF) as u16;
            }
            self.cnt_2s = (rem / 65_536) as u16;
            self.cnt_32k = (rem % 65_536) as u16;

            if (self.mode_ctrl & 0x10) != 0 {
                let period = 4096u64 << (self.mode_ctrl & 0x07);
                let sum = (self.tmr_ticks as u64) + ticks;
                if sum >= period {
                    self.flag_ctrl |= 0x40; // RB_RTC_TMR_FLAG
                    woke = true;
                }
                self.tmr_ticks = (sum % period) as u32;
            } else {
                self.tmr_ticks = 0;
            }
        }
        pfic.sync_level_irq(irq::RTC, self.irq_asserted());
        woke
    }

    /// Advance RTC by `dt_ns` nanoseconds in O(1), preserving sub-tick remainder.
    pub fn advance_ns(&mut self, dt_ns: u64, clk_32k_hz: u32, pfic: &mut Pfic) -> bool {
        let total = (self.ns_rem as u128) + (dt_ns as u128) * (clk_32k_hz as u128);
        let ticks = (total / 1_000_000_000u128) as u64;
        self.ns_rem = (total % 1_000_000_000u128) as u64;
        self.advance_ticks(ticks, pfic)
    }

    /// Advance RTC by `sys_cycles` at `sys_clk_hz`. Returns `true` if an RTC wakeup event occurred.
    pub fn step(&mut self, sys_cycles: u32, sys_clk_hz: u32, clk_32k_hz: u32, pfic: &mut Pfic) -> bool {
        let fsys = sys_clk_hz.max(32_000) as u64;
        self.cycle_acc += (sys_cycles as u64) * (clk_32k_hz as u64);
        let ticks_32k = self.cycle_acc / fsys;
        self.cycle_acc %= fsys;
        self.advance_ticks(ticks_32k, pfic)
    }

    pub fn read_u8(&self, offset: u32) -> u8 {
        match offset {
            0x30 => self.flag_ctrl,
            0x31 => self.mode_ctrl,
            0x34..=0x37 => (self.trig >> ((offset - 0x34) * 8)) as u8,
            0x38 => self.cnt_32k as u8,
            0x39 => (self.cnt_32k >> 8) as u8,
            0x3A => self.cnt_2s as u8,
            0x3B => (self.cnt_2s >> 8) as u8,
            0x3C => self.cnt_day as u8,
            0x3D => ((self.cnt_day >> 8) & 0x3F) as u8,
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8, safe_unlocked: bool) {
        match offset {
            0x30 => {
                if (val & 0x10) != 0 {
                    self.flag_ctrl &= !0x40;
                }
                if (val & 0x20) != 0 {
                    self.flag_ctrl &= !0x80;
                }
            }
            0x31 => {
                if safe_unlocked {
                    let mut next = val & 0x3F;
                    if (val & 0x40) != 0 {
                        // RB_RTC_LOAD_LO
                        self.cnt_32k = (self.trig & 0xFFFF) as u16;
                        self.cnt_2s = ((self.trig >> 16) & 0xFFFF) as u16;
                    }
                    if (val & 0x80) != 0 {
                        // RB_RTC_LOAD_HI
                        self.cnt_day = (self.trig & 0x3FFF) as u16;
                    }
                    if (next & 0x10) == 0 {
                        self.tmr_ticks = 0;
                    }
                    // Auto clear LOAD bits
                    next &= 0x3F;
                    self.mode_ctrl = next;
                }
            }
            0x34..=0x37 => {
                if safe_unlocked {
                    let s = (offset - 0x34) * 8;
                    self.trig = (self.trig & !(0xFF << s)) | ((val as u32) << s);
                }
            }
            _ => {}
        }
    }
}
