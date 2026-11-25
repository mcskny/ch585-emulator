use crate::peripherals::irq;
use crate::pfic::Pfic;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtPull {
    None,
    PullUp,
    PullDown,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtPinDrive {
    Floating,
    Low,
    High,
    /// Analog voltage in mV (e.g. from Hall sensor / potentiometer)
    AnalogMv(u16),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinElectricState {
    Low,
    High,
    HiZ,
    Analog,
}

/// CH585M GPIO (PA0..15, PB0..23), Alternate Pin Remap, Digital Input Disable, and GPIO Interrupts.
#[derive(Debug, Clone)]
pub struct Gpio {
    // 0x4000_1018..0x4000_101F
    pub pin_alternate: u16, // R16_PIN_ALTERNATE
    pub pin_config: u16,    // R16_PIN_CONFIG
    pub pin_in_dis: u32,    // R32_PIN_IN_DIS

    // 0x4000_1090..0x4000_109F
    pub pa_int_en: u16,
    pub pb_int_en: u16,
    pub pa_int_mode: u16, // 0=level, 1=edge
    pub pb_int_mode: u16,
    pub pa_int_if: u16,
    pub pb_int_if: u16,

    // PA 0x4000_10A0..0x4000_10BB
    pub pa_dir: u16,
    pub pa_out: u16,
    pub pa_pu: u16,
    pub pa_pd_drv: u16,

    // PB 0x4000_10C0..0x4000_10DB
    pub pb_dir: u32,
    pub pb_out: u32,
    pub pb_pu: u32,
    pub pb_pd_drv: u32,

    // External stimulus
    pub ext_pa: [ExtPinDrive; 16],
    pub ext_pb: [ExtPinDrive; 24],
    pub ext_pull_pa: [ExtPull; 16],
    pub ext_pull_pb: [ExtPull; 24],
    pub contention_log: Vec<String>,

    // Peripheral output overrides (e.g. PWM/TMR outputs)
    pub periph_out_pa_mask: u16,
    pub periph_out_pa_val: u16,
    pub periph_out_pb_mask: u32,
    pub periph_out_pb_val: u32,

    // Cached O(1) bitmasks derived from ext_pa/ext_pb/ext_pull_*
    ext_pa_driven_mask: u16,
    ext_pa_high_mask: u16,
    ext_pa_strong_high: u16,
    ext_pa_strong_low: u16,
    ext_pa_pullup_mask: u16,
    ext_pb_driven_mask: u32,
    ext_pb_high_mask: u32,
    ext_pb_strong_high: u32,
    ext_pb_strong_low: u32,
    ext_pb_pullup_mask: u32,

    // Previous pin samples for edge detection + dirty tracking
    prev_pa_pin: u16,
    prev_pb_pin: u32,
    pub version: u64,
}

impl Default for Gpio {
    fn default() -> Self {
        Self::new()
    }
}

impl Gpio {
    pub fn new() -> Self {
        // PB23 (RST) has built-in pull-up; PB14 (TIO) has built-in pull-up
        let mut pb_pu = (1 << 23) | (1 << 14);
        // PB23 defaults high via pull-up
        let _ = &mut pb_pu;
        Self {
            pin_alternate: 0,
            pin_config: 0,
            pin_in_dis: 0,
            pa_int_en: 0,
            pb_int_en: 0,
            pa_int_mode: 0,
            pb_int_mode: 0,
            pa_int_if: 0,
            pb_int_if: 0,
            pa_dir: 0,
            pa_out: 0,
            pa_pu: 0,
            pa_pd_drv: 0,
            pb_dir: 0,
            pb_out: 0,
            pb_pu,
            pb_pd_drv: 0,
            ext_pa: [ExtPinDrive::Floating; 16],
            ext_pb: [ExtPinDrive::Floating; 24],
            ext_pull_pa: [ExtPull::None; 16],
            ext_pull_pb: [ExtPull::None; 24],
            contention_log: Vec::new(),
            periph_out_pa_mask: 0,
            periph_out_pa_val: 0,
            periph_out_pb_mask: 0,
            periph_out_pb_val: 0,
            ext_pa_driven_mask: 0,
            ext_pa_high_mask: 0,
            ext_pa_strong_high: 0,
            ext_pa_strong_low: 0,
            ext_pa_pullup_mask: 0,
            ext_pb_driven_mask: 0,
            ext_pb_high_mask: 0,
            ext_pb_strong_high: 0,
            ext_pb_strong_low: 0,
            ext_pb_pullup_mask: 0,
            prev_pa_pin: 0,
            prev_pb_pin: (1 << 23) | (1 << 14),
            version: 1,
        }
    }

    #[inline]
    pub fn bump_version(&mut self) {
        self.version = self.version.wrapping_add(1);
    }

    #[inline]
    pub fn has_level_int_enabled(&self) -> bool {
        ((self.pa_int_en & !self.pa_int_mode) != 0) || ((self.pb_int_en & !self.pb_int_mode) != 0)
    }

    pub fn set_ext_drive(&mut self, port: char, idx: u8, drive: ExtPinDrive) {
        let (driven, high, sh, sl) = match drive {
            ExtPinDrive::High => (true, true, true, false),
            ExtPinDrive::Low => (true, false, false, true),
            ExtPinDrive::AnalogMv(mv) => (true, mv >= 1650, false, false),
            ExtPinDrive::Floating => (false, false, false, false),
        };
        match port {
            'A' | 'a' if (idx as usize) < 16 => {
                let i = idx as usize;
                let prev = self.ext_pa[i];
                self.ext_pa[i] = drive;
                let bit = 1u16 << idx;
                if driven { self.ext_pa_driven_mask |= bit; } else { self.ext_pa_driven_mask &= !bit; }
                if high { self.ext_pa_high_mask |= bit; } else { self.ext_pa_high_mask &= !bit; }
                if sh { self.ext_pa_strong_high |= bit; } else { self.ext_pa_strong_high &= !bit; }
                if sl { self.ext_pa_strong_low |= bit; } else { self.ext_pa_strong_low &= !bit; }
                if prev != drive {
                    self.bump_version();
                }
            }
            'B' | 'b' if (idx as usize) < 24 => {
                let i = idx as usize;
                let prev = self.ext_pb[i];
                self.ext_pb[i] = drive;
                let bit = 1u32 << idx;
                if driven { self.ext_pb_driven_mask |= bit; } else { self.ext_pb_driven_mask &= !bit; }
                if high { self.ext_pb_high_mask |= bit; } else { self.ext_pb_high_mask &= !bit; }
                if sh { self.ext_pb_strong_high |= bit; } else { self.ext_pb_strong_high &= !bit; }
                if sl { self.ext_pb_strong_low |= bit; } else { self.ext_pb_strong_low &= !bit; }
                if prev != drive {
                    self.bump_version();
                }
            }
            _ => {}
        }
    }

    pub fn set_ext_pull(&mut self, port: char, idx: u8, pull: ExtPull) {
        let up = pull == ExtPull::PullUp;
        match port {
            'A' | 'a' if (idx as usize) < 16 => {
                let i = idx as usize;
                let prev = self.ext_pull_pa[i];
                self.ext_pull_pa[i] = pull;
                let bit = 1u16 << idx;
                if up { self.ext_pa_pullup_mask |= bit; } else { self.ext_pa_pullup_mask &= !bit; }
                if prev != pull {
                    self.bump_version();
                }
            }
            'B' | 'b' if (idx as usize) < 24 => {
                let i = idx as usize;
                let prev = self.ext_pull_pb[i];
                self.ext_pull_pb[i] = pull;
                let bit = 1u32 << idx;
                if up { self.ext_pb_pullup_mask |= bit; } else { self.ext_pb_pullup_mask &= !bit; }
                if prev != pull {
                    self.bump_version();
                }
            }
            _ => {}
        }
    }

    pub fn rebuild_ext_masks(&mut self) {
        let mut pa_drv = 0u16;
        let mut pa_hi = 0u16;
        let mut pa_sh = 0u16;
        let mut pa_sl = 0u16;
        let mut pa_pu = 0u16;
        for i in 0..16 {
            let bit = 1u16 << i;
            match self.ext_pa[i] {
                ExtPinDrive::High => { pa_drv |= bit; pa_hi |= bit; pa_sh |= bit; }
                ExtPinDrive::Low => { pa_drv |= bit; pa_sl |= bit; }
                ExtPinDrive::AnalogMv(mv) => { pa_drv |= bit; if mv >= 1650 { pa_hi |= bit; } }
                ExtPinDrive::Floating => {}
            }
            if self.ext_pull_pa[i] == ExtPull::PullUp {
                pa_pu |= bit;
            }
        }
        let mut pb_drv = 0u32;
        let mut pb_hi = 0u32;
        let mut pb_sh = 0u32;
        let mut pb_sl = 0u32;
        let mut pb_pu = 0u32;
        for i in 0..24 {
            let bit = 1u32 << i;
            match self.ext_pb[i] {
                ExtPinDrive::High => { pb_drv |= bit; pb_hi |= bit; pb_sh |= bit; }
                ExtPinDrive::Low => { pb_drv |= bit; pb_sl |= bit; }
                ExtPinDrive::AnalogMv(mv) => { pb_drv |= bit; if mv >= 1650 { pb_hi |= bit; } }
                ExtPinDrive::Floating => {}
            }
            if self.ext_pull_pb[i] == ExtPull::PullUp {
                pb_pu |= bit;
            }
        }
        let changed = self.ext_pa_driven_mask != pa_drv
            || self.ext_pa_high_mask != pa_hi
            || self.ext_pa_strong_high != pa_sh
            || self.ext_pa_strong_low != pa_sl
            || self.ext_pa_pullup_mask != pa_pu
            || self.ext_pb_driven_mask != pb_drv
            || self.ext_pb_high_mask != pb_hi
            || self.ext_pb_strong_high != pb_sh
            || self.ext_pb_strong_low != pb_sl
            || self.ext_pb_pullup_mask != pb_pu;
        self.ext_pa_driven_mask = pa_drv;
        self.ext_pa_high_mask = pa_hi;
        self.ext_pa_strong_high = pa_sh;
        self.ext_pa_strong_low = pa_sl;
        self.ext_pa_pullup_mask = pa_pu;
        self.ext_pb_driven_mask = pb_drv;
        self.ext_pb_high_mask = pb_hi;
        self.ext_pb_strong_high = pb_sh;
        self.ext_pb_strong_low = pb_sl;
        self.ext_pb_pullup_mask = pb_pu;
        if changed {
            self.bump_version();
        }
    }

    #[inline]
    pub fn set_periph_out(&mut self, pa_mask: u16, pa_val: u16, pb_mask: u32, pb_val: u32) {
        if self.periph_out_pa_mask != pa_mask
            || self.periph_out_pa_val != pa_val
            || self.periph_out_pb_mask != pb_mask
            || self.periph_out_pb_val != pb_val
        {
            self.periph_out_pa_mask = pa_mask;
            self.periph_out_pa_val = pa_val;
            self.periph_out_pb_mask = pb_mask;
            self.periph_out_pb_val = pb_val;
            self.bump_version();
        }
    }

    #[inline]
    pub fn pin_level(&self, port: char, idx: u8) -> bool {
        match port {
            'A' | 'a' if idx < 16 => (self.effective_pa_pin() & (1 << idx)) != 0,
            'B' | 'b' if idx < 24 => (self.effective_pb_pin() & (1 << idx)) != 0,
            _ => false,
        }
    }

    #[inline]
    pub fn effective_pa_pin(&self) -> u16 {
        let in_dis = (self.pin_in_dis & 0xFFFF) as u16;
        let float_val = (self.pa_pu & !self.pa_pd_drv)
            | (!(self.pa_pu | self.pa_pd_drv) & self.ext_pa_pullup_mask);
        let in_val = (self.ext_pa_driven_mask & self.ext_pa_high_mask)
            | (!self.ext_pa_driven_mask & float_val);
        let mcu_val = (self.periph_out_pa_mask & self.periph_out_pa_val)
            | (!self.periph_out_pa_mask & ((self.pa_dir & self.pa_out) | (!self.pa_dir & in_val)));
        mcu_val & !in_dis
    }

    #[inline]
    pub fn effective_pb_pin(&self) -> u32 {
        let in_dis = ((self.pin_in_dis >> 16) & 0xFFFF) | (((self.pin_config as u32) & 0xFF00) << 8);
        let float_val = (self.pb_pu & !self.pb_pd_drv)
            | (!(self.pb_pu | self.pb_pd_drv) & self.ext_pb_pullup_mask);
        let in_val = (self.ext_pb_driven_mask & self.ext_pb_high_mask)
            | (!self.ext_pb_driven_mask & float_val);
        let mcu_val = (self.periph_out_pb_mask & self.periph_out_pb_val)
            | (!self.periph_out_pb_mask & ((self.pb_dir & self.pb_out) | (!self.pb_dir & in_val)));
        (mcu_val & !in_dis) & 0x00FF_FFFF
    }

    pub fn pa_electric(&self, idx: u8) -> PinElectricState {
        let i = idx as usize;
        let bit = 1u16 << i;
        if (self.pin_in_dis & (bit as u32)) != 0 && (self.pa_dir & bit) == 0 {
            return PinElectricState::Analog;
        }
        if (self.periph_out_pa_mask & bit) != 0 {
            return if (self.periph_out_pa_val & bit) != 0 {
                PinElectricState::High
            } else {
                PinElectricState::Low
            };
        }
        if (self.pa_dir & bit) != 0 {
            return if (self.pa_out & bit) != 0 {
                PinElectricState::High
            } else {
                PinElectricState::Low
            };
        }
        match self.ext_pa[i] {
            ExtPinDrive::High => PinElectricState::High,
            ExtPinDrive::Low => PinElectricState::Low,
            ExtPinDrive::AnalogMv(_) => PinElectricState::Analog,
            ExtPinDrive::Floating => {
                if (self.pa_pu & bit) != 0 && (self.pa_pd_drv & bit) == 0 {
                    PinElectricState::High
                } else if (self.pa_pd_drv & bit) != 0 {
                    PinElectricState::Low
                } else {
                    match self.ext_pull_pa[i] {
                        ExtPull::PullUp => PinElectricState::High,
                        ExtPull::PullDown => PinElectricState::Low,
                        ExtPull::None => PinElectricState::HiZ,
                    }
                }
            }
        }
    }

    pub fn pb_electric(&self, idx: u8) -> PinElectricState {
        let i = idx as usize;
        let bit = 1u32 << i;
        let in_dis = if i < 16 {
            (self.pin_in_dis & (bit << 16)) != 0
        } else {
            (self.pin_config & (1 << (i - 8))) != 0
        };
        if in_dis && (self.pb_dir & bit) == 0 {
            return PinElectricState::Analog;
        }
        if (self.periph_out_pb_mask & bit) != 0 {
            return if (self.periph_out_pb_val & bit) != 0 {
                PinElectricState::High
            } else {
                PinElectricState::Low
            };
        }
        if (self.pb_dir & bit) != 0 {
            return if (self.pb_out & bit) != 0 {
                PinElectricState::High
            } else {
                PinElectricState::Low
            };
        }
        match self.ext_pb[i] {
            ExtPinDrive::High => PinElectricState::High,
            ExtPinDrive::Low => PinElectricState::Low,
            ExtPinDrive::AnalogMv(_) => PinElectricState::Analog,
            ExtPinDrive::Floating => {
                if (self.pb_pu & bit) != 0 && (self.pb_pd_drv & bit) == 0 {
                    PinElectricState::High
                } else if (self.pb_pd_drv & bit) != 0 {
                    PinElectricState::Low
                } else {
                    match self.ext_pull_pb[i] {
                        ExtPull::PullUp => PinElectricState::High,
                        ExtPull::PullDown => PinElectricState::Low,
                        ExtPull::None => PinElectricState::HiZ,
                    }
                }
            }
        }
    }

    /// Update GPIO interrupt flags and fire PFIC IRQs if enabled (O(1) bitwise evaluation).
    pub fn evaluate_interrupts(&mut self, pfic: &mut Pfic) -> bool {
        let pa_now = self.effective_pa_pin();
        let pb_now = self.effective_pb_pin();
        let mut woke = false;

        if self.pa_int_en != 0 {
            let edge = self.pa_int_mode;
            let pol = self.pa_out;
            let rise = !self.prev_pa_pin & pa_now;
            let fall = self.prev_pa_pin & !pa_now;
            let edge_trig = edge & ((pol & rise) | (!pol & fall));
            let level_trig = !edge & !(pa_now ^ pol);
            let trig = self.pa_int_en & (edge_trig | level_trig);
            if trig != 0 {
                self.pa_int_if |= trig;
                woke = true;
            }
        }
        pfic.sync_level_irq(irq::GPIO_A, (self.pa_int_if & self.pa_int_en) != 0);

        if self.pb_int_en != 0 {
            let intx_remap = (self.pin_alternate & 0x2000) != 0;
            let pb16_sel = (self.pin_config & 0x0010) != 0;
            let b8_phys = if intx_remap {
                22
            } else if pb16_sel {
                16
            } else {
                8
            };
            let remap16 = |word: u32| -> u16 {
                let mut r = (word as u16) & !0x0100;
                r |= (((word >> b8_phys) & 1) as u16) << 8;
                if intx_remap {
                    r = (r & !0x0200) | ((((word >> 23) & 1) as u16) << 9);
                }
                r
            };
            let cur = remap16(pb_now);
            let prev = remap16(self.prev_pb_pin);
            let pol = remap16(self.pb_out);
            let edge = self.pb_int_mode;
            let rise = !prev & cur;
            let fall = prev & !cur;
            let edge_trig = edge & ((pol & rise) | (!pol & fall));
            let level_trig = !edge & !(cur ^ pol);
            let trig = self.pb_int_en & (edge_trig | level_trig);
            if trig != 0 {
                self.pb_int_if |= trig;
                woke = true;
            }
        }
        pfic.sync_level_irq(irq::GPIO_B, (self.pb_int_if & self.pb_int_en) != 0);

        self.prev_pa_pin = pa_now;
        self.prev_pb_pin = pb_now;
        woke
    }

    /// O(1) bitwise check for strong drive contention where MCU output opposes strong external drive.
    pub fn check_contention(&mut self) {
        if self.contention_log.len() >= 64 {
            return;
        }
        let pa_out_en = self.periph_out_pa_mask | self.pa_dir;
        let pa_out_val = (self.periph_out_pa_mask & self.periph_out_pa_val)
            | (!self.periph_out_pa_mask & self.pa_out);
        let pa_conflict = pa_out_en
            & ((pa_out_val & self.ext_pa_strong_low) | (!pa_out_val & self.ext_pa_strong_high));

        let pb_out_en = (self.periph_out_pb_mask | self.pb_dir) & 0x00FF_FFFF;
        let pb_out_val = (self.periph_out_pb_mask & self.periph_out_pb_val)
            | (!self.periph_out_pb_mask & self.pb_out);
        let pb_conflict = pb_out_en
            & ((pb_out_val & self.ext_pb_strong_low) | (!pb_out_val & self.ext_pb_strong_high));

        if (pa_conflict as u32) | pb_conflict == 0 {
            return;
        }

        let mut m = pa_conflict as u32;
        while m != 0 {
            let i = m.trailing_zeros();
            m &= m - 1;
            let mcu_high = ((pa_out_val >> i) & 1) != 0;
            let msg = if mcu_high {
                format!("PA{i} contention: MCU=HIGH vs EXT=LOW")
            } else {
                format!("PA{i} contention: MCU=LOW vs EXT=HIGH")
            };
            if !self.contention_log.contains(&msg) && self.contention_log.len() < 64 {
                self.contention_log.push(msg);
            }
        }
        let mut m = pb_conflict;
        while m != 0 {
            let i = m.trailing_zeros();
            m &= m - 1;
            let mcu_high = ((pb_out_val >> i) & 1) != 0;
            let msg = if mcu_high {
                format!("PB{i} contention: MCU=HIGH vs EXT=LOW")
            } else {
                format!("PB{i} contention: MCU=LOW vs EXT=HIGH")
            };
            if !self.contention_log.contains(&msg) && self.contention_log.len() < 64 {
                self.contention_log.push(msg);
            }
        }
    }

    pub fn read_u8(&self, offset: u32) -> u8 {
        match offset {
            0x18 => self.pin_alternate as u8,
            0x19 => (self.pin_alternate >> 8) as u8,
            0x1A => self.pin_config as u8,
            0x1B => (self.pin_config >> 8) as u8,
            0x1C..=0x1F => (self.pin_in_dis >> ((offset & 3) * 8)) as u8,

            0x90 => self.pa_int_en as u8,
            0x91 => (self.pa_int_en >> 8) as u8,
            0x92 => self.pb_int_en as u8,
            0x93 => (self.pb_int_en >> 8) as u8,
            0x94 => self.pa_int_mode as u8,
            0x95 => (self.pa_int_mode >> 8) as u8,
            0x96 => self.pb_int_mode as u8,
            0x97 => (self.pb_int_mode >> 8) as u8,
            0x9C => self.pa_int_if as u8,
            0x9D => (self.pa_int_if >> 8) as u8,
            0x9E => self.pb_int_if as u8,
            0x9F => (self.pb_int_if >> 8) as u8,

            0xA0 => self.pa_dir as u8,
            0xA1 => (self.pa_dir >> 8) as u8,
            0xA4 => self.effective_pa_pin() as u8,
            0xA5 => (self.effective_pa_pin() >> 8) as u8,
            0xA8 => self.pa_out as u8,
            0xA9 => (self.pa_out >> 8) as u8,
            0xB0 => self.pa_pu as u8,
            0xB1 => (self.pa_pu >> 8) as u8,
            0xB4 => self.pa_pd_drv as u8,
            0xB5 => (self.pa_pd_drv >> 8) as u8,

            0xC0..=0xC2 => (self.pb_dir >> ((offset - 0xC0) * 8)) as u8,
            0xC4..=0xC6 => (self.effective_pb_pin() >> ((offset - 0xC4) * 8)) as u8,
            0xC8..=0xCA => (self.pb_out >> ((offset - 0xC8) * 8)) as u8,
            0xD0..=0xD2 => (self.pb_pu >> ((offset - 0xD0) * 8)) as u8,
            0xD4..=0xD6 => (self.pb_pd_drv >> ((offset - 0xD4) * 8)) as u8,
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        let v16 = val as u16;
        match offset {
            0x18 => self.pin_alternate = (self.pin_alternate & 0xFF00) | v16,
            0x19 => self.pin_alternate = (self.pin_alternate & 0x00FF) | (v16 << 8),
            0x1A => self.pin_config = (self.pin_config & 0xFF00) | v16,
            0x1B => self.pin_config = (self.pin_config & 0x00FF) | (v16 << 8),
            0x1C..=0x1F => {
                let s = (offset & 3) * 8;
                self.pin_in_dis = (self.pin_in_dis & !(0xFF << s)) | ((val as u32) << s);
            }

            0x90 => self.pa_int_en = (self.pa_int_en & 0xFF00) | v16,
            0x91 => self.pa_int_en = (self.pa_int_en & 0x00FF) | (v16 << 8),
            0x92 => self.pb_int_en = (self.pb_int_en & 0xFF00) | v16,
            0x93 => self.pb_int_en = (self.pb_int_en & 0x00FF) | (v16 << 8),
            0x94 => self.pa_int_mode = (self.pa_int_mode & 0xFF00) | v16,
            0x95 => self.pa_int_mode = (self.pa_int_mode & 0x00FF) | (v16 << 8),
            0x96 => self.pb_int_mode = (self.pb_int_mode & 0xFF00) | v16,
            0x97 => self.pb_int_mode = (self.pb_int_mode & 0x00FF) | (v16 << 8),
            0x9C => self.pa_int_if &= !v16,
            0x9D => self.pa_int_if &= !(v16 << 8),
            0x9E => self.pb_int_if &= !v16,
            0x9F => self.pb_int_if &= !(v16 << 8),

            0xA0 => self.pa_dir = (self.pa_dir & 0xFF00) | v16,
            0xA1 => self.pa_dir = (self.pa_dir & 0x00FF) | (v16 << 8),
            0xA8 => self.pa_out = (self.pa_out & 0xFF00) | v16,
            0xA9 => self.pa_out = (self.pa_out & 0x00FF) | (v16 << 8),
            0xAC => self.pa_out &= !v16,
            0xAD => self.pa_out &= !(v16 << 8),
            0xB0 => self.pa_pu = (self.pa_pu & 0xFF00) | v16,
            0xB1 => self.pa_pu = (self.pa_pu & 0x00FF) | (v16 << 8),
            0xB4 => self.pa_pd_drv = (self.pa_pd_drv & 0xFF00) | v16,
            0xB5 => self.pa_pd_drv = (self.pa_pd_drv & 0x00FF) | (v16 << 8),
            0xB8 => self.pa_out |= v16,
            0xB9 => self.pa_out |= v16 << 8,

            0xC0..=0xC2 => {
                let s = (offset - 0xC0) * 8;
                self.pb_dir = (self.pb_dir & !(0xFF << s)) | ((val as u32) << s);
            }
            0xC8..=0xCA => {
                let s = (offset - 0xC8) * 8;
                self.pb_out = (self.pb_out & !(0xFF << s)) | ((val as u32) << s);
            }
            0xCC..=0xCE => {
                let s = (offset - 0xCC) * 8;
                self.pb_out &= !((val as u32) << s);
            }
            0xD0..=0xD2 => {
                let s = (offset - 0xD0) * 8;
                self.pb_pu = (self.pb_pu & !(0xFF << s)) | ((val as u32) << s);
            }
            0xD4..=0xD6 => {
                let s = (offset - 0xD4) * 8;
                self.pb_pd_drv = (self.pb_pd_drv & !(0xFF << s)) | ((val as u32) << s);
            }
            0xD8..=0xDA => {
                let s = (offset - 0xD8) * 8;
                self.pb_out |= (val as u32) << s;
            }
            _ => return,
        }
        self.bump_version();
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
