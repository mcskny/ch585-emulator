use crate::calibration::AdcCalibrationModel;
use crate::peripherals::gpio::{ExtPinDrive, Gpio};
use crate::peripherals::irq;
use crate::pfic::Pfic;

#[derive(Debug, Clone, Copy)]
pub enum AdcWaveform {
    Dc(i32),
    Sine {
        offset_mv: i32,
        amp_mv: i32,
        freq_hz: f32,
    },
    Ramp {
        min_mv: i32,
        max_mv: i32,
        period_ms: f32,
    },
    Pulse {
        base_mv: i32,
        peak_mv: i32,
        period_ms: f32,
        duty: f32,
    },
}

impl Default for AdcWaveform {
    fn default() -> Self {
        Self::Dc(1650)
    }
}

/// 14-channel 12-bit ADC + PGA + Multi-channel Scan + DMA + TouchKey stub (`0x4000_1054..0x4000_107F`).
#[derive(Debug, Clone)]
pub struct Adc {
    // Registers
    pub tkey_count: u8,   // 0x54
    pub tkey_convert: u8, // 0x56
    pub tkey_cfg: u8,     // 0x57
    pub channel: u8,      // 0x58
    pub cfg: u8,          // 0x59
    pub convert: u8,      // 0x5A
    pub tem_sensor: u8,   // 0x5B
    pub data: u16,        // 0x5C
    pub int_flag: u8,     // 0x5E
    pub ctrl_dma: u8,     // 0x61
    pub dma_if: u8,       // 0x62
    pub auto_cycle: u8,   // 0x63
    pub dma_now: u32,     // 0x64
    pub dma_beg: u32,     // 0x68
    pub dma_end: u32,     // 0x6C
    pub scan_cfg1: u32,   // 0x70
    pub scan_cfg2: u32,   // 0x74
    pub tkey_sel: u32,    // 0x7C

    // External signal sources (A0..A13 in mV) + internal VBAT & Temp
    pub channels: [AdcWaveform; 14],
    pub noise_mv: [u16; 14],
    pub vbat_mv: i32,
    pub temp_c: i32,
    rng_state: u64,

    // Internal conversion state
    converting: bool,
    conv_cycles_rem: u32,
    scan_step: u8,
    auto_timer: u32,

    // Calibration, RC settling & DMA activity
    pub calibration: AdcCalibrationModel,
    pub pin_source_ohm: [u32; 14],
    hold_cap_mv: f64,
    pub dma_active_step: bool,
}

impl Default for Adc {
    fn default() -> Self {
        Self::new()
    }
}

impl Adc {
    pub fn new() -> Self {
        Self {
            tkey_count: 0,
            tkey_convert: 0,
            tkey_cfg: 0,
            channel: 0,
            cfg: 0,
            convert: 0,
            tem_sensor: 0,
            data: 0,
            int_flag: 0,
            ctrl_dma: 0,
            dma_if: 0,
            auto_cycle: 0,
            dma_now: 0,
            dma_beg: 0,
            dma_end: 0,
            scan_cfg1: 0,
            scan_cfg2: 0,
            tkey_sel: 0,
            channels: [AdcWaveform::Dc(1650); 14],
            noise_mv: [0; 14],
            vbat_mv: 3300,
            temp_c: 25,
            rng_state: 0x5850_1234_ABCD_EF01,
            converting: false,
            conv_cycles_rem: 0,
            scan_step: 0,
            auto_timer: 0,
            calibration: AdcCalibrationModel::default(),
            pin_source_ohm: [0; 14],
            hold_cap_mv: 0.0,
            dma_active_step: false,
        }
    }

    /// Map ADC external channel index (0..13) to GPIO PA bit index.
    pub fn channel_to_pa_idx(ch: u8) -> Option<usize> {
        match ch {
            0 => Some(4),   // A0 = PA4
            1 => Some(5),   // A1 = PA5
            2 => Some(12),  // A2 = PA12
            3 => Some(13),  // A3 = PA13
            4 => Some(14),  // A4 = PA14
            5 => Some(15),  // A5 = PA15
            6 => Some(3),   // A6 = PA3
            7 => Some(2),   // A7 = PA2
            8 => Some(1),   // A8 = PA1
            9 => Some(0),   // A9 = PA0
            10 => Some(6),  // A10 = PA6
            11 => Some(7),  // A11 = PA7
            12 => Some(8),  // A12 = PA8
            13 => Some(9),  // A13 = PA9
            _ => None,
        }
    }

    #[inline]
    pub fn conv_cycles_remaining(&self) -> u32 {
        self.conv_cycles_rem
    }

    pub fn is_converting(&self) -> bool {
        self.converting || (self.ctrl_dma & 0x04) != 0
    }

    #[inline]
    pub fn next_event_cycles(&self) -> Option<u32> {
        if self.converting {
            Some(self.conv_cycles_rem.max(1))
        } else if (self.ctrl_dma & 0x80) != 0 {
            let period = ((256u32 - self.auto_cycle as u32).max(1)) * 16;
            Some(period.saturating_sub(self.auto_timer).max(1))
        } else {
            None
        }
    }

    #[inline]
    pub fn will_sample_in(&self, cycles: u32) -> bool {
        if cycles == 0 {
            return false;
        }
        if self.converting {
            cycles >= self.conv_cycles_rem
        } else if (self.ctrl_dma & 0x80) != 0 {
            let period = ((256u32 - self.auto_cycle as u32).max(1)) * 16;
            self.auto_timer.saturating_add(cycles) >= period
                && cycles >= self.conv_cycles_per_sample()
        } else {
            false
        }
    }

    pub fn sample_channel_mv(&self, ch: u8, time_s: f64, gpio: &Gpio) -> i32 {
        if ch < 14 {
            // If GPIO pin has an explicit AnalogMv or High/Low drive, let it override default
            if let Some(pa_idx) = Self::channel_to_pa_idx(ch) {
                match gpio.ext_pa[pa_idx] {
                    ExtPinDrive::AnalogMv(mv) => return mv as i32,
                    ExtPinDrive::High => return 3300,
                    ExtPinDrive::Low => return 0,
                    ExtPinDrive::Floating => {}
                }
            }
            match self.channels[ch as usize] {
                AdcWaveform::Dc(mv) => mv,
                AdcWaveform::Sine {
                    offset_mv,
                    amp_mv,
                    freq_hz,
                } => {
                    let phase = 2.0 * std::f64::consts::PI * (freq_hz as f64) * time_s;
                    offset_mv + ((amp_mv as f64) * phase.sin()).round() as i32
                }
                AdcWaveform::Ramp {
                    min_mv,
                    max_mv,
                    period_ms,
                } => {
                    let p = (period_ms.max(0.1) as f64) * 1e-3;
                    let frac = (time_s % p) / p;
                    min_mv + ((max_mv - min_mv) as f64 * frac).round() as i32
                }
                AdcWaveform::Pulse {
                    base_mv,
                    peak_mv,
                    period_ms,
                    duty,
                } => {
                    let p = (period_ms.max(0.1) as f64) * 1e-3;
                    let frac = (time_s % p) / p;
                    if frac < (duty as f64) { peak_mv } else { base_mv }
                }
            }
        } else if ch == 14 {
            self.vbat_mv
        } else {
            0
        }
    }

    pub fn set_noise_seed(&mut self, seed: u64) {
        self.rng_state = if seed == 0 {
            0x5850_1234_ABCD_EF01
        } else {
            seed
        };
    }

    pub fn set_all_noise_mv(&mut self, mv: u16) {
        self.noise_mv.fill(mv);
    }

    fn next_noise_mv(&mut self, ch: u8) -> i32 {
        if ch >= 14 {
            return 0;
        }
        let amp = self.noise_mv[ch as usize] as i32;
        if amp == 0 {
            return 0;
        }
        let mut x = self.rng_state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.rng_state = x;
        let span = (amp * 2 + 1) as u64;
        (x % span) as i32 - amp
    }

    fn voltage_to_raw12(&self, ch: u8, time_s: f64, gpio: &Gpio, noise_delta_mv: i32) -> u16 {
        if (self.cfg & 0x08) != 0 {
            // RB_ADC_OFS_TEST: ADC_DataCalib_Rough computes 2048 - ((~R16_ADC_DATA) & 0xFFF)
            // Return 2047 so (~2047)&0xFFF == 2048 => rough offset = 0
            return 2047;
        }
        if (self.cfg & 0x01) == 0 {
            return 0; // Powered off
        }

        let diff_en = (self.cfg & 0x04) != 0;
        let ga = (self.cfg >> 4) & 0x03;
        let gain2 = (self.convert & 0x02) != 0;

        if ch == 15 {
            // Internal temperature sensor calibrated at ~25 C -> 2048 at 25C
            let raw = 2048 + (self.temp_c - 25) * 283 / 100;
            return raw.clamp(0, 4095) as u16;
        }

        if !diff_en {
            let mv = if ch == 14 {
                // VBAT internal channel (divided by ~1/3 or 1050 offset; CH58x_adc uses -12dB: mV = adc*1050/512 - 3150)
                self.vbat_mv
            } else {
                (self.sample_channel_mv(ch, time_s, gpio) + noise_delta_mv).clamp(0, 3300)
            };
            let raw = match (gain2, ga) {
                (false, 0) => (mv + 3150) * 512 / 1050,     // -12 dB
                (false, 1) => (mv + 1050) * 1024 / 1050,    // -6 dB
                (false, 2) => mv * 2048 / 1050,             // 0 dB
                (false, 3) | (true, 0) => (mv - 525) * 4096 / 1050, // +6 dB
                (true, 1) => (mv - 788) * 8192 / 1050,      // +12 dB
                (true, 2) => (mv - 919) * 16384 / 1050,     // +18 dB
                (true, 3) => (mv - 984) * 32768 / 1050,     // +24 dB
                _ => 0,
            };
            raw.clamp(0, 4095) as u16
        } else {
            let (pos_ch, neg_ch) = if (ch & 1) == 0 { (0, 2) } else { (1, 3) };
            let dv = self.sample_channel_mv(pos_ch, time_s, gpio)
                - self.sample_channel_mv(neg_ch, time_s, gpio);
            let raw = match (gain2, ga) {
                (false, 0) => (dv + 4200) * 512 / 1050,
                (false, 1) => (dv + 2100) * 1024 / 1050,
                (false, 2) => (dv + 1050) * 2048 / 1050,
                (false, 3) | (true, 0) => (dv + 525) * 4096 / 1050,
                (true, 1) => (dv + 262) * 8192 / 1050,
                (true, 2) => (dv + 131) * 16384 / 1050,
                (true, 3) => (dv + 66) * 32768 / 1050,
                _ => 2048,
            };
            raw.clamp(0, 4095) as u16
        }
    }

    #[inline]
    fn scan_enabled(&self) -> bool {
        (self.scan_cfg2 & (1 << 28)) != 0
    }

    #[inline]
    fn scan_count(&self) -> u8 {
        (((self.scan_cfg2 >> 24) & 0x0F) as u8 + 1).min(14)
    }

    fn scan_channel_at(&self, step: u8) -> u8 {
        match step {
            0 => self.channel & 0x1F,
            1..=8 => ((self.scan_cfg1 >> ((step - 1) * 4)) & 0x0F) as u8,
            9..=13 => ((self.scan_cfg2 >> ((step - 9) * 4)) & 0x0F) as u8,
            _ => self.channel & 0x1F,
        }
    }

    fn conv_cycles_per_sample(&self) -> u32 {
        // ~17 Tadc; keep snappy in emulator (16..40 sys cycles per sample)
        let div_sel = (self.cfg >> 6) & 3;
        match div_sel {
            0 => 16,
            1 => 20,
            2 => 28,
            _ => 36,
        }
    }

    #[inline]
    fn current_gain_idx(&self) -> u8 {
        let ga = (self.cfg >> 4) & 0x03;
        let gain2 = (self.convert & 0x02) != 0;
        match (gain2, ga) {
            (false, 0) => 0,
            (false, 1) => 1,
            (false, 2) => 2,
            (false, 3) | (true, 0) => 3,
            (true, 1) => 4,
            (true, 2) => 5,
            _ => 6,
        }
    }

    fn voltage_to_raw12_calibrated(&mut self, ch: u8, time_s: f64, gpio: &Gpio, noise_delta_mv: i32) -> u16 {
        if self.calibration.is_default()
            || (self.cfg & 0x08) != 0
            || (self.cfg & 0x01) == 0
            || (self.cfg & 0x04) != 0
        {
            return self.voltage_to_raw12(ch, time_s, gpio, noise_delta_mv);
        }

        let gain_idx = self.current_gain_idx();
        let cal = self.calibration.lookup(ch, gain_idx).cloned();

        let mut mv = if ch == 14 {
            self.vbat_mv as f64
        } else if ch == 15 {
            let base = self.voltage_to_raw12(ch, time_s, gpio, noise_delta_mv) as f64;
            let adjusted = if let Some(c) = cal {
                base * c.gain_scale + c.offset_counts
            } else {
                base
            };
            return adjusted.round().clamp(0.0, 4095.0) as u16;
        } else {
            (self.sample_channel_mv(ch, time_s, gpio) + noise_delta_mv).clamp(0, 3300) as f64
        };

        if ch < 14 && self.calibration.settling.enabled {
            let settling = &self.calibration.settling;
            let ch_ohm = settling.channel_source_ohm[ch as usize];
            let r_ext = if ch_ohm > 0 {
                ch_ohm as f64
            } else {
                self.pin_source_ohm[ch as usize] as f64
            };
            let r_tot = r_ext + settling.internal_r_ohm.max(1.0);
            let c_f = settling.sample_cap_pf.max(0.1) * 1e-12;
            let div_scale = match (self.cfg >> 6) & 3 {
                0 => 1.0,
                1 => 1.25,
                2 => 1.75,
                _ => 2.25,
            };
            let t_samp_ns = settling.base_sample_ns.max(10.0) * div_scale;
            let tau_ns = (r_tot * c_f) * 1e9;
            let alpha = 1.0 - (-t_samp_ns / tau_ns.max(1e-6)).exp();
            self.hold_cap_mv += alpha * (mv - self.hold_cap_mv);
            mv = self.hold_cap_mv;
        }

        let mv_cal = (mv + cal.as_ref().map_or(0.0, |c| c.offset_mv)).clamp(0.0, 3300.0);
        let raw_base = if ch < 14 {
            if let Some(c) = cal.as_ref().filter(|c| !c.curve_mv_to_counts.is_empty()) {
                AdcCalibrationModel::eval_curve(&c.curve_mv_to_counts, mv_cal)
            } else {
                match gain_idx {
                    0 => (mv_cal + 3150.0) * 512.0 / 1050.0,
                    1 => (mv_cal + 1050.0) * 1024.0 / 1050.0,
                    2 => mv_cal * 2048.0 / 1050.0,
                    3 => (mv_cal - 525.0) * 4096.0 / 1050.0,
                    4 => (mv_cal - 788.0) * 8192.0 / 1050.0,
                    5 => (mv_cal - 919.0) * 16384.0 / 1050.0,
                    _ => (mv_cal - 984.0) * 32768.0 / 1050.0,
                }
            }
        } else {
            match gain_idx {
                0 => (mv_cal + 3150.0) * 512.0 / 1050.0,
                1 => (mv_cal + 1050.0) * 1024.0 / 1050.0,
                2 => mv_cal * 2048.0 / 1050.0,
                3 => (mv_cal - 525.0) * 4096.0 / 1050.0,
                4 => (mv_cal - 788.0) * 8192.0 / 1050.0,
                5 => (mv_cal - 919.0) * 16384.0 / 1050.0,
                _ => (mv_cal - 984.0) * 32768.0 / 1050.0,
            }
        };
        let adjusted = if let Some(c) = cal {
            raw_base * c.gain_scale + c.offset_counts
        } else {
            raw_base
        };
        adjusted.round().clamp(0.0, 4095.0) as u16
    }

    pub fn step(
        &mut self,
        cycles: u32,
        total_cycles: u64,
        sys_clk_hz: u32,
        gpio: &Gpio,
        pfic: &mut Pfic,
        sram: &mut [u8],
    ) {
        self.dma_active_step = false;
        // Auto-DMA periodic trigger
        if (self.ctrl_dma & 0x80) != 0 && !self.converting {
            let period = ((256u32 - self.auto_cycle as u32).max(1)) * 16;
            self.auto_timer += cycles;
            if self.auto_timer >= period {
                self.auto_timer = 0;
                self.start_conversion();
            }
        }

        if !self.converting {
            self.sync_irq(pfic);
            return;
        }

        if self.conv_cycles_rem > cycles {
            self.conv_cycles_rem -= cycles;
            self.sync_irq(pfic);
            return;
        }
        self.conv_cycles_rem = 0;

        let time_s = (total_cycles as f64) / (sys_clk_hz.max(1) as f64);
        let is_scan = self.scan_enabled();
        let active_ch = if is_scan {
            self.scan_channel_at(self.scan_step)
        } else {
            self.channel & 0x1F
        };

        let noise_delta = self.next_noise_mv(active_ch);
        let sample = self.voltage_to_raw12_calibrated(active_ch, time_s, gpio, noise_delta);
        self.data = sample;
        self.int_flag |= 0x80; // RB_ADC_IF_EOC

        // Write DMA if enabled
        if (self.ctrl_dma & 0x01) != 0 && self.dma_end > self.dma_beg {
            self.dma_active_step = true;
            let addr = (self.dma_now & 0x1FFFF) as usize;
            if addr + 2 <= sram.len() {
                sram[addr..addr + 2].copy_from_slice(&sample.to_le_bytes());
            }
            self.dma_now += 2;
            if self.dma_now >= self.dma_end {
                self.dma_if |= 0x08; // RB_ADC_IF_DMA_END
                if (self.ctrl_dma & 0x04) != 0 {
                    // Loop mode
                    self.dma_now = self.dma_beg;
                }
            }
        }
        self.dma_if |= 0x10; // RB_ADC_IF_END_ADC

        if is_scan {
            let total = self.scan_count();
            self.scan_step += 1;
            if self.scan_step < total {
                // Continue scanning next channel in queue
                self.conv_cycles_rem = self.conv_cycles_per_sample();
                self.sync_irq(pfic);
                return;
            }
            // Scan finished!
            self.scan_step = 0;
            self.converting = false;
            self.convert &= !0x81; // clear RB_ADC_START & RB_ADC_EOC_X
            self.scan_cfg2 |= 1 << 31; // RB_ADC_IF_SCAN_END
            self.sync_irq(pfic);
        } else {
            // Single or continuous conversion finished
            let cont = (self.ctrl_dma & 0x40) != 0 && (self.dma_if & 0x08) == 0;
            if cont {
                self.conv_cycles_rem = self.conv_cycles_per_sample();
            } else {
                self.converting = false;
                self.convert &= !0x81;
            }
            self.sync_irq(pfic);
        }
    }

    fn sync_irq(&self, pfic: &mut Pfic) {
        let eoc = (self.int_flag & 0x80) != 0;
        let dma_end = (self.ctrl_dma & 0x08) != 0 && (self.dma_if & 0x08) != 0;
        let scan_end = (self.scan_cfg2 & (1 << 29)) != 0 && (self.scan_cfg2 & (1 << 31)) != 0;
        pfic.sync_level_irq(irq::ADC, eoc || dma_end || scan_end);
    }

    fn start_conversion(&mut self) {
        self.converting = true;
        self.scan_step = 0;
        self.conv_cycles_rem = self.conv_cycles_per_sample();
        self.convert |= 0x81; // RB_ADC_START + RB_ADC_EOC_X while busy
        self.int_flag &= !0x80;
    }

    pub fn read_u8(&self, offset: u32) -> u8 {
        match offset {
            0x54 => self.tkey_count,
            0x56 => self.tkey_convert,
            0x57 => self.tkey_cfg,
            0x58 => self.channel,
            0x59 => self.cfg,
            0x5A => self.convert,
            0x5B => self.tem_sensor,
            0x5C => self.data as u8,
            0x5D => (self.data >> 8) as u8,
            0x5E => self.int_flag,
            0x61 => self.ctrl_dma,
            0x62 => self.dma_if,
            0x63 => self.auto_cycle,
            0x64..=0x67 => (self.dma_now >> ((offset - 0x64) * 8)) as u8,
            0x68..=0x6B => (self.dma_beg >> ((offset - 0x68) * 8)) as u8,
            0x6C..=0x6F => (self.dma_end >> ((offset - 0x6C) * 8)) as u8,
            0x70..=0x73 => (self.scan_cfg1 >> ((offset - 0x70) * 8)) as u8,
            0x74..=0x77 => (self.scan_cfg2 >> ((offset - 0x74) * 8)) as u8,
            0x7C..=0x7F => (self.tkey_sel >> ((offset - 0x7C) * 8)) as u8,
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        match offset {
            0x54 => self.tkey_count = val,
            0x56 => {
                // TouchKey_ExcutSingleConver sets RB_TKEY_START=1 and polls until 0
                self.tkey_convert = val & !0x01;
                self.data = 1800; // baseline capacitance count
                self.int_flag |= 0x80;
            }
            0x57 => self.tkey_cfg = val,
            0x58 => self.channel = val,
            0x59 => self.cfg = val,
            0x5A => {
                // Writing R8_ADC_CONVERT clears RB_ADC_IF_EOC; bit0=1 starts conversion if idle
                self.int_flag &= !0x80;
                self.convert = (self.convert & 0x80) | (val & 0x7F);
                if (val & 0x01) != 0 {
                    if !self.converting {
                        self.start_conversion();
                    }
                } else {
                    self.converting = false;
                    self.convert &= !0x81;
                }
            }
            0x5B => self.tem_sensor = val,
            0x61 => self.ctrl_dma = val,
            0x62 => self.dma_if &= !val, // RW1C
            0x63 => self.auto_cycle = val,
            0x64..=0x67 => {
                let s = (offset - 0x64) * 8;
                self.dma_now = (self.dma_now & !(0xFF << s)) | ((val as u32) << s);
            }
            0x68..=0x6B => {
                let s = (offset - 0x68) * 8;
                self.dma_beg = (self.dma_beg & !(0xFF << s)) | ((val as u32) << s);
                self.dma_now = self.dma_beg;
            }
            0x6C..=0x6F => {
                let s = (offset - 0x6C) * 8;
                self.dma_end = (self.dma_end & !(0xFF << s)) | ((val as u32) << s);
            }
            0x70..=0x73 => {
                let s = (offset - 0x70) * 8;
                self.scan_cfg1 = (self.scan_cfg1 & !(0xFF << s)) | ((val as u32) << s);
            }
            0x74..=0x77 => {
                let s = (offset - 0x74) * 8;
                if offset == 0x77 {
                    // Bit 7 of byte 3 is RB_ADC_IF_SCAN_END (RW1C)
                    let clear_if = (val & 0x80) != 0;
                    let low = val & 0x7F;
                    let old_if = self.scan_cfg2 & (1 << 31);
                    self.scan_cfg2 = (self.scan_cfg2 & 0x80FF_FFFF) | ((low as u32) << 24);
                    if clear_if {
                        self.scan_cfg2 &= !(1 << 31);
                    } else {
                        self.scan_cfg2 |= old_if;
                    }
                } else {
                    self.scan_cfg2 = (self.scan_cfg2 & !(0xFF << s)) | ((val as u32) << s);
                }
            }
            0x7C..=0x7F => {
                let s = (offset - 0x7C) * 8;
                self.tkey_sel = (self.tkey_sel & !(0xFF << s)) | ((val as u32) << s);
            }
            _ => {}
        }
    }

    pub fn read_u32(&self, offset: u32) -> u32 {
        match offset {
            0x58 => u32::from_le_bytes([self.channel, self.cfg, self.convert, self.tem_sensor]),
            0x5C => (self.data as u32) | ((self.int_flag as u32) << 16),
            0x60 => u32::from_le_bytes([0, self.ctrl_dma, self.dma_if, self.auto_cycle]),
            0x64 => self.dma_now,
            0x68 => self.dma_beg,
            0x6C => self.dma_end,
            0x70 => self.scan_cfg1,
            0x74 => self.scan_cfg2,
            0x7C => self.tkey_sel,
            _ => {
                let b0 = self.read_u8(offset) as u32;
                let b1 = self.read_u8(offset + 1) as u32;
                let b2 = self.read_u8(offset + 2) as u32;
                let b3 = self.read_u8(offset + 3) as u32;
                b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
            }
        }
    }

    pub fn write_u32(&mut self, offset: u32, val: u32) {
        match offset {
            0x64 => self.dma_now = val,
            0x68 => {
                self.dma_beg = val;
                self.dma_now = val;
            }
            0x6C => self.dma_end = val,
            0x70 => self.scan_cfg1 = val,
            0x74 => {
                let clear_if = (val & (1 << 31)) != 0;
                let old_if = self.scan_cfg2 & (1 << 31);
                self.scan_cfg2 = val & !(1 << 31);
                if !clear_if {
                    self.scan_cfg2 |= old_if;
                }
            }
            0x7C => self.tkey_sel = val,
            _ => {
                let b = val.to_le_bytes();
                for i in 0..4 {
                    self.write_u8(offset + i as u32, b[i]);
                }
            }
        }
    }
}
