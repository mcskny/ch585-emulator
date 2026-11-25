use crate::peripherals::irq;
use crate::pfic::Pfic;

pub const RST_FLAG_SW: u8 = 0x00;
pub const RST_FLAG_RPOR: u8 = 0x01;
pub const RST_FLAG_WTR: u8 = 0x02;
pub const RST_FLAG_MR: u8 = 0x03;
pub const RST_FLAG_GPWSM: u8 = 0x05;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerMode {
    Active = 0,
    Idle = 1,
    Halt = 2,
    Sleep = 3,
    Shutdown = 4,
}

/// Datasheet-derived typical current draw table (uA) for CH585 core modes and peripherals.
#[derive(Debug, Clone, Copy)]
pub struct CurrentTable {
    pub active_dcdc_ua: f64,
    pub active_ldo_ua: f64,
    pub idle_ua: f64,
    pub halt_ua: f64,
    pub sleep_ua: f64,
    pub shutdown_ua: f64,
    pub adc_ua: f64,
    pub usbhs_active_ua: f64,
    pub usbhs_suspend_ua: f64,
    pub ble_rf_ua: f64,
    pub uart_ua: f64,
    pub spi_ua: f64,
    pub tmr_ua: f64,
    pub pwm_ua: f64,
    pub bat_mon_ha_ua: f64,
    pub bat_mon_lp_ua: f64,
}

impl Default for CurrentTable {
    fn default() -> Self {
        Self {
            active_dcdc_ua: 4_000.0,
            active_ldo_ua: 6_000.0,
            idle_ua: 1_800.0,
            halt_ua: 300.0,
            sleep_ua: 2.5,
            shutdown_ua: 0.4,
            adc_ua: 400.0,
            usbhs_active_ua: 12_000.0,
            usbhs_suspend_ua: 300.0,
            ble_rf_ua: 6_000.0,
            uart_ua: 100.0,
            spi_ua: 120.0,
            tmr_ua: 80.0,
            pwm_ua: 80.0,
            bat_mon_ha_ua: 210.0,
            bat_mon_lp_ua: 1.0,
        }
    }
}

/// Accumulated nanosecond-weighted activity counters for per-peripheral current accounting.
#[derive(Debug, Clone, Default)]
pub struct PeriphTimeNs {
    pub active_dcdc_ns: u64,
    pub active_ldo_ns: u64,
    pub adc_ns: u64,
    pub usbhs_active_ns: u64,
    pub usbhs_suspend_ns: u64,
    pub ble_rf_ns: u64,
    pub uart_unit_ns: u64,
    pub spi_unit_ns: u64,
    pub tmr_unit_ns: u64,
    pub pwm_ns: u64,
    pub bat_mon_ha_ns: u64,
    pub bat_mon_lp_ns: u64,
}

/// Per-mode and per-peripheral average current breakdown (uA).
#[derive(Debug, Clone, Default)]
pub struct PowerReport {
    pub total_time_ns: u64,
    pub unpowered_ns: u64,
    pub mode_ns: [u64; 5],
    pub core_avg_ua: f64,
    pub adc_avg_ua: f64,
    pub usbhs_avg_ua: f64,
    pub ble_rf_avg_ua: f64,
    pub uart_avg_ua: f64,
    pub spi_avg_ua: f64,
    pub tmr_avg_ua: f64,
    pub pwm_avg_ua: f64,
    pub bat_mon_avg_ua: f64,
    pub total_avg_ua: f64,
}

/// System control, Safe Access, Clocks, Power, IWDG/WDOG, OSC Calibration, and Flash SPI controller.
#[derive(Debug, Clone)]
pub struct SysCtrl {
    // Safe Access (`0x4000_1040`)
    safe_stage: u8, // 0=locked, 1=saw 0x57, 2=unlocked
    safe_timer: u8, // remaining cycles (approx 16..32)
    pub safe_mode_ctrl: u8, // 0x10
    pub safe_clk_ctrl: u8,  // 0x11
    pub safe_debug_ctrl: u8,// 0x12
    pub safe_lrst_ctrl: u8, // 0x13

    // Global & Reset (`0x4000_1044..0x4000_1047`)
    pub glob_rom_cfg: u8,   // 0x44 (low 3 bits = reset_flag RO)
    pub reset_flag: u8,     // 0x01 = RPOR, 0x00 = SW, 0x02 = WTR
    pub glob_cfg_info: u8,  // 0x45
    pub rst_wdog_ctrl: u8,  // 0x46
    pub glob_reset_keep: u8,// 0x47
    pub wdog_count: u8,     // 0x43
    wdog_prescaler: u32,

    // IWDG (`0x4000_1000..0x4000_1007`)
    pub iwdg_cfg: u32,      // bits[11:0]=RLR, [14:12]=PR, bit15=PVU, [27:16]=CNT, bit29=follow_stop
    iwdg_enabled: bool,
    iwdg_unprotected: bool,
    iwdg_counter: u16,
    iwdg_tick_acc: u64,

    // Clocks (`0x4000_1008..0x4000_100B`, `0x4000_102C..0x4000_102F`, `0x4000_1048..0x4000_1053`)
    pub clk_sys_cfg: u16,   // 0x08
    pub hfck_pwr_ctrl: u8,  // 0x0A
    pub misc_ctrl: u32,     // 0x48 (byte3 at 0x4B = pll_config)
    pub xt32m_tune: u8,     // 0x4E
    pub int32k_tune: u16,   // 0x2C
    pub xt32k_tune: u8,     // 0x2E
    pub ck32k_config: u8,   // 0x2F
    /// External 32.768 kHz crystal failure fault injection
    pub lse_failed: bool,

    // Sleep / Power (`0x4000_100C..0x4000_100F`, `0x4000_1020..0x4000_1026`)
    pub slp_clk_off0: u8,   // 0x0C
    pub slp_clk_off1: u8,   // 0x0D
    pub slp_wake_ctrl: u8,  // 0x0E
    pub slp_power_ctrl: u8, // 0x0F
    pub power_plan: u16,    // 0x20
    pub aux_power_adj: u16, // 0x22
    pub bat_det_ctrl: u8,   // 0x24
    pub bat_det_cfg: u8,    // 0x25
    pub bat_status: u8,     // 0x26
    bat_irq_latched: bool,

    // Watchdog time-skip remainder tracking
    wdog_ns_rem: u64,
    iwdg_ns_rem: u64,

    // Osc Calib (`0x4000_1050..0x4000_1053`)
    pub osc_cal_cnt: u16,
    pub osc_cal_ov_cnt: u8,
    pub osc_cal_ctrl: u8,
    osc_cal_active_cycles: u32,

    // Requests to SoC
    pub sw_reset_requested: bool,
    pub wdog_reset_requested: bool,
}

impl Default for SysCtrl {
    fn default() -> Self {
        Self::new()
    }
}

impl SysCtrl {
    pub fn new() -> Self {
        Self {
            safe_stage: 0,
            safe_timer: 0,
            safe_mode_ctrl: 0x01, // RB_SAFE_AUTO_EN default 1
            safe_clk_ctrl: 0,
            safe_debug_ctrl: 0,
            safe_lrst_ctrl: 0,
            glob_rom_cfg: 0x20,   // ROM ctrl enabled
            reset_flag: 0x01,     // RST_FLAG_RPOR
            glob_cfg_info: 0x09,  // ROM_READ | BOOT_EN (CFG_RESET_EN=0 so PB23 works as GPIO/UART2)
            rst_wdog_ctrl: 0,
            glob_reset_keep: 0,
            wdog_count: 0,
            wdog_prescaler: 0,
            iwdg_cfg: 0x0FFF_0FFF, // RLR=0xFFF, CNT=0xFFF
            iwdg_enabled: false,
            iwdg_unprotected: false,
            iwdg_counter: 0x0FFF,
            iwdg_tick_acc: 0,
            clk_sys_cfg: 0x0003,  // Default 16MHz / 3 = 5.33MHz per CH585SFR.h
            hfck_pwr_ctrl: 0x02,  // RC16M on
            misc_ctrl: 0,
            xt32m_tune: 0x01,
            int32k_tune: 0x7F << 5,
            xt32k_tune: 0x01,
            ck32k_config: 0x02,   // INT32K on
            lse_failed: false,
            slp_clk_off0: 0,
            slp_clk_off1: 0,
            slp_wake_ctrl: 0,
            slp_power_ctrl: 0,
            power_plan: 0x009F,
            aux_power_adj: 0,
            bat_det_ctrl: 0,
            bat_det_cfg: 0,
            bat_status: 0,
            bat_irq_latched: false,
            wdog_ns_rem: 0,
            iwdg_ns_rem: 0,
            osc_cal_cnt: 0,
            osc_cal_ov_cnt: 0,
            osc_cal_ctrl: 0,
            osc_cal_active_cycles: 0,
            sw_reset_requested: false,
            wdog_reset_requested: false,
        }
    }

    pub fn soft_reset_preserve_keep(&mut self, reset_flag: u8) {
        let keep = self.glob_reset_keep;
        let lse_failed = self.lse_failed;
        *self = Self::new();
        self.glob_reset_keep = keep;
        self.lse_failed = lse_failed;
        self.reset_flag = reset_flag;
    }

    pub fn power_on_reset(&mut self) {
        let lse_failed = self.lse_failed;
        *self = Self::new();
        self.lse_failed = lse_failed;
        self.reset_flag = RST_FLAG_RPOR;
    }

    #[inline]
    pub fn is_bat_monitor_enabled(&self) -> bool {
        (self.bat_det_ctrl & 0x03) != 0
    }

    /// Evaluate CH585 battery / VDD voltage monitor (`R8_BAT_DET_CTRL`, `R8_BAT_DET_CFG`, `R8_BAT_STATUS`).
    pub fn evaluate_bat_monitor(&mut self, vdd_mv: i32, pfic: &mut Pfic) {
        let (lower_mv, low_mv) = if (self.bat_det_ctrl & 0x02) != 0 {
            // Low-power monitor (LPLevel_1V7..LPLevel_2V4, ~1 uA)
            let idx = (((self.bat_det_ctrl & 0x01) << 2) | (self.bat_det_cfg & 0x03)) as i32;
            let low = 1700 + idx * 100;
            (low - 100, low)
        } else if (self.bat_det_ctrl & 0x01) != 0 {
            // High-accuracy monitor (HALevel_1V9..HALevel_2V5, ~210 uA)
            match self.bat_det_cfg & 0x03 {
                0 => (1700, 1950),
                1 => (1900, 2150),
                2 => (2100, 2350),
                _ => (2300, 2550),
            }
        } else {
            self.bat_status = 0;
            self.bat_irq_latched = false;
            return;
        };

        let mut st = 0u8;
        if vdd_mv < lower_mv {
            st |= 0x01; // RB_BAT_STAT_LOWER
        }
        if vdd_mv < low_mv {
            st |= 0x02; // RB_BAT_STAT_LOW
        }
        self.bat_status = st;

        let lower_ie = (self.bat_det_ctrl & 0x04) != 0;
        let low_ie = (self.bat_det_ctrl & 0x08) != 0;
        let triggered = (lower_ie && (st & 0x01) != 0) || (low_ie && (st & 0x02) != 0);
        if triggered && !self.bat_irq_latched {
            self.bat_irq_latched = true;
            if lower_ie && low_ie {
                pfic.set_pending(irq::NMI);
            } else {
                pfic.set_pending(irq::WDOG_BAT);
            }
        } else if !triggered {
            self.bat_irq_latched = false;
        }
    }

    pub fn next_wdog_ns(&self, deep_pwr_down: bool) -> Option<u64> {
        let mut best: Option<u64> = None;
        if !deep_pwr_down && (self.rst_wdog_ctrl & 0x06) != 0 {
            let sys_hz = self.sys_clk_hz().max(32_000) as u128;
            let rem_inc = (256u64 - self.wdog_count as u64).max(1);
            let rem_cyc = (rem_inc * 131_072).saturating_sub(self.wdog_prescaler as u64) as u128;
            let scaled = (rem_cyc * 1_000_000_000u128).saturating_sub(self.wdog_ns_rem as u128);
            let ns = scaled.div_ceil(sys_hz).max(1) as u64;
            best = Some(ns);
        }
        if self.iwdg_enabled {
            let pr_shift = ((self.iwdg_cfg >> 12) & 0x7) as u32;
            let div = (4u128) << pr_shift;
            let rem_decr = (self.iwdg_counter as u128) + 1;
            let scaled = (rem_decr * div * 1_000_000_000u128).saturating_sub(self.iwdg_ns_rem as u128);
            let ns = scaled.div_ceil(32_000).max(1) as u64;
            best = Some(best.map_or(ns, |b| b.min(ns)));
        }
        best
    }

    pub fn advance_wdog_ns(&mut self, dt_ns: u64, deep_pwr_down: bool, pfic: &mut Pfic) {
        if dt_ns == 0 {
            return;
        }
        let sys_hz = self.sys_clk_hz().max(32_000) as u128;
        if !deep_pwr_down {
            let total_cyc_scaled = (self.wdog_ns_rem as u128) + (dt_ns as u128) * sys_hz;
            let cyc = (total_cyc_scaled / 1_000_000_000u128) as u64;
            self.wdog_ns_rem = (total_cyc_scaled % 1_000_000_000u128) as u64;

            if self.safe_stage == 2 && (self.safe_mode_ctrl & 0x01) != 0 && cyc > 0 {
                if (self.safe_timer as u64) > cyc {
                    self.safe_timer -= cyc as u8;
                } else {
                    self.safe_stage = 0;
                    self.safe_timer = 0;
                }
            }

            if (self.rst_wdog_ctrl & 0x06) != 0 && cyc > 0 {
                let total_pre = (self.wdog_prescaler as u64) + cyc;
                let incs = total_pre / 131_072;
                self.wdog_prescaler = (total_pre % 131_072) as u32;
                if incs > 0 {
                    let sum = (self.wdog_count as u64) + incs;
                    self.wdog_count = (sum & 0xFF) as u8;
                    if sum >= 256 {
                        self.rst_wdog_ctrl |= 0x10;
                        if (self.rst_wdog_ctrl & 0x04) != 0 {
                            pfic.set_pending(irq::WDOG_BAT);
                        }
                        if (self.rst_wdog_ctrl & 0x02) != 0 {
                            self.wdog_reset_requested = true;
                        }
                    }
                }
            }
        }

        if self.iwdg_enabled {
            let pr_shift = ((self.iwdg_cfg >> 12) & 0x7) as u32;
            let div = (4u128) << pr_shift;
            let total_scaled = (self.iwdg_ns_rem as u128) + (dt_ns as u128) * 32_000u128;
            let step_den = div * 1_000_000_000u128;
            let decrs = total_scaled / step_den;
            self.iwdg_ns_rem = (total_scaled % step_den) as u64;
            if decrs > (self.iwdg_counter as u128) {
                self.iwdg_counter = 0;
                self.wdog_reset_requested = true;
            } else {
                self.iwdg_counter -= decrs as u16;
            }
            self.iwdg_cfg = (self.iwdg_cfg & !(0xFFF << 16)) | ((self.iwdg_counter as u32) << 16);
        }
    }

    #[inline]
    pub fn is_safe_unlocked(&self) -> bool {
        self.safe_stage == 2
    }

    /// Current system clock frequency in Hz, matching `GetSysClock()` in `CH58x_sys.c`.
    pub fn sys_clk_hz(&self) -> u32 {
        let mode = self.clk_sys_cfg & 0xC0;
        if mode == 0xC0 {
            return 32_000;
        }
        let base_osc = if (self.clk_sys_cfg & 0x0200) != 0 {
            32_000_000
        } else {
            16_000_000
        };
        let div = (self.clk_sys_cfg & 0x1F) as u32;
        if (self.clk_sys_cfg & 0x0100) != 0 {
            if div == 0 {
                base_osc
            } else {
                312_000_000 / div
            }
        } else if div == 0 {
            base_osc
        } else {
            base_osc / div
        }
    }

    /// Approximate 32K clock bit toggling for RTC_InitTime / RTC_SetCycle32k polling.
    pub fn ck32k_pin_state(&self, total_cycles: u64) -> u8 {
        if self.lse_failed && (self.ck32k_config & 0x04) != 0 {
            return 0;
        }
        let fsys = self.sys_clk_hz().max(32_000) as u64;
        let half_period = (fsys / 64_000).max(1);
        if ((total_cycles / half_period) & 1) != 0 {
            0x80
        } else {
            0
        }
    }

    pub fn step(&mut self, cycles: u32, total_cycles: u64, pfic: &mut Pfic) {
        // Safe access auto-close timer
        if self.safe_stage == 2 && (self.safe_mode_ctrl & 0x01) != 0 {
            if (self.safe_timer as u32) > cycles {
                self.safe_timer -= cycles as u8;
            } else {
                self.safe_stage = 0;
                self.safe_timer = 0;
            }
        }

        // Legacy 8-bit WDOG (counts at Fsys / 131072)
        if (self.rst_wdog_ctrl & 0x06) != 0 {
            self.wdog_prescaler += cycles;
            while self.wdog_prescaler >= 131_072 {
                self.wdog_prescaler -= 131_072;
                let (next, overflow) = self.wdog_count.overflowing_add(1);
                self.wdog_count = next;
                if overflow {
                    self.rst_wdog_ctrl |= 0x10; // RB_WDOG_INT_FLAG
                    if (self.rst_wdog_ctrl & 0x04) != 0 {
                        pfic.set_pending(irq::WDOG_BAT);
                    }
                    if (self.rst_wdog_ctrl & 0x02) != 0 {
                        self.wdog_reset_requested = true;
                    }
                }
            }
        }

        // Independent WDOG (IWDG) running off 32K LSI
        if self.iwdg_enabled {
            let fsys = self.sys_clk_hz().max(32_000) as u64;
            let pr_shift = ((self.iwdg_cfg >> 12) & 0x7) as u32;
            let div = 4u64 << pr_shift; // 4..512
            let cycles_per_tick = (fsys * div / 32_000).max(1);
            self.iwdg_tick_acc += cycles as u64;
            while self.iwdg_tick_acc >= cycles_per_tick {
                self.iwdg_tick_acc -= cycles_per_tick;
                if self.iwdg_counter == 0 {
                    self.wdog_reset_requested = true;
                    break;
                } else {
                    self.iwdg_counter -= 1;
                }
            }
            self.iwdg_cfg = (self.iwdg_cfg & !(0xFFF << 16)) | ((self.iwdg_counter as u32) << 16);
        }

        // Oscillator calibration counter (for Calibration_LSI)
        if (self.osc_cal_ctrl & 0x20) != 0 && (self.osc_cal_ctrl & 0x08) == 0 {
            // RB_OSC_CNT_EN=1 and not yet halted
            let total_idx = (self.osc_cal_ctrl & 0x07) as usize;
            let target_32k_cycles: u32 = match total_idx {
                0 => 1,
                1 => 2,
                2 => 4,
                3 => 32,
                4 => 64,
                5 => 128,
                6 => 1024,
                _ => 2047,
            };
            let fsys = self.sys_clk_hz();
            let needed_sys = (fsys as u64 * target_32k_cycles as u64 / 32_000).max(4) as u32;
            self.osc_cal_active_cycles += cycles;
            if self.osc_cal_active_cycles >= needed_sys {
                let total_cnt = needed_sys;
                self.osc_cal_ov_cnt = (total_cnt / 0x3FFF).min(255) as u8;
                self.osc_cal_cnt = (total_cnt % 0x3FFF) as u16 | 0x8000; // set RB_OSC_CAL_IF
                self.osc_cal_ctrl |= 0x08; // RB_OSC_CNT_HALT
                self.osc_cal_active_cycles = 0;
            }
        }
        let _ = total_cycles;
    }

    pub fn read_u8(&self, offset: u32, total_cycles: u64) -> u8 {
        match offset {
            0x00..=0x03 => (self.read_iwdg(0x00) >> ((offset & 3) * 8)) as u8,
            0x04..=0x07 => (self.read_iwdg(0x04) >> ((offset & 3) * 8)) as u8,
            0x08 => self.clk_sys_cfg as u8,
            0x09 => (self.clk_sys_cfg >> 8) as u8,
            0x0A => self.hfck_pwr_ctrl,
            0x0C => self.slp_clk_off0,
            0x0D => self.slp_clk_off1,
            0x0E => self.slp_wake_ctrl,
            0x0F => self.slp_power_ctrl,
            0x10 => self.safe_mode_ctrl,
            0x11 => self.safe_clk_ctrl,
            0x12 => self.safe_debug_ctrl,
            0x13 => self.safe_lrst_ctrl,
            0x20 => self.power_plan as u8,
            0x21 => (self.power_plan >> 8) as u8,
            0x22 => self.aux_power_adj as u8,
            0x23 => (self.aux_power_adj >> 8) as u8,
            0x24 => self.bat_det_ctrl,
            0x25 => self.bat_det_cfg,
            0x26 => self.bat_status,
            0x2C => self.int32k_tune as u8,
            0x2D => (self.int32k_tune >> 8) as u8,
            0x2E => self.xt32k_tune,
            0x2F => (self.ck32k_config & 0x7F) | self.ck32k_pin_state(total_cycles),
            0x40 => {
                // R8_SAFE_ACCESS_SIG read: RB_SAFE_ACC_ACT (bit3), RB_SAFE_ACC_MODE (bits1:0=3 if active)
                if self.is_safe_unlocked() {
                    0x0B | ((self.safe_timer & 0x7) << 4)
                } else {
                    self.safe_stage & 0x03
                }
            }
            0x41 => 0x93, // R8_CHIP_ID = ID_CH585
            0x42 => 0x0C, // R8_SAFE_ACCESS_ID = 0x0C
            0x43 => self.wdog_count,
            0x44 => (self.glob_rom_cfg & 0xF8) | (self.reset_flag & 0x07),
            0x45 => self.glob_cfg_info,
            0x46 => self.rst_wdog_ctrl,
            0x47 => self.glob_reset_keep,
            0x48..=0x4B => (self.misc_ctrl >> ((offset & 3) * 8)) as u8,
            0x4E => self.xt32m_tune,
            0x50 => self.osc_cal_cnt as u8,
            0x51 => (self.osc_cal_cnt >> 8) as u8,
            0x52 => self.osc_cal_ov_cnt,
            0x53 => self.osc_cal_ctrl,
            _ => 0,
        }
    }

    fn read_iwdg(&self, offset: u32) -> u32 {
        match offset {
            0x04 => self.iwdg_cfg & !(1 << 15), // PVU=0 (ready)
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        match offset {
            0x08 => {
                if self.is_safe_unlocked() {
                    self.clk_sys_cfg = (self.clk_sys_cfg & 0xFF00) | (val as u16);
                }
            }
            0x09 => {
                if self.is_safe_unlocked() {
                    self.clk_sys_cfg = (self.clk_sys_cfg & 0x00FF) | ((val as u16) << 8);
                }
            }
            0x0A => {
                if self.is_safe_unlocked() {
                    self.hfck_pwr_ctrl = val;
                }
            }
            0x0C => {
                if self.is_safe_unlocked() {
                    self.slp_clk_off0 = val;
                }
            }
            0x0D => {
                if self.is_safe_unlocked() {
                    self.slp_clk_off1 = val;
                }
            }
            0x0E => {
                if self.is_safe_unlocked() {
                    self.slp_wake_ctrl = val;
                }
            }
            0x0F => {
                if self.is_safe_unlocked() {
                    self.slp_power_ctrl = val;
                }
            }
            0x10 => self.safe_mode_ctrl = val,
            0x11 => {
                if self.is_safe_unlocked() {
                    self.safe_clk_ctrl = val;
                }
            }
            0x12 => {
                if self.is_safe_unlocked() {
                    self.safe_debug_ctrl = val;
                }
            }
            0x13 => {
                if self.is_safe_unlocked() {
                    self.safe_lrst_ctrl = val;
                }
            }
            0x20 => {
                if self.is_safe_unlocked() {
                    self.power_plan = (self.power_plan & 0xFF00) | (val as u16);
                }
            }
            0x21 => {
                if self.is_safe_unlocked() {
                    self.power_plan = (self.power_plan & 0x00FF) | ((val as u16) << 8);
                }
            }
            0x22 => {
                if self.is_safe_unlocked() {
                    self.aux_power_adj = (self.aux_power_adj & 0xFF00) | (val as u16);
                }
            }
            0x23 => {
                if self.is_safe_unlocked() {
                    self.aux_power_adj = (self.aux_power_adj & 0x00FF) | ((val as u16) << 8);
                }
            }
            0x24 => {
                if self.is_safe_unlocked() {
                    self.bat_det_ctrl = val;
                }
            }
            0x25 => {
                if self.is_safe_unlocked() {
                    self.bat_det_cfg = val;
                }
            }
            0x2C => {
                if self.is_safe_unlocked() {
                    self.int32k_tune = (self.int32k_tune & 0xFF00) | (val as u16);
                }
            }
            0x2D => {
                if self.is_safe_unlocked() {
                    self.int32k_tune = (self.int32k_tune & 0x00FF) | ((val as u16) << 8);
                }
            }
            0x2E => {
                if self.is_safe_unlocked() {
                    self.xt32k_tune = val;
                }
            }
            0x2F => {
                if self.is_safe_unlocked() {
                    self.ck32k_config = val & 0x7F;
                }
            }
            0x40 => match val {
                0x57 => {
                    self.safe_stage = 1;
                }
                0xA8 if self.safe_stage == 1 => {
                    self.safe_stage = 2;
                    self.safe_timer = 64; // generous window (hardware is ~16 sys cycles)
                }
                _ => {
                    self.safe_stage = 0;
                    self.safe_timer = 0;
                }
            },
            0x43 => {
                self.wdog_count = val;
                self.rst_wdog_ctrl &= !0x10;
            }
            0x44 => {
                if self.is_safe_unlocked() {
                    self.glob_rom_cfg = val;
                }
            }
            0x46 => {
                if self.is_safe_unlocked() {
                    if (val & 0x01) != 0 {
                        self.sw_reset_requested = true;
                    }
                    let mut next = val & !0x01;
                    // Bit 4 is RW1C
                    if (val & 0x10) != 0 {
                        next &= !0x10;
                    } else {
                        next |= self.rst_wdog_ctrl & 0x10;
                    }
                    self.rst_wdog_ctrl = next;
                } else if (val & 0x10) != 0 {
                    self.rst_wdog_ctrl &= !0x10;
                }
            }
            0x47 => self.glob_reset_keep = val,
            0x48..=0x4B => {
                if self.is_safe_unlocked() {
                    let shift = (offset & 3) * 8;
                    self.misc_ctrl = (self.misc_ctrl & !(0xFF << shift)) | ((val as u32) << shift);
                }
            }
            0x4E => {
                if self.is_safe_unlocked() {
                    self.xt32m_tune = val;
                }
            }
            0x50 | 0x51 => {
                if offset == 0x51 {
                    if (val & 0x40) != 0 {
                        self.osc_cal_ov_cnt = 0;
                    }
                    if (val & 0x80) != 0 {
                        self.osc_cal_cnt &= !0x8000;
                    }
                }
            }
            0x53 => {
                if self.is_safe_unlocked() {
                    let prev_en = (self.osc_cal_ctrl & 0x20) != 0;
                    let now_en = (val & 0x20) != 0;
                    if now_en && !prev_en {
                        self.osc_cal_ctrl = val & !0x08; // clear HALT on fresh start
                        self.osc_cal_active_cycles = 0;
                    } else if !now_en {
                        self.osc_cal_ctrl = val & !0x08;
                        self.osc_cal_active_cycles = 0;
                    } else {
                        self.osc_cal_ctrl = (val & !0x08) | (self.osc_cal_ctrl & 0x08);
                    }
                }
            }
            _ => {}
        }
    }

    pub fn write_u32(&mut self, offset: u32, val: u32) {
        match offset {
            0x00 => {
                // R32_IWDG_KR
                match val & 0xFFFF {
                    0x5555 => self.iwdg_unprotected = true,
                    0xAAAA => {
                        self.iwdg_counter = (self.iwdg_cfg & 0xFFF) as u16;
                        self.iwdg_cfg = (self.iwdg_cfg & !(0xFFF << 16)) | ((self.iwdg_counter as u32) << 16);
                    }
                    0xCCCC => {
                        self.iwdg_enabled = true;
                    }
                    _ => {}
                }
            }
            0x04 => {
                // R32_IWDG_CFG
                if self.iwdg_unprotected {
                    self.iwdg_cfg = val;
                    self.iwdg_unprotected = false;
                }
            }
            _ => {
                let bytes = val.to_le_bytes();
                for i in 0..4 {
                    self.write_u8(offset + i as u32, bytes[i]);
                }
            }
        }
    }
}

#[derive(Debug, Clone)]
enum PendingFlashOp {
    Program {
        writes: Vec<(usize, u8)>,
        total_ns: u64,
        rem_ns: u64,
        cut_pct: Option<u8>,
    },
    Erase {
        base: usize,
        len: usize,
        total_ns: u64,
        rem_ns: u64,
        cut_pct: Option<u8>,
    },
}

/// SPI-based Flash ROM controller (`0x4000_1800..0x4000_1807`) used by `libISP585.a` (`FLASH_EEPROM_CMD`).
#[derive(Debug, Clone)]
pub struct FlashCtrl {
    pub r32_data: u32,
    pub r8_data: u8,
    pub sck: u8,
    pub ctrl: u8,
    pub cfg: u8,

    /// Simulated Page Program duration in ns (default 50 us; set 0 for instant completion).
    pub program_time_ns: u64,
    /// Simulated Page/Sector/Block Erase duration in ns (default 200 us; set 0 for instant completion).
    pub erase_time_ns: u64,
    /// Number of write/erase operations interrupted by power loss.
    pub interrupted_ops: u32,
    /// Fault injection: cut MCU power mid-way through the next started Flash program/erase at this % (0..99).
    pub power_cut_next_op_pct: Option<u8>,
    /// Set by `step()` when a scheduled `power_cut_next_op_pct` trip point is reached.
    pub power_cut_triggered: bool,
    /// Monotonically incremented whenever flash contents are written/erased by FlashCtrl.
    pub write_epoch: u64,

    // Internal SPI flash transaction state
    cs_active: bool,
    write_enable: bool,
    cmd: u8,
    phase_bytes: u32,
    addr: u32,
    staged_writes: Vec<(usize, u8)>,
    staged_erase: Option<(usize, usize)>,
    pending_op: Option<PendingFlashOp>,
}

impl Default for FlashCtrl {
    fn default() -> Self {
        Self::new()
    }
}

impl FlashCtrl {
    pub fn new() -> Self {
        Self {
            r32_data: 0,
            r8_data: 0,
            sck: 0,
            ctrl: 0,
            cfg: 0x02,
            program_time_ns: 50_000,
            erase_time_ns: 200_000,
            interrupted_ops: 0,
            power_cut_next_op_pct: None,
            power_cut_triggered: false,
            write_epoch: 0,
            cs_active: false,
            write_enable: false,
            cmd: 0,
            phase_bytes: 0,
            addr: 0,
            staged_writes: Vec::new(),
            staged_erase: None,
            pending_op: None,
        }
    }

    pub fn has_pending_op(&self) -> bool {
        self.pending_op.is_some()
    }

    pub fn remaining_ns(&self) -> Option<u64> {
        let op = self.pending_op.as_ref()?;
        let (total_ns, rem_ns, cut_pct) = match op {
            PendingFlashOp::Program {
                total_ns,
                rem_ns,
                cut_pct,
                ..
            } => (*total_ns, *rem_ns, *cut_pct),
            PendingFlashOp::Erase {
                total_ns,
                rem_ns,
                cut_pct,
                ..
            } => (*total_ns, *rem_ns, *cut_pct),
        };
        if let Some(pct) = cut_pct {
            let target_elapsed = ((total_ns as u128) * (pct.min(99) as u128)).div_ceil(100) as u64;
            let elapsed = total_ns.saturating_sub(rem_ns);
            Some(target_elapsed.saturating_sub(elapsed).max(1))
        } else {
            Some(rem_ns.max(1))
        }
    }

    pub fn step(&mut self, dt_ns: u64, flash: &mut [u8]) {
        let Some(op) = self.pending_op.as_mut() else {
            return;
        };
        let (total_ns, rem_ns, cut_pct) = match op {
            PendingFlashOp::Program {
                total_ns,
                rem_ns,
                cut_pct,
                ..
            } => (*total_ns, rem_ns, *cut_pct),
            PendingFlashOp::Erase {
                total_ns,
                rem_ns,
                cut_pct,
                ..
            } => (*total_ns, rem_ns, *cut_pct),
        };

        *rem_ns = rem_ns.saturating_sub(dt_ns);
        if let Some(pct) = cut_pct {
            let elapsed = total_ns.saturating_sub(*rem_ns);
            if (elapsed as u128) * 100 >= (total_ns as u128) * (pct.min(99) as u128) {
                self.power_cut_triggered = true;
                return;
            }
        }
        if *rem_ns == 0 {
            if let Some(finished) = self.pending_op.take() {
                match finished {
                    PendingFlashOp::Program { writes, .. } => {
                        for (idx, b) in writes {
                            if idx < flash.len() {
                                flash[idx] &= b;
                            }
                        }
                    }
                    PendingFlashOp::Erase { base, len, .. } => {
                        if base + len <= flash.len() {
                            flash[base..base + len].fill(0xFF);
                        }
                    }
                }
                self.write_enable = false;
                self.write_epoch = self.write_epoch.wrapping_add(1);
            }
        }
    }

    pub fn abort_power_loss(&mut self, flash: &mut [u8]) {
        if let Some(op) = self.pending_op.take() {
            self.interrupted_ops += 1;
            self.write_epoch = self.write_epoch.wrapping_add(1);
            match op {
                PendingFlashOp::Program {
                    writes,
                    total_ns,
                    rem_ns,
                    ..
                } => {
                    if !writes.is_empty() {
                        let elapsed = total_ns.saturating_sub(rem_ns);
                        let mut done = ((writes.len() as u128 * elapsed as u128)
                            / (total_ns.max(1) as u128))
                            as usize;
                        if done >= writes.len() {
                            done = writes.len() - 1;
                        }
                        for &(idx, val) in &writes[..done] {
                            if idx < flash.len() {
                                flash[idx] &= val;
                            }
                        }
                        let (idx, val) = writes[done];
                        if idx < flash.len() {
                            let old = flash[idx];
                            let target = old & val;
                            let mut half = old & (val | 0x0F);
                            if half == old || half == target {
                                half = old & (val | 0x55);
                            }
                            flash[idx] = half;
                        }
                    }
                }
                PendingFlashOp::Erase {
                    base,
                    len,
                    total_ns,
                    rem_ns,
                    ..
                } => {
                    if len > 0 && base + len <= flash.len() {
                        let elapsed = total_ns.saturating_sub(rem_ns);
                        let mut done =
                            ((len as u128 * elapsed as u128) / (total_ns.max(1) as u128)) as usize;
                        if done >= len {
                            done = len - 1;
                        }
                        flash[base..base + done].fill(0xFF);
                        flash[base + done] |= 0xAA;
                    }
                }
            }
        }
        self.reset_bus_state();
    }

    pub fn reset_bus_state(&mut self) {
        self.r32_data = 0;
        self.r8_data = 0;
        self.sck = 0;
        self.ctrl = 0;
        self.cfg = 0x02;
        self.cs_active = false;
        self.write_enable = false;
        self.cmd = 0;
        self.phase_bytes = 0;
        self.addr = 0;
        self.staged_writes.clear();
        self.staged_erase = None;
        self.pending_op = None;
        self.power_cut_triggered = false;
    }

    pub fn read_u32(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.r32_data,
            0x04 => {
                u32::from_le_bytes([self.r8_data, self.sck, self.ctrl & 0x7F, self.cfg])
            }
            _ => 0,
        }
    }

    pub fn read_u8(&mut self, offset: u32, flash: &[u8]) -> u8 {
        match offset {
            0x00..=0x03 => (self.r32_data >> ((offset & 3) * 8)) as u8,
            0x04 => {
                // Reading R8_FLASH_DATA clocks in next byte during active SPI transfer
                if self.cs_active {
                    let b = self.spi_read_byte(flash);
                    self.r8_data = b;
                    self.r32_data = (self.r32_data >> 8) | ((b as u32) << 24);
                }
                self.r8_data
            }
            0x05 => self.sck,
            0x06 => self.ctrl & 0x7F, // bit7=SPI shift busy always 0
            0x07 => self.cfg,
            _ => 0,
        }
    }

    pub fn write_u32(&mut self, offset: u32, val: u32, flash: &mut [u8]) {
        match offset {
            0x00 => {
                self.r32_data = val;
            }
            0x04 => {
                let b = val.to_le_bytes();
                self.write_u8(0x04, b[0], flash);
                self.write_u8(0x05, b[1], flash);
                self.write_u8(0x06, b[2], flash);
                self.write_u8(0x07, b[3], flash);
            }
            _ => {}
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8, flash: &mut [u8]) {
        match offset {
            0x00..=0x03 => {
                let shift = (offset & 3) * 8;
                self.r32_data = (self.r32_data & !(0xFF << shift)) | ((val as u32) << shift);
            }
            0x04 => {
                self.r8_data = val;
                if self.cs_active {
                    self.spi_write_byte(val, flash);
                }
            }
            0x05 => self.sck = val,
            0x06 => {
                self.ctrl = val & 0x7F;
                if val == 0 {
                    // CS high / end transaction -> launch staged program/erase if any
                    if !self.staged_writes.is_empty() {
                        let writes = std::mem::take(&mut self.staged_writes);
                        let cut_pct = self.power_cut_next_op_pct.take();
                        let dur = if self.program_time_ns == 0 && cut_pct.is_none() {
                            0
                        } else {
                            self.program_time_ns.max(1_000)
                        };
                        if dur == 0 {
                            for (idx, b) in writes {
                                if idx < flash.len() {
                                    flash[idx] &= b;
                                }
                            }
                            self.write_enable = false;
                            self.write_epoch = self.write_epoch.wrapping_add(1);
                        } else {
                            self.pending_op = Some(PendingFlashOp::Program {
                                writes,
                                total_ns: dur,
                                rem_ns: dur,
                                cut_pct,
                            });
                        }
                    } else if let Some((base, len)) = self.staged_erase.take() {
                        let cut_pct = self.power_cut_next_op_pct.take();
                        let dur = if self.erase_time_ns == 0 && cut_pct.is_none() {
                            0
                        } else {
                            self.erase_time_ns.max(1_000)
                        };
                        if dur == 0 {
                            if base + len <= flash.len() {
                                flash[base..base + len].fill(0xFF);
                            }
                            self.write_enable = false;
                            self.write_epoch = self.write_epoch.wrapping_add(1);
                        } else {
                            self.pending_op = Some(PendingFlashOp::Erase {
                                base,
                                len,
                                total_ns: dur,
                                rem_ns: dur,
                                cut_pct,
                            });
                        }
                    }
                    self.cs_active = false;
                    self.cmd = 0;
                    self.phase_bytes = 0;
                } else if (val & 0x05) == 0x05 && val != 0x15 {
                    // CS low / start transaction
                    self.cs_active = true;
                    self.cmd = 0;
                    self.phase_bytes = 0;
                    self.addr = 0;
                    self.staged_writes.clear();
                    self.staged_erase = None;
                } else if val == 0x15 && self.cs_active {
                    // Shift out LSB of r32_data
                    let byte = (self.r32_data & 0xFF) as u8;
                    self.r32_data >>= 8;
                    self.spi_write_byte(byte, flash);
                }
            }
            0x07 => self.cfg = val,
            _ => {}
        }
    }

    fn spi_write_byte(&mut self, val: u8, flash: &mut [u8]) {
        if self.phase_bytes == 0 {
            self.cmd = val;
            self.phase_bytes = 1;
            self.addr = 0;
            if self.pending_op.is_none() {
                if val == 0x06 {
                    self.write_enable = true;
                } else if val == 0x04 {
                    self.write_enable = false;
                }
            }
            return;
        }

        match self.cmd {
            // Fast Read (0x0B) / Read Unique ID (0x4B): 3 addr bytes + 2 dummy bytes
            0x0B | 0x4B => {
                if (1..=3).contains(&self.phase_bytes) {
                    self.addr = (self.addr << 8) | (val as u32);
                }
                self.phase_bytes += 1;
            }
            // Page Program (0x02): 3 addr bytes then staged data bytes
            0x02 => {
                if (1..=3).contains(&self.phase_bytes) {
                    self.addr = (self.addr << 8) | (val as u32);
                } else if self.write_enable && self.pending_op.is_none() && !flash.is_empty() {
                    let idx = (self.addr as usize) & (flash.len() - 1);
                    self.staged_writes.push((idx, val));
                    self.addr = (self.addr & !0xFF) | ((self.addr + 1) & 0xFF);
                }
                self.phase_bytes += 1;
            }
            // Sector/Block Erase (0x20=4K, 0x81=256B page erase, 0x52=32K, 0xD8=64K)
            0x20 | 0x81 | 0x52 | 0xD8 => {
                if (1..=3).contains(&self.phase_bytes) {
                    self.addr = (self.addr << 8) | (val as u32);
                    if self.phase_bytes == 3 && self.write_enable && self.pending_op.is_none() {
                        let size = match self.cmd {
                            0x81 => 256,
                            0x20 => 4096,
                            0x52 => 32768,
                            _ => 65536,
                        };
                        let base = (self.addr as usize) & !(size - 1);
                        if base + size <= flash.len() {
                            self.staged_erase = Some((base, size));
                        }
                    }
                }
                self.phase_bytes += 1;
            }
            _ => {
                self.phase_bytes += 1;
            }
        }
    }

    fn spi_read_byte(&mut self, flash: &[u8]) -> u8 {
        match self.cmd {
            0x05 => {
                // Read Status Register: bit0=WIP, bit1=WEL
                let wip = if self.pending_op.is_some() { 0x01 } else { 0x00 };
                let wel = if self.write_enable { 0x02 } else { 0x00 };
                wip | wel
            }
            0x0B => {
                let idx = (self.addr as usize) % flash.len();
                self.addr = self.addr.wrapping_add(1);
                flash[idx]
            }
            0x4B => {
                // Unique 64-bit ID bytes from 0x7F018
                let idx = (0x7F018usize + ((self.phase_bytes as usize) & 7)) % flash.len();
                self.phase_bytes += 1;
                flash[idx]
            }
            _ => 0xFF,
        }
    }
}
