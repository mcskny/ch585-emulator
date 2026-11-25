use std::collections::HashMap;

use crate::ble_stub::{BleBridge, BleScanResult};
use crate::bus::Bus;
use crate::calibration::{AdcCalibrationModel, CalibrationConfig, TimingModel};
use crate::cpu::Cpu;
use crate::elf::{ElfError, ElfImage};
use crate::ext::{
    compute_pin_voltage_mv, ExtHost, ExternalDevice, InputReadKind, InputReadMask, PinAnalogModel,
    PinChangeEvent, PinContext,
};
use crate::logic::LogicAnalyzer;
use crate::peripherals::adc::Adc;
use crate::peripherals::gpio::{ExtPinDrive, ExtPull, Gpio, PinElectricState};
use crate::peripherals::irq;
use crate::peripherals::pwm::PwmX;
use crate::peripherals::rtc::Rtc;
use crate::peripherals::spi::Spi;
use crate::peripherals::sys::{
    CurrentTable, FlashCtrl, PeriphTimeNs, PowerMode, PowerReport, SysCtrl, RST_FLAG_MR,
    RST_FLAG_RPOR, RST_FLAG_SW, RST_FLAG_WTR,
};
use crate::peripherals::tmr::Timer;
use crate::peripherals::uart::Uart;
use crate::peripherals::usbhs::UsbHs;
use crate::peripherals::{PinKind, QFN48_PINS};
use crate::pfic::{Pfic, SysTick};
use crate::profiler::{Profiler, ProfilerReport};
use crate::vcd::{export_vcd, InternalTraceSample, TraceBuffer, VcdConfig};

#[derive(Debug, Clone)]
pub struct PinSnapshot {
    pub pin_num: u8,
    pub name: &'static str,
    pub alt_funcs: &'static str,
    pub electric: PinElectricState,
    pub is_output: bool,
    pub pull_up: bool,
    pub pull_down: bool,
    pub adc_ch: Option<u8>,
    pub analog_mv: Option<i32>,
}

pub struct SysBus {
    pub flash: Vec<u8>,
    pub sram: Vec<u8>,
    pub pfic: Pfic,
    pub systick: SysTick,
    pub sys: SysCtrl,
    pub flash_ctrl: FlashCtrl,
    pub gpio: Gpio,
    pub rtc: Rtc,
    pub timers: [Timer; 4],
    pub adc: Adc,
    pub uarts: [Uart; 4],
    pub spis: [Spi; 2],
    pub pwm: PwmX,
    pub usbhs: UsbHs,
    pub ble: BleBridge,

    pub vdd_mv: i32,
    pub powered: bool,
    pub bor_threshold_mv: i32,
    pub bor_hysteresis_mv: i32,
    pub bor_active: bool,
    pub power_mode: PowerMode,
    pub time_ns_by_mode: [u64; 5],
    pub unpowered_ns: u64,
    pub sleep_skipped_ns: u64,
    pub current_table: CurrentTable,
    pub periph_time_ns: PeriphTimeNs,
    pub total_cycles: u64,
    pub total_time_ns: u64,
    time_ns_rem: u64,
    pub clk_32k_hz: u32,
    pub logic: LogicAnalyzer,
    /// Unhandled MMIO address read/write tracking for diagnostics
    pub unknown_mmio: Vec<u32>,
    /// Incremented whenever flash image contents change (invalidates CPU decode cache)
    pub flash_version: u64,
    last_flash_epoch: u64,
    last_gpio_sync_version: u64,
    last_gpio_pa: u32,
    last_gpio_pb: u32,
    pub pending_periph_cycles: u32,
    pub mmio_sync_requested: bool,
    pub ext_input_read_mask: u8,
    pub pending_input_read_sync: Option<InputReadKind>,
    pub skip_input_sync_once: bool,
    /// Two-level SRAM SMC tracking: 1KB page bitmap + exact 2B halfword bitmap on cold page hit
    pub sram_code_pages: [u64; 2],
    sram_code_halfwords: Vec<u64>,
    /// Optional cycle-level timing calibration model (default all zeros = ideal 1 cycle/insn)
    pub timing: TimingModel,
    pub dma_stall_cycles: u32,
    pub last_dma_mask: u8,
    in_dma_stall: bool,
    pub trace: TraceBuffer,
}

impl Default for SysBus {
    fn default() -> Self {
        Self::new()
    }
}

impl SysBus {
    pub const FLASH_SIZE: usize = 512 * 1024;
    pub const SRAM_SIZE: usize = 128 * 1024;

    pub fn new() -> Self {
        let mut bus = Self {
            flash: vec![0xFF; Self::FLASH_SIZE],
            sram: vec![0; Self::SRAM_SIZE],
            pfic: Pfic::new(),
            systick: SysTick::new(),
            sys: SysCtrl::new(),
            flash_ctrl: FlashCtrl::new(),
            gpio: Gpio::new(),
            rtc: Rtc::new(),
            timers: [
                Timer::new(0, irq::TMR0),
                Timer::new(1, irq::TMR1),
                Timer::new(2, irq::TMR2),
                Timer::new(3, irq::TMR3),
            ],
            adc: Adc::new(),
            uarts: [
                Uart::new(0, irq::UART0),
                Uart::new(1, irq::UART1),
                Uart::new(2, irq::UART2),
                Uart::new(3, irq::UART3),
            ],
            spis: [
                Spi::new(0, Some(irq::SPI0)),
                Spi::new(1, None),
            ],
            pwm: PwmX::new(),
            usbhs: UsbHs::new(),
            ble: BleBridge::new(),
            vdd_mv: 3300,
            powered: true,
            bor_threshold_mv: 1750,
            bor_hysteresis_mv: 50,
            bor_active: false,
            power_mode: PowerMode::Active,
            time_ns_by_mode: [0; 5],
            unpowered_ns: 0,
            sleep_skipped_ns: 0,
            current_table: CurrentTable::default(),
            periph_time_ns: PeriphTimeNs::default(),
            total_cycles: 0,
            total_time_ns: 0,
            time_ns_rem: 0,
            clk_32k_hz: 32_000,
            logic: LogicAnalyzer::default(),
            unknown_mmio: Vec::new(),
            flash_version: 1,
            last_flash_epoch: 0,
            last_gpio_sync_version: 0,
            last_gpio_pa: 0,
            last_gpio_pb: 0,
            pending_periph_cycles: 0,
            mmio_sync_requested: false,
            ext_input_read_mask: 0,
            pending_input_read_sync: None,
            skip_input_sync_once: false,
            sram_code_pages: [0, 0],
            sram_code_halfwords: vec![0u64; Self::SRAM_SIZE / 128],
            timing: TimingModel::default(),
            dma_stall_cycles: 0,
            last_dma_mask: 0,
            in_dma_stall: false,
            trace: TraceBuffer::default(),
        };
        bus.init_rom_cfg();
        bus
    }

    pub fn init_rom_cfg(&mut self) {
        let write_u32 = |flash: &mut [u8], addr: u32, val: u32| {
            let idx = addr as usize;
            if idx + 4 <= flash.len() {
                flash[idx..idx + 4].copy_from_slice(&val.to_le_bytes());
            }
        };
        write_u32(&mut self.flash, 0x7F00C, 0x0000_0000); // ROM_CFG_ADR_HW
        write_u32(&mut self.flash, 0x7F010, 0x0000_0100); // ROM_CFG_VERISON
        write_u32(&mut self.flash, 0x7F014, (25 << 16) | 2048); // ROM_CFG_TMP_25C
        // MAC: 58:50:00:12:34:56
        self.flash[0x7F018..0x7F020].copy_from_slice(&[0x58, 0x50, 0x00, 0x12, 0x34, 0x56, 0x00, 0x00]);
        write_u32(&mut self.flash, 0x7F024, 0x0000_0093); // ROM_CFG_CHIP_ID = ID_CH585
        self.flash[0x7DFF8..0x7E000].fill(0);
        write_u32(&mut self.flash, 0x7EFFC, 0xA500_005A);
        self.flash_version = self.flash_version.wrapping_add(1);
    }

    fn record_unknown_mmio(&mut self, addr: u32) {
        if self.unknown_mmio.len() < 128 && !self.unknown_mmio.contains(&addr) {
            self.unknown_mmio.push(addr);
        }
    }

    fn sync_peripheral_pins(&mut self) {
        let mut pa_mask = 0u16;
        let mut pa_val = 0u16;
        let mut pb_mask = 0u32;
        let mut pb_val = 0u32;

        // PWMX outputs PWM4..PWM11 (bit 0..7 in pwm.out_en)
        let remap_pwmx = (self.gpio.pin_alternate & 0x0400) != 0;
        for ch in 0..8 {
            if (self.pwm.out_en & (1 << ch)) == 0 {
                continue;
            }
            let out = self.pwm.outputs[ch];
            match (ch, remap_pwmx) {
                (0, false) => { pa_mask |= 1 << 12; if out { pa_val |= 1 << 12; } } // PWM4 -> PA12
                (0, true)  => { pa_mask |= 1 << 6;  if out { pa_val |= 1 << 6;  } } // PWM4_ -> PA6
                (1, false) => { pa_mask |= 1 << 13; if out { pa_val |= 1 << 13; } } // PWM5 -> PA13
                (1, true)  => { pa_mask |= 1 << 7;  if out { pa_val |= 1 << 7;  } } // PWM5_ -> PA7
                (2, _)     => { pb_mask |= 1 << 0;  if out { pb_val |= 1 << 0;  } } // PWM6 -> PB0
                (3, false) => { pb_mask |= 1 << 4;  if out { pb_val |= 1 << 4;  } } // PWM7 -> PB4
                (3, true)  => { pb_mask |= 1 << 1;  if out { pb_val |= 1 << 1;  } } // PWM7_ -> PB1
                (4, false) => { pb_mask |= 1 << 6;  if out { pb_val |= 1 << 6;  } } // PWM8 -> PB6
                (4, true)  => { pb_mask |= 1 << 2;  if out { pb_val |= 1 << 2;  } } // PWM8_ -> PB2
                (5, false) => { pb_mask |= 1 << 7;  if out { pb_val |= 1 << 7;  } } // PWM9 -> PB7
                (5, true)  => { pb_mask |= 1 << 3;  if out { pb_val |= 1 << 3;  } } // PWM9_ -> PB3
                (6, _)     => { pb_mask |= 1 << 14; if out { pb_val |= 1 << 14; } } // PWM10 -> PB14
                (7, _)     => { pb_mask |= 1 << 23; if out { pb_val |= 1 << 23; } } // PWM11 -> PB23
                _ => {}
            }
        }

        // TMR0..3 PWM outputs when RB_TMR_OUT_EN (bit3) is set and mode_in=0
        let alt = self.gpio.pin_alternate;
        if (self.timers[0].ctrl_mod & 0x09) == 0x08 {
            let out = self.timers[0].pwm_out;
            if (alt & 0x01) == 0 { pa_mask |= 1 << 9; if out { pa_val |= 1 << 9; } }
            else { pb_mask |= 1 << 23; if out { pb_val |= 1 << 23; } }
        }
        if (self.timers[1].ctrl_mod & 0x09) == 0x08 {
            let out = self.timers[1].pwm_out;
            if (alt & 0x02) == 0 { pa_mask |= 1 << 10; if out { pa_val |= 1 << 10; } }
            else { pb_mask |= 1 << 10; if out { pb_val |= 1 << 10; } }
        }
        if (self.timers[2].ctrl_mod & 0x09) == 0x08 {
            let out = self.timers[2].pwm_out;
            if (alt & 0x04) == 0 { pa_mask |= 1 << 11; if out { pa_val |= 1 << 11; } }
            else { pb_mask |= 1 << 11; if out { pb_val |= 1 << 11; } }
        }
        if (self.timers[3].ctrl_mod & 0x09) == 0x08 {
            let out = self.timers[3].pwm_out;
            if (alt & 0x08) == 0 { pb_mask |= 1 << 22; if out { pb_val |= 1 << 22; } }
            else { pa_mask |= 1 << 2; if out { pa_val |= 1 << 2; } }
        }

        self.gpio.set_periph_out(pa_mask, pa_val, pb_mask, pb_val);
    }

    pub fn accumulate_power_ns(&mut self, dt_ns: u64, mode: PowerMode) {
        self.time_ns_by_mode[mode as usize] += dt_ns;
        let slp0 = self.sys.slp_clk_off0;
        let slp1 = self.sys.slp_clk_off1;
        let deep = matches!(mode, PowerMode::Sleep | PowerMode::Shutdown);

        if mode == PowerMode::Active {
            if (self.sys.power_plan & 0x0200) != 0 {
                self.periph_time_ns.active_dcdc_ns += dt_ns;
            } else {
                self.periph_time_ns.active_ldo_ns += dt_ns;
            }
        }
        if !deep && (self.sys.safe_clk_ctrl & 0x02) == 0 && (self.adc.cfg & 0x01) != 0 {
            self.periph_time_ns.adc_ns += dt_ns;
        }
        if (slp1 & 0x20) == 0 && self.usbhs.device_enabled() {
            if self.usbhs.suspended {
                if !deep || (self.sys.slp_wake_ctrl & 0x02) != 0 {
                    self.periph_time_ns.usbhs_suspend_ns += dt_ns;
                }
            } else if !deep {
                self.periph_time_ns.usbhs_active_ns += dt_ns;
            }
        }
        if (slp1 & 0x80) == 0 && self.ble.radio_active() {
            self.periph_time_ns.ble_rf_ns += dt_ns;
        }
        if !deep {
            let mut uart_cnt = 0u64;
            for u in 0..4 {
                if (slp0 & (1 << (4 + u))) == 0
                    && ((self.uarts[u].ier & 0x40) != 0 || (self.uarts[u].fcr & 0x01) != 0)
                {
                    uart_cnt += 1;
                }
            }
            self.periph_time_ns.uart_unit_ns += dt_ns * uart_cnt;

            let mut spi_cnt = 0u64;
            for s in 0..2 {
                if (slp1 & (1 << s)) == 0 && (self.spis[s].ctrl_mod & 0x0E) != 0 {
                    spi_cnt += 1;
                }
            }
            self.periph_time_ns.spi_unit_ns += dt_ns * spi_cnt;

            let mut tmr_cnt = 0u64;
            for t in 0..4 {
                if (slp0 & (1 << t)) == 0 && (self.timers[t].ctrl_mod & 0x04) != 0 {
                    tmr_cnt += 1;
                }
            }
            self.periph_time_ns.tmr_unit_ns += dt_ns * tmr_cnt;

            if (slp1 & 0x04) == 0 && self.pwm.out_en != 0 {
                self.periph_time_ns.pwm_ns += dt_ns;
            }
        }

        if (self.sys.bat_det_ctrl & 0x02) != 0 {
            self.periph_time_ns.bat_mon_lp_ns += dt_ns;
        } else if (self.sys.bat_det_ctrl & 0x01) != 0 {
            self.periph_time_ns.bat_mon_ha_ns += dt_ns;
        }
    }

    pub fn power_report(&self) -> PowerReport {
        let mode_ns = self.time_ns_by_mode;
        let powered_ns: u64 = mode_ns.iter().sum();
        let total_ns = powered_ns + self.unpowered_ns;
        if total_ns == 0 {
            return PowerReport::default();
        }
        let denom = total_ns as f64;
        let ct = &self.current_table;
        let p = &self.periph_time_ns;

        let active_weighted = (p.active_dcdc_ns as f64) * ct.active_dcdc_ua
            + (p.active_ldo_ns as f64) * ct.active_ldo_ua;
        let other_modes = (mode_ns[PowerMode::Idle as usize] as f64) * ct.idle_ua
            + (mode_ns[PowerMode::Halt as usize] as f64) * ct.halt_ua
            + (mode_ns[PowerMode::Sleep as usize] as f64) * ct.sleep_ua
            + (mode_ns[PowerMode::Shutdown as usize] as f64) * ct.shutdown_ua;

        let core_avg_ua = (active_weighted + other_modes) / denom;
        let adc_avg_ua = ((p.adc_ns as f64) * ct.adc_ua) / denom;
        let usbhs_avg_ua = ((p.usbhs_active_ns as f64) * ct.usbhs_active_ua
            + (p.usbhs_suspend_ns as f64) * ct.usbhs_suspend_ua)
            / denom;
        let ble_rf_avg_ua = ((p.ble_rf_ns as f64) * ct.ble_rf_ua) / denom;
        let uart_avg_ua = ((p.uart_unit_ns as f64) * ct.uart_ua) / denom;
        let spi_avg_ua = ((p.spi_unit_ns as f64) * ct.spi_ua) / denom;
        let tmr_avg_ua = ((p.tmr_unit_ns as f64) * ct.tmr_ua) / denom;
        let pwm_avg_ua = ((p.pwm_ns as f64) * ct.pwm_ua) / denom;
        let bat_mon_avg_ua = ((p.bat_mon_ha_ns as f64) * ct.bat_mon_ha_ua
            + (p.bat_mon_lp_ns as f64) * ct.bat_mon_lp_ua)
            / denom;
        let total_avg_ua = core_avg_ua
            + adc_avg_ua
            + usbhs_avg_ua
            + ble_rf_avg_ua
            + uart_avg_ua
            + spi_avg_ua
            + tmr_avg_ua
            + pwm_avg_ua
            + bat_mon_avg_ua;

        PowerReport {
            total_time_ns: total_ns,
            unpowered_ns: self.unpowered_ns,
            mode_ns,
            core_avg_ua,
            adc_avg_ua,
            usbhs_avg_ua,
            ble_rf_avg_ua,
            uart_avg_ua,
            spi_avg_ua,
            tmr_avg_ua,
            pwm_avg_ua,
            bat_mon_avg_ua,
            total_avg_ua,
        }
    }

    pub fn estimated_avg_current_ua(&self) -> f64 {
        self.power_report().total_avg_ua
    }

    #[inline]
    pub fn record_trace_now(&mut self) {
        if !self.trace.enabled {
            return;
        }
        let usb_state = u8::from(self.usbhs.host_attached)
            | (u8::from(self.usbhs.enumerated) << 1)
            | (u8::from(self.usbhs.suspended) << 2)
            | (u8::from(self.usbhs.in_xfer_toggle) << 3)
            | (u8::from(self.usbhs.out_xfer_toggle) << 4);
        self.trace.record_if_changed(InternalTraceSample {
            time_ns: self.total_time_ns,
            active_irq: self.pfic.active_stack.last().copied().unwrap_or(0),
            irq_depth: self.pfic.active_stack.len().min(15) as u8,
            dma_mask: self.last_dma_mask,
            usb_state,
            usb_ep: self.usbhs.last_xfer_ep & 0x0F,
            power_mode: self.power_mode as u8,
        });
    }

    #[inline]
    fn sync_flash_epoch(&mut self) {
        if self.flash_ctrl.write_epoch != self.last_flash_epoch {
            self.last_flash_epoch = self.flash_ctrl.write_epoch;
            self.flash_version = self.flash_version.wrapping_add(1);
        }
    }

    #[inline]
    pub fn mark_sram_code_range(&mut self, sram_start: usize, byte_len: usize) {
        if byte_len == 0 || sram_start >= Self::SRAM_SIZE {
            return;
        }
        let end = (sram_start + byte_len - 1).min(Self::SRAM_SIZE - 1);
        let p0 = sram_start >> 10;
        let p1 = end >> 10;
        for page in p0..=p1 {
            self.sram_code_pages[page >> 6] |= 1u64 << (page & 63);
        }
        let hw0 = sram_start >> 1;
        let hw1 = end >> 1;
        for hw in hw0..=hw1 {
            self.sram_code_halfwords[hw >> 6] |= 1u64 << (hw & 63);
        }
    }

    #[inline(always)]
    pub fn on_sram_write(&mut self, sram_off: usize, access_bytes: usize) -> bool {
        let page = sram_off >> 10;
        if (self.sram_code_pages[page >> 6] & (1u64 << (page & 63))) != 0 {
            return self.on_sram_code_page_hit(sram_off, access_bytes);
        }
        false
    }

    #[cold]
    fn on_sram_code_page_hit(&mut self, sram_off: usize, access_bytes: usize) -> bool {
        let hw0 = sram_off >> 1;
        let hw1 = (sram_off + access_bytes - 1).min(Self::SRAM_SIZE - 1) >> 1;
        for hw in hw0..=hw1 {
            if (self.sram_code_halfwords[hw >> 6] & (1u64 << (hw & 63))) != 0 {
                self.sram_code_pages = [0, 0];
                self.sram_code_halfwords.fill(0);
                self.flash_version = self.flash_version.wrapping_add(1);
                return true;
            }
        }
        false
    }

    #[inline(always)]
    pub fn needs_input_read_sync(&self, addr: u32) -> Option<InputReadKind> {
        if self.ext_input_read_mask == 0 {
            return None;
        }
        match addr {
            0x4000_10C4..=0x4000_10C7
                if (self.ext_input_read_mask & InputReadMask::GPIO_B.bits()) != 0 =>
            {
                Some(InputReadKind::GpioB)
            }
            0x4000_10A4..=0x4000_10A7
                if (self.ext_input_read_mask & InputReadMask::GPIO_A.bits()) != 0 =>
            {
                Some(InputReadKind::GpioA)
            }
            _ => None,
        }
    }

    #[inline]
    pub fn flush_pending_peripherals(&mut self) {
        if self.pending_periph_cycles > 0 {
            let c = self.pending_periph_cycles;
            self.pending_periph_cycles = 0;
            let prev_gpio_ver = self.gpio.version;
            self.step_peripherals(c);
            if self.flash_ctrl.power_cut_triggered
                || self.sys.wdog_reset_requested
                || self.pfic.pending_words != 0
                || self.gpio.version != prev_gpio_ver
            {
                self.mmio_sync_requested = true;
            }
        }
    }

    pub fn step_peripherals(&mut self, cycles: u32) {
        if cycles == 0 {
            return;
        }
        self.total_cycles += cycles as u64;
        let sys_hz = self.sys.sys_clk_hz().max(32_000);
        let num = (self.time_ns_rem as u128) + (cycles as u128) * 1_000_000_000u128;
        let dt = ((num / (sys_hz as u128)) as u64).max(1);
        self.time_ns_rem = (num % (sys_hz as u128)) as u64;
        self.total_time_ns += dt;
        self.accumulate_power_ns(dt, self.power_mode);

        if self.flash_ctrl.has_pending_op() {
            self.flash_ctrl.step(dt, &mut self.flash);
            self.sync_flash_epoch();
        }
        self.sys.step(cycles, self.total_cycles, &mut self.pfic);
        if self.sys.is_bat_monitor_enabled() {
            self.sys.evaluate_bat_monitor(self.vdd_mv, &mut self.pfic);
        }

        let deep_pwr_down = matches!(self.power_mode, PowerMode::Sleep | PowerMode::Shutdown);
        let lse_stopped = self.sys.lse_failed && (self.sys.ck32k_config & 0x04) != 0;

        if !deep_pwr_down {
            self.systick.step(cycles, &mut self.pfic);
        }
        if !lse_stopped {
            self.rtc
                .step(cycles, sys_hz, self.clk_32k_hz, &mut self.pfic);
            if deep_pwr_down && (self.sys.slp_wake_ctrl & 0x08) == 0 {
                self.pfic.sync_level_irq(irq::RTC, false);
            }
        }

        let slp0 = self.sys.slp_clk_off0;
        let slp1 = self.sys.slp_clk_off1;
        if !deep_pwr_down {
            for t in 0..4 {
                if (slp0 & (1 << t)) == 0 {
                    self.timers[t].step(cycles, &mut self.pfic, &mut self.sram);
                }
            }
            if (self.sys.safe_clk_ctrl & 0x02) == 0 {
                self.adc.step(
                    cycles,
                    self.total_cycles,
                    sys_hz,
                    &self.gpio,
                    &mut self.pfic,
                    &mut self.sram,
                );
            }
            for u in 0..4 {
                if (slp0 & (1 << (4 + u))) == 0 {
                    self.uarts[u].step(cycles, &mut self.pfic);
                }
            }
            for s in 0..2 {
                if (slp1 & (1 << s)) == 0 {
                    self.spis[s].step(cycles, &mut self.pfic, &mut self.sram);
                }
            }
            if (slp1 & 0x04) == 0 {
                self.pwm.step(cycles, &mut self.pfic);
            }
        }

        if (slp1 & 0x20) == 0 && (!deep_pwr_down || (self.sys.slp_wake_ctrl & 0x02) != 0) {
            self.usbhs.step(cycles, &mut self.pfic, &mut self.sram);
        } else {
            self.usbhs.dma_active_step = false;
            if deep_pwr_down {
                self.pfic.sync_level_irq(irq::USB2_DEVICE, false);
            }
        }

        let mut dma_mask = 0u8;
        for t in 0..4 {
            if self.timers[t].dma_active_step {
                dma_mask |= 1 << t;
            }
        }
        if self.adc.dma_active_step {
            dma_mask |= 1 << 4;
        }
        for s in 0..2 {
            if self.spis[s].dma_active_step {
                dma_mask |= 1 << (5 + s);
            }
        }
        if self.usbhs.dma_active_step {
            dma_mask |= 1 << 7;
        }
        self.last_dma_mask = dma_mask;
        if !self.in_dma_stall && self.timing.dma_bus_penalty_cycles > 0 && dma_mask != 0 {
            self.dma_stall_cycles +=
                dma_mask.count_ones() * (self.timing.dma_bus_penalty_cycles as u32);
        }

        if self.pwm.out_en != 0
            || self.gpio.periph_out_pa_mask != 0
            || self.gpio.periph_out_pb_mask != 0
            || (self.timers[0].ctrl_mod & 0x09) == 0x08
            || (self.timers[1].ctrl_mod & 0x09) == 0x08
            || (self.timers[2].ctrl_mod & 0x09) == 0x08
            || (self.timers[3].ctrl_mod & 0x09) == 0x08
        {
            self.sync_peripheral_pins();
        }

        let pa = self.gpio.effective_pa_pin();
        let pb = self.gpio.effective_pb_pin();
        if self.gpio.version != self.last_gpio_sync_version
            || (pa as u32) != self.last_gpio_pa
            || pb != self.last_gpio_pb
            || self.gpio.has_level_int_enabled()
        {
            self.last_gpio_sync_version = self.gpio.version;
            self.last_gpio_pa = pa as u32;
            self.last_gpio_pb = pb;
            self.gpio.check_contention();
            self.gpio.evaluate_interrupts(&mut self.pfic);
            if deep_pwr_down && (self.sys.slp_wake_ctrl & 0x10) == 0 {
                self.pfic.sync_level_irq(irq::GPIO_A, false);
                self.pfic.sync_level_irq(irq::GPIO_B, false);
            }

            self.logic
                .record_if_changed(self.total_cycles, self.total_time_ns, pa, pb);
        }
        if self.trace.enabled {
            self.record_trace_now();
        }
    }

    pub fn sync_after_ext(&mut self) {
        let deep_pwr_down = matches!(self.power_mode, PowerMode::Sleep | PowerMode::Shutdown);
        self.gpio.check_contention();
        self.gpio.evaluate_interrupts(&mut self.pfic);
        if deep_pwr_down && (self.sys.slp_wake_ctrl & 0x10) == 0 {
            self.pfic.sync_level_irq(irq::GPIO_A, false);
            self.pfic.sync_level_irq(irq::GPIO_B, false);
        }
        let pa = self.gpio.effective_pa_pin();
        let pb = self.gpio.effective_pb_pin();
        self.last_gpio_sync_version = self.gpio.version;
        self.last_gpio_pa = pa as u32;
        self.last_gpio_pb = pb;
        self.logic
            .record_if_changed(self.total_cycles, self.total_time_ns, pa, pb);
    }

    fn mmio_read_u8(&mut self, addr: u32) -> u8 {
        self.flush_pending_peripherals();
        match addr {
            0x4000_1000..=0x4000_1013 | 0x4000_1020..=0x4000_102F | 0x4000_1040..=0x4000_1053 => {
                self.sys.read_u8(addr - 0x4000_1000, self.total_cycles)
            }
            0x4000_1030..=0x4000_103F => self.rtc.read_u8(addr - 0x4000_1000),
            0x4000_1054..=0x4000_107F => self.adc.read_u8(addr - 0x4000_1000),
            0x4000_1018..=0x4000_101F | 0x4000_1090..=0x4000_10DF => {
                self.gpio.read_u8(addr - 0x4000_1000)
            }
            0x4000_1800..=0x4000_1807 => {
                self.flash_ctrl.read_u8(addr - 0x4000_1800, &self.flash)
            }
            0x4000_2000..=0x4000_201F => self.timers[0].read_u8(addr - 0x4000_2000),
            0x4000_2400..=0x4000_241F => self.timers[1].read_u8(addr - 0x4000_2400),
            0x4000_2800..=0x4000_281F => self.timers[2].read_u8(addr - 0x4000_2800),
            0x4000_2C00..=0x4000_2C1F => self.timers[3].read_u8(addr - 0x4000_2C00),
            0x4000_3000..=0x4000_300F => self.uarts[0].read_u8(addr - 0x4000_3000),
            0x4000_3400..=0x4000_340F => self.uarts[1].read_u8(addr - 0x4000_3400),
            0x4000_3800..=0x4000_380F => self.uarts[2].read_u8(addr - 0x4000_3800),
            0x4000_3C00..=0x4000_3C0F => self.uarts[3].read_u8(addr - 0x4000_3C00),
            0x4000_4000..=0x4000_401F => self.spis[0].read_u8(addr - 0x4000_4000),
            0x4000_4400..=0x4000_441F => self.spis[1].read_u8(addr - 0x4000_4400),
            0x4000_5000..=0x4000_5017 => self.pwm.read_u8(addr - 0x4000_5000),
            0x4000_9000..=0x4000_9203 => self.usbhs.read_u8(addr - 0x4000_9000),
            0x4000_C000..=0x4000_DFFF => self.ble.read_u8(addr - 0x4000_C000),
            _ => {
                self.record_unknown_mmio(addr);
                0
            }
        }
    }

    fn mmio_write_u8(&mut self, addr: u32, val: u8) {
        self.flush_pending_peripherals();
        match addr {
            0x4000_1000..=0x4000_1013 | 0x4000_1020..=0x4000_102F | 0x4000_1040..=0x4000_1053 => {
                self.sys.write_u8(addr - 0x4000_1000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_1030..=0x4000_103F => {
                let unlocked = self.sys.is_safe_unlocked();
                self.rtc.write_u8(addr - 0x4000_1000, val, unlocked);
                self.mmio_sync_requested = true;
            }
            0x4000_1054..=0x4000_107F => {
                self.adc.write_u8(addr - 0x4000_1000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_1018..=0x4000_101F | 0x4000_1090..=0x4000_10DF => {
                self.gpio.write_u8(addr - 0x4000_1000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_1800..=0x4000_1807 => {
                self.flash_ctrl.write_u8(addr - 0x4000_1800, val, &mut self.flash);
                self.sync_flash_epoch();
                self.mmio_sync_requested = true;
            }
            0x4000_2000..=0x4000_201F => {
                self.timers[0].write_u8(addr - 0x4000_2000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_2400..=0x4000_241F => {
                self.timers[1].write_u8(addr - 0x4000_2400, val);
                self.mmio_sync_requested = true;
            }
            0x4000_2800..=0x4000_281F => {
                self.timers[2].write_u8(addr - 0x4000_2800, val);
                self.mmio_sync_requested = true;
            }
            0x4000_2C00..=0x4000_2C1F => {
                self.timers[3].write_u8(addr - 0x4000_2C00, val);
                self.mmio_sync_requested = true;
            }
            0x4000_3000..=0x4000_300F => self.uarts[0].write_u8(addr - 0x4000_3000, val),
            0x4000_3400..=0x4000_340F => self.uarts[1].write_u8(addr - 0x4000_3400, val),
            0x4000_3800..=0x4000_380F => self.uarts[2].write_u8(addr - 0x4000_3800, val),
            0x4000_3C00..=0x4000_3C0F => self.uarts[3].write_u8(addr - 0x4000_3C00, val),
            0x4000_4000..=0x4000_401F => self.spis[0].write_u8(addr - 0x4000_4000, val),
            0x4000_4400..=0x4000_441F => self.spis[1].write_u8(addr - 0x4000_4400, val),
            0x4000_5000..=0x4000_5017 => {
                self.pwm.write_u8(addr - 0x4000_5000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_9000..=0x4000_9203 => self.usbhs.write_u8(addr - 0x4000_9000, val),
            0x4000_C000..=0x4000_DFFF => self.ble.write_u8(addr - 0x4000_C000, val),
            _ => {
                self.record_unknown_mmio(addr);
            }
        }
    }

    fn mmio_read_u32(&mut self, addr: u32) -> u32 {
        self.flush_pending_peripherals();
        match addr {
            0x4000_1000..=0x4000_1007 => {
                let b0 = self.mmio_read_u8(addr) as u32;
                let b1 = self.mmio_read_u8(addr + 1) as u32;
                let b2 = self.mmio_read_u8(addr + 2) as u32;
                let b3 = self.mmio_read_u8(addr + 3) as u32;
                b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
            }
            0x4000_1054..=0x4000_107F => self.adc.read_u32(addr - 0x4000_1000),
            0x4000_1018..=0x4000_101F | 0x4000_1090..=0x4000_10DF => {
                self.gpio.read_u32(addr - 0x4000_1000)
            }
            0x4000_1800..=0x4000_1807 => self.flash_ctrl.read_u32(addr - 0x4000_1800),
            0x4000_2000..=0x4000_201F => self.timers[0].read_u32_mut(addr - 0x4000_2000),
            0x4000_2400..=0x4000_241F => self.timers[1].read_u32_mut(addr - 0x4000_2400),
            0x4000_2800..=0x4000_281F => self.timers[2].read_u32_mut(addr - 0x4000_2800),
            0x4000_2C00..=0x4000_2C1F => self.timers[3].read_u32_mut(addr - 0x4000_2C00),
            0x4000_3000..=0x4000_300F => self.uarts[0].read_u32(addr - 0x4000_3000),
            0x4000_3400..=0x4000_340F => self.uarts[1].read_u32(addr - 0x4000_3400),
            0x4000_3800..=0x4000_380F => self.uarts[2].read_u32(addr - 0x4000_3800),
            0x4000_3C00..=0x4000_3C0F => self.uarts[3].read_u32(addr - 0x4000_3C00),
            0x4000_4000..=0x4000_401F => self.spis[0].read_u32(addr - 0x4000_4000),
            0x4000_4400..=0x4000_441F => self.spis[1].read_u32(addr - 0x4000_4400),
            0x4000_5000..=0x4000_5017 => self.pwm.read_u32(addr - 0x4000_5000),
            0x4000_9000..=0x4000_9203 => self.usbhs.read_u32(addr - 0x4000_9000),
            _ => {
                let b0 = self.mmio_read_u8(addr) as u32;
                let b1 = self.mmio_read_u8(addr + 1) as u32;
                let b2 = self.mmio_read_u8(addr + 2) as u32;
                let b3 = self.mmio_read_u8(addr + 3) as u32;
                b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
            }
        }
    }

    fn mmio_write_u32(&mut self, addr: u32, val: u32) {
        self.flush_pending_peripherals();
        match addr {
            0x4000_1000..=0x4000_1013 | 0x4000_1020..=0x4000_102F | 0x4000_1040..=0x4000_1053 => {
                self.sys.write_u32(addr - 0x4000_1000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_1054..=0x4000_107F => {
                self.adc.write_u32(addr - 0x4000_1000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_1018..=0x4000_101F | 0x4000_1090..=0x4000_10DF => {
                self.gpio.write_u32(addr - 0x4000_1000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_1800..=0x4000_1807 => {
                self.flash_ctrl.write_u32(addr - 0x4000_1800, val, &mut self.flash);
                self.sync_flash_epoch();
                self.mmio_sync_requested = true;
            }
            0x4000_2000..=0x4000_201F => {
                self.timers[0].write_u32(addr - 0x4000_2000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_2400..=0x4000_241F => {
                self.timers[1].write_u32(addr - 0x4000_2400, val);
                self.mmio_sync_requested = true;
            }
            0x4000_2800..=0x4000_281F => {
                self.timers[2].write_u32(addr - 0x4000_2800, val);
                self.mmio_sync_requested = true;
            }
            0x4000_2C00..=0x4000_2C1F => {
                self.timers[3].write_u32(addr - 0x4000_2C00, val);
                self.mmio_sync_requested = true;
            }
            0x4000_3000..=0x4000_300F => self.uarts[0].write_u32(addr - 0x4000_3000, val),
            0x4000_3400..=0x4000_340F => self.uarts[1].write_u32(addr - 0x4000_3400, val),
            0x4000_3800..=0x4000_380F => self.uarts[2].write_u32(addr - 0x4000_3800, val),
            0x4000_3C00..=0x4000_3C0F => self.uarts[3].write_u32(addr - 0x4000_3C00, val),
            0x4000_4000..=0x4000_401F => self.spis[0].write_u32(addr - 0x4000_4000, val),
            0x4000_4400..=0x4000_441F => self.spis[1].write_u32(addr - 0x4000_4400, val),
            0x4000_5000..=0x4000_5017 => {
                self.pwm.write_u32(addr - 0x4000_5000, val);
                self.mmio_sync_requested = true;
            }
            0x4000_9000..=0x4000_9203 => self.usbhs.write_u32(addr - 0x4000_9000, val),
            _ => {
                let b = val.to_le_bytes();
                for i in 0..4 {
                    self.mmio_write_u8(addr + i as u32, b[i]);
                }
            }
        }
    }
}

impl Bus for SysBus {
    #[inline(always)]
    fn read_u8(&mut self, addr: u32) -> u8 {
        let sram_off = addr.wrapping_sub(0x2000_0000) as usize;
        if sram_off < Self::SRAM_SIZE {
            self.sram[sram_off]
        } else if (addr as usize) < Self::FLASH_SIZE {
            self.flash[addr as usize]
        } else if addr.wrapping_sub(0x4000_0000) < 0x1_0000 {
            self.mmio_read_u8(addr)
        } else if addr.wrapping_sub(0xE000_E000) < 0x1000 {
            self.flush_pending_peripherals();
            self.pfic.read_u8(addr - 0xE000_E000)
        } else if addr.wrapping_sub(0xE000_F000) < 0x1000 {
            self.flush_pending_peripherals();
            let w = self.systick.read_u32((addr - 0xE000_F000) & !3);
            ((w >> ((addr & 3) * 8)) & 0xFF) as u8
        } else {
            0
        }
    }

    #[inline(always)]
    fn read_u16(&mut self, addr: u32) -> u16 {
        let sram_off = addr.wrapping_sub(0x2000_0000) as usize;
        if sram_off <= Self::SRAM_SIZE - 2 {
            return u16::from_le_bytes([self.sram[sram_off], self.sram[sram_off + 1]]);
        }
        let f = addr as usize;
        if f <= Self::FLASH_SIZE - 2 {
            return u16::from_le_bytes([self.flash[f], self.flash[f + 1]]);
        }
        let b0 = self.read_u8(addr) as u16;
        let b1 = self.read_u8(addr.wrapping_add(1)) as u16;
        b0 | (b1 << 8)
    }

    #[inline(always)]
    fn read_u32(&mut self, addr: u32) -> u32 {
        if (addr & 3) == 0 {
            let sram_off = addr.wrapping_sub(0x2000_0000) as usize;
            if sram_off <= Self::SRAM_SIZE - 4 {
                return u32::from_le_bytes([
                    self.sram[sram_off],
                    self.sram[sram_off + 1],
                    self.sram[sram_off + 2],
                    self.sram[sram_off + 3],
                ]);
            }
            let f = addr as usize;
            if f <= Self::FLASH_SIZE - 4 {
                return u32::from_le_bytes([
                    self.flash[f],
                    self.flash[f + 1],
                    self.flash[f + 2],
                    self.flash[f + 3],
                ]);
            }
            if addr.wrapping_sub(0x4000_0000) < 0x1_0000 {
                return self.mmio_read_u32(addr);
            }
            if addr.wrapping_sub(0xE000_E000) < 0x1000 {
                self.flush_pending_peripherals();
                return self.pfic.read_u32(addr - 0xE000_E000);
            }
            if addr.wrapping_sub(0xE000_F000) < 0x1000 {
                self.flush_pending_peripherals();
                return self.systick.read_u32(addr - 0xE000_F000);
            }
        }
        let b0 = self.read_u8(addr) as u32;
        let b1 = self.read_u8(addr.wrapping_add(1)) as u32;
        let b2 = self.read_u8(addr.wrapping_add(2)) as u32;
        let b3 = self.read_u8(addr.wrapping_add(3)) as u32;
        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    }

    #[inline(always)]
    fn write_u8(&mut self, addr: u32, val: u8) {
        let sram_off = addr.wrapping_sub(0x2000_0000) as usize;
        if sram_off < Self::SRAM_SIZE {
            self.sram[sram_off] = val;
            self.on_sram_write(sram_off, 1);
        } else if addr.wrapping_sub(0x4000_0000) < 0x1_0000 {
            self.mmio_write_u8(addr, val);
        } else if addr.wrapping_sub(0xE000_E000) < 0x1000 {
            self.flush_pending_peripherals();
            self.pfic.write_u8(addr - 0xE000_E000, val);
            self.mmio_sync_requested = true;
        } else if addr.wrapping_sub(0xE000_F000) < 0x1000 {
            self.flush_pending_peripherals();
            let off = (addr - 0xE000_F000) & !3;
            let shift = (addr & 3) * 8;
            let mut w = self.systick.read_u32(off);
            w &= !(0xFF << shift);
            w |= (val as u32) << shift;
            self.systick.write_u32(off, w, &mut self.pfic);
            self.mmio_sync_requested = true;
        }
    }

    #[inline(always)]
    fn write_u16(&mut self, addr: u32, val: u16) {
        let sram_off = addr.wrapping_sub(0x2000_0000) as usize;
        if (addr & 1) == 0 && sram_off <= Self::SRAM_SIZE - 2 {
            self.sram[sram_off..sram_off + 2].copy_from_slice(&val.to_le_bytes());
            self.on_sram_write(sram_off, 2);
            return;
        }
        self.write_u8(addr, val as u8);
        self.write_u8(addr.wrapping_add(1), (val >> 8) as u8);
    }

    #[inline(always)]
    fn write_u32(&mut self, addr: u32, val: u32) {
        if (addr & 3) == 0 {
            let sram_off = addr.wrapping_sub(0x2000_0000) as usize;
            if sram_off <= Self::SRAM_SIZE - 4 {
                self.sram[sram_off..sram_off + 4].copy_from_slice(&val.to_le_bytes());
                self.on_sram_write(sram_off, 4);
                return;
            }
            if addr.wrapping_sub(0x4000_0000) < 0x1_0000 {
                self.mmio_write_u32(addr, val);
                return;
            }
            if addr.wrapping_sub(0xE000_E000) < 0x1000 {
                self.flush_pending_peripherals();
                self.pfic.write_u32(addr - 0xE000_E000, val);
                self.mmio_sync_requested = true;
                return;
            }
            if addr.wrapping_sub(0xE000_F000) < 0x1000 {
                self.flush_pending_peripherals();
                self.systick.write_u32(addr - 0xE000_F000, val, &mut self.pfic);
                self.mmio_sync_requested = true;
                return;
            }
        }
        self.write_u8(addr, val as u8);
        self.write_u8(addr.wrapping_add(1), (val >> 8) as u8);
        self.write_u8(addr.wrapping_add(2), (val >> 16) as u8);
        self.write_u8(addr.wrapping_add(3), (val >> 24) as u8);
    }

    fn pfic(&mut self) -> &mut Pfic {
        &mut self.pfic
    }

    fn pfic_ref(&self) -> &Pfic {
        &self.pfic
    }

    fn systick(&mut self) -> &mut SysTick {
        &mut self.systick
    }
}

pub struct Ch585Machine {
    pub cpu: Cpu,
    pub bus: SysBus,
    pub ext: ExtHost,
    pub sym_by_addr: HashMap<u32, Vec<String>>,
    pub addr_by_sym: HashMap<String, u32>,
    pub exact_periph_step: bool,
    pub periph_batch_cycles: u32,
    pub profiler: Profiler,
    loaded_elf: Option<ElfImage>,
}

impl Default for Ch585Machine {
    fn default() -> Self {
        Self::new()
    }
}

impl Ch585Machine {
    pub fn new() -> Self {
        let bus = SysBus::new();
        let ext = ExtHost::new(&bus.gpio);
        Self {
            cpu: Cpu::new(),
            bus,
            ext,
            sym_by_addr: HashMap::new(),
            addr_by_sym: HashMap::new(),
            exact_periph_step: false,
            periph_batch_cycles: 256,
            profiler: Profiler::new(),
            loaded_elf: None,
        }
    }

    pub fn load_elf(&mut self, bytes: &[u8]) -> Result<(), ElfError> {
        let elf = ElfImage::parse(bytes)?;
        self.sym_by_addr = elf.sym_by_addr.clone();
        self.addr_by_sym = elf.addr_by_sym.clone();
        self.profiler.load_symbols(&elf);
        self.loaded_elf = Some(elf);
        self.hard_reset();
        Ok(())
    }

    pub fn hard_reset(&mut self) {
        let timing = self.bus.timing;
        let current_table = self.bus.current_table;
        let adc_cal = self.bus.adc.calibration.clone();
        let adc_src_ohm = self.bus.adc.pin_source_ohm;
        let trace_enabled = self.bus.trace.enabled;
        let trace_max = self.bus.trace.max_samples;
        let logic_max = self.bus.logic.max_samples;
        self.cpu.reset();
        self.profiler.reset();
        self.bus = SysBus::new();
        self.bus.timing = timing;
        self.bus.current_table = current_table;
        self.bus.adc.calibration = adc_cal;
        self.bus.adc.pin_source_ohm = adc_src_ohm;
        self.bus.trace.enabled = trace_enabled;
        self.bus.trace.max_samples = trace_max;
        self.bus.logic.max_samples = logic_max;
        if let Some(elf) = &self.loaded_elf {
            for seg in &elf.segments {
                let p = seg.paddr as usize;
                if p + seg.data.len() <= self.bus.flash.len() {
                    self.bus.flash[p..p + seg.data.len()].copy_from_slice(&seg.data);
                } else if (0x2000_0000..0x2002_0000).contains(&seg.paddr) {
                    let s = (seg.paddr - 0x2000_0000) as usize;
                    if s + seg.data.len() <= self.bus.sram.len() {
                        self.bus.sram[s..s + seg.data.len()].copy_from_slice(&seg.data);
                    }
                }
            }
            self.cpu.pc = elf.entry;
        }
        self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
        self.bus.ble.resolve_symbols(&self.addr_by_sym);
        self.ext.reattach_all(
            self.bus.total_time_ns,
            self.bus.total_cycles,
            self.bus.vdd_mv,
            &mut self.bus.gpio,
            &mut self.bus.adc,
        );
        self.bus.ext_input_read_mask = self.ext.devices_input_read_mask().bits();
    }

    pub fn soft_reset(&mut self, reset_flag: u8) {
        self.soft_reset_retain(reset_flag, false);
    }

    pub fn soft_reset_retain(&mut self, reset_flag: u8, retain_ram: bool) {
        let ext_pa = self.bus.gpio.ext_pa;
        let ext_pb = self.bus.gpio.ext_pb;
        let ext_pull_pa = self.bus.gpio.ext_pull_pa;
        let ext_pull_pb = self.bus.gpio.ext_pull_pb;
        let adc_channels = self.bus.adc.channels;
        let adc_noise = self.bus.adc.noise_mv;
        let adc_vbat = self.bus.adc.vbat_mv;
        let adc_temp = self.bus.adc.temp_c;
        let adc_cal = self.bus.adc.calibration.clone();
        let adc_src_ohm = self.bus.adc.pin_source_ohm;
        let host_attached = self.bus.usbhs.host_attached;

        self.cpu.reset();
        if !retain_ram {
            self.bus.sram.fill(0);
            self.bus.sram_code_pages = [0, 0];
            self.bus.sram_code_halfwords.fill(0);
            self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
        }
        self.bus.pfic = Pfic::new();
        self.bus.systick = SysTick::new();
        self.bus.flash_ctrl.reset_bus_state();
        if reset_flag == RST_FLAG_RPOR {
            self.bus.sys.power_on_reset();
        } else {
            self.bus.sys.soft_reset_preserve_keep(reset_flag);
        }
        self.bus.gpio = Gpio::new();
        self.bus.gpio.ext_pa = ext_pa;
        self.bus.gpio.ext_pb = ext_pb;
        self.bus.gpio.ext_pull_pa = ext_pull_pa;
        self.bus.gpio.ext_pull_pb = ext_pull_pb;
        self.bus.gpio.rebuild_ext_masks();
        self.bus.rtc = Rtc::new();
        self.bus.timers = [
            Timer::new(0, irq::TMR0),
            Timer::new(1, irq::TMR1),
            Timer::new(2, irq::TMR2),
            Timer::new(3, irq::TMR3),
        ];
        self.bus.adc = Adc::new();
        self.bus.adc.channels = adc_channels;
        self.bus.adc.noise_mv = adc_noise;
        self.bus.adc.vbat_mv = adc_vbat;
        self.bus.adc.temp_c = adc_temp;
        self.bus.adc.calibration = adc_cal;
        self.bus.adc.pin_source_ohm = adc_src_ohm;
        self.bus.uarts = [
            Uart::new(0, irq::UART0),
            Uart::new(1, irq::UART1),
            Uart::new(2, irq::UART2),
            Uart::new(3, irq::UART3),
        ];
        self.bus.spis = [Spi::new(0, Some(irq::SPI0)), Spi::new(1, None)];
        self.bus.pwm = PwmX::new();
        self.bus.usbhs = UsbHs::new();
        self.bus.usbhs.host_attached = host_attached;
        self.bus.powered = true;
        self.bus.power_mode = PowerMode::Active;
        self.bus.pending_input_read_sync = None;
        self.bus.skip_input_sync_once = false;
        self.bus.ext_input_read_mask = self.ext.devices_input_read_mask().bits();
        self.bus.ble.on_soft_reset(&self.bus.flash);
        if let Some(elf) = &self.loaded_elf {
            self.cpu.pc = elf.entry;
        }
        self.bus.sync_after_ext();
        self.poll_ext();
    }

    pub const DATAFLASH_BASE: usize = 0x70000;
    pub const DATAFLASH_SIZE: usize = 32 * 1024;

    /// Set the factory BLE MAC address in InfoFlash (`0x7F018`) and sync BLE bridge state.
    pub fn set_mac_address(&mut self, mac: [u8; 6]) {
        self.bus.flash[0x7F018..0x7F01E].copy_from_slice(&mac);
        self.bus.ble.local_mac = mac;
        self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
    }

    /// Load a saved flash image: 32 KB images load into DataFlash (`0x70000..0x78000`);
    /// larger images load from address `0x00000` up to 512 KB.
    pub fn load_flash_image(&mut self, bytes: &[u8]) {
        if bytes.len() == Self::DATAFLASH_SIZE {
            self.bus.flash[Self::DATAFLASH_BASE..Self::DATAFLASH_BASE + Self::DATAFLASH_SIZE]
                .copy_from_slice(bytes);
        } else {
            let n = bytes.len().min(SysBus::FLASH_SIZE);
            self.bus.flash[..n].copy_from_slice(&bytes[..n]);
        }
        self.bus.ble.reload_bonds_from_flash(&self.bus.flash);
        self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
    }

    /// Overlay only the 32 KB DataFlash window (`0x70000..0x78000`) from a 32 KB or full 512 KB dump,
    /// preserving any freshly loaded ELF firmware code in `0x00000..0x70000`.
    pub fn load_flash_overlay_dataflash(&mut self, bytes: &[u8]) {
        if bytes.len() == Self::DATAFLASH_SIZE {
            self.bus.flash[Self::DATAFLASH_BASE..Self::DATAFLASH_BASE + Self::DATAFLASH_SIZE]
                .copy_from_slice(bytes);
            self.bus.ble.reload_bonds_from_flash(&self.bus.flash);
            self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
        } else if bytes.len() >= Self::DATAFLASH_BASE + Self::DATAFLASH_SIZE {
            self.bus.flash[Self::DATAFLASH_BASE..Self::DATAFLASH_BASE + Self::DATAFLASH_SIZE]
                .copy_from_slice(
                    &bytes[Self::DATAFLASH_BASE..Self::DATAFLASH_BASE + Self::DATAFLASH_SIZE],
                );
            self.bus.ble.reload_bonds_from_flash(&self.bus.flash);
            self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
        }
    }

    pub fn flash_bytes(&self) -> &[u8] {
        &self.bus.flash
    }

    pub fn dataflash_bytes(&self) -> &[u8] {
        &self.bus.flash[Self::DATAFLASH_BASE..Self::DATAFLASH_BASE + Self::DATAFLASH_SIZE]
    }

    pub fn pin_reset(&mut self) {
        if self.bus.powered {
            self.soft_reset_retain(RST_FLAG_MR, false);
        }
    }

    pub fn power_off(&mut self) {
        if !self.bus.powered {
            return;
        }
        self.bus.flash_ctrl.abort_power_loss(&mut self.bus.flash);
        self.bus.sync_flash_epoch();
        self.bus.pending_periph_cycles = 0;
        self.bus.mmio_sync_requested = false;
        self.bus.pending_input_read_sync = None;
        self.bus.skip_input_sync_once = false;
        self.bus.powered = false;
        self.bus.sram.fill(0);
        self.bus.sram_code_pages = [0, 0];
        self.bus.sram_code_halfwords.fill(0);
        self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
        self.cpu.reset();

        let ext_pa = self.bus.gpio.ext_pa;
        let ext_pb = self.bus.gpio.ext_pb;
        let ext_pull_pa = self.bus.gpio.ext_pull_pa;
        let ext_pull_pb = self.bus.gpio.ext_pull_pb;
        self.bus.gpio = Gpio::new();
        self.bus.gpio.ext_pa = ext_pa;
        self.bus.gpio.ext_pb = ext_pb;
        self.bus.gpio.ext_pull_pa = ext_pull_pa;
        self.bus.gpio.ext_pull_pb = ext_pull_pb;
        self.bus.gpio.rebuild_ext_masks();
        self.bus.sync_after_ext();
        self.poll_ext();
    }

    pub fn power_on(&mut self) {
        if self.bus.vdd_mv < self.bus.bor_threshold_mv {
            self.bus.bor_active = true;
            self.power_off();
            return;
        }
        if !self.bus.powered {
            self.bus.bor_active = false;
            self.soft_reset_retain(RST_FLAG_RPOR, false);
        }
    }

    pub fn power_cycle(&mut self) {
        self.power_off();
        self.power_on();
    }

    pub fn set_vdd_mv(&mut self, mv: i32) {
        self.bus.vdd_mv = mv;
        if self.bus.powered {
            self.bus.sys.evaluate_bat_monitor(mv, &mut self.bus.pfic);
            if mv < self.bus.bor_threshold_mv {
                self.bus.bor_active = true;
                self.power_off();
            }
        } else if self.bus.bor_active
            && mv >= self.bus.bor_threshold_mv + self.bus.bor_hysteresis_mv
        {
            self.bus.bor_active = false;
            self.power_on();
        }
    }

    fn classify_power_mode(&self) -> PowerMode {
        if !self.cpu.halted_wfi {
            PowerMode::Active
        } else if (self.bus.pfic.sctlr & (1 << 2)) == 0 {
            PowerMode::Idle
        } else if (self.bus.sys.power_plan & 0x8000) == 0 {
            PowerMode::Halt
        } else if (self.bus.sys.power_plan & 0x04) != 0 {
            PowerMode::Sleep
        } else {
            PowerMode::Shutdown
        }
    }

    pub fn enable_vcd_trace(&mut self, max_samples: usize) {
        self.bus.trace.enabled = true;
        self.bus.trace.max_samples = max_samples.max(64);
        self.bus.logic.max_samples = max_samples.max(self.bus.logic.max_samples);
        let pa = self.bus.gpio.effective_pa_pin();
        let pb = self.bus.gpio.effective_pb_pin();
        self.bus
            .logic
            .record_if_changed(self.bus.total_cycles, self.bus.total_time_ns, pa, pb);
        self.bus.record_trace_now();
    }

    pub fn export_vcd(&self, cfg: &VcdConfig) -> String {
        export_vcd(&self.bus.logic, &self.bus.trace, cfg)
    }

    pub fn enable_profiler(&mut self, enabled: bool) {
        self.profiler.enabled = enabled;
        if enabled && self.cpu.cycles == 0 {
            if let Some(elf) = &self.loaded_elf {
                self.profiler.load_symbols(elf);
            } else {
                self.profiler.reset();
            }
        }
    }

    pub fn set_stack_bounds(&mut self, top: u32, limit: u32) {
        self.profiler.set_stack_bounds(top, limit);
    }

    pub fn profiler_report(&self) -> ProfilerReport {
        self.profiler.report()
    }

    pub fn set_timing_model(&mut self, timing: TimingModel) {
        self.bus.timing = timing;
    }

    pub fn set_adc_calibration(&mut self, cal: AdcCalibrationModel) {
        self.bus.adc.calibration = cal;
    }

    pub fn set_current_table(&mut self, table: CurrentTable) {
        self.bus.current_table = table;
    }

    pub fn load_calibration_json(&mut self, json: &str) -> Result<(), String> {
        let cfg = CalibrationConfig::from_json_str(json)?;
        if let Some(t) = cfg.timing {
            self.bus.timing = t;
        }
        if let Some(a) = cfg.adc {
            self.bus.adc.calibration = a;
        }
        if let Some(c) = cfg.current {
            self.bus.current_table = c;
        }
        Ok(())
    }

    #[inline]
    fn apply_dma_stalls(&mut self) {
        let stall = self.bus.dma_stall_cycles;
        if stall > 0 {
            self.bus.dma_stall_cycles = 0;
            self.cpu.cycles += stall as u64;
            self.bus.in_dma_stall = true;
            self.bus.step_peripherals(stall);
            self.bus.in_dma_stall = false;
        }
    }

    /// Dispatch next pending BLE/RF guest callback if CPU is outside ISRs and not already in a callback.
    #[inline]
    pub fn dispatch_pending_ble_callback(&mut self) -> bool {
        if !self.bus.powered
            || !self.cpu.active_irqs.is_empty()
            || self.cpu.in_callback()
        {
            return false;
        }
        if let Some(cb) = self.bus.ble.pending_callbacks.pop_front() {
            self.cpu.enter_callback(&mut self.bus.sram, cb)
        } else {
            false
        }
    }

    pub fn ble_scan(&mut self) -> Option<BleScanResult> {
        self.bus
            .ble
            .central_scan(self.bus.total_time_ns, self.bus.total_cycles)
    }

    pub fn ble_connect(&mut self, peer_addr: [u8; 6]) -> bool {
        self.bus
            .ble
            .central_connect(peer_addr, self.bus.total_time_ns, self.bus.total_cycles)
    }

    pub fn ble_pair_and_bond(&mut self) -> bool {
        let mut dirty = false;
        let ok = self.bus.ble.central_pair_and_bond(
            &mut self.bus.flash,
            &mut dirty,
            self.bus.total_time_ns,
            self.bus.total_cycles,
        );
        if dirty {
            self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
        }
        ok
    }

    pub fn ble_enable_notifications(&mut self) -> usize {
        let mut dirty = false;
        let cnt = self.bus.ble.central_enable_notifications(
            &mut self.bus.flash,
            &self.bus.sram,
            &mut dirty,
            self.bus.total_time_ns,
            self.bus.total_cycles,
        );
        if dirty {
            self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
        }
        cnt
    }

    pub fn ble_gatt_write_uuid(&mut self, uuid16: u16, data: &[u8]) -> bool {
        self.bus.ble.central_gatt_write_uuid(
            &self.bus.flash,
            &self.bus.sram,
            uuid16,
            data,
            self.bus.total_time_ns,
            self.bus.total_cycles,
        )
    }

    pub fn ble_gatt_read_uuid(&mut self, uuid16: u16) -> Option<u32> {
        self.bus.ble.central_gatt_read_uuid(
            &self.bus.flash,
            &self.bus.sram,
            uuid16,
            self.bus.total_time_ns,
            self.bus.total_cycles,
        )
    }

    pub fn ble_update_conn_params(&mut self, interval: u16, latency: u16, timeout: u16) -> bool {
        self.bus.ble.central_update_conn_params(
            interval,
            latency,
            timeout,
            self.bus.total_time_ns,
            self.bus.total_cycles,
        )
    }

    pub fn ble_disconnect(&mut self, reason: u8) -> bool {
        self.bus
            .ble
            .central_disconnect(reason, self.bus.total_time_ns, self.bus.total_cycles)
    }

    pub fn ble_reconnect(&mut self) -> bool {
        let mut dirty = false;
        let ok = self.bus.ble.central_reconnect(
            &mut self.bus.flash,
            &self.bus.sram,
            &mut dirty,
            self.bus.total_time_ns,
            self.bus.total_cycles,
        );
        if dirty {
            self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
        }
        ok
    }

    fn notify_before_input_read_inner<'a, 'b>(
        &mut self,
        kind: InputReadKind,
        scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
    ) {
        if self.ext.notify_before_input_read_with(
            kind,
            self.bus.total_time_ns,
            self.bus.total_cycles,
            self.bus.vdd_mv,
            &mut self.bus.gpio,
            &mut self.bus.adc,
            scoped,
        ) {
            self.bus.sync_after_ext();
        }
    }

    fn poll_ext_inner<'a, 'b>(&mut self, scoped: Option<&'a mut (dyn ExternalDevice + 'b)>) {
        if self.ext.poll_force_with(
            self.bus.total_time_ns,
            self.bus.total_cycles,
            self.bus.vdd_mv,
            &mut self.bus.gpio,
            &mut self.bus.adc,
            scoped,
        ) {
            self.bus.sync_after_ext();
        }
    }

    pub fn step(&mut self) -> u32 {
        self.step_inner(None)
    }

    pub fn step_with<D: ExternalDevice>(&mut self, dev: &mut D) -> u32 {
        self.step_inner(Some(dev))
    }

    fn step_inner<'a, 'b>(&mut self, mut scoped: Option<&'a mut (dyn ExternalDevice + 'b)>) -> u32 {
        if !self.bus.powered {
            self.bus.total_time_ns += 1_000;
            self.bus.unpowered_ns += 1_000;
            if self.ext.poll_with(
                self.bus.total_time_ns,
                self.bus.total_cycles,
                self.bus.vdd_mv,
                &mut self.bus.gpio,
                &mut self.bus.adc,
                scoped,
            ) {
                self.bus.sync_after_ext();
            }
            return 1;
        }

        let scoped_iread = scoped
            .as_deref()
            .map_or(InputReadMask::NONE, |d| d.watched_input_reads());
        self.bus.ext_input_read_mask = self
            .ext
            .devices_input_read_mask()
            .union(scoped_iread)
            .bits();

        self.dispatch_pending_ble_callback();
        let pre_mode = self.classify_power_mode();
        self.bus.power_mode = pre_mode;

        if self.bus.ext_input_read_mask != 0
            && !self.bus.skip_input_sync_once
            && !(self.cpu.pc == crate::cpu::CALLBACK_RET_SENTINEL
                && !self.cpu.callback_stack.is_empty())
            && !(self.bus.ble.has_hooks() && self.bus.ble.in_hook_range(self.cpu.pc))
        {
            let pending_irq = self.bus.pfic.pick_pending_irq(self.cpu.nesting_enabled());
            let will_enter_irq =
                pending_irq.is_some_and(|(irqn, _)| irqn == 2 || self.cpu.mie());
            let will_stall_wfi = self.cpu.halted_wfi
                && pending_irq.is_none()
                && !(self.bus.pfic.event_pending && (self.bus.pfic.sctlr & (1 << 3)) != 0);
            if !will_stall_wfi && !will_enter_irq {
                let (insn, _) = self.cpu.fetch_decode_sys(&mut self.bus, self.cpu.pc);
                if let Some(addr) = Cpu::insn_load_addr(&insn, &self.cpu.regs) {
                    if let Some(kind) = self.bus.needs_input_read_sync(addr) {
                        self.notify_before_input_read_inner(kind, scoped.as_deref_mut());
                        self.bus.skip_input_sync_once = true;
                    }
                }
            }
        }

        let (cycles, queued_in_pending) = if self.profiler.enabled {
            let has_irq = self.cpu.service_interrupts(&mut self.bus);
            if has_irq {
                if let Some(&irqn) = self.cpu.active_irqs.last() {
                    self.profiler.on_irq_enter(
                        irqn,
                        self.cpu.pc,
                        self.cpu.cycles,
                        self.bus.total_time_ns,
                    );
                }
                if self.bus.timing.is_ideal() {
                    (1, false)
                } else {
                    let entry_extra = self.bus.timing.irq_entry_cycles as u32;
                    self.cpu.cycles += entry_extra as u64;
                    (1 + entry_extra, false)
                }
            } else {
                let has_ble = self.bus.ble.has_hooks();
                let c = self
                    .cpu
                    .run_profiled_batch(&mut self.bus, &mut self.profiler, 1, has_ble);
                (c, true)
            }
        } else if self.bus.ble.has_hooks() && self.bus.ble.in_hook_range(self.cpu.pc) {
            let mut flash_dirty = false;
            let stubbed = self.bus.ble.on_pc(
                &mut self.cpu,
                &mut self.bus.flash,
                &mut self.bus.sram,
                self.bus.total_time_ns,
                self.bus.total_cycles,
                &mut flash_dirty,
            );
            if flash_dirty {
                self.bus.flash_version = self.bus.flash_version.wrapping_add(1);
            }
            if stubbed {
                (1, false)
            } else if self.bus.timing.is_ideal() {
                (self.cpu.step(&mut self.bus), false)
            } else {
                (self.cpu.step_sys(&mut self.bus), false)
            }
        } else if self.bus.timing.is_ideal() {
            (self.cpu.step(&mut self.bus), false)
        } else {
            (self.cpu.step_sys(&mut self.bus), false)
        };
        self.bus.skip_input_sync_once = false;
        let total_c = if queued_in_pending {
            let p = self.bus.pending_periph_cycles;
            self.bus.pending_periph_cycles = 0;
            p
        } else {
            let p = self.bus.pending_periph_cycles.saturating_add(cycles);
            self.bus.pending_periph_cycles = 0;
            p
        };
        if (self.bus.ext_input_read_mask & InputReadMask::ADC.bits()) != 0
            && !matches!(self.bus.power_mode, PowerMode::Sleep | PowerMode::Shutdown)
            && (self.bus.sys.safe_clk_ctrl & 0x02) == 0
            && self.bus.adc.will_sample_in(total_c)
        {
            self.notify_before_input_read_inner(InputReadKind::Adc, scoped.as_deref_mut());
        }
        self.bus.step_peripherals(total_c);
        self.apply_dma_stalls();
        self.bus.mmio_sync_requested = false;
        if self.bus.flash_ctrl.power_cut_triggered {
            self.bus.flash_ctrl.power_cut_triggered = false;
            self.power_off();
            return cycles;
        }
        if self.ext.poll_with(
            self.bus.total_time_ns,
            self.bus.total_cycles,
            self.bus.vdd_mv,
            &mut self.bus.gpio,
            &mut self.bus.adc,
            scoped,
        ) {
            self.bus.sync_after_ext();
        }

        // Handle wakeup transitions from Sleep / Shutdown
        if !self.cpu.halted_wfi {
            match pre_mode {
                PowerMode::Sleep => {
                    self.bus.sys.power_plan &= !0x8000;
                    self.bus.power_mode = PowerMode::Active;
                }
                PowerMode::Shutdown => {
                    let retain_ram = (self.bus.sys.power_plan & (0x02 | 0x10)) != 0;
                    self.bus.sys.power_plan &= !0x8000;
                    self.soft_reset_retain(0x05, retain_ram);
                    return cycles;
                }
                _ => {}
            }
        }

        if self.bus.pfic.reset_requested || self.bus.sys.sw_reset_requested {
            self.soft_reset(RST_FLAG_SW);
        } else if self.bus.sys.wdog_reset_requested {
            self.soft_reset(RST_FLAG_WTR);
        }
        cycles
    }

    fn run_batch_inner<'a, 'b>(
        &mut self,
        max_cycles: u32,
        max_target_ns: Option<u64>,
        mut scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
    ) -> u32 {
        self.dispatch_pending_ble_callback();
        if !self.bus.powered
            || self.cpu.halted_wfi
            || self.exact_periph_step
            || self.periph_batch_cycles <= 1
        {
            return self.step_inner(scoped);
        }
        let scoped_iread = scoped
            .as_deref()
            .map_or(InputReadMask::NONE, |d| d.watched_input_reads());
        self.bus.ext_input_read_mask = self
            .ext
            .devices_input_read_mask()
            .union(scoped_iread)
            .bits();
        self.bus.mmio_sync_requested = false;

        let sys_hz = self.bus.sys.sys_clk_hz().max(32_000) as u128;
        let ns_to_max_cycles = |delta_ns: u64| -> u32 {
            if delta_ns == 0 {
                return 1;
            }
            let c = ((delta_ns as u128) * sys_hz) / 1_000_000_000u128;
            (c.clamp(1, u32::MAX as u128)) as u32
        };

        let mut budget = max_cycles.min(self.periph_batch_cycles).max(1);
        let cur_ns = self.bus.total_time_ns;
        if let Some(t_ns) = max_target_ns {
            budget = budget.min(ns_to_max_cycles(t_ns.saturating_sub(cur_ns)));
        }
        if let Some(ext_ns) = self.ext.next_event_ns_with(scoped.as_deref()) {
            budget = budget.min(ns_to_max_cycles(ext_ns.saturating_sub(cur_ns)));
        }
        if let Some(flash_rem) = self.bus.flash_ctrl.remaining_ns() {
            budget = budget.min(ns_to_max_cycles(flash_rem));
        }
        if (self.bus.sys.safe_clk_ctrl & 0x02) == 0 {
            if let Some(adc_c) = self.bus.adc.next_event_cycles() {
                budget = budget.min(adc_c.max(1));
            }
        }
        let slp0 = self.bus.sys.slp_clk_off0;
        let slp1 = self.bus.sys.slp_clk_off1;
        if (slp1 & 0x04) == 0 && (self.bus.pwm.out_en != 0 || (self.bus.pwm.int_ctrl & 0x01) != 0)
        {
            if let Some(pwm_c) = self.bus.pwm.next_edge_cycles() {
                budget = budget.min(pwm_c.max(1));
            }
        }
        for t in 0..4 {
            if (slp0 & (1 << t)) == 0 {
                if let Some(edge_c) = self.bus.timers[t].next_edge_cycles() {
                    budget = budget.min(edge_c.max(1));
                }
            }
        }

        self.bus.power_mode = PowerMode::Active;
        let has_irq = (self.bus.pfic.pending_words != 0
            || (self.bus.pfic.event_pending && (self.bus.pfic.sctlr & (1 << 3)) != 0))
            && {
                self.bus.flush_pending_peripherals();
                self.cpu.service_interrupts(&mut self.bus)
            };
        let ran = if has_irq {
            self.bus.skip_input_sync_once = false;
            if self.profiler.enabled {
                if let Some(&irqn) = self.cpu.active_irqs.last() {
                    self.profiler.on_irq_enter(
                        irqn,
                        self.cpu.pc,
                        self.cpu.cycles,
                        self.bus.total_time_ns,
                    );
                }
            }
            if self.bus.timing.is_ideal() {
                self.bus.pending_periph_cycles += 1;
                1
            } else {
                let entry_c = 1u32 + (self.bus.timing.irq_entry_cycles as u32);
                self.cpu.cycles += self.bus.timing.irq_entry_cycles as u64;
                self.bus.pending_periph_cycles += entry_c;
                entry_c
            }
        } else {
            let has_ble_hooks = self.bus.ble.has_hooks();
            if self.profiler.enabled {
                self.cpu
                    .run_profiled_batch(&mut self.bus, &mut self.profiler, budget, has_ble_hooks)
            } else if self.bus.timing.is_ideal() {
                self.cpu.run_fast_batch(&mut self.bus, budget, has_ble_hooks)
            } else {
                self.cpu.run_timed_batch(&mut self.bus, budget, has_ble_hooks)
            }
        };

        if (self.bus.ext_input_read_mask & InputReadMask::ADC.bits()) != 0
            && (self.bus.sys.safe_clk_ctrl & 0x02) == 0
            && self.bus.adc.will_sample_in(self.bus.pending_periph_cycles)
        {
            if self.bus.pending_periph_cycles > 1 {
                let pre = self.bus.pending_periph_cycles - 1;
                self.bus.pending_periph_cycles = 1;
                self.bus.step_peripherals(pre);
                if self.ext.poll_with(
                    self.bus.total_time_ns,
                    self.bus.total_cycles,
                    self.bus.vdd_mv,
                    &mut self.bus.gpio,
                    &mut self.bus.adc,
                    scoped.as_deref_mut(),
                ) {
                    self.bus.sync_after_ext();
                }
            }
            self.notify_before_input_read_inner(InputReadKind::Adc, scoped.as_deref_mut());
        }

        self.bus.flush_pending_peripherals();
        self.apply_dma_stalls();
        self.bus.mmio_sync_requested = false;
        if self.bus.flash_ctrl.power_cut_triggered {
            self.bus.flash_ctrl.power_cut_triggered = false;
            self.power_off();
            return ran;
        }
        if self.ext.poll_with(
            self.bus.total_time_ns,
            self.bus.total_cycles,
            self.bus.vdd_mv,
            &mut self.bus.gpio,
            &mut self.bus.adc,
            scoped.as_deref_mut(),
        ) {
            self.bus.sync_after_ext();
        }
        if let Some(kind) = self.bus.pending_input_read_sync.take() {
            self.notify_before_input_read_inner(kind, scoped.as_deref_mut());
            self.bus.skip_input_sync_once = true;
        }
        if self.bus.pfic.reset_requested || self.bus.sys.sw_reset_requested {
            self.soft_reset(RST_FLAG_SW);
        } else if self.bus.sys.wdog_reset_requested {
            self.soft_reset(RST_FLAG_WTR);
        }
        ran
    }

    pub fn run_cycles(&mut self, max_cycles: u64) -> u64 {
        self.run_cycles_inner(max_cycles, None)
    }

    pub fn run_cycles_with<D: ExternalDevice>(
        &mut self,
        max_cycles: u64,
        dev: &mut D,
    ) -> u64 {
        self.run_cycles_inner(max_cycles, Some(dev))
    }

    fn run_cycles_inner<'a, 'b>(
        &mut self,
        max_cycles: u64,
        mut scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
    ) -> u64 {
        self.poll_ext_inner(scoped.as_deref_mut());
        let mut ran = 0u64;
        while ran < max_cycles {
            let rem = (max_cycles - ran).min(u32::MAX as u64) as u32;
            ran += self.run_batch_inner(rem, None, scoped.as_deref_mut()) as u64;
        }
        ran
    }

    fn try_fast_forward_inner<'a, 'b>(
        &mut self,
        target_ns: u64,
        scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
    ) -> bool {
        if !self.bus.powered {
            let mut next_ns = target_ns;
            if let Some(ext_ns) = self.ext.next_event_ns_with(scoped.as_deref()) {
                next_ns = next_ns.min(ext_ns);
            }
            let jump_ns = next_ns.saturating_sub(self.bus.total_time_ns);
            if jump_ns < 1_000 {
                return false;
            }
            self.bus.total_time_ns += jump_ns;
            self.bus.unpowered_ns += jump_ns;
            self.bus.sleep_skipped_ns += jump_ns;
            self.poll_ext_inner(scoped);
            return true;
        }

        if !self.cpu.halted_wfi || self.cpu.in_callback() || self.bus.ble.has_pending_callbacks() {
            return false;
        }
        if self.bus.pfic.pick_pending_irq(self.cpu.nesting_enabled()).is_some()
            || (self.bus.pfic.event_pending && (self.bus.pfic.sctlr & (1 << 3)) != 0)
            || self.bus.pfic.reset_requested
            || self.bus.sys.sw_reset_requested
            || self.bus.sys.wdog_reset_requested
        {
            return false;
        }

        let mode = self.classify_power_mode();
        let deep = matches!(mode, PowerMode::Sleep | PowerMode::Shutdown);
        let lse_stopped = self.bus.sys.lse_failed && (self.bus.sys.ck32k_config & 0x04) != 0;
        let slp0 = self.bus.sys.slp_clk_off0;
        let slp1 = self.bus.sys.slp_clk_off1;

        // Keep cycle-accurate stepping while USB HS is active (unsuspended & host attached).
        if (slp1 & 0x20) == 0
            && (!deep || (self.bus.sys.slp_wake_ctrl & 0x02) != 0)
            && self.bus.usbhs.device_enabled()
            && self.bus.usbhs.host_attached
            && !self.bus.usbhs.suspended
        {
            return false;
        }

        if !deep {
            if (self.bus.sys.safe_clk_ctrl & 0x02) == 0 && self.bus.adc.is_converting() {
                return false;
            }
            if (slp1 & 0x04) == 0
                && (self.bus.pwm.out_en != 0 || (self.bus.pwm.int_ctrl & 0x01) != 0)
            {
                return false;
            }
            if (self.bus.sys.osc_cal_ctrl & 0x20) != 0 && (self.bus.sys.osc_cal_ctrl & 0x08) == 0 {
                return false;
            }
            for u in 0..4 {
                if (slp0 & (1 << (4 + u))) == 0
                    && (!self.bus.uarts[u].tx_fifo.is_empty()
                        || !self.bus.uarts[u].rx_fifo.is_empty())
                {
                    return false;
                }
            }
            for s in 0..2 {
                if (slp1 & (1 << s)) == 0
                    && (self.bus.spis[s].total_cnt > 0 || !self.bus.spis[s].fifo.is_empty())
                {
                    return false;
                }
            }
            for t in 0..4 {
                if (slp0 & (1 << t)) == 0 && self.bus.timers[t].is_pwm_or_dma_active() {
                    return false;
                }
            }
        }

        let cur_ns = self.bus.total_time_ns;
        let mut next_ns = target_ns;
        if let Some(ext_ns) = self.ext.next_event_ns_with(scoped.as_deref()) {
            next_ns = next_ns.min(ext_ns);
        }
        if let Some(rf_ns) = self.bus.ble.rf.next_event_ns() {
            next_ns = next_ns.min(rf_ns);
        }
        if let Some(flash_rem) = self.bus.flash_ctrl.remaining_ns() {
            next_ns = next_ns.min(cur_ns.saturating_add(flash_rem));
        }
        if !lse_stopped && (!deep || (self.bus.sys.slp_wake_ctrl & 0x08) != 0) {
            if let Some(rtc_rem) = self.bus.rtc.next_wakeup_ns(self.bus.clk_32k_hz) {
                next_ns = next_ns.min(cur_ns.saturating_add(rtc_rem));
            }
        }
        if let Some(wdog_rem) = self.bus.sys.next_wdog_ns(deep) {
            next_ns = next_ns.min(cur_ns.saturating_add(wdog_rem));
        }

        let sys_hz = self.bus.sys.sys_clk_hz().max(32_000) as u128;
        if !deep {
            let cyc_to_ns = |c: u64| -> u64 {
                ((c as u128) * 1_000_000_000u128).div_ceil(sys_hz).max(1) as u64
            };
            if let Some(st_cyc) = self.bus.systick.next_irq_cycles() {
                next_ns = next_ns.min(cur_ns.saturating_add(cyc_to_ns(st_cyc)));
            }
            for t in 0..4 {
                if (slp0 & (1 << t)) == 0 {
                    if let Some(tmr_cyc) = self.bus.timers[t].next_irq_cycles() {
                        next_ns = next_ns.min(cur_ns.saturating_add(cyc_to_ns(tmr_cyc)));
                    }
                }
            }
        }

        let jump_ns = next_ns.saturating_sub(cur_ns);
        if jump_ns < 1_000 {
            return false;
        }

        self.bus.power_mode = mode;
        self.bus.total_time_ns += jump_ns;
        self.bus.sleep_skipped_ns += jump_ns;
        self.bus.accumulate_power_ns(jump_ns, mode);
        self.bus.flash_ctrl.step(jump_ns, &mut self.bus.flash);
        if self.bus.flash_ctrl.power_cut_triggered {
            self.bus.flash_ctrl.power_cut_triggered = false;
            self.power_off();
            return true;
        }

        self.bus
            .sys
            .advance_wdog_ns(jump_ns, deep, &mut self.bus.pfic);
        if !lse_stopped {
            self.bus
                .rtc
                .advance_ns(jump_ns, self.bus.clk_32k_hz, &mut self.bus.pfic);
            if deep && (self.bus.sys.slp_wake_ctrl & 0x08) == 0 {
                self.bus.pfic.sync_level_irq(irq::RTC, false);
            }
        }
        if !deep {
            let cyc = (((jump_ns as u128) * sys_hz) / 1_000_000_000u128) as u64;
            self.bus.total_cycles += cyc;
            self.cpu.cycles += cyc;
            self.bus.systick.advance_cycles(cyc, &mut self.bus.pfic);
            for t in 0..4 {
                if (slp0 & (1 << t)) == 0 {
                    self.bus.timers[t].advance_cycles(cyc, &mut self.bus.pfic);
                }
            }
        }

        self.poll_ext_inner(scoped);
        if self.bus.sys.wdog_reset_requested {
            self.soft_reset(RST_FLAG_WTR);
        }
        true
    }

    pub fn run_us(&mut self, us: u64) -> u64 {
        self.run_us_inner(us, None)
    }

    pub fn run_us_with<D: ExternalDevice>(&mut self, us: u64, dev: &mut D) -> u64 {
        self.run_us_inner(us, Some(dev))
    }

    fn run_us_inner<'a, 'b>(
        &mut self,
        us: u64,
        mut scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
    ) -> u64 {
        self.poll_ext_inner(scoped.as_deref_mut());
        let start_ns = self.bus.total_time_ns;
        let target_ns = start_ns.saturating_add(us.saturating_mul(1_000));
        let est_cycles = (us as u128)
            .saturating_mul(self.bus.sys.sys_clk_hz().max(78_000_000) as u128)
            / 1_000_000;
        let max_steps = (est_cycles.min(u64::MAX as u128) as u64)
            .saturating_mul(4)
            .max(1_000_000);
        let mut steps = 0u64;
        while self.bus.total_time_ns < target_ns && steps < max_steps {
            if self.try_fast_forward_inner(target_ns, scoped.as_deref_mut()) {
                steps += 1;
                continue;
            }
            steps +=
                self.run_batch_inner(self.periph_batch_cycles, Some(target_ns), scoped.as_deref_mut())
                    as u64;
        }
        #[cfg(not(target_arch = "wasm32"))]
        if self.bus.total_time_ns < target_ns {
            let reached_us = self.bus.total_time_ns.saturating_sub(start_ns) / 1_000;
            eprintln!(
                "Warning: run_us({us}) hit safety step bound ({max_steps} steps), reached {reached_us} us"
            );
        }
        steps
    }

    #[inline]
    pub fn pin_level(&self, port: char, idx: u8) -> bool {
        self.bus.gpio.pin_level(port, idx)
    }

    pub fn set_pin_drive(&mut self, port: char, idx: u8, drive: ExtPinDrive) {
        self.bus.gpio.set_ext_drive(port, idx, drive);
    }

    pub fn set_pin_pull(&mut self, port: char, idx: u8, pull: ExtPull) {
        self.bus.gpio.set_ext_pull(port, idx, pull);
    }

    pub fn sync_ext_baseline(&mut self) {
        self.ext.sync_baseline(&self.bus.gpio);
    }

    pub fn attach_device(&mut self, dev: Box<dyn ExternalDevice>) -> usize {
        let id = self.ext.attach_device(
            dev,
            self.bus.total_time_ns,
            self.bus.total_cycles,
            self.bus.vdd_mv,
            &mut self.bus.gpio,
            &mut self.bus.adc,
        );
        self.bus.ext_input_read_mask = self.ext.devices_input_read_mask().bits();
        id
    }

    pub fn on_pin_change(
        &mut self,
        cb: impl FnMut(PinChangeEvent, &mut PinContext<'_>) + 'static,
    ) -> usize {
        self.ext.on_pin_change(cb)
    }

    pub fn schedule_at_ns(&mut self, time_ns: u64, cb: impl FnOnce(&mut PinContext<'_>) + 'static) {
        self.ext.schedule_at_ns_cb(time_ns, cb);
    }

    pub fn schedule_after_us(&mut self, delay_us: u64, cb: impl FnOnce(&mut PinContext<'_>) + 'static) {
        let target = self.bus.total_time_ns.saturating_add(delay_us.saturating_mul(1_000));
        self.ext.schedule_at_ns_cb(target, cb);
    }

    pub fn schedule_device_timer(&mut self, device_id: usize, at_ns: u64, token: u64) {
        self.ext.schedule_device_timer(device_id, at_ns, token);
    }

    pub fn set_pin_analog_model(&mut self, port: char, idx: u8, model: PinAnalogModel) {
        self.ext.analog.set(port, idx, model);
        if matches!(port, 'A' | 'a') {
            for ch in 0..14u8 {
                if Adc::channel_to_pa_idx(ch) == Some(idx as usize) {
                    self.bus.adc.pin_source_ohm[ch as usize] = model.source_ohm;
                    break;
                }
            }
        }
    }

    pub fn pin_voltage_mv(&self, port: char, idx: u8) -> i32 {
        let model = self.ext.analog.get(port, idx);
        compute_pin_voltage_mv(port, idx, self.bus.vdd_mv, &self.bus.gpio, model)
    }

    pub fn next_ext_event_ns(&self) -> Option<u64> {
        self.ext.next_event_ns()
    }

    pub fn poll_ext(&mut self) {
        self.poll_ext_inner(None);
    }

    pub fn qfn48_snapshot(&self) -> Vec<PinSnapshot> {
        let time_s = (self.bus.total_time_ns as f64) * 1e-9;
        QFN48_PINS
            .iter()
            .map(|p| {
                let (electric, is_output, pull_up, pull_down) = match p.kind {
                    PinKind::Power => (PinElectricState::High, false, false, false),
                    PinKind::AnalogOnly => (PinElectricState::Analog, false, false, false),
                    PinKind::GpioA(i) => {
                        let bit = 1u16 << i;
                        let out = (self.bus.gpio.pa_dir & bit) != 0;
                        (
                            self.bus.gpio.pa_electric(i),
                            out,
                            !out && (self.bus.gpio.pa_pu & bit) != 0,
                            !out && (self.bus.gpio.pa_pd_drv & bit) != 0,
                        )
                    }
                    PinKind::GpioB(i) => {
                        let bit = 1u32 << i;
                        let out = (self.bus.gpio.pb_dir & bit) != 0;
                        (
                            self.bus.gpio.pb_electric(i),
                            out,
                            !out && (self.bus.gpio.pb_pu & bit) != 0,
                            !out && (self.bus.gpio.pb_pd_drv & bit) != 0,
                        )
                    }
                };
                let analog_mv = p
                    .adc_ch
                    .map(|ch| self.bus.adc.sample_channel_mv(ch, time_s, &self.bus.gpio));
                PinSnapshot {
                    pin_num: p.pin_num,
                    name: p.name,
                    alt_funcs: p.alt_funcs,
                    electric,
                    is_output,
                    pull_up,
                    pull_down,
                    adc_ch: p.adc_ch,
                    analog_mv,
                }
            })
            .collect()
    }
}
