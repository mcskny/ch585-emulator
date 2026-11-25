use crate::json::{parse_json, JsonValue};
use crate::peripherals::sys::CurrentTable;

/// Cycle-level execution timing model loaded optionally from JSON calibration files.
/// Default values are all zeros, representing ideal 1-cycle-per-instruction execution.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct TimingModel {
    pub flash_fetch_wait: u8,
    pub sram_fetch_wait: u8,
    pub branch_taken_penalty: u8,
    pub jump_penalty: u8,
    pub load_extra_cycles: u8,
    pub store_extra_cycles: u8,
    pub mmio_extra_cycles: u8,
    pub irq_entry_cycles: u8,
    pub irq_exit_cycles: u8,
    pub dma_bus_penalty_cycles: u8,
}

impl TimingModel {
    #[inline]
    pub fn is_ideal(&self) -> bool {
        self.flash_fetch_wait == 0
            && self.sram_fetch_wait == 0
            && self.branch_taken_penalty == 0
            && self.jump_penalty == 0
            && self.load_extra_cycles == 0
            && self.store_extra_cycles == 0
            && self.mmio_extra_cycles == 0
            && self.irq_entry_cycles == 0
            && self.irq_exit_cycles == 0
            && self.dma_bus_penalty_cycles == 0
    }

    pub fn from_json_str(src: &str) -> Result<Self, String> {
        let val = parse_json(src)?;
        Self::from_json_value(&val)
    }

    pub fn from_json_value(val: &JsonValue) -> Result<Self, String> {
        let pairs = val
            .as_object()
            .ok_or_else(|| "TimingModel JSON must be an object".to_string())?;
        let mut t = Self::default();
        for (k, v) in pairs {
            let n = v
                .as_u8()
                .ok_or_else(|| format!("TimingModel field '{k}' must be an integer 0..=255"))?;
            match k.as_str() {
                "flash_fetch_wait" => t.flash_fetch_wait = n,
                "sram_fetch_wait" => t.sram_fetch_wait = n,
                "branch_taken_penalty" => t.branch_taken_penalty = n,
                "jump_penalty" => t.jump_penalty = n,
                "load_extra_cycles" => t.load_extra_cycles = n,
                "store_extra_cycles" => t.store_extra_cycles = n,
                "mmio_extra_cycles" => t.mmio_extra_cycles = n,
                "irq_entry_cycles" => t.irq_entry_cycles = n,
                "irq_exit_cycles" => t.irq_exit_cycles = n,
                "dma_bus_penalty_cycles" => t.dma_bus_penalty_cycles = n,
                _ => {}
            }
        }
        Ok(t)
    }
}

/// Per-channel / per-gain calibration curve, offset, and gain scaling.
#[derive(Debug, Clone, PartialEq)]
pub struct AdcChannelGainCal {
    pub offset_mv: f64,
    pub offset_counts: f64,
    pub gain_scale: f64,
    pub curve_mv_to_counts: Vec<(f64, f64)>,
}

impl Default for AdcChannelGainCal {
    fn default() -> Self {
        Self {
            offset_mv: 0.0,
            offset_counts: 0.0,
            gain_scale: 1.0,
            curve_mv_to_counts: Vec::new(),
        }
    }
}

/// Sample-and-hold RC settling error model driven by external pin source impedance.
#[derive(Debug, Clone, PartialEq)]
pub struct AdcSettlingModel {
    pub enabled: bool,
    pub sample_cap_pf: f64,
    pub internal_r_ohm: f64,
    pub base_sample_ns: f64,
    pub channel_source_ohm: [u32; 14],
}

impl Default for AdcSettlingModel {
    fn default() -> Self {
        Self {
            enabled: false,
            sample_cap_pf: 8.0,
            internal_r_ohm: 2000.0,
            base_sample_ns: 150.0,
            channel_source_ohm: [0; 14],
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct AdcCalRule {
    pub channel: Option<u8>,
    pub gain_idx: Option<u8>,
    pub cal: AdcChannelGainCal,
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct AdcCalibrationModel {
    pub settling: AdcSettlingModel,
    pub entries: Vec<AdcCalRule>,
}

impl AdcCalibrationModel {
    pub fn is_default(&self) -> bool {
        !self.settling.enabled && self.entries.is_empty()
    }

    pub fn lookup(&self, ch: u8, gain_idx: u8) -> Option<&AdcChannelGainCal> {
        let mut best_priority = 0u8;
        let mut best: Option<&AdcChannelGainCal> = None;
        for rule in &self.entries {
            let match_ch = rule.channel.is_none_or(|c| c == ch);
            let match_g = rule.gain_idx.is_none_or(|g| g == gain_idx);
            if match_ch && match_g {
                let prio = match (rule.channel, rule.gain_idx) {
                    (Some(_), Some(_)) => 4,
                    (Some(_), None) => 3,
                    (None, Some(_)) => 2,
                    (None, None) => 1,
                };
                if prio > best_priority {
                    best_priority = prio;
                    best = Some(&rule.cal);
                }
            }
        }
        best
    }

    pub fn eval_curve(points: &[(f64, f64)], mv: f64) -> f64 {
        if points.is_empty() {
            return 0.0;
        }
        if points.len() == 1 || mv <= points[0].0 {
            return points[0].1;
        }
        let last = points[points.len() - 1];
        if mv >= last.0 {
            return last.1;
        }
        for w in points.windows(2) {
            let (x0, y0) = w[0];
            let (x1, y1) = w[1];
            if mv <= x1 {
                let dx = x1 - x0;
                if dx.abs() < 1e-9 {
                    return y1;
                }
                let t = (mv - x0) / dx;
                return y0 + t * (y1 - y0);
            }
        }
        last.1
    }

    pub fn from_json_str(src: &str) -> Result<Self, String> {
        let val = parse_json(src)?;
        Self::from_json_value(&val)
    }

    pub fn from_json_value(val: &JsonValue) -> Result<Self, String> {
        let pairs = val
            .as_object()
            .ok_or_else(|| "AdcCalibration JSON must be an object".to_string())?;
        let mut model = Self::default();
        for (k, v) in pairs {
            match k.as_str() {
                "settling" => model.settling = parse_settling_json(v)?,
                "entries" => {
                    let arr = v
                        .as_array()
                        .ok_or_else(|| "AdcCalibration 'entries' must be an array".to_string())?;
                    for item in arr {
                        model.entries.push(parse_adc_rule_json(item)?);
                    }
                }
                _ => {}
            }
        }
        Ok(model)
    }
}

fn parse_settling_json(val: &JsonValue) -> Result<AdcSettlingModel, String> {
    let pairs = val
        .as_object()
        .ok_or_else(|| "ADC settling JSON must be an object".to_string())?;
    let mut s = AdcSettlingModel::default();
    for (k, v) in pairs {
        match k.as_str() {
            "enabled" => {
                s.enabled = v
                    .as_bool()
                    .ok_or_else(|| "settling.enabled must be a boolean".to_string())?;
            }
            "sample_cap_pf" => {
                s.sample_cap_pf = v
                    .as_f64()
                    .ok_or_else(|| "settling.sample_cap_pf must be a number".to_string())?;
            }
            "internal_r_ohm" => {
                s.internal_r_ohm = v
                    .as_f64()
                    .ok_or_else(|| "settling.internal_r_ohm must be a number".to_string())?;
            }
            "base_sample_ns" => {
                s.base_sample_ns = v
                    .as_f64()
                    .ok_or_else(|| "settling.base_sample_ns must be a number".to_string())?;
            }
            "channel_source_ohm" => {
                if let Some(arr) = v.as_array() {
                    for (idx, item) in arr.iter().enumerate().take(14) {
                        if let Some(ohm) = item.as_u32() {
                            s.channel_source_ohm[idx] = ohm;
                        }
                    }
                } else if let Some(obj) = v.as_object() {
                    for (ch_s, item) in obj {
                        let ch: usize = ch_s
                            .parse()
                            .map_err(|_| format!("Invalid channel key '{ch_s}'"))?;
                        if ch < 14 {
                            s.channel_source_ohm[ch] = item.as_u32().ok_or_else(|| {
                                format!("channel_source_ohm[{ch}] must be u32")
                            })?;
                        }
                    }
                }
            }
            _ => {}
        }
    }
    Ok(s)
}

pub fn gain_name_to_idx(val: &JsonValue) -> Result<u8, String> {
    if let Some(idx) = val.as_u8() {
        return if idx <= 6 {
            Ok(idx)
        } else {
            Err(format!("ADC gain index {idx} out of range 0..=6"))
        };
    }
    if let Some(s) = val.as_str() {
        let norm = s.trim().to_ascii_lowercase().replace(' ', "");
        return match norm.as_str() {
            "-12db" | "-12" => Ok(0),
            "-6db" | "-6" => Ok(1),
            "0db" | "0" => Ok(2),
            "+6db" | "6db" | "6" => Ok(3),
            "+12db" | "12db" | "12" => Ok(4),
            "+18db" | "18db" | "18" => Ok(5),
            "+24db" | "24db" | "24" => Ok(6),
            _ => Err(format!("Unknown ADC gain '{s}'")),
        };
    }
    Err("ADC gain must be an integer 0..=6 or gain string like \"-6dB\"".to_string())
}

fn parse_adc_rule_json(val: &JsonValue) -> Result<AdcCalRule, String> {
    let pairs = val
        .as_object()
        .ok_or_else(|| "ADC calibration rule must be an object".to_string())?;
    let mut channel = None;
    let mut gain_idx = None;
    let mut cal = AdcChannelGainCal::default();
    for (k, v) in pairs {
        match k.as_str() {
            "channel" => {
                if !matches!(v, JsonValue::Null) {
                    let ch = v
                        .as_u8()
                        .ok_or_else(|| "ADC cal entry 'channel' must be integer".to_string())?;
                    channel = Some(ch);
                }
            }
            "gain" | "gain_idx" => {
                if !matches!(v, JsonValue::Null) {
                    gain_idx = Some(gain_name_to_idx(v)?);
                }
            }
            "offset_mv" => {
                cal.offset_mv = v
                    .as_f64()
                    .ok_or_else(|| "ADC cal 'offset_mv' must be number".to_string())?;
            }
            "offset_counts" => {
                cal.offset_counts = v
                    .as_f64()
                    .ok_or_else(|| "ADC cal 'offset_counts' must be number".to_string())?;
            }
            "gain_scale" => {
                cal.gain_scale = v
                    .as_f64()
                    .ok_or_else(|| "ADC cal 'gain_scale' must be number".to_string())?;
            }
            "curve" | "curve_mv_to_counts" => {
                let arr = v
                    .as_array()
                    .ok_or_else(|| "ADC cal curve must be an array of [mv, counts]".to_string())?;
                for pt in arr {
                    let pair = pt
                        .as_array()
                        .ok_or_else(|| "Curve point must be [mv, counts]".to_string())?;
                    if pair.len() < 2 {
                        return Err("Curve point must have 2 numbers [mv, counts]".to_string());
                    }
                    let mv = pair[0]
                        .as_f64()
                        .ok_or_else(|| "Curve point mv must be number".to_string())?;
                    let cnt = pair[1]
                        .as_f64()
                        .ok_or_else(|| "Curve point counts must be number".to_string())?;
                    cal.curve_mv_to_counts.push((mv, cnt));
                }
                cal.curve_mv_to_counts
                    .sort_by(|a, b| a.0.total_cmp(&b.0));
            }
            _ => {}
        }
    }
    Ok(AdcCalRule {
        channel,
        gain_idx,
        cal,
    })
}

pub fn current_table_from_json_str(src: &str) -> Result<CurrentTable, String> {
    let val = parse_json(src)?;
    current_table_from_json_value(&val)
}

pub fn current_table_from_json_value(val: &JsonValue) -> Result<CurrentTable, String> {
    let pairs = val
        .as_object()
        .ok_or_else(|| "CurrentTable JSON must be an object".to_string())?;
    let mut table = CurrentTable::default();
    for (k, v) in pairs {
        let n = v
            .as_f64()
            .ok_or_else(|| format!("CurrentTable field '{k}' must be a finite number"))?;
        match k.as_str() {
            "active_dcdc_ua" => table.active_dcdc_ua = n,
            "active_ldo_ua" => table.active_ldo_ua = n,
            "idle_ua" => table.idle_ua = n,
            "halt_ua" => table.halt_ua = n,
            "sleep_ua" => table.sleep_ua = n,
            "shutdown_ua" => table.shutdown_ua = n,
            "adc_ua" => table.adc_ua = n,
            "usbhs_active_ua" => table.usbhs_active_ua = n,
            "usbhs_suspend_ua" => table.usbhs_suspend_ua = n,
            "ble_rf_ua" => table.ble_rf_ua = n,
            "uart_ua" => table.uart_ua = n,
            "spi_ua" => table.spi_ua = n,
            "tmr_ua" => table.tmr_ua = n,
            "pwm_ua" => table.pwm_ua = n,
            "bat_mon_ha_ua" => table.bat_mon_ha_ua = n,
            "bat_mon_lp_ua" => table.bat_mon_lp_ua = n,
            _ => {}
        }
    }
    Ok(table)
}

/// Top-level combined calibration file payload (`{ "timing": ..., "adc": ..., "current": ... }`).
#[derive(Debug, Clone, Default)]
pub struct CalibrationConfig {
    pub timing: Option<TimingModel>,
    pub adc: Option<AdcCalibrationModel>,
    pub current: Option<CurrentTable>,
}

impl CalibrationConfig {
    pub fn from_json_str(src: &str) -> Result<Self, String> {
        let val = parse_json(src)?;
        Self::from_json_value(&val)
    }

    pub fn from_json_value(val: &JsonValue) -> Result<Self, String> {
        let pairs = val
            .as_object()
            .ok_or_else(|| "CalibrationConfig JSON must be an object".to_string())?;
        let mut cfg = Self::default();
        for (k, v) in pairs {
            match k.as_str() {
                "timing" if !matches!(v, JsonValue::Null) => {
                    cfg.timing = Some(TimingModel::from_json_value(v)?);
                }
                "adc" if !matches!(v, JsonValue::Null) => {
                    cfg.adc = Some(AdcCalibrationModel::from_json_value(v)?);
                }
                "current" if !matches!(v, JsonValue::Null) => {
                    cfg.current = Some(current_table_from_json_value(v)?);
                }
                _ => {}
            }
        }
        Ok(cfg)
    }
}
