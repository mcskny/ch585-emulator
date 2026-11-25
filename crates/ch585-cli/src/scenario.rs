use ch585_boards::{BatteryModel, MacropadV2Handle};
use ch585_core::peripherals::adc::AdcWaveform;
use ch585_core::peripherals::gpio::{ExtPinDrive, ExtPull};
use ch585_core::soc::Ch585Machine;

#[derive(Debug, Clone)]
pub enum EventAction {
    Pin { port: char, idx: u8, drive: ExtPinDrive },
    Pull { port: char, idx: u8, pull: ExtPull },
    Adc { ch: usize, mv: i32 },
    AdcNoise { ch: Option<usize>, mv: u16 },
    UsbDetach,
    UsbAttach,
    UsbSuspend,
    UsbResume,
    UsbEnum,
    UartRx { idx: usize, bytes: Vec<u8> },
    EncSteps(i32),
    Shift165(u16),
    RailMv(i32),
    BatMv(i32),
    ResetPin,
    ResetSw,
    PowerOff,
    PowerOn,
    PowerCycle,
    VddMv(i32),
    FlashCutPct(u8),
    BleScan,
    BleConnect,
    BlePair,
    BleNotify,
    BleParamUpdate { interval: u16, latency: u16, timeout: u16 },
    BleDisconnect { reason: u8 },
    BleReconnect,
    BleGattWrite { uuid: u16, data: Vec<u8> },
}

#[derive(Debug, Clone)]
pub struct TimedEvent {
    pub at_us: u64,
    pub action: EventAction,
}

pub fn parse_pin_name(s: &str) -> Option<(char, u8)> {
    let u = s.trim().to_ascii_uppercase();
    let port = if u.starts_with("PA") {
        'A'
    } else if u.starts_with("PB") {
        'B'
    } else {
        return None;
    };
    let idx: u8 = u[2..].parse().ok()?;
    if (port == 'A' && idx < 16) || (port == 'B' && idx < 24) {
        Some((port, idx))
    } else {
        None
    }
}

pub fn parse_pin_drive_val(v: &str) -> Option<ExtPinDrive> {
    match v.trim().to_ascii_lowercase().as_str() {
        "0" | "low" => Some(ExtPinDrive::Low),
        "1" | "high" => Some(ExtPinDrive::High),
        "z" | "float" | "floating" => Some(ExtPinDrive::Floating),
        _ => None,
    }
}

pub fn parse_pull_val(v: &str) -> Option<ExtPull> {
    match v.trim().to_ascii_lowercase().as_str() {
        "up" | "pu" | "pullup" => Some(ExtPull::PullUp),
        "down" | "pd" | "pulldown" => Some(ExtPull::PullDown),
        "none" | "0" | "off" => Some(ExtPull::None),
        _ => None,
    }
}

pub fn parse_time_us(s: &str) -> Option<u64> {
    let t = s.trim().to_ascii_lowercase();
    if let Some(v) = t.strip_suffix("ms") {
        let num: f64 = v.trim().parse().ok()?;
        return Some((num.max(0.0) * 1000.0).round() as u64);
    }
    if let Some(v) = t.strip_suffix("us") {
        let num: f64 = v.trim().parse().ok()?;
        return Some(num.max(0.0).round() as u64);
    }
    if let Some(v) = t.strip_suffix('s') {
        let num: f64 = v.trim().parse().ok()?;
        return Some((num.max(0.0) * 1_000_000.0).round() as u64);
    }
    t.parse::<u64>().ok().map(|ms| ms * 1000)
}

pub fn parse_action(kind: &str, spec: &str) -> Result<EventAction, String> {
    match kind.trim().to_ascii_lowercase().as_str() {
        "pin" => {
            let (pin_s, val_s) = spec
                .split_once('=')
                .or_else(|| spec.split_once(':'))
                .ok_or_else(|| format!("Invalid pin spec '{spec}' (expected PB9=0|1|z)"))?;
            let (port, idx) = parse_pin_name(pin_s).ok_or_else(|| format!("Bad pin '{pin_s}'"))?;
            let drive = parse_pin_drive_val(val_s).ok_or_else(|| format!("Bad drive '{val_s}'"))?;
            Ok(EventAction::Pin { port, idx, drive })
        }
        "pull" => {
            let (pin_s, val_s) = spec
                .split_once('=')
                .or_else(|| spec.split_once(':'))
                .ok_or_else(|| format!("Invalid pull spec '{spec}'"))?;
            let (port, idx) = parse_pin_name(pin_s).ok_or_else(|| format!("Bad pin '{pin_s}'"))?;
            let pull = parse_pull_val(val_s).ok_or_else(|| format!("Bad pull '{val_s}'"))?;
            Ok(EventAction::Pull { port, idx, pull })
        }
        "adc" => {
            let (ch_s, mv_s) = spec
                .split_once('=')
                .or_else(|| spec.split_once(':'))
                .ok_or_else(|| format!("Invalid adc spec '{spec}' (expected <ch>=<mv>)"))?;
            let ch: usize = ch_s.trim().parse().map_err(|_| format!("Bad ch '{ch_s}'"))?;
            let mv: i32 = mv_s
                .trim()
                .trim_end_matches("mv")
                .trim_end_matches("mV")
                .parse()
                .map_err(|_| format!("Bad mV '{mv_s}'"))?;
            if ch >= 14 {
                return Err(format!("ADC channel {ch} out of range 0..13"));
            }
            Ok(EventAction::Adc { ch, mv })
        }
        "adc-noise" | "noise" => {
            if let Some((ch_s, mv_s)) = spec.split_once('=').or_else(|| spec.split_once(':')) {
                let ch: usize = ch_s.trim().parse().map_err(|_| format!("Bad ch '{ch_s}'"))?;
                let mv: u16 = parse_mv_u16(mv_s)?;
                Ok(EventAction::AdcNoise { ch: Some(ch), mv })
            } else {
                Ok(EventAction::AdcNoise {
                    ch: None,
                    mv: parse_mv_u16(spec)?,
                })
            }
        }
        "usb" => match spec.trim().to_ascii_lowercase().as_str() {
            "detach" | "unplug" => Ok(EventAction::UsbDetach),
            "attach" | "plug" => Ok(EventAction::UsbAttach),
            "suspend" => Ok(EventAction::UsbSuspend),
            "resume" => Ok(EventAction::UsbResume),
            "enum" | "enumerate" => Ok(EventAction::UsbEnum),
            other => Err(format!("Unknown usb action '{other}'")),
        },
        "uart" => {
            let (u_s, txt) = spec
                .split_once('=')
                .ok_or_else(|| format!("Invalid uart spec '{spec}' (expected <0..3>=<text>)"))?;
            let idx: usize = u_s.trim().parse().map_err(|_| format!("Bad uart idx '{u_s}'"))?;
            if idx >= 4 {
                return Err("UART index must be 0..3".into());
            }
            let unescaped = txt.replace("\\r", "\r").replace("\\n", "\n");
            Ok(EventAction::UartRx {
                idx,
                bytes: unescaped.into_bytes(),
            })
        }
        "enc" => {
            let steps: i32 = spec.trim().parse().map_err(|_| format!("Bad encoder steps '{spec}'"))?;
            Ok(EventAction::EncSteps(steps))
        }
        "sr165" => {
            let val = parse_u16_hex_or_dec(spec)?;
            Ok(EventAction::Shift165(val))
        }
        "rail" => {
            let mv: i32 = spec
                .trim()
                .trim_end_matches("mv")
                .trim_end_matches("mV")
                .parse()
                .map_err(|_| format!("Bad rail mV '{spec}'"))?;
            Ok(EventAction::RailMv(mv))
        }
        "bat" | "vbat" => {
            let mv: i32 = spec
                .trim()
                .trim_end_matches("mv")
                .trim_end_matches("mV")
                .parse()
                .map_err(|_| format!("Bad battery mV '{spec}'"))?;
            Ok(EventAction::BatMv(mv))
        }
        "reset" => match spec.trim().to_ascii_lowercase().as_str() {
            "pin" | "mr" | "manual" => Ok(EventAction::ResetPin),
            "sw" | "soft" => Ok(EventAction::ResetSw),
            other => Err(format!("Unknown reset action '{other}' (expected pin|sw)")),
        },
        "power" | "pwr" => match spec.trim().to_ascii_lowercase().as_str() {
            "off" => Ok(EventAction::PowerOff),
            "on" => Ok(EventAction::PowerOn),
            "cycle" => Ok(EventAction::PowerCycle),
            other => Err(format!("Unknown power action '{other}' (expected on|off|cycle)")),
        },
        "vdd" => {
            let mv: i32 = spec
                .trim()
                .trim_end_matches("mv")
                .trim_end_matches("mV")
                .parse()
                .map_err(|_| format!("Bad VDD mV '{spec}'"))?;
            Ok(EventAction::VddMv(mv))
        }
        "flash-cut" | "flash_cut" => {
            let pct: u8 = spec
                .trim()
                .trim_end_matches('%')
                .parse()
                .map_err(|_| format!("Bad flash-cut percentage '{spec}'"))?;
            Ok(EventAction::FlashCutPct(pct.min(99)))
        }
        "ble" => {
            let t = spec.trim();
            let low = t.to_ascii_lowercase();
            if low == "scan" {
                Ok(EventAction::BleScan)
            } else if low == "connect" {
                Ok(EventAction::BleConnect)
            } else if low == "pair" || low == "bond" {
                Ok(EventAction::BlePair)
            } else if low == "notify" || low == "cccd" {
                Ok(EventAction::BleNotify)
            } else if low == "reconnect" {
                Ok(EventAction::BleReconnect)
            } else if let Some(rest) = low.strip_prefix("disconnect") {
                let reason = if let Some(v) = rest.strip_prefix('=') {
                    parse_u16_hex_or_dec(v)? as u8
                } else {
                    0x13
                };
                Ok(EventAction::BleDisconnect { reason })
            } else if let Some(rest) = low.strip_prefix("param=") {
                let nums: Vec<&str> = rest.split(',').collect();
                let interval = nums.first().copied().unwrap_or("12").parse::<u16>().map_err(|_| format!("Bad ble param interval '{rest}'"))?;
                let latency = nums.get(1).copied().unwrap_or("0").parse::<u16>().map_err(|_| format!("Bad ble param latency '{rest}'"))?;
                let timeout = nums.get(2).copied().unwrap_or("400").parse::<u16>().map_err(|_| format!("Bad ble param timeout '{rest}'"))?;
                Ok(EventAction::BleParamUpdate { interval, latency, timeout })
            } else if let Some(rest) = t.strip_prefix("write=").or_else(|| t.strip_prefix("WRITE=")) {
                let (u_s, d_s) = rest.split_once('=').ok_or_else(|| format!("Invalid ble write '{t}' (expected write=<uuid>=<hex>)"))?;
                let uuid = parse_u16_hex_or_dec(u_s)?;
                let data = parse_hex_bytes(d_s)?;
                Ok(EventAction::BleGattWrite { uuid, data })
            } else {
                Err(format!("Unknown ble action '{spec}'"))
            }
        }
        other => Err(format!("Unknown event kind '{other}'")),
    }
}

pub fn parse_at_arg(arg: &str) -> Result<TimedEvent, String> {
    let (time_s, rest) = arg
        .split_once(':')
        .ok_or_else(|| format!("Invalid --at '{arg}' (expected <time>:<kind>:<spec>)"))?;
    let at_us = parse_time_us(time_s).ok_or_else(|| format!("Invalid time '{time_s}'"))?;
    let (kind, spec) = rest
        .split_once(':')
        .ok_or_else(|| format!("Invalid --at '{arg}' (expected <time>:<kind>:<spec>)"))?;
    let action = parse_action(kind, spec)?;
    Ok(TimedEvent { at_us, action })
}

fn parse_hex_bytes(s: &str) -> Result<Vec<u8>, String> {
    let clean: String = s.chars().filter(|c| !c.is_whitespace() && *c != ':' && *c != '-').collect();
    if (clean.len() & 1) != 0 {
        return Err(format!("Odd-length hex string '{s}'"));
    }
    let mut out = Vec::with_capacity(clean.len() / 2);
    for i in (0..clean.len()).step_by(2) {
        let b = u8::from_str_radix(&clean[i..i + 2], 16)
            .map_err(|_| format!("Bad hex byte in '{s}'"))?;
        out.push(b);
    }
    Ok(out)
}

fn parse_mv_u16(s: &str) -> Result<u16, String> {
    let cleaned = s.trim().trim_end_matches("mV").trim_end_matches("mv").trim();
    cleaned.parse::<u16>().map_err(|_| format!("Invalid mV '{s}'"))
}

fn parse_u16_hex_or_dec(s: &str) -> Result<u16, String> {
    let t = s.trim();
    if let Some(hex) = t.strip_prefix("0x").or_else(|| t.strip_prefix("0X")) {
        u16::from_str_radix(hex, 16).map_err(|_| format!("Invalid hex u16 '{s}'"))
    } else {
        t.parse::<u16>().map_err(|_| format!("Invalid decimal u16 '{s}'"))
    }
}

pub fn apply_event_action(
    mach: &mut Ch585Machine,
    board: &mut Option<MacropadV2Handle>,
    action: &EventAction,
) {
    match action {
        EventAction::Pin { port, idx, drive } => {
            mach.set_pin_drive(*port, *idx, *drive);
            mach.poll_ext();
        }
        EventAction::Pull { port, idx, pull } => {
            mach.set_pin_pull(*port, *idx, *pull);
            mach.poll_ext();
        }
        EventAction::Adc { ch, mv } => {
            if let Some(b) = board.as_ref() {
                b.set_adc_channel_mv(mach, *ch, *mv);
            } else if *ch < 14 {
                mach.bus.adc.channels[*ch] = AdcWaveform::Dc(*mv);
            }
        }
        EventAction::AdcNoise { ch, mv } => {
            if let Some(c) = ch {
                if *c < 14 {
                    mach.bus.adc.noise_mv[*c] = *mv;
                }
            } else {
                mach.bus.adc.set_all_noise_mv(*mv);
            }
        }
        EventAction::UsbDetach => mach.bus.usbhs.detach(),
        EventAction::UsbAttach => mach.bus.usbhs.attach(),
        EventAction::UsbSuspend => mach.bus.usbhs.suspend(),
        EventAction::UsbResume => mach.bus.usbhs.resume(),
        EventAction::UsbEnum => mach.bus.usbhs.trigger_enumerate(),
        EventAction::UartRx { idx, bytes } => {
            if *idx < 4 {
                mach.bus.uarts[*idx].inject_rx(bytes);
            }
        }
        EventAction::EncSteps(steps) => {
            if let Some(b) = board.as_ref() {
                b.queue_encoder_steps(mach, *steps);
            }
        }
        EventAction::Shift165(bits) => {
            if let Some(b) = board.as_ref() {
                b.set_shift165_bits(mach, *bits);
            }
        }
        EventAction::RailMv(mv) => {
            if let Some(b) = board.as_ref() {
                b.set_supply_mv(mach, *mv);
            }
        }
        EventAction::BatMv(mv) => {
            if let Some(b) = board.as_ref() {
                b.set_battery_mv(mach, *mv);
            } else {
                let mut bat = BatteryModel::default();
                bat.enabled = true;
                bat.set_mv(*mv);
                bat.apply(&mut mach.bus.adc);
            }
        }
        EventAction::ResetPin => mach.pin_reset(),
        EventAction::ResetSw => mach.soft_reset(ch585_core::peripherals::sys::RST_FLAG_SW),
        EventAction::PowerOff => mach.power_off(),
        EventAction::PowerOn => mach.power_on(),
        EventAction::PowerCycle => mach.power_cycle(),
        EventAction::VddMv(mv) => mach.set_vdd_mv(*mv),
        EventAction::FlashCutPct(pct) => {
            mach.bus.flash_ctrl.power_cut_next_op_pct = Some(*pct);
        }
        EventAction::BleScan => {
            let _ = mach.ble_scan();
        }
        EventAction::BleConnect => {
            mach.ble_connect([0; 6]);
        }
        EventAction::BlePair => {
            mach.ble_pair_and_bond();
        }
        EventAction::BleNotify => {
            mach.ble_enable_notifications();
        }
        EventAction::BleParamUpdate {
            interval,
            latency,
            timeout,
        } => {
            mach.ble_update_conn_params(*interval, *latency, *timeout);
        }
        EventAction::BleDisconnect { reason } => {
            mach.ble_disconnect(*reason);
        }
        EventAction::BleReconnect => {
            mach.ble_reconnect();
        }
        EventAction::BleGattWrite { uuid, data } => {
            mach.ble_gatt_write_uuid(*uuid, data);
        }
    }
}

/// Minimal dependency-free parser for scenario JSON arrays:
/// e.g. `[{"at_ms": 200, "adc": "1=2800"}, {"at": "250ms", "action": "pin:PB9=0"}, ...]`
pub fn parse_scenario_json(src: &str) -> Result<Vec<TimedEvent>, String> {
    let mut events = Vec::new();
    let trimmed = src.trim();
    if !trimmed.starts_with('[') || !trimmed.ends_with(']') {
        return Err("Scenario JSON must be an array [...] of event objects".into());
    }
    let body = &trimmed[1..trimmed.len() - 1];
    for obj_str in split_top_level_objects(body) {
        let fields = parse_flat_json_obj(obj_str)?;
        let at_us = if let Some(v) = fields.get("at_us") {
            v.parse::<u64>().map_err(|_| format!("Invalid at_us '{v}'"))?
        } else if let Some(v) = fields.get("at_ms") {
            let ms: f64 = v.parse().map_err(|_| format!("Invalid at_ms '{v}'"))?;
            (ms.max(0.0) * 1000.0).round() as u64
        } else if let Some(v) = fields.get("at") {
            parse_time_us(v).ok_or_else(|| format!("Invalid at '{v}'"))?
        } else {
            0
        };

        if let Some(act) = fields.get("action") {
            let (k, sp) = act
                .split_once(':')
                .ok_or_else(|| format!("Invalid action '{act}'"))?;
            events.push(TimedEvent {
                at_us,
                action: parse_action(k, sp)?,
            });
            continue;
        }
        for key in [
            "pin",
            "pull",
            "adc",
            "adc-noise",
            "noise",
            "usb",
            "uart",
            "enc",
            "sr165",
            "rail",
            "bat",
            "reset",
            "power",
            "pwr",
            "vdd",
            "flash-cut",
            "ble",
        ] {
            if let Some(val) = fields.get(key) {
                events.push(TimedEvent {
                    at_us,
                    action: parse_action(key, val)?,
                });
            }
        }
    }
    Ok(events)
}

fn split_top_level_objects(s: &str) -> Vec<&str> {
    let mut out = Vec::new();
    let bytes = s.as_bytes();
    let mut depth = 0i32;
    let mut in_str = false;
    let mut esc = false;
    let mut start = None;

    for (i, &b) in bytes.iter().enumerate() {
        if in_str {
            if esc {
                esc = false;
            } else if b == b'\\' {
                esc = true;
            } else if b == b'"' {
                in_str = false;
            }
            continue;
        }
        match b {
            b'"' => in_str = true,
            b'{' => {
                if depth == 0 {
                    start = Some(i);
                }
                depth += 1;
            }
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    if let Some(st) = start.take() {
                        out.push(&s[st..=i]);
                    }
                }
            }
            _ => {}
        }
    }
    out
}

fn parse_flat_json_obj(s: &str) -> Result<std::collections::HashMap<String, String>, String> {
    let inner = s.trim().trim_start_matches('{').trim_end_matches('}');
    let mut map = std::collections::HashMap::new();
    let mut cur = String::new();
    let mut parts = Vec::new();
    let mut in_str = false;
    let mut esc = false;
    for ch in inner.chars() {
        if in_str {
            cur.push(ch);
            if esc {
                esc = false;
            } else if ch == '\\' {
                esc = true;
            } else if ch == '"' {
                in_str = false;
            }
        } else if ch == '"' {
            in_str = true;
            cur.push(ch);
        } else if ch == ',' {
            parts.push(std::mem::take(&mut cur));
        } else {
            cur.push(ch);
        }
    }
    if !cur.trim().is_empty() {
        parts.push(cur);
    }

    for p in parts {
        let trimmed = p.trim();
        if trimmed.is_empty() {
            continue;
        }
        let Some((k, v)) = trimmed.split_once(':') else {
            return Err(format!("Bad JSON field '{trimmed}'"));
        };
        let key = unquote_json(k);
        let val = unquote_json(v);
        map.insert(key, val);
    }
    Ok(map)
}

fn unquote_json(s: &str) -> String {
    let t = s.trim();
    if t.starts_with('"') && t.ends_with('"') && t.len() >= 2 {
        t[1..t.len() - 1]
            .replace("\\\"", "\"")
            .replace("\\\\", "\\")
            .replace("\\n", "\n")
            .replace("\\r", "\r")
    } else {
        t.to_string()
    }
}
