use std::fmt::Write as _;

use crate::ble_stub::{BleHidReport, BleTimestampedEvent};
use crate::json::JsonValue;
use crate::logic::LogicSample;
use crate::peripherals::sys::{PowerMode, PowerReport};
use crate::profiler::ProfilerReport;
use crate::soc::Ch585Machine;

#[derive(Debug, Clone, Copy)]
pub struct PerfStats {
    pub wall_ms: f64,
    pub speed_x: f64,
    pub mips: f64,
}

#[derive(Debug, Clone)]
pub struct UartChannelReport {
    pub idx: usize,
    pub tx_bytes: usize,
    pub tx_text: String,
    pub tx_hex: String,
}

#[derive(Debug, Clone)]
pub struct UsbPacketEntry {
    pub ep: u8,
    pub is_in: bool,
    pub data_hex: String,
}

#[derive(Debug, Clone)]
pub struct DecodedNkroReport {
    pub mods: u8,
    pub keys: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct DecodedVendorResponse {
    pub msg_type: u8,
    pub seq: u8,
    pub data_hex: String,
}

#[derive(Debug, Clone)]
pub struct MachineReport {
    pub schema_version: u32,
    pub pc: u32,
    pub cycles: u64,
    pub instret: u64,
    pub exact_periph_step: bool,
    pub halted_wfi: bool,
    pub sys_clk_hz: u32,
    pub lse_failed: bool,
    pub time_ns: u64,
    pub time_us: u64,
    pub sleep_skipped_us: u64,
    pub unpowered_us: u64,
    pub powered: bool,
    pub bor_active: bool,
    pub vdd_mv: i32,
    pub last_reset_code: u8,
    pub last_reset_name: &'static str,
    pub flash_interrupted_ops: u32,
    pub mode_us: [u64; 5],
    pub power: PowerReport,
    pub pa_dir: u16,
    pub pa_out: u16,
    pub pa_pin: u16,
    pub pb_dir: u32,
    pub pb_out: u32,
    pub pb_pin: u32,
    pub contention: Vec<String>,
    pub logic_samples_count: usize,
    pub gpio_transitions: Option<Vec<LogicSample>>,
    pub uarts: Vec<UartChannelReport>,
    pub usb_attached: bool,
    pub usb_enumerated: bool,
    pub usb_suspended: bool,
    pub usb_remote_wakeup_count: u32,
    pub usb_dev_ad: u8,
    pub usb_vid: u16,
    pub usb_pid: u16,
    pub usb_packets: Vec<UsbPacketEntry>,
    pub usb_nkro_reports: Vec<DecodedNkroReport>,
    pub usb_ep4_telemetry: Vec<String>,
    pub usb_ep4_responses: Vec<DecodedVendorResponse>,
    pub ble_hooks: usize,
    pub ble_advertising: bool,
    pub ble_adv_data_hex: String,
    pub ble_scan_rsp_hex: String,
    pub ble_events: Vec<String>,
    pub ble_hid_reports_hex: Vec<String>,
    pub ble_central_state: &'static str,
    pub ble_bonds: usize,
    pub ble_conn_interval: u16,
    pub ble_conn_latency: u16,
    pub ble_conn_timeout: u16,
    pub ble_tmos_start_calls: u64,
    pub ble_low_power_calls: u64,
    pub ble_timestamped_events: Vec<BleTimestampedEvent>,
    pub ble_timestamped_hid_reports: Vec<BleHidReport>,
    pub rf_mode: &'static str,
    pub rf_bound: bool,
    pub rf_tx_count: u64,
    pub rf_rx_count: u64,
    pub rf_rx_crc_errors: u64,
    pub rf_timeouts: u64,
    pub unknown_mmio: Vec<u32>,
    pub profile: Option<ProfilerReport>,
}

pub fn bytes_to_hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for &b in bytes {
        let _ = write!(s, "{b:02X}");
    }
    s
}

pub fn hid_usage_name(code: u8) -> String {
    match code {
        0x04..=0x1D => format!("{}", (b'A' + (code - 0x04)) as char),
        0x1E..=0x26 => format!("{}", (b'1' + (code - 0x1E)) as char),
        0x27 => "0".into(),
        0x28 => "Enter".into(),
        0x29 => "Esc".into(),
        0x2A => "Bksp".into(),
        0x2B => "Tab".into(),
        0x2C => "Space".into(),
        _ => format!("0x{code:02X}"),
    }
}

pub fn decode_nkro_usages(data: &[u8]) -> Option<(u8, Vec<String>)> {
    if data.len() < 2 {
        return None;
    }
    let mods = data[0];
    let mut keys = Vec::new();
    for (byte_idx, &b) in data.iter().enumerate().skip(1) {
        for bit in 0..8 {
            if (b & (1 << bit)) != 0 {
                let usage = (8 * (byte_idx - 1) + bit) as u8;
                keys.push(hid_usage_name(usage));
            }
        }
    }
    Some((mods, keys))
}

impl MachineReport {
    pub fn from_machine(
        mach: &Ch585Machine,
        uart_filter: Option<usize>,
        include_gpio_transitions: bool,
    ) -> Self {
        let tm = &mach.bus.time_ns_by_mode;
        let reset_code = mach.bus.sys.reset_flag & 0x07;
        let reset_name = match reset_code {
            0x00 => "SW",
            0x01 => "RPOR",
            0x02 => "WTR",
            0x03 => "MR",
            0x05 => "GPWSM",
            _ => "OTHER",
        };

        let mut uarts = Vec::new();
        for u in 0..4 {
            if uart_filter.is_some() && uart_filter != Some(u) {
                continue;
            }
            let tx = &mach.bus.uarts[u].tx_log;
            uarts.push(UartChannelReport {
                idx: u,
                tx_bytes: tx.len(),
                tx_text: String::from_utf8_lossy(tx).to_string(),
                tx_hex: bytes_to_hex(tx),
            });
        }

        let u = &mach.bus.usbhs;
        let vid = if u.device_desc.len() >= 10 {
            u16::from_le_bytes([u.device_desc[8], u.device_desc[9]])
        } else {
            0
        };
        let pid = if u.device_desc.len() >= 12 {
            u16::from_le_bytes([u.device_desc[10], u.device_desc[11]])
        } else {
            0
        };

        let mut usb_packets = Vec::with_capacity(u.in_packets.len());
        let mut usb_nkro_reports = Vec::new();
        let mut telem_buf = String::new();
        let mut usb_ep4_responses = Vec::new();
        for pkt in &u.in_packets {
            usb_packets.push(UsbPacketEntry {
                ep: pkt.ep,
                is_in: pkt.is_in,
                data_hex: bytes_to_hex(&pkt.data),
            });
            if pkt.ep == 1 && pkt.is_in {
                if let Some((mods, keys)) = decode_nkro_usages(&pkt.data) {
                    usb_nkro_reports.push(DecodedNkroReport { mods, keys });
                }
            } else if pkt.ep == 4
                && pkt.is_in
                && pkt.data.len() >= 6
                && pkt.data[0] == 0x57
                && pkt.data[1] == 0x01
            {
                if pkt.data[2] == 0x02 {
                    let n = ((pkt.data[5] & 0x7F) as usize).min(pkt.data.len().saturating_sub(6));
                    telem_buf.push_str(&String::from_utf8_lossy(&pkt.data[6..6 + n]));
                } else {
                    usb_ep4_responses.push(DecodedVendorResponse {
                        msg_type: pkt.data[2],
                        seq: pkt.data[3],
                        data_hex: bytes_to_hex(&pkt.data[4..]),
                    });
                }
            }
        }
        let usb_ep4_telemetry: Vec<String> = telem_buf
            .lines()
            .filter(|l| !l.is_empty())
            .map(ToString::to_string)
            .collect();

        let ble = &mach.bus.ble;
        Self {
            schema_version: 1,
            pc: mach.cpu.pc,
            cycles: mach.cpu.cycles,
            instret: mach.cpu.instret,
            exact_periph_step: mach.exact_periph_step,
            halted_wfi: mach.cpu.halted_wfi,
            sys_clk_hz: mach.bus.sys.sys_clk_hz(),
            lse_failed: mach.bus.sys.lse_failed,
            time_ns: mach.bus.total_time_ns,
            time_us: mach.bus.total_time_ns / 1000,
            sleep_skipped_us: mach.bus.sleep_skipped_ns / 1000,
            unpowered_us: mach.bus.unpowered_ns / 1000,
            powered: mach.bus.powered,
            bor_active: mach.bus.bor_active,
            vdd_mv: mach.bus.vdd_mv,
            last_reset_code: reset_code,
            last_reset_name: reset_name,
            flash_interrupted_ops: mach.bus.flash_ctrl.interrupted_ops,
            mode_us: [
                tm[PowerMode::Active as usize] / 1000,
                tm[PowerMode::Idle as usize] / 1000,
                tm[PowerMode::Halt as usize] / 1000,
                tm[PowerMode::Sleep as usize] / 1000,
                tm[PowerMode::Shutdown as usize] / 1000,
            ],
            power: mach.bus.power_report(),
            pa_dir: mach.bus.gpio.pa_dir,
            pa_out: mach.bus.gpio.pa_out,
            pa_pin: mach.bus.gpio.effective_pa_pin(),
            pb_dir: mach.bus.gpio.pb_dir,
            pb_out: mach.bus.gpio.pb_out,
            pb_pin: mach.bus.gpio.effective_pb_pin(),
            contention: mach.bus.gpio.contention_log.clone(),
            logic_samples_count: mach.bus.logic.samples.len(),
            gpio_transitions: if include_gpio_transitions {
                Some(mach.bus.logic.samples.clone())
            } else {
                None
            },
            uarts,
            usb_attached: u.host_attached,
            usb_enumerated: u.enumerated,
            usb_suspended: u.suspended,
            usb_remote_wakeup_count: u.remote_wakeup_count,
            usb_dev_ad: u.dev_ad,
            usb_vid: vid,
            usb_pid: pid,
            usb_packets,
            usb_nkro_reports,
            usb_ep4_telemetry,
            usb_ep4_responses,
            ble_hooks: ble.hooks.len(),
            ble_advertising: ble.advertising_enabled,
            ble_adv_data_hex: bytes_to_hex(&ble.adv_data),
            ble_scan_rsp_hex: bytes_to_hex(&ble.scan_rsp_data),
            ble_events: ble.event_log.clone(),
            ble_hid_reports_hex: ble.hid_reports.iter().map(|r| bytes_to_hex(r)).collect(),
            ble_central_state: ble.central_state.as_str(),
            ble_bonds: ble.bonds.len(),
            ble_conn_interval: ble.conn_params.interval,
            ble_conn_latency: ble.conn_params.latency,
            ble_conn_timeout: ble.conn_params.timeout,
            ble_tmos_start_calls: ble.tmos_start_calls,
            ble_low_power_calls: ble.low_power_calls,
            ble_timestamped_events: ble.timestamped_events.clone(),
            ble_timestamped_hid_reports: ble.timestamped_hid_reports.clone(),
            rf_mode: ble.rf.mode.as_str(),
            rf_bound: ble.rf.bound_connected,
            rf_tx_count: ble.rf.tx_count,
            rf_rx_count: ble.rf.rx_count,
            rf_rx_crc_errors: ble.rf.rx_crc_err_count,
            rf_timeouts: ble.rf.timeout_count,
            unknown_mmio: mach.bus.unknown_mmio.clone(),
            profile: if mach.profiler.enabled {
                Some(mach.profiler_report())
            } else {
                None
            },
        }
    }

    pub fn to_json_value(&self, board: Option<JsonValue>, perf: Option<PerfStats>) -> JsonValue {
        let mut gpio_fields = vec![
            ("pa_dir".into(), JsonValue::Str(format!("0x{:04X}", self.pa_dir))),
            ("pa_out".into(), JsonValue::Str(format!("0x{:04X}", self.pa_out))),
            ("pa_pin".into(), JsonValue::Str(format!("0x{:04X}", self.pa_pin))),
            ("pb_dir".into(), JsonValue::Str(format!("0x{:06X}", self.pb_dir))),
            ("pb_out".into(), JsonValue::Str(format!("0x{:06X}", self.pb_out))),
            ("pb_pin".into(), JsonValue::Str(format!("0x{:06X}", self.pb_pin))),
            (
                "contention".into(),
                JsonValue::Array(
                    self.contention
                        .iter()
                        .map(|s| JsonValue::Str(s.clone()))
                        .collect(),
                ),
            ),
            (
                "logic_samples_count".into(),
                JsonValue::Number(self.logic_samples_count as f64),
            ),
        ];
        if let Some(samples) = &self.gpio_transitions {
            gpio_fields.push((
                "transitions".into(),
                JsonValue::Array(
                    samples
                        .iter()
                        .map(|s| {
                            JsonValue::Object(vec![
                                ("cycle".into(), JsonValue::Number(s.cycle as f64)),
                                ("time_ns".into(), JsonValue::Number(s.time_ns as f64)),
                                ("pa".into(), JsonValue::Str(format!("0x{:04X}", s.pa))),
                                ("pb".into(), JsonValue::Str(format!("0x{:06X}", s.pb))),
                            ])
                        })
                        .collect(),
                ),
            ));
        }

        let mut root = vec![
            ("schema_version".into(), JsonValue::Number(self.schema_version as f64)),
            (
                "cpu".into(),
                JsonValue::Object(vec![
                    ("pc".into(), JsonValue::Str(format!("0x{:08X}", self.pc))),
                    ("cycles".into(), JsonValue::Number(self.cycles as f64)),
                    ("instret".into(), JsonValue::Number(self.instret as f64)),
                    ("exact".into(), JsonValue::Bool(self.exact_periph_step)),
                    ("halted_wfi".into(), JsonValue::Bool(self.halted_wfi)),
                ]),
            ),
            (
                "clock".into(),
                JsonValue::Object(vec![
                    ("sys_clk_hz".into(), JsonValue::Number(self.sys_clk_hz as f64)),
                    ("lse_failed".into(), JsonValue::Bool(self.lse_failed)),
                ]),
            ),
            (
                "time".into(),
                JsonValue::Object(vec![
                    ("time_ns".into(), JsonValue::Number(self.time_ns as f64)),
                    ("time_us".into(), JsonValue::Number(self.time_us as f64)),
                    (
                        "sleep_skipped_us".into(),
                        JsonValue::Number(self.sleep_skipped_us as f64),
                    ),
                    ("unpowered_us".into(), JsonValue::Number(self.unpowered_us as f64)),
                ]),
            ),
            (
                "power".into(),
                JsonValue::Object(vec![
                    ("powered".into(), JsonValue::Bool(self.powered)),
                    ("bor_active".into(), JsonValue::Bool(self.bor_active)),
                    ("vdd_mv".into(), JsonValue::Number(self.vdd_mv as f64)),
                    (
                        "last_reset".into(),
                        JsonValue::Object(vec![
                            ("code".into(), JsonValue::Number(self.last_reset_code as f64)),
                            ("name".into(), JsonValue::Str(self.last_reset_name.to_string())),
                        ]),
                    ),
                    (
                        "flash_interrupted_ops".into(),
                        JsonValue::Number(self.flash_interrupted_ops as f64),
                    ),
                    (
                        "mode_us".into(),
                        JsonValue::Object(vec![
                            ("active".into(), JsonValue::Number(self.mode_us[0] as f64)),
                            ("idle".into(), JsonValue::Number(self.mode_us[1] as f64)),
                            ("halt".into(), JsonValue::Number(self.mode_us[2] as f64)),
                            ("sleep".into(), JsonValue::Number(self.mode_us[3] as f64)),
                            ("shutdown".into(), JsonValue::Number(self.mode_us[4] as f64)),
                        ]),
                    ),
                    (
                        "current_ua".into(),
                        JsonValue::Object(vec![
                            ("core".into(), JsonValue::Number(self.power.core_avg_ua)),
                            ("adc".into(), JsonValue::Number(self.power.adc_avg_ua)),
                            ("usbhs".into(), JsonValue::Number(self.power.usbhs_avg_ua)),
                            ("ble_rf".into(), JsonValue::Number(self.power.ble_rf_avg_ua)),
                            ("uart".into(), JsonValue::Number(self.power.uart_avg_ua)),
                            ("spi".into(), JsonValue::Number(self.power.spi_avg_ua)),
                            ("tmr".into(), JsonValue::Number(self.power.tmr_avg_ua)),
                            ("pwm".into(), JsonValue::Number(self.power.pwm_avg_ua)),
                            ("bat_mon".into(), JsonValue::Number(self.power.bat_mon_avg_ua)),
                            ("total".into(), JsonValue::Number(self.power.total_avg_ua)),
                        ]),
                    ),
                ]),
            ),
            ("gpio".into(), JsonValue::Object(gpio_fields)),
            (
                "uart".into(),
                JsonValue::Array(
                    self.uarts
                        .iter()
                        .map(|u| {
                            JsonValue::Object(vec![
                                ("idx".into(), JsonValue::Number(u.idx as f64)),
                                ("tx_bytes".into(), JsonValue::Number(u.tx_bytes as f64)),
                                ("tx_text".into(), JsonValue::Str(u.tx_text.clone())),
                                ("tx_hex".into(), JsonValue::Str(u.tx_hex.clone())),
                            ])
                        })
                        .collect(),
                ),
            ),
            (
                "usbhs".into(),
                JsonValue::Object(vec![
                    ("host_attached".into(), JsonValue::Bool(self.usb_attached)),
                    ("enumerated".into(), JsonValue::Bool(self.usb_enumerated)),
                    ("suspended".into(), JsonValue::Bool(self.usb_suspended)),
                    (
                        "remote_wakeup_count".into(),
                        JsonValue::Number(self.usb_remote_wakeup_count as f64),
                    ),
                    ("dev_ad".into(), JsonValue::Number(self.usb_dev_ad as f64)),
                    ("vid".into(), JsonValue::Str(format!("0x{:04X}", self.usb_vid))),
                    ("pid".into(), JsonValue::Str(format!("0x{:04X}", self.usb_pid))),
                    (
                        "packets_count".into(),
                        JsonValue::Number(self.usb_packets.len() as f64),
                    ),
                    (
                        "packets".into(),
                        JsonValue::Array(
                            self.usb_packets
                                .iter()
                                .map(|p| {
                                    JsonValue::Object(vec![
                                        ("ep".into(), JsonValue::Number(p.ep as f64)),
                                        (
                                            "dir".into(),
                                            JsonValue::Str(if p.is_in { "IN" } else { "OUT" }.to_string()),
                                        ),
                                        ("data_hex".into(), JsonValue::Str(p.data_hex.clone())),
                                    ])
                                })
                                .collect(),
                        ),
                    ),
                    (
                        "decoded".into(),
                        JsonValue::Object(vec![
                            (
                                "nkro_reports".into(),
                                JsonValue::Array(
                                    self.usb_nkro_reports
                                        .iter()
                                        .map(|r| {
                                            JsonValue::Object(vec![
                                                ("mods".into(), JsonValue::Str(format!("0x{:02X}", r.mods))),
                                                (
                                                    "keys".into(),
                                                    JsonValue::Array(
                                                        r.keys
                                                            .iter()
                                                            .map(|k| JsonValue::Str(k.clone()))
                                                            .collect(),
                                                    ),
                                                ),
                                            ])
                                        })
                                        .collect(),
                                ),
                            ),
                            (
                                "ep4_telemetry".into(),
                                JsonValue::Array(
                                    self.usb_ep4_telemetry
                                        .iter()
                                        .map(|s| JsonValue::Str(s.clone()))
                                        .collect(),
                                ),
                            ),
                            (
                                "ep4_responses".into(),
                                JsonValue::Array(
                                    self.usb_ep4_responses
                                        .iter()
                                        .map(|r| {
                                            JsonValue::Object(vec![
                                                ("msg_type".into(), JsonValue::Number(r.msg_type as f64)),
                                                ("seq".into(), JsonValue::Number(r.seq as f64)),
                                                ("data_hex".into(), JsonValue::Str(r.data_hex.clone())),
                                            ])
                                        })
                                        .collect(),
                                ),
                            ),
                        ]),
                    ),
                ]),
            ),
            (
                "ble".into(),
                JsonValue::Object(vec![
                    ("hooks".into(), JsonValue::Number(self.ble_hooks as f64)),
                    (
                        "advertising_enabled".into(),
                        JsonValue::Bool(self.ble_advertising),
                    ),
                    (
                        "central_state".into(),
                        JsonValue::Str(self.ble_central_state.to_string()),
                    ),
                    ("bonds".into(), JsonValue::Number(self.ble_bonds as f64)),
                    (
                        "conn_params".into(),
                        JsonValue::Object(vec![
                            ("interval".into(), JsonValue::Number(self.ble_conn_interval as f64)),
                            ("latency".into(), JsonValue::Number(self.ble_conn_latency as f64)),
                            ("timeout".into(), JsonValue::Number(self.ble_conn_timeout as f64)),
                        ]),
                    ),
                    (
                        "tmos_start_calls".into(),
                        JsonValue::Number(self.ble_tmos_start_calls as f64),
                    ),
                    (
                        "low_power_calls".into(),
                        JsonValue::Number(self.ble_low_power_calls as f64),
                    ),
                    (
                        "adv_data_hex".into(),
                        JsonValue::Str(self.ble_adv_data_hex.clone()),
                    ),
                    (
                        "scan_rsp_data_hex".into(),
                        JsonValue::Str(self.ble_scan_rsp_hex.clone()),
                    ),
                    (
                        "events".into(),
                        JsonValue::Array(
                            self.ble_events
                                .iter()
                                .map(|e| JsonValue::Str(e.clone()))
                                .collect(),
                        ),
                    ),
                    (
                        "timestamped_events".into(),
                        JsonValue::Array(
                            self.ble_timestamped_events
                                .iter()
                                .map(|e| {
                                    JsonValue::Object(vec![
                                        ("time_ns".into(), JsonValue::Number(e.time_ns as f64)),
                                        ("cycle".into(), JsonValue::Number(e.cycle as f64)),
                                        ("kind".into(), JsonValue::Str(e.kind.clone())),
                                        ("detail".into(), JsonValue::Str(e.detail.clone())),
                                    ])
                                })
                                .collect(),
                        ),
                    ),
                    (
                        "hid_reports_hex".into(),
                        JsonValue::Array(
                            self.ble_hid_reports_hex
                                .iter()
                                .map(|h| JsonValue::Str(h.clone()))
                                .collect(),
                        ),
                    ),
                    (
                        "timestamped_hid_reports".into(),
                        JsonValue::Array(
                            self.ble_timestamped_hid_reports
                                .iter()
                                .map(|r| {
                                    JsonValue::Object(vec![
                                        ("time_ns".into(), JsonValue::Number(r.time_ns as f64)),
                                        ("cycle".into(), JsonValue::Number(r.cycle as f64)),
                                        ("conn_handle".into(), JsonValue::Number(r.conn_handle as f64)),
                                        ("report_id".into(), JsonValue::Number(r.report_id as f64)),
                                        ("report_type".into(), JsonValue::Number(r.report_type as f64)),
                                        ("attr_handle".into(), JsonValue::Number(r.attr_handle as f64)),
                                        ("data_hex".into(), JsonValue::Str(bytes_to_hex(&r.data))),
                                    ])
                                })
                                .collect(),
                        ),
                    ),
                ]),
            ),
            (
                "rf".into(),
                JsonValue::Object(vec![
                    ("mode".into(), JsonValue::Str(self.rf_mode.to_string())),
                    ("bound".into(), JsonValue::Bool(self.rf_bound)),
                    ("tx_count".into(), JsonValue::Number(self.rf_tx_count as f64)),
                    ("rx_count".into(), JsonValue::Number(self.rf_rx_count as f64)),
                    (
                        "rx_crc_errors".into(),
                        JsonValue::Number(self.rf_rx_crc_errors as f64),
                    ),
                    ("timeouts".into(), JsonValue::Number(self.rf_timeouts as f64)),
                ]),
            ),
            (
                "unknown_mmio".into(),
                JsonValue::Array(
                    self.unknown_mmio
                        .iter()
                        .map(|a| JsonValue::Str(format!("0x{a:08X}")))
                        .collect(),
                ),
            ),
        ];

        if let Some(prof) = &self.profile {
            root.push(("profile".into(), prof.to_json_value(Some(32))));
        }
        if let Some(p) = perf {
            root.push((
                "perf".into(),
                JsonValue::Object(vec![
                    ("wall_ms".into(), JsonValue::Number(p.wall_ms)),
                    ("speed_x".into(), JsonValue::Number(p.speed_x)),
                    ("mips".into(), JsonValue::Number(p.mips)),
                ]),
            ));
        }
        if let Some(b) = board {
            root.push(("board".into(), b));
        }
        JsonValue::Object(root)
    }

    pub fn to_json_string(
        &self,
        board: Option<JsonValue>,
        perf: Option<PerfStats>,
        pretty: bool,
    ) -> String {
        self.to_json_value(board, perf).to_json_string(pretty)
    }

    /// Deterministic snapshot suitable for hardware-vs-emulator or cross-run differential comparison
    /// (omits wall-clock performance metrics).
    pub fn to_diff_json_string(&self) -> String {
        self.to_json_string(None, None, true)
    }
}
