use std::alloc::{alloc, dealloc, Layout};
use std::cell::RefCell;
use std::fmt::Write as _;

use ch585_core::cpu::Privilege;
use ch585_core::peripherals::adc::AdcWaveform;
use ch585_core::peripherals::gpio::{ExtPinDrive, PinElectricState};
use ch585_core::peripherals::{PinKind, QFN48_PINS};
use ch585_core::soc::Ch585Machine;

struct WasmState {
    mach: Ch585Machine,
    uart_cursors: [usize; 4],
    snapshot_json: Vec<u8>,
}

impl WasmState {
    fn new() -> Self {
        Self {
            mach: Ch585Machine::new(),
            uart_cursors: [0; 4],
            snapshot_json: Vec::with_capacity(16 * 1024),
        }
    }
}

thread_local! {
    static STATE: RefCell<WasmState> = RefCell::new(WasmState::new());
}

#[no_mangle]
pub extern "C" fn ch585_wasm_abi_version() -> u32 {
    2
}

#[no_mangle]
pub extern "C" fn wasm_alloc(size: usize) -> *mut u8 {
    let layout = match Layout::from_size_align(size.max(1), 8) {
        Ok(l) => l,
        Err(_) => return std::ptr::null_mut(),
    };
    unsafe { alloc(layout) }
}

#[no_mangle]
pub extern "C" fn wasm_free(ptr: *mut u8, size: usize) {
    if ptr.is_null() {
        return;
    }
    if let Ok(layout) = Layout::from_size_align(size.max(1), 8) {
        unsafe { dealloc(ptr, layout) };
    }
}

#[no_mangle]
pub extern "C" fn ch585_init() {
    STATE.with(|cell| {
        *cell.borrow_mut() = WasmState::new();
    });
}

#[no_mangle]
pub extern "C" fn ch585_reset() {
    STATE.with(|cell| {
        let mut s = cell.borrow_mut();
        s.mach.hard_reset();
        s.uart_cursors = [0; 4];
        s.snapshot_json.clear();
    });
}

/// Returns `0` on success, `-1` if invalid pointer/len, `-2` on ELF parse error.
#[no_mangle]
pub extern "C" fn ch585_load_elf(ptr: *const u8, len: usize) -> i32 {
    if ptr.is_null() || len == 0 {
        return -1;
    }
    let bytes = unsafe { std::slice::from_raw_parts(ptr, len) };
    STATE.with(|cell| {
        let mut s = cell.borrow_mut();
        s.mach = Ch585Machine::new();
        s.uart_cursors = [0; 4];
        s.snapshot_json.clear();
        match s.mach.load_elf(bytes) {
            Ok(()) => 0,
            Err(_) => -2,
        }
    })
}

#[no_mangle]
pub extern "C" fn ch585_step_cycles(cycles: u32) -> u32 {
    STATE.with(|cell| {
        let mut s = cell.borrow_mut();
        s.mach.run_cycles(cycles as u64) as u32
    })
}

#[no_mangle]
pub extern "C" fn ch585_step_us(us: u32) -> u32 {
    STATE.with(|cell| {
        let mut s = cell.borrow_mut();
        s.mach.run_us(us as u64) as u32
    })
}

/// `port`: `b'A'`/`0` -> PA, `b'B'`/`1` -> PB.
/// `mode`: `0` = Floating, `1` = Low, `2` = High, `3` = AnalogMv(mv).
#[no_mangle]
pub extern "C" fn ch585_set_pin(port: u8, idx: u8, mode: u8, mv: u16) {
    let port_char = match port {
        0 | b'A' | b'a' => 'A',
        1 | b'B' | b'b' => 'B',
        _ => return,
    };
    let drive = match mode {
        1 => ExtPinDrive::Low,
        2 => ExtPinDrive::High,
        3 => ExtPinDrive::AnalogMv(mv),
        _ => ExtPinDrive::Floating,
    };
    STATE.with(|cell| {
        cell.borrow_mut().mach.set_pin_drive(port_char, idx, drive);
    });
}

/// `kind`: `0`=Dc(p0), `1`=Sine{offset=p0, amp=p1, freq_hz=p2}, `2`=Ramp{min=p0, max=p1, period_ms=p2},
/// `3`=Pulse{base=p0, peak=p1, period_ms=p2, duty=p3}.
#[no_mangle]
pub extern "C" fn ch585_set_adc_wave(ch: u8, kind: u8, p0: i32, p1: i32, p2: f32, p3: f32) {
    if (ch as usize) >= 14 {
        return;
    }
    let wave = match kind {
        1 => AdcWaveform::Sine {
            offset_mv: p0,
            amp_mv: p1,
            freq_hz: p2,
        },
        2 => AdcWaveform::Ramp {
            min_mv: p0,
            max_mv: p1,
            period_ms: p2,
        },
        3 => AdcWaveform::Pulse {
            base_mv: p0,
            peak_mv: p1,
            period_ms: p2,
            duty: p3,
        },
        _ => AdcWaveform::Dc(p0),
    };
    STATE.with(|cell| {
        cell.borrow_mut().mach.bus.adc.channels[ch as usize] = wave;
    });
}

#[no_mangle]
pub extern "C" fn ch585_uart_inject_rx(uart_idx: u8, ptr: *const u8, len: usize) {
    if (uart_idx as usize) >= 4 || ptr.is_null() || len == 0 {
        return;
    }
    let bytes = unsafe { std::slice::from_raw_parts(ptr, len) };
    STATE.with(|cell| {
        cell.borrow_mut().mach.bus.uarts[uart_idx as usize].inject_rx(bytes);
    });
}

#[no_mangle]
pub extern "C" fn ch585_usb_enumerate() {
    STATE.with(|cell| {
        cell.borrow_mut().mach.bus.usbhs.trigger_enumerate();
    });
}

#[no_mangle]
pub extern "C" fn ch585_usb_send_out(ep: u8, ptr: *const u8, len: usize) {
    if ptr.is_null() {
        return;
    }
    let bytes = unsafe { std::slice::from_raw_parts(ptr, len) };
    STATE.with(|cell| {
        cell.borrow_mut()
            .mach
            .bus
            .usbhs
            .queue_out_packet(ep, bytes.to_vec());
    });
}

#[no_mangle]
pub extern "C" fn ch585_usb_clear_packets() {
    STATE.with(|cell| {
        cell.borrow_mut().mach.bus.usbhs.in_packets.clear();
    });
}

#[no_mangle]
pub extern "C" fn ch585_logic_clear() {
    STATE.with(|cell| {
        cell.borrow_mut().mach.bus.logic.clear();
    });
}

#[no_mangle]
pub extern "C" fn ch585_build_snapshot(max_logic: u32) -> usize {
    STATE.with(|cell| {
        let mut s = cell.borrow_mut();
        let json = build_snapshot_json(&mut s, max_logic as usize);
        s.snapshot_json = json.into_bytes();
        s.snapshot_json.len()
    })
}

#[no_mangle]
pub extern "C" fn ch585_get_snapshot_ptr() -> *const u8 {
    STATE.with(|cell| cell.borrow().snapshot_json.as_ptr())
}

#[no_mangle]
pub extern "C" fn ch585_get_snapshot_len() -> usize {
    STATE.with(|cell| cell.borrow().snapshot_json.len())
}

fn push_json_str(out: &mut String, s: &str) {
    out.push('"');
    for ch in s.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

fn push_hex_str(out: &mut String, bytes: &[u8]) {
    out.push('"');
    for (i, b) in bytes.iter().enumerate() {
        if i > 0 {
            out.push(' ');
        }
        let _ = write!(out, "{b:02X}");
    }
    out.push('"');
}

fn build_snapshot_json(state: &mut WasmState, max_logic: usize) -> String {
    let mach = &state.mach;
    let mut out = String::with_capacity(12 * 1024);

    let mode_str = match mach.cpu.priv_mode {
        Privilege::Machine => "M",
        Privilege::User => "U",
    };

    out.push('{');
    let _ = write!(
        out,
        "\"pc\":{},\"cycles\":{},\"instret\":{},\"timeUs\":{},\"sysClkHz\":{},\"wfi\":{},\"mode\":\"{}\",\"unknownMmio\":{},",
        mach.cpu.pc,
        mach.cpu.cycles,
        mach.cpu.instret,
        mach.bus.total_time_ns / 1000,
        mach.bus.sys.sys_clk_hz(),
        mach.cpu.halted_wfi,
        mode_str,
        mach.bus.unknown_mmio.len(),
    );

    // PFIC summary
    let pend0 = mach.bus.pfic.ipr[0] | mach.bus.pfic.hw_ipr[0];
    let pend1 = mach.bus.pfic.ipr[1] | mach.bus.pfic.hw_ipr[1];
    let _ = write!(
        out,
        "\"pfic\":{{\"en\":[{},{}],\"pend\":[{},{}],\"active\":{:?}}},",
        mach.bus.pfic.isr[0],
        mach.bus.pfic.isr[1],
        pend0,
        pend1,
        mach.bus.pfic.active_stack,
    );

    // GPIO port registers
    let _ = write!(
        out,
        "\"gpio\":{{\"paDir\":{},\"paOut\":{},\"paPin\":{},\"pbDir\":{},\"pbOut\":{},\"pbPin\":{}}},",
        mach.bus.gpio.pa_dir,
        mach.bus.gpio.pa_out,
        mach.bus.gpio.effective_pa_pin(),
        mach.bus.gpio.pb_dir,
        mach.bus.gpio.pb_out,
        mach.bus.gpio.effective_pb_pin(),
    );

    // 48 QFN48 pins
    let snaps = mach.qfn48_snapshot();
    out.push_str("\"pins\":[");
    for (idx, p) in snaps.iter().enumerate() {
        if idx > 0 {
            out.push(',');
        }
        let st = match p.electric {
            PinElectricState::Low => "low",
            PinElectricState::High => "high",
            PinElectricState::HiZ => "hiz",
            PinElectricState::Analog => "analog",
        };
        let (port_str, pin_idx_json, ext_str) = match QFN48_PINS[idx].kind {
            PinKind::GpioA(i) => {
                let ext = ext_label(mach.bus.gpio.ext_pa[i as usize]);
                ("A", i as i32, ext)
            }
            PinKind::GpioB(i) => {
                let ext = ext_label(mach.bus.gpio.ext_pb[i as usize]);
                ("B", i as i32, ext)
            }
            PinKind::AnalogOnly => ("X", -1, "analog"),
            PinKind::Power => ("P", -1, "power"),
        };
        out.push('{');
        let _ = write!(out, "\"pin\":{},\"name\":", p.pin_num);
        push_json_str(&mut out, p.name);
        out.push_str(",\"alt\":");
        push_json_str(&mut out, p.alt_funcs);
        let _ = write!(
            out,
            ",\"port\":\"{port_str}\",\"pidx\":{pin_idx_json},\"state\":\"{st}\",\"ext\":\"{ext_str}\",\"out\":{},\"pu\":{},\"pd\":{},",
            p.is_output, p.pull_up, p.pull_down
        );
        match p.adc_ch {
            Some(ch) => {
                let _ = write!(out, "\"adc\":{ch},");
            }
            None => out.push_str("\"adc\":null,"),
        }
        match p.analog_mv {
            Some(mv) => {
                let _ = write!(out, "\"mv\":{mv}");
            }
            None => out.push_str("\"mv\":null"),
        }
        out.push('}');
    }
    out.push_str("],");

    // Live 14 ADC channel voltages
    let time_s = (mach.bus.total_time_ns as f64) * 1e-9;
    out.push_str("\"adcMv\":[");
    for ch in 0..14u8 {
        if ch > 0 {
            out.push(',');
        }
        let mv = mach
            .bus
            .adc
            .sample_channel_mv(ch, time_s, &mach.bus.gpio);
        let _ = write!(out, "{mv}");
    }
    out.push_str("],");

    // UART deltas
    out.push_str("\"uartDeltas\":[");
    for i in 0..4 {
        if i > 0 {
            out.push(',');
        }
        let tx = &mach.bus.uarts[i].tx_log;
        let cur = state.uart_cursors[i].min(tx.len());
        let text = String::from_utf8_lossy(&tx[cur..]);
        push_json_str(&mut out, &text);
        state.uart_cursors[i] = tx.len();
    }
    out.push_str("],");

    // USB HS
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
    let _ = write!(
        out,
        "\"usb\":{{\"enumerated\":{},\"devAd\":{},\"vid\":{},\"pid\":{},\"cfgLen\":{},\"hidDescs\":{},\"totalPkts\":{},\"packets\":[",
        u.enumerated,
        u.dev_ad,
        vid,
        pid,
        u.config_desc.len(),
        u.hid_report_descs.len(),
        u.in_packets.len(),
    );
    let start_pkt = u.in_packets.len().saturating_sub(24);
    for (idx, pkt) in u.in_packets[start_pkt..].iter().enumerate() {
        if idx > 0 {
            out.push(',');
        }
        let _ = write!(
            out,
            "{{\"ep\":{},\"in\":{},\"len\":{},\"hex\":",
            pkt.ep,
            pkt.is_in,
            pkt.data.len()
        );
        push_hex_str(&mut out, &pkt.data[..pkt.data.len().min(64)]);
        out.push('}');
    }
    out.push_str("]},");

    // BLE Bridge
    let b = &mach.bus.ble;
    let _ = write!(
        out,
        "\"ble\":{{\"hooks\":{},\"advertising\":{},\"advHex\":",
        b.hooks.len(),
        b.advertising_enabled,
    );
    push_hex_str(&mut out, &b.adv_data);
    out.push_str(",\"scanHex\":");
    push_hex_str(&mut out, &b.scan_rsp_data);
    out.push_str(",\"events\":[");
    let start_ev = b.event_log.len().saturating_sub(24);
    for (idx, ev) in b.event_log[start_ev..].iter().enumerate() {
        if idx > 0 {
            out.push(',');
        }
        push_json_str(&mut out, ev);
    }
    out.push_str("],\"hidReports\":[");
    let start_hr = b.hid_reports.len().saturating_sub(8);
    for (idx, rpt) in b.hid_reports[start_hr..].iter().enumerate() {
        if idx > 0 {
            out.push(',');
        }
        push_hex_str(&mut out, rpt);
    }
    out.push_str("]},");

    // Logic analyzer samples: [time_ns, pa, pb]
    let samples = &mach.bus.logic.samples;
    let take_n = max_logic.max(1);
    let start_log = samples.len().saturating_sub(take_n);
    let _ = write!(out, "\"logicTotal\":{},\"logic\":[", samples.len());
    for (idx, smp) in samples[start_log..].iter().enumerate() {
        if idx > 0 {
            out.push(',');
        }
        let _ = write!(out, "[{},{},{}]", smp.time_ns, smp.pa, smp.pb);
    }
    out.push_str("]}");

    out
}

fn ext_label(d: ExtPinDrive) -> &'static str {
    match d {
        ExtPinDrive::Floating => "float",
        ExtPinDrive::Low => "low",
        ExtPinDrive::High => "high",
        ExtPinDrive::AnalogMv(_) => "analog",
    }
}
