use std::fmt::Write as _;

use crate::logic::LogicAnalyzer;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InternalTraceSample {
    pub time_ns: u64,
    pub active_irq: u8,
    pub irq_depth: u8,
    pub dma_mask: u8,
    pub usb_state: u8,
    pub usb_ep: u8,
    pub power_mode: u8,
}

impl Default for InternalTraceSample {
    fn default() -> Self {
        Self {
            time_ns: 0,
            active_irq: 0,
            irq_depth: 0,
            dma_mask: 0,
            usb_state: 0,
            usb_ep: 0,
            power_mode: 0,
        }
    }
}

impl InternalTraceSample {
    #[inline]
    fn same_state(&self, other: &Self) -> bool {
        self.active_irq == other.active_irq
            && self.irq_depth == other.irq_depth
            && self.dma_mask == other.dma_mask
            && self.usb_state == other.usb_state
            && self.usb_ep == other.usb_ep
            && self.power_mode == other.power_mode
    }
}

/// Change-compressed ring buffer recording internal IRQ, DMA, USB, and power state transitions.
#[derive(Debug, Clone)]
pub struct TraceBuffer {
    pub enabled: bool,
    pub samples: Vec<InternalTraceSample>,
    pub max_samples: usize,
    last: Option<InternalTraceSample>,
}

impl Default for TraceBuffer {
    fn default() -> Self {
        Self::new(65_536)
    }
}

impl TraceBuffer {
    pub fn new(max_samples: usize) -> Self {
        Self {
            enabled: false,
            samples: Vec::new(),
            max_samples: max_samples.max(64),
            last: None,
        }
    }

    pub fn clear(&mut self) {
        self.samples.clear();
        self.last = None;
    }

    #[inline]
    pub fn record_if_changed(&mut self, sample: InternalTraceSample) {
        if !self.enabled {
            return;
        }
        if let Some(prev) = &self.last {
            if prev.same_state(&sample) {
                return;
            }
        }
        self.last = Some(sample);
        if self.samples.len() >= self.max_samples {
            let drop_n = self.max_samples / 8;
            self.samples.drain(0..drop_n);
        }
        self.samples.push(sample);
    }
}

/// Configuration selecting GPIO pins and internal channels included in VCD export.
#[derive(Debug, Clone)]
pub struct VcdConfig {
    /// Explicit pins to export (`('A', idx)` / `('B', idx)`). When empty, all changing pins are auto-selected.
    pub pins: Vec<(char, u8)>,
    pub include_internal: bool,
}

impl Default for VcdConfig {
    fn default() -> Self {
        Self {
            pins: Vec::new(),
            include_internal: true,
        }
    }
}

fn vcd_id(mut idx: usize) -> String {
    let mut chars = Vec::new();
    loop {
        chars.push((33u8 + (idx % 94) as u8) as char);
        idx /= 94;
        if idx == 0 {
            break;
        }
        idx -= 1;
    }
    chars.into_iter().rev().collect()
}

#[inline]
fn read_pin_from_sample(pa: u16, pb: u32, port: char, idx: u8) -> u8 {
    match port {
        'A' | 'a' if idx < 16 => ((pa >> idx) & 1) as u8,
        'B' | 'b' if idx < 24 => ((pb >> idx) & 1) as u8,
        _ => 0,
    }
}

fn resolve_pins(logic: &LogicAnalyzer, cfg: &VcdConfig) -> Vec<(char, u8)> {
    if !cfg.pins.is_empty() {
        let mut out = Vec::new();
        for &(p, idx) in &cfg.pins {
            let port = p.to_ascii_uppercase();
            if ((port == 'A' && idx < 16) || (port == 'B' && idx < 24))
                && !out.contains(&(port, idx))
            {
                out.push((port, idx));
            }
        }
        return out;
    }
    let mut pa_diff = 0u16;
    let mut pb_diff = 0u32;
    for w in logic.samples.windows(2) {
        pa_diff |= w[0].pa ^ w[1].pa;
        pb_diff |= w[0].pb ^ w[1].pb;
    }
    if pa_diff == 0 && pb_diff == 0 {
        if let Some(first) = logic.samples.first() {
            pa_diff = first.pa;
            pb_diff = first.pb;
        }
    }
    let mut pins = Vec::new();
    for i in 0..16u8 {
        if ((pa_diff >> i) & 1) != 0 {
            pins.push(('A', i));
        }
    }
    for i in 0..24u8 {
        if ((pb_diff >> i) & 1) != 0 {
            pins.push(('B', i));
        }
    }
    pins
}

pub fn export_vcd(logic: &LogicAnalyzer, trace: &TraceBuffer, cfg: &VcdConfig) -> String {
    let pins = resolve_pins(logic, cfg);
    let mut out = String::with_capacity(4096);
    out.push_str("$date\n  2026-11-24\n$end\n");
    out.push_str("$version\n  ch585-emulator\n$end\n");
    out.push_str("$timescale 1ns $end\n");
    out.push_str("$scope module ch585 $end\n");

    let mut next_sig = 0usize;
    let mut pin_ids = Vec::with_capacity(pins.len());
    if !pins.is_empty() {
        out.push_str("$scope module gpio $end\n");
        for &(port, idx) in &pins {
            let id = vcd_id(next_sig);
            next_sig += 1;
            let _ = writeln!(out, "$var wire 1 {id} P{port}{idx} $end");
            pin_ids.push(id);
        }
        out.push_str("$upscope $end\n");
    }

    let internal_base = next_sig;
    if cfg.include_internal {
        out.push_str("$scope module irq $end\n");
        let _ = writeln!(out, "$var wire 8 {} irq_active $end", vcd_id(internal_base));
        let _ = writeln!(out, "$var wire 4 {} irq_depth $end", vcd_id(internal_base + 1));
        out.push_str("$upscope $end\n");

        out.push_str("$scope module dma $end\n");
        let _ = writeln!(out, "$var wire 8 {} dma_mask $end", vcd_id(internal_base + 2));
        for (i, name) in [
            "dma_tmr0", "dma_tmr1", "dma_tmr2", "dma_tmr3", "dma_adc", "dma_spi0", "dma_spi1",
            "dma_usb",
        ]
        .iter()
        .enumerate()
        {
            let _ = writeln!(out, "$var wire 1 {} {name} $end", vcd_id(internal_base + 3 + i));
        }
        out.push_str("$upscope $end\n");

        out.push_str("$scope module usb $end\n");
        for (i, name) in [
            "usb_attached",
            "usb_enumerated",
            "usb_suspended",
            "usb_in_tog",
            "usb_out_tog",
        ]
        .iter()
        .enumerate()
        {
            let _ = writeln!(out, "$var wire 1 {} {name} $end", vcd_id(internal_base + 11 + i));
        }
        let _ = writeln!(out, "$var wire 4 {} usb_ep $end", vcd_id(internal_base + 16));
        out.push_str("$upscope $end\n");

        out.push_str("$scope module power $end\n");
        let _ = writeln!(out, "$var wire 3 {} power_mode $end", vcd_id(internal_base + 17));
        out.push_str("$upscope $end\n");
    }

    out.push_str("$upscope $end\n");
    out.push_str("$enddefinitions $end\n");

    let mut l_idx = 0usize;
    let mut cur_pa = 0u16;
    let mut cur_pb = 0u32;
    while l_idx < logic.samples.len() && logic.samples[l_idx].time_ns == 0 {
        cur_pa = logic.samples[l_idx].pa;
        cur_pb = logic.samples[l_idx].pb;
        l_idx += 1;
    }

    let mut t_idx = 0usize;
    let mut cur_tr = InternalTraceSample::default();
    while t_idx < trace.samples.len() && trace.samples[t_idx].time_ns == 0 {
        cur_tr = trace.samples[t_idx];
        t_idx += 1;
    }

    out.push_str("#0\n$dumpvars\n");
    for (p_i, &(port, idx)) in pins.iter().enumerate() {
        let bit = read_pin_from_sample(cur_pa, cur_pb, port, idx);
        let _ = writeln!(out, "{bit}{}", pin_ids[p_i]);
    }
    if cfg.include_internal {
        write_internal_signals(&mut out, internal_base, &cur_tr, None);
    }
    out.push_str("$end\n");

    let mut last_pa = cur_pa;
    let mut last_pb = cur_pb;
    let mut last_tr = cur_tr;

    while l_idx < logic.samples.len() || t_idx < trace.samples.len() {
        let next_t = match (logic.samples.get(l_idx), trace.samples.get(t_idx)) {
            (Some(l), Some(t)) => l.time_ns.min(t.time_ns),
            (Some(l), None) => l.time_ns,
            (None, Some(t)) => t.time_ns,
            (None, None) => break,
        };
        while l_idx < logic.samples.len() && logic.samples[l_idx].time_ns <= next_t {
            cur_pa = logic.samples[l_idx].pa;
            cur_pb = logic.samples[l_idx].pb;
            l_idx += 1;
        }
        while t_idx < trace.samples.len() && trace.samples[t_idx].time_ns <= next_t {
            cur_tr = trace.samples[t_idx];
            t_idx += 1;
        }

        let mut frame = String::new();
        for (p_i, &(port, idx)) in pins.iter().enumerate() {
            let bit = read_pin_from_sample(cur_pa, cur_pb, port, idx);
            let prev_bit = read_pin_from_sample(last_pa, last_pb, port, idx);
            if bit != prev_bit {
                let _ = writeln!(frame, "{bit}{}", pin_ids[p_i]);
            }
        }
        if cfg.include_internal {
            write_internal_signals(&mut frame, internal_base, &cur_tr, Some(&last_tr));
        }

        if !frame.is_empty() {
            let _ = writeln!(out, "#{next_t}");
            out.push_str(&frame);
        }
        last_pa = cur_pa;
        last_pb = cur_pb;
        last_tr = cur_tr;
    }

    out
}

fn write_internal_signals(
    out: &mut String,
    base: usize,
    cur: &InternalTraceSample,
    prev: Option<&InternalTraceSample>,
) {
    if prev.is_none_or(|p| p.active_irq != cur.active_irq) {
        let _ = writeln!(out, "b{:08b} {}", cur.active_irq, vcd_id(base));
    }
    if prev.is_none_or(|p| p.irq_depth != cur.irq_depth) {
        let _ = writeln!(out, "b{:04b} {}", cur.irq_depth & 0x0F, vcd_id(base + 1));
    }
    if prev.is_none_or(|p| p.dma_mask != cur.dma_mask) {
        let _ = writeln!(out, "b{:08b} {}", cur.dma_mask, vcd_id(base + 2));
        for bit in 0..8u8 {
            let cur_b = (cur.dma_mask >> bit) & 1;
            let changed = prev.is_none_or(|p| ((p.dma_mask >> bit) & 1) != cur_b);
            if changed {
                let _ = writeln!(out, "{cur_b}{}", vcd_id(base + 3 + bit as usize));
            }
        }
    }
    if prev.is_none_or(|p| p.usb_state != cur.usb_state) {
        for bit in 0..5u8 {
            let cur_b = (cur.usb_state >> bit) & 1;
            let changed = prev.is_none_or(|p| ((p.usb_state >> bit) & 1) != cur_b);
            if changed {
                let _ = writeln!(out, "{cur_b}{}", vcd_id(base + 11 + bit as usize));
            }
        }
    }
    if prev.is_none_or(|p| p.usb_ep != cur.usb_ep) {
        let _ = writeln!(out, "b{:04b} {}", cur.usb_ep & 0x0F, vcd_id(base + 16));
    }
    if prev.is_none_or(|p| p.power_mode != cur.power_mode) {
        let _ = writeln!(out, "b{:03b} {}", cur.power_mode & 0x07, vcd_id(base + 17));
    }
}
