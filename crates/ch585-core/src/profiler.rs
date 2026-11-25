use std::collections::HashMap;
use std::fmt::Write as _;

use crate::elf::{resolve_func_in, ElfImage, ElfSymbol};
use crate::json::JsonValue;
use crate::peripherals::irq;

pub fn irq_default_name(irqn: u8) -> &'static str {
    match irqn {
        irq::RESET => "RESET",
        irq::NMI => "NMI",
        irq::EXC => "EXC",
        irq::SYSTICK => "SysTick",
        irq::SWI => "SWI",
        irq::TMR0 => "TMR0",
        irq::GPIO_A => "GPIO_A",
        irq::GPIO_B => "GPIO_B",
        irq::SPI0 => "SPI0",
        irq::BLEB => "BLEB",
        irq::BLEL => "BLEL",
        irq::USB => "USB",
        irq::TMR1 => "TMR1",
        irq::TMR2 => "TMR2",
        irq::UART0 => "UART0",
        irq::UART1 => "UART1",
        irq::RTC => "RTC",
        irq::ADC => "ADC",
        irq::I2C => "I2C",
        irq::PWMX => "PWMX",
        irq::TMR3 => "TMR3",
        irq::UART2 => "UART2",
        irq::UART3 => "UART3",
        irq::WDOG_BAT => "WDOG_BAT",
        irq::NFC => "NFC",
        irq::USB2_DEVICE => "USB2_DEVICE",
        irq::USB2_HOST => "USB2_HOST",
        irq::LED => "LED",
        _ => "IRQ",
    }
}

#[derive(Debug, Clone)]
pub struct FuncProfileEntry {
    pub addr: u32,
    pub name: String,
    pub calls: u64,
    pub self_cycles: u64,
    pub self_ns: u64,
}

#[derive(Debug, Clone)]
pub struct IsrProfileEntry {
    pub irqn: u8,
    pub name: String,
    pub count: u64,
    pub total_cycles: u64,
    pub total_ns: u64,
    pub max_cycles: u64,
    pub max_ns: u64,
}

#[derive(Debug, Clone)]
pub struct LongestIsr {
    pub irqn: u8,
    pub name: String,
    pub cycles: u64,
    pub ns: u64,
    pub at_ns: u64,
}

#[derive(Debug, Clone)]
pub struct StackOverflowEvent {
    pub time_ns: u64,
    pub pc: u32,
    pub sp: u32,
    pub limit: u32,
}

#[derive(Debug, Clone)]
pub struct StackProfile {
    pub top: u32,
    pub limit: u32,
    pub min_sp: u32,
    pub min_sp_pc: u32,
    pub min_sp_sym: String,
    pub max_used_bytes: u32,
    pub overflow: bool,
    pub overflow_events: Vec<StackOverflowEvent>,
}

#[derive(Debug, Clone)]
pub struct ProfilerReport {
    pub total_cycles: u64,
    pub total_ns: u64,
    pub stack: StackProfile,
    pub longest_isr: Option<LongestIsr>,
    pub isrs: Vec<IsrProfileEntry>,
    pub functions: Vec<FuncProfileEntry>,
}

impl ProfilerReport {
    pub fn format_text(&self, top_n: usize) -> String {
        let mut out = String::new();
        let _ = writeln!(
            out,
            "=== PROFILE ===\nTotalCycles={} TotalNs={} StackTop=0x{:08X} StackLimit=0x{:08X} MinSP=0x{:08X} (@ 0x{:08X} {}) MaxStackUsed={}B Overflow={}",
            self.total_cycles,
            self.total_ns,
            self.stack.top,
            self.stack.limit,
            self.stack.min_sp,
            self.stack.min_sp_pc,
            self.stack.min_sp_sym,
            self.stack.max_used_bytes,
            self.stack.overflow,
        );
        if self.stack.overflow {
            for ev in self.stack.overflow_events.iter().take(4) {
                let _ = writeln!(
                    out,
                    "  STACK OVERFLOW WARN: t={}ns PC=0x{:08X} SP=0x{:08X} < Limit=0x{:08X}",
                    ev.time_ns, ev.pc, ev.sp, ev.limit
                );
            }
        }
        if let Some(l) = &self.longest_isr {
            let _ = writeln!(
                out,
                "LongestISR: IRQ{} ({}) {} cycles ({} ns) @ {} ns",
                l.irqn, l.name, l.cycles, l.ns, l.at_ns
            );
        }
        if !self.isrs.is_empty() {
            out.push_str("Top ISRs:\n");
            for isr in self.isrs.iter().take(top_n) {
                let _ = writeln!(
                    out,
                    "  IRQ{:<2} {:<24} count={} total_cyc={} max_cyc={} ({} ns)",
                    isr.irqn, isr.name, isr.count, isr.total_cycles, isr.max_cycles, isr.max_ns
                );
            }
        }
        if !self.functions.is_empty() {
            out.push_str("Top Functions:\n");
            let denom = self.total_cycles.max(1) as f64;
            for f in self.functions.iter().take(top_n) {
                let pct = (f.self_cycles as f64) * 100.0 / denom;
                let _ = writeln!(
                    out,
                    "  0x{:08X} {:<32} calls={:<6} cyc={:<9} ({:>5.1}%)",
                    f.addr, f.name, f.calls, f.self_cycles, pct
                );
            }
        }
        out
    }

    pub fn to_json_value(&self, top_n: Option<usize>) -> JsonValue {
        let limit_fn = top_n.unwrap_or(self.functions.len());
        let longest = match &self.longest_isr {
            Some(l) => JsonValue::Object(vec![
                ("irqn".into(), JsonValue::Number(l.irqn as f64)),
                ("name".into(), JsonValue::Str(l.name.clone())),
                ("cycles".into(), JsonValue::Number(l.cycles as f64)),
                ("ns".into(), JsonValue::Number(l.ns as f64)),
                ("at_ns".into(), JsonValue::Number(l.at_ns as f64)),
            ]),
            None => JsonValue::Null,
        };
        let stack = JsonValue::Object(vec![
            ("top".into(), JsonValue::Str(format!("0x{:08X}", self.stack.top))),
            ("limit".into(), JsonValue::Str(format!("0x{:08X}", self.stack.limit))),
            ("min_sp".into(), JsonValue::Str(format!("0x{:08X}", self.stack.min_sp))),
            ("min_sp_pc".into(), JsonValue::Str(format!("0x{:08X}", self.stack.min_sp_pc))),
            ("min_sp_sym".into(), JsonValue::Str(self.stack.min_sp_sym.clone())),
            ("max_used_bytes".into(), JsonValue::Number(self.stack.max_used_bytes as f64)),
            ("overflow".into(), JsonValue::Bool(self.stack.overflow)),
            (
                "overflow_events".into(),
                JsonValue::Array(
                    self.stack
                        .overflow_events
                        .iter()
                        .map(|ev| {
                            JsonValue::Object(vec![
                                ("time_ns".into(), JsonValue::Number(ev.time_ns as f64)),
                                ("pc".into(), JsonValue::Str(format!("0x{:08X}", ev.pc))),
                                ("sp".into(), JsonValue::Str(format!("0x{:08X}", ev.sp))),
                                ("limit".into(), JsonValue::Str(format!("0x{:08X}", ev.limit))),
                            ])
                        })
                        .collect(),
                ),
            ),
        ]);
        let isrs = JsonValue::Array(
            self.isrs
                .iter()
                .map(|i| {
                    JsonValue::Object(vec![
                        ("irqn".into(), JsonValue::Number(i.irqn as f64)),
                        ("name".into(), JsonValue::Str(i.name.clone())),
                        ("count".into(), JsonValue::Number(i.count as f64)),
                        ("total_cycles".into(), JsonValue::Number(i.total_cycles as f64)),
                        ("total_ns".into(), JsonValue::Number(i.total_ns as f64)),
                        ("max_cycles".into(), JsonValue::Number(i.max_cycles as f64)),
                        ("max_ns".into(), JsonValue::Number(i.max_ns as f64)),
                    ])
                })
                .collect(),
        );
        let funcs = JsonValue::Array(
            self.functions
                .iter()
                .take(limit_fn)
                .map(|f| {
                    JsonValue::Object(vec![
                        ("addr".into(), JsonValue::Str(format!("0x{:08X}", f.addr))),
                        ("name".into(), JsonValue::Str(f.name.clone())),
                        ("calls".into(), JsonValue::Number(f.calls as f64)),
                        ("self_cycles".into(), JsonValue::Number(f.self_cycles as f64)),
                        ("self_ns".into(), JsonValue::Number(f.self_ns as f64)),
                    ])
                })
                .collect(),
        );
        JsonValue::Object(vec![
            ("total_cycles".into(), JsonValue::Number(self.total_cycles as f64)),
            ("total_ns".into(), JsonValue::Number(self.total_ns as f64)),
            ("stack".into(), stack),
            ("longest_isr".into(), longest),
            ("isrs".into(), isrs),
            ("functions".into(), funcs),
        ])
    }
}

#[derive(Debug, Clone)]
struct FuncAcc {
    name: String,
    calls: u64,
    self_cycles: u64,
    self_ns: u64,
}

#[derive(Debug, Clone)]
struct IsrAcc {
    name: String,
    count: u64,
    total_cycles: u64,
    total_ns: u64,
    max_cycles: u64,
    max_ns: u64,
}

#[derive(Debug, Clone)]
struct ActiveIsrSpan {
    irqn: u8,
    name: String,
    start_cycles: u64,
    start_ns: u64,
}

/// ELF symbol, ISR latency, and stack depth profiler.
#[derive(Debug, Clone)]
pub struct Profiler {
    pub enabled: bool,
    symbols: Vec<ElfSymbol>,
    sym_name_by_addr: HashMap<u32, String>,
    stack_top: u32,
    stack_limit: u32,
    stack_limit_override: Option<u32>,
    min_sp: u32,
    min_sp_pc: u32,
    overflow_events: Vec<StackOverflowEvent>,
    func_acc: HashMap<u32, FuncAcc>,
    isr_acc: HashMap<u8, IsrAcc>,
    active_isrs: Vec<ActiveIsrSpan>,
    longest_isr: Option<LongestIsr>,
    total_cycles: u64,
    total_ns: u64,
    pending_call: bool,
}

impl Default for Profiler {
    fn default() -> Self {
        Self::new()
    }
}

impl Profiler {
    pub fn new() -> Self {
        Self {
            enabled: false,
            symbols: Vec::new(),
            sym_name_by_addr: HashMap::new(),
            stack_top: 0x2002_0000,
            stack_limit: 0x2000_2000,
            stack_limit_override: None,
            min_sp: 0x2002_0000,
            min_sp_pc: 0,
            overflow_events: Vec::new(),
            func_acc: HashMap::new(),
            isr_acc: HashMap::new(),
            active_isrs: Vec::new(),
            longest_isr: None,
            total_cycles: 0,
            total_ns: 0,
            pending_call: false,
        }
    }

    pub fn load_symbols(&mut self, elf: &ElfImage) {
        self.symbols = elf.symbols.clone();
        self.sym_name_by_addr.clear();
        for (addr, names) in &elf.sym_by_addr {
            if let Some(first) = names.first() {
                self.sym_name_by_addr.insert(*addr, first.clone());
            }
        }
        let (top, limit) = elf.detect_stack_bounds();
        self.stack_top = top.unwrap_or(0x2002_0000);
        if let Some(ov) = self.stack_limit_override {
            self.stack_limit = ov;
        } else {
            self.stack_limit = limit.unwrap_or(0x2000_2000);
        }
        self.reset();
    }

    pub fn set_stack_bounds(&mut self, top: u32, limit: u32) {
        self.stack_top = top;
        self.stack_limit = limit;
        self.stack_limit_override = Some(limit);
        if self.total_cycles == 0 {
            self.min_sp = top;
        }
    }

    pub fn reset(&mut self) {
        self.min_sp = self.stack_top;
        self.min_sp_pc = 0;
        self.overflow_events.clear();
        self.func_acc.clear();
        self.isr_acc.clear();
        self.active_isrs.clear();
        self.longest_isr = None;
        self.total_cycles = 0;
        self.total_ns = 0;
        self.pending_call = false;
    }

    #[inline]
    pub fn mark_pending_call(&mut self) {
        self.pending_call = true;
    }

    #[inline]
    pub fn take_pending_call(&mut self) -> bool {
        let was = self.pending_call;
        self.pending_call = false;
        was
    }

    fn resolve_bucket(&self, pc: u32) -> (u32, String) {
        if let Some(sym) = resolve_func_in(&self.symbols, pc) {
            return (sym.addr, sym.name.clone());
        }
        if let Some(name) = self.sym_name_by_addr.get(&pc) {
            return (pc, name.clone());
        }
        if self.symbols.is_empty() {
            return (pc & !0xF, format!("0x{:08X}", pc & !0xF));
        }
        let bucket = pc & !0xFF;
        (bucket, format!("0x{bucket:08X}"))
    }

    pub fn on_block(&mut self, pc: u32, sp: u32, cycles: u64, ns: u64, is_call: bool) {
        if !self.enabled {
            return;
        }
        self.total_cycles += cycles;
        self.total_ns += ns;

        let (b_addr, b_name) = self.resolve_bucket(pc);
        let entry = self.func_acc.entry(b_addr).or_insert_with(|| FuncAcc {
            name: b_name,
            calls: 0,
            self_cycles: 0,
            self_ns: 0,
        });
        entry.self_cycles += cycles;
        entry.self_ns += ns;
        if is_call {
            entry.calls += 1;
        }

        if (0x2000_0000..=0x2002_0000).contains(&sp) && sp <= self.stack_top {
            if sp < self.min_sp {
                self.min_sp = sp;
                self.min_sp_pc = pc;
            }
            if sp < self.stack_limit && self.overflow_events.len() < 64 {
                let record = self
                    .overflow_events
                    .last()
                    .is_none_or(|prev| sp < prev.sp || pc != prev.pc);
                if record {
                    self.overflow_events.push(StackOverflowEvent {
                        time_ns: self.total_ns,
                        pc,
                        sp,
                        limit: self.stack_limit,
                    });
                }
            }
        }
    }

    pub fn on_irq_enter(&mut self, irqn: u8, entry_pc: u32, cycles: u64, ns: u64) {
        if !self.enabled {
            return;
        }
        let name = if let Some(sym) = resolve_func_in(&self.symbols, entry_pc) {
            sym.name.clone()
        } else if let Some(exact) = self.sym_name_by_addr.get(&entry_pc) {
            exact.clone()
        } else {
            irq_default_name(irqn).to_string()
        };
        let acc = self.isr_acc.entry(irqn).or_insert_with(|| IsrAcc {
            name: name.clone(),
            count: 0,
            total_cycles: 0,
            total_ns: 0,
            max_cycles: 0,
            max_ns: 0,
        });
        acc.count += 1;
        self.active_isrs.push(ActiveIsrSpan {
            irqn,
            name,
            start_cycles: cycles,
            start_ns: ns,
        });
    }

    pub fn on_irq_exit(&mut self, irqn: u8, cycles: u64, ns: u64) {
        if !self.enabled {
            return;
        }
        let Some(pos) = self.active_isrs.iter().rposition(|s| s.irqn == irqn) else {
            return;
        };
        let span = self.active_isrs.remove(pos);
        let dur_c = cycles.saturating_sub(span.start_cycles).max(1);
        let dur_ns = ns.saturating_sub(span.start_ns);
        if let Some(acc) = self.isr_acc.get_mut(&irqn) {
            acc.total_cycles += dur_c;
            acc.total_ns += dur_ns;
            if dur_c > acc.max_cycles {
                acc.max_cycles = dur_c;
            }
            if dur_ns > acc.max_ns {
                acc.max_ns = dur_ns;
            }
        }
        let replace = self
            .longest_isr
            .as_ref()
            .is_none_or(|prev| dur_c > prev.cycles || (dur_c == prev.cycles && dur_ns > prev.ns));
        if replace {
            self.longest_isr = Some(LongestIsr {
                irqn,
                name: span.name,
                cycles: dur_c,
                ns: dur_ns,
                at_ns: span.start_ns,
            });
        }
    }

    pub fn report(&self) -> ProfilerReport {
        let mut functions: Vec<FuncProfileEntry> = self
            .func_acc
            .iter()
            .map(|(&addr, acc)| FuncProfileEntry {
                addr,
                name: acc.name.clone(),
                calls: acc.calls,
                self_cycles: acc.self_cycles,
                self_ns: acc.self_ns,
            })
            .collect();
        functions.sort_by(|a, b| {
            b.self_cycles
                .cmp(&a.self_cycles)
                .then_with(|| a.addr.cmp(&b.addr))
        });

        let mut isrs: Vec<IsrProfileEntry> = self
            .isr_acc
            .iter()
            .map(|(&irqn, acc)| IsrProfileEntry {
                irqn,
                name: acc.name.clone(),
                count: acc.count,
                total_cycles: acc.total_cycles,
                total_ns: acc.total_ns,
                max_cycles: acc.max_cycles,
                max_ns: acc.max_ns,
            })
            .collect();
        isrs.sort_by(|a, b| {
            b.total_cycles
                .cmp(&a.total_cycles)
                .then_with(|| a.irqn.cmp(&b.irqn))
        });

        let min_sp_sym = if self.min_sp_pc == 0 {
            "<none>".to_string()
        } else {
            self.resolve_bucket(self.min_sp_pc).1
        };
        let max_used_bytes = if self.min_sp <= self.stack_top {
            self.stack_top.saturating_sub(self.min_sp)
        } else {
            0
        };

        ProfilerReport {
            total_cycles: self.total_cycles,
            total_ns: self.total_ns,
            stack: StackProfile {
                top: self.stack_top,
                limit: self.stack_limit,
                min_sp: self.min_sp,
                min_sp_pc: self.min_sp_pc,
                min_sp_sym,
                max_used_bytes,
                overflow: !self.overflow_events.is_empty(),
                overflow_events: self.overflow_events.clone(),
            },
            longest_isr: self.longest_isr.clone(),
            isrs,
            functions,
        }
    }
}
