use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::peripherals::adc::{Adc, AdcWaveform};
use crate::peripherals::gpio::{ExtPinDrive, ExtPull, Gpio};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinAnalogModel {
    /// Output resistance of an external driver in Ohms (0 -> 1 Ohm stiff source)
    pub source_ohm: u32,
    /// Optional external resistive load in Ohms
    pub load_ohm: Option<u32>,
    /// True if `load_ohm` connects to VDD instead of GND
    pub load_to_vdd: bool,
    /// External pull resistor value in Ohms (default 100k)
    pub ext_pull_ohm: u32,
}

impl Default for PinAnalogModel {
    fn default() -> Self {
        Self {
            source_ohm: 0,
            load_ohm: None,
            load_to_vdd: false,
            ext_pull_ohm: 100_000,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinAnalogTable {
    pub pa: [PinAnalogModel; 16],
    pub pb: [PinAnalogModel; 24],
}

impl Default for PinAnalogTable {
    fn default() -> Self {
        Self {
            pa: [PinAnalogModel::default(); 16],
            pb: [PinAnalogModel::default(); 24],
        }
    }
}

impl PinAnalogTable {
    pub fn get(&self, port: char, idx: u8) -> PinAnalogModel {
        match port {
            'A' | 'a' if (idx as usize) < 16 => self.pa[idx as usize],
            'B' | 'b' if (idx as usize) < 24 => self.pb[idx as usize],
            _ => PinAnalogModel::default(),
        }
    }

    pub fn set(&mut self, port: char, idx: u8, model: PinAnalogModel) {
        match port {
            'A' | 'a' if (idx as usize) < 16 => self.pa[idx as usize] = model,
            'B' | 'b' if (idx as usize) < 24 => self.pb[idx as usize] = model,
            _ => {}
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinWatchMask {
    pub pa: u16,
    pub pb: u32,
}

impl PinWatchMask {
    pub const NONE: Self = Self { pa: 0, pb: 0 };
    pub const ALL: Self = Self {
        pa: 0xFFFF,
        pb: 0x00FF_FFFF,
    };

    #[inline]
    pub const fn with_pa(mut self, idx: u8) -> Self {
        if idx < 16 {
            self.pa |= 1u16 << idx;
        }
        self
    }

    #[inline]
    pub const fn with_pb(mut self, idx: u8) -> Self {
        if idx < 24 {
            self.pb |= 1u32 << idx;
        }
        self
    }

    #[inline]
    pub const fn with_pin(self, port: char, idx: u8) -> Self {
        match port {
            'A' | 'a' => self.with_pa(idx),
            'B' | 'b' => self.with_pb(idx),
            _ => self,
        }
    }

    #[inline]
    pub const fn union(self, other: Self) -> Self {
        Self {
            pa: self.pa | other.pa,
            pb: self.pb | other.pb,
        }
    }

    #[inline]
    pub const fn contains(self, port: char, idx: u8) -> bool {
        match port {
            'A' | 'a' if idx < 16 => (self.pa & (1u16 << idx)) != 0,
            'B' | 'b' if idx < 24 => (self.pb & (1u32 << idx)) != 0,
            _ => false,
        }
    }

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.pa == 0 && (self.pb & 0x00FF_FFFF) == 0
    }
}

impl Default for PinWatchMask {
    fn default() -> Self {
        Self::ALL
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputReadKind {
    GpioA,
    GpioB,
    Adc,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InputReadMask(u8);

impl InputReadMask {
    pub const NONE: Self = Self(0);
    pub const GPIO_A: Self = Self(1 << 0);
    pub const GPIO_B: Self = Self(1 << 1);
    pub const ADC: Self = Self(1 << 2);
    pub const ALL: Self = Self(0b111);

    #[inline]
    pub const fn bits(self) -> u8 {
        self.0
    }

    #[inline]
    pub const fn union(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }

    #[inline]
    pub const fn contains(self, kind: InputReadKind) -> bool {
        let bit = match kind {
            InputReadKind::GpioA => Self::GPIO_A.0,
            InputReadKind::GpioB => Self::GPIO_B.0,
            InputReadKind::Adc => Self::ADC.0,
        };
        (self.0 & bit) != 0
    }

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinChangeEvent {
    pub time_ns: u64,
    pub cycle: u64,
    pub port: char,
    pub idx: u8,
    pub level: bool,
    pub is_output: bool,
    pub output_val: bool,
    pub pull_up: bool,
    pub pull_down: bool,
    pub high_drive_20ma: bool,
    pub voltage_mv: i32,
}

enum TimerAction {
    Device { id: usize, token: u64 },
    Callback(Box<dyn FnOnce(&mut PinContext<'_>)>),
}

struct ScheduledTimer {
    at_ns: u64,
    seq: u64,
    action: TimerAction,
}

impl PartialEq for ScheduledTimer {
    fn eq(&self, other: &Self) -> bool {
        self.at_ns == other.at_ns && self.seq == other.seq
    }
}
impl Eq for ScheduledTimer {}
impl PartialOrd for ScheduledTimer {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for ScheduledTimer {
    fn cmp(&self, other: &Self) -> Ordering {
        // Min-heap by earliest timestamp, then FIFO seq
        other
            .at_ns
            .cmp(&self.at_ns)
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

#[derive(Default)]
pub struct TimerQueue {
    heap: BinaryHeap<ScheduledTimer>,
    next_seq: u64,
}

impl TimerQueue {
    pub fn clear(&mut self) {
        self.heap.clear();
    }

    pub fn next_ns(&self) -> Option<u64> {
        self.heap.peek().map(|t| t.at_ns)
    }

    pub fn cancel_device_token(&mut self, device_id: usize, token: u64) {
        self.heap.retain(|t| {
            !matches!(
                t.action,
                TimerAction::Device { id, token: tk } if id == device_id && tk == token
            )
        });
    }

    fn push(&mut self, at_ns: u64, action: TimerAction) {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        self.heap.push(ScheduledTimer { at_ns, seq, action });
    }

    fn pop_due(&mut self, time_ns: u64) -> Option<ScheduledTimer> {
        if self.heap.peek().is_some_and(|t| t.at_ns <= time_ns) {
            self.heap.pop()
        } else {
            None
        }
    }
}

pub struct PinContext<'a> {
    time_ns: u64,
    cycle: u64,
    vdd_mv: i32,
    current_device_id: Option<usize>,
    gpio: &'a mut Gpio,
    adc: &'a mut Adc,
    timers: &'a mut TimerQueue,
    analog: &'a mut PinAnalogTable,
}

impl<'a> PinContext<'a> {
    pub fn time_ns(&self) -> u64 {
        self.time_ns
    }

    pub fn cycle(&self) -> u64 {
        self.cycle
    }

    pub fn vdd_mv(&self) -> i32 {
        self.vdd_mv
    }

    pub fn gpio(&self) -> &Gpio {
        self.gpio
    }

    pub fn adc(&self) -> &Adc {
        self.adc
    }

    #[inline]
    pub fn pin_is_output(&self, port: char, idx: u8) -> bool {
        match port {
            'A' | 'a' if idx < 16 => {
                let bit = 1u16 << idx;
                ((self.gpio.pa_dir | self.gpio.periph_out_pa_mask) & bit) != 0
            }
            'B' | 'b' if idx < 24 => {
                let bit = 1u32 << idx;
                ((self.gpio.pb_dir | self.gpio.periph_out_pb_mask) & bit) != 0
            }
            _ => false,
        }
    }

    #[inline]
    pub fn mcu_output_level(&self, port: char, idx: u8) -> bool {
        match port {
            'A' | 'a' if idx < 16 => {
                let bit = 1u16 << idx;
                if (self.gpio.periph_out_pa_mask & bit) != 0 {
                    (self.gpio.periph_out_pa_val & bit) != 0
                } else if (self.gpio.pa_dir & bit) != 0 {
                    (self.gpio.pa_out & bit) != 0
                } else {
                    match self.gpio.ext_pull_pa[idx as usize] {
                        ExtPull::PullUp => true,
                        ExtPull::PullDown => false,
                        ExtPull::None => {
                            (self.gpio.pa_pu & bit) != 0 && (self.gpio.pa_pd_drv & bit) == 0
                        }
                    }
                }
            }
            'B' | 'b' if idx < 24 => {
                let bit = 1u32 << idx;
                if (self.gpio.periph_out_pb_mask & bit) != 0 {
                    (self.gpio.periph_out_pb_val & bit) != 0
                } else if (self.gpio.pb_dir & bit) != 0 {
                    (self.gpio.pb_out & bit) != 0
                } else {
                    match self.gpio.ext_pull_pb[idx as usize] {
                        ExtPull::PullUp => true,
                        ExtPull::PullDown => false,
                        ExtPull::None => {
                            (self.gpio.pb_pu & bit) != 0 && (self.gpio.pb_pd_drv & bit) == 0
                        }
                    }
                }
            }
            _ => false,
        }
    }

    pub fn read_pin(&self, port: char, idx: u8) -> bool {
        self.gpio.pin_level(port, idx)
    }

    pub fn pin_voltage_mv(&self, port: char, idx: u8) -> i32 {
        let model = self.analog.get(port, idx);
        compute_pin_voltage_mv(port, idx, self.vdd_mv, self.gpio, model)
    }

    pub fn set_pin_drive(&mut self, port: char, idx: u8, drive: ExtPinDrive) {
        self.gpio.set_ext_drive(port, idx, drive);
    }

    pub fn set_pin_pull(&mut self, port: char, idx: u8, pull: ExtPull) {
        self.gpio.set_ext_pull(port, idx, pull);
    }

    pub fn set_pin_analog_model(&mut self, port: char, idx: u8, model: PinAnalogModel) {
        self.analog.set(port, idx, model);
    }

    pub fn set_adc_mv(&mut self, ch: usize, mv: i32) {
        if ch < 14 {
            self.adc.channels[ch] = AdcWaveform::Dc(mv);
        }
    }

    pub fn set_adc_wave(&mut self, ch: usize, wave: AdcWaveform) {
        if ch < 14 {
            self.adc.channels[ch] = wave;
        }
    }

    pub fn set_vbat_mv(&mut self, mv: i32) {
        self.adc.vbat_mv = mv;
    }

    pub fn schedule_at_ns(&mut self, time_ns: u64, token: u64) {
        if let Some(id) = self.current_device_id {
            self.timers.push(time_ns, TimerAction::Device { id, token });
        }
    }

    pub fn schedule_after_ns(&mut self, delay_ns: u64, token: u64) {
        self.schedule_at_ns(self.time_ns.saturating_add(delay_ns), token);
    }

    pub fn schedule_after_us(&mut self, delay_us: u64, token: u64) {
        self.schedule_after_ns(delay_us.saturating_mul(1_000), token);
    }

    pub fn cancel_timer(&mut self, token: u64) {
        if let Some(id) = self.current_device_id {
            self.timers.cancel_device_token(id, token);
        }
    }

    pub fn reschedule_at_ns(&mut self, time_ns: u64, token: u64) {
        if let Some(id) = self.current_device_id {
            self.timers.cancel_device_token(id, token);
            self.timers.push(time_ns, TimerAction::Device { id, token });
        }
    }

    pub fn reschedule_after_ns(&mut self, delay_ns: u64, token: u64) {
        self.reschedule_at_ns(self.time_ns.saturating_add(delay_ns), token);
    }

    pub fn reschedule_after_us(&mut self, delay_us: u64, token: u64) {
        self.reschedule_after_ns(delay_us.saturating_mul(1_000), token);
    }
}

pub trait ExternalDevice {
    fn watched_pins(&self) -> PinWatchMask {
        PinWatchMask::ALL
    }
    fn watched_input_reads(&self) -> InputReadMask {
        InputReadMask::NONE
    }
    fn next_event_ns(&self) -> Option<u64> {
        None
    }
    fn on_attach(&mut self, _ctx: &mut PinContext<'_>) {}
    fn on_pin_change(&mut self, _ev: PinChangeEvent, _ctx: &mut PinContext<'_>) {}
    fn on_before_input_read(&mut self, _kind: InputReadKind, _ctx: &mut PinContext<'_>) {}
    fn on_timer(&mut self, _token: u64, _ctx: &mut PinContext<'_>) {}
    fn on_time_reached(&mut self, _ctx: &mut PinContext<'_>) {}
}

type PinListener = Box<dyn FnMut(PinChangeEvent, &mut PinContext<'_>)>;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct GpioSnap {
    pa_eff: u16,
    pa_out_en: u16,
    pa_out_val: u16,
    pa_pu: u16,
    pa_pd_drv: u16,
    pb_eff: u32,
    pb_out_en: u32,
    pb_out_val: u32,
    pb_pu: u32,
    pb_pd_drv: u32,
}

impl GpioSnap {
    fn from_gpio(gpio: &Gpio) -> Self {
        let pa_out_en = gpio.pa_dir | gpio.periph_out_pa_mask;
        let pa_out_val = (gpio.pa_out & !gpio.periph_out_pa_mask)
            | (gpio.periph_out_pa_val & gpio.periph_out_pa_mask);
        let pb_out_en = gpio.pb_dir | gpio.periph_out_pb_mask;
        let pb_out_val = (gpio.pb_out & !gpio.periph_out_pb_mask)
            | (gpio.periph_out_pb_val & gpio.periph_out_pb_mask);
        Self {
            pa_eff: gpio.effective_pa_pin(),
            pa_out_en,
            pa_out_val,
            pa_pu: gpio.pa_pu,
            pa_pd_drv: gpio.pa_pd_drv,
            pb_eff: gpio.effective_pb_pin(),
            pb_out_en,
            pb_out_val,
            pb_pu: gpio.pb_pu,
            pb_pd_drv: gpio.pb_pd_drv,
        }
    }
}

pub struct ExtHost {
    devices: Vec<Option<Box<dyn ExternalDevice>>>,
    device_watches: Vec<PinWatchMask>,
    device_input_reads: Vec<InputReadMask>,
    devices_watch_mask: PinWatchMask,
    devices_input_read_mask: InputReadMask,
    pin_listeners: Vec<Option<PinListener>>,
    timers: TimerQueue,
    pub analog: PinAnalogTable,
    prev_snap: GpioSnap,
    last_gpio_version: u64,
}

impl Default for ExtHost {
    fn default() -> Self {
        Self::new(&Gpio::new())
    }
}

impl ExtHost {
    pub fn new(initial_gpio: &Gpio) -> Self {
        Self {
            devices: Vec::new(),
            device_watches: Vec::new(),
            device_input_reads: Vec::new(),
            devices_watch_mask: PinWatchMask::NONE,
            devices_input_read_mask: InputReadMask::NONE,
            pin_listeners: Vec::new(),
            timers: TimerQueue::default(),
            analog: PinAnalogTable::default(),
            prev_snap: GpioSnap::from_gpio(initial_gpio),
            last_gpio_version: initial_gpio.version,
        }
    }

    pub fn sync_baseline(&mut self, gpio: &Gpio) {
        self.prev_snap = GpioSnap::from_gpio(gpio);
        self.last_gpio_version = gpio.version;
    }

    pub fn clear_timers(&mut self) {
        self.timers.clear();
    }

    pub fn next_timer_ns(&self) -> Option<u64> {
        self.timers.next_ns()
    }

    pub fn next_event_ns(&self) -> Option<u64> {
        self.next_event_ns_with(None)
    }

    pub fn next_event_ns_with<'b>(&self, scoped: Option<&(dyn ExternalDevice + 'b)>) -> Option<u64> {
        let mut best = self.timers.next_ns();
        for dev in self.devices.iter().flatten() {
            if let Some(t) = dev.next_event_ns() {
                best = Some(best.map_or(t, |b| b.min(t)));
            }
        }
        if let Some(dev) = scoped {
            if let Some(t) = dev.next_event_ns() {
                best = Some(best.map_or(t, |b| b.min(t)));
            }
        }
        best
    }

    #[inline]
    pub fn devices_input_read_mask(&self) -> InputReadMask {
        self.devices_input_read_mask
    }

    fn rebuild_combined_masks(&mut self) {
        let mut watch = PinWatchMask::NONE;
        for (idx, dev_opt) in self.devices.iter().enumerate() {
            if dev_opt.is_some() {
                if let Some(&w) = self.device_watches.get(idx) {
                    watch = watch.union(w);
                }
            }
        }
        let mut iread = InputReadMask::NONE;
        for (idx, dev_opt) in self.devices.iter().enumerate() {
            if dev_opt.is_some() {
                if let Some(&r) = self.device_input_reads.get(idx) {
                    iread = iread.union(r);
                }
            }
        }
        self.devices_watch_mask = watch;
        self.devices_input_read_mask = iread;
    }

    pub fn refresh_device_masks(&mut self) {
        self.device_watches.resize(self.devices.len(), PinWatchMask::ALL);
        self.device_input_reads
            .resize(self.devices.len(), InputReadMask::NONE);
        for (idx, dev_opt) in self.devices.iter().enumerate() {
            if let Some(dev) = dev_opt {
                self.device_watches[idx] = dev.watched_pins();
                self.device_input_reads[idx] = dev.watched_input_reads();
            } else {
                self.device_watches[idx] = PinWatchMask::NONE;
                self.device_input_reads[idx] = InputReadMask::NONE;
            }
        }
        self.rebuild_combined_masks();
    }

    pub fn attach_device(
        &mut self,
        mut dev: Box<dyn ExternalDevice>,
        time_ns: u64,
        cycle: u64,
        vdd_mv: i32,
        gpio: &mut Gpio,
        adc: &mut Adc,
    ) -> usize {
        let id = self.devices.len();
        self.devices.push(None);
        self.device_watches.push(PinWatchMask::ALL);
        self.device_input_reads.push(InputReadMask::NONE);
        let mut ctx = PinContext {
            time_ns,
            cycle,
            vdd_mv,
            current_device_id: Some(id),
            gpio,
            adc,
            timers: &mut self.timers,
            analog: &mut self.analog,
        };
        dev.on_attach(&mut ctx);
        let watch = dev.watched_pins();
        let iread = dev.watched_input_reads();
        self.devices[id] = Some(dev);
        self.device_watches[id] = watch;
        self.device_input_reads[id] = iread;
        self.rebuild_combined_masks();
        self.poll(time_ns, cycle, vdd_mv, gpio, adc);
        id
    }

    pub fn reattach_all(
        &mut self,
        time_ns: u64,
        cycle: u64,
        vdd_mv: i32,
        gpio: &mut Gpio,
        adc: &mut Adc,
    ) {
        self.timers.clear();
        self.sync_baseline(gpio);
        self.device_watches.resize(self.devices.len(), PinWatchMask::ALL);
        self.device_input_reads
            .resize(self.devices.len(), InputReadMask::NONE);
        for id in 0..self.devices.len() {
            let Some(mut dev) = self.devices[id].take() else {
                self.device_watches[id] = PinWatchMask::NONE;
                self.device_input_reads[id] = InputReadMask::NONE;
                continue;
            };
            let mut ctx = PinContext {
                time_ns,
                cycle,
                vdd_mv,
                current_device_id: Some(id),
                gpio,
                adc,
                timers: &mut self.timers,
                analog: &mut self.analog,
            };
            dev.on_attach(&mut ctx);
            self.device_watches[id] = dev.watched_pins();
            self.device_input_reads[id] = dev.watched_input_reads();
            self.devices[id] = Some(dev);
        }
        self.rebuild_combined_masks();
    }

    pub fn on_pin_change(
        &mut self,
        cb: impl FnMut(PinChangeEvent, &mut PinContext<'_>) + 'static,
    ) -> usize {
        let id = self.pin_listeners.len();
        self.pin_listeners.push(Some(Box::new(cb)));
        id
    }

    pub fn schedule_at_ns_cb(
        &mut self,
        time_ns: u64,
        cb: impl FnOnce(&mut PinContext<'_>) + 'static,
    ) {
        self.timers.push(time_ns, TimerAction::Callback(Box::new(cb)));
    }

    pub fn schedule_device_timer(&mut self, device_id: usize, at_ns: u64, token: u64) {
        self.timers.push(
            at_ns,
            TimerAction::Device {
                id: device_id,
                token,
            },
        );
    }

    pub fn poll_force(
        &mut self,
        time_ns: u64,
        cycle: u64,
        vdd_mv: i32,
        gpio: &mut Gpio,
        adc: &mut Adc,
    ) -> bool {
        self.poll_force_with(time_ns, cycle, vdd_mv, gpio, adc, None)
    }

    pub fn poll_force_with<'a, 'b>(
        &mut self,
        time_ns: u64,
        cycle: u64,
        vdd_mv: i32,
        gpio: &mut Gpio,
        adc: &mut Adc,
        scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
    ) -> bool {
        gpio.rebuild_ext_masks();
        self.last_gpio_version = gpio.version.wrapping_sub(1);
        self.poll_with(time_ns, cycle, vdd_mv, gpio, adc, scoped)
    }

    /// Evaluate GPIO state changes and due timers, propagating combinational responses.
    /// Returns `true` if any pin event, timer callback, or scheduled deadline ran.
    pub fn poll(
        &mut self,
        time_ns: u64,
        cycle: u64,
        vdd_mv: i32,
        gpio: &mut Gpio,
        adc: &mut Adc,
    ) -> bool {
        self.poll_with(time_ns, cycle, vdd_mv, gpio, adc, None)
    }

    /// Same as [`ExtHost::poll`], additionally inspecting an unattached scoped device.
    pub fn poll_with<'a, 'b>(
        &mut self,
        time_ns: u64,
        cycle: u64,
        vdd_mv: i32,
        gpio: &mut Gpio,
        adc: &mut Adc,
        mut scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
    ) -> bool {
        let has_watchers =
            !self.devices.is_empty() || !self.pin_listeners.is_empty() || scoped.is_some();
        let timer_due = self.timers.next_ns().is_some_and(|t| t <= time_ns);
        let dev_due = self
            .devices
            .iter()
            .flatten()
            .any(|d| d.next_event_ns().is_some_and(|t| t <= time_ns))
            || scoped
                .as_deref()
                .and_then(|d| d.next_event_ns())
                .is_some_and(|t| t <= time_ns);

        // O(1) fast path when neither GPIO changed nor any external timer/deadline is due
        if !has_watchers && !timer_due && !dev_due {
            return false;
        }
        if !timer_due
            && !dev_due
            && gpio.version == self.last_gpio_version
            && GpioSnap::from_gpio(gpio) == self.prev_snap
        {
            return false;
        }

        let scoped_watch = scoped
            .as_deref()
            .map_or(PinWatchMask::NONE, |d| d.watched_pins());
        let combined_watch = if !self.pin_listeners.is_empty() {
            PinWatchMask::ALL
        } else {
            self.devices_watch_mask.union(scoped_watch)
        };

        let mut scoped_last_due: Option<u64> = None;
        let mut attached_last_due_inline: [Option<u64>; 16] = [None; 16];
        let mut attached_last_due_extra: Vec<Option<u64>> = if self.devices.len() > 16 {
            vec![None; self.devices.len() - 16]
        } else {
            Vec::new()
        };

        let mut any_activity = false;
        for _pass in 0..8 {
            let mut progressed = false;

            if has_watchers {
                let cur = GpioSnap::from_gpio(gpio);
                if cur != self.prev_snap {
                    let pa_mask = ((cur.pa_eff ^ self.prev_snap.pa_eff)
                        | (cur.pa_out_en ^ self.prev_snap.pa_out_en)
                        | (cur.pa_out_val ^ self.prev_snap.pa_out_val)
                        | (cur.pa_pu ^ self.prev_snap.pa_pu)
                        | (cur.pa_pd_drv ^ self.prev_snap.pa_pd_drv))
                        & combined_watch.pa;
                    let pb_mask = ((cur.pb_eff ^ self.prev_snap.pb_eff)
                        | (cur.pb_out_en ^ self.prev_snap.pb_out_en)
                        | (cur.pb_out_val ^ self.prev_snap.pb_out_val)
                        | (cur.pb_pu ^ self.prev_snap.pb_pu)
                        | (cur.pb_pd_drv ^ self.prev_snap.pb_pd_drv))
                        & combined_watch.pb;
                    self.prev_snap = cur;

                    if pa_mask != 0 {
                        let mut m = pa_mask;
                        while m != 0 {
                            let idx = m.trailing_zeros() as u8;
                            m &= m - 1;
                            let ev =
                                build_pin_event('A', idx, time_ns, cycle, vdd_mv, gpio, &self.analog);
                            self.emit_pin_event_with(
                                ev,
                                vdd_mv,
                                gpio,
                                adc,
                                scoped.as_deref_mut(),
                                scoped_watch,
                            );
                        }
                        progressed = true;
                    }
                    if pb_mask != 0 {
                        let mut m = pb_mask;
                        while m != 0 {
                            let idx = m.trailing_zeros() as u8;
                            m &= m - 1;
                            let ev =
                                build_pin_event('B', idx, time_ns, cycle, vdd_mv, gpio, &self.analog);
                            self.emit_pin_event_with(
                                ev,
                                vdd_mv,
                                gpio,
                                adc,
                                scoped.as_deref_mut(),
                                scoped_watch,
                            );
                        }
                        progressed = true;
                    }
                }
            } else {
                self.prev_snap = GpioSnap::from_gpio(gpio);
            }

            while let Some(timer) = self.timers.pop_due(time_ns) {
                progressed = true;
                match timer.action {
                    TimerAction::Device { id, token } => {
                        if id < self.devices.len() {
                            if let Some(mut dev) = self.devices[id].take() {
                                let mut ctx = PinContext {
                                    time_ns: timer.at_ns,
                                    cycle,
                                    vdd_mv,
                                    current_device_id: Some(id),
                                    gpio,
                                    adc,
                                    timers: &mut self.timers,
                                    analog: &mut self.analog,
                                };
                                dev.on_timer(token, &mut ctx);
                                self.devices[id] = Some(dev);
                            }
                        }
                    }
                    TimerAction::Callback(cb) => {
                        let mut ctx = PinContext {
                            time_ns: timer.at_ns,
                            cycle,
                            vdd_mv,
                            current_device_id: None,
                            gpio,
                            adc,
                            timers: &mut self.timers,
                            analog: &mut self.analog,
                        };
                        cb(&mut ctx);
                    }
                }
            }

            for id in 0..self.devices.len() {
                let due = self.devices[id]
                    .as_deref()
                    .and_then(|d| d.next_event_ns())
                    .filter(|&t| t <= time_ns);
                if let Some(due_ns) = due {
                    let slot = if id < 16 {
                        &mut attached_last_due_inline[id]
                    } else {
                        &mut attached_last_due_extra[id - 16]
                    };
                    if *slot != Some(due_ns) {
                        *slot = Some(due_ns);
                        if let Some(mut dev) = self.devices[id].take() {
                            let mut ctx = PinContext {
                                time_ns,
                                cycle,
                                vdd_mv,
                                current_device_id: Some(id),
                                gpio,
                                adc,
                                timers: &mut self.timers,
                                analog: &mut self.analog,
                            };
                            dev.on_time_reached(&mut ctx);
                            self.devices[id] = Some(dev);
                            progressed = true;
                        }
                    }
                }
            }

            if let Some(dev) = scoped.as_deref_mut() {
                if let Some(due_ns) = dev.next_event_ns().filter(|&t| t <= time_ns) {
                    if scoped_last_due != Some(due_ns) {
                        scoped_last_due = Some(due_ns);
                        let mut ctx = PinContext {
                            time_ns,
                            cycle,
                            vdd_mv,
                            current_device_id: None,
                            gpio,
                            adc,
                            timers: &mut self.timers,
                            analog: &mut self.analog,
                        };
                        dev.on_time_reached(&mut ctx);
                        progressed = true;
                    }
                }
            }

            if !progressed {
                break;
            }
            any_activity = true;
        }
        self.last_gpio_version = gpio.version;
        any_activity
    }

    pub fn notify_before_input_read_with<'a, 'b>(
        &mut self,
        kind: InputReadKind,
        time_ns: u64,
        cycle: u64,
        vdd_mv: i32,
        gpio: &mut Gpio,
        adc: &mut Adc,
        scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
    ) -> bool {
        let mut invoked = false;
        if self.devices_input_read_mask.contains(kind) {
            for id in 0..self.devices.len() {
                if !self.device_input_reads.get(id).is_some_and(|m| m.contains(kind)) {
                    continue;
                }
                let Some(mut dev) = self.devices[id].take() else {
                    continue;
                };
                let mut ctx = PinContext {
                    time_ns,
                    cycle,
                    vdd_mv,
                    current_device_id: Some(id),
                    gpio,
                    adc,
                    timers: &mut self.timers,
                    analog: &mut self.analog,
                };
                dev.on_before_input_read(kind, &mut ctx);
                self.devices[id] = Some(dev);
                invoked = true;
            }
        }
        if let Some(dev) = scoped {
            if dev.watched_input_reads().contains(kind) {
                let mut ctx = PinContext {
                    time_ns,
                    cycle,
                    vdd_mv,
                    current_device_id: None,
                    gpio,
                    adc,
                    timers: &mut self.timers,
                    analog: &mut self.analog,
                };
                dev.on_before_input_read(kind, &mut ctx);
                invoked = true;
            }
        }
        invoked
    }

    fn emit_pin_event_with<'a, 'b>(
        &mut self,
        ev: PinChangeEvent,
        vdd_mv: i32,
        gpio: &mut Gpio,
        adc: &mut Adc,
        scoped: Option<&'a mut (dyn ExternalDevice + 'b)>,
        scoped_watch: PinWatchMask,
    ) {
        for id in 0..self.devices.len() {
            if !self
                .device_watches
                .get(id)
                .is_some_and(|m| m.contains(ev.port, ev.idx))
            {
                continue;
            }
            let Some(mut dev) = self.devices[id].take() else {
                continue;
            };
            let mut ctx = PinContext {
                time_ns: ev.time_ns,
                cycle: ev.cycle,
                vdd_mv,
                current_device_id: Some(id),
                gpio,
                adc,
                timers: &mut self.timers,
                analog: &mut self.analog,
            };
            dev.on_pin_change(ev, &mut ctx);
            self.devices[id] = Some(dev);
        }
        if let Some(dev) = scoped {
            if scoped_watch.contains(ev.port, ev.idx) {
                let mut ctx = PinContext {
                    time_ns: ev.time_ns,
                    cycle: ev.cycle,
                    vdd_mv,
                    current_device_id: None,
                    gpio,
                    adc,
                    timers: &mut self.timers,
                    analog: &mut self.analog,
                };
                dev.on_pin_change(ev, &mut ctx);
            }
        }
        for id in 0..self.pin_listeners.len() {
            let Some(mut cb) = self.pin_listeners[id].take() else {
                continue;
            };
            let mut ctx = PinContext {
                time_ns: ev.time_ns,
                cycle: ev.cycle,
                vdd_mv,
                current_device_id: None,
                gpio,
                adc,
                timers: &mut self.timers,
                analog: &mut self.analog,
            };
            cb(ev, &mut ctx);
            self.pin_listeners[id] = Some(cb);
        }
    }
}

fn build_pin_event(
    port: char,
    idx: u8,
    time_ns: u64,
    cycle: u64,
    vdd_mv: i32,
    gpio: &Gpio,
    analog: &PinAnalogTable,
) -> PinChangeEvent {
    let model = analog.get(port, idx);
    let (level, is_output, output_val, pull_up, pull_down, high_drive_20ma) = if port == 'A' {
        let bit = 1u16 << idx;
        let out_en = (gpio.pa_dir & bit) != 0 || (gpio.periph_out_pa_mask & bit) != 0;
        let out_val = if (gpio.periph_out_pa_mask & bit) != 0 {
            (gpio.periph_out_pa_val & bit) != 0
        } else {
            (gpio.pa_out & bit) != 0
        };
        let drv = (gpio.pa_pd_drv & bit) != 0;
        (
            (gpio.effective_pa_pin() & bit) != 0,
            out_en,
            out_val,
            !out_en && (gpio.pa_pu & bit) != 0,
            !out_en && drv,
            out_en && drv,
        )
    } else {
        let bit = 1u32 << idx;
        let out_en = (gpio.pb_dir & bit) != 0 || (gpio.periph_out_pb_mask & bit) != 0;
        let out_val = if (gpio.periph_out_pb_mask & bit) != 0 {
            (gpio.periph_out_pb_val & bit) != 0
        } else {
            (gpio.pb_out & bit) != 0
        };
        let drv = (gpio.pb_pd_drv & bit) != 0;
        (
            (gpio.effective_pb_pin() & bit) != 0,
            out_en,
            out_val,
            !out_en && (gpio.pb_pu & bit) != 0,
            !out_en && drv,
            out_en && drv,
        )
    };
    PinChangeEvent {
        time_ns,
        cycle,
        port,
        idx,
        level,
        is_output,
        output_val,
        pull_up,
        pull_down,
        high_drive_20ma,
        voltage_mv: compute_pin_voltage_mv(port, idx, vdd_mv, gpio, model),
    }
}

/// Millman resistive divider solver combining MCU driver (5mA=100R, 20mA=25R, PU/PD=40k),
/// external pull resistor, external drive + source impedance, and optional pin load.
pub fn compute_pin_voltage_mv(
    port: char,
    idx: u8,
    vdd_mv: i32,
    gpio: &Gpio,
    model: PinAnalogModel,
) -> i32 {
    let vdd = vdd_mv.max(0) as f64;
    let mut g_sum = 0.0f64;
    let mut gv_sum = 0.0f64;

    let (is_out, out_high, pu, pd_drv, ext_drive, ext_pull) = if port == 'A' && (idx as usize) < 16 {
        let i = idx as usize;
        let bit = 1u16 << i;
        let out = (gpio.periph_out_pa_mask & bit) != 0 || (gpio.pa_dir & bit) != 0;
        let high = if (gpio.periph_out_pa_mask & bit) != 0 {
            (gpio.periph_out_pa_val & bit) != 0
        } else {
            (gpio.pa_out & bit) != 0
        };
        (
            out,
            high,
            (gpio.pa_pu & bit) != 0,
            (gpio.pa_pd_drv & bit) != 0,
            gpio.ext_pa[i],
            gpio.ext_pull_pa[i],
        )
    } else if port == 'B' && (idx as usize) < 24 {
        let i = idx as usize;
        let bit = 1u32 << i;
        let out = (gpio.periph_out_pb_mask & bit) != 0 || (gpio.pb_dir & bit) != 0;
        let high = if (gpio.periph_out_pb_mask & bit) != 0 {
            (gpio.periph_out_pb_val & bit) != 0
        } else {
            (gpio.pb_out & bit) != 0
        };
        (
            out,
            high,
            (gpio.pb_pu & bit) != 0,
            (gpio.pb_pd_drv & bit) != 0,
            gpio.ext_pb[i],
            gpio.ext_pull_pb[i],
        )
    } else {
        return 0;
    };

    if is_out
        && ext_pull == ExtPull::None
        && matches!(ext_drive, ExtPinDrive::Floating)
        && model.load_ohm.is_none()
    {
        return if out_high { vdd_mv.max(0) } else { 0 };
    }

    if is_out {
        let r_mcu = if pd_drv { 25.0 } else { 100.0 };
        let g = 1.0 / r_mcu;
        g_sum += g;
        if out_high {
            gv_sum += g * vdd;
        }
    } else if pu && !pd_drv {
        let g = 1.0 / 40_000.0;
        g_sum += g;
        gv_sum += g * vdd;
    } else if pd_drv {
        g_sum += 1.0 / 40_000.0;
    }

    let ext_pull_g = 1.0 / (model.ext_pull_ohm.max(1) as f64);
    match ext_pull {
        ExtPull::PullUp => {
            g_sum += ext_pull_g;
            gv_sum += ext_pull_g * vdd;
        }
        ExtPull::PullDown => {
            g_sum += ext_pull_g;
        }
        ExtPull::None => {}
    }

    let r_src = model.source_ohm.max(1) as f64;
    let src_g = 1.0 / r_src;
    match ext_drive {
        ExtPinDrive::High => {
            g_sum += src_g;
            gv_sum += src_g * vdd;
        }
        ExtPinDrive::Low => {
            g_sum += src_g;
        }
        ExtPinDrive::AnalogMv(mv) => {
            g_sum += src_g;
            gv_sum += src_g * (mv as f64);
        }
        ExtPinDrive::Floating => {}
    }

    if let Some(r_load) = model.load_ohm {
        let load_g = 1.0 / (r_load.max(1) as f64);
        g_sum += load_g;
        if model.load_to_vdd {
            gv_sum += load_g * vdd;
        }
    }

    if g_sum > 0.0 {
        (gv_sum / g_sum).round() as i32
    } else {
        0
    }
}
