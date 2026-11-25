use std::cell::RefCell;
use std::rc::Rc;

use ch585_core::ext::{ExternalDevice, PinChangeEvent, PinContext, PinWatchMask};
use ch585_core::peripherals::adc::{Adc, AdcWaveform};
use ch585_core::peripherals::gpio::{ExtPinDrive, ExtPull, Gpio};
use ch585_core::soc::Ch585Machine;

const TOKEN_ENCODER: u64 = 1;
const TOKEN_WS_LATCH: u64 = 2;
const TOKEN_CHARGER_BLINK: u64 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PinRef {
    pub port: char,
    pub idx: u8,
}

impl PinRef {
    pub const fn new(port: char, idx: u8) -> Self {
        Self { port, idx }
    }

    #[inline]
    pub fn matches(&self, ev: &PinChangeEvent) -> bool {
        self.port.eq_ignore_ascii_case(&ev.port) && self.idx == ev.idx
    }
}

#[inline]
fn read_gpio_pin(gpio: &Gpio, pin: PinRef) -> bool {
    gpio.pin_level(pin.port, pin.idx)
}

#[inline]
fn drive_gpio_pin(gpio: &mut Gpio, pin: PinRef, drive: ExtPinDrive) {
    gpio.set_ext_drive(pin.port, pin.idx, drive);
}

/// Ratiometric analog power rail model (`VCC_ANALOG` gated by LDO EN pin such as `PB16`).
#[derive(Debug, Clone)]
pub struct RatiometricRail {
    pub enabled: bool,
    pub en_pin: PinRef,
    pub vcc_analog_mv: i32,
    pub vana_ch: u8,
    pub ratios: [Option<f32>; 14],
    last_rail_mv: Option<i32>,
}

impl Default for RatiometricRail {
    fn default() -> Self {
        Self {
            enabled: false,
            en_pin: PinRef::new('B', 16),
            vcc_analog_mv: 3300,
            vana_ch: 4,
            ratios: [None; 14],
            last_rail_mv: None,
        }
    }
}

impl RatiometricRail {
    pub fn set_channel_mv(&mut self, ch: usize, mv: i32) {
        if ch < 14 && self.vcc_analog_mv > 0 {
            self.ratios[ch] = Some(mv as f32 / self.vcc_analog_mv as f32);
            self.last_rail_mv = None;
        }
    }

    pub fn set_supply_mv(&mut self, mv: i32) {
        self.vcc_analog_mv = mv.clamp(0, 5000);
        self.last_rail_mv = None;
    }

    pub fn invalidate(&mut self) {
        self.last_rail_mv = None;
    }

    pub fn apply(&mut self, en_high: bool, adc: &mut Adc) {
        if !self.enabled {
            return;
        }
        let rail_mv = if en_high { self.vcc_analog_mv } else { 0 };
        if self.last_rail_mv == Some(rail_mv) {
            return;
        }
        self.last_rail_mv = Some(rail_mv);
        if (self.vana_ch as usize) < 14 {
            adc.channels[self.vana_ch as usize] = AdcWaveform::Dc(rail_mv / 2);
        }
        for (ch, ratio_opt) in self.ratios.iter().enumerate() {
            if let Some(ratio) = ratio_opt {
                let scaled = ((rail_mv as f32) * ratio).round() as i32;
                adc.channels[ch] = AdcWaveform::Dc(scaled.clamp(0, 3300));
            }
        }
    }
}

/// Li-ion battery voltage model feeding an ADC channel through a resistor divider (`1M / 470k`).
#[derive(Debug, Clone)]
pub struct BatteryModel {
    pub enabled: bool,
    pub ch: u8,
    pub mv: i32,
    pub r_top_k: u32,
    pub r_bot_k: u32,
    last_mv: Option<i32>,
}

impl Default for BatteryModel {
    fn default() -> Self {
        Self {
            enabled: false,
            ch: 13, // PA9 / AIN13
            mv: 3700,
            r_top_k: 1000,
            r_bot_k: 470,
            last_mv: None,
        }
    }
}

impl BatteryModel {
    pub fn pin_mv(&self) -> i32 {
        let denom = (self.r_top_k + self.r_bot_k).max(1) as i64;
        ((self.mv as i64 * self.r_bot_k as i64 + denom / 2) / denom) as i32
    }

    pub fn set_mv(&mut self, mv: i32) {
        self.mv = mv.clamp(0, 5000);
        self.last_mv = None;
    }

    pub fn mark_synced(&mut self) {
        self.last_mv = Some(self.mv);
    }

    pub fn apply(&mut self, adc: &mut Adc) {
        if !self.enabled || self.last_mv == Some(self.mv) {
            return;
        }
        self.last_mv = Some(self.mv);
        if (self.ch as usize) < 14 {
            adc.channels[self.ch as usize] = AdcWaveform::Dc(self.pin_mv());
        }
        adc.vbat_mv = self.mv;
    }
}

/// 74HC165 16-bit parallel-in serial-out shift register driven via GPIO pin events.
#[derive(Debug, Clone)]
pub struct ShiftReg165 {
    pub enabled: bool,
    pub data_pin: PinRef,  // QH -> PB3
    pub latch_pin: PinRef, // PL# -> PB5 (active low)
    pub clk_pin: PinRef,   // CP -> PB6 (rising edge)
    pub input_bits: u16,
    shift_reg: u16,
    prev_clk: bool,
}

impl Default for ShiftReg165 {
    fn default() -> Self {
        Self {
            enabled: false,
            data_pin: PinRef::new('B', 3),
            latch_pin: PinRef::new('B', 5),
            clk_pin: PinRef::new('B', 6),
            input_bits: 0xFFFF,
            shift_reg: 0xFFFF,
            prev_clk: false,
        }
    }
}

impl ShiftReg165 {
    pub fn evaluate(&mut self, latch_high: bool, clk_high: bool) -> ExtPinDrive {
        if !latch_high {
            self.shift_reg = self.input_bits;
        } else if !self.prev_clk && clk_high {
            self.shift_reg = (self.shift_reg << 1) | 1;
        }
        self.prev_clk = clk_high;
        if (self.shift_reg & 0x8000) != 0 {
            ExtPinDrive::High
        } else {
            ExtPinDrive::Low
        }
    }
}

/// Incremental quadrature encoder (A/B Gray code) driven by scheduled emulator timers.
#[derive(Debug, Clone)]
pub struct QuadEncoder {
    pub enabled: bool,
    pub pin_a: PinRef,
    pub pin_b: PinRef,
    pub step_period_ns: u64,
    pending_steps: i32,
    phase: u8,
    timer_armed: bool,
}

impl Default for QuadEncoder {
    fn default() -> Self {
        Self {
            enabled: false,
            pin_a: PinRef::new('B', 1),
            pin_b: PinRef::new('B', 2),
            step_period_ns: 2_000_000,
            pending_steps: 0,
            phase: 0,
            timer_armed: false,
        }
    }
}

impl QuadEncoder {
    pub fn pending(&self) -> i32 {
        self.pending_steps
    }

    pub fn current_levels(&self) -> (ExtPinDrive, ExtPinDrive) {
        const GRAY: [(bool, bool); 4] = [
            (true, true),
            (false, true),
            (false, false),
            (true, false),
        ];
        let (a, b) = GRAY[(self.phase & 3) as usize];
        (
            if a { ExtPinDrive::High } else { ExtPinDrive::Low },
            if b { ExtPinDrive::High } else { ExtPinDrive::Low },
        )
    }

    fn advance_one(&mut self) -> bool {
        if self.pending_steps > 0 {
            self.phase = (self.phase + 1) & 3;
            self.pending_steps -= 1;
            true
        } else if self.pending_steps < 0 {
            self.phase = (self.phase + 3) & 3;
            self.pending_steps += 1;
            true
        } else {
            false
        }
    }
}

/// WS2812 / NeoPixel single-wire NZR decoder (`PB10` default) built on pin-change + latch timer events.
#[derive(Debug, Clone)]
pub struct Ws2812Decoder {
    pub enabled: bool,
    pub pin: PinRef,
    pub leds: Vec<[u8; 3]>,
    pub frame_count: u64,
    prev_level: bool,
    last_edge_ns: u64,
    cur_bytes: Vec<u8>,
    cur_byte: u8,
    bit_count: u8,
}

impl Default for Ws2812Decoder {
    fn default() -> Self {
        Self {
            enabled: false,
            pin: PinRef::new('B', 10),
            leds: Vec::new(),
            frame_count: 0,
            prev_level: false,
            last_edge_ns: 0,
            cur_bytes: Vec::new(),
            cur_byte: 0,
            bit_count: 0,
        }
    }
}

impl Ws2812Decoder {
    fn on_edge(&mut self, level: bool, time_ns: u64, ctx: &mut PinContext<'_>) {
        if !self.enabled || level == self.prev_level {
            return;
        }
        let dur = time_ns.saturating_sub(self.last_edge_ns);
        if self.prev_level && !level {
            if (120..=2000).contains(&dur) {
                let bit = if dur >= 550 { 1 } else { 0 };
                self.cur_byte = (self.cur_byte << 1) | bit;
                self.bit_count += 1;
                if self.bit_count == 8 {
                    self.cur_bytes.push(self.cur_byte);
                    self.cur_byte = 0;
                    self.bit_count = 0;
                }
            }
            ctx.reschedule_after_ns(50_000, TOKEN_WS_LATCH);
        }
        self.prev_level = level;
        self.last_edge_ns = time_ns;
    }

    fn on_latch_timer(&mut self, time_ns: u64) {
        if !self.enabled || self.prev_level || self.cur_bytes.is_empty() {
            return;
        }
        if time_ns.saturating_sub(self.last_edge_ns) >= 50_000 {
            let mut decoded = Vec::new();
            for chunk in self.cur_bytes.chunks_exact(3) {
                decoded.push([chunk[1], chunk[0], chunk[2]]);
            }
            if !decoded.is_empty() {
                self.leds = decoded;
                self.frame_count += 1;
            }
            self.cur_bytes.clear();
            self.cur_byte = 0;
            self.bit_count = 0;
        }
    }
}

/// BQ25300 Li-ion charger IC model (open-drain STAT driven from VBUS & active-LOW CHG_EN).
#[derive(Debug, Clone)]
pub struct Bq25300 {
    pub enabled: bool,
    pub vbus_pin: PinRef,   // PB9
    pub chg_en_pin: PinRef, // PB7
    pub stat_pin: PinRef,   // PB8
    pub en_active_low: bool,
    pub battery_full: bool,
    pub fault: bool,
    pub charging: bool,
    blink_scheduled: bool,
}

impl Default for Bq25300 {
    fn default() -> Self {
        Self {
            enabled: false,
            vbus_pin: PinRef::new('B', 9),
            chg_en_pin: PinRef::new('B', 7),
            stat_pin: PinRef::new('B', 8),
            en_active_low: true,
            battery_full: false,
            fault: false,
            charging: false,
            blink_scheduled: false,
        }
    }
}

impl Bq25300 {
    pub fn evaluate(&mut self, time_ns: u64, vbus: bool, chg_en_pin_high: bool) -> ExtPinDrive {
        let en_active = if self.en_active_low {
            !chg_en_pin_high
        } else {
            chg_en_pin_high
        };
        self.charging = vbus && en_active && !self.battery_full && !self.fault;
        if self.fault && vbus {
            if ((time_ns / 500_000_000) & 1) == 0 {
                ExtPinDrive::Low
            } else {
                ExtPinDrive::Floating
            }
        } else if self.charging {
            ExtPinDrive::Low
        } else {
            ExtPinDrive::Floating
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct MacropadV2State {
    pub ratiometric: RatiometricRail,
    pub battery: BatteryModel,
    pub shift165: ShiftReg165,
    pub encoder: QuadEncoder,
    pub ws2812: Ws2812Decoder,
    pub charger: Bq25300,
}

#[derive(Clone)]
pub struct MacropadV2Handle {
    device_id: usize,
    state: Rc<RefCell<MacropadV2State>>,
}

impl MacropadV2Handle {
    pub fn state(&self) -> std::cell::Ref<'_, MacropadV2State> {
        self.state.borrow()
    }

    pub fn state_mut(&self) -> std::cell::RefMut<'_, MacropadV2State> {
        self.state.borrow_mut()
    }

    pub fn sync(&self, mach: &mut Ch585Machine) {
        let mut st = self.state.borrow_mut();
        let time_ns = mach.bus.total_time_ns;

        let en_high = read_gpio_pin(&mach.bus.gpio, st.ratiometric.en_pin);
        st.ratiometric.apply(en_high, &mut mach.bus.adc);
        st.battery.apply(&mut mach.bus.adc);

        if st.shift165.enabled {
            let latch = read_gpio_pin(&mach.bus.gpio, st.shift165.latch_pin);
            let clk = read_gpio_pin(&mach.bus.gpio, st.shift165.clk_pin);
            let qh = st.shift165.evaluate(latch, clk);
            let pin = st.shift165.data_pin;
            drive_gpio_pin(&mut mach.bus.gpio, pin, qh);
        }

        if st.charger.enabled {
            let vbus = read_gpio_pin(&mach.bus.gpio, st.charger.vbus_pin);
            let chg_en = read_gpio_pin(&mach.bus.gpio, st.charger.chg_en_pin);
            let stat = st.charger.evaluate(time_ns, vbus, chg_en);
            let pin = st.charger.stat_pin;
            drive_gpio_pin(&mut mach.bus.gpio, pin, stat);
        }

        if st.encoder.enabled {
            let (a, b) = st.encoder.current_levels();
            let pa = st.encoder.pin_a;
            let pb = st.encoder.pin_b;
            drive_gpio_pin(&mut mach.bus.gpio, pa, a);
            drive_gpio_pin(&mut mach.bus.gpio, pb, b);
        }
        drop(st);
        mach.poll_ext();
    }

    pub fn set_battery_mv(&self, mach: &mut Ch585Machine, mv: i32) {
        {
            let mut st = self.state.borrow_mut();
            st.battery.enabled = true;
            st.battery.set_mv(mv);
            st.battery.apply(&mut mach.bus.adc);
        }
        mach.poll_ext();
    }

    pub fn set_supply_mv(&self, mach: &mut Ch585Machine, mv: i32) {
        {
            let mut st = self.state.borrow_mut();
            st.ratiometric.set_supply_mv(mv);
            let en_high = read_gpio_pin(&mach.bus.gpio, st.ratiometric.en_pin);
            st.ratiometric.apply(en_high, &mut mach.bus.adc);
        }
        mach.poll_ext();
    }

    pub fn set_adc_channel_mv(&self, mach: &mut Ch585Machine, ch: usize, mv: i32) {
        {
            let mut st = self.state.borrow_mut();
            if ch < 14 {
                mach.bus.adc.channels[ch] = AdcWaveform::Dc(mv);
                if st.ratiometric.enabled {
                    st.ratiometric.set_channel_mv(ch, mv);
                }
                if st.battery.enabled && ch == st.battery.ch as usize {
                    st.battery.mark_synced();
                }
            }
        }
        mach.poll_ext();
    }

    pub fn set_shift165_bits(&self, mach: &mut Ch585Machine, bits: u16) {
        {
            let mut st = self.state.borrow_mut();
            st.shift165.input_bits = bits;
            if st.shift165.enabled {
                let latch = read_gpio_pin(&mach.bus.gpio, st.shift165.latch_pin);
                let clk = read_gpio_pin(&mach.bus.gpio, st.shift165.clk_pin);
                let qh = st.shift165.evaluate(latch, clk);
                let pin = st.shift165.data_pin;
                drive_gpio_pin(&mut mach.bus.gpio, pin, qh);
            }
        }
        mach.poll_ext();
    }

    pub fn queue_encoder_steps(&self, mach: &mut Ch585Machine, steps: i32) {
        if steps == 0 {
            return;
        }
        let mut schedule_delay = None;
        {
            let mut st = self.state.borrow_mut();
            st.encoder.pending_steps = st.encoder.pending_steps.saturating_add(steps);
            if st.encoder.enabled && !st.encoder.timer_armed {
                st.encoder.timer_armed = true;
                schedule_delay = Some(st.encoder.step_period_ns);
            }
        }
        if let Some(delay_ns) = schedule_delay {
            let target = mach.bus.total_time_ns.saturating_add(delay_ns);
            mach.schedule_device_timer(self.device_id, target, TOKEN_ENCODER);
        }
    }
}

struct MacropadDevice {
    state: Rc<RefCell<MacropadV2State>>,
}

impl MacropadDevice {
    fn sync_rail(&self, ctx: &mut PinContext<'_>) {
        let mut st = self.state.borrow_mut();
        let en_high = ctx.read_pin(st.ratiometric.en_pin.port, st.ratiometric.en_pin.idx);
        st.ratiometric.invalidate();
        let rail_mv = if en_high { st.ratiometric.vcc_analog_mv } else { 0 };
        if st.ratiometric.enabled {
            if (st.ratiometric.vana_ch as usize) < 14 {
                ctx.set_adc_mv(st.ratiometric.vana_ch as usize, rail_mv / 2);
            }
            for (ch, r_opt) in st.ratiometric.ratios.iter().enumerate() {
                if let Some(r) = r_opt {
                    let mv = ((rail_mv as f32) * r).round() as i32;
                    ctx.set_adc_mv(ch, mv.clamp(0, 3300));
                }
            }
        }
    }

    fn sync_battery(&self, ctx: &mut PinContext<'_>) {
        let mut st = self.state.borrow_mut();
        if st.battery.enabled {
            let pin_mv = st.battery.pin_mv();
            let mv = st.battery.mv;
            let ch = st.battery.ch as usize;
            st.battery.mark_synced();
            ctx.set_adc_mv(ch, pin_mv);
            ctx.set_vbat_mv(mv);
        }
    }

    fn sync_charger(&self, ctx: &mut PinContext<'_>) {
        let mut need_blink = false;
        {
            let mut st = self.state.borrow_mut();
            if !st.charger.enabled {
                return;
            }
            let vbus = ctx.read_pin(st.charger.vbus_pin.port, st.charger.vbus_pin.idx);
            let chg_en = ctx.read_pin(st.charger.chg_en_pin.port, st.charger.chg_en_pin.idx);
            let stat = st.charger.evaluate(ctx.time_ns(), vbus, chg_en);
            let pin = st.charger.stat_pin;
            ctx.set_pin_drive(pin.port, pin.idx, stat);
            if st.charger.fault && vbus && !st.charger.blink_scheduled {
                st.charger.blink_scheduled = true;
                need_blink = true;
            }
        }
        if need_blink {
            ctx.schedule_after_ns(500_000_000, TOKEN_CHARGER_BLINK);
        }
    }

    fn sync_shift165(&self, ctx: &mut PinContext<'_>) {
        let mut st = self.state.borrow_mut();
        if !st.shift165.enabled {
            return;
        }
        let latch = ctx.read_pin(st.shift165.latch_pin.port, st.shift165.latch_pin.idx);
        let clk = ctx.read_pin(st.shift165.clk_pin.port, st.shift165.clk_pin.idx);
        let qh = st.shift165.evaluate(latch, clk);
        let data_pin = st.shift165.data_pin;
        ctx.set_pin_drive(data_pin.port, data_pin.idx, qh);
    }
}

impl ExternalDevice for MacropadDevice {
    fn watched_pins(&self) -> PinWatchMask {
        let st = self.state.borrow();
        PinWatchMask::NONE
            .with_pin(st.ratiometric.en_pin.port, st.ratiometric.en_pin.idx)
            .with_pin(st.shift165.latch_pin.port, st.shift165.latch_pin.idx)
            .with_pin(st.shift165.clk_pin.port, st.shift165.clk_pin.idx)
            .with_pin(st.charger.vbus_pin.port, st.charger.vbus_pin.idx)
            .with_pin(st.charger.chg_en_pin.port, st.charger.chg_en_pin.idx)
            .with_pin(st.ws2812.pin.port, st.ws2812.pin.idx)
    }

    fn on_attach(&mut self, ctx: &mut PinContext<'_>) {
        ctx.set_pin_pull('B', 7, ExtPull::PullDown);
        ctx.set_pin_pull('B', 0, ExtPull::PullUp);

        let pending_enc = {
            let mut st = self.state.borrow_mut();
            st.encoder.timer_armed = false;
            st.charger.blink_scheduled = false;
            if st.encoder.enabled {
                let (a, b) = st.encoder.current_levels();
                ctx.set_pin_drive(st.encoder.pin_a.port, st.encoder.pin_a.idx, a);
                ctx.set_pin_drive(st.encoder.pin_b.port, st.encoder.pin_b.idx, b);
                if st.encoder.pending_steps != 0 {
                    st.encoder.timer_armed = true;
                    Some(st.encoder.step_period_ns)
                } else {
                    None
                }
            } else {
                None
            }
        };
        if let Some(delay) = pending_enc {
            ctx.schedule_after_ns(delay, TOKEN_ENCODER);
        }

        self.sync_rail(ctx);
        self.sync_battery(ctx);
        self.sync_shift165(ctx);
        self.sync_charger(ctx);
    }

    fn on_pin_change(&mut self, ev: PinChangeEvent, ctx: &mut PinContext<'_>) {
        let (is_en, is_165, is_chg, is_ws) = {
            let st = self.state.borrow();
            (
                st.ratiometric.en_pin.matches(&ev),
                st.shift165.latch_pin.matches(&ev) || st.shift165.clk_pin.matches(&ev),
                st.charger.vbus_pin.matches(&ev) || st.charger.chg_en_pin.matches(&ev),
                st.ws2812.pin.matches(&ev),
            )
        };
        if is_en {
            self.sync_rail(ctx);
        }
        if is_165 {
            self.sync_shift165(ctx);
        }
        if is_chg {
            self.sync_charger(ctx);
        }
        if is_ws {
            let mut st = self.state.borrow_mut();
            st.ws2812.on_edge(ev.level, ev.time_ns, ctx);
        }
    }

    fn on_timer(&mut self, token: u64, ctx: &mut PinContext<'_>) {
        match token {
            TOKEN_ENCODER => {
                let mut reschedule = None;
                {
                    let mut st = self.state.borrow_mut();
                    st.encoder.timer_armed = false;
                    if st.encoder.enabled && st.encoder.advance_one() {
                        let (a, b) = st.encoder.current_levels();
                        ctx.set_pin_drive(st.encoder.pin_a.port, st.encoder.pin_a.idx, a);
                        ctx.set_pin_drive(st.encoder.pin_b.port, st.encoder.pin_b.idx, b);
                        if st.encoder.pending_steps != 0 {
                            st.encoder.timer_armed = true;
                            reschedule = Some(st.encoder.step_period_ns);
                        }
                    }
                }
                if let Some(d) = reschedule {
                    ctx.schedule_after_ns(d, TOKEN_ENCODER);
                }
            }
            TOKEN_WS_LATCH => {
                let mut st = self.state.borrow_mut();
                st.ws2812.on_latch_timer(ctx.time_ns());
            }
            TOKEN_CHARGER_BLINK => {
                {
                    let mut st = self.state.borrow_mut();
                    st.charger.blink_scheduled = false;
                }
                self.sync_charger(ctx);
            }
            _ => {}
        }
    }
}

pub struct MacropadV2Board;

impl MacropadV2Board {
    pub fn attach(mach: &mut Ch585Machine) -> MacropadV2Handle {
        let mut st = MacropadV2State::default();
        st.ratiometric.enabled = true;
        st.ratiometric.en_pin = PinRef::new('B', 16);
        st.ratiometric.vana_ch = 4;
        for ch in [1usize, 0, 10, 9, 8, 7, 6, 5, 3, 2, 11, 12] {
            st.ratiometric.ratios[ch] = Some(0.5);
        }
        st.battery.enabled = true;
        st.battery.ch = 13;
        st.battery.set_mv(3700);

        st.shift165.enabled = true;
        st.encoder.enabled = true;
        st.ws2812.enabled = true;
        st.charger.enabled = true;
        st.charger.en_active_low = true;

        let shared = Rc::new(RefCell::new(st));
        let dev = MacropadDevice {
            state: Rc::clone(&shared),
        };
        let device_id = mach.attach_device(Box::new(dev));
        MacropadV2Handle {
            device_id,
            state: shared,
        }
    }
}
