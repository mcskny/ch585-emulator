use std::collections::BTreeMap;

use crate::peripherals::gpio::{ExtPinDrive, ExtPull};
use crate::rf::{RfMedium, RfMediumConfig};
use crate::soc::Ch585Machine;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct GpioPinRef {
    pub chip: usize,
    pub port: char,
    pub idx: u8,
}

impl GpioPinRef {
    pub fn new(chip: usize, port: char, idx: u8) -> Self {
        Self {
            chip,
            port: port.to_ascii_uppercase(),
            idx,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GpioWire {
    pub a: GpioPinRef,
    pub b: GpioPinRef,
    pub pull: ExtPull,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UartWire {
    pub src_chip: usize,
    pub src_uart: usize,
    pub dst_chip: usize,
    pub dst_uart: usize,
    pub tx_cursor: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OwnDrive {
    StrongHigh,
    StrongLow,
    WeakPullUp,
    WeakPullDown,
    HighZ,
}

/// Synchronized multi-chip cluster sharing a nanosecond clock, GPIO/UART wires, and a 2.4 GHz RF ether.
pub struct MultiChipCluster {
    pub chips: Vec<Ch585Machine>,
    pub names: Vec<String>,
    pub rf_medium: RfMedium,
    pub gpio_wires: Vec<GpioWire>,
    pub uart_wires: Vec<UartWire>,
    pub sim_time_ns: u64,
    pub quantum_us: u64,
}

impl Default for MultiChipCluster {
    fn default() -> Self {
        Self::new()
    }
}

impl MultiChipCluster {
    pub fn new() -> Self {
        Self::with_rf_config(RfMediumConfig::default())
    }

    pub fn with_rf_config(rf_cfg: RfMediumConfig) -> Self {
        Self {
            chips: Vec::new(),
            names: Vec::new(),
            rf_medium: RfMedium::new(rf_cfg),
            gpio_wires: Vec::new(),
            uart_wires: Vec::new(),
            sim_time_ns: 0,
            quantum_us: 50,
        }
    }

    /// Add a chip to the cluster, assigning a distinct MAC suffix if it still holds the factory default.
    pub fn add_chip(&mut self, name: impl Into<String>, mut mach: Ch585Machine) -> usize {
        let idx = self.chips.len();
        let default_mac = [0x58, 0x50, 0x00, 0x12, 0x34, 0x56];
        if idx > 0 && mach.bus.ble.local_mac == default_mac {
            mach.set_mac_address([0x58, 0x50, 0x00, 0x12, 0x34, 0x56u8.wrapping_add(idx as u8)]);
        }
        self.sim_time_ns = self.sim_time_ns.max(mach.bus.total_time_ns);
        self.chips.push(mach);
        self.names.push(name.into());
        idx
    }

    pub fn chip(&self, idx: usize) -> &Ch585Machine {
        &self.chips[idx]
    }

    pub fn chip_mut(&mut self, idx: usize) -> &mut Ch585Machine {
        &mut self.chips[idx]
    }

    pub fn connect_gpio(&mut self, a: GpioPinRef, b: GpioPinRef) {
        self.connect_gpio_with_pull(a, b, ExtPull::None);
    }

    pub fn connect_gpio_with_pull(&mut self, a: GpioPinRef, b: GpioPinRef, pull: ExtPull) {
        self.gpio_wires.push(GpioWire { a, b, pull });
    }

    pub fn connect_uart(
        &mut self,
        src_chip: usize,
        src_uart: usize,
        dst_chip: usize,
        dst_uart: usize,
    ) {
        self.uart_wires.push(UartWire {
            src_chip,
            src_uart,
            dst_chip,
            dst_uart,
            tx_cursor: 0,
        });
    }

    pub fn connect_uart_duplex(
        &mut self,
        chip_a: usize,
        uart_a: usize,
        chip_b: usize,
        uart_b: usize,
    ) {
        self.connect_uart(chip_a, uart_a, chip_b, uart_b);
        self.connect_uart(chip_b, uart_b, chip_a, uart_a);
    }

    fn read_chip_own_drive(mach: &Ch585Machine, pin: GpioPinRef) -> OwnDrive {
        if !mach.bus.powered {
            return OwnDrive::HighZ;
        }
        let g = &mach.bus.gpio;
        match pin.port {
            'A' if (pin.idx as usize) < 16 => {
                let bit = 1u16 << pin.idx;
                if (g.periph_out_pa_mask & bit) != 0 {
                    if (g.periph_out_pa_val & bit) != 0 {
                        OwnDrive::StrongHigh
                    } else {
                        OwnDrive::StrongLow
                    }
                } else if (g.pa_dir & bit) != 0 {
                    if (g.pa_out & bit) != 0 {
                        OwnDrive::StrongHigh
                    } else {
                        OwnDrive::StrongLow
                    }
                } else if (g.pa_pu & bit) != 0 {
                    OwnDrive::WeakPullUp
                } else if (g.pa_pd_drv & bit) != 0 {
                    OwnDrive::WeakPullDown
                } else {
                    OwnDrive::HighZ
                }
            }
            'B' if (pin.idx as usize) < 24 => {
                let bit = 1u32 << pin.idx;
                if (g.periph_out_pb_mask & bit) != 0 {
                    if (g.periph_out_pb_val & bit) != 0 {
                        OwnDrive::StrongHigh
                    } else {
                        OwnDrive::StrongLow
                    }
                } else if (g.pb_dir & bit) != 0 {
                    if (g.pb_out & bit) != 0 {
                        OwnDrive::StrongHigh
                    } else {
                        OwnDrive::StrongLow
                    }
                } else if (g.pb_pu & bit) != 0 {
                    OwnDrive::WeakPullUp
                } else if (g.pb_pd_drv & bit) != 0 {
                    OwnDrive::WeakPullDown
                } else {
                    OwnDrive::HighZ
                }
            }
            _ => OwnDrive::HighZ,
        }
    }

    /// Propagate inter-chip GPIO wires without feedback latching.
    pub fn sync_gpio_wires(&mut self) {
        if self.gpio_wires.is_empty() {
            return;
        }
        let mut net_inputs: BTreeMap<GpioPinRef, (bool, bool, ExtPull)> = BTreeMap::new();
        let mut accumulate = |dst: GpioPinRef, peer_drv: OwnDrive, wire_pull: ExtPull| {
            let entry = net_inputs.entry(dst).or_insert((false, false, ExtPull::None));
            match peer_drv {
                OwnDrive::StrongHigh => entry.0 = true,
                OwnDrive::StrongLow => entry.1 = true,
                OwnDrive::WeakPullUp => {
                    if entry.2 == ExtPull::None {
                        entry.2 = ExtPull::PullUp;
                    }
                }
                OwnDrive::WeakPullDown => {
                    if entry.2 == ExtPull::None {
                        entry.2 = ExtPull::PullDown;
                    }
                }
                OwnDrive::HighZ => {}
            }
            if wire_pull != ExtPull::None && entry.2 == ExtPull::None {
                entry.2 = wire_pull;
            }
        };

        for wire in &self.gpio_wires {
            if wire.a.chip >= self.chips.len() || wire.b.chip >= self.chips.len() {
                continue;
            }
            let drv_a = Self::read_chip_own_drive(&self.chips[wire.a.chip], wire.a);
            let drv_b = Self::read_chip_own_drive(&self.chips[wire.b.chip], wire.b);
            accumulate(wire.b, drv_a, wire.pull);
            accumulate(wire.a, drv_b, wire.pull);
        }

        for (pin, (seen_high, seen_low, pull)) in net_inputs {
            let mach = &mut self.chips[pin.chip];
            let drive = if seen_low && !seen_high {
                ExtPinDrive::Low
            } else if seen_high && !seen_low {
                ExtPinDrive::High
            } else if seen_low && seen_high {
                // Wired fight on external bus resolves low (dominant)
                ExtPinDrive::Low
            } else {
                ExtPinDrive::Floating
            };
            let prev_v = mach.bus.gpio.version;
            mach.set_pin_drive(pin.port, pin.idx, drive);
            mach.set_pin_pull(pin.port, pin.idx, pull);
            if mach.bus.gpio.version != prev_v {
                mach.bus.sync_after_ext();
            }
        }
    }

    /// Propagate bytes from each wired TX UART `tx_log` into the peer RX FIFO (with FIFO backpressure).
    pub fn sync_uart_wires(&mut self) {
        if self.uart_wires.is_empty() {
            return;
        }
        for wire in &mut self.uart_wires {
            if wire.src_chip >= self.chips.len()
                || wire.dst_chip >= self.chips.len()
                || wire.src_chip == wire.dst_chip
                || wire.src_uart >= 4
                || wire.dst_uart >= 4
            {
                continue;
            }
            if !self.chips[wire.src_chip].bus.powered {
                wire.tx_cursor = 0;
                continue;
            }
            let tx_len = self.chips[wire.src_chip].bus.uarts[wire.src_uart].tx_log.len();
            if tx_len < wire.tx_cursor {
                wire.tx_cursor = tx_len;
            }
            if wire.tx_cursor >= tx_len || !self.chips[wire.dst_chip].bus.powered {
                continue;
            }
            let rx_free = 8usize.saturating_sub(
                self.chips[wire.dst_chip].bus.uarts[wire.dst_uart]
                    .rx_fifo
                    .len(),
            );
            if rx_free == 0 {
                continue;
            }
            let take = (tx_len - wire.tx_cursor).min(rx_free);
            let bytes = self.chips[wire.src_chip].bus.uarts[wire.src_uart].tx_log
                [wire.tx_cursor..wire.tx_cursor + take]
                .to_vec();
            wire.tx_cursor += take;
            let dst = &mut self.chips[wire.dst_chip];
            dst.bus.uarts[wire.dst_uart].inject_rx(&bytes);
        }
    }

    /// Advance all chips synchronously for `total_us` microseconds on a shared quantum timeline.
    pub fn run_us(&mut self, total_us: u64) {
        if total_us == 0 || self.chips.is_empty() {
            return;
        }
        let end_ns = self.sim_time_ns.saturating_add(total_us.saturating_mul(1_000));
        let quantum_ns = self.quantum_us.max(1).saturating_mul(1_000);

        while self.sim_time_ns < end_ns {
            let next_ns = self.sim_time_ns.saturating_add(quantum_ns).min(end_ns);
            for chip in &mut self.chips {
                if chip.bus.total_time_ns < next_ns {
                    let delta_us = (next_ns - chip.bus.total_time_ns).div_ceil(1_000);
                    chip.run_us(delta_us);
                }
            }
            self.sim_time_ns = next_ns;
            self.sync_gpio_wires();
            self.sync_uart_wires();
            self.rf_medium.step(self.sim_time_ns, &mut self.chips);
            for chip in &mut self.chips {
                if chip.dispatch_pending_ble_callback() {
                    // Enter immediately so the next quantum starts executing inside the callback
                }
            }
        }
    }
}
