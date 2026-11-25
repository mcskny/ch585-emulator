use std::collections::VecDeque;

use crate::pfic::Pfic;

/// 16C550-compatible UART with 8-byte TX/RX FIFOs (`UART0..UART3`).
#[derive(Debug, Clone)]
pub struct Uart {
    pub idx: u8,
    pub irqn: u8,
    pub mcr: u8,
    pub ier: u8,
    pub fcr: u8,
    pub lcr: u8,
    pub msr: u8,
    pub dl: u16,
    pub div: u8,
    pub adr: u8,

    pub rx_fifo: VecDeque<u8>,
    pub tx_fifo: VecDeque<u8>,
    /// Captured transmitted bytes for CLI / Web UI serial terminal
    pub tx_log: Vec<u8>,
    tx_cycle_acc: u32,
    rx_idle_cycles: u32,
    rx_timeout_pending: bool,
    thr_empty_pending: bool,
}

impl Uart {
    pub fn new(idx: u8, irqn: u8) -> Self {
        Self {
            idx,
            irqn,
            mcr: 0,
            ier: 0,
            fcr: 0,
            lcr: 0,
            msr: 0,
            dl: 1,
            div: 1,
            adr: 0xFF,
            rx_fifo: VecDeque::with_capacity(8),
            tx_fifo: VecDeque::with_capacity(8),
            tx_log: Vec::new(),
            tx_cycle_acc: 0,
            rx_idle_cycles: 0,
            rx_timeout_pending: false,
            thr_empty_pending: false,
        }
    }

    pub fn inject_rx(&mut self, bytes: &[u8]) {
        for &b in bytes {
            if self.rx_fifo.len() < 8 {
                self.rx_fifo.push_back(b);
            }
        }
        self.rx_idle_cycles = 0;
        self.rx_timeout_pending = false;
    }

    pub fn take_tx_bytes(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.tx_log)
    }

    fn rx_trig_threshold(&self) -> usize {
        match (self.fcr >> 6) & 0x03 {
            0 => 1,
            1 => 2,
            2 => 4,
            _ => 7,
        }
    }

    pub fn iir(&self) -> u8 {
        let fifo_bits = if (self.fcr & 0x01) != 0 { 0xC0 } else { 0x00 };
        if (self.ier & 0x01) != 0 && self.rx_fifo.len() >= self.rx_trig_threshold() {
            fifo_bits | 0x04 // UART_II_RECV_RDY
        } else if (self.ier & 0x01) != 0 && self.rx_timeout_pending && !self.rx_fifo.is_empty() {
            fifo_bits | 0x0C // UART_II_RECV_TOUT
        } else if (self.ier & 0x02) != 0 && self.thr_empty_pending {
            fifo_bits | 0x02 // UART_II_THR_EMPTY
        } else {
            fifo_bits | 0x01 // UART_II_NO_INTER
        }
    }

    pub fn lsr(&self) -> u8 {
        let mut s = 0u8;
        if !self.rx_fifo.is_empty() {
            s |= 0x01; // RB_LSR_DATA_RDY
        }
        if self.tx_fifo.is_empty() {
            s |= 0x20 | 0x40; // TX_FIFO_EMP | TX_ALL_EMP
        }
        s
    }

    #[inline]
    pub fn next_event_cycles(&self) -> Option<u32> {
        if self.tx_fifo.is_empty() && self.rx_fifo.is_empty() {
            return None;
        }
        let bit_cycles = ((self.dl.max(1) as u32) * (self.div.max(1) as u32) * 8).max(1);
        let byte_cycles = (bit_cycles * 10).min(1000);
        let mut best: Option<u32> = None;
        if !self.tx_fifo.is_empty() {
            let c = byte_cycles.saturating_sub(self.tx_cycle_acc).max(1);
            best = Some(c);
        }
        if !self.rx_fifo.is_empty()
            && self.rx_fifo.len() < self.rx_trig_threshold()
            && !self.rx_timeout_pending
        {
            let c = (byte_cycles * 4).saturating_sub(self.rx_idle_cycles).max(1);
            best = Some(best.map_or(c, |b| b.min(c)));
        }
        best
    }

    #[inline]
    pub fn step(&mut self, cycles: u32, pfic: &mut Pfic) {
        if self.tx_fifo.is_empty()
            && self.rx_fifo.is_empty()
            && !self.thr_empty_pending
            && self.ier == 0
        {
            self.rx_idle_cycles = 0;
            pfic.sync_level_irq(self.irqn, false);
            return;
        }
        // Shift out TX FIFO at configured baud (or fast rate)
        let bit_cycles = ((self.dl.max(1) as u32) * (self.div.max(1) as u32) * 8).max(1);
        let byte_cycles = (bit_cycles * 10).min(1000); // cap stall in high baud / fast tests
        if !self.tx_fifo.is_empty() {
            self.tx_cycle_acc += cycles;
            while self.tx_cycle_acc >= byte_cycles {
                self.tx_cycle_acc -= byte_cycles;
                if let Some(b) = self.tx_fifo.pop_front() {
                    if (self.mcr & 0x10) != 0 {
                        // Loopback
                        if self.rx_fifo.len() < 8 {
                            self.rx_fifo.push_back(b);
                        }
                    } else {
                        self.tx_log.push(b);
                    }
                    if self.tx_fifo.is_empty() {
                        self.thr_empty_pending = true;
                        break;
                    }
                } else {
                    break;
                }
            }
        }

        // RX timeout if bytes sit below threshold for > 4 char times
        if !self.rx_fifo.is_empty() && self.rx_fifo.len() < self.rx_trig_threshold() {
            self.rx_idle_cycles += cycles;
            if self.rx_idle_cycles >= byte_cycles * 4 {
                self.rx_timeout_pending = true;
            }
        } else {
            self.rx_idle_cycles = 0;
        }

        // Fire PFIC interrupt if RB_MCR_INT_OE (or always when enabled)
        let asserted = (self.iir() & 0x01) == 0 && ((self.mcr & 0x08) != 0 || self.ier != 0);
        pfic.sync_level_irq(self.irqn, asserted);
    }

    pub fn read_u8(&mut self, offset: u32) -> u8 {
        match offset {
            0x00 => self.mcr,
            0x01 => self.ier,
            0x02 => self.fcr,
            0x03 => self.lcr,
            0x04 => {
                let v = self.iir();
                if (v & 0x0F) == 0x02 {
                    self.thr_empty_pending = false;
                }
                v
            }
            0x05 => self.lsr(),
            0x06 => self.msr,
            0x08 => {
                let b = self.rx_fifo.pop_front().unwrap_or(0);
                if self.rx_fifo.is_empty() {
                    self.rx_timeout_pending = false;
                }
                b
            }
            0x0A => self.rx_fifo.len() as u8,
            0x0B => self.tx_fifo.len() as u8,
            0x0C => self.dl as u8,
            0x0D => (self.dl >> 8) as u8,
            0x0E => self.div,
            0x0F => self.adr,
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        match offset {
            0x00 => self.mcr = val,
            0x01 => {
                if (val & 0x80) != 0 {
                    // RB_IER_RESET
                    let idx = self.idx;
                    let irqn = self.irqn;
                    let log = std::mem::take(&mut self.tx_log);
                    *self = Self::new(idx, irqn);
                    self.tx_log = log;
                    return;
                }
                let old = self.ier;
                self.ier = val;
                if (old & 0x02) == 0 && (val & 0x02) != 0 && self.tx_fifo.is_empty() {
                    self.thr_empty_pending = true;
                }
            }
            0x02 => {
                if (val & 0x02) != 0 {
                    self.rx_fifo.clear();
                    self.rx_timeout_pending = false;
                }
                if (val & 0x04) != 0 {
                    self.tx_fifo.clear();
                }
                self.fcr = val & !0x06;
            }
            0x03 => self.lcr = val,
            0x08 => {
                self.thr_empty_pending = false;
                if self.tx_fifo.len() < 8 {
                    self.tx_fifo.push_back(val);
                }
            }
            0x0C => self.dl = (self.dl & 0xFF00) | (val as u16),
            0x0D => self.dl = (self.dl & 0x00FF) | ((val as u16) << 8),
            0x0E => self.div = val,
            0x0F => self.adr = val,
            _ => {}
        }
    }

    pub fn read_u32(&mut self, offset: u32) -> u32 {
        let b0 = self.read_u8(offset) as u32;
        let b1 = self.read_u8(offset + 1) as u32;
        let b2 = self.read_u8(offset + 2) as u32;
        let b3 = self.read_u8(offset + 3) as u32;
        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    }

    pub fn write_u32(&mut self, offset: u32, val: u32) {
        let b = val.to_le_bytes();
        for i in 0..4 {
            self.write_u8(offset + i as u32, b[i]);
        }
    }
}
