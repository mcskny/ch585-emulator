use std::collections::VecDeque;

use crate::pfic::Pfic;

/// SPI0 (`0x4000_4000`, DMA + IRQ 19) and SPI1 (`0x4000_4400`).
#[derive(Debug, Clone)]
pub struct Spi {
    pub idx: u8,
    pub irqn: Option<u8>,
    pub ctrl_mod: u8,  // 0x00
    pub ctrl_cfg: u8,  // 0x01
    pub inter_en: u8,  // 0x02
    pub clock_div: u8, // 0x03
    pub buffer: u8,    // 0x04
    pub run_flag: u8,  // 0x05
    pub int_flag: u8,  // 0x06
    pub total_cnt: u16,// 0x0C
    pub fifo: VecDeque<u8>, // 0x10
    pub dma_now: u32,  // 0x14
    pub dma_beg: u32,  // 0x18
    pub dma_end: u32,  // 0x1C

    /// Transmitted bytes observed externally (or loopback / virtual slave response)
    pub tx_log: Vec<u8>,
    pub rx_feed: VecDeque<u8>,
    pub dma_active_step: bool,
}

impl Spi {
    pub fn new(idx: u8, irqn: Option<u8>) -> Self {
        Self {
            idx,
            irqn,
            ctrl_mod: 0,
            ctrl_cfg: 0,
            inter_en: 0,
            clock_div: 4,
            buffer: 0xFF,
            run_flag: 0,
            int_flag: 0x40, // RB_SPI_FREE
            total_cnt: 0,
            fifo: VecDeque::with_capacity(8),
            dma_now: 0,
            dma_beg: 0,
            dma_end: 0,
            tx_log: Vec::new(),
            rx_feed: VecDeque::new(),
            dma_active_step: false,
        }
    }

    #[inline]
    pub fn step(&mut self, _cycles: u32, pfic: &mut Pfic, sram: &mut [u8]) {
        self.dma_active_step = false;
        if self.total_cnt == 0 && self.fifo.is_empty() {
            if let Some(irqn) = self.irqn {
                pfic.sync_level_irq(irqn, (self.int_flag & self.inter_en & 0x9F) != 0);
            }
            return;
        }
        let dir_in = (self.ctrl_mod & 0x10) != 0;
        let dma_en = (self.ctrl_cfg & 0x01) != 0;

        if dma_en && self.total_cnt > 0 && self.dma_end > self.dma_beg {
            let mut rem = self.total_cnt;
            while rem > 0 && self.dma_now < self.dma_end {
                self.dma_active_step = true;
                let addr = (self.dma_now & 0x1FFFF) as usize;
                if dir_in {
                    let b = self.rx_feed.pop_front().unwrap_or(0xFF);
                    if addr < sram.len() {
                        sram[addr] = b;
                    }
                } else {
                    let b = if addr < sram.len() { sram[addr] } else { 0xFF };
                    self.tx_log.push(b);
                }
                self.dma_now += 1;
                rem -= 1;
            }
            self.total_cnt = rem;
            if rem == 0 {
                self.int_flag |= 0x01 | 0x08 | 0x40; // CNT_END | DMA_END | FREE
                if (self.ctrl_cfg & 0x04) != 0 {
                    self.dma_now = self.dma_beg;
                }
            }
        } else if !dir_in {
            while let Some(b) = self.fifo.pop_front() {
                self.tx_log.push(b);
                if self.total_cnt > 0 {
                    self.total_cnt -= 1;
                    if self.total_cnt == 0 {
                        self.int_flag |= 0x01 | 0x02 | 0x40;
                    }
                }
            }
            self.int_flag |= 0x40;
        } else {
            // Master read via FIFO
            while self.total_cnt > 0 && self.fifo.len() < 8 {
                let b = self.rx_feed.pop_front().unwrap_or(0xFF);
                self.fifo.push_back(b);
                self.total_cnt -= 1;
                if self.total_cnt == 0 {
                    self.int_flag |= 0x01 | 0x02 | 0x40;
                }
            }
        }

        if let Some(irqn) = self.irqn {
            pfic.sync_level_irq(irqn, (self.int_flag & self.inter_en & 0x9F) != 0);
        }
    }

    pub fn read_u8(&mut self, offset: u32) -> u8 {
        match offset {
            0x00 => self.ctrl_mod,
            0x01 => self.ctrl_cfg,
            0x02 => self.inter_en,
            0x03 => self.clock_div,
            0x04 => {
                if (self.ctrl_cfg & 0x10) != 0 {
                    self.int_flag &= !0x02;
                }
                self.buffer
            }
            0x05 => self.run_flag,
            0x06 => self.int_flag,
            0x07 | 0x13 => self.fifo.len() as u8,
            0x0C => self.total_cnt as u8,
            0x0D => (self.total_cnt >> 8) as u8,
            0x10 => {
                if (self.ctrl_cfg & 0x10) != 0 {
                    self.int_flag &= !0x02;
                }
                self.fifo.pop_front().unwrap_or(0xFF)
            }
            0x14..=0x17 => (self.dma_now >> ((offset - 0x14) * 8)) as u8,
            0x18..=0x1B => (self.dma_beg >> ((offset - 0x18) * 8)) as u8,
            0x1C..=0x1F => (self.dma_end >> ((offset - 0x1C) * 8)) as u8,
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        match offset {
            0x00 => {
                if (val & 0x02) != 0 {
                    self.fifo.clear();
                    self.total_cnt = 0;
                    self.int_flag = 0x40;
                }
                self.ctrl_mod = val & !0x02;
            }
            0x01 => self.ctrl_cfg = val,
            0x02 => self.inter_en = val,
            0x03 => self.clock_div = val,
            0x04 => {
                // Single-byte exchange via R8_SPI0_BUFFER
                if (self.ctrl_mod & 0x10) != 0 {
                    self.buffer = self.rx_feed.pop_front().unwrap_or(0xFF);
                } else {
                    self.tx_log.push(val);
                    self.buffer = self.rx_feed.pop_front().unwrap_or(0xFF);
                }
                self.int_flag |= 0x42; // FREE | BYTE_END
            }
            0x06 => self.int_flag = (self.int_flag & !val) | 0x40, // RW1C (FREE remains RO)
            0x0C => self.total_cnt = (self.total_cnt & 0x0F00) | (val as u16),
            0x0D => self.total_cnt = (self.total_cnt & 0x00FF) | (((val as u16) & 0x0F) << 8),
            0x10 => {
                if self.fifo.len() < 8 {
                    self.fifo.push_back(val);
                }
                if (self.ctrl_cfg & 0x10) != 0 {
                    self.int_flag &= !0x02;
                }
            }
            0x14..=0x17 => {
                let s = (offset - 0x14) * 8;
                self.dma_now = (self.dma_now & !(0xFF << s)) | ((val as u32) << s);
            }
            0x18..=0x1B => {
                let s = (offset - 0x18) * 8;
                self.dma_beg = (self.dma_beg & !(0xFF << s)) | ((val as u32) << s);
                self.dma_now = self.dma_beg;
            }
            0x1C..=0x1F => {
                let s = (offset - 0x1C) * 8;
                self.dma_end = (self.dma_end & !(0xFF << s)) | ((val as u32) << s);
            }
            _ => {}
        }
    }

    pub fn read_u32(&mut self, offset: u32) -> u32 {
        match offset {
            0x14 => self.dma_now,
            0x18 => self.dma_beg,
            0x1C => self.dma_end,
            _ => {
                let b0 = self.read_u8(offset) as u32;
                let b1 = self.read_u8(offset + 1) as u32;
                let b2 = self.read_u8(offset + 2) as u32;
                let b3 = self.read_u8(offset + 3) as u32;
                b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
            }
        }
    }

    pub fn write_u32(&mut self, offset: u32, val: u32) {
        match offset {
            0x14 => self.dma_now = val,
            0x18 => {
                self.dma_beg = val;
                self.dma_now = val;
            }
            0x1C => self.dma_end = val,
            _ => {
                let b = val.to_le_bytes();
                for i in 0..4 {
                    self.write_u8(offset + i as u32, b[i]);
                }
            }
        }
    }
}
