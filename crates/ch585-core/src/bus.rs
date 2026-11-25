use crate::pfic::{Pfic, SysTick};

/// Generic memory bus interface used by the QingKe V3C CPU.
pub trait Bus {
    fn read_u8(&mut self, addr: u32) -> u8;
    fn read_u16(&mut self, addr: u32) -> u16;
    fn read_u32(&mut self, addr: u32) -> u32;
    fn write_u8(&mut self, addr: u32, val: u8);
    fn write_u16(&mut self, addr: u32, val: u16);
    fn write_u32(&mut self, addr: u32, val: u32);

    fn pfic(&mut self) -> &mut Pfic;
    fn pfic_ref(&self) -> &Pfic;
    fn systick(&mut self) -> &mut SysTick;
    fn on_wfi(&mut self, _wait_cycles: u32) {}
}

/// Minimal flat test bus for CPU/PFIC/SysTick unit tests.
pub struct FlatBus {
    pub flash: Vec<u8>,
    pub sram: Vec<u8>,
    pub pfic: Pfic,
    pub systick: SysTick,
}

impl Default for FlatBus {
    fn default() -> Self {
        Self::new()
    }
}

impl FlatBus {
    pub const FLASH_SIZE: usize = 512 * 1024;
    pub const SRAM_SIZE: usize = 128 * 1024;

    pub fn new() -> Self {
        Self {
            flash: vec![0xFF; Self::FLASH_SIZE],
            sram: vec![0; Self::SRAM_SIZE],
            pfic: Pfic::new(),
            systick: SysTick::new(),
        }
    }

    pub fn load_flash(&mut self, addr: u32, data: &[u8]) {
        let start = addr as usize;
        if start + data.len() <= self.flash.len() {
            self.flash[start..start + data.len()].copy_from_slice(data);
        }
    }

    pub fn load_sram(&mut self, addr: u32, data: &[u8]) {
        if addr >= 0x2000_0000 {
            let start = (addr - 0x2000_0000) as usize;
            if start + data.len() <= self.sram.len() {
                self.sram[start..start + data.len()].copy_from_slice(data);
            }
        }
    }
}

impl Bus for FlatBus {
    fn read_u8(&mut self, addr: u32) -> u8 {
        if (addr as usize) < Self::FLASH_SIZE {
            self.flash[addr as usize]
        } else if (0x2000_0000..0x2000_0000 + Self::SRAM_SIZE as u32).contains(&addr) {
            self.sram[(addr - 0x2000_0000) as usize]
        } else if (0xE000_E000..0xE000_F000).contains(&addr) {
            self.pfic.read_u8(addr - 0xE000_E000)
        } else if (0xE000_F000..0xE001_0000).contains(&addr) {
            let w = self.systick.read_u32((addr - 0xE000_F000) & !3);
            ((w >> ((addr & 3) * 8)) & 0xFF) as u8
        } else {
            0
        }
    }

    fn read_u16(&mut self, addr: u32) -> u16 {
        let b0 = self.read_u8(addr) as u16;
        let b1 = self.read_u8(addr.wrapping_add(1)) as u16;
        b0 | (b1 << 8)
    }

    fn read_u32(&mut self, addr: u32) -> u32 {
        if (addr & 3) == 0 {
            if (addr as usize + 4) <= Self::FLASH_SIZE {
                let i = addr as usize;
                return u32::from_le_bytes([
                    self.flash[i],
                    self.flash[i + 1],
                    self.flash[i + 2],
                    self.flash[i + 3],
                ]);
            }
            if (0x2000_0000..0x2000_0000 + Self::SRAM_SIZE as u32 - 3).contains(&addr) {
                let i = (addr - 0x2000_0000) as usize;
                return u32::from_le_bytes([
                    self.sram[i],
                    self.sram[i + 1],
                    self.sram[i + 2],
                    self.sram[i + 3],
                ]);
            }
            if (0xE000_E000..0xE000_F000).contains(&addr) {
                return self.pfic.read_u32(addr - 0xE000_E000);
            }
            if (0xE000_F000..0xE001_0000).contains(&addr) {
                return self.systick.read_u32(addr - 0xE000_F000);
            }
        }
        let b0 = self.read_u8(addr) as u32;
        let b1 = self.read_u8(addr.wrapping_add(1)) as u32;
        let b2 = self.read_u8(addr.wrapping_add(2)) as u32;
        let b3 = self.read_u8(addr.wrapping_add(3)) as u32;
        b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
    }

    fn write_u8(&mut self, addr: u32, val: u8) {
        if (0x2000_0000..0x2000_0000 + Self::SRAM_SIZE as u32).contains(&addr) {
            self.sram[(addr - 0x2000_0000) as usize] = val;
        } else if (0xE000_E000..0xE000_F000).contains(&addr) {
            self.pfic.write_u8(addr - 0xE000_E000, val);
        } else if (0xE000_F000..0xE001_0000).contains(&addr) {
            let off = (addr - 0xE000_F000) & !3;
            let shift = (addr & 3) * 8;
            let mut w = self.systick.read_u32(off);
            w &= !(0xFF << shift);
            w |= (val as u32) << shift;
            self.systick.write_u32(off, w, &mut self.pfic);
        }
    }

    fn write_u16(&mut self, addr: u32, val: u16) {
        self.write_u8(addr, val as u8);
        self.write_u8(addr.wrapping_add(1), (val >> 8) as u8);
    }

    fn write_u32(&mut self, addr: u32, val: u32) {
        if (addr & 3) == 0 {
            if (0x2000_0000..0x2000_0000 + Self::SRAM_SIZE as u32 - 3).contains(&addr) {
                let i = (addr - 0x2000_0000) as usize;
                self.sram[i..i + 4].copy_from_slice(&val.to_le_bytes());
                return;
            }
            if (0xE000_E000..0xE000_F000).contains(&addr) {
                self.pfic.write_u32(addr - 0xE000_E000, val);
                return;
            }
            if (0xE000_F000..0xE001_0000).contains(&addr) {
                self.systick.write_u32(addr - 0xE000_F000, val, &mut self.pfic);
                return;
            }
        }
        self.write_u8(addr, val as u8);
        self.write_u8(addr.wrapping_add(1), (val >> 8) as u8);
        self.write_u8(addr.wrapping_add(2), (val >> 16) as u8);
        self.write_u8(addr.wrapping_add(3), (val >> 24) as u8);
    }

    fn pfic(&mut self) -> &mut Pfic {
        &mut self.pfic
    }

    fn pfic_ref(&self) -> &Pfic {
        &self.pfic
    }

    fn systick(&mut self) -> &mut SysTick {
        &mut self.systick
    }
}
