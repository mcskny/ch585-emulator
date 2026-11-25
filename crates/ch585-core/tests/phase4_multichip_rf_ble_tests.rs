use std::collections::HashMap;

use ch585_core::peripherals::irq;
use ch585_core::peripherals::sys::PowerMode;
use ch585_core::prelude::*;

fn root_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

fn read_u32_sram(mach: &Ch585Machine, addr: u32) -> u32 {
    let off = (addr - 0x2000_0000) as usize;
    u32::from_le_bytes(mach.bus.sram[off..off + 4].try_into().unwrap())
}

fn write_u32_sram(mach: &mut Ch585Machine, addr: u32, val: u32) {
    let off = (addr - 0x2000_0000) as usize;
    mach.bus.sram[off..off + 4].copy_from_slice(&val.to_le_bytes());
}

fn write_u16_sram(mach: &mut Ch585Machine, addr: u32, val: u16) {
    let off = (addr - 0x2000_0000) as usize;
    mach.bus.sram[off..off + 2].copy_from_slice(&val.to_le_bytes());
}

#[derive(Clone, Copy)]
enum FixupKind {
    Jal { rd: u8 },
    Branch { funct3: u32, rs1: u8, rs2: u8 },
}

struct Asm {
    base: u32,
    words: Vec<u32>,
    labels: HashMap<String, usize>,
    fixups: Vec<(usize, String, FixupKind)>,
}

impl Asm {
    fn new(base: u32) -> Self {
        Self {
            base,
            words: Vec::new(),
            labels: HashMap::new(),
            fixups: Vec::new(),
        }
    }

    fn cur_pc(&self) -> u32 {
        self.base + (self.words.len() as u32) * 4
    }

    fn label(&mut self, name: &str) -> u32 {
        let addr = self.cur_pc();
        self.labels.insert(name.to_string(), self.words.len());
        addr
    }

    fn push(&mut self, w: u32) {
        self.words.push(w);
    }

    fn lui(&mut self, rd: u8, imm20: u32) {
        self.push(((imm20 & 0xF_FFFF) << 12) | ((rd as u32) << 7) | 0x37);
    }

    fn addi(&mut self, rd: u8, rs1: u8, imm: i32) {
        self.push((((imm as u32) & 0xFFF) << 20) | ((rs1 as u32) << 15) | ((rd as u32) << 7) | 0x13);
    }

    fn andi(&mut self, rd: u8, rs1: u8, imm: i32) {
        self.push(
            (((imm as u32) & 0xFFF) << 20)
                | ((rs1 as u32) << 15)
                | (7 << 12)
                | ((rd as u32) << 7)
                | 0x13,
        );
    }

    fn add(&mut self, rd: u8, rs1: u8, rs2: u8) {
        self.push(((rs2 as u32) << 20) | ((rs1 as u32) << 15) | ((rd as u32) << 7) | 0x33);
    }

    fn or(&mut self, rd: u8, rs1: u8, rs2: u8) {
        self.push(
            ((rs2 as u32) << 20) | ((rs1 as u32) << 15) | (6 << 12) | ((rd as u32) << 7) | 0x33,
        );
    }

    fn li(&mut self, rd: u8, val: u32) {
        let hi = val.wrapping_add(0x800) >> 12;
        let lo = ((val as i32) << 20) >> 20;
        if hi == 0 {
            self.addi(rd, 0, lo);
        } else {
            self.lui(rd, hi);
            if lo != 0 {
                self.addi(rd, rd, lo);
            }
        }
    }

    fn lw(&mut self, rd: u8, rs1: u8, imm: i32) {
        self.push(
            (((imm as u32) & 0xFFF) << 20)
                | ((rs1 as u32) << 15)
                | (2 << 12)
                | ((rd as u32) << 7)
                | 0x03,
        );
    }

    fn lbu(&mut self, rd: u8, rs1: u8, imm: i32) {
        self.push(
            (((imm as u32) & 0xFFF) << 20)
                | ((rs1 as u32) << 15)
                | (4 << 12)
                | ((rd as u32) << 7)
                | 0x03,
        );
    }

    fn sw(&mut self, rs2: u8, rs1: u8, imm: i32) {
        let u = imm as u32;
        self.push(
            (((u >> 5) & 0x7F) << 25)
                | ((rs2 as u32) << 20)
                | ((rs1 as u32) << 15)
                | (2 << 12)
                | ((u & 0x1F) << 7)
                | 0x23,
        );
    }

    fn sb(&mut self, rs2: u8, rs1: u8, imm: i32) {
        let u = imm as u32;
        self.push(
            (((u >> 5) & 0x7F) << 25)
                | ((rs2 as u32) << 20)
                | ((rs1 as u32) << 15)
                | ((u & 0x1F) << 7)
                | 0x23,
        );
    }

    fn jal(&mut self, rd: u8, label: &str) {
        let idx = self.words.len();
        self.push(0);
        self.fixups.push((idx, label.to_string(), FixupKind::Jal { rd }));
    }

    fn beq(&mut self, rs1: u8, rs2: u8, label: &str) {
        let idx = self.words.len();
        self.push(0);
        self.fixups.push((
            idx,
            label.to_string(),
            FixupKind::Branch {
                funct3: 0,
                rs1,
                rs2,
            },
        ));
    }

    fn bne(&mut self, rs1: u8, rs2: u8, label: &str) {
        let idx = self.words.len();
        self.push(0);
        self.fixups.push((
            idx,
            label.to_string(),
            FixupKind::Branch {
                funct3: 1,
                rs1,
                rs2,
            },
        ));
    }

    fn ret(&mut self) {
        self.push(0x0000_8067); // jalr x0, 0(x1)
    }

    fn call_abs(&mut self, addr: u32) {
        self.li(6, addr); // t1 = addr
        self.push(0x0003_00E7); // jalr x1, 0(t1)
    }

    fn csrrw(&mut self, rd: u8, csr: u16, rs1: u8) {
        self.push(((csr as u32) << 20) | ((rs1 as u32) << 15) | (1 << 12) | ((rd as u32) << 7) | 0x73);
    }

    fn csrrsi(&mut self, rd: u8, csr: u16, zimm: u8) {
        self.push(
            ((csr as u32) << 20)
                | (((zimm as u32) & 0x1F) << 15)
                | (6 << 12)
                | ((rd as u32) << 7)
                | 0x73,
        );
    }

    fn wfi(&mut self) {
        self.push(0x1050_0073);
    }

    fn mret(&mut self) {
        self.push(0x3020_0073);
    }

    fn finish(mut self, flash: &mut [u8]) -> HashMap<String, u32> {
        for (idx, lbl, kind) in &self.fixups {
            let target_idx = self.labels[lbl];
            let imm = ((target_idx as isize) - (*idx as isize)) * 4;
            let u = imm as u32;
            self.words[*idx] = match *kind {
                FixupKind::Jal { rd } => {
                    (((u >> 20) & 1) << 31)
                        | (((u >> 1) & 0x3FF) << 21)
                        | (((u >> 11) & 1) << 20)
                        | (((u >> 12) & 0xFF) << 12)
                        | ((rd as u32) << 7)
                        | 0x6F
                }
                FixupKind::Branch { funct3, rs1, rs2 } => {
                    (((u >> 12) & 1) << 31)
                        | (((u >> 5) & 0x3F) << 25)
                        | ((rs2 as u32) << 20)
                        | ((rs1 as u32) << 15)
                        | (funct3 << 12)
                        | (((u >> 1) & 0xF) << 8)
                        | (((u >> 11) & 1) << 7)
                        | 0x63
                }
            };
        }
        let base = self.base as usize;
        for (i, w) in self.words.iter().enumerate() {
            let off = base + i * 4;
            flash[off..off + 4].copy_from_slice(&w.to_le_bytes());
        }
        self.labels
            .into_iter()
            .map(|(k, v)| (k, self.base + (v as u32) * 4))
            .collect()
    }
}

fn install_stubs(mach: &mut Ch585Machine, names: &[&str], base_addr: u32) -> HashMap<String, u32> {
    let mut map = HashMap::new();
    for (i, &name) in names.iter().enumerate() {
        let addr = base_addr + (i as u32) * 4;
        // Write fallback `ret` (`jalr x0, 0(x1)`) just in case
        mach.bus.flash[addr as usize..addr as usize + 4].copy_from_slice(&0x0000_8067u32.to_le_bytes());
        map.insert(name.to_string(), addr);
    }
    mach.addr_by_sym = map.clone();
    mach.bus.ble.resolve_symbols(&mach.addr_by_sym);
    mach.bus.flash_version = mach.bus.flash_version.wrapping_add(1);
    map
}

#[test]
fn test_multichip_gpio_wire_and_uart_duplex() {
    // Chip 0 firmware at 0x200:
    //  - Set PA4 output High (R32_PA_DIR = 0x10, R32_PA_OUT = 0x10)
    //  - Poll PB0 in R32_PB_PIN until high
    //  - Write 0xCAFE_BEEF to SRAM 0x2000_0400
    //  - Write "PING!" to UART1 THR (0x4000_3408)
    //  - Poll UART1 LSR (0x4000_3405 & 1) and read 5 bytes into SRAM 0x2000_0410..
    //  - Then set PA5 output High (for contention check later) and idle
    let mut m0 = Ch585Machine::new();
    {
        let mut a = Asm::new(0x200);
        a.li(2, 0x2001_0000); // sp
        a.li(5, 0x4000_1000); // GPIO base
        a.addi(7, 0, 0x10);
        a.sw(7, 5, 0xA0); // PA_DIR |= PA4
        a.sw(7, 5, 0xA8); // PA_OUT |= PA4 (High)

        a.label("wait_pb0");
        a.lw(8, 5, 0xC4); // R32_PB_PIN
        a.andi(8, 8, 1);
        a.beq(8, 0, "wait_pb0");

        // Record GPIO handshake success
        a.li(9, 0x2000_0400);
        a.li(10, 0xCAFE_BEEF);
        a.sw(10, 9, 0);

        // Send "PING!" on UART1 (THR=0x4000_3408)
        a.li(11, 0x4000_3400);
        for &ch in b"PING!" {
            a.addi(12, 0, ch as i32);
            a.sb(12, 11, 0x08);
        }

        // Read 5 bytes from UART1 RX into SRAM 0x2000_0410..
        a.li(13, 0x2000_0410);
        a.addi(14, 0, 5); // count
        a.label("rx_loop");
        a.lbu(15, 11, 0x05); // UART1 LSR
        a.andi(15, 15, 1);
        a.beq(15, 0, "rx_loop");
        a.lbu(16, 11, 0x08); // UART1 RBR
        a.sb(16, 13, 0);
        a.addi(13, 13, 1);
        a.addi(14, 14, -1);
        a.bne(14, 0, "rx_loop");

        // Now drive PA5 High for contention test while Chip 1 drives PA5 Low
        a.addi(7, 0, 0x30); // PA4 | PA5
        a.sw(7, 5, 0xA0);
        a.sw(7, 5, 0xA8);

        a.label("done");
        a.jal(0, "done");
        a.finish(&mut m0.bus.flash);
        m0.cpu.pc = 0x200;
    }

    // Chip 1 firmware at 0x200:
    //  - Configure PB0 + PA5 output (PA5=Low, PB0=Low initially)
    //  - Poll PA4 in R32_PA_PIN until high
    //  - Drive PB0 High
    //  - Read 5 bytes from UART1 RX into SRAM 0x2000_0410..
    //  - Transmit "PONG!" back on UART1
    let mut m1 = Ch585Machine::new();
    {
        let mut a = Asm::new(0x200);
        a.li(2, 0x2001_0000);
        a.li(5, 0x4000_1000);
        a.addi(7, 0, 0x20); // PA5 out Low
        a.sw(7, 5, 0xA0);
        a.sw(0, 5, 0xA8);
        a.addi(7, 0, 0x01); // PB0 out Low
        a.sw(7, 5, 0xC0);
        a.sw(0, 5, 0xC8);

        a.label("wait_pa4");
        a.lw(8, 5, 0xA4); // R32_PA_PIN
        a.andi(8, 8, 0x10);
        a.beq(8, 0, "wait_pa4");

        a.addi(7, 0, 1);
        a.sw(7, 5, 0xC8); // PB0 = High

        a.li(11, 0x4000_3400);
        a.li(13, 0x2000_0410);
        a.addi(14, 0, 5);
        a.label("rx_loop");
        a.lbu(15, 11, 0x05);
        a.andi(15, 15, 1);
        a.beq(15, 0, "rx_loop");
        a.lbu(16, 11, 0x08);
        a.sb(16, 13, 0);
        a.addi(13, 13, 1);
        a.addi(14, 14, -1);
        a.bne(14, 0, "rx_loop");

        for &ch in b"PONG!" {
            a.addi(12, 0, ch as i32);
            a.sb(12, 11, 0x08);
        }

        a.label("done");
        a.jal(0, "done");
        a.finish(&mut m1.bus.flash);
        m1.cpu.pc = 0x200;
    }

    let mut cluster = MultiChipCluster::new();
    cluster.quantum_us = 25;
    let c0 = cluster.add_chip("chip0", m0);
    let c1 = cluster.add_chip("chip1", m1);
    cluster.connect_gpio(GpioPinRef::new(c0, 'A', 4), GpioPinRef::new(c1, 'A', 4));
    cluster.connect_gpio(GpioPinRef::new(c0, 'B', 0), GpioPinRef::new(c1, 'B', 0));
    cluster.connect_gpio(GpioPinRef::new(c0, 'A', 5), GpioPinRef::new(c1, 'A', 5));
    cluster.connect_uart_duplex(c0, 1, c1, 1);

    cluster.run_us(2_000);

    assert_eq!(cluster.sim_time_ns, 2_000_000);
    assert_eq!(read_u32_sram(cluster.chip(c0), 0x2000_0400), 0xCAFE_BEEF);
    assert_eq!(&cluster.chip(c1).bus.sram[0x410..0x415], b"PING!");
    assert_eq!(&cluster.chip(c0).bus.sram[0x410..0x415], b"PONG!");
    assert!(
        !cluster.chip(c0).bus.gpio.contention_log.is_empty(),
        "PA5 opposing push-pull drive should record contention on chip0"
    );
    assert!(
        !cluster.chip(c1).bus.gpio.contention_log.is_empty(),
        "PA5 opposing push-pull drive should record contention on chip1"
    );
}

#[test]
fn test_rf_fast_pairing_bidirectional_dma_and_faults() {
    let stub_names = [
        "RFRole_FastInit",
        "RFRole_SetParam",
        "RFBound_SetFrequencyList",
        "RFBound_SetSpeedType",
        "RFBound_Start8kHost",
        "RFBound_Start8kDevice",
        "RFIP_ReadRssi",
        "RFIP_ReadCrc",
        "RFRole_GetStatus",
        "RFBound_Stop",
    ];

    // Helper building either Host (is_host=true) or Device (is_host=false) Fast RF firmware
    let build_node = |is_host: bool| -> Ch585Machine {
        let mut m = Ch585Machine::new();
        let syms = install_stubs(&mut m, &stub_names, 0x0100);

        // Guest callbacks & main at 0x0200:
        // SRAM layout:
        //   0x2000_1000..1040: tx_desc[4] (16B each)
        //   0x2000_1040..1080: rx_desc[4] (16B each)
        //   0x2000_1100..1200: tx_bufs (4 x 64B)
        //   0x2000_1200..1300: rx_bufs (4 x 64B)
        //   0x2000_1400: rfRoleConfig_t (20B)
        //   0x2000_1420: rfRoleParam_t (24B)
        //   0x2000_1440: freq array (1 x u32)
        //   0x2000_1448: rfHostBoundFre_t (8B)
        //   0x2000_1460: rfBoundHost_t / rfBoundDevice_t (28B)
        //   0x2000_0800: bound_cb log -> status(u32 +0), devId(+4), count(+8), peer0..3(+12)
        //   0x2000_0820: process_cb log -> mask_or(+0), last_id(+4), count(+8), rssi(+12), crc(+16)
        let mut a = Asm::new(0x0200);
        // Reserve jump to main at 0x0200
        a.jal(0, "main");

        // guest_bound_cb(staBound_t *p): a0 = p
        let bound_cb_pc = a.label("guest_bound_cb");
        a.addi(2, 2, -16);
        a.sw(1, 2, 12);
        a.sw(8, 2, 8);
        a.addi(8, 10, 0); // s0 = p
        a.li(9, 0x2000_0800);
        a.lbu(11, 8, 0); // status
        a.sw(11, 9, 0);
        a.lbu(11, 8, 2); // devId
        a.sw(11, 9, 4);
        a.lw(11, 9, 8); // count++
        a.addi(11, 11, 1);
        a.sw(11, 9, 8);
        a.lw(11, 8, 6); // PeerInfo[0..4]
        a.sw(11, 9, 12);
        a.lw(8, 2, 8);
        a.lw(1, 2, 12);
        a.addi(2, 2, 16);
        a.ret();

        // guest_process_cb(uint32_t sta, uint8_t id)
        let proc_cb_pc = a.label("guest_process_cb");
        a.addi(2, 2, -16);
        a.sw(1, 2, 12);
        a.sw(8, 2, 8);
        a.sw(9, 2, 4);
        a.addi(8, 10, 0); // s0 = sta
        a.addi(9, 11, 0); // s1 = id
        a.li(7, 0x2000_0820);
        a.lw(12, 7, 0);
        a.or(12, 12, 8);
        a.sw(12, 7, 0); // status_or
        a.sw(9, 7, 4);  // last_id
        a.lw(12, 7, 8);
        a.addi(12, 12, 1);
        a.sw(12, 7, 8); // count++
        a.call_abs(syms["RFIP_ReadRssi"]);
        a.li(7, 0x2000_0820);
        a.sw(10, 7, 12); // signed i8 RSSI sign-extended in a0
        a.call_abs(syms["RFIP_ReadCrc"]);
        a.li(7, 0x2000_0820);
        a.sw(10, 7, 16); // crc_ok
        a.lw(9, 2, 4);
        a.lw(8, 2, 8);
        a.lw(1, 2, 12);
        a.addi(2, 2, 16);
        a.ret();

        a.label("main");
        a.li(2, 0x2001_0000);
        a.li(10, 0x2000_1400);
        a.call_abs(syms["RFRole_FastInit"]);
        a.li(10, 0x2000_1420);
        a.call_abs(syms["RFRole_SetParam"]);
        a.li(10, 0x2000_1448);
        a.call_abs(syms["RFBound_SetFrequencyList"]);
        if is_host {
            a.li(10, 0x2000_1460);
            a.call_abs(syms["RFBound_Start8kHost"]);
        } else {
            a.li(10, 0x2000_1460);
            a.call_abs(syms["RFBound_Start8kDevice"]);
        }
        a.label("idle");
        a.call_abs(syms["RFRole_GetStatus"]);
        a.li(7, 0x2000_0840);
        a.sw(10, 7, 0);
        a.jal(0, "idle");

        a.finish(&mut m.bus.flash);
        m.cpu.pc = 0x0200;

        // Initialize TX and RX circular DMA descriptor rings in SRAM
        for idx in 0..4u32 {
            let tx_desc = 0x2000_1000 + idx * 16;
            let next_tx = 0x2000_1000 + ((idx + 1) % 4) * 16;
            write_u32_sram(&mut m, tx_desc, 0); // idle
            write_u32_sram(&mut m, tx_desc + 4, 64);
            write_u32_sram(&mut m, tx_desc + 8, 0x2000_1100 + idx * 64);
            write_u32_sram(&mut m, tx_desc + 12, next_tx);

            let rx_desc = 0x2000_1040 + idx * 16;
            let next_rx = 0x2000_1040 + ((idx + 1) % 4) * 16;
            write_u32_sram(&mut m, rx_desc, STA_DMA_ENABLE | 64);
            write_u32_sram(&mut m, rx_desc + 4, 64);
            write_u32_sram(&mut m, rx_desc + 8, 0x2000_1200 + idx * 64);
            write_u32_sram(&mut m, rx_desc + 12, next_rx);
        }

        // rfRoleConfig_t at 0x2000_1400
        write_u32_sram(&mut m, 0x2000_1400, 4); // TxPower=4
        write_u32_sram(&mut m, 0x2000_1404, 0x2000_1000);
        write_u32_sram(&mut m, 0x2000_1408, 0x2000_1040);
        write_u32_sram(&mut m, 0x2000_140C, proc_cb_pc);
        write_u32_sram(
            &mut m,
            0x2000_1410,
            RF_STATE_RX | RF_STATE_RX_CRCERR | RF_STATE_TX_FINISH | RF_STATE_TIMEOUT | RF_STATE_RBU,
        );

        // rfRoleParam_t at 0x2000_1420
        write_u32_sram(&mut m, 0x2000_1420, 0x7176_2345);
        write_u32_sram(&mut m, 0x2000_1424, 0x0055_5555);
        write_u32_sram(&mut m, 0x2000_1428, 2_402_000); // ch 1 -> index 1
        write_u32_sram(&mut m, 0x2000_142C, 0x20);
        write_u16_sram(&mut m, 0x2000_1430, 64);

        // rfHostBoundFre_t at 0x2000_1448
        write_u32_sram(&mut m, 0x2000_1440, 2_402_000);
        write_u32_sram(&mut m, 0x2000_1448, 1);
        write_u32_sram(&mut m, 0x2000_144C, 0x2000_1440);

        if is_host {
            // rfBoundHost_t at 0x2000_1460
            write_u32_sram(&mut m, 0x2000_1460, (8 << 16) | 1); // hop=1, periTime=8
            write_u16_sram(&mut m, 0x2000_1464, 20); // timeout = 20 ms
            // OwnInfo = [0xD0, 0x01, 0x02, 0x03, 0x04, 0x05]
            m.bus.sram[0x1466..0x146C].copy_from_slice(&[0xD0, 0x01, 0x02, 0x03, 0x04, 0x05]);
            write_u32_sram(&mut m, 0x2000_1474, bound_cb_pc);
        } else {
            // rfBoundDevice_t at 0x2000_1460: devType=0, deviceId=7 (RF_BOUND_ID), speed=8
            write_u32_sram(&mut m, 0x2000_1460, (8 << 24) | ((RF_BOUND_ID as u32) << 8));
            write_u16_sram(&mut m, 0x2000_1464, 20); // timeout = 20 ms
            m.bus.sram[0x1466..0x146C].copy_from_slice(&[0x4B, 0x42, 0x44, 0x00, 0x00, 0x01]);
            write_u32_sram(&mut m, 0x2000_1474, bound_cb_pc);
        }
        m
    };

    let mut cluster = MultiChipCluster::with_rf_config(RfMediumConfig {
        delay_us: 150,
        jitter_us: 50,
        packet_loss_pct: 0.0,
        base_rssi_dbm: -42,
        ..Default::default()
    });
    cluster.quantum_us = 25;
    let host = cluster.add_chip("dongle", build_node(true));
    let dev = cluster.add_chip("keyboard", build_node(false));

    // 1. Pairing phase (deviceId = 7 -> BOUND_STA_SUCCESS on both guests)
    cluster.run_us(3_000);
    assert_eq!(read_u32_sram(cluster.chip(host), 0x2000_0800), BOUND_STA_SUCCESS as u32);
    assert_eq!(read_u32_sram(cluster.chip(host), 0x2000_0808), 1); // host bound cb called once
    assert_eq!(read_u32_sram(cluster.chip(dev), 0x2000_0800), BOUND_STA_SUCCESS as u32);
    assert_eq!(read_u32_sram(cluster.chip(dev), 0x2000_0808), 1); // dev bound cb called once
    assert_eq!(read_u32_sram(cluster.chip(host), 0x2000_0840), 1); // RFRole_GetStatus == 1
    assert_eq!(cluster.chip(host).bus.ble.rf.peer_info, [0x4B, 0x42, 0x44, 0x00, 0x00, 0x01]);
    assert_eq!(cluster.chip(dev).bus.ble.rf.peer_info, [0xD0, 0x01, 0x02, 0x03, 0x04, 0x05]);

    // 2. Device -> Host TX DMA ring with 150..200 us air propagation latency
    let payload_d2h = [0xAA, 0x55, 0x01, 0x02, 0x03, 0x04];
    cluster.chip_mut(dev).bus.sram[0x1100..0x1106].copy_from_slice(&payload_d2h);
    write_u32_sram(
        cluster.chip_mut(dev),
        0x2000_1000,
        STA_DMA_ENABLE | (payload_d2h.len() as u32),
    );

    // At +50 us, packet is queued/in-flight and NOT delivered yet (delay >= 150 us)
    cluster.run_us(50);
    assert_eq!(read_u32_sram(cluster.chip(dev), 0x2000_1000) & STA_DMA_ENABLE, 0);
    assert_eq!(read_u32_sram(cluster.chip(host), 0x2000_0828), 0);

    // By +250 us total (300 us from start of step 2), packet arrives at Host RX DMA ring
    cluster.run_us(250);
    assert_ne!(read_u32_sram(cluster.chip(dev), 0x2000_0820) & RF_STATE_TX_FINISH, 0);
    assert_ne!(read_u32_sram(cluster.chip(host), 0x2000_0820) & RF_STATE_RX, 0);
    assert_eq!(
        read_u32_sram(cluster.chip(host), 0x2000_1040),
        payload_d2h.len() as u32
    );
    assert_eq!(&cluster.chip(host).bus.sram[0x1200..0x1206], &payload_d2h);

    // 3. Host -> Device reply over Host TX DMA ring
    let payload_h2d = [0xDE, 0xAD, 0xBE, 0xEF];
    cluster.chip_mut(host).bus.sram[0x1100..0x1104].copy_from_slice(&payload_h2d);
    write_u32_sram(
        cluster.chip_mut(host),
        0x2000_1000,
        STA_DMA_ENABLE | (payload_h2d.len() as u32),
    );
    cluster.run_us(300);
    assert_ne!(read_u32_sram(cluster.chip(dev), 0x2000_0820) & RF_STATE_RX, 0);
    assert_eq!(&cluster.chip(dev).bus.sram[0x1200..0x1204], &payload_h2d);
    assert_eq!(read_u32_sram(cluster.chip(dev), 0x2000_082C) as i32, -42);

    // 4. Channel interference -> RF_STATE_RX_CRCERR + degraded RSSI
    let ch = ((2_402_000u32 - 2_400_000) / 2_000) as usize;
    cluster.rf_medium.config.channel_interference_dbm[ch] = 64;
    cluster.chip_mut(host).bus.sram[0x1140..0x1144].copy_from_slice(&[1, 2, 3, 4]);
    write_u32_sram(cluster.chip_mut(host), 0x2000_1010, STA_DMA_ENABLE | 4);
    cluster.run_us(300);
    assert_ne!(
        read_u32_sram(cluster.chip(dev), 0x2000_0820) & RF_STATE_RX_CRCERR,
        0
    );
    assert_eq!(read_u32_sram(cluster.chip(dev), 0x2000_0830), 0); // RFIP_ReadCrc() == 0
    assert!((read_u32_sram(cluster.chip(dev), 0x2000_082C) as i32) < -50);

    // 5. Packet loss 50% across 20 frames -> partial drops & partial deliveries
    cluster.rf_medium.config.channel_interference_dbm[ch] = 0;
    cluster.rf_medium.config.delay_us = 10;
    cluster.rf_medium.config.jitter_us = 0;
    cluster.rf_medium.config.packet_loss_pct = 50.0;
    for i in 0..20 {
        let slot = i % 4;
        // Re-arm RX on host so RBU does not happen
        let rx_desc = 0x2000_1040 + (slot as u32) * 16;
        write_u32_sram(cluster.chip_mut(host), rx_desc, STA_DMA_ENABLE | 64);
        let tx_desc = 0x2000_1000 + (slot as u32) * 16;
        cluster.chip_mut(dev).bus.sram[0x1100 + slot * 64] = i as u8;
        write_u32_sram(cluster.chip_mut(dev), tx_desc, STA_DMA_ENABLE | 4);
        cluster.run_us(100);
    }
    assert!(cluster.rf_medium.total_dropped_loss > 0);
    assert!(cluster.rf_medium.total_delivered >= 5);

    // 6. Link drop & timeout (> 20 ms) -> guest bound_cb gets BOUND_STA_TIMEOUT (0x17)
    cluster.rf_medium.config.packet_loss_pct = 0.0;
    cluster.rf_medium.link_dropped = true;
    cluster.run_us(25_000);
    assert_eq!(
        read_u32_sram(cluster.chip(host), 0x2000_0800),
        BOUND_STA_TIMEOUT as u32
    );
    assert_ne!(
        read_u32_sram(cluster.chip(host), 0x2000_0820) & RF_STATE_TIMEOUT,
        0
    );
    assert_eq!(
        read_u32_sram(cluster.chip(dev), 0x2000_0800),
        BOUND_STA_TIMEOUT as u32
    );
    assert!(!cluster.chip(host).bus.ble.rf.bound_connected);
    assert!(!cluster.chip(dev).bus.ble.rf.bound_connected);
}

#[test]
fn test_rf_basic_tx_rx_and_rssi() {
    let stub_names = [
        "RFRole_BasicInit",
        "RFRole_SetParam",
        "RFIP_SetTxPower",
        "RFIP_SetTxStart",
        "RFIP_SetTxParm",
        "RFIP_SetRx",
        "RFIP_ReadRssi",
        "RFIP_ReadCrc",
    ];

    let build_basic = |is_tx: bool| -> Ch585Machine {
        let mut m = Ch585Machine::new();
        let syms = install_stubs(&mut m, &stub_names, 0x0100);
        let mut a = Asm::new(0x0200);
        a.jal(0, "main");

        // process_cb(sta, id): save sta_or -> 0x2000_0900, rssi -> 0x2000_0904, crc -> 0x2000_0908
        let cb_pc = a.label("basic_cb");
        a.addi(2, 2, -16);
        a.sw(1, 2, 12);
        a.li(7, 0x2000_0900);
        a.lw(12, 7, 0);
        a.or(12, 12, 10);
        a.sw(12, 7, 0);
        a.call_abs(syms["RFIP_ReadRssi"]);
        a.li(7, 0x2000_0900);
        a.sw(10, 7, 4);
        a.call_abs(syms["RFIP_ReadCrc"]);
        a.li(7, 0x2000_0900);
        a.sw(10, 7, 8);
        a.lw(1, 2, 12);
        a.addi(2, 2, 16);
        a.ret();

        a.label("main");
        a.li(2, 0x2001_0000);
        a.li(10, 0x2000_1400);
        a.call_abs(syms["RFRole_BasicInit"]);
        a.li(10, 0x2000_1420);
        a.call_abs(syms["RFRole_SetParam"]);
        a.addi(10, 0, 4);
        a.call_abs(syms["RFIP_SetTxPower"]);
        if is_tx {
            a.call_abs(syms["RFIP_SetTxStart"]);
            a.li(10, 0x2000_1480); // rfipTx_t
            a.call_abs(syms["RFIP_SetTxParm"]);
        } else {
            a.li(10, 0x2000_14A0); // rfipRx_t
            a.call_abs(syms["RFIP_SetRx"]);
        }
        a.label("idle");
        a.jal(0, "idle");
        a.finish(&mut m.bus.flash);
        m.cpu.pc = 0x0200;

        write_u32_sram(&mut m, 0x2000_1400, 4);
        write_u32_sram(&mut m, 0x2000_140C, cb_pc);
        write_u32_sram(
            &mut m,
            0x2000_1410,
            RF_STATE_RX | RF_STATE_RX_CRCERR | RF_STATE_TX_FINISH | RF_STATE_TIMEOUT,
        );
        // rfRoleParam_t
        write_u32_sram(&mut m, 0x2000_1420, 0x1234_ABCD);
        write_u32_sram(&mut m, 0x2000_1424, 0x555555);
        write_u32_sram(&mut m, 0x2000_1428, 2_440_000);

        if is_tx {
            // TxBuf at 0x2000_1600: [0x55, 4, 0x11, 0x22, 0x33, 0x44]
            m.bus.sram[0x1600..0x1606].copy_from_slice(&[0x55, 4, 0x11, 0x22, 0x33, 0x44]);
            // rfipTx_t at 0x2000_1480
            write_u32_sram(&mut m, 0x2000_1480, 0x1234_ABCD);
            write_u32_sram(&mut m, 0x2000_1484, 0x555555);
            write_u32_sram(&mut m, 0x2000_1488, 2_440_000);
            write_u32_sram(&mut m, 0x2000_1490, 0x2000_1600);
        } else {
            // rfipRx_t at 0x2000_14A0
            write_u32_sram(&mut m, 0x2000_14A0, 0x1234_ABCD);
            write_u32_sram(&mut m, 0x2000_14A4, 0x555555);
            write_u32_sram(&mut m, 0x2000_14A8, 2_440_000);
            write_u32_sram(&mut m, 0x2000_14B0, 0x2000_1700); // rxDMA
            write_u16_sram(&mut m, 0x2000_14B6, 32); // rxMaxLen
            write_u16_sram(&mut m, 0x2000_14B8, 0);  // no timeout
        }
        m
    };

    // Important: start RX before TX sends, or delay > quantum ensures RX is armed before delivery!
    let mut cluster = MultiChipCluster::with_rf_config(RfMediumConfig {
        delay_us: 100,
        base_rssi_dbm: -38,
        ..Default::default()
    });
    cluster.quantum_us = 25;
    let rx_chip = cluster.add_chip("rx", build_basic(false));
    let tx_chip = cluster.add_chip("tx", build_basic(true));

    cluster.run_us(500);
    assert_ne!(
        read_u32_sram(cluster.chip(tx_chip), 0x2000_0900) & RF_STATE_TX_FINISH,
        0
    );
    assert_ne!(
        read_u32_sram(cluster.chip(rx_chip), 0x2000_0900) & RF_STATE_RX,
        0
    );
    assert_eq!(read_u32_sram(cluster.chip(rx_chip), 0x2000_0904) as i32, -38);
    assert_eq!(read_u32_sram(cluster.chip(rx_chip), 0x2000_0908), 1);
    assert_eq!(
        &cluster.chip(rx_chip).bus.sram[0x1700..0x1706],
        &[0x55, 4, 0x11, 0x22, 0x33, 0x44]
    );
}

#[test]
fn test_ble_virtual_central_full_lifecycle_and_flash_bond_persistence() {
    let elf_path = root_dir().join("kaynak/test_elf/ble_klavye.elf");
    let Ok(elf_bytes) = std::fs::read(&elf_path) else {
        eprintln!("Skipping test_ble_virtual_central_full_lifecycle_and_flash_bond_persistence: optional ELF fixture not found");
        return;
    };

    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf_bytes).unwrap();

    // 1. Boot & advertise
    mach.run_us(80_000);
    let uart = String::from_utf8_lossy(&mach.bus.uarts[0].tx_log).to_string();
    assert!(uart.contains("Initialized.."), "UART0={uart}");
    assert!(uart.contains("Advertising.."), "UART0={uart}");
    assert!(mach.bus.ble.advertising_enabled);

    // 2. Scan
    let scan = mach.ble_scan().expect("Peripheral should be advertising");
    assert!(scan.advertising);
    assert!(!scan.scan_rsp_data.is_empty());

    // 3. Connect -> guest prints "Connected.."
    assert!(mach.ble_connect([0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x01]));
    mach.run_us(20_000);
    let uart = String::from_utf8_lossy(&mach.bus.uarts[0].tx_log).to_string();
    assert!(uart.contains("Connected.."), "UART0={uart}");
    assert_eq!(mach.bus.ble.central_state, BleCentralState::Connected);

    // 4. Pair & Bond -> persisted to DataFlash SNV
    assert!(mach.ble_pair_and_bond());
    mach.run_us(10_000);
    assert_eq!(mach.bus.ble.central_state, BleCentralState::PairedBonded);
    assert_eq!(mach.bus.ble.bonds.len(), 1);
    assert_eq!(&mach.bus.flash[0x77000..0x77008], b"CH58BOND");

    // 5. GATT read HID Report Map (UUID 0x2A4B) via HidDev_ReadAttrCB
    let read_tag = mach
        .ble_gatt_read_uuid(0x2A4B)
        .expect("HID Report Map attribute must be discovered");
    mach.run_us(2_000);
    let read_res = mach
        .cpu
        .last_callback_results
        .iter()
        .find(|r| r.tag == read_tag)
        .expect("GATT read callback must complete")
        .clone();
    assert_eq!(read_res.ret_a0, 0);
    assert_eq!(&read_res.out_data[..4], &[0x05, 0x01, 0x09, 0x06]); // Usage Page Generic Desktop, Usage Keyboard

    // 6. GATT write HID Keyboard LED Output Report (UUID 0x2A4D, writable)
    assert!(mach.ble_gatt_write_uuid(0x2A4D, &[0x03]));
    mach.run_us(2_000);

    // 7. Enable CCCD notifications -> triggers TMOS report timer -> captures timestamped HID reports
    let cccd_cnt = mach.ble_enable_notifications();
    assert!(cccd_cnt >= 3, "Expected >=3 HID/BAS CCCDs, got {cccd_cnt}");
    mach.run_us(420_000);
    assert!(
        mach.bus.ble.timestamped_hid_reports.len() >= 2,
        "Expected key press + release HID reports, got {:?}",
        mach.bus.ble.timestamped_hid_reports
    );
    assert!(mach.bus.ble.timestamped_hid_reports[0].time_ns > 100_000_000);
    assert_eq!(mach.bus.ble.timestamped_hid_reports[0].data[2], 0x04); // 'A'
    assert_eq!(mach.bus.ble.timestamped_hid_reports[1].data[2], 0x00); // release

    // 8. Connection parameter update
    assert!(mach.ble_update_conn_params(24, 2, 400));
    mach.run_us(10_000);
    assert_eq!(
        mach.bus.ble.conn_params,
        BleConnParams {
            interval: 24,
            latency: 2,
            timeout: 400,
        }
    );

    // 9. Disconnect -> guest logs "Disconnected.. Reason:13" and restarts advertising
    assert!(mach.ble_disconnect(0x13));
    mach.run_us(20_000);
    let uart = String::from_utf8_lossy(&mach.bus.uarts[0].tx_log).to_string();
    assert!(uart.contains("Disconnected.. Reason:13"), "UART0={uart}");
    assert!(mach.bus.ble.advertising_enabled);

    // 10. Cold reboot / flash persistence -> reload DataFlash SNV bond and reconnect
    let saved_dataflash = mach.dataflash_bytes().to_vec();
    let mut rebooted = Ch585Machine::new();
    rebooted.load_elf(&elf_bytes).unwrap();
    rebooted.load_flash_overlay_dataflash(&saved_dataflash);
    rebooted.run_us(80_000);
    assert_eq!(
        rebooted.bus.ble.bonds.len(),
        1,
        "Bond persisted in DataFlash SNV must survive reboot"
    );
    assert!(rebooted.ble_reconnect());
    rebooted.run_us(400_000);
    let uart2 = String::from_utf8_lossy(&rebooted.bus.uarts[0].tx_log).to_string();
    assert!(uart2.contains("Connected.."), "UART0 after reconnect={uart2}");
    assert_eq!(rebooted.bus.ble.central_state, BleCentralState::PairedBonded);
    assert!(
        rebooted.bus.ble.timestamped_hid_reports.len() >= 2,
        "Reconnected bonded device should resume sending HID reports"
    );
    assert!(rebooted.bus.unknown_mmio.is_empty());
}

#[test]
fn test_ble_tmos_and_lowpower_sleep_interaction() {
    let stub_names = ["CH58x_BLEInit", "tmos_start_task", "CH58x_LowPower", "LowPower_Sleep_WFE"];
    let mut m = Ch585Machine::new();
    let syms = install_stubs(&mut m, &stub_names, 0x0100);

    // Put a real `wfi; ret` inside `LowPower_Sleep_WFE` at its symbol address
    let lpw_addr = syms["LowPower_Sleep_WFE"] as usize;
    // Wait: other stubs occupy consecutive 4B slots at 0x0100..0x0110, so move LowPower_Sleep_WFE!
    let lpw_fn = 0x0140u32;
    m.addr_by_sym.insert("LowPower_Sleep_WFE".into(), lpw_fn);
    m.bus.ble.resolve_symbols(&m.addr_by_sym);

    let mut a = Asm::new(0x0200);
    a.jal(0, "main");

    let _ = a.label("rtc_isr");
    // Clear RTC TRIG_FLAG (write 0x20 to 0x4000_1030) and increment 0x2000_0900
    a.li(5, 0x4000_1030);
    a.addi(7, 0, 0x20);
    a.sb(7, 5, 0);
    a.li(9, 0x2000_0900);
    a.lw(10, 9, 0);
    a.addi(10, 10, 1);
    a.sw(10, 9, 0);
    a.mret();

    // Put LowPower_Sleep_WFE implementation right here, aligned cleanly
    let lpw_label = a.label("lp_sleep_impl");
    a.wfi();
    a.ret();

    // Put CH58x_LowPower implementation: configure Sleep + RTC wakeup +320 ticks (~10ms), enable MIE, call LowPower_Sleep_WFE
    let ch58x_lp_label = a.label("ch58x_lp_impl");
    a.addi(2, 2, -16);
    a.sw(1, 2, 12);
    a.li(5, 0x4000_1000);
    // Unlock safe access (0x57, 0xA8 to 0x4000_1040)
    a.addi(7, 0, 0x57);
    a.sb(7, 5, 0x40);
    a.addi(7, 0, 0xA8u8 as i8 as i32);
    a.sb(7, 5, 0x40);
    // RB_SLP_RTC_WAKE (0x08 -> 0x4000_100E)
    a.addi(7, 0, 0x08);
    a.sb(7, 5, 0x0E);
    // Power plan Sleep (0x8004 -> 0x4000_1020)
    a.li(7, 0x8004);
    a.sw(7, 5, 0x20);
    // Set RTC_TRIG = cur + a0 (320 ticks)
    a.lw(7, 5, 0x38); // R32_RTC_CNT_32K + 2S
    a.add(7, 7, 10);
    a.sw(7, 5, 0x34);
    // Safe unlock + enable RTC_TRIG_EN (0x20 -> 0x4000_1031)
    a.addi(7, 0, 0x57);
    a.sb(7, 5, 0x40);
    a.addi(7, 0, 0xA8u8 as i8 as i32);
    a.sb(7, 5, 0x40);
    a.addi(7, 0, 0x20);
    a.sb(7, 5, 0x31);
    // Clear pending flags (0x30 -> 0x4000_1030)
    a.addi(7, 0, 0x30);
    a.sb(7, 5, 0x30);
    // Set PFIC SCTLR SLEEPDEEP (bit 2 = 0x04 at 0xE000_ED10)
    a.li(6, 0xE000_ED10);
    a.addi(7, 0, 0x04);
    a.sw(7, 6, 0);
    // Enable RTC IRQ (28) in PFIC IENR[0] (0xE000_E100)
    a.li(6, 0xE000_E100);
    a.li(7, 1 << irq::RTC);
    a.sw(7, 6, 0);
    // Enable global interrupts (mstatus MIE = 1<<3)
    a.csrrsi(0, 0x300, 8);
    // Call LowPower_Sleep_WFE (patched below once label resolved)
    a.jal(1, "lp_sleep_impl");
    a.lw(1, 2, 12);
    a.addi(2, 2, 16);
    a.ret();

    a.label("main");
    a.li(2, 0x2001_0000);
    // Set mtvec = 0x0000_0003 (Mode 3 vector table at 0)
    a.addi(7, 0, 3);
    a.csrrw(0, 0x305, 7);
    a.call_abs(syms["CH58x_BLEInit"]);
    // tmos_start_task(1, 0x0001, 80) -> ~50ms
    a.addi(10, 0, 1);
    a.addi(11, 0, 1);
    a.addi(12, 0, 80);
    a.call_abs(syms["tmos_start_task"]);
    // Loop calling CH58x_LowPower(320) -> ~10ms sleep per wake
    a.label("sleep_loop");
    a.addi(10, 0, 320);
    a.jal(1, "ch58x_lp_impl");
    a.jal(0, "sleep_loop");

    let lbls = a.finish(&mut m.bus.flash);
    let _ = lpw_addr;
    let _ = lpw_label;
    let _ = ch58x_lp_label;

    // Install RTC vector table entry (irq 28 -> 28*4 = 0x70) and rebind symbol hooks to real code
    m.bus.flash[0x70..0x74].copy_from_slice(&lbls["rtc_isr"].to_le_bytes());
    m.addr_by_sym.insert("CH58x_LowPower".into(), lbls["ch58x_lp_impl"]);
    m.addr_by_sym.insert("LowPower_Sleep_WFE".into(), lbls["lp_sleep_impl"]);
    m.bus.ble.resolve_symbols(&m.addr_by_sym);
    m.bus.flash_version = m.bus.flash_version.wrapping_add(1);
    m.cpu.pc = 0x0200;

    m.run_us(35_000);

    assert_eq!(m.bus.ble.tmos_start_calls, 1);
    assert!(m.bus.ble.low_power_calls >= 2);
    assert!(m.bus.ble.sleep_hook_calls >= 2);
    assert!(m.bus.sleep_skipped_ns > 15_000_000, "sleep_skipped_ns={}", m.bus.sleep_skipped_ns);
    assert!(
        m.bus.time_ns_by_mode[PowerMode::Sleep as usize] > 15_000_000,
        "Sleep mode time={:?}",
        m.bus.time_ns_by_mode
    );
    assert!(read_u32_sram(&m, 0x2000_0900) >= 2, "RTC wakeup ISR must fire repeatedly");
}
