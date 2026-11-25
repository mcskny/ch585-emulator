use ch585_core::bus::{Bus, FlatBus};
use ch585_core::cpu::decode::{decode_16, decode_32, Insn};
use ch585_core::cpu::{Cpu, Privilege};
use ch585_core::elf::ElfImage;

#[test]
fn test_xw_encodings_match_wch_assembler() {
    // Opcodes generated directly by riscv-wch-elf-as -march=rv32imc_xw:
    // 0x2000: c.lbu x8, 0(x8)
    // 0x3004: c.lbu x9, 1(x8)
    // 0x3ff8: c.lbu x14, 31(x15)
    assert_eq!(decode_16(0x2000), Insn::Lbu { rd: 8, rs1: 8, imm: 0 });
    assert_eq!(decode_16(0x3004), Insn::Lbu { rd: 9, rs1: 8, imm: 1 });
    assert_eq!(decode_16(0x3ff8), Insn::Lbu { rd: 14, rs1: 15, imm: 31 });

    // c.lhu x8, 0(x8)=0x2002, x9, 2(x8)=0x2026, x13, 32(x8)=0x3016, x14, 62(x15)=0x3ffa
    assert_eq!(decode_16(0x2002), Insn::Lhu { rd: 8, rs1: 8, imm: 0 });
    assert_eq!(decode_16(0x2026), Insn::Lhu { rd: 9, rs1: 8, imm: 2 });
    assert_eq!(decode_16(0x3016), Insn::Lhu { rd: 13, rs1: 8, imm: 32 });
    assert_eq!(decode_16(0x3ffa), Insn::Lhu { rd: 14, rs1: 15, imm: 62 });

    // c.sb x8, 0(x8)=0xa000, x14, 31(x15)=0xbff8
    assert_eq!(decode_16(0xa000), Insn::Sb { rs1: 8, rs2: 8, imm: 0 });
    assert_eq!(decode_16(0xbff8), Insn::Sb { rs1: 15, rs2: 14, imm: 31 });

    // c.sh x8, 0(x8)=0xa002, x14, 62(x15)=0xbffa
    assert_eq!(decode_16(0xa002), Insn::Sh { rs1: 8, rs2: 8, imm: 0 });
    assert_eq!(decode_16(0xbffa), Insn::Sh { rs1: 15, rs2: 14, imm: 62 });

    // c.lbusp x8, 0(sp)=0x8000, x13, 15(sp)=0x8794
    assert_eq!(decode_16(0x8000), Insn::Lbu { rd: 8, rs1: 2, imm: 0 });
    assert_eq!(decode_16(0x8794), Insn::Lbu { rd: 13, rs1: 2, imm: 15 });

    // c.lhusp x8, 0(sp)=0x8020, x12, 16(sp)=0x80b0, x13, 30(sp)=0x87b4
    assert_eq!(decode_16(0x8020), Insn::Lhu { rd: 8, rs1: 2, imm: 0 });
    assert_eq!(decode_16(0x80b0), Insn::Lhu { rd: 12, rs1: 2, imm: 16 });
    assert_eq!(decode_16(0x87b4), Insn::Lhu { rd: 13, rs1: 2, imm: 30 });

    // c.sbsp x8, 0(sp)=0x8040, x13, 15(sp)=0x87d4
    assert_eq!(decode_16(0x8040), Insn::Sb { rs1: 2, rs2: 8, imm: 0 });
    assert_eq!(decode_16(0x87d4), Insn::Sb { rs1: 2, rs2: 13, imm: 15 });

    // c.shsp x8, 0(sp)=0x8060, x13, 30(sp)=0x87f4
    assert_eq!(decode_16(0x8060), Insn::Sh { rs1: 2, rs2: 8, imm: 0 });
    assert_eq!(decode_16(0x87f4), Insn::Sh { rs1: 2, rs2: 13, imm: 30 });

    // mcpy a2(12), a1(11), a5(15) = 0x78b6700f
    assert_eq!(
        decode_32(0x78b6_700f),
        Insn::Mcpy { rd: 12, rs1: 11, rs2: 15 }
    );
}

#[test]
fn test_xw_and_mcpy_execution() {
    let mut bus = FlatBus::new();
    let mut cpu = Cpu::new();
    cpu.regs[2] = 0x2000_1000; // sp
    cpu.regs[15] = 0x2000_2000; // a5

    // Write sequence of XW instructions at 0x0:
    // c.li a4, 0xAB -> 0x4705? Actually use addi/sb directly:
    // Let's put 0xAB into x14, execute c.sb x14, 31(x15) [0xbff8],
    // then c.lbu x8, 31(x15) [0x3fc0? rs1=15(7)->(7<<7)=0x380, rd=8(0), uimm=31->0x1c60|0x2000=0x3c60|0x380=0x3fe0]
    // Even simpler: use 0x3ff8 -> rd=14, rs1=15, imm=31!
    cpu.regs[14] = 0x1234_56AB;
    let prog: [u16; 7] = [
        0xbff8, // c.sb x14, 31(x15) -> [0x2000_201F] = 0xAB
        0x87d4, // c.sbsp x13, 15(sp) -> [sp + 15] = regs[13] as u8
        0x87f4, // c.shsp x13, 30(sp) -> [sp + 30] = regs[13] as u16
        0x3fe0, // c.lbu x8, 31(x15) -> x8 = 0xAB
        0x8780, // c.lbusp x8, 15(sp)? Wait: x8 is rd=8 (bits 4:2=0), imm=15 (bits 10:7=15 -> 0x0780) => 0x8780
        0x87a0, // c.lhusp x8, 30(sp)? bits 6:5=01 (0x20), imm=30 -> 0x87a0
        0x0001, // c.nop
    ];
    cpu.regs[13] = 0xBEEF_CAFE;
    for (i, &w) in prog.iter().enumerate() {
        bus.load_flash((i * 2) as u32, &w.to_le_bytes());
    }

    cpu.step(&mut bus); // c.sb
    assert_eq!(bus.read_u8(0x2000_201F), 0xAB);

    cpu.step(&mut bus); // c.sbsp
    assert_eq!(bus.read_u8(0x2000_100F), 0xFE);

    cpu.step(&mut bus); // c.shsp
    assert_eq!(bus.read_u16(0x2000_101E), 0xCAFE);

    // c.lbu x8, 31(x15):
    cpu.regs[8] = 0;
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[8], 0xAB);

    // c.lbusp x8, 15(sp):
    cpu.regs[8] = 0;
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[8], 0xFE);

    // c.lhusp x8, 30(sp):
    cpu.regs[8] = 0;
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[8], 0xCAFE);

    // Test mcpy a2, a1, a5 (0x78b6700f)
    bus.write_u32(0x2000_3000, 0xDEAD_BEEF);
    bus.write_u16(0x2000_3004, 0x1234);
    cpu.regs[11] = 0x2000_3000; // start
    cpu.regs[12] = 0x2000_3006; // end
    cpu.regs[15] = 0x2000_3100; // dst
    let mcpy_pc = 0x100u32;
    bus.load_flash(mcpy_pc, &0x78b6_700fu32.to_le_bytes());
    cpu.pc = mcpy_pc;
    cpu.step(&mut bus);
    assert_eq!(bus.read_u32(0x2000_3100), 0xDEAD_BEEF);
    assert_eq!(bus.read_u16(0x2000_3104), 0x1234);
    assert_eq!(cpu.regs[11], 0x2000_3006);
    assert_eq!(cpu.regs[15], 0x2000_3106);
}

#[test]
fn test_bitmanip_zba_zbb_zbc_zbs() {
    let mut bus = FlatBus::new();
    let mut cpu = Cpu::new();
    cpu.regs[10] = 0x0000_0003; // a0
    cpu.regs[11] = 0x1000_0001; // a1

    let insns: [u32; 9] = [
        // sh2add a2(12), a0(10), a1(11): funct7=0x10, rs2=11, rs1=10, f3=4, rd=12, op=0x33
        (0x10 << 25) | (11 << 20) | (10 << 15) | (4 << 12) | (12 << 7) | 0x33,
        // andn a3(13), a1(11), a0(10): funct7=0x20, f3=7
        (0x20 << 25) | (10 << 20) | (11 << 15) | (7 << 12) | (13 << 7) | 0x33,
        // clz a4(14), a0(10): imm=0x600, f3=1, op=0x13
        (0x600 << 20) | (10 << 15) | (1 << 12) | (14 << 7) | 0x13,
        // cpop a5(15), a0(10): imm=0x602, f3=1, op=0x13
        (0x602 << 20) | (10 << 15) | (1 << 12) | (15 << 7) | 0x13,
        // bseti a0(10), a0(10), 8: imm=0x14<<5 | 8, f3=1, op=0x13
        (0x14 << 25) | (8 << 20) | (10 << 15) | (1 << 12) | (10 << 7) | 0x13,
        // bexti a1(11), a0(10), 8: imm=0x24<<5 | 8, f3=5, op=0x13
        (0x24 << 25) | (8 << 20) | (10 << 15) | (5 << 12) | (11 << 7) | 0x13,
        // zext.h a0(10), a1(11): pack with rs2=0 -> funct7=0x04, f3=4, op=0x33
        (0x04 << 25) | (0 << 20) | (11 << 15) | (4 << 12) | (10 << 7) | 0x33,
        // rev8 a0(10), a1(11): imm=0x698, f3=5, op=0x13
        (0x698 << 20) | (11 << 15) | (5 << 12) | (10 << 7) | 0x13,
        // clmul a0(10), a0(10), a1(11): funct7=0x05, f3=1, op=0x33
        (0x05 << 25) | (11 << 20) | (10 << 15) | (1 << 12) | (10 << 7) | 0x33,
    ];
    for (i, &w) in insns.iter().enumerate() {
        bus.load_flash((i * 4) as u32, &w.to_le_bytes());
    }

    cpu.step(&mut bus);
    assert_eq!(cpu.regs[12], (3 << 2) + 0x1000_0001); // sh2add
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[13], 0x1000_0001 & !3); // andn
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[14], 30); // clz(3)
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[15], 2); // cpop(3)
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[10], 0x103); // bseti(3, 8)
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[11], 1); // bexti(0x103, 8)
    cpu.regs[11] = 0xDEAD_BEEF;
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[10], 0xBEEF); // zext.h
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[10], 0xEFBE_ADDE); // rev8
    cpu.regs[10] = 0b101;
    cpu.regs[11] = 0b11;
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[10], 0b1111); // clmul(0b101, 0b11) = 0b101 ^ 0b1010 = 0b1111
}

#[test]
fn test_hpe_mtvec_mode3_and_systick_wfi() {
    let mut bus = FlatBus::new();
    let mut cpu = Cpu::new();

    // Vector table in SRAM at 0x2000_0000
    // Entry 12 (SysTick_IRQn) -> 0x0000_0200
    // Entry 14 (SWI_IRQn) -> 0x0000_0300
    bus.write_u32(0x2000_0000 + 12 * 4, 0x0000_0200);
    bus.write_u32(0x2000_0000 + 14 * 4, 0x0000_0300);

    // Enable HPE + Nesting (`intsyscr = 3`), mtvec = 0x2000_0003, mstatus = 0x88, user mode
    cpu.intsyscr = 3;
    cpu.mtvec = 0x2000_0003;
    cpu.mstatus = 0x88;
    cpu.priv_mode = Privilege::User;
    cpu.regs[10] = 0xCAFE_BABE; // caller-saved a0

    // Main code at 0x0000_0100: wfi; j 0x100
    bus.load_flash(0x100, &0x1050_0073u32.to_le_bytes()); // wfi
    bus.load_flash(0x104, &0x0000_006fu32.to_le_bytes()); // j .

    // SysTick ISR at 0x0000_0200: clobbers a0 (li a0, 42), triggers SWI? or just mret
    // addi a0, x0, 42 -> 0x02a00513; mret -> 0x30200073
    bus.load_flash(0x200, &0x02a0_0513u32.to_le_bytes());
    bus.load_flash(0x204, &0x3020_0073u32.to_le_bytes());

    // Configure SysTick: CMP = 4, CTRL = STRE|STCLK|STIE|STE (0x0F), PFIC enable IRQ 12
    bus.pfic.write_u32(0x100, 1 << 12);
    bus.systick.write_u32(0x10, 4, &mut bus.pfic);
    bus.systick.write_u32(0x00, 0x0F, &mut bus.pfic);

    cpu.pc = 0x100;
    cpu.step(&mut bus); // executes WFI -> halted_wfi = true
    assert!(cpu.halted_wfi);

    // Advance SysTick until IRQ fires
    for _ in 0..6 {
        bus.systick.step(1, &mut bus.pfic);
        cpu.step(&mut bus);
        if cpu.pc == 0x200 {
            break;
        }
    }
    assert_eq!(cpu.pc, 0x200);
    assert!(!cpu.halted_wfi);
    assert!(bus.pfic.is_active(12));

    // Execute ISR: clobbers a0, then mret restores a0 via HPE!
    cpu.step(&mut bus);
    assert_eq!(cpu.regs[10], 42);
    cpu.step(&mut bus); // mret
    assert_eq!(cpu.pc, 0x104); // returned after WFI
    assert_eq!(cpu.regs[10], 0xCAFE_BABE); // restored by HPE!
    assert!(!bus.pfic.is_active(12));
    assert_eq!(cpu.priv_mode, Privilege::User);
}

#[test]
fn test_elf_loader_parses_real_test_elf() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .join("kaynak/test_elf/uygulama.elf");
    let Ok(bytes) = std::fs::read(&path) else {
        eprintln!("Skipping test_elf_loader_parses_real_test_elf: optional ELF fixture not found");
        return;
    };
    let elf = ElfImage::parse(&bytes).expect("valid ELF");
    assert_eq!(elf.entry, 0);
    assert!(elf.addr_by_sym.contains_key("main"));
    assert!(elf.addr_by_sym.contains_key("SetSysClock"));
    // Verify highcode segment uses flash LMA (paddr < 0x1000) even though vaddr is 0x2000_0000
    assert!(elf.segments.iter().any(|s| s.vaddr == 0x2000_0000 && s.paddr < 0x1000));
}

#[test]
fn test_hpe_intsyscr_bit5_context_switch_and_machine_mode_preserve() {
    let mut bus = FlatBus::new();
    let mut cpu = Cpu::new();

    // Mode 3 vector table in SRAM at 0x2000_0000
    bus.write_u32(0x2000_0000 + 12 * 4, 0x0000_0200); // SysTick -> 0x200
    bus.write_u32(0x2000_0000 + 14 * 4, 0x0000_0300); // SWI     -> 0x300

    // Simulate startup state after initial MRET into main(): Machine mode,
    // intsyscr = 3 (HPE + nesting), mtvec = 0x2000_0003, mstatus = 0x0088 (MIE=1, MPIE=1, MPP=0).
    cpu.intsyscr = 3;
    cpu.mtvec = 0x2000_0003;
    cpu.mstatus = 0x0088;
    cpu.priv_mode = Privilege::Machine;
    cpu.regs[2] = 0x2000_0800; // sp (task A)
    cpu.regs[10] = 0x1234_5678; // a0 (task A)

    // Task A at 0x100: j 0x100
    bus.load_flash(0x100, &0x0000_006fu32.to_le_bytes());

    // Normal HPE ISR (IRQ 12) at 0x200:
    //   addi a0, x0, 42
    //   mret
    bus.load_flash(0x200, &0x02a0_0513u32.to_le_bytes());
    bus.load_flash(0x204, &0x3020_0073u32.to_le_bytes());

    // Context-switch SW_Handler (IRQ 14) at 0x300 (same pattern as FreeRTOS portASM.S):
    //   li a1, 0x20
    //   csrs 0x804, a1       (set INTSYSCR bit 5 to suppress HPE unstack on this mret)
    //   li t0, 0x400
    //   csrw mepc, t0
    //   li a0, 99
    //   lui sp, 0x20001
    //   mret
    bus.load_flash(0x300, &0x0200_0593u32.to_le_bytes());
    bus.load_flash(0x304, &0x8045_a073u32.to_le_bytes());
    bus.load_flash(0x308, &0x4000_0293u32.to_le_bytes());
    bus.load_flash(0x30C, &0x3412_9073u32.to_le_bytes());
    bus.load_flash(0x310, &0x0630_0513u32.to_le_bytes());
    bus.load_flash(0x314, &0x2000_1137u32.to_le_bytes());
    bus.load_flash(0x318, &0x3020_0073u32.to_le_bytes());

    // Task B at 0x400: j 0x400
    bus.load_flash(0x400, &0x0000_006fu32.to_le_bytes());

    // Enable IRQ 12 & 14
    bus.pfic.write_u32(0x100, (1 << 12) | (1 << 14));

    // 1. Trigger normal HPE IRQ 12 and verify GPR restore + Machine mode preservation
    cpu.pc = 0x100;
    bus.pfic.set_pending(12);
    cpu.step(&mut bus); // enters IRQ 12 at 0x200
    assert_eq!(cpu.pc, 0x200);
    assert_eq!(cpu.hpe_stack.len(), 1);
    cpu.step(&mut bus); // li a0, 42
    assert_eq!(cpu.regs[10], 42);
    cpu.step(&mut bus); // mret -> restores a0 & stays in Machine mode
    assert_eq!(cpu.pc, 0x100);
    assert_eq!(cpu.regs[10], 0x1234_5678);
    assert_eq!(cpu.priv_mode, Privilege::Machine);
    assert!(cpu.mie());
    assert!(cpu.hpe_stack.is_empty());
    assert!(!bus.pfic.is_active(12));

    // 2. Trigger SWI_IRQn via SysTick->CTLR bit 31 and run context switch with INTSYSCR bit 5
    bus.systick.write_u32(0x00, 1 << 31, &mut bus.pfic);
    assert!(bus.pfic.is_pending(14));
    cpu.step(&mut bus); // enters SW_Handler at 0x300
    assert_eq!(cpu.pc, 0x300);
    assert_eq!(cpu.hpe_stack.len(), 1);

    for _ in 0..7 {
        cpu.step(&mut bus);
    }
    assert_eq!(cpu.pc, 0x400); // switched to Task B, NOT Task A's 0x100
    assert_eq!(cpu.regs[10], 99); // Task B's a0 preserved, NOT overwritten by HPE pop
    assert_eq!(cpu.regs[2], 0x2000_1000); // Task B's sp preserved
    assert_eq!(cpu.intsyscr & 0x20, 0); // bit 5 auto-cleared on mret
    assert!(cpu.hpe_stack.is_empty());
    assert!(!bus.pfic.is_active(14));
    assert_eq!(cpu.priv_mode, Privilege::Machine);
    assert!(cpu.mie());

    // 3. Normal HPE interrupt afterwards must still push & restore Task B's GPRs normally
    bus.pfic.set_pending(12);
    cpu.step(&mut bus); // enters IRQ 12
    assert_eq!(cpu.pc, 0x200);
    cpu.step(&mut bus); // li a0, 42
    assert_eq!(cpu.regs[10], 42);
    cpu.step(&mut bus); // mret -> restores Task B's a0 = 99
    assert_eq!(cpu.pc, 0x400);
    assert_eq!(cpu.regs[10], 99);
    assert!(cpu.hpe_stack.is_empty());
}

#[test]
fn test_hpe_nested_interrupt_preserves_outer_mepc_and_mstatus() {
    let mut bus = FlatBus::new();
    let mut cpu = Cpu::new();

    bus.write_u32(0x2000_0000 + 12 * 4, 0x0000_0200); // IRQ 12 (low priority)
    bus.write_u32(0x2000_0000 + 17 * 4, 0x0000_0300); // IRQ 17 (high priority)

    cpu.intsyscr = 3; // HPE + nesting
    cpu.mtvec = 0x2000_0003;
    cpu.mstatus = 0x0088;
    cpu.priv_mode = Privilege::Machine;
    cpu.regs[10] = 0x1111_2222;

    // Main loop at 0x100: j 0x100
    bus.load_flash(0x100, &0x0000_006fu32.to_le_bytes());

    // Low-priority IRQ 12 at 0x200:
    //   0x200: li a0, 55
    //   0x204: csrsi mstatus, 8      (re-enable interrupts for nesting)
    //   0x208: lui a5, 0xe000e
    //   0x20C: lui a4, 0x20          (1 << 17)
    //   0x210: sw a4, 0x200(a5)      (pend IRQ 17 in PFIC->IPSR[0])
    //   0x214: nop
    //   0x218: csrci mstatus, 8
    //   0x21C: mret
    bus.load_flash(0x200, &0x0370_0513u32.to_le_bytes());
    bus.load_flash(0x204, &0x3004_6073u32.to_le_bytes());
    bus.load_flash(0x208, &0xe000_e7b7u32.to_le_bytes());
    bus.load_flash(0x20C, &0x0002_0737u32.to_le_bytes());
    bus.load_flash(0x210, &0x20e7_a023u32.to_le_bytes());
    bus.load_flash(0x214, &0x0000_0013u32.to_le_bytes());
    bus.load_flash(0x218, &0x3004_7073u32.to_le_bytes());
    bus.load_flash(0x21C, &0x3020_0073u32.to_le_bytes());

    // High-priority IRQ 17 at 0x300:
    //   0x300: li a0, 77
    //   0x304: mret
    bus.load_flash(0x300, &0x04d0_0513u32.to_le_bytes());
    bus.load_flash(0x304, &0x3020_0073u32.to_le_bytes());

    bus.pfic.write_u8(0x400 + 12, 0x80); // IRQ 12 low priority
    bus.pfic.write_u8(0x400 + 17, 0x00); // IRQ 17 high priority
    bus.pfic.write_u32(0x100, (1 << 12) | (1 << 17));

    cpu.pc = 0x100;
    bus.pfic.set_pending(12);
    cpu.step(&mut bus); // enter IRQ 12
    assert_eq!(cpu.pc, 0x200);
    assert_eq!(cpu.mepc, 0x100);

    for _ in 0..5 {
        cpu.step(&mut bus); // li, csrsi, lui, lui, sw -> pends IRQ 17
    }
    cpu.step(&mut bus); // service_interrupts enters IRQ 17 before nop at 0x214
    assert_eq!(cpu.pc, 0x300);
    assert_eq!(cpu.hpe_stack.len(), 2);
    assert_eq!(cpu.mepc, 0x214);

    cpu.step(&mut bus); // li a0, 77
    assert_eq!(cpu.regs[10], 77);
    cpu.step(&mut bus); // inner mret -> returns to 0x214 in IRQ 12 with outer CSRs & a0=55
    assert_eq!(cpu.pc, 0x214);
    assert_eq!(cpu.regs[10], 55);
    assert_eq!(cpu.mepc, 0x100);
    assert_eq!(cpu.mcause, 0x8000_000C);
    assert_eq!(cpu.hpe_stack.len(), 1);
    assert!(!bus.pfic.is_active(17));
    assert!(bus.pfic.is_active(12));

    cpu.step(&mut bus); // nop
    cpu.step(&mut bus); // csrci mstatus, 8
    cpu.step(&mut bus); // outer mret -> returns to main 0x100 with a0=0x1111_2222
    assert_eq!(cpu.pc, 0x100);
    assert_eq!(cpu.regs[10], 0x1111_2222);
    assert_eq!(cpu.priv_mode, Privilege::Machine);
    assert!(cpu.mie());
    assert!(cpu.hpe_stack.is_empty());
    assert!(!bus.pfic.is_active(12));
}
