#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Insn {
    // RV32I
    Lui { rd: u8, imm: i32 },
    Auipc { rd: u8, imm: i32 },
    Jal { rd: u8, imm: i32 },
    Jalr { rd: u8, rs1: u8, imm: i32 },
    Beq { rs1: u8, rs2: u8, imm: i32 },
    Bne { rs1: u8, rs2: u8, imm: i32 },
    Blt { rs1: u8, rs2: u8, imm: i32 },
    Bge { rs1: u8, rs2: u8, imm: i32 },
    Bltu { rs1: u8, rs2: u8, imm: i32 },
    Bgeu { rs1: u8, rs2: u8, imm: i32 },
    Lb { rd: u8, rs1: u8, imm: i32 },
    Lh { rd: u8, rs1: u8, imm: i32 },
    Lw { rd: u8, rs1: u8, imm: i32 },
    Lbu { rd: u8, rs1: u8, imm: i32 },
    Lhu { rd: u8, rs1: u8, imm: i32 },
    Sb { rs1: u8, rs2: u8, imm: i32 },
    Sh { rs1: u8, rs2: u8, imm: i32 },
    Sw { rs1: u8, rs2: u8, imm: i32 },
    Addi { rd: u8, rs1: u8, imm: i32 },
    Slti { rd: u8, rs1: u8, imm: i32 },
    Sltiu { rd: u8, rs1: u8, imm: i32 },
    Xori { rd: u8, rs1: u8, imm: i32 },
    Ori { rd: u8, rs1: u8, imm: i32 },
    Andi { rd: u8, rs1: u8, imm: i32 },
    Slli { rd: u8, rs1: u8, shamt: u8 },
    Srli { rd: u8, rs1: u8, shamt: u8 },
    Srai { rd: u8, rs1: u8, shamt: u8 },
    Add { rd: u8, rs1: u8, rs2: u8 },
    Sub { rd: u8, rs1: u8, rs2: u8 },
    Sll { rd: u8, rs1: u8, rs2: u8 },
    Slt { rd: u8, rs1: u8, rs2: u8 },
    Sltu { rd: u8, rs1: u8, rs2: u8 },
    Xor { rd: u8, rs1: u8, rs2: u8 },
    Srl { rd: u8, rs1: u8, rs2: u8 },
    Sra { rd: u8, rs1: u8, rs2: u8 },
    Or { rd: u8, rs1: u8, rs2: u8 },
    And { rd: u8, rs1: u8, rs2: u8 },
    Fence,
    FenceI,
    Ecall,
    Ebreak,
    Mret,
    Wfi,
    Csrrw { rd: u8, rs1: u8, csr: u16 },
    Csrrs { rd: u8, rs1: u8, csr: u16 },
    Csrrc { rd: u8, rs1: u8, csr: u16 },
    Csrrwi { rd: u8, zimm: u8, csr: u16 },
    Csrrsi { rd: u8, zimm: u8, csr: u16 },
    Csrrci { rd: u8, zimm: u8, csr: u16 },

    // M extension
    Mul { rd: u8, rs1: u8, rs2: u8 },
    Mulh { rd: u8, rs1: u8, rs2: u8 },
    Mulhsu { rd: u8, rs1: u8, rs2: u8 },
    Mulhu { rd: u8, rs1: u8, rs2: u8 },
    Div { rd: u8, rs1: u8, rs2: u8 },
    Divu { rd: u8, rs1: u8, rs2: u8 },
    Rem { rd: u8, rs1: u8, rs2: u8 },
    Remu { rd: u8, rs1: u8, rs2: u8 },

    // Zba
    Sh1add { rd: u8, rs1: u8, rs2: u8 },
    Sh2add { rd: u8, rs1: u8, rs2: u8 },
    Sh3add { rd: u8, rs1: u8, rs2: u8 },

    // Zbb
    Andn { rd: u8, rs1: u8, rs2: u8 },
    Orn { rd: u8, rs1: u8, rs2: u8 },
    Xnor { rd: u8, rs1: u8, rs2: u8 },
    Clz { rd: u8, rs1: u8 },
    Ctz { rd: u8, rs1: u8 },
    Cpop { rd: u8, rs1: u8 },
    Max { rd: u8, rs1: u8, rs2: u8 },
    Maxu { rd: u8, rs1: u8, rs2: u8 },
    Min { rd: u8, rs1: u8, rs2: u8 },
    Minu { rd: u8, rs1: u8, rs2: u8 },
    SextB { rd: u8, rs1: u8 },
    SextH { rd: u8, rs1: u8 },
    ZextH { rd: u8, rs1: u8 },
    Rol { rd: u8, rs1: u8, rs2: u8 },
    Ror { rd: u8, rs1: u8, rs2: u8 },
    Rori { rd: u8, rs1: u8, shamt: u8 },
    OrcB { rd: u8, rs1: u8 },
    Rev8 { rd: u8, rs1: u8 },

    // Zbc
    Clmul { rd: u8, rs1: u8, rs2: u8 },
    Clmulh { rd: u8, rs1: u8, rs2: u8 },
    Clmulr { rd: u8, rs1: u8, rs2: u8 },

    // Zbs
    Bclr { rd: u8, rs1: u8, rs2: u8 },
    Bclri { rd: u8, rs1: u8, shamt: u8 },
    Bext { rd: u8, rs1: u8, rs2: u8 },
    Bexti { rd: u8, rs1: u8, shamt: u8 },
    Binv { rd: u8, rs1: u8, rs2: u8 },
    Binvi { rd: u8, rs1: u8, shamt: u8 },
    Bset { rd: u8, rs1: u8, rs2: u8 },
    Bseti { rd: u8, rs1: u8, shamt: u8 },

    // WCH QingKe custom
    Mcpy { rd: u8, rs1: u8, rs2: u8 },

    Illegal(u32),
}

#[inline]
fn sign_extend(val: u32, bits: u8) -> i32 {
    let shift = 32 - bits;
    ((val << shift) as i32) >> shift
}

#[inline]
fn rvc_reg(idx3: u16) -> u8 {
    8 + (idx3 as u8 & 7)
}

/// Decode a 16-bit RVC / WCH XW instruction.
pub fn decode_16(inst: u16) -> Insn {
    let op = inst & 0b11;
    let funct3 = (inst >> 13) & 0b111;

    match op {
        0b00 => match funct3 {
            0b000 => {
                // C.ADDI4SPN: nzuimm[5:4|9:6|2|3]
                let nzuimm = (((inst >> 6) & 1) << 2)
                    | (((inst >> 5) & 1) << 3)
                    | (((inst >> 11) & 0x3) << 4)
                    | (((inst >> 7) & 0xF) << 6);
                if nzuimm == 0 {
                    Insn::Illegal(inst as u32)
                } else {
                    Insn::Addi {
                        rd: rvc_reg((inst >> 2) & 7),
                        rs1: 2,
                        imm: nzuimm as i32,
                    }
                }
            }
            0b001 => {
                // WCH XW: c.lbu rd', uimm(rs1')
                let uimm = ((inst >> 12) & 1)
                    | (((inst >> 5) & 1) << 1)
                    | (((inst >> 6) & 1) << 2)
                    | (((inst >> 10) & 1) << 3)
                    | (((inst >> 11) & 1) << 4);
                Insn::Lbu {
                    rd: rvc_reg((inst >> 2) & 7),
                    rs1: rvc_reg((inst >> 7) & 7),
                    imm: uimm as i32,
                }
            }
            0b010 => {
                // C.LW
                let uimm = (((inst >> 6) & 1) << 2)
                    | (((inst >> 10) & 0x7) << 3)
                    | (((inst >> 5) & 1) << 6);
                Insn::Lw {
                    rd: rvc_reg((inst >> 2) & 7),
                    rs1: rvc_reg((inst >> 7) & 7),
                    imm: uimm as i32,
                }
            }
            0b100 => {
                // WCH XW SP-relative byte/half loads & stores (bits[6:5])
                let sub = (inst >> 5) & 0b11;
                let reg = rvc_reg((inst >> 2) & 7);
                match sub {
                    0b00 => {
                        // c.lbusp rd', uimm(sp)
                        let uimm = (inst >> 7) & 0xF;
                        Insn::Lbu {
                            rd: reg,
                            rs1: 2,
                            imm: uimm as i32,
                        }
                    }
                    0b01 => {
                        // c.lhusp rd', uimm(sp)
                        let uimm = (((inst >> 8) & 1) << 1)
                            | (((inst >> 9) & 1) << 2)
                            | (((inst >> 10) & 1) << 3)
                            | (((inst >> 7) & 1) << 4);
                        Insn::Lhu {
                            rd: reg,
                            rs1: 2,
                            imm: uimm as i32,
                        }
                    }
                    0b10 => {
                        // c.sbsp rs2', uimm(sp)
                        let uimm = (inst >> 7) & 0xF;
                        Insn::Sb {
                            rs1: 2,
                            rs2: reg,
                            imm: uimm as i32,
                        }
                    }
                    0b11 => {
                        // c.shsp rs2', uimm(sp)
                        let uimm = (((inst >> 8) & 1) << 1)
                            | (((inst >> 9) & 1) << 2)
                            | (((inst >> 10) & 1) << 3)
                            | (((inst >> 7) & 1) << 4);
                        Insn::Sh {
                            rs1: 2,
                            rs2: reg,
                            imm: uimm as i32,
                        }
                    }
                    _ => unreachable!(),
                }
            }
            0b101 => {
                // WCH XW: c.sb rs2', uimm(rs1')
                let uimm = ((inst >> 12) & 1)
                    | (((inst >> 5) & 1) << 1)
                    | (((inst >> 6) & 1) << 2)
                    | (((inst >> 10) & 1) << 3)
                    | (((inst >> 11) & 1) << 4);
                Insn::Sb {
                    rs1: rvc_reg((inst >> 7) & 7),
                    rs2: rvc_reg((inst >> 2) & 7),
                    imm: uimm as i32,
                }
            }
            0b110 => {
                // C.SW
                let uimm = (((inst >> 6) & 1) << 2)
                    | (((inst >> 10) & 0x7) << 3)
                    | (((inst >> 5) & 1) << 6);
                Insn::Sw {
                    rs1: rvc_reg((inst >> 7) & 7),
                    rs2: rvc_reg((inst >> 2) & 7),
                    imm: uimm as i32,
                }
            }
            _ => Insn::Illegal(inst as u32),
        },
        0b01 => match funct3 {
            0b000 => {
                // C.NOP / C.ADDI
                let rd = ((inst >> 7) & 0x1F) as u8;
                let imm = sign_extend((((inst >> 12) & 1) << 5) as u32 | ((inst >> 2) & 0x1F) as u32, 6);
                Insn::Addi { rd, rs1: rd, imm }
            }
            0b001 => {
                // C.JAL (RV32): rd=x1
                let imm = c_j_imm(inst);
                Insn::Jal { rd: 1, imm }
            }
            0b010 => {
                // C.LI
                let rd = ((inst >> 7) & 0x1F) as u8;
                let imm = sign_extend((((inst >> 12) & 1) << 5) as u32 | ((inst >> 2) & 0x1F) as u32, 6);
                Insn::Addi { rd, rs1: 0, imm }
            }
            0b011 => {
                let rd = ((inst >> 7) & 0x1F) as u8;
                if rd == 2 {
                    // C.ADDI16SP: nzimm[9|4|6|8:7|5]
                    let raw = (((inst >> 6) & 1) << 4)
                        | (((inst >> 2) & 1) << 5)
                        | (((inst >> 5) & 1) << 6)
                        | (((inst >> 3) & 0x3) << 7)
                        | (((inst >> 12) & 1) << 9);
                    let imm = sign_extend(raw as u32, 10);
                    if imm == 0 {
                        Insn::Illegal(inst as u32)
                    } else {
                        Insn::Addi { rd: 2, rs1: 2, imm }
                    }
                } else {
                    // C.LUI
                    let w = inst as u32;
                    let raw = (((w >> 2) & 0x1F) << 12) | (((w >> 12) & 1) << 17);
                    let imm = sign_extend(raw, 18);
                    if imm == 0 {
                        Insn::Illegal(inst as u32)
                    } else {
                        Insn::Lui { rd, imm }
                    }
                }
            }
            0b100 => {
                let funct2 = (inst >> 10) & 0b11;
                let rd = rvc_reg((inst >> 7) & 7);
                let shamt = ((((inst >> 12) & 1) << 5) | ((inst >> 2) & 0x1F)) as u8;
                match funct2 {
                    0b00 => {
                        if shamt >= 32 {
                            Insn::Illegal(inst as u32)
                        } else {
                            Insn::Srli { rd, rs1: rd, shamt }
                        }
                    }
                    0b01 => {
                        if shamt >= 32 {
                            Insn::Illegal(inst as u32)
                        } else {
                            Insn::Srai { rd, rs1: rd, shamt }
                        }
                    }
                    0b10 => {
                        let imm = sign_extend(
                            (((inst >> 12) & 1) << 5) as u32 | ((inst >> 2) & 0x1F) as u32,
                            6,
                        );
                        Insn::Andi { rd, rs1: rd, imm }
                    }
                    0b11 => {
                        let bit12 = (inst >> 12) & 1;
                        let funct2_b = (inst >> 5) & 0b11;
                        let rs2 = rvc_reg((inst >> 2) & 7);
                        if bit12 == 0 {
                            match funct2_b {
                                0b00 => Insn::Sub { rd, rs1: rd, rs2 },
                                0b01 => Insn::Xor { rd, rs1: rd, rs2 },
                                0b10 => Insn::Or { rd, rs1: rd, rs2 },
                                0b11 => Insn::And { rd, rs1: rd, rs2 },
                                _ => unreachable!(),
                            }
                        } else {
                            Insn::Illegal(inst as u32)
                        }
                    }
                    _ => unreachable!(),
                }
            }
            0b101 => {
                // C.J
                Insn::Jal {
                    rd: 0,
                    imm: c_j_imm(inst),
                }
            }
            0b110 => {
                // C.BEQZ
                Insn::Beq {
                    rs1: rvc_reg((inst >> 7) & 7),
                    rs2: 0,
                    imm: c_b_imm(inst),
                }
            }
            0b111 => {
                // C.BNEZ
                Insn::Bne {
                    rs1: rvc_reg((inst >> 7) & 7),
                    rs2: 0,
                    imm: c_b_imm(inst),
                }
            }
            _ => unreachable!(),
        },
        0b10 => match funct3 {
            0b000 => {
                // C.SLLI
                let rd = ((inst >> 7) & 0x1F) as u8;
                let shamt = ((((inst >> 12) & 1) << 5) | ((inst >> 2) & 0x1F)) as u8;
                if shamt >= 32 {
                    Insn::Illegal(inst as u32)
                } else {
                    Insn::Slli { rd, rs1: rd, shamt }
                }
            }
            0b001 => {
                // WCH XW: c.lhu rd', uimm(rs1')
                let uimm = (((inst >> 5) & 1) << 1)
                    | (((inst >> 6) & 1) << 2)
                    | (((inst >> 10) & 1) << 3)
                    | (((inst >> 11) & 1) << 4)
                    | (((inst >> 12) & 1) << 5);
                Insn::Lhu {
                    rd: rvc_reg((inst >> 2) & 7),
                    rs1: rvc_reg((inst >> 7) & 7),
                    imm: uimm as i32,
                }
            }
            0b010 => {
                // C.LWSP
                let rd = ((inst >> 7) & 0x1F) as u8;
                if rd == 0 {
                    Insn::Illegal(inst as u32)
                } else {
                    let uimm = (((inst >> 4) & 0x7) << 2)
                        | (((inst >> 12) & 1) << 5)
                        | (((inst >> 2) & 0x3) << 6);
                    Insn::Lw {
                        rd,
                        rs1: 2,
                        imm: uimm as i32,
                    }
                }
            }
            0b100 => {
                let bit12 = (inst >> 12) & 1;
                let rd = ((inst >> 7) & 0x1F) as u8;
                let rs2 = ((inst >> 2) & 0x1F) as u8;
                if bit12 == 0 {
                    if rs2 == 0 {
                        if rd == 0 {
                            Insn::Illegal(inst as u32)
                        } else {
                            // C.JR
                            Insn::Jalr {
                                rd: 0,
                                rs1: rd,
                                imm: 0,
                            }
                        }
                    } else {
                        // C.MV
                        Insn::Add {
                            rd,
                            rs1: 0,
                            rs2,
                        }
                    }
                } else if rd == 0 && rs2 == 0 {
                    Insn::Ebreak
                } else if rs2 == 0 {
                    // C.JALR
                    Insn::Jalr {
                        rd: 1,
                        rs1: rd,
                        imm: 0,
                    }
                } else {
                    // C.ADD
                    Insn::Add { rd, rs1: rd, rs2 }
                }
            }
            0b101 => {
                // WCH XW: c.sh rs2', uimm(rs1')
                let uimm = (((inst >> 5) & 1) << 1)
                    | (((inst >> 6) & 1) << 2)
                    | (((inst >> 10) & 1) << 3)
                    | (((inst >> 11) & 1) << 4)
                    | (((inst >> 12) & 1) << 5);
                Insn::Sh {
                    rs1: rvc_reg((inst >> 7) & 7),
                    rs2: rvc_reg((inst >> 2) & 7),
                    imm: uimm as i32,
                }
            }
            0b110 => {
                // C.SWSP
                let rs2 = ((inst >> 2) & 0x1F) as u8;
                let uimm = (((inst >> 9) & 0xF) << 2) | (((inst >> 7) & 0x3) << 6);
                Insn::Sw {
                    rs1: 2,
                    rs2,
                    imm: uimm as i32,
                }
            }
            _ => Insn::Illegal(inst as u32),
        },
        _ => Insn::Illegal(inst as u32),
    }
}

#[inline]
fn c_j_imm(inst: u16) -> i32 {
    let raw = (((inst >> 3) & 0x7) << 1)
        | (((inst >> 11) & 1) << 4)
        | (((inst >> 2) & 1) << 5)
        | (((inst >> 7) & 1) << 6)
        | (((inst >> 6) & 1) << 7)
        | (((inst >> 9) & 0x3) << 8)
        | (((inst >> 8) & 1) << 10)
        | (((inst >> 12) & 1) << 11);
    sign_extend(raw as u32, 12)
}

#[inline]
fn c_b_imm(inst: u16) -> i32 {
    let raw = (((inst >> 3) & 0x3) << 1)
        | (((inst >> 10) & 0x3) << 3)
        | (((inst >> 2) & 1) << 5)
        | (((inst >> 5) & 0x3) << 6)
        | (((inst >> 12) & 1) << 8);
    sign_extend(raw as u32, 9)
}

/// Decode a 32-bit RV32IM + Zba/Zbb/Zbc/Zbs + WCH mcpy instruction.
pub fn decode_32(inst: u32) -> Insn {
    let opcode = inst & 0x7F;
    let rd = ((inst >> 7) & 0x1F) as u8;
    let funct3 = (inst >> 12) & 0x7;
    let rs1 = ((inst >> 15) & 0x1F) as u8;
    let rs2 = ((inst >> 20) & 0x1F) as u8;
    let funct7 = (inst >> 25) & 0x7F;

    match opcode {
        0x37 => Insn::Lui {
            rd,
            imm: (inst & 0xFFFF_F000) as i32,
        },
        0x17 => Insn::Auipc {
            rd,
            imm: (inst & 0xFFFF_F000) as i32,
        },
        0x6F => {
            let raw = (((inst >> 21) & 0x3FF) << 1)
                | (((inst >> 20) & 1) << 11)
                | (((inst >> 12) & 0xFF) << 12)
                | (((inst >> 31) & 1) << 20);
            Insn::Jal {
                rd,
                imm: sign_extend(raw, 21),
            }
        }
        0x67 if funct3 == 0 => Insn::Jalr {
            rd,
            rs1,
            imm: sign_extend(inst >> 20, 12),
        },
        0x63 => {
            let raw = (((inst >> 8) & 0xF) << 1)
                | (((inst >> 25) & 0x3F) << 5)
                | (((inst >> 7) & 1) << 11)
                | (((inst >> 31) & 1) << 12);
            let imm = sign_extend(raw, 13);
            match funct3 {
                0b000 => Insn::Beq { rs1, rs2, imm },
                0b001 => Insn::Bne { rs1, rs2, imm },
                0b100 => Insn::Blt { rs1, rs2, imm },
                0b101 => Insn::Bge { rs1, rs2, imm },
                0b110 => Insn::Bltu { rs1, rs2, imm },
                0b111 => Insn::Bgeu { rs1, rs2, imm },
                _ => Insn::Illegal(inst),
            }
        }
        0x03 => {
            let imm = sign_extend(inst >> 20, 12);
            match funct3 {
                0b000 => Insn::Lb { rd, rs1, imm },
                0b001 => Insn::Lh { rd, rs1, imm },
                0b010 => Insn::Lw { rd, rs1, imm },
                0b100 => Insn::Lbu { rd, rs1, imm },
                0b101 => Insn::Lhu { rd, rs1, imm },
                _ => Insn::Illegal(inst),
            }
        }
        0x23 => {
            let raw = ((inst >> 7) & 0x1F) | (((inst >> 25) & 0x7F) << 5);
            let imm = sign_extend(raw, 12);
            match funct3 {
                0b000 => Insn::Sb { rs1, rs2, imm },
                0b001 => Insn::Sh { rs1, rs2, imm },
                0b010 => Insn::Sw { rs1, rs2, imm },
                _ => Insn::Illegal(inst),
            }
        }
        0x13 => {
            let imm12 = inst >> 20;
            let imm = sign_extend(imm12, 12);
            let shamt = (imm12 & 0x1F) as u8;
            match funct3 {
                0b000 => Insn::Addi { rd, rs1, imm },
                0b010 => Insn::Slti { rd, rs1, imm },
                0b011 => Insn::Sltiu { rd, rs1, imm },
                0b100 => Insn::Xori { rd, rs1, imm },
                0b110 => Insn::Ori { rd, rs1, imm },
                0b111 => Insn::Andi { rd, rs1, imm },
                0b001 => match imm12 {
                    0x600 => Insn::Clz { rd, rs1 },
                    0x601 => Insn::Ctz { rd, rs1 },
                    0x602 => Insn::Cpop { rd, rs1 },
                    0x604 => Insn::SextB { rd, rs1 },
                    0x605 => Insn::SextH { rd, rs1 },
                    _ => match funct7 {
                        0x00 => Insn::Slli { rd, rs1, shamt },
                        0x14 => Insn::Bseti { rd, rs1, shamt },
                        0x24 => Insn::Bclri { rd, rs1, shamt },
                        0x34 => Insn::Binvi { rd, rs1, shamt },
                        _ => Insn::Illegal(inst),
                    },
                },
                0b101 => match imm12 {
                    0x287 => Insn::OrcB { rd, rs1 },
                    0x698 => Insn::Rev8 { rd, rs1 },
                    _ => match funct7 {
                        0x00 => Insn::Srli { rd, rs1, shamt },
                        0x20 => Insn::Srai { rd, rs1, shamt },
                        0x30 => Insn::Rori { rd, rs1, shamt },
                        0x24 => Insn::Bexti { rd, rs1, shamt },
                        _ => Insn::Illegal(inst),
                    },
                },
                _ => unreachable!(),
            }
        }
        0x33 => match (funct7, funct3) {
            // RV32I
            (0x00, 0b000) => Insn::Add { rd, rs1, rs2 },
            (0x20, 0b000) => Insn::Sub { rd, rs1, rs2 },
            (0x00, 0b001) => Insn::Sll { rd, rs1, rs2 },
            (0x00, 0b010) => Insn::Slt { rd, rs1, rs2 },
            (0x00, 0b011) => Insn::Sltu { rd, rs1, rs2 },
            (0x00, 0b100) => Insn::Xor { rd, rs1, rs2 },
            (0x00, 0b101) => Insn::Srl { rd, rs1, rs2 },
            (0x20, 0b101) => Insn::Sra { rd, rs1, rs2 },
            (0x00, 0b110) => Insn::Or { rd, rs1, rs2 },
            (0x00, 0b111) => Insn::And { rd, rs1, rs2 },
            // M
            (0x01, 0b000) => Insn::Mul { rd, rs1, rs2 },
            (0x01, 0b001) => Insn::Mulh { rd, rs1, rs2 },
            (0x01, 0b010) => Insn::Mulhsu { rd, rs1, rs2 },
            (0x01, 0b011) => Insn::Mulhu { rd, rs1, rs2 },
            (0x01, 0b100) => Insn::Div { rd, rs1, rs2 },
            (0x01, 0b101) => Insn::Divu { rd, rs1, rs2 },
            (0x01, 0b110) => Insn::Rem { rd, rs1, rs2 },
            (0x01, 0b111) => Insn::Remu { rd, rs1, rs2 },
            // Zba
            (0x10, 0b010) => Insn::Sh1add { rd, rs1, rs2 },
            (0x10, 0b100) => Insn::Sh2add { rd, rs1, rs2 },
            (0x10, 0b110) => Insn::Sh3add { rd, rs1, rs2 },
            // Zbb
            (0x20, 0b111) => Insn::Andn { rd, rs1, rs2 },
            (0x20, 0b110) => Insn::Orn { rd, rs1, rs2 },
            (0x20, 0b100) => Insn::Xnor { rd, rs1, rs2 },
            (0x05, 0b100) => Insn::Min { rd, rs1, rs2 },
            (0x05, 0b101) => Insn::Minu { rd, rs1, rs2 },
            (0x05, 0b110) => Insn::Max { rd, rs1, rs2 },
            (0x05, 0b111) => Insn::Maxu { rd, rs1, rs2 },
            (0x04, 0b100) if rs2 == 0 => Insn::ZextH { rd, rs1 },
            (0x30, 0b001) => Insn::Rol { rd, rs1, rs2 },
            (0x30, 0b101) => Insn::Ror { rd, rs1, rs2 },
            // Zbc
            (0x05, 0b001) => Insn::Clmul { rd, rs1, rs2 },
            (0x05, 0b010) => Insn::Clmulr { rd, rs1, rs2 },
            (0x05, 0b011) => Insn::Clmulh { rd, rs1, rs2 },
            // Zbs
            (0x24, 0b001) => Insn::Bclr { rd, rs1, rs2 },
            (0x24, 0b101) => Insn::Bext { rd, rs1, rs2 },
            (0x34, 0b001) => Insn::Binv { rd, rs1, rs2 },
            (0x14, 0b001) => Insn::Bset { rd, rs1, rs2 },
            _ => Insn::Illegal(inst),
        },
        0x0F => match funct3 {
            0b000 => Insn::Fence,
            0b001 => Insn::FenceI,
            0b111 if ((inst >> 25) & 0x3) == 0 => {
                // WCH mcpy: [31:27]=rs2, [26:25]=00, [24:20]=rs1, [19:15]=rd, [14:12]=111, [11:7]=0, [6:0]=0x0F
                let m_rd = ((inst >> 15) & 0x1F) as u8;
                let m_rs1 = ((inst >> 20) & 0x1F) as u8;
                let m_rs2 = ((inst >> 27) & 0x1F) as u8;
                Insn::Mcpy {
                    rd: m_rd,
                    rs1: m_rs1,
                    rs2: m_rs2,
                }
            }
            _ => Insn::Illegal(inst),
        },
        0x73 => {
            let csr = (inst >> 20) as u16;
            match funct3 {
                0b000 => match inst >> 20 {
                    0x000 => Insn::Ecall,
                    0x001 => Insn::Ebreak,
                    0x105 => Insn::Wfi,
                    0x302 => Insn::Mret,
                    _ => Insn::Illegal(inst),
                },
                0b001 => Insn::Csrrw { rd, rs1, csr },
                0b010 => Insn::Csrrs { rd, rs1, csr },
                0b011 => Insn::Csrrc { rd, rs1, csr },
                0b101 => Insn::Csrrwi { rd, zimm: rs1, csr },
                0b110 => Insn::Csrrsi { rd, zimm: rs1, csr },
                0b111 => Insn::Csrrci { rd, zimm: rs1, csr },
                _ => Insn::Illegal(inst),
            }
        }
        _ => Insn::Illegal(inst),
    }
}
