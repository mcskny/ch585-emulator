use std::collections::HashMap;

#[derive(Debug, Clone)]
pub struct ElfSegment {
    pub paddr: u32,
    pub vaddr: u32,
    pub memsz: u32,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ElfSymbol {
    pub addr: u32,
    pub size: u32,
    pub name: String,
    pub is_func: bool,
}

#[derive(Debug, Clone)]
pub struct ElfImage {
    pub entry: u32,
    pub segments: Vec<ElfSegment>,
    pub sym_by_addr: HashMap<u32, Vec<String>>,
    pub addr_by_sym: HashMap<String, u32>,
    pub symbols: Vec<ElfSymbol>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ElfError {
    TooShort,
    BadMagic,
    NotElf32,
    NotLittleEndian,
    NotRiscV,
    TruncatedTable,
}

impl std::fmt::Display for ElfError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

impl std::error::Error for ElfError {}

fn r16(b: &[u8], off: usize) -> Result<u16, ElfError> {
    let slice = b.get(off..off + 2).ok_or(ElfError::TruncatedTable)?;
    Ok(u16::from_le_bytes([slice[0], slice[1]]))
}

fn r32(b: &[u8], off: usize) -> Result<u32, ElfError> {
    let slice = b.get(off..off + 4).ok_or(ElfError::TruncatedTable)?;
    Ok(u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]))
}

impl ElfImage {
    pub fn parse(bytes: &[u8]) -> Result<Self, ElfError> {
        if bytes.len() < 52 {
            return Err(ElfError::TooShort);
        }
        if &bytes[0..4] != b"\x7fELF" {
            return Err(ElfError::BadMagic);
        }
        if bytes[4] != 1 {
            return Err(ElfError::NotElf32);
        }
        if bytes[5] != 1 {
            return Err(ElfError::NotLittleEndian);
        }
        let e_machine = r16(bytes, 18)?;
        if e_machine != 0xF3 {
            return Err(ElfError::NotRiscV);
        }

        let entry = r32(bytes, 24)?;
        let e_phoff = r32(bytes, 28)? as usize;
        let e_shoff = r32(bytes, 32)? as usize;
        let e_phentsize = r16(bytes, 42)? as usize;
        let e_phnum = r16(bytes, 44)? as usize;
        let e_shentsize = r16(bytes, 46)? as usize;
        let e_shnum = r16(bytes, 48)? as usize;

        let mut segments = Vec::new();
        for i in 0..e_phnum {
            let off = e_phoff + i * e_phentsize;
            let p_type = r32(bytes, off)?;
            if p_type != 1 {
                continue; // PT_LOAD
            }
            let p_offset = r32(bytes, off + 4)? as usize;
            let p_vaddr = r32(bytes, off + 8)?;
            let p_paddr = r32(bytes, off + 12)?;
            let p_filesz = r32(bytes, off + 16)? as usize;
            let p_memsz = r32(bytes, off + 20)?;
            let data = if p_filesz == 0 {
                Vec::new()
            } else {
                bytes
                    .get(p_offset..p_offset + p_filesz)
                    .ok_or(ElfError::TruncatedTable)?
                    .to_vec()
            };
            segments.push(ElfSegment {
                paddr: p_paddr,
                vaddr: p_vaddr,
                memsz: p_memsz,
                data,
            });
        }

        let mut sym_by_addr: HashMap<u32, Vec<String>> = HashMap::new();
        let mut addr_by_sym: HashMap<String, u32> = HashMap::new();
        let mut symbols: Vec<ElfSymbol> = Vec::new();

        if e_shoff != 0 && e_shnum > 0 && e_shentsize >= 40 {
            for i in 0..e_shnum {
                let sh_off = e_shoff + i * e_shentsize;
                let sh_type = r32(bytes, sh_off + 4)?;
                if sh_type != 2 {
                    continue; // SHT_SYMTAB
                }
                let sh_offset = r32(bytes, sh_off + 16)? as usize;
                let sh_size = r32(bytes, sh_off + 20)? as usize;
                let sh_link = r32(bytes, sh_off + 24)? as usize;
                let sh_entsize = r32(bytes, sh_off + 36)? as usize;
                if sh_entsize < 16 || sh_link >= e_shnum {
                    continue;
                }
                let str_sh_off = e_shoff + sh_link * e_shentsize;
                let str_offset = r32(bytes, str_sh_off + 16)? as usize;
                let str_size = r32(bytes, str_sh_off + 20)? as usize;
                let strtab = bytes
                    .get(str_offset..str_offset + str_size)
                    .ok_or(ElfError::TruncatedTable)?;

                let count = sh_size / sh_entsize;
                for s in 0..count {
                    let sym_off = sh_offset + s * sh_entsize;
                    let st_name = r32(bytes, sym_off)? as usize;
                    let st_value = r32(bytes, sym_off + 4)?;
                    let st_size = r32(bytes, sym_off + 8)?;
                    let st_info = *bytes.get(sym_off + 12).ok_or(ElfError::TruncatedTable)?;
                    let st_shndx = r16(bytes, sym_off + 14)?;
                    let st_type = st_info & 0x0F;
                    // Keep functions, objects, or notype symbols in valid sections
                    if st_shndx == 0 || st_name == 0 || st_name >= strtab.len() {
                        continue;
                    }
                    if st_type == 3 {
                        continue; // STT_SECTION
                    }
                    let end = strtab[st_name..]
                        .iter()
                        .position(|&c| c == 0)
                        .map(|p| st_name + p)
                        .unwrap_or(strtab.len());
                    if let Ok(name) = std::str::from_utf8(&strtab[st_name..end]) {
                        if !name.is_empty() && !name.starts_with('$') {
                            sym_by_addr.entry(st_value).or_default().push(name.to_string());
                            addr_by_sym.entry(name.to_string()).or_insert(st_value);
                            symbols.push(ElfSymbol {
                                addr: st_value,
                                size: st_size,
                                name: name.to_string(),
                                is_func: st_type == 2,
                            });
                        }
                    }
                }
            }
        }

        symbols.sort_by(|a, b| a.addr.cmp(&b.addr).then_with(|| b.is_func.cmp(&a.is_func)));

        Ok(Self {
            entry,
            segments,
            sym_by_addr,
            addr_by_sym,
            symbols,
        })
    }

    pub fn symbol_at(&self, addr: u32) -> Option<&str> {
        self.sym_by_addr
            .get(&addr)
            .and_then(|v| v.first().map(|s| s.as_str()))
    }

    pub fn resolve_func(&self, pc: u32) -> Option<&ElfSymbol> {
        resolve_func_in(&self.symbols, pc)
    }

    pub fn detect_stack_bounds(&self) -> (Option<u32>, Option<u32>) {
        let top = ["_eusrstack", "_estack", "__stack_top"]
            .iter()
            .find_map(|k| self.addr_by_sym.get(*k).copied());
        let limit = ["_ebss", "_end", "__bss_end__"]
            .iter()
            .find_map(|k| self.addr_by_sym.get(*k).copied());
        (top, limit)
    }
}

pub(crate) fn resolve_func_in(symbols: &[ElfSymbol], pc: u32) -> Option<&ElfSymbol> {
    let idx = symbols.partition_point(|s| s.addr <= pc);
    for i in (0..idx).rev() {
        let s = &symbols[i];
        if !s.is_func {
            continue;
        }
        if s.size > 0 && pc < s.addr.saturating_add(s.size) {
            return Some(s);
        }
        if s.size == 0 && pc.saturating_sub(s.addr) < 4096 {
            return Some(s);
        }
        if pc.saturating_sub(s.addr) >= 8192 {
            break;
        }
    }
    for i in (0..idx).rev() {
        let s = &symbols[i];
        if s.is_func && pc.saturating_sub(s.addr) < 4096 {
            return Some(s);
        }
        if pc.saturating_sub(s.addr) >= 4096 {
            break;
        }
    }
    None
}
