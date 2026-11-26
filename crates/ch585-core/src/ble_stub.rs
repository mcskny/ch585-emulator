use std::collections::{HashMap, VecDeque};

use crate::cpu::{Cpu, PendingCallback};
use crate::rf::{
    format_mac_colon, read_slice_mem, read_u16_mem, read_u32_mem, read_u8_mem, write_slice_sram,
    write_u8_sram, RfOutgoingFrame, RfPacketKind, RfRoleListEntry, RfMode, RfState,
};

const SNV_BOND_MAGIC: &[u8; 8] = b"CH58BOND";
const SNV_BOND_RECORD_SIZE: usize = 48;
const SNV_BLOCK_BYTES: usize = 256;
const MAX_SNV_BONDS: usize = 4;

fn crc32_ieee(bytes: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &b in bytes {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BleCentralState {
    Idle,
    Scanning,
    Connected,
    PairedBonded,
    Disconnected,
}

impl BleCentralState {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::Scanning => "Scanning",
            Self::Connected => "Connected",
            Self::PairedBonded => "PairedBonded",
            Self::Disconnected => "Disconnected",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BleConnParams {
    pub interval: u16,
    pub latency: u16,
    pub timeout: u16,
}

impl Default for BleConnParams {
    fn default() -> Self {
        Self {
            interval: 12, // 15.0 ms (12 * 1.25 ms)
            latency: 0,
            timeout: 400, // 4.0 s (400 * 10 ms)
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BleBondRecord {
    pub peer_addr: [u8; 6],
    pub peer_addr_type: u8,
    pub ltk: [u8; 16],
    pub ediv: u16,
    pub rand: [u8; 8],
    pub cccd_enabled: bool,
    pub bonded_at_ns: u64,
}

#[derive(Debug, Clone)]
pub struct BleTimestampedEvent {
    pub time_ns: u64,
    pub cycle: u64,
    pub kind: String,
    pub detail: String,
}

#[derive(Debug, Clone)]
pub struct BleHidReport {
    pub time_ns: u64,
    pub cycle: u64,
    pub conn_handle: u16,
    pub report_id: u8,
    pub report_type: u8,
    pub attr_handle: u16,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone)]
pub struct BleGattService {
    pub attrs_ptr: u32,
    pub num_attrs: u16,
    pub read_cb: u32,
    pub write_cb: u32,
    pub auth_cb: u32,
}

#[derive(Debug, Clone)]
pub struct BleScanResult {
    pub advertising: bool,
    pub adv_data: Vec<u8>,
    pub scan_rsp_data: Vec<u8>,
    pub device_name: Option<String>,
    pub mac_addr: [u8; 6],
}

#[derive(Debug, Clone)]
pub struct TmosTimerEntry {
    pub task_id: u8,
    pub event: u16,
    pub deadline_ns: u64,
    pub period_ns: Option<u64>,
}

#[derive(Debug, Clone)]
pub enum BleHookKind {
    // BLE GAP / Bond / GATT / HID
    BleInit,
    BleLibInit,
    GapRolePeripheralStartDevice,
    GapRoleSetParam,
    GapRoleGetParam,
    GapRoleConnParamUpdateReq,
    GapRoleTerminateLink,
    GapRoleUpdatePhy,
    GapBondMgrSetParam,
    GapBondMgrGetParam,
    GattServAppRegisterService,
    HidDevReport,
    GattNotify,
    GattIndicate,
    // TMOS & LowPower
    TmosStartTask,
    TmosStartReloadTask,
    TmosStopTask,
    Ch58xLowPower,
    LowPowerSleep,
    // 2.4 GHz RF (`wchrf.h`)
    RfRoleFastInit,
    RfRoleBasicInit,
    RfRoleSetParam,
    RfRoleStop,
    RfRoleShut,
    RfRoleSwitchMode,
    RfRoleClearTxData,
    RfRoleGetStatus,
    RfRoleNoopOk,
    RfBoundSetFreqList,
    RfBoundSetSpeedType,
    RfBoundStartHost,
    RfBoundStartDevice,
    RfIpSetTxPower,
    RfIpReadRssi,
    RfIpReadCrc,
    RfIpSetTxStart,
    RfIpSetTxParm,
    RfIpSetRx,
    RfTx,
    RfRx,
}

/// Virtual BLE / 2.4G / RF API bridge (`0x4000_C000..0x4000_DFFF` region) + Virtual BLE Central.
#[derive(Debug, Clone)]
pub struct BleBridge {
    /// PC -> API hook mapping resolved from ELF `.symtab`
    pub hooks: HashMap<u32, BleHookKind>,
    pub advertising_enabled: bool,
    pub adv_data: Vec<u8>,
    pub scan_rsp_data: Vec<u8>,
    pub local_mac: [u8; 6],
    pub event_log: Vec<String>,
    pub timestamped_events: Vec<BleTimestampedEvent>,
    pub hid_reports: Vec<Vec<u8>>,
    pub timestamped_hid_reports: Vec<BleHidReport>,

    // Virtual BLE Central & callback state
    pub central_state: BleCentralState,
    pub conn_handle: u16,
    pub conn_params: BleConnParams,
    pub peer_addr: [u8; 6],
    pub central_addr: [u8; 6],
    pub bonds: Vec<BleBondRecord>,
    pub snv_addr: u32,
    pub gap_task_id: u8,
    pub gap_state_cb: u32,
    pub gap_param_update_cb: u32,
    pub bond_passcode_cb: u32,
    pub bond_pair_state_cb: u32,
    pub gatt_services: Vec<BleGattService>,
    pub gatt_bm_free_addr: u32,

    // TMOS & Sleep tracking
    pub tmos_timers: Vec<TmosTimerEntry>,
    pub tmos_start_calls: u64,
    pub low_power_calls: u64,
    pub sleep_hook_calls: u64,
    pub lib_idle_cb: u32,

    // 2.4 GHz RF virtual state
    pub rf: RfState,

    // Pending RISC-V callbacks dispatched at safe instruction boundaries
    pub pending_callbacks: VecDeque<PendingCallback>,

    cb_seq: u32,
    last_hid_sig: Option<(u64, Vec<u8>)>,
    last_observed: Option<(u32, u64)>,
    min_pc: u32,
    max_pc: u32,
    hook_bloom: [u64; 64],
    hook_table: Vec<(u32, BleHookKind)>,
}

impl Default for BleBridge {
    fn default() -> Self {
        Self::new()
    }
}

impl BleBridge {
    pub fn new() -> Self {
        Self {
            hooks: HashMap::new(),
            advertising_enabled: false,
            adv_data: Vec::new(),
            scan_rsp_data: Vec::new(),
            local_mac: [0x58, 0x50, 0x00, 0x12, 0x34, 0x56],
            event_log: Vec::new(),
            timestamped_events: Vec::new(),
            hid_reports: Vec::new(),
            timestamped_hid_reports: Vec::new(),
            central_state: BleCentralState::Idle,
            conn_handle: 0xFFFF,
            conn_params: BleConnParams::default(),
            peer_addr: [0; 6],
            central_addr: [0xC0, 0xDE, 0xBA, 0x5E, 0x00, 0x01],
            bonds: Vec::new(),
            snv_addr: 0x77E00,
            gap_task_id: 0,
            gap_state_cb: 0,
            gap_param_update_cb: 0,
            bond_passcode_cb: 0,
            bond_pair_state_cb: 0,
            gatt_services: Vec::new(),
            gatt_bm_free_addr: 0,
            tmos_timers: Vec::new(),
            tmos_start_calls: 0,
            low_power_calls: 0,
            sleep_hook_calls: 0,
            lib_idle_cb: 0,
            rf: RfState::new(),
            pending_callbacks: VecDeque::new(),
            cb_seq: 1,
            last_hid_sig: None,
            last_observed: None,
            min_pc: u32::MAX,
            max_pc: 0,
            hook_bloom: [0; 64],
            hook_table: Vec::new(),
        }
    }

    #[inline]
    pub fn has_hooks(&self) -> bool {
        !self.hooks.is_empty()
    }

    #[inline]
    pub fn has_pending_callbacks(&self) -> bool {
        !self.pending_callbacks.is_empty()
    }

    #[inline]
    pub fn in_hook_range(&self, pc: u32) -> bool {
        !self.hooks.is_empty() && pc >= self.min_pc && pc <= self.max_pc
    }

    #[inline(always)]
    fn bloom_slot(pc: u32) -> (usize, u64) {
        let bit = ((pc >> 1) & 4095) as usize;
        (bit >> 6, 1u64 << (bit & 63))
    }

    #[inline(always)]
    pub fn has_hook_at(&self, pc: u32) -> bool {
        if self.hooks.is_empty() {
            return false;
        }
        if self.hooks.len() != self.hook_table.len() {
            return self.hooks.contains_key(&pc);
        }
        if pc < self.min_pc || pc > self.max_pc {
            return false;
        }
        let (word, mask) = Self::bloom_slot(pc);
        if (self.hook_bloom[word] & mask) == 0 {
            return false;
        }
        self.hook_table.binary_search_by_key(&pc, |&(a, _)| a).is_ok()
    }

    #[inline]
    pub fn lookup_hook(&self, pc: u32) -> Option<BleHookKind> {
        if self.hooks.is_empty() {
            return None;
        }
        if self.hooks.len() != self.hook_table.len() {
            return self.hooks.get(&pc).cloned();
        }
        if pc < self.min_pc || pc > self.max_pc {
            return None;
        }
        let (word, mask) = Self::bloom_slot(pc);
        if (self.hook_bloom[word] & mask) == 0 {
            return None;
        }
        self.hook_table
            .binary_search_by_key(&pc, |&(a, _)| a)
            .ok()
            .map(|idx| self.hook_table[idx].1.clone())
    }

    #[inline]
    pub fn radio_active(&self) -> bool {
        self.advertising_enabled
            || matches!(
                self.central_state,
                BleCentralState::Connected | BleCentralState::PairedBonded
            )
            || self.rf.is_active()
    }

    pub fn on_soft_reset(&mut self, flash: &[u8]) {
        self.advertising_enabled = false;
        self.central_state = BleCentralState::Idle;
        self.conn_handle = 0xFFFF;
        self.gap_state_cb = 0;
        self.gap_param_update_cb = 0;
        self.bond_passcode_cb = 0;
        self.bond_pair_state_cb = 0;
        self.gatt_services.clear();
        self.tmos_timers.clear();
        self.pending_callbacks.clear();
        self.last_observed = None;
        self.rf.reset_transient();
        self.reload_bonds_from_flash(flash);
    }

    pub fn next_event_ns(&self) -> Option<u64> {
        let rf_ns = self.rf.next_event_ns();
        let tmos_ns = self.tmos_timers.iter().map(|t| t.deadline_ns).min();
        match (rf_ns, tmos_ns) {
            (Some(a), Some(b)) => Some(a.min(b)),
            (Some(a), None) => Some(a),
            (None, Some(b)) => Some(b),
            (None, None) => None,
        }
    }

    pub fn resolve_symbols(&mut self, addr_by_sym: &HashMap<String, u32>) {
        self.hooks.clear();
        self.hook_bloom = [0; 64];
        self.hook_table.clear();
        self.last_observed = None;
        self.min_pc = u32::MAX;
        self.max_pc = 0;
        self.gatt_bm_free_addr = addr_by_sym.get("GATT_bm_free").copied().unwrap_or(0);

        let map = [
            // BLE GAP / Bond / GATT / HID
            ("CH58x_BLEInit", BleHookKind::BleInit),
            ("BLE_LibInit", BleHookKind::BleLibInit),
            ("GAPRole_PeripheralStartDevice", BleHookKind::GapRolePeripheralStartDevice),
            ("GAPRole_SetParameter", BleHookKind::GapRoleSetParam),
            ("GAPRole_GetParameter", BleHookKind::GapRoleGetParam),
            ("GAPRole_PeripheralConnParamUpdateReq", BleHookKind::GapRoleConnParamUpdateReq),
            ("GAPRole_TerminateLink", BleHookKind::GapRoleTerminateLink),
            ("GAPRole_UpdatePHY", BleHookKind::GapRoleUpdatePhy),
            ("GAPBondMgr_SetParameter", BleHookKind::GapBondMgrSetParam),
            ("GAPBondMgr_GetParameter", BleHookKind::GapBondMgrGetParam),
            ("GATTServApp_RegisterService", BleHookKind::GattServAppRegisterService),
            ("HidDev_Report", BleHookKind::HidDevReport),
            ("GATT_Notification", BleHookKind::GattNotify),
            ("GATT_Indication", BleHookKind::GattIndicate),
            // TMOS & LowPower
            ("tmos_start_task", BleHookKind::TmosStartTask),
            ("tmos_start_reload_task", BleHookKind::TmosStartReloadTask),
            ("tmos_stop_task", BleHookKind::TmosStopTask),
            ("CH58x_LowPower", BleHookKind::Ch58xLowPower),
            ("LowPower_Idle", BleHookKind::LowPowerSleep),
            ("LowPower_Halt", BleHookKind::LowPowerSleep),
            ("LowPower_Sleep", BleHookKind::LowPowerSleep),
            ("LowPower_Halt_WFE", BleHookKind::LowPowerSleep),
            ("LowPower_Sleep_WFE", BleHookKind::LowPowerSleep),
            // 2.4 GHz RF (`wchrf.h`)
            ("RFRole_FastInit", BleHookKind::RfRoleFastInit),
            ("RFRole_BasicInit", BleHookKind::RfRoleBasicInit),
            ("RFRole_SetParam", BleHookKind::RfRoleSetParam),
            ("RFRole_Stop", BleHookKind::RfRoleStop),
            ("RFRole_Shut", BleHookKind::RfRoleShut),
            ("RFRole_SwitchMode", BleHookKind::RfRoleSwitchMode),
            ("RFRole_ClearTxData", BleHookKind::RfRoleClearTxData),
            ("RFRole_GetStatus", BleHookKind::RfRoleGetStatus),
            ("RFRole_LowPower", BleHookKind::RfRoleNoopOk),
            ("RFRole_PHYUpdate", BleHookKind::RfRoleNoopOk),
            ("RFRole_MapCheck", BleHookKind::RfRoleNoopOk),
            ("RFRole_StartLpTx", BleHookKind::RfRoleNoopOk),
            ("RFIP_Calibration", BleHookKind::RfRoleNoopOk),
            ("RFIP_WakeUpRegInit", BleHookKind::RfRoleNoopOk),
            ("RFIP_SetTxDelayTime", BleHookKind::RfRoleNoopOk),
            ("RFBound_Stop", BleHookKind::RfRoleStop),
            ("RFBound_SetFrequencyList", BleHookKind::RfBoundSetFreqList),
            ("RFBound_SetSpeedType", BleHookKind::RfBoundSetSpeedType),
            ("RFBound_StartHost", BleHookKind::RfBoundStartHost),
            ("RFBound_Start8kHost", BleHookKind::RfBoundStartHost),
            ("RFBound_StartLpHost", BleHookKind::RfBoundStartHost),
            ("RFBound_StartMutiHost", BleHookKind::RfBoundStartHost),
            ("RFBound_StartDevice", BleHookKind::RfBoundStartDevice),
            ("RFBound_Start8kDevice", BleHookKind::RfBoundStartDevice),
            ("RFBound_StartLpDevice", BleHookKind::RfBoundStartDevice),
            ("RFBound_StartMutiDevice", BleHookKind::RfBoundStartDevice),
            ("RFIP_SetTxPower", BleHookKind::RfIpSetTxPower),
            ("RFIP_ReadRssi", BleHookKind::RfIpReadRssi),
            ("RFIP_ReadCrc", BleHookKind::RfIpReadCrc),
            ("RFIP_SetTxStart", BleHookKind::RfIpSetTxStart),
            ("RFIP_SetTxParm", BleHookKind::RfIpSetTxParm),
            ("RFIP_SetRx", BleHookKind::RfIpSetRx),
            ("RF_Tx", BleHookKind::RfTx),
            ("RF_Rx", BleHookKind::RfRx),
        ];
        for (name, kind) in map {
            if let Some(&addr) = addr_by_sym.get(name) {
                self.hooks.insert(addr, kind.clone());
                let (word, mask) = Self::bloom_slot(addr);
                self.hook_bloom[word] |= mask;
                self.hook_table.push((addr, kind));
                self.min_pc = self.min_pc.min(addr);
                self.max_pc = self.max_pc.max(addr);
            }
        }
        self.hook_table.sort_unstable_by_key(|&(a, _)| a);
        self.hook_table.dedup_by_key(|entry| entry.0);
    }

    fn next_tag(&mut self, prefix: u32) -> u32 {
        let s = self.cb_seq;
        self.cb_seq = self.cb_seq.wrapping_add(1);
        prefix | (s & 0x00FF_FFFF)
    }

    fn push_event(&mut self, time_ns: u64, cycle: u64, kind: &str, detail: String) {
        if self.event_log.len() >= 256 {
            self.event_log.drain(0..64);
        }
        self.event_log.push(detail.clone());
        if self.timestamped_events.len() >= 512 {
            self.timestamped_events.drain(0..128);
        }
        self.timestamped_events.push(BleTimestampedEvent {
            time_ns,
            cycle,
            kind: kind.to_string(),
            detail,
        });
    }

    fn resolve_snv_flash_offset(snv_addr: u32) -> usize {
        if (0x70000..=0x77E00).contains(&snv_addr) {
            snv_addr as usize
        } else if snv_addr > 0 && snv_addr <= 0x7E00 {
            0x70000 + (snv_addr as usize)
        } else {
            0x77E00
        }
    }

    pub fn save_bonds_to_flash(&self, flash: &mut [u8]) -> bool {
        let off = Self::resolve_snv_flash_offset(self.snv_addr);
        if off + SNV_BLOCK_BYTES > flash.len() {
            return false;
        }
        let count = self.bonds.len().min(MAX_SNV_BONDS);
        let mut blk = vec![0xFFu8; SNV_BLOCK_BYTES];
        blk[0..8].copy_from_slice(SNV_BOND_MAGIC);
        blk[8..10].copy_from_slice(&1u16.to_le_bytes());
        blk[10..12].copy_from_slice(&(count as u16).to_le_bytes());
        for (idx, bond) in self.bonds.iter().take(count).enumerate() {
            let b = &mut blk[16 + idx * SNV_BOND_RECORD_SIZE..16 + (idx + 1) * SNV_BOND_RECORD_SIZE];
            b.fill(0);
            b[0..6].copy_from_slice(&bond.peer_addr);
            b[6] = bond.peer_addr_type;
            b[7] = u8::from(bond.cccd_enabled);
            b[8..24].copy_from_slice(&bond.ltk);
            b[24..26].copy_from_slice(&bond.ediv.to_le_bytes());
            b[26..34].copy_from_slice(&bond.rand);
            b[34..42].copy_from_slice(&bond.bonded_at_ns.to_le_bytes());
        }
        let payload_end = 16 + count * SNV_BOND_RECORD_SIZE;
        let mut crc_input = Vec::with_capacity(12 + count * SNV_BOND_RECORD_SIZE);
        crc_input.extend_from_slice(&blk[0..12]);
        crc_input.extend_from_slice(&blk[16..payload_end]);
        let crc = crc32_ieee(&crc_input);
        blk[12..16].copy_from_slice(&crc.to_le_bytes());
        flash[off..off + SNV_BLOCK_BYTES].copy_from_slice(&blk);
        true
    }

    pub fn reload_bonds_from_flash(&mut self, flash: &[u8]) -> usize {
        let off = Self::resolve_snv_flash_offset(self.snv_addr);
        if off + SNV_BLOCK_BYTES > flash.len() {
            return self.bonds.len();
        }
        let blk = &flash[off..off + SNV_BLOCK_BYTES];
        if &blk[0..8] != SNV_BOND_MAGIC {
            return self.bonds.len();
        }
        let count = u16::from_le_bytes([blk[10], blk[11]]) as usize;
        if count > MAX_SNV_BONDS {
            return self.bonds.len();
        }
        let stored_crc = u32::from_le_bytes([blk[12], blk[13], blk[14], blk[15]]);
        let payload_end = 16 + count * SNV_BOND_RECORD_SIZE;
        let mut crc_input = Vec::with_capacity(12 + count * SNV_BOND_RECORD_SIZE);
        crc_input.extend_from_slice(&blk[0..12]);
        crc_input.extend_from_slice(&blk[16..payload_end]);
        if crc32_ieee(&crc_input) != stored_crc {
            return self.bonds.len();
        }
        let mut loaded = Vec::with_capacity(count);
        for idx in 0..count {
            let b = &blk[16 + idx * SNV_BOND_RECORD_SIZE..16 + (idx + 1) * SNV_BOND_RECORD_SIZE];
            let mut peer_addr = [0u8; 6];
            peer_addr.copy_from_slice(&b[0..6]);
            let mut ltk = [0u8; 16];
            ltk.copy_from_slice(&b[8..24]);
            let ediv = u16::from_le_bytes([b[24], b[25]]);
            let mut rand = [0u8; 8];
            rand.copy_from_slice(&b[26..34]);
            let bonded_at_ns = u64::from_le_bytes([
                b[34], b[35], b[36], b[37], b[38], b[39], b[40], b[41],
            ]);
            loaded.push(BleBondRecord {
                peer_addr,
                peer_addr_type: b[6],
                ltk,
                ediv,
                rand,
                cccd_enabled: b[7] != 0,
                bonded_at_ns,
            });
        }
        self.bonds = loaded;
        self.bonds.len()
    }

    pub fn erase_bonds_in_flash(&mut self, flash: &mut [u8]) {
        self.bonds.clear();
        let off = Self::resolve_snv_flash_offset(self.snv_addr);
        if off + SNV_BLOCK_BYTES <= flash.len() {
            flash[off..off + SNV_BLOCK_BYTES].fill(0xFF);
        }
    }

    fn parse_adv_name(adv: &[u8], scan_rsp: &[u8]) -> Option<String> {
        for data in [scan_rsp, adv] {
            let mut pos = 0usize;
            while pos < data.len() {
                let len = data[pos] as usize;
                if len == 0 || pos + 1 + len > data.len() {
                    break;
                }
                let ad_type = data[pos + 1];
                if ad_type == 0x09 || ad_type == 0x08 {
                    return Some(String::from_utf8_lossy(&data[pos + 2..pos + 1 + len]).to_string());
                }
                pos += 1 + len;
            }
        }
        None
    }

    /// Virtual BLE Central: scan peripheral advertising / scan response data.
    pub fn central_scan(&mut self, now_ns: u64, cycle: u64) -> Option<BleScanResult> {
        self.central_state = BleCentralState::Scanning;
        if !self.advertising_enabled {
            self.push_event(
                now_ns,
                cycle,
                "BleScanEmpty",
                format!("Central scan@{now_ns}ns: no advertiser active"),
            );
            return None;
        }
        let device_name = Self::parse_adv_name(&self.adv_data, &self.scan_rsp_data);
        let res = BleScanResult {
            advertising: true,
            adv_data: self.adv_data.clone(),
            scan_rsp_data: self.scan_rsp_data.clone(),
            device_name: device_name.clone(),
            mac_addr: self.local_mac,
        };
        self.push_event(
            now_ns,
            cycle,
            "BleScanFound",
            format!(
                "Central found addr={} name={:?} adv={}B scan_rsp={}B",
                format_mac_colon(&self.local_mac),
                device_name,
                self.adv_data.len(),
                self.scan_rsp_data.len()
            ),
        );
        Some(res)
    }

    fn queue_gap_state_cb(&mut self, new_state: u32, event_blob: Vec<u8>) {
        if self.gap_state_cb == 0 {
            return;
        }
        let tag = self.next_tag(0xB100_0000);
        let mut args = [0u32; 8];
        args[0] = new_state;
        self.pending_callbacks.push_back(PendingCallback {
            entry_pc: self.gap_state_cb,
            args,
            stack_blob: event_blob,
            blob_arg_patches: vec![(1, 0)],
            read_back: None,
            tag,
            wake_from_wfi: true,
        });
    }

    fn queue_param_update_cb(&mut self, conn_handle: u16, p: BleConnParams) {
        if self.gap_param_update_cb == 0 {
            return;
        }
        let tag = self.next_tag(0xB200_0000);
        let mut args = [0u32; 8];
        args[0] = conn_handle as u32;
        args[1] = p.interval as u32;
        args[2] = p.latency as u32;
        args[3] = p.timeout as u32;
        self.pending_callbacks.push_back(PendingCallback {
            entry_pc: self.gap_param_update_cb,
            args,
            stack_blob: Vec::new(),
            blob_arg_patches: Vec::new(),
            read_back: None,
            tag,
            wake_from_wfi: true,
        });
    }

    fn queue_pair_state_cb(&mut self, conn_handle: u16, state: u8, status: u8) {
        if self.bond_pair_state_cb == 0 {
            return;
        }
        let tag = self.next_tag(0xB300_0000);
        let mut args = [0u32; 8];
        args[0] = conn_handle as u32;
        args[1] = state as u32;
        args[2] = status as u32;
        self.pending_callbacks.push_back(PendingCallback {
            entry_pc: self.bond_pair_state_cb,
            args,
            stack_blob: Vec::new(),
            blob_arg_patches: Vec::new(),
            read_back: None,
            tag,
            wake_from_wfi: true,
        });
    }

    /// Virtual BLE Central: establish GAP connection (`GAPROLE_CONNECTED`).
    pub fn central_connect(&mut self, peer_addr: [u8; 6], now_ns: u64, cycle: u64) -> bool {
        let addr = if peer_addr == [0; 6] {
            self.central_addr
        } else {
            peer_addr
        };
        self.peer_addr = addr;
        self.conn_handle = 0x0000;
        self.central_state = BleCentralState::Connected;
        self.advertising_enabled = false;

        // Build `gapEstLinkReqEvent_t` (24B)
        let mut ev = vec![0u8; 24];
        ev[0] = 0xD0; // GAP_MSG_EVENT
        ev[1] = 0x00; // SUCCESS
        ev[2] = 0x05; // GAP_LINK_ESTABLISHED_EVENT
        ev[3] = 0x00; // ADDRTYPE_PUBLIC
        ev[4..10].copy_from_slice(&addr);
        ev[10..12].copy_from_slice(&self.conn_handle.to_le_bytes());
        ev[12] = 0x04; // GAP_PROFILE_PERIPHERAL
        ev[14..16].copy_from_slice(&self.conn_params.interval.to_le_bytes());
        ev[16..18].copy_from_slice(&self.conn_params.latency.to_le_bytes());
        ev[18..20].copy_from_slice(&self.conn_params.timeout.to_le_bytes());

        self.push_event(
            now_ns,
            cycle,
            "BleConnect",
            format!(
                "Central connected handle=0x{:04X} peer={}",
                self.conn_handle,
                format_mac_colon(&addr)
            ),
        );
        self.queue_gap_state_cb(4 /* GAPROLE_CONNECTED */, ev);
        true
    }

    /// Virtual BLE Central: pair & persist bond record into chip DataFlash SNV.
    pub fn central_pair_and_bond(
        &mut self,
        flash: &mut [u8],
        flash_dirty: &mut bool,
        now_ns: u64,
        cycle: u64,
    ) -> bool {
        if !matches!(
            self.central_state,
            BleCentralState::Connected | BleCentralState::PairedBonded
        ) {
            return false;
        }
        let mut ltk = [0u8; 16];
        for i in 0..16 {
            ltk[i] = self.peer_addr[i % 6] ^ (0xA5u8.wrapping_add(i as u8 * 17));
        }
        let record = BleBondRecord {
            peer_addr: self.peer_addr,
            peer_addr_type: 0,
            ltk,
            ediv: u16::from_le_bytes([self.peer_addr[0], self.peer_addr[1]]),
            rand: [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
            cccd_enabled: self
                .bonds
                .iter()
                .find(|b| b.peer_addr == self.peer_addr)
                .map(|b| b.cccd_enabled)
                .unwrap_or(false),
            bonded_at_ns: now_ns,
        };
        if let Some(existing) = self.bonds.iter_mut().find(|b| b.peer_addr == record.peer_addr) {
            *existing = record;
        } else {
            if self.bonds.len() >= MAX_SNV_BONDS {
                self.bonds.remove(0);
            }
            self.bonds.push(record);
        }
        if self.save_bonds_to_flash(flash) {
            *flash_dirty = true;
        }
        self.central_state = BleCentralState::PairedBonded;
        let handle = self.conn_handle;
        self.queue_pair_state_cb(handle, 0, 0); // STARTED
        self.queue_pair_state_cb(handle, 1, 0); // COMPLETE
        self.queue_pair_state_cb(handle, 2, 0); // BONDED
        self.queue_pair_state_cb(handle, 3, 0); // BOND_SAVED
        self.push_event(
            now_ns,
            cycle,
            "BlePairBond",
            format!(
                "Central paired & bonded peer={} snv=0x{:05X} (bonds={})",
                format_mac_colon(&self.peer_addr),
                Self::resolve_snv_flash_offset(self.snv_addr),
                self.bonds.len()
            ),
        );
        true
    }

    fn find_gatt_attrs(
        &self,
        flash: &[u8],
        sram: &[u8],
        target_uuid16: u16,
    ) -> Vec<(u32, u8, u32, u32)> {
        let mut found = Vec::new();
        for svc in &self.gatt_services {
            for idx in 0..svc.num_attrs {
                let attr_addr = svc.attrs_ptr.wrapping_add((idx as u32) * 16);
                let uuid_len = read_u8_mem(flash, sram, attr_addr);
                if uuid_len == 2 {
                    let uuid_ptr = read_u32_mem(flash, sram, attr_addr.wrapping_add(4));
                    let uuid16 = read_u16_mem(flash, sram, uuid_ptr);
                    if uuid16 == target_uuid16 {
                        let perms = read_u8_mem(flash, sram, attr_addr.wrapping_add(8));
                        found.push((attr_addr, perms, svc.read_cb, svc.write_cb));
                    }
                }
            }
        }
        found
    }

    /// Virtual BLE Central: enable CCCD (0x2902) notifications across registered GATT services.
    pub fn central_enable_notifications(
        &mut self,
        flash: &mut [u8],
        sram: &[u8],
        flash_dirty: &mut bool,
        now_ns: u64,
        cycle: u64,
    ) -> usize {
        if !matches!(
            self.central_state,
            BleCentralState::Connected | BleCentralState::PairedBonded
        ) {
            return 0;
        }
        let attrs = self.find_gatt_attrs(flash, sram, 0x2902);
        let mut count = 0usize;
        for (attr_addr, _perms, _read_cb, write_cb) in attrs {
            if write_cb == 0 {
                continue;
            }
            let tag = self.next_tag(0xB400_0000);
            let mut args = [0u32; 8];
            args[0] = self.conn_handle as u32;
            args[1] = attr_addr;
            args[3] = 2; // len
            args[4] = 0; // offset
            args[5] = 0x12; // ATT_WRITE_REQ
            self.pending_callbacks.push_back(PendingCallback {
                entry_pc: write_cb,
                args,
                stack_blob: vec![0x01, 0x00],
                blob_arg_patches: vec![(2, 0)],
                read_back: None,
                tag,
                wake_from_wfi: true,
            });
            count += 1;
        }
        if count > 0 {
            if let Some(bond) = self.bonds.iter_mut().find(|b| b.peer_addr == self.peer_addr) {
                if !bond.cccd_enabled {
                    bond.cccd_enabled = true;
                    if self.save_bonds_to_flash(flash) {
                        *flash_dirty = true;
                    }
                }
            }
        }
        self.push_event(
            now_ns,
            cycle,
            "BleEnableCccd",
            format!("Central enabled {count} GATT CCCD notifications"),
        );
        count
    }

    /// Virtual BLE Central: write characteristic by 16-bit UUID (e.g. HID OUT report `0x2A4D` or `0x2A32`).
    pub fn central_gatt_write_uuid(
        &mut self,
        flash: &[u8],
        sram: &[u8],
        target_uuid16: u16,
        data: &[u8],
        now_ns: u64,
        cycle: u64,
    ) -> bool {
        if !matches!(
            self.central_state,
            BleCentralState::Connected | BleCentralState::PairedBonded
        ) {
            return false;
        }
        let attrs = self.find_gatt_attrs(flash, sram, target_uuid16);
        for (attr_addr, perms, _read_cb, write_cb) in attrs {
            if write_cb == 0 || (perms & 0xAA) == 0 {
                continue;
            }
            let tag = self.next_tag(0xB500_0000);
            let mut args = [0u32; 8];
            args[0] = self.conn_handle as u32;
            args[1] = attr_addr;
            args[3] = data.len() as u32;
            args[4] = 0;
            args[5] = 0x12; // ATT_WRITE_REQ
            self.pending_callbacks.push_back(PendingCallback {
                entry_pc: write_cb,
                args,
                stack_blob: data.to_vec(),
                blob_arg_patches: vec![(2, 0)],
                read_back: None,
                tag,
                wake_from_wfi: true,
            });
            self.push_event(
                now_ns,
                cycle,
                "BleGattWrite",
                format!("Central GATT write uuid=0x{target_uuid16:04X} {}B", data.len()),
            );
            return true;
        }
        false
    }

    /// Virtual BLE Central: read characteristic by 16-bit UUID; returns callback `tag` to inspect in `cpu.last_callback_results`.
    pub fn central_gatt_read_uuid(
        &mut self,
        flash: &[u8],
        sram: &[u8],
        target_uuid16: u16,
        now_ns: u64,
        cycle: u64,
    ) -> Option<u32> {
        if !matches!(
            self.central_state,
            BleCentralState::Connected | BleCentralState::PairedBonded
        ) {
            return None;
        }
        let attrs = self.find_gatt_attrs(flash, sram, target_uuid16);
        for (attr_addr, perms, read_cb, _write_cb) in attrs {
            if read_cb == 0 || (perms & 0x55) == 0 {
                continue;
            }
            let tag = self.next_tag(0xB600_0000);
            // Layout in stack_blob: [0..2]: uint16_t len, [4..68]: uint8_t value[64]
            let blob = vec![0u8; 68];
            let mut args = [0u32; 8];
            args[0] = self.conn_handle as u32;
            args[1] = attr_addr;
            args[4] = 0;  // offset
            args[5] = 64; // maxLen
            args[6] = 0x0A; // ATT_READ_REQ
            self.pending_callbacks.push_back(PendingCallback {
                entry_pc: read_cb,
                args,
                stack_blob: blob,
                blob_arg_patches: vec![(2, 4), (3, 0)], // a2=pValue(&blob[4]), a3=pLen(&blob[0])
                read_back: Some((0, 4, 64)),
                tag,
                wake_from_wfi: true,
            });
            self.push_event(
                now_ns,
                cycle,
                "BleGattRead",
                format!("Central GATT read uuid=0x{target_uuid16:04X} tag=0x{tag:08X}"),
            );
            return Some(tag);
        }
        None
    }

    pub fn central_update_conn_params(
        &mut self,
        interval: u16,
        latency: u16,
        timeout: u16,
        now_ns: u64,
        cycle: u64,
    ) -> bool {
        if !matches!(
            self.central_state,
            BleCentralState::Connected | BleCentralState::PairedBonded
        ) {
            return false;
        }
        self.conn_params = BleConnParams {
            interval,
            latency,
            timeout,
        };
        let handle = self.conn_handle;
        self.queue_param_update_cb(handle, self.conn_params);
        self.push_event(
            now_ns,
            cycle,
            "BleConnParamUpdate",
            format!("ConnParams interval={interval} latency={latency} timeout={timeout}"),
        );
        true
    }

    pub fn central_disconnect(&mut self, reason: u8, now_ns: u64, cycle: u64) -> bool {
        if !matches!(
            self.central_state,
            BleCentralState::Connected | BleCentralState::PairedBonded
        ) {
            return false;
        }
        let handle = self.conn_handle;
        self.central_state = BleCentralState::Disconnected;
        self.conn_handle = 0xFFFF;

        // `gapTerminateLinkEvent_t` (8B)
        let mut ev = vec![0u8; 8];
        ev[0] = 0xD0; // GAP_MSG_EVENT
        ev[1] = 0x00;
        ev[2] = 0x06; // GAP_LINK_TERMINATED_EVENT
        ev[4..6].copy_from_slice(&handle.to_le_bytes());
        ev[6] = reason;
        ev[7] = 0x04;

        self.push_event(
            now_ns,
            cycle,
            "BleDisconnect",
            format!("Central disconnected reason=0x{reason:02X}"),
        );
        self.queue_gap_state_cb(3 /* GAPROLE_WAITING */, ev);
        true
    }

    /// Reconnect using persisted bond in DataFlash SNV.
    pub fn central_reconnect(
        &mut self,
        flash: &mut [u8],
        sram: &[u8],
        flash_dirty: &mut bool,
        now_ns: u64,
        cycle: u64,
    ) -> bool {
        if self.bonds.is_empty() {
            self.reload_bonds_from_flash(flash);
        }
        let Some(bond) = self.bonds.first().cloned() else {
            return false;
        };
        if !self.central_connect(bond.peer_addr, now_ns, cycle) {
            return false;
        }
        self.central_state = BleCentralState::PairedBonded;
        let handle = self.conn_handle;
        self.queue_pair_state_cb(handle, 2 /* BONDED */, 0);
        if bond.cccd_enabled {
            self.central_enable_notifications(flash, sram, flash_dirty, now_ns, cycle);
        }
        self.push_event(
            now_ns,
            cycle,
            "BleReconnect",
            format!(
                "Central reconnected via SNV bond peer={} cccd={}",
                format_mac_colon(&bond.peer_addr),
                bond.cccd_enabled
            ),
        );
        true
    }

    fn return_stub(cpu: &mut Cpu, ret_val: u32) -> bool {
        cpu.regs[10] = ret_val;
        cpu.pc = cpu.regs[1];
        true
    }

    fn record_hid_report(
        &mut self,
        time_ns: u64,
        cycle: u64,
        conn_handle: u16,
        report_id: u8,
        report_type: u8,
        attr_handle: u16,
        data: Vec<u8>,
    ) {
        if let Some((prev_ns, prev_data)) = &self.last_hid_sig {
            if time_ns.saturating_sub(*prev_ns) < 200_000 && *prev_data == data && attr_handle != 0 {
                if let Some(last) = self.timestamped_hid_reports.last_mut() {
                    if last.attr_handle == 0 {
                        last.attr_handle = attr_handle;
                    }
                }
                return;
            }
        }
        self.last_hid_sig = Some((time_ns, data.clone()));
        self.hid_reports.push(data.clone());
        if self.timestamped_hid_reports.len() >= 512 {
            self.timestamped_hid_reports.drain(0..128);
        }
        self.timestamped_hid_reports.push(BleHidReport {
            time_ns,
            cycle,
            conn_handle,
            report_id,
            report_type,
            attr_handle,
            data,
        });
    }

    fn queue_gatt_bm_free(&mut self, noti_bytes: &[u8], opcode: u8) {
        if self.gatt_bm_free_addr == 0 {
            return;
        }
        let tag = self.next_tag(0xBF00_0000);
        let mut args = [0u32; 8];
        args[1] = opcode as u32;
        self.pending_callbacks.push_back(PendingCallback {
            entry_pc: self.gatt_bm_free_addr,
            args,
            stack_blob: noti_bytes.to_vec(),
            blob_arg_patches: vec![(0, 0)],
            read_back: None,
            tag,
            wake_from_wfi: false,
        });
    }

    /// Inspect or intercept BLE/RF/TMOS library API call at `cpu.pc`.
    /// Returns `true` if the function was stubbed (returned to `ra`) and `cpu.step` should be skipped.
    #[inline]
    pub fn on_pc(
        &mut self,
        cpu: &mut Cpu,
        flash: &mut [u8],
        sram: &mut [u8],
        time_ns: u64,
        cycle: u64,
        flash_dirty: &mut bool,
    ) -> bool {
        let pc = cpu.pc;
        if self.last_observed == Some((pc, cpu.instret)) {
            return false;
        }
        let Some(kind) = self.lookup_hook(pc) else {
            return false;
        };
        self.last_observed = Some((pc, cpu.instret));

        match kind {
            BleHookKind::BleInit => {
                self.push_event(time_ns, cycle, "BleInit", "CH58x_BLEInit()".to_string());
                false
            }
            BleHookKind::BleLibInit => {
                let cfg_ptr = cpu.regs[10];
                if cfg_ptr != 0 {
                    let snv = read_u32_mem(flash, sram, cfg_ptr.wrapping_add(8));
                    if snv != 0 {
                        self.snv_addr = snv;
                    }
                    let mac = read_slice_mem(flash, sram, cfg_ptr.wrapping_add(26), 6);
                    if mac.len() == 6 && mac.iter().any(|&b| b != 0) {
                        self.local_mac.copy_from_slice(&mac);
                    }
                    self.lib_idle_cb = read_u32_mem(flash, sram, cfg_ptr.wrapping_add(36));
                    let reloaded = self.reload_bonds_from_flash(flash);
                    self.push_event(
                        time_ns,
                        cycle,
                        "BleLibInit",
                        format!(
                            "BLE_LibInit(snv=0x{:05X}, mac={}, bonds_reloaded={reloaded})",
                            self.snv_addr,
                            format_mac_colon(&self.local_mac)
                        ),
                    );
                }
                false
            }
            BleHookKind::GapRolePeripheralStartDevice => {
                self.gap_task_id = cpu.regs[10] as u8;
                let p_bond_cb = cpu.regs[11];
                let p_app_cb = cpu.regs[12];
                if p_bond_cb != 0 {
                    self.bond_passcode_cb = read_u32_mem(flash, sram, p_bond_cb);
                    self.bond_pair_state_cb = read_u32_mem(flash, sram, p_bond_cb.wrapping_add(4));
                }
                if p_app_cb != 0 {
                    self.gap_state_cb = read_u32_mem(flash, sram, p_app_cb);
                    self.gap_param_update_cb = read_u32_mem(flash, sram, p_app_cb.wrapping_add(8));
                }
                self.push_event(
                    time_ns,
                    cycle,
                    "GapStartDevice",
                    format!(
                        "GAPRole_PeripheralStartDevice(task={}, stateCB=0x{:08X}, pairCB=0x{:08X})",
                        self.gap_task_id, self.gap_state_cb, self.bond_pair_state_cb
                    ),
                );
                false
            }
            BleHookKind::GapRoleSetParam => {
                let param = cpu.regs[10];
                let len = cpu.regs[11] as usize;
                let ptr = cpu.regs[12];
                let data = read_slice_mem(flash, sram, ptr, len.min(64));
                match param {
                    0x0305 => {
                        self.advertising_enabled = data.first().copied().unwrap_or(0) != 0;
                        self.push_event(
                            time_ns,
                            cycle,
                            "GapAdvertEnabled",
                            format!("GAPRole_SetParameter(ADVERT_ENABLED={})", self.advertising_enabled),
                        );
                    }
                    0x0306 => {
                        self.adv_data = data;
                        self.push_event(
                            time_ns,
                            cycle,
                            "GapAdvertData",
                            format!("GAPRole_SetParameter(ADVERT_DATA, {}B)", self.adv_data.len()),
                        );
                    }
                    0x0307 => {
                        self.scan_rsp_data = data;
                        self.push_event(
                            time_ns,
                            cycle,
                            "GapScanRspData",
                            format!("GAPRole_SetParameter(SCAN_RSP_DATA, {}B)", self.scan_rsp_data.len()),
                        );
                    }
                    _ => {
                        self.push_event(
                            time_ns,
                            cycle,
                            "GapSetParam",
                            format!("GAPRole_SetParameter(0x{param:04X}, {len}B)"),
                        );
                    }
                }
                false
            }
            BleHookKind::GapRoleGetParam => {
                let param = cpu.regs[10];
                let ptr = cpu.regs[11];
                if param == 0x030D
                    && matches!(
                        self.central_state,
                        BleCentralState::Connected | BleCentralState::PairedBonded
                    )
                {
                    write_u8_sram(sram, ptr, 4); // GAPROLE_CONNECTED
                    return Self::return_stub(cpu, 0);
                }
                false
            }
            BleHookKind::GapRoleConnParamUpdateReq => {
                if matches!(
                    self.central_state,
                    BleCentralState::Connected | BleCentralState::PairedBonded
                ) {
                    let conn = cpu.regs[10] as u16;
                    let _min_int = cpu.regs[11] as u16;
                    let max_int = cpu.regs[12] as u16;
                    let lat = cpu.regs[13] as u16;
                    let to = cpu.regs[14] as u16;
                    let params = BleConnParams {
                        interval: max_int.max(_min_int).max(6),
                        latency: lat,
                        timeout: to.max(10),
                    };
                    self.conn_params = params;
                    self.queue_param_update_cb(conn, params);
                    self.push_event(
                        time_ns,
                        cycle,
                        "ConnParamUpdateReq",
                        format!("Peripheral requested interval={} lat={} to={}", params.interval, params.latency, params.timeout),
                    );
                    return Self::return_stub(cpu, 0);
                }
                false
            }
            BleHookKind::GapRoleTerminateLink => {
                if matches!(
                    self.central_state,
                    BleCentralState::Connected | BleCentralState::PairedBonded
                ) {
                    self.central_disconnect(0x16, time_ns, cycle);
                    return Self::return_stub(cpu, 0);
                }
                false
            }
            BleHookKind::GapRoleUpdatePhy => {
                if matches!(
                    self.central_state,
                    BleCentralState::Connected | BleCentralState::PairedBonded
                ) {
                    self.push_event(
                        time_ns,
                        cycle,
                        "PhyUpdate",
                        format!("GAPRole_UpdatePHY(tx=0x{:X}, rx=0x{:X})", cpu.regs[12], cpu.regs[13]),
                    );
                    return Self::return_stub(cpu, 0);
                }
                false
            }
            BleHookKind::GapBondMgrSetParam => {
                let param = cpu.regs[10];
                if param == 0x0410 {
                    // GAPBOND_ERASE_ALLBONDS
                    self.erase_bonds_in_flash(flash);
                    *flash_dirty = true;
                    self.push_event(time_ns, cycle, "BondEraseAll", "GAPBOND_ERASE_ALLBONDS -> DataFlash cleared".into());
                }
                false
            }
            BleHookKind::GapBondMgrGetParam => {
                let param = cpu.regs[10];
                let ptr = cpu.regs[11];
                if param == 0x0415 {
                    // GAPBOND_BOND_COUNT
                    write_u8_sram(sram, ptr, self.bonds.len() as u8);
                    return Self::return_stub(cpu, 0);
                }
                false
            }
            BleHookKind::GattServAppRegisterService => {
                let attrs_ptr = cpu.regs[10];
                let num_attrs = cpu.regs[11] as u16;
                let cbs_ptr = cpu.regs[13];
                let (read_cb, write_cb, auth_cb) = if cbs_ptr != 0 {
                    (
                        read_u32_mem(flash, sram, cbs_ptr),
                        read_u32_mem(flash, sram, cbs_ptr.wrapping_add(4)),
                        read_u32_mem(flash, sram, cbs_ptr.wrapping_add(8)),
                    )
                } else {
                    (0, 0, 0)
                };
                self.gatt_services.push(BleGattService {
                    attrs_ptr,
                    num_attrs,
                    read_cb,
                    write_cb,
                    auth_cb,
                });
                self.push_event(
                    time_ns,
                    cycle,
                    "GattRegisterService",
                    format!("GATTServApp_RegisterService(attrs={num_attrs}, r=0x{read_cb:08X}, w=0x{write_cb:08X})"),
                );
                false
            }
            BleHookKind::HidDevReport => {
                let id = cpu.regs[10] as u8;
                let rpt_type = cpu.regs[11] as u8;
                let len = (cpu.regs[12] as usize).min(64);
                let ptr = cpu.regs[13];
                let data = read_slice_mem(flash, sram, ptr, len);
                self.push_event(
                    time_ns,
                    cycle,
                    "HidDevReport",
                    format!("HidDev_Report(id={id}, type={rpt_type}, data={data:02X?})"),
                );
                self.record_hid_report(time_ns, cycle, self.conn_handle, id, rpt_type, 0, data);
                false
            }
            BleHookKind::GattNotify | BleHookKind::GattIndicate => {
                let conn = cpu.regs[10] as u16;
                let p_noti = cpu.regs[11];
                let handle = read_u16_mem(flash, sram, p_noti);
                let len = (read_u16_mem(flash, sram, p_noti.wrapping_add(2)) as usize).min(247);
                let p_val = read_u32_mem(flash, sram, p_noti.wrapping_add(4));
                let data = read_slice_mem(flash, sram, p_val, len);
                let is_ind = matches!(kind, BleHookKind::GattIndicate);
                let label = if is_ind { "GATT_Indication" } else { "GATT_Notification" };
                self.push_event(
                    time_ns,
                    cycle,
                    label,
                    format!("{label}(conn=0x{conn:04X}, handle=0x{handle:04X}, {len}B: {:02X?})", &data[..data.len().min(16)]),
                );
                self.record_hid_report(time_ns, cycle, conn, 0, 1, handle, data);
                if matches!(
                    self.central_state,
                    BleCentralState::Connected | BleCentralState::PairedBonded
                ) {
                    let noti_copy = read_slice_mem(flash, sram, p_noti, 8);
                    self.queue_gatt_bm_free(&noti_copy, if is_ind { 0x1D } else { 0x1B });
                    return Self::return_stub(cpu, 0);
                }
                false
            }
            BleHookKind::TmosStartTask => {
                let task_id = cpu.regs[10] as u8;
                let event = cpu.regs[11] as u16;
                let ticks = cpu.regs[12] as u64;
                let deadline_ns = time_ns.saturating_add(ticks.saturating_mul(625_000));
                if let Some(t) = self
                    .tmos_timers
                    .iter_mut()
                    .find(|t| t.task_id == task_id && t.event == event)
                {
                    t.deadline_ns = deadline_ns;
                    t.period_ns = None;
                } else {
                    self.tmos_timers.push(TmosTimerEntry {
                        task_id,
                        event,
                        deadline_ns,
                        period_ns: None,
                    });
                }
                self.tmos_start_calls += 1;
                false
            }
            BleHookKind::TmosStartReloadTask => {
                let task_id = cpu.regs[10] as u8;
                let event = cpu.regs[11] as u16;
                let ticks = cpu.regs[12] as u64;
                let period_ns = ticks.saturating_mul(625_000).max(625_000);
                let deadline_ns = time_ns.saturating_add(period_ns);
                if let Some(t) = self
                    .tmos_timers
                    .iter_mut()
                    .find(|t| t.task_id == task_id && t.event == event)
                {
                    t.deadline_ns = deadline_ns;
                    t.period_ns = Some(period_ns);
                } else {
                    self.tmos_timers.push(TmosTimerEntry {
                        task_id,
                        event,
                        deadline_ns,
                        period_ns: Some(period_ns),
                    });
                }
                self.tmos_start_calls += 1;
                false
            }
            BleHookKind::TmosStopTask => {
                let task_id = cpu.regs[10] as u8;
                let event = cpu.regs[11] as u16;
                self.tmos_timers
                    .retain(|t| !(t.task_id == task_id && t.event == event));
                false
            }
            BleHookKind::Ch58xLowPower => {
                self.low_power_calls += 1;
                self.push_event(
                    time_ns,
                    cycle,
                    "Ch58xLowPower",
                    format!("CH58x_LowPower(wakeup_rtc={})", cpu.regs[10]),
                );
                false
            }
            BleHookKind::LowPowerSleep => {
                self.sleep_hook_calls += 1;
                false
            }

            // ---- 2.4 GHz RF (`wchrf.h`) Virtual Driver Hooks (stubbed cleanly) ----
            BleHookKind::RfRoleFastInit => {
                let p_conf = cpu.regs[10];
                self.rf.tx_power_dbm = read_u8_mem(flash, sram, p_conf) as i8;
                self.rf.tx_ring_head = read_u32_mem(flash, sram, p_conf.wrapping_add(4));
                self.rf.tx_cur_desc = self.rf.tx_ring_head;
                self.rf.rx_ring_head = read_u32_mem(flash, sram, p_conf.wrapping_add(8));
                self.rf.rx_cur_desc = self.rf.rx_ring_head;
                self.rf.process_cb = read_u32_mem(flash, sram, p_conf.wrapping_add(12));
                self.rf.process_mask = read_u32_mem(flash, sram, p_conf.wrapping_add(16));
                self.rf.initialized = true;
                self.rf.rfip_mode = 1;
                self.rf.log_event(
                    time_ns,
                    format!(
                        "RFRole_FastInit(tx=0x{:08X}, rx=0x{:08X}, cb=0x{:08X})",
                        self.rf.tx_ring_head, self.rf.rx_ring_head, self.rf.process_cb
                    ),
                );
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfRoleBasicInit => {
                let p_conf = cpu.regs[10];
                self.rf.tx_power_dbm = read_u8_mem(flash, sram, p_conf) as i8;
                self.rf.process_cb = read_u32_mem(flash, sram, p_conf.wrapping_add(12));
                self.rf.process_mask = read_u32_mem(flash, sram, p_conf.wrapping_add(16));
                self.rf.mode = RfMode::BasicIdle;
                self.rf.initialized = true;
                self.rf.rfip_mode = 1;
                self.rf.log_event(
                    time_ns,
                    format!("RFRole_BasicInit(cb=0x{:08X})", self.rf.process_cb),
                );
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfRoleSetParam => {
                let p = cpu.regs[10];
                self.rf.access_address = read_u32_mem(flash, sram, p);
                self.rf.crc_init = read_u32_mem(flash, sram, p.wrapping_add(4));
                let freq = read_u32_mem(flash, sram, p.wrapping_add(8));
                if freq != 0 {
                    self.rf.frequency_khz = freq;
                }
                self.rf.properties = read_u32_mem(flash, sram, p.wrapping_add(12));
                self.rf.rx_max_len = read_u16_mem(flash, sram, p.wrapping_add(16));
                self.rf.send_interval = read_u16_mem(flash, sram, p.wrapping_add(18));
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfRoleStop => {
                self.rf.bound_active = false;
                self.rf.bound_connected = false;
                self.rf.peer_chip_id = None;
                self.rf.mode = RfMode::Idle;
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfRoleShut => {
                self.rf.reset_transient();
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfRoleSwitchMode => {
                self.rf.rfip_mode = cpu.regs[10] as u8;
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfRoleClearTxData => {
                self.rf.clear_tx_descriptors(sram);
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfRoleGetStatus => {
                let st = if self.rf.bound_connected {
                    1
                } else if self.rf.bound_active || self.rf.mode == RfMode::BasicRx {
                    2
                } else {
                    0
                };
                Self::return_stub(cpu, st)
            }
            BleHookKind::RfRoleNoopOk => Self::return_stub(cpu, 0),
            BleHookKind::RfBoundSetFreqList => {
                let p = cpu.regs[10];
                let n = (read_u8_mem(flash, sram, p) as usize).min(64);
                let arr_ptr = read_u32_mem(flash, sram, p.wrapping_add(4));
                self.rf.freq_list.clear();
                for i in 0..n {
                    self.rf
                        .freq_list
                        .push(read_u32_mem(flash, sram, arr_ptr.wrapping_add((i * 4) as u32)));
                }
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfBoundSetSpeedType => {
                let p = cpu.regs[10];
                let n = (read_u8_mem(flash, sram, p) as usize).min(8);
                let arr_ptr = read_u32_mem(flash, sram, p.wrapping_add(4));
                self.rf.speed_list.clear();
                for i in 0..n {
                    let item = arr_ptr.wrapping_add((i * 9) as u32);
                    let mut peer = [0u8; 6];
                    let bytes = read_slice_mem(flash, sram, item.wrapping_add(3), 6);
                    peer.copy_from_slice(&bytes);
                    self.rf.speed_list.push(RfRoleListEntry {
                        device_id: read_u8_mem(flash, sram, item),
                        rssi: read_u8_mem(flash, sram, item.wrapping_add(1)) as i8,
                        dev_type: read_u8_mem(flash, sram, item.wrapping_add(2)),
                        peer_info: peer,
                    });
                }
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfBoundStartHost => {
                let p = cpu.regs[10];
                self.rf.hop = read_u8_mem(flash, sram, p);
                self.rf.peri_time = read_u8_mem(flash, sram, p.wrapping_add(2));
                self.rf.dev_type = read_u8_mem(flash, sram, p.wrapping_add(3));
                self.rf.timeout_ms = read_u16_mem(flash, sram, p.wrapping_add(4));
                let own = read_slice_mem(flash, sram, p.wrapping_add(6), 6);
                let peer = read_slice_mem(flash, sram, p.wrapping_add(12), 6);
                self.rf.own_info.copy_from_slice(&own);
                self.rf.peer_info.copy_from_slice(&peer);
                self.rf.bound_cb = read_u32_mem(flash, sram, p.wrapping_add(20));
                self.rf.mode = RfMode::FastHost;
                self.rf.bound_active = true;
                self.rf.bound_connected = false;
                self.rf.peer_chip_id = None;
                self.rf.tx_cur_desc = self.rf.tx_ring_head;
                self.rf.rx_cur_desc = self.rf.rx_ring_head;
                self.rf.bound_started_ns = time_ns;
                self.rf.last_link_ok_ns = time_ns;
                self.rf.log_event(
                    time_ns,
                    format!(
                        "RFBound_StartHost(own={}, timeout={}ms, cb=0x{:08X})",
                        format_mac_colon(&self.rf.own_info),
                        self.rf.timeout_ms,
                        self.rf.bound_cb
                    ),
                );
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfBoundStartDevice => {
                let p = cpu.regs[10];
                self.rf.dev_type = read_u8_mem(flash, sram, p);
                self.rf.dev_id = read_u8_mem(flash, sram, p.wrapping_add(1));
                self.rf.speed_khz = read_u8_mem(flash, sram, p.wrapping_add(3));
                self.rf.timeout_ms = read_u16_mem(flash, sram, p.wrapping_add(4));
                let own = read_slice_mem(flash, sram, p.wrapping_add(6), 6);
                let peer = read_slice_mem(flash, sram, p.wrapping_add(12), 6);
                self.rf.own_info.copy_from_slice(&own);
                self.rf.peer_info.copy_from_slice(&peer);
                self.rf.bound_cb = read_u32_mem(flash, sram, p.wrapping_add(20));
                self.rf.mode = RfMode::FastDevice;
                self.rf.bound_active = true;
                self.rf.bound_connected = false;
                self.rf.peer_chip_id = None;
                self.rf.tx_cur_desc = self.rf.tx_ring_head;
                self.rf.rx_cur_desc = self.rf.rx_ring_head;
                self.rf.bound_started_ns = time_ns;
                self.rf.last_link_ok_ns = time_ns;
                self.rf.log_event(
                    time_ns,
                    format!(
                        "RFBound_StartDevice(id={}, own={}, timeout={}ms, cb=0x{:08X})",
                        self.rf.dev_id,
                        format_mac_colon(&self.rf.own_info),
                        self.rf.timeout_ms,
                        self.rf.bound_cb
                    ),
                );
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfIpSetTxPower => {
                self.rf.tx_power_dbm = cpu.regs[10] as i8;
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfIpReadRssi => Self::return_stub(cpu, self.rf.last_rssi_dbm as i32 as u32),
            BleHookKind::RfIpReadCrc => Self::return_stub(cpu, u32::from(self.rf.last_crc_ok)),
            BleHookKind::RfIpSetTxStart => {
                self.rf.basic_tx_armed = true;
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfIpSetTxParm => {
                let p = cpu.regs[10];
                let aa = read_u32_mem(flash, sram, p);
                let crc_init = read_u32_mem(flash, sram, p.wrapping_add(4));
                let freq = read_u32_mem(flash, sram, p.wrapping_add(8));
                let tx_dma = read_u32_mem(flash, sram, p.wrapping_add(16));
                let pkt_data_len = read_u8_mem(flash, sram, tx_dma.wrapping_add(1)) as usize;
                let total_len = (2 + pkt_data_len).min(256);
                let payload = read_slice_mem(flash, sram, tx_dma, total_len);
                self.rf.basic_tx_armed = false;
                self.rf.tx_count += 1;
                self.rf.outbox.push_back(RfOutgoingFrame {
                    kind: RfPacketKind::BasicRaw,
                    frequency_khz: if freq != 0 { freq } else { self.rf.frequency_khz },
                    access_address: if aa != 0 { aa } else { self.rf.access_address },
                    crc_init,
                    dev_id: 0,
                    tx_power_dbm: self.rf.tx_power_dbm,
                    payload,
                    sent_ns: time_ns,
                });
                let mut q = std::mem::take(&mut self.pending_callbacks);
                self.rf.queue_process_cb(crate::rf::RF_STATE_TX_FINISH, 0, &mut q);
                self.pending_callbacks = q;
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfIpSetRx => {
                let p = cpu.regs[10];
                let aa = read_u32_mem(flash, sram, p);
                let crc_init = read_u32_mem(flash, sram, p.wrapping_add(4));
                let freq = read_u32_mem(flash, sram, p.wrapping_add(8));
                let props = read_u32_mem(flash, sram, p.wrapping_add(12));
                let rx_dma = read_u32_mem(flash, sram, p.wrapping_add(16));
                let max_len = read_u16_mem(flash, sram, p.wrapping_add(22));
                let timeout_half_us = read_u16_mem(flash, sram, p.wrapping_add(24)) as u64;
                if aa != 0 {
                    self.rf.access_address = aa;
                }
                self.rf.crc_init = crc_init;
                self.rf.frequency_khz = freq;
                self.rf.properties = props;
                self.rf.basic_rx_dma = rx_dma;
                self.rf.basic_rx_max_len = max_len;
                self.rf.basic_rx_timeout_us = timeout_half_us / 2;
                self.rf.basic_rx_start_ns = time_ns;
                self.rf.mode = RfMode::BasicRx;
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfTx => {
                let ptr = cpu.regs[10];
                let len = (cpu.regs[11] as usize).min(256);
                let payload = read_slice_mem(flash, sram, ptr, len);
                self.rf.tx_count += 1;
                self.rf.outbox.push_back(RfOutgoingFrame {
                    kind: RfPacketKind::LegacyRaw,
                    frequency_khz: self.rf.frequency_khz,
                    access_address: self.rf.access_address,
                    crc_init: self.rf.crc_init,
                    dev_id: 0,
                    tx_power_dbm: self.rf.tx_power_dbm,
                    payload,
                    sent_ns: time_ns,
                });
                self.push_event(time_ns, cycle, "RfTx", format!("RF_Tx({len}B)"));
                Self::return_stub(cpu, 0)
            }
            BleHookKind::RfRx => {
                let ptr = cpu.regs[10];
                let len = (cpu.regs[11] as u16).min(256);
                self.rf.basic_rx_dma = ptr;
                self.rf.basic_rx_max_len = len;
                self.rf.basic_rx_timeout_us = 0;
                self.rf.basic_rx_start_ns = time_ns;
                self.rf.mode = RfMode::BasicRx;
                if ptr != 0 {
                    write_slice_sram(sram, ptr, &[]);
                }
                self.push_event(time_ns, cycle, "RfRx", format!("RF_Rx(buf=0x{ptr:08X}, {len}B)"));
                Self::return_stub(cpu, 0)
            }
        }
    }

    pub fn read_u8(&self, _offset: u32) -> u8 {
        // BLE/2.4G registers are bridged at wchrf/BLE API level; returning 0 prevents stalls on closed-source baseband bits.
        0
    }

    pub fn write_u8(&mut self, _offset: u32, _val: u8) {}
}
