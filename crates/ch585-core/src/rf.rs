use std::collections::VecDeque;
use std::fmt::Write as _;

use crate::cpu::PendingCallback;
use crate::soc::Ch585Machine;

pub const STA_DMA_ENABLE: u32 = 0x8000_0000;
pub const STA_LEN_MASK: u32 = 0x0FFF;

pub const RF_STATE_RX: u32 = 1 << 0;
pub const RF_STATE_RBU: u32 = 1 << 1;
pub const RF_STATE_TIMEOUT: u32 = 1 << 2;
pub const RF_STATE_RX_CRCERR: u32 = 1 << 3;
pub const RF_STATE_TX_FINISH: u32 = 1 << 4;
pub const RF_STATE_TX_IDLE: u32 = 1 << 5;
pub const RF_STATE_RX_RETRY: u32 = 1 << 6;
pub const RF_STATE_MAP_UPDATE: u32 = 1 << 7;

pub const BOUND_STA_SUCCESS: u8 = 0x00;
pub const BOUND_STA_FAILURE: u8 = 0x01;
pub const BOUND_STA_INVALIDPARM: u8 = 0x02;
pub const BOUND_STA_TIMEOUT: u8 = 0x17;

pub const RF_BOUND_ID: u8 = 7;
pub const RF_ROLE_HOST: u8 = 0;
pub const RF_ROLE_DEVICE: u8 = 1;

pub(crate) fn read_u8_mem(flash: &[u8], sram: &[u8], addr: u32) -> u8 {
    let f = addr as usize;
    if f < flash.len() {
        flash[f]
    } else if (0x2000_0000..0x2002_0000).contains(&addr) {
        let s = (addr - 0x2000_0000) as usize;
        if s < sram.len() {
            sram[s]
        } else {
            0
        }
    } else {
        0
    }
}

pub(crate) fn read_u16_mem(flash: &[u8], sram: &[u8], addr: u32) -> u16 {
    let b0 = read_u8_mem(flash, sram, addr) as u16;
    let b1 = read_u8_mem(flash, sram, addr.wrapping_add(1)) as u16;
    b0 | (b1 << 8)
}

pub(crate) fn read_u32_mem(flash: &[u8], sram: &[u8], addr: u32) -> u32 {
    let b0 = read_u8_mem(flash, sram, addr) as u32;
    let b1 = read_u8_mem(flash, sram, addr.wrapping_add(1)) as u32;
    let b2 = read_u8_mem(flash, sram, addr.wrapping_add(2)) as u32;
    let b3 = read_u8_mem(flash, sram, addr.wrapping_add(3)) as u32;
    b0 | (b1 << 8) | (b2 << 16) | (b3 << 24)
}

pub(crate) fn read_slice_mem(flash: &[u8], sram: &[u8], addr: u32, len: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(len);
    for i in 0..len {
        out.push(read_u8_mem(flash, sram, addr.wrapping_add(i as u32)));
    }
    out
}

pub(crate) fn read_u32_sram(sram: &[u8], addr: u32) -> u32 {
    if (0x2000_0000..0x2002_0000).contains(&addr) {
        let s = (addr - 0x2000_0000) as usize;
        if s + 4 <= sram.len() {
            return u32::from_le_bytes([sram[s], sram[s + 1], sram[s + 2], sram[s + 3]]);
        }
    }
    0
}

pub(crate) fn write_u8_sram(sram: &mut [u8], addr: u32, val: u8) -> bool {
    if (0x2000_0000..0x2002_0000).contains(&addr) {
        let s = (addr - 0x2000_0000) as usize;
        if s < sram.len() {
            sram[s] = val;
            return true;
        }
    }
    false
}

pub(crate) fn write_u32_sram(sram: &mut [u8], addr: u32, val: u32) -> bool {
    if (0x2000_0000..0x2002_0000).contains(&addr) {
        let s = (addr - 0x2000_0000) as usize;
        if s + 4 <= sram.len() {
            sram[s..s + 4].copy_from_slice(&val.to_le_bytes());
            return true;
        }
    }
    false
}

pub(crate) fn write_slice_sram(sram: &mut [u8], addr: u32, data: &[u8]) -> usize {
    if !(0x2000_0000..0x2002_0000).contains(&addr) {
        return 0;
    }
    let s = (addr - 0x2000_0000) as usize;
    if s >= sram.len() {
        return 0;
    }
    let n = data.len().min(sram.len() - s);
    sram[s..s + n].copy_from_slice(&data[..n]);
    n
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RfMode {
    Idle,
    BasicIdle,
    BasicRx,
    FastHost,
    FastDevice,
}

impl RfMode {
    pub fn as_str(&self) -> &'static str {
        match self {
            Self::Idle => "Idle",
            Self::BasicIdle => "BasicIdle",
            Self::BasicRx => "BasicRx",
            Self::FastHost => "FastHost",
            Self::FastDevice => "FastDevice",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RfPacketKind {
    FastData,
    BasicRaw,
    LegacyRaw,
}

#[derive(Debug, Clone)]
pub struct RfOutgoingFrame {
    pub kind: RfPacketKind,
    pub frequency_khz: u32,
    pub access_address: u32,
    pub crc_init: u32,
    pub dev_id: u8,
    pub tx_power_dbm: i8,
    pub payload: Vec<u8>,
    pub sent_ns: u64,
}

#[derive(Debug, Clone)]
pub struct RfRoleListEntry {
    pub device_id: u8,
    pub rssi: i8,
    pub dev_type: u8,
    pub peer_info: [u8; 6],
}

#[derive(Debug, Clone)]
pub struct RfEventLogEntry {
    pub time_ns: u64,
    pub summary: String,
}

/// Per-chip virtual 2.4 GHz RF controller state (bridged from `wchrf.h` Fast/Bound & Basic APIs).
#[derive(Debug, Clone)]
pub struct RfState {
    pub mode: RfMode,
    pub rfip_mode: u8,
    pub initialized: bool,
    pub tx_power_dbm: i8,
    pub tx_ring_head: u32,
    pub tx_cur_desc: u32,
    pub rx_ring_head: u32,
    pub rx_cur_desc: u32,
    pub process_cb: u32,
    pub process_mask: u32,

    pub access_address: u32,
    pub crc_init: u32,
    pub frequency_khz: u32,
    pub properties: u32,
    pub rx_max_len: u16,
    pub send_interval: u16,
    pub freq_list: Vec<u32>,
    pub speed_list: Vec<RfRoleListEntry>,

    pub bound_active: bool,
    pub bound_connected: bool,
    pub bound_cb: u32,
    pub dev_id: u8,
    pub dev_type: u8,
    pub peri_time: u8,
    pub hop: u8,
    pub speed_khz: u8,
    pub timeout_ms: u16,
    pub own_info: [u8; 6],
    pub peer_info: [u8; 6],
    pub peer_chip_id: Option<usize>,
    pub bound_started_ns: u64,
    pub last_rx_ns: u64,
    pub last_link_ok_ns: u64,

    pub basic_rx_dma: u32,
    pub basic_rx_max_len: u16,
    pub basic_rx_timeout_us: u64,
    pub basic_rx_start_ns: u64,
    pub basic_tx_armed: bool,

    pub last_rssi_dbm: i8,
    pub last_crc_ok: bool,
    pub tx_count: u64,
    pub rx_count: u64,
    pub rx_crc_err_count: u64,
    pub rx_rbu_count: u64,
    pub timeout_count: u64,
    pub bound_success_count: u32,
    pub bound_timeout_count: u32,

    pub outbox: VecDeque<RfOutgoingFrame>,
    pub event_log: Vec<RfEventLogEntry>,
    cb_seq: u32,
}

impl Default for RfState {
    fn default() -> Self {
        Self::new()
    }
}

impl RfState {
    pub fn new() -> Self {
        Self {
            mode: RfMode::Idle,
            rfip_mode: 0,
            initialized: false,
            tx_power_dbm: 0,
            tx_ring_head: 0,
            tx_cur_desc: 0,
            rx_ring_head: 0,
            rx_cur_desc: 0,
            process_cb: 0,
            process_mask: 0,
            access_address: 0x7176_2345,
            crc_init: 0x0055_5555,
            frequency_khz: 2_402_000,
            properties: 0,
            rx_max_len: 251,
            send_interval: 0,
            freq_list: Vec::new(),
            speed_list: Vec::new(),
            bound_active: false,
            bound_connected: false,
            bound_cb: 0,
            dev_id: RF_BOUND_ID,
            dev_type: 0,
            peri_time: 8,
            hop: 0,
            speed_khz: 8,
            timeout_ms: 100,
            own_info: [0; 6],
            peer_info: [0; 6],
            peer_chip_id: None,
            bound_started_ns: 0,
            last_rx_ns: 0,
            last_link_ok_ns: 0,
            basic_rx_dma: 0,
            basic_rx_max_len: 251,
            basic_rx_timeout_us: 0,
            basic_rx_start_ns: 0,
            basic_tx_armed: false,
            last_rssi_dbm: -45,
            last_crc_ok: true,
            tx_count: 0,
            rx_count: 0,
            rx_crc_err_count: 0,
            rx_rbu_count: 0,
            timeout_count: 0,
            bound_success_count: 0,
            bound_timeout_count: 0,
            outbox: VecDeque::new(),
            event_log: Vec::new(),
            cb_seq: 1,
        }
    }

    pub fn reset_transient(&mut self) {
        self.mode = RfMode::Idle;
        self.initialized = false;
        self.bound_active = false;
        self.bound_connected = false;
        self.peer_chip_id = None;
        self.basic_rx_dma = 0;
        self.basic_tx_armed = false;
        self.outbox.clear();
    }

    #[inline]
    pub fn is_active(&self) -> bool {
        self.mode != RfMode::Idle && (self.bound_active || self.mode == RfMode::BasicRx || self.initialized)
    }

    pub fn log_event(&mut self, time_ns: u64, summary: String) {
        if self.event_log.len() >= 256 {
            self.event_log.drain(0..64);
        }
        self.event_log.push(RfEventLogEntry { time_ns, summary });
    }

    pub fn next_event_ns(&self) -> Option<u64> {
        if self.mode == RfMode::BasicRx && self.basic_rx_timeout_us > 0 {
            Some(
                self.basic_rx_start_ns
                    .saturating_add(self.basic_rx_timeout_us.saturating_mul(1_000)),
            )
        } else {
            None
        }
    }

    fn next_tag(&mut self, prefix: u32) -> u32 {
        let s = self.cb_seq;
        self.cb_seq = self.cb_seq.wrapping_add(1);
        prefix | (s & 0x00FF_FFFF)
    }

    pub(crate) fn queue_process_cb(
        &mut self,
        status: u32,
        id: u8,
        callbacks: &mut VecDeque<PendingCallback>,
    ) {
        if self.process_cb == 0 {
            return;
        }
        if self.process_mask != 0 && (self.process_mask & status) == 0 {
            return;
        }
        let tag = self.next_tag(0x5200_0000);
        let mut args = [0u32; 8];
        args[0] = status;
        args[1] = id as u32;
        callbacks.push_back(PendingCallback {
            entry_pc: self.process_cb,
            args,
            stack_blob: Vec::new(),
            blob_arg_patches: Vec::new(),
            read_back: None,
            tag,
            wake_from_wfi: true,
        });
    }

    pub(crate) fn queue_bound_cb(
        &mut self,
        status: u8,
        role: u8,
        dev_id: u8,
        peer_info: [u8; 6],
        callbacks: &mut VecDeque<PendingCallback>,
    ) {
        if self.bound_cb == 0 {
            return;
        }
        let mut blob = vec![0u8; 12];
        blob[0] = status;
        blob[1] = role;
        blob[2] = dev_id;
        blob[3] = self.dev_type;
        blob[4] = self.peri_time;
        blob[5] = self.hop;
        blob[6..12].copy_from_slice(&peer_info);
        let tag = self.next_tag(0x5300_0000);
        callbacks.push_back(PendingCallback {
            entry_pc: self.bound_cb,
            args: [0; 8],
            stack_blob: blob,
            blob_arg_patches: vec![(0, 0)],
            read_back: None,
            tag,
            wake_from_wfi: true,
        });
    }

    pub fn clear_tx_descriptors(&mut self, sram: &mut [u8]) {
        self.outbox.clear();
        if self.tx_ring_head == 0 {
            return;
        }
        let mut cur = self.tx_ring_head;
        for _ in 0..16 {
            if !(0x2000_0000..0x2002_0000).contains(&cur) {
                break;
            }
            let st = read_u32_sram(sram, cur);
            write_u32_sram(sram, cur, st & !STA_DMA_ENABLE);
            let next = read_u32_sram(sram, cur.wrapping_add(12));
            if next == 0 || next == self.tx_ring_head {
                break;
            }
            cur = next;
        }
        self.tx_cur_desc = self.tx_ring_head;
    }

    /// Poll guest TX DMA descriptor ring (`RF_DMADESCTypeDef`) for queued outgoing frames.
    pub fn poll_tx_dma(
        &mut self,
        sram: &mut [u8],
        now_ns: u64,
        callbacks: &mut VecDeque<PendingCallback>,
    ) {
        if !matches!(self.mode, RfMode::FastHost | RfMode::FastDevice) || !self.bound_connected {
            return;
        }
        let start = if self.tx_cur_desc != 0 {
            self.tx_cur_desc
        } else {
            self.tx_ring_head
        };
        if !(0x2000_0000..0x2002_0000).contains(&start) {
            return;
        }

        // Probe starting at current cursor; if empty, scan ahead in the ring so index alignment never stalls.
        let mut cur = start;
        let mut found_start = None;
        for _ in 0..8 {
            if !(0x2000_0000..0x2002_0000).contains(&cur) {
                break;
            }
            let status = read_u32_sram(sram, cur);
            if (status & STA_DMA_ENABLE) != 0 {
                found_start = Some(cur);
                break;
            }
            let next = read_u32_sram(sram, cur.wrapping_add(12));
            cur = if next != 0 { next } else { self.tx_ring_head };
            if cur == start {
                break;
            }
        }
        let Some(mut desc) = found_start else {
            return;
        };

        for _ in 0..8 {
            if !(0x2000_0000..0x2002_0000).contains(&desc) {
                break;
            }
            let status = read_u32_sram(sram, desc);
            if (status & STA_DMA_ENABLE) == 0 {
                break;
            }
            let buf_size = read_u32_sram(sram, desc.wrapping_add(4)) as usize;
            let buf_addr = read_u32_sram(sram, desc.wrapping_add(8));
            let next = read_u32_sram(sram, desc.wrapping_add(12));
            let pkt_len = ((status & STA_LEN_MASK) as usize).min(buf_size.max(264)).min(512);
            let payload = read_slice_mem(&[], sram, buf_addr, pkt_len);
            write_u32_sram(sram, desc, status & !STA_DMA_ENABLE);

            self.tx_count += 1;
            self.outbox.push_back(RfOutgoingFrame {
                kind: RfPacketKind::FastData,
                frequency_khz: self.current_frequency_khz(),
                access_address: self.access_address,
                crc_init: self.crc_init,
                dev_id: self.dev_id,
                tx_power_dbm: self.tx_power_dbm,
                payload,
                sent_ns: now_ns,
            });
            let id = self.dev_id;
            self.queue_process_cb(RF_STATE_TX_FINISH, id, callbacks);

            let next_desc = if next != 0 { next } else { self.tx_ring_head };
            self.tx_cur_desc = next_desc;
            desc = next_desc;
            if desc == start && self.tx_ring_head != 0 {
                // Only one lap per poll
                let next_st = read_u32_sram(sram, desc);
                if (next_st & STA_DMA_ENABLE) == 0 {
                    break;
                }
            }
        }
    }

    pub fn current_frequency_khz(&self) -> u32 {
        if self.frequency_khz != 0 {
            self.frequency_khz
        } else if let Some(&f) = self.freq_list.first() {
            f
        } else {
            2_402_000
        }
    }

    /// Deliver an incoming RF air-frame into the chip's RX DMA ring or Basic RX buffer.
    pub fn deliver_rx_frame(
        &mut self,
        sram: &mut [u8],
        payload: &[u8],
        dev_id: u8,
        rssi_dbm: i8,
        crc_err: bool,
        now_ns: u64,
        callbacks: &mut VecDeque<PendingCallback>,
    ) -> bool {
        self.last_rssi_dbm = rssi_dbm;
        self.last_crc_ok = !crc_err;
        self.last_rx_ns = now_ns;
        self.last_link_ok_ns = now_ns;

        match self.mode {
            RfMode::FastHost | RfMode::FastDevice => {
                let desc = if self.rx_cur_desc != 0 {
                    self.rx_cur_desc
                } else {
                    self.rx_ring_head
                };
                if !(0x2000_0000..0x2002_0000).contains(&desc) {
                    self.rx_rbu_count += 1;
                    self.queue_process_cb(RF_STATE_RBU, dev_id, callbacks);
                    return false;
                }
                let status = read_u32_sram(sram, desc);
                if (status & STA_DMA_ENABLE) == 0 {
                    self.rx_rbu_count += 1;
                    self.queue_process_cb(RF_STATE_RBU, dev_id, callbacks);
                    return false;
                }
                let buf_size = read_u32_sram(sram, desc.wrapping_add(4)) as usize;
                let buf_addr = read_u32_sram(sram, desc.wrapping_add(8));
                let next = read_u32_sram(sram, desc.wrapping_add(12));
                let cap = buf_size.max(1).min(512);
                let copy_len = payload.len().min(cap);
                write_slice_sram(sram, buf_addr, &payload[..copy_len]);
                // WCH Fast RF sets Status length without STA_DMA_ENABLE once RX completes
                write_u32_sram(sram, desc, (copy_len as u32) & STA_LEN_MASK);
                self.rx_cur_desc = if next != 0 { next } else { self.rx_ring_head };

                if crc_err {
                    self.rx_crc_err_count += 1;
                    self.queue_process_cb(RF_STATE_RX_CRCERR, dev_id, callbacks);
                } else {
                    self.rx_count += 1;
                    self.queue_process_cb(RF_STATE_RX, dev_id, callbacks);
                }
                true
            }
            RfMode::BasicRx => {
                let cap = (self.basic_rx_max_len as usize).saturating_add(4).clamp(4, 260);
                let copy_len = payload.len().min(cap);
                if self.basic_rx_dma != 0 {
                    write_slice_sram(sram, self.basic_rx_dma, &payload[..copy_len]);
                }
                self.mode = RfMode::BasicIdle;
                if crc_err {
                    self.rx_crc_err_count += 1;
                    self.queue_process_cb(RF_STATE_RX_CRCERR, 0, callbacks);
                } else {
                    self.rx_count += 1;
                    self.queue_process_cb(RF_STATE_RX, 0, callbacks);
                }
                true
            }
            _ => false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RfMediumConfig {
    pub packet_loss_pct: f32,
    pub delay_us: u64,
    pub jitter_us: u64,
    pub base_rssi_dbm: i8,
    pub channel_interference_dbm: [i8; 40],
    pub seed: u64,
}

impl Default for RfMediumConfig {
    fn default() -> Self {
        Self {
            packet_loss_pct: 0.0,
            delay_us: 25,
            jitter_us: 0,
            base_rssi_dbm: -45,
            channel_interference_dbm: [0; 40],
            seed: 0x5852_4632_4700_0001,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RfDeliveryOutcome {
    Delivered,
    DroppedPacketLoss,
    DroppedLinkDown,
    CrcError,
}

#[derive(Debug, Clone)]
pub struct RfMediumLogEntry {
    pub time_ns: u64,
    pub src_chip: usize,
    pub dst_chip: Option<usize>,
    pub freq_khz: u32,
    pub len: usize,
    pub outcome: RfDeliveryOutcome,
}

#[derive(Debug, Clone)]
struct InFlightFrame {
    deliver_at_ns: u64,
    src_chip: usize,
    dst_chip: Option<usize>,
    frame: RfOutgoingFrame,
}

/// Shared 2.4 GHz virtual ether connecting multiple CH585 chips.
#[derive(Debug, Clone)]
pub struct RfMedium {
    pub config: RfMediumConfig,
    pub link_dropped: bool,
    pub drop_link_at_ns: Option<u64>,
    pub restore_link_at_ns: Option<u64>,
    in_flight: Vec<InFlightFrame>,
    pub total_sent: u64,
    pub total_delivered: u64,
    pub total_dropped_loss: u64,
    pub total_dropped_link: u64,
    pub total_crc_errors: u64,
    pub packet_log: Vec<RfMediumLogEntry>,
    prng: u64,
}

impl Default for RfMedium {
    fn default() -> Self {
        Self::new(RfMediumConfig::default())
    }
}

impl RfMedium {
    pub fn new(config: RfMediumConfig) -> Self {
        let seed = if config.seed == 0 { 1 } else { config.seed };
        Self {
            config,
            link_dropped: false,
            drop_link_at_ns: None,
            restore_link_at_ns: None,
            in_flight: Vec::new(),
            total_sent: 0,
            total_delivered: 0,
            total_dropped_loss: 0,
            total_dropped_link: 0,
            total_crc_errors: 0,
            packet_log: Vec::new(),
            prng: seed,
        }
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.prng;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.prng = x;
        x.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }

    fn next_unit_f32(&mut self) -> f32 {
        ((self.next_u64() >> 40) as f32) / 16_777_216.0
    }

    fn channel_index(freq_khz: u32) -> usize {
        if freq_khz >= 2_400_000 {
            (((freq_khz - 2_400_000) / 2_000) as usize).min(39)
        } else {
            (freq_khz as usize).min(39)
        }
    }

    fn push_log(&mut self, entry: RfMediumLogEntry) {
        if self.packet_log.len() >= 512 {
            self.packet_log.drain(0..128);
        }
        self.packet_log.push(entry);
    }

    fn speed_list_allows(host: &RfState, dev: &RfState) -> bool {
        if host.speed_list.is_empty() {
            return true;
        }
        host.speed_list.iter().any(|entry| {
            let id_ok = entry.device_id == dev.dev_id
                || (dev.dev_id == RF_BOUND_ID && entry.device_id == RF_BOUND_ID);
            let type_ok = entry.dev_type == 0 || entry.dev_type == dev.dev_type;
            let info_ok = entry.peer_info == [0; 6] || entry.peer_info == dev.own_info;
            id_ok && type_ok && info_ok
        })
    }

    fn bound_pair_compatible(host: &RfState, dev: &RfState) -> bool {
        if !Self::speed_list_allows(host, dev) {
            return false;
        }
        if dev.dev_id == RF_BOUND_ID {
            return true;
        }
        dev.peer_info == host.own_info
            || host.peer_info == dev.own_info
            || dev.peer_info == [0; 6]
    }

    /// Advance 2.4 GHz channel pairing, TX DMA extraction, delayed delivery, faults & timeouts.
    pub fn step(&mut self, now_ns: u64, chips: &mut [Ch585Machine]) {
        if let Some(t) = self.drop_link_at_ns {
            if now_ns >= t {
                self.link_dropped = true;
                self.drop_link_at_ns = None;
            }
        }
        if let Some(t) = self.restore_link_at_ns {
            if now_ns >= t {
                self.link_dropped = false;
                self.restore_link_at_ns = None;
            }
        }

        // 1. Pair / reconnect Fast Host <-> Fast Device peers when medium is up
        if !self.link_dropped {
            let n = chips.len();
            for hi in 0..n {
                if !chips[hi].bus.powered {
                    continue;
                }
                let h_ready = {
                    let rf = &chips[hi].bus.ble.rf;
                    rf.mode == RfMode::FastHost && rf.bound_active && !rf.bound_connected
                };
                if !h_ready {
                    continue;
                }
                for di in 0..n {
                    if hi == di || !chips[di].bus.powered {
                        continue;
                    }
                    let matched = {
                        let h = &chips[hi].bus.ble.rf;
                        let d = &chips[di].bus.ble.rf;
                        d.mode == RfMode::FastDevice
                            && d.bound_active
                            && !d.bound_connected
                            && Self::bound_pair_compatible(h, d)
                    };
                    if !matched {
                        continue;
                    }
                    let assigned_id = if chips[di].bus.ble.rf.dev_id == RF_BOUND_ID {
                        0
                    } else {
                        chips[di].bus.ble.rf.dev_id
                    };
                    let host_info = chips[hi].bus.ble.rf.own_info;
                    let dev_info = chips[di].bus.ble.rf.own_info;

                    {
                        let hb = &mut chips[hi].bus.ble;
                        hb.rf.bound_connected = true;
                        hb.rf.peer_chip_id = Some(di);
                        hb.rf.dev_id = assigned_id;
                        hb.rf.peer_info = dev_info;
                        hb.rf.last_link_ok_ns = now_ns;
                        hb.rf.bound_success_count += 1;
                        hb.rf.log_event(
                            now_ns,
                            format!("RFBound Host paired with chip#{di} (devId={assigned_id})"),
                        );
                        hb.rf.queue_bound_cb(
                            BOUND_STA_SUCCESS,
                            RF_ROLE_HOST,
                            assigned_id,
                            dev_info,
                            &mut hb.pending_callbacks,
                        );
                    }
                    {
                        let db = &mut chips[di].bus.ble;
                        db.rf.bound_connected = true;
                        db.rf.peer_chip_id = Some(hi);
                        db.rf.dev_id = assigned_id;
                        db.rf.peer_info = host_info;
                        db.rf.last_link_ok_ns = now_ns;
                        db.rf.bound_success_count += 1;
                        db.rf.log_event(
                            now_ns,
                            format!("RFBound Device paired with chip#{hi} (devId={assigned_id})"),
                        );
                        db.rf.queue_bound_cb(
                            BOUND_STA_SUCCESS,
                            RF_ROLE_DEVICE,
                            assigned_id,
                            host_info,
                            &mut db.pending_callbacks,
                        );
                    }
                    break;
                }
            }
        }

        // 2. Poll TX DMA rings and enqueue outgoing frames into air medium
        for (src_idx, chip) in chips.iter_mut().enumerate() {
            if !chip.bus.powered {
                continue;
            }
            let ble = &mut chip.bus.ble;
            ble.rf.poll_tx_dma(&mut chip.bus.sram, now_ns, &mut ble.pending_callbacks);
            while let Some(frame) = ble.rf.outbox.pop_front() {
                self.total_sent += 1;
                if self.link_dropped {
                    self.total_dropped_link += 1;
                    self.push_log(RfMediumLogEntry {
                        time_ns: now_ns,
                        src_chip: src_idx,
                        dst_chip: ble.rf.peer_chip_id,
                        freq_khz: frame.frequency_khz,
                        len: frame.payload.len(),
                        outcome: RfDeliveryOutcome::DroppedLinkDown,
                    });
                    continue;
                }
                let loss_p = self.config.packet_loss_pct.clamp(0.0, 100.0) / 100.0;
                if loss_p > 0.0 && self.next_unit_f32() < loss_p {
                    self.total_dropped_loss += 1;
                    self.push_log(RfMediumLogEntry {
                        time_ns: now_ns,
                        src_chip: src_idx,
                        dst_chip: ble.rf.peer_chip_id,
                        freq_khz: frame.frequency_khz,
                        len: frame.payload.len(),
                        outcome: RfDeliveryOutcome::DroppedPacketLoss,
                    });
                    continue;
                }
                let jitter = if self.config.jitter_us > 0 {
                    self.next_u64() % (self.config.jitter_us + 1)
                } else {
                    0
                };
                let deliver_at_ns = frame
                    .sent_ns
                    .saturating_add((self.config.delay_us + jitter).saturating_mul(1_000));
                let dst_chip = match frame.kind {
                    RfPacketKind::FastData => ble.rf.peer_chip_id,
                    RfPacketKind::BasicRaw | RfPacketKind::LegacyRaw => None,
                };
                self.in_flight.push(InFlightFrame {
                    deliver_at_ns,
                    src_chip: src_idx,
                    dst_chip,
                    frame,
                });
            }
        }

        // 3. Deliver due in-flight frames
        let mut remaining = Vec::with_capacity(self.in_flight.len());
        let due_queue = std::mem::take(&mut self.in_flight);
        for item in due_queue {
            if item.deliver_at_ns > now_ns {
                remaining.push(item);
                continue;
            }
            if self.link_dropped {
                self.total_dropped_link += 1;
                self.push_log(RfMediumLogEntry {
                    time_ns: now_ns,
                    src_chip: item.src_chip,
                    dst_chip: item.dst_chip,
                    freq_khz: item.frame.frequency_khz,
                    len: item.frame.payload.len(),
                    outcome: RfDeliveryOutcome::DroppedLinkDown,
                });
                continue;
            }

            let ch = Self::channel_index(item.frame.frequency_khz);
            let interf = self.config.channel_interference_dbm[ch].max(0);
            let rssi_dbm = (self.config.base_rssi_dbm as i32 - (interf as i32 / 2))
                .clamp(-110, 10) as i8;
            let crc_err = if interf >= 60 {
                true
            } else if interf > 20 {
                let err_p = ((interf - 20) as f32) / 40.0;
                self.next_unit_f32() < err_p
            } else {
                false
            };

            match item.frame.kind {
                RfPacketKind::FastData => {
                    if let Some(dst_idx) = item.dst_chip {
                        if let Some(dst) = chips.get_mut(dst_idx) {
                            if dst.bus.powered && dst.bus.ble.rf.bound_connected {
                                let ble = &mut dst.bus.ble;
                                let ok = ble.rf.deliver_rx_frame(
                                    &mut dst.bus.sram,
                                    &item.frame.payload,
                                    item.frame.dev_id,
                                    rssi_dbm,
                                    crc_err,
                                    now_ns,
                                    &mut ble.pending_callbacks,
                                );
                                if ok {
                                    if crc_err {
                                        self.total_crc_errors += 1;
                                    } else {
                                        self.total_delivered += 1;
                                    }
                                    self.push_log(RfMediumLogEntry {
                                        time_ns: now_ns,
                                        src_chip: item.src_chip,
                                        dst_chip: Some(dst_idx),
                                        freq_khz: item.frame.frequency_khz,
                                        len: item.frame.payload.len(),
                                        outcome: if crc_err {
                                            RfDeliveryOutcome::CrcError
                                        } else {
                                            RfDeliveryOutcome::Delivered
                                        },
                                    });
                                }
                            }
                        }
                    }
                }
                RfPacketKind::BasicRaw | RfPacketKind::LegacyRaw => {
                    for (dst_idx, dst) in chips.iter_mut().enumerate() {
                        if dst_idx == item.src_chip || !dst.bus.powered {
                            continue;
                        }
                        if dst.bus.ble.rf.mode != RfMode::BasicRx {
                            continue;
                        }
                        let freq_match = dst.bus.ble.rf.frequency_khz == 0
                            || item.frame.frequency_khz == 0
                            || dst.bus.ble.rf.frequency_khz == item.frame.frequency_khz;
                        let aa_match = dst.bus.ble.rf.access_address == 0
                            || item.frame.access_address == 0
                            || dst.bus.ble.rf.access_address == item.frame.access_address;
                        if !freq_match || !aa_match {
                            continue;
                        }
                        let ble = &mut dst.bus.ble;
                        if ble.rf.deliver_rx_frame(
                            &mut dst.bus.sram,
                            &item.frame.payload,
                            0,
                            rssi_dbm,
                            crc_err,
                            now_ns,
                            &mut ble.pending_callbacks,
                        ) {
                            if crc_err {
                                self.total_crc_errors += 1;
                            } else {
                                self.total_delivered += 1;
                            }
                            self.push_log(RfMediumLogEntry {
                                time_ns: now_ns,
                                src_chip: item.src_chip,
                                dst_chip: Some(dst_idx),
                                freq_khz: item.frame.frequency_khz,
                                len: item.frame.payload.len(),
                                outcome: if crc_err {
                                    RfDeliveryOutcome::CrcError
                                } else {
                                    RfDeliveryOutcome::Delivered
                                },
                            });
                        }
                    }
                }
            }
        }
        self.in_flight = remaining;

        // 4. Refresh keepalives and check Fast/Bound & BasicRx timeouts
        let mut peers_alive = vec![false; chips.len()];
        if !self.link_dropped {
            for (idx, chip) in chips.iter().enumerate() {
                if !chip.bus.powered || !chip.bus.ble.rf.bound_connected {
                    continue;
                }
                if let Some(p) = chip.bus.ble.rf.peer_chip_id {
                    if let Some(peer) = chips.get(p) {
                        peers_alive[idx] = peer.bus.powered && peer.bus.ble.rf.bound_connected;
                    }
                }
            }
        }

        for (idx, chip) in chips.iter_mut().enumerate() {
            if !chip.bus.powered {
                continue;
            }
            let ble = &mut chip.bus.ble;
            if ble.rf.bound_connected {
                if peers_alive[idx] {
                    ble.rf.last_link_ok_ns = now_ns;
                } else {
                    let to_ms = ble.rf.timeout_ms.max(1) as u64;
                    if now_ns.saturating_sub(ble.rf.last_link_ok_ns) >= to_ms * 1_000_000 {
                        let role = if ble.rf.mode == RfMode::FastHost {
                            RF_ROLE_HOST
                        } else {
                            RF_ROLE_DEVICE
                        };
                        let id = ble.rf.dev_id;
                        let peer_info = ble.rf.peer_info;
                        ble.rf.bound_connected = false;
                        ble.rf.peer_chip_id = None;
                        ble.rf.timeout_count += 1;
                        ble.rf.bound_timeout_count += 1;
                        ble.rf.log_event(
                            now_ns,
                            format!("RFBound timeout after {} ms", to_ms),
                        );
                        ble.rf.queue_process_cb(RF_STATE_TIMEOUT, id, &mut ble.pending_callbacks);
                        ble.rf.queue_bound_cb(
                            BOUND_STA_TIMEOUT,
                            role,
                            id,
                            peer_info,
                            &mut ble.pending_callbacks,
                        );
                    }
                }
            } else if ble.rf.mode == RfMode::BasicRx && ble.rf.basic_rx_timeout_us > 0 {
                if now_ns.saturating_sub(ble.rf.basic_rx_start_ns)
                    >= ble.rf.basic_rx_timeout_us.saturating_mul(1_000)
                {
                    ble.rf.mode = RfMode::BasicIdle;
                    ble.rf.timeout_count += 1;
                    ble.rf.queue_process_cb(RF_STATE_TIMEOUT, 0, &mut ble.pending_callbacks);
                }
            }
        }
    }
}

pub fn format_mac_colon(addr: &[u8; 6]) -> String {
    let mut s = String::with_capacity(17);
    for (i, &b) in addr.iter().enumerate() {
        if i > 0 {
            s.push(':');
        }
        let _ = write!(s, "{b:02X}");
    }
    s
}
