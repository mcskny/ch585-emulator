use std::collections::VecDeque;

use crate::peripherals::irq;
use crate::pfic::Pfic;

#[derive(Debug, Clone)]
pub struct UsbPacketLog {
    pub ep: u8,
    pub is_in: bool,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone)]
enum ControlReqStep {
    SendSetup,
    DataIn,
    DataOut,
    StatusIn,
    StatusOut,
    WaitAddrSet,
}

#[derive(Debug, Clone)]
struct ControlRequest {
    setup: [u8; 8],
    data_out: Vec<u8>,
    data_in: Vec<u8>,
    out_offset: usize,
    expected_in: usize,
    tag: ControlTag,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ControlTag {
    GetDeviceDesc,
    SetAddress(u8),
    GetConfigHeader,
    GetConfigFull,
    GetString(u8),
    GetHidReportDesc(u16),
    SetIdle(u16),
    SetConfig(u8),
    CustomOut,
}

#[derive(Debug, Clone)]
pub struct UsbHs {
    // Core USBHS device registers (0x4000_9000..0x4000_9200)
    pub ctrl: u8,         // 0x00
    pub base_mode: u8,    // 0x01
    pub int_en: u8,       // 0x02
    pub dev_ad: u8,       // 0x03
    pub wake_ctrl: u8,    // 0x04
    pub test_mode: u8,    // 0x05
    pub lpm_data: u16,    // 0x06
    pub int_fg: u8,       // 0x08 (bit4 is computed from EP DONE bits)
    pub int_st: u8,       // 0x09
    pub mis_st: u8,       // 0x0A
    pub frame_no: u16,    // 0x0C
    pub bus_st: u16,      // 0x0E

    pub ep_tx_en: u16,    // 0x10
    pub ep_rx_en: u16,    // 0x12
    pub ep_t_tog_auto: u16,// 0x14
    pub ep_r_tog_auto: u16,// 0x16
    pub ep_t_burst: u8,   // 0x18
    pub ep_t_burst_mode: u8,// 0x19
    pub ep_r_burst: u8,   // 0x1A
    pub ep_r_res_mode: u8,// 0x1B
    pub ep_af_mode: u32,  // 0x1C

    pub ep0_dma: u32,     // 0x20
    pub ep_rx_dma: [u32; 8], // indices 1..=7 at 0x24..0x3C
    pub ep_tx_dma: [u32; 8], // indices 1..=7 at 0x40..0x58
    pub ep_max_len: [u32; 8],// indices 0..=7 at 0x5C..0x78
    pub ep_rx_len: [u16; 8], // 0x7C (ep0), 0x80+4*(n-1)
    pub ep_r_size: [u16; 8], // 0x82+4*(n-1)
    pub ep_t_len: [u16; 8],  // 0x9C + 4*n
    pub ep_tx_ctrl: [u8; 8], // 0x9E + 4*n
    pub ep_rx_ctrl: [u8; 8], // 0x9F + 4*n
    pub ep_t_iso: u16,    // 0xBC
    pub ep_r_iso: u16,    // 0xBE
    pub pll_ctrl: u8,     // 0x200

    // Virtual USB Host state
    pub host_attached: bool,
    pub enumerated: bool,
    pub suspended: bool,
    pub remote_wakeup_count: u32,
    bus_reset_sent: bool,
    attach_delay: u32,
    ctrl_cooldown: u32,
    req_queue: VecDeque<ControlRequest>,
    active_req: Option<(ControlRequest, ControlReqStep)>,

    // Parsed descriptors & logs from device
    pub device_desc: Vec<u8>,
    pub config_desc: Vec<u8>,
    pub string_descs: Vec<(u8, Vec<u8>)>,
    pub hid_report_descs: Vec<(u16, Vec<u8>)>,
    pub in_packets: Vec<UsbPacketLog>,
    pub out_queues: [VecDeque<Vec<u8>>; 8],
    sof_acc: u32,
    pub dma_active_step: bool,
    pub last_xfer_ep: u8,
    pub in_xfer_toggle: bool,
    pub out_xfer_toggle: bool,
}

impl Default for UsbHs {
    fn default() -> Self {
        Self::new()
    }
}

impl UsbHs {
    pub fn new() -> Self {
        Self {
            ctrl: 0x08, // PHY_SUSPENDM
            base_mode: 0,
            int_en: 0,
            dev_ad: 0,
            wake_ctrl: 0,
            test_mode: 0,
            lpm_data: 0,
            int_fg: 0,
            int_st: 0,
            mis_st: 0x08, // SIE_FREE
            frame_no: 0,
            bus_st: 0,
            ep_tx_en: 0,
            ep_rx_en: 0,
            ep_t_tog_auto: 0,
            ep_r_tog_auto: 0,
            ep_t_burst: 0,
            ep_t_burst_mode: 0,
            ep_r_burst: 0,
            ep_r_res_mode: 0,
            ep_af_mode: 0,
            ep0_dma: 0,
            ep_rx_dma: [0; 8],
            ep_tx_dma: [0; 8],
            ep_max_len: [0; 8],
            ep_rx_len: [0; 8],
            ep_r_size: [0; 8],
            ep_t_len: [0; 8],
            ep_tx_ctrl: [0; 8],
            ep_rx_ctrl: [0; 8],
            ep_t_iso: 0,
            ep_r_iso: 0,
            pll_ctrl: 0,
            host_attached: true,
            enumerated: false,
            suspended: false,
            remote_wakeup_count: 0,
            bus_reset_sent: false,
            attach_delay: 2000,
            ctrl_cooldown: 0,
            req_queue: VecDeque::new(),
            active_req: None,
            device_desc: Vec::new(),
            config_desc: Vec::new(),
            string_descs: Vec::new(),
            hid_report_descs: Vec::new(),
            in_packets: Vec::new(),
            out_queues: std::array::from_fn(|_| VecDeque::new()),
            sof_acc: 0,
            dma_active_step: false,
            last_xfer_ep: 0,
            in_xfer_toggle: false,
            out_xfer_toggle: false,
        }
    }

    fn reset_host_session(&mut self) {
        self.enumerated = false;
        self.suspended = false;
        self.bus_reset_sent = false;
        self.attach_delay = 2000;
        self.ctrl_cooldown = 0;
        self.req_queue.clear();
        self.active_req = None;
        self.device_desc.clear();
        self.config_desc.clear();
        self.string_descs.clear();
        self.hid_report_descs.clear();
        self.mis_st &= !0x81; // clear HS_MOD & READY until next reset/enum
    }

    pub fn detach(&mut self) {
        self.host_attached = false;
        self.reset_host_session();
    }

    pub fn attach(&mut self) {
        self.host_attached = true;
        if !self.enumerated && !self.bus_reset_sent {
            self.attach_delay = 500;
        }
    }

    pub fn suspend(&mut self) {
        if self.enumerated && !self.suspended {
            self.suspended = true;
            self.int_fg |= 0x02; // USBHS_UDIF_SUSPEND
        }
    }

    pub fn resume(&mut self) {
        if self.suspended {
            self.suspended = false;
            self.int_fg &= !0x02;
            self.int_fg |= 0x40; // USBHS_UDIF_LINK_RDY
        }
    }

    pub fn trigger_enumerate(&mut self) {
        self.host_attached = true;
        self.reset_host_session();
        self.attach_delay = 100;
    }

    pub fn queue_out_packet(&mut self, ep: u8, data: Vec<u8>) {
        if (1..8).contains(&ep) {
            self.out_queues[ep as usize].push_back(data);
        }
    }

    pub fn queue_ep0_set_report(&mut self, report_type: u8, report_id: u8, iface: u16, data: Vec<u8>) {
        let len = data.len() as u16;
        let wval = ((report_type as u16) << 8) | (report_id as u16);
        let setup = [
            0x21,
            0x09, // HID SET_REPORT
            wval as u8,
            (wval >> 8) as u8,
            iface as u8,
            (iface >> 8) as u8,
            len as u8,
            (len >> 8) as u8,
        ];
        self.req_queue.push_back(ControlRequest {
            setup,
            data_out: data,
            data_in: Vec::new(),
            out_offset: 0,
            expected_in: 0,
            tag: ControlTag::CustomOut,
        });
    }

    fn effective_int_fg(&self) -> u8 {
        let any_done = (0..8).any(|ep| {
            ((self.ep_tx_ctrl[ep] & 0x80) != 0) || ((self.ep_rx_ctrl[ep] & 0x80) != 0)
        });
        (self.int_fg & !0x10) | if any_done { 0x10 } else { 0 }
    }

    pub fn device_enabled(&self) -> bool {
        (self.ctrl & 0x20) != 0 && (self.ctrl & 0x03) == 0 && (self.pll_ctrl & 0x04) != 0
    }

    fn has_ready_transfer(&self) -> bool {
        const SRAM_LEN: usize = 128 * 1024;
        if self.active_req.is_none() {
            if !self.req_queue.is_empty() {
                let dma = (self.ep0_dma & 0x1FFFF) as usize;
                return dma + 8 <= SRAM_LEN && (self.ep_rx_en & 1) != 0;
            }
        } else if let Some((_, ref step)) = self.active_req {
            return match step {
                ControlReqStep::SendSetup => {
                    let dma = (self.ep0_dma & 0x1FFFF) as usize;
                    dma + 8 <= SRAM_LEN && (self.ep_rx_en & 1) != 0
                }
                ControlReqStep::DataIn | ControlReqStep::StatusIn => {
                    matches!(self.ep_tx_ctrl[0] & 0x03, 0x01 | 0x02)
                }
                ControlReqStep::DataOut | ControlReqStep::StatusOut => {
                    matches!(self.ep_rx_ctrl[0] & 0x03, 0x01 | 0x02)
                }
                ControlReqStep::WaitAddrSet => self.dev_ad != 0,
            };
        }

        if self.enumerated {
            for ep in 1..8usize {
                if (self.ep_tx_en & (1 << ep)) != 0
                    && (self.ep_tx_ctrl[ep] & 0x80) == 0
                    && (self.ep_tx_ctrl[ep] & 0x03) == 0x02
                {
                    return true;
                }
                if (self.ep_rx_en & (1 << ep)) != 0
                    && (self.ep_rx_ctrl[ep] & 0x80) == 0
                    && (self.ep_rx_ctrl[ep] & 0x03) == 0x02
                    && !self.out_queues[ep].is_empty()
                {
                    return true;
                }
            }
        }
        false
    }

    /// Cycles until the next observable USBHS event (reset, transfer, detach, or SOF IRQ).
    pub fn next_event_cycles(&self, sof_irq_observable: bool) -> Option<u32> {
        if !self.device_enabled() || !self.host_attached {
            return if self.bus_reset_sent
                || self.enumerated
                || self.suspended
                || self.active_req.is_some()
                || !self.req_queue.is_empty()
            {
                Some(1)
            } else {
                None
            };
        }
        if self.suspended {
            return None;
        }
        let mut best: Option<u32> = None;
        if sof_irq_observable && (self.int_en & 0x20) != 0 && (self.int_fg & 0x20) == 0 {
            let sof_c = 7800u32.saturating_sub(self.sof_acc).max(1);
            best = Some(sof_c);
        }
        if !self.bus_reset_sent {
            let att_c = self.attach_delay.max(1);
            best = Some(best.map_or(att_c, |b| b.min(att_c)));
        } else if (self.int_fg & 0x41) == 0 && !self.any_ep_done() && self.has_ready_transfer() {
            let xfer_c = self.ctrl_cooldown.max(1);
            best = Some(best.map_or(xfer_c, |b| b.min(xfer_c)));
        }
        best
    }

    /// Advances USBHS SOF/cooldown counters in O(1) while no transfer/reset fires.
    pub fn advance_idle_cycles(&mut self, cycles: u64, pfic: &mut Pfic) {
        self.dma_active_step = false;
        if cycles == 0 || !self.device_enabled() || !self.host_attached || self.suspended {
            return;
        }
        let total = (self.sof_acc as u64).saturating_add(cycles);
        let wraps = total / 7800;
        self.sof_acc = (total % 7800) as u32;
        if wraps > 0 {
            self.frame_no = self.frame_no.wrapping_add(wraps as u16);
            if (self.int_en & 0x20) != 0 {
                self.int_fg |= 0x20;
            }
        }
        let cyc_u32 = cycles.min(u32::MAX as u64) as u32;
        if !self.bus_reset_sent {
            self.attach_delay = self.attach_delay.saturating_sub(cyc_u32);
        } else if (self.int_fg & 0x41) == 0 && !self.any_ep_done() {
            self.ctrl_cooldown = self.ctrl_cooldown.saturating_sub(cyc_u32);
        }
        self.sync_irq(pfic);
    }

    fn any_ep_done(&self) -> bool {
        (0..8).any(|ep| ((self.ep_tx_ctrl[ep] | self.ep_rx_ctrl[ep]) & 0x80) != 0)
    }

    fn sync_irq(&self, pfic: &mut Pfic) {
        let asserted = (self.effective_int_fg() & self.int_en) != 0;
        pfic.sync_level_irq(irq::USB2_DEVICE, asserted);
    }

    fn queue_std_enumeration(&mut self) {
        self.req_queue.clear();
        // 1. Get Device Descriptor (18 bytes)
        self.req_queue.push_back(ControlRequest {
            setup: [0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 18, 0],
            data_out: Vec::new(),
            data_in: Vec::new(),
            out_offset: 0,
            expected_in: 18,
            tag: ControlTag::GetDeviceDesc,
        });
        // 2. Set Address = 1
        self.req_queue.push_back(ControlRequest {
            setup: [0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00],
            data_out: Vec::new(),
            data_in: Vec::new(),
            out_offset: 0,
            expected_in: 0,
            tag: ControlTag::SetAddress(1),
        });
        // 3. Get Config Descriptor header (9 bytes)
        self.req_queue.push_back(ControlRequest {
            setup: [0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 9, 0],
            data_out: Vec::new(),
            data_in: Vec::new(),
            out_offset: 0,
            expected_in: 9,
            tag: ControlTag::GetConfigHeader,
        });
    }

    fn finish_control_request(&mut self, req: ControlRequest) {
        match req.tag {
            ControlTag::GetDeviceDesc => {
                self.device_desc = req.data_in;
            }
            ControlTag::SetAddress(_) => {}
            ControlTag::GetConfigHeader => {
                if req.data_in.len() >= 4 {
                    let total_len = u16::from_le_bytes([req.data_in[2], req.data_in[3]]);
                    self.req_queue.push_front(ControlRequest {
                        setup: [
                            0x80,
                            0x06,
                            0x00,
                            0x02,
                            0x00,
                            0x00,
                            total_len as u8,
                            (total_len >> 8) as u8,
                        ],
                        data_out: Vec::new(),
                        data_in: Vec::new(),
                        out_offset: 0,
                        expected_in: total_len as usize,
                        tag: ControlTag::GetConfigFull,
                    });
                }
            }
            ControlTag::GetConfigFull => {
                self.config_desc = req.data_in.clone();
                // Parse interfaces to fetch HID Report descriptors and Strings
                let mut follow_ups = Vec::new();
                let mut idx = 0usize;
                let mut cur_iface: u16 = 0;
                while idx + 2 <= req.data_in.len() {
                    let b_len = req.data_in[idx] as usize;
                    if b_len < 2 || idx + b_len > req.data_in.len() {
                        break;
                    }
                    let b_type = req.data_in[idx + 1];
                    if b_type == 0x04 && b_len >= 9 {
                        cur_iface = req.data_in[idx + 2] as u16;
                    } else if b_type == 0x21 && b_len >= 9 {
                        // HID descriptor: wDescriptorLength at bytes 7..9
                        let rep_len = u16::from_le_bytes([req.data_in[idx + 7], req.data_in[idx + 8]]);
                        follow_ups.push(ControlRequest {
                            setup: [
                                0x81,
                                0x06,
                                0x00,
                                0x22,
                                cur_iface as u8,
                                (cur_iface >> 8) as u8,
                                rep_len as u8,
                                (rep_len >> 8) as u8,
                            ],
                            data_out: Vec::new(),
                            data_in: Vec::new(),
                            out_offset: 0,
                            expected_in: rep_len as usize,
                            tag: ControlTag::GetHidReportDesc(cur_iface),
                        });
                        follow_ups.push(ControlRequest {
                            setup: [
                                0x21,
                                0x0A, // SET_IDLE
                                0x00,
                                0x00,
                                cur_iface as u8,
                                (cur_iface >> 8) as u8,
                                0x00,
                                0x00,
                            ],
                            data_out: Vec::new(),
                            data_in: Vec::new(),
                            out_offset: 0,
                            expected_in: 0,
                            tag: ControlTag::SetIdle(cur_iface),
                        });
                    }
                    idx += b_len;
                }
                for str_idx in 0u8..=3u8 {
                    follow_ups.push(ControlRequest {
                        setup: [0x80, 0x06, str_idx, 0x03, 0x00, 0x00, 64, 0],
                        data_out: Vec::new(),
                        data_in: Vec::new(),
                        out_offset: 0,
                        expected_in: 64,
                        tag: ControlTag::GetString(str_idx),
                    });
                }
                let cfg_val = req.data_in.get(5).copied().unwrap_or(1);
                follow_ups.push(ControlRequest {
                    setup: [0x00, 0x09, cfg_val, 0x00, 0x00, 0x00, 0x00, 0x00],
                    data_out: Vec::new(),
                    data_in: Vec::new(),
                    out_offset: 0,
                    expected_in: 0,
                    tag: ControlTag::SetConfig(cfg_val),
                });
                // Push in reverse onto front so order is preserved
                for r in follow_ups.into_iter().rev() {
                    self.req_queue.push_front(r);
                }
            }
            ControlTag::GetString(idx) => {
                self.string_descs.push((idx, req.data_in));
            }
            ControlTag::GetHidReportDesc(iface) => {
                self.hid_report_descs.push((iface, req.data_in));
            }
            ControlTag::SetIdle(_) => {}
            ControlTag::SetConfig(_) => {
                self.enumerated = true;
            }
            ControlTag::CustomOut => {}
        }
    }

    pub fn step(&mut self, cycles: u32, pfic: &mut Pfic, sram: &mut [u8]) {
        self.dma_active_step = false;
        if !self.device_enabled() || !self.host_attached {
            if self.bus_reset_sent
                || self.enumerated
                || self.suspended
                || self.active_req.is_some()
                || !self.req_queue.is_empty()
            {
                self.reset_host_session();
            }
            self.sync_irq(pfic);
            return;
        }

        if self.suspended {
            self.sync_irq(pfic);
            return;
        }

        // High-speed SOF every 125us (~7800 cycles @ 62.4MHz)
        self.sof_acc += cycles;
        if self.sof_acc >= 7800 {
            self.sof_acc -= 7800;
            self.frame_no = self.frame_no.wrapping_add(1);
            if (self.int_en & 0x20) != 0 {
                self.int_fg |= 0x20;
            }
        }

        // Initial connect + Bus Reset sequence
        if !self.bus_reset_sent {
            if self.attach_delay > cycles {
                self.attach_delay -= cycles;
                self.sync_irq(pfic);
                return;
            }
            self.attach_delay = 0;
            self.bus_reset_sent = true;
            let hs = (self.base_mode & 0x03) == 0x01;
            self.mis_st = (if hs { 0x80 } else { 0x00 }) | 0x08 | 0x01; // HS_MOD | SIE_FREE | READY
            self.int_fg |= 0x40 | 0x01; // LINK_RDY | BUS_RST
            self.queue_std_enumeration();
            self.ctrl_cooldown = 1500;
            self.sync_irq(pfic);
            return;
        }

        // Wait until bus reset / previous interrupt flags settle
        if (self.int_fg & 0x41) != 0 || self.any_ep_done() {
            self.sync_irq(pfic);
            return;
        }

        if self.ctrl_cooldown > cycles {
            self.ctrl_cooldown -= cycles;
            self.sync_irq(pfic);
            return;
        }
        self.ctrl_cooldown = 0;

        // Advance EP0 Control Transfers
        if self.active_req.is_none() {
            if let Some(req) = self.req_queue.pop_front() {
                self.active_req = Some((req, ControlReqStep::SendSetup));
            }
        }

        if let Some((mut req, step)) = self.active_req.take() {
            let dma = (self.ep0_dma & 0x1FFFF) as usize;
            let tx_res = self.ep_tx_ctrl[0] & 0x03;
            let rx_res = self.ep_rx_ctrl[0] & 0x03;

            match step {
                ControlReqStep::SendSetup => {
                    if dma + 8 <= sram.len() && (self.ep_rx_en & 1) != 0 {
                        sram[dma..dma + 8].copy_from_slice(&req.setup);
                        self.dma_active_step = true;
                        self.last_xfer_ep = 0;
                        self.out_xfer_toggle = !self.out_xfer_toggle;
                        self.ep_rx_len[0] = 8;
                        self.ep_rx_ctrl[0] = (self.ep_rx_ctrl[0] & 0x07) | 0x80 | 0x10 | 0x08; // DONE | TOG_MATCH | SETUP_IS
                        self.int_st = 0x00; // EP0 OUT/SETUP
                        let next_step = if (req.setup[0] & 0x80) != 0 && req.expected_in > 0 {
                            ControlReqStep::DataIn
                        } else if !req.data_out.is_empty() {
                            ControlReqStep::DataOut
                        } else {
                            ControlReqStep::StatusIn
                        };
                        self.active_req = Some((req, next_step));
                        self.ctrl_cooldown = 200;
                    } else {
                        self.active_req = Some((req, ControlReqStep::SendSetup));
                    }
                }
                ControlReqStep::DataIn => {
                    if tx_res == 0x01 {
                        // STALL: abort request cleanly
                        self.ep_tx_ctrl[0] |= 0x80;
                        self.int_st = 0x10;
                    } else if tx_res == 0x02 {
                        let len = (self.ep_t_len[0] as usize).min(64);
                        if dma + len <= sram.len() {
                            req.data_in.extend_from_slice(&sram[dma..dma + len]);
                        }
                        self.dma_active_step = true;
                        self.last_xfer_ep = 0;
                        self.in_xfer_toggle = !self.in_xfer_toggle;
                        self.ep_tx_ctrl[0] |= 0x80; // DONE
                        self.int_st = 0x10; // EP0 IN
                        self.ctrl_cooldown = 200;
                        if len < 64 || req.data_in.len() >= req.expected_in {
                            self.active_req = Some((req, ControlReqStep::StatusOut));
                        } else {
                            self.active_req = Some((req, ControlReqStep::DataIn));
                        }
                    } else {
                        self.active_req = Some((req, ControlReqStep::DataIn));
                    }
                }
                ControlReqStep::DataOut => {
                    if rx_res == 0x01 {
                        self.ep_rx_ctrl[0] |= 0x80;
                        self.int_st = 0x00;
                    } else if rx_res == 0x02 {
                        let rem = req.data_out.len() - req.out_offset;
                        let chunk = rem.min(64);
                        if dma + chunk <= sram.len() {
                            sram[dma..dma + chunk]
                                .copy_from_slice(&req.data_out[req.out_offset..req.out_offset + chunk]);
                        }
                        self.dma_active_step = true;
                        self.last_xfer_ep = 0;
                        self.out_xfer_toggle = !self.out_xfer_toggle;
                        req.out_offset += chunk;
                        self.ep_rx_len[0] = chunk as u16;
                        self.ep_rx_ctrl[0] = (self.ep_rx_ctrl[0] & 0x07) | 0x80 | 0x10; // DONE | TOG_MATCH (not SETUP)
                        self.int_st = 0x00;
                        self.ctrl_cooldown = 200;
                        if req.out_offset >= req.data_out.len() {
                            self.active_req = Some((req, ControlReqStep::StatusIn));
                        } else {
                            self.active_req = Some((req, ControlReqStep::DataOut));
                        }
                    } else {
                        self.active_req = Some((req, ControlReqStep::DataOut));
                    }
                }
                ControlReqStep::StatusIn => {
                    if tx_res == 0x01 {
                        self.ep_tx_ctrl[0] |= 0x80;
                        self.int_st = 0x10;
                    } else if tx_res == 0x02 {
                        self.ep_tx_ctrl[0] |= 0x80;
                        self.last_xfer_ep = 0;
                        self.in_xfer_toggle = !self.in_xfer_toggle;
                        self.int_st = 0x10;
                        self.ctrl_cooldown = 200;
                        if let ControlTag::SetAddress(_) = req.tag {
                            self.active_req = Some((req, ControlReqStep::WaitAddrSet));
                        } else {
                            self.finish_control_request(req);
                        }
                    } else {
                        self.active_req = Some((req, ControlReqStep::StatusIn));
                    }
                }
                ControlReqStep::StatusOut => {
                    if rx_res == 0x01 {
                        self.ep_rx_ctrl[0] |= 0x80;
                        self.int_st = 0x00;
                    } else if rx_res == 0x02 {
                        self.ep_rx_len[0] = 0;
                        self.last_xfer_ep = 0;
                        self.out_xfer_toggle = !self.out_xfer_toggle;
                        self.ep_rx_ctrl[0] = (self.ep_rx_ctrl[0] & 0x07) | 0x80 | 0x10;
                        self.int_st = 0x00;
                        self.ctrl_cooldown = 200;
                        self.finish_control_request(req);
                    } else {
                        self.active_req = Some((req, ControlReqStep::StatusOut));
                    }
                }
                ControlReqStep::WaitAddrSet => {
                    // Wait until firmware writes R8_USB2_DEV_AD
                    if self.dev_ad != 0 {
                        self.finish_control_request(req);
                    } else {
                        self.active_req = Some((req, ControlReqStep::WaitAddrSet));
                    }
                }
            }
            self.sync_irq(pfic);
            return;
        }

        // Once enumerated, poll EP1..EP7 IN & OUT endpoints
        if self.enumerated {
            for ep in 1..8usize {
                if (self.ep_tx_en & (1 << ep)) != 0
                    && (self.ep_tx_ctrl[ep] & 0x80) == 0
                    && (self.ep_tx_ctrl[ep] & 0x03) == 0x02
                {
                    let len = (self.ep_t_len[ep] as usize).min(1024);
                    let dma = (self.ep_tx_dma[ep] & 0x1FFFF) as usize;
                    let data = if dma + len <= sram.len() {
                        sram[dma..dma + len].to_vec()
                    } else {
                        Vec::new()
                    };
                    self.in_packets.push(UsbPacketLog {
                        ep: ep as u8,
                        is_in: true,
                        data,
                    });
                    self.dma_active_step = true;
                    self.last_xfer_ep = ep as u8;
                    self.in_xfer_toggle = !self.in_xfer_toggle;
                    self.ep_tx_ctrl[ep] |= 0x80; // DONE
                    self.int_st = 0x10 | (ep as u8);
                    self.ctrl_cooldown = 200;
                    self.sync_irq(pfic);
                    return;
                }

                if (self.ep_rx_en & (1 << ep)) != 0
                    && (self.ep_rx_ctrl[ep] & 0x80) == 0
                    && (self.ep_rx_ctrl[ep] & 0x03) == 0x02
                {
                    if let Some(pkt) = self.out_queues[ep].pop_front() {
                        let max_l = (self.ep_max_len[ep] as usize).max(64).min(1024);
                        let len = pkt.len().min(max_l);
                        let dma = (self.ep_rx_dma[ep] & 0x1FFFF) as usize;
                        if dma + len <= sram.len() {
                            sram[dma..dma + len].copy_from_slice(&pkt[..len]);
                        }
                        self.ep_rx_len[ep] = len as u16;
                        self.ep_r_size[ep] = len as u16;
                        self.ep_rx_ctrl[ep] = (self.ep_rx_ctrl[ep] & 0x0F) | 0x80 | 0x10; // DONE | TOG_MATCH
                        self.in_packets.push(UsbPacketLog {
                            ep: ep as u8,
                            is_in: false,
                            data: pkt[..len].to_vec(),
                        });
                        self.dma_active_step = true;
                        self.last_xfer_ep = ep as u8;
                        self.out_xfer_toggle = !self.out_xfer_toggle;
                        self.int_st = ep as u8;
                        self.ctrl_cooldown = 200;
                        self.sync_irq(pfic);
                        return;
                    }
                }
            }
        }

        self.sync_irq(pfic);
    }

    pub fn read_u8(&self, offset: u32) -> u8 {
        match offset {
            0x00 => self.ctrl,
            0x01 => self.base_mode,
            0x02 => self.int_en,
            0x03 => self.dev_ad,
            0x04 => self.wake_ctrl,
            0x05 => self.test_mode,
            0x06 => self.lpm_data as u8,
            0x07 => (self.lpm_data >> 8) as u8,
            0x08 => self.effective_int_fg(),
            0x09 => self.int_st,
            0x0A => self.mis_st,
            0x0C => self.frame_no as u8,
            0x0D => (self.frame_no >> 8) as u8,
            0x0E => self.bus_st as u8,
            0x0F => (self.bus_st >> 8) as u8,
            0x10 => self.ep_tx_en as u8,
            0x11 => (self.ep_tx_en >> 8) as u8,
            0x12 => self.ep_rx_en as u8,
            0x13 => (self.ep_rx_en >> 8) as u8,
            0x14 => self.ep_t_tog_auto as u8,
            0x15 => (self.ep_t_tog_auto >> 8) as u8,
            0x16 => self.ep_r_tog_auto as u8,
            0x17 => (self.ep_r_tog_auto >> 8) as u8,
            0x18 => self.ep_t_burst,
            0x19 => self.ep_t_burst_mode,
            0x1A => self.ep_r_burst,
            0x1B => self.ep_r_res_mode,
            0x1C..=0x1F => (self.ep_af_mode >> ((offset - 0x1C) * 8)) as u8,
            0x20..=0x23 => (self.ep0_dma >> ((offset - 0x20) * 8)) as u8,
            0x24..=0x3F => {
                let idx = ((offset - 0x24) / 4) as usize + 1;
                (self.ep_rx_dma[idx] >> ((offset & 3) * 8)) as u8
            }
            0x40..=0x5B => {
                let idx = ((offset - 0x40) / 4) as usize + 1;
                (self.ep_tx_dma[idx] >> ((offset & 3) * 8)) as u8
            }
            0x5C..=0x7B => {
                let idx = ((offset - 0x5C) / 4) as usize;
                (self.ep_max_len[idx] >> ((offset & 3) * 8)) as u8
            }
            0x7C => self.ep_rx_len[0] as u8,
            0x7D => (self.ep_rx_len[0] >> 8) as u8,
            0x80..=0x9B => {
                let idx = ((offset - 0x80) / 4) as usize + 1;
                match offset & 3 {
                    0 => self.ep_rx_len[idx] as u8,
                    1 => (self.ep_rx_len[idx] >> 8) as u8,
                    2 => self.ep_r_size[idx] as u8,
                    _ => (self.ep_r_size[idx] >> 8) as u8,
                }
            }
            0x9C..=0xBB => {
                let idx = ((offset - 0x9C) / 4) as usize;
                match offset & 3 {
                    0 => self.ep_t_len[idx] as u8,
                    1 => (self.ep_t_len[idx] >> 8) as u8,
                    2 => self.ep_tx_ctrl[idx],
                    _ => self.ep_rx_ctrl[idx],
                }
            }
            0xBC => self.ep_t_iso as u8,
            0xBD => (self.ep_t_iso >> 8) as u8,
            0xBE => self.ep_r_iso as u8,
            0xBF => (self.ep_r_iso >> 8) as u8,
            0x200 => self.pll_ctrl,
            _ => 0,
        }
    }

    pub fn write_u8(&mut self, offset: u32, val: u8) {
        match offset {
            0x00 => {
                if (val & 0x04) != 0 {
                    // USBHS_UD_CLR_ALL
                    self.int_fg = 0;
                    for ep in 0..8 {
                        self.ep_tx_ctrl[ep] &= !0x80;
                        self.ep_rx_ctrl[ep] &= !0x80;
                    }
                }
                if (val & 0x03) != 0 {
                    // Reset SIE/LINK
                    self.dev_ad = 0;
                }
                self.ctrl = val & !0x04;
            }
            0x01 => self.base_mode = val,
            0x02 => self.int_en = val,
            0x03 => self.dev_ad = val & 0x7F,
            0x04 => {
                if (val & 0x01) != 0 {
                    // USBHS_UD_UD_REMOTE_WKUP: triggers resume if suspended and auto-clears bit 0
                    if self.suspended {
                        self.suspended = false;
                        self.remote_wakeup_count += 1;
                        self.int_fg &= !0x02;
                        self.int_fg |= 0x40;
                    }
                    self.wake_ctrl = val & !0x01;
                } else {
                    self.wake_ctrl = val;
                }
            }
            0x05 => self.test_mode = val,
            0x06 => self.lpm_data = (self.lpm_data & 0xFF00) | (val as u16),
            0x07 => self.lpm_data = (self.lpm_data & 0x00FF) | ((val as u16) << 8),
            0x08 => self.int_fg &= !(val & !0x10), // RW1C (except bit4 RO)
            0x10 => self.ep_tx_en = (self.ep_tx_en & 0xFF00) | (val as u16),
            0x11 => self.ep_tx_en = (self.ep_tx_en & 0x00FF) | ((val as u16) << 8),
            0x12 => self.ep_rx_en = (self.ep_rx_en & 0xFF00) | (val as u16),
            0x13 => self.ep_rx_en = (self.ep_rx_en & 0x00FF) | ((val as u16) << 8),
            0x14 => self.ep_t_tog_auto = (self.ep_t_tog_auto & 0xFF00) | (val as u16),
            0x15 => self.ep_t_tog_auto = (self.ep_t_tog_auto & 0x00FF) | ((val as u16) << 8),
            0x16 => self.ep_r_tog_auto = (self.ep_r_tog_auto & 0xFF00) | (val as u16),
            0x17 => self.ep_r_tog_auto = (self.ep_r_tog_auto & 0x00FF) | ((val as u16) << 8),
            0x18 => self.ep_t_burst = val,
            0x19 => self.ep_t_burst_mode = val,
            0x1A => self.ep_r_burst = val,
            0x1B => self.ep_r_res_mode = val,
            0x1C..=0x1F => {
                let s = (offset - 0x1C) * 8;
                self.ep_af_mode = (self.ep_af_mode & !(0xFF << s)) | ((val as u32) << s);
            }
            0x20..=0x23 => {
                let s = (offset - 0x20) * 8;
                self.ep0_dma = (self.ep0_dma & !(0xFF << s)) | ((val as u32) << s);
            }
            0x24..=0x3F => {
                let idx = ((offset - 0x24) / 4) as usize + 1;
                let s = (offset & 3) * 8;
                self.ep_rx_dma[idx] = (self.ep_rx_dma[idx] & !(0xFF << s)) | ((val as u32) << s);
            }
            0x40..=0x5B => {
                let idx = ((offset - 0x40) / 4) as usize + 1;
                let s = (offset & 3) * 8;
                self.ep_tx_dma[idx] = (self.ep_tx_dma[idx] & !(0xFF << s)) | ((val as u32) << s);
            }
            0x5C..=0x7B => {
                let idx = ((offset - 0x5C) / 4) as usize;
                let s = (offset & 3) * 8;
                self.ep_max_len[idx] = (self.ep_max_len[idx] & !(0xFF << s)) | ((val as u32) << s);
            }
            0x7C => self.ep_rx_len[0] = (self.ep_rx_len[0] & 0xFF00) | (val as u16),
            0x7D => self.ep_rx_len[0] = (self.ep_rx_len[0] & 0x00FF) | ((val as u16) << 8),
            0x80..=0x9B => {
                let idx = ((offset - 0x80) / 4) as usize + 1;
                match offset & 3 {
                    0 => self.ep_rx_len[idx] = (self.ep_rx_len[idx] & 0xFF00) | (val as u16),
                    1 => self.ep_rx_len[idx] = (self.ep_rx_len[idx] & 0x00FF) | ((val as u16) << 8),
                    2 => self.ep_r_size[idx] = (self.ep_r_size[idx] & 0xFF00) | (val as u16),
                    _ => self.ep_r_size[idx] = (self.ep_r_size[idx] & 0x00FF) | ((val as u16) << 8),
                }
            }
            0x9C..=0xBB => {
                let idx = ((offset - 0x9C) / 4) as usize;
                match offset & 3 {
                    0 => self.ep_t_len[idx] = (self.ep_t_len[idx] & 0xFF00) | (val as u16),
                    1 => self.ep_t_len[idx] = (self.ep_t_len[idx] & 0x00FF) | ((val as u16) << 8),
                    2 => self.ep_tx_ctrl[idx] = val,
                    _ => self.ep_rx_ctrl[idx] = val,
                }
            }
            0xBC => self.ep_t_iso = (self.ep_t_iso & 0xFF00) | (val as u16),
            0xBD => self.ep_t_iso = (self.ep_t_iso & 0x00FF) | ((val as u16) << 8),
            0xBE => self.ep_r_iso = (self.ep_r_iso & 0xFF00) | (val as u16),
            0xBF => self.ep_r_iso = (self.ep_r_iso & 0x00FF) | ((val as u16) << 8),
            0x200 => self.pll_ctrl = val,
            _ => {}
        }
    }

    pub fn read_u32(&self, offset: u32) -> u32 {
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
