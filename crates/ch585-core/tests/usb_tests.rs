use ch585_core::soc::Ch585Machine;

fn root_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap()
        .to_path_buf()
}

#[test]
fn test_usb_hid_elf_enumeration_and_bidirectional_hid_transfers() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/usb_hid.elf")) else {
        eprintln!("Skipping test_usb_hid_elf_enumeration_and_bidirectional_hid_transfers: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();

    // 1. Run until enumerated
    mach.run_cycles(500_000);
    let uart0 = String::from_utf8_lossy(&mach.bus.uarts[0].tx_log).to_string();
    assert!(
        uart0.contains("Compatibility HID Running On USBHS Controller"),
        "UART0: {uart0}"
    );
    assert!(mach.bus.usbhs.enumerated);
    assert_eq!(mach.bus.usbhs.dev_ad, 1);
    assert_eq!(mach.bus.usbhs.device_desc.len(), 18);
    assert_eq!(
        u16::from_le_bytes([mach.bus.usbhs.device_desc[8], mach.bus.usbhs.device_desc[9]]),
        0x1A86
    );
    assert_eq!(
        u16::from_le_bytes([mach.bus.usbhs.device_desc[10], mach.bus.usbhs.device_desc[11]]),
        0xFE07
    );
    assert!(!mach.bus.usbhs.hid_report_descs.is_empty());

    // 2. Host -> EP1 OUT -> UART2 TX
    let msg = b"USB2UART";
    let mut pkt = vec![msg.len() as u8, 0];
    pkt.extend_from_slice(msg);
    mach.bus.usbhs.queue_out_packet(1, pkt);
    mach.run_cycles(200_000);
    let uart2_tx = String::from_utf8_lossy(&mach.bus.uarts[2].tx_log).to_string();
    assert_eq!(uart2_tx, "USB2UART");

    // 3. UART2 RX -> EP2 IN (allow TMR2 timeout to trigger upload)
    mach.bus.uarts[2].inject_rx(b"UART2USB");
    mach.run_us(15_000);
    let ep2_in = mach
        .bus
        .usbhs
        .in_packets
        .iter()
        .find(|p| p.ep == 2 && p.is_in)
        .expect("Expected EP2 IN HID report from UART2 RX");
    assert_eq!(&ep2_in.data[2..10], b"UART2USB");
    assert!(mach.bus.unknown_mmio.is_empty());
}
