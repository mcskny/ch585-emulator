use ch585_core::peripherals::adc::AdcWaveform;
use ch585_core::peripherals::gpio::ExtPinDrive;
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
fn test_uygulama_elf_boot_usb_enumeration_hall_key_and_vendor_ep4() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/uygulama.elf")) else {
        eprintln!("Skipping test_uygulama_elf_boot_usb_enumeration_hall_key_and_vendor_ep4: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();

    // Drive VBUS sense high (PB9) and rest voltage (1650 mV) on Hall sensor ADC channels
    mach.set_pin_drive('B', 9, ExtPinDrive::High);
    for ch in 0..14 {
        mach.bus.adc.channels[ch] = AdcWaveform::Dc(1650);
    }

    // Boot, finish REST_CAL N=16, and complete USB HS enumeration
    mach.run_us(120_000);

    let uart2 = String::from_utf8_lossy(&mach.bus.uarts[2].tx_log).to_string();
    assert!(uart2.contains("CHIP=0x93"), "UART2: {uart2}");
    assert!(uart2.contains("SYSCLK=62400000"), "UART2: {uart2}");
    assert!(uart2.contains("REST_CAL N=16"), "UART2: {uart2}");

    assert!(mach.bus.usbhs.enumerated);
    assert_eq!(
        u16::from_le_bytes([mach.bus.usbhs.device_desc[8], mach.bus.usbhs.device_desc[9]]),
        0x1A86
    );
    assert_eq!(
        u16::from_le_bytes([mach.bus.usbhs.device_desc[10], mach.bus.usbhs.device_desc[11]]),
        0xFE10
    );
    assert_eq!(mach.bus.usbhs.hid_report_descs.len(), 4);

    // Depress key on scanned ADC channels -> trigger non-zero NKRO HID report on EP1 IN
    mach.bus.usbhs.in_packets.clear();
    mach.bus.adc.channels[0] = AdcWaveform::Dc(2800);
    mach.bus.adc.channels[1] = AdcWaveform::Dc(2800);
    mach.run_us(30_000);

    let has_nonzero_nkro = mach
        .bus
        .usbhs
        .in_packets
        .iter()
        .any(|p| p.ep == 1 && p.is_in && p.data.iter().any(|&b| b != 0));
    assert!(
        has_nonzero_nkro,
        "Expected non-zero EP1 IN NKRO keyboard report after driving Hall ADC channel; packets={:?}",
        mach.bus.usbhs.in_packets
    );

    // Send 64-byte vendor command on EP4 OUT starting with magic [0x57, 0x01, 0x01, 0x42]
    mach.bus.usbhs.in_packets.clear();
    let mut vendor_pkt = vec![0u8; 64];
    vendor_pkt[0] = 0x57;
    vendor_pkt[1] = 0x01;
    vendor_pkt[2] = 0x01;
    vendor_pkt[3] = 0x42;
    mach.bus.usbhs.queue_out_packet(4, vendor_pkt);
    mach.run_us(15_000);

    let ep4_in = mach
        .bus
        .usbhs
        .in_packets
        .iter()
        .find(|p| p.ep == 4 && p.is_in)
        .expect("Expected EP4 IN vendor response packet");
    assert!(!ep4_in.data.is_empty());
    assert!(mach.bus.unknown_mmio.is_empty(), "Unknown MMIO: {:08X?}", mach.bus.unknown_mmio);
}

#[test]
fn test_ble_klavye_elf_boot_and_advertising_bridge() {
    let Ok(elf) = std::fs::read(root_dir().join("kaynak/test_elf/ble_klavye.elf")) else {
        eprintln!("Skipping test_ble_klavye_elf_boot_and_advertising_bridge: optional ELF fixture not found");
        return;
    };
    let mut mach = Ch585Machine::new();
    mach.load_elf(&elf).unwrap();

    mach.run_us(150_000);

    let uart0 = String::from_utf8_lossy(&mach.bus.uarts[0].tx_log).to_string();
    assert!(uart0.contains("CH585_BLE_LIB"), "UART0: {uart0}");
    assert!(uart0.contains("Initialized.."), "UART0: {uart0}");
    assert!(uart0.contains("Advertising.."), "UART0: {uart0}");

    assert!(mach.bus.ble.advertising_enabled);
    assert!(!mach.bus.ble.adv_data.is_empty());
    assert!(!mach.bus.ble.scan_rsp_data.is_empty());
    assert!(!mach.bus.ble.event_log.is_empty());
    assert!(mach.bus.unknown_mmio.is_empty(), "Unknown MMIO: {:08X?}", mach.bus.unknown_mmio);
}
