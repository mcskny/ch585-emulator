pub mod adc;
pub mod gpio;
pub mod pwm;
pub mod rtc;
pub mod spi;
pub mod sys;
pub mod tmr;
pub mod uart;
pub mod usbhs;

pub mod irq {
    pub const RESET: u8 = 1;
    pub const NMI: u8 = 2;
    pub const EXC: u8 = 3;
    pub const SYSTICK: u8 = 12;
    pub const SWI: u8 = 14;
    pub const TMR0: u8 = 16;
    pub const GPIO_A: u8 = 17;
    pub const GPIO_B: u8 = 18;
    pub const SPI0: u8 = 19;
    pub const BLEB: u8 = 20;
    pub const BLEL: u8 = 21;
    pub const USB: u8 = 22;
    pub const TMR1: u8 = 24;
    pub const TMR2: u8 = 25;
    pub const UART0: u8 = 26;
    pub const UART1: u8 = 27;
    pub const RTC: u8 = 28;
    pub const ADC: u8 = 29;
    pub const I2C: u8 = 30;
    pub const PWMX: u8 = 31;
    pub const TMR3: u8 = 32;
    pub const UART2: u8 = 33;
    pub const UART3: u8 = 34;
    pub const WDOG_BAT: u8 = 35;
    pub const NFC: u8 = 36;
    pub const USB2_DEVICE: u8 = 37;
    pub const USB2_HOST: u8 = 38;
    pub const LED: u8 = 39;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PinKind {
    Power,
    AnalogOnly,
    GpioA(u8),
    GpioB(u8),
}

#[derive(Debug, Clone, Copy)]
pub struct Qfn48PinDef {
    pub pin_num: u8,
    pub name: &'static str,
    pub kind: PinKind,
    pub adc_ch: Option<u8>,
    pub alt_funcs: &'static str,
}

/// Complete CH585M QFN48 pinout table from CH585DS1_EN.PDF Table 1-1.
pub const QFN48_PINS: [Qfn48PinDef; 48] = [
    Qfn48PinDef { pin_num: 1, name: "VDCID", kind: PinKind::Power, adc_ch: None, alt_funcs: "Digital LDO Power" },
    Qfn48PinDef { pin_num: 2, name: "VSW", kind: PinKind::Power, adc_ch: None, alt_funcs: "DC-DC Switch" },
    Qfn48PinDef { pin_num: 3, name: "VIO33", kind: PinKind::Power, adc_ch: None, alt_funcs: "VDD33 / VBAT" },
    Qfn48PinDef { pin_num: 4, name: "PA7", kind: PinKind::GpioA(7), adc_ch: Some(11), alt_funcs: "TXD2/PWM5_/LED6/A11" },
    Qfn48PinDef { pin_num: 5, name: "PA8", kind: PinKind::GpioA(8), adc_ch: Some(12), alt_funcs: "RXD1/LED7/A12" },
    Qfn48PinDef { pin_num: 6, name: "PA9", kind: PinKind::GpioA(9), adc_ch: Some(13), alt_funcs: "TMR0/TXD1/A13" },
    Qfn48PinDef { pin_num: 7, name: "PB9", kind: PinKind::GpioB(9), adc_ch: None, alt_funcs: "NFCI" },
    Qfn48PinDef { pin_num: 8, name: "PB8", kind: PinKind::GpioB(8), adc_ch: None, alt_funcs: "NFCM" },
    Qfn48PinDef { pin_num: 9, name: "PB17", kind: PinKind::GpioB(17), adc_ch: None, alt_funcs: "NFC+" },
    Qfn48PinDef { pin_num: 10, name: "PB16", kind: PinKind::GpioB(16), adc_ch: None, alt_funcs: "NFC-" },
    Qfn48PinDef { pin_num: 11, name: "PB15", kind: PinKind::GpioB(15), adc_ch: None, alt_funcs: "TCK/MISO_/DTR_" },
    Qfn48PinDef { pin_num: 12, name: "PB14", kind: PinKind::GpioB(14), adc_ch: None, alt_funcs: "TIO/MOSI_/PWM10/DSR_" },
    Qfn48PinDef { pin_num: 13, name: "PB13", kind: PinKind::GpioB(13), adc_ch: None, alt_funcs: "U2D+/SCK0_/SCL/TXD1_" },
    Qfn48PinDef { pin_num: 14, name: "PB12", kind: PinKind::GpioB(12), adc_ch: None, alt_funcs: "U2D-/SCS_/SDA/RXD1_" },
    Qfn48PinDef { pin_num: 15, name: "PB11", kind: PinKind::GpioB(11), adc_ch: None, alt_funcs: "UD+/TMR2_" },
    Qfn48PinDef { pin_num: 16, name: "PB10", kind: PinKind::GpioB(10), adc_ch: None, alt_funcs: "UD-/TMR1_" },
    Qfn48PinDef { pin_num: 17, name: "PB7", kind: PinKind::GpioB(7), adc_ch: None, alt_funcs: "TXD0/PWM9" },
    Qfn48PinDef { pin_num: 18, name: "PB6", kind: PinKind::GpioB(6), adc_ch: None, alt_funcs: "RTS/PWM8" },
    Qfn48PinDef { pin_num: 19, name: "PB5", kind: PinKind::GpioB(5), adc_ch: None, alt_funcs: "DTR" },
    Qfn48PinDef { pin_num: 20, name: "PB4", kind: PinKind::GpioB(4), adc_ch: None, alt_funcs: "RXD0/PWM7" },
    Qfn48PinDef { pin_num: 21, name: "PB3", kind: PinKind::GpioB(3), adc_ch: None, alt_funcs: "DCD/PWM9_" },
    Qfn48PinDef { pin_num: 22, name: "PB2", kind: PinKind::GpioB(2), adc_ch: None, alt_funcs: "PWM8_" },
    Qfn48PinDef { pin_num: 23, name: "PB1", kind: PinKind::GpioB(1), adc_ch: None, alt_funcs: "DSR/PWM7_" },
    Qfn48PinDef { pin_num: 24, name: "PB0", kind: PinKind::GpioB(0), adc_ch: None, alt_funcs: "CTS/PWM6" },
    Qfn48PinDef { pin_num: 25, name: "PB23", kind: PinKind::GpioB(23), adc_ch: None, alt_funcs: "TMR0_/TXD2_/PWM11/RST" },
    Qfn48PinDef { pin_num: 26, name: "PB22", kind: PinKind::GpioB(22), adc_ch: None, alt_funcs: "TMR3/RXD2_" },
    Qfn48PinDef { pin_num: 27, name: "PB21", kind: PinKind::GpioB(21), adc_ch: None, alt_funcs: "SCL_/TXD3_" },
    Qfn48PinDef { pin_num: 28, name: "PB20", kind: PinKind::GpioB(20), adc_ch: None, alt_funcs: "SDA_/RXD3_" },
    Qfn48PinDef { pin_num: 29, name: "PB19", kind: PinKind::GpioB(19), adc_ch: None, alt_funcs: "GPIO" },
    Qfn48PinDef { pin_num: 30, name: "PB18", kind: PinKind::GpioB(18), adc_ch: None, alt_funcs: "GPIO" },
    Qfn48PinDef { pin_num: 31, name: "X32MO", kind: PinKind::AnalogOnly, adc_ch: None, alt_funcs: "32MHz HSE Out" },
    Qfn48PinDef { pin_num: 32, name: "X32MI", kind: PinKind::AnalogOnly, adc_ch: None, alt_funcs: "32MHz HSE In" },
    Qfn48PinDef { pin_num: 33, name: "VINTA", kind: PinKind::Power, adc_ch: None, alt_funcs: "Analog Power" },
    Qfn48PinDef { pin_num: 34, name: "ANT", kind: PinKind::AnalogOnly, adc_ch: None, alt_funcs: "2.4GHz RF Antenna" },
    Qfn48PinDef { pin_num: 35, name: "VDCIA", kind: PinKind::Power, adc_ch: None, alt_funcs: "Analog LDO Power" },
    Qfn48PinDef { pin_num: 36, name: "PA4", kind: PinKind::GpioA(4), adc_ch: Some(0), alt_funcs: "RXD3/LEDC/A0" },
    Qfn48PinDef { pin_num: 37, name: "PA5", kind: PinKind::GpioA(5), adc_ch: Some(1), alt_funcs: "TXD3/LED4/A1" },
    Qfn48PinDef { pin_num: 38, name: "PA6", kind: PinKind::GpioA(6), adc_ch: Some(10), alt_funcs: "RXD2/PWM4_/LED5/A10" },
    Qfn48PinDef { pin_num: 39, name: "PA0", kind: PinKind::GpioA(0), adc_ch: Some(9), alt_funcs: "SCK1/LED0/A9" },
    Qfn48PinDef { pin_num: 40, name: "PA1", kind: PinKind::GpioA(1), adc_ch: Some(8), alt_funcs: "MOSI1/LED1/A8" },
    Qfn48PinDef { pin_num: 41, name: "PA2", kind: PinKind::GpioA(2), adc_ch: Some(7), alt_funcs: "TMR3_/MISO1/RI/LED2/A7" },
    Qfn48PinDef { pin_num: 42, name: "PA3", kind: PinKind::GpioA(3), adc_ch: Some(6), alt_funcs: "LED3/A6" },
    Qfn48PinDef { pin_num: 43, name: "PA15", kind: PinKind::GpioA(15), adc_ch: Some(5), alt_funcs: "MISO/RXD0_/A5" },
    Qfn48PinDef { pin_num: 44, name: "PA14", kind: PinKind::GpioA(14), adc_ch: Some(4), alt_funcs: "MOSI/TXD0_/A4" },
    Qfn48PinDef { pin_num: 45, name: "PA13", kind: PinKind::GpioA(13), adc_ch: Some(3), alt_funcs: "SCK0/PWM5/A3" },
    Qfn48PinDef { pin_num: 46, name: "PA12", kind: PinKind::GpioA(12), adc_ch: Some(2), alt_funcs: "SCS/PWM4/A2" },
    Qfn48PinDef { pin_num: 47, name: "PA11", kind: PinKind::GpioA(11), adc_ch: None, alt_funcs: "X32KO/TMR2" },
    Qfn48PinDef { pin_num: 48, name: "PA10", kind: PinKind::GpioA(10), adc_ch: None, alt_funcs: "X32KI/TMR1" },
];
