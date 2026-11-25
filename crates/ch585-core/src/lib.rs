//! Chip-generic WCH CH585M (QingKe V3C RV32IMAC + XW) emulator core library.
//!
//! External boards and attached peripherals plug into [`soc::Ch585Machine`] via
//! [`ext::ExternalDevice`], [`ext::PinAnalogModel`], pin/ADC stimulus, and USB/BLE virtual interfaces.

pub mod ble_stub;
pub mod bus;
pub mod calibration;
pub mod cluster;
pub mod cpu;
pub mod elf;
pub mod ext;
pub mod json;
pub mod logic;
pub mod peripherals;
pub mod pfic;
pub mod profiler;
pub mod report;
pub mod rf;
pub mod soc;
pub mod vcd;

/// Commonly used chip-level types for embedding `ch585-core` as a Rust library dependency.
pub mod prelude {
    pub use crate::ble_stub::{
        BleBondRecord, BleBridge, BleCentralState, BleConnParams, BleHidReport, BleScanResult,
        BleTimestampedEvent,
    };
    pub use crate::calibration::{
        AdcCalibrationModel, AdcChannelGainCal, AdcSettlingModel, CalibrationConfig, TimingModel,
    };
    pub use crate::cluster::{GpioPinRef, GpioWire, MultiChipCluster, UartWire};
    pub use crate::cpu::{CallbackResult, Cpu, PendingCallback, CALLBACK_RET_SENTINEL};
    pub use crate::elf::{ElfError, ElfImage, ElfSymbol};
    pub use crate::ext::{
        compute_pin_voltage_mv, ExternalDevice, InputReadKind, InputReadMask, PinAnalogModel,
        PinChangeEvent, PinContext, PinWatchMask,
    };
    pub use crate::json::{parse_json, JsonValue};
    pub use crate::peripherals::adc::AdcWaveform;
    pub use crate::peripherals::gpio::{ExtPinDrive, ExtPull, PinElectricState};
    pub use crate::peripherals::sys::{CurrentTable, PowerMode, PowerReport};
    pub use crate::profiler::{Profiler, ProfilerReport};
    pub use crate::report::{MachineReport, PerfStats};
    pub use crate::rf::{
        RfDeliveryOutcome, RfMedium, RfMediumConfig, RfMode, RfState, BOUND_STA_SUCCESS,
        BOUND_STA_TIMEOUT, RF_BOUND_ID, RF_STATE_RBU, RF_STATE_RX, RF_STATE_RX_CRCERR,
        RF_STATE_TIMEOUT, RF_STATE_TX_FINISH, STA_DMA_ENABLE, STA_LEN_MASK,
    };
    pub use crate::soc::{Ch585Machine, PinSnapshot, SysBus};
    pub use crate::vcd::{TraceBuffer, VcdConfig};
}
