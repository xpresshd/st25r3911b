#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Register {
    // IO Configuration
    IOConfiguration1 = 0x00,
    IOConfiguration2 = 0x01,
    // Operation control and Mode definition
    OperationControlRegister = 0x02,
    ModeDefinitionRegister = 0x03,
    BitRateDefinitionRegister = 0x04,
    // Configuration
    ISO1443AAndNFC106kbsRegister = 0x05,
    ISO1443BRegister = 0x06,
    ISO1443BAndFeliCaRegister = 0x07,
    StreamModeRegister = 0x08,
    AuxiliaryRegister = 0x09,
    ReceiverConfigurationRegister1 = 0x0A,
    ReceiverConfigurationRegister2 = 0x0B,
    ReceiverConfigurationRegister3 = 0x0C,
    ReceiverConfigurationRegister4 = 0x0D,
    // Timer definition
    MaskReceiveTimerRegister = 0x0E,
    NoResponseTimerRegister1 = 0x0F,
    NoResponseTimerRegister2 = 0x10,
    GeneralPurposeAndNoResponseTimerControlRegister = 0x11,
    GeneralPurposeTimerRegister1 = 0x12,
    GeneralPurposeTimerRegister2 = 0x13,
    // Interrupt and associated reporting
    MainInterruptRegister = 0x14,
    MaskTimerAndNFCInterruptRegister = 0x15,
    MaskErrorAndWakeUpInterruptRegister = 0x16,
    MainInterruptRegister2 = 0x17,
    MaskTimerAndNFCInterruptRegister2 = 0x18,
    ErrorAndWakeUpInterruptRegister = 0x19,
    FIFOStatusRegister1 = 0x1A,
    FIFOStatusRegister2 = 0x1B,
    CollisionDisplayRegister = 0x1C,
    // Definition of transmitted bytes
    NumberOfTransmittedBytesRegister1 = 0x1D,
    NumberOfTransmittedBytesRegister2 = 0x1E,
    // NFCIP bit rate detection display 
    NFCIPBitRateDetectionDisplayRegister = 0x1F,
    // A/D converter output 
    ADConverterOutputRegister = 0x20,
    // Antenna calibration
    AntennaCalibrationControlRegister = 0x21,
    AntennaCalibrationTargetRegister = 0x22,
    AntennaCalibrationDisplayRegister = 0x23,
    // AM modulation depth and Antenna driver
    AMModulationDepthControlRegister = 0x24,
    AMModulationDepthDisplayRegister = 0x25,
    RFOAMModulatedLevelDefinitionRegister = 0x26,
    RFONormalLevelDefinitionRegister = 0x27,
    // External field detector threshold 
    ExternalFieldDetectorThresholdRegister = 0x29,
    // Regulator
    RegulatorVoltageControlRegister = 0x2A,
    RegulatorAndTimerDisplayRegister = 0x2B,
    // Receiver State display
    RSSIDisplayRegister = 0x2C,
    GainReductionStateRegister = 0x2D,
    // Capacitive sensor 
    CapacitiveSensorControlRegister = 0x2E,
    CapacitiveSensorDisplayRegister = 0x2F,
    // Auxiliary display
    AuxiliaryDisplayRegister = 0x30,
    // WakeUp
    WakeUpTimerControlRegister = 0x31,
    AmplitudeMeasurementConfigurationRegister = 0x32,
    AmplitudeMeasurementReferenceRegister = 0x33,
    AmplitudeMeasurementAutoAveragingDisplayRegister = 0x34,
    AmplitudeMeasurementDisplayRegister = 0x35,
    PhaseMeasurementConfigurationRegister = 0x36,
    PhaseMeasurementReferenceRegister = 0x37,
    PhaseMeasurementAutoAveragingDisplayRegister = 0x38,
    PhaseMeasurementDisplayRegister = 0x39,
    CapacitanceMeasurementConfigurationRegister = 0x3A,
    CapacitanceMeasurementReferenceRegister = 0x3B,
    CapacitanceMeasurementAutoAveragingDisplayRegister = 0x3C,
    CapacitanceMeasurementDisplayRegister = 0x3D,
    // IC Identity
    ICIdentity = 0x3F,
}

impl From<Register> for u8 {
    #[inline(always)]
    fn from(variant: Register) -> Self {
        variant as _
    }
}

const R: u8 = 1 << 6;
const W: u8 = 0 << 6;

impl Register {
    pub fn read_address(&self) -> u8 {
        (*self as u8) | R
    }

    pub fn write_address(&self) -> u8 {
        (*self as u8) | W
    }
}

bitflags! {
    pub struct InterruptFlags: u8 {
        const BIT_COLLISION = 0b0000_0001;
        const CRC_ERROR = 0b0000_0010;
        const RECEIVE_DATA_CODING_ERROR = 0b000_0100;
        const END_OF_TRANSMISSION = 0b000_1000;
        const END_OF_RECEIVE = 0b0001_0000;
        const FIFO_WATER_LEVEL = 0b0010_0000;
        const NFC_EVENT = 0b0100_0000;
        const OSCILLATOR_FREQUENCY_STABLE = 0b1000_0000;

        const ALL = 0b1111_1111;
    }
}
