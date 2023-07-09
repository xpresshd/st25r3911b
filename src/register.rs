use bitflags::bitflags;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    MaskMainInterruptRegister = 0x14,
    MaskTimerAndNFCInterruptRegister = 0x15,
    MaskErrorAndWakeUpInterruptRegister = 0x16,
    MainInterruptRegister = 0x17,
    TimerAndNFCInterruptRegister = 0x18,
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

bitflags! {
    #[derive(Debug, Clone, Copy)]
    pub struct InterruptFlags: u32 {
        const MASK_ALL = 0xFFFFFF; // Disable all interrupts
        const ERROR = 0b000_0001;
        const TIMER = 0b000_0010;
        const BIT_COLLISION = 0b000_0100;
        const END_OF_TRANSMISSION = 0b000_1000;
        const END_OF_RECEIVE = 0b0001_0000;
        const START_OF_RECEIVE = 0b0010_0000;
        const FIFO_WATER_LEVEL = 0b0100_0000;
        const OSCILLATOR_FREQUENCY_STABLE = 0b1000_0000;

        // Timer and NFC Interrupt Register
        const INITIATOR_BIT_RATE_WAS_RECOGNIZED = 0b0000_0001 << 8;
        /// An external field was not detected during RF Collision Avoidance,
        /// field was switched on, IRQ is sent after minimum guard time according to NFCIP-1
        const MINIMUM_GUARD_TIME_EXPIRE = 0b0000_0010 << 8;
        /// An external field was detected during RF Collision Avoidance
        const FIELD_COLLISION_DETECTED = 0b0000_0100 << 8;
        const EXTERNAL_FIELD_DROP_BELOW = 0b0000_1000 << 8;
        const EXTERNAL_FIELD_HIGHER_THAN = 0b0001_0000 << 8;
        const GENERAL_TIMER_EXPIRE = 0b0010_0000 << 8;
        const NO_RESPONSE_TIMER_EXPIRE = 0b0100_0000 << 8;
        const TERMINATION_OF_DIRECT_COMMAND = 0b1000_0000 << 8;

        // Error and wake-up interrupt register
        const WAKE_UP_CAPACITANCE_MEASUREMENT = 0b0000_0001 << 16;
        const WAKE_UP_PHASE_MEASUREMENT = 0b0000_0010 << 16;
        const WAKE_UP_AMPLITUDE_MEASUREMENT = 0b0000_0100 << 16;
        const WAKE_UP_TIMER_INTERRUPT = 0b0000_1000 << 16;
        const HARD_FRAMING_ERROR = 0b0001_0000 << 16;
        const SOFT_FRAMING_ERROR = 0b0010_0000 << 16;
        const PARITY_ERROR = 0b0100_0000 << 16;
        const CRC_ERROR = 0b1000_0000 << 16;
    }
}

#[allow(dead_code)]
/// The operation mode of the chip, i.e. the protocol that is used.
pub enum OperationMode {
    /// ISO14443A initiator
    PollNFCA,
    /// ISO14443B initiator
    PollNFCB,
    /// FeliCa™ initiator
    PollNFCF,
    /// NFCA Type 1 Tag initiator
    PollTopaz,
    /// NFCIP1 active communication initiator mode with
    /// NFC Automatic Response RF Collision Avoidance enabled
    PollActiveP2P,
    /// NFCIP1 active communication initiator mode with
    /// NFC Automatic Response RF Collision Avoidance enabled
    ListenActiveP2P,
}

#[allow(dead_code)]
/// The speed to be used for transmission or reception.
pub enum Bitrate {
    /// Carrier frequency / 128 = ~106 kbit/s
    Kb106,
    /// Carrier frequency / 64 = ~212 kbit/s
    Kb212,
    /// Carrier frequency / 32 = ~424 kbit/s
    Kb424,
    /// Carrier frequency / 16 = ~848 kbit/s
    Kb848,
    /// Carrier frequency / 8 = ~1695 kbit/s
    Kb1695,
    /// Carrier frequency / 4 = ~3390 kbit/s
    Kb3390,
    /// Carrier frequency / 2 = ~6780 kbit/s
    Kb6780,
}
