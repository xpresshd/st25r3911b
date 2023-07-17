//! ST25R3911B register definitions
use bitflags::bitflags;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Register {
    /// *RW* IO Configuration Register 1
    IOConfiguration1 = 0x00,
    /// *RW* IO Configuration Register 2
    IOConfiguration2 = 0x01,
    /// *RW* Operation Control Register
    OperationControlRegister = 0x02,
    /// *RW* Mode Definition Register
    ModeDefinitionRegister = 0x03,
    /// *RW* Bit Rate Definition Register
    BitRateDefinitionRegister = 0x04,
    /// *RW* ISO14443A and NFC 106 kBit/s Settings Register
    ISO1443AAndNFC106kbsRegister = 0x05,
    /// *RW* ISO14443B Settings Register 1
    ISO1443BRegister = 0x06,
    /// *RW* ISO14443B Settings Register 2
    ISO1443BAndFeliCaRegister = 0x07,
    /// *RW* Stream Mode Definition Register
    StreamModeRegister = 0x08,
    /// *RW* Auxiliary Definition Register
    AuxiliaryRegister = 0x09,
    /// *RW* Receiver Configuration Register 1
    ReceiverConfigurationRegister1 = 0x0A,
    /// *RW* Receiver Configuration Register 2
    ReceiverConfigurationRegister2 = 0x0B,
    /// *RW* Receiver Configuration Register 3
    ReceiverConfigurationRegister3 = 0x0C,
    /// *RW* Receiver Configuration Register 4
    ReceiverConfigurationRegister4 = 0x0D,
    /// *RW* Mask Receive Timer Register
    MaskReceiveTimerRegister = 0x0E,
    /// *RW* No-response Timer Register 1
    NoResponseTimerRegister1 = 0x0F,
    /// *RW* No-response Timer Register 2
    NoResponseTimerRegister2 = 0x10,
    /// *RW* General Purpose and No-response Timer Control Register
    GeneralPurposeAndNoResponseTimerControlRegister = 0x11,
    /// *RW* General Purpose Timer Register 1
    GeneralPurposeTimerRegister1 = 0x12,
    /// *RW* General Purpose Timer Register 2
    GeneralPurposeTimerRegister2 = 0x13,
    /// *RW* Mask Main Interrupt Register
    MaskMainInterruptRegister = 0x14,
    /// *RW* Mask Timer and NFC Interrupt Register
    MaskTimerAndNFCInterruptRegister = 0x15,
    /// *RW* Mask Error and Wake-up Interrupt Register
    MaskErrorAndWakeUpInterruptRegister = 0x16,
    /// *R*  Main Interrupt Register
    MainInterruptRegister = 0x17,
    /// *R*  Timer and NFC Interrupt Register
    TimerAndNFCInterruptRegister = 0x18,
    /// *R*  Error and Wake-up Interrupt Register
    ErrorAndWakeUpInterruptRegister = 0x19,
    /// *R*  FIFO RX Status Register 1
    FIFOStatusRegister1 = 0x1A,
    /// *R*  FIFO RX Status Register 2
    FIFOStatusRegister2 = 0x1B,
    /// *R*  Collision Display Register
    CollisionDisplayRegister = 0x1C,
    /// *RW* Number of Transmitted Bytes Register 1
    NumberOfTransmittedBytesRegister1 = 0x1D,
    /// *RW* Number of Transmitted Bytes Register 2
    NumberOfTransmittedBytesRegister2 = 0x1E,
    /// *R*  NFCIP Bit Rate Detection Display Register
    NFCIPBitRateDetectionDisplayRegister = 0x1F,
    /// *R*  A/D Converter Output Register
    ADConverterOutputRegister = 0x20,
    /// *RW* Antenna Calibration Control Register
    AntennaCalibrationControlRegister = 0x21,
    /// *RW* Antenna Calibration Target Register
    AntennaCalibrationTargetRegister = 0x22,
    /// *R*  Antenna Calibration Display Register
    AntennaCalibrationDisplayRegister = 0x23,
    /// *RW* AM Modulation Depth Control Register
    AMModulationDepthControlRegister = 0x24,
    /// *R*  AM Modulation Depth Display Register
    AMModulationDepthDisplayRegister = 0x25,
    /// *RW* RFO AM Modulation (On) Level Definition Register
    RFOAMModulatedLevelDefinitionRegister = 0x26,
    /// *RW* RFO Normal (AM Off) Level Definition Register
    RFONormalLevelDefinitionRegister = 0x27,
    /// *RW* External Field Detector Threshold Register
    ExternalFieldDetectorThresholdRegister = 0x29,
    /// *RW* Regulated Voltage Control Register
    RegulatorVoltageControlRegister = 0x2A,
    /// *R*  Regulator Display Register
    RegulatorAndTimerDisplayRegister = 0x2B,
    /// *R*  RSSI Display Register
    RSSIDisplayRegister = 0x2C,
    /// *R*  Gain Reduction State Register
    GainReductionStateRegister = 0x2D,
    /// *RW* Capacitive Sensor Control Register
    CapacitiveSensorControlRegister = 0x2E,
    /// *R*  Capacitive Sensor Display Register
    CapacitiveSensorDisplayRegister = 0x2F,
    /// *R*  Auxiliary Display Register
    AuxiliaryDisplayRegister = 0x30,
    /// *RW* Wake-up Timer Control Register
    WakeUpTimerControlRegister = 0x31,
    /// *RW* Amplitude Measurement Configuration Register
    AmplitudeMeasurementConfigurationRegister = 0x32,
    /// *RW* Amplitude Measurement Reference Register
    AmplitudeMeasurementReferenceRegister = 0x33,
    /// *R*  Amplitude Measurement Auto Averaging Display Register
    AmplitudeMeasurementAutoAveragingDisplayRegister = 0x34,
    /// *R*  Amplitude Measurement Display Register
    AmplitudeMeasurementDisplayRegister = 0x35,
    /// *RW* Phase Measurement Configuration Register
    PhaseMeasurementConfigurationRegister = 0x36,
    /// *RW* Phase Measurement Reference Register
    PhaseMeasurementReferenceRegister = 0x37,
    /// *R*  Phase Measurement Auto Averaging Display Register
    PhaseMeasurementAutoAveragingDisplayRegister = 0x38,
    /// *R*  Phase Measurement Display Register
    PhaseMeasurementDisplayRegister = 0x39,
    /// *RW* Capacitance Measurement Configuration Register
    CapacitanceMeasurementConfigurationRegister = 0x3A,
    /// *RW* Capacitance Measurement Reference Register
    CapacitanceMeasurementReferenceRegister = 0x3B,
    /// *R*  Capacitance Measurement Auto Averaging Display Register
    CapacitanceMeasurementAutoAveragingDisplayRegister = 0x3C,
    /// *R*  Capacitance Measurement Display Register
    CapacitanceMeasurementDisplayRegister = 0x3D,
    /// *R*  Chip Id: 0 for old silicon, v2 silicon: 0x09
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

        // Main interrupt register

        /// An interrupt in the `Error and wake-up interrupt register` is active.
        const ERROR = 0b000_0001;
        /// An interrupt in the `Timer and NFC interrupt register` is active.
        const TIMER = 0b000_0010;
        /// Bit collision event.
        const BIT_COLLISION = 0b000_0100;
        /// Transmission end event.
        const END_OF_TRANSMISSION = 0b000_1000;
        /// Receive end event.
        const END_OF_RECEIVE = 0b0001_0000;
        /// Receive start event.
        const START_OF_RECEIVE = 0b0010_0000;
        /// Set during receive: FIFO is almost full and has to be read out.
        /// Set during transmit: FIFO is almost empty and requires additional data.
        const FIFO_WATER_LEVEL = 0b0100_0000;
        /// Set after oscillator is started by setting Operation Control register bit en.
        const OSCILLATOR_FREQUENCY_STABLE = 0b1000_0000;

        // Timer and NFC interrupt register

        const INITIATOR_BIT_RATE_WAS_RECOGNIZED = 0b0000_0001 << 8;
        /// An external field was not detected during RF Collision Avoidance,
        /// field was switched on, IRQ is sent after minimum guard time according to NFCIP-1.
        const MINIMUM_GUARD_TIME_EXPIRE = 0b0000_0010 << 8;
        /// An external field was detected during RF Collision Avoidance
        const FIELD_COLLISION_DETECTED = 0b0000_0100 << 8;
        /// Detected external field dropped below target activation level.
        const EXTERNAL_FIELD_DROP_BELOW = 0b0000_1000 << 8;
        /// Detected external field higher than target activation level.
        const EXTERNAL_FIELD_HIGHER_THAN = 0b0001_0000 << 8;
        /// General purpose timer expired
        const GENERAL_TIMER_EXPIRE = 0b0010_0000 << 8;
        /// No-Response timer expired.
        const NO_RESPONSE_TIMER_EXPIRE = 0b0100_0000 << 8;
        /// Direct command finished executing.
        const TERMINATION_OF_DIRECT_COMMAND = 0b1000_0000 << 8;

        // Error and wake-up interrupt register

        /// Result of capacitance measurement was Δcm larger than reference
        const WAKE_UP_CAPACITANCE_MEASUREMENT = 0b0000_0001 << 16;
        /// Result of phase measurement was Δpm larger than reference
        const WAKE_UP_PHASE_MEASUREMENT = 0b0000_0010 << 16;
        /// Result of amplitude measurement was Δam larger than reference
        const WAKE_UP_AMPLITUDE_MEASUREMENT = 0b0000_0100 << 16;
        /// Timeout after execution of *start wake-up timer* command
        const WAKE_UP_TIMER_INTERRUPT = 0b0000_1000 << 16;
        /// Framing error which results in corrupted Rx data
        const HARD_FRAMING_ERROR = 0b0001_0000 << 16;
        /// Framing error which does not result in corrupted Rx data
        const SOFT_FRAMING_ERROR = 0b0010_0000 << 16;
        /// Receive data parity error
        const PARITY_ERROR = 0b0100_0000 << 16;
        /// Receive data CRC error
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
