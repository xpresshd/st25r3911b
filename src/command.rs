use crate::register::Register;

#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
/// An instruction to the reader to perform a certain action.
/// This operation is a single-byte value starting with two leading '1' bits (`0b11xx_xxxx`).
pub enum DirectCommand {
    /// Puts the st25r3911b in default state (same as after power-up)
    SetDefault = 0xC1,
    /// Stops all activities and clears FIFO
    Clear = 0xC2,
    /// Starts a transmit sequence using automatic CRC generation
    TransmitWithCRC = 0xC4,
    /// Starts a transmit sequence without automatic CRC generation
    TransmitWithoutCRC = 0xC5,
    /// Transmits REQA command (ISO-14443A mode only)
    TransmitREQA = 0xC6,
    /// Transmits WUPA command (ISO-14443A mode only)
    TransmitWUPA = 0xC7,
    /// Performs initial RF collision avoidance and switch on the field
    NFCInitialFieldOn = 0xC8,
    /// Performs response RF collision avoidance and switch on the field
    NFCResponseFieldOn = 0xC9,
    /// Performs response RF collision avoidance with n=0 and switch on the field
    NFCResponseFieldOnWithN0 = 0xCA,
    /// Accepted in NFCIP-1 active communication bit rate detection mode
    GoToNormalNFCMode = 0xCB,
    /// Presets Rx and Tx configuration based on state of Mode definition register
    /// and Bit rate definition register
    AnalogPreset = 0xCC,
    /// Receive after this command is ignored
    MaskReceiveData = 0xD0,
    /// Receive data following this command is normally processed
    /// (this command has priority over internal mask receive timer)
    UnmaskReceiveData = 0xD1,
    /// Amplitude of signal present on RFI inputs is measured.
    /// The result is stored in A/D converter output register
    MeasureAmplitude = 0xD3,
    /// Performs gain reduction based on the current noise level
    Squelch = 0xD4,
    /// Clears the current squelch setting and loads the manual gain reduction
    /// from Receiver configuration register 1
    ResetRxGain = 0xD5,
    /// Adjusts supply regulators according to the current supply voltage level
    AdjustRegulators = 0xD6,
    /// Starts sequence which activates the TX, measures the modulation depth
    /// and adapts it to comply with the specified modulation depth
    CalibrateModulationDepth = 0xD7,
    /// Starts the sequence to adjust parallel capacitances connected to TRIMx_y pins
    /// so that the antenna LC tank is in resonance
    CalibrateAntenna = 0xD8,
    /// Measurement of phase difference between the signal on RFO and RFI
    MeasurePhase = 0xD9,
    /// Clears RSSI bits and restarts the measurement
    ClearRSSI = 0xDA,
    /// Amplitude of signal present on RFI inputs is measured,
    /// result is stored in A/D converter output register
    TransparentMode = 0xDC,
    /// Calibrates capacitive sensor
    CalibrateCapacitiveSensor = 0xDD,
    /// Perform capacitor sensor measurement
    MeasureCapacitance = 0xDE,
    /// Measure power supply
    MeasurePowerSupply = 0xDF,
    /// Start general purpose timer
    StartGeneralPurposeTimer = 0xE0,
    /// Start wake-up timer
    StartWakeUpTimer = 0xE1,
    /// Start mask-receive timer, which blocks the receiver and reception process
    /// for a configured time after the end of transmission.
    StartMaskReceiveTimer = 0xE2,
    /// Start no-response timer, to observe whether a tag response is detected
    /// in a configured time after the end of transmission.
    StartNoResponseTimer = 0xE3,
    /// Enable/W to test registers
    TestAccess = 0xFC,
}
impl From<DirectCommand> for u8 {
    #[inline(always)]
    fn from(variant: DirectCommand) -> Self {
        variant as _
    }
}

impl DirectCommand {
    /// The bits to be sent for this operation
    pub fn pattern(&self) -> u8 {
        (*self as u8) | 0b1100_0000
    }
}


#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// An operation performed on one of the registers.
/// This operation starts with a leading '0' bit (`0b0xxx_xxxx`).
pub enum RegisterOperation {
    /// Set the value of a register
    Write(Register),
    /// Get the value of a register
    Read(Register),
}

impl RegisterOperation {
    /// The bits to be sent for this operation
    pub fn pattern(&self) -> u8 {
        match self {
            Self::Write(w) => (*w as u8) & 0b0011_1111,
            Self::Read(r) => ((*r as u8) & 0b0011_1111) | 0b0100_0000,
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// An operation performed on the 96-byte FIFO,
/// which contains the data received from the picc or to be transmitted to the picc.
/// This operation starts with leading '10' bits (`0b10xx_xxxx`).
pub enum FifoOperation {
    /// Write one or more bytes to the FIFO
    Load,
    /// Read one or more bytes from the FIFO
    Read,
}

impl FifoOperation {
    /// The bits to be sent for this operation
    pub fn pattern(&self) -> u8 {
        match self {
            Self::Load => 0b1000_0000,
            Self::Read => 0b1011_1111,
        }
    }
}
