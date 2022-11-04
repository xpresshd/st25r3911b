#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum Command {
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
    /// Presets Rx and Tx configuration based on state of Mode definition register and Bit rate definition register
    AnalogPreset = 0xCC,
    /// Receive after this command is ignored 
    MaskReceiveData = 0xD0,
    /// Receive data following this command is normally processed (this command has priority over internal mask receive timer)
    UnmaskReceiveData = 0xD1,

    /// Amplitude of signal present on RFI inputs is measured. The result is stored in A/D converter output register
    MeasureAmplitude = 0xD3,
    /// Performs gain reduction based on the current noise level
    Squelch = 0xD4,
    /// Clears the current squelch setting and loads the manual gain reduction from Receiver configuration register 1
    ResetRxGain = 0xD5,
    /// Adjusts supply regulators according to the current supply voltage level
    AdjustRegulators = 0xD6,
    /// Starts sequence which activates the TX, measures the modulation depth and adapts it to comply with the specified modulation depth
    CalibrateModulationDepth = 0xD7,
    /// Starts the sequence to adjust parallel capacitances connected to TRIMx_y pins so that the antenna LC tank is in resonance
    CalibrateAntenna = 0xD8,
    /// Measurement of phase difference between the signal on RFO and RFI
    MeasurePhase = 0xD9,
    /// Clears RSSI bits and restarts the measurement
    ClearRSSI = 0xDA,
    /// Amplitude of signal present on RFI inputs is measured, result is stored in A/D converter output register
    TransparentMode = 0xDC,
    /// Calibrates capacitive sensor
    CalibrateCapacitiveSensor = 0xDD,
    /// Perform capacitor sensor measurement 
    MeasureCapacitance = 0xDE,
    /// Measure power supply
    MeasurePowerSupply = 0xDF,
    /// 
    StartGeneralPurposeTimer = 0xE0,
    /// 
    StartWakeUpTimer = 0xE1,
    /// 
    StartMaskReceiveTimer = 0xE2,
    /// 
    StartNoResponseTimer = 0xE3,
    /// Enable/W to test registers
    TestAccess = 0xFC,
}
impl From<Command> for u8 {
    #[inline(always)]
    fn from(variant: Command) -> Self {
        variant as _
    }
}

const C: u8 = (1 << 7) + (1 << 6);

impl Command {
    pub fn command_pattern(&self) -> u8 {
        (*self as u8) | C
    }
}
