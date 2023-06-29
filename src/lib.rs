#![cfg_attr(not(feature = "std"), no_std)]

use embedded_hal as hal;

use defmt::debug;
use hal::delay;
use hal::digital::InputPin;
use hal::spi;

use command::Command;
use register::{Bitrate, InterruptFlags, OperationMode, Register};

mod command;
mod picc;
mod register;

/// Answer To reQuest A
pub struct AtqA {
    pub bytes: [u8; 2],
}

#[derive(Hash, Eq, PartialEq)]
pub enum Uid {
    /// Single sized UID, 4 bytes long
    Single(GenericUid<4>),
    /// Double sized UID, 7 bytes long
    Double(GenericUid<7>),
    /// Trip sized UID, 10 bytes long
    Triple(GenericUid<10>),
}

impl Uid {
    pub fn as_bytes(&self) -> &[u8] {
        match &self {
            Uid::Single(u) => u.as_bytes(),
            Uid::Double(u) => u.as_bytes(),
            Uid::Triple(u) => u.as_bytes(),
        }
    }
}

#[derive(Hash, Eq, PartialEq)]
pub struct GenericUid<const T: usize>
where
    [u8; T]: Sized,
{
    /// The UID can have 4, 7 or 10 bytes.
    bytes: [u8; T],
    /// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
    sak: picc::Sak,
}

impl<const T: usize> GenericUid<T> {
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn is_compliant(&self) -> bool {
        self.sak.is_compliant()
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoData<const L: usize> {
    /// The contents of the FIFO buffer
    buffer: [u8; L],
    /// The number of valid bytes in the buffer
    valid_bytes: usize,
    /// The number of valid bits in the last byte
    valid_bits: u8,
}

impl<const L: usize> FifoData<L> {
    /// Copies FIFO data to destination buffer.
    /// Assumes the FIFO data is aligned properly to append directly to the current known bits.
    /// Returns the number of valid bits in the destination buffer after copy.
    pub fn copy_bits_to(&self, dst: &mut [u8], dst_valid_bits: u8) -> u8 {
        if self.valid_bytes == 0 {
            return dst_valid_bits;
        }
        let dst_valid_bytes = dst_valid_bits / 8;
        let dst_valid_last_bits = dst_valid_bits % 8;
        let mask: u8 = 0xFF << dst_valid_last_bits;
        let mut idx = dst_valid_bytes as usize;
        dst[idx] = (self.buffer[0] & mask) | (dst[idx] & !mask);
        idx += 1;
        let len = self.valid_bytes - 1;
        if len > 0 {
            dst[idx..idx + len].copy_from_slice(&self.buffer[1..=len]);
        }
        dst_valid_bits + (len * 8) as u8 + self.valid_bits
    }
}

pub struct St25r3911b<SPI, IRQ, DELAY> {
    spi: SPI,
    intr: IRQ,
    delay: DELAY,
    interrupt_mask: u32,
}

impl<SPI, IRQ, DELAY> St25r3911b<SPI, IRQ, DELAY>
where
    SPI: spi::SpiDevice,
    IRQ: InputPin,
    DELAY: delay::DelayUs,
{
    pub fn new(spi: SPI, intr: IRQ, delay: DELAY) -> Result<Self, Error<SPI::Error, IRQ::Error>> {
        let mut st25r3911b = Self {
            spi,
            intr,
            delay,
            interrupt_mask: 0,
        };
        debug!("New ST25R3911B driver instance");
        st25r3911b.initialize_chip()?;

        let silicon_rev = st25r3911b.check_chip_id()?;
        defmt::info!("With silicon revision {=u8:x}", silicon_rev);

        // Set FIFO Water Levels to be used
        st25r3911b.modify_register(Register::IOConfiguration1, 0, 0b0011_0000)?;

        // Always have CRC in FIFO upon reception and Enable External Field Detector
        st25r3911b.modify_register(Register::AuxiliaryRegister, 0, 1 << 6 | 1 << 4)?;

        st25r3911b.calibrate()?;

        Ok(st25r3911b)
    }

    pub fn initialize_chip(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        // reset
        self.reset()?;
        // Set Operation Control Register to default value
        self.write_register(Register::OperationControlRegister, 0)?;
        // Enable pull downs on miso line
        self.write_register(Register::IOConfiguration2, 0b0001_1000)?;

        // after reset all interrupts are enabled. so disable them at first
        self.disable_interrupts(InterruptFlags::MASK_ALL)?;
        // and clear them, just to be sure...
        self.clear_interrupts()?;

        // Turn on Oscillator
        self.enable_interrupts(InterruptFlags::OSCILLATOR_FREQUENCY_STABLE)?;
        self.modify_register(Register::OperationControlRegister, 0, 1 << 7)?;
        self.wait_for_interrupt(InterruptFlags::OSCILLATOR_FREQUENCY_STABLE, 10)?;
        self.disable_interrupts(InterruptFlags::OSCILLATOR_FREQUENCY_STABLE)?;

        // TODO: either let user specify or measure with `MeasurePowerSupply` direct command.
        // Set power supply voltage range
        // self.modify_register(Register::IOConfiguration2, 1 << 7, ...)?;

        // Make sure Transmitter and Receiver are disabled
        self.modify_register(Register::OperationControlRegister, 1 << 6 | 1 << 3, 0)?;

        Ok(())
    }

    /// Configure reader for use in ISO14443A mode
    pub fn iso14443a(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        // Enable ISO14443A
        self.set_mode(OperationMode::PollNFCA)?;

        // Set bit rate to 106 kbits/s
        self.set_bitrate(Bitrate::Kb106, Bitrate::Kb106)?;

        // Presets RX and TX configuration
        self.execute_command(Command::AnalogPreset)?;

        Ok(())
    }

    /// Read the IC identity register and verify this is a ST25R3911B.
    /// Returns the silicon revision or an `InvalidDevice` error.
    pub fn check_chip_id(&mut self) -> Result<u8, Error<SPI::Error, IRQ::Error>> {
        let identity = self.read_register(Register::ICIdentity)?;

        if identity & 0b1111_1000 != 8 {
            return Err(Error::InvalidDevice);
        }
        Ok(identity & 0b0000_0111)
    }

    fn calibrate(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        self.adjust_regulators()?;

        // REMARK: Silicon workaround ST25R3911 Errata #1.5
        // Always run the command Calibrate Antenna twice
        self.calibrate_antenna()?;
        self.calibrate_antenna()?;

        // Adjust the regulators again with the Antenna calibrated
        self.adjust_regulators()?;

        Ok(())
    }

    fn adjust_regulators(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        // Reset logic and set regulated voltages to be defined by result of Adjust Regulators cmd
        self.modify_register(Register::RegulatorVoltageControlRegister, 0, 1 << 7)?;
        self.modify_register(Register::RegulatorVoltageControlRegister, 1 << 7, 0)?;

        self.execute_command(Command::AdjustRegulators)?;
        Ok(())
    }

    fn calibrate_antenna(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        self.execute_command(Command::CalibrateAntenna)?;
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        self.execute_command(Command::SetDefault)
    }

    fn set_mode(&mut self, mode: OperationMode) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        let reg_val = match mode {
            OperationMode::PollNFCA => 0b0000_1000,
            OperationMode::PollNFCB => 0b0001_0000,
            OperationMode::PollNFCF => 0b0001_1000,
            OperationMode::PollTopaz => 0b0010_0000,
            OperationMode::PollActiveP2P => 0b0000_0001,
            OperationMode::ListenActiveP2P => 0b1000_1001,
        };

        self.write_register(Register::ModeDefinitionRegister, reg_val)
    }

    fn set_bitrate(
        &mut self,
        tx_bitrate: Bitrate,
        rx_bitrate: Bitrate,
    ) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        let tx_val = match tx_bitrate {
            Bitrate::Kb106 => 0b0000_0000,
            Bitrate::Kb212 => 0b0001_0000,
            Bitrate::Kb424 => 0b0010_0000,
            Bitrate::Kb848 => 0b0011_0000,
            Bitrate::Kb1695 => 0b0100_0000,
            Bitrate::Kb3390 => 0b0101_0000,
            Bitrate::Kb6780 => 0b0110_0000,
        };
        let rx_val = match rx_bitrate {
            Bitrate::Kb106 => 0b0000_0000,
            Bitrate::Kb212 => 0b0000_0001,
            Bitrate::Kb424 => 0b0000_0010,
            Bitrate::Kb848 => 0b0000_0011,
            Bitrate::Kb1695 => 0b0000_0100,
            Bitrate::Kb3390 => 0b0000_0101,
            Bitrate::Kb6780 => 0b0000_0110,
        };

        self.write_register(Register::BitRateDefinitionRegister, tx_val | rx_val)
    }

    pub fn field_on(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        // set recommended threshold
        self.modify_register(Register::ExternalFieldDetectorThresholdRegister, 0x0F, 0x07)?;
        // set no peer threshold
        self.modify_register(Register::ExternalFieldDetectorThresholdRegister, 0xF0, 0x30)?;

        self.delay.delay_ms(5);

        let intr_flags =
            InterruptFlags::FIELD_COLLISION_DETECTED | InterruptFlags::MINIMUM_GUARD_TIME_EXPIRE;

        self.enable_interrupts(intr_flags)?;
        self.execute_command(Command::NFCInitialFieldOn)?;
        let intr_res = self.wait_for_interrupt(intr_flags, 10);
        self.disable_interrupts(intr_flags)?;

        let intr = intr_res?;
        debug!("intr: {:?}", defmt::Debug2Format(&intr));
        if intr.contains(InterruptFlags::MINIMUM_GUARD_TIME_EXPIRE) {
            // Also enable Receiver
            self.modify_register(Register::OperationControlRegister, 0, 1 << 6 | 1 << 3)?;

            // Wait specific Guard Time when listener is exposed to an Unmodulated Carrier.
            self.delay.delay_ms(5);
            return Ok(());
        }
        Err(Error::FailedToTurnOnField)
    }

    /// Sends command to enter HALT state
    pub fn hlta(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        let buffer: [u8; 2] = [picc::Command::HLTA as u8, 0];

        // The standard says:
        //   If the PICC responds with any modulation during a period of 1 ms
        //   after the end of the frame containing the HLTA command,
        //   this response shall be interpreted as 'not acknowledge'.
        // We interpret that this way: Only Error::Timeout is a success.
        match self.anticollision_transmit::<0>(&buffer, 2, 0, false) {
            Err(Error::Timeout) => Ok(()),
            Ok(_) => Err(Error::Nak),
            Err(e) => Err(e),
        }
    }

    /// Sends a Wake UP type A to nearby PICCs
    pub fn wupa(&mut self) -> Result<Option<AtqA>, Error<SPI::Error, IRQ::Error>> {
        debug!("wupa");

        self.process_reqa_wupa(Command::TransmitWUPA)
    }

    /// Sends a REQuest type A to nearby PICCs
    pub fn reqa(&mut self) -> Result<Option<AtqA>, Error<SPI::Error, IRQ::Error>> {
        debug!("reqa");

        self.process_reqa_wupa(Command::TransmitREQA)
    }

    fn process_reqa_wupa(
        &mut self,
        cmd: Command,
    ) -> Result<Option<AtqA>, Error<SPI::Error, IRQ::Error>> {
        debug!("reqa");

        // Enable anti collision to recognize collision in first byte of SENS_REQ
        self.modify_register(Register::ISO1443AAndNFC106kbsRegister, 0, 0b0000_0001)?;

        // Disable CRC while receiving since ATQA has no CRC included and use long range
        self.modify_register(Register::AuxiliaryRegister, 0, 1 << 7 | 1)?;

        // Set time when RX is not active after transmission
        // self.write_register(
        //     Register::MaskReceiveTimerRegister,
        //     utils::fc_to_64fc(MASK_RECEIVE_TIMER) as u8,
        // )?;

        // Set time before it RX should be detected
        let no_response_timer_fc = 35 + 9; // Step in 0.3 ms
        self.modify_register(
            Register::GeneralPurposeAndNoResponseTimerControlRegister,
            0b0000_0011,
            0,
        )?;

        self.write_register(
            Register::NoResponseTimerRegister1,
            (no_response_timer_fc >> 8) as u8,
        )?;
        self.write_register(
            Register::NoResponseTimerRegister2,
            no_response_timer_fc as u8,
        )?;

        // Prepare transmission
        // Clear EMVCo mode
        self.modify_register(
            Register::GeneralPurposeAndNoResponseTimerControlRegister,
            0b0000_0010,
            0,
        )?;
        // Clear FIFO
        self.execute_command(Command::Clear)?;
        // Disable all interrupts
        self.disable_interrupts(InterruptFlags::MASK_ALL)?;
        self.clear_interrupts()?;
        // Reset RX Gain
        self.execute_command(Command::ResetRxGain)?;
        // End prepare transmission

        let interrupt_flags = InterruptFlags::START_OF_RECEIVE
            | InterruptFlags::END_OF_RECEIVE
            | InterruptFlags::END_OF_TRANSMISSION
            | InterruptFlags::BIT_COLLISION
            | InterruptFlags::TIMER
            | InterruptFlags::ERROR
            | InterruptFlags::NO_RESPONSE_TIMER_EXPIRE
            | InterruptFlags::GENERAL_TIMER_EXPIRE
            | InterruptFlags::CRC_ERROR
            | InterruptFlags::PARITY_ERROR
            | InterruptFlags::SOFT_FARMING_ERROR
            | InterruptFlags::HARD_FARMING_ERROR;

        // Enable TX and RX interrupts
        self.enable_interrupts(interrupt_flags)?;

        // Clear nbtx bits before sending WUPA/REQA - otherwise ST25R3911 will report parity error
        self.write_register(Register::NumberOfTransmittedBytesRegister2, 0)?;

        self.execute_command(cmd)?;

        let intr_res = self.wait_for_interrupt(interrupt_flags, 2);
        self.disable_interrupts(interrupt_flags)?;
        let intr = intr_res?;

        debug!("intr: {:?}", defmt::Debug2Format(&intr));

        // Start of TransceiveRx

        // Only raise Timeout if NRE is detected with no Rx Start (NRT EMV mode)
        if intr.contains(InterruptFlags::NO_RESPONSE_TIMER_EXPIRE)
            && !intr.contains(InterruptFlags::START_OF_RECEIVE)
        {
            return Ok(None);
        }

        // Only raise Link Loss if EOF is detected with no Rx Start
        if intr.contains(InterruptFlags::EXTERNAL_FIELD_DROP_BELOW)
            && !intr.contains(InterruptFlags::START_OF_RECEIVE)
        {
            return Err(Error::LinkLoss);
        }
        // If dont have end of receive and start of receive, throw io error
        if !intr.contains(InterruptFlags::END_OF_RECEIVE)
            || !intr.contains(InterruptFlags::START_OF_RECEIVE)
        {
            return Err(Error::IOError);
        }

        // Error check part
        if intr.contains(InterruptFlags::HARD_FARMING_ERROR)
            || intr.contains(InterruptFlags::SOFT_FARMING_ERROR)
        {
            return Err(Error::FarmingError);
        }
        if intr.contains(InterruptFlags::PARITY_ERROR) {
            return Err(Error::ParityError);
        }
        if intr.contains(InterruptFlags::CRC_ERROR) {
            return Err(Error::CRCError);
        }

        let fifo_reg1 = self.read_register(Register::FIFOStatusRegister1)?;
        let fifo_reg2 = self.read_register(Register::FIFOStatusRegister2)?;

        // Check if the reception ends with an incomplete byte (residual bits)
        if fifo_reg2 & (7 << 1 | 1 << 4) != 0 {
            return Err(Error::FifoIncompleteByte);
        }

        // Check if the reception ends with missing parity bit
        if fifo_reg2 & (1 << 0) != 0 {
            return Err(Error::FarmingError);
        }

        // Read data
        let bytes_total = fifo_reg1;

        if bytes_total != 2 {
            // No PICC in area
            return Ok(None);
        }

        let mut buffer = [0u8; 2];

        self.read_fifo(&mut buffer)?;

        Ok(Some(AtqA { bytes: buffer }))
    }

    fn anticollision_transmit<const RX: usize>(
        &mut self,
        tx_buf: &[u8],
        tx_bytes: usize,
        tx_bits: u8,
        without_crc: bool,
    ) -> Result<FifoData<RX>, Error<SPI::Error, IRQ::Error>> {
        match without_crc {
            true => {
                self.modify_register(Register::ISO1443AAndNFC106kbsRegister, 0, 1 << 0)?;
                self.modify_register(Register::AuxiliaryRegister, 0, 1 << 7)?;
            }
            false => {
                self.modify_register(Register::ISO1443AAndNFC106kbsRegister, 1 << 0, 0)?;
                self.modify_register(Register::AuxiliaryRegister, 1 << 7, 0)?;
            }
        }
        // Enable long range
        self.modify_register(Register::AuxiliaryRegister, 0, 1)?;

        // Set time before it RX should be detected
        let no_response_timer_fc = 35 + 9; // utils::fc_to_64fc(NO_RESPONSE_TIMER * 10);
        self.modify_register(
            Register::GeneralPurposeAndNoResponseTimerControlRegister,
            0b0000_0011,
            0,
        )?;

        self.write_register(
            Register::NoResponseTimerRegister1,
            (no_response_timer_fc >> 8) as u8,
        )?;
        self.write_register(
            Register::NoResponseTimerRegister2,
            no_response_timer_fc as u8,
        )?;

        // Prepare transmission
        // Clear EMVCo mode
        self.modify_register(
            Register::GeneralPurposeAndNoResponseTimerControlRegister,
            0b0000_0010,
            0,
        )?;
        // Clear FIFO
        self.execute_command(Command::Clear)?;
        // Disable all interrupts
        self.disable_interrupts(InterruptFlags::MASK_ALL)?;
        self.clear_interrupts()?;
        // Reset RX Gain
        self.execute_command(Command::ResetRxGain)?;

        let interrupt_flags = InterruptFlags::END_OF_TRANSMISSION
            | InterruptFlags::BIT_COLLISION
            | InterruptFlags::START_OF_RECEIVE
            | InterruptFlags::END_OF_RECEIVE
            | InterruptFlags::NO_RESPONSE_TIMER_EXPIRE;

        // Enable TX and RX interrupts
        self.enable_interrupts(interrupt_flags)?;

        self.write_fifo(tx_buf)?;

        // Set number of complete bytes
        self.modify_register(
            Register::NumberOfTransmittedBytesRegister2,
            0xF8,
            (tx_bytes << 3) as u8 & 0xF8,
        )?;
        let msb_val = (tx_bytes >> 0x5) as u8;
        self.write_register(Register::NumberOfTransmittedBytesRegister1, msb_val)?;
        // End Set number of complete bytes

        // Set number of incomplete bit in last byte
        self.modify_register(
            Register::NumberOfTransmittedBytesRegister2,
            0x07,
            tx_bits & 0x07,
        )?;
        // End Set number of incomplete bit in last byte

        match without_crc {
            true => {
                self.execute_command(Command::TransmitWithoutCRC)?;
            }
            false => {
                self.execute_command(Command::TransmitWithCRC)?;
            }
        }
        let intr_res = self.wait_for_interrupt(interrupt_flags, 2);
        self.disable_interrupts(interrupt_flags)?;
        let intr = intr_res?;

        debug!("intr: {:?}", defmt::Debug2Format(&intr));

        if intr.contains(InterruptFlags::BIT_COLLISION) {
            return Err(Error::Collision);
        }

        let fifo_reg2 = self.read_register(Register::FIFOStatusRegister2)?;
        // Check if the reception ends with an incomplete byte (residual bits)
        if fifo_reg2 & (7 << 1 | 1 << 4) != 0 {
            return Err(Error::Collision);
        }

        // Only raise Timeout if NRE is detected with no Rx Start (NRT EMV mode)
        if intr.contains(InterruptFlags::NO_RESPONSE_TIMER_EXPIRE)
            && !intr.contains(InterruptFlags::START_OF_RECEIVE)
        {
            return Err(Error::Timeout);
        }

        self.fifo_data()
    }

    pub fn select(&mut self) -> Result<Uid, Error<SPI::Error, IRQ::Error>> {
        debug!("Select");
        let mut cascade_level: u8 = 0;
        let mut uid_bytes: [u8; 10] = [0u8; 10];
        let mut uid_idx: usize = 0;
        let sak = 'cascade: loop {
            let cmd = match cascade_level {
                0 => picc::Command::SelCl1,
                1 => picc::Command::SelCl2,
                2 => picc::Command::SelCl3,
                _ => unreachable!(),
            };
            let mut known_bits = 0;
            let mut tx = [0u8; 9];
            tx[0] = cmd as u8;
            let mut anticollision_cycle_counter = 0;

            debug!("Select with cascade {}", cascade_level);
            'anticollision: loop {
                anticollision_cycle_counter += 1;
                debug!(
                    "Stating anticollision loop nr {} read uid_bytes {=[?]:x}",
                    anticollision_cycle_counter, uid_bytes
                );

                if anticollision_cycle_counter > 32 {
                    return Err(Error::AntiCollisionMaxLoopsReached);
                }
                let tx_last_bits = known_bits % 8;
                let tx_bytes = 2 + known_bits / 8;
                let end = tx_bytes as usize + if tx_last_bits > 0 { 1 } else { 0 };
                tx[1] = (tx_bytes << 4) + tx_last_bits;

                debug!(
                    "known_bits: {}, end: {}, tx_bytes: {}, tx_last_bits: {}",
                    known_bits, end, tx_bytes, tx_last_bits,
                );

                // Tell transmit the only send `tx_last_bits` of the last byte
                // and also to put the first received bit at location `tx_last_bits`.
                // This makes it easier to append the received bits to the uid (in `tx`).
                match self.anticollision_transmit::<5>(
                    &tx[0..end],
                    tx_bytes as usize,
                    tx_last_bits,
                    true,
                ) {
                    Ok(fifo_data) => {
                        fifo_data.copy_bits_to(&mut tx[2..=6], known_bits);
                        debug!("Read full response {:?}", fifo_data);
                        break 'anticollision;
                    }
                    Err(Error::Collision) => {
                        let coll_reg = self.read_register(Register::CollisionDisplayRegister)?;
                        debug!("coll_reg: 0b{:08b}", coll_reg);

                        let bytes_before_coll = ((coll_reg >> 4) & 0b1111) - 2;
                        let bits_before_coll = (coll_reg >> 1) & 0b111;
                        debug!(
                            "bytes_before_coll: {}, bits_before_coll: {}",
                            bytes_before_coll, bits_before_coll
                        );

                        let coll_pos = bytes_before_coll * 8 + bits_before_coll + 1;

                        if coll_pos < known_bits || coll_pos > 8 * 9 {
                            // No progress
                            return Err(Error::Collision);
                        }

                        let fifo_data = self.fifo_data::<5>()?;
                        debug!("Read partial response {:?}", fifo_data);

                        fifo_data.copy_bits_to(&mut tx[2..=6], known_bits);
                        known_bits = coll_pos;

                        // Set the bit of collision position to 1
                        let count = known_bits % 8;
                        let check_bit = (known_bits - 1) % 8;
                        let index: usize =
                            1 + (known_bits / 8) as usize + if count != 0 { 1 } else { 0 };
                        // TODO safe check that index is in range
                        tx[index] |= 1 << check_bit;
                    }
                    Err(Error::Timeout) => {
                        debug!("Timeout anticollision");
                        return Err(Error::Timeout);
                    }
                    Err(e) => return Err(e),
                }
            }

            // send select
            tx[1] = 0x70; // NVB: 7 valid bytes
            tx[6] = tx[2] ^ tx[3] ^ tx[4] ^ tx[5]; // BCC
            let mut retries = 3;
            let rx = loop {
                if retries < 0 {
                    break Err(Error::Timeout);
                }
                retries -= 1;
                match self.anticollision_transmit::<3>(&tx[0..7], 7, 0, false) {
                    Ok(res) => break Ok(res),
                    Err(Error::Timeout) => continue,
                    Err(e) => break Err(e),
                }
            }?;

            let sak = picc::Sak::from(rx.buffer[0]);

            if !sak.is_complete() {
                uid_bytes[uid_idx..uid_idx + 3].copy_from_slice(&tx[3..6]);
                uid_idx += 3;
                cascade_level += 1;
            } else {
                uid_bytes[uid_idx..uid_idx + 4].copy_from_slice(&tx[2..6]);
                break 'cascade sak;
            }
        };

        match cascade_level {
            0 => Ok(Uid::Single(GenericUid {
                bytes: uid_bytes[0..4].try_into().unwrap(),
                sak,
            })),
            1 => Ok(Uid::Double(GenericUid {
                bytes: uid_bytes[0..7].try_into().unwrap(),
                sak,
            })),
            2 => Ok(Uid::Triple(GenericUid {
                bytes: uid_bytes,
                sak,
            })),
            _ => unreachable!(),
        }
    }

    fn fifo_data<const RX: usize>(
        &mut self,
    ) -> Result<FifoData<RX>, Error<SPI::Error, IRQ::Error>> {
        let mut buffer = [0u8; RX];
        let mut valid_bytes: usize = 0;
        let mut valid_bits = 0;

        if RX > 0 {
            let fifo_status = self.read_register(Register::FIFOStatusRegister1)?;
            let fifo_reg2 = self.read_register(Register::FIFOStatusRegister2)?;

            valid_bytes = fifo_status as usize;
            if valid_bytes > RX {
                return Err(Error::FifoNoRoom);
            }
            if valid_bytes > 0 {
                self.read_fifo(&mut buffer[0..valid_bytes])?;

                // Check if the reception ends with an incomplete byte (residual bits)
                if fifo_reg2 & (7 << 1 | 1 << 4) != 0 {
                    valid_bits = (fifo_reg2 & (7 << 1)) >> 1;
                }
            }
        }

        Ok(FifoData {
            buffer,
            valid_bytes,
            valid_bits,
        })
    }

    pub fn clear_interrupts(&mut self) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        self.read_register(Register::MainInterruptRegister)?;
        self.read_register(Register::TimerAndNFCInterruptRegister)?;
        self.read_register(Register::ErrorAndWakeUpInterruptRegister)?;
        Ok(())
    }

    pub fn enable_interrupts(
        &mut self,
        flags: InterruptFlags,
    ) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        self.modify_interrupt(flags.bits(), 0)
    }

    pub fn disable_interrupts(
        &mut self,
        flags: InterruptFlags,
    ) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        self.modify_interrupt(0, flags.bits())
    }

    fn modify_interrupt(
        &mut self,
        clr_mask: u32,
        set_mask: u32,
    ) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        let old_mask = self.interrupt_mask;
        let new_mask = (!old_mask & clr_mask) | (old_mask & clr_mask);

        self.interrupt_mask &= !clr_mask;
        self.interrupt_mask |= set_mask;

        if new_mask & 0xff != 0 {
            self.write_register(
                Register::MaskMainInterruptRegister,
                self.interrupt_mask as u8,
            )?;
        }

        if new_mask >> 8 & 0xff != 0 {
            self.write_register(
                Register::MaskTimerAndNFCInterruptRegister,
                (self.interrupt_mask >> 8) as u8,
            )?;
        }

        if new_mask >> 16 & 0xff != 0 {
            self.write_register(
                Register::MaskErrorAndWakeUpInterruptRegister,
                (self.interrupt_mask >> 16) as u8,
            )?;
        }

        Ok(())
    }

    fn modify_register(
        &mut self,
        reg: Register,
        clr_mask: u8,
        set_mask: u8,
    ) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        let mut value = self.read_register(reg)?;
        value &= !clr_mask;
        value |= set_mask;
        self.write_register(reg, value)?;
        Ok(())
    }

    pub fn execute_command(
        &mut self,
        command: Command,
    ) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        debug!("Executing command: {:?}", command);
        self.spi
            .write(&[command.command_pattern()])
            .map_err(Error::Spi)
    }

    pub fn write_register(
        &mut self,
        reg: Register,
        val: u8,
    ) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        debug!("Write register {:?} value: 0b{:08b}", reg, val);
        self.spi
            .write(&[reg.write_address(), val])
            .map_err(Error::Spi)
    }

    pub fn read_register(&mut self, reg: Register) -> Result<u8, Error<SPI::Error, IRQ::Error>> {
        let address = [reg.read_address()];
        let mut value = [0];

        let mut operations = [
            spi::Operation::Write(&address),
            spi::Operation::Read(&mut value),
        ];
        self.spi.transaction(&mut operations).map_err(Error::Spi)?;
        Ok(value[0])
    }

    fn read_fifo<'b>(
        &mut self,
        buffer: &'b mut [u8],
    ) -> Result<&'b [u8], Error<SPI::Error, IRQ::Error>> {
        let fifo_cmd = [0b10111111];
        let mut operations = [
            spi::Operation::Write(&fifo_cmd),
            spi::Operation::Read(buffer),
        ];
        self.spi.transaction(&mut operations).map_err(Error::Spi)?;
        Ok(buffer)
    }

    fn write_fifo(&mut self, bytes: &[u8]) -> Result<(), Error<SPI::Error, IRQ::Error>> {
        debug!("Write in fifo: {=[?]:x}", bytes);
        let fifo_cmd = [0b10000000];
        let mut operations = [
            spi::Operation::Write(&fifo_cmd),
            spi::Operation::Write(bytes),
        ];
        self.spi.transaction(&mut operations).map_err(Error::Spi)
    }

    fn wait_for_interrupt(
        &mut self,
        mask: InterruptFlags,
        timeout_in_ms: u16,
    ) -> Result<InterruptFlags, Error<SPI::Error, IRQ::Error>> {
        debug!("Wait for interrupt {}ms", timeout_in_ms);
        let mut i = 0;
        let mut interrupt = 0u32;
        loop {
            if self.intr.is_high().map_err(Error::InterruptPin)? {
                interrupt |= self.read_register(Register::MainInterruptRegister)? as u32;
                interrupt |=
                    (self.read_register(Register::TimerAndNFCInterruptRegister)? as u32) << 8;
                interrupt |=
                    (self.read_register(Register::ErrorAndWakeUpInterruptRegister)? as u32) << 16;

                // Match any interrupt
                if interrupt & mask.bits() > 0 {
                    return Ok(InterruptFlags::from_bits_truncate(interrupt));
                }
            }

            if i >= timeout_in_ms {
                break;
            }
            self.delay.delay_ms(1);
            i += 1;
        }

        Err(Error::InterruptTimeout)
    }
}

#[derive(Debug)]
pub enum Error<SPIE, GPIOE> {
    Spi(SPIE),
    InterruptPin(GPIOE),

    /// Set when Calibrate antenna sequence was not able to adjust resonance
    AntennaCalibration,

    InterruptTimeout,
    InvalidDevice,
    FailedToTurnOnField,

    LinkLoss,
    IOError,
    FarmingError,
    CRCError,
    ParityError,
    FifoIncompleteByte,
    FifoNoRoom,
    Collision,
    AntiCollisionMaxLoopsReached,
    Timeout,
    Nak,
}
