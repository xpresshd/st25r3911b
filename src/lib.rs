// #![cfg_attr(not(feature = "std"), no_std)]

extern crate delog;
#[macro_use]
extern crate bitflags;

use embedded_hal as hal;
use hal::blocking::delay;
use hal::blocking::spi;
use hal::digital::v2::InputPin;
use hal::digital::v2::OutputPin;
use hal::prelude::_embedded_hal_blocking_spi_Transfer;
use hal::prelude::_embedded_hal_blocking_spi_Write;

use command::Command;
use register::{InterruptFlags, Register};

use crate::consts::MASK_RECEIVE_TIMER;
use crate::consts::NO_RESPONSE_TIMER;

pub mod command;
mod consts;
mod picc;
pub mod register;
mod utils;

delog::generate_macros!();

#[derive(Debug)]
pub enum SPIOrCSError<E, OPE> {
    SPI(E),
    CS(OPE),
}

pub trait SpiWithCustomCS {
    type Spi: spi::Transfer<u8, Error = Self::SpiError> + spi::Write<u8, Error = Self::SpiError>;
    type SpiError;

    fn with_cs_low<F, T, CS, OPE>(
        &mut self,
        cs: &mut CS,
        f: F,
    ) -> Result<T, SPIOrCSError<Self::SpiError, OPE>>
    where
        F: FnOnce(&mut Self::Spi) -> Result<T, Self::SpiError>,
        CS: OutputPin<Error = OPE>;
}

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
pub struct FifoData<const L: usize> {
    /// The contents of the FIFO buffer
    buffer: [u8; L],
    /// The number of valid bytes in the buffer
    valid_bytes: usize,
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
        dst_valid_bits + (len * 8) as u8
    }
}

pub struct ST25R3911B<SPICS, CS, INTR, DELAY> {
    spi_with_custom_cs: SPICS,
    // Chip select pin
    cs: CS,
    /// Interrupt pin
    intr: INTR,
    delay: DELAY,
    interrupt_mask: u32,
}

impl<OPE, CS, INTR, SPICS, DELAY> ST25R3911B<SPICS, CS, INTR, DELAY>
where
    SPICS: SpiWithCustomCS,
    CS: OutputPin<Error = OPE>,
    INTR: InputPin<Error = OPE>,
    DELAY: delay::DelayMs<u16>,
{
    pub fn new(
        spi_with_custom_cs: SPICS,
        cs: CS,
        intr: INTR,
        delay: DELAY,
    ) -> Result<Self, Error<SPICS::SpiError, OPE>> {
        let mut st25r3911b = Self {
            spi_with_custom_cs,
            cs,
            intr,
            delay,
            interrupt_mask: 0,
        };
        println!("New ST25R3911B driver instance");
        st25r3911b.initialize_chip()?;

        st25r3911b.check_chip_id()?;

        // Set FIFO Water Levels to be used
        st25r3911b.modify_register(Register::IOConfiguration1, 0, 0b0011_0000)?;

        // Always have CRC in FIFO upon reception and Enable External Field Detector
        st25r3911b.modify_register(Register::AuxiliaryRegister, 0, 1 << 6 | 1 << 4)?;

        st25r3911b.calibrate()?;

        Ok(st25r3911b)
    }

    pub fn initialize_chip(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        // reset
        self.reset()?;
        // Set Operation Control Register to default value
        self.write_register(Register::OperationControlRegister, 0)?;
        // Set power supply 3.3V and enable pull downs on miso line
        self.write_register(Register::IOConfiguration2, 0b1001_1000)?;

        // after reset all interrupts are enabled. so disable them at first
        self.disable_interrupts(InterruptFlags::MASK_ALL)?;
        // and clear them, just to be sure...
        self.clear_interrupts()?;

        // Turn on Oscillator
        self.enable_interrupts(InterruptFlags::OSCILLATOR_FREQUENCY_STABLE)?;
        self.modify_register(Register::OperationControlRegister, 0, 1 << 7)?;
        self.wait_for_interrupt(InterruptFlags::OSCILLATOR_FREQUENCY_STABLE, 10)?;
        self.disable_interrupts(InterruptFlags::OSCILLATOR_FREQUENCY_STABLE)?;

        // Make sure Transmitter and Receiver are disabled
        self.modify_register(Register::OperationControlRegister, 1 << 6 | 1 << 3, 0)?;

        self.check_chip_id()?;

        Ok(())
    }

    fn check_chip_id(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        let identity = self.read_register(Register::ICIdentity)?;

        if identity & 0b11111000 != 8 {
            return Err(Error::InvalidDevice);
        }
        Ok(())
    }

    fn calibrate(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.adjust_regulators()?;

        // REMARK: Silicon workaround ST25R3911 Errata #1.5
        // Always run the command Calibrate Antenna twice
        self.calibrate_antenna()?;
        self.calibrate_antenna()?;

        // Adjust the regulators again with the Antenna calibrated
        self.adjust_regulators()?;

        Ok(())
    }

    fn adjust_regulators(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        // Reset logic and set regulated voltages to be defined by result of Adjust Regulators command
        self.modify_register(Register::RegulatorVoltageControlRegister, 0, 1 << 7)?;
        self.modify_register(Register::RegulatorVoltageControlRegister, 1 << 7, 0)?;

        self.execute_command(Command::AdjustRegulators)?;
        Ok(())
    }

    fn calibrate_antenna(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.execute_command(Command::CalibrateAntenna)?;
        Ok(())
    }

    pub fn reset(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.execute_command(Command::SetDefault)
    }

    pub fn field_on(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        // set no collision threshold
        self.modify_register(Register::ExternalFieldDetectorThresholdRegister, 0x0F, 0x03)?;
        // set no peer threshold
        self.modify_register(Register::ExternalFieldDetectorThresholdRegister, 0xF0, 0x30)?;

        let intr_flags =
            InterruptFlags::FIELD_COLLISION_DETECTED | InterruptFlags::MINIMUM_GUARD_TIME_EXPIRE;

        self.enable_interrupts(intr_flags)?;
        self.execute_command(Command::NFCInitialFieldOn)?;
        let intr_res = self.wait_for_interrupt(intr_flags, 10);
        self.disable_interrupts(intr_flags)?;

        let intr = intr_res?;
        if intr.contains(InterruptFlags::MINIMUM_GUARD_TIME_EXPIRE) {
            // Also enable Receiver
            self.modify_register(Register::OperationControlRegister, 0, 1 << 6 | 1 << 3)?;

            // Wait specific Guard Time when listener is exposed to an Unmodulated Carrier.
            self.delay.delay_ms(5);
            return Ok(());
        }
        Err(Error::FailedToTurnOnField)
    }

    /// Sends a REQuest type A to nearby PICCs
    pub fn reqa(&mut self) -> Result<Option<AtqA>, Error<SPICS::SpiError, OPE>> {
        println!("reqa");

        // TODO: make sure tx_en on Operation control register is on
        self.read_register(Register::OperationControlRegister)?;

        // Enable anti collision to recognize collision in first byte of SENS_REQ
        self.modify_register(Register::ISO1443AAndNFC106kbsRegister, 0, 0b0000_0001)?;

        // Disable CRC while receiving since ATQA has no CRC included
        self.modify_register(Register::AuxiliaryRegister, 0, 1 << 7)?;

        // Set time when RX is not active after transmission
        // self.write_register(
        //     Register::MaskReceiveTimerRegister,
        //     utils::fc_to_64fc(MASK_RECEIVE_TIMER) as u8,
        // )?;

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

        // TODO: Check which command to run
        self.execute_command(Command::TransmitREQA)?;

        let intr_res = self.wait_for_interrupt(interrupt_flags, 2);
        self.disable_interrupts(interrupt_flags)?;
        let intr = intr_res?;

        println!("intr: {:?}", intr);

        let fifo_reg = self.read_register(Register::FIFOStatusRegister1)?;
        let fifo_reg2 = self.read_register(Register::FIFOStatusRegister2)?;

        if fifo_reg == 0b11111111 {
            // No PICC in area
            return Ok(None);
        }
        let mut buffer = [0u8; 2];

        self.read_fifo(&mut buffer)?;

        Ok(Some(AtqA { bytes: buffer }))
    }

    /// Sends a Wake UP type A to nearby PICCs
    pub fn wupa(&mut self) -> Result<Option<AtqA>, Error<SPICS::SpiError, OPE>> {
        println!("wupa");
        // self.setup_interrupt_mask(InterruptFlags::END_OF_RECEIVE)?;
        // self.execute_command(Command::TransmitWUPA)?;

        // self.wait_for_interrupt(5)?;

        let fifo_reg = self.read_register(Register::FIFOStatusRegister1)?;

        if fifo_reg == 0b11111111 {
            // No PICC in area
            return Ok(None);
        }
        let mut buffer = [0u8; 2];

        self.read_fifo(&mut buffer)?;

        Ok(Some(AtqA { bytes: buffer }))
    }

    // /// Sends command to enter HALT state
    // pub fn hlta(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
    //     println!("hlta");
    //     // The standard says:
    //     //   If the PICC responds with any modulation during a period of 1 ms
    //     //   after the end of the frame containing the HLTA command,
    //     //   this response shall be interpreted as 'not acknowledge'.
    //     // We interpret that this way: Only Error::Timeout is a success.
    //     match self.communicate_to_picc::<0>(&[0x50, 0x00], 0, false, true) {
    //         Err(Error::InterruptTimeout) => Ok(()),
    //         Ok(_) => Err(Error::NotAcknowledged),
    //         Err(e) => Err(e),
    //     }
    // }

    // pub fn select(&mut self) -> Result<Uid, Error<SPICS::SpiError, OPE>> {
    //     println!("Select");
    //     let mut cascade_level: u8 = 0;
    //     let mut uid_bytes: [u8; 10] = [0u8; 10];
    //     let mut uid_idx: usize = 0;
    //     let sak = 'cascade: loop {
    //         let cmd = match cascade_level {
    //             0 => picc::Command::SelCl1,
    //             1 => picc::Command::SelCl2,
    //             2 => picc::Command::SelCl3,
    //             _ => unreachable!(),
    //         };
    //         let mut known_bits = 0;
    //         let mut tx = [0u8; 9];
    //         tx[0] = cmd as u8;
    //         let mut anticollision_cycle_counter = 0;

    //         println!("Select with cascade {}", cascade_level);
    //         'anticollision: loop {
    //             anticollision_cycle_counter += 1;
    //             println!("Stating anticollision loop nr {} read uid_bytes {:x?}", anticollision_cycle_counter, uid_bytes);

    //             if anticollision_cycle_counter > 32 {
    //                 return Err(Error::AntiCollisionMaxLoopsReached);
    //             }
    //             let tx_last_bits = known_bits % 8;
    //             let tx_bytes = 2 + known_bits / 8;
    //             let end = tx_bytes as usize + if tx_last_bits > 0 { 1 } else { 0 };
    //             tx[1] = (tx_bytes << 4) + tx_last_bits;

    //             // Tell transceive the only send `tx_last_bits` of the last byte
    //             // and also to put the first received bit at location `tx_last_bits`.
    //             // This makes it easier to append the received bits to the uid (in `tx`).
    //             match self.communicate_to_picc::<5>(&tx[0..end], tx_last_bits, true, false) {
    //                 Ok(fifo_data) => {
    //                     fifo_data.copy_bits_to(&mut tx[2..=6], known_bits);
    //                     println!("Read full response {:?}", fifo_data);
    //                     break 'anticollision;
    //                 }
    //                 Err(Error::Collision) => {
    //                     let coll_reg = self.read_register(Register::Collision)?;

    //                     let bytes_before_coll = ((coll_reg >> 4) & 0b1111) - 2;
    //                     let bits_before_coll = (coll_reg >> 1) & 0b111;

    //                     let coll_pos = bytes_before_coll * 8 + bits_before_coll + 1;

    //                     if coll_pos < known_bits || coll_pos > 8 * 9 {
    //                         // No progress
    //                         return Err(Error::Collision);
    //                     }

    //                     let fifo_data = self.fifo_data::<5>()?;
    //                     println!("Read partial response {:?}", fifo_data);

    //                     fifo_data.copy_bits_to(&mut tx[2..=6], known_bits);
    //                     known_bits = coll_pos;

    //                     // Set the bit of collision position to 1
    //                     let count = known_bits % 8;
    //                     let check_bit = (known_bits - 1) % 8;
    //                     let index: usize =
    //                         1 + (known_bits / 8) as usize + if count != 0 { 1 } else { 0 };
    //                     // TODO safe check that index is in range
    //                     tx[index] |= 1 << check_bit;
    //                 }
    //                 Err(e) => return Err(e),
    //             }
    //         }

    //         // send select
    //         tx[1] = 0x70; // NVB: 7 valid bytes
    //         tx[6] = tx[2] ^ tx[3] ^ tx[4] ^ tx[5]; // BCC

    //         let rx = self.communicate_to_picc::<1>(&tx[0..7], 0, false, true)?;
    //         // println!("rx {:?}", rx);

    //         let sak = picc::Sak::from(rx.buffer[0]);

    //         if !sak.is_complete() {
    //             uid_bytes[uid_idx..uid_idx + 3].copy_from_slice(&tx[3..6]);
    //             uid_idx += 3;
    //             cascade_level += 1;
    //         } else {
    //             uid_bytes[uid_idx..uid_idx + 4].copy_from_slice(&tx[2..6]);
    //             break 'cascade sak;
    //         }
    //     };

    //     match cascade_level {
    //         0 => Ok(Uid::Single(GenericUid {
    //             bytes: uid_bytes[0..4].try_into().unwrap(),
    //             sak,
    //         })),
    //         1 => Ok(Uid::Double(GenericUid {
    //             bytes: uid_bytes[0..7].try_into().unwrap(),
    //             sak,
    //         })),
    //         2 => Ok(Uid::Triple(GenericUid {
    //             bytes: uid_bytes,
    //             sak,
    //         })),
    //         _ => unreachable!(),
    //     }
    // }

    // /// Sends a Wake UP type A to nearby PICCs
    // pub fn communicate_to_picc<const RX: usize>(
    //     &mut self,
    //     // the data to be sent
    //     tx_buffer: &[u8],
    //     // number of bits in the last byte that will be transmitted
    //     tx_last_bits: u8,
    //     with_anti_collision: bool,
    //     with_crc: bool,
    // ) -> Result<FifoData<RX>, Error<SPICS::SpiError, OPE>> {
    //     println!("Communicate to picc {:x?}", tx_buffer);
    //     self.setup_interrupt_mask(InterruptFlags::END_OF_RECEIVE)?;

    //     self.execute_command(Command::Clear)?;

    //     let full_bytes_num = if tx_last_bits == 0 {
    //         tx_buffer.len()
    //     } else {
    //         tx_buffer.len() - 1
    //     };

    //     let flags = (full_bytes_num << 6)
    //         + (((tx_last_bits & 0x7) << 3) as usize)
    //         + (with_anti_collision as usize);

    //     self.write_register(Register::NumberOfTransmittedBytes0, flags as u8)?;
    //     self.write_register(
    //         Register::NumberOfTransmittedBytes1,
    //         (full_bytes_num >> 2) as u8,
    //     )?;

    //     // Enable AGC (Useful in case the transponder is close to the reader)
    //     self.write_register(Register::ReceiverConfiguration, 0x80)?;

    //     if with_crc {
    //         self.write_register(Register::ConfigurationRegister3, 0x0)?;
    //     } else {
    //         self.write_register(Register::ConfigurationRegister3, 0x80)?;
    //     }

    //     self.write_fifo(tx_buffer)?;

    //     if with_crc {
    //         self.execute_command(Command::TransmitWithCRC)?;
    //     } else {
    //         self.execute_command(Command::TransmitWithoutCRC)?;
    //     }

    //     let intr = self.wait_for_interrupt(5)?;

    //     if intr.contains(InterruptFlags::BIT_COLLISION) {
    //         return Err(Error::Collision);
    //     }

    //     self.fifo_data()
    // }

    // fn fifo_data<const RX: usize>(&mut self) -> Result<FifoData<RX>, Error<SPICS::SpiError, OPE>> {
    //     let mut buffer = [0u8; RX];
    //     let mut valid_bytes: usize = 0;

    //     if RX > 0 {
    //         let fifo_status = self.read_register(Register::FIFOStatus)?;

    //         valid_bytes = (fifo_status >> 2) as usize;
    //         if valid_bytes > RX {
    //             return Err(Error::NoRoom);
    //         }
    //         if valid_bytes > 0 {
    //             self.read_fifo(&mut buffer[0..valid_bytes])?;
    //         }
    //     }

    //     Ok(FifoData {
    //         buffer,
    //         valid_bytes,
    //     })
    // }

    pub fn clear_interrupts(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.read_register(Register::MainInterruptRegister)?;
        self.read_register(Register::TimerAndNFCInterruptRegister)?;
        self.read_register(Register::ErrorAndWakeUpInterruptRegister)?;
        Ok(())
    }

    pub fn enable_interrupts(
        &mut self,
        flags: InterruptFlags,
    ) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.modify_interrupt(flags.bits(), 0)
    }

    pub fn disable_interrupts(
        &mut self,
        flags: InterruptFlags,
    ) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.modify_interrupt(0, flags.bits())
    }

    fn modify_interrupt(
        &mut self,
        clr_mask: u32,
        set_mask: u32,
    ) -> Result<(), Error<SPICS::SpiError, OPE>> {
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
    ) -> Result<(), Error<SPICS::SpiError, OPE>> {
        let mut value = self.read_register(reg)?;
        value &= !clr_mask;
        value |= set_mask;
        self.write_register(reg, value)?;
        Ok(())
    }

    pub fn execute_command(&mut self, command: Command) -> Result<(), Error<SPICS::SpiError, OPE>> {
        println!("Executing command: {:?}", command);
        self.write(&[command.command_pattern()])
    }

    pub fn write_register(
        &mut self,
        reg: Register,
        val: u8,
    ) -> Result<(), Error<SPICS::SpiError, OPE>> {
        println!("Write register {:?} value: 0b{:08b}", reg, val);
        self.write(&[reg.write_address(), val])
    }

    pub fn read_register(&mut self, reg: Register) -> Result<u8, Error<SPICS::SpiError, OPE>> {
        let mut buffer = [reg.read_address(), 0];

        self.spi_with_custom_cs
            .with_cs_low(&mut self.cs, |spi| {
                let buffer = spi.transfer(&mut buffer)?;
                println!(
                    "Read register {:?} got value value: 0b{:08b}",
                    reg, buffer[1]
                );

                Ok(buffer[1])
            })
            .map_err(Error::SpiWithCS)
    }

    fn read_fifo<'b>(
        &mut self,
        buffer: &'b mut [u8],
    ) -> Result<&'b [u8], Error<SPICS::SpiError, OPE>> {
        self.spi_with_custom_cs
            .with_cs_low(&mut self.cs, move |spi| {
                // initiate fifo read
                spi.transfer(&mut [0b10111111])?;

                let n = buffer.len();
                for slot in &mut buffer[..n] {
                    *slot = spi.transfer(&mut [0])?[0];
                }

                println!("Read from fifo: {:x?}", buffer);
                Ok(&*buffer)
            })
            .map_err(Error::SpiWithCS)
    }

    fn write_fifo(&mut self, bytes: &[u8]) -> Result<(), Error<SPICS::SpiError, OPE>> {
        println!("Write in fifo: {:x?}", bytes);
        self.spi_with_custom_cs
            .with_cs_low(&mut self.cs, |spi| {
                // initiate fifo write
                spi.transfer(&mut [0b10000000])?;

                spi.write(bytes)?;

                Ok(())
            })
            .map_err(Error::SpiWithCS)
    }

    fn wait_for_interrupt(
        &mut self,
        mask: InterruptFlags,
        timeout_in_ms: u16,
    ) -> Result<InterruptFlags, Error<SPICS::SpiError, OPE>> {
        println!("Wait for interrupt {}ms", timeout_in_ms);
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

    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.spi_with_custom_cs
            .with_cs_low(&mut self.cs, |spi| {
                spi.write(bytes)?;

                Ok(())
            })
            .map_err(Error::SpiWithCS)
    }
}

#[derive(Debug)]
pub enum Error<E, OPE> {
    SpiWithCS(SPIOrCSError<E, OPE>),
    InterruptPin(OPE),

    /// Set when Calibrate antenna sequence was not able to adjust resonance
    AntennaCalibration,

    InterruptTimeout,
    NoRoom,
    Collision,
    Proprietary,
    AntiCollisionMaxLoopsReached,
    IncompleteFrame,
    NotAcknowledged,
    InvalidDevice,
    FailedToTurnOnField,
}
