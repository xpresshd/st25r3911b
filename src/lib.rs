#![cfg_attr(not(feature = "std"), no_std)]

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
use register::{Register, InterruptFlags};

mod picc;
pub mod command;
pub mod register;

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
}

impl<OPE, CS, INTR, SPICS, DELAY> ST25R3911B<SPICS, CS, INTR, DELAY>
where
    SPICS: SpiWithCustomCS,
    CS: OutputPin<Error = OPE>,
    INTR: InputPin<Error = OPE>,
    DELAY: delay::DelayMs<u16>,
{

    pub fn new(spi_with_custom_cs: SPICS, cs: CS, intr: INTR, delay: DELAY) -> Result<Self, Error<SPICS::SpiError, OPE>> {
        let mut st25r3911b = Self {
            spi_with_custom_cs,
            cs,
            intr,
            delay,
        };
        debug!("New ST25R3911B driver instance");
        st25r3911b.reset()?;


        let identity = st25r3911b.read_register(Register::ICIdentity)?;

        if identity & 0b11111000 != 8 {
            return Err(Error::InvalidDevice);
        }

        // TODO: investigate and write comment
        // st25r3911b.write_register(Register::RegulatedVoltageDefinition, 0xA8)?;

        // st25r3911b.execute_command(Command::CalibrateAntenna)?;

        // st25r3911b.delay.delay_ms(1);
        // let val = st25r3911b.read_register(Register::AntennaCalibration)?;

        // if val & 0x8 != 0 {
        //     return Err(Error::AntennaCalibration);
        // }
        // // Enables oscillator and regulator
        // // Enables receiver operation
        // // Enables RF output
        // st25r3911b.write_register(Register::OperationControl, 0xD0)?;

        // // PM demodulation
        // // st25r3911b.write_register(Register::ConfigurationRegister5, 0b1000_0000)?;
        // st25r3911b.execute_command(Command::Clear)?;

        // st25r3911b.setup_interrupt_mask(InterruptFlags::END_OF_RECEIVE)?;

        Ok(st25r3911b)
    }

    pub fn reset(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.execute_command(Command::SetDefault)
    }

    // /// Sends a REQuest type A to nearby PICCs
    // pub fn reqa(&mut self) -> Result<Option<AtqA>, Error<SPICS::SpiError, OPE>> {
    //     info!("reqa");
    //     self.execute_command(Command::Clear)?;
    //     self.write_register(Register::ConfigurationRegister3, 0x80)?;
    //     self.setup_interrupt_mask(InterruptFlags::END_OF_RECEIVE)?;
    //     self.execute_command(Command::TransmitREQA)?;

    //     self.wait_for_interrupt(5)?;

    //     let fifo_reg = self.read_register(Register::FIFOStatus)?;

    //     if fifo_reg >> 2 == 0b00111111 {
    //         // No PICC in area
    //         return Ok(None);
    //     }
    //     let mut buffer = [0u8; 2];

    //     self.read_fifo(&mut buffer)?;

    //     Ok(Some(AtqA { bytes: buffer }))
    // }

    // /// Sends a Wake UP type A to nearby PICCs
    // pub fn wupa(&mut self) -> Result<Option<AtqA>, Error<SPICS::SpiError, OPE>> {
    //     info!("wupa");
    //     self.setup_interrupt_mask(InterruptFlags::END_OF_RECEIVE)?;
    //     self.execute_command(Command::TransmitWUPA)?;

    //     self.wait_for_interrupt(5)?;

    //     let fifo_reg = self.read_register(Register::FIFOStatus)?;

    //     if fifo_reg >> 2 == 0b00111111 {
    //         // No PICC in area
    //         return Ok(None);
    //     }
    //     let mut buffer = [0u8; 2];

    //     self.read_fifo(&mut buffer)?;

    //     Ok(Some(AtqA { bytes: buffer }))
    // }

    // /// Sends command to enter HALT state
    // pub fn hlta(&mut self) -> Result<(), Error<SPICS::SpiError, OPE>> {
    //     info!("hlta");
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
    //     info!("Select");
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

    //         debug!("Select with cascade {}", cascade_level);
    //         'anticollision: loop {
    //             anticollision_cycle_counter += 1;
    //             debug!("Stating anticollision loop nr {} read uid_bytes {:x?}", anticollision_cycle_counter, uid_bytes);

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
    //                     debug!("Read full response {:?}", fifo_data);
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
    //                     debug!("Read partial response {:?}", fifo_data);

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
    //     info!("Communicate to picc {:x?}", tx_buffer);
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

    // pub fn setup_interrupt_mask(&mut self, flags: InterruptFlags) -> Result<u8, Error<SPICS::SpiError, OPE>> {
    //     // Need to invert bits
    //     self.write_register(Register::MaskInterrupt, !flags.bits())?;
    //     // Clear interrupts
    //     self.read_register(Register::Interrupt)
    // }

    pub fn execute_command(&mut self, command: Command) -> Result<(), Error<SPICS::SpiError, OPE>> {
        debug!("Executing command: {:?}", command);
        self.write(&[command.command_pattern()])
    }

    pub fn write_register(&mut self, reg: Register, val: u8) -> Result<(), Error<SPICS::SpiError, OPE>> {
        debug!("Write register {:?} value: 0b{:08b}", reg, val);
        self.write(&[reg.write_address(), val])
    }

    pub fn read_register(&mut self, reg: Register) -> Result<u8, Error<SPICS::SpiError, OPE>> {
        let mut buffer = [reg.read_address(), 0];

        self.spi_with_custom_cs.with_cs_low(&mut self.cs,|spi| {
            let buffer = spi.transfer(&mut buffer)?;
            debug!("Read register {:?} got value value: 0b{:08b}", reg, buffer[1]);

            Ok(buffer[1])
        }).map_err(Error::SpiWithCS)
    }

    // fn read_fifo<'b>(&mut self, buffer: &'b mut [u8]) -> Result<&'b [u8], Error<SPICS::SpiError, OPE>> {
    //     self.spi_with_custom_cs.with_cs_low(&mut self.cs, move |spi| {
    //         // initiate fifo read
    //         spi.transfer(&mut [0b10111111])?;

    //         let n = buffer.len();
    //         for slot in &mut buffer[..n] {
    //             *slot = spi.transfer(&mut [0])?[0];
    //         }

    //         debug!("Read from fifo: {:x?}", buffer);
    //         Ok(&*buffer)
    //     }).map_err(Error::SpiWithCS)
    // }

    // fn write_fifo(&mut self, bytes: &[u8]) -> Result<(), Error<SPICS::SpiError, OPE>> {
    //     debug!("Write in fifo: {:x?}", bytes);
    //     self.spi_with_custom_cs.with_cs_low(&mut self.cs,|spi| {
    //         // initiate fifo write
    //         spi.transfer(&mut [0b10000000])?;

    //         spi.write(bytes)?;

    //         Ok(())
    //     }).map_err(Error::SpiWithCS)
    // }

    // fn wait_for_interrupt(&mut self, timeout_in_ms: u16) -> Result<InterruptFlags, Error<SPICS::SpiError, OPE>> {
    //     debug!("Wait for interrupt {}ms", timeout_in_ms);
    //     let mut i = 0;
    //     loop {
    //         if self.intr.is_high().map_err(Error::InterruptPin)? {
    //             return Ok(InterruptFlags::from_bits_truncate(
    //                 self.read_register(Register::Interrupt)?,
    //             ));
    //         }

    //         if i >= timeout_in_ms {
    //             break;
    //         }
    //         self.delay.delay_ms(1);
    //         i += 1;
    //     }

    //     Err(Error::InterruptTimeout)
    // }

    fn write(&mut self, bytes: &[u8]) -> Result<(), Error<SPICS::SpiError, OPE>> {
        self.spi_with_custom_cs.with_cs_low(&mut self.cs, |spi| {
            spi.write(bytes)?;

            Ok(())
        }).map_err(Error::SpiWithCS)
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
}
