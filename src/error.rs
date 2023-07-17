/// The error type for this crate.
/// It is generic over the underlying SPI and GPIO error types.
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
    FramingError,
    CRCError,
    ParityError,
    FifoIncompleteByte,
    FifoNoRoom,
    Collision,
    AntiCollisionMaxLoopsReached,
    Timeout,
    Nak,
}

#[cfg(feature = "defmt")]
impl<SPIE, GPIOE> defmt::Format for Error<SPIE, GPIOE> {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::Spi(_) => defmt::write!(f, "Error::Spi"),
            Error::InterruptPin(_) => defmt::write!(f, "Error::InterruptPin"),
            Error::AntennaCalibration => defmt::write!(f, "Error::AntennaCalibration"),
            Error::InterruptTimeout => defmt::write!(f, "Error::InterruptTimeout"),
            Error::InvalidDevice => defmt::write!(f, "Error::InvalidDevice"),
            Error::FailedToTurnOnField => defmt::write!(f, "Error::FailedToTurnOnField"),
            Error::LinkLoss => defmt::write!(f, "Error::LinkLoss"),
            Error::IOError => defmt::write!(f, "Error::IOError"),
            Error::FramingError => defmt::write!(f, "Error::FramingError"),
            Error::CRCError => defmt::write!(f, "Error::CRCError"),
            Error::ParityError => defmt::write!(f, "Error::ParityError"),
            Error::FifoIncompleteByte => defmt::write!(f, "Error::FifoIncompleteByte"),
            Error::FifoNoRoom => defmt::write!(f, "Error::FifoNoRoom"),
            Error::Collision => defmt::write!(f, "Error::Collision"),
            Error::AntiCollisionMaxLoopsReached => {
                defmt::write!(f, "Error::AntiCollisionMaxLoopsReached")
            }
            Error::Timeout => defmt::write!(f, "Error::Timeout"),
            Error::Nak => defmt::write!(f, "Error::Nak"),
        };
    }
}

#[cfg(feature = "std")]
impl<E> std::fmt::Display for Error<E> {
    fn fmt(&self, _f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // TODO
        Ok(())
    }
}

#[cfg(feature = "std")]
impl<E> std::error::Error for Error<E>
where
    E: std::fmt::Debug,
{
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        // TODO
        None
    }

    fn description(&self) -> &str {
        "description() is deprecated; use Display"
    }

    fn cause(&self) -> Option<&dyn std::error::Error> {
        self.source()
    }
}

