//! Driver error type, shared by the blocking and async drivers over both
//! transports.

/// Errors returned by every driver operation.
///
/// `E` is the bus error type: the I2C error for I2C transports, or the
/// `SpiDevice` error (which already covers bus and chip-select failures)
/// for SPI transports.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<E> {
    /// The underlying bus transaction failed.
    Bus(E),
    /// The WHO_AM_I register did not read `0x05`.
    InvalidDeviceId {
        /// The value actually read from the WHO_AM_I register.
        got: u8,
    },
    /// A Ctrl9 command, or a self-test, did not complete within its
    /// time budget. On silicon without the requested feature (for
    /// example pedometer commands on the A variant) this is the
    /// expected failure mode.
    CommandTimeout,
    /// The requested configuration is not meaningful for this device;
    /// no bus writes were performed.
    InvalidConfig(&'static str),
    /// A numeric argument (for example a host delta offset) is outside
    /// the range representable in the device's fixed-point format.
    ValueOutOfRange,
    /// Calibration-on-demand ran but reported failure in COD_STATUS.
    /// The failing checks can be inspected with the driver's
    /// `cod_status` method.
    CalibrationFailed,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Self {
        Self::Bus(err)
    }
}
