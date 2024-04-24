#![allow(clippy::missing_const_for_fn)]
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, SevenBitAddress};

use crate::command::register::ship_info::{DeviceID, RevisionID};
use crate::command::WHO_AM_I;

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum DeviceAddress {
    /// SA0 external pullup or floationg (intenal pullup)
    Primary = 0x6A,
    /// SA0 external pulldown
    Secondary = 0x6B,
}

impl From<DeviceAddress> for u8 {
    fn from(value: DeviceAddress) -> Self {
        match value {
            DeviceAddress::Primary => 0x6A,
            DeviceAddress::Secondary => 0x6B,
        }
    }
}

impl Default for DeviceAddress {
    fn default() -> Self {
        Self::Primary
    }
}

/// Qmi8658 Driver
pub struct Qmi8658<I, D> {
    /// I2C Interface
    pub(crate) interface: I,
    /// I2C Address
    pub(crate) addr: DeviceAddress,
    /// Delay
    pub(crate) _delay: D,
}

// #[derive(Clone, Debug)]
// pub enum Error {
//     /// I2C Error,
//     I2CError(embedded_hal::i2c::Error)
// }

impl<I, D> Qmi8658<I, D>
where
    I: I2c<SevenBitAddress>,
    D: DelayNs,
{
    /// Create a new Qmi8658 driver with default I2C address.
    pub fn new(interface: I, delay: D) -> Self {
        Self {
            interface,
            addr: DeviceAddress::Primary,
            _delay: delay,
        }
    }

    /// Create new Qmi8658 driver with secondary I2C address.
    pub fn new_secondary_address(interface: I, delay: D) -> Self {
        Self {
            interface,
            addr: DeviceAddress::Secondary,
            _delay: delay,
        }
    }

    /// Give back the I2C interface
    pub fn release(self) -> I {
        self.interface
    }

    /// Get the device id
    ///
    /// # Errors
    ///
    pub fn get_device_id(&mut self) -> Result<DeviceID, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[WHO_AM_I], &mut buffer)?;
        let value = match buffer[0] {
            0x05 => DeviceID::QST,
            _ => DeviceID::Unknown,
        };

        Ok(value)
    }

    /// Get the device revision id
    ///
    /// # Errors
    ///
    pub fn get_device_revision_id(&mut self) -> Result<RevisionID, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[WHO_AM_I], &mut buffer)?;
        Ok(buffer[0])
    }
}
