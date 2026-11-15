//! Bus transports: how register reads and writes reach the sensor.
//!
//! The drivers are generic over [`RegisterIo`](crate::interface::RegisterIo)
//! (blocking) or `AsyncRegisterIo` (async, behind the `async`
//! feature). Two transports are provided:
//! [`I2cInterface`](crate::interface::I2cInterface) over
//! `embedded-hal` I2C, and
//! [`SpiInterface`](crate::interface::SpiInterface) over
//! `embedded-hal` SPI devices.
//!
//! SPI framing follows the datasheet SPI address format figure: the
//! first byte carries a read flag in bit 7 (1 = read, 0 = write) and
//! the 7-bit register address in bits 6:0, data MSB first, host SPI
//! Mode 0 or 3 (auto-detected by the sensor) at up to 15 MHz, 4-wire.
//! Burst reads rely on CTRL1 address auto-increment. Writes always go
//! one register per transaction: the datasheet does not support burst
//! writes to configuration registers, and the CAL parameter blocks are
//! the only multi-byte writes this driver performs.

/// I2C device address, selected by the SA0 pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum DeviceAddress {
    /// `0x6A` — SA0 high or floating (internal pull-up).
    #[default]
    Primary = 0x6A,
    /// `0x6B` — SA0 pulled low.
    Secondary = 0x6B,
}

impl DeviceAddress {
    /// The 7-bit I2C address.
    #[must_use]
    pub const fn as_u8(self) -> u8 {
        self as u8
    }
}

/// Blocking register access used by the synchronous driver.
pub trait RegisterIo {
    /// Bus error type.
    type Error;

    /// Write one register.
    ///
    /// # Errors
    /// Returns the bus error if the transaction fails.
    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;

    /// Write `data` starting at register `reg` (address auto-increment
    /// must be enabled on the device for this to land contiguously).
    ///
    /// # Errors
    /// Returns the bus error if a transaction fails.
    fn write_block(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// Read registers starting at `reg` into `buf`.
    ///
    /// # Errors
    /// Returns the bus error if the transaction fails.
    fn read_block(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Read one register.
    ///
    /// # Errors
    /// Returns the bus error if the transaction fails.
    fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_block(reg, &mut buf)?;
        Ok(buf[0])
    }
}

/// I2C transport over an `embedded-hal` I2C bus.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct I2cInterface<I> {
    /// The underlying I2C bus.
    pub bus: I,
    /// Device address on the bus.
    pub address: DeviceAddress,
}

impl<I> I2cInterface<I> {
    /// Wrap an I2C bus and device address.
    pub const fn new(bus: I, address: DeviceAddress) -> Self {
        Self { bus, address }
    }

    /// Give back the underlying bus.
    pub fn release(self) -> I {
        self.bus
    }
}

impl<I: embedded_hal::i2c::I2c> RegisterIo for I2cInterface<I> {
    type Error = I::Error;

    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.bus.write(self.address.as_u8(), &[reg, value])
    }

    fn write_block(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        // Blocks are prefixed with the register address in one
        // transaction; the largest block this driver sends is the
        // 8-byte CAL parameter image, so chunk at 8 data bytes.
        let mut buf = [0u8; 9];
        let mut offset = 0usize;
        while offset < data.len() {
            let take = (data.len() - offset).min(8);
            buf[0] = reg.wrapping_add(u8::try_from(offset).unwrap_or(u8::MAX));
            buf[1..=take].copy_from_slice(&data[offset..offset + take]);
            self.bus.write(self.address.as_u8(), &buf[..=take])?;
            offset += take;
        }
        Ok(())
    }

    fn read_block(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.write_read(self.address.as_u8(), &[reg], buf)
    }
}

/// SPI transport over an `embedded-hal` SPI device (bus + chip select).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SpiInterface<S> {
    /// The underlying SPI device.
    pub device: S,
}

impl<S> SpiInterface<S> {
    /// Wrap an SPI device.
    pub const fn new(device: S) -> Self {
        Self { device }
    }

    /// Give back the underlying SPI device.
    pub fn release(self) -> S {
        self.device
    }
}

/// SPI first byte for a register read: bit 7 set, 7-bit address.
const fn spi_read_addr(reg: u8) -> u8 {
    reg | 0x80
}

/// SPI first byte for a register write: bit 7 clear, 7-bit address.
const fn spi_write_addr(reg: u8) -> u8 {
    reg & 0x7F
}

impl<S: embedded_hal::spi::SpiDevice> RegisterIo for SpiInterface<S> {
    type Error = S::Error;

    fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.device.write(&[spi_write_addr(reg), value])
    }

    fn write_block(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        // Burst writes to configuration registers are unsupported by
        // the device; always write one register per transaction.
        for (i, &value) in data.iter().enumerate() {
            let addr = reg.wrapping_add(u8::try_from(i).unwrap_or(u8::MAX));
            self.write_register(addr, value)?;
        }
        Ok(())
    }

    fn read_block(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.device.transaction(&mut [
            Operation::Write(&[spi_read_addr(reg)]),
            Operation::Read(buf),
        ])
    }
}

/// Async register access used by the async driver. Only ever used
/// generically, never as a trait object.
#[cfg(feature = "async")]
#[allow(async_fn_in_trait)]
pub trait AsyncRegisterIo {
    /// Bus error type.
    type Error;

    /// Write one register.
    ///
    /// # Errors
    /// Returns the bus error if the transaction fails.
    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error>;

    /// Write `data` starting at register `reg`.
    ///
    /// # Errors
    /// Returns the bus error if a transaction fails.
    async fn write_block(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error>;

    /// Read registers starting at `reg` into `buf`.
    ///
    /// # Errors
    /// Returns the bus error if the transaction fails.
    async fn read_block(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Read one register.
    ///
    /// # Errors
    /// Returns the bus error if the transaction fails.
    async fn read_register(&mut self, reg: u8) -> Result<u8, Self::Error> {
        let mut buf = [0u8; 1];
        self.read_block(reg, &mut buf).await?;
        Ok(buf[0])
    }
}

#[cfg(feature = "async")]
impl<I: embedded_hal_async::i2c::I2c> AsyncRegisterIo for I2cInterface<I> {
    type Error = I::Error;

    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.bus.write(self.address.as_u8(), &[reg, value]).await
    }

    async fn write_block(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        let mut buf = [0u8; 9];
        let mut offset = 0usize;
        while offset < data.len() {
            let take = (data.len() - offset).min(8);
            buf[0] = reg.wrapping_add(u8::try_from(offset).unwrap_or(u8::MAX));
            buf[1..=take].copy_from_slice(&data[offset..offset + take]);
            self.bus.write(self.address.as_u8(), &buf[..=take]).await?;
            offset += take;
        }
        Ok(())
    }

    async fn read_block(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.bus.write_read(self.address.as_u8(), &[reg], buf).await
    }
}

#[cfg(feature = "async")]
impl<S: embedded_hal_async::spi::SpiDevice> AsyncRegisterIo for SpiInterface<S> {
    type Error = S::Error;

    async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Self::Error> {
        self.device.write(&[spi_write_addr(reg), value]).await
    }

    async fn write_block(&mut self, reg: u8, data: &[u8]) -> Result<(), Self::Error> {
        for (i, &value) in data.iter().enumerate() {
            let addr = reg.wrapping_add(u8::try_from(i).unwrap_or(u8::MAX));
            self.write_register(addr, value).await?;
        }
        Ok(())
    }

    async fn read_block(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Self::Error> {
        use embedded_hal::spi::Operation;
        self.device
            .transaction(&mut [
                Operation::Write(&[spi_read_addr(reg)]),
                Operation::Read(buf),
            ])
            .await
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTransaction};

    #[test]
    fn i2c_framing() {
        let expectations = [
            I2cTransaction::write(0x6A, vec![0x02, 0x58]),
            I2cTransaction::write(0x6A, vec![0x0B, 1, 2, 3, 4, 5, 6, 7, 8]),
            I2cTransaction::write_read(0x6B, vec![0x00], vec![0x05]),
        ];
        let mock = I2cMock::new(&expectations);
        let mut iface = I2cInterface::new(mock, DeviceAddress::Primary);
        RegisterIo::write_register(&mut iface, 0x02, 0x58).unwrap();
        RegisterIo::write_block(&mut iface, 0x0B, &[1, 2, 3, 4, 5, 6, 7, 8]).unwrap();
        iface.address = DeviceAddress::Secondary;
        assert_eq!(RegisterIo::read_register(&mut iface, 0x00).unwrap(), 0x05);
        iface.release().done();
    }

    /// Minimal SpiDevice stub recording written bytes and serving reads.
    #[derive(Debug)]
    struct StubSpi {
        written: Vec<u8>,
        reads: Vec<u8>,
    }

    impl embedded_hal::spi::ErrorType for StubSpi {
        type Error = core::convert::Infallible;
    }

    impl embedded_hal::spi::SpiDevice for StubSpi {
        fn transaction(
            &mut self,
            operations: &mut [embedded_hal::spi::Operation<'_, u8>],
        ) -> Result<(), Self::Error> {
            for op in operations {
                match op {
                    embedded_hal::spi::Operation::Write(bytes) => {
                        self.written.extend_from_slice(bytes);
                    }
                    embedded_hal::spi::Operation::Read(buf) => {
                        let n = buf.len().min(self.reads.len());
                        buf[..n].copy_from_slice(&self.reads[..n]);
                        self.reads.drain(..n);
                    }
                    _ => {}
                }
            }
            Ok(())
        }
    }

    #[test]
    fn spi_framing() {
        let stub = StubSpi {
            written: Vec::new(),
            reads: vec![0x05],
        };
        let mut iface = SpiInterface::new(stub);
        // Write: read bit clear.
        iface.write_register(0x02, 0x58).unwrap();
        // Block write: one single-register transaction per byte.
        iface.write_block(0x0B, &[0xAA, 0xBB]).unwrap();
        // Read: read bit set, then data phase.
        assert_eq!(iface.read_register(0x00).unwrap(), 0x05);
        let stub = iface.release();
        assert_eq!(stub.written, vec![0x02, 0x58, 0x0B, 0xAA, 0x0C, 0xBB, 0x80]);
    }
}
