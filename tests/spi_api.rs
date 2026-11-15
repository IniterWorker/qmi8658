//! Transaction-level tests of the blocking SPI driver.
#![allow(clippy::unwrap_used, clippy::panic)]

use embedded_hal::digital::{ErrorType, OutputPin};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_hal_mock::eh1::{
    delay::NoopDelay,
    spi::{Mock as SpiMock, Transaction as SpiTx},
};
use qmi8658::{Qmi8658, config::Config};

/// Chip-select pin that does nothing; the bus mock checks the data.
#[derive(Debug)]
struct Cs;

impl ErrorType for Cs {
    type Error = core::convert::Infallible;
}

impl OutputPin for Cs {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// One SPI-device write call: data plus the bus flush that ends it.
fn wr(bytes: &[u8]) -> Vec<SpiTx<u8>> {
    vec![SpiTx::write_vec(bytes.to_vec()), SpiTx::flush()]
}

/// Register read: address byte with the read flag (bit 7), data, flush.
fn rd(reg: u8, data: &[u8]) -> Vec<SpiTx<u8>> {
    vec![
        SpiTx::write_vec(vec![reg | 0x80]),
        SpiTx::read_vec(data.to_vec()),
        SpiTx::flush(),
    ]
}

fn cmd(cmd: u8) -> Vec<SpiTx<u8>> {
    let mut v = wr(&[0x0A, cmd]);
    v.extend(rd(0x2D, &[0x80]));
    v.extend(wr(&[0x0A, 0x00]));
    v.extend(rd(0x2D, &[0x00]));
    v
}

#[test]
fn spi_init_and_read_raw() {
    let mut script = wr(&[0x60, 0xB0]);
    script.extend(rd(0x00, &[0x05]));
    script.extend(wr(&[0x02, 0x58]));
    script.extend(wr(&[0x0B, 0x01]));
    script.extend(cmd(0x12));
    for bytes in [
        [0x08, 0x00],
        [0x03, 0x15],
        [0x04, 0x55],
        [0x06, 0x00],
        [0x08, 0x03],
    ] {
        script.extend(wr(&bytes));
    }
    let mut block = vec![0u8; 17];
    block[0..3].copy_from_slice(&[0x56, 0x34, 0x12]);
    block[9..11].copy_from_slice(&8192i16.to_le_bytes());
    script.extend(rd(0x30, &block));

    let bus = SpiMock::new(&script);
    let mut check = bus.clone();
    let device = ExclusiveDevice::new_no_delay(bus, Cs).unwrap();
    let mut imu = Qmi8658::new_spi(device, NoopDelay {});
    imu.init(Config::default()).unwrap();
    let raw = imu.read_raw().unwrap();
    assert_eq!(raw.timestamp, 0x12_3456);
    assert_eq!(raw.accel, [0, 0, 8192]);
    drop(imu);
    check.done();
}
