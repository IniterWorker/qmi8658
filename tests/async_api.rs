//! Transaction-level tests of the async I2C driver.
#![allow(clippy::unwrap_used, clippy::panic)]
#![cfg(feature = "async")]

use embedded_hal_mock::eh1::{
    delay::NoopDelay,
    i2c::{Mock as I2cMock, Transaction as Tx},
};
use futures::executor::block_on;
use qmi8658::{
    DeviceAddress, Qmi8658Async,
    config::{Config, TapConfig},
};

fn write(reg: u8, val: u8) -> Tx {
    Tx::write(0x6A, vec![reg, val])
}

fn read(reg: u8, bytes: &[u8]) -> Tx {
    Tx::write_read(0x6A, vec![reg], bytes.to_vec())
}

fn cmd(cmd: u8) -> Vec<Tx> {
    vec![
        write(0x0A, cmd),
        read(0x2D, &[0x80]),
        write(0x0A, 0x00),
        read(0x2D, &[0x00]),
    ]
}

fn init_script() -> Vec<Tx> {
    let mut v = vec![
        write(0x60, 0xB0),
        read(0x00, &[0x05]),
        write(0x02, 0x58),
        write(0x0B, 0x01),
    ];
    v.extend(cmd(0x12));
    v.extend([
        write(0x08, 0x00),
        write(0x03, 0x15),
        write(0x04, 0x55),
        write(0x06, 0x00),
        write(0x08, 0x03),
    ]);
    v
}

#[test]
fn async_init_read_sample_and_tap() {
    let mut block = vec![0u8; 17];
    block[3..5].copy_from_slice(&0x1980i16.to_le_bytes());
    block[9..11].copy_from_slice(&8192i16.to_le_bytes());
    let mut script = init_script();
    script.push(read(0x30, &block));
    // One Ctrl9 flow: tap configure (sensors off, two parameter calls,
    // CTRL8 enable, restore).
    script.push(read(0x08, &[0x03]));
    script.push(write(0x08, 0x00));
    script.push(Tx::write(0x6A, vec![0x0B, 20, 0, 50, 0, 250, 0, 0, 0x01]));
    script.extend(cmd(0x0C));
    script.push(Tx::write(
        0x6A,
        vec![0x0B, 8, 32, 0x20, 0x03, 0x90, 0x01, 0, 0x02],
    ));
    script.extend(cmd(0x0C));
    script.push(read(0x09, &[0x00]));
    script.push(write(0x09, 0x01));
    script.push(write(0x08, 0x03));

    let mut imu =
        Qmi8658Async::new_i2c(I2cMock::new(&script), DeviceAddress::Primary, NoopDelay {});
    block_on(async {
        imu.init(Config::default()).await.unwrap();
        let s = imu.read_sample().await.unwrap();
        assert!((s.accel_g[2] - 1.0).abs() < 1e-6);
        assert!((s.temperature_c - 25.5).abs() < 1e-6);
        imu.configure_tap(&TapConfig::default(), true)
            .await
            .unwrap();
    });
    let (iface, _) = imu.release();
    iface.release().done();
}
