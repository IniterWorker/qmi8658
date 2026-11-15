//! Host-runnable end-to-end demo over a mock I2C bus:
//! `cargo run --example mock_poll`
//!
//! Scripts the exact transactions a real QMI8658 would answer, runs
//! `init` with the default configuration, and prints one converted
//! sample (the fixture encodes +1 g on Z, 0 dps, 25.5 °C).

use embedded_hal_mock::eh1::{
    delay::NoopDelay,
    i2c::{Mock, Transaction},
};
use qmi8658::{DeviceAddress, Qmi8658, config::Config};

fn main() {
    let mut block = vec![0u8; 17];
    block[3..5].copy_from_slice(&0x1980i16.to_le_bytes()); // 25.5 °C
    block[9..11].copy_from_slice(&8192i16.to_le_bytes()); // AZ = 1 g at ±4 g
    let script = [
        Transaction::write(0x6A, vec![0x60, 0xB0]), // soft reset
        Transaction::write_read(0x6A, vec![0x00], vec![0x05]), // WHO_AM_I
        Transaction::write(0x6A, vec![0x02, 0x58]), // CTRL1
        Transaction::write(0x6A, vec![0x0B, 0x01]), // CAL1_L (AHB gating off)
        Transaction::write(0x6A, vec![0x0A, 0x12]), // Ctrl9 AHB command
        Transaction::write_read(0x6A, vec![0x2D], vec![0x80]),
        Transaction::write(0x6A, vec![0x0A, 0x00]), // Ctrl9 ack
        Transaction::write_read(0x6A, vec![0x2D], vec![0x00]),
        Transaction::write(0x6A, vec![0x08, 0x00]), // CTRL7 off
        Transaction::write(0x6A, vec![0x03, 0x15]), // CTRL2 ±4 g 224 Hz
        Transaction::write(0x6A, vec![0x04, 0x55]), // CTRL3 ±512 dps 224 Hz
        Transaction::write(0x6A, vec![0x06, 0x00]), // CTRL5 filters off
        Transaction::write(0x6A, vec![0x08, 0x03]), // CTRL7 accel+gyro on
        Transaction::write_read(0x6A, vec![0x30], block),
    ];
    let mut imu = Qmi8658::new_i2c(Mock::new(&script), DeviceAddress::Primary, NoopDelay {});
    imu.init(Config::default()).expect("init failed");
    let sample = imu.read_sample().expect("read_sample failed");
    println!(
        "accel [{:.2}, {:.2}, {:.2}] g, gyro [{:.1}, {:.1}, {:.1}] dps, temp {:.1} °C",
        sample.accel_g[0],
        sample.accel_g[1],
        sample.accel_g[2],
        sample.gyro_dps[0],
        sample.gyro_dps[1],
        sample.gyro_dps[2],
        sample.temperature_c,
    );
    let (iface, _) = imu.release();
    iface.release().done();
}
