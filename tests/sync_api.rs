//! Transaction-level tests of the blocking I2C driver.
#![allow(clippy::unwrap_used, clippy::panic)]

use embedded_hal_mock::eh1::{
    delay::NoopDelay,
    i2c::{Mock as I2cMock, Transaction as Tx},
};
use qmi8658::{
    DeviceAddress, Error, Qmi8658,
    config::{Config, FifoConfig, TapConfig},
    data::{FifoFrameFormat, FifoFrames, GyroGains},
};

type Drv = Qmi8658<qmi8658::interface::I2cInterface<I2cMock>, NoopDelay>;

fn write(reg: u8, val: u8) -> Tx {
    Tx::write(0x6A, vec![reg, val])
}

fn read(reg: u8, bytes: &[u8]) -> Tx {
    Tx::write_read(0x6A, vec![reg], bytes.to_vec())
}

/// One successful Ctrl9 handshake for `cmd`.
fn cmd(cmd: u8) -> Vec<Tx> {
    vec![
        write(0x0A, cmd),
        read(0x2D, &[0x80]),
        write(0x0A, 0x00),
        read(0x2D, &[0x00]),
    ]
}

/// Full `init(Config::default())` transaction script.
fn init_script() -> Vec<Tx> {
    let mut v = vec![
        write(0x60, 0xB0),   // soft reset
        read(0x00, &[0x05]), // WHO_AM_I
        write(0x02, 0x58),   // CTRL1: ADDR_AI, little-endian, INT1+INT2 on
        write(0x0B, 0x01),   // CAL1_L = 1 (disable AHB clock gating)
    ];
    v.extend(cmd(0x12));
    v.extend([
        write(0x08, 0x00), // CTRL7 sensors off
        write(0x03, 0x15), // CTRL2: ±4 g, 224.2 Hz
        write(0x04, 0x55), // CTRL3: ±512 dps, 224.2 Hz
        write(0x06, 0x00), // CTRL5: filters off
        write(0x08, 0x03), // CTRL7: accel + gyro on
    ]);
    v
}

fn driver(script: Vec<Tx>) -> Drv {
    Qmi8658::new_i2c(I2cMock::new(&script), DeviceAddress::Primary, NoopDelay {})
}

fn finish(d: Drv) {
    let (iface, _) = d.release();
    iface.release().done();
}

/// 17-byte sample block: ts 0x123456, 25.5 °C, AZ = 1 g at ±4 g, GX = -1 dps at ±512.
fn sample_block() -> Vec<u8> {
    let mut b = vec![0u8; 17];
    b[0..3].copy_from_slice(&[0x56, 0x34, 0x12]);
    b[3..5].copy_from_slice(&0x1980i16.to_le_bytes());
    b[9..11].copy_from_slice(&8192i16.to_le_bytes());
    b[11..13].copy_from_slice(&(-64i16).to_le_bytes());
    b
}

#[test]
fn init_and_read_sample() {
    let mut script = init_script();
    script.push(read(0x30, &sample_block()));
    let mut imu = driver(script);
    imu.init(Config::default()).unwrap();
    let s = imu.read_sample().unwrap();
    assert_eq!(s.timestamp, 0x12_3456);
    assert!((s.accel_g[2] - 1.0).abs() < 1e-6);
    assert!((s.gyro_dps[0] + 1.0).abs() < 1e-6);
    assert!((s.temperature_c - 25.5).abs() < 1e-6);
    finish(imu);
}

#[test]
fn init_rejects_wrong_device_id() {
    let mut imu = driver(vec![write(0x60, 0xB0), read(0x00, &[0x42])]);
    let err = imu.init(Config::default()).unwrap_err();
    assert!(matches!(err, Error::InvalidDeviceId { got: 0x42 }));
    finish(imu);
}

#[test]
fn invalid_config_makes_no_bus_writes() {
    let mut imu = driver(vec![]);
    let cfg = Config::new().without_accel().without_gyro();
    assert!(matches!(
        imu.apply_config(cfg).unwrap_err(),
        Error::InvalidConfig(_)
    ));
    finish(imu);
}

#[test]
fn tap_configure_sequence() {
    let mut script = init_script();
    script.push(read(0x08, &[0x03])); // save CTRL7
    script.push(write(0x08, 0x00)); // sensors off
    script.push(Tx::write(0x6A, vec![0x0B, 20, 0, 50, 0, 250, 0, 0, 0x01]));
    script.extend(cmd(0x0C));
    script.push(Tx::write(
        0x6A,
        vec![0x0B, 8, 32, 0x20, 0x03, 0x90, 0x01, 0, 0x02],
    ));
    script.extend(cmd(0x0C));
    script.push(read(0x09, &[0x00])); // CTRL8 RMW
    script.push(write(0x09, 0x01)); // Tap_EN
    script.push(write(0x08, 0x03)); // restore CTRL7
    let mut imu = driver(script);
    imu.init(Config::default()).unwrap();
    imu.configure_tap(&TapConfig::default(), true).unwrap();
    finish(imu);
}

#[test]
fn fifo_configure_and_read() {
    let mut script = init_script();
    script.push(write(0x13, 0x01)); // watermark
    script.push(write(0x14, 0x0E)); // FIFO_CTRL: 128 samples, Stream
    script.extend(cmd(0x04)); // reset FIFO
    // read_fifo: status (6 words = 12 bytes, not empty), REQ_FIFO, burst, RD_MODE clear.
    script.push(read(0x15, &[6, 0x10]));
    script.extend(cmd(0x05));
    let mut frame = vec![0u8; 12];
    frame[4..6].copy_from_slice(&8192i16.to_le_bytes()); // AZ raw
    frame[6..8].copy_from_slice(&64i16.to_le_bytes()); // GX raw
    script.push(read(0x17, &frame));
    script.push(read(0x14, &[0x8E])); // FIFO_CTRL with RD_MODE set
    script.push(write(0x14, 0x0E)); // RD_MODE cleared
    let mut imu = driver(script);
    imu.init(Config::default()).unwrap();
    imu.configure_fifo(FifoConfig::default()).unwrap();
    assert_eq!(imu.fifo_format(), FifoFrameFormat::AccelAndGyro);
    let mut buf = [0u8; 64];
    let n = imu.read_fifo(&mut buf).unwrap();
    assert_eq!(n, 12);
    let frames: Vec<_> = FifoFrames::new(&buf[..n], imu.fifo_format()).collect();
    assert_eq!(frames.len(), 1);
    assert_eq!(frames[0].accel, Some([0, 0, 8192]));
    assert_eq!(frames[0].gyro, Some([64, 0, 0]));
    finish(imu);
}

#[test]
fn accel_self_test_passes() {
    let mut script = init_script();
    script.extend([
        read(0x03, &[0x15]), // save CTRL2
        read(0x04, &[0x55]), // save CTRL3
        read(0x08, &[0x03]), // save CTRL7
        write(0x08, 0x00),   // sensors off
        write(0x03, 0x93),   // CTRL2: aST on, ±4 g, 1000 Hz standalone
        read(0x2D, &[0x01]), // STATUSINT avail
        write(0x03, 0x13),   // aST off
        read(0x2D, &[0x00]), // avail cleared
    ]);
    let mut dvx = vec![0u8; 6];
    for i in 0..3 {
        dvx[i * 2..i * 2 + 2].copy_from_slice(&1000i16.to_le_bytes()); // 500 mg each
    }
    script.push(read(0x51, &dvx));
    script.extend([write(0x03, 0x15), write(0x04, 0x55), write(0x08, 0x03)]);
    let mut imu = driver(script);
    imu.init(Config::default()).unwrap();
    let report = imu.self_test_accel().unwrap();
    assert!(report.passed);
    assert!((report.delta[0] - 500.0).abs() < 1e-6);
    finish(imu);
}

#[test]
fn gyro_self_test_can_fail() {
    let mut script = init_script();
    script.extend([
        read(0x03, &[0x15]),
        read(0x04, &[0x55]),
        read(0x08, &[0x03]),
        write(0x08, 0x00),
        write(0x04, 0x80), // CTRL3: gST on with reset FS/ODR (hardware-verified trigger)
        read(0x2D, &[0x01]),
        write(0x04, 0x00), // gST off
        read(0x2D, &[0x00]),
    ]);
    let mut dvx = vec![0u8; 6];
    for i in 0..3 {
        dvx[i * 2..i * 2 + 2].copy_from_slice(&1600i16.to_le_bytes()); // 100 dps each
    }
    script.push(read(0x51, &dvx));
    script.extend([write(0x03, 0x15), write(0x04, 0x55), write(0x08, 0x03)]);
    let mut imu = driver(script);
    imu.init(Config::default()).unwrap();
    let report = imu.self_test_gyro().unwrap();
    assert!(!report.passed);
    assert!((report.delta[2] - 100.0).abs() < 1e-6);
    finish(imu);
}

#[test]
fn cod_success_returns_gains() {
    let mut script = init_script();
    script.push(read(0x08, &[0x03])); // save CTRL7
    script.push(write(0x08, 0x00)); // sensors off
    script.extend(cmd(0xA2));
    script.push(read(0x46, &[0x00])); // COD_STATUS success
    script.push(read(0x51, &[1, 2, 3, 4, 5, 6]));
    script.push(write(0x08, 0x03)); // restore CTRL7
    let mut imu = driver(script);
    imu.init(Config::default()).unwrap();
    assert_eq!(
        imu.calibrate_gyro_on_demand().unwrap(),
        GyroGains([1, 2, 3, 4, 5, 6])
    );
    finish(imu);
}

#[test]
fn cod_failure_reports_calibration_failed() {
    let mut script = init_script();
    script.push(read(0x08, &[0x03]));
    script.push(write(0x08, 0x00));
    script.extend(cmd(0xA2));
    script.push(read(0x46, &[0x01])); // COD_Failed
    script.push(write(0x08, 0x03)); // restore still happens
    let mut imu = driver(script);
    imu.init(Config::default()).unwrap();
    assert!(matches!(
        imu.calibrate_gyro_on_demand().unwrap_err(),
        Error::CalibrationFailed
    ));
    finish(imu);
}

#[test]
fn ctrl9_timeout_when_done_never_asserts() {
    let mut script = vec![write(0x60, 0xB0), read(0x00, &[0x05])];
    script.push(write(0x02, 0x58));
    script.push(write(0x0B, 0x01));
    script.push(write(0x0A, 0x12));
    // 100 ms budget at 1 ms poll cadence: initial poll + 100 retries.
    for _ in 0..=100 {
        script.push(read(0x2D, &[0x00]));
    }
    let mut imu = driver(script);
    assert!(matches!(
        imu.init(Config::default()).unwrap_err(),
        Error::CommandTimeout
    ));
    finish(imu);
}

#[test]
fn device_id_uses_single_reads() {
    // Works before `init` (address auto-increment not yet enabled).
    let mut imu = driver(vec![read(0x00, &[0x05]), read(0x01, &[0x68])]);
    let id = imu.device_id().unwrap();
    assert_eq!(id.who_am_i, 0x05);
    assert_eq!(id.revision, 0x68);
    finish(imu);
}

#[test]
fn gyro_self_test_retries_dropped_first_trigger() {
    // Hardware quirk: the first gST trigger (notably after an accel
    // self-test) never signals; the driver clears gST and re-triggers.
    let mut script = init_script();
    script.extend([
        read(0x03, &[0x15]),
        read(0x04, &[0x55]),
        read(0x08, &[0x03]),
        write(0x08, 0x00),
        write(0x04, 0x80),
    ]);
    // 700 ms budget at 5 ms cadence: polls at 0,5,...,700 ms.
    for _ in 0..=140 {
        script.push(read(0x2D, &[0x00]));
    }
    script.extend([
        write(0x04, 0x00), // clear gST
        write(0x04, 0x80), // re-trigger
        read(0x2D, &[0x01]),
        write(0x04, 0x00),
        read(0x2D, &[0x00]),
    ]);
    let mut dvx = vec![0u8; 6];
    for i in 0..3 {
        dvx[i * 2..i * 2 + 2].copy_from_slice(&16000i16.to_le_bytes()); // 1000 dps each
    }
    script.push(read(0x51, &dvx));
    script.extend([write(0x03, 0x15), write(0x04, 0x55), write(0x08, 0x03)]);
    let mut imu = driver(script);
    imu.init(Config::default()).unwrap();
    let report = imu.self_test_gyro().unwrap();
    assert!(report.passed);
    assert!((report.delta[0] - 1000.0).abs() < 1e-6);
    finish(imu);
}
