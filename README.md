# QMI8658

<p align="left">
    <a href="https://github.com/initerworker/qmi8658/actions/workflows/rust-release.yml"><img src="https://github.com/initerworker/qmi8658/actions/workflows/rust-release.yml/badge.svg" alt="Github CI/CD"></a>
    <a href="https://crates.io/crates/qmi8658"><img src="https://img.shields.io/crates/v/qmi8658.svg" alt="Crates.io"></a>
    <a href="https://docs.rs/qmi8658"><img src="https://docs.rs/qmi8658/badge.svg" alt="Docs.rs"></a>
</p>

Rust `no_std` driver for the QMI8658 low-noise 6-axis inertial
measurement unit: blocking and async APIs over I2C and SPI, built on
`embedded-hal` 1.0 / `embedded-hal-async` 1.0.

## Support matrix

| API | I2C | SPI (4-wire) |
|---|---|---|
| Blocking (`embedded-hal`) | yes | yes |
| Async (`embedded-hal-async`, feature `async`) | yes | yes |

Features: FIFO (bypass/FIFO/stream), tap and any/no/significant-motion
engines, pedometer and Wake-on-Motion (C variant), host delta offsets,
gyro calibration-on-demand, accel/gyro self-tests, SyncSample
data locking, `defmt` and `log` integrations behind features.

SPI is verified by transaction-level mock tests; hardware testing uses
an ESP32-S3 board whose IMU is wired over I2C.

## Open-To-Idea

Please feel free to open an issue with your design improvements/considerations.

## QMI8658 6-axis IMU driver

A `no_std` driver for the QST QMI8658 inertial measurement unit,
built on `embedded-hal` 1.0 (blocking, [`Qmi8658`]) and, behind the
`async` feature, `embedded-hal-async` 1.0 ([`Qmi8658Async`]). Both
drivers work over I2C and SPI (4-wire) and share one typed
configuration and data model.

### Features

| Feature | Default | Effect |
|---|---|---|
| `async` | no | async driver [`Qmi8658Async`] over `embedded-hal-async` |
| `defmt` | no | `defmt::Format` implementations for public types |
| `log` | no | `log` diagnostics from init, Ctrl9 timeouts, self-tests |

The blocking driver is always available.

### Wiring notes

- I2C address is selected by the SA0 pin: high or floating gives
  [`DeviceAddress::Primary`] (`0x6A`), low gives
  [`DeviceAddress::Secondary`] (`0x6B`).
- SPI: the sensor auto-detects Mode 0/3; configure the host for
  Mode 0, MSB first, at most 15 MHz, 4-wire.
- Wait at least 15 ms after power-up before `init`; gyroscope
  data is valid ≈150 ms + 3/ODR after the gyro is enabled.
- While the FIFO is being drained its filling is paused, so read
  it promptly.

### Silicon variants

Pedometer and Wake-on-Motion are documented as C-variant features;
the ±2048 dps gyro range and the CTRL1 interrupt-pin enable bits
as A/B-variant features. All features are exposed
unconditionally; commands the silicon does not implement fail
with [`Error::CommandTimeout`]. The revision ID is reported
verbatim, never asserted. Hardware-verified example: the part on
the ESP32-S3 reference board reports revision `0x7C` yet accepts
and runs the pedometer configuration, so do not gate features on
the revision byte.

SPI support is verified by transaction-level mock tests; the
reference board used for hardware testing wires the sensor over
I2C.

### Quickstart (blocking, I2C)

Shown against a mock bus so the example runs as a doctest; on
hardware, pass your HAL's I2C and delay types instead.

```rust
use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::{Mock, Transaction}};
use qmi8658::{DeviceAddress, Qmi8658, config::Config};

let mut imu = Qmi8658::new_i2c(Mock::new(&script), DeviceAddress::Primary, NoopDelay {});
imu.init(Config::default()).unwrap();
let sample = imu.read_sample().unwrap();
assert!((sample.accel_g[2] - 1.0).abs() < 1e-6);
```

### Async (Embassy-style)

The async driver is executor-agnostic; it only needs
`embedded-hal-async` bus and delay implementations from your HAL.

```rust
use qmi8658::{DeviceAddress, Qmi8658Async, config::Config};

// `i2c` and `delay` are your HAL's async types (for example an
// Embassy-time-backed delay and an async I2C peripheral).
let mut imu = Qmi8658Async::new_i2c(i2c, DeviceAddress::Primary, delay);
imu.init(Config::default()).await?;
let sample = imu.read_sample().await?;
```

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the
work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
