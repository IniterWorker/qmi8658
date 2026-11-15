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

{{readme}}

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
