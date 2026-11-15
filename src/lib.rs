//! # QMI8658 6-axis IMU driver
//!
//! A `no_std` driver for the QST QMI8658 inertial measurement unit,
//! built on `embedded-hal` 1.0 (blocking, [`Qmi8658`]) and, behind the
//! `async` feature, `embedded-hal-async` 1.0 ([`Qmi8658Async`]). Both
//! drivers work over I2C and SPI (4-wire) and share one typed
//! configuration and data model.
//!
//! ## Features
//!
//! | Feature | Default | Effect |
//! |---|---|---|
//! | `async` | no | async driver [`Qmi8658Async`] over `embedded-hal-async` |
//! | `defmt` | no | `defmt::Format` implementations for public types |
//! | `log` | no | `log` diagnostics from init, Ctrl9 timeouts, self-tests |
//!
//! The blocking driver is always available.
//!
//! ## Wiring notes
//!
//! - I2C address is selected by the SA0 pin: high or floating gives
//!   [`DeviceAddress::Primary`] (`0x6A`), low gives
//!   [`DeviceAddress::Secondary`] (`0x6B`).
//! - SPI: the sensor auto-detects Mode 0/3; configure the host for
//!   Mode 0, MSB first, at most 15 MHz, 4-wire.
//! - Wait at least 15 ms after power-up before `init`; gyroscope
//!   data is valid ≈150 ms + 3/ODR after the gyro is enabled.
//! - While the FIFO is being drained its filling is paused, so read
//!   it promptly.
//!
//! ## Silicon variants
//!
//! Pedometer and Wake-on-Motion are documented as C-variant features;
//! the ±2048 dps gyro range and the CTRL1 interrupt-pin enable bits
//! as A/B-variant features. All features are exposed
//! unconditionally; commands the silicon does not implement fail
//! with [`Error::CommandTimeout`]. The revision ID is reported
//! verbatim, never asserted. Hardware-verified example: the part on
//! the ESP32-S3 reference board reports revision `0x7C` yet accepts
//! and runs the pedometer configuration, so do not gate features on
//! the revision byte.
//!
//! SPI support is verified by transaction-level mock tests; the
//! reference board used for hardware testing wires the sensor over
//! I2C.
//!
//! ## Quickstart (blocking, I2C)
//!
//! Shown against a mock bus so the example runs as a doctest; on
//! hardware, pass your HAL's I2C and delay types instead.
//!
//! ```
//! use embedded_hal_mock::eh1::{delay::NoopDelay, i2c::{Mock, Transaction}};
//! use qmi8658::{DeviceAddress, Qmi8658, config::Config};
//!
//! # let mut block = vec![0u8; 17];
//! # block[9..11].copy_from_slice(&8192i16.to_le_bytes()); // AZ = 1 g at ±4 g
//! # let script = [
//! #     Transaction::write(0x6A, vec![0x60, 0xB0]),
//! #     Transaction::write_read(0x6A, vec![0x00], vec![0x05]),
//! #     Transaction::write(0x6A, vec![0x02, 0x58]),
//! #     Transaction::write(0x6A, vec![0x0B, 0x01]),
//! #     Transaction::write(0x6A, vec![0x0A, 0x12]),
//! #     Transaction::write_read(0x6A, vec![0x2D], vec![0x80]),
//! #     Transaction::write(0x6A, vec![0x0A, 0x00]),
//! #     Transaction::write_read(0x6A, vec![0x2D], vec![0x00]),
//! #     Transaction::write(0x6A, vec![0x08, 0x00]),
//! #     Transaction::write(0x6A, vec![0x03, 0x15]),
//! #     Transaction::write(0x6A, vec![0x04, 0x55]),
//! #     Transaction::write(0x6A, vec![0x06, 0x00]),
//! #     Transaction::write(0x6A, vec![0x08, 0x03]),
//! #     Transaction::write_read(0x6A, vec![0x30], block),
//! # ];
//! // `script` holds the mock I2C transaction script (elided here).
//! let mut imu = Qmi8658::new_i2c(Mock::new(&script), DeviceAddress::Primary, NoopDelay {});
//! imu.init(Config::default()).unwrap();
//! let sample = imu.read_sample().unwrap();
//! assert!((sample.accel_g[2] - 1.0).abs() < 1e-6);
//! # let (iface, _) = imu.release();
//! # iface.release().done();
//! ```
//!
//! ## Async (Embassy-style)
//!
//! The async driver is executor-agnostic; it only needs
//! `embedded-hal-async` bus and delay implementations from your HAL.
//!
//! ```ignore
//! use qmi8658::{DeviceAddress, Qmi8658Async, config::Config};
//!
//! // `i2c` and `delay` are your HAL's async types (for example an
//! // Embassy-time-backed delay and an async I2C peripheral).
//! let mut imu = Qmi8658Async::new_i2c(i2c, DeviceAddress::Primary, delay);
//! imu.init(Config::default()).await?;
//! let sample = imu.read_sample().await?;
//! ```

#![cfg_attr(not(test), no_std)]
// clippy warning level lints
#![warn(
    missing_docs,
    clippy::pedantic,
    clippy::nursery,
    clippy::dbg_macro,
    clippy::unwrap_used,
    clippy::map_err_ignore,
    clippy::panic,
    clippy::unimplemented,
    clippy::unreachable,
    clippy::clone_on_ref_ptr,
    clippy::create_dir,
    clippy::exit,
    clippy::filetype_is_file,
    clippy::float_cmp_const,
    clippy::indexing_slicing,
    clippy::let_underscore_must_use,
    clippy::lossy_float_literal,
    clippy::string_slice,
    clippy::try_err
)]
// clippy deny/error level lints, they always have a quick fix that should be preferred
#![deny(
    clippy::multiple_inherent_impl,
    clippy::rc_buffer,
    clippy::rc_mutex,
    clippy::rest_pat_in_fully_bound_structs,
    clippy::self_named_module_files,
    clippy::separated_literal_suffix,
    clippy::str_to_string,
    clippy::string_add,
    clippy::unnecessary_self_imports,
    clippy::unneeded_field_pattern,
    clippy::verbose_file_reads
)]
// allowed rules
#![allow(
    clippy::module_name_repetitions,
    clippy::similar_names,
    clippy::cast_possible_truncation,
    clippy::redundant_pub_crate,
    clippy::indexing_slicing,
    // Register/config structs faithfully model hardware bit fields and
    // datasheet names; these style lints fight that modelling.
    clippy::struct_excessive_bools,
    clippy::doc_markdown,
    clippy::trivially_copy_pass_by_ref,
    // Match ergonomics (`Some(x)` against `&Option<_>`) is idiomatic here.
    clippy::pattern_type_mismatch
)]
#![cfg_attr(test, allow(clippy::unwrap_used, clippy::panic))]

/// Typed configuration types and `const` builders.
pub mod config;
/// Measurement output types and conversions.
pub mod data;
/// Bus transports (I2C/SPI) and the register-access traits.
pub mod interface;
/// Register addresses, bit layouts, and field encodings.
pub mod registers;

/// Common imports for application code.
pub mod prelude;

mod driver;
#[cfg(feature = "async")]
mod driver_async;
mod error;

pub use driver::Qmi8658;
#[cfg(feature = "async")]
pub use driver_async::Qmi8658Async;
pub use error::Error;
pub use interface::DeviceAddress;
