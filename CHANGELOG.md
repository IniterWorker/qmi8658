# Changelog

Rust `no_std` driver for the QMI8658 6-axis inertial measurement unit.

<!-- next-header -->

## [Unreleased] - ReleaseDate

## [0.2.0] - 2026-11-15

Complete rewrite of the driver (breaking).

### Added

- Async driver `Qmi8658Async` over `embedded-hal-async` 1.0, behind the
  `async` feature; executor-agnostic.
- SPI (4-wire) transport for both drivers, alongside I2C: first byte
  carries the read flag in bit 7 and the 7-bit register address
  (per the QST datasheet SPI protocol figure). Transaction-test
  verified; hardware confirmation of SPI is still pending because
  the reference board wires the sensor over I2C.
- Typed, `const`-builder configuration (`Config`, `AccelConfig`,
  `GyroConfig`, `FifoConfig`, `InterruptConfig`) and full feature
  coverage: FIFO, tap / any- / no- / significant-motion engines,
  pedometer and Wake-on-Motion (C variant), host delta offsets,
  gyro calibration-on-demand with gain save/restore, self-tests,
  SyncSample data locking.
- `defmt` feature deriving `defmt::Format` on public types.
- Raw register escape hatch (`read_register`/`write_register`/…)
  and a public `registers` module with the register map.
- Nix flake development environment with a sandboxed host-test
  check; CI runs `nix flake check` and `just ci`.

### Changed

- **Breaking:** the old raw `get_ctrlX`/`set_ctrlX` API is gone;
  the crate is now `Qmi8658::new_i2c`/`new_spi` + `init(Config)`.
  No compatibility aliases.
- **Breaking:** feature `loglib` renamed to `log`.
- Edition 2024, MSRV 1.85. Dependencies `bitfield` and `libm`
  removed; register encode/decode is hand-written const code.

### Fixed (relative to 0.1.0)

- Y-acceleration no longer mixes the Y high byte with the X low byte.
- One consistent gyro scaling (`raw / LSB_per_dps`) in the register
  and FIFO paths.
- Temperature uses the datasheet `TEMP_H + TEMP_L/256` formula
  (was a divisor of 265).
- Soft reset writes the datasheet value `0xB0` to RESET (was `0x4D`).
- Ctrl9 handshake no longer maps unknown command bytes to `Ack`,
  and its retry budget is reset between the done/ack phases.
- No `unsafe` transmutation or panics on unknown register encodings;
  reserved values are masked to documented variants.
