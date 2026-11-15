//! Blocking QMI8658 driver over `embedded-hal` 1.0 traits.

use embedded_hal::{delay::DelayNs, i2c::I2c, spi::SpiDevice};

use crate::config::{
    Config, FifoConfig, InterruptConfig, MotionConfig, MotionEngines, PedometerConfig, TapConfig,
    WomConfig,
};
use crate::data::{
    ActivityStatus, DataStatus, DeviceId, FifoFrameFormat, FifoStatus, GyroGains, RawSample,
    Sample, Scales, SelfTestReport, TapEvent, UniqueId, accel_offset_to_q4_12,
    gyro_offset_to_q11_5, selftest_accel_delta_mg, selftest_gyro_delta_dps,
};
use crate::error::Error;
use crate::interface::{DeviceAddress, I2cInterface, RegisterIo, SpiInterface};
use crate::registers::{
    self, AccelStandaloneOdr, CodStatus, Ctrl1, Ctrl2, Ctrl3, Ctrl7, Ctrl8, Ctrl9Command, FifoCtrl,
    FifoMode, FifoStatusReg, SOFT_RESET_VALUE, StatusInt, WHO_AM_I_VALUE,
};

/// Default Ctrl9 handshake budget. Calibration-on-demand passes a
/// larger budget because its done bit asserts only after ≈1.5 s.
const CTRL9_TIMEOUT_MS: u32 = 100;
const COD_TIMEOUT_MS: u32 = 2000;

/// Blocking QMI8658 driver.
///
/// `IO` is a register transport ([`I2cInterface`] or [`SpiInterface`],
/// via [`new_i2c`](Self::new_i2c) / [`new_spi`](Self::new_spi)) and `D`
/// a blocking delay provider.
pub struct Qmi8658<IO, D> {
    io: IO,
    delay: D,
    config: Config,
    scales: Scales,
    fifo_format: FifoFrameFormat,
    fifo_mode: FifoMode,
}

impl<I, D> Qmi8658<I2cInterface<I>, D>
where
    I: I2c,
    D: DelayNs,
{
    /// Create a driver over an I2C bus. Call [`init`](Self::init) next.
    pub fn new_i2c(bus: I, address: DeviceAddress, delay: D) -> Self {
        Self::with_io(I2cInterface::new(bus, address), delay)
    }
}

impl<S, D> Qmi8658<SpiInterface<S>, D>
where
    S: SpiDevice,
    D: DelayNs,
{
    /// Create a driver over an SPI device (4-wire, host Mode 0 or 3,
    /// MSB first, up to 15 MHz). Call [`init`](Self::init) next.
    pub fn new_spi(device: S, delay: D) -> Self {
        Self::with_io(SpiInterface::new(device), delay)
    }
}

impl<IO, D> Qmi8658<IO, D>
where
    IO: RegisterIo,
    D: DelayNs,
{
    fn with_io(io: IO, delay: D) -> Self {
        Self {
            io,
            delay,
            config: Config::default(),
            scales: Scales::default(),
            fifo_format: FifoFrameFormat::AccelAndGyro,
            fifo_mode: FifoMode::Bypass,
        }
    }

    /// Give back the transport and delay provider.
    pub fn release(self) -> (IO, D) {
        (self.io, self.delay)
    }

    /// The configuration most recently applied (or the default before
    /// the first `init`).
    #[must_use]
    pub const fn config(&self) -> Config {
        self.config
    }

    /// FIFO frame layout implied by the currently configured sensors.
    #[must_use]
    pub const fn fifo_format(&self) -> FifoFrameFormat {
        self.fifo_format
    }

    /// Reset the device, verify its identity, set up the serial
    /// interface (address auto-increment, little-endian output, both
    /// interrupt pins driven), disable AHB clock gating so the
    /// data-locking mechanism is reliable, and apply `config`.
    ///
    /// Wait at least 15 ms after power-up before calling this. After a
    /// successful `init`, gyroscope data is valid once ≈150 ms plus
    /// 3/ODR have elapsed; accelerometer data after ≈3 ms plus 3/ODR.
    /// On failure the device may be left in its reset state; calling
    /// `init` again is safe.
    ///
    /// # Errors
    /// Bus errors, [`Error::InvalidDeviceId`] when WHO_AM_I is not
    /// `0x05`, or [`Error::InvalidConfig`] for an invalid `config`.
    pub fn init(&mut self, config: Config) -> Result<(), Error<IO::Error>> {
        self.io.write_register(registers::RESET, SOFT_RESET_VALUE)?;
        self.delay.delay_ms(15);
        let who = self.io.read_register(registers::WHO_AM_I)?;
        if who != WHO_AM_I_VALUE {
            return Err(Error::InvalidDeviceId { got: who });
        }
        let interrupts = InterruptConfig::default();
        let ctrl1 = Ctrl1 {
            sim_3wire: false,
            addr_ai: true,
            big_endian: false,
            int2_en: interrupts.int2_enable,
            int1_en: interrupts.int1_enable,
            fifo_int_on_int1: interrupts.fifo_interrupt_on_int1,
            sensor_disable: false,
        };
        self.io.write_register(registers::CTRL1, ctrl1.to_bits())?;
        self.set_ahb_clock_gating(false)?;
        self.apply_config(config)?;
        #[cfg(feature = "log")]
        log::info!("qmi8658: init ok, config={config:?}");
        Ok(())
    }

    /// Validate and apply a sensor configuration: sensors are stopped,
    /// CTRL2/CTRL3/CTRL5 rewritten, then the configured sensors are
    /// enabled in CTRL7. The data-ready signal is (re-)enabled on INT2;
    /// use [`configure_interrupts`](Self::configure_interrupts) afterwards
    /// for other routing. Gyroscope data is valid ≈150 ms + 3/ODR after
    /// enabling; this function only waits out the 3 ms accelerometer
    /// turn-on when the accelerometer runs alone.
    ///
    /// # Errors
    /// [`Error::InvalidConfig`] (with no bus writes performed) when both
    /// sensors are disabled, when a standalone/low-power accelerometer
    /// ODR is combined with the gyroscope, or when SyncSample is
    /// requested while the FIFO is not bypassed.
    pub fn apply_config(&mut self, config: Config) -> Result<(), Error<IO::Error>> {
        config
            .validate(self.fifo_mode)
            .map_err(Error::InvalidConfig)?;
        self.io
            .write_register(registers::CTRL7, Ctrl7::default().to_bits())?;
        if let Some(accel) = &config.accel {
            self.io
                .write_register(registers::CTRL2, accel.ctrl2().to_bits())?;
        }
        if let Some(gyro) = &config.gyro {
            self.io
                .write_register(registers::CTRL3, gyro.ctrl3().to_bits())?;
        }
        self.io
            .write_register(registers::CTRL5, config.ctrl5().to_bits())?;
        self.io
            .write_register(registers::CTRL7, config.ctrl7().to_bits())?;
        self.config = config;
        self.scales = config.scales();
        self.fifo_format = config.fifo_format();
        if config.accel.is_some() && config.gyro.is_none() {
            self.delay.delay_ms(3);
        }
        Ok(())
    }

    /// Soft-reset the device and verify WHO_AM_I afterwards. All
    /// device configuration is lost; follow with
    /// `init(self.config())` to restore operation.
    ///
    /// # Errors
    /// Bus errors or [`Error::InvalidDeviceId`].
    pub fn soft_reset(&mut self) -> Result<(), Error<IO::Error>> {
        self.io.write_register(registers::RESET, SOFT_RESET_VALUE)?;
        self.delay.delay_ms(15);
        let who = self.io.read_register(registers::WHO_AM_I)?;
        if who != WHO_AM_I_VALUE {
            return Err(Error::InvalidDeviceId { got: who });
        }
        self.fifo_mode = FifoMode::Bypass;
        Ok(())
    }

    /// Read WHO_AM_I and REVISION_ID (revision reported verbatim;
    /// it differs between silicon variants).
    ///
    /// # Errors
    /// Bus errors.
    pub fn device_id(&mut self) -> Result<DeviceId, Error<IO::Error>> {
        // Single-register reads: address auto-increment is only set
        // up by `init`, and probing identity beforehand must work.
        let who_am_i = self.io.read_register(registers::WHO_AM_I)?;
        let revision = self.io.read_register(registers::REVISION_ID)?;
        Ok(DeviceId { who_am_i, revision })
    }

    /// Copy and read the firmware version and unique serial ID.
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn unique_id(&mut self) -> Result<UniqueId, Error<IO::Error>> {
        self.send_command(Ctrl9Command::CopyUsid, CTRL9_TIMEOUT_MS)?;
        let mut firmware = [0u8; 3];
        self.io.read_block(registers::DQW_L, &mut firmware)?;
        let mut usid = [0u8; 6];
        self.io.read_block(registers::DVX_L, &mut usid)?;
        Ok(UniqueId { firmware, usid })
    }

    /// Read the new-data flags (STATUS0).
    ///
    /// # Errors
    /// Bus errors.
    pub fn data_ready(&mut self) -> Result<DataStatus, Error<IO::Error>> {
        let s = registers::Status0::from_bits(self.io.read_register(registers::STATUS0)?);
        Ok(DataStatus {
            accel_ready: s.accel_ready,
            gyro_ready: s.gyro_ready,
        })
    }

    /// Read one complete raw sample set (timestamp, temperature,
    /// accelerometer, gyroscope) in a single burst.
    ///
    /// In SyncSample mode STATUSINT is read first, which locks the
    /// current sample set; the burst runs through the last gyro byte,
    /// releasing the lock. In Non-SyncSample mode the burst is read
    /// directly.
    ///
    /// # Errors
    /// Bus errors.
    pub fn read_raw(&mut self) -> Result<RawSample, Error<IO::Error>> {
        if self.config.sync_sample {
            let s = StatusInt::from_bits(self.io.read_register(registers::STATUSINT)?);
            if s.avail && !s.locked {
                // Wait out the worst-case data-lock delay (270 µs at
                // the lowest low-power ODR).
                self.delay.delay_us(300);
            }
        }
        let mut buf = [0u8; 17];
        self.io.read_block(registers::TIMESTAMP_L, &mut buf)?;
        Ok(RawSample::from_block(&buf))
    }

    /// Read one sample set converted to g, dps, and °C using the
    /// currently configured ranges.
    ///
    /// # Errors
    /// Bus errors.
    pub fn read_sample(&mut self) -> Result<Sample, Error<IO::Error>> {
        let raw = self.read_raw()?;
        Ok(self.scales.convert(&raw))
    }

    /// Read raw accelerometer counts `[x, y, z]`.
    ///
    /// # Errors
    /// Bus errors.
    pub fn read_accel_raw(&mut self) -> Result<[i16; 3], Error<IO::Error>> {
        let mut buf = [0u8; 6];
        self.io.read_block(registers::AX_L, &mut buf)?;
        Ok(axes(&buf))
    }

    /// Read raw gyroscope counts `[x, y, z]`.
    ///
    /// # Errors
    /// Bus errors.
    pub fn read_gyro_raw(&mut self) -> Result<[i16; 3], Error<IO::Error>> {
        let mut buf = [0u8; 6];
        self.io.read_block(registers::GX_L, &mut buf)?;
        Ok(axes(&buf))
    }

    /// Read the raw temperature value (signed 8.8 fixed point,
    /// 1/256 °C per LSB; see `data::temperature_c`).
    ///
    /// # Errors
    /// Bus errors.
    pub fn read_temperature_raw(&mut self) -> Result<i16, Error<IO::Error>> {
        let mut buf = [0u8; 2];
        self.io.read_block(registers::TEMP_L, &mut buf)?;
        Ok(i16::from_le_bytes(buf))
    }

    /// Read the 24-bit sample timestamp counter.
    ///
    /// # Errors
    /// Bus errors.
    pub fn read_timestamp(&mut self) -> Result<u32, Error<IO::Error>> {
        let mut buf = [0u8; 3];
        self.io.read_block(registers::TIMESTAMP_L, &mut buf)?;
        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    /// Configure the FIFO and reset its contents. Sensor enable state
    /// is not touched. When both sensors feed the FIFO they must run
    /// at the same ODR (device requirement).
    ///
    /// # Errors
    /// [`Error::InvalidConfig`] when SyncSample mode is on and `fifo`
    /// does not bypass the FIFO; bus errors; [`Error::CommandTimeout`].
    pub fn configure_fifo(&mut self, fifo: FifoConfig) -> Result<(), Error<IO::Error>> {
        if self.config.sync_sample && fifo.mode != FifoMode::Bypass {
            return Err(Error::InvalidConfig(
                "FIFO cannot be used in SyncSample mode",
            ));
        }
        self.io
            .write_register(registers::FIFO_WTM_TH, fifo.watermark)?;
        let ctrl = FifoCtrl {
            read_mode: false,
            size: fifo.size,
            mode: fifo.mode,
        };
        self.io
            .write_register(registers::FIFO_CTRL, ctrl.to_bits())?;
        self.send_command(Ctrl9Command::ResetFifo, CTRL9_TIMEOUT_MS)?;
        self.fifo_mode = fifo.mode;
        Ok(())
    }

    /// Read the FIFO fill state.
    ///
    /// # Errors
    /// Bus errors.
    pub fn fifo_status(&mut self) -> Result<FifoStatus, Error<IO::Error>> {
        let mut buf = [0u8; 2];
        self.io.read_block(registers::FIFO_SMPL_CNT, &mut buf)?;
        let reg = FifoStatusReg::from_bits(buf[1]);
        let words = (u16::from(reg.sample_count_msb) << 8) | u16::from(buf[0]);
        Ok(FifoStatus {
            full: reg.full,
            watermark_reached: reg.watermark,
            overflow: reg.overflow,
            not_empty: reg.not_empty,
            bytes_available: 2 * words,
        })
    }

    /// Clear all FIFO data, count, and flags.
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn reset_fifo(&mut self) -> Result<(), Error<IO::Error>> {
        self.send_command(Ctrl9Command::ResetFifo, CTRL9_TIMEOUT_MS)
    }

    /// Drain FIFO bytes into `buf`, returning how many were read
    /// (always a whole number of frames; decode with
    /// `data::FifoFrames`). FIFO filling is paused while draining and
    /// resumes before this returns, including on the error path.
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn read_fifo(&mut self, buf: &mut [u8]) -> Result<usize, Error<IO::Error>> {
        let status = self.fifo_status()?;
        let frame_len = self.fifo_format.frame_len();
        let capacity = buf.len() / frame_len * frame_len;
        let bytes = (status.bytes_available as usize).min(capacity);
        if bytes == 0 {
            return Ok(0);
        }
        self.send_command(Ctrl9Command::RequestFifo, CTRL9_TIMEOUT_MS)?;
        let read_result = self.io.read_block(registers::FIFO_DATA, &mut buf[..bytes]);
        let clear_result = self.modify_register(registers::FIFO_CTRL, |b| {
            FifoCtrl {
                read_mode: false,
                ..FifoCtrl::from_bits(b)
            }
            .to_bits()
        });
        read_result?;
        clear_result?;
        Ok(bytes)
    }

    /// Send tap parameters and enable or disable the tap engine.
    /// Sensors are stopped while the parameters are transferred
    /// (device requirement) and restored afterwards.
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn configure_tap(
        &mut self,
        config: &TapConfig,
        enable: bool,
    ) -> Result<(), Error<IO::Error>> {
        self.with_sensors_off(|this| {
            this.configure_engine(
                config.encode_call1(),
                config.encode_call2(),
                Ctrl9Command::ConfigureTap,
            )?;
            this.modify_ctrl8(|c| Ctrl8 {
                tap_en: enable,
                ..c
            })
        })
    }

    /// Send motion parameters and select which engines run.
    /// Sensors are stopped during the transfer and restored after.
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn configure_motion(
        &mut self,
        config: &MotionConfig,
        enable: MotionEngines,
    ) -> Result<(), Error<IO::Error>> {
        self.with_sensors_off(|this| {
            this.configure_engine(
                config.encode_call1(),
                config.encode_call2(),
                Ctrl9Command::ConfigureMotion,
            )?;
            this.modify_ctrl8(|c| Ctrl8 {
                sig_motion_en: enable.significant_motion,
                no_motion_en: enable.no_motion,
                any_motion_en: enable.any_motion,
                ..c
            })
        })
    }

    /// Send pedometer parameters and enable or disable the engine
    /// (C variant). Sensors are stopped during the transfer and
    /// restored after.
    ///
    /// # Errors
    /// Bus errors, or [`Error::CommandTimeout`] on silicon without a
    /// pedometer.
    pub fn configure_pedometer(
        &mut self,
        config: &PedometerConfig,
        enable: bool,
    ) -> Result<(), Error<IO::Error>> {
        self.with_sensors_off(|this| {
            this.configure_engine(
                config.encode_call1(),
                config.encode_call2(),
                Ctrl9Command::ConfigurePedometer,
            )?;
            this.modify_ctrl8(|c| Ctrl8 {
                pedometer_en: enable,
                ..c
            })
        })
    }

    /// Enter Wake-on-Motion mode (C variant): no sensor data is
    /// produced; the configured interrupt pin toggles on motion.
    /// Requires an accelerometer in the current configuration.
    ///
    /// # Errors
    /// [`Error::InvalidConfig`] without an accelerometer configuration
    /// or with a zero threshold; bus errors; [`Error::CommandTimeout`]
    /// on silicon without WoM.
    pub fn enable_wake_on_motion(&mut self, config: &WomConfig) -> Result<(), Error<IO::Error>> {
        let Some(accel) = self.config.accel else {
            return Err(Error::InvalidConfig(
                "Wake-on-Motion requires an accelerometer configuration",
            ));
        };
        if config.threshold_mg == 0 {
            return Err(Error::InvalidConfig(
                "Wake-on-Motion threshold must be non-zero",
            ));
        }
        self.modify_ctrl7(|c| Ctrl7 {
            accel_en: false,
            ..c
        })?;
        self.io
            .write_register(registers::CTRL2, accel.ctrl2().to_bits())?;
        self.io.write_block(registers::CAL1_L, &config.encode())?;
        self.send_command(Ctrl9Command::WriteWomSetting, CTRL9_TIMEOUT_MS)?;
        self.modify_ctrl7(|c| Ctrl7 {
            accel_en: true,
            ..c
        })
    }

    /// Leave Wake-on-Motion mode and restore the sensor enables from
    /// the current configuration.
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn disable_wake_on_motion(&mut self) -> Result<(), Error<IO::Error>> {
        self.modify_ctrl7(|c| Ctrl7 {
            accel_en: false,
            gyro_en: false,
            ..c
        })?;
        self.io.write_register(registers::CAL1_L, 0)?;
        self.send_command(Ctrl9Command::WriteWomSetting, CTRL9_TIMEOUT_MS)?;
        let (accel_en, gyro_en) = (self.config.accel.is_some(), self.config.gyro.is_some());
        self.modify_ctrl7(|c| Ctrl7 {
            gyro_en,
            accel_en,
            ..c
        })
    }

    /// Read the activity engine event flags (STATUS1).
    ///
    /// # Errors
    /// Bus errors.
    pub fn activity_status(&mut self) -> Result<ActivityStatus, Error<IO::Error>> {
        let s = registers::Status1::from_bits(self.io.read_register(registers::STATUS1)?);
        Ok(ActivityStatus::from_status1(s))
    }

    /// Read and decode the current tap event, if one is flagged.
    ///
    /// # Errors
    /// Bus errors.
    pub fn tap_event(&mut self) -> Result<Option<TapEvent>, Error<IO::Error>> {
        let s = registers::Status1::from_bits(self.io.read_register(registers::STATUS1)?);
        if !s.tap {
            return Ok(None);
        }
        let reg = registers::TapStatusReg::from_bits(self.io.read_register(registers::TAP_STATUS)?);
        Ok(TapEvent::from_reg(reg))
    }

    /// Read the 24-bit pedometer step count (C variant).
    ///
    /// # Errors
    /// Bus errors.
    pub fn step_count(&mut self) -> Result<u32, Error<IO::Error>> {
        let mut buf = [0u8; 3];
        self.io.read_block(registers::STEP_CNT_L, &mut buf)?;
        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    /// Reset the pedometer step count to zero (C variant).
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn reset_step_count(&mut self) -> Result<(), Error<IO::Error>> {
        self.send_command(Ctrl9Command::ResetPedometer, CTRL9_TIMEOUT_MS)
    }

    /// Apply interrupt pin enables and routing (CTRL1/CTRL7/CTRL8
    /// read-modify-writes; unrelated bits preserved).
    ///
    /// # Errors
    /// Bus errors.
    pub fn configure_interrupts(
        &mut self,
        config: &InterruptConfig,
    ) -> Result<(), Error<IO::Error>> {
        let cfg = *config;
        self.modify_register(registers::CTRL1, |b| {
            Ctrl1 {
                int1_en: cfg.int1_enable,
                int2_en: cfg.int2_enable,
                fifo_int_on_int1: cfg.fifo_interrupt_on_int1,
                ..Ctrl1::from_bits(b)
            }
            .to_bits()
        })?;
        self.modify_ctrl8(|c| Ctrl8 {
            activity_on_int1: cfg.activity_interrupt_on_int1,
            ctrl9_handshake_statusint: !cfg.ctrl9_handshake_on_int1,
            ..c
        })?;
        self.modify_ctrl7(|c| Ctrl7 {
            drdy_dis: !cfg.data_ready_enable,
            ..c
        })
    }

    /// Apply host accelerometer delta offsets in g (signed Q4.12 on
    /// the wire). Lost on reset.
    ///
    /// # Errors
    /// [`Error::ValueOutOfRange`] outside `-8 <= g < 8`; bus errors;
    /// [`Error::CommandTimeout`].
    pub fn apply_accel_host_delta_offset(
        &mut self,
        offset_g: [f32; 3],
    ) -> Result<(), Error<IO::Error>> {
        let mut block = [0u8; 6];
        for (i, g) in offset_g.into_iter().enumerate() {
            let raw = accel_offset_to_q4_12(g).ok_or(Error::ValueOutOfRange)?;
            block[i * 2..i * 2 + 2].copy_from_slice(&raw.to_le_bytes());
        }
        self.io.write_block(registers::CAL1_L, &block)?;
        self.send_command(Ctrl9Command::AccelHostDeltaOffset, CTRL9_TIMEOUT_MS)
    }

    /// Apply host gyroscope delta offsets in dps (signed Q11.5 on
    /// the wire). Lost on reset.
    ///
    /// # Errors
    /// [`Error::ValueOutOfRange`] outside `-1024 <= dps < 1024`; bus
    /// errors; [`Error::CommandTimeout`].
    pub fn apply_gyro_host_delta_offset(
        &mut self,
        offset_dps: [f32; 3],
    ) -> Result<(), Error<IO::Error>> {
        let mut block = [0u8; 6];
        for (i, dps) in offset_dps.into_iter().enumerate() {
            let raw = gyro_offset_to_q11_5(dps).ok_or(Error::ValueOutOfRange)?;
            block[i * 2..i * 2 + 2].copy_from_slice(&raw.to_le_bytes());
        }
        self.io.write_block(registers::CAL1_L, &block)?;
        self.send_command(Ctrl9Command::GyroHostDeltaOffset, CTRL9_TIMEOUT_MS)
    }

    /// Run gyroscope calibration-on-demand. Takes ≈1.5 s; keep the
    /// device still. Sensors are stopped for the calibration and
    /// restored afterwards. On success the new gains are returned
    /// (and already applied by the device).
    ///
    /// # Errors
    /// [`Error::CalibrationFailed`] when COD_STATUS reports failure
    /// (inspect [`cod_status`](Self::cod_status)); bus errors;
    /// [`Error::CommandTimeout`].
    pub fn calibrate_gyro_on_demand(&mut self) -> Result<GyroGains, Error<IO::Error>> {
        let saved = Ctrl7::from_bits(self.io.read_register(registers::CTRL7)?);
        self.io.write_register(
            registers::CTRL7,
            Ctrl7 {
                accel_en: false,
                gyro_en: false,
                ..saved
            }
            .to_bits(),
        )?;
        let result = (|| {
            self.send_command(Ctrl9Command::OnDemandCalibration, COD_TIMEOUT_MS)?;
            if !self.cod_status()?.is_success() {
                return Err(Error::CalibrationFailed);
            }
            let mut gains = GyroGains::default();
            self.io.read_block(registers::DVX_L, &mut gains.0)?;
            Ok(gains)
        })();
        let restore = self
            .io
            .write_register(registers::CTRL7, saved.to_bits())
            .map_err(Error::Bus);
        match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }
    }

    /// Restore gyroscope gains saved from an earlier
    /// [`calibrate_gyro_on_demand`](Self::calibrate_gyro_on_demand).
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn apply_gyro_gains(&mut self, gains: &GyroGains) -> Result<(), Error<IO::Error>> {
        let gains = *gains;
        self.with_sensors_off(|this| {
            this.io.write_block(registers::CAL1_L, &gains.0)?;
            this.send_command(Ctrl9Command::ApplyGyroGains, CTRL9_TIMEOUT_MS)
        })
    }

    /// Read the calibration-on-demand status register. Only valid
    /// after a calibration-on-demand run.
    ///
    /// # Errors
    /// Bus errors.
    pub fn cod_status(&mut self) -> Result<CodStatus, Error<IO::Error>> {
        Ok(CodStatus::from_bits(
            self.io.read_register(registers::COD_STATUS)?,
        ))
    }

    /// Run the accelerometer self-test: each axis must respond with
    /// more than 200 mg. Register state is restored afterwards,
    /// including on failure. Report deltas are in milli-g.
    ///
    /// # Errors
    /// [`Error::CommandTimeout`] if the device does not signal
    /// completion; bus errors.
    pub fn self_test_accel(&mut self) -> Result<SelfTestReport, Error<IO::Error>> {
        let saved2 = self.io.read_register(registers::CTRL2)?;
        let saved3 = self.io.read_register(registers::CTRL3)?;
        let saved7 = self.io.read_register(registers::CTRL7)?;
        let result = (|| {
            self.io
                .write_register(registers::CTRL7, Ctrl7::default().to_bits())?;
            self.delay.delay_ms(10);
            let test = Ctrl2 {
                self_test: true,
                odr_bits: AccelStandaloneOdr::Hz1000 as u8,
                ..Ctrl2::from_bits(saved2)
            };
            self.io.write_register(registers::CTRL2, test.to_bits())?;
            self.wait_statusint_avail(true, 500)?;
            self.io.write_register(
                registers::CTRL2,
                Ctrl2 {
                    self_test: false,
                    ..test
                }
                .to_bits(),
            )?;
            self.wait_statusint_avail(false, 500)?;
            let raw = self.read_dvx()?;
            let delta = [
                selftest_accel_delta_mg(raw[0]),
                selftest_accel_delta_mg(raw[1]),
                selftest_accel_delta_mg(raw[2]),
            ];
            Ok(SelfTestReport {
                passed: delta.iter().all(|d| d.abs() > 200.0),
                delta,
            })
        })();
        let restore = self.restore_ctrl2_3_7(saved2, saved3, saved7);
        let report = match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }?;
        #[cfg(feature = "log")]
        log::info!("qmi8658: accel self-test {report:?}");
        Ok(report)
    }

    /// Run the gyroscope self-test: each axis must respond with more
    /// than 300 dps. Takes ≈400 ms. Register state is restored
    /// afterwards, including on failure. Report deltas are in dps.
    ///
    /// # Errors
    /// [`Error::CommandTimeout`] if the device does not signal
    /// completion; bus errors.
    pub fn self_test_gyro(&mut self) -> Result<SelfTestReport, Error<IO::Error>> {
        let saved2 = self.io.read_register(registers::CTRL2)?;
        let saved3 = self.io.read_register(registers::CTRL3)?;
        let saved7 = self.io.read_register(registers::CTRL7)?;
        let result = (|| {
            self.io
                .write_register(registers::CTRL7, Ctrl7::default().to_bits())?;
            self.delay.delay_ms(10);
            // The gyro self-test only triggers with the reset-default
            // full-scale/ODR fields (CTRL3 = 0x80), not with the
            // running configuration; verified on hardware. The saved
            // CTRL3 is restored afterwards.
            let test = Ctrl3 {
                self_test: true,
                ..Ctrl3::default()
            };
            self.io.write_register(registers::CTRL3, test.to_bits())?;
            match self.wait_statusint_avail(true, 700) {
                Ok(()) => {}
                Err(Error::CommandTimeout) => {
                    // Silicon quirk (hardware-verified): the first
                    // gST trigger after an accelerometer self-test is
                    // silently dropped; a second trigger runs within
                    // ~300 ms. Clear and trigger once more.
                    self.io.write_register(
                        registers::CTRL3,
                        Ctrl3 {
                            self_test: false,
                            ..test
                        }
                        .to_bits(),
                    )?;
                    self.delay.delay_ms(10);
                    self.io.write_register(registers::CTRL3, test.to_bits())?;
                    self.wait_statusint_avail(true, 1000)?;
                }
                Err(e) => return Err(e),
            }
            self.io.write_register(
                registers::CTRL3,
                Ctrl3 {
                    self_test: false,
                    ..test
                }
                .to_bits(),
            )?;
            self.wait_statusint_avail(false, 1000)?;
            let raw = self.read_dvx()?;
            let delta = [
                selftest_gyro_delta_dps(raw[0]),
                selftest_gyro_delta_dps(raw[1]),
                selftest_gyro_delta_dps(raw[2]),
            ];
            Ok(SelfTestReport {
                passed: delta.iter().all(|d| d.abs() > 300.0),
                delta,
            })
        })();
        let restore = self.restore_ctrl2_3_7(saved2, saved3, saved7);
        let report = match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }?;
        #[cfg(feature = "log")]
        log::info!("qmi8658: gyro self-test {report:?}");
        Ok(report)
    }

    /// Enable or disable each sensor in CTRL7 and update the cached
    /// configuration to match. Enabling a sensor that has no cached
    /// configuration is rejected; use
    /// [`apply_config`](Self::apply_config) instead.
    ///
    /// # Errors
    /// [`Error::InvalidConfig`] as described; bus errors.
    pub fn set_sensors_enabled(&mut self, accel: bool, gyro: bool) -> Result<(), Error<IO::Error>> {
        if accel && self.config.accel.is_none() {
            return Err(Error::InvalidConfig(
                "no accelerometer configuration cached; use apply_config",
            ));
        }
        if gyro && self.config.gyro.is_none() {
            return Err(Error::InvalidConfig(
                "no gyroscope configuration cached; use apply_config",
            ));
        }
        self.modify_ctrl7(|c| Ctrl7 {
            accel_en: accel,
            gyro_en: gyro,
            ..c
        })?;
        if !accel {
            self.config.accel = None;
        }
        if !gyro {
            self.config.gyro = None;
        }
        self.fifo_format = self.config.fifo_format();
        Ok(())
    }

    /// Put the gyroscope into snooze mode (drive only, no data) or
    /// back to full mode. Only effective while the gyro is enabled.
    ///
    /// # Errors
    /// Bus errors.
    pub fn set_gyro_snooze(&mut self, snooze: bool) -> Result<(), Error<IO::Error>> {
        self.modify_ctrl7(|c| Ctrl7 {
            gyro_snooze: snooze,
            ..c
        })
    }

    /// Switch the internal AHB clock gating. `init` disables gating
    /// because the data-locking mechanism needs it off (on I2C at all
    /// ODRs; on SPI for accel-only below 500 Hz). Re-enabling saves a
    /// little power when locking is never used.
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub fn set_ahb_clock_gating(&mut self, enabled: bool) -> Result<(), Error<IO::Error>> {
        self.io
            .write_register(registers::CAL1_L, u8::from(!enabled))?;
        self.send_command(Ctrl9Command::AhbClockGating, CTRL9_TIMEOUT_MS)
    }

    /// Raw escape hatch: read one register.
    ///
    /// # Errors
    /// Bus errors.
    pub fn read_register(&mut self, reg: u8) -> Result<u8, Error<IO::Error>> {
        Ok(self.io.read_register(reg)?)
    }

    /// Raw escape hatch: write one register.
    ///
    /// # Errors
    /// Bus errors.
    pub fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<IO::Error>> {
        Ok(self.io.write_register(reg, value)?)
    }

    /// Raw escape hatch: burst-read registers starting at `reg`.
    ///
    /// # Errors
    /// Bus errors.
    pub fn read_registers(&mut self, reg: u8, buf: &mut [u8]) -> Result<(), Error<IO::Error>> {
        Ok(self.io.read_block(reg, buf)?)
    }

    /// Raw escape hatch: write registers starting at `reg`. Over SPI
    /// this is one single-register transaction per byte.
    ///
    /// # Errors
    /// Bus errors.
    pub fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<IO::Error>> {
        Ok(self.io.write_block(reg, data)?)
    }

    /// Run the Ctrl9 handshake: write the command, poll STATUSINT
    /// bit 7 at 1 ms intervals until done or `timeout_ms` elapses,
    /// acknowledge, then poll (fresh budget) until the bit clears.
    fn send_command(&mut self, cmd: Ctrl9Command, timeout_ms: u32) -> Result<(), Error<IO::Error>> {
        self.io.write_register(registers::CTRL9, cmd as u8)?;
        self.wait_cmd_done(true, timeout_ms, cmd)?;
        self.io
            .write_register(registers::CTRL9, Ctrl9Command::Ack as u8)?;
        self.wait_cmd_done(false, timeout_ms, cmd)
    }

    fn wait_cmd_done(
        &mut self,
        done: bool,
        timeout_ms: u32,
        cmd: Ctrl9Command,
    ) -> Result<(), Error<IO::Error>> {
        let _ = cmd; // only used by the log call below when `log` is on
        let mut elapsed = 0u32;
        loop {
            let s = StatusInt::from_bits(self.io.read_register(registers::STATUSINT)?);
            if s.cmd_done == done {
                return Ok(());
            }
            if elapsed >= timeout_ms {
                #[cfg(feature = "log")]
                log::warn!("qmi8658: Ctrl9 command {cmd:?} timed out after {timeout_ms} ms");
                return Err(Error::CommandTimeout);
            }
            self.delay.delay_ms(1);
            elapsed += 1;
        }
    }

    fn wait_statusint_avail(
        &mut self,
        avail: bool,
        timeout_ms: u32,
    ) -> Result<(), Error<IO::Error>> {
        let mut elapsed = 0u32;
        loop {
            let s = StatusInt::from_bits(self.io.read_register(registers::STATUSINT)?);
            if s.avail == avail {
                return Ok(());
            }
            if elapsed >= timeout_ms {
                return Err(Error::CommandTimeout);
            }
            self.delay.delay_ms(5);
            elapsed += 5;
        }
    }

    fn modify_register(
        &mut self,
        reg: u8,
        f: impl FnOnce(u8) -> u8,
    ) -> Result<(), Error<IO::Error>> {
        let current = self.io.read_register(reg)?;
        self.io.write_register(reg, f(current))?;
        Ok(())
    }

    fn modify_ctrl7(&mut self, f: impl FnOnce(Ctrl7) -> Ctrl7) -> Result<(), Error<IO::Error>> {
        self.modify_register(registers::CTRL7, |b| f(Ctrl7::from_bits(b)).to_bits())
    }

    fn modify_ctrl8(&mut self, f: impl FnOnce(Ctrl8) -> Ctrl8) -> Result<(), Error<IO::Error>> {
        self.modify_register(registers::CTRL8, |b| f(Ctrl8::from_bits(b)).to_bits())
    }

    /// Stop both sensors, run `f`, then restore the previous CTRL7.
    /// A restore bus error is returned in preference to `f`'s result.
    fn with_sensors_off<R>(
        &mut self,
        f: impl FnOnce(&mut Self) -> Result<R, Error<IO::Error>>,
    ) -> Result<R, Error<IO::Error>> {
        let saved = Ctrl7::from_bits(self.io.read_register(registers::CTRL7)?);
        self.io.write_register(
            registers::CTRL7,
            Ctrl7 {
                accel_en: false,
                gyro_en: false,
                ..saved
            }
            .to_bits(),
        )?;
        let result = f(self);
        let restore = self
            .io
            .write_register(registers::CTRL7, saved.to_bits())
            .map_err(Error::Bus);
        match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }
    }

    /// Two-call Ctrl9 parameter transfer used by the activity engines.
    fn configure_engine(
        &mut self,
        call1: [u8; 8],
        call2: [u8; 8],
        cmd: Ctrl9Command,
    ) -> Result<(), Error<IO::Error>> {
        self.io.write_block(registers::CAL1_L, &call1)?;
        self.send_command(cmd, CTRL9_TIMEOUT_MS)?;
        self.io.write_block(registers::CAL1_L, &call2)?;
        self.send_command(cmd, CTRL9_TIMEOUT_MS)
    }

    fn read_dvx(&mut self) -> Result<[i16; 3], Error<IO::Error>> {
        let mut buf = [0u8; 6];
        self.io.read_block(registers::DVX_L, &mut buf)?;
        Ok(axes(&buf))
    }

    fn restore_ctrl2_3_7(
        &mut self,
        ctrl2: u8,
        ctrl3: u8,
        ctrl7: u8,
    ) -> Result<(), Error<IO::Error>> {
        self.io.write_register(registers::CTRL2, ctrl2)?;
        self.io.write_register(registers::CTRL3, ctrl3)?;
        self.io.write_register(registers::CTRL7, ctrl7)?;
        Ok(())
    }
}

pub(crate) const fn axes(buf: &[u8; 6]) -> [i16; 3] {
    [
        i16::from_le_bytes([buf[0], buf[1]]),
        i16::from_le_bytes([buf[2], buf[3]]),
        i16::from_le_bytes([buf[4], buf[5]]),
    ]
}
