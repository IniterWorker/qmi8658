//! Async QMI8658 driver over `embedded-hal-async` 1.0 traits.
//!
//! Method-for-method mirror of the blocking [`crate::Qmi8658`] driver;
//! all register encodings and conversions come from the same pure
//! core, and every wait is an awaited delay (never a busy spin), so
//! the driver cooperates with any executor.

use embedded_hal_async::{delay::DelayNs, i2c::I2c, spi::SpiDevice};

use crate::config::{
    Config, FifoConfig, InterruptConfig, MotionConfig, MotionEngines, PedometerConfig, TapConfig,
    WomConfig,
};
use crate::data::{
    ActivityStatus, DataStatus, DeviceId, FifoFrameFormat, FifoStatus, GyroGains, RawSample,
    Sample, Scales, SelfTestReport, TapEvent, UniqueId, accel_offset_to_q4_12,
    gyro_offset_to_q11_5, selftest_accel_delta_mg, selftest_gyro_delta_dps,
};
use crate::driver::axes;
use crate::error::Error;
use crate::interface::{AsyncRegisterIo, DeviceAddress, I2cInterface, SpiInterface};
use crate::registers::{
    self, AccelStandaloneOdr, CodStatus, Ctrl1, Ctrl2, Ctrl3, Ctrl7, Ctrl8, Ctrl9Command, FifoCtrl,
    FifoMode, FifoStatusReg, SOFT_RESET_VALUE, StatusInt, WHO_AM_I_VALUE,
};

const CTRL9_TIMEOUT_MS: u32 = 100;
const COD_TIMEOUT_MS: u32 = 2000;

/// Async QMI8658 driver. See [`crate::Qmi8658`] for the semantics of
/// each operation; only the bus and delay access differs (awaited).
pub struct Qmi8658Async<IO, D> {
    io: IO,
    delay: D,
    config: Config,
    scales: Scales,
    fifo_format: FifoFrameFormat,
    fifo_mode: FifoMode,
}

impl<I, D> Qmi8658Async<I2cInterface<I>, D>
where
    I: I2c,
    D: DelayNs,
{
    /// Create a driver over an async I2C bus. Call `init` next.
    pub fn new_i2c(bus: I, address: DeviceAddress, delay: D) -> Self {
        Self::with_io(I2cInterface::new(bus, address), delay)
    }
}

impl<S, D> Qmi8658Async<SpiInterface<S>, D>
where
    S: SpiDevice,
    D: DelayNs,
{
    /// Create a driver over an async SPI device (4-wire, host Mode 0
    /// or 3, MSB first, up to 15 MHz). Call `init` next.
    pub fn new_spi(device: S, delay: D) -> Self {
        Self::with_io(SpiInterface::new(device), delay)
    }
}

impl<IO, D> Qmi8658Async<IO, D>
where
    IO: AsyncRegisterIo,
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

    /// Async mirror of [`crate::Qmi8658::init`].
    ///
    /// # Errors
    /// Bus errors, [`Error::InvalidDeviceId`], or
    /// [`Error::InvalidConfig`].
    pub async fn init(&mut self, config: Config) -> Result<(), Error<IO::Error>> {
        self.io
            .write_register(registers::RESET, SOFT_RESET_VALUE)
            .await?;
        self.delay.delay_ms(15).await;
        let who = self.io.read_register(registers::WHO_AM_I).await?;
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
        self.io
            .write_register(registers::CTRL1, ctrl1.to_bits())
            .await?;
        self.set_ahb_clock_gating(false).await?;
        self.apply_config(config).await?;
        #[cfg(feature = "log")]
        log::info!("qmi8658: init ok, config={config:?}");
        Ok(())
    }

    /// Async mirror of [`crate::Qmi8658::apply_config`].
    ///
    /// # Errors
    /// [`Error::InvalidConfig`] (no bus writes performed) or bus errors.
    pub async fn apply_config(&mut self, config: Config) -> Result<(), Error<IO::Error>> {
        config
            .validate(self.fifo_mode)
            .map_err(Error::InvalidConfig)?;
        self.io
            .write_register(registers::CTRL7, Ctrl7::default().to_bits())
            .await?;
        if let Some(accel) = &config.accel {
            self.io
                .write_register(registers::CTRL2, accel.ctrl2().to_bits())
                .await?;
        }
        if let Some(gyro) = &config.gyro {
            self.io
                .write_register(registers::CTRL3, gyro.ctrl3().to_bits())
                .await?;
        }
        self.io
            .write_register(registers::CTRL5, config.ctrl5().to_bits())
            .await?;
        self.io
            .write_register(registers::CTRL7, config.ctrl7().to_bits())
            .await?;
        self.config = config;
        self.scales = config.scales();
        self.fifo_format = config.fifo_format();
        if config.accel.is_some() && config.gyro.is_none() {
            self.delay.delay_ms(3).await;
        }
        Ok(())
    }

    /// Async mirror of [`crate::Qmi8658::soft_reset`].
    ///
    /// # Errors
    /// Bus errors or [`Error::InvalidDeviceId`].
    pub async fn soft_reset(&mut self) -> Result<(), Error<IO::Error>> {
        self.io
            .write_register(registers::RESET, SOFT_RESET_VALUE)
            .await?;
        self.delay.delay_ms(15).await;
        let who = self.io.read_register(registers::WHO_AM_I).await?;
        if who != WHO_AM_I_VALUE {
            return Err(Error::InvalidDeviceId { got: who });
        }
        self.fifo_mode = FifoMode::Bypass;
        Ok(())
    }

    /// Async mirror of [`crate::Qmi8658::device_id`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn device_id(&mut self) -> Result<DeviceId, Error<IO::Error>> {
        let mut buf = [0u8; 2];
        self.io.read_block(registers::WHO_AM_I, &mut buf).await?;
        Ok(DeviceId {
            who_am_i: buf[0],
            revision: buf[1],
        })
    }

    /// Async mirror of [`crate::Qmi8658::unique_id`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn unique_id(&mut self) -> Result<UniqueId, Error<IO::Error>> {
        self.send_command(Ctrl9Command::CopyUsid, CTRL9_TIMEOUT_MS)
            .await?;
        let mut firmware = [0u8; 3];
        self.io.read_block(registers::DQW_L, &mut firmware).await?;
        let mut usid = [0u8; 6];
        self.io.read_block(registers::DVX_L, &mut usid).await?;
        Ok(UniqueId { firmware, usid })
    }

    /// Async mirror of [`crate::Qmi8658::data_ready`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn data_ready(&mut self) -> Result<DataStatus, Error<IO::Error>> {
        let s = registers::Status0::from_bits(self.io.read_register(registers::STATUS0).await?);
        Ok(DataStatus {
            accel_ready: s.accel_ready,
            gyro_ready: s.gyro_ready,
        })
    }

    /// Async mirror of [`crate::Qmi8658::read_raw`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn read_raw(&mut self) -> Result<RawSample, Error<IO::Error>> {
        if self.config.sync_sample {
            let s = StatusInt::from_bits(self.io.read_register(registers::STATUSINT).await?);
            if s.avail && !s.locked {
                self.delay.delay_us(300).await;
            }
        }
        let mut buf = [0u8; 17];
        self.io.read_block(registers::TIMESTAMP_L, &mut buf).await?;
        Ok(RawSample::from_block(&buf))
    }

    /// Async mirror of [`crate::Qmi8658::read_sample`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn read_sample(&mut self) -> Result<Sample, Error<IO::Error>> {
        let raw = self.read_raw().await?;
        Ok(self.scales.convert(&raw))
    }

    /// Async mirror of [`crate::Qmi8658::read_accel_raw`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn read_accel_raw(&mut self) -> Result<[i16; 3], Error<IO::Error>> {
        let mut buf = [0u8; 6];
        self.io.read_block(registers::AX_L, &mut buf).await?;
        Ok(axes(&buf))
    }

    /// Async mirror of [`crate::Qmi8658::read_gyro_raw`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn read_gyro_raw(&mut self) -> Result<[i16; 3], Error<IO::Error>> {
        let mut buf = [0u8; 6];
        self.io.read_block(registers::GX_L, &mut buf).await?;
        Ok(axes(&buf))
    }

    /// Async mirror of [`crate::Qmi8658::read_temperature_raw`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn read_temperature_raw(&mut self) -> Result<i16, Error<IO::Error>> {
        let mut buf = [0u8; 2];
        self.io.read_block(registers::TEMP_L, &mut buf).await?;
        Ok(i16::from_le_bytes(buf))
    }

    /// Async mirror of [`crate::Qmi8658::read_timestamp`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn read_timestamp(&mut self) -> Result<u32, Error<IO::Error>> {
        let mut buf = [0u8; 3];
        self.io.read_block(registers::TIMESTAMP_L, &mut buf).await?;
        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    /// Async mirror of [`crate::Qmi8658::configure_fifo`].
    ///
    /// # Errors
    /// [`Error::InvalidConfig`], bus errors, or
    /// [`Error::CommandTimeout`].
    pub async fn configure_fifo(&mut self, fifo: FifoConfig) -> Result<(), Error<IO::Error>> {
        if self.config.sync_sample && fifo.mode != FifoMode::Bypass {
            return Err(Error::InvalidConfig(
                "FIFO cannot be used in SyncSample mode",
            ));
        }
        self.io
            .write_register(registers::FIFO_WTM_TH, fifo.watermark)
            .await?;
        let ctrl = FifoCtrl {
            read_mode: false,
            size: fifo.size,
            mode: fifo.mode,
        };
        self.io
            .write_register(registers::FIFO_CTRL, ctrl.to_bits())
            .await?;
        self.send_command(Ctrl9Command::ResetFifo, CTRL9_TIMEOUT_MS)
            .await?;
        self.fifo_mode = fifo.mode;
        Ok(())
    }

    /// Async mirror of [`crate::Qmi8658::fifo_status`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn fifo_status(&mut self) -> Result<FifoStatus, Error<IO::Error>> {
        let mut buf = [0u8; 2];
        self.io
            .read_block(registers::FIFO_SMPL_CNT, &mut buf)
            .await?;
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

    /// Async mirror of [`crate::Qmi8658::reset_fifo`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn reset_fifo(&mut self) -> Result<(), Error<IO::Error>> {
        self.send_command(Ctrl9Command::ResetFifo, CTRL9_TIMEOUT_MS)
            .await
    }

    /// Async mirror of [`crate::Qmi8658::read_fifo`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn read_fifo(&mut self, buf: &mut [u8]) -> Result<usize, Error<IO::Error>> {
        let status = self.fifo_status().await?;
        let frame_len = self.fifo_format.frame_len();
        let capacity = buf.len() / frame_len * frame_len;
        let bytes = (status.bytes_available as usize).min(capacity);
        if bytes == 0 {
            return Ok(0);
        }
        self.send_command(Ctrl9Command::RequestFifo, CTRL9_TIMEOUT_MS)
            .await?;
        let read_result = self
            .io
            .read_block(registers::FIFO_DATA, &mut buf[..bytes])
            .await;
        let clear_result = self
            .modify_register(registers::FIFO_CTRL, |b| {
                FifoCtrl {
                    read_mode: false,
                    ..FifoCtrl::from_bits(b)
                }
                .to_bits()
            })
            .await;
        read_result?;
        clear_result?;
        Ok(bytes)
    }

    /// Async mirror of [`crate::Qmi8658::configure_tap`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn configure_tap(
        &mut self,
        config: &TapConfig,
        enable: bool,
    ) -> Result<(), Error<IO::Error>> {
        let saved = self.sensors_off().await?;
        let result = async {
            self.configure_engine(
                config.encode_call1(),
                config.encode_call2(),
                Ctrl9Command::ConfigureTap,
            )
            .await?;
            self.modify_ctrl8(|c| Ctrl8 {
                tap_en: enable,
                ..c
            })
            .await
        }
        .await;
        let restore = self.restore_ctrl7(saved).await;
        match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }
    }

    /// Async mirror of [`crate::Qmi8658::configure_motion`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn configure_motion(
        &mut self,
        config: &MotionConfig,
        enable: MotionEngines,
    ) -> Result<(), Error<IO::Error>> {
        let saved = self.sensors_off().await?;
        let result = async {
            self.configure_engine(
                config.encode_call1(),
                config.encode_call2(),
                Ctrl9Command::ConfigureMotion,
            )
            .await?;
            self.modify_ctrl8(|c| Ctrl8 {
                sig_motion_en: enable.significant_motion,
                no_motion_en: enable.no_motion,
                any_motion_en: enable.any_motion,
                ..c
            })
            .await
        }
        .await;
        let restore = self.restore_ctrl7(saved).await;
        match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }
    }

    /// Async mirror of [`crate::Qmi8658::configure_pedometer`].
    ///
    /// # Errors
    /// Bus errors, or [`Error::CommandTimeout`] on silicon without a
    /// pedometer.
    pub async fn configure_pedometer(
        &mut self,
        config: &PedometerConfig,
        enable: bool,
    ) -> Result<(), Error<IO::Error>> {
        let saved = self.sensors_off().await?;
        let result = async {
            self.configure_engine(
                config.encode_call1(),
                config.encode_call2(),
                Ctrl9Command::ConfigurePedometer,
            )
            .await?;
            self.modify_ctrl8(|c| Ctrl8 {
                pedometer_en: enable,
                ..c
            })
            .await
        }
        .await;
        let restore = self.restore_ctrl7(saved).await;
        match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }
    }

    /// Async mirror of [`crate::Qmi8658::enable_wake_on_motion`].
    ///
    /// # Errors
    /// [`Error::InvalidConfig`], bus errors, or
    /// [`Error::CommandTimeout`].
    pub async fn enable_wake_on_motion(
        &mut self,
        config: &WomConfig,
    ) -> Result<(), Error<IO::Error>> {
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
        })
        .await?;
        self.io
            .write_register(registers::CTRL2, accel.ctrl2().to_bits())
            .await?;
        self.io
            .write_block(registers::CAL1_L, &config.encode())
            .await?;
        self.send_command(Ctrl9Command::WriteWomSetting, CTRL9_TIMEOUT_MS)
            .await?;
        self.modify_ctrl7(|c| Ctrl7 {
            accel_en: true,
            ..c
        })
        .await
    }

    /// Async mirror of [`crate::Qmi8658::disable_wake_on_motion`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn disable_wake_on_motion(&mut self) -> Result<(), Error<IO::Error>> {
        self.modify_ctrl7(|c| Ctrl7 {
            gyro_en: false,
            accel_en: false,
            ..c
        })
        .await?;
        self.io.write_register(registers::CAL1_L, 0).await?;
        self.send_command(Ctrl9Command::WriteWomSetting, CTRL9_TIMEOUT_MS)
            .await?;
        let (accel_en, gyro_en) = (self.config.accel.is_some(), self.config.gyro.is_some());
        self.modify_ctrl7(|c| Ctrl7 {
            gyro_en,
            accel_en,
            ..c
        })
        .await
    }

    /// Async mirror of [`crate::Qmi8658::activity_status`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn activity_status(&mut self) -> Result<ActivityStatus, Error<IO::Error>> {
        let s = registers::Status1::from_bits(self.io.read_register(registers::STATUS1).await?);
        Ok(ActivityStatus::from_status1(s))
    }

    /// Async mirror of [`crate::Qmi8658::tap_event`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn tap_event(&mut self) -> Result<Option<TapEvent>, Error<IO::Error>> {
        let s = registers::Status1::from_bits(self.io.read_register(registers::STATUS1).await?);
        if !s.tap {
            return Ok(None);
        }
        let reg =
            registers::TapStatusReg::from_bits(self.io.read_register(registers::TAP_STATUS).await?);
        Ok(TapEvent::from_reg(reg))
    }

    /// Async mirror of [`crate::Qmi8658::step_count`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn step_count(&mut self) -> Result<u32, Error<IO::Error>> {
        let mut buf = [0u8; 3];
        self.io.read_block(registers::STEP_CNT_L, &mut buf).await?;
        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    /// Async mirror of [`crate::Qmi8658::reset_step_count`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn reset_step_count(&mut self) -> Result<(), Error<IO::Error>> {
        self.send_command(Ctrl9Command::ResetPedometer, CTRL9_TIMEOUT_MS)
            .await
    }

    /// Async mirror of [`crate::Qmi8658::configure_interrupts`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn configure_interrupts(
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
        })
        .await?;
        self.modify_ctrl8(|c| Ctrl8 {
            activity_on_int1: cfg.activity_interrupt_on_int1,
            ctrl9_handshake_statusint: !cfg.ctrl9_handshake_on_int1,
            ..c
        })
        .await?;
        self.modify_ctrl7(|c| Ctrl7 {
            drdy_dis: !cfg.data_ready_enable,
            ..c
        })
        .await
    }

    /// Async mirror of [`crate::Qmi8658::apply_accel_host_delta_offset`].
    ///
    /// # Errors
    /// [`Error::ValueOutOfRange`], bus errors, or
    /// [`Error::CommandTimeout`].
    pub async fn apply_accel_host_delta_offset(
        &mut self,
        offset_g: [f32; 3],
    ) -> Result<(), Error<IO::Error>> {
        let mut block = [0u8; 6];
        for (i, g) in offset_g.into_iter().enumerate() {
            let raw = accel_offset_to_q4_12(g).ok_or(Error::ValueOutOfRange)?;
            block[i * 2..i * 2 + 2].copy_from_slice(&raw.to_le_bytes());
        }
        self.io.write_block(registers::CAL1_L, &block).await?;
        self.send_command(Ctrl9Command::AccelHostDeltaOffset, CTRL9_TIMEOUT_MS)
            .await
    }

    /// Async mirror of [`crate::Qmi8658::apply_gyro_host_delta_offset`].
    ///
    /// # Errors
    /// [`Error::ValueOutOfRange`], bus errors, or
    /// [`Error::CommandTimeout`].
    pub async fn apply_gyro_host_delta_offset(
        &mut self,
        offset_dps: [f32; 3],
    ) -> Result<(), Error<IO::Error>> {
        let mut block = [0u8; 6];
        for (i, dps) in offset_dps.into_iter().enumerate() {
            let raw = gyro_offset_to_q11_5(dps).ok_or(Error::ValueOutOfRange)?;
            block[i * 2..i * 2 + 2].copy_from_slice(&raw.to_le_bytes());
        }
        self.io.write_block(registers::CAL1_L, &block).await?;
        self.send_command(Ctrl9Command::GyroHostDeltaOffset, CTRL9_TIMEOUT_MS)
            .await
    }

    /// Async mirror of [`crate::Qmi8658::calibrate_gyro_on_demand`].
    ///
    /// # Errors
    /// [`Error::CalibrationFailed`], bus errors, or
    /// [`Error::CommandTimeout`].
    pub async fn calibrate_gyro_on_demand(&mut self) -> Result<GyroGains, Error<IO::Error>> {
        let saved = self.sensors_off().await?;
        let result = async {
            self.send_command(Ctrl9Command::OnDemandCalibration, COD_TIMEOUT_MS)
                .await?;
            if !self.cod_status().await?.is_success() {
                return Err(Error::CalibrationFailed);
            }
            let mut gains = GyroGains::default();
            self.io.read_block(registers::DVX_L, &mut gains.0).await?;
            Ok(gains)
        }
        .await;
        let restore = self.restore_ctrl7(saved).await;
        match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }
    }

    /// Async mirror of [`crate::Qmi8658::apply_gyro_gains`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn apply_gyro_gains(&mut self, gains: &GyroGains) -> Result<(), Error<IO::Error>> {
        let gains = *gains;
        let saved = self.sensors_off().await?;
        let result = async {
            self.io.write_block(registers::CAL1_L, &gains.0).await?;
            self.send_command(Ctrl9Command::ApplyGyroGains, CTRL9_TIMEOUT_MS)
                .await
        }
        .await;
        let restore = self.restore_ctrl7(saved).await;
        match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }
    }

    /// Async mirror of [`crate::Qmi8658::cod_status`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn cod_status(&mut self) -> Result<CodStatus, Error<IO::Error>> {
        Ok(CodStatus::from_bits(
            self.io.read_register(registers::COD_STATUS).await?,
        ))
    }

    /// Async mirror of [`crate::Qmi8658::self_test_accel`].
    ///
    /// # Errors
    /// [`Error::CommandTimeout`] or bus errors.
    pub async fn self_test_accel(&mut self) -> Result<SelfTestReport, Error<IO::Error>> {
        let saved2 = self.io.read_register(registers::CTRL2).await?;
        let saved3 = self.io.read_register(registers::CTRL3).await?;
        let saved7 = self.io.read_register(registers::CTRL7).await?;
        let result = async {
            self.io
                .write_register(registers::CTRL7, Ctrl7::default().to_bits())
                .await?;
            self.delay.delay_ms(10).await;
            let test = Ctrl2 {
                self_test: true,
                odr_bits: AccelStandaloneOdr::Hz1000 as u8,
                ..Ctrl2::from_bits(saved2)
            };
            self.io
                .write_register(registers::CTRL2, test.to_bits())
                .await?;
            self.wait_statusint_avail(true, 500).await?;
            self.io
                .write_register(
                    registers::CTRL2,
                    Ctrl2 {
                        self_test: false,
                        ..test
                    }
                    .to_bits(),
                )
                .await?;
            self.wait_statusint_avail(false, 500).await?;
            let raw = self.read_dvx().await?;
            let delta = [
                selftest_accel_delta_mg(raw[0]),
                selftest_accel_delta_mg(raw[1]),
                selftest_accel_delta_mg(raw[2]),
            ];
            Ok(SelfTestReport {
                passed: delta.iter().all(|d| d.abs() > 200.0),
                delta,
            })
        }
        .await;
        let restore = self.restore_ctrl2_3_7(saved2, saved3, saved7).await;
        let report = match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }?;
        #[cfg(feature = "log")]
        log::info!("qmi8658: accel self-test {report:?}");
        Ok(report)
    }

    /// Async mirror of [`crate::Qmi8658::self_test_gyro`].
    ///
    /// # Errors
    /// [`Error::CommandTimeout`] or bus errors.
    pub async fn self_test_gyro(&mut self) -> Result<SelfTestReport, Error<IO::Error>> {
        let saved2 = self.io.read_register(registers::CTRL2).await?;
        let saved3 = self.io.read_register(registers::CTRL3).await?;
        let saved7 = self.io.read_register(registers::CTRL7).await?;
        let result = async {
            self.io
                .write_register(registers::CTRL7, Ctrl7::default().to_bits())
                .await?;
            self.delay.delay_ms(10).await;
            let test = Ctrl3 {
                self_test: true,
                ..Ctrl3::from_bits(saved3)
            };
            self.io
                .write_register(registers::CTRL3, test.to_bits())
                .await?;
            self.wait_statusint_avail(true, 1000).await?;
            self.io
                .write_register(
                    registers::CTRL3,
                    Ctrl3 {
                        self_test: false,
                        ..test
                    }
                    .to_bits(),
                )
                .await?;
            self.wait_statusint_avail(false, 1000).await?;
            let raw = self.read_dvx().await?;
            let delta = [
                selftest_gyro_delta_dps(raw[0]),
                selftest_gyro_delta_dps(raw[1]),
                selftest_gyro_delta_dps(raw[2]),
            ];
            Ok(SelfTestReport {
                passed: delta.iter().all(|d| d.abs() > 300.0),
                delta,
            })
        }
        .await;
        let restore = self.restore_ctrl2_3_7(saved2, saved3, saved7).await;
        let report = match (result, restore) {
            (_, Err(e)) => Err(e),
            (r, Ok(())) => r,
        }?;
        #[cfg(feature = "log")]
        log::info!("qmi8658: gyro self-test {report:?}");
        Ok(report)
    }

    /// Async mirror of [`crate::Qmi8658::set_sensors_enabled`].
    ///
    /// # Errors
    /// [`Error::InvalidConfig`] or bus errors.
    pub async fn set_sensors_enabled(
        &mut self,
        accel: bool,
        gyro: bool,
    ) -> Result<(), Error<IO::Error>> {
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
            gyro_en: gyro,
            accel_en: accel,
            ..c
        })
        .await?;
        if !accel {
            self.config.accel = None;
        }
        if !gyro {
            self.config.gyro = None;
        }
        self.fifo_format = self.config.fifo_format();
        Ok(())
    }

    /// Async mirror of [`crate::Qmi8658::set_gyro_snooze`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn set_gyro_snooze(&mut self, snooze: bool) -> Result<(), Error<IO::Error>> {
        self.modify_ctrl7(|c| Ctrl7 {
            gyro_snooze: snooze,
            ..c
        })
        .await
    }

    /// Async mirror of [`crate::Qmi8658::set_ahb_clock_gating`].
    ///
    /// # Errors
    /// Bus errors or [`Error::CommandTimeout`].
    pub async fn set_ahb_clock_gating(&mut self, enabled: bool) -> Result<(), Error<IO::Error>> {
        self.io
            .write_register(registers::CAL1_L, u8::from(!enabled))
            .await?;
        self.send_command(Ctrl9Command::AhbClockGating, CTRL9_TIMEOUT_MS)
            .await
    }

    /// Async mirror of [`crate::Qmi8658::read_register`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn read_register(&mut self, reg: u8) -> Result<u8, Error<IO::Error>> {
        Ok(self.io.read_register(reg).await?)
    }

    /// Async mirror of [`crate::Qmi8658::write_register`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn write_register(&mut self, reg: u8, value: u8) -> Result<(), Error<IO::Error>> {
        Ok(self.io.write_register(reg, value).await?)
    }

    /// Async mirror of [`crate::Qmi8658::read_registers`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn read_registers(
        &mut self,
        reg: u8,
        buf: &mut [u8],
    ) -> Result<(), Error<IO::Error>> {
        Ok(self.io.read_block(reg, buf).await?)
    }

    /// Async mirror of [`crate::Qmi8658::write_registers`].
    ///
    /// # Errors
    /// Bus errors.
    pub async fn write_registers(&mut self, reg: u8, data: &[u8]) -> Result<(), Error<IO::Error>> {
        Ok(self.io.write_block(reg, data).await?)
    }

    async fn send_command(
        &mut self,
        cmd: Ctrl9Command,
        timeout_ms: u32,
    ) -> Result<(), Error<IO::Error>> {
        self.io.write_register(registers::CTRL9, cmd as u8).await?;
        self.wait_cmd_done(true, timeout_ms, cmd).await?;
        self.io
            .write_register(registers::CTRL9, Ctrl9Command::Ack as u8)
            .await?;
        self.wait_cmd_done(false, timeout_ms, cmd).await
    }

    async fn wait_cmd_done(
        &mut self,
        done: bool,
        timeout_ms: u32,
        cmd: Ctrl9Command,
    ) -> Result<(), Error<IO::Error>> {
        let _ = cmd;
        let mut elapsed = 0u32;
        loop {
            let s = StatusInt::from_bits(self.io.read_register(registers::STATUSINT).await?);
            if s.cmd_done == done {
                return Ok(());
            }
            if elapsed >= timeout_ms {
                #[cfg(feature = "log")]
                log::warn!("qmi8658: Ctrl9 command {cmd:?} timed out after {timeout_ms} ms");
                return Err(Error::CommandTimeout);
            }
            self.delay.delay_ms(1).await;
            elapsed += 1;
        }
    }

    async fn wait_statusint_avail(
        &mut self,
        avail: bool,
        timeout_ms: u32,
    ) -> Result<(), Error<IO::Error>> {
        let mut elapsed = 0u32;
        loop {
            let s = StatusInt::from_bits(self.io.read_register(registers::STATUSINT).await?);
            if s.avail == avail {
                return Ok(());
            }
            if elapsed >= timeout_ms {
                return Err(Error::CommandTimeout);
            }
            self.delay.delay_ms(5).await;
            elapsed += 5;
        }
    }

    async fn modify_register(
        &mut self,
        reg: u8,
        f: impl FnOnce(u8) -> u8,
    ) -> Result<(), Error<IO::Error>> {
        let current = self.io.read_register(reg).await?;
        self.io.write_register(reg, f(current)).await?;
        Ok(())
    }

    async fn modify_ctrl7(
        &mut self,
        f: impl FnOnce(Ctrl7) -> Ctrl7,
    ) -> Result<(), Error<IO::Error>> {
        self.modify_register(registers::CTRL7, |b| f(Ctrl7::from_bits(b)).to_bits())
            .await
    }

    async fn modify_ctrl8(
        &mut self,
        f: impl FnOnce(Ctrl8) -> Ctrl8,
    ) -> Result<(), Error<IO::Error>> {
        self.modify_register(registers::CTRL8, |b| f(Ctrl8::from_bits(b)).to_bits())
            .await
    }

    /// Stop both sensors, returning the previous CTRL7 for
    /// [`restore_ctrl7`](Self::restore_ctrl7).
    async fn sensors_off(&mut self) -> Result<Ctrl7, Error<IO::Error>> {
        let saved = Ctrl7::from_bits(self.io.read_register(registers::CTRL7).await?);
        self.io
            .write_register(
                registers::CTRL7,
                Ctrl7 {
                    gyro_en: false,
                    accel_en: false,
                    ..saved
                }
                .to_bits(),
            )
            .await?;
        Ok(saved)
    }

    async fn restore_ctrl7(&mut self, saved: Ctrl7) -> Result<(), Error<IO::Error>> {
        Ok(self
            .io
            .write_register(registers::CTRL7, saved.to_bits())
            .await?)
    }

    async fn configure_engine(
        &mut self,
        call1: [u8; 8],
        call2: [u8; 8],
        cmd: Ctrl9Command,
    ) -> Result<(), Error<IO::Error>> {
        self.io.write_block(registers::CAL1_L, &call1).await?;
        self.send_command(cmd, CTRL9_TIMEOUT_MS).await?;
        self.io.write_block(registers::CAL1_L, &call2).await?;
        self.send_command(cmd, CTRL9_TIMEOUT_MS).await
    }

    async fn read_dvx(&mut self) -> Result<[i16; 3], Error<IO::Error>> {
        let mut buf = [0u8; 6];
        self.io.read_block(registers::DVX_L, &mut buf).await?;
        Ok(axes(&buf))
    }

    async fn restore_ctrl2_3_7(
        &mut self,
        ctrl2: u8,
        ctrl3: u8,
        ctrl7: u8,
    ) -> Result<(), Error<IO::Error>> {
        self.io.write_register(registers::CTRL2, ctrl2).await?;
        self.io.write_register(registers::CTRL3, ctrl3).await?;
        self.io.write_register(registers::CTRL7, ctrl7).await?;
        Ok(())
    }
}
