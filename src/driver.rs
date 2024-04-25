#![allow(clippy::missing_const_for_fn)]
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::{I2c, SevenBitAddress};

use crate::command::constants::{
    AX_HIGH, AX_LOW, AY_HIGH, AZ_HIGH, AZ_LOW, COD_STATUS, CTRL1, CTRL2, CTRL3, CTRL5, CTRL7,
    CTRL8, CTRL9, FIFO_CTRL, FIFO_DATA, FIFO_SMPL_CNT, FIFO_STATUS, FIFO_WTM_TH, GX_HIGH, GX_LOW,
    GY_HIGH, GZ_HIGH, GZ_LOW, STATUS0, STATUS1, STATUSINT, STEP_CNT_HIGH, STEP_CNT_LOW,
    STEP_CNT_MID, TAP_STATUS, TEMP_HIGH, TEMP_LOW, TIMESTAMP_HIGH, TIMESTAMP_LOW, TIMESTAMP_MID,
    WHO_AM_I,
};
use crate::command::register::acceleration::{
    AccelerationFullRegister, AccelerationOutput, AngularFullRegister, AngularRateOutput,
};
use crate::command::register::cod_status::CODStatusRegister;
use crate::command::register::ctrl1::Ctrl1Register;
use crate::command::register::ctrl2::Ctrl2Register;
use crate::command::register::ctrl3::Ctrl3Register;
use crate::command::register::ctrl5::Ctrl5Register;
use crate::command::register::ctrl7::Ctrl7Register;
use crate::command::register::ctrl8::Ctrl8Register;
use crate::command::register::ctrl9::Ctrl9Register;
use crate::command::register::fifo_ctrl::FIFOControlRegister;
use crate::command::register::fifo_data::FIFODataRegister;
use crate::command::register::fifo_sample_count::FIFOSampleCountRegister;
use crate::command::register::fifo_status::FIFOStatusRegister;
use crate::command::register::fifo_wtm_th::FIFOWTMRegister;
use crate::command::register::ship_info::{DeviceID, RevisionID};
use crate::command::register::status_0::OutputDataStatusRegister;
use crate::command::register::status_1::MiscellaneousStatusRegister;
use crate::command::register::status_int::SensorDataAvailableAndLockRegister;
use crate::command::register::tap_status::TapStatusRegister;
use crate::command::register::temp::Temperature;
use crate::command::register::timestamp::SampleTimeStamp;

#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum DeviceAddress {
    /// SA0 external pullup or floationg (intenal pullup)
    Primary = 0x6A,
    /// SA0 external pulldown
    Secondary = 0x6B,
}

impl From<DeviceAddress> for u8 {
    fn from(value: DeviceAddress) -> Self {
        match value {
            DeviceAddress::Primary => 0x6A,
            DeviceAddress::Secondary => 0x6B,
        }
    }
}

impl Default for DeviceAddress {
    fn default() -> Self {
        Self::Primary
    }
}

/// Qmi8658 Driver
pub struct Qmi8658<I, D> {
    /// I2C Interface
    pub(crate) interface: I,
    /// I2C Address
    pub(crate) addr: DeviceAddress,
    /// Delay
    pub(crate) _delay: D,
}

// #[derive(Clone, Debug)]
// pub enum Error {
//     /// I2C Error,
//     I2CError(embedded_hal::i2c::Error)
// }

impl<I, D> Qmi8658<I, D>
where
    I: I2c<SevenBitAddress>,
    D: DelayNs,
{
    /// Create a new Qmi8658 driver with default I2C address.
    pub fn new(interface: I, delay: D) -> Self {
        Self {
            interface,
            addr: DeviceAddress::Primary,
            _delay: delay,
        }
    }

    /// Create new Qmi8658 driver with secondary I2C address.
    pub fn new_secondary_address(interface: I, delay: D) -> Self {
        Self {
            interface,
            addr: DeviceAddress::Secondary,
            _delay: delay,
        }
    }

    /// Give back the I2C interface
    pub fn release(self) -> I {
        self.interface
    }

    /// Get the device id.
    ///
    /// This function retrieves the device ID from the connected device.
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_device_id(&mut self) -> Result<DeviceID, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[WHO_AM_I], &mut buffer)?;
        let value = match buffer[0] {
            0x05 => DeviceID::QST,
            _ => DeviceID::Unknown,
        };

        Ok(value)
    }

    /// Get the device revision id
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_device_revision_id(&mut self) -> Result<RevisionID, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[WHO_AM_I], &mut buffer)?;

        Ok(buffer[0])
    }

    /// Get Serial Interface and Sensor Enable.
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_crtl1(&mut self) -> Result<Ctrl1Register, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[CTRL1], &mut buffer)?;

        Ok(Ctrl1Register(buffer[0]))
    }

    /// Accelerometer Settings
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_ctrl2(&mut self) -> Result<Ctrl2Register, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[CTRL2], &mut buffer)?;

        Ok(Ctrl2Register(buffer[0]))
    }

    /// Gyroscope Settings
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_ctrl3(&mut self) -> Result<Ctrl3Register, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[CTRL3], &mut buffer)?;

        Ok(Ctrl3Register(buffer[0]))
    }

    /// Sensor Data Processing Settings
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_ctrl5(&mut self) -> Result<Ctrl5Register, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[CTRL5], &mut buffer)?;

        Ok(Ctrl5Register(buffer[0]))
    }

    /// Enable Sensors and Configure Data Reads
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_ctrl7(&mut self) -> Result<Ctrl7Register, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[CTRL7], &mut buffer)?;

        Ok(Ctrl7Register(buffer[0]))
    }

    /// Motion Detection Control
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_ctrl8(&mut self) -> Result<Ctrl8Register, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[CTRL8], &mut buffer)?;

        Ok(Ctrl8Register(buffer[0]))
    }

    /// Host Commands. Register Address: 10 (0x0A)
    /// Referred to: CTRL 9 Functionality (Executing Pre-defined Commands)
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    ///
    pub fn get_ctrl9(&mut self) -> Result<Ctrl9Register, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[CTRL9], &mut buffer)?;

        Ok(match buffer[0] {
            0x04 => Ctrl9Register::CtrlCmdRstFifo,
            0x05 => Ctrl9Register::CtrlCmdReqFifo,
            0x08 => Ctrl9Register::CtrlCmdWriteWomSetting,
            0x09 => Ctrl9Register::CtrlCmdAccelHostDeltaOffset,
            0x0A => Ctrl9Register::CtrlCmdGyroHostDeltaOffset,
            0x0C => Ctrl9Register::CtrlCmdConfigureTap,
            0x0D => Ctrl9Register::CtrlCmdConfigurePedometer,
            0x0E => Ctrl9Register::CtrlCmdConfigureMotion,
            0x0F => Ctrl9Register::CtrlCmdResetPedometer,
            0x10 => Ctrl9Register::CtrlCmdCopyUsid,
            0x11 => Ctrl9Register::CtrlCmdSetRpu,
            0x12 => Ctrl9Register::CtrlCmdAhbClockGating,
            0xA2 => Ctrl9Register::CtrlCmdOnDemandCalibration,
            0xAA => Ctrl9Register::CtrlCmdApplyGyroGains,
            // TODO: improve this error handling
            _ => Ctrl9Register::CtrlCmdAck,
        })
    }

    /// FIFO Watermark Register Address
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_fifo_wtm(&mut self) -> Result<FIFOWTMRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[FIFO_WTM_TH], &mut buffer)?;

        Ok(buffer[0])
    }

    /// FIFO Control Register Address
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_fifo_ctrl(&mut self) -> Result<FIFOControlRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[FIFO_CTRL], &mut buffer)?;

        Ok(FIFOControlRegister(buffer[0]))
    }

    /// FIFO Sample Count Register Address
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_fifo_sample_cnt(&mut self) -> Result<FIFOSampleCountRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[FIFO_SMPL_CNT], &mut buffer)?;

        Ok(FIFOSampleCountRegister(buffer[0]))
    }

    /// FIFO Status
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_fifo_status(&mut self) -> Result<FIFOStatusRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[FIFO_STATUS], &mut buffer)?;

        Ok(FIFOStatusRegister(buffer[0]))
    }

    /// FIFO Data
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_fifo_data(&mut self) -> Result<FIFODataRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[FIFO_DATA], &mut buffer)?;

        Ok(FIFODataRegister(buffer[0]))
    }

    /// Sensor Data Available and Lock Register Address
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_sensor_data_available_and_lock(
        &mut self,
    ) -> Result<SensorDataAvailableAndLockRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[STATUSINT], &mut buffer)?;

        Ok(SensorDataAvailableAndLockRegister(buffer[0]))
    }

    /// Output Data Status Register
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_output_data_status(&mut self) -> Result<OutputDataStatusRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[STATUS0], &mut buffer)?;

        Ok(OutputDataStatusRegister(buffer[0]))
    }

    /// Miscellaneous Status
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_miscellaneous_status(&mut self) -> Result<MiscellaneousStatusRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[STATUS1], &mut buffer)?;

        Ok(MiscellaneousStatusRegister(buffer[0]))
    }

    /// 3 Bytes Sample Time Stamp – Output Count.
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_sample_time_stamp_output_count(&mut self) -> Result<SampleTimeStamp, I::Error> {
        let mut buffer = [0u8; 1];
        let mut timestamp: SampleTimeStamp = 0;
        self.interface
            .write_read(self.addr.into(), &[TIMESTAMP_HIGH], &mut buffer)?;
        timestamp |= SampleTimeStamp::from(buffer[0]) << 16;
        self.interface
            .write_read(self.addr.into(), &[TIMESTAMP_MID], &mut buffer)?;
        timestamp |= SampleTimeStamp::from(buffer[0]) << 8;
        self.interface
            .write_read(self.addr.into(), &[TIMESTAMP_LOW], &mut buffer)?;
        timestamp |= SampleTimeStamp::from(buffer[0]);

        Ok(timestamp)
    }

    /// Temp Sensor Output
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_temperature(&mut self) -> Result<Temperature, I::Error> {
        let mut buffer = [0u8; 1];
        let mut temp: Temperature = 0;
        self.interface
            .write_read(self.addr.into(), &[TEMP_HIGH], &mut buffer)?;
        temp |= Temperature::from(buffer[0]) << 8;
        self.interface
            .write_read(self.addr.into(), &[TEMP_LOW], &mut buffer)?;
        temp |= Temperature::from(buffer[0]);

        Ok(temp)
    }

    /// Get Acceleration Output Helper
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    fn get_acceleration_helper(
        &mut self,
        high_addr: u8,
        low_addr: u8,
    ) -> Result<AccelerationFullRegister, I::Error> {
        let mut buffer = [0u8; 1];
        let mut tmp: AccelerationFullRegister = 0;
        self.interface
            .write_read(self.addr.into(), &[high_addr], &mut buffer)?;
        tmp |= AccelerationFullRegister::from(buffer[0]) << 8;
        self.interface
            .write_read(self.addr.into(), &[low_addr], &mut buffer)?;
        tmp |= AccelerationFullRegister::from(buffer[0]);
        Ok(tmp)
    }

    /// Get Acceleration Output
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_acceleration(&mut self) -> Result<AccelerationOutput, I::Error> {
        Ok(AccelerationOutput {
            x: self.get_acceleration_helper(AX_HIGH, AX_LOW)?,
            y: self.get_acceleration_helper(AY_HIGH, AX_LOW)?,
            z: self.get_acceleration_helper(AZ_HIGH, AZ_LOW)?,
        })
    }

    /// Get Angular Rate Output Helper
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    fn get_angular_rate_helper(
        &mut self,
        high_addr: u8,
        low_addr: u8,
    ) -> Result<AngularFullRegister, I::Error> {
        let mut buffer = [0u8; 1];
        let mut tmp: AngularFullRegister = 0;
        self.interface
            .write_read(self.addr.into(), &[high_addr], &mut buffer)?;
        tmp |= AngularFullRegister::from(buffer[0]) << 8;
        self.interface
            .write_read(self.addr.into(), &[low_addr], &mut buffer)?;
        tmp |= AngularFullRegister::from(buffer[0]);
        Ok(tmp)
    }

    /// Get Angular Rate Output
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_angular_rate(&mut self) -> Result<AngularRateOutput, I::Error> {
        Ok(AngularRateOutput {
            x: self.get_angular_rate_helper(GX_HIGH, GX_LOW)?,
            y: self.get_angular_rate_helper(GY_HIGH, GX_LOW)?,
            z: self.get_angular_rate_helper(GZ_HIGH, GZ_LOW)?,
        })
    }

    /// Get Calibration-On-Demand (COD) Status Register
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_cod_status(&mut self) -> Result<CODStatusRegister, I::Error> {
        let mut buffer = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[COD_STATUS], &mut buffer)?;

        Ok(CODStatusRegister(buffer[0]))
    }

    /// Get Activity Detection Output Registers
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_tap_status(&mut self) -> Result<TapStatusRegister, I::Error> {
        let mut buffer: [u8; 1] = [0u8; 1];
        self.interface
            .write_read(self.addr.into(), &[TAP_STATUS], &mut buffer)?;

        Ok(TapStatusRegister(buffer[0]))
    }

    /// 24-bit Step Count detected by Pedometer engine
    ///
    /// # Errors
    ///
    /// This function can return an error if there is an issue during the communication process.
    ///
    /// Possible errors include:
    /// - Communication error: This can occur if there is a problem communicating with the device over the interface.
    pub fn get_step_cnt(&mut self) -> Result<SampleTimeStamp, I::Error> {
        let mut buffer = [0u8; 1];
        let mut timestamp: SampleTimeStamp = 0;
        self.interface
            .write_read(self.addr.into(), &[STEP_CNT_HIGH], &mut buffer)?;
        timestamp |= SampleTimeStamp::from(buffer[0]) << 16;
        self.interface
            .write_read(self.addr.into(), &[STEP_CNT_MID], &mut buffer)?;
        timestamp |= SampleTimeStamp::from(buffer[0]) << 8;
        self.interface
            .write_read(self.addr.into(), &[STEP_CNT_LOW], &mut buffer)?;
        timestamp |= SampleTimeStamp::from(buffer[0]);

        Ok(timestamp)
    }
}
