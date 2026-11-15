//! Register addresses, bit layouts, and field encodings of the QMI8658.
//!
//! Every value in this module comes from the QST QMI8658 datasheets:
//! the A-variant register map (Rev D, Table 20) and, for the pedometer and
//! Wake-on-Motion features that only exist on the C variant, the
//! QMI8658C datasheet (Rev A). Register structs convert losslessly with
//! [`to_bits`](crate::registers::Ctrl1::to_bits) / `from_bits`;
//! decoding never panics —
//! reserved field encodings are masked to the nearest documented variant.

/// Device identifier register, always reads [`WHO_AM_I_VALUE`].
pub const WHO_AM_I: u8 = 0x00;
/// Device revision register (variant dependent, reported verbatim).
pub const REVISION_ID: u8 = 0x01;
/// Serial interface and interrupt pin configuration.
pub const CTRL1: u8 = 0x02;
/// Accelerometer full-scale, output data rate, and self-test.
pub const CTRL2: u8 = 0x03;
/// Gyroscope full-scale, output data rate, and self-test.
pub const CTRL3: u8 = 0x04;
/// Low-pass filter configuration for both sensors.
pub const CTRL5: u8 = 0x06;
/// Sensor enables, SyncSample mode, DRDY routing, gyro snooze.
pub const CTRL7: u8 = 0x08;
/// Motion/tap/pedometer engine enables and Ctrl9 handshake routing.
pub const CTRL8: u8 = 0x09;
/// Host command register (see [`Ctrl9Command`]).
pub const CTRL9: u8 = 0x0A;
/// Calibration/parameter register 1, low byte.
pub const CAL1_L: u8 = 0x0B;
/// Calibration/parameter register 1, high byte.
pub const CAL1_H: u8 = 0x0C;
/// Calibration/parameter register 2, low byte.
pub const CAL2_L: u8 = 0x0D;
/// Calibration/parameter register 2, high byte.
pub const CAL2_H: u8 = 0x0E;
/// Calibration/parameter register 3, low byte.
pub const CAL3_L: u8 = 0x0F;
/// Calibration/parameter register 3, high byte.
pub const CAL3_H: u8 = 0x10;
/// Calibration/parameter register 4, low byte.
pub const CAL4_L: u8 = 0x11;
/// Calibration/parameter register 4, high byte (parameter set selector).
pub const CAL4_H: u8 = 0x12;
/// FIFO watermark threshold, in samples.
pub const FIFO_WTM_TH: u8 = 0x13;
/// FIFO mode, size, and read-mode bit.
pub const FIFO_CTRL: u8 = 0x14;
/// FIFO sample count, low 8 bits (unit: 2-byte words).
pub const FIFO_SMPL_CNT: u8 = 0x15;
/// FIFO status flags and sample count high bits.
pub const FIFO_STATUS: u8 = 0x16;
/// FIFO data output port.
pub const FIFO_DATA: u8 = 0x17;
/// Ctrl9 done bit and SyncSample lock/avail bits.
pub const STATUSINT: u8 = 0x2D;
/// Accelerometer/gyroscope data-available flags.
pub const STATUS0: u8 = 0x2E;
/// Activity engine event flags.
pub const STATUS1: u8 = 0x2F;
/// Sample timestamp, low byte (24-bit, `0x30..=0x32`).
pub const TIMESTAMP_L: u8 = 0x30;
/// Temperature output, low byte (`0x33..=0x34`, signed 8.8 fixed point).
pub const TEMP_L: u8 = 0x33;
/// Acceleration X output, low byte (accel block `0x35..=0x3A`).
pub const AX_L: u8 = 0x35;
/// Angular rate X output, low byte (gyro block `0x3B..=0x40`).
pub const GX_L: u8 = 0x3B;
/// Calibration-on-demand status.
pub const COD_STATUS: u8 = 0x46;
/// General-purpose output block start (firmware version after COPY_USID).
pub const DQW_L: u8 = 0x49;
/// General-purpose output block: self-test deltas / COD gains / USID (`0x51..=0x56`).
pub const DVX_L: u8 = 0x51;
/// Tap event axis, polarity, and count.
pub const TAP_STATUS: u8 = 0x59;
/// Pedometer step count, low byte (24-bit, `0x5A..=0x5C`, C variant).
pub const STEP_CNT_L: u8 = 0x5A;
/// Soft-reset register; write [`SOFT_RESET_VALUE`].
pub const RESET: u8 = 0x60;

/// Expected content of the [`WHO_AM_I`] register.
pub const WHO_AM_I_VALUE: u8 = 0x05;
/// Value written to [`RESET`] to trigger a soft reset (datasheet Table 28).
pub const SOFT_RESET_VALUE: u8 = 0xB0;

/// Accelerometer full-scale range (CTRL2 bits 6:4).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AccelRange {
    /// ±2 g, 16384 LSB/g.
    #[default]
    G2 = 0,
    /// ±4 g, 8192 LSB/g.
    G4 = 1,
    /// ±8 g, 4096 LSB/g.
    G8 = 2,
    /// ±16 g, 2048 LSB/g.
    G16 = 3,
}

impl AccelRange {
    /// Sensitivity in LSB per g (datasheet Table 8).
    #[must_use]
    pub const fn lsb_per_g(self) -> u16 {
        match self {
            Self::G2 => 16384,
            Self::G4 => 8192,
            Self::G8 => 4096,
            Self::G16 => 2048,
        }
    }

    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            1 => Self::G4,
            2 => Self::G8,
            3 => Self::G16,
            _ => Self::G2,
        }
    }
}

/// Gyroscope full-scale range (CTRL3 bits 6:4).
///
/// ±2048 dps exists on the A/B variants only; the C variant tops out at
/// ±1024 dps.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum GyroRange {
    /// ±16 dps, 2048 LSB/dps.
    #[default]
    Dps16 = 0,
    /// ±32 dps, 1024 LSB/dps.
    Dps32 = 1,
    /// ±64 dps, 512 LSB/dps.
    Dps64 = 2,
    /// ±128 dps, 256 LSB/dps.
    Dps128 = 3,
    /// ±256 dps, 128 LSB/dps.
    Dps256 = 4,
    /// ±512 dps, 64 LSB/dps.
    Dps512 = 5,
    /// ±1024 dps, 32 LSB/dps.
    Dps1024 = 6,
    /// ±2048 dps, 16 LSB/dps (A/B variants only).
    Dps2048 = 7,
}

impl GyroRange {
    /// Sensitivity in LSB per dps (datasheet Table 9).
    #[must_use]
    pub const fn lsb_per_dps(self) -> u16 {
        match self {
            Self::Dps16 => 2048,
            Self::Dps32 => 1024,
            Self::Dps64 => 512,
            Self::Dps128 => 256,
            Self::Dps256 => 128,
            Self::Dps512 => 64,
            Self::Dps1024 => 32,
            Self::Dps2048 => 16,
        }
    }

    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0b111 {
            1 => Self::Dps32,
            2 => Self::Dps64,
            3 => Self::Dps128,
            4 => Self::Dps256,
            5 => Self::Dps512,
            6 => Self::Dps1024,
            7 => Self::Dps2048,
            _ => Self::Dps16,
        }
    }
}

/// Output data rate shared by the gyroscope and by the accelerometer in
/// 6DOF mode (CTRL3 bits 3:0, CTRL2 bits 3:0).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Odr {
    /// 7174.4 Hz.
    #[default]
    Hz7174 = 0,
    /// 3587.2 Hz.
    Hz3587 = 1,
    /// 1793.6 Hz.
    Hz1794 = 2,
    /// 896.8 Hz.
    Hz897 = 3,
    /// 448.4 Hz.
    Hz448 = 4,
    /// 224.2 Hz.
    Hz224 = 5,
    /// 112.1 Hz.
    Hz112 = 6,
    /// 56.05 Hz.
    Hz56 = 7,
    /// 28.025 Hz.
    Hz28 = 8,
}

impl Odr {
    /// Exact output data rate in Hz (datasheet Table 23).
    #[must_use]
    pub const fn hz(self) -> f32 {
        match self {
            Self::Hz7174 => 7174.4,
            Self::Hz3587 => 3587.2,
            Self::Hz1794 => 1793.6,
            Self::Hz897 => 896.8,
            Self::Hz448 => 448.4,
            Self::Hz224 => 224.2,
            Self::Hz112 => 112.1,
            Self::Hz56 => 56.05,
            Self::Hz28 => 28.025,
        }
    }

    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0b1111 {
            1 => Self::Hz3587,
            2 => Self::Hz1794,
            3 => Self::Hz897,
            4 => Self::Hz448,
            5 => Self::Hz224,
            6 => Self::Hz112,
            7 => Self::Hz56,
            // Encodings 9..=15 are undocumented; mask to the slowest rate.
            8..=u8::MAX => Self::Hz28,
            _ => Self::Hz7174,
        }
    }
}

/// Accelerometer-only output data rates (CTRL2 bits 3:0, gyroscope off).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum AccelStandaloneOdr {
    /// 1000 Hz.
    Hz1000 = 3,
    /// 500 Hz.
    Hz500 = 4,
    /// 250 Hz.
    Hz250 = 5,
    /// 125 Hz.
    Hz125 = 6,
    /// 62.5 Hz.
    Hz62 = 7,
    /// 31.25 Hz.
    Hz31 = 8,
}

impl AccelStandaloneOdr {
    /// Output data rate in Hz.
    #[must_use]
    pub const fn hz(self) -> f32 {
        match self {
            Self::Hz1000 => 1000.0,
            Self::Hz500 => 500.0,
            Self::Hz250 => 250.0,
            Self::Hz125 => 125.0,
            Self::Hz62 => 62.5,
            Self::Hz31 => 31.25,
        }
    }
}

/// Accelerometer low-power output data rates (CTRL2 bits 3:0 = `0b1100..=0b1111`).
///
/// Low-power mode is only available while the gyroscope is disabled.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum LowPowerOdr {
    /// 128 Hz.
    Hz128 = 12,
    /// 21 Hz.
    Hz21 = 13,
    /// 11 Hz.
    Hz11 = 14,
    /// 3 Hz.
    Hz3 = 15,
}

impl LowPowerOdr {
    /// Output data rate in Hz.
    #[must_use]
    pub const fn hz(self) -> f32 {
        match self {
            Self::Hz128 => 128.0,
            Self::Hz21 => 21.0,
            Self::Hz11 => 11.0,
            Self::Hz3 => 3.0,
        }
    }
}

/// Accelerometer output data rate: 6DOF, standalone, or low-power encoding.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AccelOdr {
    /// Rate shared with the gyroscope's natural-frequency-derived ODRs.
    SixDof(Odr),
    /// Accelerometer-only rate (gyroscope disabled).
    Standalone(AccelStandaloneOdr),
    /// Low-power rate (gyroscope disabled).
    LowPower(LowPowerOdr),
}

impl Default for AccelOdr {
    fn default() -> Self {
        Self::SixDof(Odr::Hz224)
    }
}

impl AccelOdr {
    /// CTRL2 low-nibble encoding of this rate.
    #[must_use]
    pub const fn bits(self) -> u8 {
        match self {
            Self::SixDof(odr) => odr as u8,
            Self::Standalone(odr) => odr as u8,
            Self::LowPower(odr) => odr as u8,
        }
    }

    /// Output data rate in Hz.
    #[must_use]
    pub const fn hz(self) -> f32 {
        match self {
            Self::SixDof(odr) => odr.hz(),
            Self::Standalone(odr) => odr.hz(),
            Self::LowPower(odr) => odr.hz(),
        }
    }

    /// Whether this rate can be combined with an enabled gyroscope.
    #[must_use]
    pub const fn is_six_dof(self) -> bool {
        matches!(self, Self::SixDof(_))
    }
}

/// Low-pass filter bandwidth, as a percentage of the ODR (CTRL5).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum LpfMode {
    /// 2.66 % of ODR.
    #[default]
    Pct2_66 = 0,
    /// 3.63 % of ODR.
    Pct3_63 = 1,
    /// 5.39 % of ODR.
    Pct5_39 = 2,
    /// 13.37 % of ODR.
    Pct13_37 = 3,
}

impl LpfMode {
    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            1 => Self::Pct3_63,
            2 => Self::Pct5_39,
            3 => Self::Pct13_37,
            _ => Self::Pct2_66,
        }
    }
}

/// FIFO operating mode (FIFO_CTRL bits 1:0). Encoding `0b11` is reserved
/// and never emitted; it decodes as [`FifoMode::Stream`].
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FifoMode {
    /// FIFO disabled; data registers and DRDY are active.
    #[default]
    Bypass = 0,
    /// Filling stops when the FIFO is full.
    Fifo = 1,
    /// Oldest samples are discarded when the FIFO is full.
    Stream = 2,
}

impl FifoMode {
    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            1 => Self::Fifo,
            2 | 3 => Self::Stream,
            _ => Self::Bypass,
        }
    }
}

/// FIFO depth in samples (FIFO_CTRL bits 3:2). One sample is 6 bytes with
/// one sensor enabled, 12 bytes with both.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum FifoSize {
    /// 16 samples.
    #[default]
    Samples16 = 0,
    /// 32 samples.
    Samples32 = 1,
    /// 64 samples.
    Samples64 = 2,
    /// 128 samples.
    Samples128 = 3,
}

impl FifoSize {
    /// Depth in samples.
    #[must_use]
    pub const fn samples(self) -> u16 {
        match self {
            Self::Samples16 => 16,
            Self::Samples32 => 32,
            Self::Samples64 => 64,
            Self::Samples128 => 128,
        }
    }

    pub(crate) const fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            1 => Self::Samples32,
            2 => Self::Samples64,
            3 => Self::Samples128,
            _ => Self::Samples16,
        }
    }
}

/// Predefined host commands written to CTRL9 (datasheet Table 29; the
/// pedometer and Wake-on-Motion commands are C-variant features).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Ctrl9Command {
    /// Acknowledge a completed command, ending the Ctrl9 handshake.
    Ack = 0x00,
    /// Reset the FIFO (data, count, and flags).
    ResetFifo = 0x04,
    /// Enter FIFO read mode so FIFO_DATA can be drained.
    RequestFifo = 0x05,
    /// Write Wake-on-Motion settings from CAL1 (C variant).
    WriteWomSetting = 0x08,
    /// Apply host accelerometer delta offsets from CAL1..CAL3 (signed Q4.12, unit g).
    AccelHostDeltaOffset = 0x09,
    /// Apply host gyroscope delta offsets from CAL1..CAL3 (signed Q11.5, unit dps).
    GyroHostDeltaOffset = 0x0A,
    /// Configure the tap engine from the CAL parameter set.
    ConfigureTap = 0x0C,
    /// Configure the pedometer engine from the CAL parameter set (C variant).
    ConfigurePedometer = 0x0D,
    /// Configure the any/no/significant-motion engines from the CAL parameter set.
    ConfigureMotion = 0x0E,
    /// Reset the pedometer step count (C variant).
    ResetPedometer = 0x0F,
    /// Copy firmware version and unique serial ID into the UI registers.
    CopyUsid = 0x10,
    /// Configure the internal IO pull-up resistors from CAL1_L.
    SetRpu = 0x11,
    /// Switch internal AHB clock gating (CAL1_L: 1 disables gating).
    AhbClockGating = 0x12,
    /// Run gyroscope calibration-on-demand (takes ≈1.5 s).
    OnDemandCalibration = 0xA2,
    /// Restore gyroscope gains previously saved from a calibration-on-demand run.
    ApplyGyroGains = 0xAA,
}

const fn bit(byte: u8, shift: u8) -> bool {
    (byte >> shift) & 1 == 1
}

const fn put(condition: bool, shift: u8) -> u8 {
    (condition as u8) << shift
}

/// CTRL1 register image: serial interface and interrupt pin setup.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl1 {
    /// Bit 7 (SIM): `true` selects 3-wire SPI, `false` 4-wire SPI.
    pub sim_3wire: bool,
    /// Bit 6 (ADDR_AI): register address auto-increments during bursts.
    pub addr_ai: bool,
    /// Bit 5 (BE): `true` makes multi-byte output data big-endian.
    pub big_endian: bool,
    /// Bit 4 (INT2_EN): INT2 pin is push-pull (`false`: high-Z). A variant only.
    pub int2_en: bool,
    /// Bit 3 (INT1_EN): INT1 pin is push-pull (`false`: high-Z). A variant only.
    pub int1_en: bool,
    /// Bit 2 (FIFO_INT_SEL): FIFO interrupt on INT1 (`false`: INT2).
    pub fifo_int_on_int1: bool,
    /// Bit 0: disables the internal high-speed oscillator (power-down).
    pub sensor_disable: bool,
}

impl Default for Ctrl1 {
    /// Datasheet power-on default (`0b0010_0000`): big-endian output.
    fn default() -> Self {
        Self {
            sim_3wire: false,
            addr_ai: false,
            big_endian: true,
            int2_en: false,
            int1_en: false,
            fifo_int_on_int1: false,
            sensor_disable: false,
        }
    }
}

impl Ctrl1 {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.sim_3wire, 7)
            | put(self.addr_ai, 6)
            | put(self.big_endian, 5)
            | put(self.int2_en, 4)
            | put(self.int1_en, 3)
            | put(self.fifo_int_on_int1, 2)
            | put(self.sensor_disable, 0)
    }

    /// Decode from the register byte. Reserved bit 1 is dropped.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            sim_3wire: bit(b, 7),
            addr_ai: bit(b, 6),
            big_endian: bit(b, 5),
            int2_en: bit(b, 4),
            int1_en: bit(b, 3),
            fifo_int_on_int1: bit(b, 2),
            sensor_disable: bit(b, 0),
        }
    }
}

/// CTRL2 register image: accelerometer range, ODR, and self-test.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl2 {
    /// Bit 7 (aST): accelerometer self-test enable.
    pub self_test: bool,
    /// Bits 6:4 (aFS): full-scale range.
    pub range: AccelRange,
    /// Bits 3:0 (aODR): raw ODR encoding; see [`AccelOdr::bits`].
    pub odr_bits: u8,
}

impl Ctrl2 {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.self_test, 7) | ((self.range as u8) << 4) | (self.odr_bits & 0b1111)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            self_test: bit(b, 7),
            range: AccelRange::from_bits(b >> 4),
            odr_bits: b & 0b1111,
        }
    }
}

/// CTRL3 register image: gyroscope range, ODR, and self-test.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl3 {
    /// Bit 7 (gST): gyroscope self-test enable.
    pub self_test: bool,
    /// Bits 6:4 (gFS): full-scale range.
    pub range: GyroRange,
    /// Bits 3:0 (gODR): output data rate.
    pub odr: Odr,
}

impl Ctrl3 {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.self_test, 7) | ((self.range as u8) << 4) | (self.odr as u8)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            self_test: bit(b, 7),
            range: GyroRange::from_bits(b >> 4),
            odr: Odr::from_bits(b),
        }
    }
}

/// CTRL5 register image: low-pass filters.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl5 {
    /// Bits 6:5 (gLPF_MODE): gyroscope filter bandwidth.
    pub gyro_lpf_mode: LpfMode,
    /// Bit 4 (gLPF_EN): gyroscope filter enable.
    pub gyro_lpf_en: bool,
    /// Bits 2:1 (aLPF_MODE): accelerometer filter bandwidth.
    pub accel_lpf_mode: LpfMode,
    /// Bit 0 (aLPF_EN): accelerometer filter enable.
    pub accel_lpf_en: bool,
}

impl Ctrl5 {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        ((self.gyro_lpf_mode as u8) << 5)
            | put(self.gyro_lpf_en, 4)
            | ((self.accel_lpf_mode as u8) << 1)
            | put(self.accel_lpf_en, 0)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            gyro_lpf_mode: LpfMode::from_bits(b >> 5),
            gyro_lpf_en: bit(b, 4),
            accel_lpf_mode: LpfMode::from_bits(b >> 1),
            accel_lpf_en: bit(b, 0),
        }
    }
}

/// CTRL7 register image: sensor enables and data-read mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl7 {
    /// Bit 7 (SyncSample): enables the data-locking mechanism; FIFO unavailable.
    pub sync_sample: bool,
    /// Bit 5 (DRDY_DIS): blocks the data-ready signal from INT2.
    pub drdy_dis: bool,
    /// Bit 4 (gSN): gyroscope snooze (drive only, no sense data).
    pub gyro_snooze: bool,
    /// Bit 1 (gEN): gyroscope enable.
    pub gyro_en: bool,
    /// Bit 0 (aEN): accelerometer enable.
    pub accel_en: bool,
}

impl Ctrl7 {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.sync_sample, 7)
            | put(self.drdy_dis, 5)
            | put(self.gyro_snooze, 4)
            | put(self.gyro_en, 1)
            | put(self.accel_en, 0)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            sync_sample: bit(b, 7),
            drdy_dis: bit(b, 5),
            gyro_snooze: bit(b, 4),
            gyro_en: bit(b, 1),
            accel_en: bit(b, 0),
        }
    }
}

/// CTRL8 register image: activity engines and Ctrl9 handshake routing.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Ctrl8 {
    /// Bit 7 (CTRL9_Handshake_Type): `true` signals Ctrl9 completion on
    /// STATUSINT bit 7 only, `false` also raises INT1.
    pub ctrl9_handshake_statusint: bool,
    /// Bit 6 (ACTIVITY_INT_SEL): activity interrupts on INT1 (`false`: INT2).
    pub activity_on_int1: bool,
    /// Bit 4: pedometer engine enable (C variant; reserved on A).
    pub pedometer_en: bool,
    /// Bit 3: significant-motion engine enable.
    pub sig_motion_en: bool,
    /// Bit 2: no-motion engine enable.
    pub no_motion_en: bool,
    /// Bit 1: any-motion engine enable.
    pub any_motion_en: bool,
    /// Bit 0: tap engine enable.
    pub tap_en: bool,
}

impl Ctrl8 {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.ctrl9_handshake_statusint, 7)
            | put(self.activity_on_int1, 6)
            | put(self.pedometer_en, 4)
            | put(self.sig_motion_en, 3)
            | put(self.no_motion_en, 2)
            | put(self.any_motion_en, 1)
            | put(self.tap_en, 0)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            ctrl9_handshake_statusint: bit(b, 7),
            activity_on_int1: bit(b, 6),
            pedometer_en: bit(b, 4),
            sig_motion_en: bit(b, 3),
            no_motion_en: bit(b, 2),
            any_motion_en: bit(b, 1),
            tap_en: bit(b, 0),
        }
    }
}

/// FIFO_CTRL register image.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoCtrl {
    /// Bit 7 (FIFO_RD_MODE): FIFO is being drained; filling is paused.
    /// Set by the `RequestFifo` command, must be cleared by the host.
    pub read_mode: bool,
    /// Bits 3:2 (FIFO_SIZE): FIFO depth.
    pub size: FifoSize,
    /// Bits 1:0 (FIFO_MODE): FIFO operating mode.
    pub mode: FifoMode,
}

impl FifoCtrl {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.read_mode, 7) | ((self.size as u8) << 2) | (self.mode as u8)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            read_mode: bit(b, 7),
            size: FifoSize::from_bits(b >> 2),
            mode: FifoMode::from_bits(b),
        }
    }
}

/// FIFO_STATUS register image.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoStatusReg {
    /// Bit 7: FIFO filled level equals the configured size.
    pub full: bool,
    /// Bit 6: filled level reached the watermark threshold.
    pub watermark: bool,
    /// Bit 5: samples were dropped because the FIFO overflowed.
    pub overflow: bool,
    /// Bit 4: FIFO holds at least one sample.
    pub not_empty: bool,
    /// Bits 1:0: high bits of the 10-bit sample count (unit: 2-byte words).
    pub sample_count_msb: u8,
}

impl FifoStatusReg {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.full, 7)
            | put(self.watermark, 6)
            | put(self.overflow, 5)
            | put(self.not_empty, 4)
            | (self.sample_count_msb & 0b11)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            full: bit(b, 7),
            watermark: bit(b, 6),
            overflow: bit(b, 5),
            not_empty: bit(b, 4),
            sample_count_msb: b & 0b11,
        }
    }
}

/// STATUSINT register image.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct StatusInt {
    /// Bit 7: a Ctrl9 command has completed and awaits acknowledgement.
    pub cmd_done: bool,
    /// Bit 1: in SyncSample mode, the current sample set is locked.
    /// Otherwise mirrors the INT1 pin level.
    pub locked: bool,
    /// Bit 0: in SyncSample mode, new sensor data is available.
    /// Otherwise mirrors the INT2 pin level.
    pub avail: bool,
}

impl StatusInt {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.cmd_done, 7) | put(self.locked, 1) | put(self.avail, 0)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            cmd_done: bit(b, 7),
            locked: bit(b, 1),
            avail: bit(b, 0),
        }
    }
}

/// STATUS0 register image: new-data flags.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status0 {
    /// Bit 1 (gDA): new gyroscope data since the last read.
    pub gyro_ready: bool,
    /// Bit 0 (aDA): new accelerometer data since the last read.
    pub accel_ready: bool,
}

impl Status0 {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.gyro_ready, 1) | put(self.accel_ready, 0)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            gyro_ready: bit(b, 1),
            accel_ready: bit(b, 0),
        }
    }
}

/// STATUS1 register image: activity event flags.
///
/// Bits 4 (pedometer) and 2 (Wake-on-Motion) are C-variant features and
/// read as reserved on the A variant.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status1 {
    /// Bit 7: significant motion detected.
    pub significant_motion: bool,
    /// Bit 6: no motion detected.
    pub no_motion: bool,
    /// Bit 5: any motion detected.
    pub any_motion: bool,
    /// Bit 4: pedometer event (C variant).
    pub pedometer: bool,
    /// Bit 2: Wake-on-Motion event (C variant).
    pub wake_on_motion: bool,
    /// Bit 1: tap detected.
    pub tap: bool,
}

impl Status1 {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.significant_motion, 7)
            | put(self.no_motion, 6)
            | put(self.any_motion, 5)
            | put(self.pedometer, 4)
            | put(self.wake_on_motion, 2)
            | put(self.tap, 1)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            significant_motion: bit(b, 7),
            no_motion: bit(b, 6),
            any_motion: bit(b, 5),
            pedometer: bit(b, 4),
            wake_on_motion: bit(b, 2),
            tap: bit(b, 1),
        }
    }
}

/// COD_STATUS register image: calibration-on-demand result detail.
/// The value is only valid after an `OnDemandCalibration` command;
/// an all-zero byte means success.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CodStatus {
    /// Bit 7: gyro X sensitivity below the low limit.
    pub x_limit_low_fail: bool,
    /// Bit 6: gyro X sensitivity above the high limit.
    pub x_limit_high_fail: bool,
    /// Bit 5: gyro Y sensitivity below the low limit.
    pub y_limit_low_fail: bool,
    /// Bit 4: gyro Y sensitivity above the high limit.
    pub y_limit_high_fail: bool,
    /// Bit 3: significant vibration during calibration.
    pub accel_check_fail: bool,
    /// Bit 2: gyroscope startup failed when calibration was called.
    pub startup_failed: bool,
    /// Bit 1: calibration was called while the gyroscope was enabled.
    pub gyro_was_enabled: bool,
    /// Bit 0: calibration failed; no correction applied.
    pub cod_failed: bool,
}

impl CodStatus {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.x_limit_low_fail, 7)
            | put(self.x_limit_high_fail, 6)
            | put(self.y_limit_low_fail, 5)
            | put(self.y_limit_high_fail, 4)
            | put(self.accel_check_fail, 3)
            | put(self.startup_failed, 2)
            | put(self.gyro_was_enabled, 1)
            | put(self.cod_failed, 0)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            x_limit_low_fail: bit(b, 7),
            x_limit_high_fail: bit(b, 6),
            y_limit_low_fail: bit(b, 5),
            y_limit_high_fail: bit(b, 4),
            accel_check_fail: bit(b, 3),
            startup_failed: bit(b, 2),
            gyro_was_enabled: bit(b, 1),
            cod_failed: bit(b, 0),
        }
    }

    /// Whether the calibration succeeded (register reads `0x00`).
    #[must_use]
    pub const fn is_success(self) -> bool {
        self.to_bits() == 0
    }
}

/// TAP_STATUS register image (raw field encodings; see
/// [`crate::data::TapEvent`] for the decoded form).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TapStatusReg {
    /// Bit 7: tap approached along the negative direction of the tap axis.
    pub negative_polarity: bool,
    /// Bits 5:4: tap axis encoding (0 none, 1 X, 2 Y, 3 Z).
    pub axis_bits: u8,
    /// Bits 1:0: tap count encoding (0 none, 1 single, 2 double, 3 n/a).
    pub count_bits: u8,
}

impl TapStatusReg {
    /// Encode to the register byte.
    #[must_use]
    pub const fn to_bits(self) -> u8 {
        put(self.negative_polarity, 7) | ((self.axis_bits & 0b11) << 4) | (self.count_bits & 0b11)
    }

    /// Decode from the register byte.
    #[must_use]
    pub const fn from_bits(b: u8) -> Self {
        Self {
            negative_polarity: bit(b, 7),
            axis_bits: (b >> 4) & 0b11,
            count_bits: b & 0b11,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ctrl1_round_trip() {
        for b in [0x00u8, 0x20, 0x58, 0xFD, 0x84] {
            assert_eq!(Ctrl1::from_bits(b).to_bits(), b & !0b10);
        }
        assert_eq!(Ctrl1::default().to_bits(), 0x20);
        let c = Ctrl1::from_bits(0x58);
        assert!(c.addr_ai && c.int1_en && c.int2_en && !c.big_endian);
    }

    #[test]
    fn ctrl2_round_trip() {
        for b in 0..=255u8 {
            // Bits 6:4 encodings 4..=7 are undocumented and mask to 0..=3.
            let expected = (b & 0x8F) | ((b >> 4) & 0b11) << 4;
            assert_eq!(Ctrl2::from_bits(b).to_bits(), expected);
        }
        let c = Ctrl2 {
            self_test: true,
            range: AccelRange::G8,
            odr_bits: AccelOdr::SixDof(Odr::Hz224).bits(),
        };
        assert_eq!(c.to_bits(), 0xA5);
    }

    #[test]
    fn ctrl3_round_trip() {
        for b in [0x00u8, 0x85, 0x78, 0x03] {
            assert_eq!(Ctrl3::from_bits(b).to_bits(), b);
        }
        let c = Ctrl3 {
            self_test: false,
            range: GyroRange::Dps512,
            odr: Odr::Hz224,
        };
        assert_eq!(c.to_bits(), 0x55);
    }

    #[test]
    fn ctrl5_round_trip() {
        for b in 0..=255u8 {
            assert_eq!(Ctrl5::from_bits(b).to_bits(), b & 0x77);
        }
        let c = Ctrl5 {
            gyro_lpf_mode: LpfMode::Pct5_39,
            gyro_lpf_en: true,
            accel_lpf_mode: LpfMode::Pct3_63,
            accel_lpf_en: true,
        };
        assert_eq!(c.to_bits(), 0x53);
    }

    #[test]
    fn ctrl7_round_trip() {
        for b in [0x00u8, 0x83, 0x81, 0x82, 0x33, 0xB3] {
            assert_eq!(Ctrl7::from_bits(b).to_bits(), b);
        }
    }

    #[test]
    fn ctrl8_round_trip() {
        for b in 0..=255u8 {
            assert_eq!(Ctrl8::from_bits(b).to_bits(), b & 0xDF);
        }
    }

    #[test]
    fn fifo_ctrl_round_trip() {
        for b in [0x00u8, 0x8E, 0x0D, 0x86] {
            assert_eq!(FifoCtrl::from_bits(b).to_bits(), b);
        }
        // Reserved mode 0b11 decodes as Stream and re-encodes as 0b10.
        assert_eq!(FifoCtrl::from_bits(0x03).mode, FifoMode::Stream);
        assert_eq!(FifoCtrl::from_bits(0x03).to_bits(), 0x02);
    }

    #[test]
    fn status_round_trips() {
        for b in [0x00u8, 0x83, 0x81] {
            assert_eq!(StatusInt::from_bits(b).to_bits(), b);
        }
        for b in 0..=3u8 {
            assert_eq!(Status0::from_bits(b).to_bits(), b);
        }
        for b in [0x00u8, 0xE0, 0x12, 0xF6] {
            assert_eq!(Status1::from_bits(b).to_bits(), b);
        }
        for b in [0x00u8, 0xF3, 0x83] {
            assert_eq!(FifoStatusReg::from_bits(b).to_bits(), b);
        }
        for b in 0..=255u8 {
            assert_eq!(CodStatus::from_bits(b).to_bits(), b);
        }
        assert!(CodStatus::from_bits(0).is_success());
        assert!(!CodStatus::from_bits(1).is_success());
        for b in [0x00u8, 0xA2, 0x31, 0xB3] {
            assert_eq!(TapStatusReg::from_bits(b).to_bits(), b);
        }
    }

    #[test]
    fn scales_match_datasheet() {
        assert_eq!(AccelRange::G2.lsb_per_g(), 16384);
        assert_eq!(AccelRange::G4.lsb_per_g(), 8192);
        assert_eq!(AccelRange::G8.lsb_per_g(), 4096);
        assert_eq!(AccelRange::G16.lsb_per_g(), 2048);
        assert_eq!(GyroRange::Dps16.lsb_per_dps(), 2048);
        assert_eq!(GyroRange::Dps512.lsb_per_dps(), 64);
        assert_eq!(GyroRange::Dps2048.lsb_per_dps(), 16);
    }

    #[test]
    fn odr_encodings() {
        assert_eq!(Odr::Hz224 as u8, 5);
        assert_eq!(AccelOdr::Standalone(AccelStandaloneOdr::Hz1000).bits(), 3);
        assert_eq!(AccelOdr::LowPower(LowPowerOdr::Hz3).bits(), 15);
        assert!((Odr::Hz28.hz() - 28.025).abs() < 0.001);
        assert!((Odr::Hz7174.hz() - 7174.4).abs() < 0.1);
        assert!(AccelOdr::SixDof(Odr::Hz112).is_six_dof());
        assert!(!AccelOdr::LowPower(LowPowerOdr::Hz128).is_six_dof());
    }

    #[test]
    fn ctrl9_command_values() {
        assert_eq!(Ctrl9Command::Ack as u8, 0x00);
        assert_eq!(Ctrl9Command::ResetFifo as u8, 0x04);
        assert_eq!(Ctrl9Command::RequestFifo as u8, 0x05);
        assert_eq!(Ctrl9Command::WriteWomSetting as u8, 0x08);
        assert_eq!(Ctrl9Command::ConfigureTap as u8, 0x0C);
        assert_eq!(Ctrl9Command::ConfigurePedometer as u8, 0x0D);
        assert_eq!(Ctrl9Command::ConfigureMotion as u8, 0x0E);
        assert_eq!(Ctrl9Command::AhbClockGating as u8, 0x12);
        assert_eq!(Ctrl9Command::OnDemandCalibration as u8, 0xA2);
        assert_eq!(Ctrl9Command::ApplyGyroGains as u8, 0xAA);
    }
}
