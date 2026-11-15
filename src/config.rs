//! Typed, `const`-friendly configuration for the QMI8658.
//!
//! All builder methods are `const fn` taking and returning `Self`, so a
//! complete configuration can be declared as a `static`/`const` item.
//! Activity-engine parameters use datasheet-native units; each field
//! documents its LSB weight and the parameter table it comes from.

use crate::data::{FifoFrameFormat, Scales};
use crate::registers::{
    AccelOdr, AccelRange, Ctrl2, Ctrl3, Ctrl5, Ctrl7, FifoMode, FifoSize, GyroRange, LpfMode, Odr,
};

/// Top-level sensor configuration passed to `init` / `apply_config`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config {
    /// Accelerometer setup; `None` keeps the accelerometer off.
    pub accel: Option<AccelConfig>,
    /// Gyroscope setup; `None` keeps the gyroscope off.
    pub gyro: Option<GyroConfig>,
    /// SyncSample mode: reading STATUSINT locks the current sample set
    /// until the last data byte is read. The FIFO cannot be used while
    /// this is enabled.
    pub sync_sample: bool,
}

impl Default for Config {
    /// Both sensors on at 224.2 Hz, accel ±4 g, gyro ±512 dps, filters off.
    fn default() -> Self {
        Self {
            accel: Some(AccelConfig::default()),
            gyro: Some(GyroConfig::default()),
            sync_sample: false,
        }
    }
}

impl Config {
    /// New default configuration (both sensors on).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            accel: Some(AccelConfig::default_const()),
            gyro: Some(GyroConfig::default_const()),
            sync_sample: false,
        }
    }

    /// Set the accelerometer configuration.
    #[must_use]
    pub const fn with_accel(mut self, accel: AccelConfig) -> Self {
        self.accel = Some(accel);
        self
    }

    /// Disable the accelerometer.
    #[must_use]
    pub const fn without_accel(mut self) -> Self {
        self.accel = None;
        self
    }

    /// Set the gyroscope configuration.
    #[must_use]
    pub const fn with_gyro(mut self, gyro: GyroConfig) -> Self {
        self.gyro = Some(gyro);
        self
    }

    /// Disable the gyroscope.
    #[must_use]
    pub const fn without_gyro(mut self) -> Self {
        self.gyro = None;
        self
    }

    /// Enable or disable SyncSample (data-locking) mode.
    #[must_use]
    pub const fn with_sync_sample(mut self, enabled: bool) -> Self {
        self.sync_sample = enabled;
        self
    }
    /// Validate against the device rules the drivers enforce before any
    /// bus write. Returns the static message for `Error::InvalidConfig`.
    pub(crate) const fn validate(&self, current_fifo_mode: FifoMode) -> Result<(), &'static str> {
        if self.accel.is_none() && self.gyro.is_none() {
            return Err("at least one sensor must be enabled");
        }
        if self.gyro.is_some() {
            if let Some(accel) = &self.accel {
                if !accel.odr.is_six_dof() {
                    return Err(
                        "accelerometer standalone/low-power ODR requires the gyroscope disabled",
                    );
                }
            }
        }
        if self.sync_sample && !matches!(current_fifo_mode, FifoMode::Bypass) {
            return Err("SyncSample mode requires the FIFO bypassed; bypass the FIFO first");
        }
        Ok(())
    }

    /// CTRL5 image from the low-pass filter settings of enabled sensors.
    pub(crate) const fn ctrl5(&self) -> Ctrl5 {
        let mut ctrl5 = Ctrl5 {
            gyro_lpf_mode: LpfMode::Pct2_66,
            gyro_lpf_en: false,
            accel_lpf_mode: LpfMode::Pct2_66,
            accel_lpf_en: false,
        };
        if let Some(accel) = &self.accel {
            if let Some(mode) = accel.lpf {
                ctrl5.accel_lpf_mode = mode;
                ctrl5.accel_lpf_en = true;
            }
        }
        if let Some(gyro) = &self.gyro {
            if let Some(mode) = gyro.lpf {
                ctrl5.gyro_lpf_mode = mode;
                ctrl5.gyro_lpf_en = true;
            }
        }
        ctrl5
    }

    /// CTRL7 image enabling exactly the configured sensors.
    pub(crate) const fn ctrl7(&self) -> Ctrl7 {
        Ctrl7 {
            sync_sample: self.sync_sample,
            drdy_dis: false,
            gyro_snooze: false,
            gyro_en: self.gyro.is_some(),
            accel_en: self.accel.is_some(),
        }
    }

    /// Conversion scales for this configuration. Disabled sensors keep
    /// the driver-default range so raw reads remain well defined.
    pub(crate) const fn scales(&self) -> Scales {
        Scales {
            accel: match &self.accel {
                Some(a) => a.range,
                None => AccelRange::G4,
            },
            gyro: match &self.gyro {
                Some(g) => g.range,
                None => GyroRange::Dps512,
            },
        }
    }

    /// FIFO frame layout implied by the enabled sensors.
    pub(crate) const fn fifo_format(&self) -> FifoFrameFormat {
        match (self.accel.is_some(), self.gyro.is_some()) {
            (true, true) => FifoFrameFormat::AccelAndGyro,
            (false, true) => FifoFrameFormat::GyroOnly,
            _ => FifoFrameFormat::AccelOnly,
        }
    }
}

/// Accelerometer range, output data rate, and filter.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AccelConfig {
    /// Full-scale range.
    pub range: AccelRange,
    /// Output data rate. Standalone and low-power rates require the
    /// gyroscope to be disabled.
    pub odr: AccelOdr,
    /// Low-pass filter bandwidth; `None` disables the filter.
    pub lpf: Option<LpfMode>,
}

impl AccelConfig {
    pub(crate) const fn default_const() -> Self {
        Self {
            range: AccelRange::G4,
            odr: AccelOdr::SixDof(Odr::Hz224),
            lpf: None,
        }
    }

    /// New accelerometer configuration with the filter disabled.
    #[must_use]
    pub const fn new(range: AccelRange, odr: AccelOdr) -> Self {
        Self {
            range,
            odr,
            lpf: None,
        }
    }

    /// Enable the low-pass filter with the given bandwidth.
    #[must_use]
    pub const fn with_lpf(mut self, mode: LpfMode) -> Self {
        self.lpf = Some(mode);
        self
    }

    /// Disable the low-pass filter.
    #[must_use]
    pub const fn without_lpf(mut self) -> Self {
        self.lpf = None;
        self
    }
    /// CTRL2 image for this configuration (self-test off).
    pub(crate) const fn ctrl2(&self) -> Ctrl2 {
        Ctrl2 {
            self_test: false,
            range: self.range,
            odr_bits: self.odr.bits(),
        }
    }
}

impl Default for AccelConfig {
    fn default() -> Self {
        Self::default_const()
    }
}

/// Gyroscope range, output data rate, and filter.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroConfig {
    /// Full-scale range.
    pub range: GyroRange,
    /// Output data rate.
    pub odr: Odr,
    /// Low-pass filter bandwidth; `None` disables the filter.
    pub lpf: Option<LpfMode>,
}

impl GyroConfig {
    pub(crate) const fn default_const() -> Self {
        Self {
            range: GyroRange::Dps512,
            odr: Odr::Hz224,
            lpf: None,
        }
    }

    /// New gyroscope configuration with the filter disabled.
    #[must_use]
    pub const fn new(range: GyroRange, odr: Odr) -> Self {
        Self {
            range,
            odr,
            lpf: None,
        }
    }

    /// Enable the low-pass filter with the given bandwidth.
    #[must_use]
    pub const fn with_lpf(mut self, mode: LpfMode) -> Self {
        self.lpf = Some(mode);
        self
    }

    /// Disable the low-pass filter.
    #[must_use]
    pub const fn without_lpf(mut self) -> Self {
        self.lpf = None;
        self
    }
    /// CTRL3 image for this configuration (self-test off).
    pub(crate) const fn ctrl3(&self) -> Ctrl3 {
        Ctrl3 {
            self_test: false,
            range: self.range,
            odr: self.odr,
        }
    }
}

impl Default for GyroConfig {
    fn default() -> Self {
        Self::default_const()
    }
}

/// FIFO operating mode, depth, and watermark.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoConfig {
    /// FIFO operating mode.
    pub mode: FifoMode,
    /// FIFO depth in samples.
    pub size: FifoSize,
    /// Watermark level in samples; must not exceed `size`.
    /// A non-zero value enables the watermark flag/interrupt.
    pub watermark: u8,
}

impl Default for FifoConfig {
    fn default() -> Self {
        Self {
            mode: FifoMode::Stream,
            size: FifoSize::Samples128,
            watermark: 1,
        }
    }
}

impl FifoConfig {
    /// New default FIFO configuration (Stream, 128 samples, watermark 1).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            mode: FifoMode::Stream,
            size: FifoSize::Samples128,
            watermark: 1,
        }
    }

    /// Set the FIFO operating mode.
    #[must_use]
    pub const fn with_mode(mut self, mode: FifoMode) -> Self {
        self.mode = mode;
        self
    }

    /// Set the FIFO depth.
    #[must_use]
    pub const fn with_size(mut self, size: FifoSize) -> Self {
        self.size = size;
        self
    }

    /// Set the watermark level in samples.
    #[must_use]
    pub const fn with_watermark(mut self, watermark: u8) -> Self {
        self.watermark = watermark;
        self
    }
}

/// Interrupt pin enables and signal routing.
///
/// The driver never owns the INT1/INT2 pins; this only controls how the
/// sensor drives them. Note on `ctrl9_handshake_on_int1`: driver-internal
/// Ctrl9 waits always poll STATUSINT regardless of this routing, so
/// commands work in every configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct InterruptConfig {
    /// INT1 pin push-pull enable (CTRL1 INT1_EN; A variant only,
    /// C-variant pins are always push-pull).
    pub int1_enable: bool,
    /// INT2 pin push-pull enable (CTRL1 INT2_EN; A variant only).
    pub int2_enable: bool,
    /// Route the FIFO watermark/full interrupt to INT1 (`false`: INT2).
    pub fifo_interrupt_on_int1: bool,
    /// Route activity (motion/tap/pedometer) interrupts to INT1 (`false`: INT2).
    pub activity_interrupt_on_int1: bool,
    /// Also signal Ctrl9 command completion on INT1 (`false`: STATUSINT only).
    pub ctrl9_handshake_on_int1: bool,
    /// Drive the data-ready pulse on INT2 (Non-SyncSample, FIFO bypassed).
    pub data_ready_enable: bool,
}

impl Default for InterruptConfig {
    /// Datasheet reset routing with both pins driven: FIFO, activity, and
    /// DRDY on INT2; Ctrl9 completion on STATUSINT only.
    fn default() -> Self {
        Self {
            int1_enable: true,
            int2_enable: true,
            fifo_interrupt_on_int1: false,
            activity_interrupt_on_int1: false,
            ctrl9_handshake_on_int1: false,
            data_ready_enable: true,
        }
    }
}

impl InterruptConfig {
    /// New default interrupt configuration.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            int1_enable: true,
            int2_enable: true,
            fifo_interrupt_on_int1: false,
            activity_interrupt_on_int1: false,
            ctrl9_handshake_on_int1: false,
            data_ready_enable: true,
        }
    }
}

/// Axis priority used to pick the tap axis when several axes peak in the
/// same sample (tap parameter table, `Priority[2:0]`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum TapPriority {
    /// X before Y before Z.
    #[default]
    Xyz = 0,
    /// X before Z before Y.
    Xzy = 1,
    /// Y before X before Z.
    Yxz = 2,
    /// Y before Z before X.
    Yzx = 3,
    /// Z before X before Y.
    Zxy = 4,
    /// Z before Y before X.
    Zyx = 5,
}

/// Tap engine parameters (datasheet tap parameter table).
///
/// Rates are in accelerometer samples, so the effective times depend
/// on the accel ODR; an ODR above 200 Hz is recommended for tap
/// detection. Defaults are the examples given in the datasheet table
/// at 500 Hz ODR.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TapConfig {
    /// Axis priority when peaks coincide.
    pub priority: TapPriority,
    /// Maximum duration of a valid peak, in samples (e.g. 20 @ 500 Hz).
    pub peak_window: u8,
    /// Minimum quiet time after the first tap before a second tap is
    /// valid, in samples (e.g. 50 @ 500 Hz).
    pub tap_window: u16,
    /// Maximum time from the first tap's peak for a second tap to count
    /// as a double tap, in samples (e.g. 250 @ 500 Hz).
    pub double_tap_window: u16,
    /// Averaging weight for the acceleration average, in units of
    /// 1/128 (e.g. 8 = 0.0625). Larger weights the newest sample more.
    pub alpha: u8,
    /// Averaging weight for the movement-magnitude average, in units
    /// of 1/128 (e.g. 32 = 0.25).
    pub gamma: u8,
    /// Peak detection threshold on the squared linear acceleration,
    /// in units of 1/1024 g² (e.g. `0x0320` = 0.8 g²).
    pub peak_mag_thr: u16,
    /// Undefined-motion (quiet) threshold, in units of 1/1024 g²
    /// (e.g. `0x0190` = 0.4 g²).
    pub udm_thr: u16,
}

impl Default for TapConfig {
    fn default() -> Self {
        Self {
            priority: TapPriority::Xyz,
            peak_window: 20,
            tap_window: 50,
            double_tap_window: 250,
            alpha: 8,
            gamma: 32,
            peak_mag_thr: 0x0320,
            udm_thr: 0x0190,
        }
    }
}

impl TapConfig {
    /// New tap configuration with the datasheet example values.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            priority: TapPriority::Xyz,
            peak_window: 20,
            tap_window: 50,
            double_tap_window: 250,
            alpha: 8,
            gamma: 32,
            peak_mag_thr: 0x0320,
            udm_thr: 0x0190,
        }
    }

    /// First parameter set image, CAL1_L..=CAL4_H (selector `0x01`).
    pub(crate) const fn encode_call1(&self) -> [u8; 8] {
        [
            self.peak_window,
            self.priority as u8,
            self.tap_window.to_le_bytes()[0],
            self.tap_window.to_le_bytes()[1],
            self.double_tap_window.to_le_bytes()[0],
            self.double_tap_window.to_le_bytes()[1],
            0,
            0x01,
        ]
    }

    /// Second parameter set image, CAL1_L..=CAL4_H (selector `0x02`).
    pub(crate) const fn encode_call2(&self) -> [u8; 8] {
        [
            self.alpha,
            self.gamma,
            self.peak_mag_thr.to_le_bytes()[0],
            self.peak_mag_thr.to_le_bytes()[1],
            self.udm_thr.to_le_bytes()[0],
            self.udm_thr.to_le_bytes()[1],
            0,
            0x02,
        ]
    }
}

/// Which motion engines to enable after configuring their parameters.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionEngines {
    /// Any-motion engine.
    pub any_motion: bool,
    /// No-motion engine.
    pub no_motion: bool,
    /// Significant-motion engine (needs any- and no-motion running).
    pub significant_motion: bool,
}

/// How per-axis motion events combine into one engine event.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AxisLogic {
    /// Any enabled axis reporting an event triggers the engine.
    #[default]
    Or,
    /// All enabled axes must report an event.
    And,
}

/// Any/no/significant-motion parameters (datasheet motion parameter
/// table). Slope thresholds are in units of 1/32 g (0.03125 g) per
/// sample; windows are in accelerometer samples.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MotionConfig {
    /// Any-motion slope thresholds per axis `[x, y, z]`, 1/32 g units.
    pub any_threshold: [u8; 3],
    /// No-motion slope thresholds per axis `[x, y, z]`, 1/32 g units.
    pub no_threshold: [u8; 3],
    /// Consecutive samples above threshold for a valid any-motion event.
    pub any_window: u8,
    /// Consecutive samples below threshold for a valid no-motion event.
    pub no_window: u8,
    /// Idle samples after a first any-motion event before significant
    /// motion can be confirmed by a second one.
    pub sig_wait_window: u16,
    /// Maximum samples from the first any-motion event to confirm
    /// significant motion with a second one.
    pub sig_confirm_window: u16,
    /// Axes participating in any-motion detection `[x, y, z]`.
    pub any_axes: [bool; 3],
    /// Axes participating in no-motion detection `[x, y, z]`.
    pub no_axes: [bool; 3],
    /// Combination logic for any-motion axis events.
    pub any_logic: AxisLogic,
    /// Combination logic for no-motion axis events.
    pub no_logic: AxisLogic,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self {
            any_threshold: [16, 16, 16],
            no_threshold: [4, 4, 4],
            any_window: 5,
            no_window: 50,
            sig_wait_window: 100,
            sig_confirm_window: 500,
            any_axes: [true, true, true],
            no_axes: [true, true, true],
            any_logic: AxisLogic::Or,
            no_logic: AxisLogic::Or,
        }
    }
}

impl MotionConfig {
    /// New motion configuration: any-motion at 0.5 g/sample slope for
    /// 5 samples, no-motion below 0.125 g/sample for 50 samples, all
    /// axes, OR logic.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            any_threshold: [16, 16, 16],
            no_threshold: [4, 4, 4],
            any_window: 5,
            no_window: 50,
            sig_wait_window: 100,
            sig_confirm_window: 500,
            any_axes: [true, true, true],
            no_axes: [true, true, true],
            any_logic: AxisLogic::Or,
            no_logic: AxisLogic::Or,
        }
    }

    /// MOTION_MODE_CTRL byte: bit7 no-logic, bits 6:4 no-axes Z/Y/X,
    /// bit3 any-logic, bits 2:0 any-axes Z/Y/X.
    const fn mode_ctrl(&self) -> u8 {
        ((self.no_logic as u8) << 7)
            | ((self.no_axes[2] as u8) << 6)
            | ((self.no_axes[1] as u8) << 5)
            | ((self.no_axes[0] as u8) << 4)
            | ((self.any_logic as u8) << 3)
            | ((self.any_axes[2] as u8) << 2)
            | ((self.any_axes[1] as u8) << 1)
            | (self.any_axes[0] as u8)
    }

    /// First parameter set image, CAL1_L..=CAL4_H (selector `0x01`).
    pub(crate) const fn encode_call1(&self) -> [u8; 8] {
        [
            self.any_threshold[0],
            self.any_threshold[1],
            self.any_threshold[2],
            self.no_threshold[0],
            self.no_threshold[1],
            self.no_threshold[2],
            self.mode_ctrl(),
            0x01,
        ]
    }

    /// Second parameter set image, CAL1_L..=CAL4_H (selector `0x02`).
    pub(crate) const fn encode_call2(&self) -> [u8; 8] {
        [
            self.any_window,
            self.no_window,
            self.sig_wait_window.to_le_bytes()[0],
            self.sig_wait_window.to_le_bytes()[1],
            self.sig_confirm_window.to_le_bytes()[0],
            self.sig_confirm_window.to_le_bytes()[1],
            0,
            0x02,
        ]
    }
}

/// Pedometer parameters (C variant; datasheet pedometer parameter
/// table). Durations are in accelerometer samples. Defaults are the
/// datasheet worked example at 50 Hz ODR.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PedometerConfig {
    /// Sample batch/window size for the step calculation (samples).
    pub sample_cnt: u16,
    /// Valid peak-to-peak threshold, in units of 1/1024 g
    /// (`0x00CC` ≈ 200 mg).
    pub fix_peak2peak: u16,
    /// Peak-vs-average threshold, in units of 1/1024 g
    /// (`0x0066` ≈ 100 mg).
    pub fix_peak: u16,
    /// Step timeout: counting restarts if no peak occurs within this
    /// many samples.
    pub time_up: u16,
    /// Minimum step duration; peaks inside this quiet time are ignored.
    pub time_low: u8,
    /// Continuous steps required before counting starts (screens out
    /// non-step vibration).
    pub cnt_entry: u8,
    /// Calculation precision tweak; 0 is recommended.
    pub fix_precision: u8,
    /// Step-count register update granularity: registers update (and
    /// the pedometer event fires) every this many valid steps.
    pub sig_count: u8,
}

impl Default for PedometerConfig {
    fn default() -> Self {
        Self {
            sample_cnt: 50,
            fix_peak2peak: 0x00CC,
            fix_peak: 0x0066,
            time_up: 200,
            time_low: 20,
            cnt_entry: 10,
            fix_precision: 0,
            sig_count: 4,
        }
    }
}

impl PedometerConfig {
    /// New pedometer configuration with the datasheet example values
    /// (50 Hz accel ODR).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            sample_cnt: 50,
            fix_peak2peak: 0x00CC,
            fix_peak: 0x0066,
            time_up: 200,
            time_low: 20,
            cnt_entry: 10,
            fix_precision: 0,
            sig_count: 4,
        }
    }

    /// First parameter set image, CAL1_L..=CAL4_H (selector `0x01`).
    pub(crate) const fn encode_call1(&self) -> [u8; 8] {
        [
            self.sample_cnt.to_le_bytes()[0],
            self.sample_cnt.to_le_bytes()[1],
            self.fix_peak2peak.to_le_bytes()[0],
            self.fix_peak2peak.to_le_bytes()[1],
            self.fix_peak.to_le_bytes()[0],
            self.fix_peak.to_le_bytes()[1],
            0,
            0x01,
        ]
    }

    /// Second parameter set image, CAL1_L..=CAL4_H (selector `0x02`).
    pub(crate) const fn encode_call2(&self) -> [u8; 8] {
        [
            self.time_up.to_le_bytes()[0],
            self.time_up.to_le_bytes()[1],
            self.time_low,
            self.cnt_entry,
            self.fix_precision,
            self.sig_count,
            0,
            0x02,
        ]
    }
}

/// Interrupt pin used to signal a Wake-on-Motion event.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WomInterrupt {
    /// Signal on INT1.
    #[default]
    Int1,
    /// Signal on INT2.
    Int2,
}

/// Wake-on-Motion parameters (C variant; datasheet WoM register table).
/// In WoM mode the device produces no sensor data; it only toggles the
/// selected interrupt pin when motion exceeds the threshold.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WomConfig {
    /// Slope threshold for a valid movement, in mg (1 mg/LSB, one byte).
    /// Must be non-zero; zero is the disable encoding.
    pub threshold_mg: u8,
    /// Samples ignored after WoM is enabled (6-bit field), screening
    /// out startup transients.
    pub blanking_samples: u8,
    /// Pin that signals WoM events.
    pub interrupt: WomInterrupt,
    /// Idle level of the WoM interrupt pin; each event toggles it.
    pub initial_level_high: bool,
}

impl Default for WomConfig {
    fn default() -> Self {
        Self {
            threshold_mg: 50,
            blanking_samples: 0,
            interrupt: WomInterrupt::Int1,
            initial_level_high: false,
        }
    }
}

impl WomConfig {
    /// New Wake-on-Motion configuration (50 mg threshold, INT1, idle low).
    #[must_use]
    pub const fn new() -> Self {
        Self {
            threshold_mg: 50,
            blanking_samples: 0,
            interrupt: WomInterrupt::Int1,
            initial_level_high: false,
        }
    }

    /// CAL1 image `[CAL1_L, CAL1_H]`: threshold byte, then interrupt
    /// select in bits 7:6 (bit 7 initial level, bit 6 set for INT2)
    /// and blanking time in bits 5:0.
    pub(crate) const fn encode(&self) -> [u8; 2] {
        let pin_bit = match self.interrupt {
            WomInterrupt::Int1 => 0,
            WomInterrupt::Int2 => 1,
        };
        [
            self.threshold_mg,
            ((self.initial_level_high as u8) << 7)
                | (pin_bit << 6)
                | (self.blanking_samples & 0x3F),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn const_config_builders() {
        const CFG: Config = Config::new()
            .with_accel(
                AccelConfig::new(AccelRange::G2, AccelOdr::SixDof(Odr::Hz112))
                    .with_lpf(LpfMode::Pct5_39),
            )
            .without_gyro()
            .with_sync_sample(true);
        assert_eq!(CFG.accel.map(|a| a.range), Some(AccelRange::G2));
        assert_eq!(CFG.accel.and_then(|a| a.lpf), Some(LpfMode::Pct5_39));
        assert!(CFG.gyro.is_none() && CFG.sync_sample);
        assert_eq!(Config::default(), Config::new());
    }

    #[test]
    fn tap_encoder_layout() {
        let cfg = TapConfig {
            priority: TapPriority::Zyx,
            peak_window: 20,
            tap_window: 50,
            double_tap_window: 250,
            alpha: 8,
            gamma: 32,
            peak_mag_thr: 0x0320,
            udm_thr: 0x0190,
        };
        assert_eq!(cfg.encode_call1(), [20, 5, 50, 0, 250, 0, 0, 0x01]);
        assert_eq!(cfg.encode_call2(), [8, 32, 0x20, 0x03, 0x90, 0x01, 0, 0x02]);
    }

    #[test]
    fn motion_encoder_layout() {
        let cfg = MotionConfig {
            any_threshold: [1, 2, 3],
            no_threshold: [4, 5, 6],
            any_window: 7,
            no_window: 8,
            sig_wait_window: 0x1234,
            sig_confirm_window: 0x5678,
            any_axes: [true, false, true],
            no_axes: [false, true, false],
            any_logic: AxisLogic::And,
            no_logic: AxisLogic::Or,
        };
        // mode_ctrl: no_logic=0, no Z=0,Y=1,X=0, any_logic=1, any Z=1,Y=0,X=1
        assert_eq!(cfg.encode_call1(), [1, 2, 3, 4, 5, 6, 0b0010_1101, 0x01]);
        assert_eq!(cfg.encode_call2(), [7, 8, 0x34, 0x12, 0x78, 0x56, 0, 0x02]);
    }

    #[test]
    fn pedometer_encoder_layout() {
        let cfg = PedometerConfig::default();
        assert_eq!(
            cfg.encode_call1(),
            [0x32, 0x00, 0xCC, 0x00, 0x66, 0x00, 0, 0x01]
        );
        assert_eq!(cfg.encode_call2(), [0xC8, 0x00, 20, 10, 0, 4, 0, 0x02]);
    }

    #[test]
    fn wom_encoder_layout() {
        // Datasheet WoM table: 00=INT1/0, 10=INT1/1, 01=INT2/0, 11=INT2/1.
        let base = WomConfig {
            threshold_mg: 100,
            blanking_samples: 0x7F,
            ..WomConfig::new()
        };
        assert_eq!(base.encode(), [100, 0x3F]);
        let int2_high = WomConfig {
            interrupt: WomInterrupt::Int2,
            initial_level_high: true,
            ..base
        };
        assert_eq!(int2_high.encode(), [100, 0xFF]);
        let int2_low = WomConfig {
            interrupt: WomInterrupt::Int2,
            initial_level_high: false,
            blanking_samples: 0,
            ..base
        };
        assert_eq!(int2_low.encode(), [100, 0x40]);
    }
}
