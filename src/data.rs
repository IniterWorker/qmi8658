//! Measurement output types and pure raw-to-physical conversions.
//!
//! Temperature uses the datasheet formula `T = TEMP_H + TEMP_L / 256` °C,
//! i.e. the TEMP register pair is a signed 8.8 fixed-point value with a
//! resolution of 1/256 °C.

use crate::registers::{AccelRange, GyroRange, Status1, TapStatusReg};

/// One raw sample set exactly as read from the output registers.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RawSample {
    /// Raw accelerometer counts `[x, y, z]`.
    pub accel: [i16; 3],
    /// Raw gyroscope counts `[x, y, z]`.
    pub gyro: [i16; 3],
    /// Raw temperature: signed 8.8 fixed point, 1/256 °C per LSB.
    pub temperature_raw: i16,
    /// 24-bit sample timestamp counter (wraps at `0xFF_FFFF`).
    pub timestamp: u32,
}

const fn i16_at(b: &[u8; 17], offset: usize) -> i16 {
    i16::from_le_bytes([b[offset], b[offset + 1]])
}

impl RawSample {
    /// Decode the contiguous 17-byte block starting at TIMESTAMP_L
    /// (`0x30..=0x40`): timestamp (3), temperature (2), accel (6), gyro (6).
    pub(crate) const fn from_block(b: &[u8; 17]) -> Self {
        Self {
            timestamp: (b[0] as u32) | ((b[1] as u32) << 8) | ((b[2] as u32) << 16),
            temperature_raw: i16_at(b, 3),
            accel: [i16_at(b, 5), i16_at(b, 7), i16_at(b, 9)],
            gyro: [i16_at(b, 11), i16_at(b, 13), i16_at(b, 15)],
        }
    }
}

/// One sample set converted to physical units.
#[derive(Clone, Copy, PartialEq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Sample {
    /// Acceleration in g, `[x, y, z]`.
    pub accel_g: [f32; 3],
    /// Angular rate in degrees per second, `[x, y, z]`.
    pub gyro_dps: [f32; 3],
    /// Die temperature in °C.
    pub temperature_c: f32,
    /// 24-bit sample timestamp counter (wraps at `0xFF_FFFF`).
    pub timestamp: u32,
}

/// Full-scale ranges needed to convert [`RawSample`] counts.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Scales {
    /// Accelerometer range the raw counts were captured with.
    pub accel: AccelRange,
    /// Gyroscope range the raw counts were captured with.
    pub gyro: GyroRange,
}

impl Default for Scales {
    fn default() -> Self {
        Self {
            accel: AccelRange::G4,
            gyro: GyroRange::Dps512,
        }
    }
}

impl Scales {
    /// Convert one raw sample set to physical units.
    #[must_use]
    pub fn convert(&self, raw: &RawSample) -> Sample {
        let a = f32::from(self.accel.lsb_per_g());
        let g = f32::from(self.gyro.lsb_per_dps());
        Sample {
            accel_g: [
                f32::from(raw.accel[0]) / a,
                f32::from(raw.accel[1]) / a,
                f32::from(raw.accel[2]) / a,
            ],
            gyro_dps: [
                f32::from(raw.gyro[0]) / g,
                f32::from(raw.gyro[1]) / g,
                f32::from(raw.gyro[2]) / g,
            ],
            temperature_c: temperature_c(raw.temperature_raw),
            timestamp: raw.timestamp,
        }
    }

    /// Convert raw accelerometer counts to milli-g using integer math.
    #[must_use]
    pub fn accel_mg(&self, raw: [i16; 3]) -> [i32; 3] {
        let lsb = i32::from(self.accel.lsb_per_g());
        [
            i32::from(raw[0]) * 1000 / lsb,
            i32::from(raw[1]) * 1000 / lsb,
            i32::from(raw[2]) * 1000 / lsb,
        ]
    }

    /// Convert raw gyroscope counts to milli-dps using integer math.
    #[must_use]
    pub fn gyro_mdps(&self, raw: [i16; 3]) -> [i32; 3] {
        let lsb = i32::from(self.gyro.lsb_per_dps());
        [
            i32::from(raw[0]) * 1000 / lsb,
            i32::from(raw[1]) * 1000 / lsb,
            i32::from(raw[2]) * 1000 / lsb,
        ]
    }
}

/// Convert the raw signed 8.8 temperature value to °C.
#[must_use]
pub fn temperature_c(raw: i16) -> f32 {
    f32::from(raw) / 256.0
}

/// Convert the raw signed 8.8 temperature value to milli-°C (integer math).
#[must_use]
pub fn temperature_mdegc(raw: i16) -> i32 {
    i32::from(raw) * 1000 / 256
}

/// New-data flags from STATUS0.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DataStatus {
    /// New accelerometer data since the last read.
    pub accel_ready: bool,
    /// New gyroscope data since the last read.
    pub gyro_ready: bool,
}

/// Activity engine event flags from STATUS1.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ActivityStatus {
    /// Significant-motion event.
    pub significant_motion: bool,
    /// No-motion event.
    pub no_motion: bool,
    /// Any-motion event.
    pub any_motion: bool,
    /// Pedometer event (C variant).
    pub pedometer_event: bool,
    /// Wake-on-Motion event (C variant).
    pub wake_on_motion: bool,
    /// Tap event; detail in the driver's `tap_event` method.
    pub tap: bool,
}

impl ActivityStatus {
    pub(crate) const fn from_status1(s: Status1) -> Self {
        Self {
            significant_motion: s.significant_motion,
            no_motion: s.no_motion,
            any_motion: s.any_motion,
            pedometer_event: s.pedometer,
            wake_on_motion: s.wake_on_motion,
            tap: s.tap,
        }
    }

    /// Whether any event flag is set.
    #[must_use]
    pub const fn any(&self) -> bool {
        self.significant_motion
            || self.no_motion
            || self.any_motion
            || self.pedometer_event
            || self.wake_on_motion
            || self.tap
    }
}

/// A device axis.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Axis {
    /// X axis.
    X,
    /// Y axis.
    Y,
    /// Z axis.
    Z,
}

/// Whether a tap event was a single or a double tap.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TapCount {
    /// Single tap.
    Single,
    /// Double tap.
    Double,
}

/// A decoded tap event from TAP_STATUS.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TapEvent {
    /// Single or double tap.
    pub count: TapCount,
    /// Axis on which the first valid peak occurred.
    pub axis: Axis,
    /// `true` if the tap approached along the negative axis direction.
    pub negative_polarity: bool,
}

impl TapEvent {
    pub(crate) const fn from_reg(reg: TapStatusReg) -> Option<Self> {
        let count = match reg.count_bits {
            1 => TapCount::Single,
            2 => TapCount::Double,
            _ => return None,
        };
        let axis = match reg.axis_bits {
            1 => Axis::X,
            2 => Axis::Y,
            3 => Axis::Z,
            _ => return None,
        };
        Some(Self {
            count,
            axis,
            negative_polarity: reg.negative_polarity,
        })
    }
}

/// FIFO fill state, combining FIFO_STATUS and FIFO_SMPL_CNT.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoStatus {
    /// Filled level equals the configured FIFO size.
    pub full: bool,
    /// Filled level reached the watermark threshold.
    pub watermark_reached: bool,
    /// Samples were dropped because the FIFO overflowed.
    pub overflow: bool,
    /// FIFO holds at least one sample.
    pub not_empty: bool,
    /// Bytes available to drain: `2 * 10-bit sample count in words`.
    pub bytes_available: u16,
}

/// Which sensors each FIFO frame contains (datasheet FIFO data pattern:
/// accelerometer axes first, then gyroscope axes).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FifoFrameFormat {
    /// 6 bytes of accelerometer data per frame.
    AccelOnly,
    /// 6 bytes of gyroscope data per frame.
    GyroOnly,
    /// 6 bytes of accelerometer data followed by 6 bytes of gyroscope data.
    AccelAndGyro,
}

impl FifoFrameFormat {
    /// Bytes per frame.
    #[must_use]
    pub const fn frame_len(self) -> usize {
        match self {
            Self::AccelOnly | Self::GyroOnly => 6,
            Self::AccelAndGyro => 12,
        }
    }
}

/// One decoded FIFO frame of raw counts.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FifoFrame {
    /// Raw accelerometer counts, present unless the format is gyro-only.
    pub accel: Option<[i16; 3]>,
    /// Raw gyroscope counts, present unless the format is accel-only.
    pub gyro: Option<[i16; 3]>,
}

const fn axes_at(b: &[u8], offset: usize) -> [i16; 3] {
    [
        i16::from_le_bytes([b[offset], b[offset + 1]]),
        i16::from_le_bytes([b[offset + 2], b[offset + 3]]),
        i16::from_le_bytes([b[offset + 4], b[offset + 5]]),
    ]
}

/// Iterator decoding a drained FIFO byte buffer into [`FifoFrame`]s.
/// A trailing partial frame is ignored.
#[derive(Clone, Debug)]
pub struct FifoFrames<'a> {
    bytes: &'a [u8],
    format: FifoFrameFormat,
    offset: usize,
}

impl<'a> FifoFrames<'a> {
    /// Create an iterator over `bytes` decoded with `format`.
    #[must_use]
    pub const fn new(bytes: &'a [u8], format: FifoFrameFormat) -> Self {
        Self {
            bytes,
            format,
            offset: 0,
        }
    }
}

impl Iterator for FifoFrames<'_> {
    type Item = FifoFrame;

    fn next(&mut self) -> Option<Self::Item> {
        let len = self.format.frame_len();
        let frame = self.bytes.get(self.offset..self.offset + len)?;
        self.offset += len;
        Some(match self.format {
            FifoFrameFormat::AccelOnly => FifoFrame {
                accel: Some(axes_at(frame, 0)),
                gyro: None,
            },
            FifoFrameFormat::GyroOnly => FifoFrame {
                accel: None,
                gyro: Some(axes_at(frame, 0)),
            },
            FifoFrameFormat::AccelAndGyro => FifoFrame {
                accel: Some(axes_at(frame, 0)),
                gyro: Some(axes_at(frame, 6)),
            },
        })
    }
}

/// Result of an accelerometer or gyroscope self-test.
#[derive(Clone, Copy, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SelfTestReport {
    /// Per-axis response deltas: milli-g for the accelerometer
    /// (pass threshold 200 mg) or dps for the gyroscope (threshold 300 dps),
    /// matching the self-test method that produced the report.
    pub delta: [f32; 3],
    /// `true` when every axis exceeds the datasheet pass threshold.
    pub passed: bool,
}

/// Gyroscope gain parameters produced by calibration-on-demand.
///
/// Three opaque 16-bit values, one per axis, as little-endian pairs.
/// Save them to non-volatile storage and restore them after a reset
/// instead of re-running the calibration.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GyroGains(pub [u8; 6]);

/// Identity registers that can be read at any time.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DeviceId {
    /// WHO_AM_I content; `0x05` on genuine QMI8658 silicon.
    pub who_am_i: u8,
    /// REVISION_ID content, reported verbatim (variant dependent,
    /// e.g. `0x7C` or `0x68` in the vendor datasheets).
    pub revision: u8,
}

/// Firmware version and unique serial ID, copied into the UI registers
/// by the CopyUsid command.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UniqueId {
    /// Internal firmware version bytes.
    pub firmware: [u8; 3],
    /// Per-device unique serial ID bytes.
    pub usid: [u8; 6],
}

/// Convert an accelerometer host delta offset in g to the signed Q4.12
/// format used by the `AccelHostDeltaOffset` command.
/// Returns `None` for values outside `-8 <= g < 8` or NaN.
///
/// Scaling uses integer arithmetic on milli-g to stay exact and
/// float-formatting free: `raw = round(g * 4096)`.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub fn accel_offset_to_q4_12(g: f32) -> Option<i16> {
    if !(-8.0..8.0).contains(&g) {
        return None;
    }
    // Round to nearest, ties away from zero, in plain float arithmetic.
    let scaled = g * 4096.0;
    let rounded = if scaled >= 0.0 {
        scaled + 0.5
    } else {
        scaled - 0.5
    };
    // `as` on floats truncates toward zero and saturates; range checked above.
    Some(rounded as i32 as i16)
}

/// Convert a gyroscope host delta offset in dps to the signed Q11.5
/// format used by the `GyroHostDeltaOffset` command.
/// Returns `None` for values outside `-1024 <= dps < 1024` or NaN.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
pub fn gyro_offset_to_q11_5(dps: f32) -> Option<i16> {
    if !(-1024.0..1024.0).contains(&dps) {
        return None;
    }
    let scaled = dps * 32.0;
    let rounded = if scaled >= 0.0 {
        scaled + 0.5
    } else {
        scaled - 0.5
    };
    Some(rounded as i32 as i16)
}

/// Convert an accelerometer self-test delta (signed U5.11, 0.5 mg/LSB)
/// to milli-g.
#[must_use]
pub fn selftest_accel_delta_mg(raw: i16) -> f32 {
    f32::from(raw) * 0.5
}

/// Convert a gyroscope self-test delta (signed U12.4, 62.5 mdps/LSB)
/// to dps.
#[must_use]
pub fn selftest_gyro_delta_dps(raw: i16) -> f32 {
    f32::from(raw) * 0.0625
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn block_decode() {
        let mut b = [0u8; 17];
        b[0..3].copy_from_slice(&[0x56, 0x34, 0x12]);
        b[3..5].copy_from_slice(&0x1980i16.to_le_bytes()); // 25.5 °C
        b[9..11].copy_from_slice(&8192i16.to_le_bytes()); // AZ = 1 g at ±4 g
        b[11..13].copy_from_slice(&(-64i16).to_le_bytes()); // GX = -1 dps at ±512
        let raw = RawSample::from_block(&b);
        assert_eq!(raw.timestamp, 0x12_3456);
        assert_eq!(raw.accel, [0, 0, 8192]);
        assert_eq!(raw.gyro, [-64, 0, 0]);
        let sample = Scales::default().convert(&raw);
        assert!((sample.accel_g[2] - 1.0).abs() < 1e-6);
        assert!((sample.gyro_dps[0] + 1.0).abs() < 1e-6);
        assert!((sample.temperature_c - 25.5).abs() < 1e-6);
    }

    #[test]
    fn scale_extremes() {
        let scales = Scales {
            accel: AccelRange::G16,
            gyro: GyroRange::Dps2048,
        };
        let raw = RawSample {
            accel: [-32768, 16384, 0],
            gyro: [-32768, 16384, 0],
            temperature_raw: 0,
            timestamp: 0,
        };
        let s = scales.convert(&raw);
        assert!((s.accel_g[0] + 16.0).abs() < 1e-6);
        assert!((s.accel_g[1] - 8.0).abs() < 1e-6);
        assert!((s.gyro_dps[0] + 2048.0).abs() < 1e-3);
        assert_eq!(scales.accel_mg(raw.accel), [-16000, 8000, 0]);
        assert_eq!(scales.gyro_mdps(raw.gyro), [-2_048_000, 1_024_000, 0]);
    }

    #[test]
    fn temperature_forms() {
        assert_eq!(temperature_mdegc(0x1980), 25500);
        assert_eq!(temperature_mdegc(-256), -1000);
        assert!((temperature_c(0x1980) - 25.5).abs() < 1e-6);
    }

    #[test]
    fn offset_fixed_point() {
        assert_eq!(accel_offset_to_q4_12(1.0), Some(4096));
        assert_eq!(accel_offset_to_q4_12(-0.5), Some(-2048));
        assert_eq!(accel_offset_to_q4_12(8.0), None);
        assert_eq!(accel_offset_to_q4_12(-8.0), Some(-32768));
        assert_eq!(accel_offset_to_q4_12(f32::NAN), None);
        assert!(accel_offset_to_q4_12(7.999).is_some());
        assert_eq!(gyro_offset_to_q11_5(100.0), Some(3200));
        assert_eq!(gyro_offset_to_q11_5(-1024.0), Some(-32768));
        assert_eq!(gyro_offset_to_q11_5(1024.0), None);
        assert_eq!(gyro_offset_to_q11_5(f32::INFINITY), None);
    }

    #[test]
    fn selftest_units() {
        assert!((selftest_accel_delta_mg(400) - 200.0).abs() < 1e-6);
        assert!((selftest_gyro_delta_dps(4800) - 300.0).abs() < 1e-6);
    }

    #[test]
    fn fifo_frames_both_sensors() {
        let mut buf = [0u8; 25]; // two full frames + trailing byte
        buf[0..2].copy_from_slice(&100i16.to_le_bytes());
        buf[6..8].copy_from_slice(&(-200i16).to_le_bytes());
        buf[12..14].copy_from_slice(&300i16.to_le_bytes());
        let frames: Vec<_> = FifoFrames::new(&buf, FifoFrameFormat::AccelAndGyro).collect();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].accel, Some([100, 0, 0]));
        assert_eq!(frames[0].gyro, Some([-200, 0, 0]));
        assert_eq!(frames[1].accel, Some([300, 0, 0]));
    }

    #[test]
    fn fifo_frames_single_sensor() {
        let mut buf = [0u8; 12];
        buf[4..6].copy_from_slice(&8192i16.to_le_bytes());
        let frames: Vec<_> = FifoFrames::new(&buf, FifoFrameFormat::AccelOnly).collect();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].accel, Some([0, 0, 8192]));
        assert_eq!(frames[0].gyro, None);
        let gyro_frames: Vec<_> = FifoFrames::new(&buf, FifoFrameFormat::GyroOnly).collect();
        assert_eq!(gyro_frames[0].accel, None);
        assert_eq!(gyro_frames[0].gyro, Some([0, 0, 8192]));
    }

    #[test]
    fn tap_decode() {
        let ev = TapEvent::from_reg(TapStatusReg::from_bits(0xB2)).unwrap();
        assert_eq!(ev.count, TapCount::Double);
        assert_eq!(ev.axis, Axis::Z);
        assert!(ev.negative_polarity);
        assert!(TapEvent::from_reg(TapStatusReg::from_bits(0x00)).is_none());
        // Count encoding 3 is "n/a", axis 0 is "no tap".
        assert!(TapEvent::from_reg(TapStatusReg::from_bits(0x13)).is_none());
        assert!(TapEvent::from_reg(TapStatusReg::from_bits(0x01)).is_none());
    }
}
