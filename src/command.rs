#![allow(clippy::doc_markdown)]
#![allow(dead_code)]

pub mod constants {
    /// Device Identifier Register (R)
    /// Default:  00000101
    pub const WHO_AM_I: u8 = 0x00;
    /// Device Revision ID Register (R)
    /// Default:  01101000
    pub const REVISION_ID: u8 = 0x01;
    /// SPI Interface and Sensor Enable Register (W)
    /// Default:  00100000
    pub const CTRL1: u8 = 0x02;
    /// Accelerometer: Output Data Rate, Full Scale, Self-Test Register (W)
    /// Default:  00000000
    pub const CTRL2: u8 = 0x03;
    /// Gyroscope: Output Data Rate, Full Scale, Self-Test Register (W)
    /// Default:  00000000
    pub const CTRL3: u8 = 0x04;
    /// Reserved Register (W)
    /// Default:  00000000
    pub const RESERVED_U1: u8 = 0x05;
    /// Low pass filter setting Register (W)
    /// Default:  00000000
    pub const CTRL5: u8 = 0x06;
    /// Reserved Register (W)
    /// Default:  00000000
    pub const RESERVED_U2: u8 = 0x07;
    /// Enable Sensors Register (W)
    /// Default:  00000000
    pub const CTRL7: u8 = 0x08;
    /// Motion Detection Control Register (W)
    /// Default:  00000000
    pub const CTRL8: u8 = 0x09;
    /// Host Commands Host Controlled Calibration Registers (See CTRL9, Usage is Optional) Register (W)
    /// Default:  00000000
    pub const CTRL9: u8 = 0x0A;
    /// Calibration Low Register Register (RW)
    /// Default:  00000000
    pub const CAL1_LOW: u8 = 0x0B;
    /// Calibration High Register Register (RW)
    /// Default: 00000000
    pub const CAL1_HIGH: u8 = 0x0C;
    /// Calibration Low Register Register (RW)
    /// Default:  00000000
    pub const CAL2_LOW: u8 = 0x0D;
    /// Calibration High Register Register (RW)
    /// Default: 00000000
    pub const CAL2_HIGH: u8 = 0x0E;
    /// Calibration Low Register Register (RW)
    /// Default:  00000000
    pub const CAL3_LOW: u8 = 0x0F;
    /// Calibration High Register Register (RW)
    /// Default: 00000000
    pub const CAL3_HIGH: u8 = 0x10;
    /// Calibration Low Register Register (RW)
    /// Default:  00000000
    pub const CAL4_LOW: u8 = 0x11;
    /// Calibration High Register Register (RW)
    /// Default: 00000000
    pub const CAL4_HIGH: u8 = 0x12;
    /// FIFO watermark level, in ODRs Register (W)
    /// Default:  00000000
    pub const FIFO_WTM_TH: u8 = 0x13;
    /// FIFO Setup Register (W)
    /// Default:  00000000
    pub const FIFO_CTRL: u8 = 0x14;
    /// FIFO sample count LSBs Register (R)
    /// Default:  00000000
    pub const FIFO_SMPL_CNT: u8 = 0x15;
    /// FIFO Status Register (R)
    /// Default:  00000000
    pub const FIFO_STATUS: u8 = 0x16;
    /// FIFO Data Status Registers Register (R)
    /// Default:  00000000
    pub const FIFO_DATA: u8 = 0x17;
    /// Sensor Data Availability with the Locking mechanism, `CmdDone` (CTRL9 protocol bit). Register (R)
    /// Default:  00000000
    pub const STATUSINT: u8 = 0x2D;
    /// Output Data Over Run and Data Availability. Register (R)
    /// Default:  00000000
    pub const STATUS0: u8 = 0x2E;
    /// Miscellaneous Status: Any Motion, No Motion, Significant Motion, Pedometer, Tap. Timestamp Register Register (R)
    /// Default:  00000000
    pub const STATUS1: u8 = 0x2F;
    /// Sample Time Stamp Register (R)
    /// Default:  00000000
    pub const TIMESTAMP_LOW: u8 = 0x30;
    /// Register (R)
    /// Default:  00000000
    pub const TIMESTAMP_MID: u8 = 0x31;
    ///  Register (R)
    /// Default:  00000000
    pub const TIMESTAMP_HIGH: u8 = 0x32;
    /// Temperature Output Data Register (R)
    /// Default:  00000000
    pub const TEMP_LOW: u8 = 0x33;
    /// `TEMP_LOW` – Low 8 bits. `TEMP_HIGH` – upper 8 bits Register (R)
    /// Default:  00000000
    pub const TEMP_HIGH: u8 = 0x34;
    /// X-axis Acceleration Low Register (R)
    /// Default:  00000000
    pub const AX_LOW: u8 = 0x35;
    /// X-axis Acceleration High Register (R)
    /// Default:  00000000
    pub const AX_HIGH: u8 = 0x36;
    /// Y-axis Acceleration Low Register (R)
    /// Default:  00000000
    pub const AY_LOW: u8 = 0x37;
    /// Y-axis Acceleration High Register (R)
    /// Default:  00000000
    pub const AY_HIGH: u8 = 0x38;
    /// Z-axis Acceleration Low Register (R)
    /// Default:  00000000
    pub const AZ_LOW: u8 = 0x39;
    /// Z-axis Acceleration High Register (R)
    /// Default:  00000000
    pub const AZ_HIGH: u8 = 0x3A;
    /// X-axis Angular Rate Low Register (R)
    /// Default:  00000000
    pub const GX_LOW: u8 = 0x3B;
    /// X-axis Angular Rate High Register (R)
    /// Default:  00000000
    pub const GX_HIGH: u8 = 0x3C;
    /// Y-axis Angular Rate Low Register (R)
    /// Default:  00000000
    pub const GY_LOW: u8 = 0x3D;
    /// Z-axis Angular Rate High Register (R)
    /// Default:  00000000
    pub const GY_HIGH: u8 = 0x3E;
    /// Z-axis Angular Rate Low Register (R)
    /// Default:  00000000
    pub const GZ_LOW: u8 = 0x3F;
    /// Z-axis Angular Rate High Register (R)
    /// Default:  00000000
    pub const GZ_HIGH: u8 = 0x40;
    /// Calibration-On-Demand status register Register (R)
    /// Default:  00000000
    pub const COD_STATUS: u8 = 0x46;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_QW_LOW: u8 = 0x49;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_QW_HIGH: u8 = 0x4A;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_QX_LOW: u8 = 0x4B;
    /// Reserved Register (R)
    /// Default:  00000000
    pub const D_QX_HIGH: u8 = 0x4C;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_QY_LOW: u8 = 0x4D;
    /// Reserved Register (R)
    /// Default:  00000000
    pub const D_QY_HIGH: u8 = 0x4E;
    /// Reserved Register (R)
    /// Default:  00000000
    pub const D_QZ_LOW: u8 = 0x4F;
    /// Reserved Register (R)
    /// Default:  00000000
    pub const D_QZ_HIGH: u8 = 0x50;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_VX_LOW: u8 = 0x51;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_VX_HIGH: u8 = 0x52;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_VY_LOW: u8 = 0x53;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_VY_HIGH: u8 = 0x54;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_VZ_LOW: u8 = 0x55;
    /// General purpose register Register (R)
    /// Default:  00000000
    pub const D_VZ_HIGH: u8 = 0x56;
    /// Axis, direction, number of detected Tap Register (R)
    /// Default:  00000000
    pub const TAP_STATUS: u8 = 0x59;
    /// Low byte of step count of Pedometer Register (R)
    /// Default:  00000000
    pub const STEP_CNT_LOW: u8 = 0x5A;
    /// Middle byte of step count of Pedometer Register (R)
    /// Default:  00000000
    pub const STEP_CNT_MID: u8 = 0x5B;
    /// High byte of step count of Pedometer Register (R)
    /// Default:  00000000
    pub const STEP_CNT_HIGH: u8 = 0x5C;
    /// Soft Reset Register Register (W)
    /// Default:  00000000
    pub const RESET: u8 = 0x60;
}
pub mod register {
    pub mod ship_info {

        /// Device Identifier
        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        pub enum DeviceID {
            QST = 0x05,
            Unknown,
        }

        /// Device Revision ID
        pub type RevisionID = u8;
    }

    pub mod ctrl1 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(IntDirection {
            /// Output on Int2
            Int2 = 0b0,
            /// Output on Int1
            Int1 = 0b1,
        });

        bitfield! {
            /// Serial Interface and Sensor Enable
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct Ctrl1Register(u8);
            impl Debug;
            pub bool, sim, set_sim: 7;
            pub bool, addr_ai, set_addr_ai: 6;
            pub bool, be, set_be: 5;
            pub bool, int2_enable, set_int2_enable: 4;
            pub bool, int1_enable, set_int1_enable: 3;
            pub IntDirection, fifo_int_sel, set_fifo_int_sel: 2, 2;
            pub bool, reserved, set_reserved: 1;
            pub bool, sensor_disable, set_sensor_disable: 0;
        }
    }

    pub mod ctrl2 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(AccelerometerFS {
            /// Accelerometer Full-scale = ±2 g
            FS2G = 0b000,
            /// Accelerometer Full-scale = ±4 g
            FS4G = 0b001,
            /// Accelerometer Full-scale = ±8 g
            FS8G = 0b010,
            /// Accelerometer Full-scale = ±16 g
            FS16G = 0b011,
        });

        define_enum_with_bitrange!(AccelerometerODR {
            /// ODR Rate (Hz): (Accel only) N/A
            /// ODR Rate (Hz) (6DOF)(13): 7174.4 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR0 = 0b0000,
            /// ODR Rate (Hz): (Accel only) N/A
            /// ODR Rate (Hz) (6DOF)(13): 3587.2 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR1 = 0b0001,
            /// ODR Rate (Hz): (Accel only) N/A
            /// ODR Rate (Hz) (6DOF)(13): 1793.6
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR2 = 0b0010,
            /// ODR Rate (Hz): (Accel only): 1000
            /// ODR Rate (Hz) (6DOF)(13): 896.8
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR3 = 0b0011,
            /// ODR Rate (Hz): (Accel only): 500
            /// ODR Rate (Hz) (6DOF)(13): 448.4
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR4 = 0b0100,
            /// ODR Rate (Hz): (Accel only): 250
            /// ODR Rate (Hz) (6DOF)(13): 224.2
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR5 = 0b0101,
            /// ODR Rate (Hz): (Accel only): 125
            /// ODR Rate (Hz) (6DOF)(13): 112.1
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR6 = 0b0110,
            /// ODR Rate (Hz): (Accel only): 62
            /// ODR Rate (Hz) (6DOF)(13): 56.05
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR7 = 0b0111,
            /// ODR Rate (Hz): (Accel only): 31
            /// ODR Rate (Hz) (6DOF)(13): 28.025
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalAODR8 = 0b1000,
            /// N/A
            ReservedAODR9 = 0b1001,
            /// N/A
            ReservedAODR10 = 0b1010,
            /// N/A
            ReservedAODR11 = 0b1011,
            /// ODR Rate (Hz): (Accel only): 128
            /// ODR Rate (Hz) (6DOF)(13): N/A
            /// Mode: Low Power
            /// Duty Cycle: 100%
            LowPowerAODR12 = 0b1100,
            /// ODR Rate (Hz): (Accel only): 128
            /// ODR Rate (Hz) (6DOF)(13): N/A
            /// Mode: Low Power
            /// Duty Cycle: 58%
            LowPowerAODR13 = 0b1101,
            /// ODR Rate (Hz): (Accel only): 128
            /// ODR Rate (Hz) (6DOF)(13): N/A
            /// Mode: Low Power
            /// Duty Cycle: 31%
            LowPowerAODR14 = 0b1110,
            /// ODR Rate (Hz): (Accel only): 128
            /// ODR Rate (Hz) (6DOF)(13): N/A
            /// Mode: Low Power
            /// Duty Cycle: 8.5
            LowPowerAODR15 = 0b1111,
        });

        bitfield! {
            /// Accelerometer Settings: Address:
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct Ctrl2Register(u8);
            impl Debug;
            pub bool, ast, set_ast: 7;
            #[doc = "Set Accelerometer Full-scale"]
            pub AccelerometerFS, afs, set_afs: 6, 4;
            #[doc = "Set Accelerometer Output Data Rate (ODR)"]
            pub AccelerometerODR, aodr, set_aodr: 3, 0;
        }

        #[cfg(test)]
        mod test {
            use bitfield::{BitRange, BitRangeMut};

            use super::AccelerometerFS;
            use super::AccelerometerODR;
            use super::Ctrl2Register;

            #[test]
            fn test_afs_bit_range() {
                // Test AFS2G
                let value_2g: u8 = 0b0000_0000;
                assert_eq!(
                    <u8 as BitRange<AccelerometerFS>>::bit_range(&value_2g, 2, 0),
                    AccelerometerFS::FS2G
                );

                // Test AFS4G
                let value_4g: u8 = 0b0000_0001;
                assert_eq!(
                    <u8 as BitRange<AccelerometerFS>>::bit_range(&value_4g, 2, 0),
                    AccelerometerFS::FS4G
                );

                // Test AFS8G
                let value_8g: u8 = 0b0000_0010;
                assert_eq!(
                    <u8 as BitRange<AccelerometerFS>>::bit_range(&value_8g, 2, 0),
                    AccelerometerFS::FS8G
                );

                // Test AFS16G
                let value_16g: u8 = 0b0000_0011;
                assert_eq!(
                    <u8 as BitRange<AccelerometerFS>>::bit_range(&value_16g, 2, 0),
                    AccelerometerFS::FS16G
                );
            }

            #[test]
            fn test_afs_set_bit_range() {
                let mut value: u8 = 0;

                // Set AFS2G
                value.set_bit_range(2, 0, AccelerometerFS::FS2G);
                assert_eq!(value, 0b0000_0000);

                // Set AFS4G
                value.set_bit_range(2, 0, AccelerometerFS::FS4G);
                assert_eq!(value, 0b0000_0001);

                // Set AFS8G
                value.set_bit_range(2, 0, AccelerometerFS::FS8G);
                assert_eq!(value, 0b0000_0010);

                // Set AFS16G
                value.set_bit_range(2, 0, AccelerometerFS::FS16G);
                assert_eq!(value, 0b0000_0011);
            }

            #[test]
            fn test_struct_bit_range() {
                let register: u8 = 0b1000_0111;

                let data = Ctrl2Register(register);
                assert!(data.ast());
                assert_eq!(data.afs(), AccelerometerFS::FS2G);
                assert_eq!(data.aodr(), AccelerometerODR::NormalAODR7);
            }

            #[test]
            fn test_struct_bit_range_bis() {
                let register: u8 = 0b0000_0111;

                let data = Ctrl2Register(register);
                assert!(!data.ast());
                assert_eq!(data.afs(), AccelerometerFS::FS2G);
                assert_eq!(data.aodr(), AccelerometerODR::NormalAODR7);
            }

            #[test]
            fn test_struct_set_bit_range() {
                let register_result: u8 = 0b1000_0111;
                let register_default: u8 = 0b0000_0000;

                let mut data = Ctrl2Register(register_default);
                data.set_ast(true);
                data.set_afs(AccelerometerFS::FS2G);
                data.set_aodr(AccelerometerODR::NormalAODR7);
                assert_eq!(data.0, register_result);
            }

            #[test]
            fn test_struct_set_bit_range_bis() {
                let register_result: u8 = 0b0000_0111;
                let register_default: u8 = 0b0000_0000;

                let mut data = Ctrl2Register(register_default);
                data.set_ast(false);
                data.set_afs(AccelerometerFS::FS2G);
                data.set_aodr(AccelerometerODR::NormalAODR7);
                assert_eq!(data.0, register_result);
            }
        }
    }

    pub mod ctrl3 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(GyroscopeFS {
            /// ±16 dps
            DPS16 = 0b000,
            /// ±32 dps
            DPS32 = 0b001,
            /// ±64 dps
            DPS64 = 0b010,
            /// ±128 dps
            DPS128 = 0b011,
            /// ±256 dps
            DPS256 = 0b100,
            /// ±512 dps
            DPS512 = 0b101,
            /// ±1024 dps
            DPS1024 = 0b110,
            /// ±2048 dps
            DPS2048 = 0b111,
        });

        // Gyroscope Settings
        define_enum_with_bitrange!(GyroscopeODR {
            /// ODR Rate (Hz): 7174.4 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD0 = 0b0000,
            /// ODR Rate (Hz): 3587.2 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD1 = 0b0001,
            /// ODR Rate (Hz): 1793.6 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD2 = 0b0010,
            /// ODR Rate (Hz): 896.8 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD3 = 0b0011,
            /// ODR Rate (Hz): 448.4 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD4 = 0b0100,
            /// ODR Rate (Hz): 224.2 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD5 = 0b0101,
            /// ODR Rate (Hz): 112.1 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD6 = 0b0110,
            /// ODR Rate (Hz): 56.05 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD7 = 0b0111,
            /// ODR Rate (Hz): 28.025 Hz
            /// Mode: Normal
            /// Duty Cycle: 100%
            NormalGORD8 = 0b1000,
        });

        bitfield! {
            /// Gyroscope Settings
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct Ctrl3Register(u8);
            impl Debug;
            #[doc = "Set Gyroscope Self-Test Enable/Disable"]
            pub bool, gst, set_gst: 7;
            #[doc = "Set Gyroscope Full-scale"]
            pub GyroscopeFS, gfs, set_gfs: 6, 4;
            #[doc = "Set Gyroscope Output Data Rate (ODR)"]
            pub GyroscopeODR, godr, set_godr: 3, 0;
        }

        #[cfg(test)]
        mod test {
            use bitfield::{BitRange, BitRangeMut};

            use super::Ctrl3Register;
            use super::GyroscopeFS;
            use super::GyroscopeODR;

            #[test]
            fn test_gfs_bit_range() {
                // Test DPS16
                let value_dps16: u8 = 0b0000_0000;
                assert_eq!(
                    <u8 as BitRange<GyroscopeFS>>::bit_range(&value_dps16, 6, 4),
                    GyroscopeFS::DPS16
                );

                // Test DPS32
                let value_dps32: u8 = 0b0001_0000;
                assert_eq!(
                    <u8 as BitRange<GyroscopeFS>>::bit_range(&value_dps32, 6, 4),
                    GyroscopeFS::DPS32
                );

                // Test DPS64
                let value_dps64: u8 = 0b0010_0000;
                assert_eq!(
                    <u8 as BitRange<GyroscopeFS>>::bit_range(&value_dps64, 6, 4),
                    GyroscopeFS::DPS64
                );

                // Test DPS128
                let value_dps128: u8 = 0b0011_0000;
                assert_eq!(
                    <u8 as BitRange<GyroscopeFS>>::bit_range(&value_dps128, 6, 4),
                    GyroscopeFS::DPS128
                );

                // Test DPS256
                let value_dps256: u8 = 0b0100_0000;
                assert_eq!(
                    <u8 as BitRange<GyroscopeFS>>::bit_range(&value_dps256, 6, 4),
                    GyroscopeFS::DPS256
                );

                // Test DPS512
                let value_dps512: u8 = 0b0101_0000;
                assert_eq!(
                    <u8 as BitRange<GyroscopeFS>>::bit_range(&value_dps512, 6, 4),
                    GyroscopeFS::DPS512
                );

                // Test DPS1024
                let value_dps1024: u8 = 0b0110_0000;
                assert_eq!(
                    <u8 as BitRange<GyroscopeFS>>::bit_range(&value_dps1024, 6, 4),
                    GyroscopeFS::DPS1024
                );

                // Test DPS2048
                let value_dps2048: u8 = 0b0111_0000;
                assert_eq!(
                    <u8 as BitRange<GyroscopeFS>>::bit_range(&value_dps2048, 6, 4),
                    GyroscopeFS::DPS2048
                );
            }

            #[test]
            fn test_gfs_set_bit_range() {
                let mut value: u8 = 0;

                // Set DPS16
                value.set_bit_range(6, 4, GyroscopeFS::DPS16);
                assert_eq!(value, 0b0000_0000);

                // Set DPS32
                value.set_bit_range(6, 4, GyroscopeFS::DPS32);
                assert_eq!(value, 0b0001_0000);

                // Set DPS32
                value.set_bit_range(6, 4, GyroscopeFS::DPS64);
                assert_eq!(value, 0b0010_0000);
            }

            #[test]
            fn test_struct_bit_range() {
                let register: u8 = 0b1000_0111;

                let data = Ctrl3Register(register);
                assert!(data.gst());
                assert_eq!(data.gfs(), GyroscopeFS::DPS16);
                assert_eq!(data.godr(), GyroscopeODR::NormalGORD7);
            }

            #[test]
            fn test_struct_set_bit_range() {
                let register_result: u8 = 0b1000_0111;
                let register_default: u8 = 0b0000_0000;

                let mut data = Ctrl3Register(register_default);
                data.set_gst(true);
                data.set_gfs(GyroscopeFS::DPS16);
                data.set_godr(GyroscopeODR::NormalGORD7);
                assert_eq!(data.0, register_result);
            }
        }
    }

    pub mod ctrl5 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(GLPFMode {
            /// 2.66% of ODR
            LPF2_66 = 0b00,
            /// 3.63% of ODR
            LPF3_63 = 0b01,
            /// 5.39% of ODR
            LPF5_39 = 0b10,
            /// 13.37% of ODR
            LPF13_37 = 0b11,
        });

        define_enum_with_bitrange!(ALPFMode {
            /// 2.66% of ODR
            LPF2_66 = 0b00,
            /// 3.63% of ODR
            LPF3_63 = 0b01,
            /// 5.39% of ODR
            LPF5_39 = 0b10,
            /// 13.37% of ODR
            LPF13_37 = 0b11,
        });

        bitfield! {
            /// Sensor Data Processing Settings
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct Ctrl5Register(u8);
            impl Debug;
            #[doc = "Reserved"]
            pub bool, reserved_7, set_reserved_7: 7;
            #[doc = "Set Gyroscope LPF Mode"]
            pub GLPFMode, glpf, set_glpf: 6, 5;
            #[doc = "Set Gyroscope LPF Enable"]
            pub bool, glpf_enable, set_glpf_enable: 4;
            #[doc = "Reserved"]
            pub bool, reserved_3, set_reserved_3: 3;
            #[doc = "Set Accelerometer LPF Mode"]
            pub ALPFMode, alpf, set_alpf: 2, 0;
            #[doc = "Set Accelerometer LPF Enable"]
            pub bool, alpf_enable, set_alpf_enable: 0;
        }
    }

    pub mod ctrl7 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(Gsn {
            /// Gyroscope in Full Mode (Drive and Sense are enabled)
            FullMode = 0b00,
            /// Gyroscope in Snooze Mode (only Drive enabled)
            SnoozeMode = 0b01,
        });

        bitfield! {
            /// Enable Sensors and COnfigure Data Reads
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct Ctrl7Register(u8);
            impl Debug;
            #[doc = "Set Sync Sample Enable"]
            pub bool, sync_sample_enable, set_sync_sample_enable: 7;
            #[doc = "Reserved"]
            pub bool, reserved_6, set_reserved_6: 6;
            #[doc = "Set Data Ready Disabled\n\
                    DRDY(Data Ready) is enabled, is driven to INT2 pin\n\
                    DRDY(Data Ready) is disabled, is blocked from the INT2 pin\n\
            "]
            pub bool, data_ready_disable, set_data_ready_disable: 5;
            #[doc = "Set Gyroscope Mode. This bit is effective only when Gyroscope is enabled. Refer to 7.1."]
            pub GSN, gsn, set_gsn: 4;
            #[doc = "Reserved"]
            pub u8, reserved_2_3, set_reserved_2_3: 3, 2;
            #[doc = "Set Gyroscope Enable"]
            pub bool, gyroscope_enable, set_gyroscope_enable: 1;
            #[doc = "Set Accelerometer Enable"]
            pub bool, accelerometer_enable, set_accelerometer_enable: 0;
        }
    }

    pub mod ctrl8 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(HandShakeType {
            /// Use INT1 as CTRL9 handshake
            Int1 = 0b0,
            /// Use STATUSINT.bit7 as CTRL9 handshake
            StatusIntBit7 = 0b1,
        });

        define_enum_with_bitrange!(ActivityIntSelect {
            ///  INT2 is used for Activity Detection event interrupt
            Int2 = 0b0,
            /// INT1 is used for Activity Detection event interrupt
            Int1 = 0b1,
        });

        bitfield! {
            /// Enable Sensors and COnfigure Data Reads
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct Ctrl8Register(u8);
            impl Debug;
            #[doc = "Set CTRL9_HandShake_Type"]
            pub HandShakeType, ctrl9_handshake_type, set_ctrl9_handshake_type: 7;
            #[doc = "Set Activity Int Selection\n\
                    ## NOTE: this bit influences the Any/No/Sig-motion, Pedometer, Tap Detection interrupt
            "]
            pub ActivityIntSelect, activity_int_select, set_activity_int_select: 6;
            #[doc = "Reserved"]
            pub u8, reserved, set_reserved: 5;
            #[doc = "Set Pedo Enable"]
            pub bool, pedo_enable, set_pedo_enable: 4;
            #[doc = "Set Sig Motion Enable"]
            pub bool, sig_motion_enable, set_sig_motion_enable: 3;
            #[doc = "Set No Motion Enable"]
            pub bool, no_motion_enable, set_no_motion_enable: 2;
            #[doc = "Set Any Motion Enable"]
            pub bool, any_motion_enable, set_any_motion_enable: 1;
            #[doc = "Set Tap Enable"]
            pub bool, tap_enable, set_tap_enable: 0;
        }
    }

    pub mod ctrl9 {
        #[derive(Debug, PartialEq, Eq, Clone, Copy)]
        #[repr(u8)]
        pub enum Ctrl9Register {
            /// Acknowledgement. Host acknowledges to QMI8658, to end the protocol.
            /// Prototype Type: Ctrl9
            CtrlCmdAck = 0x00,
            /// Reset FIFO from Host
            /// Prototype Type: Ctrl9
            CtrlCmdRstFifo = 0x04,
            /// Get FIFO data from Device
            /// Prototype Type: Ctrl9
            CtrlCmdReqFifo = 0x05,
            /// Set up and enable Wake on Motion (WoM)
            /// Prototype Type: WCtrl9
            CtrlCmdWriteWomSetting = 0x08,
            /// Change accelerometer offset
            /// Prototype Type: WCtrl9
            CtrlCmdAccelHostDeltaOffset = 0x09,
            /// Change gyroscope offset
            /// Prototype Type: WCtrl9
            CtrlCmdGyroHostDeltaOffset = 0x0A,
            /// Configure Tap detection
            /// Prototype Type: WCtrl9
            CtrlCmdConfigureTap = 0x0C,
            /// Configure Pedometer
            /// Prototype Type: WCtrl9
            CtrlCmdConfigurePedometer = 0x0D,
            /// Configure Any Motion / No Motion / Significant Motion detection
            /// Prototype Type: WCtrl9
            CtrlCmdConfigureMotion = 0x0E,
            /// Reset pedometer count (step count)
            /// Prototype Type: WCtrl9
            CtrlCmdResetPedometer = 0x0F,
            /// Copy USID and FW Version to UI registers
            /// Prototype Type: Ctrl9R
            CtrlCmdCopyUsid = 0x10,
            /// Configures IO pull-ups
            /// Prototype Type: WCtrl9
            CtrlCmdSetRpu = 0x11,
            /// Internal AHB clock gating switch
            /// Prototype Type: WCtrl9
            CtrlCmdAhbClockGating = 0x12,
            /// On-Demand Calibration on gyroscope
            /// Prototype Type: WCtrl9
            CtrlCmdOnDemandCalibration = 0xA2,
            /// Restore the saved Gyroscope gains
            /// Prototype Type: WCtrl9
            CtrlCmdApplyGyroGains = 0xAA,
        }
    }

    pub mod cal {
        use bitfield::bitfield;

        bitfield! {
            /// FIFO Control Register
            #[derive(Debug, PartialEq, Eq, Clone, Copy, Default)]
            pub struct Calibration(u16);
            #[doc = "HIGH"]
            pub u8, high, set_high: 15, 8;
            #[doc = "LOW"]
            pub u8, low, set_low: 7, 0;
        }

        /// Host Controlled Calibration Registers
        #[derive(Debug, PartialEq, Eq, Clone, Copy, Default)]
        pub struct CalibrationRegisters {
            pub cal1: Calibration,
            pub cal2: Calibration,
            pub cal3: Calibration,
            pub cal4: Calibration,
        }
    }

    pub mod fifo_wtm_th {

        /// Number of ODRs(Samples) needed to trigger FIFO watermark
        pub type FIFOWTMRegister = u8;
    }

    pub mod fifo_ctrl {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(FIFOMode {
            /// Bypass (FIFO disable)
            Bypass = 0b00,
            /// FIFO
            Fifo = 0b01,
            /// Stream
            Stream = 0b10,
            /// Reserved
            Reserved = 0b11,
        });

        define_enum_with_bitrange!(FIFOSize {
            /// 16 samples
            Size16 = 0b00,
            /// 32 samples
            Size32 = 0b01,
            /// 64 samples
            Size64 = 0b10,
            /// 128 samples
            Size128 = 0b11,
        });

        bitfield! {
            /// FIFO Control Register
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct FIFOControlRegister(u8);
            impl Debug;
            #[doc = "FIFO Read Mode\n\
                    FIFO is in Write mode, sensor data (if enabled) can be filled into FIFO\n\
                    FIFO is in Read mode, FIFO data can be read via FIFO_DATA register\n\
            "]
            pub bool, fifo_rd_mode, set_fifo_rd_mode: 7;
            #[doc = "Reserved"]
            pub u8, reserved_4_6, set_reserved_4_6: 6, 4;
            #[doc = "FIFO Size"]
            pub FIFOSize, fifo_size, set_fifo_size: 3, 2;
            #[doc = "FIFO Mode"]
            pub FIFOMode, fifo_mode, set_fifo_mode: 1, 0;
        }
    }

    pub mod fifo_sample_count {
        pub type FIFOSampleCountRegister = u8;
    }

    pub mod fifo_status {
        use bitfield::bitfield;

        bitfield! {
            /// FIFO Status Register
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct FIFOStatusRegister(u8);
            impl Debug;
            #[doc = "FIFO is full"]
            pub bool, fifo_full, set_fifo_full: 7;
            #[doc = "FIFO Water Mark Level is hit"]
            pub bool, fifo_wtm, set_fifo_wtm: 6;
            #[doc = "FIFO Overflow condition has happened"]
            pub bool, fifo_overflow, set_fifo_overflow: 5;
            #[doc = "FIFO Not Empty"]
            pub bool, fifo_not_empty, set_fifo_not_empty: 4;
            #[doc = "Reserved"]
            pub u8, reserved_3_2, set_reserved_3_2: 3, 2;
            #[doc = "2 MS bits of FIFO Sample Count in word (2bytes)"]
            pub u8, fifo_smpl_cnt_msb, set_fifo_smpl_cnt_msb: 1, 0;
        }
    }

    pub mod fifo_data {
        use bitfield::bitfield;

        bitfield! {
            /// FIFO Data Register
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct FIFODataRegister(u8);
            impl Debug;
            #[doc = "FIFO Data, 8 bit FIFO data output."]
            pub bool, fifo_data: 7, 0;
        }
    }

    pub mod status_int {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(Ctrl9CmdDone {
            /// Not Completed
            NotCompleted = 0b0,
            /// Done
            Done = 0b1,
        });

        bitfield! {
            /// Sensor Data Available and Lock Register
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct SensorDataAvailableAndLockRegister(u8);
            impl Debug;
            #[doc = "Indicates CTRL9 Command was done, as part of CTRL9 protocol"]
            pub Ctrl9CmdDone, ctrl9_cmd_done, set_ctrl9_cmd_done: 7, 7;
            #[doc = "Reserved"]
            pub u8, reserved_2_6, set_reserved_2_6: 6, 2;
            #[doc = "if `ctrl7.sync_sample_enable` is enabled then `false` sensor data is not locked, \
                    else `true` sensor data is locked.\n\
                    if `ctrl7.sync_sample_enable` is disabled then the bit shows the same value of INT1.\n\
            "]
            pub bool, locked, set_locked: 1;
            #[doc = "if `ctrl7.sync_sample_enable` is enabled then `false` sensor data is not available, \
                    else `true` sensor data is available for reading.\n\
                    if `ctrl7.sync_sample_enable` is disabled then the bit shows the same value of INT2.\n\
            "]
            pub bool, available, set_available: 0;
        }

        #[cfg(test)]
        mod test {
            use bitfield::BitRange;

            use crate::command::register::status_int::{
                Ctrl9CmdDone, SensorDataAvailableAndLockRegister,
            };

            #[test]
            fn test_cmd_done_done() {
                // Test AFS2G
                let value_2g: u8 = 0b1000_0000;
                assert_eq!(
                    <u8 as BitRange<Ctrl9CmdDone>>::bit_range(&value_2g, 7, 7),
                    Ctrl9CmdDone::Done
                );
            }

            #[test]
            fn test_cmd_done_read_done() {
                let register: u8 = 128;

                let data = SensorDataAvailableAndLockRegister(register);
                assert_eq!(data.ctrl9_cmd_done(), Ctrl9CmdDone::Done);
            }

            #[test]
            fn test_cmd_available_read() {
                let register: u8 = 0b0000_0001;

                let data = SensorDataAvailableAndLockRegister(register);
                assert!(data.available());
            }
        }
    }

    pub mod status_0 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(UpdateData {
            /// No updates since last read
            NoUpdate = 0b00,
            /// New data available
            NewData = 0b01,
        });

        bitfield! {
            /// Output Data Status Register
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct OutputDataStatusRegister(u8);
            impl Debug;
            #[doc = "Reserved"]
            pub u8, reserved_2_7, set_reserved_2_7: 7, 2;
            #[doc = "Gyroscope new data available"]
            pub UpdateData, gyroscope_data_available, set_gyroscope_data_available: 1;
            #[doc = "Accelerometer new data available"]
            pub UpdateData, accelerometer_data_available, set_accelerometer_data_available: 0;
        }
    }

    pub mod status_1 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(EngineDetect {
            /// Not Detected
            NotDetected = 0b00,
            /// Detected
            Detected = 0b01,
        });

        bitfield! {
            /// Miscellaneous Status Register
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct MiscellaneousStatusRegister(u8);
            impl Debug;
            #[doc = "Significant Motion\n\
                * No Significant-Motion was detected\n\
                * Significant-Motion was detected\n\
            "]
            pub EngineDetect, significant_motion, set_significant_motion: 7;
            #[doc = "No Motion\n\
                * No No-Motion was detected\n\
                * No-Motion was detected\n\
            "]
            pub EngineDetect, no_motion, set_no_motion: 6;
            #[doc = "Any Motion\n\
                * No Any-Motion was detected\n\
                * Any-Motion was detected\n\
            "]
            pub EngineDetect, any_motion, set_any_motion: 5;
            #[doc = "Pedometer\n\
                * No step was detected\n\
                * step was detected\n\
            "]
            pub EngineDetect, pedometer, set_pedometer: 4;
            #[doc = "Reserved"]
            pub u8, reserved_3, set_reserved_3: 3;
            #[doc = "WoM\n\
                * No WoM was detected\n\
                * WoM was detected\n\
            "]
            pub EngineDetect, wom, set_wom: 2;
            #[doc = "Tap\n\
                * No Tap was detected\n\
                * Tap was detected\n\
            "]
            pub EngineDetect, tap, set_tap: 1;
            #[doc = "Reserved"]
            pub u8, reserved_0, set_reserved_0: 0;
        }
    }

    pub mod timestamp {
        //! Sample time stamp. Count incremented by one for each sample
        //! (x, y, z data set) from sensor with highest ODR (circular register
        //! 0x0-0xFFFFFF).
        //!

        /// Time Stamp Register Size
        pub type TimeStampRegister = u8;

        /// Sample Time Stamp
        pub type SampleTimeStamp = u32;
    }

    pub mod temp {
        //! Temperature output (°C) in two’s complement.
        //! T = `TEMP_H` + (`TEMP_L` / 256)

        /// Temperature Register Size
        pub type TempRegister = u8;

        /// Temperature Sensor Register Address: 0x33 – 0x34
        pub type Temperature = f32;
    }

    pub mod acceleration {
        /// Acceleration Register Size
        pub type AccelerationRegister = u8;

        /// Acceleration Register Size
        pub type Acceleration = f32;

        /// Angular Register Size
        pub type AngularRegister = u8;

        /// Angular Register Size
        pub type Angular = f32;

        /// Acceleration Output. Register Address: 0x35 – 0x3A
        #[derive(Debug, Default, PartialEq, Clone, Copy)]
        pub struct AccelerationOutput {
            /// AX
            pub x: Acceleration,
            /// AY
            pub y: Acceleration,
            /// AZ
            pub z: Acceleration,
        }

        /// Angular Rate Output. Register Address: 0x3B – 0x40
        #[derive(Default, Debug, PartialEq, Clone, Copy)]
        pub struct AngularRateOutput {
            /// AX
            pub x: Angular,
            /// AY
            pub y: Angular,
            /// AZ
            pub z: Angular,
        }
    }

    pub mod cod_status {
        use bitfield::bitfield;

        bitfield! {
            #[doc = "COD Status Register"]
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct CODStatusRegister(u8);
            impl Debug;
            #[doc = "COD passed for checking low sensitivity limit of X axis of gyroscope\n\
            COD failed for checking low sensitivity limit of X axis of gyroscope"]
            pub bool, x_limit_l_fail, set_x_limit_l_fail: 7;
            #[doc = "COD passed for checking high sensitivity limit of X axis of gyroscope\n\
            COD failed for checking high sensitivity limit of X axis of gyroscope"]
            pub bool, x_limit_h_fail, set_x_limit_h_fail: 6;
            #[doc = "COD passed for checking low sensitivity limit of Y axis of gyroscope\n\
            COD failed for checking low sensitivity limit of Y axis of gyroscope"]
            pub bool, y_limit_l_fail, set_y_limit_l_fail: 5;
            #[doc = "COD passed for checking high sensitivity limit of Y axis of gyroscope\n\
            COD failed for checking high sensitivity limit of Y axis of gyroscope"]
            pub bool, y_limit_h_fail, set_y_limit_h_fail: 4;
            #[doc = "Accelerometer checked pass (no significant vibration happened during COD)\n\
            Accelerometer checked failed (significant vibration happened during COD)"]
            pub bool, accel_check, set_accel_check: 3;
            #[doc = "Gyroscope startup succeeded\n\
            Gyroscope startup failure happened when COD was called"]
            pub bool, startup_failed, set_startup_failed: 2;
            #[doc = "COD was called when gyroscope was not enabled\n\
            COD was called while gyroscope was enabled, COD return failure"]
            pub bool, gyro_enabled, set_gyro_enabled: 1;
            #[doc = "COD succeeded, new gain parameters will be applied to GX & GY data\n\
            COD failed; no COD correction applied"]
            pub bool, cod_failed, set_cod_failed: 0;
        }
    }

    pub mod tap_status {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(TapPolarity {
            /// Tap was detected on the positive direction of the Tap axis
            Positive = 0b0,
            /// Tap was detected on the negative direction of the Tap axis
            Negative = 0b1,
        });

        define_enum_with_bitrange!(TapAxis {
            /// No Tap was detected
            None = 0b00,
            /// Tap was detected on X axis
            XAxis = 0b01,
            /// Tap was detected on Y axis
            YAxis = 0b10,
            /// Tap was detected on Z axis
            ZAxis = 0b11,
        });

        define_enum_with_bitrange!(TapNumber {
            /// No Tap was detected
            None = 0b00,
            /// Single-Tap was detected
            Single = 0b01,
            /// Double-Tap was detected
            Double = 0b10,
            /// NA
            NA = 0b11,
        });

        bitfield! {
            /// Tap Status Register
            #[doc = "Tap Status Register"]
            #[derive(Clone, Copy, PartialEq, Eq)]
            pub struct TapStatusRegister(u8);
            impl Debug;
            #[doc = "Tap was detected on the positive direction of the Tap axis\n\
                    Tap was detected on the negative direction of the Tap axis\
            "]
            pub TapPolarity, tap_polarity, set_tap_polarity: 7;
            #[doc = "Reserved"]
            pub u8, reserved_6, set_reserved_6: 6;
            #[doc = "No Tap was detected\n\
                    Tap was detected on X axis\n\
                    Tap was detected on Y axis\n\
                    Tap was detected on Z axis\
            "]
            pub TapAxis, tap_axis, set_tap_axis: 5, 4;
            #[doc = "Reserved"]
            pub u8, reserved_3_2, set_reserved_3_2: 3, 2;
            #[doc = "No Tap was detected\n\
                    Single-Tap was detected\n\
                    Double-Tap was detected\n\
                    NA\
            "]
            pub TapNumber, tap_num, set_tap_num: 1, 0;
        }
    }

    pub mod step_count {

        /// Step Count Register Size
        pub type StepCountRegister = u8;

        /// 24 `BitCounter`
        pub type StepCounter = u32;
    }

    pub mod reset {

        /// Reset Register
        pub type ResetRegister = u8;
    }
}
