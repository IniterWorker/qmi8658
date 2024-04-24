#![allow(dead_code)]

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
pub const STEP_CNT_LOWOW: u8 = 0x5A;
/// Middle byte of step count of Pedometer Register (R)
/// Default:  00000000
pub const STEP_CNT_MIDL: u8 = 0x5B;
/// High byte of step count of Pedometer Register (R)
/// Default:  00000000
pub const STEP_CNT_HIGHIGH: u8 = 0x5C;
/// Soft Reset Register Register (W)
/// Default:  00000000
pub const RESET: u8 = 0x60;

pub mod register {
    pub mod ship_info {

        /// Device Identifier
        #[derive(Debug, Clone, Copy)]
        #[repr(u8)]
        pub enum DeviceID {
            QSTSensor = 0x05,
            Unknown,
        }

        /// Device Revision ID
        pub type RevisionID = u8;
    }

    pub mod ctrl1 {
        use bitfield::bitfield;

        bitfield! {
            /// Serial Interface and Sensor Enable
            pub struct Ctrl1Register(u8);
            impl Debug;
            bool, sim, set_sim: 7;
            bool, addr_ai, set_addr_ai: 6;
            bool, be, set_be: 5;
            bool, int2_enable, set_int2_enable: 4;
            bool, int1_enable, set_int1_enable: 3;
            bool, fifo_int_sel, set_fifo_int_sel: 2;
            bool, reserved, set_reserved: 1;
            bool, sensor_disable, set_sensor_disable: 0;
        }
    }

    pub mod ctrl2 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(Afs {
            /// Accelerometer Full-scale = ±2 g
            AFS2G = 0b000,
            /// Accelerometer Full-scale = ±4 g
            AFS4G = 0b001,
            /// Accelerometer Full-scale = ±8 g
            AFS8G = 0b010,
            /// Accelerometer Full-scale = ±16 g
            AFS16G = 0b011,
        });

        define_enum_with_bitrange!(Aodr {
            /// N/A 7174.4 Hz, Normal, 100%
            NormalAODR0 = 0b0000,
            /// N/A 3587.2 Hz, Normal, 100%
            NormalAODR1 = 0b0001,
            /// N/A 1793.6 Hz, Normal, 100%
            NormalAODR2 = 0b0010,
            /// 1000 896.8 Hz, Normal, 100%
            NormalAODR3 = 0b0011,
            /// 500 448.4  Hz, Normal, 100%
            NormalAODR4 = 0b0100,
            /// 250 224.2 Hz, Normal, 100%
            NormalAODR5 = 0b0101,
            /// 125 112.1 Hz, Normal, 100%
            NormalAODR6 = 0b0110,
            /// 62.5 56.05  Hz, Normal, 100%
            NormalAODR7 = 0b0111,
            /// 31.25 28.025 Hz, Normal, 100%
            NormalAODR8 = 0b1000,
            /// N/A
            ReservedAODR9 = 0b1001,
            /// N/A
            ReservedAODR10 = 0b1010,
            /// N/A
            ReservedAODR11 = 0b1011,
            /// Low Power, 100%
            LowPowerAODR12 = 0b1100,
            /// Low Power, 58%
            LowPowerAODR13 = 0b1101,
            /// Low Power, 31%
            LowPowerAODR14 = 0b1110,
            /// Low Power, 8.5%
            LowPowerAODR15 = 0b1111,
        });

        bitfield! {
            /// Accelerometer Settings: Address:
            pub struct Ctrl2Register(u8);
            impl Debug;
            bool, ast, set_ast: 7;
            #[doc = "Set Accelerometer Full-scale"]
            Afs, afs, set_afs: 6, 4;
            #[doc = "Set Accelerometer Output Data Rate (ODR)"]
            Aodr, aodr, set_aodr: 3, 0;
        }

        #[cfg(test)]
        mod test {
            use bitfield::{BitRange, BitRangeMut};

            use super::Afs;
            use super::Aodr;
            use super::Ctrl2Register;

            #[test]
            fn test_afs_bit_range() {
                // Test AFS2G
                let value_2g: u8 = 0b0000_0000;
                assert_eq!(
                    <u8 as BitRange<Afs>>::bit_range(&value_2g, 2, 0),
                    Afs::AFS2G
                );

                // Test AFS4G
                let value_4g: u8 = 0b0000_0001;
                assert_eq!(
                    <u8 as BitRange<Afs>>::bit_range(&value_4g, 2, 0),
                    Afs::AFS4G
                );

                // Test AFS8G
                let value_8g: u8 = 0b0000_0010;
                assert_eq!(
                    <u8 as BitRange<Afs>>::bit_range(&value_8g, 2, 0),
                    Afs::AFS8G
                );

                // Test AFS16G
                let value_16g: u8 = 0b0000_0011;
                assert_eq!(
                    <u8 as BitRange<Afs>>::bit_range(&value_16g, 2, 0),
                    Afs::AFS16G
                );
            }

            #[test]
            fn test_afs_set_bit_range() {
                let mut value: u8 = 0;

                // Set AFS2G
                value.set_bit_range(2, 0, Afs::AFS2G);
                assert_eq!(value, 0b0000_0000);

                // Set AFS4G
                value.set_bit_range(2, 0, Afs::AFS4G);
                assert_eq!(value, 0b0000_0001);

                // Set AFS8G
                value.set_bit_range(2, 0, Afs::AFS8G);
                assert_eq!(value, 0b0000_0010);

                // Set AFS16G
                value.set_bit_range(2, 0, Afs::AFS16G);
                assert_eq!(value, 0b0000_0011);
            }

            #[test]
            fn test_struct_bit_range() {
                let register: u8 = 0b1000_0111;

                let data = Ctrl2Register(register);
                assert!(data.ast());
                assert_eq!(data.afs(), Afs::AFS2G);
                assert_eq!(data.aodr(), Aodr::NormalAODR7);
            }

            #[test]
            fn test_struct_set_bit_range() {
                let register_result: u8 = 0b1000_0111;
                let register_default: u8 = 0b0000_0000;

                let mut data = Ctrl2Register(register_default);
                data.set_ast(true);
                data.set_afs(Afs::AFS2G);
                data.set_aodr(Aodr::NormalAODR7);
                assert_eq!(data.0, register_result);
            }
        }
    }

    pub mod ctrl3 {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(Gfs {
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
        define_enum_with_bitrange!(Godr {
            /// 7174.4 Hz Normal 100%
            Hz7174_4 = 0b0000,
            /// 3587.2 Hz Normal 100%
            Hz3587_2 = 0b0001,
            /// 1793.6 Hz Normal 100%
            Hz1793_6 = 0b0010,
            /// 896.8 Hz Normal 100%
            Hz896_8 = 0b0011,
            /// 448.4 Hz Normal 100%
            Hz448_4 = 0b0100,
            /// 224.2 Hz Normal 100%
            Hz224_2 = 0b0101,
            /// 112.1 Hz Normal 100%
            Hz112_1 = 0b0110,
            /// 56.05 Hz Normal 100%
            Hz56_05 = 0b0111,
            /// 28.025 Hz Normal 100%
            Hz28_025 = 0b1000,
        });

        bitfield! {
            pub struct Ctrl3Register(u8);
            impl Debug;
            bool, ast, set_ast: 7;
            #[doc = "Set Gyroscope Full-scale"]
            Gfs, gfs, set_gfs: 6, 4;
            #[doc = "Set Gyroscope Output Data Rate (ODR)"]
            Godr, godr, set_godr: 3, 0;
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
            pub struct Ctrl5Register(u8);
            impl Debug;
            #[doc = "Reserved"]
            bool, reserved_7, set_reserved_7: 7;
            #[doc = "Set Gyroscope LPF Mode"]
            GLPFMode, glpf, set_glpf: 6, 5;
            #[doc = "Set Gyroscope LPF Enable"]
            bool, glpf_enable, set_glpf_enable: 4;
            #[doc = "Reserved"]
            bool, reserved_3, set_reserved_3: 3;
            #[doc = "Set Accelerometer LPF Mode"]
            ALPFMode, alpf, set_alpf: 2, 0;
            #[doc = "Set Accelerometer LPF Enable"]
            bool, alpf_enable, set_alpf_enable: 0;
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
            pub struct Ctrl7Register(u8);
            impl Debug;
            #[doc = "Set Sync Sample Enable"]

            bool, sync_sample_enable, set_sync_sample_enable: 7;
            #[doc = "Reserved"]
            bool, reserved_6, set_reserved_6: 6;
            #[doc = "Set Data Ready Disabled\n\
                    DRDY(Data Ready) is enabled, is driven to INT2 pin\n\
                    DRDY(Data Ready) is disabled, is blocked from the INT2 pin\n\
            "]
            bool, data_ready_disable, set_data_ready_disable: 5;
            #[doc = "Set Gyroscope Mode. This bit is effective only when Gyroscope is enabled. Refer to 7.1."]
            GSN, gsn, set_gsn: 4;
            #[doc = "Reserved"]
            u8, reserved_3, set_reserved_3: 3, 2;
            #[doc = "Set Gyroscope Enable"]
            bool, gyroscope_enable, set_gyroscope_enable: 1;
            #[doc = "Set Accelerometer Enable"]
            bool, accelerometer_enable, set_accelerometer_enable: 0;
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
            pub struct Ctrl8Register(u8);
            impl Debug;
            #[doc = "Set CTRL9_HandShake_Type"]
            HandShakeType, ctrl9_handshake_type, set_ctrl9_handshake_type: 7;
            #[doc = "Set Activity Int Selection\n\
                    ## NOTE: this bit influences the Any/No/Sig-motion, Pedometer, Tap Detection interrupt
            "]
            ActivityIntSelect, activity_int_select, set_activity_int_select: 6;
            #[doc = "Set Data Ready Disabled\n\
                    DRDY(Data Ready) is enabled, is driven to INT2 pin\n\
                    DRDY(Data Ready) is disabled, is blocked from the INT2 pin\n\
            "]
            bool, data_ready_disable, set_data_ready_disable: 5;
            #[doc = "Set Gyroscope Mode. This bit is effective only when Gyroscope is enabled. Refer to 7.1."]
            GSN, gsn, set_gsn: 4;
            #[doc = "Reserved"]
            u8, reserved_3, set_reserved_3: 3, 2;
            #[doc = "Set Gyroscope Enable"]
            bool, gyroscope_enable, set_gyroscope_enable: 1;
            #[doc = "Set Accelerometer Enable"]
            bool, accelerometer_enable, set_accelerometer_enable: 0;
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

    pub mod fifo_wtm_th {

        /// Number of ODRs(Samples) needed to trigger FIFO watermark
        pub struct FIFOWTMRegister(u8);
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
            pub struct FIFOControlRegister(u8);
            impl Debug;
            ///
            #[doc = "FIFO Read Mode\n\
                    FIFO is in Write mode, sensor data (if enabled) can be filled into FIFO\n\
                    FIFO is in Read mode, FIFO data can be read via FIFO_DATA register\n\
            "]
            bool, fifo_rd_mode, set_fifo_rd_mode: 7;
            #[doc = "Reserved"]
            u8, reserved_4_6, set_reserved_4_6: 6, 4;
            #[doc = "FIFO Size"]
            FIFOSize, fifo_size, set_fifo_size: 3, 2;
            #[doc = "FIFO Mode"]
            FIFOMode, fifo_mode, set_fifo_mode: 1, 0;
        }
    }

    pub mod fifo_sample_count {
        use bitfield::bitfield;

        bitfield! {
            /// FIFO Sample Count Register
            pub struct FIFOSampleCountRegister(u8);
            impl Debug;
            #[doc = "FIFO Count"]
            bool, fifo_smpl_cnt_lsb: 7, 0;
        }
    }

    pub mod fifo_status {
        use bitfield::bitfield;

        bitfield! {
            /// FIFO Status Register
            pub struct FIFOStatusRegister(u8);
            impl Debug;
            #[doc = "FIFO is full"]
            bool, fifo_full, set_fifo_full: 7;
            #[doc = "FIFO Water Mark Level is hit"]
            bool, fifo_wtm, set_fifo_wtm: 6;
            #[doc = "FIFO Overflow condition has happened"]
            bool, fifo_overflow, set_fifo_overflow: 5;
            #[doc = "FIFO Not Empty"]
            bool, fifo_not_empty, set_fifo_not_empty: 4;
            #[doc = "Reserved"]
            u8, reserved_3_2, set_reserved_3_2: 3, 2;
            #[doc = "2 MS bits of FIFO Sample Count in word (2bytes)"]
            u8, fifo_smpl_cnt_msb, set_fifo_smpl_cnt_msb: 1, 0;
        }
    }

    pub mod fifo_data {
        use bitfield::bitfield;

        bitfield! {
            /// FIFO Data Register
            pub struct FIFODataRegister(u8);
            impl Debug;
            #[doc = "FIFO Data, 8 bit FIFO data output."]
            bool, fifo_data: 7, 0;
        }
    }

    pub mod status_int {
        use bitfield::{bitfield, BitRange, BitRangeMut};

        use crate::define_enum_with_bitrange;

        define_enum_with_bitrange!(Ctrl9CmdDone {
            /// Not Completed
            NotCompleted = 0b00,
            /// Done
            Done = 0b01,
        });

        bitfield! {
            /// Sensor Data Available and Lock Register
            pub struct SensorDataAvailableAndLockRegister(u8);
            impl Debug;
            ///
            #[doc = "Indicates CTRL9 Command was done, as part of CTRL9 protocol"]
            Ctrl9CmdDone, ctrl9_cmd_done, set_ctrl9_cmd_done: 7;
            #[doc = "Reserved"]
            u8, reserved_2_6, set_reserved_2_6: 6, 2;
            #[doc = "if `ctrl7.sync_sample_enable` is enabled then `false` sensor data is not locked, \
                    else `true` sensor data is locked.\n\
                    if `ctrl7.sync_sample_enable` is disabled then the bit shows the same value of INT1.\n\
            "]
            bool, locked, set_locked: 1;
            #[doc = "if `ctrl7.sync_sample_enable` is enabled then `false` sensor data is not available, \
                    else `true` sensor data is available for reading.\n\
                    if `ctrl7.sync_sample_enable` is disabled then the bit shows the same value of INT2.\n\
            "]
            bool, available, set_available: 0;
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
            pub struct OutputDataStatusRegister(u8);
            impl Debug;
            ///
            #[doc = "Reserved"]
            u8, reserved_2_7, set_reserved_2_7: 7, 2;
            #[doc = "Gyroscope new data available"]
            UpdateData, gyroscope_data_available, set_gyroscope_data_available: 1;
            #[doc = "Accelerometer new data available"]
            UpdateData, accelerometer_data_available, set_accelerometer_data_available: 0;
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
            pub struct MiscellaneousStatusRegister(u8);
            impl Debug;
            #[doc = "Significant Motion\n\
                * No Significant-Motion was detected\n\
                * Significant-Motion was detected\n\
            "]
            EngineDetect, significant_motion, set_significant_motion: 7;
            #[doc = "No Motion\n\
                * No No-Motion was detected\n\
                * No-Motion was detected\n\
            "]
            EngineDetect, no_motion, set_no_motion: 6;
            #[doc = "Any Motion\n\
                * No Any-Motion was detected\n\
                * Any-Motion was detected\n\
            "]
            EngineDetect, any_motion, set_any_motion: 5;
            #[doc = "Pedometer\n\
                * No step was detected\n\
                * step was detected\n\
            "]
            EngineDetect, pedometer, set_pedometer: 4;
            #[doc = "Reserved"]
            u8, reserved_3, set_reserved_3: 3;
            #[doc = "WoM\n\
                * No WoM was detected\n\
                * WoM was detected\n\
            "]
            EngineDetect, wom, set_wom: 2;
            #[doc = "Tap\n\
                * No Tap was detected\n\
                * Tap was detected\n\
            "]
            EngineDetect, tap, set_tap: 1;
            #[doc = "Reserved"]
            u8, reserved_0, set_reserved_0: 0;
        }
    }

    pub mod timestamp {
        //! Sample time stamp. Count incremented by one for each sample
        //! (x, y, z data set) from sensor with highest ODR (circular register
        //! 0x0-0xFFFFFF).
        pub struct SampleTimeStamp(u32);

        pub struct TimeStampRegister(u8);
    }

    pub mod temp {
        //! Temperature output (°C) in two’s complement.
        //! T = `TEMP_H` + (`TEMP_L` / 256)

        pub struct TempRegister(u8);

        /// Temperature Sensor Register Address: 0x33 – 0x34
        pub struct Temperature(u16);
    }

    pub mod acceleration {
        pub struct AccelerationRegister(u8);
        pub struct AngularRegister(u8);

        /// Acceleration Output. Register Address: 0x35 – 0x3A
        pub struct AccelerationOutput {
            /// AX
            pub x: u16,
            /// AY
            pub y: u16,
            /// AZ
            pub z: u16,
        }

        /// Angular Rate Output. Register Address: 0x3B – 0x40
        pub struct AngularRateOutput {
            /// AX
            pub x: u16,
            /// AY
            pub y: u16,
            /// AZ
            pub z: u16,
        }
    }

    pub mod cod_status {
        use bitfield::bitfield;

        bitfield! {
            #[doc = "COD Status Register"]
            pub struct CODStatusRegister(u8);
            impl Debug;
            #[doc = "COD passed for checking low sensitivity limit of X axis of gyroscope\n\
            COD failed for checking low sensitivity limit of X axis of gyroscope"]
            bool, x_limit_l_fail, set_x_limit_l_fail: 7;
            #[doc = "COD passed for checking high sensitivity limit of X axis of gyroscope\n\
            COD failed for checking high sensitivity limit of X axis of gyroscope"]
            bool, x_limit_h_fail, set_x_limit_h_fail: 6;
            #[doc = "COD passed for checking low sensitivity limit of Y axis of gyroscope\n\
            COD failed for checking low sensitivity limit of Y axis of gyroscope"]
            bool, y_limit_l_fail, set_y_limit_l_fail: 5;
            #[doc = "COD passed for checking high sensitivity limit of Y axis of gyroscope\n\
            COD failed for checking high sensitivity limit of Y axis of gyroscope"]
            bool, y_limit_h_fail, set_y_limit_h_fail: 4;
            #[doc = "Accelerometer checked pass (no significant vibration happened during COD)\n\
            Accelerometer checked failed (significant vibration happened during COD)"]
            bool, accel_check, set_accel_check: 3;
            #[doc = "Gyroscope startup succeeded\n\
            Gyroscope startup failure happened when COD was called"]
            bool, startup_failed, set_startup_failed: 2;
            #[doc = "COD was called when gyroscope was not enabled\n\
            COD was called while gyroscope was enabled, COD return failure"]
            bool, gyro_enabled, set_gyro_enabled: 1;
            #[doc = "COD succeeded, new gain parameters will be applied to GX & GY data\n\
            COD failed; no COD correction applied"]
            bool, cod_failed, set_cod_failed: 0;
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
            pub struct TapStatusRegister(u8);
            impl Debug;
            #[doc = "Tap was detected on the positive direction of the Tap axis\n\
                    Tap was detected on the negative direction of the Tap axis\
            "]
            TapPolarity, tap_polarity, set_tap_polarity: 7;
            #[doc = "Reserved"]
            u8, reserved_6, set_reserved_6: 6;
            #[doc = "No Tap was detected\n\
                    Tap was detected on X axis\n\
                    Tap was detected on Y axis\n\
                    Tap was detected on Z axis\
            "]
            TapAxis, tap_axis, set_tap_axis: 5, 4;
            #[doc = "Reserved"]
            u8, reserved_3_2, set_reserved_3_2: 3, 2;
            #[doc = "No Tap was detected\n\
                    Single-Tap was detected\n\
                    Double-Tap was detected\n\
                    NA\
            "]
            TapNumber, tap_num, set_tap_num: 1, 0;
        }
    }

    pub mod step_count {
        pub struct StepCountRegister(u8);

        /// 24 `BitCounter`
        pub struct StepCounter(u32);
    }

    pub mod reset {
        pub struct ResetRegister(u8);
    }
}
