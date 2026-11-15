//! Common imports for application code: `use qmi8658::prelude::*;`

#[cfg(feature = "async")]
pub use crate::Qmi8658Async;
pub use crate::config::{AccelConfig, Config, GyroConfig};
pub use crate::registers::{AccelOdr, AccelRange, GyroRange, Odr};
pub use crate::{DeviceAddress, Qmi8658};
