#![cfg_attr(not(test), no_std)]

use serde::{Deserialize, Serialize};

pub mod io;
pub mod motor_control;
pub mod protocol;
pub mod uart_com;
pub mod utility;

extern crate alloc;

/// enum that keeps track of the mode of the drone
#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Copy)]
pub enum DroneMode {
    Safe,
    Manual,
    Panic,
    Calibrate,
    YawControl,
    FullControl,
    RawMode,
}
