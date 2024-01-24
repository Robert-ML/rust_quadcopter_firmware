use serde::{Deserialize, Serialize};

use crate::{uart_com, DroneMode};
use fixed::types::I16F16;

pub const DEFAULT_CAP: usize = 32;

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub enum DataT {
    // PC -> Drone
    Control(ControlDT),
    Mode(DroneMode),
    Tuning(TuningDT),

    // Drone -> PC
    SensorData(SensorDT),
    HealthData(HealthDT),
    MotorsState(MotorsDT),

    // Error warning types used
    Warning(WarningDT),
    //Error(ErrorDT),

    // Duplex
    Message(heapless::String<DEFAULT_CAP>),
    AckNack(u8), // 1 means ACK | 0 means NACK
    KeepAlive,

    Empty, // no data, to be used for signaling nothing was read

    CalibratedAck(CalibratedValuesDT),

    SensorReading(SensorValuesDT),

    SonsorNotCalibrated,

    MovementErrors(CalculatedErrors),

    UpdateP(UpdatePDT),
    UpdateP1P2(UpdateP1P2DT),

    StartLogging,
    StopLogging,
    StartLogReporting,
    StopLogReporting,
    SensorLog(SensorLogDT),
}

impl DataT {
    pub fn to_packet<const CAP: usize>(&self) -> Result<heapless::Vec<u8, CAP>, uart_com::Error> {
        match postcard::to_vec::<DataT, CAP>(self) {
            Ok(serialized) => {
                return uart_com::frame::<CAP>(serialized);
            }
            Err(_) => {
                return Err(uart_com::Error::ENOMEM);
            }
        };
    }

    pub fn from_packet<const CAP: usize>(
        frame: heapless::Vec<u8, CAP>,
    ) -> Result<DataT, uart_com::Error> {
        let serialized = uart_com::unframe(frame)?;

        match postcard::from_bytes::<DataT>(&serialized) {
            Ok(data) => {
                return Ok(data);
            }
            Err(_) => {
                return Err(uart_com::Error::Des);
            }
        };
    }
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Copy)]
pub struct UpdateP1P2DT {
    pub p1: I16F16,
    pub p2: I16F16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Copy)]
pub struct UpdatePDT {
    pub p: I16F16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Copy)]
pub struct CalculatedErrors {
    pub yaw_error: I16F16,
    pub pitch_error: I16F16,
    pub roll_error: I16F16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Copy)]
pub struct SensorValuesDT {
    pub gyro_pitch: I16F16,
    pub gyro_roll: I16F16,
    pub gyro_yaw: I16F16,
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Copy)]
pub struct CalibratedValuesDT {
    pub gyro_pitch_offset: I16F16,
    pub gyro_roll_offset: I16F16,
    pub gyro_yaw_offset: I16F16,
    pub accel_x_offset: i16,
    pub accel_y_offset: i16,
    pub accel_z_offset: i16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Copy)]
pub struct ControlDT {
    pub lift: u16,
    pub roll: u16,
    pub pitch: u16,
    pub yaw: u16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub struct TuningDT {
    pub p: I16F16,
    pub i: I16F16,
    pub d: I16F16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub struct SensorDT {
    pub sp: u8,
    pub sq: u8,
    pub sr: u8,
    pub sax: u8,
    pub say: u8,
    pub saz: u8,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub struct HealthDT {
    pub bat: u16, // battery percentage
    pub cpu: u8,  // CPU utilization as a percentage
    pub pres: u8,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub struct MotorsDT {
    pub ae1: u16,
    pub ae2: u16,
    pub ae3: u16,
    pub ae4: u16,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone)]
pub enum WarningDT {
    ControlNotNeutral,
    SensorNotCalibrated,
}

#[derive(Serialize, Deserialize, Debug, PartialEq, Clone, Copy)]
pub struct SensorLogDT {
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub roll: I16F16,
    pub pitch: I16F16,
    pub yaw: I16F16,
}
