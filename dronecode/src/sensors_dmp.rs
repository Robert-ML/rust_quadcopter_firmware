use crate::drone::state::DroneState;
use crate::yaw_pitch_roll::YawPitchRoll;
use fixed::types::I16F16;
use tudelft_quadrupel::block;
use tudelft_quadrupel::mpu::read_dmp_bytes;
type FP = I16F16;

pub struct SensorsDMP {
    gyro_x_new: i16,
    gyro_y_new: i16,
    gyro_z_new: i16,
    gyro_x_old: i16,
    gyro_y_old: i16,
    gyro_z_old: i16,
    accel_x_new: i16,
    accel_y_new: i16,
    accel_z_new: i16,
    accel_x_old: i16,
    accel_y_old: i16,
    accel_z_old: i16,
    sensor_new: YawPitchRoll,
    sensor_old: YawPitchRoll,
}

impl SensorsDMP {
    pub fn new() -> Self {
        SensorsDMP {
            gyro_x_new: 0,
            gyro_y_new: 0,
            gyro_z_new: 0,
            gyro_x_old: 0,
            gyro_y_old: 0,
            gyro_z_old: 0,
            accel_x_new: 0,
            accel_y_new: 0,
            accel_z_new: 0,
            accel_x_old: 0,
            accel_y_old: 0,
            accel_z_old: 0,
            sensor_new: YawPitchRoll {
                yaw: FP::from_num(0),
                pitch: FP::from_num(0),
                roll: FP::from_num(0),
            },
            sensor_old: YawPitchRoll {
                yaw: FP::from_num(0),
                pitch: FP::from_num(0),
                roll: FP::from_num(0),
            },
        }
    }

    // the values we get from the sensor are adjusted with the offset from calibration mode
    pub fn get_dmp_roll_value(&self, state: &DroneState) -> FP {
        let val = FP::from_num(self.sensor_new.roll) - state.calibrated_data.roll_offset;
        val
    }

    pub fn get_dmp_pitch_value(&self, state: &DroneState) -> FP {
        let val = FP::from_num(self.sensor_new.pitch) - state.calibrated_data.pitch_offset;
        val
    }

    pub fn get_dmp_yaw_value(&self, state: &DroneState) -> FP {
        let val = FP::from_num(self.sensor_new.yaw) - state.calibrated_data.yaw_offset;
        val
    }

    pub fn get_dmp_roll_value_old(&self, state: &DroneState) -> FP {
        let val = FP::from_num(self.sensor_old.roll) - state.calibrated_data.roll_offset;
        val
    }

    pub fn get_dmp_pitch_value_old(&self, state: &DroneState) -> FP {
        let val = FP::from_num(self.sensor_old.pitch) - state.calibrated_data.pitch_offset;
        val
    }

    pub fn get_dmp_yaw_value_old(&self, state: &DroneState) -> FP {
        let val = FP::from_num(self.sensor_old.yaw) - state.calibrated_data.yaw_offset;
        val
    }

    // TODO : Error handling
    pub fn update_sensor_readings_dmp(&mut self) {
        self.sensor_old = self.sensor_new;
        let quaternion = block!(read_dmp_bytes()).unwrap();
        self.sensor_new = YawPitchRoll::from(quaternion);
    }

    // TODO: Update on each tick and pick up then
    pub fn get_gyro_sensor(&self) -> YawPitchRoll {
        self.sensor_new
    }
}
