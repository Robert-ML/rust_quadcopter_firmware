use crate::yaw_pitch_roll::YawPitchRoll;
use fixed::types::I16F16;
use tudelft_quadrupel::block;
use tudelft_quadrupel::mpu::structs::Accel;
use tudelft_quadrupel::mpu::structs::Gyro;
use tudelft_quadrupel::mpu::{read_dmp_bytes, read_raw};
type FP = I16F16;
pub struct CalibrationData {
    pub roll_offset: FP,
    pub yaw_offset: FP,
    pub pitch_offset: FP,
    pub gyro_x_offset: i16,
    pub gyro_y_offset: i16,
    pub gyro_z_offset: i16,
    pub accel_x_offset: i16,
    pub accel_y_offset: i16,
    pub accel_z_offset: i16,
    pub is_calibrated: bool,
}

impl CalibrationData {
    pub fn new() -> Self {
        CalibrationData {
            roll_offset: Default::default(),
            yaw_offset: Default::default(),
            pitch_offset: Default::default(),
            gyro_x_offset: 0,
            gyro_y_offset: 0,
            gyro_z_offset: 0,
            accel_x_offset: 0,
            accel_y_offset: 0,
            accel_z_offset: 0,
            is_calibrated: false,
        }
    }

    pub fn is_calibrated(&self) -> bool {
        return self.is_calibrated;
    }

    pub fn calibrate(&mut self, sample_size: i16) {
        self.is_calibrated = false;
        self.pitch_offset = FP::from_num(0);
        self.roll_offset = FP::from_num(0);
        self.yaw_offset = FP::from_num(0);
        let mut accel_data: [i32; 3] = [0; 3];
        let mut gyro_data: [i32; 3] = [0; 3];

        for i in 1..sample_size {
            let quaternion = block!(read_dmp_bytes()).unwrap();

            // Y/P/R
            let ypr = YawPitchRoll::from(quaternion);
            self.pitch_offset += FP::from_num(ypr.pitch);
            self.roll_offset += FP::from_num(ypr.roll);
            self.yaw_offset += FP::from_num(ypr.yaw);

            // Acceleration, Gyro
            let (accel, gyro) = Self::get_accel_gyro_raw();
            accel_data[0] += accel.x as i32;
            accel_data[1] += accel.y as i32;
            accel_data[2] += accel.z as i32;
            gyro_data[0] += gyro.x as i32;
            gyro_data[1] += gyro.y as i32;
            gyro_data[2] += gyro.z as i32;
        }

        // here we save the offset
        self.pitch_offset = self.pitch_offset / FP::from_num(sample_size);
        self.roll_offset = self.roll_offset / FP::from_num(sample_size);
        self.yaw_offset = self.yaw_offset / FP::from_num(sample_size);

        self.accel_x_offset = (accel_data[0] / sample_size as i32) as i16;
        self.accel_y_offset = (accel_data[1] / sample_size as i32) as i16;
        self.accel_z_offset = (accel_data[2] / sample_size as i32) as i16;

        self.gyro_x_offset = (gyro_data[0] / sample_size as i32) as i16;
        self.gyro_y_offset = (gyro_data[1] / sample_size as i32) as i16;
        self.gyro_z_offset = (gyro_data[2] / sample_size as i32) as i16;

        self.is_calibrated = true;
    }

    pub fn get_accel_gyro_raw() -> (Accel, Gyro) {
        let (accel, gyro) = read_raw().unwrap();
        return (accel, gyro);
    }
}
