use crate::calibrationdata::CalibrationData;
use cordic::atan2;
use fixed::{traits::FromFixed, types::I32F32};
use fixed_sqrt::FixedSqrt;
use tudelft_quadrupel::mpu::{
    read_raw,
    structs::{Accel, Gyro},
};

type FP = fixed::types::I16F16;

const P2PHI: i32 = 94000;
const A2G: i32 = 16384;
const C1: i32 = 50;
const C2: i32 = 15000;

pub struct SensorsRaw {
    phi: FP,
    phi_b: FP,
    phi_der: FP,
    theta: FP,
    theta_b: FP,
    theta_der: FP,
    yaw_der: FP,
}

//p= sp - b;
//phi = phi + p * P2PHI;
//e = phi - sphi;
//phi = phi – e / C1;
//b = b + (e/P2PHI) / C2;

impl SensorsRaw {
    pub fn new() -> Self {
        Self {
            phi: FP::from_num(0),
            phi_b: FP::from_num(0),
            phi_der: FP::from_num(0),
            theta: FP::from_num(0),
            theta_b: FP::from_num(0),
            theta_der: FP::from_num(0),
            yaw_der: FP::from_num(0),
        }
    }

    pub fn read(&self, calibration_data: &CalibrationData) -> (Accel, Gyro) {
        let raw = read_raw().unwrap();
        let mut a = raw.0;
        a.x -= calibration_data.accel_x_offset;
        a.y -= calibration_data.accel_y_offset;
        a.z -= calibration_data.accel_z_offset;

        let mut g = raw.1;
        g.x -= calibration_data.gyro_x_offset;
        g.y -= calibration_data.gyro_y_offset;
        g.z -= calibration_data.gyro_z_offset;
        return (a, g);
    }

    pub fn update_sensor_readings_raw(&mut self, calibration_data: &CalibrationData) {
        let (ref mut accel, ref mut gyro) = self.read(calibration_data);

        self.update_sensor_roll(&gyro, &accel);
        self.update_sensor_pitch(&gyro, &accel);
        self.update_sensor_yaw(&gyro);
    }

    pub fn update_sensor_roll(&mut self, gyro: &Gyro, accel: &Accel) {
        let ax = FP::from_num(accel.x) / FP::from_num(A2G);
        let ay = FP::from_num(accel.y) / FP::from_num(A2G);
        let az = FP::from_num(accel.z) / FP::from_num(A2G);

        self.phi_der = -self.phi;

        let sp = FP::from_num(gyro.x);
        let p = sp - self.phi_b;
        self.phi = self.phi + (p / FP::from_num(P2PHI));
        let sphi = atan2(FP::from_num(ay), FP::from_num(az));
        let e = self.phi - sphi;
        self.phi = self.phi - (e / FP::from_num(C1));
        self.phi_b = self.phi_b + (e * FP::from_num(P2PHI)) / FP::from_num(C2);
        self.phi_der += self.phi;
    }

    pub fn update_sensor_pitch(&mut self, gyro: &Gyro, accel: &Accel) {
        let ax = FP::from_num(accel.x) / FP::from_num(A2G);
        let ay = FP::from_num(accel.y) / FP::from_num(A2G);
        let az = FP::from_num(accel.z) / FP::from_num(A2G);

        self.theta_der = -self.theta;

        let sq = FP::from_num(gyro.y);
        let q = sq - self.theta_b;
        self.theta = self.theta + (q / FP::from_num(P2PHI));
        let sqr: I32F32 = FixedSqrt::sqrt(I32F32::from_num(ay * ay + az * az));

        let product: FP = FP::from_fixed(sqr);
        let stheta = atan2(FP::from_num(ax), product);
        let e = self.theta - stheta;
        self.theta = self.theta - (e / FP::from_num(C1));
        self.theta_b = self.theta_b + (e * FP::from_num(P2PHI)) / FP::from_num(C2);
        self.theta_der += self.theta;
    }

    pub fn update_sensor_yaw(&mut self, gyro: &Gyro) {
        self.yaw_der = FP::from_num(gyro.z) / FP::from_num(P2PHI);
    }

    pub fn get_yaw_value_der(&self) -> FP {
        return self.yaw_der;
    }

    pub fn get_pitch_der(&self) -> FP {
        return self.theta_der;
    }

    pub fn get_pitch_value(&self) -> FP {
        return self.theta;
    }

    pub fn get_roll_value(&self) -> FP {
        return self.phi;
    }
    pub fn get_roll_der(&self) -> FP {
        return self.phi_der;
    }
}
