use fixed::types::I16F16;

use common::{
    io::*,
    protocol::{ControlDT, DataT, UpdateP1P2DT, UpdatePDT, WarningDT},
    DroneMode,
};

use crate::gui::GuiParams;
use crate::input::joystick::INPUT_STATE_JS;
use crate::input::keyboard::INPUT_STATE_KB;
use crate::{
    input::{self, get_pitch, get_roll, get_throttle, get_yaw},
    utils::constants::{RUNNER_PERIOD_COMMAND, RUNNER_PERIOD_KEEP_ALIVE},
};

const BUF_CAP: usize = 256;

pub struct Logic {
    pipe: ComT<BUF_CAP>,

    mode: DroneMode,
}

impl Logic {
    pub fn default() -> Self {
        Self {
            pipe: ComT::<BUF_CAP>::new(
                crate::serial_wrapper::receive_bytes,
                crate::serial_wrapper::send_bytes,
            ),
            mode: DroneMode::Safe,
        }
    }

    pub fn tick(
        &mut self,
        iter_count: u32,
        delta: &std::time::Duration,
        gui_params_modifier_3: GuiParams,
    ) {
        // for now only simple functionality to send over control commands,
        // mode changes and keep alive messages
        self.perform_periodic_tasks(iter_count, delta);

        self.check_drone_coms(gui_params_modifier_3);
    }

    fn perform_periodic_tasks(&mut self, iter_count: u32, _delta: &std::time::Duration) {
        if iter_count % RUNNER_PERIOD_COMMAND == 0 {
            // TODO: do error handling

            let control_data = DataT::Control(ControlDT {
                lift: get_throttle(),
                roll: get_roll(),
                pitch: get_pitch(),
                yaw: get_yaw(),
            });
            // log::debug!("Sending control package: {:#?}", control_data);
            self.pipe.send_data::<BUF_CAP>(control_data).unwrap();
        }

        if iter_count % RUNNER_PERIOD_KEEP_ALIVE == 0 {
            // TODO: do error handling
            match self.pipe.send_data::<BUF_CAP>(DataT::KeepAlive) {
                Ok(_) => {}
                Err(e) => log::debug!("[ERROR]: failed to send ACK with code: {:#?}", e),
            }
        }

        if (*INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap())
            || (*INPUT_STATE_JS.is_new_mode_request_received.lock().unwrap())
        {
            // perform mode change
            if let Some(req_mode) = input::get_mode_pressed() {
                // TODO: do error handling
                log::debug!("Switch mode request: {:?}", req_mode);

                match self.pipe.send_data::<BUF_CAP>(DataT::Mode(req_mode)) {
                    Ok(_) => {}
                    Err(e) => {
                        log::error!("[ERROR]: sending mode change {:#?}", e);
                    }
                }
            }
            *INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap() = false;
        }

        if *INPUT_STATE_KB.is_pid_updated.lock().unwrap() {
            // adjust P values if different from 0
            let yaw_p_trim = input::get_yaw_p();
            if yaw_p_trim != 0 {
                match self.pipe.send_data::<BUF_CAP>(DataT::UpdateP(UpdatePDT {
                    p: I16F16::from_num(yaw_p_trim),
                })) {
                    Ok(_) => {}
                    Err(e) => {
                        log::error!("[ERROR]: sending yaw p trim {:#?}", e);
                    }
                }
            }
            *INPUT_STATE_KB.is_pid_updated.lock().unwrap() = false;
        }

        if *INPUT_STATE_KB.data_logging_state.lock().unwrap() {
            let logging_state: DataT = INPUT_STATE_KB.data_logging_action.lock().unwrap().clone();

            match self.pipe.send_data::<BUF_CAP>(logging_state) {
                Ok(_) => {}
                Err(e) => {
                    log::error!("[ERROR]: sending Logging state {:#?}", e);
                }
            }

            *INPUT_STATE_KB.data_logging_state.lock().unwrap() = false;
        }

        if *INPUT_STATE_KB.is_full_pid_updated.lock().unwrap() {
            let p1 = input::get_full_control_p1();
            let p2 = input::get_full_control_p2();

            match self
                .pipe
                .send_data::<BUF_CAP>(DataT::UpdateP1P2(UpdateP1P2DT {
                    p1: I16F16::from_num(p1),
                    p2: I16F16::from_num(p2),
                })) {
                Ok(_) => {}
                Err(e) => {
                    log::error!("[ERROR]: sending full p trim {:#?}", e);
                }
            }
            *INPUT_STATE_KB.is_full_pid_updated.lock().unwrap() = false;
        }
    }

    fn check_drone_coms(&mut self, gui_params_modifier_3: GuiParams) {
        match self.pipe.read_data::<BUF_CAP>() {
            Ok(data) => self.handle_message(data, gui_params_modifier_3),

            Err(e) => self.handle_read_error(e),
        }
    }

    fn handle_message(&mut self, data: DataT, gui_params_modifier_3: GuiParams) {
        match data {
            DataT::Mode(mode) => {
                log::info!("Drone mode is now: {:#?}", mode);
                self.mode = mode;
                match mode {
                    DroneMode::Safe => {
                        *gui_params_modifier_3.drone_mode.lock().unwrap() = "Safe".to_string();
                    }
                    DroneMode::Manual => {
                        *gui_params_modifier_3.drone_mode.lock().unwrap() = "Manual".to_string();
                    }
                    DroneMode::Panic => {
                        *gui_params_modifier_3.drone_mode.lock().unwrap() = "Panic".to_string();
                    }
                    DroneMode::Calibrate => {
                        *gui_params_modifier_3.drone_mode.lock().unwrap() = "Calibrate".to_string();
                    }
                    DroneMode::YawControl => {
                        *gui_params_modifier_3.drone_mode.lock().unwrap() =
                            "YawControl".to_string();
                    }
                    DroneMode::FullControl => {
                        *gui_params_modifier_3.drone_mode.lock().unwrap() =
                            "FullControl".to_string();
                    }
                    DroneMode::RawMode => {
                        *gui_params_modifier_3.drone_mode.lock().unwrap() = "RawMode".to_string();
                    }
                }
            }

            DataT::Warning(warn) => {
                log::warn!("Drone gave warning: {:#?}", warn);
                match warn {
                    WarningDT::ControlNotNeutral => {
                        *gui_params_modifier_3.last_message_received.lock().unwrap() =
                            "Warning:Control Not Neutral ".to_string();
                    }
                    WarningDT::SensorNotCalibrated => {
                        *gui_params_modifier_3.last_message_received.lock().unwrap() =
                            "Warning:gave warning:Sensor Not Calibrated ".to_string();
                    }
                }
            }

            DataT::Message(s) => {
                log::info!("Drone sent: \"{}\"", s);
                *gui_params_modifier_3
                    .debug_prints_from_drone
                    .lock()
                    .unwrap() = "Drone Sent:".to_string() + &*s.to_string();
            }

            DataT::KeepAlive => {
                // TODO: keep track of keep alive on the PC
            }

            DataT::CalibratedAck(calibrated_values) => {
                log::info!(
                    "Drone sent Calibrated Values: {} , {}, {}, {}, {} , {} ,",
                    calibrated_values.gyro_pitch_offset,
                    calibrated_values.gyro_roll_offset,
                    calibrated_values.gyro_yaw_offset,
                    calibrated_values.accel_x_offset,
                    calibrated_values.accel_y_offset,
                    calibrated_values.accel_z_offset
                )
            }

            DataT::SonsorNotCalibrated => {
                log::info!("Drone sent sensor not calibrated message")
            }

            DataT::SensorReading(calibrated_values) => {
                log::info!(
                    "Drone sent Calibrated Values: {} , {}, {}, {}, {} , {} ,",
                    calibrated_values.gyro_roll,
                    calibrated_values.gyro_yaw,
                    calibrated_values.gyro_pitch,
                    calibrated_values.accel_x,
                    calibrated_values.accel_y,
                    calibrated_values.accel_z
                )
            }

            DataT::HealthData(healthDT) => {
                log::info!(
                    "Drone sent health data -> battery: {}, cpu: {}, pressure: {}",
                    healthDT.bat,
                    healthDT.cpu,
                    healthDT.pres,
                );
                *gui_params_modifier_3.battery_health.lock().unwrap() = healthDT.bat;
            }

            DataT::MovementErrors(offsets) => {
                log::info!(
                    "Drone sent PID error Values: {} , {}, {}",
                    offsets.pitch_error,
                    offsets.roll_error,
                    offsets.yaw_error,
                )
            }

            DataT::MotorsState(motors) => {
                // log::info!(
                //     "Drone sent motor values: {} , {}, {} , {}",
                //     motors.ae1,
                //     motors.ae2,
                //     motors.ae3,
                //     motors.ae4,
                // );
                *gui_params_modifier_3.motor_1_value.lock().unwrap() = motors.ae1;
                *gui_params_modifier_3.motor_2_value.lock().unwrap() = motors.ae2;
                *gui_params_modifier_3.motor_3_value.lock().unwrap() = motors.ae3;
                *gui_params_modifier_3.motor_4_value.lock().unwrap() = motors.ae4;
            }

            DataT::UpdateP(updated_p_value) => {
                log::info!("Updated P : {}", updated_p_value.p,);

                *gui_params_modifier_3.yaw_p.lock().unwrap() = updated_p_value.p.to_num::<i32>();
            }
            DataT::UpdateP1P2(updated_p1_p2_values) => {
                log::info!(
                    "Updated P1: {}, P2: {}",
                    updated_p1_p2_values.p1,
                    updated_p1_p2_values.p2
                );
                *gui_params_modifier_3.rp_p1.lock().unwrap() =
                    updated_p1_p2_values.p1.to_num::<i32>();
                *gui_params_modifier_3.rp_p2.lock().unwrap() =
                    updated_p1_p2_values.p2.to_num::<i32>();
            }

            DataT::SensorLog(sensor_data) => {
                log::info!(
                    "gyrox: {} , gyroy: {} , gyroz: {}, accelx: {} , accely: {} , accelz: {} , roll: {}, pitch: {} , yaw: {}", sensor_data.gyro_x , sensor_data.gyro_y , sensor_data.gyro_z , sensor_data.accel_x, sensor_data.accel_y , sensor_data.accel_z , sensor_data.pitch , sensor_data.roll, sensor_data.yaw
                );
            }

            _ => {}
        }
    }

    // TODO
    fn handle_read_error(&mut self, _e: ComErr) {}
}
