// Rust libraries

use core::time::Duration;
// TUDelft library
use tudelft_quadrupel::led::Led::{Green, Red, Yellow};
// Our libraries
use common::protocol::DataT::*;
use common::protocol::{ControlDT, DataT, WarningDT};
use common::DroneMode;

// This crate imports
use crate::drone::state::DroneState;

// This module imports
use super::ModeTrait;

// Constants

#[derive(Debug)]
pub struct SafeMode;

impl ModeTrait for SafeMode {
    fn operate(state: &mut DroneState, iter_count: u32, delta_t: Duration) -> DroneMode {
        // Debug LEDs
        Green.on();
        Yellow.off();
        Red.off();

        let mut next_mode: DroneMode = DroneMode::Safe;
        if !Self::is_battery_low(state) {
            next_mode = Self::check_for_input(state, iter_count);
            // motor control
            Self::do_motor_control(state, delta_t);

            // do periodic stuff if we do not change the mode
            if next_mode == Self::get_mode() {
                Self::do_periodic(state, iter_count);
            }
        }

        next_mode
    }

    fn check_for_input(state: &mut DroneState, _iter_count: u32) -> DroneMode {
        // Maybe not allocate a new object
        // Initialize the state to return to, default is we don't change states
        let mut ret: DroneMode = Self::get_mode();
        let mut exit: bool = false;

        // Must read from pipe to not allow it to be filled!
        while (exit == false) && (ret == Self::get_mode()) {
            // Handle packet/payload/message
            match state.read_data() {
                Control(control) => {
                    // save control command
                    state.set_cc(control);
                }

                Mode(mode) => {
                    // Check if the last control package received was {thrust = 0 & p/r/y = 1024}
                    if mode == DroneMode::Panic {
                        ret = DroneMode::Panic
                    } else if !is_control_neutral(state.get_cc()) {
                        // warn PC we can not change mode because controls are not neutral
                        state.send_data(DataT::Warning(WarningDT::ControlNotNeutral));

                        return ret;
                    } else if mode == DroneMode::Manual {
                        // Transition to Manual Mode
                        ret = DroneMode::Manual;
                    } else if mode == DroneMode::Calibrate {
                        // Transition to Manual Mode
                        ret = DroneMode::Calibrate;
                    } else if mode == DroneMode::YawControl {
                        if !state.calibrated_data.is_calibrated() {
                            state.send_data(Warning(WarningDT::SensorNotCalibrated));
                            ret = DroneMode::Safe;
                        } else {
                            // Transition to Yaw control Mode
                            ret = DroneMode::YawControl;
                        }
                    } else if mode == DroneMode::FullControl {
                        if !state.calibrated_data.is_calibrated() {
                            state.send_data(Warning(WarningDT::SensorNotCalibrated));
                            ret = DroneMode::Safe;
                        } else {
                            // Transition to Yaw control Mode
                            ret = DroneMode::FullControl;
                        }
                    } else if mode == DroneMode::RawMode {
                        if !state.calibrated_data.is_calibrated() {
                            state.send_data(Warning(WarningDT::SensorNotCalibrated));
                            ret = DroneMode::Safe;
                        } else {
                            // Transition to Yaw control Mode
                            ret = DroneMode::RawMode;
                        }
                    }
                }

                KeepAlive => {
                    state.got_keep_alive();
                }

                StartLogging => {
                    state.start_logging();
                }
                StopLogging => {
                    state.stop_logging();
                }
                StartLogReporting => {
                    state.log_report_start();
                }
                StopLogReporting => {
                    state.log_report_stop();
                }

                Empty => {
                    exit = true;
                }

                // Ignore other messages
                _ => {}
            }
        }

        ret
    }

    fn do_motor_control(state: &mut DroneState, delta_t: Duration) {
        // TODO
        state.set_motors([0, 0, 0, 0]);
    }

    fn get_mode() -> DroneMode {
        DroneMode::Safe
    }
}

fn is_control_neutral(ctrl: ControlDT) -> bool {
    let dead_margin: u16 = 50;
    let upper = 1024 + dead_margin;
    let lower = 1024 - dead_margin;

    if ctrl.lift > 0 {
        return false;
    }
    if ctrl.roll > upper || ctrl.roll < lower {
        return false;
    }
    if ctrl.pitch > upper || ctrl.pitch < lower {
        return false;
    }
    if ctrl.yaw > upper || ctrl.yaw < lower {
        return false;
    }
    return true;
}
