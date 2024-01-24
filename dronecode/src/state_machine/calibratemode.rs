// Rust libraries
use core::time::Duration;

// TUDelft library
use tudelft_quadrupel::led::Led::{Green, Red, Yellow};

// Our libraries
use common::protocol::DataT::{Empty, KeepAlive, Mode};
use common::DroneMode;

use crate::drone::state::DroneState;

use super::ModeTrait;

const SAMPLE_SIZE: i16 = 200; // DO NOT INCREASE ABOVE 3. NEED TO DISCUSS IF 100 Values NEEDED

#[derive(Clone, Copy)]
pub struct CalibrateMode {}

impl CalibrateMode {
    pub fn new() -> Self {
        CalibrateMode {}
    }
}

impl ModeTrait for CalibrateMode {
    fn operate(state: &mut DroneState, iter_count: u32, delta_t: Duration) -> DroneMode {
        // Debug LEDs
        Green.off();
        Yellow.on();
        Red.off();

        if Self::is_battery_low(state) {
            return DroneMode::Panic;
        } else {
            // state.sensors.calibrate(SAMPLE_SIZE);
            state.calibrated_data.calibrate(SAMPLE_SIZE);
        }

        DroneMode::Panic
    }

    fn check_for_input(state: &mut DroneState, iter_count: u32) -> DroneMode {
        let mut ret: DroneMode = Self::get_mode();
        let mut exit: bool = false;

        // Must read from pipe to not allow it to be filled!
        while (exit == false) && (ret == Self::get_mode()) {
            // Handle packet/payload/message
            match state.read_data() {
                Mode(mode) => {
                    if mode == DroneMode::Safe {
                        // Transition to Manual Mode
                        ret = DroneMode::Panic;
                    } else if mode == DroneMode::Safe {
                        // Transition to Manual Mode
                        ret = DroneMode::Safe;
                    }
                    // TODO: decide if to handle wrong mode transitions or quietly ignore them?
                }

                KeepAlive => {
                    state.got_keep_alive();
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
        state.set_motors([0, 0, 0, 0]);
    }

    fn get_mode() -> DroneMode {
        DroneMode::Calibrate
    }
}
