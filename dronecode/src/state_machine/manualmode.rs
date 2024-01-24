// Rust libraries
use core::time::Duration;

// TUDelft library
use tudelft_quadrupel::led::Led::{Green, Red, Yellow};

// Our libraries
use common::motor_control::motor_mapping;
use common::protocol::DataT::*;
use common::DroneMode;

// This crate imports
use crate::drone::state::DroneState;

// This module imports
use super::ModeTrait;

#[derive(Debug)]
pub struct ManualMode;

impl ModeTrait for ManualMode {
    fn operate(state: &mut DroneState, iter_count: u32, delta_t: Duration) -> DroneMode {
        // Debug LEDs
        Green.on();
        Yellow.on();
        Red.off();

        let next_mode: DroneMode;

        if Self::is_battery_low(state) {
            next_mode = DroneMode::Panic;
        } else {
            next_mode = Self::check_for_input(state, iter_count);
            // motor control
            Self::do_motor_control(state, delta_t);
        }

        // do periodic stuff if we do not change the mode
        if next_mode == Self::get_mode() {
            Self::do_periodic(state, iter_count);
        }

        next_mode
    }

    fn check_for_input(state: &mut DroneState, _iter_count: u32) -> DroneMode {
        // Maybe not allocate a new object
        // Initialize the state to return to, default is we don't change states
        let ret: DroneMode = Self::get_mode();
        let mut exit: bool = false;

        // Must read from pipe to not allow it to be filled!
        while (exit == false) && (ret == Self::get_mode()) {
            // Handle packet/payload/message
            match state.read_data() {
                Control(control) => {
                    state.set_cc(control);
                }

                Mode(mode) => {
                    if (mode == DroneMode::Safe) || (mode == DroneMode::Panic) {
                        return DroneMode::Panic;
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
            };
        }

        ret
    }

    fn do_motor_control(state: &mut DroneState, delta_t: Duration) {
        // TODO: handle errors
        let motor_comman: [u16; 4] = motor_mapping(state.get_cc_as_vec()).unwrap();

        state.set_motors(motor_comman);
    }

    fn get_mode() -> DroneMode {
        DroneMode::Manual
    }
}
