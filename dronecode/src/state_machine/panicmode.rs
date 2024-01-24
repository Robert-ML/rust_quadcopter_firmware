// Rust libraries
use core::cmp::max;
use core::time::Duration;

// TUDelft library
use tudelft_quadrupel::led::Led::{Green, Red, Yellow};

// Our libraries
use common::DroneMode;

// This crate imports
use crate::drone::state::DroneState;

// This module imports
use super::ModeTrait;

#[derive(Debug)]
pub struct PanicMode {}

impl ModeTrait for PanicMode {
    fn operate(state: &mut DroneState, iter_count: u32, delta_t: Duration) -> DroneMode {
        // Debug LEDs
        Green.off();
        Yellow.off();
        Red.on();

        Self::check_for_input(state, iter_count);

        Self::is_battery_low(state);

        // motor control
        Self::do_motor_control(state, delta_t);

        // transition to safe mode if motors stopped spinning
        if are_motors_zero(state.get_motors()) {
            DroneMode::Safe
        } else {
            // do periodic stuff if we do not change the mode
            Self::do_periodic(state, iter_count);

            DroneMode::Panic
        }
    }

    fn check_for_input(state: &mut DroneState, _iter_count: u32) -> DroneMode {
        let mut exit: bool = false;
        // Must read from pipe to not allow it to be filled!

        while exit != false {
            match state.read_data() {
                common::protocol::DataT::KeepAlive => {
                    state.got_keep_alive();
                }

                common::protocol::DataT::Empty => {
                    exit = true;
                }

                _ => {}
            };
        }

        DroneMode::Panic
    }

    fn do_motor_control(state: &mut DroneState, delta_t: Duration) {
        let motor_command: [u16; 4] = state.get_motors();

        let new_mc = decrease_motors(motor_command, state.config.panic_motor_reduction);

        state.set_motors(new_mc);
    }

    fn get_mode() -> DroneMode {
        DroneMode::Panic
    }
}

fn are_motors_zero(values: [u16; 4]) -> bool {
    for v in values {
        if v != 0 {
            return false;
        }
    }
    true
}

// TODO: make test cases because these casts between u16 and i32 and back to u16
fn decrease_motors(values: [u16; 4], reduction: u16) -> [u16; 4] {
    let mut new: [u16; 4] = values.clone();

    for v in &mut new {
        *v = max(0 as i32, *v as i32 - reduction as i32) as u16;
    }

    new
}
