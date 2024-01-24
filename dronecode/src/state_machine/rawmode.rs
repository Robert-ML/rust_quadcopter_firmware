use crate::drone::controller::{pitch_control_raw, roll_control_raw, yaw_control_raw};
use crate::drone::state::DroneState;
use crate::state_machine::ModeTrait;
use common::motor_control::motor_mapping;
use common::protocol::DataT::{Control, Empty, KeepAlive, Mode, UpdateP, UpdateP1P2};
use common::protocol::{DataT, UpdateP1P2DT, UpdatePDT};
use common::DroneMode;
use core::time::Duration;

pub struct RawMode {}

impl ModeTrait for RawMode {
    fn operate(state: &mut DroneState, iter_count: u32, delta_t: Duration) -> DroneMode {
        let next_mode: DroneMode;
        state.sensors_dmp.update_sensor_readings_dmp();
        state.debug_info = common::protocol::DataT::Message(heapless::String::from(
            alloc::format!(
                "dmp:{}, raw:{}",
                state.sensors_dmp.get_dmp_yaw_value(state)
                    - state.sensors_dmp.get_dmp_yaw_value_old(state),
                state.sensors_raw.get_yaw_value_der()
            )
            .as_str(),
        ));

        if Self::is_battery_low(state) {
            next_mode = DroneMode::Panic;
        } else {
            next_mode = Self::check_for_input(state, iter_count);
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

                UpdateP(updatedPidValues) => {
                    state.P = updatedPidValues.p;
                    state.send_data(DataT::UpdateP(UpdatePDT { p: state.P }));
                }
                UpdateP1P2(updatedFullControlPIDValues) => {
                    state.P1 = updatedFullControlPIDValues.p1;
                    state.P2 = updatedFullControlPIDValues.p2;
                    state.send_data(DataT::UpdateP1P2(UpdateP1P2DT {
                        p1: state.P1,
                        p2: state.P2,
                    }));
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
        // TODO: Everything
        // // TODO: handle errors

        let mut cc = state.get_cc_as_vec();
        let roll_command: i32 = cc[1] as i32 - 1024;
        let pitch_command: i32 = cc[2] as i32 - 1024;
        let yaw_command: i32 = cc[3] as i32 - 1024; // [-1024, 1024]
        cc[1] = roll_control_raw(roll_command, state, delta_t);
        cc[2] = pitch_control_raw(pitch_command, state, delta_t);
        cc[3] = yaw_control_raw(yaw_command, state, delta_t);

        // TODO: MODIFY IT SO THAT PID HAPPENS HERE ITSELF
        // state.debug_info = common::protocol::DataT::Message(heapless::String::from(
        //     alloc::format!("y{}", response).as_str(),
        // ));

        let mapped_motor_value = motor_mapping(cc);

        state.set_motors(mapped_motor_value.unwrap());
    }

    fn get_mode() -> DroneMode {
        DroneMode::RawMode
    }
}
