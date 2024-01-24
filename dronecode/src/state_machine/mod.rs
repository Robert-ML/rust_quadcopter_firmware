pub(crate) mod calibratemode;
pub(crate) mod fullcontrolmode;
pub(crate) mod manualmode;
pub(crate) mod panicmode;
pub(crate) mod rawmode;
pub(crate) mod safemode;
pub(crate) mod yawcontrolmode;

// Rust libraries
use common::protocol::{HealthDT, MotorsDT};
use core::time::Duration;

// TUDelft library
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::led::Red;

// Our libraries
use common::DroneMode;

use crate::drone::state::DroneState;

/*
 * LED colors for each MODE:
 * - safe: green
 * - panic: red
 * - calibrate: yellow
 * - manual: green yellow
 * - yaw control: green red
 * - full control: yellow red
 */

/// ModeTrait describes what modes should implement so that the drone state can
/// perform actions depending on the mode.
pub trait ModeTrait {
    /// Runs the operations the drone should do at this tick in time in this
    /// mode. Operations like: reading from the serial and taking actions in
    /// response to what messages are received, do the motor control, do
    /// periodic actions if the mode will not be changed.
    ///
    /// @return - the new `DroneMode` the drone should transition to
    ///
    /// Note: If the drone transitions to a new mode, it is expected to call
    /// again `operate` on the new mode to take the new actions and continue
    /// reading messages from the pipe to empty it.
    fn operate(state: &mut DroneState, iter_count: u32, delta_t: Duration) -> DroneMode;

    /// Consumes as much as possible from the ring buffer but if mode changes,
    /// returns prematurly.
    fn check_for_input(state: &mut DroneState, iter_count: u32) -> DroneMode;

    /// The logic of the mode for motor control. If motors should not spin in
    /// a certain mode, then give a [0; 4] motor comand.
    fn do_motor_control(state: &mut DroneState, delta_t: Duration);

    // Get the `DroneMode` that this structure is implementing.
    fn get_mode() -> DroneMode;

    // Common functions
    fn is_battery_low(state: &mut DroneState) -> bool {
        let mut battery_value: u16 = 2000;

        if state.config.check_battery {
            battery_value = read_battery();
        }

        if battery_value < 1050 {
            // Send Battery Low Message Here
            return true;
        }
        Red.off();
        false
    }

    /// A function called by every mode in order to perform periodic actions.
    fn do_periodic(state: &mut DroneState, iter_count: u32) {
        // send ACK periodically
        if iter_count % state.config.ka_tick_period == 0 {
            state.send_alive();
        }

        if state.config.check_battery {
            if iter_count % state.config.battery_printing_time == 0 {
                state.send_data(common::protocol::DataT::HealthData(HealthDT {
                    bat: read_battery(),
                    cpu: 0,
                    pres: 0,
                }));
            }
        }

        if iter_count % 10 == 0 {
            let d = state.debug_info.clone();
            &state.send_data(d);
        }

        //#[cfg(debug_assertions)]
        {
            if iter_count % state.config.debug_motor_command_period == 0 {
                let mc: [u16; 4] = state.get_motors();

                state.send_data(common::protocol::DataT::MotorsState(MotorsDT {
                    ae1: mc[0],
                    ae2: mc[1],
                    ae3: mc[2],
                    ae4: mc[3],
                }));
            }

            // if iter_count % 9100 == 0 {
            //     // Print sensor values
            //     state.send_data(common::protocol::DataT::Message(heapless::String::from(
            //         alloc::format!("E r {:#?}", state.sensors.get_gyro_pitch_value()).as_str(),
            //     )));
            // }
            // debug periodic message
            if iter_count % state.config.debug_info_period == 0 {
                let d = state.debug_info.clone();
                state.send_data(d);
            }
        }
    }
}
