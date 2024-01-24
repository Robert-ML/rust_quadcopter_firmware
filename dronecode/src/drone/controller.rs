use core::{
    cmp::{max, min},
    time::Duration,
};

use fixed::traits::FromFixed;

use super::state::DroneState;

type FP = fixed::types::I16F16;

// To debug info print stuff:
// state.debug_info = common::protocol::DataT::Message(heapless::String::from(
//     alloc::format!("r{}", response).as_str(),
// ));

// Runs the basic Proportional rate control loop
pub fn yaw_control_dmp(yaw_command: i32, state: &mut DroneState, delta_t: Duration) -> u16 {
    let yaw_new = state.sensors_dmp.get_dmp_yaw_value(state) * FP::from_num(100);
    let yaw_old = state.sensors_dmp.get_dmp_yaw_value_old(state) * FP::from_num(100);
    let mut sensor_d_yaw = yaw_old - yaw_new;
    let dt = FP::from_num(delta_t.as_millis());
    let mut sensor_yaw_rate = sensor_d_yaw / dt;

    // TODO: Look up scaling for the joystick value
    let scaling_constant = FP::from_num(64);
    let error =
        (FP::from_num(yaw_command) / scaling_constant) - (sensor_yaw_rate * scaling_constant); // / FP::from_num(1000);

    let mut response = state.P * error * 5;

    response = max(response, FP::from_num(-1022));
    response = min(response, FP::from_num(1022));
    return (response + FP::from_num(1024)).to_num::<u16>();
}

pub fn roll_control_dmp(setp: i32, state: &mut DroneState, delta_t: Duration) -> u16 {
    return _pitch_roll_control_dmp(
        setp,
        state.sensors_dmp.get_dmp_roll_value_old(state),
        state.sensors_dmp.get_dmp_roll_value(state),
        state,
        delta_t,
    );
}

pub fn pitch_control_dmp(setp: i32, state: &mut DroneState, delta_t: Duration) -> u16 {
    return _pitch_roll_control_dmp(
        setp,
        state.sensors_dmp.get_dmp_pitch_value_old(state),
        state.sensors_dmp.get_dmp_pitch_value(state),
        state,
        delta_t,
    );
}

pub fn _pitch_roll_control_dmp(
    setp: i32,
    old: FP,
    new: FP,
    state: &mut DroneState,
    delta_t: Duration,
) -> u16 {
    let _old = old * FP::from_num(100);
    let _new = new * FP::from_num(100);
    let rate = (_old - _new) / FP::from_num(delta_t.as_millis());

    let scaling_constant = FP::from_num(30);
    let response = state.P1
        * ((FP::from_num(setp) / scaling_constant) - (_new * FP::from_num(0.75)))
        + state.P2 * rate;
    // let response = state.P1 * ((FP::from_num(setp) / scaling_constant) - (_new)) - state.P2 * rate;

    clip_and_scale_response(response)
}

pub fn yaw_control_raw(yaw_command: i32, state: &mut DroneState, delta_t: Duration) -> u16 {
    let mut sensor_d_yaw =
        FP::from_fixed(state.sensors_raw.get_yaw_value_der()) * FP::from_num(100);
    let dt = FP::from_num(delta_t.as_millis());
    let mut sensor_yaw_rate = sensor_d_yaw / dt;

    // TODO: Look up scaling for the joystick value
    let scaling_constant = FP::from_num(64);
    let error =
        (FP::from_num(yaw_command) / scaling_constant) - (sensor_yaw_rate * scaling_constant); // / FP::from_num(1000);

    let mut response = state.P * error * 5;
    // state.debug_info = common::protocol::DataT::Message(heapless::String::from(
    //     alloc::format!("r{}", response).as_str(),
    // ));

    response = max(response, FP::from_num(-1022));
    response = min(response, FP::from_num(1022));
    return (response + FP::from_num(1024)).to_num::<u16>();
}

pub fn roll_control_raw(setp: i32, state: &mut DroneState, delta_t: Duration) -> u16 {
    return _pitch_roll_control_raw(
        setp,
        FP::from_fixed(state.sensors_raw.get_roll_der()),
        FP::from_fixed(state.sensors_raw.get_roll_value()),
        state,
        delta_t,
    );
}

pub fn pitch_control_raw(setp: i32, state: &mut DroneState, delta_t: Duration) -> u16 {
    return _pitch_roll_control_raw(
        setp,
        FP::from_fixed(state.sensors_raw.get_pitch_der()),
        FP::from_fixed(state.sensors_raw.get_pitch_value()),
        state,
        delta_t,
    );
}

pub fn _pitch_roll_control_raw(
    setp: i32,
    der: FP,
    new: FP,
    state: &mut DroneState,
    delta_t: Duration,
) -> u16 {
    let _new = new * FP::from_num(100);
    let rate = der / FP::from_num(delta_t.as_millis());

    let scaling_constant = FP::from_num(30);
    //let response = state.P1 * ((FP::from_num(setp)/scaling_constant) - (_new*scaling_constant)) - state.P2 * rate;
    let response = state.P1
        * ((FP::from_num(setp) / scaling_constant) - (_new * FP::from_num(0.75)))
        - state.P2 * rate;

    clip_and_scale_response(response)
}

fn clip_and_scale_response(response: FP) -> u16 {
    let _r = min(max(response, FP::from_num(-1022)), FP::from_num(1022));
    return (_r + FP::from_num(1024)).to_num::<u16>();
}

// pub fn kalman_filter() {
//     //p= sp - b;
//     //phi = phi + p * P2PHI;
//     //e = phi - sphi;
//     //phi = phi – e / C1;
//     //b = b + (e/P2PHI) / C2;
// }
