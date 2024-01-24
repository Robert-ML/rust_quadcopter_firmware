use std::sync::{Arc, Mutex};
// Rust libraries
use std::cmp;
pub mod joystick;
pub mod keyboard;

use common::protocol::DataT;
use common::DroneMode;
// Other crates
use joystick::INPUT_STATE_JS;
use keyboard::INPUT_STATE_KB;

pub struct InputState {
    mode_request: Arc<Mutex<Option<DroneMode>>>,
    throttle: Arc<Mutex<i32>>,
    roll: Arc<Mutex<i32>>,
    pitch: Arc<Mutex<i32>>,
    roll_trim: Arc<Mutex<i32>>,
    pitch_trim: Arc<Mutex<i32>>,
    throttle_trim: Arc<Mutex<i32>>,
    yaw: Arc<Mutex<i32>>,
    yaw_trim: Arc<Mutex<i32>>,
    yaw_p: Arc<Mutex<i32>>,
    roll_pitch_p1: Arc<Mutex<i32>>,
    roll_pitch_p2: Arc<Mutex<i32>>,
    pub(crate) data_logging_action: Arc<Mutex<DataT>>,
    pub(crate) data_logging_state: Arc<Mutex<bool>>,
    pub(crate) is_new_mode_request_received: Arc<Mutex<bool>>,
    pub(crate) is_pid_updated: Arc<Mutex<bool>>,
    pub(crate) is_full_pid_updated: Arc<Mutex<bool>>,
}

impl InputState {
    pub fn set_throttle(&self, value: i32) {
        *(self.throttle.lock().unwrap()) = value;
    }

    pub fn get_throttle(&self) -> i32 {
        *self.throttle.lock().unwrap() + *self.throttle_trim.lock().unwrap()
    }

    pub fn set_roll(&self, value: i32) {
        *(self.roll.lock().unwrap()) = value;
    }

    pub fn get_roll(&self) -> i32 {
        *self.roll.lock().unwrap() + *self.roll_trim.lock().unwrap()
    }

    pub fn get_mode_pressed(&self) -> Option<DroneMode> {
        let val: Option<DroneMode> = *(self.mode_request.lock().unwrap());
        *(self.mode_request.lock().unwrap()) = None;
        return val;
    }

    pub fn set_mode_pressed(&self, mode: DroneMode) {
        *(self.mode_request.lock().unwrap()) = Some(mode)
    }

    pub fn set_pitch(&self, value: i32) {
        *(self.pitch.lock().unwrap()) = value;
    }

    pub fn get_pitch(&self) -> i32 {
        *self.pitch.lock().unwrap() + *self.pitch_trim.lock().unwrap()
    }

    pub fn set_yaw(&self, value: i32) {
        *(self.yaw.lock().unwrap()) = value;
    }

    pub fn get_yaw(&self) -> i32 {
        *self.yaw.lock().unwrap() + *self.yaw_trim.lock().unwrap()
    }

    pub fn get_yaw_p(&self) -> i32 {
        *self.yaw_p.lock().unwrap()
    }

    pub fn get_full_control_p1(&self) -> i32 {
        *self.roll_pitch_p1.lock().unwrap()
    }

    pub fn get_full_control_p2(&self) -> i32 {
        *self.roll_pitch_p2.lock().unwrap()
    }
}

// Constants
const MAX_VALUE: i32 = 2047; // as a percentage

// TODO: looks like some of this code is repeating, just with other variables.
// Can it be extracted to another funciton to be more modular?
pub fn get_throttle() -> u16 {
    let js = INPUT_STATE_JS.get_throttle();
    let kb = INPUT_STATE_KB.get_throttle();
    clip_to_valid_range(js + kb)
}

pub fn get_pitch() -> u16 {
    let js = INPUT_STATE_JS.get_pitch();
    let kb = INPUT_STATE_KB.get_pitch();
    clip_to_valid_range(js + kb)
}

pub fn get_roll() -> u16 {
    let js = INPUT_STATE_JS.get_roll();
    let kb = INPUT_STATE_KB.get_roll();
    clip_to_valid_range(js + kb)
}

pub fn get_yaw() -> u16 {
    let js = INPUT_STATE_JS.get_yaw();
    let kb = INPUT_STATE_KB.get_yaw();
    clip_to_valid_range(js + kb)
}

pub fn get_yaw_p() -> i32 {
    INPUT_STATE_KB.get_yaw_p()
}

pub fn get_full_control_p1() -> i32 {
    INPUT_STATE_KB.get_full_control_p1()
}

pub fn get_full_control_p2() -> i32 {
    INPUT_STATE_KB.get_full_control_p2()
}

// Clips any number to the range [0, 2047]
pub fn clip_to_valid_range(x: i32) -> u16 {
    return cmp::min(MAX_VALUE, cmp::max(0 as i32, x)) as u16;
}

pub fn get_mode_pressed() -> Option<DroneMode> {
    // The joystick can only go to panic mode, so we prioritize its output
    let js: Option<DroneMode> = INPUT_STATE_JS.get_mode_pressed();
    let kb: Option<DroneMode> = INPUT_STATE_KB.get_mode_pressed();
    if let Some(mode) = js {
        return Some(mode);
    }
    return kb;
}
