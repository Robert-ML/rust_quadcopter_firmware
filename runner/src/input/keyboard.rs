// Rust libraries
use std::io::{stdin, stdout, Write};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

// Other crates
use lazy_static::lazy_static;
use termion::event::Key;
use termion::input::TermRead;
use termion::raw::IntoRawMode;

// Our libraries
use crate::gui::GuiParams;
use common::protocol::DataT;
use common::DroneMode;

// This crate imports
use crate::input::InputState;

lazy_static! {
    pub static ref INPUT_STATE_KB: InputState = InputState {
        mode_request: Arc::new(Mutex::new(None)),
        throttle: Arc::new(Mutex::new(0)),
        roll: Arc::new(Mutex::new(0)),
        pitch: Arc::new(Mutex::new(0)),
        yaw: Arc::new(Mutex::new(0)),
        yaw_p: Arc::new(Mutex::new(25)),
        roll_pitch_p1: Arc::new(Mutex::new(6)),
        roll_pitch_p2: Arc::new(Mutex::new(57)),
        is_new_mode_request_received: Arc::new(Mutex::new(false)),
        is_pid_updated: Arc::new(Mutex::new(false)),
        is_full_pid_updated: Arc::new(Mutex::new(false)),
        roll_trim: Arc::new(Mutex::new(0)),
        pitch_trim: Arc::new(Mutex::new(0)),
        throttle_trim: Arc::new(Mutex::new(0)),
        yaw_trim: Arc::new(Mutex::new(0)),
        data_logging_state: Arc::new(Mutex::new(false)),
        data_logging_action: Arc::new(Mutex::new(DataT::StopLogReporting)),
    };
}

pub fn set_mode_pressed(mode: DroneMode) {
    *INPUT_STATE_KB.mode_request.lock().unwrap() = Some(mode)
}

pub fn increment_lift_trim() {
    *INPUT_STATE_KB.throttle_trim.lock().unwrap() += 1;
}

pub fn decrement_lift_trim() {
    *INPUT_STATE_KB.throttle_trim.lock().unwrap() -= 1;
}

pub fn increment_roll_trim() {
    *INPUT_STATE_KB.roll_trim.lock().unwrap() += 1;
}

pub fn decrement_roll_trim() {
    *INPUT_STATE_KB.roll_trim.lock().unwrap() -= 1;
}

pub fn increment_pitch_trim() {
    *INPUT_STATE_KB.pitch_trim.lock().unwrap() += 1;
}

pub fn decrement_pitch_trim() {
    *INPUT_STATE_KB.pitch_trim.lock().unwrap() -= 1;
}

pub fn increment_yaw_trim() {
    *INPUT_STATE_KB.yaw_trim.lock().unwrap() += 1;
}

pub fn decrement_yaw_trim() {
    *INPUT_STATE_KB.yaw_trim.lock().unwrap() -= 1;
}

pub fn increment_yaw_p() {
    *INPUT_STATE_KB.yaw_p.lock().unwrap() += 1;
}

pub fn decrement_yaw_p() {
    *INPUT_STATE_KB.yaw_p.lock().unwrap() -= 1;
}
pub fn increment_rollpitch_p1() {
    *INPUT_STATE_KB.roll_pitch_p1.lock().unwrap() += 1;
}

pub fn decrement_rollpitch_p1() {
    *INPUT_STATE_KB.roll_pitch_p1.lock().unwrap() -= 1;
}
pub fn increment_rollpitch_p2() {
    *INPUT_STATE_KB.roll_pitch_p2.lock().unwrap() += 1;
}

pub fn decrement_rollpitch_p2() {
    *INPUT_STATE_KB.roll_pitch_p2.lock().unwrap() -= 1;
}

pub fn reset_keyboard_values() {
    *INPUT_STATE_KB.pitch_trim.lock().unwrap() = 0;
    *INPUT_STATE_KB.roll_trim.lock().unwrap() = 0;
    *INPUT_STATE_KB.yaw_trim.lock().unwrap() = 0;
    *INPUT_STATE_KB.throttle_trim.lock().unwrap() = 0;
}

pub(crate) fn key_input(gui_params_1: GuiParams) {
    // println!("thread");
    let stdin = stdin();
    let mut stdout = stdout().into_raw_mode().unwrap();
    /*write!(stdout,
    "{}{}Backspace to exit. Type stuff, use alt, and so on.{}",
    termion::clear::All,
    termion::cursor::Goto(1, 1),
    termion::cursor::Hide)
     .unwrap();*/
    stdout.flush().unwrap();
    for c in stdin.keys() {
        // log::info!("\r");

        match c.unwrap() {
            Key::Char('q') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Increment Yaw Request".to_string();
                increment_yaw_trim();
            }
            Key::Char('w') => decrement_yaw_trim(),
            Key::Char('0') | Key::Esc | Key::Char(' ') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Requested Safe Mode".to_string();
                log::info!("mode {:#?}(0)", DroneMode::Safe);
                set_mode_pressed(DroneMode::Safe);
                *INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap() = true;
            }
            Key::Char('1') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Requested Panic Mode".to_string();
                log::info!("mode {:#?}(1)", DroneMode::Panic);
                set_mode_pressed(DroneMode::Panic);
                *INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap() = true;
            }
            Key::Char('2') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Requested Manual Mode".to_string();
                log::info!("mode {:#?}(2)", DroneMode::Manual);
                set_mode_pressed(DroneMode::Manual);
                *INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap() = true;
            }
            Key::Char('3') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Requested Calibrate Mode".to_string();
                log::info!("mode {:#?}(3)", DroneMode::Calibrate);
                set_mode_pressed(DroneMode::Calibrate);
                *INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap() = true;
            }
            Key::Char('4') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Requested Yaw Control Mode".to_string();
                log::info!("mode {:#?}(3)", DroneMode::YawControl);
                set_mode_pressed(DroneMode::YawControl);
                *INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap() = true;
            }

            Key::Char('5') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Requested Full Control Mode".to_string();
                log::info!("mode {:#?}(3)", DroneMode::FullControl);
                set_mode_pressed(DroneMode::FullControl);
                *INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap() = true;
            }

            Key::Char('f') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "clear GUI debug".to_string();
                *gui_params_1.debug_prints_from_drone.lock().unwrap() = " ".to_string();
                *gui_params_1.last_message_received.lock().unwrap() = " ".to_string();
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() = " ".to_string();
            }

            Key::Char('6') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Requested Raw Mode".to_string();
                log::info!("mode {:#?}(3)", DroneMode::RawMode);
                set_mode_pressed(DroneMode::RawMode);
                *INPUT_STATE_KB.is_new_mode_request_received.lock().unwrap() = true;
            }

            Key::Char('a') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Increment Lift trim".to_string();
                increment_lift_trim();
            }
            Key::Char('z') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Decrement Lift trim".to_string();
                decrement_lift_trim()
            }
            Key::Char('u') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Increment Yaw P".to_string();
                log::info!("yaw control P up");
                increment_yaw_p();
                *INPUT_STATE_KB.is_pid_updated.lock().unwrap() = true;
            }
            Key::Char('j') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Decrement Yaw P".to_string();
                log::info!("yaw control P down");
                decrement_yaw_p();
                *INPUT_STATE_KB.is_pid_updated.lock().unwrap() = true;
            }
            Key::Char('i') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Increment Roll/Pitch P1".to_string();
                log::info!("Control P1 up");
                increment_rollpitch_p1();
                *INPUT_STATE_KB.is_full_pid_updated.lock().unwrap() = true;
            }
            Key::Char('k') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Decrement Roll/Pitch P1".to_string();
                log::info!("Control P1 down");
                decrement_rollpitch_p1();
                *INPUT_STATE_KB.is_full_pid_updated.lock().unwrap() = true;
            }
            Key::Char('o') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Increment Roll/Pitch P2".to_string();
                log::info!("Control P2 up");
                increment_rollpitch_p2();
                *INPUT_STATE_KB.is_full_pid_updated.lock().unwrap() = true;
            }
            Key::Char('l') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Decrement Roll/Pitch P2".to_string();
                log::info!("Control P2 down");
                decrement_rollpitch_p2();
                *INPUT_STATE_KB.is_full_pid_updated.lock().unwrap() = true;
            }

            // TODO : GUI Additions
            Key::Char('c') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Start Logging".to_string();
                *INPUT_STATE_KB.data_logging_state.lock().unwrap() = true;
                *INPUT_STATE_KB.data_logging_action.lock().unwrap() = DataT::StartLogging;
            }

            Key::Char('v') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Stop Logging".to_string();
                *INPUT_STATE_KB.data_logging_state.lock().unwrap() = true;
                *INPUT_STATE_KB.data_logging_action.lock().unwrap() = DataT::StopLogging;
            }

            Key::Char('b') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Start Log Reporting".to_string();
                *INPUT_STATE_KB.data_logging_state.lock().unwrap() = true;
                *INPUT_STATE_KB.data_logging_action.lock().unwrap() = DataT::StartLogReporting;
            }

            Key::Char('n') => {
                *gui_params_1.last_keyboard_key_pressed.lock().unwrap() =
                    "Stop Log Reporting".to_string();
                *INPUT_STATE_KB.data_logging_state.lock().unwrap() = true;
                *INPUT_STATE_KB.data_logging_action.lock().unwrap() = DataT::StopLogReporting;
            }

            Key::Char('r') => reset_keyboard_values(),
            Key::Left => increment_roll_trim(),
            Key::Right => decrement_roll_trim(),
            Key::Up => increment_pitch_trim(),
            Key::Down => decrement_pitch_trim(),
            Key::Backspace => break,
            _ => {}
        }
        //stdout.flush().unwrap();
        //write!(stdout, "{}", termion::cursor::Show).unwrap();
        thread::sleep(Duration::from_millis(1));
    }
}
