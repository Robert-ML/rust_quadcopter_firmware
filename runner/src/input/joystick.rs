// Rust libraries
use std::sync::{Arc, Mutex};
use std::task::Poll::{self, Pending};

use common::DroneMode;
// Other crates
use common::protocol::DataT;
use lazy_static::lazy_static;
use pasts::Loop;
use stick::{Controller, Event, Listener};

use crate::input::InputState;
type Exit = usize;

lazy_static! {
    pub static ref INPUT_STATE_JS: InputState = InputState {
        mode_request: Arc::new(Mutex::new(None)),
        throttle: Arc::new(Mutex::new(2047)),
        roll: Arc::new(Mutex::new(1024)),
        pitch: Arc::new(Mutex::new(1024)),
        yaw: Arc::new(Mutex::new(1024)),
        yaw_p: Arc::new(Mutex::new(0)),
        roll_pitch_p1: Arc::new(Mutex::new(5)),
        roll_pitch_p2: Arc::new(Mutex::new(5)),
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

pub struct State {
    listener: Listener,
    controllers: Vec<Controller>,
}

impl State {
    fn connect(&mut self, controller: Controller) -> Poll<Exit> {
        // println!(
        //     "Connected p{}, id: {:016X}, name: {}",
        //     self.controllers.len() + 1,
        //     controller.id(),
        //     controller.name(),
        // );
        self.controllers.push(controller);
        Pending
    }

    fn event(&mut self, id: usize, event: Event) -> Poll<Exit> {
        let _player = id + 1;

        match event {
            Event::CamZ(x) => {
                INPUT_STATE_JS.set_yaw(scale_input(x, -0.5 as f64, -1 as f64));
            }

            Event::Throttle(x) => {
                INPUT_STATE_JS.set_throttle(scale_input(x, 1 as f64, 0 as f64));
            }

            Event::JoyY(x) => {
                INPUT_STATE_JS.set_pitch(scale_input(x, -1 as f64, 1 as f64));
            }

            Event::JoyX(x) => {
                INPUT_STATE_JS.set_roll(scale_input(x, -1 as f64, 1 as f64));
            }
            Event::Trigger(true) => {
                INPUT_STATE_JS.set_mode_pressed(DroneMode::Panic);
                *INPUT_STATE_JS.is_new_mode_request_received.lock().unwrap() = true;
            }

            _ => {}
        }
        Pending
    }
}

pub fn scale_input(value: f64, lowest_value: f64, highest_value: f64) -> i32 {
    let scaled_value = ((value - lowest_value) / (highest_value - lowest_value)) * (2048 as f64);
    scaled_value as i32
}

pub async fn event_loop() {
    let mut state = State {
        listener: Listener::default(),
        controllers: Vec::new(),
    };

    let player_id = Loop::new(&mut state)
        .when(|s| &mut s.listener, State::connect)
        .poll(|s| &mut s.controllers, State::event)
        .await;
    // println!("p{} ended the session", player_id);
}
