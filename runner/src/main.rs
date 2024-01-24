#![feature(once_cell)]

// This crate configuration
mod gui;
mod input;
mod logger;
mod logic;
mod serial_wrapper;
mod utils;

// Rust libraries
use std::env::args;
use std::path::PathBuf;
use std::process::{exit, Command};
use std::sync::{Arc, Mutex};
use std::thread;
use std::thread::sleep;
use std::time::Duration;
use std::time::Instant;

// TUDelft library
use crate::gui::{gui_terminal_init, GuiParams};
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};

use crate::input::{joystick, keyboard};
use crate::utils::constants::TICK_RATE;

// Our libraries

// This crate imports

// TODO: put the joystick, keyboard and maybe quad_control_mappers in their own
// module, not next to main.

fn main() {
    logger::set_logger();
    log::info!("_________________START_DRONE_CODE________________________________");

    let gui_values = GuiParams {
        status: Arc::new(Mutex::new(0)),
        drone_mode: Arc::new(Mutex::new("Safe".to_string())),
        last_keyboard_key_pressed: Arc::new(Mutex::new("NA".to_string())),
        joystick_roll_input: Arc::new(Mutex::new(0)),
        joystick_pitch_input: Arc::new(Mutex::new(0)),
        joystick_yaw_input: Arc::new(Mutex::new(0)),
        joystick_throttle_input: Arc::new(Mutex::new(0)),
        motor_1_value: Arc::new(Mutex::new(0)),
        motor_2_value: Arc::new(Mutex::new(0)),
        motor_3_value: Arc::new(Mutex::new(0)),
        motor_4_value: Arc::new(Mutex::new(0)),
        yaw_p: Arc::new(Mutex::new(25)),
        rp_p1: Arc::new(Mutex::new(6)),
        rp_p2: Arc::new(Mutex::new(57)),
        last_message_received: Arc::new(Mutex::new("nothing so far".to_string())),
        debug_prints_from_drone: Arc::new(Mutex::new("nothing so far".to_string())),
        is_battery_weak: Arc::new(Mutex::new(false)),
        battery_health: Arc::new(Mutex::new(0)),
    };

    let gui_params_modifier_1 = gui_values.clone();
    let gui_params_modifier_2 = gui_values.clone();
    let gui_params_modifier_3 = gui_values.clone();

    // Try to upload the updated code to the drone
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    // Setup the serialport connection to the drone, for debugging purposes, serial is an option.
    serial_wrapper::init_global_serial(port, 115200);

    // The code then enters the main loop, which runs every 1000 ms for now.
    let interval = Duration::from_millis(TICK_RATE);

    let mut logic: logic::Logic = logic::Logic::default();

    start_keybord_joystick_interface(gui_params_modifier_1);

    thread::spawn(move || {
        gui_values_aggregator(gui_params_modifier_2);
    });

    sleep(Duration::from_millis(1000));

    let _ = thread::spawn(move || {
        let mut next_time;
        for i in 1.. {
            next_time = Instant::now() + interval;

            // program logic
            logic.tick(i, &interval, gui_params_modifier_3.clone());

            if next_time < Instant::now() {
                log::error!("Deadline exceeded by {:?}", Instant::now() - next_time);
                next_time = Instant::now() + interval;
            }

            // Busy wait
            sleep(next_time - Instant::now());
        }
    });

    gui_terminal_init(gui_values).expect("Unable to start Gui")
}

fn gui_values_aggregator(gui_params_modifier_2: GuiParams) {
    loop {
        //TODO: SEE IF THIS HURTS DATA SENDING
        *gui_params_modifier_2.joystick_pitch_input.lock().unwrap() =
            joystick::INPUT_STATE_JS.get_pitch();

        *gui_params_modifier_2.joystick_roll_input.lock().unwrap() =
            joystick::INPUT_STATE_JS.get_roll();

        *gui_params_modifier_2.joystick_yaw_input.lock().unwrap() =
            joystick::INPUT_STATE_JS.get_yaw();

        *gui_params_modifier_2
            .joystick_throttle_input
            .lock()
            .unwrap() = joystick::INPUT_STATE_JS.get_throttle();

        thread::sleep(Duration::from_millis(100));
        // *gui_params_modifier_2.drone_modelock().unwrap() =  joystick::INPUT_STATE_JS.get_mode_pressed();
    }
}

fn start_keybord_joystick_interface(gui_params_modifier_1: GuiParams) {
    let _joystick_thread = thread::spawn(move || {
        log::info!("Starting joystick Interface");
        pasts::block_on(joystick::event_loop());
    });

    let _keyboard_thread = thread::spawn(move || {
        log::info!("Starting Keyboard Interface");
        (keyboard::key_input(gui_params_modifier_1));
    });
}

// Start the interface passing the serial port as an argument to the program. This might be used later for the GUI
#[allow(unused)]
fn start_interface(port: &PathBuf) {
    let mut cmd = Command::new("python");
    cmd
        // there must be a `my_interface.py` file of course
        .arg("my_interface.py")
        // pass the serial port as a command line parameter to the python program
        .arg(port.to_str().unwrap());

    match cmd.output() {
        Err(e) => {
            log::error!("{}", e);
            exit(1);
        }
        Ok(i) if !i.status.success() => exit(i.status.code().unwrap_or(1)),
        Ok(_) => {}
    }
}
