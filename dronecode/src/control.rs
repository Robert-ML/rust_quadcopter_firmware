// Rust libraries
use core::time::Duration;
use tudelft_quadrupel::flash::flash_chip_erase;

// Our libraries
use common::protocol::DataT;

// TUDelft library
use tudelft_quadrupel::motor::set_motor_max;
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};

// This crate imports
use crate::drone::state::DroneState;

pub fn control_loop() -> ! {
    set_tick_frequency(100);
    set_motor_max(800);
    let mut last = Instant::now();
    let mut drone: DroneState = DroneState::new();
    flash_chip_erase();

    for i in 1.. {
        let delta_t: Duration = update_last_n_get_delta(&mut last);
        if delta_t.as_millis() > 10 {
            let err = DataT::Message(heapless::String::<32>::from("Exceeding deadline!"));
            drone.send_data(err);
        }
        drone.tick(i, delta_t);
        wait_for_next_tick();
    }
    unreachable!();
}

#[inline]
fn update_last_n_get_delta(last: &mut Instant) -> Duration {
    let now = Instant::now();
    let dt = now.duration_since(*last);
    *last = now;
    dt
}
