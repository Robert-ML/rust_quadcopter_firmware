#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

// This crate configuration
mod control;

mod calibrationdata;
mod drone;
mod sensors_dmp;
mod sensors_raw;
mod state_machine;
mod yaw_pitch_roll;

use core::alloc::Layout;
use core::mem::MaybeUninit;

// Other crates
extern crate alloc;

// TUDelft library
use tudelft_quadrupel::initialize::initialize;
use tudelft_quadrupel::led::Led::{Blue, Green, Red, Yellow};

use tudelft_quadrupel::{entry, uart};

// Our libraries

// This crate imports
use crate::control::control_loop;

// This module imports

// Constants
/// The heap size of your drone code in bytes.
/// Note: there are 8192 bytes of RAM available.
const HEAP_SIZE: usize = 4096;

#[entry]
fn main() -> ! {
    {
        static mut HEAP_MEMORY: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        // SAFETY: HEAP_MEMORY is a local static. That means, that at
        // the end of the surrounding block scope (not the main function, the local scope)
        // it is impossible to use HEAP_MEMORY *except* through this mutable reference
        // created here. Since that means that there's only one reference to HEAP_MEMORY,
        // this is safe.
        //
        // As soon as the first driver (led driver) is initialized, the yellow led turns on.
        // That's also the last thing that's turned off. If the yellow led stays on and your
        // program doesn't run, you know that the boot procedure has failed.
        initialize(unsafe { &mut HEAP_MEMORY }, true);
    }

    Blue.off();
    Green.off();
    Yellow.off();
    Red.off();

    control_loop()
}

#[allow(unused_imports)]
use core::panic::PanicInfo;
use tudelft_quadrupel::motor::set_motors;

#[inline(never)]
#[cfg(not(test))]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // On panic:
    // * try and write the panic message on UART
    // * blink the red light

    use alloc::format;
    use common::protocol::DataT;
    use tudelft_quadrupel::led::Led::Red;
    use tudelft_quadrupel::time::assembly_delay;
    use tudelft_quadrupel::uart::send_bytes;

    if uart::is_initialized() {
        let mut pipe = common::io::ComT::<8>::new(
            tudelft_quadrupel::uart::receive_bytes,
            tudelft_quadrupel::uart::send_bytes,
        );

        let mut msg: &str = "panic!";

        if let Some(m) = info.payload().downcast_ref::<&str>() {
            msg = m;
        }

        pipe.send_data::<64>(DataT::Message(heapless::String::from(
            alloc::format!("{:?}", msg).as_str(),
        )));
    }
    set_motors([0, 0, 0, 0]);
    // Start blinking red
    loop {
        let _ = Red.toggle();
        assembly_delay(1_000_000)
    }
}

#[alloc_error_handler]
fn alloc_error(layout: Layout) -> ! {
    // When an allocation error happens, we panic.
    // However, we do not want to have UART initialized, since
    // the panic handler uses the allocator to print a message over
    // UART. However, if UART is not initialized, it won't attempt
    // to allocate the message.
    //
    // instead, to signal this, we turn the green light on too
    // (together with blinking red of the panic)
    Green.on();

    // Safety: after this we panic and go into an infinite loop
    unsafe { uart::uninitialize() };

    panic!("out of memory: {layout:?}");
}
