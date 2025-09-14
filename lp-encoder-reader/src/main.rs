//! Writes rotary encoder output to a known point in memory
//!
//! When using the ESP32-C6's LP core, this address in memory is `0x5000_2000`.
//!
//! When using the ESP32-S2 or ESP32-S3's ULP core, this address in memory is
//! `0x5000_0400` (but is `0x400`` from the ULP's point of view!).
//!
//! Make sure the LP RAM is cleared before loading the code.

#![no_std]
#![no_main]

use esp_lp_hal::{gpio::Input, prelude::*};
use panic_halt as _;
use rotary_encoder_embedded::{Direction, RotaryEncoder};

cfg_if::cfg_if! {
    if #[cfg(feature = "esp32c6")] {
        const ADDRESS: u32 = 0x5000_2000;
    } else if #[cfg(any(feature = "esp32s2", feature = "esp32s3"))] {
        const ADDRESS: u32 = 0x400;
    }
}

#[entry]
fn main(rotary_dt: Input<2>, rotary_clk: Input<3>) -> ! {
    let mut i: i32 = 0;

    let ptr = ADDRESS as *mut i32;

    let mut rotary_encoder = RotaryEncoder::new(rotary_dt, rotary_clk).into_standard_mode();

    loop {
        match rotary_encoder.update() {
            Direction::Clockwise => {
                i = i.wrapping_add(1i32);
            }
            Direction::Anticlockwise => {
                i = i.wrapping_sub(1i32);
            }
            Direction::None => {
                // Do nothing
            }
        }

        unsafe {
            ptr.write_volatile(i);
        }

        esp_lp_hal::delay::Delay.delay_millis(1);
    }
}
