#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use bt_hci::controller::ExternalController;
use defmt::info;
use embassy_executor::Spawner;
use embedded_graphics::mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::{Drawable, Point};
use embedded_graphics::text::{Baseline, Text};
use embedded_hal_compat::ReverseCompat;
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc};
use esp_hal::timer::systimer::SystemTimer;
use esp_hal::timer::timg::TimerGroup;
use esp_hal_buzzer::{Buzzer, VolumeType};
use esp_wifi::ble::controller::BleConnector;
use sh1106::mode::GraphicsMode;
use {esp_backtrace as _, esp_println as _};

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // generator version: 0.5.0

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);
    // COEX needs more RAM - so we've added some more
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 64 * 1024);

    let timer0 = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(timer0.alarm0);

    info!("Embassy initialized!");

    let rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timer1 = TimerGroup::new(peripherals.TIMG0);
    let wifi_init =
        esp_wifi::init(timer1.timer0, rng).expect("Failed to initialize WIFI/BLE controller");
    let (mut _wifi_controller, _interfaces) = esp_wifi::wifi::new(&wifi_init, peripherals.WIFI)
        .expect("Failed to initialize WIFI controller");
    // find more examples https://github.com/embassy-rs/trouble/tree/main/examples/esp32
    let transport = BleConnector::new(&wifi_init, peripherals.BT);
    let _ble_controller = ExternalController::<_, 20>::new(transport);
    // TODO: Spawn some tasks
    let _ = spawner;

    let i2c = I2c::new(peripherals.I2C0, Config::default())
        .unwrap()
        .with_sda(peripherals.GPIO23)
        .with_scl(peripherals.GPIO22)
        .reverse();

    let mut display: GraphicsMode<_> = sh1106::Builder::new().connect_i2c(i2c).into();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut buzzer = Buzzer::new(
        &ledc,
        timer::Number::Timer0,
        channel::Number::Channel1,
        peripherals.GPIO21,
    ).with_volume(peripherals.GPIO20, VolumeType::Duty);

    let mut counter: u32 = 0;
    loop {
        display.clear();

        counter += 1;
        let mut buffer = [0u8; 32];
        let count_str =
            format_no_std::show(&mut buffer, format_args!("Count: {}", counter)).unwrap();
        Text::with_baseline(count_str, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();

        info!("Counter: {}", counter);

        if counter % 2 == 0 {
            buzzer.play(counter).unwrap();
            buzzer.set_volume(20).unwrap();
        } else {
            buzzer.set_volume(0).unwrap();
        }
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-rc.0/examples/src/bin
}
