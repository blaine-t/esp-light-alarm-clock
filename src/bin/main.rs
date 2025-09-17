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
use embassy_time::Timer;
use embedded_graphics::mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::{Drawable, Point};
use embedded_graphics::text::{Baseline, Text};
use embedded_hal_compat::ReverseCompat;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::lp_io::LowPowerInput;
use esp_hal::gpio::AnyPin;
use esp_hal::i2c::master::{Config, I2c};
use esp_hal::ledc::{channel, timer, LSGlobalClkSource, Ledc};
use esp_hal::load_lp_code;
use esp_hal::lp_core::{LpCore, LpCoreWakeupSource};
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
    
    spawner.spawn(lp_core_task(peripherals.LP_CORE, peripherals.GPIO2, peripherals.GPIO3)).ok();
    spawner.spawn(display_task(peripherals.I2C0, peripherals.GPIO23, peripherals.GPIO22)).ok();
    spawner.spawn(buzzer_task(peripherals.LEDC, peripherals.GPIO21.into(), peripherals.GPIO20.into())).ok();

    // Main loop can be empty since tasks handle everything
    loop {
        // Tasks are running independently
        Timer::after_millis(1000).await;
    }
}

// Function to read counter from LP Core shared memory
fn read_counter() -> i32 {
    unsafe {
        let ptr = 0x5000_2000 as *const i32; // RTC_DATA_LOW base address
        ptr.read_volatile()
    }
}

#[embassy_executor::task]
async fn lp_core_task(
    lp_core_peripheral: esp_hal::peripherals::LP_CORE<'static>, 
    gpio2: esp_hal::peripherals::GPIO2<'static>, 
    gpio3: esp_hal::peripherals::GPIO3<'static>
) {
    let rotary_dt = LowPowerInput::new(gpio2);
    let rotary_clk = LowPowerInput::new(gpio3);
    rotary_dt.pullup_enable(true);
    rotary_clk.pullup_enable(true);

    // Initialize LP Core
    let mut lp_core = LpCore::new(lp_core_peripheral);
    lp_core.stop();

    // Load and start the LP encoder reader program
    let lp_core_code = load_lp_code!(
        "lp-encoder-reader/target/riscv32imac-unknown-none-elf/release/lp-encoder-reader"
    );
    lp_core_code.run(
        &mut lp_core,
        LpCoreWakeupSource::HpCpu,
        rotary_dt,
        rotary_clk,
    );

    // LP Core setup is complete, task can finish or loop if needed
    loop {
        Timer::after_millis(10000).await; // Check periodically if needed
    }
}

#[embassy_executor::task]
async fn buzzer_task(ledc_peripheral: esp_hal::peripherals::LEDC<'static>, buzzer_pin: AnyPin<'static>, volume_pin: AnyPin<'static>) {
    let mut ledc = Ledc::new(ledc_peripheral);
    ledc.set_global_slow_clock(LSGlobalClkSource::APBClk);
    let mut buzzer = Buzzer::new(
        &mut ledc,
        timer::Number::Timer0,
        channel::Number::Channel1,
        buzzer_pin,
    )
    .with_volume(volume_pin, VolumeType::Duty);
    
    buzzer.play(1000).unwrap();

    loop {
        let counter = read_counter();

        if counter % 2 == 0 {
            buzzer.set_volume(25).unwrap();
        } else {
            buzzer.set_volume(0).unwrap();
        }

        Timer::after_millis(500).await; // Check periodically if needed
    }
}

#[embassy_executor::task]
async fn display_task(
    i2c0_peripheral: esp_hal::peripherals::I2C0<'static>,
    gpio23: esp_hal::peripherals::GPIO23<'static>,
    gpio22: esp_hal::peripherals::GPIO22<'static>
) {
    let i2c = I2c::new(i2c0_peripheral, Config::default())
        .unwrap()
        .with_sda(gpio23)
        .with_scl(gpio22)
        .reverse();

    let mut display: GraphicsMode<_> = sh1106::Builder::new().connect_i2c(i2c).into();
    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    loop {
        let counter = read_counter();

        display.clear();

        let mut buffer = [0u8; 32];
        let count_str =
            format_no_std::show(&mut buffer, format_args!("Count: {}", counter)).unwrap();
        Text::with_baseline(count_str, Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();

        Timer::after_millis(1).await; // Free core to do other tasks between refreshes
    }
}
