#![no_std]
#![no_main]

mod fmt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{Duration, Timer};
use fmt::info;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::task]
async fn blinker(mut led: Output<'static>) {
    info!("Run blinker!");
    loop {
        led.set_high();
        Timer::after(Duration::from_millis(500)).await;
        led.set_low();
        Timer::after(Duration::from_millis(500)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("Hello, World!");

    let p = embassy_stm32::init(Default::default());
    let led = Output::new(p.PC13, Level::High, Speed::Low);

    spawner.spawn(blinker(led).unwrap());

    loop {
        Timer::after(Duration::from_secs(1)).await;
    }
}
