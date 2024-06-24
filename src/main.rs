#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_variables)]
#![deny(unsafe_code)]

use esp_backtrace as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{clock::ClockControl, embassy, IO, ledc, peripherals::Peripherals, prelude::*};
use esp_hal::timer::TimerGroup;
use esp_println::println;
use crate::devices::FanController;

pub mod devices;

#[main]
async fn main(spawner: Spawner) {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();

    embassy::init(&clocks, TimerGroup::new(peripherals.TIMG0, &clocks));




    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let pwm_pin = io.pins.gpio33.degrade();
    let rpm_pin = io.pins.gpio34.degrade();




    let ledc = ledc::LEDC::new(peripherals.LEDC, &clocks);

    let mut hstimer0 = ledc.get_timer::<ledc::HighSpeed>(ledc::timer::Number::Timer0);

    hstimer0.configure(ledc::timer::config::Config {
        duty: ledc::timer::config::Duty::Duty10Bit,
        clock_source: ledc::timer::HSClockSource::APBClk,
        frequency: 25u32.kHz(),
    }).unwrap();



    let mut pwm_channel = ledc.get_channel(ledc::channel::Number::Channel0, pwm_pin);

    pwm_channel.configure(ledc::channel::config::Config {
        timer: &hstimer0,
        duty_pct: 10,
        pin_config: ledc::channel::config::PinConfig::OpenDrain,
    }).unwrap();







    let fan_controller = FanController::new(pwm_channel);

    fan_controller.get_fan_speed();





    println!("Hello world!");

    loop {
        println!("Loop...");

        Timer::after(Duration::from_millis(500u64)).await;
    }
}
