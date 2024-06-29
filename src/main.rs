#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![allow(unused_variables)]

use esp_backtrace as _;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::{clock::ClockControl, embassy, IO, ledc, peripherals::Peripherals, prelude::*};
use esp_hal::gpio::{AnyPin, InputOutputAnalogPinType, Unknown};
use esp_hal::ledc::channel::Channel;
use esp_hal::ledc::HighSpeed;
use esp_hal::timer::TimerGroup;
use esp_println::println;
use crate::tasks::cooling;

mod tasks;

#[main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();

    embassy::init(&clocks, TimerGroup::new(peripherals.TIMG0, &clocks));

    esp_hal::interrupt::enable(
        esp_hal::peripherals::Interrupt::GPIO,
        esp_hal::interrupt::Priority::Priority1,
    ).unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let ledc = ledc::LEDC::new(peripherals.LEDC, &clocks);


    let mut hstimer0 = ledc.get_timer::<ledc::HighSpeed>(ledc::timer::Number::Timer0);

    hstimer0.configure(ledc::timer::config::Config {
        duty: ledc::timer::config::Duty::Duty10Bit,
        clock_source: ledc::timer::HSClockSource::APBClk,
        frequency: 25u32.kHz(),
    }).unwrap();

    let pwm_pin = io.pins.gpio33.degrade();
    let mut pwm_channel = ledc.get_channel(ledc::channel::Number::Channel0, pwm_pin);

    pwm_channel.configure(ledc::channel::config::Config {
        timer: &hstimer0,
        duty_pct: 10,
        pin_config: ledc::channel::config::PinConfig::OpenDrain,
    }).unwrap();

    let rpm_pin = io.pins.gpio34.into_floating_input().degrade();
    let one_wire_pin = io.pins.gpio25.into_open_drain_output();

    spawner.spawn(cooling::task(one_wire_pin, rpm_pin, unsafe { core::mem::transmute::<Channel<'_, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>, Channel<'static, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>>(pwm_channel) })).unwrap();

    println!("Hello world!");

    loop {
        println!("Loop...");

        Timer::after(Duration::from_millis(500u64)).await;
    }
}
