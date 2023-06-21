#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

mod devices;
mod units;

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32_hal::clock::Clocks;
use esp32_hal::peripherals::{RTC_CNTL, TIMG0, TIMG1};
use esp32_hal::system::PeripheralClockControl;
use esp32_hal::{
    clock::ClockControl, embassy, ledc::LEDC, peripherals::Peripherals, prelude::*,
    timer::TimerGroup, Rtc, IO,
};
use esp_backtrace as _;
use static_cell::StaticCell;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    setup_timers(
        peripherals.TIMG0,
        peripherals.TIMG1,
        peripherals.RTC_CNTL,
        &clocks,
        &mut system.peripheral_clock_control,
    );

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let ledc = LEDC::new(
        peripherals.LEDC,
        &clocks,
        &mut system.peripheral_clock_control,
    );

    devices::FanDevice::new(io.pins.gpio33, io.pins.gpio34, ledc);

    let executor = EXECUTOR.init(Executor::new());

    executor.run(|spawner| {
        spawner.spawn(run1()).unwrap();
        spawner.spawn(run2()).unwrap();
    })
}

fn setup_timers(
    timg0: TIMG0,
    timg1: TIMG1,
    rtc_cntl: RTC_CNTL,
    clocks: &Clocks,
    peripheral_clock_control: &mut PeripheralClockControl,
) {
    let timer_group0 = TimerGroup::new(timg0, &clocks, peripheral_clock_control);

    let timer_group1 = TimerGroup::new(timg1, &clocks, peripheral_clock_control);

    let mut rtc = Rtc::new(rtc_cntl);
    let mut wdt0 = timer_group0.wdt;
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    embassy::init(&clocks, timer_group0.timer0);
}

#[embassy_executor::task]
async fn run1() {
    loop {
        esp_println::println!("Hello world from embassy using esp-hal-async!");
        Timer::after(Duration::from_millis(1_000)).await;
    }
}

#[embassy_executor::task]
async fn run2() {
    loop {
        esp_println::println!("Bing!");
        Timer::after(Duration::from_millis(5_000)).await;
    }
}
