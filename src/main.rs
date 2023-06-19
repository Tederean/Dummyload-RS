#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use embassy_time::{Duration, Timer};
use esp32_hal::{
    clock::ClockControl,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    Rtc,
    embassy
};
use esp_backtrace as _;
use static_cell::StaticCell;

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );

    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );

    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    let mut wdt0 = timer_group0.wdt;
    let mut wdt1 = timer_group1.wdt;

    rtc.rwdt.disable();
    wdt0.disable();
    wdt1.disable();

    embassy::init(&clocks, timer_group0.timer0);

    let executor = EXECUTOR.init(Executor::new());

    executor.run(|spawner| {
        spawner.spawn(run1()).unwrap();
        spawner.spawn(run2()).unwrap();
    })
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