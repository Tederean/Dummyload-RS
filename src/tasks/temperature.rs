use core::convert::Infallible;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::{GpioPin, OpenDrain, Output};
use onewire::OneWire;

const SUBSCRIBER_COUNT: usize = 1;

pub static TEMPERATURE_CHANGED: PubSubChannel::<CriticalSectionRawMutex, Option<uom::si::f32::ThermodynamicTemperature>, 1, SUBSCRIBER_COUNT, 1> = PubSubChannel::<CriticalSectionRawMutex, Option<uom::si::f32::ThermodynamicTemperature>, 1, SUBSCRIBER_COUNT, 1>::new();

#[embassy_executor::task]
pub async fn task(mut one_wire_pin: GpioPin<Output<OpenDrain>, 25>) -> ! {
    let publisher = TEMPERATURE_CHANGED.publisher().unwrap();

    // Use critical section mutex in order to disable interrupts.
    // OneWire protocol is time-critical and must always be running with disabled interrupts.
    let one_wire_mutex = Mutex::<CriticalSectionRawMutex, OneWire<Infallible>>::new(OneWire::new(&mut one_wire_pin, false));

    let mut time = Instant::now();

    loop {
        one_wire_mutex.lock(|one_wire| {

        });

        Timer::after(Duration::from_millis(700)).await;

        one_wire_mutex.lock(|one_wire| {

        });

        time += Duration::from_millis(1000);

        Timer::after(time - Instant::now()).await;
    }
}
