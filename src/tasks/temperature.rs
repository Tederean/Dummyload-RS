use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::{AnyPin, Floating, Input, InputOnlyAnalogPinType};

const SUBSCRIBER_COUNT: usize = 1;

pub static TEMPERATURE_CHANGED: PubSubChannel::<CriticalSectionRawMutex, Option<uom::si::f32::ThermodynamicTemperature>, 1, SUBSCRIBER_COUNT, 1> = PubSubChannel::<CriticalSectionRawMutex, Option<uom::si::f32::ThermodynamicTemperature>, 1, SUBSCRIBER_COUNT, 1>::new();

#[embassy_executor::task]
pub async fn task(mut rpm_pin: AnyPin<Input<Floating>, InputOnlyAnalogPinType>) -> ! {
    let publisher = TEMPERATURE_CHANGED.publisher().unwrap();

    const DELAY: Duration = Duration::from_millis(1000);

    loop {
        Timer::after(DELAY).await;
    }
}
