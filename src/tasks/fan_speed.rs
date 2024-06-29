use embassy_futures::select::Either::{First, Second};
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_async::digital::Wait;
use esp_hal::gpio::{AnyPin, Floating, Input, InputOnlyAnalogPinType};

const SUBSCRIBER_COUNT: usize = 1;

pub static FAN_SPEED_CHANGED: PubSubChannel::<CriticalSectionRawMutex, Option<uom::si::f32::AngularVelocity>, 1, SUBSCRIBER_COUNT, 1> = PubSubChannel::<CriticalSectionRawMutex, Option<uom::si::f32::AngularVelocity>, 1, SUBSCRIBER_COUNT, 1>::new();

#[embassy_executor::task]
pub async fn task(mut rpm_pin: AnyPin<Input<Floating>, InputOnlyAnalogPinType>) -> ! {
    let publisher = FAN_SPEED_CHANGED.publisher().unwrap();

    const TIMEOUT: Duration = Duration::from_millis(300);
    const DELAY: Duration = Duration::from_millis(1000);

    loop {
        let timeout_future = Timer::after(TIMEOUT);
        let elapsed_us_future = measure_cycle_us(&mut rpm_pin);

        match select(timeout_future, elapsed_us_future).await {
            First(_) => {
                publisher.publish_immediate(None);
            },
            Second(elapsed_us) => {
                let elapsed_time = uom::si::f32::Time::new::<uom::si::time::microsecond>(elapsed_us as f32);

                let fan_speed = uom::si::f32::Angle::HALF_TURN / elapsed_time;

                publisher.publish_immediate(Some(fan_speed.into()));
            }
        }

        Timer::after(DELAY).await;
    }
}

async fn measure_cycle_us(rpm_pin: &mut AnyPin<Input<Floating>, InputOnlyAnalogPinType>) -> u64 {
    rpm_pin.wait_for_rising_edge().await.unwrap();

    let first_rising_edge = Instant::now();

    rpm_pin.wait_for_rising_edge().await.unwrap();

    let second_rising_edge = Instant::now();

    (second_rising_edge - first_rising_edge).as_micros()
}