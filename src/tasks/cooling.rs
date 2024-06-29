use core::convert::Infallible;
use embassy_futures::join::join;
use embassy_futures::select::Either::{First, Second};
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal_async::digital::Wait;
use esp_hal::gpio::{AnyPin, Floating, GpioPin, Input, InputOnlyAnalogPinType, InputOutputAnalogPinType, OpenDrain, Output, Unknown};
use esp_hal::ledc::channel::Channel;
use esp_hal::ledc::HighSpeed;
use onewire::OneWire;

const SUBSCRIBER_COUNT: usize = 1;

pub static CHANNEL: PubSubChannel::<CriticalSectionRawMutex, ChannelUpdate, 1, SUBSCRIBER_COUNT, 1> = PubSubChannel::<CriticalSectionRawMutex, ChannelUpdate, 1, SUBSCRIBER_COUNT, 1>::new();

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct ChannelUpdate {
    fan_power: uom::si::f32::Ratio,
    fan_speed: Result<uom::si::f32::AngularVelocity, FanSpeedError>,
    temperature: Result<uom::si::f32::ThermodynamicTemperature, TemperatureError>,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum FanSpeedError {
    SignalTimeout,
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum TemperatureError {
    SensorNotFound,
    WireNotHigh,
    CrcMismatch(u8, u8),
    FamilyCodeMismatch(u8, u8),
}

#[embassy_executor::task]
pub async fn task(
    mut one_wire_pin: GpioPin<Output<OpenDrain>, 25>,
    mut rpm_pin: AnyPin<Input<Floating>, InputOnlyAnalogPinType>,
    mut pwm_channel: Channel<'static, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>,
    ) -> ! {
    let publisher = CHANNEL.publisher().unwrap();

    // Use critical section mutex in order to disable interrupts.
    // OneWire protocol is time-critical and must always be running with disabled interrupts.
    let one_wire_mutex = Mutex::<CriticalSectionRawMutex, OneWire<Infallible>>::new(OneWire::new(&mut one_wire_pin, false));

    let mut time = Instant::now();

    loop {
        let temperature_future = measure_temperature(&one_wire_mutex);
        let fan_speed_future = measure_fan_speed(&mut rpm_pin);

        let (temperature, fan_speed) = join(temperature_future, fan_speed_future).await;
        let fan_power = adjust_fan_power(&mut pwm_channel, &temperature);

        publisher.publish_immediate(ChannelUpdate {
            fan_power,
            fan_speed,
            temperature,
        });

        time += Duration::from_millis(1000);

        let now = Instant::now();

        if now < time {
            Timer::after(time - now).await;
        }
    }
}

fn adjust_fan_power(
    pwm_channel: &mut Channel<'static, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>,
    temperature_result: &Result<uom::si::f32::ThermodynamicTemperature, TemperatureError>,
    ) -> uom::si::f32::Ratio {
        match temperature_result {
            Ok(temperature) => {
                let duty_cycle = f32::clamp(3.5f32 * temperature.get::<uom::si::thermodynamic_temperature::degree_celsius>() - 75.0f32, 0.0f32, 100.0f32);

                pwm_channel.set_duty_cycle_percent(duty_cycle as u8).unwrap();

                uom::si::f32::Ratio::new::<uom::si::ratio::percent>(duty_cycle)
            },
            Err(_) => {
                pwm_channel.set_duty_cycle_percent(100).unwrap();

                uom::si::f32::Ratio::new::<uom::si::ratio::percent>(100.0f32)
            },
        }
}

async fn measure_temperature(one_wire_mutex: &Mutex::<CriticalSectionRawMutex, OneWire<'_, Infallible>>) -> Result<uom::si::f32::ThermodynamicTemperature, TemperatureError> {
    one_wire_mutex.lock(|one_wire| {

    });

    Timer::after(Duration::from_millis(300)).await;

    one_wire_mutex.lock(|one_wire| {

    });

    todo!()
}

async fn measure_fan_speed(rpm_pin: &mut AnyPin<Input<Floating>, InputOnlyAnalogPinType>) -> Result<uom::si::f32::AngularVelocity, FanSpeedError> {
    let timeout_future = Timer::after(Duration::from_millis(300));
    let elapsed_us_future = measure_cycle_us(rpm_pin);

    match select(timeout_future, elapsed_us_future).await {
        First(_) => {
            Err(FanSpeedError::SignalTimeout)
        },
        Second(elapsed_us) => {
            let elapsed_time = uom::si::f32::Time::new::<uom::si::time::microsecond>(elapsed_us as f32);

            let fan_speed = uom::si::f32::Angle::HALF_TURN / elapsed_time;

            Ok(fan_speed.into())
        }
    }
}

async fn measure_cycle_us(rpm_pin: &mut AnyPin<Input<Floating>, InputOnlyAnalogPinType>) -> u64 {
    rpm_pin.wait_for_rising_edge().await.unwrap();

    let first_rising_edge = Instant::now();

    rpm_pin.wait_for_rising_edge().await.unwrap();

    let second_rising_edge = Instant::now();

    (second_rising_edge - first_rising_edge).as_micros()
}