use core::cell::RefCell;
use core::convert::Infallible;
use ds18b20::{Ds18b20, Resolution};
use embassy_futures::join::join;
use embassy_futures::select::Either::{First, Second};
use embassy_futures::select::select;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal::pwm::SetDutyCycle;
use embedded_hal_async::digital::Wait;
use esp_hal::gpio::{AnyPin, Floating, GpioPin, Input, InputOnlyAnalogPinType, InputOutputAnalogPinType, OpenDrain, Output, Unknown};
use esp_hal::ledc::channel::Channel;
use esp_hal::ledc::HighSpeed;
use one_wire_bus::{Address, OneWire, OneWireError};

const SUBSCRIBER_COUNT: usize = 1;

pub static CHANNEL: PubSubChannel::<CriticalSectionRawMutex, ChannelUpdate, 1, SUBSCRIBER_COUNT, 1> = PubSubChannel::<CriticalSectionRawMutex, ChannelUpdate, 1, SUBSCRIBER_COUNT, 1>::new();

#[derive(Clone, Copy, Debug)]
pub struct ChannelUpdate {
    fan_power: uom::si::f32::Ratio,
    fan_speed: Result<uom::si::f32::AngularVelocity, FanSpeedError>,
    temperature: Result<uom::si::f32::ThermodynamicTemperature, TemperatureError>,
}

#[derive(Clone, Copy, Debug)]
pub enum FanSpeedError {
    SignalTimeout,
}

#[derive(Clone, Copy, Debug)]
pub enum TemperatureError {
    SetupError(OneWireError<Infallible>),
    StartMeasurementError(OneWireError<Infallible>),
    ReadMeasurementError(OneWireError<Infallible>),
}

type OpenDrainPin = GpioPin<Output<OpenDrain>, 25>;

struct CriticalSectionData {
    one_wire_bus: RefCell<OneWire<OpenDrainPin>>,
    first_sensor: Ds18b20,
    second_sensor: Ds18b20,
}

#[embassy_executor::task]
pub async fn task(
    one_wire_pin: OpenDrainPin,
    mut rpm_pin: AnyPin<Input<Floating>, InputOnlyAnalogPinType>,
    mut pwm_channel: Channel<'static, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>,
    ) -> ! {
    let publisher = CHANNEL.publisher().unwrap();

    let mut delay = Delay;
    let one_wire_result = setup_one_wire(one_wire_pin, &mut delay);

    let mut time = Instant::now();

    loop {
        let temperature_future = measure_temperature(&one_wire_result, &mut delay);
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

fn setup_one_wire(one_wire_pin: OpenDrainPin, delay: &mut Delay) -> Result<Mutex::<CriticalSectionRawMutex, CriticalSectionData>, OneWireError<Infallible>> {
    let critical_section_data = CriticalSectionData {
        one_wire_bus: RefCell::new(OneWire::new(one_wire_pin)?),
        first_sensor: Ds18b20::new::<Infallible>(Address(12970953402624835368u64))?, // or maybe 2954247596452151988
        second_sensor: Ds18b20::new::<Infallible>(Address(15132681223968980776))?, // or maybe 2954331210875470546
    };

    // Use critical section mutex in order to disable interrupts.
    // OneWire protocol is time-critical and must always be running with disabled interrupts.

    let one_wire = Mutex::<CriticalSectionRawMutex, CriticalSectionData>::new(critical_section_data);

    one_wire.lock(|data| {
        let mut one_wire_bus = data.one_wire_bus.borrow_mut();

        setup_ds18b20(&mut one_wire_bus, &data.first_sensor, delay)?;
        setup_ds18b20(&mut one_wire_bus, &data.second_sensor, delay)?;

        Ok(())
    })?;

    Ok(one_wire)
}

#[inline]
fn setup_ds18b20(one_wire_bus: &mut OneWire<OpenDrainPin>, sensor: &Ds18b20, delay: &mut Delay) -> Result<(), OneWireError<Infallible>> {
    sensor.set_config(0, 60, Resolution::Bits10, one_wire_bus, delay)
}

async fn measure_temperature(one_wire_result: &Result<Mutex::<CriticalSectionRawMutex, CriticalSectionData>, OneWireError<Infallible>>, delay: &mut Delay) -> Result<uom::si::f32::ThermodynamicTemperature, TemperatureError> {
    match one_wire_result {
        Err(err) => Err(TemperatureError::SetupError(err.clone())),
        Ok(one_wire) => {
            one_wire.lock(|data| {
                let mut one_wire_bus = data.one_wire_bus.borrow_mut();

                data.first_sensor.start_temp_measurement(&mut one_wire_bus, delay)?;
                data.second_sensor.start_temp_measurement(&mut one_wire_bus, delay)?;

                Ok(())
            }).map_err(|err| TemperatureError::StartMeasurementError(err))?;

            Timer::after(Duration::from_millis(190)).await; // Datasheet: 187.5 ms

            let temperature_degree_celsius = one_wire.lock(|data| {
                let mut one_wire_bus = data.one_wire_bus.borrow_mut();

                let first_temperature = data.first_sensor.read_data(&mut one_wire_bus, delay)?;
                let second_temperature = data.second_sensor.read_data(&mut one_wire_bus, delay)?;

                Ok(f32::max(first_temperature.temperature, second_temperature.temperature))
            }).map_err(|err| TemperatureError::ReadMeasurementError(err))?;

            Ok(uom::si::f32::ThermodynamicTemperature::new::<uom::si::thermodynamic_temperature::degree_celsius>(temperature_degree_celsius))
        },
    }
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

fn adjust_fan_power(
    pwm_channel: &mut Channel<'static, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>,
    temperature_result: &Result<uom::si::f32::ThermodynamicTemperature, TemperatureError>,
    ) -> uom::si::f32::Ratio {
        match temperature_result {
            Ok(temperature) => {
                let duty_cycle = f32::clamp(3.5f32 * temperature.get::<uom::si::thermodynamic_temperature::degree_celsius>() - 75.0f32, 30.0f32, 100.0f32);

                pwm_channel.set_duty_cycle_percent(duty_cycle as u8).unwrap();

                uom::si::f32::Ratio::new::<uom::si::ratio::percent>(duty_cycle)
            },
            Err(_) => {
                pwm_channel.set_duty_cycle_percent(100).unwrap();

                uom::si::f32::Ratio::new::<uom::si::ratio::percent>(100.0f32)
            },
        }
}