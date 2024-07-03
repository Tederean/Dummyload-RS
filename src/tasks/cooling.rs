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
use embedded_hal_async::digital::Wait;
use esp_hal::gpio::{AnyPin, Floating, GpioPin, Input, InputOnlyAnalogPinType, InputOutputAnalogPinType, OpenDrain, Output, Unknown};
use esp_hal::ledc::channel::Channel;
use esp_hal::ledc::HighSpeed;
use esp_hal::prelude::_esp_hal_ledc_channel_ChannelHW;
use one_wire_bus::{Address, OneWire, OneWireError};
use uom::si::f32::{ThermodynamicTemperature, AngularVelocity, Time, Ratio, Angle};
use uom::si::thermodynamic_temperature;
use uom::si::time;
use uom::si::ratio;

const SUBSCRIBER_COUNT: usize = 1;

pub static CHANNEL: PubSubChannel::<CriticalSectionRawMutex, ChannelUpdate, 1, SUBSCRIBER_COUNT, 1> = PubSubChannel::<CriticalSectionRawMutex, ChannelUpdate, 1, SUBSCRIBER_COUNT, 1>::new();

#[derive(Clone, Copy, Debug)]
pub struct ChannelUpdate {
    pub fan_power: Ratio,
    pub fan_speed: Result<AngularVelocity, FanSpeedError>,
    pub temperature: Result<ThermodynamicTemperature, TemperatureError>,
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
type PwmChannel = Channel<'static, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>;
type FloatingInputPin = AnyPin<Input<Floating>, InputOnlyAnalogPinType>;
type OneWireMutex = Mutex<CriticalSectionRawMutex, OneWireData>;

struct OneWireData {
    bus: RefCell<OneWire<OpenDrainPin>>,
    resolution: Resolution,
    sensors: [Ds18b20; 2],
}

#[embassy_executor::task]
pub async fn task(
    one_wire_pin: OpenDrainPin,
    mut rpm_pin: FloatingInputPin,
    mut pwm_channel: PwmChannel,
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

        if now < time { // 'time - now' panics if 'now' is larger than 'time'.
            Timer::after(time - now).await;
        }
    }
}

fn setup_one_wire(one_wire_pin: OpenDrainPin, delay: &mut Delay) -> Result<OneWireMutex, OneWireError<Infallible>> {

    // Use critical section mutex in order to disable interrupts.
    // OneWire protocol is time-critical and must always be running with disabled interrupts.

    let one_wire_mutex = OneWireMutex::new(OneWireData {
        bus: RefCell::new(OneWire::new(one_wire_pin)?),
        resolution: Resolution::Bits12,
        sensors: [
            Ds18b20::new::<Infallible>(Address(12970953402624835368u64))?,
            Ds18b20::new::<Infallible>(Address(15132681223968980776u64))?,
        ],
    });

    one_wire_mutex.lock(|one_wire| {
        let mut one_wire_bus = one_wire.bus.borrow_mut();

        for sensor in &one_wire.sensors {
            sensor.set_config(0, 60, one_wire.resolution, &mut one_wire_bus, delay)?;
        }

        ds18b20::simultaneous_save_to_eeprom(&mut one_wire_bus, delay)?;
        ds18b20::simultaneous_recall_from_eeprom(&mut one_wire_bus, delay)?;

        Ok(())
    })?;

    Ok(one_wire_mutex)
}

async fn measure_temperature(one_wire_result: &Result<OneWireMutex, OneWireError<Infallible>>, delay: &mut Delay) -> Result<ThermodynamicTemperature, TemperatureError> {
    match one_wire_result {
        Err(err) => Err(TemperatureError::SetupError(err.clone())),
        Ok(one_wire_mutex) => {
            let delay_duration_ms = one_wire_mutex.lock(|one_wire| {
                let mut one_wire_bus = one_wire.bus.borrow_mut();

                ds18b20::start_simultaneous_temp_measurement(&mut one_wire_bus, delay)?;

                Ok(one_wire.resolution.max_measurement_time_millis())

            }).map_err(|err| TemperatureError::StartMeasurementError(err))?;

            Timer::after(Duration::from_millis((delay_duration_ms + 10) as u64)).await;

            one_wire_mutex.lock(|one_wire| {
                let mut one_wire_bus = one_wire.bus.borrow_mut();

                let mut temperature_degree_celsius = ThermodynamicTemperature::new::<thermodynamic_temperature::kelvin>(0.0f32).get::<thermodynamic_temperature::degree_celsius>();

                for sensor in &one_wire.sensors {
                    let ds18b20_result = sensor.read_data(&mut one_wire_bus, delay)?;

                    if ds18b20_result.temperature > temperature_degree_celsius {
                        temperature_degree_celsius = ds18b20_result.temperature;
                    }
                }

                Ok(ThermodynamicTemperature::new::<thermodynamic_temperature::degree_celsius>(temperature_degree_celsius))

            }).map_err(|err| TemperatureError::ReadMeasurementError(err))
        }
    }
}

async fn measure_fan_speed(rpm_pin: &mut FloatingInputPin) -> Result<AngularVelocity, FanSpeedError> {
    let timeout_future = Timer::after(Duration::from_millis(700));
    let elapsed_us_future = measure_cycle_us(rpm_pin);

    match select(timeout_future, elapsed_us_future).await {
        First(_) => {
            Err(FanSpeedError::SignalTimeout)
        },
        Second(elapsed_us) => {
            let elapsed_time = Time::new::<time::microsecond>(elapsed_us as f32);

            let fan_speed = Angle::HALF_TURN / elapsed_time;

            Ok(fan_speed.into())
        }
    }
}

async fn measure_cycle_us(rpm_pin: &mut FloatingInputPin) -> u64 {
    rpm_pin.wait_for_rising_edge().await.unwrap();

    let first_rising_edge = Instant::now();

    rpm_pin.wait_for_rising_edge().await.unwrap();

    let second_rising_edge = Instant::now();

    (second_rising_edge - first_rising_edge).as_micros()
}

fn adjust_fan_power(pwm_channel: &mut PwmChannel, temperature_result: &Result<ThermodynamicTemperature, TemperatureError>) -> Ratio {
        match temperature_result {
            Ok(temperature) => {
                let duty_cycle_ratio = f32::clamp(0.035f32 * temperature.get::<thermodynamic_temperature::degree_celsius>() - 0.75f32, 0.3f32, 1.0f32);

                pwm_channel.set_duty_hw((duty_cycle_ratio * 1024.0f32) as u32);

                Ratio::new::<ratio::ratio>(duty_cycle_ratio)
            },
            Err(_) => {
                pwm_channel.set_duty_hw(1024);

                Ratio::new::<ratio::ratio>(1.0f32)
            },
        }
}