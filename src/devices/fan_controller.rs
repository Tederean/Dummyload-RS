use embedded_hal::pwm::SetDutyCycle;
use esp_hal::gpio::{AnyPin, InputOutputAnalogPinType, Unknown};
use esp_hal::ledc::channel::Channel;
use esp_hal::ledc::HighSpeed;
use refinement::{Predicate, Refinement};

pub struct RatioPredicate;

impl Predicate<uom::si::f32::Ratio> for RatioPredicate {
    fn test(x: &uom::si::f32::Ratio) -> bool {
        let percent = x.get::<uom::si::ratio::percent>();

        percent.is_finite() && percent >= 30.0f32 && percent <= 100.0f32
    }
}

pub type Ratio = Refinement<uom::si::f32::Ratio, RatioPredicate>;

pub struct FanController<'a> {
    pwm_channel: Channel<'a, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>,
    fan_speed: Option<uom::si::f32::Frequency>,
    fan_power: Ratio,
}

impl<'a> FanController<'a> {
    pub fn new(pwm_channel: Channel<'a, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>) -> FanController {
        let full_power = Ratio::new(uom::si::f32::Ratio::new::<uom::si::ratio::ratio>(1.0f32)).unwrap();

        let mut fan_controller = FanController {
            pwm_channel,
            fan_speed: None,
            fan_power: full_power,
        };

        fan_controller.set_fan_power(full_power);

        fan_controller
    }

    pub fn set_fan_power(&mut self, fan_power: Ratio) {
        let percent = fan_power.get::<uom::si::ratio::percent>() as u8;

        self.pwm_channel.set_duty_cycle_percent(percent).unwrap();

        self.fan_power = fan_power;
    }

    pub fn get_fan_power(&self) -> Ratio {
        self.fan_power
    }

    pub fn get_fan_speed(&self) -> Option<uom::si::f32::Frequency> {
        self.fan_speed
    }

    pub fn measure_fan_speed(&self) -> Result<(), ()> {
        todo!()
    }
}
