use core::fmt::Error;
use esp32_hal::gpio::{
    Bank1GpioRegisterAccess, DualCoreInteruptStatusRegisterAccessBank1, Gpio33Signals,
    Gpio34Signals, InputOnlyAnalogPinType, InputOutputAnalogPinType,
};
use esp32_hal::prelude::*;
use esp32_hal::{
    gpio,
    gpio::GpioPin,
    ledc::{
        channel::{self, ChannelIFace},
        timer::{self, TimerIFace},
        HighSpeed, LEDC,
    },
};

type FanPwmPin = GpioPin<
    gpio::Unknown,
    Bank1GpioRegisterAccess,
    DualCoreInteruptStatusRegisterAccessBank1,
    InputOutputAnalogPinType,
    Gpio33Signals,
    33,
>;
type FanRpmPin = GpioPin<
    gpio::Unknown,
    Bank1GpioRegisterAccess,
    DualCoreInteruptStatusRegisterAccessBank1,
    InputOnlyAnalogPinType,
    Gpio34Signals,
    34,
>;

type PwmChannel<'a> = esp32_hal::ledc::channel::Channel<'a, HighSpeed, GpioPin<esp32_hal::gpio::Unknown, Bank1GpioRegisterAccess, DualCoreInteruptStatusRegisterAccessBank1, InputOutputAnalogPinType, Gpio33Signals, 33>>;

#[derive(Debug, Copy, Clone)]
pub struct FanPower {
    percent: u8,
}

impl FanPower {
    pub fn new(percent: u8) -> Option<FanPower> {
        match percent {
            30..=100 => Some(FanPower { percent }),
            _ => None,
        }
    }

    pub fn get_percent(&self) -> u8 {
        self.percent
    }

    pub fn get_duty_cycle(&self) -> u8 {
        let ratio = (self.percent as f32) / 100.0_f32;

        ((ratio * 255.0_f32) + 0.5_f32) as u8
    }
}

#[derive(Debug, Copy, Clone)]
pub struct FanSpeed {
    rpm: u16,
}

impl FanSpeed {
    pub fn new(rpm: u16) -> FanSpeed {
        FanSpeed { rpm }
    }

    pub fn get_rpm(&self) -> u16 {
        self.rpm
    }
}

pub struct FanDevice {
    fan_power: FanPower,
    //pwm_channel: PwmChannel<'d>,
    //hstimer0: esp32_hal::ledc::timer::Timer<'d, HighSpeed>,
    //ledc: LEDC<'d>,
}

impl FanDevice {
    pub fn new(
        fan_pwm_pin: FanPwmPin,
        fan_rpm_pin: FanRpmPin,
        ledc: LEDC,
    ) -> FanDevice {
        let mut hstimer0 = ledc.get_timer::<HighSpeed>(timer::Number::Timer0);

        hstimer0.configure(timer::config::Config {
            duty: timer::config::Duty::Duty8Bit,
            clock_source: timer::HSClockSource::APBClk,
            frequency: 25u32.kHz(),
        }).unwrap();

        let mut pwm_channel = ledc.get_channel(channel::Number::Channel0, fan_pwm_pin);

        pwm_channel.configure(channel::config::Config {
            timer: &hstimer0,
            duty_pct: 10,
            pin_config: channel::config::PinConfig::PushPull,
        }).unwrap();

        let fan_power = FanPower::new(100_u8).unwrap();
        let mut fan_device = FanDevice {
            fan_power,
            //pwm_channel,
            //ledc,
            //hstimer0,
        };

        fan_device.set_fan_power(fan_power);
        fan_device
    }

    pub fn set_fan_power(&mut self, fan_power: FanPower) {
        self.fan_power = fan_power;
    }

    pub fn get_fan_power(&self) -> FanPower {
        self.fan_power
    }

    pub fn get_fan_speed(&self) -> FanSpeed {
        FanSpeed::new(0)
    }
}
