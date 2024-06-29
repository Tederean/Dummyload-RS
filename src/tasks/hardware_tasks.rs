
#[embassy_executor::task]
pub async fn fan_controller_task(
    //pwm_channel: Channel<'_, HighSpeed, AnyPin<Unknown, InputOutputAnalogPinType>>,
    //channel: &'static PriorityChannel<CriticalSectionRawMutex, ChannelEntry, Max, 20>
    ) {
    let full_power = uom::si::f32::Ratio::new::<uom::si::ratio::ratio>(1.0f32);

    //loop {
        //fn set_fan_power(&mut self, fan_power: FanPowerRatio) {
        //    let new_percent = fan_power.get::<uom::si::ratio::percent>() as u8;
        //    let old_percent = self.fan_power.get::<uom::si::ratio::percent>() as u8;
//
        //    if old_percent != new_percent {
        //        self.pwm_channel.set_duty_cycle_percent(new_percent).unwrap();
        //        self.fan_power = fan_power;
//
       //         self.channel.try_send(ChannelEntry::FanPowerChanged(fan_power)).unwrap();
       //     }
       // }
    //}
}





