#[derive(Debug, Copy, Clone)]
pub struct Voltage {
    volt: f32,
}

impl Voltage {
    pub fn new(volt: f32) -> Option<Voltage> {
        if volt.is_finite() && volt.is_sign_positive() {
            Some(Voltage { volt });
        }

        return None;
    }

    pub fn get_volt(&self) -> f32 {
        self.volt
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Current {
    amps: f32,
}

impl Current {
    pub fn new(amps: f32) -> Option<Current> {
        if amps.is_finite() && amps.is_sign_positive() {
            Some(Current { amps });
        }

        return None;
    }

    pub fn get_amps(&self) -> f32 {
        self.amps
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Power {
    watt: f32,
}

impl Power {
    pub fn new(watt: f32) -> Option<Power> {
        if watt.is_finite() && watt.is_sign_positive() {
            Some(Power { watt });
        }

        return None;
    }

    pub fn get_watt(&self) -> f32 {
        self.watt
    }
}

#[derive(Debug, Copy, Clone)]
pub struct Temperature {
    degree_celsius: f32,
}

impl Temperature {
    pub fn new(degree_celsius: f32) -> Option<Temperature> {
        if degree_celsius.is_finite() && degree_celsius >= -273.15_f32 {
            Some(Temperature { degree_celsius });
        }

        return None;
    }

    pub fn get_degree_celsius(&self) -> f32 {
        self.degree_celsius
    }
}

#[derive(Debug, Copy, Clone)]
pub struct VoltageMinimum {
    volt: f32,
}

impl VoltageMinimum {
    pub fn new(volt: f32) -> Option<VoltageMinimum> {
        if volt.is_finite() && volt >= 4.5_f32 && volt <= 33.0_f32 {
            Some(VoltageMinimum { volt });
        }

        return None;
    }

    pub fn get_volt(&self) -> f32 {
        self.volt
    }
}

#[derive(Debug, Copy, Clone)]
pub struct TargetCurrent {
    amps: f32,
}

impl TargetCurrent {
    pub fn new(amps: f32) -> Option<TargetCurrent> {
        if amps.is_finite() && amps.is_sign_positive() && amps <= 20.0_f32 {
            Some(TargetCurrent { amps });
        }

        return None;
    }

    pub fn get_amps(&self) -> f32 {
        self.amps
    }
}

#[derive(Debug, Copy, Clone)]
pub struct PowerMaximum {
    watt: f32,
}

impl PowerMaximum {
    pub fn new(watt: f32) -> Option<PowerMaximum> {
        if watt.is_finite() && watt >= 1.0_f32 && watt <= 205.0_f32 {
            Some(PowerMaximum { watt });
        }

        return None;
    }

    pub fn get_watt(&self) -> f32 {
        self.watt
    }
}
