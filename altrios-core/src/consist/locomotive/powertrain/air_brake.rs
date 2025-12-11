use super::*;

const TOL: f64 = 1e-3;
const PSI_TO_PASCAL: f64 = 6894.76;  // Conversion factor from PSI to Pascal

#[serde_api]
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq, StateMethods, SetCumulative)]
#[cfg_attr(feature = "pyo3", pyclass(module = "altrios", subclass, eq))]
/// Struct for modeling Air Brake System for locomotive
pub struct AirBrake {
    #[serde(default)]
    /// struct for tracking current state
    pub state: AirBrakeState,
    /// Reservoir volume in cubic meters
    pub reservoir_volume: si::Volume,
    /// Orifice diameter for leak modeling (meters)
    pub orifice_diameter: si::Length,
    /// Compressor displacement in cubic meters per second (stored as f64)
    pub compressor_displacement: f64,
    /// Compressor efficiency (0 to 1)
    pub compressor_efficiency: si::Ratio,
    /// Upper pressure limit for compressor control (Pascals)
    pub pressure_upper_limit: si::Pressure,
    /// Lower pressure limit for compressor control (Pascals)
    pub pressure_lower_limit: si::Pressure,
    /// time step interval between saves. 1 is a good option. If None, no saving occurs.
    pub save_interval: Option<usize>,
    /// Custom vector of [Self::state]
    #[serde(default)]
    pub history: AirBrakeStateHistoryVec,
}

#[pyo3_api]
impl AirBrake {
    #[getter("reservoir_volume_m3")]
    fn get_reservoir_volume_py(&self) -> f64 {
        self.reservoir_volume.get::<si::cubic_meter>()
    }

    #[getter("orifice_diameter_m")]
    fn get_orifice_diameter_py(&self) -> f64 {
        self.orifice_diameter.get::<si::meter>()
    }

    #[getter("compressor_displacement_m3_per_s")]
    fn get_compressor_displacement_py(&self) -> f64 {
        self.compressor_displacement
    }

    #[staticmethod]
    #[pyo3(name = "default")]
    fn default_py() -> Self {
        Self::default()
    }
}

impl Default for AirBrake {
    fn default() -> Self {
        Self {
            state: Default::default(),
            // Default values based on typical locomotive air brake system
            reservoir_volume: 1.0 * uc::M3,  // 1 cubic meter reservoir
            orifice_diameter: 0.001 * uc::M,  // 1mm leak orifice
            compressor_displacement: 0.01,  // ~10 L/s (m³/s)
            compressor_efficiency: 0.85 * uc::R,  // 85% efficiency
            pressure_upper_limit: 100.0 * PSI_TO_PASCAL * uc::PASCAL,  // 100 PSI
            pressure_lower_limit: 90.0 * PSI_TO_PASCAL * uc::PASCAL,   // 90 PSI
            save_interval: None,
            history: Default::default(),
        }
    }
}

impl Init for AirBrake {
    fn init(&mut self) -> Result<(), Error> {
        self.state.init()?;
        Ok(())
    }
}

impl SerdeAPI for AirBrake {}

// non-py methods
impl AirBrake {
    /// Calculate air brake dynamics for one time step
    /// 
    /// # Arguments
    /// - `dt`: time step size
    /// - `brake_demand`: brake pressure demand (0.0 to 1.0)
    pub fn solve_air_brake(
        &mut self,
        dt: si::Time,
        brake_demand: f64,
    ) -> anyhow::Result<()> {
        ensure!(
            dt > si::Time::ZERO,
            format!(
                "{}\n dt must always be greater than 0.0",
                format_dbg!(dt > si::Time::ZERO)
            )
        );

        ensure!(
            brake_demand >= 0.0 && brake_demand <= 1.0,
            format!(
                "{}\n brake_demand must be between 0.0 and 1.0",
                format_dbg!(brake_demand >= 0.0 && brake_demand <= 1.0)
            )
        );

        let current_pressure = *self.state.current_reservoir_pressure.get_stale(|| format_dbg!())?;

        // Calculate leak rate using orifice equation
        // Q = Cd * A * sqrt(2 * P / rho)
        // For 50 scfm at 100 psi specification
        // scfm = standard cubic feet per minute at 14.7 psi and 70°F
        // 50 scfm = 0.0236 m³/s at standard conditions
        let target_leak_scfm = 50.0;  // scfm at 100 psi
        let target_leak_m3_per_s = target_leak_scfm * 0.000471947;  // Convert scfm to m³/s
        let target_pressure_psi = 100.0;
        let target_pressure_pascal = target_pressure_psi * PSI_TO_PASCAL * uc::PASCAL;
        
        // Calculate leak at current pressure (proportional to sqrt(P))
        let pressure_ratio = if current_pressure > si::Pressure::ZERO {
            (current_pressure / target_pressure_pascal).get::<si::ratio>().sqrt()
        } else {
            0.0
        };
        let leak_rate_m3_per_s = target_leak_m3_per_s * pressure_ratio;

        // Determine compressor on/off based on hysteresis control
        let compressor_was_on = *self.state.compressor_on.get_stale(|| format_dbg!())?;
        let compressor_on = if compressor_was_on {
            // Turn off if pressure reaches upper limit
            current_pressure < self.pressure_upper_limit
        } else {
            // Turn on if pressure drops to lower limit
            current_pressure <= self.pressure_lower_limit
        };

        // Calculate compressor power if on
        let compressor_power = if compressor_on {
            // Power = (P * Q) / efficiency
            // Where P is pressure rise and Q is volume flow rate
            // Work with raw values: Pressure (Pa) * Volume (m³) / Time (s) = Power (W)
            let pressure_rise = self.pressure_upper_limit - current_pressure;
            let volume_rate = self.compressor_displacement;
            // Pressure * Volume/Time = Energy/Time = Power
            // We need to be careful with units here
            let power_raw = pressure_rise.value * volume_rate / self.compressor_efficiency.get::<si::ratio>();
            power_raw * uc::W
        } else {
            si::Power::ZERO
        };

        // Calculate net volume rate change
        let net_volume_rate = if compressor_on {
            self.compressor_displacement - leak_rate_m3_per_s
        } else {
            -leak_rate_m3_per_s
        };

        // Calculate pressure change using ideal gas law
        // dP/dt = (dV/dt) * (P / V)
        // For small time steps: dP = (dV/dt) * dt * (P / V)
        let volume_change = net_volume_rate * dt.get::<si::second>();
        let pressure_change = if self.reservoir_volume > si::Volume::ZERO {
            volume_change * (current_pressure / self.reservoir_volume).value * uc::PASCAL
        } else {
            si::Pressure::ZERO
        };

        let new_pressure = (current_pressure + pressure_change)
            .max(si::Pressure::ZERO)
            .min(self.pressure_upper_limit * 1.2);  // Allow 20% overpressure

        // Brake pipe pressure follows demand and reservoir pressure
        let max_brake_pressure = new_pressure * 0.9;  // Brake pipe typically ~90% of reservoir
        let brake_pipe_pressure = max_brake_pressure * brake_demand;

        // Calculate total compressed air delivered (in standard cubic feet)
        let air_delivered_m3 = if compressor_on {
            self.compressor_displacement * dt.get::<si::second>()
        } else {
            0.0
        };
        // Convert to scfm for tracking
        let air_delivered_scf = air_delivered_m3 / 0.0283168;  // m³ to cubic feet

        // Update states
        self.state.current_reservoir_pressure.update(new_pressure, || format_dbg!())?;
        self.state.brake_pipe_pressure.update(brake_pipe_pressure, || format_dbg!())?;
        self.state.compressor_on.update(compressor_on, || format_dbg!())?;
        self.state.current_compressor_power.update(compressor_power, || format_dbg!())?;
        self.state.total_compressed_air_scf.increment(air_delivered_scf, || format_dbg!())?;

        Ok(())
    }
}

#[serde_api]
#[derive(
    Clone, Debug, Deserialize, Serialize, PartialEq, HistoryVec, StateMethods, SetCumulative,
)]
#[cfg_attr(feature = "pyo3", pyclass(module = "altrios", subclass, eq))]
pub struct AirBrakeState {
    /// iteration counter
    pub i: TrackedState<usize>,
    /// Current compressor power draw
    pub current_compressor_power: TrackedState<si::Power>,
    /// Current reservoir pressure
    pub current_reservoir_pressure: TrackedState<si::Pressure>,
    /// Brake pipe pressure
    pub brake_pipe_pressure: TrackedState<si::Pressure>,
    /// Compressor on/off state
    pub compressor_on: TrackedState<bool>,
    /// Total compressor energy consumed
    pub total_compressor_energy: TrackedState<si::Energy>,
    /// Total compressed air delivered in standard cubic feet
    pub total_compressed_air_scf: TrackedState<f64>,
}

#[pyo3_api]
impl AirBrakeState {}

impl Init for AirBrakeState {}
impl SerdeAPI for AirBrakeState {}

impl Default for AirBrakeState {
    fn default() -> Self {
        Self {
            i: Default::default(),
            current_compressor_power: Default::default(),
            current_reservoir_pressure: Default::default(),
            brake_pipe_pressure: Default::default(),
            compressor_on: TrackedState::new(false),
            total_compressor_energy: Default::default(),
            total_compressed_air_scf: Default::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_air_brake() -> AirBrake {
        AirBrake {
            reservoir_volume: 1.0 * uc::M3,
            orifice_diameter: 0.001 * uc::M,
            compressor_displacement: 0.01,  // m³/s
            compressor_efficiency: 0.85 * uc::R,
            pressure_upper_limit: 100.0 * PSI_TO_PASCAL * uc::PASCAL,
            pressure_lower_limit: 90.0 * PSI_TO_PASCAL * uc::PASCAL,
            save_interval: None,
            ..Default::default()
        }
    }

    #[test]
    fn test_air_brake_initialization() {
        let mut ab = test_air_brake();
        ab.init().unwrap();
        ab.check_and_reset(|| format_dbg!()).unwrap();
        // After init, pressure starts at zero, so compressor should turn on
        ab.solve_air_brake(uc::S * 1.0, 0.0).unwrap();
        assert!(*ab.state.compressor_on.get_fresh(|| format_dbg!()).unwrap());
    }

    #[test]
    fn test_compressor_turns_on() {
        let mut ab = test_air_brake();
        ab.init().unwrap();
        ab.check_and_reset(|| format_dbg!()).unwrap();
        
        // Compressor should turn on when pressure is at lower limit
        ab.solve_air_brake(uc::S * 1.0, 0.0).unwrap();
        assert!(*ab.state.compressor_on.get_fresh(|| format_dbg!()).unwrap());
    }

    #[test]
    fn test_default() {
        let _ab = AirBrake::default();
    }
}
