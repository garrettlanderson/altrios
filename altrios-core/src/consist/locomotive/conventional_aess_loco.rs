use super::powertrain::air_brake::AirBrake;
use super::powertrain::electric_drivetrain::ElectricDrivetrain;
use super::powertrain::fuel_converter::FuelConverter;
use super::powertrain::generator::Generator;
use super::powertrain::reversible_energy_storage::ReversibleEnergyStorage;
use super::powertrain::ElectricMachine;
use super::LocoTrait;
use super::*;
use crate::imports::*;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, StateMethods, SetCumulative)]
#[serde_api]
#[cfg_attr(feature = "pyo3", pyclass(module = "altrios", subclass, eq))]
/// Conventional AESS (Automatic Engine Start-Stop) locomotive with starter battery and air brake
pub struct ConventionalAESSLoco {
    #[has_state]
    pub fc: FuelConverter,
    #[has_state]
    pub gen: Generator,
    #[has_state]
    pub edrv: ElectricDrivetrain,
    #[has_state]
    pub starter_battery: ReversibleEnergyStorage,
    #[has_state]
    pub air_brake: AirBrake,
    #[has_state]
    pub aess_controller: AESSController,
}

#[pyo3_api]
impl ConventionalAESSLoco {
    #[new]
    pub fn __new__(
        fuel_converter: FuelConverter,
        generator: Generator,
        electric_drivetrain: ElectricDrivetrain,
        starter_battery: ReversibleEnergyStorage,
        air_brake: AirBrake,
        aess_controller: AESSController,
    ) -> Self {
        Self {
            fc: fuel_converter,
            gen: generator,
            edrv: electric_drivetrain,
            starter_battery,
            air_brake,
            aess_controller,
        }
    }
}

impl ConventionalAESSLoco {
    pub fn new(
        fuel_converter: FuelConverter,
        generator: Generator,
        electric_drivetrain: ElectricDrivetrain,
        starter_battery: ReversibleEnergyStorage,
        air_brake: AirBrake,
        aess_controller: AESSController,
    ) -> Self {
        Self {
            fc: fuel_converter,
            gen: generator,
            edrv: electric_drivetrain,
            starter_battery,
            air_brake,
            aess_controller,
        }
    }

    /// Solve energy consumption for AESS locomotive
    /// # Arguments
    /// - `pwr_out_req`: power required at the wheel/rail interface
    /// - `dt`: time step size
    /// - `pwr_aux`: power demand for auxiliary systems
    /// - `assert_limits`: whether to fail when powertrain capabilities are exceeded
    /// - `brake_demand`: brake demand fraction (0.0 to 1.0)
    pub fn solve_energy_consumption(
        &mut self,
        pwr_out_req: si::Power,
        dt: si::Time,
        pwr_aux: si::Power,
        assert_limits: bool,
        brake_demand: f64,
    ) -> anyhow::Result<()> {
        // Update air brake state
        self.air_brake.solve_air_brake(dt, brake_demand)?;

        // Get current state information
        let battery_soc = *self.starter_battery.state.soc.get_stale(|| format_dbg!())?;
        let engine_temp = *self.fc.state.current_temp.get_stale(|| format_dbg!())?;
        let compressor_on = *self.air_brake.state.compressor_on.get_stale(|| format_dbg!())?;

        // Update AESS controller to determine if engine should be on
        let engine_on = self.aess_controller.update_engine_state(
            dt,
            battery_soc,
            engine_temp,
            compressor_on,
            pwr_out_req,
        )?;

        // Calculate starter battery power draw/charge
        let battery_constant_load = self.aess_controller.battery_constant_load;
        let compressor_power = *self.air_brake.state.pwr_compressor.get_fresh(|| format_dbg!())?;
        
        // Battery provides constant load and compressor power when engine is off
        // Battery charges when engine is on
        let battery_power = if engine_on {
            // Engine is on - charge battery
            -self.aess_controller.battery_charge_power
        } else {
            // Engine is off - battery provides constant load and compressor
            battery_constant_load + compressor_power
        };

        // Solve starter battery power
        // RES takes pwr_prop_req and pwr_aux_req separately
        // For starter battery, all power is aux load
        self.starter_battery.solve_energy_consumption(
            si::Power::ZERO,  // pwr_prop_req
            battery_power,     // pwr_aux_req
            dt,
        )?;

        // Solve main powertrain
        self.edrv.set_pwr_in_req(pwr_out_req, dt)?;

        self.gen.set_pwr_in_req(
            *self
                .edrv
                .state
                .pwr_elec_prop_in
                .get_fresh(|| format_dbg!())?,
            pwr_aux,
            engine_on,
            dt,
        )?;

        self.fc.solve_energy_consumption(
            *self.gen.state.pwr_mech_in.get_fresh(|| format_dbg!())?,
            dt,
            engine_on,
            assert_limits,
        )?;

        Ok(())
    }
}

impl Mass for ConventionalAESSLoco {
    fn mass(&self) -> anyhow::Result<Option<si::Mass>> {
        self.derived_mass().with_context(|| format_dbg!())
    }

    fn set_mass(
        &mut self,
        _new_mass: Option<si::Mass>,
        _side_effect: MassSideEffect,
    ) -> anyhow::Result<()> {
        Err(anyhow!(
            "`set_mass` not enabled for {}",
            stringify!(ConventionalAESSLoco)
        ))
    }

    fn derived_mass(&self) -> anyhow::Result<Option<si::Mass>> {
        let fc_mass = self.fc.mass().with_context(|| format_dbg!())?;
        let battery_mass = self.starter_battery.mass().with_context(|| format_dbg!())?;
        
        match (fc_mass, battery_mass) {
            (Some(fc), Some(bat)) => Ok(Some(fc + bat)),
            (Some(fc), None) => Ok(Some(fc)),
            (None, Some(bat)) => Ok(Some(bat)),
            (None, None) => Ok(None),
        }
    }

    fn expunge_mass_fields(&mut self) {
        self.fc.expunge_mass_fields();
        self.gen.expunge_mass_fields();
        self.starter_battery.expunge_mass_fields();
    }
}

impl Init for ConventionalAESSLoco {
    fn init(&mut self) -> Result<(), Error> {
        self.fc.init()?;
        self.gen.init()?;
        self.edrv.init()?;
        self.starter_battery.init()?;
        self.air_brake.init()?;
        self.aess_controller.init()?;
        Ok(())
    }
}

impl SerdeAPI for ConventionalAESSLoco {}

impl LocoTrait for ConventionalAESSLoco {
    fn set_curr_pwr_max_out(
        &mut self,
        pwr_aux: Option<si::Power>,
        elev_and_temp: Option<(si::Length, si::ThermodynamicTemperature)>,
        _train_mass: Option<si::Mass>,
        _train_speed: Option<si::Velocity>,
        dt: si::Time,
    ) -> anyhow::Result<()> {
        self.fc.set_cur_pwr_out_max(elev_and_temp, dt)?;
        self.gen.set_cur_pwr_max_out(
            *self.fc.state.pwr_out_max.get_fresh(|| format_dbg!())?,
            Some(pwr_aux.with_context(|| format_dbg!("`pwr_aux` not provided"))?),
        )?;
        self.edrv.set_cur_pwr_max_out(
            *self
                .gen
                .state
                .pwr_elec_prop_out_max
                .get_fresh(|| format_dbg!())?,
            None,
        )?;
        self.edrv.set_cur_pwr_regen_max(si::Power::ZERO)?;
        self.gen
            .set_pwr_rate_out_max(self.fc.pwr_out_max / self.fc.pwr_ramp_lag)?;
        self.edrv.set_pwr_rate_out_max(
            *self
                .gen
                .state
                .pwr_rate_out_max
                .get_fresh(|| format_dbg!())?,
        )?;
        Ok(())
    }

    fn get_energy_loss(&self) -> anyhow::Result<si::Energy> {
        Ok(*self.fc.state.energy_loss.get_stale(|| format_dbg!())?
            + *self.gen.state.energy_loss.get_stale(|| format_dbg!())?
            + *self.edrv.state.energy_loss.get_stale(|| format_dbg!())?
            + *self.starter_battery.state.energy_loss.get_stale(|| format_dbg!())?)
    }
}

/// AESS Controller for automatic engine start-stop
#[serde_api]
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, StateMethods, SetCumulative)]
#[cfg_attr(feature = "pyo3", pyclass(module = "altrios", subclass, eq))]
pub struct AESSController {
    #[serde(default)]
    pub state: AESSControllerState,
    /// Low SOC threshold - engine starts if battery SOC drops below this (default 0.5)
    pub soc_low_threshold: si::Ratio,
    /// High SOC threshold - engine may turn off if SOC is above this (default 0.95)
    pub soc_high_threshold: si::Ratio,
    /// Low temperature threshold - engine starts if temperature drops below this (default 120°F = 48.89°C)
    pub temp_low_threshold: si::ThermodynamicTemperature,
    /// High temperature threshold - engine may turn off if temperature is above this (default 180°F = 82.22°C)
    pub temp_high_threshold: si::ThermodynamicTemperature,
    /// Duration all shutdown conditions must be met before turning off engine (default 600s = 10 min)
    pub shutdown_delay: si::Time,
    /// Battery constant load (power draw when engine is off)
    pub battery_constant_load: si::Power,
    /// Battery charge power (when engine is on)
    pub battery_charge_power: si::Power,
    /// time step interval between saves
    pub save_interval: Option<usize>,
    #[serde(default)]
    pub history: AESSControllerStateHistoryVec,
}

#[pyo3_api]
impl AESSController {
    #[new]
    #[pyo3(signature = (
        soc_low_threshold=0.5,
        soc_high_threshold=0.95,
        temp_low_threshold_celsius=48.89,
        temp_high_threshold_celsius=82.22,
        shutdown_delay_seconds=600.0,
        battery_constant_load_watts=500.0,
        battery_charge_power_watts=10000.0,
        save_interval=None,
    ))]
    pub fn __new__(
        soc_low_threshold: f64,
        soc_high_threshold: f64,
        temp_low_threshold_celsius: f64,
        temp_high_threshold_celsius: f64,
        shutdown_delay_seconds: f64,
        battery_constant_load_watts: f64,
        battery_charge_power_watts: f64,
        save_interval: Option<usize>,
    ) -> Self {
        Self {
            state: Default::default(),
            soc_low_threshold: soc_low_threshold * uc::R,
            soc_high_threshold: soc_high_threshold * uc::R,
            temp_low_threshold: si::ThermodynamicTemperature::new::<si::kelvin>(
                temp_low_threshold_celsius + uc::CELSIUS_TO_KELVIN,
            ),
            temp_high_threshold: si::ThermodynamicTemperature::new::<si::kelvin>(
                temp_high_threshold_celsius + uc::CELSIUS_TO_KELVIN,
            ),
            shutdown_delay: shutdown_delay_seconds * uc::S,
            battery_constant_load: battery_constant_load_watts * uc::W,
            battery_charge_power: battery_charge_power_watts * uc::W,
            save_interval,
            history: Default::default(),
        }
    }
}

impl Default for AESSController {
    fn default() -> Self {
        Self {
            state: Default::default(),
            soc_low_threshold: 0.5 * uc::R,
            soc_high_threshold: 0.95 * uc::R,
            temp_low_threshold: si::ThermodynamicTemperature::new::<si::kelvin>(48.89 + uc::CELSIUS_TO_KELVIN),
            temp_high_threshold: si::ThermodynamicTemperature::new::<si::kelvin>(82.22 + uc::CELSIUS_TO_KELVIN),
            shutdown_delay: 600.0 * uc::S,
            battery_constant_load: 500.0 * uc::W,
            battery_charge_power: 10000.0 * uc::W,
            save_interval: None,
            history: Default::default(),
        }
    }
}

impl Init for AESSController {
    fn init(&mut self) -> Result<(), Error> {
        self.state.init()?;
        Ok(())
    }
}

impl SerdeAPI for AESSController {}

impl AESSController {
    /// Update engine state based on AESS logic
    pub fn update_engine_state(
        &mut self,
        dt: si::Time,
        battery_soc: si::Ratio,
        engine_temp: si::ThermodynamicTemperature,
        compressor_on: bool,
        tractive_power: si::Power,
    ) -> anyhow::Result<bool> {
        let current_engine_on = *self.state.engine_on.get_stale(|| format_dbg!())?;

        // Check start conditions
        let should_start = battery_soc < self.soc_low_threshold
            || engine_temp < self.temp_low_threshold
            || compressor_on;

        // Check shutdown conditions (all must be met)
        let can_shutdown = battery_soc > self.soc_high_threshold
            && engine_temp > self.temp_high_threshold
            && !compressor_on
            && tractive_power == si::Power::ZERO;

        let engine_on = if should_start {
            // Start engine immediately if any start condition is met
            self.state.shutdown_timer.update(si::Time::ZERO, || format_dbg!())?;
            true
        } else if can_shutdown {
            // Increment shutdown timer if all conditions met
            self.state.shutdown_timer.increment(dt, || format_dbg!())?;
            
            // Only turn off if timer exceeds delay
            if *self.state.shutdown_timer.get_fresh(|| format_dbg!())? >= self.shutdown_delay {
                false
            } else {
                current_engine_on
            }
        } else {
            // Reset timer if conditions not met
            self.state.shutdown_timer.update(si::Time::ZERO, || format_dbg!())?;
            current_engine_on
        };

        self.state.engine_on.update(engine_on, || format_dbg!())?;
        self.state.should_start.update(should_start, || format_dbg!())?;
        self.state.can_shutdown.update(can_shutdown, || format_dbg!())?;

        Ok(engine_on)
    }
}

#[serde_api]
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, HistoryVec, StateMethods, SetCumulative)]
#[cfg_attr(feature = "pyo3", pyclass(module = "altrios", subclass, eq))]
pub struct AESSControllerState {
    pub i: TrackedState<usize>,
    /// Current engine on/off state
    pub engine_on: TrackedState<bool>,
    /// Time accumulator for shutdown conditions
    pub shutdown_timer: TrackedState<si::Time>,
    /// Flag indicating if start conditions are met
    pub should_start: TrackedState<bool>,
    /// Flag indicating if shutdown conditions are met
    pub can_shutdown: TrackedState<bool>,
}

#[pyo3_api]
impl AESSControllerState {}

impl Init for AESSControllerState {}
impl SerdeAPI for AESSControllerState {}

impl Default for AESSControllerState {
    fn default() -> Self {
        Self {
            i: Default::default(),
            engine_on: TrackedState::new(true),
            shutdown_timer: Default::default(),
            should_start: TrackedState::new(false),
            can_shutdown: TrackedState::new(false),
        }
    }
}
