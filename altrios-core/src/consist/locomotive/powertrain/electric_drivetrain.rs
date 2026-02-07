use super::*;
use crate::consist::locomotive::powertrain::ElectricMachine;
use crate::imports::*;
#[cfg(feature = "pyo3")]
use crate::pyo3::*;

#[serde_api]
#[derive(Deserialize, Serialize, Debug, Clone, PartialEq, StateMethods, SetCumulative)]
#[cfg_attr(feature = "pyo3", pyclass(module = "altrios", subclass, eq))]
/// Struct for modeling electric drivetrain.  This includes power electronics, motor, axle ...
/// everything involved in converting high voltage electrical power to force exerted by the wheel on the track.  
pub struct ElectricDrivetrain {
    #[serde(default)]
    /// struct for tracking current state
    pub state: ElectricDrivetrainState,
    /// Shaft output power fraction array at which efficiencies are evaluated.
    pub pwr_out_frac_interp: Vec<f64>,

    /// Efficiency array corresponding to [Self::pwr_out_frac_interp] and [Self::pwr_in_frac_interp]
    pub eta_interp: Vec<f64>,
    /// Electrical input power fraction array at which efficiencies are evaluated.
    /// Calculated during runtime if not provided.
    #[serde(skip)]
    pub pwr_in_frac_interp: Vec<f64>,
    /// ElectricDrivetrain maximum output power assuming that positive and negative tractive powers have same magnitude
    pub pwr_out_max: si::Power,
    /// Optional minimum output power (negative for regen). When `None`, defaults to `-pwr_out_max`
    /// for symmetric behavior. Set to a negative value to allow asymmetric power limits.
    #[serde(default)]
    pub pwr_out_min: Option<si::Power>,
    // TODO: add `mass` here
    /// Time step interval between saves. 1 is a good option. If None, no saving occurs.
    pub save_interval: Option<usize>,
    /// Custom vector of [Self::state] haha
    #[serde(default)]
    pub history: ElectricDrivetrainStateHistoryVec,
}

#[pyo3_api]
impl ElectricDrivetrain {
    #[new]
    #[pyo3(signature = (pwr_out_frac_interp, eta_interp, pwr_out_max_watts, pwr_out_min_watts=None, save_interval=None))]
    fn __new__(
        pwr_out_frac_interp: Vec<f64>,
        eta_interp: Vec<f64>,
        pwr_out_max_watts: f64,
        pwr_out_min_watts: Option<f64>,
        save_interval: Option<usize>,
    ) -> anyhow::Result<Self> {
        Self::new(
            pwr_out_frac_interp,
            eta_interp,
            pwr_out_max_watts,
            pwr_out_min_watts,
            save_interval,
        )
    }

    #[staticmethod]
    #[pyo3(name = "default")]
    fn default_py() -> Self {
        Self::default()
    }

    #[getter("eta_max")]
    fn get_eta_max_py(&self) -> f64 {
        self.get_eta_max()
    }

    #[setter("__eta_max")]
    fn set_eta_max_py(&mut self, eta_max: f64) -> anyhow::Result<()> {
        Ok(self.set_eta_max(eta_max).map_err(PyValueError::new_err)?)
    }

    #[getter("eta_min")]
    fn get_eta_min_py(&self) -> f64 {
        self.get_eta_min()
    }

    #[getter("eta_range")]
    fn get_eta_range_py(&self) -> f64 {
        self.get_eta_range()
    }

    #[setter("__eta_range")]
    fn set_eta_range_py(&mut self, eta_range: f64) -> anyhow::Result<()> {
        Ok(self
            .set_eta_range(eta_range)
            .map_err(PyValueError::new_err)?)
    }
}

impl ElectricDrivetrain {
    pub fn new(
        pwr_out_frac_interp: Vec<f64>,
        eta_interp: Vec<f64>,
        pwr_out_max_watts: f64,
        pwr_out_min_watts: Option<f64>,
        save_interval: Option<usize>,
    ) -> anyhow::Result<Self> {
        ensure!(
            eta_interp.len() == pwr_out_frac_interp.len(),
            format!(
                "{}\nedrv eta_interp and pwr_out_frac_interp must be the same length",
                eta_interp.len() == pwr_out_frac_interp.len()
            )
        );

        ensure!(
            pwr_out_frac_interp.iter().all(|x| *x >= 0.0),
            format!(
                "{}\nedrv pwr_out_frac_interp must be non-negative",
                format_dbg!(pwr_out_frac_interp.iter().all(|x| *x >= 0.0))
            )
        );

        ensure!(
            pwr_out_frac_interp.iter().all(|x| *x <= 1.0),
            format!(
                "{}\nedrv pwr_out_frac_interp must be less than or equal to 1.0",
                format_dbg!(pwr_out_frac_interp.iter().all(|x| *x <= 1.0))
            )
        );

        let pwr_out_min = pwr_out_min_watts.map(|w| uc::W * w);
        if let Some(min) = pwr_out_min {
            ensure!(
                min <= si::Power::ZERO,
                format!(
                    "{}\nedrv pwr_out_min ({:.6} MW) must be non-positive",
                    format_dbg!(min <= si::Power::ZERO),
                    min.get::<si::megawatt>()
                )
            );
        }

        let history = ElectricDrivetrainStateHistoryVec::new();
        let pwr_out_max_watts = uc::W * pwr_out_max_watts;
        let state = ElectricDrivetrainState::default();

        let mut edrv = ElectricDrivetrain {
            state,
            pwr_out_frac_interp,
            eta_interp,
            pwr_in_frac_interp: Vec::new(),
            pwr_out_max: pwr_out_max_watts,
            pwr_out_min,
            save_interval,
            history,
        };
        edrv.set_pwr_in_frac_interp()?;
        Ok(edrv)
    }

    /// Returns the resolved minimum output power. If `pwr_out_min` is `None`, returns `-pwr_out_max`
    /// (symmetric behavior).
    pub fn pwr_out_min_resolved(&self) -> si::Power {
        self.pwr_out_min.unwrap_or(-self.pwr_out_max)
    }

    /// Returns the absolute value (positive magnitude) of the resolved minimum output power.
    pub fn pwr_out_min_abs(&self) -> si::Power {
        -self.pwr_out_min_resolved()
    }

    /// Returns the power limit magnitude to use for normalization based on the sign of `pwr`.
    /// Positive power normalizes by `pwr_out_max`; negative power normalizes by `|pwr_out_min|`.
    fn pwr_limit_for(&self, pwr: si::Power) -> si::Power {
        if pwr >= si::Power::ZERO {
            self.pwr_out_max
        } else {
            self.pwr_out_min_abs()
        }
    }

    pub fn set_pwr_in_frac_interp(&mut self) -> anyhow::Result<()> {
        // make sure vector has been created
        self.pwr_in_frac_interp = self
            .pwr_out_frac_interp
            .iter()
            .zip(self.eta_interp.iter())
            .map(|(x, y)| x / y)
            .collect();
        // verify monotonicity
        ensure!(
            self.pwr_in_frac_interp.windows(2).all(|w| w[0] < w[1]),
            format!(
                "{}\nedrv pwr_in_frac_interp ({:?}) must be monotonically increasing",
                format_dbg!(self.pwr_in_frac_interp.windows(2).all(|w| w[0] < w[1])),
                self.pwr_in_frac_interp
            )
        );
        Ok(())
    }

    pub fn set_cur_pwr_regen_max(&mut self, pwr_max_regen_in: si::Power) -> anyhow::Result<()> {
        if self.pwr_in_frac_interp.is_empty() {
            self.set_pwr_in_frac_interp()?;
        }
        let regen_limit = self.pwr_out_min_abs();
        let eta = uc::R
            * interp1d(
                &(pwr_max_regen_in / regen_limit).get::<si::ratio>().abs(),
                &self.pwr_out_frac_interp,
                &self.eta_interp,
                false,
            )?;
        self.state
            .pwr_mech_regen_max
            .update((pwr_max_regen_in * eta).min(regen_limit), || format_dbg!())?;
        ensure!(*self.state.pwr_mech_regen_max.get_fresh(|| format_dbg!())? >= si::Power::ZERO);
        Ok(())
    }

    /// Set `pwr_in_req` required to achieve desired `pwr_out_req` with time step size `dt`.
    pub fn set_pwr_in_req(&mut self, pwr_out_req: si::Power, _dt: si::Time) -> anyhow::Result<()> {
        ensure!(
            almost_le_uom(&pwr_out_req, &self.pwr_out_max, None),
            format!(
                "{}\nedrv required power ({:.6} MW) exceeds static max power ({:.6} MW)",
                format_dbg!(pwr_out_req <= self.pwr_out_max),
                pwr_out_req.get::<si::megawatt>(),
                self.pwr_out_max.get::<si::megawatt>()
            ),
        );
        ensure!(
            almost_le_uom(&self.pwr_out_min_resolved(), &pwr_out_req, None),
            format!(
                "{}\nedrv required power ({:.6} MW) is below static min power ({:.6} MW)",
                format_dbg!(self.pwr_out_min_resolved() <= pwr_out_req),
                pwr_out_req.get::<si::megawatt>(),
                self.pwr_out_min_resolved().get::<si::megawatt>()
            ),
        );

        ensure!(
            almost_le_uom(
                &pwr_out_req,
                self.state.pwr_mech_out_max.get_fresh(|| format_dbg!())?,
                Some(1e-5)
            ),
            format!(
                "{}\nedrv required power ({:.6} MW) exceeds dynamic max power ({:.6} MW)",
                format_dbg!(
                    pwr_out_req.abs()
                        <= *self.state.pwr_mech_out_max.get_fresh(|| format_dbg!())?
                ),
                pwr_out_req.get::<si::megawatt>(),
                self.state
                    .pwr_mech_out_max
                    .get_fresh(|| format_dbg!())?
                    .get::<si::megawatt>()
            ),
        );

        self.state
            .pwr_out_req
            .update(pwr_out_req, || format_dbg!())?;

        let pwr_norm = self.pwr_limit_for(pwr_out_req);
        self.state.eta.update(
            uc::R
                * interp1d(
                    &(pwr_out_req / pwr_norm).get::<si::ratio>().abs(),
                    &self.pwr_out_frac_interp,
                    &self.eta_interp,
                    false,
                )
                .with_context(|| format_dbg!())?,
            || format_dbg!(),
        )?;
        ensure!(
            *self.state.eta.get_fresh(|| format_dbg!())? >= 0.0 * uc::R
                || *self.state.eta.get_fresh(|| format_dbg!())? <= 1.0 * uc::R,
            format!(
                "{}\nedrv eta ({}) must be between 0 and 1",
                format_dbg!(
                    *self.state.eta.get_fresh(|| format_dbg!())? >= 0.0 * uc::R
                        || *self.state.eta.get_fresh(|| format_dbg!())? <= 1.0 * uc::R
                ),
                self.state
                    .eta
                    .get_fresh(|| format_dbg!())?
                    .get::<si::ratio>()
            )
        );

        // `pwr_mech_prop_out` is `pwr_out_req` unless `pwr_out_req` is more negative than `pwr_mech_regen_max`,
        // in which case, excess is handled by `pwr_mech_dyn_brake`
        self.state.pwr_mech_prop_out.update(
            pwr_out_req.max(-*self.state.pwr_mech_regen_max.get_fresh(|| format_dbg!())?),
            || format_dbg!(),
        )?;

        self.state.pwr_mech_dyn_brake.update(
            -(pwr_out_req - *self.state.pwr_mech_prop_out.get_fresh(|| format_dbg!())?),
            || format_dbg!(),
        )?;
        ensure!(
            *self.state.pwr_mech_dyn_brake.get_fresh(|| format_dbg!())? >= si::Power::ZERO,
            "Mech Dynamic Brake Power cannot be below 0.0"
        );

        // if pwr_out_req is negative, need to multiply by eta
        self.state.pwr_elec_prop_in.update(
            if pwr_out_req > si::Power::ZERO {
                *self.state.pwr_mech_prop_out.get_fresh(|| format_dbg!())?
                    / *self.state.eta.get_fresh(|| format_dbg!())?
            } else {
                *self.state.pwr_mech_prop_out.get_fresh(|| format_dbg!())?
                    * *self.state.eta.get_fresh(|| format_dbg!())?
            },
            || format_dbg!(),
        )?;

        self.state.pwr_elec_dyn_brake.update(
            *self.state.pwr_mech_dyn_brake.get_fresh(|| format_dbg!())?
                * *self.state.eta.get_fresh(|| format_dbg!())?,
            || format_dbg!(),
        )?;

        // loss does not account for dynamic braking
        self.state.pwr_loss.update(
            (*self.state.pwr_mech_prop_out.get_fresh(|| format_dbg!())?
                - *self.state.pwr_elec_prop_in.get_fresh(|| format_dbg!())?)
            .abs(),
            || format_dbg!(),
        )?;

        Ok(())
    }

    impl_get_set_eta_max_min!();
    impl_get_set_eta_range!();
}

// failed attempt at making path to default platform independent
// const EDRV_DEFAULT_PATH_STR: &'static str = include_str!(concat!(
//     env!("CARGO_MANIFEST_DIR"),
//     "/src/consist/locomotive/powertrain/electric_drivetrain.default.yaml"
// ));

impl Init for ElectricDrivetrain {
    fn init(&mut self) -> Result<(), Error> {
        self.state.init()?;
        Ok(())
    }
}
impl SerdeAPI for ElectricDrivetrain {}

impl Default for ElectricDrivetrain {
    fn default() -> Self {
        // let file_contents = include_str!(EDRV_DEFAULT_PATH_STR);
        let file_contents = include_str!("electric_drivetrain.default.yaml");
        let mut edrv = Self::from_yaml(file_contents, false).unwrap();
        edrv.init().unwrap();
        edrv
    }
}

impl ElectricMachine for ElectricDrivetrain {
    /// Set current max possible output power, `pwr_mech_out_max`,
    /// given `pwr_in_max` from upstream component.
    fn set_cur_pwr_max_out(
        &mut self,
        pwr_in_max: si::Power,
        pwr_aux: Option<si::Power>,
    ) -> anyhow::Result<()> {
        ensure!(pwr_aux.is_none(), format_dbg!(pwr_aux.is_none()));
        if self.pwr_in_frac_interp.is_empty() {
            self.set_pwr_in_frac_interp()?;
        }
        let eta = uc::R
            * interp1d(
                &(pwr_in_max / self.pwr_out_max).get::<si::ratio>().abs(),
                &self.pwr_in_frac_interp,
                &self.eta_interp,
                false,
            )?;

        self.state.pwr_mech_out_max.update(
            self.pwr_out_max.min(pwr_in_max * eta).max(si::Power::ZERO),
            || format_dbg!(),
        )?;
        Ok(())
    }

    /// Set current power out max ramp rate, `pwr_rate_out_max` given `pwr_rate_in_max`
    /// from upstream component.  
    fn set_pwr_rate_out_max(&mut self, pwr_rate_in_max: si::PowerRate) -> anyhow::Result<()> {
        self.state.pwr_rate_out_max.update(
            if *self.state.eta.get_stale(|| format_dbg!())? > si::Ratio::ZERO {
                pwr_rate_in_max * *self.state.eta.get_stale(|| format_dbg!())?
            } else {
                pwr_rate_in_max * uc::R * 1.0
            },
            || format_dbg!(),
        )?;
        Ok(())
    }
}

#[serde_api]
#[derive(
    Clone,
    Debug,
    Default,
    Deserialize,
    Serialize,
    PartialEq,
    HistoryVec,
    StateMethods,
    SetCumulative,
)]
#[cfg_attr(feature = "pyo3", pyclass(module = "altrios", subclass, eq))]
pub struct ElectricDrivetrainState {
    /// index
    pub i: TrackedState<usize>,
    /// Component efficiency based on current power demand.
    pub eta: TrackedState<si::Ratio>,
    // Component limits
    /// Maximum possible positive traction power.
    pub pwr_mech_out_max: TrackedState<si::Power>,
    /// Maximum possible regeneration power going to ReversibleEnergyStorage.
    pub pwr_mech_regen_max: TrackedState<si::Power>,
    /// max ramp-up rate
    pub pwr_rate_out_max: TrackedState<si::PowerRate>,

    // Current values
    /// Raw power requirement from boundary conditions
    pub pwr_out_req: TrackedState<si::Power>,
    /// Electrical power to propulsion from ReversibleEnergyStorage and Generator.
    /// negative value indicates regenerative braking
    pub pwr_elec_prop_in: TrackedState<si::Power>,
    /// Mechanical power to propulsion, corrected by efficiency, from ReversibleEnergyStorage and Generator.
    /// Negative value indicates regenerative braking.
    pub pwr_mech_prop_out: TrackedState<si::Power>,
    /// Mechanical power from dynamic braking.  Positive value indicates braking; this should be zero otherwise.
    pub pwr_mech_dyn_brake: TrackedState<si::Power>,
    /// Electrical power from dynamic braking, dissipated as heat.
    pub pwr_elec_dyn_brake: TrackedState<si::Power>,
    /// Power lost in regeneratively converting mechanical power to power that can be absorbed by the battery.
    pub pwr_loss: TrackedState<si::Power>,

    // Cumulative energy values
    /// cumulative mech energy in from fc
    pub energy_elec_prop_in: TrackedState<si::Energy>,
    /// cumulative elec energy out
    pub energy_mech_prop_out: TrackedState<si::Energy>,
    /// cumulative energy has lost due to imperfect efficiency
    /// Mechanical energy from dynamic braking.
    pub energy_mech_dyn_brake: TrackedState<si::Energy>,
    /// Electrical energy from dynamic braking, dissipated as heat.
    pub energy_elec_dyn_brake: TrackedState<si::Energy>,
    /// Cumulative energy lost in regeneratively converting mechanical power to power that can be absorbed by the battery.
    pub energy_loss: TrackedState<si::Energy>,
}

#[pyo3_api]
impl ElectricDrivetrainState {}

impl Init for ElectricDrivetrainState {}
impl SerdeAPI for ElectricDrivetrainState {}

#[cfg(test)]
mod tests {
    use super::*;
    fn test_edrv() -> ElectricDrivetrain {
        ElectricDrivetrain::new(vec![0.0, 1.0], vec![0.9, 0.8], 8e6, None, None).unwrap()
    }

    /// Verify that calling `step()` increments the state index `i` by one.
    #[test]
    fn test_that_i_increments() {
        let mut edrv = test_edrv();
        edrv.check_and_reset(|| format_dbg!()).unwrap();
        edrv.step(|| format_dbg!()).unwrap();
        assert_eq!(1, *edrv.state.i.get_fresh(|| format_dbg!()).unwrap());
    }

    /// Verify that `save_state` appends to history when `save_interval` is `Some(1)`.
    #[test]
    #[allow(clippy::field_reassign_with_default)]
    fn test_that_history_has_len_1() {
        let mut edrv: ElectricDrivetrain = ElectricDrivetrain::default();
        edrv.save_interval = Some(1);
        assert!(edrv.history.is_empty());
        edrv.save_state(|| format_dbg!()).unwrap();
        assert_eq!(1, edrv.history.len());
    }

    /// Verify that `save_state` does not append to history when `save_interval` is `None`.
    #[test]
    fn test_that_history_has_len_0() {
        let mut edrv: ElectricDrivetrain = ElectricDrivetrain::default();
        assert!(edrv.history.is_empty());
        edrv.save_state(|| format_dbg!()).unwrap();
        assert!(edrv.history.is_empty());
    }

    /// Verify that efficiency getters and setters (`eta_max`, `eta_min`, `eta_range`) work correctly.
    #[test]
    fn test_get_and_set_eta() {
        let mut res = test_edrv();
        let eta_max = 0.9;
        let eta_min = 0.8;
        let eta_range = 0.1;

        eta_test_body!(res, eta_max, eta_min, eta_range);
    }

    // --- pwr_out_min tests ---

    /// Helper: build an asymmetric edrv with pwr_out_max = 8 MW, pwr_out_min = -3 MW
    fn test_edrv_asymmetric() -> ElectricDrivetrain {
        ElectricDrivetrain::new(vec![0.0, 1.0], vec![0.9, 0.8], 8e6, Some(-3e6), None).unwrap()
    }

    /// When `pwr_out_min` is `None`, `pwr_out_min_resolved()` should return `-pwr_out_max`,
    /// preserving the original symmetric behavior.
    #[test]
    fn test_pwr_out_min_resolved_none_is_symmetric() {
        let edrv = test_edrv();
        assert_eq!(edrv.pwr_out_min_resolved(), -edrv.pwr_out_max);
    }

    /// When `pwr_out_min` is `Some(-3 MW)`, `pwr_out_min_resolved()` should return exactly -3 MW.
    #[test]
    fn test_pwr_out_min_resolved_some() {
        let edrv = test_edrv_asymmetric();
        assert_eq!(edrv.pwr_out_min_resolved().get::<si::megawatt>(), -3.0);
    }

    /// When `pwr_out_min` is `None` (symmetric), `pwr_out_min_abs()` should equal `pwr_out_max`.
    #[test]
    fn test_pwr_out_min_abs_none() {
        let edrv = test_edrv();
        assert_eq!(edrv.pwr_out_min_abs(), edrv.pwr_out_max);
    }

    /// When `pwr_out_min` is `Some(-3 MW)`, `pwr_out_min_abs()` should return the positive
    /// magnitude 3 MW.
    #[test]
    fn test_pwr_out_min_abs_some() {
        let edrv = test_edrv_asymmetric();
        assert_eq!(edrv.pwr_out_min_abs().get::<si::megawatt>(), 3.0);
    }

    /// `pwr_limit_for` should return `pwr_out_max` (8 MW) when given a positive power value,
    /// so that positive power fractions are normalized against the positive limit.
    #[test]
    fn test_pwr_limit_for_positive() {
        let edrv = test_edrv_asymmetric();
        assert_eq!(edrv.pwr_limit_for(uc::W * 1e6), edrv.pwr_out_max);
    }

    /// `pwr_limit_for` should return `|pwr_out_min|` (3 MW) when given a negative power value,
    /// so that regen power fractions are normalized against the regen limit.
    #[test]
    fn test_pwr_limit_for_negative() {
        let edrv = test_edrv_asymmetric();
        assert_eq!(edrv.pwr_limit_for(uc::W * -1e6).get::<si::megawatt>(), 3.0);
    }

    /// `pwr_limit_for` should return `pwr_out_max` when given exactly zero power,
    /// treating zero as the positive/traction side.
    #[test]
    fn test_pwr_limit_for_zero() {
        let edrv = test_edrv_asymmetric();
        assert_eq!(edrv.pwr_limit_for(si::Power::ZERO), edrv.pwr_out_max);
    }

    /// Constructor should return an error when `pwr_out_min` is positive, since the
    /// minimum output power must be non-positive (zero or negative for regen).
    #[test]
    fn test_new_rejects_positive_pwr_out_min() {
        let result = ElectricDrivetrain::new(vec![0.0, 1.0], vec![0.9, 0.8], 8e6, Some(1e6), None);
        assert!(result.is_err());
    }

    /// Constructor should succeed when `pwr_out_min` is exactly zero, meaning no
    /// regenerative braking capability.
    #[test]
    fn test_new_accepts_zero_pwr_out_min() {
        let result = ElectricDrivetrain::new(vec![0.0, 1.0], vec![0.9, 0.8], 8e6, Some(0.0), None);
        assert!(result.is_ok());
    }

    /// The default YAML configuration should deserialize with `pwr_out_min = None`,
    /// ensuring backward compatibility with existing config files that omit the field.
    #[test]
    fn test_default_has_none_pwr_out_min() {
        let edrv = ElectricDrivetrain::default();
        assert!(edrv.pwr_out_min.is_none());
    }

    /// With symmetric limits (`pwr_out_min = None`), requesting positive power at half
    /// of `pwr_out_max` should succeed.
    #[test]
    fn test_symmetric_edrv_set_pwr_in_req_positive() {
        let mut edrv = test_edrv();
        edrv.check_and_reset(|| format_dbg!()).unwrap();
        // Set dynamic limits so set_pwr_in_req passes the dynamic check
        edrv.set_cur_pwr_max_out(edrv.pwr_out_max, None).unwrap();
        edrv.set_cur_pwr_regen_max(edrv.pwr_out_max).unwrap();
        // Requesting half-max should succeed
        let result = edrv.set_pwr_in_req(uc::W * 4e6, uc::S * 1.0);
        assert!(result.is_ok());
    }

    /// With symmetric limits (`pwr_out_min = None`), requesting negative power at half
    /// of `pwr_out_max` magnitude should succeed because the resolved min is `-pwr_out_max`.
    #[test]
    fn test_symmetric_edrv_set_pwr_in_req_negative() {
        let mut edrv = test_edrv();
        edrv.check_and_reset(|| format_dbg!()).unwrap();
        edrv.set_cur_pwr_max_out(edrv.pwr_out_max, None).unwrap();
        edrv.set_cur_pwr_regen_max(edrv.pwr_out_max).unwrap();
        // Requesting negative half-max should succeed (symmetric)
        let result = edrv.set_pwr_in_req(uc::W * -4e6, uc::S * 1.0);
        assert!(result.is_ok());
    }

    /// With asymmetric limits (max = 8 MW, min = -3 MW), requesting -2 MW is within the
    /// allowed range and should succeed.
    #[test]
    fn test_asymmetric_set_pwr_in_req_within_bounds() {
        let mut edrv = test_edrv_asymmetric();
        edrv.check_and_reset(|| format_dbg!()).unwrap();
        edrv.set_cur_pwr_max_out(edrv.pwr_out_max, None).unwrap();
        edrv.set_cur_pwr_regen_max(edrv.pwr_out_min_abs()).unwrap();
        // -2 MW is within [-3, 8] MW range
        let result = edrv.set_pwr_in_req(uc::W * -2e6, uc::S * 1.0);
        assert!(result.is_ok());
    }

    /// With asymmetric limits (max = 8 MW, min = -3 MW), requesting -5 MW exceeds the
    /// minimum power limit and should return an error.
    #[test]
    fn test_asymmetric_set_pwr_in_req_exceeds_min() {
        let mut edrv = test_edrv_asymmetric();
        edrv.check_and_reset(|| format_dbg!()).unwrap();
        edrv.set_cur_pwr_max_out(edrv.pwr_out_max, None).unwrap();
        edrv.set_cur_pwr_regen_max(edrv.pwr_out_min_abs()).unwrap();
        // -5 MW is below pwr_out_min of -3 MW — should fail
        let result = edrv.set_pwr_in_req(uc::W * -5e6, uc::S * 1.0);
        assert!(result.is_err());
    }

    /// With asymmetric limits (min = -3 MW), `set_cur_pwr_regen_max` should clamp the
    /// regen capacity to `|pwr_out_min|` (3 MW) even when a larger input is offered.
    #[test]
    fn test_asymmetric_regen_max_clamped_to_pwr_out_min_abs() {
        let mut edrv = test_edrv_asymmetric();
        edrv.check_and_reset(|| format_dbg!()).unwrap();
        // Offer more regen capacity than pwr_out_min_abs (3 MW) — should be clamped
        edrv.set_cur_pwr_regen_max(uc::W * 10e6).unwrap();
        let regen_max = edrv
            .state
            .pwr_mech_regen_max
            .get_fresh(|| format_dbg!())
            .unwrap();
        assert!(*regen_max <= edrv.pwr_out_min_abs());
    }
}
