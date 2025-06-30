use core::sync::atomic::AtomicBool;

use super::messages::{self, ControlResponse};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationStatus {
    Idle,
    Requested,
    InProgress {
        step: usize,
        total_steps: usize,
        resistance_kom: f32,
    },
}

pub struct ControlAction {
    pub is_calibrate: bool,
    pub potentiometer: u16,
}

pub struct RkCoefficients {
    pub v0: f32,
    pub a0: f32,
    pub a1: f32,
    pub a2: f32,
    pub a3: f32,
}

impl Default for RkCoefficients {
    fn default() -> Self {
        Self {
            v0: 0.0,
            a0: 0.0,
            a1: 0.0,
            a2: 1.0,
            a3: 0.0,
        }
    }
}

impl RkCoefficients {
    pub fn calc(&self, adc_val: crate::box_ploat::BoxPlot<f32>) -> (f32, f32) {
        let avg = adc_val.average();

        let v1 = avg - self.v0;
        let v2 = v1 * v1;
        let v3 = v2 * v1;

        let r_k = 1.0 / (self.a0 + v1 * self.a1 + v2 * self.a2 + v3 * self.a3);

        let err_avg = adc_val.iqr() / 2.0;

        let erro_r_k = err_avg * r_k / avg;

        (r_k, erro_r_k)
    }
}

#[allow(dead_code)]
pub struct ControlState {
    pub is_calibrate_selected: bool,
    pub is_calibrate_in_progress: CalibrationStatus,
    pub pll_state: messages::PllState,
    pub potentiometer: u16,
    pub self_freq_potenciometer: u16,
}

pub struct Control {
    updated: AtomicBool,
    save_requested: AtomicBool,

    is_calibrate_selected: bool,
    is_calibrate_in_progress: CalibrationStatus,
    pll_state: messages::PllState,
    potentiometer: u16,
    self_freq_potenciometer: u16,

    measure_time: u16,
    f_ref: u32,
    pll_multiplier: u32,

    rk_coefficients: RkCoefficients,
    rk_zero_offset: f32,
    potenciometer_total_resistance_kohm: f32,
}

impl Control {
    pub fn new<E: defmt::Format, S: crate::settings::SettingsStorage<E>>(
        pll_multiplier: u32,
        settings: &mut S,
    ) -> Self {
        let loaded_settings = settings.data();

        Self {
            updated: AtomicBool::new(false),
            save_requested: AtomicBool::new(false),
            is_calibrate_selected: false,
            is_calibrate_in_progress: CalibrationStatus::Idle,

            pll_state: if cfg!(feature = "auto-pll") {
                messages::PllState::Auto
            } else {
                messages::PllState::Off
            },

            potentiometer: 0,
            self_freq_potenciometer: 0,

            measure_time: crate::config::FREQ_MEASURE_TIME_MS,
            f_ref: loaded_settings.fref,
            pll_multiplier,

            rk_coefficients: RkCoefficients::default(),

            rk_zero_offset: loaded_settings.rk_zero_offset,
            potenciometer_total_resistance_kohm: loaded_settings
                .potenciometer_total_resistance_kohm,
        }
    }

    #[allow(dead_code)]
    pub fn current_control_state(&self) -> ControlState {
        ControlState {
            is_calibrate_selected: self.is_calibrate_selected,
            is_calibrate_in_progress: self.is_calibrate_in_progress,
            pll_state: self.pll_state,
            potentiometer: self.potentiometer,
            self_freq_potenciometer: self.self_freq_potenciometer,
        }
    }

    pub fn get_action(&self) -> Option<ControlAction> {
        if self.is_updated() {
            Some(ControlAction {
                is_calibrate: self.is_calibrate_selected,
                potentiometer: self.potentiometer,
            })
        } else {
            None
        }
    }

    pub fn reset_control_request(&mut self) {
        self.updated
            .store(false, core::sync::atomic::Ordering::Relaxed);
    }

    pub fn save_done(&mut self) {
        self.save_requested
            .store(false, core::sync::atomic::Ordering::Relaxed);
    }

    pub fn pll_state(&self) -> messages::PllState {
        self.pll_state
    }

    pub fn is_updated(&self) -> bool {
        self.updated.load(core::sync::atomic::Ordering::Relaxed)
    }

    pub fn is_save_requested(&self) -> bool {
        self.save_requested
            .load(core::sync::atomic::Ordering::Relaxed)
    }

    pub fn calibrate_required(&self) -> bool {
        self.is_calibrate_in_progress == CalibrationStatus::Requested
    }

    pub fn is_calibrating(&self) -> bool {
        matches!(
            self.is_calibrate_in_progress,
            CalibrationStatus::InProgress { .. }
        )
    }

    pub fn calibration_updated(&mut self, step: usize, total_steps: usize, resistance_kom: f32) {
        self.is_calibrate_in_progress = CalibrationStatus::InProgress {
            step,
            total_steps,
            resistance_kom,
        };
    }

    pub fn calibrate_done(&mut self, rk_coeffs: &[f32], basis: f32) {
        self.rk_coefficients.v0 = basis;

        self.rk_coefficients.a0 = rk_coeffs[0];
        self.rk_coefficients.a1 = rk_coeffs[1];
        self.rk_coefficients.a2 = rk_coeffs[2];
        self.rk_coefficients.a3 = rk_coeffs[3];

        self.is_calibrate_in_progress = CalibrationStatus::Idle;
    }

    pub fn update(&mut self) {
        self.updated
            .store(true, core::sync::atomic::Ordering::Relaxed);
    }

    pub fn save_request(&mut self) {
        self.save_requested
            .store(true, core::sync::atomic::Ordering::Relaxed);
    }

    pub fn into_response(&self) -> ControlResponse {
        ControlResponse {
            is_calibrate: self.is_calibrate_selected,
            pll_state: self.pll_state.into(),
            potentiometer: self.potentiometer as u32,
            rk_coefficients: super::messages::RkCoefficients {
                v0: Some(self.rk_coefficients.v0),
                r0: Some(self.rk_coefficients.a0),
                a0: Some(self.rk_coefficients.a1),
                a1: Some(self.rk_coefficients.a2),
                a2: Some(self.rk_coefficients.a3),
            },
            is_calibration_in_progress: self.is_calibrate_in_progress != CalibrationStatus::Idle,
            self_freq_control: self.self_freq_potenciometer as u32,
            fref: self.f_ref,
            rk_zero_offset: self.rk_zero_offset,
            potentiometer_resistance: self.potenciometer_total_resistance_kohm,
        }
    }

    pub fn get_measure_time(&self) -> u16 {
        self.measure_time
    }

    pub fn get_f_ref(&self) -> u32 {
        self.f_ref * self.pll_multiplier
    }

    pub fn rk_coeffs(&self) -> &RkCoefficients {
        &self.rk_coefficients
    }

    pub fn rk_zero_offset(&self) -> f32 {
        self.rk_zero_offset
    }

    pub fn saveable_settings(&self) -> crate::settings::AppSettings {
        crate::settings::AppSettings {
            fref: self.f_ref,
            rk_zero_offset: self.rk_zero_offset,
            potenciometer_total_resistance_kohm: self.potenciometer_total_resistance_kohm,
        }
    }
}

pub fn process_control(
    control: &super::messages::ControlRequest,
    current_control_state: &mut Control,
) {
    if let Some(set_calibrate) = control.set_calibrate {
        current_control_state.is_calibrate_selected = set_calibrate;
        current_control_state.update();
    }

    if let Some(set_pll_state) = control.set_pll_state {
        current_control_state.pll_state = set_pll_state.try_into().expect("Invalid enum value");
        current_control_state.update();
    }

    if let Some(set_potentiometer) = control.set_potentiometer {
        current_control_state.potentiometer = set_potentiometer as u16;
        current_control_state.update();
    }

    if let Some(set_self_freq_control) = control.set_self_freq_control {
        current_control_state.self_freq_potenciometer = set_self_freq_control as u16;
        current_control_state.update();
    }

    if let Some(set_fref) = control.set_fref {
        current_control_state.f_ref = set_fref;
        current_control_state.save_request();
    }

    if let Some(set_rk_zero_offset) = control.set_rk_zero_offset {
        current_control_state.rk_zero_offset = set_rk_zero_offset;
        current_control_state.save_request();
    }

    if let Some(set_potenciometer_total_resistance) = control.set_potentiometer_resistance {
        current_control_state.potenciometer_total_resistance_kohm =
            set_potenciometer_total_resistance;
        current_control_state.save_request();
    }

    if let Some(set_rk_coefficients) = &control.set_rk_coefficients {
        if let Some(v0) = set_rk_coefficients.v0 {
            current_control_state.rk_coefficients.v0 = v0;
        }

        if let Some(r0) = set_rk_coefficients.r0 {
            current_control_state.rk_coefficients.a0 = r0;
        }

        if let Some(a0) = set_rk_coefficients.a0 {
            current_control_state.rk_coefficients.a1 = a0;
        }

        if let Some(a1) = set_rk_coefficients.a1 {
            current_control_state.rk_coefficients.a2 = a1;
        }

        if let Some(a2) = set_rk_coefficients.a2 {
            current_control_state.rk_coefficients.a3 = a2;
        }
    }

    if &Some(true) == &control.calibration_request && !current_control_state.is_calibrating() {
        current_control_state.is_calibrate_in_progress = CalibrationStatus::Requested;
    }
}
