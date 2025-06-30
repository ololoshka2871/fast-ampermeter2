use crate::box_ploat::BoxPlot;

#[derive(Copy, Clone, Default)]
pub struct OutputData {
    // Output values
    pub rk: f32,
    pub error_rk: f32,
    pub f: f32,

    // Adc raw values
    pub adc_rx: BoxPlot<f32>,
    pub adc_v_pll: BoxPlot<f32>,

    // freqmeter_raw_values
    pub f_target: u32,
    pub f_result: u32,

    pub is_pll_on: bool,

    pub f_unstable_avg: f32,

    pub button_state: bool,
}

impl OutputData {
    pub fn update_frequency(&mut self, f: f32, unstable_avg: f32, target: u32, result: u32) {
        self.f = f;
        self.f_unstable_avg = unstable_avg;
        self.f_target = target;
        self.f_result = result;
    }

    pub fn update_analog(
        &mut self,
        rk: f32,
        error_rk: f32,
        adc_rx: BoxPlot<f32>,
        adc_v_pll: BoxPlot<f32>,
    ) {
        self.rk = rk;
        self.error_rk = error_rk;
        self.adc_rx = adc_rx;
        self.adc_v_pll = adc_v_pll;
    }

    pub fn update_pll_state(&mut self, on: bool) {
        self.is_pll_on = on;
    }

    pub fn update_button(&mut self, value: bool) {
        self.button_state = value;
    }
}

pub fn process_output(
    request: &super::messages::OutputReq,
    resp: &mut super::messages::OutputResponse,
    current_output_data: &OutputData,
) {
    if let Some(_main_values) = &request.get_main_values {
        resp.rk = Some(current_output_data.rk);
        resp.error_rk = Some(current_output_data.error_rk);
        resp.freq = Some(current_output_data.f);
        resp.average_freq_unstable = Some(current_output_data.f_unstable_avg);
    }

    if let Some(_get_raw) = &request.get_raw {
        resp.f_result = Some(super::messages::FreqmeterResult {
            target: current_output_data.f_target,
            result: current_output_data.f_result,
        });
        resp.adc_rx = Some(current_output_data.adc_rx.into());
        resp.adc_v_pll = Some(current_output_data.adc_v_pll.into());

        resp.is_pll_on = Some(current_output_data.is_pll_on);

        resp.button = Some(current_output_data.button_state);
    }
}
