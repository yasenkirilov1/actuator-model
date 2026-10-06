# Private analog acquisition and cheaper ADC study

This directory owns an isolated native Simulink fork. The canonical model,
shared helpers and delivered 2026-09-30 package are preserved. The model is
`acf_force_control_adc_trial.slx`; its physical plant, control strategies and
robot gates are unchanged from the retained V8 model with native calibration
instruments. `acquisition_contract.json` binds its frozen inputs and smoke
checks. Build failures under `build_attempts/` are software rejections, not
physical experiments or failed force qualification.

## Native acquisition path

Each active measurement uses a continuous two-pole **complete characterized
front-end response**, followed by the converter's own analog input filter,
its native impulse response, the actual ADC output sampler, independent ADC
output noise, input range, quantization, per-conversion EMA and physical
delivery pipeline. The existing 10 kHz controller then uses the latest
available frame causally. The chosen two-pole response includes assembled
sensor and conditioning behavior; it does not remove accelerometer phase or
promise that any external RC network produces that response.

The SAR model has the ADS8588S ±5 V range's third-order 16 kHz Butterworth
input filter. Its conversion, readout and host processing share a **total
20 µs** post-aperture pipeline, rather than adding those times twice. The
Sinc3 model uses the exact OSR kernel convolved three times at its explicitly
stated modulator clock. Coefficients are aggregated into causal fine-clock
delay bins before a genuine 30, 15 or 7.5 kSPS output latch. The Sinc3 group
delay belongs to that filter and is not added again to the 20 µs delivery
pipeline. Native refinement and a complex frequency-response audit quantify
the fine-clock approximation.

Sensor noise is injected **before analog filtering** at a fixed fine clock
with a fixed one-sided PSD derived from the retained nominal 100 kSPS
response. It is not rescaled to maintain the same output RMS at every new
ADC rate or cutoff. Converter noise is a separate post-conversion process:
the Sinc3 datasheet noise has already passed through its converter filter,
so it is not passed through the same filter again. SNR-derived SAR total
noise subtracts the ideal quantization variance before the explicit quantizer.
Optional converter output correlation preserves RMS while testing sensitivity
to colored noise. It is an engineering stress, not a manufacturer guarantee.

Force ADC input restores a known static gravity offset and subtracts its
quantized tare after conversion. Accelerometer ADC input includes gravity;
the stored tare uses the actual converter quantum and the calibrated frame.
The ADC force noise and range are effective reconstructed-axis assumptions:
the real six-bridge calibration matrix must verify that its channel gains and
ADC noise attain these axis-equivalent budgets. This model is planar; eight
hardware channels fit six bridge voltages and two acceleration axes. Nine
channels would be required for three raw acceleration axes.

## Parameter contract

Put this directory's `helpers/` first on the MATLAB path, then call
`p=acf_prepare(p)`. The dispatcher calls `acf_adc_prepare` and the retained
`acf_prepare_baseline_v8`; a MATLAB script does not replace the native plant.

Parameters live under `p.adcStudy`:

| Field | Meaning |
|---|---|
| `fineClock_s` | Fixed sensor-noise and converter impulse clock; a multiple of the solver step |
| `outputRate_Hz` | Actual per-channel ADC output rate |
| `converter` | `sar` or `sinc3` |
| `modulatorRate_Hz` | Actual Sinc3 modulator clock; OSR is this divided by output rate |
| `analogCutoff_Hz`, `analogDamping` | Complete characterized analog response envelope |
| `identifiedAnalogCutoff_Hz` | Known response for causal issued-motion alignment |
| `converterAnalogOrder`, `converterAnalogCutoff_Hz` | SAR order 3 / 16 kHz; Sinc3 order 0 |
| `emaTime_s`, `identifiedEmaTime_s` | Actual and identified per-conversion EMA targets |
| `pipelineDelay_s`, `identifiedPipelineDelay_s` | Total actual and known post-conversion delivery pipeline |
| `accelerationExtraDelay_s` | Additional acceleration-only phase-matching stress; default zero |
| `forceQuantum_N`, `accelerationQuantum_m_s2` | Converter code spacing in physical units |
| `forceAdcNoiseRms_N`, `accelerationAdcNoiseRms_m_s2` | Additive output noise, excluding separately modeled ideal quantization |
| `adcNoiseCorrelationTime_s` | Optional output correlation; zero gives independent conversion noise |
| `forceAdcRange_N`, `accelerationAdcRange_g` | Bipolar ADC input ranges after the stated analog gain/offset |
| `forceAnalogToneAmplitude_N`, `lateralAnalogToneAmplitude_N` | Explicit pre-filter sensor-side tone probes |
| `accelerationZAnalogToneAmplitude_m_s2`, `accelerationXAnalogToneAmplitude_m_s2` | Explicit pre-filter acceleration tone probes |
| `analogToneFrequency_Hz`, `analogTonePhase_rad` | Tone probe frequency and phase |

Derived `p.adcStudy.d` contains the filter matrices, complete coefficient
arrays, exact output period, reference PSD, actual/identified channel timing
and converter response approximation. Existing `p.acquisition` cutoff scales
and relative pipeline residuals remain separate actual/identified errors.
Its legacy `rawSampleTime_s` and `p.sensor.sampleTime` become the internal
fine clock; the **actual ADC output clock** is `p.adcStudy.outputSampleTime_s`.
`p.force.sampleTime` remains the independent controller period.

The fine clock must divide both ADC and controller periods exactly. A clock
of `1/300000` seconds supports SAR 100/50/20/10 kSPS, Sinc3 30/15/7.5 kSPS
and a 10 kHz controller. A SAR-only 10 µs clock can be checked separately and
refined to 5 or 2.5 µs. Do not silently label a 30 kSPS clock as 32 kSPS.
Changing converter architecture or filter length changes the native state
dimensions and requires recompilation; reuse compilation only where valid.

`adc_acquisition_log` records all conversion frames, including four ADC input
values and four clipping magnitudes. `normal_contact_log` remains the native
surface-normal force record. Native full-history force scores include startup;
neither this source freeze nor a quiet smoke test qualifies the sharp bump.
