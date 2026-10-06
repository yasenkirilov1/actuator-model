# Selected budget ADC with the hot winding: native inputs

This folder contains **seven prepared inputs, not executed experiments**.
`ADXL1002_8000_M08_30k_current100_hot32V_configs.mat` contains the retained four
30 s tighter-static and three 4 s matched-mechanics stress cases. Run this MAT
through `qualify_adc_acquisition_20261002` using `customCasesFile`; its separate
completed output must establish force results and native sample coverage.

The source is the frozen `ADXL1002_reference_8000_L0_current100_24V` group. Every
acquisition field remains exactly equal, including the ADS131M08 30 kSPS Sinc3
clock, assumed matched 8 kHz complete assembled response, force/acceleration
noise PSD and ADC noise, ranges, quantization, filtering, pipeline and relative
timing residuals. This does not assert the manufacturer's complete sensor
transfer function or physical sanding performance.

Only the electrical design and its force-to-current gain conversion change:

- Actual coil Kf: **24.4 N/A**; inductance: **4 mH**.
- Actual operating resistance: **8.896 Ω**, from 6.4 Ω at 25 °C times 1.39.
- Available bipolar output: **32 V**, for the proposed 36 V supply context.
- The **100 µs current-response target stays unchanged**.
- Identified Kf retains each source identified/actual Kf ratio. The lower-link
  inverse uses that new identified value, and current-based `forceKp`, `forceKi`
  and `forceKd` scale by 20/24.4. Its derived numerator is rebuilt and checked.

All other static calibration, mechanics, mass identification, robot timing,
travel, current limits and controller inputs remain unchanged. The preparation
recipe checks every input subtree and records all nine changed fields and the
derived inverse coefficients in `manifest.json`. Existing metadata fields also
remain intact.

These are new hot-winding dynamic simulation inputs, not a voltage mapping of
old traces. Operating resistance is identified for the native current regulator.
No time-varying thermal state, supplier hot-force guarantee, Kf variation over
stroke or measured assembled driver phase is introduced. Continuous temperature,
driver dropout and physical prototype behavior require their own measurements.

The canonical model, shared helpers, historical evidence and b370 native
acquisition source are unchanged. The local recipe refuses to overwrite a
completed manifest. `input_seal.json` binds this folder's frozen inputs and
documentation.
