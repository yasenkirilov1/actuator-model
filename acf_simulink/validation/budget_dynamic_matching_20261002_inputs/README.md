# Tighter assembled dynamic matching: paired inputs

This folder contains **prepared native inputs, not executed experiments**.
The primary MAT is `ADXL354C_1900_L0_current50_36V_dynamic_match_configs.mat`.
The analogous `L50` MAT retains the extra 50 µs common latency as a sensitivity
comparison. Each has the exact seven retained stress cases: four 30 s
tighter-static calibration cases and three 4 s matched-mechanics cases.

Run each through `qualify_adc_acquisition_20261002` using `customCasesFile`.
The separate runner output must establish completed sample counts, native
histories, startup behavior and force results before any pass is claimed.
`manifest.json` binds source/input hashes, every changed field, original/new
values, exact deltas, seeds and timing. The local frozen recipe reproduces them.

Only these acquisition targets change:

- Each of the four actual anti-alias scale deviations from one is multiplied
  by 0.1. A source ±10% channel scale therefore becomes ±1%.
- The actual common anti-alias cutoff deviation from its identified value
  is multiplied by 0.1. It combines multiplicatively with channel scale;
  two +1% factors give a +2.01% effective cutoff error, not +1% overall.
- The actual common EMA time deviation from its identified value is
  multiplied by 0.1. Optional per-channel EMA scales would receive the same
  change; none exists in these frozen inputs.
- Each of the four relative delivery offsets is multiplied by 0.2. A source
  ±10 µs offset becomes ±2 µs.

**Common actual pipeline residual remains intact.** Neither nominal nor
identified pipeline changes. All identified acquisition dynamics, static bias,
scale, frame/cross-axis calibration, mass and mechanics identification, robot
timing, current response, usable voltage, sensor noise density, range and ADC
settings remain byte-equivalent MATLAB values outside the requested acquisition
fields. Preparation rebuilds derived coefficients and verifies the unchanged
input parameter subtrees; it does not simulate a separate hidden physical model.

The reference remains the ADXL354C noise/range anchored **assumed engineered
complete-response envelope** at 1.9 kHz, with a 50 µs current target and 36 V
scientific drive. This is a tighter assembled matching target, not perfect
sensors, better static calibration, a manufacturer's complete transfer-function
claim or physical sensor qualification. The L50 comparison changes no additional
fields and retains all 50 µs of its original added latency.

The native b370 acquisition source, canonical model, shared helpers, earlier
inputs and historical evidence are unchanged. `input_seal.json` binds this
folder. A corrected floating-point preparation assertion is recorded separately
as a software rejection; no native simulation occurred in that attempt.
