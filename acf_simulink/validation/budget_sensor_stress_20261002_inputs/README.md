# Budget accelerometer: retained simultaneous-stress inputs

This folder contains **prepared inputs, not executed experiments**. The native
runner `experiments/qualify_adc_acquisition_20261002.m` can load each MAT through
`options.customCasesFile`. Each file contains `configs` with seven full prepared
parameter structures: the original four tighter-static 30 s cases and three
matched, known-stiffness/damping 4 s mixed cases. The total is 49 cases across
seven hardware/envelope groups. Failed simulations and completed results belong
in the runner's separate output folders.

`manifest.json` binds every input file, retained parameter source, source-model
hash, case identity, actual and identified pipeline, noise assumption and drive.
The native source is the immutable `adc_acquisition_trial_20261002` acquisition
fork. No canonical model or shared helper was changed.

| Group prefix | Complete-response envelope | Extra common latency | Current target / usable voltage |
|---|---:|---:|---:|
| `ADXL354C_1900_L0_current100_24V` | 1.9 kHz | 0 µs | 100 µs / 24 V |
| `ADXL354C_1900_L0_current50_36V` | 1.9 kHz | 0 µs | 50 µs / 36 V |
| `ADXL354C_1900_L50_current50_36V` | 1.9 kHz | 50 µs | 50 µs / 36 V |
| `digital70_5100_L80_current100_24V` | 5.1 kHz | 80 µs | 100 µs / 24 V |
| `digital70_5100_L80_current50_36V` | 5.1 kHz | 80 µs | 50 µs / 36 V |
| `ADXL1002_reference_8000_L0_current100_24V` | 8 kHz | 0 µs | 100 µs / 24 V |
| `ADXL354C_1900_L0_current50_hot32V` | 1.9 kHz | 0 µs | 50 µs / 32 V |

These are acceptance envelopes for an engineered, matched **complete assembled
response**. The ADXL354C inputs anchor noise density at 33.75 µg/√Hz and range at
±8 g. They do not assert the manufacturer's complete transfer function. The
digital comparison anchors density at 70 µg/√Hz, range at ±16 g and code spacing
at 32 g / 65,536; the shared matched converter response is an acceptance target,
not an asserted ICM clock, FIR, sample interface or external ADC implementation.
The ADXL1002 reference preserves the original acceleration PSD exactly and
requires its stated 8 kHz assembled response. None establishes physical sanding
vibration performance or qualifies a purchased sensor.

All groups use the explicitly clocked ADS131M08 Sinc3 envelope at 30 kSPS,
3.84 MHz modulator rate and a 1/300,000 s fine/noise clock. The force ADC uses an
axis-equivalent ±160 N range, a 320 N / 2²⁴ quantum and 21.31 µV per-conversion
output noise mapped through the stated gain. Analog acceleration gain is
0.1 V/g for ADXL354 and 0.02 V/g for the ADXL1002 reference. Quantization variance
is subtracted from total converter-noise variance before adding explicit noise.
Digital acceleration gets its explicit code spacing and no additional analog
ADC noise. PSD normalization retains the original 3,177.12656176 Hz reference
noise bandwidth rather than forcing equal output noise at every new cutoff.

The actual source common pipeline and all four channel offsets remain intact;
the additional latency is added to both actual and identified paths. The
identified pipeline starts at 20 µs, so the source common timing residual remains.
Force/acceleration frame, bias, scale, cross-axis, mass and mechanics residuals
are preserved. The script verifies these fields after preparation. Recorded
seed identities and full startup scoring remain available to the runner.

The hot-winding group changes actual electrical constants to Kf = 24.4 N/A,
L = 4 mH and **R = 8.896 Ω**. The resistance is 6.4 Ω at 25 °C multiplied by
1.39; no time-varying thermal state is introduced. Its 32 V usable bipolar output
is the modeled margin for a 36 V supply. The actual/identified Kf ratio from each
source stress case is retained, and current-based PI/PID coefficients scale by
20/24.4. Peak/current limits, travel and mechanics remain unchanged. Operating
resistance is identified for the current regulator; supplier hot-force behavior,
Kf over stroke and assembled driver phase still require measurement. These are
new hot-winding dynamic simulation inputs, not a voltage mapping of old traces.

The preparation recipe is frozen locally beside the inputs. It refuses to
overwrite a completed manifest. `input_seal.json` binds the MATs, manifest,
recipe, this README and a preserved preparation rejection; that software
rejection is separate from physical failures or native qualification.
