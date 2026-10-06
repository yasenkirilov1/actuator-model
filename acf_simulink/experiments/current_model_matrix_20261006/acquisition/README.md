# Newest-model acquisition diagnostics

The [predeclared contract](diagnostic_contract.json) fixes 228 short diagnostic
records: four separately frozen actuator/pad banks, three mass/force/orientation
corners, and 19 baseline or one-at-a-time sensing changes. Controllers are held
fixed. Every integration endpoint is scored in memory, including noisy loaded
startup. No arrays or traces are retained.

Compile the original engine into an owned temporary directory and verify the
two frozen soft-pad pilot recordings before running the matrix:

```bash
python acf_simulink/experiments/current_model_matrix_20261006/acquisition/runner.py pilot
python acf_simulink/experiments/current_model_matrix_20261006/acquisition/runner.py matrix --banks path/to/selected_banks.json --workers 2
python acf_simulink/experiments/current_model_matrix_20261006/acquisition/runner.py verify
```

The selected-bank manifest names the immutable engine hash and exactly four
banks. A bank can contain `id` and `config`, or reference `config_file`,
`config_sha256`, `config_key` (default `base_config`) and optional `gains_key`.
Referenced paths are relative to the project. Source references and compact
parameter deltas make each diagnostic reproducible without copying parameter
banks into the result file.

These are simplified acquisition **response diagnostics**. Bit changes adjust
force and acceleration quantization only. Rate changes use an assumed inverse-
rate converter transport delay; analog source noise stays fixed. Matched analog
bandwidth changes force, acceleration and position together. The engine measures
the resulting filtered noise rather than assuming that the source RMS fields
equal output RMS. It does not implement exact ADS131M04 SINC3 response or a
separate converter noise law.

The acceleration-removal condition jointly removes inertia reconstruction,
acceleration feedforward and integrated tool-velocity estimation, while keeping
gravity tare and identified tool mass. The frozen engine does not support an
independent reconstruction-only switch or channel-specific timestamp skew.
Those limitations are reported explicitly.

The source/code repeatability pilot is separate from independent native physical
replay. The main records are finite fixed-robot, two-second axial-force
diagnostics; they do not qualify installed sensors, full sanding normal force,
engagement, recentering or thermal duty. Physical stops, force failures, sensor/
actuator range screens and catalogue screw RMS-current screens are reported
separately. Unknown continuous coil heating remains unknown.

Evidence is written under
[`validation/current_model_matrix_20261006/acquisition`](../../../validation/current_model_matrix_20261006/acquisition).
The source plus evidence must fit the assigned 700 KiB retained cap, with an
8 MiB temporary build cap under the shared root storage forecast. The original
study, frozen evidence and historical 42 ADC diagnostics are preserved.

## Executed diagnostic result

All **228** predeclared records executed, with zero preparation rejections.
The compact coverage audit checks unique identities and reconstructs each full
parameter configuration from its frozen bank and recorded deltas. The engine
scored **185,927,578 integration endpoints plus 224 extra event-side force
limits**, including loaded-equilibrium startup. Runtime was 105.9 s with two
threads. The diagnostic engine and API are pinned by the contract; the
source-bound pilots reproduce every non-runtime recorded soft-pad metric.

At the three declared corners per soft-pad actuator, 24/16/14/12-bit code
spacing and 30/15 ksample/s all pass the force, physical and rating screens.
These finite response tests do not prove that a 12-bit hardware converter
meets the installed accuracy requirement. Quantization tests retain the same
analog noise source and simplified transfer. At 7.5 ksample/s the light coil
case loses contact. Matched 500/950 Hz bandwidth also loses contact in the
light coil case; 1.9/3.8 kHz pass these corners. The screw actuator passes
all of those acquisition changes.

Removing acceleration jointly removes all acceleration-based control uses.
All three soft coil corners fail force; two lose contact, and the remaining
heavy/high-force recording exceeds the proposed sensor range. All three soft
screw corners pass force, but the two heavy cases have 1.500/1.684 A RMS,
above the reference motor's 1.42 A catalogue screen. This does not isolate the
inertia reconstruction term from integrated velocity feedback or feedforward.

The strongest calibration limitation is **+1% identified S mass**: both heavy
soft coil corners fail force and sensor eligibility; the light coil and all
three screw corners pass. A -1% mass change passes all six soft corners, with
0.237239 N worst error. Positive force bias up to 0.05 N and acceleration bias
up to 0.02 m/s² pass all six, with 0.305916 N worst at the larger acceleration
bias. These one-at-a-time sensitivities do not establish a calibrated residual
box. PCB, mount and tool mass belong in identified S, and the installed
mechanical/acquisition response needs validation. Large force extrema from
sensor-ineligible continuations are not usable prototype predictions.

The 0/0.014/0.028/0.056 m/s² normalized acceleration-noise conditions all pass
the six soft corners. The largest soft-pad error at the 0.056 source is
0.161144 N. Filtered output RMS is recorded separately from the fixed-reference
source setting. No separate ADXL354-plus-ADC noise model was introduced.

Every rigid-plate diagnostic fails the 0.5 N force requirement. The known
ideal-envelope acceleration jumps already impose a heavy-tool force-error
floor; the selected fixed controllers also have larger transient errors.
Those failures cannot establish that a different ADC alone would fix the
no-pad mechanics.

Detailed metrics are in
[`diagnostic_results.json`](../../../validation/current_model_matrix_20261006/acquisition/diagnostic_results.json),
with matched deltas in
[`paired_deltas.json`](../../../validation/current_model_matrix_20261006/acquisition/paired_deltas.json)
and compact source/coverage checks in
[`audit.json`](../../../validation/current_model_matrix_20261006/acquisition/audit.json).
Completed outputs are preserved; a new experiment needs a separately versioned
study rather than overwriting these records.
