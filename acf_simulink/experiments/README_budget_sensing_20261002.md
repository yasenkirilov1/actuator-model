# Reproducing the lower-cost sensing study — 2 October 2026

The prototype decision is ADS131M08 at 30 ksample/s with an engineered,
measured matched 8 kHz complete response, retaining the reference S-mounted
accelerometers. The study's 999 native recordings are separate from the older
3,500-case scientific-reference grid. Read
[the sensing report](../../reports/sensing-cost-study-20261002.md) for the
tradeoffs and [the execution ledger](../validation/budget_sensing_execution_ledger_20261002.json)
for executed counts, complete-history metrics and receipt hashes.

## Frozen native source

`../validation/adc_acquisition_trial_20261002/` contains the private SLX,
18 frozen helpers, baseline candidate and acquisition contract. The source
SLX SHA256 is
`b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5`.
The physical plant and causal controller remain native Simulink blocks.
Continuous response precedes sampling; the source includes SAR internal
filtering or Sinc³ decimation, ADC noise, quantization/range, converter-clock
EMA, readout delay and the 10 kHz latch. The 8 kHz response is an engineering
envelope to measure on the assembled sensors, not a supplier transfer guarantee.

The public `generated/acf_force_control_simple.slx` is unchanged and still
contains the scientific-reference acquisition. Its SHA256 is
`48287e6a17bc9d3144e3d6a77c59ecc601ba4fa26a7ec748173dc89d9bf718de`.
The older delivered ZIP is preserved. Do not replace a source or input file and
then reuse the retained results as though its hash had not changed.

## Executed cohorts

| Cohort | Recordings | Scope |
| --- | ---: | --- |
| SAR and Sinc³ bandwidth/rate screen | 168 | Four heavy/light, clean/seed-7 cases per setting |
| Selected M08 nominal range | 700 | Five masses × ten targets × seven orientations × two drag ratios × seed 29 |
| ADC repeated-bump calibration | 16 | Four 30 s cases for each of four ADC settings |
| Cheaper-acceleration nominal envelopes | 36 | Includes labeled counterfactual bandwidths, excluded from recommendations |
| Sensor/drive joint-error envelopes | 49 | Seven combinations × four 30 s and three 4 s cases |
| Tighter dynamic matching | 7 | L0 only; L50 prepared but not executed |
| Selected M08 hot winding | 7 | Kf24.4, R8.896, L4mH, 32 V usable, 100 µs current target |
| ADC noise/timestamp probes | 8 | Four one-at-a-time changes × seeds 7/29 |
| Solver refinement | 2 | Half physical step with fixed acquisition clocks |
| Quiet synthetic sensor tones | 6 | Separate from the 993 bump recordings |
| **Total** | **999** | **1,502,400,999 native force samples scored** |

Nine warm/cold compilation-parity recordings are separate software evidence.
All failed physical recordings remain in their native cohort folders.
Prepared-only cases and preparation/software rejections are excluded. The
sequential worker's deliberate handoff to two private workers is documented
in `validation/budget_sensor_parallel_handoff_20261002.json`; it does not add
extra physical cases.

## Rerun one retained comparison

Use an isolated `matlab -batch` process with its own `MATLAB_PREFDIR`, never the
user's open desktop. `qualify_adc_acquisition_20261002.m` copies the frozen
model/helpers and creates private cache/code/SDI storage. It restores the
process's SDI preferences and clears completed histories after scoring. A
30 s recording temporarily requires several gigabytes of RAM.

The following MATLAB code reruns the seven selected hot-winding cases from
their exact prepared inputs. Run from the repository or extracted study root.
The output path must be new.

```matlab
project = pwd;
addpath(fullfile(project,'acf_simulink','experiments'));
stateRoot = fullfile(getenv('HOME'),'.local','state','actuator-model');
if ~isfolder(stateRoot), mkdir(stateRoot); end
source = fullfile(project,'acf_simulink','validation','adc_acquisition_trial_20261002');
inputs = fullfile(project,'acf_simulink','validation', ...
    'budget_adc_hot_coil_20261002_inputs', ...
    'ADXL1002_8000_M08_30k_current100_hot32V_configs.mat');
output = tempname(stateRoot);
receipt = qualify_adc_acquisition_20261002(source,output, ...
    'budget_adc_selected_hot_coil', ...
    struct('customCasesFile',inputs,'useFastRestart',false));
assert(strcmp(receipt.status,'complete'));
disp(receipt);
disp(output);
```

Every executed cohort also retains `input_configs.mat`, allowing the same
custom-input call with that file. The worker's private model changes only
observation sinks; full-history parity was checked before using the compact
19-column logger. Its columns and force-balance checks are in each receipt.
Do not infer passes from generated inputs or from the maximum in a reduced plot.

The 700-case grid was executed in three disjoint partitions, using the same
source and seed 29. Their receipts are under
`validation/adc_budget_range_20261002_part1` through `part3`.
The independent warm/cold audit establishes exact history parity for the tested
seed, endpoint and cutoff changes. New clock, delay or topology changes should
run with Fast Restart off unless separately audited.

## Rebuild evidence and reports

With Python, NumPy, SciPy, Matplotlib and h5py installed:

```sh
python acf_simulink/experiments/summarize_budget_sensing_20261002.py --final
python acf_simulink/experiments/integrate_budget_sensing_report_20261002.py --complete
python acf_simulink/experiments/build_inertial_report.py --config acf_simulink/validation/budget_sensing_report_inputs_20261002.json --complete
```

The summarizer checks completion, native force balance, model identity,
receipt counts and exact 700-condition coverage. It reads full selected
histories for waveform maxima before reducing displayed samples. The report
integrator uses the preserved pre-study report for unaffected explanation.
The report app retains its identity and protected runtime; build/export it
using its own `AGENTS.md`. Keep the delivered HTML at
`reports/active-contact-flange-report.html` so its relative documentation
links resolve.

The source-level Sinc³ approximation and solver-refinement receipts are
numerical checks. Neither validates a real accelerometer's resonant phase,
ADC board timing, calibrated drift or powered-sander vibration. Prototype
acceptance remains maximum independent reference-force error plus applicable
uncertainty at most 0.45 N throughout the loaded bump recording.

The dated local delivery is bound by
`validation/budget_sensing_delivery_manifest_20261002.json`. Verify its bytes,
complete condition identities, retained failures, public-model preservation
and three selected full native histories with:

```sh
python acf_simulink/experiments/verify_budget_sensing_delivery_20261002.py
```

This streams full histories and hashes the preserved original archive, so it
needs the retained evidence files. `--seal` is an authoring operation for a
new reviewed delivery, not a way to repair a failed verification.
