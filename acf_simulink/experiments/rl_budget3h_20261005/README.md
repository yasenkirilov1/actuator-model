# Unified recurrent controller: run and reproduce

This folder contains the private unified-controller study of 5 October 2026.
One shared Puffer minGRU policy requests both coil current and robot correction.
Its objective is workpiece-normal force within **±0.5 N over the entire
recording**, including startup and the **1 mm high, 10 mm wide sharp bump at
100 mm/s**, for S = 0.1–2 kg, targets 5–50 N and fixed orientations 0–90°.
Travel within **±7.5 mm of midstroke** is acceptable; there is no centering cost
inside that band. Contact loss and the 0/25 mm physical stops still fail a run.

The physical calculations come from the retained native Simulink model,
compiled to C. Python does not replace its physics. These experiments do not
promote a controller into the public SLX. To open and run the delivered baseline
in MATLAB, use [GETTING_STARTED.md](../../GETTING_STARTED.md).

## Run the saved controller

From a terminal, paste:

```bash
cd /home/tommie/Projects/actuator-model
/home/tommie/Pesten/puffer4-native/.venv/bin/python \
  acf_simulink/experiments/rl_budget3h_20261005/run_saved_policy.py
```

This reproduces the locked **32-case paired test**: the existing PI/feedforward
controller and the saved learned controller receive identical physical cases
and noise seeds. The wrapper creates a new timestamped directory under
`acf_simulink/validation/rl_budget3h_20261005/reproductions/` and prints its path.
Existing results and the saved controller are preserved. Until the final
`selected_controller` package and its selection reviews exist, it refuses to
start.

The launcher reads **`selected_controller/cohort_manifest.json`** for the
actual bank, native binary, review and their hashes. It has no fallback to the
earlier sensor chain. The locked budget-sensor winner therefore runs against
its own physics and acquisition, including its current-loop tuning. Its
checkpoint SHA256 is
`ca74a1be34ff2cb801f2a094aeae2e30968f52b3aaec6aacea216ca2b11ddaef`.
The selected sensing model uses ADXL354C plus ADS131M08, a 1.9 kHz matched
response envelope and 50 µs current response. Qualification results must be
read from completed recordings, not inferred from this recipe.

To choose another retained cohort:

```bash
/home/tommie/Pesten/puffer4-native/.venv/bin/python \
  acf_simulink/experiments/rl_budget3h_20261005/run_saved_policy.py --cohort nominal700

/home/tommie/Pesten/puffer4-native/.venv/bin/python \
  acf_simulink/experiments/rl_budget3h_20261005/run_saved_policy.py --cohort long4
```

| Cohort | Cases per controller | Recording length | Storage and default processes |
| --- | ---: | ---: | --- |
| `holdout32` | 32 | 4–6 s | Every native history; 2 processes |
| `nominal700` | 700 = 10 masses × 10 forces × 7 orientations | 2 s | Every native sample scored; full worst/failure histories retained through exact replays; 4 processes |
| `long4` | 4 | 30 s | Every native sample scored; full worst/failure histories retained through exact replays; 2 processes |

Storage modes and process defaults come from the locked cohort manifest.
The grid uses fresh seeds and nominal calibration. The longer cohort contains
repeated sharp bumps and two sustained axial robot drifts. These are finite
simulation tests, not a prototype validation or a proof of every uncertainty
combination. Long recordings need several gigabytes of working RAM per process.
Use `--workers 1` to reduce concurrent memory use, `--output /absolute/new/path`
to choose a new destination, or `--dry-run` to validate the package and print
the command without simulating. The wrapper exposes no alternate checkpoint,
physical parameters or selection-review overrides.

## What the saved package contains

The locked package lives at
`acf_simulink/validation/rl_budget3h_20261005/selected_controller/`:

- `policy_checkpoint.pt`: architecture, tensors, named features and ancestry.
- `policy_inference.c` and `.h`: deterministic CPU inference.
- `policy_weights.f32`, `.json`, `.npz`: flat weights, layout and inspectable arrays.
- `policy_export_manifest.json`: hashes of those exported artifacts.
- `source/`: exact executing Python/C source and `frozen_source_manifest.json`.
- `cohort_manifest.json`: the checkpoint identity, reviewed selection bank,
  and each cohort's bank/native/review paths, hashes, split and storage mode.
- `selection_review_holdout.json`, `_grid.json`, `_long.json`: checkpoint,
  physical-source and bank hashes, all reviewed selection IDs and their evidence.

All cohort-manifest paths are relative to the study directory, including
`selected_controller/...` review paths. Each cohort binds the bank, native
library, metadata, scenario overlay and build seal separately. The launcher
verifies these hashes before calling the frozen
`source/final_evaluate.py`. The evaluator freezes inputs again before any
physical outcome, checks the model/library/parameter-bank seals and acquisition
clocks, and checks C/Python feature construction. A mismatched action scale or
feature list is a software rejection, not an alternative test configuration.

Inspect the new output's `results.json` first. `status: complete` and
`lockedInputsUnchanged: true` mean the requested evaluation finished; force
passes and failures are separate fields. `input_lock.json`, `actor_contract.json`
and `artifact_manifest.json` identify its inputs, readout activity and files.
Full records contain FP64 native timestamps, physical truth and requested
actions. Every physical sample at about **3.333 µs**, including time zero and
the final sample, enters force metrics. Display plots may use extrema-preserving
bins. Aggregate grid replays are counted separately from unique recordings.

## Policy and exact C contract

The actor receives **40 named causal features**: 18 native measured/observer
values, 21 known physical/acquisition/timing values and the 100 Hz robot phase.
Eight additional causal products are computed inside the policy, including
identified mass × measured acceleration / force constant. A trainable
measurement path and a shared **32-state, two-layer minGRU** feed both action
means. A critic shares the recurrent representation during training. Plant
truth, future robot motion and the bump profile never enter actor inference.
See the [shared recurrent controller diagram](../../validation/rl_budget3h_20261005/figures/unified_recurrent_control_band10_v3.svg).

Use the exported header and layout, rather than reconstructing tensor order.
The single-context entry point is:

```c
void acf_unified_policy_step(const float *weights, const float *observation,
    float *state, int reset_before, float *mean, float *value);
```

`observation` has 40 FP32 values; `state` has 64 FP32 values in layer-major order.
`mean` receives two latent action means and `value` receives the critic value.
The flat FP32 buffer has **8,181 values**: 8,101 stored neural parameters plus
80 normalization values. Its precise offsets are in `policy_weights.json`.
The batch entry point uses independent `[batch, layer, hidden]` state buffers;
Torch training uses `[layer, batch, hidden]` instead.

At each **100 µs actor decision**, carry the updated state into the next call.
Reset it only at an episode/tool-context reset, not when a robot gate closes.
Evaluation uses deterministic C means, with the study's exact action mapping:

- Current request = identified static equilibrium current + **2 A × tanhf(mean[0])**.
  The policy supplies full dynamic current control. The native current servo,
  notch, electrical/current limits and antiwindup remain; no dynamic force PI
  sits beneath the learned request.
- Raw robot correction = **10 mm/s × tanhf(mean[1])**, considered only on the
  native **100 Hz** command clock when the observed force-stability gate is open.
  Native delay-aware sign, amplitude and velocity constraints apply afterward.
  Commands already queued still move the robot after the gate closes.

The accepted C evaluator compiles with `-fno-fast-math -ffp-contract=off`.
Retain its arithmetic and all affine biases/normalization. Some inverse-model
initializations use large cancelling readouts: tiny C/Torch recurrent rounding
differences can then fail the PPO behavior-replay guards. Final C recordings
qualify the actual exported C actor; substituting Torch inference changes the
implementation being tested.

## Train a new candidate

Use the existing local runtime:

```bash
/home/tommie/Pesten/puffer4-native/.venv/bin/python -c \
  'import torch, pufferlib; print(torch.__version__, pufferlib.__version__); print(torch.cuda.is_available())'
```

Study executors are preserved in `validation/rl_budget3h_20261005/executors/`.
Each numbered version has source hashes. Run a frozen version for a known
recipe; changing current experiment files does not change an old executor.
The examples below use **v6**, the earlier 144-case sustained-drift development
bank with the earlier sensing chain, and the preserved gen5 warm start.
They are historical development recipes, not the budget-sensor finalist's
training or its evaluation. Use the locked cohort manifest to identify a final
candidate's actual sensing, current response and bank; do not substitute one
physical/acquisition bank for another. Training is stochastic; the original run's
arguments, seed, sources and ancestry remain in its own evidence directory.

The development bank has **84 training**, **28 selection** and **32 held-out**
identities. Training IDs are 0–47, 96–119 and 128–139; selection IDs are 48–63,
120–127 and 140–143. IDs 64–95 must not enter training or selection. The earlier
128-case bank's axial motion returns to zero; the additional 144-case bank
contains sustained drift. They are distinct development cohorts.

A bounded recurrent PPO run:

```bash
acf_run_id="$(date -u +%Y%m%dT%H%M%SZ)"
/home/tommie/Pesten/puffer4-native/.venv/bin/python \
  acf_simulink/validation/rl_budget3h_20261005/executors/v6/train.py \
  --native-build acf_simulink/validation/rl_budget3h_20261005/native_bank_sustained \
  --bank acf_simulink/validation/rl_budget3h_20261005/scenario_bank_sustained/manifest.json \
  --resume-policy acf_simulink/validation/rl_budget3h_20261005/reward_search_band_8203/checkpoints/generation_0005/policy_checkpoint.pt \
  --output "acf_simulink/validation/rl_budget3h_20261005/reproductions/ppo_${acf_run_id}" \
  --seed 9401 --device cuda --torch-threads 2 --native-threads 2 \
  --decision-repeat 1 --environments 24 --rollout 4096 --updates 40 \
  --chunk-length 2048 --burnin-steps 256 --minibatch-sequences 8 \
  --actor-learning-rate 0.00001 --critic-learning-rate 0.0003 \
  --log-std-learning-rate 0.00001 --reward-scale 10 \
  --gamma-time-s 1 --lambda-time-s 0.3 --force-lambda-time-s 0.02 \
  --force-error-scale-N 0.30 --force-excess-weight 25 \
  --acceptable-travel-radius-m 0.0075 --travel-excess-weight 10 \
  --selection-initial --eval-every 20 --max-seconds 600
```

`--decision-repeat 1` explicitly retains the selected study's 10 kHz actor
clock; the runner's generic default is different. `--resume-policy` imports
weights with a fresh optimizer. Exact continuation instead uses `--resume`
with `latest_training.pt`, including recurrent collector state, queues,
optimizer and RNG state. Do not combine those options. Time-limit bootstraps
use a cloned recurrent state; contact loss and stops use terminal values.
Learning-rate, critic and per-action GAE choices are recorded in the run.

A bounded whole-episode reward-search run over 55 parameter directions:

```bash
acf_run_id="$(date -u +%Y%m%dT%H%M%SZ)"
/home/tommie/Pesten/puffer4-native/.venv/bin/python \
  acf_simulink/validation/rl_budget3h_20261005/executors/v6/reward_search.py \
  --native-build acf_simulink/validation/rl_budget3h_20261005/native_bank_sustained \
  --bank acf_simulink/validation/rl_budget3h_20261005/scenario_bank_sustained/manifest.json \
  --policy acf_simulink/validation/rl_budget3h_20261005/reward_search_band_8203/checkpoints/generation_0005/policy_checkpoint.pt \
  --parameter-spec acf_simulink/validation/rl_budget3h_20261005/executors/v6/reward_search_band_spec.json \
  --output "acf_simulink/validation/rl_budget3h_20261005/reproductions/es_${acf_run_id}" \
  --mode search --indices train --seed 9402 --workers 3 \
  --population 12 --generations 20 --minutes 10 --sigma 0.12 \
  --train-batch-size 12 --selection-every 5 \
  --force-excess-weight 25 --band-objective --robot-scale 0.01
```

Reward search is evolutionary policy optimization, not PPO. Its parameter
specification gives the evolved directions; it does not train all 8,101 stored
parameters. The later partial-inverse refinement uses a separate 37-direction
specification retained with its run. Known-mechanics initialization, filter
initialization and reward-evolved changes must be reported separately. Neither
lane uses teacher action labels. Actual normal force may supply reward and
training diagnostics while remaining absent from inference.

Use complete selection episodes to choose a checkpoint; `latest_training.pt`
or the last generation is not automatically the best policy. Preserve
software rejections, physical failures, source hashes and training weights.
The earlier `validation/rl_unified_20261005/` tests are development evidence
for this study and must not be described as fresh qualification.

## Final evaluation discipline

`final_evaluate.py` requires the deliberate `--run-locked-holdout` flag and a
review written **after selection and before reading final outcomes**. The
review binds `selectedCheckpointSha256`, `evaluationBankSha256`,
`sourceModelSha256`, `selectionOnly: true`, all 28 `selectionIDs` and hashed
`selectionEvidence`. Separate nominal/long qualification banks also require
`selectionBankSha256` and `--selection-bank`. Do not generate a review merely
to bypass these checks or retune on a fresh cohort after seeing its results.
The saved-policy wrapper supplies the already reviewed files.

For a new implementation check before selection, use `--fixture-train` with
TRAIN IDs and, optionally, a short `--fixture-duration`. A fixture is not a
full sharp-bump result. Learned-memory reset ablations are likewise restricted
to TRAIN fixtures; they leave native sensors, observers and queued commands
unchanged. Read the driver help for this expert interface.

The authorized study ran within a three-hour wall-clock budget starting
12:53:44 UTC on 5 October 2026. Its contract, executed ledgers, per-run failures
and locked final results live under `validation/rl_budget3h_20261005/`.
Report preparation uses those recorded statuses and hashes. Planned tests,
generated banks and interrupted work must not be counted as completed cases.
