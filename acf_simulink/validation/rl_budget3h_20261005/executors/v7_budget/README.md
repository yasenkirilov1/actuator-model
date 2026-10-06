# Three-hour unified controller study — 5 October 2026

The experiment starts at **12:53:44 UTC** and has a user-authorized wall-clock
budget ending at **15:53:44 UTC**. Training is planned to stop by 15:33:44 UTC,
leaving twenty minutes for native replay, comparison and report delivery.

The physical objective remains the workpiece-normal force requirement in the
root `AGENTS.md`: ±0.5 N over the complete recording, including startup and the
1 mm / 10 mm sharp bump at 100 mm/s. One shared causal recurrent actor requests
coil current and robot recentering across identified tools and force targets.
The inner current servo, physical limits, force-stability gate and robot delay
queue remain part of the native plant/control interface.

The preceding short experiment is preserved under
`validation/rl_unified_20261005/`. Its twelve formerly held-out cases are now
development evidence. A fresh test cohort is frozen before this study's final
selection and receives no training or checkpoint selection.

## Experiment lanes

- A bounded diagnostic repeats the earlier policy family with a zero-initialized
  value head and a smaller optimizer step (`legacy_pilot.py`).
- Recurrent PPO trains a shared Puffer minGRU with a trainable direct measurement
  path, identified context and separately controlled value learning.
- Whole-episode reward search explores a compact parameterization of the shared
  actor, avoiding the need to assign a delayed robot effect to one time step.

All initialization choices, feature transformations and trainable parameters
must be reported. Reward search is an evolutionary policy-optimization method;
it must not be described as PPO. Neither lane uses expert action labels or a
dynamic PI controller beneath the learned current output. Plant truth is used
for reward and evaluation only, never as an actor observation.

## Evidence and resource rules

- Native physical calculations come from the accepted Simulink-generated C.
  New threading/packaging requires parity with the accepted serial histories.
- Use the RTX 3090 for batch optimization when its measured throughput beats
  CPU execution. Small sequential inference may remain in C on the CPU.
- Preserve every software rejection separately. Log each physical failure with
  its case, seed, exact requested action sequence and native interval scores so
  it can be replayed; retain representative full native failure histories and
  full final qualification traces. Do not imply all training waveforms are kept.
- Freeze the executing Python/C inputs for each run. Preserve prior results and
  the canonical public Simulink model. No candidate is promoted merely because
  its training reward improves.
- Select with separate complete development episodes. Lock the chosen weights
  before the final fresh tests. Score every physical sample, including startup,
  and compare with the existing deterministic controller on the same cases.
- The Markdown and offline HTML report will give actual executed counts,
  limitations, plots, and a concrete recommendation based on those results.

Status and timing are recorded in
`acf_simulink/validation/rl_budget3h_20261005/study_contract.json`.

## Budget sensor runtime v7

This executor preserves the accepted v6 policy, exporter, native C arithmetic,
final driver and environment. The evolutionary runner adds opt-in parallel
initial, selection and final audits through `--audit-workers`; its default
remains one. Serial and two-thread complete-episode statistics were bit-identical
on four development cases, recorded in
`validation/rl_budget3h_20261005/reward_search_parallel_audit_check/results.json`.

The new target is the user's combined accelerometer/ADC budget below EUR 100.
The versioned ADXL354-like/M08 native bank models an 8 g acceleration range,
33.75 micro-g per square-root hertz noise density, 100 mV/g sensitivity, matched
1.9 kHz acquisition response, 30 ksample/s raw acquisition and a 100 microsecond
actor cadence. The hot coil has 32 V usable bipolar output and a 50 microsecond
current-response target. Actual/identified calibration and timing residuals
remain distinct and are preserved by the bank recipe. This is a declared
component-response model, not assembled hardware validation.

The immutable 144-case development bank is `native_bank_budget50/`, with
metadata in `scenario_bank_budget50/manifest.json` relative to the study folder.
It retains 84 training, 28 selection and 32 untouched final identities. Raw
robot requests remain bounded to 10 mm/s before the native stability gate,
command envelope and delayed queue. Final force tolerance remains 0.5 N;
positions within 7.5 mm of center carry zero band-excess cost.

The short budget refinement starts from a retained fast-sensor checkpoint,
uses 37 constrained current-control directions, and keeps the robot mapping
fixed for an identical observation history. Target lock is 15:28 to 15:30 UTC,
leaving time for independent final tests. Executed generations, warm-start
ancestry and sensor-bank identities belong in the result ledger. No unchanged
fast-sensor result qualifies this budget sensor chain.
