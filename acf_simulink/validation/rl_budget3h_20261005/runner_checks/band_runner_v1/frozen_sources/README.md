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
