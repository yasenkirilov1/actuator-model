# Validation record

## Current measured-inertia design, 2026-09-30

The current work uses the native normal-contact model, a multi-axis B–S cell,
S-mounted accelerometer, S-relative-A position, causal issued-command robot
prediction, a lower guided spring pair/parallel damper and an upper rigid link.
The design target is true surface-normal pressing force, ±0.5 N across the
complete native recording. See the [current report](../reports/active-contact-flange-report.md)
and [reproduction guide](REPRODUCING_RESULTS.md) for each study's exact scope.

Current completed evidence includes:

- [Final acquisition freeze](validation/normal_contact_trial_inertial_v8_manifest.json):
  nine10s endpoint records pass, worst0.322415N; full source/known-mass bridges.
- [Actual mechanical variation](validation/inertial_mechanics_v8_summary_20260930.md):
  72 actual k/c ±20% records meet the force band, worst0.374370N. The
  [20mm geometry rescore](validation/inertial_mechanics_v8_20mm_geometry_rescore_20260930.json)
  makes all72 packing checks pass at33.6N preload; it adds zero force runs.
- [Calibration evidence](validation/inertial_uncertainty_v8_summary.json):
  42 sensing OAT,14 robot-response and60 simultaneous-error records pass.
  Joint worst0.448085N; two endpoints/ten26-dimensional corners/three seeds,
  not exhaustive or full-range joint uncertainty. Six software proofs are separate.
- [Current-response alternatives](validation/normal_mass_controller_final_performance_20260930_summary.json):
  12 repeated-heavy10s records pass.50/100/150µs worst0.312708/0.345322/0.372302N.
  Selected100µs; higher voltage leaves unclipped histories bitwise unchanged.
- [Real electrical headroom](validation/normal_voltage_20260930_final_sensor_summary.json):
  Original16 native runs/15distinct conditions. Successful hot-coil31V output matches
  ideal36V;28.473V peak request. Four19/20V hot-output failures remain retained.
- [Complete nominal grid](validation/normal_mass_controller_final_grid_20260930_summary.json):
  **3500/3500 pass**, exact/disjoint coverage and700 five-seed groups audited;
  worst0.315810929N. Every400001-sample history is scored, including startup.
  The two archived recipes reuse1060 native rows and add2440, with122 extra
  recovery records excluded. All fixed33.6N-preload packing checks pass.
- [Final numerical refinement](validation/normal_mass_controller_final_grid_20260930_solver_refinement_portable_summary.json):
  exact global worst at10/5/2.5µs is0.315810929/0.315822976/0.315825399N.
  Both qualified fine histories pass; rejected software audit attempts remain
  separate. All72 loggers are compared on their actual time grids.
- [Long joint-error repeats](validation/inertial_uncertainty_v8_joint_long/inertial_uncertainty_budget_repeated_20260930_results.json):
  four30s histories pass,3,000,001 native samples each. Worst0.490396047N,
  leaving only0.009604N stress reserve. The selected tighter static calibration is
  separately recorded below; catalog accuracy does not establish it.
- [Tighter static calibration](validation/inertial_uncertainty_v8_calibrated_static_long/inertial_uncertainty_calibrated_static_long_20260930_results.json):
  four paired 30 s histories pass, worst 0.415311 N and 0.084689 N reserve.
  Exactly sixteen static residuals halve; noise, acquisition, robot timing,
  actual mechanics and current response remain fixed. The lower quiet-force
  offset explains almost all of the improvement; transient excess barely changes.
- [Calibrated mechanical stress](validation/inertial_mechanics_calibrated_static_mixed_comparison_20260930.md):
  three new 4 s records at the known actual k/c corner pass, worst 0.376938 N;
  their three reused baselines and exact static-only guards are retained.
  This does not qualify the full combined uncertainty box.
- [Assembled mechanics identification](validation/inertial_mechanics_mixed_comparison_20260930.md):
  six4s histories at one heavy combined corner. Original mixed worst0.489996N;
  exactly known actual k/c supplied to the inverse improves it to0.453026N.
  This is a paired three-seed sensitivity, not a residual identification budget.
- [Final actuator comparison](validation/inertial_actuators_final_actual_acquisition_v8_20260930_reviewed_comparison.json):
  all9 pass: four matched coil/screw4s pairs plus one5µs screw refinement.
  The RE25 continuous rating fails in all four load cases despite transient passes.
- [Full-grid hot headroom](validation/normal_voltage_final_grid_hot_mapping_20260930.json):
  3498 conservative bounds plus2 native exceptions support31V usable output
  under the stated nominal known-Kf/matched-response assumptions. Exactly two
  extra native records give18 final electrical runs/17 unique; this is not
  3500 real-coil runs, thermal qualification or joint-budget headroom proof.
- [Calibrated hot-winding screen](validation/normal_voltage_calibrated_static_headroom_20260930.json):
  seven existing histories, 13,200,007 native samples and zero new simulations.
  One 10 µs request reaches 31.107069 V; 32 V usable output leaves 0.892931 V
  demand reserve under the stated mapping. No native 31 V force failure or
  32 V real-winding force qualification is inferred.
- [Public native acceptance/promotion](validation/normal_contact_trial_delivery_manifest_v8_20260930.json):
  source-history parity, direct Run, exact slow guide0.07328N, both physical
  failure stops and save/reopen pass.33 wire stubs removed with connected
  edges/parameters/workspace preserved;13 diagram pages exported and reviewed.
- [Actual worst replay](validation/inertial_native_worst_replay_browser_20260930.json):
  full native peak0.315810929N retained exactly, Worst error button seeks
  2.00063s, no invented failure events and readable mechanism/limits display.

Numerical passes support the modeled tested envelope; physical sanding remains
a prototype validation task. Separate mechanical and sensing tolerances cannot
be combined into a guaranteed joint allowance from their independent tests.

## Historical evidence

The remainder records older snapshots and workflows. Its words “current” or
“final” refer to those checkpoints, including superseded rigid-link controllers,
older signal definitions and packages. They are not the current design's
qualification. Preserve them as history; use the report and source hashes above
for the delivered measured-inertia model.


The saved `generated/acf_force_control_simple.slx` is the only active physical
implementation. Its blocks execute in Simulink; MATLAB helpers provide settings,
run orchestration, metrics and plots/replay. Builders, duplicate script physics,
and superseded models were archived outside the project on 2026-09-29.

## Existing executed evidence

- [Conventional model comparisons](validation/simple_results.json): 15 maintained-
  contact cases against the former MATLAB reference. Maximum force difference
  0.000191297 N and position difference 6.74868e-8 m. Contact/travel failures,
  seeded-noise repeatability and timestep convergence were checked.
- [Layout and selector checks](validation/layout_results.json): all 18 pages had
  zero block overlaps and zero wire/block interior intersections. Nine before/
  after cases compared all 35 recorded signals, including both failure stops;
  the largest difference was 5.91e-14. The actual selector dialog, save/reopen
  behavior and preservation of custom parameters were checked.
- [Earlier expanded RK4 comparisons](validation/native_results.json): historical
  evidence for the superseded model, not the current workflow.

The cleaned native model's six-second baseline measured a 0.655-second median
across three warm runs, with full logging and no plots. Startup and window
opening are excluded. Historical reference-comparison code is available in the
external archive, not as an active simulation backend.

## Model-first cleanup checks

The [executed cleanup check](validation/model_primary_results.json), run in
MATLAB R2026b, found **zero difference in all 35 recorded signals** for each
of the three actuators before/after removing the redundant files. The actual
`acf_experiment` script passed with B+S=2 kg and a peak force error of
**0.0828277 N**. It also verified that:

- direct Simulink edits survive open, run and sweeps;
- parameter reads use the model workspace and current Stop Time/solver step;
- switching actuators selects the matching native variant;
- sweeps and replay use the saved blocks, without rebuilding;
- saved/reopened model edits are retained;
- removed files are absent from active callbacks and MATLAB dependencies.

At that cleanup checkpoint, the active package contained 15 helpers and one physical SLX model. The 18
removed MATLAB files and eight superseded models are preserved in
`/home/tommie/.local/state/actuator-model/before-model-primary-20260929-221938/removed/`.
These are software checks with illustrative hardware values, not measurements
of a physical flange.

## Controller guide example

The [proportional-only comparison](validation/control_guide_results.json)
disabled both voice-coil force-error integration and the integrator's
anti-windup tracking input, retaining the holding command. For the slow-bump
case, PI peaked at 0.0828277 N error and proportional-only control at
0.115664 N; both passed the 0.5 N band. This checks the documented workflow
for this case, not general superiority of either strategy.

## Final actual-sensor qualification, 2026-09-30

The current saved default uses a rigid R–A attachment and rigid B–S attachment,
with a voice coil between A and B. The load cell is between B and S with 1.4 kg
downstream. Position is S relative to A. B+S is 2 kg; A adds 0.4 kg. There are
16 top-level MATLAB helpers and one authoritative physical SLX.

- [Final range](validation/rigid_sensor_qualification_final_rigid_dense.json):
  **350/350** native runs passed: ten force targets (5–50 N), seven fixed
  orientations (0–90°), and a clean run plus four noise seeds at every point.
  Worst axial force error was **0.426347443 N**, leaving **0.073652557 N**
  margin. The 1 mm × 10 mm bump was scanned at 100 mm/s. The worst case was
  50 N/60°, seed 29. Its independent 25 µs repeat passed at 0.425959764 N;
  aligned contact-force waveforms differed by at most 0.000455611 N.
- [Saved-model checks](validation/final_saved_model_results.json): **815 native
  blocks across all actuator/controller variants**, zero hidden code blocks,
  zero algebraic loops and maximum subsystem depth three. These counts include
  inactive alternatives. The independent worst-case regression matched all
  **35 logged signals and all observer states exactly**. Rigid force balance
  residual was below 9×10⁻¹⁵ N. Five flat-contact actuator/controller smoke
  cases passed; contact loss and travel-stop fixtures stopped and marked failed.
- [Save/reopen](validation/final_save_reopen_results.json) and
  [final diagram cleanup](validation/final_diagram_cleanup_results.json): exact
  native regression after saving/reopening. All 24 diagram pages have zero
  block overlaps and zero wires through block interiors. Labels use the S–A
  sensor frame. Three dangling wire branches were removed; the observer's
  worst recording and a PI flat-contact recording remained exactly unchanged.
  This geometry audit does not prove arbitrary future edits stay tidy.
- [Delayed recentering](validation/final_robot_gate_results.json): the final
  contact-estimate gate and delay-based gain cap passed clean/noisy 80 ms and
  3 s delay tests. Gate alone still reached a travel stop in the long-delay
  fixture. At 3 s delay the noisy capped case retained a 2.05 mm offset after
  60 s: safety is demonstrated for this test, rapid centering is not.

One four-second worst-case run, including coefficient preparation and all
logging, took **2.454 s** in the verification process. This is a single timing,
not a median, and is not comparable to the earlier six-second baseline above:
sampling, controller and logged states changed. A checked Riccati doubling
solver removes the former lengthy coefficient-preparation iteration; native
regressions preserved force within 3×10⁻⁹ N for five checked cases.

## Calibration failures and physical scope

The nominal pass assumes synchronized 10 kHz sensing, known 100 µs channel
delays, 0.02 N RMS force noise and 1 µm RMS position noise. The current model
does **not** demonstrate tolerance to uncalibrated prototype hardware:

- Unknown coil force constant **±0.1%** failed every tested noisy high-load
  precision case. Extra unknown **100 µs force delay** caused contact loss.
- Earlier tight mass sweeps changed the identified inertia coefficient only;
  actual physical mass and exact gravity tare stayed fixed. Separate timing,
  bias and winding studies mostly used the earlier 20 ms gate. They are not
  a joint robustness qualification of the final 100 ms gate.
- [54 bounded estimator-uncertainty experiments](validation/controller_current_uncertainty_summary.json)
  produced no fully passing six-case cohort. The nine-state nominal estimator
  remains selected; the experimental tenth state was not promoted.
- Force qualification is **axial**, not surface-normal or three dimensional.
  Robot/load-cell structural compliance, cooling, friction identification,
  temperature drift, changing orientation, impacts after failure and fatigue
  are outside this qualification.

The parallel lanes retain their own execution records and sensing assumptions.
The spring study contains **400 distinct compliant designs**, 1,600 nominal
corner runs and 100 noisy stress runs. The other lanes retain thousands of
native executions; reruns, debug recordings, preparation rejections and
interrupted work are kept distinct from successful parameter points. The
[HTML report](generated/prototype_design_report.html) exposes reviewed datasets
and graph sources. The [replay](generated/sharp_bump_and_spring_sweep.html)
contains two actual-sensor histories, each with all 80,001 native samples.
