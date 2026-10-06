# Active contact flange: agent context

## Current budget and learning handoff — 2026-10-05

For budget sensing, unified learned control, reruns or current report edits,
read `acf_simulink/experiments/rl_budget3h_20261005/README.md` and
`acf_simulink/validation/rl_report_20261005/final/report_inputs.json` first.
The latest report is `reports/active-contact-flange-report-rl-3h-20261005.md`,
also delivered under `/home/tommie/Documents/Notes Remote/S7 Project/`.
Preserve the older reference reports and packages below.

The user specifies **under €100 for accelerometer and ADC chips on their own
PCB**. The conditional prototype choice is one ADXL354C and one ADS131M08
(two acceleration axes plus six raw bridge channels); other electronics are
outside that subtotal. The new experiments declare a 1.9 kHz matched response,
30 ksample/s ADC, 10 kHz control and 50 µs current response with the hot coil
at 32 V usable output. Measure the installed transfer and timing before
claiming those sensor envelopes describe hardware.

The unified 2×32 minGRU checkpoint and all three cohort inputs are locked in
`validation/rl_budget3h_20261005/selected_controller/` under `acf_simulink/`.
Both controllers pass the new 700-case nominal grid. The learned controller
passes only 30/32 fresh cases (0.678521 N worst) and 3/4 long cases (one travel
stop at 19.13094 s). Keep it as research evidence. Deterministic PI/feedforward
passes all 736 force/physical cases but reaches 0.492243 N and leaves the soft
travel band in two long cases. Good worst-case margin remains unestablished.
The public SLX is unchanged and retains its scientific-reference acquisition.

Travel inside **±7.5 mm about the stroke midpoint has zero centering cost**;
report band excursions separately from force failures and hard travel stops.
Training stopped before the authorized three-hour limit. Final tests remain
evaluation evidence; any further tuning requires a new candidate and fresh
test separation. Full histories, failed cases, export checks and independent
MATLAB replays are retained in the study directory.

## Goal and current requirements

Design and optimize an active flange for sanding. The controlled quantity is
the **normal pressing force experienced by the workpiece**. Keep it within
**±0.5 N throughout the recording**, including the sharp bump, for force
targets **5–50 N** and fixed flange orientations **0–90°**.

- Sharp test: **1 mm height, 10 mm width, 100 mm/s tangential scanning**.
- Tool mass **S = 0.1–2.0 kg**. Holder/moving attachment **B = 0.6 kg** is the
  current assumption, so B+S spans **0.7–2.6 kg**. Robot-side A is currently
  0.4 kg additional. The earlier S=1.4 kg case is one test point.
- The load cell is rigidly mounted **between B and S**, with the entire S
  mass downstream. The user approves a **multi-axis load cell**.
- Position measures **S relative to A**. An internal magnet-gap measurement
  is equivalent up to sign/zero only when B–S is rigid.
- Tool weight is known or tared before use. Static tare removes gravity;
  dynamic inertia still needs compensation.
- The user also approves a **good accelerometer mounted on S**. Use realistic
  manufacturer specifications for noise, range, filter phase, acquisition
  delay and calibration; synchronize it with the force channels. Reconstruct
  the contact-force vector from the load cell and downstream S inertia before
  projecting onto the estimated surface normal.
- Controller inputs may include measured force, relative position and a
  causal prediction from **robot commands already issued**. The controller
  has **no known bump profile, surface preview or future motion**. Additional
  sensor proposals must be labeled and separately qualified.
- Compare voice coil and leadscrew, control strategies and realistic
  spring/damper values. Also test removed springs as **rigid connections**
  and zero external damping. Internal actuator friction still exists.
- Combined **free spring length and maximum installed spring length** must
  each stay at or below **100 mm**. This is a spring-only packing condition,
  not a qualified whole-flange dimension. Approximate component targets are
  wanted; a purchasing part list is unnecessary.
- Stop and mark failed at contact loss or a travel stop. Subsequent impacts
  and free flight are outside the selected model.
- Robot recentering needs a force-stability gate and appropriate delay/gain
  handling; queued commands remain relevant after the gate closes.
- Investigate performance limits, including whether higher supply voltage,
  current bandwidth, better sensing or changed mechanics improve the result.

## Start work from the correct source

The authoritative physical model is
`acf_simulink/generated/acf_force_control_simple.slx`. Parameters and presets
live in its **Model Workspace**, principally structure `p`. MATLAB helpers
prepare coefficients, apply settings, execute the native model and inspect
results. Keep one physical plant implemented in native Simulink blocks.

For running, parameter changes or controller selection, read
`acf_simulink/GETTING_STARTED.md`. For equations or sensor/control contracts,
read `acf_simulink/MODEL_NOTES.md` and inspect the saved diagram. For rerunning
retained studies, read `acf_simulink/REPRODUCING_RESULTS.md` and the relevant
recipe in `acf_simulink/experiments/`.

These guides and retained reports describe their recorded model versions.
Use the requirements above for ongoing work and update affected guides when
a new candidate is promoted. Check model, parameter and helper hashes before
reusing a retained result.

The user wants an understandable, untangled, native Simulink diagram with
shallow subsystems and useful actuator/controller selectors. Use ordinary
Gain, Sum, Integrator, State-Space, filter and limit blocks. Matrix preparation
before simulation is acceptable; a hidden MATLAB plant solver defeats the
purpose. Keep experiments fast by reusing compilation where valid rather
than sacrificing the required physics or verification.

## Experiment and qualification discipline

Run experiments in isolated **`matlab -batch`** processes with private model
copies, cache/code directories and simulation-data storage. Restore any SDI
preferences changed by that process. Preserve the user's open desktop model
and other applications; previous desktop automation targeted an unrelated
window. MATLAB should remain dark and Simulink consistently light.

The scientific reference coil is Kf=20 N/A, R=4 Ω, L=3 mH, bipolar
3.5 A/24 V, 25 mm travel and a 100 µs current-response target. Final acquisition
uses 100 ksample/s raw sampling, matched two-pole 5.1 kHz response envelopes,
a 50 µs raw-clock EMA, fresh 10 kHz controller latching and a 20 µs pipeline.
Conditioned noise targets are 0.02 N RMS per force axis, 0.014 m/s² RMS
acceleration and 1 µm RMS position, before final quantization. These are model
assumptions. The real-coil headroom tests use Kf=24.4 N/A, R=6.4 Ω, L=4 mH;
the current prototype recommendation is a 36 V supply with at least 32 V
usable bipolar output. Keep actual and identified masses, mechanics, sensor
response and robot timing separate; retune identification for each known tool.

A rigid mounting does not make the load cell infinitely stiff. The finite-cell
rigid-link reference exposes a roughly 1.64 kHz mode. The selected compliant
pair is much softer than the representative 64 MN/m cell; its joining frame
is currently massless and the tiny series-compliance correction is neglected.
Check real frame, cell and guide modes before calling a controller practical.
Structural damping, sensor calibration and driver phase require assembled
measurements. Match force, acceleration and estimated-normal timestamps;
keep any present-time actuator prediction separate from that measurement frame.
The noise values above describe conditioned output noise: inspect native
injection and the fixed nominal response normalization before changing them.
Force noise is bounded uniform and acceleration noise is Gaussian. Neither
establishes real sanding-vibration performance.

At fixed current-response tuning, higher voltage improves the modeled drive
only when voltage clipping limits current slew. Test clipping and waveform
changes; distinguish voltage changes from deliberately faster current-loop
tuning. Size continuous heating separately from peak effort.

Bind every force qualification claim to the actual normal-force model,
recorded signals and metric definition used. Evaluate full native histories,
including startup. Retain failed cases and preparation rejections separately;
planned cases and generated scripts are not executed experiments. Distinguish
nominal passes, parameter mismatch, counterfactual sensing, finite test grids
and physical prototype validation. Sample RMS noise does not establish bias,
scale accuracy, latency, phase or temperature performance.

For a promoted candidate, verify force balance, sensor location/tare, causal
controller inputs, native block structure, absence of algebraic loops, solver
refinement, save/reopen behavior and failure stops. Verify diagram readability
after wiring changes. Preserve the frozen input model/helpers and selected
native traces so another agent can reproduce the result.

Evidence lives in `acf_simulink/validation/`; recipes live in
`acf_simulink/experiments/`. `validation/inertial_delivery_manifest_20260930.json` and
`validation/inertial_delivery_package_checks_20260930.json` bind the new
measured-inertia archive. Older `delivery_*` records are historical.
When changing a delivered artifact, update its package/manifest consistently
or preserve the original package and clearly version the new delivery.

## Reports and visualizations

The requested final deliverable is a **comprehensive Markdown report**,
starting with the agreed goal and ending with **one concrete system
recommendation**. Explain the final model, control strategy, experiments,
limitations, approximate actuator/mechanical specifications, sensor needs
and accuracy, and prototype validation, with supporting graphs.

Restate a materially revised goal to the user before writing so they can
correct it. Describe the corrected final system and its actual results;
the user explicitly wants the final report to omit the narrative about the
earlier force-component correction. Report substantive limitations of the
final design honestly and preserve the internal evidence/history.

Keep HTML visualizations available. The existing report authoring project is
`reports/prototype-design-20260930/`; **read its own `AGENTS.md` before editing
it**. The current report identity and protected runtime must be preserved.
Its reviewed data are `src/data.json`, narrative is under `src/content/report/`,
and figures are under `src/content/assets/`. The standalone report is `reports/active-contact-flange-report.html`; its
relative links require that location. The native worst replay is
`acf_simulink/generated/active_contact_flange_inertial_worst_20260930.html`. The Markdown report is a
repository artifact, not an instruction to create a cloud Page.

## Parallel work and preservation

The user explicitly requested parallel experiments. Assign disjoint file
ownership and private simulation directories; workers must accommodate each
other's edits. A single integrator should promote a reviewed candidate into
the canonical SLX and shared helpers. Keep inactive alternatives usable and
test only the branches affected by a change.

Preserve unrelated dirty files, project metadata, user configuration,
checkpoints and historical evidence. Stage narrowly if requested; commit or
push only within the user's instruction. The user's authorization already
covers reversible model fixes, local experiments and documentation.

## Active handoff — 2026-09-30

The current candidate is upper-rigid with a lower opposed spring pair:
19,746 N/m net stiffness, 60 N·s/m parallel damping, 33.6 N preload per spring,
9.873 N/mm individual rate and approximately **20±0.49 mm free length**.
Combined free length is 40.98 mm. PI is 0.3 A/N and 20 A/(N·s), with a 1 kHz
feedback filter and a proper causal lower-link inverse rolled off at 1.5 kHz.
Measured S acceleration, drag reconstruction, a two-state S–A motion observer
and already-issued robot commands supply feedforward; surface truth stays in
plant/diagnostic paths. A fast 0.08 N RMS variation gate resets a 100 ms dwell
before new 100 Hz robot commands, in addition to the slow stability gate.

Scientific V8 SLX SHA256:
`73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643`.
Final candidate MAT is `validation/inertial_final_candidate_20260930.mat`
under `acf_simulink/`; SHA256:
`d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7`.
For source extraction, parameter hashes, grid identity, acquisition contracts
or reruns, read `acf_simulink/REPRODUCING_RESULTS.md` and the V8 manifest.
The older `lower_series_v1` response-envelope studies remain reference
results, separate from the final acquisition. Use result status and hashes.

The **3,500-case nominal grid is complete and audited: all pass**, with
700 five-seed groups and 1,400,003,500 native force samples. Worst error is
0.315810929 N; the exact worst at 2.5 µs is 0.315825399 N. Each of four
partitions has 875 unique identities. The two immutable execution recipes
reuse 1,060 disk-backed records and add 2,440 RAM-backed records without
changing the scientific inputs. Extra recovery runs are excluded. Read the
coverage and portable numerical manifests for identity/count/hash evidence.

Completed final-source evidence also includes nine 10 s acquisition endpoints,
72 actual spring/damper ±20% recordings, 42 sensing and 14 robot one-at-a-time
records, 60 sampled simultaneous-calibration records, four 30 s simultaneous-
error repeats, nine mixed-mechanics/calibration records, nine final combined
coil/screw recordings and 18 final electrical records (17 unique).
The original joint residual stress boundary leaves only 0.009604 N reserve
in the long worst. Exactly identifying actual k/c improves one separate mixed
corner from 0.489996 to 0.453026 N; do not assume it fixes the long sensor bias.
Four tighter-static 30 s repeats pass with 0.415311 N worst error and
0.084689 N reserve. Only sixteen static residuals halve; noise, AA/pipeline,
robot timing and current response remain unchanged. Three matched known-k/c
mixed cases using the same static targets pass at 0.376938 N worst. These are
finite paired cases; they do not prove the entire tighter residual box or
physical sanding performance. Use the tighter calibration column in the report
for prototype targets, and identify assembled k/c rather than catalog values.

The public model has been promoted, SHA256:
`48287e6a17bc9d3144e3d6a77c59ecc601ba4fa26a7ec748173dc89d9bf718de`.
Acceptance proves source-history parity, direct Run after edits, the exact
slow-bump guide example (0.07328 N worst), both physical failure stops and
save/reopen. Thirty-three unused wire stubs were removed while preserving
all connected edges, blocks and workspace values. The delivered diagram PDF
has 13 lossless raster pages; it is not a vector export. Old canonical files
and the user's unsaved desktop state are preserved.

The conditional real-hot-coil screen supports 31 V usable output on the
nominal grid through 3,498 conservative bounds and two explicitly simulated
exceptions. This is not 3,500 real-winding simulations. The seven tighter-static long/mixed histories receive a separate pointwise
hot-winding screen: one request reaches 31.1071 V, so specify at least 32 V
usable output. It gives 0.8929 V minimum recorded mapping reserve. Continuous
hot force, Kf over used stroke, driver dropout/phase and thermal operation
require bench qualification. The reference RE25 screw passes the four transient comparison
conditions but exceeds its continuous-current rating in all of them.

Each simulation uses private model, cache, preferences and SDI storage. Check
available RAM before concurrent MATLAB processes. Clear SDI after each case
and release full histories after scoring. Use only owned temporary folders
for RAM-backed SDI, restore its preference before cleanup, and preserve all
failed/native and software-rejected attempts separately. Recorded disk versus
RAM-backed SDI parity covers all 72 channels; the paired warm pilot was about
5.84× faster under its recorded load. That is a host-specific benchmark.

For report updates, read `reports/active-contact-flange-report.md` and
`acf_simulink/validation/inertial_report_inputs.json`; inspect the exact source
JSONs named there. Root integrates reports/helpers/delivery; completed workers'
scientific evidence remains immutable. The completed delivery is
`acf_simulink/generated/active_contact_flange_inertial_20260930.zip`. Its
manifest binds selected full histories, portable frozen inputs, the report,
graphs, replay and diagram. Preserve the historical package. Run
`python acf_simulink/experiments/verify_inertial_delivery.py` in a fresh
extraction before edits to verify delivered bytes and finite-grid identities.
Browser/replay and package checks are separate from physical qualification.
