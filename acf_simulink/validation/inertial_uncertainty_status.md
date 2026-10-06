# Inertial uncertainty study status

The lower-series controller has completed the final V8 one-factor, finite
combined-corner and repeated-record uncertainty studies. Four paired30-second
records also qualify a narrower static-calibration target at the selected
stress corners. These are executed finite native experiments. This file records
this agent's work separately from the controller, mechanics, electrical and
actuator studies.

The score is the maximum absolute error of native `normalContact.data(:,1)`
against the requested normal pressing force, over **every recorded sample,
including startup**. Contact loss, travel stops, incomplete runs and overcurrent
also fail a run. Force balance is checked independently. No physical surface
profile or true acceleration is provided directly to the controller.

## Executed histories

| Frozen source / bounded cohort | Native cases | Passed | Worst normal error | Evidence |
| --- | ---: | ---: | ---: | --- |
| V2 rigid zero-instrument smoke, heavy seed1729 | 1 | 1 | 0.349923 N | `inertial_uncertainty_smoke_20260930_results.json` |
| V3 rigid zero-instrument smoke, heavy seed1729 | 1 | 1 | 0.349923 N | `inertial_uncertainty_v3_smoke/` |
| V3 finite64 MN/m cell, narrow total-current notch, heavy seed29 repeated10s | 1 | 0 | 0.544395 N | `inertial_uncertainty_finite_provisional/` |
| V4 finite cell, feedbackLP1000 Hz, heavy seed29 repeated10s | 1 | 1 | 0.470635 N | `inertial_uncertainty_v4_feedback_provisional/` |
| V4 finite cell, broad current notch, heavy seeds7/29/1729 repeated10s | 3 | 1 | 0.501261 N | `inertial_uncertainty_v4_broad_notch_provisional/` |
| V6 finite cell, measured-drag ratio5ms, feedbackLP1000 Hz, lead0, heavy seeds7/29/1729 repeated10s | 3 | 1 | 0.509138 N | `inertial_uncertainty_v6_ratio_provisional/` |

These are **10 native executions**, including two equivalence checks, with five
passes and five failures. There were no preparation/contract rejections in
these executions. Their sensor calibration, identified mass and robot timing
were matched: only the declared seeded sensor noise and repeated bump were
present. A passing single seed is not a qualification of that candidate.
In particular, other agents also found four-second failures for the V4
feedback-only candidate. None of these candidates is promoted by this lane.

Full failed native histories are retained. V6 retains all three full histories.
Source-model/helper/candidate/recipe hashes and exact parameter structures
are in each result file and its private snapshot. V2/V3 instrument insertion
was bitwise identical to the corresponding reference histories, including
native plant, contact, accelerometer, motion-observer and gate records.

Read-only exact peak decomposition JSON/CSV accompanies the provisional
finite-cell, V4-feedback, V4-broad-notch and V6 records. The generic
`inertial_uncertainty_native_diagnosis.py` reproduces the latter diagnostics
from the retained MAT histories. It never supplies a signal to a simulation.

## Completed V7 matched-acquisition timing screen

This separate screen holds PI gains at 0.06/20, feedback filtering at 1 kHz
with damping 1.5, guide-velocity prediction at 400 µs, current response at
100 µs, and measured drag-ratio filtering at 5 ms. Total-current notch and
the force-variation gate are disabled. The lower link is the 64 MN/m finite
cell with zero modeled damping; the upper link is rigid. Each configuration
has four heavy S=2 kg/F=50 N/90°/µ=0.3 records (clean and seeds7/29/1729),
and one S=0.1 kg/F=5 N/90°/seed7 record. All record 10 seconds and seven bumps.

| Matched force/IMU filter and transport | Direct acceleration FF: worst / passes | 1 ms FF command filter + causal lead: worst / passes |
| --- | ---: | ---: |
| 100 µs filter, 100 µs transport | 0.510097 N / 4 of 5 | 0.555579 N / 3 of 5 |
| 100 µs filter, zero extra transport | 0.780593 N / 0 of 5 | 0.767594 N / 0 of 5 |
| 150 µs filter, zero extra transport | 0.597303 N / 2 of 5 | 0.596500 N / 1 of 5 |
| 200 µs filter, zero extra transport | **0.407597 N / 5 of 5** | 0.435253 N / 5 of 5 |

There are **40 unique timing configurations**, with 20 passes and 20 normal
force failures. An external termination (exit143, cause unknown) left a final
34-case checkpoint although its last printed progress was32. An eight-case
private recovery thus repeated original cases33–34; those two native histories
are bitwise identical and retained as benchmarks. The resulting **42 native
executions** are not 42 distinct qualification cases. Across this screen and
the earlier cohorts, this lane has recorded 52 executions: 27 passes and 25
failures, with no preparation/contract rejections. All 40 unique timing cases
completed with no contact/travel/current guards or voltage/effort clipping.

The best direct point has a 0.092403 N nominal force margin. It is an endpoint
response-target result, not an uncertainty allowance or qualification of the
entire mass/force/orientation range. Force output noise stays at 0.02 N RMS;
IMU output noise scales as `.0123*sqrt(100us/filterTime)` in this immutable
screen. Exact native EMA ENBW scaling would increase its 200 µs value from
0.00869741 to 0.00895447 m/s²; the adjusted parameter target is prepared but
**unexecuted**. Neither the filter coefficient nor zero additional sample
transport establishes an exact ATI/ADXL hardware transfer. Analog phase and
mounted response need identification and acceptance tests.

Read-only native guide reconstruction explains why simply removing transport
at the fixed guide setting can worsen the result: around the second bump apex,
heavy seed7 guide direction changes 170 µs before physical coil direction at
the original timing, 314 µs early at 100 µs/zero transport, and 272 µs early at
150 µs/zero transport. At the worst faster point, 0.616741 N of its 0.780593 N
error comes from B motion relative to the finite cell; clipping remains zero.
This is a fixed-controller sensitivity, not proof that faster acquisition
cannot improve a retuned controller.

The assembled evidence and standalone HTML/SVG/PNG comparison are
`inertial_uncertainty_v7_timing_summary.*`. Original checkpoints and all full
native histories remain under `inertial_uncertainty_v7_matched_acquisition/`
and `inertial_uncertainty_v7_matched_acquisition_remainder/`; source, candidate,
helper and recipe hashes are retained. Unity-instrument V7 traces agree with
the independent controller worker within 1.49e-10 N, with bitwise sample grids
and gate decisions. This is numerical equivalence, not full bitwise parity.
The two actual recovery duplicates have full bitwise parity in
`inertial_uncertainty_v7_recovery_duplicate_equivalence.json`.

`inertial_uncertainty_v7_timing200_screened_candidate.mat` reproduces the
screened point. `inertial_uncertainty_v7_timing200_exact_enbw_target_candidate.mat`
contains the prepared, unexecuted increased-noise target. Neither is promoted
by this lane.

## Completed lower-series one-factor uncertainty study

The new frozen model is `lower_series_v1`, SHA256
`a0c2e358503d51c53c630ca436f20c1a618c5d024f98dd46338ee328f6c94f81`,
with candidate SHA256
`94fcfa29c708c705c2dccf3db34296568fd90f53f6b3b51cfe929e5a52e53738`.
Its lower link is a 19,746 N/m opposed spring pair plus 60 N·s/m external
damper. The load cell has 64 MN/m massless series compliance. The upper link
is rigid. PI is 0.3/20 A/N and A/(N·s), with 1 kHz Butterworth feedback
filtering and a proper 1.5 kHz identified lower-link inverse. This cohort
retains legacy matched 100 µs EMA filtering plus 100 µs sensor transport.

Six native recordings qualify the private residual instrumentation and
Accelerator mode for **two endpoint configurations**. Uninstrumented Normal
versus zero-instrument Normal and Normal versus Accelerator are bitwise
identical across all 72 signal channels and sample grids. These six executions
are paired software proofs, not six distinct operating points. All pass.
See `inertial_uncertainty_lower_v1_instrumentation_parity.json`.

| Cohort | Native cases | Passed | Failed | Worst normal error |
| --- | ---: | ---: | ---: | ---: |
| Core calibration/noise/position/robot OAT | 96 | 91 | 5 | 0.777479 N |
| Identified holder B mass ±0.5/1% | 8 | 8 | 0 | 0.309876 N |
| Issued scan-speed identification ±1/3/5% | 8 | 8 | 0 | 0.335361 N |

These **112 one-factor recordings** have no preparation or native-contract
rejections, contact losses, travel stops or incomplete runs. The force-balance
audits pass. Actual physical masses, scanning motion and starting equilibrium
current remain unchanged during identification-error tests. Identified B/S/Kf
are used consistently in reconstruction, current feedforward, holding current
and the proper identified lower-link inverse. Its spring/damper model is not
silently retuned from physical values.

Separate group worsts include 0.488019 N for acceleration bias ±0.05 m/s²,
0.428090 N for axial force bias ±0.1 N, 0.428302 N for force gain ±0.2%,
0.328811 N for identified S mass ±1%, 0.333032 N for position gain ±1%, and
0.425443 N for actual robot response 40/100 ms versus identified 60 ms.
These passes **do not establish simultaneous tolerances**.

The five failures all vary actual robot transport: heavy seed7 fails at
100/120 ms and seed1729 at 60/100/120 ms, while the issued-command predictor
stays at 80 ms. At the worst retained peak, 2.1263 s on the flat after the
bump, actual normal force is 50.777479 N while the controller estimates
50.019716 N and its mean/rate gate remains ready. Actual robot speed is
+2.138 mm/s while the issued-command predictor gives −2.199 mm/s. The causal
slope estimate is −0.0453 on a physically flat surface. The drive does not
clip in this failure. Prediction mismatch and already queued motion therefore
remain material even with a force-stability gate. The complete native peak
and read-only decomposition are retained under `inertial_uncertainty_lower_v1_core/`.

At deliberately increased acceleration output noise of 0.03/0.05 m/s²,
four passing records briefly clip voltage. The 0.05 m/s² points reach about
0.64% voltage clipping, and one clips issued current effort for 0.0025% of
samples; actual peak current remains below 3.5 A. Noise robustness must not
be interpreted as a bias, gain or timing allowance.

The actual configured common preload is 33.6 N per spring. It passes every
history; the common feasible interval across the 112 records is
30.449–36.786 N. Combined maximum free spring length is 37.18 mm and maximum
installed length is 30.374 mm. The score checks this fixed preload, rather
than substituting a newly feasible preload after seeing a trace. These are
spring-only packing results.

The exact groups, provenance, failed native histories, drive clipping and
packing audits are summarized in `inertial_uncertainty_lower_v1_summary.json`.
Its supporting HTML, SVG and PNG figures are available with the same prefix.
The holder and scan follow-ups ran sequentially after a host-memory pause;
the two paused extra processes executed zero cases and are separately recorded
in `inertial_uncertainty_lower_v1_memory_pause.json`.

Including the six new paired proofs, this source adds **118 native executions:
113 passes and five failures**. This lane's cumulative retained count is now
**170 executions: 140 passes and 30 failures**, with the distinct source
versions and duplicate benchmarks described above. This total is not a count
of independent operating points or a combined qualification claim.

## Final V8 actual-sensing qualification

`experiments/qualify_inertial_uncertainty.m` now prepares the V8 native actual
anti-alias response, sensor-frame/cross-talk, timing and fine actual robot
delay phases, followed by joint tolerance corners and 30-second histories.
Calibration injections act in native sensor blocks, before reconstruction.
Physical masses/scan motion remain fixed when their identified values vary;
the robot predictor and recenter gain stay fixed when actual robot delay or
response varies. Identified S and B are used consistently in current
feedforward, and identified S in contact-force reconstruction.

Force bias means drift after tare; scale is residual calibrated tared-force
gain error. Sensor-frame rotation is relative to the flange frame used by the
normal estimate. Fixed gravity is tared in each sensor's frame; dynamic frame
error remains. These definitions avoid confusing sample RMS noise with
accuracy or calibration tolerance.

The runner separates native failures from preparation/contract rejections,
retains returned but rejected histories separately, and scores no decimated
plot arrays. Plot excerpts include full-native minimum/maximum envelopes so
that modal peaks survive display decimation. Gate statistics combine the
slow mean/rate gate with the fast variation gate only when enabled, while
retaining robot motion that remains queued after closure.

The V8 immutable source adds a native three-pole anti-alias/EMA sensor chain
at 100 kHz with nonzero 20 µs nominal pipeline. Actual channel responses and
pipeline offsets are separate from the identified issued-motion response.
Its actual robot transport uses 100 µs ticks while the sender, predictor and
gate stay at 100 Hz, enabling honest 75/85 ms actual-delay tests. The runner
sets active transfer-function initial states consistently with residual force
calibration and rotated gravity tare. Final unity and Normal/Accelerator
software proofs precede this source's uncertainty records. A first launch
stopped before any simulation because the private copy inherited the freeze's
read-only permission; only the new private copy is now made writable, and
the setup failure log is preserved. No original frozen file was changed.

The actual-sensing software proofs are now complete: six native recordings
for two endpoint configurations have **bitwise unity and Normal/Accelerator
parity** across all 72 recorded channels. The source is SHA256
`73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643`.
The initial proof and OAT candidate SHA is
`36e61547154e87d77b289d053cab40553ed849d61fa3cf278f20f902a3ad0708`.

| Final V8 cohort | Native cases | Passed | Worst normal error |
| --- | ---: | ---: | ---: |
| Actual sensing/frame/cross-talk OAT | 42 | 42 | 0.325377 N |
| Fine actual robot delay/response OAT | 14 | 14 | 0.451997 N |
| Combined deterministic tolerance corners | 60 | 60 | **0.448085 N** |
| Original worst-corner repeated30s | 4 | 4 | **0.490396 N** |
| Same four30s records, static residuals halved | 4 | 4 | **0.415311 N** |

All 124 uncertainty recordings complete without native rejections, contact
loss, travel stops, incomplete runs, current overrun or voltage/effort
clipping. Full-history force balances pass. Actual robot delay ±5 ms versus
identified 80 ms has a separate OAT worst of 0.355389 N; extending the actual
delay residual to ±10 ms gives 0.451997 N. Actual response ±5 ms versus
identified 60 ms gives 0.301284 N. These OAT values are separate sensitivities.

The combined grid uses **two endpoints**: S=2 kg/F=50 N/90° and
S=0.1 kg/F=5 N/0°, each with seeds7/29/1729 and ten deterministic antipodal
corners in 26 dimensions. Its declared budget is:

| Residual after calibration/tare | Combined test value |
| --- | --- |
| Force bias, each modeled axis | ±0.02 N |
| Force gain, each modeled axis | ±0.1% |
| Force/accelerometer frame relative to flange | ±0.1° each |
| Force cross-talk X→Z and Z→X | ±0.1% each |
| Acceleration bias, each modeled axis | ±0.01 m/s² |
| Acceleration scale | ±0.5% |
| Identified S/B mass and force constant | ±0.5% each |
| Position noise/gain | 1 µm RMS / ±0.1% |
| Common actual force-sensor pipeline | 20 µs ±10 µs |
| Lateral/acceleration/position pipeline versus force | ±10 µs each |
| Actual common AA cutoff versus identified 5100 Hz | ±5% |
| Lateral/acceleration/position AA cutoff versus force | ±5% each |
| Actual robot delay/response versus identified 80/60 ms | ±5 ms each |
| Identified scanning speed versus physical 100 mm/s | ±0.1% |

The pipeline combination permits actual channel delays from 0–40 µs. Raw
sampling remains synchronized at 100 kHz and the controller at 10 kHz.
The AA convention allows other-channel pairwise differences to reach 10%
of the common cutoff. The identified issued-motion AA/EMA/pipeline remains
5100 Hz/50 µs/20 µs regardless of actual sensing mismatch. Raw noise PSD
normalization stays tied to the nominal response. Output RMS targets are
0.02 N force, 0.014 m/s² acceleration and 1 µm position; they do not define
the calibration budget by themselves.

The final selected candidate SHA is
`d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7`.
It differs from the initial V8 proof candidate only in approximate spring
metadata: 18.1→20 mm free length and part text. The physical coefficients
and 100 µs/24 V current provision are unchanged. The configured 33.6 N
preload passes all joint records, with a shared feasible interval
30.422–40.590 N and combined free/installed lengths 40.98/34.174 mm.

The joint worst is heavy seed7/corner8 at 2.00058 s, with **0.051915 N
remaining margin**. Native S acceleration is 0.215044 m/s² while its sensed
value is 0.125720 m/s². Actual and predicted robot speeds are nearly equal
at this peak. The retained exact decomposition is +0.633208 N from actual
current tracking, −7.268498 N from command/feedforward relative to the
rigid-equivalent current, and +7.083375 N from B inertia relative to S.
The proper native inverse intentionally offsets that large B contribution;
it is not an independent unavoidable error. The identity residual is
2.09e−14 N. The force peak and 60 ms native window are retained under
`inertial_uncertainty_v8_joint/`.

The **finite joint grid passes**, but it is not an exhaustive 26-dimensional
hypercube or a full-range joint uncertainty qualification. Mechanical k/c
variation, changing orientation, random latency jitter, pad dynamics and
sanding vibration are outside this joint grid. Native nominal full-range
and mechanical stress studies have their own evidence and scope.

Summary JSON/HTML and inspected sensor/robot/joint SVG/PNG figures are
`inertial_uncertainty_v8_summary.*`; the exact corner vectors and calibration
conventions are in `inertial_uncertainty_v8_joint_budget_plan.json`.
The first joint launch stopped in case generation because of a MATLAB
nested-scope variable collision, before any native run. The failure log is
preserved; renaming the private case variable fixed the recipe. Both V8
setup failures are separate from native metric rejections.

The four original30-second records reuse heavy corner8 seeds7/29/1729 and
light corner3 seed29, the worst previously executed corner for each selected
endpoint/seed. Every history has3,000,001 points and24 bumps. Their peaks are
0.488090/0.487071/0.490396/0.086891N, leaving only0.009604N worst margin.
All four full histories, the worst native trace and sealed original figure
remain unchanged under `inertial_uncertainty_v8_joint_long/`.

The paired refinement halves **only16 static residual dimensions**: the two
force biases/gains, force and acceleration frame angles, two force cross terms,
two acceleration biases and common acceleration gain, identified S/B/Kf gains,
position gain and identified scan gain. Proposed assembled targets become
force zero±0.01N, force gain/cross-talk±0.05%, each frame±0.05°, acceleration
bias±0.005m/s²/gain±0.25%, identified S/B/Kf±0.25%, and position/scan gain±0.05%.
All noise amplitudes/seeds, ADC range/quantum, actual/identified AA and pipeline,
robot mismatches, mechanics, identified k/c, controller and100µs current
response remain identical. Whole-parameter guards and four prepared configurations
were frozen before the first simulation; actual physical startup current remains
at its unchanged equilibrium. These are installed calibration acceptance targets,
not noise reductions or demonstrated manufacturer/prototype accuracy.

The four refined30-second peaks are0.411872/0.411243/0.415311/0.075628N.
Worst margin becomes **0.084689N**, recovering0.075085N from static calibration
alone in these finite paired records. All histories, metrics, guards and private
source/helper/recipe snapshots are retained under
`inertial_uncertainty_v8_calibrated_static_long/`. The inspected paired chart and
all-native envelope/scalar CSVs are `inertial_uncertainty_v8_calibration_refinement.*`.
The portable input folder `inertial_uncertainty_v8_portable_inputs/` binds both
exact executed recipes, source model, selected candidate, helpers, original and
refined instrumented diagrams and prepared configurations. Relative rerun
wrappers preserve the two recipe versions after ZIP extraction.

The refined scope is **four stress-corner records**, not a new60-corner grid,
full-range combined uncertainty or continuous tolerance box. The original60
corners and long records retain their original wider static budget. The nominal
full-range qualification is a separate controller-worker study.

Including the six paired V8 proofs, V8 adds **130 executions, all passing**.
This lane's cumulative retained count is **300 native executions: 270 passes
and30 historical failures**, with source versions and duplicate benchmark scope
kept separate. Both long processes exited0 and restored their private SDI
preferences; all eight full native histories are durable. No native job remains
in this lane.
