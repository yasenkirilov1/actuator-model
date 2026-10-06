# Streaming series-elastic screw and pad experiments

This is a separate development study for **total axial pressing force**, with
one B–S load measurement, S acceleration and S–A position. Axial sanding-drag
coupling is neglected. These results do not qualify surface-normal force on an
unknown slope, the sanding process, local pressure, or a physical prototype.
The canonical MATLAB/Simulink model, earlier controllers and earlier evidence
were preserved. The root-owned [study contract](study_contract.json) defines the
original experiment; [the 5 mm appendix contract](pad5mm_contract.json) adds the
user's later 77 mm ×5 mm geometry without changing the 10 mm recordings.

## Physical model and signs

Displacements are positive away from the workpiece. The physical state order is
`[zA, vA, zB, vB, zS, vS, q, vq, i]`. Robot position R is fixed for these 2 s
recordings. A is 1.5 kg, B is 0.6 kg, and S is 0.1–2 kg. The prescribed R–A
spring is 20,000 N/m with zero damping and unlimited ideal working travel.
The external B–S attachment is rigid; the sensing cell retains 64 MN/m stiffness
and zero assumed cell damping. Flange stroke, screw/nut stroke, SEA spring
working range, contact loss and the assumed pad working range remain guarded.

The signed cell force is `L = kcell (zS − zB)`. The physical force balance on
S is `Fc = L + mS (aS + gaxis)`, where `gaxis = g cos(angle)`. The acquisition
chain adds the known static tool weight to the cell reading, then reconstructs
force using the measured S acceleration. Static tare does not remove dynamic
inertia. The all-step cell range screen uses the **raw signed cell force**, and
the accelerometer range screen uses **specific acceleration** `aS + gaxis`.
Neither the electrical ADC span nor a decimated figure establishes part range.

The screw has an independent nut/rotor coordinate `q = zA − zNut` and a signed,
bidirectional SEA spring between nut and B. `u = zA − zB` and
`Fa = kSEA (q − u)`; there is no SEA viscous damper. The coupled A/nut mass
matrix includes the nut's translational mass and the rotor/screw inertia
referred through `ell = lead/(2 pi)`. In the chosen coordinates its entries are
`mA + mNut`, `−mNut`, and `mNut + J/ell²`. This prevents the nut from behaving
as a massless perfect position source. Nut gravity requires the equilibrium
motor current `(Fa − mNut gaxis) ell/Kt`, rather than `Fa ell/Kt`.

The brushed RE35 reference has hot winding resistance 6.5052 Ω, inductance
1.29 mH, `Kt = Ke = 0.0758`, total rotary inertia 7.04e−6 kg·m² and 0.075 kg
nut translation mass. Lead is 5 or 10 mm/revolution. The matched hot coil has
`Kf = Ke = 24.4`, 8.896 Ω and 4 mH. Both use 32 V usable bipolar output,
3.5 A peak command and a 50 µs identified current-response target. The physical
current derivative uses clipped voltage, back EMF, winding R and L. Current
clamp does not by itself establish actual-current compliance under backdrive,
so actual current and motor speed are scored separately. Shaft Coulomb/viscous
friction and flange-guide friction are retained. Zero backlash, calibrated
encoder zero and an ideal identified current-response envelope are assumptions.

## Pad geometry and the bump

The surface is a full-width ridge with 1 mm height, 10 mm scan-direction width
and 100 mm/s scanning. Its smooth compact profile is
`h = 64 H r³ (1 − r)³` for `0 < r < 1`; otherwise h is zero. A point reaches
the bump at 0.8 s. A 77 mm footprint first reaches it at 0.415 s.

The circular footprint uses 61 scan-direction strips at midpoint offsets.
Weights are each strip's **integrated circular chord area**, normalized to sum
to one. Each strip first applies unilateral Kelvin–Voigt contact:
compression must be positive, and its force is clipped at zero. Only then are
weighted forces summed. A broad pad therefore cannot create tensile contact
on its trailing portions. Total flat stiffness and damping are held fixed when
comparing point and wide geometry. This is spatial mechanical compliance, not
a controller that receives a smoothed or previewed surface signal. Extraction
holes and detailed abrasive deformation are not resolved; no local pressure is
computed as a design metric.

Whole-pad stiffnesses 20, 50 and 100 kN/m are representative model values.
Constant damping 0–150 N·s/m is a sensitivity envelope, not a measured foam
constitutive law. In particular, high constant Kelvin damping cannot be assumed
at the cell's kilohertz mode. The original 20 kN/m cases use 10 mm thickness;
50/100 kN/m cases use 5 mm thickness. A half-thickness compression guard is an
assumed linear-model working limit, **not a manufacturer's bottoming threshold**.
The user-specified 5 mm appendix explicitly retains 5 mm thickness when testing
20 kN/m and records any consequent working-range failures.

`pad_k = 1e6` is a stiff finite-compliance surrogate. `pad_k = 0` is a separate
rigid-point constraint: S follows the profile and force comes from the cell/S
force balance. This branch is point-only. Applying soft-pad gains unchanged to
either branch is a sensitivity diagnostic; it is not a tuned actuator ranking.
When a rigid constraint would require tensile reaction, the run stops as lost
contact. That required reaction is not a physical negative pressing force.

## Controller and acquisition contract

The outer controller requests actuator force from static holding effort,
force error, optional PI integral, measured-inertia feedforward and damping
derived from leaky integration of filtered measured S acceleration. Its law is
`Fa_request = Fref − (mB + mS_identified) gaxis + Kp error + I −
inertia_gain (mB + mS_identified) a_filtered + D vS_estimated`.
The selected conservative c0 coil gains have Kp 10, Ki 0, D 200 N·s/m and
inertia gain zero. The selected conservative SEA c0 gains have Kp 0.5, Ki 0,
D 100 N·s/m, inertia gain zero and a 60 Hz inner position servo. SEA gains
are independently developed; the coil current controller is not copied to the
motor. Position feedback gives a causal q target through the series spring;
the encoder has 14-bit/revolution quantization. Nut velocity is derived from
sampled quantized encoder history with a 1 kHz low-pass filter, rather than
read from the physical state. Previous encoder position is initialized to the
quantized equilibrium value to avoid a fictitious startup speed.

No controller receives h, h derivatives, true S/A velocity or future robot
motion. Surface truth appears only in the plant and diagnostics. Known tool
mass is updated per case, not learned from future observations. An optional
servo target-velocity feedforward can amplify noisy force-command derivatives;
its behavior is explicitly recorded in the frozen gain bank. Zero upper/SEA
damping does not remove physical motor/guide friction or pad damping.

The model uses matched two-pole 1.9 kHz force, acceleration and position fronts,
30 ksample/s ADC output, a 50 µs EMA at that output clock, 49.6 µs ADC transport
delay plus a 20 µs pipeline, and fresh 10 kHz controller latching. The ADC
transport is a **group-delay approximation**, not an exact SINC3 transfer.
Fractional ADC and pipeline timestamps divide the regular 2.5 µs integration
step so acquisitions are not rounded to the solver grid. One command remains
held through each RK4 step and updates at the next control event.

Noise density is normalized to the fixed 3,177.13 Hz reference bandwidth:
0.02 N force RMS, 0.014 m/s² acceleration RMS and 1 µm position RMS at that
reference. Force noise is bounded uniform; acceleration/position noise is
Gaussian. These values are not promises of present filtered RMS. Fine-clock
injection is normalized to step duration; measured output noise is retained in
metrics. Quantization, biases and scale fields are distinct. Nominal runs use
zero residual bias and unity scale; installed sanding vibration, temperature
drift, cross-axis sensitivity, mounting flexibility and alias rejection are
not experimentally established.

## Evidence, counts and ratings

The original compact audit is in
`validation/sea_pad_dynamic_20261006/engine/original_study_summary.json` relative
to `acf_simulink/`. It contains 1,843 primary executed recordings: 1,049
development and 794 evaluation/diagnostic. There are 216 separately counted
observation-only range reruns and 24 known selected trace/command reruns.
These are recording counts, not claims of unique parameter settings. The
original primary recordings score 1,291,494,950 C integration endpoints,
including acquisition-event subdivisions. They are not native Simulink sample
counts. Reused wide records in exact-width pairs do not count twice.

`engine/final_summary.json` and `engine/evidence_manifest.json` add the 5 mm
appendix when complete and hash each compact input. Failed force outcomes,
physical stops and preparation rejections remain distinct. One initial pilot
output serialization attempt failed after its simulation completed, was fixed
and rerun; it has no separate retained scientific record. Earlier development
records without complete source hashes are explicitly labeled historical.
No final-source hash is retroactively asserted for those runs.

Motor actual current, 12,000 rpm speed and its 1.42 A continuous RMS catalog
screen are independent of the ±0.5 N force flag. The c80 wide SEA group passes
54/54 force cases but exceeds continuous RMS current in two; it is not an
eligible continuous motor design merely because force error is small. The
conservative c0 wide motor group stays below that RMS screen. Coil continuous
thermal rating is unspecified; average hot copper loss is reported rather
than silently assumed acceptable. Sensor screens are raw cell ±100 N and
specific acceleration ±8 g. All-step checks, not plot extrema, determine them.

`force_balance_peak` is an algebraic consistency identity. Independent native
Simulink replay supplies the physical-plant comparison; its own guard failures
and tolerances must be read from the native evidence. It does not independently
validate the C sensor/controller implementation. Noise-free closed-loop
refinement compares 61→121 strips and 2.5→1.25 µs without changing the candidate.
The largest original force-peak changes are 0.001426 N spatially and 0.00004154 N
temporally. Noise is disabled because changing the random injection clock would
otherwise confound numerical convergence with a changed realization.

## Reproduction and storage

`engine.c` is a streaming double-precision RK4 implementation compiled by
`engine_api.py` with GCC `-O3 -std=c11 -Wall -Wextra -Werror`, without fast-math.
Shared libraries are source-hash versioned; loaded libraries are never
overwritten. Current frozen source SHA256 is
`be43185291e39e6348efd5fb687482db29615a13e516fc52cc0bc306ed3de4a5`.
`operational_source_adoption.json` records the rigid-point branch, causal
encoder correction, acquisition approximation and range instruments. The
16 loaded-equilibrium derivative checks have passed. Source snapshots remain
beside the compact engine evidence.

The runner sequence is pilot/screen, original point campaign, quiet SEA
development, then frozen point/width groups with fresh seeds. Further runners
perform paired factors, instrumentation, transmission, material sensitivity,
numerical and unchanged-voltage diagnostics. `runner_pad5mm.py` appends exactly
108 fresh wide cases, 16 matched point cases, eight 20 kN/m working-range corners
and two selected command/trace captures, using the prior conservative gains.
It declares the preserved forecast hash and the later operational allocation
before launching. `runner_aggregate.py` audits evidence without simulations.

Run a recipe from the repository with `python
acf_simulink/experiments/sea_pad_dynamic_20261006/runner_pad5mm.py` or run the
audit with `python acf_simulink/experiments/sea_pad_dynamic_20261006/runner_aggregate.py`.
Recipes must not be treated as proof until their executed metrics exist.

The shared experiment-array cap is 5 GiB. This study's total retained allocation
is 32 MiB; the final operational split is 18 MiB engine, 10 MiB native and
4 MiB root/report. Engine histories are scored in RAM; no MAT/NPZ bulk arrays
are created. Selected CSV traces use lossless gzip. The trace-compression
manifest resolves older recorded `.csv` paths to preserved `.csv.gz` files.
New native commands are gzip-compressed and decompressed only in the native
worker's private temporary directory. Full physical-replay arrays remain in
RAM and are released after scoring. Preserve original contracts, compact
failures, source snapshots, locks and report evidence when reproducing.
