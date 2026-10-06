# Current model: coil/spring-driven screw × soft pad/rigid 77 mm plate

This study began 6 October and completed 7 October 2026. It supplies the entire
quantitative comparison in the current [prototype report](../../../reports/active-contact-flange-report-design-explained-20261006.md).
The user's preferred development recommendation is a **voice coil with an
integrated sensing/control PCB**. The data retain the screw's lower winding
loss and greater tolerance to the tested mass-identification errors.

## Contract and physical scope

Read [study_contract.json](study_contract.json) first. The original
[internal-spring/pad study](../sea_pad_dynamic_20261006/README.md), its
[base contract](../sea_pad_dynamic_20261006/study_contract.json), and
[5 mm contract](../sea_pad_dynamic_20261006/pad5mm_contract.json) establish the
frozen physical lineage. Original engines, models, reports, HTML, delivery
archives and desktop state remain preserved.

All primary comparisons use A = 1.5 kg, B = 0.6 kg, S = 0.1–2 kg, a prescribed
20,000 N/m upper spring with zero damping and ideal unlimited upper travel,
and rigid external B–S attachment with a finite 64 MN/m sensing cell. Guide
friction is assumed 0.2 N Coulomb plus 2 N·s/m viscous. S acceleration and a
one-axis B–S cell reconstruct total **axial** contact force. Drag coupling is
neglected. No workpiece-normal force reconstruction on arbitrary slopes,
engagement, recentering, calibration-box, spindle or thermal qualification is
claimed.

The fixed robot scans a full-width 1 mm × 10 mm polynomial ridge at 0.1 m/s.
Two-second simulations begin at loaded equilibrium. Sources use 2.5 µs base
integration, 30 ksample/s acquisition, 10 kHz controller latching, simplified
matched 1.9 kHz two-pole responses, 49.6 µs converter-delay approximation,
50 µs ADC-output smoothing and 20 µs pipeline. Source noise is normalized at a
fixed 3,177.13 Hz reference bandwidth. These are assumed response/noise models,
not exact manufacturer transfer functions or measured installed electronics.

The coil is the hot-winding Kf = 24.4 N/A, R = 8.896 Ω, L = 4 mH candidate,
with bipolar 3.5 A/32 V usable drive, 50 µs current response and 25 mm stroke.
The screw includes a separate 75 g nut/carriage, RE35 motor inertia and
10 mm lead, plus a 5 N/mm nut–B spring with zero damping and a 60 Hz encoder
servo. Its 1.42 A RMS-current screen is not installed thermal validation.
Stiction, backlash, self-locking and a brake are not modeled.

## Equal footprint, different contact laws

**Soft:** circular 77 mm × 5 mm pad, assumed whole-pad stiffness 50 N/mm and
zero damping, 61 area-weighted unilateral strips. Total force is summed, not
computed from a pre-averaged surface profile. S moves dynamically. A 2.5 mm
working-compression guard limits the linear model. The material values are
unmeasured assumptions, not Mirka specifications; local pressure is excluded.

**Without a soft pad:** ideal rigid 77 mm translating plate with no tilt. The
versioned [engine.c](engine.c) uses the continuous analytic maximum of the ridge
under the full footprint, rather than the earlier point-contact k=0 branch or
a discrete maximum of strip heights. Entry/exit of the envelope plateau causes
2.4 m/s² acceleration jumps. At each, both force limits are scored with the
same continuous cell force. Peak-error lower bound is 1.2 × S mass N; this is
1.68/2.4 N at S = 1.4/2 kg. Real residual compliance or tilt might regularize
this ideal limit. A causal controller cannot remove its instantaneous jump.

[Plant checks](../../validation/current_model_matrix_20261006/plant/readiness.json)
cover analytic derivatives, equilibrium, event alignment, one-sided force
balance, contact stops and exact old-source soft-case repeatability. The
125 derivative samples, 36 equilibrium checks and verification recordings are
separate from campaign outcomes. Five failed verification-fixture attempts
remain retained; subsequent constant-current fixtures fixed the test setup,
without weakening a force guard or tuning a campaign controller.

## Locked controllers and fresh nominal outcomes

The [selected banks](../../validation/current_model_matrix_20261006/control/selected_banks.json)
contain four full scalar configurations and hashed locks. Force-integral gains
are zero. Soft branches keep their prior locked gains. Each rigid actuator has
18 families × four development corners, then a separately locked selection.
No rigid family passes every development corner. No evaluation or sensing
outcome was used for final retuning.

| Bank | Force gain (N/N) | Velocity gain (N·s/m) | Explicit inertia correction |
|---|---:|---:|---|
| Soft coil | 10 | 200 | Off; 80 Hz acceleration for velocity estimate |
| Soft screw | 0.5 | 100 | Off; 100 Hz acceleration for velocity estimate |
| Rigid coil | 0.25 | 0 | On, 500 Hz filtering |
| Rigid screw | 2 | 0 | On, 250 Hz filtering |

Both rigid branches add 250 Hz force-feedback smoothing. Velocity estimation
integrates filtered acquired acceleration with a 1 Hz leak. The screw maps
force to internal-spring deflection, applies a 300 Hz target filter and sampled
nut encoder servo. Acquisition removal disables multiple acceleration uses;
it is not a reconstruction-only experiment.

Each final selected grid has 54 identities: S = 0.1/1.4/2 kg × F = 5/20/50 N ×
orientation = 0/45/90° × two seeds 260701/260702. Development uses 260700.

| Selected configuration | Force + eligibility passes | Worst error | Contact stops |
|---|---:|---:|---:|
| Coil + soft pad | 54/54 | 0.053004803 N | 0 |
| Screw + soft pad | 54/54 | 0.115305276 N | 0 |
| Coil + rigid plate | 0/54 | 7.465322396 N | 12 |
| Screw + rigid plate | 0/54 | 7.774783703 N | 12 |

Soft-coil/screw maximum average hot winding losses are 37.3565/7.4354 W.
Soft coil gains transferred directly to the rigid plate lose contact in all
54 cases. The screw transfer has 12 stops and all 54 RMS-current failures.
These transfer records remain separate from the separately selected rigid
comparison. Failed and rating-ineligible attempts are not discarded.

[Control completion](../../validation/current_model_matrix_20261006/control/matrix_completion.json)
and its [audit](../../validation/current_model_matrix_20261006/control/completion_audit.json)
bind exact identities, configuration hashes, rankings, original sources and
all event limits. See the [control recipe](control/README.md).

## Current sensing, drive and calibration diagnostics

[Acquisition](acquisition/README.md) records 4 banks × 3 corners × 19 variants =
228, with a fresh diagnostic seed and fixed gains. All soft-pad 12/14/16/24-bit
code-spacing corners pass, as do 15/30 ksample/s. A 7.5 ksample/s conversion
and matched 500/950 Hz response lose contact in the light coil case. Bit tests
retain source noise/timing; converter-rate changes increase assumed delay.
Channel skew and the exact ADS131M04 SINC3 response are not represented.

Acceleration-noise settings up to 0.056 m/s² reference-normalized RMS pass all
six soft corners, at 0.161144 N worst. +0.02 m/s² acceleration bias raises the
screw worst to 0.305916 N. Removing acceleration fails all three soft-coil
corners; soft screw passes force in all three, but two exceed its 1.42 A RMS
screen. Every rigid-plate sensing diagnostic fails force.

+1% identified S mass fails both heavy coil cases, including sensor range.
The declared adaptive [calibration follow-up](calibration_followup_contract.json)
adds 120 recordings at a new seed, unchanged gains. +0.25% mass passes 3/3
soft-coil corners; +0.5% fails both heavy corners. All soft-screw mass follow-ups
pass. Separate ±0.25% acceleration gain, ±0.05% force gain and one signed joint
mismatch pass the soft corners; no full simultaneous residual box is qualified.
Large force extrema after sensor ineligibility are not prototype predictions.

[Drive diagnostics](drive_checks_contract.json) add36 recordings, including 12
separately executed baseline repeats. 32→48 V at unchanged 50 µs current
response is bitwise identical for the soft cases, which do not clip. At 48 V
and 25 µs response, coil three-corner worst improves 0.045049→0.042244 N; screw
changes 0.113403→0.113394 N. Rigid cases remain failures. More voltage alone is
not the dominant limitation in the selected soft configurations.

The report's mass/calibration targets and cheapest-ADC discussion are
**development proposals**, not demonstrated joint hardware accuracy. Dated
component-price reviews are reused as price evidence only. ADXL354C's
mechanical resonance and real filter phase require installed measurement.

## Counts, native verification and scope

| Primary C recordings | Records | Integration endpoints | Extra left force limits |
|---|---:|---:|---:|
| Rigid controller development | 144 | 102,514,972 | 248 |
| Final selected/transfer grids | 324 | 229,956,425 | 320 |
| Acquisition diagnostics | 228 | 185,927,578 | 224 |
| Drive diagnostics | 36 | 30,334,627 | 36 |
| Adaptive calibration diagnostics | 120 | 101,115,417 | 120 |
| **Total** | **852** | **649,849,019** | **948** |

This is executed-record count, not 852unique fresh qualification conditions.
Pilots, fixture verification and command regeneration are separate observations.
Raw engine `steps` includes additional left force values in the rigid branch;
receipts explicitly distinguish them from integration endpoints. Root drive/
calibration aggregate labels were corrected from their retained record metrics;
no run, configuration or outcome changed.

The [native recipe](native/README.md) rebuilds a shallow physical Simulink
plant from ordinary blocks, with 9 Integrators and 6 subsystems. Four locked,
completed representative command captures reproduce original metrics before
independent physical replay. The controller/acquisition themselves are not
independently implemented in that replay. No model is promoted to the user's
desktop; the old public models remain frozen.

All four native base replays pass both unchanged 100 µN footprint guards.
Maximum native/regular-C force difference is 2.796 nN; comparison with original
split-event traces at 10 kHz reaches 1.591 µN. Force-balance residual is
1.137e−13 N; save/reopen and zero algebraic loops pass. Base and half-step
rigid replays score 6,400,014 force values in memory. Matched-grid refinement
changes are 0.594/10.113 µN; the refined screw maximum grows 76.671 µN from
intervening endpoints, preserving its failure. Those matched-waveform and
full-maximum measures are distinct. No refinement acceptance threshold was
predeclared. No new soft spatial/time refinement or native stop probes ran;
the four completed native representatives do not exercise partial soft contact.
The original study's partial-contact convergence limits and failed guards
remain in its frozen evidence.

[Native audit](../../validation/current_model_matrix_20261006/native/validation_audit.json)
binds source, lock/configuration and gzip trace hashes. The 500 Hz compressed
traces are for display only; all-step metrics and exact left/right event pairs
establish peaks. Bulk commands/reference arrays were regenerated privately
and removed after scoring.

## Storage and report delivery

The [forecast](../../validation/current_model_matrix_20261006/storage_forecast.json)
limits this entire study's incremental retained source/evidence/figures/report
copies to 3 MiB, and temporary work to 640 MiB. Native peak temporary files were
321,319,394 bytes and were released. Full histories are not saved. Shared
project/owned-SATA arrays remain under the user's 5 GiB limit. Nothing outside
owned temporary directories is cleaned up by this study.

The prior 11febde repository/Notes report copies are preserved under
[`previous_report_11febde`](../../validation/current_model_matrix_20261006/previous_report_11febde).
Eight report SVGs comprise six current result graphs and two unchanged proposed
architecture diagrams. Hardware targets and dated prices are separate from
executed simulation results. HTML remains frozen at the user's request.

The root [report input manifest](../../validation/current_model_matrix_20261006/report/report_inputs.json)
and [Markdown delivery manifest](../../validation/current_model_matrix_20261006/report/markdown_delivery_manifest.json)
bind source, metrics, figures, Markdown and Notes. Verify these without running
new experiments:

```bash
PYTHONDONTWRITEBYTECODE=1 python acf_simulink/experiments/current_model_matrix_20261006/finalize_report.py --verify
```

The frozen execution recipes target their declared output directories. To run
new experiments or tune gains, create a separately versioned contract/output
study and repeat budget, lock, fresh-evaluation and native checks rather than
overwriting completed evidence. Python 3/NumPy and a C compiler run the sweeps;
MATLAB/Simulink R2026b is needed for the independent native batch replays.
