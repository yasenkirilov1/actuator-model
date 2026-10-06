# Series-elastic actuator and soft-pad experiments

This versioned study answers two questions: can a leadscrew driving its own
spring handle the sharp bump, and **how do soft pads influence our results?**
It tests **total pressing force along the flange axis**, with one-axis B–S
sensing, S acceleration and no sanding drag. The original surface-normal
qualification remains separate. No local-pressure metric is evaluated.

The initial contract is [study_contract.json](study_contract.json). Compact
executed results go to
[`validation/sea_pad_dynamic_20261006`](../../validation/sea_pad_dynamic_20261006/).
The study is complete. Development, locked evaluation, instrumentation and
presentation reruns remain separately counted. The later user-selected
**77 mm × 5 mm** geometry is bound by
[pad5mm_contract.json](pad5mm_contract.json), preserving the earlier 10 mm cases.

## Executed results

The selected 5 mm configuration uses **50 N/mm whole-pad stiffness**, zero
pad damping, and the previously frozen conservative controllers. That stiffness
is an engineering assumption, **not a Mirka material specification**. The
manufacturer geometry is recorded in the appendix contract and source notes.

| 77 mm × 5 mm nominal grid | Voice coil | Internal-spring screw |
|---|---:|---:|
| Force and physical passes | 54/54 | 54/54 |
| Worst total axial force error | 0.046075 N | 0.118307 N |
| Largest 2 s average modeled hot winding loss | 37.357 W | 7.437 W |
| Largest RMS current | 2.049 A | 1.069 A |
| Maximum pad compression | 1.988 mm | 1.989 mm |
| Maximum voltage request | 21.10 V | 26.38 V |

Each grid contains S = 0.1/1.4/2 kg, force = 5/20/50 N, orientation =
0/45/90°, and two fresh noise seeds. Every scheduled endpoint counts, including
noisy startup from loaded equilibrium. Both groups pass the recorded sensor,
peak-current and motor-speed screens. The screw passes its catalogue 1.42 A
continuous RMS-current screen; coil continuous thermal capacity remains unknown.
Neither modeled copper loss nor a two-second run proves thermal steady state.

The fresh results are
[coil](../../validation/sea_pad_dynamic_20261006/evaluation/coil_pad5mm_k50_c0.json)
and
[screw](../../validation/sea_pad_dynamic_20261006/evaluation/sea_pad5mm_k50_c0.json).
The [engine summary](../../validation/sea_pad_dynamic_20261006/engine/final_summary.json)
accounts for **1,975 primary executed recordings**, comprising 1,049 development
and 926 evaluation/diagnostic records, plus 216 separate instrumentation reruns
and 26 selected trace/command captures. Reused wide records are not counted twice.
There are zero preparation rejections and 634 recorded stops across tuning and
diagnostics. **1,403,759,763 C integration endpoints** were scored; that includes
acquisition-event subdivisions and is not a native Simulink sample count.

With unchanged gains and matching conditions/seeds, each actuator has **0/8
point versus 8/8 wide** passes. Narrowing contact changes the system; these
are transfer sensitivities, not independently retuned hardware rankings.
Three coil point cases exceed the proposed sensor ranges. Their trajectories
beyond that eligibility failure must not be interpreted as prototype predictions.

The existing 5 mm material sensitivity has **32/32 passes at 50 N/mm** across
four damping values and four corner conditions for each actuator. At 100 N/mm,
**23/32 pass**, and five fail through contact loss. The 20 N/mm, 5 mm corners
use the entire assumed 2.5 mm working range at a static 50 N; all four such
50 N records stop essentially during startup. This is an assumed linear-law
validity boundary, not measured physical bottoming or force-control failure.

Earlier 20 N/mm/10 mm wide-pad grids pass at 0.077590 N coil and 0.115372 N
screw with zero pad damping. A more aggressive c80 screw candidate passes force
at 0.047051 N but exceeds continuous catalogue current in two cases. These
recordings stay historical; they are not relabelled as the 5 mm pad.

## What changes mechanically

The prescribed robot–A spring stays at **20 N/mm with zero damping and
unlimited ideal travel**. A is 1.5 kg, B is 0.6 kg, and S is varied from
0.1 to 2 kg. The lower B–S mounting stays rigid, with the sensing cell's
**64 MN/m finite stiffness**. Guide friction is the declared smooth 0.2 N
Coulomb plus 2 N·s/m viscous surrogate.

The screw has an independent nut coordinate and a captured, signed spring
between the nut and B. This spring is internal to the actuator. The nut can
move differently from the tool; its rotary inertia is not added to S. The
motor is an adequately sized RE35 terminal-DC reference with explicit current,
voltage, winding resistance, inductance and transmission friction. Spring
rates, preload/working limits, translating nut mass and screw inertia are
declared engineering assumptions. See the
[manufacturer sources and sizing](../../validation/sea_pad_dynamic_20261006/assumptions/README.md).

Positive displacement points away from the workpiece. The cell carries

\[
L=k_\mathrm{cell}(z_S-z_B),\qquad
m_Sa_S=F_\mathrm{contact}-L-m_Sg_\mathrm{axis}.
\]

Therefore the causal measurement reconstruction is

\[
\widehat F_\mathrm{contact}=L_\mathrm{tared,measured}
  +\widehat m_Sa_{S,\mathrm{measured}}.
\]

The physical cell force comes from its deformation. It is not synthesized
from pad truth. Static gravity tare does not remove dynamic inertia.

## What a soft pad means here

The pad is a vertical Kelvin–Voigt foundation: a stiffness in parallel with
an optional material damping term. It lets S move independently of the
surface. The point-contact surrogate isolates material compliance. A
**77 mm circular footprint** adds the physical effect of distributed contact.
Both use the same **full-width, 1 mm high, 10 mm long ridge at 100 mm/s**.
The pad's total flat-contact stiffness and damping are held constant when
its footprint changes.

For strip j, normalized area weight w is taken from the exact circular chord
area, while the surface is evaluated at the strip midpoint. Its compression
is surface height minus tool position. A strip transmits force only while
compressed, and a tensile force is clipped to zero. **This clipping happens
before summing total force.** Partial contact is allowed; complete contact
loss fails the recording. Spatial refinement uses more strips separately
from time-step refinement.

The material cases are representative whole-pad stiffnesses, not measured
Mirka properties. Published foam compression data bound their plausibility.
Material damping is an unmeasured sensitivity, including **zero**. The
declared half-thickness working-compression limit prevents extrapolating the
linear material law into an implausible range. It is a model-validity guard,
not a measured bottoming limit or a pressure criterion.

A compliant pad invalidates the old assumption that tool velocity directly
reveals surface slope. The new one-axis study does not reuse that correction.
It also does not reuse the old 1.64 kHz cell-mode feedback: dynamic S changes
the cell's modes. These records must not be relabelled as a new full normal-
force qualification.

## Control, acquisition and test separation

The causal force controller uses acquired cell, S acceleration and S–A
position. Force PI, rolled-off inertia compensation and estimated tool
velocity can be tuned. The screw additionally uses its motor/nut encoder and
a spring-deflection/nut-position servo. Encoder speed must come from sampled
position history, not the true plant velocity. No controller input contains
surface geometry or future motion.

Acquisition has matched 1.9 kHz two-pole response, 30 ksample/s output,
50 µs EMA and 10 kHz control. The converter's nominal group delay is
represented as transport delay; this is an explicitly simplified response,
not an exact ADS131M04 SINC3 implementation. Noise is normalized to the fixed
reference bandwidth, and measured filtered RMS is retained in the metrics.
The new pad study assumes 0.014 m/s² acceleration noise at 3,177.13 Hz
reference bandwidth, with about 0.0104 m/s² measured filtered RMS. It does not
reproduce the historical budget chain's 0.0186557 m/s² sensor source plus separate
ADC noise. Quantization and the simplified converter delay are included; this
is not an exact manufacturer sensor/converter transfer or installed noise guarantee.

Each recording starts at **loaded equilibrium** with acquisition states
initialized consistently. All subsequent samples, including noisy startup,
are scored. This does not test first engagement from zero force. The robot
is held fixed for matched two-second fast-actuator/pad diagnostics; delayed
recentering and long sanding operation are outside these short comparisons.

Tune separately on designated development cases. Freeze each candidate's
parameters and source identity before the matched, fresh-seed evaluation.
Retain failed candidates and preparation rejections. A force error above
0.5 N fails a record even if it completes. Physical or model-validity stops
fail it independently.

## Native verification and storage

The fast experiment engine is checked against a separate native Simulink
physical plant built from ordinary blocks. Its command replay verifies the
physical equations and recorded force histories; it is **not an independent
closed-loop controller qualification**. Equilibrium, force balance,
algebraic-loop absence, save/reopen, timing and spatial refinement receive
separate checks. Preserve any failed strict parity guard with its actual
numerical result.

The [native summary](../../validation/sea_pad_dynamic_20261006/native/validation_summary.json)
records **nine independent base replay histories and 7,200,009 native samples**,
plus four deliberate failure-stop probes (6,230 outputs) and three separate
refinement comparisons. Both selected 5 mm replays pass the unchanged numerical
guards and reproduce their recorded force maxima. The native graph has six
shallow subsystems, nine ordinary Integrators, no hidden MATLAB plant solver,
no algebraic loops, and verified save/reopen. Read the
[beginner guide](../../generated/sea_pad_dynamic_20261006/GETTING_STARTED.md)
for model opening and consistent replay selection. It is a physical-plant replay,
not a newly promoted closed-loop replacement for the frozen public model.

Retained limitations include a **32.181125 µN** failed strict point-replay guard
against a 10 µN limit, with matching force-failure outcome, and a separate
partial-contact case with **0.331836 N** waveform change under timestep halving.
Its peak error changes only 0.444323→0.444353 N; that does not establish a
converged contact-edge waveform. Original noise-free closed-loop refinement
changes force peaks by at most **0.001426 N** for 61→121 strips and **0.00004154 N**
for 2.5→1.25 µs. Those refinements do not newly qualify all 5 mm cases.

## Reproduction

The [engine recipe](engine_README.md) names all runners and source versions.
Run the compact audit without simulations with:

```bash
python acf_simulink/experiments/sea_pad_dynamic_20261006/runner_aggregate.py
```

The 5 mm runner can regenerate its bounded grids and selected traces. It writes
the study's evaluation paths; use an isolated copy to preserve recorded evidence.
The native guide describes a private `matlab -batch` process, temporary storage
and preference cleanup. Do not use the user's open desktop session for sweeps.

The main report is
[Designing the active sanding flange](../../../reports/active-contact-flange-report-design-explained-20261006.md).
Its new SVGs are rebuilt with `build_pad5mm_figures.py`; the earlier 10 mm
comparison illustrations remain under `build_report_figures.py`.

The delivered Markdown and seven referenced SVGs are bound by
[markdown_delivery_manifest.json](../../validation/sea_pad_dynamic_20261006/report/markdown_delivery_manifest.json).
[report_inputs.json](../../validation/sea_pad_dynamic_20261006/report/report_inputs.json)
binds the current study, selected grids and separate historical reference inputs.
[completion_checks.json](../../validation/sea_pad_dynamic_20261006/report/completion_checks.json)
records protected-file hashes, native trace roundtrips, links and storage.
Verify that delivery without rerunning or rewriting scientific evidence with:

```bash
python acf_simulink/experiments/sea_pad_dynamic_20261006/finalize_report.py
```

The [storage forecast](../../validation/sea_pad_dynamic_20261006/storage_forecast.json)
allocates at most 32 MiB of retained study output, with compact JSON/CSV and
selected decimated traces. All integration samples are scored in memory;
bulk histories are not retained. Existing frozen models, delivery archives
and the user's open desktop model are preserved.
