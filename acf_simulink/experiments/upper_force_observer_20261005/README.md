# Measured-force upper-motion observer candidate

Storage update, 2026-10-06: the user's disk budget supersedes blanket raw-history
retention below. Banks, models, compact results and selected report traces
remain. Read the [cleanup record](../../validation/storage_cleanup_20261006/README.md)
before replaying historical outputs; removed bulk histories require regeneration.

The current corrected export is
`validation/upper_force_observer_20261005/native_cellsource_v3/`, with locked
inputs in `bank_cellsource_v2/`. The first `native/` export is **rejected
controller wiring evidence**: its nested `From(measured_force_N)` could not
see the root-local Goto and generated C used only the −Fref bias. Its failed
recordings do not assess the measured-force observer or the mechanics. The
original bank, export and failed recordings remain preserved. The first
local-source guard attempt and the cancelled lower-damped preparation also
remain preserved as inactive attempts.

The eligible private development candidate retains the **required** upper spring at
20,000 N/m with zero external damping, the finite 64 MN/m B–S load cell with
zero structural damping, the original six PI/notch profiles, the same eleven
physical development cases and seeds, and both 0.4 kg and 1.5 kg A masses.
The native physical plant, ADXL354C-like sensing, matched 1.9 kHz acquisition,
30 ksample/s ADC, 10 kHz force controller, 50 µs physical current response,
32 V usable hot-coil drive and 25 mm actuator travel remain unchanged.

The upper spring has **no length or travel constraint** in this experiment,
as requested. `upperLengthUnconstrained` and the upper pair's
`lengthAndTravelUnconstrained` are explicit in the bank. Old catalog fields
and the parent bank's `springPackingVerified` field are inherited descriptive
metadata; they do not qualify an upper spring packing arrangement. No lower
external spring is present. Contact-loss and actuator-travel failure guards
remain active.

The sole controller change replaces the issued-current/identified-lag net
actuator-force prediction in the four-state upper observer with a causal
force balance from existing sensors:

$$
\delta \widehat F_a = \delta L - \widehat m_B a_{S,z}
+ \frac{\widehat m_B}{\widehat k_1}
\frac{s^2}{(1+s/\omega_c)^2}\delta L,
\qquad \omega_c=2\pi\,1500\ \mathrm{rad/s}.
$$

The selected source is now an explicit subsystem connection from the local
`Contact force reconstruction/Cell error to deviation` gain output, before
the +Fref bias. This is the acquired gravity-tared **axial B–S cell** deviation
after the existing ADC, pipeline and fresh 10 kHz latch:
`delta L = acquired axial cell - Fref`. It is not the reconstructed normal force.
Measured S acceleration is already gravity-free and shares that acquisition
frame. The identified mass is `p.uncertainty.massB_model_kg`; stiffness is
`p.inertial.lowerLoadInverse.identifiedStiffness_N_m`. No second acquisition,
pipeline, guide-friction subtraction or holding-current subtraction is added.
The reconstructed net force includes the effect of physical guide friction
and actual electrical clipping as transmitted through the cell and inertia.

The second derivative is a proper, critically damped continuous native
State-Space filter with zero initial state. It has finite direct feedthrough
and a high-frequency correction gain of 0.832748 for the nominal 0.6 kg holder
and 64 MN/m cell. It is causal, but differentiates noise and approximates the
exact cell-deformation correction above the regularization band. It is a new
regularization, rather than a copy of the earlier discrete lower inverse.

The observer remains an exact-ZOH four-state estimate at 10 kHz, with states
`[A−R, vA−vR, A−S, vA−vS]`. Position correction is separated from the prior
state output so it introduces no artificial direct force/friction loop.
Corrected A velocity enters the existing surface-normal geometry; corrected
A acceleration enters the existing causal feedforward. The only source tags
are measured S acceleration and predictions from robot commands already
issued; axial cell deviation uses the direct subsystem wire. Plant truth is
confined to scoring outputs. A numerical pre-export guard perturbs the actual
local cell-error gain inlet by −1, 0 and +1 N and verifies the selected head
observer input is +1, 0 and −1 N. Actual series are retained. Original acquired
input wiring is restored before export. Emitted C additionally proves the
selected deviation retains the measured `Sensorsampledelay` term.

Run preparation/export in an isolated MATLAB batch process:

```matlab
addpath('acf_simulink/experiments/upper_force_observer_20261005');
build_upper_force_native_cellsource_v2( ...
    '/absolute/new/private/export/directory', ...
    '/home/tommie/Projects/actuator-model/acf_simulink/validation/upper_force_observer_20261005/bank_cellsource_v2/configs.mat');
```

The recipe deliberately rejects an existing bank/export directory to preserve
prior evidence. Compile and seal the generated graph using
`python acf_simulink/experiments/upper_force_observer_20261005/seal_native_cellsource_v2.py`
with the new export directory as its argument. Run
`verify_force_observer_graph_cellsource_v2` in a separate MATLAB batch process for saved-copy
compilation, zero-loop and source-tag acceptance for both A masses.

Root owns evaluation, retained histories, figures and report integration.
Prepared configurations, code generation and saved-graph acceptance are not
executed force qualification or physical prototype evidence. No RL training
or fresh-test tuning occurs in this candidate preparation.
