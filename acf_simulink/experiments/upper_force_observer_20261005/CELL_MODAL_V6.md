# Bounded cell-mode feedback experiment V6

This is a private controller experiment on the user-mandated plant: the
upper connection is 20,000 N/m with zero external damping and ideal unlimited
spring length/travel, and the lower connection is rigid with a finite
64 MN/m, undamped B–S load cell. It does not change the public model or any
physical stiffness, damping, guide friction, sensor noise, sensor timing,
baseline PI/notch, force reconstruction or upper-motion observer.

## Question and mechanism

The corrected observer development recordings showed post-bump force ringing
near the cell mode. Root owns those histories and metrics. This experiment
asks whether bounded feedback from the existing acquired axial cell can
actively remove that mode's energy while retaining the existing sensing.

Define cell deformation from loaded equilibrium as `q = B − S`. The tared
cell deviation is `δL = −k1 q`. Since the B mass has an actuator term
`−Fa/mB`, a positive `Fa = c_virtual qdot` removes modal energy. The desired
force correction is therefore `−(c_virtual/k1) δLdot`.

Blind differentiation is unsafe under the modeled acquisition delay. A
private native sine harness, including the actual ADC clock, latching and
execution order, measured the delivered cell transfer at 1,643.745 Hz as
gain **0.704242430** and phase **−146.017204°**. Current response and the
command hold add further phase. The correction therefore uses a proper
phase-shaped resonant filter. It bypasses the baseline current notch, which
would substantially attenuate a new damping correction at that frequency.

## Native implementation

The input is the actual `Cell error to deviation` output in the reconstruction
subsystem. A unique explicitly global tag carries that acquired `δL` to a
native two-state Discrete State-Space block. No surface profile, future robot
command, plant truth, current sensor or new sensor is used.

The normalized continuous resonator has

`A = [0, w; −w, −2ζw]`, `B = [0; 2ζw]`, where `w = sqrt(k1/mB)`.

Its bandpass and proper derivative bases have outputs `[0, 1]` and
`[−1, −2ζ]` with direct terms 0 and `2ζ`. Exact zero-order-hold discretization
sets the two native states at 10 kHz. Two real basis weights are solved so
the delivered force has the desired damping phase at the identified carrier,
including the measured acquisition response, 50 µs current response and
10 kHz command hold. The filter is stable, proper and has zero DC gain.

The auxiliary output is limited to **±0.05 A**. It is added after the existing
notch and before the original **±3.5 A** current limit. PI antiwindup uses the
total prelimit request. Acquisition/reconstruction/observer timing remains
unchanged. Actual winding current appears only in a diagnostic harness log;
it is absent from controller inputs.

## Frozen preparation and checks

The bank has 18 families: selected observer parents **24 and 33**, resonator
widths `ζ = 0.05, 0.10, 0.20`, and virtual damping targets
`500, 1500, 3000 N·s/m`. Every family retains the same eleven development
cases and seeds for A masses **0.4 and 1.5 kg**: 396 prepared configurations.
The actual external physical damping remains zero.

The sign/phase harness delivered gain ratios **0.999932–1.000099** and phase
errors **−0.0108 to −0.0126°** at the carrier across the three filter widths.
The inferred virtual modal power had the damping sign. This validates the
input-to-current-to-force phase contract for the diagnostic sinusoid; it is
not a force pass, broadband stability result or hardware measurement.

The emitted-code audit verifies that both the existing upper observer and
new modal filter have live acquired-cell input, that the new current branch
is bounded after the notch, and that antiwindup uses the total request.
Independent saved-copy acceptance checks both observer parents, both A
masses and all three widths. Root performs the full-native development
scoring and retains all failure histories.

## Reproduction and indexing

Read `validation/upper_force_observer_20261005/handoff_cell_modal_v6.json`
after acceptance completes. It binds the sealed model, parameter bank,
library, source audits, phase harness and saved-copy acceptance by hash.
Use the unchanged `native_api.py` and `native_bridge.c` under this recipe
directory. Prepared configurations do not imply executed force experiments.

The zero-based scenario index is

`198*observerIndex0 + 22*(localFamilyID1−1) + 11*massAIndex0 + caseID0`.

Observer index 0/1 means parent 24/33. A index 0/1 means 0.4/1.5 kg.
Local family is `3*(widthID1−1)+dampingID1`, with widths and damping targets
in the order listed above. The global family/gain ID is
`9*observerIndex0+localFamilyID1`. Heavy A, S=2 kg, F=50 N, 90° is case 7,
so its initial probe indices are `18+22*j`, for `j=0..17`.

## Limits

The correction matches one identified carrier. Its off-carrier phase,
noise excitation, bounded-output behavior, interaction with the force loop,
coupled structural modes and parameter/timing mismatch still require
experiments. The finite development bank cannot establish performance on
all sanding trajectories. Installed cell/frame modes and driver/acquisition
phase require measurement before a physical implementation can rely on this
controller. No candidate is promoted by this preparation alone.
