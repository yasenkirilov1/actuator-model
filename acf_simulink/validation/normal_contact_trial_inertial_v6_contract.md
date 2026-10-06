# Private native inertial model v6

Immutable source:
`/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v6-20260930/frozen-inertial-v6`.
The manifest records every model, parameter, recipe and helper hash. Model SHA256:
`72bed757d52763f197e208bc2e9aa210f8d6d3c07394f1b36aefe56114b7a149`.

This is a controller source, not a final qualified parameter set. The MAT file
holds the default archival case; experiment workers set the known masses,
finite sensor stiffness, force reference and identified sensor/drive settings.
No canonical model or shared MATLAB helper was changed by this builder.

## Interfaces retained from v3/v4

The common root `Contact force measurement` page reconstructs the contact vector
from the B–S force-cell vector plus known S mass times measured S acceleration.
Its two-state motion estimator uses S-relative-A position, measured S
acceleration and the already-issued-command robot predictor. Normal projection
uses the aligned vector and causal direction. Its root output 1 is
`reference − estimated normal force`; output 2 is estimated head travel.

`Force controller/Coil PI and PID` consumes that common measurement. The ordinary
PI/PID correction can pass through its feedback-only rolloff before it joins
measured-motion feedforward and holding current. The independently optional
total-current notch precedes the current limit. Antiwindup compares limited
current with the notch output. Native electrical/current-regulator paths and
original column 33 remain the actual current request entering that regulator.

`Force controller/Drive PI and PID` also consumes the common reconstructed
normal error. Its ordinary generic drive units and leadscrew motor loops remain
the baseline implementation; actuator comparisons own any private direct-current
screw extension. Optional current shaping described here is coil-specific.

## V5 force stability and acceleration prediction

`p.robot.forceGate.variation` has fields `enabled` (default false), `meanTime`
(0.1 s), `varianceTime` (0.02 s) and `rmsLimit_N` (0.08 N). The page
`Recenter travel/Force settling gate/Fast force variation and dwell` samples
reconstructed normal error at `p.force.sampleTime`, normally 100 µs. It computes
the present-sample EMA mean, squares the deviation from that mean and computes
an EMA variance. Both filter memories initialize to zero. Equal-order
Discrete Transfer Fcn coefficients `[1−a,0]/[1,−a]` implement each specified
recurrence without an additional sample delay.

Variance must be at most `rmsLimit_N²`. This condition is combined with the
held existing robot-rate instantaneous-error/trend conditions. Every fast
instability resets the continuous dwell counter. Its Unit Delay memory
initializes to zero, and its saturation and threshold are
`1 + ceil(p.robot.forceGate.dwellTime/p.force.sampleTime)`, normally 1001 samples.
The final ready flag is sampled at the original robot rate. Disabled variation
selects the original dwell behavior; the existing overall force-gate bypass
remains effective. The detector never reads actual contact force or surface
truth. Its native 10 s recursion proof passed, although that outgoing deep-notch
controller recording failed the force band and is retained as a failure.

`p.inertial.accelerationPrediction_s` defaults to zero;
`p.inertial.accelerationDerivativeFilter_s` defaults to 0.3 ms. The new
`Filtered measured acceleration rate`, `Acceleration FF prediction time` and
`Causal acceleration for current FF` blocks are in
`Force controller/Coil PI and PID/Measured motion feedforward`. The derivative
is `(1−a)/Ts * [1,−1]/[1,−a]`. Only acceleration-current feedforward uses
`measured aZ + prediction time * filtered measured acceleration rate`.
Force reconstruction, normal direction and guide velocity prediction continue
to use the original synchronized measured acceleration.

## V6 measured drag geometry

`p.contact.geometryMode` is `direct_lateral` (default, preserves v5) or
`measured_drag_ratio`. `p.contact.dragRatioFilter_s` defaults to 5 ms and
`p.contact.dragRatioMinForce_N` to 1 N.

The native page is
`Contact force measurement/Normal force projection/Measured drag geometry`.
With matched reconstructed forces and causal estimated slope `s`, it evaluates:

```text
c = sqrt(1+s²)
raw ratio = −(Rx + s*Rz) / max(1 N, estimated normal force*c)
raw ratio is clamped to [−1,1]
valid = estimated normal force > 1 N
```

Its one-state EMA uses `a = exp(−Ts/dragRatioFilter_s)` and initializes its
memory to zero. Valid samples update `ratio = a*previous + (1−a)*raw`; invalid
samples retain the previous estimate. The optional geometry request is:

```text
requested Rz = normal reference * (1 − filtered ratio*s) / c
```

This uses measured tangential/normal force ratio rather than feeding the
instantaneous radial magnitude through the geometry request. The actual
normal-force feedback projection is unchanged. The physical friction
coefficient and bump profile do not enter this estimator/controller.

## Logging contract

Original 35 physical/control columns are unchanged; column 1 is physical axial
reaction Rz in these private studies. Normal qualification uses
`r.normalContact.data(:,1)` throughout the complete native recording. The
public integrator may present normal N as its primary data channel separately.

`r.normalContact.data` retains eight columns:

1. Actual normal N.
2. Actual lateral Rx.
3. Actual slope, diagnostic only.
4. Reconstructed/estimated normal N.
5. Causal estimated slope.
6. Selected requested axial Rz.
7. Filtered/delayed lateral force-cell measurement, before X inertia correction.
8. Raw sampled issued-command predicted robot velocity, before frame matching.

`r.accelerometer.data` remains
`[measured aZ, measured aX, physical aZ, physical aX, reconstructed Rz, reconstructed Rx]`;
`r.motionObserver.data` remains `[head-gap deviation, head-gap velocity]`.
Existing predictor and robot-gate logs are retained.

New native sidecars, each with `time` exactly equal to `r.time`, are:

- `r.forceVariation.data`: mean normal error, RMS deviation, stable-sample count,
  fast stable condition, fast continuous-dwell ready.
- `r.accelerationFeedforward.data`: original measured aZ, predicted aZ for
  current feedforward.
- `r.measuredDrag.data`: filtered drag ratio, bounded raw drag ratio, valid flag.

## Native proof scope

Default v5 off/zero options reproduced four full v4 histories exactly. Default
v6 direct-lateral mode reproduced those complete v5 histories exactly,
including the new variation and acceleration-feedforward sidecars.

Two v6 enabled-ratio native pilots used S=2 kg, B=0.6 kg, 50 N, 90°,
physical friction 0.3, finite 64 MN/m zero-damping B–S cell, upper rigid,
no external springs, 100 µs current response, matched 100 µs sensor responses,
100 µs sensor delays and 10 µs integration. PI .03/20 had a 1000 Hz feedback
rolloff and no total-current notch; guide prediction was 400 µs, acceleration
prediction zero and the optional variation gate disabled. The clean peak was
0.360121 N and seed-1729 noisy peak 0.485447 N, both over complete 0–4 s histories.
Native drag-ratio raw/EMA/guard/request identities and physical normal force
balance passed. These two pilots establish the implementation; endpoint,
frequency, duration and calibration qualification are separate worker studies.

The independent finite-cell position instrument audit sampled 400001 native
points and proved that its pre-coordinate signal equals physical `xS−xA`
exactly. Its delayed noisy output also matched the specified S–A coordinate
exactly. S-relative-A differed from B-relative-A, excluding accidental use of
magnet-gap position in the finite-cell sensing path.
