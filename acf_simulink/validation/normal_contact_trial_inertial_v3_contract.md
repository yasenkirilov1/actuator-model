# Private native inertial controller contract

The immutable worker source is
`/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v3-20260930/frozen-inertial-v3-helpers1`.
It contains `acf_force_control_normal_trial.slx`, `normal_contact_trial_candidate.mat`
and private `helpers/`. The model SHA-256 is
`a16c8a2deaa98efafda008a82017bbc211d21a2022c375608211a6d7bea892ed`.
Full hashes are in `normal_contact_trial_inertial_v3_helpers1_manifest.json`.
The helpers1 revision guards unchanged mask and variant assignments during
Fast Restart; its model and candidate are byte-identical to frozen v3.

This source provides the common native sensing and controller architecture.
Its saved mechanical preset has both attachments rigid. Finite load-cell
stiffness, current-notch selection and broad parameter qualification are
separate recorded experiments; this source is not a physical qualification.

## Shared measurement and native controller pages

`Contact force measurement` is independent of actuator/controller selection.
It accepts the sampled axial-cell error and measured S-relative-to-A position.
Its outputs are reconstructed normal-force error `reference − estimate`, and
estimated head travel. Its three shallow child pages are:

- `Contact force reconstruction`: tared force plus known downstream S inertia.
- `Head motion estimator`: matched position/issued-command prediction and a
  native two-state head-gap/velocity observer.
- `Normal force projection`: the measured force vector projected onto the
  causal normal direction inferred from S velocity and commanded scan speed.

With Z along the flange axis and X along the imposed scan direction,
`Rz = FcellZ_tared + ms*aZ_measured` and
`Rx = FcellX_tared + ms*aX_measured`. Static tare removes the fixed gravity
vector. The controller computes `s_hat = vS_hat/vScan_commanded` and
`N_hat = (Rz − s_hat*Rx)/sqrt(1+s_hat²)`.
The model supplies physical surface motion only to the plant and sensor pages;
the controller cannot read true slope, contact force, friction coefficient or
future surface motion.

The selected coil variant is `Force controller/Coil PI and PID`. It contains
`Normal force PI and PID` and `Measured motion feedforward` pages, a holding
current, current summation, optional native current notch, current limits and
antiwindup. `p.coil.strategy` is `pi` or `pid`. The legacy `observer` alias is
accepted by the private selector and maps to PI when Kd is zero, PID otherwise.
The coil-law coefficients are `p.inertial.forceKp`, `forceKi`, `forceKd`,
`derivativeFilter` and `positionBandwidth_Hz`. Ki must remain positive because
the antiwindup conversion divides by Ki. PID enables the configured derivative
gain; with Kd zero its derivative contribution is zero.

`p.inertial.accelerationFeedforwardScale` controls the measured-acceleration
current term `−(B+S)*aZ/Kf`; `guideFrictionFeedforwardScale` controls viscous and
Coulomb guide compensation. `p.contact.geometryFeedforwardScale` controls the
axial correction needed to achieve the requested normal force. These fields
are scalar gains, so bounded experimental scales can be retained. The private
UI offers PI/PID and an acceleration-feedforward checkbox; explicit UI edits
set that acceleration scale to 0 or 1.

`Force controller/Drive PI and PID` uses the same reconstructed normal error
and existing drive-specific command units. The screw speed/current loops are
preserved. The inherited selector switches screw/ideal comparisons to the
saved compliant attachment preset when rigid links are requested. The native
upper-rigid screw extension is a separate actuator experiment. No new screw
acceleration feedforward or screw current notch is supplied by this source.

## Causal guide prediction and optional current notch

`p.inertial.guideVelocityPrediction_s` defaults to 0. The feedforward page
optionally uses
`vGap_ff = vGap_hat + Tlead*(aRobot_pred_aligned − aS_measured)` for guide
friction only. Normal projection continues to use timestamp-matched vGap.
The exact Gain path is
`Force controller/Coil PI and PID/Measured motion feedforward/Identified guide prediction time`.

`p.inertial.currentNotch` has `enabled`, `frequency_Hz`, `zeroDamping` and
`poleDamping`. The native block is
`Force controller/Coil PI and PID/Optional structural-mode current notch`.
The private `acf_prepare` computes `p.d.currentNotch.numerator`, `denominator`
and `initialStates` with a bilinear transform prewarped at the specified
frequency. The disabled coefficients are `[1 0 0]/[1 0 0]`. The two native
direct-form states start at holding current divided by the denominator sum,
so the enabled filter begins at its flat-contact DC equilibrium. Antiwindup
compares saturation output against the notch output. This filter shapes coil
current in amperes; it does not shape the screw speed request.

## Sensor timing, noise and calibration parameters

The starting candidate has synchronized 100 µs sampling and identified
100 µs delays. `p.sensor.filterTime` and `p.accel.filterTime` are both 200 µs.
Matching filters are also applied to position and issued-command motion
prediction so the projected force vector and estimated direction refer to
the same effective time. The accelerometer response is an identified
first-order envelope; it is not an exact manufacturer analog transfer function.

`p.accel` contains `enabled`, `filterTime`, `delay`, `delaySamples`, `noiseRms`,
`bias`, `scaleError`, `range_g` and `quantum`. The starting acceleration noise
is stationary colored Gaussian RMS 0.012 m/s² per axis. Gaussian noise is
injected before the identified filter with a discrete variance normalization.
The starting range is ±8 g and acceleration ADC quantum is 0.00187038 m/s².
Static quiet tare includes quantization; `bias` represents residual drift
after that tare, and `scaleError` remains a dynamic calibration error.
Axial and lateral cell noise remain independent 0.02 N RMS sampled noise.

Changing a known tool mass requires updating `p.mechanics.mS`,
`p.sensor.distalMass_kg`, and `p.sensor.distalMass_model_kg`, followed by private
`acf_prepare`. The saved source assumes known B and S for current feedforward
and holding-current preparation. An independent mass-mismatch experiment
must also use the identified mass in those controller terms while retaining
the physical mass in the plant.

## Structural cell compliance versus optional springs

Use `p.sensor.structuralStiffness_N_m` and
`p.sensor.structuralDamping_Ns_m` as descriptive physical sensor metadata in
the promoted preset. When no external lower spring is present, effective
native link `p.mechanics.k1/c1` represents this finite B–S cell stiffness and
damping, with `lowerRigid=false`. Such a case has no external lower-spring
catalog entry and should not consume the 100 mm spring packing allowance.
External spring flags, dimensions and packing should be recorded separately.
The current trial helper performs dynamics with the effective k1/c1 values;
the sensor metadata do not independently alter the equations.

## Native histories and qualification evidence

The existing `r.time`/`r.data` schema has its original 35 channels. Rz in its
first column remains the physical axial reaction. Additional histories are:

| History | Columns |
| --- | --- |
| `r.normalContact.data` | N actual, Rx actual, true slope, N estimate, slope estimate, requested Rz, measured lateral cell force, raw issued-command predicted robot velocity |
| `r.accelerometer.data` | measured aZ, measured aX, physical aZ, physical aX, reconstructed Rz, reconstructed Rx |
| `r.motionObserver.data` | head-gap deviation, head-gap velocity |

Every sidecar time vector equals `r.time` exactly, including t=0 and every
native solver sample. The old nine-state observer history is absent.
Tracking qualification uses actual `normalContact.data(:,1)`, complete run
duration, contact and travel-stop flags. The old axial tracking metric is
diagnostic only.

Four paired noisy endpoint recordings show bitwise-identical original 35,
normal 8, acceleration 6 and motion 2 channels between frozen inertial v2 and
v3 with notch bypass and guide prediction time 0. The first heavy corner
(S=2 kg, 50 N, 90°, μ=0.3, seed 1729) has peak normal error 0.349922765 N at
50 µs and 0.349876248 N at 25 µs. The native selected controller has no
algebraic loops and reads only sensor channels and commands already issued.
See `normal_contact_trial_inertial_v3_equivalence.json`,
`normal_contact_trial_inertial_audit.json`, and
`normal_contact_trial_inertial_frame.json` for the associated recordings.
