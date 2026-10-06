# Frozen V8 acquisition and controller contract

The immutable experiment source is
`/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v8-contract-20260930/frozen-inertial-v8`.
The model SHA256 is `73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643`;
`normal_contact_trial_candidate.mat` SHA256 is
`36e61547154e87d77b289d053cab40553ed849d61fa3cf278f20f902a3ad0708`.
The complete helper, source, coefficient and native bridge hashes are in
`normal_contact_trial_inertial_v8_manifest.json`. No canonical promotion is implied.

The selected physical model uses A=0.4 kg, B=0.6 kg, known S=0.1–2 kg,
upper rigid attachment, lower opposed spring pair k=19,746 N/m and c=60 Ns/m.
Each spring has stored preload **33.6 N**. The series cell is represented by the
retained massless-series approximation, not another mounting resonance.
The native force controller is measured-acceleration PI, P=0.3 A/N, I=20 A/(N s),
feedback-only 1 kHz Butterworth rolloff, no total-current notch, drag-ratio
geometry request with 5 ms filtering, 400 us guide-velocity prediction, and a
proper 1.5 kHz lower-load inverse. It reads no profile or future robot commands.

## Actual acquisition and identified motion frame

Raw acquisition is 100 kHz; controller latching is 10 kHz. Each channel uses
engineered 2-pole 5.1 kHz Butterworth antialias conditioning followed by a fresh
50 us EMA. Nominal nonzero electronics/processing pipeline is 20 us. This is
an identified engineering response target; it does not claim an exact guaranteed
manufacturer transfer function.

| Configuration | Field |
|---|---|
| Enable actual path | `p.acquisition.enabled=true` |
| Raw sample period | `rawSampleTime_s=1e-5` |
| Actual common AA cutoff/damping | `antiAliasCutoff_Hz=5100`, `antiAliasDamping=1/sqrt(2)` |
| Actual EMA time | `emaTime_s=50e-6` |
| Actual common pipeline | `pipelineDelay_s=20e-6` |
| Actual differential AA factors | `forceAntiAliasScale`, `lateralAntiAliasScale`, `accelerationAntiAliasScale`, `positionAntiAliasScale`, all 1 |
| Actual differential pipelines | `forceDelayOffset_s`, `lateralDelayOffset_s`, `accelerationDelayOffset_s`, `positionDelayOffset_s`, all 0 |
| Fixed identified issued-motion frame | `identifiedAntiAliasCutoff_Hz=5100`, `identifiedEmaTime_s=50e-6`, `identifiedPipelineDelay_s=20e-6` |

Prepared coefficients live in `p.d.acquisition.responses.{force,lateral,acceleration,position,frame}`;
each has `numerator`, `denominator`, and `steadyStatePerUnitInput`.
Nominal numerator is `[0.003773241118012132 0.007546482236024264 0.003773241118012132 0]`;
denominator is `[1 -2.371153349157007 1.906701403726165 -0.5204550900971091]`.
Three canonical DF-II states start at static measured input divided by the
denominator sum. The trailing numerator zero is intentional: the EMA uses the
fresh raw sample. Actual AA/pipeline changes do not update the identified
issued-motion frame. Position has its actual measurement pipeline and actual
common conditioning; robot acceleration/velocity use the identified frame.
Acceleration is conditioned once, then used directly by the 2-state motion observer.

## Noise, raw calibration and initialization

`p.sensor.noiseRms=0.02 N`, `p.contact.lateralNoiseRms_N=0.02 N`,
`p.accel.noiseRms=0.014 m/s²` and `p.sensor.strokeNoiseRms=1 um` describe
nominal conditioned RMS. Force/position sources are bounded uniform; accelerometer
noise is Gaussian. The raw RMS multiplier is **3.96705064104797**, derived from
nominal impulse energy **0.06354253123515088**. It stays referenced to the nominal
5.1 kHz/50 us path when actual AA tolerances vary, preserving raw noise PSD.
Acceleration range is 50 g; retained post-response ADC quantum is 0.00468 m/s²,
a conservative quantization placement. Static specific gravity is present before
scale/noise/filter/ADC and known static tare follows quantization; dynamic inertia
is never removed by static tare.

Active raw/native filter paths, all prefixed by `acf_force_control_normal_trial/`:

- `Force measurement/Engineered common AA and EMA`: raw sampled tared axial cell + normalized force noise, IC `p.force.reference * ...responses.force.steadyStatePerUnitInput`.
- `Lateral B-S force measurement/Engineered common AA and EMA`: sampled lateral cell + normalized force noise, IC `-p.force.reference*p.contact.mu * ...responses.lateral.steadyStatePerUnitInput` for loaded physical equilibrium.
- `S accelerometer and ADC/Z engineered common AA and EMA` and X: inputs remain `Z/X noisy sensor`, after specific acceleration/gravity and `Z/X sensor scale`; IC is `(1+p.accel.scaleError)*g_axis * ...responses.acceleration.steadyStatePerUnitInput`.
- `Contact force measurement/Head motion estimator/Position common AA and EMA`: zero deviation IC; physical readout is S relative to A.
- The same page's `Robot acceleration common AA and EMA` and `Robot velocity common AA and EMA`: zero IC and identified frame coefficients.

Residual frame/cross-axis instrumentation must also update each active higher-order
filter's static IC consistently. Editing only the inactive legacy EMA memory does
not initialize the active sensor path.

Known S mass refreshes automatically from `p.mechanics.mS`.
`p.sensor.distalMassCalibrationScale=1` is the nominal identified mass multiplier;
set that explicit scale for an identified-S residual, instead of overwriting only
`distalMass_model_kg`. Actual and identified masses remain distinct.
The lower inverse uses identified B, k, c and Kf through the exact fields
`identifiedMassB_kg`, `identifiedStiffness_N_m`, `identifiedDamping_Ns_m`,
`identifiedForceConstant_N_A`. Its desired-load subtraction uses
`p.sensor.distalMass_model_kg * measured_S_acceleration_z`.

## Actual robot transport and force-stability gate

`p.robot.delay` now uses actual-only held 100 us transport samples in
`Robot command delay/Robot command samples`; `p.d.robotDelaySamples` is 750/800/850
for 75/80/85 ms. `p.d.actualRobotTransportSampleTime_s=1e-4`.
Issued commands, predictor and robot gate remain at 100 Hz; predictor 80 ms still
uses `p.robotPrediction.delaySamples=8`. Full default 80 ms histories are bitwise
identical to the coarser source. Finer actual timing does not reveal actual robot
motion to the observer.

The fast variation gate is enabled. At 10 kHz it filters reconstructed normal
error with 100 ms mean and 20 ms variance, requires RMS <=0.08 N, and continuously
resets a 100 ms dwell on every fast failure. Existing robot-rate instantaneous
0.2 N band and 2 N/s trend conditions remain. Ready is sampled at 100 Hz before
commands are sent. Gate input is reconstructed normal force before controller
filters; no physical truth force enters it.

## Evidence and retained logs

All nine actual-AA native 10 s cases passed: a clean heavy check and eight noisy
S/F/orientation endpoints, worst absolute normal error **0.3224154873 N**.
Every run checks the actual stored preload and spring packing; combined free
length is 37.18 mm and combined installed length 30.373558 mm. These are a finite
nominal endpoint cohort, not a hardware qualification or complete uncertainty claim.
The final actual/identified-frame and fine-transport representation bridge is
bitwise exact on a full noisy-heavy 10 s native trace. An S-only 2 ->0.1 kg edit
also reproduces the explicit known-light-mass trace bitwise.

Experimental `r.data` remains the original 35-channel physical layout with **Rz**
in column 1. `r.normalContact` has 8 columns; acceleration 6, motion observer 2,
command predictor 3, measured drag 3, fast variation 5, acceleration FF **2**,
and lower inverse **4**. The separate public runner remaps authoritative force
metrics to N after preserving the axial diagnostic. Lower inverse columns are
`[desiredRzDeviation, identifiedS*measuredAz, desiredLowerLoadDeviation, extraCurrent]`.
