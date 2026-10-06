# Corrected native screw extension: v7 integration contract

Use `/home/tommie/.local/state/actuator-model/inertial-actuators-frozen-corrected-screw-extension-v7-20260930/`, model SHA **ceb689e0c1b83ceedec52bda10f7270881e95672f1fd5cfe6e7d5c978c85514a**. It copies frozen v7 normal-contact sensing/geometry and adds the private native screw extension. Helpers, actual recipe snapshot and every file hash are beside the model. This supersedes the earlier v4 controller mapping; the physical equations remain the same. Only the integrator promotes shared files.

The subsequent **guarded source** to use for integration is `/home/tommie/.local/state/actuator-model/inertial-actuators-frozen-guarded-screw-extension-v7-20260930/`, SHA **c76e34b679215250b6cc3a09046ed6994d736a429df3d0178e5a6fc26f7672a9**. Its `Rigid-A moving B inertia` Gain is `p.mechanics.mB*double(p.mechanics.upperRigid)`, so compliant A preserves the original weighted-load equation. All four native preservation cases pass physical identities/zero-loop checks. The two selected upper-rigid flat cases hold 20 N to roundoff; compliant-A force control fails with the current estimator and remains an explicitly unqualified alternative.

## Native blocks to copy

| Path below model root | Purpose |
|---|---|
| `Actuator and RABS mechanics/Boundary load selection` | Explicitly sends the actual B–S cell force into the actuator when A is rigid; compliant A keeps the previous weighted load. |
| `Actuator and RABS mechanics/Actuator/Leadscrew` | Existing winding/current/friction states plus rigid-A force balance, transmitted-force variant and current/speed input variant. |
| `Force controller/Screw normal PI and PID` | Common measured normal-force error, direct-current PI/PID and causal motion/geometry/friction feedforward. |

The actuator parent has `PropagateVariantConditions='on'`. Explicit port wiring is necessary: a nested From block does not read the former parent's local spring-load Goto. Preserve the existing mechanics mass `p.d.mu`; the screw uses the independent derived field `p.d.screwMovingMass`.

For `ell=lead/(2*pi)`, `Jax=Jrot/ell^2`, rigid A and finite B–S cell:

```text
L = q1 - mB*(g + aR)
uAcceleration = (netTorque/ell - L)/(mB + Jax)
Fa = mB*uAcceleration + L
```

`netTorque=Kt*i-tauC*tanh(omega/omegaEps)-bRot*omega`. The native both-rigid structural reference uses `Fa=netTorque/ell-Jax*(aR-aS)`. All winding, clipping, free-body, contact and failure-stop equations remain native.

## Controller conversion and independent time constants

| Coil quantity | Screw controller quantity |
|---|---|
| Force constant Kf | `Kt/ell` |
| Coulomb force | `tauC/ell` (both old `coulombFriction` and current `coulombForce` field names are mapped) |
| Viscous force coefficient | `bRot/ell^2` |
| Translational friction transition speed | `omegaEps*ell` |
| Flat current | `p.d.screwHoldCurrent` |
| Current limits | `±p.screw.peakCurrent` |
| Normal force P/I/D | `p.screw.currentForceKp/Ki/Kd`, units A/N, A/(N·s), A·s/N |
| Force derivative time | `p.screw.currentDerivativeFilter` |

The cloned current controller rejects any remaining coil-specific Gain reference, except the intentionally shared PI/PID strategy. Its acceleration feedforward is `-(B+identifiedS+Jax)*ell/Kt * measuredPredictedSaccel`, plus `Jrot/(ell*Kt) * issuedRobotAcceleration`. B acceleration is approximated by S acceleration; this approximation is a finite-cell limitation. Guide velocity is also estimated from measured S–A motion rather than measured B velocity.

These cloned blocks have independent screw parameters and derived coefficients:

| Path below `Force controller/Screw normal PI and PID` | Parameters |
|---|---|
| `Normal force PI and PID/Feedback structural-mode filter` | `p.d.screwFeedbackFilter`, from `p.screw.feedbackFilter` |
| `Optional structural-mode current notch` | `p.d.screwCurrentNotch`, from `p.screw.currentNotch` |
| `Measured motion feedforward/Filtered measured acceleration rate` | `p.d.screwAccelerationPrediction`, from `p.screw.accelerationDerivativeFilter_s` |
| `Measured motion feedforward/Acceleration FF prediction time` | `p.screw.accelerationPrediction_s` |
| `Measured motion feedforward/Acceleration FF command filter` | `p.d.screwAccelerationFeedforwardFilter`, from `p.screw.accelerationFeedforwardFilter_s` |

The optional FF filter is a present-value EMA `[1-a,0]/[1,-a]`, `a=exp(-Ts/tau)`, with identity coefficients for tau=0. These FF-only operations do not alter force reconstruction, direction estimation or guide acceleration.

## Public selector rules

1. Voice coil selects actuator `Voice coil` and controller `Coil PI and PID`.
2. Leadscrew with `p.screw.controlStrategy='current'` selects actuator `Leadscrew`, current source `Current command`, and controller `Screw normal PI and PID`.
3. Leadscrew with strategy `speed` selects current source `Speed loop` and controller `Drive PI and PID`. This inherited branch has exact shaft-velocity sensing and is an optimistic extra-encoder reference, not the selected two-sensor-plus-accelerometer strategy.
4. PI/PID dropdown changes the shared strategy and enables the chosen branch's own P/I/D gains. The acceleration-feedforward checkbox applies the common enable scale to either current controller.
5. Upper-rigid leadscrew is permitted. Transmitted-force source is `Finite B-S cell` or `Rigid B-S reference` according to lowerRigid. Boundary-load source is `Rigid A` or `Compliant A` according to upperRigid. No mechanical preset substitution occurs.
6. The unsupported rigid ideal-position reference raises an explicit error requiring an intentionally selected compliant preset.

## Selection and evidence

The provisional best **worst-of-clean/noisy** heavy sharp corner uses the JSON structure `selected_screw_minimax.json.screw` in the frozen folder: 10 mm lead, total J=1.58e-6 kg·m², RE25 winding, 5 A peak/24 V, Kp=0.04080896 A/N, Ki=27.20597 A/(N·s), own feedback-only 800 Hz lowpass, 200 µs acceleration prediction, 300 µs derivative time, FF-command filter zero and total-current notch disabled. Its full 10 s clean/noisy seed1729 errors are **0.514960/0.543611 N**, so it fails ±0.5 N. Choosing the clean-only 350 µs minimum gives 0.476907 N clean but 0.657266 N noisy and is not the robust selection.

**The saved source workspace still carries the earlier 350 µs clean reference.** Explicitly apply the JSON structure or the later selected candidate before promotion. Six v7 FF-filter cases are a separate retained report, `inertial_actuators_acceleration_command_filter_v7_20260930.json`; its recorded status determines execution counts. The source structural/controller mapping proof does not establish force qualification. Every completed native case checks normal-force identity, downstream and actuated free-body balance and rotor balance, with residuals below 1e-12 N and no algebraic loops.

The guarded source workspace already carries the 200 µs minimax screw. Its selected parameter MAT is `inertial_actuators_selected_screw_minimax_v7.mat`. All six v7 FF-filter cases completed and failed (full peaks 0.650–0.741 N), so the filter-zero minimax selection remains unchanged. No further open tuning is planned.

The RE25 continuous catalog rating of 1.16 A/26.3 mNm does not sustain 50 N at 10 mm lead. The 5 A peak drive is an engineering transient provision without a qualified pulse duration. A larger continuous-torque motor is physically plausible but was not dynamically simulated. This comparison does not establish universal screw unsuitability or physical prototype qualification. Final save/reopen/diagram and endpoint checks belong to the promoted source and selected parameter version.

## Updated common-sensor selection

The figures immediately above use the recorded **200 µs force/acceleration filters, 8 g acceleration range and 0.00187038 m/s² quantization**. They are diagnostic results and are not a paired comparison with the later 100 µs coil provision. The guarded physical extension remains valid; its saved workspace is not the final common parameter preset.

The later six executed cases in `inertial_actuators_matched100us_prediction_v7_20260930.json` explicitly use synchronized 10 kHz sampling, 100 µs force and acceleration filters, 100 µs sensor transports, 50 g acceleration range and 0.00468 m/s² quantization. The modeled noisy case uses 0.02 N force noise, 0.0123 m/s² acceleration noise and 1 µm position noise. Noise-off cases retain acceleration quantization.

| Screw acceleration prediction | Noise off | Seed 1729 |
|---|---:|---:|
| 0 µs | 0.517969 N | 0.554612 N |
| 100 µs | 0.480006 N | 0.524711 N |
| 200 µs | 0.450267 N | 0.532311 N |

The selected paired worst case is **100 µs**, still outside ±0.5 N with noise. Import **`p.screw` only** from `inertial_actuators_selected_screw_matched100us_v7.mat` (SHA256 `04d080d943e2e185b39580e173e7b9be45096d67ffe1126eed268be91f77bd08`); the parameter variable is `p`. The matching JSON binds the actual selection. Apply the integrator's final common/coil parameter file separately. No physical/controller wiring changed for this selection.

The corrected matched-sensor endpoint and refinement campaigns are recorded separately under `inertial_actuators_matched100us_*`; inspect their actual statuses before counting runs or making a force claim. The older slow cases had a mismatch between physical and issued scan speed and are excluded from fair slow-bump claims, as recorded in `inertial_actuators_contract_review_v7.json`. New cases record both values and set both to 100 mm/s sharp or 20 mm/s slow.
