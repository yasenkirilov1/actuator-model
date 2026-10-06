# Native screw extension for integration

**Superseded controller source:** before promotion, review found that this frozen version retained the coil's `p.coil.coulombForce=0.2 N` in the screw feedforward instead of converting shaft Coulomb torque by `ell`. The physical plant and rotor/free-body proofs below remain valid. The force results describe the undercompensated controller and must not be presented as the final fair screw limit. The corrected recipe now maps both field names and asserts no unmapped coil gain remains. A subsequent frozen source and campaign will supersede this controller.

The frozen private source is `/home/tommie/.local/state/actuator-model/inertial-actuators-frozen-screw-extension-v4-20260930/`. Its SLX SHA-256 is **14582bcafa0ece4c39a94950f9d49d7786b5004dda6c740f96506ed5c5109018**. The adjacent helpers, input candidate, selected screw and actual recipe snapshot are retained; the machine-readable companion records every hash. Do not use the saved build-baseline coil controller as a qualified selection. Apply the final coil candidate before promotion.

## Plant contract

The common normal-force sensing, finite B–S cell, S acceleration, S–A position, robot predictor, contact and stop logic remain the frozen v4 model. The extension adds **upper-rigid screw support** without substituting soft external springs. Let `ell=lead/(2*pi)`, `Jax=Jrot/ell^2`, and `u=xA-xB`. With finite B–S stiffness and A rigidly following the robot:

```text
L = q1 - mB*(gravityAxial + robotAcceleration)
uAcceleration = (netMotorTorque/ell - L)/(mB + Jax)
transmittedActuatorForce = mB*uAcceleration + L
```

`netMotorTorque=Kt*current-CoulombTorque*tanh(omega/frictionSpeed)-viscousTorque*omega`. Winding voltage, current response, back EMF and current/voltage clipping are preserved. The downstream equations give `Fc=q1+S*(g+aS)` and `Fc=Fa+B*(g+aB)+S*(g+aS)`. The native geometric normal-force projection is unchanged.

The both-rigid structural reference instead uses `Fa=netMotorTorque/ell-Jax*(aA-aS)`. It is counterfactual zero cell compliance, not the selected finite physical sensor.

Native boundary-load wiring is explicit through `Actuator and RABS mechanics/Boundary load selection`. This is essential: a nested `From` cannot read the former parent's local load `Goto`. The screw's inertia gains use `p.d.screwMovingMass`; **the shared mechanics mass `p.d.mu` is unchanged**, preserving the coil plant.

## Controller and selector contract

The selected alternative is `p.screw.controlStrategy='current'`. The native `Force controller/Screw normal PI and PID` uses the same supplied measurements as the coil. Feedback gains have units **A/N, A/(N·s), A·s/N**. Its identified acceleration feedforward includes B+S+reflected rotor mass and issued robot acceleration; geometry and nonzero friction feedforward are causal. It approximates B acceleration by measured S acceleration, which is a limitation for finite-cell modes. No preview, actual surface acceleration or actual current is a controller input.

The selected screw's feedback-only lowpass is stored independently in `p.screw.feedbackFilter`, with `kind='lowpass'`, cutoff 800 Hz and damping 1/sqrt(2). Feedforward bypasses it. The optional total-current notch is disabled in the selected setting. The selector chooses current versus inherited speed control from `p.screw.controlStrategy`, and PI/PID from the shared dropdown. The inherited **speed branch uses exact shaft velocity** and is an optimistic extra encoder reference; it is not the selected common-sensor controller. Choosing the ideal-position reference with rigid attachments produces an explicit error asking for a compliant preset, rather than changing the topology silently.

## Executed evidence and limits

Four native force-balance smoke runs include coil/screw and finite/both-rigid structural configurations. The selected screw has no algebraic loops and its rotor and free-body residuals are below 1e-12 N. Its difficult sharp corner remains outside the force limit:

| Native step | Full recording peak error |
|---:|---:|
| 25 µs | 0.604008855 N |
| 12.5 µs | 0.604245233 N |
| 6.25 µs | 0.604291620 N |

The best setting from 108 distinct native tuning cases is 10 mm lead, RE25 reference winding, 1.58e-6 kg·m² total rotating inertia, 5 A peak/24 V, current PI Kp=0.04080896 A/N and Ki=27.20597 A/(N·s). It reached 4.03337 A and 9.63778 V without voltage clipping. **The enlarged 5 A peak provision is an engineering drive assumption; the RE25's catalog continuous 1.16 A/26.3 mNm provision does not support 50 N at this pitch.** A transient pass would not remove that limit. A larger continuous-torque screw motor is physically plausible, as documented in the separate actuator provision note; it has not been dynamically qualified.

The 32 sharp/slow mass/force/orientation endpoint runs are a separate campaign and were still running when this extension was frozen. Their status must come from `inertial_actuators_screw_endpoints_v4_20260930.json`; planned rows are not executions. The canonical model and shared helpers have not been edited by this worker. Root must merge and verify the selected coil histories, useful selectors, save/reopen behavior and diagram readability before promotion.
