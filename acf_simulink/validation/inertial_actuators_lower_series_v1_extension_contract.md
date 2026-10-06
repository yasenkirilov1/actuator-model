# Lower-series v1 native screw donor contract

The reviewed donor is `/home/tommie/.local/state/actuator-model/inertial-actuators-frozen-lower-series-screw-extension-v1-20260930/`. It contains the native model, all matching private helpers, source candidate, actual recipe snapshot, selected screw parameters, executed comparison and a complete manifest. No canonical files were changed. The integrator alone promotes the reviewed extension.

SLX SHA256: `9bb441184d1cd96d8ed3c75d4f6fc3aec33389f567ff989a94b13c5af0bf1096`.

Recipe SHA256: `ed8406f63baae269c83b80ea4500a5fea0ed74926342a2b7d5fe1d68bf573abd`.

Source common plant/controller is lower-series v1: SLX SHA256 `a0c2e358503d51c53c630ca436f20c1a618c5d024f98dd46338ee328f6c94f81`. This is the source's 100 µs sensor/transport, slower 100 ms robot-gate provision. Later acquisition/gate source changes need their own pairing and qualifications.

## Native paths to transfer

| Path below model root | Change |
|---|---|
| `Actuator and RABS mechanics/Boundary load selection` | Sends the actual lower-link force into the upper-rigid screw, retains original weighted load for the source coil. |
| `Actuator and RABS mechanics/Actuator/Leadscrew` | Rigid-A B/rotor balance, current/speed source variant and finite-link/rigid-reference transmitted-force variant. |
| `Force controller/Drive PI and PID` | Optional causal measured S–A velocity feedforward for the inherited speed reference; inactive in the selected current comparison. |
| `Force controller/Screw normal PI and PID` | Direct-current PI/PID with mapped torque/pitch/friction/current fields, reflected inertia and independent screw coefficient pointers. |
| `Force controller/Screw normal PI and PID/Lower spring inverse feedforward/Proper identified lower-link inverse` | Numerator/denominator use `p.d.screwLowerLoadInverse`; source coil continues using `p.inertial.lowerLoadInverse`. |

The actuator subsystem has variant-condition propagation enabled. The rigid-A moving-B Gain is `p.mechanics.mB*double(p.mechanics.upperRigid)`. Derive `p.d.screwMovingMass=mB` when upperRigid, otherwise preserve the original reduced moving mass. Do not overwrite global `p.d.mu`.

```text
ell = lead/(2*pi); Jax = Jrot/ell^2; Kax = Kt/ell
L = q1 - B*(g+aRobot)
udd = (netTorque/ell - L)/(B+Jax)
Fa = B*udd + L
```

The current branch's acceleration feedforward is `-(B+identifiedS+Jax)/Kax * measured_aS`, plus the separate `Jax/Kax * issued_robot_acceleration`. Retain both terms. The cloned controller maps every force/current/friction/time field:

```text
coil Kf -> Kt/ell
Coulomb force -> tauC/ell
viscous force coefficient -> bRot/ell^2
friction transition velocity -> omegaEps*ell
current limits -> +/-screw.peakCurrent
flat holding current -> p.d.screwHoldCurrent
PI/PID gains -> screw.currentForceKp/Ki/Kd (current units)
force derivative time -> screw.currentDerivativeFilter
feedback/notch/acceleration-prediction coefficients -> independent p.d.screw... fields
```

## Reflected-inertia inverse coefficients

The source input remains `delta requested axial Rz - identified S*measured S acceleration`. Its downstream inertia Gain is **`p.sensor.distalMass_model_kg`**, preserving actual versus identified mass. No force-channel truth, future profile or new B sensor is added.

The helper recomputes the proper bilinear 1.5 kHz inverse from identified stiffness/damping and the known screw transform. Its effective mass is `source.identifiedMassB_kg + screw.rotorInertia/ell^2`, and its force constant is Kax. Independent `p.d.screwLowerLoadInverse` must be derived from the base identified B on every preparation; do not repeatedly add Jax to an already reflected mass.

At Ts=100 µs, k=19,746 N/m, c=60 N·s/m, base identified B=0.6 kg, 10 mm lead and total Jrot=1.58e-6 kg·m²:

```text
effective mass = 1.2237589981488473 kg
force constant = 14.70265361880023 N/A
numerator   = [3.5786134162816774, -3.5786134162816774,
               -3.5786134162816774, 3.5786134162816774]
denominator = [1, -1.7154119456215446,
               0.9957927719956349, -0.263401371552554]
```

The source coil numerator is `[1.2898400806393393,-1.2898400806393393,-1.2898400806393393,1.2898400806393393]`, with the same denominator. These numbers bind this recorded source, not an arbitrary sampling period or future link design.

## Selector contract

1. Voice coil selects its native actuator and `Coil PI and PID` branch. Its current and inverse coefficients remain unchanged.
2. Screw strategy `current` selects its native actuator, `Current command` request source, and `Screw normal PI and PID` branch. Strategy `speed` selects `Speed loop` and inherited `Drive PI and PID` instead.
3. For the upper-rigid screw, the boundary load selector chooses `Rigid A`; its input is actual q1. All other source branches keep the original weighted load. This actor-specific rule is necessary for bitwise source-coil preservation, including the old gap-acceleration diagnostic.
4. `lowerRigid` chooses `Finite B-S cell` or `Rigid B-S reference` for the screw transmitted-force source. No spring preset is silently substituted. The source's normal-force reconstruction remains common to both actuators.
5. PI/PID selection and motion-feedforward enable can remain common, while gain units and coefficients are branch-specific. Inactive speed/reference branches are alternatives, not qualified by these current-branch histories.
6. An ideal-position reference requesting rigid mechanics is rejected explicitly rather than changing topology.

The integrator has separately reviewed a physical gap-acceleration diagnostic `aA-aB`. Changing that diagnostic in the final assembly can be appropriate but is distinct from this donor's exact source-coil preservation proof. Rotor balance here uses the physical A/B acceleration columns rather than the source's reduced-mass gap-acceleration diagnostic.

## Selected parameters and evidence

Use `inertial_actuators_selected_screw_carrier_inverse_v1.mat`, variable **p**, SHA256 `16a3c15423ecc1a24e5efb9b20e81204e18e924cf151f4f05e5a9109717edb43`. Import **p.screw only** and recompute derived fields using the promoted helper/common candidate. The frozen equivalent is `selected_screw.mat`, with matching JSON coefficients. The saved model workspace retains the original coil base; the workspace alone is not the final integrated preset.

Selected screw: current-domain PI, lead 10 mm, RE25 winding Kt=0.0234/R=2.32/L=0.238 mH, total rotor inertia 1.58e-6 kg·m², 5 A peak/24 V. P=0.408089598 A/N, I=27.205973178 A/(N·s), D=0, common 1 kHz Butterworth feedback, 1.5 kHz proper inverse, no acceleration prediction/command filter/current notch. Source guide prediction remains 400 µs and uses S–A motion as an approximation to the actuator gap motion.

Eight scored native ten-second histories pass; four per actuator. One additional source-coil parity reference was executed before extension. All 35 original coil channels and the seven recorded sidecar groups are bitwise identical after extension. Screw force/rotor balances are within 2.3e-14 N; there are no selected-branch algebraic loops, stops or clipping. `inertial_actuators_lower_series_v1_comparison.md` and the complete JSON/CSV/native histories bind the results. Solver refinement and public save/reopen/readability checks still belong to final integration.

The RE25 reference fails continuous static sizing at all four selected conditions. At 50 N/90°, 3.40075 A and 26.83098 W cold copper loss exceed its 1.16 A catalog continuous provision. The 5 A peak model has no qualified pulse duration. These transient successes do not qualify that small motor for continuous sanding or establish universal suitability of a larger, unsimulated screw servo.
