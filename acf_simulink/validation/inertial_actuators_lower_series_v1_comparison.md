# Paired native actuator comparison on lower-series v1

Both modeled actuators pass the four selected sharp-bump conditions with the compliant carrier and its causal inverse feedforward. The worst voice-coil error is **0.328079499 N** and the worst leadscrew error is **0.333697430 N**, evaluated over every native timestamp of each ten-second recording, including startup. These are eight executed nominal recordings, not qualification of the whole force/mass/orientation range or a physical prototype.

The small RE25 reference screw motor cannot continuously sustain the required load. Its transient tracking pass is useful evidence that a screw can work with suitable mechanics and control; it does not make this reference winding a satisfactory continuous-sanding actuator. The voice coil remains the simpler direct-drive provision, subject to the integrator's separate electrical, sensing, mechanical and repeat tests.

## Recorded conditions and results

Each recording contains one 1 mm high, 10 mm wide bump scanned at 100 mm/s. The true and controller-issued tangential scan speeds agree. The holder B is 0.6 kg; tool S and force/orientation settings are shown below. A is rigidly connected to the imposed robot boundary. Sanding drag ratio is 0.3. Both branches use the same spring/damper link, B–S multi-axis force measurement, S acceleration measurement, S–A position measurement, reconstruction of the actual contact-force vector, causal direction estimate, robot-command prediction and recentering gate.

| Condition | Coil peak error | Screw peak error | Coil peak current | Screw peak current |
|---|---:|---:|---:|---:|
| S=2 kg, 50 N, 90°, stochastic noise off | 0.326402 N | 0.333697 N | 2.80676 A | 3.91651 A |
| S=2 kg, 50 N, 90°, seed 1729 | 0.328079 N | 0.331290 N | 2.90379 A | 4.08102 A |
| S=2 kg, 5 N, 0°, seed 1729 | 0.321389 N | 0.322911 N | 1.48231 A | 2.56614 A |
| S=0.1 kg, 50 N, 90°, seed 1729 | 0.094038 N | 0.216180 N | 2.65109 A | 3.66882 A |

All eight complete ten seconds without contact loss, travel stop, overcurrent, current-command clipping or voltage clipping. Maximum coil voltage is 17.48043 V and maximum screw voltage is 10.03765 V, both below their 24 V limits. Across these recordings travel spans 10.56194–16.15596 mm inside the modeled 25 mm range. The force margin in the worst recorded case is approximately 0.1663 N. One noisy seed and four paired conditions do not establish a statistical or manufacturing uncertainty margin.

The common sampling period is 100 µs. Force and acceleration filtering and identified transport are each 100 µs; position transport is 100 µs. Noisy rows use 0.02 N RMS force noise per modeled force channel, 1 µm RMS position noise and 0.0123 m/s² RMS acceleration noise. Acceleration range is 50 g and quantization is 0.00468 m/s². Noise-off rows retain that quantization. The baseline current response target is 100 µs. These are matched model assumptions; residual gain, tare, bias, phase and temperature errors are not covered by this campaign.

The recentering gate is the source's slower provision: 100 ms force filtering and dwell, 0.2 N error band, 2 N/s maximum rate, variation gate disabled, robot command delay 80 ms. Later acquisition and gate candidates must retain their own results; these recordings are not relabeled as a later version.

## Mechanics and causal current control

The mechanical path is `B -> opposed guided spring pair with parallel damper -> joining frame -> multi-axis B–S load cell -> S`. Effective lower-link stiffness is **19,746 N/m**, damping **60 N·s/m**, with **33.6 N common preload** for the spring packing provision. The cell itself is nominally 64 MN/m and is represented by a massless series-compliance approximation; 19,746 N/m is not its structural stiffness. The separate cell/mount resonance and structural modes are outside this approximation. The source manifest records combined free spring length 37.18 mm and maximum combined installed length 30.37356 mm, each under the spring-only 100 mm condition.

Both current controllers use PI feedback with the same force-domain P=6 N/N and I=400 N/(N·s), a feedback-only two-pole 1 kHz Butterworth lowpass, and a proper lower-link inverse regularized by a two-pole 1.5 kHz Butterworth filter. The coil gains are 0.3 A/N and 20 A/(N·s). The screw gains are divided by its axial force constant: **0.408089598 A/N and 27.205973178 A/(N·s)**. D, total-current notch and acceleration-prediction time are zero. The inherited current-force reconstruction and geometry feedforward use measurements and previously issued robot commands; neither branch receives the bump profile, actual surface acceleration, actual slope or future commands.

Let `ell=lead/(2*pi)`, `Kax=Kt/ell` and `Jax=Jrot/ell^2`. The screw current feedforward in force units is:

```text
requested_Rz - (B+identified_S)*g
  - (B+identified_S+Jax)*measured_S_acceleration
  + (B+Jax)*proper_inverse[delta_requested_Rz - identified_S*measured_S_acceleration]
  + Jax*issued_robot_acceleration + estimated_internal_friction
```

Here `proper_inverse = s^2/(identified_k + identified_c*s)` with the source's regularizing two-pole filter. Only the deviation from initial equilibrium enters the inverse, avoiding an artificial initial reference step. With the 10 mm lead, Kt=0.0234 N·m/A and total rotor/shaft/coupler inertia 1.58e-6 kg·m², **Kax=14.702653619 N/A** and **Jax=0.623758998 kg**. The inverse therefore uses effective identified carrier mass **1.223758998 kg**, not B alone. The independent derived screw coefficient structure is `p.d.screwLowerLoadInverse`; the coil's coefficient structure remains unchanged.

The native screw retains winding inductance/resistance, current response, voltage/current saturation, rotor states, transformed inertia and explicit friction. Coulomb torque 0.001 N·m converts to **0.628318531 N**, rotational viscous damping 1e-6 N·m·s/rad converts by `1/ell^2`, and the 1 rad/s friction transition converts by `ell`. The current controller's conversions match those of its physical branch. Source guide feedforward predicts velocity by 400 µs. Its causal observer estimates **S–A motion**, which approximates B–A actuator-gap motion under the compliant link. No B position sensor or new gap-velocity estimator was added. The approximation must be validated in the assembled mechanism.

For rigid A and finite lower link, the native screw free-body equations are:

```text
L = q1 - B*(g + aRobot)
uAcceleration = (netTorque/ell - L)/(B + Jax)
actuatorForce = B*uAcceleration + L
netTorque = Kt*i - tauC*tanh(omega/omegaEps) - bRot*omega
```

These are implemented by ordinary native Gain, Sum, winding, Integrator, filter and limit blocks. Parameter preparation computes coefficients rather than solving the plant in hidden MATLAB code. Global reduced-mass fields remain unchanged; the screw uses its independent derived moving mass.

## Continuous sizing and comparison limits

The reference RE25 302005 is a 24 V, 130 g winding example with Kt=0.0234 N·m/A, R=2.32 Ω, L=0.238 mH, catalog rotor inertia 1.08e-6 kg·m², continuous current 1.16 A and continuous torque 26.3 mN·m. Added shaft/coupler inertia 0.5e-6 kg·m² is an engineering assumption. The model allows bipolar 5 A peaks with **no qualified pulse duration or motor heating state**. Catalog sources are [maxon product data](https://www.maxongroup.com/maxon/view/product/302005) and [maxon RE25 technical sheet](https://www.maxongroup.com/medias/sys_master/root/8992314097694/EN-22-154.pdf).

At 50 N/90°, the ideal pitch transform needs **3.40075 A continuously** and **26.83098 W cold copper loss** before guide losses. A separate 90% transmission-efficiency sizing bound raises this to **3.77861 A**, still outside the catalog rating. All four selected screw cases fail continuous static catalog/current/torque sizing; even the heavy 5 N/0° case requires about 1.3947 A to oppose gravity. Native transmission is an ideal reversible pitch transform with explicit losses; 90% efficiency is a conservative sizing calculation, not an extra hidden model loss.

The nominal coil Kf=20 N/A, R=4 Ω, L=3 mH, bipolar 3.5 A/24 V provision requires 2.5 A and 25 W cold copper loss at 50 N/90°. Its 100 µs current response and thermal capacity remain component/drive design targets. The two winding losses alone do not determine whole-system energy use or actuator superiority. The coil avoids screw conversion, shaft/coupler inertia and backlash, but its guide, cables, force-position variation, body mass, heat dissipation and inner current loop need physical validation.

A larger continuously rated screw servo is plausible; it was not substituted into these dynamic runs. Previous pitch/controller diagnostics and finite-cell recordings describe their recorded configurations and are separate from this soft-link result. This small bounded comparison supports neither universal screw failure nor qualification of every voice coil. It establishes that the corrected native screw extension can meet the transient target at the selected common-source conditions, while its small reference motor remains thermally unsuitable for continuous high-force sanding.

## Numerical evidence and frozen integration source

An extra native heavy/noisy coil reference was executed **before** inserting the screw extension. After insertion, the corresponding coil history is bitwise identical in all 35 original channels and seven sidecar groups: normal contact, accelerometer, motion observer, measured drag, force variation, acceleration feedforward and lower inverse. This extra reference is not included in the eight scored comparison cases. There were zero preparation rejections and zero algebraic loops in the selected source branches.

Every native history verifies normal-force identity and downstream/actuated free-body balances. Maximum residuals are 2.8422e-14 N, 3.5527e-15 N and 7.1054e-15 N respectively; screw rotor-balance residual is at most 2.2205e-14 N. All recordings use fixed RK4 step 10 µs with native 100 µs sensor/controller sampling. No solver-refinement campaign has yet been executed on this new compliant-carrier comparison; the previous finite-cell screw refinement is not a substitute. Final source promotion requires the integrator's refinement, save/reopen and readability checks.

Evidence is retained in `inertial_actuators_carrier_inverse_pair_v1_20260930.json`, its CSV/checkpoint, eight `..._native_0001.mat` through `..._native_0008.mat`, and `..._unmodified_source_coil.mat`. All run failures from older campaigns remain retained separately. The export recipe `inertial_actuators_export_carrier_inverse_v1.m` reads existing histories and exports graph points with every full-native global force peak retained; export does not execute simulations.

Frozen donor: `/home/tommie/.local/state/actuator-model/inertial-actuators-frozen-lower-series-screw-extension-v1-20260930/`. Model SHA256 is `9bb441184d1cd96d8ed3c75d4f6fc3aec33389f567ff989a94b13c5af0bf1096`; source SLX SHA256 is `a0c2e358503d51c53c630ca436f20c1a618c5d024f98dd46338ee328f6c94f81`; candidate MAT SHA256 is `94fcfa29c708c705c2dccf3db34296568fd90f53f6b3b51cfe929e5a52e53738`; actual recipe snapshot SHA256 is `ed8406f63baae269c83b80ea4500a5fea0ed74926342a2b7d5fe1d68bf573abd`. Helpers and complete manifest are beside the donor.

Selected parameter file `inertial_actuators_selected_screw_carrier_inverse_v1.mat` contains variable **p**, derived from noisy heavy case 6 and prepared by the frozen private helper. Import **p.screw only** when merging with a later common/coil candidate. Its SHA256 is `16a3c15423ecc1a24e5efb9b20e81204e18e924cf151f4f05e5a9109717edb43`. Derived coefficients must be recomputed with the promoted helper and actual identified mass/stiffness/damping. The frozen donor's saved workspace retains the source coil base rather than implying that its entire parameter structure is the final delivered preset.
