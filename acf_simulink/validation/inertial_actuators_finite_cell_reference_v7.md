# Native actuator comparison: finite-cell reference

This is a completed **reference experiment**, not the later compliant-carrier design or a physical prototype qualification. The controlled metric is the maximum absolute error of true normal workpiece force across every native timestamp, including startup. The target is ±0.5 N for 5–50 N, S=0.1–2 kg with B=0.6 kg, and fixed orientations 0–90°. This actuator study samples the force, mass and orientation endpoints; the integrator's broader study supplies additional grid points and uncertainty checks.

## Recorded system

A is rigidly constrained to the modeled robot motion. The B–S cell has finite axial stiffness **64 MN/m**, with no external springs and no external damping. The controller receives the B–S force vector, measured S acceleration and measured S–A position, plus robot commands already issued. It receives no bump profile, future motion or surface preview. Contact force is reconstructed with the known/tared downstream S mass and projected onto the estimated normal before feedback.

The explicit common sensing provision is synchronized **10 kHz**, identified **100 µs force and acceleration filters**, **100 µs sensor transports**, 0.02 N force noise, 0.0123 m/s² acceleration noise and 1 µm position noise. Acceleration range is 50 g with **0.00468 m/s² quantization**. These remain modeled system targets requiring assembled acquisition and calibration. Stochastic-noise-off cases retain acceleration quantization. Force noise is bounded uniform and acceleration noise is Gaussian; a single noise seed does not establish statistical robustness or sanding-vibration performance.

The recorded robot gate uses error-band, rate and dwell checks with 100 ms smoothing. Its optional fast-variation check is disabled. Later gate, acquisition or mechanical changes require their own comparison; these rows are not silently transferred to a different system.

The small screw provision uses the [maxon RE25 302005](https://www.maxongroup.com/maxon/view/product/302005) winding reference: Kt=23.4 mNm/A, R=2.32 Ω, L=0.238 mH and motor rotor inertia 1.08×10⁻⁶ kg·m². Add **0.5×10⁻⁶ kg·m²** shaft/coupler inertia as an engineering assumption. At **10 mm/rev**, Kax=14.7027 N/A and reflected rotating mass is **0.623759 kg**. The native winding/current loop has a 100 µs identified response target with bipolar **5 A peak/24 V** provision. The peak allowed duration was not qualified.

Internal friction remains nonzero: 1 mNm Coulomb, 1 µNm·s/rad viscous and a 1 rad/s smooth transition. Correct force-domain conversion gives 0.628319 N Coulomb friction, bRot/ell² viscosity and omegaEps*ell transition velocity. Actual screw, bearings, alignment, cables and seals require measurement; no zero-friction result is substituted.

The selected current-domain controller uses PI gains **0.04080896 A/N** and **27.20597 A/(N·s)**, independent 800 Hz feedback lowpass, **100 µs causal acceleration prediction**, 300 µs derivative filtering and no FF-command filter or total-current notch. It includes reflected-inertia and issued-robot-acceleration terms. It approximates B motion by S motion for the stiff cell. The inherited speed controller additionally assumes exact shaft velocity and is an optimistic encoder reference; it is not the selected common-sensor controller.

## Executed results

Six complete 10 s heavy sharp-corner pilots selected by the worst of stochastic-noise-off and seed1729:

| Acceleration prediction | Noise off | Seed1729 |
|---|---:|---:|
| 0 µs | 0.517969 N | 0.554612 N |
| 100 µs | 0.480006 N | 0.524711 N |
| 200 µs | 0.450267 N | 0.532311 N |

The 100 µs prediction minimizes this paired worst case. Selecting the lowest clean peak alone would choose a different setting and hide a noisier result. The selected pair still misses the force target by **0.024711 N** with noise.

The endpoint study then executed **32 native 10 s recordings**, with zero preparation rejections:

| Scenario and tool mass | Passed recordings | Worst full-record force error |
|---|---:|---:|
| Sharp, S=0.1 kg | 8/8 | 0.250180 N |
| Sharp, S=2 kg | 4/8 | 0.524711 N |
| Slow, S=0.1 kg | 8/8 | 0.234685 N |
| Slow, S=2 kg | 8/8 | 0.465528 N |

Each row comprises F=5/50 N, θ=0/90°, and stochastic noise off/on. **28/32** recordings pass. All four S=2 kg noisy sharp corners fail; clean heavy sharp recordings and all slow recordings pass. Sharp is one 1 mm high, 10 mm wide event at 100 mm/s. Slow is one 1 mm high, 60 mm wide event at 20 mm/s. Physical and issued scanning speeds match. These are single events within full 10 s recordings, not repeated-bump qualification.

All recordings complete without contact loss, travel stops, current clipping or voltage clipping. Across the endpoint grid peak current is about **4.044 A** and peak voltage about **9.652 V**, leaving transient reserve relative to the modeled 5 A/24 V drive. Raising voltage alone does not address an unclipped controller/sensing failure in this model.

The heavy clean sharp case remains below 0.5 N under solver refinement:

| Solver step | Full-record peak error |
|---|---:|
| 25 µs | 0.479788 N |
| 12.5 µs | 0.479903 N |
| 10 µs, endpoint recording | 0.480006 N |
| 6.25 µs | 0.480040 N |

The three refinement runs are native histories, not recalculated statistics. The maximum normal-force identity and physical free-body/rotor residual across the endpoint study is **2.85×10⁻¹⁴ N**. No algebraic loops occur. These algebraic checks establish internal model consistency; they do not validate the physical prototype.

## Continuous design limit

The RE25 catalog continuous provision is **1.16 A/26.3 mNm**. Static 50 N at 10 mm/rev needs **3.401 A** and **26.831 W** cold copper loss before guide losses. The conservative 90% motoring sizing allowance needs 3.779 A. Twenty endpoint conditions exceed continuous current/torque sizing, even though most transient force histories pass. Heating and permitted peak duty were not simulated, so this reference is unsuitable for continuous full-range sanding as specified.

A shorter pitch reduces current but raises reflected inertia: at 3/5/10 mm the assumed rotating assembly reflects **6.931/2.495/0.624 kg**. Cold 50 N copper losses are **2.415/6.708/26.831 W**. At 3 mm, catalog continuous torque with the separate 90% sizing allowance only provides **49.6 N** before guide losses. Passing a current criterion alone is insufficient.

A larger continuous-torque motor can remove the static limitation. The [maxon EC-i30 539489](https://www.maxongroup.com/maxon/view/product/539489) reference is 242 g, 103 mNm continuous and 1.55×10⁻⁶ kg·m² motor inertia. At 10 mm lead it provides 64.7 N ideal, or 58.2 N with the illustrative 90% sizing allowance, while the assumed shaft/coupler gives 0.809 kg reflected mass. **It was not simulated.** Commutation/current conventions, ripple, screw elasticity, backlash, bearings, friction and heating require a separate identified model and native test. The RE25 result cannot prove that every screw actuator is unsuitable.

The comparison therefore supports retaining current-controlled screws as an alternative for less demanding motion, while selecting the final actuator from the updated common-mechanics qualification and continuous thermal provision. This file does not recommend an unqualified RE25 purchase or claim a voice coil is universally superior.

## Inspectable evidence

- `inertial_actuators_reviewed_endpoint_comparison_v7.json` binds all 32 rows, source/helper/recipe provenance and exact assumptions.
- `inertial_actuators_matched100us_prediction_v7_20260930.json` binds the six executed selection cases.
- `inertial_actuators_matched100us_solver_refinement_v7_20260930.json` binds the three refinement histories.
- `inertial_actuators_selected_screw_matched100us_v7.mat/.json` stores the independently selected screw parameters; import `p.screw` only into the final common candidate.
- `inertial_actuators_chart_data_v7.json` supplies reviewed pilot and analytical pitch-sizing points.
- `inertial_actuators_matched100us_replay_v7_20260930.json` exports six native traces at approximately 1 ms spacing plus their native extrema. Every plotted force peak is checked against the full native metric; full-resolution MAT histories remain available.
- `inertial_actuators_carrier_inverse_plan.md` is preparation for the different compliant-carrier model, with zero executions at publication.

Historical diagnostic campaigns and excluded conditions remain in their original files and internal review notes. They do not enlarge the executed count or force claim for this reference system.
