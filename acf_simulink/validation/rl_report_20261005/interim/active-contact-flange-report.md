# Active contact flange for sanding

## Intro

We want a compact active flange that keeps the **normal pressing force experienced by the workpiece within ±0.5 N of a 5–50 N target**, including while crossing a **1 mm high, 10 mm wide bump at 100 mm/s**. It should work with fixed flange orientations from **0° to 90°** and interchangeable tool masses **S = 0.1–2 kg**. The assumed moving holder B is 0.6 kg, giving **0.7–2.6 kg total tool-side mass**. Robot-side body A is budgeted separately: 0.4 kg in the generic baseline, approximately 1.5 kg for the real coil example evaluated below.

The controller may use a multi-axis load cell between B and S, tool position relative to A, an accelerometer mounted on S, known tool weight or a gravity tare, and a causal prediction of robot commands already issued. It receives no bump profile or surface preview. We want understandable native Simulink blocks, a measured performance margin, reasonable spring and damper values, and a practical actuator and sensing recommendation. Any springs must fit within **100 mm combined free length and 100 mm combined maximum installed length**. Removed springs are replaced by rigid connections.

The first-prototype accelerometer and ADC **chips together should cost less than €100**, for assembly on our own PCB. This budget excludes the load cell, amplifiers, PCB fabrication and assembly. The normal-force requirement is unchanged.

## Question

**How should we design a compact active sanding flange that meets the pressing-force requirement with useful margin, reasonable mechanics and affordable sensing—and does a small learned controller improve that system?**

Robot recentering accepts ±7.5 mm about mid-stroke without a centering penalty; force tracking and physical stops retain their limits.

The component questions below cover mechanics, actuator/drive, force/position sensing, acceleration sensing, acquisition, fast force control, robot commands, learned control and prototype qualification. Each uses the same **Intro → Question → Results → Recommendation** structure. The learned policies have the same causal input restrictions and physical limits as the existing controller.

## Results

**Scientific-reference result:** all 3,500 nominal sharp-bump cases pass, worst error **0.315811 N**. Four repeated 30 s stress cases with the selected static calibration pass at **0.415311 N** worst. The recommended prototype uses a voice coil, measured-inertia feedforward PI, a synchronized multi-axis load cell and S-mounted accelerometers, with assembled calibration and dynamic verification.

**Lower-cost sensing update, 2 October:** the ADS131M08 at 30 ksample/s with a matched 8 kHz complete-response envelope passes all **700 new nominal conditions**, worst **0.292804 N**. Seven selected hot-coil long/mixed cases pass at **0.438821 N** worst. The 999-recording cost study supports reducing ADC cost while retaining the reference acceleration sensors; the cheaper sensor envelopes consume substantial margin or fail. These results have their own source, noise seeds and qualification scope.

**Unified learning study, 5 October — budget retarget in progress:** one shared recurrent policy controls coil current and robot correction. The faster reference-sensor candidate passes all 28 development cases with 0.372088 N worst error. The new €100 chip budget selects an ADXL354C/ADS131M08 candidate for separate tuning and testing; the earlier force result does not qualify that changed sensing chain.

### Mechanics: load path, springs and guides

#### Intro

The mechanical assembly must preserve a measurable load path while allowing the actuator to correct tool motion quickly. Spring packing, guide friction, structural modes and the partition of moving mass all affect the force controller.

#### Question

Which attachment arrangement and approximate spring/damper values can handle the specified bump across the tool-mass range, fit the spring-only 100 mm limits and remain practical to build?

#### Results

##### Mechanical model and sensor placement

The robot supplies the slow gross motion. The upper attachment connects the robot boundary R to body A. A short-stroke actuator between A and holder B supplies the fast correction. From B, the mechanical load passes through the lower attachment and then the multi-axis load cell to the sanding tool S, which contacts the workpiece. Either attachment can be compliant or rigid.

The position sensor measures S relative to A. The accelerometer is mounted on S, beyond the load cell.

![Mechanical load path and sensor placement](figures/acf_mechanism_and_sensors.svg)

*Conceptual layout of the selected candidate. Every lower axial load passes through the cell; S–A position spans the full mechanism. This is not a dimensional drawing.*

An attachment spring is an opposed, guided pair: both springs stay in compression while their difference supplies a signed net force. A damper acts across the same relative motion. A rigid attachment removes that motion while retaining the connection and its reaction force. Zero spring stiffness without a replacement connection would leave a free body, so that is a different physical system.

Positions are positive away from the workpiece. The actuator gap is `u = xA − xB`; actual travel is its configured midpoint plus u. R is the prescribed robot-side boundary. A/B/S masses follow Newton's law. With both attachments compliant:

```text
q1 = q10 + k1 (xS − xB) + c1 (vS − vB)
q2 = q20 + k2 (xA − xR) + c2 (vA − vR)

mA aA = Fa − q2 − mA g_axis
mB aB = q1 − Fa − mB g_axis
mS aS = Rz − q1 − mS g_axis
```

`q1` is the lower connection force, `q2` the upper connection force, `Fa` the actuator's net axial force after actual guide friction, and `Rz` the axial component of workpiece reaction. The coil's electromagnetic force is `Kf I`; subtract guide friction to obtain Fa in these motion balances. `g_axis = g cos(theta)` represents gravity along the flange axis. The orientation changes the operating load, rather than changing the mass or stiffness.

For rigid attachments the model enforces the corresponding constraints and calculates their reaction forces. It does not approximate a rigid connection with an enormous numerical spring. If both attachments are rigid, A follows the robot and B follows S, so:

```text
Rz = Fa + (mB + mS) (g_axis + aS)
```

This arrangement still moves: the actuator changes A–B separation. Removing the external springs removes additional compliant joints; it does not lock the actuator at one position.

The selected candidate makes the **upper attachment rigid** and puts an **opposed spring pair and a parallel damper in the lower attachment**. All of that attachment's load passes through the multi-axis cell before reaching S. A spring or guide that transfers axial load directly to S around the sensing element would invalidate the force reconstruction. The spring/damper joining frame is treated as massless in this model; its real inertia is an important prototype check.

Each recording starts at its own loaded equilibrium for the selected force and orientation. The reference link loads are `q10 = Fref − mS g_axis` and `q20 = Fref − (mA+mB+mS) g_axis`; the starting actuator load is `Fa0 = Fref − (mB+mS) g_axis`. An opposing spring pair's common preload is a separate quantity. The fixed-orientation recordings do not establish the transient behavior of changing force target or rotating a loaded tool.

##### Contact model and disturbance severity

The contact model represents a tool following a prescribed smooth surface while sliding tangentially. For local slope s, the unit normal and tangent in the scanning plane are:

```text
n = (−s, 1) / sqrt(1+s²)
t = ( 1, s) / sqrt(1+s²)
R = N n − mu N t
N = n · R
```

N is the normal pressing force we score. The sliding drag is represented by `mu`; the plant may use a specified value, but the controller does not receive it. It measures the lateral force component instead. For positive scanning:

```text
N  = Rz sqrt(1+s²) / (1−mu s)
Rx = −N (s+mu) / sqrt(1+s²)
N  = (Rz − s Rx) / sqrt(1+s²)
```

The lateral reaction is carried by the guides and robot boundary. The current model includes that force in contact reconstruction but does not simulate lateral guide deflection or robot structural bending.

The bump is a smooth polynomial over its width. With H=1 mm, transit time T=0.1 s and normalized progress `q=(t−t_bump)/T`, its height is `h=64 H q³(1−q)³` for 0≤q≤1, and zero outside. Height, velocity and acceleration join continuously at entry/exit. This gives a reproducible demanding disturbance without a mathematically instantaneous displacement step.

The peak downward surface acceleration is **2.4 m/s²**, the peak axial speed is approximately **34.3 mm/s**, and the peak jerk is approximately **384 m/s³**. Thus a 2 kg downstream tool can contribute **4.8 N** of dynamic inertia to the difference between the cell reading and contact reaction. With a 0.1 kg tool the same effect is only **0.24 N**. The entire moving B+S mass also affects actuator effort, independently of how much mass lies beyond the load cell.

![Specified bump motion and required tool-inertia compensation](figures/bump_motion_and_inertia.svg)

*Calculated from the specified surface polynomial; this figure shows required motion and inertial force, rather than a controller qualification.*

Contact is maintained only while its calculated required normal reaction is nonnegative. The first contact loss or travel-stop hit stops the recording and marks it failed. This chosen model does not continue into free flight, hard-stop impact, or re-contact.

##### Finite stiffness, joining mass and guides

Rigidly fastening the sensor does not make it infinitely stiff. In the stiff lower-connection reference, the specified Mini27 axial stiffness and B=0.6 kg produce a maintained-contact B-side mode near **1.64 kHz**. The selected external spring/damper instead gives a **passive B-stage frequency near 28.9 Hz in maintained contact**, with damping ratio about 0.28. That is the open mechanical frequency; the closed-loop response also contains force control, acquisition, current dynamics and filtering. Its native plant uses **k1=19,746 N/m and c1=60 N·s/m directly**, with a massless joining frame and no independent cell state. It neglects the representative 64 MN/m cell's approximately **0.031% static series-compliance correction**. The stiff no-springs reference retains the finite cell.

A real joining plate between the spring pair and cell introduces another degree of freedom. Illustrative 50–200 g plate masses with a 64 MN/m cell give high modes around **5.7–2.85 kHz**, rather than removing every structural mode. These are analytical estimates, not native force qualification of a built fixture. Keep the joining assembly light and stiff, measure its modes under load, and include any material mode in the identified controller model before accepting the prototype's margin.

Partition the assembled masses by that load path. S inertia reconstruction uses only mass downstream of the sensing element; the inverse's B mass is actuator-side mass above the spring. An intervening frame is a separate mass: at low frequency its inertia loads the spring together with S, but it is not sensed as downstream tool inertia. Split any frame mass out of the nominal holder budget and include its measured dynamics, rather than entering the same total mass into every compensation gain.

A compact crossed-roller guide is a plausible low-friction starting point. THK quotes a rolling friction coefficient of 0.001–0.0025 for its crossed-roller guides. For an explicitly assumed 50–100 N equivalent bearing/preload load, that corresponds to 0.05–0.25 N of rolling drag. The modeled 0.2 N Coulomb term is therefore a reasonable target to investigate, rather than a measured assembled value. Seal drag, preload, contamination and moving cables require their own checks; the 2 N·s/m viscous term and the smooth reversal transition are engineering assumptions. [THK guide specification](https://tech.thk.com/es/products/pdf/en_b00_015.pdf).

##### Spring and damper search

The lower-compliance search contains **168 native recordings** across PI, filtered PID and inverse-assisted control, using equal clean/seed-1729 pairs at each compared tuning. The net pair stiffnesses are 12.826 and 19.746 N/mm, with 0, 15 and 60 N·s/m damping. No tested plain PI or PID setting meets ±0.5 N on both recordings while also satisfying spring packing. With the causal lower-link inverse, the 19.746 N/mm pair passes with either 15 or 60 N·s/m damping. The selected 60 N·s/m setting gives better mechanical damping and is the setting tested repeatedly and across the broader operating range.

![Controller, lower spring rate and damper search](figures/inertial_mechanics_spring_tuning.svg)

*168 native recordings. Each cell shows the best complete clean/noisy tuning pair within that family, so this is a tuning search rather than an equal-gain comparison. Hatched cells fail spring working range even where force tracking passes; contact-loss cases are not shown as successful low errors.*

The softer 12.826 N/mm pair can meet force tracking with inverse assistance, but its recorded excursions do not fit the selected spring working envelope. Zero external damping is also tested; it does not provide a qualified alternative in this search. Completely removed springs retain rigid connections and the load cell's own finite stiffness, as in the structural reference above. Removing damping does not remove the actuator's modeled guide friction.

##### Preload and early working-envelope evidence

The repeated cohort admits a common preload interval of approximately **30.38–36.86 N per spring**, and the configured 33.6 N lies inside it. With 18.1 mm nominal spring free length and ±0.49 mm length tolerance, the conservative total free length is **37.18 mm** and total maximum installed length is **30.37 mm**. In the worst repeated trace, individual spring force spans about **5.70–61.50 N** and compression **0.58–6.23 mm**. The parallel damper reaches about **33.2 mm/s**, **1.99 N** and **0.066 W instantaneous dissipation**; the modeled 10 s mean dissipation is only 1.48 mW. Stiction, seal force and frequency-dependent damping still need characterization.

##### Identified mechanical coefficients and numerical refinement

An additional **18 native recordings** vary the controller's identified stiffness and damping independently by ±20% while holding the actual selected mechanics fixed. Both clean and noisy heavy-tool cases pass all nine combinations; the worst is **0.36166 N**. This establishes tolerance to those particular identification errors. It does not establish the behavior of a physically changed spring or damper, which is a separate test.

Six solver-refinement recordings at 10, 5 and 2.5 µs all pass. The maximum peak-error difference is approximately **11.9 µN** for the noisy trace and **1.1 µN** for the clean trace. Thus the candidate's roughly 0.13 N repeated-test margin is much larger than this measured time-step sensitivity.

##### Actual mechanical tolerance and combined calibration cases

Physical spring and damper tolerance receives a separate test. **72 native recordings** vary actual lower stiffness and damping independently by **±20%**, while the controller continues to use the nominal identified 19,746 N/m and 60 N·s/m. Each of the nine mechanical combinations has eight common cases: the light/heavy tool endpoints, a clean condition and noise seeds 7, 29 and 1729 at 50 N/90° and drag ratio 0.3. All 72 meet the force requirement; the worst is **0.37437 N**, without drive clipping, contact loss or a travel stop. This is physical-parameter variation, rather than merely changing the controller's coefficients.

The approximate spring target is **20 mm nominal free length per spring**, ±0.49 mm length tolerance, 9.873 N/mm nominal rate, at least the modeled 68.54 N individual force envelope and 3.18 mm cyclic working stroke. Keep the pair guided and use **33.6 N common preload per spring**. The original shorter 18.1 mm geometry leaves insufficient compression room for the softest ±20% specimens at this fixed preload. Increasing the approximate free length to 20 mm changes packing, not the native force dynamics. Analytical rescoring of the same 72 complete histories makes **all 72 packing checks pass**, with no invented additional force simulations.

Across those physical variations, a shared feasible preload interval is approximately **30.91–38.89 N per spring**. Individual spring force spans **5.65–61.55 N**, compression **0.477–7.790 mm**, and the smallest tolerance-inclusive installed spring length is **11.72 mm**, above the 11.05 mm minimum working length plus clearance. The conservative cyclic-motion bound is at most **1.09 mm**, below the 3.18 mm provision. Combined free spring length is **40.98 mm**; maximum combined installed length is **35.31 mm** across stiffness tolerance. Both remain well below the 100 mm spring-only limits. These are approximate working-envelope targets, not a claim that a specific catalog spring has already been qualified.

![Actual spring and damper tolerance with fixed nominal controller](figures/inertial_mechanics_v8_actual_tolerance.svg)

*72 native force histories, eight per mechanical combination. Force values use every sample. The packing panel is an analytical 20 mm geometry rescore of those histories; it does not add force experiments.*

The lower pair shapes relative dynamics and keeps the sensing load path attached; its common internal preload does **not** offload static pressing force to the robot. At 50 N and 90°, the generic 20 N/A, 4 Ω coil still holds 2.5 A and dissipates 25 W. The real coil's holding heat is evaluated separately below.

A separate **six-record combined stress comparison** adds actual stiffness +20% and damping −20% to the heavy simultaneous-calibration corner, using three common noise seeds. With the inverse still using nominal k/c, all three pass but the worst reaches **0.489996 N**, leaving only **0.010004 N** reserve. Changing only the inverse's identified stiffness/damping to the known actual **23,695.2 N/m and 48 N·s/m** lowers the worst to **0.453026 N**, recovering **0.036970 N** and leaving **0.046974 N** reserve. Peak current and voltage rise to 3.0045 A and 21.3319 V, without clipping; packing still passes.

![Assembled mechanical identification improves the combined stress corner](figures/inertial_mechanics_mixed_comparison.svg)

*Three paired seeds at one heavy combined corner, six complete four-second native histories. The bars score all samples and the entry traces show every sample in the displayed window. Known actual k/c are supplied exactly in the second group; this does not establish a residual k/c calibration allowance or a complete combined-tolerance region.*

This makes assembled mechanical identification part of the prototype recommendation. Measure the spring pair's loaded stiffness and damper response and enter those values in the inverse. It does not justify assuming that identifying k/c also removes unrelated sensor or robot-timing errors.

The tighter static calibration targets described below are also tested at this same mechanical corner. Three further four-second records preserve the known actual k/c, physical inputs, noise and timing, and halve only the same sixteen static residuals. All pass, with peaks **0.373044, 0.376938 and 0.357305 N**. The worst reserve is **0.123062 N**, compared with 0.046974 N before that calibration refinement. Peak current/voltage are **2.9987 A / 21.9529 V**, without clipping or stops; spring packing still passes. This is a second narrow paired check of the proposed calibration targets, not a full combined mechanical/calibration grid.

![Static calibration at the assembled mechanical stress corner](figures/inertial_mixed_static_calibration.svg)

*Three new four-second histories paired with three reused known-k/c baselines. Every 400,001 native sample determines each metric; the entry detail retains all native samples. Physical and identified k/c are fixed while only sixteen static calibration residuals halve.*

#### Recommendation

Use an **upper rigid attachment** and a **lower guided opposed spring pair** with **19.746 N/mm net stiffness**, approximately **9.873 N/mm per spring**, **33.6 N common preload per spring**, and **60 N·s/m parallel damping**. Start around **20 mm free length per spring**, allowing the stated ±0.49 mm tolerance; the conservative combined free length is **40.98 mm**, and the evaluated maximum combined installed length is **35.31 mm**. Provide the modeled individual force and cyclic-travel envelope.

Keep the complete lower axial load through the cell, budget **B = 0.6 kg** separately from **S = 0.1–2 kg**, and identify assembled stiffness/damping for the inverse. Measure guide stiction, cable drag and joining-frame modes. The finite tests support this starting design; the spring-only packing limit does not qualify the whole flange size.

### Actuator and electrical drive

#### Intro

The actuator supplies both fast correction and continuous pressing effort. Current response, usable voltage, moving inertia, continuous force over stroke and heat must be assessed together.

#### Question

Should the prototype use a voice coil or leadscrew, and what force constant, stroke, current, voltage and thermal capacity must the actuator and driver provide?

#### Results

##### Representative real coil and current-drive hardware

The **Moticont LVCM-070-051-01** is a useful real voice-coil reference: 25.4 mm stroke, 24.4 N/A force constant, 6.4 Ω resistance, 4 mH inductance and a 240 g moving coil. Its published continuous force is 64.7 N near the centre, falling to approximately 52 N near the ends in the manufacturer's curve. The stationary body weighs 1.24 kg, so it requires roughly 1.5 kg of robot-side A allowance rather than the generic 0.4 kg baseline. The moving-coil mass fits inside B=0.6 kg, leaving about 360 g for the holder, guides, sensor attachments and cables. [Product specification](https://www.moticont.com/lvcm-070-051-01.htm), [manufacturer drawing and force curve](https://www.moticont.com/pdf/lvcm-070-051-01.pdf).

At a 50 N horizontal hold this coil needs approximately 2.05 A and dissipates 26.9 W at its nominal resistance. A 39% resistance increase raises the hold voltage from 13.1 V to 18.2 V and coil heat to 37.4 W. That makes hot-coil amplifier output headroom a more meaningful voltage question than simply comparing nominal 24 V and 48 V supplies. The continuous-force curve, thermal mounting, usable position range and actual current response still need checking together.

A linear current amplifier is useful for the first precise-force prototype because it avoids introducing PWM current ripple into the averaged-voltage model. This carries a cooling cost. With 20 N/A and 4 Ω, a 50 N horizontal hold needs 2.5 A: the coil dissipates 25 W, and an idealized 24 V linear output stage dissipates another 35 W. Raising the supply to 48 V raises that driver loss to 95 W at the same operating point. These are electrical heat calculations; actual voltage drops and thermal mounting must be checked against the driver's safe operating area.

A switched amplifier can be more efficient, but its current ripple needs measurement and, where necessary, filtering or a qualified mechanical attenuation path. A current-loop bandwidth number alone does not bound instantaneous force ripple. Keep the bulky driver and cooling on the robot or in its cabinet, rather than counting them inside the 0.6 kg moving holder allowance.

##### Native actuator comparisons and transmission sizing

A voice coil converts signed current directly into signed force and avoids screw backlash and rotating inertia. It is a strong first-prototype choice, but it needs continuous holding current, adequate cooling, a low-friction guide, calibrated force constant over the used stroke and a clean current regulator. A large catalogue peak force does not establish continuous 50 N sanding performance.

The lower spring/damper and correct carrier inverse also make a leadscrew dynamically viable. Four equal-condition pairs using the earlier reference sensing chain all pass:

| Ten-second native recording, one bump | Voice-coil peak error | Leadscrew peak error |
| --- | ---: | ---: |
| S=2 kg, 50 N, 90°, clean | 0.3264 N | 0.3337 N |
| S=2 kg, 50 N, 90°, seed 1729 | 0.3281 N | 0.3313 N |
| S=2 kg, 5 N, 0°, seed 1729 | 0.3214 N | 0.3229 N |
| S=0.1 kg, 50 N, 90°, seed 1729 | 0.0940 N | 0.2162 N |

The coil reaches approximately **2.90 A / 17.48 V** in this cohort, and the screw **4.08 A / 10.04 V**, without clipping. These eight recordings are a bounded actuator comparison, not full-grid qualification of the screw or the final acquisition chain.

The final combined native diagram receives its own **four matched pairs with the selected actual acquisition graph**, plus one screw refinement. All **nine recordings pass**. These recordings last **four seconds with one bump**, and are separate from the ten-second reference table above.

| Final four-second native recording | Voice-coil peak error | Leadscrew peak error |
| --- | ---: | ---: |
| S=2 kg, 50 N, 90°, clean | 0.293260 N | 0.301007 N |
| S=2 kg, 50 N, 90°, seed 1729 | 0.283522 N | 0.292191 N |
| S=2 kg, 5 N, 0°, seed 1729 | 0.274796 N | 0.282736 N |
| S=0.1 kg, 50 N, 90°, seed 1729 | 0.100459 N | 0.219436 N |

The final coil cases reach **2.8992 A / 18.4449 V**, and the screw **4.1554 A / 10.3831 V**. All avoid clipping, contact loss and travel stops, pass spring packing, and satisfy independent normal/axial force balances within 2.85×10⁻¹⁴ N. The heavy noisy screw repeat at 5 µs changes its peak by only **1.49×10⁻⁸ N**. This is useful native branch acceptance; it remains four operating conditions, rather than the coil's complete grid or combined calibration qualification for the screw.

![Final matched native voice-coil and leadscrew results](figures/inertial_final_actuator_peak_errors.svg)

*Four final-acquisition pairs, all eight base recordings passing. Peaks use every original sample; the ninth recording is the separate 5 µs screw refinement.*

![Final heavy-tool bump with the two actuators](figures/inertial_final_actuator_heavy_bump.svg)

*Heavy S=2 kg, 50 N, 90°, seed 1729. Display histories are reduced to approximately 1 ms with native global extrema retained; qualification uses every 10 µs sample. Travel is actuator B–A travel, distinct from the measured S–A position.*

The screw model retains motor current/inductance, shaft friction, lead and rotor inertia. With `ell=lead/(2 pi)`, the reflected rotational inertia is `Jax=Jrot/ell²`. The motor must accelerate that inertia as well as B. For the compliant lower link, its inverse uses **mB+Jax**, and issued robot acceleration contributes a separate correction. It does not treat shaft torque divided by lead as contact force under every accelerating condition.

The representative motor is a 24 V **maxon RE25 302005**, with Kt≈0.0234 N·m/A, R≈2.32 Ω, rotor inertia 1.08×10⁻⁶ kg·m² and continuous torque approximately 26.3 mN·m. Adding an assumed 0.5×10⁻⁶ kg·m² screw inertia gives **0.624 kg reflected inertia at 10 mm/revolution**. The same assembly gives 2.50 kg at 5 mm/revolution and 6.93 kg at 3 mm/revolution. A smaller lead increases force per ampere but substantially increases the fast inertial load. [Manufacturer motor data](https://www.maxongroup.com/maxon/view/product/302005).

![Screw lead, reflected inertia and continuous holding current](figures/screw_pitch_inertia_and_heating.svg)

*Analytical sizing for the reference motor and assumed screw inertia. The 90% efficiency curve is a conservative holding calculation, not an efficiency model used to claim a dynamic pass. Motor torque and temperature also limit continuous operation.*

At 10 mm/revolution, an ideal 50 N horizontal hold requires **3.40 A and 26.8 W winding heat**, far above the motor's 1.16 A continuous-current rating. Its ideal continuous axial force is only about 16.5 N before transmission losses. Reducing lead to 3 mm/revolution approaches the continuous-current limit, but 90% efficiency leaves only about 49.6 N from the stated torque rating, before friction, while adding nearly 7 kg of reflected inertia. This particular motor is not suitable for the whole requested envelope even though the peak simulations pass.

A larger servo and low-backlash screw could be a credible alternative. At 10 mm/revolution and 90% efficiency, 50 N alone requires roughly **0.088 N·m continuous shaft torque**. A practical design needs additional continuous and transient reserve, low total rotor/screw inertia, an encoder/commutation solution, measured stiction and a carrier inverse calibrated to the actual mechanism. The larger maxon EC-i30 539489 is a static sizing reference with approximately 103 mN·m continuous torque; it is not dynamically qualified here. [Manufacturer motor data](https://www.maxongroup.com/maxon/view/product/539489).

Pneumatic or other servo actuators could also work with their own identified dynamics. This study does not qualify their pressure dynamics, valve delay, seal friction or backlash. For the first precise-force prototype, the voice coil provides the most direct implementation of the tested current-to-force controller. The screw results show that the control/mechanical concept is transferable; they do not show that every screw actuator or motor will satisfy it.

##### Voltage headroom, current limits and continuous heat

The retained lower-series electrical study contains **92 distinct configurations from 95 durable native runs**, including explicit parity/reference repeats. **80 pass and 12 fail**. Higher generic bus voltage at the configured 100 µs current response leaves the unclipped recordings unchanged, including the native force/current histories. Increasing the peak-current limit from 3 A to 5 A likewise makes no difference where the largest current request remains about 2.99 A. Additional supply or current capacity helps only when it removes a real limit or supports a separately qualified faster current regulator.

Force-constant uncertainty is tested separately. Unknown ±5% changes pass the sampled cases, with approximately 0.428 N worst error. Unknown ±10% causes startup-transient failures up to about 0.927 N; after 50 ms those traces are within approximately 0.348 N. They still fail the report's full-recording requirement. Increasing voltage does not correct that calibration problem. Establish the operating holding current and force constant before starting the qualified disturbance test, and map any meaningful position/temperature dependence.

The real Moticont winding is more demanding electrically than the generic 4 Ω/3 mH example. All sampled ideal 24, 36 and 48 V cold/hot cases pass with the matched controller gains and inverse force constant. However, at an assumed 100°C resistance rise, its **requested voltage reaches about 27 V**. Ideal 24 V clips briefly; the contact-force effect in those sampled recordings is small. An effective **19–20 V output** from a nominal 24 V linear amplifier fails the four tested hot cases, with worst errors about **5.71 N at 19 V and 2.81 N at 20 V**. The 4–5 V amplifier-loss envelope is an explicit conservative assumption, rather than a measured dropout specification at 24 V.

The final acquisition candidate receives an initial **16-run electrical check over 15 distinct conditions**, extended by two native records below to **18 runs / 17 distinct conditions**. Its generic 24/48 V references are bitwise equal across all 72 channels. The real cold/hot coil at ideal 36 V passes, and the hot coil with **31 V usable output** is bitwise equal to its ideal 36 V reference for both seeds, without clipping. The actual hot-coil request reaches **28.473 V**, leaving **2.527 V** at that conservative 31 V output envelope. Hot 19 and 20 V outputs fail all four sampled low-headroom conditions, with worst errors **5.716 and 2.882 N**; contact remains closed and each recording completes. Their pre-bump error remains at most 0.151 N, locating the failure in dynamic voltage headroom rather than startup.

This subset supports 31 V usable bipolar output for the representative real coil; the broader calibrated-mechanics check below raises the selected output requirement to **32 V**. The 4–5 V stage-loss envelope remains an explicit conservative allowance, not a verified TA115 dropout specification. A 48 V supply does not improve the unclipped response at unchanged tuning and increases a linear driver's heat. Voltage demand includes noisy current regulation as well as static IR and back EMF; checking only the 18.23 V hot holding drop would miss the transient requirement.

![Real-coil usable voltage, tracking and steady holding heat](figures/inertial_final_voltage_headroom.svg)

*16 completed native recordings, including one duplicate software parity reference. The heat panel is a separate steady-load calculation from the stated winding resistance and supply, not a thermal simulation.*

The hot-winding headroom check also covers **all 3,500 nominal grid identities**, using **3,498 sufficient conservative voltage bounds and two directly executed exception identities**. With known Kf=24.4 N/A, Rhot=8.896 Ω, L=4 mH and unchanged 100 µs current response, scale current-domain control by 20/24.4. The generic native regulator permits the exact mapping `Vhot = 1.22 Vgeneric + 2.411803 I − 3.813115 (Icommand − I)`. A triangle bound from each recorded peak proves 31 V sufficient for 3,498 cases. Two conservative bounds exceed it because their separate peaks need not occur together. Their actual hot31V native runs request at most **28.473 V** and pass without clipping; the additional generic reference matches its archived grid identity exactly.

This closes the conditional finite-grid electrical screen with two extra native runs, rather than claiming 3,500 real-winding simulations. It depends on the identified constant force constant, matched hot R/L compensation, upper-rigid kinematics, normalized controller and the recorded nominal mechanics/sensors. It does not qualify unknown hot force constant, thermal behavior, the full joint-error budget or continuous force over stroke. The complete derivation and exact parameter/waveform audits are in [the headroom evidence](/home/tommie/Projects/actuator-model/acf_simulink/validation/normal_voltage_final_grid_hot_mapping_20260930.md).

A further **pointwise algebraic hot-winding check** uses all native samples of the four tighter-static 30 s histories and three tighter-static known-k/c mixed histories. It preserves their residual Kf identification while uniformly rescaling all current-domain control by 20/24.4. The long cases request at most **29.5915 V**. One mixed case requests **31.1071 V for one 10 µs sample**, so a 31 V envelope does not reproduce every inspected trajectory without clipping. This is not an observed force failure. **32 V usable terminal output** covers all seven checked histories with at least **0.8929 V** reserve. The mapping adds zero real-coil simulations and retains constant identified winding/current-response assumptions.

Therefore specify a **36 V supply with measured usable bipolar output of at least 32 V** if the selected driver can provide it under hot load. Do not assume a 5 V stage loss and still call that envelope sufficient. If the actual amplifier cannot provide the required terminal output, choose a suitable lower-drop driver, revise the winding or qualify a higher supply with its thermal consequences. Increasing voltage beyond the verified unclipped envelope does not improve the selected response.

At 50 N and 90°, the real coil holds about **2.05 A**, dissipating **26.9 W cold or 37.4 W at the assumed hot resistance**. Representative RMS current is almost the same as the hold current: holding load dominates coil heat. An ideal linear stage at 36 V then dissipates approximately **46.9 W cold or 36.4 W hot**, independently of cabinet/control electronics. The TA115-class amplifier requires an adequate heatsink/airflow and safe-operating-area check at its actual case temperature. Its published 50 W allowance at a 70°C heatsink leaves little reserve above the calculated 46.9 W cold holding loss. Keep the heatsink cooler or select a stage with adequate operating margin; the generic 20 N/A, 4 Ω winding would require 65 W driver dissipation at 36 V. The Moticont drawing states its winding values at 25°C and inductance at 1 kHz; hot resistance, frequency-dependent inductance, force-constant drift and the published continuous-force curve must be considered together. [Driver datasheet](https://trustautomation.com/datasheets/TA115DataSheet.pdf), [coil drawing](https://www.moticont.com/pdf/lvcm-070-051-01.pdf).

##### New current-response and voltage experiments

The unified-controller study adds a matched drive experiment using one fixed, earlier recurrent checkpoint. At the development worst case, **32, 36 and 48 V usable output give the same 0.418728 N learned peak and 0.428618 N baseline peak with the 100 µs current-response tuning**. Raising voltage removes a few late clipping samples but does not change the worst bump-entry error. Current-loop response and supply voltage are separate design choices.

Changing the modeled and identified response from **100 to 50 µs** gives a clearer benefit. Across the same **28 development conditions**, both controllers pass at both response settings: **112 complete native recordings and 111,600,112 native samples**. The baseline worst peak improves from **0.428618 to 0.397116 N**; the earlier learned checkpoint improves from **0.418728 to 0.373499 N**. Its peak improves in 26 of 28 pairs; the largest of two regressions is 0.003485 N. Brief voltage-clipping samples increase from 104 to 27,176 across its 28 recordings at 32 V, while no current-limit sample occurs. Every recording still passes the force requirement.

![Measured development benefit of faster current response](figures/current_response_28cases.svg)

*Matched development cases and fixed earlier checkpoint, with every native sample scored. These are additional response tests, not final-winner qualification at 50 µs. The primary learned-policy results below use 100 µs and 32 V.*

![Voltage and current-loop response in the same development case](figures/actuator_limit_peaks.svg)

*Twelve complete recordings separate voltage-only changes from faster current tuning. More voltage removes clipping; changing current response reduces the entry transient. The faster 32 V learned waveform was independently replayed in MATLAB.*

This supports testing a **50 µs current-response option** on the actual hot coil and driver. It does not support buying a higher-voltage supply as a substitute for measuring loop phase and bandwidth. Confirm stability, ripple, usable voltage and heat before promoting the faster tuning; the tested current-response envelope is an assumption about a regulator, not a demonstrated amplifier specification.

#### Recommendation

Use a **guided voice coil** for the first prototype: approximately **20–25 N/A**, **25 mm travel**, and preferably **65–70 N or more continuous force over the used stroke and operating temperature**. The evaluated winding range is roughly **4–6.4 Ω cold resistance** and **3–4 mH inductance**. Provide bipolar current, at least the modeled **3.5 A peak provision**, and a verified **100 µs current-response target**.

Start with a **36 V supply only if the driver provides at least 32 V usable bipolar output under hot load**. The latest seven M08 hot-coil tests request up to 31.738466 V, so actual dropout needs measurement. Size cooling for continuous holding effort, not just the bump. A suitably larger low-backlash screw remains an alternative, but the tested RE25 is thermally undersized; its transient passes do not qualify continuous 50 N sanding.

### Force and position measurement

#### Intro

The B–S cell carries the tool’s downstream mass, so its reading contains support and inertia as well as contact loading. Relative position helps estimate motion and surface direction, while multi-axis force distinguishes pressing from sanding drag.

#### Question

Where should the sensors be mounted, and how can their readings reconstruct the force the workpiece experiences rather than only the force inside the flange?

#### Results

##### Reaction-vector reconstruction and the surface normal

The cell is between B and S, so it carries the entire downstream tool. Its reading includes the forces needed to support and accelerate S. Static tare can remove known gravity; it cannot remove changing acceleration.

![Reconstructing normal pressing force from synchronized measurements](figures/acf_force_reconstruction.svg)

*Force, acceleration and estimated normal must refer to the same effective time and frame. The observer uses measured motion and robot commands already issued.*

An accelerometer measures **specific acceleration**, `f = aS − g_vector`. At rest it reports the support acceleration opposing gravity. With the cell sign defined as positive workpiece reaction, Newton's law gives the exact translational relationship:

```text
R = Lraw + mS f
```

With consistent static gravity tare on both channels, the same reconstruction becomes:

```text
Rhat = Ltared + mS adynamic_measured
Nhat = nhat · Rhat
```

This explains why adding an accelerometer is useful. The contact-force estimate comes from two physical measurements and the known downstream mass; it does not need to infer acceleration from the coil's force constant or pretend that the B–S cell directly measures contact force. Taring the cell and then adding the accelerometer's unmodified gravity reading would double-count gravity. Both channels use one consistent convention.

Force and acceleration must describe the same instant and frame. If the cell transfer is `HL` and the acceleration transfer is `Ha`, the reconstructed vector is:

```text
Rhat = HL R + mS (Ha − HL) f
```

Matched complete filtering and delay cancel the inertia term; equal sample rates alone do not. At S=2 kg and jerk 384 m/s³, 100 µs differential latency can introduce about **0.0768 N** of inertia error. Matching the faster channel to an identified common response is usually preferable to stacking both sensor filters and adding unnecessary delay.

The normal-direction estimate also needs the measurement's effective timestamp. A delayed reaction vector contains delayed sanding drag; projecting it onto a present normal can produce an apparent normal-force error. The controller can keep a matched direction for measurement reconstruction and a separate causal prediction for its actuator command.

The controller estimates local normal direction from tool motion without receiving the bump geometry. Relative S–A position constrains gap and velocity, measured S acceleration supplies the fast motion information, and the issued-command robot predictor supplies the expected A motion. A small position-corrected velocity observer provides the estimate; dividing normal tool velocity by tangential scan velocity gives local slope under the model's maintained-contact assumption. The known actual surface trajectory remains a plant input and a validation reference only.

##### Component references, range and calibrated accuracy

These parts ground the engineering assumptions. They are representative prototype choices rather than a purchasing list or a claim that catalog values prove the assembled force accuracy.

| Component class | Representative example | Useful published properties | What still needs verification |
| --- | --- | --- | --- |
| Compact multi-axis cell | ATI Mini27 Titanium SI-80-4 | ±80 N transverse, ±160 N axial; effective resolution 0.03/0.06 N; 33.4 g; calculated axial stiffness 64 MN/m | Combined-load residual calibration, temperature drift, loaded mechanical response and complete channel timing |
| Higher-stiffness cell | ATI Nano25 SI-125-3 | ±125/±500 N; effective resolution about 0.021/0.0625 N; axial stiffness 110 MN/m | Broad axial range does not itself improve accuracy; calibration and mounted response remain necessary |
| S–A position | Protected Renishaw TONiC with RELM scale | 0.1 µm counter interface is available; certified scale accuracy approximately ±1 µm at 20°C | Bracket deflection, dust protection, magnetic reference/limit behavior, encoder latency and lost-count detection |
| Low-noise current drive | Trust Automation TA115 class | 24–48 V, current mode, published 5 kHz bandwidth into 2.5 mH, forced-air cooling | Actual 3–4 mH hot-coil response, output headroom, residual current noise and thermal safe operating area |

Sources: [ATI Mini27](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Mini27+Titanium), [ATI Nano25](https://www.ati-ia.com/products/ft/ft_models.aspx?id=Nano25), [TONiC/RELM documentation](https://www.renishaw.com/media/pdf/en/fa9b5d5f3056478c8031f7c3e8091102.pdf), [TA115 datasheet](https://trustautomation.com/datasheets/TA115DataSheet.pdf).

The load cell must accommodate **raw force before gravity tare**. A 2 kg tool contributes up to 19.6 N laterally at a 90° orientation; sanding drag and sloped contact add to that load. A tare changes the reported zero, not hardware capacity. Moments and combined-axis ratings also need checking for the real tool's lever arm.

Published force resolution is not calibrated accuracy. Current Mini27 factory accuracy is stated as 0.75–1.25% of full scale, depending on axis: that does not establish a 0.05–0.1 N residual. The assembled system needs independent known-load checks after any offset, gain, axis-matrix and temperature corrections. Sensor calibration limits belong beside the control results, rather than being hidden inside a low-noise assumption. [Current manufacturer specification](https://ati.novanta.com/product/mini27-titanium-force-torque-sensor/).

#### Recommendation

Use a **multi-axis load cell in the complete B–S load path**, with all S mass beyond it. Measure **S relative to A** with a protected position sensor and synchronized timestamp. A load cell needs capacity for raw gravity, sanding drag and moments before tare.

Reconstruct the reaction vector from the consistently tared load and measured S inertia, then project onto the estimated surface normal at the same effective time. Target **±0.01 N residual force zero per used axis**, **±0.05% force gain/cross-axis residuals**, **±0.05° frame alignment**, approximately **1 µm reference-conditioned position noise**, and **±0.05% position-scale residual**. These are assembled post-calibration targets; catalog resolution alone does not establish them.

### Acceleration measurement

#### Intro

Acceleration measured directly on S supplies the dynamic-inertia correction. Lower sensor cost is useful only if noise, complete response, bias and synchronization retain the required force margin.

#### Question

Can a cheaper accelerometer replace the fast reference sensor without materially reducing force margin, and what resolution, noise, bias and timing must the complete acceleration chain achieve?

#### Results

##### Reference sensor, specific acceleration and acquisition budgets

The analog **ADXL1002** is the faster acquisition reference: ±50 g and 25 µg/√Hz, with sufficient bandwidth to characterize the load-cell mode. It is single-axis; use two orthogonal units for the present scanning plane and three for a general spatial system. External antialias filtering, offset/gain calibration and matched timing are necessary. Its 11 kHz response specification is not an 11 kHz first-order low-pass corner. [Manufacturer datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl1001-1002.pdf).

The original scientific reference uses a simultaneous-sampling **AD7606C-18**, which provides eight 18-bit analog channels. Budget about 0.65 µs conversion provision without oversampling, then include its input filter, readout and computation in the complete timing measurement. A force interface exposing six amplified and conditioned bridge channels plus the two acceleration axes fits one converter for the planar model; the ATI calibration matrix converts those bridge values into force and torque. A full XYZ accelerometer arrangement requires nine channels: use two common-trigger converters, characterize inter-device timing, and latch the position encoder at the same epoch. [ADC datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606c-18.pdf), [ATI TWE interface manual](https://www.ati-ia.com/app_content/documents/9620-05-TWE.pdf).

The ADXL1002's typical response is within 5% out to 4.7 kHz, while its mechanical resonance is around 21 kHz. This makes it a useful reference for a faster acquisition design around the cell's kilohertz modes. An illustrative 10 kΩ/10 nF output pole gives approximately 100 µs, consistent with the datasheet's capacitive-load provisions. At that dominant response, theoretical white acceleration noise is approximately **0.01226 m/s² RMS**. A 40 mV/g sensitivity and the ADC's 0–5 V range give a **0.00468 m/s² quantum**; including ideal quantization gives about 0.01234 m/s² RMS, or **0.0247 N RMS at S=2 kg**.

That calculation is a starting budget. A single 100 µs RC sampled directly at 10 kHz is not a complete antialias design. Acquire faster, provide adequate out-of-band analog rejection, then decimate to the controller rate; count the complete filter and decimator delay. ATI's DAQ interface includes a published two-pole 5.1 kHz analog filter; a TWE bridge interface needs designed conditioning and filtering instead. Match the installed complex force/acceleration responses over the relevant band, rather than treating either sensor as an ideal first-order part. The AD7606C-18 also contributes input-filter phase: its low-bandwidth mode specifies a typical 25 kHz bandwidth and about 6.8 µs phase delay. The stated acceleration quantum is ideal resolution, rather than the ADC's complete noise or calibrated accuracy.

The scientific-reference acquisition uses a **100 ksample/s simultaneous raw clock**, matched two-pole approximately 5.1 kHz analog responses, a **50 µs EMA at that raw clock**, and a fresh-sample latch into the **10 kHz force controller**. The native nominal low-frequency delay of the filter pair is **88.92 µs**, before the selected 20 µs acquisition/computation allowance. The accelerometer white-noise budget is approximately **0.014 m/s² RMS** after nominal conditioning, before quantization and final sampling. The ideal acceleration quantum adds about 0.00135 m/s² RMS quantization noise. The 0.014 m/s² conditioned value is a measurement-chain acceptance target, rather than a guaranteed assembled specification: ADC, amplifier and reference noise consume that allowance and must be measured. Exact ATI filter damping, installed sensor response and total timing still need identification; a nominal Butterworth response is an engineering envelope, not a claim about an unspecified manufacturer filter shape. This acquisition graph is used by the scientific-reference qualification below. The dated lower-cost study later in this report separately evaluates replacement converters with their own filters, quantization, noise and timing.

The three-axis **ADXL354C** remains a useful lower-bandwidth comparison. Its ±8 g noise budget is about 0.0117 m/s² RMS with a 200 µs dominant response, but its internal response around the cell's kilohertz mode needs particular care. It cannot be substituted for a faster sensor by copying only the RMS-noise value. [Manufacturer datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf).

The accelerometer must measure S's rigid-body acceleration near its centre of mass, or rotation and sensor lever arm must be compensated. Mounting on a flexible circuit-board corner can measure board vibration instead. Before selecting range, run the real sander on an instrumented fixture and check clipping, rectified bias and temperature drift; low white-noise density alone does not establish accurate DC force during sanding.

The ADXL1002 output zero and sensitivity are ratiometric to its supply. The outer ±0.01 m/s² bias stress corresponds to **40.8 µV** at its nominal 40 mV/g output; the selected **±0.005 m/s² target is only 20.4 µV**. With an independent ADC reference, approximately **40.8 µV of uncompensated supply change**, or **8 ppm at 5 V**, uses the tighter zero-error allowance. Use a stable, calibrated sensor/reference ratio or measured supply correction, warm up and tare at operating temperature, and verify drift through a powered sanding run. Initial tare alone does not establish that performance over time. [Accelerometer datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl1001-1002.pdf).

The identified downstream mass includes everything on S's side of the sensing element: tool, pad, brackets, sensor carrier, cable portion and fasteners that move with it. The assembly's centre of mass and any rotation must be measured consistently with the force/acceleration frames. The actuator's stationary body still moves with the robot; its mass is robot payload even when it does not contribute to the fast B+S inertia.

##### Executed lower-cost response and calibration experiments

Two reference single-axis ADXL1002s cost approximately **€129.76** at the observed quantity-one prices. One three-axis **ADXL354C is about €60.44**, while an **ICM-42688-P is about €5.33** and **IIS3DWB about €19.90**. Those are sensor IC comparisons, not complete board costs. The analog ADXL354 still uses the external ADC; the digital candidates have their own conversion. [Component and price review](/home/tommie/Projects/actuator-model/acf_simulink/validation/accelerometer_cost_component_review_20261002.md).

ADXL354C's ±8 g setting has an approximately 33.75 µg/√Hz noise-density target and 100 mV/g sensitivity. A centered buffered input into the M08 gives roughly **0.0000140 m/s² per ADC code** and **0.00209 m/s² typical ADC noise**. Its higher sensor noise is larger than the ADC contribution. That resolution is sufficient for investigating the force objective; it does not establish bias, scale, temperature drift or vibration rectification.

The **published 1.9 kHz overall 3 dB bandwidth is not a complete transfer function**. The ADXL354 has an internal sinc-shaped response and a 2.4 kHz mechanical mode. Current manufacturer documentation does not supply the full coefficients or phase. The following simulations therefore test clearly identified **required matched response envelopes** with the part's noise and range; they do not claim that the part implements a two-pole Butterworth filter. [ADI datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf), [response review](/home/tommie/Projects/actuator-model/acf_simulink/validation/budget_accelerometer_response_review_20261002.md).

![Cheaper accelerometer response and noise experiments](figures/budget_accelerometer_envelopes.svg)

*36 completed native nominal recordings. Analog cases cover both physical endpoints and seeds 7/29; the digital envelope cases cover the heavy endpoint. Counterfactual 3/5.1 kHz ADXL354 bandwidths are retained in the source but excluded from this practical comparison. Extra common delay is known to the estimator and matched across measurements.*

At the existing 100 µs current target, the ADXL354-like 1.9 kHz envelope gives **0.405691 N** worst error with no additional common latency. An extra 50 µs raises it to **0.457041 N**; 100 µs raises it to **0.484946 N**. Noise alone is therefore not the deciding obstacle. These delay increments are sensitivity cases; published bandwidth cannot prove that they bracket the real part's phase.

A separately tested **50 µs current target at ideal 36 V** improves the same envelope to **0.378082 N**, or **0.429852 N** with the extra 50 µs. This changes current-loop bandwidth as well as drive provision. It is not evidence that increasing voltage alone improves an unclipped 100 µs loop. The joint calibration and real-hot-winding tests are needed before choosing that trade.

The low-cost digital envelope at 70 µg/√Hz and an additional common 80 µs gives **0.434807 N** in its nominal heavy endpoint. With 120 µs it reaches **0.477704 N**. The ICM's own documented decimation, selectable filters, clock and DRDY/readout timestamps still need to establish the required complete response. Its published delay combines configured filters and clocks; it cannot be replaced by an arbitrary RC pole. The native digital cases use a labeled common response/latency envelope, not an implementation of the ICM's undocumented complete transfer. Calling its output rate its sensing bandwidth would be incorrect.

The stock ADXL354 evaluation board also includes 0.1 µF output capacitance. With the documented 32 kΩ output impedance, that adds a **3.2 ms RC**, about 49.7 Hz. Such a configuration is far slower than the studied envelope. A fast prototype needs intentionally selected output capacitance and buffering. [ADI evaluation guide](https://www.analog.com/media/en/technical-documentation/user-guides/eval-adxl354-355-ug-1030.pdf).

The decisive followup preserves **four 30 s calibration-error recordings and three four-second known-k/c mixed-mechanics recordings** for each of seven sensing/drive combinations. The controller uses the same mechanics and causal measurements. Actual and identified sensor/mass/robot residuals stay separate.

| Required acceleration-response envelope and drive | Passed / executed | Worst error | Reserve to 0.5 N |
| --- | ---: | ---: | ---: |
| Reference density, matched 8 kHz, 100 µs current, scientific 24 V coil | 7 / 7 | 0.438821 N | 0.061179 N |
| ADXL354-like 1.9 kHz, no extra delay, 100 µs current, scientific 24 V coil | 5 / 7 | 0.516806 N | −0.016806 N |
| ADXL354-like 1.9 kHz, no extra delay, 50 µs current, ideal 36 V coil | 7 / 7 | 0.482489 N | 0.017511 N |
| ADXL354-like 1.9 kHz, extra 50 µs, 50 µs current, ideal 36 V coil | 4 / 7 | 0.530965 N | −0.030965 N |
| Digital 70 µg/√Hz, matched 5.1 kHz + 80 µs, 100 µs current | 3 / 7 | 0.581260 N | −0.081260 N |
| Digital 70 µg/√Hz, matched 5.1 kHz + 80 µs, 50 µs current | 3 / 7 | 0.559963 N | −0.059963 N |
| ADXL354-like 1.9 kHz, no extra delay, 50 µs current, hot real-coil 32 V output | 7 / 7 | 0.482489 N | 0.017511 N |

![Cheaper sensing under calibration and drive stress](figures/budget_sensor_calibration_stress.svg)

*49 executed native recordings; failures are retained. Each row is a declared sensor response envelope, not a supplier-certified accelerometer transfer. The 0.45 N design threshold retains 0.05 N reserve.*

The faster ADXL354-like case passes these seven inputs, but its remaining reserve is less than one third of the reference envelope's. Its hot-coil variant uses Kf=24.4 N/A, fixed R=8.896 Ω, L=4 mH and 32 V usable terminal output; brief voltage clipping occurs, and the recorded worst error stays essentially the same. This is a real-winding native simulation with a **deliberately faster 50 µs current target**, not a claim that voltage alone fixes sensing delay. Adding 50 µs common sensing latency still produces failures. The digital response envelopes fail four of seven cases even with the faster current loop.

A final **seven-case tighter dynamic-matching experiment** reduces analog-response and EMA residuals from ±10% to ±1% and relative channel offsets from ±10 µs to ±2 µs, while preserving static calibration, common pipeline uncertainty, robot timing, mechanics, noise and drive. Common and per-channel response factors both remain, so their combined maximum is approximately 2.01%. The no-extra-delay ADXL354-like envelope passes at **0.489946 N worst**, only **0.010054 N reserve**. Tighter matching did not recover the desired margin; the peak need not improve monotonically because it combines deterministic error and seeded noise. Seven extra-delay matching inputs were prepared but **not executed**, and are excluded from the experiment count.

Therefore a cheap accelerometer is **not yet supported as a low-loss replacement**. ADXL354C is an attractive analog cost target for a later measured-transfer study, but the executed envelope results do not justify recommending it for this force specification. The digital part's large chip saving does not compensate for the observed failures. Keep two ADXL1002-class analog axes in this planar prototype; use three axes and sufficient simultaneous ADC channels for spatial reconstruction. Their own resonance and assembled phase still require measurement.

##### First-prototype chip budget

The new first-prototype target is **under €100 for accelerometer and ADC chips combined**, on our own PCB. One three-axis ADXL354C with one ADS131M08 is the budget candidate. The planar prototype acquires two acceleration axes alongside six bridge channels; a digital position interface is separate. Full three-axis acquisition needs a second synchronized ADC if all six bridge channels are retained.

This lower-cost accelerometer changes the controller's phase and noise problem. The current reward-training run therefore uses its declared ±8 g / 33.75 µg/√Hz / 100 mV/g envelope with a matched 1.9 kHz response and faster 50 µs current regulation. The completed older tests above explain the difficulty; fresh recurrent-control tests are pending. The part's published bandwidth still does not specify its complete installed phase response.

#### Recommendation

Evaluate the ADXL354C plus ADS131M08 as the first-prototype chip-budget candidate. Complete its own frozen-controller tests and measure the assembled transfer, timestamps, warm bias and powered-tool clipping before claiming the force specification. The ADXL1002 results remain a faster-sensor reference.

### Acquisition electronics: amplifier, ADC and timing

#### Intro

The instrumentation amplifier sets the useful range before conversion. ADC noise, filter phase, clocking, readout and controller latching determine how accurately and promptly force and acceleration can be combined.

#### Question

What is the lowest-cost acquisition chain supported by native experiments, and why are its bandwidth and sample rate necessary rather than simply choosing a large bit count?

#### Results

##### Converters, input scaling and signal path

The ADC's bit count does not determine its useful force resolution by itself. An instrumentation amplifier maps the load cell's useful range into the converter's voltage range. The important quantities are input-referred noise, quantization after that gain, matched channel response, and the age of the sample when the controller uses it.

Indicative single-IC prices retrieved on 2 October 2026 are **€63.86 for AD7606C-18**, **€19.13 for ADS8588S**, and **€9.25 for ADS131M08**, excluding VAT and shipping. These are documented regional price observations; they are not checked shopping carts. The ADC-only savings are approximately 70% and 86%. Instrumentation amplifiers, references, filtering and calibration remain part of the electronics. [Price/specification review](/home/tommie/Projects/actuator-model/acf_simulink/validation/adc_cost_component_review_20261002.md).

| Converter | Tested practical setting | Force quantum at effective ±160 N | ADC acceleration quantum with reference analog conditioning | Intrinsic ADC response |
| --- | --- | ---: | ---: | --- |
| ADS8588S, 16-bit simultaneous SAR | 50 ksample/s per channel, ±5 V | 0.004883 N | 0.018705 m/s², ADXL1002 centered gain 2 | Third-order 16 kHz analog filter, about 19.9 µs DC delay |
| ADS131M08, 24-bit simultaneous delta-sigma | 30 ksample/s per channel, PGA1 | 0.0000191 N | 0.0000701 m/s², ADXL1002 gain 0.5 | Sinc³, OSR128 at 3.84 MHz modulator, 49.6 µs group delay |

Acceleration resolution is the step between adjacent codes; it is not the RMS noise or calibration accuracy. The SNR-equivalent ADC acceleration noise in these two configurations is approximately **0.01236 m/s²** for the SAR's datasheet-minimum-SNR-derived model noise and **0.01045 m/s²** for the delta-sigma typical OSR128 noise. The SAR minimum is specified at 200 ksample/s; it is a modeling provision rather than a verified noise bound at the tested 50 ksample/s. The M08 figure is typical, rather than a guaranteed maximum at the selected clock. The experiment injects ADC noise separately from physical sensor noise and avoids counting ideal quantization variance twice. Separate M08 probes double its typical noise and add 200 µs output-noise correlation. Measure both converters' assembled noise at the chosen rate. [TI SAR datasheet](https://www.ti.com/lit/ds/symlink/ads8588s.pdf), [TI delta-sigma datasheet](https://www.ti.com/lit/ds/symlink/ads131m08.pdf).

The M08 uses a **7.68 MHz master clock**, a 3.84 MHz modulator and OSR128 to produce 30 ksample/s. Reading status, eight channels and CRC requires 240 SPI bits: **15.625 µs at 15.36 MHz SPI**. The modeled **20 µs total post-conversion pipeline includes that readout**, leaving about 4.375 µs for the MCU path; it is not another 20 µs added to readout. The SAR's worst conversion plus a 128-bit read at 16 MHz is about 11.9 µs within the same total allowance. Verify those budgets on the chosen MCU.

With an analog accelerometer, **yes, the external ADC measures acceleration as well as force**. Two acceleration axes plus six raw bridge channels fit an eight-channel ADC in the current planar arrangement. A digital accelerometer uses its own internal ADC and a timestamped SPI interface; it does not consume an external ADC channel. A spatial arrangement with six bridges and three analog acceleration axes needs nine channels. Position has its own encoder interface. The bridge calibration matrix must establish the effective force-axis noise and range; raw channel specifications alone do not establish them.

![Lower-cost acquisition signal path and timing budget](figures/acf_acquisition_timing.svg)

*Selected M08 signal path and nominal low-frequency component delays. The full response includes frequency-dependent phase, sample age and controller latching; the 20 µs pipeline already includes SPI readout.*

##### Bandwidth, rate, timing, noise and hot-coil experiments

The new private native Simulink model includes a continuous instrumentation response **before sampling**, the SAR's analog response or a genuine Sinc³ decimator, converter noise, range/quantization, an EMA with a 50 µs time-constant setting at the converter-output clock, the readout pipeline and fresh 10 kHz controller latching. Physical sensor noise density stays fixed when sample rate changes. Qualification uses every physical normal-force sample, including startup; plots are reduced only afterward. The force and acceleration response envelopes are matched, and position has a separate encoder with an aligned measurement frame.

The screen executes **168 recordings**, with two heavy/light physical endpoints and clean/seed-7 noise variants at each setting. The instrumented physical plant, estimator and controller are frozen. A separate nine-recording audit checks warm/cold compilation parity across seed, mass/force/orientation and response changes; all physical and ADC samples in the tested pairs match exactly.

![Native ADC bandwidth and rate sweep](figures/budget_adc_bandwidth_rate.svg)

*The numbers are worst complete-recording errors, in N, across four recordings per cell. Values above 0.500 N fail. The 0.450 N design threshold reserves 0.050 N for the physical requirement. This is a finite endpoint screen, not a continuous bandwidth proof.*

The low-bandwidth failures are substantial. With a 500 Hz matched response, the best rate still gives roughly 0.608 N error for the SAR and 0.672 N for the delta-sigma chain. A 1 kHz response is only a nominal pass at some settings. The familiar 10 Hz bump transit does not capture the controller's need to recover downstream inertia and react at bump entry/exit; the executed waveform resolves the faster dynamics.

Rate matters even with the same analog cutoff. At 8 kHz response, the delta-sigma group delay increases from about **49.6 µs at 30 ksample/s**, to **99.6 µs at 15 ksample/s**, to **199.6 µs at 7.5 ksample/s**. Lower rates also age the samples more before each controller latch. The controller rate remains 10 kHz in all these comparisons.

The four retained **30 s repeated-bump calibration cases** preserve the selected tighter static residuals, channel-response mismatches and robot timing errors. They expose the trade more sharply:

| Converter and setting | Completed / passed | Worst normal-force error | Remaining reserve to 0.5 N |
| --- | ---: | ---: | ---: |
| ADS131M08, 15 ksample/s, 8 kHz matched response | 4 / 4 | 0.494316 N | 0.005684 N |
| ADS131M08, 30 ksample/s, 5.1 kHz matched response | 4 / 4 | 0.451330 N | 0.048670 N |
| ADS131M08, 30 ksample/s, 8 kHz matched response | 4 / 4 | 0.438821 N | 0.061179 N |
| ADS8588S, 50 ksample/s, 5.1 kHz matched response | 4 / 4 | 0.404214 N | 0.095786 N |

Thus 15 ksample/s is a numerical pass in these four cases, but does not retain a useful design margin. Keeping **30 ksample/s** has an experimental justification. The selected **8 kHz** response retains over 0.05 N reserve in these cases; 5.1 kHz narrowly misses that preferred reserve. This establishes a tested choice, rather than proving that 8 kHz is the minimum possible cutoff. The value describes a complete engineered response target: include the accelerometer's own amplitude/phase rather than assigning it an arbitrary filter corner.

For orientation, a nominal two-pole 8 kHz response contributes **28.1 µs low-frequency delay**, Sinc³ adds **49.6 µs**, and the sampled EMA contributes **35.2 µs** at 30 ksample/s. Including the nominal pipeline gives about **132.9 µs before sample-age and controller-latch effects**. These are component delay calculations, not a single frequency-independent phase model. Native execution retains the full filters and clocked behavior. At 15 ksample/s, the decimator alone doubles its delay despite the unchanged 8 kHz analog response; the measured force-error increase above verifies that timing matters.

![Native long-test force waveforms](figures/budget_adc_long_waveforms.svg)

*Each curve shows the bump containing the full-recording worst sample in that setting's worst 30 s recording. The cases retain calibration error and seeded noise. Dashed lines are the force requirement; dotted lines reserve 0.05 N.*

The selected M08 chain also completes a **700-case nominal grid**: five S masses, ten force targets, seven orientations, two physical drag ratios and **one noise seed, 29, per condition**. All 700 unique cases pass; worst complete-history error is **0.292803986 N**, leaving **0.207196014 N** reserve. Every one of the 1,200,001 native force samples in each four-second recording is scored. This is 700 new noise realizations, not the historical five-seed 3,500-case qualification. Preparation hashes, condition identities and disjoint partition coverage are recorded in [the grid audit](/home/tommie/Projects/actuator-model/acf_simulink/validation/budget_adc_range_grid_20261002.json).

![ADC noise and timing robustness probes](figures/budget_sensing_response_noise.svg)

*Two heavy-tool recordings per probe, seeds 7/29. Each changes one acquisition property. These nominal probes are separate from the simultaneous calibration-error cases.*

Doubling the ADC's typical noise gives **0.339688 N** worst error; introducing a 200 µs correlation time gives **0.332057 N**. An extra acceleration-only skew of 20 µs gives **0.358505 N**, and 50 µs gives **0.381665 N**. The tested nominal cases pass, but timing consumes appreciably more margin than doubling converter noise. Keep the specified common clock and tested relative timestamp residuals rather than treating those probes as permission for arbitrary skew. Halving the physical solver step with unchanged acquisition clocks changes the paired peak by only **0.000000304 N**. Six additional quiet sensor-tone probes at 19, 49 and 99 kHz are retained separately; they do not constitute a measured sanding-vibration spectrum.

The selected **30 ksample/s / 8 kHz chain is also executed with the hot real-winding parameters**: Kf=24.4 N/A, R=8.896 Ω, L=4 mH, 32 V usable bipolar output and the unchanged **100 µs** current target. Current-domain gains are rescaled by 20/24.4 while their identified Kf residuals remain. All seven retained long/mixed cases pass, worst **0.438821 N**, with **zero voltage clipping**. Maximum requested voltage is **31.738466 V**, leaving only **0.261534 V** below the modeled output limit; peak current is **2.455697 A**. This is a new native electrical check, not reuse of the older pointwise voltage screen. At least 32 V usable output remains supported on these cases, but driver dropout and hot winding tolerances need an assembled headroom measurement. The fixed hot resistance does not simulate evolving temperature. [Selected hot-coil evidence](/home/tommie/Projects/actuator-model/acf_simulink/validation/budget_adc_hot_coil_summary_20261002.json).

#### Recommendation

Use an **ADS131M08 at 30 ksample/s per simultaneous channel** as the prototype starting point. Engineer and measure a **matched approximately 8 kHz complete force/acceleration response**, retain the **50 µs EMA setting**, and verify the **20 µs total post-conversion pipeline including SPI**. Feed fresh samples to the **10 kHz force controller** and align the position encoder and estimated normal to the same measurement frame.

Use these experiments to choose acquisition hardware and a measured response target. Characterize the assembled acceleration and force chains together: complex response, effective timestamp, axis matching, ADC/MCU timing, hot scale/bias and powered-tool vibration. Match the force, acceleration and estimated-normal frames before reconstructing workpiece force. The tighter calibration targets remain **±0.01 N force zero, ±0.005 m/s² acceleration bias**, and the other residuals in the main report. At S=2 kg, 0.005 m/s² acceleration bias already corresponds to 0.01 N of inertia error.

The required physical acceptance remains **maximum independent reference-force error plus its applicable uncertainty ≤0.45 N** over the full loaded bump recording, including startup and powered sanding. The bandwidth/cost study is a finite simulation result; it does not provide powered-tool vibration or prototype accuracy measurements.

The native acquisition variant, its helper/parameter hashes, all executed cohort receipts and selected full histories are retained under `acf_simulink/validation/`. The public desktop model still contains the scientific reference chain, so its older results remain reproducible. Use [the lower-cost study recipe](/home/tommie/Projects/actuator-model/acf_simulink/experiments/README_budget_sensing_20261002.md) to run the frozen variant in a private batch process. Replacing hardware response assumptions requires a new recorded comparison.

**Concrete recommendation:** build the guided voice-coil/measured-inertia PI prototype with the selected lower spring pair and 60 N·s/m damping, a synchronized multi-axis B–S cell, S–A position measurement and the reference S-mounted acceleration sensors. Reduce acquisition cost with **ADS131M08 at 30 ksample/s per channel**, a measured matched **8 kHz complete response**, 50 µs EMA setting, **20 µs total readout/MCU pipeline** and fresh **10 kHz control**. Retain the **100 µs current target and at least 32 V usable bipolar output** from a nominal 36 V supply. Keep ADS8588S at 50 ksample/s/5.1 kHz as the approximately €19 converter alternative if its stronger observed four-case margin or simpler response is preferred. Choose a cheaper accelerometer only after its measured transfer and full loaded tests recover at least 0.05 N reserve; the current experiments do not show that.

### Fast force controller

#### Intro

The fast controller combines moderate force-error feedback with compensation for gravity, observed tool inertia, drag and the compliant lower mechanism. Its inputs remain causal; surface truth belongs to the physical plant and scoring.

#### Question

Does PI, PID or a more advanced causal strategy solve the sharp bump with useful margin, and which dynamics actually limit further improvement?

#### Results

##### Controller architecture and causal inputs

The fast loop regulates reconstructed normal force. A PI/PID correction combines with gravity compensation, measured-acceleration feedforward and a measured correction for surface slope and sanding drag. The two-state motion observer estimates S–A separation and velocity from relative position, measured S acceleration and the causal robot-motion predictor. With upper A rigidly attached to the prescribed robot boundary, these give an estimate of the tool's axial velocity and therefore local surface slope. An upper compliant attachment needs an additional estimate of A's motion before this reconstruction can be qualified.

![Fast force control and gated delayed robot recentering](figures/acf_causal_control_loops.svg)

*The force loop corrects fast disturbances. The robot loop manages travel slowly; closing its force-stability gate leaves already-issued commands in the queue and predictor.*

The geometry feedforward uses an estimated ratio of tangential drag to pressing force. From the reconstructed reaction vector and estimated slope:

```text
mu_hat_raw = −(Rx_hat + s_hat Rz_hat) / (Rz_hat − s_hat Rx_hat)
Rz_request = Nref (1 − mu_hat s_hat) / sqrt(1+s_hat²)
```

The ratio is accepted only while estimated pressing force and the denominator exceed 1 N, is bounded to [−1,1], and is smoothed with a 5 ms EMA. It starts at zero and holds its previous value when invalid. This supplies measured drag compensation without assuming that the controller knows the plant's friction coefficient. A radial fluctuation of reaction force cancels in the ratio, which helps avoid turning geometry feedforward into an additional high-frequency force-feedback path. Force reconstruction itself still uses the measured vector projected onto the estimated normal.

For an identified coil force constant Kf and positive error `e = Nhat − Nref`, the main current request is:

```text
Irequest = Ih_hold + (Rz_request − Nref)/Kf
           − (mB_identified + mS_identified) aS_FF/Kf
           + friction_estimate(vgap_hat)/Kf
           + lower_inverse_current
           + feedback_filter(−Kp e − Ki integral(e) − Kd derivative_filtered(e))
```

The measured-acceleration term corrects the disturbance after it is observed. It supplies no future surface information. A separate optional causal derivative lead and feedforward filter trade transient delay against amplification of accelerometer noise. The identified masses and force constant determine these commands; the physical plant retains its actual masses, winding and force constant. Saturation and antiwindup keep the integrator consistent with available current. PI gains are expressed explicitly in A/N and A/(N·s); changing Kf does not silently change user gains.

The coil's own current regulator retains winding resistance, inductance, back EMF and supply clipping. Actual current follows its physical winding state, rather than jumping instantly to the request. The force loop and current loop are separate: a fast software update does not establish either loop's physical bandwidth.

Fast feedback and feedforward have different filtering needs. Feedback must avoid exciting delayed structural modes; feedforward needs to follow the tool's measured acceleration promptly. Filtering the entire current request can stabilize a mode while also delaying the useful acceleration correction. The structural controller search therefore examines both total-current mode suppression and filtering of the feedback correction alone, with the same physical sensor inputs. Every selected filter must be tested with repeated bumps and changed structural frequency, rather than accepted from one quiet trace.

##### The lower-link inverse

A lower spring/damper gives B and S different accelerations. Compensating only the measured S acceleration as though B always moved with S leaves a substantial error. Increasing PI gain or adding derivative action did not recover the required margin in the tested compliant designs.

Let `q` be the change in S–B separation from its loaded equilibrium, and let `deltaL` be the **actual** lower-link load change. The physical constitutive law and force balance use that actual motion. Separately, the controller forms `deltaL_desired` from measured S acceleration and the requested axial contact reaction:

```text
deltaL_desired = (Rz_request − Nref) − mS_identified aS_measured
deltaL = k1 q + c1 dq/dt
aB = aS − d²q/dt²
Fa_net = Rz − (mB+mS)(g_axis+aS) + mB d²q/dt²
Fcoil = Fa_net + guide_friction
```

The last acceleration term is the missing B–S relative-motion correction. The selected native branch converts measured desired load into an extra current:

```text
I_lower_inverse(s) = (mB_identified / Kf_identified)
                     × s² / (k1_identified + c1_identified s)
                     × LP2(s, 1500 Hz, damping 0.707)
                     × deltaL_desired(s)
```

The two-pole low-pass makes this a proper, causal transfer rather than an unbounded ideal differentiation. It is implemented with ordinary Sum, Gain and Discrete Transfer Fcn blocks. The controller uses recorded sensor inputs and known mechanism parameters; it receives neither the true tool acceleration nor the bump trajectory. The filter and identified coefficients are prepared before simulation, while the physical force and motion still run in native blocks.

This correction helps the actuator accelerate B differently from S while the lower compliance carries the requested load. In the retained worst repeated-bump trace it reaches about **0.50 A**, equivalent to **9.94 N** for the generic 20 N/A coil. It changes the required current waveform appreciably even though the short bump itself is only 1 mm high.

| Selected core setting | Value | Meaning |
| --- | ---: | --- |
| Upper connection | Rigid | A follows the modeled robot boundary |
| Lower opposed-pair net stiffness | 19,746 N/m | Two approximately 9.873 N/mm springs |
| Lower parallel damping | 60 N·s/m | About 2 N peak damping force in the retained repeated test |
| Common preload | 33.6 N per spring | Both springs remain seated throughout the tested motion |
| Proportional / integral gain | 0.30 A/N / 20 A/(N·s) | Moderate force correction around the feedforward command |
| Derivative gain | 0 | No derivative term in the selected PI preset |
| Feedback-only filter | 1 kHz, two poles, damping 0.707 | Limits force-feedback noise without filtering every feedforward term |
| Lower-link inverse filter | 1.5 kHz, two poles, damping 0.707 | Limits derivative effort in the causal mechanism inverse |
| Guide-velocity prediction | 400 µs | A measured-motion timing correction, not surface preview |
| Measured drag-ratio filter | 5 ms | Smooths the inferred tangential/normal force ratio |
| Force/current control update | 10 kHz | Distinct from the actuator's actual current bandwidth |
| Physical integration step | 10 µs | Reference mechanics checked against 5 and 2.5 µs; final acquisition refinement reported with its own source |

These are coefficients for the stated masses, generic coil and selected sensing model. The known tool mass, holder mass, force constant and lower-link coefficients must be updated together. Deliberate calibration-error tests retain a distinction between actual plant parameters and the controller's identified values.

##### Equal-cohort PI/PID/feedforward comparison

The first controller screen uses the same physical measurements and force reconstruction for every strategy. It contains **72 gain-search recordings and 56 strategy recordings**, each scored from its full native history. These recordings use rigid attachments as a reference while the finite-sensor structural controller is being qualified.

| Rigid-reference strategy, equal eight-case cohort | Worst normal-force error | Cases passing ±0.5 N |
| --- | ---: | ---: |
| PI with measured-acceleration, geometry and guide-friction feedforward | 0.374 N | 8/8 |
| Same, with small filtered derivative action | 0.378 N | 8/8 |
| Same, with half guide-friction feedforward | 0.380 N | 8/8 |
| Same, with guide-friction feedforward removed | 0.385 N | 8/8 |
| Geometry/friction feedforward, without acceleration feedforward | 1.667 N | 0/8 |
| PI with static holding current only | 1.840 N | 0/8 |

The cohort includes the light and heavy tool endpoints, a clean run and three noise seeds at each. Increasing proportional or integral gain increased the worst seeded error. This supports a moderate PI correction with measured feedforward; derivative action is not automatically an improvement. The selected mechanism and its measured-acquisition results are evaluated separately below; the rigid-reference comparison is used to explain the control choice.

![Measured acceleration improves the equal-cohort controller comparison](figures/inertial_controller_reference_comparison.svg)

*56 completed native recordings, with the same reconstructed normal-force measurement for every strategy. This reference uses rigid connections; the real finite-cell design is qualified separately.*

![PI gain sweep for the rigid reference](figures/inertial_PI_reference_gain_search.svg)

*72 completed native recordings, eight common cases per gain pair. Every cell reports the worst full-history normal-force error, including startup.*

##### Finite-cell structural sensitivity

The finite load-cell study changes the structural problem substantially. In 72 recordings over six cell stiffnesses, the same measured-drag geometry strategy retains contact without voltage or current clipping. With Kp=0.06 A/N, Ki=20 A/(N·s) and a feedback-only two-pole 1 kHz filter with damping 1.5, **35 of 36 cases** meet the force band. The 64 MN/m cohort's worst error is **0.492 N**, leaving only 0.008 N reserve; the 110 MN/m cohort has a 0.570 N noise-sensitive failure. Thus short nominal passes alone are insufficient for a practical recommendation.

A separate six-case 110 MN/m study applies a 1 ms acceleration-feedforward filter and causal derivative prediction. All six pass, with **0.471 N** worst error. The corresponding eight-case 64 MN/m cohort reaches **0.493 N**: smoothing reduces stationary structural noise but consumes transient margin. These are comparative structural results; the final full-range timing and calibration qualification is reported separately below.

![Load-cell stiffness and controller sensitivity](figures/inertial_cell_stiffness_sensitivity.svg)

*72 native stiffness/controller recordings plus six filtered-feedforward recordings at 110 MN/m. Every point is a common cohort's worst full-history error. The star is a separately tested controller setting, rather than an interpolated filtered-controller curve.*

##### Selected core controller and repeated bump test

The selected core candidate then passes **32 short endpoint recordings** and **three 10 s repeated-bump recordings** with a heavy 2 kg S, 50 N, 90°, drag ratio 0.3, and noise seeds 7, 29 and 1729. The repeated recordings cross seven sharp bumps each. The worst error is **0.37263 N**, leaving **0.12737 N** of nominal force reserve; there are no current/voltage clips, contact losses or travel-stop hits. These recordings use the matched 100 µs response-envelope reference. Final acquisition, calibration and full-grid results are reported separately so they are not confused with this core controller test.

![Native repeated sharp-bump force, current and spring motion](figures/inertial_mechanics_selected_repeated.svg)

*Retained seed-29 heavy-tool recording, scored at every original 10 µs sample. The display uses min/max envelopes to preserve peaks. Current limits are ±3.5 A; the force band is ±0.5 N.*

##### Observed limits and faster-drive tradeoffs

In the selected lower-series reference, the worst 80-case endpoint reaches **0.33570 N** at about **0.77 ms after bump entry**. Its exact native force balance shows +6.695 N of B-side relative inertia and −6.717 N from the causal lower-link inverse: the combined residual is only −0.022 N. Other simultaneous contributions are +0.848 N current tracking, +0.211 N acceleration timing, +0.406 N guide-friction estimation, +0.008 N geometry and −1.114 N filtered PI correction. They sum to the actual normal-force error within 3.3×10⁻¹⁴ N. The terms interact and partly cancel; their individual peaks must not be treated as independent error allowances.

The guide-friction residual has a clear mechanical cause. The position/acceleration observer supplies S–A motion, while guide friction acts on B–A motion. These velocities differ when the lower spring moves. Estimating that difference from measured link load is possible, but differentiation also amplifies load-cell noise near a Coulomb-friction reversal. The delivered qualified controller keeps the tested approximation; a new guide observer needs its own native qualification. More voltage does not correct a wrong motion estimate.

For comparison, the stiff finite-cell baseline reaches its 0.492 N peak about 0.44 ms after bump entry. An exact reconstruction from its retained native signals attributes approximately **0.095 N to current tracking, 0.184 N to acceleration timing and 0.216 N to B–S relative inertia**, with smaller geometry/friction terms and about −0.009 N PI correction. Physical acceleration is used for these diagnostic balances only; the controller receives measured acceleration.

The loaded cell mode is visible near 1.6 kHz. Halving the current-response target from 100 to 50 µs does not solve it: the three equal eight-case cohorts have worst errors of **0.526, 0.594 and 0.510 N**, depending on feedforward shaping. All retain contact, and none clip voltage or current. The faster drive more strongly excites the cell through measurement noise. A supply increase cannot cure a force error caused by unsaturated sensor delay and structural noise; voltage helps when the winding actually runs out of voltage for the required slew.

![Native force-error balance and the faster-drive tradeoff](figures/inertial_performance_limit_decomposition.svg)

*The upper panel uses every retained 10 µs sample around the first bump entry. The lower panel decomposes each setting's worst retained endpoint trace; its peak time and noise seed can differ. This is a balance analysis of five existing recordings, rather than an additional experiment.*

These stiff-cell results motivated synchronized low-latency analog acquisition, response matching and separate feedback/feedforward shaping. More proportional gain, more prediction or a faster current loop are not universal improvements. The current-response comparison for the selected compliant mechanism is separate from this earlier stiff-cell result.

The robot predictor is another practical limit. In the core 96-case calibration screen, **91 cases pass and five fail**, all caused by actual robot command delay differing from the assumed 80 ms. With a 120 ms actual delay, one heavy-tool trace reaches **0.777 N after the bump has ended**, without clipping, a stop or packing failure. At that instant the actual robot velocity is about +2.14 mm/s while the prediction is −2.20 mm/s; the observer infers an apparent slope on a flat surface. The force gate can see a stable reconstructed force even while the true surface force is wrong. Therefore the gate and predictor must be used together with an identified loaded robot response. The final sensor version's joint calibration tests determine the acceptable timing allowance.

For the **selected compliant mechanism and final acquisition**, twelve complete 10 s native recordings compare 50, 100 and 150 µs current-response targets at 24 and 36 V, using two repeated-heavy noise seeds. Each recording crosses seven bumps. All twelve pass. Worst force errors are **0.31271, 0.34532 and 0.37230 N**, respectively; the corresponding nominal margins are **0.18729, 0.15468 and 0.12770 N**.

![Current-response speed and voltage headroom in the selected mechanism](figures/inertial_final_current_response.svg)

*12 native repeated-heavy recordings. The 24/36 V comparison changes available voltage separately from current-response tuning; all original samples are scored.*

The faster 50 µs response improves this sampled worst by only **0.03261 N** compared with 100 µs, while doubling the modeled first-order bandwidth from about 1.59 to 3.18 kHz and raising requested voltage to **28.05 V**. At 24 V it clips for 42–43 native 10 µs samples, about 0.42–0.43 ms total over 10 s. A 36 V ideal bus removes that clipping and changes the waveform, but leaves the cohort's peak force error equal to about 10⁻¹² N. At 100 and 150 µs, all 72 logged channels in each matched 24/36 V recording pair are bitwise identical because neither bus clips. Faster current response slightly increases flat/noise RMS while reducing the sharp transient peak. The selected **100 µs** target gives useful nominal margin with lower required bandwidth; 50 µs is a separately tested development option, not the full-grid or joint-calibration target. A useful local timing estimate is `S × peak jerk × time mismatch`: at S=2 kg, 384 m/s³ and 50 µs, this is about **0.0384 N** of tool-inertia mismatch. It explains why tens of microseconds matter, while remaining an engineering sensitivity estimate rather than a prediction of the complete closed-loop improvement.

The selected current-loop study has settled-flat peaks around **0.084–0.091 N**, while its global maxima occur during bump entry/recovery. The worst four-second joint-calibration trace likewise peaks at **0.58 ms after entry**: true normal force is 50.448 N while its estimate is 50.267 N, and actual versus measured S acceleration is 0.215 versus 0.126 m/s². At that instant the actual/predicted robot speed differs by only 0.081 µm/s, so this peak primarily exposes sensing/drive timing. The exact balance is +0.633 N current tracking, −7.268 N command/feedforward and +7.083 N B-relative inertia, summing within 2.1×10⁻¹⁴ N. The lower inverse deliberately cancels the large relative-inertia contribution; it is not a standalone unavoidable 7 N error.

The next useful performance work is to measure and match installed sensor/driver phase, reduce friction and calibrate the lower inverse under the real mount. Raising supply voltage beyond the verified usable headroom leaves those errors unchanged. A faster current loop can provide a modest improvement once its noise, hot-load stability and wider calibration envelope are demonstrated.

The longer repeats show why calibration deserves priority: the original static residuals consume about 0.15 N as sustained offset, and halving them recovers about 0.075 N of worst error without changing drive bandwidth or timing. This is a larger observed improvement in its paired cohort than the 0.033 N nominal benefit from doubling current bandwidth in the separate drive study. Those are different cohorts, so the benefits are not additive. Lower tool mass also reduces inertia and timing sensitivity directly; keep S light where the sanding task allows it.

The paired long-worst diagnosis makes that distinction explicit. On the same quiet flat interval, true-force offset falls from **0.148810 to 0.074255 N**, while peak error falls from 0.490396 to 0.415311 N. Subtracting that offset leaves nearly unchanged transient excess: **0.341586 versus 0.341055 N**. The robot is essentially stationary at both late peaks, and both gates are ready. Calibration removes a hidden steady pressing offset; it does not materially improve transient tracking or reduce noise in this comparison.

#### Recommendation

Use **measured-inertia Feedforward PI** with the causal lower-link inverse. The generic 20 N/A coil setting is **Kp = 0.3 A/N**, **Ki = 20 A/(N·s)** and **Kd = 0**, with a **1 kHz feedback-only filter** and **1.5 kHz inverse roll-off**. Identify assembled k/c, downstream mass and Kf; rescale current-domain gains for another force constant.

Retain physical current dynamics, saturation and antiwindup. Raising voltage helps only when clipping limits slew; a deliberately faster current loop needs its own sensing/noise and thermal qualification. Assess the learned residual separately below before changing this prototype baseline.

### Robot commands and recentering

#### Intro

The robot provides slow gross motion and recenters the short-stroke actuator. Its delay makes unqualified force chasing risky, and commands already transmitted continue to affect motion after new commands are blocked.

#### Question

When is force stable enough to issue a recentering command, and how should queued motion and loaded robot delay constrain that command?

#### Results

##### Stability gate, delay-aware gain and command prediction

The robot loop recenters actuator travel slowly. Its output should not chase a 0.1 s bump through a delayed industrial robot. The existing gate checks reconstructed force error within ±0.2 N, filtered trend within 2 N/s and a 100 ms stable interval. A second detector evaluates force variation at every 10 kHz force sample: a 100 ms error mean, 20 ms fluctuation-variance filter and an 0.08 N RMS threshold. Every failing fast sample resets the stable dwell, including threshold crossings between robot ticks. Robot commands are issued at 100 Hz only after the dwell is complete. These thresholds are explicit control settings, rather than a promise that filtered force alone bounds the true workpiece force.

The requested recenter gain is bounded using expected robot delay, response time and gate timing. For the modeled 80 ms command delay, 60 ms response and two 100 ms gate intervals, an 0.8/s request is capped at approximately 0.735/s. A physical robot still needs a measured loaded response before accepting this gain.

Closing the gate suppresses new correction commands. It cannot cancel commands already in a delay queue. The command predictor therefore models already-issued motion; it is not a preview of future bumps. The selected recordings test that distinction explicitly.

The prescribed-motion robot model is suitable for studying command delay and stroke management. It does not yet represent force-induced robot compliance, joint dynamics or a controller that changes its own behavior under load. Those properties require measurements of the actual robot and a low-gain prototype recenter test before increasing correction gain.

##### Accepted travel band and sustained motion

The current objective accepts **±7.5 mm about the 12.5 mm stroke midpoint without a centering penalty**. The 0–25 mm physical stops remain unchanged, leaving 5 mm between either accepted-band edge and the corresponding stop. The learned controller may leave the robot stationary anywhere inside that band. Excursion outside it adds an increasing soft cost; contact loss and a physical stop still fail the run.

The raw learned correction can request up to ±10 mm/s, then the existing delay-aware sign/amplitude envelope, force gate and robot limits constrain the actual command. The native envelope permits only about 5.15 mm/s at a 7.5 mm offset. With a sustained 5 mm/s outward programmed motion, the steady balance is therefore near a 7.3 mm offset, leaving little band margin. Allowing a broad band does not justify waiting until its edge before compensating a known delayed command.

Development includes bounded programs that reverse, constant nominal motion from the beginning of the recording, and initial offsets outside the accepted band but inside the physical stops. Full histories report maximum and final band excess and time inside; an intentional initial excursion is not mislabeled as a failed physical stop. Queued nominal and correction commands remain active after the gate closes. Fresh long tests separately assess whether the selected policy maintains travel under continued motion.

#### Recommendation

Keep the robot update at **100 Hz**, with the slow force-error/trend gate and the **0.08 N RMS variation detector** evaluated at fast force samples. Require the **100 ms stable dwell** before new recentering corrections. Use the loaded-delay/response gain cap, track issued commands explicitly and preserve their queued effect when the gate closes.

The evaluated reference is **80 ms delay / 60 ms response**, with **±5 ms identification residuals**. Measure the actual loaded robot and begin with a low recenter gain. Keep ±7.5 mm as an acceptable travel region, anticipate already-issued motion and retain the same hard gate and command envelope for learned control.

### Learned control policy

> **Final sensing contract pending root's locked package.** The new budget candidate uses ADXL354C plus ADS131M08, a declared 1.9 kHz matched response envelope and 50 µs current response. These are candidate model settings, not executed qualification results. Final sensor, acquisition and driver statements must come from `selected_controller/cohort_manifest.json` and its hashed bank/native metadata. Earlier gen5 memory and arithmetic records below retain their own sensing/current-response inputs.

#### Intro

Our goal is one compact controller choosing voice-coil current and robot correction from causal measurements and known tool parameters. The workpiece's normal force must stay within ±0.5 N through the 1 mm × 10 mm bump scanned at 100 mm/s, across 5–50 N, S = 0.1–2 kg and fixed orientations of 0–90°. B is 0.6 kg. Travel within ±7.5 mm of center has zero centering cost.

Both actions use one shared history: current updates every 100 µs and new robot corrections are permitted at 100 Hz. Native Simulink mechanics, contact, sensing, current regulation, command delays and limits remain the environment.

The latest prototype budget is **less than €100 for accelerometer and ADC chips together**. The current training is being transferred to an ADXL354C-like ±8 g sensing envelope, a matched 1.9 kHz complete response and 50 µs current-loop target. The earlier faster-sensor experiments below retain their own scope. Fresh budget-chain results have not yet been executed.

#### Question

Can the policy learn fast force control and delayed robot correction using measurable inputs and the existing stability constraints? A good result needs complete force recordings and useful robot actions; low force error alone can hide a robot that receives no commands.

#### Results

##### Design and causal inputs

The network has a 48-to-32 encoder, two genuine Puffer minGRU layers with 32 states each, and two action means. Recurrent, direct measurement and physical-product readouts combine within the same actor. The direct path preserves fast force response alongside the recurrent history.

![One shared recurrent controller and its causal signal paths](figures/unified_recurrent_control_band10_v3.svg)

*Two outputs share one history. The hard robot gate, issued-command queue, current regulation and physical limits remain explicit.*

The observation contains 40 values; eight products are derived inside the policy. Fixed centering/scaling comes from engineering ranges and is stored with the weights.

| Input group | External values | What the policy receives |
|---|---:|---|
| Force reconstruction | 3 | Normal, axial and lateral force from synchronized load-cell and downstream S-inertia measurements. |
| Tool motion and geometry | 6 | Measured S acceleration in two modeled directions, S–A position, observed relative velocity, estimated actuator travel and estimated local slope. |
| Commands, guard and recenter history | 9 | Present/aligned predictions from issued robot commands; issued current, total robot velocity and correction; gate readiness; recenter error. |
| Known or identified context | 21 | Target, masses, orientation, spring/damper and coil parameters, electrical/travel limits, equilibrium current and acquisition/robot timing. |
| Robot clock | 1 | The current 100 Hz decision phase. Absolute episode time and bump timing are absent. |
| Derived features inside the policy | 8 additional | Mass-conditioned acceleration, S–B interactions, force geometry, issued robot acceleration, damping × velocity and stiffness × travel error. |

Inference uses the causal observation port and identified context. True force and hidden states feed scoring/rewards. Surface profile and future motion stay outside inference; prediction uses commands already issued, including queued commands.

The minGRU state is carried between decisions and across rollout/chunk boundaries. It resets at a new physical episode. Closing the robot gate preserves both the neural history and commands already in flight. The initial paired filters span 0.1, 0.4, 3 and 20 ms; their weights and update gates can change during learning.

The two latent means are mapped to full requests as follows:

- Current request: known static equilibrium current plus `2 A × tanh(current mean)`.
- Robot correction request: `10 mm/s × tanh(robot mean)` at eligible 100 Hz events.

The experimental current replaces the dynamic force PI/feedforward command. Static equilibrium uses known weight/orientation and target; native current regulation and physical limits still apply. Robot force-stability/variation gates, dwell, delay-aware sign/amplitude envelope and velocity limits remain hard constraints. An ineligible update sends no new correction and cannot withdraw an issued command.

Travel excursion beyond center ±7.5 mm is penalized. Some development cases intentionally start outside: report entry, time inside and maximum/final excess. Physical 0–25 mm stops remain failures. A stationary robot within the band is acceptable.

##### Initialization and learning

Initialization uses a 0.1 A/N direct force gain and signed filter pairs cancelling their common startup component. Later probes initialized trainable recurrent/auxiliary connections with an approximate known spring/damper inverse. This disclosed physics prior remains within the actor's coefficients.

PPO samples continuous latent actions and updates the actor from rewards and a value head. Discount/advantage constants use physical seconds. Contact/travel failures terminate; time limits bootstrap the final causal observation with cloned state. Robot losses use eligible 100 Hz actions. The stabilized critic detaches its gradient from the shared trunk to limit actor interference.

Evolutionary reward search evaluates candidates in the native closed loop using reward, antithetic directions and covariance adaptation. There are no teacher actions or supervised labels. The band/history stage searched 55 coefficient directions; inverse refinement searches 37. The architecture stores 8,101 trainable parameters. These searches optimize constrained subspaces; PPO can update the full actor. The selected checkpoint and its ancestry are recorded with the results below. [PPO method](https://arxiv.org/abs/1707.06347), [evolutionary reward optimization](https://arxiv.org/abs/1703.03864), [minGRU architecture](https://arxiv.org/abs/2410.01201).

Whole trajectories separate training, selection and final evaluation. Weights, source, native library, inputs, seeds and identities lock before fresh outcomes. Metrics include every native sample, startup and final sample. Failures/rejections remain retained. Assembled sanding performance requires measurement.

##### Completed development evidence

One fixed band-search checkpoint, `48618b8d21c4f5920d870a05f5eb198c958632f51ffab6f0abfd6a8255ef0ad4`, was tested with its 64 learned states either carried normally or reset at every 100 µs decision. Both variants used identical weights, sensors, observers, plant, seeds and robot queues. TRAIN cases 0, 40, 96 and 139 produced eight complete recordings and 7,800,008 scored native samples. All eight passed the force requirement and avoided physical failure.

The force-peak changes were small and mixed. Carried/reset peaks were 0.10811/0.10832 N, 0.40415/0.40447 N, 0.15539/0.15087 N and 0.23457/0.23877 N. Travel showed a clearer history effect: TRAIN 96 spent 90.28%/51.59% of its recording inside the accepted band, and TRAIN 139 spent 82.71%/0%. The latter ended inside the band with carried state and 1.406 mm beyond it with resets. This is evidence that the learned history affects robot correction in these development cases. The reset variant retains the external causal observers and is not an independently optimized memoryless controller.

That checkpoint has 30 nonzero recurrent current-readout weights and 10 nonzero robot-readout weights, with L2 norms 0.784 and 4.285. Its band-search stage evolved 55 directions. The experiment isolates the effect of retaining this network's state.

![Effect of carrying or resetting recurrent state](figures/memory_reliance_v2.svg)

*Four development pairs using the same earlier gen5 weights. History helps stroke management; force-peak changes are small and mixed. This is not a newly optimized memoryless baseline.*

##### Native inference and numerical precision

C inference carries 64 float32 states and emits action means/value. Its 8,181-float buffer holds parameters plus 80 normalization values, about 32.7 kB. CPU collection avoids per-tick GPU synchronization; the GPU performs PPO learning. Native recordings independently score force, electrical effort, travel and commands.

An inverse initialization with readout coefficients up to 662.1 amplified C/PyTorch state differences and cancellation error. Its first rollout showed a maximum current-request difference of 0.3155 mA and log-probability differences of 0.0356 on CUDA/0.0394 on CPU. Replay guards stopped that PPO continuation before an update. Both readout rounding and recurrent arithmetic contributed.

Fixed C closed-loop results retain their implementation/hashes. PPO needs verified behavior replay. Export acceptance must use the actual selected weights and arithmetic; compiler options and datatype are part of that contract.

The replay measurements are retained in `validation/rl_budget3h_20261005/runner_checks/inverse_replay_v1/results.json`; the four-case memory test is retained in `validation/rl_puffer_probe_20261005_final_driver_checks/memory_ablation_gen5/results.json`, both under `acf_simulink/`.

##### Current development result

**Study in progress.** The faster-sensor evolved candidate passes all **28 development cases**, with **0.372088 N worst error** and mean per-recording RMS of **0.037378 N**. These cases guide selection; the prepared fresh 32-case test, 700-condition nominal grid and four 30 s cases have not been run. The report will bind its final recommendation to the locked winner and those completed tests.

The reward compares physical failures first, then force passes, then a joint score dominated by worst force error and soft travel-band excess. It adds a small RMS term. Robot inactivity inside the accepted band is valid. Constant-motion cases test whether the policy eventually recenters.

##### Why stronger compensation is not always better

Initializing stronger inverse dynamics reduced force peaks, but the strongest version raised force variation enough to close the robot gate. One sustained-motion development run hit a travel stop after 1.093 s and was rejected. A partial version completed the same 4 s recording. The optimizer is refining the force response and gate-compatible behavior together.

![Compensation, force stability and remaining travel](figures/force_gate_travel_tradeoff.svg)

*Identical development scenario141: the stronger inverse loses travel despite a small force error. Physical failure cannot be exchanged for a better force metric.*

#### Recommendation

Complete the locked fresh evaluations before selecting a prototype controller. The public native Simulink model remains the verified deterministic reference while these local experiments continue.

### Calibration, model qualification and prototype validation

#### Intro

Native simulation supports a design choice only within its recorded model, metric, inputs and test identities. Calibration residuals, mechanical identification and physical validation consume reserve beyond nominal noise.

#### Question

What has actually passed, how much margin remains under mismatch, and what must be measured before claiming ±0.5 N on the powered physical prototype?

#### Results

##### Full scientific-reference grid and numerical qualification

The selected final system retains the soft lower link and causal inverse, and uses the explicit 100 ksample/s matched acquisition graph, the 10 kHz controller and the fast force-variation gate. Nine complete 10 s endpoint recordings pass, including a clean heavy-tool reference and noisy light/heavy endpoints. Their worst normal-force error is **0.32242 N**. A separate two-seed repeated-heavy drive study, discussed below, reaches **0.34532 N** at the same selected 100 µs current response. Noise seeds and test cohorts differ; each quoted maximum belongs to its recorded cohort.

The broad nominal grid covers five S masses (0.1, 0.5, 1, 1.4 and 2 kg), ten force targets (5–50 N in 5 N increments), seven fixed orientations (0–90° in 15° increments), two physical drag ratios (0 and 0.3), and one clean plus four noisy conditions. **All 3,500 unique four-second cases pass**, with all **400,001 native samples per case** scored, including startup: **1,400,003,500 force samples in total**. The coverage audit finds no missing or duplicate identities and verifies all 700 common five-seed groups. The worst error is **0.315811 N**, leaving **0.184189 N** inside the ±0.5 N band. It occurs with S=2 kg, 50 N, 75°, drag ratio 0.3 and seed 97, at 2.00063 s. This qualifies the specified finite nominal grid; the calibration and physical-parameter screens have their own narrower scopes.

The worst errors at S=0.1, 0.5, 1, 1.4 and 2 kg are respectively **0.1033, 0.1096, 0.1719, 0.2295 and 0.3158 N**. Across the grid, the generic winding reaches **2.9466 A and 19.5489 V**, without current/voltage clipping, contact loss or a travel stop. The fixed 33.6 N preload passes every case, with a common feasible interval of **30.374–40.637 N per spring**, 40.98 mm conservative combined free length and 34.174 mm combined installed length. The largest normal-force balance residual is 2.84×10⁻¹⁴ N.

The four interleaved partitions use one immutable scientific model, candidate and helper set. Their two execution recipes differ only in temporary simulation-data storage: 1,060 validated disk-backed recordings are reused once, and 2,440 remaining identities use private RAM-backed storage after a full-channel parity proof. The 122 earlier recovery recordings remain separate from the 3,500 unique cases. The case CSV, complete coverage audit and retained worst native history are in `validation/normal_mass_controller_final_grid_20260930_*`.

![Full nominal force, orientation and tool-mass qualification](figures/inertial_final_range_coverage.svg)

*3,500 completed native recordings. Each cell shows the worst complete-history error over both drag ratios and all five clean/noisy conditions. All masses have 700 recordings; the common color scale ends at the ±0.5 N limit.*

![Native worst case from the complete nominal grid](figures/inertial_final_worst_native.svg)

*The retained global worst recording: S=2 kg, 50 N, 75°, drag ratio 0.3, seed 97. Every 10 µs sample in the plotted window is shown; all 400,001 samples determine the metric. The 100 Hz robot gate remains ready here because its filtered stability criteria remain satisfied.*

Rerunning this exact worst identity at **5 and 2.5 µs** gives peak errors of **0.315822976 and 0.315825399 N**, compared with 0.315810929 N at 10 µs. The finer difference is only **2.42 µN**, and the total change from the working step is **14.47 µN**. Both refined histories pass force, spring packing, contact and travel checks without clipping. All 72 physical/control channels are compared on their own sample grids; the four robot-gate channels remain at their actual 100 Hz rate. Raw acquisition, controller periods, noise and physical inputs are unchanged. This supports the 10 µs working step for this final candidate and worst nominal case.

##### Random noise, fixed offsets and what is not injected

The nominal 700-case lower-cost grid includes sensor noise, converter noise and explicit quantization, with zero residual static calibration errors. The selected long/mixed stress cases add the tighter fixed biases, gains, alignment and identified-parameter errors. These selected corners do not exhaust the complete error box.

| Measurement | Original reference-conditioned sensor noise, RMS | Fixed residual zero in selected stress cases |
| --- | ---: | ---: |
| Force, each used axis | 0.02 N | ±0.01 N |
| S acceleration, each used axis | 0.014 m/s² | ±0.005 m/s² |
| S–A position | 1 µm | No independent position-zero error stressed |

Force and position noise start as bounded uniform noise; acceleration noise is Gaussian. Input noise density is fixed when changing conditioning or sample rate, so the listed conditioned RMS values are not constant at every bandwidth. Converter noise is added separately before smoothing and explicit quantization; the selected M08 reference-acceleration chain uses approximately **0.00284 N force noise** and **0.01045 m/s² acceleration noise**, RMS at that injection point. Position uses a separate encoder. These contributions cannot be added directly without their transfer functions.

The fixed residual biases remain constant during each recording. The studies do not provide measured powered-sander vibration, vibration rectification, evolving thermal drift, random sampling jitter or switching-current ripple. Those need assembled measurements.

##### Executed calibration and timing mismatch

The sensor requirement is a calibrated, synchronized measurement chain. ADC bit count, catalog resolution and sample rate are only parts of that chain. The completed final-source calibration work includes **42 sensing one-at-a-time recordings**, **14 robot-response recordings**, and **60 simultaneous-error recordings**. All 116 meet the force band in their declared cohorts. Six separate native endpoint/parity records verify that the instrumentation and Accelerator runner preserve the complete 72 logged channels bitwise; they are software proofs rather than extra uncertainty cases.

The joint sample uses two operating endpoints: S=2 kg, 50 N, 90°, and S=0.1 kg, 5 N, 0°. Each has three noise seeds and ten deterministic sign corners. Twenty-six declared residual errors change together. Each recording lasts four seconds. The worst is **0.44809 N**, leaving **0.05191 N** to the limit, at the heavy endpoint with seed 7 and corner 8. All recordings complete without clipping, contact loss or a travel stop, and the actual 33.6 N preload passes their 20 mm spring packing checks. This finite corner sample does not prove every combination inside a 26-dimensional error box or every mass/force/orientation under those simultaneous errors.

Four **30 s repeats**, each crossing **24 sharp bumps**, extend the difficult joint corners: three heavy corner-8 seeds and one light corner-3 seed. All four pass every one of their **3,000,001 native samples**, including startup, without clipping or physical stops. Heavy worst errors are **0.488090, 0.487071 and 0.490396 N**; the light case reaches **0.086891 N**. The heavy seed-1729 maximum occurs **0.690 ms into the tenth bump**, at 12.80069 s. It leaves only **0.009604 N** reserve, despite the substantial nominal-grid margin.

The heavy repeats also show a sustained true-force offset of approximately **+0.147–0.148 N** while the feedback holds its own reconstructed force near target. Calibration errors therefore consume substantial reserve before the sharp transient arrives. The values below are tested outer stress limits, rather than a desirable routine operating point. A tare removes zero/gravity offset but cannot by itself correct load gain, cross-axis coupling or response mismatch.

| Measurement or identified quantity | Tested outer stress budget | Selected calibration/noise target | Design meaning |
| --- | --- | --- | --- |
| Force, each used axis | ±0.02 N zero, ±0.1% reading gain | **±0.01 N zero, ±0.05% gain**; same 0.02 N conditioned RMS noise | Calibrate the operating force range and gravity tare; factory full-scale accuracy alone is insufficient |
| Force cross-axis coupling | ±0.1% residual per tested direction | **±0.05%** | Calibrate the matrix under representative axial/lateral loads |
| Force and accelerometer frame | ±0.1° residual alignment each | **±0.05° each** | Align sensor axes to flange coordinates and confirm tare consistency |
| S acceleration | ±0.01 m/s² bias, ±0.5% scale | **±0.005 m/s² bias, ±0.25% scale**; same 0.014 m/s² conditioned RMS | About 0.51 mg bias; verify thermal drift and powered-tool vibration |
| Known downstream S mass | ±0.5% identified residual | **±0.25%**, about ±5 g at S=2 kg | Include everything beyond the cell and use tool-specific identification |
| Holder B and force constant | ±0.5% identified residual each | **±0.25% each** | Partition moving masses and map calibrated Kf over the used stroke/temperature |
| S-relative-A position | ±0.1% scale residual | **±0.05% scale**; same 1 µm conditioned RMS | Synchronize the relative sensor; count size is not complete accuracy |
| Commanded scan speed | ±0.1% identified residual | **±0.05%** | Use actual issued tangential motion and its timestamp |
| Analog response matching | Common cutoff ±5%; other axes ±5% relative to force | Same tested mismatch; identify complex response | Pairwise cutoff difference can reach 10% of common cutoff; this was unchanged in the refinement |
| Acquisition timing | Common 20 µs pipeline ±10 µs; other used channels ±10 µs relative | Same tested mismatch, common clock | Actual delays span 0–40 µs; this was unchanged in the refinement |
| Robot prediction | Actual 80 ms delay ±5 ms and 60 ms response ±5 ms | Same **±5 ms** residual targets | Identify loaded response, retain pending commands and check timing |

These budgets are **residuals after calibration**, not guaranteed out-of-box specifications or interchangeable independent allowances. The joint test holds mechanical k/c nominal; the ±20% physical mechanical study is separate. A fixed delay mismatch is also different from random latency jitter. Known actual k/c does not remove the persistent sensor-calibration offset in these nominal-mechanics repeats.

The selected static targets receive a direct **four-record, 30 s paired refinement**. Only sixteen bias, gain, alignment and identification residuals are halved; noise, analog/pipeline mismatch, robot delay/response, actual mechanics, current response and PI settings remain unchanged. All four new records pass their full native histories. The heavy three peaks become **0.411872, 0.411243 and 0.415311 N**, and the light peak **0.075628 N**. The worst reserve increases from **0.009604 to 0.084689 N**, recovering **0.075085 N**. Sustained heavy force offset falls to approximately **+0.073–0.074 N**.

This is evidence that static calibration can recover useful reserve without a faster or noisier controller. It qualifies those four declared repeats, rather than the whole tighter error box or every nominal grid point with simultaneous residuals. The tighter targets need warm known-load/axis-matrix calibration, accurate mass measurements and tool-specific Kf identification; they are requirements to verify on assembled hardware. The manufacturer examples are plausible measurement-chain starting points, rather than guarantees of those residuals.

![Tighter calibration recovers margin through repeated sharp bumps](figures/inertial_calibration_refinement.svg)

*The same four 30 s corner cases before and after sixteen static residuals are halved. Every 3,000,001-sample history determines its metric; the lower histories use all-native min/max bins. Noise, analog response, pipeline, robot timing, actual mechanics and current response remain unchanged.*

![Final sensing and robot-response sensitivity](figures/inertial_uncertainty_v8_sensing.svg)

*42 native sensing records. Each entry reports its own tested worst full-history force error; it is not an independently additive budget.*

![Simultaneous calibration errors at the two declared endpoints](figures/inertial_uncertainty_v8_joint.svg)

*60 native records, ten deterministic 26-dimensional corners, three seeds at each endpoint. The full ±0.5 N band includes startup and every bump sample.*

The ATI Mini27-class cell's published force resolution and full-scale accuracy do not establish the required 0.02 N conditioned noise or selected ±0.01 N residual zero. A TWE/raw bridge interface with calibrated conditioning is the reference route for controlling the complete chain; a turnkey low-rate digital interface is not automatically an equivalent substitution. Use two appropriately aligned ADXL1002-class analog acceleration axes on S, a simultaneous ADC/clock for force and acceleration, and timestamp the relative-position counter to that same frame. If all six raw bridge channels and two acceleration axes are acquired, an eight-channel simultaneous ADC fits that analog count; the position counter is separate.

Mount the accelerometer on the rigid downstream assembly so it represents the modeled mass acceleration. Tool rotation, pad flexure or an orbital mechanism can make acceleration at the sensor differ from the assembly center-of-mass acceleration; this model does not supply that correction. Test the powered sanding tool under real vibration, with the complete antialias chain. A white-noise pass is not a vibration or aliasing guarantee.

A useful physical validation separates three measurements: static force calibration over the intended force/orientation range, a dynamic response/phase measurement of force versus acceleration, and an independent contact-force reference while running the bump. Verify current-loop response under the hot winding, low-friction motion, assembled mounting modes and loaded robot response before claiming the simulated margin on hardware.

##### Model limitations and disturbance scope

The model calculates force balances, explicit rigid constraints, spring/damper loads, stroke, actuator current and voltage, sensor filtering/noise/delay, causal estimation, fast force control and slow delayed robot recentering. It can reject a proposed spring pack for its working travel and combined length, and it can identify whether a recording fails because of force error, contact loss, travel exhaustion, current limit or drive voltage.

It represents translational point contact in a scanning plane. A real sanding pad distributes pressure over an area and deforms. Local surface shape, pad compliance, rotations, moments, abrasive removal and spatially varying friction can change the relationship between tool motion and the local surface normal. The guides and robot must carry lateral force and moments without introducing unmodeled motion. Tool vibration and accelerometer vibration rectification are not supplied by the bump waveform.

The qualified bump is the specified **C² polynomial profile**: height, velocity and acceleration are continuous, with finite jerk jumps at entry and exit. Height, width and speed alone do not specify disturbance severity. As an analytical comparison, a half-cosine bump of the same dimensions has an approximately **1.974 m/s² acceleration jump** at entry, equivalent to **3.95 N of S inertia at 2 kg**. A continuous spring/damper force and finite current response cannot instantly cancel an unpreviewed jump. This comparison adds no simulation qualification; sharper edges need their own tests and realistic pad/contact compliance. Extra voltage does not remove the causal limitation.

Electrical winding dynamics and voltage clipping are retained, but winding temperature is a separately calculated operating constraint until thermal dynamics and force-constant drift are measured. Coil geometry, position dependence, cable forces, bearing stiction, backlash and whole-flange packaging require prototype characterization. A 100 mm spring allowance is not a 100 mm envelope for the actuator, guides, sensor and sanding head together.

Each run starts at a loaded equilibrium with known or calibrated tool mass and fixed orientation. A finite parameter/noise grid establishes performance on its recorded cases; it does not prove every possible disturbance, every noise realization or a continuously rotating operating trajectory. Numerical qualification is evidence for choosing and testing a prototype, rather than a substitute for testing one.

##### Running, changing and reproducing the native model

Open the saved diagram with `addpath('acf_simulink'); mdl = acf_open;` from the project folder in MATLAB. The root **Actuator selection** chooses coil or leadscrew; **Force controller selection** chooses Feedforward PI, PI or PID. The green Run button uses structure **p in the Model Workspace**, so the physical diagram remains the leading model. The [getting-started guide](/home/tommie/Projects/actuator-model/acf_simulink/GETTING_STARTED.md) shows the parameter editor, controller gain fields, removed-spring preset and the exact slow-bump example with B+S=2 kg. That slow example completes with **0.07328 N** worst force error. The [13-page native diagram PDF](/home/tommie/Projects/actuator-model/acf_simulink/generated/active_flange_native_model_pages_20260930.pdf) lets you inspect the blocks without opening MATLAB.

The delivered model uses ordinary native blocks and shallow subsystems, with separate physical drive, force reconstruction, motion observer, fast controller and delayed robot paths. Private acceptance checks establish source-history parity, direct Run after parameter edits, both physical failure stops and save/reopen behavior. The public diagram removes 33 unused wire stubs while preserving all connected edges and parameter values. Preserve any unsaved edits before reopening a model already loaded in the desktop.

For repeated studies, use Accelerator mode, reuse valid compilation and clear simulation-data history after scoring each case. Temporary SDI logging was a substantial bottleneck. A paired four-case pilot compared private disk and RAM-backed SDI storage with bitwise equality across all 72 channels: median warm ten-second execution fell from **39.78 to 6.81 s**, about **5.84×** under the recorded concurrent load. Final workers typically completed a four-second simulation in roughly **2–2.8 s warm**. These are host/load-specific timings, not cold-start or UI timings. The physical model and 10 µs working step stayed unchanged.

The [reproduction guide](/home/tommie/Projects/actuator-model/acf_simulink/REPRODUCING_RESULTS.md) identifies the immutable scientific source, candidate, two exact grid recipes, calibration/actuator studies, execution checks and portable input snapshots. Metrics always use full native histories before plot reduction. The offline HTML report supports source inspection and filtering of all 3,500 cases; the separate replay preserves the worst native force sample. Historical studies and interrupted/software-rejected attempts remain recorded separately from completed physical cases.

#### Recommendation

Use the **tighter post-calibration residuals**, identify each assembled tool/mechanism, and check force balance, response/phase, hot driver headroom and structural modes. The nominal 3,500-case reference grid and 700-case lower-cost grid have different frozen acquisition sources and noise cohorts; neither proves all combinations of mismatch or real sanding vibration. Keep the learned-policy test cohort distinct from those established baselines.

Validate with an independent workpiece-force reference while the sander is powered and at operating temperature. The proposed physical criterion is **maximum absolute reference-force error plus applicable measurement uncertainty ≤0.45 N**, preferably with about **0.02 N or better reference uncertainty**. This is an acceptance target, not an observed hardware pass.

## Recommendation

Build the first prototype around a **guided voice coil, an upper rigid attachment, one lower opposed spring pair and a 60 N·s/m parallel damper**, using the **measured-inertia Feedforward PI** controller. Keep the B–S multi-axis cell in the complete lower load path, mount synchronized acceleration sensing on S and measure S relative to A. Identify the assembled spring/damper values for the inverse, rather than relying on catalog rates. Use the issued-command robot predictor with the force-stability/variation gate and delay-aware recentering gain cap.

| Subsystem | Concrete starting specification |
| --- | --- |
| Lower spring pair | Net 19.746 N/mm, approximately 9.873 N/mm each, about 20 mm free length each, 33.6 N preload each |
| Spring working envelope | At least the modeled 68.54 N individual force envelope and 3.18 mm cyclic stroke; guided, no cell bypass |
| Parallel damping | Approximately 60 N·s/m with low stiction; characterize at roughly 0–35 mm/s |
| Moving mass | B budget 0.6 kg, interchangeable S=0.1–2 kg; include cell/tool-side attachments in the correct mass |
| Coil | Approximately 20–25 N/A, preferably at least 65–70 N continuous **over the used stroke and at operating temperature**, 25 mm travel, bipolar drive |
| Electrical drive | Start around R=4–6.4 Ω, L=3–4 mH; 3.5 A or more peak provision, continuous current sized from hot holding load |
| Supply | 36 V if the driver provides **at least 32 V usable bipolar output** under load; verify dropout, phase and heatsinking |
| Inner current loop | Identified 100 µs response target under the actual hot load; verify phase, ripple and current readback |
| Fast force control | 10 kHz; P=0.3 A/N and I=20 A/(N·s) for the 20 N/A generic coil; rescale/identify for another Kf |
| Dynamic compensation | Measured S acceleration, measured drag/geometry, guide-friction feedforward; lower-load inverse at 1.5 kHz using measured assembled k/c |
| Acquisition | ADS131M08, 30 ksample/s per simultaneous channel, measured matched ~8 kHz complete response, 50 µs EMA setting, 20 µs total readout/MCU pipeline |
| Acceleration sensors | Retain two ADXL1002-class analog axes for the planar prototype; three for spatial reconstruction, with enough simultaneous ADC channels |
| Position and calibration | ~1 µm conditioned position noise; use the tighter static targets above, including ±0.01 N force zero and ±0.005 m/s² acceleration bias |
| Robot | Identify loaded delay/response, target ±5 ms residuals, retain queued commands and gate new recentering commands |
| Learned control | Unified recurrent reward-trained candidate under evaluation; retain the verified deterministic reference pending fresh tests |

The lower-cost acquisition is the new prototype starting point; the public desktop model retains the frozen scientific-reference chain. Seven new native hot-winding cases with the selected M08 chain pass at 0.438821 N worst, but request up to **31.738466 V**, leaving only **0.261534 V** at a 32 V usable-output limit. Measure real driver dropout and hot-coil headroom. The previously reviewed approximately €9 converter reduces ADC cost; the tested cheaper accelerometers do not preserve the preferred 0.05 N force reserve.

**Unified learning study, 5 October — in progress:** the shared recurrent controller has a promising development result. Final prototype promotion depends on the frozen-weight fresh tests and implementation checks below; the present hardware starting recommendation remains unchanged.

A Moticont LVCM-070-051-01-class coil demonstrates plausible force/stroke/electrical values, but it is not a blanket purchase recommendation: its continuous-force curve near stroke ends, hot force constant and cooling must satisfy the required operating region. Budget approximately 1.5 kg total for A in this example, including its 1.24 kg stationary body; its moving coil uses about 0.24 kg of B. Check the robot payload and whole mechanical package. The spring length limit alone does not bound that package.

Use the tighter static-calibration column as the starting acceptance specification, with measured assembled k/c and verified sensor/driver phase. A proposed physical bench criterion is **maximum absolute independent reference-force error plus its applicable measurement uncertainty ≤0.45 N** through the complete loaded test. Prefer a reference with approximately 0.02 N or better applicable uncertainty. This preserves at least 0.05 N against the ±0.5 N requirement if the criterion is achieved; it is a prototype test target, not an already observed hardware pass. Confirm it at operating temperature and with the sanding tool powered.

A larger low-backlash screw servo remains a credible alternative if it supplies at least roughly 0.088 N·m continuous torque at 10 mm/revolution plus reserve, while keeping rotor/screw inertia and friction low. The tested RE25 reference is thermally undersized for continuous 50 N. For the first prototype, choose the coil because it implements the tested force/current path with fewer transmission uncertainties. Validate the complete sensor, drive, guide, spring/damper, mount and robot chain against an independent surface-force reference before claiming ±0.5 N on the powered sanding tool.

**Build the first prototype with the guided voice coil, selected lower spring/damper pair, synchronized M08 and fast analog acceleration sensing, and deterministic force/robot controllers. Accept the assembled system only after independent powered-sanding force tests meet the stated margin.**
