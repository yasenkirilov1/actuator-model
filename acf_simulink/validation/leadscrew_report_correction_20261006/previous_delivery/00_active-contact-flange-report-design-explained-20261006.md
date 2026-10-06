# Designing the active sanding flange

## Intro

Our goal is a flange that keeps the **normal pressing force experienced by the workpiece within ±0.5 N**, including a **1 mm high, 10 mm wide bump crossed at 100 mm/s**. The required operating range is **5–50 N**, tool masses of **0.1–2 kg**, and fixed flange orientations from **0–90°**.

For the **first prototype**, we will use a **one-axis load cell plus tool acceleration** and ignore drag coupling into that axis. Separating normal force from sanding drag on slopes remains an open problem requiring multi-axis force sensing. We defer that extension for now.

The mechanics are prescribed: a **20 N/mm upper spring with zero external damping and unlimited ideal length/travel**, and a **rigid lower attachment**. The load cell still has finite stiffness. The actuator has a real stroke limit. We may use measured force, tool acceleration, tool position relative to the flange body, and robot commands already issued. The controller receives no surface preview.

**A** is the actuator body, **B** the 0.6 kg moving holder, and **S** the tool beyond the load cell. Full-range reference tests use A = 1.5 kg for realistic body/mounting mass. Taring tool weight removes gravity, but not dynamic inertia.

## Question

Can this arrangement meet the force requirement, and what actuator, load cell, accelerometer and acquisition electronics should we use for a first prototype? In particular, where can we reduce cost without accepting too much force error?

## Results

**The current controller is a useful development baseline, but the prescribed arrangement does not yet meet the complete requirement.** The following recorded tests used multi-axis reconstruction. They inform the design; they do not qualify the new one-axis prototype. The controller was locked before these fresh tests:

| Test group | Passed | Largest force error | Hard stops |
|---|---:|---:|---:|
| 700 nominal combinations of mass, force and orientation | 683/700 | 0.580 N | 0 |
| 32 sensing/calibration and motion stresses | 21/32 | 3.948 N | 0 |
| Four repeated-bump/drift recordings, up to 30 s | 1/4 | 0.691 N | 1 |

Every simulation sample counts, including startup. Contact loss or a travel stop fails the recording. Two long tests leave the accepted ±7.5 mm band; one reaches a hard stop after 4.19 s. These recommendations support an **instrumented development prototype**.

![Where the nominal fixed design meets the force requirement](figures/fixed-force-operating-grid-20261006.svg)

*Each square contains the largest recorded error across seven orientations. Outlines use the unrounded 0.5 N threshold. This finite grid does not include the separate calibration/motion stresses or long tests.*

### Mechanics: load path, springs and guides

#### Intro

The upper spring connects the robot to A. The actuator moves B relative to A. The load cell connects B to S, and the surface supports S while contact is maintained.

![Prescribed mechanics and first-prototype measurement locations](figures/prototype-mechanics-single-axis-20261006.svg)

*Position measures S relative to A. The load cell carries the entire tool S; an accelerometer is mounted on S.*

#### Question

What is a mechanical “mode,” and why can a nominally rigid connection still cause force ringing?

#### Results

A **mode** is a characteristic pattern of vibration: which parts move relative to one another, and how quickly they oscillate. A disturbed mass on a spring rings in one such pattern; a structure can have several. [Modal-analysis explanation](https://www.mathworks.com/discovery/modal-analysis.html).

The upper spring permits relatively slow body motion. A different, much faster mode involves tiny deformation of the load cell and movement of B relative to S. With the reference **64 MN/m cell stiffness** and 0.6 kg holder, its frequency is about **1.64 kHz**, or one cycle every **0.61 ms**. A 0.5 N cell-force change requires only **7.8 nanometres** of deformation. “Rigidly mounted” therefore does not mean mechanically infinitely stiff.

The native Simulink plant applies mass force balances, gravity, surface geometry, drag, coil electrical dynamics and guide friction. Zero external damping leaves the assumed internal **0.2 N Coulomb friction and 2 N·s/m viscous friction**. Maintained contact makes S follow the smooth bump; the simulation stops when contact or travel limits fail.

![Load-cell mode and the purpose of its feedback](figures/load-cell-mode-explained-20261006.svg)

*The mechanical motion is schematic and exaggerated. The feedback uses acquired axial load-cell data; it has no access to future surface shape.*

#### Recommendation

Keep the prescribed spring and lower attachment. Measure the assembled cell/frame/guide vibration frequencies and friction before tuning the controller. Real sensor or mounting stiffness can shift the ringing frequency substantially; the present narrow-band correction must then be identified again.

### Actuator and electrical drive

#### Intro

The actuator must change force quickly while supporting sustained sanding loads, within the moving-mass and thermal budgets.

#### Question

Why start with a voice coil rather than a leadscrew, and which voice-coil parameters should we look for?

#### Results

A voice coil makes linear force directly: **force ≈ force constant × current**. A leadscrew converts rotation into linear motion, providing mechanical advantage but adding rotational inertia, friction and possible backlash to the force-control problem.

The modeled bump requires **34.35 mm/s peak tool speed and 2.4 m/s² peak acceleration**. At S = 2 kg, tool inertia alone changes the support-force measurement by **4.8 N**. These are tool motion values; actuator travel also depends on A moving on the upper spring.

The RE25 sizing comparison shows the trade-off. At **10 mm screw lead**, holding 50 N ideally needs about **3.40 A**, above that motor's **1.16 A continuous rating**. A 3 mm lead reduces this to 1.02 A before losses, but increases the equivalent translating rotary inertia from **0.624 kg to 6.93 kg**. This is the extra mass the motor's rotational inertia effectively adds when accelerating the tool.

![Leadscrew pitch trades holding current against effective inertia](figures/leadscrew-pitch-inertia-current-20261006.svg)

*Analytical RE25 sizing, separate from the native transmission model. Friction and efficiency add to the ideal holding demand.*

Both actuators passed four historical transient tests with different mechanics. A larger motor or transmission could work; these results favour a voice coil without ruling out all leadscrews.

For a coil, examine the following parameters together:

| Parameter | Development target and reason |
|---|---|
| Force constant | Around **24.4–25 N/A**, specified over the used stroke; converts force demand into current |
| Continuous force | At least **55–60 N at the actual hot temperature and stroke**, with the specified mounting/cooling |
| Peak force/current | About **85 N / 3.5 A**, with permitted pulse duration and duty cycle |
| Travel | **25 mm**, retaining a ±7.5 mm working band about the midpoint |
| Resistance and inductance | Reference: **6.4 Ω cold, 8.896 Ω modeled hot, 4 mH**; these set heat and current-slew voltage |
| Mass | Include moving coil in B and stationary body/mount in A; the reference body alone is about **1.24 kg** |

Voltage demand is **V = RI + L·dI/dt + Kf·v**: winding resistance, changing current and motion-induced voltage all matter. A steady 50 N hot-coil hold needs **2.05 A and 37.4 W**. Check sustained hot force near stroke ends, not just catalog cold/centre ratings.

Higher voltage helps only when the driver is clipping. One unchanged-controller nominal run is bitwise identical at 32 and 48 V. In a clipped stress case, raising usable voltage reduces error from **2.748 to 0.650 N** and removes clipping; demand reaches **33.02 V**. Both stressed runs fail ±0.5 N. Extra voltage addresses one limitation, while sensing, ringing and robot motion remain relevant.

#### Recommendation

Use a voice coil for the first prototype, with an adjustable **48 V-rated bipolar driver** and a **≥36 V usable-output development target**. Check actual winding voltage after driver losses. Provide a separate fast current-measurement loop, overtemperature protection and adequate cooling. The evaluated controller used 32 V; this recommendation is not a newly qualified 36/48 V configuration.

### Force and position measurement

#### Intro

The B–S load cell measures the force supporting the tool. That includes the surface reaction, tool inertia and, before tare, gravity. It is not automatically the force pressing normally into the workpiece.

#### Question

What kind of load cell do we need, and can we get away with a single force axis?

#### Results

**Yes, for our reduced first-prototype scope.** With known tool mass and consistent gravity tare, add **S mass × measured S acceleration** to the axial cell reading. This reconstructs axial contact force: tool acceleration contributes 4.8 N at the heavy-tool bump peak, so static zeroing alone is insufficient. Holder B is upstream of the cell and is not part of this inertia correction.

On an aligned flat test surface, axial contact force is normal pressing force. On a slope, the estimate also needs surface direction: without drag, divide the axial component by the cosine of the angle between the sensor axis and surface normal. S–A position and measured motion help estimate that direction causally.

**Open problem: sanding drag.** Drag is tangent to the surface, but on a slope it can project onto a fixed sensor axis. Acceleration removes inertia; it cannot separate two unknown contact-force components. Addressing this needs two force axes for planar sanding, or three for spatial sanding. We will ignore this coupling for now; [the geometry and measurement evidence](/home/tommie/Projects/actuator-model/acf_simulink/experiments/report_readability_20261006/loadcell/README.md) remain available for the later extension.

Raw axial cell force ranges from **−18.9 to +59.8 N** in the 736 reference tests. A roughly **±100 N measurement range** is a reasonable sizing target with reserve. The earlier ±160 N number is **ADC electrical scaling**, not a required physical cell rating.

#### Recommendation

Use a **one-axis cell between B and S**, with an accelerometer on S and position measured from S to A. Target **±0.01 N residual zero**, **±0.05% residual gain error** and **±0.05° alignment** after calibration. Check off-axis sensitivity, stiffness, overload survival and mounting. Reserve multi-axis sensing for the drag problem.

### Acceleration measurement

#### Intro

An accelerometer on S measures the tool motion needed to remove its inertial contribution from the force reading.

#### Question

How accurate must it be, and does it also need an ADC?

#### Results

An acceleration error of **0.01 m/s² becomes 0.02 N force error at S = 2 kg**. Averaging can reduce random noise; it cannot remove fixed bias, scale error or a timing mismatch with the load-cell signal.

The reference is an **ADXL354C**, an analogue accelerometer whose outputs need conversion. The recorded planar model acquires two axes; the first-prototype four-channel ADC can accommodate one load-cell output and all three acceleration outputs. A digital accelerometer has an internal converter but still needs response-delay and noise checks. The model's reference-conditioned acceleration noise is **0.0187 m/s² RMS**, before ADC noise; this is not guaranteed installed output noise.

#### Recommendation

Use the ADXL354C as the development reference, rigidly mounted on S. Target **±0.005 m/s² residual bias**, **±0.25% scale residual** and synchronized force/acceleration acquisition. Use three acceleration axes for spatial motion. Test vibration from the running spindle: resolution alone does not establish useful acceleration accuracy.

### Acquisition electronics: amplifier, ADC and timing

#### Intro

An **ADC** turns analogue voltages into numbers. The controller needs those numbers to estimate contact force and decide coil current. The instrumentation amplifier first strengthens the small bridge signal and limits unwanted high-frequency content.

#### Question

Is a separate ADC necessary? How good must it be, and what is the cheapest option for an acceptable force error?

#### Results

**Conversion is necessary for analogue sensors; a separate converter chip is not inherently necessary.** A microcontroller may already contain ADCs, or a digital load cell may contain one. What matters is the complete acquisition path, including noise, analogue filtering, conversion, communication and timestamps.

**Resolution** sets one digital step; advertised bits do not guarantee accuracy. **Sampling rate** counts fresh readings per channel. **Bandwidth** describes changes the sensor/filter can follow; rapid sampling cannot restore changes a slow filter removed. Delay and channel mismatch affect inertia correction and vibration feedback.

The reference **ADS131M08** supplies eight simultaneous channels at **30,000 fresh samples/s per channel**, followed by **10,000 controller updates/s**. Its modeled path includes matched two-pole **1.9 kHz responses**, about **49.6 µs converter group delay**, a **50 µs smoothing filter running at the ADC output rate**, and **20 µs pipeline including SPI**. “Matched” means force and acceleration see comparable timing and filtering, rather than merely sharing an advertised cutoff. [TI datasheet](https://www.ti.com/lit/ds/symlink/ads131m08.pdf).

The fixed-controller diagnostics compare seven known development cases, changing either code spacing or actual converter rate/noise/delay. All startup samples count; the controller was not retuned.

| Converter setting | Cases passing ±0.5 N | Largest recorded error |
|---|---:|---:|
| 30 ksample/s, 24-bit codes | 7/7 | 0.495 N |
| 30 ksample/s, 16-bit codes | 7/7 | 0.492 N |
| 30 ksample/s, 14-bit codes | 6/7 | 0.502 N |
| 30 ksample/s, 12-bit codes | 4/7 | 0.553 N |
| 15 ksample/s, 24-bit codes | 6/7 | 0.539 N |
| 7.5 ksample/s, 24-bit codes | 0/7 | 52.218 N; all stopped during startup |

![ADC resolution and sampling-rate comparisons on fixed mechanics](figures/adc-fixed-diagnostics-20261006.svg)

*42 completed diagnostic recordings, not a fresh qualification grid. Bit tests retain M08 noise and timing; rate tests change converter delay too. The stopped 7.5 ksample/s recordings are shown separately because they never reach the bump.*

At the modeled scaling, **16 bits gives 0.00488 N per force step and 0.00359 m/s² per acceleration step**. This looks usable, but does not qualify a real 16-bit chip's noise or accuracy. Lower sample rates reduce converter noise while increasing delay from **49.6 to 99.6 or 199.6 µs**. Here that timing change makes control worse. The experiments support keeping 30 ksample/s with this controller; they do not establish a universal minimum or an optimal analogue cutoff.

| ADC option | Recorded chip cost, VAT included | Decision |
|---|---:|---|
| ADS131M04, four simultaneous channels | €7.15 | Fits one force plus three acceleration outputs |
| ADS131M08, eight simultaneous channels | €11.43 | Keep for the recorded eight-channel arrangement or later expansion |
| MCP3208, multiplexed 12-bit | €2.95 | Code spacing, channel skew and rate need a different qualified design |
| Existing microcontroller ADC | €0 additional chip cost | Potential saving; measured noise, filtering and timing still unproven |

*Quantity-one prices observed 6 October 2026; other electronics and shipping excluded. [Parts, sources and experiment details](/home/tommie/Projects/actuator-model/acf_simulink/experiments/report_readability_20261006/adc/README.md).*

MCP3208's **100 ksample/s is aggregate**, giving at most 25 ksample/s each for four sequential channels at 5 V. Neither it nor a built-in microcontroller ADC inherits the simultaneous-converter results. M04's reviewed OSR128 noise and filter topology match M08's corresponding setting. [M04 datasheet](https://www.ti.com/lit/ds/symlink/ads131m04.pdf).

#### Recommendation

Choose **ADS131M04** for the first prototype. It is the cheapest reviewed option that plausibly retains the reference timing/noise envelope, and keeps the ADXL354C-plus-ADC chips below €100 at the recorded prices. Keep **30 ksample/s per channel** and the modeled matched **1.9 kHz response** initially. Validate the assembled chain and one-axis controller before claiming performance; cheaper acquisition remains a separate development option.

### Fast force controller

#### Intro

The coil handles rapid force changes. The robot has a slower role: keeping the actuator away from its travel limits.

#### Question

What does “cell-mode feedback” actually do, and why does it improve the bump response?

#### Results

The main **PI controller** changes current in proportion to force error and accumulated error. Its gains are **0.0246 A/N and 16.4 A/(N·s)**, with a 1 kHz force-feedback filter. Measured-inertia feedforward adds effort for tool motion before a large force error develops. A body-motion observer estimates A's unmeasured movement from force, acceleration, relative position and issued robot commands.

**Cell-mode feedback is a separate vibration correction.** It selects the load-cell ringing band and times a small extra coil force to oppose the oscillating motion. Its combined frequency/phase filter has no steady-state contribution; the ordinary force loop still sets the average pressing force. Correction is bounded to **±0.05 A**, or approximately **±1.22 N coil force**.

Timing is essential. Sensors, ADC filters and the current driver delay the correction. If we simply push against an old measurement, the holder may already be moving the other way and the push can feed the vibration. The selected filter compensates the identified response near 1.64 kHz. It is a targeted correction, not a general guarantee of damping at every frequency.

![Measured improvement from cell-mode feedback in a development case](figures/fixed-mechanics-force-limit-20261005.svg)

*Same 2 kg tool, 50 N target, 90° orientation and noise seed. Complete-recording error falls from 1.074 to 0.459 N, and post-bump cell-band power falls by 14.6 dB. This is a matched development example, not the worst fresh test.*

#### Recommendation

Use this inspectable deterministic architecture as the starting point. Identify real cell and driver phase before retuning. The selected family passed its eleven development points, but the fresh results show that more robust sensing and vibration control are still needed.

### Robot commands and recentering

#### Intro

A robot correction can arrive after the force condition that triggered it has disappeared.

#### Question

How do we avoid turning recentering into delayed positive feedback?

#### Results

New **100 Hz commands** require stable force: a **0.08 N RMS variation gate** resets a **100 ms dwell**, alongside a slower stability condition. The predictor retains issued commands, approximately 80 ms delay and 60 ms robot response. Closing the gate does not cancel commands already queued. Travel within **±7.5 mm** about the midpoint incurs no centering cost.

#### Recommendation

Keep conservative, delay-aware recentering and the stability gate. Identify the robot's actual timing and test repeated bumps plus sustained drift. The long-test travel stop shows why a good short bump response is insufficient.

### Learned control policy

#### Intro

We also trained one recurrent policy to request coil current and robot correction from measurement history.

#### Question

Should learning replace the deterministic controller?

#### Results

On the earlier mechanics, the retained policy passed 700 nominal tests but only 30/32 fresh and 3/4 long tests; one long case reached a travel stop. Those results do not transfer to the newly prescribed mechanics.

#### Recommendation

Keep it as research evidence. Retrain and independently test any learned candidate on the current plant before considering it for the prototype.

### Calibration, model qualification and prototype validation

#### Intro

The simulation tests a stated physical and sensing model, not every disturbance from a real sanding machine.

#### Question

Which checks turn this development recommendation into an accepted system?

#### Results

The model assumes fixed orientation, planar translation, known tool mass and preloaded smooth contact. Pad compliance, spindle rotation, angular inertia, frame flexibility, temperature drift and sanding vibration need assembled tests. Force noise is bounded uniform; acceleration noise is Gaussian. Offsets, scale errors, timing and ADC effects are separately represented. Noise fields are normalized to a fixed **3,177 Hz reference bandwidth**, not guaranteed installed RMS.

MATLAB reproduces the reference nominal/stress failures within **31 µN waveform difference**, although stricter parity checks remain failed. Locked inputs, compact results and selected histories are retained; bulk histories were removed in the [storage audit](/home/tommie/Projects/actuator-model/acf_simulink/validation/storage_cleanup_20261006/README.md).

#### Recommendation

Measure assembled masses, modes, friction, current response and sensor timing; calibrate axial load response, off-axis sensitivity and acceleration alignment. Target **≤2 µs relative timing residual**. Compare reconstructed force with an independent reference on the scoped bench setup through repeated bumps and long drift. Then test powered sanding and the deferred drag extension. Aim for **≤0.45 N peak error**, leaving 0.05 N reserve.

The [fixed-model guide](/home/tommie/Projects/actuator-model/acf_simulink/generated/fixed_upper_20261005/GETTING_STARTED_FIXED.md), [ADC details](/home/tommie/Projects/actuator-model/acf_simulink/experiments/report_readability_20261006/adc/README.md) and [earlier report](/home/tommie/Projects/actuator-model/reports/active-contact-flange-report-rl-3h-20261005.md) provide the instructions and detailed evidence.

## Recommendation

The present controller remains **unqualified across the complete ±0.5 N range**. Prototype measurements and a new independently tested candidate must address the exposed sensing, ringing and long-travel failures before claiming success.

**Build an instrumented voice-coil prototype with the prescribed 20 N/mm undamped upper spring and rigid lower attachment.** Use a 25 mm coil sized for hot continuous force, an adjustable 48 V-rated bipolar driver, **one-axis B–S load sensing**, an S–A encoder and an ADXL354C on S. Acquire the force output and three acceleration outputs with **ADS131M04**. Defer multi-axis drag separation.

Start with the deterministic PI/inertia/observer/cell-mode controller, 30 ksample/s acquisition, 10 kHz force control and stability-gated 100 Hz robot recentering.
