# Designing the active sanding flange

## Intro

Our goal is a flange that keeps the **normal pressing force experienced by the workpiece within ±0.5 N**, including a **1 mm high, 10 mm wide bump crossed at 100 mm/s**. The required operating range is **5–50 N**, tool masses of **0.1–2 kg**, and fixed flange orientations from **0–90°**.

For the **first prototype**, we will use a **one-axis load cell plus tool acceleration** and ignore drag coupling into that axis. Separating normal force from sanding drag on slopes remains an open problem requiring multi-axis force sensing. We defer that extension for now.

The prescribed mechanics are a **20 N/mm upper spring with zero external damping and unlimited ideal travel**, and a **rigid lower attachment** with a finite-stiffness load cell. Actuator stroke remains limited. The controller receives force, acceleration, S–A position and issued robot commands, with no surface preview.

**A** is the 1.5 kg actuator body/mount, **B** the 0.6 kg holder, and **S** the tool beyond the load cell. Taring weight removes gravity, but not dynamic inertia.

## Question

Can this arrangement meet the force requirement, and how should we build a voice-coil prototype with integrated, programmable sensing electronics? Where can we reduce cost without excessive error?

## Results

**The locked rigid-contact reference missed the complete requirement. The new soft-pad tests are substantially more promising.** The reference tests below used multi-axis reconstruction and a controller locked before evaluation; they do not qualify the one-axis prototype:

| Test group | Passed | Largest force error | Hard stops |
|---|---:|---:|---:|
| 700 nominal combinations of mass, force and orientation | 683/700 | 0.580 N | 0 |
| 32 sensing/calibration and motion stresses | 21/32 | 3.948 N | 0 |
| Four repeated-bump/drift recordings, up to 30 s | 1/4 | 0.691 N | 1 |

Every simulation sample counts, including startup. Contact loss or a travel stop fails the recording. Two long tests leave the ±7.5 mm band; one reaches a hard stop after 4.19 s.

The new comparison uses **total axial pressing force**, one-axis cell plus S acceleration, and the same upper spring. Two-second recordings start loaded, with the robot fixed. For the selected **77 mm × 5 mm pad geometry**, assuming **50 N/mm stiffness and zero damping**, both actuators pass their 54-case grids: **0.046 N worst for the coil, 0.118 N for the spring-driven screw**. Engagement, calibration stresses and delayed recentering still need testing.

![Where the nominal fixed design meets the force requirement](figures/fixed-force-operating-grid-20261006.svg)

*Each square contains the largest recorded error across seven orientations. Outlines use the unrounded 0.5 N threshold. This finite grid does not include the separate calibration/motion stresses or long tests.*

### Mechanics: load path, springs and guides

#### Intro

The upper spring connects the robot to A. The actuator moves B relative to A. The cell connects B to S, and the pad supports S against the workpiece.

![Voice-coil mechanics and first-prototype measurement locations](figures/prototype-mechanics-voice-coil-pcb-20261006.svg)

*The coil acts between A and B. Position measures S relative to A; the cell carries S, including its sensing PCB. The upper spring remains prescribed.*

#### Question

What is a mechanical “mode,” and why can a nominally rigid connection still cause force ringing?

#### Results

A **mode** is a characteristic pattern of vibration: which parts move relative to one another, and how quickly they oscillate. A disturbed mass on a spring rings in one such pattern; a structure can have several. [Modal-analysis explanation](https://www.mathworks.com/discovery/modal-analysis.html).

The upper spring permits relatively slow body motion. A faster mode involves tiny load-cell deformation and B moving relative to S. With the reference **64 MN/m cell stiffness** and 0.6 kg holder, it is about **1.64 kHz when S is constrained by rigid contact**. With a soft pad, both masses move and the corresponding mode is roughly **1.87–4.35 kHz** across the tool-mass range. A 0.5 N cell-force change needs only **7.8 nanometres** of deformation. Rigid mounting does not eliminate this vibration.

The native Simulink plant applies mass force balances, gravity, contact and electrical dynamics. Zero external damping leaves assumed guide friction of **0.2 N Coulomb and 2 N·s/m viscous**. The new pad model lets S move independently of the surface; contact loss and hard travel limits still stop the run.

![Load-cell mode and the purpose of its feedback](figures/load-cell-mode-explained-20261006.svg)


#### Recommendation

Keep the prescribed spring and lower attachment. Measure assembled cell/frame/guide modes and friction before tuning. The illustration explains the rigid-contact reference; its specific vibration correction does not transfer automatically to soft contact.

### Actuator and electrical drive

#### Intro

The actuator must change force quickly and support sustained sanding loads without excessive mass or heat.

#### Question

How does a voice coil compare with a leadscrew driving a spring, and which voice-coil parameters should we look for?

#### Results

A voice coil makes force directly: **force ≈ force constant × current**. The tested screw compresses an internal **5 N/mm spring between its independent nut and B**. Force feedback adjusts spring deflection through an encoder-based nut servo. The prescribed upper spring and rigid B–S mounting remain. Rotary inertia loads the nut coordinate; it is not automatically extra tool mass. [Series-elastic principles](https://fab.cba.mit.edu/classes/865.15/people/rebecca.kleinberger/assets/papers/SEA_Pratt.pdf).

The screw uses an **RE35-class motor**, **10 mm lead**, assumed **75 g nut/carriage**, and a **60 Hz nut servo**, with zero added spring damping. [Parameters, sources and reproduction](/home/tommie/Projects/actuator-model/acf_simulink/experiments/sea_pad_dynamic_20261006/README.md).

Each grid tests S = **0.1/1.4/2 kg**, force **5/20/50 N**, orientation **0/45/90°**, and two fresh noise seeds. With the **5 mm pad at an assumed 50 N/mm**, the coil's **0.046 N** peak leaves **0.454 N nominal reserve** against the limit; the screw's **0.118 N** leaves **0.382 N**. Their largest two-second average hot-winding losses are **37.4 W versus 7.4 W**. Screw RMS current reaches **1.07 A**, below its 1.42 A catalogue limit.

![Voice-coil and series-elastic screw force errors and winding losses](figures/pad5mm-force-and-actuator-results-20261006.svg)

*All samples count. Both wide-pad grids pass physical, sensor-range and peak-actuator checks. Copper loss estimates heat generation; neither two seconds nor a catalogue current screen establishes installed thermal performance.*

A separate damped **20 N/mm, 10 mm-thick** narrow-contact study also favours the coil: **54/54 at 0.338 N**, versus screw **0/54 at 1.730 N**. With the new 5 mm pad's unchanged gains, both fail all eight matched point-contact cases. Width, material response and tuning jointly determine performance.

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

Higher voltage helps when clipping limits current slew. Earlier unchanged-controller **32/48 V pairs** with the 10 mm broad pad are identical when unclipped. A rigid-contact stress improves **2.748→0.650 N**; both runs fail. The new 5 mm grids request at most **21.1 V coil / 26.4 V screw**, so voltage is currently ample at 32 V.

#### Recommendation

**Choose the voice coil for the first prototype.** It gives the lower force error and a direct current-to-force interface for experimenting with controllers. It also avoids the screw's nut servo, internal spring calibration and backlash mechanism. This choice prioritizes force regulation and mechanical simplicity. The screw remains a credible, lower-heating alternative if energy use becomes the deciding constraint.

Use an adjustable **48 V-rated bipolar current driver**, **3.5 A peak capacity** and at least **32 V usable output**. Verify the **50 µs modeled current-response target**, hot force across the stroke and sustained cooling. The approximately **37 W winding-loss case** makes thermal design part of the coil recommendation.

### Soft pads

#### Intro

A soft pad lets the tool move while the pad compresses. Its width also spreads the encounter with a bump over time. Compliance and footprint are separate physical effects.

#### Question

**How do soft pads influence our results?**

#### Results

The reference is **77 mm diameter and 5 mm thickness**, like the [Mirka interface](https://www.mirka.com/nl-nl/p/Interface-77---6-GRIP/?variant=94118). **50 N/mm is our unmeasured whole-pad assumption**, not a Mirka specification. Tests cover softer/stiffer material, **0/40/80/150 N·s/m damping**, and point versus circular contact. The original full-width ridge remains. Forces are summed from compressed strips after allowing each strip to separate; tool motion stays dynamic.

![Effects of stiffness and assumed damping for a 5 mm pad](figures/pad5mm-material-sensitivity-20261006.svg)

*Four matched conditions per material setting, with conservative gains held fixed. Changing a pad can change stability, rather than merely smooth the disturbance.*

At **50 N/mm and 5 mm thickness**, all **32** material-sensitivity recordings pass, including zero damping. At **100 N/mm**, **23/32** pass; five fail through contact loss. Exact width comparisons give **0/8 point versus 8/8 wide** for each actuator. Some narrow cases exceed sensor ranges and are ineligible diagnostics, not usable prototype trajectories.

The **20 N/mm, 5 mm** cases use the entire assumed **2.5 mm working-compression allowance at 50 N** and stop during startup. This is a linear-model validity limit, not measured Mirka bottoming. Published foam data support the stiffness scale, while constant modeled damping can overstate high-frequency suppression. [Material sources](https://www.rogerscorp.com/-/media/project/rogerscorp/documents/elastomeric-material-solutions/poron/english/data-sheets/17-006-poron-4701-30-very-soft.pdf).

#### Recommendation

Use the **77 mm × 5 mm geometry** and measure compression and frequency response. Around **50 N/mm**, flat compression is **1 mm at 50 N**; the selected tests stay below **1.99 mm maximum**. Retune for the actual pad and confirm nonlinear compression, partial contact and long-term behaviour.

### Force and position measurement

#### Intro

The B–S load cell measures the force supporting the tool. That includes the surface reaction, tool inertia and, before tare, gravity. It is not automatically the force pressing normally into the workpiece.

#### Question

What kind of load cell do we need, and can we get away with a single force axis?

#### Results

**Yes, for our reduced first-prototype scope.** With known tool mass and consistent gravity tare, add **S mass × measured S acceleration** to the axial cell reading. In the rigid-contact reference, the heavy tool's inertial term reaches **4.8 N**; a soft pad changes its motion. Static zeroing alone is insufficient. Holder B is upstream of the cell and is not part of the inertia correction.

On an aligned flat test surface, axial contact force is normal pressing force. On a slope, conversion also needs surface direction. **With a soft pad, tool motion alone does not reveal surface slope**: some motion is pad compression. The new diagnostics therefore report total axial force without inferring an unmeasured normal.

**Open problem: sanding drag.** Tangential drag can project onto a fixed sensor axis on a slope. Acceleration removes inertia, but cannot separate unknown pressing and drag components. That needs two force axes for planar sanding, or three spatially. We defer it; [measurement evidence](/home/tommie/Projects/actuator-model/acf_simulink/experiments/report_readability_20261006/loadcell/README.md) supports the later extension.

The selected 5 mm wide-pad grids span **−19.2 to +55.0 N** raw cell force. Use roughly **±100 N physical range** with overload reserve. Some narrow-contact diagnostics exceed sensor ratings and are rejected. The ±160 N ADC scaling is electrical headroom, not a required cell rating.

#### Recommendation

Use a **one-axis cell between B and S**, with an accelerometer on S and position measured from S to A. Target **±0.01 N residual zero**, **±0.05% residual gain error** and **±0.05° alignment** after calibration. Check off-axis sensitivity, stiffness, overload survival and mounting. Reserve multi-axis sensing for the drag problem.

### Acceleration measurement

#### Intro

An accelerometer on S measures the tool motion needed to remove its inertial contribution from the force reading.

#### Question

How accurate must it be, and does it also need an ADC?

#### Results

An acceleration error of **0.01 m/s² becomes 0.02 N force error at S = 2 kg**. Averaging can reduce random noise; it cannot remove fixed bias, scale error or a timing mismatch with the load-cell signal.

The **ADXL354C** has three analogue outputs, so it needs conversion. Four ADC channels accommodate force and all three acceleration axes. [Manufacturer specifications](https://www.analog.com/en/products/adxl354.html). Earlier acquisition studies use **0.0187 m/s² reference-normalized noise plus ADC noise**. The new pad diagnostics use **0.014 m/s² normalized noise**, about **0.0104 m/s² filtered RMS**, and simplified converter modeling. Neither guarantees installed sensor performance.

#### Recommendation

Mount ADXL354C rigidly on S. Target **±0.005 m/s² residual bias**, **±0.25% scale residual** and synchronized acquisition. Test the running spindle: digital resolution alone does not establish useful acceleration accuracy.

### Acquisition electronics: amplifier, ADC and timing

#### Intro

An **ADC** turns analogue voltages into numbers. The controller uses them to estimate contact force and decide actuator current. The instrumentation amplifier strengthens the small bridge signal and limits unwanted high-frequency content.

#### Question

Is a separate ADC necessary? How good must it be, and what is the cheapest option for an acceptable force error?

#### Results

**Conversion is necessary for analogue sensors; a separate converter chip is not inherently necessary.** A microcontroller may already contain ADCs, or a digital load cell may contain one. What matters is the complete acquisition path, including noise, analogue filtering, conversion, communication and timestamps.

**Resolution** sets one digital step; advertised bits do not guarantee accuracy. **Sampling rate** counts fresh readings per channel. **Bandwidth** describes changes the sensor/filter can follow; rapid sampling cannot restore changes a slow filter removed. Delay and channel mismatch affect inertia correction and vibration feedback.

The reference **ADS131M08** supplies eight simultaneous channels at **30,000 fresh samples/s per channel**, with **10,000 controller updates/s**. The model includes matched two-pole **1.9 kHz responses**, approximately **49.6 µs converter delay**, **50 µs smoothing at the ADC output rate**, and a **20 µs pipeline including SPI**. Matched means comparable timing/filtering for force and acceleration. [TI datasheet](https://www.ti.com/lit/ds/symlink/ads131m08.pdf).

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

At modeled scaling, **16 bits gives 0.00488 N per force step and 0.00359 m/s² per acceleration step**. This supports testing a real 16-bit converter; its noise/timing remain unproven. Lower rates reduce converter noise but increase delay to **99.6 or 199.6 µs**, worsening this controller's result. Keep 30 ksample/s initially; this is not a universal minimum or an optimized cutoff.

| ADC option | Recorded chip cost, VAT included | Decision |
|---|---:|---|
| ADS131M04, four simultaneous channels | €7.15 | Fits one force plus three acceleration outputs |
| ADS131M08, eight simultaneous channels | €11.43 | Keep for the recorded eight-channel arrangement or later expansion |
| MCP3208, multiplexed 12-bit | €2.95 | Code spacing, channel skew and rate need a different qualified design |
| Existing microcontroller ADC | €0 additional chip cost | Potential saving; measured noise, filtering and timing still unproven |

*Quantity-one prices observed 6 October 2026; other electronics and shipping excluded. [Parts, sources and experiment details](/home/tommie/Projects/actuator-model/acf_simulink/experiments/report_readability_20261006/adc/README.md).*

MCP3208's **100 ksample/s is aggregate**, giving at most 25 ksample/s each for four sequential channels at 5 V. Neither it nor a built-in microcontroller ADC inherits the simultaneous-converter results. M04's reviewed OSR128 noise and filter topology match M08's corresponding setting. [M04 datasheet](https://www.ti.com/lit/ds/symlink/ads131m04.pdf).

#### Recommendation

Choose **ADS131M04**, the cheapest reviewed option plausibly retaining reference timing/noise. ADXL354C plus ADC chips stay below €100 at the recorded prices. Start at **30 ksample/s per channel** with matched **1.9 kHz responses**; validate the assembled chain and one-axis controller. The new soft-pad tests have not reswept ADC limits.

### Custom PCB: integrated sensing and local control

#### Intro

A **custom PCB** brings amplification, acquisition and control into one configurable assembly. Mount its accelerometer on S so it measures the same downstream mass used in force reconstruction.

#### Question

Why design our own board, and how does integration improve customization, ease of use, programming and robustness?

#### Results

The acquisition experiments show why timing belongs in the system design: halving converter rate increased worst error from **0.495 to 0.539 N**, while 12-bit codes reached **0.553 N**. These are historical controller diagnostics, rather than measurements of a PCB. They support controlling the full signal chain instead of selecting modules by advertised resolution alone.

![Proposed custom PCB integrating amplification, acceleration sensing, ADC and local control](figures/integrated-sensing-pcb-20261006.svg)

*Proposed architecture. The B–S cell connects to the PCB on S; the power driver sits on A, away from the small analogue signals. Include board and mounting mass in S.*

| Design benefit | Concrete implementation |
|---|---|
| **Customizability** | Selectable amplifier gain and filter components; firmware settings for tool mass, calibration, controller gains and acquisition rate |
| **Ease of use** | One configuration/tare interface, consistent connectors and saved tool presets; calibrated data and fault status in one log |
| **Programming** | Accessible USB/debug connection; timer-driven ADC acquisition and a local 10 kHz force loop, independent of robot communication delays |
| **Robustness** | Short differential bridge wiring, stable excitation/reference, controlled power layout, protected connectors and strain relief; watchdog and driver-disable path |

Use the **ADS131M04's four simultaneous channels** for amplified force and the ADXL354C's three outputs. Common acquisition timing makes channel alignment easier to characterize. Match the complete sensor/filter responses; sample simultaneity alone does not remove phase differences. Keep coil-current switching and its heat near the driver on A. [ADC specifications](https://www.ti.com/product/ADS131M04).

#### Recommendation

**Build one custom sensing/control PCB for the prototype**, including the instrumentation amplifier, analogue filtering, accelerometer, ADC, microcontroller and programming/robot interfaces. Provide gain/filter options and calibration test points. The under-€100 target covers accelerometer and ADC chips; PCB, amplifier, MCU and driver costs are additional. Verify noise, timing, temperature drift and powered-spindle behaviour on the assembled board before claiming an improvement in robustness or force accuracy.

### Fast force controller

#### Intro

The actuator handles rapid force changes; the robot slowly recenters its travel.

#### Question

Which controller should run on the PCB, and what does “cell-mode feedback” do?

#### Results

The rigid-contact reference uses **PI**: proportional current reacts to force error, while integral action corrects accumulated error. Gains are **0.0246 A/N and 16.4 A/(N·s)**, with 1 kHz feedback filtering. Measured-inertia feedforward and a body-motion observer supply additional correction.

**Cell-mode feedback selects the load-cell ringing band** and times a small extra coil force to oppose that oscillation. It contributes nothing at steady state; the main force loop sets average force. Correction is bounded to **±0.05 A**, approximately **±1.22 N coil force**.

By the time a delayed correction arrives, the holder may have reversed direction, so the correction could feed vibration. The reference filter compensates phase near 1.64 kHz. Soft contact changes the response; the pad controllers disable that correction.

![Measured improvement from cell-mode feedback in a development case](figures/fixed-mechanics-force-limit-20261005.svg)

*Same 2 kg tool, 50 N target, 90° orientation and noise seed. Complete-recording error falls from 1.074 to 0.459 N, and post-bump cell-band power falls by 14.6 dB. This is a matched development example, not the worst fresh test.*

The recurrent learned controller remains research evidence: on earlier mechanics it passed **700 nominal, 30/32 fresh and 3/4 long tests**, with one travel stop. Its results do not transfer to this pad; retain deterministic control for the first prototype.

#### Recommendation

For the coil and soft pad, start with tested **force feedback plus estimated-velocity damping**: **10 N correction per N force error** and **200 N·s/m velocity correction**, with no integral or extra inertia feedforward. Acceleration still reconstructs contact force. The screw comparison uses **0.5 N/N, 100 N·s/m and a 60 Hz nut servo**. Identify installed pad/cell/driver response before adopting or changing gains.

### Robot commands and recentering

#### Intro

A robot correction can arrive after the force condition that triggered it has disappeared.

#### Question

How do we avoid turning recentering into delayed positive feedback?

#### Results

The reference recentering controller permits **100 Hz commands** only after stable force: a **0.08 N RMS variation gate** resets a **100 ms dwell**, alongside a slower stability condition. Its predictor retains issued commands, approximately 80 ms delay and 60 ms robot response. Closing the gate does not cancel queued commands. Travel within **±7.5 mm** about the midpoint incurs no centering cost.

#### Recommendation

Keep conservative, delay-aware recentering and the stability gate. Identify the robot's actual timing and test repeated bumps plus sustained drift. The long-test travel stop shows why a good short bump response is insufficient.

### Calibration, model qualification and prototype validation

#### Intro

The simulation tests a stated physical and sensing model, not every disturbance from a real sanding machine.

#### Question

Which checks turn this development recommendation into an accepted system?

#### Results

The new pad model assumes fixed orientation, translation, known tool mass and initially loaded contact. Stiffness is linear within a guarded range; damping is a sensitivity. Nonlinearity, spindle rotation, angular inertia, frame flexibility and sanding vibration need assembled tests. Force noise is bounded uniform and acceleration noise Gaussian, normalized to a **3,177 Hz reference bandwidth**. The new grids use zero residual bias/gain error; they do not replace calibration stress tests.

Independent native Simulink command replays check the new physical plant. A screw replay retains a failed **32.18 µN** strict numerical guard, with matching force-failure outcome. A separate partial-contact diagnostic has timestep-sensitive waveforms. [Numerical checks and retained failures](/home/tommie/Projects/actuator-model/acf_simulink/experiments/sea_pad_dynamic_20261006/README.md) distinguish these from qualification. Compact metrics and selected traces replace bulk histories.

#### Recommendation

Measure assembled masses, modes, friction, current response and sensor timing; calibrate axial load response, off-axis sensitivity and acceleration alignment. Target **≤2 µs relative timing residual**. Compare reconstructed force with an independent reference on the scoped bench setup through repeated bumps and long drift. Then test powered sanding and the deferred drag extension. Aim for **≤0.45 N peak error**, leaving 0.05 N reserve.

The [native pad-model guide](/home/tommie/Projects/actuator-model/acf_simulink/generated/sea_pad_dynamic_20261006/GETTING_STARTED.md) explains opening its physical command replay. The [study recipe](/home/tommie/Projects/actuator-model/acf_simulink/experiments/sea_pad_dynamic_20261006/README.md) documents the closed-loop experiments.

## Recommendation

The complete sanding requirement remains **unqualified**. The soft-pad fast-loop results provide useful margin for development, while long drift, calibration and powered sanding remain acceptance work.

**Build the first prototype around a voice coil and custom sensing/control PCB.** Keep the prescribed **20 N/mm undamped upper spring**, rigid B–S mounting and selected **77 mm × 5 mm pad geometry**. Choose approximately **25 N/A**, **25 mm stroke**, **55–60 N continuous hot force** and **85 N peak force**, with a **48 V-rated bipolar driver delivering at least 32 V usable output**. Verify actual pad stiffness and provide cooling for the modeled **37 W winding-loss case**.

Integrate **load-cell amplification, filtering, ADXL354C, ADS131M04 and a programmable MCU** on a PCB mounted on S. Connect the **one-axis B–S cell** and **S–A position sensor**; acquire force and acceleration at **30 ksample/s**. This gives us a configurable, calibratable control platform with fewer separate sensing assemblies. Keep the hot power driver on A and defer multi-axis drag separation.

Start with the tested coil force/velocity controller at **10 kHz**, with the robot fixed. Integrate and separately test stability-gated **100 Hz robot recentering** after the fast loop is verified. The coil's lower nominal error and simpler direct actuation justify this development choice; the screw remains the lower-heating comparison if thermal tests make the coil impractical.
