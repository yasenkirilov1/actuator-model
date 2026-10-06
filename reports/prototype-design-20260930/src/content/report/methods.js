export const modelExplanation = `## The selected mechanism keeps the moving tool side rigid

The physical chain is **robot boundary R → attachment → body A → actuator → holder B → attachment → tool S → workpiece**. In the selected candidate both attachments are rigid: A follows R, and B is fixed to S. The actuator gap remains free to move. This removes the two passive spring/damper links; it does not lock the actuator or leave the tool disconnected. The moving holder B is 0.6 kg and tool S is 1.4 kg, giving **B+S = 2 kg**. Robot-side A adds 0.4 kg.

Positions increase away from the workpiece. Let r be robot position, y surface/tool position, xA and xB the actuator attachments, and g = 9.81 cos(angle) the axial gravity component. Reported travel is 12.5 mm plus the change in gap xA − xB; allowed travel is 0–25 mm. At 0° gravity assists contact; at 90° it has no axial component. Each run has a fixed orientation and starts at its corresponding loaded equilibrium.

The broader model also supports compliant attachments. Their forces and mass balances are:

    q1 = q10 + k1*(y - xB) + c1*(ydot - vB)
    q2 = q20 + k2*(xA - r) + c2*(vA - vR)
    mA*aA = Fa - q2 - mA*g
    mB*aB = q1 - Fa - mB*g
    mS*aS = Fc - q1 - mS*g

Here q1/q2 are lower/upper attachment forces, Fa is transmitted actuator force after sliding friction, and Fc is true contact force. Stiffness is in N/m and damping in N·s/m. Reference loads are q10 = Fref − mS g, q20 = Fref − (mA+mB+mS)g and Fa0 = Fref − (mB+mS)g. Native Sum, Gain and Integrator blocks execute the balances; the masses do not receive arbitrary fractions of actuator displacement.

Rigid variants enforce xA = r and xB = y, then calculate reactions q2 = Fa − mA(g+rddot) and q1 = Fa + mB(g+yddot). The selected plant consequently reduces to **Fc = Fa + 2(g+yddot)**. Spring stiffness is not made artificially large to achieve this constraint. Zero external damping leaves the coil's modeled Coulomb and viscous friction in place.

The bump is h × 64q³(1−q)³ for q = (t−start)/duration between 0 and 1. Its 1 mm height, 10 mm width and 100 mm/s scan speed give a **100 ms crossing**, 34.346 mm/s peak tool speed and acceleration from approximately +1.92 to −2.40 m/s². The controller receives neither this profile nor its derivatives or clock. While contact holds, the plant imposes tool motion and calculates Fc. Negative required reaction or a travel stop terminates the run as failed; later free flight and impact are not simulated.

The preliminary PDF's heavy-tool cases used S = 2 kg plus B = 0.6 kg. The present tool side totals 2 kg. The earlier report also distinguished static ±0.5 N accuracy from transient excursions; this study requires the band throughout the recorded bump. Those differences prevent a direct performance comparison without matching mass, disturbance and requirement.`;

export const controllerExplanation = `## Force and relative position reconstruct contact force without surface preview

The selected controller receives the **gravity-tared B–S load-cell reading** and **tool S position relative to body A**, together with its own issued commands. With B rigidly attached to S, S−A displacement is the magnet-gap displacement with opposite sign. This equivalence disappears when the lower attachment is compliant. The modeled position noise is 1 µm RMS; it is not an absolute-position or calibration guarantee.

The fast loop runs at **10 kHz**. Force and position each have a known 100 µs measurement delay; the modeled force-filter and current-response time constants are also 100 µs. Sampling, transport delay, filtering and drive response are separate quantities. A sensor that outputs at 10 kHz but adds an unknown millisecond-scale digital filter would not match this tuning.

A discrete observer estimates nine physical state components: lower/upper link deflection and velocity, gap and velocity, winding-current deviation, surface acceleration and robot-acceleration residual. Filter/delay memory adds internal states. Rigid links constrain their deflection states; these are not nine freely moving mechanical parts. No measured acceleration, surface preview or winding-current readback enters the force algorithm.

Force and position correct the estimate each sample. A native predictor obtains nominal robot motion from **commands already issued**, using the configured robot delay, speed/acceleration bounds and response. A residual state accommodates disagreement. It receives no actual robot-state truth. This extra command history matters because relative position alone cannot separate tool motion from robot motion. The result depends on identifying the robot response accurately enough; mismatch tests address that assumption separately.

The observer predicts the actual cell equation, including the known downstream mass, then reconstructs contact force. For rigid links, Coulomb friction changes force directly. The native correction subtracts estimated Coulomb force both in the force-filter prediction and in reconstructed contact force. Viscous friction is already represented in the linear force row. Consistent correction prevents friction from being mistaken for a change in tool acceleration or winding current.

The observer supplies estimates to the feedback law. Integral action regulates **estimated contact force**; estimated current, acceleration and friction provide correction/compensation. For the selected nominal rigid model, the unclipped law is:

    Ihold = (Fref − 2*g)/Kf
    eta = integral(Fref − Fcontact_hat)
    i_cmd = Ihold − 0.58413293*(i_hat − Ihold)
                 − 0.15841329*aS_hat + 0.05*friction_hat + 1.26410981*eta

Current is in A, acceleration in m/s², friction in N and eta in N·s. This expression has no separate explicit proportional contact-force term; current/acceleration correction contributes feedback. The Observer menu label selects this estimator/controller combination. It is not a competing basic feedback law or a literal force PID. The request is limited to ±3.5 A with integral anti-windup; winding current and 24 V clipping remain independent physical states/limits. A physical drive still needs calibrated current regulation.

The slow loop recentres estimated actuator gap. Its force gate uses filtered reconstructed contact-force error: magnitude within 0.2 N, trend below 2 N/s, continuously for 100 ms. A separate **100 ms gate filter** smooths that decision; it does not change the fast 100 µs force-measurement filter. The nominal robot has 80 ms command delay and 60 ms response. Effective gain is the smaller of the requested 0.8/s and 0.25 divided by delay + response + dwell + gate-filter time, giving approximately **0.735/s**. The predictor uses identified 80/60 ms timing, with 60 ms residual decay and residual jerk power 0.0002.

The gate opened in **all 350 final sharp-bump cases**, with readiness in 97.506% of clean samples and 20.698–33.417% of noisy samples. The earlier 20 ms gate filter never became ready in the noisy cases. Separate flat-surface tests at 50 N/90° started 4 mm off centre. With known 80 ms delay, capped clean/noisy 20 s runs passed, with peak force errors 0.0644/0.2536 N. With known 3 s delay, ungated and gate-only clean runs reached a travel stop; capped clean/noisy 60 s runs passed, at 0.0301/0.2497 N. The noisy run remained **2.05 mm off centre at 60 s**, demonstrating slow active recentering. The predictor was updated to each known delay. Queued commands and independent nominal motion remain; these passes do not prove tolerance to unknown timing or arbitrary delay.

Native Discrete State-Space, Gain, Sum, Integrator and Saturation blocks execute this control. MATLAB prepares matrices before a run using checked Riccati and state-feedback synthesis; it does not integrate a hidden plant. Smaller forceScale_N emphasizes tracking and smaller currentScale_A penalizes current request. These are design weights, not physical component ratings. PI/PID remain selectable comparisons, but their earlier near-contact passes are counterfactual for the specified upstream cell.`;

export const springExplanation = `## Passive springs are comparison designs, not a requirement of the selected candidate

Both selected attachments are rigid, so the candidate uses **zero passive springs and zero external dashpots**. Its combined spring length is therefore zero. Guidance, actuator, sensor, stops and mounting thickness still occupy space; passing the 100 mm spring-only condition does not prove the complete flange fits in 100 mm.

The comprehensive compliant study remains useful for assessing alternatives. Each elastic link was screened as **two guided, opposed compression springs**. If one spring has rate ks, the pair has effective rate k = 2ks. Common preload P compresses both; differential preload carries the signed holding load. Compression is P/ks ± (q0/k+δ). During motion one spring shortens as the other lengthens; spring force is q0+kδ, with damper force c times relative velocity added separately.

Checks cover the entire recorded motion: seating compression of at least 0.25 mm, manufacturer's minimum working length plus 0.5 mm clearance, cyclic excursion, conservative individual force envelope and free-length tolerance. Both combined free length and maximum combined installed length must separately stay within **100 mm**. Common preload can be post-screened on a trace because it changes seating/compression while leaving net physical link load unchanged.

Four catalogue pairs per link and damping values 0, 5, 15, 30 and 60 N·s/m gave 400 designs. Four force/orientation corners produced 1,600 native runs. Twenty-five designs met geometry at every corner; all used the strongest tested pair. Those 25 received 100 more noisy/delayed cases, with 15 passing under the fixed study controller and **near-contact sensing**. This is counterfactual evidence for the actual B–S cell, not a qualification that transfers to it.

The strongest example uses individual 9.873 N/mm springs with 18.1 mm free length, giving 19.746 N/mm per link. Two opposed pairs have 74.36 mm conservative free length including tolerances. [Gutekunst's catalogue](https://www.federnshop.com/download/pdf/Gutekunst-Federnkatalog-2013-E.pdf) grounds the envelopes; its [dynamic-rating conditions](https://www.gutekunst-net.de/ebook/Gutekunst-Federnkatalog-2013-E/files/assets/basic-html/page27.html) require suitable spring preparation. Per-bump cyclic passes do not establish fatigue under repeated changes of force/orientation: the selected four-spring study fallback spanned about 3.28/3.97 mm between operating points against a 3.18 mm catalogue cyclic envelope.

Damping is an equivalent viscous target, not inherent steel-spring damping. At 50 mm/s, 60 N·s/m gives 3 N and 0.15 W instantaneous dissipation. Real devices need measured force–velocity curves. [Airpot's FAQ](https://www.airpot.com/faqs/) describes start-up/vibration departures from ideal viscous behavior. Adding passive compliance can provide other design benefits, but it changes dynamics, packaging and sensing and requires requalification.`;

export const forceDefinition = `## The qualified quantity is axial force, not automatically surface-normal force

Fc is the scalar reaction along the flange axis. The preliminary PDF calls the reference normal force while using one-dimensional axial equations. The sharp spatial bump has maximum slope 0.34346 and local-normal tilt 18.96°, making the distinction material at high force.

With negligible tangential load, normal force is Faxis/cos(tilt). At maximum slope, holding 50 N axial corresponds to **52.87 N normal** and approximately **17.17 N lateral reaction**. Holding 50 N normal would instead need 47.29 N axial there. These are geometric calculations, not simulated controller results.

The force/orientation qualification therefore applies to **axial force under the stated contact assumptions**. It does not establish ±0.5 N true-normal regulation on a curved workpiece. A multi-axis sensor can measure a resultant without profile preview when tangential load is negligible. Grinding or sliding introduces tangential force, so resultant magnitude alone does not isolate the normal component; a normal-direction estimate and lateral contact model would be needed.

Tool tilt, pad area, distributed pressure, contact moments and side-load-dependent guide friction are absent. Guide and sensor sizing must still account for these loads. This extension concerns the force definition itself; tighter axial controller tuning cannot remove it.`;

export const sensorExplanation = `## Known tool mass and tare are useful, but inertia still needs reconstruction

The actual cell is rigidly mounted **between B and S**, with the full **1.4 kg tool downstream**. Before tare its axial force is Fc − 1.4(g+yddot). After an exact static gravity tare it is **Fcell = Fc − 1.4 yddot**. Zeroing the tool weight removes gravity, not acceleration. The sharp bump can leave a **3.36 N dynamic difference**, even with perfect zero and known mass. A force-only loop holding Fcell constant would be regulating a different quantity from contact force.

In the selected rigid mechanism, Fa denotes actuator effort after sliding friction. The two force relations are:

    Fc = Fa + 2*g + 2*yddot
    Fcell = Fa + 2*g + 0.6*yddot     after exact tool-gravity tare

The cell retains an acceleration term from B, and relative position supplies motion information. The observer uses both, its modeled drive current and issued robot-command predictor to estimate acceleration and reconstruct Fc. No extra accelerometer or exposed winding-current sensor is used for the simulated qualification. That result remains conditional on identified mass, friction, drive response, robot response and calibrated sensing.

The selected position coordinate is **S relative to A**, measured on the moving tool and actuator frame. Rigid B–S makes an internal magnet-gap measurement equivalent up to sign. On a compliant lower link, magnet position and tool position differ; a position sensor must measure the coordinate actually used by the controller. Mounting deformation, guide tilt and sensor offsets need testing on the assembled flange.

Calibration tests expose a narrower envelope than the nominal force margin suggests. An **extra unmodeled 100 µs force delay** destabilized the candidate before the bump and caused contact loss. Force bias of **±0.05 N** and identified downstream-mass error of **±1%** also failed. A separate earlier ±0.1/±0.2% mass-coefficient study passed 64 cases, changing only the identified inertia-compensation coefficient: actual masses and exact gravity tare stayed nominal, with a 20 ms gate filter. This does not qualify joint mechanical/final-gate tolerance. Earlier near-contact tolerance results cannot be transferred to the actual cell.

Timing matters as much as micrometre resolution. The candidate uses 10 kHz force/position sampling, known 100 µs delays, 0.02 N RMS force noise and 1 µm RMS position noise. Raw second differencing of independent 1 µm samples at 10 kHz would yield about **245 m/s² RMS acceleration noise**, far above the bump acceleration. The controller therefore uses model-based filtering rather than differentiating raw position twice.

A useful algebraic sensitivity follows if actuator effort were independently known: reconstructing Fc gives coefficients 3.33 on cell force and −2.33 on effort plus gravity. A 0.1 N allocation would allow approximately **0.03 N cell bias** in this simplified reconstruction; it is not the full closed-loop transfer function. Independently measured tool acceleration offers the physical alternative **Fc = raw cell + 1.4*(aS+g)**, removing Kf from force reconstruction. That correction would need specific-force error below **0.071 m/s²** for a 0.1 N allocation, before other errors. It is an untested controller alternative or validation instrument. Current readback alone still needs Kf to infer actuator effort.

[ATI's sensor guide](https://www.ati-ia.com/Library/documents/ATI_Overview_Catalog.pdf) distinguishes resolution and accuracy and requires range to include tooling/dynamic loads. Its [uncertainty definitions](https://www.ati-ia.com/library/documents/FT_FAQ.pdf) show why percent-of-full-scale accuracy is not percent of the applied force. Noise, zero/scale bias, drift, cross-axis error and latency need separate specifications. The untared cell can also see both force signs at low contact force with vertical tooling.

A sensor upstream of all 2 kg rigid tool-side mass has a tared signal Fc − 2*yddot. Substituting the rigid mass balance gives Fa + 2*g: its signal contains no tool-acceleration information beyond actuator effort. Without independently known robot motion, relative position cannot identify common robot/tool acceleration. The issued-command predictor adds conditional motion information, but that different sensor position requires its own observability and qualification checks. Near-contact sensing with negligible distal mass is another counterfactual design; its earlier roughly 0.30 N result must not be presented as the specified cell's performance.`;

export const coilExplanation = `## Size the coil for simultaneous force, current slew and continuous heating

The electrical model is **Fa = Kf i − sliding friction** and **V = Ri + L di/dt + Kf × gap velocity**, in consistent SI units. The candidate provision is approximately **20 N/A, 4 Ω, 3 mH, ±3.5 A and ±24 V**, with 25 mm modeled travel and a **100 µs current-regulator target**. The passive L/R time constant is 0.75 ms; active regulation aims faster but remains voltage-limited.

A 100 µs target is not a promise of a full 0-to-2.5 A step in 100 µs. Inductance alone would require 75 V for that initial slew, before resistance/back EMF. Qualification starts at loaded equilibrium and tests the finite-acceleration bump, with actual winding current and clipped voltage integrated. Larger force commands from rest are a different test.

At 50 N/90°, holding current is 2.5 A for Kf = 20 N/A, and cold copper loss is **25 W** at 4 Ω. Exact rigid tracking needs up to 54.8 N transmitted effort before friction, or 2.74 A nominally. A 10% lower force constant would need 3.04 A before friction. This explains the 3.5 A provision over the earlier 3 A baseline; the executed tolerance/sensitivity tables determine the supported margin. At 5 N/0°, holding current is negative, so bidirectional force/current capability matters even though contact force stays positive.

Voltage must cover resistance, inductive slew and back EMF **at the same instant**. Adding their separate maxima overstates the requirement; ignoring noisy current slew understates it. A higher bus voltage cannot fix unknown sensor latency or an incorrect force measurement. Kf, resistance, inductance, stroke, moving mass and cooling also cannot be selected independently as though every sweep combination were a purchasable winding.

The final dense grid reached **2.75246 A and 11.81564 V** without clipping, but electrical headroom did not ensure robustness. An **unknown 10% lower Kf** produced approximately **14.85 N** peak error; an **unknown 25% slower current response** produced about **0.575 N**. Even unknown **−0.1%/+0.1% Kf** failed every tested noisy high-load corner, with worst errors **0.729/0.839 N**. No nonzero unknown-Kf allowance was demonstrated. An identified alternative coil retuned before use is different from unexpected physical drift. Force constant versus position/temperature and regulator dynamics therefore need bench identification; the nominal two-sensor result is not robust prototype qualification.

The initial motor constant Kf/√R is 10 N/√W. A higher value reduces resistive loss for the same force, subject to stroke and thermal constraints. [Novanta's motor-constant guidance](https://novanta.com/robotics-automation/technical-paper/motor-constant-matters/) emphasizes hot operating conditions and measuring thermal resistance in the housing. [Maxon's motor formulas](https://www.maxongroup.co.kr/medias/sys_master/8805405622302.pdf) give copper's approximately 0.0039/K resistance coefficient. A 100 K rise takes 4 Ω to about 5.56 Ω and 2.5 A loss to **34.75 W**, before force-constant changes. No thermal states or continuous duty limit are simulated.

An [official Sensata drawing](https://www.sensata.com/sites/default/files/a/sensata-linear-actuator-sensor-LAS-28-53-drawing.pdf) demonstrates a roughly 20 N/A, 3 mH, 25 mm-stroke actuator envelope. Its 0.755 kg moving-coil assembly exceeds the present 0.6 kg B allocation. It is a plausibility example, not a part recommendation; actual moving mass must fit the allocation or be modeled again. A stationary winding with moving magnet is worth evaluating for heat removal, wiring and moving mass, but this simulation does not qualify its packaging.`;

export const alternativesExplanation = `## The voice coil fits this model, while other actuators need matched mechanisms

A voice coil offers direct bidirectional force/current control without reflected screw inertia or backlash. That makes it a strong starting point for this short-stroke force loop with the actual cell and relative-position measurement. It is not a complete flange: guidance, cooling, calibrated sensing, current drive and overtravel protection remain necessary. The simulated candidate needs no passive spring or external damper; whether the prototype should add compliance for another purpose is a separate design decision.

The screw comparison retains native motor electrical dynamics, reflected inertia, speed/current limits and an outer force controller. With lead ell = pitch/(2π), rotor inertia contributes J/ell² equivalent axial mass. The original 5 mm/rev and 2e−5 kg·m² example adds about **31.6 kg**. Greater lead reduces that reflected inertia while reducing mechanical advantage and increasing torque demand. [Thomson's screw selection material](https://www.thomsonlinear.com/en/products/ball-screws?solution=motioneering) explains the lead/drive context.

Its parameter/gain tests assess that screw model, not all possible servos. The older favorable coil/PI/PID comparisons with near-contact sensing are also counterfactual for the actual B–S cell. Transmission flexibility, backlash, breakaway friction, moving inertia and sensor layout need matching before concluding a screw is better or worse for the prototype.

An ironless direct linear servo has related advantages, with different stroke, commutation and packaging options. Force ripple, moving mass, encoder accuracy and cooling need evaluation; it is not separately simulated. [Aerotech's application guide](https://www.aerotech.com/wp-content/uploads/2020/12/linear-motors-application-en.pdf) covers these selection issues.

Pneumatic active flanges are commercially credible: the [FerRobotics ACF-K datasheet](https://www.ferrobotics.com/app/uploads/2025/03/FerRobotics_ACF-K-Datasheet-2025-EN-web.pdf) specifies compressed-air supply and force compliance. Chamber compressibility, valves, pressure sensing and friction need a different plant. [Festo's force-control documentation](https://www.festo.com/media/catalog/202774_documentation.pdf) explicitly separates pressure-control repeatability from effective workpiece force and friction. Neither pneumatic nor hydraulic performance is qualified by these coil/screw runs.

The ideal-position variant has finite modeled speed, acceleration and force limits. It is a comparison reference, not a realizable servo or proof that a position-controlled actuator would reproduce the force result.`;

export const limitationsExplanation = `## A pass qualifies the recorded axial cases under identified assumptions

The final native grid passed **350/350 cases**: 5–50 N in 5 N increments, 0–90° in 15° increments, and one clean plus four noisy conditions per point. Worst axial error was **0.426347 N at 50 N/60°, seed 29**, leaving **0.073653 N** to the ±0.5 N limit. A repeat of that case at 25 µs integration steps also passed, at 0.425960 N; the normal grid uses 50 µs steps with fixed 100 µs measurement/control clocks. This is finite nominal evidence, not proof for all parameters or hardware. Other disturbance shapes/speeds and target/orientation transitions require separate tests.

Exact tool-gravity tare is an assumption, not dynamic inertial compensation. The actual measurement row is included in the observer, but unknown distal-mass error, friction, force constant, position calibration and latency can corrupt reconstructed force. The recorded bias, unknown-delay, mass and drive mismatches show that **nominal model qualification is not robust prototype qualification**. Issued commands predict robot motion conditionally; real communication jitter, structural motion and response changes are not measured by that predictor. Separate mismatch tests are more informative than repeating an identified design sweep. The robot boundary follows an imposed delay/response/limit model: contact and actuator forces do not dynamically deflect that robot. Real robot stiffness and load-dependent tracking therefore remain unqualified, even when the command predictor matches the imposed response.

Rigid unilateral contact prescribes tool motion while reaction is nonnegative, then stops at contact loss or a travel limit. Real pad/workpiece compliance, contact friction, moments and impact peaks are omitted. The both-rigid candidate has no modeled passive spring recovery: guide/stops, off-contact operation and power-off behavior need a separate physical design and test.

The coil model omits thermal states, nonlinear force constant over stroke/temperature, magnetic saturation, PWM/deadtime, eddy-current effects and cable forces. Spring alternatives use linear coefficients without buckling, spring mass, rubbing, hysteresis or full-life fatigue. External damping omits fluid dynamics and temperature dependence; the screw omits detailed backlash and transmission compliance. Relative tool/frame flexibility and lateral loading are absent from all variants.

Observer coefficients are regenerated for identified parameter changes in design sweeps. Unknown-parameter cases change the physical model while retaining the controller's assumptions. Independent one-factor passes do not imply joint tolerances. Another 54 process-noise/bias-state experiments found no all-passing cohort, so the selected nine-state estimator remains unchanged. Nominal passes with exact drive and robot timing set identification requirements rather than guaranteeing arbitrary components.

Fixed-step refinement, saved-model reopen checks, algebraic-loop inspection and replay comparisons support reproducibility. They do not validate omitted physics. The report distinguishes synthesis rejection, tracking error, contact/travel failure and spring-envelope rejection. Missing cases are not zero error, and counterfactual near-contact passes remain separate from the actual-cell qualification.`;

export const prototypePlan = `## Build the simplest candidate, then identify the quantities its estimator needs

Start with **rigid R–A and B–S attachments**, the voice-coil candidate and a guided 0–25 mm travel envelope. There is no spring/damper purchase requirement for that simulated topology. Weigh A, B and S separately; the complete moving assembly must fit B+S = 2 kg. Verify guide alignment, tool/frame rigidity, sensor force path and full stroke/overtravel packaging. Adding springs or a flexible mount changes the model rather than supplying a free safety margin.

Keep the B–S cell and identify its **1.4 kg downstream mass**. Calibrate raw zero, scale and orientation-dependent gravity tare with the actual tool. Check signed force range, thermal drift and cross-axis response. Position must describe S relative to A; with a rigid B–S link an internal magnet-gap sensor can provide the equivalent coordinate. Measure position scale, bias, repeatability, RMS noise and end-to-end timing, rather than specifying micrometre resolution alone.

Use the candidate's **10 kHz force/position rate**, 0.02 N RMS force-noise assumption, 1 µm RMS position-noise assumption and **known 100 µs measurement delays** as starting bench targets. Verify actual filter phase/response as well as sample output rate. These are modeled conditions; the sensitivity tables show how much calibration/timing error was actually tolerated. No extra accelerometer or current-readback input is used by the selected simulated force algorithm. Independent acceleration/current measurements can nevertheless reduce reliance on the inferred drive/motion model or provide validation; any changed feedback scheme needs its own simulation and bench qualification.

Identify Kf over usable travel, cold/hot R, L, friction and actual current-loop response. A roughly **20 N/A, 4 Ω, 3 mH, 3.5 A, 24 V** provision is a starting envelope, not a guaranteed part. Measure winding current/voltage on loaded transients and the thermal response at **50 N/90°**, where the model predicts 25 W cold holding loss. Confirm continuous heat removal separately from peak force/current. The drive must regulate both current signs and handle back EMF; its command alone does not prove actual effort.

Measure robot command delay, response, speed and acceleration limits. Configure the issued-command predictor from those measurements. Retain the reconstructed-force dwell gate and conservative recenter gain; check long runs with repeated bumps and parameter mismatch, not only one apparently settled trace.

Finally compare the estimator with an **independent contact-force measurement** during commissioning. Repeat low-force tests first, then 5/50 N, 0/90°, intermediate points, repeated disturbances, heating and measured tolerances. Contact loss, a travel stop or force-band violation is a failure. If the requirement is local surface-normal force, add the direction/lateral-contact measurements before treating an axial result as a normal-force pass. These tests determine whether the identified model is sufficiently accurate for the real flange.`;
