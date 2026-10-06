# Contact-normal force and variable tool-mass physics review

This is a derivation and observability review for the final model design. The
numerical sizing calculations here are analytical checks, **not completed
native Simulink qualification runs**. The companion JSON contains the
calculation inputs, equations, results and source hashes at review time.

## Requirement and mass definitions

The controlled quantity is the resultant pressing force normal to the
sanding surface, denoted `N`, over 5–50 N with an allowable tracking error of
±0.5 N. The sharp surface test is 1 mm high and 10 mm wide, traversed at
100 mm/s. The tool mass downstream of the B–S cell is variable:

```
mS = 0.1 ... 2.0 kg
mB = 0.6 kg
mA = 0.4 kg, additional robot-side body mass
M = mB + mS = 0.7 ... 2.6 kg
```

Each known tool mass needs consistent updates to the physical mass, cell
downstream mass, gravity compensation and identified observer mass. This
range does not preserve a fixed 2 kg B+S total. Parameter scheduling based
on a known installed tool is different from proving a fixed controller
tolerant to unknown mass changes during operation.

## Consistent local contact force

Use local flange coordinates with `+z` away from the workpiece and `+x`
along the scan. For a stationary profile `z=h(x)`, define:

```
s = dh/dx
c = sqrt(1+s^2)
n = [-s, 1]/c                  outward local surface normal
t = [1, s]/c                   positive scan tangent
```

For a positive scan and a passive tangential drag `T=mu*N`, the reaction on
the tool is:

```
R = N*n - T*t
Rx = -N*(s+mu)/c
Rz =  N*(1-mu*s)/c
N  = dot(n,R) = (Rz-s*Rx)/c
```

`N` is also the equal-and-opposite pressing force experienced by the
workpiece. The tangential component is separate. The contact-power check is
`dot(R,[vx,s*vx]) = -T*vx*c <= 0`, as required for passive drag.

The above projection does not require a known friction coefficient when
both components of `R` are measured or estimated. This is preferable for a
sanding process whose tangential force changes with abrasive condition,
tool speed and material. A scalar axial cell plus slope instead requires
`N=Rz*c/(1-mu*s)` and therefore requires identified tangential behavior.
Unknown `mu` and unknown `N` cannot both be determined from one axial force
component, even if slope is exactly known.

For a 50 N normal target, the maximum slope gives:

| Tangential ratio `mu` | Rising-face `Rz` | Falling-face `Rz` |
|---|---:|---:|
| 0 | 47.289 N | 47.289 N |
| 0.1 | 45.664 N | 48.913 N |
| 0.3 | 42.416 N | 52.161 N |
| 0.5 | 39.168 N | 55.409 N |
| 1.0 | 31.047 N | 63.530 N |

These ratios are illustrative sizing/stress inputs. They are not measured
properties of the intended sanding process or established material bounds.

## Vector B–S load-cell balance

Let `g_vec` be gravitational acceleration, including its sign, and `aS` the
tool's acceleration vector. Define the load-cell signal as the load from S
transmitted back toward B. The distal-tool free-body balance is:

```
L_raw = R + mS*g_vec - mS*aS
L_tared = L_raw - mS*g_vec = R - mS*aS
R_hat = L_tared + mS*aS_hat
N_hat = dot(n_hat, R_hat)
```

Gravity tare removes the static term, not dynamic inertia. With the modeled
constant tangential scan, `aSx=0`, so `L_tared_x=Rx`; the axial channel needs
the tool acceleration correction. A changing scan speed instead needs its
own known/estimated tangential acceleration compensation.

A vector cell must carry **all** distal-tool forces. A guide, cable or other
parallel force path bypassing the cell would invalidate these equations.
Both force channels must include their actual filtering, sampling, noise,
bias and delay. A load cell with only an axial sensing element does not
supply a lateral measurement merely because the mount is rigid.

For each fixed modeled orientation, exact vector tare can remove both
gravity components. Hardware operation through orientation changes needs
orientation-aware gravity compensation or a new tare, with its uncertainty
included in the force budget.

## Both-rigid axial mechanics and conditioning

Let `Fa` be the actual axial force transferred by the actuator after guide
friction, and `g_axis` the positive magnitude of gravity projected downward
along the axis. With both attachments rigid:

```
Rz = Fa + M*(g_axis+aSz)
L_tared_z = Rz-mS*aSz = Fa + M*g_axis + mB*aSz
```

The upstream cell's dynamic acceleration coefficient is `mB=0.6 kg`, not
`M`. Eliminating acceleration yields:

```
Rz = (M/mB)*L_tared_z - (mS/mB)*(Fa+M*g_axis)
```

For deviations about a loaded equilibrium:

```
delta_Rz = (M/mB)*delta_L_tared_z - (mS/mB)*delta_Fa
```

Thus a reconstruction relying on actuator-force prediction becomes more
poorly conditioned as distal tool mass rises. This algebraic sensitivity is
not itself a certified closed-loop calibration tolerance; filtering,
estimation and feedback determine the actual dynamic response.

| S mass | B+S mass | Cell coefficient `M/mB` | Actuator coefficient `mS/mB` | Largest tool inertial term |
|---|---:|---:|---:|---:|
| 0.1 kg | 0.7 kg | 1.167 | 0.167 | 0.24 N |
| 0.5 kg | 1.1 kg | 1.833 | 0.833 | 1.20 N |
| 1.0 kg | 1.6 kg | 2.667 | 1.667 | 2.40 N |
| 1.4 kg | 2.0 kg | 3.333 | 2.333 | 3.36 N |
| 2.0 kg | 2.6 kg | 4.333 | 3.333 | 4.80 N |

Reducing S improves the measurement problem but does not remove the
specified 50 N holding-force requirement. At a horizontal flange axis,
50 N still needs 2.5 A with a 20 N/A coil and dissipates 25 W at 4 ohm.

## Causal local-normal estimation

For the point-contact kinematics at nonzero scan speed:

```
vSz = s*vx
s_hat = vSz_hat/vx_hat
```

When both attachments are rigid, the actuator-gap velocity satisfies
`uDot=vAz-vSz`, so:

```
vSz_hat = vAz_hat-uDot_hat
```

The existing `Issued-command robot predictor` already computes
`[predicted_position, predicted_velocity, predicted_acceleration]`. Its
root connection currently selects acceleration; its velocity output can be
exposed without supplying robot truth or surface preview. The current
observer's state 9 estimates robot-acceleration residual, so a velocity
estimate robust to robot mismatch should include residual-velocity dynamics
or a controlled integration of that residual with suitable initialization.
Using nominal predicted velocity alone leaves an unmodeled slope error when
actual robot motion differs from the prediction.

For a compliant B–S link, `vSz_hat=vAz_hat+lowerVelocity_hat-uDot_hat`.
The lower-link state is essential in that case; S-relative-A position and
magnet gap no longer have identical kinematics.

Only estimated tool motion and issued robot/scan commands may enter this
normal estimator. Plant profile height, true derivative, known bump clock,
true robot velocity and true acceleration must remain validation-only
signals. Commands are predictions of scan velocity, not measurements of
actual scan motion. Tangential tracking mismatch needs a separate bound or
measurement in hardware.

Division by scan speed needs a low-speed guard. Near a stop, retain the last
credible local normal or use a separately known robot/workpiece orientation;
there is no slope observation from a stationary point's height alone.

## Synchronization and practical error budget

The vector reaction estimate and normal estimate must refer to the same
time. Mixing a present predicted axial force with a delayed lateral force
produces an artificial normal-force error. For this bump, the slope rate
reaches 24 per second and lateral force can change at approximately
1,200 N/s at a 50 N target. A 0.2 ms lateral/axial timing mismatch can
contribute roughly 0.08 N after projection. This is a sizing estimate;
native joint-channel timing tests must determine the actual peak.

Suitable implementations either align all components to one timestamp and
control for the known delay, or causally predict all components to the
present using measurement-history states. Extra lateral-force filtering
cannot be omitted from the plant merely because the projection equation is
algebraic.

Normal-direction error also mixes tangential load into the pressing force.
For a small angle error `dtheta`:

```
N_hat-N approximately -T*dtheta - N*dtheta^2/2
```

At 50 N and `T/N=0.3`, allocating 0.1 N to that first-order term permits
about 0.382 degrees normal-angle error. At `T/N=0.5`, it permits about
0.229 degrees. These are separate allocations, not a proven joint sensor
specification.

Similarly, a 0.1 N axial-inertia allocation gives the conservative bound
`abs(acceleration_error)<=0.1/mS`. This is 1 m/s² at S=0.1 kg but only
0.05 m/s², approximately 5.1 mg, at S=2 kg. Position resolution alone does
not specify acceleration accuracy or its bandwidth/phase. Independent
sensor bias, noise, scale error, timing and drift need a joint budget.

## Electrical sizing and higher voltage

For the corrected normal target, quasi-ideal axial actuator effort is:

```
Fa_required = N_ref*(1-mu*s)/sqrt(1+s^2) - M*(g_axis+aSz)
```

At 50 N, horizontal axis and S=2 kg, a dense analytical evaluation over
the bump gives the following maxima, before guide friction:

| Tangential ratio | Required maximum axial force | Current at 20 N/A |
|---|---:|---:|
| 0 | 56.240 N | 2.812 A |
| 0.3 | 57.303 N | 2.865 A |
| 0.5 | 59.026 N | 2.951 A |
| 1.0 | 65.325 N | 3.266 A |

These values show that the proposed ±3.5 A coil provision can cover the
analytical nominal effort envelope for these particular stress ratios, with
little additional force reserve in the most demanding example. They do not
show that the sampled controller tracks within ±0.5 N.

The voltage equation is `V=R*i+L*di/dt+Ke*uDot`. Higher voltage can improve
performance when this demand reaches the supply limit or when faster
current regulation needs additional transient voltage. It cannot directly
repair force-estimation error, uncertain drive force constant, sensor delay,
normal estimation or mechanical compliance. The existing 24 V nominal
case had about 11.8 V peak demand; a voltage-only change would therefore
leave its unclipped trajectory unchanged. Repeating the corrected full-mass
test with voltage/current-response pairs and hot-winding resistance is the
appropriate way to establish useful electrical headroom.

At constant current, increasing supply voltage does not increase `i^2*R`
force-holding loss; heating is set by current and winding resistance. A
100 microsecond current-response target is active regulation, not the
passive 0.75 ms `L/R` time constant of a 4 ohm, 3 mH winding.

## Minimal drive-force calibration option

On a genuine stationary, settled contact hold, the tared axial cell provides
a direct actuator-force calibration relation:

```
Kf_est = (L_tared_z - M*g_axis + friction_hat)/i
```

This requires negligible tool acceleration, settled regulated current,
known tare/mass/orientation, and current sufficiently far from zero. Useful
guards include a lower current threshold, low estimated velocity and
acceleration, a long current hold, force stability and physically credible
gain clamps. Freeze adaptation during surface motion. If only issued
current is available, steady current tracking is an additional driver
assumption; readback improves that assumption but does not calibrate Kf.

A single static point cannot separate Kf from an unknown force offset or
static guide friction. Multiple quasi-static positive/negative current
points, repeated over stroke and temperature, separate gain and offset more
reliably. Bench identification before running the high-bandwidth controller
is more defensible than expecting slow adaptation to rescue the first
milliseconds of an initially mismatched observer.

For an approximately constant identified Kf, the proposed mapping is valid:

```
i_physical_request = (Kf_nominal/Kf_est)*i_effective_request
i_effective_issued = (Kf_est/Kf_nominal)*i_physical_clipped
```

Enforce physical current limits after mapping. Feed the physically clipped
command converted back to effective units into both observer and antiwindup.
Holding-current and friction feedforward stay in effective units. Rapid
changes of Kf_est add unmodeled effective-current dynamics, so identification
must be slow or staged. Calibration must still be followed by native
uncertainty tests; it cannot be presented as broad robust qualification by
itself.

## Scope of the contact model

The consistent two-dimensional equations model a guided tool following one
effective contact point on a prescribed scan. They calculate pressing force,
tangential drag and lateral guide reaction without adding robot or pad
rotational dynamics. A finite sanding pad can bridge the bump and distribute
pressure over multiple normals. Its contact compliance, pressure field,
rotation, moments, guide side-loading effects and robot compliance require
separate physical modeling or prototype measurements. These are assumptions
of the final model description, not experiment outcomes.

## Selected compliant lower series link: physical audit

This addendum reviews [the selected lower-series candidate](inertial_mechanics_lower_series_v1_candidate_20260930.mat) and its private native source at `/home/tommie/.local/state/actuator-model/inertial-mechanics-selected-lower-series-20260930/lower_series_v1`. The SLX SHA256 is `a0c2e358503d51c53c630ca436f20c1a618c5d024f98dd46338ee328f6c94f81`. The source has upper attachment rigid, lower net stiffness19746 N/m and damping60 N·s/m, B0.6kg, P0.3 A/N, I20 A/(N·s), feedback-only Butterworth LP1000, and proper inverse-feedforward rolloff1500Hz. The selected common spring preload is33.6 N per spring. This is an independent read-only derivation and source/retained-signal audit; no additional native runs or shared model edits were made.

### Physical force path

The intended connection is:

```text
Actuator stage B
      |
      +-- opposed guided spring pair --+
      |                                |
      +---------- damper --------------+
                                       |
                              joining frame
                                       |
                              multi-axis cell
                                       |
                              distal tool S
```

The pair and damper act in parallel, then connect **in series with the cell**. All supports of S, including any lateral support, must transmit their forces through the sensing element. A separate guide or cable attachment crossing from the upstream structure directly to S would change the force balance. A noncontact position sensor can measure S relative to A without providing a mechanical bypass.

The raw B–S cell balance remains `Lraw = R + mS*g_vec − mS*aS`, independent of the soft upstream link. Thus calibrated, aligned cell force plus measured S inertia still reconstructs the reaction experienced by the workpiece. Mass above the sensing element is not part of the distal mass correction. Place the accelerometer on S and count the complete measured distal assembly in the known mass.

### Inverse formula and native audit

For axial `q=zS−zB`, positive outward S acceleration, and positive downward axial gravity magnitude:

```text
Lraw = Rz − mS*(g+aS)
(k+c*s)*q = Lraw
Fa = Rz − (mB+mS)*(g+aS) + mB*qdd
```

Here `Fa` is the net actuator force after actual guide friction. Winding force `Kf*i` additionally supplies that friction. About the loaded equilibrium, the controller's desired lower-load deviation is:

```text
deltaL_requested = deltaRz_requested − mS*measured_aS
inverse_current = (mB/Kf)*s²/(k+c*s)*deltaL_requested
                  * w²/(s²+2*zeta*w*s+w²)
```

The sign and downstream mass coefficient are correct. The selected two-pole rolloff makes this inverse causal and strictly proper for c60. The desired axial force comes from the native measured force-vector/friction-ratio geometry; this branch does not use the future bump profile. Its inverse coefficients use explicitly identified spring, damping, B mass and force constant, rather than silently reading a perturbed physical parameter as known.

The actual preparation fields are `identifiedStiffness_N_m`, `identifiedDamping_Ns_m`, `identifiedMassB_kg`, and `identifiedForceConstant_N_A`. The actual native coefficient arrays reproduce independently with zero residual. Their maximum pole radius is0.96762 and DC output is zero.

Replaying the retained worst short-run native lower-load input reproduces the extra current within3.77e−13 A. The logged requested lower load equals requested axial deviation minus identified S acceleration inertia exactly. The modeled spring-plus-damper force agrees exactly with its recorded load; the distal-tool balance agrees within7.1e−15 N. The recorded actuator force is already net of guide friction, and its B force balance agrees exactly; winding force minus explicit friction agrees within7.1e−15 N. These checks establish the equation and wiring in that retained source, not hardware performance.

### Low mode and finite load-cell compliance

The lower pair gives an open mechanical natural frequency28.872Hz and damping ratio0.276, or0.285 including the2 N·s/m guide coefficient. A massless joining frame in series with a64 MN/m cell gives:

```text
Zseries(s) = kc*(k+c*s)/(kc+k+c*s)
qB = L/(k+c*s) + L/kc
```

Its static effective stiffness is19739.91 N/m, a0.03084% reduction. The low-frequency effective damping is59.963 N·s/m. The complex cell-compliance correction is approximately0.0352% at the29Hz mode and0.884% at1500Hz. The massless finite-cell model adds a very fast real relaxation pole with time constant0.937µs; it does not create a separate1.64kHz oscillator. That is a justified low-mode approximation for a massless series node.

It does **not** establish that every physical load-cell or fixture mode has disappeared. A nonzero joining-frame/proximal-cell mass adds another degree of freedom. For a constrained S and cell stiffness64 MN/m, its high mode is approximately `sqrt(kc/mC)/(2*pi)`:

| Proximal joining mass mC | High-mode frequency | Approximate damping ratio from c60 |
|---|---:|---:|
| 10 g | 12.7 kHz | 0.0375 |
| 25 g | 8.05 kHz | 0.0237 |
| 50 g | 5.69 kHz | 0.0168 |
| 100 g | 4.03 kHz | 0.0119 |
| 200 g | 2.85 kHz | 0.0084 |

These are analytical assembly sensitivities, not identified modes or completed native controller tests. If this mass is additional, include it in the actuator's total moving mass. If it lies within the prescribed0.6kg upstream hardware total, split that total between the upstream actuator stage and the joining mass; the inverse inertia coefficient should represent the stage on the spring side, not automatically the entire proximal assembly. Mount, cell-internal and finite-pad/tool modes require their own measured assembly response. Keep upper fixtures light and stiff, and qualify the loaded transfer and antialiasing around any material high mode.

### Damping and identification sensitivity

The sampled low-mode screen uses physical k±20%, guide tangent damping2–1002 N·s/m, slopes0/±0.34346, and µ0.3/0.5, with the selected feedback and sensor/current timing. For each damping value there are120 frozen points. All120 are stable at c30,45,60,75,90 N·s/m. At c15, six grow; at c0, sixty grow. This is stability evidence for the linear maintained-contact approximation; it does not prove force accuracy under damping drift or justify removing the selected damper.

Independent inverse identification errors also affect force tracking. Applying altered inverse coefficients to the **retained causal lower-load input**, then propagating the incremental request through the nominal sampled closed loop, gives:

| Identified coefficient error | Largest calculated incremental normal-force change |
|---|---:|
| k−10% / k+10% | 0.047 / 0.043 N |
| c−20% / c+20% | 0.178 / 0.132 N |
| B mass±10% | 0.089 N |
| Combined k−10%, c−20%, B+10% | 0.294 N |
| Combined k+10%, c+20%, B−10% | 0.207 N |

This is a read-only incremental sensitivity with the original robot trajectory and nonlinear inputs held. It omits changing Coulomb friction, gate decisions, clipping and time-varying geometry; it is not a new native maximum. Do not add these maxima to the nominal maximum as qualification. The values justify identifying the assembled spring rate and especially the damper over relevant velocity, frequency and temperature, with identified coefficients held separately during uncertainty tests. Full combined mismatch and repeated-bump results belong to the native qualification batches.

### Causal actuator gap and guide-velocity estimate, if needed

With S-relative-A position, the actual B actuator gap can be reconstructed from measured lower load without another position sensor:

```text
deltaq_hat = 1/(k+c*s)*(measured_Ltared − Rz_equilibrium)
q_hat = known_q0 + deltaq_hat
known_q0 = (Rz_equilibrium − mS*g)/k

gap_B = (zA−zS) + q_hat
vA−vB = (vA−vS) + q_hat_dot
q_hat_dot = (deltaL_measured − k*deltaq_hat)/c
```

This is one constitutive state, independent of Kf; align it with the head-motion measurement frame. The tiny series-cell deflection can be added as `L/kc` if needed. The formula is useful for coil travel/recentering and actual guide-friction velocity; the S-relative-A signal remains appropriate for observing S motion and the local normal.

A direct velocity estimate is sensitive to force noise. At the existing0.02N cell output noise, qdot RMS is about0.331mm/s. A50/100/200Hz extra velocity LP gives0.029/0.048/0.074mm/s respectively. Near zero guide speed, the current modeled Coulomb slope can turn the unfiltered velocity noise into approximately0.33N of small-signal guide force, before nonlinear saturation. Add such a branch only if retained guide/stroke errors justify it and score its noise and lag natively; the nominal force result does not require an unsolicited observer change.

### Preload and prototype implications

The32 retained short endpoint cases share a feasible per-spring preload interval30.3686–36.8665N. The selected33.6N is inside it. The pair uses two springs of approximately9.87N/mm each, giving net19.75N/mm. Its conservative combined free length is37.18mm and maximum combined installed length is about30.37mm at the midpoint preload, within the100mm requirement. The selected stored preload now agrees with the feasible design; common preload cancels from ideal equal-pair net dynamics while keeping both springs seated.

The retained heavy native case has peak relative lower-link speed0.03306m/s. At c60, this gives about1.98N peak damping force and0.0656W peak mechanical dissipation. This supports a compact damper sizing target, but seal friction, hysteresis, frequency dependence and temperature drift still need characterization. A low-friction damping arrangement is preferable to assuming a generic dashpot has the required Kelvin–Voigt behavior.

**Physical interpretation:** the compliant series pair reduces transmission of fast actuator-stage disturbances and moves the modeled holder dynamics into a range that the causal inverse and force feedback can control. Use approximately20kN/m net spring rate,60N·s/m damping, and34N preload per opposed spring as the prototype starting values, subject to the selected native uncertainty results. Keep the cell in the complete distal load path, measure the assembled damping and mass allocation, and qualify the remaining loaded cell/fixture response before claiming physical force-band performance.


## Root Markdown draft: independent physical wording review

The [current root report](../../reports/active-contact-flange-report.md) was reviewed independently with its selected-v1 results still pending. Its translational balances, normal/tangent projection, measured friction ratio and raw/tared acceleration reconstruction have consistent signs. The selected inverse, its physical support path and finite-fixture limits need to be reflected in the final version. This review makes no root-file changes.

- Add the positive, proper lower-load inverse to the selected current equation.
- Label the **1.64 kHz** mode as the rigid-lower finite-cell comparison. The selected spring/damper mode is **28.87 Hz**. A massless cell in series alters its static stiffness by **0.03084%**; a real additional 50/100/200 g proximal joining mass can produce approximate 5.69/4.03/2.85 kHz modes. These are assembly sensitivities, not identified hardware modes.
- Show the complete lower load path: **B → opposed pair and damper in parallel → joining frame → cell → S**. All distal support loads pass the cell.
- With this compliant link, S-relative-A velocity differs from actual B-relative-A actuator velocity by the link velocity. The measured-load constitutive observer above can reconstruct that difference if retained guide/stroke errors justify its noise/lag cost.
- Call the ADC inputs six **amplified/conditioned bridge channels**, followed by the ATI calibration matrix. ATI's 5.1 kHz two-pole antialias response belongs to its DAQ interface; TWE requires customer conditioning. [ATI interfaces](https://www.ati-ia.com/products/ft/ft_SystemInterfaces.aspx).
- The ADC also has an analog input filter: typical low-bandwidth phase delay is **6.8 µs**, in addition to conversion and processing. Keep the approximately **0.65 µs conversion provision** free of uncertain typ/max column attribution. The **0.00468 m/s² quantum** is ideal resolution; converter/reference noise and drift remain separate. [AD7606C-18 Rev B](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606c-18.pdf).

**Report-ready interpretation:** The selected opposed spring pair and parallel damper form a compliant link above the B–S cell, giving a modeled holder mode near 29 Hz. The much stiffer cell changes that low-frequency stiffness by only about 0.03%, but the real joining frame, cell and mounts can add higher modes that must be identified on the loaded assembly. Use synchronized, calibrated force and tool-acceleration channels with an identified common amplitude/phase response; ideal ADC resolution and conversion time alone do not establish force accuracy.

## Fresh final V8 report and practical measurement review

The current [Markdown report](../../reports/active-contact-flange-report.md) and public guides were reviewed against the final source **73bbc465…937643**, final candidate **d8c49dd3…979c7**, and completed joint/mechanical/current/voltage evidence. No source, helper or root report was edited and no native simulation was launched. Exact file hashes and observations are in `final_V8_report_review_20260930` in the companion JSON.

### Physical contracts and precise wording

The normal vector projection, S-inertia reconstruction, force signs and proper positive lower-link inverse are consistent. State the selected mechanics explicitly: **native k1 = 19,746 N/m and c1 = 60 N·s/m directly, with a massless joining frame; the representative 64 MN/m cell's approximately 0.031% static series-compliance correction is neglected, with no independent cell state in this selected branch**. The stiff no-springs comparison retains the cell dynamics.

The constitutive equation uses the **actual load deviation** from loaded equilibrium. Its separate **desired** load is formed from requested axial force and measured S acceleration. The displacement q is also relative to that equilibrium. Label 28.9 Hz as the **passive/open mechanical** frequency. An ideal instantaneous P-only flat-contact calculation gives about 76.4 Hz and damping 0.73, but omits integral/filter/current/robot dynamics and is not a measured native pole.

### Native bump confirmation

In `simulink/systems/system_287.xml`, frozen V8 native Polyval arrays are `[-64 192 -192 64 0 0 0]`, `[-384 960 -768 192 0 0]`, and `[-1920 3840 -2304 384 0]`. The H, H/T and H/T² scales establish **h = 64 H ξ³(1−ξ)³** over the clamped event, zero outside. The waveform is **C² but not C³** at entry and exit. Position, velocity and acceleration join continuously at zero; jerk has finite jumps. H = 1 mm and T = 0.1 s give peak downward acceleration **−2.4 m/s²** and peak absolute jerk **384 m/s³**. This is not a half-cosine acceleration step.

### Measurement and drive findings

- Treat **0.014 m/s² conditioned acceleration RMS** as an installed chain acceptance target. ADC/PGA/reference and conditioning noise are additional to the ideal bit quantum; the ADXL density calculation alone does not establish the complete target. Preserve the declared separate final quantization.
- ADXL1002 zero and sensitivity are ratiometric. The ±0.01 m/s² bias allocation is only about **40.8 µV** output drift; approximately **81.6 µV supply change, or 16 ppm at 5 V**, can consume it through the zero alone. Use a calibrated ratiometric reference or supply correction, warm-up/tare at operating temperature, and check installed thermal drift. [ADXL1001/1002 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL1001-1002.pdf).
- Qualify the actual **3–4 mH hot winding**. TA115's 5 kHz bandwidth is specified into 2.5 mH; its SOA chart gives **50 W at 70°C**. The real coil's ideal 36 V cold hold dissipates **46.90 W in the driver**, leaving about 3.10 W of that allowance; the generic 20 N/A, 4 Ω coil at 36 V requires **65 W** driver dissipation. Cooling and transient SOA remain required. [TA115 datasheet](https://trustautomation.com/datasheets/TA115DataSheet.pdf).
- Moticont's drawing specifies its nominal values at **25°C**, with 4 mH measured at 1 kHz and 64.7 N continuous force near midstroke. The requirement for 65–70 N over the used stroke and temperature is a prototype target, not an out-of-box guarantee. [Manufacturer drawing](https://www.moticont.com/pdf/lvcm-070-051-01.pdf).
- Describe bitwise voltage comparisons as **all 72 logged channels in each matched pair**. The current-response comparison itself has **12 native recordings**.

### Completed evidence and practical conclusion

The 60 sampled simultaneous-calibration records all pass, worst **0.448085 N**; their remaining 0.051915 N is model reserve in that finite two-endpoint sample. The 72 actual k/c variations and analytical 20 mm geometry rescore pass their declared force/packing checks, with **40.98 mm** combined free length and **35.31 mm** maximum installed length. The rescore adds no force simulations.

The 12 repeated current-response recordings pass, with worst **0.31271/0.34532/0.37230 N** at 50/100/150 µs respectively. The 16-run final electrical subset has 15 distinct conditions, twelve passing runs and four low-voltage failures. Its real hot coil requests **28.473 V**; 31 V usable output leaves **2.527 V** in that cohort. The report correctly separates this provision, holding heat, continuous force and calibration from nominal peak passes.

The recommendation is a coherent first-prototype specification. Acceptance still requires measured complete sensor/drive response, real mounted mass allocation, stable calibrated bias/gain/reference, coil continuous operation over its used region, and independently checked workpiece force. Planned full-grid, longer joint, mixed-error and diagram-acceptance work must retain their own completion status.


## Completed 3500-case grid and public promotion review

Read-only review on 2026-09-30T12:51:56.792442+00:00. No MATLAB process or new native simulation was used. The reviewed report snapshot is `432beb72a5e216b96e6d76ea431359f9db684fe0fc2d6460f97af19f52c81a6e`; later long/refinement/electrical integration has separate status.

The independent CSV audit verifies 3,500 unique identities with no missing, unexpected or duplicate points, 700 groups sharing all five noise conditions, and 1,400,003,500 scored samples. Each of four completed shards contributes 875 histories. Immutable predecessor and source bindings verify 1,060 disk-backed rows reused once plus 2,440 RAM-backed continuations; the 122 recovery histories are excluded. All 27 archived scientific inputs and all nine frozen aggregate files match their manifests. The report conversion preserves every original metric; only display identities/angle rounding change with raw values retained.

The complete retained global worst trace independently reproduces **0.315810929 N at 2.00063 s**, S=2 kg, 50 N, 75 degrees, drag ratio .3, seed97. Every one of its 22,001 excerpt samples matches the original actual/estimated force and timestamps exactly. Actual workpiece-normal projection and full axial balance agree within 2.84e-14 and 7.1e-15 N. At its peak the slow gate is ready with .002058 N mean error and .16612 N/s trend, and the fast variation gate is ready with .056892 N RMS. Throughout the plotted interval the fast RMS reaches only .078043 N, below .08 N. The caption is supported for both branches. Gate readiness is filtered permission for another recentering command; it is not an instantaneous force-error bound.

The canonical public model SHA is **48287e6a17bc9d3144e3d6a77c59ecc601ba4fa26a7ec748173dc89d9bf718de**. Current model, four owned helpers, both candidate files and all five promotion evidence hashes match. Three complete ten-second endpoint comparisons establish source fidelity to scientific V8, with zero difference in the physical normal/sensor/motion/drag/inverse/gate paths and at most 3.02e-14 diagnostic difference. Independently parsing before/after ZIP XML confirms exactly **33 unconnected line stubs removed**, with all **1,092 connected edges, 1,161 block signatures, 42 system signatures and the Model Workspace unchanged**. This is a checked display/metadata bridge to the scientific grid, rather than an invented 3,500-case public-diagram rerun.

The new coverage and native-worst PNGs are readable and source-bound. Their mass maxima and cell maxima match the completed nominal grid; the plotted worst waveform uses every native sample in its window. No substantive unsupported claim was found in these new sections. The active AGENTS handoff still said grid/promotion were pending at the initial read; root was notified to refresh it.

The mixed six-history comparison also supports its narrow conclusion: three heavy calibration-corner8 seeds with actual k+20%/c-20% reach **.489995725 N**, leaving **.010004275 N**; changing only inverse identified k/c to the exact actual **23695.2 N/m and 48 Ns/m** yields **.453025760 N**, leaving **.046974240 N**. B/S/Kf identification residuals, all 26 calibration residuals, PI, timing, physics and original candidate remain preserved. The result supports assembled mechanical identification at that one corner; it does not establish an identification-error allowance or full combined tolerance grid.

The voltage worker's real-hot-coil conversion is algebraically correct. With real currents scaled by 20/24.4, the generic regulator gives `Vg=30*(u-i)+4*i+20*v`, and the real winding gives `Vr=(20/24.4)*(40*(u-i)+8.896*i)+24.4*v`. Eliminating velocity produces `Vr=1.22*Vg+2.411803278688525*i-3.813114754098365*(u-i)`. The sum-of-maxima bound is conservative. Use the native limited reference u and requested, unclipped Vg; rescale every force-domain branch, preserve the current-response target and confirm real headroom before claiming identical force trajectories. The calculation alone adds no native or hardware qualification.

Evidence and exact hashes are in JSON key `final_3500_public_promotion_review_20260930`.


## Final long-repeat margin and prototype calibration interpretation

Read-only review on 2026-09-30T13:05:41.946809+00:00; no new MATLAB process, simulation or shared-file edit. The completed long result JSON is `7900999f5090b2e9c1e1ca3f2a6b359359a9c494d7ec8518b4b1aeeb00a3a23e`. This section interprets recorded worker metrics; the retained worst-trace decomposition is reviewed separately when supplied.

All four **30 s** recordings pass, with **3,000,001 native samples each**, including loaded startup. Heavy S=2 kg/50 N/90-degree calibration corner 8 seeds 7/29/1729 reach **.488090338, .487070904 and .490396047 N**. The last is the cohort worst at **12.80069 s**, leaving only **.009603953 N**. The light S=.1 kg/5 N/0-degree case reaches **.086890612 N**. These four recordings retain nominal actual lower mechanics; they are not a long combined spring/damper/calibration tolerance mesh.

The heavy settled true-force offsets are **+.147686, +.147300 and +.148054 N**, and normal-reconstruction RMS error is about .154 N. This supports the interpretation that persistent measurement/estimation calibration error consumes a substantial part of the reserve, even when the feedback holds its own reconstructed force near the set point. The three finite-grid nominal maxima and simultaneous-error maxima must remain distinct: the nominal 3500 cohort has .184189 N reserve, the original mixed mechanics corner has .010004 N, its exactly reidentified-k/c counterpart has .046974 N, and the longest nominal-mechanics calibration cohort has only .009604 N.

This is a pass at a tested stress boundary, not a generous hardware reserve. Exact assembled k/c identification improved the mixed mechanics cohort; it cannot be assumed to fix the long nominal-mechanics cohort, whose actual k/c already match the nominal inverse. A static tool tare alone cannot identify load gain, cross-axis mixing, dynamic response or every drift after tare. The listed calibration residuals should therefore be described as tested outer stress limits, rather than routinely acceptable independent allocations.

Local sensitivity gives scale to the requirements without predicting a new coupled run: at 50 N, .1% reading-gain error is .05 N; at S=2 kg, .01 m/s² residual acceleration bias contributes .02 N to force reconstruction. With drag ratio .3 and normal force 50 N, a .1-degree normal/frame-angle error can mix about .026 N of tangential force into the normal result. These are first-order examples, not independent additive allowances or a replacement for native qualification.

For the prototype, warm up and tare the complete installed chain, calibrate force gain/matrix under representative combined axial/lateral loads and orientations, characterize acceleration offset/gain/frame and complex force/acceleration timing, enter measured assembled k/c and moving masses, and measure loaded robot response. Confirm true workpiece force using an independent calibrated reference while the powered tool crosses the prescribed bump. An engineering acceptance target such as **at most .4 N measured peak error**, with reference uncertainty explicitly included, would preserve about .1 N nominal design reserve; this is a **proposed acceptance target**, not a newly qualified tolerance box. Aim inside the tested residual boundaries and requalify measured residuals rather than assuming that halving the table entries guarantees a particular new peak.

The integrated final evidence has also been checked against its result summaries: **nine final actuator recordings** are four four-second matched pairs plus one screw refinement, not the earlier ten-second comparison; all nine pass dynamics while the tested RE25 screw fails its catalog continuous holding-current criterion. The final nominal worst at 5/2.5 µs reaches .315822976/.315825399 N, with 14.47 µN total peak change from 10 µs and 2.42 µN between finer histories. Electrical coverage is explicitly **3,498 sufficient algebraic bounds plus two native exception identities**, not 3,500 real-winding simulations. All these scopes are appropriate in the integrated report.

The new disturbance-shape limitation is physically correct: the native compact polynomial is C2, with continuous height/velocity/acceleration and finite jerk jumps. A same-size half-cosine bump has an entry acceleration jump of 1.973921 m/s², corresponding to 3.947842 N downstreamS inertia at 2 kg. Spring displacement and relative velocity are continuous at that edge, so the spring/damper force is continuous and a finite-bandwidth controller without preview cannot cancel the contact-reaction jump instantly. This is an analytical causal limit, not an additional simulated case.

Sources: `inertial_uncertainty_v8_joint_long/inertial_uncertainty_budget_repeated_20260930_results.json`, `inertial_mechanics_calibrated_mechanics_20260930_results.json`, `inertial_actuators_final_actual_acquisition_v8_20260930_reviewed_comparison.json`, `normal_mass_controller_final_grid_20260930_solver_refinement_summary.json`, and `normal_voltage_20260930_final_grid_missing_headroom_proof_summary.json`.


### Mass partition for a real joining frame

This is a prospective prototype contract, not a modeled additional frame or qualified result. Let J denote a real frame between the lower spring/damper and the cell, with mass mJ. At frequencies where the stiff cell keeps J moving with S, the correct approximate balances become

```text
Rz_reconstructed = cell_force_tared + mS * measured_aS
Fa_net = Rz - (mB_actuated + mJ + mS) * (g_axis + aS)
         + mB_actuated * (aS - aB)
delta_lower_load_desired = delta_Rz_requested - (mJ + mS) * measured_aS
I_inverse = (mB_actuated / Kf) * s^2/(k + c*s) * LP2 * delta_lower_load_desired
```

Thus force reconstruction continues to use only downstream S. Total acceleration feedforward includes J, the spring-load inverse uses S+J as the load below the spring, and its differentiating numerator uses actuator-side B above the spring. If J is included inside the 0.6 kg holder budget, split that budget for identification; do not enter the whole moving-assembly weight as each mass. Near the real cell/frame mode, J and S require separate dynamics, which the selected massless-frame plant does not qualify. The report already identifies this mechanical approximation; this partition explains why identifying assembled k/c alone is not every required prototype measurement.


## Independent audit of tighter static calibration, four 30 s repeats

Read-only evidence review on 2026-09-30. No MATLAB process or new native simulation was launched; no model, helper, report or public-document file was edited. This section audits the completed worker records and supersedes the earlier tentative calibration proposal with the actual paired refinement evidence.

The tightened result JSON is `inertial_uncertainty_v8_calibrated_static_long/inertial_uncertainty_calibrated_static_long_20260930_results.json`, SHA256 `34386cf4796359563a3e98955ae4eb0cdf82577bd11a7fcb5919a01204f13711`; its results MAT is `7c5f313dba81cbfd07bdd018ef8021e01fab0721cd6a8f5ef7651726cbc5c354`. The prepared four-configurations MAT is `be9e5f65ff211ee7a1d086574da499a74dcfde5f5168a4aad585d9c7a3701a7a`. Frozen scientific model `73bbc465...`, selected candidate `d8c49dd3...`, private model `a6e461f0...`, recipe snapshot `247d1296...` and all sixteen source/private helper pairs independently match their recorded hashes.

### Native graph and parameter isolation

The new private model `a6e461f0...` was compared directly with original long-run model `6f31addc...` and the retained original instrumented model `4ac68479...`. All **1,136 complete block signatures, 1,072 connected source/destination edges, 37 system parameter signatures and Model Workspace bytes are identical**. The only differing ZIP member is `metadata/coreProperties.xml`. Therefore the changed SLX hash does not conceal a different physical/control graph or saved baseline parameter structure.

Independent SciPy reads of the original four configurations and the prepared tightened configurations confirm the recipe's guard. Exactly these **sixteen static primitive residuals** are halved about their actual/zero values:

| Residual group | Primitive paths | Tightened absolute target |
| --- | --- | --- |
| Force zeros | `sensor.bias`, `contact.lateralBias_N` | 0.01 N each |
| Force reading gain | `uncertainty.forceScaleZ`, `uncertainty.forceScaleX` | 0.05% each |
| Sensor frame alignment | `uncertainty.forceFrameAngle_rad`, `uncertainty.accelFrameAngle_rad` | 0.05 degrees each |
| Force matrix coupling | `uncertainty.forceCrossZFromX`, `uncertainty.forceCrossXFromZ` | 0.05% each |
| Acceleration zeros | `accel.bias`, `uncertainty.accelBiasX_m_s2` | 0.005 m/s² each |
| Acceleration reading gain | `accel.scaleError` | 0.25% |
| Identified S, B and Kf | `sensor.distalMassCalibrationScale`, `uncertainty.massB_model_kg`, `uncertainty.forceConstant_model_N_A` | 0.25% each |
| Position and issued scan scale | `uncertainty.positionScale`, `contact.commandedScanSpeed_m_s` | 0.05% each |

Each configuration has 24 changed paths: these 16 primitives, the corresponding identified S mass and inverse identified B/Kf/numerator, and four identified-mass matrices for the inactive alternative controller. All other primitives remain equal. The complete derived structure also remains equal after excluding only the inactive `d.coilControl` matrices. In particular actual mechanics, equilibrium/current response, PI and feedback/inverse filter settings, force/acceleration/position noise and seeds, acquisition clocks/AA/pipeline mismatch, robot timing and observer coefficients, surface, simulation step and gate criteria are unchanged. The four original case IDs and seeds are retained: heavy IDs 8/18/28 with seeds 7/29/1729, and light ID 43 with seed 29.

### Results and warranted conclusion

JSON and returned MAT metrics agree for all four rows. Each completed **3,000,001 native samples over 30 s**, including startup and 24 compact C2 sharp bumps, with no contact/travel stop or effort/voltage clipping. Reported projection/force-balance residuals remain at roundoff (normal at most 2.85e-14 N, general axial at most 5.22e-15 N and lower-link at most 3.56e-15 N).

| Case | Original peak error (N) | Tightened peak error (N) | Tightened settled true-force offset (N) |
| --- | ---: | ---: | ---: |
| S=2 kg, 50 N, 90 degrees, seed 7 | 0.488090338320 | 0.411871702658 | +0.073590813 |
| S=2 kg, 50 N, 90 degrees, seed 29 | 0.487070903916 | 0.411242938565 | +0.073093549 |
| S=2 kg, 50 N, 90 degrees, seed 1729 | 0.490396046587 | 0.415310748449 | +0.073884698 |
| S=0.1 kg, 5 N, 0 degrees, seed 29 | 0.086890611837 | 0.075628164907 | -0.012384191 |

The cohort worst improves by **0.075085298138 N**, with reserve to ±0.5 N increasing from **0.009603953413 to 0.084689251551 N**. The tightened worst occurs at 12.80070 s, 0.700 ms into the tenth bump. Static calibration therefore recovers material reserve in a controlled paired comparison; the persistent heavy offset approximately halves. This does not establish a full tighter-calibration error box, a new 3,500-case simultaneous-residual mesh, every noise realization, or physical performance. Only the four declared repeats have been qualified under these tighter targets. Combined changed-mechanics cases and real-hot-winding headroom have separate evidence and must be described separately when available.

### Prototype interpretation

These targets require calibration of the assembled chain, rather than buying a device whose catalog RMS noise happens to be small. At the ADXL1002 nominal 40 mV/g sensitivity, a 0.005 m/s² acceleration bias is about **0.51 mg or 20.4 µV**. Without ratiometric/reference correction, approximately 40.8 µV supply movement at its half-supply zero already equals that allowance, about 8.2 ppm of a 5 V supply. Warm-up, stable or measured supply/reference, assembled axis calibration, thermal checks and powered-tool vibration tests therefore matter. Oversampling can reduce random noise but cannot guarantee this residual offset or gain. At S=2 kg the tightened acceleration bias contributes about 0.01 N to reconstructed force; 0.25% acceleration gain or S-mass error acting on the prescribed 2.4 m/s² peak gives about 0.012 N each. These are local sensitivity examples, not independent additive allowances or predictions of a coupled run.

The root report's proposed physical acceptance criterion of **at most 0.45 N including independent-reference uncertainty** is consistent as a development target, leaving at least 0.05 N to the actual ±0.5 N requirement. An explicit interpretation is `maximum absolute reference-force error + applicable reference uncertainty <= 0.45 N`, evaluated with sufficient synchronized bandwidth to retain the sharp transient and powered-tool vibration. The tightened model's 0.4153 N peak leaves only 0.0347 N to that proposed bench target, so it cannot be presented as an already passed physical acceptance test. The criterion is a new bench requirement; it does not replace calibration, timing or fixture-mode measurement and does not confer qualification on untested disturbances.
