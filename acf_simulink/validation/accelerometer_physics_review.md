# Tool-mounted accelerometer: force reconstruction and sensing review

This review concerns a B–S vector load cell and an accelerometer mounted on
S, with S mass known in the range 0.1–2 kg. It gives physical equations,
sensor examples and analytical budgets. It does not claim native controller
qualification or a purchased sensor's performance in the prototype.

## Force reconstruction: signs and gravity

Use a common, calibrated coordinate frame. Let `R` be the workpiece reaction
on the tool, `L_raw` the distal-tool load transmitted back through the B–S
cell, `aS` the distal center-of-mass acceleration, and `g_vec` gravitational
acceleration, including its downward sign. Newton's law gives:

```
L_raw = R + mS*g_vec - mS*aS
```

An accelerometer reports **specific acceleration**:

```
fS = aS - g_vec
```

For an upward-sensitive axis, a stationary sensor therefore reads `+g`, not
zero. The force reconstruction is consequently:

```
R_hat = L_raw + mS*fS_measured
N_hat = dot(n_hat,R_hat)
```

The equal-and-opposite reaction on the workpiece has the same pressing-force
magnitude `N`. This reconstruction needs no coil force constant, current
estimate, guide-friction estimate, or robot-acceleration estimate.

If the load cell is gravity-tared instead, use dynamic acceleration:

```
L_tared = L_raw - mS*g_vec = R - mS*aS
R_hat = L_tared + mS*aS_measured
```

Combining a gravity-tared cell with unmodified specific acceleration adds
gravity twice. Combining a raw cell with zeroed dynamic acceleration omits
gravity. The implementation must choose one convention consistently.

For each fixed orientation, zeroing the cell with the entire installed tool
out of contact, and subtracting the accelerometer's stationary reading,
produces the second convention directly. This removes the fixed gravity
component and stationary acceleration bias. Orientation changes after that
zero require updated gravity compensation or the fully calibrated raw
specific-acceleration convention.

If the sensor axes differ from the cell axes, transform acceleration into
the cell frame before multiplying by mass. All distal loads must pass through
the cell. The accelerometer must represent the distal center of mass; an
offset point on a rigid rotating tool has additional
`alpha cross r + omega cross (omega cross r)` acceleration. Internal moving
tool parts and flexible brackets need separate checks.

## Synchronization is part of the force equation

Let `HL` and `Ha` represent the complete cell and accelerometer transfers,
including mounting, electronics, filtering, sampling and delay. With matched
frames and exact mass:

```
R_hat = HL*R + mS*(Ha-HL)*fS
```

Matching only sample frequency does not cancel the inertial term. The
amplitude and phase responses must agree over the frequency band used by
the controller. If the transfers match, reconstruction yields the common
filtered/delayed reaction `H*R`.

Two causal matching approaches are available:

1. Equalize the faster path to an identified common response, with enough
   delay to accommodate both physical channels.
2. Cross-filter: apply `Ha` to the measured cell and `HL` to measured
   acceleration. Both then have transfer `HL*Ha`. This guarantees cancellation
   in the modeled linear transfer but incurs both channels' phase delay.

Limited phase-lead compensation can be tested, but inversion of an unknown
transfer or a pure delay is not a causal calibration. An identified magnitude
bandwidth is insufficient to specify phase. The normal-direction estimate
must also refer to the reconstructed force's effective timestamp, or all
quantities must be causally predicted to the present.

For the sharp bump, the largest jerk is 384 m/s³. A small differential delay
therefore gives the conservative first-order bound:

```
abs(force_reconstruction_error) <= mS*384*abs(differential_delay)
```

| S mass | Error from 100 microseconds mismatch | Delay for a separate 0.1 N allocation |
|---|---:|---:|
| 0.1 kg | 0.00384 N | 2,604 microseconds |
| 0.5 kg | 0.01920 N | 521 microseconds |
| 1.0 kg | 0.03840 N | 260 microseconds |
| 1.4 kg | 0.05376 N | 186 microseconds |
| 2.0 kg | 0.07680 N | 130 microseconds |

At S=2 kg, a separate 0.05 N allocation reduces the differential-delay
target to about 65 microseconds. These bounds concern relative timing;
the common delay still affects feedback stability and feedforward response.

## A simpler controller and kinematic observer

The measured contact vector permits ordinary force PI or filtered PID on
`N_reference-N_hat`, with current saturation and antiwindup. An accelerometer
also provides causal inertia feedforward. For the both-rigid topology,
`M=mB+mS`, while the reconstruction above uses only `mS`.

For slope `s`, let `c=sqrt(1+s*s)`. The desired axial reaction can be calculated
without knowing sanding friction:

```
Rz_reference = N_reference*c + s*Rx_hat
```

Useful current feedforward is:

```
i_ff = (Rz_reference - M*fSz_hat + friction_hat)/Kf_nominal
```

Equivalently use `M*(g_axis+aSz_hat)` with the tared/dynamic convention.
Force feedback corrects feedforward error. Kf remains an actuator gain and
feedforward parameter, but no longer determines what the force sensor says.
Unknown Kf therefore does not create false contact-force reconstruction.
Its remaining gain/dynamic uncertainty still requires a native robustness
test. Current readback can improve the drive model without calibrating Kf.

Tool-position differentiation can be replaced with a small kinematic
observer. For both-rigid attachments, define states:

```
q = S position relative to A
vS = absolute axial tool velocity
dvA = residual robot/body velocity relative to issued-command prediction

qdot = vS - vA_predicted - dvA
vSdot = measured_dynamic_aSz
dvAdot = -dvA/residual_time + process uncertainty
```

Measure `q` with the S–A position sensor; initialize a stationary start
consistently. This three-state model is observable when the residual decay
is nonzero. It uses no force signal or Kf. Acceleration supplies prompt
velocity changes, while position innovations can correct robot-prediction
mismatch rather than assigning every mismatch to the workpiece shape.
Bias/process weights and residual dynamics must be tested, not assumed.

At nonzero scan speed:

```
s_hat = vS_hat/vx_predicted
n_hat = [-s_hat,1]/sqrt(1+s_hat*s_hat)
```

This is causal: it uses measured tool motion and issued robot/scan commands,
without the bump clock, profile derivative or future surface information.
Use a low-speed guard and retain the last credible normal near a stop.
Actual tangential scan mismatch remains an uncertainty. For compliant
attachments, A need not follow the robot boundary, so body-motion residuals
must cover attachment dynamics or use a more suitable observer.

The normal angle also has its own budget. With tangential drag `T`, a small
angle error gives approximately `-T*angle_error` in normal force. At 50 N,
`T/N=0.5`, and a separate 0.1 N allocation, this permits about 0.229 degrees.
The bump slope rate reaches 24 per second, so a naive velocity estimate with
1 ms lag can create about 0.6 N error at that operating point.

## Practical sensor examples

The following are examples to ground approximate specifications, not a
parts list or proof of installed performance.

**ADXL354C** provides three analog axes and selectable ±2/±8 g ranges.
The datasheet gives 22.5 micro-g/sqrt(Hz) in ±2 g and estimates 50% more
noise in ±8 g, with 0.1 V/g nominal sensitivity there. Its response includes
an approximately 1.5 kHz sinc-shaped antialias filter, 2.4 kHz mechanical
resonance, and output resistance of 32 kohm; overall bandwidth is 1.9 kHz.
Initial sensitivity and cross-axis calibration are required. The datasheet
also documents vibration rectification, so installed sanding vibration
must be tested. [ADXL354/355 datasheet, Rev D](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf).

**ADXL355** adds digital filtering. Its fastest 4 kHz output has 0.63 ms
digital group delay; interpolation increases this to 0.88 ms, before the
full acquisition/software path. This is a different timing provision from
a synchronized analog acquisition. [ADXL354/355 datasheet, Table 10 and
Table 12](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf).

**ADXL1002** is single-axis, ±50 g, with 25 micro-g/sqrt(Hz) noise and
40 mV/g nominal sensitivity at 5 V. Its 11 kHz 3 dB point is a gain rise near
its 21 kHz mechanical resonance, not a first-order low-pass corner. It needs
external antialias filtering, calibration and suitable sampling.
[ADXL1001/1002 datasheet, Rev A](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL1001-1002.pdf).

An **AD7606C-18** illustrates simultaneous analog acquisition. Its
18-bit, eight-channel converter can use an approximately 0.65 microsecond
conversion provision without oversampling. The flattened Table 3 extraction
does not independently establish typ/max column attribution here. Channel
aperture, filtering and controller readout latency still need verification.
[AD7606C-18 datasheet, Rev B](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606c-18.pdf?isDownload=true).

## Noise and response envelope for experiments

For an explicitly assumed first-order response with time constant `tau`,
one-sided white acceleration noise density `na` gives:

```
ENBW = 1/(4*tau)
acceleration_noise_RMS = na*sqrt(ENBW)
normal_inertia_noise_RMS = mS*acceleration_noise_RMS
```

At ±8 g, the ADXL354 density example is
`33.75e-6*9.81 = 0.0003311 m/s^2/sqrt(Hz)`. This yields:

| Assumed common time constant | ENBW | Acceleration RMS | Inertia-noise RMS at S=2 kg |
|---|---:|---:|---:|
| 200 microseconds | 1,250 Hz | 0.01171 m/s² | 0.02341 N |
| 300 microseconds | 833 Hz | 0.00956 m/s² | 0.01912 N |
| 400 microseconds | 625 Hz | 0.00828 m/s² | 0.01655 N |

These are calculations for the declared response envelope, **not a fitted
ADXL354 transfer**. Its actual mechanical, sinc and external RC response is
not a first-order filter. Test 200–400 microseconds as an experimental range
and differential delays of ±50/±100 microseconds. Measure the complete
installed transfer over the bump/control band before assigning a prototype
timing specification.

The 32 kohm output resistance with 1 nF, 4.7 nF or 10 nF capacitance adds
32, 150.4 or 320 microseconds respectively. Thus a 200 microsecond full-chain
assumption can be optimistic once external filtering is included. A
400 microsecond test does not establish a guaranteed worst-case bound.

For 0–5 V, 18-bit conversion and 0.1 V/g sensitivity, quantization is
0.001871 m/s² per LSB, or 0.000540 m/s² RMS under the standard uniform
quantization assumption. ADC/reference offset, gain and noise remain
additional errors. Ratiometric acquisition or calibrated reference tracking
is needed to avoid converting supply drift into acceleration.

Noise density specifies a spectrum, not independent post-filter sample
noise. A simulation using independent noise after the sensor response must
be described as an RMS stress model. A physical white-density model should
inject noise through the identified filtering, with anti-aliasing and
sample-rate effects included. Sample peaks and a hard ±0.5 N requirement
cannot be inferred from RMS alone.

## Calibration allocations and suggested stress cases

For a separate 0.1 N inertial-compensation allocation:

```
acceleration_bias_or_error <= 0.1/mS
```

This permits 1 m/s² at S=0.1 kg but only 0.05 m/s², approximately 5.1 mg,
at S=2 kg. Smaller mass helps the measurement budget directly.

The following are useful native stress inputs, to be tested independently
and jointly:

| Uncertainty | Example test | Largest direct contribution at S=2 kg |
|---|---:|---:|
| Acceleration bias after fixed-orientation zero | ±0.02 m/s² | ±0.04 N per projected axis |
| Acceleration gain after fixed-orientation zero | ±0.5% / ±1% | ≤0.024 / 0.048 N using 2.4 m/s² acceleration bound |
| Raw specific-acceleration gain without gravity cancellation | ±1% | 0.196 N from gravity alone |
| Distal mass error with dynamic/tared convention | ±1% | ≤0.048 N |
| Distal mass error with raw/specific convention | ±1% | 0.196 N from gravity alone |
| Cell force bias | ±0.02 / ±0.05 N | Approximately the projected cell bias |
| Differential acceleration/cell delay | ±50 / ±100 microseconds | ≤0.0384 / 0.0768 N by jerk bound |

These are separate allocations. They cannot all spend the same 0.1 N budget.
With independent equal-axis noise and a unit normal, the projected
accelerometer noise has the same RMS as one axis. Deterministic biases,
cross-axis terms and shared calibration errors require vector projection
and a combined bound instead.

A startup tare is insufficient to determine sensor scale and alignment.
Calibrate the accelerometer's bias, gain and axis coupling with a 3×3 map,
then verify its dynamic transfer. Perform the corresponding multi-axis cell
calibration and mechanical load-path check. Repeated static orientations,
dynamic motion, warming, and operation with the sanding motor running are
necessary checks of the physical assumptions.

## Recommended interpretation

A calibrated S accelerometer plus the vector B–S cell changes the force
measurement into a direct, mass-compensated reaction measurement. This
supports a simpler normal-force controller and removes the strong reliance
on Kf-dependent force reconstruction. Begin with synchronized measured-normal
PI and acceleration feedforward, then test mass, orientation, drag, voltage,
sensor transfer and calibration uncertainty in the native model.

Use the ADXL354C noise figures and 200–400 microsecond transfer envelope as
a plausible initial experiment design. Keep phase and vibration verification
as measured prototype tasks. Select the final sensor response and controller
only after the native joint tests demonstrate useful margin across S=0.1–2 kg.

## Addendum: a faster analog acquisition candidate

Orthogonal **ADXL1002** sensors can reasonably support an identified dominant
response near **100 microseconds**, plus a separately budgeted
**100 microseconds for acquisition and control**. This is a buildable
signal-chain acceptance target, not a manufacturer guarantee of that
combined transfer or the ±0.5 N force result.

The datasheet lists a typical 5%-deviation band of 4.7 kHz. Its mechanical resonance
is 21 kHz and the output amplifier has a 70 kHz small-signal pole. The
11 kHz 3 dB specification concerns increasing response toward resonance.
Thus the core has much more separation from a 1.6 kHz mechanical cell mode
than ADXL354. [ADXL1001/1002 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL1001-1002.pdf).

Analog Devices' shaker measurements of ADXL354 in ±8 g found approximately
1.3 kHz for its ±5% flat band and documented axis-dependent resonance
enhancement. This supports using a measured AC transfer near a 1.6 kHz mode,
rather than representing ADXL354 solely by its low-frequency noise and
an arbitrary short delay. [Analog Devices AC-calibration article](https://www.analog.com/en/resources/technical-articles/2017/05/01/09/59/measuring-ac-accelerations-to-calibrate-or-not-to-calibrate.html).

The faster sensor does **not** remove the cell/attachment's mechanical mode.
It reduces additional sensor-response distortion around that mode. The
controller must still avoid exciting the mode, and the complete cell and
acceleration channels must be measured and matched.

### Response, noise and ADC provision

An externally designed dominant pole of 100 microseconds has a
1,591.55 Hz corner and a 2,500 Hz equivalent noise bandwidth. At the
ADXL1002 density example of 25 micro-g/sqrt(Hz):

```
noise_density = 0.00024525 m/s^2/sqrt(Hz)
ideal_white_noise_RMS = 0.0122625 m/s^2
inertia_noise_RMS_at_S2 = 0.024525 N
```

For example, 10 kohm and 10 nF give that nominal RC pole and meet the
ADXL1002's stated capacitive-load conditions. A 1 Mohm ADC input changes
the pole to about 99 microseconds and introduces about 1% DC attenuation,
which belongs in the signal-chain calibration. This example demonstrates a
feasible dominant pole; it is not a complete antialias circuit.

At 40 mV/g, 0–5 V and 18 bits:

```
ADC_quantum = 0.004678 m/s^2
ideal_quantization_RMS = 0.001350 m/s^2
combined_acceleration_RMS = 0.012337 m/s^2
combined_inertia_noise_RMS_at_S2 = 0.024673 N
```

The last combination assumes independent white sensor and uniform
quantization errors. Adding an independent 0.02 N cell RMS gives about
0.03176 N projected-force RMS at S=2 kg. This remains an analytical noise
estimate, not an installed noise guarantee.

A realistic native comparison should retain the theoretical 0.01226 m/s²
noise case and also stress acceleration RMS at 0.015 and 0.02 m/s². That
checks sensitivity to additional electronics/aliasing noise without
silently claiming the theoretical density calculation covers the entire
signal chain.

One external RC can establish a dominant pole, but it is insufficient by
itself to establish the complete noise and aliasing specification. The
ADXL1002 electronics pass substantial out-of-band noise and a 200 kHz
internal clock component. Analog Devices demonstrates that insufficient
filtering can leave excess RMS even when the in-band noise density agrees
with the datasheet. [ADXL1002 noise-measurement guide](https://wiki.analog.com/_media/mems/reference_document_on_achieving_best_noise_performance_with_adxl1002.pdf).

Use stronger out-of-band analog rejection and faster simultaneous raw
acquisition, followed by calibrated decimation to the control rate. Count
every additional pole and digital-filter delay in the acceptance envelope.
The raw ADC rate should be chosen with the actual filter and interfering
frequencies; a 10 kHz controller rate does not imply it is safe to sample
the unqualified analog sensor at only 10 kHz. The sub-microsecond typical
conversion time of the AD7606C-18 makes a 100 microsecond acquisition/control
allowance plausible, but filtering, triggering, data transfer and processing
determine actual latency. A desktop/network loop cannot be assumed to meet
that deterministic allowance.

### Synchronization with ATI analog force acquisition

ATI DAQ F/T exposes six conditioned strain-gage channels that can be sampled
together with acceleration channels. ATI identifies its DAQ antialias
filter as second order at 5.1 kHz and notes that tooling changes the
transducer's mechanical response. [ATI system interfaces](https://www.ati-ia.com/products/ft/ft_SystemInterfaces.aspx).

Use a common hardware acquisition trigger, then apply the ATI calibration
matrix before force reconstruction. Two accelerometer axes plus six strain
channels fit an eight-channel simultaneous ADC for this two-dimensional
model. Three acceleration axes plus six strain channels need at least nine
simultaneous channels. Supply/reference monitoring can require another
channel. Multiple converters require shared triggering and phase
verification rather than assuming independently clocked records align.

The ATI 5.1 kHz filter remains part of the force transfer. It must not be
discarded when assigning a nominal 100 microsecond pole to the accelerated
experiment. Equalizing the sensor paths to a common identified response,
or including both transfers in a causal compensated measurement, is still
necessary. Identifying only the first-order low-frequency time constant
does not determine behavior around a mechanical mode.

### A high-frequency matching check

For a tool-acceleration harmonic of amplitude `A` at frequency `f`, sensor
transfer mismatch contributes:

```
force_error_amplitude = mS*A*abs(Ha-HL)
```

With equal magnitudes and a residual pure delay:

```
force_error_amplitude = 2*mS*A*abs(sin(pi*f*delay_mismatch))
```

At S=2 kg, A=1 m/s² and 1.6 kHz, a 50 microsecond mismatch gives
0.995 N and 100 microseconds gives 1.927 N. These are hypothetical harmonic
sizing examples, not amplitudes observed in the native model. They show why
the sharp bump's low-frequency jerk budget cannot certify phase matching
for actual high-frequency tool vibration.

For a 0.1 N allocation under that example, relative phase must be within
about 2.87 degrees, equivalent to roughly 5 microseconds at 1.6 kHz. The
correct hardware acceptance check uses measured acceleration amplitudes and
the measured complex transfer over the frequencies that materially affect
force, together with the controller's high-frequency attenuation.

### Acceptance criteria for the faster experiment

Use the 100 microsecond dominant-response plus 100 microsecond
acquisition/control provision as an explicitly tested target. Before treating
it as a prototype specification:

1. Measure the assembled cell, accelerometer, mounts, conditioning and ADC
   magnitude/phase response over the bump band and relevant structural modes.
2. Verify interchannel timing and the complete deterministic path to current
   command, including any decimation and processing delay.
3. Measure full-path noise and clock/alias rejection with the sander off and
   running, over range, position and temperature.
4. Calibrate bias, scale and axis coupling; retain operating-temperature
   checks. Startup zero does not establish gain or eliminate later drift.
5. Retain native common-response, differential-phase, noise and calibration
   stress cases. Report passes for their actual modeled envelopes.

The ADXL1002 route is a more credible fast-sensing candidate than assigning
an ideal 100 microsecond response to ADXL354. The real improvement must come
from the designed and identified signal chain and the resulting native
controller margin.


## Native noise placement and measured quiet-window budget (V4 audit)

The actual frozen V4 wiring treats `accel.noiseRms = 0.0123 m/s²` as the **post-response noise target**, before ADC quantization. Although Gaussian samples enter before the EMA, their gain includes `sqrt((1+alpha)/(1-alpha))`, which compensates that filter's attenuation. For Ts = tau = 100 µs, alpha = exp(-1), the EMA standard-deviation factor is 0.67979, and injected acceleration noise is 0.01809 m/s². Post-EMA RMS is therefore 0.0123 m/s²; adding the 0.00468 m/s² ADC quantum in quadrature gives 0.012374 m/s². Raising this configured value to 0.018 or 0.020 would raise the intended **output** noise, not repair a missing normalization.

Both force channels add bounded-uniform noise **after** their true-signal EMA and before transport delay. The configured 0.02 N is therefore already output RMS, with support ±sqrt(3)*0.02 = ±0.03464 N. It is not attenuated to 0.0136 N. This is verified by tracing the actual blocks and subtracting the noise-free filtered physical cell force from the retained measurement.

The [frozen model](</home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v4-20260930/frozen-inertial-v4/acf_force_control_normal_trial.slx>) has SHA256 `2aaafd4da3b6f5593157f1667e84e2fb72ebdc994804f743f8e5c80d8202668b`. The empirical window is 6.3–6.7 s of [the retained matched finite-cell trace](inertial_uncertainty_finite_provisional/inertial_uncertainty_repeated_screen_20260930_native_traces/case_0001.mat), sampled at 10 kHz. Both physical S accelerations are zero in that window.

| Channel | Configured RMS | Ideal output noise RMS | Measured isolated noise RMS | Measured channel standard deviation | Noise-free physical signal after transfer |
|---|---:|---:|---:|---:|---:|
| B–S force Z | 0.020 N | 0.020 N | 0.02008 N | 0.04709 N | 0.04283 N |
| B–S force X | 0.020 N | 0.020 N | 0.01977 N | 0.02377 N | 0.01285 N |
| S acceleration Z | 0.0123 m/s² | 0.012374 m/s² | 0.01264 m/s² | 0.01264 m/s² | 0 |
| S acceleration X | 0.0123 m/s² | 0.012374 m/s² | 0.01216 m/s² | 0.01216 m/s² | 0 |

Measured reconstructed Rz and Rx have standard deviations 0.05202 and 0.03395 N; estimated normal force is 0.05108 N. These include physical closed-loop force variation, cell noise, and acceleration compensation. Unfiltered physical contact Rz and Rx alone have standard deviations 0.04812 and 0.01444 N. Those physical components should not be described as sensor noise. They are not necessarily independent of measurement noise because that noise drives the closed loop.

For sizing only, independent sensor errors give `sigma_R = sqrt(0.02² + (mS*0.012374)²)`: about 0.02004 N at S = 0.1 kg, 0.02646 N at S = 1.4 kg, and 0.03182 N at S = 2 kg per force-vector axis, before slope-estimation and physical modal variation. Exact values and measured means are in the JSON.

The [ADXL1002 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL1001-1002.pdf) gives typical 25 µg/√Hz density. A hypothetical white sequence at 10 kHz would have raw RMS 0.01734 m/s²; this discrete EMA's ENBW is 2310.6 Hz, giving 0.01179 m/s² without normalization. A true analog first-order tau = 100 µs envelope has ENBW 2500 Hz and density-derived RMS 0.01226 m/s². The configured 0.0123 m/s² output target agrees with the latter sizing estimate. This does not certify installed phase response, clock-noise aliasing, vibration rectification, or device-to-device maximum noise.

The force noise distribution deserves a separate sensitivity. Bounded uniform noise has shorter tails than Gaussian ADC/amplifier thermal noise at the same RMS. For 100,000 independent Gaussian output samples with sigma = 0.02 N, the 95% and 99% quantiles of the maximum absolute sample are about 0.1004 and 0.1065 N, compared with the uniform bound 0.03464 N. Closed-loop filtering and correlation change the resulting force-error peaks; a variance comparison alone cannot qualify that effect. Keep the native default and add an explicitly named, equal-output-RMS Gaussian force-noise test if distribution-tail margin is to be claimed. No source distribution was changed in this review.

## Designed common fast analog path with fresh acquisition

A whole 100 µs software transport delay can be avoided using ATI analog force channels, orthogonal ADXL1002 sensors, simultaneous AD7606C-18 acquisition, and a deterministic local control processor. The ADC's approximately 0.65 µs conversion provision covers only conversion; readout, filtering, force calibration, computation, and current-command update still need a measured timing bound. A fresh-frame target of 0–20 µs acquisition/processing, with 50 µs sensitivity, is an engineering provision. It is not a manufacturer guarantee. [ATI system interfaces](https://www.ati-ia.com/products/ft/ft_SystemInterfaces.aspx), [AD7606C-18 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606c-18.pdf).

Removing that transport does not remove analog phase. A direct 100 µs EMA at 10 kHz has DC delay 58.2 µs and phase −21.27° at 1643.7 Hz. A continuous 100 µs RC has phase −45.92° there. A common physical cascade of that RC and an illustrative Butterworth ATI 5.1 kHz filter has approximately 144 µs DC delay and phase −72.9° before electronics/processing. Simply assigning zero additional transport to the old EMA does not reproduce that cascade.

An alternative is to design the ADXL conditioning to match the actual ATI two-pole 5.1 kHz response, rather than selecting an arbitrary 100 µs RC. Use an appropriate buffer and active analog filter so the ADXL output-loading limits are respected. ATI already contains its filter: the native common path represents one effective matched filter per signal, and does not authorize adding a duplicate physical filter to the force path. Identify ATI's actual poles/damping and the ADXL installed intrinsic response, then implement a causal match or a conservative common response. The published AA order and cutoff do not establish an exact Butterworth phase guarantee. [ATI system interfaces](https://www.ati-ia.com/products/ft/ft_SystemInterfaces.aspx), [ADXL1001/1002 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL1001-1002.pdf).

The tested design assumption is two-pole Butterworth AA at 5.1 kHz, raw acquisition at 100 kHz, and a common direct EMA with time constant 50 µs at that raw rate. The controller receives the latest frame every 100 µs. It has:

| Quantity | Engineering value |
|---|---:|
| Analog AA DC group delay | 44.13 µs |
| Raw digital EMA DC group delay | 45.17 µs |
| Common response DC group delay | 89.30 µs |
| Gain / phase at 20 Hz | 0.99998 / −0.643° |
| Gain / phase at 1643.7 Hz | 0.88416 / −51.41° |
| Gain / phase at 2155 Hz | 0.81580 / −66.38° |
| Acceleration ENBW | 3180.10 Hz |
| Density-derived output acceleration RMS | 0.013830 m/s² |

The ADXL intrinsic phase, analog filter deviations, and finite processing must be added to this engineering transfer. Its phase advantage over the earlier EMA100 plus100 µs transport is plausible; it is not the transfer of an ideal zero-delay sensor.

### Minimal native path

At a 10 µs raw sample period, the bilinear-prewarped 5.1 kHz AA approximation has:

```text
AA numerator   = [0.0208156716160, 0.0416313432320, 0.0208156716160]
AA denominator = [1, -1.5524225960790, 0.6356852825431]

EMA numerator   = [0.1812692469220, 0]
EMA denominator = [1, -0.8187307530780]
```

The EMA numerator includes the final zero because it must use the fresh sample. A constant numerator with the same denominator would add a raw sample of delay. Equivalently use the visible recursion `y[k] = alpha*y[k-1] + (1-alpha)*x[k]`. Two visible filter blocks followed by a rate-transition/latch explain the three response states. The combined coefficients are retained in JSON.

Run the effective common response before the 10 kHz sensor/controller transition. Use matched force and acceleration outputs to reconstruct the contact vector. Keep the original measured acceleration for that reconstruction; actuator-only FF filters remain separate. No true contact force, programmed surface profile, or future sensor data enters the controller. Include a finite pipeline target/stress explicitly, rather than equating omission of a whole 100 µs sample with zero physical latency.

### Noise and alias budget

At 25 µg/√Hz, the analog AA alone gives RMS 0.01846 m/s². The complete analog-AA plus raw-EMA response gives 0.013830 m/s². Thus use approximately **0.014 m/s² output RMS** as the minimum rounded native engineering budget, with 0.015 and 0.020 stress cases. The earlier 0.0123 budget cannot be reused for this faster chain. These are typical density calculations, not installed maximum specifications. [ADXL1001/1002 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL1001-1002.pdf).

Physical ADC quantization occurs before digital filtering at 100 kHz. With quantum 0.00468 m/s², white quantization RMS falls from 0.001351 at raw input to about 0.000427 m/s² after EMA50. The combined density/quantization estimate is 0.013837 m/s². Retaining the earlier after-response quantizer is a named conservative approximation and gives 0.013896 m/s². Neither calculation covers ADC nonlinearity or coherent quantization.

The two-pole analog AA attenuates 50 kHz, the raw ADC Nyquist frequency, by 39.66 dB. The white-noise contribution above 50 kHz before digital filtering is only about 0.000329 m/s² under the assumed density. Decimation to 10 kHz still folds about 7.19% of output white-noise variance from above 5 kHz, with 2.39% additional noise power density at 1643.7 Hz. That modest white-noise result does not prove rejection of narrow sanding lines or clock components.

For example, total assumed analog/digital attenuation is about 19.7 dB at 9 kHz, which aliases to 1 kHz on the control grid. It is 40.4 dB at 21 kHz before the ADXL mechanical-response increase, and the analog AA attenuates the 200 kHz clock region by 63.7 dB before sampling. A clock coherent with a 100 kHz raw rate can fold to DC; a later digital filter cannot remove an already folded spur. Measure these installed artifacts with the sander running, and add rejection if the actual force budget requires it. Additional filtering brings its own phase and must be retained in the identified transfer.

The exact raw/filter-lifted pole and stationary-noise comparison is in [the structural review](structural_control_review.md). In this designed response, P0.06/LP800 with damping 1.5 is stable across all 240 tested 40–110 MN/m frozen points at 0/20/50 µs pipeline. P0.06/LP1000 fails three low-stiffness ascent points at ideal zero pipeline, while its 20/50 µs cases are stable. These results demonstrate why the controller must be qualified against the complete installed response. They do not establish ±0.5 N peak performance.


The head-position and issued-robot kinematic alignment branches also need the common effective acquisition frame, including their own delays. The 40 Hz head-motion observer remains a separate limit on causal direction estimation. Do not apply the common filter twice to acceleration already conditioned in the sensor path.

## Final manufacturer-driven acquisition check

The analog sensor candidates and timing provisions remain representative engineering choices. The published ADXL1002 4.7 kHz band describes response within 5% of DC gain; 11 kHz is a gain-deviation specification near its resonance, not a first-order corner. Its mounted phase and conditioning are identified assembly properties. [ADXL1001/1002 Rev A](https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL1001-1002.pdf).

Include the ADC's analog input-filter response as well as conversion, readout and computation. AD7606C-18 low-bandwidth mode has a typical 25 kHz corner and 6.8 µs phase delay. The general high-bandwidth mode gives 220 kHz/1.1 µs; for 0–5 V it gives 150 kHz/1.5 µs. Different channel modes require explicit matching. The approximately 0.65 µs no-oversampling conversion value is used here as a provision; earlier typ/max labels from flattened PDF columns are not independently verified. Oversampling adds latency. [AD7606C-18 Rev B](https://www.analog.com/media/en/technical-documentation/data-sheets/ad7606c-18.pdf).

Ideal 18-bit quantization is only one noise term. Converter/PGA/reference noise, gain, offset, nonlinearity and thermal drift need separate calibrated and measured budgets. Measure acceleration and force PSD/RMS after the actual conditioning and decimation, with rigid-body and mount motion separated from electronic noise. These checks do not require raising the correctly normalized native 0.0123 m/s² output noise merely because the signal is filtered.

Use six conditioned bridge channels from a DAQ interface, or provide documented TWE amplification and conditioning before simultaneous conversion. Apply ATI's calibration matrix after digitization. The published two-pole 5.1 kHz antialias filter is associated with DAQ F/T; TWE supplies no built-in antialias stage. A designed matched filter is therefore a declared engineering transfer, not an automatic property of every ATI interface. [ATI interfaces](https://www.ati-ia.com/products/ft/ft_SystemInterfaces.aspx).
