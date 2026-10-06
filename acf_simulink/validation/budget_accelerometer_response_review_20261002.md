# ADXL354C response and acceptance review — 2 October 2026

**Manufacturer sources do not constrain the complete ADXL354C phase enough to replace the explicitly assumed 1.9 kHz two-pole acceptance response with a documented part transfer.** The native latency screens establish useful controller acceptance targets. They do not establish that the real part falls inside those targets. This read-only review performs no new simulations and preserves the earlier component review and source files.

## What the primary sources establish

The current **Rev D datasheet**, Table 1 and the Filter section, gives a nominal 2.4 kHz mechanical resonance, Q = 7.4 for X/Y and 1.5 for Z, approximately 1.5 kHz sinc-shaped internal analog filtering and 1.9 kHz overall 3 dB bandwidth. Figures 8–10 show overall axis response; they are not bare mechanical-core response. There is no complete analog impulse, implementation clock, pole/zero list or full-path delay bound. The 32 kΩ output resistance is disclosed. The ADXL355 digital delay tables cannot supply the missing ADXL354 analog delay. [ADI Rev D datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf).

**Source-interpretation correction:** the 1.9 kHz table entry says “3 dB, overall transfer function,” without specifying a minus sign. A resonant overall response is not a monotonic low-pass. The earlier component review's `overallMinus3dB_Hz` field and signed narrative were too specific; this review supersedes that interpretation while preserving the original research artifacts. Use **published overall 3 dB bandwidth** in integration. The native Butterworth envelope still has its chosen −3 dB cutoff at 1.9 kHz, which is an explicit modeling assumption.

An older official article, **Measuring AC Accelerations: To Calibrate or Not to Calibrate?**, by ADI MEMS applications engineer Siddharth Tallur, describes the internal analog filter more specifically as sinc³ with about a 1.5 kHz 3 dB corner. It reports physical sinusoidal shaker measurements from 30 Hz to 5 kHz and resonant gain enhancement on top of the internal filtering. It also describes typical device-to-device resonance variation below 2% and Q variation below 10%, and recommends AC calibration where absolute frequency response matters. It supplies no complete phase/impulse specification. Its historical sinc³ description is informative; it is not permission to substitute a guessed clocked FIR for the current Rev D part. [Official technical article](https://www.analog.com/en/resources/technical-articles/2017/05/01/09/59/measuring-ac-accelerations-to-calibrate-or-not-to-calibrate.html).

This limitation is explicit in manufacturer support. **ADI employee Anthony DeSimone** says that poles/zeros and filter coefficients are not supplied, and recommends final-assembly calibration when response is material. Another **ADI employee, NevadaMark**, confirms that the frequency-response figures already include sensor and internal filtering. Adding a second internal sinc filter to a fit of the published overall response would double-count its magnitude response. [Manufacturer coefficient answer](https://ez.analog.com/mems/f/q-a/89325/adxl354-adxl355-frequency-response-for-seismic-purposes/185812), [manufacturer whole-chain clarification](https://ez.analog.com/mems/f/q-a/89311/adxl356-adxl354-frequency-response).

Official **UG-1030** describes access and external output filtering on the evaluation board; it contains no additional full sensor phase specification. Its ADXL354 schematic and BOM use **0.1 µF capacitors on the three outputs**. This matters greatly for a control experiment: with 32 kΩ, that adds an RC time constant of **3.2 ms** and a **49.74 Hz** corner. The stock evaluation-board response is therefore much slower than the bare analog output used as the screen anchor. Select output capacitance deliberately and buffer the outputs for the chosen acquisition front end. [Evaluation-board guide](https://www.analog.com/media/en/technical-documentation/user-guides/eval-adxl354-355-ug-1030.pdf).

## Can another 50–100 µs plausibly bracket the part?

The tested complete analog envelope is Butterworth second order:

`Henv(s) = wn² / (s² + sqrt(2) wn s + wn²)`, with `wn = 2π × 1900`.

Its low-frequency group delay is **118.463 µs**. Adding a pure 50 or 100 µs delay makes that portion **168.463 or 218.463 µs**. The native acquisition also includes the actual Sinc³ ADC, EMA and pipeline. Those stages belong on both sides of a consistent comparison and must not be counted twice.

These delay probes are plausible sensitivity experiments, but **there is no manufacturer-supported upper bound proving that 218.463 µs covers the real sensor response**. A magnitude plot cannot fix phase uniquely: adding `exp(-sT)` changes delay without changing magnitude. A resonance plus sinc response also changes gain and group delay with frequency; a constant delay cannot represent all such differences.

For scale, consider an explicitly **hypothetical standard causal averaging filter**, not an ADXL354 model:

`Hbox(f) = exp(-jπ fT) × sin(π fT)/(π fT)`.

For one boxcar with −3 dB at 1.5 kHz, `fT = 0.44294647` and group delay `T/2 = 147.649 µs`. For three cascaded boxcars, `fT = 0.26193817` and group delay `3T/2 = 261.938 µs`. The latter is approximately **143.475 µs greater than the two-pole envelope even before mechanical response**. This numerical illustration is motivated by the older manufacturer's sinc³ wording; neither this impulse response nor its clock is disclosed for the real part. It cannot be promoted as a measured delay estimate, but it shows why +100 µs cannot automatically be called conservative.

The calculations can be reproduced with the standard library:

```python
import math
for order in (1, 3):
    lo, hi = 0.0, 1.0
    for _ in range(100):
        x = (lo + hi) / 2
        amplitude = (math.sin(math.pi*x)/(math.pi*x))**order
        if amplitude > 1/math.sqrt(2): lo = x
        else: hi = x
    x = (lo + hi) / 2
    print(order, x, order*x/(2*1500)*1e6)
print("envelope DC delay us", math.sqrt(2)/(2*math.pi*1900)*1e6)
```

No mechanical damping value was invented or fitted in this review. The disclosed axis Q values help explain why X/Y and Z need distinct assembled measurements; they do not reveal the undisclosed electronics transfer.

External RC has its own explicit contribution:

| Output capacitance | Extra RC time constant | RC −3 dB corner |
|---|---:|---:|
| 100 pF | 3.2 µs | 49.74 kHz |
| 1 nF | 32 µs | 4.974 kHz |
| Stock evaluation board, 100 nF | 3.2 ms | 49.74 Hz |

These are nominal `R × C` calculations. Actual tolerance, parasitics, buffering and amplifier filtering remain additional measured terms. A low-pass RC's phase is frequency-dependent, so even its known time constant should be implemented as a filter rather than always replaced by a pure delay.

## What the existing native evidence supports

The root's retained [36-case response study](budget_sensors_screen_20261002/results.json) includes four primary records per latency value: the heavy and light endpoint, each with seeds 7 and 29. For the matched 1.9 kHz envelope and original 100 µs current-response target, the recorded maximum normal-force errors are:

| Common extra latency | Worst recorded error | Reserve to ±0.5 N |
|---|---:|---:|
| 0 µs | 0.405691294 N | 0.094308706 N |
| 50 µs | 0.457040717 N | 0.042959283 N |
| 100 µs | 0.484946478 N | 0.015053522 N |

These are executed native model histories, unlike the boxcar calculation above. The response and identified response are deliberately matched in those cases. The last row has a small remaining reserve and does not separately cover real axis-response mismatch, part/temperature spread or powered-tool vibration. Extra 150/200 µs screens could help locate the conditional delay limit, but would still not establish the actual sensor transfer. Replacing the assumed transfer by a measured one and rerunning is the decisive test.

The inspected result file SHA256 is `851b5c9d07ef36db6b2d15835651f423cd029258bea55d497ed8162198e0c5af`. Its frozen model and helper identities are recorded inside that file; the [input README](budget_accelerometer_envelopes_20261002_inputs/README.md) defines the finite endpoints and optimistic optional comparators. Faster-current-driver cases are a separate drive tradeoff.

## Practical acceptance measurements

1. **Measure the complete mechanical-acceleration-to-controller-frame transfer.** Mount the intended ADXL354C board on the actual S bracket and acquire it through the selected buffer/gain, ADC, clock, digital filtering and pipeline. Put a calibrated reference accelerometer as close as practical on a rigid fixture. Acquire its channel with a common timebase or calibrate its own phase. Measure complex gain and unwrapped phase per used axis, not just RMS or correlation lag. The known reference/DAQ response must be removed or included explicitly.
2. **Cover the frequencies that the control model uses.** Start with a low-amplitude 5 Hz–5 kHz sweep, with dense points through 0.5–3 kHz to expose the sensor and assembly modes. Use the actual response near the controller/filter/inverse bands and structural modes. Respect the datasheet's frequency-dependent mechanical headroom rather than choosing one excitation amplitude for every frequency. Repeat orientation and tool/fixture configurations that change the mount. The manufacturer's shaker procedure is useful precedent; its published amplitude plot alone does not calibrate this assembly.
3. **Separate phase components and check their variation.** Measure the analog output and its reference while logging conversion edges and the delivered controller sample. This distinguishes sensor/RC/amplifier phase from ADC filtering, acquisition age, pipeline and scheduling. Repeat warm/cold and several parts. Match force, acceleration and estimated-normal measurement frames, including cross-axis delay. At 1 kHz, a 10 µs mismatch alone is 3.6°; whether it is acceptable must be scored in the native histories.
4. **Identify a causal transfer and rerun.** Fit a causal model to the complex measurements, retain residuals and noise PSD, and replay the exact native force cases with actual and identified responses separated. Do not cancel resonances or delay using noncausal preview. A scalar cutoff or DC delay can be an initial fit descriptor but is not the whole acceptance criterion. Require the intended error reserve after measurement/model uncertainty and calibration stress, rather than merely crossing below 0.5 N once.
5. **Test the powered sander and measurement chain together.** Measure acceleration peaks, PSD, clipping, bias rectification, temperature and sample timing with the tool powered and in contact. Check the sensor's mechanical headroom and overrange-recovery behavior as well as the ADC range. Electronics self-test can detect defects and reveal part of the chain, but it cannot qualify the actual mechanical mounting or the contact-force reconstruction.

**Recommendation:** retain the 1.9 kHz two-pole response as a clearly identified acceptance envelope and use its latency sweep to guide the prototype measurement. Do not label +50/+100 µs as a manufacturer-backed ADXL354 bound. The inexpensive analog sensor remains a plausible candidate; confirming and modeling its assembled complex response is the concrete next step needed to replace the assumption.
