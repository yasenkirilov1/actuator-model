# Robot force-stability gate: fast-variation review

The robot should receive a recentering command only after the estimated force has stayed near its target and sufficiently steady. The existing gate can remain ready during a fast force oscillation because its force checks occur every 10 ms. A 10 kHz force-variation check with a continuously reset settling timer closes this gap in the retained trajectory.

This is a read-only, offline replay of one matched/noisy native trajectory. No model or helper was changed, and no altered closed-loop response is claimed. The accompanying [JSON](force_stability_gate_variation_review.json) records the source hashes, complete parameter sweep, recursions, timing, and validation results.

## Trace and exact replay

The source is [case_0001.mat](inertial_uncertainty_finite_provisional/inertial_uncertainty_repeated_screen_20260930_native_traces/case_0001.mat), the 10 s repeated-bump finite-cell test: S = 2 kg, target 50 N, horizontal flange, friction coefficient 0.3, sensor-noise seed 29, and no injected calibration, mass, timing, or robot-prediction mismatch. The fifth bump starts at 6.8 s and has its apex at 6.85 s.

The proposed gate uses the reconstructed workpiece-normal force, `r.normalContact.data(:,4)`, sampled every 100 µs. Actual normal force, column 1, is used only to assess the replay. Bump times select comparison windows only; the gate contains no surface-profile or bump-clock input.

The native gate has the following behavior:

- Every 10 ms it checks the **instantaneous sampled** normal-force error against ±0.2 N. This band is not applied to the filtered mean.
- It filters that sampled error with a 100 ms first-order trend filter. The Discrete Transfer Fcn realization adds one 10 ms sample of delay.
- It checks the filtered trend change against 2 N/s and requires 11 consecutive stable samples, covering 100 ms.

Replaying those equations reproduces the logged filtered error to 1.1 × 10⁻¹⁷ N and the logged rate to 7.8 × 10⁻¹⁶ N/s, with zero `stableNow` or `ready` mismatches. The observed failure to close is therefore consistent with the actual gate implementation.

At 6.85061 s, actual force error is +0.5444 N and the reconstructed estimate is +0.1223 N; native readiness remains true. The estimate itself contains a 1650 Hz component. Its force RMS in the 1–2.5 kHz band is 0.1239 N in 6.82–6.88 s, versus 0.0356 N in the quiet 6.3–6.7 s window. The fast variation is present in an available sensor-derived signal even though a single sampled value can sit near target.

## Small native-block addition

For error `e = Nestimate - Nreference`, maintain a fast, causal mean and variance:

```text
Ts = 0.0001 s
meanTime = 0.1 s
varianceTime = 0.02 s

mean[k] = aMean * mean[k-1] + (1-aMean) * e[k]
deviation[k] = e[k] - mean[k]
variance[k] = aVariance * variance[k-1]
              + (1-aVariance) * deviation[k]^2

aMean = exp(-Ts/meanTime) = 0.9990004998
aVariance = exp(-Ts/varianceTime) = 0.9950124792
variationOK = variance <= 0.0064 N^2  # RMS <= 0.08 N
```

A Sum, Product, Gain, and Unit Delay realization is sufficient. Compare variance directly to the squared limit; a Square Root block is useful only for a human-readable RMS diagnostic. A Discrete Filter with direct feedthrough gives the same recurrence. Using a constant numerator in a Discrete Transfer Fcn adds a sample delay, so the intended timing should be explicit.

Retain the existing near-target and trend checks. Hold their `stableNow` result on the 10 kHz grid, combine it with `variationOK`, and reset the settling counter on **every** fast failure. Require a full 100 ms of continuous combined stability before `ready` becomes true. The robot continues to consume this readiness at its existing 100 Hz command rate; the force loop stays at 10 kHz.

Merely ANDing the new RMS flag into the old ready signal is insufficient. The RMS briefly dips to 0.0780 N at the exact failing peak, just below the 0.08 N limit. A continuously reset settling timer remains closed because the estimate has not stayed stable for 100 ms.

## Measured duty and noise discrimination

The 20 ms, 0.08 N setting first rejects the fifth bump at 6.8007 s. It keeps readiness closed through the 6.85061 s apex. The instantaneous variance flag can dip in and out during the bump; continuous dwell handles those dips.

| Comparison window | Mean estimated RMS | Maximum estimated RMS | Existing readiness | Proposed readiness |
|---|---:|---:|---:|---:|
| Quiet startup, 0.25–0.85 s | 0.0502 N | 0.0587 N | 100% | 100% |
| Before fifth bump, 6.3–6.7 s | 0.0511 N | 0.0615 N | 100% | 100% |
| Fifth-bump peak window, 6.82–6.88 s | 0.1060 N | 0.1518 N | 100% | 0% |
| Late quiet window, 9.5–9.9 s | 0.0518 N | 0.0648 N | 100% | 100% |

Across 6.24 s of broader flat comparison intervals, the mean RMS is 0.0498 N, the 99th percentile is 0.0585 N, and the maximum is 0.0648 N. The 0.08 N criterion produces zero rejects within those intervals. Readiness is 99.62% there, with the small reduction caused by a settling interval carried over from an earlier bump.

Across the full 10 s trace, existing readiness is 91.60% and the proposed fast timer is ready for 81.21%. After sampling that result at the existing robot-command rate, the held robot readiness is 81.50%; its fifth-bump closure occurs at 6.81 s. The table uses the continuous fast timer state. The reduction is concentrated around bumps and subsequent settling. No companion noise-free 10 s run was available; “quiet” here means flat portions of this same noisy trajectory.

| Variance time | RMS limit | Readiness at failing apex | Full-trace readiness | Interpretation |
|---|---:|---|---:|---|
| 20 ms | 0.05 N | Closed | 0.82% | Rejects ordinary sensor variation too often |
| 20 ms | 0.08 N | Closed | 81.21% | Good separation in this trace |
| 20 ms | 0.09 N | Closed | 83.85% | Also detects earlier variation in this bump |
| 20 ms | 0.10 N | Ready | 84.94% | First fifth-bump rejection is 6.8527 s, after the apex |
| 50 ms | 0.08 N | Ready | 81.63% | Integration is too slow to close before this apex |

The JSON also records 5, 10, and 50 ms windows and additional limits. The 0.08 N setting is a starting value supported by this trace, not a qualified noise threshold for every tool mass or prototype vibration condition.

## Meaning and required follow-through

Use the calibrated reconstructed force before a controller-only notch or rolloff, when that signal is available with correctly matched load-cell and accelerometer channel transfers. A notch that removes a physical mode from the gate input can also remove the variation this detector needs to see. The detector must be checked again on the final sensor/filter architecture.

The new gate does not anticipate a first unseen force excursion. In this trace the force briefly exceeds ±0.5 N at 6.80041–6.80048 s, before the RMS rejects at 6.8007 s. It also cannot withdraw robot commands already in the 80 ms command-delay queue. The fast force controller and mechanical design still carry the force-tracking requirement.

Actual force exceeds tolerance for 27 native 10 µs samples, totaling 0.27 ms. Existing readiness is true for 88.9% of those samples; the proposed continuous-dwell replay is true for 29.6%, all from the early excursion before enough variance has accumulated. These figures describe readiness on a fixed saved trajectory and do not rescore it as a passing run.

**Recommendation:** add the 20 ms variance check at 10 kHz, start with a 0.08 N RMS limit, and reset the 100 ms dwell on every fast instability. Then rerun the native repeated-bump test with the gate connected to recentering, followed by the selected mass/force/orientation/friction corners and independent noise seeds. Compare force tracking, stroke margin, readiness recovery, and queued robot motion while closed. This is the smallest addition that directly tests the user’s requested “semi-stable force” condition against fast variation.
