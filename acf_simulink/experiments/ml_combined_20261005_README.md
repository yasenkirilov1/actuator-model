# Combined small learned policies, 2026-10-05

This screen applies the trained fast force residual and trained robot
recentering imitation to one private native physical model. The force residual
uses the previously selected gain of 1 and retains its ±0.05 A limit. Both
weight sets are fixed before this screen; no result is used for retuning.
The existing PI/feedforward, current/voltage limits, robot delay/gain envelope,
both force-stability gates and queued-command dynamics remain present.

Four new identities were predeclared, using seeds 3101–3104. Each has an
eight-second deterministic baseline and an eight-second combined-policy
recording. Initial excess-force displacement is zero; the requested initial
position offset supplies the recenter demand directly.

| Case | Parameters | Baseline peak normal-force error | Combined peak error |
|---|---|---:|---:|
| 1 | S = 0.1 kg, 5 N, 0°, μ = 0, −1.5 mm offset | 0.026881934 N | 0.043450399 N |
| 2 | S = 2 kg, 50 N, 90°, μ = 0.3, +1.5 mm offset | 0.348118388 N | 0.355113428 N |
| 3 | S = 1 kg, 25 N, 45°, μ = 0.3, +1 mm offset, actual 85 ms delay/55 ms response | 0.175370945 N | 0.185500319 N |
| 4 | Retained tight-static heavy calibration/filter/timing corner, S = 2 kg, 50 N, 90°, +1.5 mm offset | 0.452692472 N | 0.449780766 N |

The first three cases have nominal sensor/model calibration and zero nominal
axial path velocity. Case 4 retains the source corner's sensor, mechanical,
filter/timing and identification residuals, as well as its programmed nominal
path. It has a new seed and a larger initial recenter offset than the earlier
retained qualification history. Its new result is separate from that history.

All eight recordings pass the ±0.5 N force requirement and physical/ADC
guards. All 19,200,008 native physical samples are scored, including startup.
The learned combination improves one peak by 2.912 mN and worsens three by
16.568, 6.995 and 10.129 mN. The greatest baseline/combined peaks are 0.452692
and 0.449781 N; this narrow change is not evidence of a robust advantage.

Both learned actions are active: the combined recordings have 3,009 nonzero
robot command frames and 320,004 nonzero fast current-residual frames. There
is no ADC clipping, gate-closed correction or sign/gain-envelope violation.
Direct affine-network replay agrees with native inference to 1.30×10⁻¹⁸ m
for the robot policy and 2.43×10⁻¹⁷ A for the force residual. Every paired input
structure is identical except for the robot-policy enable flag; the global
force-residual gain of 0 or 1 supplies the other intervention.

The source is native model SHA256
`b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5`.
Robot weights are SHA256
`1fc08b808bc43986bd927532d60edf27a930bcfe6ef5a5ff4f545f391406e52e`;
force weights are SHA256
`e21982cf35ed2790e831bab0cdd45f76271bfb90b872195c9493bce094fc8b05`.
The private source uses ADS131M08 30 ksample/s, matched 8 kHz response
envelopes and the real hot-coil model Kf = 24.4 N/A, R = 8.896 Ω, L = 4 mH,
32 V usable bipolar output and a 100 µs current-response target.

Run `ml_combined_20261005_run(pwd, freshOutputFolder)` in an isolated
`matlab -batch` process with a private `MATLAB_PREFDIR`. The runner preserves
the frozen source, copies the two reusable native builders, weights and
helpers, uses private cache/code and owned RAM-backed SDI storage, and restores
its SDI preferences on exit. It keeps selected complete native histories and
all physically failed histories; none of these eight recordings failed.

`validation/ml_combined_20261005/results.json` contains complete-history rows;
`validation/ml_combined_20261005_figures/summary.json` is the audited paired
summary. That figure folder contains the SVG paired comparison and worst
trajectory. Generate them with `ml_combined_20261005_plot.py`, passing the
native result folder and a fresh figure-output folder.

**Recommendation:** retain deterministic PI/feedforward for fast force control
and deterministic gated robot recentering. These two fixed small networks are
usable experimental alternatives, but their finite tests do not establish a
consistent performance advantage. No public model or historical delivery
package is changed. This is not full-range, full residual-box or physical
sanding qualification.
