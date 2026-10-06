# Calibrated-static hot-coil voltage headroom

The seven retained histories support a **verified 32 V usable terminal-voltage envelope** for the conditional hot-coil transformation. The maximum exact demand is **31.107069 V**, leaving **0.892931 V** reserve at 32 V. A nominal **36 V supply** is a reasonable starting point only if the amplifier delivers at least this loaded output envelope and the required 100 µs current response.

This is a pointwise algebraic screen of existing native histories. It adds **zero native simulations**. All startup samples were retained. Four 30 s records and three 4 s records contain **13,200,007 samples** in total at 10 µs spacing; each generic 24 V history is complete and has zero native voltage or current clipping.

| Retained family | Seed | S (kg) | Reference (N) | Original native normal error peak (N) | Mapped hot voltage peak (V) | Reserve at 31 V (V) | Reserve at 32 V (V) | Samples above 31 V |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| long 30 s | 7 | 2 | 50 | 0.411872 | 29.117569 | 1.882431 | 2.882431 | 0 |
| long 30 s | 29 | 2 | 50 | 0.411243 | 29.591533 | 1.408467 | 2.408467 | 0 |
| long 30 s | 1729 | 2 | 50 | 0.415311 | 28.566356 | 2.433644 | 3.433644 | 0 |
| long 30 s | 29 | 0.1 | 5 | 0.075628 | 2.495437 | 28.504563 | 29.504563 | 0 |
| mixed 4 s | 7 | 2 | 50 | 0.373044 | 31.107069 | -0.107069 | 0.892931 | 1 |
| mixed 4 s | 29 | 2 | 50 | 0.376938 | 29.572088 | 1.427912 | 2.427912 | 0 |
| mixed 4 s | 1729 | 2 | 50 | 0.357305 | 29.691802 | 1.308198 | 2.308198 | 0 |

All four improved 30 s records fit 31 V; their maximum demand is 29.591533 V. Mixed seed 7 reaches 31.107069 V at 3.8701 s, after the programmed bump. Exactly one recorded 10 µs sample exceeds 31 V. This small excursion prevents a claim of exact unclipped equivalence at 31 V; it does **not** establish a normal-force failure under a hypothetical 31 V drive. No clipped counterfactual was simulated.

The mixed records use actual and identified lower-link stiffness 23,695.2 N/m and damping 48 N·s/m. The long records use 19,746 N/m and 60 N·s/m. The 16 static residual calibration errors are halved in both families; the original AA response, transport delays, noise, robot delay/response residuals, guide and other mechanical parameters remain fixed. These records complement the separately audited nominal 3,500-case grid and the separately executed winding cases; their case counts must not be combined as additional real-coil simulations.

## Exact current and winding transformation

The generic physical winding is Kf = 20 N/A, R = 4 Ω and L = 3 mH. The conditional real hot winding is Kf = 24.4 N/A, R = 8.896 Ω and L = 4 mH, with the same matched 100 µs current regulator. The resistance corresponds to 6.4 Ω × 1.39, an assumed 100 °C copper temperature rise rather than a qualified safe winding temperature.

Let `s = 20/24.4`, `i` be native generic physical current, `u` the same limited current command used by the native regulator, and `v` the coil-gap velocity. Native requested voltage is `Vg = (0.003/0.0001)(u−i) + 4i + 20v`. Both generic clipping flags are zero, so it equals the recorded applied voltage. The exact transformed demand is:

```text
ireal = s i;  ureal = s u
Vhot = (0.004/0.0001)s(u−i) + 8.896s i + 24.4v
     = 1.092896174863388 Vg + 2.920218579234974 i + 2.542076502732243 v
```

Each full native history passed both the native voltage-equation check and the direct-vs-transformed winding calculation. Maximum residuals are 3.55 × 10⁻¹⁵ V and 1.07 × 10⁻¹⁴ V respectively.

Every active current-unit PI gain, holding/acceleration/guide feedforward term, proper inverse current output and current initial condition must scale by `s`. The transformed PI gains are Kp = 0.245901639 A/N and Ki = 16.393442623 A/(N·s); Kd remains zero. The identified Kf residual is preserved: heavy records retain physical/identified 20/20.05 N/A as 24.4/24.461 N/A, and the light record retains 20/19.95 as 24.4/24.339 N/A. The inverse uses that residual identified Kf, rather than silently correcting it to the physical value.

Across all seven mapped histories, peak physical current is 2.465575 A and peak command is 2.518865 A, below the retained 3.5 A limit. Signed minimum/maximum, peak, RMS, initial states and native command/current ranges are retained in the [CSV](normal_voltage_calibrated_static_headroom_20260930.csv) and [JSON](normal_voltage_calibrated_static_headroom_20260930.json).

## Scope of the recommendation

Verify the chosen 36 V amplifier on the actual cold and hot winding: at least a 32 V bipolar terminal envelope at the relevant current, with a 100 µs load-specific response and acceptable phase/ripple. A 4 V amplifier-drop allowance is an engineering target, not a manufacturer guarantee. If the driver cannot provide that envelope at 36 V, a higher bus can restore delivered-voltage headroom; it does not improve the configured current bandwidth or force response once clipping is absent.

Conditional trajectory equivalence assumes the whole force/current transformation, matched resistance/inductance/back-EMF compensation, identical upper-rigid robot kinematics and sensing/mechanics calibration, and an amplifier that delivers the calculated voltage/current without extra phase, ripple or saturation. No Kf-versus-stroke/temperature, real amplifier dynamics, finite robot impedance or thermal safety is qualified by this algebra. The reported native normal-force errors belong to the original generic records, rather than to newly executed real-coil cases.

## Frozen evidence

- Base V8 model: `73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643`.
- Final candidate: `d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7`.
- Long instrumented model: `a6e461f018c9c1d5c784db2ae43781dddfa5b0fe8082caf42c49bb8d270c5009`; recipe `247d12963558899afe255fd5a7c8f24eb74e50b50234495839e53299de5dec0e`.
- Mixed instrumented model: `4ac684797c4a41d3a73c8146357baa97ba18098415b3940c942762330e2d3a4b`; recipe `3dbf23e27c29d15f7f3d91114e2fe24c494f67b6662497e74acd13ffbda48603`.
- All seven native trace hashes, source reports/configuration hashes, 16 helper hashes, reader recipe and aggregate recipe are retained in the aggregate JSON. Every source byte hash was checked again while building this document; MAT files were streamed for hashes without loading arrays.
