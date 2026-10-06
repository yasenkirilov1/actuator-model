# Hot-coil headroom across the final nominal grid

The combined finite-grid screen supports **31 V usable terminal output** under the stated nominal assumptions: **3498** cases have sufficient conservative bounds, and the remaining **2** identities have explicit passing native hot-coil runs with no clipping. The maximum triangle bound alone is **32.164688 V**; its excess is conservative rather than an observed headroom failure.

This is a read-only algebraic audit of **3500 existing, uniquely identified native cases**, scored over every 0–4 s sample. The algebraic calculation itself adds no MATLAB simulations. Two separately authorized native cases subsequently close the one identity absent from the earlier winding subset. The complete coverage audit passed, and every generic case has zero voltage and effort clipping. The final electrical evidence now contains 18 native simulations/17 unique conditions, including those two extra cases; the 3500 supported nominal identities are not 3500 real-coil simulations.

The most conservative bound occurs at S = 2 kg, B = 0.6 kg, 50 N, 90 degrees, mu = 0.3, seed = 7, global case 3497. Its existing generic peaks are 19.548871182 V, 2.912545186 A and 0.338458052 A current-tracking error. The grid has no retained peak gap-speed metric, so the native regulator equation eliminates that missing quantity exactly.

## Derivation

For the identified 100 µs current regulator, with applied current command u, actual current i and coil-gap velocity v:

`Vg = (Lg/tau)(u-i) + Rg*i + Kg*v = 30(u-i) + 4i + 20v`.

Preserve the mechanical force trajectory by scaling current commands and current-unit feedback gains by `20/24.4 = 0.819672131148`. Thus the real PI gains become `0.245901639344 A/N` and `16.393442622951 A/(N*s)`, and the identified lower-load inverse Kf is explicitly 24.4 N/A. Both native current-loop compensation and physical winding use the real known R/L/Kf.

From generic `Kf = 20 N/A, R = 4 ohm, L = 3 mH` to real `Kf = 24.4 N/A, L = 4 mH, Rhot = 6.4*1.39 = 8.896 ohm`, with identical tau:

`Vhot = 1.092896174863Vg + 2.920218579235i + 2.542076502732v`.

Substituting the generic regulator gives the exact equivalent expression:

`Vhot = 1.22Vg + 2.411803278689i - 3.813114754098(u-i)`.

Each case therefore has the conservative triangle bound:

`peakAbs(Vhot) <= 1.22*peakAbs(Vg) + 2.411803278689*peakAbs(i) + 3.813114754098*peakAbs(u-i)`.

The three maxima may occur at different times. The reported value is an upper bound on counterfactual requested terminal voltage, not an attained peak or a newly simulated result.

|Known S mass (kg)|Audited cases|Maximum voltage bound (V)|
|---:|---:|---:|
|0.1|700|20.655786|
|0.5|700|22.640209|
|1|700|25.421939|
|1.4|700|28.112787|
|2|700|32.164688|

## Native cross-check

The retained generic 24 V and real-hot 36 V pair at S 2 kg, 50 N, 90 degrees, mu 0.3, seed 1729 shares all 400001 exact 10 µs timestamps. The mapped requested-voltage waveform agrees with the real native waveform within **2.06e-11 V**. True normal-force trajectories agree within **4.76e-13 N**. The real 36 V and effective 31 V requested-voltage and normal-force traces are bitwise equal for this retained pair.

The separate successful final winding subset reaches **28.472994 V** hot requested terminal voltage, leaving **2.527006 V** at 31 V available output. Those native operating points pass; their result and this conservative full-grid bound have different scopes. Effective 19/20 V hot cases fail during the bump, and raising an already unclipped voltage envelope does not change the configured current-response time.

## Closing the two conservative exceptions

Only global cases 3492 and 3497 exceed the 31 V triangle bound: both have S = 2 kg, B = 0.6 kg, target 50 N, orientation 90 degrees and noise seed 7. Their drag coefficients are respectively 0 and 0.3. Case 3497 was already covered by a native hot-coil 31 V run, with all 72 channels bitwise equal to its retained 36 V reference.

The additional proof executes exactly two cases for global 3492: a generic 24 V reference and the identified real-hot motor with 31 V available output. The generic result matches the stored grid peak error exactly: **0.301496536196 N**. All **222** nonderived input parameters match its archived grid configuration exactly. The normalized real-hot run gives **0.301496536196 N**, with peak requested voltage **28.440406496 V** and **2.559593504 V** reserve. Every 0–4 s native sample, including loaded startup, is retained and scored; voltage and effort clipping are both zero.

The new pair's full true-normal trajectory differs by at most **1.99e-13 N** and its mapped hot-voltage waveform by **1.33e-11 V**. Feedback/feedforward/inverse gains use the verified 20/24.4 current scaling and the same 100 µs response/3.5 A limit. Physical current, current command and voltage are transformed as expected; the upper boundary reaction changes with the separately modeled A mass. All remaining physical states, tool/contact forces and sensor/gate histories match within numerical precision.

The combined result is therefore **3498 sufficient algebraic bounds plus 2 directly executed native exception identities**. It retains constant identified Kf/R/L, the modeled hot-resistance envelope, upper-rigid robot motion and the selected sensor/mechanical/controller assumptions. It provides no thermal, unknown-hot-Kf, force-versus-stroke or hardware qualification. Exact sources, parameter/metric checks, waveform residuals and counts are retained in [the exception proof audit](normal_voltage_20260930_final_grid_missing_headroom_proof_summary.json).

## Design meaning and limits

This mapping assumes constant identified Kf, matched hot resistance/inductance compensation, 100 µs current response, the same selected B/S mechanics and upper-rigid robot-side kinematics, and properly normalized force-domain control. The largest scaled real current command is 2.470694 A, below the 3.5 A limit. The native retained real-motor case includes A 1.5 kg versus generic A 0.4 kg; equal imposed kinematics make its force/gap trajectory equivalent in this source, without providing physical body-inertia qualification.

The 1.39 copper-resistance factor is an assumed 100 °C winding-temperature rise. Safe winding temperature, actual coil Kf versus position/temperature, amplifier output drop, current-loop bandwidth/noise and thermal dissipation require hardware characterization. Terminal 31 V is an envelope, not a guaranteed output specification from a 36 V amplifier supply. No all 60 joint-uncertainty headroom bound is given because those rows retain neither gap-speed nor current-tracking maxima.

The exact CSV/coverage/model/candidate/recipe hashes, assumptions, all envelope counts and retained native trace hashes are saved in [normal_voltage_final_grid_hot_mapping_20260930.json](normal_voltage_final_grid_hot_mapping_20260930.json). Source grid: [normal_mass_controller_final_grid_20260930_rows.csv](normal_mass_controller_final_grid_20260930_rows.csv); coverage: [normal_mass_controller_final_grid_20260930_coverage.json](normal_mass_controller_final_grid_20260930_coverage.json); final winding evidence: [normal_voltage_20260930_final_sensor_summary.json](normal_voltage_20260930_final_sensor_summary.json).
