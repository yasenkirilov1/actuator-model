# Bounded observer design bank

This private candidate changes only the measured-force upper observer's Q/R
design weights and the regularized cell-deformation derivative cutoff. The
upper spring remains **20,000 N/m, zero external damping, unlimited ideal
length/travel**. The lower connection remains rigid apart from the explicit
**64 MN/m B–S load cell**, with zero structural damping. Physical guide
friction, coil, controller PI/notch profile 1, sensors, noise, calibration,
acquisition and timestamps remain identical to the corrected v3 candidate.

The 40 observer profiles combine the ten rows below with derivative cutoffs
150, 400, 800 and 1,500 Hz, in that order. The position RMS column is the
observer's **algorithm covariance weight**. It does not replace the actual
1 µm sensor-noise assumption.

| Covariance profile | Force design density, N/√Hz | Acceleration design density, m/s²/√Hz | Algorithm position RMS, µm |
| --- | ---: | ---: | ---: |
| 1 | 0.05 | 0.04 | 1 |
| 2 | 0.005 | 0.004 | 1 |
| 3 | 0.0005 | 0.0004 | 1 |
| 4 | 0.05 | 0.004 | 1 |
| 5 | 0.005 | 0.0004 | 1 |
| 6 | 0.005 | 0.004 | 2 |
| 7 | 0.005 | 0.004 | 5 |
| 8 | 0.0005 | 0.0004 | 2 |
| 9 | 0.0005 | 0.0004 | 5 |
| 10 | 0.05 | 0.04 | 5 |

All profiles retain the explicit local acquired axial-cell source, identified
holder inertia and causal four-state structure. The force-balance derivative
is critically damped and proper; only its cutoff changes. Lower cutoffs reduce
noise amplification but approximate the physical finite-cell correction over
a narrower band. Development recordings must determine that tradeoff.

The locked bank contains 880 prepared configurations: 40 profiles × the same
11 physical development cases × A = 0.4/1.5 kg. Preparing a configuration does
not establish that it was executed. No fresh-test result is used for tuning.
Root runs bounded endpoint probes first and owns force scoring, histories,
selection and report integration.

For one-based observer profile `P` and zero-based case `C`, the zero-based
native scenario index is `22*(P-1) + 11*M + C`, where `M=0` selects A=0.4 kg
and `M=1` selects A=1.5 kg. `P=4*(covarianceProfile-1)+derivativeProfile`.
`gainID` aliases this observer profile for the existing evaluator; the fixed
PI profile is separately named `controlBaseGainID=1`. The highest-force heavy
90° endpoint is case 7, so the first probe set is `7 + 22*j`, j=0…39.

The preparer asserts every physical, sensing and PI/controller section is
unchanged and every prepared non-observer coefficient is identical to its
parent. Profile 4 (nominal Q/R and 1,500 Hz) reproduces all 22 corrected v3
observer coefficient structures exactly. Coefficient caching reuses only
deterministic design math, keyed by every design input; no simulation state
is cached. Per-case equilibrium and derivative values are recomputed.

The builder uses a private native graph, reuses the verified explicit local
source installer and numerical −1/0/+1 N wiring guard, and exports ordinary
native blocks. The guard's short perturbation harnesses are excluded from
force qualification. The seal compiles the unchanged C bridge, checks native
case 0 against Coder's parameters and binds the model, helpers, configuration
bank, code and library. A separate MATLAB process reopens a saved model copy,
compiles both A masses and verifies zero algebraic loops and authorized
sensor sources. These are implementation checks, not hardware qualification.

Frozen bank: `validation/upper_force_observer_20261005/bank_observer_noise_v4/`.
Native export: `validation/upper_force_observer_20261005/native_observer_noise_v4/`.
Recipes are `prepare_and_build_observer_noise_v4.m` and
`seal_native_noise_v4.py` in the parent experiment directory. Reproductions
must use new private output directories and preserve all earlier inputs.
