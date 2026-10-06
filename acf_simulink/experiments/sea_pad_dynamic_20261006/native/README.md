# Native Simulink plant checks for the series-elastic screw and soft pad

This is an isolated research plant, built from ordinary Simulink blocks. It
replays recorded current requests from the separately executed controller
study. It independently checks the physical equations and numerical execution;
it does **not** independently implement or qualify the controller in Simulink.
The public model and the user's open desktop model are preserved.

The saved research model and readable vector diagram are in
[`generated/sea_pad_dynamic_20261006/`](../../../generated/sea_pad_dynamic_20261006/).
Compact results and exact command histories are in
[`validation/sea_pad_dynamic_20261006/native/`](../../../validation/sea_pad_dynamic_20261006/native/).
The study contract is [`../study_contract.json`](../study_contract.json).

## What the blocks calculate

Six shallow subsystems separate the electrical drive, actuator/screw spring,
A–B–S body dynamics, distributed pad contact, physical stops and output
recording. Nine Integrators hold A/B/S positions and velocities, independent
screw-nut position and velocity, and winding current. Gain, Sum, Product,
Switch, saturation and matrix Gain blocks calculate the forces and
accelerations. The library reference is an ordinary Compare To Constant block;
there is no MATLAB Function, Stateflow plant or hidden MATLAB ODE solver.
MATLAB prepares constants, equilibria and the inverse mass matrix before a run.

The fixed upper spring is 20,000 N/m with zero upper damping. A is 1.5 kg,
B is 0.6 kg, and the B–S load cell retains 64 MN/m stiffness. Tool mass S
varies by case. Internal guide friction remains. The SEA nut has a distinct
position and a 75 g translating mass; motor/screw inertia appears through the
screw lead in a coupled A/nut mass matrix. Screw current therefore drives a
nut connected to B through the series spring, rather than treating screw
motion as direct contact force. Nut gravity is included in the equilibrium
current. Coil and screw electrical parameters include voltage/current limits,
resistance, inductance, back EMF and a 50 microsecond current-response target.

For a pad, 61 longitudinal strips have exact integrated circular chord-area
weights. Each strip evaluates its own surface height, compression and
unilateral spring/damper force. Negative force is clamped **locally before
summation**. A broad pad therefore changes the plant physically; the model
does not average the surface profile before applying a point contact law.
The ridge is 1 mm high and 10 mm wide, traversed at 100 mm/s.

The scored quantity is total **axial** contact force under the reduced
prototype assumption that sanding drag coupling is neglected. No local
pressure, wear or removal-rate score is calculated. These checks do not
qualify surface-normal force reconstruction in the presence of unknown drag.
They begin at loaded equilibrium; force-zero startup and delayed robot
recentering are not tested. Robot displacement is held at zero in the retained
two-second replays. A rigid-point branch exists in the builder, but the
native replays here do not establish its validation.

## Executed checks and their limits

The original seven physical replay cases score every one of their
**5,600,007 native 2.5 microsecond samples** in memory. Their saved graphs have
zero algebraic loops and pass save/reopen checks. Force-balance residuals are
below 3e-15 N. Three matched native refinement checks retain 800,001 original
base timestamps each; intervening half-step outputs are not all saved.
The separate preparation test scores 8,001 equilibrium samples and obtains a
5.7e-13 N force residual. Its older diagram/hash is historical preparation
evidence, not the final model's graph identity.

| Recorded replay | Native peak force error | Interpretation |
|---|---:|---|
| Tuned coil, point, 50 N / S=2 kg | 0.115392 N | One retained physical replay |
| Tuned SEA, point, 50 N / S=2 kg | 1.759383 N | Fails the force requirement |
| Tuned SEA, 77 mm pad, 50 N / S=2 kg | 0.047462 N | Earlier 10 mm-thick, 20 kN/m diagnostic |
| Partial-contact coil, 77 mm pad, 5 N / S=2 kg | 0.444323 N | Diagnostic with contact-event refinement limitation |
| Coil, 77 mm × 5 mm pad, 50 N / S=1.4 kg / 90° | 0.046075 N | Fresh conservative-pad worst-history replay |
| SEA, 77 mm × 5 mm pad, 5 N / S=2 kg / 45° | 0.118307 N | Fresh conservative-pad worst-history replay |

The three open-loop pilot histories are also retained. Their worst errors
are 8.815057 N for coil/point, 6.408994 N for SEA/point and 0.197960 N for
SEA/broad pad. These are physical demonstrations with a prescribed small
current perturbation, not tuned closed-loop performance comparisons.

Native versus regular-grid C replay agrees within 3e-11 N across these
histories. Agreement between two implementations of the same equations does
not establish that the physical pad parameters describe a real pad.

The engine controller's original acquisition events split integration steps.
Comparing its force with the native replay at 10 kHz timestamps leaves a
**32.181125 microN SEA point discrepancy**, which **fails the unchanged
10 microN point guard**. This failure is preserved in `replay_results.json`.
The earlier broad SEA discrepancy is 17.603779 microN and passes its declared
100 microN footprint guard. A separate C refinement assessment does not
remove the point failure. Native/regular-C parity and original event-split
parity are reported separately.

The partial-contact replay reaches a minimum active footprint of **7.852%**
while total contact remains 4.696–5.444 N. It verifies that local patches can
unload without declaring whole-pad contact loss. However, halving the native
timestep changes the contact-edge force waveform by **0.331836 N**, despite
nearly unchanged peak-error metrics (0.444323 and 0.444353 N). The discontinuous
Kelvin damping at local contact events is not demonstrated to be numerically
converged. This case must remain a diagnostic, not force qualification.

Four deliberate out-of-envelope software probes additionally execute
**6,230 output samples**. Whole-contact loss, configured flange travel,
configured SEA spring deflection and configured pad compression each stop
the run and are marked failed. These short probes exercise the guards;
their deliberately reduced limits do not qualify the actuator's real travel.
They leave the saved model hash unchanged. Failed preparation/check attempts
remain recorded in their batch logs.

The bounded append for the user's **77 mm × 5 mm** pad uses conservative
50 kN/m total pad stiffness, zero pad damping and a 2.5 mm compression guard.
Its two independently replayed worst histories and final default identity are
listed in `pad5mm_replay_results.json` and `validation_summary.json`. Each
scores 800,001 native steps, bringing the nine base histories to **7,200,009
fully scored native samples**. Both remain fully in contact and pass the
unchanged 100 microN footprint guard against regular-grid C and the original
split-event recordings. The native/original split-event differences are
1.08e-10 N for coil and 2.82e-12 N for SEA. Their force-balance residuals are
below 3e-14 N. Both saved/reopened diagrams have zero algebraic loops.
No extra refinement simulations ran for this bounded append. It changes
neither controller gains nor the prior results, and it is physical replay
validation rather than an independent native controller test. The final model
defaults to the fresh coil example. Its SHA256 is
`8134f122efe88b0c18d4da80fbea13e059fefa1d453d4bc3e0db47addeedf2e0`.

## Reproduce without retaining large arrays

Read the parent storage forecast and check available disk/RAM first. Use one
private `matlab -batch` process, private preferences, cache/code and SDI.
The largest observed earlier temporary footprint was 180,756,025 bytes.
The worker allocation is 10 MiB retained and 600 MiB temporary, with at most
300 MiB temporary arrays. Do not run multiple MATLAB copies for this worker.

`prepare_native_replay.py` reads a retained cases JSON and regenerates
regular-grid C physical reference samples into an owned temporary folder.
It limits a batch to three cases. It supports gzip command histories,
decompressing them only into that temporary folder. Then
`run_native_replays(root, manifestName, resultName, runRefinement)` builds,
runs, scores, saves and reopens the ordinary-block native model. For the
bounded two-case 5 mm append, `runRefinement` is false, so only the two
requested native histories execute. This helper overwrites the standalone
research model; preserve its current version before reproducing older cases.

All full reference arrays and SDI/build data are released after scoring.
Selected representative 500 Hz CSV traces are retained as lossless gzip
archives, with roundtrip hashes in `trace_archive_manifest.json`. Python
`numpy.loadtxt` and `pandas.read_csv` can read their `.csv.gz` files directly.
Historical result JSONs retain the original uncompressed CSV paths; recover
the exact original bytes from gzip when a consumer needs those paths.
Temporary binary-reference hashes are historical execution records; those
binary files are deliberately absent until regenerated. Do not claim that
present-day missing-file checks pass.

The original pilot model/sources are under `pilot_frozen/`, and the executed
partial-contact model/sources are under `partial_frozen/`. Their manifests
bind exact saved bytes. The research default may change for a later replay,
but these frozen versions and previous result JSONs remain unchanged.
