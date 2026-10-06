# Small learned robot-recentering policy, 2026-10-05

This experiment attempts supervised imitation of the existing safe robot
recentering policy. The deterministic fast force controller and physical plant
stay unchanged. No public model or delivered package is promoted by this study.

The scientific source is the private native acquisition model under
`validation/adc_acquisition_trial_20261002/`, SHA256
`b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5`.
The reference analog acceleration envelope and ADS131M08 settings are retained:
30 ksample/s converter output, matched 8 kHz analog response envelopes and
10 kHz fast force control. The drive uses Kf = 24.4 N/A, hot R = 8.896 Ω,
L = 4 mH, 32 V usable bipolar output and a 100 µs current-response target.

## Learning and constraints

The network input is the causal estimated distance from actuator mid stroke,
sampled at 100 Hz. The original S–A sensor and motion/force reconstruction
supply that estimate. The network does not receive surface truth, true normal
force, a future bump profile or future robot motion. Its supervised target is
the existing pre-gate robot recenter correction, recorded from native blocks.

Four positive and four negative ReLU neurons form an odd-symmetric policy.
The four hinge locations and four paired output coefficients are learned;
input sign weights and odd symmetry are fixed architectural constraints.
There are eight learned scalar parameters and 25 stored affine coefficients.
Training minimizes observed-command squared error with equal weighting of
stroke-error bins. Four initialization seeds are fitted, and untouched
selection trajectories select one. There is no reinforcement learning or
online adaptation.

Learning returns an equivalent deadband distance. The existing delay-bounded
gain scales this into robot velocity, and a separate hard directional and
amplitude envelope allows only the teacher's sign and no greater magnitude.
The original force-band/trend/dwell and fast force-variation/dwell gates remain
outside the network. Robot velocity/acceleration limits and queued-command
delay are unchanged. Closing a gate does not clear already-issued commands.

Inference uses ordinary native Gain, Bias, Saturation (ReLU), MinMax and Switch
blocks. `ml_robot_20261005_build_policy(mdl)` can apply the same inference
graph to another loaded private copy for a combined experiment. It requires
`p.mlRobot.enabled`, `inputScale_m`, `W1`, `b1`, `W2`, `b2`, and `zeroOutput`.
The trained structure is saved in `network_weights.mat`; use `enabled = 1`
for inference. The builder keeps the original teacher selectable.

## Data splits

Eight six-second native teacher trajectories train the policy. Four other
six-second teacher trajectories select initialization. Training varies tool
mass among 0.1, 1.4 and 2 kg, target force between 5 and 50 N, orientation
between 0 and 90°, noise seed, and positive/negative initial travel offsets
between 0.75 and 2 mm. Nominal axial path velocity is zero in these recenter
trajectories, so the nonzero robot correction is an actual learning target.
Each trajectory includes startup and the sharp bump at two seconds.

The recenter trajectories use zero sensor static residuals but retain a fixed
0.25% B-mass/Kf identification residual in the inherited lower-link force
inverse. The whole paired force strategy is identical, so these force-model
residuals are not an advantage granted to learning.

Eight untouched six-second test identities introduce 0.5, 1 and 1.8 kg masses,
new force/orientation combinations, new noise seeds and ±5 ms actual robot
delay/response errors relative to the identified predictor. Within those eight,
two range endpoints use untouched noise and travel conditions. These identities
are all paired with the teacher under exactly the same parameters and seed.

Three retained calibration/mechanics configurations are also tested in pairs
for eight seconds each. These include joint static residuals, filter/timing
errors and the nonzero nominal robot path. Their durations are shorter than
the retained 30-second qualification recordings; they are not a replacement
for that qualification. Their delayed-request metric includes the nominal
path as flagged in the result rows.

## Run the experiment

Use isolated `matlab -batch` processes, one at a time within this lane. Each
runner copies its model and helpers into its output folder, sets private cache
and code directories, creates owned RAM-backed SDI storage, clears SDI after
each case, and restores its prior SDI preferences on exit. Give each process
a private `MATLAB_PREFDIR`. Preserve existing output folders; choose a fresh
output path for a rerun.

From the repository root, the MATLAB collection command is:

```matlab
addpath('acf_simulink/experiments');
ml_robot_20261005_run(pwd, 'collect', ...
    fullfile(pwd,'acf_simulink','validation','ml_robot_20261005_collect_rerun'));
```

Fit the network with the scientific Python environment:

```bash
/home/tommie/.local/state/actuator-model/sensor-study-python/bin/python \
  acf_simulink/experiments/ml_robot_20261005_train.py \
  acf_simulink/validation/ml_robot_20261005_collect_rerun \
  acf_simulink/validation/ml_robot_20261005_training_rerun
```

Then execute the separate test trajectories:

```matlab
ml_robot_20261005_run(pwd, 'heldout', ...
    fullfile(pwd,'acf_simulink','validation','ml_robot_20261005_heldout_rerun'), ...
    fullfile(pwd,'acf_simulink','validation','ml_robot_20261005_training_rerun', ...
    'network_weights.mat'));
```

Create the audited SVG comparisons with `ml_robot_20261005_plot.py`, passing
the collection, training, held-out and fresh figure-output folders in that
order. `training_summary.json` binds every compact training source by hash;
the native result JSONs bind model, recipe, helper and parameter sources.

## Scoring and interpretation

Every physical sample is scored at the native 300 kHz integration clock,
including startup. Qualification checks actual workpiece normal force,
contact/travel stops, current, voltage, spring packing and force balance.
Plot excerpts are separate from these complete-history metrics. Every case
retains its robot-rate causal features, preparation settings and a display
excerpt. Selected complete native histories and all physically failing
histories are retained. Software rejections remain separate from physical
failures.

Command auditing reports the raw-network error and the error after the safety
envelope separately, plus nonzero command frames, gate violations, sign/gain
violations, queued motion during closure and direct affine-network replay
agreement. The network must do useful nonzero recentering; a constant-zero
policy does not pass this evidence requirement.

An imitation policy can demonstrate a small causal native ML implementation
without demonstrating an improved controller. The final paired results decide
whether learning has earned its extra complexity. This finite study does not
qualify powered sanding vibration, temperature drift, random communication
jitter, the whole joint residual box or physical hardware.

## Executed result

The eight training trajectories provide 4,808 robot-rate frames, including
4,682 nonzero issued corrections. The four selection trajectories provide
2,404 separate frames. Seed 29 is selected from the four initialization seeds.
On the recorded selection data, raw and bounded teacher-command error is at
floating-point precision; the largest error is 2.17×10⁻¹⁹ m/s. This describes
the recorded trajectories, not an exact equality for every possible input.

All 12 collection recordings pass, with 21,600,012 native physical samples
scored. The separate robot-policy test has 11 paired identities and 22 native
recordings, scoring 43,200,022 physical samples. All 11 teacher and 11 learned
recordings pass. Their worst actual normal-force errors are 0.417867976693 and
0.417867976702 N respectively; the maximum change in paired peak error is
9.15×10⁻¹² N. Gate-closed command and sign/gain-envelope violation counts are
both zero. Raw-network maximum teacher-action error is 2.71×10⁻¹⁶ m/s, and
no frame requires a material safety-envelope correction. Direct replay of
the exported network agrees within 1.73×10⁻¹⁸ m of its native logged output.

The reviewed comparison, active calibration-stress trajectory and learned
transfer SVGs are under `validation/ml_robot_20261005_figures_final/`.
The active plot distinguishes the recenter correction from the total request
containing the programmed nominal path. Eight nominal holdouts did not close
their gates; the active illustration therefore uses a calibration-stress
case with actual gate closures. Complete native histories are retained for
selected cases; compact figures retain every exact scored peak and do not
replace complete-history scoring.

**Recommendation:** keep the deterministic DeadZone/Gain recenter policy and
its two hard force-stability gates. The network is a successful compact
imitation, but adds neither a demonstrated performance advantage nor a simpler
implementation. Its weights remain available for reproducing the experiment.
