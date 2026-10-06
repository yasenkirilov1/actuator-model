# Reproduce and audit the measured-inertia results

Use [GETTING_STARTED.md](GETTING_STARTED.md) for ordinary runs. The current model is `generated/acf_force_control_simple.slx`. Studies use immutable inputs so later diagram cleanup cannot silently change their provenance. MATLAB and Simulink R2026b were used.

## Final nominal grid

The exact grid is **3,500 unique four-second recordings**:

- S = 0.1, 0.5, 1.0, 1.4 and 2.0 kg; B = 0.6 kg.
- Normal force = 5, 10, …, 50 N.
- Fixed orientation = 0°, 15°, …, 90°.
- Physical sanding drag ratio = 0 or 0.3.
- Clean condition and noise seeds 7, 29, 97 and 1729.

Clean cases retain filtering, delay and acceleration quantization. The controller uses measured drag and a causal predictor, without reading plant drag or surface profile. Every native sample is scored, including startup; contact/travel stops and spring packing are guards. **All 3,500 recorded cases pass**, with exact/disjoint coverage, all 700 five-seed groups and 1,400,003,500 force samples audited. The worst error is 0.315810929 N.

Frozen scientific SLX SHA-256:
`73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643`.
Final [candidate MAT](validation/inertial_final_candidate_20260930.mat) SHA-256:
`d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7`.
It includes 20 mm approximate spring geometry and the 100 µs current target.
See [V8 freeze manifest](validation/normal_contact_trial_inertial_v8_manifest.json).
The recorded grid has two execution segments with unchanged model, candidate,
helpers, native logs, integration and scoring:

- **r1:** recipe SHA-256
  `04ae0ae95e61d44695338b1ac6f58e1afec8329825562c67121be55af116a2db`;
  1,060 validated cases with private disk-backed SDI storage.
- **r2:** recipe SHA-256
  `7cef2cdd2d302ccbaaeb877406b067843f64d606f8b8ba78b8cb2d5e1af5516f`;
  the remaining 2,440 identities with owned temporary storage under `/dev/shm`.

The current `experiments/qualify_inertial_mass_controller.m` is the r2 recipe.
The r1 snapshot is retained in
`validation/final_grid/part3/r1_source_archive/`. Each continuation validates
the predecessor configuration, hashes and complete JSONL prefix before
reusing its rows. The aggregate audits all five identity fields for missing
or duplicate cases and preserves the execution-recipe binding per row.
See `validation/normal_mass_controller_final_grid_20260930_resume_plan.json`
and the four predecessor audits. Their prepared status describes the saved
plan; final execution status belongs to the result and coverage files.

Earlier recovery checkpoints remain separate and are excluded from the 3,500 unique cases. Each final failure retains its complete native trace; first/worst traces and durable metric checkpoints are retained too. One interrupted r1 worker had four complete JSONL rows beyond its last MAT checkpoint; those rows were independently validated and included once in its 268-row prefix. An interrupted process is not itself a model failure.

Run in an isolated batch process with private MATLAB preferences, a new output folder/tag, and the extracted immutable V8 source. The original source folder is:

```
/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v8-contract-20260930/frozen-inertial-v8
```

The versioned delivery includes the same bytes under
`acf_simulink/validation/final_grid/immutable_execution_source/scientific_source/`.
Use that extracted directory as `sourceFolder`; its candidate is
`inertial_final_candidate_20260930.mat`, and its `helpers/` are the matching
scientific helpers. The adjacent `source_manifest.json` binds all 27 source,
recipe and audit files. The public diagram is separately bridged to this
scientific model; do not substitute its SLX in a recipe that checks the V8 hash.

```matlab
addpath('acf_simulink/experiments');
sourceFolder = '/path/to/extracted/frozen-inertial-v8';
candidateFile = fullfile(pwd,'acf_simulink','validation', ...
    'inertial_final_candidate_20260930.mat');
options = struct('massS_kg',[.1 .5 1 1.4 2], ...
    'forceValues_N',5:5:50,'angleValues_deg',0:15:90, ...
    'muValues',[0 .3],'seeds',[0 7 29 97 1729], ...
    'allowFiniteCell',true,'simulationMode','accelerator', ...
    'checkpointEvery',8,'clearSDIEvery',1, ...
    'keepNativeWorstInMemory',false,'retainPlotExcerpts',false, ...
    'sdiStorageParent','/dev/shm', ...
    'partitionIndex',1,'partitionCount',4,'tag','_repetition_part1');
qualify_inertial_mass_controller(sourceFolder, ...
    fullfile(pwd,'repeated-grid','part1'),'full_range_mass', ...
    candidateFile,options);
```

For four parallel workers use indices 1–4, separate folders/tags and separate preference directories. Each receives 875 interleaved cases; a single worker can use partitionCount = 1. Four workers use substantial RAM. Clear SDI between cases and release histories after scoring. Do not run experiments inside the user's desktop model.

The recipe creates a unique, marked SDI folder, restores the previous storage
preference, and removes only that owned temporary folder. A four-second run
temporarily uses about 219 MB of SDI storage; a ten-second run uses about
548 MB. Check both RAM and `/dev/shm` space, especially for longer histories.
To use disk instead, omit `sdiStorageParent`; label the execution path.
Do not raise the solver step above 10 µs: the 100 ksample/s acquisition is
part of the model. Accelerator and storage reuse improve runtime without
removing that acquisition.

The paired final-source storage pilot retained all 72 logged channels and
found bitwise parity with disk storage. Its measured warm ten-second runtimes
were 39.782 s on disk and 6.809 s in RAM-backed storage, a 5.84× improvement
under the recorded concurrent load. The final four-worker continuation
typically took 2.0–2.8 s per four-second recording; these are host/load
measurements, not general MATLAB performance guarantees. The pilot recipe
is `experiments/benchmark_inertial_tmpfs_storage.m`, and its immutable
results are `validation/normal_mass_controller_tmpfs_sdi_pilot_20260930_v8_results.json`.

For a fresh rerun, leave `resumeReportFile` and `resumeAuditFile` empty.
For an interrupted repetition, use the documented predecessor audit rather
than combining rows by position or silently overwriting a result. Choose a
new output folder and tag for every execution.

## Other retained studies

Recipes in `experiments/` copy their immutable model, helpers and recipe, record hashes and score native logs. The report identifies each source version. Cohorts cover controller gains/strategies, springs/dampers/rigid replacements, actual and identified mechanical variation, joint calibration errors, repeated bumps, realistic windings/voltage, actuator comparison and 50/100/150 µs current-response alternatives.

The 20 mm spring geometry rescore reuses native force histories analytically; it is not another force experiment. Native runs, analytical sizing, read-only decompositions, setup errors and interrupted executions are distinct evidence classes. Use each original result JSON, source, candidate and options to repeat it. Planned scripts are not executed experiments.

Specific final evidence and inputs:

- `normal_mass_controller_final_grid_20260930_solver_refinement_portable_summary.json`
  binds the 10/5/2.5 µs worst-identity histories, their full sample counts and
  the actual recipes. Frozen recipes/collector are under
  `final_grid/immutable_numerical_refinement_source/`. Two qualified fine
  histories differ by only 2.42 µN; the extra unqualified native audit-return
  and wrapper rejection are explicitly excluded from qualification.
- `inertial_mechanics_mixed_comparison_20260930.md` shows how to rerun the
  original three mixed-mechanics seeds and their known-actual k/c counterparts.
  `inertial_mechanics_mixed_source_20260930/` contains the instrumented SLX,
  original configurations, helpers and exact frozen recipes.
- `inertial_actuators_final_actual_acquisition_v8_20260930_notes.md` binds the
  nine final 4 s coil/screw records, configuration MAT and combined donor.
  `qualify_inertial_actuators_final.m` uses that donor, not an earlier single-
  actuator scientific model. The public promotion has its own full-history
  acceptance bridge; the mesh bridge for donor case 2 is six scalar metrics.
- `normal_voltage_final_grid_hot_mapping_20260930.md` describes the exact
  known-winding mapping, 3,498 sufficient bounds and two native exceptions.
  `normal_voltage_20260930_final_grid_missing_headroom_proof_source/` contains
  the exact model/helpers/recipe for the additional generic/hot31V pair.
  Final electrical count is 18 native runs over 17 distinct conditions.
- `inertial_uncertainty_v8_joint_long/` retains four original 30 s calibration-
  stress histories, 24 bumps each, scored at every native sample. Their worst
  error is 0.490396047 N. Tighter calibration repeats are a separate study;
  use their actual prepared configurations and source equality guards.

The original and tighter-static long inputs are portable under
`validation/inertial_uncertainty_v8_portable_inputs/`. Its manifest checks all
47 local source/configuration/recipe members, and its relative wrappers
`run_original_long.m` and `run_calibrated_long.m` take a fresh output folder.
The refined four 30 s cases halve exactly sixteen static residuals. Noise,
AA/pipeline, robot timing and current response stay unchanged. Full prepared
configurations and equality guards record that contract.

The further three refined mixed-mechanics cases are reproduced from
`validation/inertial_mechanics_calibrated_static_mixed_source_20260930/`.
Use its exact recipe and configurations; the known actual k/c remain fixed.
`normal_voltage_calibrated_static_headroom_20260930.json` records a read-only
pointwise mapping of these three and the four refined long histories, not
seven new real-winding simulations. The selected prototype envelope is
**32 V usable output**, conditional on the amplifier and hot current response;
the separate nominal-grid 31 V evidence retains its own assumptions.

Recipes record original absolute paths for provenance. The delivery manifest
maps each original source to its extracted member. Pass the extracted source,
candidate and fresh output directory explicitly; preserve the recorded inputs
and results. The native trace policy names selected full histories included
in the package and those retained in the workspace with hashes and rerun inputs.

To export the actual nominal worst without simulating again, run
`experiments/export_inertial_worst_replay.m` with the extracted project root
and a **new** HTML filename. It checks the native trace SHA and complete-history
peak before display reduction. The delivered replay's Worst error button seeks
the exact 2.00063 s native sample.

## Verify the delivered diagram

The public model is a cleaned combined diagram with selectors, scientifically equivalent coil behavior and the reviewed screw extension. Delivery proof compares complete native histories, including normal-force, sensor, motion, drag, feedforward, inverse and gate logs. Floating-point changes to a diagnostic are recorded separately from physical/controller behavior.

Public SLX SHA-256:
`48287e6a17bc9d3144e3d6a77c59ecc601ba4fa26a7ec748173dc89d9bf718de`.
`validation/normal_contact_trial_delivery_manifest_v8_20260930.json` binds its
acceptance, public prepared candidate, metadata-only correction and removal
of 33 destinationless wire stubs. Every connected edge, block parameter and
workspace value was preserved by the wire cleanup. The diagram PDF contains
13 lossless raster pages, with the image/source recipe bound separately.

Checks include native block structure, no algebraic loops, save/reopen, controller/actuator branches, mass-only/scan-only/common-delay edits, slow B+S = 2 kg example and both failure stops. Use the delivery manifest/checks belonging to the archive version. An older package remains historical evidence.

The HTML report/replay display recorded data. Label any display downsampling; never replace the native force metric with thinned data. Figure/source manifests bind plotted evidence, and the archive manifest binds included files. Finite-grid and calibration tests support the modeled envelope, not all continuous combinations or physical sanding qualification.

## Verify an extracted delivery

The historical aggregate reader intentionally embeds original absolute paths
and path-equality assertions. To rerun that historical audit elsewhere,
normalize paths in separate copies (including the four private r1 recipe
aliases), preserving the original certificates. Its archived `audit_recipes`
copy also has a different relative root. The portable verifier below audits
the delivered manifest and grid directly without modifying those certificates.
For new simulations, use a fresh output folder and no historical resume inputs.

Extract `generated/active_contact_flange_inertial_20260930.zip` into a fresh
folder and run from its root:

```sh
python acf_simulink/experiments/verify_inertial_delivery.py
```

This standard-library check verifies every delivered file's SHA-256/size,
all 3,500 unique grid identities, native sample counts and the worst metric.
It performs no simulation or hardware qualification. The package includes
selected complete native histories and all recorded grid configurations and
metrics; other large histories remain hash-referenced with frozen rerun inputs.
See the manifest's `nativeTracePolicy` for the exact selection. Original
absolute paths in historical evidence remain provenance, with archive member
mapping supplied by the delivery manifest.

Open `reports/active-contact-flange-report.html` directly for the offline report;
keep it in `reports/` so its relative guide and diagram links resolve. The
replay is `acf_simulink/generated/active_contact_flange_inertial_worst_20260930.html`.
Both are standalone HTML files. Their browser checks bind the displayed
artifact hashes. Preserve the original dated archive when creating later versions.
