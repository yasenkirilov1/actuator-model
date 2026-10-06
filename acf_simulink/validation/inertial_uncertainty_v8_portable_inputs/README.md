# Reproduce the paired calibration records

This folder is portable after ZIP extraction. The two recipes are the exact
snapshots executed for the original and tighter-static four30-second cohorts.
Use the uninstrumented source in `source/`; the instrumented diagrams in
`models/` are retained for inspection and must not be instrumented again.
Selected native full histories remain separately retained; the manifest binds
their original paths and hashes without copying all eight large files here.

Requirements: Linux MATLAB R2026b with Simulink, GNU `sha256sum`, the usual C
compiler for Accelerator, at least12GiB available RAM,10GiB free persistent
storage, and adequate `/dev/shm` capacity. Run only one worker at a time. The
native source, selected parameters, helpers and recipe hashes are bound in
`manifest.json`. Recorded absolute paths in retained JSON are original-host
provenance; the wrappers resolve their actual inputs relative to this folder.

From this extracted folder, with MATLAB on PATH:

```sh
mkdir -p private_preferences
MATLAB_PREFDIR="$PWD/private_preferences" matlab -batch 'run_original_long'
MATLAB_PREFDIR="$PWD/private_preferences" matlab -batch 'run_calibrated_long'
```

Each command calls its exact recipe with explicit source/candidate/prior files,
Accelerator mode and tmpfs SDI. It creates a private clone/cache/code/SDI and
writes new evidence to `rerun_output/original/` or `rerun_output/calibrated/`.
Existing results are never overwritten. The native source/candidate/16 helper
hashes must match the recorded storage-only bitwise proof. Each process restores
its SDI preferences and removes only its own newly created temporary SDI folder.
The source of the second command is the recorded original4 configurations in
`data/`; rerunning the first command is not a prerequisite for the second.

For disk-backed SDI instead, call `run_original_long('disk')` or
`run_calibrated_long('disk')` in a batch expression. Disk versus tmpfs was
qualified as a storage-only, full-history bitwise change for this native source
and candidate; it does not change equations, control, solver or scoring.

The tighter cohort halves exactly16 static residuals. It retains identical
noise amplitudes/seeds, actual and identified AA/pipeline/robot mismatches,
physical mechanics/startup equilibrium and100us current response. The guard
and frozen pre-simulation configurations are in `data/`. Each recording has
3000001 native points and24 bumps; every true-normal-force sample is scored.
The chart CSVs contain all-native min/max force envelopes and complete-history
scalar peaks. These finite pairs do not prove an exhaustive tolerance box,
full-range combined uncertainty, actual hardware calibration or sanding wear.
