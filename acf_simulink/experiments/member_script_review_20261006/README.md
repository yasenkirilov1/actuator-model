# Group member script: bounded review checks, 2026-10-06

This is a code audit of the uploaded `afc_polishing_profile_3d.m.txt`, not a
replacement plant, native MATLAB replay or controller qualification.

Source SHA256:
`332aa45edd7fd97fb3f596180e3209b49640250d4072825a1405af92d0b9675f`.

`check_member_equations.c` transcribes solver lines 629–768 in their original
update order. Gaussian noise is disabled. The polishing/sanding cases retain
the original deterministic sensor-error terms. All seven cases are 20 s;
the flat PID comparison removes surface variations, spindle forcing and
sensor errors. PID off retains constant-current feedforward. A single
polishing case repeats at half the integration step.

The code allocates no full histories. Scalar metrics score every integration
step; two flat-case traces are downsampled to 100 Hz for the figure. The
reported omitted inertia is `1.1 kg * abs(acceleration)`, a diagnostic term
magnitude, not a corrected physical load-cell replay.

From the project root, reproduce into an owned temporary directory:

```bash
task_review_dir=$(mktemp -d /tmp/actuator-member-check.XXXXXX)
gcc -O2 -std=c11 acf_simulink/experiments/member_script_review_20261006/check_member_equations.c -lm -o "$task_review_dir/check"
"$task_review_dir/check" > "$task_review_dir/metrics.jsonl" 2> "$task_review_dir/robot_trace_no_header.csv"
```

Compare the seven rows with
`acf_simulink/validation/member_script_review_20261006/deterministic_checks.json`.
The retained CSV adds the header `case,time_s,contact_error_N`. The C output
contains neither a JSON wrapper nor CSV header; its numerical rows should
match within normal compiler/platform roundoff.

`make_review_figures.py` reads the retained compact JSON/CSV and regenerates
two SVGs plus `review_checks.json`. It also evaluates pad-area arithmetic and
a partial-contact counterexample. It does not execute a model. Regeneration
overwrites those audit figures and checks; preserve the frozen delivery when
comparing another source version.

The report is
`reports/reviews/group-member-polishing-model-review-20261006.md` and its
self-contained offline HTML sibling. `delivery_manifest.json` in the audit
validation directory binds source, audit artifacts, reports, delivery copies
and the current model/main-report hashes.

No original script, physical model, desktop MATLAB state or main report was
changed. The storage forecast is 600,000 retained bytes and 1,000,000 temporary
bytes, with zero new MAT/NPZ arrays. These outputs provide equation checks,
not seven qualified simulations of our flange.
