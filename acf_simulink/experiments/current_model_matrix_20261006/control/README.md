# Current-plant controller matrix

This folder owns the compact controller comparisons. The completed
`sea_pad_dynamic_20261006` source, contracts, locks, evidence and compile stamps
remain read-only. The runner imports its ABI without writing bytecode and
compiles into an owned temporary directory, removed after each batch.

`controller_plan.json` declares the finite grid, bounded gain bank, development
corners, seed separation and storage before execution. The root's immutable
`study_contract.json` defines an ideal rigid 77 mm flat plate and binds the
development/evaluation axes. Development/evaluation require that exact contract
and the reviewed engine-worker source/API hashes recorded before hard execution.

The six groups distinguish the current soft-pad controller, the same gains
transferred to no pad, and independently developed no-pad gains, for each
actuator. There are 54 final cases per group. The no-pad search uses 18 fixed
families on four declared corners per actuator. It preserves every stopped,
failed, rejected or rating-ineligible trial. No tuning follows final evaluation.

The force flag requires completion and every integration endpoint within
±0.5 N. Physical, peak-current/speed, raw-cell/accelerometer and motor RMS-current
flags remain separate. Coil continuous thermal capacity is unknown. These are
2 s axial-force development comparisons from loaded equilibrium with a fixed
robot; engagement, recentering, normal force with unknown drag and physical
thermal operation are outside their scope.

Run the two existing soft-pad worst-case identities for private-source parity:

```sh
python acf_simulink/experiments/current_model_matrix_20261006/control/bounded_runner.py pilot
```

Once the root contract is locked, develop both no-pad controllers **before**
running any final group:

```sh
python acf_simulink/experiments/current_model_matrix_20261006/control/bounded_runner.py develop --actuator 0 --contract ROOT_CONTRACT.json
python acf_simulink/experiments/current_model_matrix_20261006/control/bounded_runner.py develop --actuator 1 --contract ROOT_CONTRACT.json
```

`lock --actuator 0 --branch soft --name coil_soft --contract
ROOT_CONTRACT.json` creates an unchanged-gain lock, with corresponding names for
the screw and transfer groups. `banks --contract ROOT_CONTRACT.json` publishes
the four selected locks/configs before any acquisition or final outcomes.
`evaluate --lock LOCK.json` evaluates a frozen lock.
Locks reference frozen source parameters and retain only geometry/controller
deltas. Compact JSONL journals contain scalar metric columns and exact config
hashes; `metric_schema.json` names the columns and `journal_rows` decodes them.
Each completed row is flushed immediately. The per-group summary supplies coverage,
sample count, failure/rating counts and the largest force-error identity.

Acquisition/native workers can import `bounded_runner.py` read-only and call
`resolve_record(lock_path, record_id)` for exact parameters and original
metrics. `private_engine(source, expected_sha256)` yields the private library
and build metadata. They own any diagnostic command/trace captures within their
separate allocations. For the reviewed current fork, the library's
`_matrix_api.simulate` is its event-aware wrapper; it preserves both transition
limits and extra-limit counts. The original ABI is used only for the original
soft-pilot source and unchanged struct/field checks. No original compile routine
is called.

Audit the completed compact evidence without new simulations:

```sh
python -B acf_simulink/experiments/current_model_matrix_20261006/control/verify_controller_matrix.py
```

`matrix_completion.json` binds the six final groups and native representative
identities. The audit reconstructs all 468 primary configs, verifies exact
54-case coverage and fresh-seed separation, rechecks rating/force flags and both
rigid-event limits, and verifies the selected gain family from all development
outcomes. `completion_audit.json` is a refreshed verification receipt; scientific
locks, declarations, journals and completion results remain immutable.
