# Independent native physical command replays for the current matrix

This bounded wrapper versions the ordinary-block SEA/pad native builder.
Recorded 10 kHz current requests drive the plant; this does not independently
implement or qualify the controller. All prior models, manifests, outputs and
the user's desktop session are read-only.

The builder supports distributed positive-stiffness contact, rigid point
contact and the newly contracted ideal rigid 77 mm plate. The latter follows
the continuous analytic maximum of the ridge across its footprint; ordinary
clamp/switch/polynomial blocks implement that contact envelope. Its acceleration
jumps at entry to and exit from the peak-height plateau. Both one-sided forces
are scored explicitly, alongside every ordinary native endpoint.

Before running, the root matrix contract and selected controller banks must be
frozen. `selection.json` must bind the matrix contract, current engine and API
SHA256 values, with up to four case objects containing `identity`, `group`,
full `config` (or a hashed JSON file path), `commands_csv`, original metrics,
and a `refinement` boolean. Commands are a complete 2 s gzip CSV containing
`time,current,robot,force` or `time,current`. Robot displacement must be zero.
Case names use letters, numbers, underscore and hyphen only. Alternatively,
`capture_from_locked_config: true` requests an observation-only command capture
in private temporary storage. Every nonruntime metric and the recorded event
limits must equal the selected final recording before native replay proceeds.
Its source, config and exact temporary command hashes are retained; bulk
commands are regenerated from those frozen inputs when reproduced.

`prepare_matrix_native.py` first checks this selection. It compiles the frozen
C current-matrix engine privately and regenerates temporary physical
reference force at every base endpoint plus states at controller timestamps.
No files in the old engine output directory are written.

`run_matrix_native.m` uses one private batch process and private preferences,
cache, code and SDI storage. Each native and refinement integration endpoint is
scored in memory; only compact metrics and selected compressed 500 Hz traces
are retained. Both native/regular-C and native/original-event-split comparisons
use the unchanged 10 microN point and 100 microN footprint force guards. Failed
guards remain failed. Native force balance, finite cell location/tare,
algebraic-loop absence and save/reopen receive separate checks.

The worker allocation in `validation/current_model_matrix_20261006/storage_forecast.json`
is 500 KiB retained and 600 MiB temporary, shared with the project-wide 5 GiB
array cap. One case's full refinement dataset is approximately 320 MB; reference
files for four cases add approximately 32 MB. No bulk arrays or saved research
model are retained. The source hashes and a temporary saved model hash identify
the executed plant; reproducing it rebuilds a private model from the frozen
builder and explicit configuration.

Four completed selected histories have now executed: two soft-pad replays and
two rigid-plate force failures. All four pass both unchanged 100 microN
footprint guards. The 10 microN point guard remains declared but is not exercised
by this four-group matrix. There are 3,200,004 base endpoints plus four additional
left-limit values. Both rigid histories also execute at 1.25 microseconds with
every one of their 3,200,002 combined refinement endpoints scored, plus four
left-limit values. Thus 6,400,014 values are scored; left limits reuse existing
event timestamps and are not additional time steps.

Halving the step changes matched force waveforms by 0.594 microN for coil and
10.113 microN for screw. Every intervening refinement endpoint is also scored:
the screw's peak error rises by 76.671 microN to 7.774861 N. Both rigid histories
remain force failures. These are numerical differences, with no separately
predeclared refinement pass guard or controller qualification. The selected
soft histories remain fully in contact; no new soft time/spatial refinement or
native failure-stop probes execute in this batch. Recorded physical stops in
the controller matrix remain separate evidence.

Compact results, exact event pairs and the audit are under
`acf_simulink/validation/current_model_matrix_20261006/native/`. The 500 Hz
display traces can miss high-frequency peaks and the event timestamps; use
all-step JSON metrics and `rigidSideEvents` for quantitative claims. All owned
temporary references, command captures, model/cache, SDI and preferences have
been released. The observed temporary peak was 321,319,394 bytes, below 600 MiB.

Audit without running MATLAB or regenerating arrays:

```bash
python acf_simulink/experiments/current_model_matrix_20261006/native/audit_matrix_native.py
```

To reproduce, use an isolated copy of this completed directory so the wrapper
does not overwrite frozen result paths, then run `launch_matrix_native.py` with
that copy's selection and `--contract-sha256` equal to the frozen matrix
contract. The launcher refuses source/config/lock mismatches, forecasts shared
storage, and retains any failed strict guard rather than changing its threshold.
