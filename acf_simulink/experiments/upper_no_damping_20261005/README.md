# Fixed upper spring and rigid lower attachment

This study follows the user's current mechanical prescription: a **20,000 N/m
upper R–A spring, zero external damping, unlimited upper spring length/travel**,
and **no external B–S spring or damper**. B–S remains attached through the
representative **64 MN/m finite-stiffness load cell**. This is why the native
`lowerRigid` flag is false in these records: its true setting would remove the
cell's structural degree of freedom as well. It does not indicate a free link.
The 25 mm actuator stroke, contact-loss/travel-stop failures and physical guide
friction remain active.

The user requests an approximately **2,000-word main report**, aimed at choosing
and designing the first prototype and presenting/defending that design. The
main report is [active-contact-flange-report-upper-spring-20261005.md](../../../reports/active-contact-flange-report-upper-spring-20261005.md).
Detailed evidence is linked here rather than embedded in its narrative.

## Current result and runnable model

The selected deterministic candidate uses normal-force PI **0.0245902 A/N**
and **16.3934 A/(N·s)**, measured-inertia feedforward, a four-state body-motion
observer and phase-shaped cell-mode feedback bounded to **±0.05 A**. The
[executing controller contract](../../validation/upper_no_damping_20261005/report/fixed_controller_contract.json)
binds the live gain fields and separately identified modal response. It is a
development baseline, not an accepted ±0.5 N design.

Start with the [fixed-model beginner guide](qualification_v8/GETTING_STARTED_FIXED.md).
The versioned model is under `acf_simulink/generated/fixed_upper_20261005/`,
with adjacent namespaced helpers. The original canonical SLX, earlier reports
and the user's open desktop model remain preserved. There is no external RL
input to enable before running this standalone deterministic model.

The [locked test aggregate](../../validation/upper_no_damping_20261005/report/fixed_qualification_results.json)
contains **736 completed new-seed tests**: **683/700 nominal passes**, worst
**0.580467 N**; **21/32 calibration/motion stress passes**, worst **3.948372 N**;
and **1/4 long repeated-bump/drift passes**, worst **0.691107 N**. One long case
stops at **4.192677 s**. Two long recordings leave the ±7.5 mm soft band.
All **496,558,539 native points** are scored, including startup; incomplete
histories caused by physical stops remain failures. No tuning used these
final outcomes. A new candidate requires new selection and fresh tests.

All 700 nominal cases use **A=1.5 kg, B=0.6 kg**, ten known tool masses spanning
0.1–2 kg, ten force targets spanning 5–50 N, and seven fixed angles spanning
0–90°. The fresh and long cases preserve specified residual/schedule stresses,
while replacing the earlier mechanics with the current fixed arrangement.
Actual quantities and identified parameters remain separate in the sealed bank.

The final 12-family PI/modal comparison has **132 accepted development
recordings**; family8 passes all eleven development points at **0.495250 N**
worst. The full accepted development ledger contains **457 recordings**, not
457 unique physical conditions. Rejected preparations are outside that count.

## Evidence and interpretation

- [Accepted development ledger](../../validation/upper_no_damping_20261005/report/fixed_development_results.json)
  binds only completed, accepted implementations to native results and input
  locks. Prepared bank rows are not executed experiments.
- [Fixed mechanical/sensing requirements](../../validation/upper_no_damping_20261005/report/fixed_mechanics_contract.json)
  records user constraints rather than measured performance.
- [Native waveform and spectral analysis](../../validation/upper_no_damping_20261005/report/fixed_control_waveform.json)
  binds the figure to a complete native history and its result hash. Spectral
  variance uses a Hann-window periodogram; time-domain peaks use every native
  point, including startup.
- [Historical acquisition-rate comparison](../../validation/upper_no_damping_20261005/report/fixed_acquisition_reference.json)
  explains the 30 ksample/s choice on its earlier compliant-link plant. It does
  not qualify the newly fixed mechanics.
- [Independent voltage pairs](../../validation/upper_no_damping_20261005/report/fixed_voltage_diagnostics.json)
  contain two 32/48 V comparisons with unchanged current-response tuning.
  The unclipped development example is bitwise identical at both voltages.
  A separate post-outcome stress diagnostic improves **2.748183 → 0.650463 N**,
  removes 2,386 clipped samples, and reaches **33.021374 V** peak demand.
  Both stress runs fail force accuracy. These four diagnostic runs are outside
  the 736-case qualification count and did not select or retune the controller.

The three main SVGs show the load path, a matched development comparison
before/after mode feedback, and historical acquisition-rate experiments. The
mode-feedback figure is **1.073615 → 0.458519 N**, with **14.6 dB** lower
post-bump cell-band power. It is not the worst fresh qualification case.

## Acquisition and numerical checks

The active budget chain has two-pole **1.9 kHz** matched response envelopes,
**30 ksample/s** simultaneous ADC output, **50 µs EMA at the ADC output clock**,
**20 µs pipeline** and fresh **10 kHz control**. The physical/noise clock is
300 kHz. The inherited `p.acquisition.antiAliasCutoff_Hz=5100` is a normalization
reference; prepared `p.adcStudy.d.channels.*.cutoff_Hz` gives the actual response.

Noise inputs **0.02 N**, **0.0186557 m/s²**, and **1 µm** are reference-conditioned
RMS targets at the frozen **3,177.13 Hz** normalization bandwidth. The preparer
converts them to input noise densities; actual filtered RMS depends on the
active response. ADC noise/quantization are additional. Their RMS does not
establish calibrated bias, thermal stability or sanding-vibration immunity.

Independent MATLAB replay of developmentworst170 passes the original strict
all-27-channel guard: maximum difference **6.25×10⁻¹⁰**. The two qualification
worst replays retain **`passed:false`** under that same strict guard. Their
native times/counts match exactly, their force-waveform differences are
**16.66 µN** for nominal673 and **30.50 µN** for stress728, and they reproduce
the original engineering failures. Neither the tolerance nor the failed JSONs
was rewritten. The linked numerical assessment distinguishes source auditing
from agreement of force metrics.

Read the [independent native/delivery checks](../../validation/upper_no_damping_20261005/report/fixed_native_verification.json)
and the [post-review numerical assessment](../../validation/upper_no_damping_20261005/qualification_v8/matlab_parity/post_review_numerical_assessment_v1.json).
The bounded source audit found exact equality for all576 prepared parameter
expressions and unchanged78 sparse-index arrays in both failed replay cases.
No concrete source/mapping mismatch was found. Floating-point/quantizer
sensitivity remains a hypothesis, not a proved cause.

One deterministic closed-loop solver refinement halves the physical step to
600 kHz while preserving the 300 kHz noise, 30 kHz ADC and 10 kHz controller
clocks. Its developmentworst170 peak changes by **16.60 µN**. This numerical
check does not establish physical performance or qualify the full grid.

Earlier upper-compliant observer attempts used inaccessible local `From`
sources. Their histories are preserved as rejected controller implementations;
they do not establish mechanical feasibility limits. The implementation audit
is [live_source_audit.json](../../validation/upper_no_damping_20261005/report/live_source_audit.json).
Corrected acquired-force and issued-current observers have explicit hierarchical
connections and independent source/sign guards.

The scientific/public and earlier RL reference models remain preserved. Their
force passes describe their recorded mechanics, not this new arrangement.

## Rerun a retained development case

The physical plant executes the emitted C from native Simulink blocks; Python
only schedules independent contexts, records native outputs and scores them.
Each frozen native directory supplies its model, helpers, parameter bank,
bridge/source seals, scenarios and metadata. The evaluator verifies those
inputs before execution and afterward, and snapshots its own source.

A locked nominal worst-case rerun:

```bash
cd /home/tommie/Projects/actuator-model
python acf_simulink/experiments/upper_no_damping_20261005/evaluate_mechanics.py \
  --native-build acf_simulink/validation/upper_no_damping_20261005/qualification_v8/native \
  --output /absolute/new/owned/output/directory \
  --indices 673 --workers 1
```

Scenario673 is the selected controller's known nominal failure at S=2 kg and
35 N. Use a new output directory: the executor never overwrites previous
evidence. `status: complete` means execution finished; `passed` separately
states force/physical success. Every approximately 3.333 µs native point enters
the ±0.5 N metric. Observer display samples at 100 Hz are diagnostic only.
The default baseline mode is explicitly enabled by this executor; the private
RL-instrumented frozen model's external-mode input must not be left at zero
when replaying it directly in MATLAB. The new standalone model removes that
external selection branch and needs no such input.

## Storage and preserved references

The user's 2026-10-06 storage instruction supersedes full-history retention.
The [cleanup and retention record](../../validation/storage_cleanup_20261006/README.md)
lists the intentionally removed bulk recordings and the selected traces kept.
Compact metrics, locked inputs, native models, recipes and original delivery
archives remain. The current local `qualification_v8` fixtures and independent
replays remain, along with selected nominal/stress failures and the long travel
stop. Both mode-comparison traces used by the report remain.

The [earlier relocation manifest](../../validation/upper_no_damping_20261005/report/history_archive_manifest.json)
records historical paths and hashes; it no longer establishes that every raw
file exists. A missing recording must be regenerated from its frozen inputs
into a bounded temporary directory. Mounting the SATA drive alone will not
restore intentionally removed traces.

The [older detailed report](../../../reports/active-contact-flange-report-rl-3h-20261005.md)
and completed RL study remain separate references. The report author's earlier
identity, all reviewed old queries and protected runtime are preserved during
the concise revision.
