# Active contact flange for sanding

MATLAB/Simulink models, experiment recipes, compact results and prototype
design reports for an active sanding flange. The design goal is ±0.5 N
workpiece-normal pressing-force error over 5–50 N and 0.1–2 kg tools.

Start with the current
[prototype design report](reports/active-contact-flange-report-design-explained-20261006.md).
The complete sanding requirement remains unqualified. The latest study tests
total axial force with one-axis sensing, tool acceleration, a fixed robot and
two-second recordings starting at loaded equilibrium.

The selected 77 mm × 5 mm pad geometry uses an **unmeasured 50 N/mm stiffness
assumption and zero pad damping**. Both 54-case grids pass: voice coil worst
error 0.046075 N, internal-spring screw 0.118307 N. The report recommends a
**voice-coil prototype with a custom sensing/control PCB**, prioritizing lower
force error, direct actuation and configurable electronics. The screw retains
lower modeled winding loss (7.4 W versus the coil's 37.4 W). These are simulation
results, not measured Mirka pad properties or hardware qualification.

The proposed PCB integrates load-cell amplification/filtering, an ADXL354C,
ADS131M04 and local MCU control on S. The bipolar power driver stays on A.
This architecture supports customization, one calibration/configuration
interface, programming and controlled sensing timing; assembled robustness
and thermal performance still require testing.

- [Latest actuator/pad study and reproduction](acf_simulink/experiments/sea_pad_dynamic_20261006/README.md)
- [Open the shallow native pad-model replay](acf_simulink/generated/sea_pad_dynamic_20261006/GETTING_STARTED.md)
- [Run the preserved closed-loop reference model](acf_simulink/GETTING_STARTED.md)
- [Fixed-upper-spring model guide](acf_simulink/generated/fixed_upper_20261005/GETTING_STARTED_FIXED.md)
- [Historical study reproduction](acf_simulink/REPRODUCING_RESULTS.md)

The native pad SLX replays recorded controller commands to verify the physical
plant. It is separate from the closed-loop experiment engine and the preserved
reference SLX. MATLAB/Simulink R2026b was used for the native recordings.

## Repository contents and external evidence

The repository includes source, SLX models, reports, figures, compact evidence,
learned checkpoints and reviewed frozen parameter/configuration inputs. Raw
simulation histories, build binaries/caches, private MATLAB preferences,
browser logs and multi-gigabyte delivery archives remain outside Git.
Historical manifests retain their original hashes; missing raw recordings
need regeneration before waveform comparisons can be repeated. Read the
[storage handoff](acf_simulink/validation/storage_cleanup_20261006/README.md).

Compact evidence formerly reached through host-specific SSD links is published
under [github_snapshot_20261006](acf_simulink/validation/github_snapshot_20261006/).
On a fresh Linux checkout, verify those copies and recreate their relative
aliases with:

```bash
python acf_simulink/experiments/restore_compact_aliases.py --restore
```

Existing paths are preserved. The aliases provide compact evidence and inputs;
they do not restore excluded raw histories. Some delivered reports retain
author-machine file links; the relative links above provide portable entry points.
