# Experiment storage cleanup — 2026-10-06

The user requested that the experiment disk explosion stop. Bulk output
retention was reduced to a small representative set. Older results and hashes
remain historical records; this cleanup changes availability, not their recorded
pass/fail outcomes. No new controller tuning or force qualification occurred.

`bulk_plan.json` records the exact scope, removal candidates and retained arrays.
`bulk_cleanup.json` records the files actually removed and filesystem free-space
measurements. `retention.json` verifies that the retained arrays and compact
evidence were unchanged. `removed_history_links.json` records only symlinks
whose explicitly removed target became unavailable.

The raw cleanup initially removed 3,778 generated MAT/NPZ files, totaling
**209.577 GiB logical bytes**, from project validation and the project's owned
SATA evidence folder. One selected comparison trace had an archive alias whose
resolved target was omitted from the first keep list. A bounded replay of its
frozen source restored the **exact original NPZ bytes and all array hashes**;
`trace_recovery.json` records that check. The keep rule now protects alias targets.
The retained MAT/NPZ arrays total approximately **4.05 GiB**, including
configuration inputs. One of the 557 removed output links was restored for that
trace. Actual recovered filesystem space is reported separately from logical
bytes in the audit JSONs.

Retained material includes:

- The runnable native model bundle, canonical models, all recipes and helpers.
- Parameter/configuration banks, frozen controller inputs and learned checkpoints.
- All compact JSON/CSV evidence, report figures, Markdown and offline HTML.
- All local current `qualification_v8` replay, voltage and acceptance fixtures.
- Selected development, nominal/stress worst-case and long travel-stop traces;
  both native traces behind the report's cell-mode comparison.
- ADC reference cases 170/171, selected 12/14-bit and slower-rate residuals,
  and short 7.5 ksample/s startup failures. All 42 case summaries remain.
- The original delivery ZIPs and unrelated user files.

The old manifests identify the original recorded bytes. A removed trace needs
regeneration from its frozen model, parameters, seeds and recipe before an
independent waveform check can be repeated. The stored case scores and failed
guards were preserved; they were not replaced with reconstructed claims.

`git_temporary_cleanup.json` records abandoned `tmp_pack_*` and `tmp_obj_*`
removal. Valid Git packs and HEAD were preserved. Raw validation arrays,
simulation caches and rebuildable report exports are now excluded by the root
`.gitignore`; source, models, JSON/CSV evidence and figures remain visible.

The **shared 5 GiB experiment-array budget** is specified in root `AGENTS.md`.
Future experiments must forecast storage before launch, stream full-sample
scores, retain compact results by default and release temporary data per batch.
Original delivery archives are preserved outside this raw-array budget.
