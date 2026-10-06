#!/usr/bin/env python3
"""Export paired native calibration results and a portable frozen-input bundle.

No simulation occurs here. Full-history metrics come from executed native rows;
plots use min/max envelopes over every native point in each bin.
"""
from pathlib import Path
import csv
import hashlib
import json
import shutil
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parent
ORIGINAL = ROOT / "inertial_uncertainty_v8_joint_long"
REFINED = ROOT / "inertial_uncertainty_v8_calibrated_static_long"
OS = "inertial_uncertainty_budget_repeated_20260930"
RS = "inertial_uncertainty_calibrated_static_long_20260930"
PREFIX = ROOT / "inertial_uncertainty_v8_calibration_refinement"
BUNDLE = ROOT / "inertial_uncertainty_v8_portable_inputs"
PROOF = ROOT / "normal_mass_controller_tmpfs_sdi_pilot_20260930_v8_results.json"


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def binding(path):
    path = Path(path)
    return dict(file=str(path.relative_to(ROOT)), bytes=path.stat().st_size, sha256=sha(path))


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2) + "\n")


def portable_wrapper(version, phase, prior):
    return f"""function report=run_{version}_long(storageMode)
% Relative-path rerun of the exact recorded recipe. Never use the instrumented
% inspection models as recipe inputs: the recipe adds residual instruments.
assert(~usejava('desktop'),'Run with matlab -batch, not the user desktop.');
if nargin<1,storageMode='tmpfs';end
here=fileparts(mfilename('fullpath'));
privateRoot=fullfile(getenv('HOME'),'.local','state','actuator-model');
if ~isfolder(privateRoot),mkdir(privateRoot);end
recipe=fullfile(here,'{version}','experiments');
addpath(recipe,'-begin');clear qualify_inertial_uncertainty;
out=fullfile(here,'rerun_output','{version}');
report=qualify_inertial_uncertainty(fullfile(here,'source'),out,'{phase}', ...
 fullfile(here,'source','normal_contact_trial_candidate.mat'),'accelerator', ...
 fullfile(here,'data','{prior}'),storageMode);
end
"""


def main(build_bundle=True):
    original_file, refined_file = ORIGINAL / f"{OS}_results.json", REFINED / f"{RS}_results.json"
    original, refined = (json.loads(p.read_text()) for p in (original_file, refined_file))
    assert original["status"] == refined["status"] == "complete"
    assert original["executed"] == refined["executed"] == 4
    assert original["sourceModelSha256"] == refined["sourceModelSha256"]
    assert original["candidateSha256"] == refined["candidateSha256"]
    assert original["sourceStillMatches"] and refined["sourceStillMatches"]
    assert original["privateSourceUnmodified"] and refined["privateSourceUnmodified"]
    assert original["sourceHelpersStillMatch"] and refined["sourceHelpersStillMatch"]
    assert all(g["passed"] and g["staticResidualCount"] == 16 and
               g["physicalAndTimingCoefficientsIdentical"] for g in refined["staticCalibrationGuard"])
    rows = []
    traces = []
    for before, after in zip(original["rows"], refined["rows"]):
        assert all(before[k] == after[k] for k in ("id", "seed", "value", "massS_actual_kg",
                    "reference_N", "angle_deg", "mu", "nativeSamples", "endTime_s",
                    "robotDelay_actual_s", "robotResponse_actual_s", "actualAAcutoff_Hz",
                    "actualPipelineDelay_s", "forceNoiseRms_N", "accelNoiseRms_m_s2", "positionNoiseRms_m"))
        rows.append(dict(caseId=before["id"], priorJointCaseId=before["value"],
                         massS_kg=before["massS_actual_kg"], reference_N=before["reference_N"],
                         orientation_deg=before["angle_deg"], seed=before["seed"],
                         originalPeakError_N=before["peakNormalForceError_N"],
                         calibratedStaticPeakError_N=after["peakNormalForceError_N"],
                         reduction_N=before["peakNormalForceError_N"]-after["peakNormalForceError_N"],
                         originalMargin_N=.5-before["peakNormalForceError_N"],
                         calibratedStaticMargin_N=.5-after["peakNormalForceError_N"],
                         originalPeakTime_s=before["peakNormalForceErrorTime_s"],
                         calibratedStaticPeakTime_s=after["peakNormalForceErrorTime_s"],
                         nativeSamples=before["nativeSamples"], passedOriginal=before["passed"],
                         passedCalibratedStatic=after["passed"]))
        traces.extend([dict(cohort=cohort, caseId=before["id"], **binding(row["nativeTrace"]))
                       for cohort, row in (("original", before), ("calibrated_static", after))])
    with PREFIX.with_suffix(".csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, rows[0].keys()); writer.writeheader(); writer.writerows(rows)
    old_worst, new_worst = original["worstNormalForceError_N"], refined["worstNormalForceError_N"]
    summary = dict(status="complete", scope="Paired four30s stress-corner records,24 bumps per record; not a full-range refined uncertainty grid or continuous tolerance box.",
        metric="Maximum absolute true normal force error over all native samples including startup; each record has3000001 points. Contact/travel/current/incomplete and configured spring packing guards apply.",
        sourceModelSha256=original["sourceModelSha256"], candidateSha256=original["candidateSha256"],
        originalResults=binding(original_file), calibratedStaticResults=binding(refined_file),
        preparedConfigurations=binding(refined["preparedConfigurationsFile"]),
        originalInstrumentedModelSha256=original["privateModelSha256"],
        calibratedStaticInstrumentedModelSha256=refined["privateModelSha256"],
        exactExecutedRecipeHashes=dict(original=original["experimentSha256"], calibratedStatic=refined["experimentSha256"]),
        guards=refined["staticCalibrationGuard"], rows=rows, nativeTraces=traces,
        originalWorst_N=old_worst, calibratedStaticWorst_N=new_worst,
        originalMargin_N=.5-old_worst, calibratedStaticMargin_N=.5-new_worst,
        marginRecovered_N=old_worst-new_worst,
        staticTargets=dict(forceBias_N=.01, forceGainFraction=.0005, forceCrossTalkFraction=.0005,
            forceAndAccelFrame_deg=.05, accelerationBias_m_s2=.005, accelerationGainFraction=.0025,
            identifiedSAndBMassGainFraction=.0025, identifiedForceConstantGainFraction=.0025,
            positionGainFraction=.0005, identifiedScanGainFraction=.0005),
        unchanged="Same seed/noise amplitudes, ADC range/quantum, actual and identified AA/pipeline, actual robot mismatch, physical mechanics, identified k/c, controller,100us current response,24V supply and actual physical startup equilibrium. Assembled calibration targets; no improved sensor noise or new part is assumed.",
        prototypeLimit="These calibration residual targets require installed-system verification across load,orientation and temperature. RMS noise and manufacturer factory specifications do not demonstrate this assembled accuracy.")
    write_json(PREFIX.with_suffix(".json"), summary)

    old_plot = loadmat(ORIGINAL / f"{OS}_plot_traces.mat", simplify_cells=True)["plotTraces"]
    new_plot = loadmat(REFINED / f"{RS}_plot_traces.mat", simplify_cells=True)["plotTraces"]
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True, gridspec_kw={"height_ratios": [1, 1.15]})
    x = np.arange(4); width = .34
    axes[0].bar(x-width/2, [r["originalPeakError_N"] for r in rows], width, color="#8b98a5", label="Original static residuals")
    axes[0].bar(x+width/2, [r["calibratedStaticPeakError_N"] for r in rows], width, color="#0369a1", label="Static residuals halved")
    axes[0].axhline(.5, color="#b91c1c", linestyle="--", label="±0.5 N requirement")
    axes[0].set_xticks(x, ["Heavy · seed 7", "Heavy · seed 29", "Heavy · seed 1729", "Light · seed 29"])
    axes[0].set_ylabel("Peak |normal force error| (N)"); axes[0].set_ylim(0, .57)
    axes[0].legend(loc="upper right", fontsize=8, frameon=True,
                   facecolor="white", framealpha=1, edgecolor="none")
    axes[0].set_title("Tighter assembled calibration recovers 75 mN of worst margin\nSame four 30-second stress records; noise, timing, mechanics and drive unchanged", loc="left")
    for i, row in enumerate(rows):
        for shift, key in ((-width/2, "originalPeakError_N"), (width/2, "calibratedStaticPeakError_N")):
            axes[0].text(i+shift, row[key]+.008, f"{row[key]:.3f}", ha="center", fontsize=8)
    envelope_files = []
    for cohort, plots, color in (("original", old_plot, "#8b98a5"), ("calibrated_static", new_plot, "#0369a1")):
        plot = plots[2]; ref = plot["row"]["reference_N"]
        low, high = plot["normalForceMinimum_N"]-ref, plot["normalForceMaximum_N"]-ref
        axes[1].fill_between(plot["time_s"], low, high, color=color, alpha=.7, linewidth=0, label=cohort.replace("_", " "))
        chart_path = PREFIX.with_name(PREFIX.name+f"_{cohort}_worst_envelope.csv")
        np.savetxt(chart_path, np.column_stack([plot["time_s"], low, high]), delimiter=",",
                   header="bin_start_s,minimum_normal_error_N,maximum_normal_error_N", comments="")
        envelope_files.append(binding(chart_path))
    axes[1].axhline(.5, color="#b91c1c", linestyle="--"); axes[1].axhline(-.5, color="#b91c1c", linestyle="--")
    axes[1].set(xlim=(0,30), ylim=(-.54,.54), xlabel="Time (s)", ylabel="Normal force error (N)")
    axes[1].set_title("Worst pair: S = 2 kg / 50 N / 90° / seed 1729 · corner 8\nAll native force samples retained in min/max plot bins", loc="left", fontsize=10)
    axes[1].legend(loc="lower right", frameon=False, fontsize=8)
    for suffix in ("svg", "png"):
        fig.savefig(PREFIX.with_suffix("."+suffix), dpi=160)
    plt.close(fig)
    summary["chartData"] = [binding(PREFIX.with_suffix(".csv")), *envelope_files]
    write_json(PREFIX.with_suffix(".json"), summary)

    if not build_bundle:
        manifest_file = BUNDLE / "manifest.json"
        manifest = json.loads(manifest_file.read_text())
        for entry in manifest["files"]:
            if entry["file"].startswith("chart_data/"):
                source = ROOT / Path(entry["file"]).name
                destination = BUNDLE / entry["file"]
                shutil.copyfile(source, destination)
                entry.update(bytes=destination.stat().st_size, sha256=sha(destination))
        manifest["visualReview"] = "Opaque white top legend prevents the requirement line crossing legend text; scientific data unchanged."
        write_json(manifest_file, manifest)
        print("Paired charts and portable chart bindings refreshed; frozen native inputs unchanged.")
        return

    # The portable bundle is an immutable input/recipe snapshot, not another
    # executed cohort. Large native histories remain separately retained.
    BUNDLE.mkdir(exist_ok=False)
    entries = []
    def copy(source, relative, role, expected=None):
        source, destination = Path(source), BUNDLE / relative
        actual = sha(source)
        if expected:
            assert actual == expected, f"Recorded source changed: {source}"
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        assert sha(destination) == actual
        entries.append(dict(file=relative, role=role, bytes=destination.stat().st_size, sha256=actual))
    source = Path(original["sourceModel"]).parent
    copy(original["sourceModel"], "source/acf_force_control_normal_trial.slx", "Uninstrumented native source used by both exact recipes", original["sourceModelSha256"])
    copy(original["candidateFile"], "source/normal_contact_trial_candidate.mat", "Explicit final selected nominal candidate", original["candidateSha256"])
    for helper in original["helperSources"]:
        copy(source / "helpers" / helper["file"], "source/helpers/"+helper["file"], "Frozen native preparation/run/metric helper", helper["sha256"])
    for version, report in (("original", original), ("calibrated", refined)):
        copy(report["privateModel"], f"models/{version}_long_instrumented.slx", "Executed instrumented native diagram; inspection only", report["privateModelSha256"])
        copy(report["experimentSnapshot"], f"{version}/experiments/qualify_inertial_uncertainty.m", "Exact executed recipe snapshot", report["experimentSha256"])
        copy(PROOF, f"{version}/validation/{PROOF.name}", "Required storage-only bitwise proof for tmpfs execution", report["storageOnlyProof"]["sha256"])
    for folder, stem in ((ORIGINAL, OS), (REFINED, RS)):
        for suffix in ("results.mat", "results.json", "rows.csv", "plot_traces.mat", "storage_cleanup.json"):
            copy(folder / f"{stem}_{suffix}", "data/"+f"{stem}_{suffix}", "Recorded configurations, scalar metrics or complete-force plot envelope")
    for suffix in ("results.mat", "results.json", "rows.csv"):
        file=ROOT / "inertial_uncertainty_v8_joint" / f"inertial_uncertainty_budget_joint_20260930_{suffix}"
        copy(file, "data/"+file.name, "Prior executed60-corner inputs from which the original4 worst cases were selected")
    copy(refined["preparedConfigurationsFile"], "data/"+Path(refined["preparedConfigurationsFile"]).name, "Four prepared tighter-static configurations frozen before first native simulation", refined["preparedConfigurationsSha256"])
    for p in (PREFIX.with_suffix(".json"), PREFIX.with_suffix(".csv"), PREFIX.with_suffix(".svg"), PREFIX.with_suffix(".png"),
              *[ROOT/e["file"] for e in envelope_files]):
        copy(p, "chart_data/"+p.name, "Paired source-backed chart, scalar data or complete-force envelope")
    for version in ("original", "calibrated"):
        phase = "budget_repeated" if version == "original" else "calibrated_static_long"
        prior = "inertial_uncertainty_budget_joint_20260930_results.mat" if version == "original" else f"{OS}_results.mat"
        wrapper = BUNDLE / f"run_{version}_long.m"
        wrapper.write_text(portable_wrapper(version, phase, prior))
        entries.append(dict(file=wrapper.name, role="Relative-path launch wrapper; invokes unchanged exact recipe", bytes=wrapper.stat().st_size, sha256=sha(wrapper)))
    readme = BUNDLE / "README.md"
    readme.write_text("""# Reproduce the paired calibration records

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
""")
    entries.append(dict(file=readme.name, role="Portable relative-path rerun instructions and qualification scope", bytes=readme.stat().st_size, sha256=sha(readme)))
    manifest = dict(status="frozen", scope=summary["scope"], sourceModelSha256=summary["sourceModelSha256"],
        candidateSha256=summary["candidateSha256"], exactExecutedRecipeHashes=summary["exactExecutedRecipeHashes"],
        files=entries, representativeFullNativeTraceReferences=[binding(ORIGINAL/f"{OS}_worst_native_trace.mat"), binding(REFINED/f"{RS}_worst_native_trace.mat")],
        allRetainedFullNativeTraceReferences=traces, sourceBoundComparison=str(PREFIX.relative_to(ROOT))+".json",
        launchSemantics=dict(simulationMode="accelerator", storageMode="tmpfs", maxNumCompThreads=2,
                            process="Isolated matlab -batch; private Model Workspace,cache/code/SDI and explicit selected candidate.",
                            originalPhase="budget_repeated", calibratedPhase="calibrated_static_long",
                            guard="Exactly16 static primitives and their declared identified-controller coefficients; all other prepared d fields identical."))
    write_json(BUNDLE / "manifest.json", manifest)
    print(json.dumps(dict(originalWorst_N=old_worst, calibratedStaticWorst_N=new_worst,
                         marginRecovered_N=old_worst-new_worst, bundle=str(BUNDLE), files=len(entries)), indent=2))


if __name__ == "__main__":
    main("--charts-only" not in sys.argv)
