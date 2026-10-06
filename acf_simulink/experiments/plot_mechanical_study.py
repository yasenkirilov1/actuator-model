#!/usr/bin/env python3
"""Source-backed standalone figures for the native mechanical parameter study.

Run with /tmp/acf-opt-python/bin/python. No simulation or source file mutation.
The hybrid upper-rigid design is a separate optimization, outside the 400 grid.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.patches import Patch, Rectangle
import numpy as np

BLUE = "#245b7b"
INK = "#243342"
MUTED = "#596878"
GRID = "#d9e1e7"
COUNT_CMAP = LinearSegmentedColormap.from_list("study_counts", ["#f4f8fb", "#a9cbdc", "#245b7b"])
ERROR_CMAP = LinearSegmentedColormap.from_list("study_error", ["#fffbf1", "#f0d39b", "#aa6a20", "#633a0c"])
RATES = [3954, 7346, 12826, 19746]
DAMPING = [0, 5, 15, 30, 60]


def read_source(path: Path):
    payload = path.read_bytes()
    return json.loads(payload), {"path": f"acf_simulink/validation/{path.name}",
        "sha256": hashlib.sha256(payload).hexdigest(), "bytes": len(payload)}


def normalise(study: str, data: dict):
    source_designs = data["summary"] if study == "low" else data["designs"]
    source_cases = data["rows"] if study == "low" else data["cases"]
    designs, cases = [], []
    for d in source_designs:
        x = {"study": study, "id": d["id"], "designKey": f"{study}:{d['id']}",
             **{k: d[k] for k in ["k1_N_m", "k2_N_m", "c1_Ns_m", "c2_Ns_m", "lowerPart", "upperPart"]}}
        x.update(trackingPassed=d["physicalFullRangePassed"] if study == "low" else d["nominalTrackingPassedAllCorners"],
                 geometryPassed=d["geometryFullRangePassed"] if study == "low" else d["geometryPassedAllCorners"],
                 fullyFeasible=d["fullRangePassed"] if study == "low" else d["fullyFeasibleNominal"],
                 worstNominalError_N=d["worstError_N"] if study == "low" else d["worstNominalError_N"],
                 feasiblePreloads_N=d["feasiblePreloads_N"],
                 stressTested=False if study == "low" else d["stressTested"],
                 fullyFeasibleStress=None if study == "low" or not d["stressTested"] else d["fullyFeasibleStress"],
                 worstStressError_N=None if study == "low" else d["worstStressError_N"])
        if study == "high":
            for k in ["nominalCommonPreloadMin_N", "nominalCommonPreloadMax_N", "stressCommonPreloadMin_N", "stressCommonPreloadMax_N"]:
                x[k] = d.get(k)
        designs.append(x)
    for c in source_cases:
        ident = c["configId"] if study == "low" else c["id"]
        cases.append({"study": study, "designKey": f"{study}:{ident}", "cornerId": c["cornerId"],
            "caseKey": f"{study}:{ident}:corner{c['cornerId']}",
            "force_N": c["force_N"] if study == "low" else c["referenceForce_N"],
            "orientation_deg": c["orientation_deg"] if study == "low" else c["angle_deg"],
            "executed": c["executed"] if study == "low" else not bool(c["executionError"]),
            "trackingPassed": c["physicalPassed"] if study == "low" else c["trackingPassed"],
            "geometryPreloadMask": c["geometryPassed"] if study == "low" else c["feasiblePreloadMask"],
            "peakForceError_N": c["peakAbsForceError_N"], "peakCurrent_A": c["peakCurrent_A"],
            "peakVoltage_V": c["peakVoltage_V"], "completed": c["completed"],
            "lostContact": c["lostContact"], "strokeStopHit": c["strokeStopHit"]})
    assert len(designs) == 200 and len(cases) == 800
    assert len({c["caseKey"] for c in cases}) == 800
    for d in designs:
        group = [c for c in cases if c["designKey"] == d["designKey"]]
        assert len(group) == 4 and all(c["executed"] for c in group)
        assert {(c["force_N"], c["orientation_deg"]) for c in group} == {(5, 0), (5, 90), (50, 0), (50, 90)}
        assert abs(max(c["peakForceError_N"] for c in group) - d["worstNominalError_N"]) < 1e-8
        assert all(c["trackingPassed"] for c in group) == d["trackingPassed"]
        geometry = np.all([c["geometryPreloadMask"] for c in group], axis=0).any()
        assert bool(geometry) == d["geometryPassed"]
        assert (d["trackingPassed"] and d["geometryPassed"]) == d["fullyFeasible"]
    return designs, cases


def set_style():
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 12,
        "axes.labelsize": 13, "axes.titlesize": 16, "xtick.labelsize": 12,
        "ytick.labelsize": 12, "text.color": INK, "axes.labelcolor": INK,
        "xtick.color": INK, "ytick.color": INK, "axes.edgecolor": INK,
        "figure.facecolor": "white", "axes.facecolor": "white",
        "svg.fonttype": "none", "savefig.facecolor": "white"})


def matrix_axes(title, subtitle, xlabel, ylabel, labels_x, labels_y):
    fig, ax = plt.subplots(figsize=(10, 8.5), dpi=140)
    fig.subplots_adjust(left=.16, right=.83, top=.82, bottom=.22)
    fig.text(.08, .96, title, fontsize=18, fontweight="bold", va="top")
    fig.text(.08, .91, subtitle, fontsize=11.5, color=MUTED, va="top")
    ax.set_xticks(range(len(labels_x)), labels_x)
    ax.set_yticks(range(len(labels_y)), labels_y)
    ax.set_xlabel(xlabel, labelpad=12); ax.set_ylabel(ylabel, labelpad=12)
    ax.tick_params(length=0)
    ax.set_xticks(np.arange(-.5, len(labels_x), 1), minor=True)
    ax.set_yticks(np.arange(-.5, len(labels_y), 1), minor=True)
    ax.grid(which="minor", color="white", linewidth=2)
    ax.tick_params(which="minor", bottom=False, left=False)
    for spine in ax.spines.values(): spine.set_visible(False)
    return fig, ax


def finish(fig, out: Path, name: str, caption: str):
    fig.text(.08, .12, caption, fontsize=11, color=MUTED, va="top", linespacing=1.55)
    fig.savefig(out / f"mechanical_study_{name}.png", dpi=140)
    fig.savefig(out / f"mechanical_study_{name}.svg")
    plt.close(fig)


def figure_feasibility(rows, out):
    values = np.array([[next(x["jointPassed"] for x in rows if x["k1_N_m"] == k1 and x["k2_N_m"] == k2)
                        for k2 in RATES] for k1 in RATES])
    fig, ax = matrix_axes("Force tracking and spring feasibility", "25 damping designs per stiffness pair; all four force / orientation corners must pass.",
        "Upper (R–A) pair stiffness, k2 [N/mm]", "Lower (B–S) pair stiffness, k1 [N/mm]",
        [f"{v / 1000:.3f}" for v in RATES], [f"{v / 1000:.3f}" for v in RATES])
    im = ax.imshow(values, cmap=COUNT_CMAP, vmin=0, vmax=25, aspect="auto")
    for x in rows:
        i, j = RATES.index(x["k1_N_m"]), RATES.index(x["k2_N_m"])
        color = "white" if x["jointPassed"] >= 15 else INK
        ax.text(j, i-.23, f"Joint {x['jointPassed']} / 25", ha="center", va="center", fontsize=13, fontweight="bold", color=color)
        ax.text(j, i+.01, f"Tracking {x['trackingPassed']} / 25", ha="center", va="center", fontsize=11.5, color=color)
        ax.text(j, i+.24, f"Geometry {x['geometryPassed']} / 25", ha="center", va="center", fontsize=11.5, color=color)
        if not x["geometryPassed"]:
            ax.add_patch(Rectangle((j-.49, i+.39), .98, .10, fill=False, hatch="////", edgecolor="#aab5bf", linewidth=0))
    cb = fig.colorbar(im, ax=ax, fraction=.048, pad=.05, ticks=[0, 5, 10, 15, 20, 25])
    cb.set_label("Tracking + geometry passing designs")
    finish(fig, out, "feasibility_matrix", "Corners: 5 and 50 N × 0° and 90°; ±0.5 N throughout. Hatching: no geometric solution.\n"
        "Preloads screened: 10, 20, 28, 30, 32 N; high-rate study also screens 34 N. Geometry includes tolerances.")


def figure_stress(rows, out):
    values = np.array([[next(x["worstStressError_N"] for x in rows if x["c1_Ns_m"] == c1 and x["c2_Ns_m"] == c2)
                        for c2 in DAMPING] for c1 in DAMPING], dtype=float)
    fig, ax = matrix_axes("Two stiff spring pairs: damping under stress", "k1 = k2 = 19.746 N/mm; worst peak error across all four force / orientation corners.",
        "Upper external damping, c2 [N·s/m]", "Lower external damping, c1 [N·s/m]",
        ["0\nremoved", "5", "15", "30", "60"], ["0  removed", "5", "15", "30", "60"])
    im = ax.imshow(values, cmap=ERROR_CMAP, vmin=0, vmax=1, aspect="auto")
    for x in rows:
        i, j = DAMPING.index(x["c1_Ns_m"]), DAMPING.index(x["c2_Ns_m"])
        color = "white" if x["worstStressError_N"] >= .62 else INK
        ax.text(j, i-.10, f"{x['worstStressError_N']:.3f} N", ha="center", va="center", color=color, fontsize=13, fontweight="bold")
        label = "PASS" if x["fullyFeasibleStress"] else "FAIL"
        ax.text(j, i+.19, label, ha="center", va="center", color=color, fontsize=11.5)
        if not x["fullyFeasibleStress"]:
            ax.add_patch(Rectangle((j-.485, i-.485), .97, .97, fill=False, edgecolor=INK, linewidth=1.6))
    cb = fig.colorbar(im, ax=ax, fraction=.048, pad=.05, ticks=[0, .25, .5, .75, 1])
    cb.set_label("Worst stress |force error| [N]")
    cb.ax.axhline(.5, color=INK, linewidth=1.5, linestyle="--")
    finish(fig, out, "ss_damping_stress", "PASS requires peak error ≤0.5 N plus contact, travel, current and spring geometry checks.\n"
        "Stress: 0.02 N RMS force noise, seed 7, plus 0.2 ms unmodeled delay. Coil friction remains present.")


def figure_clean_min(rows, out):
    values = np.array([[next(x["minimumWorstNominalError_N"] for x in rows if x["k1_N_m"] == k1 and x["k2_N_m"] == k2)
                        for k2 in RATES] for k1 in RATES])
    fig, ax = matrix_axes("Similar clean force errors hide spring failures", "Minimum four-corner peak error over 25 damping choices; geometry is checked separately.",
        "Upper (R–A) pair stiffness, k2 [N/mm]", "Lower (B–S) pair stiffness, k1 [N/mm]",
        [f"{v / 1000:.3f}" for v in RATES], [f"{v / 1000:.3f}" for v in RATES])
    im = ax.imshow(values, cmap=ERROR_CMAP, vmin=0, vmax=.5, aspect="auto")
    for x in rows:
        i, j = RATES.index(x["k1_N_m"]), RATES.index(x["k2_N_m"])
        ax.text(j, i-.22, f"{x['minimumWorstNominalError_N']:.4f} N", ha="center", va="center", fontsize=13, fontweight="bold")
        ax.text(j, i+.02, f"c1 / c2: {x['bestC1_Ns_m']:g} / {x['bestC2_Ns_m']:g}", ha="center", va="center", fontsize=11.5)
        ax.text(j, i+.24, "Pack possible" if x["geometryFeasibleDesigns"] else "Pack rejected", ha="center", va="center", fontsize=11.5)
        if not x["geometryFeasibleDesigns"]:
            ax.add_patch(Rectangle((j-.49, i+.39), .98, .10, fill=False, hatch="////", edgecolor="#aab5bf", linewidth=0))
    cb = fig.colorbar(im, ax=ax, fraction=.048, pad=.05, ticks=[0, .1, .2, .3, .4, .5])
    cb.set_label("Minimum clean peak |force error| [N]")
    finish(fig, out, "clean_minimum_matrix", "Every stiffness pair can track below ±0.5 N in the nominal simulation. Fifteen of sixteen\n"
        "pairs have no feasible tested spring envelope. Good force tracking alone does not establish mechanical feasibility.")


def figure_lengths(rows, out):
    fig, ax = plt.subplots(figsize=(10, 6.8), dpi=140)
    fig.subplots_adjust(left=.30, right=.94, top=.79, bottom=.30)
    fig.text(.07, .96, "Compact spring arrangements", fontsize=18, fontweight="bold", va="top")
    fig.text(.07, .90, "Combined lengths include catalogue free-length tolerances; all bars start at zero.", fontsize=11.5, color=MUTED, va="top")
    ys = np.arange(len(rows))
    free = [x["combinedFreeLengthMax_mm"] for x in rows]
    installed = [x["combinedInstalledLengthMax_mm"] for x in rows]
    ax.barh(ys-.17, free, height=.27, color=BLUE, label="Maximum combined free length")
    ax.barh(ys+.17, installed, height=.27, facecolor="white", edgecolor=BLUE, linewidth=1.4, hatch="////", label="Combined installed length")
    for i, x in enumerate(rows):
        ax.text(x["combinedFreeLengthMax_mm"]-1, i-.17, f"{x['combinedFreeLengthMax_mm']:.2f} mm", va="center", ha="right", color="white", fontsize=12)
        ax.text(x["combinedInstalledLengthMax_mm"]+1, i+.17, f"{x['combinedInstalledLengthMax_mm']:.2f} mm", va="center", fontsize=12)
    ax.axvline(100, color=INK, linewidth=1.5, linestyle="--")
    ax.text(100, -.55, "100 mm limit", ha="center", va="bottom", fontsize=11)
    ax.set_yticks(ys, [x["chartLabel"] for x in rows]); ax.invert_yaxis()
    ax.set_xlim(0, 112); ax.set_xticks([0, 25, 50, 75, 100])
    ax.set_xlabel("Summed spring length [mm]")
    ax.grid(axis="x", color=GRID, linewidth=.7); ax.set_axisbelow(True)
    ax.tick_params(axis="y", length=0)
    for name in ["top", "right", "left"]: ax.spines[name].set_visible(False)
    fig.legend(handles=[Patch(facecolor=BLUE, label="Maximum combined free length"),
        Patch(facecolor="white", edgecolor=BLUE, hatch="////", label="Combined installed length")],
        loc="lower left", bbox_to_anchor=(.07, .18), ncol=1, frameon=False, fontsize=11.5)
    finish(fig, out, "spring_lengths", "Installed lengths use the recorded common preload: hybrid 32 N, S/S 34 N, earlier soft design 10 N.\n"
        "The hybrid is a separate optimization. The earlier soft tune fails full-range geometry.\n"
        "These are summed spring lengths, not overall flange dimensions; coil, guides and mounts are excluded.")


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--validation-dir", type=Path)
    args = parser.parse_args(); out = args.validation_dir or Path(__file__).resolve().parents[1] / "validation"
    names = ["full_range_mechanics_low_results.json", "full_range_mechanics_high_results.json",
             "rigid_sensor_qualification_dense_selected.json", "spring_damper_sweep_results.json"]
    sources = [read_source(out / n) for n in names]
    (low, _), (high, _), (hybrid, _), (old, _) = sources
    assert low["catalogue"]["springs"] == high["catalogue"]["springs"]
    ld, lc = normalise("low", low); hd, hc = normalise("high", high)
    designs, cases = ld + hd, lc + hc
    assert len({d["designKey"] for d in designs}) == 400
    feasibility, clean = [], []
    for k1 in RATES:
        for k2 in RATES:
            group = [d for d in designs if d["k1_N_m"] == k1 and d["k2_N_m"] == k2]
            assert len(group) == 25 and {(d["c1_Ns_m"], d["c2_Ns_m"]) for d in group} == {(a,b) for a in DAMPING for b in DAMPING}
            feasibility.append({"k1_N_m": k1, "k2_N_m": k2, "designs": 25,
                "trackingPassed": sum(d["trackingPassed"] for d in group), "geometryPassed": sum(d["geometryPassed"] for d in group),
                "jointPassed": sum(d["fullyFeasible"] for d in group), "sourceDesignKeys": [d["designKey"] for d in group]})
            best = min(group, key=lambda d: d["worstNominalError_N"])
            clean.append({"k1_N_m": k1, "k2_N_m": k2, "minimumWorstNominalError_N": best["worstNominalError_N"],
                "bestDesignKey": best["designKey"], "bestC1_Ns_m": best["c1_Ns_m"], "bestC2_Ns_m": best["c2_Ns_m"],
                "bestDesignGeometryPassed": best["geometryPassed"], "geometryFeasibleDesigns": sum(d["geometryPassed"] for d in group)})
    ss = [d for d in hd if d["k1_N_m"] == 19746 and d["k2_N_m"] == 19746]
    assert len(ss) == 25 and all(d["stressTested"] and d["worstStressError_N"] is not None for d in ss), "Wait for all25 final S/S stress designs."
    stress_rows = []
    for d in ss:
        group = [c for c in high["stressCases"] if c["id"] == d["id"]]
        assert len(group) == 4 and all(c["completed"] for c in group)
        assert {(c["referenceForce_N"], c["angle_deg"]) for c in group} == {(5,0),(5,90),(50,0),(50,90)}
        assert abs(max(c["peakAbsForceError_N"] for c in group) - d["worstStressError_N"]) < 1e-8
        stress_rows.append({k:d[k] for k in ["designKey", "c1_Ns_m", "c2_Ns_m", "worstStressError_N", "fullyFeasibleStress"]})
    hp = hybrid["baseParameters"]; assert hp["mechanics"]["upperRigid"] and not hp["mechanics"]["lowerRigid"]
    hs = hp["springs"]["lower"]; pack = high["selectedParameters"]["springs"]
    length_rows = [
        {"design": "upper-rigid S pair", "chartLabel": "Upper rigid + S pair\nSeparate optimization",
         "commonPreload_N": hp["springs"]["commonPreload_N"],
         "combinedFreeLengthMax_mm": 2000*(hs["freeLength_m"]+hs["freeLengthTolerance_m"]),
         "combinedInstalledLengthMax_mm": max(r["installedSpringLengthMax_mm"] for r in hybrid["rows"]),
         "source": names[2], "springCount": 2},
        {"design": "compliant S/S", "chartLabel": "Two compliant S pairs\nFull-corner study",
         "commonPreload_N": pack["commonPreload_N"],
         "combinedFreeLengthMax_mm": sum(2000*(pack[j]["freeLength_m"]+pack[j]["freeLengthTolerance_m"]) for j in ["lower", "upper"]),
         "combinedInstalledLengthMax_mm": sum(2000*(pack[j]["freeLength_m"]+pack[j]["freeLengthTolerance_m"]-pack["commonPreload_N"]/pack[j]["rate_N_m"]) for j in ["lower", "upper"]),
         "source": names[1], "sourceDesignKey": f"high:{high['selected']['id']}", "springCount": 4},
        {"design": "earlier 20 N soft tune", "chartLabel": "Earlier soft pairs\n20 N-specific tune",
         "commonPreload_N": old["parameters"]["springs"]["commonPreload_N"],
         "combinedFreeLengthMax_mm": old["selected"]["freeLengthMax_mm"],
         "combinedInstalledLengthMax_mm": old["selected"]["installedLengthMax_mm"],
         "source": names[3], "springCount": 4},
    ]
    assert np.allclose([r["combinedFreeLengthMax_mm"] for r in length_rows], [37.18, 74.36, 89.9])
    result = {"schemaVersion": 1, "createdUtc": datetime.now(timezone.utc).isoformat(),
        "sourceManifest": [m for _,m in sources], "nominalDesigns": 400, "nominalNativeSimulations": 1600,
        "nominalTrackingPassedDesigns": sum(d["trackingPassed"] for d in designs),
        "nominalJointPassedDesigns": sum(d["fullyFeasible"] for d in designs),
        "stressSSTestedDesigns": 25, "stressSSNativeSimulations": 100,
        "stressSSFullyFeasibleDesigns": sum(d["fullyFeasibleStress"] for d in ss),
        "geometryPreloadsByStudy_N": {"low":low["preloadValues_N"], "high":high["commonPreloads_N"]},
        "cornerForceOrientation": [[5,0],[5,90],[50,0],[50,90]], "forceTolerance_N": .5,
        "scope": "Compliant four-corner mechanical sweep; these400 designs do not include rigid variants or dense range qualification. A single admissible common preload must work at all four corners.",
        "nominalSensing": "Zero force noise;1 micrometer RMS magnet noise; calibrated0.2 ms magnet delay.",
        "stressSensing": high["stressTest"],
        "recovery": {"lowRecoveredRows":low.get("executionInterruption",{}).get("recoveredRows",0),
            "lowGeometrySizingDetailUnavailableForRecoveredRows":True,
            "actualPreloadPassFlagsAndRejectionReasonsRetained":True,
            "interruptionWasExecutionFailureNotModelFailure":True},
        "geometryScope": "Finite preload grids differ by the additional34 N screen in the high study; high admissible intervals are post-screened from native histories. Catalogue sizing envelopes remain conditional, not prototype validation.",
        "figureDatasets": {"feasibilityMatrix":feasibility,"stressDamping":stress_rows,"cleanMinimumMatrix":clean,"springLengths":length_rows},
        "separateHybridReference": {"source":names[2],"upperRigid":True,"lowerRigid":False,
            "k1_N_m":hp["mechanics"]["k1"],"c1_Ns_m":hp["mechanics"]["c1"],
            "solverStep_s":hp["sim"]["step"],"worstDenseError_N":hybrid.get("worstError_N"),
            "includedIn400Designs":False}, "designs":designs,"nominalCases":cases}
    set_style(); figure_feasibility(feasibility,out); figure_stress(stress_rows,out)
    figure_clean_min(clean,out); figure_lengths(length_rows,out)
    for name, (_, manifest) in zip(names,sources):
        assert hashlib.sha256((out/name).read_bytes()).hexdigest() == manifest["sha256"], "Source changed during figure generation; rerun."
    result["figureFiles"] = [f"mechanical_study_{n}.{ext}" for n in ["feasibility_matrix","ss_damping_stress","clean_minimum_matrix","spring_lengths"] for ext in ["png","svg"]]
    (out/"mechanical_study_combined.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    print(json.dumps({k:result[k] for k in ["nominalDesigns","nominalNativeSimulations","nominalTrackingPassedDesigns","nominalJointPassedDesigns","stressSSFullyFeasibleDesigns","figureFiles"]},indent=2))


if __name__ == "__main__": main()
