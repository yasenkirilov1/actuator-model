#!/usr/bin/env python3
"""Audit paired held-out native recordings and produce numerical SVG figures."""
from __future__ import annotations
import hashlib
import json
import argparse
from pathlib import Path
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
STUDY = ROOT / "acf_simulink/validation/ml_force_20261005"


def read_json(path): return json.loads(path.read_text())
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    reports = {phase: read_json(STUDY / phase / "results.json") for phase in ["collect", "select", "test"]}
    assert all(r["status"] == "complete" for r in reports.values())
    assert [len(reports[p]["rows"]) for p in ["collect", "select", "test"]] == [12, 12, 24]
    selection = read_json(STUDY / "selected_gain.json")
    training = read_json(STUDY / "training.json")
    weights = loadmat(STUDY / "trained_weights.mat", simplify_cells=True)
    rows = reports["test"]["rows"]
    pairs = []
    parity = []
    for i in range(0, len(rows), 2):
        base, learned = rows[i:i+2]
        assert base["group"] == learned["group"] and base["id"] == learned["id"]
        assert base["gain"] == 0 and learned["gain"] == selection["selectedGain"]
        for field in ["massS_kg", "reference_N", "angle_deg", "mu", "seed"]:
            assert base[field] == learned[field]
        pairs.append({"group": base["group"], "id": base["id"], "massS_kg": base["massS_kg"],
                      "reference_N": base["reference_N"], "angle_deg": base["angle_deg"], "seed": base["seed"],
                      "baseline": base, "learned": learned,
                      "peakChange_N": learned["peakNormalForceError_N"]-base["peakNormalForceError_N"],
                      "rmsChange_N": learned["rmsNormalForceError_N"]-base["rmsNormalForceError_N"]})
        path = STUDY / "test" / f"case_{i+2:03d}_features.mat"
        f = loadmat(path, simplify_cells=True)["features"]; d = f["data"]
        x = np.clip((d[:, :10]-weights["mean"])/weights["scale"], -6, 6)
        hidden = np.maximum(0, x @ weights["W1"].T + weights["b1"])
        raw = hidden @ np.atleast_1d(weights["W2"]) + weights["b2"]
        pred = np.clip(raw, -.05, .05)*selection["selectedGain"]
        err = float(np.max(np.abs(pred-d[:, 10]))); assert err < 1e-12
        parity.append({"file": str(path), "sha256": sha(path), "frames": len(d), "maximumInferenceDifference_A": err,
                       "anyFeatureClippingFraction": float(np.mean(np.any(np.abs((d[:, :10]-weights["mean"])/weights["scale"])>6, axis=1))),
                       "actionClippingFraction": float(np.mean(np.abs(raw)>.05))})
    baseline = [r for r in rows if r["gain"] == 0]; learned = [r for r in rows if r["gain"] != 0]
    result = {"status": "complete_finite_native_holdout", "date": "2026-10-05", "architecture": "10-8-1 ReLU",
              "trainedParameters": 97, "seed": training["seed"], "selectedResidualGain": selection["selectedGain"],
              "trainingFramesUsed": training["trainingFramesUsed"], "trainingTrajectories": 8,
              "selectionTrajectories": 4, "selectionNativeLearnedRecordings": 12,
              "holdoutPairedTrajectories": 12, "holdoutBaselineRecordings": 12, "holdoutLearnedRecordings": 12,
              "executedNativeRecordings": sum(len(r["rows"]) for r in reports.values()),
              "nativeSamplesScored": sum(row["nativeSamples"] for r in reports.values() for row in r["rows"]),
              "baselineHoldoutPasses": sum(r["passed"] for r in baseline), "learnedHoldoutPasses": sum(r["passed"] for r in learned),
              "baselineHoldoutWorstError_N": max(r["peakNormalForceError_N"] for r in baseline),
              "learnedHoldoutWorstError_N": max(r["peakNormalForceError_N"] for r in learned),
              "pairedPeakImproved": sum(p["peakChange_N"] < 0 for p in pairs),
              "pairedPeakWorsened": sum(p["peakChange_N"] > 0 for p in pairs),
              "baselineHoldoutMeanPeak_N": float(np.mean([r["peakNormalForceError_N"] for r in baseline])),
              "learnedHoldoutMeanPeak_N": float(np.mean([r["peakNormalForceError_N"] for r in learned])),
              "maximumPairedPeakRegression_N": max(p["peakChange_N"] for p in pairs),
              "maximumLearnedCurrent_A": max(r["peakCurrent_A"] for r in learned),
              "maximumLearnedVoltage_V": max(r["peakVoltage_V"] for r in learned),
              "maximumLearnedAbsStroke_m": max(r["peakAbsStroke_m"] for r in learned),
              "maximumResidualCurrent_A": max(r["peakResidualCurrent_A"] for r in learned),
              "projectionBalanceResidual_N": max(r["projectionBalanceResidual_N"] for r in rows),
              "rigidBalanceResidual_N": max(r["rigidBalanceResidual_N"] for r in rows),
              "lowerBalanceResidual_N": max(r["lowerBalanceResidual_N"] for r in rows),
              "allInferenceParityPassed": True, "inferenceParity": parity, "pairs": pairs,
              "sourceModelSha256": reports["test"]["sourceModelSha256"], "weightsSha256": sha(STUDY / "trained_weights.mat"),
              "scope": "Whole native histories including startup. Independent trajectory/seed holdout; parameter endpoints may overlap selection. Four stress trajectories are a finite subset, not the entire residual box. No powered-sander or hardware test.",
              "learningObjective": "Approximate PI/total-mass inertia sensor-lag teacher imitation, not optimal-action or reward optimization. Teacher omits the compliant inverse's changed response."}
    result["outcome"] = ("supports_improvement_in_this_finite_test" if result["learnedHoldoutPasses"] == 12
                         and result["learnedHoldoutWorstError_N"] < result["baselineHoldoutWorstError_N"]
                         and result["learnedHoldoutMeanPeak_N"] <= result["baselineHoldoutMeanPeak_N"]
                         else "does_not_support_replacing_deterministic_controller")
    out = STUDY / "summary.json"; out.write_text(json.dumps(result, indent=2)+"\n")
    figures = STUDY / "figures"; figures.mkdir(exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "svg.fonttype": "none",
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(10, 4.6), layout="constrained")
    x = np.arange(len(pairs)); w = .36
    ax.bar(x-w/2, [p["baseline"]["peakNormalForceError_N"] for p in pairs], w, color="#64748b", label="Deterministic PI + feedforward")
    ax.bar(x+w/2, [p["learned"]["peakNormalForceError_N"] for p in pairs], w, color="#0d9488", label="With trained current residual")
    ax.axhline(.5, color="#b91c1c", linestyle="--", label="Required maximum, 0.5 N")
    labels=[f"N{p['id']}" if p["group"]=="holdout_nominal" else f"S{p['id']}" for p in pairs]
    ax.set_xticks(x, labels); ax.set_ylabel("Worst absolute workpiece-normal force error (N)")
    ax.set_xlabel("Independent paired recordings: N = nominal; S = calibrated long/mixed stress")
    ax.set_title("Native simulated holdout: trained current residual versus deterministic control", pad=34)
    ax.legend(loc="lower left", bbox_to_anchor=(0, 1.01), ncol=3, frameon=False, fontsize=9)
    ax.set_ylim(0, max(.55, result["learnedHoldoutWorstError_N"]+.08))
    fig.savefig(figures / "ml_force_holdout_peaks.svg"); plt.close(fig)
    fig, ax = plt.subplots(figsize=(8, 3.8), layout="constrained")
    h=training["history"]
    ax.plot([r["epoch"] for r in h], [np.sqrt(r["trainMse_A2"])*1000 for r in h], color="#0d9488", label="8 training trajectories")
    ax.plot([r["epoch"] for r in h], [np.sqrt(r["selectionMse_A2"])*1000 for r in h], color="#7c3aed", label="4 selection trajectories")
    ax.set_xlabel("Training epoch"); ax.set_ylabel("Approximate teacher imitation RMS error (mA)")
    ax.set_title("10 inputs, 8 ReLU neurons, 97 learned parameters"); ax.legend(frameon=False)
    fig.savefig(figures / "ml_force_training.svg"); plt.close(fig)
    worstpair=max(range(len(pairs)),key=lambda i:pairs[i]["learned"]["peakNormalForceError_N"])
    excerpts=[]
    for j in [2*worstpair+1,2*worstpair+2]:
        excerpts.append(loadmat(STUDY / "test" / f"case_{j:03d}_excerpt.mat",simplify_cells=True)["excerpt"])
    fig, axes = plt.subplots(2,1,figsize=(9,5.6),layout="constrained",sharex=True)
    for e,color,label in zip(excerpts,["#64748b","#0d9488"],["Deterministic","Learned residual"]):
        t,d=e["time"],e["data"];mask=(t>=1.98)&(t<=2.23)
        axes[0].plot(t[mask]-2,d[mask,14]-e["p"]["force"]["reference"],color=color,label=label)
        axes[1].plot(t[mask]-2,d[mask,2],color=color,label=label)
    axes[0].axhline(.5,color="#b91c1c",linestyle="--");axes[0].axhline(-.5,color="#b91c1c",linestyle="--")
    axes[0].set_ylabel("Normal force error (N)");axes[0].legend(frameon=False)
    axes[0].set_title("Sharp-bump excerpt of the worst learned holdout; scores use the full native recording")
    axes[1].set_ylabel("Actual coil current (A)");axes[1].set_xlabel("Time from bump start (s)")
    fig.savefig(figures / "ml_force_worst_bump.svg");plt.close(fig)
    print(json.dumps({k:result[k] for k in ["outcome","executedNativeRecordings","nativeSamplesScored", "baselineHoldoutPasses","learnedHoldoutPasses","baselineHoldoutWorstError_N","learnedHoldoutWorstError_N","pairedPeakImproved","pairedPeakWorsened"]},indent=2))


if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--study", type=Path, default=STUDY)
    STUDY = parser.parse_args().study.resolve()
    main()
