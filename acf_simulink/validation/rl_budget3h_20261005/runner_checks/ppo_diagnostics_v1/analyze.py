"""Read retained PPO scoreboards; write only this diagnostic directory.

This is a selection-data analysis, not an additional simulation or test run.
Run after the three lanes finish to bind immutable source hashes.
"""
from __future__ import annotations

import hashlib
import json
import statistics
from datetime import datetime, timezone
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
BANK = HERE.parent.parent
LANES = [
    ("ppo_band_8401", "8401 · 0.45 N / 20 ms", "#205E86", "o"),
    ("ppo_band_8402", "8402 · 0.45 N / 10 ms", "#AD6B13", "s"),
    ("ppo_tail_8403", "8403 · 0.30 N / 20 ms", "#945572", "^"),
]


def read_bound(path: Path) -> tuple[dict, dict]:
    raw = path.read_bytes()
    return json.loads(raw), {
        "path": str(path.relative_to(BANK)),
        "sha256": hashlib.sha256(raw).hexdigest(),
    }


def analyze_lane(name: str) -> dict:
    run = BANK / name
    result, source = read_bound(run / "results.json")
    updates = result["updates"]
    selections = []
    sources = [source]
    for path in sorted((run / "selection").glob("selection_update_*.json")):
        selection, source = read_bound(path)
        sources.append(source)
        point = {
            key: selection[key]
            for key in (
                "phase", "recordings", "passed", "physicalFailures",
                "worstNormalForceError_N", "meanRmsNormalForceError_N",
                "jointScore", "jointScoreTerms", "finalTravelBandViolations",
            )
        }
        point["update"] = int(selection["phase"].split("_")[-1])
        point["worstCase"] = max(
            selection["rows"], key=lambda row: row["peakNormalForceError_N"]
        )["identity"]
        point["robotNonzeroIssued100HzFrames"] = sum(
            row["robotNonzeroIssued100HzFrames"] for row in selection["rows"]
        )
        point["robot100HzFrames"] = sum(
            row["robot100HzFrames"] for row in selection["rows"]
        )
        point["peakIssuedRobotCorrection_m_s"] = max(
            row["peakIssuedRobotCorrection_m_s"] for row in selection["rows"]
        )
        point["rows"] = [
            {k: row[k] for k in (
                "identity", "peakNormalForceError_N", "rmsNormalForceError_N",
                "finalAbsTravelError_m", "integratedTravelBandPeakExcess_m2_s",
            )}
            for row in selection["rows"]
        ]
        selections.append(point)
    initial_rows = {r["identity"]: r for r in selections[0]["rows"]}
    paired = []
    for row in selections[-1]["rows"]:
        initial = initial_rows[row["identity"]]
        paired.append({
            "identity": row["identity"],
            "peakChange_N": row["peakNormalForceError_N"] - initial["peakNormalForceError_N"],
            "rmsChange_N": row["rmsNormalForceError_N"] - initial["rmsNormalForceError_N"],
            "finalAbsTravelChange_m": row["finalAbsTravelError_m"] - initial["finalAbsTravelError_m"],
        })
    threshold = float(result["args"]["force_error_scale_N"])
    recent = updates[-40:]
    return {
        "name": name,
        "status": result["status"],
        "stopReason": result.get("stopReason"),
        "updates": len(updates),
        "decisions": sum(x["transitions"] for x in updates),
        "completedEpisodes": sum(x["episodesCompleted"] for x in updates),
        "completedEpisodePasses": sum(x["episodePasses"] for x in updates),
        "physicalFailures": sum(x["physicalFailures"] for x in updates),
        "selectedUpdate": result.get("selectedUpdate"),
        "trainingRolloutPeak_N": max(x["rolloutPeakNormalForceError_N"] for x in updates),
        "forcePenaltyThreshold_N": threshold,
        "rolloutsWithThresholdExcess": sum(x["rolloutPeakNormalForceError_N"] > threshold for x in updates),
        "recent40UpdateMedians": {
            k: statistics.median(x[k] for x in recent)
            for k in (
                "approx_kl", "grad_norm", "critic_grad_norm", "force_advantage_std",
                "advantage_std", "current_policy_loss", "robot_policy_loss",
                "optimizationElapsed_s",
            )
        },
        "selection": selections,
        "pairedLastMinusInitial": paired,
        "sources": sources,
    }


def main() -> None:
    lanes = [analyze_lane(name) for name, *_ in LANES]
    if any(lane["status"] != "training_complete" for lane in lanes):
        raise RuntimeError("Wait for all three retained runs to finish before freezing this analysis")
    result = {
        "createdUTC": datetime.now(timezone.utc).isoformat(),
        "scope": "Read-only analysis of completed PPO training and reused selection data; no held-out records accessed.",
        "comparison": "Each curve is normalized to its own warm start. Different seeds, initial checkpoints, GAE horizons and banks prevent a controlled causal comparison between lanes.",
        "forcePenaltyCaveat": "The .30 N lane changes both the threshold for the additional peak penalty and force-error reward scaling; it also starts from another ES checkpoint and uses the sustained bank.",
        "noUncertaintyClaim": "These are deterministic repeated evaluations of finite selection banks, not confidence intervals or hardware qualification.",
        "lanes": lanes,
    }
    (HERE / "results.json").write_text(json.dumps(result, indent=2) + "\n")
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 10.5,
        "axes.spines.top": False, "axes.spines.right": False,
        "axes.edgecolor": "#9AA4AD", "axes.labelcolor": "#273746",
        "text.color": "#273746", "xtick.color": "#52616C", "ytick.color": "#52616C",
        "svg.fonttype": "none",
    })
    fig, axes = plt.subplots(1, 3, figsize=(12.8, 5.0))
    metrics = [
        ("worstNormalForceError_N", "Worst force error"),
        ("meanRmsNormalForceError_N", "Mean RMS force error"),
        ("jointScore", "Joint force / travel score"),
    ]
    for ax, (metric, title) in zip(axes, metrics):
        ax.axhline(0, color="#354753", lw=1, ls="--")
        ax.set_title(title, loc="left", fontsize=12, fontweight="bold", pad=12)
        ax.grid(axis="y", color="#E6EBEF", lw=.8)
        ax.set_axisbelow(True)
        for lane, (_, label, color, marker) in zip(lanes, LANES):
            points = lane["selection"]
            initial = points[0][metric]
            x = [p["update"] for p in points]
            y = [(p[metric] / initial - 1) * 100 for p in points]
            ax.plot(x, y, label=label, color=color, marker=marker, ms=4.5, lw=1.8)
        ax.set_xlabel("Completed PPO updates")
        ax.set_xlim(-6, 309)
        ax.set_xticks([0, 100, 200, 300])
        ax.set_ylabel("Change from own warm start (%)")
    fig.suptitle("PPO checkpoint selection outcomes", x=.06, y=.985, ha="left", fontsize=17, fontweight="bold")
    fig.text(.06, .918, "Lower is better. The dashed line is each lane’s starting policy.", fontsize=11.5)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", bbox_to_anchor=(.049, .86), ncol=3, frameon=False, handlelength=2.4)
    fig.text(.06, .085, "8401 and 8402: 24 reused selection cases. 8403: 28 cases including sustained robot motion.", fontsize=9.5)
    fig.text(.06, .041, "Threshold / force GAE time shown in the legend. Lanes also differ in warm start and learning rate; this is not a causal ablation.", fontsize=9.2)
    fig.subplots_adjust(left=.065, right=.98, top=.73, bottom=.22, wspace=.34)
    fig.savefig(HERE / "ppo_selection_change.svg")
    fig.savefig(HERE / "ppo_selection_change.png", dpi=155)
    plt.close(fig)
    print(json.dumps([{k: lane[k] for k in ("name", "updates", "decisions", "completedEpisodes", "physicalFailures", "selectedUpdate")} for lane in lanes], indent=2))


if __name__ == "__main__":
    main()
