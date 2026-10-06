"""Plot the retained paired native recordings; no model execution."""
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    folder = Path(__file__).resolve().parent
    summary = json.loads(
        (folder / "inertial_actuators_lower_series_v1_reviewed_comparison.json").read_text()
    )
    replay_path = Path(summary["replayFile"])
    assert hashlib.sha256(replay_path.read_bytes()).hexdigest() == summary["replaySHA256"]
    replay = json.loads(replay_path.read_text())
    assert replay["reportSHA256"] == summary["sourceReportSHA256"]
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "svg.fonttype": "none",
        }
    )
    colors = ["#16669a", "#ad5722"]
    fig, ax = plt.subplots(figsize=(8.4, 4.4), layout="constrained")
    x = np.arange(4)
    width = 0.32
    for index, key in enumerate(["coilPeakError_N", "screwPeakError_N"]):
        values = [pair[key] for pair in summary["pairs"]]
        bars = ax.bar(
            x + (index - 0.5) * width,
            values,
            width,
            color=colors[index],
            label=["Voice coil", "Leadscrew"][index],
        )
        ax.bar_label(bars, labels=[f"{v:.3f}" for v in values], padding=3, fontsize=9)
    ax.axhline(0.5, color="#aa2635", linestyle="--", linewidth=1.5, label="0.5 N requirement")
    ax.set_ylim(0, 0.58)
    ax.set_ylabel("Full-recording peak normal-force error (N)")
    ax.set_xticks(
        x,
        [
            "2 kg / 50 N / 90°\nnoise off",
            "2 kg / 50 N / 90°\nseed 1729",
            "2 kg / 5 N / 0°\nseed 1729",
            "0.1 kg / 50 N / 90°\nseed 1729",
        ],
    )
    ax.legend(loc="upper left", ncol=3, frameon=False)
    ax.set_title(
        "Both actuators pass the selected compliant-carrier recordings",
        loc="left",
        fontweight="bold",
    )
    fig.supxlabel(
        "Ten-second native runs; 1 mm × 10 mm bump at 100 mm/s; 100 µs matched sensors.\n"
        "Four paired nominal conditions, not full-range or hardware qualification.",
        fontsize=9,
    )
    save(fig, folder, "inertial_actuators_lower_series_v1_peak_errors")

    fig, axes = plt.subplots(3, 1, figsize=(8.4, 7.3), sharex=True, layout="constrained")
    for index, case_index in enumerate([2, 6]):
        case = replay["cases"][case_index - 1]
        values = np.asarray(case["data"])
        assert np.max(np.abs(values[:, 3])) == case["fullNativePeakError_N"]
        window = values[(values[:, 0] >= 1.98) & (values[:, 0] <= 2.13)]
        label = ["Voice coil", "Leadscrew"][index]
        for ax, column in zip(axes, [3, 4, 6]):
            ax.plot(window[:, 0], window[:, column], color=colors[index], linewidth=1.2, label=label)
        axes[0].scatter(case["peakErrorTime_s"], case["peakSignedError_N"], color=colors[index], s=24, zorder=4)
    for bound in [-0.5, 0.5]:
        axes[0].axhline(bound, color="#aa2635", linestyle="--", linewidth=1)
    axes[0].set_ylim(-0.55, 0.55)
    axes[0].set_ylabel("Normal-force error (N)")
    axes[0].legend(loc="upper right", frameon=False)
    axes[0].set_title("Heavy noisy sharp bump: causal carrier inverse in both branches", loc="left", fontweight="bold")
    axes[1].set_ylabel("Current (A)")
    # Replay column 6 is native plant travel (r.data[:,1] in Python indexing),
    # used for actuator stops. The measured S-A channel is a different signal.
    axes[2].set_ylabel("Actuator travel (mm)")
    axes[2].set_xlabel("Time (s)")
    for ax in axes:
        ax.grid(axis="y", color="#dddddd", linewidth=0.6)
        ax.set_xlim(1.98, 2.13)
    fig.supxlabel(
        "S = 2 kg, B = 0.6 kg, 50 N, 90°, seed 1729. Plot keeps exact full-native force peaks.\n"
        "No clipping; RE25 screw continuous rating remains insufficient despite its transient pass.",
        fontsize=9,
    )
    save(fig, folder, "inertial_actuators_lower_series_v1_heavy_bump")


def save(fig, folder, stem):
    for extension in ["svg", "png"]:
        fig.savefig(folder / f"{stem}.{extension}", dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
