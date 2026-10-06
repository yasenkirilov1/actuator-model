#!/usr/bin/env python3
"""Plot retained final actuator histories; never execute a simulation."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot(replay_file: Path, output_prefix: Path) -> None:
    replay = json.loads(replay_file.read_text())
    cases = replay["cases"]
    assert len(cases) == 9
    assert [c["row"]["caseIndex"] for c in cases] == list(range(1, 10))
    columns = {name: i for i, name in enumerate(replay["columns"])}
    for case in cases:
        data = np.asarray(case["data"], dtype=float)
        assert np.max(np.abs(data[:, columns["normalForceError_N"]])) == case["fullNativePeakError_N"]
    colors = {"voice_coil": "#2263a5", "leadscrew": "#c65330"}
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "svg.fonttype": "none"})
    fig, ax = plt.subplots(figsize=(10, 4.5), layout="constrained")
    x = np.arange(4)
    for offset, actuator, selected in [(-.18, "voice_coil", cases[:4]), (.18, "leadscrew", cases[4:8])]:
        values = np.array([c["fullNativePeakError_N"] for c in selected])
        bars = ax.bar(x + offset, values, width=.34, color=colors[actuator],
                      label="Voice coil" if actuator == "voice_coil" else "Leadscrew")
        for bar, case in zip(bars, selected):
            if not case["row"]["passed"]:
                bar.set_hatch("///")
            ax.annotate(f"{case['fullNativePeakError_N']:.3f}",
                        (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                        xytext=(0, 5), textcoords="offset points", ha="center", fontsize=9)
    ax.axhline(replay["forceTolerance_N"], color="#555", ls="--", lw=1.2, label="0.5 N requirement")
    ax.set_xticks(x, ["2 kg · 50 N · 90°\nnoise off", "2 kg · 50 N · 90°\nseed 1729",
                     "2 kg · 5 N · 0°\nseed 1729", "0.1 kg · 50 N · 90°\nseed 1729"])
    ax.set_ylabel("Full-record peak normal-force error (N)")
    ax.set_ylim(0, max(.56, 1.18 * max(c["fullNativePeakError_N"] for c in cases[:8])))
    ax.set_title("Final acquisition: paired native actuator checks", loc="left", fontsize=14)
    ax.legend(loc="upper left", frameon=False, ncol=3)
    ax.grid(axis="y", alpha=.18)
    save(fig, output_prefix.with_name(output_prefix.name + "_peak_errors"), replay)
    plt.close(fig)

    selected = [cases[1], cases[5]]
    first = np.asarray(selected[0]["data"], dtype=float)
    t = first[:, columns["time_s"]]
    active = first[:, columns["surfaceHeight_mm"]] > 1e-8
    assert np.any(active)
    bounds = (float(t[active][0]) - .03, float(t[active][-1]) + .03)
    fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True, layout="constrained")
    series = [("normalForceError_N", "Normal-force error (N)"),
              ("current_A", "Actuator current (A)"),
              ("voltage_V", "Drive voltage (V)"),
              ("actuatorTravel_mm", "Actuator travel (mm)")]
    for case in selected:
        data = np.asarray(case["data"], dtype=float)
        actuator = case["row"]["actuator"]
        for ax, (name, ylabel) in zip(axes, series):
            ax.plot(data[:, columns["time_s"]], data[:, columns[name]],
                    color=colors[actuator], lw=1.2,
                    label="Voice coil" if actuator == "voice_coil" else "Leadscrew")
            ax.set_ylabel(ylabel)
            ax.grid(alpha=.18)
    axes[0].axhspan(-.5, .5, color="#739b73", alpha=.10)
    axes[0].axhline(.5, color="#777", lw=.8, ls="--")
    axes[0].axhline(-.5, color="#777", lw=.8, ls="--")
    axes[0].set_ylim(-max(.55, max(c["fullNativePeakError_N"] for c in selected) * 1.15),
                     max(.55, max(c["fullNativePeakError_N"] for c in selected) * 1.15))
    axes[0].set_title("Sharp bump · 2 kg tool · 50 N · 90° · seed 1729", loc="left", fontsize=14)
    axes[0].legend(frameon=False, ncol=2)
    axes[-1].set_xlim(*bounds)
    axes[-1].set_xlabel("Time (s)")
    save(fig, output_prefix.with_name(output_prefix.name + "_heavy_bump"), replay)
    plt.close(fig)


def save(fig: plt.Figure, prefix: Path, replay: dict) -> None:
    description = ("Source-backed plot from retained native histories. "
                   f"Report SHA256 {replay['reportSHA256']}; "
                   f"native source SHA256 {replay['sourceModelSHA256']}; "
                   "qualification uses full histories including startup; plotted samples retain every force peak.")
    fig.savefig(prefix.with_suffix(".svg"), metadata={"Description": description})
    fig.savefig(prefix.with_suffix(".png"), dpi=180, metadata={"Description": description})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    parser.add_argument("output_prefix", type=Path)
    args = parser.parse_args()
    plot(args.replay, args.output_prefix)
