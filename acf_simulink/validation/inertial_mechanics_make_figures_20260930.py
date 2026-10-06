#!/usr/bin/env python3
"""Create standalone reviewed mechanics figures from retained native records.

All qualification metrics use complete recordings. Plot envelopes preserve
extrema while reducing SVG size; they do not rescore decimated histories.
"""
from hashlib import sha256
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LogNorm
from matplotlib.patches import Rectangle
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parent
plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                     "axes.spines.right": False, "svg.fonttype": "none"})


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def save(fig, name):
    fig.savefig(ROOT / f"{name}.svg", bbox_inches="tight")
    fig.savefig(ROOT / f"{name}.png", dpi=170, bbox_inches="tight")
    plt.close(fig)


def envelope(time, values, width=25):
    count = len(time) // width
    t = time[:count * width].reshape(count, width)
    y = values[:count * width].reshape(count, width)
    return t.mean(axis=1), y.min(axis=1), y.max(axis=1)


def spring_tuning():
    path = ROOT / "inertial_mechanics_spring_chart_data_20260930.json"
    data = json.loads(path.read_text())
    families = ["PI", "Filtered PID", "PI or PID plus proper inverse"]
    columns = [(rate, damping) for rate in [12.826, 19.746] for damping in [0, 15, 60]]
    matrix = np.full((3, 6), np.nan)
    selected = {}
    for row in data["bestCompletedPerFamilyAndMechanics"]:
        i = families.index(row["controllerFamily"])
        j = columns.index((row["rate_N_mm"], row["damping_Ns_m"]))
        tune = row["bestCompletedTune"]
        selected[(i, j)] = tune
        if tune is not None:
            matrix[i, j] = tune["worstPeakError_N"]
    fig, ax = plt.subplots(figsize=(10.3, 4.1))
    fig.subplots_adjust(bottom=.23)
    cmap = plt.get_cmap("YlOrRd").copy()
    cmap.set_bad("#e7e7e7")
    im = ax.imshow(matrix, cmap=cmap, norm=LogNorm(vmin=.25, vmax=15), aspect="auto")
    ax.set_xticks(range(6), [f"{k:g} N/mm\n{c:g} N·s/m" for k, c in columns])
    ax.set_yticks(range(3), ["Tuned PI", "Tuned filtered PID", "Proper inverse + PI/PID"])
    for (i, j), tune in selected.items():
        if tune is None:
            label = "Contact loss\nin all tunings"
        else:
            value = tune["worstPeakError_N"]
            label = f"{value:.3f} N"
            if not tune["packingFeasible"]:
                ax.add_patch(Rectangle((j-.5, i-.5), 1, 1, fill=False,
                                       hatch="///", edgecolor="#646464", linewidth=0))
                label += "\nPacking reject"
            if tune["bothPhysicalPassed"]:
                label += "\nForce pass"
        ax.text(j, i, label, ha="center", va="center", fontsize=8.5,
                color="white" if tune and tune["worstPeakError_N"] > 3 else "#101010")
    ax.set_title("Best complete clean + noisy tuning pair; ±0.5 N requirement")
    fig.colorbar(im, ax=ax, label="Worst full-recording force error (N)", fraction=.035)
    fig.text(.02, .02, "Sharp bump, S=2 kg, F=50 N, 90°, μ=0.3. Bounded tuning search; hatched cells fail spring working range.", fontsize=8.5)
    save(fig, "inertial_mechanics_spring_tuning_20260930")
    return {"file": path.name, "sha256": digest(path)}


def selected_repeat():
    path = ROOT / "inertial_mechanics_lower_spring_repeated_20260930_lower_series_v1_10s_results.json"
    report = json.loads(path.read_text())
    row = max(report["rows"], key=lambda row: row["peakNormalForceError_N"])
    trace = Path(row["nativeTraceFile"])
    r = loadmat(trace, simplify_cells=True, variable_names=["r"])["r"]
    t, d, p = r["time"], r["data"], r["parameters"]
    normal = r["normalContact"]["data"]
    inverse = r["lowerInverse"]["data"]
    error = normal[:, 0] - p["force"]["reference"]
    peak = int(np.argmax(np.abs(error)))
    assert len(t) == 1_000_001 and np.max(np.abs(error)) == row["peakNormalForceError_N"]
    assert row["jointPassed"] and row["configuredCommonPreloadPassed"]
    delta = d[:, 6] - d[:, 5]
    relative_velocity = d[:, 9] - d[:, 8]
    damper_force = p["mechanics"]["c1"] * relative_velocity
    damper_power = p["mechanics"]["c1"] * relative_velocity**2
    spring = p["springs"]["lower"]
    ks = spring["rate_N_m"]
    shift = p["d"]["q10"] / p["mechanics"]["k1"] + delta
    preload = p["springs"]["commonPreload_N"]
    spring_forces = np.column_stack((preload + ks * shift, preload - ks * shift))
    compression = spring_forces / ks
    head_gap = p["mechanics"]["midTravel"] + d[:, 4] - d[:, 6]
    metadata = {
        "sourceResult": path.name, "sourceResultSHA256": digest(path),
        "nativeTrace": str(trace), "nativeTraceSHA256": digest(trace),
        "sourceModelSHA256": report["sourceModelSHA256"], "sampleCount": len(t),
        "metric": "All original native true normal-force samples, including startup",
        "peakNormalError_N": float(abs(error[peak])), "peakTime_s": float(t[peak]),
        "forceTarget_N": p["force"]["reference"], "seed": row["seed"],
        "peakLowerInverseCurrent_A": float(np.max(np.abs(inverse[:, 3]))),
        "peakLowerRelativeSpeed_m_s": float(np.max(np.abs(relative_velocity))),
        "peakDamperForce_N": float(np.max(np.abs(damper_force))),
        "peakDamperDissipation_W": float(np.max(damper_power)),
        "meanDamperDissipation_W": float(np.trapezoid(damper_power, t) / t[-1]),
        "minIndividualSpringForce_N": float(np.min(spring_forces)),
        "maxIndividualSpringForce_N": float(np.max(spring_forces)),
        "minIndividualCompression_mm": float(1000 * np.min(compression)),
        "maxIndividualCompression_mm": float(1000 * np.max(compression)),
        "maxActualHeadVsActuatorGap_mm": float(1000 * np.max(np.abs(head_gap-d[:, 1]))),
        "configuredCommonPreload_N": preload,
        "combinedFreeLengthMax_mm": row["combinedFreeLengthMax_mm"],
        "combinedInstalledLengthMax_mm": row["combinedInstalledLengthMax_mm"],
        "perBumpPeaks": [],
    }
    fig, axes = plt.subplots(3, 1, figsize=(10.3, 6.4), sharex=True)
    tx, low, high = envelope(t, error)
    axes[0].fill_between(tx, low, high, color="#2563ad", alpha=.85, linewidth=0)
    axes[0].plot(tx, low, color="#2563ad", linewidth=.4, alpha=.7)
    axes[0].plot(tx, high, color="#2563ad", linewidth=.4, alpha=.7)
    axes[0].axhline(.5, color="#a53333", linestyle="--")
    axes[0].axhline(-.5, color="#a53333", linestyle="--")
    axes[0].set_ylim(-.55, .55)
    axes[0].set_ylabel("Normal force error (N)")
    axes[0].set_title(f"Selected series spring + damper: repeated sharp bumps, seed {row['seed']}")
    axes[0].annotate(f"Worst {abs(error[peak]):.3f} N", xy=(t[peak], error[peak]),
                     xytext=(t[peak]+.4, .37), arrowprops={"arrowstyle": "->"}, fontsize=9)
    tx, low, high = envelope(t, d[:, 13])
    axes[1].fill_between(tx, low, high, color="#24613d", alpha=.8, linewidth=0)
    axes[1].set_ylabel("Coil current (A)")
    axes[1].axhline(p["coil"]["peakCurrent"], color="#a53333", linestyle="--")
    axes[1].axhline(-p["coil"]["peakCurrent"], color="#a53333", linestyle="--")
    tx, low, high = envelope(t, delta * 1000)
    axes[2].fill_between(tx, low, high, color="#7b4b9d", alpha=.8, linewidth=0)
    axes[2].plot(tx, low, color="#7b4b9d", linewidth=.7)
    axes[2].plot(tx, high, color="#7b4b9d", linewidth=.7)
    axes[2].set_ylabel("Dynamic spring shift (mm)")
    axes[2].set_xlabel("Time (s)")
    axes[2].set_xlim(0, 10)
    for ax in axes:
        ax.grid(alpha=.2)
    fig.tight_layout()
    save(fig, "inertial_mechanics_selected_repeated_20260930")

    fig, ax = plt.subplots(figsize=(10.3, 3.6))
    surface = p["surface"]
    starts = np.arange(surface["startTime"], t[-1]-.1, surface["repeatPeriod"])
    for start in starts:
        mask = (t >= start-.003) & (t <= start+.13)
        ix = np.flatnonzero(mask)
        ix_peak = ix[np.argmax(np.abs(error[mask]))]
        metadata["perBumpPeaks"].append({"startTime_s": float(start),
                                        "peakError_N": float(abs(error[ix_peak])),
                                        "peakTime_s": float(t[ix_peak])})
        ax.plot(t[mask]-start, error[mask], linewidth=.75, alpha=.75,
                label=f"{start:g} s: {abs(error[ix_peak]):.3f} N")
    ax.axhline(.5, color="#a53333", linestyle="--")
    ax.axhline(-.5, color="#a53333", linestyle="--")
    ax.set_ylim(-.55, .55)
    ax.set_xlim(-.003, .13)
    ax.set_xlabel("Time from each bump entry (s)")
    ax.set_ylabel("Normal force error (N)")
    ax.set_title("Each bump in the complete 10 s recording; no discarded startup or post-bump samples")
    ax.grid(alpha=.2)
    ax.legend(fontsize=8, ncol=4, loc="lower center")
    fig.tight_layout()
    save(fig, "inertial_mechanics_selected_bump_overlay_20260930")
    (ROOT / "inertial_mechanics_selected_trace_review_20260930.json").write_text(json.dumps(metadata, indent=2) + "\n")
    return {"file": path.name, "sha256": digest(path), "traceSHA256": digest(trace)}


if __name__ == "__main__":
    sources = [spring_tuning(), selected_repeat()]
    manifest = {"sources": sources, "figures": []}
    for name in ["inertial_mechanics_spring_tuning_20260930",
                 "inertial_mechanics_selected_repeated_20260930",
                 "inertial_mechanics_selected_bump_overlay_20260930"]:
        for suffix in ["svg", "png"]:
            path = ROOT / f"{name}.{suffix}"
            manifest["figures"].append({"file": path.name, "sha256": digest(path)})
    (ROOT / "inertial_mechanics_figure_manifest_20260930.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"figureCount": len(manifest["figures"]), "sources": sources}))
