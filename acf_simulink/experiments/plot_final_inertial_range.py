"""Plot the completed, audited native grid and its retained worst recording.

The coverage audit must succeed first. This reads native evidence only;
it does not run a surrogate or replace the full-history force metric.
"""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[2]
V = ROOT / "acf_simulink" / "validation"
DEST = ROOT / "reports" / "figures"
SUMMARY = V / "normal_mass_controller_final_grid_20260930_summary.json"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(fig, name):
    fig.savefig(DEST / f"{name}.png", dpi=180, bbox_inches="tight")
    fig.savefig(DEST / f"{name}.svg", bbox_inches="tight")
    plt.close(fig)


def main():
    summary = json.loads(SUMMARY.read_text())
    assert summary["status"] == "complete_audited_finite_nominal_grid"
    assert summary["coverage"]["auditPassed"] and summary["coverage"]["complete"]
    assert summary["validatedNativeHistories"] == 3500
    rows_file = V / "normal_mass_controller_final_grid_20260930_rows.csv"
    with rows_file.open() as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 3500
    masses = [.1, .5, 1, 1.4, 2]
    forces, angles = list(range(5, 51, 5)), list(range(0, 91, 15))
    grouped = summary["groupedSurface"]
    assert len(grouped) == 700 and all(r["seedCases"] == 5 for r in grouped)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "figure.facecolor": "white", "axes.facecolor": "white"})
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.5), layout="constrained")
    mass_peaks = []
    for mass, ax in zip(masses, axes.flat):
        grid = np.full((10, 7), np.nan)
        for i, force in enumerate(forces):
            for j, angle in enumerate(angles):
                cohort = [r for r in grouped if r["massS_kg"] == mass and
                          r["reference_N"] == force and r["angle_deg"] == angle]
                assert len(cohort) == 2 and all(r["seedCases"] == 5 for r in cohort)
                grid[i, j] = max(r["worstNormalForceError_N"] for r in cohort)
        im = ax.imshow(grid, origin="lower", vmin=0, vmax=.5,
                       cmap="YlOrBr", aspect="auto")
        for i in range(10):
            for j in range(7):
                ax.text(j, i, f"{grid[i,j]:.3f}", ha="center", va="center",
                        fontsize=7.5, color="white" if grid[i,j] > .35 else "#222")
        ax.set_xticks(range(7), angles)
        ax.set_yticks(range(10), forces)
        ax.set_xlabel("Fixed flange orientation (°)")
        ax.set_ylabel("Normal-force target (N)")
        ax.set_title(f"S = {mass:g} kg · 700 recordings")
        mass_peaks.append(float(np.max(grid)))
    ax = axes.flat[-1]
    ax.bar(range(5), mass_peaks, color="#087f8c")
    ax.axhline(.5, color="#222", ls="--", label="±0.5 N requirement")
    for i, value in enumerate(mass_peaks):
        ax.text(i, value+.012, f"{value:.3f}", ha="center")
    ax.set_xticks(range(5), [f"{m:g}" for m in masses])
    ax.set_xlabel("Downstream tool S mass (kg)")
    ax.set_ylabel("Worst full-history force error (N)")
    ax.set_ylim(0, .57)
    ax.set_title(f"{summary['passed']:,} / 3,500 cases pass")
    ax.legend(frameon=False, loc="upper left", fontsize=9)
    fig.colorbar(im, ax=list(axes.flat[:5]), shrink=.8,
                 label="Worst native force error over both drag ratios and five seeds (N)")
    save(fig, "inertial_final_range_coverage")

    trace_path = Path(summary["worstNativeTrace"])
    assert digest(trace_path) == summary["worstNativeTraceSHA256"]
    loaded = loadmat(trace_path, simplify_cells=True)
    native = loaded.get("worst", loaded.get("r"))
    assert isinstance(native, dict)
    t, d = native["time"], native["data"]
    normal = native["normalContact"]["data"]
    reference = d[:, 30]
    error = normal[:, 0] - reference
    estimated_error = normal[:, 3] - reference
    assert len(t) == 400001 and t[-1] == 4
    peak_index = int(np.argmax(np.abs(error)))
    peak = float(abs(error[peak_index]))
    assert abs(peak - summary["worstCase"]["peakNormalForceError_N"]) < 1e-12
    p = native["parameters"]
    entry = p["surface"]["startTime"]
    take = (t >= entry-.02) & (t <= entry+.20)
    assert take[peak_index], "Worst sample is outside the declared bump plot window"
    z = (t[take]-entry)*1000
    gate = native["robotGate"]
    gate_index = np.clip(np.searchsorted(gate["time"], t[take], side="right")-1,
                         0, len(gate["time"])-1)
    excerpt = []
    for at, truth, estimate, current, voltage, travel, ready in zip(
        t[take], error[take], estimated_error[take], d[take,13],
        d[take,14], d[take,1]*1000, gate["ready"][gate_index]):
        excerpt.append({"time_s": float(at), "normalError_N": float(truth),
                        "estimatedNormalError_N": float(estimate),
                        "current_A": float(current), "voltage_V": float(voltage),
                        "actuatorTravel_mm": float(travel), "robotGateReady": bool(ready)})
    excerpt_file = V / "inertial_final_worst_native_excerpt_20260930.json"
    excerpt_file.write_text(json.dumps({
        "status": "complete_read_only_native_excerpt", "newSimulations": 0,
        "nativeTrace": str(trace_path), "nativeTraceSHA256": digest(trace_path),
        "gridSummarySHA256": digest(SUMMARY), "nativeSamplesScored": len(t),
        "peakNormalForceError_N": peak, "peakTime_s": float(t[peak_index]),
        "case": summary["worstCase"], "rows": excerpt,
        "scope": "Every native10us sample from20ms before to200ms after bump entry; metric uses all400001 samples. Gate is held from its native100Hz history.",
    }, indent=2) + "\n")
    fig, axes = plt.subplots(3, 2, figsize=(13, 9), layout="constrained")
    axes[0,0].plot(z, error[take], color="#087f8c", lw=1.2, label="Actual surface force")
    axes[0,0].plot(z, estimated_error[take], color="#564592", lw=.9,
                   alpha=.85, label="Sensor reconstruction")
    axes[0,0].set_title("Sharp bump: actual and reconstructed normal force")
    axes[0,0].set_ylabel("Force error (N)")
    axes[0,0].legend(frameon=False, fontsize=8)
    zoom = take & (t >= entry) & (t <= entry+.003)
    axes[0,1].plot((t[zoom]-entry)*1000, error[zoom], color="#087f8c", lw=1.4)
    axes[0,1].plot((t[zoom]-entry)*1000, estimated_error[zoom], color="#564592", lw=1)
    axes[0,1].scatter([(t[peak_index]-entry)*1000], [error[peak_index]],
                      color="#d1495b", s=20, zorder=4)
    axes[0,1].set_title(f"Entry detail · native worst {peak:.6f} N")
    axes[0,1].set_ylabel("Force error (N)")
    for ax in axes[0]:
        ax.axhline(.5, color="#222", ls="--", lw=.8)
        ax.axhline(-.5, color="#222", ls="--", lw=.8)
    axes[1,0].plot(z, d[take,13], color="#087f8c", lw=1)
    axes[1,0].set_ylabel("Coil current (A)")
    axes[1,0].set_title("Native winding current")
    axes[1,1].plot(z, d[take,14], color="#d18b20", lw=1)
    axes[1,1].set_ylabel("Winding voltage (V)")
    axes[1,1].set_title("Drive demand at selected100µs response")
    axes[2,0].plot(z, d[take,1]*1000, color="#564592", lw=1.2)
    axes[2,0].set_ylabel("Actuator B–A travel (mm)")
    axes[2,0].set_title("Measured plant travel; not S–A encoder distance")
    gate_take = (gate["time"] >= entry-.02) & (gate["time"] <= entry+.20)
    axes[2,1].step((gate["time"][gate_take]-entry)*1000,
                   gate["ready"][gate_take], where="post", color="#087f8c", lw=1.5)
    axes[2,1].set_ylim(-.1, 1.1)
    axes[2,1].set_yticks([0,1], ["Hold new commands", "Ready"])
    axes[2,1].set_title("Native robot recentering gate")
    for ax in axes.flat:
        ax.set_xlabel("Time from bump entry (ms)")
        ax.grid(alpha=.2)
    save(fig, "inertial_final_worst_native")
    manifest_path = V / "inertial_report_figure_sources.json"
    manifest = json.loads(manifest_path.read_text())
    common = {"kind": "native_simulation", "file": str(SUMMARY.relative_to(ROOT)),
              "sha256": digest(SUMMARY), "rowsCSV_SHA256": digest(rows_file),
              "scope": summary["qualificationScope"], "rows": 3500,
              "nativeSamplesScored": summary["nativeSamplesScored"]}
    manifest["inertial_final_range_coverage"] = common
    manifest["inertial_final_worst_native"] = {
        **common, "nativeTrace": str(trace_path), "nativeTraceSHA256": digest(trace_path),
        "excerptFile": str(excerpt_file.relative_to(ROOT)), "excerptSHA256": digest(excerpt_file),
        "plotScope": "All native samples in the displayed bump window; complete history determines the reported metric."}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"cases": len(rows), "peakError_N": peak,
                      "worstTraceSHA256": digest(trace_path), "excerptRows": len(excerpt)}))


if __name__ == "__main__":
    main()
