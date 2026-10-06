#!/usr/bin/env python3
"""Evidence-only figures for the frozen actual B/S and S-A controller."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
import numpy as np

folder = Path(__file__).resolve().parent
source = folder / "actuator_qualification_actual_sensor_winding.json"
report = json.loads(source.read_text())
rows = report["rows"]
assert all(r["simulated"] for r in rows), "Execution errors must be resolved before drawing performance."
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                     "axes.titleweight": "bold"})
scenarios = ["nominal", "Kf_minus10_unknown", "tau_plus25_unknown", "joint_winding"]
labels = ["Nominal", "Unknown Kf −10%", "Unknown response +25%", "Joint winding"]
summaries = []
def finite_max(group, key):
    values = [r[key] for r in group if isinstance(r.get(key), (float, int))]
    return max(values) if values else None

for scenario in scenarios:
    group = [r for r in rows if r["Ilimit_A"] == 3.5 and r["Vlimit_V"] == 24
             and r["scenario"] == scenario]
    if len(group) != 16:
        continue
    entry = {"scenario": scenario, "nativeRuns": len(group),
             "fullPassed": sum(r["passed"] for r in group),
             "bumpRecoveryPassed": sum(r["bumpRecoveryPassed"] for r in group),
             "worstFullError_N": finite_max(group, "peakError_N"),
             "worstStartupError_N": finite_max(group, "startupPeakError_N"),
             "worstPrebumpError_N": finite_max(group, "prebumpPeakError_N"),
             "worstBumpRecoveryError_N": finite_max(group, "bumpRecoveryPeakError_N"),
             "worstFinalError_N": finite_max(group, "finalPeakError_N"),
             "lostContactRuns": sum(r["lostContact"] for r in group),
             "completedRuns": sum(r["completed"] for r in group),
             "bumpWindowEvaluatedRuns": sum(isinstance(r["bumpRecoveryPeakError_N"], (float, int)) for r in group),
             "minimumCurrentReserve_A": min(r["currentReserve_A"] for r in group),
             "minimumVoltageReserve_V": min(r["voltageReserve_V"] for r in group),
             "maximumRequestedVoltage_V": max(r["peakRequestedVoltage_V"] for r in group),
             "maximumHoldCopperLoss_W": max(r["holdCopperLoss_W"] for r in group)}
    summaries.append(entry)
if len(summaries) == 4:
    x = np.arange(4)
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.6))
    fig.subplots_adjust(left=.075, right=.975, top=.79, bottom=.34, wspace=.25)
    for metric, shift, color, label in [
        ("worstFullError_N", -.20, "#0072b2", "Full 0–4 s"),
        ("worstBumpRecoveryError_N", .20, "#e69f00", "Bump + recovery 2–2.4 s")]:
        values = [r[metric] for r in summaries]
        bars = axes[0].bar(x + shift, values, width=.38, color=color, label=label)
        for bar, value in zip(bars, values):
            axes[0].annotate(f"{value:.3f}", (bar.get_x() + bar.get_width()/2, value),
                             xytext=(0, 4), textcoords="offset points", ha="center", fontsize=9)
    axes[0].axhline(.5, color="#334155", ls="--", lw=1.3, label="0.5 N requirement")
    axes[0].set_ylabel("Worst true contact-force error (N)")
    axes[0].set_title("Startup retained in the full score")
    axes[0].set_xticks(x, labels, rotation=18, ha="right")
    axes[0].legend(frameon=False, fontsize=9, loc="upper left")
    axes[0].set_ylim(0, max(r["worstFullError_N"] for r in summaries) * 1.24)
    reserves = [r["minimumCurrentReserve_A"] for r in summaries]
    bars = axes[1].bar(x, reserves, color="#009e73", width=.65)
    for bar, value in zip(bars, reserves):
        axes[1].annotate(f"{value:.3f} A", (bar.get_x()+bar.get_width()/2, value),
                         xytext=(0, 4), textcoords="offset points", ha="center", fontsize=9)
    axes[1].set_title("Reserve below the 3.5 A limit")
    axes[1].set_ylabel("Minimum actual-current reserve (A)")
    axes[1].set_xticks(x, labels, rotation=18, ha="right")
    axes[1].set_ylim(min(0, min(reserves) * 1.2), max(reserves) * 1.3)
    for ax in axes:
        ax.grid(axis="y", alpha=.2); ax.set_axisbelow(True)
    fig.suptitle("Coil-model sensitivity with the actual prototype sensor layout",
                 fontsize=17, fontweight="bold", y=.975)
    fig.text(.5, .91, "B/S load cell: 1.4 kg downstream; S−A tool position; both attachments rigid; 3.5 A /24 V",
             ha="center", fontsize=11.5)
    fig.text(.075, .20, "16 native runs per case: 5/50 N × 0/90° × four seeds; 0.02 N force RMS and 1 µm position RMS.", fontsize=10)
    fig.text(.075, .16, "Known 100 µs force/position delays and filter; 10 kHz controller; nominal 100 µs current response.", fontsize=10)
    fig.text(.075, .12, "Outer force/current observer stays nominal. Joint: physical Kf −10%, R/L +30%, response +25%.", fontsize=10)
    fig.text(.075, .08, "Inner drive uses calibrated R/L/back EMF. Nominal holding-current initial condition is preserved.", fontsize=10)
    fig.text(.075, .04, "The bump score is supplementary: it does not replace the full-run pass criterion or qualify untested hardware.", fontsize=10)
    for ext in ("png", "svg"):
        fig.savefig(folder/f"actuator_qualification_actual_sensor_selected.{ext}", dpi=170, facecolor="white")
    plt.close(fig)

if report["status"] == "complete":
    assert report["simulations"] == 320 and len(rows) == 320
    currents = [3, 3.5, 4]; voltages = [18, 24, 30]
    matrices = [np.full((3, 3), np.nan) for _ in range(3)]
    matrix_summary = []
    for i, current in enumerate(currents):
        for j, voltage in enumerate(voltages):
            group = [r for r in rows if r["Ilimit_A"] == current and r["Vlimit_V"] == voltage
                     and r["scenario"] in ("nominal", "joint_winding")]
            assert len(group) == 32
            full = max(r["peakError_N"] for r in group)
            bump = finite_max(group, "bumpRecoveryPeakError_N")
            reserve = min(r["currentReserve_A"] for r in group)
            vreserve = min(r["voltageReserve_V"] for r in group)
            for matrix, value in zip(matrices, [full, bump, reserve]):
                matrix[i, j] = value
            matrix_summary.append({"currentLimit_A": current, "voltageLimit_V": voltage,
                                   "nativeRuns": 32, "fullPassed": sum(r["passed"] for r in group),
                                   "bumpPassed": sum(r["bumpRecoveryPassed"] for r in group),
                                   "worstFullError_N": full, "worstBumpRecoveryError_N": bump,
                                   "minimumCurrentReserve_A": reserve, "minimumVoltageReserve_V": vreserve})
    fig, axes = plt.subplots(1, 3, figsize=(14, 6.1))
    fig.subplots_adjust(left=.06, right=.94, top=.78, bottom=.30, wspace=.43)
    panels = [(matrices[0], "Whole-run force error (N)", "YlOrRd", Normalize(0, max(1, np.nanmax(matrices[0])))),
              (matrices[1], "Bump + recovery error (N)", "YlOrRd", Normalize(0, max(.6, np.nanmax(matrices[1])))),
              (matrices[2], "Minimum current reserve (A)", "YlGn", Normalize(0, max(1, np.nanmax(matrices[2]))))]
    for ax, (matrix, title, cmap, norm) in zip(axes, panels):
        im = ax.imshow(matrix, cmap=cmap, norm=norm, origin="lower", aspect="auto")
        ax.set_title(title, pad=12)
        ax.set_xticks(range(3), [str(v) for v in voltages]); ax.set_yticks(range(3), [str(i) for i in currents])
        ax.set_xlabel("Supply voltage (V)"); ax.set_ylabel("Current command limit (A)")
        for i in range(3):
            for j in range(3):
                ax.text(j, i, f"{matrix[i,j]:.3f}", ha="center", va="center", fontweight="bold", color="#101820")
        fig.colorbar(im, ax=ax, fraction=.045, pad=.03)
    fig.suptitle("Current and voltage boundaries with the actual B/S sensor controller", fontsize=17, fontweight="bold", y=.975)
    fig.text(.5, .91, "Both rigid; actual S−A tool position; 1 mm ×10 mm bump at 100 mm/s; 2 kg tool + holder", ha="center", fontsize=11.5)
    fig.text(.055, .19, "Each cell: 32 native runs = nominal + joint winding × four force/orientation corners × four noise seeds.", fontsize=10)
    fig.text(.055, .145, "Joint winding: Kf −10%, R/L +30%, response +25%; outer observer nominal, inner current driver calibrated.", fontsize=10)
    fig.text(.055, .10, "Whole-run scores retain nominal-current startup under unknown Kf. Bump + recovery is the separately reported 2–2.4 s window.", fontsize=10)
    fig.text(.055, .055, "Force requirement is 0.5 N throughout. Independent electrical tolerances do not represent a qualified real coil winding.", fontsize=10)
    fig.text(.055, .022, "All 144 nominal matrix runs passed; all 144 joint matrix runs failed. The remaining32 study runs isolate Kf and response changes.", fontsize=10)
    for ext in ("png", "svg"):
        fig.savefig(folder/f"actuator_qualification_actual_sensor_matrix.{ext}", dpi=170, facecolor="white")
    plt.close(fig)
else:
    matrix_summary = []

summary = {"source": str(source), "sourceStatus": report["status"], "completedNativeRuns": report["simulations"],
           "selectedDrive": summaries, "currentVoltageMatrix": matrix_summary,
           "fullCriterion": "true-force error <=0.5N over 0–4s, no contact loss/travel stop/overcurrent; bump-window score is supplementary"}
(folder/"actuator_qualification_actual_sensor_summary.json").write_text(json.dumps(summary, indent=2)+"\n")
print(json.dumps(summary, indent=2))
