#!/usr/bin/env python3
"""Summarize only the bounded native characterization/precision experiments."""
from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

folder = Path(__file__).resolve().parent
report = json.loads((folder/"actuator_qualification_actual_sensor_calibration.json").read_text())
rows = report["rows"]
assert all(r["simulated"] for r in rows)
summary = []
for config in report["configs"]:
    scenario = report["scenarios"][config["scenario"]-1]["name"]
    group = [r for r in rows if r["scenario"] == scenario]
    expected = 16 if scenario == "matched_joint_winding" else 4
    if len(group) != expected:
        continue
    summary.append({"scenario": scenario, "nativeRuns": len(group),
                    "fullPassed": sum(r["passed"] for r in group),
                    "worstFullError_N": max(r["peakError_N"] for r in group),
                    "worstBumpRecoveryError_N": max(r["bumpRecoveryPeakError_N"] for r in group),
                    "minimumCurrentReserve_A": min(r["currentReserve_A"] for r in group),
                    "minimumVoltageReserve_V": min(r["voltageReserve_V"] for r in group),
                    "maximumHoldCopperLoss_W": max(r["holdCopperLoss_W"] for r in group),
                    "actualKf_N_A": group[0]["actualKf_N_A"],
                    "actualCurrentResponse_ms": group[0]["actualCurrentResponse_ms"],
                    "matchedOuterModel": group[0]["matchedOuterModel"]})
result = {"sourceStatus": report["status"], "completedNativeRuns": report["simulations"], "scenarios": summary,
          "interpretation": "Matched joint cohort tests hardware capacity with identified parameters. Precision cases are OFAT unknown outer-model errors at50N/90deg only; they are not joint tolerance or full-range calibration qualification."}
(folder/"actuator_qualification_actual_sensor_calibration_summary.json").write_text(json.dumps(result, indent=2)+"\n")
if report["status"] == "complete":
    assert len(rows) == 48 and report["simulations"] == 48 and len(summary) == 9
    baseline_report = json.loads((folder/"actuator_qualification_actual_sensor_winding.json").read_text())
    baseline_rows = [r for r in baseline_report["rows"] if r["scenario"] == "nominal" and r["Ilimit_A"] == 3.5
                     and r["Vlimit_V"] == 24 and r["force_N"] == 50 and r["angle_deg"] == 90]
    assert len(baseline_rows) == 4
    baseline = max(r["peakError_N"] for r in baseline_rows)
    entries = {r["scenario"]: r for r in summary}
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11, "axes.titleweight": "bold"})
    fig, axes = plt.subplots(1, 2, figsize=(13, 6.8))
    fig.subplots_adjust(left=.08, right=.975, top=.79, bottom=.35, wspace=.26)
    tests = [([-0.5, -0.1, 0, 0.1, 0.5], ["Kf_minus_0p5pct", "Kf_minus_0p1pct", None, "Kf_plus_0p1pct", "Kf_plus_0p5pct"],
              "Unknown force-constant error", "Physical Kf deviation from model (%)", "#0072b2"),
             ([-5, -1, 0, 1, 5], ["tau_minus_5pct", "tau_minus_1pct", None, "tau_plus_1pct", "tau_plus_5pct"],
              "Unknown current-response error", "Physical response-time deviation (%)", "#e69f00")]
    for ax, (x, keys, title, xlabel, color) in zip(axes, tests):
        y = [entries[k]["worstFullError_N"] if k else baseline for k in keys]
        ax.plot(x, y, "o-", color=color, lw=1.5, markersize=6)
        for px, py in zip(x, y):
            ax.annotate(f"{py:.3f}", (px, py), xytext=(0, 8), textcoords="offset points", ha="center", fontsize=9)
        ax.axhline(.5, color="#334155", ls="--", lw=1.3, label="0.5 N requirement")
        ax.set_xticks(x); ax.set_xlabel(xlabel); ax.set_ylabel("Worst true contact-force error (N)")
        ax.set_title(title); ax.grid(alpha=.2); ax.set_axisbelow(True)
        ax.set_ylim(0, max(.65, max(y)*1.2))
        ax.legend(frameon=False, fontsize=9, loc="upper left")
    matched = entries["matched_joint_winding"]
    fig.suptitle("Calibration precision for the actual two-sensor controller", fontsize=17, fontweight="bold", y=.975)
    fig.text(.5, .91, "3.5 A /24 V; B/S cell with 1.4 kg downstream; S−A position; both attachments rigid", ha="center", fontsize=11.5)
    fig.text(.08, .235, f"Identified joint winding (16 corners/seeds): Kf18 N/A, R5.2 Ω, L3.9 mH, response125 µs; "
             f"{matched['fullPassed']}/16 pass, worst {matched['worstFullError_N']:.3f} N.", fontsize=10)
    fig.text(.08, .19, "Precision plots: 50 N /90° only, four noise seeds per point; nominal model Kf20 N/A and response100 µs.", fontsize=10)
    fig.text(.08, .145, "0.02 N force RMS, 1 µm position RMS, known100 µs sensor delays/filter; 10 kHz controller; no actual-current readback.", fontsize=10)
    fig.text(.08, .10, "Each precision parameter changes separately. These finite probes do not establish simultaneous tolerance or full-range calibration limits.", fontsize=10)
    fig.text(.08, .055, "The inner drive remains calibrated. Actual identification and validation over position/temperature are prototype work.", fontsize=10)
    for ext in ("png", "svg"):
        fig.savefig(folder/f"actuator_qualification_actual_sensor_calibration_precision.{ext}", dpi=170, facecolor="white")
print(json.dumps(result, indent=2))
