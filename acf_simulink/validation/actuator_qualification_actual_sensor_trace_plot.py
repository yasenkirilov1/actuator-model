#!/usr/bin/env python3
"""Export and draw actual native two-sensor traces; no fabricated trajectories."""
import json
from pathlib import Path
import sys
sys.path.insert(0, "/home/tommie/.local/state/actuator-model/qualification-20260930/actuators/python_deps")
import h5py
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

folder = Path(__file__).resolve().parent
stem = "actuator_qualification_actual_sensor_winding"
source = folder / f"{stem}_checkpoint.mat"
def string(dataset):
    return "".join(chr(int(x)) for x in dataset[()].flatten())
def scalar(group, key):
    return float(group[key][0, 0])

scenarios = ["nominal", "Kf_minus10_unknown", "tau_plus25_unknown", "joint_winding"]
labels = ["Nominal", "Unknown Kf −10%", "Unknown response +25%", "Joint winding"]
colors = ["#0072b2", "#d55e00", "#e69f00", "#cc79a7"]
columns = [0, 1, 2, 3, 12, 13, 14, 30, 31, 32]
names = ["Fc_N", "travel_m", "gapSpeed_m_s", "gapAcceleration_m_s2", "actuatorForce_N",
         "current_A", "voltage_V", "requestedForce_N", "measuredForceTared_N", "currentCommand_A"]
records = {}
metadata = []
with h5py.File(source, "r") as f:
    for ref in f["traces"][()].flatten():
        r = f[ref]
        scenario = string(r["windingActual"]["name"])
        if scenario not in scenarios or scalar(r["parameters"]["sensor"], "seed") != 1729:
            continue
        assert scenario not in records
        t = r["time"][()].flatten(); d = r["data"][columns, :].T
        peak = float(np.max(np.abs(d[:, 0] - d[:, 7])))
        assert abs(peak - scalar(r["metrics"], "peakAbsForceError_N")) < 1e-9
        csv = folder/f"actuator_qualification_actual_sensor_trace_{scenario}.csv"
        np.savetxt(csv, np.column_stack([t, d]), delimiter=",", header=",".join(["time_s"]+names),
                   comments="", fmt="%.12g")
        records[scenario] = (t, d)
        metadata.append({"scenario": scenario, "csv": str(csv), "sourceMat": str(source),
                         "force_N": scalar(r["parameters"]["force"], "reference"), "angle_deg": 90,
                         "seed": 1729, "peakError_N": peak,
                         "peakCurrent_A": float(np.max(np.abs(d[:, 5]))),
                         "peakVoltage_V": float(np.max(np.abs(d[:, 6]))),
                         "samples": len(t), "start_s": float(t[0]), "end_s": float(t[-1])})
assert set(records) == set(scenarios), "Need complete selected-drive trace cohorts."
(folder/"actuator_qualification_actual_sensor_trace_metadata.json").write_text(
    json.dumps({"scope": "50N/90deg, seed1729, actual B/S cell with exact static gravity tare and S-A position, outer model nominal, no measured current", "traces": metadata}, indent=2)+"\n")

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10.5})
fig, axes = plt.subplots(2, 2, figsize=(13.2, 8.5))
fig.subplots_adjust(left=.075, right=.98, top=.79, bottom=.22, hspace=.4, wspace=.26)
for scenario, label, color in zip(scenarios, labels, colors):
    t, d = records[scenario]; err = d[:, 0]-d[:, 7]
    zoom = (t >= 1.97) & (t <= 2.2)
    axes[0, 0].plot(t, err, color=color, lw=.6, label=label)
    axes[0, 1].plot(t[zoom], err[zoom], color=color, lw=.8)
    axes[1, 0].plot(t[zoom], d[zoom, 5], color=color, lw=.8)
    axes[1, 1].plot(t[zoom], d[zoom, 6], color=color, lw=.7)
for ax in axes[0]:
    ax.axhspan(-.5, .5, color="#ecf7ed", zorder=-10)
    ax.axhline(.5, color="#334155", ls="--", lw=1)
    ax.axhline(-.5, color="#334155", ls="--", lw=1)
    ax.set_ylabel("True contact-force error (N)")
axes[0, 0].set_title("Whole run: startup and sustained error"); axes[0, 0].set_xlim(0, 4)
axes[0, 1].set_title("Bump response: error remains after settling")
axes[1, 0].set_title("Actual current during the bump"); axes[1, 0].set_ylabel("Actual current (A)")
axes[1, 0].axhline(3.5, color="#334155", ls="--", lw=1)
axes[1, 1].set_title("Applied voltage during the bump"); axes[1, 1].set_ylabel("Applied voltage (V)")
axes[1, 1].axhline(24, color="#334155", ls="--", lw=1)
for ax in axes.flat:
    ax.set_xlabel("Time (s)"); ax.grid(alpha=.2); ax.set_axisbelow(True)
for ax in [axes[0, 1], axes[1, 0], axes[1, 1]]:
    ax.set_xlim(1.97, 2.2)
fig.suptitle("Actuator-model mismatch with the actual two-sensor controller", fontsize=17, fontweight="bold", y=.97)
fig.text(.5, .92, "50 N /90°; 1 mm ×10 mm bump at 100 mm/s; B+S =2 kg; force cell has 1.4 kg downstream", ha="center", fontsize=11.5)
fig.legend(*axes[0, 0].get_legend_handles_labels(), loc="upper center", bbox_to_anchor=(.5, .88),
           ncol=4, frameon=False, fontsize=10)
fig.text(.075, .115, "3.5 A/24 V; force RMS 0.02 N, S−A position RMS 1 µm; seed1729; no actual-current measurement or surface preview.", fontsize=10)
fig.text(.075, .077, "Outer observer remains Kf20 N/A and 100 µs current response. Joint case: actual Kf18, R5.2 Ω, L3.9 mH, response125 µs.", fontsize=10)
fig.text(.075, .039, "Inner current driver is calibrated; persistent gain-mismatch error shows that nominal force capacity does not establish controller robustness.", fontsize=10)
for ext in ("png", "svg"):
    fig.savefig(folder/f"actuator_qualification_actual_sensor_traces.{ext}", dpi=170, facecolor="white")
print(json.dumps(metadata, indent=2))
