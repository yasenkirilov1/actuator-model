"""Plot the final native recentering study; preserve the plot inputs in JSON."""
from pathlib import Path
import json
import shutil

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[2]
VALIDATION = ROOT / "acf_simulink/validation"
ASSETS = ROOT / "reports/prototype-design-20260930/src/content/assets"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                     "axes.spines.top": False, "axes.spines.right": False,
                     "figure.dpi": 140, "savefig.dpi": 180})


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    return value


def save(figure, stem):
    for extension in ("png", "svg"):
        figure.savefig(VALIDATION / f"{stem}.{extension}", bbox_inches="tight")
    shutil.copy2(VALIDATION / f"{stem}.png", ASSETS / f"{stem}.png")
    plt.close(figure)


data = loadmat(VALIDATION / "final_robot_gate_plot_data.mat", simplify_cells=True)
traces = data["traces"]
colors = {"ungated": "#a44047", "gate_only": "#b77f21",
          "gate_and_delay_gain_cap": "#176c9c"}
labels = {"ungated": "Ungated, clean", "gate_only": "Gate only, clean",
          "gate_and_delay_gain_cap": "Gate + gain cap"}
figure, axes = plt.subplots(3, 2, figsize=(11, 9), sharex="col")
for column, delay in enumerate((0.08, 3.0)):
    for trace in traces:
        if not np.isclose(trace["delay_s"], delay):
            continue
        policy = trace["policy"]
        noisy = bool(trace["noisy"])
        label = labels[policy]
        if policy == "gate_and_delay_gain_cap":
            label += ", noisy" if noisy else ", clean"
        color, style = colors[policy], "--" if noisy else "-"
        time = trace["time_s"]
        axes[0, column].plot(time, trace["travelError_mm"], color=color,
                             ls=style, lw=1.5, label=label)
        axes[1, column].plot(time, trace["forceError_N"], color=color,
                             ls=style, lw=1.0)
        gate_time = trace["gateTime_s"]
        ready = trace["gateReady"]
        window = min(100, len(ready))
        average = np.convolve(ready, np.ones(window) / window, mode="valid")
        axes[2, column].plot(gate_time[window - 1:], average, color=color,
                             ls=style, lw=1.0)
        trace["plotGateAverageWindow_s"] = (window - 1) * 0.01
        trace["plotGateAverageTime_s"] = gate_time[window - 1:]
        trace["plotGateAverageReadyFraction"] = average
        if not trace["metricsFromFullNativeGrid"]["completed"]:
            axes[0, column].plot(time[-1], trace["travelError_mm"][-1],
                                 "x", color=color, ms=10, mew=2)
    axes[0, column].set_title(f"Robot command delay: {delay:g} s")
    axes[0, column].axhspan(-0.5, 0.5, color="#5b9854", alpha=0.12)
    axes[1, column].axhline(0, color="#68727d", lw=0.7)
    axes[2, column].set_ylim(-0.03, 1.03)
    axes[2, column].set_xlabel("Time (s)")
    for axis in axes[:, column]:
        axis.grid(alpha=0.17)
    axes[0, column].legend(fontsize=8, loc="best")
axes[0, 0].set_ylabel("Travel error from midpoint (mm)")
axes[1, 0].set_ylabel("True contact-force error (N)")
axes[2, 0].set_ylabel("Gate ready (1 s moving average)")
figure.suptitle("Stable force alone does not stabilize delayed recentering")
figure.text(0.08, 0.01,
            "Actual B–S force cell and S–A position. Both links rigid. "
            "× = travel-stop failure. Metrics use every native sample; curves use 10 ms samples.",
            fontsize=9)
figure.tight_layout(rect=(0, 0.03, 1, 0.97))
save(figure, "final_robot_gate_response")
evidence = {
    "source": "final_robot_gate_plot_data.mat",
    "metricsSource": "final_robot_gate_results.json",
    "scope": "Native two-sensor recentering; 100 ms force-gate filter, -4 mm initial offset",
    "displaySampling_s": 0.01,
    "metricsSampling": "Every native 50 us plant sample",
    "rows": plain(traces),
}
(VALIDATION / "final_robot_gate_plot_data.json").write_text(
    json.dumps(evidence, indent=2) + "\n")
print("Created final native robot-gate figure and inspectable plot inputs.")
