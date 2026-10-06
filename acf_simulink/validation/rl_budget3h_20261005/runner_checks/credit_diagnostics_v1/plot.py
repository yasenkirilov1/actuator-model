"""Render the retained randomized-current analysis without rerunning it."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
data = json.loads((HERE / "results.json").read_text())
plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 11,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.edgecolor": "#9AA4AD", "text.color": "#273746",
    "axes.labelcolor": "#273746", "xtick.color": "#52616C",
    "ytick.color": "#52616C", "svg.fonttype": "none",
})
fig, axes = plt.subplots(1, 2, figsize=(12, 5.6))
colors = ("#205E86", "#AD6B13")
for (name, run), color in zip(data["runs"].items(), colors):
    response = np.asarray(run["impulse_N_A"])
    time_ms = np.arange(len(response)) * data["dt_s"] * 1000
    axes[0].plot(time_ms, response, lw=1.4, color=color,
                 label=f"{name[-4:]} · {run['trainingDecisions']:,} decisions")
axes[0].axhline(0, color="#A0ABB3", lw=.8)
axes[0].set_xlim(0, 8)
axes[0].set_xlabel("Lag after randomized current decision (ms)")
axes[0].set_ylabel("Estimated local response (N/A)")
axes[0].set_title("Reconstructed measured-force response", loc="left", fontsize=12, fontweight="bold")
axes[0].legend(frameon=False, fontsize=9.5)
time = np.linspace(0, 120, 500)
for tau, color, style, label in (
    (5, "#945572", ":", "5 ms force · considered, not run"),
    (10, "#AD6B13", "--", "10 ms force"),
    (20, "#205E86", "-", "20 ms force"),
    (300, "#536E3E", "-.", "300 ms robot"),
):
    axes[1].plot(time, np.exp(-time / tau), color=color, lw=1.7, ls=style, label=label)
axes[1].axvline(80, color="#8D989F", lw=.9, ls=":")
axes[1].text(82, .55, "80 ms\nnominal robot delay", color="#596772", fontsize=9.5)
axes[1].set_xlim(0, 120)
axes[1].set_ylim(0, 1.05)
axes[1].set_xlabel("Reward delay (ms)")
axes[1].set_ylabel("GAE trace weight, before common discount")
axes[1].set_title("Credit traces for the two control timescales", loc="left", fontsize=12, fontweight="bold")
axes[1].legend(frameon=False, fontsize=9.3, loc="upper right")
for ax in axes:
    ax.grid(axis="y", color="#E6EBEF", lw=.8)
    ax.set_axisbelow(True)
fig.suptitle("Current perturbations and force-credit timing", x=.064, y=.97, ha="left", fontsize=17, fontweight="bold")
fig.text(.064, .893, "Retained training data only; no test cases and no additional controller experiment.", fontsize=11)
fig.text(.064, .122, "90% of estimated response energy appears by 1.3–1.4 ms; 98.3–99.3% by 5 ms in these two runs.", fontsize=10)
fig.text(.064, .075, "This is a local closed-loop estimate including acquisition and policy reaction. Mixed cases and noisy correlation tails", fontsize=9.4)
fig.text(.064, .039, "limit its interpretation; it does not establish open-loop actuator bandwidth or hardware performance.", fontsize=9.4)
fig.subplots_adjust(left=.072, right=.98, top=.79, bottom=.24, wspace=.30)
fig.savefig(HERE / "current_innovation_credit.svg")
fig.savefig(HERE / "current_innovation_credit.png", dpi=155)
plt.close(fig)
