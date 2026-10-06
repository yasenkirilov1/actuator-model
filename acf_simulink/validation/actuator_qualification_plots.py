"""Plot the recorded coil screen; no simulated data are synthesized here."""
from pathlib import Path
import json
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parent
report = json.loads((ROOT / "actuator_qualification_coil.json").read_text())
rows = report["rows"]
assert len(rows) == 260 and report["simulations"] == 200
assert report["simulationErrors"] == 0

plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                     "axes.spines.right": False, "svg.fonttype": "none"})
fig, axes = plt.subplots(2, 3, figsize=(15, 8.4))
fig.subplots_adjust(left=.065, right=.985, top=.885, bottom=.23, wspace=.29, hspace=.38)
fields = [
    ("forceConstant", "Force constant (N/A)", 1),
    ("resistance", "Winding resistance (Ω)", 1),
    ("inductance", "Winding inductance (mH)", 1000),
    ("peakCurrent", "Peak current limit (A)", 1),
    ("voltage", "Voltage limit (V)", 1),
    ("currentResponse", "Current response target (ms)", 1000),
]
colors = {False: "#156fba", True: "#cf6830"}
styles = {"strong_compliant": "-", "both_rigid": "--"}
for ax, (field, label, factor) in zip(axes.ravel(), fields):
    selected = [r for r in rows if r["group"] == "coil_ofat" and r["variedParameter"] == field]
    for mode in styles:
        for stress in colors:
            actual = [r for r in selected if r["mechanics"] == mode and r["stress"] == stress and r["simulated"]]
            actual.sort(key=lambda r: r["variedValue"])
            assert all(r["peakError_N"] is not None for r in actual)
            ax.plot([factor*r["variedValue"] for r in actual], [r["peakError_N"] for r in actual],
                    styles[mode], color=colors[stress], marker="o", markersize=4, linewidth=1.8)
    ax.set_yscale("log")
    ax.set_ylim(.1, 8)
    ax.set_yticks([.1, .2, .5, 1, 2, 5], ["0.1", "0.2", "0.5", "1", "2", "5"])
    ax.minorticks_off()
    ax.axhline(.5, color="#526173", linewidth=1)
    ax.axhspan(.5, 8, facecolor="#c12d3a", alpha=.035)
    ticks = sorted({factor*r["variedValue"] for r in selected})
    rejected = {factor*r["variedValue"] for r in selected if r["failurePhase"] == "preparation"}
    ax.set_xticks(ticks, [f"{v:g}" + ("†" if v in rejected else "") for v in ticks])
    ax.set_xlabel(label)
    ax.set_ylabel("Peak true-force error (N)")
    ax.grid(axis="y", alpha=.16, which="major")
fig.suptitle("Electrical limits and current response: 50 N at 90°", fontsize=17, y=.965)
legend = [Line2D([0], [0], color=colors[False], lw=2, label="Clean force, stroke noise retained"),
          Line2D([0], [0], color=colors[True], lw=2, label="0.02 N force noise + extra 0.2 ms delay"),
          Line2D([0], [0], color="#555", lw=2, label="Strong compliant spring pairs"),
          Line2D([0], [0], color="#555", lw=2, ls="--", label="Both attachments rigid")]
fig.legend(handles=legend, loc="upper center", bbox_to_anchor=(.5,.15), ncol=2, frameon=False)
fig.text(.065, .035, "Matched observer coefficients, fixed design weights; seed 7 in both arms. 1 µm RMS stroke noise.\n"
         "Horizontal line: 0.5 N limit. † No starting equilibrium at that parameter value; no error point is plotted. "
         "This one-factor screen is not full-range winding qualification.", fontsize=9, color="#334155")
for extension in ("png", "svg"):
    fig.savefig(ROOT / f"actuator_qualification_coil_sensitivity.{extension}", dpi=180)
plt.close(fig)
print("Wrote coil sensitivity PNG/SVG from 108 recorded one-factor case rows.")
