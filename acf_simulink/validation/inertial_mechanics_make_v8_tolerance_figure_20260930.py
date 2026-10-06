#!/usr/bin/env python3
"""Plot executed V8 force metrics and analytical 20 mm spring packing."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "inertial_mechanics_v8_20mm_geometry_rescore_20260930.json"
data = json.loads(SOURCE.read_text())
rows = data["rows"]
assert len(rows) == 72 and all(r["new20mmPackingPassed"] for r in rows)
rates = sorted({r["actualStiffness_N_m"] for r in rows})
dampers = sorted({r["actualDamping_Ns_m"] for r in rows})
metrics = np.empty((3, 3))
groups = []
for i, k in enumerate(rates):
    for j, c in enumerate(dampers):
        cohort = [r for r in rows if r["actualStiffness_N_m"] == k and r["actualDamping_Ns_m"] == c]
        assert len(cohort) == 8
        metrics[i, j] = max(r["peakNormalForceError_N"] for r in cohort)
        groups.append((k, c, max(r["feasibleCommonPreloadMin_N"] for r in cohort),
                       min(r["feasibleCommonPreloadMax_N"] for r in cohort)))

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "svg.fonttype": "none",
                     "axes.spines.top": False, "axes.spines.right": False})
fig, (force, packing) = plt.subplots(1, 2, figsize=(12.8, 6),
    gridspec_kw={"width_ratios": [1.25, 1]})
fig.subplots_adjust(left=.09, right=.97, bottom=.23, top=.77, wspace=.55)
fig.suptitle("Spring and damper tolerance with fixed controller coefficients", x=.06, ha="left",
             y=.95, fontsize=17, fontweight="bold")
fig.text(.06, .89, "72 native runs: S = 0.1 / 2 kg; 50 N; 90°; sanding drag μ = 0.3; clean + seeds 7 / 29 / 1729.", fontsize=10)
fig.text(.06, .852, "Whole 4 s recording, including startup and sharp bump. Identified inverse stays at 19.746 N/mm and 60 N·s/m.", fontsize=10)
image = force.imshow(metrics, origin="upper", vmin=0, vmax=.5, cmap="viridis", aspect="auto")
force.set_xticks(range(3), [f"{c:g}" for c in dampers])
force.set_yticks(range(3), [f"{k / 1000:.3f}" for k in rates])
force.set_xlabel("Actual damping (N·s/m)")
force.set_ylabel("Actual combined spring rate (N/mm)")
force.set_title("Worst normal-force error in each 8-case cohort", loc="left", fontsize=11, pad=12)
for i in range(3):
    for j in range(3):
        force.text(j, i, f"{metrics[i, j]:.3f} N", ha="center", va="center", color="white", fontweight="bold")
bar = fig.colorbar(image, ax=force, fraction=.035, pad=.025)
bar.set_ticks([0, .1, .2, .3, .4, .5])
bar.ax.set_ylabel("Error (N); requirement ≤ 0.5 N")

labels = []
for index, (k, c, low, high) in enumerate(groups):
    y = 8 - index
    packing.plot([low, high], [y, y], color="#326b91", linewidth=5, solid_capstyle="round")
    labels.append(f"{k / 1000:.3f} / {c:g}")
packing.axvspan(data["summary"]["commonPreloadIntersectionMin_N"],
               data["summary"]["commonPreloadIntersectionMax_N"], color="#72ad82", alpha=.2)
packing.axvline(33.6, color="#333333", linestyle="--", linewidth=1.4, label="Chosen 33.6 N")
packing.set_yticks(range(8, -1, -1), labels)
packing.set_ylim(-.7, 8.7)
packing.set_xlim(28.5, 42)
packing.set_xlabel("Feasible common preload per spring (N)")
packing.set_ylabel("Actual rate (N/mm) / damping (N·s/m)")
packing.set_title("20 mm spring target: packing from native excursions", loc="left", fontsize=11, pad=12)
packing.grid(axis="x", alpha=.2)
packing.legend(frameon=False, loc="lower right", fontsize=9)
fig.text(.06, .12, "All 72 force histories pass. All 72 fit the approximate 20 mm spring target at 33.6 N preload per spring.", fontsize=10, fontweight="bold")
fig.text(.06, .077, "Packing is an analytical rescore of existing native excursions; it adds zero force simulations. Original 18.1 mm packing rejects remain retained.", fontsize=9)
fig.text(.06, .041, "Combined free length ≤ 40.98 mm; combined installed length ≤ 35.31 mm. These bound spring packing, not the whole flange.", fontsize=9)

stem = ROOT / "inertial_mechanics_v8_actual_tolerance_20260930"
fig.savefig(stem.with_suffix(".svg"), facecolor="white")
fig.savefig(stem.with_suffix(".png"), dpi=150, facecolor="white")
plt.close(fig)
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
manifest = {"sourceRescore": SOURCE.name, "sourceRescoreSHA256": digest(SOURCE),
    "nativeForceExecutions": 72, "newForceExecutionsForPackingOrFigure": 0,
    "nativeModelSHA256": data["sourceModelSHA256"],
    "metric": "max(abs(true normal force - 50 N)) over every 4 s native sample; eight cases per matrix cell",
    "geometry": "approximate 20 mm free length per spring, 33.6 N common preload; analytical packing only",
    "script": Path(__file__).name, "scriptSHA256": digest(Path(__file__)),
    "figures": [{"file": path.name, "sha256": digest(path)} for path in [stem.with_suffix(".svg"), stem.with_suffix(".png")]]}
(ROOT / "inertial_mechanics_v8_tolerance_figure_manifest_20260930.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps(manifest))
