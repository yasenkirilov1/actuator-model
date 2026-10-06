"""Figures for the user's 77 mm x 5 mm pad; source-backed total-force metrics."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
STUDY = ROOT / "acf_simulink/validation/sea_pad_dynamic_20261006"
OUTPUT = ROOT / "reports/figures"
CHECKS = STUDY / "report"


def source(path: Path) -> dict:
    return {"path": str(path.relative_to(ROOT)),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def build_material_figure() -> dict:
    path = STUDY / "evaluation/conservative_material_sensitivity.json"
    data = json.loads(path.read_text())
    rows = []
    fig, ax = plt.subplots(figsize=(10.7, 4.5), layout="constrained")
    for actuator, actor_label, color in [(0, "Voice coil", "#197b9b"),
                                         (1, "Screw + spring", "#a16a29")]:
        for stiffness, style in [(50000, "-"), (100000, "--")]:
            group = []
            for damping in [0, 40, 80, 150]:
                records = [r for r in data["records"]
                           if r["config"]["actuator"] == actuator
                           and r["config"]["pad_k"] == stiffness
                           and r["config"]["pad_c"] == damping]
                assert len(records) == 4
                assert all(r["config"]["pad_diameter"] == .077
                           and r["config"]["pad_thickness"] == .005 for r in records)
                row = {"actuator": actor_label, "stiffness_N_mm": stiffness/1000,
                       "damping_N_s_m": damping, "count": 4,
                       "passes": sum(r["metrics"]["force_pass"] for r in records),
                       "stops": sum(not r["metrics"]["completed"] for r in records),
                       "peak_error_N": max(r["metrics"]["force_peak_error"] for r in records)}
                group.append(row)
                rows.append(row)
            ax.plot([r["damping_N_s_m"] for r in group], [r["peak_error_N"] for r in group],
                    marker="o", color=color, ls=style, lw=1.8,
                    label=f"{actor_label} · {stiffness/1000:g} N/mm")
            for row in group:
                if row["passes"] < 4:
                    ax.annotate(f'{row["passes"]}/4 pass',
                                (row["damping_N_s_m"], row["peak_error_N"]),
                                xytext=(5, 7), textcoords="offset points", fontsize=9, color=color)
    ax.axhline(.5, color="#444444", lw=1, ls=":")
    ax.text(150, .5, "0.5 N limit", ha="right", va="bottom", fontsize=10)
    ax.set_yscale("log")
    ax.set_ylim(.025, 80)
    ax.set_xlim(-5, 175)
    ax.set_xticks([0, 40, 80, 150])
    ax.set_xlabel("Assumed pad material damping (N·s/m)")
    ax.set_ylabel("Worst total-force error in four corner recordings (N)")
    ax.legend(loc="upper right", frameon=False, ncols=2, fontsize=10)
    ax.grid(color="#e1e4e8", which="major", lw=.7)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    fig.suptitle("How do soft pads influence our results?", fontsize=16, fontweight="bold")
    ax.set_title("77 mm diameter × 5 mm thickness · controller gains held fixed", fontsize=11)
    fig.supxlabel("64 executed recordings · same 1 mm × 10 mm ridge at 100 mm/s\n"
                  "50 N/mm: 32/32 pass. 100 N/mm: 23/32 pass; five failed recordings reach a physical stop.\n"
                  "Damping values are sensitivity assumptions, not measured Mirka properties.", fontsize=9)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    CHECKS.mkdir(parents=True, exist_ok=True)
    output = OUTPUT / "pad5mm-material-sensitivity-20261006.svg"
    fig.savefig(output, metadata={"Title": "How do soft pads influence our results?"})
    fig.savefig(CHECKS / "pad5mm-material-sensitivity-preview.png", dpi=135)
    plt.close(fig)
    return {"figure": source(output), "sources": [source(path)], "rows": rows,
            "scope": "Total axial force, 2 s loaded-equilibrium diagnostics; no local pressure metric or hardware qualification."}


def build_actuator_figure() -> dict:
    pairs_path = STUDY / "evaluation/pad5mm_exact_width_pairs.json"
    paired = json.loads(pairs_path.read_text())
    rows = []
    sources = [source(pairs_path)]
    for name, label, color in [("coil_pad5mm_k50_c0", "Voice coil", "#197b9b"),
                                ("sea_pad5mm_k50_c0", "Screw + spring", "#a16a29")]:
        path = STUDY / "evaluation" / f"{name}.json"
        data = json.loads(path.read_text())
        assert data["executed_count"] == data["pass_count"] == 54
        assert data["physical_stop_count"] == 0
        assert all(r["config"]["pad_thickness"] == .005
                   and r["config"]["pad_k"] == 50000
                   and r["config"]["pad_c"] == 0 for r in data["records"])
        pairs = [r for r in paired["pairs"] if r["group"] == name]
        assert len(pairs) == 8
        point_metrics = [r["point_record"]["metrics"] for r in pairs]
        wide_metrics = [r["wide_metrics"] for r in pairs]
        row = {"label": label, "color": color,
               "matched_point_peak_N": max(m["force_peak_error"] for m in point_metrics),
               "matched_wide_peak_N": max(m["force_peak_error"] for m in wide_metrics),
               "matched_point_passes": sum(m["force_pass"] for m in point_metrics),
               "matched_wide_passes": sum(m["force_pass"] for m in wide_metrics),
               "matched_point_stops": sum(not m["completed"] for m in point_metrics),
               "matched_point_sensor_range_failures": sum(not m["sensor_ratings_pass"] for m in point_metrics),
               "full_grid_peak_N": data["worst_error_N"], "full_grid_count": 54,
               "full_grid_peak_copper_loss_W": max(r["metrics"]["copper_loss_average_W"] for r in data["records"]),
               "full_grid_peak_current_rms_A": max(r["metrics"]["current_rms"] for r in data["records"])}
        rows.append(row)
        sources.append(source(path))
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(11.8, 4.7), layout="constrained",
                               gridspec_kw={"width_ratios": [1.25, 1]})
    for i, r in enumerate(rows):
        ax.barh(i, r["full_grid_peak_N"], height=.55, color=r["color"])
        ax.text(r["full_grid_peak_N"]+.015, i,
                f'{r["full_grid_peak_N"]:.3f} N · 54/54 pass', va="center", fontsize=11)
        bx.barh(i, r["full_grid_peak_copper_loss_W"], height=.55, color=r["color"])
        bx.text(r["full_grid_peak_copper_loss_W"]+.8, i,
                f'{r["full_grid_peak_copper_loss_W"]:.1f} W', va="center", fontsize=11)
        bx.text(1.0, i+.38, f'54/54 force passes · full-grid peak {r["full_grid_peak_N"]:.3f} N',
                fontsize=9, va="center")
    for a in (ax, bx):
        a.set_yticks(np.arange(2), [r["label"] for r in rows])
        a.set_ylim(1.55, -.55)
        a.grid(axis="x", color="#e1e4e8", lw=.7)
        a.set_axisbelow(True)
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)
    ax.set_xlim(0, .60)
    ax.axvline(.5, color="#333333", ls="--", lw=1.1)
    ax.text(.5, .96, "0.5 N limit", fontsize=10, ha="center", va="top",
            transform=ax.get_xaxis_transform())
    ax.set_xlabel("Worst full-record total-force error across 54 cases (N)")
    ax.set_title("Force performance with the 77 mm pad", fontsize=12, pad=18)
    bx.set_xlim(0, 46)
    bx.set_xlabel("Largest 2 s average hot-winding loss (W)")
    bx.set_title("Heating across the 54-case wide-pad grids", fontsize=12, pad=18)
    fig.suptitle("77 mm × 5 mm pad: both pass; screw winding loss is lower",
                 fontsize=15, fontweight="bold")
    fig.supxlabel("Assumed whole-pad stiffness 50 N/mm · zero pad damping · 32 V usable output\n"
                  "S=0.1/1.4/2 kg × force=5/20/50 N × orientation=0/45/90° × two fresh noise seeds.\n"
                  "Matched width checks: both 0/8 point vs 8/8 wide. Ineligible narrow-contact histories are not plotted.", fontsize=9)
    output = OUTPUT / "pad5mm-force-and-actuator-results-20261006.svg"
    fig.savefig(output, metadata={"Title": "5 mm interface pad: total-force and actuator comparisons"})
    fig.savefig(CHECKS / "pad5mm-force-and-actuator-results-preview.png", dpi=135)
    plt.close(fig)
    return {"figure": source(output), "sources": sources, "rows": rows,
            "scope": "Axial total-force, loaded equilibrium, robot fixed; modeled winding loss is not measured temperature or thermal qualification."}


def build_mechanics_figure() -> dict:
    template = ROOT / "reports/figures/prototype-mechanics-single-axis-20261006.svg"
    svg = ET.fromstring(template.read_text())
    ns = "{http://www.w3.org/2000/svg}"
    replacements = {
        "Prescribed mechanics and first-prototype sensing": "Prescribed mechanics with the spring-driven screw",
        "Voice coil": "Screw + spring",
        "Internal guide friction": "Independent nut + encoder",
        "25 mm actuator stroke": "Internal spring: 5 N/mm",
        "Correct for surface slope": "Total axial contact force",
        "Normal-force estimate": "Force feedback",
        "Axial B–S force": "Tared B–S force",
        "Force, S acceleration and surface-direction estimates must describe the same instant.":
            "Force and S acceleration must describe the same instant. The pad is 77 mm diameter × 5 mm thick.",
    }
    for element in svg.iter(ns+"text"):
        if element.text in replacements:
            element.text = replacements[element.text]
            if element.text in ["Independent nut + encoder", "Internal spring: 5 N/mm"]:
                element.set("font-size", "11.5")
            if element.text == "Total axial contact force":
                element.set("font-size", "16")
    svg.find(ns+"title").text = "Spring-driven screw prototype and one-axis force reconstruction"
    svg.find(ns+"desc").text = ("Robot R, prescribed undamped20N/mm upper spring, body A, internal series-elastic "
                                "screw, B holder, finite-stiffness cell and S tool with a77mm by5mm pad. "
                                "Tared cell force plus S inertia reconstruct total axial force. "
                                "Unknown surface-normal and drag separation are deferred.")
    for path in svg.iter(ns+"path"):
        if path.get("d") == "M446 230 L446 277 L1132 277 L1132 230":
            path.set("d", "M446 230 L446 277 L1212 277 L1212 170 L1195 170")
    group = svg.find(ns+"g")
    accelerometer = ET.SubElement(group, ns+"text", {"x": "1132.5", "y": "193", "text-anchor": "middle",
                                                    "font-size": "11", "fill": "#24374b"})
    accelerometer.text = "Accelerometer on S"
    ET.SubElement(group, ns+"rect", {"x": "1070", "y": "241", "width": "125", "height": "28",
                                    "rx": "4", "fill": "#ddf3ed", "stroke": "#328b78", "stroke-width": "2"})
    pad = ET.SubElement(group, ns+"text", {"x": "1132.5", "y": "260", "text-anchor": "middle",
                                          "font-size": "13", "fill": "#24374b"})
    pad.text = "Soft pad · 5 mm"
    ET.SubElement(group, ns+"path", {"d": "M1132 226 L1132 241", "stroke": "#50677a", "stroke-width": "2"})
    ET.SubElement(group, ns+"rect", {"x": "1055", "y": "304", "width": "150", "height": "9",
                                    "fill": "#8297ab"})
    workpiece = ET.SubElement(group, ns+"text", {"x": "1045", "y": "312", "text-anchor": "end",
                                               "font-size": "12", "fill": "#526578"})
    workpiece.text = "Workpiece"
    ET.SubElement(group, ns+"path", {"d": "M1132 269 L1132 304", "stroke": "#50677a", "stroke-width": "1.5"})
    output = OUTPUT / "prototype-mechanics-pad5mm-20261006.svg"
    ET.register_namespace("", "http://www.w3.org/2000/svg")
    output.write_text(ET.tostring(svg, encoding="unicode"))
    return {"figure": source(output), "sources": [source(template),
            source(ROOT / "acf_simulink/experiments/sea_pad_dynamic_20261006/pad5mm_contract.json")],
            "scope": "Schematic of load path and measurement placement; not a dimensional spring drawing."}


def main() -> None:
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "text.color": "#242a31", "axes.edgecolor": "#88929c",
                         "svg.fonttype": "none"})
    figures = [build_material_figure(), build_actuator_figure(), build_mechanics_figure()]
    (CHECKS / "pad5mm_figure_manifest.json").write_text(json.dumps({"figures": figures}, indent=2)+"\n")
    print(json.dumps({"figures": [x["figure"]["path"] for x in figures]}))


if __name__ == "__main__":
    main()
