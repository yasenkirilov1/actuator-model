"""Source-backed total-force figures; no local-pressure plots or HTML changes."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
EVIDENCE = ROOT / "acf_simulink/validation/sea_pad_dynamic_20261006"
OUT = ROOT / "reports/figures"
CHECKS = EVIDENCE / "report"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_pad_figure() -> dict:
    paired_path = EVIDENCE / "evaluation/paired_coil_factors.json"
    native_path = EVIDENCE / "native/pilot_replay_results.json"
    paired = json.loads(paired_path.read_text())
    native = json.loads(native_path.read_text())
    assert paired["executed_count"] == len(paired["records"]) == 40
    groups: dict[tuple, list] = {}
    for record in paired["records"]:
        c = record["config"]
        groups.setdefault((c["pad_k"], c["pad_c"], c["pad_diameter"]), []).append(record)
    wanted = [(20000, 80, 0), (20000, 80, .077), (50000, 80, 0),
              (100000, 80, 0), (20000, 0, 0), (20000, 0, .077)]
    labels = ["20 N/mm · point · c=80", "20 N/mm · 77 mm · c=80",
              "50 N/mm · point · c=80", "100 N/mm · point · c=80",
              "20 N/mm · point · c=0", "20 N/mm · 77 mm · c=0"]
    rows = []
    for key, label in zip(wanted, labels):
        records = groups[key]
        assert len(records) == 4
        rows.append({"label": label,
                     "worst_error_N": max(r["metrics"]["force_peak_error"] for r in records),
                     "passes": sum(r["metrics"]["force_pass"] for r in records),
                     "stops": sum(not r["metrics"]["completed"] for r in records),
                     "count": len(records)})
    fig = plt.figure(figsize=(12.2, 5.25), layout="constrained")
    ax, bx = fig.subplots(1, 2, gridspec_kw={"width_ratios": [1.05, 1]})
    y = np.arange(len(rows))
    colors = ["#137e72" if r["passes"] == 4 else "#b75245" for r in rows]
    ax.barh(y, [r["worst_error_N"] for r in rows], color=colors, height=.55)
    ax.set_yticks(y, labels, fontsize=10)
    ax.invert_yaxis()
    ax.set_xscale("log")
    ax.set_xlim(.10, 100)
    ax.axvline(.5, color="#333333", ls="--", lw=1.1)
    ax.text(.52, -.64, "0.5 N limit", fontsize=9, color="#444444")
    for i, r in enumerate(rows):
        ax.text(r["worst_error_N"] * 1.13, i,
                f'{r["worst_error_N"]:.2f} N · {r["passes"]}/4', va="center", fontsize=9)
    ax.set_xlabel("Largest total-force error in four matched records (N)", fontsize=10)
    ax.set_title("Changing the pad with one fixed coil controller", fontsize=12, pad=18)
    ax.grid(axis="x", color="#e1e4e8", lw=.7)
    ax.set_axisbelow(True)

    cases = {r["identity"]: r for r in native["cases"]}
    source_traces = []
    for identity, label, color in [("pilot_sea_point", "Point contact", "#b75245"),
                                    ("pilot_sea_77mm", "77 mm footprint", "#137e72")]:
        r = cases[identity]
        trace_path = Path(r["traceFile"])
        if not trace_path.exists():
            trace_path = trace_path.with_suffix(trace_path.suffix + ".gz")
        a = np.genfromtxt(trace_path, delimiter=",", names=True)
        target = r["config"]["force_target"]
        bx.plot(a["time_s"], a["Fc_axial"]-target, color=color, lw=1.65,
                label=f'{label}: {r["nativeForcePeakError_N"]:.3f} N peak')
        source_traces.append({"path": str(trace_path.relative_to(ROOT)),
                              "sha256": digest(trace_path), "case": identity,
                              "native_all_step_peak_N": r["nativeForcePeakError_N"]})
    bx.axhspan(-.5, .5, color="#137e72", alpha=.12)
    bx.axhline(.5, color="#555555", lw=.7, ls="--")
    bx.axhline(-.5, color="#555555", lw=.7, ls="--")
    bx.set_xlim(.35, 1.8)
    bx.set_xlabel("Time (s)", fontsize=10)
    bx.set_ylabel("Total-force error (N)", fontsize=10)
    bx.set_title("Width effect with the same motor-current command", fontsize=12, pad=18)
    bx.legend(loc="lower right", fontsize=9, frameon=False)
    bx.grid(color="#e1e4e8", lw=.7)
    for x in (ax, bx):
        for spine in ["top", "right"]:
            x.spines[spine].set_visible(False)
    fig.suptitle("Pad damping, stiffness and width have different effects", fontsize=16,
                 fontweight="bold")
    fig.supxlabel("Total axial pressing force only · same 1 mm × 10 mm full-width ridge at 100 mm/s\n"
                  "Left: noisy closed-loop tests; damping c in N·s/m. Right: native held-current plant test, 20 N, S=1.4 kg.",
                  fontsize=10)
    OUT.mkdir(parents=True, exist_ok=True)
    CHECKS.mkdir(parents=True, exist_ok=True)
    path = OUT / "soft-pad-total-force-effects-20261006.svg"
    plt.rcParams["svg.fonttype"] = "none"
    fig.savefig(path, metadata={"Title": "How do soft pads influence our results?",
                                "Description": "Executed total-force experiments; no local pressure metrics."})
    fig.savefig(CHECKS / "soft-pad-total-force-effects-preview.png", dpi=140)
    plt.close(fig)
    return {"figure": str(path.relative_to(ROOT)), "sha256": digest(path),
            "sources": [{"path": str(p.relative_to(ROOT)), "sha256": digest(p)}
                        for p in [paired_path, native_path]],
            "trace_sources": source_traces, "paired_rows": rows,
            "scope": "Total axial force. Fixed-gain comparisons do not establish optimal performance for the stiffer or undamped pads."}


def build_actuator_figure() -> dict:
    groups = [
        ("coil_point20k_c0", "coil_wide20k_c0_width_transfer", "Coil · c=0", "#197b9b"),
        ("coil_point20k_c80", "coil_wide20k_c80_width_transfer", "Coil · c=80", "#197b9b"),
        ("sea_v2_point20k_c0_5k_l10", "sea_v2_wide20k_c0_5k_l10_width_transfer", "Screw + spring · c=0", "#a16a29"),
        ("sea_v2_point20k_c80_5k_l10", "sea_v2_wide20k_c80_5k_l10_width_transfer", "Screw + spring · c=80", "#a16a29"),
    ]
    rows, sources = [], []
    for point_name, wide_name, label, color in groups:
        files = [EVIDENCE / "evaluation" / f"{n}.json" for n in (point_name, wide_name)]
        point, wide = [json.loads(p.read_text()) for p in files]
        assert point["executed_count"] == wide["executed_count"] == 54
        for data in (point, wide):
            assert len(data["records"]) == 54
            assert data["worst_error_N"] == max(r["metrics"]["force_peak_error"] for r in data["records"])
        metrics = [r["metrics"] for r in wide["records"]]
        row = {"label": label, "color": color,
               "point_peak_N": point["worst_error_N"], "wide_peak_N": wide["worst_error_N"],
               "point_passes": point["pass_count"], "wide_passes": wide["pass_count"],
               "point_stops": point["physical_stop_count"], "wide_stops": wide["physical_stop_count"],
               "max_wide_average_copper_loss_W": max(m["copper_loss_average_W"] for m in metrics),
               "max_wide_current_rms_A": max(m["current_rms"] for m in metrics),
               "wide_motor_rms_rating_failures": sum(
                   r["config"]["continuous_current"] > 0
                   and r["metrics"]["current_rms"] > r["config"]["continuous_current"]
                   for r in wide["records"]),
               "count_per_group": 54}
        rows.append(row)
        sources.extend({"path": str(p.relative_to(ROOT)), "sha256": digest(p)} for p in files)

    fig, (ax, bx) = plt.subplots(1, 2, figsize=(12.2, 5.0), layout="constrained",
                               gridspec_kw={"width_ratios": [1.25, 1]})
    y = np.arange(len(rows))
    for i, r in enumerate(rows):
        ax.barh(i-.16, r["point_peak_N"], height=.29, color=r["color"], alpha=.3,
                label="Point contact" if i == 0 else None)
        ax.barh(i+.16, r["wide_peak_N"], height=.29, color=r["color"],
                label="77 mm pad" if i == 0 else None)
        for yy, prefix in [(i-.16, "point"), (i+.16, "wide")]:
            ax.text(r[f"{prefix}_peak_N"]*1.11, yy,
                    f'{r[f"{prefix}_peak_N"]:.3f} N · {r[f"{prefix}_passes"]}/54',
                    va="center", fontsize=9)
        bx.barh(i, r["max_wide_average_copper_loss_W"], height=.58, color=r["color"])
        bx.text(r["max_wide_average_copper_loss_W"]+.75, i-.08,
                f'{r["max_wide_average_copper_loss_W"]:.1f} W', va="center", fontsize=10)
        if r["wide_motor_rms_rating_failures"]:
            bx.text(1.0, i+.19,
                    f'{r["wide_motor_rms_rating_failures"]}/54 exceed motor continuous-current rating',
                    color="#9d2e25", fontsize=9, va="center",
                    bbox={"facecolor": "white", "edgecolor": "none", "alpha": .94, "pad": 2})
    ax.set_yticks(y, [r["label"] for r in rows], fontsize=10)
    bx.set_yticks(y, [r["label"] for r in rows], fontsize=10)
    for a in (ax, bx):
        a.invert_yaxis()
        a.grid(axis="x", color="#e1e4e8", lw=.7)
        a.set_axisbelow(True)
        for spine in ("top", "right"):
            a.spines[spine].set_visible(False)
    ax.set_xscale("log")
    ax.set_xlim(.015, 40)
    ax.axvline(.5, color="#333333", ls="--", lw=1.1)
    ax.text(.52, -.68, "0.5 N limit", fontsize=9)
    ax.set_xlabel("Worst total-force error across 54 records (N)")
    ax.legend(loc="lower right", frameon=False, fontsize=9)
    ax.set_title("Force performance", fontsize=12, pad=18)
    bx.set_xlim(0, 46)
    bx.set_xlabel("Largest 2 s average hot-winding loss (W)")
    bx.set_title("Electrical heating with the 77 mm pad", fontsize=12, pad=18)
    fig.suptitle("A spring-driven screw is a credible wide-pad alternative", fontsize=16, fontweight="bold")
    fig.supxlabel("Whole-pad stiffness 20 N/mm · pad damping c in N·s/m · 32 V usable output\n"
                  "Three tool masses × three force targets × three orientations × two noise seeds per group.\n"
                  "Wide-pad gains frozen from point development; evaluation seeds differ between point and wide cohorts.",
                  fontsize=9)
    path = OUT / "series-elastic-pad-actuator-results-20261006.svg"
    fig.savefig(path, metadata={"Title": "Series-elastic screw and voice-coil total-force results",
                                "Description": "Executed short axial-force diagnostics and hot copper losses; no pressure metrics."})
    fig.savefig(CHECKS / "series-elastic-pad-actuator-results-preview.png", dpi=140)
    plt.close(fig)
    return {"figure": str(path.relative_to(ROOT)), "sha256": digest(path), "sources": sources,
            "rows": rows, "scope": "Short axial total-force diagnostics. Finite nominal grid; no full normal-force, thermal or robot qualification."}


def main() -> None:
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.labelcolor": "#242a31", "text.color": "#242a31",
                         "axes.edgecolor": "#88929c", "svg.fonttype": "none"})
    manifest = {"status": "source_backed_figures_generated", "figures": [build_pad_figure(), build_actuator_figure()]}
    (CHECKS / "figure_manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print(json.dumps({"figures": [f["figure"] for f in manifest["figures"]]}, indent=2))


if __name__ == "__main__":
    main()
