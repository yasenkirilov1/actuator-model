"""Figures for the measured-acceleration flange study, with explicit sources.

Formula figures are calculations, not simulated qualification. Native result
figures are added only when their retained recordings are available.
"""
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT / "reports" / "figures"
VALIDATION = ROOT / "acf_simulink" / "validation"
DEST.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.grid": True, "grid.alpha": .2,
    "figure.facecolor": "white", "axes.facecolor": "white",
    "savefig.facecolor": "white",
})
PALETTE = ["#087f8c", "#d1495b", "#564592", "#d18b20"]
provenance = {}


def save(fig, stem, source):
    fig.savefig(DEST / f"{stem}.png", dpi=180, bbox_inches="tight")
    fig.savefig(DEST / f"{stem}.svg", bbox_inches="tight")
    plt.close(fig)
    provenance[stem] = source


q = np.linspace(0, 1, 1001)
duration = .01 / .1
height = .001 * 64 * q**3 * (1-q)**3
dh = .001 * 192 * q**2 * (1-q)**2 * (1-2*q)
d2h = .001 * (384*q - 2304*q**2 + 3840*q**3 - 1920*q**4)
velocity = dh / duration
acceleration = d2h / duration**2
rows = [{"timeFromEntry_s": float(t), "height_mm": float(h*1000),
         "velocity_m_s": float(v), "acceleration_m_s2": float(a),
         **{f"inertiaDifferenceS{m:g}_N": float(m*a) for m in [.1, 1.4, 2]}}
        for t, h, v, a in zip(q*duration, height, velocity, acceleration)]
(VALIDATION / "inertial_bump_calculation.json").write_text(json.dumps({
    "kind": "analytical_calculation", "height_m": .001, "width_m": .01,
    "scanSpeed_m_s": .1, "formula": "h=64 H q^3(1-q)^3; a=d2h/dq2/T^2",
    "source": "acf_simulink/experiments/plot_inertial_report.py", "rows": rows,
    "qualification": "Required maintained-contact motion; not controller performance.",
}, indent=2) + "\n")

fig, ax = plt.subplots(3, 1, figsize=(9, 7.2), sharex=True, layout="constrained")
ax[0].plot(q*duration*1000, height*1000, color=PALETTE[0], lw=2)
ax[0].set_ylabel("Surface height (mm)")
ax[0].set_title("The 100 ms bump requires up to 2.4 m/s² tool acceleration")
ax[1].plot(q*duration*1000, acceleration, color=PALETTE[1], lw=2)
ax[1].set_ylabel("Acceleration (m/s²)")
for mass, color in zip([.1, 1.4, 2], PALETTE):
    ax[2].plot(q*duration*1000, mass*acceleration, color=color, lw=1.8,
               label=f"S = {mass:g} kg")
ax[2].set_ylabel("S inertial force (N)")
ax[2].set_xlabel("Time from bump entry (ms)")
ax[2].legend(loc="upper center", bbox_to_anchor=(.5, 1.2), ncol=3, frameon=False)
save(fig, "bump_motion_and_inertia", {
    "kind": "analytical_calculation", "file": "acf_simulink/validation/inertial_bump_calculation.json",
    "description": "Motion and downstream inertia calculated from the specified bump polynomial.",
})

gain_file = VALIDATION / "normal_mass_controller_inertial_pi_screen_20260930_results.json"
strategy_file = VALIDATION / "normal_mass_controller_inertial_strategy_screen_20260930_results.json"
if gain_file.is_file() and strategy_file.is_file():
    gains = json.loads(gain_file.read_text())
    strategies = json.loads(strategy_file.read_text())
    assert gains["status"] == "complete" and strategies["status"] == "complete"
    ranked = strategies["controllerRanking"]
    labels = []
    for row in ranked:
        if row["accelerationFeedforwardScale"] == 0:
            label = "PI + static hold" if row["geometryFeedforwardScale"] == 0 else "PI + geometry/friction FF"
        elif row["forceKd_A_s_N"]:
            label = f"PID + all FF (Kd={row['forceKd_A_s_N']:g})"
        elif row["guideFrictionFeedforwardScale"] == 1:
            label = "PI + measured acceleration, geometry, friction FF"
        else:
            label = f"PI + acceleration/geometry; friction FF ×{row['guideFrictionFeedforwardScale']:g}"
        labels.append(label)
    peaks = [row["worstNormalForceError_N"] for row in ranked]
    fig, ax = plt.subplots(figsize=(10, 5.2), layout="constrained")
    bars = ax.barh(np.arange(len(peaks)), peaks,
                   color=[PALETTE[0] if v <= .5 else PALETTE[1] for v in peaks])
    ax.set_yticks(np.arange(len(peaks)), labels)
    ax.invert_yaxis()
    ax.axvline(.5, ls="--", color="#222", label="±0.5 N requirement")
    ax.set_xlim(0, 2.2)
    ax.set_xlabel("Worst true normal-force error (N)")
    ax.set_title("Equal eight-case rigid-reference controller comparison")
    for bar, row in zip(bars, ranked):
        ax.text(bar.get_width()+.025, bar.get_y()+bar.get_height()/2,
                f"{row['worstNormalForceError_N']:.3f} N; {row['passedCases']}/{row['cohortCases']} pass",
                va="center", fontsize=9)
    ax.legend(loc="upper right", frameon=False)
    save(fig, "inertial_controller_reference_comparison", {
        "kind": "native_simulation", "file": str(strategy_file.relative_to(ROOT)),
        "scope": "Rigid attachments; equal clean/three-seed light and heavy endpoint cohorts, not finite-cell qualification.",
    })

    kp_values = sorted({row["forceKp_A_N"] for row in gains["controllerRanking"]})
    ki_values = sorted({row["forceKi_A_Ns"] for row in gains["controllerRanking"]})
    grid = np.full((len(kp_values), len(ki_values)), np.nan)
    for row in gains["controllerRanking"]:
        grid[kp_values.index(row["forceKp_A_N"]), ki_values.index(row["forceKi_A_Ns"])] = row["worstNormalForceError_N"]
    fig, ax = plt.subplots(figsize=(7.4, 4.8), layout="constrained")
    im = ax.imshow(grid, cmap="YlOrRd", vmin=.3, vmax=.5, aspect="auto")
    ax.grid(False)
    ax.set_xticks(np.arange(len(ki_values)), [str(v) for v in ki_values])
    ax.set_yticks(np.arange(len(kp_values)), [str(v) for v in kp_values])
    ax.set_xlabel("Integral gain (A / (N s))")
    ax.set_ylabel("Proportional gain (A/N)")
    ax.set_title("Higher PI gains did not improve the rigid-reference worst case")
    for (y, x), value in np.ndenumerate(grid):
        ax.text(x, y, f"{value:.3f} N", ha="center", va="center",
                color="white" if value > .46 else "#222")
    fig.colorbar(im, ax=ax, label="Worst true normal-force error (N)")
    save(fig, "inertial_PI_reference_gain_search", {
        "kind": "native_simulation", "file": str(gain_file.relative_to(ROOT)),
        "scope": "72 completed native rigid-reference recordings; eight common cases per gain pair.",
    })

stiffness_file = VALIDATION / "inertial_mechanics_stiffness_chart_data_20260930.json"
shaped_stiffness_file = VALIDATION / "inertial_mechanics_structural_cell_rate_pilot_20260930_v7_110M_FF1ms_results.json"
if stiffness_file.is_file():
    stiffness = json.loads(stiffness_file.read_text())
    maxima = stiffness["sixCaseMaxima"]
    controllers = list(dict.fromkeys(row["controller"] for row in maxima))
    fig, ax = plt.subplots(figsize=(9, 5.0), layout="constrained")
    for controller, color in zip(controllers, PALETTE):
        cohort = sorted((row for row in maxima if row["controller"] == controller),
                        key=lambda row: row["stiffness_MN_m"])
        ax.plot([row["stiffness_MN_m"] for row in cohort],
                [row["worstPeakError_N"] for row in cohort], "o-", lw=1.8,
                label=controller, color=color)
    if shaped_stiffness_file.is_file():
        shaped = json.loads(shaped_stiffness_file.read_text())
        assert shaped["status"] == "complete"
        peak = max(row["peakNormalForceError_N"] for row in shaped["rows"])
        ax.scatter([110], [peak], marker="*", s=160, color=PALETTE[2], zorder=4,
                   label="P 0.06 + filtered acceleration FF (110 MN/m only)")
        combined_rows = [dict(row, accelerationFF="unshaped") for row in stiffness["rows"]]
        for row in shaped["rows"]:
            combined_rows.append({
                "id": f"v7-shaped110-case-{row['caseIndex']:04d}",
                "controller": "P 0.06; I 20; feedback damping 1.5; FF filter 1 ms",
                "accelerationFF": "filtered with causal prediction",
                "stiffness_MN_m": row["k1_N_m"] / 1e6,
                "massS_kg": row["massS_kg"], "seed": row["seed"],
                "sensorNoiseEnabled": row["forceNoiseRms_N"] > 0,
                "peakError_N": row["peakNormalForceError_N"],
                "rmsError_N": row["rmsNormalForceError_N"],
                "passed": row["physicalPassed"], "caseIndex": row["caseIndex"],
                "resultFile": shaped_stiffness_file.name,
            })
        structural_view = VALIDATION / "inertial_report_structural_summary.json"
        structural_view.write_text(json.dumps({
            "status": "complete", "kind": "view_of_retained_native_results",
            "newSimulations": 0, "rows": combined_rows,
            "sources": [{"file": str(file.relative_to(ROOT)),
                         "sha256": hashlib.sha256(file.read_bytes()).hexdigest()}
                        for file in [stiffness_file, shaped_stiffness_file]],
            "scope": "72 unshaped stiffness/controller cases and six separate filtered-FF cases at 110 MN/m. No interpolation or claim of a complete filtered stiffness grid.",
        }, indent=2) + "\n")
    ax.axhline(.5, color="#222", ls="--", label="±0.5 N requirement")
    ax.set_xlabel("Axial load-cell stiffness (MN/m)")
    ax.set_ylabel("Worst normal-force error (N)")
    ax.set_title("Stiffness changes the loaded mode and sensor-noise response")
    ax.legend(loc="upper left", fontsize=8, frameon=False)
    ax.set_ylim(0, max(.85, max(row["worstPeakError_N"] for row in maxima)*1.1))
    save(fig, "inertial_cell_stiffness_sensitivity", {
        "kind": "native_simulation", "files": [str(stiffness_file.relative_to(ROOT)),
                                                   str(shaped_stiffness_file.relative_to(ROOT))],
        "scope": "Six common cases per stiffness/controller: finite undamped cell, upper rigid, sharp bump, no preview. Star is a separate six-case filtered-FF study; not a complete retuned stiffness grid.",
    })

drive_file = VALIDATION / "normal_mass_controller_v7_drive_shape_analysis_20260930.json"
if drive_file.is_file():
    drive = json.loads(drive_file.read_text())
    assert drive["status"] == "complete_read_only_analysis"
    traces = drive["traces"]
    labels = {"current100_direct_FF": "100 µs drive\nDirect acceleration FF",
              "current100_FF_LP1ms": "100 µs drive\nFiltered/predicted FF",
              "current50_direct_FF": "50 µs drive\nDirect acceleration FF",
              "current50_predicted_FF": "50 µs drive\nPredicted FF",
              "current50_FF_LP1ms": "50 µs drive\nFiltered/predicted FF"}
    term_names = {"currentTracking_N": "Current tracking",
                  "accelerationFeedforwardError_N": "Acceleration timing",
                  "normalProjectionFeedforward_N": "Geometry",
                  "instantaneousGuideFeedforward_N": "Guide friction",
                  "filteredPIcorrection_N": "PI correction",
                  "finiteCellInertia_N": "Cell-mode inertia"}
    term_colors = ["#d18b20", "#087f8c", "#9ca6af", "#80b3b5", "#564592", "#d1495b"]
    fig, axes = plt.subplots(2, 1, figsize=(10, 8.0), layout="constrained")
    for trace, color in zip(traces[:2], PALETTE):
        native = np.genfromtxt(trace["excerpt"], delimiter=",", names=True)
        take = (native["time_s"] >= 1.9998) & (native["time_s"] <= 2.0025)
        axes[0].plot((native["time_s"][take]-2)*1000, native["normalError_N"][take],
                     color=color, lw=1.5, label=labels[trace["label"]].replace("\n", ": "))
    axes[0].axhline(.5, color="#222", ls="--")
    axes[0].set_xlabel("Time from first bump entry (ms)")
    axes[0].set_ylabel("Normal-force error (N)")
    axes[0].set_title("Filtering suppresses noise, but adds a transient timing cost")
    axes[0].legend(loc="upper right", fontsize=8, frameon=True,
                   facecolor="white", framealpha=.95, edgecolor="none")
    positive, negative = np.zeros(len(traces)), np.zeros(len(traces))
    x = np.arange(len(traces))
    for (term, label), color in zip(term_names.items(), term_colors):
        values = np.array([trace["fullPeak"]["terms_N"][term] for trace in traces])
        base = np.where(values >= 0, positive, negative)
        axes[1].bar(x, values, bottom=base, color=color, label=label)
        positive += np.maximum(values, 0)
        negative += np.minimum(values, 0)
    signed_peaks = [trace["fullPeak"]["signedError_N"] for trace in traces]
    axes[1].scatter(x, signed_peaks, color="#111", marker="D", s=28,
                    label="Actual signed peak", zorder=5)
    for limit in [-.5, .5]:
        axes[1].axhline(limit, color="#222", ls="--", lw=1)
    axes[1].set_xticks(x, [labels[trace["label"]] for trace in traces], fontsize=8)
    axes[1].set_ylabel("Contribution at each trace's peak (N)")
    axes[1].set_title("Faster current response can increase structural noise without clipping")
    axes[1].legend(loc="upper left", bbox_to_anchor=(0, -.2), ncol=3, fontsize=8,
                   frameon=False)
    save(fig, "inertial_performance_limit_decomposition", {
        "kind": "native_trace_analysis", "file": str(drive_file.relative_to(ROOT)),
        "scope": drive["scope"], "forceBalanceResidual_N": max(
            trace["normalForceDecompositionResidual_N"] for trace in traces),
        "qualification": "Diagnostic force-balance decomposition, not extra simulations or equal-seed qualification. All five traces have zero voltage and effort clipping.",
    })

actuator_file = VALIDATION / "inertial_actuators_chart_data_v7.json"
if actuator_file.is_file():
    actuator = json.loads(actuator_file.read_text())
    sizing = actuator["pitchSizing"]["points"]
    lead = [row["lead_mm_rev"] for row in sizing]
    fig, axes = plt.subplots(2, 1, figsize=(8.6, 7.0), sharex=True,
                             layout="constrained")
    axes[0].plot(lead, [row["reflectedRotatingMass_kg"] for row in sizing],
                 "o-", color=PALETTE[2], lw=2)
    axes[0].set_ylabel("Reflected rotating inertia (kg)")
    axes[0].set_title("Screw pitch trades reflected inertia against holding effort")
    for row in sizing:
        axes[0].annotate(f"{row['reflectedRotatingMass_kg']:.2f} kg",
                         (row["lead_mm_rev"], row["reflectedRotatingMass_kg"]),
                         xytext=(7, 5), textcoords="offset points", fontsize=9)
    axes[1].plot(lead, [row["coldHoldingCurrent_A"] for row in sizing], "o-",
                 color=PALETTE[0], label="Ideal 50 N hold")
    axes[1].plot(lead, [row["holdingCurrent90percentSizing_A"] for row in sizing],
                 "o--", color=PALETTE[1], label="90% efficiency sizing bound")
    axes[1].axhline(sizing[0]["catalogContinuousCurrent_A"], color="#222", ls="--",
                    label="RE25 continuous current; torque also limits")
    axes[1].set_ylabel("Holding current (A)")
    axes[1].set_xlabel("Screw lead (mm/revolution)")
    axes[1].legend(frameon=False, fontsize=8)
    save(fig, "screw_pitch_inertia_and_heating", {
        "kind": "analytical_calculation", "file": str(actuator_file.relative_to(ROOT)),
        "rowsPath": ["pitchSizing", "points"],
        "scope": actuator["pitchSizing"]["scope"],
        "qualification": actuator["pitchSizing"]["caveat"],
    })

mechanics_manifest = VALIDATION / "inertial_mechanics_figure_manifest_20260930.json"
if mechanics_manifest.is_file():
    manifest = json.loads(mechanics_manifest.read_text())
    for old_stem, new_stem in [
        ("inertial_mechanics_spring_tuning_20260930", "inertial_mechanics_spring_tuning"),
        ("inertial_mechanics_selected_repeated_20260930", "inertial_mechanics_selected_repeated"),
        ("inertial_mechanics_selected_bump_overlay_20260930", "inertial_mechanics_selected_bump_overlay"),
    ]:
        for suffix in ["png", "svg"]:
            source = VALIDATION / f"{old_stem}.{suffix}"
            recorded = next(row for row in manifest["figures"] if row["file"] == source.name)
            assert hashlib.sha256(source.read_bytes()).hexdigest() == recorded["sha256"]
            shutil.copy2(source, DEST / f"{new_stem}.{suffix}")
        provenance[new_stem] = {
            "kind": "native_simulation", "sourceManifest": str(mechanics_manifest.relative_to(ROOT)),
            "sourceManifestSHA256": hashlib.sha256(mechanics_manifest.read_bytes()).hexdigest(),
            "scope": "Selected lower spring/damper controller search and repeated-bump reference; retained original native recordings, not additional simulations.",
        }

performance_file = VALIDATION / "normal_mass_controller_final_performance_20260930_summary.json"
if performance_file.is_file():
    performance = json.loads(performance_file.read_text())
    assert performance["status"] == "completed_bounded_alternative_drive_comparison"
    groups = performance["configurationGroups"]
    assert len(groups) == 3 and all(row["cases"] == 4 for row in groups)
    response = [row["currentResponse_us"] for row in groups]
    peaks = [row["worstPeakNormalForceError_N"] for row in groups]
    volts = [row["maximumPeakVoltage_V"] for row in groups]
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.7), layout="constrained")
    colors = [PALETTE[2], PALETTE[0], "#929aa4"]
    axes[0].bar(np.arange(3), peaks, width=.62, color=colors)
    axes[0].axhline(.5, color="#222", ls="--", label="±0.5 N requirement")
    for j, peak in enumerate(peaks):
        axes[0].text(j, peak+.012, f"{peak:.3f} N", ha="center")
    axes[0].set_ylim(0, .57)
    axes[0].set_ylabel("Worst true normal-force error (N)")
    axes[0].set_xticks(np.arange(3), [f"{v} µs" + ("\nselected" if v == 100 else "") for v in response])
    axes[0].set_title("A faster drive yields a modest transient benefit")
    axes[0].legend(loc="upper right", frameon=False, fontsize=8)
    axes[1].bar(np.arange(3), volts, width=.62, color=colors)
    axes[1].axhline(24, color=PALETTE[1], ls="--", label="24 V available")
    axes[1].axhline(36, color="#333", ls=":", label="36 V available")
    for j, voltage in enumerate(volts):
        axes[1].text(j, voltage+.7, f"{voltage:.2f} V", ha="center")
    axes[1].set_ylim(0, 40)
    axes[1].set_ylabel("Peak drive voltage in 36 V records (V)")
    axes[1].set_xticks(np.arange(3), [f"{v} µs" for v in response])
    axes[1].set_title("50 µs needs more voltage and bandwidth")
    axes[1].legend(loc="upper right", frameon=False, fontsize=8)
    save(fig, "inertial_final_current_response", {
        "kind": "native_simulation", "file": str(performance_file.relative_to(ROOT)),
        "sha256": hashlib.sha256(performance_file.read_bytes()).hexdigest(),
        "scope": performance["scope"],
        "rows": 12, "nativeSamplesScored": performance["comparisonNativeSamplesScored"],
        "springMetadataScope": performance["springMetadataScope"],
        "qualification": "100µs is the final grid target.50µs and150µs are bounded alternatives; model bandwidth is not measured driver capability.",
    })

# Copy reviewed worker figures without changing their underlying calculations.
for old_stem, new_stem, evidence_name, evidence_kind, scope in [
    ("inertial_mechanics_v8_actual_tolerance_20260930", "inertial_mechanics_v8_actual_tolerance",
     "inertial_mechanics_v8_20mm_geometry_rescore_20260930.json", "native_simulation_and_analytical_geometry_rescore",
     "72 actual-k/c native recordings with fixed nominal controller;20mm packing analytically rescored, no new force runs."),
    ("inertial_uncertainty_v8_summary_sensing", "inertial_uncertainty_v8_sensing",
     "inertial_uncertainty_v8_summary.json", "native_simulation",
     "42 final-acquisition sensing OAT native records; separate from joint allowance."),
    ("inertial_uncertainty_v8_summary_joint", "inertial_uncertainty_v8_joint",
     "inertial_uncertainty_v8_summary.json", "native_simulation",
     "60 final-acquisition26-dimensional deterministic corners at two endpoints, three seeds; not exhaustive error box."),
    ("normal_voltage_20260930_final_sensor_headroom", "inertial_final_voltage_headroom",
     "normal_voltage_20260930_final_sensor_summary.json", "native_simulation_and_analytical_heat",
     "16 final-source native electrical records/15distinct conditions; steady linear-stage heat is an analytical calculation."),
]:
    evidence = VALIDATION / evidence_name
    if evidence.is_file() and (VALIDATION / f"{old_stem}.png").is_file():
        for suffix in ["png", "svg"]:
            source = VALIDATION / f"{old_stem}.{suffix}"
            shutil.copy2(source, DEST / f"{new_stem}.{suffix}")
        provenance[new_stem] = {
            "kind": evidence_kind, "file": str(evidence.relative_to(ROOT)),
            "sha256": hashlib.sha256(evidence.read_bytes()).hexdigest(), "scope": scope,
            "copiedFigureSHA256": hashlib.sha256((DEST / f"{new_stem}.png").read_bytes()).hexdigest(),
            "workerFigure": str((VALIDATION / f"{old_stem}.png").relative_to(ROOT)),
        }

(VALIDATION / "inertial_report_figure_sources.json").write_text(
    json.dumps(provenance, indent=2) + "\n")
print(json.dumps({"figures": list(provenance), "destination": str(DEST)}))
