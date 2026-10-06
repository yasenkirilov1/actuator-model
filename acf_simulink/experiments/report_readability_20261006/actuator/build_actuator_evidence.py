"""Build source-bound actuator figures; never simulate or modify the plant.

Run with: uv run --with numpy --with scipy --with matplotlib python <this file>
The user explicitly requested SVG report figures. Native histories determine
all experimental metrics. Display reduction keeps per-bin minima and maxima;
the native sample counts and full-record peaks are checked before plotting.
"""
from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "acf_simulink/validation/report_readability_20261006/actuator"
BLUE, TEAL, ORANGE, INK, GRAY = "#2563eb", "#008b83", "#c85a22", "#25364b", "#617184"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 11,
                    "svg.fonttype": "none", "axes.spines.top": False,
                    "axes.spines.right": False, "axes.labelcolor": INK,
                    "text.color": INK, "xtick.color": INK, "ytick.color": INK,
                    "axes.edgecolor": "#b2bac4", "savefig.facecolor": "white"})
SOURCES: dict[str, str] = {}


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def bind(path: Path, expected: str | None = None) -> str:
    digest = sha(path)
    if expected is not None:
        assert digest == expected, str(path)
    SOURCES[str(path.relative_to(ROOT))] = digest
    return digest


def read_json(relative: str) -> dict:
    path = ROOT / relative
    bind(path)
    return json.loads(path.read_text())


def write_json(name: str, value: dict) -> None:
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def write_csv(name: str, rows: list[dict]) -> None:
    with (OUT / name).open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def extrema(t: np.ndarray, y: np.ndarray, bins: int = 1500) -> tuple[np.ndarray, np.ndarray]:
    """Keep minimum and maximum in each contiguous bin, in time order."""
    width = max(1, int(np.ceil(len(t) / bins)))
    chosen = {0, len(t) - 1}
    for first in range(0, len(t), width):
        segment = y[first:first + width]
        chosen.add(first + int(np.argmin(segment)))
        chosen.add(first + int(np.argmax(segment)))
    idx = np.asarray(sorted(chosen))
    return t[idx], y[idx]


def save_figure(fig: plt.Figure, stem: str) -> dict:
    paths = []
    for extension in ("svg", "png"):
        path = OUT / f"{stem}.{extension}"
        fig.savefig(path, dpi=150, bbox_inches="tight")
        paths.append({"path": str(path.relative_to(ROOT)), "sha256": sha(path)})
    plt.close(fig)
    return {"id": stem, "artifacts": paths, "visuallyReviewed": False}


def bump_demand() -> tuple[dict, dict]:
    H, W, scan = .001, .01, .1
    T = W / scan
    q = np.linspace(0, 1, 10001)
    coefficients = np.array([-64., 192., -192., 64., 0., 0., 0.])
    h = H * np.polyval(coefficients, q)
    v = H / T * np.polyval(np.polyder(coefficients), q)
    a = H / T**2 * np.polyval(np.polyder(coefficients, 2), q)
    j = H / T**3 * np.polyval(np.polyder(coefficients, 3), q)
    def exact_extrema(order: int) -> tuple[float, float, float]:
        polynomial = np.polyder(coefficients, order)
        roots = np.roots(np.polyder(polynomial))
        points = [0., 1.] + [float(r.real) for r in roots if abs(r.imag) < 1e-12 and 0 < r.real < 1]
        values = H / T**order * np.polyval(polynomial, points)
        i = int(np.argmax(np.abs(values)))
        return float(values.min()), float(values.max()), float(abs(values[i]))
    amin, amax, aabs = exact_extrema(2)
    vmin, vmax, vabs = exact_extrema(1)
    _, _, jabs = exact_extrema(3)
    assert abs(aabs - 2.4) < 1e-12 and abs(vabs - .0343460041344) < 1e-12
    native = ROOT / "acf_simulink/validation/upper_no_damping_20261005/qualification_v8/native/native_parameter_bank.c"
    bind(native)
    first = native.read_text()[:50000]
    for s in (".Bumpcomponent1_Coefs = {-64,192,-192,64,0,0,0}",
              ".Bumpcomponent2_Coefs = {-384,960,-768,192,0,0}",
              ".Bumpcomponent3_Coefs = {-1920,3840,-2304,384,0}"):
        assert s in first
    rows = [{"timeAfterBumpStart_ms": float(t * 1000), "height_mm": float(x * 1000),
             "toolVelocity_mm_s": float(vv * 1000), "toolAcceleration_m_s2": float(aa),
             "toolJerk_m_s3": float(jj)} for t, x, vv, aa, jj in zip(q*T, h, v, a, j)]
    write_csv("sharp_bump_kinematics.csv", rows)
    fig, axes = plt.subplots(3, 1, figsize=(10.8, 7), sharex=True)
    for ax, values, label, limits in zip(axes, (h*1000, v*1000, a),
                                       ("Height (mm)", "Tool velocity (mm/s)", "Tool acceleration (m/s²)"),
                                       ((-.05, 1.2), (-41, 41), (-2.8, 2.5))):
        ax.plot(q*T*1000, values, color=BLUE, linewidth=2)
        ax.axhline(0, color="#b2bac4", linewidth=.8)
        ax.set(ylabel=label, ylim=limits, xlim=(0, 100))
        ax.grid(axis="y", alpha=.15)
    axes[0].set_title("What does the 1 mm sharp bump ask the tool to do?", loc="left", fontweight="bold", pad=15)
    axes[0].text(51, .94, "1 mm height", fontsize=11)
    axes[1].text(4, -33, f"Peak speed: {vabs*1000:.1f} mm/s", fontsize=11)
    axes[2].text(4, -2.55, "Peak acceleration: 2.4 m/s²", fontsize=11)
    axes[2].set_xlabel("Time after entering bump (ms)")
    fig.subplots_adjust(hspace=.17, bottom=.16, left=.13, right=.96)
    fig.text(.13, .06, "Calculated from the exact native C² profile: 10 mm wide at 100 mm/s.\n"
             "This is tool motion; actuator travel also includes motion of A and robot recentering.", fontsize=10, color=GRAY)
    figure = save_figure(fig, "sharp-bump-motion-demand")
    return {"evidenceType": "analytical_exact_native_profile", "height_m": H, "width_m": W,
            "scanSpeed_m_s": scan, "bumpDuration_s": T, "polynomialDescending": coefficients.tolist(),
            "equation": "h/H = 64 q^3 (1-q)^3; q=(t-t0)/(W/scanSpeed)",
            "peakToolSpeed_m_s": vabs, "minimumToolAcceleration_m_s2": amin,
            "maximumToolAcceleration_m_s2": amax, "peakToolAcceleration_m_s2": aabs,
            "peakToolJerk_m_s3": jabs, "toolOnlyInertiaAtS2kg_N": 2*aabs,
            "holderPlusToolInertiaIfCoMovingB06S2_N": 2.6*aabs,
            "limits": "Tool motion under maintained contact, not a complete force controller or bound on relative actuator motion; upper A is mobile. No simulation was added."}, figure


def screw_tradeoff(bump: dict) -> tuple[dict, dict]:
    provision = read_json("acf_simulink/validation/inertial_actuators_provision.json")
    parameters = read_json("acf_simulink/validation/inertial_actuators_selected_screw_carrier_inverse_v1_parameters.json")
    screw = parameters["screw"]
    J, Kt, eta = screw["rotorInertia"], screw["torqueConstant"], screw["sizingEfficiency"]
    assert np.isclose(J, 1.58e-6) and Kt == .0234 and eta == .9
    rows = []
    for lead in (.003, .005, .01):
        ell = lead / (2*np.pi)
        mass = J / ell**2
        holding_torque = 50 * ell
        alpha = bump["peakToolAcceleration_m_s2"] / ell
        omega = bump["peakToolSpeed_m_s"] / ell
        inertia_torque = J * alpha
        rows.append({"lead_mm_rev": lead*1000, "reflectedRotaryMass_kg": mass,
                     "idealForceConstant_N_A": Kt/ell, "ideal50NHoldingCurrent_A": holding_torque/Kt,
                     "holdingCurrent90percentEfficiency_A": holding_torque/(eta*Kt),
                     "catalogContinuousCurrent_A": 1.16, "catalogContinuousTorque_mNm": 26.3,
                     "catalogTorque50N90percentPassBeforeGuideLoss": bool(.0263*eta/ell >= 50),
                     "idealCold50NCopperLoss_W": (holding_torque/Kt)**2 * screw["resistance"],
                     "profileRotorSpeed_rpm": omega*60/(2*np.pi),
                     "profileRotorAcceleration_rad_s2": alpha,
                     "rotaryInertiaPeakTorque_mNm": inertia_torque*1000,
                     "rotaryInertiaPeakCurrent_A": inertia_torque/Kt,
                     "rotaryInertiaEquivalentAxialForce_N": mass*bump["peakToolAcceleration_m_s2"]})
    for row, old in zip(rows, provision["pitchSizing"]):
        assert abs(row["reflectedRotaryMass_kg"] - old["reflectedRotatingMass_kg"]) < 1e-12
        assert abs(row["ideal50NHoldingCurrent_A"] - old["coldHoldingCurrent_A"]) < 1e-12
    write_csv("screw_pitch_tradeoff.csv", rows)
    fig, axes = plt.subplots(1, 2, figsize=(10.8, 5.4))
    x = np.arange(3)
    masses = [r["reflectedRotaryMass_kg"] for r in rows]
    currents = [r["ideal50NHoldingCurrent_A"] for r in rows]
    axes[0].bar(x, masses, width=.56, color=BLUE)
    axes[1].bar(x, currents, width=.56, color=TEAL)
    for ax, vals, unit in zip(axes, (masses, currents), ("kg", "A")):
        ax.set_xticks(x, ["3 mm/rev", "5 mm/rev", "10 mm/rev"])
        ax.set_ylim(0, max(vals)*1.22)
        ax.grid(axis="y", alpha=.15)
        ax.set_axisbelow(True)
        for i, val in enumerate(vals):
            offset = .3 if unit == "A" and i == 0 else .035*ax.get_ylim()[1]
            ax.text(i, val+offset, f"{val:.2f} {unit}", ha="center", fontweight="bold")
    axes[0].set(ylabel="Equivalent rotating mass (kg)")
    axes[0].set_title("A finer screw makes rapid motion harder", loc="left", fontsize=12, pad=12)
    axes[1].set(ylabel="Current to hold 50 N (A)")
    axes[1].axhline(1.16, color=ORANGE, linestyle="--", linewidth=1.5)
    axes[1].text(.03, .95, "RE25 continuous limit: 1.16 A", color=ORANGE, fontsize=10,
                 transform=axes[1].transAxes, va="top")
    axes[1].set_title("A coarser screw needs more holding current", loc="left", fontsize=12, pad=12)
    fig.suptitle("How does screw pitch trade holding current against inertia?", x=.1, ha="left", fontsize=15, fontweight="bold")
    fig.subplots_adjust(left=.1, right=.97, top=.79, bottom=.23, wspace=.3)
    fig.text(.1, .07, "Analytical RE25 example: motor + assumed screw/coupler inertia 1.58×10⁻⁶ kg·m².\n"
             "Ideal holding-current calculation excludes guide losses; 90% efficiency adds 11%.\n"
             "The extra 'mass' is rotary inertia expressed as the force needed to accelerate the tool.", fontsize=10, color=GRAY)
    figure = save_figure(fig, "leadscrew-pitch-inertia-current")
    return {"evidenceType": "analytical_sizing_bound_separate_from_native_transmission", "rows": rows,
            "motor": provision["motor"], "rotorInertia_kg_m2": screw["motorRotorInertia_kg_m2"],
            "assumedScrewCouplerInertia_kg_m2": screw["screwCouplerInertia_kg_m2"],
            "formulas": {"ell": "lead/(2*pi)", "reflectedMass": "J/ell^2",
                         "idealHoldingCurrent": "F*ell/Kt", "90percentHoldingCurrent": "F*ell/(0.9*Kt)",
                         "rotorAcceleration": "a/ell", "rotaryInertiaTorque": "J*a/ell"},
            "kinematicSizingAssumption": "Motor body stationary, moving assembly follows the tool profile. This assumption is not the current mobile-A controller simulation.",
            "nativeTransmission": "Ideal reversible pitch conversion plus explicit Coulomb/viscous torque; no scalar efficiency multiplier is inserted into its force balance.",
            "limitations": ["90% efficiency is engineering sizing, not a measured transmission efficiency.",
                            "The assumed screw/coupler inertia is not a purchased part specification.",
                            "3 mm lead is near but below 50 N continuous torque at 90% efficiency (49.57 N before guide friction).",
                            "A larger motor or different winding/screw remains a valid alternative; this does not show all leadscrews fail."],
            "largerMotorStaticCounterexample": provision["largerMotorStaticReference"]}, figure


def historical_comparison() -> tuple[dict, dict]:
    data = read_json("acf_simulink/validation/inertial_actuators_final_actual_acquisition_v8_20260930_reviewed_comparison.json")
    bind(Path(data["reportFile"]), data["reportSHA256"])
    bind(Path(data["replayFile"]), data["replaySHA256"])
    pairs = data["pairs"]
    for row in data["rows"][:8]:
        path = Path(row["nativeTraceFile"])
        bind(path)
        x = loadmat(path, simplify_cells=True)
        r = x["r"]
        native = r["normalContact"]["data"]
        names = list(r["normalContact"]["names"])
        # Read semantic channel names; do not assume the generic axial Fc signal.
        assert "normalForce_N" in names
        idx = names.index("normalForce_N")
        error = native[:, idx] - row["referenceForce_N"]
        assert len(error) == row["nativeSamples"]
        assert abs(float(np.max(np.abs(error))) - row["peakNormalForceError_N"]) < 1e-10
        assert r["parameters"]["mechanics"]["upperRigid"] == 1
        assert r["parameters"]["mechanics"]["mA"] == .4
        assert r["parameters"]["mechanics"]["k1"] == 19746
        assert r["parameters"]["mechanics"]["c1"] == 60
    rows = [{"condition": f"S={p['massS_kg']:g} kg, F={p['referenceForce_N']:g} N, {p['angle_deg']:g}°, "
                          + ("noise off" if p['seedRequested']==0 else "one noisy seed"),
             "coilPeakNormalError_N": p["coilPeakNormalForceError_N"],
             "screwPeakNormalError_N": p["screwPeakNormalForceError_N"],
             "coilPeakCurrent_A": p["coilPeakCurrent_A"], "screwPeakCurrent_A": p["screwPeakCurrent_A"],
             "coilPassed": p["coilPassed"], "screwPassed": p["screwPassed"],
             "screwContinuousStaticCatalogPassed": p["screwContinuousStaticCatalogPassed"]} for p in pairs]
    write_csv("historical_matched_actuator_results.csv", rows)
    fig, ax = plt.subplots(figsize=(10.8, 5.6))
    y = np.arange(4)
    ax.barh(y-.17, [p["coilPeakNormalForceError_N"] for p in pairs], height=.3, color=BLUE, label="Voice coil")
    ax.barh(y+.17, [p["screwPeakNormalForceError_N"] for p in pairs], height=.3,
            color=TEAL, hatch="///", edgecolor="white", linewidth=.4,
            label="10 mm-lead RE25 screw")
    for i, pair in enumerate(pairs):
        for delta, field in ((-.17, "coilPeakNormalForceError_N"), (.17, "screwPeakNormalForceError_N")):
            value = pair[field]; ax.text(value+.007, i+delta, f"{value:.3f}", va="center", fontsize=10)
    ax.set_yticks(y, ["2 kg, 50 N, 90°\nnoise off", "2 kg, 50 N, 90°\nnoise on",
                         "2 kg, 5 N, 0°\nnoise on", "0.1 kg, 50 N, 90°\nnoise on"])
    ax.invert_yaxis()
    ax.axvline(.5, color=ORANGE, linestyle="--", linewidth=1.5)
    ax.text(.498, -.61, "0.5 N limit", ha="right", color=ORANGE, fontsize=10)
    ax.set(xlim=(0, .54), xlabel="Worst normal-force error over the entire 4 s recording (N)")
    ax.set_title("Can a leadscrew work? These earlier transient tests say yes", loc="left", fontweight="bold", pad=15)
    ax.legend(frameon=False, loc="lower right", fontsize=10)
    ax.grid(axis="x", alpha=.15); ax.set_axisbelow(True)
    fig.subplots_adjust(left=.2, right=.95, top=.83, bottom=.27)
    fig.text(.2, .07, "Historical mechanics: rigid upper, 19.746 N/mm lower pair + 60 N·s/m damper, A = 0.4 kg.\n"
             "Eight matched native tests, one screw refinement: all transient passes. The RE25\n"
             "exceeded its continuous holding-current rating in all four conditions.\n"
             "These results do not qualify the current undamped-upper / rigid-lower arrangement.", fontsize=10, color=GRAY)
    figure = save_figure(fig, "historical-voicecoil-leadscrew-errors")
    return {"evidenceType": "historical_matched_native_experiments_different_mechanics", "rows": rows,
            "recordingDuration_s": 4, "nativeBaseRecordings": 8, "nativeRefinements": 1,
            "passedTransientRecordings": 9, "fullHistoriesRechecked": 8,
            "mechanics": {"upperRigid": True, "lowerStiffness_N_m": 19746,
                          "lowerDamping_N_s_m": 60, "A_kg": .4, "B_kg": .6},
            "catalogContinuousCurrent_A": 1.16, "50NStaticCurrent_A": 3.400746647262721,
            "50NColdCopperLoss_W": 26.83098040057524, "screwPeakCurrentLimit_A": 5,
            "currentLoopResponseTarget_s": .0001,
            "sourceModelSHA256": data["sourceModelSHA256"], "candidateSHA256": data["candidateSHA256"],
            "scope": "A selected matched controller for each actuator, four conditions and one noisy seed; not all actuators, fixed mechanics, full-grid or installed thermal qualification."}, figure


def voltage_pairs() -> tuple[dict, dict]:
    data = read_json("acf_simulink/validation/upper_no_damping_20261005/report/fixed_voltage_diagnostics.json")
    for rel, digest in data["sourceHashes"].items(): bind(ROOT / rel, digest)
    rows, records = [], []
    for row in data["rows"]:
        path = Path(row["historyFile"])
        bind(path, row["historySha256"])
        record = loadmat(path, simplify_cells=True)
        t, truth, p = record["nativeTime"], record["truth"], record["p"]
        assert len(t) == int(row["sampleCount"]) and len(truth) == len(t)
        assert np.max(np.abs(truth[:, 0]-p["force"]["reference"])) == row["normalForcePeakError_N"]
        assert np.max(np.abs(truth[:, 14])) == row["peakCurrent_A"]
        assert np.max(np.abs(truth[:, 15])) == row["peakVoltage_V"]
        assert int(np.sum(truth[:, 23]>.5)) == row["voltageClipSamples"]
        assert p["mechanics"]["mA"] == 1.5 and p["mechanics"]["k2"] == 20000
        assert p["mechanics"]["k1"] == 64e6 and p["mechanics"]["c1"] == 0 and p["mechanics"]["c2"] == 0
        assert p["coil"]["currentResponse"] == .00005
        records.append((t, truth, p))
        row = dict(row)
        row["coilForcePeak_N"] = float(np.abs(truth[:,14]).max()*p["coil"]["forceConstant"])
        row["coilRmsCurrent_A"] = float(np.sqrt(np.mean(truth[:,14]**2)))
        row["coilAverageCopperLoss_W"] = row["coilRmsCurrent_A"]**2*p["coil"]["resistance"]
        row["voltageClipRecordedTime_s"] = row["voltageClipSamples"]*float(np.median(np.diff(t)))
        rows.append(row)
    assert np.array_equal(records[0][0], records[1][0])
    assert np.array_equal(records[0][1], records[1][1])
    assert np.array_equal(records[2][0], records[3][0])
    write_csv("fixed_voltage_pair_metrics.csv", [
        {k:r[k] for k in ("stage", "scenario", "usableVoltage_V", "normalForcePeakError_N",
                          "peakCurrent_A", "peakVoltage_V", "voltageClipSamples", "sampleCount",
                          "coilRmsCurrent_A", "coilAverageCopperLoss_W")} for r in rows])
    fig, axes = plt.subplots(3, 1, figsize=(10.8, 8.2), sharex=True)
    display_rows = []
    for (t, truth, p), row, color, style in zip(records[2:], rows[2:], (BLUE, TEAL), ("-", "--")):
        window = (t >= 4.91) & (t <= 5.12)
        values = (truth[:,0]-p["force"]["reference"], truth[:,14], truth[:,15])
        for ax, values_, signal, unit in zip(axes, values, ("normalForceError", "coilCurrent", "driverOutputVoltage"), ("N", "A", "V")):
            tx, yy = extrema(t[window], values_[window], 1800)
            ax.plot(tx, yy, color=color, linestyle=style, linewidth=1.25, label=f"{row['usableVoltage_V']:g} V usable")
            display_rows.extend({"supplyUsable_V": row["usableVoltage_V"], "signal": signal,
                                 "unit": unit, "time_s": float(tt), "value": float(v)} for tt,v in zip(tx, yy))
            ax.grid(axis="y", alpha=.15)
    axes[0].set(ylabel="Normal-force error (N)", ylim=(-3.0, 3.5))
    axes[0].axhline(.5, color=ORANGE, linestyle=":", linewidth=1.2)
    axes[0].axhline(-.5, color=ORANGE, linestyle=":", linewidth=1.2)
    axes[0].set_title("Does more voltage fix the clipped stress case? It helps, but both runs fail", loc="left", fontsize=13, fontweight="bold", pad=15)
    axes[0].legend(frameon=False, loc="upper left", ncol=2, fontsize=10)
    axes[0].text(4.915, -2.63, "Full 6 s peaks: 2.748 N → 0.650 N; target ±0.5 N", fontsize=11)
    axes[1].set(ylabel="Coil current (A)", ylim=(1.6, 2.55))
    axes[2].set(ylabel="Driver output (V)", ylim=(0, 39), xlabel="Native recording time (s)", xlim=(4.91, 5.12))
    axes[2].axhline(32, color=BLUE, linestyle=":", linewidth=1.2)
    axes[2].text(5.08, 34.6, "32 V rail", color=BLUE, fontsize=10)
    peak48 = int(np.argmax(records[3][1][:, 15]))
    axes[2].annotate("48 V run reaches 33.02 V\nwithout clipping",
                     xy=(records[3][0][peak48], records[3][1][peak48, 15]), xytext=(4.915, 36.1),
                     arrowprops={"arrowstyle": "-", "color": GRAY}, fontsize=10, color=TEAL)
    fig.subplots_adjust(left=.13, right=.96, top=.89, bottom=.23, hspace=.22)
    fig.text(.13, .045, "Current response, controller and sensor chain unchanged; fixed prescribed mechanics, S = 2 kg, F = 50 N.\n"
             "32 V clips at 2,386 native points across the full run; 48 V clips at none. Display retains extrema.\n"
             "A separate unclipped nominal pair is bitwise identical: 0.459 N peak and 23.76 V demand at either rail.\n"
             "These four runs are diagnostics, outside the 736-case qualification and controller selection.", fontsize=10, color=GRAY)
    write_csv("fixed_voltage_trace_display.csv", display_rows)
    figure = save_figure(fig, "fixed-mechanics-voltage-headroom")
    return {"evidenceType": "paired_independent_matlab_counterfactual_diagnostics", "rows": rows,
            "qualificationEpisodes": 0, "newRuns": 0, "controllerRetuned": False,
            "identicalUnclippedNativeArrays": True, "matchedStressTimeArrays": True,
            "actualAndIdentifiedCurrentResponse_s": .00005,
            "displayReduction": "All metrics use every native point; plotted traces retain minima/maxima in each contiguous bin within4.91–5.12s.",
            "scope": data["scope"]}, figure


def coil_targets() -> dict:
    examples = read_json("acf_simulink/validation/realistic_voice_coil_examples.json")
    drawing = OUT / "sources/moticont-lvcm-070-051-01-20261006.pdf"
    bind(drawing, examples["sources"]["moticont_drawing"]["download_sha256"])
    Kf, Rc, Rh, L = 24.4, 6.4, 8.896, .004
    rows = []
    for force in (5, 50, 55, 60, 85):
        current = force/Kf
        rows.append({"force_N": force, "current_A": current,
                     "copperLoss25C_W": current**2*Rc, "copperLossHotModel_W": current**2*Rh,
                     "resistiveVoltageHotModel_V": current*Rh,
                     "headroomAt32VIgnoringEmfSlew_V": 32-current*Rh})
    write_csv("coil_static_current_heating.csv", rows)
    targets = [
        {"parameter": "Useful actuator stroke", "target": "25 mm, with controller kept near its central15 mm band",
         "reason": "Travel allowance includes bump, upper-body motion and delayed recentering; stroke alone does not establish recentering success."},
        {"parameter": "Continuous force", "target": "At least55–60 N at hot operating temperature and every used stroke position",
         "reason": "50 N pressing plus reserve; continuous thermal rating is separate from catalog peak force and must cover the actual mounting/cooling."},
        {"parameter": "Peak force at our driver current", "target": "Approximately85 N at a3.5 A bipolar current limit",
         "reason": "Kf24.4×3.5=85.4 N; the manufacturer's204.3 N intermittent rating requires8.37 A and is not available from this driver."},
        {"parameter": "Force constant Kf", "target": "Approximately24.4–25 N/A or an equally capable current/thermal combination",
         "reason": "50 N requires2.05 A at24.4 N/A. Kf alone is not efficiency; inspect Kf²/R and force variation across position and temperature."},
        {"parameter": "Motor constant", "target": "Prefer about10 N/√W or better at operating temperature",
         "reason": "Copper loss is(F/Km)². The studied reference is9.64 cold and8.18 in the hot-resistance model; hot50 N already needs37.36 W."},
        {"parameter": "Winding resistance and inductance", "target": "Start with R25C≈6.4 Ω, Rhot≈8.9 Ω, L≈4 mH; lower loss/slew demand is desirable",
         "reason": "These are reference values, not universal minimums. Driver voltage must coverIR + L·di/dt + back EMF + losses."},
        {"parameter": "Moving mass", "target": "Prefer roughly0.24 kg actuator moving assembly inside the total B=0.6 kg budget",
         "reason": "The reference leaves0.36 kg for holder, guides/cables and hardware. Extra moving mass changes control and cell response."},
        {"parameter": "Stationary mass", "target": "Account for≈1.24 kg reference body plus mount inside A; current tests use A=1.5 kg",
         "reason": "Stationary coil/magnet mass still moves on the prescribed upper spring; it cannot be omitted from A."},
        {"parameter": "Guidance and force smoothness", "target": "Low drag/cable force, negligible transmission backlash, identified force-position map",
         "reason": "A bare voice coil needs external guidance; direct drive removes screw backlash but not guide stiction or mounting modes."},
        {"parameter": "Driver usable output", "target": "Adjustable48 V-rated bipolar driver; aim for≥36 V usable development headroom",
         "reason": "The stress48 V diagnostic needed33.02 V and still failed.48 V rating is component capability;36 V usable is a design target, not a newly qualified controller."},
        {"parameter": "Driver current control", "target": "Bipolar3.5 A and separately measured50 µs current-response target",
         "reason": "The model's first-order response target is not the winding's passiveL/R response or a guarantee from peak-force specification. Separate fast current sensing is required."},
    ]
    write_csv("voice_coil_parameter_targets.csv", targets)
    return {"evidenceType": "engineering_targets_with_primary_part_reference", "targets": targets,
            "staticCalculations": rows, "reference": examples["examples"][0],
            "hotResistanceAssumption": "8.896Ω =6.4Ω×(1+0.0039×100°C); model assumption, not manufacturer measured hot rating.",
            "passiveTimeConstant25C_s": L/Rc, "passiveTimeConstantHot_s": L/Rh,
            "coldMotorConstant_N_sqrtW": Kf/np.sqrt(Rc), "hotModelMotorConstant_N_sqrtW": Kf/np.sqrt(Rh),
            "driverLimitedPeakForce_N": 3.5*Kf,
            "catalogIntermittentForceCurrent_A": 204.3/Kf,
            "thermalWarning": "The drawing's45 W maximum continuous power and64.7 N reference are at25°C; a55–60 N guaranteed hot target is not established by that drawing. Model hot60 N copper loss is53.79 W. Extra bus voltage does not reduceI²R heat at the same force/current."}


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    bump, f1 = bump_demand()
    screw, f2 = screw_tradeoff(bump)
    historical, f3 = historical_comparison()
    voltage, f4 = voltage_pairs()
    targets = coil_targets()
    read_json("acf_simulink/validation/upper_no_damping_20261005/report/fixed_controller_contract.json")
    primary = read_json("acf_simulink/validation/report_readability_20261006/actuator/primary_sources_review_20261006.json")
    assessment = read_json("acf_simulink/validation/upper_no_damping_20261005/report/fixed_qualification_results.json")
    write_json("actuator_evidence.json", {
        "schemaVersion": 1, "status": "calculation_and_native_history_checks_complete_visual_review_pending",
        "reviewedDate": "2026-10-06", "owner": "voicecoil_report_evidence",
        "newNativeExperiments": 0, "prototypeMeasurements": 0,
        "question": "Which voice-coil parameters matter, and why prefer direct drive to the studied leadscrew?",
        "bumpDemand": bump, "screwPitchTradeoff": screw, "historicalComparison": historical,
        "fixedVoltageDiagnostics": voltage, "voiceCoilTargets": targets,
        "primarySourcesReviewed": primary["sources"],
        "currentQualificationTotals": assessment["totals"],
        "currentQualificationScope": assessment["scope"],
        "interpretation": [
            "A voice coil makes force approximately proportional to current and removes rotary transmission inertia/backlash. Real guides/cables and force-position variation remain.",
            "The exact bump lasts100 ms; its peak tool acceleration is2.4 m/s², not an instantaneous step.",
            "A screw trades holding-current advantage against rotary inertia. The selected RE25's10mm lead has good transient results on different mechanics but fails continuous holding current.",
            "No existing matched screw qualification has been established on the current fixed undamped upper mechanics. Merely switching its inactive actuator branch would not match the observer or cell feedback phase.",
            "Higher usable voltage changes a clipped current waveform but not an unclipped one. It does not fix the remaining force failure or thermal loss.",
            "Choose a voice coil for the development prototype; do not treat these results as proof it is perfect or every screw is unsuitable."
        ],
        "figures": [f1, f2, f3, f4], "sourceHashes": SOURCES,
        "reproduction": "uv run --with numpy --with scipy --with matplotlib python acf_simulink/experiments/report_readability_20261006/actuator/build_actuator_evidence.py",
        "reviewNotes": "Numerical asserts compare full native force/current/voltage/clip metrics; SVG visual review is recorded separately after rendering. No frozen source bytes are changed."
    })
    print(json.dumps({"status": "built", "figures": 4, "sourceFiles": len(SOURCES),
                      "bumpMaxAcceleration_m_s2": bump["peakToolAcceleration_m_s2"],
                      "fixedStress32V_N": voltage["rows"][2]["normalForcePeakError_N"],
                      "fixedStress48V_N": voltage["rows"][3]["normalForcePeakError_N"]}))


if __name__ == "__main__":
    main()
