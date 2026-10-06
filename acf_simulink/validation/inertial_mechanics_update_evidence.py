#!/usr/bin/env python3
"""Rebuild the owned mechanics evidence index from completed native studies.

This does not execute simulations. The stiffness rows are reviewed inputs for
the report author's existing HTML/Markdown visualizations, not a new report.
"""
from collections import defaultdict
from hashlib import sha256
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parent


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def study_rows(study):
    rows = study.get("rows", [])
    return [rows] if isinstance(rows, dict) else rows


def rebuild_index():
    studies = []
    by_model = defaultdict(lambda: {"nativeExecutions": 0, "setupRejections": 0})
    for path in sorted(ROOT.glob("inertial_mechanics_*_results.json")):
        data = json.loads(path.read_text())
        rows = study_rows(data)
        native = sum(bool(row.get("executed")) for row in rows)
        rejected = sum(not bool(row.get("executed")) for row in rows)
        assert native == data["executedCases"], path
        assert rejected == data.get("setupRejections", 0), path
        model = data["sourceModelSHA256"]
        by_model[model]["nativeExecutions"] += native
        by_model[model]["setupRejections"] += rejected
        retained_failed = [
            row["nativeTraceFile"]
            for row in rows
            if row.get("executed") and not row.get("physicalPassed")
            and row.get("nativeTraceFile")
        ]
        for trace in retained_failed:
            assert Path(trace).is_file(), trace
        studies.append({
            "resultFile": path.name,
            "resultSHA256": digest(path),
            "phase": data["phase"],
            "status": data["status"],
            "nativeExecutions": native,
            "setupRejections": rejected,
            "individualPhysicalPasses": sum(bool(row.get("physicalPassed")) for row in rows),
            "sourceModelSHA256": model,
            "recipeSHA256": data["recipeSHA256"],
            "privateFolder": data["privateFolder"],
            "failedNativeTraceCount": len(retained_failed),
            "elapsedSeconds": data.get("elapsedSeconds"),
        })
    index = {
        "date": "2026-09-30",
        "status": "lower_series_inverse_pilot_selected_long_run_and_final_qualification_pending",
        "policy": (
            "Executed native histories, including repetitions and solver refinements, "
            "across intermediate controller versions. Short-run individual passes do "
            "not qualify the final system. Preparation rejections are separate from "
            "physical failures. The bounded lower-series spring studies and the "
            "earlier undamped sensing-cell studies have distinct controller and "
            "mechanical contracts. No final full-range grid is implied."
        ),
        "nativeExecutions": sum(study["nativeExecutions"] for study in studies),
        "setupRejections": sum(study["setupRejections"] for study in studies),
        "byModelSHA256": dict(by_model),
        "studies": studies,
        "nextRequired": (
            "The selected lower-series spring/damper and proper B-inertia inverse "
            "feedforward need repeated-bump, numerical refinement, actual versus "
            "identified parameter and final native full-range qualification."
        ),
    }
    target = ROOT / "inertial_mechanics_evidence_index_20260930.json"
    target.write_text(json.dumps(index, indent=2) + "\n")
    lines = [
        "# Inertial mechanics evidence index", "",
        "Status: lower-series inverse feedforward pilot selected; long-run and final qualification are pending.", "",
        f"The recorded studies contain {index['nativeExecutions']} native simulations and "
        f"{index['setupRejections']} separate preparation rejections. These counts include repeated "
        "parameter points, numerical refinement and failed intermediate controllers. They are "
        "not a count of qualified final design points.", "",
        "| Study | Native runs | Preparation rejects | Individual physical passes | Status |",
        "|---|---:|---:|---:|---|",
    ]
    for study in studies:
        name = study["resultFile"]
        lines.append(
            f"| [{name}]({name}) | {study['nativeExecutions']} | {study['setupRejections']} "
            f"| {study['individualPhysicalPasses']} | {study['status']} |"
        )
    lines += [
        "", "Every input result file and its native model/recipe are hash-bound in the JSON index. "
        "Later studies retain every failed full native trace in their private directory; early "
        "studies retain first/best/worst traces only. Source model versions remain distinct.", "",
        "Initial external spring topology tuning comprises 126 runs on earlier inertial controller "
        "versions. It does not establish that all spring designs fail: the measured-motion "
        "feedforward and robot observer require separate qualification for compliant links. "
        "The subsequent lower-spring studies fairly tune larger PI gains, filtered PID and "
        "a proper causal B-inertia inverse before selecting a compact spring/damper candidate. "
        "The four-rate full spring grid has not been executed on the final controller.", "",
        "The V6 pilots compare 40–110 MN/m structural cell rates with zero assumed structural "
        "damping, no external springs, 2 kg and 0.1 kg tools, 50 N, 90°, and a finite seed cohort. "
        "All 72 V6 runs retained contact and completed; force errors above 0.5 N remain failures. "
        "These stiffness values and hypothetical damping cases are design sensitivities, not "
        "manufacturer stiffness tolerances or damping specifications.", "",
        index["nextRequired"], "",
    ]
    (ROOT / "inertial_mechanics_evidence_index_20260930.md").write_text("\n".join(lines))
    return index


def stiffness_chart_data():
    labels = {
        "v6_ratio": "P 0.03; I 20; feedback damping 0.707",
        "v6_ratio_damping15": "P 0.06; I 20; feedback damping 1.5",
    }
    points = []
    sources = []
    grouped = defaultdict(list)
    for suffix, label in labels.items():
        path = ROOT / (
            "inertial_mechanics_structural_cell_rate_pilot_20260930_"
            f"{suffix}_results.json"
        )
        study = json.loads(path.read_text())
        sources.append({
            "file": path.name,
            "sha256": digest(path),
            "sourceModelSHA256": study["sourceModelSHA256"],
            "candidateSHA256": study["candidateSHA256"],
            "recipeSHA256": study["recipeSHA256"],
        })
        for row in study_rows(study):
            assert row["executed"] and row["completed"]
            assert row["referenceForce_N"] == 50 and row["angle_deg"] == 90
            assert not row["contactLoss"] and not row["strokeStop"]
            assert not row["currentSaturationFraction"] and not row["voltageSaturationFraction"]
            point = {
                "id": f"{suffix}-case-{row['caseIndex']:04d}",
                "controller": label,
                "stiffness_MN_m": row["k1_N_m"] / 1e6,
                "massS_kg": row["massS_kg"],
                "seed": row["seed"],
                "sensorNoiseEnabled": row["forceNoiseRms_N"] > 0,
                "peakError_N": row["peakNormalForceError_N"],
                "rmsError_N": row["rmsNormalForceError_N"],
                "passed": row["physicalPassed"],
                "caseIndex": row["caseIndex"],
                "resultFile": path.name,
            }
            points.append(point)
            grouped[(label, point["stiffness_MN_m"])].append(point)
    maxima = []
    for (label, stiffness), rows in grouped.items():
        worst = max(rows, key=lambda row: row["peakError_N"])
        maxima.append({
            "controller": label,
            "stiffness_MN_m": stiffness,
            "worstPeakError_N": worst["peakError_N"],
            "caseCount": len(rows),
            "passedCount": sum(row["passed"] for row in rows),
            "worstCaseId": worst["id"],
            "worstMassS_kg": worst["massS_kg"],
            "worstSeed": worst["seed"],
        })
    result = {
        "date": "2026-09-30",
        "status": "intermediate_controller_stiffness_pilot",
        "metric": "max(abs(true workpiece normal force - 50 N)) over every native sample, 0–4 s",
        "threshold_N": 0.5,
        "scope": (
            "Sharp bump; 90 degree fixed orientation; mu 0.3; B 0.6 kg; "
            "heavy S 2 kg with clean and seeds 7, 29, 1729, and light S 0.1 kg "
            "with clean and seed 29; 10 us RK4 plant step. The clean rows disable "
            "random sensor noise but retain sampling, filter, delay and quantization."
        ),
        "commonController": (
            "Measured drag ratio 5 ms; normal force PI with feedback-only 1000 Hz "
            "second-order lowpass; no total-current notch; guide prediction 400 us; "
            "acceleration feedforward lead 0; matched force/acceleration 100 us filters "
            "and 100 us acquisition delays."
        ),
        "mechanics": (
            "Finite B-S structural cell, upper rigid, no external springs, "
            "zero structural damping. Rates are sensitivity values, not a "
            "qualified cell manufacturing tolerance."
        ),
        "chartGuidance": (
            "Use a scatter plot of all reviewed point errors versus numeric stiffness, "
            "faceted by controller with common axes and a 0.5 N reference. Mass and "
            "seed belong in accessible tooltips/source inspection. A separate grouped "
            "dot/bar comparison may display the six-case maxima. Do not display the "
            "seed spread or stiffness sensitivity as confidence intervals or imply "
            "continuous qualified stiffness coverage."
        ),
        "sources": sources,
        "rows": points,
        "sixCaseMaxima": maxima,
    }
    assert len(points) == 72 and len(maxima) == 12
    target = ROOT / "inertial_mechanics_stiffness_chart_data_20260930.json"
    target.write_text(json.dumps(result, indent=2) + "\n")


def spring_chart_data():
    cohorts = {
        "PI": "inertial_mechanics_lower_spring_gain_pilot_20260930_v6_high_gain_results.json",
        "Filtered PID": "inertial_mechanics_lower_spring_pid_pilot_20260930_v6_filtered_PID_results.json",
        "PI or PID plus proper inverse": "inertial_mechanics_lower_spring_inverse_mechanics_20260930_inverse_rates_damping_results.json",
    }
    points, sources, paired, best = [], [], [], []
    for family, name in cohorts.items():
        path = ROOT / name
        data = json.loads(path.read_text())
        sources.append({"file": name, "sha256": digest(path),
                        "sourceModelSHA256": data["sourceModelSHA256"],
                        "recipeSHA256": data["recipeSHA256"]})
        groups = defaultdict(list)
        for row in study_rows(data):
            assert row["executed"] and row["massS_kg"] == 2
            assert row["referenceForce_N"] == 50 and row["angle_deg"] == 90
            assert row["frictionCoefficient"] == 0.3
            point = {"controllerFamily": family, "caseIndex": row["caseIndex"],
                     "resultFile": name, "rate_N_mm": row["k1_N_m"] / 1000,
                     "damping_Ns_m": row["c1_Ns_m"], "P_A_N": row["forceKp"],
                     "I_A_N_s": row["forceKi"], "D_A_s_N": row["forceKd"],
                     "seed": row["seed"], "noiseEnabled": row["forceNoiseRms_N"] > 0,
                     "completed": row["completed"], "physicalPassed": row["physicalPassed"],
                     "packingFeasible": row["packingPassed"],
                     "peakError_N": row["peakNormalForceError_N"],
                     "failureReason": row["failureReason"],
                     "inverseCutoff_Hz": row.get("lowerInverseCutoff_Hz"),
                     "feedbackDamping": row["feedbackFilterDamping"],
                     "currentClipped": row["currentSaturationFraction"] > 0,
                     "voltageClipped": row["voltageSaturationFraction"] > 0}
            points.append(point)
            key = (point["rate_N_mm"], point["damping_Ns_m"], point["P_A_N"],
                   point["I_A_N_s"], point["D_A_s_N"], point["inverseCutoff_Hz"])
            groups[key].append(point)
        for key, rows in groups.items():
            assert len(rows) == 2 and sum(row["noiseEnabled"] for row in rows) == 1
            assert all(row["seed"] == 1729 for row in rows if row["noiseEnabled"])
            paired.append({"controllerFamily": family, "rate_N_mm": key[0],
                           "damping_Ns_m": key[1], "P_A_N": key[2],
                           "I_A_N_s": key[3], "D_A_s_N": key[4],
                           "inverseCutoff_Hz": key[5],
                           "bothCompleted": all(row["completed"] for row in rows),
                           "bothPhysicalPassed": all(row["physicalPassed"] for row in rows),
                           "packingFeasible": all(row["packingFeasible"] for row in rows),
                           "worstPeakError_N": max(row["peakError_N"] for row in rows),
                           "caseIndices": [row["caseIndex"] for row in rows],
                           "resultFile": name})
    by_design = defaultdict(list)
    for pair in paired:
        by_design[(pair["controllerFamily"], pair["rate_N_mm"], pair["damping_Ns_m"])].append(pair)
    for (family, rate, damping), rows in by_design.items():
        complete = [row for row in rows if row["bothCompleted"]]
        selected = min(complete, key=lambda row: row["worstPeakError_N"]) if complete else None
        best.append({"controllerFamily": family, "rate_N_mm": rate,
                     "damping_Ns_m": damping, "testedTunePairs": len(rows),
                     "completeTunePairs": len(complete), "bestCompletedTune": selected})
    result = {"date": "2026-09-30", "status": "bounded_design_and_controller_search",
              "metric": "maximum absolute true normal force error across all native samples, 0–4 s",
              "threshold_N": 0.5,
              "scope": "S2 kg, B0.6 kg, F50 N, 90 degrees, mu0.3, sharp bump, upper rigid, native10us RK4",
              "cohort": "Each compared tuning has one clean case and one identical seed1729 noisy case; clean retains filter/delay/quantization.",
              "packingPolicy": "Search rows evaluate the feasible common-preload interval from recorded spring excursions; final selected preset33.6N must be checked separately.",
              "chartGuidance": "Show minimum worst error among complete clean/noisy tuning pairs by rate/damping and family; clearly label tuned family, tested-tuning count and packing rejects. Contact-loss pairs are failures and must not be plotted as successful low errors. This is a bounded search, not a continuous qualification or confidence interval.",
              "sources": sources, "nativeRows": points, "equalCohortTuningPairs": paired,
              "bestCompletedPerFamilyAndMechanics": best}
    assert len(points) == 168 and len(paired) == 84 and len(best) == 18
    (ROOT / "inertial_mechanics_spring_chart_data_20260930.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    index = rebuild_index()
    stiffness_chart_data()
    spring_chart_data()
    print(json.dumps({"nativeExecutions": index["nativeExecutions"],
                      "setupRejections": index["setupRejections"],
                      "studies": len(index["studies"])}))
