"""Freeze new qualification identities from retained context templates.

This writes schedules, calibration inputs and deterministic seeds only. It does
not prepare native parameters, select a controller or execute any simulation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parents[4]
STUDY = ROOT / "acf_simulink/validation/rl_budget3h_20261005"

# These are context or calibration fields. Physical springs, damping, guide
# friction and controller choices are deliberately absent from this list.
OVERRIDE_PATHS = (
    "sensor.bias", "sensor.strokeBias", "sensor.distalMassCalibrationScale",
    "accel.bias", "accel.scaleError",
    "uncertainty.forceScaleZ", "uncertainty.forceScaleX",
    "uncertainty.positionScale", "uncertainty.forceFrameAngle_rad",
    "uncertainty.accelFrameAngle_rad", "uncertainty.forceCrossZFromX",
    "uncertainty.forceCrossXFromZ", "uncertainty.accelBiasX_m_s2",
    "uncertainty.massB_model_kg", "uncertainty.forceConstant_model_N_A",
    "uncertainty.recenterGain",
    "acquisition.antiAliasCutoff_Hz", "acquisition.emaTime_s",
    "acquisition.forceAntiAliasScale", "acquisition.lateralAntiAliasScale",
    "acquisition.accelerationAntiAliasScale", "acquisition.positionAntiAliasScale",
    "acquisition.forceDelayOffset_s", "acquisition.lateralDelayOffset_s",
    "acquisition.accelerationDelayOffset_s", "acquisition.positionDelayOffset_s",
    "acquisition.pipelineDelay_s", "adcStudy.pipelineDelay_s",
    "robot.delay", "robot.responseTime", "robot.maxSpeed",
    "robot.maxAcceleration", "robot.nominalVelocity", "robot.errorRiseTime",
    "robot.errorForce", "robot.initialExcessForce", "robot.initialPositionOffset_m",
    "robotPrediction.delay", "robotPrediction.responseTime",
    "robotPrediction.maxSpeed", "robotPrediction.maxAcceleration",
    "robotPrediction.boundedNominalPath",
    "contact.commandedScanSpeed_m_s", "contact.lateralBias_N",
)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scalar(value):
    array = np.asarray(value)
    assert array.ndim == 0 or array.size == 1
    item = array.item()
    return item if isinstance(item, (str, int, bool)) else float(item)


def get_path(parameter: dict, path: str):
    value = parameter
    for part in path.split("."):
        value = value[part]
    return scalar(value)


def bank(relative: str):
    path = STUDY / relative
    rows = loadmat(path, simplify_cells=True)["configs"]
    return path, [rows] if isinstance(rows, dict) else list(rows)


def context(row: dict, source_index: int, cohort: str, index: int, seed: int):
    p = row["p"]
    return {
        "cohort": cohort,
        "caseID": index,
        "identity": f"fixed_mechanics_v8_{cohort}_{index:03d}",
        "templateScenario": source_index,
        "templateIdentity": row["identity"],
        "seed": seed,
        "massS_kg": scalar(p["mechanics"]["mS"]),
        "force_N": scalar(p["force"]["reference"]),
        "angle_rad": scalar(p["mechanics"]["angle"]),
        "mu": scalar(p["contact"]["mu"]),
        "duration_s": scalar(p["sim"]["stopTime"]),
        "surface": {name: scalar(p["surface"][name]) for name in
                    ("kind", "startTime", "height", "width", "scanSpeed", "repeatPeriod")},
        "overrides": [{"path": path, "value": get_path(p, path)} for path in OVERRIDE_PATHS],
        "excludedTemplateMechanics": {
            "actualLowerStiffness_N_m": scalar(p["mechanics"]["k1"]),
            "actualLowerDamping_Ns_m": scalar(p["mechanics"]["c1"]),
            "actualUpperRigid": bool(p["mechanics"]["upperRigid"]),
            "actualMassA_kg": scalar(p["mechanics"]["mA"]),
            "reason": "User fixes upper20k/c0/unlimited, lower finite64MN/m/c0 and A1.5kg.",
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    assert not output.exists(), "Preserve every previously frozen recipe."
    nominal_file, nominal = bank("qualification_bank_budget700/configs.mat")
    fresh_file, fresh = bank("scenario_bank_budget50/configs.mat")
    long_file, long = bank("long_qualification_bank_budget/configs.mat")
    selected_fresh = [(i, row) for i, row in enumerate(fresh) if row["split"] == "holdout"]
    assert len(nominal) == 700 and len(selected_fresh) == 32 and len(long) == 4

    rows = []
    for index, row in enumerate(nominal):
        item = context(row, index, "nominal700", index, 730000 + index)
        # Nominal response/calibration defaults will come from the selected
        # controller, not from the old physical/controller parameter set.
        item["overrides"] = []
        assert item["duration_s"] == 2 and item["surface"]["startTime"] == 1
        assert item["surface"]["kind"] == "sharp_bump" and item["mu"] == .3
        rows.append(item)
    for index, (source_index, row) in enumerate(selected_fresh):
        rows.append(context(row, source_index, "fresh32", index, 740000 + index))
    for index, row in enumerate(long):
        rows.append(context(row, index, "long4", index, 750000 + index))
    assert len(rows) == 736 and len({r["seed"] for r in rows}) == 736
    for scenario, row in enumerate(rows):
        row["scenario"] = scenario
        row["split"] = "qualification_" + row["cohort"]
    combinations = {(r["massS_kg"], r["force_N"], r["angle_rad"]) for r in rows[:700]}
    assert len(combinations) == 700

    recipe = {
        "status": "qualification_context_recipe_frozen_no_controller_selected_no_simulation",
        "qualificationEpisodesExecuted": 0,
        "preparedNativeParameterCases": 0,
        "date": "2026-10-05",
        "counts": {"nominal700": 700, "fresh32": 32, "long4": 4, "total": 736},
        "scenarioRangesInclusive": {"nominal700": [0, 699], "fresh32": [700, 731], "long4": [732, 735]},
        "gridMassS_kg": [.1, .3, .5, .7, .9, 1.1, 1.3, 1.5, 1.7, 2],
        "gridForce_N": list(range(5, 51, 5)),
        "gridAngle_deg": list(range(0, 91, 15)),
        "seedContract": "730000+nominalID0,740000+freshID0,750000+longID0; frozen before new qualification outcomes.",
        "sourceTemplates": [
            {"cohort": name, "configsFile": str(path), "configsSha256": sha(path)}
            for name, path in (("nominal700", nominal_file), ("fresh32", fresh_file), ("long4", long_file))
        ],
        "fixedMechanics": {"massA_kg": 1.5, "massB_kg": .6,
                           "upperStiffness_N_m": 20000, "upperDamping_Ns_m": 0,
                           "upperLengthAndTravelUnconstrained": True,
                           "lowerExternalSpringAbsent": True, "finiteCellStiffness_N_m": 64e6,
                           "finiteCellDamping_Ns_m": 0},
        "calibrationScope": "24 known-calibration varied contexts, eight retained simultaneous residual contexts, and four long residual contexts. No mechanical optimization or new noise reduction.",
        "translationContract": "Schedules, calibration/timing, actual versus identified robot inputs and tool mass are templates only. Exclude inherited spring/damper variation, spring catalogs, lower inverse tuning and the old long mixed-mechanics label. Selected PI/notch/observer/lead/guide/modal identification and sensor-noise chain remain separately locked.",
        "modalContract": "Frozen selected complex acquired transfer, identified50us current response and selected modal coefficients; hidden actual response changes never retune them.",
        "qualificationContract": "Prepare parameters only after a development-only candidate lock. Never select or retune on these outcomes. Root scores every native major point including startup/end and retains failed/worst exact replays. Finite simulation evidence, not hardware validation.",
        "overridePaths": list(OVERRIDE_PATHS),
        "cases": rows,
        "recipeSourceSha256": sha(Path(__file__)),
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(recipe, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": recipe["status"], "output": str(output),
                      "sha256": sha(output), "counts": recipe["counts"]}, indent=2))


if __name__ == "__main__":
    main()
