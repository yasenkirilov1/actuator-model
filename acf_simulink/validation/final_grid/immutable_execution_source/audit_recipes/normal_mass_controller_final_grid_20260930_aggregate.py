#!/usr/bin/env python3
"""Audit four immutable native mesh shards and combine their scored metrics.

Full-rate simulation and metric computation are performed by the recorded
MATLAB recipe. This reader never simulates, replaces a model, or rewrites a
worker's evidence. --partial produces an explicitly incomplete checkpoint.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import itertools
import json
import math
from pathlib import Path
from collections import defaultdict

ROOT = Path(__file__).resolve().parents[2]
VALIDATION = ROOT / "acf_simulink/validation"
PREFIX = VALIDATION / "normal_mass_controller_final_grid_20260930"
MODEL_SHA = "73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643"
CANDIDATE_SHA = "d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7"
RECIPE_SHA = "04ae0ae95e61d44695338b1ac6f58e1afec8329825562c67121be55af116a2db"
CONTINUATION_RECIPE_SHA = "7cef2cdd2d302ccbaaeb877406b067843f64d606f8b8ba78b8cb2d5e1af5516f"
MASSES = [.1, .5, 1., 1.4, 2.]
FORCES = list(range(5, 51, 5))
ANGLES = list(range(0, 91, 15))
MUS = [0., .3]
SEEDS = [0, 7, 29, 97, 1729]
IDENTITY_FIELDS = ["massS_kg", "reference_N", "angle_deg", "mu", "seedRequested"]
EXPECTED = list(itertools.product(MASSES, FORCES, ANGLES, MUS, SEEDS))


def digest(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def close(a, b) -> bool:
    if isinstance(b, bool) or isinstance(b, str):
        return a == b
    return isinstance(a, (float, int)) and math.isfinite(a) and math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-12)


def identity(row: dict) -> tuple:
    return tuple(round(float(row[k]), 9) for k in IDENTITY_FIELDS)


def write_json(path: Path, value) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def immutable_prefix(part: int) -> tuple[dict, list[dict], dict]:
    """Load the unchanged r1 report and separately audited complete append log."""
    audit_file = Path(str(PREFIX) + f"_part{part}_r1_resume_audit.json")
    audit = json.loads(audit_file.read_text())
    assert audit["status"] == "audited_immutable_r1_prefix" and audit["part"] == part
    for path_key, hash_key in [("checkpointFile", "checkpointSha256"),
                               ("reportMatFile", "reportMatSha256"),
                               ("rowsFile", "rowsSha256"),
                               ("configurationFile", "configurationSha256"),
                               ("recipeSnapshot", "recipeSha256"),
                               ("worstNativeTrace", "worstNativeTraceSha256"),
                               ("controllerWorstNativeTrace", "controllerWorstNativeTraceSha256")]:
        assert digest(Path(audit[path_key])) == audit[hash_key]
    prior = json.loads(Path(audit["checkpointFile"]).read_text())
    rows = [json.loads(line) for line in Path(audit["rowsFile"]).read_text().splitlines()]
    original_rows = prior["rows"] if isinstance(prior["rows"], list) else [prior["rows"]]
    assert len(original_rows) == audit["checkpointRows"]
    assert len(rows) == audit["validatedRows"] and rows[:len(original_rows)] == original_rows
    assert prior["experimentSha256"] == RECIPE_SHA and audit["recipeSha256"] == RECIPE_SHA
    if audit["recoveredCompleteTailRows"]:
        assert part == 4 and prior["status"] == "running" and len(rows) - len(original_rows) == 4
        assert digest(Path(audit["interruptionFile"])) == audit["interruptionSha256"]
    else:
        assert prior["status"] == "stopped_by_coordinator"
    audit["auditFile"] = str(audit_file)
    audit["auditSha256"] = digest(audit_file)
    return prior, rows, audit


def read_shard(part: int) -> tuple[Path, dict, list[dict], list[dict], dict | None]:
    folder = VALIDATION / f"final_grid/part{part}"
    stem = f"normal_mass_controller_inertial_full_range_mass_20260930_v8_final_part{part}"
    path = folder / (stem + "_r2_results.json")
    audit_file = Path(str(PREFIX) + f"_part{part}_r1_resume_audit.json")
    prior, prefix_rows, audit = immutable_prefix(part) if audit_file.is_file() else (None, [], None)
    if path.is_file():
        report = json.loads(path.read_text())
        rows = report["rows"] if isinstance(report["rows"], list) else [report["rows"]]
        assert report["experimentSha256"] == CONTINUATION_RECIPE_SHA
        assert digest(Path(report["experimentSnapshot"])) == CONTINUATION_RECIPE_SHA
        assert audit is not None and rows[:len(prefix_rows)] == prefix_rows
        segments = report["executionSegments"]
        if isinstance(segments, dict):
            segments = [segments]
        assert len(segments) == 2
        old, new = segments
        count = len(prefix_rows)
        assert (old["firstLocalCase"], old["lastLocalCase"], old["executedLastLocalCase"]) == (1, count, count)
        assert old["recipeSha256"] == RECIPE_SHA and old["storageBackend"] == "disk"
        assert old["reportFile"] == audit["reportMatFile"] and old["reportSha256"] == audit["reportMatSha256"]
        assert old["rowsFile"] == audit["rowsFile"] and old["rowsSha256"] == audit["rowsSha256"]
        assert old["auditFile"] == str(audit_file) and old["auditSha256"] == digest(audit_file)
        assert (new["firstLocalCase"], new["lastLocalCase"], new["executedLastLocalCase"]) == (count + 1, 875, len(rows))
        assert new["recipeSha256"] == CONTINUATION_RECIPE_SHA and new["storageBackend"] == "tmpfs"
        assert new["reportFile"] == str(path)
        assert report["predecessorReportFile"] == audit["reportMatFile"]
        assert report["predecessorReportSha256"] == audit["reportMatSha256"]
        assert report["predecessorAuditSha256"] == digest(audit_file)
        assert digest(Path(report["predecessorReusedReportFile"])) == report["predecessorReusedReportSha256"]
        assert report["predecessorValidatedNativeHistories"] == count
        assert report["newNativeAttempts"] == len(rows) - count
        assert report["newValidatedNativeHistories"] == sum(r["executed"] for r in rows[count:])
        assert report["SDIStorageBackend"] == "tmpfs" and report["options"]["sdiStorageParent"] == "/dev/shm"
        append_file = Path(new["rowsFile"])
        # Active checkpoints may trail the append log by at most seven cases.
        # Only checkpointed new rows are included until the worker finishes.
        append_rows = [json.loads(line) for line in append_file.read_text().splitlines() if line.strip()]
        assert append_rows[:len(rows) - count] == rows[count:]
        assert [r["id"] for r in rows] == list(range(1, len(rows) + 1))
        if report["status"] == "complete":
            assert len(append_rows) == 875 - count
        audit["originalElapsedSeconds"] = prior["elapsedSeconds"]
        return path, report, rows, segments, audit
    if prior is not None:
        path = Path(audit["checkpointFile"])
        report = dict(prior)
        report["rows"] = prefix_rows
        for key in ["attempts", "executed", "validatedNativeHistories", "nativeReturned", "passed"]:
            report[key] = len(prefix_rows)
        report["worstNativeTrace"] = audit["worstNativeTrace"]
        report["status"] = "audited_r1_prefix_waiting_for_continuation"
        segment = {"firstLocalCase": 1, "lastLocalCase": len(prefix_rows),
                   "executedLastLocalCase": len(prefix_rows), "recipeSha256": RECIPE_SHA,
                   "reportFile": audit["reportMatFile"], "rowsFile": audit["rowsFile"],
                   "storageBackend": "disk"}
        return path, report, prefix_rows, [segment], audit
    path = folder / (stem + "_r1_results.json")
    report = json.loads(path.read_text())
    rows = report["rows"] if isinstance(report["rows"], list) else [report["rows"]]
    segment = {"firstLocalCase": 1, "lastLocalCase": len(rows),
               "executedLastLocalCase": len(rows), "recipeSha256": RECIPE_SHA,
               "reportFile": str(path), "rowsFile": str(folder / (stem + "_r1_rows.jsonl")),
               "storageBackend": "disk"}
    return path, report, rows, [segment], None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--partial", action="store_true", help="Allow incomplete shards; never label them qualified.")
    args = parser.parse_args()
    expected_index = {tuple(round(float(x), 9) for x in point): i + 1 for i, point in enumerate(EXPECTED)}
    audit_errors, shards, rows, helpers, tags = [], [], [], None, None
    identities_seen = set()
    fixed = {
        "massB_kg": .6, "lowerRigid": False, "upperRigid": True,
        "externalLowerSpringPair": True, "lowerLinkStiffness_N_m": 19746,
        "lowerLinkDamping_Ns_m": 60, "cellStiffness_N_m": 64e6,
        "cellSeparateNativeState": False, "lowerPairIndividualSpringRate_N_m": 9873,
        "lowerInverseEnabled": True, "lowerInverseScale": 1,
        "lowerInverseCutoff_Hz": 1500, "lowerInverseFilterDamping": 1 / math.sqrt(2),
        "lowerInverseIdentifiedStiffness_N_m": 19746,
        "lowerInverseIdentifiedDamping_Ns_m": 60,
        "lowerInverseIdentifiedMassB_kg": .6,
        "lowerInverseIdentifiedForceConstant_N_A": 20,
        "currentResponse_s": 100e-6, "simulationStep_s": 10e-6,
        "forceKp_A_N": .3, "forceKi_A_Ns": 20, "forceKd_A_s_N": 0,
        "currentNotchEnabled": False, "feedbackFilterEnabled": True,
        "feedbackFilterCutoff_Hz": 1000, "feedbackFilterDamping": 1 / math.sqrt(2),
        "geometryMode": "measured_drag_ratio", "dragRatioFilter_s": .005,
        "guideVelocityPrediction_s": .0004, "accelerationPrediction_s": 0,
        "accelerationFeedforwardFilter_s": 0,
        "voltageLimit_V": 24, "currentLimit_A": 3.5,
        "rawAcquisitionEnabled": True, "rawAcquisitionPeriod_s": 10e-6,
        "rawAntiAliasCutoff_Hz": 5100, "rawAntiAliasDamping": 1 / math.sqrt(2),
        "rawAcquisitionEMA_s": 50e-6, "rawAcquisitionPipeline_s": 20e-6,
        "identifiedAcquisitionAntiAliasCutoff_Hz": 5100,
        "identifiedAcquisitionEMA_s": 50e-6,
        "identifiedAcquisitionPipeline_s": 20e-6,
        "forceControllerPeriod_s": 100e-6, "forceVarianceGateEnabled": True,
        "forceVarianceGateRmsLimit_N": .08,
        "commonSpringPreload_N": 33.6, "requestedStopTime_s": 4,
        "surfaceKind": "sharp_bump", "combinedFreeSpringLength_mm": 40.98,
        "combinedInstalledSpringLength_mm": 34.17355818900031,
    }
    for part in range(1, 5):
        folder = VALIDATION / f"final_grid/part{part}"
        if not any(folder.glob("*_r[12]_results.json")):
            shards.append({"part": part, "status": "not_present", "rows": 0})
            continue
        try:
            path, report, source_rows, segments, predecessor_audit = read_shard(part)
        except json.JSONDecodeError:
            if args.partial:
                shards.append({"part": part, "status": "checkpoint_write_in_progress", "rows": 0})
                continue
            raise
        shard = {"part": part, "resultFile": str(path), "status": report["status"],
                 "rows": len(source_rows), "executed": report["executed"],
                 "passed": report["passed"], "elapsedSeconds": report["elapsedSeconds"],
                 "processPeakResidentMiB": report.get("processPeakResidentMiB"),
                 "resultSHA256": digest(path), "worstNativeTrace": report.get("worstNativeTrace"),
                 "executionSegments": segments, "predecessorAudit": predecessor_audit,
                 "newNativeHistories": report.get("newValidatedNativeHistories", 0),
                 "reusedR1NativeHistories": predecessor_audit["validatedRows"] if predecessor_audit else 0}
        shards.append(shard)
        for key, value in [("modelSha256", MODEL_SHA), ("candidateSha256", CANDIDATE_SHA),
                           ("simulationMode", "accelerator"),
                           ("plannedCases", 875), ("unpartitionedPlannedCases", 3500)]:
            if report.get(key) != value:
                audit_errors.append(f"part{part}: {key} differs from frozen contract")
        if report.get("experimentSha256") not in [RECIPE_SHA, CONTINUATION_RECIPE_SHA]:
            audit_errors.append(f"part{part}: unknown experiment recipe")
        opts = report["options"]
        for key, value in [("partitionIndex", part), ("partitionCount", 4),
                           ("massS_kg", MASSES), ("forceValues_N", FORCES),
                           ("angleValues_deg", ANGLES), ("muValues", MUS), ("seeds", SEEDS),
                           ("clearSDIEvery", 1), ("keepNativeWorstInMemory", False),
                           ("retainPlotExcerpts", False)]:
            if opts.get(key) != value:
                audit_errors.append(f"part{part}: options.{key} differs")
        if report.get("unpartitionedCaseIndices") != list(range(part, 3501, 4)):
            audit_errors.append(f"part{part}: disjoint global index list differs")
        if helpers is None:
            helpers, tags = report["helperSources"], report["controllerFromTags"]
        elif helpers != report["helperSources"] or tags != report["controllerFromTags"]:
            audit_errors.append(f"part{part}: helper or controller-input provenance differs")
        base = report["baseParameters"]
        spring = base["springs"]["lower"]
        if not close(spring["freeLength_m"], .02) or not close(base["springs"]["commonPreload_N"], 33.6):
            audit_errors.append(f"part{part}: stored approximate20mm spring target/preload differs")
        if not close(base["sim"]["evaluateFrom"], 0) or not base["robot"]["forceGate"]["enabled"]:
            audit_errors.append(f"part{part}: startup scoring or slow force gate disabled")
        if report["status"] == "complete":
            if not report.get("privateFrozenModelUnchanged") or not report.get("upstreamSourceUnchanged"):
                audit_errors.append(f"part{part}: source immutability check missing/failed")
        failed_native_rows = [r for r in source_rows if r["executed"] and not r["passed"]]
        retained_failures = report.get("failedNativeTraces", [])
        if len(retained_failures) != len(failed_native_rows) or any(not Path(f).is_file() for f in retained_failures):
            audit_errors.append(f"part{part}: some failed validated histories were not retained")
        for local_position, original in enumerate(source_rows, 1):
            row = dict(original)
            point = identity(row)
            global_id = expected_index.get(point)
            if point in identities_seen:
                audit_errors.append(f"part{part}/case{local_position}: duplicate physical identity")
            identities_seen.add(point)
            if global_id != part + 4 * (local_position - 1) or row["id"] != local_position:
                audit_errors.append(f"part{part}/case{local_position}: wrong ordered identity/global index")
            for key, value in fixed.items():
                if not close(row.get(key), value):
                    audit_errors.append(f"part{part}/case{local_position}: {key} differs")
            noise = [0, 0, 0, 0] if row["seedRequested"] == 0 else [.02, 1e-6, .02, .014]
            for key, value in zip(["forceNoiseRms_N", "positionNoiseRms_m", "lateralNoiseRms_N", "accelNoiseRms_m_s2"], noise):
                if not close(row.get(key), value):
                    audit_errors.append(f"part{part}/case{local_position}: {key} differs")
            if row["executed"]:
                if row["completed"] and (row["nativeSamples"] != 400001 or not close(row["endTime_s"], 4)):
                    audit_errors.append(f"part{part}/case{local_position}: incomplete native sampling grid")
                expected_pass = (row["completed"] and not row["lostContact"] and not row["strokeStop"]
                                 and row["peakNormalForceError_N"] <= .5 and row["springPackingPassed"])
                if row["passed"] != expected_pass:
                    audit_errors.append(f"part{part}/case{local_position}: failure classification differs")
            row["partitionIndex"] = part
            row["globalCaseId"] = global_id
            segment = next(s for s in segments if s["firstLocalCase"] <= local_position <= s["executedLastLocalCase"])
            row["executionRecipeSha256"] = segment["recipeSha256"]
            row["nativeRowsFile"] = segment["rowsFile"]
            row["executionEvidenceReportFile"] = segment["reportFile"]
            row["transientSDIStorageBackend"] = segment["storageBackend"]
            rows.append(row)
    rows.sort(key=lambda r: r["globalCaseId"] or 99999)
    complete = len(rows) == 3500 and all(s["status"] == "complete" for s in shards)
    missing = [list(point) for point in expected_index if point not in identities_seen]
    recoveries = []
    for part in [1, 4]:
        stopped = VALIDATION / f"final_grid/part{part}/normal_mass_controller_inertial_full_range_mass_20260930_v8_final_part{part}_results.json"
        if stopped.is_file():
            prior = json.loads(stopped.read_text())
            assert prior["status"] == "stopped_by_coordinator"
            assert prior["modelSha256"] == MODEL_SHA and prior["candidateSha256"] == CANDIDATE_SHA
            recoveries.append({"part": part, "resultFile": str(stopped), "resultSHA256": digest(stopped),
                               "status": prior["status"], "attempts": prior["attempts"],
                               "nativeReturned": prior["nativeReturned"], "validatedNativeHistories": prior["validatedNativeHistories"],
                               "passed": prior["passed"], "excludedFromUniqueGrid": True,
                               "reason": "Stopped safely after a complete case to add retention of every failed native history; parameters and model did not change. The corresponding points were rerun in the final r1 shards."})
    coverage = {"planned": 3500, "recorded": len(rows), "uniqueIdentities": len(identities_seen),
                "complete": complete, "auditPassed": not audit_errors,
                "identityFields": IDENTITY_FIELDS, "missingIdentities": missing, "auditErrors": audit_errors,
                "shards": shards, "modelSHA256": MODEL_SHA, "candidateSHA256": CANDIDATE_SHA,
                "recipeSHA256": RECIPE_SHA, "recipeSHA256s": [RECIPE_SHA, CONTINUATION_RECIPE_SHA],
                "helperSources": helpers, "controllerFromTags": tags,
                "aggregatorFile": str(Path(__file__).resolve()), "aggregatorSHA256": digest(Path(__file__)),
                "segmentedExecutionScope": "Original r1 complete native rows are reused exactly with recipe04ae provenance; remaining ordered identities execute once with the reviewed continuation recipe. Only transient SDI storage changes, with all72-channel bitwise source proof. Original interrupted part4 checkpoint and complete append-log recovery remain independently inspectable."}
    coverage["recoveryExecutions"] = recoveries
    coverage["additionalRecoveryNativeHistories"] = sum(r["nativeReturned"] for r in recoveries)
    coverage["reusedR1NativeHistories"] = sum(s.get("reusedR1NativeHistories", 0) for s in shards)
    coverage["newContinuationNativeHistories"] = sum(s.get("newNativeHistories", 0) for s in shards)
    coverage["tmpfsExecutionParityProof"] = str(VALIDATION / "normal_mass_controller_tmpfs_sdi_pilot_20260930_v8_results.json")
    coverage["recordedExecutionCountScope"] = "Counts durable native-returned scored rows, including122 explicitly rerun recovery histories. The interrupted part4 process has no graceful exit marker; an unrecorded in-flight attempt cannot be counted or ruled out from these records. Its268 complete scored histories are proven and reused, with four recovered beyond the264-row checkpoint."
    write_json(Path(str(PREFIX) + "_coverage.json"), coverage)
    validated = [r for r in rows if r["executed"]]
    failures = [r for r in rows if not r["passed"]]
    worst = max(validated, key=lambda r: r["peakNormalForceError_N"], default=None)
    groups = defaultdict(list)
    for row in validated:
        groups[identity(row)[:-1]].append(row)
    grouped = []
    for point, values in sorted(groups.items()):
        peak = max(values, key=lambda r: r["peakNormalForceError_N"])
        grouped.append(dict(zip(IDENTITY_FIELDS[:-1], point)) | {
            "seedCases": len(values), "passed": sum(r["passed"] for r in values),
            "worstNormalForceError_N": peak["peakNormalForceError_N"],
            "worstSeed": peak["seedRequested"], "minimumMargin_N": .5 - peak["peakNormalForceError_N"]})
    spring_low = max((r["minimumCommonSpringPreload_N"] for r in validated), default=None)
    spring_high = min((r["maximumCommonSpringPreload_N"] for r in validated), default=None)
    summary = {"status": "complete_audited_finite_nominal_grid" if complete and not audit_errors else "incomplete_or_audit_failed",
               "qualificationScope": "Finite nominal sharp-bump grid, exact known mass/tare, fixed orientations and Coulomb drag coefficients; every recorded native sample is scored. This is not hardware qualification or an uncertainty guarantee.",
               "plannedCases": 3500, "attempts": len(rows), "validatedNativeHistories": len(validated),
               "passed": len(rows) - len(failures), "failures": len(failures),
               "meshNativeExecutionsIncludingRecovery": sum(r["nativeReturned"] for r in recoveries) + sum(r["nativeReturned"] for r in rows),
               "seedZeroMeaning": "Stochastic force, position, lateral-force and acceleration noise are disabled. Actual sensor filters, acquisition delay, quantization and causal control remain enabled.",
               "nativeSamplesScored": sum(r["nativeSamples"] for r in validated),
               "coverage": coverage, "worstCase": worst,
               "minimumNormalForceMargin_N": .5 - worst["peakNormalForceError_N"] if worst else None,
               "fixedSpringPreload_N": 33.6, "commonSpringPreloadIntersection_N": [spring_low, spring_high],
               "allActualFixedSpringPreloadsPassed": all(r["springPackingPassed"] for r in validated) if validated else False,
               "worstByMass": [{"massS_kg": mass, "cases": len(values), "worstCase": max(values, key=lambda r: r["peakNormalForceError_N"])}
                               for mass in MASSES if (values := [r for r in validated if close(r["massS_kg"], mass)])],
               "groupedSurface": grouped, "failedCases": failures}
    for field in ["peakCurrent_A", "rmsCurrent_A", "peakVoltage_V", "voltageClipFraction", "effortClipFraction",
                  "peakNormalEstimateError_N", "peakLowerInverseCurrent_A", "combinedFreeSpringLength_mm",
                  "combinedInstalledSpringLength_mm", "normalBalanceResidual_N", "generalAxialBalanceResidual_N",
                  "nativeActuatorForceResidual_N", "lowerInverseInputIdentityResidual_N"]:
        summary["maximum_" + field] = max((r[field] for r in validated if r.get(field) is not None), default=None)
    if complete and worst:
        selected_shard = shards[worst["partitionIndex"] - 1]
        trace = Path(selected_shard["worstNativeTrace"])
        summary["worstNativeTrace"] = str(trace)
        summary["worstNativeTraceSHA256"] = digest(trace)
    write_json(Path(str(PREFIX) + "_summary.json"), summary)
    write_json(Path(str(PREFIX) + "_failures.json"), failures)
    write_csv(Path(str(PREFIX) + "_rows.csv"), rows)
    write_csv(Path(str(PREFIX) + "_groups.csv"), grouped)
    print(json.dumps({"complete": complete, "rows": len(rows), "validated": len(validated),
                      "passed": len(rows) - len(failures), "auditErrors": len(audit_errors),
                      "worstError_N": worst["peakNormalForceError_N"] if worst else None,
                      "preloadIntersection_N": [spring_low, spring_high]}))
    if audit_errors or (not args.partial and not complete):
        raise SystemExit("Grid audit failed or the requested3500 cases are incomplete; inspect coverage.json.")


if __name__ == "__main__":
    main()
