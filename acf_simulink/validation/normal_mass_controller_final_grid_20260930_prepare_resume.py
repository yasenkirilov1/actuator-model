#!/usr/bin/env python3
"""Bind immutable r1 checkpoints and complete scored JSONL records for resume.

This is an evidence reader, not a simulator. It does not modify a worker's
checkpoint, configuration, trace or append log. New audit files record the
four stopped prefixes, including the explicitly interrupted part4 tail.
"""
from __future__ import annotations

import hashlib
import itertools
import json
from pathlib import Path

VALIDATION = Path(__file__).resolve().parent
PREFIX = VALIDATION / "normal_mass_controller_final_grid_20260930"
MODEL_SHA = "73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643"
CANDIDATE_SHA = "d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7"
RECIPE_SHA = "04ae0ae95e61d44695338b1ac6f58e1afec8329825562c67121be55af116a2db"
EXPECTED_COUNTS = [275, 263, 254, 268]
EXPECTED = list(itertools.product([.1, .5, 1, 1.4, 2], range(5, 51, 5),
                                 range(0, 91, 15), [0, .3], [0, 7, 29, 97, 1729]))
IDENTITY_FIELDS = ["massS_kg", "reference_N", "angle_deg", "mu", "seedRequested"]


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_new(path: Path, value) -> None:
    assert not path.exists(), f"Preserve the existing audit: {path}"
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")


def main() -> None:
    audits, all_indices, helper_contract = [], [], None
    for part, wanted_count in enumerate(EXPECTED_COUNTS, 1):
        folder = VALIDATION / f"final_grid/part{part}"
        stem = folder / f"normal_mass_controller_inertial_full_range_mass_20260930_v8_final_part{part}_r1"
        checkpoint_file = Path(str(stem) + "_results.json")
        report_mat = Path(str(stem) + "_results.mat")
        rows_file = Path(str(stem) + "_rows.jsonl")
        checkpoint = json.loads(checkpoint_file.read_text())
        content = rows_file.read_text()
        assert content.endswith("\n"), "Incomplete append-log record boundary"
        rows = [json.loads(line) for line in content.splitlines()]
        checkpoint_rows = checkpoint["rows"]
        if isinstance(checkpoint_rows, dict):
            checkpoint_rows = [checkpoint_rows]
        assert len(rows) == wanted_count
        assert rows[:len(checkpoint_rows)] == checkpoint_rows
        assert checkpoint["experimentSha256"] == RECIPE_SHA
        assert sha(Path(checkpoint["experimentSnapshot"])) == RECIPE_SHA
        assert checkpoint["modelSha256"] == MODEL_SHA
        assert sha(Path(checkpoint["privateModel"])) == MODEL_SHA
        assert sha(Path(checkpoint["sourceModel"])) == MODEL_SHA
        assert checkpoint["candidateSha256"] == CANDIDATE_SHA
        assert sha(Path(checkpoint["candidateFile"])) == CANDIDATE_SHA
        worker = Path(checkpoint["privateModel"]).parent
        assert sha(worker / "candidate.mat") == CANDIDATE_SHA
        for helper in checkpoint["helperSources"]:
            assert sha(worker / "helpers" / helper["file"]) == helper["sha256"]
        if helper_contract is None:
            helper_contract = checkpoint["helperSources"]
        assert helper_contract == checkpoint["helperSources"]
        opts = checkpoint["options"]
        assert opts["partitionIndex"] == part and opts["partitionCount"] == 4
        assert opts["simulationMode"] == "accelerator" and opts["simulationStep_s"] == 10e-6
        assert opts["clearSDIEvery"] == 1 and not opts["keepNativeWorstInMemory"]
        assert checkpoint["unpartitionedCaseIndices"] == list(range(part, 3501, 4))
        for local, row in enumerate(rows, 1):
            global_index = part + 4 * (local - 1)
            point = tuple(round(float(row[k]), 9) for k in IDENTITY_FIELDS)
            assert point == tuple(round(float(x), 9) for x in EXPECTED[global_index - 1])
            assert row["id"] == local
            assert row["executed"] and row["nativeReturned"] and row["completed"] and row["passed"]
            assert row["nativeSamples"] == 400001 and row["endTime_s"] == 4
            assert not row["lostContact"] and not row["strokeStop"] and row["springPackingPassed"]
            assert row["peakNormalForceError_N"] <= .5 and row["reason"] == "pass"
            assert row["normalBalanceResidual_N"] <= 1e-9
            assert row["generalAxialBalanceResidual_N"] <= 1e-9
            assert row["nativeActuatorForceResidual_N"] <= 1e-9
            assert row["lowerInverseInputIdentityResidual_N"] <= 1e-9
            all_indices.append(global_index)
        tail_count = len(rows) - len(checkpoint_rows)
        interruption_file = None
        if tail_count:
            assert part == 4 and tail_count == 4 and checkpoint["status"] == "running"
            interruption_file = folder / "part4_r1_interruption_recovery.json"
            interruption = json.loads(interruption_file.read_text())
            assert interruption["reportedProcessExitCode"] == 143
            assert interruption["nativeValidatedCases"] == len(rows)
            assert interruption["rowsSha256"] == sha(rows_file)
            classification = "process_exit_143_no_graceful_stop_marker"
        else:
            assert checkpoint["status"] == "stopped_by_coordinator"
            classification = "native_prefix_checkpoint_stopped_after_case"
        worst_native = Path(str(stem) + "_worst_native_trace.mat")
        first_native = Path(str(stem) + "_first_native_trace.mat")
        group_native = Path(checkpoint["controllerWorstNativeTraces"][0])
        assert len(checkpoint["controllerWorstNativeTraces"]) == 1
        assert not checkpoint["failedNativeTraces"]
        audit = {
            "status": "audited_immutable_r1_prefix", "part": part,
            "checkpointFile": str(checkpoint_file), "checkpointSha256": sha(checkpoint_file),
            "reportMatFile": str(report_mat), "reportMatSha256": sha(report_mat),
            "rowsFile": str(rows_file), "rowsSha256": sha(rows_file),
            "configurationFile": checkpoint["configurationFile"],
            "configurationSha256": sha(Path(checkpoint["configurationFile"])),
            "checkpointRows": len(checkpoint_rows), "validatedRows": len(rows),
            "recoveredCompleteTailRows": tail_count,
            "interruptionClassification": classification,
            "reportedProcessExitCode": 0 if part == 3 else 143,
            "interruptionFile": str(interruption_file) if interruption_file else "",
            "interruptionSha256": sha(interruption_file) if interruption_file else "",
            "recipeSnapshot": checkpoint["experimentSnapshot"], "recipeSha256": RECIPE_SHA,
            "privateModel": checkpoint["privateModel"], "modelSha256": MODEL_SHA,
            "candidateFile": checkpoint["candidateFile"], "candidateSha256": CANDIDATE_SHA,
            "helperSources": checkpoint["helperSources"],
            "firstNativeTrace": str(first_native), "firstNativeTraceSha256": sha(first_native),
            "worstNativeTrace": str(worst_native), "worstNativeTraceSha256": sha(worst_native),
            "controllerWorstNativeTrace": str(group_native),
            "controllerWorstNativeTraceSha256": sha(group_native),
            "lastLocalCase": len(rows), "nextLocalCase": len(rows) + 1,
            "lastGlobalCaseIndex": part + 4 * (len(rows) - 1),
            "nextGlobalCaseIndex": part + 4 * len(rows),
            "remainingCases": 875 - len(rows),
            "remainingGlobalCaseIndices": list(range(part + 4 * len(rows), 3501, 4)),
            "scope": "Complete scored native rows only, reused unchanged with the original r1 recipe. Original checkpoints, append logs, configuration and retained histories are never rewritten. Exit143 is not classified as a graceful stop. No unrecorded in-flight execution is counted as a validated history.",
        }
        audit_file = Path(str(PREFIX) + f"_part{part}_r1_resume_audit.json")
        write_new(audit_file, audit)
        audits.append({"part": part, "file": str(audit_file), "sha256": sha(audit_file),
                       "validatedRows": len(rows), "remainingCases": 875 - len(rows)})
    assert len(all_indices) == 1060 and len(set(all_indices)) == 1060
    plan = {"status": "prepared_for_root_review_not_launched", "r1ValidatedRows": 1060,
            "remainingUniqueNativeCases": 2440, "totalPlannedUniqueIdentities": 3500,
            "parts": audits, "modelSha256": MODEL_SHA, "candidateSha256": CANDIDATE_SHA,
            "originalRecipeSha256": RECIPE_SHA,
            "executionChange": "Only temporary SDI storage moves to per-process owned /dev/shm directories. Native physics, helpers, control, acquisition, integration, native logs and metrics remain fixed.",
            "parityProof": str(VALIDATION / "normal_mass_controller_tmpfs_sdi_pilot_20260930_v8_results.json"),
            "prepareRecipeSha256": sha(Path(__file__))}
    write_new(Path(str(PREFIX) + "_resume_plan.json"), plan)
    print(json.dumps(plan, indent=2))


if __name__ == "__main__":
    main()
