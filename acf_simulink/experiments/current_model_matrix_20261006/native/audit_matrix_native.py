"""Verify compact native evidence without regenerating any simulation arrays."""
from __future__ import annotations

import gzip
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OUT = ROOT / "acf_simulink/validation/current_model_matrix_20261006/native"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    source = OUT / "native_results.json"
    report = json.loads(source.read_text())
    selection = json.loads((OUT / "selection.json").read_text())
    launch = json.loads((OUT / "launch_attempt01.json").read_text())
    assert report["status"] == "native_current_matrix_command_replays_complete"
    assert report["qualification"] is False and report["controllerIndependentlyVerified"] is False
    assert sha(OUT / "selection.json") == report["sourceSelectionSha256"]
    assert len(report["cases"]) == 4 and len(report["refinements"]) == 2
    assert launch["temporary_directory_released"] and not Path(launch["temporary_directory"]).exists()
    for binding in report["sourceBindings"]:
        assert sha(Path(binding["path"])) == binding["sha256"]
    guards = []
    captures = []
    events = []
    for case, selected in zip(report["cases"], selection["cases"]):
        assert case["identity"] == selected["identity"]
        assert sha(Path(selected["controller_lock_path"])) == selected["controller_lock_sha256"]
        # MATLAB JSON normalizes integral double values to JSON integers.
        # Compare scalar values, then verify the original canonical input hash.
        assert case["config"] == selected["config"]
        config_digest = hashlib.sha256(json.dumps(selected["config"], sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        assert config_digest == case["configSha256"] == selected["config_sha256"]
        assert case["algebraicLoopCount"] == 0 and case["forceBalancePeak_N"] < 1e-8
        assert case["scoredEveryNativeEndpoint"] and case["nativeEndpointCount"] == 800001
        assert case["nativeScores"]["completed"] and not case["nativeScores"]["physicalFailed"]
        capture = case["commandCapture"]
        assert capture["original_metrics_identical"] and capture["points"] == 20001
        for name in capture["compared_fields"]:
            assert capture["force_metrics"][name] == case["originalMetrics"][name]
        captures.append(dict(identity=case["identity"], controller_lock_sha256=selected["controller_lock_sha256"],
                             config_sha256=config_digest, commands_uncompressed_sha256=case["originalCommandsUncompressedSha256"],
                             source_record=selected["source_record_identity"], nonruntime_metrics_exact=True,
                             event_limits_exact="event_force_limits" in capture["compared_fields"],
                             command_capture_count=1, temporary_commands_released=True))
        archive = Path(case["traceArchive"])
        assert sha(archive) == case["traceArchiveSha256"]
        raw = gzip.decompress(archive.read_bytes())
        assert hashlib.sha256(raw).hexdigest() == case["traceUncompressedSha256"]
        guard = case["predeclaredForceGuard_N"]
        assert guard == (1e-4 if case["config"]["pad_diameter"] > 0 else 1e-5)
        assert case["forceParityPassed"] == (case["native_vs_regular_C_force_peak_difference_N"] <= guard)
        assert case["originalSplitEventForceParityAtControllerTicksPassed"] == (case["native_vs_original_split_event_force_peak_difference_at_10k_N"] <= guard)
        guards.append(dict(identity=case["identity"], force_guard_N=guard,
                           native_regular_C_difference_N=case["native_vs_regular_C_force_peak_difference_N"],
                           native_regular_C_pass=case["forceParityPassed"],
                           native_original_split_event_difference_at_10k_N=case["native_vs_original_split_event_force_peak_difference_at_10k_N"],
                           native_original_split_event_pass=case["originalSplitEventForceParityAtControllerTicksPassed"],
                           peak_error_all_endpoints_and_sides_N=case["nativeScores"]["forcePeakErrorIncludingSideLimits_N"],
                           force_requirement_met_in_replay=case["nativeScores"]["forceRequirementMetInReplay"]))
        for event in case["rigidSideEvents"]:
            expected_jump = case["config"]["mass_s"] * (event["right_acceleration_m_s2"] - event["left_acceleration_m_s2"])
            assert abs(event["right_force_N"] - event["left_force_N"] - expected_jump) < 1e-10
            events.append(dict(identity=case["identity"], **event))
    refinements = []
    for row in report["refinements"]:
        assert row["scoredEveryRefinementEndpoint"] and row["refinementEndpointCount"] == 1600001
        assert row["extraLeftLimitCount"] == 2 and row["matchedBaseTimestamps"] == 800001
        assert row["refinementScoredValueCount"] == 1600003 and row["forceBalancePeak_N"] < 1e-8
        assert not row["refinedScores"]["forceRequirementMetInReplay"]
        refinements.append(dict(identity=row["identity"], base_step_s=row["baseStep_s"], half_step_s=row["refinementStep_s"],
                                matched_waveform_peak_difference_N=row["matchedForceWaveformPeakDifference_N"],
                                base_peak_error_N=row["basePeakErrorIncludingSideLimits_N"],
                                refined_all_endpoint_peak_error_N=row["refinedPeakErrorIncludingSideLimits_N"],
                                peak_metric_change_N=row["refinedPeakErrorIncludingSideLimits_N"] - row["basePeakErrorIncludingSideLimits_N"],
                                endpoint_count=int(row["refinementEndpointCount"]), extra_left_limit_count=row["extraLeftLimitCount"],
                                outcome_unchanged_force_failure=True, predeclared_refinement_pass_guard=None))
    endpoint_count = sum(case["nativeEndpointCount"] for case in report["cases"])
    left_count = sum(case["extraLeftLimitCount"] for case in report["cases"])
    refined_count = sum(row["refinementScoredValueCount"] for row in report["refinements"])
    assert endpoint_count == report["totalNativeEndpointsScored"] == 3200004
    assert left_count == report["totalExtraRigidLeftLimitsScored"] == 4
    assert refined_count == report["totalRefinementScoredValues"] == 3200006
    recipe_bindings = [dict(path=str(path.relative_to(ROOT)), sha256=sha(path))
                       for path in sorted(HERE.glob("*")) if path.is_file()]
    audit = dict(status="compact_native_audit_passed", qualification=False, controller_independently_verified=False,
                 native_results_sha256=sha(source), selection_sha256=sha(OUT / "selection.json"),
                 launch_audit_sha256=sha(OUT / "launch_attempt01.json"), recipe_bindings=recipe_bindings,
                 physical_source_bindings_verified=True, native_force_balance_max_N=max(c["forceBalancePeak_N"] for c in report["cases"]),
                 native_base_endpoints=int(endpoint_count), native_extra_left_limits=int(left_count),
                 native_scored_values=int(endpoint_count+left_count), refinement_scored_values=int(refined_count),
                 total_native_and_refinement_scored_values=int(endpoint_count+left_count+refined_count),
                 left_limits_share_event_timestamps=True, guard_results=guards, command_capture_results=captures,
                 rigid_base_event_pairs=events, refinement_results=refinements,
                 physical_stops_in_selected_native_histories=0, new_native_failure_probe_count=0,
                 current_soft_partial_contact_observed=any(c["nativeScores"]["partialSoftPadContactObserved"] for c in report["cases"]),
                 scope_limits=["External current-command replay; no independent native controller or acquisition test.",
                               "Loaded-equilibrium2s fixed-robot axial force; no engagement/recentering/normal-force/thermal qualification.",
                               "Rigid ideal plate, no tilt and no remaining interface compliance; force jumps are model-limit evidence.",
                               "Two rigid waveform refinements only; no new soft spatial/time refinement or native physical-stop probes.",
                               "500Hz display traces may miss high-frequency extrema; all-step metrics and exact side-event pairs are authoritative.",
                               "Half-step maxima include every intervening endpoint; matched-grid waveform difference is a distinct metric."],
                 saved_reopened_private_model_verified=report["modelSavedReopened"], saved_model_retained=False,
                 temporary_reference_and_command_arrays_released=True, temporary_bytes_peak=launch["temporary_peak_polled_bytes"],
                 temporary_cap_bytes=launch["worker_temporary_cap_bytes"])
    output = OUT / "validation_audit.json"
    output.write_text(json.dumps(audit, indent=2) + "\n")
    retained = sum(path.stat().st_size for folder in (HERE, OUT) for path in folder.rglob("*") if path.is_file())
    assert retained <= 500 * 1024
    print(json.dumps(dict(status=audit["status"], retained_bytes=retained,
                          scored_values=audit["total_native_and_refinement_scored_values"],
                          all_strict_guards_passed=all(g["native_regular_C_pass"] and g["native_original_split_event_pass"] for g in guards))))


if __name__ == "__main__":
    main()
