"""Audit compact controller evidence without running or retuning simulations."""
from __future__ import annotations

import json
import math
from pathlib import Path

import bounded_runner as runner


def check_metrics(row: dict, p: dict) -> int:
    assert row["status"] == "executed"
    m = row["metrics"]
    assert m["force_pass"] == (m["completed"] and m["force_peak_error"] <= .5)
    assert m["physical_pass"] == m["completed"]
    assert m["actuator_ratings_pass"] == (m["current_peak"] <= p["current_limit"]+1e-6 and
        (p["actuator"] == 0 or m["rotor_speed_peak_rpm"] <= 12000))
    assert m["sensor_ratings_pass"] == (max(abs(m["cell_raw_min"]), abs(m["cell_raw_max"])) <= 100
                                          and m["specific_acceleration_peak"] <= 8*9.80665)
    continuous = bool(m["current_rms"] <= p["continuous_current"]) if p["continuous_current"] else None
    assert m["continuous_rms_current_pass"] == continuous
    assert m["eligibility_pass"] == (m["completed"] and m["actuator_ratings_pass"]
        and m["sensor_ratings_pass"] and m["continuous_static_current_pass"] is not False
        and m["continuous_rms_current_pass"] is not False)
    assert m["force_and_eligibility_pass"] == (m["force_pass"] and m["eligibility_pass"])
    if m["completed"]:
        assert m["control_count"] == 20001 and m["adc_count"] == 60001
    else:
        assert m["stop_reason"] == "contact_loss" and m["force_min"] <= 0
    events = m.get("event_force_limits", [])
    g = p["gravity"]*math.cos(math.radians(p["angle_deg"]))
    for event in events:
        assert event["scored_side_count"] == 2
        for side in ("left", "right"):
            required = event["continuous_cell_raw_N"]+p["mass_s"]*(event[f"acceleration_{side}_m_s2"]+g)
            assert abs(required-event[f"force_{side}_N"]) <= 1e-8
            assert m["force_peak_error"]+1e-10 >= abs(event[f"force_{side}_N"]-p["force_target"])
        assert abs(abs(event["force_right_N"]-event["force_left_N"])-2.4*p["mass_s"]) <= 1e-8
        assert abs(abs(event["acceleration_right_m_s2"]-event["acceleration_left_m_s2"])-2.4) <= 1e-12
        invalid = -1 if event["force_left_N"] <= 0 else 1 if event["force_right_N"] <= 0 else 0
        assert event["first_invalid_side"] == invalid
    if p["pad_k"] == 0:
        assert m["one_sided_extra_limit_count"] == len(events)
        assert m["scored_integration_endpoint_count"]+len(events) == m["steps"]
        if m["completed"]:
            assert len(events) == 2
            assert m["force_peak_error"]+1e-10 >= 1.2*p["mass_s"]
    return len(events)


def main() -> None:
    runner.verify_original_inputs()
    out = runner.OUT
    complete_path = out / "matrix_completion.json"
    complete = runner.read(complete_path)
    spec = runner.read_contract(runner.ROOT_CONTRACT)
    selected_path = out / "selected_banks.json"
    selected = runner.read(selected_path)
    assert complete["selected_banks_sha256"] == runner.sha(selected_path)
    declaration = runner.read(out / "final_matrix_declaration.json")
    assert complete["declaration_sha256"] == runner.sha(out / "final_matrix_declaration.json")
    assert declaration["runner_sha256"] == runner.sha(Path(runner.__file__))
    final_rows = []
    development_rows = []
    event_pairs = 0
    for group in complete["groups"]:
        name = group["name"]
        summary_path = runner.ROOT / group["summary_path"]
        assert runner.sha(summary_path) == group["summary_sha256"]
        summary = runner.read(summary_path)
        lock_path = out / f"{name}_lock.json"
        assert runner.sha(lock_path) == summary["lock_file_sha256"] == declaration["controller_lock_sha256"][name]
        lock = runner.read(lock_path)
        base, _ = runner.lock_parameters(lock)
        records_path = runner.ROOT / summary["records_path"]
        assert runner.sha(records_path) == summary["records_sha256"]
        rows = runner.journal_rows(records_path)
        assert len(rows) == len({r["id"] for r in rows}) == 54
        assert {runner.object_sha(r["case"]) for r in rows} == {runner.object_sha(c) for c in runner.nominal_cases(spec)}
        assert runner.summarize(rows) == summary["summary"] == group["summary"]
        for row in rows:
            p, resolved = runner.resolve_record(lock_path, row["id"])
            assert row == resolved
            assert runner.object_sha(p) == row["config_sha256"]
            event_pairs += check_metrics(row, p)
        final_rows.extend(rows)
        if lock["development"] is not None:
            dev = lock["development"]
            dev_decl_path = runner.ROOT / dev["declaration_path"]
            dev_records_path = runner.ROOT / dev["records_path"]
            assert runner.sha(dev_decl_path) == dev["declaration_sha256"]
            assert runner.sha(dev_records_path) == dev["records_sha256"]
            dev_decl = runner.read(dev_decl_path)
            assert dev_decl["runner_sha256"] == runner.sha(Path(runner.__file__))
            assert len(dev_decl["families"]) == 18 and len(dev_decl["cases"]) == 4
            rows = runner.journal_rows(dev_records_path)
            assert len(rows) == len({r["id"] for r in rows}) == 72
            assert {r["case"]["seed"] for r in rows} == {spec["development"]["noise_seed"]}
            assert not {r["case"]["seed"] for r in rows} & set(spec["evaluation"]["noise_seeds"])
            by_family = {ix: [r for r in rows if r["family"] == ix] for ix in range(18)}
            assert all(len(v) == 4 for v in by_family.values())
            selected_family = min(by_family, key=lambda ix: (*runner.selection_score(by_family[ix]), ix))
            assert selected_family == dev["selected_family"]
            assert lock["controller_delta"] == dev_decl["families"][selected_family]
            for row in rows:
                p = runner.parameters_for_case(runner.root_config(lock["actuator"], "no_pad", spec,
                    dev_decl["families"][row["family"]]), row["case"])
                assert runner.object_sha(p) == row["config_sha256"]
                event_pairs += check_metrics(row, p)
            development_rows.extend(rows)
    assert runner.summarize(final_rows) == complete["final_summary"]
    assert runner.summarize(development_rows) == complete["development_summary"]
    assert len(final_rows) == 324 and len(development_rows) == 144
    for bank in selected["banks"]:
        lock_path = runner.ROOT / bank["controller_lock_path"]
        assert runner.sha(lock_path) == bank["controller_lock_sha256"]
        p, _ = runner.lock_parameters(runner.read(lock_path))
        assert p == bank["config"] and runner.object_sha(p) == bank["config_sha256"]
    assert len(complete["native_representatives"]) == 4
    assert not (out / "tmp").exists()
    result = {"status": "passed_without_new_simulations", "qualification": False,
              "matrix_completion_sha256": runner.sha(complete_path),
              "selected_banks_sha256": runner.sha(selected_path),
              "audit_recipe_sha256": runner.sha(Path(__file__)),
              "exact_final_records": len(final_rows), "exact_development_records": len(development_rows),
              "force_limit_pairs_verified": event_pairs,
              "integration_endpoints_scored": complete["final_summary"]["integration_endpoints_scored"]+
                                             complete["development_summary"]["integration_endpoints_scored"],
              "additional_one_sided_limits_scored": complete["final_summary"]["one_sided_extra_limits_scored"]+
                                                    complete["development_summary"]["one_sided_extra_limits_scored"],
              "seed_separation_and_full_identity_coverage": True,
              "all_failed_stopped_and_rating_violating_trials_retained": True,
              "source_parameters_metrics_locks_and_ranking_bound": True,
              "old_source_contract_api_hashes_preserved": True,
              "private_builds_removed": True, "new_bulk_arrays": 0,
              "storage": runner.budget_check()}
    # This is a regenerated verification receipt, not a frozen scientific input.
    (out / "completion_audit.json").write_text(json.dumps(result, sort_keys=True,
        separators=(",", ":"), allow_nan=False)+"\n")
    runner.budget_check()
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))


if __name__ == "__main__":
    main()
