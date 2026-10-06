"""Bounded fixed-controller acquisition diagnostics, without bulk retention."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from contextlib import contextmanager
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys
import tempfile
import time


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OUT = ROOT / "acf_simulink/validation/current_model_matrix_20261006/acquisition"
CONTRACT = HERE / "diagnostic_contract.json"
sys.dont_write_bytecode = True
EVENT_COLUMNS = ["time_s", "force_left_N", "force_right_N", "acceleration_left_m_s2",
                 "acceleration_right_m_s2", "continuous_cell_raw_N", "first_invalid_side", "scored_side_count"]
EVENT_CONVENTION = "right-continuous ordinary endpoint; additional left limit; first invalid contact side stops at that time"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> dict:
    return json.loads(path.read_text())


def dump(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(data, allow_nan=False, separators=(",", ":")) + "\n"
    owned_bytes = sum(p.stat().st_size for folder in (HERE, OUT)
                      if folder.exists() for p in folder.rglob("*") if p.is_file())
    previous_bytes = path.stat().st_size if path.exists() else 0
    limit = read(CONTRACT)["retained_cap_bytes"]
    if owned_bytes - previous_bytes + len(encoded.encode()) > limit:
        raise RuntimeError("Acquisition source and retained evidence exceed the assigned cap.")
    path.write_text(encoded)


def load_api(spec: dict):
    path = ROOT / spec["engine_api"]
    if sha(path) != spec["engine_api_sha256"]:
        raise RuntimeError("The declared engine API changed.")
    module_spec = importlib.util.spec_from_file_location("acquisition_frozen_engine_api", path)
    module = importlib.util.module_from_spec(module_spec)
    module_spec.loader.exec_module(module)
    return module


@contextmanager
def private_engine(spec: dict):
    """Use a temporary frozen source and build; leave historical OUT untouched."""
    source = ROOT / spec["engine_source"]
    if sha(source) != spec["engine_source_sha256"]:
        raise RuntimeError("The declared immutable engine source changed.")
    if sha(ROOT / spec["matrix_contract"]) != spec["matrix_contract_sha256"]:
        raise RuntimeError("The root matrix contract changed.")
    forecast = read(ROOT / spec["storage_forecast"])
    allocation = forecast["allocations"]["acquisition"]
    if allocation["retained"] < spec["retained_cap_bytes"] or allocation["temporary"] < spec["temporary_cap_bytes"]:
        raise RuntimeError("Contract exceeds the root acquisition allocation.")
    if forecast["shared_array_audit"]["bytes"] > forecast["shared_array_limit_bytes"]:
        raise RuntimeError("Root prelaunch array audit exceeds the shared array budget.")
    if shutil.disk_usage("/tmp").free < spec["temporary_cap_bytes"]:
        raise RuntimeError("Insufficient private build space.")
    if shutil.disk_usage(ROOT).free < spec["retained_cap_bytes"]:
        raise RuntimeError("Insufficient compact evidence space.")
    with tempfile.TemporaryDirectory(prefix="acf_matrix_acquisition_") as folder:
        temporary = Path(folder)
        (temporary / "engine.c").write_bytes(source.read_bytes())
        api = load_api(spec)
        api.HERE = temporary
        api.OUT = temporary / "build"
        library = api.compile_engine()
        used = sum(p.stat().st_size for p in temporary.rglob("*") if p.is_file())
        binary_directory = getattr(library, "_engine_private_directory", None)
        if binary_directory is not None:
            used += sum(p.stat().st_size for p in Path(binary_directory.name).rglob("*") if p.is_file())
        if used > spec["temporary_cap_bytes"]:
            raise RuntimeError("Private acquisition build exceeds its forecast.")
        build = read(api.OUT / "build.json")
        build["temporary_bytes"] = used
        build["retained_arrays"] = 0
        try:
            yield api, library, build
            if sha(source) != spec["engine_source_sha256"]:
                raise RuntimeError("Engine source changed during the fixed-source run.")
            if sha(ROOT / spec["engine_api"]) != spec["engine_api_sha256"]:
                raise RuntimeError("Engine API changed during the fixed-source run.")
        finally:
            if hasattr(api, "cleanup_engine"):
                api.cleanup_engine(library)


def config_digest(config: dict) -> str:
    return hashlib.sha256(json.dumps(config, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def preparation_rejection(p: dict) -> str | None:
    """Same loaded-equilibrium guards as the frozen runner_study.py."""
    g = p["gravity"] * math.cos(math.radians(p["angle_deg"]))
    fa = p["force_target"] - (p["mass_b"] + p["mass_s"]) * g
    zs = 0 if p["pad_k"] == 0 else -p["force_target"] / p["pad_k"]
    zb = zs - (p["force_target"] - p["mass_s"] * g) / p["cell_k"]
    za = (fa - (p["mass_a"] + (p["nut_mass"] if p["actuator"] else 0)) * g) / p["upper_k"]
    if abs(za - zb) > p["tool_travel_limit"]:
        return "initial_flange_stroke"
    if p["pad_compression_limit"] and -zs > p["pad_compression_limit"]:
        return "initial_pad_compression"
    if p["actuator"]:
        q = za - zb + fa / p["sea_k"]
        if abs(q) > p["nut_travel_limit"]:
            return "initial_nut_stroke"
        if abs(fa / p["sea_k"]) > p["spring_deflection_limit"]:
            return "initial_spring_deflection"
    current = fa / p["force_constant"] if not p["actuator"] else (fa - p["nut_mass"] * g) * p["lead"] / (2 * math.pi * p["force_constant"])
    if abs(current) > p["current_limit"]:
        return "initial_peak_current"
    if abs(current * p["resistance"]) > p["voltage_limit"]:
        return "initial_voltage"
    return None


def run_config(api, library, config: dict) -> dict:
    rejection = preparation_rejection(config)
    if rejection:
        return {"status": "preparation_rejected", "reason": rejection}
    metrics, trace = api.simulate(library, config, trace_rate=0)
    if len(trace):
        raise AssertionError("Trace retention must stay disabled.")
    metrics["force_criterion_pass"] = metrics["force_peak_error"] <= 0.5
    metrics["physical_pass"] = metrics["completed"]
    metrics["eligible_pass"] = bool(metrics["force_pass"] and metrics["actuator_ratings_pass"] and
                                    metrics["sensor_ratings_pass"] and
                                    metrics["continuous_rms_current_pass"] is not False)
    if "event_force_limits" in metrics:
        metrics["event_force_limit_values"] = [[event[name] for name in EVENT_COLUMNS]
                                               for event in metrics.pop("event_force_limits")]
        if metrics.pop("event_endpoint_convention") != EVENT_CONVENTION:
            raise RuntimeError("Rigid-event scoring convention changed.")
    return {"status": "executed", "metrics": metrics}


def pilot(spec: dict) -> None:
    rows = []
    with private_engine(spec) as (api, library, build):
        for actuator in ("coil", "sea"):
            path = ROOT / f"acf_simulink/validation/sea_pad_dynamic_20261006/evaluation/{actuator}_pad5mm_k50_c0.json"
            reference = read(path)["records"][0]
            start = time.monotonic()
            actual = run_config(api, library, reference["config"])
            differences = {}
            for field, expected in reference["metrics"].items():
                if field == "runtime_s":
                    continue
                observed = actual.get("metrics", {}).get(field)
                if observed != expected:
                    differences[field] = {"expected": expected, "observed": observed}
            row = {"id": reference["id"], "source_file": str(path.relative_to(ROOT)),
                   "source_sha256": sha(path), "parameter_sha256": config_digest(reference["config"]),
                   "all_nonruntime_metrics_identical": not differences, "differences": differences,
                   "wall_runtime_s": time.monotonic() - start,
                   "metrics": actual.get("metrics"), "status": actual["status"]}
            rows.append(row)
            print(f"PILOT {actuator}: parity={not differences}, runtime={row['wall_runtime_s']:.3f}s", flush=True)
        result = {"status": "passed" if all(r["all_nonruntime_metrics_identical"] for r in rows) else "failed",
                  "scope": "Frozen-source and runner repeatability, not independent native controller verification.",
                  "engine_source_sha256": spec["engine_source_sha256"], "build": build, "records": rows,
                  "engine_api_sha256": spec["engine_api_sha256"],
                  "contract_sha256": sha(CONTRACT),
                  "scored_endpoints": sum(r["metrics"]["steps"] for r in rows)}
        dump(OUT / "pilot_parity.json", result)
    if result["status"] != "passed":
        raise RuntimeError("Retain failed pilot parity; do not proceed to diagnostic matrix.")


def lookup(value: dict, key: str):
    for part in key.split("."):
        value = value[part]
    return value


def read_banks(path: Path, spec: dict, api) -> tuple[list, dict]:
    manifest = read(path)
    if manifest["engine_source_sha256"] != spec["engine_source_sha256"]:
        raise RuntimeError("Bank and acquisition source hashes differ.")
    if manifest.get("api_sha256", spec["engine_api_sha256"]) != spec["engine_api_sha256"]:
        raise RuntimeError("Bank and acquisition API hashes differ.")
    if manifest.get("contract_sha256", spec["matrix_contract_sha256"]) != spec["matrix_contract_sha256"]:
        raise RuntimeError("Bank and acquisition matrix contract hashes differ.")
    banks = []
    for record in manifest["banks"]:
        if "config_file" in record:
            config_path = ROOT / record["config_file"]
            expected = record.get("config_sha256")
            if expected and sha(config_path) != expected:
                raise RuntimeError(f"Frozen bank file changed: {config_path}")
            container = read(config_path)
            config = dict(lookup(container, record.get("config_key", "base_config")))
            gains_key = record.get("gains_key")
            if gains_key:
                config.update(lookup(container, gains_key))
            source = {"file": str(config_path.relative_to(ROOT)), "sha256": sha(config_path)}
        else:
            config = dict(record["config"])
            source = {"file": str(path.relative_to(ROOT)), "sha256": sha(path), "bank_id": record["id"]}
        api.make_parameters(config)
        if config["robot_enabled"] or config["external_current"] or config["external_robot_amplitude"]:
            raise RuntimeError("Banks must use fixed robot and their causal controller.")
        if config["analog_cutoff"] != 1900 or config["adc_rate"] != 30000 or config["accel_noise_rms"] != 0.014:
            raise RuntimeError("Diagnostic baseline assumptions differ from the selected bank.")
        banks.append({"id": record["id"], "config": config, "source": source,
                      "parameter_sha256": config_digest(config)})
    if len(banks) != spec["banks_required"] or len({b["id"] for b in banks}) != len(banks):
        raise RuntimeError("Require exactly four uniquely identified selected banks.")
    if sorted((int(b["config"]["actuator"]), b["config"]["pad_k"] == 0) for b in banks) != [(0, False), (0, True), (1, False), (1, True)]:
        raise RuntimeError("Banks must cover both actuators with and without the soft pad.")
    return banks, {"file": str(path.relative_to(ROOT)), "sha256": sha(path)}


def case_config(bank: dict, corner: dict, variant: dict, spec: dict) -> tuple[dict, dict]:
    config = dict(bank["config"])
    config.update({k: v for k, v in corner.items() if k != "id"})
    config.update(identified_mass_s=corner["mass_s"], seed=spec["fresh_noise_seed"])
    config.update(variant.get("deltas", {}))
    if "bits" in variant:
        config.update(force_quantum=320 / 2 ** variant["bits"], accel_quantum=235.3596 / 2 ** variant["bits"])
    if "adc_rate" in variant:
        config.update(adc_rate=variant["adc_rate"], pipeline_time=20e-6 + 49.6e-6 * 30000 / variant["adc_rate"])
    if "identified_mass_factor" in variant:
        config["identified_mass_s"] = corner["mass_s"] * variant["identified_mass_factor"]
    deltas = {k: v for k, v in config.items() if v != bank["config"][k]}
    return config, deltas


def group_summary(rows: list) -> dict:
    executed = [r for r in rows if r["status"] == "executed"]
    metrics = [r["metrics"] for r in executed]
    worst = max(executed, key=lambda r: r["metrics"]["force_peak_error"]) if executed else None
    return {"planned": len(rows), "executed": len(executed), "preparation_rejected": len(rows) - len(executed),
            "force_criterion_pass": sum(m["force_criterion_pass"] for m in metrics),
            "force_and_physical_pass": sum(m["force_pass"] for m in metrics),
            "eligible_pass": sum(m["eligible_pass"] for m in metrics),
            "physical_stops": sum(not m["physical_pass"] for m in metrics),
            "actuator_range_failures": sum(not m["actuator_ratings_pass"] for m in metrics),
            "sensor_range_failures": sum(not m["sensor_ratings_pass"] for m in metrics),
            "continuous_rms_current_failures": sum(m["continuous_rms_current_pass"] is False for m in metrics),
            "scored_force_values": sum(m["steps"] for m in metrics),
            "scored_integration_endpoints": sum(m.get("scored_integration_endpoint_count", m["steps"]) for m in metrics),
            "additional_one_sided_force_limits": sum(m.get("one_sided_extra_limit_count", 0) for m in metrics),
            "worst_error_N": worst["metrics"]["force_peak_error"] if worst else None,
            "worst_id": worst["id"] if worst else None,
            "maximum_filtered_force_noise_rms_N": max((m["force_noise_filtered_rms"] for m in metrics), default=None),
            "maximum_filtered_accel_noise_rms_m_s2": max((m["accel_noise_filtered_rms"] for m in metrics), default=None)}


def matrix(spec: dict, manifest_path: Path, workers: int) -> None:
    if (OUT / "diagnostic_results.json").exists():
        raise RuntimeError("Preserve the completed matrix; use a separately versioned diagnostic study for new execution.")
    if not (OUT / "pilot_parity.json").exists() or read(OUT / "pilot_parity.json")["status"] != "passed":
        raise RuntimeError("Successful frozen-source pilot required before the main matrix.")
    pilot_source = read(OUT / "pilot_parity.json")
    if pilot_source["engine_source_sha256"] != spec["engine_source_sha256"] or pilot_source.get("engine_api_sha256") != spec["engine_api_sha256"]:
        raise RuntimeError("Pilot and declared matrix engine/API sources differ.")
    readiness_path = ROOT / spec["engine_readiness"]
    readiness = read(readiness_path)
    for field, expected in (("engine_source_sha256", spec["engine_source_sha256"]),
                            ("api_sha256", spec["engine_api_sha256"]),
                            ("contract_sha256", spec["matrix_contract_sha256"])):
        if readiness[field] != expected:
            raise RuntimeError("Released engine source and acquisition contract differ.")
    if readiness["status"] != "ready_for_bounded_development_and_native_replay" or sha(ROOT / readiness["checks_path"]) != readiness["checks_sha256"]:
        raise RuntimeError("Passed source-bound engine release checks required.")
    readiness_ref = {"file": str(readiness_path.relative_to(ROOT)), "sha256": sha(readiness_path)}
    with private_engine(spec) as (api, library, build):
        banks, manifest_ref = read_banks(manifest_path, spec, api)
        jobs = [(bank, corner, variant) for bank in banks for corner in spec["corners"] for variant in spec["variants"]]
        if len(jobs) != spec["planned_main_count"]:
            raise AssertionError("Contract and generated count differ.")
        dump(OUT / "prelaunch.json", {"status": "recorded_before_main_execution", "time_utc": datetime.now(timezone.utc).isoformat(),
             "contract_sha256": sha(CONTRACT), "banks": [{k: v for k, v in b.items() if k != "config"} for b in banks],
             "manifest": manifest_ref, "planned_count": len(jobs), "workers": workers,
             "engine_readiness": readiness_ref,
             "engine_source_sha256": spec["engine_source_sha256"], "build": build,
             "temporary_cap_bytes": spec["temporary_cap_bytes"], "retained_cap_bytes": spec["retained_cap_bytes"]})

        def run(job):
            bank, corner, variant = job
            config, deltas = case_config(bank, corner, variant, spec)
            identity = f"{bank['id']}__{corner['id']}__{variant['id']}"
            return {"id": identity, "bank_id": bank["id"], "corner_id": corner["id"],
                    "variant_id": variant["id"], "variant_group": variant["group"],
                    "parameter_deltas": deltas, "parameter_sha256": config_digest(config),
                    **run_config(api, library, config)}

        start = time.monotonic()
        rows = []
        with ThreadPoolExecutor(max_workers=workers) as pool:
            for index, row in enumerate(pool.map(run, jobs), 1):
                rows.append(row)
                if index % 12 == 0 or index == len(jobs):
                    metrics = row.get("metrics", {})
                    print(f"MATRIX {index}/{len(jobs)} {row['id']} {row['status']} "
                          f"error={metrics.get('force_peak_error', float('nan')):.6g}N elapsed={time.monotonic()-start:.1f}s", flush=True)
                    dump(OUT / "progress.json", {"status": "executing" if index < len(jobs) else "all_jobs_returned",
                                                 "returned": index, "planned": len(jobs), "wall_runtime_s": time.monotonic() - start})
        by_bank = {bank["id"]: group_summary([r for r in rows if r["bank_id"] == bank["id"]]) for bank in banks}
        by_variant = {variant["id"]: group_summary([r for r in rows if r["variant_id"] == variant["id"]]) for variant in spec["variants"]}
        by_bank_variant = {bank["id"]: {variant["id"]: group_summary([r for r in rows if r["bank_id"] == bank["id"] and r["variant_id"] == variant["id"]])
                                      for variant in spec["variants"]} for bank in banks}
        contact_by_bank = {b["id"]: "ideal_rigid_plate" if b["config"]["pad_k"] == 0 else "soft_5mm_pad" for b in banks}
        by_contact_variant = {contact: {variant["id"]: group_summary([r for r in rows if contact_by_bank[r["bank_id"]] == contact and r["variant_id"] == variant["id"]])
                                        for variant in spec["variants"]} for contact in ("soft_5mm_pad", "ideal_rigid_plate")}
        result = {"status": "executed_fixed_controller_acquisition_response_diagnostics", "contract_sha256": sha(CONTRACT),
                  "engine_source_sha256": spec["engine_source_sha256"], "manifest": manifest_ref,
                  "engine_readiness": readiness_ref,
                  "banks": [{k: v for k, v in b.items() if k != "config"} for b in banks], "records": rows,
                  "summary": group_summary(rows), "by_bank": by_bank, "by_variant": by_variant,
                  "by_bank_variant": by_bank_variant, "wall_runtime_s": time.monotonic() - start,
                  "by_contact_variant": by_contact_variant,
                  "all_integration_endpoints_scored": True, "loaded_equilibrium_startup_included": True,
                  "retained_arrays": 0, "untested": spec["untested"], "no_diagnostic_retuning": True,
                  "event_force_limit_columns": EVENT_COLUMNS, "event_scoring_convention": EVENT_CONVENTION,
                  "rigid_contact_interpretation": spec["rigid_contact_interpretation"],
                  "calibration_scope": spec["calibration_scope"],
                  "scope": spec["scope"]}
        dump(OUT / "diagnostic_results.json", result)
        baseline = {(r["bank_id"], r["corner_id"]): r for r in rows if r["variant_id"] == "baseline"}
        paired = []
        for row in rows:
            reference = baseline[row["bank_id"], row["corner_id"]]
            if row["status"] == "executed" and reference["status"] == "executed":
                paired.append({"id": row["id"], "baseline_id": reference["id"],
                               "peak_error_difference_N": row["metrics"]["force_peak_error"] - reference["metrics"]["force_peak_error"],
                               "eligible_pass": row["metrics"]["eligible_pass"],
                               "baseline_eligible_pass": reference["metrics"]["eligible_pass"]})
        dump(OUT / "paired_deltas.json", {"diagnostic_results_sha256": sha(OUT / "diagnostic_results.json"), "pairs": paired})
        print(json.dumps(result["summary"]), flush=True)


def verify(spec: dict) -> None:
    """Audit current input bindings, complete case identities and scored counters."""
    path = OUT / "diagnostic_results.json"
    result = read(path)
    if result["contract_sha256"] != sha(CONTRACT) or result["engine_source_sha256"] != spec["engine_source_sha256"]:
        raise AssertionError("Diagnostic source/contract binding changed.")
    manifest = result["manifest"]
    manifest_path = ROOT / manifest["file"]
    if sha(manifest_path) != manifest["sha256"]:
        raise AssertionError("Frozen selected-bank manifest changed.")
    readiness = result["engine_readiness"]
    if sha(ROOT / readiness["file"]) != readiness["sha256"]:
        raise AssertionError("Engine release binding changed.")
    api = load_api(spec)
    banks, _ = read_banks(manifest_path, spec, api)
    bank_map = {bank["id"]: bank for bank in banks}
    corner_map = {corner["id"]: corner for corner in spec["corners"]}
    variant_map = {variant["id"]: variant for variant in spec["variants"]}
    expected = {f"{bank['id']}__{corner['id']}__{variant['id']}" for bank in banks
                for corner in spec["corners"] for variant in spec["variants"]}
    rows = result["records"]
    if len(rows) != len(expected) or {row["id"] for row in rows} != expected:
        raise AssertionError("Diagnostic coverage or identity uniqueness failed.")
    for row in rows:
        config, deltas = case_config(bank_map[row["bank_id"]], corner_map[row["corner_id"]], variant_map[row["variant_id"]], spec)
        if config_digest(config) != row["parameter_sha256"] or deltas != row["parameter_deltas"]:
            raise AssertionError(f"Diagnostic parameter reconstruction failed: {row['id']}")
        if row["status"] != "executed":
            continue
        m = row["metrics"]
        completed = m["stop_code"] == 0
        if m["completed"] != completed or m["force_criterion_pass"] != (m["force_peak_error"] <= 0.5) or m["force_pass"] != (completed and m["force_peak_error"] <= 0.5):
            raise AssertionError(f"Force/physical accounting failed: {row['id']}")
        eligible = bool(m["force_pass"] and m["actuator_ratings_pass"] and m["sensor_ratings_pass"] and m["continuous_rms_current_pass"] is not False)
        if m["eligible_pass"] != eligible or m["steps"] < 1 or int(m["steps"]) != m["steps"]:
            raise AssertionError(f"Rating/sample accounting failed: {row['id']}")
        if completed and (m["adc_count"] != int(config["adc_rate"] * config["duration"]) + 1 or
                          m["control_count"] != int(config["control_rate"] * config["duration"]) + 1):
            raise AssertionError(f"Completed-record acquisition coverage failed: {row['id']}")
        if "event_force_limit_values" in m:
            if len(m["event_force_limit_values"]) != m["one_sided_extra_limit_count"] or m["scored_integration_endpoint_count"] + m["one_sided_extra_limit_count"] != m["steps"]:
                raise AssertionError(f"Rigid endpoint/limit accounting failed: {row['id']}")
    if result["summary"] != group_summary(rows):
        raise AssertionError("Top-level scored counters differ from individual records.")
    paired = read(OUT / "paired_deltas.json")
    if paired["diagnostic_results_sha256"] != sha(path):
        raise AssertionError("Matched diagnostic delta binding changed.")
    for name in ("pilot_parity.json", "pilot_parity_original_engine.json"):
        provenance = read(OUT / name)
        if sha(ROOT / provenance["contract_snapshot_file"]) != provenance["contract_sha256"]:
            raise AssertionError("Exact pre-pilot contract snapshot binding failed.")
    used = sum(p.stat().st_size for folder in (HERE, OUT) for p in folder.rglob("*") if p.is_file())
    value = {"status": "passed", "diagnostic_results_sha256": sha(path), "contract_sha256": sha(CONTRACT),
             "unique_case_count": len(rows), "parameter_deltas_reconstruct_exactly": True,
             "force_physical_rating_counts_separate": True, "completed_acquisition_counts_include_startup": True,
             "integration_endpoint_and_extra_limit_counts_verified": True, "pilot_contract_snapshots_hash_exact": True,
             "summary": result["summary"], "retained_bytes_before_audit": used,
             "retained_cap_bytes": spec["retained_cap_bytes"], "retained_arrays": 0,
             "scope": "Compact input/coverage/counter audit; no new simulation or hardware qualification."}
    dump(OUT / "audit.json", value)
    print(json.dumps(value), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("pilot", "matrix", "verify"))
    parser.add_argument("--banks", type=Path)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.workers <= 4:
        parser.error("Use one to four bounded workers.")
    spec = read(CONTRACT)
    if args.mode == "pilot":
        pilot(spec)
    elif args.mode == "verify":
        verify(spec)
    else:
        if args.banks is None:
            parser.error("matrix requires --banks pointing to the frozen four-bank manifest")
        matrix(spec, args.banks.resolve(), args.workers)


if __name__ == "__main__":
    main()
