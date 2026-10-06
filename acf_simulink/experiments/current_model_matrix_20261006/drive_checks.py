"""Bounded voltage/current-response diagnostics on the four frozen banks."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import itertools
import json
from pathlib import Path

from engine_api import compile_engine, cleanup_engine, simulate

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / "acf_simulink/validation/current_model_matrix_20261006/report"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    contract_path = HERE / "drive_checks_contract.json"
    contract = json.loads(contract_path.read_text())
    lock_path = ROOT / "acf_simulink/validation/current_model_matrix_20261006/control/selected_banks.json"
    lock = json.loads(lock_path.read_text())
    assert lock["engine_source_sha256"] == sha(HERE / "engine.c")
    if "api_sha256" in lock:
        assert lock["api_sha256"] == sha(HERE / "engine_api.py")
    assert len(lock["banks"]) == 4
    jobs = list(itertools.product(lock["banks"], contract["corners"], contract["variants"]))
    assert len(jobs) == contract["maximum_executed_records"] == 36
    fields = ["steps", "stop_reason", "completed", "force_pass", "force_peak_error",
              "force_peak_time", "force_rms_error", "current_peak", "current_rms",
              "copper_loss_average_W", "voltage_peak_request", "voltage_clipped_fraction",
              "current_clipped_fraction", "actuator_ratings_pass", "sensor_ratings_pass",
              "continuous_rms_current_pass", "force_balance_peak",
              "one_sided_extra_limit_count"]
    lib = compile_engine()
    def run(job):
        bank, corner, variant = job
        config = {**bank["config"], **corner,
                  "identified_mass_s": corner["mass_s"], "seed": contract["seed"],
                  "voltage_limit": variant["voltage_limit"],
                  "current_response": variant["current_response"]}
        metrics, trace = simulate(lib, config)
        assert trace.shape[0] == 0
        return {"bank": bank["id"], "corner": corner, "variant": variant["id"],
                "config_sha256": hashlib.sha256(json.dumps(config, sort_keys=True).encode()).hexdigest(),
                "metrics": {k: metrics[k] for k in fields if k in metrics}}
    try:
        with ThreadPoolExecutor(max_workers=4) as pool:
            rows = list(pool.map(run, jobs))
    finally:
        cleanup_engine(lib)
    summaries = []
    for bank in lock["banks"]:
        for variant in contract["variants"]:
            group = [r["metrics"] for r in rows if r["bank"] == bank["id"] and r["variant"] == variant["id"]]
            summaries.append({"bank": bank["id"], "variant": variant["id"],
                              "executed": len(group), "force_passes": sum(m["force_pass"] for m in group),
                              "worst_error_N": max(m["force_peak_error"] for m in group),
                              "max_voltage_request_V": max(m["voltage_peak_request"] for m in group),
                              "max_voltage_clipped_fraction": max(m["voltage_clipped_fraction"] for m in group)})
    result = {"status": "completed_fixed_controller_drive_diagnostics",
              "contract_sha256": sha(contract_path), "selected_banks_sha256": sha(lock_path),
              "engine_source_sha256": sha(HERE / "engine.c"), "api_sha256": sha(HERE / "engine_api.py"),
              "executed_record_count": len(rows),
              "scored_force_value_count": int(sum(r["metrics"]["steps"] for r in rows)),
              "additional_one_sided_force_limits": sum(r["metrics"].get("one_sided_extra_limit_count", 0) for r in rows),
              "scored_integration_endpoint_count": int(sum(r["metrics"]["steps"] - r["metrics"].get("one_sided_extra_limit_count", 0) for r in rows)),
              "baseline_note": "Twelve32V/50us points repeat acquisition baseline inputs; they are separately executed paired diagnostics, not twelve new unique conditions.",
              "records": rows, "summaries": summaries}
    OUT.mkdir(exist_ok=True)
    path = OUT / "drive_diagnostics.json"
    path.write_text(json.dumps(result, separators=(",", ":")) + "\n")
    assert path.stat().st_size <= contract["retained_budget_bytes"]
    print(json.dumps({"status": "completed", "records": len(rows), "bytes": path.stat().st_size}))


if __name__ == "__main__":
    main()
