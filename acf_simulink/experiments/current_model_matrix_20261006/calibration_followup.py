"""Fixed-bank calibration sensitivities prompted by the +1% mass failure."""
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
    contract_path = HERE / "calibration_followup_contract.json"
    contract = json.loads(contract_path.read_text())
    lock_path = ROOT / "acf_simulink/validation/current_model_matrix_20261006/control/selected_banks.json"
    lock = json.loads(lock_path.read_text())
    assert lock["engine_source_sha256"] == sha(HERE / "engine.c")
    assert lock["api_sha256"] == sha(HERE / "engine_api.py")
    jobs = list(itertools.product(lock["banks"], contract["corners"], contract["variants"]))
    assert len(jobs) == contract["maximum_executed_records"] == 120
    fields = ["steps", "stop_reason", "completed", "force_pass", "force_peak_error",
              "force_peak_time", "force_rms_error", "force_reconstruction_peak_error",
              "current_rms", "sensor_ratings_pass", "actuator_ratings_pass",
              "continuous_rms_current_pass", "one_sided_extra_limit_count"]
    lib = compile_engine()
    def run(job):
        bank, corner, variant = job
        config = {**bank["config"], **corner, **variant["deltas"],
                  "identified_mass_s": corner["mass_s"]*variant["identified_mass_factor"],
                  "seed": contract["seed"]}
        metrics, trace = simulate(lib, config)
        assert not len(trace)
        eligible = (metrics["force_pass"] and metrics["sensor_ratings_pass"]
                    and metrics["actuator_ratings_pass"]
                    and metrics["continuous_rms_current_pass"] is not False)
        return {"bank": bank["id"], "corner": corner, "variant": variant["id"],
                "eligible_pass": bool(eligible),
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
            group = [r for r in rows if r["bank"] == bank["id"] and r["variant"] == variant["id"]]
            summaries.append({"bank": bank["id"], "variant": variant["id"],
                              "executed": len(group), "eligible_passes": sum(r["eligible_pass"] for r in group),
                              "worst_error_N": max(r["metrics"]["force_peak_error"] for r in group),
                              "sensor_range_failures": sum(not r["metrics"]["sensor_ratings_pass"] for r in group)})
    result = {"status": "completed_followup_calibration_diagnostics_not_new_qualification",
              "contract_sha256": sha(contract_path), "selected_banks_sha256": sha(lock_path),
              "engine_source_sha256": sha(HERE / "engine.c"), "api_sha256": sha(HERE / "engine_api.py"),
              "executed_record_count": len(rows),
              "scored_force_value_count": int(sum(r["metrics"]["steps"] for r in rows)),
              "additional_one_sided_force_limits": sum(r["metrics"].get("one_sided_extra_limit_count", 0) for r in rows),
              "scored_integration_endpoint_count": int(sum(r["metrics"]["steps"] - r["metrics"].get("one_sided_extra_limit_count", 0) for r in rows)),
              "adaptive_note": "Triggered by known development diagnostics; no retuning and no claim of independent qualification.",
              "records": rows, "summaries": summaries}
    OUT.mkdir(exist_ok=True)
    path = OUT / "calibration_followup.json"
    path.write_text(json.dumps(result, separators=(",", ":"))+"\n")
    assert path.stat().st_size <= contract["retained_budget_bytes"]
    print(json.dumps({"status": "completed", "records": len(rows), "bytes": path.stat().st_size}))


if __name__ == "__main__":
    main()
