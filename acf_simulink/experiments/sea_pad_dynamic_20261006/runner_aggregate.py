"""Audit retained compact records without launching any simulations."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from engine_api import HERE, OUT, ROOT
from runner_study import VALIDATION, dump


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path):
    return json.loads(path.read_text())


def rows_from(name: str, data) -> list[dict]:
    if isinstance(data, list):
        return data
    if name.endswith("exact_width_pairs.json") or name.endswith("pad5mm_exact_width_pairs.json"):
        return [r["point_record"] for r in data["pairs"]]
    if name == "closed_loop_numerical_refinement.json":
        return [r for pair in data["records"] for r in pair["records"]]
    if name == "voltage_limit_pairs.json":
        return [{"id":r["group"]+suffix,"status":"executed","config":r["config"],"metrics":r[key]}
                for r in data["records"] for suffix,key in [("_32V","metrics_32V"),("_48V","metrics_48V")]]
    return data["records"]


def metrics_summary(rows: list[dict]) -> dict:
    executed = [r for r in rows if "metrics" in r]
    m = [r["metrics"] for r in executed]
    return {
        "record_count":len(rows),
        "executed_record_count":len(executed),
        "preparation_rejected_count":sum(r.get("status")=="preparation_rejected" for r in rows),
        "other_unexecuted_count":sum("metrics" not in r and r.get("status")!="preparation_rejected" for r in rows),
        "force_pass_count":sum(bool(v["force_pass"]) for v in m),
        "completed_count":sum(bool(v["completed"]) for v in m),
        "physical_stops":dict(sorted(Counter(v["stop_reason"] for v in m if not v["completed"]).items())),
        "force_worst_error_N":max((v["force_peak_error"] for v in m),default=None),
        "scored_integration_endpoint_count":sum(round(v["steps"]) for v in m),
        "sensor_ratings_checked_count":sum("sensor_ratings_pass" in v for v in m),
        "sensor_ratings_failed_count":sum(v.get("sensor_ratings_pass") is False for v in m),
        "actuator_ratings_checked_count":sum("actuator_ratings_pass" in v for v in m),
        "actuator_ratings_failed_count":sum(v.get("actuator_ratings_pass") is False for v in m),
    }


def source_info(path: Path, category: str) -> tuple[dict,list[dict]]:
    data=read(path); rows=rows_from(path.name,data)
    source_hash=None
    if isinstance(data,dict):
        source_hash=data.get("engine_source_sha256",data.get("source_sha256"))
        source_hash=source_hash or data.get("candidate_lock",{}).get("engine_source_sha256")
    info={"path":str(path.relative_to(ROOT)),"sha256":sha(path),"category":category,
          "recorded_engine_source_sha256":source_hash,
          "source_binding_status":"recorded_source_hash" if source_hash else "historical_development_no_complete_source_hash_recorded",
          **metrics_summary(rows)}
    return info,rows


def grid_summary(path: Path) -> dict:
    data=read(path); rows=[r for r in data["records"] if "metrics" in r]
    motors=[r for r in rows if r["config"]["actuator"]!=0]
    return {"source":str(path.relative_to(ROOT)),**metrics_summary(rows),
            "maximum_current_rms_A":max(r["metrics"]["current_rms"] for r in rows),
            "maximum_copper_loss_W":max(r["metrics"]["copper_loss_average_W"] for r in rows),
            "motor_continuous_rms_checked_count":len(motors),
            "motor_continuous_rms_failed_count":sum(r["metrics"]["current_rms"]>r["config"]["continuous_current"] for r in motors),
            "maximum_unclipped_voltage_request_V":max(r["metrics"]["voltage_peak_request"] for r in rows),
            "minimum_contact_force_N":min(r["metrics"]["force_min"] for r in rows),
            "maximum_tool_absolute_deviation_m":max(r["metrics"]["tool_peak_deviation"] for r in rows),
            "maximum_spring_signed_deflection_abs_m":max(r["metrics"]["spring_peak_deflection"] for r in rows),
            "maximum_pad_compression_m":max(r["metrics"]["pad_peak_compression"] for r in rows)}


def paired_width_summary(data: dict) -> list[dict]:
    result=[]
    for name in sorted({r["group"] for r in data["pairs"]}):
        rows=[r for r in data["pairs"] if r["group"]==name]
        result.append({"group":name,"pair_count":len(rows),
            "point_force_pass_count":sum(r["point_record"]["metrics"]["force_pass"] for r in rows),
            "wide_force_pass_count":sum(r["wide_metrics"]["force_pass"] for r in rows),
            "point_physical_stops":dict(Counter(r["point_record"]["metrics"]["stop_reason"] for r in rows
                                               if not r["point_record"]["metrics"]["completed"])),
            "point_worst_error_N":max(r["point_record"]["metrics"]["force_peak_error"] for r in rows),
            "wide_worst_error_N":max(r["wide_metrics"]["force_peak_error"] for r in rows)})
    return result


def main():
    parser=argparse.ArgumentParser(); parser.add_argument("--original-only",action="store_true"); args=parser.parse_args()
    sources=[]; categories={}; all_primary=[]
    development=[p for p in sorted((VALIDATION/"development").glob("*.json")) if not p.name.endswith("_lock.json")]
    evaluation=[p for p in sorted((VALIDATION/"evaluation").glob("*.json"))
                if "_range_replay" not in p.name and p.name!="range_replay_summary.json"
                and not p.name.endswith("_metadata.json") and (not args.original_only or "pad5mm" not in p.name)]
    ranges=sorted((VALIDATION/"evaluation").glob("*_range_replay.json"))
    for category,paths in [("primary_development",development),("primary_evaluation_and_diagnostics",evaluation),
                           ("observation_only_instrumentation_reruns",ranges)]:
        records=[]
        for path in paths:
            info,rows=source_info(path,category); sources.append(info); records.extend(rows)
        categories[category]=metrics_summary(records)
        if category.startswith("primary"):all_primary.extend(records)
    numeric=read(VALIDATION/"evaluation/closed_loop_numerical_refinement.json")
    width=read(VALIDATION/"evaluation/exact_width_pairs.json")
    width_summary=paired_width_summary(width)
    forecast_path=VALIDATION/"storage_forecast.json"; forecast=read(forecast_path)
    owned_bytes={folder:sum(p.stat().st_size for p in (VALIDATION/folder).rglob("*") if p.is_file())
                 for folder in ("engine","development","evaluation")}
    total=sum(owned_bytes.values()); cap=forecast["allocations"]["engine"]["retained_bytes"]
    if total>cap:raise RuntimeError(f"Retained engine allocation exceeded: {total}>{cap}")
    grid_paths=[p for p in evaluation if isinstance(read(p),dict) and "planned_count" in read(p)]
    result={"schema_version":1,"status":"completed_compact_record_audit",
        "recorded_utc":datetime.now(timezone.utc).isoformat(),
        "scope":"Reduced-scope total axial pressing force; drag ignored, robot fixed,2s loaded equilibrium. No local pressure or hardware qualification.",
        "original_only":args.original_only,"current_engine_source_sha256":sha(HERE/"engine.c"),
        "current_api_sha256":sha(HERE/"engine_api.py"),"base_contract_sha256":sha(HERE/"study_contract.json"),
        "categories":categories,"all_primary_recordings":metrics_summary(all_primary),
        "count_method":"Retained executed records, not unique parameter points. Duplicate observations and presentation reruns are excluded from primary counts. Voltage and refinement pair members count separately; reused wide records do not count again.",
        "known_presentation_reruns":{"original_point_worst":4,"quiet_point_worst":2,"wide_worst":4,
            "paired_factor_traces":6,"native_command_capture":3,"partial_unilateral_native_command_capture":1,
            "exact_width_illustrations":4,**({"pad5mm_selected_worst_command_and_trace_capture":2} if not args.original_only and any("pad5mm" in p.name for p in evaluation) else {})},
        "known_software_rejected_attempts":[{"count":1,"phase":"initial coil pilot output serialization",
            "reason":"A numpy.bool scalar prevented JSON serialization after the simulation completed; corrected and rerun. Failed output attempt has no retained distinct scientific record and is excluded from primary counts."}],
        "locked_grid_summaries":[grid_summary(p) for p in grid_paths],
        "exact_original_width_pairs":width_summary,
        "unchanged_voltage_pairs":[{"group":r["group"],
            "peak_error_32V_N":r["metrics_32V"]["force_peak_error"],
            "peak_error_48V_N":r["metrics_48V"]["force_peak_error"],
            "force_pass_32V":r["metrics_32V"]["force_pass"],"force_pass_48V":r["metrics_48V"]["force_pass"],
            "recorded_1kHz_trace_bitwise_identical":r["recorded_1kHz_trace_bitwise_identical"]}
            for r in read(VALIDATION/"evaluation/voltage_limit_pairs.json")["records"]],
        "numerical_refinement":{"noise_disabled":True,"execution_count":numeric["execution_count"],
            "max_spatial_peak_difference_N":numeric["max_spatial_peak_difference_N"],
            "max_temporal_peak_difference_N":numeric["max_temporal_peak_difference_N"]},
        "storage":{"owned_compact_bytes_by_folder_before_this_manifest":owned_bytes,"total_bytes_before_this_manifest":total,
            "engine_retained_limit_bytes":cap,"operational_forecast_sha256":sha(forecast_path),
            "bulk_array_files_created":0,"temporary_physical_replay_arrays":"RAM only; native worker owns independent budget and evidence"},
        "limitations":["Plant replay parity does not independently validate controller/acquisition code.",
            "Pure ADC group-delay transport is not an exact SINC3 frequency-response model.",
            "Constant Kelvin-Voigt pad values and 50% compression guard are engineering assumptions, not measured pad specifications.",
            "Continuous motor RMS eligibility is separate from short-run force passes; coil thermal catalog limit remains unspecified.",
            "force_balance_peak is an algebraic consistency identity, not independent validation.",
            "Scored endpoints include ADC/pipeline event subdivisions and are not a native Simulink sample count."]}
    if not args.original_only and (VALIDATION/"evaluation/pad5mm_exact_width_pairs.json").exists():
        result["pad5mm_appendix"]={"extension_contract_sha256":sha(HERE/"pad5mm_contract.json"),
            "prelaunch_record":str((OUT/"pad5mm_prelaunch.json").relative_to(ROOT)),
            "declared_forecast_hash":read(HERE/"pad5mm_contract.json")["storage_forecast_sha256"],
            "operational_forecast_hash":sha(forecast_path),
            "exact_width_pairs":paired_width_summary(read(VALIDATION/"evaluation/pad5mm_exact_width_pairs.json")),
            "guard_corners":metrics_summary(read(VALIDATION/"evaluation/pad5mm_k20_guard_corners.json")["records"]),
            "guard_interpretation":"Half-thickness guard is an assumed linear-material validity boundary, not measured manufacturer bottoming."}
    name="original_study_summary.json" if args.original_only else "final_summary.json"
    dump(OUT/name,result)
    dump(OUT/("original_evidence_manifest.json" if args.original_only else "evidence_manifest.json"),
         {"schema_version":1,"summary_path":str((OUT/name).relative_to(ROOT)),"summary_sha256":sha(OUT/name),"sources":sources,
          "current_recipe_hashes":{p.name:sha(p) for p in sorted(HERE.glob("runner*.py"))},
          "source_snapshot_hashes":{p.name:sha(p) for p in sorted(OUT.glob("engine_source_*.c"))}})
    print(json.dumps({"primary":result["all_primary_recordings"],"categories":categories,"storage":result["storage"]},indent=2))


if __name__=="__main__":main()
