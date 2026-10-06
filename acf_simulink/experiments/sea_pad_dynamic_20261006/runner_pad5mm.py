"""Append the user's 77 mm x5 mm geometry using already frozen gains."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import hashlib
import json
import shutil

import numpy as np

from engine_api import HERE, OUT, ROOT, compile_engine, simulate, write_trace
from runner_campaign import evaluated_config
from runner_study import VALIDATION, default_config, dump, execute


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    return json.loads(path.read_text())


def main():
    extension_path=HERE/"pad5mm_contract.json"; spec=load(extension_path)
    if sha(extension_path)!="674b1217279f8a9ca689f55afbcecbd6640b63cf45c4240f63b2549074a5efff":
        raise RuntimeError("The separately declared 5 mm contract changed.")
    if sha(HERE/"study_contract.json")!=spec["base_contract_sha256"]:
        raise RuntimeError("The frozen base study contract changed.")
    forecast_path=VALIDATION/"storage_forecast.json"; forecast=load(forecast_path)
    existing=sum(p.stat().st_size for folder in ("engine","development","evaluation")
                 for p in (VALIDATION/folder).rglob("*") if p.is_file())
    limit=forecast["allocations"]["engine"]["retained_bytes"]; retained_increment=2*1024**2
    if existing+retained_increment>limit:
        raise RuntimeError("The compact extension forecast does not fit the current engine allocation.")
    if shutil.disk_usage(VALIDATION).free<128*1024**2:
        raise RuntimeError("Insufficient temporary free space.")
    prelaunch=OUT/"pad5mm_prelaunch.json"
    if not prelaunch.exists():
        dump(prelaunch,{"status":"recorded_before_5mm_extension_launch","time_utc":datetime.now(timezone.utc).isoformat(),
            "extension_contract_sha256":sha(extension_path),
            "forecast_hash_declared_in_extension_contract":spec["storage_forecast_sha256"],
            "operational_forecast_sha256":sha(forecast_path),"operational_engine_limit_bytes":limit,
            "owned_evidence_bytes_before_launch":existing,"forecast_new_retained_upper_bound_bytes":retained_increment,
            "filesystem_free_bytes":shutil.disk_usage(VALIDATION).free,
            "temporary_peak_upper_bound_bytes":128*1024**2,"new_bulk_array_bytes":0,
            "planned_grid_runs":108,"planned_matched_point_runs":16,"planned_guard_corners":8,
            "planned_selected_trace_command_reruns":2})
    lib=compile_engine(); seeds=spec["planned_cases"]["nominal_axes"]["fresh_noise_seeds"]
    names=[("coil_pad5mm_k50_c0","coil_wide20k_c0_width_transfer"),
           ("sea_pad5mm_k50_c0","sea_v2_wide20k_c0_5k_l10_width_transfer")]
    groups=[]; replay_cases=[]
    for name,source in names:
        original=load(VALIDATION/"development"/f"{source}_lock.json")
        base={**default_config(round(original["base_config"]["actuator"])),**original["base_config"],
              "pad_k":spec["pad"]["nominal_whole_pad_k_N_m"],"pad_c":0,
              "pad_diameter":spec["user_geometry"]["diameter_m"],
              "pad_thickness":spec["user_geometry"]["thickness_m"],
              "pad_compression_limit":spec["pad"]["working_compression_limit_m"]}
        lock={**original,"name":name,"base_config":base,"frozen_gain_source":source,
              "extension_contract_sha256":sha(extension_path),"cohort_seeds":seeds,
              "geometry_transfer":"Unchanged conservative c0 gains; representative whole-pad k50k N/m,77mm x5mm geometry; no retuning or material-specification claim.",
              "engine_source_sha256":sha(HERE/"engine.c")}
        lock.pop("parameter_sha256",None)
        lock["parameter_sha256"]=hashlib.sha256(json.dumps(lock,sort_keys=True).encode()).hexdigest()
        lock_path=VALIDATION/"development"/f"{name}_lock.json"
        if lock_path.exists() and load(lock_path)!=lock:
            raise RuntimeError(f"Existing {name} lock differs; preserve the earlier lock.")
        if not lock_path.exists():dump(lock_path,lock)
        group=evaluated_config(lock,trace=False,seeds=seeds); groups.append(group)
        worst=max((r for r in group["records"] if "metrics" in r),
                  key=lambda r:r["metrics"]["force_peak_error"]+(0 if r["metrics"]["completed"] else 1000))
        config_path=OUT/"replay"/f"{name}_config.json"
        commands_path=OUT/"replay"/f"{name}_commands.csv.gz"
        trace_path=VALIDATION/"evaluation"/f"{name}_worst.csv.gz"
        metadata_path=VALIDATION/"evaluation"/f"{name}_worst_trace_metadata.json"
        if not commands_path.exists():
            metrics,trace=simulate(lib,worst["config"],trace_rate=10000)
            if metrics["force_peak_error"]!=worst["metrics"]["force_peak_error"]:
                raise AssertionError("Selected command-capture replay changed the full-step force peak.")
            dump(config_path,worst["config"])
            np.savetxt(commands_path,trace[:,[0,6,17,1]],delimiter=",",comments="",fmt="%.17g",
                       header="time_s,current_command_A,robot_displacement_m,engine_total_force_N")
            write_trace(trace_path,trace[::10])
            dump(metadata_path,{"id":worst["id"],"config":worst["config"],"metrics":metrics,
                                "source_sha256":sha(HERE/"engine.c"),"selected_from":name,
                                "trace_rate_Hz":1000,"command_rate_Hz":10000})
        replay_cases.append({"name":name,"id":worst["id"],"config":str(config_path),
            "commands":str(commands_path),"commands_compression":"gzip","metrics":worst["metrics"],
            "trace":str(trace_path),"source_sha256":sha(HERE/"engine.c")})
        print("PAD5MM",name,group["pass_count"],group["worst_error_N"],flush=True)
    dump(OUT/"replay/replay_cases_pad5mm.json",{"cases":replay_cases,
        "engine_source_sha256":sha(HERE/"engine.c"),"stage":"selected_physical_plant_replay_not_controller_qualification"})
    conditions=[(.1,5,0),(.1,50,90),(2.,5,0),(2.,50,90)]
    pairs_path=VALIDATION/"evaluation/pad5mm_exact_width_pairs.json"
    if not pairs_path.exists():
        pairs=[]
        for group in groups:
            name=group["candidate_lock"]["name"]
            wide=[r for r in group["records"] if
                  (r["config"]["mass_s"],r["config"]["force_target"],r["config"]["angle_deg"]) in conditions]
            def matched(row):
                point=execute(lib,{**row["config"],"pad_diameter":0},row["id"]+"_matched_point")
                return {"group":name,"wide_record_id":row["id"],"wide_source":f"{name}.json",
                        "point_record":point,"wide_metrics":row["metrics"],
                        "matched_fields":"Only diameter changes; same k/c/thickness,gains,force,masses,angle,seed,sensing/driver."}
            with ThreadPoolExecutor(max_workers=4) as pool:pairs.extend(pool.map(matched,wide))
        dump(pairs_path,{"status":"executed_exact_width_pairs_no_retuning","pairs":pairs,
            "new_point_execution_count":len(pairs),"wide_records_reused_count":len(pairs),
            "conditions":conditions,"seeds":seeds,"extension_contract_sha256":sha(extension_path),
            "engine_source_sha256":sha(HERE/"engine.c")})
    guard_path=VALIDATION/"evaluation/pad5mm_k20_guard_corners.json"
    if not guard_path.exists():
        requests=[]
        for group in groups:
            lock=group["candidate_lock"]
            for ix,(mass,force,angle) in enumerate(conditions):
                p={**lock["base_config"],**lock["gains"],"pad_k":20000,
                   "mass_s":mass,"identified_mass_s":mass,"force_target":force,"angle_deg":angle,"seed":seeds[0]}
                requests.append((p,lock["name"]+f"_k20_guard_{ix}"))
        with ThreadPoolExecutor(max_workers=4) as pool:
            rows=list(pool.map(lambda request:execute(lib,*request),requests))
        dump(guard_path,{"status":"executed_assumed_working_range_guard_diagnostic","records":rows,
            "guard_note":"Half-thickness2.5mm guard is a linear-model validity assumption, not measured Mirka bottoming.",
            "extension_contract_sha256":sha(extension_path),"engine_source_sha256":sha(HERE/"engine.c")})
    final_bytes=sum(p.stat().st_size for folder in ("engine","development","evaluation")
                    for p in (VALIDATION/folder).rglob("*") if p.is_file())
    if final_bytes>limit:raise RuntimeError(f"Engine retained allocation exceeded:{final_bytes}>{limit}")
    print("PAD5MM_STORAGE",final_bytes,limit,flush=True)


if __name__=="__main__":main()
