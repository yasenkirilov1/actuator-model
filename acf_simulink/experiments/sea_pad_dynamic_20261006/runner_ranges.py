"""Observation-only final-source replay: all-step sensor and actuator ranges."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import json

from engine_api import HERE, compile_engine
from runner_study import VALIDATION, default_config, dump, execute


def main():
    lib=compile_engine()
    groups=["coil_point20k_c0","coil_point20k_c80","sea_v2_point20k_c0_5k_l10","sea_v2_point20k_c80_5k_l10"]
    results=[]
    for name in groups:
        original=json.loads((VALIDATION/"evaluation"/f"{name}.json").read_text())
        def run(row):
            p={**default_config(round(row["config"]["actuator"])),**row["config"]}
            new=execute(lib,p,row["id"]+"_range_replay")
            new["reference_peak_error_N"]=row["metrics"]["force_peak_error"]
            new["force_peak_replay_difference_N"]=abs(new["metrics"]["force_peak_error"]-row["metrics"]["force_peak_error"])
            return new
        with ThreadPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,original["records"]))
        result={"name":name,"status":"observation_only_instrumentation_replay_no_retuning",
                "original_evaluation_source":f"{name}.json","records":rows,
                "source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest(),
                "count":len(rows),"force_pass_count":sum(r["metrics"]["force_pass"] for r in rows),
                "sensor_rating_pass_count":sum(r["metrics"]["sensor_ratings_pass"] for r in rows),
                "actuator_rating_pass_count":sum(r["metrics"]["actuator_ratings_pass"] for r in rows),
                "maximum_peak_replay_difference_N":max(r["force_peak_replay_difference_N"] for r in rows),
                "cell_abs_peak_N":max(max(abs(r["metrics"]["cell_raw_min"]),abs(r["metrics"]["cell_raw_max"])) for r in rows),
                "specific_acceleration_peak_m_s2":max(r["metrics"]["specific_acceleration_peak"] for r in rows)}
        dump(VALIDATION/"evaluation"/f"{name}_range_replay.json",result);results.append(result)
        print(name,result["force_pass_count"],result["sensor_rating_pass_count"],result["actuator_rating_pass_count"],
              result["cell_abs_peak_N"],result["specific_acceleration_peak_m_s2"],result["maximum_peak_replay_difference_N"],flush=True)
    dump(VALIDATION/"evaluation/range_replay_summary.json",[{k:v for k,v in r.items() if k!="records"} for r in results])


if __name__=="__main__":main()
