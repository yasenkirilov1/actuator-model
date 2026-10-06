"""Exact common-seed point/wide comparisons under four frozen controllers."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import json

import numpy as np

from engine_api import HERE, compile_engine, simulate
from runner_study import VALIDATION, default_config, dump, execute


def main():
    lib=compile_engine();pairs=[]
    groups=["coil_wide20k_c0_width_transfer","coil_wide20k_c80_width_transfer",
            "sea_v2_wide20k_c0_5k_l10_width_transfer","sea_v2_wide20k_c80_5k_l10_width_transfer"]
    conditions=[(.1,5,0),(.1,50,90),(2.,5,0),(2.,50,90)]
    for name in groups:
        group=json.loads((VALIDATION/"evaluation"/f"{name}.json").read_text())
        originals=[r for r in group["records"] if (r["config"]["mass_s"],r["config"]["force_target"],r["config"]["angle_deg"]) in conditions]
        def run(original):
            p={**default_config(round(original["config"]["actuator"])),**original["config"],"pad_diameter":0}
            point=execute(lib,p,original["id"]+"_matchedpoint")
            return {"group":name,"wide_record_id":original["id"],"wide_source":f"{name}.json",
                    "point_record":point,"wide_metrics":original["metrics"],
                    "matched_fields":"Same k/c,gains,masses,force/orientation,seed,ADC/currentdriver;onlydiameter changes."}
        with ThreadPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,originals))
        pairs.extend(rows)
        if "c0_" in name:
            original=next(r for r in originals if r["config"]["mass_s"]==2 and r["config"]["force_target"]==50 and r["config"]["seed"]==260621)
            for geometry in ["point","wide"]:
                p={**default_config(round(original["config"]["actuator"])),**original["config"],"pad_diameter":0 if geometry=="point" else .077}
                _,trace=simulate(lib,p,trace_rate=1000)
                columns=[0,1,2,5,6,16,19]
                path=VALIDATION/"evaluation"/f"exact_pair_{'coil' if p['actuator']==0 else 'sea'}_{geometry}_c0_S2_F50.csv.gz"
                np.savetxt(path,trace[:,columns],delimiter=",",comments="",fmt="%.12g",
                           header="time_s,contact_total_axial_N,force_estimate_N,current_A,current_command_A,force_actuator_N,pad_compression_m")
    result={"status":"executed_exact_geometry_pairs_no_retuning","new_point_execution_count":len(pairs),
            "wide_records_reused_count":len(pairs),"pairs":pairs,"conditions":conditions,"seeds":[260621,260622],
            "engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest()}
    dump(VALIDATION/"evaluation/exact_width_pairs.json",result)
    for name in groups:
        rows=[r for r in pairs if r["group"]==name]
        print("WIDTHPAIR",name,"point",max(r["point_record"]["metrics"]["force_peak_error"] for r in rows),
              "wide",max(r["wide_metrics"]["force_peak_error"] for r in rows),flush=True)


if __name__=="__main__":main()
