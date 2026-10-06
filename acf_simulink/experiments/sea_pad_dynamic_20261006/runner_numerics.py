"""Noise-free closed-loop temporal/spatial refinement without extra histories."""
from __future__ import annotations

import hashlib
import json

from engine_api import HERE, compile_engine
from runner_study import VALIDATION, default_config, dump, execute


def main():
    lib=compile_engine();rows=[]
    wide=["coil_wide20k_c0_width_transfer","coil_wide20k_c80_width_transfer",
          "sea_v2_wide20k_c0_5k_l10_width_transfer","sea_v2_wide20k_c80_5k_l10_width_transfer"]
    for name in wide:
        group=json.loads((VALIDATION/"evaluation"/f"{name}.json").read_text())
        worst=max(group["records"],key=lambda r:r["metrics"]["force_peak_error"])
        low=next(r for r in group["records"] if r["config"]["mass_s"]==2 and r["config"]["force_target"]==5 and r["config"]["angle_deg"]==0)
        for label,original in [("selected_worst",worst),("lowforce_corner",low)]:
            p={**default_config(round(original["config"]["actuator"])),**original["config"],"noise_enabled":0}
            a=execute(lib,p,name+"_"+label+"_61strips")
            b=execute(lib,{**p,"pad_points":121},name+"_"+label+"_121strips")
            rows.append({"group":name,"case":label,"kind":"strip_refinement","records":[a,b],
                         "peak_difference_N":abs(a["metrics"]["force_peak_error"]-b["metrics"]["force_peak_error"]),
                         "same_physical_stop_outcome":a["metrics"]["stop_reason"]==b["metrics"]["stop_reason"]})
    groups=["coil_point20k_c80","coil_wide20k_c0_width_transfer",
            "sea_v2_point20k_c80_5k_l10","sea_v2_wide20k_c0_5k_l10_width_transfer"]
    for name in groups:
        group=json.loads((VALIDATION/"evaluation"/f"{name}.json").read_text())
        worst=max(group["records"],key=lambda r:r["metrics"]["force_peak_error"])
        p={**default_config(round(worst["config"]["actuator"])),**worst["config"],"noise_enabled":0}
        a=execute(lib,p,name+"_dt2p5us")
        b=execute(lib,{**p,"dt":1.25e-6},name+"_dt1p25us")
        rows.append({"group":name,"kind":"time_refinement","records":[a,b],
                     "peak_difference_N":abs(a["metrics"]["force_peak_error"]-b["metrics"]["force_peak_error"]),
                     "same_physical_stop_outcome":a["metrics"]["stop_reason"]==b["metrics"]["stop_reason"]})
    result={"status":"executed_noise_free_closed_loop_refinement","records":rows,
            "execution_count":2*len(rows),"noise_disabled_reason":"Different fine clocks change randomnoise draw histories; isolate integration/spatial quadrature convergence.",
            "engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest(),
            "max_spatial_peak_difference_N":max(r["peak_difference_N"] for r in rows if r["kind"]=="strip_refinement"),
            "max_temporal_peak_difference_N":max(r["peak_difference_N"] for r in rows if r["kind"]=="time_refinement")}
    dump(VALIDATION/"evaluation/closed_loop_numerical_refinement.json",result)
    for row in rows:print("REFINE",row["group"],row["kind"],row.get("case",""),row["peak_difference_N"],flush=True)


if __name__=="__main__":main()
