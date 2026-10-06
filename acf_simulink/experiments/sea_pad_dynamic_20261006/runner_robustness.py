"""Fixed conservative wide-pad controllers under material parameter changes."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import itertools
import json

from engine_api import HERE, compile_engine
from runner_study import VALIDATION, default_config, dump, execute


def main():
    lib=compile_engine()
    sources={0:"coil_point20k_c0",1:"sea_v2_point20k_c0_5k_l10"}
    gains={a:json.loads((VALIDATION/"development"/f"{name}_lock.json").read_text())["gains"] for a,name in sources.items()}
    cases=[(.1,5,0),(.1,50,90),(2.,5,0),(2.,50,90)]
    combinations=list(itertools.product([0,1],[20000,50000,100000],[0,40,80,150],cases))
    def run(item):
        index,(actuator,k,c,case)=item
        mass,force,angle=case
        p={**default_config(actuator,pad_k=k,pad_c=c,diameter=.077,lead=.01),**gains[actuator],
           "mass_s":mass,"identified_mass_s":mass,"force_target":force,"angle_deg":angle,"seed":260640}
        return execute(lib,p,f"material_sensitivity_{index:03d}")
    with ThreadPoolExecutor(max_workers=4) as pool:records=list(pool.map(run,enumerate(combinations)))
    result={"status":"executed_fixed_conservative_controller_material_sensitivity",
            "controller_sources":sources,"gains":gains,"conditions":cases,"seed":260640,
            "records":records,"count":len(records),"width_m":.077,
            "scope":"Reduced-scope axial total-force material sensitivity, notrobust/hardware qualification",
            "engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest()}
    dump(VALIDATION/"evaluation/conservative_material_sensitivity.json",result)
    for actuator,k,c in itertools.product([0,1],[20000,50000,100000],[0,40,80,150]):
        rr=[r for r in records if r["config"]["actuator"]==actuator and r["config"]["pad_k"]==k and r["config"]["pad_c"]==c]
        executed=[r for r in rr if r["status"]=="executed"]
        print(f"MATERIAL a{actuator} k{k:g} c{c:g}: {sum(r['metrics']['force_pass'] for r in executed)}/{len(executed)},worst{max(r['metrics']['force_peak_error'] for r in executed):.6f}N, RMS{max(r['metrics']['current_rms'] for r in executed):.6f}A",flush=True)


if __name__=="__main__":main()
