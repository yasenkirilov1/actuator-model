"""Matched series-rate/lead sensitivities and explicit rigid-point diagnostics."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import itertools
import json

from engine_api import HERE, compile_engine
from runner_study import VALIDATION, default_config, dump, execute


def main():
    lib=compile_engine()
    lock=json.loads((VALIDATION/"development/sea_v2_point20k_c80_5k_l10_lock.json").read_text())
    gains=lock["gains"]
    cases=[(.1,5,0),(.1,50,90),(2.,5,0),(2.,50,90)]
    combinations=list(itertools.product([3500,5000,10000],[.005,.01],[0,.077],cases))
    def run(item):
        ix,(spring,lead,diameter,case)=item
        mass,force,angle=case
        p={**default_config(1,pad_c=80,diameter=diameter,sea_k=spring,lead=lead),**gains,
           "mass_s":mass,"identified_mass_s":mass,"force_target":force,"angle_deg":angle,"seed":260630}
        return execute(lib,p,f"transmission_{ix:03d}")
    with ThreadPoolExecutor(max_workers=4) as pool:records=list(pool.map(run,enumerate(combinations)))
    result={"status":"executed_matched_transmission_sensitivity_no_gain_retuning",
            "controller_source_lock":"sea_v2_point20k_c80_5k_l10_lock.json","gains":gains,
            "conditions":cases,"seed":260630,"records":records,"count":len(records),
            "engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest()}
    dump(VALIDATION/"evaluation/transmission_sensitivity.json",result)
    for spring,lead,diameter in itertools.product([3500,5000,10000],[.005,.01],[0,.077]):
        rr=[r for r in records if r["config"]["sea_k"]==spring and r["config"]["lead"]==lead and r["config"]["pad_diameter"]==diameter]
        executed=[r for r in rr if r["status"]=="executed"]
        print(f"TRANSMISSION s{spring:g} l{lead:g} d{diameter:g}: {sum(r['metrics']['force_pass'] for r in executed)}/{len(executed)},worst{max(r['metrics']['force_peak_error'] for r in executed):.6f}",flush=True)
    # The actual no-pad rigid-point branch differs from the finite1MN/m surrogate.
    rows=[]
    coil_gains=dict(force_kp=40,force_ki=0,velocity_gain=80,inertia_gain=.8,inertia_cutoff=80)
    for actuator,case in itertools.product([0,1],cases):
        mass,force,angle=case
        p={**default_config(actuator,pad_k=0,pad_c=0,lead=.01),**(coil_gains if actuator==0 else gains),
           "mass_s":mass,"identified_mass_s":mass,"force_target":force,"angle_deg":angle,"seed":260630}
        rows.append(execute(lib,p,f"rigid_point_a{actuator}_S{mass}_F{force}_angle{angle}"))
    dump(VALIDATION/"evaluation/rigid_point_matched_diagnostic.json",
         {"status":"executed_rigid_point_loaded_equilibrium_no_retuning_from20k_c80",
          "records":rows,"count":len(rows),"rigid_contract":"pad_k=0;S=h;F=L+mS*(aS+g);originalsharp1mm/10mm/100mm/s",
          "engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest()})
    for row in rows:print("RIGID",row["id"],row["metrics"]["force_peak_error"],row["metrics"]["stop_reason"],flush=True)


if __name__=="__main__":main()
