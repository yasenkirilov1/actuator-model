"""Unchanged coil controller: isolate pad stiffness/damping and footprint."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import hashlib
import itertools
import json

from engine_api import HERE, compile_engine, simulate, write_trace
from runner_study import VALIDATION, default_config, dump, execute


def main():
    lib=compile_engine()
    gains=dict(force_kp=40,force_ki=0,velocity_gain=80,inertia_gain=.8,inertia_cutoff=80)
    pads=[(k,c,0) for k,c in itertools.product([20000,50000,100000],[0,80])]
    pads.extend([(20000,0,.077),(20000,80,.077),(0,0,0),(1000000,80,0)])
    conditions=[(.1,5,0),(.1,50,90),(2.,5,0),(2.,50,90)]
    def run(item):
        index,(k,c,d),case=item
        mass,force,angle=case
        p={**default_config(0,pad_k=k,pad_c=c,diameter=d),**gains,
           "mass_s":mass,"identified_mass_s":mass,"force_target":force,
           "angle_deg":angle,"seed":260620}
        return execute(lib,p,f"paired_{index:03d}")
    items=[(ix,pad,case) for ix,(pad,case) in enumerate(itertools.product(pads,conditions))]
    with ThreadPoolExecutor(max_workers=4) as pool:rows=list(pool.map(run,items))
    result={"status":"executed_unchanged_controller_factor_diagnostic",
            "force_quantity":"total axial pressing force, no drag; not normal-force qualification",
            "gains":gains,"conditions":conditions,"pads":pads,"seed":260620,
            "engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest(),
            "records":rows,"count":len(rows),"executed_count":sum(r["status"]=="executed" for r in rows)}
    dump(VALIDATION/"evaluation/paired_coil_factors.json",result)
    for k,c,d in pads:
        rr=[r for r in rows if r["config"]["pad_k"]==k and r["config"]["pad_c"]==c and r["config"]["pad_diameter"]==d]
        print(f"PAIRED k{k:g} c{c:g} d{d:g}: {sum(r['metrics']['force_pass'] for r in rr)}/{len(rr)} pass, worst{max(r['metrics']['force_peak_error'] for r in rr):.6f}N",flush=True)
    # Four matched traces illustrate point/footprint and material variation.
    for k,c,d in [(20000,0,0),(20000,80,0),(20000,0,.077),(20000,80,.077),(0,0,0),(1000000,80,0)]:
        rr=next(r for r in rows if r["config"]["pad_k"]==k and r["config"]["pad_c"]==c and r["config"]["pad_diameter"]==d and r["config"]["mass_s"]==2 and r["config"]["force_target"]==50)
        _,trace=simulate(lib,rr["config"],trace_rate=500)
        write_trace(VALIDATION/"evaluation"/f"paired_k{k:g}_c{c:g}_d{d:g}_S2_F50.csv",trace)


if __name__=="__main__":main()
