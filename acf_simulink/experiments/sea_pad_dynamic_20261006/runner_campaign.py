"""Adaptive bounded development on designated corners, then frozen evaluation.

All outcomes, including failed and stopped candidates, are retained as metrics.
No candidate is selected using evaluation cases or evaluation noise seeds.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import itertools
import json
from pathlib import Path
import time

import numpy as np

from engine_api import HERE, OUT, ROOT, compile_engine, simulate, write_trace
from runner_study import VALIDATION, contract, default_config, dump, execute

DEVELOPMENT_CASES=[(2.,50.,90.),(.1,50.,90.),(2.,5.,0.),(1.4,20.,45.)]


def compact_row(row:dict, gains:dict, case:tuple) -> dict:
    return {"id":row["id"],"gains":gains,"case":{"mass_s":case[0],"force":case[1],"angle_deg":case[2]},
            "status":row["status"],**({"metrics":row["metrics"]} if row["status"]=="executed" else {"reason":row["reason"]})}


def score(row:dict) -> float:
    if row["status"]!="executed":return 100000
    return row["metrics"]["force_peak_error"]+(0 if row["metrics"]["completed"] else 1000)


def candidates(actuator:int) -> list[dict]:
    if actuator==0:
        result=[dict(force_kp=k,force_ki=0,velocity_gain=d,inertia_gain=0,inertia_cutoff=80)
                for k,d in itertools.product([5,10,15,20,30,40],[80,130,200,350,600])]
        result.extend(dict(force_kp=k,force_ki=0,velocity_gain=d,inertia_gain=ff,inertia_cutoff=cut)
                      for k,d,ff,cut in itertools.product([10,20,40],[80,200],[.5,.8,1],[30,80,150]))
        result.extend(dict(force_kp=k,force_ki=ki,velocity_gain=d,inertia_gain=.8,inertia_cutoff=80)
                      for k,ki,d in itertools.product([10,20,40],[100,300],[100,250]))
        return result
    result=[dict(force_kp=k,force_ki=0,velocity_gain=d,inertia_gain=ff,inertia_cutoff=cut,
                 servo_hz=s,servo_zeta=1,servo_target_cutoff=300)
            for k,d,ff,cut,s in itertools.product([0,.5,2,5],[50,200,500],[0,.5,1],[30,100],[20,50,100])]
    # Uniform deterministic subset; same bounded search space for each SEA configuration.
    rng=np.random.default_rng(260610)
    indices=sorted(rng.choice(len(result),size=120,replace=False))
    return [result[j] for j in indices]


def develop_config(name:str, base:dict, top_count:int=8) -> dict:
    output=VALIDATION/"development"/f"{name}.json"
    if output.exists():return json.loads(output.read_text())
    lib=compile_engine();start=time.time();families=candidates(round(base["actuator"]))
    records=[]
    def run(ix:int, gains:dict, case:tuple):
        p={**base,**gains,"mass_s":case[0],"identified_mass_s":case[0],"force_target":case[1],"angle_deg":case[2]}
        return compact_row(execute(lib,p,f"{name}_g{ix:03d}_m{case[0]}_f{case[1]}_a{case[2]}"),gains,case)
    with ThreadPoolExecutor(max_workers=4) as pool:
        pending={pool.submit(run,i,g,DEVELOPMENT_CASES[0]):i for i,g in enumerate(families)}
        for fut in as_completed(pending):records.append((pending[fut],fut.result()))
    records.sort(key=lambda item:item[0])
    ranked=sorted(records,key=lambda item:(score(item[1]),item[0]))[:top_count]
    round2=[]
    with ThreadPoolExecutor(max_workers=4) as pool:
        pending={pool.submit(run,i,families[i],case):(i,case) for i,_ in ranked for case in DEVELOPMENT_CASES[1:]}
        for fut in as_completed(pending):round2.append((pending[fut][0],fut.result()))
    all_records=records+round2
    ranked_worst=[]
    for ix,_ in ranked:
        outcomes=[r for i,r in all_records if i==ix]
        ranked_worst.append((max(map(score,outcomes)),ix,outcomes))
    best=min(ranked_worst,key=lambda item:(item[0],item[1]))
    lock={"name":name,"base_config":base,"gains":families[best[1]],"candidate_index":best[1],
          "development_worst_error_N":max(r["metrics"]["force_peak_error"] for r in best[2]),
          "development_all_completed":all(r["metrics"]["completed"] for r in best[2]),
          "development_cases":DEVELOPMENT_CASES,"selection_score":best[0],
          "locked_before_evaluation":True,
          "engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest(),
          "engine_binary_sha256":hashlib.sha256(Path(lib._name).read_bytes()).hexdigest(),
          "study_contract_sha256":hashlib.sha256((HERE/"study_contract.json").read_bytes()).hexdigest()}
    lock["parameter_sha256"]=hashlib.sha256(json.dumps(lock,sort_keys=True).encode()).hexdigest()
    result={"status":"executed_development","candidate_families":len(families),
            "records":[r for _,r in sorted(all_records,key=lambda item:(item[0],str(item[1]["case"])))],
            "candidate_lock":lock,"runtime_s":time.time()-start}
    dump(output,result);dump(VALIDATION/"development"/f"{name}_lock.json",lock)
    print(f"DEVELOPED {name}: {best[0]:.6f} N ({len(all_records)} executed candidates/corners), {families[best[1]]}",flush=True)
    return result


def evaluated_config(lock:dict, trace:bool=False, seeds:list[int]|None=None) -> dict:
    name=lock["name"];path=VALIDATION/"evaluation"/f"{name}.json"
    if path.exists():return json.loads(path.read_text())
    lib=compile_engine();base={**lock["base_config"],**lock["gains"]};spec=contract()["intended_evaluation"]
    rows=[];start=time.time()
    cohort_seeds=seeds if seeds is not None else spec["new_noise_seeds"]
    cases=list(itertools.product(spec["mS_kg"],spec["force_N"],spec["orientation_deg"],cohort_seeds))
    def run(ix:int,case:tuple):
        mass,force,angle,seed=case
        p={**base,"mass_s":mass,"identified_mass_s":mass,"force_target":force,"angle_deg":angle,"seed":seed}
        return execute(lib,p,f"{name}_evaluation_{ix:03d}")
    with ThreadPoolExecutor(max_workers=4) as pool:
        pending={pool.submit(run,ix,case):ix for ix,case in enumerate(cases)}
        for fut in as_completed(pending):rows.append((pending[fut],fut.result()))
    rows=[r for _,r in sorted(rows)]
    executed=[r for r in rows if r["status"]=="executed"]
    result={"status":"executed_locked_finite_development_evaluation","candidate_lock":lock,
            "records":rows,"planned_count":len(cases),"executed_count":len(executed),
            "cohort_noise_seeds":cohort_seeds,
            "preparation_rejected_count":len(rows)-len(executed),
            "pass_count":sum(r["metrics"]["force_pass"] for r in executed),
            "physical_stop_count":sum(not r["metrics"]["completed"] for r in executed),
            "worst_error_N":max(r["metrics"]["force_peak_error"] for r in executed),
            "runtime_s":time.time()-start}
    dump(path,result)
    print(f"EVALUATED {name}: {result['pass_count']}/{len(executed)} passes; worst{result['worst_error_N']:.6f}N",flush=True)
    if trace:
        worst=max(executed,key=lambda r:r["metrics"]["force_peak_error"]+(0 if r["metrics"]["completed"] else 1000))
        m,tr=simulate(lib,worst["config"],trace_rate=1000)
        write_trace(VALIDATION/"evaluation"/f"{name}_worst.csv.gz",tr)
        dump(VALIDATION/"evaluation"/f"{name}_worst_trace_metadata.json",{"id":worst["id"],"metrics":m,"config":worst["config"]})
    return result


def main():
    configs=[
      ("coil_point20k_c0",default_config(0)),
      ("sea_point20k_c0_5k_l10",default_config(1,lead=.01)),
      ("coil_point20k_c80",default_config(0,pad_c=80)),
      ("sea_point20k_c80_5k_l10",default_config(1,pad_c=80,lead=.01)),
    ]
    for name,base in configs:
        result=develop_config(name,base)
        evaluated_config(result["candidate_lock"],trace=True)


if __name__=="__main__":main()
