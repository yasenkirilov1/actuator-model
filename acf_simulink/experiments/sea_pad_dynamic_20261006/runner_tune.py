"""Pilot tuning phase; retains every tested candidate and failure compactly."""
from __future__ import annotations

import argparse
import hashlib
import itertools
import json
from pathlib import Path
import time

from engine_api import compile_engine
from runner_study import VALIDATION, default_config, dump, execute


def quick_tune(actuator: int, pad_k: float = 20000, pad_c: float = 0,
               diameter: float = 0, sea_k: float = 5000, lead: float = .005) -> dict:
    lib=compile_engine()
    base=default_config(actuator,pad_k,pad_c,diameter,sea_k,lead)
    families=[]
    if actuator==0:
        for kp,ki,vg in itertools.product([.5,2,5,10,20,40],[0,100,400],[0,80,250]):
            families.append(dict(force_kp=kp,force_ki=ki,velocity_gain=vg,inertia_gain=0))
    else:
        for kp,ki,servo in itertools.product([0,.25,1,3,10],[0,5,20,100],[20,50,100]):
            families.append(dict(force_kp=kp,force_ki=ki,velocity_gain=0,inertia_gain=0,servo_hz=servo))
    records=[];best=None;start=time.time()
    for ix,gains in enumerate(families):
        p={**base,**gains}
        row=execute(lib,p,f"candidate_{ix:04d}")
        records.append({"id":row["id"],"gains":gains,"status":row["status"],
                        **({"metrics":row["metrics"]} if row["status"]=="executed" else {"reason":row["reason"]})})
        if row["status"]=="executed":
            score=row["metrics"]["force_peak_error"]+(0 if row["metrics"]["completed"] else 1000)
            if best is None or score<best[0]:
                best=(score,ix,gains,row["metrics"])
                print(f"{actuator=} best {ix}: {score:.5f}N {gains}",flush=True)
    result={"status":"executed_tuning_screen_not_qualification","base_config":base,
            "candidate_count":len(records),"records":records,
            "selected":{"index":best[1],"gains":best[2],"metrics":best[3]},
            "runtime_s":time.time()-start}
    path=VALIDATION/"development"/f"screen_a{actuator}_k{pad_k:g}_c{pad_c:g}_d{diameter:g}_s{sea_k:g}_l{lead:g}.json"
    dump(path,result)
    return result


if __name__=="__main__":
    parser=argparse.ArgumentParser();parser.add_argument("actuator",type=int)
    parser.add_argument("--pad-k",type=float,default=20000);parser.add_argument("--pad-c",type=float,default=0)
    parser.add_argument("--diameter",type=float,default=0);parser.add_argument("--sea-k",type=float,default=5000)
    parser.add_argument("--lead",type=float,default=.005)
    a=parser.parse_args();quick_tune(a.actuator,a.pad_k,a.pad_c,a.diameter,a.sea_k,a.lead)
