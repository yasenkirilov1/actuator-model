"""Geometry-only transfer: frozen point controllers applied to broad pads."""
from __future__ import annotations

import hashlib
import json

from engine_api import HERE
from runner_campaign import evaluated_config
from runner_study import VALIDATION, default_config, dump


def main():
    sources=["coil_point20k_c0","coil_point20k_c80",
             "sea_v2_point20k_c0_5k_l10","sea_v2_point20k_c80_5k_l10"]
    for source in sources:
        old=json.loads((VALIDATION/"development"/f"{source}_lock.json").read_text())
        name=source.replace("point","wide")+"_width_transfer"
        base={**default_config(round(old["base_config"]["actuator"])),**old["base_config"],"pad_diameter":.077}
        lock={**old,"name":name,"base_config":base,
              "geometry_transfer":"Point-to77mm footprint; same total flat k,c, same controller gains, new noise seeds; no retuning",
              "source_point_lock":source,"engine_source_sha256":hashlib.sha256((HERE/"engine.c").read_bytes()).hexdigest()}
        lock["parameter_sha256"]=hashlib.sha256(json.dumps(lock,sort_keys=True).encode()).hexdigest()
        dump(VALIDATION/"development"/f"{name}_lock.json",lock)
        evaluated_config(lock,trace=True,seeds=[260621,260622])


if __name__=="__main__":main()
