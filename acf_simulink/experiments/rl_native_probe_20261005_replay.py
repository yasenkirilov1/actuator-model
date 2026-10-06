"""Convert a frozen C-native recording to the independent MATLAB replay input."""
import argparse
import hashlib
from pathlib import Path

import numpy as np
from scipy.io import savemat


def export_mat(record, target, scenario, seed=-1, lower_travel_limit=None):
    record, target = Path(record).resolve(), Path(target).resolve()
    if target.exists():
        raise FileExistsError("Preserve prior replay inputs")
    loaded = np.load(record)
    data = {name: loaded[name] for name in ("time", "inputs", "truth")}
    if data["inputs"].shape != (len(data["time"]),3) or data["truth"].shape != (len(data["time"]),27):
        raise ValueError("Expected full physical-tick native time/input3/truth27")
    data.update(scenario=int(scenario), seed=int(seed), sourceRecording=str(record),
                sourceRecordingSha256=hashlib.sha256(record.read_bytes()).hexdigest(),
                lowerTravelLimit_m=float("nan") if lower_travel_limit is None else float(lower_travel_limit))
    target.parent.mkdir(parents=True,exist_ok=True)
    savemat(target,data,do_compression=False,oned_as="column")
    return target


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("record");parser.add_argument("target");parser.add_argument("--scenario",type=int,required=True)
    parser.add_argument("--seed",type=int,default=-1);parser.add_argument("--lower-travel-limit",type=float)
    args=parser.parse_args()
    print(export_mat(args.record,args.target,args.scenario,args.seed,args.lower_travel_limit))
