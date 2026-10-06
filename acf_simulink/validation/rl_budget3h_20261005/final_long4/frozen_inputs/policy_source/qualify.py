"""Freeze selection, then compare one trained shared policy on native holdout."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil

from native_api import NativePlant
from policy_export import load_policy
from train import evaluate,get_scenarios,write_json


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--runs",nargs="+",required=True)
    parser.add_argument("--native-build",required=True)
    parser.add_argument("--bank",required=True)
    parser.add_argument("--output",required=True)
    args=parser.parse_args()
    folder=Path(args.output);folder.mkdir(parents=True,exist_ok=False)
    candidates=[]
    for directory in map(Path,args.runs):
        report=json.loads((directory/"results.json").read_text())
        assert report["status"]=="training_complete" and report["selection"]
        candidates.append({"directory":str(directory.resolve()),"seed":report["seed"],
                           "rank":report["selectedRank"],"update":report["selectedUpdate"],
                           "transitions":report["trainedTransitions"]})
    selected=min(candidates,key=lambda x:tuple(x["rank"]))
    source=Path(selected["directory"])/"selected_policy"
    checkpoint=source/"policy_checkpoint.pt"
    selection={"status":"frozen_before_holdout","candidates":candidates,"selected":selected,
               "selectedCheckpointSha256":hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
               "ranking":"Fewest physical failures, most full-episode±0.5N passes, lowest worst peak error, lowest mean RMS; selection episodes only.",
               "heldoutRetuning":False}
    write_json(folder/"selection_lock.json",selection)
    shutil.copytree(source,folder/"selected_policy")
    plant=NativePlant(args.native_build);scenarios=get_scenarios(plant,args.bank)
    indices=[i for i,r in enumerate(scenarios) if r["split"]=="holdout"]
    trained=load_policy(checkpoint)
    initial=load_policy(Path(selected["directory"])/"rollouts/initial_policy/policy_checkpoint.pt")
    outputs={}
    for label,policy,baseline in (("baseline",trained,True),("untrained",initial,False),("trained",trained,False)):
        outputs[label]=evaluate(plant,scenarios,indices,policy,folder/label,label,
                                baseline=baseline,save_histories=True)
        print(f"RL_HOLDOUT {label} passes={outputs[label]['passed']}/{len(indices)} "
              f"peak={outputs[label]['worstNormalForceError_N']:.9g}N "
              f"physicalFailures={outputs[label]['physicalFailures']}",flush=True)
    pairs=[]
    for b,u,t in zip(outputs["baseline"]["rows"],outputs["untrained"]["rows"],outputs["trained"]["rows"],strict=True):
        assert b["identity"]==u["identity"]==t["identity"] and b["seed"]==u["seed"]==t["seed"]
        pairs.append({"identity":b["identity"],"baselinePeak_N":b["peakNormalForceError_N"],
                      "untrainedPeak_N":u["peakNormalForceError_N"],"trainedPeak_N":t["peakNormalForceError_N"],
                      "baselinePassed":b["passed"],"untrainedPassed":u["passed"],"trainedPassed":t["passed"],
                      "trainedMinusBaseline_N":t["peakNormalForceError_N"]-b["peakNormalForceError_N"],
                      "trainedPhysicalFailure":t["physicalFailure"]})
    report={"status":"holdout_complete_native_replay_pending","selection":selection,
            "cases":len(indices),"nativeRecordings":sum(x["recordings"] for x in outputs.values()),
            "nativeSamples":sum(x["totalNativeSamples"] for x in outputs.values()),
            "comparisons":pairs,
            "outcomes":{name:{k:v for k,v in result.items() if k not in ("rows","featureNames")}
                        for name,result in outputs.items()},
            "trainedBetterCases":sum(x["trainedMinusBaseline_N"]<0 for x in pairs),
            "trainedWorseCases":sum(x["trainedMinusBaseline_N"]>0 for x in pairs),
            "promoted":False,"scope":"Finite new held-out native codegen recordings, every physical tick incl startup. No retuning or hardware qualification."}
    write_json(folder/"results.json",report)
    # Shared policy weights cannot change while outcomes are inspected.
    assert hashlib.sha256(checkpoint.read_bytes()).hexdigest()==selection["selectedCheckpointSha256"]


if __name__=="__main__":main()
