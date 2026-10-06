"""Bounded reward comparison after the full-inverse robot-gate failure."""
from pathlib import Path
import argparse,sys,time,shutil
import numpy as np
import torch
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from reward_search_inverse_probe import candidate
from policy_export import load_policy,export_policy,flat_weights
from reward_search import EpisodeEvaluator,build_evaluator,load_scenarios,aggregate,joint_band_score,write_json,sha
sys.path.insert(0,str(HERE.parent/'rl_unified_20261005'))
from env import FEATURE_NAMES

def main():
    a=argparse.ArgumentParser();a.add_argument('--base',required=True);a.add_argument('--native-build',required=True);a.add_argument('--bank',required=True);a.add_argument('--output',required=True);args=a.parse_args()
    torch.set_num_threads(1);out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    base=load_policy(args.base);scenarios=load_scenarios(args.native_build,args.bank)
    for row in scenarios:row['robotActionScale_m_s']=.01
    initial=out/'base_policy';export_policy(base,initial,FEATURE_NAMES,{'noTeacher':True,'physicalActionScales':{'currentOffset_A':2.,'robotRequest_m_s':.01}})
    library=build_evaluator(args.native_build,initial,out);e=EpisodeEvaluator(library,scenarios)
    train=[15,27,31,35,39,40,42,46,*range(128,140)];selection=[i for i,s in enumerate(scenarios) if s['split']=='selection']
    assert all(scenarios[i]['split']=='train' for i in train)
    reports=[];start=time.monotonic();evaluations=0;recovery=0
    for strength in [.3,.6]:
        p,fit=candidate(base,strength);name='inverse_'+str(strength).replace('.','p');folder=out/name
        export_policy(p,folder,FEATURE_NAMES,{'physicsInitialization':True,'inverseStrength':strength,'sourceCheckpointSha256':sha(args.base),'noTeacher':True,'physicalActionScales':{'currentOffset_A':2.,'robotRequest_m_s':.01}})
        write_json(folder/'frequency_fit.json',fit)
        for split,indices in [('train',train),('selection',selection)]:
            rows=e.evaluate(flat_weights(p),indices);evaluations+=len(rows)
            result=aggregate(rows);result.update(split=split,inverseStrength=strength,jointScore=joint_band_score(rows),policyDirectory=str(folder.resolve()))
            result['rank']=[result['physicalFailures'],-result['passed'],result['jointScore'],result['worstNormalForceError_N'],result['meanRmsNormalForceError_N']]
            failed=[r['scenario'] for r in rows if r['physicalFailure']]
            if failed:
                recorded=e.evaluate(flat_weights(p),failed[:1],record_dir=folder/(split+'_failed_native'));recovery+=len(recorded)
                write_json(folder/(split+'_failed_native')/'metrics.json',{'rows':recorded,'recoveryExcludedFromComparison':True})
            reports.append(result)
            write_json(out/'results.json',{'status':'running','comparisons':reports,'episodeComparisons':evaluations,'recoveryEpisodes':recovery,'elapsed_s':time.monotonic()-start})
            print({k:v for k,v in result.items() if k!='rows'},flush=True)
    write_json(out/'results.json',{'status':'complete','comparisons':reports,'episodeComparisons':evaluations,'recoveryEpisodes':recovery,'elapsed_s':time.monotonic()-start,'sourceCheckpointSha256':sha(args.base),'noFinalHoldout':True})

if __name__=='__main__':main()
