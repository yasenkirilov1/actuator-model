"""Parallel native-reward comparison of causal inverse rolloff initializations."""
from pathlib import Path
import argparse,copy,multiprocessing as mp,sys,time,shutil
import numpy as np
import torch
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from reward_search_inverse_probe import frequency_fit
from policy_export import load_policy,export_policy,flat_weights
from reward_search import EpisodeEvaluator,build_evaluator,load_scenarios,aggregate,joint_band_score,write_json,sha
sys.path.insert(0,str(HERE.parent/'rl_unified_20261005'))
from env import FEATURE_NAMES

_E=None
def initialize(library,scenarios):
    global _E
    torch.set_num_threads(1);_E=EpisodeEvaluator(library,scenarios)
def evaluate(task):
    name,weights,indices,folder=task
    rows=_E.evaluate(weights,indices)
    failed=[r['scenario'] for r in rows if r['physicalFailure']]
    recovery=0
    if failed:
        recorded=_E.evaluate(weights,failed[:1],record_dir=Path(folder)/'failed_native')
        write_json(Path(folder)/'failed_native/replay.json',{'rows':recorded,'recoveryExcludedFromComparison':True});recovery=len(recorded)
    return name,rows,recovery

def delta_inverse(base,cutoff,strength):
    p=copy.deepcopy(base);fits={}
    with torch.no_grad():
        for label,pairs,aux,sign in [('acceleration',range(4,8),1,-1),('geometry',range(8,12),2,1)]:
            old,d0,f0=frequency_fit(p,pairs,.1,sign,cutoff=1500.)
            new,d1,f1=frequency_fit(p,pairs,.1,sign,cutoff=cutoff)
            delta=strength*new-.3*old
            for j,pair in enumerate(pairs):
                even=2*pair;p.actor.weight[0,even]+=float(delta[j]);p.actor.weight[0,even+1]-=float(delta[j])
            p.actor_aux.weight[0,aux]+=float(strength*d1-.3*d0)
            fits[label]={'oldIntendedStrength':.3,'oldFit':f0,'newIntendedStrength':strength,'newFit':f1,'scope':'Add the difference between two mechanics frequency responses; no teacher trajectory fit.'}
    return p,fits

def main():
    a=argparse.ArgumentParser();a.add_argument('--base',required=True);a.add_argument('--native-build',required=True);a.add_argument('--bank',required=True);a.add_argument('--output',required=True);a.add_argument('--workers',type=int,default=3);args=a.parse_args()
    torch.set_num_threads(1);out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    base=load_policy(args.base);scenarios=load_scenarios(args.native_build,args.bank)
    for row in scenarios:row['robotActionScale_m_s']=.01
    provenance={'noTeacher':True,'sourceCheckpointSha256':sha(args.base),'sourceCheckpoint':str(Path(args.base).resolve()),'physicalActionScales':{'currentOffset_A':2.,'robotRequest_m_s':.01}}
    export_policy(base,out/'base_policy',FEATURE_NAMES,provenance)
    library=build_evaluator(args.native_build,out/'base_policy',out);evaluator=EpisodeEvaluator(library,scenarios)
    train=[15,27,31,35,39,40,42,46,*range(128,140)];selection=[i for i,s in enumerate(scenarios) if s['split']=='selection']
    assert all(scenarios[i]['split']=='train' for i in train)
    candidates={'base':{'policyDirectory':str((out/'base_policy').resolve()),'weights':flat_weights(base),'rolloff_Hz':1500.,'intendedStrength':.3}}
    for cutoff in [400.,700.,1000.]:
        for strength in [.6,1.]:
            p,fits=delta_inverse(base,cutoff,strength);name=f'rolloff_{cutoff:g}_strength_{strength:g}'.replace('.','p');folder=out/name
            export_policy(p,folder,FEATURE_NAMES,{**provenance,'inverseRolloff_Hz':cutoff,'intendedInverseStrength':strength,'physicsInitialization':True})
            write_json(folder/'frequency_fit.json',fits)
            candidates[name]={'policyDirectory':str(folder.resolve()),'weights':flat_weights(p),'rolloff_Hz':cutoff,'intendedStrength':strength}
    reports=[];start=time.monotonic();episodes=0;recovery=0
    with mp.get_context('spawn').Pool(args.workers,initializer=initialize,initargs=(str(library),scenarios)) as pool:
        tasks=[(name,c['weights'],train,str(out/name)) for name,c in candidates.items()]
        for name,rows,recovered in pool.imap_unordered(evaluate,tasks):
            result=aggregate(rows);result.update(name=name,split='train',jointScore=joint_band_score(rows),**{k:v for k,v in candidates[name].items() if k!='weights'})
            result['rank']=[result['physicalFailures'],-result['passed'],result['jointScore'],result['worstNormalForceError_N'],result['meanRmsNormalForceError_N']]
            reports.append(result);episodes+=len(rows);recovery+=recovered
            write_json(out/'results.json',{'status':'running_train_comparison','comparisons':reports,'episodeComparisons':episodes,'recoveryEpisodes':recovery,'elapsed_s':time.monotonic()-start})
            print({k:v for k,v in result.items() if k!='rows'},flush=True)
    # Only the two strongest candidates from the bounded training screen go to
    # the fixed selection bank. The previous accepted checkpoint stays eligible.
    eligible=sorted((r for r in reports if r['physicalFailures']==0),key=lambda r:r['rank'])[:2]
    for prior in eligible:
        name=prior['name'];rows=evaluator.evaluate(candidates[name]['weights'],selection)
        result=aggregate(rows);result.update(name=name,split='selection',jointScore=joint_band_score(rows),**{k:v for k,v in candidates[name].items() if k!='weights'})
        result['rank']=[result['physicalFailures'],-result['passed'],result['jointScore'],result['worstNormalForceError_N'],result['meanRmsNormalForceError_N']]
        failed=[r['scenario'] for r in rows if r['physicalFailure']]
        if failed:
            recorded=evaluator.evaluate(candidates[name]['weights'],failed[:1],record_dir=out/name/'selection_failed_native');recovery+=len(recorded)
            write_json(out/name/'selection_failed_native/replay.json',{'rows':recorded,'recoveryExcludedFromComparison':True})
        reports.append(result);episodes+=len(rows)
        write_json(out/'results.json',{'status':'running_selection_comparison','comparisons':reports,'episodeComparisons':episodes,'recoveryEpisodes':recovery,'elapsed_s':time.monotonic()-start})
        print({k:v for k,v in result.items() if k!='rows'},flush=True)
    write_json(out/'results.json',{'status':'complete','comparisons':reports,'episodeComparisons':episodes,'recoveryEpisodes':recovery,'elapsed_s':time.monotonic()-start,'provenance':provenance,'noFinalHoldout':True})
if __name__=='__main__':main()
