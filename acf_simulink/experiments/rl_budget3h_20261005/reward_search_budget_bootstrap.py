"""Native-reward selection of DC-preserving history attenuation for a new sensor bank."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
import argparse,copy,json,sys,time,shutil
import numpy as np
import torch
HERE=Path(__file__).resolve().parent
from policy_export import load_policy,export_policy,flat_weights
from reward_search import EpisodeEvaluator,build_evaluator,load_scenarios,aggregate,joint_band_score,write_json,sha
from env import FEATURE_NAMES

def attenuate(base,scale):
    policy=copy.deepcopy(base);changes=[]
    with torch.no_grad():
        for pair in range(4,12):
            unit=2*pair;gate=policy.recurrent.layers[0].bias[64+unit].item()
            highway=1/(1+np.exp(-gate));dc=.1*(highway*1.25+(1-highway)*2)
            original=float(policy.actor.weight[0,unit]);aux=1 if pair<8 else 2
            policy.actor.weight[0,unit]*=scale;policy.actor.weight[0,unit+1]*=scale
            policy.actor_aux.weight[0,aux]+=(1-scale)*original*dc
            changes.append({'pair':pair,'originalPairedReadout':original,'scale':scale,
              'directAuxiliaryIndex':aux,'directDelta':(1-scale)*original*dc})
    return policy,changes

def main():
    p=argparse.ArgumentParser();p.add_argument('--policy',required=True);p.add_argument('--native-build',required=True)
    p.add_argument('--bank',required=True);p.add_argument('--output',required=True);p.add_argument('--workers',type=int,default=4)
    a=p.parse_args();torch.set_num_threads(1);out=Path(a.output);out.mkdir(parents=True,exist_ok=False)
    shutil.copy2(Path(__file__),out/Path(__file__).name)
    base=load_policy(a.policy);scenarios=load_scenarios(a.native_build,a.bank)
    for row in scenarios:row['robotActionScale_m_s']=.01
    train=[15,27,31,35,39,40,42,46,*range(128,140)]
    selection=[i for i,row in enumerate(scenarios) if row['split']=='selection']
    assert all(scenarios[i]['split']=='train' for i in train)
    provenance={'method':'DC_preserving_attenuation_selected_by_native_episode_rewards',
      'sourceCheckpoint':str(Path(a.policy).resolve()),'sourceCheckpointSha256':sha(a.policy),
      'noTeacherActions':True,'physicalActionScales':{'currentOffset_A':2.,'robotRequest_m_s':.01}}
    candidates={}
    for scale in [.25,.5,.75,1.]:
        policy,changes=attenuate(base,scale);name=f'history_{scale:g}'.replace('.','p');target=out/name
        export_policy(policy,target,FEATURE_NAMES,{**provenance,'historyScale':scale})
        write_json(target/'initialization.json',{'changes':changes,'linearizedDcPreserved':True})
        candidates[name]={'weights':flat_weights(policy),'directory':str(target.resolve()),'scale':scale}
    first=next(iter(candidates.values()));library=build_evaluator(a.native_build,first['directory'],out)
    evaluator=EpisodeEvaluator(library,scenarios);reports=[];start=time.monotonic();recovery=0
    with ThreadPoolExecutor(max_workers=a.workers) as pool:
        def run(name,ids,split):
            nonlocal recovery
            candidate=candidates[name]
            rows=list(pool.map(lambda i:evaluator.evaluate(candidate['weights'],[i])[0],ids))
            for row in rows:
                row['recenterPassed']=True;row['recenterScreenApplied']=False
                row['finalBandExcess_m']=max(abs(row['finalTravelError_m'])-.0075,0)
            result=aggregate(rows);result.update(name=name,split=split,historyScale=candidate['scale'],
              policyDirectory=candidate['directory'],jointScore=joint_band_score(rows))
            result['rank']=[result['physicalFailures'],-result['passed'],result['jointScore'],
                result['worstNormalForceError_N'],result['meanRmsNormalForceError_N']]
            failed=[r['scenario'] for r in rows if r['physicalFailure']]
            if failed:
                target=out/name/(split+'_failed_native')
                replay=evaluator.evaluate(candidate['weights'],failed[:1],record_dir=target)
                write_json(target/'replay.json',{'rows':replay,'excludedFromComparisonCounts':True});recovery+=1
            reports.append(result)
            write_json(out/'results.json',{'status':'running','comparisons':reports,
              'episodeComparisons':sum(len(r['rows']) for r in reports),'recoveryEpisodes':recovery})
            print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
        for name in candidates:run(name,train,'train')
        shortlisted=sorted(reports,key=lambda r:r['rank'])[:2]
        for result in shortlisted:run(result['name'],selection,'selection')
    selected=min((r for r in reports if r['split']=='selection'),key=lambda r:r['rank'])
    write_json(out/'results.json',{'status':'complete','comparisons':reports,'selected':selected['name'],
      'selectedPolicy':str(Path(selected['policyDirectory'])/'policy_checkpoint.pt'),
      'episodeComparisons':sum(len(r['rows']) for r in reports),'recoveryEpisodes':recovery,
      'elapsed_s':time.monotonic()-start,'provenance':provenance,'nativeLibrarySha256':sha(Path(a.native_build)/'libacf_native.so'),
      'scope':'Training and selection only, no final cases and no CMA optimizer updates.'})

if __name__=='__main__':main()
