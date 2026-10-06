"""Paired development-only screen of a frozen policy on a versioned sensor bank."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime,timezone
import argparse,json,shutil,time
import numpy as np
from reward_search import EpisodeEvaluator,aggregate,build_evaluator,load_scenarios,joint_band_score,sha,write_json

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--native-build',required=True);parser.add_argument('--bank',required=True)
    parser.add_argument('--policy',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--workers',type=int,default=4)
    a=parser.parse_args();folder=Path(a.output);folder.mkdir(parents=True,exist_ok=False)
    source=Path(a.policy).resolve();policy_folder=folder/'frozen_policy'
    shutil.copytree(source.parent,policy_folder)
    shutil.copy2(Path(__file__),folder/Path(__file__).name)
    scenarios=load_scenarios(a.native_build,a.bank)
    for row in scenarios:row['robotActionScale_m_s']=.01
    ids=[i for i,row in enumerate(scenarios) if row['split']=='selection']
    assert len(ids)==28 and not set(ids).intersection(range(64,96))
    write_json(folder/'consumed_scenarios.json',scenarios)
    library=build_evaluator(a.native_build,policy_folder,folder)
    evaluator=EpisodeEvaluator(library,scenarios)
    weights=np.fromfile(policy_folder/'policy_weights.f32',dtype=np.float32)
    config={'arguments':vars(a),'startedUtc':datetime.now(timezone.utc).isoformat(),
      'policyCheckpointSha256':sha(source),'nativeLibrarySha256':sha(Path(a.native_build)/'libacf_native.so'),
      'scenarioManifestSha256':sha(a.bank),'physicalActionScales':{'currentOffset_A':2.,'robotRequest_m_s':.01},
      'scope':'Paired development screen. Final scenarios 64 through 95 are not evaluated.'}
    write_json(folder/'run_configuration.json',config)
    start=time.monotonic();comparisons=[]
    with ThreadPoolExecutor(max_workers=a.workers) as pool:
        for baseline in [True,False]:
            rows=list(pool.map(lambda index:evaluator.evaluate(weights,[index],baseline=baseline)[0],ids))
            for row in rows:
                row['recenterPassed']=True;row['recenterScreenApplied']=False
                row['finalBandExcess_m']=max(abs(row['finalTravelError_m'])-.0075,0)
            result=aggregate(rows);result['jointScore']=joint_band_score(rows)
            result['controller']='native_baseline' if baseline else 'frozen_learned_policy'
            result['rank']=[result['physicalFailures'],-result['passed'],result['jointScore'],
                result['worstNormalForceError_N'],result['meanRmsNormalForceError_N']]
            comparisons.append(result)
            write_json(folder/'partial_results.json',{'comparisons':comparisons})
            print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
    # Preserve one complete failed physical replay for each controller/failure
    # type. Every other identity, exact policy and deterministic seed is retained.
    replays=[]
    for result,baseline in zip(comparisons,[True,False]):
        seen=set()
        for row in result['rows']:
            if not row['physicalFailure']:continue
            kind='contact_loss' if row['contactLoss'] else 'travel_stop'
            if kind in seen:continue
            seen.add(kind);target=folder/'failed_native'/result['controller']/kind
            replay=evaluator.evaluate(weights,[row['scenario']],record_dir=target,baseline=baseline)
            write_json(target/'replay.json',{'rows':replay,'excludedFromScreenCounts':True})
            replays.extend(replay)
    output={'status':'complete','comparisons':comparisons,'episodeComparisons':sum(len(x['rows']) for x in comparisons),
      'actorDecisions':sum(x['controllerDecisions'] for x in comparisons),'elapsed_s':time.monotonic()-start,
      'recoveryEpisodesExcluded':len(replays),'configuration':config}
    write_json(folder/'results.json',output)
    print(json.dumps({k:v for k,v in output.items() if k not in ['comparisons','configuration']}),flush=True)

if __name__=='__main__':main()
