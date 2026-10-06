from pathlib import Path
import hashlib
import json
import subprocess
import sys
import time

root=Path('acf_simulink/validation/rl_puffer_probe_20261005_final_driver_checks/memory_ablation_gen5').resolve()
root.mkdir()
source=Path('acf_simulink/validation/rl_budget3h_20261005/executors/v5').resolve()
checkpoint=Path('acf_simulink/validation/rl_budget3h_20261005/reward_search_band_8203/checkpoints/generation_0005/policy_checkpoint.pt').resolve()
native=Path('acf_simulink/validation/rl_budget3h_20261005/native_bank_sustained').resolve()
bank=Path('acf_simulink/validation/rl_budget3h_20261005/scenario_bank_sustained/manifest.json').resolve()
driver=Path('acf_simulink/experiments/rl_budget3h_20261005/final_evaluate.py').resolve()
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert digest(checkpoint)=='48618b8d21c4f5920d870a05f5eb198c958632f51ffab6f0abfd6a8255ef0ad4'
cases=json.loads(bank.read_text())['cases'];ids=[0,40,96,139]
assert all(cases[i]['split']=='train' for i in ids)
contract={'status':'fixed_before_development_outcomes','checkpointSha256':digest(checkpoint),
    'bankSha256':digest(bank),'nativeLibrarySha256':digest(native/'libacf_native.so'),
    'driverSha256':digest(driver),'policySourceHashes':{p.name:digest(p) for p in source.iterdir() if p.is_file()},
    'cases':[{'scenario':i,'identity':cases[i]['identity'],'seed':cases[i]['seed'],'duration_s':cases[i]['duration_s']} for i in ids],
    'interpretation':'Development memory-reliance test. Same weights/causal sensors/observer/plant/queues; only learned64-state carry resets every100us in counterfactual. Not optimized memoryless baseline or final qualification.',
    'freshHoldoutEpisodesExecuted':0,'qualification700EpisodesExecuted':0}
(root/'development_lock.json').write_text(json.dumps(contract,indent=2)+'\n')
for label,extra in [('carried',[]),('reset',['--fixture-reset-state-every-step'])]:
    command=[sys.executable,str(driver),'--checkpoint',str(checkpoint),'--policy-source-dir',str(source),
        '--native-build',str(native),'--bank',str(bank),'--output',str(root/label),
        '--fixture-train','--indices',','.join(map(str,ids))]+extra
    started=time.monotonic();run=subprocess.run(command,capture_output=True,text=True)
    (root/(label+'.log')).write_text(run.stdout+run.stderr)
    print(label,run.returncode,time.monotonic()-started,run.stdout,flush=True)
    run.check_returncode()
normal=json.loads((root/'carried/results.json').read_text());reset=json.loads((root/'reset/results.json').read_text())
paired=[]
for a,b in zip(normal['outcomes']['selected']['rows'],reset['outcomes']['selected']['rows'],strict=True):
    assert a['identity']==b['identity'] and a['seed']==b['seed']
    paired.append({'scenario':a['scenario'],'identity':a['identity'],'seed':a['seed'],
        'carriedForcePassed':a['passed'],'resetForcePassed':b['passed'],
        'carriedPeak_N':a['peakNormalForceError_N'],'resetPeak_N':b['peakNormalForceError_N'],
        'carriedRms_N':a['rmsNormalForceError_N'],'resetRms_N':b['rmsNormalForceError_N'],
        'resetMinusCarriedPeak_N':b['peakNormalForceError_N']-a['peakNormalForceError_N'],
        'carriedEndTravelBandExcess_m':a['finalTravelBandExcess_m'],
        'resetEndTravelBandExcess_m':b['finalTravelBandExcess_m'],
        'carriedWithinBandFraction':a['travelWithinBandFraction'],
        'resetWithinBandFraction':b['travelWithinBandFraction'],
        'carriedIssuedFrames':a['issuedRobotFrames'],'resetIssuedFrames':b['issuedRobotFrames'],
        'carriedPhysicalFailure':a['physicalFailure'],'resetPhysicalFailure':b['physicalFailure']})
training=json.loads(Path('acf_simulink/validation/rl_budget3h_20261005/reward_search_band_8203/results.json').read_text())
results={'status':'complete','fixtureOnly':True,'lockedCheckpointSha256':digest(checkpoint),
    'storedTrainableParameters':normal['actorContract']['storedTrainableParameters'],
    'selectionStageRewardOptimizedDimensions':training['parameterCountOptimized'],
    'recurrentActionReadout':normal['actorContract']['recurrentActionReadout'],
    'paired':paired,'totalUniqueDevelopmentRecordings':8,
    'totalNativeSamples':normal['totalNativeSamples']+reset['totalNativeSamples'],
    'carriedResultsSha256':digest(root/'carried/results.json'),'resetResultsSha256':digest(root/'reset/results.json'),
    'interpretation':contract['interpretation'],'freshHoldoutEpisodesExecuted':0,
    'qualification700EpisodesExecuted':0}
(root/'results.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results),flush=True)
