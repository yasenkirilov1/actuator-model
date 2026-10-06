from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys
import time

base=Path(__file__).resolve().parent/'run_v2';base.mkdir()
python=sys.executable
driver=Path('acf_simulink/experiments/rl_budget3h_20261005/final_evaluate.py').resolve()
source=Path('acf_simulink/validation/rl_budget3h_20261005/executors/v5').resolve()
checkpoint=Path('acf_simulink/validation/rl_budget3h_20261005/reward_search_band_8203/checkpoints/generation_0005/policy_checkpoint.pt').resolve()
native=Path('acf_simulink/validation/rl_budget3h_20261005/native_bank_sustained').resolve()
bank=Path('acf_simulink/validation/rl_budget3h_20261005/scenario_bank_sustained/manifest.json').resolve()
common=[python,str(driver),'--checkpoint',str(checkpoint),'--policy-source-dir',str(source),
        '--native-build',str(native),'--bank',str(bank)]
results={'status':'running','trainOnly':True,'guards':{},'benchmarks':{}}
def run(name,options,expect_success=True,common_args=common):
    folder=base/name
    command=common_args+['--output',str(folder)]+options
    started=time.monotonic();completed=subprocess.run(command,capture_output=True,text=True)
    (base/(name+'.log')).write_text(completed.stdout+completed.stderr)
    outcome=json.loads((folder/'results.json').read_text()) if (folder/'results.json').exists() else {
        'status':'rejected_before_output_creation','error':{'message':completed.stderr.splitlines()[-1]}}
    assert (completed.returncode==0)==expect_success,(name,completed.stdout,completed.stderr)
    print(name,completed.returncode,outcome.get('status'),time.monotonic()-started,flush=True)
    return outcome

for name,options in [('guard_default_holdout',[]),('guard_fixture_holdout',['--fixture-train','--indices','64']),
                     ('guard_reset_holdout',['--fixture-reset-state-every-step'])]:
    outcome=run(name,options,False)
    assert not (base/name/'input_lock.json').exists()
    results['guards'][name]={'rejected':True,'reason':outcome['error']['message'],'noPhysicalEpisode':True}

bad_native=base/'bad_native';bad_native.mkdir()
for name in ['metadata.json','scenarios.json','bridge_build.json','native_parameter_bank.c','acf_rl_native_context.c']:
    shutil.copy2(native/name,bad_native/name)
(bad_native/'libacf_native.so').write_bytes(b'intentionally bad private seal')
bad_common=common.copy();bad_common[bad_common.index('--native-build')+1]=str(bad_native)
outcome=run('guard_bad_native',['--fixture-train','--indices','0'],False,bad_common)
assert 'seal mismatch' in outcome['error']['message']
results['guards']['guard_bad_native']={'rejected':True,'reason':outcome['error']['message'],'noPhysicalEpisode':True}

import torch
saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
saved['features']=saved['features'].copy();saved['features'][0]='intentionally_wrong_private_feature'
bad_checkpoint=base/'bad_checkpoint.pt';torch.save(saved,bad_checkpoint)
bad_common=common.copy();bad_common[bad_common.index('--checkpoint')+1]=str(bad_checkpoint)
outcome=run('guard_bad_feature',['--fixture-train','--indices','0'],False,bad_common)
assert 'feature names differ' in outcome['error']['message'] and not (base/'guard_bad_feature'/'executor').exists()
results['guards']['guard_bad_feature']={'rejected':True,'reason':outcome['error']['message'],'noPhysicalEpisode':True}
saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
saved['provenance']['physicalActionScales']['robotRequest_m_s']=.005
bad_scale=base/'bad_action_scale.pt';torch.save(saved,bad_scale)
bad_common=common.copy();bad_common[bad_common.index('--checkpoint')+1]=str(bad_scale)
outcome=run('guard_bad_action_scale',['--fixture-train','--indices','0'],False,bad_common)
assert 'physical action scales differ' in outcome['error']['message'] and not (base/'guard_bad_action_scale'/'executor').exists()
results['guards']['guard_bad_action_scale']={'rejected':True,'reason':outcome['error']['message'],'noPhysicalEpisode':True}

for workers,name in [(1,'aggregate_serial2s'),(4,'aggregate_workers4_2s')]:
    outcome=run(name,['--fixture-train','--indices','0,1,2,3','--baseline','--aggregate-only',
                      '--workers',str(workers)])
    assert outcome['status']=='fixture_complete' and outcome['completedRecordings']==8
    assert outcome['totalNativeSamples']==8*600001
    assert outcome['aggregateReplaysComplete'] and all(r['deterministicReplayMatched'] for r in outcome['aggregateReplays'])
    assert all(r['sourceActionMappingMaxResidual_SI']<1e-6 for r in outcome['outcomes']['selected']['rows'])
    assert outcome['actorContract']['physicalActionScales']['robotRequest_m_s']==.01
    assert any(r['nonzeroRobotRequests']>0 for r in outcome['outcomes']['selected']['rows'])
    results['benchmarks'][name]={'workers':workers,'elapsed_s':outcome['elapsed_s'],
        'uniqueRecordings':8,'uniqueNativeSamples':outcome['totalNativeSamples'],
        'replaySamples':outcome['aggregateReplayNativeSamples'],
        'summedOrdinaryCaseElapsed_s':sum(r['elapsed_s'] for phase in outcome['outcomes'].values() for r in phase['rows'])}

serial=json.loads((base/'aggregate_serial2s/results.json').read_text())
parallel=json.loads((base/'aggregate_workers4_2s/results.json').read_text())
for phase in ['baseline','selected']:
    for left,right in zip(serial['outcomes'][phase]['rows'],parallel['outcomes'][phase]['rows'],strict=True):
        for key in ['identity','seed','rawNativeSha256','rawActionsSha256','nativeSamples','peakNormalForceError_N',
                    'rmsNormalForceError_N','maxTravelBandExcess_m','passed','travelBandPassed']:
            assert left[key]==right[key],(phase,left['identity'],key,left[key],right[key])
results['serialFourWorkerBitExact']=True
results['status']='complete';results['driverSha256']=hashlib.sha256(driver.read_bytes()).hexdigest()
results['freshHoldoutEpisodesExecuted']=0;results['qualification700EpisodesExecuted']=0
(base/'acceptance_results.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results),flush=True)
