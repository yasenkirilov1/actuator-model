"""Native array collector parity, resume and measured speed comparison."""
from pathlib import Path
import json,sys,time
import numpy as np
import torch
ROOT=Path('/home/tommie/Projects/actuator-model');MOD=ROOT/'acf_simulink/validation/rl_budget3h_20261005/runner_checks/fast_acceptance_v1/frozen_sources'
sys.path.insert(0,str(MOD))
from train import Collector as Reference,get_scenarios,write_json
from train_v2 import Collector as Fast
from env import RewardConfig,FEATURE_NAMES
from policy import UnifiedPolicy
from native_api import NativePlant
OUT=ROOT/'acf_simulink/validation/rl_budget3h_20261005/runner_checks/fast_acceptance_v1';OUT.mkdir(exist_ok=False)
torch.set_num_threads(4);torch.manual_seed(89501)
plant=NativePlant(ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_thread_probe',threads=4)
scenarios=get_scenarios(plant,ROOT/'acf_simulink/validation/rl_unified_20261005/scenario_bank/manifest.json')
policy=UnifiedPolicy(len(FEATURE_NAMES),dt=1e-4)
with torch.no_grad():policy.critic.weight.fill_(.1);policy.critic.bias.fill_(.3)
short=[{**r,'duration_s':.017} for r in scenarios]
report={'status':'running','scope':'Equivalent collector transport and vectorized scoring on same native graph; training identities only.'}
try:
    one=Reference(plant,short,policy,OUT/'reference',8,89500,RewardConfig(return_scale=10),train_ids=[0,1],failure_native_limit=1)
    two=Fast(plant,short,policy,OUT/'fast',8,89500,RewardConfig(return_scale=10),train_ids=[0,1],failure_native_limit=1)
    b1,m1=one.collect(512,0);b2,m2=two.collect(512,0)
    report['parity']={}
    for name in ('observations','latent_actions','log_probs','values','rewards','terminated','truncated','next_values','reset_before','action_mask','initial_state','state_inputs','log_probs_by_action'):
        a,b=getattr(b1,name),getattr(b2,name)
        error=float((a.double()-b.double()).abs().max());report['parity'][name]=error
        assert error==0,name
    assert np.array_equal(one.last_rollout[1]['physical_interval'],two.last_rollout[1]['physical_interval'])
    report['nativeMetricsBitIdentical']=True
    state=two.state_dict()
    resumed=Fast(plant,short,policy,OUT/'resumed',8,89501,RewardConfig(return_scale=10),train_ids=[0,1],failure_native_limit=1)
    resumed.load_state_dict(state)
    ba,ma=two.collect(101,1);bb,mb=resumed.collect(101,1)
    assert torch.equal(ba.observations,bb.observations) and torch.equal(ba.latent_actions,bb.latent_actions)
    report['exactResumePassed']=True
    one.close();two.close();resumed.close()
    # Native failure parity and full representative replay, including masks.
    one=Reference(plant,scenarios,policy,OUT/'reference_failure',1,89503,RewardConfig(),train_ids=[0],failure_native_limit=1)
    two=Fast(plant,scenarios,policy,OUT/'fast_failure',1,89503,RewardConfig(),train_ids=[0],failure_native_limit=1)
    for collector in (one,two):
        original=collector.cpu.sample
        def pressure(*args,_sample=original,**kwargs):
            output=_sample(*args,**kwargs);output['actions'][:,0]=-1;return output
        collector.cpu.sample=pressure
    b1,m1=one.collect(32,0);b2,m2=two.collect(32,0)
    assert torch.equal(b1.observations,b2.observations) and torch.equal(b1.next_values,b2.next_values)
    assert m1['physicalFailures']==m2['physicalFailures']>0
    report['retainedFailureCount']=m2['physicalFailures'];report['nativeFailureReplayPassed']=two.native_failures_saved==1
    one.close();two.close()
    timing={}
    for cls,label in ((Reference,'objects'),(Fast,'arrays')):
        collector=cls(plant,scenarios,policy,OUT/f'timing_{label}',16,89507,RewardConfig(),failure_native_limit=0,save_every=0)
        collector.collect(128,0)
        started=time.perf_counter();batch,metrics=collector.collect(2048,1);elapsed=time.perf_counter()-started
        timing[label]={'elapsed_s':elapsed,'decisions':32768,'decisionsPerSecond':32768/elapsed}
        collector.close()
    report['timing']=timing;report['speedRatio']=timing['objects']['elapsed_s']/timing['arrays']['elapsed_s']
    report['status']='passed'
except BaseException as error:
    report['status']='failed';report['error']=repr(error);write_json(OUT/'results.json',report);raise
write_json(OUT/'results.json',report);print(json.dumps(report,indent=2))
