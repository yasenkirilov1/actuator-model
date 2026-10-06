"""Independent native collector checks; bounded train-only fixtures."""
import importlib.util,json,sys,time
from pathlib import Path
import numpy as np
import torch
ROOT=Path('/home/tommie/Projects/actuator-model')
NEW=ROOT/'acf_simulink/experiments/rl_budget3h_20261005'
OLD=ROOT/'acf_simulink/experiments/rl_unified_20261005'
sys.path[:0]=[str(NEW),str(OLD)]
from train import Collector,get_scenarios,write_json
from env import FeatureEncoder,RewardConfig,FEATURE_NAMES
from policy import UnifiedPolicy
from native_api import NativePlant
OUT=ROOT/'acf_simulink/validation/rl_budget3h_20261005/runner_checks/acceptance_v1'
OUT.mkdir(exist_ok=False)
build=ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_thread_probe'
plant=NativePlant(build,threads=2)
scenarios=get_scenarios(plant,ROOT/'acf_simulink/validation/rl_unified_20261005/scenario_bank/manifest.json')
report={'status':'running','nativeBuild':str(build),'note':'Training cases only; short-horizon fixtures alter truncation bookkeeping, not the physical graph.'}
try:
    legacy_spec=importlib.util.spec_from_file_location('legacy_env',OLD/'env.py')
    legacy=importlib.util.module_from_spec(legacy_spec);sys.modules['legacy_env']=legacy;legacy_spec.loader.exec_module(legacy)
    rng=np.random.default_rng(89001);observations=rng.normal(size=(len(scenarios),18));steps=rng.integers(0,100000,len(scenarios))
    new=FeatureEncoder(scenarios).encode(observations,np.arange(len(scenarios)),steps)
    expected=legacy.feature_array(observations,scenarios,steps)
    report['featuresMaxResidual']=float(abs(new-expected).max());assert np.array_equal(new,expected)
    torch.manual_seed(89901);policy=UnifiedPolicy(len(FEATURE_NAMES),dt=1e-4).cpu()
    # Ensure the critic is informative, including at boundaries. A zero-valued
    # critic would make a bootstrap equivalence check vacuous.
    with torch.no_grad():
        policy.critic.weight.fill_(.21);policy.critic.bias.fill_(.7)
    short=[{**r,'duration_s':.004} for r in scenarios]
    collector=Collector(plant,short,policy,OUT/'bootstrap',2,89003,RewardConfig(),train_ids=[0,1],failure_native_limit=0)
    original=plant.step_batch;reference=[]
    def probe(contexts,actions):
        result=original(contexts,actions)
        raw=np.stack([r.obs for r in result])
        features=collector.encoder.encode(raw,collector.identities,collector.steps+1)
        _,value=collector.cpu.infer(features,state=collector.cpu.state.copy())
        reference.append(value)
        return result
    plant.step_batch=probe
    batch,metrics=collector.collect(64,0)
    plant.step_batch=original
    expected=np.asarray(reference);expected[batch.terminated.numpy()]=0
    actual=batch.next_values.numpy()
    report['bootstrapMaxResidual']=float(abs(expected-actual).max())
    report['truncationsChecked']=int(batch.truncated.sum());report['bootstrapReferenceNonzero']=int((abs(expected)>.01).sum())
    assert report['bootstrapMaxResidual']==0 and report['truncationsChecked']>0 and report['bootstrapReferenceNonzero']>0
    state=collector.state_dict()
    resumed=Collector(plant,short,policy,OUT/'resumed',2,777,RewardConfig(),train_ids=[0,1],failure_native_limit=0)
    resumed.load_state_dict(state)
    one,m1=collector.collect(37,1);two,m2=resumed.collect(37,1)
    for name in ('observations','latent_actions','log_probs','values','rewards','terminated','truncated','next_values','reset_before','action_mask','initial_state'):
        assert torch.equal(getattr(one,name),getattr(two,name)),name
    report['exactResumeRolloutPassed']=True
    collector.close();resumed.close()
    # A deliberate contact-loss fixture proves that all requests, exact seed,
    # endpoint and representative full native history survive a physical stop.
    failure=Collector(plant,scenarios,policy,OUT/'failure',1,89009,RewardConfig(),train_ids=[0],failure_native_limit=1)
    sample=failure.cpu.sample
    def hard_contact_loss(*args,**kwargs):
        output=sample(*args,**kwargs);output['actions'][:,0]=-1;return output
    failure.cpu.sample=hard_contact_loss
    batch,metrics=failure.collect(32,0)
    failure.close()
    rows=[json.loads(x) for x in (OUT/'failure/training_episodes.jsonl').read_text().splitlines()]
    failures=[r for r in rows if r['terminated']]
    assert len(failures)>0 and len(list((OUT/'failure/failed_training_actions').glob('failed_[0-9]*.npz')))>=len(failures)
    assert failures[0]['exactActionReplayPassed']
    report['retainedPhysicalFailures']=len(failures);report['representativeNativeReplayPassed']=True
    report['behaviorReplayMeanMaxResidual']=metrics['behaviorReplayMeanMaxResidual']
    report['behaviorReplayLogProbMaxResidual']=metrics['behaviorReplayLogProbMaxResidual']
    report['status']='passed'
except BaseException as error:
    report['status']='failed';report['error']=repr(error);write_json(OUT/'results.json',report);raise
write_json(OUT/'results.json',report)
print(json.dumps(report,indent=2))
