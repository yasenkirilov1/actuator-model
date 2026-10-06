"""Credit-horizon, boundary and exact-equivalence checks for opt-in PPO."""
from pathlib import Path
from types import SimpleNamespace
import sys,json,math
import numpy as np
import torch
ROOT=Path('/home/tommie/Projects/actuator-model');sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from policy import UnifiedPolicy
from ppo import PPOTrainer as BaseTrainer,PPOConfig as BaseConfig,RolloutBatch,compute_gae
from ppo_multiscale import PPOTrainer,PPOConfig,multiscale_advantages
OUT=ROOT/'acf_simulink/validation/rl_budget3h_20261005/runner_checks/multiscale_v1';OUT.mkdir(exist_ok=False)
RUN=ROOT/'acf_simulink/validation/rl_budget3h_20261005/ppo_warm_10k_8301'
torch.set_num_threads(2);torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
report={'status':'running'}
try:
    # A reward100ms after issuing a robot command still receives long credit;
    # force credit is intentionally much shorter. This uses no simulator truth.
    reward=torch.zeros(1001,1,dtype=torch.float64);reward[-1,0]=-1
    zero=torch.zeros_like(reward);end=torch.zeros_like(reward,dtype=torch.bool)
    fixture=SimpleNamespace(rewards=reward,values=zero,next_values=zero,terminated=end,truncated=end.clone())
    cfg=PPOConfig(dt=.0001,lambda_time_s=.3,force_lambda_time_s=.02,gamma_time_s=1)
    channels,returns=multiscale_advantages(fixture,cfg)
    force_expected=-math.exp(-.1*(1/.02+1));robot_expected=-math.exp(-.1*(1/.3+1))
    assert abs(float(channels[0,0,0])-force_expected)<1e-12
    assert abs(float(channels[0,0,1])-robot_expected)<1e-12
    assert abs(float(channels[0,0,1]))>.6
    report['100msDelayedRewardCredit']={'force':float(channels[0,0,0]),'robot':float(channels[0,0,1]),'robotDelayPreserved':True}
    long,long_returns=compute_gae(reward,zero,zero,end,end,cfg.gamma,cfg.gae_lambda)
    assert torch.equal(channels[...,1],long) and torch.equal(returns,long_returns)
    report['criticTargetsAndRobotAdvantagesBitIdentical']=True
    # Later-episode reward cannot pass either kind of boundary. Truncation
    # preserves its own final-observation bootstrap; physical stop drops it.
    boundary={}
    for mode in ('terminated','truncated'):
        flags=end.clone();flags[400]=True;next_value=zero.clone();next_value[400]=2
        f=SimpleNamespace(rewards=reward,values=zero,next_values=next_value,
                          terminated=flags if mode=='terminated' else end,
                          truncated=flags if mode=='truncated' else end)
        actual,target=multiscale_advantages(f,cfg)
        clipped=reward.clone();clipped[401:]=0
        f2=SimpleNamespace(**{**vars(f),'rewards':clipped})
        expected,_=multiscale_advantages(f2,cfg)
        assert torch.equal(actual[:401],expected[:401])
        expected_boundary=0 if mode=='terminated' else 2*cfg.gamma
        assert abs(float(actual[400,0,0])-expected_boundary)<1e-12
        assert abs(float(actual[400,0,1])-expected_boundary)<1e-12
        boundary[mode]={'laterEpisodeLeakage':False,'boundaryBootstrap':float(actual[400,0,0])}
    report['boundaries']=boundary
    raw=dict(np.load(RUN/'rollouts/rollout_000000.npz'))
    T,N=2048,4
    fields=('observations','latent_actions','log_probs','values','rewards','terminated','truncated','next_values',
            'reset_before','action_mask','initial_state','state_inputs','log_probs_by_action')
    arrays={}
    for name in fields:
        value=raw[name]
        arrays[name]=value[:,:N] if name=='initial_state' else (value[:T,:,:N] if name=='state_inputs' else value[:T,:N])
    batch=RolloutBatch(**{name:torch.as_tensor(value,device='cuda') for name,value in arrays.items()})
    checkpoint=torch.load(RUN/'rollouts/initial_policy/policy_checkpoint.pt',map_location='cpu',weights_only=False)
    common=dict(dt=.0001,lambda_time_s=.3,gamma_time_s=1,chunk_length=512,burnin_steps=64,epochs=2,minibatch_sequences=4)
    def update(cls,config):
        policy=UnifiedPolicy(**checkpoint['config']).cuda();policy.load_state_dict(checkpoint['state_dict'])
        trainer=cls(policy,config)
        torch.manual_seed(89702);torch.cuda.manual_seed_all(89702)
        metrics=trainer.update(batch)
        return {name:p.detach().cpu().clone() for name,p in policy.state_dict().items()},metrics
    baseline,metrics=update(BaseTrainer,BaseConfig(**common))
    for label,force in (('disabled',None),('equal',.3)):
        state,result=update(PPOTrainer,PPOConfig(**common,force_lambda_time_s=force))
        differences={name:float((state[name].double()-baseline[name].double()).abs().max()) for name in state}
        assert max(differences.values())==0,label
        report[label+'UpdateBitIdentical']=True
    multi,result=update(PPOTrainer,PPOConfig(**common,force_lambda_time_s=.02))
    report['multiscaleUpdate']=result
    report['changedForceParameters']=any(not torch.equal(multi[name],baseline[name]) for name in multi if name.startswith('actor'))
    assert report['changedForceParameters'] and result['eligible_robot_actions']>0
    report['status']='passed'
except BaseException as error:
    report['status']='failed';report['error']=repr(error);raise
finally:(OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
