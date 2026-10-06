"""Bounded stop/reset and CUDA integration acceptance, not a search lane."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
import torch
ROOT=Path('/home/tommie/Projects/actuator-model')
SOURCE=ROOT/'acf_simulink/experiments/rl_budget3h_20261005'
BASE=ROOT/'acf_simulink/validation/rl_budget3h_20261005'
sys.path.insert(0,str(SOURCE))
from env import RewardConfig
from train import Collector,get_scenarios,reward_value
from native_api import NativePlant
from policy import UnifiedPolicy
from ppo_multiscale import PPOTrainer,PPOConfig
OUT=BASE/'runner_checks/running_peak_v1'
report={'status':'running','scope':'Training-case acceptance only; one CUDA optimizer update, no selection or held-out access.'}
torch.set_num_threads(2);torch.manual_seed(84531)
try:
    plant=NativePlant(BASE/'native_bank_sustained',threads=2)
    scenarios=get_scenarios(plant,BASE/'scenario_bank_sustained/manifest.json')
    pure=RewardConfig(force_error_scale_N=.45,force_running_peak_weight=1,
        force_mse_weight=0,force_excess_weight=0,recenter_weight=0,effort_weight=0,
        current_change_weight=0,robot_change_weight=0,physical_failure_penalty=0)
    context=plant.create(0,seed=84531,record=True)
    peak=context.result.peak_normal_error_N;reward=0.0;old=np.zeros(2)
    action=np.array([[scenarios[0]['knownEquilibriumCurrent_A']-2,0.0]])
    for step in range(200):
        result=plant.step_batch([context],action)[0]
        prior=0.0 if step==0 else peak
        peak=max(peak,result.peak_normal_error_N)
        issued=result.obs[[13,15]]
        reward+=reward_value(result,issued,old,pure,previous_running_peak=prior,running_peak=peak)
        old=issued
        if result.terminated:break
    assert result.physical_failure
    record=context.take_record()
    true_peak=float(np.abs(record['truth'][:,0]-scenarios[0]['force_N']).max())
    assert peak==true_peak and abs(reward+(true_peak/.45)**2)<1e-10
    np.savez_compressed(OUT/'failure_native.npz',**record)
    report['physicalFailure']={'steps':step+1,'peak_N':peak,'nativeSamples':len(record['time']),
        'sumReward':reward,'telescopingResidual':abs(reward+(true_peak/.45)**2),
        'firstTime_s':float(record['time'][0]),'lastRecordedTime_s':float(record['time'][-1])}
    context.reset(0,seed=84532,baseline=False)
    assert context.result.peak_normal_error_N<peak
    report['failureResetClearsNativePeak']=True
    context.close()

    checkpoint=BASE/'reward_search_band_8203/checkpoints/generation_0005/policy_checkpoint.pt'
    saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
    policy=UnifiedPolicy(**saved['config']).cuda();policy.load_state_dict(saved['state_dict'])
    cfg=RewardConfig(force_error_scale_N=.45,force_running_peak_weight=1,
        force_mse_weight=1,force_excess_weight=0,return_scale=10,
        acceptable_travel_radius_m=.0075,travel_excess_weight=10,inside_travel_weight=0)
    collector=Collector(plant,scenarios,policy,OUT/'cuda_smoke',4,84533,cfg,
                        train_ids=[0,96,128],failure_native_limit=0)
    batch,collection=collector.collect(2048,0)
    before={name:parameter.detach().clone() for name,parameter in policy.named_parameters()}
    trainer=PPOTrainer(policy,PPOConfig(dt=.0001,force_lambda_time_s=.005,
        lambda_time_s=.3,chunk_length=512,burnin_steps=64,epochs=1,minibatch_sequences=4,
        actor_learning_rate=1e-5,critic_learning_rate=3e-4))
    metrics=trainer.update(batch)
    delta=max(float((parameter.detach()-before[name]).abs().max()) for name,parameter in policy.named_parameters())
    assert metrics['optimizer_steps']>0 and delta>0 and collection['forceRunningPeakCostSum']>0
    assert metrics['eligible_robot_actions']>0
    assert all(torch.isfinite(parameter).all() for parameter in policy.parameters())
    report['cudaIntegration']={'collection':collection,'optimization':metrics,'maximumParameterChange':delta}
    collector.close()
    report['sourceHashes']={name:hashlib.sha256((SOURCE/name).read_bytes()).hexdigest() for name in ('train.py','env.py','ppo_multiscale.py')}
    report['status']='passed'
except BaseException as error:
    report['status']='failed';report['error']=repr(error);raise
finally:(OUT/'results_extra.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
