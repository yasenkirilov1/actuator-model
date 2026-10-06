"""Reward-train one recurrent actor on the generated native Simulink plant.

Whole-episode selection is separate from held-out qualification. Baseline
actions never enter actor/critic features or optimization targets. The inner
current servo, acquisition, observers, hard robot gate and queue stay native.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict, fields
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import time
import traceback
import os
import signal
import inspect
import sys

import numpy as np
import torch

from env import (CPolicy, DT, FEATURE_NAMES, FeatureEncoder, RewardConfig, action_mask,
                 feature_array, physical_actions)
from policy import UnifiedPolicy
from policy_export import export_policy
from ppo import PPOConfig, PPOTrainer, RolloutBatch

_PREPARING_OUTPUT = None


def write_json(path, value):
    def native_scalar(obj):
        if isinstance(obj,np.generic):return obj.item()
        raise TypeError(f"Unsupported JSON value {type(obj).__name__}")
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False,default=native_scalar)+"\n")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def get_scenarios(plant, bank):
    rows=json.loads(Path(bank).read_text())["cases"]
    assert len(rows)==len(plant.scenarios)
    result=[]
    for row, identified in zip(rows,plant.scenarios):
        assert int(row["scenario"])==int(identified["scenario"])
        merged={**row,**identified}
        merged["identifiedRobotResponse_s"]=identified["identifiedRobotResponseTime_s"]
        merged["identifiedCurrentResponse_s"]=identified["currentResponse_s"]
        result.append(merged)
        assert "identifiedMassB_kg" in result[-1]
    return result


def initial_result(context):
    return context.result


def advance(plant,contexts,actions,repeat=1):
    if repeat==1:return plant.step_batch(contexts,actions)
    from native_repeat import repeat_native_step
    return repeat_native_step(plant,contexts,actions,repeat)


def advance_arrays(plant,contexts,actions,repeat=1):
    """Use the same native graph with array transport and unchanged scoring."""
    if repeat==1 and hasattr(plant,'step_batch_arrays'):
        return plant.step_batch_arrays(contexts,actions)
    if hasattr(plant,'step_repeat_batch_arrays'):
        return plant.step_repeat_batch_arrays(contexts,actions,repeat)
    from native_api import METRIC_NAMES
    rows=advance(plant,contexts,actions,repeat)
    return (np.stack([r.obs for r in rows]),np.stack([r.truth_last for r in rows]),
            np.stack([r.comparator for r in rows]),
            np.array([[r.metrics[name] for name in METRIC_NAMES] for r in rows],np.float64))


def array_rewards(metrics,issued,previous,cfg):
    peak=metrics[:,6]/cfg.force_error_scale_N
    cost=(cfg.force_mse_weight*metrics[:,7]/cfg.force_error_scale_N**2
          +cfg.force_excess_weight*np.maximum(peak-1,0)**2
          +cfg.recenter_weight*metrics[:,10]/cfg.recenter_scale_m**2
          +cfg.effort_weight*metrics[:,13]/cfg.effort_scale_A**2
          +cfg.current_change_weight*((issued[:,0]-previous[:,0])/cfg.current_change_scale_A)**2
          +cfg.robot_change_weight*((issued[:,1]-previous[:,1])/cfg.robot_change_scale_m_s)**2)
    failed=(metrics[:,17]>0)|(metrics[:,18]>0)
    reward=-(metrics[:,0]*cost+cfg.physical_failure_penalty*failed)*cfg.return_scale
    if not np.isfinite(reward).all():raise FloatingPointError('Non-finite native reward; preserve rejected attempt')
    return reward


def reward_value(result, request, previous_request, cfg):
    peak=result.peak_normal_error_N/cfg.force_error_scale_N
    cost=(cfg.force_mse_weight*result.mean_squared_normal_error_N2/cfg.force_error_scale_N**2
          +cfg.force_excess_weight*max(peak-1,0)**2
          +cfg.recenter_weight*result.mean_squared_travel_error_m2/cfg.recenter_scale_m**2
          +cfg.effort_weight*result.mean_squared_current_A2/cfg.effort_scale_A**2
          +cfg.current_change_weight*((request[0]-previous_request[0])/cfg.current_change_scale_A)**2
          +cfg.robot_change_weight*((request[1]-previous_request[1])/cfg.robot_change_scale_m_s)**2)
    reward=-float(result.elapsed_s)*cost-cfg.physical_failure_penalty*bool(result.physical_failure)
    if not np.isfinite(reward):
        raise FloatingPointError("Non-finite native reward; preserve rejected attempt")
    return float(reward)*cfg.return_scale


def evaluate(plant, scenarios, indices, policy, directory, name, seed_offset=0,
             baseline=False, save_histories=False, centering_baseline=None,
             minimum_centering_fraction=.5):
    """A whole native episode set; no gradients or test-driven retuning."""
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    chosen=[scenarios[i] for i in indices]
    repeat=round(policy.dt/DT)
    assert repeat>=1 and abs(repeat*DT-policy.dt)<1e-12 and 100%repeat==0
    cpu=CPolicy(policy,directory/f"{name}_inference",len(chosen),{"phase":name,"baseline":baseline})
    contexts=[plant.create(scenario=i,seed=int(scenarios[i]["seed"])+seed_offset,
                           baseline=baseline,record=save_histories) for i in indices]
    last=[initial_result(c) for c in contexts]
    steps=np.zeros(len(chosen),np.int64);resets=np.ones(len(chosen),np.uint8)
    finished=np.zeros(len(chosen),bool)
    rows=[{"scenario":int(i),"identity":r["identity"],"seed":int(r["seed"])+seed_offset,
           "baseline":baseline,"peakNormalForceError_N":last[j].peak_normal_error_N,
           "physicalFailure":bool(last[j].physical_failure),"normalSquaredErrorSum_N2":0.0,
           "nativeSamples":0,"controllerSteps":0,"reward":0.0,"force_N":r["force_N"],
           "massS_kg":r["massS_kg"],"angle_rad":r["angle_rad"],"offset_m":r["offset_m"],
           "midTravel_m":r["midTravel_m"],"initialTravel_m":float(last[j].truth_last[2]),
           "initialOffset_m":float(last[j].truth_last[2]-r["midTravel_m"]),
           "initialAbsTravelError_m":float(abs(last[j].truth_last[2]-r["midTravel_m"])),
           "robot100HzFrames":0,"gateReady100HzFrames":0,"robotNonzeroIssued100HzFrames":0,
           "robotIssuedChanges100Hz":0,"closedGateNonzeroIssued100HzFrames":0,
           "peakIssuedRobotCorrection_m_s":0.0}
          for j,(i,r) in enumerate(zip(indices,chosen))]
    history=[[] for _ in chosen];started=time.monotonic();cfg=RewardConfig()
    encoder=FeatureEncoder(scenarios);identifiers=np.asarray(indices,np.int64)
    previous=np.array([[r["knownEquilibriumCurrent_A"],0] for r in chosen],np.float64)
    try:
        while not finished.all():
            active=np.flatnonzero(~finished)
            observed=np.stack([r.obs for r in last]);features=encoder.encode(observed,identifiers,steps)
            mask=action_mask(observed,steps)
            if baseline:
                means=np.zeros((len(chosen),2),np.float32)
                requests=np.array([[r["knownEquilibriumCurrent_A"],0] for r in chosen])
            else:
                means,_=cpu.infer(features,resets)
                normalized=np.tanh(means)*mask
                requests=encoder.actions(normalized,identifiers)
            resets[:]=0
            results=advance(plant,[contexts[j] for j in active],requests[active],repeat)
            for j,result in zip(active,results):
                if result.terminated and not result.physical_failure:
                    raise FloatingPointError("Native software/nonfinite termination during evaluation")
                row=rows[j];r=chosen[j]
                history[j].append([int(steps[j])*DT,*requests[j],*features[j],*means[j]])
                # Observe the actual postgate command on the native100Hz frame.
                # A held nonzero command is distinguished from a newly changed
                # command; closing the gate does not erase queued motion.
                if steps[j]%100==0:
                    row["robot100HzFrames"]+=1
                    row["gateReady100HzFrames"]+=int(observed[j,16]>.5)
                    row["robotNonzeroIssued100HzFrames"]+=int(abs(result.obs[15])>1e-12)
                    row["robotIssuedChanges100Hz"]+=int(abs(result.obs[15]-previous[j,1])>1e-12)
                    row["closedGateNonzeroIssued100HzFrames"]+=int(observed[j,16]<=.5 and abs(result.obs[15])>1e-12)
                row["peakIssuedRobotCorrection_m_s"]=max(row["peakIssuedRobotCorrection_m_s"],float(abs(result.obs[15])))
                steps[j]+=repeat;last[j]=result
                row["controllerSteps"]+=1;row["nativeSamples"]+=int(result.native_samples)
                row["normalSquaredErrorSum_N2"]+=result.mean_squared_normal_error_N2*result.native_samples
                row["peakNormalForceError_N"]=max(row["peakNormalForceError_N"],result.peak_normal_error_N)
                row["physicalFailure"]|=bool(result.physical_failure)
                issued=np.array([result.obs[13],result.obs[15]])
                row["reward"]+=reward_value(result,issued,previous[j],cfg);previous[j]=issued
                if result.physical_failure or steps[j]>=round(r["duration_s"]/DT):
                    finished[j]=True
                    row["completed"]=bool(not result.physical_failure and steps[j]>=round(r["duration_s"]/DT))
                    row["endTime_s"]=float(result.end_s)
                    row["rmsNormalForceError_N"]=(row["normalSquaredErrorSum_N2"]/max(1,row["nativeSamples"]))**.5
                    row["passed"]=row["completed"] and row["peakNormalForceError_N"]<=.5
                    row["designMarginPassed"]=row["passed"] and row["peakNormalForceError_N"]<=.45
                    row["lastTruth"]=np.asarray(result.truth_last).tolist()
                    row["finalTravel_m"]=float(result.truth_last[2])
                    row["finalAbsTravelError_m"]=float(abs(result.truth_last[2]-r["midTravel_m"]))
                    row["centeringReduction_m"]=row["initialAbsTravelError_m"]-row["finalAbsTravelError_m"]
                    if save_histories:
                        record=contexts[j].take_record()
                        error=record["truth"][:,0]-r["force_N"]
                        row["nativeSamples"]=len(error)
                        row["peakNormalForceError_N"]=float(np.abs(error).max())
                        row["rmsNormalForceError_N"]=float(np.sqrt(np.mean(error**2)))
                        row["nativeFirstTime_s"]=float(record["time"][0])
                        row["nativeLastTime_s"]=float(record["time"][-1])
                        row["nativeGridComplete"]=bool(len(error)==round(record["time"][-1]/plant.metadata["step_s"])+1)
                        assert row["nativeFirstTime_s"]==0 and row["nativeGridComplete"]
                        row["passed"]=row["completed"] and row["peakNormalForceError_N"]<=.5
                        row["designMarginPassed"]=row["passed"] and row["peakNormalForceError_N"]<=.45
                        target=directory/f"{name}_{r['identity']}_native.npz"
                        np.savez_compressed(target,**record)
                        row["nativeHistoryFile"]=str(target.resolve());row["nativeHistorySha256"]=sha(target)
                    actions=directory/f"{name}_{r['identity']}_actions.npz"
                    np.savez_compressed(actions,history=np.asarray(history[j],np.float64))
                    row["actionHistoryFile"]=str(actions.resolve());row["actionHistorySha256"]=sha(actions)
    finally:
        for context in contexts: context.close()
    report={"status":"complete","phase":name,"baseline":baseline,"rows":rows,
            "recordings":len(rows),"passed":sum(r["passed"] for r in rows),
            "physicalFailures":sum(r["physicalFailure"] for r in rows),
            "worstNormalForceError_N":max(r["peakNormalForceError_N"] for r in rows),
            "meanRmsNormalForceError_N":float(np.mean([r["rmsNormalForceError_N"] for r in rows])),
            "totalNativeSamples":sum(r["nativeSamples"] for r in rows),
            "elapsed_s":time.monotonic()-started,"featureNames":FEATURE_NAMES,
            "fullNativeHistories":save_histories,"noTestRetuning":True}
    if centering_baseline is not None:
        apply_centering_contract(report,centering_baseline,minimum_centering_fraction)
    write_json(directory/f"{name}.json",report)
    return report


def apply_centering_contract(report,baseline,minimum_fraction=.5):
    """A joint selection guard against force-only policies that leave the robot idle.

    This contract applies to the named nominal, zero-programmed-path cohort
    with at least0.5mm initial true stroke error. Stress cases with programmed
    path motion remain separate. The baseline is a matched executed recording,
    not an actor observation or training-action target.
    """
    if not 0<=minimum_fraction<=1:raise ValueError('centering fraction must be in [0,1]')
    if isinstance(baseline,(str,Path)):baseline=json.loads(Path(baseline).read_text())
    baseline_rows=baseline.get('cases',baseline.get('rows',[]))
    by_identity={r['identity']:r for r in baseline_rows}
    eligible=[]
    for row in report['rows']:
        nominal='_nominal_' in row['identity']
        applies=nominal and row['initialAbsTravelError_m']>=.0005-1e-12
        row['centeringConstraintApplies']=bool(applies)
        if not applies:
            row['centeringExclusion']='programmed-path stress cohort' if not nominal else 'initial offset below0.5mm'
            continue
        if row['identity'] not in by_identity:raise ValueError('centering baseline lacks matched nominal identity')
        reference=by_identity[row['identity']]
        initial=float(reference.get('initial_abs_recenter_m',reference.get('initialAbsTravelError_m')))
        final=float(reference.get('final_abs_recenter_m',reference.get('finalAbsTravelError_m')))
        if abs(initial-row['initialAbsTravelError_m'])>1e-10:
            raise ValueError('centering baseline has a different physical initial offset')
        baseline_reduction=initial-final
        if baseline_reduction<=0:raise ValueError('matched centering baseline made no progress')
        required=minimum_fraction*baseline_reduction
        row['baselineCenteringReduction_m']=baseline_reduction
        row['requiredCenteringReduction_m']=required
        row['fractionOfBaselineCenteringReduction']=row['centeringReduction_m']/baseline_reduction
        row['centeringPassed']=bool(row['completed'] and row['centeringReduction_m']>=required-1e-12)
        eligible.append(row)
    if not eligible:raise ValueError('centering contract found no eligible nominal offset recordings')
    report.update(centeringEligibleRecordings=len(eligible),
                  centeringViolations=sum(not r['centeringPassed'] for r in eligible),
                  minimumCenteringFraction=minimum_fraction,
                  minimumAchievedCenteringFraction=min(r['fractionOfBaselineCenteringReduction'] for r in eligible),
                  centeringContract='Named nominal zero-programmed-path recordings with initial absolute stroke error>=0.5mm must achieve the specified fraction of matched native-baseline initial-to-final error reduction.',
                  jointPassed=sum(r['passed'] and (not r['centeringConstraintApplies'] or r['centeringPassed']) for r in report['rows']))
    return report


class Collector:
    """C-policy sampling on native contexts with bounded evidence retention.

    Every failed episode retains its identity, seed, exact requested-action
    sequence and complete accumulated metrics. Ordinary contexts hold no native
    history; requested actions are sufficient to reconstruct their deterministic
    native trajectory. The first requested failures are additionally replayed
    with full native history as an immediate verification of that contract.
    """
    def __init__(self,plant,scenarios,policy,directory,environments,seed,reward_cfg,
                 decision_repeat=1,train_ids=None,save_every=50,failure_native_limit=1):
        self.plant,self.scenarios,self.policy=plant,scenarios,policy
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        self.rng=np.random.default_rng(seed);self.reward_cfg=reward_cfg
        self.decision_repeat=decision_repeat;self.encoder=FeatureEncoder(scenarios)
        self.horizon_steps=np.asarray([round(r['duration_s']/DT) for r in scenarios],np.int64)
        self.train_ids=np.asarray(train_ids if train_ids is not None else
            [i for i,r in enumerate(scenarios) if r['split']=='train'],np.int64)
        if not len(self.train_ids):raise ValueError('empty training split')
        self.identities=self.rng.choice(self.train_ids,environments)
        self.seeds=self.rng.integers(100000,2000000000,size=environments)
        self.contexts=[plant.create(scenario=int(i),seed=int(s),baseline=False,record=False)
                       for i,s in zip(self.identities,self.seeds)]
        self.last=[initial_result(c) for c in self.contexts]
        self.current_obs=np.stack([r.obs for r in self.last])
        self.steps=np.zeros(environments,np.int64);self.resets=np.ones(environments,np.uint8)
        self.cpu=CPolicy(policy,self.directory/'initial_policy',environments,{'phase':'training_initial'})
        self.previous=np.column_stack((self.encoder.equilibrium[self.identities],np.zeros(environments)))
        self.episode_count=0;self.transitions=0;self.failed_count=0
        self.episode_peak=np.array([x.peak_normal_error_N for x in self.last],np.float64)
        self.episode_returns=np.zeros(environments,np.float64)
        self.episode_squared_error=np.zeros(environments,np.float64)
        self.episode_native_samples=np.zeros(environments,np.int64)
        self.episode_peak_current=np.zeros(environments,np.float64)
        self.episode_peak_voltage=np.zeros(environments,np.float64)
        self.episode_voltage_clip=np.zeros(environments,np.int64)
        self.episode_current_limit=np.zeros(environments,np.int64)
        self.episode_actions=[[] for _ in range(environments)]
        self.failed_dir=self.directory/'failed_training_actions';self.failed_dir.mkdir()
        self.failure_native_limit=int(failure_native_limit);self.native_failures_saved=0
        self.save_every=int(save_every);self.last_rollout=None
        self.stop_requested=False

    def close(self):
        for context in self.contexts:context.close()

    def save_last_rollout(self,reason='final'):
        if self.last_rollout is None:return None
        update,arrays=self.last_rollout
        path=self.directory/f'rollout_{update:06d}.npz'
        if not path.exists():np.savez_compressed(path,**arrays)
        return {'update':update,'path':str(path.resolve()),'sha256':sha(path),'reason':reason}

    def state_dict(self):
        """Resume restores physical contexts by exact actions, then carried state."""
        names=('identities','seeds','steps','resets','previous','episode_peak','episode_returns',
               'episode_squared_error','episode_native_samples','episode_peak_current',
               'episode_peak_voltage','episode_voltage_clip','episode_current_limit')
        return {**{name:getattr(self,name).copy() for name in names},
            'rng':self.rng.bit_generator.state,'cpu_state':self.cpu.state.copy(),
            'actions':[np.asarray(x,np.float64).reshape(-1,2) for x in self.episode_actions],
            'episode_count':self.episode_count,'transitions':self.transitions,
            'failed_count':self.failed_count,'native_failures_saved':self.native_failures_saved,
            'train_ids':self.train_ids.copy(),'decision_repeat':self.decision_repeat,
            'last_observations':self.current_obs.copy(),
            'resume_contract':'Native reset and replay exact requests; restore recurrent/RNG state; no hidden-state reset at rollout boundaries.'}

    def load_state_dict(self,state):
        if state['decision_repeat']!=self.decision_repeat:raise ValueError('resume decision clock changed')
        if len(state['identities'])!=len(self.contexts):raise ValueError('resume environment count changed')
        if not np.array_equal(state['train_ids'],self.train_ids):raise ValueError('resume train IDs changed')
        for j,context in enumerate(self.contexts):
            context.reset(scenario=int(state['identities'][j]),seed=int(state['seeds'][j]),baseline=False)
        actions=state['actions']
        for t in range(max(map(len,actions),default=0)):
            active=[j for j,a in enumerate(actions) if t<len(a)]
            results=advance(self.plant,[self.contexts[j] for j in active],np.asarray([actions[j][t] for j in active]),self.decision_repeat)
            if any(r.terminated for r in results):raise AssertionError('active resume episode unexpectedly terminated')
        self.last=[initial_result(c) for c in self.contexts]
        self.current_obs=np.stack([r.obs for r in self.last])
        if not np.array_equal(np.stack([r.obs for r in self.last]),state['last_observations']):
            raise AssertionError('resume native observation replay mismatch')
        for name in ('identities','seeds','steps','resets','previous','episode_peak','episode_returns',
                     'episode_squared_error','episode_native_samples','episode_peak_current',
                     'episode_peak_voltage','episode_voltage_clip','episode_current_limit'):
            setattr(self,name,np.asarray(state[name]).copy())
        self.rng.bit_generator.state=state['rng'];self.cpu.state[:]=state['cpu_state']
        self.episode_actions=[[row.copy() for row in a] for a in actions]
        for name in ('episode_count','transitions','failed_count','native_failures_saved'):
            setattr(self,name,int(state[name]))

    def _complete(self,j,result,terminated,truncated,update):
        scenario=int(self.identities[j]);chosen=self.scenarios[scenario]
        row={'episode':self.episode_count,'update':update,'identity':chosen['identity'],
             'scenario':scenario,'seed':int(self.seeds[j]),'steps':int(self.steps[j]),
             'decisions':len(self.episode_actions[j]),'decisionRepeat':self.decision_repeat,
             'terminated':bool(terminated),'truncated':bool(truncated),
             'contactLoss':bool(result.contact_loss),'travelStop':bool(result.travel_stop),
             'endTime_s':float(result.end_s),'nativeSamples':int(self.episode_native_samples[j]),
             'peakNormalForceError_N':float(self.episode_peak[j]),
             'rmsNormalForceError_N':float(np.sqrt(self.episode_squared_error[j]/max(1,self.episode_native_samples[j]))),
             'peakCurrent_A':float(self.episode_peak_current[j]),
             'peakVoltage_V':float(self.episode_peak_voltage[j]),
             'voltageClipSamples':int(self.episode_voltage_clip[j]),
             'currentLimitSamples':int(self.episode_current_limit[j]),
             'return':float(self.episode_returns[j]),
             'passed':bool(truncated and self.episode_peak[j]<=.5)}
        if terminated:
            actions=np.asarray(self.episode_actions[j],np.float64)
            path=self.failed_dir/f'failed_{self.episode_count:08d}.npz'
            np.savez_compressed(path,requested_actions=actions,scenario=np.int64(scenario),
                                seed=np.int64(self.seeds[j]),decision_repeat=np.int64(self.decision_repeat),
                                final_observation=result.obs,final_truth=result.truth_last)
            row['failureActionsFile']=str(path.resolve());row['failureActionsSha256']=sha(path)
            if self.native_failures_saved<self.failure_native_limit:
                replay=self.plant.create(scenario=scenario,seed=int(self.seeds[j]),baseline=False,record=True)
                try:
                    replay_result=None
                    for action in actions:
                        replay_result=advance(self.plant,[replay],action[None,:],self.decision_repeat)[0]
                    assert replay_result is not None and replay_result.physical_failure
                    assert np.array_equal(replay_result.obs,result.obs)
                    assert np.array_equal(replay_result.truth_last,result.truth_last)
                    native=self.failed_dir/f'failed_{self.episode_count:08d}_native.npz'
                    np.savez_compressed(native,**replay.take_record())
                    row['nativeFailureFile']=str(native.resolve());row['nativeFailureSha256']=sha(native)
                    row['exactActionReplayPassed']=True;self.native_failures_saved+=1
                finally:replay.close()
            self.failed_count+=1
        with (self.directory/'training_episodes.jsonl').open('a') as stream:
            stream.write(json.dumps(row,allow_nan=False)+'\n')
        self.episode_count+=1
        i=int(self.rng.choice(self.train_ids));s=int(self.rng.integers(100000,2000000000))
        self.identities[j]=i;self.seeds[j]=s
        self.contexts[j].reset(scenario=i,seed=s,baseline=False)
        self.last[j]=initial_result(self.contexts[j]);self.current_obs[j]=self.last[j].obs
        self.steps[j]=0;self.resets[j]=1
        self.previous[j]=[self.encoder.equilibrium[i],0]
        self.episode_peak[j]=self.last[j].peak_normal_error_N
        for name in ('episode_returns','episode_squared_error','episode_native_samples','episode_peak_current',
                     'episode_peak_voltage','episode_voltage_clip','episode_current_limit'):
            getattr(self,name)[j]=0
        self.episode_actions[j]=[]
        return row

    def collect(self,length,update,deadline=None):
        count=len(self.contexts);D=len(FEATURE_NAMES)
        initial_state=self.cpu.state.transpose(1,0,2).copy()
        observations=np.empty((length,count,D),np.float32)
        latents=np.empty((length,count,2),np.float32);oldmeans=np.empty_like(latents)
        log_probs_by_action=np.empty_like(latents)
        state_inputs=np.empty((length,self.policy.num_layers,count,self.policy.hidden_size),np.float32)
        log_probs=np.empty((length,count),np.float32);values=np.empty_like(log_probs)
        rewards=np.empty_like(log_probs);next_values=np.zeros_like(log_probs)
        terminated=np.zeros((length,count),bool);truncated=np.zeros_like(terminated)
        reset_before=np.zeros((length,count),bool);masks=np.empty_like(latents)
        physical_interval=np.empty((length,count,8),np.float64)
        requests_saved=np.empty((length,count,2),np.float64)
        issued_correction_frames=0;issued_correction_peak=0.0;closed_gate_nonzero_frames=0
        started=time.monotonic();episodes=[];executed=0;bootstrap_calls=0
        for t in range(length):
            raw=self.current_obs
            issued_correction_frames+=int((np.abs(raw[:,15])>1e-12).sum())
            issued_correction_peak=max(issued_correction_peak,float(np.abs(raw[:,15]).max()))
            closed_gate_nonzero_frames+=int(((raw[:,16]<.5)&(np.abs(raw[:,15])>1e-12)).sum())
            features=self.encoder.encode(raw,self.identities,self.steps);mask=action_mask(raw,self.steps)
            reset_before[t]=self.resets;observations[t]=features;masks[t]=mask
            state_inputs[t]=self.cpu.state.transpose(1,0,2)
            output=self.cpu.sample(features,self.rng,self.resets,mask)
            self.resets[:]=0
            latents[t]=output['latent_actions'];oldmeans[t]=output['means']
            log_probs[t]=output['log_prob'];values[t]=output['values']
            log_probs_by_action[t]=output['log_prob_by_action']
            # Ordinary next state is exactly the next sampled behavior value.
            # Episode-boundary final values were already captured before reset.
            if t:
                continuing=~(terminated[t-1]|truncated[t-1])
                next_values[t-1,continuing]=values[t,continuing]
            requests=self.encoder.actions(output['actions'],self.identities);requests_saved[t]=requests
            next_raw,next_truth,next_comparator,native=advance_arrays(self.plant,self.contexts,requests,self.decision_repeat)
            self.steps+=self.decision_repeat
            issued=next_raw[:,[13,15]]
            for j,request in enumerate(requests):self.episode_actions[j].append(request.copy())
            failed=(native[:,17]>0)|(native[:,18]>0)
            rejected=np.flatnonzero((native[:,20]>0)&~failed)
            if len(rejected):
                for j in rejected:
                    path=self.failed_dir/f'software_rejected_update_{update}_env_{j}.npz'
                    np.savez_compressed(path,requested_actions=np.asarray(self.episode_actions[j]),
                                        scenario=self.identities[j],seed=self.seeds[j],final_obs=next_raw[j])
                raise FloatingPointError('Native software/nonfinite termination; exact requests retained')
            rewards[t]=array_rewards(native,issued,self.previous,self.reward_cfg)
            physical_interval[t]=np.column_stack((native[:,3],native[:,6],native[:,7],native[:,10],
                                                   native[:,13],native[:,2],failed,self.identities))
            terminated[t]=failed
            truncated[t]=~failed&(self.steps>=self.horizon_steps[self.identities])
            self.episode_peak=np.maximum(self.episode_peak,native[:,6])
            self.episode_returns+=rewards[t].astype(np.float64)
            self.episode_squared_error+=native[:,7]*native[:,3]
            self.episode_native_samples+=native[:,3].astype(np.int64)
            self.episode_peak_current=np.maximum(self.episode_peak_current,native[:,11])
            self.episode_peak_voltage=np.maximum(self.episode_peak_voltage,native[:,12])
            self.episode_voltage_clip+=native[:,14].astype(np.int64)
            self.episode_current_limit+=native[:,15].astype(np.int64)
            # Only horizon truncations need an immediate final-state bootstrap.
            # Failures are zero. This cloned inference cannot alter carried state.
            ending=np.flatnonzero(truncated[t])
            if len(ending):
                next_features=self.encoder.encode(next_raw[ending],self.identities[ending],self.steps[ending])
                _,next_values[t,ending]=self.cpu.infer(next_features,state=self.cpu.state[ending].copy())
                bootstrap_calls+=1
            self.current_obs=next_raw;self.previous=issued
            for j in np.flatnonzero(terminated[t]|truncated[t]):
                from native_api import _result
                result=_result(next_raw[j],next_truth[j],next_comparator[j],native[j])
                episodes.append(self._complete(j,result,terminated[t,j],truncated[t,j],update))
            executed=t+1
            if self.stop_requested or (deadline is not None and time.time()>=deadline):break
        continuing=np.flatnonzero(~(terminated[executed-1]|truncated[executed-1]))
        if len(continuing):
            next_feature=self.encoder.encode(self.current_obs[continuing],self.identities[continuing],self.steps[continuing])
            _,next_values[executed-1,continuing]=self.cpu.infer(next_feature,state=self.cpu.state[continuing].copy())
            bootstrap_calls+=1
        arrays=dict(observations=observations[:executed],latent_actions=latents[:executed],
                    old_means=oldmeans[:executed],log_probs=log_probs[:executed],values=values[:executed],
                    log_probs_by_action=log_probs_by_action[:executed],state_inputs=state_inputs[:executed],
                    rewards=rewards[:executed],terminated=terminated[:executed],truncated=truncated[:executed],
                    next_values=next_values[:executed],reset_before=reset_before[:executed],
                    action_mask=masks[:executed],initial_state=initial_state,
                    requested_actions=requests_saved[:executed],physical_interval=physical_interval[:executed])
        self.last_rollout=(update,arrays);saved=None
        if update==0 or (self.save_every>0 and (update+1)%self.save_every==0):
            saved=self.save_last_rollout('periodic')
        self.transitions+=executed*count
        excluded={'old_means','requested_actions','physical_interval'}
        device=next(self.policy.parameters()).device
        batch=RolloutBatch(**{k:torch.as_tensor(v,device=device) for k,v in arrays.items() if k not in excluded})
        intervals=physical_interval[:executed];weights=intervals[:,:,0]
        metrics={'collectionElapsed_s':time.monotonic()-started,'transitions':executed*count,
                 'totalTransitions':self.transitions,'episodesCompleted':len(episodes),
                 'episodePasses':sum(r['passed'] for r in episodes),
                 'physicalFailures':int(terminated[:executed].sum()),'meanReward':float(rewards[:executed].mean()),
                 'nativeTicks':int(weights.sum()),'rolloutPeakNormalForceError_N':float(intervals[:,:,1].max()),
                 'nonzeroIssuedRobotCorrectionFrames':issued_correction_frames,
                 'peakIssuedRobotCorrection_m_s':issued_correction_peak,
                 'heldCorrectionWhileGateClosedFrames':closed_gate_nonzero_frames,
                 'meanSquaredNormalError_N2':float(np.sum(intervals[:,:,2]*weights)/max(1,weights.sum())),
                 'bootstrapInferenceCalls':bootstrap_calls,'ordinaryNextValuesReused':True,'vectorizedNativeScoring':True,
                 'completedRolloutSteps':executed,'requestedRolloutSteps':length}
        if saved:metrics.update(rolloutFile=saved['path'],rolloutSha256=saved['sha256'])
        if update==0:
            with torch.no_grad():
                replay=self.policy.sequence(batch.observations,batch.initial_state,batch.reset_before)
                log,_,_=self.policy.evaluate_latent(replay.mean,batch.latent_actions,batch.action_mask)
                metrics['behaviorReplayMeanMaxResidual']=float((replay.mean-torch.as_tensor(oldmeans[:executed],device=device)).abs().max())
                metrics['behaviorReplayValueMaxResidual']=float((replay.value-batch.values).abs().max())
                metrics['behaviorReplayLogProbMaxResidual']=float((log-batch.log_probs).abs().max())
                assert metrics['behaviorReplayMeanMaxResidual']<1e-4
                assert metrics['behaviorReplayLogProbMaxResidual']<.005
        return batch,metrics


def parse_ids(spec,scenarios,split):
    if spec is None:return [i for i,r in enumerate(scenarios) if r['split']==split]
    path=Path(spec)
    raw=json.loads(path.read_text()) if path.is_file() else spec.split(',')
    if isinstance(raw,dict):raw=raw.get('ids',raw.get('scenarios'))
    by_name={r['identity']:i for i,r in enumerate(scenarios)}
    ids=[by_name[x] if isinstance(x,str) and x in by_name else int(x) for x in raw]
    if not ids or len(ids)!=len(set(ids)):raise ValueError('split IDs must be nonempty and unique')
    if any(i<0 or i>=len(scenarios) for i in ids):raise ValueError('unknown split ID')
    if any(scenarios[i]['split']!=split for i in ids):
        raise ValueError(f'explicit IDs must belong to {split}; held-out data cannot train/select')
    return ids


def atomic_checkpoint(path,payload):
    target=Path(path);temporary=target.with_suffix('.tmp')
    torch.save(payload,temporary);os.replace(temporary,target)


def main():
    global _PREPARING_OUTPUT
    parser=argparse.ArgumentParser()
    parser.add_argument('--native-build',required=True);parser.add_argument('--bank',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--seed',type=int,default=8101)
    parser.add_argument('--updates',type=int,default=100000);parser.add_argument('--environments',type=int,default=16)
    parser.add_argument('--rollout',type=int,default=1024);parser.add_argument('--eval-every',type=int,default=25)
    parser.add_argument('--decision-repeat',type=int,default=10)
    parser.add_argument('--current-log-std',type=float,default=-4.5);parser.add_argument('--robot-log-std',type=float,default=-1.5)
    parser.add_argument('--hidden-size',type=int,default=32);parser.add_argument('--layers',type=int,default=2)
    parser.add_argument('--min-memory-s',type=float,default=.0003);parser.add_argument('--max-memory-s',type=float,default=.3)
    parser.add_argument('--physics-init-force-gain',type=float,default=.1)
    parser.add_argument('--detach-critic-trunk',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--learning-rate',type=float,default=1e-5)
    parser.add_argument('--actor-learning-rate',type=float);parser.add_argument('--critic-learning-rate',type=float)
    parser.add_argument('--log-std-learning-rate',type=float)
    parser.add_argument('--value-coef',type=float,default=.1);parser.add_argument('--value-clip',type=float,default=0.0)
    parser.add_argument('--value-huber-delta',type=float,default=1.0)
    parser.add_argument('--robot-policy-weight',type=float,default=1.0)
    parser.add_argument('--channelwise-policy-loss',action=argparse.BooleanOptionalAction,default=True)
    parser.add_argument('--entropy-coef',type=float,default=1e-4);parser.add_argument('--clip-coef',type=float,default=.1)
    parser.add_argument('--target-kl',type=float,default=.02);parser.add_argument('--max-grad-norm',type=float,default=.5)
    parser.add_argument('--epochs',type=int,default=2);parser.add_argument('--minibatch-sequences',type=int,default=8)
    parser.add_argument('--gamma-time-s',type=float,default=1.0);parser.add_argument('--lambda-time-s',type=float,default=.3)
    parser.add_argument('--chunk-length',type=int,default=1024);parser.add_argument('--burnin-steps',type=int,default=64)
    parser.add_argument('--train-ids');parser.add_argument('--selection-ids')
    parser.add_argument('--centering-baseline');parser.add_argument('--minimum-centering-fraction',type=float,default=.5)
    parser.add_argument('--device',default='cuda');parser.add_argument('--torch-threads',type=int,default=4)
    parser.add_argument('--native-threads',type=int,default=1)
    parser.add_argument('--no-selection',action='store_true');parser.add_argument('--selection-initial',action='store_true')
    parser.add_argument('--save-rollout-every',type=int,default=50);parser.add_argument('--checkpoint-every',type=int,default=10)
    parser.add_argument('--failure-native-limit',type=int,default=1)
    parser.add_argument('--max-seconds',type=float);parser.add_argument('--deadline-utc')
    parser.add_argument('--deadline-reserve-s',type=float,default=30)
    parser.add_argument('--resume');parser.add_argument('--resume-policy')
    parser.add_argument('--resume-reset-optimizer',action='store_true')
    parser.add_argument('--reward-scale',type=float,default=10.0)
    for item in fields(RewardConfig):
        if item.name!='return_scale':
            parser.add_argument('--'+item.name.replace('_','-'),type=float,default=item.default)
    args=parser.parse_args();directory=Path(args.output);directory.mkdir(parents=True,exist_ok=False)
    _PREPARING_OUTPUT=directory/'results.json'
    write_json(directory/'results.json',{'status':'preparing','args':vars(args)})
    if args.decision_repeat<1 or 100%args.decision_repeat:raise ValueError('decision repeat must divide native 100 Hz gate period')
    if args.resume and args.resume_policy:raise ValueError('choose exact checkpoint resume or policy-only initialization')
    if min(args.environments,args.rollout,args.eval_every,args.checkpoint_every)<1:raise ValueError('positive runner counts required')
    torch.set_num_threads(args.torch_threads);torch.manual_seed(args.seed);np.random.seed(args.seed)
    if args.device.startswith('cuda'):torch.cuda.manual_seed_all(args.seed)
    torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
    start=time.monotonic();started_wall=time.time();deadline=None
    if args.max_seconds:deadline=started_wall+args.max_seconds
    if args.deadline_utc:
        specified=datetime.fromisoformat(args.deadline_utc.replace('Z','+00:00'))
        if specified.tzinfo is None:specified=specified.replace(tzinfo=timezone.utc)
        deadline=specified.timestamp() if deadline is None else min(deadline,specified.timestamp())
    collection_deadline=None if deadline is None else deadline-args.deadline_reserve_s
    from native_api import NativePlant
    source_plant=NativePlant(args.native_build);scenarios=get_scenarios(source_plant,args.bank)
    train_ids=parse_ids(args.train_ids,scenarios,'train');selection_ids=parse_ids(args.selection_ids,scenarios,'selection')
    native_inputs=directory/'execution_inputs';native_inputs.mkdir()
    for name in ('libacf_native.so','metadata.json','bridge_build.json','acf_rl_native.slx',
                 'native_parameter_bank.c','parameter_mapping.json','acf_rl_native_context.c'):
        source=Path(args.native_build)/name
        if source.is_file():shutil.copy2(source,native_inputs/name)
    write_json(native_inputs/'scenarios.json',scenarios)
    shutil.copytree(Path(__file__).parent,directory/'frozen_sources',ignore=shutil.ignore_patterns('__pycache__'))
    plant=NativePlant(native_inputs)
    if hasattr(plant,'set_threads'):plant.set_threads(args.native_threads)
    elif args.native_threads!=1:raise ValueError('native library does not support requested threads')
    write_json(directory/'consumed_scenarios.json',scenarios)
    centering_baseline=None
    if args.centering_baseline:
        centering_baseline=json.loads(Path(args.centering_baseline).read_text())
        shutil.copy2(args.centering_baseline,directory/'centering_baseline.json')
    resumed=None;policy_seed=None
    if args.resume:resumed=torch.load(args.resume,map_location=args.device,weights_only=False)
    if args.resume_policy:
        checkpoint_path=Path(args.resume_policy)
        if checkpoint_path.is_dir():checkpoint_path=checkpoint_path/'policy_checkpoint.pt'
        policy_seed=torch.load(checkpoint_path,map_location=args.device,weights_only=False)
    policy_config={'obs_dim':len(FEATURE_NAMES),'hidden_size':args.hidden_size,'num_layers':args.layers,
                   'dt':DT*args.decision_repeat,'min_memory_s':args.min_memory_s,'max_memory_s':args.max_memory_s,
                   'initial_log_std':(args.current_log_std,args.robot_log_std)}
    policy_parameters=inspect.signature(UnifiedPolicy).parameters
    for name in ('physics_init_force_gain','detach_critic_trunk'):
        if name in policy_parameters:policy_config[name]=getattr(args,name)
    # Architecture and feature transforms belong to the checkpoint. A requested
    # controller-clock change is not an exact resume and must be deliberate in
    # an independently initialized experiment.
    if resumed is not None:policy_config=resumed['policy_config']
    elif policy_seed is not None and 'config' in policy_seed:policy_config=policy_seed['config']
    if policy_config['obs_dim']!=len(FEATURE_NAMES) or abs(policy_config['dt']-DT*args.decision_repeat)>1e-12:
        raise ValueError('checkpoint observation contract or decision clock differs from requested run')
    policy=UnifiedPolicy(**policy_config).to(args.device)
    cfg_kwargs={name:getattr(args,name) for name in ('learning_rate','value_coef','value_clip','entropy_coef',
                'clip_coef','target_kl','max_grad_norm','epochs','minibatch_sequences','gamma_time_s','lambda_time_s',
                'chunk_length','burnin_steps')}
    supported={f.name for f in fields(PPOConfig)}
    for name in ('actor_learning_rate','critic_learning_rate','log_std_learning_rate',
                 'value_huber_delta','robot_policy_weight','channelwise_policy_loss'):
        value=getattr(args,name)
        if value is not None:
            if name not in supported:raise ValueError(f'PPO implementation does not support {name}')
            cfg_kwargs[name]=value
    cfg=PPOConfig(dt=policy.dt,**cfg_kwargs);trainer=PPOTrainer(policy,cfg)
    reward_cfg=RewardConfig(**{item.name:(args.reward_scale if item.name=='return_scale' else getattr(args,item.name))
                              for item in fields(RewardConfig)})
    if args.resume:
        if resumed.get('bank_sha256')!=sha(args.bank):raise ValueError('resume bank differs')
        if resumed.get('reward_config')!=asdict(reward_cfg):raise ValueError('exact resume reward configuration differs')
        if args.resume_reset_optimizer:policy.load_state_dict(resumed['policy'])
        else:trainer.load_state_dict(resumed)
    if args.resume_policy:
        policy.load_state_dict(policy_seed.get('policy',policy_seed.get('state_dict',policy_seed)))
    report={'status':'training','date':'2026-10-05','seed':args.seed,'args':vars(args),
            'architecture':policy.config(),'parameterCount':sum(p.numel() for p in policy.parameters()),
            'featureNames':FEATURE_NAMES,'reward':{**reward_cfg.contract(),'policyDecisionInterval_s':policy.dt},
            'ppo':asdict(cfg),'nativeBuild':str(Path(args.native_build).resolve()),'bankSha256':sha(args.bank),
            'frozenNativeFileHashes':{p.name:sha(p) for p in native_inputs.iterdir() if p.is_file()},
            'inputSourceHashes':{p.name:sha(p) for p in Path(__file__).parent.glob('*.py')},
            'teacherActionsUsed':False,'dynamicPIUnderCurrentActor':False,'heldoutOutcomesUsedDuringTraining':False,
            'historyCarriedBetweenRollouts':True,'trainingIds':train_ids,'selectionIds':selection_ids,
            'deadlineUTC':None if deadline is None else datetime.fromtimestamp(deadline,timezone.utc).isoformat(),
            'retention':{'fullOrdinaryNativeHistories':False,'everyPhysicalFailureExactActions':True,
                         'everyCompletedEpisodeMetrics':True,'detailedRolloutEvery':args.save_rollout_every,
                         'firstAndLastDetailedRollout':True,'representativeNativeFailures':args.failure_native_limit},
            'updates':[],'selection':[]}
    if centering_baseline is not None:
        report['centeringBaselineSha256']=sha(args.centering_baseline)
        report['selectionRankFields']=['physicalFailures','centeringViolations','negativeForcePasses','worstNormalForceError_N','meanRmsNormalForceError_N']
    write_json(directory/'results.json',report)
    collector=Collector(plant,scenarios,policy,directory/'rollouts',args.environments,args.seed,reward_cfg,
                        args.decision_repeat,train_ids,args.save_rollout_every,args.failure_native_limit)
    if resumed is not None:
        collector.load_state_dict(resumed['collector']);report['resumeNativeReplayPassed']=True
        if 'torch_rng' in resumed:torch.set_rng_state(resumed['torch_rng'].cpu())
        if args.device.startswith('cuda') and resumed.get('cuda_rng'):
            torch.cuda.set_rng_state_all([state.cpu() for state in resumed['cuda_rng']])
    previous_handlers={}
    for signum in (signal.SIGTERM,signal.SIGINT):
        previous_handlers[signum]=signal.signal(signum,lambda *_:setattr(collector,'stop_requested',True))
    best_rank=None;completed_updates=0
    def checkpoint():
        atomic_checkpoint(directory/'latest_training.pt',{**trainer.state_dict(),'collector':collector.state_dict(),
            'torch_rng':torch.get_rng_state(),'cuda_rng':torch.cuda.get_rng_state_all() if args.device.startswith('cuda') else None,
            'bank_sha256':sha(args.bank),'reward_config':asdict(reward_cfg),'source_run':str(directory.resolve())})
    def selection(update):
        nonlocal best_rank
        selected=evaluate(plant,scenarios,selection_ids,policy,directory/'selection',f'selection_update_{update:06d}',
                          centering_baseline=centering_baseline,minimum_centering_fraction=args.minimum_centering_fraction)
        rank=(selected['physicalFailures'],)
        if centering_baseline is not None:rank+=(selected['centeringViolations'],)
        rank+=(-selected['passed'],selected['worstNormalForceError_N'],selected['meanRmsNormalForceError_N'])
        candidate=directory/'selection_policies'/f'update_{update:06d}'
        export_policy(policy,candidate,FEATURE_NAMES,{'seed':args.seed,'selectionUpdate':update,'selectionRank':rank,
                      'trainedTransitions':collector.transitions,'teacherUsed':False})
        report['selection'].append({'update':update,'rank':rank,'worstNormalForceError_N':selected['worstNormalForceError_N'],
                                   'passed':selected['passed'],'physicalFailures':selected['physicalFailures'],
                                   'centeringViolations':selected.get('centeringViolations'),
                                   'centeringEligibleRecordings':selected.get('centeringEligibleRecordings'),
                                   'minimumAchievedCenteringFraction':selected.get('minimumAchievedCenteringFraction'),
                                   'checkpointFile':str((candidate/'policy_checkpoint.pt').resolve()),
                                   'checkpointSha256':sha(candidate/'policy_checkpoint.pt')})
        if best_rank is None or rank<best_rank:
            best_rank=rank
            export_policy(policy,directory/'selected_policy',FEATURE_NAMES,{'seed':args.seed,'selectionUpdate':update,
                          'selectionRank':rank,'trainedTransitions':collector.transitions,'teacherUsed':False})
            report['selectedUpdate']=update;report['selectedRank']=rank
        print(f"RL_SELECTION seed={args.seed} update={update} pass={selected['passed']}/{len(selection_ids)} "
              f"peak={selected['worstNormalForceError_N']:.6g}N fails={selected['physicalFailures']} "
              f"centeringViolations={selected.get('centeringViolations','unscored')}",flush=True)
    try:
        _PREPARING_OUTPUT=None
        if args.selection_initial and not args.no_selection:selection(0)
        first_update=int(trainer.update_count)
        for update in range(first_update,first_update+args.updates):
            if collector.stop_requested or (collection_deadline is not None and time.time()>=collection_deadline):break
            batch,metrics=collector.collect(args.rollout,update,collection_deadline)
            # A gracefully shortened rollout remains valid. Reserve enough time
            # for its optimizer step and final atomic checkpoint.
            timer=time.monotonic();optimization=trainer.update(batch)
            metrics.update(optimization);metrics['optimizationElapsed_s']=time.monotonic()-timer
            metrics['updateIndex']=update;metrics['elapsed_s']=time.monotonic()-start
            report['updates'].append(metrics);collector.cpu.refresh(policy);completed_updates+=1
            print(f"RL_UPDATE seed={args.seed} update={update+1} transitions={collector.transitions} "
                  f"errorRms={metrics['meanSquaredNormalError_N2']**.5:.6g}N peak={metrics['rolloutPeakNormalForceError_N']:.6g}N "
                  f"fails={metrics['physicalFailures']} collect={metrics['collectionElapsed_s']:.2f}s "
                  f"learn={metrics['optimizationElapsed_s']:.2f}s kl={metrics.get('approx_kl',0):.4g}",flush=True)
            remaining=float('inf') if collection_deadline is None else collection_deadline-time.time()
            if not args.no_selection and ((update+1)%args.eval_every==0 or completed_updates==args.updates) and remaining>15:
                selection(update+1)
            if completed_updates%args.checkpoint_every==0:checkpoint()
            write_json(directory/'results.json',report);del batch
        checkpoint()
        report['lastDetailedRollout']=collector.save_last_rollout()
        export_policy(policy,directory/'final_policy',FEATURE_NAMES,{'seed':args.seed,'trainedTransitions':collector.transitions})
        report['status']='training_complete';report['trainedTransitions']=collector.transitions
        report['episodesCompleted']=collector.episode_count;report['physicalFailureEpisodes']=collector.failed_count
        report['elapsed_s']=time.monotonic()-start;report['completedUpdatesThisRun']=completed_updates
        report['stopReason']='signal' if collector.stop_requested else ('deadline' if completed_updates<args.updates else 'updates')
    except BaseException:
        report['status']='software_rejection';report['error']=traceback.format_exc()
        try:checkpoint();report['lastDetailedRollout']=collector.save_last_rollout('software_rejection')
        except BaseException:report['checkpointError']=traceback.format_exc()
        write_json(directory/'results.json',report);raise
    finally:
        collector.close();write_json(directory/'results.json',report)
        for signum,handler in previous_handlers.items():signal.signal(signum,handler)


if __name__=='__main__':
    try:main()
    except BaseException:
        # Preparation failures are evidence too. Do not replace an existing
        # run's completed/rejected status when output-directory reuse is refused.
        if _PREPARING_OUTPUT is not None:
            destination=_PREPARING_OUTPUT
            if destination.is_file():
                preparation=json.loads(destination.read_text())
                if preparation.get('status') in ('preparing','training') and not preparation.get('updates'):
                    preparation['status']='preparation_rejection';preparation['error']=traceback.format_exc()
                    write_json(destination,preparation)
        raise
