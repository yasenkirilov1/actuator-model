"""Reward-train one recurrent actor on the generated native Simulink plant.

Whole-episode selection is separate from held-out qualification. Baseline
actions never enter actor/critic features or optimization targets. The inner
current servo, acquisition, observers, hard robot gate and queue stay native.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time
import traceback

import numpy as np
import torch

from env import (CPolicy, DT, FEATURE_NAMES, RewardConfig, action_mask,
                 feature_array, physical_actions)
from policy import UnifiedPolicy
from policy_export import export_policy
from ppo import PPOConfig, PPOTrainer, RolloutBatch


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
    return float(reward)


def evaluate(plant, scenarios, indices, policy, directory, name, seed_offset=0,
             baseline=False, save_histories=False):
    """A whole native episode set; no gradients or test-driven retuning."""
    directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
    chosen=[scenarios[i] for i in indices]
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
           "massS_kg":r["massS_kg"],"angle_rad":r["angle_rad"],"offset_m":r["offset_m"]}
          for j,(i,r) in enumerate(zip(indices,chosen))]
    history=[[] for _ in chosen];started=time.monotonic();cfg=RewardConfig()
    previous=np.array([[r["knownEquilibriumCurrent_A"],0] for r in chosen],np.float64)
    try:
        while not finished.all():
            active=np.flatnonzero(~finished)
            observed=np.stack([r.obs for r in last]);features=feature_array(observed,chosen,steps)
            mask=action_mask(observed,steps)
            if baseline:
                means=np.zeros((len(chosen),2),np.float32)
                requests=np.array([[r["knownEquilibriumCurrent_A"],0] for r in chosen])
            else:
                means,_=cpu.infer(features,resets)
                normalized=np.tanh(means)*mask
                requests=physical_actions(normalized,chosen)
            resets[:]=0
            results=plant.step_batch([contexts[j] for j in active],requests[active])
            for j,result in zip(active,results):
                if result.terminated and not result.physical_failure:
                    raise FloatingPointError("Native software/nonfinite termination during evaluation")
                row=rows[j];r=chosen[j]
                history[j].append([int(steps[j])*DT,*requests[j],*features[j],*means[j]])
                steps[j]+=1;last[j]=result
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
    write_json(directory/f"{name}.json",report)
    return report


class Collector:
    def __init__(self,plant,scenarios,policy,directory,environments,seed,reward_cfg):
        self.plant,self.scenarios,self.policy=plant,scenarios,policy
        self.directory=Path(directory);self.directory.mkdir(parents=True,exist_ok=True)
        self.rng=np.random.default_rng(seed);self.reward_cfg=reward_cfg
        self.train_ids=np.array([i for i,r in enumerate(scenarios) if r["split"]=="train"])
        self.identities=self.rng.choice(self.train_ids,environments)
        self.seeds=self.rng.integers(100000,2000000000,size=environments)
        self.contexts=[plant.create(scenario=int(i),seed=int(s),baseline=False,record=True)
                       for i,s in zip(self.identities,self.seeds)]
        self.last=[initial_result(c) for c in self.contexts]
        self.steps=np.zeros(environments,np.int64);self.resets=np.ones(environments,np.uint8)
        self.cpu=CPolicy(policy,self.directory/"initial_policy",environments,{"phase":"training_initial"})
        self.previous=np.array([[scenarios[i]["knownEquilibriumCurrent_A"],0] for i in self.identities])
        self.episode_count=0
        self.episode_peak=np.array([x.peak_normal_error_N for x in self.last],np.float64)
        self.episode_returns=np.zeros(environments,np.float64)
        self.transitions=0
        self.failed_dir=self.directory/"failed_training_native";self.failed_dir.mkdir()

    def close(self):
        for context in self.contexts: context.close()

    def collect(self,length,update):
        count=len(self.contexts);D=len(FEATURE_NAMES)
        initial_state=self.cpu.state.transpose(1,0,2).copy()
        observations=np.empty((length,count,D),np.float32)
        latents=np.empty((length,count,2),np.float32);oldmeans=np.empty_like(latents)
        log_probs=np.empty((length,count),np.float32);values=np.empty_like(log_probs)
        rewards=np.empty_like(log_probs);next_values=np.empty_like(log_probs)
        terminated=np.zeros((length,count),bool);truncated=np.zeros_like(terminated)
        reset_before=np.zeros((length,count),bool);masks=np.empty_like(latents)
        physical_interval=np.empty((length,count,8),np.float64)
        requests_saved=np.empty((length,count,2),np.float64)
        issued_correction_frames=0;issued_correction_peak=0.0;closed_gate_nonzero_frames=0
        started=time.monotonic();episodes=[]
        for t in range(length):
            chosen=[self.scenarios[i] for i in self.identities]
            raw=np.stack([x.obs for x in self.last])
            issued_correction_frames+=int((np.abs(raw[:,15])>1e-12).sum())
            issued_correction_peak=max(issued_correction_peak,float(np.abs(raw[:,15]).max()))
            closed_gate_nonzero_frames+=int(((raw[:,16]<.5)&(np.abs(raw[:,15])>1e-12)).sum())
            features=feature_array(raw,chosen,self.steps)
            mask=action_mask(raw,self.steps)
            reset_before[t]=self.resets;observations[t]=features;masks[t]=mask
            output=self.cpu.sample(features,self.rng,self.resets,mask)
            self.resets[:]=0
            latents[t]=output["latent_actions"];oldmeans[t]=output["means"]
            log_probs[t]=output["log_prob"];values[t]=output["values"]
            requests=physical_actions(output["actions"],chosen);requests_saved[t]=requests
            results=self.plant.step_batch(self.contexts,requests)
            self.steps+=1
            issued=np.array([[r.obs[13],r.obs[15]] for r in results])
            for j,result in enumerate(results):
                if result.terminated and not result.physical_failure:
                    raise FloatingPointError("Native software/nonfinite termination; retain rejection")
                rewards[t,j]=reward_value(result,issued[j],self.previous[j],self.reward_cfg)
                physical_interval[t,j]=[result.native_samples,result.peak_normal_error_N,
                    result.mean_squared_normal_error_N2,result.mean_squared_travel_error_m2,
                    result.mean_squared_current_A2,result.end_s,
                    float(result.physical_failure),chosen[j]["scenario"]]
                terminated[t,j]=bool(result.physical_failure)
                truncated[t,j]=not terminated[t,j] and self.steps[j]>=round(chosen[j]["duration_s"]/DT)
                self.episode_peak[j]=max(self.episode_peak[j],result.peak_normal_error_N)
                self.episode_returns[j]+=float(rewards[t,j])
            # Bootstrap the actual final observation with a CLONED post-action
            # recurrent state, before reset. This never advances actor state.
            next_raw=np.stack([x.obs for x in results])
            next_feature=feature_array(next_raw,chosen,self.steps)
            _,next_values[t]=self.cpu.infer(next_feature,state=self.cpu.state.copy())
            self.last=list(results);self.previous=issued
            for j in np.flatnonzero(terminated[t]|truncated[t]):
                row={"episode":self.episode_count,"update":update,
                     "identity":chosen[j]["identity"],"scenario":int(self.identities[j]),
                     "seed":int(self.seeds[j]),"steps":int(self.steps[j]),
                     "terminated":bool(terminated[t,j]),"truncated":bool(truncated[t,j]),
                     "peakNormalForceError_N":float(self.episode_peak[j]),
                     "return":float(self.episode_returns[j]),
                     "passed":bool(truncated[t,j] and self.episode_peak[j]<=.5)}
                if terminated[t,j]:
                    record=self.contexts[j].take_record()
                    path=self.failed_dir/f"failed_{self.episode_count:06d}.npz"
                    np.savez_compressed(path,**record)
                    row["nativeFailureFile"]=str(path.resolve());row["nativeFailureSha256"]=sha(path)
                else:
                    self.contexts[j].take_record(copy=False) # reset below releases the scored buffer
                with (self.directory/"training_episodes.jsonl").open("a") as stream:
                    stream.write(json.dumps(row,allow_nan=False)+"\n")
                episodes.append(row);self.episode_count+=1
                i=int(self.rng.choice(self.train_ids));s=int(self.rng.integers(100000,2000000000))
                self.identities[j]=i;self.seeds[j]=s
                self.contexts[j].reset(scenario=i,seed=s,baseline=False)
                self.last[j]=initial_result(self.contexts[j]);self.steps[j]=0;self.resets[j]=1
                self.previous[j]=[self.scenarios[i]["knownEquilibriumCurrent_A"],0]
                self.episode_peak[j]=self.last[j].peak_normal_error_N;self.episode_returns[j]=0
        self.transitions+=length*count
        path=self.directory/f"rollout_{update:04d}.npz"
        np.savez_compressed(path,observations=observations,latent_actions=latents,
                            old_means=oldmeans,log_probs=log_probs,values=values,rewards=rewards,
                            terminated=terminated,truncated=truncated,next_values=next_values,
                            reset_before=reset_before,action_mask=masks,initial_state=initial_state,
                            requested_actions=requests_saved,physical_interval=physical_interval)
        tensors={"observations":observations,"latent_actions":latents,"log_probs":log_probs,
                 "values":values,"rewards":rewards,"terminated":terminated,"truncated":truncated,
                 "next_values":next_values,"reset_before":reset_before,"action_mask":masks,
                 "initial_state":initial_state}
        device=next(self.policy.parameters()).device
        batch=RolloutBatch(**{k:torch.as_tensor(v,device=device) for k,v in tensors.items()})
        metrics={"collectionElapsed_s":time.monotonic()-started,"transitions":length*count,
                 "totalTransitions":self.transitions,"episodesCompleted":len(episodes),
                 "episodePasses":sum(r["passed"] for r in episodes),
                 "physicalFailures":int(terminated.sum()),"meanReward":float(rewards.mean()),
                 "nativeTicks":int(physical_interval[:,:,0].sum()),
                 "rolloutPeakNormalForceError_N":float(physical_interval[:,:,1].max()),
                 "nonzeroIssuedRobotCorrectionFrames":issued_correction_frames,
                 "peakIssuedRobotCorrection_m_s":issued_correction_peak,
                 "heldCorrectionWhileGateClosedFrames":closed_gate_nonzero_frames,
                 "meanSquaredNormalError_N2":float(physical_interval[:,:,2].mean()),
                 "rolloutFile":str(path.resolve()),"rolloutSha256":sha(path)}
        if update==0:
            with torch.no_grad():
                replay=self.policy.sequence(batch.observations,batch.initial_state,batch.reset_before)
                log,_,_=self.policy.evaluate_latent(replay.mean,batch.latent_actions,batch.action_mask)
                metrics["behaviorReplayMeanMaxResidual"]=float((replay.mean-torch.as_tensor(oldmeans,device=device)).abs().max())
                metrics["behaviorReplayValueMaxResidual"]=float((replay.value-batch.values).abs().max())
                metrics["behaviorReplayLogProbMaxResidual"]=float((log-batch.log_probs).abs().max())
                assert metrics["behaviorReplayMeanMaxResidual"]<1e-4
                assert metrics["behaviorReplayLogProbMaxResidual"]<.005
        return batch,metrics


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--native-build",required=True);parser.add_argument("--bank",required=True)
    parser.add_argument("--output",required=True);parser.add_argument("--seed",type=int,default=7101)
    parser.add_argument("--updates",type=int,default=80);parser.add_argument("--environments",type=int,default=16)
    parser.add_argument("--rollout",type=int,default=2048);parser.add_argument("--eval-every",type=int,default=10)
    parser.add_argument("--device",default="cuda");parser.add_argument("--no-selection",action="store_true")
    args=parser.parse_args();directory=Path(args.output);directory.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(4);torch.manual_seed(args.seed);np.random.seed(args.seed)
    if args.device.startswith("cuda"):torch.cuda.manual_seed_all(args.seed)
    torch.backends.cuda.matmul.allow_tf32=False
    torch.backends.cudnn.allow_tf32=False
    from native_api import NativePlant
    source_plant=NativePlant(args.native_build);scenarios=get_scenarios(source_plant,args.bank)
    native_inputs=directory/"execution_inputs";native_inputs.mkdir()
    for name in ("libacf_native.so","metadata.json","bridge_build.json","acf_rl_native.slx",
                 "native_parameter_bank.c","parameter_mapping.json","acf_rl_native_context.c"):
        source=Path(args.native_build)/name
        if source.is_file():shutil.copy2(source,native_inputs/name)
    write_json(native_inputs/"scenarios.json",scenarios)
    shutil.copytree(Path(__file__).parent,directory/"frozen_sources",ignore=shutil.ignore_patterns("__pycache__"))
    plant=NativePlant(native_inputs)
    write_json(directory/"consumed_scenarios.json",scenarios)
    policy=UnifiedPolicy(len(FEATURE_NAMES),initial_log_std=(-5.0,-1.5)).to(args.device)
    cfg=PPOConfig(chunk_length=min(2048,args.rollout),minibatch_sequences=min(8,args.environments))
    trainer=PPOTrainer(policy,cfg);reward_cfg=RewardConfig()
    report={"status":"training","date":"2026-10-05","seed":args.seed,"args":vars(args),
            "architecture":policy.config(),"parameterCount":sum(p.numel() for p in policy.parameters()),
            "featureNames":FEATURE_NAMES,"reward":reward_cfg.contract(),"ppo":asdict(cfg),
            "nativeBuild":str(Path(args.native_build).resolve()),"bankSha256":sha(args.bank),
            "frozenNativeFileHashes":{p.name:sha(p) for p in native_inputs.iterdir() if p.is_file()},
            "inputSourceHashes":{p.name:sha(p) for p in Path(__file__).parent.glob("*.py")},
            "teacherActionsUsed":False,"dynamicPIUnderCurrentActor":False,
            "heldoutOutcomesUsedDuringTraining":False,"historyCarriedBetweenRollouts":True,
            "updates":[],"selection":[]}
    write_json(directory/"results.json",report)
    collector=Collector(plant,scenarios,policy,directory/"rollouts",args.environments,args.seed,reward_cfg)
    selection_ids=[i for i,r in enumerate(scenarios) if r["split"]=="selection"]
    best_rank=None;start=time.monotonic()
    try:
        for update in range(args.updates):
            batch,metrics=collector.collect(args.rollout,update)
            timer=time.monotonic();optimization=trainer.update(batch)
            metrics.update(optimization);metrics["optimizationElapsed_s"]=time.monotonic()-timer
            metrics["updateIndex"]=update;metrics["elapsed_s"]=time.monotonic()-start
            report["updates"].append(metrics);collector.cpu.refresh(policy)
            torch.save({**trainer.state_dict(),"rng":collector.rng.bit_generator.state},directory/"latest_training.pt")
            print(f"RL_UPDATE seed={args.seed} update={update+1}/{args.updates} transitions={collector.transitions} "
                  f"errorRms={metrics['meanSquaredNormalError_N2']**.5:.6g}N peak={metrics['rolloutPeakNormalForceError_N']:.6g}N "
                  f"fails={metrics['physicalFailures']} collect={metrics['collectionElapsed_s']:.2f}s learn={metrics['optimizationElapsed_s']:.2f}s",flush=True)
            if not args.no_selection and ((update+1)%args.eval_every==0 or update+1==args.updates):
                selected=evaluate(plant,scenarios,selection_ids,policy,directory/"selection",
                                  f"selection_update_{update+1:04d}")
                rank=(selected["physicalFailures"],-selected["passed"],selected["worstNormalForceError_N"],
                      selected["meanRmsNormalForceError_N"])
                report["selection"].append({"update":update+1,"rank":rank,
                    "worstNormalForceError_N":selected["worstNormalForceError_N"],"passed":selected["passed"],
                    "physicalFailures":selected["physicalFailures"]})
                if best_rank is None or rank<best_rank:
                    best_rank=rank
                    export_policy(policy,directory/"selected_policy",FEATURE_NAMES,
                                  {"seed":args.seed,"selectionUpdate":update+1,"selectionRank":rank,
                                   "trainedTransitions":collector.transitions,"teacherUsed":False})
                    report["selectedUpdate"]=update+1;report["selectedRank"]=rank
                print(f"RL_SELECTION seed={args.seed} update={update+1} pass={selected['passed']}/8 "
                      f"peak={selected['worstNormalForceError_N']:.6g}N fails={selected['physicalFailures']}",flush=True)
            write_json(directory/"results.json",report)
            del batch
        export_policy(policy,directory/"final_policy",FEATURE_NAMES,{"seed":args.seed,"trainedTransitions":collector.transitions})
        report["status"]="training_complete";report["trainedTransitions"]=collector.transitions
        report["episodesCompleted"]=collector.episode_count;report["elapsed_s"]=time.monotonic()-start
    except BaseException:
        report["status"]="software_rejection";report["error"]=traceback.format_exc()
        write_json(directory/"results.json",report);raise
    finally:
        collector.close();write_json(directory/"results.json",report)


if __name__=="__main__":main()
