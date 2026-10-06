"""Reward-only whole-episode search for one recurrent current/robot policy.

The C evaluator calls the accepted Simulink-generated plant. No independent
plant model, teacher actions or supervised targets are used. Selected policy
coefficients are evolved from complete training-episode force/centering costs.
"""
from __future__ import annotations
import argparse
import ctypes as ct
from datetime import datetime
import hashlib
import json
import math
import multiprocessing as mp
from pathlib import Path
import shutil
import subprocess
import sys
import time

import numpy as np

HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'rl_unified_20261005'
ROOT=HERE.parents[2]
STAT_NAMES=['peakNormalForceError_N','rmsNormalForceError_N','mseNormalForceError_N2',
 'finalTravelError_m','rmsTravelError_m','peakCurrent_A','peakVoltage_V',
 'voltageClipFraction','currentLimitFraction','contactLoss','travelStop',
 'softwareFailure','completed','nativeSamples','controllerDecisions','endTime_s',
 'issuedRobotFrames','eligibleRobotDecisions','peakIssuedRobot_m_s','initialForceError_N',
 'maxTravel_m','minTravel_m','forceOutsideFraction','nativeCommandTime_s','recordSamples',
 'bandExcessIntegralUpper_m2_s','maxAbsoluteTravelError_m','timeOutsideBandUpper_s','initialTravelErrorNative_m']
DOUBLE_PTR=ct.POINTER(ct.c_double)
FLOAT_PTR=ct.POINTER(ct.c_float)

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def write_json(path,value):
    Path(path).write_text(json.dumps(value,indent=2,allow_nan=False,
          default=lambda x:x.item() if isinstance(x,np.generic) else str(x))+'\n')

def scenario_context(row):
    return np.asarray([row['force_N'],row['knownEquilibriumCurrent_A'],row['midTravel_m'],
      row['angle_rad'],row['identifiedMassS_kg'],row['identifiedMassB_kg'],row['massA_kg'],
      row['identifiedK_N_m'],row['identifiedC_N_s_m'],row['identifiedKf_N_A'],
      row['coilR_ohm'],row['coilL_H'],row['usableVoltage_V'],row['peakCurrent_A'],
      row['travelMax_m']-row['travelMin_m'],row.get('identifiedCurrentResponse_s',row.get('currentResponse_s',.0001)),
      row.get('identifiedRobotDelay_s',.08),row.get('identifiedRobotResponse_s',row.get('identifiedRobotResponseTime_s',.06)),
      row.get('identifiedAntiAliasCutoff_Hz',8000),row.get('identifiedEmaTime_s',.00005),
      row.get('identifiedPipelineDelay_s',.00002),row['duration_s'],row.get('acceptedRecenterBand_m',.0075),row.get('robotActionScale_m_s',.005)],dtype=np.float64)

class EpisodeEvaluator:
    def __init__(self,library,scenarios,repeat=1):
        self.library=ct.CDLL(str(Path(library).resolve()))
        self.fn=self.library.acf_search_episode
        self.fn.argtypes=[ct.c_int,ct.c_int,DOUBLE_PTR,FLOAT_PTR,ct.c_int,ct.c_int,
                          DOUBLE_PTR,DOUBLE_PTR,ct.c_size_t,DOUBLE_PTR,ct.c_size_t]
        self.fn.restype=ct.c_int
        self.feature_fn=self.library.acf_search_features
        self.feature_fn.argtypes=[DOUBLE_PTR,DOUBLE_PTR,ct.c_int64,FLOAT_PTR]
        self.scenarios=scenarios;self.contexts=[scenario_context(r) for r in scenarios]
        self.repeat=repeat
        self.stats_count=int(self.library.acf_search_stats_count()) if hasattr(self.library,'acf_search_stats_count') else 25
    def evaluate(self,weights,indices,seed_offset=0,record_dir=None,baseline=False):
        weights=np.ascontiguousarray(weights,dtype=np.float32);rows=[]
        for index in indices:
            scenario=self.scenarios[index];stats=np.zeros(self.stats_count,np.float64)
            context=self.contexts[index]
            if record_dir is not None:
                capacity=round(scenario['duration_s']/ (1e-4/30))+64
                record=np.empty((capacity,31),np.float64)
                actions=np.empty((round(scenario['duration_s']/(1e-4*self.repeat))+1,45),np.float64)
            else:record=actions=None
            status=self.fn(int(scenario['scenario']),int(scenario['seed'])+seed_offset,
                context.ctypes.data_as(DOUBLE_PTR),weights.ctypes.data_as(FLOAT_PTR),self.repeat,int(baseline),
                stats.ctypes.data_as(DOUBLE_PTR),record.ctypes.data_as(DOUBLE_PTR) if record is not None else None,
                0 if record is None else len(record),actions.ctypes.data_as(DOUBLE_PTR) if actions is not None else None,
                0 if actions is None else len(actions))
            if status:raise RuntimeError(f'C evaluator rejected status {status}')
            row={**dict(zip(STAT_NAMES[:self.stats_count],stats.tolist(),strict=True)),'scenario':index,'identity':scenario['identity'],
                 'seed':int(scenario['seed'])+seed_offset,'force_N':scenario['force_N'],
                 'massS_kg':scenario['massS_kg'],'angle_rad':scenario['angle_rad'],
                 'initialTravelError_m':scenario['offset_m']}
            row['physicalFailure']=bool(row['contactLoss'] or row['travelStop'])
            row['passed']=bool(row['completed'] and row['peakNormalForceError_N']<=.5)
            row['recenterProgressFraction']=float(1-abs(row['finalTravelError_m'])/max(abs(scenario['offset_m']),1e-6))
            row['recenterPassed']=bool(abs(scenario['offset_m'])<.0002 or (row['completed'] and row['recenterProgressFraction']>=.1 and row['issuedRobotFrames']>0))
            if row['softwareFailure']:raise FloatingPointError(f'Native software failure {row}')
            if record is not None:
                record=record[:int(stats[24])];actions=actions[:int(stats[14])]
                error=record[:,4]-scenario['force_N']
                row['peakNormalForceError_N']=float(np.max(np.abs(error)))
                row['rmsNormalForceError_N']=float(np.sqrt(np.mean(error**2)))
                row['nativeSamples']=len(error)
                row['nativeGridComplete']=bool(len(error)==round(record[-1,0]/(1e-4/30))+1 and record[0,0]==0)
                assert row['nativeGridComplete']
                target=Path(record_dir);target.mkdir(parents=True,exist_ok=True)
                nativefile=target/f"{scenario['identity']}_native.npz"
                np.savez_compressed(nativefile,time=record[:,0],inputs=record[:,1:4],truth=record[:,4:])
                row['nativeHistoryFile']=str(nativefile.resolve());row['nativeHistorySha256']=sha(nativefile)
                actionfile=target/f"{scenario['identity']}_actions.npz"
                np.savez_compressed(actionfile,history=actions)
                row['actionHistoryFile']=str(actionfile.resolve());row['actionHistorySha256']=sha(actionfile)
            rows.append(row)
        return rows

def aggregate(rows):
    return {'recordings':len(rows),'passed':sum(r['passed'] for r in rows),
      'physicalFailures':sum(r['physicalFailure'] for r in rows),
      'recenterFailures':sum(not r['recenterPassed'] for r in rows),
      'meanRecenterProgressFraction':float(np.mean([r['recenterProgressFraction'] for r in rows if abs(r['initialTravelError_m'])>=.0005 and 'stress' not in r['identity']])) if any(abs(r['initialTravelError_m'])>=.0005 and 'stress' not in r['identity'] for r in rows) else None,
      'worstNormalForceError_N':max(r['peakNormalForceError_N'] for r in rows),
      'meanRmsNormalForceError_N':float(np.mean([r['rmsNormalForceError_N'] for r in rows])),
      'nativeSamples':sum(r['nativeSamples'] for r in rows),
      'controllerDecisions':sum(r['controllerDecisions'] for r in rows),
      'eligibleRobotDecisions':sum(r['eligibleRobotDecisions'] for r in rows),
      'issuedRobotFrames':sum(r['issuedRobotFrames'] for r in rows),
      'rows':rows}

def objective(rows,centering_weight=.05,force_excess_weight=0.0):
    peak=np.array([r['peakNormalForceError_N'] for r in rows])
    mse=np.array([r['mseNormalForceError_N2'] for r in rows])
    travel=np.array([r['finalTravelError_m']/.005 for r in rows if 'stress' not in r['identity']])
    excess=np.maximum(peak-.45,0)
    failure=sum(r['physicalFailure'] or not r['completed'] for r in rows)
    # Every component is from actual native trajectory rewards, not teacher labels.
    return float(1e4*failure + .35*np.max(peak**2)+.3*np.mean(peak**2)+np.mean(mse)
                 +centering_weight*(np.mean(travel**2) if len(travel) else 0)
                 +force_excess_weight*(np.max(excess**2)+.4*np.mean(excess**2)))

def joint_band_score(rows):
    """Frozen common selection score; band excursion is soft, stops are separate."""
    peak=max(r['peakNormalForceError_N'] for r in rows)
    mse=np.mean([(r['rmsNormalForceError_N']/.05)**2 for r in rows])
    integrated=np.mean([r['bandExcessIntegralUpper_m2_s']/1e-6/max(r['endTime_s'],1e-4) for r in rows])
    final=np.mean([(max(abs(r['finalTravelError_m'])-.0075,0)/.001)**2 for r in rows])
    return float((peak/.45)**2+.02*mse+integrated+.5*final)


def build_evaluator(native_build,policy_dir,out):
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    source=HERE/'reward_search_native.c';lib=out/'reward_search_native.so'
    command=['cc','-std=c99','-O3','-fPIC','-shared','-fno-fast-math','-ffp-contract=off',
        '-I'+str(Path(policy_dir).resolve()),str(source),str(Path(policy_dir)/'policy_inference.c'),
        str(Path(native_build).resolve()/'libacf_native.so'),'-Wl,-rpath,'+str(Path(native_build).resolve()),
        '-lm','-o',str(lib)]
    result=subprocess.run(command,capture_output=True,text=True)
    (out/'compile.log').write_text(result.stdout+result.stderr);result.check_returncode()
    write_json(out/'compile_manifest.json',{'command':command,'sourceSha256':sha(source),
       'nativeLibrarySha256':sha(Path(native_build)/'libacf_native.so'),
       'policySourceSha256':sha(Path(policy_dir)/'policy_inference.c'),'evaluatorSha256':sha(lib)})
    return lib

_WORKER=None

def init_worker(library,scenarios,repeat):
    global _WORKER
    _WORKER=EpisodeEvaluator(library,scenarios,repeat)

def worker_eval(task):
    weights,indices,seed_offset,failed_folder=task
    rows=_WORKER.evaluate(weights,indices,seed_offset)
    failed=[r['scenario'] for r in rows if r['physicalFailure']]
    if failed and failed_folder:
        recorded=_WORKER.evaluate(weights,failed,seed_offset,record_dir=failed_folder)
        write_json(Path(failed_folder)/'failures.json',{'rows':recorded,'replayExcludedFromTrainingCount':True})
    return rows

class CMA:
    """Standard rank-mu covariance adaptation with antithetic proposals."""
    def __init__(self,x,sigma,population,rng):
        self.mean=np.array(x,np.float64);self.n=len(x);self.sigma=sigma;self.population=population;self.rng=rng
        self.mu=population//2;self.weights=np.log(self.mu+.5)-np.log(np.arange(1,self.mu+1));self.weights/=self.weights.sum()
        self.mueff=1/np.sum(self.weights**2);n=self.n
        self.cc=(4+self.mueff/n)/(n+4+2*self.mueff/n);self.cs=(self.mueff+2)/(n+self.mueff+5)
        self.c1=2/((n+1.3)**2+self.mueff);self.cmu=min(1-self.c1,2*(self.mueff-2+1/self.mueff)/((n+2)**2+self.mueff))
        self.damps=1+2*max(0,math.sqrt((self.mueff-1)/(n+1))-1)+self.cs
        self.pc=np.zeros(n);self.ps=np.zeros(n);self.C=np.eye(n);self.generation=0
        self.chi=math.sqrt(n)*(1-1/(4*n)+1/(21*n*n))
    def ask(self):
        vals,vecs=np.linalg.eigh(self.C);vals=np.maximum(vals,1e-12)
        self.sqrtC=(vecs*np.sqrt(vals))@vecs.T;self.invsqrtC=(vecs/np.sqrt(vals))@vecs.T
        half=self.rng.standard_normal((math.ceil(self.population/2),self.n))
        z=np.concatenate([half,-half],axis=0)[:self.population]
        self.candidates=self.mean+self.sigma*(z@self.sqrtC.T)
        return self.candidates
    def tell(self,cost):
        ordered=np.argsort(cost,kind='stable');old=self.mean.copy()
        chosen=self.candidates[ordered[:self.mu]];self.mean=self.weights@chosen
        y=(self.mean-old)/self.sigma
        self.ps=(1-self.cs)*self.ps+math.sqrt(self.cs*(2-self.cs)*self.mueff)*(self.invsqrtC@y)
        self.generation+=1
        hsig=float(np.linalg.norm(self.ps)/math.sqrt(1-(1-self.cs)**(2*self.generation))/self.chi < 1.4+2/(self.n+1))
        self.pc=(1-self.cc)*self.pc+hsig*math.sqrt(self.cc*(2-self.cc)*self.mueff)*y
        dy=(chosen-old)/self.sigma
        rankmu=(dy.T*self.weights)@dy
        self.C=(1-self.c1-self.cmu)*self.C+self.c1*(np.outer(self.pc,self.pc)+(1-hsig)*self.cc*(2-self.cc)*self.C)+self.cmu*rankmu
        self.C=(self.C+self.C.T)/2
        self.sigma*=math.exp((self.cs/self.damps)*(np.linalg.norm(self.ps)/self.chi-1))
        self.sigma=float(np.clip(self.sigma,1e-5,2))


def load_scenarios(native_build,bank):
    raw=json.loads(Path(bank).read_text())['cases']
    overlay=json.loads((Path(native_build)/'scenarios.json').read_text())
    return [dict(a,**b) for a,b in zip(raw,overlay,strict=True)]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--native-build',required=True);parser.add_argument('--bank',required=True)
    parser.add_argument('--policy',required=True);parser.add_argument('--output',required=True)
    parser.add_argument('--policy-module-dir',default=str(HERE))
    parser.add_argument('--mode',choices=['benchmark','evaluate','search'],default='benchmark')
    parser.add_argument('--indices',default='0,3,7,12,18,23')
    parser.add_argument('--seed',type=int,default=8201);parser.add_argument('--workers',type=int,default=3)
    parser.add_argument('--population',type=int,default=16);parser.add_argument('--generations',type=int,default=100)
    parser.add_argument('--sigma',type=float,default=.15);parser.add_argument('--minutes',type=float,default=100)
    parser.add_argument('--parameters',default='actor.weight,actor_skip.weight')
    parser.add_argument('--parameter-spec',help='JSON list of name, flattened indices, and search scale')
    parser.add_argument('--initialization',choices=['checkpoint','force_balance'],default='checkpoint')
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--train-batch-size',type=int,default=0)
    parser.add_argument('--batch-profile',choices=['all_strata','current_stress'],default='all_strata')
    parser.add_argument('--until-utc',help='Optional absolute ISO8601 deadline for starting another generation')
    parser.add_argument('--selection-every',type=int,default=5)
    parser.add_argument('--centering-weight',type=float,default=.05)
    parser.add_argument('--require-centering',action='store_true')
    parser.add_argument('--force-excess-weight',type=float,default=0.0)
    parser.add_argument('--selection-baseline')
    parser.add_argument('--band-objective',action='store_true')
    parser.add_argument('--robot-scale',type=float,default=.005)
    parser.add_argument('--initial-robot-band',action='store_true')
    args=parser.parse_args()
    folder=Path(args.output);folder.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(OLD));sys.path.insert(0,args.policy_module_dir)
    import torch
    torch.set_num_threads(1)
    from policy_export import load_policy,export_policy,flat_weights,arrays_from_policy
    from env import FEATURE_NAMES,feature_array
    policy=load_policy(args.policy)
    if args.initialization=='force_balance':
        with torch.no_grad():
            policy.actor.weight.zero_();policy.actor.bias.zero_()
            policy.actor_skip.weight.zero_();policy.actor_skip.weight[0,0]=-.025
            policy.actor_skip.weight[1,17]=1.0
            policy.actor_aux.weight.zero_();policy.actor_aux.weight[0,0]=-1.0;policy.actor_aux.weight[0,2]=1.0
    if args.initial_robot_band:
        with torch.no_grad():
            policy.actor_skip.weight[1].zero_();policy.actor_aux.weight[1].zero_();policy.actor.weight[1].zero_();policy.actor.bias[1]=0
            policy.encoder.weight[30].zero_();policy.encoder.weight[31].zero_()
            policy.encoder.weight[30,17]=4.;policy.encoder.weight[31,17]=-4.
            policy.encoder.bias[30]=-6.;policy.encoder.bias[31]=-6.
            policy.actor.weight[0,30:32]=0
            policy.actor.weight[1,30]=3.;policy.actor.weight[1,31]=-3.
            policy.actor_skip.weight[1,14]=-.55;policy.actor_skip.weight[1,15]=.55
    mapping_contract={'currentOffset_A':2.0,'robotRequest_m_s':args.robot_scale}
    frozen=folder/'frozen_sources';frozen.mkdir()
    for source in [Path(__file__),HERE/'reward_search_native.c']:
        shutil.copy2(source,frozen/source.name)
    write_json(folder/'run_configuration.json',{'arguments':vars(args),
       'sourceCheckpointSha256':sha(args.policy),'sourceCheckpoint':str(Path(args.policy).resolve()),
       'nativeLibrarySha256':sha(Path(args.native_build)/'libacf_native.so'),
       'parameterSpecSha256':sha(args.parameter_spec) if args.parameter_spec else None,
       'startedUtc':datetime.now().astimezone().isoformat()})
    export_policy(policy,folder/'initial_policy',FEATURE_NAMES,{'optimizer':'reward_only_antithetic_CMA','physicalActionScales':mapping_contract,'smoothRobotBandInitialization':args.initial_robot_band})
    weights=flat_weights(policy);repeat=round(policy.dt/1e-4)
    scenarios=load_scenarios(args.native_build,args.bank)
    for row in scenarios:row['robotActionScale_m_s']=args.robot_scale
    write_json(folder/'consumed_scenarios.json',scenarios)
    library=build_evaluator(args.native_build,folder/'initial_policy',folder)
    evaluator=EpisodeEvaluator(library,scenarios,repeat)
    # Reproduce all forty causal features exactly, including identified context.
    rng=np.random.default_rng(args.seed);errors=[]
    for k,row in enumerate(scenarios):
        observed=rng.normal(size=18);step=int(rng.integers(0,40000));cf=np.empty(40,np.float32)
        evaluator.feature_fn(observed.ctypes.data_as(DOUBLE_PTR),evaluator.contexts[k].ctypes.data_as(DOUBLE_PTR),step,cf.ctypes.data_as(FLOAT_PTR))
        pf=feature_array(observed[None], [dict(row,identifiedRobotResponse_s=row.get('identifiedRobotResponseTime_s',.06),identifiedCurrentResponse_s=row.get('currentResponse_s',.0001))],np.array([step]))[0]
        errors.append(float(np.max(np.abs(cf-pf))))
    assert max(errors)==0,(max(errors),errors)
    write_json(folder/'feature_parity.json',{'cases':len(errors),'maxAbsoluteDifference':max(errors),'exact':True})
    indices=[i for i,r in enumerate(scenarios) if r['split']=='train'] if args.indices=='train' else [int(x) for x in args.indices.split(',')]
    start=time.monotonic();initial=evaluator.evaluate(weights,indices,record_dir=folder/'initial_histories' if args.record else None)
    if args.band_objective:
        for row in initial:row['recenterPassed']=True;row['recenterScreenApplied']=False
    initial_report=aggregate(initial);initial_report['elapsed_s']=time.monotonic()-start
    write_json(folder/'initial.json',initial_report)
    print('INITIAL '+json.dumps({k:v for k,v in initial_report.items() if k!='rows'}),flush=True)
    if args.mode in ('benchmark','evaluate'):return
    # Use exact tensor layout from exporter; optimization operates on designated
    # policy weights only and never changes the physical graph or action limits.
    layout=json.loads((folder/'initial_policy/policy_weights.json').read_text())['offsets']
    spec=json.loads(Path(args.parameter_spec).read_text()) if args.parameter_spec else [dict(name=n) for n in args.parameters.split(',')]
    requested=[s['name'] for s in spec]
    columns=[];parameter_labels=[]
    for item in spec:
        if 'terms' in item:
            column=np.zeros(len(weights),np.float32)
            for term in item['terms']:
                name=term['name'];key=name.replace('.','_')
                if name.startswith('recurrent.layers.'):
                    parts=name.split('.');key=f'recurrent_{parts[2]}_{parts[3]}'
                info=layout[key]
                assert len(term['indices'])==len(term['factors'])
                for i,factor in zip(term['indices'],term['factors']):
                    assert 0<=i<info['count']
                    column[info['offset']+i]+=item.get('scale',1.0)*factor
            columns.append(column);parameter_labels.append(item)
            continue
        name=item['name'];key=name.replace('.','_')
        if name.startswith('recurrent.layers.'):
            parts=name.split('.');key=f'recurrent_{parts[2]}_{parts[3]}'
        if key not in layout:raise KeyError(f'{name} -> {key}; layout={list(layout)}')
        info=layout[key]
        groups=item.get('groups',[[i] for i in item.get('indices',list(range(info['count'])))])
        for group in groups:
            if isinstance(group,dict):ii=group['indices'];factors=group['factors']
            else:ii=group;factors=[1.0]*len(ii)
            assert all(0<=i<info['count'] for i in ii) and len(ii)==len(factors)
            column=np.zeros(len(weights),np.float32)
            for i,factor in zip(ii,factors):column[info['offset']+i]=item.get('scale',1.0)*factor
            columns.append(column);parameter_labels.append({'name':name,'indices':ii,'factors':factors,'scale':item.get('scale',1.0)})
    mapping=np.stack(columns,axis=1)
    write_json(folder/'parameter_spec.json',spec)
    center=np.zeros(len(columns),np.float64)
    search=CMA(center,args.sigma,args.population,rng)
    def export_flat(vector,target,provenance):
        with torch.no_grad():
            for name,tensor in policy.named_parameters():
                key=name.replace('.','_')
                if name.startswith('recurrent.layers.'):
                    parts=name.split('.');key=f'recurrent_{parts[2]}_{parts[3]}'
                if key not in layout:raise KeyError((name,key))
                info=layout[key]
                tensor.copy_(torch.from_numpy(vector[info['offset']:info['offset']+info['count']].reshape(info['shape'])))
        return export_policy(policy,target,FEATURE_NAMES,{**provenance,'physicalActionScales':mapping_contract,'acceptedRecenterBand_m':.0075})
    cost_function=(lambda rows:joint_band_score(rows)+args.force_excess_weight*max(0,max(r['peakNormalForceError_N'] for r in rows)-.45)**2+1e4*sum(r['physicalFailure'] or not r['completed'] for r in rows)) if args.band_objective else (lambda rows:objective(rows,centering_weight=args.centering_weight,force_excess_weight=args.force_excess_weight))
    best=cost_function(initial);best_weights=weights.copy();best_gen=0
    selection_ids=[i for i,r in enumerate(scenarios) if r['split']=='selection']
    references={}
    if args.selection_baseline:
        baseline_rows=json.loads(Path(args.selection_baseline).read_text())['cases']
        references={int(r['scenario']):r for r in baseline_rows[:12] if r['initial_abs_recenter_m']>=.0005}
        shutil.copy2(args.selection_baseline,folder/'centering_baseline.json')
    def selection_screen(rows):
        for row in rows:
            if args.band_objective:
                row['recenterScreenApplied']=False;row['recenterPassed']=True
                row['finalBandExcess_m']=max(abs(row['finalTravelError_m'])-.0075,0)
                continue
            ref=references.get(int(row['scenario']))
            if not ref:
                row['recenterScreenApplied']=False
                if references:row['recenterPassed']=True
                continue
            required_reduction=.5*max(0,ref['initial_abs_recenter_m']-ref['final_abs_recenter_m'])
            row['recenterScreenApplied']=True
            row['recenterTargetAbsoluteFinalError_m']=ref['initial_abs_recenter_m']-required_reduction
            row['recenterPassed']=bool(row['completed'] and abs(row['finalTravelError_m'])<=row['recenterTargetAbsoluteFinalError_m']+1e-9)
        return rows
    best_selection=None;selection_reports=[];history=[];evaluations=len(initial)
    if args.band_objective:
        initial_selection=aggregate(selection_screen(evaluator.evaluate(weights,selection_ids)))
        initial_selection['jointScore']=joint_band_score(initial_selection['rows'])
        initial_rank=(initial_selection['physicalFailures'],-initial_selection['passed'],initial_selection['jointScore'],initial_selection['worstNormalForceError_N'],initial_selection['meanRmsNormalForceError_N'])
        initial_selection.update(generation=0,proposal='initial_policy',rank=initial_rank)
        selection_reports.append(initial_selection);evaluations+=len(selection_ids)
        best_selection=initial_rank;np.save(folder/'selected_flat_weights.npy',weights)
        checkpoint_dir=folder/'checkpoints'/'generation_0000'
        export_flat(weights,checkpoint_dir,{'optimizer':'initial_or_warm_start_before_new_reward_search','selectionRank':initial_rank,'sourceCheckpointSha256':sha(args.policy)})
        write_json(folder/'latest_selected_checkpoint.json',{'directory':str(checkpoint_dir.resolve()),'generation':0,'rank':initial_rank,'checkpointSha256':sha(checkpoint_dir/'policy_checkpoint.pt')})
        write_json(folder/'selection.json',selection_reports)
        print('INITIAL_SELECTION '+json.dumps({k:v for k,v in initial_selection.items() if k!='rows'}),flush=True)
    deadline=time.monotonic()+args.minutes*60
    if args.until_utc:
        deadline=min(deadline,time.monotonic()+datetime.fromisoformat(args.until_utc).timestamp()-time.time())
    batch_mode=bool(args.train_batch_size and args.train_batch_size<len(indices))
    if args.batch_profile=='current_stress':
        assert args.train_batch_size==12,'Current refinement uses the declared3/5/2/2 twelve-case profile'
        group_ids=[np.array([i for i in indices if 40<=i<48]),np.array([i for i in indices if 12<=i<40 and scenarios[i].get('bumpHeight_m',0)>=.001]),np.array([i for i in indices if 96<=i<128]),np.array([i for i in indices if i>=128])]
        assert all(len(x) for x in group_ids),'Current refinement requires every declared stratum'
        fixed_batch_sizes=[3,5,2,2]
    else:
        group_ids=[np.array([i for i in indices if i<12]),np.array([i for i in indices if 12<=i<40]),np.array([i for i in indices if 40<=i<48]),np.array([i for i in indices if 96<=i<128]),np.array([i for i in indices if i>=128])]
        fixed_batch_sizes=None
    group_ids=[x for x in group_ids if len(x)]
    queues=[list(rng.permutation(g)) for g in group_ids]
    def next_batch():
        if not batch_mode:return indices
        selected_ids=[]
        # Rotate everycase with commoncandidates; eachbatchincludesstress.
        sizes=fixed_batch_sizes or [max(1,round(args.train_batch_size*len(g)/len(indices))) for g in group_ids]
        while sum(sizes)>args.train_batch_size:sizes[int(np.argmax(sizes))]-=1
        while sum(sizes)<args.train_batch_size:sizes[int(np.argmax([len(g)/n for g,n in zip(group_ids,sizes)]))]+=1
        for j,(g,n) in enumerate(zip(group_ids,sizes)):
            for _ in range(n):
                if not queues[j]:queues[j]=list(rng.permutation(g))
                selected_ids.append(int(queues[j].pop()))
        return selected_ids
    with mp.get_context('spawn').Pool(args.workers,initializer=init_worker,initargs=(str(library),scenarios,repeat)) as pool:
        for generation in range(1,args.generations+1):
            if time.monotonic()>=deadline or (folder/'STOP').exists():break
            batch_indices=next_batch()
            candidates=search.ask();vectors=[]
            for x in candidates:
                w=weights+(mapping@x).astype(np.float32);vectors.append(w)
            returns=pool.map(worker_eval,[(w,batch_indices,0,str(folder/'failed_training_native'/f'generation{generation:04d}_candidate{j:02d}')) for j,w in enumerate(vectors)])
            if args.band_objective:
                for rows in returns:selection_screen(rows)
            costs=np.array([cost_function(rows) for rows in returns]);search.tell(costs)
            winner=int(costs.argmin());evaluations+=sum(len(x) for x in returns)
            if batch_mode or costs[winner]<best:
                best=float(costs[winner]);best_weights=vectors[winner].copy();best_gen=generation
                np.save(folder/'best_flat_weights.npy',best_weights)
            item={'generation':generation,'elapsed_s':time.monotonic()-start,'sigma':search.sigma,
                  'bestTrainingObjective':best,'generationBestObjective':float(costs.min()),
                  'generationMeanObjective':float(costs.mean()),'trainingBest':aggregate(returns[winner]),
                  'episodeEvaluations':evaluations,'trainingCases':batch_indices,'trainingObjectiveComparableAcrossGenerations':not batch_mode,'candidateResults':returns}
            history.append(item)
            np.savez_compressed(folder/f'generation_{generation:04d}.npz',candidates=candidates,objectives=costs,parameterMapping=mapping,baseFlatWeights=weights,mean=search.mean,C=search.C,pc=search.pc,ps=search.ps)
            with (folder/'generations.jsonl').open('a') as f:f.write(json.dumps(item)+'\n')
            print(f"GEN {generation} best={best:.8g} current={costs.min():.8g} sigma={search.sigma:.5g} peak={item['trainingBest']['worstNormalForceError_N']:.6g} failures={item['trainingBest']['physicalFailures']} seconds={item['elapsed_s']:.3f}",flush=True)
            if generation%args.selection_every==0 or generation==1:
                proposals=[('generation_best',best_weights)]
                if batch_mode:proposals.append(('distribution_mean',weights+(mapping@search.mean).astype(np.float32)))
                evaluated=[]
                for proposal_name,proposal_weights in proposals:
                    proposal_rows=selection_screen(evaluator.evaluate(proposal_weights,selection_ids))
                    pr=aggregate(proposal_rows);evaluations+=len(proposal_rows)
                    pr['jointScore']=joint_band_score(proposal_rows) if args.band_objective else None
                    rank=(pr['physicalFailures'],-pr['passed'],pr['jointScore'],pr['worstNormalForceError_N'],pr['meanRmsNormalForceError_N']) if args.band_objective else (pr['physicalFailures'],pr['recenterFailures'] if args.require_centering else 0,-pr['passed'],pr['worstNormalForceError_N'],pr['meanRmsNormalForceError_N'])
                    evaluated.append((rank,pr,proposal_name,proposal_weights))
                retained_proposals=[{'proposal':n,'rank':rr,'aggregate':{k:v for k,v in pr.items() if k!='rows'},'rows':pr['rows']} for rr,pr,n,_ in evaluated]
                rank,selected_report,proposal_name,selection_weights=min(evaluated,key=lambda x:x[0])
                selected_report['evaluatedProposals']=retained_proposals
                selected_report['proposal']=proposal_name
                selected_report.update(generation=generation,trainingBestGeneration=best_gen,rank=rank)
                selection_reports.append(selected_report)
                write_json(folder/'selection.json',selection_reports)
                if best_selection is None or rank<best_selection:
                    best_selection=rank;np.save(folder/'selected_flat_weights.npy',selection_weights)
                    checkpoint_dir=folder/'checkpoints'/f'generation_{generation:04d}'
                    export_flat(selection_weights,checkpoint_dir,{'optimizer':'reward_only_antithetic_CMA','generation':generation,'selectionRank':rank,'noTeacher':True})
                    write_json(folder/'latest_selected_checkpoint.json',{'directory':str(checkpoint_dir.resolve()),'generation':generation,'rank':rank,'checkpointSha256':sha(checkpoint_dir/'policy_checkpoint.pt')})
                print('SELECT '+json.dumps({k:v for k,v in selected_report.items() if k not in ('rows','evaluatedProposals')}),flush=True)
    # Reconstitute a portable checkpoint using the public exporter tensor order.
    chosen=np.load(folder/'selected_flat_weights.npy') if (folder/'selected_flat_weights.npy').exists() else best_weights
    export_flat(chosen,folder/'selected_policy',{'optimizer':'reward_only_antithetic_CMA','training_cases':indices,'selection_cases':selection_ids,'no_teacher':True})
    final_rows=selection_screen(evaluator.evaluate(chosen,selection_ids,record_dir=folder/'selected_histories' if args.record else None))
    full_training_rows=evaluator.evaluate(chosen,indices)
    if args.band_objective:
        for row in full_training_rows:row['recenterPassed']=True;row['recenterScreenApplied']=False
    full_training_audit=aggregate(full_training_rows)
    evaluations+=len(indices)
    write_json(folder/'full_training_final.json',full_training_audit)
    report={'status':'search_complete','seed':args.seed,'generations':len(history),'episodeEvaluations':evaluations+len(final_rows),
        'elapsed_s':time.monotonic()-start,'parameterCountOptimized':len(columns),'parameters':requested,'trainingBatchSize':args.train_batch_size,'trainingBatchProfile':args.batch_profile,'trainingCases':indices,'centeringWeight':args.centering_weight,'requireCentering':args.require_centering,'bandObjective':args.band_objective,'robotActionScale_m_s':args.robot_scale,'forceExcessWeight':args.force_excess_weight,'centeringSelectionDesignScreen':('No hard recentering screen; user accepted a soft±7.5mm band.' if args.band_objective else 'For12zero-pathnominalselectioncases with >=.5mminitialoffset: attain>=50%baselineinitial-to-finalabsolute-errorreduction. Stressnominalpath casesexcluded. Engineeringdesignscreen, notuserprescribedspec.'),
        'initialTraining':initial_report,'finalTraining':full_training_audit,'selectedSelection':aggregate(final_rows),'selection':selection_reports,
        'bestTrainingObjective':best,'bestTrainingGeneration':best_gen,'nativeLibrarySha256':sha(Path(args.native_build)/'libacf_native.so'),
        'sharedWeightsForEveryCase':True,'scope':'Finite development search. Fresh holdout required; no promotion.'}
    write_json(folder/'results.json',report)

if __name__=='__main__':main()
