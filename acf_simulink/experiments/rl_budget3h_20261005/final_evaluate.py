"""Lock concrete policy/plant bytes, then record a finite native evaluation.

This driver never trains or changes weights. Unseen holdout requires explicit
--run-locked-holdout plus a selection-only review. --fixture-train can execute
only training IDs, optionally with a short startup horizon, for acceptance.

Selection review JSON must bind selectedCheckpointSha256, evaluationBankSha256,
sourceModelSha256, selectionIDs and selectionOnly=true. Optional training-bank
and training-graph hashes make a transfer between parameter banks explicit.
Additional evidence files are frozen when listed as {file,sha256} entries in
selectionEvidence. Root authors the review after selection, before holdout.

Physical integration is exclusively libacf_native.so. The accepted C episode
adapter is copied from the supplied frozen policy source; baseline-mode NN
inference is bypassed. A TRAIN-only reset flag supports a memory ablation.
Every native sample is independently scored, including time0 and the final
sample. Aggregate mode releases ordinary histories after scoring and hashing;
exact replays preserve every failed case and each phase's force/travel worst.
ZIP level1 preserves full NumPy array precision at bounded compression cost.
"""
from __future__ import annotations

import argparse
import ctypes as ct
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import zipfile

import numpy as np

STAT_NAMES = [
    'peakNormalForceError_N','rmsNormalForceError_N','mseNormalForceError_N2',
    'finalTravelError_m','rmsTravelError_m','peakCurrent_A','peakVoltage_V',
    'voltageClipFraction','currentLimitFraction','contactLoss','travelStop',
    'softwareFailure','completed','nativeSamples','controllerDecisions','endTime_s',
    'issuedRobotFrames','eligibleRobotDecisions','peakIssuedRobot_m_s','initialForceError_N',
    'maxTravel_m','minTravel_m','forceOutsideFraction','nativeCommandTime_s','recordSamples',
]
SOURCE_NAMES = ('policy.py','policy_export.py','env.py','reward_search.py','reward_search_native.c')
_STOP = False
_WORKER = None


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda:handle.read(4*1024*1024),b''):
            digest.update(block)
    return digest.hexdigest()


def array_sha(array):
    """Bind dtype/shape and exact uncompressed bytes for deterministic replay."""
    array=np.ascontiguousarray(array)
    digest=hashlib.sha256(json.dumps([array.dtype.str,list(array.shape)]).encode())
    if array.size:digest.update(memoryview(array).cast('B'))
    return digest.hexdigest()


def atomic_json(path,value):
    path=Path(path);temporary=path.with_name(path.name+'.tmp')
    with temporary.open('w') as handle:
        json.dump(value,handle,indent=2,allow_nan=False,
                  default=lambda x:x.item() if isinstance(x,np.generic) else str(x))
        handle.write('\n');handle.flush();os.fsync(handle.fileno())
    os.replace(temporary,path)


def atomic_npz(path,arrays,compression_level=1):
    """NumPy-compatible ZIP, full fp64 native histories, atomic publication."""
    path=Path(path);temporary=path.with_name(path.name+'.tmp')
    with zipfile.ZipFile(temporary,'w',compression=zipfile.ZIP_DEFLATED,
                        compresslevel=compression_level,allowZip64=True) as archive:
        for key,array in arrays.items():
            with archive.open(key+'.npy','w',force_zip64=True) as handle:
                np.lib.format.write_array(handle,np.asarray(array),allow_pickle=False)
    with temporary.open('rb') as handle:os.fsync(handle.fileno())
    os.replace(temporary,path)


def parse_indices(text,scenarios,fixture=False,evaluation_split='holdout'):
    if text:
        indices=[int(x) for x in text.split(',')]
    else:
        indices=[i for i,row in enumerate(scenarios) if row['split']==('train' if fixture else evaluation_split)]
        if fixture:indices=indices[:1]
    if not indices or len(set(indices))!=len(indices):
        raise ValueError('case IDs must be nonempty and unique')
    if any(i<0 or i>=len(scenarios) for i in indices):raise ValueError('unknown case ID')
    expected='train' if fixture else evaluation_split
    if any(scenarios[i]['split']!=expected for i in indices):
        raise ValueError(f'this mode may execute only {expected} cases')
    return indices


def scenario_context(row):
    return np.asarray([
        row['force_N'],row['knownEquilibriumCurrent_A'],row['midTravel_m'],row['angle_rad'],
        row['identifiedMassS_kg'],row['identifiedMassB_kg'],row['massA_kg'],
        row['identifiedK_N_m'],row['identifiedC_N_s_m'],row['identifiedKf_N_A'],
        row['coilR_ohm'],row['coilL_H'],row['usableVoltage_V'],row['peakCurrent_A'],
        row['travelMax_m']-row['travelMin_m'],
        row.get('identifiedCurrentResponse_s',row.get('currentResponse_s',.0001)),
        row.get('identifiedRobotDelay_s',.08),
        row.get('identifiedRobotResponse_s',row.get('identifiedRobotResponseTime_s',.06)),
        row.get('identifiedAntiAliasCutoff_Hz',8000),row.get('identifiedEmaTime_s',.00005),
        row.get('identifiedPipelineDelay_s',.00002),row['duration_s'],
    ],np.float64)


def freeze_inputs(args,folder):
    source=Path(args.policy_source_dir).resolve();native=Path(args.native_build).resolve()
    checkpoint=Path(args.checkpoint).resolve()
    if checkpoint.is_dir():checkpoint=checkpoint/'policy_checkpoint.pt'
    bank=Path(args.bank).resolve()
    raw=json.loads(bank.read_text());metadata=json.loads((native/'metadata.json').read_text())
    overlay=json.loads((native/'scenarios.json').read_text());bridge=json.loads((native/'bridge_build.json').read_text())
    if len(raw['cases'])!=len(overlay):raise ValueError('bank and native scenario counts differ')
    if metadata['sourceModelSha256']!=raw['sourceModelSha256']:
        raise ValueError('physical source model differs between bank and native build')
    if (not math.isclose(metadata['actionStep_s'],.0001,abs_tol=1e-15) or
        not math.isclose(metadata['robotStep_s'],.01,abs_tol=1e-15) or
        not math.isclose(metadata['step_s']*metadata['physicalTicksPerAction'],metadata['actionStep_s'],abs_tol=1e-15)):
        raise ValueError('native physical/action/robot clocks differ from the accepted adapter')
    if metadata.get('scoringNames')!=['true_normal_N','plant_log_25','true_surface_slope']:
        raise ValueError('full-native scoring channel contract differs')
    if metadata.get('configsSha256')!=raw.get('configsSha256'):
        raise ValueError('prepared parameter-bank hash differs')
    if not bridge.get('native_computation_recovered_exactly'):
        raise ValueError('native computation recovery proof missing')
    seal={'libacf_native.so':'shared_library_sha256','native_parameter_bank.c':'parameter_bank_sha256',
          'acf_rl_native_context.c':'adapted_c_sha256'}
    for name,key in seal.items():
        if not (native/name).is_file() or sha(native/name)!=bridge[key]:
            raise ValueError(f'native seal mismatch: {name}')
    emitted=native/'code/acf_rl_native_grt_rtw/acf_rl_native.c'
    if emitted.exists() and sha(emitted)!=bridge['emitted_c_sha256']:
        raise ValueError('emitted native C hash differs from seal')
    scenarios=[]
    for i,(row,identified) in enumerate(zip(raw['cases'],overlay,strict=True)):
        if int(row['scenario'])!=i or int(identified['scenario'])!=i:
            raise ValueError('native scenario indices do not match bank')
        if identified.get('identity',row['identity'])!=row['identity']:
            raise ValueError('native scenario identity differs')
        merged={**row,**identified}
        merged['identifiedRobotResponse_s']=identified['identifiedRobotResponseTime_s']
        merged['identifiedCurrentResponse_s']=identified['currentResponse_s']
        if not np.isfinite(scenario_context(merged)).all():raise ValueError('nonfinite identified context')
        scenarios.append(merged)
    indices=parse_indices(args.indices,scenarios,args.fixture_train,args.evaluation_split)
    if not args.fixture_train and not args.run_locked_holdout:
        raise ValueError('holdout requires explicit --run-locked-holdout after selection')
    review=None
    if not args.fixture_train:
        if not args.selection_review:raise ValueError('holdout requires --selection-review')
        review=json.loads(Path(args.selection_review).read_text())
        expected={
            'selectedCheckpointSha256':sha(checkpoint),'evaluationBankSha256':sha(bank),
            'sourceModelSha256':metadata['sourceModelSha256'],'selectionOnly':True,
        }
        for key,value in expected.items():
            if review.get(key)!=value:raise ValueError(f'selection review mismatch: {key}')
        selection_scenarios=scenarios
        selection_bank=None
        if args.selection_bank:
            selection_bank=Path(args.selection_bank).resolve()
            selected_bank=json.loads(selection_bank.read_text())
            if selected_bank['sourceModelSha256']!=metadata['sourceModelSha256']:
                raise ValueError('reviewed selection bank uses a different physical graph')
            if review.get('selectionBankSha256')!=sha(selection_bank):
                raise ValueError('separate selection-bank review hash differs')
            selection_scenarios=selected_bank['cases']
        if args.evaluation_split=='qualification' and selection_bank is None:
            raise ValueError('fresh qualification grid requires --selection-bank and its review hash')
        selection=[i for i,row in enumerate(selection_scenarios) if row['split']=='selection']
        if not selection:raise ValueError('selection review must bind a nonempty selection set')
        if sorted(review.get('selectionIDs',[]))!=selection:
            raise ValueError('selection review must cover the complete new-bank selection set')
        if review.get('trainingSourceModelSha256',metadata['sourceModelSha256'])!=metadata['sourceModelSha256']:
            raise ValueError('training and evaluation physical source model differ')
    frozen=folder/'frozen_inputs';frozen.mkdir()
    policy_source=frozen/'policy_source';policy_source.mkdir()
    for name in SOURCE_NAMES:
        if not (source/name).is_file():raise FileNotFoundError(source/name)
    for path in sorted(source.iterdir()):
        if path.is_file() and path.suffix in ('.py','.c','.h','.json'):
            shutil.copy2(path,policy_source/path.name)
    shutil.copy2(Path(__file__),frozen/'final_evaluate.py')
    shutil.copy2(checkpoint,frozen/'selected_checkpoint.pt')
    initial=None
    if args.initial_checkpoint:
        initial=Path(args.initial_checkpoint).resolve()
        if initial.is_dir():initial=initial/'policy_checkpoint.pt'
        shutil.copy2(initial,frozen/'initial_checkpoint.pt')
    shutil.copy2(bank,frozen/'bank.json')
    if (bank.parent/'configs.mat').exists():shutil.copy2(bank.parent/'configs.mat',frozen/'configs.mat')
    native_frozen=frozen/'native';native_frozen.mkdir()
    for name in ('libacf_native.so','metadata.json','scenarios.json','bridge_build.json',
                 'native_parameter_bank.c','acf_rl_native_context.c','parameter_mapping.json',
                 'acf_rl_native.slx','native_bank_aux.h'):
        if (native/name).is_file():shutil.copy2(native/name,native_frozen/name)
    if (native/'code/acf_rl_native_grt_rtw').exists():
        destination=native_frozen/'generated_code';destination.mkdir()
        for path in sorted((native/'code/acf_rl_native_grt_rtw').iterdir()):
            if path.is_file() and path.suffix in ('.c','.h'):shutil.copy2(path,destination/path.name)
    if review is not None:
        shutil.copy2(args.selection_review,frozen/'selection_review.json')
        if args.selection_bank:shutil.copy2(args.selection_bank,frozen/'selection_bank.json')
        evidence=frozen/'selection_evidence';evidence.mkdir()
        for index,item in enumerate(review.get('selectionEvidence',[])):
            original=Path(item['file']).resolve()
            if sha(original)!=item['sha256']:raise ValueError('selection evidence hash differs')
            shutil.copy2(original,evidence/f'{index:03d}_{original.name}')
    atomic_json(frozen/'consumed_scenarios.json',scenarios)
    hashes={str(p.relative_to(frozen)):sha(p) for p in sorted(frozen.rglob('*')) if p.is_file()}
    contract={
        'checkpoint':str(checkpoint),'checkpointSha256':sha(checkpoint),
        'initialCheckpoint':str(initial) if initial else None,
        'initialCheckpointSha256':sha(initial) if initial else None,
        'policySourceDirectory':str(source),'nativeBuildDirectory':str(native),'bank':str(bank),
        'bankSha256':sha(bank),'sourceModelSha256':metadata['sourceModelSha256'],
        'nativeLibrarySha256':sha(native/'libacf_native.so'),
        'preparedConfigsSha256':metadata.get('configsSha256'),
        'selectionReviewSha256':sha(args.selection_review) if review is not None else None,
        'trainingBankSha256':review.get('trainingBankSha256') if review else None,
        'parameterBankTransfer':bool(review and review.get('trainingBankSha256') and
                                     review['trainingBankSha256']!=sha(bank)),
        'cases':[{'scenario':i,'identity':scenarios[i]['identity'],'seed':scenarios[i]['seed'],
                  'split':scenarios[i]['split']} for i in indices],
        'phases':['selected']+(['baseline'] if args.baseline else [])+(['initial'] if initial else []),
        'fixtureOnly':bool(args.fixture_train),'fixtureDuration_s':args.fixture_duration,
        'travelBandRadius_m':args.travel_band_radius_m,
        'evaluationSplit':args.evaluation_split,'aggregateOnly':args.aggregate_only,
        'workers':args.workers,'fixtureResetStateEveryStep':args.fixture_reset_state_every_step,
        'selectionBankSha256':review.get('selectionBankSha256') if review else None,
        'frozenFiles':hashes,
    }
    # Lock is published before imports, compilation, or physical evaluation.
    atomic_json(folder/'input_lock.json',{'status':'weights_and_inputs_locked_before_evaluation',
        'lockedAtUtc':datetime.now(timezone.utc).isoformat(),'contract':contract,
        'metric':'Full-native actual workpiece-normal force incl time0/final; no retuning.',
        'nativeStep_s':metadata['step_s'],'nativeActionStep_s':metadata['actionStep_s'],
        'robotStep_s':metadata['robotStep_s'],'maximumContextsInChunk':args.chunk_size})
    return frozen,scenarios,indices,metadata,contract


def import_frozen(frozen):
    directory=(frozen/'policy_source').resolve()
    sys.path.insert(0,str(directory))
    modules={name:importlib.import_module(name) for name in ('policy','policy_export','env','reward_search')}
    for name,module in modules.items():
        if Path(module.__file__).resolve()!=directory/(name+'.py'):
            raise RuntimeError(f'import escaped frozen policy source: {name}')
    return modules


def build_executor(frozen,policy_directory,output,reset_every_step=False):
    original=(frozen/'policy_source/reward_search_native.c').read_text()
    anchor='acf_unified_policy_step(w,feature,state,step==0,mean,&value);'
    if original.count(anchor)!=1:raise ValueError('accepted episode adapter policy-call anchor changed')
    # Baseline uses the native deterministic branch, no NN inference or actions.
    policy_call=anchor.replace('step==0','1') if reset_every_step else anchor
    transformed=original.replace(anchor,'if (baseline) { mean[0]=mean[1]=0; value=0; } else '+policy_call,1)
    output.mkdir();source=output/'final_episode.c';source.write_text(transformed)
    library=output/'final_episode.so';native=frozen/'native/libacf_native.so'
    command=['cc','-std=c99','-O3','-fPIC','-shared','-fno-fast-math','-ffp-contract=off',
             '-I'+str(policy_directory.resolve()),str(source),str(policy_directory/'policy_inference.c'),
             str(native.resolve()),'-Wl,-rpath,'+str(native.parent.resolve()),'-lm','-o',str(library)]
    result=subprocess.run(command,capture_output=True,text=True)
    (output/'compile.log').write_text(result.stdout+result.stderr);result.check_returncode()
    atomic_json(output/'compile_manifest.json',{'command':command,'episodeSourceSha256':sha(source),
        'acceptedEpisodeSourceSha256':sha(frozen/'policy_source/reward_search_native.c'),
        'policyCSourceSha256':sha(policy_directory/'policy_inference.c'),
        'nativeLibrarySha256':sha(native),'compiledExecutorSha256':sha(library),
        'onlyAdapterChanges':['Skip NN inference in native baseline mode.']+
           (['TRAIN fixture ablation: reset learned recurrent state on every actor decision.'] if reset_every_step else []),
        'physicalGraphUnchanged':True})
    return library


def verify_feature_parity(evaluator,module,scenarios,output):
    rng=np.random.default_rng(20261005);worst=0.0
    for i,row in enumerate(scenarios):
        observation=rng.normal(size=(1,18));step=int(rng.integers(0,100000))
        actual=np.empty(40,np.float32);context=evaluator.contexts[i]
        evaluator.feature_fn(observation.ctypes.data_as(ct.POINTER(ct.c_double)),
            context.ctypes.data_as(ct.POINTER(ct.c_double)),step,
            actual.ctypes.data_as(ct.POINTER(ct.c_float)))
        expected=module.feature_array(observation,[row],np.array([step]))[0]
        worst=max(worst,float(np.max(np.abs(actual-expected))))
    if worst!=0:raise ValueError(f'C/Python fixed causal feature mapping differs: {worst}')
    atomic_json(output/'feature_parity.json',{'status':'complete','syntheticOnly':True,
        'contextCount':len(scenarios),'maximumDifference':worst,
        'noPhysicalCasesExecuted':True})


def score_full(record,actions,stats,scenario,metadata,baseline=False,travel_band_radius_m=.0075):
    """Independent full-record metrics; diagnostic C interval stats retained."""
    native_dt=float(metadata['step_s']);data=record[:,4:31];time_axis=record[:,0]
    count=len(record);finite=all(np.isfinite(record[s:s+131072]).all() for s in range(0,count,131072))
    finite=finite and np.isfinite(actions).all() and np.isfinite(stats).all()
    row={name:(float(value) if np.isfinite(value) else None) for name,value in zip(STAT_NAMES,stats[:25],strict=True)}
    row['additionalEpisodeStatistics']=[float(x) if np.isfinite(x) else None for x in stats[25:]]
    if len(stats)==29:
        row['additionalEpisodeStatisticNames']=['intervalTravelBandExcessSquaredIntegral_m2_s',
            'intervalMaxAbsTravelError_m','intervalTimeOutsideBandUpper_s','initialActualTravelError_m']
    row.update(scenario=int(scenario['scenario']),identity=scenario['identity'],seed=int(scenario['seed']),
               force_N=float(scenario['force_N']),massS_kg=float(scenario['massS_kg']),
               angle_rad=float(scenario['angle_rad']),initialTravelError_m=float(scenario['offset_m']),
               baseline=bool(baseline),nativeSamples=count,fullNativeHistory=True,
               softwareRejected=bool(stats[11]>.5 or not finite or not count))
    if not count:
        row.update(passed=False,completed=False,physicalFailure=False,nativeGridComplete=False)
        return row
    if not np.isfinite(time_axis).all():
        row.update(passed=False,completed=False,softwareRejected=True,nativeGridComplete=False,
                   physicalFailure=bool(stats[9]>.5 or stats[10]>.5),
                   nativeFirstTime_s=None,nativeLastTime_s=None,endTime_s=None)
        return row
    ticks=np.rint(time_axis/native_dt).astype(np.int64)
    grid=bool(time_axis[0]==0 and len(ticks)==int(ticks[-1])+1 and
              np.max(np.abs(time_axis-ticks*native_dt))<max(1e-12,native_dt*1e-6) and
              np.all(np.diff(ticks)==1))
    row.update(nativeFirstTime_s=float(time_axis[0]),nativeLastTime_s=float(time_axis[-1]),
               nativeGridComplete=grid,endTime_s=float(time_axis[-1]))
    row['softwareRejected']|=not grid
    if not finite:
        row.update(passed=False,completed=False,physicalFailure=bool(stats[9] or stats[10]))
        return row
    error=data[:,0]-scenario['force_N'];current=np.abs(data[:,14]);voltage=np.abs(data[:,15])
    contact=bool(np.any(data[:,18]<.5) or stats[9]>.5)
    stop=bool(np.any(data[:,21]>.5) or stats[10]>.5)
    completed=bool(stats[12]>.5 and time_axis[-1]>=scenario['duration_s']-native_dt/2)
    row.update(
        peakNormalForceError_N=float(np.abs(error).max()),rmsNormalForceError_N=float(np.sqrt(np.mean(error**2))),
        mseNormalForceError_N2=float(np.mean(error**2)),initialForceError_N=float(abs(error[0])),
        finalForceError_N=float(error[-1]),forceOutsideSamples=int(np.sum(np.abs(error)>.5)),
        forceOutsideFraction=float(np.mean(np.abs(error)>.5)),peakCurrent_A=float(current.max()),
        rmsCurrent_A=float(np.sqrt(np.mean(current**2))),peakVoltage_V=float(voltage.max()),
        voltageClipSamples=int(np.sum(data[:,23]>.5)),currentLimitSamples=int(np.sum(data[:,22]>.5)),
        voltageClipFraction=float(np.mean(data[:,23]>.5)),currentLimitFraction=float(np.mean(data[:,22]>.5)),
        overCurrentSamples=int(np.sum(current>scenario['peakCurrent_A']*(1+1e-4))),
        overVoltageSamples=int(np.sum(voltage>scenario['usableVoltage_V']+1e-7)),
        minTravel_m=float(data[:,2].min()),maxTravel_m=float(data[:,2].max()),
        finalTravel_m=float(data[-1,2]),finalTravelError_m=float(data[-1,2]-scenario['midTravel_m']),
        rmsTravelError_m=float(np.sqrt(np.mean((data[:,2]-scenario['midTravel_m'])**2))),
        peakActualRobotSideVelocity_m_s=float(np.abs(data[:,8]).max()),
        contactLoss=contact,travelStop=stop,physicalFailure=contact or stop,completed=completed)
    if len(actions):
        gate=actions[:,19]>.5;clock=np.rint(actions[:,0]/metadata['actionStep_s']).astype(np.int64)
        eligible=gate&(clock%round(metadata['robotStep_s']/metadata['actionStep_s'])==0)
        requested=actions[:,2];issued=actions[:,18]*.005;total_issued=actions[:,17]*.005
        nonzero=np.abs(requested)>1e-12
        closed=~gate;closures=np.flatnonzero(gate[:-1]&~gate[1:])+1
        row.update(
            eligibleRobotDecisions=int(eligible.sum()),nonzeroRobotRequests=int(nonzero.sum()),
            requestGateOrClockViolations=int(np.sum(nonzero&~eligible)),
            peakRawRobotRequest_m_s=float(np.abs(requested).max()),
            peakIssuedRobot_m_s=float(np.abs(issued).max()),
            issuedRobotFramesAtActorClock=int(np.sum(np.abs(issued)>1e-12)),
            peakIssuedTotalRobotVelocity_m_s=float(np.abs(total_issued).max()),
            forceGateClosures=int(len(closures)),
            heldIssuedCorrectionWhileGateClosedFrames=int(np.sum(closed&(np.abs(issued)>1e-12))),
            issuedPredictionWhileGateClosedFrames=int(np.sum(closed&(np.abs(actions[:,12]*.005)>1e-12))),
            closureWithIssuedPredictionFrames=int(np.sum(np.abs(actions[closures,12]*.005)>1e-12)),
            robotCommandHistoryFrame='Before each actor update; native issued correction frames also counted by C at every100us.',
            rawRequestsApplied=not baseline)
    travel_error=data[:,2]-scenario['midTravel_m']
    band_excess=np.maximum(np.abs(travel_error)-travel_band_radius_m,0)
    row.update(travelBandRadius_m=float(travel_band_radius_m),
        travelWithinBandFraction=float(np.mean(band_excess<=1e-12)),
        travelOutsideBandSamples=int(np.sum(band_excess>1e-12)),
        travelOutsideBandTime_s=float(np.sum(band_excess[:-1]>1e-12)*native_dt),
        maxAbsTravelError_m=float(np.abs(travel_error).max()),
        maxTravelBandExcess_m=float(band_excess.max()),
        rmsTravelBandExcess_m=float(np.sqrt(np.mean(band_excess**2))),
        finalTravelBandExcess_m=float(band_excess[-1]),
        finalTravelInsideBand=bool(band_excess[-1]<=1e-12),
        travelBandPassed=bool(completed and np.max(band_excess)<=1e-12))
    # A stationary robot is acceptable when the whole travel history fits the
    # user's band. Center-seeking improvement is not a qualification target.
    row['recenterPassed']=row['travelBandPassed']
    row['forcePassed']=bool(completed and row['peakNormalForceError_N']<=.5 and not contact and not stop)
    row['passed']=bool(row['forcePassed'] and not row['softwareRejected'] and
                       row['overCurrentSamples']==0 and row['overVoltageSamples']==0 and
                       row.get('requestGateOrClockViolations',0)==0)
    row['designMarginPassed']=bool(row['passed'] and row['peakNormalForceError_N']<=.45)
    row['forceAndTravelBandPassed']=bool(row['passed'] and row['travelBandPassed'])
    return row


def check_executor_contract(evaluator):
    count=25
    if hasattr(evaluator.library,'acf_search_stats_count'):
        evaluator.library.acf_search_stats_count.restype=ct.c_int
        count=int(evaluator.library.acf_search_stats_count())
    if count not in (25,29):raise ValueError('unrecognized C episode-stat layout')
    if getattr(evaluator,'stats_count',25)!=count:
        raise ValueError('Python/C episode-stat layouts differ')
    required=24 if count==29 else 22
    if any(len(c)!=required for c in evaluator.contexts):
        raise ValueError('Python/C identified-context layouts differ')
    return count


def run_case(evaluator,weights,index,scenario,metadata,directory,baseline,compression,
             travel_band_radius_m=.0075,env_module=None,retain=True):
    directory.mkdir(parents=True,exist_ok=True)
    native_dt=float(metadata['step_s']);actor_dt=evaluator.repeat*float(metadata['actionStep_s'])
    capacity=round(scenario['duration_s']/native_dt)+64
    record=np.empty((capacity,31),np.float64)
    actions=np.empty((math.ceil(scenario['duration_s']/actor_dt)+1,45),np.float64)
    stats=np.zeros(check_executor_contract(evaluator),np.float64)
    context=evaluator.contexts[index]
    dp=ct.POINTER(ct.c_double);fp=ct.POINTER(ct.c_float)
    started=time.monotonic()
    status=evaluator.fn(index,int(scenario['seed']),context.ctypes.data_as(dp),weights.ctypes.data_as(fp),
        evaluator.repeat,int(baseline),stats.ctypes.data_as(dp),record.ctypes.data_as(dp),len(record),
        actions.ctypes.data_as(dp),len(actions))
    record=record[:max(0,min(len(record),int(stats[24])))]
    actions=actions[:max(0,min(len(actions),int(stats[14])))]
    native_path=directory/'native.npz';action_path=directory/'actions.npz'
    # Ordinary full-record mode preserves all bytes before failure scoring.
    if retain:
        atomic_npz(native_path,{'time':record[:,0],'inputs':record[:,1:4],'truth':record[:,4:31]},compression)
        atomic_npz(action_path,{'history':actions},compression)
    row=score_full(record,actions,stats,scenario,metadata,baseline,travel_band_radius_m)
    if env_module is not None and len(actions) and not baseline:
        normalized=np.tanh(actions[:,43:45].astype(np.float32))
        clock=np.rint(actions[:,0]/metadata['actionStep_s']).astype(np.int64)
        normalized[:,1]*=(actions[:,19]>.5)&(clock%100==0)
        expected=env_module.physical_actions(normalized,[scenario]*len(actions))
        residual=float(np.max(np.abs(expected-actions[:,1:3])))
        row['sourceActionMappingMaxResidual_SI']=residual
        if residual>1e-6:
            row.update(softwareRejected=True,passed=False,forceAndTravelBandPassed=False,
                       designMarginPassed=False,actionMappingMismatch=True)
    if status:row.update(softwareRejected=True,passed=False,forceAndTravelBandPassed=False,
                         designMarginPassed=False,executorStatus=int(status))
    # Even aggregate mode preserves failed bytes immediately; subsequent exact
    # replays retain all failures plus each phase's force/travel worst case.
    retained=retain or not row.get('passed',False) or row.get('softwareRejected',False)
    if retained and not retain:
        atomic_npz(native_path,{'time':record[:,0],'inputs':record[:,1:4],'truth':record[:,4:31]},compression)
        atomic_npz(action_path,{'history':actions},compression)
    row.update(rawNativeSha256=array_sha(record),rawActionsSha256=array_sha(actions),
               fullNativeHistoryScored=True,retainedHistory=retained,
               nativeHistoryFile=str(native_path.resolve()) if retained else None,
               nativeHistorySha256=sha(native_path) if retained else None,
               actionHistoryFile=str(action_path.resolve()) if retained else None,
               actionHistorySha256=sha(action_path) if retained else None,
               elapsed_s=time.monotonic()-started,
               nativeHistoryBytes=native_path.stat().st_size if retained else 0,
               actionHistoryBytes=action_path.stat().st_size if retained else 0)
    atomic_json(directory/'metrics.json',row)
    return row


def worker_initialize(frozen,library,scenarios,repeat,metadata,weights,compression,band):
    """Spawned processes load only locked sources and independent C contexts."""
    global _WORKER
    modules=import_frozen(Path(frozen))
    import torch
    torch.set_num_threads(1)
    evaluator=modules['reward_search'].EpisodeEvaluator(library,scenarios,repeat)
    check_executor_contract(evaluator)
    _WORKER=(evaluator,modules['env'],scenarios,metadata,weights,compression,band)


def worker_run(task):
    phase,index,directory,retain=task
    evaluator,env,scenarios,metadata,weights,compression,band=_WORKER
    return run_case(evaluator,weights['selected' if phase=='baseline' else phase],index,
        scenarios[index],metadata,Path(directory),phase=='baseline',compression,band,env,retain)


def replay_aggregates(report,evaluator,weights,scenarios,metadata,folder,args,env,deadline):
    replay_rows=[]
    for phase,outcome in report['outcomes'].items():
        rows=outcome['rows']
        selected={r['scenario'] for r in rows if not r.get('passed',False) or
                  not r.get('travelBandPassed',False) or r.get('softwareRejected',False)}
        finite=[r for r in rows if not r.get('softwareRejected')]
        if finite:
            selected.add(max(finite,key=lambda r:r['peakNormalForceError_N'])['scenario'])
            selected.add(max(finite,key=lambda r:r.get('maxTravelBandExcess_m',0))['scenario'])
        for original in rows:
            i=original['scenario']
            if i not in selected:continue
            if _STOP or (deadline is not None and time.time()>=deadline):
                report['aggregateReplays']=replay_rows
                return False
            replay=run_case(evaluator,weights['selected' if phase=='baseline' else phase],i,scenarios[i],
                metadata,folder/'retained_replays'/phase/f'case_{i:03d}',phase=='baseline',
                args.compression_level,args.travel_band_radius_m,env,True)
            matched=all(original[k]==replay[k] for k in ('rawNativeSha256','rawActionsSha256','nativeSamples'))
            replay.update(phase=phase,deterministicReplayMatched=matched,
                          replayExcludedFromUniqueCaseCount=True)
            replay_rows.append(replay)
            report['aggregateReplays']=replay_rows
            atomic_json(folder/'results.json',report)
            if not matched:raise AssertionError('locked deterministic replay differs from original native bytes')
            original.update(retainedHistory=True,retainedViaDeterministicReplay=True,
                nativeHistoryFile=replay['nativeHistoryFile'],nativeHistorySha256=replay['nativeHistorySha256'],
                actionHistoryFile=replay['actionHistoryFile'],actionHistorySha256=replay['actionHistorySha256'],
                nativeHistoryBytes=replay['nativeHistoryBytes'],actionHistoryBytes=replay['actionHistoryBytes'])
            atomic_json(folder/phase/f'case_{i:03d}'/'metrics.json',original)
    report['aggregateReplayNativeSamples']=sum(r['nativeSamples'] for r in replay_rows)
    report['aggregateReplaysComplete']=True
    return True


def aggregate(rows):
    finite=[r for r in rows if not r.get('softwareRejected')]
    return {'recordings':len(rows),'passed':sum(bool(r['passed']) for r in rows),
        'designMarginPassed':sum(bool(r.get('designMarginPassed')) for r in rows),
        'physicalFailures':sum(bool(r.get('physicalFailure')) for r in rows),
        'softwareRejected':sum(bool(r.get('softwareRejected')) for r in rows),
        'nativeSamples':sum(r['nativeSamples'] for r in rows),
        'worstNormalForceError_N':max((r['peakNormalForceError_N'] for r in finite),default=None),
        'meanRmsNormalForceError_N':float(np.mean([r['rmsNormalForceError_N'] for r in finite])) if finite else None,
        'recenterFailures':sum(not r.get('recenterPassed',False) for r in rows),
        'travelBandPassed':sum(bool(r.get('travelBandPassed')) for r in rows),
        'forceAndTravelBandPassed':sum(bool(r.get('forceAndTravelBandPassed')) for r in rows),
        'maximumTravelBandExcess_m':max((r.get('maxTravelBandExcess_m',0) for r in finite),default=None),
        'requestGateOrClockViolations':sum(r.get('requestGateOrClockViolations',0) for r in rows),
        'rows':rows}


def verify_lock(folder,contract):
    frozen=folder/'frozen_inputs'
    for name,digest in contract['frozenFiles'].items():
        if sha(frozen/name)!=digest:raise RuntimeError(f'locked input changed: {name}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('checkpoint','policy-source-dir','native-build','bank','output'):
        parser.add_argument('--'+name,required=True)
    parser.add_argument('--baseline',action='store_true')
    parser.add_argument('--initial-checkpoint');parser.add_argument('--selection-review')
    parser.add_argument('--selection-bank',help='Reviewed selection manifest when evaluating a separate fresh grid')
    parser.add_argument('--evaluation-split',choices=('holdout','qualification'),default='holdout')
    parser.add_argument('--indices');parser.add_argument('--run-locked-holdout',action='store_true')
    parser.add_argument('--fixture-train',action='store_true');parser.add_argument('--fixture-duration',type=float)
    parser.add_argument('--fixture-reset-state-every-step',action='store_true',
                        help='TRAIN-only memory ablation; sensors/observer/native plant unchanged')
    parser.add_argument('--aggregate-only',action='store_true',
                        help='Score every native sample in RAM; retain worst/failures through exact replays')
    parser.add_argument('--workers',type=int,default=1,help='Independent spawned C contexts, maximum4')
    parser.add_argument('--travel-band-radius-m',type=float,default=.0075)
    parser.add_argument('--chunk-size',type=int,default=8);parser.add_argument('--compression-level',type=int,default=1)
    parser.add_argument('--deadline-utc');parser.add_argument('--deadline-reserve-s',type=float,default=10)
    args=parser.parse_args()
    if not 1<=args.chunk_size<=8:raise ValueError('chunk size must be1..8')
    if not 1<=args.workers<=4:raise ValueError('workers must be1..4')
    if not 0<=args.compression_level<=9:raise ValueError('compression level must be0..9')
    if args.fixture_duration is not None and (not args.fixture_train or args.fixture_duration<=0):
        raise ValueError('a shortened horizon is permitted only for positive-duration training fixtures')
    if args.fixture_train and args.run_locked_holdout:raise ValueError('choose fixture or locked holdout mode')
    if args.fixture_reset_state_every_step and not args.fixture_train:
        raise ValueError('state-reset ablation is permitted only on TRAIN fixtures')
    if args.travel_band_radius_m<=0:raise ValueError('travel band radius must be positive')
    folder=Path(args.output).resolve();folder.mkdir(parents=True,exist_ok=False)
    started=time.monotonic();report={'status':'preparing','args':vars(args),'outcomes':{},'promoted':False}
    atomic_json(folder/'results.json',report)
    global _STOP
    def stop_signal(signum,frame):
        global _STOP
        _STOP=True
    signal.signal(signal.SIGINT,stop_signal);signal.signal(signal.SIGTERM,stop_signal)
    try:
        frozen,scenarios,indices,metadata,contract=freeze_inputs(args,folder)
        modules=import_frozen(frozen);export=modules['policy_export'];features=modules['env'].FEATURE_NAMES
        import torch
        torch.set_num_threads(1)
        policies={};weights={};saved_checkpoints={}
        for label,file in [('selected','selected_checkpoint.pt')]+([('initial','initial_checkpoint.pt')]
                if args.initial_checkpoint else []):
            saved=torch.load(frozen/file,map_location='cpu',weights_only=False)
            saved_checkpoints[label]=saved
            if saved.get('features')!=features:raise ValueError('checkpoint feature names differ from frozen env')
            policy=export.load_policy(frozen/file,'cpu')
            if json.dumps(saved['config'],sort_keys=True)!=json.dumps(policy.config(),sort_keys=True):
                raise ValueError('checkpoint-declared architecture differs from constructed policy')
            if policy.obs_dim!=len(features) or policy.obs_dim!=40:raise ValueError('unsupported observation contract')
            repeat=round(policy.dt/metadata['actionStep_s'])
            if repeat<1 or abs(repeat*metadata['actionStep_s']-policy.dt)>1e-12 or 100%repeat:
                raise ValueError('actor clock does not divide retained native robot clock')
            if not all(torch.isfinite(p).all() for p in policy.parameters()):raise ValueError('nonfinite weights')
            if not all(torch.isfinite(p).all() for p in policy.buffers()):raise ValueError('nonfinite normalization buffers')
            if hasattr(policy,'obs_scale') and not torch.all(policy.obs_scale>0):raise ValueError('nonpositive normalization scale')
            if policies and policy.config()!=policies['selected'].config():
                # std may have learned, but inference architecture/clock must match.
                a,b=policy.config().copy(),policies['selected'].config().copy()
                a.pop('initial_log_std',None);b.pop('initial_log_std',None)
                if a!=b:raise ValueError('initial and selected inference architectures differ')
            policies[label]=policy;weights[label]=np.ascontiguousarray(export.flat_weights(policy),np.float32)
            export.export_policy(policy,folder/(label+'_policy'),features,
                {'phase':'locked_final_evaluation','checkpointSha256':sha(frozen/file),'unqualifiedAtExport':True})
        repeat=round(policies['selected'].dt/metadata['actionStep_s'])
        provenance=saved_checkpoints['selected'].get('provenance',{})
        scales=provenance.get('physicalActionScales')
        if not scales:raise ValueError('checkpoint must declare its physicalActionScales in provenance')
        if float(scales.get('currentOffset_A',0))!=2:
            raise ValueError('accepted episode adapter supports exactly2A full-current offset scale')
        if float(scales.get('robotRequest_m_s',0)) not in (.005,.01):
            raise ValueError('checkpoint robot action scale must be5 or10mm/s')
        if (float(modules['env'].CURRENT_SCALE_A)!=scales['currentOffset_A'] or
            float(modules['env'].ROBOT_SCALE_M_S)!=scales['robotRequest_m_s']):
            raise ValueError('checkpoint and supplied frozen env physical action scales differ')
        for row in scenarios:
            row['robotActionScale_m_s']=float(scales['robotRequest_m_s'])
            row['acceptedRecenterBand_m']=args.travel_band_radius_m
        readout=policies['selected'].actor.weight.detach().cpu().numpy()
        actor_contract={'checkpointSha256':contract['checkpointSha256'],'physicalActionScales':scales,
            'externalFeatureCount':len(features),'architecture':policies['selected'].config(),
            'storedTrainableParameters':sum(p.numel() for p in policies['selected'].parameters()),
            'recurrentActionReadout':{name:{'nonzeroParameters':int(np.count_nonzero(readout[i])),
                    'l2Norm':float(np.linalg.norm(readout[i])),'maxAbs':float(np.abs(readout[i]).max())}
                    for i,name in enumerate(('current','robot'))},
            'checkpointProvenance':provenance,'fixtureResetStateEveryStep':args.fixture_reset_state_every_step,
            'sameActionMappingForAllComparisonActors':True}
        atomic_json(folder/'actor_contract.json',actor_contract)
        report['actorContract']=actor_contract
        contract['physicalActionScales']=scales
        lock=json.loads((folder/'input_lock.json').read_text())
        lock['contract']=contract;lock['actorContractSha256']=sha(folder/'actor_contract.json')
        lock['actorArchitectureResolvedBeforeEvaluation']=True
        atomic_json(folder/'input_lock.json',lock)
        for i in indices:
            if args.fixture_duration is not None:scenarios[i]={**scenarios[i],'duration_s':args.fixture_duration}
            duration=scenarios[i]['duration_s']
            if args.travel_band_radius_m>min(scenarios[i]['midTravel_m']-scenarios[i]['travelMin_m'],
                                             scenarios[i]['travelMax_m']-scenarios[i]['midTravel_m']):
                raise ValueError('accepted travel band extends beyond physical stops')
            if abs(round(duration/policies['selected'].dt)*policies['selected'].dt-duration)>1e-12:
                raise ValueError('episode duration must align with the actor clock')
        library=build_executor(frozen,folder/'selected_policy',folder/'executor',args.fixture_reset_state_every_step)
        evaluator=modules['reward_search'].EpisodeEvaluator(library,scenarios,repeat)
        check_executor_contract(evaluator)
        verify_feature_parity(evaluator,modules['env'],scenarios,folder)
        phases=(['baseline'] if args.baseline else [])+(['initial'] if args.initial_checkpoint else [])+['selected']
        report.update(status='evaluating_locked_inputs',inputLockSha256=sha(folder/'input_lock.json'),
                      contract=contract,architecture=policies['selected'].config(),featureNames=features,
                      sourceHashes={n:sha(frozen/'policy_source'/n) for n in SOURCE_NAMES},
                      requestedRecordings=len(indices)*len(phases),cases=len(indices),fixtureOnly=args.fixture_train,
                      noRetuning=True,parameterBankTransfer=contract['parameterBankTransfer'])
        deadline=None
        if args.deadline_utc:
            parsed=datetime.fromisoformat(args.deadline_utc.replace('Z','+00:00'))
            if parsed.tzinfo is None:parsed=parsed.replace(tzinfo=timezone.utc)
            deadline=parsed.timestamp()-args.deadline_reserve_s
        executor=None
        try:
            if args.workers>1:
                executor=ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('spawn'),
                    initializer=worker_initialize,initargs=(str(frozen),str(library),scenarios,repeat,metadata,
                        weights,args.compression_level,args.travel_band_radius_m))
            for phase in phases:
                rows=[];report['outcomes'][phase]=aggregate(rows)
                for chunk_start in range(0,len(indices),args.chunk_size):
                    if _STOP or (deadline is not None and time.time()>=deadline):break
                    chunk_rows=[];chunk=indices[chunk_start:chunk_start+args.chunk_size]
                    tasks=[(phase,i,str(folder/phase/f'case_{i:03d}'),not args.aggregate_only) for i in chunk]
                    produced=executor.map(worker_run,tasks) if executor else (
                        run_case(evaluator,weights['selected' if phase=='baseline' else phase],i,scenarios[i],metadata,
                            folder/phase/f'case_{i:03d}',phase=='baseline',args.compression_level,
                            args.travel_band_radius_m,modules['env'],not args.aggregate_only) for i in chunk)
                    for i,row in zip(chunk,produced,strict=True):
                        rows.append(row);chunk_rows.append(row)
                        report['outcomes'][phase]=aggregate(rows)
                        report['elapsed_s']=time.monotonic()-started
                        atomic_json(folder/'results.json',report)
                        atomic_json(folder/'progress.json',{'status':report['status'],'phase':phase,'completedCases':len(rows),
                            'completedRecordings':sum(x['recordings'] for x in report['outcomes'].values()),
                            'requestedRecordings':report['requestedRecordings'],'lastIdentity':row['identity'],
                            'lastPassed':row['passed'],'elapsed_s':report['elapsed_s']})
                        print('FINAL_ROW '+json.dumps({'phase':phase,'scenario':i,'identity':row['identity'],
                              'passed':row['passed'],'peak_N':row.get('peakNormalForceError_N'),
                              'nativeSamples':row['nativeSamples'],'elapsed_s':row['elapsed_s']}),flush=True)
                    atomic_json(folder/f'{phase}_chunk_{chunk_start//args.chunk_size:03d}.json',{'rows':chunk_rows})
                if _STOP or (deadline is not None and time.time()>=deadline):break
        finally:
            if executor:executor.shutdown(wait=True,cancel_futures=True)
        complete=sum(x['recordings'] for x in report['outcomes'].values())==report['requestedRecordings']
        if complete and args.aggregate_only:
            complete=replay_aggregates(report,evaluator,weights,scenarios,metadata,folder,args,modules['env'],deadline)
        report['status']='fixture_complete' if complete and args.fixture_train else ('complete' if complete else 'incomplete_stopped')
        report['completedRecordings']=sum(x['recordings'] for x in report['outcomes'].values())
        report['totalNativeSamples']=sum(x['nativeSamples'] for x in report['outcomes'].values())
        report['elapsed_s']=time.monotonic()-started
        if complete and args.baseline:
            paired=[]
            for baseline,selected in zip(report['outcomes']['baseline']['rows'],report['outcomes']['selected']['rows'],strict=True):
                if baseline['identity']!=selected['identity'] or baseline['seed']!=selected['seed']:
                    raise AssertionError('paired scenario or seed differs')
                paired.append({'identity':selected['identity'],'seed':selected['seed'],
                    'baselinePassed':baseline['passed'],'selectedPassed':selected['passed'],
                    'baselinePeak_N':baseline.get('peakNormalForceError_N'),'selectedPeak_N':selected.get('peakNormalForceError_N'),
                    'selectedMinusBaseline_N':selected['peakNormalForceError_N']-baseline['peakNormalForceError_N']
                      if not selected['softwareRejected'] and not baseline['softwareRejected'] else None})
            report['paired']=paired
        verify_lock(folder,contract)
        report['lockedInputsUnchanged']=True
        report['metricContract']={'force':'true_normal_N minus target, every physical major sample inclstartup/end',
            'pass':'Completed;±0.5N; no contactloss/travelstop/software rejection/overcurrent/overvoltage/request-gate violations.',
            'travelBand':'Separate full-native±7.5mm(default) acceptance aroundmidstroke; no penalty or mandatoryrobotmotion insideband.',
            'scope':'Finite native generated-Simulink episodes. Physical prototype and entire tolerance boxes unqualified.',
            'robot':'Raw requests and carried issued-command features recorded atactorclock; C counts issuedmotion every100us.'}
        atomic_json(folder/'results.json',report)
        manifest={str(p.relative_to(folder)):sha(p) for p in sorted(folder.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json'}
        atomic_json(folder/'artifact_manifest.json',{'status':report['status'],'files':manifest})
        print('FINAL_COMPLETE '+json.dumps({'status':report['status'],'recordings':report['completedRecordings'],
              'nativeSamples':report['totalNativeSamples'],'elapsed_s':report['elapsed_s']}),flush=True)
        return 0 if complete else 2
    except BaseException as error:
        report.update(status='software_rejected',error={'type':type(error).__name__,'message':str(error)},
                      elapsed_s=time.monotonic()-started)
        atomic_json(folder/'results.json',report)
        raise


if __name__=='__main__':sys.exit(main())
