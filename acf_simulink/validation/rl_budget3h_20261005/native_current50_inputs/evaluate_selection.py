from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import sys,json,hashlib,time,shutil
import numpy as np
ROOT=Path('/home/tommie/Projects/actuator-model');D=ROOT/'acf_simulink/validation/rl_budget3h_20261005';I=D/'native_current50_inputs';O=D/'native_current50_results'
O.mkdir(exist_ok=False)
sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from native_api import NativePlant,build_library
sys.path.insert(0,str(I));from reward_search import EpisodeEvaluator,build_evaluator,write_json
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
checkpoint='48618b8d21c4f5920d870a05f5eb198c958632f51ffab6f0abfd6a8255ef0ad4'
assert sha(I/'policy/policy_checkpoint.pt')==checkpoint
indices=list(range(48,64))+list(range(120,128))+list(range(140,144));assert len(indices)==28 and not set(indices)&set(range(64,96))
weights=np.fromfile(I/'policy/policy_weights.f32',dtype=np.float32)
build_library(D/'native_bank_current50')
plants={100:NativePlant(D/'native_bank_sustained'),50:NativePlant(D/'native_bank_current50')}
evaluators={}
for tau,plant in plants.items():
 scenarios=plant.scenarios
 assert len(scenarios)==144
 for r in scenarios:r['robotActionScale_m_s']=.01;r['acceptedRecenterBand_m']=.0075
 write_json(O/f'consumed_scenarios_{tau}us.json',scenarios)
 lib=build_evaluator(plant.folder,I/'policy',O/f'evaluator_{tau}us')
 evaluators[tau]=EpisodeEvaluator(lib,scenarios,repeat=1)
 assert all(abs(scenarios[k]['currentResponse_s']-tau*1e-6)<1e-15 and scenarios[k]['usableVoltage_V']==32 for k in indices)
# Direct V/tau dependence has already been proved on all20,000frames in the
# actuator probe. The frozen exact same exported weights/source are consumed.
assert sha(I/'policy/policy_weights.f32')==sha(D/'native_actuator_probe_inputs/policy/policy_weights.f32')
def run(task):
 tau,mode,k=task
 row=evaluators[tau].evaluate(weights,[k],record_dir=O/f'{tau}us'/mode,baseline=mode=='baseline')[0]
 row['mode']=mode;row['currentResponse_us']=tau;row['usableVoltage_V']=32
 data=np.load(row['nativeHistoryFile']);t=data['time'];truth=data['truth'];error=truth[:,0]-row['force_N'];peak=int(np.argmax(abs(error)))
 row.update(peakForceTime_s=float(t[peak]),signedPeakForceError_N=float(error[peak]),
    voltageClipSamples=int(np.count_nonzero(truth[:,23]>.5)),currentLimitSamples=int(np.count_nonzero(truth[:,22]>.5)),
    fullNativePeakCurrent_A=float(np.max(abs(truth[:,14]))),fullNativePeakVoltage_V=float(np.max(abs(truth[:,15]))),
    startupIncluded=bool(t[0]==0),lastNativeTime_s=float(t[-1]),
    finalBandExcess_m=max(abs(row['finalTravelError_m'])-.0075,0))
 return row
start=time.perf_counter();rows=[]
tasks=[(tau,mode,k) for tau in [100,50] for mode in ['baseline','learned'] for k in indices]
with ThreadPoolExecutor(max_workers=4) as pool:
 futures={pool.submit(run,t):t for t in tasks}
 for f in as_completed(futures):
  row=f.result();rows.append(row)
  write_json(O/'progress.json',{'completed':len(rows),'planned':len(tasks),'wall_s':time.perf_counter()-start,'rows':rows})
  print('DONE',len(rows),row['currentResponse_us'],row['mode'],row['scenario'],row['peakNormalForceError_N'],flush=True)
rows.sort(key=lambda r:(-r['currentResponse_us'],r['mode'],r['scenario']))
pairs=[]
for mode in ['baseline','learned']:
 for k in indices:
  a=next(r for r in rows if r['currentResponse_us']==100 and r['mode']==mode and r['scenario']==k)
  b=next(r for r in rows if r['currentResponse_us']==50 and r['mode']==mode and r['scenario']==k)
  pairs.append({'mode':mode,'scenario':k,'identity':a['identity'],'force_N':a['force_N'],'massS_kg':a['massS_kg'],
    'peak100_N':a['peakNormalForceError_N'],'peak50_N':b['peakNormalForceError_N'],
    'peakChange_N':b['peakNormalForceError_N']-a['peakNormalForceError_N'],
    'rms100_N':a['rmsNormalForceError_N'],'rms50_N':b['rmsNormalForceError_N'],
    'rmsChange_N':b['rmsNormalForceError_N']-a['rmsNormalForceError_N'],
    'voltageClipSamples100':a['voltageClipSamples'],'voltageClipSamples50':b['voltageClipSamples'],
    'finalBandExcess100_m':a['finalBandExcess_m'],'finalBandExcess50_m':b['finalBandExcess_m'],
    'passed100':a['passed'],'passed50':b['passed']})
groups=[]
for tau in [100,50]:
 for mode in ['baseline','learned']:
  rr=[r for r in rows if r['currentResponse_us']==tau and r['mode']==mode]
  groups.append({'mode':mode,'currentResponse_us':tau,'recordings':len(rr),'passed':sum(r['passed'] for r in rr),
   'physicalFailures':sum(r['physicalFailure'] for r in rr),'nativeSamples':sum(r['nativeSamples'] for r in rr),
   'worstNormalForceError_N':max(r['peakNormalForceError_N'] for r in rr),'meanRmsNormalForceError_N':float(np.mean([r['rmsNormalForceError_N'] for r in rr])),
   'voltageClipSamples':sum(r['voltageClipSamples'] for r in rr),'currentLimitSamples':sum(r['currentLimitSamples'] for r in rr),
   'worstFinalBandExcess_m':max(r['finalBandExcess_m'] for r in rr),
   'peakCurrent_A':max(r['fullNativePeakCurrent_A'] for r in rr),'peakVoltage_V':max(r['fullNativePeakVoltage_V'] for r in rr)})
summary={'status':'fixed_checkpoint_driver_response_development_complete','checkpointSha256':checkpoint,
 'planned':112,'recordings':len(rows),'selectionScenarios':indices,'holdoutTrajectoriesExecuted':0,'trainingUpdates':0,
 'usableVoltage_V':32,'wall_s':time.perf_counter()-start,'nativeSamples':sum(r['nativeSamples'] for r in rows),
 'contract':'Paired full duration/allnative histories includingstartup/end. Samegen5weights,scenarioseeds/profile/mechanics/calibration/robotprogram.50uschangesdriverresponseandmatchingidentifiedobservercoefficients;100usoriginalfrozenbankunchanged. No heldoutselection orpromotion.',
 'groups':groups,'pairs':pairs,'rows':rows,'source100ConfigsSha256':plants[100].metadata['configsSha256'],
 'counterfactual50ConfigsSha256':plants[50].metadata['configsSha256'],
 'source100LibrarySha256':sha(plants[100].folder/'libacf_native.so'),'counterfactual50LibrarySha256':sha(plants[50].folder/'libacf_native.so')}
write_json(O/'summary.json',summary);print(json.dumps({'groups':groups,'wall_s':summary['wall_s'],'nativeSamples':summary['nativeSamples']}),flush=True)
