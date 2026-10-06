from pathlib import Path
import sys,json,hashlib,shutil,time,ctypes as ct
import numpy as np
ROOT=Path('/home/tommie/Projects/actuator-model');D=ROOT/'acf_simulink/validation/rl_budget3h_20261005';I=D/'native_actuator_probe_inputs';B=D/'native_actuator_probe';O=D/'native_actuator_probe_results';O.mkdir(exist_ok=False)
sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'));from native_api import build_library,NativePlant
build_library(B);plant=NativePlant(B)
sys.path.insert(0,str(I));from reward_search import EpisodeEvaluator,build_evaluator,write_json
scenarios=plant.scenarios
for r in scenarios:r['robotActionScale_m_s']=.01;r['acceptedRecenterBand_m']=.0075
write_json(O/'consumed_scenarios.json',scenarios)
lib=build_evaluator(B,I/'policy',O/'evaluator');eval=EpisodeEvaluator(lib,scenarios,repeat=1);weights=np.fromfile(I/'policy/policy_weights.f32',dtype=np.float32)
allrows=[];start=time.perf_counter()
for mode in ['baseline','learned']:
 rows=eval.evaluate(weights,list(range(6)),record_dir=O/mode,baseline=mode=='baseline')
 for r,s in zip(rows,scenarios):r['mode']=mode;r['usableVoltage_V']=s['usableVoltage_V'];r['currentResponse_s']=s['currentResponse_s']
 allrows.extend(rows);write_json(O/f'{mode}_rows.json',{'rows':rows});print(mode,'finished',time.perf_counter()-start,flush=True)
# Structural proof excludes every input path from V/tau to the actor/trunk.
a=np.load(I/'policy/policy_weights.npz');assert np.count_nonzero(a['encoder_weight'][:,[29,33]])==0 and np.count_nonzero(a['actor_skip_weight'][:,[29,33]])==0
# Auxiliary features in frozenC use only indices0,1,3,4,6,7,10,19,20,24,25,26.
source=(I/'policy/policy_inference.c').read_text();assert 'observation[29]' not in source and 'observation[33]' not in source
# Replay every20k causal feature frame, varying only staticV/tau inputs.
history=np.load(O/'learned'/f"{scenarios[0]['identity']}_actions.npz")['history'];features=np.ascontiguousarray(history[:,3:43],np.float32)
fn=eval.library.acf_unified_policy_step;FP=ct.POINTER(ct.c_float);fn.argtypes=[FP,FP,FP,ct.c_int,FP,FP]
def infer_sequence(voltage,tau):
 state=np.zeros(64,np.float32);mean=np.empty(2,np.float32);value=np.empty(1,np.float32);out=np.empty((len(features),67),np.float32)
 for k,feat in enumerate(features):
  feat=feat.copy();feat[29]=voltage/32;feat[33]=tau/.0001
  fn(weights.ctypes.data_as(FP),feat.ctypes.data_as(FP),state.ctypes.data_as(FP),int(k==0),mean.ctypes.data_as(FP),value.ctypes.data_as(FP));out[k,:2]=mean;out[k,2]=value[0];out[k,3:]=state
 return out
reference=infer_sequence(32,.0001);comparisons=[]
for voltage,tau in [(36,.0001),(48,.0001),(32,.00005),(36,.00005),(48,.00005)]:
 out=infer_sequence(voltage,tau);assert np.array_equal(out,reference);comparisons.append({'voltage_V':voltage,'tau_s':tau,'all_actor_critic_state_values_bit_identical':True,'max_difference':0.})
write_json(O/'context_independence.json',{'status':'zero_direct_context_dependence_proved','frames':len(features),'compared_outputs':'two latentmeans,recurrentcriticvalue,all64hiddenstates perframe','voltage_and_currenttau_encoder_skip_columns_zero':True,'auxiliary_has_noV_orTaupath':True,'counterfactuals':comparisons,'interpretation':'ChangingstaticV/tau cannot directlyalterpolicyaction for fixedcausalhistory.50us alsochanges matchingidentifiedcommand/currentresponseobserver preparation; thatis explicittuningchange, notvoltage-onlyevidence.'})
# Fullnative extrema/current/voltage/cliptiming and pointwise waveform contrasts.
for row in allrows:
 data=np.load(row['nativeHistoryFile']);t=data['time'];truth=data['truth'];error=truth[:,0]-row['force_N'];peak=int(np.argmax(abs(error)));clips=np.flatnonzero(truth[:,23]>.5);limits=np.flatnonzero(truth[:,22]>.5)
 row.update(peak_force_time_s=float(t[peak]),peak_force_signed_error_N=float(error[peak]),voltage_clip_samples=len(clips),current_limit_samples=len(limits),first_voltage_clip_time_s=float(t[clips[0]]) if len(clips) else None,last_voltage_clip_time_s=float(t[clips[-1]]) if len(clips) else None,full_native_peak_current_A=float(np.max(abs(truth[:,14]))),full_native_peak_voltage_V=float(np.max(abs(truth[:,15]))))
contrasts=[]
for mode in ['baseline','learned']:
 subset=[r for r in allrows if r['mode']==mode]
 for anchor in [0,3]:
  a0=np.load(subset[anchor]['nativeHistoryFile'])
  for j in [anchor+1,anchor+2]:
   b0=np.load(subset[j]['nativeHistoryFile']);assert np.array_equal(a0['time'],b0['time'])
   delta=b0['truth']-a0['truth'];contrasts.append({'mode':mode,'tau_s':subset[anchor]['currentResponse_s'],'from_V':32,'to_V':subset[j]['usableVoltage_V'],'normal_waveform_max_difference_N':float(np.max(abs(delta[:,0]))),'current_waveform_max_difference_A':float(np.max(abs(delta[:,14]))),'voltage_waveform_max_difference_V':float(np.max(abs(delta[:,15]))),'all_truth_bit_identical':bool(np.array_equal(a0['truth'],b0['truth']))})
summary={'status':'development_probe_complete','source_scenario':61,'source_checkpoint_sha256':hashlib.sha256((I/'policy/policy_checkpoint.pt').read_bytes()).hexdigest(),'cases':12,'native_samples':sum(r['nativeSamples'] for r in allrows),'wall_s':time.perf_counter()-start,'rows':allrows,'same_tau_voltage_contrasts':contrasts,'context_independence':'context_independence.json','qualification_claim':False,'holdout_trajectories_executed':0,'tuning_contract':'100usvoltagepairschangeonlyusableV;50usgroupchangesphysicalcurrenttargetandmatchingidentifiedmodelcoefficients. Actorweights/scales/seed/profile/calibrationfixed.'}
write_json(O/'summary.json',summary);print(json.dumps(summary,indent=2),flush=True)
