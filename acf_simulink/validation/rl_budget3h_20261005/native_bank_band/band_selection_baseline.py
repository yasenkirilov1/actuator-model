from pathlib import Path
import sys,json,time,numpy as np
ROOT=Path('/home/tommie/Projects/actuator-model');sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from native_api import NativePlant
F=ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_bank_band';p=NativePlant(F,threads=2);ids=list(range(120,128));cs=[p.create(i,baseline=True,record=i==121) for i in ids]
rows=[];start=time.perf_counter()
for i,c in zip(ids,cs):
 s=p.scenarios[i];r={'scenario':i,'identity':s['identity'],'duration_s':s['duration_s'],'initialOffset_m':s['offset_m'],'knownNominalRobotVelocity_m_s':s['knownNominalRobotVelocity_m_s'],'initial_true_travel_m':float(c.result.truth_last[2]),'peak_normal_error_N':float(c.result.peak_normal_error_N),'peak_band_excess_m':max(0,abs(float(c.result.truth_last[2])-s['midTravel_m'])-.0075),'robot_nonzero_frames':0,'gate_ready_frames':0,'terminated':False,'native_samples':0,'peak_current_A':0.,'peak_voltage_V':0.,'min_travel_m':float(c.result.truth_last[2]),'max_travel_m':float(c.result.truth_last[2]),'band_excess_time_upper_bound_s':0.};rows.append(r)
for tick in range(600):
 active=[j for j,r in enumerate(rows) if tick*.01<r['duration_s']-1e-12 and not r['terminated']]
 if not active:break
 obs,truth,cmp,m=p.step_repeat_batch_arrays([cs[j] for j in active],np.zeros((len(active),2)),100)
 for k,j in enumerate(active):
  r=rows[j];s=p.scenarios[ids[j]];r['peak_normal_error_N']=max(r['peak_normal_error_N'],float(m[k,6]));r['peak_band_excess_m']=max(r['peak_band_excess_m'],max(0,abs(m[k,8]-s['midTravel_m'])-.0075,abs(m[k,9]-s['midTravel_m'])-.0075));r['min_travel_m']=min(r['min_travel_m'],float(m[k,8]));r['max_travel_m']=max(r['max_travel_m'],float(m[k,9]));r['peak_current_A']=max(r['peak_current_A'],float(m[k,11]));r['peak_voltage_V']=max(r['peak_voltage_V'],float(m[k,12]));r['native_samples']+=int(m[k,3]);r['terminated']=bool(m[k,20]);r['robot_nonzero_frames']+=int(abs(obs[k,15])>1e-12);r['gate_ready_frames']+=int(obs[k,16]>.5);r['final_true_travel_m']=float(truth[k,2]);r['final_robot_correction_m_s']=float(obs[k,15]);r['final_time_s']=float(m[k,2]);r['contact_loss']=bool(m[k,17]);r['travel_stop']=bool(m[k,18]);r['force_pass']=bool(r['peak_normal_error_N']<=.5 and not r['terminated']);r['final_abs_recenter_m']=abs(float(truth[k,2])-s['midTravel_m']);r['final_band_excess_m']=max(0,r['final_abs_recenter_m']-.0075)
  if m[k,8]<s['midTravel_m']-.0075 or m[k,9]>s['midTravel_m']+.0075:r['band_excess_time_upper_bound_s']+=float(m[k,0])
for i,c in zip(ids,cs):
 if i==121:np.savez_compressed(F/'band_selection121_baseline_native.npz',**c.take_record())
 c.close()
summary={'status':'native_appended_selection_baseline_executed','rows':rows,'force_passed':sum(r['force_pass'] for r in rows),'cases':8,'worst_force_error_N':max(r['peak_normal_error_N'] for r in rows),'holdout64_95_trajectories_executed':0,'wall_s':time.perf_counter()-start,'band_contract':'Bandexcess≥0=max(abs(truephysicaltravel-mid)-7.5mm,0); initialoutsidebandallowed byselectedtestdesign. Intervalextrema forpeaks andtimeupperboundcoverallphysicalticks. Forcepass remains±0.5N includingstartup andphysicalfailurestops.'}
(F/'band_selection_baseline.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
