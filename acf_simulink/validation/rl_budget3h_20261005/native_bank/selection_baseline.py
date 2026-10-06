from pathlib import Path
import sys,json,time
import numpy as np
ROOT=Path('/home/tommie/Projects/actuator-model');sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from native_api import NativePlant
F=ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_bank';p=NativePlant(F,threads=2)
ids=list(range(48,64));cs=[p.create(i,baseline=True,record=i in [56,63]) for i in ids]
peak=np.zeros(16);normalsq=np.zeros(16);samples=np.zeros(16);current=np.zeros(16);voltage=np.zeros(16);minstroke=np.full(16,np.inf);maxstroke=np.full(16,-np.inf);failure=np.zeros(16,bool);robot=np.zeros(16,int);gate=np.zeros(16,int);strokeinitial=np.array([c.result.truth_last[2] for c in cs]);requested=np.zeros((16,2));start=time.perf_counter()
for step in range(200):
 obs,truth,cmp,m=p.step_repeat_batch_arrays(cs,requested,100)
 peak=np.maximum(peak,m[:,6]);normalsq+=m[:,7]*m[:,3];samples+=m[:,3];current=np.maximum(current,m[:,11]);voltage=np.maximum(voltage,m[:,12]);minstroke=np.minimum(minstroke,m[:,8]);maxstroke=np.maximum(maxstroke,m[:,9]);failure|=m[:,20]>.5;robot+=np.abs(obs[:,15])>1e-12;gate+=obs[:,16]>.5
rows=[]
for k,i in enumerate(ids):
 s=p.scenarios[i];r={'scenario':i,'identity':s['identity'],'force_N':s['force_N'],'initial_travel_m':strokeinitial[k],'final_travel_m':float(truth[k,2]),'initial_abs_recenter_m':abs(strokeinitial[k]-s['midTravel_m']),'final_abs_recenter_m':abs(float(truth[k,2])-s['midTravel_m']),'peak_normal_error_N':peak[k],'force_pass':bool(peak[k]<=.5 and not failure[k]),'terminated':bool(failure[k]),'normal_rms_N':float(np.sqrt(normalsq[k]/samples[k])),'physical_advanced_samples':int(samples[k]),'min_travel_m':minstroke[k],'max_travel_m':maxstroke[k],'peak_current_A':current[k],'peak_voltage_V':voltage[k],'robot_nonzero_100Hz_frames':int(robot[k]),'gate_ready_100Hz_frames':int(gate[k])}
 rows.append(r)
 if i in [56,63]:
  record=cs[k].take_record();np.savez_compressed(F/f'selection_{i}_baseline_native.npz',**record)
summary={'status':'native_selection_baseline_executed','cases':rows,'passed':sum(r['force_pass'] for r in rows),'total':len(rows),'worst_peak_N':max(r['peak_normal_error_N'] for r in rows),'elapsed_wall_s':time.perf_counter()-start,'holdout_cases_evaluated':0,'recenter_frame_contract':'Native observed postgate correction and gate on 100Hz boundary snapshots; physical force extrema cover every3.333us tick and endpoints. RMS covers600000advanced ticks inclstartup; endpointextrema included.'}
(F/'selection_baseline.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
[x.close() for x in cs]
