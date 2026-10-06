from pathlib import Path
import sys,time,json
import numpy as np
ROOT=Path('/home/tommie/Projects/actuator-model');sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from native_api import NativePlant,METRIC_NAMES
F=ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_thread_probe';p=NativePlant(F,threads=4)
cs=[p.create(i,seed=15100+i,baseline=False,record=True) for i in range(8)]
cr=[p.create(i,seed=15100+i,baseline=False,record=True) for i in range(8)]
weighted_diff=0;maxerr=0;groups=600
for group in range(groups):
 cmp=np.stack([c.result.comparator for c in cs]);actions=np.column_stack((cmp[:,0]+.00005*np.sin(group*.1+np.arange(8)),cmp[:,1]*.8))
 parts=[p.step_batch(cs,actions) for k in range(10)]
 repeat=p.step_repeat_batch_arrays(cr,actions,10)
 for i in range(8):
  final=parts[-1][i]
  assert np.array_equal(final.obs,repeat[0][i]);assert np.array_equal(final.truth_last,repeat[1][i]);assert np.array_equal(final.comparator,repeat[2][i])
  count=sum(a[i].native_samples for a in parts);assert count==repeat[3][i,3]
  for col in [7,10,13]:
   weighted=sum(a[i].metrics[METRIC_NAMES[col]]*a[i].native_samples for a in parts)/count
   weighted_diff=max(weighted_diff,abs(weighted-repeat[3][i,col]))
  for col in [4,8]:assert min(a[i].metrics[METRIC_NAMES[col]] for a in parts)==repeat[3][i,col]
  for col in [5,6,9,11,12]:assert max(a[i].metrics[METRIC_NAMES[col]] for a in parts)==repeat[3][i,col]
  for col in [14,15,16]:assert sum(a[i].metrics[METRIC_NAMES[col]] for a in parts)==repeat[3][i,col]
for a,b in zip(cs,cr):
 for key,value in a.take_record().items():assert np.array_equal(value,b.take_record()[key]),key
 a.close();b.close()
# Contact failure must stop during a repeat, not advance to its nominal horizon.
a=p.create(0,baseline=False,record=True);b=p.create(0,baseline=False,record=True)
count=0
for k in range(100):
 x=a.step(-3.5,0);count+=x.native_samples
 if x.terminated:break
y=p.step_repeat_batch_arrays([b],np.array([[-3.5,0.]]),100)
assert y[3][0,20]==1 and y[3][0,17]==1 and y[3][0,3]==count
for key,value in a.take_record().items():assert np.array_equal(value,b.take_record()[key]),key
summary={'repeat10_vs_10individual_bit_identical_native_histories':True,'contexts':8,'repeat_groups':groups,
 'paired_physical_samples':8*(groups*300+1),'all_endpoint_obs_truth_comparator_exact':True,'largest_mse_aggregation_roundoff':weighted_diff,
 'failure_stops_inside_repeat':True,'contact_failure_time_s':float(y[3][0,2]),'contact_failure_samples':int(count),'trials':[]}
a.close();b.close()
for threads in [1,2,4,8]:
 p.set_threads(threads);c=[p.create(i,baseline=True) for i in range(16)];actions=np.zeros((16,2));iterations=1000
 t=time.perf_counter()
 for k in range(iterations):p.step_repeat_batch_arrays(c,actions,10)
 elapsed=time.perf_counter()-t
 summary['trials'].append({'threads':threads,'contexts':16,'actor_decisions':iterations*16,'native100us_steps':iterations*16*10,'elapsed_s':elapsed,'native_steps_s':iterations*16*10/elapsed,'actor_decisions_s':iterations*16/elapsed})
 [x.close() for x in c]
 print(json.dumps(summary['trials'][-1]),flush=True)
(F/'repeat_acceptance.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
