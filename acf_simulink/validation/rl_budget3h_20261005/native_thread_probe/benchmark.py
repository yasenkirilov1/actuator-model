from pathlib import Path
import sys,time,json,ctypes as ct
import numpy as np
ROOT=Path('/home/tommie/Projects/actuator-model')
sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from native_api import NativePlant
F=ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_thread_probe'
p=NativePlant(F,threads=1)
N=16;steps=6000;scenarios=[i%24 for i in range(N)]
def episode(threads,record):
 p.set_threads(threads)
 contexts=[p.create(s,seed=11100+i,baseline=False,record=record) for i,s in enumerate(scenarios)]
 hist=[];cmp=np.stack([c.result.comparator for c in contexts])
 start=time.perf_counter()
 for k in range(steps):
  acts=np.column_stack((cmp[:,0]+.0001*np.sin(k*.001+np.arange(N)),cmp[:,1]*.75))
  arrays=p.step_batch_arrays(contexts,acts);cmp=arrays[2]
  if record:hist.append(np.concatenate(arrays,axis=1))
 elapsed=time.perf_counter()-start
 records=[c.take_record() for c in contexts] if record else []
 [c.close() for c in contexts]
 return np.stack(hist) if record else None,records,elapsed
serial,serial_records,ts=episode(1,True)
parallel,parallel_records,tp=episode(4,True)
assert np.array_equal(serial,parallel),'Serial and parallel endpoint outputs differ'
for a,b in zip(serial_records,parallel_records):
 for key in a:assert np.array_equal(a[key],b[key]),f'Native full-history {key} differs'
summary={'serial_parallel_bit_identical':True,'paired_contexts':N,'action_steps_per_context':steps,
 'native_samples':sum(len(r['time']) for r in serial_records),'all_returned_channels':72,
 'serial_s':ts,'parallel4_s':tp,'trials':[]}
np.savez_compressed(F/'identity_representative.npz',**serial_records[0])
for threads in [1,2,4,8]:
 p.set_threads(threads);cs=[p.create(s,seed=12100+i,baseline=True) for i,s in enumerate(scenarios)]
 acts=np.zeros((N,2));start=time.perf_counter();iterations=2500
 for k in range(iterations):p.step_batch_arrays(cs,acts)
 elapsed=time.perf_counter()-start
 summary['trials'].append({'threads':threads,'contexts':N,'transitions':N*iterations,'elapsed_s':elapsed,'transitions_s':N*iterations/elapsed})
 [c.close() for c in cs]
 print(json.dumps(summary['trials'][-1]),flush=True)
# Compare old single-thread codegen wrapping and strict-math x86-64 compilation.
import importlib.util
spec=importlib.util.spec_from_file_location('prior_api',ROOT/'acf_simulink/experiments/rl_unified_20261005/native_api.py');old=importlib.util.module_from_spec(spec);sys.modules[spec.name]=old;spec.loader.exec_module(old)
oldplant=old.NativePlant(ROOT/'acf_simulink/validation/rl_native_probe_20261005_bank')
p.set_threads(4);oldcs=[oldplant.create(i,seed=14100+i,baseline=True,record=True) for i in range(4)];newcs=[p.create(i,seed=14100+i,baseline=True,record=True) for i in range(4)]
for k in range(4000):
 a=oldplant.step_batch(oldcs,np.zeros((4,2)));b=p.step_batch(newcs,np.zeros((4,2)))
 for x,y in zip(a,b):
  for key in ['obs','truth_last','comparator']:assert np.array_equal(getattr(x,key),getattr(y,key)),key
  assert x.metrics==y.metrics,'metrics'
for x,y in zip(oldcs,newcs):
 a=x.take_record();b=y.take_record()
 for key in a:assert np.array_equal(a[key],b[key]),key
 x.close();y.close()
summary['prior_bridge_bit_identical']=True;summary['prior_bridge_contexts']=4;summary['prior_bridge_native_samples']=4*120001
summary['hashes']=json.loads((F/'bridge_build.json').read_text())
(F/'thread_acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2),flush=True)
