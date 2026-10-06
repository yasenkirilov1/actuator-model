from pathlib import Path
import sys,time,json,numpy as np
ROOT=Path('/home/tommie/Projects/actuator-model');sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from native_api import NativePlant
F=ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_bank_band';old=NativePlant(F.parent/'native_bank',threads=2);new=NativePlant(F,threads=2)
ids=[0,23,56,63];a=[old.create(i,seed=15100+i,baseline=False,record=True) for i in ids];b=[new.create(i,seed=15100+i,baseline=False,record=True) for i in ids]
for k in range(3000):
 cmp=np.stack([c.result.comparator for c in a]);action=np.column_stack((cmp[:,0]+.00003*np.sin(k*.001+np.arange(4)),cmp[:,1]*.8))
 x=old.step_batch(a,action);y=new.step_batch(b,action)
 for p,q in zip(x,y):
  assert np.array_equal(p.obs,q.obs) and np.array_equal(p.truth_last,q.truth_last) and np.array_equal(p.comparator,q.comparator)
  assert p.metrics==q.metrics
for p,q in zip(a,b):
 for key,value in p.take_record().items():assert np.array_equal(value,q.take_record()[key]),key
 p.close();q.close()
report={'status':'original_native_prefix_trajectories_identical','scenarios':ids,'action_steps_each':3000,'physical_samples':360004,'endpoint72_values_exact':True,'full_native_time_inputs_truth_exact':True,'original_holdout64_95_not_run':True}
(F/'prefix_native_parity.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
