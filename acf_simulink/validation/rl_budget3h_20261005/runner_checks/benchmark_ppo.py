"""CPU/CUDA training update timing on one exact retained real rollout."""
from pathlib import Path
import sys,json,hashlib,shutil,time,statistics
import numpy as np
import torch
ROOT=Path('/home/tommie/Projects/actuator-model')
RUN=ROOT/'acf_simulink/validation/rl_budget3h_20261005/ppo_warm_10k_8301'
OUT=ROOT/'acf_simulink/validation/rl_budget3h_20261005/runner_checks/cpu_cuda_benchmark_v1'
OUT.mkdir(exist_ok=False)
sys.path.insert(0,str(RUN/'frozen_sources'))
from policy import UnifiedPolicy
from ppo import PPOTrainer,PPOConfig,RolloutBatch
for source,name in ((RUN/'rollouts/rollout_000000.npz','rollout.npz'),
                    (RUN/'rollouts/initial_policy/policy_checkpoint.pt','initial_policy.pt'),
                    (RUN/'results.json','run_snapshot.json')):
    shutil.copy2(source,OUT/name)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
report={'status':'running','sourceRun':str(RUN),'sources':{p.name:sha(p) for p in OUT.iterdir()},
        'scope':'Independent scratch clones only; active experiment policy/optimizer unchanged.',
        'torchThreads':2,'device':torch.cuda.get_device_name(0),'repetitions':[]}
raw=dict(np.load(OUT/'rollout.npz'))
names=('observations','latent_actions','log_probs','values','rewards','terminated','truncated',
       'next_values','reset_before','action_mask','initial_state','state_inputs','log_probs_by_action')
arrays={name:raw[name] for name in names};report['batchShape']=list(arrays['observations'].shape)
report['tensorBytes']=sum(a.nbytes for a in arrays.values())
checkpoint=torch.load(OUT/'initial_policy.pt',map_location='cpu',weights_only=False)
config=PPOConfig(**json.loads((OUT/'run_snapshot.json').read_text())['ppo'])
torch.set_num_threads(2);torch.backends.cuda.matmul.allow_tf32=False;torch.backends.cudnn.allow_tf32=False
original_randperm=torch.randperm
outputs={};start_budget=time.monotonic()
try:
    for repetition in range(3):
        for device in ('cpu','cuda'):
            if time.monotonic()-start_budget>55:break
            torch.manual_seed(89601);torch.cuda.manual_seed_all(89601)
            policy=UnifiedPolicy(**checkpoint['config']).to(device)
            policy.load_state_dict(checkpoint['state_dict']);trainer=PPOTrainer(policy,config)
            # Separate data movement from optimizer work; model loading and
            # source NPZ decoding are not part of either recurring operation.
            if device=='cuda':torch.cuda.synchronize()
            start=time.perf_counter()
            batch=RolloutBatch(**{name:torch.as_tensor(value,device=device) for name,value in arrays.items()})
            if device=='cuda':torch.cuda.synchronize()
            transfer=time.perf_counter()-start
            permutation_rng=torch.Generator(device='cpu');permutation_rng.manual_seed(89602)
            def same_randperm(count,*args,**kwargs):
                destination=kwargs.pop('device',None);kwargs.pop('generator',None)
                return original_randperm(count,*args,generator=permutation_rng,device='cpu',**kwargs).to(destination or 'cpu')
            torch.randperm=same_randperm
            start=time.perf_counter();metrics=trainer.update(batch)
            if device=='cuda':torch.cuda.synchronize()
            elapsed=time.perf_counter()-start;torch.randperm=original_randperm
            report['repetitions'].append({'repetition':repetition,'device':device,'transfer_s':transfer,
                'update_s':elapsed,'total_s':transfer+elapsed,'metrics':metrics})
            if repetition==0:outputs[device]={key:value.detach().cpu().clone() for key,value in policy.state_dict().items()}
            del trainer,policy,batch
    report['parameterMaxResidualCPUvsCUDA']=max(float((outputs['cpu'][key].double()-outputs['cuda'][key].double()).abs().max()) for key in outputs['cpu'])
    summary={}
    for device in ('cpu','cuda'):
        warm=[r for r in report['repetitions'] if r['device']==device and r['repetition']>0]
        summary[device]={'warmMedianUpdate_s':statistics.median(r['update_s'] for r in warm),
                         'warmMedianIncludingTransfer_s':statistics.median(r['total_s'] for r in warm)}
    report['summary']=summary
    report['cudaSpeedupIncludingTransfer']=summary['cpu']['warmMedianIncludingTransfer_s']/summary['cuda']['warmMedianIncludingTransfer_s']
    report['sameInitialPolicyAndOptimizer']=True;report['sameMinibatchPermutations']=True
    report['status']='complete'
except BaseException as error:
    report['status']='failed';report['error']=repr(error);raise
finally:
    torch.randperm=original_randperm
    (OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({key:report[key] for key in ('status','batchShape','tensorBytes','parameterMaxResidualCPUvsCUDA','summary','cudaSpeedupIncludingTransfer')},indent=2))
