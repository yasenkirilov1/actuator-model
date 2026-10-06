"""Read-only precision diagnosis of rejected 8404, without policy changes."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import torch

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent.parent/'ppo_peak_8404'
sys.path.insert(0,str(ROOT/'frozen_sources'))
from policy import UnifiedPolicy

torch.set_num_threads(2)
torch.backends.cuda.matmul.allow_tf32=False
checkpoint=ROOT/'rollouts/initial_policy/policy_checkpoint.pt'
rollout=ROOT/'rollouts/rollout_000000.npz'
saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
raw=np.load(rollout)
T,N,D=raw['observations'].shape
report={'scope':'Read-only diagnostic of a retained software-rejected rollout; no optimizer updates, no native simulation and no held-out outcomes.',
        'sources':{str(p.relative_to(HERE.parent.parent)):hashlib.sha256(p.read_bytes()).hexdigest()
                   for p in (checkpoint,rollout,ROOT/'results.json',ROOT/'frozen_sources/policy.py')},
        'shape':[T,N,D], 'sequenceReplay':{}, 'singleStepFromStoredCState':{}}


def measure(left,right):
    difference=(left.double()-right.double()).abs()
    return {'maxByAction':difference.flatten(0,1).max(0).values.tolist(),
            'rmsByAction':difference.square().mean((0,1)).sqrt().tolist()}


for device in ('cpu','cuda'):
    policy=UnifiedPolicy(**saved['config']).to(device)
    policy.load_state_dict(saved['state_dict'])
    arrays={k:torch.as_tensor(raw[k],device=device) for k in (
        'observations','initial_state','reset_before','latent_actions','action_mask','old_means','log_probs')}
    with torch.no_grad():
        seq=policy.sequence(arrays['observations'],arrays['initial_state'],arrays['reset_before'])
        log,_,_=policy.evaluate_latent(seq.mean,arrays['latent_actions'],arrays['action_mask'])
        si=(seq.mean.tanh()-arrays['old_means'].tanh())*torch.tensor([2.,.01],device=device)
        report['sequenceReplay'][device]={
            'latentMean':measure(seq.mean,arrays['old_means']),
            'physicalRequest':measure(si,torch.zeros_like(si)),
            'logProbabilityMax':float((log-arrays['log_probs']).abs().max()),
            'logProbabilityRMS':float((log-arrays['log_probs']).square().mean().sqrt())}
        state=torch.as_tensor(raw['state_inputs'].transpose(1,0,2,3).reshape(policy.num_layers,T*N,policy.hidden_size),device=device)
        step=policy.step(arrays['observations'].reshape(T*N,D),state,
                         arrays['reset_before'].reshape(-1),deterministic=True)
        expected=torch.as_tensor(raw['state_inputs'][1:].transpose(1,0,2,3),device=device)
        nextstate=step.state.reshape(policy.num_layers,T,N,policy.hidden_size)[:,:-1]
        continuing=~arrays['reset_before'][1:]
        state_diff=(nextstate-expected)[:,continuing,:]
        report['singleStepFromStoredCState'][device]={
            'latentMean':measure(step.mean.reshape(T,N,2),arrays['old_means']),
            'internalStateMax':float(state_diff.abs().max()),
            'internalStateRMS':float(state_diff.square().mean().sqrt())}

policy=UnifiedPolicy(**saved['config'])
policy.load_state_dict(saved['state_dict'])
obs=torch.from_numpy(raw['observations'])
reset=torch.from_numpy(raw['reset_before'])
initial=torch.from_numpy(raw['initial_state'])
reference=torch.from_numpy(raw['old_means'])
with torch.no_grad():
    sequence=policy.sequence(obs,initial,reset)
    means=[];state=initial.clone()
    for t in range(T):
        item=policy.step(obs[t],state,reset[t],deterministic=True)
        state=item.state;means.append(item.mean)
    serial=torch.stack(means)
    # Reconstruct highway outputs from the retained C post-step recurrent
    # states. Encoder/highway arithmetic remains Torch, so this separates
    # contributions approximately rather than proving an exact error budget.
    cpost=torch.from_numpy(raw['state_inputs'][1:])
    normalized=policy.normalize(obs[:-1]);auxiliary=policy.auxiliary(obs[:-1])
    hidden=torch.tanh(policy.encoder(torch.cat((normalized,auxiliary),dim=-1)))
    for index,layer in enumerate(policy.recurrent.layers):
        _,_,projection=layer(hidden).chunk(3,dim=-1)
        hidden=policy.recurrent._highway(hidden,cpost[:,index],projection)
    mean32,_=policy.heads(hidden,normalized,auxiliary)
    def linear64(value,layer):
        result=value.double()@layer.weight.double().T
        return result if layer.bias is None else result+layer.bias.double()
    mean64=(linear64(hidden,policy.actor)+linear64(normalized,policy.actor_skip)
            +linear64(auxiliary,policy.actor_aux))
    report['arithmeticSplit']={
        'cpuSequenceMinusCpuSerial':measure(sequence.mean,serial),
        'cpuSerialMinusC':measure(serial,reference),
        'CstatesTorchHighwayFloat32HeadsMinusC':measure(mean32,reference[:-1]),
        'CstatesTorchHighwayFloat64HeadsMinusC':measure(mean64,reference[:-1]),
        'float32VersusFloat64HeadsAtSameHidden':measure(mean32,mean64)}
    report['maximumAbsoluteWeights']={name:float(getattr(policy,name).weight.abs().max())
                                     for name in ('actor','actor_skip','actor_aux')}
report['interpretation']='Large inverse readouts amplify both recurrent-state rounding and readout cancellation. Existing mean and likelihood guards stay unchanged. Native C evaluation of fixed weights and acceptability of PPO replay are separate evidence gates.'
report['runDisposition']='Initial 28/28 selection passed; first rollout was retained but no optimizer update executed. Root started a separate bounded peak-objective lane from the earlier accepted checkpoint.'
(HERE/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('sequenceReplay','arithmeticSplit','maximumAbsoluteWeights')},indent=2))
