"""Prepare a causal, paired minGRU filter basis without supervised targets.

Paired positive/negative encoder projections share gates and constant offsets,
so their readout difference cancels the startup of Puffer's positive state.
Only measurement channels are projected. All matrices remain trainable in PPO.
"""
from pathlib import Path
import argparse,json,math,sys
import torch


def initialize_filter_bank(policy):
    if policy.hidden_size!=32 or policy.num_layers!=2:raise ValueError('Expected32x2sharedminGRU')
    # Eight pairs over high-value causal inputs: force error, acceleration
    # mass products (if enabled), geometry correction and relative velocity.
    channels=[0,0,0,0,41 if policy.encoder.in_features>=48 else 3,
              41 if policy.encoder.in_features>=48 else 3,
              42 if policy.encoder.in_features>=48 else 8,6]
    taus=[.0001,.0004,.003,.020,.0001,.0004,.0004,.0004]
    channels=channels+channels
    taus=taus+[.003,.020,.050,.15,.003,.020,.003,.020]
    assert len(channels)==16
    with torch.no_grad():
        policy.encoder.weight.zero_();policy.encoder.bias.zero_()
        policy.actor.weight.zero_()
        for pair,(channel,tau) in enumerate(zip(channels,taus)):
            policy.encoder.weight[2*pair,channel]=.05
            policy.encoder.weight[2*pair+1,channel]=-.05
        for l,layer in enumerate(policy.recurrent.layers):
            layer.weight.zero_();layer.bias.zero_()
            for pair,tau in enumerate(taus):
                for unit in (2*pair,2*pair+1):
                    # Positive candidate keeps _g exactly affine over bounded tanh.
                    layer.weight[unit,unit]=1.0
                    layer.bias[unit]=1.0
                    update=-math.expm1(-policy.dt/tau)
                    layer.bias[32+unit]=math.log(update/(1-update))
                    # Second layer starts as a trainable highway bypass;
                    # recurrence in the first layer stores the filter history.
                    layer.bias[64+unit]=20.0 if l==0 else -20.0
    return {'initialization':'paired causal minGRU filter bank, no teacher labels',
        'channels':channels,'timeConstants_s':taus,'encoderScale':.05,
        'recurrentLayers':2,'secondLayerInitiallyBypassed':True,
        'pairedReadoutDifferenceCancelsCommonStartup':True}


def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--output',required=True)
    p.add_argument('--module-dir',default=str(Path(__file__).parent));a=p.parse_args()
    sys.path.insert(0,a.module_dir)
    from policy_export import load_policy,export_policy
    from env import FEATURE_NAMES
    policy=load_policy(a.input);metadata=initialize_filter_bank(policy)
    export_policy(policy,a.output,FEATURE_NAMES,metadata)
    print(json.dumps(metadata))

if __name__=='__main__':main()
