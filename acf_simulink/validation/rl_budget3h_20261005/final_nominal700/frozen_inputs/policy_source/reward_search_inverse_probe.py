"""Disclosed mechanics-based neural initialization, selected by native rewards.

No teacher actions or simulation trajectories enter the frequency fit. It
approximates the known causal spring/damper inverse with genuine minGRU filters.
"""
from pathlib import Path
import argparse,copy,json,sys,time
import numpy as np
import torch

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from policy_export import load_policy,export_policy,flat_weights
from reward_search import EpisodeEvaluator,build_evaluator,load_scenarios,aggregate,joint_band_score,write_json,sha
sys.path.insert(0,str(HERE.parent/'rl_unified_20261005'))
from env import FEATURE_NAMES


def frequency_fit(policy,pairs,encoder_scale,sign,k=19746.,c=60.,mass_b=.6,cutoff=1500.):
    freq=np.geomspace(3.,2500.,400);s=2j*np.pi*freq;dt=policy.dt
    layer0=policy.recurrent.layers[0].bias.detach().numpy()
    layer1=policy.recurrent.layers[1].bias.detach().numpy()
    channels=[];dc=[]
    for pair in pairs:
        unit=2*pair
        u0=1/(1+np.exp(-layer0[32+unit]));u1=1/(1+np.exp(-layer1[32+unit]))
        h0=1/(1+np.exp(-layer0[64+unit]));h1=1/(1+np.exp(-layer1[64+unit]))
        f0=u0/(1-(1-u0)*np.exp(-s*dt));f1=u1/(1-(1-u1)*np.exp(-s*dt))
        # Opposed g(x) derivatives total1.25 at zero; layer1 sits positive.
        channels.append(encoder_scale*(h0*1.25*f0+(1-h0)*2)*(h1*f1+(1-h1)))
        dc.append(encoder_scale*(h0*1.25+(1-h0)*2))
    h=np.stack(channels,axis=1);dc=np.asarray(dc)
    zeta=1/np.sqrt(2);omega=2*np.pi*cutoff
    target=sign*s*s/(k+c*s)*(omega*omega)/(s*s+2*zeta*omega*s+omega*omega)
    if sign>0:target*=mass_b
    # Enforce exact zero DC through the corresponding direct physical feature.
    x=h-dc[None,:];weight=1/np.sqrt(1+(freq/150.)**4)
    matrix=np.vstack([(x*weight[:,None]).real,(x*weight[:,None]).imag])
    y=np.r_[target.real*weight,target.imag*weight]
    coefficients=np.linalg.lstsq(matrix,y,rcond=None)[0]
    direct=-dc@coefficients;response=direct+h@coefficients
    return coefficients,float(direct),{'frequency_Hz':freq.tolist(),
      'targetReal':target.real.tolist(),'targetImag':target.imag.tolist(),
      'fitReal':response.real.tolist(),'fitImag':response.imag.tolist(),
      'pairedCoefficients':coefficients.tolist(),'directCoefficient':float(direct),
      'weightedComplexRmsResidual':float(np.sqrt(np.mean(np.abs((response-target)*weight)**2))),
      'mechanics':{'k_N_m':k,'c_N_s_m':c,'mB_kg':mass_b,'rolloff_Hz':cutoff,'rolloffDamping':float(zeta)},
      'scope':'Known mechanics frequency response, not a fit to baseline or trajectory actions.'}


def candidate(base,strength,encoder_scale=.1):
    policy=copy.deepcopy(base)
    fits={}
    with torch.no_grad():
        for name,pairs,feature,aux,sign in [('acceleration',range(4,8),41,1,-1),('geometry',range(8,12),42,2,1)]:
            coefficients,direct,fit=frequency_fit(policy,pairs,encoder_scale,sign)
            fits[name]=fit
            for j,pair in enumerate(pairs):
                even=pair*2;old=float(policy.actor.weight[0,even])
                policy.encoder.weight[even].zero_();policy.encoder.weight[even+1].zero_()
                policy.encoder.bias[even:even+2].zero_()
                policy.encoder.weight[even,feature]=encoder_scale
                policy.encoder.weight[even+1,feature]=-encoder_scale
                value=old/encoder_scale+strength*float(coefficients[j])
                policy.actor.weight[0,even]=value;policy.actor.weight[0,even+1]=-value
            policy.actor_aux.weight[0,aux]+=strength*direct
    return policy,fits


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--policy',required=True)
    parser.add_argument('--native-build',required=True);parser.add_argument('--bank',required=True)
    parser.add_argument('--output',required=True);parser.add_argument('--indices',default='15,27,31,35,39,40,42,46')
    parser.add_argument('--strengths',default='0,.03,.1,.3,1');args=parser.parse_args()
    torch.set_num_threads(1);out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    base=load_policy(args.policy);scenarios=load_scenarios(args.native_build,args.bank)
    indices=[int(x) for x in args.indices.split(',')]
    assert all(scenarios[i]['split']=='train' for i in indices)
    for row in scenarios:row['robotActionScale_m_s']=.01
    provenance={'method':'known_mechanics_frequency_initialization_then_native_reward_comparison',
       'sourceCheckpoint':str(Path(args.policy).resolve()),'sourceCheckpointSha256':sha(args.policy),
       'noTeacherActions':True,'physicalActionScales':{'currentOffset_A':2.,'robotRequest_m_s':.01}}
    export_policy(base,out/'base_policy',FEATURE_NAMES,provenance)
    library=build_evaluator(args.native_build,out/'base_policy',out)
    evaluator=EpisodeEvaluator(library,scenarios)
    policies=[('base',base,None)]
    for strength in map(float,args.strengths.split(',')):
        policy,fits=candidate(base,strength)
        name='inverse_'+str(strength).replace('.','p')
        export_policy(policy,out/name,FEATURE_NAMES,{**provenance,'inverseStrength':strength,'encoderBasisScale':.1})
        write_json(out/name/'frequency_fit.json',fits)
        policies.append((name,policy,strength))
    rows=[];start=time.monotonic();recovery=0
    for name,policy,strength in policies:
        metrics=evaluator.evaluate(flat_weights(policy),indices)
        result=aggregate(metrics);result.update(name=name,inverseStrength=strength,jointScore=joint_band_score(metrics))
        result['rank']=[result['physicalFailures'],-result['passed'],result['jointScore'],result['worstNormalForceError_N'],result['meanRmsNormalForceError_N']]
        result['elapsed_s']=time.monotonic()-start
        failed=[r['scenario'] for r in metrics if r['physicalFailure']]
        if failed:
            # Exact weights and all scalar failures retained; one full native
            # representative per candidate bounds disk/compression cost.
            recorded=evaluator.evaluate(flat_weights(policy),failed[:1],record_dir=out/name/'failed_native')
            write_json(out/name/'failed_native/metrics.json',{'rows':recorded,'recoveryExcludedFromComparisonCount':True})
            recovery+=len(recorded)
        rows.append(result)
        write_json(out/'results.json',{'status':'running','comparisons':rows,
           'episodeComparisons':sum(len(x['rows']) for x in rows),'recoveryEpisodes':recovery,
           'trainOnly':True,'nativeLibrarySha256':sha(Path(args.native_build)/'libacf_native.so')})
        print(json.dumps({k:v for k,v in result.items() if k!='rows'}),flush=True)
    best=min(rows,key=lambda r:r['rank'])
    report={'status':'complete','comparisons':rows,'selected':best['name'],
       'episodeComparisons':sum(len(x['rows']) for x in rows),'recoveryEpisodes':recovery,
       'trainOnly':True,'nativeLibrarySha256':sha(Path(args.native_build)/'libacf_native.so'),
       'elapsed_s':time.monotonic()-start,'provenance':provenance}
    write_json(out/'results.json',report)

if __name__=='__main__':main()
