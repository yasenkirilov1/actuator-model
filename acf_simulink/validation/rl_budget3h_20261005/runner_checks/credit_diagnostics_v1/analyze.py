"""Read-only current-exploration response estimate from retained training data.

The output is the measured/reconstructed normal-force error seen by the actor.
Random latent innovations instrument the randomized SI current perturbation.
This estimates local closed-loop response including acquisition and subsequent
policy reaction, not open-loop actuator bandwidth or hardware qualification.
"""
from pathlib import Path
import hashlib,json
import numpy as np
from scipy.signal import correlate
ROOT=Path('/home/tommie/Projects/actuator-model')/'acf_simulink/validation/rl_budget3h_20261005'
OUT=Path(__file__).resolve().parent
K=500;DT=.0001;report={'scope':__doc__,'dt_s':DT,'runs':{}}
for name in ('ppo_band_8401','ppo_band_8402'):
    run=ROOT/name
    files=[run/'rollouts'/f'rollout_{u:06d}.npz' for u in (0,49,99,149,199)
           if (run/'rollouts'/f'rollout_{u:06d}.npz').exists()]
    positive=np.zeros(K+1);negative=np.zeros(K+1);den_pos=np.zeros(K+1);den_neg=np.zeros(K+1)
    pieces=[];total=0
    for file in files:
        with np.load(file) as data:
            y=data['observations'][:,:,0].astype(np.float64)*.5
            latent=data['latent_actions'][:,:,0].astype(np.float64)
            mean=data['old_means'][:,:,0].astype(np.float64)
            reset=data['reset_before']
        innovation=latent-mean
        delta_current=2*(np.tanh(latent)-np.tanh(mean))
        for env in range(y.shape[1]):
            cuts=sorted(set([0,*np.flatnonzero(reset[:,env]).tolist(),len(y)]))
            for start,end in zip(cuts[:-1],cuts[1:]):
                length=end-start
                if length<2*K:continue
                output=y[start:end,env];instrument=innovation[start:end,env];perturbation=delta_current[start:end,env]
                output=output-output.mean();instrument=instrument-instrument.mean();perturbation=perturbation-perturbation.mean()
                covariance=correlate(output,instrument,mode='full',method='fft')
                prefix=np.r_[0,np.cumsum(instrument*perturbation)]
                a=covariance[length-1:length+K];b=covariance[length-1-np.arange(K+1)]
                dp=prefix[length-np.arange(K+1)];dn=prefix[-1]-prefix[np.arange(K+1)]
                positive+=a;negative+=b;den_pos+=dp;den_neg+=dn
                pieces.append((a,b,dp,dn));total+=length
    h=positive/den_pos;acausal=negative/den_neg
    noise=np.mean(acausal[10:]**2);energy=np.maximum(h**2-noise,0);energy[0]=0
    cumulative=np.cumsum(energy);cumulative/=cumulative[-1]
    rng=np.random.default_rng(84501);bootstrap=[]
    for _ in range(100):
        indices=rng.integers(0,len(pieces),len(pieces))
        sums=[np.sum([pieces[j][col] for j in indices],axis=0) for col in range(4)]
        bh=sums[0]/sums[2];bn=sums[1]/sums[3]
        power=np.maximum(bh**2-np.mean(bn[10:]**2),0);power[0]=0
        bc=np.cumsum(power);bc/=bc[-1]
        bootstrap.append([np.searchsorted(bc,.9)*DT*1000,bc[50],bc[100],bc[200]])
    report['runs'][name]={'files':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        'segments':len(pieces),'trainingDecisions':total,'negativeLagRms_N_A':float(noise**.5),
        'energyTimeQuantiles_ms':{str(p):float(np.searchsorted(cumulative,p)*DT*1000) for p in (.5,.8,.9,.95)},
        'energyFractionsByTime_ms':{str(ms):float(cumulative[round(ms/1000/DT)]) for ms in (1,2,5,10,20,30,50)},
        'bootstrap90percentIntervals_t90_frac5_frac10_frac20':np.quantile(bootstrap,[.05,.5,.95],axis=0).tolist(),
        'impulse_N_A':h.tolist(),'negativeLag_N_A':acausal.tolist()}
report['limitations']=['The output here is reconstructed measured force, not a retained signed true-force history.',
 'Episode segments are demeaned; finite-sample correlation tails and mixed operating cases limit tail-energy precision.',
 'Squared innovations correlated against retained true-force MSE separately showed the same approximately0.4ms main response; those longer-tail covariance estimates were noisy and are not used to claim exact true-force bandwidth.',
 'Energy timing supports a bounded5ms force-credit experiment; it does not prove that5ms optimizes PPO or establish sensor/actuator hardware bandwidth.']
(OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print({name:{k:r[k] for k in ('trainingDecisions','energyTimeQuantiles_ms','energyFractionsByTime_ms')} for name,r in report['runs'].items()})
