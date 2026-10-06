"""Band objective and cross-cadence score verification on native training cases."""
from pathlib import Path
import sys,json,copy
import numpy as np
ROOT=Path('/home/tommie/Projects/actuator-model');sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from env import RewardConfig,travel_cost,FeatureEncoder,ROBOT_SCALE_M_S
from train import apply_travel_band_contract,advance_band_evaluation,get_scenarios
from native_api import NativePlant
OUT=ROOT/'acf_simulink/validation/rl_budget3h_20261005/runner_checks/band_v1';OUT.mkdir(exist_ok=False)
report={'status':'running'}
try:
    cfg=RewardConfig(acceptable_travel_radius_m=.0075,travel_excess_weight=10,inside_travel_weight=0)
    peak=np.array([0,.006,.0075,.009])
    cost=travel_cost(.0125-peak,.0125+peak,np.zeros(4),.0125,cfg)
    assert np.allclose(cost[:3],0,atol=1e-24) and abs(cost[3]-.9)<1e-12
    report['bandCosts']=cost.tolist();report['insideCostZero']=True
    row={'identity':'fixture','initialAbsTravelError_m':.009,'finalAbsTravelError_m':.009,
         'peakAbsTravelError_m':.009,'duration_s':4,'completed':True,'passed':True,
         'rmsNormalForceError_N':.05,'integratedTravelBandPeakExcess_m2_s':4*(.0015)**2}
    sample={'rows':[row],'worstNormalForceError_N':.45,'passed':1}
    apply_travel_band_contract(sample,.0075)
    assert abs(sample['jointScore']-4.395)<1e-12 and sample['passed']==1
    report['constant9mmScore']=sample['jointScore'];report['forcePassNotChangedByBand']=True
    plant=NativePlant(ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_bank_band',threads=2)
    scenarios=get_scenarios(plant,ROOT/'acf_simulink/validation/rl_budget3h_20261005/scenario_bank_band/manifest.json')
    encoder=FeatureEncoder(scenarios)
    requests=encoder.actions(np.array([[0,1],[0,-1]],np.float32),[96,97])
    assert requests[0,1]==.01 and requests[1,1]==-.01 and ROBOT_SCALE_M_S==.01
    report['robotRequestRange_m_s']=[-.01,.01]
    left=plant.create(96,seed=89911);right=plant.create(96,seed=89911)
    action=np.array([[scenarios[96]['knownEquilibriumCurrent_A'],0.0]])
    mid=np.array([scenarios[96]['midTravel_m']])
    one=0
    for _ in range(10):
        r,b=advance_band_evaluation(plant,[left],action,1,mid,.0075);one+=b[0]
    rows,total=advance_band_evaluation(plant,[right],action,10,mid,.0075)
    assert one==total[0] and np.array_equal(left.result.obs,right.result.obs)
    assert np.array_equal(left.result.truth_last,right.result.truth_last)
    report['common100usProxyBitIdenticalAcrossActorCadences']=True
    report['nativeIntegratedPeakExcess_m2_s']=float(one)
    report['nativeEndpointUnchanged']=True
    left.close();right.close();report['status']='passed'
except BaseException as error:report['status']='failed';report['error']=repr(error);raise
finally:(OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
