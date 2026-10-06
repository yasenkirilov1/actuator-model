from pathlib import Path
import copy
import importlib.util
import json
import tempfile
import numpy as np

source=Path('acf_simulink/experiments/rl_budget3h_20261005/final_evaluate.py').resolve()
spec=importlib.util.spec_from_file_location('final_evaluate_checks',source)
driver=importlib.util.module_from_spec(spec);spec.loader.exec_module(driver)
bank=json.loads(Path('acf_simulink/validation/rl_budget3h_20261005/scenario_bank_sustained/manifest.json').read_text())
metadata=json.loads(Path('acf_simulink/validation/rl_budget3h_20261005/native_bank_sustained/metadata.json').read_text())
scenario=copy.deepcopy(bank['cases'][0]);scenario['duration_s']=3*metadata['step_s'];scenario['offset_m']=0
record=np.zeros((4,31));record[:,0]=np.arange(4)*metadata['step_s']
record[:,4]=scenario['force_N'];record[:,6]=scenario['midTravel_m'];record[:,22]=1
stats=np.zeros(29);stats[12]=1;stats[24]=4
actions=np.zeros((0,45))
results={}
def score(data=record,history=actions,s=stats):
    return driver.score_full(data,history,s,scenario,metadata)

row=score();assert row['passed'] and row['travelBandPassed'] and row['nativeGridComplete']
results['zero_robot_inside_band_passes']=True
data=record.copy();data[-1,4]+=.7
row=score(data);assert not row['passed'] and row['forceOutsideSamples']==1 and abs(row['peakNormalForceError_N']-.7)<1e-12
results['final_sample_force_failure_detected']=True
data=record.copy();data[-1,22]=0
row=score(data);assert row['contactLoss'] and row['physicalFailure'] and not row['passed']
results['unflagged_record_contact_loss_detected']=True
data=record.copy();data[-1,25]=1
row=score(data);assert row['travelStop'] and row['physicalFailure'] and not row['passed']
results['unflagged_record_travel_stop_detected']=True
data=record.copy();data[1,18]=scenario['peakCurrent_A']+.1;data[2,19]=scenario['usableVoltage_V']+.1
data[1,26]=1;data[2,27]=1
row=score(data);assert not row['passed'] and row['overCurrentSamples']==1 and row['overVoltageSamples']==1 and row['currentLimitSamples']==1 and row['voltageClipSamples']==1
results['electrical_extrema_and_flags_scored']=True
data=record.copy();data[-1,4]=np.nan
row=score(data);assert row['softwareRejected'] and not row['passed']
results['nonfinite_record_rejected']=True
data=record.copy();data[0,6]+=0.009
row=score(data);assert row['passed'] and not row['travelBandPassed'] and row['travelWithinBandFraction']==.75 and row['travelOutsideBandSamples']==1
assert abs(row['travelOutsideBandTime_s']-metadata['step_s'])<1e-15
results['force_and_band_acceptance_separate']=True
history=np.zeros((1,45));history[0,2]=.001
row=score(history=history);assert not row['passed'] and row['requestGateOrClockViolations']==1
results['closed_gate_raw_request_rejected']=True
history=np.zeros((1,45));history[0,0]=.0001;history[0,19]=1;history[0,2]=.001
row=score(history=history);assert not row['passed'] and row['requestGateOrClockViolations']==1
results['non100Hz_raw_request_rejected']=True
with tempfile.TemporaryDirectory() as folder:
    path=Path(folder)/'roundtrip.npz'
    data=np.arange(60,dtype=np.float64).reshape(10,6)[:,::2]
    driver.atomic_npz(path,{'noncontiguous':data},1)
    loaded=np.load(path)['noncontiguous'];assert driver.array_sha(loaded)==driver.array_sha(data)
results['lossless_noncontiguous_npz_roundtrip']=True
for fixture,text,split in [(True,'64','holdout'),(False,'0','train')]:
    try:driver.parse_indices(text,bank['cases'],fixture)
    except ValueError:pass
    else:raise AssertionError('split guard accepted wrong case')
results['split_guards']=True
output=Path(__file__).parent/'scoring_results.json'
driver.atomic_json(output,{'status':'complete','syntheticOnly':True,'noPhysicalCasesExecuted':True,
    'driverSha256':driver.sha(source),'checks':results})
print(json.dumps(results))
