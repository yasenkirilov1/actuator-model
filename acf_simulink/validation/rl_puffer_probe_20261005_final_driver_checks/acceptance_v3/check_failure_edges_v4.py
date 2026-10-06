from pathlib import Path
import importlib.util
import json
import numpy as np

source=Path('acf_simulink/experiments/rl_budget3h_20261005/final_evaluate.py').resolve()
spec=importlib.util.spec_from_file_location('final_evaluate_edges',source)
driver=importlib.util.module_from_spec(spec);spec.loader.exec_module(driver)
scenario=json.loads(Path('acf_simulink/validation/rl_budget3h_20261005/scenario_bank_sustained/manifest.json').read_text())['cases'][0]
metadata=json.loads(Path('acf_simulink/validation/rl_budget3h_20261005/native_bank_sustained/metadata.json').read_text())
stats=np.zeros(29);actions=np.empty((0,45));record=np.empty((0,31))
row=driver.score_full(record,actions,stats,scenario,metadata)
assert row['softwareRejected'] and not row['passed'] and row['nativeSamples']==0
assert len(driver.array_sha(record))==64 and len(driver.array_sha(actions))==64
driver.atomic_json(Path(__file__).parent/'empty_attempt_metrics.json',row)
record=np.zeros((2,31));record[:,0]=[0,np.nan];record[:,4]=scenario['force_N'];record[:,22]=1
row=driver.score_full(record,actions,stats,scenario,metadata)
assert row['softwareRejected'] and not row['passed'] and row['nativeLastTime_s'] is None
driver.atomic_json(Path(__file__).parent/'nonfinite_timestamp_metrics.json',row)
driver.atomic_json(Path(__file__).parent/'failure_edges_v4_results.json',{'status':'complete',
    'syntheticOnly':True,'noPhysicalCasesExecuted':True,'driverSha256':driver.sha(source),
    'emptyAttemptHashAndJsonPreserved':True,'nonfiniteTimestampJsonPreserved':True})
print('empty/nonfinite failure-edge checks passed')
