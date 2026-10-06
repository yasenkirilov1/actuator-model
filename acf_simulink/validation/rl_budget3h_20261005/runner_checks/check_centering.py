"""Executed centering guard against a zero-robot shortcut."""
from pathlib import Path
import json,sys
import torch
ROOT=Path('/home/tommie/Projects/actuator-model');sys.path.insert(0,str(ROOT/'acf_simulink/experiments/rl_budget3h_20261005'))
from native_api import NativePlant
from policy_export import load_policy
from train import evaluate,get_scenarios,write_json
OUT=ROOT/'acf_simulink/validation/rl_budget3h_20261005/runner_checks/centering_v1';OUT.mkdir(exist_ok=False)
plant=NativePlant(ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_bank',threads=2)
scenarios=get_scenarios(plant,ROOT/'acf_simulink/validation/rl_budget3h_20261005/scenario_bank/manifest.json')
baseline=ROOT/'acf_simulink/validation/rl_budget3h_20261005/native_bank/selection_baseline.json'
policy=load_policy(ROOT/'acf_simulink/validation/rl_budget3h_20261005/reward_search_direct_8201/checkpoints/generation_0005/policy_checkpoint.pt')
torch.set_num_threads(2)
reference=evaluate(plant,scenarios,[48,49],policy,OUT/'baseline','baseline',baseline=True,centering_baseline=baseline)
assert reference['passed']==2 and reference['centeringViolations']==0
with torch.no_grad():
    for layer in (policy.actor,policy.actor_skip,policy.actor_aux):
        layer.weight[1].zero_()
        if layer.bias is not None:layer.bias[1].zero_()
zero=evaluate(plant,scenarios,[48,49],policy,OUT/'zero_robot','zero_robot',centering_baseline=baseline)
assert zero['passed']==2 and zero['physicalFailures']==0 and zero['centeringViolations']==2
assert all(r['robotNonzeroIssued100HzFrames']==0 for r in zero['rows'])
report={'status':'passed','baselineForcePasses':reference['passed'],'baselineCenteringViolations':reference['centeringViolations'],
        'baselineProgressFractions':[r['fractionOfBaselineCenteringReduction'] for r in reference['rows']],
        'zeroRobotForcePasses':zero['passed'],'zeroRobotCenteringViolations':zero['centeringViolations'],
        'zeroRobotProgressFractions':[r['fractionOfBaselineCenteringReduction'] for r in zero['rows']],
        'zeroRobotCorrectlyRejected':True,'nativeRecordings':4}
write_json(OUT/'results.json',report);print(json.dumps(report,indent=2))
