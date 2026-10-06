"""Native, reset, exact-resume and zero-weight checks for episodic peak cost."""
from dataclasses import replace
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import sys

import numpy as np
import torch

ROOT = Path('/home/tommie/Projects/actuator-model')
SOURCE = ROOT/'acf_simulink/experiments/rl_budget3h_20261005'
BASE = ROOT/'acf_simulink/validation/rl_budget3h_20261005'
sys.path.insert(0, str(SOURCE))
from env import RewardConfig, force_running_peak_cost, FEATURE_NAMES
from train import Collector, get_scenarios, evaluate, array_rewards, reward_value
from native_api import NativePlant, _result
from policy import UnifiedPolicy

OUT = BASE/'runner_checks/running_peak_v1'
OUT.mkdir(exist_ok=False)
report = {'status': 'running', 'scope': 'Training cases only; no held-out records accessed.'}
torch.set_num_threads(2)
try:
    cfg = RewardConfig(force_error_scale_N=.45, force_running_peak_weight=1,
        force_mse_weight=0, force_excess_weight=0, recenter_weight=0,
        effort_weight=0, current_change_weight=0, robot_change_weight=0,
        physical_failure_penalty=0, return_scale=1)
    initial = .2
    running = initial
    cost = 0.0
    for step, interval_peak in enumerate((.1, .18, .3, .25, .4)):
        previous = 0 if step == 0 else running
        running = max(running, interval_peak)
        cost += float(force_running_peak_cost(previous, running, cfg))
    assert abs(cost-(.4/.45)**2) < 1e-14
    report['analyticTelescopingResidual'] = abs(cost-(.4/.45)**2)
    report['nonzeroStartupIncluded'] = True

    # Native interval arrays and object transport must implement the same
    # dt-independent increment, with zero cost when the running maximum holds.
    native = np.zeros((2,24),np.float64)
    native[:,0] = [1e-4, 1e-3]
    native[:,6] = [.3,.4]
    issued = np.zeros((2,2))
    array = array_rewards(native, issued, issued, cfg,
                          previous_running_peak=np.array([.2,.4]), running_peak=np.array([.3,.4]))
    assert np.allclose(array, [-((.3**2-.2**2)/.45**2),0], rtol=0, atol=1e-15)
    scalar = []
    for j in range(2):
        result = _result(np.zeros(18),np.zeros(27),np.zeros(3),native[j])
        scalar.append(reward_value(result,issued[j],issued[j],cfg,
            previous_running_peak=[.2,.4][j],running_peak=[.3,.4][j]))
    assert np.array_equal(array,np.asarray(scalar))
    report['nativeArrayObjectIncrementParity'] = True
    report['noDtMultiplier'] = True

    plant = NativePlant(BASE/'native_bank_sustained',threads=2)
    scenarios = get_scenarios(plant,BASE/'scenario_bank_sustained/manifest.json')
    checkpoint = BASE/'reward_search_band_8203/checkpoints/generation_0005/policy_checkpoint.pt'
    saved = torch.load(checkpoint,map_location='cpu',weights_only=False)
    policy = UnifiedPolicy(**saved['config']).cpu()
    policy.load_state_dict(saved['state_dict'])
    report['policyCheckpoint'] = str(checkpoint)
    report['policyCheckpointSha256'] = hashlib.sha256(checkpoint.read_bytes()).hexdigest()

    # The same native model, two full training recordings, both include t=0.
    native_eval = evaluate(plant,scenarios,[0,42],policy,OUT/'full_native','train_peak',
                           save_histories=True,reward_config=cfg)
    residuals = []
    rows = []
    for row in native_eval['rows']:
        expected = -(row['peakNormalForceError_N']/.45)**2
        residuals.append(abs(row['reward']-expected))
        assert row['nativeFirstTime_s']==0 and row['nativeGridComplete']
        assert abs(row['reward']-expected)<1e-10
        rows.append({k:row[k] for k in ('identity','nativeSamples','nativeFirstTime_s',
                    'peakNormalForceError_N','reward','completed','physicalFailure')})
    report['fullNativeRecords'] = rows
    report['fullNativeTelescopingMaxResidual'] = max(residuals)

    # Artificially short horizons exercise frequent reset/truncation without
    # modifying the native physics. Resume occurs in the middle of an episode.
    short = [{**r,'duration_s':.004} for r in scenarios]
    collector = Collector(plant,short,policy,OUT/'collector',2,84521,cfg,
                          train_ids=[0,1],failure_native_limit=0)
    first,_ = collector.collect(23,0)
    state = collector.state_dict()
    resumed = Collector(plant,short,policy,OUT/'resumed',2,99,cfg,
                        train_ids=[0,1],failure_native_limit=0)
    resumed.load_state_dict(state)
    left,_ = collector.collect(97,1)
    right,_ = resumed.collect(97,1)
    for name in vars(left):
        lvalue,rvalue=getattr(left,name),getattr(right,name)
        if isinstance(lvalue,torch.Tensor):assert torch.equal(lvalue,rvalue),name
    for name,array in collector.last_rollout[1].items():
        assert np.array_equal(array,resumed.last_rollout[1][name]),name
    episodes=[json.loads(line) for line in (OUT/'collector/training_episodes.jsonl').read_text().splitlines()]
    assert len(episodes)>=4 and all(r['truncated'] and not r['terminated'] for r in episodes)
    error=max(abs(r['return']+(r['peakNormalForceError_N']/.45)**2) for r in episodes)
    assert error<2e-7
    report['resetTruncationEpisodes'] = len(episodes)
    report['collectorFloat32ReturnTelescopingMaxResidual'] = error
    report['exactMidEpisodeResumePassed'] = True
    bad = copy.deepcopy(state);bad['episode_peak'][0]+=.01
    try:resumed.load_state_dict(bad)
    except AssertionError as exc:assert 'force peak' in str(exc)
    else:raise AssertionError('corrupted carried peak was not detected')
    report['corruptedPeakRejected'] = True
    collector.close();resumed.close()

    # Zero new weight must preserve every prior collected value, reset and
    # request bit-for-bit against the complete frozen v5 collector.
    reference_path=BASE/'executors/v5/train.py'
    spec=importlib.util.spec_from_file_location('frozen_v5_train',reference_path)
    reference=importlib.util.module_from_spec(spec);sys.modules[spec.name]=reference;spec.loader.exec_module(reference)
    zero=RewardConfig(force_running_peak_weight=0,acceptable_travel_radius_m=.0075)
    current=Collector(plant,short,policy,OUT/'zero_current',2,84525,zero,train_ids=[0,1],failure_native_limit=0)
    previous=reference.Collector(plant,short,policy,OUT/'zero_reference',2,84525,zero,train_ids=[0,1],failure_native_limit=0)
    for update,length in enumerate((27,71)):
        current.collect(length,update);previous.collect(length,update)
        assert current.last_rollout[1].keys()==previous.last_rollout[1].keys()
        for name,array in current.last_rollout[1].items():
            assert np.array_equal(array,previous.last_rollout[1][name]),name
    report['zeroWeightBitIdenticalToFrozenV5'] = True
    report['zeroWeightReferenceSha256'] = hashlib.sha256(reference_path.read_bytes()).hexdigest()
    current.close();previous.close()
    frozen=OUT/'frozen_sources';frozen.mkdir()
    report['sourceHashes']={}
    for name in ('train.py','env.py','ppo_multiscale.py'):
        shutil.copy2(SOURCE/name,frozen/name)
        report['sourceHashes'][name]=hashlib.sha256((SOURCE/name).read_bytes()).hexdigest()
    report['status']='passed'
except BaseException as error:
    report['status']='failed';report['error']=repr(error)
    raise
finally:
    (OUT/'results.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
