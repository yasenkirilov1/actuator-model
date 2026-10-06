"""Audit completed root PPO attempts; do not simulate or read held-out data."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

HERE=Path(__file__).resolve().parent
BASE=HERE.parent.parent
NAMES=[
    'legacy_lr_8101', 'pilot_10k_8201', 'pilot_5k_8202', 'pilot_1k_8203',
    'ppo_warm_10k_8301', 'ppo_warm_10k_8302', 'ppo_band_8401',
    'ppo_band_8402', 'ppo_tail_8403', 'ppo_peak_8404', 'ppo_peak_8405',
]


def bound(path):
    return {'path':str(path.relative_to(BASE)),
            'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


attempts=[]
for name in NAMES:
    directory=BASE/name
    source=directory/'results.json'
    record=json.loads(source.read_text())
    updates=record.get('updates',[])
    assert record['status'] in ('training_complete','preparation_rejection','software_rejection')
    completed=len(updates)
    decisions=sum(row['transitions'] for row in updates)
    episodes=sum(row['episodesCompleted'] for row in updates)
    failures=sum(row['physicalFailures'] for row in updates)
    assert record.get('trainedTransitions',decisions)==decisions
    assert record.get('episodesCompleted',episodes)==episodes
    assert record.get('physicalFailureEpisodes',failures)==failures
    row={
        'name':name, 'status':record['status'], 'seed':record.get('seed',record.get('args',{}).get('seed')),
        'stopReason':record.get('stopReason'), 'completedPpoRolloutUpdates':completed,
        'completedOptimizerMinibatchSteps':sum(item.get('optimizer_steps',0) for item in updates),
        'decisionsUsedByCompletedUpdates':decisions, 'executedRejectedDecisions':0,
        'completedTrainingEpisodes':episodes,
        'completedTrainingEpisodeForcePasses':sum(item['episodePasses'] for item in updates),
        'trainingPhysicalFailures':failures,
        'selectedUpdate':record.get('selectedUpdate'), 'selectedRank':record.get('selectedRank'),
        'decisionRepeat':record.get('args',{}).get('decision_repeat'),
        'nativeBuild':record.get('nativeBuild'), 'bankSha256':record.get('bankSha256'),
        'reward':record.get('reward'), 'ppoConfig':record.get('ppo'),
        'warmStart':record.get('policyWarmStart'), 'sources':[bound(source)],
    }
    selection=[]
    for path in sorted((directory/'selection').glob('*.json')):
        data=json.loads(path.read_text())
        if 'recordings' not in data:continue
        point={k:data.get(k) for k in (
            'phase','recordings','passed','physicalFailures','worstNormalForceError_N',
            'meanRmsNormalForceError_N','jointScore','finalTravelBandViolations')}
        selection.append(point)
        row['sources'].append(bound(path))
    row['selectionEvaluations']=selection
    row['completedSelectionRecordings']=sum(point['recordings'] for point in selection)
    if updates:
        row['trainingRolloutWorstError_N']=max(item['rolloutPeakNormalForceError_N'] for item in updates)
        row['optimizerEarlyStops']=sum(bool(item.get('early_stop')) for item in updates)
    if name=='ppo_peak_8404':
        checkpoint=directory/'latest_training.pt'
        saved=torch.load(checkpoint,map_location='cpu',weights_only=False)
        assert saved['update_count']==0 and saved['collector']['episode_count']==0
        assert saved['collector']['failed_count']==0
        rollout=directory/'rollouts/rollout_000000.npz'
        data=np.load(rollout)
        count=int(np.prod(data['observations'].shape[:2]))
        assert count==saved['collector']['transitions']==98304
        assert not data['terminated'].any()
        row['executedRejectedDecisions']=count
        row['rejectionReason']='C-versus-Torch behavior replay mean guard; likelihood also exceeded its existing guard on diagnosis. No optimizer update.'
        row['sources'].extend([bound(checkpoint),bound(rollout)])
    elif record['status']=='preparation_rejection':
        assert not updates
        row['rejectionReason']='Scenario metadata overlay length mismatch during native preparation; zero executed decisions.'
    final_checkpoint=directory/'latest_training.pt'
    if record['status']=='training_complete' and final_checkpoint.exists():
        saved=torch.load(final_checkpoint,map_location='cpu',weights_only=False)
        assert saved['update_count']==completed
        row['checkpointOptimizerUpdateCountVerified']=True
        if 'collector' in saved:
            assert saved['collector']['transitions']==decisions
            assert saved['collector']['episode_count']==episodes
            row['checkpointCollectorCountsVerified']=True
        else:
            row['checkpointCollectorCountsVerified']=False
            row['legacyCheckpointNote']='Legacy checkpoint contains optimizer state but no collector snapshot; transition/episode totals are cross-checked between final report and per-update records.'
        row['sources'].append(bound(final_checkpoint))
    if (directory/'selected_policy/policy_checkpoint.pt').exists():
        row['selectedCheckpoint']=bound(directory/'selected_policy/policy_checkpoint.pt')
    attempts.append(row)

totals={key:sum(row[key] for row in attempts) for key in (
    'completedPpoRolloutUpdates','completedOptimizerMinibatchSteps',
    'decisionsUsedByCompletedUpdates','executedRejectedDecisions',
    'completedTrainingEpisodes','completedTrainingEpisodeForcePasses','trainingPhysicalFailures',
    'completedSelectionRecordings')}
totals['allExecutedTrainingDecisions']=totals['decisionsUsedByCompletedUpdates']+totals['executedRejectedDecisions']
totals['attempts']=len(attempts)
totals['completedTrainingAttempts']=sum(row['status']=='training_complete' for row in attempts)
totals['preparationRejections']=sum(row['status']=='preparation_rejection' for row in attempts)
totals['postCollectionSoftwareRejections']=sum(row['status']=='software_rejection' for row in attempts)
ledger={
    'status':'complete','createdUTC':datetime.now(timezone.utc).isoformat(),
    'scope':'Root-owned PPO attempts during the three-hour budget. Excludes ES searches, runner acceptance/smoke tests, discarded plans and held-out qualification.',
    'definitions':{
        'ppoRolloutUpdate':'One collected batch followed by PPO optimization; may contain several optimizer minibatch steps.',
        'decisions':'One physical action decision in one environment; all actual decision intervals and model versions are retained per attempt.',
        'episodes':'Completed training episodes only; partial active episodes remain in final checkpoints and are not added to this count.',
        'selectionRecordings':'Repeated completed recordings on reused selection banks, not distinct or held-out cases.',
        'comparability':'Banks, initialization, action scales, reward objectives and ranking rules changed between phases; ranks are meaningful within their recorded contracts.'},
    'totals':totals,'attempts':attempts,
    'finalConclusion':'The four band-aware completed PPO lanes kept their own starting checkpoint. Their last evaluated checkpoints reduced mean RMS but worsened worst force error and shared score. Earlier PPO lanes improved their force-only rank but did not pass every selection case. No PPO checkpoint superseded the selected ES candidate.',
}
(HERE/'ledger.json').write_text(json.dumps(ledger,indent=2)+'\n')
print(json.dumps(totals,indent=2))
