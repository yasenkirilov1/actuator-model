"""Executed reward comparisons and verification, separate from CMA updates."""
from pathlib import Path
import json,hashlib
import numpy as np

BASE=Path(__file__).resolve().parents[2]/'validation/rl_budget3h_20261005'
def read(path):return json.loads(path.read_text())
def checksum(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def totals(rows):
    return {'episodes':len(rows),'actorDecisions':int(sum(r['controllerDecisions'] for r in rows)),
      'nativeScoringSamples':int(sum(r['nativeSamples'] for r in rows)),
      'physicalFailures':sum(bool(r['physicalFailure']) for r in rows)}

comparisons=[]
for name in ['reward_search_inverse_probe_8206','reward_search_inverse_followup_8206','reward_search_rolloff_probe_8208']:
    source=BASE/name/'results.json';d=read(source)
    assert d['status']=='complete'
    train=[];selection=[]
    for r in d['comparisons']:
        (selection if r.get('split')=='selection' else train).extend(r['rows'])
    assert len(train)+len(selection)==d['episodeComparisons']
    comparisons.append({'run':name,'trainingComparison':totals(train),
      'selectionComparison':totals(selection),'optimizerUpdates':0,
      'source':str(source),'sourceSha256':checksum(source),'scope':'Deterministic mechanics initialization chosen by complete native episode rewards.'})
source=BASE/'reward_search_inverse_selection_8206/initial.json';d=read(source)
comparisons.append({'run':'reward_search_inverse_selection_8206','trainingComparison':totals([]),
  'selectionComparison':totals(d['rows']),'optimizerUpdates':0,
  'source':str(source),'sourceSha256':checksum(source),'scope':'Rejected full-inverse selection comparison.'})

verification=[]
for name in ['reward_search_benchmark','reward_search_direct_dev12_g5','reward_search_band_smoke']:
    source=BASE/name/'initial.json';d=read(source)
    verification.append({'run':name,**totals(d['rows']),'source':str(source),'sourceSha256':checksum(source)})
for name in ['reward_search_band_proxy_check','reward_search_band_proxy_positive']:
    source=BASE/name/'results.json';d=read(source)
    verification.append({'run':name,**totals([d['row']]),'source':str(source),'sourceSha256':checksum(source)})
for name in ['reward_search_parity','reward_search_v1_parity_fixed']:
    folder=BASE/name;actions=list(folder.glob('*/*actions.npz'));natives=list(folder.glob('*/*native.npz'))
    assert len(actions)==len(natives)
    verification.append({'run':name,'episodes':len(actions),
      'actorDecisions':sum(len(np.load(p)['history']) for p in actions),
      'fullNativeHistoryPoints':sum(len(np.load(p)['time']) for p in natives),
      'source':str(folder/'results.json'),'sourceSha256':checksum(folder/'results.json'),
      'scope':'Both C and Python-loop trajectories counted separately; this is implementation verification.'})
source=BASE/'reward_search_parallel_audit_check/results.json'
if source.exists():
    d=read(source)
    verification.append({'run':'reward_search_parallel_audit_check','episodes':d['episodes'],
      'actorDecisions':d['actorDecisions'],'source':str(source),'sourceSha256':checksum(source),
      'scope':'Serial-versus-threaded complete-episode statistics; no optimizer update.'})

replays=[]
for relative in ['reward_search_inverse_selection_8206/failed_native/replay.json',
 'reward_search_inverse_followup_8206/inverse_0p6/train_failed_native/metrics.json',
 'reward_search_inverse_followup_8206/inverse_0p3/sustained_gate_replay/replay.json']:
    source=BASE/relative;d=read(source)
    replays.append({'source':str(source),**totals(d['rows']),'sourceSha256':checksum(source),
       'includedInComparisonCounts':False})

report={'status':'audited_executed_auxiliary_work','comparisons':comparisons,
  'trainingComparisonEpisodes':sum(x['trainingComparison']['episodes'] for x in comparisons),
  'selectionComparisonEpisodes':sum(x['selectionComparison']['episodes'] for x in comparisons),
  'comparisonActorDecisions':sum(x[k]['actorDecisions'] for x in comparisons for k in ['trainingComparison','selectionComparison']),
  'comparisonPhysicalFailures':sum(x[k]['physicalFailures'] for x in comparisons for k in ['trainingComparison','selectionComparison']),
  'verification':verification,'verificationEpisodes':sum(x['episodes'] for x in verification),
  'verificationActorDecisions':sum(x['actorDecisions'] for x in verification),
  'nativeRecoveryAndFigureAudits':replays,'nativeRecoveryAndFigureEpisodes':sum(x['episodes'] for x in replays),
  'softwareRejections':[{'source':str(BASE/'reward_search_v1_parity/software_rejection.json'),**read(BASE/'reward_search_v1_parity/software_rejection.json')}],
  'scope':'Additional to the CMA ledger. No final holdout cases; no optimizer updates are claimed for these comparisons. Counts represent retained executed records, not planned work. Native interval counters and full-history endpoint counts are named separately.'}
target=BASE/'reward_search_auxiliary_ledger.json';target.write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if isinstance(v,(int,str))},indent=2))
