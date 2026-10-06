"""Freeze the budget candidate registry first, then audit its separate work ledger."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,time
from reward_search_ledger import ledger

BASE=Path(__file__).resolve().parents[2]/'validation/rl_budget3h_20261005'
RUN=BASE/'reward_search_budget_8211'
def read(p):return json.loads(p.read_text())
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def write(p,d):p.write_text(json.dumps(d,indent=2)+'\n')
def totals(rows):return {'episodes':len(rows),'actorDecisions':int(sum(r['controllerDecisions'] for r in rows)),
    'nativeScoringSamples':int(sum(r['nativeSamples'] for r in rows)),
    'physicalFailures':sum(bool(r['physicalFailure']) for r in rows)}

def registry():
    bootstrap=BASE/'reward_search_budget_bootstrap_8210/results.json'
    data=read(bootstrap);candidates=[]
    for row in data['comparisons']:
        if row['split']!='selection':continue
        directory=Path(row['policyDirectory']);checkpoint=directory/'policy_checkpoint.pt'
        candidates.append({'name':row['name'],'method':'Reward-selected linearized-DC-preserving history attenuation of previously evolved weights; no new CMA update.',
          'directory':str(directory),'checkpoint':str(checkpoint),'checkpointSha256':sha(checkpoint),
          'flatWeightsSha256':sha(directory/'policy_weights.f32'),'rank':row['rank'],
          'selectionRecordings':row['recordings'],'selectionEvidence':str(bootstrap),
          'selectionEvidenceSha256':sha(bootstrap),'selectionEntry':{'name':row['name'],'split':'selection'},
          'sourceDirectory':str(BASE/'executors/v7_budget')})
    selected=read(RUN/'latest_selected_checkpoint.json')
    selection_file=RUN/'selection.json';selection=read(selection_file)
    row=next(r for r in selection if r['generation']==selected['generation'])
    directory=Path(selected['directory'])
    candidates.append({'name':'budget_cma_8211_selected','method':'Reward-only constrained CMA refinement, 37 independent directions; initial generation zero is inherited bootstrap if selected.',
      'directory':str(directory),'checkpoint':str(directory/'policy_checkpoint.pt'),
      'checkpointSha256':selected['checkpointSha256'],'flatWeightsSha256':sha(directory/'policy_weights.f32'),
      'rank':selected['rank'],'generation':selected['generation'],'selectionRecordings':row['recordings'],
      'selectionEvidence':str(selection_file),'selectionEvidenceSha256':sha(selection_file),
      'selectionEntry':{'generation':selected['generation']},'sourceDirectory':str(BASE/'executors/v7_budget')})
    for c in candidates:assert c['selectionRecordings']==28 and sha(Path(c['checkpoint']))==c['checkpointSha256']
    chosen=min(candidates,key=lambda c:c['rank'])
    screen=read(BASE/'reward_search_budget_screen_8210/results.json')
    baseline=next(r for r in screen['comparisons'] if r['controller']=='native_baseline')
    result={'status':'budget_selection_locked_before_final_tests','lockedUtc':datetime.now(timezone.utc).isoformat(),
      'nativeLibrary':str(BASE/'native_bank_budget50/libacf_native.so'),
      'nativeLibrarySha256':sha(BASE/'native_bank_budget50/libacf_native.so'),
      'sourceDirectory':str(BASE/'executors/v7_budget'),'sourceManifestSha256':sha(BASE/'executors/v7_budget/frozen_source_manifest.json'),
      'candidates':candidates,'selectedLearnedController':chosen,
      'baselineSelection':{k:baseline[k] for k in ['rank','recordings','passed','physicalFailures','worstNormalForceError_N','meanRmsNormalForceError_N']},
      'scope':'All candidates ranked on the same 28 budget development cases. No final-case outcomes were inspected. The original checkpoint and provenance are preserved.'}
    write(BASE/'reward_search_budget_candidate_registry.json',result)
    print(json.dumps({'registry':str(BASE/'reward_search_budget_candidate_registry.json'),'selected':chosen},indent=2),flush=True)
    return result

def main():
    while not (RUN/'selected_policy/policy_checkpoint.pt').exists():time.sleep(5)
    reg=registry()
    while not (RUN/'results.json').exists():time.sleep(5)
    es=ledger(RUN)
    fast=BASE/'reward_search_inverse_8207/checkpoints/generation_0025/policy_checkpoint.pt'
    es['ancestry']={'status':'explicit_budget_warm_start','immediateCheckpoint':str(BASE/'reward_search_budget_bootstrap_8210/history_0p25/policy_checkpoint.pt'),
      'immediateCheckpointSha256':sha(BASE/'reward_search_budget_bootstrap_8210/history_0p25/policy_checkpoint.pt'),
      'fastSensorAncestor':str(fast),'fastSensorAncestorSha256':sha(fast),
      'fastStageLedger':str(BASE/'reward_search_completed_ledger.json'),'fastStageLedgerSha256':sha(BASE/'reward_search_completed_ledger.json'),
      'scope':'Ancestral episodes are already counted in the sealed fast-stage ledger; never add them twice.'}
    screen_path=BASE/'reward_search_budget_screen_8210/results.json';screen=read(screen_path)
    boot_path=BASE/'reward_search_budget_bootstrap_8210/results.json';boot=read(boot_path)
    screen_rows=[r for comparison in screen['comparisons'] for r in comparison['rows']]
    training=[r for comparison in boot['comparisons'] if comparison['split']=='train' for r in comparison['rows']]
    selection=[r for comparison in boot['comparisons'] if comparison['split']=='selection' for r in comparison['rows']]
    for rows in [screen_rows,training,selection]:assert not {r['scenario'] for r in rows}.intersection(range(64,96))
    result={'status':'audited_budget_stage_only','evolutionaryRun':es,
      'pairedBaselinePolicyScreen':totals(screen_rows),'initializationTrainingComparisons':totals(training),
      'initializationSelectionComparisons':totals(selection),
      'recoveryEpisodesExcludedFromOptimization':screen['recoveryEpisodesExcluded']+boot['recoveryEpisodes']+es['failedNativeRecoveryEpisodes'],
      'selectionRegistry':str(BASE/'reward_search_budget_candidate_registry.json'),
      'threadAuditPerformance':'Native mutex serializes threaded episode stepping; numeric parity is accepted, no audit speedup is claimed.',
      'sourceEvidence':[{'path':str(p),'sha256':sha(p)} for p in [screen_path,boot_path,RUN/'results.json',BASE/'reward_search_completed_ledger.json']],
      'scope':'Only the newly executed budget-sensor work is counted here. The sealed fast-sensor ledger and manifest are unchanged; final tests are outside this ledger.'}
    write(BASE/'reward_search_budget_ledger.json',result)
    groups=[]
    for name in ['reward_search_budget_screen_8210','reward_search_budget_bootstrap_8210','reward_search_budget_8211','executors/v7_budget']:
        files=sorted(p for p in (BASE/name).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
        groups.append({'name':name,'files':[{'path':str(p.resolve()),'bytes':p.stat().st_size,'sha256':sha(p)} for p in files]})
    manifest={'status':'budget_sources_and_evidence_bound','groups':groups,'selectedCheckpoint':reg['selectedLearnedController'],
      'ledgerSha256':sha(BASE/'reward_search_budget_ledger.json'),'registrySha256':sha(BASE/'reward_search_budget_candidate_registry.json'),
      'fastStageManifestSha256':sha(BASE/'reward_search_final_source_manifest.json')}
    write(BASE/'reward_search_budget_source_manifest.json',manifest)
    print(json.dumps({'status':result['status'],'cmaGenerations':es['completedGenerations'],
      'cmaTrainingEpisodes':es['trainingEpisodeEvaluationsScored'],'selectionRegistry':result['selectionRegistry']},indent=2),flush=True)

if __name__=='__main__':main()
