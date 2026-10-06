"""Audit executed, scored ES work without counting planned or inflight work."""
from pathlib import Path
import argparse,json,hashlib
import numpy as np


def load(path):return json.loads(Path(path).read_text())
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ancestry(folder):
    """Bind the known experimental warm starts to exact retained bytes."""
    sibling=folder.parent
    chain={
      'reward_search_direct_8201':[],
      'reward_search_recurrent_8202':[('reward_search_direct_8201',10)],
      'reward_search_band_8203':[('reward_search_direct_8201',15)],
      'reward_search_sustained_8204':[('reward_search_direct_8201',15),('reward_search_band_8203',5)],
    }.get(folder.name)
    if chain is None:return {'status':'no_explicit_ancestry_mapping'}
    parents=[]
    for run,generation in chain:
        parent=sibling/run
        checkpoint=parent/f'checkpoints/generation_{generation:04d}'
        logs=[json.loads(x) for x in (parent/'generations.jsonl').read_text().splitlines()]
        episodes=0
        for g in logs:
            if g['generation']>generation:continue
            if 'candidateResults' in g:episodes+=sum(map(len,g['candidateResults']))
            else:
                arrays=np.load(parent/f"generation_{g['generation']:04d}.npz")
                episodes+=len(arrays['candidates'])*len(g['trainingBest']['rows'])
        parents.append({'run':run,'generation':generation,
          'checkpointSha256':digest(checkpoint/'policy_checkpoint.pt'),
          'flatWeightsSha256':digest(checkpoint/'policy_weights.f32'),
          'completedOptimizerGenerationsThroughCheckpoint':generation,
          'trainingEpisodesThroughCheckpoint':episodes})
    return {'status':'explicit_retained_checkpoint_chain','parents':parents,
      'ancestralTrainingEpisodes':sum(p['trainingEpisodesThroughCheckpoint'] for p in parents),
      'initialization':('Fresh Puffer network with stated static force-balance coefficients; no teacher labels.'
         if not parents else 'Checkpoint warm start; the band run additionally initializes a smooth robot-band neural feature and causal issued-command cancellation.'),
      'scope':'Ancestry counts are subsets of the executed run ledger, never additional episodes.'}

def ledger(folder):
    folder=Path(folder)
    results=load(folder/'results.json') if (folder/'results.json').exists() else {}
    logs=[json.loads(x) for x in (folder/'generations.jsonl').read_text().splitlines()] if (folder/'generations.jsonl').exists() else []
    selection=load(folder/'selection.json') if (folder/'selection.json').exists() else []
    initial=load(folder/'initial.json') if (folder/'initial.json').exists() else None
    layout=load(folder/'initial_policy/policy_weights.json')
    export=load(folder/'initial_policy/policy_export_manifest.json')
    train_episodes=0;train_decisions=0.;train_samples=0.;training_failures=0
    train_counters_complete=True;training_scenarios=set();groups=[]
    failed_replays={}
    old_manifest=folder/'failed_training_native/replay_manifest.json'
    if old_manifest.exists():
        for candidate in load(old_manifest)['rows']:
            failed_replays[(1,int(candidate['candidate']))]=candidate['rows']
    for g in logs:
        generation=int(g['generation']);snapshot=folder/f'generation_{generation:04d}.npz'
        data=np.load(snapshot);population=len(data['candidates'])
        rowslist=g.get('candidateResults')
        if rowslist is not None:
            assert len(rowslist)==population
            for rows in rowslist:
                train_episodes+=len(rows)
                train_decisions+=sum(r['controllerDecisions'] for r in rows)
                train_samples+=sum(r['nativeSamples'] for r in rows)
                training_failures+=sum(r['physicalFailure'] for r in rows)
                training_scenarios.update(r['scenario'] for r in rows)
            groups.append({'generation':generation,'population':population,'cases':g['trainingCases'],'episodeEvaluations':sum(map(len,rowslist))})
        else:
            case_rows=g['trainingBest']['rows'];case_ids=[r['scenario'] for r in case_rows]
            train_episodes+=population*len(case_ids);training_scenarios.update(case_ids)
            # The short direct run has deterministic full0.8s horizons except
            # explicitly replayed physically failed candidates. Each failed
            # replay matches its saved scalar objective exactly.
            normal_decisions={r['scenario']:r['controllerDecisions'] for r in case_rows}
            all_success=g['trainingBest']['physicalFailures']==0
            for ci,cost in enumerate(data['objectives']):
                if cost<1e4 and all_success:
                    train_decisions+=sum(normal_decisions.values())
                    train_samples+=sum(r['nativeSamples'] for r in case_rows)
                elif (generation,ci) in failed_replays and all_success:
                    failed={r['scenario']:r for r in failed_replays[(generation,ci)]}
                    training_failures+=len(failed)
                    for r in case_rows:
                        if r['scenario'] in failed:
                            f=failed[r['scenario']];train_decisions+=f['controllerDecisions']
                            # Full failure recording count includes t0; preserve
                            # a separate counter coverage note instead of mixing.
                            train_counters_complete=False
                        else:
                            train_decisions+=r['controllerDecisions'];train_samples+=r['nativeSamples']
                else:train_counters_complete=False
            groups.append({'generation':generation,'population':population,'cases':case_ids,'episodeEvaluations':population*len(case_ids),'recoveryCounterSource':str(old_manifest) if old_manifest.exists() else None})
    selection_executed=0;selection_retained=0;selection_decisions=0.;selection_samples=0.;selection_failures=0
    selection_counter_complete=True
    for s in selection:
        if 'evaluatedProposals' in s:
            proposal_rows=[p['rows'] for p in s['evaluatedProposals']]
            expected=len(proposal_rows)
        else:
            proposal_rows=[s['rows']]
            expected=1 if s.get('generation')==0 or not results.get('trainingBatchSize',0) and 'proposal' not in s else 2
        selection_executed+=expected*len(s['rows'])
        selection_retained+=sum(len(x) for x in proposal_rows)
        if expected!=len(proposal_rows):selection_counter_complete=False
        for rows in proposal_rows:
            selection_decisions+=sum(r['controllerDecisions'] for r in rows)
            selection_samples+=sum(r['nativeSamples'] for r in rows)
            selection_failures+=sum(r['physicalFailure'] for r in rows)
    audits=[]
    if initial:audits.append({'name':'initial_training_audit','report':initial})
    if 'finalTraining' in results:audits.append({'name':'final_training_audit','report':results['finalTraining']})
    if results.get('status')=='search_complete' and 'selectedSelection' in results:audits.append({'name':'final_selected_selection_audit','report':results['selectedSelection']})
    audit_summary=[{'name':x['name'],'episodes':len(x['report']['rows']),'decisions':sum(r['controllerDecisions'] for r in x['report']['rows']),'nativeSamples':sum(r['nativeSamples'] for r in x['report']['rows']),'physicalFailures':sum(r['physicalFailure'] for r in x['report']['rows'])} for x in audits]
    failed_native=[]
    for p in sorted((folder/'failed_training_native').glob('*/failures.json')) if (folder/'failed_training_native').exists() else []:
        r=load(p);failed_native.extend(r['rows'])
    old_replay_count=load(old_manifest)['replayEpisodesExcludedFromTrainingCount'] if old_manifest.exists() else 0
    selected=load(folder/'latest_selected_checkpoint.json') if (folder/'latest_selected_checkpoint.json').exists() else None
    row={'run':folder.name,'status':results.get('status','running'),'completedGenerations':len(logs),
      'optimizedDegrees':len(np.load(folder/f"generation_{logs[0]['generation']:04d}.npz")['candidates'][0]) if logs else None,
      'storedTrainableNNParameters':export['parameter_count'],'storedFloatsIncludingFixedNormalization':layout['float_count'],
      'trainingEpisodeEvaluationsScored':train_episodes,'trainingActorDecisionsScored':int(train_decisions),
      'trainingPhysicalFailures':training_failures,'trainingDistinctScenarios':sorted(training_scenarios),
      'trainingNativeSamplesRetained':int(train_samples),'trainingNativeCounterComplete':train_counters_complete,
      'selectionEpisodeEvaluationsExecuted':selection_executed,'selectionEpisodeMetricsRetained':selection_retained,
      'selectionActorDecisionsRetained':int(selection_decisions),'selectionNativeSamplesRetained':int(selection_samples),
      'selectionPhysicalFailuresInRetainedMetrics':selection_failures,'selectionCounterCoverageComplete':selection_counter_complete,
      'audits':audit_summary,'failedNativeRecoveryEpisodes':len(failed_native)+old_replay_count,
      'failedNativeRecoveryIncludedInTraining':False,
      'retainedScoredEpisodeEvaluations':train_episodes+selection_executed+sum(x['episodes'] for x in audit_summary),
      'wallTime_s':results.get('elapsed_s',logs[-1]['elapsed_s'] if logs else None),
      'wallTimeScope':'Complete process including audits when results.elapsed_s is present; otherwise last fully scored generation.',
      'selectedCheckpoint':selected,'ancestry':ancestry(folder),'exportProvenance':layout.get('provenance',{}),'generations':groups,
      'inflightExcluded':results.get('inFlightAtInterruption',None),
      'counterScope':'Native samples are interval scoring samples; full history includes separate endpoint conventions. Only retained counters are summed; unrecorded losing selection proposals are not assumed to complete full horizons.',
      'sourceHashes':{p.name:digest(p) for p in [folder/'generations.jsonl',folder/'selection.json',folder/'initial.json'] if p.exists()}}
    reported=results.get('episodeEvaluations',results.get('scoredEpisodeEvaluations'))
    if reported is not None:
        assert row['retainedScoredEpisodeEvaluations']==reported,(folder,reported,row['retainedScoredEpisodeEvaluations'])
        row['recordedEpisodeCounterMatchesAudit']=True
    row['trainingActorCounterSource']=('Per-candidate retained statistics.' if all('candidateResults' in g for g in logs)
        else 'Completed candidate objectives establish full horizons for successful episodes; exact failed-candidate native replays supply interrupted actor counts.')
    return row


def main():
    a=argparse.ArgumentParser();a.add_argument('runs',nargs='+');a.add_argument('--output',required=True);args=a.parse_args()
    rows=[ledger(p) for p in args.runs]
    d={'status':'audited_retained_scored_work','runs':rows,'trainingEpisodes':sum(r['trainingEpisodeEvaluationsScored'] for r in rows),'trainingActorDecisions':sum(r['trainingActorDecisionsScored'] for r in rows),'notes':['Optimization, selection, audit and recovery work remain separate.','No planned generation or unscored interrupted work is counted as an executed update.','Some historical losing selection proposal counters were not retained; exact weights, native library and deterministic seeds remain available for replay.']}
    Path(args.output).write_text(json.dumps(d,indent=2)+'\n');print(json.dumps({k:v for k,v in d.items() if k!='runs'},indent=2))

if __name__=='__main__':main()
