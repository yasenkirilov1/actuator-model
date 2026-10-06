"""Freeze the selection-only winner before any fresh qualification runs.

This administrative recipe reads completed development evidence, copies the
winning export and accepted executor, and prepares three cohort-bound reviews.
It neither trains a policy nor executes a physical case.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study', type=Path, required=True)
    parser.add_argument('--candidates', nargs='*', default=[])
    parser.add_argument('--candidate-registry', type=Path,
                        help='Immutable completed selection registry; audits may finish separately')
    parser.add_argument('--cohort-spec', type=Path,
                        help='Prepared cohort banks/native builds; paths relative to study')
    parser.add_argument('--source-dir', type=Path,
                        help='Accepted frozen runtime; default executors/v6')
    parser.add_argument('--bootstrap-results', type=Path,
                        help='Completed budget history-strength comparison, selection rows only')
    args=parser.parse_args()
    study=args.study.resolve()
    assert all(not (study/name).exists() for name in ('final_holdout32','final_nominal700','final_long4'))
    band=read(study/'selection_contract_band_v2.json')
    expected=sorted(band['selection_ids'])
    candidates=[]
    registry_evidence=[]
    if args.candidate_registry:
        registry=read(args.candidate_registry)
        assert registry['status']=='budget_selection_locked_before_final_tests'
        registry_evidence.append(args.candidate_registry.resolve())
        for item in registry['candidates']:
            checkpoint=Path(item['checkpoint'])
            assert sha(checkpoint)==item['checkpointSha256']
            evidence=Path(item['selectionEvidence'])
            assert sha(evidence)==item['selectionEvidenceSha256']
            document=read(evidence)
            rows=document if isinstance(document,list) else document['comparisons']
            match=[r for r in rows if all(r.get(k)==v for k,v in item['selectionEntry'].items()) and r.get('rank')==item['rank']]
            assert len(match)==1,(item['name'],len(match))
            selected=match[0]
            assert sorted(r['scenario'] for r in selected['rows'])==expected
            assert selected['recordings']==28
            directory=Path(item['directory'])
            run=directory.parent.parent.name if 'generation' in item else directory.parent.name
            candidates.append({'run':run,'candidate':item['name'],'generation':item.get('generation'),
                'method':'evolutionary_reward_search' if 'generation' in item else 'reward_selected_history_attenuation_of_evolved_policy',
                'directory':str(directory),'checkpointSha256':sha(checkpoint),'rank':item['rank'],
                'selection':{k:selected[k] for k in ('recordings','passed','physicalFailures','worstNormalForceError_N','meanRmsNormalForceError_N','jointScore')}})
            registry_evidence.append(evidence)
    for name in args.candidates:
        run=study/name
        metadata=read(run/'latest_selected_checkpoint.json')
        assert (run/'results.json').is_file(), 'Finish and audit the run before selection lock'
        result=read(run/'results.json')
        assert result['status'] in ('search_complete','search_stopped_after_completed_generation'), result['status']
        checkpoint=Path(metadata['directory'])/'policy_checkpoint.pt'
        assert sha(checkpoint)==metadata['checkpointSha256']
        selections=read(run/'selection.json')
        match=[r for r in selections if r.get('generation')==metadata['generation'] and r.get('rank')==metadata['rank']]
        assert len(match)==1,(name,len(match))
        selected=match[0]
        assert sorted(r['scenario'] for r in selected['rows'])==expected
        assert selected['recordings']==len(expected)==28
        candidates.append({'run':name,'generation':metadata['generation'],'method':'evolutionary_reward_search',
                           'directory':metadata['directory'],'checkpointSha256':sha(checkpoint),
                           'rank':metadata['rank'],
                           'selection':{k:selected[k] for k in ('recordings','passed','physicalFailures','worstNormalForceError_N','meanRmsNormalForceError_N','jointScore')}})
    if args.bootstrap_results:
        bootstrap=read(args.bootstrap_results)
        assert bootstrap['status']=='complete'
        for row in bootstrap['comparisons']:
            if row.get('split')!='selection':continue
            assert row['recordings']==28
            assert sorted(r['scenario'] for r in row['rows'])==expected
            checkpoint=Path(row['policyDirectory'])/'policy_checkpoint.pt'
            candidates.append({'run':args.bootstrap_results.parent.name,'candidate':row['name'],
                'generation':None,'method':'reward_selected_history_attenuation_of_evolved_policy',
                'directory':row['policyDirectory'],'checkpointSha256':sha(checkpoint),'rank':row['rank'],
                'selection':{k:row[k] for k in ('recordings','passed','physicalFailures','worstNormalForceError_N','meanRmsNormalForceError_N','jointScore')}})
    candidates.sort(key=lambda x:tuple(x['rank']))
    assert candidates
    winner=candidates[0]
    folder=study/'selected_controller';folder.mkdir(exist_ok=False)
    original=Path(winner['directory'])
    for file in original.iterdir():
        if file.is_file():shutil.copy2(file,folder/file.name)
    manifest=read(folder/'policy_export_manifest.json')
    for file,digest in manifest['files'].items():assert sha(folder/file)==digest,file
    source=folder/'source';shutil.copytree(args.source_dir or study/'executors/v6',source)
    source_manifest=read(source/'frozen_source_manifest.json')
    for file,digest in source_manifest['files'].items():assert sha(source/file)==digest,file
    for file in ('reward_search.py','reward_search_native.c'):
        if winner['method']=='evolutionary_reward_search':
            assert sha(source/file)==sha(study/winner['run']/'frozen_sources'/file)
    evidence=folder/'selection_evidence';evidence.mkdir()
    paths=[study/'selection_contract_band_v2.json',study/'study_contract.json',
           study/'reward_search_completed_ledger.json',study/'reward_search_auxiliary_ledger.json',
           study/'runner_checks/ppo_final_ledger_v1/ledger.json']
    for candidate in candidates:
        if args.candidate_registry:continue
        if candidate['method']!='evolutionary_reward_search':continue
        paths += [study/candidate['run']/name for name in
                  ('results.json','selection.json','latest_selected_checkpoint.json','run_configuration.json')]
    if args.bootstrap_results:paths.append(args.bootstrap_results.resolve())
    paths+=list(dict.fromkeys(registry_evidence))
    budget_ledger=study/'reward_search_budget_ledger.json'
    if budget_ledger.exists():paths.append(budget_ledger)
    evidence_rows=[]
    for index,path in enumerate(paths):
        target=evidence/f'{index:02d}_{path.parent.name}_{path.name}'
        shutil.copy2(path,target)
        evidence_rows.append({'file':str(target),'sha256':sha(target),'original':str(path)})
    timestamp=datetime.now(timezone.utc).isoformat()
    specification=read(args.cohort_spec) if args.cohort_spec else {
        'holdout32':{'bank':'scenario_bank_sustained/manifest.json','nativeBuild':'native_bank_sustained','evaluationSplit':'holdout','cases':32,'aggregateOnly':False,'workers':2},
        'nominal700':{'bank':'qualification_bank_700/manifest.json','nativeBuild':'native_qualification_700','evaluationSplit':'qualification','cases':700,'aggregateOnly':True,'workers':4},
        'long4':{'bank':'long_qualification_bank/manifest.json','nativeBuild':'native_long_qualification','evaluationSplit':'qualification','cases':4,'aggregateOnly':True,'workers':2},
    }
    assert set(specification)=={'holdout32','nominal700','long4'}
    selection_bank=study/specification['holdout32']['bank']
    selection=read(selection_bank)
    common={'selectedCheckpointSha256':winner['checkpointSha256'],
            'sourceModelSha256':selection['sourceModelSha256'],
            'trainingSourceModelSha256':selection['sourceModelSha256'],
            'trainingBankSha256':sha(selection_bank),
            'selectionBankSha256':sha(selection_bank),
            'selectionIDs':expected,'selectionOnly':True,'lockedUTC':timestamp,
            'selectedCandidate':winner,'comparedFinalists':candidates,
            'selectionEvidence':evidence_rows,
            'rule':'Lexicographic recorded common28-case ranking; no fresh outcomes inspected.',
            'finalWeightsMayNotBeRetunedAfterFreshEvaluation':True}
    cohorts={}
    for key,name in [('holdout32','holdout'),('nominal700','grid'),('long4','long')]:
        item=specification[key];bank=study/item['bank'];native=study/item['nativeBuild']
        assert read(bank)['sourceModelSha256']==common['sourceModelSha256']
        review=folder/f'selection_review_{name}.json'
        write(review,{**common,'evaluationBankSha256':sha(bank),
                                                   'evaluationBank':str(bank)})
        cohorts[key]={**item,'bankSha256':sha(bank),'nativeLibrarySha256':sha(native/'libacf_native.so'),
                      'nativeMetadataSha256':sha(native/'metadata.json'),
                      'nativeScenariosSha256':sha(native/'scenarios.json'),
                      'nativeBridgeBuildSha256':sha(native/'bridge_build.json'),
                      'selectionReview':str(review.relative_to(study)),
                      'selectionReviewSha256':sha(review)}
    write(folder/'cohort_manifest.json',{'selectedCheckpointSha256':winner['checkpointSha256'],
          'selectionBank':str(selection_bank.relative_to(study)),
          'selectionBankSha256':sha(selection_bank),'cohorts':cohorts})
    write(folder/'selection_lock.json',{'status':'locked_before_fresh_evaluation',
          'lockedUTC':timestamp,'winner':winner,'candidates':candidates,
          'sourceManifestSha256':sha(source/'frozen_source_manifest.json'),
          'files':{str(p.relative_to(folder)):sha(p) for p in sorted(folder.rglob('*')) if p.is_file()}})
    print(json.dumps({'status':'locked_before_fresh_evaluation','folder':str(folder),'winner':winner}))


if __name__=='__main__':main()
