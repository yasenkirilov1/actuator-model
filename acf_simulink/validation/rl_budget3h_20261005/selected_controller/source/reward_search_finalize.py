"""Request timely owned-run stops, then bind completed ES evidence for review."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,json,subprocess,sys,time,hashlib

HERE=Path(__file__).resolve().parent
BASE=HERE.parents[1]/'validation/rl_budget3h_20261005'
RUNS=['reward_search_direct_8201','reward_search_recurrent_8202','reward_search_band_8203',
      'reward_search_sustained_8204','reward_search_current_8205','reward_search_inverse_8207',
      'reward_search_inverse_seed_8209']
ACTIVE=RUNS[-2:]
def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda:handle.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()
def read(path):return json.loads(path.read_text())

def manifest():
    runs=[]
    for name in RUNS:
        folder=BASE/name
        if not (folder/'results.json').exists():raise RuntimeError(f'{name} has no final audited result')
        result=read(folder/'results.json');latest=read(folder/'latest_selected_checkpoint.json')
        selected=Path(latest['directory'])
        assert digest(selected/'policy_checkpoint.pt')==latest['checkpointSha256']
        files=[p for p in selected.iterdir() if p.is_file()]
        files.extend(p for p in (folder/'initial_policy').glob('*') if p.is_file())
        files.extend(p for p in (folder/'frozen_sources').glob('*') if p.is_file())
        files.extend(folder.glob('generation_*.npz'))
        files.extend(p for p in (folder/'failed_training_native').rglob('*') if p.is_file())
        files.extend(folder/p for p in ['results.json','initial.json','selection.json','generations.jsonl','parameter_spec.json','compile_manifest.json','run_configuration.json','latest_selected_checkpoint.json','consumed_scenarios.json','reward_search_native.so','best_flat_weights.npy','selected_flat_weights.npy'] if (folder/p).exists())
        files=sorted(set(files))
        row={'run':name,'status':result['status'],'selectedCheckpoint':latest,
          'files':[{'path':str(p.resolve()),'bytes':p.stat().st_size,'sha256':digest(p)} for p in files]}
        final=result.get('finalTraining')
        if final:row['finalTrainingAudit']={k:v for k,v in final.items() if k!='rows'}
        rows=result.get('selectedSelection')
        if rows:row['finalSelectedSelectionAudit']={k:v for k,v in rows.items() if k!='rows'}
        runs.append(row)
    global_files=list(HERE.glob('reward_search*'))
    global_files.extend(HERE/p for p in ['policy.py','policy_export.py'])
    global_files.extend(BASE/p for p in ['reward_search_completed_ledger.json','reward_search_auxiliary_ledger.json'])
    global_files.extend(p for name in ['native_bank','native_bank_band','native_bank_sustained'] for p in (BASE/name).glob('*.json'))
    auxiliary=[]
    for name in ['reward_search_inverse_probe_8206','reward_search_inverse_selection_8206',
      'reward_search_inverse_followup_8206','reward_search_rolloff_probe_8208',
      'reward_search_inverse_gate_figure','reward_search_progress_figure']:
        paths=sorted(p for p in (BASE/name).rglob('*') if p.is_file())
        auxiliary.append({'name':name,'files':[{'path':str(p.resolve()),
          'bytes':p.stat().st_size,'sha256':digest(p)} for p in paths]})
    global_files=[p for p in global_files if p.is_file()]
    best=min((r['selectedCheckpoint'] for r in runs if r['run'] in ACTIVE),key=lambda x:x['rank'])
    return {'status':'completed_es_sources_and_evidence_bound','generatedUtc':datetime.now(timezone.utc).isoformat(),
      'scope':'C-native evolutionary policies and their development evidence only. No final-holdout results or report promotion are included.',
      'runs':runs,'auxiliaryArtifacts':auxiliary,'leadingDevelopmentCheckpoint':best,
      'files':[{'path':str(p.resolve()),'bytes':p.stat().st_size,'sha256':digest(p)} for p in global_files]}

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--stop-at-utc',default='2026-10-05T15:14:00+00:00');parser.add_argument('--once',action='store_true');a=parser.parse_args()
    if not a.once:
        target=datetime.fromisoformat(a.stop_at_utc).timestamp()
        while time.time()<target:time.sleep(min(30,target-time.time()))
        for name in ACTIVE:
            (BASE/name/'STOP').write_text('Graceful timed stop: complete current generation and final audits before the 15:20 UTC winner lock.\n')
        print('STOP requests written '+datetime.now(timezone.utc).isoformat(),flush=True)
        deadline=datetime.fromisoformat('2026-10-05T15:24:00+00:00').timestamp()
        while not all((BASE/name/'results.json').exists() for name in ACTIVE):
            if time.time()>deadline:raise TimeoutError('Final audits did not finish by15:24 UTC; preserve checkpoints and inspect live jobs.')
            time.sleep(15)
    subprocess.run([sys.executable,str(HERE/'reward_search_ledger.py'),*[str(BASE/n) for n in RUNS],
      '--output',str(BASE/'reward_search_completed_ledger.json')],check=True)
    subprocess.run([sys.executable,str(HERE/'reward_search_auxiliary_ledger.py')],check=True)
    subprocess.run(['/usr/bin/python',str(HERE/'reward_search_progress_plot.py')],check=True)
    d=manifest();(BASE/'reward_search_final_source_manifest.json').write_text(json.dumps(d,indent=2)+'\n')
    print(json.dumps({'status':d['status'],'leadingDevelopmentCheckpoint':d['leadingDevelopmentCheckpoint']},indent=2),flush=True)

if __name__=='__main__':main()
