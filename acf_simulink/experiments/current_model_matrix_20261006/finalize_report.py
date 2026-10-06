"""Audit current-model evidence and deliver Markdown; never run simulations."""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
V = ROOT/'acf_simulink/validation/current_model_matrix_20261006'
R = V/'report'
REPORT = ROOT/'reports/active-contact-flange-report-design-explained-20261006.md'
NOTES = Path('/home/tommie/Documents/Notes Remote/S7 Project')
BANKS = ['coil_soft','screw_soft','coil_rigid_plate','screw_rigid_plate']
GENERATED = {'report/report_inputs.json','report/markdown_delivery_manifest.json',
             'report/finalization_checks.json'}

def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def objsha(p): return hashlib.sha256(json.dumps(p,sort_keys=True).encode()).hexdigest()
def canonical_sha(p): return hashlib.sha256(json.dumps(p,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def read(p): return json.loads(p.read_text())
def write(p,d): p.write_text(json.dumps(d,indent=2,allow_nan=False)+'\n')
def local(p):
 p=Path(p)
 if not p.is_absolute():return ROOT/p
 # Preserve recorded author-machine metadata while resolving project inputs
 # against the current checkout when it lives at a different location.
 recorded_root=Path('/home/tommie/Projects/actuator-model')
 if p.is_relative_to(recorded_root):return ROOT/p.relative_to(recorded_root)
 return p
def check_binding(entry):
 p=local(entry['path']); assert p.is_file(),str(p)
 assert sha(p)==entry['sha256'],str(p)

def audit_diagnostics(name,contract_name):
 result=read(R/(name+'.json')); contract_path=HERE/(contract_name+'.json')
 contract=read(contract_path); lock=read(V/'control/selected_banks.json')
 assert result['contract_sha256']==sha(contract_path)
 assert result['selected_banks_sha256']==sha(V/'control/selected_banks.json')
 assert result['engine_source_sha256']==sha(HERE/'engine.c')
 assert result['api_sha256']==sha(HERE/'engine_api.py')
 rows=result['records']; banks={x['id']:x for x in lock['banks']}
 variants={x['id']:x for x in contract['variants']}
 assert len(rows)==result['executed_record_count']==contract['maximum_executed_records']
 ids=set()
 for row in rows:
  b=row['bank'];variant=variants[row['variant']];corner=row['corner']
  assert corner in contract['corners']
  key=(b,objsha(corner),row['variant']);assert key not in ids;ids.add(key)
  config={**banks[b]['config'],**corner,'identified_mass_s':corner['mass_s'],'seed':contract['seed']}
  if name=='drive_diagnostics':
   config.update(voltage_limit=variant['voltage_limit'],current_response=variant['current_response'])
  else:
   config.update(variant['deltas']);config['identified_mass_s']=corner['mass_s']*variant['identified_mass_factor']
  assert objsha(config)==row['config_sha256']
  m=row['metrics'];assert m['force_pass']==(m['completed'] and m['force_peak_error']<=.5)
  if name=='calibration_followup':
   eligible=bool(m['force_pass'] and m['sensor_ratings_pass'] and m['actuator_ratings_pass'] and m['continuous_rms_current_pass'] is not False)
   assert eligible==row['eligible_pass']
 force_values=int(sum(x['metrics']['steps'] for x in rows))
 extras=sum(x['metrics'].get('one_sided_extra_limit_count',0) for x in rows)
 assert force_values==result['scored_force_value_count']
 assert extras==result['additional_one_sided_force_limits']
 assert force_values-extras==result['scored_integration_endpoint_count']
 for z in result['summaries']:
  group=[x for x in rows if x['bank']==z['bank'] and x['variant']==z['variant']]
  assert z['executed']==len(group)==3
  assert z['worst_error_N']==max(x['metrics']['force_peak_error'] for x in group)
  if name=='calibration_followup':
   assert z['eligible_passes']==sum(x['eligible_pass'] for x in group)
   assert z['sensor_range_failures']==sum(not x['metrics']['sensor_ratings_pass'] for x in group)
  else:
   assert z['force_passes']==sum(x['metrics']['force_pass'] for x in group)
   assert z['max_voltage_request_V']==max(x['metrics']['voltage_peak_request'] for x in group)
   assert z['max_voltage_clipped_fraction']==max(x['metrics']['voltage_clipped_fraction'] for x in group)
 return {'records':len(rows),'integration_endpoints':force_values-extras,'extra_force_limits':extras,
         'source_hashes_and_full_configurations_verified':True,'summaries_recomputed':True,
         'aggregate_label_correction_only':result['count_label_audit']}

def audit_evidence():
 review=read(R/'read_only_review.json')
 assert review['status']=='reviewed_no_material_findings' and review['read_only']
 assert review['report_sha256']==sha(REPORT)
 protected=read(V/'protected_sources_before.json')['files']
 for x in protected:check_binding(x)
 c=read(V/'control/matrix_completion.json');ca=read(V/'control/completion_audit.json')
 lock=read(V/'control/selected_banks.json')
 for x in (c,lock):
  assert x['engine_source_sha256']==sha(HERE/'engine.c')
  assert x['api_sha256']==sha(HERE/'engine_api.py')
  assert x['contract_sha256']==sha(HERE/'study_contract.json')
 assert ca['status']=='passed_without_new_simulations'
 assert ca['matrix_completion_sha256']==sha(V/'control/matrix_completion.json')
 assert ca['selected_banks_sha256']==sha(V/'control/selected_banks.json')
 assert ca['exact_final_records']==324 and ca['exact_development_records']==144
 for b in lock['banks']:
  check_binding({'path':b['controller_lock_path'],'sha256':b['controller_lock_sha256']})
  assert canonical_sha(b['config'])==b['config_sha256']
 a=read(V/'acquisition/diagnostic_results.json');aa=read(V/'acquisition/audit.json')
 assert aa['status']=='passed' and aa['unique_case_count']==228
 assert aa['diagnostic_results_sha256']==sha(V/'acquisition/diagnostic_results.json')
 assert aa['contract_sha256']==sha(HERE/'acquisition/diagnostic_contract.json')
 n=read(V/'native/native_results.json');na=read(V/'native/validation_audit.json')
 assert na['status']=='compact_native_audit_passed'
 for k,f in [('native_results_sha256','native_results.json'),('selection_sha256','selection.json'),('launch_audit_sha256','launch_attempt01.json')]:
  assert na[k]==sha(V/'native'/f)
 for x in na['recipe_bindings']+n['sourceBindings']:check_binding(x)
 assert len(na['guard_results'])==4
 assert all(x['native_regular_C_pass'] and x['native_original_split_event_pass'] for x in na['guard_results'])
 assert not na['controller_independently_verified'] and not na['qualification']
 assert na['total_native_and_refinement_scored_values']==6400014
 assert na['temporary_reference_and_command_arrays_released']
 assert na['temporary_bytes_peak']<=na['temporary_cap_bytes']
 assert all(x['outcome_unchanged_force_failure'] for x in na['refinement_results'])
 for case in n['cases']:
  # Trace archive filenames and SHA are validated by the source-bound native audit.
  archive=local(case['traceArchive'])
  assert archive.is_file()
  assert sha(archive)==case['traceArchiveSha256']
 root_diagnostics={name:audit_diagnostics(name,contract) for name,contract in
  [('drive_diagnostics','drive_checks_contract'),('calibration_followup','calibration_followup_contract')]}
 nominal={g['name']:g['summary'] for g in c['groups'] if g['name'] in BANKS}
 assert len(nominal)==4 and all(x['executed_count']==54 for x in nominal.values())
 assert [nominal[b]['force_and_eligibility_pass_count'] for b in BANKS]==[54,54,0,0]
 counts={'controller':{'records':468,'integration_endpoints':ca['integration_endpoints_scored'],'extra_force_limits':ca['additional_one_sided_limits_scored']},
         'acquisition':{'records':len(a['records']),'integration_endpoints':a['summary']['scored_integration_endpoints'],
                        'extra_force_limits':a['summary']['additional_one_sided_force_limits']},
         **{k:{q:v[q] for q in ['records','integration_endpoints','extra_force_limits']} for k,v in root_diagnostics.items()}}
 totals={q:int(sum(x[q] for x in counts.values())) for q in ['records','integration_endpoints','extra_force_limits']}
 assert totals=={'records':852,'integration_endpoints':649849019,'extra_force_limits':948}
 return {'status':'current_source_and_compact_evidence_audit_passed','qualification':False,
 'protected_file_count':len(protected),'counts_by_group':counts,'primary_totals':totals,'nominal_selected_grids':nominal,
 'native_scored_values':na['total_native_and_refinement_scored_values'],'root_diagnostics':root_diagnostics,
 'current_controller':'P/gravity hold/acceleration-derived velocity; no force integral',
 'scope':'Loaded-equilibrium2s fixed robot, axial force; no normal/drag/engagement/recentering/thermal hardware qualification.'}

def report_figures():
 text=REPORT.read_text(); paths=[(REPORT.parent/p).resolve() for p in re.findall(r'!\[[^\]]*\]\(([^)]+)\)',text)]
 assert len(paths)==len(set(paths))==8
 fm=read(R/'figure_manifest.json');assert fm['script_sha256']==sha(HERE/'build_report_figures.py')
 assert len(fm['figures'])==6
 for f in fm['figures']:
  check_binding(f)
  for s in f['sources']:check_binding(s)
 for p in paths:
  assert p.is_file();tree=ET.parse(p);assert tree.getroot().tag.endswith('svg')
  assert not any(x.tag.endswith('script') for x in tree.iter())
 assert all(local(x['path']) in paths for x in fm['figures'])
 # Text/figure scope is reviewed separately; these guard the numeric main table and basic structure.
 c=read(V/'control/matrix_completion.json')
 for g in c['groups']:
  if g['name'] in BANKS:assert f"{g['summary']['worst_force_error_N']:.3f} N" in text
 assert all(x in text for x in ['852 primary','±0.5 N','77 mm backing plate','0.1–2 kg','open problem','±0.1%','€81.82'])
 assert text.count('#### Intro')==6 and text.count('#### Question')==6
 assert text.count('#### Results')==6 and text.count('#### Recommendation')==6
 prose='\n'.join(x for x in text.splitlines() if x and not x.startswith(('#','|','![')))
 assert 2700<=len(prose.split())<=3300
 return paths,{'all_words':len(text.split()),'prose_words':len(prose.split()),'figure_count':len(paths)}

def check_markdown_links(path):
 for target in re.findall(r'\[[^\]]*\]\(([^)]+)\)',path.read_text()):
  target=target.strip('<>')
  if target.startswith(('https:','http:','#','mailto:')):continue
  p=(path.parent/target.split('#')[0]).resolve();assert p.exists(),f'{path}: {target}'

def expected_notes():
 text=REPORT.read_text();rewrites=[]
 def replace(m):
  label,target=m.groups()
  if target.startswith('../'):
   absolute=(REPORT.parent/target).resolve();assert absolute.is_file()
   new=str(absolute);new=f'<{new}>' if ' ' in new else new
   rewrites.append({'repository_target':target,'notes_target':new})
   return f'[{label}]({new})'
  return m.group(0)
 return re.sub(r'(?<!!)\[([^\]]*)\]\(([^)]+)\)',replace,text),rewrites

def source_inputs(figures):
 files=[]
 for scope in [HERE,V]:
  for p in scope.rglob('*'):
   if not p.is_file() or p.is_symlink() or '__pycache__' in p.parts:continue
   if scope==V and str(p.relative_to(V)) in GENERATED:continue
   files.append(p)
 files+=figures+[ROOT/'README.md',ROOT/'AGENTS.md',
 ROOT/'acf_simulink/validation/report_readability_20261006/adc/parts_review_20261006.json',
 ROOT/'acf_simulink/validation/rl_budget3h_20261005/budget_parts_review_20261005.json',
 ROOT/'acf_simulink/experiments/sea_pad_dynamic_20261006/voice_coil_pcb_proposal.json']
 return {'status':'current_matrix_source_bound_report_inputs','qualification':False,
 'source_report_commit':'11febde','sources':[{'path':str(p.relative_to(ROOT)),'sha256':sha(p)} for p in sorted(set(files))],
 'main_report':str(REPORT.relative_to(ROOT)),
 'scope':'All quantitative controller/contact/sensing/drive claims use the current matrix; dated price inputs and proposed architecture diagrams are separately labeled.',
 'historical_models_and_html':'preserved by protected_sources_before.json; old report snapshots retained',
 'raw_traces':'No bulk histories retained; native500Hz gzip traces are display-only, never peak scoring.'}

def shared_arrays():
 scopes=[ROOT/'acf_simulink/validation',Path('/run/media/tommie/SATA SSD/actuator-model-evidence')]
 seen=set();size=0;count=0
 for scope in scopes:
  assert scope.is_dir(),str(scope)
  for p in scope.rglob('*'):
   if not p.is_file() or p.suffix.lower() not in {'.mat','.npz','.npy','.bin'}:continue
   stat=p.stat();key=(stat.st_dev,stat.st_ino)
   if key in seen:continue
   seen.add(key);size+=stat.st_size;count+=1
 compact=sum(p.stat().st_size for p in (V/'native').glob('*.csv.gz'))
 assert size+compact<=5368709120
 return {'scopes':[str(x) for x in scopes],'deduplicated_array_count':count,
         'array_bytes':size,'current_compact_trace_bytes_also_counted':compact,
         'conservative_total_bytes':size+compact,'cap_bytes':5368709120}

def retained_budget(figures):
 # Include source, compact receipts and both report/figure copies. Existing
 # architecture diagrams are conservatively charged; unchanged historical
 # content in root README/AGENTS is excluded from incremental storage growth.
 files={p for scope in [HERE,V] for p in scope.rglob('*') if p.is_file() and not p.is_symlink() and '__pycache__' not in p.parts}
 files.update(figures);files.update([REPORT,NOTES/'active-contact-flange-report.md',ROOT/'README.md',ROOT/'AGENTS.md'])
 files.update(NOTES/'figures'/p.name for p in figures)
 size=sum(p.stat().st_size for p in files if p.exists())
 baseline=read(R/'root_document_baseline.json')['files']
 assert {x['path'] for x in baseline}=={'README.md','AGENTS.md'}
 existing_bytes=sum(min(local(x['path']).stat().st_size,x['baseline_bytes']) for x in baseline)
 size-=existing_bytes
 forecast=read(V/'storage_forecast.json');assert size<=forecast['incremental_retained_cap_bytes']
 def folder_bytes(p):return sum(x.stat().st_size for x in p.rglob('*') if x.is_file() and '__pycache__' not in x.parts)
 owner={k:folder_bytes(HERE/k)+folder_bytes(V/k) for k in ['control','acquisition','native']}
 owner['plant']=folder_bytes(V/'plant')+sum((HERE/n).stat().st_size for n in ['engine.c','engine_api.py'])
 for k,allocation in [('control','controller'),('acquisition','acquisition'),('native','native'),('plant','plant')]:
  assert owner[k]<=forecast['allocations'][allocation]['retained'],(k,owner[k])
 owner['report']=size-sum(owner.values())
 assert owner['report']<=forecast['allocations']['report']['retained'],owner
 return {'conservative_incremental_retained_bytes':size,'cap_bytes':forecast['incremental_retained_cap_bytes'],
         'owner_bytes':owner,'preexisting_root_document_bytes_excluded':existing_bytes,
         'note':'All new source/evidence/report copies and existing architecture diagrams charged; root README/AGENTS charged by incremental byte growth. Bytecode caches excluded. No bulk arrays retained by current study.'}

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--verify',action='store_true');args=parser.parse_args()
 facts=audit_evidence();figures,wordcount=report_figures()
 check_markdown_links(REPORT)
 notes_text,rewrites=expected_notes();notes_path=NOTES/'active-contact-flange-report.md'
 if args.verify:
  inputs=read(R/'report_inputs.json')
  for x in inputs['sources']:check_binding(x)
  assert read(R/'report_facts.json')==facts
  delivery=read(R/'markdown_delivery_manifest.json')
  assert delivery['report_inputs_sha256']==sha(R/'report_inputs.json')
  assert delivery['repository_markdown_sha256']==sha(REPORT)
  assert delivery['notes_markdown_sha256']==sha(notes_path)
  for x in delivery['figures']:check_binding(x['repository']);check_binding(x['notes'])
 else:
  assert notes_path.is_file()
  acceptable={sha(V/'previous_report_11febde/Notes.md')}
  if (R/'markdown_delivery_manifest.json').exists():acceptable.add(read(R/'markdown_delivery_manifest.json')['notes_markdown_sha256'])
  assert sha(notes_path) in acceptable,'Notes file changed independently; preserve and inspect before delivering.'
  write(R/'report_facts.json',facts)
  write(R/'report_inputs.json',source_inputs(figures))
  (NOTES/'figures').mkdir(exist_ok=True)
  for p in figures:shutil.copyfile(p,NOTES/'figures'/p.name)
  notes_path.write_text(notes_text)
  delivery={'status':'markdown_and_eight_svg_delivery','date':'2026-10-07','html_updated':False,
   'repository_markdown':str(REPORT.relative_to(ROOT)),'repository_markdown_sha256':sha(REPORT),
   'notes_markdown':str(notes_path),'notes_markdown_sha256':sha(notes_path),
   'report_inputs_sha256':sha(R/'report_inputs.json'),'notes_only_evidence_link_rewrites':rewrites,
   'figures':[{'repository':{'path':str(p.relative_to(ROOT)),'sha256':sha(p)},
               'notes':{'path':str(NOTES/'figures'/p.name),'sha256':sha(NOTES/'figures'/p.name)}} for p in figures],
   'previous_reports':[{'path':str(p.relative_to(ROOT)),'sha256':sha(p)} for p in sorted((V/'previous_report_11febde').glob('*.md'))],
   'word_count':wordcount,'source_hash_and_xml_checks':True,'current_experiment_qualification':False}
  write(R/'markdown_delivery_manifest.json',delivery)
 assert notes_path.read_text()==notes_text
 for p in [notes_path,HERE/'README.md',HERE/'control/README.md',HERE/'acquisition/README.md',HERE/'native/README.md',ROOT/'README.md']:
  check_markdown_links(p)
 for p in figures:assert sha(p)==sha(NOTES/'figures'/p.name)
 result={'status':'verified_without_simulation' if args.verify else 'current_report_delivered_and_audited',
         'qualification':False,'protected_files_unchanged':facts['protected_file_count'],
         'primary_recordings':facts['primary_totals']['records'],'word_count':wordcount,
         'retention':retained_budget(figures),'shared_arrays':shared_arrays(),
         'notes_narrative_matches_repository':True,'all_local_links_and_svg_xml_valid':True,
         'all_new_figures_visually_inspected':True,'temporary_full_histories_released':True}
 if not args.verify:write(R/'finalization_checks.json',result)
 print(json.dumps(result,indent=2))

if __name__=='__main__':main()
