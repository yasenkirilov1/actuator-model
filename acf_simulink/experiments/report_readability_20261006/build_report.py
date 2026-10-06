"""Revise authored report content while preserving all existing reviewed data.

Uses the unchanged shared report renderer and product-owned runtime. New
experiments enter as independently source-bound queries, never rewritten old rows.
"""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / 'reports/prototype-design-20260930'
VAL = ROOT / 'acf_simulink/validation/report_readability_20261006/report'
ID = 'report:578d0259-115c-40f2-9264-4c57b68ab138'


def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda:f.read(4*1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--markdown',type=Path,required=True)
    parser.add_argument('--inputs',type=Path,required=True)
    parser.add_argument('--complete',action='store_true')
    args=parser.parse_args()
    saved=json.loads((VAL/'previous_authoring_manifest.json').read_text())
    data_entry=next(item for item in saved['files'] if item['source']==str(APP/'src/data.json'))
    original_path=Path(data_entry['snapshot'])
    assert sha(original_path)==data_entry['sha256']
    original=json.loads(original_path.read_text())
    current=json.loads((APP/'src/data.json').read_text())
    assert original['id']==current['id']==ID
    assert original['authoredRevision']==current['authoredRevision']
    assert all(current['queries'].get(k)==v for k,v in original['queries'].items())
    inputs=json.loads(args.inputs.read_text())
    queries=dict(current['queries'])
    for entry in inputs['queries']:
        path=ROOT/entry['file']
        assert sha(path)==entry['sha256'],path
        doc=json.loads(path.read_text())
        assert doc['status']=='complete',path
        rows=doc
        for key in entry.get('rowsPath',['rows']):rows=rows[key]
        assert isinstance(rows,list) and rows,path
        sources={f:sha(ROOT/f) for f in entry.get('sourceFiles',[])}
        assert doc.get('sourceHashes')==sources,path
        reviewed={'rows':rows,'source':{
            'label':entry['label'],'files':[entry['file'],*sources],
            'sha256':sha(path),'sourceFileSha256':sources,
            'kind':entry['kind'],'caveats':entry.get('caveats',[])}}
        if entry['id'] in original['queries']:
            assert reviewed==original['queries'][entry['id']],entry['id']
        queries[entry['id']]=reviewed
    spec=importlib.util.spec_from_file_location('nested_report',ROOT/'acf_simulink/experiments/build_nested_ml_report_20261005.py')
    helper=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    def source_ids(path,config):
        if len(path)>1:
            return inputs['componentQueryIds'].get(path[1],[])
        return inputs['rootQueryIds'].get(path[0],[])
    helper.source_ids=source_ids
    config=json.loads((ROOT/'acf_simulink/validation/budget_sensing_report_inputs_20261002.json').read_text())
    config['newFigures']=inputs['figures']
    for figure in inputs['figures']:
        assert sha(args.markdown.parent/figure['file'])==figure['sha256']
    if args.complete:
        assert inputs['status']=='complete'
        assert '**Study in progress.**' not in args.markdown.read_text()
    assets,sections=helper.render_content(args.markdown.read_text(),args.markdown.parent,config,queries)
    data={**current,'queries':queries,'buildStatus':'complete' if args.complete else 'updating',
          'report':{**current.get('report',{}),'asOf':inputs.get('evidenceAsOf','2026-10-06')}}
    assert data['id']==ID and data['authoredRevision']==original['authoredRevision']
    assert all(data['queries'][k]==v for k,v in original['queries'].items())
    (APP/'src/data.json').write_text(json.dumps(data,ensure_ascii=False,allow_nan=False)+'\n')
    manifest={'status':data['buildStatus'],'reportId':ID,'authoredRevision':data['authoredRevision'],
              'preservedQueries':len(original['queries']),'totalQueries':len(queries),
              'markdown':str(args.markdown),'markdownSha256':sha(args.markdown),
              'inputs':str(args.inputs),'inputsSha256':sha(args.inputs),
              'figures':assets,'componentBlocks':[c['title'] for s in sections for c in s['children']]}
    (VAL/'html_authoring_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'status':data['buildStatus'],'queries':len(queries),'figures':len(assets),'preservedQueries':len(original['queries'])}))


if __name__=='__main__':main()
