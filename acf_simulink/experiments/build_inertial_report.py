"""Render the requested Markdown study into the existing evidence-backed app.

Only authored content and reviewed data are changed. A manifest explicitly
selects completed evidence; an unfinished experiment cannot become a result
by being present in the validation directory. The app identity is preserved.
"""
import argparse
import hashlib
import json
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / 'reports' / 'prototype-design-20260930'
MD = ROOT / 'reports' / 'active-contact-flange-report.md'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def slug(value):
    return re.sub(r'[^a-z0-9]+', '-', value.lower()).strip('-')


def split_tables(value):
    """Keep report tables structured; RichNarrative does not render GFM tables."""
    lines = value.splitlines()
    blocks, prose = [], []
    at = 0
    while at < len(lines):
        line = lines[at].strip()
        following = lines[at + 1].strip() if at + 1 < len(lines) else ''
        separators = [cell.strip() for cell in following.strip('|').split('|')]
        is_table = (line.startswith('|') and following.startswith('|') and
                    all(re.fullmatch(r':?-{3,}:?', cell) for cell in separators))
        if not is_table:
            prose.append(lines[at])
            at += 1
            continue
        if '\n'.join(prose).strip():
            blocks.append(('prose', '\n'.join(prose).strip()))
        prose = []
        headers = [cell.strip() for cell in line.strip('|').split('|')]
        assert len(headers) == len(separators)
        table_rows = []
        at += 2
        while at < len(lines) and lines[at].strip().startswith('|'):
            cells = [cell.strip() for cell in lines[at].strip().strip('|').split('|')]
            assert len(cells) == len(headers), 'Inconsistent Markdown table width'
            table_rows.append({f'column{i}': cell for i, cell in enumerate(cells)})
            at += 1
        assert table_rows
        blocks.append(('table', {'headers': headers, 'rows': table_rows,
                                 'rightAligned': [cell.endswith(':') for cell in separators]}))
    if '\n'.join(prose).strip():
        blocks.append(('prose', '\n'.join(prose).strip()))
    return blocks


def load_queries(config):
    queries = {}
    sources = []
    for spec in config['queries']:
        path = ROOT / spec['file']
        doc = json.loads(path.read_text())
        if spec.get('requiredStatus'):
            assert doc.get('status') in spec['requiredStatus'], (path, doc.get('status'))
        rows = doc
        for key in spec.get('rowsPath', ['rows']):
            rows = rows[key]
        if isinstance(rows, dict) and spec.get('dictionaryRows'):
            rows = [{'sourceKey': key, **value} for key, value in rows.items()]
        assert isinstance(rows, list) and rows, f'No reviewed rows: {path}'
        entry = {'file': spec['file'], 'sha256': digest(path), 'rowCount': len(rows),
                 'kind': spec['kind']}
        sources.append(entry)
        queries[spec['id']] = {
            'rows': rows,
            'source': {'label': spec['label'], 'files': [spec['file']],
                       'sha256': entry['sha256'], 'kind': spec['kind'],
                       'caveats': spec.get('caveats', [])},
        }
    return queries, sources


def render_content(config):
    markdown = MD.read_text()
    blocks = re.split(r'^## (.+)\s*$', markdown, flags=re.M)
    assert len(blocks) > 2
    sections, imports, assets = [], [], []
    image_pattern = re.compile(r'!\[([^\]]*)\]\((figures/[^)]+)\)\s*\n(?:\n)?(?:\*([^\n]+)\*)?')
    figure_specs = {f['file']: f for f in config.get('figures', [])}
    for heading, body in zip(blocks[1::2], blocks[2::2]):
        section_id = 'report:' + slug(heading)
        fragments = []
        start = 0
        part = 0
        table_number = 0
        def append_copy(copy):
            nonlocal part, table_number
            if not copy:
                return
            if part == 0:
                copy = '## ' + heading + '\n\n' + copy
            for kind, value in split_tables(copy):
                if kind == 'prose':
                    fragments.append({'kind': kind, 'id': section_id + f':part-{part}',
                                      'value': value})
                    part += 1
                else:
                    table_number += 1
                    query_ids = config['tableQueries'][slug(heading)]
                    fragments.append({'kind': kind,
                                      'id': section_id + f':table-{table_number}',
                                      'title': config['tableTitles'][slug(heading)],
                                      'queryIds': query_ids, **value})
        for match in image_pattern.finditer(body):
            copy = body[start:match.start()].strip()
            append_copy(copy)
            relative = match.group(2)
            assert relative in figure_specs, f'Figure has no reviewed source: {relative}'
            spec = figure_specs[relative]
            asset_name = 'inertial-' + Path(relative).name
            source_path = MD.parent / relative
            assert source_path.is_file(), source_path
            destination = APP / 'src' / 'content' / 'assets' / asset_name
            shutil.copy2(source_path, destination)
            import_name = 'figure' + str(len(imports))
            imports.append((import_name, '../assets/' + asset_name))
            assets.append({'file': str(source_path.relative_to(ROOT)),
                           'sha256': digest(source_path), 'queryId': spec['queryId']})
            fragments.append({'kind': 'figure', 'id': 'inertial:' + Path(relative).stem,
                              'title': match.group(1), 'caption': match.group(3) or spec.get('caption', ''),
                              'queryId': spec['queryId'], 'asset': import_name})
            start = match.end()
        copy = body[start:].strip()
        append_copy(copy)
        sections.append({'id': section_id, 'title': heading,
                         'queryIds': config.get('sectionQueries', {}).get(slug(heading), []),
                         'fragments': fragments})
    lines = ["import React, {useState} from 'react';",
             "import {RichNarrative,ReportSection,DataComponent,DataTable,useDataApp} from '../../data-app-public.jsx';"]
    for name, path in imports:
        lines.append(f'import {name} from {json.dumps(path)};')
    lines.append('const sections=' + json.dumps(sections, ensure_ascii=False) + ';')
    lines.append('const figureAssets={' + ','.join(f'{name}:{name}' for name, _ in imports) + '};')
    lines.append("const c=(field,label)=>({field,label});const n=(field,label)=>({field,label,renderCell:value=>typeof value==='number'?value.toFixed(3):'—'});")
    lines.append(r"""
function Figure({fragment,rows}) {return <DataComponent id={fragment.id} queryId={fragment.queryId} kind="custom" title={fragment.title} sourceRows={rows} displayRows={rows}><figure className="study-figure"><img src={figureAssets[fragment.asset]} alt={fragment.title} loading="lazy"/><RichNarrative id={fragment.id+':caption'} value={fragment.caption}/></figure></DataComponent>}
function InlineCell({value}){return String(value).split(/(\*\*[^*]+\*\*|`[^`]+`)/g).map((part,i)=>part.startsWith('**')?<strong key={i}>{part.slice(2,-2)}</strong>:part.startsWith('`')?<code key={i}>{part.slice(1,-1)}</code>:<React.Fragment key={i}>{part}</React.Fragment>)}
function ReportTable({fragment,queries}){const sourceRowsByQuery=Object.fromEntries(fragment.queryIds.map(id=>[id,queries[id]?.rows||[]]));return <DataComponent id={fragment.id} queryId={fragment.queryIds[0]} queryIds={fragment.queryIds} kind="table" title={fragment.title} sourceRows={sourceRowsByQuery[fragment.queryIds[0]]} sourceRowsByQuery={sourceRowsByQuery} displayRows={fragment.rows}><div className="study-table-wrap" data-reviewed-rows><table className="study-table" aria-label={fragment.title}><thead><tr>{fragment.headers.map((header,i)=><th scope="col" key={i} className={fragment.rightAligned[i]?'numeric':''}>{header}</th>)}</tr></thead><tbody>{fragment.rows.map((row,index)=><tr key={index}>{fragment.headers.map((_,i)=><td key={i} className={fragment.rightAligned[i]?'numeric':''}><InlineCell value={row['column'+i]}/></td>)}</tr>)}</tbody></table></div></DataComponent>}
export function ReportContent(){
 const {snapshot,appTitle}=useDataApp();const R=id=>snapshot.queries[id]?.rows||[];
 const [mass,setMass]=useState('all'),[force,setForce]=useState('all');
 const range=R('qualified_range');
 const shown=range.filter(r=>(mass==='all'||r.massS_kg===Number(mass))&&(force==='all'||r.reference_N===Number(force)));
 const table=<DataComponent id="inertial:qualified-case-table" queryId="qualified_range" kind="table" title="Inspect the qualification recordings" sourceRows={range} displayRows={shown}><DataTable rows={shown} caption="Native full-history qualification" toolbarControls={<><label>Tool S <select value={mass} onChange={e=>setMass(e.target.value)}><option value="all">All masses</option>{[...new Set(range.map(r=>r.massS_kg))].sort((a,b)=>a-b).map(v=><option key={v} value={v}>{v} kg</option>)}</select></label><label>Target <select value={force} onChange={e=>setForce(e.target.value)}><option value="all">All forces</option>{[...new Set(range.map(r=>r.reference_N))].sort((a,b)=>a-b).map(v=><option key={v} value={v}>{v} N</option>)}</select></label></>} columns={[c('massS_kg','S (kg)'),c('reference_N','Target (N)'),c('angle_deg','Angle (°)'),c('mu','Friction'),c('seedRequested','Noise seed'),n('peakNormalForceError_N','Peak error (N)'),n('peakCurrent_A','Current (A)'),n('peakVoltage_V','Voltage (V)'),c('passed','Pass')]}/></DataComponent>;
 return <article className="report-content" aria-label="Active contact flange measured-acceleration study"><header className="report-hero"><h1>{appTitle}</h1></header>{sections.map(section=>{const content=<>{section.fragments.map(fragment=>fragment.kind==='figure'?<Figure key={fragment.id} fragment={fragment} rows={R(fragment.queryId)}/>:fragment.kind==='table'?<ReportTable key={fragment.id} fragment={fragment} queries={snapshot.queries}/>:<RichNarrative key={fragment.id} id={fragment.id} value={fragment.value}/>)}{section.title==='Final acquisition qualification and mechanical tolerance'&&range.length>0?table:null}</>;return section.queryIds.length?<ReportSection key={section.id} id={section.id} title={section.title} showHeading={false} queryId={section.queryIds[0]} queryIds={section.queryIds} sourceRows={R(section.queryIds[0])} sourceRowsByQuery={Object.fromEntries(section.queryIds.map(id=>[id,R(id)]))}>{content}</ReportSection>:<section key={section.id}>{content}</section>})}</article>
}
""")
    (APP / 'src' / 'content' / 'report' / 'ReportContent.jsx').write_text('\n'.join(lines))
    return assets


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, default=ROOT / 'acf_simulink' / 'validation' / 'inertial_report_inputs.json')
    parser.add_argument('--complete', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    old_data = json.loads((APP / 'src' / 'data.json').read_text())
    assert old_data['id'] == 'report:578d0259-115c-40f2-9264-4c57b68ab138'
    if args.complete:
        assert config.get('readyForDelivery') is True, 'Evidence and narrative review required.'
        assert '**Working report:**' not in MD.read_text(), 'Markdown remains a draft.'
    queries, sources = load_queries(config)
    for fig in config.get('figures', []):
        assert fig['queryId'] in queries
    for ids in config.get('sectionQueries', {}).values():
        assert all(i in queries for i in ids)
    assets = render_content(config)
    data = {**old_data, 'title': 'Active contact flange: control and prototype design',
            'surface': 'report', 'queries': queries, 'filters': [],
            'buildStatus': 'complete' if args.complete else 'updating',
            'report': {**old_data.get('report', {}), 'asOf': config['asOf']}}
    (APP / 'src' / 'data.json').write_text(json.dumps(data, ensure_ascii=False, allow_nan=False) + '\n')
    manifest = {'reportId': data['id'], 'markdown': str(MD.relative_to(ROOT)),
                'markdownSha256': digest(MD), 'status': data['buildStatus'],
                'evidence': sources, 'figures': assets, 'configSha256': digest(args.config)}
    (ROOT / 'acf_simulink' / 'validation' / 'inertial_html_report_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'reportId': data['id'], 'buildStatus': data['buildStatus'],
                      'queries': len(queries), 'figures': len(assets)}))


if __name__ == '__main__':
    main()
