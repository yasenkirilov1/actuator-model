"""Build the nested Markdown revision in the existing report app.

The original report, scientific evidence and protected runtime are preserved.
This recipe changes authored report content, copied figures and reviewed data.
"""
import argparse
import hashlib
import importlib.util
import json
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / 'reports/prototype-design-20260930'
VAL = ROOT / 'acf_simulink/validation/ml_report_20261005'
REPORT_ID = 'report:578d0259-115c-40f2-9264-4c57b68ab138'
spec = importlib.util.spec_from_file_location('original_report_helpers', Path(__file__).with_name('build_inertial_report.py'))
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def slug(value):
    return re.sub(r'[^a-z0-9]+', '-', value.lower()).strip('-')


def parse_sections(markdown):
    """Retain the requested real heading hierarchy and unique contextual IDs."""
    split = re.split(r'^(#{2,5}) (.+)\s*$', markdown, flags=re.M)
    roots = []
    stack = []
    for level, title, body in zip(split[1::3], split[2::3], split[3::3]):
        depth = len(level)
        while stack and stack[-1]['depth'] >= depth:
            stack.pop()
        parent = stack[-1] if stack else None
        path = (parent['path'] if parent else []) + [title]
        node = {'id': 'report:nested:' + ':'.join(map(slug, path)), 'title': title,
                'depth': depth, 'path': path, 'body': body.strip(), 'children': []}
        (parent['children'] if parent else roots).append(node)
        stack.append(node)
    assert [r['title'] for r in roots] == ['Intro', 'Question', 'Results', 'Recommendation']
    return roots


def source_ids(path, config):
    old = config['sectionQueries']
    lookup = {
        'Mechanics: load path, springs and guides': ['selected_settings', 'actual_mechanics_tolerance', 'lower_spring_search', 'lower_series_repeated_reference', 'mixed_mechanics_identification', 'mixed_static_calibration', 'force_position_parts'],
        'Actuator and electrical drive': old['voice-coil-leadscrew-and-other-actuator-choices'] + old['voltage-current-capacity-and-heat'] + ['acceleration_and_drive_parts', 'budget_adc_hot_coil'],
        'Force and position measurement': ['force_position_parts', 'selected_settings', 'lower_series_force_balance', 'joint_budget_plan'],
        'Acceleration measurement': old['realistic-acceleration-sensing'] + old['cheaper-accelerometers-are-limited-mainly-by-response'] + ['joint_budget_plan'],
        'Acquisition electronics: amplifier, ADC and timing': old['why-a-cheaper-adc-can-be-enough'] + old['native-experiments-justify-the-bandwidth-and-sample-rate'] + old['acceptance-of-the-lower-cost-sensing-chain'],
        'Fast force controller': old['experiments-and-results'] + old['what-limits-performance'] + ['selected_settings'],
        'Robot commands and recentering': ['selected_settings', 'final_robot_OAT', 'joint_budget_plan'],
        'Learned control policy': ['ml_force_training', 'ml_force_test', 'ml_robot_training', 'ml_robot_test', 'ml_combined_test'],
        'Calibration, model qualification and prototype validation': old['final-acquisition-qualification-and-mechanical-tolerance'] + old['required-sensing-calibration-and-timing'] + ['budget_adc_range', 'budget_sensing_execution'],
    }
    specific = {
        'Learned fast current correction': ['ml_force_training', 'ml_force_test'],
        'Learned robot commands': ['ml_robot_training', 'ml_robot_test'],
        'Both learned actions in the same closed loop': ['ml_combined_test'],
        'Contact model and disturbance severity': ['bump_motion', 'report_model_schematics'],
        'Mechanical model and sensor placement': ['selected_settings', 'report_model_schematics'],
        'Equal-cohort PI/PID/feedforward comparison': ['reference_PI_gains', 'reference_strategies'],
        'Finite-cell structural sensitivity': ['cell_stiffness'],
        'Spring and damper search': ['lower_spring_search'],
        'Preload and early working-envelope evidence': ['lower_series_repeated_reference'],
        'Selected core controller and repeated bump test': ['lower_series_repeated_reference'],
        'Full scientific-reference grid and numerical qualification': ['qualified_range', 'final_acquisition_endpoints', 'final_worst_excerpt', 'final_solver_refinement'],
        'Random noise, fixed offsets and what is not injected': ['joint_budget_plan', 'budget_adc_electrical', 'budget_sensing_robustness'],
        'Executed calibration and timing mismatch': old['required-sensing-calibration-and-timing'],
        'Observed limits and faster-drive tradeoffs': old['what-limits-performance'],
        'Converters, input scaling and signal path': old['why-a-cheaper-adc-can-be-enough'] + ['report_model_schematics'],
        'Bandwidth, rate, timing, noise and hot-coil experiments': old['native-experiments-justify-the-bandwidth-and-sample-rate'],
        'Reference sensor, specific acceleration and acquisition budgets': ['acceleration_and_drive_parts'],
        'Executed lower-cost response and calibration experiments': old['cheaper-accelerometers-are-limited-mainly-by-response'],
        'Native actuator comparisons and transmission sizing': old['voice-coil-leadscrew-and-other-actuator-choices'],
        'Voltage headroom, current limits and continuous heat': old['voltage-current-capacity-and-heat'] + ['budget_adc_hot_coil'],
    }
    if path[-1] in specific:
        ids = specific[path[-1]]
    elif len(path) > 1:
        ids = lookup.get(path[1], [])
    elif path[0] == 'Results':
        ids = ['qualified_range', 'final_static_calibration_long', 'budget_adc_range', 'budget_adc_hot_coil', 'budget_sensing_execution']
    elif path[0] == 'Recommendation':
        ids = old['concrete-recommendation'] + ['ml_force_test', 'ml_robot_test', 'ml_combined_test']
    else:
        ids = ['report_model_schematics']
    return list(dict.fromkeys(ids))


def render_content(markdown, figure_root, config, queries):
    roots = parse_sections(markdown)
    figure_specs = {str(Path(f['file']).with_suffix('.svg')): f for f in config['figures']}
    for stem in ['acf_mechanism_and_sensors', 'acf_force_reconstruction', 'acf_causal_control_loops', 'acf_acquisition_timing']:
        figure_specs[f'figures/{stem}.svg'] = {'queryId': 'report_model_schematics'}
    for item in config.get('newFigures', []):
        figure_specs[item['file']] = item
    imports = []
    assets = []
    figures_seen = set()
    image_pattern = re.compile(r'!\[([^\]]*)\]\((figures/[^)]+)\)\s*\n(?:\n)?(?:\*([^\n]+)\*)?')

    def render_node(node):
        node['queryIds'] = [q for q in source_ids(node['path'], config) if q in queries]
        body = node.pop('body')
        fragments = []
        part = 0
        table_number = 0

        def append_copy(copy, include_heading=False):
            nonlocal part, table_number
            if include_heading:
                copy = '#' * node['depth'] + ' ' + node['title'] + '\n\n' + copy
            if not copy.strip():
                return
            for kind, value in helpers.split_tables(copy):
                if kind == 'prose':
                    fragments.append({'kind': kind, 'id': node['id'] + f':part-{part}', 'value': value})
                    part += 1
                else:
                    table_number += 1
                    assert node['queryIds'], f'Table without source context: {node["path"]}'
                    fragments.append({'kind': kind, 'id': node['id'] + f':table-{table_number}',
                                      'title': node['title'] + ' · ' + node['path'][1] if len(node['path']) > 1 else node['title'],
                                      'queryIds': node['queryIds'], **value})

        append_copy('', include_heading=True)
        start = 0
        for match in image_pattern.finditer(body):
            append_copy(body[start:match.start()].strip())
            relative = match.group(2)
            assert relative not in figures_seen, f'Duplicate figure: {relative}'
            figures_seen.add(relative)
            assert relative in figure_specs, f'Figure without source: {relative}'
            fig_spec = figure_specs[relative]
            assert fig_spec['queryId'] in queries
            source_path = figure_root / relative
            assert source_path.is_file(), source_path
            asset_name = 'inertial-' + Path(relative).name
            shutil.copy2(source_path, APP / 'src/content/assets' / asset_name)
            import_name = f'figure{len(imports)}'
            imports.append((import_name, '../assets/' + asset_name))
            assets.append({'file': str(source_path), 'sha256': digest(source_path), 'queryId': fig_spec['queryId']})
            fragments.append({'kind': 'figure', 'id': 'inertial:' + Path(relative).stem,
                              'title': match.group(1), 'caption': match.group(3) or '',
                              'queryId': fig_spec['queryId'], 'asset': import_name})
            start = match.end()
        append_copy(body[start:].strip())
        node['fragments'] = fragments
        for child in node['children']:
            render_node(child)

    for node in roots:
        render_node(node)
    jsx = ["import React, {useState} from 'react';",
           "import {RichNarrative,ReportSection,DataComponent,DataTable,useDataApp} from '../../data-app-public.jsx';"]
    jsx += [f'import {name} from {json.dumps(path)};' for name, path in imports]
    jsx += ['const sections=' + json.dumps(roots, ensure_ascii=False) + ';',
            'const figureAssets={' + ','.join(f'{n}:{n}' for n, _ in imports) + '};']
    jsx.append(r'''
function Figure({fragment,queries}){const rows=queries[fragment.queryId]?.rows||[];return <DataComponent id={fragment.id} queryId={fragment.queryId} kind="custom" title={fragment.title} sourceRows={rows} displayRows={rows}><figure className="study-figure"><img src={figureAssets[fragment.asset]} alt={fragment.title} loading="eager"/><RichNarrative id={fragment.id+':caption'} value={fragment.caption}/></figure></DataComponent>}
function InlineCell({value}){return String(value).split(/(\*\*[^*]+\*\*|`[^`]+`)/g).map((part,i)=>part.startsWith('**')?<strong key={i}>{part.slice(2,-2)}</strong>:part.startsWith('`')?<code key={i}>{part.slice(1,-1)}</code>:<React.Fragment key={i}>{part}</React.Fragment>)}
function ReportTable({fragment,queries}){const rows=Object.fromEntries(fragment.queryIds.map(id=>[id,queries[id]?.rows||[]]));return <DataComponent id={fragment.id} queryId={fragment.queryIds[0]} queryIds={fragment.queryIds} kind="table" title={fragment.title} sourceRows={rows[fragment.queryIds[0]]} sourceRowsByQuery={rows} displayRows={fragment.rows}><div className="study-table-wrap" data-reviewed-rows><table className="study-table" aria-label={fragment.title}><thead><tr>{fragment.headers.map((h,i)=><th scope="col" key={i} className={fragment.rightAligned[i]?'numeric':''}>{h}</th>)}</tr></thead><tbody>{fragment.rows.map((row,k)=><tr key={k}>{fragment.headers.map((_,i)=><td key={i} className={fragment.rightAligned[i]?'numeric':''}><InlineCell value={row['column'+i]}/></td>)}</tr>)}</tbody></table></div></DataComponent>}
function StudyNode({node,queries,qualificationTable}){const rows=Object.fromEntries(node.queryIds.map(id=>[id,queries[id]?.rows||[]]));const body=<>{node.fragments.map(f=>f.kind==='figure'?<Figure key={f.id} fragment={f} queries={queries}/>:f.kind==='table'?<ReportTable key={f.id} fragment={f} queries={queries}/>:<RichNarrative key={f.id} id={f.id} value={f.value}/>)}{node.title==='Full scientific-reference grid and numerical qualification'?qualificationTable:null}</>;return <section className="study-node" data-depth={node.depth} data-phase={node.title} id={node.id}><div className="study-node-body">{node.queryIds.length?<ReportSection id={node.id+':evidence'} title={node.title} showHeading={false} queryId={node.queryIds[0]} queryIds={node.queryIds} sourceRows={rows[node.queryIds[0]]} sourceRowsByQuery={rows}>{body}</ReportSection>:body}</div>{node.children.map(child=><StudyNode key={child.id} node={child} queries={queries} qualificationTable={qualificationTable}/>)}</section>}
export function ReportContent(){const {snapshot,appTitle}=useDataApp();const [mass,setMass]=useState('all'),[force,setForce]=useState('all');const range=snapshot.queries.qualified_range?.rows||[];const shown=range.filter(r=>(mass==='all'||r.massS_kg===Number(mass))&&(force==='all'||r.reference_N===Number(force)));const c=(field,label)=>({field,label});const n=(field,label)=>({field,label,renderCell:v=>typeof v==='number'?v.toFixed(3):'—'});const table=<DataComponent id="inertial:qualified-case-table" queryId="qualified_range" kind="table" title="Inspect the qualification recordings" sourceRows={range} displayRows={shown}><DataTable rows={shown} caption="Native full-history qualification" toolbarControls={<><label>Tool S <select aria-label="Tool S" value={mass} onChange={e=>setMass(e.target.value)}><option value="all">All masses</option>{[...new Set(range.map(r=>r.massS_kg))].sort((a,b)=>a-b).map(v=><option key={v} value={v}>{v} kg</option>)}</select></label><label>Target <select aria-label="Target force" value={force} onChange={e=>setForce(e.target.value)}><option value="all">All forces</option>{[...new Set(range.map(r=>r.reference_N))].sort((a,b)=>a-b).map(v=><option key={v} value={v}>{v} N</option>)}</select></label></>} columns={[c('massS_kg','S (kg)'),c('reference_N','Target (N)'),c('angle_deg','Angle (°)'),c('mu','Friction'),c('seedRequested','Noise seed'),n('peakNormalForceError_N','Peak error (N)'),n('peakCurrent_A','Current (A)'),n('peakVoltage_V','Voltage (V)'),c('passed','Pass')]}/></DataComponent>;return <article className="report-content" aria-label="Active contact flange nested design study"><header className="report-hero"><h1>{appTitle}</h1></header>{sections.map(node=><StudyNode key={node.id} node={node} queries={snapshot.queries} qualificationTable={table}/>)}</article>}
''')
    (APP / 'src/content/report/ReportContent.jsx').write_text('\n'.join(jsx))
    return assets, roots


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--markdown', type=Path, required=True)
    parser.add_argument('--figure-root', type=Path)
    parser.add_argument('--config', type=Path, default=ROOT / 'acf_simulink/validation/budget_sensing_report_inputs_20261002.json')
    parser.add_argument('--extra', type=Path)
    parser.add_argument('--complete', action='store_true')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    old = json.loads((VAL / 'previous_authoring/src/data.json').read_text())
    assert old['id'] == REPORT_ID
    queries = dict(old['queries'])
    diagram_file = VAL / 'diagram_contract.json'
    diagrams = json.loads(diagram_file.read_text())
    queries['report_model_schematics'] = {'rows': diagrams['rows'], 'source': {
        'label': 'Documented model and acquisition schematic contracts',
        'files': [str(diagram_file.relative_to(ROOT))], 'sha256': digest(diagram_file),
        'kind': diagrams['kind'], 'caveats': diagrams['caveats']}}
    if args.extra:
        extra = json.loads(args.extra.read_text())
        for item in extra['queries']:
            path = ROOT / item['file']
            doc = json.loads(path.read_text())
            assert doc.get('status') == 'complete', path
            rows = doc
            for field in item.get('rowsPath', ['rows']):
                rows = rows[field]
            assert isinstance(rows, list) and rows
            queries[item['id']] = {'rows': rows, 'source': {
                'label': item['label'], 'files': [item['file']] + item.get('sourceFiles', []), 'sha256': digest(path),
                'sourceFileSha256': {f: digest(ROOT / f) for f in item.get('sourceFiles', [])},
                'kind': item['kind'], 'caveats': item.get('caveats', [])}}
        config['newFigures'] = extra['figures']
    markdown = args.markdown.read_text()
    if args.complete:
        assert args.extra, 'Completed ML evidence is required.'
        assert '**Study in progress.**' not in markdown and '**Pending executed evidence.**' not in markdown
    assets, sections = render_content(markdown, args.figure_root or args.markdown.parent, config, queries)
    data = {**old, 'authoredRevision': old.get('authoredRevision', 0) + 1, 'queries': queries, 'buildStatus': 'complete' if args.complete else 'updating',
            'report': {**old.get('report', {}), 'asOf': '2026-10-05'}}
    assert all(data['queries'][key] == value for key, value in old['queries'].items())
    (APP / 'src/data.json').write_text(json.dumps(data, ensure_ascii=False, allow_nan=False) + '\n')
    manifest = {'status': data['buildStatus'], 'reportId': REPORT_ID,
                'markdown': str(args.markdown), 'markdownSha256': digest(args.markdown),
                'queries': {key: {'rowCount': len(q['rows']), 'source': q['source']} for key, q in queries.items()},
                'figures': assets, 'configSha256': digest(args.config),
                'preservedOriginalQueries': len(old['queries']), 'topLevelBlocks': [s['title'] for s in sections]}
    (VAL / 'html_manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps({'reportId': REPORT_ID, 'status': data['buildStatus'], 'queries': len(queries), 'figures': len(assets)}))


if __name__ == '__main__':
    main()
