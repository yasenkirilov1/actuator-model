"""Prepare compact report evidence and one figure; never run simulations.

Only existing JSON metrics are read. Native arrays and models are untouched.
The wrappers distinguish user scope, diagnostics, sizing and qualification.
"""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[3]
VAL = ROOT / 'acf_simulink/validation/report_readability_20261006'
OUT = VAL / 'report'
FIGURES = ROOT / 'reports/figures'
ADC_SOURCE = Path('/run/media/tommie/SATA SSD/actuator-model-evidence/20261006-report-readability/adc-private/executed/results.json')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def relative(path):
    return str(Path(path).relative_to(ROOT))


def write(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def wrapper(name, rows, sources, kind, **metadata):
    path = OUT / f'{name}.json'
    write(path, {'status': 'complete', 'kind': kind, 'rows': rows,
                 'sourceHashes': {relative(p): sha(p) for p in sources}, **metadata})
    return path


def query(name, path, label, caveats=()):
    doc = json.loads(path.read_text())
    return {'id': name, 'file': relative(path), 'sha256': sha(path),
            'label': label, 'kind': doc['kind'],
            'sourceFiles': list(doc['sourceHashes']), 'caveats': list(caveats)}


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    FIGURES.mkdir(parents=True, exist_ok=True)
    # Portable compact results; no histories are copied or opened.
    adc_path = VAL / 'adc/executed_results_compact.json'
    shutil.copy2(ADC_SOURCE, adc_path)
    parity_path = VAL / 'adc/reference_native_parity.json'
    replay_path = VAL / 'adc/matlab_replay.log'
    shutil.copy2(ADC_SOURCE.parents[1] / 'reference_native_parity.json', parity_path)
    shutil.copy2(ADC_SOURCE.parents[1] / 'matlab_replay.log', replay_path)
    adc = json.loads(adc_path.read_text())
    assert adc['status'] == 'complete_diagnostic_evidence_only'
    assert adc['executedDiagnosticRecordings'] == 42
    assert adc['qualificationRecordings'] == 0 and adc['controllerRetuned'] is False
    assert sum(g['cases'] for g in adc['groups']) == 42
    assert sum(g['nativeSamples'] for g in adc['groups']) == adc['nativeSamples']
    assert json.loads(parity_path.read_text())['all27ChannelsBitwiseExact'] is True
    assert 'MATLAB is exiting because of fatal error' in replay_path.read_text()
    adc_query = wrapper('adc_native_diagnostics', adc['groups'], [adc_path, parity_path, replay_path],
                        'executed_diagnostics_not_qualification',
                        nativeSamples=adc['nativeSamples'],
                        scope=adc['scope'], controllerRetuned=False,
                        independentMatlabRateReplay='No completed independent rate replay: the retained log shows a MATLAB startup fatal error before simulation execution.')

    parts_path = VAL / 'adc/parts_review_20261006.json'
    parts = json.loads(parts_path.read_text())
    parts_query = wrapper('adc_parts_reviewed', parts['parts'], [parts_path],
                          'manufacturer_specifications_and_recorded_prices',
                          pricesObservedAt=parts['observed_at_utc'])

    scope_path = OUT / 'prototype_scope_source.json'
    write(scope_path, {
        'date': '2026-10-06',
        'userInstruction': "Ah I see now, put that in the document as an open problem that needs a multi axis sensor, drag. But it's fine to ignore for now! We are already slightly out of scope anyway!",
        'prototypeAssumption': 'One-axis B-S load sensing and S acceleration; neglect axial sanding-drag coupling for now.',
        'openProblem': 'Distinguishing normal pressing force from unknown tangent drag on slopes requires multi-axis force sensing.',
        'frozenEvidence': 'Recorded multi-axis studies remain reference evidence and do not qualify the new single-axis architecture.',
        'thisRevision': 'Documentation and figures only; no new simulation, training or model changes.'})
    original_mechanics = FIGURES / 'fixed-upper-rigid-lower-20261005.svg'
    fixed_contract = ROOT / 'acf_simulink/validation/upper_no_damping_20261005/report/fixed_mechanics_contract.json'
    scope_query = wrapper('single_axis_prototype_scope',
                          [json.loads(scope_path.read_text())], [scope_path, original_mechanics, fixed_contract],
                          'user_scope_decision_not_experimental_result')

    # Version the explanatory diagram rather than changing the reference figure.
    ET.register_namespace('', 'http://www.w3.org/2000/svg')
    mechanics = ET.parse(original_mechanics)
    replacements = {
        'Fixed upper spring and rigid lower connection': 'First-prototype mechanics and one-axis sensing',
        'Fixed mechanics for the current prototype': 'Prescribed mechanics and first-prototype sensing',
        'Control and sensing must work around these prescribed connections.': 'Prototype assumption: neglect sanding drag coupled into the force axis.',
        '0.4 / 1.5 kg': '1.5 kg',
        'Multi-axis load cell': 'One-axis load cell',
        'Workpiece-normal force measurement': 'Force reconstruction with drag coupling neglected',
        'B–S cell vector': 'Axial B–S force',
        'Add tool inertia': 'Add S mass × S acceleration',
        'Project onto normal': 'Correct for surface slope',
        'Pressing force': 'Normal-force estimate',
        'An accelerometer on S supplies dynamic inertia; force, acceleration and normal estimates share one timestamp.': 'Force, S acceleration and surface-direction estimates must describe the same instant.',
        'Upper spring travel is unconstrained. Contact loss and the voice coil’s travel stops still fail a run.': 'Separating unknown sanding drag on slopes is deferred; that extension needs additional force axes.'}
    for node in mechanics.getroot().iter():
        if node.text in replacements:
            node.text = replacements[node.text]
        if node.tag.endswith('desc'):
            node.text = ('Robot R, prescribed 20 N/mm undamped upper spring, body A, voice coil, holder B, '
                         'finite-stiffness single-axis load cell and tool S. Position measures S relative to A. '
                         'Add measured tool inertia and correct surface angle under the no-axial-drag assumption.')
        if node.text == 'Add S mass × S acceleration':
            node.set('font-size', '16')
    mechanics.write(FIGURES / 'prototype-mechanics-single-axis-20261006.svg', encoding='unicode')

    cell_path = VAL / 'loadcell/loadcell_evidence.json'
    cell = json.loads(cell_path.read_text())
    cell_query = wrapper('loadcell_scope_evidence', [
        {k: cell[k] for k in ['geometry', 'equations', 'nativeCapacityRows',
                              'capacityTargets', 'calibrationTargets']}], [cell_path],
                         'recorded_capacity_and_measurement_geometry',
                         scope='Reference multi-axis model and offline reconstruction; no new single-axis closed-loop qualification.')

    actuator_path = VAL / 'actuator/actuator_evidence_reviewed.json'
    actuator = json.loads(actuator_path.read_text())
    actuator_query = wrapper('actuator_sizing_reviewed', [
        {k: actuator[k] for k in ['bumpDemand', 'screwPitchTradeoff',
                                  'historicalComparison', 'fixedVoltageDiagnostics',
                                  'voiceCoilTargets']}], [actuator_path],
                             'analytical_sizing_and_separate_recorded_comparisons')
    screw_figure = next(f for f in actuator['figures'] if f['id'] == 'leadscrew-pitch-inertia-current')
    screw_svg = next(f for f in screw_figure['artifacts'] if f['path'].endswith('.svg'))
    assert sha(ROOT / screw_svg['path']) == screw_svg['sha256']
    shutil.copy2(ROOT / screw_svg['path'], FIGURES / 'leadscrew-pitch-inertia-current-20261006.svg')

    # Point plots use a common axis and disclose the separate stopped group.
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'svg.fonttype': 'none', 'svg.hashsalt': 'acf-adc-20261006'})
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 4.8), sharey=True)
    bits = [12, 14, 16, 24]
    bit_groups = [next(g for g in adc['groups'] if g['rate_Hz'] == 30000 and g['quantizationBits'] == b) for b in bits]
    rate_groups = [next(g for g in adc['groups'] if g['rate_Hz'] == rate and g['quantizationBits'] == 24) for rate in [15000, 30000]]
    for axis, groups, labels, title in [
            (axes[0], bit_groups, [str(b) for b in bits], 'Code spacing at 30 ksample/s'),
            (axes[1], rate_groups, ['15', '30'], 'Converter rate with 24-bit codes')]:
        for i, g in enumerate(groups):
            error = g['worstTotalPeakError_N']
            axis.scatter(i, error, s=70, color='#087f8c' if g['passes'] == g['cases'] else '#b85032', zorder=3)
            axis.annotate(f"{error:.3f} N\n{g['passes']}/{g['cases']} pass", (i, error),
                          xytext=(0, 10), textcoords='offset points', ha='center', fontsize=10)
        axis.axhline(.5, color='#b85032', linestyle='--', linewidth=1.2)
        axis.set_xticks(range(len(labels)), labels)
        axis.set_xlim(-.55, len(labels) - .45)
        axis.set_ylim(.45, .585)
        axis.set_title(title, fontsize=11, pad=12)
        axis.grid(axis='y', color='#d9e2e7', linewidth=.8)
        axis.spines[['top', 'right']].set_visible(False)
    axes[0].set_xlabel('Quantization bits; ADC noise/timing unchanged')
    axes[1].set_xlabel('Fresh samples per channel (thousands/s)')
    axes[0].set_ylabel('Largest force error over seven cases (N)')
    axes[1].text(.03, .501, '0.5 N requirement', transform=axes[1].get_yaxis_transform(),
                 ha='left', va='bottom', color='#9b4128', fontsize=9)
    fig.suptitle('ADC diagnostics on the locked fixed-mechanics controller', fontsize=14, x=.5, y=.98)
    fig.text(.06, .09, '7.5 ksample/s: 0/7 pass; all recordings stopped during startup, before the bump.\n'
             'Largest recorded error 52.218 N; excluded from the shared plotting range, included in the report table.', fontsize=9)
    fig.text(.06, .025, '42 completed diagnostics; known development corners, no retuning, multi-axis reference sensing. Not hardware qualification.',
             fontsize=9, color='#455361')
    fig.subplots_adjust(left=.09, right=.97, top=.83, bottom=.29, wspace=.25)
    figure_path = FIGURES / 'adc-fixed-diagnostics-20261006.svg'
    fig.savefig(figure_path, metadata={'Date': None})
    (OUT / 'previews').mkdir(exist_ok=True)
    fig.savefig(OUT / 'previews/adc-fixed-diagnostics-20261006.png', dpi=120)
    plt.close(fig)

    old = json.loads((ROOT / 'acf_simulink/validation/upper_no_damping_20261005/report/report_inputs_compact_final.json').read_text())
    mappings = dict(old['componentQueryIds'])
    roots = {key: mappings.pop(key) for key in ['Intro', 'Question', 'Results', 'Recommendation']}
    for ids in roots.values():
        ids.append('single_axis_prototype_scope')
    roots['Results'].append('explained_nominal_grid')
    roots['Recommendation'] += ['adc_native_diagnostics', 'adc_parts_reviewed', 'actuator_sizing_reviewed']
    mappings['Mechanics: load path, springs and guides'].append('explained_cell_mode')
    mappings['Actuator and electrical drive'].append('actuator_sizing_reviewed')
    mappings['Force and position measurement'] += ['single_axis_prototype_scope', 'loadcell_scope_evidence']
    mappings['Acceleration measurement'] += ['single_axis_prototype_scope', 'adc_parts_reviewed']
    mappings['Acquisition electronics: amplifier, ADC and timing'] += ['adc_native_diagnostics', 'adc_parts_reviewed', 'single_axis_prototype_scope']
    mappings['Fast force controller'].append('explained_cell_mode')
    mappings['Calibration, model qualification and prototype validation'].append('single_axis_prototype_scope')

    figures = [f for f in old['figures'] if 'adc-rate-evidence' not in f['file'] and 'fixed-upper-rigid-lower' not in f['file']]
    figures += [{'file': f'figures/{name}', 'sha256': sha(FIGURES / name), 'queryId': q} for name, q in [
        ('fixed-force-operating-grid-20261006.svg', 'explained_nominal_grid'),
        ('prototype-mechanics-single-axis-20261006.svg', 'single_axis_prototype_scope'),
        ('load-cell-mode-explained-20261006.svg', 'explained_cell_mode'),
        ('leadscrew-pitch-inertia-current-20261006.svg', 'actuator_sizing_reviewed'),
        ('adc-fixed-diagnostics-20261006.svg', 'adc_native_diagnostics')]]
    entries = [
        query('explained_cell_mode', OUT / 'explained_cell_mode.json', 'Cell-mode mechanics and locked feedback contract'),
        query('explained_nominal_grid', OUT / 'explained_nominal_grid.json', '700 locked reference nominal recordings'),
        query('adc_native_diagnostics', adc_query, '42 completed fixed-controller ADC diagnostics',
              ['Known development cases; no new qualification or controller retuning.',
               'Bit tests change code spacing only; rate tests change converter noise and delay.',
               'The independent MATLAB rate replay did not complete.',
               'Bulk raw histories were removed under the storage audit; compact results are retained.']),
        query('adc_parts_reviewed', parts_query, 'Manufacturer specifications and recorded 6 October prices'),
        query('single_axis_prototype_scope', scope_query, 'First-prototype sensing scope accepted by the user'),
        query('loadcell_scope_evidence', cell_query, 'Reference measurement geometry and load-cell capacity'),
        query('actuator_sizing_reviewed', actuator_query, 'Actuator sizing and distinct historical/native diagnostics')]
    destination = OUT / 'report_inputs_design_explained.json'
    write(destination, {'status': 'complete', 'evidenceAsOf': '2026-10-06', 'queries': entries,
                        'componentQueryIds': mappings, 'rootQueryIds': roots, 'figures': figures,
                        'scope': 'Readable report revision and reduced prototype sensing scope. No new simulation or training in this authoring step.'})
    print(json.dumps({'status': 'complete', 'input': relative(destination), 'newQueries': len(entries),
                      'figures': len(figures), 'newSimulationRecordings': 0,
                      'compactAdcBytes': adc_path.stat().st_size}))


if __name__ == '__main__':
    main()
