"""Bind the concise report to accepted executed fixed-mechanics results.

Rebuild after new completed cohorts; never count prepared bank rows as runs.
Historical invalid-source observers remain preserved outside this ledger.
"""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[3]
REPORT = ROOT / 'acf_simulink/validation/upper_no_damping_20261005/report'
BASE = ROOT / 'acf_simulink/validation'

COHORTS = [
    ('Acquired-force observer, initial tuning', 'upper_force_observer_20261005/evaluation/measured_force_endpoint_development_v3/results.json'),
    ('Corrected issued-current observer', 'upper_no_damping_20261005/evaluation/corrected_current_endpoint_development_v5/results.json'),
    ('Guide-compensation choices, issued-current observer', 'upper_no_damping_20261005/evaluation/guide_ff_endpoint_development_v6/results.json'),
    ('Observer weights and cell derivative: one endpoint', 'upper_force_observer_20261005/evaluation/observer_noise_case7_development_v4/results.json'),
    ('Two observer designs: remaining endpoints', 'upper_force_observer_20261005/evaluation/observer_noise_promising_endpoints_v4/results.json'),
    ('Sixteen acceleration-lead designs: two endpoints', 'upper_force_observer_20261005/evaluation/observer_lead_two_endpoints_v5/results.json'),
    ('Guide-compensation choices, acquired-force observer', 'upper_no_damping_20261005/evaluation/force_observer_guide_v7_three_endpoints/results.json'),
    ('Phase-shaped cell-mode feedback: two endpoints', 'upper_force_observer_20261005/evaluation/cell_modal_v6_two_endpoints/results.json'),
    ('Two cell-mode feedback settings: remaining corners', 'upper_force_observer_20261005/evaluation/cell_modal_v6_remaining_endpoints/results.json'),
    ('No-lead modal controller: remaining development points', 'upper_force_observer_20261005/evaluation/cell_modal_v6_family17_dev11_remaining/results.json'),
    ('Modal feedback with acceleration lead and guide smoothing', 'upper_force_observer_20261005/evaluation/cell_modal_lead_v7_four_endpoints/results.json'),
    ('Final live normal-PI and cell-mode gain comparison', 'upper_force_observer_20261005/evaluation/controller_gain_v9_A15_dev11/results.json'),
]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    sources, rows, cases = {}, [], []
    for label, relative in COHORTS:
        path = BASE / relative
        document = json.loads(path.read_text())
        assert document['status'] == 'complete' and document['lockedInputsUnchanged']
        assert document['cases'] == len(document['rows'])
        assert all(r['upperK_N_m'] == 20000 and r['upperC_N_s_m'] == 0
                   and r['actualK_N_m'] == 64e6 and r['actualC_N_s_m'] == 0
                   and not r['lowerExternalSpringPresent'] and not r['upperRigid']
                   and r['nativeGridComplete'] and not r['softwareRejected']
                   for r in document['rows'])
        source = str(path.relative_to(ROOT))
        sources[source] = sha(path)
        lock = path.with_name('input_lock.json')
        sources[str(lock.relative_to(ROOT))] = sha(lock)
        worst = max(document['rows'], key=lambda r: r['peakNormalForceError_N'])
        rows.append({'experiment': label, 'stage': 'development',
                     'recordings': document['cases'], 'passed': document['passed'],
                     'hardFailures': document['physicalFailures'],
                     'nativeSamples': document['nativeSamples'],
                     'worstNormalForceError_N': worst['peakNormalForceError_N'],
                     'worstIdentity': worst['identity'], 'source': source,
                     'sourceSha256': sources[source],
                     'retentionReplaysExcluded': True})
        cases += [{**r, 'experiment': label, 'source': source} for r in document['rows']]
    assert sum(r['recordings'] for r in rows) == len(cases)
    out = {'status': 'complete', 'kind': 'accepted_native_fixed_mechanics_development',
           'rows': rows, 'sourceHashes': sources,
           'totals': {'recordings': len(cases), 'passed': sum(r['passed'] for r in cases),
                      'nativeSamples': sum(r['nativeSamples'] for r in cases),
                      'hardFailures': sum(r['physicalFailure'] for r in cases)},
           'scope': 'Accepted executed development configurations only; tuning cases do not establish fresh-test or physical qualification.',
           'excludedImplementationEvidence': 'Earlier upper-compliant observers with disconnected local From sources are retained as implementation rejections, not mechanical performance evidence.'}
    (REPORT / 'fixed_development_results.json').write_text(json.dumps(out, indent=2) + '\n')
    inputs = json.loads((REPORT / 'report_inputs_draft.json').read_text())
    inputs['queries'] = [q for q in inputs['queries'] if q['id'] not in (
        'fixed_development_results', 'fixed_control_waveform', 'fixed_acquisition_reference',
        'fixed_voltage_diagnostics', 'fixed_controller_contract', 'fixed_native_verification')]
    for name, label in (
        ('fixed_development_results', 'Executed accepted fixed-mechanics control development'),
        ('fixed_control_waveform', 'Native development waveform and cell-mode spectral analysis'),
        ('fixed_acquisition_reference', 'Earlier native ADC-rate comparison; different reference mechanics')):
        path = REPORT / (name + '.json')
        document = json.loads(path.read_text())
        inputs['queries'].append({'id': name, 'file': str(path.relative_to(ROOT)),
                                 'sha256': sha(path), 'label': label, 'kind': document['kind'],
                                 'sourceFiles': list(document['sourceHashes']),
                                 'caveats': [document['scope']]})
    diagram = next(f for f in inputs['figures'] if 'fixed-upper-rigid-lower' in f['file'])
    diagram['sha256'] = sha(ROOT / 'reports' / diagram['file'])
    inputs['figures'] = [diagram, *json.loads((REPORT / 'compact_figure_manifest.json').read_text())]
    current = ['fixed_mechanics_contract', 'fixed_development_results']
    qualified = REPORT / 'fixed_qualification_results.json'
    if qualified.exists():
        document = json.loads(qualified.read_text())
        assert document['status'] == 'complete'
        inputs['queries'] = [q for q in inputs['queries'] if q['id'] != 'fixed_qualification_results']
        inputs['queries'].append({'id': 'fixed_qualification_results',
            'file': str(qualified.relative_to(ROOT)), 'sha256': sha(qualified),
            'label': 'Locked candidate: new-seed fixed-mechanics force and physical tests',
            'kind': document['kind'], 'sourceFiles': list(document['sourceHashes']),
            'caveats': [document['scope']]})
        current += ['fixed_qualification_results']
    acceptance = []
    for name, label in (
        ('fixed_voltage_diagnostics', 'Independent unchanged-controller MATLAB voltage pairs'),
        ('fixed_controller_contract', 'Locked executing normal PI, modal feedback and observer design'),
        ('fixed_native_verification', 'Independent MATLAB histories, solver refinement and native-model delivery checks')):
        path = REPORT / (name + '.json')
        if not path.exists():
            continue
        document = json.loads(path.read_text())
        assert document['status'] == 'complete'
        inputs['queries'].append({'id': name, 'file': str(path.relative_to(ROOT)),
            'sha256': sha(path), 'label': label, 'kind': document['kind'],
            'sourceFiles': list(document['sourceHashes']), 'caveats': [document['scope']]})
        acceptance.append(name)
    inputs['mechanicsQueryIds'] = current + ['fixed_control_waveform']
    inputs['componentQueryIds'] = {
        'Intro': ['fixed_mechanics_contract'],
        'Question': ['fixed_mechanics_contract'],
        'Results': current,
        'Recommendation': current + acceptance + ['rl_budget_parts', 'rl_budget_sensing_contract'],
        'Mechanics: load path, springs and guides': current + ['fixed_control_waveform'],
        'Actuator and electrical drive': current + ['fixed_control_waveform', 'fixed_voltage_diagnostics', 'coil_parts', 'final_actuator_comparison'],
        'Force and position measurement': ['fixed_mechanics_contract', 'force_position_parts', 'joint_budget_plan'],
        'Acceleration measurement': ['rl_budget_parts', 'rl_budget_sensing_contract', 'fixed_mechanics_contract', 'budget_accelerometer_stress'],
        'Acquisition electronics: amplifier, ADC and timing': ['fixed_acquisition_reference', 'rl_budget_parts', 'rl_budget_sensing_contract', 'budget_adc_screen'],
        'Fast force controller': current + ['fixed_control_waveform', 'fixed_controller_contract'],
        'Robot commands and recentering': current + ['rl_method_contract', 'final_robot_OAT'],
        'Learned control policy': ['rl_selected_candidate', 'rl_nominal_grid', 'rl_fresh_tests', 'rl_long_tests'],
        'Calibration, model qualification and prototype validation': current + acceptance + ['rl_budget_sensing_contract', 'joint_budget_plan'],
    }
    (REPORT / 'report_inputs_compact_draft.json').write_text(json.dumps(inputs, indent=2) + '\n')
    print(json.dumps(out['totals']))


if __name__ == '__main__':
    main()
