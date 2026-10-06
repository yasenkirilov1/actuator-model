#!/usr/bin/env python3
"""Bind the three native static-calibration records and their portable inputs.

This performs file/record checks and copies inputs. It runs no simulations and
does not change the original mixed-source snapshot or nominal candidate.
"""
import hashlib
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent
STEM = 'inertial_mechanics_calibrated_static_mixed_20260930'
FIGURE = 'inertial_mechanics_calibrated_static_mixed_comparison_20260930'
SOURCE = ROOT / 'inertial_mechanics_mixed_source_20260930'
PORTABLE = ROOT / 'inertial_mechanics_calibrated_static_mixed_source_20260930'


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def read(path):
    return json.loads(path.read_text())


def frozen_copy(source, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        assert sha(target) == sha(source), f'Preserve existing input: {target}'
    else:
        shutil.copy2(source, target)
    assert sha(target) == sha(source)


def write_frozen(path, value):
    content = json.dumps(value, indent=2) + '\n'
    if path.exists():
        assert path.read_text() == content, f'Preserve existing manifest: {path}'
    else:
        path.write_text(content)


new = read(ROOT / f'{STEM}_results.json')
baseline_path = ROOT / 'inertial_mechanics_calibrated_mechanics_20260930_results.json'
baseline = read(baseline_path)
launch = read(ROOT / f'{STEM}_invocation_manifest.json')
figure_manifest = read(ROOT / f'{FIGURE}_figure_manifest.json')
old_portable = read(SOURCE / 'portable_input_manifest.json')
assert new['status'] == baseline['status'] == 'complete'
assert new['executedCases'] == new['passedCases'] == 3
assert new['preparationRejects'] == 0 and new['onlySixteenStaticChangesVerified']
assert [row['seed'] for row in new['rows']] == [7, 29, 1729]
assert len(new['rows']) == len(baseline['rows']) == 3
assert new['sourceModelSHA256'] == baseline['sourceModelSHA256']
assert new['sourceModelSHA256'] == old_portable['calibratedNativeModelSHA256']
assert new['baseV8NativeModelSHA256'] == old_portable['recordedBaseV8NativeModelSHA256']
assert sha(SOURCE / 'acf_force_control_normal_trial.slx') == new['sourceModelSHA256']
assert sha(SOURCE / 'joint_configs.mat') == new['configurationSHA256']
assert sha(ROOT / f'{STEM}_configuration.mat') == new['preparedConfigurationSHA256']
recipe = ROOT / 'inertial_mechanics_run_calibrated_static_mixed_20260930.m'
assert sha(recipe) == new['recipeSHA256']
nominal_candidate = ROOT / 'inertial_final_candidate_20260930.mat'
nominal_sha = 'd8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7'
assert sha(nominal_candidate) == nominal_sha

native_inputs = []
for prior, row, path, expected in zip(
        baseline['rows'], new['rows'], new['baselineNativeFiles'],
        new['baselineNativeSHA256']):
    assert prior['seed'] == row['seed']
    prior_path = ROOT / Path(path).name
    assert sha(prior_path) == expected == prior['nativeTraceSHA256']
    new_path = ROOT / Path(row['nativeTraceFile']).name
    assert sha(new_path) == row['nativeTraceSHA256']
    assert row['passed'] and row['completed'] and row['forcePassed'] and row['packingPassed']
    assert row['staticCalibrationGuard']['passed']
    assert row['staticCalibrationGuard']['staticResidualCount'] == 16
    assert row['staticCalibrationGuard']['physicalAndTimingCoefficientsIdentical']
    assert row['preparedParametersPreservedAfterRun']
    assert row['actualLowerStiffness_N_m'] == row['identifiedLowerStiffness_N_m'] == 23695.2
    assert row['actualLowerDamping_Ns_m'] == row['identifiedLowerDamping_Ns_m'] == 48
    native_inputs.append({'file': '../' + prior_path.name, 'sha256': expected,
                          'role': 'Original full returned known-actual-k/c native input parameters',
                          'seed': row['seed']})
for helper in new['helperSources']:
    assert sha(SOURCE / 'helpers' / helper['file']) == helper['sha256']
for entry in figure_manifest['sources'] + figure_manifest['figures']:
    assert sha(ROOT / entry['file']) == entry['sha256']
assert sha(ROOT / figure_manifest['script']) == figure_manifest['scriptSHA256']
assert sha(ROOT / figure_manifest['chartData']) == figure_manifest['chartDataSHA256']
assert figure_manifest['newNativeCohortExecutions'] == 3
assert figure_manifest['reusedBaselineNativeHistories'] == 3
assert figure_manifest['additionalForceExecutionsForFigure'] == 0

for name in ['acf_force_control_normal_trial.slx', 'joint_configs.mat', 'prepared_manifest.json']:
    frozen_copy(SOURCE / name, PORTABLE / name)
for helper in sorted((SOURCE / 'helpers').glob('*.m')):
    frozen_copy(helper, PORTABLE / 'helpers' / helper.name)
frozen_copy(recipe, PORTABLE / 'recipes' / recipe.name)
frozen_copy(ROOT / f'{STEM}_configuration.mat', PORTABLE / 'prepared_static_configurations.mat')
portable_files = [
    {'file': str(path.relative_to(PORTABLE)), 'sha256': sha(path)}
    for path in sorted(PORTABLE.rglob('*'))
    if path.is_file() and path.name != 'portable_input_manifest.json'
]
portable_manifest = {
    'status': 'portable_immutable_input_copy',
    'scope': 'Three new4s calibrated-static cases, paired with three original known-actual-k/c '
             'records. Copying and packaging add zero force simulations. Exact16 static '
             'residual changes and declared coefficient dependencies only; no main candidate change.',
    'recordedSourceDirectory': str(SOURCE),
    'originalCalibratedDonorDirectory': '/home/tommie/.local/state/actuator-model/'
                                       'tpba1927d1_c384_4398_a279_c8cfe77fab78',
    'recordedBaseV8NativeModelSHA256': new['baseV8NativeModelSHA256'],
    'calibratedNativeModelSHA256': new['sourceModelSHA256'],
    'jointConfigurationsSHA256': new['configurationSHA256'],
    'preparedStaticConfigurationsSHA256': new['preparedConfigurationSHA256'],
    'recipeSHA256': new['recipeSHA256'],
    'primitiveStaticResidualCount': 16,
    'physicalAndTimingParameterGuardsPassed': True,
    'files': portable_files,
    'externalRetainedNativeInputs': native_inputs,
    'externalRetainedBaselineResult': {'file': '../' + baseline_path.name, 'sha256': sha(baseline_path)},
    'reproduction': {
        'recipe': 'recipes/' + recipe.name,
        'arguments': ['This portable source folder', 'A fresh output folder',
                      'Parent validation folder containing the three original known-actual native files'],
        'isolation': 'MATLAB -batch with private MATLAB_PREFDIR; recipe creates private model/cache/code/SDI folders.',
        'baseline': 'Each original full r.parameters is loaded; configs are frozen before simulation.',
        'recordedMode': 'normal', 'nativeSamplesPerCase': 400001,
        'duration_s': 4, 'step_s': 1e-5,
    },
}
write_frozen(PORTABLE / 'portable_input_manifest.json', portable_manifest)

artifact_names = [
    f'{STEM}_results.json', f'{STEM}_configuration.mat', f'{STEM}_invocation_manifest.json',
    recipe.name, baseline_path.name,
    f'{FIGURE}.md', f'{FIGURE}.png', f'{FIGURE}.svg',
    f'{FIGURE}_chart_data.json', f'{FIGURE}_figure_manifest.json',
    'inertial_mechanics_make_calibrated_static_mixed_comparison_20260930.py',
    Path(__file__).name,
]
artifact_names += [Path(row['nativeTraceFile']).name for row in baseline['rows'] + new['rows']]
artifact_names += [str(PORTABLE.relative_to(ROOT) / entry['file']) for entry in portable_files]
artifact_names += [str(PORTABLE.relative_to(ROOT) / 'portable_input_manifest.json')]
artifacts = [{'file': name, 'sha256': sha(ROOT / name)} for name in artifact_names]
evidence = {
    'status': 'complete_source_bound_paired_evidence',
    'newNativeCases': 3, 'retainedBaselineCasesReused': 3, 'displayedNativeHistories': 6,
    'additionalForceSimulationsForPackagingOrFigure': 0,
    'caseScope': 'S2kg/F50N/90deg/mu0.3, joint calibration corner8, '
                 'actual lowerk+20%/c-20%, known actual inverse k/c, seeds7/29/1729,4s each.',
    'guardScope': 'Exactly16 static residuals halved; allowed identified mass/Kf and inactive '
                  'controller coefficient dependencies only. Every other primitive/derived '
                  'physical/timing coefficient is equal; native returned p equals frozen prepared p.',
    'originalWorstNormalForceError_N': baseline['worstNormalForceError_N'],
    'newWorstNormalForceError_N': new['worstNormalForceError_N'],
    'newWorstMargin_N': .5 - new['worstNormalForceError_N'],
    'calibratedNativeModelSHA256': new['sourceModelSHA256'],
    'recordedBaseV8NativeModelSHA256': new['baseV8NativeModelSHA256'],
    'nominalCandidateUnchanged': {'file': nominal_candidate.name, 'sha256': nominal_sha},
    'validationBasis': 'Executed native recipe assertions plus source/config/trace/figure SHA checks. '
                       'Calibration scope is three paired seeds, not a new full-grid or continuous-box qualification.',
    'originalInvocationStatusPreserved': launch['status'],
    'artifacts': artifacts,
}
write_frozen(ROOT / f'{FIGURE}_evidence_manifest.json', evidence)
print(json.dumps({'newNativeCases': 3, 'newWorst_N': new['worstNormalForceError_N'],
                  'margin_N': evidence['newWorstMargin_N'], 'artifacts': len(artifacts),
                  'portableManifest': str(PORTABLE / 'portable_input_manifest.json'),
                  'evidenceManifestSHA256': sha(ROOT / f'{FIGURE}_evidence_manifest.json')}))
