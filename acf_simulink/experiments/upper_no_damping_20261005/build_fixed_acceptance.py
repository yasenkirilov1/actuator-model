"""Bind current controller, voltage diagnostics and native checks to sources.

Replays, numerical refinement and post-outcome diagnostics are not additional
qualification episodes. Frozen input packages are never modified here.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'acf_simulink/validation/upper_no_damping_20261005'
QUAL = BASE / 'qualification_v8'
REPORT = BASE / 'report'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def add_source(path, sources, expected=None):
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    value = sha(path)
    assert expected is None or value == expected, path
    try:
        name = str(path.relative_to(ROOT))
    except ValueError:
        name = str(path)
    sources[name] = value


def write(name, kind, rows, sources, scope):
    document = {'status': 'complete', 'kind': kind, 'rows': rows,
                'sourceHashes': sources, 'scope': scope,
                'qualificationEpisodes': 0}
    (REPORT / (name + '.json')).write_text(json.dumps(document, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model-acceptance', type=Path)
    args = parser.parse_args()
    rows, sources = [], {}
    for folder, stage in (('paired_voltage_matlab', 'unclipped development example'),
                          ('paired_voltage_stress_725', 'post-outcome clipped stress diagnostic')):
        path = QUAL / folder / 'paired_voltage_check.json'
        data = json.loads(path.read_text())
        assert data['qualificationEpisodes'] == 0 and not data['controllerRetuned']
        assert data['acquisitionAndModalCoefficientsUnchanged'] and data['sameNativeTimeArray']
        assert data['actualCurrentResponse_s'] == data['identifiedModalCurrentResponse_s'] == 50e-6
        assert [r['usableVoltage_V'] for r in data['runs']] == [32, 48]
        add_source(path, sources)
        add_source(data['sourceModelFile'], sources, data['sourceModelSha256'])
        add_source(data['configsFile'], sources, data['configsSha256'])
        for run in data['runs']:
            assert run['algebraicLoopCount'] == 0
            add_source(run['historyFile'], sources, run['historySha256'])
            rows.append({**run, 'stage': stage, 'scenario': data['scenario0'],
                         'seed': data['seed'], 'controllerRetuned': False,
                         'actualAndIdentifiedCurrentResponse_s': 50e-6,
                         'waveformsExactlyEqualAcrossVoltages':
                             data['maximumForceWaveformDifference_N'] == 0 and
                             data['maximumCurrentWaveformDifference_A'] == 0 and
                             data['maximumVoltageWaveformDifference_V'] == 0})
    write('fixed_voltage_diagnostics', 'paired_independent_matlab_voltage_diagnostics',
          rows, sources, 'Two unchanged-controller 32/48 V pairs. The stress pair was requested after final outcomes; neither pair is qualification or controller selection evidence.')

    path = QUAL / 'selected_native_contracts_v1.json'
    data = json.loads(path.read_text())
    sources = {}
    add_source(path, sources)
    add_source(QUAL / 'bank/configs.mat', sources, data['configsSha256'])
    add_source(QUAL / 'candidate_lock.json', sources, data['candidateLockSha256'])
    add_source(QUAL / 'native/helpers/acf_adc_prepare.m', sources)
    pi, modal = data['selectedJointPIGainContract'], data['selectedCellModalContract']
    write('fixed_controller_contract', 'locked_executing_controller_parameters',
          [{'component': 'Live normal-force PI', **pi},
           {'component': 'Bounded phase-shaped cell-mode feedback', **modal['parameters']},
           {'component': 'Body-motion observer', **data['nominalQualificationCase0ObserverExample']['design']},
           {'component': 'Active acquisition and noise normalization',
            'matchedTwoPoleCutoff_Hz': 1900, 'adcOutputRate_Hz': 30000,
            'emaTimeConstant_s': 50e-6, 'emaExecutionClock_Hz': 30000,
            'nominalEmaLowFrequencyDelay_s': (1/30000) / math.expm1((1/30000) / 50e-6),
            'referenceNoiseENBW_Hz': 0.06354253123515036 / (2 * 10e-6),
            'referenceConditionedForceNoiseRms_N': 0.02,
            'referenceConditionedAccelerationNoiseRms_m_s2': 0.018655706922592906,
            'referenceConditionedPositionNoiseRms_m': 1e-6,
            'note': 'Input PSD normalization uses immutable nominal reference response; actual filtered RMS depends on the active response. ADC noise/quantization are additional.'}],
          sources, 'Selected live parameter fields and independent modal identification. Observer coefficients depend on known mass; inherited source-build case0 metadata does not identify the selected controller.')

    if args.model_acceptance is None:
        print('Controller/voltage sources verified; native delivery checks not requested.')
        return
    rows, sources = [], {}
    for folder in ('selected_development_worst_170', 'qualification_worst_673', 'qualification_worst_728'):
        path = QUAL / 'matlab_parity' / folder / 'native_replay.json'
        data = json.loads(path.read_text())
        assert data['sameSampleCount']
        assert data['maximumTimeDifference_s'] == 0 and data['algebraicLoopCount'] == 0
        assert len(data['truthPerChannelMaximumDifference']) == 27
        # Retain the original strict guard and its failures. This aggregate
        # reports engineering agreement separately; it never changes passed.
        force_difference = data['truthPerChannelMaximumDifference'][0]
        assert force_difference < 0.0001
        if folder == 'selected_development_worst_170':
            assert data['passed']
        add_source(path, sources)
        add_source(data['sourceRecording'], sources, data['sourceRecordingSha256'])
        # Historical replay schema calls the supplied comparison fixture
        # recordingMat; the independently simulated full MAT is a separate file.
        fixture = QUAL / 'independent_inputs' / (folder + '.mat')
        add_source(fixture, sources, data['recordingMatSha256'])
        add_source(path.with_name('native_recording.mat'), sources)
        rows.append({'check': 'Independent native MATLAB replay', **data,
                     'strictReplayPassed': data['passed'],
                     'maximumForceWaveformDifference_N': force_difference,
                     'forceFailureClassificationAgrees':
                         (data['nativeNormalPeak_N'] > 0.5) == (data['cNormalPeak_N'] > 0.5)})
    path = QUAL / 'solver_refinement/selected_development_worst_170/deterministic_refinement.json'
    data = json.loads(path.read_text())
    assert data['executedCorrectly'] and data['closedLoopDeterministicRefinement']
    assert data['algebraicLoopCount'] == 0 and abs(data['peakDifference_N']) < 0.0001
    assert data['rawNoiseClock_s'] == 1/300000 and data['rawAdcSampleTime_s'] == 1/30000
    assert data['controllerSampleTime_s'] == 1/10000
    add_source(path, sources)
    rows.append({'check': 'Closed-loop physical solver refinement', **data})
    data = json.loads(args.model_acceptance.read_text())
    assert data['status'] == 'versioned_fixed_mechanics_native_model_delivery_accepted'
    assert data['allConnectedEdgesPreserved'] and data['allModelWorkspaceValuesPreserved']
    assert data['defaultFullTruthArrayMaximumDifference'] == 0 and data['saveReopenExact']
    assert data['algebraicLoopCount'] == 0 and not data['externalInputNeeded']
    assert data['baselineHardSelected'] and data['scopeShowsWorkpieceNormalError']
    assert not data['scopeTruthUsedByController'] and data['namespaceOnlyHelpers']
    add_source(args.model_acceptance, sources)
    add_source(data['modelFile'], sources, data['modelSha256'])
    add_source(data['parentAcceptanceFile'], sources, data['parentAcceptanceSha256'])
    parent = json.loads(Path(data['parentAcceptanceFile']).read_text())
    for field in ('defaultNormalReferencePassed', 'directRunAfterParameterEditsPassed',
                  'saveReopenPassed', 'contactStopPassed', 'travelStopPassed', 'acceleratorPassed'):
        assert parent[field]
    model_folder = Path(data['modelFile']).parent
    bundle_manifest = model_folder / 'bundle_manifest.json'
    bundle = json.loads(bundle_manifest.read_text())
    assert bundle['status'] == 'versioned_fixed_mechanics_native_model_bundle_frozen'
    assert bundle['developmentOnly'] and not bundle['fullRequirementQualified']
    add_source(bundle_manifest, sources)
    for entry in bundle['files']:
        add_source(model_folder / entry['file'], sources, entry['sha256'])
    guide = ROOT / 'acf_simulink/experiments/upper_no_damping_20261005/qualification_v8/GETTING_STARTED_FIXED.md'
    add_source(guide, sources)
    rows.append({'check': 'Versioned standalone native model acceptance', **data})
    rows.append({'check': 'Standalone parent parameter edits, failure stops and accelerator acceptance', **parent})
    numerical_review = QUAL / 'matlab_parity/post_review_numerical_assessment_v1.json'
    assert numerical_review.exists(), 'Retain the independent coefficient/source review before delivery.'
    add_source(numerical_review, sources)
    assessment = json.loads(numerical_review.read_text())
    assert assessment['completed'] and assessment['conclusion']['sourceAndParameterAuditPassed']
    assert not assessment['originalStrictResultFlagsModified']
    assert assessment['originalStrictReplayGuard']['bothOriginalResultsRemainFailed']
    for entry in assessment['provenance']['sourceAndParameterBindings']:
        add_source(entry['path'], sources, entry['sha256'])
    for entry in assessment['numericalReassessment']['results']:
        add_source(entry['independentInputPath'], sources, entry['independentInputSha256'])
        add_source(entry['matlabFullHistoryPath'], sources, entry['matlabFullHistorySha256'])
        add_source(entry['originalStrictResultPath'], sources, entry['originalStrictResultSha256'])
    rows.append({'check': 'Post-review numerical assessment; original strict guards retained',
                 **assessment})
    write('fixed_native_verification', 'independent_native_and_delivery_checks', rows, sources,
          'Completed independent native replay checks retain their original strict guard results, including two failures. Engineering waveform differences, one solver refinement and standalone model acceptance are separate. These checks do not turn failed force cases into passes or establish hardware performance.')
    print('Controller, voltage diagnostics and native delivery checks verified.')


if __name__ == '__main__':
    main()
