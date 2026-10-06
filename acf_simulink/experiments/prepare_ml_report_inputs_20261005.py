"""Expose reviewed ML study rows without modifying frozen worker evidence."""
import hashlib
import json
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VAL = ROOT / 'acf_simulink/validation'
OUT = VAL / 'ml_report_20261005'
NOTES = Path('/home/tommie/Documents/Notes Remote/S7 Project')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(relative):
    return json.loads((ROOT / relative).read_text())


def reviewed(name, rows, sources, scope):
    assert rows and isinstance(rows, list)
    doc = {'status': 'complete', 'kind': 'reviewed_native_study_excerpt',
           'scope': scope, 'rows': rows,
           'sources': [{'file': f, 'sha256': digest(ROOT / f)} for f in sources]}
    path = OUT / (name + '.json')
    path.write_text(json.dumps(doc, indent=2, allow_nan=False) + '\n')
    return str(path.relative_to(ROOT))


def main():
    OUT.mkdir(exist_ok=True)
    force_file = 'acf_simulink/validation/ml_force_20261005/summary.json'
    force_training_file = 'acf_simulink/validation/ml_force_20261005/training.json'
    force = read(force_file)
    assert force['status'] == 'complete_finite_native_holdout'
    assert force['holdoutPairedTrajectories'] == 12
    force_rows = []
    for pair in force['pairs']:
        b, learned = pair['baseline'], pair['learned']
        assert b['executed'] and learned['executed']
        assert b['gain'] == 0 and learned['gain'] == 1
        force_rows.append({
            'group': pair['group'], 'id': pair['id'], 'massS_kg': b['massS_kg'],
            'force_N': b['reference_N'], 'angle_deg': b['angle_deg'], 'seed': b['seed'],
            'duration_s': b['endTime_s'], 'baseline_peak_N': b['peakNormalForceError_N'],
            'learned_peak_N': learned['peakNormalForceError_N'], 'change_N': pair['peakChange_N'],
            'baseline_rms_N': b['rmsNormalForceError_N'], 'learned_rms_N': learned['rmsNormalForceError_N'],
            'baseline_passed': b['passed'], 'learned_passed': learned['passed'],
            'baseline_native_samples': int(b['nativeSamples']), 'learned_native_samples': int(learned['nativeSamples']),
            'learned_peak_current_A': learned['peakCurrent_A'], 'learned_peak_voltage_V': learned['peakVoltage_V'],
            'learned_peak_residual_A': learned['peakResidualCurrent_A'],
        })
    force_scope = 'Twelve untouched paired native trajectories; whole-history normal-force scores including startup. Finite nominal and selected calibration/mechanics tests; not hardware or full-grid qualification.'
    force_query_file = reviewed('force_holdout_reviewed', force_rows,
                                [force_file, 'acf_simulink/validation/ml_force_20261005/final_checks.json'], force_scope)
    force_training = read(force_training_file)
    force_training_scope = '180-epoch supervised approximate-current teacher fit. Whole-trajectory training/selection split. Teacher fit is not closed-loop force qualification.'
    force_training_query_file = reviewed('force_training_reviewed', force_training['history'],
                                         [force_training_file, 'acf_simulink/validation/ml_force_20261005/selected_gain.json'], force_training_scope)

    robot_file = 'acf_simulink/validation/ml_robot_20261005_figures_final/summary.json'
    robot_native = 'acf_simulink/validation/ml_robot_20261005_heldout/results.json'
    robot_training_file = 'acf_simulink/validation/ml_robot_20261005_training/training_summary.json'
    robot = read(robot_file)
    assert robot['status'] == 'paired_native_results_audited'
    assert robot['heldout_identities'] == 11 and read(robot_native)['status'] == 'complete'
    robot_scope = 'Eleven untouched robot-policy pairs, with unchanged deterministic fast force control and hard safety constraints. Eight nominal six-second pairs and three selected eight-second stress pairs; no full-grid or hardware qualification.'
    robot_query_file = reviewed('robot_holdout_reviewed', robot['paired_results'],
                                [robot_file, robot_native], robot_scope)
    robot_training = read(robot_training_file)
    robot_training_scope = 'Native logged recenter-law imitation with eight training and four selection trajectories. Raw-network teacher reproduction checked separately from the hard envelope. No test trajectories used for fitting.'
    robot_training_query_file = reviewed('robot_training_reviewed', robot_training['sources'],
                                         [robot_training_file], robot_training_scope)

    combined_file = 'acf_simulink/validation/ml_combined_20261005_figures/summary.json'
    combined_native = 'acf_simulink/validation/ml_combined_20261005/results.json'
    combined = read(combined_file)
    assert combined['status'] == 'complete' and combined['identities'] == 4
    assert read(combined_native)['status'] == 'complete'
    combined_scope = 'Four new predeclared eight-second pairs. Fixed independently trained networks and force gain 1. Both actions active; no joint/end-to-end training, retuning, full uncertainty-box or physical qualification.'
    combined_query_file = reviewed('combined_holdout_reviewed', combined['rows'],
                                   [combined_file, combined_native], combined_scope)

    query_specs = [
        ('ml_force_training', force_training_query_file, [force_training_file, 'acf_simulink/validation/ml_force_20261005/selected_gain.json'], 'Force-network training and independent gain selection', force_training_scope),
        ('ml_force_test', force_query_file, [force_file, 'acf_simulink/validation/ml_force_20261005/final_checks.json'], 'Force-network paired native holdout and implementation checks', force_scope),
        ('ml_robot_training', robot_training_query_file, [robot_training_file], 'Robot-network trajectory-level training and selection', robot_training_scope),
        ('ml_robot_test', robot_query_file, [robot_file, robot_native], 'Robot-network paired native holdout', robot_scope),
        ('ml_combined_test', combined_query_file, [combined_file, combined_native], 'Two fixed learned policies in the same native closed loop', combined_scope),
    ]
    queries = [{'id': name, 'file': file, 'sourceFiles': sources, 'label': label,
                'kind': 'reviewed_native_experiment', 'caveats': [scope]}
               for name, file, sources, label, scope in query_specs]
    figure_sources = [
        ('ml_force_20261005/figures/ml_force_training.svg', 'ml_force_training'),
        ('ml_force_20261005/figures/ml_force_holdout_peaks.svg', 'ml_force_test'),
        ('ml_force_20261005/figures/ml_force_worst_bump.svg', 'ml_force_test'),
        ('ml_robot_20261005_figures_final/ml_robot_learned_policy.svg', 'ml_robot_training'),
        ('ml_robot_20261005_figures_final/ml_robot_paired_results.svg', 'ml_robot_test'),
        ('ml_robot_20261005_figures_final/ml_robot_active_trajectory.svg', 'ml_robot_test'),
        ('ml_combined_20261005_figures/ml_combined_paired_results.svg', 'ml_combined_test'),
        ('ml_combined_20261005_figures/ml_combined_worst_trajectory.svg', 'ml_combined_test'),
    ]
    figures = []
    for source, query in figure_sources:
        p = VAL / source
        target = NOTES / 'figures' / p.name
        if target.exists():
            assert target.read_bytes() == p.read_bytes(), f'Preserve unrelated figure: {target}'
        else:
            shutil.copy2(p, target)
        figures.append({'file': 'figures/' + p.name, 'queryId': query,
                        'sourceFile': str(p.relative_to(ROOT)), 'sourceSha256': digest(p)})
    (OUT / 'ml_report_inputs.json').write_text(json.dumps({'status': 'complete', 'queries': queries,
                                                        'figures': figures}, indent=2) + '\n')
    print(json.dumps({'reviewedQueries': len(queries), 'copiedSvgFigures': len(figures),
                      'untouchedPairedTestRecordings': 24 + 22 + 8}))


if __name__ == '__main__':
    main()
