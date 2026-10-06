"""Compact figures and arithmetic checks for the member-script audit.

Reads two downsampled diagnostic traces and scalar JSON. Never loads native
MAT/NPZ histories, changes a model, or runs the attached GUI.
"""
from pathlib import Path
import csv
import hashlib
import json
import math

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
VAL = ROOT / 'acf_simulink/validation/member_script_review_20261006'
FIG = ROOT / 'reports/reviews/figures'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    FIG.mkdir(parents=True, exist_ok=True)
    checks = json.loads((VAL / 'deterministic_checks.json').read_text())
    rows = checks['rows']
    trace = {'flat_robot_pid_off': [], 'flat_robot_pid_on': []}
    for row in csv.DictReader((VAL / 'robot_trace_downsampled.csv').open()):
        trace[row['case']].append((float(row['time_s']), float(row['contact_error_N'])))

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11,
                         'svg.fonttype': 'none', 'svg.hashsalt': 'member-review-20261006'})
    fig, ax = plt.subplots(figsize=(10, 4.5))
    colors = ['#b85032', '#087f8c']
    for name, label, color in zip(trace, ['PID off; constant-current feedforward retained', 'PID on'], colors):
        t, force = np.array(trace[name]).T
        chosen = (t >= 5) & (t <= 7.5)
        ax.plot(t[chosen], force[chosen], label=label, color=color, linewidth=1.6)
    ax.set(xlabel='Time (s)', ylabel='Contact-force error (N)', ylim=(-.03, .03))
    ax.set_title('Most robot-wobble isolation is already present with PID off', fontsize=14, pad=14)
    ax.grid(color='#d9e2e7', linewidth=.8)
    ax.spines[['top', 'right']].set_visible(False)
    ax.legend(loc='lower left', fontsize=10)
    suppression = 20 * math.log10(rows[0]['robotFrequencyAmplitude_N'] / rows[1]['robotFrequencyAmplitude_N'])
    fig.text(.10, .105, f"1.2 Hz force amplitude: {rows[0]['robotFrequencyAmplitude_N']:.4f} → {rows[1]['robotFrequencyAmplitude_N']:.4f} N; extra PID rejection {suppression:.2f} dB.", fontsize=10)
    fig.text(.10, .04, 'Deterministic C transcription; flat surface, 0.15 mm robot wobble, no random noise or spindle forcing.\n'
             'This isolates the PID contribution. It is not an executed rigid-flange comparison or a MATLAB/hardware qualification.', fontsize=9)
    fig.subplots_adjust(left=.10, right=.97, top=.84, bottom=.27)
    robot_path = FIG / 'member-robot-isolation-check-20261006.svg'
    fig.savefig(robot_path, metadata={'Date': None})
    plt.close(fig)

    radius = .077 / 2
    offsets = np.linspace(-radius, radius, 35)
    strip = 2 * np.sqrt(np.maximum(0, radius**2 - offsets**2))
    bump = 2 * np.sqrt(np.maximum(0, .008**2 - offsets**2))
    fraction = float(np.sum(np.minimum(1, bump / np.maximum(.001, strip)) * strip / np.sum(strip)))
    mean_height = .00015 * fraction
    effective_force = 20000 * mean_height
    fig, axes = plt.subplots(1, 2, figsize=(10.5, 5))
    axes[0].bar([0, 1], [3, effective_force], color=['#b85032', '#087f8c'], width=.55)
    axes[0].set_xticks([0, 1], ['Peak height applied\nto whole spring', '77 mm pad\narea weighting'])
    axes[0].set(ylabel='Instant elastic-force increment (N)', ylim=(0, 3.8))
    axes[0].set_title('Its pad substantially reduces the input disturbance', fontsize=11)
    axes[0].text(0, 3.10, '3.000 N', ha='center')
    axes[0].text(1, effective_force + .10, f'{effective_force:.3f} N', ha='center')
    axes[0].text(.5, 3.5, '0.15 mm feature; 20 kN/m stiffness', ha='center', fontsize=9, color='#455361')
    axes[1].bar([0, 1], [.5, 0], color=['#087f8c', '#b85032'], width=.55)
    axes[1].set_xticks([0, 1], ['Check contact locally,\nthen sum forces', 'Average height,\nthen check contact'])
    axes[1].set(ylabel='Predicted contact force (N)', ylim=(0, .7))
    axes[1].set_title('Averaging first fails when contact is partial', fontsize=11)
    axes[1].text(0, .52, '0.500 N', ha='center')
    axes[1].text(1, .02, '0.000 N', ha='center')
    axes[1].text(.5, .63, 'Half the area: +0.05 mm compression\nOther half: 0.05 mm gap', ha='center', fontsize=9, color='#455361')
    for ax in axes:
        ax.grid(axis='y', color='#d9e2e7', linewidth=.8)
        ax.set_axisbelow(True)
        ax.spines[['top', 'right']].set_visible(False)
    fig.suptitle('Pad averaging can be real physics; contact averaging needs care', fontsize=14, y=.98)
    fig.text(.08, .095, 'Left: scalar audit of the source’s centred 16 mm circular feature, with the tool held fixed and full contact.\n'
             'Right: two equal-area regions of one strip; a unilateral spring cannot pull across a gap.', fontsize=9)
    fig.text(.08, .025, 'Neither panel is a simulation of our sharp-bump flange. These checks explain the changed disturbance and a contact-model error.',
             fontsize=9, color='#455361')
    fig.subplots_adjust(left=.08, right=.97, top=.80, bottom=.30, wspace=.38)
    pad_path = FIG / 'member-pad-contact-check-20261006.svg'
    fig.savefig(pad_path, metadata={'Date': None})
    plt.close(fig)

    result = {
        'status': 'bounded_audit_checks_complete',
        'kind': 'arithmetic_and_deterministic_code_transcription_not_qualification',
        'gaussianNoiseDisabled': True, 'nativeMatlabReplay': False,
        'robotExtraPidRejection_dB': suppression,
        'padNominal': {'diameter_m': .077, 'circularFeatureDiameter_m': .016,
                       'height_m': .00015, 'areaWeightedFraction35strips': fraction,
                       'areaWeightedHeight_m': mean_height, 'elasticIncrement_N': effective_force,
                       'unaveragedWholeSpringIncrement_N': 3},
        'partialContactCounterexample': {'areaFractions': [.5, .5],
                         'localCompression_m': [.00005, -.00005],
                         'stiffness_N_m': 20000, 'localContactThenIntegrate_N': .5,
                         'heightAverageThenClip_N': 0,
                         'scope': 'One mixed-height strip; geometry arithmetic, not a recorded default-state failure.'},
        'electricalBounds': {'named24VDriverMaximumOutput_V': 21.6,
                            'sourcePeak202p3NColdHoldVoltage_V': 202.3 / 25.6 * 5.7,
                            'source50NHorizontalCurrent_A': 50 / 25.6,
                            'ratedContinuousCurrent_A': 1.8,
                            'scriptAdcCode_N': 50 / 4096,
                            'eposAnalogCodeAt100Nper10V_N': .00564 * 100 / 10},
        'sourceHashes': {str(p.relative_to(ROOT)): sha(p) for p in [
            ROOT / 'acf_simulink/experiments/member_script_review_20261006/check_member_equations.c',
            Path(__file__), VAL / 'deterministic_checks.json', VAL / 'robot_trace_downsampled.csv']},
        'figures': [{'file': str(p.relative_to(ROOT)), 'sha256': sha(p)} for p in [robot_path, pad_path]]}
    (VAL / 'review_checks.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({'figures': len(result['figures']), 'extraPidRejection_dB': suppression,
                      'readTraceSamples': sum(len(x) for x in trace.values()), 'newQualificationRuns': 0}))


if __name__ == '__main__':
    main()
