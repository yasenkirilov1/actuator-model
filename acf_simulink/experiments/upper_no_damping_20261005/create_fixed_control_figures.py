"""Make compact, source-bound SVG figures from complete native histories.

Display reduction preserves the min/max in each time bin. Metrics and spectra
use the unreduced samples. Historical acquisition comparisons are explicitly
separate from the currently prescribed fixed mechanics.
"""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
from scipy.signal import periodogram
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[3]
FIGURES = ROOT / 'reports/figures'
REPORT = ROOT / 'acf_simulink/validation/upper_no_damping_20261005/report'
BLUE, TEAL, RED, INK = '#2563eb', '#008b83', '#dc493a', '#25364b'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def source_key(path):
    try:
        return str(Path(path).relative_to(ROOT))
    except ValueError:
        return str(path)


def load_trace(result_path):
    result = json.loads(result_path.read_text())
    history = Path(result['historyFile'])
    assert sha(history) == result['historySha256']
    with np.load(history) as z:
        t, d = z['time'], z['truth']
    error = d[:, 0] - result['force_N']
    assert abs(np.max(np.abs(error)) - result['peakNormalForceError_N']) < 1e-12
    assert len(t) == result['nativeSamples'] and result['nativeGridComplete']
    return result, history, t, error


def force_comparison(result_path, reference_path):
    selected, selected_history, t, error = load_trace(result_path)
    original, original_history, t0, error0 = load_trace(reference_path)
    for key in ('massA_kg', 'massB_kg', 'massS_kg', 'force_N', 'angle_rad', 'seed',
                'actualK_N_m', 'actualC_N_s_m', 'upperK_N_m', 'upperC_N_s_m',
                'bumpHeight_m', 'bumpWidth_m', 'scanSpeed_m_s'):
        assert selected[key] == original[key], key
    assert np.array_equal(t, t0)
    post = (t >= 1.1) & (t < 1.35)
    fs = 1 / np.median(np.diff(t))
    spectra = [periodogram(e[post], fs=fs, window='hann', detrend='constant',
                           scaling='density') for e in (error0, error)]
    f = spectra[0][0]
    band = (f >= 1300) & (f <= 1900)
    power = [float(np.sum(s[1][band])) for s in spectra]
    reductions = 10 * np.log10(power[0] / power[1])
    fractions = [float(np.sum(s[1][band]) / np.sum(s[1])) for s in spectra]
    fig = plt.figure(figsize=(11.8, 6.1))
    grid = fig.add_gridspec(2, 2, height_ratios=[1.15, 1], hspace=.48, wspace=.26)
    ax = fig.add_subplot(grid[0, :])
    ax.fill_between([.98, 1.35], -.5, .5, color=TEAL, alpha=.09)
    ax.axvspan(1, 1.1, color=BLUE, alpha=.06)
    view = (t >= .98) & (t <= 1.35)
    for e, color, label in ((error0, '#94a3b8', 'Before mode control'),
                            (error, BLUE, 'Selected deterministic controller')):
        tx, ey = extrema(t[view], e[view])
        ax.plot(tx, ey, color=color, linewidth=.8, label=label)
    for limit in (-.5, .5):
        ax.axhline(limit, color=RED, linestyle='--', linewidth=1)
    ax.set(xlim=(.98, 1.35), ylabel='Normal force error (N)', xlabel='Time (s)')
    ax.set_title(f"Same sharp bump: {selected['massS_kg']:g} kg tool, {selected['force_N']:g} N, "
                 f"{np.rad2deg(selected['angle_rad']):g}°, A = {selected['massA_kg']:g} kg",
                 loc='left', fontweight='bold', pad=11)
    ax.legend(frameon=False, loc='upper right', fontsize=9)
    ax.text(.99, ax.get_ylim()[0] * .82,
            f"Whole-record peaks: {original['peakNormalForceError_N']:.3f} → "
            f"{selected['peakNormalForceError_N']:.3f} N", color=INK, fontsize=10)
    left = max(1.1001, original['peakErrorTime_s'] - .003)
    zoom = (t >= left) & (t <= left + .008)
    ax = fig.add_subplot(grid[1, 0])
    for e, color in ((error0, '#94a3b8'), (error, BLUE)):
        ax.plot((t[zoom] - left) * 1000, e[zoom], color=color, linewidth=.9)
    for limit in (-.5, .5):
        ax.axhline(limit, color=RED, linestyle='--', linewidth=.8)
    ax.set(xlabel='Time within the zoom (ms)', ylabel='Force error (N)')
    ax.set_title('Post-bump ringing is reduced', loc='left', fontweight='bold', pad=9)
    ax = fig.add_subplot(grid[1, 1])
    view = (f >= 100) & (f <= 3500)
    for spectrum, color in zip(spectra, ('#94a3b8', BLUE)):
        ax.semilogy(f[view] / 1000, spectrum[1][view], color=color, linewidth=1.3)
    ax.axvspan(1.3, 1.9, color=TEAL, alpha=.09)
    ax.set(xlabel='Frequency (kHz)', ylabel='Force spectral density (N²/Hz)', xlim=(.1, 3.5))
    ax.set_title(f'Cell-band spectral power reduced by {reductions:.1f} dB',
                 loc='left', fontweight='bold', pad=9)
    for ax in fig.axes:
        ax.spines[['top', 'right']].set_visible(False)
        ax.grid(axis='y', alpha=.16)
    out = finish(fig, 'fixed-mechanics-force-limit-20261005.svg')
    out['queryId'] = 'fixed_control_waveform'
    rows = []
    for label, result, fraction in zip(('before_mode_control', 'selected_controller'),
                                       (original, selected), fractions):
        rows.append({**{k: result[k] for k in ('identity', 'massS_kg', 'massA_kg', 'force_N',
                     'angle_rad', 'nativeSamples', 'peakNormalForceError_N', 'peakErrorTime_s',
                     'passed', 'peakVoltage_V', 'usableVoltage_V', 'voltageClipSamples')},
                     'controller': label, 'modeVarianceFraction': fraction,
                     'modeBand_Hz': [1300, 1900], 'spectrumWindow_s': [1.1, 1.35]})
    sources = {source_key(p): sha(p) for p in (result_path, reference_path,
                                               selected_history, original_history)}
    contract = {'status': 'complete', 'kind': 'paired_native_controller_waveform_and_spectral_analysis',
                'rows': rows, 'sourceHashes': sources, 'cellBandSpectralPowerReduction_dB': reductions,
                'displayReduction': 'Min/max per bin; full original samples for metrics and spectra',
                'scope': 'Matched development case and seed; controller comparison is not whole-range or hardware qualification'}
    (REPORT / 'fixed_control_waveform.json').write_text(json.dumps(contract, indent=2) + '\n')
    return out


def extrema(t, y, maximum_points=5000):
    if len(t) <= maximum_points:
        return t, y
    boundaries = np.linspace(0, len(t), maximum_points // 2 + 1, dtype=int)
    selected = [0, len(t) - 1]
    for left, right in zip(boundaries[:-1], boundaries[1:]):
        if right > left:
            selected += [left + int(np.argmin(y[left:right])),
                         left + int(np.argmax(y[left:right]))]
    selected = np.unique(selected)
    return t[selected], y[selected]


def finish(fig, filename):
    path = FIGURES / filename
    fig.savefig(path, bbox_inches='tight', facecolor='white', metadata={'Date': None})
    plt.close(fig)
    return {'file': 'figures/' + filename, 'sha256': sha(path)}


def force_figure(result_path):
    result = json.loads(result_path.read_text())
    history = Path(result['historyFile'])
    assert sha(history) == result['historySha256']
    with np.load(history) as z:
        t, d = z['time'], z['truth']
    error = d[:, 0] - result['force_N']
    assert abs(np.max(np.abs(error)) - result['peakNormalForceError_N']) < 1e-12
    assert len(t) == result['nativeSamples'] and result['nativeGridComplete']
    post = (t >= 1.1) & (t < 1.35)
    f, spectrum = periodogram(error[post], fs=1 / np.median(np.diff(t)),
                             window='hann', detrend='constant', scaling='density')
    total = float(np.sum(spectrum))
    band_fraction = float(np.sum(spectrum[(f >= 1300) & (f <= 1900)]) / total)
    dominant = float(f[np.argmax(spectrum)])
    fig = plt.figure(figsize=(11.8, 6.1))
    grid = fig.add_gridspec(2, 2, height_ratios=[1.15, 1], hspace=.47, wspace=.26)
    ax = fig.add_subplot(grid[0, :])
    view = (t >= .98) & (t <= 1.18)
    tx, ey = extrema(t[view], error[view])
    ax.fill_between([.98, 1.18], -.5, .5, color=TEAL, alpha=.09)
    ax.axvspan(1, 1.1, color=BLUE, alpha=.06)
    ax.plot(tx, ey, color=BLUE, linewidth=.85)
    ax.axhline(.5, color=RED, linestyle='--', linewidth=1)
    ax.axhline(-.5, color=RED, linestyle='--', linewidth=1)
    ax.set(xlim=(.98, 1.18), ylabel='Normal force error (N)', xlabel='Time (s)')
    ax.set_title(f"Sharp bump: {result['massS_kg']:g} kg tool, {result['force_N']:g} N target, "
                 f"{np.rad2deg(result['angle_rad']):g}° orientation, A = {result['massA_kg']:g} kg",
                 loc='left', fontweight='bold', pad=11)
    peak = int(np.argmax(np.abs(error)))
    ax.scatter([t[peak]], [error[peak]], color=RED, s=20, zorder=5)
    ax.annotate(f"{abs(error[peak]):.3f} N peak", xy=(t[peak], error[peak]),
                xytext=(t[peak] + .010, error[peak] * .6), fontsize=10,
                arrowprops={'arrowstyle': '->', 'color': INK})
    ax.text(1.05, ax.get_ylim()[1] * .86, 'Bump crossing', ha='center', color=BLUE, fontsize=10)
    ax = fig.add_subplot(grid[1, 0])
    left = max(1.1001, result['peakErrorTime_s'] - .003)
    view = (t >= left) & (t <= left + .008)
    ax.plot((t[view] - left) * 1000, error[view], color=BLUE, linewidth=1)
    ax.axhline(.5, color=RED, linestyle='--', linewidth=.8)
    ax.axhline(-.5, color=RED, linestyle='--', linewidth=.8)
    ax.set(xlabel='Time within the zoom (ms)', ylabel='Force error (N)')
    ax.set_title('Rapid ringing after the bump', loc='left', fontweight='bold', pad=9)
    ax = fig.add_subplot(grid[1, 1])
    view = (f >= 100) & (f <= 3500)
    ax.semilogy(f[view] / 1000, spectrum[view], color=TEAL, linewidth=1.5)
    ax.axvspan(1.3, 1.9, color=TEAL, alpha=.09)
    ax.set(xlabel='Frequency (kHz)', ylabel='Force spectral density (N²/Hz)', xlim=(.1, 3.5))
    ax.set_title(f'{100 * band_fraction:.1f}% of variance in the cell-mode band',
                 loc='left', fontweight='bold', pad=9)
    ax.annotate(f'{dominant / 1000:.2f} kHz', xy=(dominant / 1000, spectrum.max()),
                xytext=(2.0, spectrum.max() * .1),
                arrowprops={'arrowstyle': '->', 'color': INK}, color=INK)
    for ax in fig.axes:
        ax.spines[['top', 'right']].set_visible(False)
        ax.grid(axis='y', alpha=.16)
    out = finish(fig, 'fixed-mechanics-force-limit-20261005.svg')
    out['queryId'] = 'fixed_control_waveform'
    row = {'identity': result['identity'], 'forceTarget_N': result['force_N'],
           'toolMassS_kg': result['massS_kg'], 'massA_kg': result['massA_kg'],
           'orientation_deg': float(np.rad2deg(result['angle_rad'])),
           'nativeSamples': len(t), 'peakNormalForceError_N': result['peakNormalForceError_N'],
           'peakErrorTime_s': result['peakErrorTime_s'], 'passed': result['passed'],
           'peakVoltage_V': result['peakVoltage_V'], 'usableVoltage_V': result['usableVoltage_V'],
           'voltageClipSamples': result['voltageClipSamples'],
           'spectrumWindow_s': [1.1, 1.35], 'dominantFrequency_Hz': dominant,
           'modeBand_Hz': [1300, 1900], 'modeVarianceFraction': band_fraction,
           'displayReduction': 'Min/max preserved in each bin; spectrum and metrics use all native points'}
    sources = {source_key(p): sha(p) for p in (result_path, history)}
    contract = {'status': 'complete', 'kind': 'native_development_waveform_and_spectral_analysis',
                'rows': [row], 'sourceHashes': sources,
                'scope': 'An executed development case, not whole-range or physical qualification'}
    (REPORT / 'fixed_control_waveform.json').write_text(json.dumps(contract, indent=2) + '\n')
    return out


def acquisition_figure():
    previous = ROOT / 'acf_simulink/validation/budget_adc_long_summary_20261002.json'
    document = json.loads(previous.read_text())
    assert document['status'] == 'complete'
    settings = [(15000, 8000), (30000, 5100), (30000, 8000)]
    selected = [next(r for r in document['rows'] if r['converter'] == 'sinc3'
                     and (r['rate_Hz'], r['cutoff_Hz']) == setting) for setting in settings]
    assert all(r['recordings'] == r['passes'] == 4 for r in selected)
    pairs = [(f"{row['rate_Hz'] // 1000}k samples/s\n{row['cutoff_Hz'] / 1000:g} kHz response",
              row['peakError_N']) for row in selected]
    x = np.arange(3)
    fig, ax = plt.subplots(figsize=(10.8, 3.6))
    ax.bar(x, [v for _, v in pairs], width=.55, color=[BLUE, TEAL, TEAL])
    ax.axhline(.5, color=RED, linestyle='--', linewidth=1.3, label='0.5 N requirement')
    ax.axhline(.45, color=INK, linestyle=':', linewidth=1.3, label='0.45 N design target')
    for i, (_, value) in enumerate(pairs):
        ax.text(i, value + .005, f'{value:.3f} N', ha='center', fontsize=11, color=INK)
    ax.set(xticks=x, xticklabels=[s for s, _ in pairs], ylim=(0, .58),
           ylabel='Worst force error (N)')
    ax.set_title('Why retain a 30 ksample/s ADC clock?', loc='left', fontweight='bold', pad=12)
    ax.text(.5, -.28, 'Historical ADS131M08 comparison: four 30 s recordings per setting; earlier compliant lower link',
            transform=ax.transAxes, ha='center', fontsize=10, color=INK)
    ax.legend(loc='upper right', frameon=False, ncol=2, fontsize=9)
    ax.spines[['top', 'right']].set_visible(False)
    ax.grid(axis='y', alpha=.16)
    out = finish(fig, 'adc-rate-evidence-compact-20261005.svg')
    out['queryId'] = 'fixed_acquisition_reference'
    sources = {str(previous.relative_to(ROOT)): sha(previous)}
    rows = [{'converter': 'ADS131M08', 'samplesPerSecond': n, 'responseCutoff_Hz': cutoff,
             'recordingLength_s': 30, 'cases': 4, 'passed': 4,
             'worstNormalForceError_N': value, 'mechanics': 'Earlier upper-rigid/lower-compliant reference'}
            for n, cutoff, (_, value) in zip([15000, 30000, 30000], [8000, 5100, 8000], pairs)]
    (REPORT / 'fixed_acquisition_reference.json').write_text(json.dumps({
        'status': 'complete', 'kind': 'historical_native_acquisition_comparison',
        'rows': rows, 'sourceHashes': sources,
        'scope': 'Explains acquisition tradeoffs; does not qualify the newly fixed upper-spring/rigid-lower design'
    }, indent=2) + '\n')
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--result', type=Path, required=True)
    parser.add_argument('--reference', type=Path)
    args = parser.parse_args()
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'text.color': INK, 'axes.labelcolor': INK,
                         'xtick.color': INK, 'ytick.color': INK,
                         'svg.fonttype': 'none', 'svg.hashsalt': 'fixed-mechanics-20261005'})
    force = (force_comparison(args.result.resolve(), args.reference.resolve())
             if args.reference else force_figure(args.result.resolve()))
    figures = [force, acquisition_figure()]
    (REPORT / 'compact_figure_manifest.json').write_text(json.dumps(figures, indent=2) + '\n')
    print(json.dumps(figures))


if __name__ == '__main__':
    main()
