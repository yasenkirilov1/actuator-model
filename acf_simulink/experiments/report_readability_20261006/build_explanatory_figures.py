"""Make requested SVG explanations from frozen contracts and recorded results.

Schematic motion is explicitly illustrative. Qualification values come from
all 700 recorded nominal cases, with seven fixed gravity orientations per cell.
"""
from pathlib import Path
import hashlib
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / 'acf_simulink/validation/report_readability_20261006/report'
FIG = ROOT / 'reports/figures'
CONTRACT = ROOT / 'acf_simulink/validation/upper_no_damping_20261005/report/fixed_controller_contract.json'
QUAL = ROOT / 'acf_simulink/validation/upper_no_damping_20261005/qualification_v8/evaluation/nominal700/results.json'
BLUE, ORANGE, INK, GRAY = '#2865b9', '#b1672e', '#253849', '#647689'
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 11.5,
                     'axes.titlesize': 14, 'axes.labelsize': 12,
                     'svg.fonttype': 'none', 'savefig.facecolor': 'white'})


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def write_query(name, rows, sources, **extra):
    output = {'status': 'complete', 'rows': rows,
              'sourceHashes': {str(p.relative_to(ROOT)): sha(p) for p in sources}, **extra}
    path = OUT / name
    path.write_text(json.dumps(output, indent=2) + '\n')
    return path


def save(fig, name):
    path = FIG / name
    preview = OUT / 'previews'
    preview.mkdir(exist_ok=True)
    fig.savefig(preview / Path(name).with_suffix('.png'), bbox_inches='tight', dpi=130)
    fig.savefig(path, bbox_inches='tight', metadata={'Date': None})
    plt.close(fig)
    return path


def box(ax, x, y, w, h, label, color=BLUE):
    ax.add_patch(Rectangle((x, y), w, h, facecolor='#f5f8fb', edgecolor=color, linewidth=1.3))
    ax.text(x + w / 2, y + h / 2, label, ha='center', va='center', color=INK)


def arrow(ax, x0, y0, x1, y1, color=INK, **kwargs):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle='-|>',
                                mutation_scale=14, color=color, linewidth=1.3, **kwargs))


def mode_figure():
    document = json.loads(CONTRACT.read_text())
    modal = next(r for r in document['rows'] if r['component'] == 'Bounded phase-shaped cell-mode feedback')
    f = modal['carrier_Hz']
    stiffness, holder = 64e6, .6
    assert abs(f - np.sqrt(stiffness / holder) / (2 * np.pi)) < 1e-9
    fig, axes = plt.subplots(2, 1, figsize=(11.8, 6.7), gridspec_kw={'height_ratios': [1.1, 1]})
    ax = axes[0]
    ax.set(xlim=(0, 12), ylim=(0, 3.6))
    ax.axis('off')
    ax.set_title('What oscillates in the load-cell mode?', loc='left', fontweight='bold')
    box(ax, .2, 1.35, 1.65, .95, 'Body A')
    box(ax, 2.65, 1.35, 1.8, .95, 'Holder B\n0.6 kg')
    box(ax, 5.35, 1.5, 2.1, .65, 'Finite-stiffness\nload cell', ORANGE)
    box(ax, 8.35, 1.35, 1.75, .95, 'Tool S')
    ax.plot([10.7, 10.7], [.85, 2.85], color=INK, linewidth=3)
    for l, r in [(1.85, 2.65), (4.45, 5.35), (7.45, 8.35), (10.1, 10.7)]:
        ax.plot([l, r], [1.82, 1.82], color=INK, linewidth=1.4)
    ax.text(2.25, 2.56, 'Coil force', ha='center', color=GRAY)
    arrow(ax, 2.7, .96, 4.4, .96, BLUE)
    arrow(ax, 4.4, .72, 2.7, .72, BLUE)
    ax.text(3.55, .29, 'B vibrates relative to S', ha='center', color=BLUE)
    ax.text(9.7, 3.08, 'Surface constrains S\nduring maintained contact', ha='center', color=GRAY)
    ax.text(6.4, .95, '64 MN/m reference stiffness', ha='center', color=ORANGE)
    ax.text(.2, 3.04, f'Reference frequency: {f/1000:.2f} kHz; one cycle takes {1000/f:.2f} ms',
            color=INK, fontweight='bold')
    ax.text(.2, -.05, 'Schematic fast motion, exaggerated for clarity. A 0.5 N cell-force change needs only 7.8 nm of flex.',
            color=GRAY, fontsize=10.5)
    ax = axes[1]
    ax.set(xlim=(0, 12), ylim=(0, 2.8))
    ax.axis('off')
    ax.set_title('How cell-mode feedback reduces ringing', loc='left', fontweight='bold')
    box(ax, .2, 1.18, 2.1, 1.05, 'Acquired axial\nload-cell signal')
    box(ax, 3.15, 1.18, 3.8, 1.05, 'Select the ringing band\nand compensate phase')
    box(ax, 7.8, 1.18, 2.35, 1.05, 'Add coil-current\ncorrection: ±0.05 A')
    arrow(ax, 2.3, 1.71, 3.15, 1.71)
    arrow(ax, 6.95, 1.71, 7.8, 1.71)
    arrow(ax, 9.0, 1.13, 9.0, .49)
    ax.text(9.0, .17, 'Coil force opposes the\noscillating motion at the selected mode', ha='center', color=BLUE)
    ax.text(.2, .45, 'Timing matters: a late correction can add energy.\nThe selected two-state filter combines band and phase shaping.', color=GRAY)
    fig.subplots_adjust(hspace=.38, top=.94, bottom=.08)
    output = save(fig, 'load-cell-mode-explained-20261006.svg')
    query = write_query('explained_cell_mode.json', [{
        'cellStiffness_N_m': stiffness, 'holderMass_kg': holder,
        'frequency_Hz': f, 'period_ms': 1000/f, 'cellFlexForHalfNewton_nm': .5/stiffness*1e9,
        'modalCurrentBound_A': modal['currentBound_A'],
        'modalForceBound_N': modal['currentBound_A']*24.4,
        'diagram': 'Schematic fast cell flexing under maintained contact; motion exaggerated',
        'input': 'Acquired axial cell deviation, not surface truth or preview',
        'scope': 'Explanation of the locked reference model, not a measured physical mode'
    }], [CONTRACT, Path(__file__), ROOT / 'acf_simulink/experiments/upper_no_damping_20261005/qualification_v8/acf_prepare_cell_modal.m'],
        kind='mechanical_mode_and_causal_controller_explanation',
        primaryReferences=[{'url':'https://www.mathworks.com/discovery/modal-analysis.html',
                            'claim':'Natural frequencies and corresponding patterns of structural motion define modes',
                            'readDate':'2026-10-06'}])
    return output, query


def qualification_figure():
    results = json.loads(QUAL.read_text())
    rows = results['rows']
    assert results['status'] == 'complete' and len(rows) == 700
    assert len({r['identity'] for r in rows}) == 700
    masses, forces = sorted({r['massS_kg'] for r in rows}), sorted({r['force_N'] for r in rows})
    assert len(masses) == len(forces) == 10
    matrix = np.zeros((10, 10))
    evidence = []
    for i, mass in enumerate(masses):
        for j, force in enumerate(forces):
            group = [r for r in rows if r['massS_kg'] == mass and r['force_N'] == force]
            assert len(group) == 7 and len({r['angle_rad'] for r in group}) == 7
            peak = max(r['peakNormalForceError_N'] for r in group)
            matrix[i,j] = peak
            evidence.append({'massS_kg':mass,'forceTarget_N':force,
                             'anglesTested':7,'peakAcrossAngles_N':peak,
                             'forcePasses':sum(bool(r['passed']) for r in group)})
    assert sum(r['passed'] for r in rows) == 683
    fig, ax = plt.subplots(figsize=(10.8, 6.5))
    im = ax.imshow(matrix, origin='lower', cmap='Blues', vmin=0, vmax=.65, aspect='auto')
    for i in range(10):
        for j in range(10):
            ax.text(j, i, f'{matrix[i,j]:.2f}', ha='center', va='center',
                    color='white' if matrix[i,j]>.33 else INK, fontsize=10.5)
            if matrix[i,j]>.5:
                ax.add_patch(Rectangle((j-.46,i-.46),.92,.92,fill=False,
                                       edgecolor=ORANGE,linewidth=2.2))
    ax.set(xticks=range(10), xticklabels=[f'{f:g}' for f in forces],
           yticks=range(10), yticklabels=[f'{m:.2f}' for m in masses],
           xlabel='Requested normal force (N)', ylabel='Tool mass S (kg)')
    ax.set_title('Where does the fixed design meet the force requirement?',
                 loc='left',fontweight='bold',pad=24)
    ax.text(0,1.02,'Each square: worst native error over seven fixed orientations. Outlined squares exceed 0.5 N.',
            transform=ax.transAxes,fontsize=10.5,color=GRAY)
    colorbar=fig.colorbar(im,ax=ax,pad=.025)
    colorbar.set_label('Peak normal-force error (N)')
    fig.subplots_adjust(bottom=.16,left=.11,right=.91,top=.86)
    fig.text(.11,.035,'700 nominal recordings: 683 pass. Sensing/motion stresses and long runs fail more often; see the cohort table.',color=GRAY,fontsize=10.5)
    output = save(fig,'fixed-force-operating-grid-20261006.svg')
    query = write_query('explained_nominal_grid.json',evidence,[QUAL,Path(__file__)],
                        kind='full_nominal_grid_aggregated_over_recorded_angles',
                        scope='Recorded finite nominal grid; no interpolation or physical qualification')
    return output,query


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    FIG.mkdir(parents=True,exist_ok=True)
    entries=[]
    for output,query in [mode_figure(),qualification_figure()]:
        entries.append({'figure':str(output.relative_to(ROOT)), 'figureSha256':sha(output),
                        'query':str(query.relative_to(ROOT)), 'querySha256':sha(query)})
    (OUT/'root_figures.json').write_text(json.dumps(entries,indent=2)+'\n')
    print(json.dumps(entries))


if __name__=='__main__':
    main()
