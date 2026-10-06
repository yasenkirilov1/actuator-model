"""Summarize the locked candidate's completed new-seed native tests.

Preparation and replay guards are not scored cases. Each cohort must have
finished against the same frozen bank and candidate selection contract.
"""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def key(path):
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--selection-lock', type=Path, required=True)
    parser.add_argument('--nominal', type=Path, required=True)
    parser.add_argument('--fresh', type=Path, required=True)
    parser.add_argument('--long', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    selected = json.loads(args.selection_lock.read_text())
    assert selected['selectionOnly'] and not selected['qualificationOutcomesInspected']
    assert selected['noRetuningOnQualification']
    sources = {key(args.selection_lock): sha(args.selection_lock)}
    rows, all_rows, build = [], [], None
    for label, path, start, count, seed in (
        ('nominal700', args.nominal, 0, 700, 730000),
        ('fresh32', args.fresh, 700, 32, 740000),
        ('long4', args.long, 732, 4, 750000)):
        result = json.loads(path.read_text())
        lock_path = path.with_name('input_lock.json')
        lock = json.loads(lock_path.read_text())
        assert result['status'] == 'complete' and result['lockedInputsUnchanged']
        assert result['cases'] == count == len(result['rows'])
        assert result['inputLockSha256'] == sha(lock_path)
        assert result['split'] == lock['split'] == 'qualification_' + label
        assert sorted(r['scenario'] for r in result['rows']) == list(range(start, start + count))
        for r in result['rows']:
            assert r['seed'] == seed + r['scenario'] - start
            assert r['upperK_N_m'] == 20000 and r['upperC_N_s_m'] == 0
            assert r['actualK_N_m'] == 64e6 and r['actualC_N_s_m'] == 0
            assert r['massA_kg'] == 1.5 and r['massB_kg'] == .6
            assert r['upperExternalSpringPresent'] and r['upperLengthUnconstrained']
            assert not r.get('upperRigid', False) and not r['lowerExternalSpringPresent']
            assert not r['softwareRejected']
            assert r['nativeGridComplete'] or r['physicalFailure']
        if build is None:
            build = lock['nativeBuild']
        assert lock['nativeBuild'] == build
        assert selected['createdUtc'] < lock['createdUtc']
        worst = max(result['rows'], key=lambda r: r['peakNormalForceError_N'])
        rows.append({'cohort': label, 'recordings': count,
                     'passed': result['passed'], 'hardFailures': result['physicalFailures'],
                     'designMarginPassed': sum(r['designMarginPassed'] for r in result['rows']),
                     'travelBandExcursions': sum(not r['travelBandPassed'] for r in result['rows']),
                     'nativeSamples': result['nativeSamples'],
                     'worstNormalForceError_N': worst['peakNormalForceError_N'],
                     'worstIdentity': worst['identity'], 'worstScenario': worst['scenario'],
                     'largestVoltage_V': max(r['peakVoltage_V'] for r in result['rows']),
                     'voltageClipSamples': sum(r['voltageClipSamples'] for r in result['rows']),
                     'largestCurrent_A': max(r['peakCurrent_A'] for r in result['rows']),
                     'source': key(path), 'sourceSha256': sha(path),
                     'retentionReplaysExcluded': True})
        for source in (path, lock_path):
            sources[key(source)] = sha(source)
        all_rows += result['rows']
    assert len({r['seed'] for r in all_rows}) == 736
    assert len({(r['massS_kg'], r['force_N'], r['angle_rad']) for r in all_rows[:700]}) == 700
    assert np.allclose(sorted({r['angle_rad'] for r in all_rows[:700]}),
                       np.deg2rad(range(0, 91, 15)), atol=1e-15)
    sources[key(Path(build) / 'metadata.json')] = sha(Path(build) / 'metadata.json')
    total = {'recordings': 736, 'passed': sum(r['passed'] for r in all_rows),
             'hardFailures': sum(r['physicalFailure'] for r in all_rows),
             'nativeSamples': sum(r['nativeSamples'] for r in all_rows),
             'designMarginPassed': sum(r['designMarginPassed'] for r in all_rows),
             'travelBandExcursions': sum(not r['travelBandPassed'] for r in all_rows),
             'worstNormalForceError_N': max(r['peakNormalForceError_N'] for r in all_rows)}
    out = {'status': 'complete', 'kind': 'locked_new_seed_fixed_mechanics_native_qualification',
           'rows': rows, 'totals': total, 'sourceHashes': sources,
           'selectedIdentity': selected['selectedIdentity'],
           'scope': 'One candidate locked before these new-seed outcomes. Full native force/physical histories scored; finite tested grid and selected residual/schedule cases, not installed hardware qualification. No retuning on these results.'}
    args.output.write_text(json.dumps(out, indent=2) + '\n')
    print(json.dumps(total))


if __name__ == '__main__':
    main()
