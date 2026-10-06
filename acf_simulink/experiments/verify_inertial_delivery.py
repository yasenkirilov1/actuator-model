"""Verify an extracted delivery before editing it; no MATLAB or simulation.

The manifest binds every delivered member. The native-grid checks independently
confirm exact case identities and complete startup-inclusive scoring counts.
This verifies the delivered evidence, not a physical sanding prototype.
"""
import argparse
import csv
import hashlib
import itertools
import json
from pathlib import Path


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(chunk)
    return value.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path,
                        default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    root = args.root.resolve()
    validation = root / 'acf_simulink' / 'validation'
    manifest = json.loads((validation / 'inertial_delivery_manifest_20260930.json').read_text())
    assert manifest['status'] == 'complete_reviewed_local_delivery'
    for entry in manifest['files']:
        file = (root / entry['path']).resolve()
        assert root in file.parents and file.is_file(), entry['path']
        assert file.stat().st_size == entry['bytes'], entry['path']
        assert digest(file) == entry['sha256'], entry['path']
    summary = json.loads((validation / 'normal_mass_controller_final_grid_20260930_summary.json').read_text())
    assert summary['coverage']['complete'] and summary['coverage']['auditPassed']
    with (validation / 'normal_mass_controller_final_grid_20260930_rows.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    identities = [(float(r['massS_kg']), float(r['reference_N']),
                   round(float(r['angle_deg']), 6), float(r['mu']),
                   int(float(r['seedRequested']))) for r in rows]
    expected = set(itertools.product([.1, .5, 1, 1.4, 2], range(5, 51, 5),
                                    range(0, 91, 15), [0, .3], [0, 7, 29, 97, 1729]))
    assert len(rows) == len(set(identities)) == 3500
    assert set(identities) == expected
    assert {int(float(r['globalCaseId'])) for r in rows} == set(range(1, 3501))
    assert all(r['passed'].lower() in {'true', '1', '1.0'} for r in rows)
    assert all(int(float(r['nativeSamples'])) == 400001 for r in rows)
    peak = max(float(r['peakNormalForceError_N']) for r in rows)
    assert abs(peak - summary['worstCase']['peakNormalForceError_N']) < 1e-12
    print(json.dumps({'status': 'verified', 'files': len(manifest['files']),
                      'uniqueNominalCases': len(rows), 'allCasesPass': True,
                      'nativeSamplesScored': 3500 * 400001,
                      'worstNominalError_N': peak,
                      'scope': 'Delivered bytes and finite native evidence, not hardware qualification.'}))


if __name__ == '__main__':
    main()
