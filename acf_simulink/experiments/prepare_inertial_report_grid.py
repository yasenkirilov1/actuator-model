"""Bind the report's searchable case rows to the completed native-grid audit."""
import csv
import hashlib
import json
import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VALIDATION = ROOT / 'acf_simulink' / 'validation'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def value(text):
    if text == '':
        return None
    if text in ('True', 'False'):
        return text == 'True'
    if re.fullmatch(r'-?\d+', text):
        return int(text)
    try:
        result = float(text)
        return result if math.isfinite(result) else None
    except ValueError:
        return text


def main():
    summary_path = VALIDATION / 'normal_mass_controller_final_grid_20260930_summary.json'
    csv_path = VALIDATION / 'normal_mass_controller_final_grid_20260930_rows.csv'
    summary = json.loads(summary_path.read_text())
    coverage = summary['coverage']
    assert coverage['complete'] and coverage['auditPassed'], 'Wait for all four workers.'
    assert coverage['recorded'] == coverage['uniqueIdentities'] == 3500
    audit_path = VALIDATION / 'normal_mass_controller_final_grid_20260930_coverage.json'
    audit = json.loads(audit_path.read_text())
    assert audit['complete'] and audit['auditPassed'] and not audit['auditErrors']
    assert audit['modelSHA256'] == '73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643'
    assert audit['candidateSHA256'] == 'd8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7'
    with csv_path.open(newline='') as stream:
        rows = [{key: value(text) for key, text in row.items()}
                for row in csv.DictReader(stream)]
    assert len(rows) == 3500
    for row in rows:
        # CSV id is local to a shard; a report row needs a unique identity.
        row['partitionCaseId'] = row['id']
        row['id'] = row['globalCaseId']
        row['rawAngle_deg'] = row['angle_deg']
        row['angle_deg'] = round(row['angle_deg'], 6)
        assert abs(row['angle_deg'] - row['rawAngle_deg']) < 1e-10
    identities = {(r['massS_kg'], r['reference_N'], r['angle_deg'], r['mu'],
                   r['seedRequested']) for r in rows}
    expected = {(s, f, a, mu, seed)
                for s in [.1, .5, 1, 1.4, 2] for f in range(5, 51, 5)
                for a in range(0, 91, 15) for mu in [0, .3]
                for seed in [0, 7, 29, 97, 1729]}
    assert identities == expected and len(identities) == len(rows)
    assert {r['globalCaseId'] for r in rows} == set(range(1, 3501))
    assert all(r['nativeReturned'] for r in rows)
    result = {'status': 'complete_audited_report_rows',
              'scope': 'Finite nominal native grid, with exact recorded metrics. '
                       'Neither continuous-range nor combined-tolerance qualification.',
              'aggregateFile': str(summary_path.relative_to(ROOT)),
              'aggregateSha256': digest(summary_path),
              'coverageSha256': digest(audit_path),
              'csvFile': str(csv_path.relative_to(ROOT)),
              'csvSha256': digest(csv_path),
              'modelSha256': '73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643',
              'candidateSha256': 'd8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7',
              'displayIdentity': 'id equals globalCaseId; original CSV id is '
                                 'partitionCaseId. Angle uses six-decimal identity '
                                 'rounding, with rawAngle_deg preserved. Metrics are unchanged.',
              'nativeSamplesScored': sum(r['nativeSamples'] for r in rows),
              'rows': rows}
    destination = VALIDATION / 'inertial_report_final_grid_rows_20260930.json'
    destination.write_text(json.dumps(result, allow_nan=False, separators=(',', ':')) + '\n')
    print(json.dumps({'status': result['status'], 'rows': len(rows),
                      'nativeSamples': result['nativeSamplesScored'],
                      'file': str(destination.relative_to(ROOT))}))


if __name__ == '__main__':
    main()
