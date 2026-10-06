"""Deliver completed concise MD/HTML/figures while preserving earlier reports."""
from pathlib import Path
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
import re
import shutil

ROOT = Path(__file__).resolve().parents[3]
REPORT = ROOT / 'acf_simulink/validation/upper_no_damping_20261005/report'
NOTES = Path('/home/tommie/Documents/Notes Remote/S7 Project')
MD = ROOT / 'reports/active-contact-flange-report-upper-spring-20261005.md'
HTML = MD.with_suffix('.html')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--html', type=Path, required=True)
    parser.add_argument('--inputs', type=Path, required=True)
    parser.add_argument('--manifest', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads((ROOT / 'reports/prototype-design-20260930/src/data.json').read_text())
    inputs = json.loads(args.inputs.read_text())
    assert data['buildStatus'] == inputs['status'] == 'complete'
    assert data['id'] == 'report:578d0259-115c-40f2-9264-4c57b68ab138'
    assert len(inputs['figures']) == 3
    text = MD.read_text()
    assert not any(x in text for x in ('Pending', 'still running', 'comparison is running'))
    words = len(re.findall(r'\b[\wµ±]+\b', text))
    assert 1700 <= words <= 2300
    previous = json.loads((REPORT / 'notes_delivery_draft.json').read_text())
    known = {Path(f['target']): {f['sha256']} for f in previous['files']}
    for target, snapshot in ((NOTES / 'active-contact-flange-report.md', '00_active-contact-flange-report-rl-3h-20261005.md'),
                             (NOTES / 'active-contact-flange-report.html', '01_active-contact-flange-report-rl-3h-20261005.html')):
        path = REPORT / 'previous_delivery' / snapshot
        if path.exists():
            known.setdefault(target, set()).add(sha(path))
    if args.manifest.exists():
        last = json.loads(args.manifest.read_text())
        for f in last['files']:
            known.setdefault(Path(f['target']), set()).add(f['sha256'])
    pairs = [(MD, NOTES / 'active-contact-flange-report.md'),
             (args.html, HTML), (args.html, NOTES / 'active-contact-flange-report.html')]
    pairs += [(ROOT / 'reports' / f['file'], NOTES / f['file']) for f in inputs['figures']]
    # Check every destination before replacing any file.
    for source, target in pairs:
        assert source.is_file()
        if target.exists() and target != HTML:
            assert sha(target) in known.get(target, set()) | {sha(source)}, \
                f'Preserve independently edited destination: {target}'
    delivered = []
    for source, target in pairs:
        target.parent.mkdir(parents=True, exist_ok=True)
        temp = target.with_name(target.name + '.acf-delivery.tmp')
        assert not temp.exists()
        shutil.copyfile(source, temp)
        assert sha(temp) == sha(source)
        with temp.open('rb') as f:
            os.fsync(f.fileno())
        os.replace(temp, target)
        delivered.append({'source': str(source.resolve()), 'target': str(target),
                          'sha256': sha(target), 'bytes': target.stat().st_size})
    output = {'status': 'complete', 'createdUtc': datetime.now(timezone.utc).isoformat(),
              'reportId': data['id'], 'authoredRevision': data.get('authoredRevision'),
              'mainReportWordCount': words, 'figures': 3,
              'evidenceInputs': {'path': str(args.inputs), 'sha256': sha(args.inputs)},
              'buildManifest': {'path': str(ROOT / 'reports/prototype-design-20260930/dist/data-app-build.json'),
                                'sha256': sha(ROOT / 'reports/prototype-design-20260930/dist/data-app-build.json')},
              'files': delivered,
              'scope': 'Concise main report and complete offline HTML. Older reference reports/packages and pre-edit snapshots preserved. Results describe finite simulation evidence, not prototype validation.'}
    args.manifest.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps({'status': output['status'], 'wordCount': words, 'files': len(delivered)}))


if __name__ == '__main__':
    main()
