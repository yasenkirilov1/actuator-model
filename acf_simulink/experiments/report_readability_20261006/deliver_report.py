"""Deliver the readable Markdown/HTML without replacing historical reports.

Guard the user's current Notes files against the prior snapshot (or a previous
delivery of this revision), then bind copied bytes and preserved evidence.
"""
from pathlib import Path
import hashlib
import json
import re
import shutil

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / 'reports/prototype-design-20260930'
VAL = ROOT / 'acf_simulink/validation/report_readability_20261006/report'
NOTES = Path('/home/tommie/Documents/Notes Remote/S7 Project')
NAME = 'active-contact-flange-report-design-explained-20261006'


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for chunk in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    prior = json.loads((VAL / 'previous_authoring_manifest.json').read_text())
    manifest_path = VAL / 'delivery_manifest.json'
    previous_delivery = json.loads(manifest_path.read_text()) if manifest_path.exists() else None
    for suffix in ['md', 'html']:
        target = NOTES / f'active-contact-flange-report.{suffix}'
        if previous_delivery:
            expected = next(x['sha256'] for x in previous_delivery['deliveredFiles'] if x['path'] == str(target))
        else:
            expected = next(x['sha256'] for x in prior['files'] if x['source'] == str(target))
        assert sha(target) == expected, f'Independent Notes changes detected: {target}'

    original_data = json.loads(Path(next(x['snapshot'] for x in prior['files'] if x['source'] == str(APP / 'src/data.json'))).read_text())
    current_data = json.loads((APP / 'src/data.json').read_text())
    assert current_data['id'] == original_data['id']
    assert current_data['authoredRevision'] == original_data['authoredRevision']
    assert current_data['buildStatus'] == 'complete'
    assert all(current_data['queries'][k] == v for k, v in original_data['queries'].items())
    css_entry = next(x for x in prior['files'] if x['source'] == str(APP / 'src/content/report/report.css'))
    assert sha(css_entry['source']) == css_entry['sha256']
    old_report = next(x for x in prior['files'] if x['source'] == str(ROOT / 'reports/active-contact-flange-report-upper-spring-20261005.md'))
    assert sha(old_report['source']) == old_report['sha256']

    bundle = ROOT / 'acf_simulink/generated/fixed_upper_20261005'
    contract = json.loads((bundle / 'bundle_manifest.json').read_text())
    unchanged = [{'path': str(bundle / f['file']), 'sha256': sha(bundle / f['file']),
                  'unchanged': sha(bundle / f['file']) == f['sha256']} for f in contract['files']]
    assert all(f['unchanged'] for f in unchanged)

    build_path = APP / 'dist/data-app-build.json'
    build = json.loads(build_path.read_text())
    assert build['runtimeSha256'] == '9e3ede84b28aded3c7379b6e6a5611f0d95b9977eb0f781e279ceefabfcbd27e'
    authoring = json.loads((VAL / 'html_authoring_manifest.json').read_text())
    assert authoring['status'] == 'complete' and authoring['preservedQueries'] == 79
    markdown = ROOT / 'reports' / f'{NAME}.md'
    text = markdown.read_text()
    assert sha(markdown) == authoring['markdownSha256']
    assert 'Open problem: sanding drag' in text and '**one-axis B–S load sensing**' in text
    assert 'being checked separately' not in text
    assert all(Path(link).exists() for link in re.findall(r'\]\((/home/[^)]+)\)', text))
    figure_names = re.findall(r'!\[[^\]]*\]\((figures/[^)]+)\)', text)
    assert len(figure_names) == len(authoring['figures']) == 6
    offline = APP / '.data-app-offline/exports' / f'{NAME}.html'
    assert offline.exists()
    repo_html = ROOT / 'reports' / f'{NAME}.html'

    copies = [(offline, repo_html), (markdown, NOTES / 'active-contact-flange-report.md'),
              (offline, NOTES / 'active-contact-flange-report.html')]
    for name in figure_names:
        source = markdown.parent / name
        target = NOTES / name
        if target.exists():
            assert sha(target) == sha(source), f'Independent figure changes detected: {target}'
        copies.append((source, target))
    for source, target in copies:
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        assert sha(source) == sha(target)

    result = {
        'status': 'delivered_and_byte_verified',
        'reportId': current_data['id'], 'authoredRevision': current_data['authoredRevision'],
        'wordsExcludingLinkTargets': len(re.sub(r'\]\([^)]*\)', ']', text).split()),
        'preservedQueries': len(original_data['queries']), 'totalQueries': len(current_data['queries']),
        'newSimulationRecordings': 0, 'fullRequirementQualified': False,
        'sourceMarkdown': str(markdown), 'sourceMarkdownSha256': sha(markdown),
        'sourceHtml': str(offline), 'sourceHtmlSha256': sha(offline),
        'reviewedInputs': str(VAL / 'report_inputs_design_explained.json'),
        'reviewedInputsSha256': sha(VAL / 'report_inputs_design_explained.json'),
        'authoringManifestSha256': sha(VAL / 'html_authoring_manifest.json'),
        'buildManifest': str(build_path), 'buildManifestSha256': sha(build_path),
        'runtimeSha256': build['runtimeSha256'],
        'deliveredFiles': [{'path': str(target), 'sha256': sha(target), 'bytes': target.stat().st_size} for _, target in copies],
        'frozenModelChecks': unchanged,
        'priorAuthoringSnapshot': str(VAL / 'previous_authoring_manifest.json'),
        'browserCheckFile': str(VAL / 'browser_checks.json')}
    manifest_path.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ['status', 'wordsExcludingLinkTargets', 'preservedQueries', 'totalQueries', 'newSimulationRecordings']}))


if __name__ == '__main__':
    main()
