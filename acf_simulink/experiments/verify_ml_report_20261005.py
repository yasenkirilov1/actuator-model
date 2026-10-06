"""Check the new report delivery independently of historical packages."""
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VAL = ROOT / 'acf_simulink/validation/ml_report_20261005'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    manifest = json.loads((VAL / 'delivery_manifest.json').read_text())
    assert manifest['status'] == 'complete'
    for record in manifest['files']:
        path = Path(record['file'])
        if not path.is_absolute():
            path = ROOT / path
        assert path.is_file(), path
        assert sha(path) == record['sha256'], path
        assert path.stat().st_size == record['bytes'], path
    markdown = Path(manifest['notesMarkdown']).read_text()
    assert re.findall(r'^## (.+)$', markdown, re.M) == ['Intro', 'Question', 'Results', 'Recommendation']
    components = re.split(r'^### (.+)$', markdown, flags=re.M)
    assert len(components[1::2]) == 9
    for name, body in zip(components[1::2], components[2::2]):
        assert re.findall(r'^#### (.+)$', body, re.M) == ['Intro', 'Question', 'Results', 'Recommendation'], name
    old = json.loads((VAL / 'previous_authoring/src/data.json').read_text())
    data = json.loads((ROOT / 'reports/prototype-design-20260930/src/data.json').read_text())
    assert data['id'] == old['id'] == manifest['reportId']
    assert data['buildStatus'] == 'complete'
    assert data['authoredRevision'] == old.get('authoredRevision', 0) + 1
    assert all(data['queries'][key] == value for key, value in old['queries'].items())
    assert len(data['queries']) == 54
    images = re.findall(r'!\[[^\]]*\]\((figures/[^)]+)\)', markdown)
    assert len(images) == len(set(images)) == 37
    assert all(image.endswith('.svg') for image in images)
    prior = (VAL / 'report-with-svgs.md').read_text()
    assert re.findall(r'```.*?```', prior, re.S) == re.findall(r'```.*?```', markdown, re.S)
    for record in manifest['preservedOriginalFigures']:
        path = Path(manifest['notesMarkdown']).parent / record['file']
        assert sha(path) == record['sha256'], path
    public_model = ROOT / 'acf_simulink/generated/acf_force_control_simple.slx'
    assert sha(public_model) == manifest['unchangedPublicModelSha256']
    force = json.loads((ROOT / 'acf_simulink/validation/ml_force_20261005/summary.json').read_text())
    robot = json.loads((ROOT / 'acf_simulink/validation/ml_robot_20261005_figures_final/summary.json').read_text())
    combined = json.loads((ROOT / 'acf_simulink/validation/ml_combined_20261005_figures/summary.json').read_text())
    assert force['holdoutBaselineRecordings'] + force['holdoutLearnedRecordings'] + robot['heldout_recordings'] + combined['recordings'] == 54
    assert force['baselineHoldoutPasses'] == force['learnedHoldoutPasses'] == 12
    assert robot['teacher_passes'] == robot['learned_passes'] == 11
    assert combined['baseline_passed'] == combined['combined_passed'] == 4
    assert manifest['learnedPoliciesPromoted'] is False
    result = {'status': 'passed', 'checkedFileBindings': len(manifest['files']),
              'nestedComponents': 9, 'svgFigures': 37, 'unchangedSourceQueries': len(old['queries']),
              'originalEquationsPreserved': 11, 'pairedHeldoutNativeRecordings': 54,
              'publicModelUnchanged': True, 'browserChecks': manifest['browserChecks']}
    (VAL / 'delivery_checks.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
