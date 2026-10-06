"""Package the reviewed model, report, evidence and reproducible frozen inputs.

An explicit plan selects files. It does not glob native trace folders or replace
the historical delivery archive. Run only after the coverage, native acceptance
and standalone-report checks in the plan have completed.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import zipfile

ROOT = Path(__file__).resolve().parents[2]


def sha256(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(chunk)
    return result.hexdigest()


def load_check(path):
    return json.loads((ROOT / path).read_text())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plan', type=Path, required=True)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text())
    assert plan['status'] == 'reviewed_for_delivery', 'Do not package a draft.'
    coverage = load_check(plan['gridSummary'])['coverage']
    assert coverage['complete'] and coverage['auditPassed']
    assert coverage['recorded'] == coverage['uniqueIdentities'] == 3500
    report = load_check(plan['reportManifest'])
    assert report['status'] == 'complete'
    browser = load_check(plan['browserChecks'])
    assert browser['status'] == 'complete' and all(browser['checks'].values())
    historical = ROOT / plan['historicalArchive']
    historical_hash = sha256(historical)
    assert historical_hash == plan['historicalArchiveSha256']
    destination = ROOT / plan['archive']
    assert not destination.exists(), 'Keep an existing delivery immutable.'
    destination.parent.mkdir(parents=True, exist_ok=True)
    paths, entries = [], []
    used = set()
    for item in plan['files']:
        path = Path(item['source'])
        if not path.is_absolute():
            path = ROOT / path
        archive_name = PurePosixPath(item['archivePath'])
        assert not archive_name.is_absolute() and '..' not in archive_name.parts
        assert str(archive_name) not in used, str(archive_name)
        assert path.is_file(), path
        used.add(str(archive_name))
        digest = sha256(path)
        if item.get('expectedSha256'):
            assert digest == item['expectedSha256'], path
        entry = {'path': str(archive_name), 'bytes': path.stat().st_size,
                 'sha256': digest, 'role': item['role'],
                 'originalSource': str(path)}
        entries.append(entry)
        paths.append((path, entry))
    manifest_name = 'acf_simulink/validation/inertial_delivery_manifest_20260930.json'
    assert manifest_name not in used
    manifest = {'status': 'complete_reviewed_local_delivery',
                'date': plan['date'], 'archive': plan['archive'],
                'scope': plan['scope'], 'files': entries,
                'checks': plan['completedChecks'],
                'planSha256': sha256(args.plan),
                'nativeTracePolicy': plan['nativeTracePolicy'],
                'historicalArchivePreserved': plan['historicalArchive']}
    temporary = destination.with_suffix(destination.suffix + '.partial')
    assert not temporary.exists(), temporary
    try:
        with zipfile.ZipFile(temporary, 'w', compression=zipfile.ZIP_DEFLATED,
                             compresslevel=6, allowZip64=True) as archive:
            for path, entry in paths:
                # Native MAT, SLX, figures and PDF are already compressed.
                # Store their bytes directly; compress the text evidence.
                compressed = path.suffix.lower() in {'.mat', '.slx', '.png', '.pdf', '.zip'}
                archive.write(path, entry['path'], compress_type=(
                    zipfile.ZIP_STORED if compressed else zipfile.ZIP_DEFLATED))
            archive.writestr(manifest_name, json.dumps(manifest, indent=2) + '\n')
        with zipfile.ZipFile(temporary) as archive:
            assert archive.testzip() is None
            assert len(archive.namelist()) == len(entries) + 1
            for entry in entries:
                result = hashlib.sha256()
                with archive.open(entry['path']) as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                        result.update(chunk)
                assert result.hexdigest() == entry['sha256'], entry['path']
        # Validate originals again so a parallel edit cannot go unnoticed.
        for path, entry in paths:
            assert sha256(path) == entry['sha256'], path
        assert sha256(historical) == historical_hash
        os.replace(temporary, destination)
    except BaseException:
        if temporary.exists():
            temporary.unlink()
        raise
    manifest_path = ROOT / manifest_name
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    checks = {'status': 'complete', 'archive': plan['archive'],
              'archiveBytes': destination.stat().st_size,
              'archiveSha256': sha256(destination),
              'includedFiles': len(entries) + 1,
              'uncompressedFileBytes': sum(x['bytes'] for x in entries),
              'historicalArchiveSha256': historical_hash,
              'checks': {'zipCRC': True, 'everyMemberSha256': True,
                         'sourceFilesUnchangedDuringPackaging': True,
                         'explicitFilePlan': True, 'finiteGridAudit': True,
                         'completeReportAndBrowserChecks': True,
                         'historicalArchivePreserved':
                         sha256(historical) == historical_hash}}
    assert checks['checks']['historicalArchivePreserved']
    (ROOT / 'acf_simulink/validation/inertial_delivery_package_checks_20260930.json').write_text(
        json.dumps(checks, indent=2) + '\n')
    print(json.dumps({k: checks[k] for k in ['status', 'archive', 'archiveBytes',
                                            'includedFiles', 'archiveSha256']}))


if __name__ == '__main__':
    main()
