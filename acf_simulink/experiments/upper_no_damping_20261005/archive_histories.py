"""Relocate this study's owned lossless histories, verifying every byte first.

Original project paths remain usable through symlinks. No retained result,
input seal, unrelated file or source computation is changed.
"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import time

ROOT = Path(__file__).resolve().parents[3]
ARCHIVE = Path('/run/media/tommie/SATA SSD/actuator-model-evidence/20261005-mechanical-studies')
STUDIES = ('upper_no_damping_20261005', 'upper_force_observer_20261005')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(4 * 1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def main():
    assert Path('/run/media/tommie/SATA SSD').is_mount(), 'Archive drive is not mounted'
    ARCHIVE.mkdir(parents=True, exist_ok=True)
    manifest = ARCHIVE / 'relocation_manifest.json'
    rows = json.loads(manifest.read_text())['files'] if manifest.exists() else []
    existing = {r['projectPath']: r for r in rows}
    total = 0
    for study in STUDIES:
        evaluation = ROOT / 'acf_simulink/validation' / study / 'evaluation'
        for source in sorted(evaluation.rglob('*.npz')):
            relative = source.relative_to(ROOT)
            if source.is_symlink():
                assert str(relative) in existing
                continue
            target = ARCHIVE / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            size = source.stat().st_size
            source_hash = digest(source)
            if not target.exists():
                partial = target.with_name(target.name + '.copying')
                shutil.copy2(source, partial)
                with partial.open('rb') as f:
                    os.fsync(f.fileno())
                assert digest(partial) == source_hash
                os.replace(partial, target)
            assert target.stat().st_size == size and digest(target) == source_hash
            assert digest(source) == source_hash, 'Source changed while copying'
            entry = dict(projectPath=str(relative), archivePath=str(target),
                         bytes=size, sha256=source_hash, verifiedUtc=time.time())
            existing[str(relative)] = entry
            rows = sorted(existing.values(), key=lambda r: r['projectPath'])
            temporary_manifest = manifest.with_suffix('.partial.json')
            temporary_manifest.write_text(json.dumps(dict(
                contract='Verified identical bytes before atomic replacement of owned NPZ files with project-path symlinks.',
                files=rows), indent=2) + '\n')
            os.replace(temporary_manifest, manifest)
            link = source.with_name(source.name + '.archive_link')
            link.symlink_to(target)
            os.replace(link, source)
            total += size
            print(f'{study}: archived {source.name}, cumulative {total / 1024**3:.3f} GiB', flush=True)
    print(f'Complete: {len(rows)} verified histories; {total / 1024**3:.3f} GiB moved this run', flush=True)


if __name__ == '__main__':
    main()
