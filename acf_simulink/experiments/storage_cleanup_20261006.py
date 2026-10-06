"""One-off audited cleanup of this project's generated recordings.

Run without arguments to write the reviewable plan. --execute applies that
exact plan after checking scope, unchanged files, tracked files and open FDs.
"""
import argparse
import collections
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
from datetime import datetime, timezone

PROJECT = Path('/home/tommie/Projects/actuator-model')
VALIDATION = PROJECT / 'acf_simulink/validation'
EVIDENCE = Path('/run/media/tommie/SATA SSD/actuator-model-evidence')
ROOTS = (VALIDATION, EVIDENCE)
AUDIT = VALIDATION / 'storage_cleanup_20261006'
CURRENT = VALIDATION / 'upper_no_damping_20261005/qualification_v8'
RUNS = EVIDENCE / '20261005-mechanical-studies/new-runs'
ADC = EVIDENCE / '20261006-report-readability/adc-private/executed'
KEEP = {
    RUNS / 'cell_modal_v6_two_endpoints/case_0370/retained_native_history.npz',
    VALIDATION / 'upper_force_observer_20261005/evaluation/observer_noise_promising_endpoints_v4/case_0722/native_history.npz',
    RUNS / 'controller_gain_v9_A15_dev11/case_0170/retained_native_history.npz',
    RUNS / 'fixed_qualification_nominal700/case_0673/native_history.npz',
    RUNS / 'fixed_qualification_fresh32/case_0728/native_history.npz',
    RUNS / 'fixed_qualification_long4/case_0733/native_history.npz',
    ADC / 'adc_sinc3_30000_q24_source170/native_history.npz',
    ADC / 'adc_sinc3_30000_q24_source171/native_history.npz',
    ADC / 'adc_sinc3_30000_q14_source170/force_residual_history.npz',
    ADC / 'adc_sinc3_15000_q24_source170/force_residual_history.npz',
    ADC / 'adc_sinc3_30000_q12_source171/force_residual_history.npz',
}
# A selected project path may be an archive alias. Protect its actual bytes too.
KEEP |= {p.resolve(strict=False) for p in tuple(KEEP)}
INPUT_NAME = re.compile(r'(config|parameter|coeff|bank|(?:^|_)inputs?(?:_|\.)|profile)', re.I)


def files(root):
    for parent, dirs, names in os.walk(root, followlinks=False):
        dirs[:] = [n for n in dirs if not Path(parent, n).is_symlink()]
        for name in names:
            p = Path(parent, name)
            if stat.S_ISREG(p.lstat().st_mode):
                yield p


def identity(p):
    s = p.lstat()
    return {'path': str(p), 'bytes': s.st_size, 'allocatedBytes': s.st_blocks * 512,
            'inode': s.st_ino, 'device': s.st_dev, 'mtimeNs': s.st_mtime_ns}


def keep_reason(p):
    if p.is_relative_to(CURRENT):
        return 'current frozen inputs, independent replays and acceptance fixtures'
    if p in KEEP:
        return 'selected representative recording'
    if p.is_relative_to(ADC) and p.parent.name.startswith('adc_sinc3_7500_'):
        return 'short representative startup failures'
    if INPUT_NAME.search(p.name) and 'trace' not in p.name.lower():
        return 'parameter/configuration input'
    if any(n in {'selected_controller', 'frozen_inputs', 'source_inputs', 'portable_inputs', 'independent_inputs'} for n in p.parts):
        return 'frozen controller or reproduction input'
    return None


def compact_fingerprint():
    h = hashlib.sha256()
    count = size = 0
    for root in ROOTS:
        for p in sorted(files(root)):
            if p.is_relative_to(AUDIT) or p.suffix in {'.mat', '.npz'}:
                continue
            item = identity(p)
            h.update(json.dumps(item, sort_keys=True).encode())
            count += 1
            size += item['bytes']
    return {'files': count, 'bytes': size, 'statFingerprint': h.hexdigest()}


def write(name, data):
    AUDIT.mkdir(exist_ok=True)
    (AUDIT / name).write_text(json.dumps(data, indent=2) + '\n')


def plan():
    missing = [str(p) for p in KEEP if not p.is_file()]
    if missing:
        raise SystemExit(f'Essential recording missing before cleanup: {missing}')
    delete, preserved = [], []
    for root in ROOTS:
        for p in files(root):
            if p.suffix not in {'.mat', '.npz'}:
                continue
            item = identity(p)
            reason = keep_reason(p)
            generated_history = p.name in {'native_history.npz', 'retained_native_history.npz', 'force_residual_history.npz', 'actions.npz'} or p.name.startswith('rollout_')
            if reason or (item['bytes'] <= 1024**2 and not generated_history):
                preserved.append({**item, 'reason': reason or 'small input or compact array'})
            else:
                delete.append(item)
    data = {'status': 'planned', 'utc': datetime.now(timezone.utc).isoformat(),
            'scope': [str(r) for r in ROOTS], 'delete': delete, 'preservedArrays': preserved,
            'plannedBytesRemoved': sum(x['bytes'] for x in delete),
            'preservedArrayBytes': sum(x['bytes'] for x in preserved),
            'compactBefore': compact_fingerprint()}
    write('bulk_plan.json', data)
    groups = collections.Counter()
    for x in delete:
        p = Path(x['path'])
        root = next(r for r in ROOTS if p.is_relative_to(r))
        groups[str(root)] += x['bytes']
    print(json.dumps({'deleteFiles': len(delete), 'plannedGiB': data['plannedBytesRemoved']/1024**3,
                      'retainedArrayGiB': data['preservedArrayBytes']/1024**3,
                      'byRootGiB': {k:v/1024**3 for k,v in groups.items()}}, indent=2))


def execute():
    data = json.loads((AUDIT / 'bulk_plan.json').read_text())
    tracked = {PROJECT / p for p in subprocess.check_output(['git', 'ls-files', '-z'], cwd=PROJECT).decode().split('\0') if p}
    open_files = set()
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            comm = (proc / 'comm').read_text().strip().lower()
            if comm.startswith('matlab'):
                raise SystemExit(f'MATLAB process {proc.name} still running; cleanup stopped')
            for fd in (proc / 'fd').iterdir():
                try:
                    open_files.add(os.readlink(fd))
                except (FileNotFoundError, PermissionError, OSError):
                    pass
        except (FileNotFoundError, PermissionError, OSError):
            pass
    for item in data['delete']:
        p = Path(item['path'])
        if not any(p.is_relative_to(r) for r in ROOTS) or p in tracked or p.is_symlink() or keep_reason(p):
            raise SystemExit(f'Unsafe cleanup target: {p}')
        if any(parent.is_symlink() for parent in p.parents) or identity(p) != item or str(p) in open_files:
            raise SystemExit(f'Changed, linked or open cleanup target: {p}')
    if compact_fingerprint() != data['compactBefore']:
        raise SystemExit('Compact evidence changed after planning; replan')
    before = {str(r): os.statvfs(r).f_bavail * os.statvfs(r).f_frsize for r in ROOTS}
    removed = []
    try:
        for item in data['delete']:
            p = Path(item['path'])
            if identity(p) != item:
                raise RuntimeError(f'File changed during cleanup: {p}')
            p.unlink()
            removed.append(item)
    finally:
        write('bulk_cleanup.json', {'status': 'removed' if len(removed) == len(data['delete']) else 'partial',
                                   'utc': datetime.now(timezone.utc).isoformat(), 'removed': removed,
                                   'logicalBytesRemoved': sum(x['bytes'] for x in removed),
                                   'freeBeforeBytes': before,
                                   'freeAfterBytes': {str(r): os.statvfs(r).f_bavail * os.statvfs(r).f_frsize for r in ROOTS}})
    removed_paths = {Path(x['path']) for x in removed}
    links = []
    for parent, dirs, names in os.walk(VALIDATION, followlinks=False):
        for name in dirs + names:
            p = Path(parent, name)
            if p.is_symlink() and p.resolve(strict=False) in removed_paths and not p.exists():
                links.append({'path': str(p), 'target': os.readlink(p)})
                p.unlink()
    write('removed_history_links.json', links)
    after = compact_fingerprint()
    unchanged = after == data['compactBefore']
    preserved_ok = all(identity(Path(x['path'])) == {k:v for k,v in x.items() if k != 'reason'} for x in data['preservedArrays'])
    write('retention.json', {'status': 'verified' if unchanged and preserved_ok else 'verification_failed',
                           'preservedArrayBytes': data['preservedArrayBytes'], 'compactAfter': after,
                           'compactUnchanged': unchanged, 'preservedArraysUnchanged': preserved_ok,
                           'recordingsIntentionallyRemoved': len(removed), 'brokenOutputLinksRemoved': len(links),
                           'historicalManifestPolicy': 'Recorded hashes and outcomes are historical; deleted traces require regeneration. Original delivery ZIPs are unchanged.'})
    if not unchanged or not preserved_ok:
        raise SystemExit('Retention verification failed')
    print(json.dumps({'status': 'verified', 'removedFiles': len(removed), 'logicalGiBRemoved': sum(x['bytes'] for x in removed)/1024**3,
                      'retainedArrayGiB': data['preservedArrayBytes']/1024**3, 'removedLinks': len(links)}, indent=2))


if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('--execute', action='store_true')
    execute() if args.parse_args().execute else plan()
