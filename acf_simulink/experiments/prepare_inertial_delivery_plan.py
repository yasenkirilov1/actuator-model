"""Select the dated delivery explicitly, preserving historical native evidence.

Small evidence/configuration files are included by named scope. Large native
histories are included only through the documented selection below. Every
member is resolved and hash-bound before the package writer runs.
"""
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VALIDATION = ROOT / 'acf_simulink' / 'validation'
PLAN = VALIDATION / 'inertial_delivery_package_plan.json'


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def main():
    members = {}

    def add(source, role, member=None, expected=None):
        path = Path(source)
        if not path.is_absolute():
            path = ROOT / path
        assert path.is_file(), path
        name = member or str(path.relative_to(ROOT))
        sha = digest(path)
        assert expected is None or sha == expected, path
        if name in members:
            assert members[name]['expectedSha256'] == sha, name
            return
        members[name] = {'source': str(path), 'archivePath': name,
                         'expectedSha256': sha, 'bytes': path.stat().st_size,
                         'role': role}

    inventory = json.loads((VALIDATION / 'inertial_portable_source_inventory_20260930.json').read_text())
    for item in inventory['files']:
        add(item['originalAbsolutePath'], item['role'],
            item['suggestedProjectRelativeArchiveMember'], item['sha256'])
    required = [item for item in inventory['selectedNativeAuditDependencies']
                if item['requiredFor'].startswith('Required')]
    assert len(required) == 8
    for item in required:
        add(item['originalAbsolutePath'], item['role'],
            item['suggestedProjectRelativeArchiveMember'], item['sha256'])

    for path in (ROOT / 'acf_simulink').iterdir():
        if path.is_file() and path.suffix in {'.m', '.md'}:
            add(path, 'Public native-model helper or guide')
    add('AGENTS.md', 'Current project context')
    add('acf_simulink.prj', 'MATLAB project entry point')
    for path in (ROOT / 'resources').rglob('*'):
        if path.is_file():
            add(path, 'Current MATLAB project metadata')
    for path in (ROOT / 'acf_simulink' / 'experiments').iterdir():
        if path.is_file() and path.suffix in {'.m', '.py'}:
            add(path, 'Recorded study, reporting or delivery recipe')
    add('acf_simulink/generated/acf_force_control_simple.slx', 'Accepted native model')
    for name in ['active_flange_native_model_pages_20260930.pdf',
                 'active_contact_flange_inertial_worst_20260930.html']:
        add(ROOT / 'acf_simulink' / 'generated' / name, 'Reviewed diagram or actual native replay')
    for path in (ROOT / 'acf_simulink' / 'generated' / 'native_block_pages_20260930').rglob('*'):
        if path.is_file():
            add(path, 'Reviewed native diagram page')
    for path in (ROOT / 'acf_simulink' / 'viewer').iterdir():
        if path.is_file():
            add(path, 'Standalone replay template')
    for name in ['active-contact-flange-report.md', 'active-contact-flange-report.html']:
        add(ROOT / 'reports' / name, 'Completed design report')
    for path in (ROOT / 'reports' / 'figures').iterdir():
        if path.suffix in {'.png', '.svg'}:
            add(path, 'Source-bound scientific report figure')
    app = ROOT / 'reports' / 'prototype-design-20260930'
    add(app / 'AGENTS.md', 'Report authoring boundaries')
    for name in ['index.html', 'package.json', 'package-lock.json',
                 'vite.config.js', 'protected-runtime.json']:
        add(app / name, 'Preserved report build/runtime identity')
    for name in ['docs', 'scripts', 'public']:
        for path in (app / name).rglob('*'):
            if path.is_file():
                add(path, 'Preserved report build/runtime support')
    for path in (app / 'src').rglob('*'):
        if path.is_file():
            add(path, 'Preserved report authoring source and reviewed data')
    add(app / 'dist' / 'data-app-build.json', 'Official split-build identity')
    for path in (ROOT / 'output' / 'playwright').glob('inertial_report_final_*.png'):
        add(path, 'Final report browser visual verification')

    # Include small historical text evidence with its original status/scope.
    # Presence in the archive does not promote a planned or rejected run.
    for path in VALIDATION.iterdir():
        if path.is_file() and path != PLAN:
            if path.suffix in {'.json', '.jsonl', '.csv', '.md'}:
                add(path, 'Retained study evidence or analytical provenance')
            elif path.suffix == '.mat' and path.stat().st_size < 25_000_000 and re.search(
                    r'candidate|config|parameters|results|plot_traces', path.name):
                add(path, 'Recorded prepared parameters or compact study data')
    scopes = ['inertial_uncertainty_v8_portable_inputs',
              'inertial_mechanics_mixed_source_20260930',
              'inertial_mechanics_calibrated_static_mixed_source_20260930',
              'normal_voltage_20260930_final_sensor_source',
              'normal_voltage_20260930_final_grid_missing_headroom_proof_source']
    for name in scopes:
        for path in (VALIDATION / name).rglob('*'):
            if path.is_file():
                add(path, 'Portable immutable study input')
    for name in ['inertial_uncertainty_v8_engineered_sensing',
                 'inertial_uncertainty_v8_robot_fine_delay',
                 'inertial_uncertainty_v8_joint',
                 'inertial_uncertainty_v8_joint_long',
                 'inertial_uncertainty_v8_calibrated_static_long']:
        for path in (VALIDATION / name).iterdir():
            if path.is_file() and path.stat().st_size < 25_000_000:
                if path.suffix in {'.json', '.csv', '.md', '.m', '.mat', '.log'}:
                    add(path, 'Recorded calibration cohort evidence and prepared inputs')

    numerical = json.loads((VALIDATION / 'normal_mass_controller_final_grid_20260930_solver_refinement_portable_summary.json').read_text())
    selected = []
    for row in numerical['rows']:
        add(row['nativeTrace'], 'Full native nominal worst or numerical-refinement history',
            expected=row['nativeTraceSha256'])
        selected.append(str(Path(row['nativeTrace']).relative_to(ROOT)))
    for name in ['inertial_uncertainty_v8_original_long_peak_decomposition.json',
                 'inertial_uncertainty_v8_calibrated_static_long_peak_decomposition.json']:
        data = json.loads((VALIDATION / name).read_text())
        add(data['nativeTrace'], 'Full native original or calibrated30s worst history',
            expected=data['nativeTraceSha256'])
        selected.append(str(Path(data['nativeTrace']).relative_to(ROOT)))
    mixed = json.loads((VALIDATION / 'inertial_mechanics_calibrated_static_mixed_20260930_results.json').read_text())
    worst = max(mixed['rows'], key=lambda row: row['peakNormalForceError_N'])
    add(worst['nativeTraceFile'], 'Full native calibrated mechanical-stress worst history',
        expected=worst['nativeTraceSHA256'])
    selected.append(str(Path(worst['nativeTraceFile']).relative_to(ROOT)))

    config = json.loads((VALIDATION / 'inertial_report_inputs.json').read_text())
    assert config['readyForDelivery']
    for spec in config['queries']:
        assert spec['file'] in members, spec['file']
    # Validate the final report's local links against the selected members.
    markdown = ROOT / 'reports' / 'active-contact-flange-report.md'
    for target in re.findall(r'\]\(([^)]+)\)', markdown.read_text()):
        if re.match(r'https?://', target):
            continue
        target = target.split('#')[0]
        if not target:
            continue
        resolved = (markdown.parent / target).resolve()
        assert resolved.is_file() or resolved.is_dir(), target
        relative = str(resolved.relative_to(ROOT))
        assert relative in members or any(name.startswith(relative + '/') for name in members), target

    historical = ROOT / 'acf_simulink' / 'generated' / 'active_contact_flange_package.zip'
    plan = {'status': 'reviewed_for_delivery', 'date': '2026-09-30',
            'scope': 'Native measured-inertia model, completed Markdown/offlineHTML report, audited3500-case finite grid, paired calibration/mechanics evidence, conditional voltage screen and reproducible selected native/frozen inputs. No hardware qualification.',
            'gridSummary': 'acf_simulink/validation/normal_mass_controller_final_grid_20260930_summary.json',
            'reportManifest': 'acf_simulink/validation/inertial_html_report_manifest.json',
            'browserChecks': 'acf_simulink/validation/inertial_report_browser_checks_20260930.json',
            'historicalArchive': str(historical.relative_to(ROOT)),
            'historicalArchiveSha256': digest(historical),
            'archive': 'acf_simulink/generated/active_contact_flange_inertial_20260930.zip',
            'completedChecks': {'finiteGridAudit': True, 'independentPhysicsReview': True,
                                'nativePublicAcceptance': True, 'sourceBoundFigures': True,
                                'reportBrowserAndOffline': True, 'actualNativeReplay': True,
                                'localMarkdownTargetsIncluded': True},
            'nativeTracePolicy': {'selectedFullHistories': selected,
                                  'historicalAggregateDependencies': [item['suggestedProjectRelativeArchiveMember'] for item in required],
                                  'otherHistories': 'Preserved in the workspace and original manifests with hashes; exact frozen configurations/recipes are included for reruns. Optional four grid first traces are excluded. All3500 configuration/metric identities are retained.',
                                  'historicalAuditRelocation': 'Original absolute-path certificates remain unchanged. Use the portable verifier; normalize separate copies to repeat the historical aggregate at another path.'},
            'files': sorted(members.values(), key=lambda item: item['archivePath'])}
    # The plan can include itself without a recursive expected digest; the
    # package manifest will bind its actual final bytes.
    plan['files'].append({'source': str(PLAN), 'archivePath': str(PLAN.relative_to(ROOT)),
                          'role': 'Explicit reviewed delivery file selection'})
    PLAN.write_text(json.dumps(plan, indent=2) + '\n')
    print(json.dumps({'files': len(plan['files']),
                      'bytes': sum(item.get('bytes', 0) for item in plan['files']),
                      'selectedFullHistories': len(selected),
                      'historicalNativeDependencies': len(required)}))


if __name__ == '__main__':
    main()
