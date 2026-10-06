"""Seal/review the dated workspace delivery without replacing the older ZIP.

Hashes bind reports, frozen source/inputs, receipts and retained failures.
The numerical review streams three selected COMPLETE native histories, not
their plotted excerpts. Use --seal only when authoring a new dated delivery.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VAL = ROOT / 'acf_simulink/validation'
MANIFEST = VAL / 'budget_sensing_delivery_manifest_20261002.json'
CHECKS = VAL / 'budget_sensing_delivery_checks_20261002.json'
MODEL_HASH = 'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'
PUBLIC_HASH = '48287e6a17bc9d3144e3d6a77c59ecc601ba4fa26a7ec748173dc89d9bf718de'
OLD_ZIP_HASH = '0e91119ff142b20664d0a9620b37aef65e8d27a81f7dd5ecfeecbadff35934f0'
SELECTED = [
    'adc_sinc_long8000_20261002',
    'budget_adc_hot_coil_20261002',
    'budget_sensor_stress_20261002_ADXL354C_1900_L50_current50_36V',
]


def read(path):
    return json.loads(path.read_text())


def digest(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def entries(doc):
    return [doc['rows']] if isinstance(doc['rows'], dict) else doc['rows']


def seal():
    files = {}
    def add(path, role):
        assert path.is_file(), path
        relative = str(path.relative_to(ROOT))
        if relative not in files:
            files[relative] = {'file': relative, 'role': role,
                               'bytes': path.stat().st_size, 'sha256': digest(path)}
    for path in [ROOT/'reports/active-contact-flange-report.md',
                 ROOT/'reports/active-contact-flange-report.html',
                 ROOT/'reports/sensing-cost-study-20261002.md',
                 VAL/'budget_sensing_report_inputs_20261002.json',
                 VAL/'budget_sensing_html_report_manifest_20261002.json',
                 ROOT/'reports/prototype-design-20260930/src/data.json',
                 ROOT/'reports/prototype-design-20260930/src/content/report/ReportContent.jsx',
                 ROOT/'reports/prototype-design-20260930/src/content/report/report.css',
                 ROOT/'reports/prototype-design-20260930/dist/data-app-build.json']:
        add(path, 'current_report')
    report = read(VAL/'budget_sensing_html_report_manifest_20261002.json')
    for record in report['evidence'] + report['figures']:
        add(ROOT/record['file'], 'report_source_or_figure')
    for path in (ROOT/'reports/figures').glob('budget_*.svg'):
        add(path, 'exportable_figure')
    source = VAL/'adc_acquisition_trial_20261002'
    for path in source.iterdir():
        if path.is_file():
            add(path, 'frozen_native_source')
    for path in (source/'helpers').glob('*.m'):
        add(path, 'frozen_helper')
    for folder in ['budget_accelerometer_envelopes_20261002_inputs',
                   'budget_sensor_stress_20261002_inputs',
                   'budget_dynamic_matching_20261002_inputs',
                   'budget_adc_hot_coil_20261002_inputs']:
        for path in (VAL/folder).iterdir():
            if path.is_file():
                add(path, 'frozen_preparation_inputs_not_execution_claim')
    for pattern in ['*cost_component_review_20261002.*',
                    'budget_accelerometer_response_review_20261002.md',
                    'budget_sensor_parallel_handoff_20261002.json']:
        for path in VAL.glob(pattern):
            if path.is_file():
                add(path, 'component_or_workflow_evidence')
    for path in (ROOT/'acf_simulink/experiments').glob('*20261002*'):
        if path.is_file() and (path.suffix in {'.m','.py','.md'}):
            add(path, 'reproduction_recipe')
    for name in ['audit_seal.json','results.json','README.md',
                 'clock_delay_parameter_inventory.json','frozen_helper_manifest.json']:
        add(VAL/'adc_fast_restart_audit_20261002'/name, 'compilation_parity_audit')
    ledger = read(VAL/'budget_sensing_execution_ledger_20261002.json')
    for cohort in ledger['rows']:
        folder = VAL/cohort['cohort']
        for name in ['results.json','rows.jsonl','input_configs.mat','prepared_configs.mat']:
            add(folder/name, 'executed_cohort_receipt_or_exact_inputs')
        for row in entries(read(folder/'results.json')):
            if not row['passed']:
                add(folder/f"case_{row['id']:04d}_native.mat", 'retained_failed_native_history')
    for folder in SELECTED:
        add(VAL/folder/'worst_native.mat', 'selected_full_native_history')
    add(ROOT/'acf_simulink/generated/acf_force_control_simple.slx', 'unchanged_public_model')
    add(ROOT/'acf_simulink/generated/active_contact_flange_inertial_20260930.zip',
        'preserved_original_archive')
    base = VAL/'budget_sensing_report_base_20261002'
    add(base/'manifest.json', 'preserved_pre_study_report')
    for record in read(base/'manifest.json')['files']:
        add(base/record['file'], 'preserved_pre_study_report')
    add(base/'reports/prototype-design-20260930/src/content/report/report.css',
        'preserved_before_mobile_path_wrap_fix')
    for name in ['bandwidth-final.png','stress-final.png','waveforms-final.png',
                 'mobile-final.png','offline-final.png']:
        add(ROOT/'output/playwright/acf-sensing-20261002'/name, 'rendered_browser_check')
    add(VAL/'budget_sensing_browser_checks_20261002.json', 'browser_check_receipt')
    manifest = {'status':'complete_versioned_workspace_delivery', 'date':'2026-10-02',
        'reportId':'report:578d0259-115c-40f2-9264-4c57b68ab138',
        'modelSha256':MODEL_HASH, 'publicModelUnchangedSha256':PUBLIC_HASH,
        'originalArchivePreservedSha256':OLD_ZIP_HASH,
        'executedRecordings':999, 'nominalGridConditions':700,
        'selectedFullHistoryCohorts':SELECTED,
        'scope':'Local versioned reports and inspectable numerical evidence. Original archive retained; no new hardware validation or public-model promotion.',
        'files':sorted(files.values(), key=lambda r:r['file'])}
    MANIFEST.write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'sealedFiles':len(files),'manifest':str(MANIFEST.relative_to(ROOT))}),flush=True)


def native_review(folder):
    import h5py
    import numpy as np
    document = read(folder/'results.json')
    expected = max(entries(document), key=lambda r:r['peakNormalForceError_N'])
    path = folder/'worst_native.mat'
    peak = 0.; balance = 0.; voltage = 0.; current = 0.
    with h5py.File(path) as handle:
        data = handle['trace/data']; times = handle['trace/time']
        assert data.shape == (19, expected['nativeSamples'])
        p = handle['trace/parameters']
        scalar = lambda key: float(p[key][0,0])
        dt = scalar('sim/step'); mb = scalar('mechanics/mB'); ms = scalar('mechanics/mS')
        gravity = scalar('d/g'); target = scalar('force/reference')
        previous = -dt
        for at in range(0,data.shape[1],200000):
            a = data[:,at:at+200000]; t = times[0,at:at+200000]
            assert np.isfinite(a).all() and np.isfinite(t).all()
            assert abs(t[0]-previous-dt)<1e-12
            if len(t)>1:
                assert np.max(np.abs(np.diff(t)-dt))<1e-12
            previous = t[-1]
            peak = max(peak,float(np.max(np.abs(a[14]-target))))
            current = max(current,float(np.max(np.abs(a[2]))))
            voltage = max(voltage,float(np.max(np.abs(a[3]))))
            balance = max(balance,float(np.max(np.abs(a[14]-(a[0]-a[17]*a[16])/np.sqrt(1+a[17]**2)))),
                float(np.max(np.abs(a[0]-a[11]-mb*(gravity+a[12])-ms*(gravity+a[18])))),
                float(np.max(np.abs(a[0]-a[13]-ms*(gravity+a[18])))))
        assert abs(peak-expected['peakNormalForceError_N'])<1e-10
        assert abs(current-expected['peakCurrent_A'])<1e-10
        assert abs(voltage-expected['peakVoltage_V'])<1e-10
        assert balance<1e-8
        assert abs(previous-expected['endTime_s'])<1e-12
    return {'file':str(path.relative_to(ROOT)), 'samples':expected['nativeSamples'],
            'peakError_N':peak, 'peakCurrent_A':current, 'peakVoltage_V':voltage,
            'maximumForceBalanceResidual_N':balance, 'matchesCompleteReceipt':True}


def verify():
    manifest = read(MANIFEST)
    indexed = {record['file']:record for record in manifest['files']}
    for record in manifest['files']:
        path = ROOT/record['file']
        assert path.stat().st_size == record['bytes'], path
        assert digest(path) == record['sha256'], path
    assert indexed['acf_simulink/validation/adc_acquisition_trial_20261002/acf_force_control_adc_trial.slx']['sha256'] == MODEL_HASH
    assert indexed['acf_simulink/generated/acf_force_control_simple.slx']['sha256'] == PUBLIC_HASH
    assert indexed['acf_simulink/generated/active_contact_flange_inertial_20260930.zip']['sha256'] == OLD_ZIP_HASH
    ledger = read(VAL/'budget_sensing_execution_ledger_20261002.json')
    assert ledger['status']=='complete' and ledger['executedRecordings']==999
    assert sum(r['recordings'] for r in ledger['rows'])==999
    assert sum(r['nativeSamples'] for r in ledger['rows'])==1502400999
    grid = read(VAL/'budget_adc_range_grid_20261002.json')
    expected = set(itertools.product([.1,.5,1,1.4,2],range(5,51,5),range(0,91,15),[0,.3],[29]))
    observed = [(r['massS_kg'],r['reference_N'],round(r['angle_deg']),r['mu'],r['seedRequested']) for r in grid['rows']]
    assert len(observed)==700 and len(set(observed))==700 and set(observed)==expected
    assert all(r['passed'] for r in grid['rows'])
    assert len([f for f in manifest['files'] if f['role']=='frozen_helper'])==18
    app = read(ROOT/'reports/prototype-design-20260930/src/data.json')
    assert app['id']==manifest['reportId'] and app['buildStatus']=='complete'
    assert len(app['queries'])==48
    text = (ROOT/'reports/active-contact-flange-report.md').read_text()
    headings = [line for line in text.splitlines() if line.startswith('## ')]
    assert headings[0]=='## Our goal' and headings[-1]=='## Concrete recommendation'
    assert '**Working report:**' not in text and '**Provisional choice:**' not in text
    histories = []
    for name in SELECTED:
        history = native_review(VAL/name)
        histories.append(history)
        print(json.dumps({'reviewedCompleteHistory':name,'samples':history['samples'],
                          'peakError_N':history['peakError_N']}),flush=True)
    checks = {'status':'passed','manifestSha256':digest(MANIFEST),
        'verifiedFiles':len(manifest['files']),'originalArchiveAndPublicModelUnchanged':True,
        'completeRecordingCount':999,'uniqueNominalGridConditions':700,
        'nativeForceSamplesInExecutedCohorts':1502400999,
        'selectedCompleteNativeHistories':histories,
        'caveat':'Checks verify retained software/artifact evidence, not physical prototype or manufacturer response.'}
    CHECKS.write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps({'status':'passed','files':len(manifest['files']),
                     'completeNativeHistories':len(histories)}),flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser();parser.add_argument('--seal',action='store_true')
    arguments = parser.parse_args()
    if arguments.seal:
        seal()
    verify()
