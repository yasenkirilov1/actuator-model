"""Review completed native sensing cohorts and export inspectable chart data.

Plots use complete-recording peak metrics scored in MATLAB. Native waveform
figures preserve the exact peak and reduce only displayed samples. Nothing in
this recipe turns a part-anchored response envelope into hardware qualification.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path

import h5py
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
VAL = ROOT / 'acf_simulink/validation'
FIG = ROOT / 'reports/figures'
MODEL_HASH = 'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'
plt.rcParams.update({'font.size': 11, 'axes.spines.top': False,
                     'axes.spines.right': False, 'savefig.facecolor': 'white'})


def sha(path):
    with path.open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def receipt(folder, require_complete=True):
    path = VAL / folder / 'results.json'
    doc = json.loads(path.read_text())
    if require_complete:
        assert doc['status'] == 'complete', (folder, doc['status'])
    assert doc['frozenAcquisitionModelSha256'] == MODEL_HASH
    rows = doc['rows']
    if isinstance(rows, dict):
        rows = [rows]
    assert all(row['executed'] and row['nativeReturned'] for row in rows)
    assert len(rows) == doc['executed']
    assert sum(row['nativeSamples'] for row in rows) == doc['nativeSamples']
    for row in rows:
        assert max(row[k] for k in ['projectionBalanceResidual_N',
                                   'rigidAxialBalanceResidual_N',
                                   'lowerLinkBalanceResidual_N']) < 1e-8
    return doc, rows, {'file': str(path.relative_to(ROOT)), 'sha256': sha(path),
                      'recordings': len(rows), 'nativeSamples': doc['nativeSamples']}


def groups(folders):
    output, sources = [], []
    for folder in folders:
        doc, rows, source = receipt(folder)
        sources.append(source)
        for key in dict.fromkeys(row['group'] for row in rows):
            selected = [row for row in rows if row['group'] == key]
            worst = max(selected, key=lambda r: r['peakNormalForceError_N'])
            hardware = selected[0]['hardware']
            output.append({'cohort': folder, 'group': key,
                'recordings': len(selected), 'passes': sum(r['passed'] for r in selected),
                'designMarginPasses': sum(r['designMarginPassed'] for r in selected),
                'peakError_N': worst['peakNormalForceError_N'],
                'reserve_N': .5-worst['peakNormalForceError_N'],
                'nativeSamples': sum(r['nativeSamples'] for r in selected),
                'worstCaseId': worst['id'], 'worstSeed': worst['seedRequested'],
                'worstMassS_kg': worst['massS_kg'], 'worstForce_N': worst['reference_N'],
                'worstAngle_deg': worst['angle_deg'],
                'converter': hardware['converter'], 'rate_Hz': hardware['outputRate_Hz'],
                'cutoff_Hz': hardware['analogCutoff_Hz'],
                'converterGroupDelay_us': selected[0]['converterGroupDelay_s']*1e6,
                'peakVoltage_V': max(r['peakVoltage_V'] for r in selected),
                'peakCurrent_A': max(r['peakCurrent_A'] for r in selected),
                'anyClipping': any(r['anyAdcClipping'] for r in selected),
                'maximumVoltageClipFraction': max(r['voltageClipFraction'] for r in selected),
                'scope': doc['scope'], 'source': source})
    return output, sources


def save_doc(name, rows, sources, **extra):
    doc = {'status': 'complete', 'date': '2026-10-02',
           'frozenAcquisitionModelSha256': MODEL_HASH,
           'metric': 'maximum absolute true workpiece-normal force error over every native sample including startup',
           'physicalRequirement_N': .5, 'preferredDesignThreshold_N': .45,
           'sources': sources, 'rows': rows, **extra}
    (VAL/name).write_text(json.dumps(doc, indent=2, allow_nan=False)+'\n')
    return doc


def savefig(fig, name):
    fig.savefig(FIG/(name+'.png'), dpi=170, bbox_inches='tight')
    fig.savefig(FIG/(name+'.svg'), bbox_inches='tight')
    plt.close(fig)


def screen_fig(rows):
    cutoffs = [500,1000,2000,3000,5100,8000]
    fig, axes = plt.subplots(1,2,figsize=(13,5.2))
    for ax, converter, rates, title in zip(axes,['sar','sinc3'],
          [[100000,50000,20000,10000],[30000,15000,7500]],
          ['ADS8588S: native SAR + 16 kHz internal filter',
           'ADS131M08: native Sinc³ decimation']):
        matrix = np.array([[next(r['peakError_N'] for r in rows
                         if r['converter']==converter and r['rate_Hz']==rate
                         and r['cutoff_Hz']==fc) for fc in cutoffs] for rate in rates])
        im = ax.imshow(matrix,aspect='auto',cmap='RdYlGn_r',vmin=.25,vmax=.8)
        ax.set_xticks(range(6),[str(v/1000).rstrip('0').rstrip('.') for v in cutoffs])
        ax.set_yticks(range(len(rates)),[f'{v/1000:g} kSPS' for v in rates])
        for i,j in itertools.product(range(len(rates)),range(6)):
            ax.text(j,i,f'{matrix[i,j]:.3f}',ha='center',va='center',fontsize=10)
        ax.set_title(title,fontsize=12); ax.set_xlabel('Matched analog response cutoff (kHz)')
    axes[0].set_ylabel('Per-channel output rate')
    fig.suptitle('168 native bump recordings: sensing delay changes the force margin',fontsize=14)
    fig.subplots_adjust(bottom=.2,top=.81,left=.09,right=.86,wspace=.2)
    color_axis=fig.add_axes([.895,.23,.016,.55])
    fig.colorbar(im,cax=color_axis,label='Worst full-history force error (N)')
    fig.text(.5,.045,'Four cases/cell: heavy/light endpoints × clean/seed 7. Requirement ≤0.500 N; preferred margin ≤0.450 N.',ha='center',fontsize=10)
    savefig(fig,'budget_adc_bandwidth_rate')


def sensor_fig(rows):
    # Counterfactual wide-band ADXL354 curves remain inspectable but are omitted
    # from this figure's practical candidate comparison.
    selected = [r for r in rows if 'counterfactual' not in r['group']]
    labels=[]
    for r in selected:
        g=r['group']
        if 'ADXL354' in g:
            extra=g.split('_L')[-1].replace('us','')
            driver='50 µs current loop' if 'faster' in g else '100 µs current loop'
            labels.append(f'ADXL354-like, 1.9 kHz, +{extra} µs\n{driver}')
        else:
            density=g.split('_d')[-1].split('_')[0]
            extra=g.split('_L')[-1].replace('us','')
            labels.append(f'Digital envelope, {density} µg/√Hz\n5.1 kHz, +{extra} µs')
    fig,ax=plt.subplots(figsize=(10,7))
    errors=[r['peakError_N'] for r in selected]
    bars=ax.barh(range(len(selected)),errors,color=['#3a8c99' if e<=.45 else '#d69a3c' for e in errors])
    ax.set_yticks(range(len(selected)),labels,fontsize=10);ax.invert_yaxis()
    ax.axvline(.5,color='#b33b38',ls='--',label='±0.5 N limit')
    ax.axvline(.45,color='#555',ls=':',label='0.05 N design reserve')
    ax.set_xlim(0,.56);ax.set_xlabel('Worst full-history error (N)')
    for bar,e in zip(bars,errors):ax.text(e+.006,bar.get_y()+bar.get_height()/2,f'{e:.3f}',va='center',fontsize=10)
    ax.set_title('Cheaper-sensor noise and response envelopes: nominal bump screen')
    ax.legend(loc='lower right',fontsize=9)
    fig.tight_layout();savefig(fig,'budget_accelerometer_envelopes')


def waveform(folder):
    doc, rows, _ = receipt(folder)
    worst=max(rows,key=lambda r:r['peakNormalForceError_N'])
    path=VAL/folder/'worst_native.mat'
    with h5py.File(path) as h:
        t=h['trace/time'][0]
        error=h['trace/data'][14]-worst['reference_N']
        assert abs(np.max(np.abs(error))-worst['peakNormalForceError_N'])<1e-10
        peak=int(np.argmax(np.abs(error)))
        start=float(h['trace/parameters/surface/startTime'][0,0])
        repeat=float(h['trace/parameters/surface/repeatPeriod'][0,0]) if 'repeatPeriod' in h['trace/parameters/surface'] else 0
        entry=start+(np.floor((t[peak]-start)/repeat)*repeat if repeat>0 else 0)
        use=np.flatnonzero((t>=entry-.015)&(t<=entry+.145))
        stride=max(1,round(50e-6/(t[1]-t[0])))
        use=np.unique(np.r_[use[::stride],peak])
        return t[use]-entry,error[use],worst,path


def long_fig(folders):
    fig,ax=plt.subplots(figsize=(11,4.7))
    for folder in folders:
        t,e,w,_=waveform(folder)
        kind='SAR' if w['hardware']['converter']=='sar' else 'Sinc³'
        label=f"{kind}, {w['hardware']['outputRate_Hz']/1000:g} kSPS, {w['hardware']['analogCutoff_Hz']/1000:g} kHz: {w['peakNormalForceError_N']:.3f} N"
        ax.plot(t,e,label=label,lw=1.1)
    ax.axhline(.5,color='#b33b38',ls='--');ax.axhline(-.5,color='#b33b38',ls='--')
    ax.axhline(.45,color='#777',ls=':',lw=.7);ax.axhline(-.45,color='#777',ls=':',lw=.7)
    ax.set_xlim(-.015,.145);ax.set_ylim(-.55,.55)
    ax.set_xlabel('Time relative to the worst bump entry (s)');ax.set_ylabel('True normal-force error (N)')
    ax.set_title('30 s calibration stress: the bump containing each complete-recording worst error')
    ax.legend(loc='lower right',fontsize=10)
    ax.grid(alpha=.18);fig.tight_layout();savefig(fig,'budget_adc_long_waveforms')


def stress_fig(rows):
    labels=[]
    for r in rows:
        name=r['group']
        if 'ADXL1002' in name:label='Reference-density envelope, 8 kHz\n100 µs current loop'
        elif 'digital70' in name:
            speed='50' if 'current50' in name else '100'
            label=f'Digital envelope, 70 µg/√Hz, +80 µs\n{speed} µs current loop'
        else:
            speed='50' if 'current50' in name else '100'
            extra='50' if 'L50' in name else '0'
            hot=', hot 32 V' if 'hot32V' in name else ''
            label=f'ADXL354-like, 1.9 kHz, +{extra} µs\n{speed} µs current loop{hot}'
        labels.append(label)
    fig,ax=plt.subplots(figsize=(10,6.8))
    errors=[r['peakError_N'] for r in rows]
    bars=ax.barh(range(len(rows)),errors,color=['#378b8b' if e<=.45 else '#d99f3d' if e<=.5 else '#bf5651' for e in errors])
    ax.set_yticks(range(len(rows)),labels,fontsize=10);ax.invert_yaxis()
    for bar,r in zip(bars,rows):
        ax.text(r['peakError_N']+.005,bar.get_y()+bar.get_height()/2,
                f"{r['peakError_N']:.3f} N; {r['passes']}/{r['recordings']} pass",va='center',fontsize=10,
                bbox=dict(facecolor='white',edgecolor='none',alpha=.9,pad=1.5))
    ax.axvline(.5,color='#a3312e',ls='--',label='±0.5 N limit')
    ax.axvline(.45,color='#555',ls=':',label='0.05 N design reserve')
    ax.set_xlim(0,max(.65,max(errors)+.14));ax.set_xlabel('Worst full-history force error (N)')
    ax.set_title('49 native joint-error comparisons: four 30 s + three 4 s cases per envelope')
    ax.legend(loc='lower right',fontsize=9);fig.tight_layout()
    savefig(fig,'budget_sensor_calibration_stress')


def robustness_fig(rows):
    selected=[r for r in rows if 'alias' not in r['cohort'] and 'refine' not in r['cohort']]
    names={'adc_sinc_noise2x_20261002':'2× ADC typical noise',
           'adc_sinc_noise_corr200us_20261002':'200 µs ADC noise correlation',
           'adc_sinc_acc_skew20us_20261002':'+20 µs acceleration-only skew',
           'adc_sinc_acc_skew50us_20261002':'+50 µs acceleration-only skew'}
    fig,ax=plt.subplots(figsize=(9,4))
    values=[r['peakError_N'] for r in selected]
    bars=ax.barh(range(len(values)),values,color='#3a8c99')
    ax.set_yticks(range(len(values)),[names[r['cohort']] for r in selected],fontsize=11);ax.invert_yaxis()
    for bar,e in zip(bars,values):ax.text(e+.005,bar.get_y()+bar.get_height()/2,f'{e:.3f}',va='center')
    ax.axvline(.5,color='#a3312e',ls='--');ax.axvline(.45,color='#555',ls=':')
    ax.set_xlim(0,.54);ax.set_xlabel('Worst full-history force error (N)')
    ax.set_title('30 kSPS / 8 kHz acquisition: nominal heavy-tool robustness probes')
    fig.tight_layout();savefig(fig,'budget_sensing_response_noise')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--final',action='store_true');args=parser.parse_args()
    FIG.mkdir(exist_ok=True)
    screen, sources=groups(['adc_sar_screen_20261002','adc_sinc_screen_20261002'])
    assert sum(r['recordings'] for r in screen)==168
    save_doc('budget_adc_screen_summary_20261002.json',screen,sources,
             fastRestartAudit='acf_simulink/validation/adc_fast_restart_audit_20261002/audit_seal.json')
    screen_fig(screen)
    sensors, sources=groups(['budget_sensors_screen_20261002'])
    save_doc('budget_accelerometer_screen_summary_20261002.json',sensors,sources,
             caveat='Part-anchored noise/range and required matched response envelopes, not exact real-part transfers. Counterfactual ADXL354 bandwidths are excluded from recommendations.')
    sensor_fig(sensors)
    folders=['adc_sinc_long5100_20261002','adc_sinc_long8000_20261002']
    if (VAL/'adc_sinc_long15000_20261002/results.json').exists():
        if json.loads((VAL/'adc_sinc_long15000_20261002/results.json').read_text())['status']=='complete':
            folders.append('adc_sinc_long15000_20261002')
    if (VAL/'adc_sar_long5100_20261002/results.json').exists():
        if json.loads((VAL/'adc_sar_long5100_20261002/results.json').read_text())['status']=='complete':
            folders.append('adc_sar_long5100_20261002')
    rows,sources=groups(folders)
    long_count=sum(r['recordings'] for r in rows)
    save_doc('budget_adc_long_summary_20261002.json',rows,sources)
    long_fig(folders)
    if args.final:
        grid=[];sources=[]
        for part in range(1,4):
            _,rows,source=receipt(f'adc_budget_range_20261002_part{part}')
            grid += rows;sources.append(source)
        expected=set(itertools.product([.1,.5,1,1.4,2],range(5,51,5),range(0,91,15),[0,.3],[29]))
        observed=[(r['massS_kg'],r['reference_N'],round(r['angle_deg']),r['mu'],r['seedRequested']) for r in grid]
        assert len(grid)==700 and len(set(observed))==700 and set(observed)==expected
        assert len({r['id'] for r in grid})==700
        save_doc('budget_adc_range_grid_20261002.json',sorted(grid,key=lambda r:r['id']),sources,
                 uniqueConditions=700,noiseSeeds=[29],nativeSamples=sum(r['nativeSamples'] for r in grid),
                 passes=sum(r['passed'] for r in grid),worstError_N=max(r['peakNormalForceError_N'] for r in grid),
                 caveat='One realization per condition; this does not replace the historical 3500-case five-seed scientific reference.')
        stress_manifest=json.loads((VAL/'budget_sensor_stress_20261002_inputs/manifest.json').read_text())
        stress_folders=['budget_sensor_stress_20261002_'+g['name'] for g in stress_manifest['groups']]
        stress,sources=groups(stress_folders)
        assert sum(r['recordings'] for r in stress)==49
        save_doc('budget_accelerometer_stress_summary_20261002.json',stress,sources,
                 inputManifestSha256=sha(VAL/'budget_sensor_stress_20261002_inputs/manifest.json'),
                 caveat='Seven paired response/drive envelopes; each preserves four 30 s and three 4 s retained calibration/mechanics cases. No real-part phase guarantee.')
        stress_fig(stress)
        matching,matching_sources=groups(['budget_dynamic_matching_L0_20261002'])
        save_doc('budget_dynamic_matching_summary_20261002.json',matching,matching_sources,
                 caveat='Seven executed L0 cases. L50 inputs were prepared but not executed. Ten dynamic-matching residuals shrink; static residuals, common pipeline, robot,mechanics,noise and drive remain unchanged.')
        robust_names=['adc_sinc_noise2x_20261002','adc_sinc_noise_corr200us_20261002',
                      'adc_sinc_acc_skew20us_20261002','adc_sinc_acc_skew50us_20261002',
                      'adc_sinc_refine_20261002','adc_sinc_alias_probe_20261002','adc_sar_alias_probe_20261002']
        robust,sources=groups(robust_names)
        save_doc('budget_sensing_robustness_20261002.json',robust,sources,
                 caveat='One-at-a-time nominal endpoint or quiet synthetic tone probes; not a simultaneous calibration-error box or measured sanding spectrum.')
        robustness_fig(robust)
        hot,hot_sources=groups(['budget_adc_hot_coil_20261002'])
        assert sum(r['recordings'] for r in hot)==7
        save_doc('budget_adc_hot_coil_summary_20261002.json',hot,hot_sources,
                 inputManifestSha256=sha(VAL/'budget_adc_hot_coil_20261002_inputs/manifest.json'),
                 caveat='Seven native selected-acquisition records with fixed hot Kf24.4 N/A, R8.896 ohm, L4 mH, 32 V usable output and unchanged 100 us current response. No evolving thermal state or measured assembled sensor/driver phase.')
        all_folders=['adc_sar_screen_20261002','adc_sinc_screen_20261002','budget_sensors_screen_20261002',
                     'adc_sinc_long5100_20261002','adc_sinc_long8000_20261002',
                     'adc_sinc_long15000_20261002','adc_sar_long5100_20261002',
                     'budget_dynamic_matching_L0_20261002','budget_adc_hot_coil_20261002']+stress_folders+robust_names+[
                     f'adc_budget_range_20261002_part{p}' for p in range(1,4)]
        recordings=[];sources=[]
        for folder in all_folders:
            doc,items,source=receipt(folder);sources.append(source)
            recordings.append({'cohort':folder,'recordings':len(items),
                  'passes':sum(r['passed'] for r in items),
                  'preferredMarginPasses':sum(r['designMarginPassed'] for r in items),
                  'nativeSamples':doc['nativeSamples'],
                  'peakError_N':doc['worstNormalForceError_N'],
                  'kind':'quiet_synthetic_tone' if 'alias_probe' in folder else 'bump_recordings'})
        assert sum(r['recordings'] for r in recordings)==999
        save_doc('budget_sensing_execution_ledger_20261002.json',recordings,sources,
                 executedRecordings=999,bumpRecordings=993,quietToneRecordings=6,
                 nativeForceSamples=sum(r['nativeSamples'] for r in recordings),
                 parityAuditRecordings=9,
                 caveat='Recording count includes deliberately paired repeats; not 999 unique parameter identities. Pilot/software/source checks are separate; no prepared-only L50 matching cases are counted.')
    print(json.dumps({'screenRecordings':168,'sensorScreenRecordings':36,'longRecordings':long_count,
                      'executedRecordings':999 if args.final else None}))


if __name__=='__main__':main()
