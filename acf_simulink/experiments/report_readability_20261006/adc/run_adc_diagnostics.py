"""Execute emitted native ADC forks and paired full-history diagnostics.

No Python plant or controller. All maxima use every native sample including
startup. Controller/modal design remains frozen; reports are diagnostic only.
"""
from __future__ import annotations
import argparse, hashlib, json, shutil, sys, time
from datetime import datetime, timezone
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
API=ROOT/'acf_simulink/experiments/upper_no_damping_20261005/qualification_v8'
sys.path.insert(0,str(API))
from native_api import NativePlant

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def arraysha(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()
def write(path,obj):Path(path).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')

def execute(plant,index):
    start=time.monotonic();ctx=plant.create(index,baseline=True,record=True)
    try:
        duration=plant.scenarios[index]['duration_s'];action=np.zeros((1,2),dtype=np.float64)
        while True:
            snap=ctx.snapshot()
            if snap.terminated or snap.end_s>=duration-plant.metadata['actionStep_s']/2:break
            count=max(1,round((duration-snap.end_s)/plant.metadata['actionStep_s']))
            plant.step_repeat_batch_arrays([ctx],action,min(100,count))
        final=ctx.snapshot();r=ctx.take_record(copy=True)
    finally:ctx.close()
    t,d=r['time'],r['truth'];case=dict(plant.scenarios[index]);n=len(t)
    assert n>0 and np.isfinite(d).all() and np.isfinite(t).all()
    dt=plant.metadata['step_s'];ticks=np.rint(t/dt).astype(np.int64)
    grid=bool(t[0]==0 and n==ticks[-1]+1 and np.all(np.diff(ticks)==1) and np.max(abs(t-ticks*dt))<1e-12)
    error=d[:,0]-case['force_N'];ipeak=int(np.argmax(abs(error)))
    complete=bool(t[-1]>=duration-dt/2);stop=bool(final.travel_stop or np.any(d[:,21]>.5));contact=bool(final.contact_loss or np.any(d[:,18]<.5))
    band=abs(d[:,2]-case['midTravel_m'])>.0075+1e-12
    case.update(nativeSamples=n,allNativeSamplesIncludingStartupScored=True,nativeGridComplete=grid,
        completed=complete,normalForcePeakError_N=float(abs(error[ipeak])),peakErrorTime_s=float(t[ipeak]),
        rmsNormalForceError_N=float(np.sqrt(np.mean(error**2))),forceOutsideSamples=int(np.sum(abs(error)>.5)),
        voltageClipSamples=int(np.sum(d[:,23]>.5)),currentLimitSamples=int(np.sum(d[:,22]>.5)),
        peakCurrent_A_actual=float(abs(d[:,14]).max()),peakVoltage_V_actual=float(abs(d[:,15]).max()),
        contactLoss=contact,travelStop=stop,physicalFailure=stop or contact,travelBandExcursion=bool(band.any()),
        nativeLastTime_s=float(t[-1]),wallTime_s=time.monotonic()-start,
        passed=bool(grid and complete and not (stop or contact) and abs(error[ipeak])<=.5),
        arrayHashes={k:arraysha(v) for k,v in r.items()},qualificationRecordings=0)
    return case,r

def main():
    ap=argparse.ArgumentParser();ap.add_argument('storage',type=Path);args=ap.parse_args();base=args.storage.resolve()
    manifest=base/'preparation_manifest.json';m=json.loads(manifest.read_text())
    assert sha(m['sourceModel'])==m['sourceModelSha256'] and sha(m['sourceConfigs'])==m['sourceConfigsSha256']
    out=base/'executed';assert not out.exists(),'Preserve earlier executed evidence';out.mkdir()
    seals=[]
    for export in m['exports']:
        p=Path(export['folder']);assert sha(p/'acf_rl_native.slx')==export['modelSha256'];assert sha(p/'bank/configs.mat')==export['configsSha256']
        plant=NativePlant(p,rebuild=True,threads=1)
        source=p/'code/acf_rl_native_grt_rtw/acf_rl_native.c'
        import re
        schedule=re.findall(r'if \(\(acf_rl_native_M->Timing.TaskCounters.TID\[\d+\]\) > \d+\).*?Sample time: \[([^,]+),',source.read_text())
        assert any(abs(float(v.rstrip('s'))-1/export['rate_Hz'])<1e-9 for v in schedule),schedule
        files=[p/'acf_rl_native.slx',p/'bank/configs.mat',p/'metadata.json',p/'scenarios.json',p/'parameter_mapping.json',p/'native_parameter_bank.c',source,p/'libacf_native.so',p/'bridge_build.json']
        seal={'group':export['group'],'files':{str(f):sha(f) for f in files},'emittedSchedulePeriods_s':schedule,'bankZeroExactlyMatchesCoder':True}
        write(p/'diagnostic_native_seal.json',seal);seals.append(seal)
    reference={};rows=[]
    # Hold at most seven reference force arrays; whole native traces go to SATA.
    # Diagnostics preserve force residuals, all case metadata and native replay recipes.
    for export in m['exports']:
        folder=Path(export['folder']);plant=NativePlant(folder,threads=1)
        for index in range(len(plant.scenarios)):
            case,r=execute(plant,index);source=case['sourceScenario'];t,d=r['time'],r['truth']
            path=out/case['identity'];path.mkdir()
            is_reference=case['outputRate_Hz']==30000 and case['quantizationBits']==24
            if is_reference:
                reference[source]=(t.copy(),d[:,0].copy(),case.copy())
                assert shutil.disk_usage(out).free>2*1024**3
                f=path/'native_history.npz';np.savez_compressed(f,**r)
                case.update(historyFile=str(f),historySha256=sha(f))
                case.update(pairedNormalForceWaveformDifference_N=0.,pairedPeakErrorChange_N=0.,pairedCommonSamples=len(t),pairedReferencePeakError_N=case['normalForcePeakError_N'])
            else:
                rt,rf,ref=reference[source];n=min(len(rt),len(t));assert np.array_equal(rt[:n],t[:n])
                residual=d[:n,0]-rf[:n];j=int(np.argmax(abs(residual)))
                case.update(pairedNormalForceWaveformDifference_N=float(abs(residual[j])),pairedWaveformPeakDifferenceTime_s=float(t[j]),
                    pairedPeakErrorChange_N=case['normalForcePeakError_N']-ref['normalForcePeakError_N'],
                    pairedCommonSamples=n,pairedEntireDuration=bool(len(rt)==len(t)),pairedReferencePeakError_N=ref['normalForcePeakError_N'])
                # Full native force/voltage/travel history and residual retained for every diagnostic.
                f=path/'force_residual_history.npz'
                np.savez_compressed(f,time=t,normal_force_N=d[:,0],voltage_V=d[:,15],current_A=d[:,14],travel_m=d[:,2],paired_force_residual_N=residual)
                case.update(forceResidualHistoryFile=str(f),forceResidualHistorySha256=sha(f))
                # Representative native failure/current-limit histories are retained;
                # all other 27-channel traces regenerate from the sealed native bank.
                if case['physicalFailure'] or (source==170 and not case['passed']):
                    f=path/'native_failure_history.npz';np.savez_compressed(f,**r)
                    case.update(historyFile=str(f),historySha256=sha(f))
            write(path/'result.json',case);rows.append(case)
            print(json.dumps({'case':case['identity'],'peak_N':case['normalForcePeakError_N'],'paired_N':case['pairedNormalForceWaveformDifference_N'],'passed':case['passed']}),flush=True)
            del r,t,d
    for seal in seals:
        assert all(sha(p)==h for p,h in seal['files'].items())
    assert sha(m['sourceModel'])==m['sourceModelSha256'] and sha(m['sourceConfigs'])==m['sourceConfigsSha256']
    grouped=[]
    for key in sorted({(r['outputRate_Hz'],r['quantizationBits']) for r in rows},reverse=True):
        part=[r for r in rows if (r['outputRate_Hz'],r['quantizationBits'])==key]
        w=max(part,key=lambda r:r['normalForcePeakError_N'])
        grouped.append(dict(rate_Hz=key[0],quantizationBits=key[1],cases=len(part),passes=sum(r['passed'] for r in part),
            worstTotalPeakError_N=w['normalForcePeakError_N'],worstScenario=w['sourceScenario'],
            largestPairedWaveformDifference_N=max(r['pairedNormalForceWaveformDifference_N'] for r in part),
            largestPairedPeakErrorIncrease_N=max(r['pairedPeakErrorChange_N'] for r in part),
            physicalFailures=sum(r['physicalFailure'] for r in part),nativeSamples=sum(r['nativeSamples'] for r in part)))
    result=dict(status='complete_diagnostic_evidence_only',executed_at_utc=datetime.now(timezone.utc).isoformat(),
        executedDiagnosticRecordings=len(rows),qualificationRecordings=0,controllerRetuned=False,
        sampling_and_converter_delay_reexported=True,originalSourceInputsPreserved=True,
        sourcePreparationManifest=str(manifest),sourcePreparationManifestSha256=sha(manifest),
        recipe=str(Path(__file__).resolve()),recipeSha256=sha(__file__),nativeSeals=seals,
        nativeSamples=sum(r['nativeSamples'] for r in rows),groups=grouped,rows=rows,
        scope='Seven selected known development corners, fixed PI/observer/modal design. Rate cases change actual sinc3 OSR/noise/output hits and causal acquisition frame. Bit cases change only code spacing at original M08 noise/timing. No ADC chip or complete controller is qualified by this subset.')
    write(out/'results.json',result)
    import csv
    with (out/'paired_results.csv').open('w',newline='') as f:
        fields=['identity','sourceScenario','massS_kg','force_N','angle_deg','outputRate_Hz','quantizationBits','normalForcePeakError_N','pairedNormalForceWaveformDifference_N','pairedPeakErrorChange_N','passed','physicalFailure','nativeSamples']
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    print(json.dumps(result['groups'],indent=2),flush=True)

if __name__=='__main__':main()
