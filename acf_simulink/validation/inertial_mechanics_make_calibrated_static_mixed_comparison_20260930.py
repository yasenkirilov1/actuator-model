#!/usr/bin/env python3
"""Export the six native mechanical/calibration histories as a scientific figure."""
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

ROOT=Path(__file__).resolve().parent
FILES=[ROOT/'inertial_mechanics_calibrated_mechanics_20260930_results.json',
       ROOT/'inertial_mechanics_calibrated_static_mixed_20260930_results.json']
reports=[json.loads(p.read_text()) for p in FILES]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert all(x['status']=='complete' and x['executedCases']==3 and x['passedCases']==3 for x in reports)
assert reports[1]['onlySixteenStaticChangesVerified']
rows=[]; curves=[]; primitiveChanges=[]
for series,report in zip(['original_static_budget','halved_static_residuals'],reports):
    for row in report['rows']:
        path=Path(row['nativeTraceFile'])
        assert sha(path)==row['nativeTraceSHA256']
        rows.append({'series':series,'seed':row['seed'],
                     'peakNormalForceError_N':row['peakNormalForceError_N'],
                     'forceMargin_N':.5-row['peakNormalForceError_N'],
                     'forcePassed':row['forcePassed'],'packingPassed':row['packingPassed'],
                     'actualStiffness_N_m':row['actualLowerStiffness_N_m'],
                     'actualDamping_Ns_m':row['actualLowerDamping_Ns_m'],
                     'identifiedStiffness_N_m':row['identifiedLowerStiffness_N_m'],
                     'identifiedDamping_Ns_m':row['identifiedLowerDamping_Ns_m'],
                     'sourceTrace':path.name,'sourceTraceSHA256':row['nativeTraceSHA256']})
        if row['seed']==29:
            r=loadmat(path,simplify_cells=True,variable_names=['r'])['r']
            t=np.asarray(r['time']); n=np.asarray(r['normalContact']['data']); e=n[:,0]-50
            assert t.size==400001 and np.isclose(np.max(np.abs(e)),row['peakNormalForceError_N'],rtol=0,atol=1e-12)
            keep=(t>=1.999)&(t<=2.008); idx=np.argmax(np.abs(e))
            curves.append({'series':series,'time_ms':((t[keep]-2)*1000).tolist(),
                           'error_N':e[keep].tolist(),'peakTime_s':float(t[idx]),
                           'peakSignedError_N':float(e[idx]),'nativeSamples':int(t.size)})
            guard=row.get('staticCalibrationGuard')
            if guard:
                beforePath=Path(reports[0]['rows'][1]['nativeTraceFile'])
                before=loadmat(beforePath,simplify_cells=True,variable_names=['r'])['r']['parameters']
                after=r['parameters']
                for path in guard['primitiveAllowedPaths']:
                    def get(p):
                        for part in path.split('.'):p=p[part]
                        return float(p)
                    primitiveChanges.append({'path':path,'originalValue':get(before),'halvedValue':get(after)})
                assert len(primitiveChanges)==16
                del before
            del r

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'svg.fonttype':'none',
                     'axes.spines.top':False,'axes.spines.right':False})
fig,(summary,entry)=plt.subplots(1,2,figsize=(13,6.3),gridspec_kw={'width_ratios':[1,1.25]})
fig.subplots_adjust(left=.075,right=.97,bottom=.23,top=.74,wspace=.32)
fig.suptitle('Static calibration in the tested mechanical corner',
             x=.055,y=.957,ha='left',fontsize=16,fontweight='bold')
fig.text(.055,.894,'Same physical plant and identified inverse: k = 23.6952 N/mm, c = 48 N·s/m; S = 2 kg; 50 N; 90°; μ = 0.3.',fontsize=10)
fig.text(.055,.854,'Only 16 static residuals are halved. Noise, acquisition/timing, robot residuals and PI/current tuning remain fixed.',fontsize=10)
colors={'original_static_budget':'#b16e21','halved_static_residuals':'#2b6e97'}
labels={'original_static_budget':'Original static residuals','halved_static_residuals':'Static residuals halved'}
seeds=[7,29,1729];x=np.arange(3);width=.35
for j,series in enumerate(colors):
    values=[next(r['peakNormalForceError_N'] for r in rows if r['series']==series and r['seed']==seed) for seed in seeds]
    bars=summary.bar(x+(j-.5)*width,values,width,color=colors[series],label=labels[series],
                     hatch='/' if series=='original_static_budget' else None,
                     edgecolor='#794c1b' if series=='original_static_budget' else colors[series],linewidth=.5)
    for bar,v in zip(bars,values):
        summary.text(bar.get_x()+bar.get_width()/2,v-.025,f'{v:.4f}',ha='center',va='center',color='white',fontsize=9)
summary.axhline(.5,color='#333333',linewidth=1.2,linestyle='--')
summary.text(2.56,.507,'0.5 N limit',ha='right',fontsize=9)
summary.set_ylim(0,.55);summary.set_xlim(-.65,2.65)
summary.set_xticks(x,[str(s) for s in seeds]);summary.set_xlabel('Noise seed')
summary.set_ylabel('Maximum absolute normal-force error (N)')
summary.set_title('Every sample of each 4 s recording',loc='left',fontsize=11,pad=12)
summary.grid(axis='y',alpha=.18);summary.set_axisbelow(True)
for curve in curves:
    series=curve['series'];style='--' if series=='original_static_budget' else '-'
    entry.plot(curve['time_ms'],curve['error_N'],color=colors[series],linestyle=style,linewidth=1.6,label=labels[series])
entry.axhline(.5,color='#333333',linewidth=1.2,linestyle='--');entry.axhline(-.5,color='#333333',linewidth=1.2,linestyle='--')
entry.axvline(0,color='#777777',linewidth=.8);entry.set_xlim(-1,8);entry.set_ylim(-.55,.55)
entry.set_xlabel('Time after bump entry (ms)');entry.set_ylabel('True normal force − 50 N (N)')
entry.set_title('Seed 29: full native samples at bump entry',loc='left',fontsize=11,pad=12)
entry.grid(alpha=.18)
handles,legend=summary.get_legend_handles_labels()
fig.legend(handles,legend,loc='upper center',bbox_to_anchor=(.51,.853),frameon=False,ncol=2,fontsize=10)
fig.text(.055,.133,'Worst recorded margin improves from 0.0470 N to 0.1231 N across these three paired seeds.',fontweight='bold',fontsize=11)
fig.text(.055,.089,'All six force/packing records pass. Scope: three paired seeds at one heavy combined corner; the nominal 3,500-case design stays fixed.',fontsize=9)
fig.text(.055,.045,'Evidence: 400,001 native samples per full recording. Both sets use known actual k/c; the longer nominal-mechanics runs are a separate cohort.',fontsize=9)
stem=ROOT/'inertial_mechanics_calibrated_static_mixed_comparison_20260930'
for ext in ['svg','png']:
    fig.savefig(stem.with_suffix('.'+ext),facecolor='white',dpi=150)
plt.close(fig)
chart={'scope':'Three new4s static-calibration cases paired with three retained known-actual k/c mixed histories at one heavy mechanical corner. Only16 static residuals and derived identification aliases/coefficient dependencies change.',
       'displayedNativeHistories':6,'newNativeCohortExecutions':3,'reusedBaselineNativeHistories':3,'additionalForceExecutionsForFigure':0,'rows':rows,'entryHistories':curves,
       'recordedNativeSourceModelSHA256':reports[0]['sourceModelSHA256'],
       'baseV8NativeModelSHA256':reports[0]['baseV8NativeModelSHA256'],
       'sources':[{'file':p.name,'sha256':sha(p)} for p in FILES],
       'primitiveStaticChanges':primitiveChanges,'staticResidualCount':16}
chart_file=ROOT/'inertial_mechanics_calibrated_static_mixed_comparison_20260930_chart_data.json';chart_file.write_text(json.dumps(chart,indent=2)+'\n')
manifest={'script':Path(__file__).name,'scriptSHA256':sha(Path(__file__)),
          'chartData':chart_file.name,'chartDataSHA256':sha(chart_file),
          'displayedNativeHistories':6,'newNativeCohortExecutions':3,'reusedBaselineNativeHistories':3,'additionalForceExecutionsForFigure':0,
          'metric':'max(abs(true normal force - 50 N)) over all 400001 native samples, including startup',
          'sources':chart['sources'],'figures':[{'file':stem.with_suffix('.'+ext).name,'sha256':sha(stem.with_suffix('.'+ext))} for ext in ['svg','png']]}
(ROOT/'inertial_mechanics_calibrated_static_mixed_comparison_20260930_figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'figures':manifest['figures'],'entryPeaks':[(c['series'],c['peakTime_s'],c['peakSignedError_N']) for c in curves]}))
