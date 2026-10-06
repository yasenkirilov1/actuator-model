"""Summarize fixed-policy development sensitivity without touching selection."""
from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parents[2]/'validation/rl_budget3h_20261005'
OUT=BASE/'reward_search_budget_limits';OUT.mkdir(exist_ok=True)
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

sources=[BASE/'reward_search_budget_screen_8210/results.json',
 BASE/'reward_search_budget_bootstrap_8210/results.json',
 BASE/'reward_search_budget_limit_current100/results.json',
 BASE/'reward_search_budget_limit_latency50/results.json']
primary_baseline=next(r for r in read(sources[0])['comparisons'] if r['controller']=='native_baseline')
primary_learned=next(r for r in read(sources[1])['comparisons'] if r['split']=='selection' and r['name']=='history_0p5')
groups=[('Primary budget chain\n50 µs current response',[primary_baseline,primary_learned])]
for label,path in [('100 µs current response\nSame sensing chain',sources[2]),
 ('50 µs current response\nAdded known 50 µs sensing delay',sources[3])]:
    comparisons=read(path)['comparisons']
    groups.append((label,[next(r for r in comparisons if r['controller']==name)
      for name in ['native_baseline','frozen_learned_policy']]))

reference={(r['scenario'],r['seed'],r['force_N'],r['massS_kg'],r['angle_rad']) for r in primary_baseline['rows']}
summary=[]
for label,comparisons in groups:
    for controller,comparison in zip(['Native baseline','Locked recurrent policy'],comparisons):
        rows=comparison['rows']
        assert {(r['scenario'],r['seed'],r['force_N'],r['massS_kg'],r['angle_rad']) for r in rows}==reference
        assert not {r['scenario'] for r in rows}.intersection(range(64,96))
        worst=max(rows,key=lambda r:r['peakNormalForceError_N'])
        summary.append({'condition':label.replace('\n','; '),'controller':controller,
          'completeForcePasses':comparison['passed'],'cases':28,'physicalFailures':comparison['physicalFailures'],
          'worstRecordedForceError_N':comparison['worstNormalForceError_N'],
          'meanRmsForceError_N':comparison['meanRmsNormalForceError_N'],
          'worstForceCase':worst['scenario'],'worstForceCaseClipFraction':worst['voltageClipFraction'],
          'maximumClipFraction':max(r['voltageClipFraction'] for r in rows),
          'failedCases':[r for r in rows if not r['passed']]})

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'svg.fonttype':'none',
 'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#a6afb5',
 'axes.labelcolor':'#23343e','text.color':'#23343e','xtick.color':'#42545f','ytick.color':'#42545f'})
fig,ax=plt.subplots(figsize=(11,6.1));fig.subplots_adjust(left=.085,right=.975,top=.79,bottom=.27)
fig.suptitle('The budget controller is sensitive to response time',x=.085,y=.965,ha='left',fontsize=17,weight='bold')
fig.text(.085,.912,'Same 28 development cases and locked policy weights. No retraining, case exclusion or final-test outcomes.',fontsize=10)
colors=['#547486','#19838a'];width=.31
for j,(name,color) in enumerate(zip(['Native baseline','Locked recurrent policy'],colors)):
    x=np.arange(3)+(j-.5)*width
    rows=summary[j::2];bars=ax.bar(x,[r['worstRecordedForceError_N'] for r in rows],width=.27,color=color,label=name,zorder=3)
    for bar,row in zip(bars,rows):
        if row['physicalFailures']:bar.set_hatch('///');bar.set_edgecolor('#833f3a')
        note=f"{row['worstRecordedForceError_N']:.3f} N\n{row['completeForcePasses']}/28 pass"
        if row['physicalFailures']:note+=f"\n{row['physicalFailures']} travel stop"+('s' if row['physicalFailures']>1 else '')
        ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+.025,note,ha='center',va='bottom',fontsize=9,
          color='#833f3a' if row['physicalFailures'] else '#23343e',linespacing=1.25)
ax.axhline(.5,color='#b44339',ls='--',lw=1.,zorder=2)
ax.text(2.49,.508,'0.5 N limit',ha='right',va='bottom',fontsize=9,color='#a3453b')
ax.set(xticks=np.arange(3),xticklabels=[g[0] for g in groups],ylabel='Worst recorded normal-force error (N)',ylim=(0,1.42),xlim=(-.5,2.5))
ax.grid(axis='y',alpha=.18,zorder=0);ax.legend(loc='upper left',bbox_to_anchor=(0,1.14),ncol=2,frameon=False)
fig.text(.085,.115,'Pass requires completing the episode and keeping every scored force sample within ±0.5 N. Hatched bars include physical travel failures.\nThe 100 µs-current learned worst case has zero voltage clipping: more supply headroom does not address that recorded response limitation.',fontsize=9,linespacing=1.5)
fig.text(.085,.045,'The delay perturbation is known and matched in the model; actual and identified calibration residuals remain separate.\nPrimary-chain reference records are reused; the two new comparisons add 112 scored episodes and two native recovery replays.',fontsize=8.5,color='#627581',linespacing=1.4)
fig.savefig(OUT/'budget_response_limits.svg');fig.savefig(OUT/'budget_response_limits.png',dpi=140)

new=[read(p) for p in sources[2:]]
report={'status':'complete_fixed_policy_development_comparison','rows':summary,
  'policyCheckpointSha256':'ca74a1be34ff2cb801f2a094aeae2e30968f52b3aaec6aacea216ca2b11ddaef',
  'newScoredEpisodes':sum(r['episodeComparisons'] for r in new),'newActorDecisions':int(sum(r['actorDecisions'] for r in new)),
  'nativeRecoveryEpisodesExcluded':sum(r['recoveryEpisodesExcluded'] for r in new),'reusedReferenceEpisodes':56,
  'sources':[{'path':str(p),'sha256':sha(p)} for p in sources],
  'scope':'Frozen weights, development cases only. No selection or final-test records changed. All failed-case metrics retained; exact weights, seeds and native bank permit replay, with representative full physical-failure histories preserved.'}
(OUT/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
files=[Path(__file__),OUT/'comparison.json',OUT/'budget_response_limits.svg',OUT/'budget_response_limits.png']
files.extend(p for name in ['reward_search_budget_limit_current100','reward_search_budget_limit_latency50'] for p in (BASE/name).rglob('*') if p.is_file())
(OUT/'manifest.json').write_text(json.dumps({'status':'bound','files':[{'path':str(p.resolve()),'bytes':p.stat().st_size,'sha256':sha(p)} for p in files]},indent=2)+'\n')
print(json.dumps({'directory':str(OUT),'newScoredEpisodes':report['newScoredEpisodes'],'newActorDecisions':report['newActorDecisions'],'nativeRecoveryEpisodesExcluded':report['nativeRecoveryEpisodesExcluded']},indent=2))
