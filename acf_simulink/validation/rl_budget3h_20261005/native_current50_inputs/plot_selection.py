from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
F=Path('/home/tommie/Projects/actuator-model/acf_simulink/validation/rl_budget3h_20261005/native_current50_results');d=json.loads((F/'summary.json').read_text())
plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none','font.size':9,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,'axes.axisbelow':True,'figure.facecolor':'white'})
colors={'baseline':'#718096','learned':'#007f8b'};ids=d['selectionScenarios'];fig,axs=plt.subplots(2,2,figsize=(12.4,8.3),layout='constrained')
for mode in ['baseline','learned']:
 p=[r for r in d['pairs'] if r['mode']==mode]
 for tau,style in [(100,'-'),(50,'--')]:
  axs[0,0].plot(np.arange(1,29),[r[f'peak{tau}_N'] for r in p],color=colors[mode],ls=style,lw=1.4,label=f'{mode.capitalize()} {tau} µs')
 axs[0,1].plot(np.arange(1,29),[r['peakChange_N'] for r in p],color=colors[mode],marker='o',ms=3,lw=1,label=mode.capitalize())
axs[0,0].axhline(.5,color='#b44343',ls=':',lw=1.3,label='0.5 N limit');axs[0,0].set(xlabel='Development case index',ylabel='Full-record peak normal-force error (N)',title='All 28 cases pass with either response target',ylim=(0,.53));axs[0,0].legend(fontsize=8,ncol=2)
axs[0,1].axhline(0,color='#888888',ls=':',lw=1);axs[0,1].set(xlabel='Same development case index',ylabel='Peak error at 50 µs minus 100 µs (N)',title='26/28 learned peaks improve; two small regressions');axs[0,1].legend(fontsize=8);axs[0,1].text(.02,.03,'Negative values mean an improvement',transform=axs[0,1].transAxes,fontsize=8)
for ax in axs[0]:
 for cut in [16.5,24.5]:ax.axvline(cut,color='#aaaaaa',lw=.8,ls=':')
 ax.set_xticks([1,4,8,12,16,20,24,28])
x=np.arange(2)
for tau,shift,alpha in [(100,-.18,.95),(50,.18,.48)]:
 rr=[next(r for r in d['groups'] if r['mode']==mode and r['currentResponse_us']==tau) for mode in ['baseline','learned']]
 bars=axs[1,0].bar(x+shift,[r['meanRmsNormalForceError_N'] for r in rr],width=.32,color=[colors['baseline'],colors['learned']],alpha=alpha,label=f'{tau} µs')
 for bar,r in zip(bars,rr):axs[1,0].text(bar.get_x()+bar.get_width()/2,bar.get_height()+.001,f"{r['meanRmsNormalForceError_N']:.4f}",ha='center',fontsize=8)
 bars=axs[1,1].bar(x+shift,[r['voltageClipSamples']/300 for r in rr],width=.32,color=[colors['baseline'],colors['learned']],alpha=alpha,label=f'{tau} µs')
 for bar,r in zip(bars,rr):axs[1,1].text(bar.get_x()+bar.get_width()/2,bar.get_height()*1.5,f"{r['voltageClipSamples']:,}",ha='center',fontsize=8)
axs[1,0].set(xticks=x,xticklabels=['Baseline','Fixed learned policy'],ylabel='Mean of 28 case RMS errors (N)',ylim=(0,.052),title='The baseline is quieter; learned RMS improves at 50 µs');axs[1,0].legend(fontsize=8)
axs[1,1].set(xticks=x,xticklabels=['Baseline','Fixed learned policy'],ylabel='Total native time at voltage clipping (ms)',yscale='log',ylim=(.004,300),title='Faster tracking spends more time at the 32 V bound');axs[1,1].legend(fontsize=8)
fig.suptitle('Native driver-response development study · fixed ES generation 5 · 32 V usable output',fontsize=13,fontweight='bold')
fig.supxlabel('112 full recordings / 111,600,112 native points, including startup and end. 50 µs retunes the identified response observer.\nCases 1–16: original selection; 17–24: stroke-edge programs; 25–28: sustained drift. No fresh holdout or training.',fontsize=8)
fig.savefig(F/'current_response_28cases.svg');fig.savefig(F/'current_response_28cases.png',dpi=110);plt.close(fig)
