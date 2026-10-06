"""Plot the retained current-history / robot-gate tradeoff as vector evidence."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parents[2]/'validation/rl_budget3h_20261005'
OUT=BASE/'reward_search_inverse_gate_figure';OUT.mkdir(exist_ok=True)
sources=[
 ('Partial inverse (0.3)', '#146b8e',BASE/'reward_search_inverse_followup_8206/inverse_0p3/sustained_gate_replay'),
 ('Full inverse (1.0)', '#b44339',BASE/'reward_search_inverse_selection_8206/failed_native')]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'svg.fonttype':'none',
 'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#a6afb5',
 'axes.labelcolor':'#23343e','text.color':'#23343e','xtick.color':'#42545f','ytick.color':'#42545f'})
fig,axes=plt.subplots(4,1,figsize=(11,9),sharex=True,gridspec_kw={'height_ratios':[2,1.15,1.5,2.1]})
fig.subplots_adjust(left=.105,right=.975,top=.87,bottom=.20,hspace=.24)
fig.suptitle('Force stability must permit timely robot recovery',x=.105,y=.965,ha='left',fontsize=17,weight='bold')
fig.text(.105,.925,'Same 50 N / 2 kg / 90° case, same robot mapping and stability gate; sustained 4 mm/s outward motion.',fontsize=10)
fig.text(.105,.902,'The stronger current-history inverse raises force variation, so new robot corrections are rarely permitted.',fontsize=10)
metrics=[]
for number,(label,color,folder) in enumerate(sources):
    native=np.load(folder/'selection_sustained_02_native.npz');t=native['time'];truth=native['truth']
    actions=np.load(folder/'selection_sustained_02_actions.npz')['history'];ta=actions[:,0]
    # Min/max envelopes preserve every native force sample within each displaybin.
    force=truth[:,0]-50.;bin_index=np.minimum((t/.002).astype(int),int(t[-1]/.002))
    starts=np.r_[0,np.flatnonzero(np.diff(bin_index))+1];minimum=np.minimum.reduceat(force,starts);maximum=np.maximum.reduceat(force,starts)
    axes[0].fill_between(t[starts],minimum,maximum,color=color,alpha=.40,label=label)
    ready=actions[:,3+16]>.5;edges=np.r_[0,np.flatnonzero(np.diff(ready))+1,len(ready)]
    ypos=1-number
    axes[1].broken_barh([(0,float(ta[-1]))],(ypos-.25,.5),facecolors='#e8ecef')
    intervals=[(float(ta[a]),float(ta[min(b,len(ta)-1)]-ta[a])) for a,b in zip(edges[:-1],edges[1:]) if ready[a]]
    axes[1].broken_barh(intervals,(ypos-.25,.5),facecolors=color)
    stride=max(1,len(actions)//3000)
    axes[2].plot(ta[::stride],actions[::stride,3+15]*5.,color=color,lw=1.3,label=label)
    stride=max(1,len(t)//4000)
    axes[3].plot(t[::stride],(truth[::stride,2]-.0125)*1000,color=color,lw=1.8,label=label)
    shared=(t>=.1)&(t<=1.05)
    metrics.append({'label':label,'forcePeak_N':float(abs(force).max()),'endTime_s':float(t[-1]),
       'commonComparisonWindow_s':[.1,1.05],
       'commonWindowForceErrorRms_N':float(np.sqrt(np.mean(force[shared]**2))),
       'commonWindowForceErrorStd_N':float(np.std(force[shared])),
       'finalTravelError_mm':float((truth[-1,2]-.0125)*1000),'readyControllerSamples':int(ready.sum()),
       'nativeHistory':str(folder/'selection_sustained_02_native.npz'),'actions':str(folder/'selection_sustained_02_actions.npz')})
axes[0].axhspan(-.5,.5,color='#eef4f6',zorder=-2);axes[0].axhline(.5,color='#7f8d95',ls='--',lw=.8);axes[0].axhline(-.5,color='#7f8d95',ls='--',lw=.8)
axes[0].set(ylabel='Normal force error (N)',ylim=(-.55,.55));axes[0].legend(loc='lower right',frameon=False,ncol=2,fontsize=9)
axes[1].set(ylabel='Gate ready',yticks=[0,1],yticklabels=['Full','Partial'],ylim=(-.5,1.5))
axes[2].set(ylabel='Issued correction\n(mm/s)');axes[2].axhline(-4,color='#7f8d95',ls='--',lw=.8)
axes[2].text(3.97,-3.9,'Correction needed to cancel outward drift',ha='right',va='bottom',fontsize=8,color='#6a7780')
axes[3].axhspan(-7.5,7.5,color='#dcecf1',alpha=.65);axes[3].axhline(12.5,color='#b44339',ls='--',lw=1.)
axes[3].axhline(7.5,color='#7f8d95',ls=':',lw=.9);axes[3].set(ylabel='Stroke from center\n(mm)',xlabel='Time (s)',ylim=(4.5,13.2),xlim=(0,4.))
axes[3].annotate('Travel stop at 1.093 s',xy=(1.0929,12.5),xytext=(1.75,11.7),color='#b44339',fontsize=10,arrowprops={'arrowstyle':'->','color':'#b44339'})
axes[3].text(3.96,7.35,'Accepted band: ±7.5 mm',ha='right',va='top',fontsize=9,color='#146b8e')
for ax in axes:ax.grid(axis='x',alpha=.18)
fig.text(.105,.065,'Both force traces remain within ±0.5 N. Full inverse: 0.274 N peak, but only 6 eligible robot command decisions before the stop.\nPartial inverse: 0.294 N peak; all 4 s complete and final stroke is 6.94 mm from center. Gate color indicates ready; gray indicates closed.\nThe full inverse stops before the 2.592 s sharp bump and fails the complete-episode requirement.',fontsize=9,linespacing=1.5)
fig.text(.105,.032,'Native scenario 141, sensor seed 141002. Display uses 2 ms min/max force envelopes; metrics use every retained physical sample.',fontsize=8,color='#647580')
fig.savefig(OUT/'force_gate_travel_tradeoff.svg');fig.savefig(OUT/'force_gate_travel_tradeoff.png',dpi=130)
(OUT/'data_sources.json').write_text(json.dumps({'scope':'Matched development case, not final holdout or hardware validation.','metrics':metrics},indent=2)+'\n')
print(json.dumps(metrics,indent=2))
