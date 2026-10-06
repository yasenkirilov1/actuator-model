from pathlib import Path
import gc
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root=Path(__file__).resolve().parent
summary=json.loads((root/'results.json').read_text())
normal=json.loads((root/'carried/results.json').read_text())['outcomes']['selected']['rows']
reset=json.loads((root/'reset/results.json').read_text())['outcomes']['selected']['rows']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':14,'svg.fonttype':'none',
    'axes.spines.top':False,'axes.spines.right':False,'axes.labelcolor':'#233e4b',
    'text.color':'#233e4b','axes.edgecolor':'#9bb2bf','xtick.color':'#526a79','ytick.color':'#526a79'})
fig,axes=plt.subplots(4,2,figsize=(15,16.8),sharex='row',facecolor='#fcfdfd')
colors=['#19779b','#db764b']
for i,(a,b,paired) in enumerate(zip(normal,reset,summary['paired'],strict=True)):
    for row,color,label in [(a,colors[0],'Carried recurrent state'),(b,colors[1],'State reset every decision')]:
        with np.load(row['nativeHistoryFile']) as data:
            time=data['time'];truth=data['truth'];stride=max(1,len(time)//5000)
            axes[i,0].plot(time[::stride],truth[::stride,0]-row['force_N'],color=color,lw=1.15,alpha=.9,label=label)
            axes[i,1].plot(time[::stride],(truth[::stride,2]-.0125)*1000,color=color,lw=1.6,label=label)
        del time,truth;gc.collect()
    axes[i,0].axhspan(-.5,.5,color='#edf7f0',zorder=-10)
    axes[i,0].axhline(.5,color='#bc514d',lw=1,ls='--');axes[i,0].axhline(-.5,color='#bc514d',lw=1,ls='--')
    axes[i,0].set_ylim(-.56,.56)
    axes[i,0].set_ylabel('Normal-force error (N)')
    axes[i,0].set_title(f"TRAIN {a['scenario']}: {a['identity']}\nFull-sample peak {a['peakNormalForceError_N']:.4f} → {b['peakNormalForceError_N']:.4f} N",loc='left',fontsize=14,pad=12)
    axes[i,1].axhspan(-7.5,7.5,color='#edf7f0',zorder=-10)
    axes[i,1].axhline(7.5,color='#7d679c',lw=1,ls='--');axes[i,1].axhline(-7.5,color='#7d679c',lw=1,ls='--')
    axes[i,1].set_ylim(-11,11)
    axes[i,1].set_ylabel('Travel from center (mm)')
    axes[i,1].set_title(f"Time inside ±7.5 mm: {100*a['travelWithinBandFraction']:.1f}% → {100*b['travelWithinBandFraction']:.1f}%\nEnd excess: {1000*a['finalTravelBandExcess_m']:.3f} → {1000*b['finalTravelBandExcess_m']:.3f} mm",loc='left',fontsize=14,pad=12)
    for ax in axes[i]:
        ax.set_facecolor('#fcfdfd');ax.grid(color='#dce6eb',lw=.6)
        ax.set_xlim(0,a['nativeLastTime_s'])
        ax.set_xlabel('Simulation time (s)')
fig.suptitle('Learned history helps robot travel in this development test',fontsize=24,x=.06,ha='left',y=.99)
fig.text(.06,.959,'Same fixed gen5 weights, sensors, observers, native plant and delay queues; only 64 learned states reset every 100 µs.',fontsize=14)
handles,labels=axes[0,0].get_legend_handles_labels()
fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.51,.937),ncol=2,frameon=False,fontsize=14)
fig.subplots_adjust(top=.865,bottom=.115,left=.075,right=.96,wspace=.24,hspace=.68)
fig.text(.06,.058,'All 8 runs passed ±0.5 N. Force-peak changes were small/mixed; recentering changed strongly in TRAIN 96 and 139.',fontsize=14)
fig.text(.06,.028,'7,800,008 native samples scored. Curves decimated for drawing. Development cases; no optimized memoryless baseline or final holdout claim.',fontsize=12)
fig.savefig(root/'memory_reliance_v2.svg',facecolor=fig.get_facecolor())
fig.savefig(root/'memory_reliance_v2.png',dpi=100,facecolor=fig.get_facecolor())
print(root/'memory_reliance_v2.svg')
