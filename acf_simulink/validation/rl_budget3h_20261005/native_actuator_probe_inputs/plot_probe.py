from pathlib import Path
import json,numpy as np,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
F=Path('/home/tommie/Projects/actuator-model/acf_simulink/validation/rl_budget3h_20261005/native_actuator_probe_results');d=json.loads((F/'summary.json').read_text())
plt.rcParams.update({'font.family':'DejaVu Sans','svg.fonttype':'none','font.size':10,'axes.spines.top':False,'axes.spines.right':False,'axes.grid':True,'grid.alpha':.18,'axes.axisbelow':True,'figure.facecolor':'white'})
colors={'baseline':'#718096','learned':'#007f8b'}
fig,axs=plt.subplots(1,2,figsize=(11.8,4.7),layout='constrained')
for mode in ['baseline','learned']:
 for tau,marker,style in [(.0001,'o','-'),(.00005,'s','--')]:
  rows=[r for r in d['rows'] if r['mode']==mode and r['currentResponse_s']==tau]
  axs[0].plot([r['usableVoltage_V'] for r in rows],[r['peakNormalForceError_N'] for r in rows],marker=marker,ls=style,color=colors[mode],lw=2,label=f"{mode.capitalize()}, {round(tau*1e6)} µs")
axs[0].axhline(.5,color='#b44343',ls=':',lw=1.5,label='0.5 N limit');axs[0].set(xlabel='Usable bipolar output (V)',ylabel='Full-record peak normal-force error (N)',ylim=(.35,.51),xticks=[32,36,48],title='Changing voltage does not lower the worst error');axs[0].legend(fontsize=8,loc='upper right')
x=np.arange(3)
for tau,color,shift in [(.0001,'#007f8b',-.18),(.00005,'#dd9851',.18)]:
 rows=[r for r in d['rows'] if r['mode']=='learned' and r['currentResponse_s']==tau];values=np.array([r['voltage_clip_samples']/300 for r in rows])
 bars=axs[1].bar(x+shift,values,width=.32,color=color,label=f'Learned, {round(tau*1e6)} µs')
 for bar,r,v in zip(bars,rows,values):axs[1].text(bar.get_x()+bar.get_width()/2,v+.15,f"{r['voltage_clip_samples']:,}",ha='center',va='bottom',fontsize=8)
axs[1].set(xticks=x,xticklabels=['32','36','48'],xlabel='Usable bipolar output (V)',ylabel='Time at voltage clipping (ms)',ylim=(0,10.5),title='A faster loop increases clipping of later requests');axs[1].legend(fontsize=8)
fig.suptitle('Actuator development probe · fixed ES generation 5 · one calibrated selection case',fontsize=13,fontweight='bold')
fig.supxlabel('Numbers above bars are native clipped samples; 3.333 µs each. 50 µs also updates the identified response model.',fontsize=8)
fig.savefig(F/'actuator_limit_peaks.svg');plt.close(fig)
fig,axs=plt.subplots(3,1,figsize=(10.9,8.5),layout='constrained')
for tau,voltage,color,style in [(.0001,32,'#007f8b','-'),(.0001,36,'#64b6be','--'),(.00005,32,'#b56c2a','-'),(.00005,36,'#e5ac6f','--')]:
 r=next(r for r in d['rows'] if r['mode']=='learned' and r['currentResponse_s']==tau and r['usableVoltage_V']==voltage);data=np.load(r['nativeHistoryFile']);t=data['time'];truth=data['truth'];label=f'{round(tau*1e6)} µs / {voltage} V'
 window=np.flatnonzero((t>=.6658)&(t<=.670));idx=window[::3];axs[0].plot((t[idx]-.666)*1000,truth[idx,0]-r['force_N'],color=color,ls=style,lw=1.4,label=label)
 peak=int(np.argmin(abs(t-r['peak_force_time_s'])));axs[0].scatter((t[peak]-.666)*1000,truth[peak,0]-r['force_N'],s=18,color=color)
 win=np.flatnonzero((t>=1.5474)&(t<=1.5481));idx=win[::2]
 axs[1].plot((t[idx]-1.5476)*1e6,truth[idx,15],color=color,ls=style,lw=1.4,label=label)
 axs[2].plot((t[idx]-1.5476)*1e6,truth[idx,14],color=color,ls=style,lw=1.4,label=label)
axs[0].axhline(-.5,color='#b44343',ls=':',lw=1);axs[0].axhline(.5,color='#b44343',ls=':',lw=1);axs[0].set(xlabel='Time from bump entry (ms)',ylabel='Normal-force error (N)',title='Bump-entry error: faster current tracking helps; extra voltage does not');axs[0].legend(ncol=4,fontsize=8)
axs[1].set(xlabel='Time from a later clipping event (µs)',ylabel='Applied voltage (V)',title='Later noise-driven requests consume voltage headroom');axs[1].axhline(32,color='#999999',ls=':',lw=.8)
axs[2].set(xlabel='Time from the same later event (µs)',ylabel='Actual winding current (A)')
fig.suptitle('Native simulated waveforms · same frozen weights, seed, mechanics and calibration',fontsize=13,fontweight='bold');fig.supxlabel('Plots show selected native points; scoring includes all 7,200,012 points, including startup.',fontsize=8)
fig.savefig(F/'actuator_limit_waveforms.svg');plt.close(fig)
