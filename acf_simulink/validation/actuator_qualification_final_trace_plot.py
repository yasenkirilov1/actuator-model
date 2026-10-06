#!/usr/bin/env python3
"""Visualize complete logged direct-force reference waveforms."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
folder=Path(__file__).resolve().parent
meta=json.loads((folder/'actuator_qualification_final_trace_metadata.json').read_text())
records=[]
for item in meta['traces']:
 d=np.genfromtxt(item['csv'],names=True,delimiter=',')
 e=d['Fc_N']-d['requestedForce_N']
 assert abs(np.max(np.abs(e))-item['peakError_N'])<1e-9
 assert abs(np.max(np.abs(d['current_A']))-item['peakCurrent_A'])<1e-9
 records.append((item,d,e))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10.5})
fig,axs=plt.subplots(2,2,figsize=(12.5,8.1))
fig.subplots_adjust(left=.07,right=.98,top=.80,bottom=.18,hspace=.42,wspace=.22)
colors=['#0072b2','#e69f00','#cc79a7']
labels=['Nominal winding','Unknown Kf −10%','Joint Kf −10%, R/L +30%, response +25%']
for (item,d,e),color,label in zip(records,colors,labels):
 t=d['time_s'];zoom=(t>=1.97)&(t<=2.18)
 axs[0,0].plot(t,e,color=color,lw=.65,label=label)
 axs[0,1].plot(t[zoom],e[zoom],color=color,lw=.75)
 axs[1,0].plot(t[zoom],d['current_A'][zoom],color=color,lw=.8)
 axs[1,1].plot(t[zoom],d['voltage_V'][zoom],color=color,lw=.7)
for ax in axs[0]:
 ax.axhspan(-.5,.5,color='#ecf7ed',zorder=-10)
 ax.axhline(.5,color='#334155',linestyle='--',lw=1);ax.axhline(-.5,color='#334155',linestyle='--',lw=1)
 ax.set_ylim(-.6,.6);ax.set_ylabel('True contact force error (N)')
axs[0,0].set_title('Whole evaluated run: 0–4 s',fontweight='bold');axs[0,0].set_xlim(0,4)
axs[0,1].set_title('Sharp bump: force response',fontweight='bold')
axs[1,0].set_title('Current during the bump',fontweight='bold');axs[1,0].set_ylabel('Actual current (A)')
axs[1,0].axhline(3.5,color='#334155',linestyle='--',lw=1,label='3.5 A limit')
axs[1,1].set_title('Applied voltage during the bump',fontweight='bold');axs[1,1].set_ylabel('Applied voltage (V)')
axs[1,1].axhline(24,color='#334155',linestyle='--',lw=1,label='24 V bus')
for ax in axs.flat:
 ax.set_xlabel('Time (s)');ax.grid(alpha=.2);ax.set_axisbelow(True)
for ax in (axs[0,1],axs[1,0],axs[1,1]):ax.set_xlim(1.97,2.18)
fig.suptitle('Direct contact-force reference: simultaneous winding changes',fontsize=17,fontweight='bold',y=.97)
fig.text(.5,.917,'50 N /90°; 1 mm ×10 mm bump at 100 mm/s; 2 kg tool and holder; seed 1729',ha='center',fontsize=12)
fig.legend(*axs[0,0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.89),ncol=3,frameon=False,fontsize=10)
fig.text(.07,.115,'3.5 A /24 V; 0.02 N RMS force noise; unknown extra 0.2 ms delay; noisy magnet measurement for control and recentering.',fontsize=10)
fig.text(.07,.079,'Outer observer remains nominal. Inner drive is calibrated to actual R/L/back EMF; thermal and current-driver implementation are separate.',fontsize=10)
fig.text(.07,.043,'This near-contact force feedback is a conditional reference. The physical load cell between B/S is assessed with its own controller.',fontsize=10)
for ext in ['png','svg']:fig.savefig(folder/f'actuator_qualification_final_trace_reference.{ext}',dpi=170,facecolor='white')
print([(r[0]['label'],r[0]['peakError_N']) for r in records])
