#!/usr/bin/env python3
"""Compare identical actual winding tolerances with two native current drives."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
folder=Path(__file__).resolve().parent
cal=json.loads((folder/'actuator_qualification_final_winding.json').read_text())
un=json.loads((folder/'actuator_qualification_uncalibrated_driver.json').read_text())
assert cal['status']=='complete' and un['status']=='complete' and un['simulations']==32
current=[3.5,4]
calibrated=[];uncalibrated=[]
for c in current:
 a=[r for r in cal['rows'] if r['Ilimit_A']==c and r['Vlimit_V']==24 and r['scenario']=='joint_Kf_minus10_R_L_plus30_tau_plus25']
 b=[r for r in un['rows'] if r['Ilimit_A']==c]
 assert len(a)==len(b)==16
 assert all(r['simulated'] for r in a+b)
 assert {(r['force_N'],r['angle_deg'],r['seed']) for r in a}=={(r['force_N'],r['angle_deg'],r['seed']) for r in b}
 calibrated.append(max(r['peakError_N'] for r in a));uncalibrated.append(max(r['peakError_N'] for r in b))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12})
fig,ax=plt.subplots(figsize=(10.5,6.6))
fig.subplots_adjust(left=.1,right=.97,top=.78,bottom=.29)
x=np.arange(2);width=.33
for pos,value,color,label in [(x-width/2,calibrated,'#0072b2','Calibrated current drive'),(x+width/2,uncalibrated,'#d55e00','Nominal feedforward, actual hot winding')]:
 bars=ax.bar(pos,value,width,color=color,label=label)
 ax.bar_label(bars,[f'{v:.3f} N' for v in value],padding=5,fontweight='bold')
ax.axhline(.5,color='#334155',linestyle='--',label='0.5 N requirement')
ax.set_ylim(0,1.43);ax.set_xticks(x,['3.5 A /24 V','4 A /24 V'])
ax.set_ylabel('Worst true contact-force error (N)');ax.set_xlabel('Current command limit and supply voltage')
ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
ax.legend(loc='upper right',fontsize=10)
fig.suptitle('A fast current loop also needs accurate DC current regulation',fontsize=17,fontweight='bold',y=.96)
fig.text(.5,.89,'Both cases: Kf −10%, R/L +30%, actual current response 0.5 ms; outer observer remains nominal',ha='center',fontsize=11)
fig.text(.10,.19,'Each bar: 16 native simulations, 5/50 N × 0/90° × four seeds; 0.02 N force noise and unknown +0.2 ms delay.',fontsize=10.5)
fig.text(.10,.145,'Calibrated drive cancels R·i and back EMF. Uncalibrated proportional drive has 0.846 current-command DC gain.',fontsize=10.5)
fig.text(.10,.100,'These failures demonstrate a current-drive requirement; they do not rule out a well-regulated physical driver.',fontsize=10.5)
fig.text(.10,.055,'Direct contact-force reference only. The physical B/S load-cell controller is assessed separately.',fontsize=10.5)
for ext in ['png','svg']:fig.savefig(folder/f'actuator_qualification_uncalibrated_driver_comparison.{ext}',dpi=170,facecolor='white')
print({'calibrated':calibrated,'uncalibrated':uncalibrated})
