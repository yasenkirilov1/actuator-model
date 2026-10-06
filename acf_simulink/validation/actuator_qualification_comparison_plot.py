#!/usr/bin/env python3
"""Compare saved near-contact force references under their stated architectures."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
folder=Path(__file__).resolve().parent
data={name:json.loads((folder/f'actuator_qualification_{name}.json').read_text()) for name in ['coil','leadscrew','leadscrew_refined','leadscrew_high_gain']}
assert data['leadscrew_high_gain']['status']=='complete'
def selected(study,group,mechanics=None):
 return [r for r in data[study]['rows'] if r['group']==group and (mechanics is None or r['mechanics']==mechanics)]
sets=[selected('leadscrew','lead_corner','default_PI'),
 selected('leadscrew_high_gain','lead_high_gain_selected'),
 selected('leadscrew_refined','ideal_refined_corner'),
 selected('coil','coil_corner','strong_compliant')]
labels=['Default screw\nPI','Retuned screw\nPI, lower inertia','Ideal position\nreference','Voice coil\nobserver/LQI']
clean=[];stress=[]
for rows in sets:
 assert rows and all(r['simulated'] for r in rows)
 a=[r['peakError_N'] for r in rows if not r['stress']];b=[r['peakError_N'] for r in rows if r['stress']]
 assert len(a)==4 and len(b) in (4,8)
 clean.append(max(a));stress.append(max(b))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11})
fig,ax=plt.subplots(figsize=(11.4,7.1));fig.subplots_adjust(left=.085,right=.975,top=.77,bottom=.30)
x=np.arange(4);width=.32
for pos,vals,color,label in [(x-width/2,clean,'#0072b2','No force noise'),(x+width/2,stress,'#e69f00','Force noise + unknown extra delay')]:
 bars=ax.bar(pos,vals,width,label=label,color=color)
 for b,v in zip(bars,vals):ax.text(b.get_x()+b.get_width()/2,v*1.11,f'{v:.3f}',ha='center',va='bottom',fontweight='bold',fontsize=10.5)
ax.axhline(.5,color='#334155',linestyle='--',label='0.5 N target')
ax.set_yscale('log');ax.set_ylim(.08,10);ax.set_yticks([.1,.2,.5,1,2,5,10],['0.1','0.2','0.5','1','2','5','10']);ax.minorticks_off()
ax.set_xticks(x,labels);ax.set_ylabel('Worst true contact-force error across four corners (N)')
ax.grid(axis='y',alpha=.23);ax.set_axisbelow(True)
fig.suptitle('Sharp-bump comparison with direct contact-force feedback',fontsize=17,fontweight='bold',y=.97)
fig.text(.5,.90,'Common compliant mechanics: k₁=k₂=19,746 N/m; c₁=30, c₂=5 N·s/m; B+S=2 kg',ha='center',fontsize=11.5)
fig.legend(*ax.get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.5,.86),ncol=3,frameon=False,fontsize=10)
fig.text(.085,.205,'1 mm ×10 mm bump at 100 mm/s; 5/50 N × 0/90°. Stress: 0.02 N RMS force noise and unknown +0.2 ms delay.',fontsize=10.3)
fig.text(.085,.160,'Screw PI: default 5 mm pitch, 3 A; retuned 10 mm, J=5×10⁻⁶ kg·m², 600 rad/s speed loop, 5 A /36 V.',fontsize=10.3)
fig.text(.085,.115,'Voice coil: 3 A /24 V and magnet sensing; ideal position source has finite motion limits, without an electrical/thermal actuator.',fontsize=10.3)
fig.text(.085,.070,'Default screw stress uses seed 7; other stress bars summarize seeds 7/29. Slow recentering uses ideal travel in this older reference.',fontsize=10.3)
fig.text(.085,.025,'These implemented controllers do not qualify the actual B/S sensor, or prove that every screw actuator/controller fails.',fontsize=10.3)
for ext in ['png','svg']:fig.savefig(folder/f'actuator_qualification_control_comparison.{ext}',dpi=170,facecolor='white')
print({'clean':clean,'stress':stress})
