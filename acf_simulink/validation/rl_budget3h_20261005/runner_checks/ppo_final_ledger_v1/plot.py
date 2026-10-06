"""Render selection curves from the audited final PPO ledger."""
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
ledger=json.loads((HERE/'ledger.json').read_text())
runs={row['name']:row for row in ledger['attempts']}
lanes=[
    ('ppo_band_8401','8401 · 0.45 N threshold / 20 ms trace','#205E86','o'),
    ('ppo_band_8402','8402 · 0.45 N threshold / 10 ms trace','#AD6B13','s'),
    ('ppo_tail_8403','8403 · 0.30 N threshold / 20 ms trace','#945572','^'),
    ('ppo_peak_8405','8405 · running-maximum cost / 5 ms trace','#536E3E','D'),
]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10.5,
    'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#9AA4AD',
    'axes.labelcolor':'#273746','text.color':'#273746','xtick.color':'#52616C',
    'ytick.color':'#52616C','svg.fonttype':'none'})
fig,axes=plt.subplots(1,3,figsize=(13.8,5.9))
metrics=[('worstNormalForceError_N','Worst force error'),
         ('meanRmsNormalForceError_N','Mean RMS force error'),
         ('jointScore','Joint force / travel score')]
for ax,(key,title) in zip(axes,metrics):
    ax.axhline(0,color='#354753',lw=1,ls='--')
    ax.set_title(title,loc='left',fontsize=12,fontweight='bold',pad=10)
    ax.grid(axis='y',color='#E6EBEF',lw=.8);ax.set_axisbelow(True)
    for name,label,color,marker in lanes:
        points=runs[name]['selectionEvaluations']
        x=[int(p['phase'].split('_')[-1]) for p in points]
        y=[100*(p[key]/points[0][key]-1) for p in points]
        ax.plot(x,y,label=label,color=color,marker=marker,ms=4.2,lw=1.7)
    ax.set_xlabel('Completed PPO updates')
    ax.set_ylabel('Change from own warm start (%)')
    ax.set_xlim(-6,309);ax.set_xticks([0,100,200,300])
fig.suptitle('PPO refinements of the band-aware controller',x=.061,y=.97,ha='left',fontsize=17,fontweight='bold')
fig.text(.061,.906,'Lower is better. Every lane kept its starting checkpoint under the shared selection score.',fontsize=11.5)
handles,labels=axes[0].get_legend_handles_labels()
fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.049,.862),ncol=2,frameon=False,fontsize=10.5,handlelength=2.5,columnspacing=2.0)
fig.text(.061,.083,'8401/8402 use 24 reused selection cases; 8403/8405 use 28. These are finite simulated selection results.',fontsize=10)
fig.text(.061,.043,'Initial policies, learning rates, rewards and case banks differ. Curves compare each lane with its own start, not isolated causal effects.',fontsize=9.5)
fig.subplots_adjust(left=.065,right=.98,top=.67,bottom=.22,wspace=.34)
fig.savefig(HERE/'ppo_final_selection_change.svg')
fig.savefig(HERE/'ppo_final_selection_change.png',dpi=155)
plt.close(fig)
