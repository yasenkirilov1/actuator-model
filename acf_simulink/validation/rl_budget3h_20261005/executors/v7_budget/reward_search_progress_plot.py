"""Plot common development selection, never rotating-batch training objectives."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE=Path(__file__).resolve().parents[2]/'validation/rl_budget3h_20261005'
OUT=BASE/'reward_search_progress_figure'
RUNS=[('reward_search_sustained_8204','Joint current + robot refinement','#b46b34'),
      ('reward_search_inverse_8207','Partial inverse, 37 learned directions','#147d87'),
      ('reward_search_inverse_seed_8209','700 Hz inverse, second search seed','#6254a3')]

def main():
    OUT.mkdir(exist_ok=True)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'svg.fonttype':'none',
      'axes.spines.top':False,'axes.spines.right':False,'axes.edgecolor':'#a6afb5',
      'axes.labelcolor':'#23343e','text.color':'#23343e','xtick.color':'#42545f','ytick.color':'#42545f'})
    fig,axes=plt.subplots(1,2,figsize=(11.6,5.6))
    fig.subplots_adjust(left=.08,right=.98,top=.74,bottom=.25,wspace=.27)
    fig.suptitle('Reward search improves the complete-episode control tradeoff',
      x=.08,y=.965,ha='left',fontsize=16,weight='bold')
    fig.text(.08,.905,'Every point reuses the same 28 development cases; all accepted policies shown pass the force and physical-stop checks.',fontsize=9.5)
    sources=[]
    for name,label,color in RUNS:
        folder=BASE/name;selection_file=folder/'selection.json'
        selection=json.loads(selection_file.read_text())
        generations=[json.loads(x) for x in (folder/'generations.jsonl').read_text().splitlines()]
        episodes={0:0};total=0
        for generation in generations:
            total+=sum(map(len,generation['candidateResults']))
            episodes[generation['generation']]=total
        best=None;points=[]
        for row in selection:
            assert row['recordings']==28
            if best is None or row['rank']<best['rank']:best=row
            points.append({'generation':row['generation'],
              'trainingEpisodes':episodes[row['generation']],
              'selectedWorstForce_N':best['worstNormalForceError_N'],
              'selectedScore':best['jointScore'],'selectedPhysicalFailures':best['physicalFailures'],
              'selectedForcePasses':best['passed']})
        if points[-1]['trainingEpisodes']<total:points.append({**points[-1],'trainingEpisodes':total})
        x=[p['trainingEpisodes'] for p in points]
        for ax,key in zip(axes,['selectedWorstForce_N','selectedScore']):
            ax.step(x,[p[key] for p in points],where='post',color=color,lw=2,label=label)
            ax.plot(x,[p[key] for p in points],color=color,marker='o',ms=3,lw=0)
        sources.append({'run':name,'label':label,'source':str(selection_file),
          'sha256':hashlib.sha256(selection_file.read_bytes()).hexdigest(),'points':points})
    axes[0].axhline(.5,color='#b44339',ls='--',lw=1)
    axes[0].text(0,.503,'Required maximum: 0.5 N',color='#9c3d36',fontsize=9,va='bottom')
    axes[0].set(ylabel='Selected worst normal-force error (N)',ylim=(.35,.52))
    axes[1].set(ylabel='Selected force + stroke-band score',ylim=(.72,1.02))
    for ax in axes:
        ax.set_xlabel('Training episode evaluations in this refinement')
        ax.grid(alpha=.18)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='upper left',bbox_to_anchor=(.072,.87),ncol=1,
      frameon=False,fontsize=9)
    fig.text(.08,.060,'Curves show the best policy under the common joint ranking, not the minimum force error alone. Each lane has a different warm start.\nSelection and audit episodes are additional; failed training proposals remain in the execution ledger. This figure does not use final test cases.',
      fontsize=8.5,linespacing=1.5,color='#526570')
    fig.savefig(OUT/'development_convergence.svg')
    fig.savefig(OUT/'development_convergence.png',dpi=140)
    (OUT/'data_sources.json').write_text(json.dumps({'generatedUtc':datetime.now(timezone.utc).isoformat(),
      'scope':'Common 28-case development selection only; separate warm starts, no final-test outcomes.',
      'sources':sources},indent=2)+'\n')
    print(json.dumps({'figure':str(OUT/'development_convergence.svg'),
      'lanes':[{'run':s['run'],'last':s['points'][-1]} for s in sources]},indent=2))

if __name__=='__main__':main()
