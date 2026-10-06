from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
v=Path('/home/tommie/Projects/actuator-model/acf_simulink/validation')
d=json.loads((v/'normal_voltage_20260930_final_sensor_subset.json').read_text())
s=json.loads((v/'normal_voltage_20260930_final_sensor_summary.json').read_text())
rows=d['rows']; blue='#245b8c'; orange='#a86620'; gray='#52575c'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titlesize':12,'axes.labelsize':10,'axes.spines.top':False,'axes.spines.right':False,'figure.facecolor':'white','axes.facecolor':'white','svg.fonttype':'none'})
fig,axes=plt.subplots(1,3,figsize=(13.5,4.9)); fig.subplots_adjust(left=.065,right=.985,bottom=.25,top=.79,wspace=.34)
for ax in axes:
 ax.grid(axis='y',color='#e1e4e7',linewidth=.7,zorder=0); ax.set_axisbelow(True)
a=axes[0]
for seed,c,marker in [(7,blue,'o'),(1729,orange,'s')]:
 rr=[r for r in rows if r['family']=='nominal_voltage' and r['simulationMode']=='accelerator' and r['seed']==seed]
 a.scatter([r['Vlimit_V'] for r in rr],[r['peakNormalError_N'] for r in rr],s=46,color=c,marker=marker,label=f'Seed {seed}',zorder=3)
a.axhline(.5,color=gray,linestyle='--',linewidth=1);a.text(47,.487,'0.5 N limit',ha='right',va='top',fontsize=9,color=gray)
a.set(xlim=(20,52),ylim=(0,.53),xticks=[24,48],xlabel='Available motor voltage (V)',ylabel='Peak absolute normal-force error (N)',title='Generic coil: 24 versus 48 V')
a.legend(frameon=False,loc='lower right',fontsize=9)
a.text(.02,.84,'All 72 histories are identical\nfor each paired noise seed.',transform=a.transAxes,ha='left',va='top',fontsize=9)
b=axes[1]
voltage_positions={volts:index for index,volts in enumerate([19,20,31,32,36])}
for seed,c,marker in [(7,blue,'o'),(1729,orange,'s')]:
 rr=[r for r in rows if r['coilLabel'].startswith('real_') and r['actualR_ohm']>r['knownR_ohm'] and r['seed']==seed]
 if not rr:
  rr=[r for r in rows if r['family'].startswith('real_') and r['actualR_ohm']>r['knownR_ohm'] and r['seed']==seed]
 b.scatter([voltage_positions[r['Vlimit_V']]+(-.07 if seed==7 else .07) for r in rr],[r['peakNormalError_N'] for r in rr],s=46,color=c,marker=marker,zorder=3)
b.axhline(.5,color=gray,linestyle='--',linewidth=1); b.text(4.4,.65,'0.5 N limit',ha='right',va='bottom',fontsize=9,color=gray)
b.set(xlim=(-.55,4.6),ylim=(0,6.8),xticks=list(voltage_positions.values()),xticklabels=[str(x) for x in voltage_positions],xlabel='Available motor voltage (V)',ylabel='Peak absolute normal-force error (N)',title='Real coil with identified hot copper')
b.text(.025,.97,'31 V and 36 V: identical histories\n19–20 V: voltage clipping, failed force',transform=b.transAxes,ha='left',va='top',fontsize=9)
c=axes[2]; x=np.arange(3); w=.34
for offset,key,label,color,hatch in [(-w/2,'coldLinearStage_W','Cold copper',blue,''),(w/2,'hotLinearStage_W','Hot copper',orange,'//')]:
 vals=[r[key] for r in s['realHoldHeatingEstimate']]
 bars=c.bar(x+offset,vals,w,label=label,color=color,hatch=hatch,edgecolor='white',linewidth=.7)
 for bar,value in zip(bars,vals): c.text(bar.get_x()+bar.get_width()/2,value+1.4,f'{value:.1f}',ha='center',fontsize=9)
c.set(ylim=(0,82),xticks=x,xticklabels=['24','36','48'],xlabel='Physical supply voltage (V)',ylabel='Estimated linear-amplifier heat (W)',title='Steady 50 N horizontal holding load')
c.legend(frameon=False,loc='upper left',fontsize=9)
fig.suptitle('Electrical headroom of the final normal-force controller',x=.065,ha='left',fontsize=15,y=.97)
fig.text(.065,.876,'Native comparison: tool S = 2 kg, holder B = 0.6 kg, 50 N, 90° gravity orientation, μ = 0.3; current response 100 µs.',fontsize=10.5)
fig.text(.065,.12,'Force panels: 0–4 s, including startup, 1 mm / 10 mm bump at 100 mm/s, seeds 7 and 1729, 10 µs integration.\nReal coil: 24.4 N/A, 6.4 Ω cold, 8.896 Ω hot (+100°C copper-rise assumption), 4 mH. The 19/20 and 31/32 V caps\nrepresent explicit amplifier-headroom envelopes. Heat panel is arithmetic at 2.049 A, zero back EMF; coil heat 26.9 / 37.4 W.\nConstant force coefficient and hypothetical hot copper do not qualify a motor’s travel-dependent force or safe temperature.',fontsize=8.6,linespacing=1.45,va='top')
base=v/'normal_voltage_20260930_final_sensor_headroom'
fig.savefig(str(base)+'.svg',bbox_inches='tight');fig.savefig(str(base)+'.png',dpi=170,bbox_inches='tight');plt.close(fig)
print(str(base)+'.svg')
