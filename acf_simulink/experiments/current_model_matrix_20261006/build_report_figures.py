"""Versioned report SVGs drawn only from the current four-bank results."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
V = ROOT/'acf_simulink/validation/current_model_matrix_20261006'
OUT = ROOT/'reports/figures'
DATA = V/'report'
BANKS = ['coil_soft','screw_soft','coil_rigid_plate','screw_rigid_plate']
LABELS = ['Coil + soft pad','Screw + soft pad','Coil + rigid plate','Screw + rigid plate']
COLORS = ['#146a9e','#bd5c20','#146a9e','#bd5c20']
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.titlesize':12,
 'axes.labelsize':11,'axes.spines.top':False,'axes.spines.right':False,
 'svg.fonttype':'none','savefig.facecolor':'#ffffff','figure.facecolor':'#ffffff',
 'axes.facecolor':'#ffffff','axes.edgecolor':'#64748b','text.color':'#172b3d',
 'axes.labelcolor':'#172b3d','xtick.color':'#172b3d','ytick.color':'#172b3d'})

def load(p): return json.loads((ROOT/p).read_text())
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def save(fig,name,question,source_paths,method):
 p=OUT/(name+'-20261007.svg')
 fig.savefig(p,bbox_inches='tight',metadata={'Date':'2026-10-07','Description':method})
 p.write_text('\n'.join(line.rstrip() for line in p.read_text().splitlines())+'\n')
 plt.close(fig)
 figures.append({'path':str(p.relative_to(ROOT)),'sha256':sha(p),'question':question,
 'sources':[{'path':str(q),'sha256':sha(ROOT/q)} for q in source_paths], 'method':method})

def style(ax,log=False):
 ax.grid(axis='y',alpha=.16); ax.set_axisbelow(True)
 if log: ax.set_yscale('log')

def limit(ax): ax.axhline(.5,color='#9b1c31',ls='--',lw=1.3,label='0.5 N limit')

def lines(ax,variants,labels,values,log=True,with_counts=False):
 for i,b in enumerate(BANKS):
  z=[values[b][v] for v in variants]
  valid=[q.get('sensor_range_failures',0)==0 for q in z]
  y=[q['worst_error_N'] if ok else np.nan for q,ok in zip(z,valid)]
  ax.plot(range(len(z)),y,color=COLORS[i],ls='-' if i<2 else '--',marker='o' if i<2 else 's',label=LABELS[i],lw=1.7)
  if with_counts:
   for j,q in enumerate(z):
    if not valid[j]:
     ax.scatter(j,.6,marker='x',color=COLORS[i],s=70)
     ax.annotate('range fail',(j,.6),xytext=(0,8 if i==0 else -16),textcoords='offset points',ha='center',fontsize=9)
    elif q.get('physical_stops',0):
     ax.scatter(j,y[j],marker='x',color=COLORS[i],s=90,zorder=5)
 ax.set_xticks(range(len(labels)),labels); style(ax,log); limit(ax)
 ax.set_ylabel('Largest force error (N)')

control_path=Path('acf_simulink/validation/current_model_matrix_20261006/control/matrix_completion.json')
acq_path=Path('acf_simulink/validation/current_model_matrix_20261006/acquisition/diagnostic_results.json')
cal_path=Path('acf_simulink/validation/current_model_matrix_20261006/report/calibration_followup.json')
contract_path=Path('acf_simulink/experiments/current_model_matrix_20261006/study_contract.json')
control=load(control_path); acq=load(acq_path); cal=load(cal_path)
selected={g['name']:g for g in control['groups'] if g['name'] in BANKS}
assert len(selected)==4 and all(g['summary']['executed_count']==54 for g in selected.values())
figures=[]

fig,axs=plt.subplots(1,2,figsize=(11,4.7),gridspec_kw={'width_ratios':[1.9,1]})
x=np.arange(4);y=[selected[b]['summary']['worst_force_error_N'] for b in BANKS]
axs[0].bar(x,y,color=COLORS,hatch=['','','//','//'],width=.62)
for j,b in enumerate(BANKS):
 z=selected[b]['summary']; axs[0].text(j,y[j]*1.22,f"{y[j]:.3f} N\n{z['force_and_eligibility_pass_count']}/54 pass",ha='center',fontsize=10)
axs[0].set_xticks(x,['Coil\nsoft pad','Screw\nsoft pad','Coil\nrigid plate','Screw\nrigid plate'])
axs[0].set_ylim(.025,22);axs[0].set_ylabel('Largest force error (N, log scale)');style(axs[0],True);limit(axs[0]);axs[0].legend(loc='upper left',frameon=False)
axs[0].set_title('Force regulation: same 77 mm footprint')
heat=[selected[b]['summary']['maximum_average_hot_copper_loss_W'] for b in BANKS[:2]]
axs[1].bar([0,1],heat,color=COLORS[:2],width=.6)
for j,h in enumerate(heat):axs[1].text(j,h+.7,f'{h:.1f} W',ha='center')
axs[1].set_xticks([0,1],['Coil\nsoft pad','Screw\nsoft pad']);axs[1].set_ylim(0,44);axs[1].set_ylabel('Maximum average winding loss (W)');style(axs[1]);axs[1].set_title('Heat during the passing 2 s tests')
fig.suptitle('What do the four fresh nominal grids tell us?',fontsize=15,y=1.02)
fig.text(.02,-.07,'Each grid: 3 masses × 3 force targets × 3 orientations × 2 seeds. Both rigid grids have 12 contact stops.\nWinding loss is modeled heat generation; it does not qualify continuous operation.',fontsize=10)
fig.tight_layout()
save(fig,'current-four-configuration-results','Which actuator and contact arrangement should we develop?', [control_path], 'All selected-bank full-record metrics, including startup. Four54-case grids; copper-loss panels only use force-eligible soft-pad grids.')

# Analytic contact geometry and its lower bound; not an experimental force curve.
t=np.linspace(.3,1.4,1101); r=(.1*(t-.8))/.01
ridge=np.where((r>0)&(r<1),.001*64*r**3*(1-r)**3,0)
left=(.1*(t-.8)-.0385)/.01;right=(.1*(t-.8)+.0385)/.01
rr=np.clip(.5,left,right); rr=np.clip(rr,0,1)
envelope=.001*64*rr**3*(1-rr)**3
fig,axs=plt.subplots(1,2,figsize=(11,4.5),gridspec_kw={'width_ratios':[1.7,1]})
axs[0].plot(t,ridge*1000,c='#728193',ls=':',lw=2,label='Surface height under centre')
axs[0].plot(t,envelope*1000,c='#146a9e',lw=2,label='Rigid plate: highest point under 77 mm')
for q in [.465,1.235]:axs[0].axvline(q,c='#9b1c31',ls='--',alpha=.7)
axs[0].annotate('Acceleration jumps here',xy=(.465,1),xytext=(.6,.5),arrowprops={'arrowstyle':'->','color':'#9b1c31'},fontsize=10)
axs[0].set_xlabel('Time (s)');axs[0].set_ylabel('Height (mm)');axs[0].set_title('A wide rigid plate follows the highest point');axs[0].legend(fontsize=9,loc='upper right',bbox_to_anchor=(1,.86),frameon=True,facecolor='white',framealpha=.95);style(axs[0])
m=np.array([.1,1.4,2]);bound=1.2*m
axs[1].bar(range(3),bound,width=.65,color='#146a9e')
for j,h in enumerate(bound):axs[1].text(j,h+.08,f'≥ {h:.2f} N',ha='center')
axs[1].set_xticks(range(3),['0.1 kg','1.4 kg','2 kg']);axs[1].set_ylim(0,2.85);axs[1].set_ylabel('Unavoidable two-sided error bound (N)');axs[1].set_title('Ideal rigidity gives a force jump');limit(axs[1]);style(axs[1]);axs[1].legend(frameon=False,loc='upper left')
fig.suptitle('Why does removing the soft pad change the problem?',fontsize=15,y=1.02)
fig.text(.02,-.06,'Geometry calculation: force jump = S mass × 2.4 m/s²; at least half the jump must miss a continuous target.\nReal interface compliance or tilt can soften this ideal limit and must be measured.',fontsize=10)
fig.tight_layout()
save(fig,'current-pad-versus-rigid-geometry','How do soft pads influence our results?', [contract_path], 'Analytic ridge/envelope and minmaxlower bound1.2*mS; theoretical geometry, not a simulated force recording. Same77mm footprint.')

fig,ax=plt.subplots(figsize=(10,4.4))
lines(ax,['baseline','bits16','bits14','bits12'],['24 bits','16 bits','14 bits','12 bits'],acq['by_bank_variant'])
ax.set_ylim(.025,12);ax.set_title('How much force error does coarser conversion add?',fontsize=15)
ax.legend(ncol=2,frameon=False,loc='upper left',bbox_to_anchor=(0,-.13))
fig.text(.02,-.18,'Three corners per configuration, fixed gains and fresh diagnostic seed. All soft-pad code-spacing tests pass 3/3.\nAll rigid-plate tests fail; these curves retain the mechanical limitation. Source noise and timing are unchanged.',fontsize=10)
fig.tight_layout()
save(fig,'current-adc-resolution','How many ADC bits do we need?', [acq_path], 'Largest full-record error across threecorners at each code spacing, same noise and timing. Not hardware ADC/ENOB proof.')

fig,axs=plt.subplots(1,2,figsize=(11,4.6))
lines(axs[0],['baseline','rate15000','rate7500'],['30 ksample/s','15 ksample/s','7.5 ksample/s'],acq['by_bank_variant'],with_counts=True)
lines(axs[1],['bandwidth500','bandwidth950','baseline','bandwidth3800'],['500 Hz','950 Hz','1,900 Hz','3,800 Hz'],acq['by_bank_variant'],with_counts=True)
for ax in axs:ax.set_ylim(.025,12);ax.tick_params(axis='x',labelsize=10)
axs[0].set_title('Sampling: light coil loses contact at 7.5k');axs[1].set_title('Filtering: light coil fails at 500/950 Hz')
handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,ncol=3,frameon=False,loc='lower center',bbox_to_anchor=(.5,-.07),fontsize=10)
fig.suptitle('What bandwidth and sampling rate survive these tests?',fontsize=15,y=1.02)
fig.text(.02,-.18,'Three corners per configuration and setting; gains unchanged. × marks a physical stop or sensor-range failure.\nRange-ineligible force peaks are omitted. Converter-rate tests change modeled transport delay; cutoff tests use matched responses.',fontsize=10)
fig.tight_layout()
save(fig,'current-acquisition-timing','How fast must the amplifier, ADC and accelerometer be?', [acq_path], 'Three-corner worst diagnostics. Range-ineligible peaks omitted and failures marked at0.6N only as categorical marker, not a force value. Physical-stop outcomes marked.')

# Mass follow-up, use eligible trajectories only; all rigid force fails remain visible.
vals={b:{} for b in BANKS}
for z in cal['summaries']:vals[z['bank']][z['variant']]=dict(z,physical_stops=0)
variants=['mass_minus0p5pct','baseline','mass_plus0p1pct','mass_plus0p25pct','mass_plus0p5pct']
fig,ax=plt.subplots(figsize=(10,4.4))
lines(ax,variants,['−0.5%','Exact','+0.1%','+0.25%','+0.5%'],vals,with_counts=True)
ax.set_ylim(.035,13);ax.set_title('How accurately must we identify downstream tool mass?',fontsize=15)
ax.set_xlabel('Error in the mass used by the controller');ax.legend(ncol=2,frameon=False,loc='upper left',bbox_to_anchor=(0,-.19))
fig.text(.02,-.25,'Adaptive follow-up: three corners per configuration, new seed and unchanged gains. +0.25% passes 3/3 soft-coil cases;\n+0.5% fails both heavy coil cases and sensor screens. × marks those failures; their ineligible continuation peaks are omitted.',fontsize=10)
fig.tight_layout()
save(fig,'current-mass-calibration','What calibration accuracy matters most?', [cal_path], 'Follow-up afterknown1%mass failure. Identifiedmass changes inertia/gravity/bodyestimation while acquisitiontare remainsnominal. No independent qualification or fullresidualbox.')

fig,axs=plt.subplots(1,2,figsize=(11,4.6))
lines(axs[0],['accel_noise0','baseline','accel_noise028','accel_noise056'],['0','0.014','0.028','0.056'],acq['by_bank_variant'])
axs[0].set_xlabel('Reference-normalized acceleration noise (m/s² RMS)');axs[0].set_title('Random noise: 4× baseline still passes soft cases')
x=np.arange(4);counts=[acq['by_bank_variant'][b]['acceleration_channel_removed']['eligible_pass'] for b in BANKS]
axs[1].bar(x,counts,color=COLORS,hatch=['','','//','//'],width=.6)
axs[1].set_xticks(x,['Coil\nsoft','Screw\nsoft','Coil\nrigid','Screw\nrigid']);axs[1].set_yticks([0,1,2,3]);axs[1].set_ylim(0,3.5);axs[1].set_ylabel('Cases passing force + ratings (of 3)');axs[1].set_title('Removing the whole acceleration channel')
for j,q in enumerate(counts):axs[1].text(j,q+.1,f'{q}/3',ha='center')
style(axs[1]);axs[0].set_ylim(.025,12)
handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,ncol=3,frameon=False,loc='lower center',bbox_to_anchor=(.5,-.07),fontsize=10)
fig.suptitle('Can we simplify or cheapen the acceleration measurement?',fontsize=15,y=1.02)
fig.text(.02,-.18,'Fixed-controller diagnostics. Removed channel disables both inertia reconstruction and acceleration-derived damping.\nScrew + soft pad passes force in 3/3 removal cases, but two exceed its 1.42 A RMS-current screen.',fontsize=10)
fig.tight_layout()
save(fig,'current-acceleration-diagnostics','Do we need an accelerometer, and how quiet must it be?', [acq_path], 'Normalizedinputnoise is not filteredRMS or manufacturernoiseguarantee; removal affects multiple controlleruses. Counts include force/physical/sensor/actuator/continuous-current screens.')

DATA.mkdir(exist_ok=True)
manifest={'status':'generated_current_matrix_svg_figures','date':'2026-10-07','script_sha256':sha(Path(__file__)),'figures':figures}
(DATA/'figure_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'figures':len(figures),'svg_bytes':sum((ROOT/z['path']).stat().st_size for z in figures)}))
