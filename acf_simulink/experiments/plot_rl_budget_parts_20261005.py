"""Plot verified chips-only acquisition costs, without performance claims."""
from pathlib import Path
import hashlib
import json
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
STUDY=ROOT/'acf_simulink/validation/rl_budget3h_20261005'
source=STUDY/'budget_parts_review_20261005.json'
data=json.loads(source.read_text())
out=STUDY/'figures';out.mkdir(exist_ok=True)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'svg.fonttype':'none'})
fig,ax=plt.subplots(figsize=(10.8,4.5),layout='constrained')
sensor=data['parts'][0]['unit_price_excluding_vat_eur']*(1+data['vat_rate'])
adc=data['parts'][1]['unit_price_excluding_vat_eur']*(1+data['vat_rate'])
for y,n in [(1,1),(0,2)]:
    total=sensor+n*adc
    ax.barh(y,sensor,height=.45,color='#1768a6',label='One ADXL354C' if n==1 else None)
    ax.barh(y,n*adc,left=sensor,height=.45,color='#31a09b',label='ADS131M08 ADC(s)' if n==1 else None)
    ax.text(total-1,y+.30,f'€{total:.2f}',va='bottom',ha='right',fontweight='bold',color='#16314b')
ax.axvline(100,color='#b44f17',ls='--',lw=1.5)
ax.text(99,1.55,'€100 chip budget',ha='right',color='#b44f17')
ax.set(xlim=(0,113),ylim=(-.8,1.8),xlabel='Quantity-one chip cost including 21% VAT (€)',
       yticks=[1,0],yticklabels=['Planar prototype\n6 bridge + 2 acceleration channels',
                                'Spatial channel provision\n6 bridge + 3 acceleration channels'],
       title='Accelerometer and ADC fit the prototype chip budget')
ax.spines[['top','right','left']].set_visible(False)
ax.grid(axis='x',alpha=.15);ax.set_axisbelow(True)
ax.legend(loc='lower left',frameon=False)
fig.text(.01,-.055,'DigiKey NL, 5 October 2026. One or two eight-channel ADCs; digital position is separate.\n'
         'Excludes amplifiers, PCB, assembly and shipping. Spatial channel provision is not 3D force qualification.',
         fontsize=9,color='#536273')
for suffix in ['svg','png']:
    fig.savefig(out/f'rl_budget_chip_cost.{suffix}',bbox_inches='tight',dpi=125,facecolor='white')
manifest={'status':'complete','source':str(source.relative_to(ROOT)),
          'sourceSha256':hashlib.sha256(source.read_bytes()).hexdigest(),
          'figureSha256':hashlib.sha256((out/'rl_budget_chip_cost.svg').read_bytes()).hexdigest(),
          'scope':'Quantity-one chips-only price calculation, not controller qualification.'}
(out/'rl_budget_chip_cost_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps(manifest))
