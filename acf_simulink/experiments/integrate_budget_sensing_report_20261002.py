"""Integrate a dated sensing supplement without losing the prior delivery.

The preserved baseline is authoritative for unaffected prose. Scientific
worker files, the original ZIP and the public desktop model remain untouched.
"""
import argparse
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
VAL=ROOT/'acf_simulink/validation'
BASE=VAL/'budget_sensing_report_base_20261002'


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--complete',action='store_true');args=parser.parse_args()
    supplement=(ROOT/'reports/sensing-cost-study-20261002.md').read_text()
    baseline=(BASE/'reports/active-contact-flange-report.md').read_text()
    # Preserve the complete original model explanation and its source-bound
    # 3500-case results. New dated sections have independent evidence.
    chapters=supplement[supplement.index('## Why a cheaper ADC can be enough'):]
    chapters=chapters.replace('## Prototype acceptance and recommendation',
                              '## Acceptance of the lower-cost sensing chain')
    marker='## Running, changing and reproducing the model'
    assert marker in baseline
    current=baseline.replace(marker,chapters+'\n\n'+marker,1)
    if args.complete:
        ledger=json.loads((VAL/'budget_sensing_execution_ledger_20261002.json').read_text())
        grid=json.loads((VAL/'budget_adc_range_grid_20261002.json').read_text())
        hot=json.loads((VAL/'budget_adc_hot_coil_summary_20261002.json').read_text())
        assert ledger['status']=='complete' and ledger['executedRecordings']==999
        assert grid['uniqueConditions']==700 and grid['passes']==700
        assert hot['rows'][0]['recordings']==7 and hot['rows'][0]['passes']==7
        old_result=next(line for line in current.splitlines() if line.startswith('**Result:**'))
        current=current.replace(old_result,old_result.replace('**Result:**','**Scientific-reference result:**',1)+
          '\n\n**Lower-cost sensing update, 2 October:** the ADS131M08 at 30 ksample/s with a matched 8 kHz complete-response envelope passes all **700 new nominal conditions**, worst **0.292804 N**. Seven selected hot-coil long/mixed cases pass at **0.438821 N** worst. The 999-recording cost study supports reducing ADC cost while retaining the reference acceleration sensors; the cheaper sensor envelopes consume substantial margin or fail. These results have their own source, noise seeds and qualification scope.',1)
        current=current.replace('A simultaneous-sampling **AD7606C-18** provides',
          'The original scientific reference uses a simultaneous-sampling **AD7606C-18**, which provides',1)
        current=current.replace('The faster acquisition design uses a **100 ksample/s simultaneous raw clock**',
          'The scientific-reference acquisition uses a **100 ksample/s simultaneous raw clock**',1)
        current=current.replace('This acquisition graph is used by the selected final controller; the tests below distinguish its nominal behavior from calibration and response mismatch.',
          'This acquisition graph is used by the scientific-reference qualification below. The dated lower-cost study later in this report separately evaluates replacement converters with their own filters, quantization, noise and timing.',1)
        old_acquisition='| Acquisition | 100 ksample/s simultaneous raw force/acceleration, matched two-pole ~5.1 kHz response, 50 µs raw EMA, known ~20 µs pipeline |'
        assert old_acquisition in current
        current=current.replace(old_acquisition,
          '| Acquisition | ADS131M08, 30 ksample/s per simultaneous channel, measured matched ~8 kHz complete response, 50 µs EMA setting, 20 µs total readout/MCU pipeline |\n'
          '| Acceleration sensors | Retain two ADXL1002-class analog axes for the planar prototype; three for spatial reconstruction, with enough simultaneous ADC channels |',1)
        recommendation_marker='A Moticont LVCM-070-051-01-class coil demonstrates'
        assert recommendation_marker in current
        current=current.replace(recommendation_marker,
          'The lower-cost acquisition is the new prototype starting point; the public desktop model retains the frozen scientific-reference chain. Seven new native hot-winding cases with the selected M08 chain pass at 0.438821 N worst, but request up to **31.738466 V**, leaving only **0.261534 V** at a 32 V usable-output limit. Measure real driver dropout and hot-coil headroom. The approximately €9 converter reduces ADC cost; the tested cheaper accelerometers do not preserve the preferred 0.05 N force reserve.\n\n'+recommendation_marker,1)
    else:
        current=current.replace('## Our goal\n',
          '## Our goal\n\n**Working report:** lower-cost sensing comparisons are being added; the new range and sensor stress studies are ongoing.\n',1)
    (ROOT/'reports/active-contact-flange-report.md').write_text(current)
    config=json.loads((BASE/'acf_simulink/validation/inertial_report_inputs.json').read_text())
    config['asOf']='2026-10-02';config['readyForDelivery']=args.complete
    specs=[
      ('budget_adc_screen','budget_adc_screen_summary_20261002.json','168 native ADC bandwidth/rate recordings','native_simulation',['rows']),
      ('budget_adc_long','budget_adc_long_summary_20261002.json','Repeated-bump calibration comparisons','native_simulation',['rows']),
      ('budget_accelerometer_screen','budget_accelerometer_screen_summary_20261002.json','36 native required sensor-response envelope recordings','native_response_envelope_simulation',['rows']),
      ('budget_adc_parts','adc_cost_component_review_20261002.json','Primary ADC contracts and indicative component prices','manufacturer_document_review',['candidates']),
      ('budget_adc_electrical','adc_cost_component_review_20261002.json','Noise, range and readout calculations','analytical_calculation',['derivedBudgets']),
      ('budget_accelerometer_parts','accelerometer_cost_component_review_20261002.json','Primary accelerometer contracts and indicative prices','manufacturer_document_review',['candidates']),
    ]
    if args.complete:
        specs += [
          ('budget_adc_range','budget_adc_range_grid_20261002.json','700 native lower-cost acquisition grid recordings','native_simulation',['rows']),
          ('budget_sensing_robustness','budget_sensing_robustness_20261002.json','Executed noise, timing and solver followups','native_simulation',['rows']),
          ('budget_accelerometer_stress','budget_accelerometer_stress_summary_20261002.json','49 native sensor/calibration/drive envelope comparisons','native_response_envelope_simulation',['rows']),
          ('budget_dynamic_matching','budget_dynamic_matching_summary_20261002.json','Seven native tighter dynamic-matching comparisons','native_response_envelope_simulation',['rows']),
          ('budget_adc_hot_coil','budget_adc_hot_coil_summary_20261002.json','Seven native selected-acquisition hot-winding comparisons','native_simulation',['rows']),
          ('budget_sensing_execution','budget_sensing_execution_ledger_20261002.json','Complete 999-recording sensing-cost execution ledger','native_execution_audit',['rows']),
        ]
    for identifier,filename,label,kind,rowspath in specs:
        query={'id':identifier,'file':'acf_simulink/validation/'+filename,
          'label':label,'kind':kind,'rowsPath':rowspath,
          'caveats':['New acquisition source b370; original 3500-case reference remains separate. Required accelerometer response envelopes do not qualify undisclosed real-part phase.']}
        if kind.startswith('native_'):
            query['requiredStatus']=['complete']
        config['queries'].append(query)
    sections={
       'why-a-cheaper-adc-can-be-enough':['budget_adc_parts','budget_adc_electrical'],
       'native-experiments-justify-the-bandwidth-and-sample-rate':['budget_adc_screen','budget_adc_long'],
       'cheaper-accelerometers-are-limited-mainly-by-response':['budget_accelerometer_screen','budget_accelerometer_parts'],
       'acceptance-of-the-lower-cost-sensing-chain':['budget_accelerometer_screen','budget_adc_long'],
    }
    if args.complete:
        sections['native-experiments-justify-the-bandwidth-and-sample-rate'] += ['budget_adc_range','budget_sensing_robustness','budget_adc_hot_coil']
        sections['cheaper-accelerometers-are-limited-mainly-by-response'] += ['budget_accelerometer_stress','budget_dynamic_matching']
        sections['acceptance-of-the-lower-cost-sensing-chain'] += ['budget_accelerometer_stress','budget_adc_range','budget_adc_hot_coil','budget_sensing_execution']
        sections['our-goal']=['qualified_range','budget_adc_range','budget_adc_hot_coil','budget_sensing_execution']
        sections['concrete-recommendation']=config['tableQueries']['concrete-recommendation']+[
          'budget_adc_parts','budget_adc_range','budget_adc_hot_coil','budget_accelerometer_stress']
    config['sectionQueries'].update(sections)
    for section,ids in sections.items():
        config['tableQueries'][section]=ids
        config['tableTitles'].setdefault(section,section.replace('-',' ').capitalize())
    config['tableTitles'].update({
      'why-a-cheaper-adc-can-be-enough':'ADC range, resolution and response',
      'native-experiments-justify-the-bandwidth-and-sample-rate':'Native repeated-bump acquisition comparisons',
      'cheaper-accelerometers-are-limited-mainly-by-response':'Sensor response and drive stress comparisons',
    })
    for filename,query in [('budget_adc_bandwidth_rate.png','budget_adc_screen'),
                           ('budget_adc_long_waveforms.png','budget_adc_long'),
                           ('budget_accelerometer_envelopes.png','budget_accelerometer_screen')]:
        config['figures'].append({'file':'figures/'+filename,'queryId':query})
    if args.complete:
        for filename,query in [('budget_sensor_calibration_stress.png','budget_accelerometer_stress'),
                              ('budget_sensing_response_noise.png','budget_sensing_robustness')]:
            config['figures'].append({'file':'figures/'+filename,'queryId':query})
    target=VAL/'budget_sensing_report_inputs_20261002.json'
    target.write_text(json.dumps(config,indent=2)+'\n')
    print(target)


if __name__=='__main__':main()
