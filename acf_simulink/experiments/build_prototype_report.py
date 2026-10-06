"""Rebuild reviewed report data from retained native simulation evidence.

Run from any directory. This updates only the report's editable data.json;
building and offline packaging use the installed Data app runtime.
"""
import argparse, json, math
from pathlib import Path
root=Path(__file__).resolve().parents[2]; v=root/'acf_simulink/validation'; dest=root/'reports/prototype-design-20260930/src/data.json'
s=json.loads(dest.read_text());q={}
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--complete',action='store_true',help='Mark finished report content complete.')
args=parser.parse_args()
def add(key,filename,field='rows',label=None,caveats=None):
 d=json.loads((v/filename).read_text());rows=d[field] if isinstance(d,dict) else d
 q[key]={'rows':rows,'source':{'label':label or filename,'files':['acf_simulink/validation/'+filename], 'caveats':caveats or []}}
 return d
for key, filename, scope in [
 ('actual_range','rigid_sensor_qualification_actual_rigid_dense.json','Actual B-S cell, 1.4 kg downstream; S-A relative position; corrected contact-error gate'),
 ('actual_calibration','rigid_sensor_qualification_actual_rigid_calibration.json','Separate mass and robot-timing calibration sensitivity; not a joint robustness proof'),
 ('actual_corners','controller_comparison_upstream_rigid_qualify.json','20 native clean/noisy corner recordings plus solver/save checks; original raw-cell gate'),
 ('actual_coulomb','controller_comparison_upstream_rigid_coulomb.json','Matched native before/after Coulomb reconstruction correction'),
 ('actual_pid','controller_comparison_upstream_pid.json','PID using actual tared B-S force, distinguished from true contact force'),
 ('reference_winding','actuator_qualification_final_winding.json','Calibrated drive and counterfactual near-contact force-reference qualification'),
 ('reference_driver_mismatch','actuator_qualification_uncalibrated_driver.json','Uncalibrated winding resistance/back-EMF driver comparison'),
 ('upstream_jerk_clean','robot_prediction_tool_jerk_ideal.json','Actual upstream sensing with zero physical noise; compliant lower link'),
 ('upstream_jerk_noise','robot_prediction_tool_jerk_noise.json','Actual upstream sensing with physical sensor noise; compliant lower link'),
 ('upstream_current_readback','robot_prediction_tool_jerk_current.json','Optional third current measurement, explicitly modeled; compliant lower link'),
]:
 if (v/filename).exists():add(key,filename,label=scope,caveats=[scope])
calibration=json.loads((v/'rigid_sensor_qualification_actual_rigid_calibration.json').read_text())
q['actual_robot_timing']={'rows':[r for r in calibration['rows'] if r['study']=='actual_robot_timing_frozen_predictor'],
 'source':{'label':'112 actual rigid-sensor robot timing cases with fixed expected response',
 'files':['acf_simulink/validation/rigid_sensor_qualification_actual_rigid_calibration.json'],
 'caveats':['Earlier 20 ms gate reference; same fast observer/controller, bounded normal-motion program.']}}
combined=json.loads((v/'mechanical_study_combined.json').read_text())
for key,field in [('mechanics_lengths','springLengths'),('mechanics_matrix','feasibilityMatrix'),('mechanics_stress_matrix','stressDamping')]:
 q[key]={'rows':combined['figureDatasets'][field],'source':{'label':'Exact plotted mechanical dataset: '+field,'files':['acf_simulink/validation/mechanical_study_combined.json'],'caveats':['Compliant design comparisons use near-contact force observation.']}}
add('dense_range','rigid_sensor_qualification_dense_measured_travel.json',label='350 native runs; measured magnet feedback in both loops',caveats=['Axial force, near-contact force observation, fixed orientation, finite force/angle grid.', 'One clean case and four noise seeds per force/orientation point. 100 microsecond plant step; 200 microsecond controller sample.'])
# Keep the full native corner records, not merely favorable designs.
low=add('mechanics_low_cases','full_range_mechanics_low_results.json',label='800 lower-rate compliant native runs plus recorded geometry screens')
high=add('mechanics_high_cases','full_range_mechanics_high_results.json','cases',label='800 higher-rate compliant native corner runs')
add('mechanics_stress','full_range_mechanics_high_results.json','stressCases',label='100 noisy/delayed native runs, all 25 geometry-feasible compliant designs')
rows=[]
for a in low['summary']:
 rows.append({**a,'id':'low:'+str(a['id']),'geometryPassed':a['geometryFullRangePassed'],'trackingPassed':a['physicalFullRangePassed'],'nominalError_N':a['worstError_N'],'stressError_N':None,'stressTested':False})
for a in high['designs']:
 rows.append({**a,'id':'high:'+str(a['id']),'geometryPassed':a['geometryPassedAllCorners'],'trackingPassed':a['nominalTrackingPassedAllCorners'],'nominalError_N':a['worstNominalError_N'],'stressError_N':a['worstStressError_N']})
q['mechanics_designs']={'rows':rows,'source':{'label':'400 distinct compliant spring/damper designs, four force/orientation corners each','files':['acf_simulink/validation/full_range_mechanics_low_results.json','acf_simulink/validation/full_range_mechanics_high_results.json'],'caveats':['Unknown or untested stress metrics are null, not zero. Controller coefficients are recalculated for each identified design.','No compliant lower-rate design passes the full-range spring geometry screen. The 25 feasible designs all use the strongest spring pair in both links.']}}
add('controller_cases','controller_comparison_verify.json',label='Ordinary execution, save/reopen and solver refinement checks on shortlisted controllers')
verify=add('controller_verified','controller_comparison_verify.json','verification',label='Shortlisted controller metrics and finer-step verification')
# Separate large phase evidence remains inspectable.
for key,n in [('controller_shortlist','controller_comparison_shortlist.json'),('coil_sweep','actuator_qualification_coil.json'),('sensing','rigid_sensor_qualification_sensing.json'),('placement','rigid_sensor_qualification_placement_selected.json'),('robustness','rigid_sensor_qualification_selected_robust.json')]:
 if (v/n).exists():add(key,n,label=n.replace('_',' ').removesuffix('.json'))
for key,n in [('leadscrew','actuator_qualification_leadscrew_refined.json'),('lead_extension','actuator_qualification_leadscrew_high_gain.json'),('ideal','actuator_qualification_ideal.json')]:
 if(v/n).exists():add(key,n)
for key,n in [('robot_gate','selected_robot_gate_results.json'),('robot_gate_fixture','long_robot_gate_results.json')]:
 if(v/n).exists():add(key,n,'cases',label='60 second recentering stress: '+('recommended mechanics' if key=='robot_gate' else 'earlier soft-pair fixture'))
for key,n in [('robot_predictor','robot_prediction_bandwidth.json'),('robot_predictor_matched','robot_prediction_precision_matched.json'),('robot_predictor_motion','robot_prediction_motion_bounded.json'),('selected_winding','actuator_qualification_selected_winding.json')]:
 if(v/n).exists():add(key,n)
# Explicit formula-generated geometry is identified as calculation, not simulation.
normal=[]
for i in range(201):
 a=i/200; slope=.001/.01*192*a*a*(1-a)**2*(1-2*a); cos=1/math.sqrt(1+slope*slope)
 normal.append({'path_mm':10*a,'surfaceHeight_mm':64*a**3*(1-a)**3,'slope':slope,'tilt_deg':math.degrees(math.atan(slope)),'normalFor50Axial_N':50/cos,'axialFor50Normal_N':50*cos,'lateralFor50Axial_N':50*slope})
q['force_geometry']={'rows':normal,'source':{'label':'Analytical one-dimensional bump geometry, not a normal-force simulation','files':['acf_simulink/experiments/build_prototype_report.py'],'metricDefinitions':[{'label':'Normal force with an axial target','definition':'With zero tangential contact load, N = Faxis sqrt(1+slope²).','formula':'50*sqrt(1+slope*slope)'}],'caveats':['Assumes negligible tangential force. Real grinding contact and guide side loads require a three-dimensional model.']}}
params=[
 ('Moving tool side','B + S','0.6 + 1.4 = 2 kg','Body A adds 0.4 kg; actual moving coil must fit holder mass'),
 ('Attachment','Upper and lower links','Rigid connections','No external springs or dampers; both bodies remain attached'),
 ('Spring length','Total','0 mm','The 100 mm spring-only budget is satisfied; whole-flange size is not modeled'),
 ('Load cell','Placement','Between B and S','Whole 1.4 kg tool is downstream; exact orientation-specific gravity tare'),
 ('Position measurement','Frame','Tool S relative to body A','Equivalent to reversed magnet gap with rigid B-S attachment'),
 ('Coil force constant','Kf','Approximately 20 N/A','Identify actual force constant; unknown factory tolerance is not qualified'),
 ('Winding','R / L','4 ohm / 3 mH','Cold nominal targets; identify hot resistance and regulate current'),
 ('Drive','Supply / current ceiling','24 V / 3.5 A','Nominal actual-sensor loop has ample electrical headroom'),
 ('Current-loop target','tau','100 microseconds','1.59 kHz first-order bandwidth; model must match actual closed-loop response'),
 ('Mechanical travel','Gap','25 mm total','Midpoint 12.5 mm; stop and fail at either limit'),
 ('Force and position sampling','Ts','100 microseconds (10 kHz)','Use synchronized sampling and identified channel transfer functions'),
 ('Force filter / configured delay','Timing','100 / 100 microseconds','Known modeled values, not an allowance for arbitrary extra latency'),
 ('Force noise / position noise','RMS','0.02 N / 1 micrometre','Native independent noise tests; bias and timing are separate requirements'),
 ('Observer cost','Force / integral / current scales','0.32 N / 0.02 N s / 1 A','Design weights, not guaranteed sensor accuracy'),
 ('Observer process model','Surface jerk power','70','Nine physical/disturbance states; no bump preview'),
 ('Robot prediction','Expected delay / response','80 / 60 ms','Reads issued velocity request only; residual acceleration state corrects mismatch'),
 ('Robot correction','Requested gain / deadband','0.8 per second / 0.5 mm','Gain is capped using delay, response, gate dwell and trend-filter time'),
 ('Force gate','Error / slope / dwell','0.2 N / 2 N/s / 100 ms','Filtered reconstructed contact error; qualified 100 ms gate trend filter')]
q['prototype_parameters']={'rows':[{'item':a,'parameter':b,'value':c,'meaning':d} for a,b,c,d in params],'source':{'label':'Actual two-sensor candidate and explicit prototype design assumptions','files':['acf_simulink/validation/controller_comparison_upstream_rigid_corrected_gate_candidate.mat','acf_simulink/validation/final_gate_100ms_corners.json','acf_simulink/generated/acf_force_control_simple.slx'],'caveats':['Approximate component targets must be identified in the assembled prototype.','Numerical qualification is axial and conditional on matched sensor/drive models.']}}
for key,n,field in [('actual_gate_corners','final_gate_100ms_corners.json','cases'),('actual_gate_recenter','final_robot_gate_results.json','cases'),('actual_winding','actuator_qualification_actual_sensor_winding.json','rows'),('actual_winding_calibration','actuator_qualification_actual_sensor_calibration.json','rows'),('actual_timing_bias','controller_comparison_upstream_rigid_timing_bias.json','rows'),('actual_precision','controller_comparison_upstream_rigid_calibration_bounds.json','rows'),('actual_mass_precision','rigid_sensor_qualification_actual_rigid_mass_tight.json','rows'),('actual_final_range','rigid_sensor_qualification_final_rigid_dense.json','rows')]:
 if (v/n).exists():add(key,n,field,label=n.replace('_',' ').removesuffix('.json'))
add('actual_gate_plot','final_robot_gate_plot_data.json',label='Exact native recentering plot data and full-rate metrics')
final_range=json.loads((v/'rigid_sensor_qualification_final_rigid_dense.json').read_text())
q['actual_final_fine']={'rows':[final_range['fineWorstCaseRow']],
 'source':{'label':'One independent 25 microsecond repeat of final worst recording',
 'files':['acf_simulink/validation/rigid_sensor_qualification_final_rigid_dense.json'],
 'caveats':['Not an additional force/orientation point or a second full range qualification.']}}
q['actual_final_range']['source']['files'] += [
 'acf_simulink/validation/rigid_sensor_qualification_final_rigid_dense_source_manifest.json',
 'acf_simulink/validation/rigid_sensor_qualification_final_rigid_dense_traces.mat']
q['actual_precision']['source']['caveats']=[
 '36 confirmed rows from an interrupted worker. The proposed +/-0.02 N and known 200 us delay cohorts did not execute.',
 'Interruption is retained in controller_comparison_upstream_rigid_calibration_bounds_interruption.json.']
uncertainty=[]
for key,filename in [('current_model_noise','controller_current_uncertainty_current_model_noise.json'),('force_bias_state','controller_current_uncertainty_force_bias_state.json')]:
 d=add('uncertainty_'+key,filename,label='Bounded actual-sensor actuator model uncertainty experiment')
 uncertainty.extend(d['rows'])
q['uncertainty_cases']={'rows':uncertainty,'source':{'label':'54 executed native uncertainty experiments; no all-passing cohort',
 'files':['acf_simulink/validation/controller_current_uncertainty_current_model_noise.json','acf_simulink/validation/controller_current_uncertainty_force_bias_state.json'],
 'caveats':['100 ms gate, 50 N/90 degrees, seeds 7 and 1729; unknown Kf +/-0.1% and nominal. Not a promoted controller.']}}
add('uncertainty_plot','controller_current_uncertainty_plot_data.json',field='series',label='Exact cohort worst errors for estimator-uncertainty figure')
calibration_keys=['actual_calibration','actual_mass_precision','actual_timing_bias','actual_precision','actual_winding','actual_winding_calibration']
q['sensor_evidence']={'rows':[{**r,'evidenceDataset':key} for key in calibration_keys for r in q[key]['rows']],
 'source':{'label':'Actual-sensor calibration, mass coefficient, timing, bias and drive evidence',
 'files':list(dict.fromkeys(file for key in calibration_keys for file in q[key]['source']['files'])),
 'caveats':['Independent sensitivity studies, mostly earlier 20 ms gate; not joint final-gate tolerance.',
 'Tight mass varies inertia-compensation coefficient only. Bias precision includes only 36 confirmed interrupted-worker rows.']}}
nominal_precision=[r for r in q['actual_winding']['rows'] if r['scenario']=='nominal' and r['Ilimit_A']==3.5 and r['Vlimit_V']==24 and r['force_N']==50 and r['angle_deg']==90]
assert len(nominal_precision)==4
q['actual_calibration_plot']={'rows':q['actual_winding_calibration']['rows']+nominal_precision,
 'source':{'label':'48 calibration cases plus four nominal zero-deviation figure anchors',
 'files':q['actual_winding_calibration']['source']['files']+q['actual_winding']['source']['files'],
 'caveats':['Earlier 20 ms gate reference. The 48-case calibration table is a separate unchanged dataset.']}}
jerk_keys=['upstream_jerk_clean','upstream_jerk_noise']
q['upstream_jerk_comparison']={'rows':[{**r,'evidenceDataset':key} for key in jerk_keys for r in q[key]['rows']],
 'source':{'label':'Actual upstream compliant observer, noiseless and noisy conditional comparison at 50 N/90 degrees',
 'files':[q[key]['source']['files'][0] for key in jerk_keys]+['acf_simulink/validation/robot_prediction_figures_sources.json'],
 'caveats':['Best compared settings differ in observer snap cost and feedforward; this is not a noise-only controlled pair.',
 'Neither the clean case nor these noise tests qualifies the final rigid topology.']}}
for key,n in [('actual_control_trace','controller_comparison_upstream_rigid_plot_data.json'),('actual_drive_plot','actuator_qualification_actual_sensor_summary.json'),('controller_summary','controller_comparison_summary.json')]:
 if (v/n).exists():
  d=json.loads((v/n).read_text());q[key]={'rows':[d], 'source':{'label':n,'files':['acf_simulink/validation/'+n],'caveats':['The companion includes exact plotted trace/data and its qualification scope.']}}
saved=json.loads((v/'final_saved_model_results.json').read_text())
layout=json.loads((v/'final_diagram_cleanup_results.json').read_text())
reg=saved['independentGoldenRegression']
q['native_verification']={'rows':[
 {'check':'Native physical implementation','result':f"{saved['nativeBlocks']} blocks; {saved['hiddenCodeBlocks']} hidden-code blocks",'detail':'Count includes all inactive actuator/controller alternatives; maximum subsystem depth three.'},
 {'check':'Saved independent worst-case regression','result':f"{layout['all35MaxDifference']} difference in all 35 signals",'detail':f"Observer-state difference {layout['observerStateMaxDifference']}; final S–A labels/tags and save/reopen checked."},
 {'check':'Diagram geometry','result':f"{len(layout['pages'])} pages; zero intersections",'detail':f"Zero block overlaps and wire/block interior intersections; {layout.get('danglingWireBranchesRemoved',0)} dangling branches removed with unchanged native behavior."},
 {'check':'Rigid force identity','result':f"{reg['forceIdentityResidual_N']:.3g} N maximum residual",'detail':'True contact force = actuator force + (B+S)*(gravity + tool acceleration).'},
 {'check':'Actuator and strategy smoke checks','result':f"{len(saved['actuatorStrategySmokeCases'])}/5 passed",'detail':'Flat contact: coil observer, PI, PID, leadscrew, ideal-position reference. Not sharp-bump qualification.'},
 {'check':'Safety stopping','result':'Both fixtures stopped and failed','detail':'Contact loss and travel stop tested separately; no post-failure impact dynamics.'},
 {'check':'Measured execution time','result':f"{reg['fullLoggingRuntime_s']:.3f} s for 4 s simulated",'detail':'One process timing with preparation and full native logging; startup excluded, not a benchmark median.'}],
 'source':{'label':'Executed saved-model, diagram and save/reopen checks',
 'files':['acf_simulink/validation/final_saved_model_results.json','acf_simulink/validation/final_diagram_cleanup_results.json','acf_simulink/validation/final_save_reopen_results.json'],
 'caveats':['Software equivalence and fixture checks do not constitute physical prototype qualification.']}}
s.update(queries=q,buildStatus='complete' if args.complete else 'updating',report={'asOf':'2026-09-30'});dest.write_text(json.dumps(s,indent=2,allow_nan=False)+'\n')
print({k:len(v['rows']) for k,v in q.items()})
