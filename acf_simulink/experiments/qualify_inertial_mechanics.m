function report=qualify_inertial_mechanics(sourceFolder,outputFolder,phase,options)
%QUALIFY_INERTIAL_MECHANICS Native mechanical comparisons with measured S inertia.
% Copies the frozen IMU model/helpers into an owned directory. The physical
% plant stays in native Simulink; optional springs and structural cell
% stiffness are separate design cases. Every original native sample is scored.
assert(~usejava('desktop'),'Run this study in an isolated MATLAB -batch process.');
project=fileparts(fileparts(mfilename('fullpath')));
if nargin<2,outputFolder=fullfile(project,'validation');end
if nargin<3,phase='structural_pilot';end
if nargin<4,options=struct;end
if strcmp(phase,'build_lower_inverse')
 report=build_lower_inverse_snapshot(sourceFolder,outputFolder,options);return;
end
if strcmp(phase,'freeze_lower_series')
 report=freeze_lower_series(sourceFolder,outputFolder,options);return;
end
options=defaults(options,struct('step_s',10e-6,'stopTime_s',4, ...
 'cellStiffness_N_m',64e6,'cellDampingRatio',.02,'candidateFile','', ...
 'forceKp',.03,'forceKi',20,'forceKd',0,'derivativeFilter_s',.0003, ...
 'accelerationFeedforwardScale',1,'guideFrictionFeedforwardScale',1, ...
 'notchEnabled',false,'notchFrequencyScale',1,'notchZeroDamping',.01, ...
 'notchPoleDamping',.5,'guideVelocityPrediction_s',0, ...
 'feedbackFilterEnabled',false,'feedbackFilterCutoff_Hz',800,'feedbackFilterDamping',1/sqrt(2), ...
 'geometryMode','','dragRatioFilter_s',NaN,'dragRatioMinForce_N',NaN, ...
 'accelerationPrediction_s',NaN,'accelerationFeedforwardFilter_s',NaN, ...
 'springRates',[3 4],'springDamping_Ns_m',[0 15 60], ...
 'lowerInverseEnabled',false,'lowerInverseCutoff_Hz',1000,'lowerInverseFilterDamping',1/sqrt(2), ...
 'lowerInverseScale',1,'lowerInverseStiffnessScale',1,'lowerInverseDampingScale',1, ...
 'lowerInverseMassBScale',1,'inverseCutoffs_Hz',[300 600 1000 1500], ...
 'inverseStiffnessScales',[.8 1 1.2],'inverseDampingScales',[.8 1 1.2], ...
 'actualStiffnessScales',[.8 1 1.2],'actualDampingScales',[.8 1 1.2], ...
 'selectedSpringRate',4,'selectedSpringDamping_Ns_m',60, ...
 'requireConfiguredPreload',false,'commonPreload_N',NaN, ...
 'surfaceKind','','repeatPeriod_s',NaN,'solverSteps_s',[10e-6 5e-6], ...
 'cellRates_N_m',[40e6 51.2e6 64e6 80e6 96e6 110e6], ...
 'forceFilterTime_s',NaN,'accelFilterTime_s',NaN,'accelNoiseRms_m_s2',NaN, ...
 'accelQuantum_m_s2',NaN,'accelRange_g',NaN, ...
 'simulationMode','normal','fastRestart',true,'keepFailedNative',true,'keepAllNative',false, ...
 'seeds',[0 7 29 1729],'caseLimit',inf,'label',''));
if isempty(options.candidateFile)
 options.candidateFile=fullfile(sourceFolder,'normal_contact_trial_candidate.mat');
end
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
assert(isfile(sourceModel) && isfile(options.candidateFile),'Frozen IMU input is not ready.');
stem=['inertial_mechanics_' phase '_20260930'];
if ~isempty(options.label),stem=[stem '_' options.label];end
if ~isfolder(outputFolder),mkdir(outputFolder);end
resultFile=fullfile(outputFolder,[stem '_results.json']);
assert(~isfile(resultFile),'Preserve previous evidence; choose a fresh output folder or label.');
stateRoot=fullfile(getenv('HOME'),'.local','state','actuator-model');
workerFolder=tempname(stateRoot);
for suffix={'','helpers','cache','code','sdi','traces'}
 mkdir(fullfile(workerFolder,suffix{1}));
end
copyfile(sourceModel,fullfile(workerFolder,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile(options.candidateFile,fullfile(workerFolder,'candidate.mat'));
copyfile([mfilename('fullpath') '.m'],fullfile(workerFolder,'qualify_inertial_mechanics.m'));
addpath(fullfile(workerFolder,'helpers'),'-begin');clear acf_prepare acf_run acf_metrics acf_select_actuator;
loaded=load(fullfile(workerFolder,'candidate.mat'),'p');base=loaded.p;
assert(isfield(base,'accel') && base.accel.enabled,'This study requires the measured-acceleration candidate, not v1.');
assert(strcmp(base.sensor.positionTarget,'tool_relative_A') && base.robotPrediction.enabled);
base.sim.stopTime=options.stopTime_s;base.sim.evaluateFrom=0;base.sim.step=options.step_s;
if ~isempty(options.surfaceKind),base.surface.kind=options.surfaceKind;end
if isfinite(options.repeatPeriod_s),base.surface.repeatPeriod=options.repeatPeriod_s;end
if isfield(base,'springs')
 base.springs.requireConfiguredPreload=options.requireConfiguredPreload;
 if isfinite(options.commonPreload_N),base.springs.commonPreload_N=options.commonPreload_N;end
end
base.inertial.forceKp=options.forceKp;base.inertial.forceKi=options.forceKi;
base.inertial.forceKd=options.forceKd;base.inertial.derivativeFilter=options.derivativeFilter_s;
base.inertial.accelerationFeedforwardScale=options.accelerationFeedforwardScale;
base.inertial.guideFrictionFeedforwardScale=options.guideFrictionFeedforwardScale;
if isfinite(options.forceFilterTime_s),base.sensor.filterTime=options.forceFilterTime_s;end
if isfinite(options.accelFilterTime_s),base.accel.filterTime=options.accelFilterTime_s;end
if isfinite(options.accelNoiseRms_m_s2),base.accel.noiseRms=options.accelNoiseRms_m_s2;end
if isfinite(options.accelQuantum_m_s2),base.accel.quantum=options.accelQuantum_m_s2;end
if isfinite(options.accelRange_g),base.accel.range_g=options.accelRange_g;end
if isfield(base.inertial,'guideVelocityPrediction_s')
 base.inertial.guideVelocityPrediction_s=options.guideVelocityPrediction_s;
end
if options.notchEnabled || startsWith(phase,'structural_notch')
 assert(isfield(base.inertial,'currentNotch'),'Wait for the common native-notch source.');
 base.inertial.currentNotch.enabled=true;
 base.inertial.currentNotch.frequency_Hz=options.notchFrequencyScale*sqrt(options.cellStiffness_N_m/base.mechanics.mB)/(2*pi);
 base.inertial.currentNotch.zeroDamping=options.notchZeroDamping;
 base.inertial.currentNotch.poleDamping=options.notchPoleDamping;
elseif isfield(base.inertial,'currentNotch')
 base.inertial.currentNotch.enabled=false;
end
if options.feedbackFilterEnabled || startsWith(phase,'structural_feedback')
 base.inertial.feedbackFilter=struct('enabled',true,'cutoff_Hz',options.feedbackFilterCutoff_Hz, ...
  'damping',options.feedbackFilterDamping);
elseif isfield(base.inertial,'feedbackFilter')
 base.inertial.feedbackFilter.enabled=false;
end
if ~isempty(options.geometryMode),base.contact.geometryMode=options.geometryMode;end
if isfinite(options.dragRatioFilter_s),base.contact.dragRatioFilter_s=options.dragRatioFilter_s;end
if isfinite(options.dragRatioMinForce_N),base.contact.dragRatioMinForce_N=options.dragRatioMinForce_N;end
if isfinite(options.accelerationPrediction_s),base.inertial.accelerationPrediction_s=options.accelerationPrediction_s;end
if isfinite(options.accelerationFeedforwardFilter_s)
 assert(isfield(base.inertial,'accelerationFeedforwardFilter_s'),'Wait for the common native acceleration-FF filter source.');
 base.inertial.accelerationFeedforwardFilter_s=options.accelerationFeedforwardFilter_s;
end
if isfield(base.inertial,'lowerLoadInverse')
 base.inertial.lowerLoadInverse.cutoff_Hz=options.lowerInverseCutoff_Hz;
 base.inertial.lowerLoadInverse.filterDamping=options.lowerInverseFilterDamping;
end
catalogFile=fullfile(project,'experiments','spring_catalog.json');catalog=jsondecode(fileread(catalogFile));
catalogHash=sha256(catalogFile);
files=dir(fullfile(workerFolder,'helpers','*.m'));helperSources=struct([]);
for j=1:numel(files)
 one=struct('file',files(j).name,'sha256',sha256(fullfile(files(j).folder,files(j).name)));
 helperSources=append(helperSources,one);
end
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
oldThreads=maxNumCompThreads(2);
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(workerFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(workerFolder,[mdl '.slx']));
if ~isempty(options.geometryMode)
 assert(getSimulinkBlockHandle([mdl '/Contact force measurement/Normal force projection/Measured drag geometry'])~=-1, ...
  'Wait for the common native measured-drag geometry source.');
end
assert(ismember(options.simulationMode,{'normal','accelerator'}));
set_param(mdl,'SimulationMode',options.simulationMode);
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,oldThreads));
configs=struct([]);
switch phase
 case 'structural_pilot'
  for topology={'rigid','structural'}
   if strcmp(topology{1},'rigid'),zetas=0;else,zetas=[0 .02 .05];end
   for zeta=zetas
    for seed=[0 1729]
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,topology{1},options.cellStiffness_N_m,0,zeta,catalog);
     add_config(q,topology{1},'structural_cell_initial_probe',zeta,0,0);
    end
   end
  end
 case 'structural'
  for cellRate=[0 20e6 64e6 110e6]
   for mass=[.1 2]
    for force=[5 50]
     for angle=[0 90]
      for seed=[0 1729]
       q=physical_case(base,mass,force,angle,seed);
       if cellRate==0,topology='rigid';else,topology='structural';end
       q=mechanical_case(q,topology,cellRate,0,0,catalog);
       add_config(q,topology,'zero_damping_structural_endpoints',0,0,0);
      end
     end
    end
   end
  end
 case {'structural_damped','structural_selected'}
  for mass=[.1 2]
   for force=[5 50]
    for angle=[0 90]
     for seed=[0 1729]
      q=physical_case(base,mass,force,angle,seed);
      q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
      add_config(q,'structural','finite_cell_endpoint_confirmation',options.cellDampingRatio,0,0);
     end
    end
   end
  end
 case 'structural_tune'
  gains=[.03 20 0;.01 5 0;.005 1 0;0 1 0;0 .1 0;0 .01 0; ...
   .01 5 .00003;.01 3 .0001;0 1 .0001];
  for zeta=[0 .02 .05]
   for j=1:size(gains,1)
    for seed=[0 1729]
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,zeta,catalog);
     q.inertial.forceKp=gains(j,1);q.inertial.forceKi=gains(j,2);q.inertial.forceKd=gains(j,3);
     add_config(q,'structural','finite_cell_controller_tuning',zeta,0,0);
    end
   end
  end
 case 'structural_integral'
  for zeta=[0 .02 .05]
   for ki=[5 20 50]
    for seed=[0 1729]
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,zeta,catalog);
     q.inertial.forceKp=0;q.inertial.forceKi=ki;q.inertial.forceKd=0;
     add_config(q,'structural','finite_cell_integral_only_search',zeta,0,0);
    end
   end
  end
 case 'structural_notch_pilot'
  for poleDamping=[.3 .5]
   for gains=[.03 20;.01 20;0 20].'
    for seed=[0 1729]
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,0,catalog);
     q.inertial.forceKp=gains(1);q.inertial.forceKi=gains(2);q.inertial.forceKd=0;
     q.inertial.currentNotch.poleDamping=poleDamping;
     add_config(q,'structural','native_total_current_notch_search',0,0,0);
    end
   end
  end
 case 'structural_notch_sensitivity'
  for frequencyScale=[.7 .8 1 1.2 1.3]
   for seed=[0 1729]
    q=physical_case(base,2,50,90,seed);
    q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
    q.inertial.currentNotch.frequency_Hz=frequencyScale*sqrt(options.cellStiffness_N_m/q.mechanics.mB)/(2*pi);
    add_config(q,'structural','identified_notch_frequency_sensitivity',options.cellDampingRatio,0,0);
   end
  end
 case 'structural_notch_guide'
  for kp=[.03 .01 0]
   for lead=[.0003 .0004]
    for seed=[0 1729]
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
     q.inertial.forceKp=kp;q.inertial.forceKi=20;q.inertial.forceKd=0;
     q.inertial.guideVelocityPrediction_s=lead;
     add_config(q,'structural','finite_cell_notch_and_causal_guide_timing',options.cellDampingRatio,0,0);
    end
   end
  end
 case 'structural_notch_depth'
  for depth=[.001 .003 .01]
   for poleDamping=[.3 .5 1]
    for seed=[0 1729]
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
     q.inertial.currentNotch.zeroDamping=depth;q.inertial.currentNotch.poleDamping=poleDamping;
     add_config(q,'structural','native_notch_depth_width_search',options.cellDampingRatio,0,0);
    end
   end
  end
 case 'structural_notch_endpoints'
  for mass=[.1 2]
   for force=[5 50]
    for angle=[0 90]
     for seed=[0 1729]
      q=physical_case(base,mass,force,angle,seed);
      q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
      add_config(q,'structural','notched_finite_cell_endpoint_confirmation',options.cellDampingRatio,0,0);
     end
    end
   end
  end
 case 'structural_notch_selected_trace'
  for seed=[0 1729]
   q=physical_case(base,2,50,90,seed);
   q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
   add_config(q,'structural','selected_finite_cell_full_native_traces',options.cellDampingRatio,0,0);
  end
 case 'structural_feedback_filter'
  assert(getSimulinkBlockHandle([mdl '/Force controller/Coil PI and PID/Normal force PI and PID/Optional feedback rolloff'])~=-1, ...
   'Wait for the common native feedback-only filter source.');
  for cutoff=[500 800 1000]
   for kp=[.01 .03]
    for lead=[.0003 .0004]
     for seed=[0 1729]
      q=physical_case(base,2,50,90,seed);
      q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
      q.inertial.feedbackFilter.cutoff_Hz=cutoff;
      q.inertial.forceKp=kp;q.inertial.forceKi=20;q.inertial.forceKd=0;
      q.inertial.guideVelocityPrediction_s=lead;
      q.inertial.currentNotch.enabled=false;
      add_config(q,'structural','feedback_only_native_rolloff_no_current_notch_search',options.cellDampingRatio,0,0);
     end
    end
   end
  end
 case 'structural_feedback_notch'
  for cutoff=[500 800 1000]
   for depth=[.01 .03]
    for seed=[0 29 1729]
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
     q.inertial.feedbackFilter.cutoff_Hz=cutoff;
     q.inertial.currentNotch.enabled=true;
     q.inertial.currentNotch.zeroDamping=depth;
     q.inertial.currentNotch.frequency_Hz=sqrt(options.cellStiffness_N_m/q.mechanics.mB)/(2*pi);
     q.inertial.currentNotch.poleDamping=.5;
     add_config(q,'structural','feedback_only_rolloff_with_mild_total_current_notch',options.cellDampingRatio,0,0);
    end
   end
  end
 case 'structural_feedback_gain'
  for cutoff=[500 800 1000]
   for gains=[.01 5;.005 1;0 1].'
    for seed=[0 29 1729]
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
     q.inertial.feedbackFilter.cutoff_Hz=cutoff;
     q.inertial.forceKp=gains(1);q.inertial.forceKi=gains(2);q.inertial.forceKd=0;
     add_config(q,'structural','feedback_only_native_rolloff_gain_search',options.cellDampingRatio,0,0);
    end
   end
  end
 case 'structural_feedback_tune'
  for filterDamping=[1/sqrt(2) 1]
   for gains=[.03 5;.03 10;.06 20;.1 20].'
    for seed=options.seeds
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
     q.inertial.feedbackFilter.damping=filterDamping;
     q.inertial.forceKp=gains(1);q.inertial.forceKi=gains(2);q.inertial.forceKd=0;
     add_config(q,'structural','feedback_only_rolloff_margin_tuning',options.cellDampingRatio,0,0);
    end
   end
  end
 case 'structural_feedback_cell_rates'
  identifiedFrequency=base.inertial.currentNotch.frequency_Hz;
  for cellRate=[20e6 40e6 64e6 92e6 110e6]
   for seed=options.seeds
    q=physical_case(base,2,50,90,seed);
    q=mechanical_case(q,'structural',cellRate,0,options.cellDampingRatio,catalog);
    q.inertial.currentNotch.frequency_Hz=identifiedFrequency;
    add_config(q,'structural','fixed_controller_actual_structural_stiffness_sensitivity',options.cellDampingRatio,0,0);
   end
  end
 case 'structural_cell_frequency_sensitivity'
  for frequencyScale=[.7 .8 .9 1.1 1.2 1.3]
   for mass=[2 .1]
    if mass==2,seeds=options.seeds;else,seeds=[0 29];end
    for seed=seeds
     q=physical_case(base,mass,50,90,seed);
     actualRate=options.cellStiffness_N_m*frequencyScale^2;
     q=mechanical_case(q,'structural',actualRate,0,options.cellDampingRatio,catalog);
     add_config(q,'structural','fixed_controller_actual_loaded_cell_frequency_sensitivity',options.cellDampingRatio,0,0);
    end
   end
  end
 case 'structural_cell_rate_pilot'
  for actualRate=options.cellRates_N_m
   for mass=[2 .1]
    if mass==2,seeds=options.seeds;else,seeds=[0 29];end
    for seed=seeds
     q=physical_case(base,mass,50,90,seed);
     q=mechanical_case(q,'structural',actualRate,0,options.cellDampingRatio,catalog);
     add_config(q,'structural','measured_drag_geometry_fixed_controller_actual_cell_rate',options.cellDampingRatio,0,0);
    end
   end
  end
 case 'structural_feedback_selected_trace'
  for seed=options.seeds
   q=physical_case(base,2,50,90,seed);
   q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
   add_config(q,'structural','feedback_only_selected_native_traces',options.cellDampingRatio,0,0);
  end
 case 'structural_feedback_endpoints'
  for mass=[.1 2]
   for force=[5 50]
    for angle=[0 90]
     for seed=[0 1729]
      q=physical_case(base,mass,force,angle,seed);
      q=mechanical_case(q,'structural',options.cellStiffness_N_m,0,options.cellDampingRatio,catalog);
      add_config(q,'structural','feedback_only_finite_cell_endpoint_confirmation',options.cellDampingRatio,0,0);
     end
    end
   end
  end
 case {'topology_screen','topology_tune'}
  if strcmp(phase,'topology_screen')
   gains=[.03 20 0;.01 5 0;0 1 0];
  else
   gains=[.03 5 .00003;.01 3 .0001;0 1 .0002;.03 5 .001];
  end
  for topology={'lower_spring','upper_spring','both_springs'}
   for damping=[0 15 60]
    for j=1:size(gains,1)
     for seed=[0 1729]
      q=physical_case(base,2,50,90,seed);
      q=mechanical_case(q,topology{1},4,damping,0,catalog);
      q.inertial.forceKp=gains(j,1);q.inertial.forceKi=gains(j,2);q.inertial.forceKd=gains(j,3);
      if gains(j,3)==.001,q.inertial.derivativeFilter=.003;end
      add_config(q,topology{1},'topology_equal_controller_search',0,4,4);
     end
    end
   end
  end
 case 'lower_spring_gain_pilot'
  for rate=options.springRates
   for damping=options.springDamping_Ns_m
    for kp=[.1 .3 1]
     for ki=[20 50 100]
      for seed=options.seeds
       q=physical_case(base,2,50,90,seed);
       q=mechanical_case(q,'lower_spring',rate,damping,0,catalog);
       q.inertial.forceKp=kp;q.inertial.forceKi=ki;q.inertial.forceKd=0;
       add_config(q,'lower_spring','compact_lower_spring_high_gain_PI_tuning',0,rate,0);
      end
     end
    end
   end
  end
 case 'lower_spring_pid_pilot'
  gains=[.3 20 .001;1 20 .001;1 20 .003];
  for rate=options.springRates
   for damping=options.springDamping_Ns_m
    for gainIndex=1:size(gains,1)
     for seed=options.seeds
      q=physical_case(base,2,50,90,seed);
      q=mechanical_case(q,'lower_spring',rate,damping,0,catalog);
      q.inertial.forceKp=gains(gainIndex,1);q.inertial.forceKi=gains(gainIndex,2);
      q.inertial.forceKd=gains(gainIndex,3);
      add_config(q,'lower_spring','compact_lower_spring_filtered_PID_tuning',0,rate,0);
     end
    end
   end
  end
 case 'lower_spring_inverse_pilot'
  assert(isfield(base.inertial,'lowerLoadInverse'),'Build the private native inverse-FF source first.');
  gains=[.3 20 0;1 20 0;.3 20 .001];
  for rate=options.springRates
   for damping=options.springDamping_Ns_m
    for gainIndex=1:size(gains,1)
     for cutoff=options.inverseCutoffs_Hz
      for seed=options.seeds
       q=physical_case(base,2,50,90,seed);
       q=mechanical_case(q,'lower_spring',rate,damping,0,catalog);
       q.inertial.forceKp=gains(gainIndex,1);q.inertial.forceKi=gains(gainIndex,2);
       q.inertial.forceKd=gains(gainIndex,3);
       q.inertial.lowerLoadInverse.cutoff_Hz=cutoff;
       add_config(q,'lower_spring','causal_measured_lower_load_proper_inverse_feedforward',0,rate,0);
      end
     end
    end
   end
  end
 case 'lower_spring_inverse_mechanics'
  assert(isfield(base.inertial,'lowerLoadInverse'),'Build the private native inverse-FF source first.');
  for rate=options.springRates
   for damping=options.springDamping_Ns_m
    for cutoff=options.inverseCutoffs_Hz
     for seed=options.seeds
      q=physical_case(base,2,50,90,seed);
      q=mechanical_case(q,'lower_spring',rate,damping,0,catalog);
      q.inertial.forceKp=.3;q.inertial.forceKi=20;q.inertial.forceKd=0;
      if damping<60,q.inertial.forceKd=.001;end
      q.inertial.lowerLoadInverse.cutoff_Hz=cutoff;
      add_config(q,'lower_spring','proper_inverse_FF_rate_damping_and_active_damping_sensitivity',0,rate,0);
     end
    end
   end
  end
 case 'lower_spring_selected'
  for mass=[.1 2]
   for force=[5 50]
    for angle=[0 90]
     for seed=options.seeds
      q=physical_case(base,mass,force,angle,seed);
      q=mechanical_case(q,'lower_spring',options.selectedSpringRate,options.selectedSpringDamping_Ns_m,0,catalog);
      add_config(q,'lower_spring','selected_lower_spring_endpoint_confirmation',0,options.selectedSpringRate,0);
     end
    end
   end
  end
 case {'lower_spring_repeated','lower_spring_refinement'}
  if strcmp(phase,'lower_spring_refinement'),steps=options.solverSteps_s;else,steps=options.step_s;end
  for step=steps
   for seed=options.seeds
    q=physical_case(base,2,50,90,seed);
    q.sim.step=step;
    q=mechanical_case(q,'lower_spring',options.selectedSpringRate,options.selectedSpringDamping_Ns_m,0,catalog);
    add_config(q,'lower_spring',phase,0,options.selectedSpringRate,0);
   end
  end
 case 'lower_spring_identified_sensitivity'
  for stiffnessScale=options.inverseStiffnessScales
   for dampingScale=options.inverseDampingScales
    for seed=options.seeds
     q=physical_case(base,2,50,90,seed);
     q=mechanical_case(q,'lower_spring',options.selectedSpringRate,options.selectedSpringDamping_Ns_m,0,catalog);
     add_config(q,'lower_spring','fixed_actual_mechanics_inverse_identification_sensitivity',0,options.selectedSpringRate,0);
     q=configs(end).p;
     identifiedOptions=options;
     identifiedOptions.lowerInverseStiffnessScale=stiffnessScale;
     identifiedOptions.lowerInverseDampingScale=dampingScale;
     q=lower_inverse_prepare(q,identifiedOptions);
     configs(end).p=q;
    end
   end
  end
 case 'lower_spring_actual_sensitivity'
  for stiffnessScale=options.actualStiffnessScales
   for dampingScale=options.actualDampingScales
    for mass=[.1 2]
     for seed=options.seeds
      q=physical_case(base,mass,50,90,seed);
      q=mechanical_case(q,'lower_spring',options.selectedSpringRate,options.selectedSpringDamping_Ns_m,0,catalog);
      % Prepare the identified inverse using the nominal known component.
      % The subsequent physical change must never retune this controller.
      add_config(q,'lower_spring','fixed_identified_inverse_actual_spring_and_damper_sensitivity',0,options.selectedSpringRate,0);
      q=configs(end).p;
      q.mechanics.k1=q.mechanics.k1*stiffnessScale;
      q.mechanics.c1=q.mechanics.c1*dampingScale;
      q.springs.lower.rate_N_m=q.mechanics.k1/2;
      q.springs.lower.part='nominal compact-pair geometry with actual-rate sensitivity';
      q.springs.actualRateSensitivity=true;
      assert(q.inertial.lowerLoadInverse.identifiedStiffness_N_m==base.mechanics.k1 && ...
       q.inertial.lowerLoadInverse.identifiedDamping_Ns_m==base.mechanics.c1, ...
       'Actual-component sensitivity must keep nominal identified inverse coefficients.');
      configs(end).p=q;
     end
    end
   end
  end
 case 'lower_spring_actual_parity'
  for mode={'normal','accelerator'}
   q=physical_case(base,2,50,90,1729);
   q=mechanical_case(q,'lower_spring',options.selectedSpringRate,options.selectedSpringDamping_Ns_m,0,catalog);
   add_config(q,'lower_spring','normal_accelerator_full_native_changed_mechanics_identity',0,options.selectedSpringRate,0);
   q=configs(end).p;
   q.mechanics.k1=q.mechanics.k1*.8;q.mechanics.c1=q.mechanics.c1*.8;
   q.springs.lower.rate_N_m=q.mechanics.k1/2;
   q.springs.lower.part='nominal compact-pair geometry with actual-rate sensitivity';
   q.springs.actualRateSensitivity=true;
   configs(end).p=q;configs(end).simulationMode=mode{1};
  end
 case 'spring_rate_screen'
  for topology={'lower_spring','upper_spring','both_springs'}
   for rate=1:4
    for damping=[0 5 15 30 60]
     for seed=[0 1729]
      q=physical_case(base,2,50,90,seed);
      q=mechanical_case(q,topology{1},rate,damping,0,catalog);
      add_config(q,topology{1},'compact_guided_pair_rate_damping_grid',0,rate,rate);
     end
    end
   end
  end
 otherwise,error('Unknown phase: %s',phase);
end
if numel(configs)>options.caseLimit,configs=configs(1:options.caseLimit);end
report=struct('date','2026-09-30','phase',phase,'status','running','recipeVersion','inertial_mechanics_v6_geometry', ...
 'sourceModel',sourceModel,'sourceModelSHA256',sha256(sourceModel),'privateFolder',workerFolder, ...
 'candidateFile',options.candidateFile,'candidateSHA256',sha256(options.candidateFile), ...
 'helperSources',helperSources,'recipeSHA256',sha256([mfilename('fullpath') '.m']), ...
 'catalogueFile',catalogFile,'catalogueSHA256',catalogHash,'catalogue',catalog,'options',options, ...
 'plannedCases',numel(configs),'executedCases',0,'setupRejections',0,'rows',struct([]), ...
 'metric','All native recorded surface-normal force samples, including startup; +/-0.5 N, completion, contact retained, no travel stop, no overcurrent.', ...
 'structuralPolicy','Cell stiffness represents the B-S sensing structure, not an optional suspension spring. No spring catalogue packing screen applies to structural-cell cases. Structural damping ratios are sensitivity assumptions, not manufacturer specifications.', ...
 'springPolicy','Two guided opposed springs per optional compliant link. Common preload is post-screened from exactly recorded link deflection extrema. Both combined free and installed lengths must be <=100 mm.', ...
 'controllerPolicy','Measured S acceleration and tared multi-axis B-S force reconstruct contact force. PI/PID gains may be tuned per topology; force and acceleration phase remain matched. No bump preview or contact-force truth enters the controller.', ...
 'notchPolicy','When enabled, the common native v3 model uses a prewarped bilinear total-current notch before saturation with equilibrium state initialization. Mechanical resonance frequency is an identified design parameter; frequency-error cases leave actual stiffness unchanged.', ...
 'feedbackFilterPolicy','When enabled, a common native scalar second-order prewarped bilinear filter rolls off only the summed P/I/D correction. Direct measured-motion, geometry and guide-friction feedforward bypass this feedback-only filter.', ...
 'geometryPolicy','The measured_drag_ratio mode uses matched reconstructed contact forces and the causal estimated surface direction to infer the tangential-to-normal drag ratio. A guarded and bounded EMA supplies geometry feedforward without the direct lateral-force amplitude feedback; the true friction coefficient stays in the physical plant only.', ...
 'lowerInversePolicy','When present and enabled, the private native branch adds mB*s^2/(identified k1+identified c1*s) times the desired axial reaction deviation minus known S times measured S acceleration. A proper two-pole rolloff limits derivative effort. Identified coefficients are stored separately from actual mechanics. This is causal measured-signal feedforward, not bump-profile preview.', ...
 'feedforwardLimit','The candidate acceleration-current feedforward assumes B shares S acceleration, and its recentering coordinate is S-A rather than actuator gap when B-S is compliant. Tuning tests preserve those causal assumptions; poor compliant performance must be interpreted with these controller limits.', ...
 'scope','Finite axial mechanics comparisons at prescribed tangential scan. The model omits lateral cell flexibility, distributed sanding-pad contact and manufacturer structural damping.', ...
 'matlabVersion',version,'solver',get_param(mdl,'Solver'),'simulationMode',options.simulationMode,'baseParameters',base);
rows=struct([]);started=tic;lastCompileKey='';worst=[];worstRow=[];worstScore=-inf;
best=[];bestRow=[];bestScore=inf;
normalReference=[];
for j=1:numel(configs)
 q=configs(j).p;
 mode=options.simulationMode;
 if ~isempty(configs(j).simulationMode),mode=configs(j).simulationMode;end
 if ~strcmp(get_param(mdl,'SimulationMode'),mode)
  set_param(mdl,'FastRestart','off');set_param(mdl,'SimulationMode',mode);lastCompileKey='';
 end
 if isfield(q.inertial,'currentNotch') && q.inertial.forceKd>0,q.coil.strategy='pid';end
 key=sprintf('%d/%d/%.12g/%.12g/%s',q.mechanics.lowerRigid,q.mechanics.upperRigid,q.sim.step,q.inertial.derivativeFilter,q.coil.strategy);
 if ~options.fastRestart
  set_param(mdl,'FastRestart','off');
 elseif ~strcmp(key,lastCompileKey)
  set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');lastCompileKey=key;
 end
 r=[];exception='';timer=tic;
 try
  quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  assert(isfield(r,'normalContact') && size(r.normalContact.data,2)==8);
  assert(isfield(r,'accelerometer') && size(r.accelerometer.data,2)==6);
  assert(isequal(r.time,r.normalContact.time,r.accelerometer.time),'Native log sample grids differ.');
  if isfield(r,'measuredDrag'),assert(isequal(r.time,r.measuredDrag.time) && size(r.measuredDrag.data,2)==3);end
  if isfield(q.inertial,'lowerLoadInverse')
   assert(isfield(r,'lowerInverse') && isequal(r.time,r.lowerInverse.time) && size(r.lowerInverse.data,2)==4);
   audit=r.lowerInverse.data;
   assert(max(abs(audit(:,3)-audit(:,1)+audit(:,2)))<1e-10,'Native desired lower-load identity failed.');
   if ~q.inertial.lowerLoadInverse.enabled,assert(~any(audit(:,4)),'Disabled inverse FF must supply zero current.');end
   if any(strcmp(phase,{'lower_spring_actual_sensitivity','lower_spring_actual_parity'}))
    requested=q.inertial.lowerLoadInverse;actual=r.parameters.inertial.lowerLoadInverse;
    assert(isequal(requested.identifiedStiffness_N_m,actual.identifiedStiffness_N_m) && ...
     isequal(requested.identifiedDamping_Ns_m,actual.identifiedDamping_Ns_m) && ...
     isequal(requested.identifiedMassB_kg,actual.identifiedMassB_kg) && ...
     isequal(requested.identifiedForceConstant_N_A,actual.identifiedForceConstant_N_A) && ...
     isequal(requested.numerator,actual.numerator) && isequal(requested.denominator,actual.denominator), ...
     'Coefficient preparation retuned a deliberately frozen identified inverse.');
   end
  end
 catch failure
  exception=getReport(failure,'extended','hyperlinks','off');r=[];
 end
 row=measure(q,r,j,configs(j),toc(timer),exception);
 row.simulationMode=mode;
 if strcmp(phase,'lower_spring_actual_parity') && ~isempty(r)
  if j==1,normalReference=r;else,report.nativeParity=native_history_proof(normalReference,r);end
 end
 if ~isempty(r) && (options.keepAllNative || (options.keepFailedNative && ~row.jointPassed))
  row.nativeTraceFile=fullfile(workerFolder,'traces',sprintf('case_%04d.mat',j));
  save(row.nativeTraceFile,'r','row','-v7');
 end
 rows=append(rows,row);
 if ~isempty(r)
  if row.peakNormalForceError_N>worstScore
   worst=r;worstRow=row;worstScore=row.peakNormalForceError_N;
  end
  if row.completed && row.peakNormalForceError_N<bestScore
   best=r;bestRow=row;bestScore=row.peakNormalForceError_N;
  end
  if j==1
   save(fullfile(outputFolder,[stem '_first_native_trace.mat']),'r','row','-v7');
  end
 end
 clear r;
 report.rows=rows;report.executedCases=sum([rows.executed]);report.setupRejections=sum(~[rows.executed]);
 report.passedCases=sum([rows.physicalPassed]);report.elapsedSeconds=toc(started);
 if mod(j,4)==0 || j==1 || j==numel(configs)
  checkpoint('running');
  fprintf('INERTIAL_MECHANICS %s %d/%d: %s S%.2g F%.0f angle%.0f k%.6g/%.6g c%.4g/%.4g gains%.3g/%.3g/%.3g seed%d: %.9g N, pass%d, duration%.3g s, elapsed%.1f s\n', ...
   phase,j,numel(configs),row.topology,row.massS_kg,row.referenceForce_N,row.angle_deg, ...
   row.k1_N_m,row.k2_N_m,row.c1_Ns_m,row.c2_Ns_m,row.forceKp,row.forceKi,row.forceKd, ...
   row.seed,row.peakNormalForceError_N,row.physicalPassed,row.actualDuration_s,toc(started));
 end
 if mod(j,8)==0,Simulink.sdi.clear;end
 if isfile(fullfile(workerFolder,'stop_after_case'))
  report.status='stopped_by_coordinator';break;
 end
end
set_param(mdl,'FastRestart','off');
report.privateSourceUnmodified=strcmp(sha256(fullfile(workerFolder,[mdl '.slx'])),report.sourceModelSHA256);
report.upstreamSourceUnmodified=strcmp(sha256(sourceModel),report.sourceModelSHA256);
assert(report.privateSourceUnmodified && report.upstreamSourceUnmodified,'Frozen model changed during this study.');
if ~strcmp(report.status,'stopped_by_coordinator'),report.status='complete';end
if ~isempty(worst)
 report.worstCase=worstRow;save(fullfile(outputFolder,[stem '_worst_native_trace.mat']),'worst','worstRow','-v7');
end
if ~isempty(best)
 report.bestCase=bestRow;save(fullfile(outputFolder,[stem '_best_native_trace.mat']),'best','bestRow','-v7');
end
checkpoint(report.status);
if strcmp(phase,'lower_spring_actual_parity')
 assert(isfield(report,'nativeParity') && report.nativeParity.bitwiseEqual, ...
  'Normal and Accelerator full native histories differ; both native traces were retained.');
end
fprintf('INERTIAL_MECHANICS_COMPLETE %s: %d executed, %d setup rejects, %d passes, elapsed %.1f s.\n', ...
 phase,report.executedCases,report.setupRejections,report.passedCases,report.elapsedSeconds);

 function add_config(q,topology,purpose,zeta,lowerIndex,upperIndex)
  if isfield(q.inertial,'lowerLoadInverse')
   q=lower_inverse_prepare(q,options);
  elseif options.lowerInverseEnabled
   error('Wait for the private native lower-load inverse-FF source.');
  end
  configs=append(configs,struct('p',q,'topology',topology,'purpose',purpose,'simulationMode','', ...
   'structuralDampingRatio',zeta,'lowerCatalogueIndex',lowerIndex,'upperCatalogueIndex',upperIndex));
 end
 function checkpoint(status)
  report.status=status;write_json(resultFile,report);
  if ~isempty(rows),writetable(struct2table(rows,'AsArray',true),fullfile(outputFolder,[stem '_cases.csv']));end
  save(fullfile(outputFolder,[stem '_checkpoint.mat']),'report','configs','rows','-v7');
 end
end

function p=physical_case(p,mass,force,angle,seed)
p.mechanics.mS=mass;p.mechanics.mB=.6;
p.sensor.distalMass_kg=mass;p.sensor.distalMass_model_kg=mass;
p.force.reference=force;p.mechanics.angle=angle*pi/180;p.contact.mu=.3;
p.sensor.seed=max(seed,7);p.sensor.noiseRms=.02;p.sensor.strokeNoiseRms=1e-6;
p.contact.lateralNoiseRms_N=.02;
if seed==0,p.sensor.noiseRms=0;p.sensor.strokeNoiseRms=0;p.contact.lateralNoiseRms_N=0;p.accel.noiseRms=0;end
end

function p=mechanical_case(p,topology,rate,damping,zeta,catalog)
p.mechanics.lowerRigid=true;p.mechanics.upperRigid=true;p.mechanics.c1=0;p.mechanics.c2=0;
switch topology
 case 'rigid'
  if isfield(p,'springs'),p=rmfield(p,'springs');end
 case 'structural'
  p.mechanics.lowerRigid=false;p.mechanics.k1=rate;
  p.mechanics.c1=2*zeta*sqrt(rate*p.mechanics.mB);
  if isfield(p,'springs'),p=rmfield(p,'springs');end
 case {'lower_spring','upper_spring','both_springs'}
  p.mechanics.lowerRigid=strcmp(topology,'upper_spring');
  p.mechanics.upperRigid=strcmp(topology,'lower_spring');
  p.mechanics.k1=2*catalog.springs(rate).rate_N_m;p.mechanics.k2=p.mechanics.k1;
  if ~p.mechanics.lowerRigid,p.mechanics.c1=damping;end
  if ~p.mechanics.upperRigid,p.mechanics.c2=damping;end
  p.springs.lower=catalog.springs(rate);p.springs.upper=catalog.springs(rate);
  if ~isfield(p.springs,'commonPreload_N'),p.springs.commonPreload_N=10;end
  p.springs.maxCombinedLength_m=.1;
  p.springs.workingClearance_m=.0005;p.springs.unseatingClearance_m=.00025;
 otherwise,error('Unknown topology');
end
end

function row=measure(q,r,index,c,elapsed,exception)
row=struct('caseIndex',index,'topology',c.topology,'purpose',c.purpose, ...
 'massS_kg',q.mechanics.mS,'massB_kg',q.mechanics.mB,'referenceForce_N',q.force.reference, ...
 'angle_deg',q.mechanics.angle*180/pi,'frictionCoefficient',q.contact.mu, ...
 'k1_N_m',q.mechanics.k1,'k2_N_m',q.mechanics.k2,'c1_Ns_m',q.mechanics.c1,'c2_Ns_m',q.mechanics.c2, ...
 'lowerRigid',q.mechanics.lowerRigid,'upperRigid',q.mechanics.upperRigid, ...
 'structuralDampingRatio',c.structuralDampingRatio, ...
 'lowerCatalogueIndex',c.lowerCatalogueIndex,'upperCatalogueIndex',c.upperCatalogueIndex, ...
 'forceKp',q.inertial.forceKp,'forceKi',q.inertial.forceKi,'forceKd',q.inertial.forceKd, ...
 'derivativeFilter_s',q.inertial.derivativeFilter, ...
 'accelerationFeedforwardScale',q.inertial.accelerationFeedforwardScale, ...
 'guideFrictionFeedforwardScale',q.inertial.guideFrictionFeedforwardScale, ...
 'forceNoiseRms_N',q.sensor.noiseRms,'accelNoiseRms_m_s2',q.accel.noiseRms, ...
 'forceFilterTime_s',q.sensor.filterTime,'accelFilterTime_s',q.accel.filterTime, ...
 'accelQuantum_m_s2',q.accel.quantum,'accelRange_g',q.accel.range_g, ...
 'seed',q.sensor.seed,'step_s',q.sim.step,'executed',~isempty(r),'physicalPassed',false, ...
 'packingPassed',false,'jointPassed',false,'completed',false,'contactLoss',false,'strokeStop',false, ...
 'overCurrent',false,'peakNormalForceError_N',NaN,'rmsNormalForceError_N',NaN, ...
 'minNormalForce_N',NaN,'maxNormalForce_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN, ...
 'peakVoltage_V',NaN,'currentSaturationFraction',NaN,'voltageSaturationFraction',NaN, ...
 'minTravel_mm',NaN,'maxTravel_mm',NaN,'peakLowerDeflection_mm',NaN,'peakUpperDeflection_mm',NaN, ...
 'peakLowerDynamicForce_N',NaN,'peakUpperDynamicForce_N',NaN,'maxHeadGapVsActuatorGap_mm',NaN, ...
 'maxRobotVsFlangeAMotion_mm',NaN, ...
 'maxNormalEstimateError_N',NaN,'normalBalanceResidual_N',NaN,'actualDuration_s',NaN, ...
 'downstreamForceBalanceResidual_N',NaN,'actuatedForceBalanceResidual_N',NaN, ...
 'notchEnabled',false,'notchFrequency_Hz',NaN,'notchZeroDamping',NaN,'notchPoleDamping',NaN, ...
 'feedbackFilterEnabled',false,'feedbackFilterCutoff_Hz',NaN,'feedbackFilterDamping',NaN, ...
 'geometryMode','','dragRatioFilter_s',NaN,'dragRatioMinForce_N',NaN,'accelerationPrediction_s',0, ...
 'accelerationFeedforwardFilter_s',0, ...
 'lowerInverseEnabled',false,'lowerInverseCutoff_Hz',NaN,'lowerInverseFilterDamping',NaN, ...
 'lowerInverseScale',0,'identifiedLowerStiffness_N_m',NaN,'identifiedLowerDamping_Ns_m',NaN, ...
 'identifiedInverseMassB_kg',NaN, ...
 'guideVelocityPrediction_s',0, ...
 'combinedFreeLengthMax_mm',NaN,'combinedInstalledLengthMax_mm',NaN, ...
 'feasibleCommonPreloadMin_N',NaN,'feasibleCommonPreloadMax_N',NaN, ...
 'configuredCommonPreload_N',NaN,'configuredCommonPreloadPassed',false, ...
 'configuredPreloadRequired',false,'executionSeconds',elapsed, ...
 'failureReason','','executionError',exception,'nativeTraceFile','');
if isfield(q.inertial,'currentNotch')
 row.notchEnabled=q.inertial.currentNotch.enabled;row.notchFrequency_Hz=q.inertial.currentNotch.frequency_Hz;
 row.notchZeroDamping=q.inertial.currentNotch.zeroDamping;row.notchPoleDamping=q.inertial.currentNotch.poleDamping;
end
if isfield(q.inertial,'feedbackFilter')
 row.feedbackFilterEnabled=q.inertial.feedbackFilter.enabled;
 row.feedbackFilterCutoff_Hz=q.inertial.feedbackFilter.cutoff_Hz;
 row.feedbackFilterDamping=q.inertial.feedbackFilter.damping;
end
if isfield(q.contact,'geometryMode'),row.geometryMode=q.contact.geometryMode;end
if isfield(q.contact,'dragRatioFilter_s'),row.dragRatioFilter_s=q.contact.dragRatioFilter_s;end
if isfield(q.contact,'dragRatioMinForce_N'),row.dragRatioMinForce_N=q.contact.dragRatioMinForce_N;end
if isfield(q.inertial,'accelerationPrediction_s'),row.accelerationPrediction_s=q.inertial.accelerationPrediction_s;end
if isfield(q.inertial,'accelerationFeedforwardFilter_s')
 row.accelerationFeedforwardFilter_s=q.inertial.accelerationFeedforwardFilter_s;
end
if isfield(q.inertial,'lowerLoadInverse')
 c=q.inertial.lowerLoadInverse;row.lowerInverseEnabled=c.enabled;
 row.lowerInverseCutoff_Hz=c.cutoff_Hz;row.lowerInverseFilterDamping=c.filterDamping;
 row.lowerInverseScale=c.scale;row.identifiedLowerStiffness_N_m=c.identifiedStiffness_N_m;
 row.identifiedLowerDamping_Ns_m=c.identifiedDamping_Ns_m;row.identifiedInverseMassB_kg=c.identifiedMassB_kg;
end
if isfield(q.inertial,'guideVelocityPrediction_s'),row.guideVelocityPrediction_s=q.inertial.guideVelocityPrediction_s;end
if isfield(q,'springs')
 row.configuredCommonPreload_N=q.springs.commonPreload_N;
 row.configuredPreloadRequired=isfield(q.springs,'requireConfiguredPreload') && q.springs.requireConfiguredPreload;
end
if isempty(r),row.failureReason='setup rejection';return;end
q=r.parameters;
t=r.time;d=r.data;n=r.normalContact.data;error=n(:,1)-q.force.reference;
row.peakNormalForceError_N=max(abs(error));row.rmsNormalForceError_N=sqrt(mean(error.^2));
row.minNormalForce_N=min(n(:,1));row.maxNormalForce_N=max(n(:,1));
row.completed=t(end)>=q.sim.stopTime-q.sim.step/2;
row.contactLoss=any(d(:,18)<.5);row.strokeStop=any(d(:,21)>.5);
row.peakCurrent_A=max(abs(d(:,14)));row.rmsCurrent_A=sqrt(mean(d(:,14).^2));
row.peakVoltage_V=max(abs(d(:,15)));row.currentSaturationFraction=mean(d(:,22)>.5);
row.voltageSaturationFraction=mean(d(:,23)>.5);
row.overCurrent=row.peakCurrent_A>q.coil.peakCurrent*(1+1e-4);
row.minTravel_mm=1000*min(d(:,2));row.maxTravel_mm=1000*max(d(:,2));
lower=d(:,7)-d(:,6);upper=d(:,5)-d(:,26);
row.peakLowerDeflection_mm=1000*max(abs(lower));row.peakUpperDeflection_mm=1000*max(abs(upper));
row.peakLowerDynamicForce_N=max(abs(d(:,24)-q.d.q10));
row.peakUpperDynamicForce_N=max(abs(d(:,25)-q.d.q20));
% Actual approved position sensor is S relative to A, even with an upper spring.
headGap=q.mechanics.midTravel+d(:,5)-d(:,7);
row.maxHeadGapVsActuatorGap_mm=1000*max(abs(headGap-d(:,2)));
row.maxRobotVsFlangeAMotion_mm=1000*max(abs(d(:,26)-d(:,5)));
row.maxNormalEstimateError_N=max(abs(n(:,4)-n(:,1)));
slope=d(:,29)/q.surface.scanSpeed;
predicted=d(:,1).*sqrt(1+slope.^2)./(1-q.contact.mu*slope);
row.normalBalanceResidual_N=max(abs(predicted-n(:,1)));
assert(row.normalBalanceResidual_N<1e-8,'Native contact-force vector balance failed.');
contact=d(:,18)>.5;
row.downstreamForceBalanceResidual_N=max(abs(d(contact,1)-d(contact,24)-q.mechanics.mS*(q.d.g+d(contact,30))));
row.actuatedForceBalanceResidual_N=max(abs(d(contact,1)-d(contact,13)- ...
 q.mechanics.mB*(q.d.g+d(contact,12))-q.mechanics.mS*(q.d.g+d(contact,30))));
assert(row.downstreamForceBalanceResidual_N<1e-8 && row.actuatedForceBalanceResidual_N<1e-8, ...
 'Native axial free-body force balance failed.');
row.actualDuration_s=t(end);
row.physicalPassed=row.completed && row.peakNormalForceError_N<=q.force.tolerance && ...
 ~row.contactLoss && ~row.strokeStop && ~row.overCurrent;
if isfield(q,'springs')
 [row.packingPassed,row.feasibleCommonPreloadMin_N,row.feasibleCommonPreloadMax_N, ...
  row.combinedFreeLengthMax_mm,row.combinedInstalledLengthMax_mm,row.configuredCommonPreloadPassed]=packing(r);
else
 row.packingPassed=true;row.combinedFreeLengthMax_mm=0;row.combinedInstalledLengthMax_mm=0;
end
row.jointPassed=row.physicalPassed && row.packingPassed;
reasons={};
if ~row.completed,reasons{end+1}='incomplete';end
if row.contactLoss,reasons{end+1}='contact loss';end
if row.strokeStop,reasons{end+1}='travel stop';end
if row.overCurrent,reasons{end+1}='overcurrent';end
if row.peakNormalForceError_N>q.force.tolerance,reasons{end+1}='normal force error';end
if ~row.packingPassed,reasons{end+1}='spring packing or working range';end
if isempty(reasons),row.failureReason='pass';else,row.failureReason=strjoin(reasons,', ');end
end

function [passed,low,high,totalFree,totalInstalled,configuredPassed]=packing(r)
% Exact common-preload interval from each recorded link's extrema, dimensions
% and individual force envelope. No rerun or altered physical trajectory.
p=r.parameters;d=r.data;pack=p.springs;low=0;high=inf;totalFree=0;
links={'lower','upper'};rigid=[p.mechanics.lowerRigid p.mechanics.upperRigid];
delta={d(:,7)-d(:,6),d(:,5)-d(:,26)};offset=[p.d.q10,p.d.q20];
stiffness=[p.mechanics.k1,p.mechanics.k2];excursionPass=true;
for j=1:2
 if rigid(j),continue;end
 s=pack.(links{j});ks=s.rate_N_m;shift=offset(j)/stiffness(j)+delta{j};
 compressionOffset=max(abs(shift));
 low=max(low,ks*(pack.unseatingClearance_m+compressionOffset));
 upperCompression=min(s.freeLength_m-s.freeLengthTolerance_m-s.minWorkingLength_m-pack.workingClearance_m, ...
  s.forceEnvelope_N/ks);
 high=min(high,ks*(upperCompression-compressionOffset));
 excursionPass=excursionPass && (max(shift)-min(shift)<=s.maxCyclicStroke_m+1e-12);
 totalFree=totalFree+2000*(s.freeLength_m+s.freeLengthTolerance_m);
end
passed=low<=high && excursionPass && totalFree<=1000*pack.maxCombinedLength_m+1e-9;
configuredPassed=pack.commonPreload_N>=low-1e-9 && pack.commonPreload_N<=high+1e-9;
if isfinite(high) && high>=low,common=(low+high)/2;else,common=low;end
if isfield(pack,'requireConfiguredPreload') && pack.requireConfiguredPreload
 common=pack.commonPreload_N;passed=passed && configuredPassed;
end
totalInstalled=0;
for j=1:2
 if rigid(j),continue;end
 s=pack.(links{j});totalInstalled=totalInstalled+2000*(s.freeLength_m+s.freeLengthTolerance_m-common/s.rate_N_m);
end
passed=passed && totalInstalled<=1000*pack.maxCombinedLength_m+1e-9;
end

function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end
function proof=native_history_proof(reference,candidate)
assert(isequal(reference.time,candidate.time));
fields={'normalContact','accelerometer','motionObserver','commandPrediction', ...
 'forceVariation','measuredDrag','accelerationFeedforward','lowerInverse'};
proof=struct('bitwiseEqual',isequal(reference.data,candidate.data), ...
 'maximumDifference',max(abs(reference.data-candidate.data),[],'all'), ...
 'samples',numel(reference.time),'channels',size(reference.data,2),'differences',struct);
for j=1:numel(fields)
 field=fields{j};assert(isfield(reference,field) && isfield(candidate,field));
 assert(isequal(reference.(field).time,candidate.(field).time));
 difference=max(abs(reference.(field).data-candidate.(field).data),[],'all');
 proof.differences.(field)=difference;
 proof.maximumDifference=max(proof.maximumDifference,difference);
 proof.bitwiseEqual=proof.bitwiseEqual && isequal(reference.(field).data,candidate.(field).data);
 proof.channels=proof.channels+size(reference.(field).data,2);
end
assert(isequal(reference.robotGate.time,candidate.robotGate.time));
gateFields=fieldnames(reference.robotGate);
for j=1:numel(gateFields)
 field=gateFields{j};if strcmp(field,'time'),continue;end
 proof.bitwiseEqual=proof.bitwiseEqual && isequal(reference.robotGate.(field),candidate.robotGate.(field));
 proof.channels=proof.channels+size(reference.robotGate.(field),2);
end
end
function s=defaults(s,d)
names=fieldnames(d);for j=1:numel(names),if ~isfield(s,names{j}),s.(names{j})=d.(names{j});end,end
end
function write_json(path,value)
fid=fopen([path '.pending'],'w');assert(fid>0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);movefile([path '.pending'],path,'f');
end
function digest=sha256(path)
quote=char(39);escaped=strrep(path,quote,[quote char(34) quote char(34) quote]);
[status,out]=system(['sha256sum -- ' quote escaped quote]);assert(status==0);
digest=out(1:64);assert(~isempty(regexp(digest,'^[a-f0-9]{64}$','once')));
end
function finish(mdl,storage,record,threads)
try
 set_param(mdl,'FastRestart','off');close_system(mdl,0);
catch
end
try
 Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);
catch
end
try
 maxNumCompThreads(threads);
catch
end
end

function p=lower_inverse_prepare(p,options)
% Controller-only coefficient preparation; the physical plant remains native.
c=p.inertial.lowerLoadInverse;
c.enabled=options.lowerInverseEnabled && ~p.mechanics.lowerRigid;
c.scale=options.lowerInverseScale;
c.identifiedStiffness_N_m=p.mechanics.k1*options.lowerInverseStiffnessScale;
c.identifiedDamping_Ns_m=p.mechanics.c1*options.lowerInverseDampingScale;
c.identifiedMassB_kg=p.mechanics.mB*options.lowerInverseMassBScale;
assert(c.identifiedStiffness_N_m>0 && c.identifiedDamping_Ns_m>=0 && ...
 c.identifiedMassB_kg>0 && c.cutoff_Hz>0 && c.cutoff_Hz<.45/p.force.sampleTime && ...
 c.filterDamping>0 && isfinite(c.scale));
% Bilinear H(s)=(mB/Kf)*s^2/(k+c*s) times a proper two-pole rolloff.
% Prewarp the rolloff corner; no unknown/future trajectory is used here.
a=2/p.force.sampleTime;w=a*tan(pi*c.cutoff_Hz*p.force.sampleTime);
quadratic=[a*a+2*c.filterDamping*w*a+w*w,2*(w*w-a*a), ...
 a*a-2*c.filterDamping*w*a+w*w];
num=(c.identifiedMassB_kg/p.coil.forceConstant)*w*w*a*a*[1,-2,1];
k=c.identifiedStiffness_N_m;d=c.identifiedDamping_Ns_m;
if d>0
 num=conv(num,[1,1]);den=conv([d*a+k,-d*a+k],quadratic);
else
 den=k*quadratic;num=[num,0];den=[den,0]; % harmless zero pole, fixed state count
end
c.numerator=num/den(1);c.denominator=den/den(1);
assert(max(abs(roots(c.denominator)))<1+1e-12,'Inverse-FF rolloff poles must be stable.');
assert(abs(sum(c.numerator))<1e-9,'Inverse-FF must have zero DC force.');
p.inertial.lowerLoadInverse=c;
end

function manifest=build_lower_inverse_snapshot(sourceFolder,outputFolder,options)
% Private optional ordinary-block controller extension. Never edit upstream.
options=defaults(options,struct('candidateFile',fullfile(sourceFolder,'normal_contact_trial_candidate.mat'), ...
 'label','lower_inverse','lowerInverseEnabled',false,'lowerInverseCutoff_Hz',1000, ...
 'lowerInverseFilterDamping',1/sqrt(2),'lowerInverseScale',1, ...
 'lowerInverseStiffnessScale',1,'lowerInverseDampingScale',1,'lowerInverseMassBScale',1));
assert(isfolder(outputFolder));stem=['inertial_mechanics_' options.label '_builder_20260930'];
manifestPath=fullfile(outputFolder,[stem '_manifest.json']);assert(~isfile(manifestPath));
originalModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');originalHash=sha256(originalModel);
stateRoot=fullfile(getenv('HOME'),'.local','state','actuator-model');workerFolder=tempname(stateRoot);
for suffix={'','helpers','cache','code','sdi'},mkdir(fullfile(workerFolder,suffix{1}));end
copyfile(originalModel,fullfile(workerFolder,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile([mfilename('fullpath') '.m'],fullfile(workerFolder,'qualify_inertial_mechanics.m'));
addpath(fullfile(workerFolder,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator;
loaded=load(options.candidateFile,'p');p=loaded.p;
p.inertial.lowerLoadInverse=struct('enabled',false,'scale',1,'cutoff_Hz',options.lowerInverseCutoff_Hz, ...
 'filterDamping',options.lowerInverseFilterDamping);
p=lower_inverse_prepare(p,options);p=acf_prepare(p);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
oldThreads=maxNumCompThreads(2);
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(workerFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(workerFolder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,oldThreads));
physicalPath=[mdl '/Actuator and RABS mechanics'];
before=native_dialog_structure(physicalPath);
parent=[mdl '/Force controller/Coil PI and PID'];sys=[parent '/Lower spring inverse feedforward'];
assert(getSimulinkBlockHandle(sys)==-1);
add_block('simulink/Ports & Subsystems/Subsystem',sys,'Position',[180 545 720 665]);
delete_line(sys,'In1/1','Out1/1');delete_block([sys '/In1']);delete_block([sys '/Out1']);
add_block('simulink/Signal Routing/From',[sys '/Desired axial reaction deviation'], ...
 'GotoTag','inertial_axial_reference_deviation_N','Position',[20 40 265 70]);
add_block('simulink/Signal Routing/From',[sys '/Measured S acceleration'], ...
 'GotoTag','measured_S_acceleration_z','Position',[20 165 265 195]);
add_block('simulink/Math Operations/Gain',[sys '/Known downstream S inertia'], ...
 'Gain','p.sensor.distalMass_model_kg','Position',[320 150 550 210]);
add_block('simulink/Math Operations/Sum',[sys '/Desired lower load deviation'], ...
 'Inputs','+-','Position',[615 35 655 120]);
add_block('simulink/Discrete/Discrete Transfer Fcn',[sys '/Proper identified lower-link inverse'], ...
 'Numerator','p.inertial.lowerLoadInverse.numerator', ...
 'Denominator','p.inertial.lowerLoadInverse.denominator','InitialStates','zeros(3,1)', ...
 'SampleTime','p.force.sampleTime','Position',[730 40 1050 115]);
add_block('simulink/Math Operations/Gain',[sys '/Optional inverse feedforward scale'], ...
 'Gain','double(p.inertial.lowerLoadInverse.enabled)*p.inertial.lowerLoadInverse.scale', ...
 'Position',[1115 40 1435 115]);
add_block('simulink/Ports & Subsystems/Out1',[sys '/extra_current_A'], ...
 'Position',[1510 69 1540 83]);
add_line(sys,'Desired axial reaction deviation/1','Desired lower load deviation/1','autorouting','on');
add_line(sys,'Measured S acceleration/1','Known downstream S inertia/1','autorouting','on');
add_line(sys,'Known downstream S inertia/1','Desired lower load deviation/2','autorouting','on');
add_line(sys,'Desired lower load deviation/1','Proper identified lower-link inverse/1','autorouting','on');
add_line(sys,'Proper identified lower-link inverse/1','Optional inverse feedforward scale/1','autorouting','on');
add_line(sys,'Optional inverse feedforward scale/1','extra_current_A/1','autorouting','on');
add_block('simulink/Signal Routing/Mux',[sys '/Native inverse audit'], ...
 'Inputs','4','Position',[990 280 995 420]);
audit={'Desired axial reaction deviation/1','Known downstream S inertia/1', ...
 'Desired lower load deviation/1','Optional inverse feedforward scale/1'};
for j=1:numel(audit),add_line(sys,audit{j},sprintf('Native inverse audit/%d',j),'autorouting','on');end
add_block('simulink/Discrete/Zero-Order Hold',[sys '/Native audit sampling'], ...
 'SampleTime','p.sim.step','Position',[1120 320 1360 380]);
add_block('simulink/Sinks/To Workspace',[sys '/Full native inverse log'], ...
 'VariableName','lower_inverse_log','SaveFormat','Timeseries','MaxDataPoints','inf', ...
 'SampleTime','-1','Position',[1430 315 1685 385]);
add_line(sys,'Native inverse audit/1','Native audit sampling/1','autorouting','on');
add_line(sys,'Native audit sampling/1','Full native inverse log/1','autorouting','on');
set_param([parent '/PI PID and feedforward current'],'Inputs','++++++');
add_line(parent,'Lower spring inverse feedforward/1','PI PID and feedforward current/6','autorouting','on');
after=native_dialog_structure(physicalPath);
assert(isequal(before,after),'The inverse-FF builder changed the physical plant.');
mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);
acf_select_actuator(mdl,p.actuator.kind);
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);
assert(isempty(loops),'Optional inverse-FF created an algebraic loop.');
save_system(mdl,fullfile(workerFolder,[mdl '.slx']));
save(fullfile(workerFolder,'normal_contact_trial_candidate.mat'),'p','-v7');
helper=fullfile(workerFolder,'helpers','acf_run.m');contents=fileread(helper);
anchor='fprintf(''%s / %s: %s | peak force error';location=strfind(contents,anchor);assert(isscalar(location));
extra=['if any(strcmp(who(out),''lower_inverse_log''))' newline ...
 ' logged=out.get(''lower_inverse_log'');r.lowerInverse=struct(''time'',logged.Time(:),''data'',rows(logged,4));' newline ...
 'end' newline];
contents=[contents(1:location-1) extra contents(location:end)];
fid=fopen(helper,'w');assert(fid>0);fprintf(fid,'%s',contents);fclose(fid);
assert(strcmp(originalHash,sha256(originalModel)),'The upstream model changed.');
manifest=struct('date','2026-09-30','status','private_native_inverse_source_prepared_no_experiments', ...
 'sourceFolder',workerFolder,'sourceModelSHA256',sha256(fullfile(workerFolder,[mdl '.slx'])), ...
 'originalModel',originalModel,'originalModelSHA256',originalHash, ...
 'sourceCandidateSHA256',sha256(options.candidateFile),'candidateSHA256', ...
 sha256(fullfile(workerFolder,'normal_contact_trial_candidate.mat')), ...
 'builderRecipeSHA256',sha256([mfilename('fullpath') '.m']), ...
 'acfRunSHA256',sha256(helper),'physicalDialogStructureUnchanged',true,'algebraicLoops',0, ...
 'defaultInverseEnabled',p.inertial.lowerLoadInverse.enabled,'nativeExecutedCases',0, ...
 'contract','Extra current = (identified mB/Kf)*s^2/(identified k1+identified c1*s) times [desired Rz deviation-known S*measured az], with proper two-pole rolloff and optional zero scale. Desired force comes from the native measured-drag geometry. No future motion or true contact geometry is connected.', ...
 'logSchema',{{'desired_Rz_deviation_N','identified_S_inertia_N','desired_lower_load_deviation_N','inverse_current_A'}});
write_json(manifestPath,manifest);
write_json(fullfile(outputFolder,[stem '_physical_dialog_structure.json']),before);
fprintf('INERTIAL_MECHANICS_INVERSE_SOURCE %s\n',workerFolder);
end

function rows=native_dialog_structure(sys)
blocks=find_system(sys,'FollowLinks','on','LookUnderMasks','all', ...
 'MatchFilter',@Simulink.match.allVariants,'Type','Block');blocks=sort(blocks);rows=struct([]);
for j=1:numel(blocks)
 b=blocks{j};parameters=get_param(b,'DialogParameters');values=struct;
 if isstruct(parameters),names=sort(fieldnames(parameters));else,names={};end
 for k=1:numel(names),values.(names{k})=get_param(b,names{k});end
 rows=append(rows,struct('path',b,'blockType',get_param(b,'BlockType'),'parameters',values));
end
end

function manifest=freeze_lower_series(sourceFolder,outputFolder,options)
options=defaults(options,struct('checkpointFile',fullfile(outputFolder, ...
 'inertial_mechanics_lower_spring_inverse_pilot_20260930_inverse_d60_rate19p746_checkpoint.mat'), ...
 'caseIndex',8,'label','lower_series_v1','commonPreload_N',33.6));
loaded=load(options.checkpointFile,'configs','report');p=loaded.configs(options.caseIndex).p;
assert(p.mechanics.k1==19746 && p.mechanics.c1==60 && p.mechanics.upperRigid && ...
 p.inertial.forceKp==.3 && p.inertial.forceKi==20 && p.inertial.forceKd==0 && ...
 p.inertial.lowerLoadInverse.enabled && p.inertial.lowerLoadInverse.cutoff_Hz==1500);
p.sensor.seed=7;p.springs.commonPreload_N=options.commonPreload_N;
p.springs.sensingCell=struct('stiffness_N_m',64e6,'separateNativeState',false, ...
 'masslessSeriesApproximation',true,'structuralDampingQualified',false, ...
 'loadPath','B -> opposed guided spring pair parallel with damper -> joining frame -> multi-axis B-S load cell -> S', ...
 'loadCellBypass',false,'prototypeMountModeQualificationRequired',true);
p.inertial.lowerLoadInverse.identifiedForceConstant_N_A=p.coil.forceConstant;
stateRoot=fullfile(getenv('HOME'),'.local','state','actuator-model');
workerFolder=fullfile(stateRoot,'inertial-mechanics-selected-lower-series-20260930',options.label);
assert(~isfolder(workerFolder),'Frozen sources are immutable; choose a new version.');
for suffix={'','helpers','cache','code','sdi'},mkdir(fullfile(workerFolder,suffix{1}));end
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');sourceHash=sha256(sourceModel);
copyfile(sourceModel,fullfile(workerFolder,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile([mfilename('fullpath') '.m'],fullfile(workerFolder,'qualify_inertial_mechanics.m'));
% Recompute only controller polynomial coefficients from explicitly identified
% parameters. Actual mechanics remain separate, including during mismatch tests.
helper=fullfile(workerFolder,'helpers','acf_prepare.m');contents=fileread(helper);
anchor=[newline 'end' newline newline 'function n = ticks'];location=strfind(contents,anchor);assert(isscalar(location));
extra=[newline 'if isfield(p,''inertial'') && isfield(p.inertial,''lowerLoadInverse'')' newline ...
 ' c=p.inertial.lowerLoadInverse;a=2/p.force.sampleTime;w=a*tan(pi*c.cutoff_Hz*p.force.sampleTime);' newline ...
 ' quadratic=[a*a+2*c.filterDamping*w*a+w*w,2*(w*w-a*a),a*a-2*c.filterDamping*w*a+w*w];' newline ...
 ' num=(c.identifiedMassB_kg/c.identifiedForceConstant_N_A)*w*w*a*a*[1,-2,1];' newline ...
 ' k=c.identifiedStiffness_N_m;d=c.identifiedDamping_Ns_m;' newline ...
 ' if d>0,num=conv(num,[1,1]);den=conv([d*a+k,-d*a+k],quadratic);' newline ...
 ' else,den=k*quadratic;num=[num,0];den=[den,0];end' newline ...
 ' c.numerator=num/den(1);c.denominator=den/den(1);p.inertial.lowerLoadInverse=c;' newline ...
 'end' newline];
contents=[contents(1:location-1) extra contents(location:end)];
fid=fopen(helper,'w');assert(fid>0);fprintf(fid,'%s',contents);fclose(fid);
addpath(fullfile(workerFolder,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator;
previous=p.inertial.lowerLoadInverse;p=acf_prepare(p);
assert(isequal(previous.numerator,p.inertial.lowerLoadInverse.numerator) && ...
 isequal(previous.denominator,p.inertial.lowerLoadInverse.denominator),'Identified coefficient refactor changed the selected controller.');
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;oldThreads=maxNumCompThreads(2);
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(workerFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(workerFolder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,oldThreads));
mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);acf_select_actuator(mdl,p.actuator.kind);
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);assert(isempty(loops));
save_system(mdl,fullfile(workerFolder,[mdl '.slx']));
selected=struct('status','bounded_sharp_pilot_selected_pending_endpoint_and_repeat_qualification', ...
 'checkpointFile',options.checkpointFile,'caseIndex',options.caseIndex,'commonPreload_N',options.commonPreload_N);
sourceCandidate=fullfile(workerFolder,'normal_contact_trial_candidate.mat');save(sourceCandidate,'p','selected','-v7');
candidateFile=fullfile(outputFolder,['inertial_mechanics_' options.label '_candidate_20260930.mat']);
assert(~isfile(candidateFile));copyfile(sourceCandidate,candidateFile);
files=dir(fullfile(workerFolder,'helpers','*.m'));helpers=struct([]);
for j=1:numel(files),helpers=append(helpers,struct('file',files(j).name,'sha256',sha256(fullfile(files(j).folder,files(j).name))));end
manifest=struct('date','2026-09-30','status',selected.status,'sourceFolder',workerFolder, ...
 'sourceModel',fullfile(workerFolder,[mdl '.slx']),'sourceModelSHA256',sha256(fullfile(workerFolder,[mdl '.slx'])), ...
 'ancestorModel',sourceModel,'ancestorModelSHA256',sourceHash,'candidateFile',candidateFile, ...
 'candidateSHA256',sha256(candidateFile),'helperSources',helpers,'recipeSHA256',sha256([mfilename('fullpath') '.m']), ...
 'algebraicLoops',0,'commonPreload_N',p.springs.commonPreload_N, ...
 'combinedFreeLengthMax_mm',2000*(p.springs.lower.freeLength_m+p.springs.lower.freeLengthTolerance_m), ...
 'combinedInstalledLengthMax_mm',2000*(p.springs.lower.freeLength_m+p.springs.lower.freeLengthTolerance_m-p.springs.commonPreload_N/p.springs.lower.rate_N_m), ...
 'nativeLowerStiffness_N_m',p.mechanics.k1,'nativeLowerDamping_Ns_m',p.mechanics.c1, ...
 'actualSensingCellStiffness_N_m',p.springs.sensingCell.stiffness_N_m, ...
 'masslessSeriesComplianceApproximation',true, ...
 'forcePath',p.springs.sensingCell.loadPath,'loadCellBypass',false, ...
 'separateActualMountModesQualified',false, ...
 'logs',struct('originalPhysical',35,'normalContact',8,'accelerometer',6,'motionObserver',2, ...
 'measuredDrag',3,'forceVariation',5,'accelerationFeedforward',2,'lowerInverse',4), ...
 'lowerInverseLogSchema',{{'desired_Rz_deviation_N','identified_S_inertia_N','desired_lower_load_deviation_N','inverse_current_A'}});
assert(strcmp(sourceHash,sha256(sourceModel)));
write_json(fullfile(outputFolder,['inertial_mechanics_' options.label '_manifest_20260930.json']),manifest);
fprintf('INERTIAL_MECHANICS_SELECTED_SOURCE %s\n',workerFolder);
end
