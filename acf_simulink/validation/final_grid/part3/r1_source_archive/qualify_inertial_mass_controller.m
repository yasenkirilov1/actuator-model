function report=qualify_inertial_mass_controller(sourceFolder,outputFolder,phase,candidateFile,options)
%QUALIFY_INERTIAL_MASS_CONTROLLER Isolated native measured-inertia experiments.
% This harness copies the completed normal-contact/accelerometer prototype.
% It scores the entire native recording, never replaces the plant, and never
% writes the authoritative model or its helpers. Seed zero denotes clean
% stochastic channels; sensor filtering, delays and ADC quantization remain.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
project=fileparts(fileparts(mfilename('fullpath')));
if nargin<2 || isempty(outputFolder),outputFolder=fullfile(project,'validation');end
if nargin<3 || isempty(phase),phase='pi_screen';end
if nargin<4 || isempty(candidateFile),candidateFile=fullfile(sourceFolder,'normal_contact_trial_candidate.mat');end
if nargin<5,options=struct;end
options=defaults(options,struct('seeds',[0 7 29 1729], 'massS_kg',1.4, ...
 'forceKp',[.03 .06 .1],'forceKi',[20 40 80], ...
 'currentResponse_s',NaN,'simulationStep_s',NaN, ...
 'derivativeCandidates',[3e-6 1e-5], ...
 'guideVelocityPrediction_s',[0 300e-6 400e-6], ...
 'feedbackCutoff_Hz',[500 800 1000], ...
 'notchZeroDamping',[.001 .01],'notchPoleDamping',[1 2], ...
 'pilotLightSeeds',[0 29], ...
 'accelerationPrediction_s',[0 100e-6 200e-6 300e-6], ...
 'accelerationDerivativeFilter_s',300e-6, ...
 'feedbackGainDamping',[.03 1.2;.03 1.5;.06 1.5], ...
 'accelerationFeedforwardFilter_s',[0 .0003 .0005 .0007 .001 .0015], ...
 'actuatorParallelDamping_Ns_m',[15 30 60 120], ...
 'performanceCurrentResponse_s',[50e-6 100e-6 150e-6], ...
 'performanceBusVoltage_V',[24 36],'performanceSeeds',[7 1729], ...
 'performanceOrdinaryReferenceTrace','', ...
 'forceValues_N',5:5:50,'angleValues_deg',0:15:90,'muValues',[0 .3], ...
 'partitionIndex',1,'partitionCount',1,'keepNativeWorstInMemory',true, ...
 'clearSDIEvery',8,'retainPlotExcerpts',true, ...
 'allowFiniteCell',false,'simulationMode','normal','checkpointEvery',1,'tag',''));
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
assert(isfile(sourceModel) && isfile(candidateFile),'Wait for the usable frozen inertial prototype.');
stem=['normal_mass_controller_inertial_' phase '_20260930' options.tag];
resultFile=fullfile(outputFolder,[stem '_results.json']);
assert(~isfile(resultFile),'Preserve earlier evidence; choose a new output folder or options.tag.');
maxNumCompThreads(2);
workerFolder=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','helpers','cache','code','sdi'}
 folder=fullfile(workerFolder,name{1});if ~isfolder(folder),mkdir(folder);end
end
if ~isfolder(outputFolder),mkdir(outputFolder);end
traceFolder=fullfile(outputFolder,[stem '_traces']);mkdir(traceFolder);
modelHash=sha256(sourceModel);candidateHash=sha256(candidateFile);
modelFile=fullfile(workerFolder,'acf_force_control_normal_trial.slx');
copyfile(sourceModel,modelFile);
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile(candidateFile,fullfile(workerFolder,'candidate.mat'));
assert(strcmp(candidateHash,sha256(fullfile(workerFolder,'candidate.mat'))), ...
 'Candidate changed while being copied; do not mix mutable input versions.');
recipeSnapshot=fullfile(workerFolder,'qualify_inertial_mass_controller.m');
copyfile([mfilename('fullpath') '.m'],recipeSnapshot);
addpath(fullfile(workerFolder,'helpers'),'-begin');
clear acf_prepare acf_run acf_metrics acf_select_actuator;
files=dir(fullfile(workerFolder,'helpers','*.m'));sources=struct([]);
for j=1:numel(files)
 sources=append(sources,struct('file',files(j).name, ...
  'sha256',sha256(fullfile(files(j).folder,files(j).name))));
end
saved=load(fullfile(workerFolder,'candidate.mat'),'p');p=saved.p;
assert(isfield(p,'accel') && p.accel.enabled && isfield(p,'inertial'), ...
 'Candidate lacks the real measured-acceleration controller.');
p=ensure_feedforward_fields(p);
if ~isfield(p.accel,'nominalNoiseRms'),p.accel.nominalNoiseRms=p.accel.noiseRms;end
p.sim.stopTime=4;p.sim.evaluateFrom=0;
if isfinite(options.simulationStep_s),p.sim.step=options.simulationStep_s;end
if isfinite(options.currentResponse_s),p.coil.currentResponse=options.currentResponse_s;end
p.robot.forceGate.filterTime=.1;p.robot.forceGate.dwellTime=.1;
assert(p.mechanics.upperRigid && p.mechanics.mB==.6);
assert(p.mechanics.lowerRigid || options.allowFiniteCell, ...
 'Finite-cell runs require the explicitly labeled allowFiniteCell option.');
assert(strcmp(p.sensor.positionTarget,'tool_relative_A') && p.robotPrediction.enabled);
assert(p.force.sampleTime==100e-6);
if isfield(p,'acquisition') && p.acquisition.enabled
 assert(p.sensor.sampleTime==p.acquisition.rawSampleTime_s && ...
  p.sim.step<=p.acquisition.rawSampleTime_s, ...
  'Resolve the final raw acquisition period; control remains10kHz.');
else
 assert(p.sensor.sampleTime==100e-6);
end
assert(p.sensor.filterTime==p.accel.filterTime && p.sensor.delay==p.accel.delay, ...
 'This cohort requires force/accelerometer filter and acquisition alignment.');
p=acf_prepare(p);
previousStorage=Simulink.sdi.getStorageLocation;
previousRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
ownedStorage=fullfile(workerFolder,'sdi');Simulink.sdi.setStorageLocation(ownedStorage);
Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,ownedStorage)); %#ok<NASGU>
assert(ismember(options.simulationMode,{'normal','accelerator'}));
set_param(mdl,'SimulationMode',options.simulationMode);
sharedMeasurement=getSimulinkBlockHandle([mdl '/Contact force measurement'])~=-1;
if sharedMeasurement
 controller=[mdl '/Force controller/Coil PI and PID'];
 feedforward=[controller '/Measured motion feedforward'];
 controlChoice='Coil PI and PID';
 auditRoots={controller,[mdl '/Contact force measurement']};
 if p.inertial.forceKd>0,p.coil.strategy='pid';else,p.coil.strategy='pi';end
else
 controller=[mdl '/Force controller/Voice coil inertial control'];
 feedforward=controller;controlChoice='Voice coil inertial control';auditRoots={controller};
end
assert(getSimulinkBlockHandle(controller)~=-1,'Frozen native inertial variant missing.');
assert(contains(get_param([feedforward '/Measured acceleration current feedforward'],'Gain'), ...
 'p.inertial.accelerationFeedforwardScale'), 'Acceleration-FF ablation must change the actual native Gain.');
assert(contains(get_param([feedforward '/Guide viscous friction current'],'Gain'), ...
 'p.inertial.guideFrictionFeedforwardScale'), 'Guide-FF ablation must change the actual native Gain.');
fromBlocks={};
for j=1:numel(auditRoots)
 found=find_system(auditRoots{j},'MatchFilter',@Simulink.match.allVariants,'BlockType','From');
 fromBlocks=[fromBlocks;found(:)]; %#ok<AGROW>
end
tags=cellfun(@(b)get_param(b,'GotoTag'),fromBlocks,'UniformOutput',false);
allowed={'measured_S_acceleration_z','measured_S_acceleration_x', ...
 'measured_lateral_contact_N','command_based_robot_acceleration', ...
 'command_based_robot_velocity','inertial_robot_velocity_aligned', ...
 'inertial_axial_reference_deviation_N','inertial_head_gap_velocity', ...
 'inertial_robot_acceleration_aligned'};
assert(all(ismember(tags,allowed)),'Unreviewed or physical-truth From input enters active inertial control.');
configs=struct([]);keys={};
switch phase
 case 'pi_screen'
  for kp=options.forceKp
   for ki=options.forceKi
    q=p;q.inertial.forceKp=kp;q.inertial.forceKi=ki;q.inertial.forceKd=0;
    q.inertial.accelerationFeedforwardScale=1;q.inertial.guideFrictionFeedforwardScale=1;
    q.contact.geometryFeedforwardScale=1;
    add_tuning_cohort(q,'common_PI_gain_search');
   end
  end
 case 'strategy_screen'
  % Identical force reconstruction and motion sensing in every strategy.
  % All variants retain the known static holding current and anti-windup.
  q=p;q.inertial.forceKd=0;q.inertial.accelerationFeedforwardScale=1;
  q.inertial.guideFrictionFeedforwardScale=1;q.contact.geometryFeedforwardScale=1;
  add_tuning_cohort(q,'PI_measured_accel_geometry_friction_FF');
  q.inertial.accelerationFeedforwardScale=0;
  add_tuning_cohort(q,'PI_geometry_friction_FF');
  q.inertial.guideFrictionFeedforwardScale=0;q.contact.geometryFeedforwardScale=0;
  add_tuning_cohort(q,'PI_static_holding_only');
  for guideScale=[0 .5]
   q=p;q.inertial.forceKd=0;q.inertial.accelerationFeedforwardScale=1;
   q.inertial.guideFrictionFeedforwardScale=guideScale;q.contact.geometryFeedforwardScale=1;
   add_tuning_cohort(q,'PI_measured_accel_geometry_partial_guide_FF');
  end
  for kd=options.derivativeCandidates
   q=p;q.inertial.forceKd=kd;q.inertial.accelerationFeedforwardScale=1;
   q.inertial.guideFrictionFeedforwardScale=1;q.contact.geometryFeedforwardScale=1;
   add_tuning_cohort(q,'PID_measured_accel_geometry_friction_FF');
  end
 case 'endpoint_confirmation'
  for mass=[.1 2]
   add_force_angle_grid(p,mass,[5 50],[0 90],[0 .3],options.seeds,'endpoint_confirmation');
  end
 case 'guide_velocity_lead'
  assert(sharedMeasurement && isfield(p.inertial,'guideVelocityPrediction_s'));
  assert(strcmp(get_param([feedforward '/Identified guide prediction time'],'Gain'), ...
   'p.inertial.guideVelocityPrediction_s'));
  for prediction=options.guideVelocityPrediction_s
   q=p;q.inertial.guideVelocityPrediction_s=prediction;
   add_tuning_cohort(q,'causal_guide_FF_velocity_prediction');
  end
 case 'fast_sensor_candidate'
  % Conditional acceptance envelope for two orthogonal ADXL1002 analog
  % axes and synchronized force acquisition, not an exact datasheet model.
  q=p;q.sensor.filterTime=100e-6;q.accel.filterTime=100e-6;
  q.sensor.delay=100e-6;q.accel.delay=100e-6;
  q.accel.noiseRms=.0123;q.accel.nominalNoiseRms=.0123;
  q.accel.quantum=.00468;q.accel.range_g=50;
  q.sensorDesign=struct('accelerometer','Two orthogonal ADXL1002 analog axes', ...
   'noiseDensity_ug_sqrtHz',25,'assumedENBW_Hz',2500, ...
   'derivedRMS_m_s2',.0122625,'modeledRMS_m_s2',.0123, ...
   'ADCbits',18,'ADCvoltageRange_V',[0 5],'sensitivity_V_g',.04, ...
   'derivedQuantum_m_s2',.004678,'modeledQuantum_m_s2',.00468, ...
   'commonResponse_s',100e-6,'commonAcquisitionDelay_s',100e-6, ...
   'forceConditioning','ATI analog conditioning: intrinsic second-order5.1kHz anti-alias response must be identified and matched, not removed.', ...
   'interpretation','Plausible designed+identified common-response acceptance envelope. Real phase, noise, synchronization and calibration remain prototype acceptance measurements.');
  add_tuning_cohort(q,'conditional_ADXL1002_matched_100us_response');
 case 'feedback_rolloff'
  filterPath=[controller '/Normal force PI and PID/Optional feedback rolloff'];
  assert(getSimulinkBlockHandle(filterPath)~=-1, ...
   'Wait for the frozen native feedback-only rolloff prototype.');
  assert(strcmp(get_param(filterPath,'Numerator'),'p.d.feedbackFilter.numerator'));
  q=p;q.inertial.feedbackFilter.enabled=false;
  add_tuning_cohort(q,'saved_total_current_notch_reference');
  for cutoff=options.feedbackCutoff_Hz
   q=p;q.inertial.currentNotch.enabled=false;
   q.inertial.feedbackFilter=struct('enabled',true, ...
    'cutoff_Hz',cutoff,'damping',1/sqrt(2));
   add_tuning_cohort(q,'feedback_only_Butterworth_direct_motion_FF');
  end
 case 'repeated_bump_confirmation'
  q=p;q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=10;
  add_tuning_cohort(q,'ten_second_repeated_bump_confirmation');
 case 'repeated_bump_pilot'
  q=p;q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=10;
  for seed=options.seeds
   add_config(physical_case(q,2,50,90,.3,seed), ...
    'ten_second_repeated_bump_heavy_pilot',seed);
  end
  for seed=options.pilotLightSeeds
   add_config(physical_case(q,.1,5,0,.3,seed), ...
    'ten_second_repeated_bump_light_pilot',seed);
  end
 case 'broad_current_notch'
  for zeroDamping=options.notchZeroDamping
   for poleDamping=options.notchPoleDamping
    q=p;q.inertial.feedbackFilter.enabled=false;
    q.inertial.currentNotch.enabled=true;
    q.inertial.currentNotch.zeroDamping=zeroDamping;
    q.inertial.currentNotch.poleDamping=poleDamping;
    for seed=options.seeds
     add_config(physical_case(q,2,50,90,.3,seed), ...
      'broad_total_current_notch_heavy_pilot',seed);
    end
    for seed=options.pilotLightSeeds
     add_config(physical_case(q,.1,5,0,.3,seed), ...
      'broad_total_current_notch_light_pilot',seed);
    end
   end
  end
 case {'acceleration_command_prediction','measured_drag_ratio_prediction'}
  predictionPath=[feedforward '/Acceleration FF prediction time'];
  assert(getSimulinkBlockHandle(predictionPath)~=-1, ...
   'Wait for the frozen native FF-only acceleration-prediction prototype.');
  assert(strcmp(get_param(predictionPath,'Gain'),'p.inertial.accelerationPrediction_s'));
  predictionBase=p;
  if strcmp(phase,'measured_drag_ratio_prediction')
   predictionBase.contact.geometryMode='measured_drag_ratio';
   predictionBase.contact.dragRatioFilter_s=.005;
   predictionBase.contact.dragRatioMinForce_N=1;
   predictionBase.inertial.currentNotch.enabled=false;
   predictionBase.inertial.feedbackFilter=struct('enabled',true, ...
    'cutoff_Hz',1000,'damping',1/sqrt(2));
  end
  for prediction=options.accelerationPrediction_s
   q=predictionBase;q.inertial.accelerationPrediction_s=prediction;
   q.inertial.accelerationDerivativeFilter_s=options.accelerationDerivativeFilter_s;
   add_tuning_cohort(q,'causal_acceleration_current_FF_prediction');
  end
 case 'full_range_mass'
  % The coordinator supplies the same candidate and explicit seed list to
  % disjoint workers. Mass, force and angle vectors can define rectangular
  % subsets; an index/count pair can instead partition one identical mesh.
  for mass=options.massS_kg
   add_force_angle_grid(p,mass,options.forceValues_N,options.angleValues_deg, ...
    options.muValues,options.seeds,'known_mass_full_force_angle_grid');
  end
 case 'final_performance_limits'
  % Deliberately vary only the inner current response and available ideal
  % bus voltage. The chosen mechanics, known-mass inverse, sensor chain,
  % force gains and force-stability gate remain the final source contract.
  assert(isfield(p.inertial,'lowerLoadInverse') && p.inertial.lowerLoadInverse.enabled && ...
   ~p.mechanics.lowerRigid && isfield(p,'springs') && p.springs.commonPreload_N==33.6);
  assert(p.robot.forceGate.enabled && p.robot.forceGate.variation.enabled, ...
   'This comparison requires the final enabled force-stability gate.');
  for tau=options.performanceCurrentResponse_s
   for bus=sort(options.performanceBusVoltage_V)
    q=p;q.coil.currentResponse=tau;q.coil.voltage=bus;
    q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=10;
    q.sim.step=min(q.sim.step,10e-6);
    for seed=options.performanceSeeds
     add_config(physical_case(q,2,50,90,.3,seed), ...
      'final_source_inner_current_response_and_ideal_bus_only',seed);
    end
   end
  end
 case 'measured_drag_ratio_feedback'
  for j=1:size(options.feedbackGainDamping,1)
   q=p;q.contact.geometryMode='measured_drag_ratio';
   q.contact.dragRatioFilter_s=.005;q.contact.dragRatioMinForce_N=1;
   q.inertial.currentNotch.enabled=false;
   q.inertial.forceKp=options.feedbackGainDamping(j,1);
   q.inertial.forceKi=20;q.inertial.forceKd=0;
   q.inertial.accelerationPrediction_s=0;
   q.inertial.feedbackFilter=struct('enabled',true,'cutoff_Hz',1000, ...
    'damping',options.feedbackGainDamping(j,2));
   add_tuning_cohort(q,'measured_drag_ratio_PI_rolloff_damping_screen');
  end
 case 'band_limited_acceleration_feedforward'
  filterPath=[feedforward '/Acceleration FF command filter'];
  assert(getSimulinkBlockHandle(filterPath)~=-1, ...
   'Wait for the frozen native FF-only command low-pass prototype.');
  for commandFilter=options.accelerationFeedforwardFilter_s
   q=p;q.inertial.accelerationFeedforwardFilter_s=commandFilter;
   q.inertial.accelerationDerivativeFilter_s=options.accelerationDerivativeFilter_s;
   if commandFilter==0
    q.inertial.accelerationPrediction_s=0;
   else
    dt=q.force.sampleTime;
    commandDCdelay=dt/(exp(dt/commandFilter)-1);
    sensorDCdelay=q.sensor.sampleTime/(exp(q.sensor.sampleTime/q.sensor.filterTime)-1);
    q.inertial.accelerationPrediction_s=commandDCdelay+sensorDCdelay+ ...
     q.sensor.delay+q.coil.currentResponse+dt/2;
   end
   add_tuning_cohort(q,'causal_band_limited_acceleration_current_FF');
  end
 case 'identified_fast_drive'
  assert(p.coil.currentResponse==50e-6, ...
   'Pass currentResponse_s=50e-6 for the distinct identified faster-drive target.');
  q=p;q.inertial.accelerationFeedforwardFilter_s=0;
  q.inertial.accelerationPrediction_s=0;
  add_tuning_cohort(q,'identified_current_50us_direct_measured_acceleration_FF');
  for commandFilter=[0 .001]
   q=p;q.inertial.accelerationFeedforwardFilter_s=commandFilter;
   q.inertial.accelerationDerivativeFilter_s=options.accelerationDerivativeFilter_s;
   dt=q.force.sampleTime;commandDCdelay=0;
   if commandFilter>0,commandDCdelay=dt/(exp(dt/commandFilter)-1);end
   sensorDCdelay=q.sensor.sampleTime/(exp(q.sensor.sampleTime/q.sensor.filterTime)-1);
   q.inertial.accelerationPrediction_s=commandDCdelay+sensorDCdelay+ ...
    q.sensor.delay+q.coil.currentResponse+dt/2;
   add_tuning_cohort(q,'identified_current_50us_compensated_acceleration_FF');
  end
 case 'identified_drive_noise_tradeoff'
  % A separately identified slower inner current response can attenuate
  % cell/noise excitation. Its increased bump lag must still earn a native
  % pass; this is not a bus-voltage intervention or a hardware claim.
  designs=[150e-6 0;150e-6 .001;200e-6 .001];
  for designIndex=1:size(designs,1)
   q=p;q.coil.currentResponse=designs(designIndex,1);
   commandFilter=designs(designIndex,2);
   q.inertial.accelerationFeedforwardFilter_s=commandFilter;
   q.inertial.accelerationDerivativeFilter_s=options.accelerationDerivativeFilter_s;
   q.inertial.accelerationPrediction_s=0;
   if commandFilter>0
    dt=q.force.sampleTime;commandDCdelay=dt/(exp(dt/commandFilter)-1);
    sensorDCdelay=q.sensor.sampleTime/(exp(q.sensor.sampleTime/q.sensor.filterTime)-1);
    q.inertial.accelerationPrediction_s=commandDCdelay+sensorDCdelay+ ...
     q.sensor.delay+q.coil.currentResponse+dt/2;
   end
   add_tuning_cohort(q,'identified_current_response_noise_phase_tradeoff');
  end
 case 'parallel_actuator_damper'
  % An external viscous element spans A--B in parallel with the actuator.
  % It does not span/bypass the B--S force cell. The existing native guide
  % viscous Gain represents the combined intrinsic2 + external damping.
  assert(p.coil.viscousFriction==2, ...
   'Use the recorded intrinsic2Ns/m baseline before adding an external damper.');
  physicalDamping=find_system(mdl,'LookUnderMasks','all', ...
   'MatchFilter',@Simulink.match.allVariants,'BlockType','Gain', ...
   'Gain','p.coil.viscousFriction');
  assert(numel(physicalDamping)==1 && ...
   contains(get_param([feedforward '/Guide viscous friction current'],'Gain'), ...
    'p.coil.viscousFriction'), ...
   'The matched damper sweep must change native physical and FF gains together.');
  for totalDamping=options.actuatorParallelDamping_Ns_m
   assert(totalDamping>=2,'Combined damping cannot be below this intrinsic baseline.');
   q=p;q.coil.viscousFriction=totalDamping;
   q.actuatorParallelDamper=struct('enabled',true,'attachment','A_to_B', ...
    'intrinsicViscous_Ns_m',2,'addedViscous_Ns_m',totalDamping-2, ...
    'combinedViscous_Ns_m',totalDamping,'feedforwardMatched',true, ...
    'interpretation','Approximate external viscous-damper target in parallel with the actuator; not a bearing-friction claim or a B-S sensing bypass.');
   q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=10;
   add_tuning_cohort(q,'matched_external_A_to_B_viscous_damper');
  end
 case 'identified_current_response'
  for tau=[50e-6 100e-6 200e-6]
   q=p;q.coil.currentResponse=tau;q.sim.step=25e-6;
   add_tuning_cohort(q,'identified_current_response_same_outer_gains');
  end
 otherwise,error('Unknown inertial study phase: %s',phase);
end
assert(options.partitionCount>=1 && options.partitionCount==round(options.partitionCount) && ...
 options.partitionIndex>=1 && options.partitionIndex<=options.partitionCount && ...
 options.partitionIndex==round(options.partitionIndex),'Invalid disjoint partition.');
unpartitionedCaseCount=numel(configs);
unpartitionedCaseIndices=options.partitionIndex:options.partitionCount:unpartitionedCaseCount;
configs=configs(unpartitionedCaseIndices);
assert(~isempty(configs),'Requested partition has no cases.');
assert(options.clearSDIEvery>=1 && options.clearSDIEvery==round(options.clearSDIEvery));
configurationFile=fullfile(outputFolder,[stem '_configuration.mat']);
save(configurationFile,'configs','options','unpartitionedCaseIndices','-v7');
rows=struct([]);ranking=struct([]);plotTraces=struct([]);
started=tic;lastCompileKey=[];worst=[];worstRow=[];worstParameters=[];worstError=-inf;
worstHistoryFile=fullfile(outputFolder,[stem '_worst_native_trace.mat']);
voltagePairReferences=struct;voltagePairs=struct([]);boundedTraceFiles={};
failedNativeTraces={};
ordinarySourceParity=[];
groupKeys={};groupWorst=[];groupTraceFiles={};
report=struct('date','2026-09-30','status','running','phase',phase, ...
 'recipeVersion','native_inertial_normal_mass_v7_partitioned_failed_history_retention','options',options, ...
 'scope','Surface-normal pressing force within +/-0.5 N on every native sample, including startup, through each case-specific full recording;1mm high10mm wide bumps at100mm/s. Conditional finite simulation, not physical qualification.', ...
 'recordingStopTimes_s',unique(arrayfun(@(c)c.p.sim.stopTime,configs)), ...
 'sourceModel',sourceModel,'modelSha256',modelHash,'privateModel',modelFile, ...
 'candidateFile',candidateFile,'candidateSha256',candidateHash,'helperSources',sources, ...
 'experimentSha256',sha256(recipeSnapshot),'experimentSnapshot',recipeSnapshot, ...
 'matlabVersion',version,'baseParameters',p,'plannedCases',numel(configs), ...
 'unpartitionedPlannedCases',unpartitionedCaseCount, ...
 'unpartitionedCaseIndices',unpartitionedCaseIndices, ...
 'configurationFile',configurationFile,'simulationMode',options.simulationMode, ...
 'controllerFromBlocks',{fromBlocks},'controllerFromTags',{tags}, ...
 'controllerAuditRoots',{auditRoots},'activeCoilVariant',controlChoice,'rows',rows, ...
 'massDefinition','S is the known tool mass downstream of the B-S cell, B=.6kg. Mechanical mass and sensor inertia compensation change together; exact static gravity tare.', ...
 'sensing','Axial+lateral tared B-S force and measured S axial/lateral acceleration reconstruct contact force; S-A position plus issued robot commands estimate motion and surface direction. All sensors retain their actual native filter/delay/quantization.', ...
 'controllerExclusions','No true surface profile/slope/friction, normal-contact truth, physical acceleration truth, actual current, future command or surface preview enters the active controller.', ...
 'accelerometerLogNames',{{'measured_az_m_s2','measured_ax_m_s2', ...
 'physical_az_m_s2','physical_ax_m_s2','reconstructed_Rz_N','reconstructed_Rx_N'}}, ...
 'motionObserverLogNames',{{'estimated_gap_displacement_m','estimated_gap_velocity_m_s'}}, ...
 'qualificationLimit','Sampled nominal mechanics/contact/known-mass/sensor model; no complete calibration, changing orientation/friction, sanding vibration or compliant robot/contact qualification.', ...
 'metrics','All scores use full native samples. Earlier35-channel axial tracking scores are separate diagnostics;250us excerpts are for plotting only.', ...
 'nativeTraceRetention','Full worst recording per controller and worker, first recording, and requested selected plot excerpts. Other full native histories are scored and released, with every case retained in metrics.');
for j=1:numel(configs)
 config=configs(j);q=config.p;
 compileKey=[q.sim.step,q.force.sampleTime,q.sensor.sampleTime,q.sensor.delay, ...
  q.sensor.strokeDelay,q.accel.delay,q.coil.currentResponse, ...
  double(strcmp(q.coil.strategy,'pid')),double(q.inertial.accelerationFeedforwardScale==0)];
 if ~isequal(lastCompileKey,compileKey)
  set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');lastCompileKey=compileKey;
 end
 timer=tic;r=[];exception='';nativeReturned=false;
 try
  quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  nativeReturned=~isempty(r);
  assert(strcmp(get_param([mdl '/Force controller'],'LabelModeActiveChoice'), ...
   controlChoice),'Incorrect native control variant selected.');
  assert(isfield(r,'normalContact') && size(r.normalContact.data,2)==8);
  assert(isfield(r,'accelerometer') && size(r.accelerometer.data,2)==6);
  assert(isfield(r,'motionObserver') && size(r.motionObserver.data,2)==2);
  assert(isequal(r.time,r.normalContact.time,r.accelerometer.time,r.motionObserver.time), ...
   'Full native physical/control/IMU grids must match exactly.');
  nativeLogNames=fieldnames(r);
  for logIndex=1:numel(nativeLogNames)
   logged=r.(nativeLogNames{logIndex});
   if isstruct(logged) && isfield(logged,'time') && isfield(logged,'data')
    assert(isequal(r.time,logged.time), ...
     'Every additional native controller log must retain the physical grid.');
   end
  end
  if isfield(r,'lowerInverse')
   assert(size(r.lowerInverse.data,2)==4, ...
    'Proper lower-link inverse native audit requires its four declared channels.');
  end
  if isfield(r,'forceVariation')
   assert(isequal(r.time,r.forceVariation.time) && size(r.forceVariation.data,2)==5, ...
    'Native fast force-stability log must match every physical sample.');
  end
  if isfield(r,'accelerationFeedforward')
   assert(isequal(r.time,r.accelerationFeedforward.time) && ...
    ismember(size(r.accelerationFeedforward.data,2),[2 3]), ...
    'Native measured/predicted FF acceleration must match every physical sample.');
   assert(isequal(r.accelerationFeedforward.data(:,1),r.accelerometer.data(:,1)), ...
    'FF prediction input must be the actual measured acceleration channel.');
  end
  if isfield(r,'measuredDrag')
   assert(isequal(r.time,r.measuredDrag.time) && size(r.measuredDrag.data,2)==3, ...
    'Native measured-drag ratio log must match every physical sample.');
  end
  row=measure_run(q,r,j,config,toc(timer),'');
  if strcmp(phase,'final_performance_limits')
   retained=fullfile(traceFolder,sprintf('performance_case_%02d_native.mat',j));
   save(retained,'r','row','-v7');boundedTraceFiles{end+1}=retained; %#ok<AGROW>
   if ~isempty(options.performanceOrdinaryReferenceTrace) && ...
     q.coil.currentResponse==100e-6 && q.coil.voltage==24 && config.seedRequested==7
    reference=load(options.performanceOrdinaryReferenceTrace,'r');
    ordinarySourceParity=compare_manual_native(reference.r,r);clear reference;
    ordinarySourceParity.referenceTrace=options.performanceOrdinaryReferenceTrace;
    ordinarySourceParity.comparisonTrace=retained;
    assert(ordinarySourceParity.allManualHistoriesBitwiseIdentical, ...
     'Final-source ordinary/Accelerator full native parity must pass.');
   end
   pairKey=sprintf('tau_%dus_seed_%d',round(q.coil.currentResponse*1e6),config.seedRequested);
   if q.coil.voltage==min(options.performanceBusVoltage_V)
    voltagePairReferences.(pairKey)=retained;
   else
    assert(isfield(voltagePairReferences,pairKey),'Run the lower bus reference before its pair.');
    reference=load(voltagePairReferences.(pairKey),'r');
    pair=compare_manual_native(reference.r,r);clear reference;
    pair.currentResponse_s=q.coil.currentResponse;pair.seed=config.seedRequested;
    pair.referenceBusVoltage_V=min(options.performanceBusVoltage_V);
    pair.comparisonBusVoltage_V=q.coil.voltage;
    pair.referenceTrace=voltagePairReferences.(pairKey);pair.comparisonTrace=retained;
    voltagePairs=append(voltagePairs,pair);
   end
  end
 catch failure
  exception=getReport(failure,'extended','hyperlinks','off');
  if nativeReturned && ~isempty(r)
   rejectionFile=fullfile(traceFolder,sprintf('case_%03d_unvalidated_native.mat',j));
   save(rejectionFile,'r','config','exception','-v7');
  end
  r=[];
  row=measure_run(q,r,j,config,toc(timer),exception);
  fprintf('INERTIAL_NATIVE_FAILURE %d: %s\n',j,failure.message);
 end
 row.nativeReturned=nativeReturned;rows=append(rows,row);
 fid=fopen(fullfile(outputFolder,[stem '_rows.jsonl']),'a');assert(fid~=-1);
 fprintf(fid,'%s\n',jsonencode(row));fclose(fid);
 if ~isempty(r)
  if ~row.passed
   failedTrace=fullfile(traceFolder,sprintf('case_%04d_failed_native.mat',j));
   save(failedTrace,'r','row','config','-v7');
   failedNativeTraces{end+1}=failedTrace; %#ok<AGROW>
  end
  if row.peakNormalForceError_N>worstError
   worstError=row.peakNormalForceError_N;worst=r;worstRow=row;worstParameters=r.parameters;
   if ~options.keepNativeWorstInMemory
    save(worstHistoryFile,'worst','worstRow','-v7');worst=[];
   end
  end
  key=controller_key(q);which=find(strcmp(groupKeys,key),1);
  if isempty(which)
   which=numel(groupKeys)+1;groupKeys{which}=key;groupWorst(which)=-inf;
   groupTraceFiles{which}=fullfile(traceFolder,sprintf('controller_%02d_worst_native.mat',which));
  end
  if row.peakNormalForceError_N>groupWorst(which)
   groupWorst(which)=row.peakNormalForceError_N;
   save(groupTraceFiles{which},'r','row','-v7');
  end
  if j==1,save(fullfile(outputFolder,[stem '_first_native_trace.mat']),'r','row','-v7');end
  if options.retainPlotExcerpts && want_plot_excerpt(q,config,phase)
   plotTraces=append(plotTraces,plot_excerpt(r,row));
  end
 end
 clear r;
 if mod(j,max(1,options.checkpointEvery))==0 || j==1 || j==numel(configs)
  checkpoint('running');
 end
 if mod(j,4)==0 || j==1
  fprintf('INERTIAL_MASS_PROGRESS %s %d/%d,%d passed: S%.2g,F%.0f,ang%.0f,mu%.1f,seed%d,Kp%.4g,Ki%.4g,Kd%.3g:%.9gN,elapsed%.1fs\n', ...
   phase,j,numel(configs),sum([rows.passed]),row.massS_kg,row.reference_N, ...
   row.angle_deg,row.mu,row.seedRequested,row.forceKp_A_N,row.forceKi_A_Ns, ...
   row.forceKd_A_s_N,row.peakNormalForceError_N,toc(started));
 end
 if mod(j,options.clearSDIEvery)==0,Simulink.sdi.clear;end
 if ~row.executed
  checkpoint('setup_failure');
  error('acf:inertialStudy:ContractFailure','Case%d lacks validated native history: %s',j,exception);
 end
 if isfile(fullfile(workerFolder,'stop_after_case'))
  if ~isempty(worst),save(worstHistoryFile,'worst','worstRow','report','-v7');end
  checkpoint('stopped_by_coordinator');fprintf('INERTIAL_STOPPED_AFTER_CASE %d.\n',j);return;
 end
end
ranking=rank_controllers(rows);report.controllerRanking=ranking;
springRows=rows([rows.externalLowerSpringPair] & [rows.executed]);
if ~isempty(springRows)
 report.commonSpringPreloadInterval_N=[max([springRows.minimumCommonSpringPreload_N]), ...
  min([springRows.maximumCommonSpringPreload_N])];
 report.fixedSpringPreloads_N=unique([springRows.commonSpringPreload_N]);
 report.allActualFixedSpringPreloadsPassed=all([springRows.springPackingPassed]);
 report.maximumCombinedFreeSpringLength_mm=max([springRows.combinedFreeSpringLength_mm]);
 report.maximumCombinedInstalledSpringLength_mm=max([springRows.combinedInstalledSpringLength_mm]);
 report.springPackingScope='Opposed lower spring pair under the recorded full trajectories, evaluated at the stored fixed preload. Catalog force-cell stiffness is a massless series reference and is not a separately qualified mounting mode.';
end
report.privateFrozenModelUnchanged=strcmp(sha256(modelFile),modelHash);
report.upstreamSourceUnchanged=strcmp(sha256(sourceModel),modelHash);
assert(report.privateFrozenModelUnchanged && report.upstreamSourceUnchanged);
if ~isempty(worstRow)
 report.worstCaseParameters=worstParameters;report.worstCaseRow=worstRow;
 report.worstNativeTrace=worstHistoryFile;
 if options.keepNativeWorstInMemory
  save(worstHistoryFile,'worst','worstRow','report','-v7');
 else
  save(worstHistoryFile,'report','-append');
 end
end
if ismember(phase,{'pi_screen','strategy_screen','guide_velocity_lead','fast_sensor_candidate','feedback_rolloff','broad_current_notch','acceleration_command_prediction','measured_drag_ratio_prediction','measured_drag_ratio_feedback','band_limited_acceleration_feedforward','identified_fast_drive','identified_drive_noise_tradeoff','parallel_actuator_damper'}) && ~isempty(ranking)
 chosen=ranking(1);selected=configs(chosen.rowIds(1)).p;
 selected=physical_case(selected,1.4,20,0,0,29);
 selected.surface.kind='sharp_bump';selected.sim.stopTime=4;selected.sim.evaluateFrom=0;
 selected=acf_prepare(selected);pSelected=selected; %#ok<NASGU>
 save(fullfile(outputFolder,[stem '_selected_candidate.mat']),'pSelected','chosen');
 p=selected;save(fullfile(outputFolder,[stem '_candidate.mat']),'p','chosen');
 report.selectedCandidate=fullfile(outputFolder,[stem '_candidate.mat']);
 report.selectedCandidateSha256=sha256(report.selectedCandidate);
 report.selectionScope='Lowest worst-case normal error on the equal finite tuning cohort only; endpoint, full-range and calibration qualification still required.';
end
checkpoint('complete');
fprintf('INERTIAL_MASS_COMPLETE %s %d/%d valid native,%d passed,worst%.9gN.\n', ...
 phase,report.executed,report.plannedCases,report.passed,worstError);

 function add_tuning_cohort(base,scenario)
  for mass=[.1 2]
   if mass==.1,reference=5;angle=0;else,reference=50;angle=90;end
   for seed=options.seeds
    add_config(physical_case(base,mass,reference,angle,.3,seed),scenario,seed);
   end
  end
 end
 function add_force_angle_grid(base,mass,forces,angles,frictions,seeds,scenario)
  for reference=forces
   for angle=angles
    for mu=frictions
     for seed=seeds
      add_config(physical_case(base,mass,reference,angle,mu,seed),scenario,seed);
     end
    end
   end
  end
 end
 function add_config(q,scenario,seedRequested)
  key=[controller_key(q) sprintf('%.17g,',[q.mechanics.mS,q.force.reference, ...
   q.mechanics.angle,q.contact.mu,seedRequested,q.coil.currentResponse,q.sim.step, ...
   q.coil.voltage,q.coil.peakCurrent,q.coil.forceConstant,q.coil.resistance,q.coil.inductance])];
  if any(strcmp(keys,key)),return;end
  keys{end+1}=key;
  configs=append(configs,struct('p',q,'scenario',scenario,'key',key,'seedRequested',seedRequested));
 end
 function checkpoint(status)
  report.status=status;report.rows=rows;report.executed=sum([rows.executed]);
  report.validatedNativeHistories=report.executed;report.nativeReturned=sum([rows.nativeReturned]);
  report.attempts=numel(rows);report.failedBeforeNativeReturn=sum(~[rows.nativeReturned]);
  report.passed=sum([rows.passed]);report.rejectedBeforeMetrics=sum(~[rows.executed]);
  report.elapsedSeconds=toc(started);report.worstNormalForceError_N=max([rows.peakNormalForceError_N]);
  report.controllerTraceKeys=groupKeys;report.controllerWorstNativeTraces=groupTraceFiles;
  report.failedNativeTraces=failedNativeTraces;
  if strcmp(phase,'final_performance_limits')
   report.voltagePairs=voltagePairs;report.performanceNativeTraces=boundedTraceFiles;
   report.ordinarySourceExecutionParity=ordinarySourceParity;
   report.performanceComparisonScope='S2/F50/90deg/drag0.3, seven repeated sharp bumps in10s, two seeds. Only configured inner current response50/100/150us and ideal available bus24/36V vary. Outer gains, known-mass inverse coefficients, actual acquisition/filter/noise/gate and33.6N preload stay fixed. This alternative-drive probe does not replace the100us full-range preset.';
  end
  if isfile('/proc/self/status')
   processStatus=fileread('/proc/self/status');
   memoryPeak=regexp(processStatus,'VmHWM:\s+(\d+)\s+kB','tokens','once');
   memoryCurrent=regexp(processStatus,'VmRSS:\s+(\d+)\s+kB','tokens','once');
   if ~isempty(memoryPeak),report.processPeakResidentMiB=str2double(memoryPeak{1})/1024;end
   if ~isempty(memoryCurrent),report.processResidentMiB=str2double(memoryCurrent{1})/1024;end
  end
  fid=fopen(resultFile,'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  writetable(struct2table(rows,'AsArray',true),fullfile(outputFolder,[stem '_rows.csv']));
  save(fullfile(outputFolder,[stem '_results.mat']),'report','-v7');
  save(fullfile(outputFolder,[stem '_plot_traces.mat']),'plotTraces','-v7');
 end
end

function q=physical_case(p,mass,reference,angle,mu,seed)
q=p;q.mechanics.mS=mass;q.mechanics.mB=.6;
q.sensor.distalMass_kg=mass;q.sensor.distalMass_model_kg=mass;
q.force.reference=reference;q.mechanics.angle=angle*pi/180;q.contact.mu=mu;
if ~strcmp(q.coil.strategy,'observer')
 if q.inertial.forceKd>0,q.coil.strategy='pid';else,q.coil.strategy='pi';end
end
q.sensor.seed=seed;if seed==0,q.sensor.seed=29;end
q.sensor.noiseRms=.02;q.sensor.strokeNoiseRms=1e-6;
q.contact.lateralNoiseRms_N=.02;q.contact.lateralBias_N=0;
% Retain the identified nominal stochastic envelope when constructing a
% representative candidate from a clean winning row.
q.accel.noiseRms=field_or(q.accel,'nominalNoiseRms',q.accel.noiseRms);
if seed==0
 q.sensor.noiseRms=0;q.sensor.strokeNoiseRms=0;
 q.contact.lateralNoiseRms_N=0;q.accel.noiseRms=0;
end
end

function p=ensure_feedforward_fields(p)
if ~isfield(p.inertial,'accelerationFeedforwardScale'),p.inertial.accelerationFeedforwardScale=1;end
if ~isfield(p.inertial,'guideFrictionFeedforwardScale'),p.inertial.guideFrictionFeedforwardScale=1;end
end

function row=measure_run(q,r,id,config,runtime,exception)
externalLowerSpringPair=isfield(q,'springs') && isfield(q.springs,'lower') && ...
 ~q.mechanics.lowerRigid;
cellStiffness=q.mechanics.k1;cellDamping=q.mechanics.c1;
cellMeaning='Modeled B-S link is the finite load-cell structural approximation.';
if externalLowerSpringPair
 cellStiffness=NaN;cellDamping=NaN;
 cellMeaning='The modeled B-S link includes an external opposed spring pair; lowerLink stiffness/damping are aggregate link parameters, not the load-cell catalog stiffness.';
 if isfield(q.springs,'sensingCell')
  cellStiffness=q.springs.sensingCell.stiffness_N_m;
  cellMeaning='Cell stiffness is a separate catalog reference in the massless series approximation; lowerLink stiffness/damping describe the modeled spring-and-damper connection. No separate high-frequency cell state is modeled.';
 end
end
springPreload=NaN;cellSeparateState=~externalLowerSpringPair && ~q.mechanics.lowerRigid;
if externalLowerSpringPair,springPreload=q.springs.commonPreload_N;end
inverse=field_or(q.inertial,'lowerLoadInverse',struct('enabled',false, ...
 'scale',0,'cutoff_Hz',NaN,'filterDamping',NaN, ...
 'identifiedStiffness_N_m',NaN,'identifiedDamping_Ns_m',NaN,'identifiedMassB_kg',NaN, ...
 'identifiedForceConstant_N_A',NaN));
notch=field_or(q.inertial,'currentNotch',struct('enabled',false, ...
 'frequency_Hz',NaN,'zeroDamping',NaN,'poleDamping',NaN));
feedbackFilter=field_or(q.inertial,'feedbackFilter',struct('enabled',false, ...
 'cutoff_Hz',NaN,'damping',NaN));
variation=field_or(q.robot.forceGate,'variation',struct('enabled',false,'rmsLimit_N',NaN));
row=struct('id',id,'scenario',config.scenario,'massS_kg',q.mechanics.mS, ...
 'massB_kg',q.mechanics.mB,'totalBS_kg',q.mechanics.mS+q.mechanics.mB, ...
 'lowerRigid',q.mechanics.lowerRigid,'upperRigid',q.mechanics.upperRigid, ...
 'externalLowerSpringPair',externalLowerSpringPair, ...
 'lowerLinkStiffness_N_m',q.mechanics.k1,'lowerLinkDamping_Ns_m',q.mechanics.c1, ...
 'cellStiffness_N_m',cellStiffness,'cellDamping_Ns_m',cellDamping, ...
 'cellStiffnessInterpretation',cellMeaning, ...
 'cellSeparateNativeState',cellSeparateState, ...
 'lowerPairIndividualSpringRate_N_m',NaN, ...
 'lowerInverseEnabled',inverse.enabled,'lowerInverseScale',inverse.scale, ...
 'lowerInverseCutoff_Hz',inverse.cutoff_Hz,'lowerInverseFilterDamping',inverse.filterDamping, ...
 'lowerInverseIdentifiedStiffness_N_m',inverse.identifiedStiffness_N_m, ...
 'lowerInverseIdentifiedDamping_Ns_m',inverse.identifiedDamping_Ns_m, ...
 'lowerInverseIdentifiedMassB_kg',inverse.identifiedMassB_kg, ...
 'lowerInverseIdentifiedForceConstant_N_A',inverse.identifiedForceConstant_N_A, ...
 'guideViscous_Ns_m',q.coil.viscousFriction, ...
 'guideCoulomb_N',q.coil.coulombForce,'guideFrictionSpeed_m_s',q.coil.frictionSpeed, ...
 'actuatorParallelDamperAdded_Ns_m',max(0,q.coil.viscousFriction-2), ...
 'actuatorParallelDamperInterpretation','Any added damping above intrinsic2Ns/m is an external A-B viscous element, not a force-cell bypass.', ...
 'reference_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi, ...
 'requestedStopTime_s',q.sim.stopTime,'surfaceKind',q.surface.kind, ...
 'mu',q.contact.mu,'seed',q.sensor.seed,'seedRequested',config.seedRequested, ...
 'geometryMode',field_or(q.contact,'geometryMode','direct_lateral'), ...
 'dragRatioFilter_s',field_or(q.contact,'dragRatioFilter_s',NaN), ...
 'dragRatioMinForce_N',field_or(q.contact,'dragRatioMinForce_N',NaN), ...
 'forceNoiseRms_N',q.sensor.noiseRms,'positionNoiseRms_m',q.sensor.strokeNoiseRms, ...
 'lateralNoiseRms_N',q.contact.lateralNoiseRms_N,'accelNoiseRms_m_s2',q.accel.noiseRms, ...
 'forceFilterTime_s',q.sensor.filterTime,'forceDelay_s',q.sensor.delay, ...
 'positionDelay_s',q.sensor.strokeDelay,'accelFilterTime_s',q.accel.filterTime, ...
 'accelDelay_s',q.accel.delay,'accelQuantum_m_s2',q.accel.quantum, ...
 'accelRange_g',q.accel.range_g, ...
 'currentResponse_s',q.coil.currentResponse,'simulationStep_s',q.sim.step, ...
 'forceKp_A_N',q.inertial.forceKp,'forceKi_A_Ns',q.inertial.forceKi, ...
 'forceKd_A_s_N',q.inertial.forceKd,'derivativeFilter_s',q.inertial.derivativeFilter, ...
 'accelerationFeedforwardScale',q.inertial.accelerationFeedforwardScale, ...
 'geometryFeedforwardScale',q.contact.geometryFeedforwardScale, ...
 'guideFrictionFeedforwardScale',q.inertial.guideFrictionFeedforwardScale, ...
 'guideVelocityPrediction_s',field_or(q.inertial,'guideVelocityPrediction_s',0), ...
 'accelerationPrediction_s',field_or(q.inertial,'accelerationPrediction_s',0), ...
 'accelerationDerivativeFilter_s',field_or(q.inertial,'accelerationDerivativeFilter_s',NaN), ...
 'accelerationFeedforwardFilter_s',field_or(q.inertial,'accelerationFeedforwardFilter_s',0), ...
 'currentNotchEnabled',notch.enabled,'currentNotchFrequency_Hz',notch.frequency_Hz, ...
 'currentNotchZeroDamping',notch.zeroDamping,'currentNotchPoleDamping',notch.poleDamping, ...
 'feedbackFilterEnabled',feedbackFilter.enabled, ...
 'feedbackFilterCutoff_Hz',feedbackFilter.cutoff_Hz, ...
 'feedbackFilterDamping',feedbackFilter.damping, ...
 'positionBandwidth_Hz',q.inertial.positionBandwidth_Hz, ...
 'voltageLimit_V',q.coil.voltage,'currentLimit_A',q.coil.peakCurrent, ...
 'executed',~isempty(r),'passed',false,'completed',false,'lostContact',false,'strokeStop',false, ...
 'runtime_s',runtime,'endTime_s',NaN,'nativeSamples',0,'peakNormalForceError_N',NaN, ...
 'rmsNormalForceError_N',NaN,'peakNormalForceErrorTime_s',NaN,'peakBeforeBumpError_N',NaN, ...
 'peakDuringBumpError_N',NaN,'peakAfterBumpError_N',NaN, ...
 'peakStartupError_N',NaN,'peakSettledFlatError_N',NaN, ...
 'peakBumpPlusRecoveryError_N',NaN, ...
 'peakNormalEstimateTrackingError_N',NaN,'peakNormalEstimateError_N',NaN, ...
 'rmsNormalEstimateError_N',NaN,'peakSlopeEstimateError',NaN, ...
 'peakReconstructedAxialError_N',NaN,'peakReconstructedLateralError_N',NaN, ...
 'peakMeasuredAzError_m_s2',NaN,'rmsMeasuredAzError_m_s2',NaN, ...
 'peakMeasuredAxError_m_s2',NaN,'peakPhysicalAz_m_s2',NaN,'peakPhysicalAx_m_s2',NaN, ...
 'peakPredictedFFAzError_m_s2',NaN,'flatPredictedFFAzRms_m_s2',NaN, ...
 'peakFilteredFFAzError_m_s2',NaN,'flatFilteredFFAzRms_m_s2',NaN, ...
 'peakSurfaceAcceleration_m_s2',NaN,'peakFiniteDifferenceSurfaceJerk_m_s3',NaN, ...
 'peakAxialDiagnosticError_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN, ...
 'peakRequestedCurrent_A',NaN,'peakAppliedCurrentCommand_A',NaN, ...
 'peakCurrentTrackingError_A',NaN,'rmsCurrentTrackingError_A',NaN, ...
 'currentTrackingAxialForceAtNormalPeak_N',NaN, ...
 'rawAcquisitionEnabled',false,'rawAcquisitionPeriod_s',q.sensor.sampleTime, ...
 'rawAntiAliasCutoff_Hz',NaN,'rawAntiAliasDamping',NaN, ...
 'rawAcquisitionEMA_s',NaN,'rawAcquisitionPipeline_s',NaN, ...
 'identifiedAcquisitionAntiAliasCutoff_Hz',NaN,'identifiedAcquisitionEMA_s',NaN, ...
 'identifiedAcquisitionPipeline_s',NaN,'forceControllerPeriod_s',q.force.sampleTime, ...
 'forceVarianceGateEnabled',variation.enabled, ...
 'forceVarianceGateRmsLimit_N',variation.rmsLimit_N, ...
 'peakVoltage_V',NaN,'voltageClipFraction',NaN,'effortClipFraction',NaN, ...
 'minTravel_mm',NaN,'maxTravel_mm',NaN,'gateReadyFraction',NaN, ...
 'peakAddedDamperForce_N',NaN,'peakAddedDamperPower_W',NaN, ...
 'meanAddedDamperPower_W',NaN,'addedDamperEnergy_J',NaN, ...
 'normalBalanceResidual_N',NaN,'lateralBalanceResidual_N',NaN, ...
 'rigidAxialBalanceResidual_N',NaN,'generalAxialBalanceResidual_N',NaN, ...
 'contactConstrainedBalanceSamples',0,'terminalMomentumResidual_N',NaN, ...
 'nativeActuatorForceResidual_N',NaN, ...
 'springPackingPassed',true,'minimumCommonSpringPreload_N',NaN, ...
 'commonSpringPreload_N',springPreload, ...
 'maximumCommonSpringPreload_N',NaN,'combinedFreeSpringLength_mm',0, ...
 'combinedInstalledSpringLength_mm',0, ...
 'peakLowerInverseCurrent_A',NaN,'lowerInverseInputIdentityResidual_N',NaN, ...
 'measuredDragNormalRequestResidual_N',NaN, ...
 'axialDiagnosticReason','','reason',exception);
if externalLowerSpringPair,row.lowerPairIndividualSpringRate_N_m=q.springs.lower.rate_N_m;end
if isfield(q,'acquisition') && q.acquisition.enabled
 a=q.acquisition;row.rawAcquisitionEnabled=true;
 row.rawAcquisitionPeriod_s=a.rawSampleTime_s;
 row.rawAntiAliasCutoff_Hz=a.antiAliasCutoff_Hz;row.rawAntiAliasDamping=a.antiAliasDamping;
 row.rawAcquisitionEMA_s=a.emaTime_s;row.rawAcquisitionPipeline_s=a.pipelineDelay_s;
 row.identifiedAcquisitionAntiAliasCutoff_Hz=a.identifiedAntiAliasCutoff_Hz;
 row.identifiedAcquisitionEMA_s=a.identifiedEmaTime_s;
 row.identifiedAcquisitionPipeline_s=a.identifiedPipelineDelay_s;
end
if isempty(r),return;end
metrics=r.metrics;normal=r.normalContact.data;accel=r.accelerometer.data;
addedDamping=max(0,q.coil.viscousFriction-2);
damperPower=addedDamping*r.data(:,3).^2;
row.peakAddedDamperForce_N=addedDamping*max(abs(r.data(:,3)));
row.peakAddedDamperPower_W=max(damperPower);
row.addedDamperEnergy_J=trapz(r.time,damperPower);
row.meanAddedDamperPower_W=row.addedDamperEnergy_J/max(eps,r.time(end)-r.time(1));
error=normal(:,1)-q.force.reference;
[row.peakNormalForceError_N,which]=max(abs(error));row.peakNormalForceErrorTime_s=r.time(which);
row.rmsNormalForceError_N=sqrt(mean(error.^2));row.completed=metrics.completed;
row.lostContact=metrics.lostContact;row.strokeStop=metrics.strokeStopHit;
row.passed=metrics.completed && row.peakNormalForceError_N<=q.force.tolerance && ...
 ~metrics.lostContact && ~metrics.strokeStopHit && ~metrics.overCurrent && metrics.impactCount==0;
if externalLowerSpringPair
 [row.springPackingPassed,row.minimumCommonSpringPreload_N, ...
  row.maximumCommonSpringPreload_N,row.combinedFreeSpringLength_mm, ...
  row.combinedInstalledSpringLength_mm]=spring_packing(r);
 row.passed=row.passed && row.springPackingPassed;
end
row.endTime_s=r.time(end);row.nativeSamples=numel(r.time);
start=q.surface.startTime;stop=start+r.parameters.d.duration;
row.peakBeforeBumpError_N=masked_peak(error,r.time<start);
row.peakDuringBumpError_N=masked_peak(error,r.time>=start & r.time<=stop);
row.peakAfterBumpError_N=masked_peak(error,r.time>stop);
row.peakStartupError_N=masked_peak(error,r.time<=.25);
if strcmp(q.surface.kind,'repeated')
 phaseTime=mod(r.time-start,q.surface.repeatPeriod);
 bumpRecovery=r.time>=start-.05 & ...
  (phaseTime<=r.parameters.d.duration+.2 | phaseTime>=q.surface.repeatPeriod-.05);
else
 bumpRecovery=r.time>=start-.05 & r.time<=stop+.2;
end
row.peakSettledFlatError_N=masked_peak(error,r.time>.25 & ~bumpRecovery);
row.peakBumpPlusRecoveryError_N=masked_peak(error,bumpRecovery);
row.peakNormalEstimateTrackingError_N=max(abs(normal(:,4)-q.force.reference));
row.peakNormalEstimateError_N=max(abs(normal(:,4)-normal(:,1)));
row.rmsNormalEstimateError_N=sqrt(mean((normal(:,4)-normal(:,1)).^2));
row.peakSlopeEstimateError=max(abs(normal(:,5)-normal(:,3)));
row.peakReconstructedAxialError_N=max(abs(accel(:,5)-r.data(:,1)));
row.peakReconstructedLateralError_N=max(abs(accel(:,6)-normal(:,2)));
row.peakMeasuredAzError_m_s2=max(abs(accel(:,1)-accel(:,3)));
row.rmsMeasuredAzError_m_s2=sqrt(mean((accel(:,1)-accel(:,3)).^2));
row.peakMeasuredAxError_m_s2=max(abs(accel(:,2)-accel(:,4)));
row.peakPhysicalAz_m_s2=max(abs(accel(:,3)));row.peakPhysicalAx_m_s2=max(abs(accel(:,4)));
if strcmp(row.geometryMode,'measured_drag_ratio')
 assert(isfield(r,'measuredDrag'),'Measured-ratio mode requires its native log.');
 expectedRequest=q.force.reference*(1-r.measuredDrag.data(:,1).*normal(:,5))./ ...
  sqrt(1+normal(:,5).^2);
 row.measuredDragNormalRequestResidual_N=max(abs(expectedRequest-normal(:,6)));
 assert(row.measuredDragNormalRequestResidual_N<1e-10, ...
  'Requested axial force must use measured filtered ratio and estimated slope.');
end
if isfield(r,'accelerationFeedforward')
 ffAz=r.accelerationFeedforward.data(:,2);
 row.peakPredictedFFAzError_m_s2=max(abs(ffAz-accel(:,3)));
 flatPreBump=r.time>.25 & r.time<q.surface.startTime-.05;
 row.flatPredictedFFAzRms_m_s2=sqrt(mean(ffAz(flatPreBump).^2));
 if size(r.accelerationFeedforward.data,2)==3
  filteredFF=r.accelerationFeedforward.data(:,3);
  row.peakFilteredFFAzError_m_s2=max(abs(filteredFF-accel(:,3)));
  row.flatFilteredFFAzRms_m_s2=sqrt(mean(filteredFF(flatPreBump).^2));
 end
end
if isfield(r,'lowerInverse')
 audit=r.lowerInverse.data;
 row.peakLowerInverseCurrent_A=max(abs(audit(:,4)));
 row.lowerInverseInputIdentityResidual_N=max(abs(audit(:,3)-audit(:,1)+audit(:,2)));
 assert(row.lowerInverseInputIdentityResidual_N<1e-10, ...
  'The inverse must use desired axial deviation minus known S times measured acceleration.');
 assert(max(abs(audit(:,2)-q.sensor.distalMass_model_kg*r.accelerometer.data(:,1)))<1e-10, ...
  'The proper inverse must use the logged measured acceleration and identified S mass.');
 assert(max(abs(audit(:,1)-(normal(:,6)-q.force.reference)))<1e-10, ...
  'The proper inverse must use the shared measured-geometry requested axial force.');
end
row.peakSurfaceAcceleration_m_s2=max(abs(r.data(:,30)));
row.peakFiniteDifferenceSurfaceJerk_m_s3=max(abs(diff(r.data(:,30))./diff(r.time)));
row.peakAxialDiagnosticError_N=metrics.peakAbsForceError_N;
row.peakCurrent_A=metrics.peakCurrent_A;row.rmsCurrent_A=metrics.rmsCurrent_A;
row.peakRequestedCurrent_A=max(abs(r.data(:,33)));
appliedCurrentCommand=min(max(r.data(:,33),-q.coil.peakCurrent),q.coil.peakCurrent);
row.peakAppliedCurrentCommand_A=max(abs(appliedCurrentCommand));
currentTrackingError=r.data(:,14)-appliedCurrentCommand;
row.peakCurrentTrackingError_A=max(abs(currentTrackingError));
row.rmsCurrentTrackingError_A=sqrt(mean(currentTrackingError.^2));
row.currentTrackingAxialForceAtNormalPeak_N=q.coil.forceConstant*currentTrackingError(which);
row.peakVoltage_V=metrics.peakVoltage_V;row.voltageClipFraction=metrics.voltageSaturationFraction;
row.effortClipFraction=metrics.effortSaturationFraction;
row.minTravel_mm=metrics.minTravel_mm;row.maxTravel_mm=metrics.maxTravel_mm;
actualSlope=r.data(:,29)/q.surface.scanSpeed;
expectedNormal=r.data(:,1).*sqrt(1+actualSlope.^2)./(1-q.contact.mu*actualSlope);
expectedLateral=-expectedNormal.*(actualSlope+q.contact.mu)./sqrt(1+actualSlope.^2);
row.normalBalanceResidual_N=max(abs(expectedNormal-normal(:,1)));
row.lateralBalanceResidual_N=max(abs(expectedLateral-normal(:,2)));
momentumResidual=r.data(:,1)-r.data(:,13)- ...
 (q.mechanics.mB+q.mechanics.mS)*r.parameters.d.g- ...
 q.mechanics.mB*r.data(:,12)-q.mechanics.mS*accel(:,3);
% The selected model stops on contact loss. Its final event sample changes
% the force constraint; there is no simulated free-flight continuation.
% Keep that sample in every force/failure score, but validate the attached
% contact momentum identity only while its constraint remains active.
attachedContact=r.data(:,18)>=.5 & r.data(:,21)<.5;
assert(any(attachedContact),'No attached-contact samples to validate.');
row.contactConstrainedBalanceSamples=sum(attachedContact);
row.generalAxialBalanceResidual_N=max(abs(momentumResidual(attachedContact)));
row.terminalMomentumResidual_N=momentumResidual(end);
if q.mechanics.lowerRigid
 rigidResidual=r.data(:,1)-r.data(:,13)- ...
  (q.mechanics.mB+q.mechanics.mS)*(r.parameters.d.g+r.data(:,30));
 row.rigidAxialBalanceResidual_N=max(abs(rigidResidual(attachedContact)));
end
assert(max([row.normalBalanceResidual_N,row.lateralBalanceResidual_N,row.generalAxialBalanceResidual_N])<1e-8, ...
 'Native normal/lateral/axial force balances failed.');
if strcmp(q.actuator.kind,'voice_coil')
 expectedActuator=q.coil.forceConstant*r.data(:,14)- ...
  q.coil.viscousFriction*r.data(:,3)- ...
  q.coil.coulombForce*tanh(r.data(:,3)/q.coil.frictionSpeed);
 row.nativeActuatorForceResidual_N=max(abs(r.data(:,13)-expectedActuator));
 assert(row.nativeActuatorForceResidual_N<1e-8, ...
  'The native actuator must apply the recorded viscous/Coulomb coefficients.');
end
if isfield(r,'robotGate'),row.gateReadyFraction=mean(r.robotGate.ready);end
row.axialDiagnosticReason=metrics.reason;
reasons={};if row.peakNormalForceError_N>q.force.tolerance,reasons{end+1}='normal force error';end
if metrics.lostContact,reasons{end+1}='contact loss';end
if metrics.strokeStopHit,reasons{end+1}='travel stop';end
if metrics.overCurrent,reasons{end+1}='overcurrent';end
if ~metrics.completed,reasons{end+1}='incomplete';end
if ~row.springPackingPassed,reasons{end+1}='spring packing or working range';end
if isempty(reasons),row.reason='pass';else,row.reason=strjoin(reasons,', ');end
end

function rankings=rank_controllers(rows)
keys=arrayfun(@(r)[r.geometryMode ':' sprintf('%.17g,',[r.forceKp_A_N,r.forceKi_A_Ns,r.forceKd_A_s_N, ...
 r.accelerationFeedforwardScale,r.geometryFeedforwardScale,r.guideFrictionFeedforwardScale, ...
 r.positionBandwidth_Hz,r.currentResponse_s,r.guideVelocityPrediction_s, ...
 r.currentNotchEnabled,r.currentNotchFrequency_Hz,r.currentNotchZeroDamping, ...
 r.currentNotchPoleDamping,r.feedbackFilterEnabled, ...
 r.feedbackFilterCutoff_Hz,r.feedbackFilterDamping, ...
 r.accelerationPrediction_s,r.accelerationDerivativeFilter_s, ...
 r.accelerationFeedforwardFilter_s, ...
 r.dragRatioFilter_s,r.dragRatioMinForce_N, ...
 r.guideViscous_Ns_m,r.guideCoulomb_N,r.guideFrictionSpeed_m_s, ...
 r.lowerInverseEnabled,r.lowerInverseScale,r.lowerInverseCutoff_Hz, ...
 r.lowerInverseFilterDamping,r.lowerInverseIdentifiedStiffness_N_m, ...
 r.lowerInverseIdentifiedDamping_Ns_m,r.lowerInverseIdentifiedMassB_kg, ...
 r.lowerInverseIdentifiedForceConstant_N_A])],rows,'UniformOutput',false);
uniqueKeys=unique(keys,'stable');rankings=struct([]);
for j=1:numel(uniqueKeys)
 cohort=rows(strcmp(keys,uniqueKeys{j}));score=max([cohort.peakNormalForceError_N]);
 if ~all([cohort.executed]) || ~all([cohort.completed]),score=inf;end
 r=cohort(1);one=struct('forceKp_A_N',r.forceKp_A_N,'forceKi_A_Ns',r.forceKi_A_Ns, ...
  'forceKd_A_s_N',r.forceKd_A_s_N,'accelerationFeedforwardScale',r.accelerationFeedforwardScale, ...
  'geometryFeedforwardScale',r.geometryFeedforwardScale, ...
  'guideFrictionFeedforwardScale',r.guideFrictionFeedforwardScale, ...
  'positionBandwidth_Hz',r.positionBandwidth_Hz,'currentResponse_s',r.currentResponse_s, ...
  'guideVelocityPrediction_s',r.guideVelocityPrediction_s, ...
  'currentNotchEnabled',r.currentNotchEnabled, ...
  'currentNotchFrequency_Hz',r.currentNotchFrequency_Hz, ...
  'currentNotchZeroDamping',r.currentNotchZeroDamping, ...
  'currentNotchPoleDamping',r.currentNotchPoleDamping, ...
  'feedbackFilterEnabled',r.feedbackFilterEnabled, ...
  'feedbackFilterCutoff_Hz',r.feedbackFilterCutoff_Hz, ...
  'feedbackFilterDamping',r.feedbackFilterDamping, ...
  'accelerationPrediction_s',r.accelerationPrediction_s, ...
  'accelerationDerivativeFilter_s',r.accelerationDerivativeFilter_s, ...
  'accelerationFeedforwardFilter_s',r.accelerationFeedforwardFilter_s, ...
  'geometryMode',r.geometryMode,'dragRatioFilter_s',r.dragRatioFilter_s, ...
  'dragRatioMinForce_N',r.dragRatioMinForce_N, ...
  'guideViscous_Ns_m',r.guideViscous_Ns_m, ...
  'guideCoulomb_N',r.guideCoulomb_N,'guideFrictionSpeed_m_s',r.guideFrictionSpeed_m_s, ...
  'lowerInverseEnabled',r.lowerInverseEnabled,'lowerInverseScale',r.lowerInverseScale, ...
  'lowerInverseCutoff_Hz',r.lowerInverseCutoff_Hz, ...
  'lowerInverseFilterDamping',r.lowerInverseFilterDamping, ...
  'lowerInverseIdentifiedStiffness_N_m',r.lowerInverseIdentifiedStiffness_N_m, ...
  'lowerInverseIdentifiedDamping_Ns_m',r.lowerInverseIdentifiedDamping_Ns_m, ...
  'lowerInverseIdentifiedMassB_kg',r.lowerInverseIdentifiedMassB_kg, ...
  'lowerInverseIdentifiedForceConstant_N_A',r.lowerInverseIdentifiedForceConstant_N_A, ...
  'actuatorParallelDamperAdded_Ns_m',r.actuatorParallelDamperAdded_Ns_m, ...
  'cohortCases',numel(cohort),'passedCases',sum([cohort.passed]), ...
  'worstNormalForceError_N',score,'minimumForceMargin_N',.5-score,'rowIds',[cohort.id]);
 rankings=append(rankings,one);
end
if ~isempty(rankings),[~,order]=sort([rankings.worstNormalForceError_N]);rankings=rankings(order);end
end

function key=controller_key(q)
notch=field_or(q.inertial,'currentNotch',struct('enabled',false, ...
 'frequency_Hz',NaN,'zeroDamping',NaN,'poleDamping',NaN));
feedbackFilter=field_or(q.inertial,'feedbackFilter',struct('enabled',false, ...
 'cutoff_Hz',NaN,'damping',NaN));
inverse=field_or(q.inertial,'lowerLoadInverse',struct('enabled',false, ...
 'scale',0,'cutoff_Hz',NaN,'filterDamping',NaN, ...
 'identifiedStiffness_N_m',NaN,'identifiedDamping_Ns_m',NaN,'identifiedMassB_kg',NaN, ...
 'identifiedForceConstant_N_A',NaN));
key=[field_or(q.contact,'geometryMode','direct_lateral') ':' ...
 sprintf('%.17g,',[q.inertial.forceKp,q.inertial.forceKi,q.inertial.forceKd, ...
 q.inertial.accelerationFeedforwardScale,q.contact.geometryFeedforwardScale, ...
 q.inertial.guideFrictionFeedforwardScale,q.inertial.positionBandwidth_Hz,q.coil.currentResponse, ...
 field_or(q.inertial,'guideVelocityPrediction_s',0), ...
 notch.enabled,notch.frequency_Hz,notch.zeroDamping,notch.poleDamping, ...
 feedbackFilter.enabled,feedbackFilter.cutoff_Hz,feedbackFilter.damping, ...
 field_or(q.inertial,'accelerationPrediction_s',0), ...
 field_or(q.inertial,'accelerationDerivativeFilter_s',NaN), ...
 field_or(q.inertial,'accelerationFeedforwardFilter_s',0), ...
 field_or(q.contact,'dragRatioFilter_s',NaN), ...
 field_or(q.contact,'dragRatioMinForce_N',NaN), ...
 q.coil.viscousFriction,q.coil.coulombForce,q.coil.frictionSpeed, ...
 inverse.enabled,inverse.scale,inverse.cutoff_Hz,inverse.filterDamping, ...
 inverse.identifiedStiffness_N_m,inverse.identifiedDamping_Ns_m, ...
 inverse.identifiedMassB_kg,inverse.identifiedForceConstant_N_A])];
end

function [passed,low,high,totalFree,totalInstalled]=spring_packing(r)
% Same physical spring policy as the retained mechanics screen. A common
% preload interval is derived from the full native trajectory; it is not a
% different preload selected at each sample. The coordinator must also
% intersect these intervals across all qualification cases.
p=r.parameters;d=r.data;pack=p.springs;low=0;high=inf;totalFree=0;
links={'lower','upper'};rigid=[p.mechanics.lowerRigid,p.mechanics.upperRigid];
delta={d(:,7)-d(:,6),d(:,5)-d(:,26)};offset=[p.d.q10,p.d.q20];
stiffness=[p.mechanics.k1,p.mechanics.k2];excursionPass=true;
for j=1:2
 if rigid(j),continue;end
 spring=pack.(links{j});rate=spring.rate_N_m;
 shift=offset(j)/stiffness(j)+delta{j};largest=max(abs(shift));
 low=max(low,rate*(pack.unseatingClearance_m+largest));
 compression=min(spring.freeLength_m-spring.freeLengthTolerance_m- ...
  spring.minWorkingLength_m-pack.workingClearance_m,spring.forceEnvelope_N/rate);
 high=min(high,rate*(compression-largest));
 excursionPass=excursionPass && max(shift)-min(shift)<=spring.maxCyclicStroke_m+1e-12;
 totalFree=totalFree+2000*(spring.freeLength_m+spring.freeLengthTolerance_m);
end
passed=low<=high && excursionPass && totalFree<=1000*pack.maxCombinedLength_m+1e-9;
common=pack.commonPreload_N;
assert(isfinite(common) && common>=0,'Selected common spring preload must be explicit.');
passed=passed && common>=low-1e-12 && common<=high+1e-12;
totalInstalled=0;
for j=1:2
 if rigid(j),continue;end
 spring=pack.(links{j});
 totalInstalled=totalInstalled+2000*(spring.freeLength_m+spring.freeLengthTolerance_m-common/spring.rate_N_m);
end
passed=passed && totalInstalled<=1000*pack.maxCombinedLength_m+1e-9;
end

function result=compare_manual_native(a,b)
% Include every versioned native logger, including logs added after this
% recipe was written. Parameters are excluded because voltage is varied.
names={'time','data'};allNames=union(fieldnames(a),fieldnames(b),'stable');
for j=1:numel(allNames)
 name=allNames{j};sample=[];
 if isfield(a,name),sample=a.(name);elseif isfield(b,name),sample=b.(name);end
 if isstruct(sample) && isfield(sample,'time'),names{end+1}=name;end %#ok<AGROW>
end
differences=struct;exact=true;maximum=0;matchingDimensions=true;dimensionMismatches={};
for j=1:numel(names)
 name=names{j};assert(isfield(a,name) && isfield(b,name),'Native paired logger missing.');
 x=a.(name);y=b.(name);difference=0;
 if isstruct(x)
  fields=fieldnames(x);assert(isequal(fields,fieldnames(y)));
  for k=1:numel(fields)
   one=fields{k};
   if ~isequal(size(x.(one)),size(y.(one)))
    exact=false;matchingDimensions=false;difference=NaN;
    dimensionMismatches{end+1}=[name '.' one];continue; %#ok<AGROW>
   end
   exact=exact && isequaln(x.(one),y.(one));
   if isnumeric(x.(one)) || islogical(x.(one))
    delta=abs(double(x.(one)(:))-double(y.(one)(:)));
    if ~isempty(delta),difference=max(difference,max(delta));end
   end
  end
 else
  if ~isequal(size(x),size(y))
   exact=false;matchingDimensions=false;difference=NaN;
   dimensionMismatches{end+1}=name; %#ok<AGROW>
  else
   exact=exact && isequaln(x,y);
   delta=abs(double(x(:))-double(y(:)));if ~isempty(delta),difference=max(delta);end
  end
 end
 differences.(name)=difference;maximum=max(maximum,difference);
end
if ~matchingDimensions,maximum=NaN;end
result=struct('allManualHistoriesBitwiseIdentical',exact, ...
 'allManualHistoriesHaveMatchingDimensions',matchingDimensions, ...
 'dimensionMismatches',{dimensionMismatches}, ...
 'durationDifference_s',b.time(end)-a.time(end), ...
 'maximumManualHistoryDifference',maximum,'channelMaximumDifferences',differences);
end

function value=masked_peak(values,mask)
if any(mask),value=max(abs(values(mask)));else,value=NaN;end
end

function trace=plot_excerpt(r,row)
stride=max(1,round(250e-6/r.parameters.sim.step));take=1:stride:numel(r.time);
trace=struct('row',row,'plotStep_s',stride*r.parameters.sim.step,'time_s',r.time(take), ...
 'normalLog',r.normalContact.data(take,:),'accelerometerLog',r.accelerometer.data(take,:), ...
 'motionObserverLog',r.motionObserver.data(take,:), ...
 'current_A',r.data(take,14),'voltage_V',r.data(take,15), ...
 'axialContactForce_N',r.data(take,1),'taredAxialCell_N',r.data(take,32));
if isfield(r,'lowerInverse'),trace.lowerInverseLog=r.lowerInverse.data(take,:);end
end

function want=want_plot_excerpt(q,config,phase)
% Bound retained plot data in700-case workers. Complete worst histories for
% each controller stay independent of these explicitly reduced excerpts.
if ismember(phase,{'full_range_mass','endpoint_confirmation'})
 want=ismember(q.force.reference,[5 50]) && ...
  ismember(round(q.mechanics.angle*180/pi),[0 90]) && ...
  ismember(config.seedRequested,[0 1729]);
else
 want=config.seedRequested==0 || (q.mechanics.mS==2 && config.seedRequested==1729);
end
end

function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end

function s=defaults(s,d)
names=fieldnames(d);for j=1:numel(names),if ~isfield(s,names{j}),s.(names{j})=d.(names{j});end,end
end

function value=field_or(s,name,fallback)
if isfield(s,name),value=s.(name);else,value=fallback;end
end

function result=sha256(file)
if ~isfile(file),file=[file '.m'];end
assert(isfile(file),'Hash input missing.');single=char(39);
escaped=strrep(file,single,[single,char(34),single,char(34),single]);
[status,output]=system(['sha256sum -- ',single,escaped,single]);assert(status==0);
result=regexp(output,'[0-9a-f]{64}','match','once');assert(~isempty(result));
end

function finish(mdl,previousStorage,previousRecord,ownedStorage)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
if strcmp(Simulink.sdi.getStorageLocation,ownedStorage),Simulink.sdi.clear;end
Simulink.sdi.setStorageLocation(previousStorage);Simulink.sdi.setRecordData(previousRecord);
end
