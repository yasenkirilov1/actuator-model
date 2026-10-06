function report=qualify_inertial_actuators(sourceFolder,outputFolder,phase,options)
%QUALIFY_INERTIAL_ACTUATORS Fair native comparison with common inertial sensing.
% Private copies only. Full histories score true surface-normal force.
% The private screw extension retains winding states and reflected inertia;
% it never replaces the finite B-S sensor by an external spring preset.
if nargin<1 || isempty(sourceFolder)
 sourceFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v7-20260930/frozen-inertial-v7';
end
project=fileparts(fileparts(mfilename('fullpath')));
if nargin<2 || isempty(outputFolder),outputFolder=fullfile(project,'validation');end
if nargin<3 || isempty(phase),phase='smoke';end
if nargin<4,options=struct;end
options=defaults(options,struct('label',phase,'step_s',1e-5,'stopTime_s',4, ...
 'cellStiffness_N_m',64e6,'cellDampingRatio',.02,'candidateFile', ...
 fullfile(sourceFolder,'normal_contact_trial_candidate.mat'),'selectedScrewFile','', ...
 'resumeCheckpointFile','', ...
 'coilGains',[.03 20 0],'coilNotch',false,'notchZeroDamping',.01, ...
 'notchPoleDamping',.3,'guideVelocityPrediction_s',0,'saveEveryTrace',false, ...
 'feedbackFilterEnabled',false,'feedbackFilterCutoff_Hz',800, ...
 'accelerometerNoiseRms_m_s2',.0123,'geometryMode','measured_drag_ratio', ...
 'sensorContract','source','preserveSourceFrame',false,'verifySourceCoilParity',false));
assert(~usejava('desktop'),'Use an isolated MATLAB batch process.');
assert(isfile(options.candidateFile));
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
assert(isfile(sourceModel));if ~isfolder(outputFolder),mkdir(outputFolder);end
stem=['inertial_actuators_' options.label];resultFile=fullfile(outputFolder,[stem '.json']);
assert(~isfile(resultFile),'Choose a new label: retained evidence is immutable.');
stateRoot=fullfile(getenv('HOME'),'.local','state','actuator-model');
workerFolder=tempname(stateRoot);mkdir(workerFolder);
for name={'helpers','cache','code','sdi','traces'},mkdir(fullfile(workerFolder,name{1}));end
modelFile=fullfile(workerFolder,'acf_force_control_normal_trial.slx');copyfile(sourceModel,modelFile);
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile(options.candidateFile,fullfile(workerFolder,'input_candidate.mat'));
copyfile([mfilename('fullpath') '.m'],fullfile(workerFolder,'recipe_snapshot.m'));
prepare_private_helpers(fullfile(workerFolder,'helpers'));
addpath(fullfile(workerFolder,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator acf_select_controller;
oldThreads=maxNumCompThreads;maxNumCompThreads(2);
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(workerFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,oldThreads)); %#ok<NASGU>
set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error');
loaded=load(options.candidateFile,'p');base=loaded.p;
assert(base.accel.enabled && base.robotPrediction.enabled && ...
 strcmp(base.sensor.positionTarget,'tool_relative_A'),'Common causal inertial sensing is required.');
base.sim.step=options.step_s;base.sim.stopTime=options.stopTime_s;base.sim.evaluateFrom=0;
if options.preserveSourceFrame
 assert(strcmp(options.sensorContract,'source'),'A frozen frame must be used without silent sensor overrides.');
 assert(isfield(base.inertial,'lowerLoadInverse') && base.inertial.lowerLoadInverse.enabled);
else
base.accel.noiseRms=options.accelerometerNoiseRms_m_s2;
if strcmp(options.sensorContract,'matched_100us_50g')
 base.sensor.sampleTime=100e-6;base.sensor.filterTime=100e-6;
 base.sensor.delay=100e-6;base.sensor.strokeDelay=100e-6;
 base.accel.filterTime=100e-6;base.accel.delay=100e-6;
 base.accel.range_g=50;base.accel.quantum=.00468;
elseif ~strcmp(options.sensorContract,'source')
 error('Unknown sensor contract. Use source or matched_100us_50g.');
end
base.contact.geometryMode=options.geometryMode;
base.mechanics.upperRigid=true;base.mechanics.lowerRigid=false;
base.mechanics.k1=options.cellStiffness_N_m;base.mechanics.k2=0;
base.mechanics.c1=2*options.cellDampingRatio*sqrt(base.mechanics.k1*base.mechanics.mB);
base.mechanics.c2=0;if isfield(base,'springs'),base=rmfield(base,'springs');end
base.inertial.forceKp=options.coilGains(1);base.inertial.forceKi=options.coilGains(2);
base.inertial.forceKd=options.coilGains(3);base.inertial.guideVelocityPrediction_s=options.guideVelocityPrediction_s;
base.inertial.currentNotch.enabled=options.coilNotch;
base.inertial.currentNotch.frequency_Hz=sqrt(base.mechanics.k1/base.mechanics.mB)/(2*pi);
base.inertial.currentNotch.zeroDamping=options.notchZeroDamping;
base.inertial.currentNotch.poleDamping=options.notchPoleDamping;
base.inertial.feedbackFilter=struct('enabled',options.feedbackFilterEnabled, ...
 'cutoff_Hz',options.feedbackFilterCutoff_Hz,'damping',1/sqrt(2));
end
base.screw=realistic_screw(base.screw);
base.screw.controlStrategy='speed';base.screw.velocityFeedforwardScale=0;
base.screw.currentForceKp=.03*base.coil.forceConstant*base.screw.lead/(2*pi*base.screw.torqueConstant);
base.screw.currentForceKi=20*base.coil.forceConstant*base.screw.lead/(2*pi*base.screw.torqueConstant);
base.screw.currentForceKd=0;base.screw.currentNotch.enabled=false;
base.screw.currentNotch.zeroDamping=options.notchZeroDamping;
base.screw.currentNotch.poleDamping=options.notchPoleDamping;
if ~isempty(options.selectedScrewFile)
 chosen=load(options.selectedScrewFile,'p');base.screw=independent_screw_filter(chosen.p);
end
if strcmp(phase,'carrier_inverse_comparison_v1')
 assert(options.preserveSourceFrame && options.verifySourceCoilParity);
 base.screw.lead=.010;base.screw.peakCurrent=5;base.screw.controlStrategy='current';
 kax=base.screw.torqueConstant/(base.screw.lead/(2*pi));
 base.screw.currentForceKp=6/kax;base.screw.currentForceKi=400/kax;base.screw.currentForceKd=0;
 base.screw.feedbackFilter.kind='common';base.screw.currentNotch.enabled=false;
 base.screw.accelerationPrediction_s=base.inertial.accelerationPrediction_s;
 base.screw.accelerationDerivativeFilter_s=base.inertial.accelerationDerivativeFilter_s;
 base.screw.accelerationFeedforwardFilter_s=0;
end
base.coil.strategy='pi';
mw=get_param(mdl,'ModelWorkspace');base=acf_prepare(base);assignin(mw,'p',base);
set_param(mdl,'FixedStep',num2str(base.sim.step,17),'StopTime',num2str(base.sim.stopTime,17));
sourceCoil=[];sourceCoilParameters=[];
if options.verifySourceCoilParity
 sourceCoilParameters=physical_case(base,2,50,90,1729);
 sourceCoilParameters.surface.kind='sharp_bump';sourceCoilParameters.surface.height=.001;
 sourceCoilParameters.surface.width=.010;sourceCoilParameters.surface.scanSpeed=.100;
 sourceCoilParameters.contact.commandedScanSpeed_m_s=.100;sourceCoilParameters.actuator.kind='voice_coil';
 sourceCoilParameters=acf_prepare(sourceCoilParameters);assignin(mw,'p',sourceCoilParameters);
 acf_select_actuator(mdl,'voice_coil');sourceCoil=acf_run(mdl,sourceCoilParameters);
 save(fullfile(outputFolder,[stem '_unmodified_source_coil.mat']),'sourceCoil','sourceCoilParameters','-v7');
 assignin(mw,'p',base);set_param(mdl,'FastRestart','off');
end
extend_screw_native(mdl);
acf_select_actuator(mdl,'voice_coil');
set_param(mdl,'SimulationCommand','update');
assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));
save_system(mdl,modelFile);extendedSHA=sha256(modelFile);
helperFiles=dir(fullfile(workerFolder,'helpers','*.m'));helperHashes=struct([]);
for j=1:numel(helperFiles)
 helperHashes=append(helperHashes,struct('file',helperFiles(j).name, ...
 'sha256',sha256(fullfile(helperFiles(j).folder,helperFiles(j).name))));
end
report=struct('date','2026-09-30','phase',phase,'status','prepared','options',options, ...
 'sourceFolder',sourceFolder,'sourceModelSHA256',sha256(sourceModel), ...
 'privateModelFile',modelFile,'privateModelSHA256',extendedSHA, ...
 'privateWorkerFolder',workerFolder,'candidateSHA256',sha256(options.candidateFile), ...
 'recipeSHA256',sha256(fullfile(workerFolder,'recipe_snapshot.m')), ...
 'recipeFile',fullfile(workerFolder,'recipe_snapshot.m'),'helperHashes',helperHashes, ...
 'commonSensorContract','B-S multi-axis cell plus measured S acceleration and S-A position; no surface preview', ...
 'sensorProvision',base.sensor,'accelerometerProvision',base.accel, ...
 'coilProvision',base.coil,'inertialProvision',base.inertial, ...
 'robotProvision',base.robot,'contactProvision',base.contact, ...
 'metric','max(abs(native normal force - Fref)) over every native time including startup; <=0.5N and no stops/overcurrent', ...
 'screwPlantEquation','Jax=Jrot/ell^2; L=q1-mB*(g+aR); udd=(torque/ell-L)/(mB+Jax); Fa=mB*udd+L', ...
 'rigidReferenceEquation','Fa=torque/ell-Jax*(aR-aS); counterfactual zero cell compliance', ...
 'screwProvision',base.screw,'screwThermalNote',sprintf( ...
 '1.16A continuous catalog; %.3gA peak is a drive assumption with no qualified allowed duration',base.screw.peakCurrent), ...
 'transmissionNote','Native ideal reversible pitch transform plus explicit Coulomb/viscous losses; 90% efficiency used only as conservative sizing bound', ...
 'speedLoopSensing','Inherited inner speed loop has exact shaft velocity: optimistic extra encoder assumption; current-domain control uses the common supplied measurements', ...
 'sources',{{'https://www.maxongroup.com/maxon/view/product/302005', ...
 'https://www.maxongroup.com/medias/sys_master/root/8992314097694/EN-22-154.pdf'}}, ...
 'plannedCases',0,'executedCases',0,'setupRejections',0,'passedCases',0,'algebraicLoopCount',0, ...
 'rows',struct([]));
report.sourceParityReferenceExecutions=double(options.verifySourceCoilParity);
configs=struct([]);
switch phase
 case 'build_only'
  % Retained native source/helpers for integration, with zero executions.
 case 'resume'
  assert(isfile(options.resumeCheckpointFile));prior=load(options.resumeCheckpointFile,'configs','rows');
  rejected=find(~[prior.rows.executed]);pending=(numel(prior.rows)+1):numel(prior.configs);
  indices=[rejected pending];configs=prior.configs(indices);
  report.resumeCheckpointSHA256=sha256(options.resumeCheckpointFile);
  report.originalCaseIndices=indices;
 case 'smoke'
  for rigid=[false true]
   for actuator={'voice_coil','leadscrew'}
    q=physical_case(base,2,50,90,0);q.actuator.kind=actuator{1};q.mechanics.lowerRigid=rigid;
    q.screw.controlStrategy='current';q.screw.currentNotch.enabled=false;
    add_config(q,['native_force_balance_' actuator{1}],'sharp');
   end
  end
 case 'preservation_proof'
  original=getVariable(mw,'pCompliant');
  for rigidA=[false true]
   for actuator={'voice_coil','leadscrew'}
    q=physical_case(base,1.4,20,0,0);q.actuator.kind=actuator{1};
    if ~rigidA
     fields={'k1','k2','c1','c2','lowerRigid','upperRigid'};
     for h=1:numel(fields),q.mechanics.(fields{h})=original.mechanics.(fields{h});end
     q.springs=original.springs;
    end
    q.screw.controlStrategy='current';q.sim.stopTime=1;
    add_config(q,'native_compliant_A_and_rigid_A_preservation','flat');
   end
  end
 case 'current_tune'
  % Thirty-six bounded tuning cases, one clean difficult corner per setting.
  % Gains expressed as equivalent coil-current feedback then converted to Nm.
  gains=[.03 20 0;.01 5 0;.003 1 0;.01 5 1e-6;.003 1 1e-6;.03 20 1e-6];
  for lead=[.003 .005 .010]
   for notch=[false true]
    for j=1:size(gains,1)
     q=physical_case(base,2,50,90,0);q.actuator.kind='leadscrew';q.screw.lead=lead;
     q.screw.controlStrategy='current';q.screw.currentNotch.enabled=notch;
     scale=base.coil.forceConstant*lead/(2*pi*q.screw.torqueConstant);
     q.screw.currentForceKp=gains(j,1)*scale;q.screw.currentForceKi=gains(j,2)*scale;
     q.screw.currentForceKd=gains(j,3)*scale;
     q.coil.strategy=choose_strategy(gains(j,3));add_config(q,'current_PI_PID_tuning','sharp');
    end
   end
 end
 case 'current_refine_v4'
  % Feedback rolloff only: prompt measured-motion FF bypasses this filter.
  % The fourth provision gives the 10mm lead sufficient transient current;
  % it remains thermally unsuitable for 50N continuous with this RE25 winding.
  assert(isfield(base.d,'feedbackFilter'),'Use the frozen v4 filter topology.');
  provisions=[.003 3.5;.005 3.5;.010 3.5;.010 5];
  gains=[.003 1 0;.01 5 0;.03 20 0];
  for h=1:size(provisions,1)
   for cutoff=[300 800]
    for j=1:size(gains,1)
     q=physical_case(base,2,50,90,0);q.actuator.kind='leadscrew';
     q.screw.lead=provisions(h,1);q.screw.peakCurrent=provisions(h,2);
     q.screw.controlStrategy='current';q.screw.currentNotch.enabled=false;
     q.inertial.feedbackFilter.enabled=true;q.inertial.feedbackFilter.cutoff_Hz=cutoff;
     scale=base.coil.forceConstant*q.screw.lead/(2*pi*q.screw.torqueConstant);
     q.screw.currentForceKp=gains(j,1)*scale;q.screw.currentForceKi=gains(j,2)*scale;
     q.screw.currentForceKd=gains(j,3)*scale;q.coil.strategy='pi';
     add_config(q,'feedback_only_filter_current_PI_tuning','sharp');
    end
   end
 end
 case 'feedback_notch_v4'
  % A feedback-only notch suppresses the identified finite-cell mode without
  % delaying measured acceleration or geometry FF. Twelve bounded settings.
  provisions=[.005 3.5;.010 5];gains=[.03 20 0;.1 50 0];
  for h=1:size(provisions,1)
   for pole=[.1 .3 .5]
    for j=1:size(gains,1)
     q=physical_case(base,2,50,90,0);q.actuator.kind='leadscrew';
     q.screw.lead=provisions(h,1);q.screw.peakCurrent=provisions(h,2);
     q.screw.controlStrategy='current';q.screw.currentNotch.enabled=false;
     q.inertial.feedbackFilter.enabled=false;q.screw.feedbackFilter.kind='notch';
     q.screw.feedbackFilter.zeroDamping=.01;q.screw.feedbackFilter.poleDamping=pole;
     scale=base.coil.forceConstant*q.screw.lead/(2*pi*q.screw.torqueConstant);
     q.screw.currentForceKp=gains(j,1)*scale;q.screw.currentForceKi=gains(j,2)*scale;
     q.screw.currentForceKd=0;q.coil.strategy='pi';
     add_config(q,'feedback_only_notch_current_PI_tuning','sharp');
    end
   end
  end
 case 'corrected_selected_v6'
  assert(isfile(options.selectedScrewFile));chosen=load(options.selectedScrewFile,'p');selected=chosen.p;
  selected.screw=independent_screw_filter(selected);
  assert(getSimulinkBlockHandle([mdl '/Force controller/Screw normal PI and PID/Measured motion feedforward/Causal acceleration for current FF'])>0, ...
   'Use the frozen native causal acceleration prediction topology.');
  for prediction=[0 100e-6 200e-6 350e-6]
   for seed=[0 1729]
    q=physical_case(base,2,50,90,seed);q.actuator.kind='leadscrew';q.screw=selected.screw;
    q.screw.accelerationPrediction_s=prediction;q.coil.strategy=selected.coil.strategy;
    add_config(q,'corrected_loss_mapping_causal_acceleration_prediction','sharp');
   end
 end
 case 'corrected_prediction_filter_v6'
  assert(isfile(options.selectedScrewFile));chosen=load(options.selectedScrewFile,'p');selected=chosen.p;
  selected.screw=independent_screw_filter(selected);
  for prediction=[350e-6 450e-6]
   for derivativeFilter=[600e-6 900e-6]
    for seed=[0 1729]
     q=physical_case(base,2,50,90,seed);q.actuator.kind='leadscrew';q.screw=selected.screw;
     q.screw.accelerationPrediction_s=prediction;q.screw.accelerationDerivativeFilter_s=derivativeFilter;
     q.coil.strategy=selected.coil.strategy;
     add_config(q,'corrected_loss_mapping_prediction_derivative_rolloff','sharp');
    end
   end
 end
 case 'acceleration_command_filter_v7'
  assert(isfile(options.selectedScrewFile));chosen=load(options.selectedScrewFile,'p');selected=chosen.p;
  selected.screw=independent_screw_filter(selected);
  assert(getSimulinkBlockHandle([mdl '/Force controller/Screw normal PI and PID/Measured motion feedforward/Acceleration FF command filter'])>0, ...
   'Use the frozen v7 acceleration-command filter topology.');
  for tau=[.0007 .001 .0015]
   a=exp(-base.force.sampleTime/tau);prediction=base.force.sampleTime*a/(1-a)+.000308;
   for seed=[0 1729]
    q=physical_case(base,2,50,90,seed);q.actuator.kind='leadscrew';q.screw=selected.screw;
    q.screw.accelerationFeedforwardFilter_s=tau;q.screw.accelerationPrediction_s=prediction;
    q.screw.accelerationDerivativeFilter_s=.0003;q.coil.strategy=selected.coil.strategy;
    add_config(q,'causal_acceleration_prediction_plus_command_lowpass','sharp');
   end
  end
 case 'speed_tune'
  % Same common measurements; optional causal observed gap velocity / ell.
  gains=[5e-7 1e-5 0;5e-6 1e-4 0;2e-5 5e-4 0;5e-5 .001 0;5e-6 1e-4 1e-9;2e-5 5e-4 1e-9];
  for lead=[.003 .005 .010]
   for feedforward=[0 1]
    for j=1:size(gains,1)
     q=physical_case(base,2,50,90,0);q.actuator.kind='leadscrew';q.screw.lead=lead;
     q.screw.controlStrategy='speed';q.screw.velocityFeedforwardScale=feedforward;
     q.screw.forceKp=gains(j,1);q.screw.forceKi=gains(j,2);q.screw.forceKd=gains(j,3);
     q.coil.strategy=choose_strategy(gains(j,3));add_config(q,'speed_PI_PID_tuning','sharp');
    end
   end
  end
 case 'carrier_inverse_comparison_v1'
  cases=[2 50 90 0;2 50 90 1729;2 5 0 1729;.1 50 90 1729];
  for actuator={'voice_coil','leadscrew'}
   for j=1:size(cases,1)
    q=physical_case(base,cases(j,1),cases(j,2),cases(j,3),cases(j,4));
    q.actuator.kind=actuator{1};add_config(q,'source_frame_paired_actuators','sharp');
   end
  end
 case 'matched_sensor_selected_v7'
  assert(strcmp(options.sensorContract,'matched_100us_50g'), ...
   'The matched-sensor comparison requires its explicit 100us/50g provision.');
  assert(isfile(options.selectedScrewFile));chosen=load(options.selectedScrewFile,'p');selected=chosen.p;
  selected.screw=independent_screw_filter(selected);
  for prediction=[0 100e-6 200e-6]
   for seed=[0 1729]
    q=physical_case(base,2,50,90,seed);q.actuator.kind='leadscrew';
    q.screw=selected.screw;q.coil.strategy=selected.coil.strategy;
    q.screw.accelerationPrediction_s=prediction;
    add_config(q,'matched_100us_screw_prediction','sharp');
   end
  end
 case {'qualification','screw_qualification'}
  assert(isfile(options.selectedScrewFile),'Supply the retained selected screw p MAT.');
  chosen=load(options.selectedScrewFile,'p');selected=chosen.p;
  selected.screw=independent_screw_filter(selected);
  actuatorKinds={'voice_coil','leadscrew'};
  if strcmp(phase,'screw_qualification'),actuatorKinds={'leadscrew'};end
  for actuator=actuatorKinds
   for scenario={'sharp','slow'}
    for mass=[.1 2]
     for force=[5 50]
      for angle=[0 90]
       for seed=[0 1729]
        q=physical_case(base,mass,force,angle,seed);q.actuator.kind=actuator{1};
        if strcmp(actuator{1},'leadscrew')
         q.screw=selected.screw;q.coil.strategy=selected.coil.strategy;
         q.inertial.feedbackFilter=selected.inertial.feedbackFilter;
        end
        add_config(q,['selected_' actuator{1}],scenario{1});
       end
      end
     end
    end
   end
  end
 case 'refinement'
  assert(isfile(options.selectedScrewFile));chosen=load(options.selectedScrewFile,'p');selected=chosen.p;
  selected.screw=independent_screw_filter(selected);
  for step=[25e-6 12.5e-6 6.25e-6]
   q=physical_case(base,2,50,90,0);q.actuator.kind='leadscrew';q.screw=selected.screw;
   q.coil.strategy=selected.coil.strategy;q.inertial.feedbackFilter=selected.inertial.feedbackFilter;
   q.sim.step=step;add_config(q,'selected_screw_solver_refinement','sharp');
  end
 otherwise,error('Unknown actuator experiment phase.');
end
rows=struct([]);report.plannedCases=numel(configs);started=tic;
bestScore=inf;worstScore=-inf;best=[];worst=[];fastKey='';
for j=1:numel(configs)
 q=configs(j).p;r=[];exception='';timer=tic;
 key=sprintf('%s_%s_%s_%d_%d_%g',q.actuator.kind,q.screw.controlStrategy,q.coil.strategy, ...
  q.mechanics.lowerRigid,q.mechanics.upperRigid,q.sim.step);
 if ~strcmp(key,fastKey),set_param(mdl,'FastRestart','off');fastKey='';end
 try
  q=acf_prepare(q);assignin(mw,'p',q);acf_select_actuator(mdl,q.actuator.kind);
  if isempty(fastKey)
   set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
   set_param(mdl,'SimulationCommand','update');loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);
   assert(isempty(loops),'Private actuator extension introduced an algebraic loop.');
   set_param(mdl,'FastRestart','on');fastKey=key;
  end
  evalc('r=acf_run(mdl,q,false);');
  assert(isequal(r.time,r.normalContact.time,r.accelerometer.time,r.motionObserver.time));
  if isfield(q.inertial,'lowerLoadInverse')
   assert(isfield(r,'lowerInverse') && isequal(r.time,r.lowerInverse.time));
  end
  if options.verifySourceCoilParity && strcmp(q.actuator.kind,'voice_coil') && ...
    q.mechanics.mS==2 && q.force.reference==50 && q.mechanics.angle==pi/2 && q.sensor.noiseRms>0
   assert(isequal(sourceCoil.time,r.time));
   fields={'data','normalContact','accelerometer','motionObserver','measuredDrag','forceVariation','accelerationFeedforward','lowerInverse'};
   parity=struct;
   for z=1:numel(fields)
    field=fields{z};a=sourceCoil.(field);b=r.(field);
    if isstruct(a),a=a.data;b=b.data;end
    delta=max(abs(a(:)-b(:)));parity.(field)=delta;
    assert(delta==0,'Private screw extension changed source-coil history %s: %.12g.',field,delta);
   end
   report.sourceCoilParity=parity;clear sourceCoil;
  end
 catch err
  exception=getReport(err,'extended','hyperlinks','off');
  set_param(mdl,'FastRestart','off');fastKey='';
 end
 try
  row=measure(q,r,j,configs(j),toc(timer),exception);
 catch validationError
  if ~isempty(r)
   save(fullfile(outputFolder,sprintf('%s_unverified_native_%04d.mat',stem,j)),'r','q','-v7');
  end
  rethrow(validationError);
 end
 if ~isempty(r)
  if options.saveEveryTrace || j==1
   traceFile=fullfile(outputFolder,sprintf('%s_native_%04d.mat',stem,j));save(traceFile,'r','row','-v7');row.nativeTraceFile=traceFile;
  end
  if row.peakNormalForceError_N> worstScore,worstScore=row.peakNormalForceError_N;worst=r;worstRow=row;end
  if row.completed && row.peakNormalForceError_N<bestScore,bestScore=row.peakNormalForceError_N;best=r;bestRow=row;end
 end
 rows=append(rows,row);clear r;
 report.rows=rows;report.executedCases=sum([rows.executed]);report.setupRejections=sum(~[rows.executed]);
 report.passedCases=sum([rows.passed]);report.elapsedSeconds=toc(started);
 checkpoint('running');
 fprintf('INERTIAL_ACTUATORS %s %d/%d %s/%s pitch%.3gmm F%.0f S%.2g angle%.0f seed%d: %.9g N pass%d current%.6g V%.6g duration%.4g elapsed%.1fs\n', ...
  phase,j,numel(configs),row.actuator,row.controller,1000*row.screwPitch_m,row.referenceForce_N, ...
  row.massS_kg,row.angle_deg,row.seed,row.peakNormalForceError_N,row.passed,row.peakCurrent_A, ...
  row.peakVoltage_V,row.actualDuration_s,toc(started));
 if mod(j,4)==0,Simulink.sdi.clear;end
 if isfile(fullfile(workerFolder,'stop_after_case')),report.status='stopped_by_coordinator';break;end
end
set_param(mdl,'FastRestart','off');
assert(strcmp(sha256(modelFile),extendedSHA),'Private source bytes changed during native runs.');
assert(strcmp(sha256(sourceModel),report.sourceModelSHA256),'Frozen upstream changed during native runs.');
if ~isempty(best)
 report.bestCase=bestRow;save(fullfile(outputFolder,[stem '_best_native_trace.mat']),'best','bestRow','-v7');
 if strcmp(best.parameters.actuator.kind,'leadscrew'),p=best.parameters;save(fullfile(outputFolder,[stem '_selected_screw.mat']),'p');end
end
if ~isempty(worst),report.worstCase=worstRow;save(fullfile(outputFolder,[stem '_worst_native_trace.mat']),'worst','worstRow','-v7');end
if ~strcmp(report.status,'stopped_by_coordinator'),report.status='complete';end
checkpoint(report.status);
fprintf('INERTIAL_ACTUATORS_COMPLETE %s: %d executed, %d setup rejects, %d passes.\n', ...
 phase,report.executedCases,report.setupRejections,report.passedCases);

 function add_config(q,purpose,scenario)
  q.surface.kind='sharp_bump';q.surface.height=.001;q.surface.width=.010;q.surface.scanSpeed=.100;
  if strcmp(scenario,'slow')
   q.surface.kind='bump';q.surface.width=.060;q.surface.scanSpeed=.020;q.sim.stopTime=max(q.sim.stopTime,6);
  elseif strcmp(scenario,'flat'),q.surface.kind='flat';
  end
  q.contact.commandedScanSpeed_m_s=q.surface.scanSpeed;
  configs=append(configs,struct('p',q,'purpose',purpose,'scenario',scenario));
 end
 function checkpoint(status)
  report.status=status;write_json(resultFile,report);
  if ~isempty(rows),writetable(struct2table(rows,'AsArray',true),fullfile(outputFolder,[stem '_cases.csv']));end
  save(fullfile(outputFolder,[stem '_checkpoint.mat']),'report','configs','rows','-v7');
 end
end

function s=realistic_screw(s)
% Manufacturer nominal winding values; all transmission/drive losses below
% are explicit engineering assumptions, not qualification of this old motor.
s.motorReference='maxon RE25 302005 (NRND): winding reference only';
s.torqueConstant=.0234;s.backEmfConstant=.0234;s.resistance=2.32;s.inductance=.000238;
s.motorRotorInertia_kg_m2=1.08e-6;s.screwCouplerInertia_kg_m2=.5e-6;
s.rotorInertia=s.motorRotorInertia_kg_m2+s.screwCouplerInertia_kg_m2;
s.motorMass_kg=.130;s.continuousCurrent_A=1.16;s.continuousTorque_Nm=.0263;
s.lead=.005;s.currentResponse=.0001;s.peakCurrent=3.5;s.voltage=24;
s.speedBandwidth=1000;s.speedDamping=.9;s.speedAntiWindup=1000;
s.maxLinearSpeedRequest=.1;s.coulombTorque=.001;s.viscousTorque=1e-6;s.frictionSpeed=1;
s.sizingEfficiency=.9;s.forceKp=.0005;s.forceKi=.005;s.forceKd=0;
s.currentDerivativeFilter=.0003;s.currentNotch=struct('enabled',false,'frequency_Hz',1000, ...
 'zeroDamping',.01,'poleDamping',.3);
s.feedbackFilter=struct('kind','common','zeroDamping',.01,'poleDamping',.3);
s.accelerationPrediction_s=0;s.accelerationDerivativeFilter_s=.0003;
s.accelerationFeedforwardFilter_s=0;
end

function screw=independent_screw_filter(selected)
% Store controller filtering per actuator so changing the coil's feedback
% filter cannot silently retune the selected screw controller.
screw=selected.screw;
if ~isfield(screw,'accelerationFeedforwardFilter_s')
 screw.accelerationFeedforwardFilter_s=0;
 if isfield(selected.inertial,'accelerationFeedforwardFilter_s')
  screw.accelerationFeedforwardFilter_s=selected.inertial.accelerationFeedforwardFilter_s;
 end
end
if ~isfield(screw,'accelerationPrediction_s')
 screw.accelerationPrediction_s=0;screw.accelerationDerivativeFilter_s=.0003;
 if isfield(selected.inertial,'accelerationPrediction_s')
  screw.accelerationPrediction_s=selected.inertial.accelerationPrediction_s;
  screw.accelerationDerivativeFilter_s=selected.inertial.accelerationDerivativeFilter_s;
 end
end
if ~isfield(screw,'feedbackFilter') || strcmp(screw.feedbackFilter.kind,'common')
 f=selected.inertial.feedbackFilter;
 screw.feedbackFilter=struct('kind','bypass','cutoff_Hz',f.cutoff_Hz, ...
  'damping',f.damping,'zeroDamping',.01,'poleDamping',.3);
 if f.enabled,screw.feedbackFilter.kind='lowpass';end
end
end

function prepare_private_helpers(folder)
path=fullfile(folder,'acf_prepare.m');s=fileread(path);
old="assert(~(m.lowerRigid || m.upperRigid) || strcmp(p.actuator.kind,'voice_coil'), ...";
assert(contains(s,old));s=strrep(s,old, ...
 "assert(~(m.lowerRigid || m.upperRigid) || strcmp(p.actuator.kind,'voice_coil') || (strcmp(p.actuator.kind,'leadscrew') && m.upperRigid), ...");
s=strrep(s,'J = p.screw.rotorInertia+p.d.mu*p.d.ell^2;', ...
 'p.d.screwMovingMass=p.d.mu;if m.upperRigid,p.d.screwMovingMass=m.mB;end;J = p.screw.rotorInertia+p.d.screwMovingMass*p.d.ell^2;');
marker='p.d.coilControl=coil_controller(p);';assert(contains(s,marker));
extra=strjoin([string(marker); ...
 "if isfield(p.screw,'currentForceKp')"; ...
 " if ~isfield(p.screw,'accelerationPrediction_s'),p.screw.accelerationPrediction_s=0;p.screw.accelerationDerivativeFilter_s=.0003;end"; ...
 " ad=exp(-p.force.sampleTime/p.screw.accelerationDerivativeFilter_s);p.d.screwAccelerationPrediction=struct('derivativeNumerator',(1-ad)/p.force.sampleTime*[1,-1],'derivativeDenominator',[1,-ad]);"; ...
 " if ~isfield(p.screw,'accelerationFeedforwardFilter_s'),p.screw.accelerationFeedforwardFilter_s=0;end;af=p.screw.accelerationFeedforwardFilter_s;dt=p.force.sampleTime;"; ...
 " if af>0,a=exp(-dt/af);num=[1-a,0];den=[1,-a];else,num=[1,0];den=[1,0];end;p.d.screwAccelerationFeedforwardFilter=struct('numerator',num,'denominator',den);"; ...
 " c=p.screw.currentNotch;c.frequency_Hz=sqrt(m.k1/(m.mB+p.screw.rotorInertia/p.d.ell^2))/(2*pi);"; ...
 " if c.enabled"; ...
 "  w=tan(pi*c.frequency_Hz*p.force.sampleTime);den=[1+2*c.poleDamping*w+w^2,2*(w^2-1),1-2*c.poleDamping*w+w^2];num=[1+2*c.zeroDamping*w+w^2,2*(w^2-1),1-2*c.zeroDamping*w+w^2];num=num/den(1);den=den/den(1);"; ...
 " else,num=[1,0,0];den=[1,0,0];end"; ...
 " p.screw.currentNotch=c;p.d.screwCurrentNotch=struct('numerator',num,'denominator',den,'initialStates',ones(2,1)*p.d.screwHoldCurrent/sum(den));"; ...
 " if ~isfield(p.screw,'feedbackFilter'),p.screw.feedbackFilter=struct('kind','common','zeroDamping',.01,'poleDamping',.3);end"; ...
 " if ~isfield(p.inertial,'feedbackFilter'),p.inertial.feedbackFilter=struct('enabled',false,'cutoff_Hz',800,'damping',1/sqrt(2));end"; ...
 " f=p.inertial.feedbackFilter;if strcmp(p.screw.feedbackFilter.kind,'lowpass'),f=p.screw.feedbackFilter;f.enabled=true;elseif strcmp(p.screw.feedbackFilter.kind,'bypass'),f.enabled=false;end"; ...
 " if f.enabled,w=tan(pi*f.cutoff_Hz*p.force.sampleTime);den=[1+2*f.damping*w+w^2,2*(w^2-1),1-2*f.damping*w+w^2];num=w^2*[1,2,1]/den(1);den=den/den(1);else,num=[1,0,0];den=[1,0,0];end"; ...
 " p.d.screwFeedbackFilter=struct('numerator',num,'denominator',den,'initialStates',zeros(2,1));"; ...
 " if strcmp(p.screw.feedbackFilter.kind,'notch')"; ...
 "  f=p.screw.feedbackFilter;w=tan(pi*c.frequency_Hz*p.force.sampleTime);den=[1+2*f.poleDamping*w+w^2,2*(w^2-1),1-2*f.poleDamping*w+w^2];num=[1+2*f.zeroDamping*w+w^2,2*(w^2-1),1-2*f.zeroDamping*w+w^2];num=num/den(1);den=den/den(1);"; ...
 "  p.d.screwFeedbackFilter=struct('numerator',num,'denominator',den,'initialStates',zeros(2,1));"; ...
 " end"; ...
 " if strcmp(p.screw.controlStrategy,'current'),p.d.speedKp=0;p.d.speedKi=0;end"; ...
 " if isfield(p.inertial,'lowerLoadInverse')"; ...
 "  c=p.inertial.lowerLoadInverse;a=2/p.force.sampleTime;w=a*tan(pi*c.cutoff_Hz*p.force.sampleTime);quadratic=[a*a+2*c.filterDamping*w*a+w*w,2*(w*w-a*a),a*a-2*c.filterDamping*w*a+w*w];"; ...
 "  effectiveB=c.identifiedMassB_kg+p.screw.rotorInertia/p.d.ell^2;kax=p.screw.torqueConstant/p.d.ell;num=(effectiveB/kax)*w*w*a*a*[1,-2,1];k=c.identifiedStiffness_N_m;d=c.identifiedDamping_Ns_m;"; ...
 "  if d>0,num=conv(num,[1,1]);den=conv([d*a+k,-d*a+k],quadratic);else,den=k*quadratic;num=[num,0];den=[den,0];end"; ...
 "  c.numerator=num/den(1);c.denominator=den/den(1);c.identifiedMassB_kg=effectiveB;c.identifiedForceConstant_N_A=kax;p.d.screwLowerLoadInverse=c;"; ...
 " end"; ...
 "end"],newline);
s=strrep(s,marker,extra);write_text(path,s);
path=fullfile(folder,'acf_select_actuator.m');s=fileread(path);
start=strfind(s,'if index~=2&&rigidRequested');
stop=strfind(s,"if strcmp(p.coil.strategy,'observer')");
assert(numel(start)==1&&numel(stop)==1&&stop>start);
s=[s(1:start-1) sprintf(['if index==3&&rigidRequested\n' ...
 ' error(''The ideal-position reference requires an explicitly chosen compliant attachment preset.'');\nend\n']) s(stop:end)];
s=strrep(s,"if index==2,controlChoice='Coil PI and PID';else,controlChoice='Drive PI and PID';end", ...
 "if index==2,controlChoice='Coil PI and PID';elseif index==1&&isfield(p.screw,'controlStrategy')&&strcmp(p.screw.controlStrategy,'current'),controlChoice='Screw normal PI and PID';else,controlChoice='Drive PI and PID';end");
marker="selector=[mdl '/Actuator selection'];";assert(contains(s,marker));
extra=strjoin([ ...
 "if index==1"; ...
 " plant=[mdl '/Actuator and RABS mechanics/Actuator/Leadscrew'];"; ...
 " src=[plant '/Current request source'];if strcmp(p.screw.controlStrategy,'current'),v='Current command';else,v='Speed loop';end;if ~strcmp(get_param(src,'LabelModeActiveChoice'),v),set_param(src,'LabelModeActiveChoice',v);end"; ...
 " src=[plant '/Transmitted force mechanics'];if p.mechanics.lowerRigid,v='Rigid B-S reference';else,v='Finite B-S cell';end;if ~strcmp(get_param(src,'LabelModeActiveChoice'),v),set_param(src,'LabelModeActiveChoice',v);end"; ...
 "end"; ...
 " src=[mdl '/Actuator and RABS mechanics/Boundary load selection'];if getSimulinkBlockHandle(src)>0,if index==1&&p.mechanics.upperRigid,v='Rigid A';else,v='Compliant A';end;if ~strcmp(get_param(src,'LabelModeActiveChoice'),v),set_param(src,'LabelModeActiveChoice',v);end;end"; string(marker)],newline);
s=strrep(s,marker,extra);write_text(path,s);
end

function extend_screw_native(mdl)
% Solve the rigid-A constraint analytically with ordinary native blocks.
set_param([mdl '/Actuator and RABS mechanics/Actuator'],'PropagateVariantConditions','on');
parent=[mdl '/Actuator and RABS mechanics'];
delete_line(parent,'Spring load on actuator/1','Actuator/3');
block('simulink/Signal Routing/Variant Source',parent,'Boundary load selection',[555 580 615 670], ...
 'VariantControlMode','label');
set_param([parent '/Boundary load selection'],'VariantControls',{'Compliant A','Rigid A'});
set_param([parent '/Boundary load selection'],'LabelModeActiveChoice','Rigid A');
link(parent,'Spring load on actuator/1','Boundary load selection/1');
link(parent,'B to S spring and damper/1','Boundary load selection/2');
link(parent,'Boundary load selection/1','Actuator/3');
sys=[mdl '/Actuator and RABS mechanics/Actuator/Leadscrew'];
delete_line(sys,'Spring load/1','Send spring_load_N/1');
from(sys,'Robot imposed acceleration','robot_boundary_acceleration',[30 1200 220 1230]);
block('simulink/Sources/Constant',sys,'Axial gravity',[260 1220 410 1250],'Value','p.d.g');
block('simulink/Math Operations/Sum',sys,'Imposed acceleration plus gravity',[470 1180 510 1270],'Inputs','++');
block('simulink/Math Operations/Gain',sys,'Rigid-A moving B inertia',[570 1190 790 1250], ...
 'Gain','p.mechanics.mB*double(p.mechanics.upperRigid)');
block('simulink/Math Operations/Sum',sys,'Cell force minus imposed B inertia',[870 1090 910 1220],'Inputs','+-');
link(sys,'Robot imposed acceleration/1','Imposed acceleration plus gravity/1');
link(sys,'Axial gravity/1','Imposed acceleration plus gravity/2');
link(sys,'Imposed acceleration plus gravity/1','Rigid-A moving B inertia/1');
link(sys,'Spring load/1','Cell force minus imposed B inertia/1');
link(sys,'Rigid-A moving B inertia/1','Cell force minus imposed B inertia/2');
link(sys,'Cell force minus imposed B inertia/1','Send spring_load_N/1');
set_param([sys '/Moving mass and reflected rotor inertia'],'Gain','1/(p.d.screwMovingMass+p.screw.rotorInertia/p.d.ell^2)');
set_param([sys '/Reduced moving mass'],'Gain','p.d.screwMovingMass');
% The both-rigid reference uses imposed relative acceleration, not a stiff spring.
from(sys,'Rigid reference surface motion','surface_motion',[30 1370 220 1400]);
block('simulink/Signal Routing/Selector',sys,'Rigid reference S acceleration',[280 1360 510 1420], ...
 'InputPortWidth','3','IndexOptionArray',{'Index vector (dialog)'},'IndexParamArray',{'3'});
block('simulink/Math Operations/Sum',sys,'Rigid reference gap acceleration',[600 1330 640 1450],'Inputs','+-');
block('simulink/Math Operations/Gain',sys,'Rigid reference rotor axial inertia',[710 1340 970 1410], ...
 'Gain','p.screw.rotorInertia/p.d.ell^2');
block('simulink/Math Operations/Sum',sys,'Rigid reference transmitted force',[1090 1320 1130 1450],'Inputs','+-');
link(sys,'Rigid reference surface motion/1','Rigid reference S acceleration/1');
link(sys,'Robot imposed acceleration/1','Rigid reference gap acceleration/1');
link(sys,'Rigid reference S acceleration/1','Rigid reference gap acceleration/2');
link(sys,'Rigid reference gap acceleration/1','Rigid reference rotor axial inertia/1');
link(sys,'Screw torque to axial force/1','Rigid reference transmitted force/1');
link(sys,'Rigid reference rotor axial inertia/1','Rigid reference transmitted force/2');
% Available torque has already removed Coulomb/viscous losses.
block('simulink/Signal Routing/Variant Source',sys,'Transmitted force mechanics',[1300 1040 1360 1190], ...
 'VariantControlMode','label');
set_param([sys '/Transmitted force mechanics'],'VariantControls',{'Finite B-S cell','Rigid B-S reference'});
set_param([sys '/Transmitted force mechanics'],'LabelModeActiveChoice','Finite B-S cell');
ph=get_param([sys '/Transmitted force'],'PortHandles');delete_line(get_param(ph.Inport,'Line'));
link(sys,'Force transmitted to the bodies/1','Transmitted force mechanics/1');
link(sys,'Rigid reference transmitted force/1','Transmitted force mechanics/2');
link(sys,'Transmitted force mechanics/1','Transmitted force/1');
% Preserve original speed-loop output, or accept an explicit current command.
delete_line(sys,'Speed PI current request/1','Send raw_current_request_A/1');
delete_line(sys,'Speed PI current request/1','Current request limit/1');
block('simulink/Signal Routing/Variant Source',sys,'Current request source',[1250 150 1310 280], ...
 'VariantControlMode','label');
set_param([sys '/Current request source'],'VariantControls',{'Speed loop','Current command'});
set_param([sys '/Current request source'],'LabelModeActiveChoice','Speed loop');
link(sys,'Speed PI current request/1','Current request source/1');
link(sys,'Command/1','Current request source/2');
link(sys,'Current request source/1','Send raw_current_request_A/1');
link(sys,'Current request source/1','Current request limit/1');
Simulink.BlockDiagram.arrangeSystem(sys,'FullLayout','true');
% Optional speed feedforward uses the common causal head-motion estimator.
sys=[mdl '/Force controller/Drive PI and PID'];
delete_line(sys,'PI plus holding command/1','Actuator command limits/1');
delete_line(sys,'PI plus holding command/1','Send raw_command/1');
from(sys,'Observed head gap velocity','inertial_head_gap_velocity',[30 700 300 730]);
block('simulink/Math Operations/Gain',sys,'Measured velocity feedforward',[400 680 700 750], ...
 'Gain','p.screw.velocityFeedforwardScale/p.d.ell');
block('simulink/Math Operations/Sum',sys,'PI PID plus measured velocity',[810 660 860 760],'Inputs','++');
link(sys,'Observed head gap velocity/1','Measured velocity feedforward/1');
link(sys,'PI plus holding command/1','PI PID plus measured velocity/1');
link(sys,'Measured velocity feedforward/1','PI PID plus measured velocity/2');
link(sys,'PI PID plus measured velocity/1','Actuator command limits/1');
link(sys,'PI PID plus measured velocity/1','Send raw_command/1');
Simulink.BlockDiagram.arrangeSystem(sys,'FullLayout','true');
% A simple current-domain alternative uses the same common sensors as the coil.
source=[mdl '/Force controller/Coil PI and PID'];sys=[mdl '/Force controller/Screw normal PI and PID'];
add_block(source,sys,'VariantControl','Screw normal PI and PID');
inverse=[sys '/Lower spring inverse feedforward/Proper identified lower-link inverse'];
if getSimulinkBlockHandle(inverse)>0
 set_param(inverse,'Numerator','p.d.screwLowerLoadInverse.numerator', ...
  'Denominator','p.d.screwLowerLoadInverse.denominator');
end
gains=find_system(sys,'FollowLinks','off','LookUnderMasks','all','BlockType','Gain');
for j=1:numel(gains)
 value=get_param(gains{j},'Gain');value=strrep(value,'p.inertial.forceKp','p.screw.currentForceKp');
 value=strrep(value,'p.inertial.forceKi','p.screw.currentForceKi');value=strrep(value,'p.inertial.forceKd','p.screw.currentForceKd');
 value=strrep(value,'p.inertial.accelerationPrediction_s','p.screw.accelerationPrediction_s');
 value=strrep(value,'p.inertial.accelerationDerivativeFilter_s','p.screw.accelerationDerivativeFilter_s');
 value=strrep(value,'p.inertial.derivativeFilter','p.screw.currentDerivativeFilter');
 value=strrep(value,'p.coil.forceConstant','(p.screw.torqueConstant/p.d.ell)');
 value=strrep(value,'p.coil.viscousFriction','(p.screw.viscousTorque/p.d.ell^2)');
 value=strrep(value,'p.coil.coulombFriction','(p.screw.coulombTorque/p.d.ell)');
 value=strrep(value,'p.coil.coulombForce','(p.screw.coulombTorque/p.d.ell)');
 value=strrep(value,'p.coil.frictionSpeed','(p.screw.frictionSpeed*p.d.ell)');
 assert(~contains(strrep(value,'p.coil.strategy',''),'p.coil.'), ...
  'Unmapped coil-specific gain in the screw controller: %s',value);
 set_param(gains{j},'Gain',value);
end
set_param([sys '/Flat holding current'],'Value','p.d.screwHoldCurrent');
set_param([sys '/Current limits'],'UpperLimit','p.screw.peakCurrent','LowerLimit','-p.screw.peakCurrent');
set_param([sys '/Optional structural-mode current notch'],'Numerator','p.d.screwCurrentNotch.numerator', ...
 'Denominator','p.d.screwCurrentNotch.denominator','InitialStates','p.d.screwCurrentNotch.initialStates');
feedback=[sys '/Normal force PI and PID/Optional feedback rolloff'];
if getSimulinkBlockHandle(feedback)>0
 set_param(feedback,'Numerator','p.d.screwFeedbackFilter.numerator', ...
  'Denominator','p.d.screwFeedbackFilter.denominator','InitialStates','p.d.screwFeedbackFilter.initialStates', ...
  'Name','Feedback structural-mode filter');
end
ff=[sys '/Measured motion feedforward'];
derivative=[ff '/Filtered measured acceleration rate'];
if getSimulinkBlockHandle(derivative)>0
 set_param(derivative,'Numerator','p.d.screwAccelerationPrediction.derivativeNumerator', ...
  'Denominator','p.d.screwAccelerationPrediction.derivativeDenominator');
end
commandFilter=[ff '/Acceleration FF command filter'];
if getSimulinkBlockHandle(commandFilter)>0
 set_param(commandFilter,'Numerator','p.d.screwAccelerationFeedforwardFilter.numerator', ...
  'Denominator','p.d.screwAccelerationFeedforwardFilter.denominator');
end
set_param([ff '/Measured acceleration current feedforward'],'Gain', ...
 '-p.inertial.accelerationFeedforwardScale*(p.mechanics.mB+p.sensor.distalMass_model_kg+p.screw.rotorInertia/p.d.ell^2)*p.d.ell/p.screw.torqueConstant');
block('simulink/Math Operations/Gain',ff,'Robot acceleration rotor current',[600 600 920 670], ...
 'Gain','p.inertial.accelerationFeedforwardScale*p.screw.rotorInertia/(p.d.ell*p.screw.torqueConstant)');
block('simulink/Math Operations/Sum',ff,'Combined measured acceleration current',[1100 600 1150 720],'Inputs','++');
delete_line(ff,'Measured acceleration current feedforward/1','acceleration_current_A/1');
link(ff,'Robot acceleration/1','Robot acceleration rotor current/1');
link(ff,'Robot acceleration rotor current/1','Combined measured acceleration current/1');
link(ff,'Measured acceleration current feedforward/1','Combined measured acceleration current/2');
link(ff,'Combined measured acceleration current/1','acceleration_current_A/1');
Simulink.BlockDiagram.arrangeSystem(ff,'FullLayout','true');
end

function p=physical_case(p,mass,force,angle,seed)
p.mechanics.mS=mass;p.mechanics.mB=.6;p.sensor.distalMass_kg=mass;p.sensor.distalMass_model_kg=mass;
p.force.reference=force;p.mechanics.angle=angle*pi/180;p.contact.mu=.3;
p.sensor.seed=max(seed,7);p.sensor.noiseRms=.02;p.sensor.strokeNoiseRms=1e-6;
p.contact.lateralNoiseRms_N=.02;
if seed==0,p.sensor.noiseRms=0;p.sensor.strokeNoiseRms=0;p.contact.lateralNoiseRms_N=0;p.accel.noiseRms=0;end
end

function row=measure(q,r,index,c,elapsed,exception)
row=struct('caseIndex',index,'purpose',c.purpose,'scenario',c.scenario,'actuator',q.actuator.kind, ...
 'controller',q.screw.controlStrategy,'massS_kg',q.mechanics.mS,'massB_kg',q.mechanics.mB, ...
 'referenceForce_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi,'seed',q.sensor.seed, ...
 'forceNoiseRms_N',q.sensor.noiseRms,'accelNoiseRms_m_s2',q.accel.noiseRms,'step_s',q.sim.step, ...
 'sensorSampleTime_s',q.sensor.sampleTime,'forceFilterTime_s',q.sensor.filterTime, ...
 'forceDelay_s',q.sensor.delay,'positionNoiseRms_m',q.sensor.strokeNoiseRms, ...
 'positionDelay_s',q.sensor.strokeDelay,'accelerationFilterTime_s',q.accel.filterTime, ...
 'accelerationDelay_s',q.accel.delay,'accelerationRange_g',q.accel.range_g, ...
 'accelerationQuantum_m_s2',q.accel.quantum,'sandingDragRatio',q.contact.mu, ...
 'surfaceHeight_m',q.surface.height,'surfaceWidth_m',q.surface.width, ...
 'physicalScanSpeed_m_s',q.surface.scanSpeed, ...
 'commandedScanSpeed_m_s',q.contact.commandedScanSpeed_m_s, ...
 'lowerRigid',q.mechanics.lowerRigid,'upperRigid',q.mechanics.upperRigid,'cellStiffness_N_m',q.mechanics.k1, ...
 'cellDamping_Ns_m',q.mechanics.c1,'screwPitch_m',q.screw.lead,'rotorInertia_kg_m2',q.screw.rotorInertia, ...
 'effectiveLowerLinkStiffness_N_m',q.mechanics.k1,'effectiveLowerLinkDamping_Ns_m',q.mechanics.c1, ...
 'loadCellStiffness_N_m',NaN,'masslessSeriesComplianceApproximation',false,'springPreload_N',NaN, ...
 'reflectedRotorMass_kg',q.screw.rotorInertia/(q.screw.lead/(2*pi))^2, ...
 'currentForceKp',q.screw.currentForceKp,'currentForceKi',q.screw.currentForceKi,'currentForceKd',q.screw.currentForceKd, ...
 'coilForceKp_A_N',q.inertial.forceKp,'coilForceKi_A_N_s',q.inertial.forceKi, ...
 'lowerInverseEnabled',false,'lowerInverseCutoff_Hz',NaN,'identifiedInverseMovingMass_kg',NaN, ...
 'identifiedInverseForceConstant_N_A',NaN,'velocityForFriction','S-A velocity approximates actuator gap velocity', ...
 'speedForceKp',q.screw.forceKp,'speedForceKi',q.screw.forceKi,'speedForceKd',q.screw.forceKd, ...
 'velocityFeedforwardScale',q.screw.velocityFeedforwardScale, ...
 'notchEnabled',q.screw.currentNotch.enabled,'notchFrequency_Hz',q.screw.currentNotch.frequency_Hz, ...
 'feedbackFilterEnabled',q.inertial.feedbackFilter.enabled, ...
 'feedbackFilterCutoff_Hz',q.inertial.feedbackFilter.cutoff_Hz, ...
 'screwFeedbackFilterKind',q.screw.feedbackFilter.kind, ...
 'screwFeedbackNotchPoleDamping',q.screw.feedbackFilter.poleDamping, ...
 'activeFeedbackFilterKind',q.screw.feedbackFilter.kind,'activeFeedbackFilterCutoff_Hz',NaN, ...
 'accelerationPrediction_s',q.screw.accelerationPrediction_s, ...
 'accelerationDerivativeFilter_s',q.screw.accelerationDerivativeFilter_s, ...
 'accelerationFeedforwardFilter_s',q.screw.accelerationFeedforwardFilter_s, ...
 'executed',~isempty(r),'passed',false,'completed',false,'contactLoss',false,'strokeStop',false,'overCurrent',false, ...
 'peakNormalForceError_N',NaN,'rmsNormalForceError_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN, ...
 'peakVoltage_V',NaN,'peakMotorSpeed_rad_s',NaN,'currentSaturationFraction',NaN,'voltageSaturationFraction',NaN, ...
 'physicalCurrentAtLimitFraction',NaN,'currentCommandAtLimitFraction',NaN, ...
 'holdCurrent_A',NaN,'holdCopperLoss_W',NaN,'continuousStaticCatalogPassed',NaN, ...
 'sizingEfficiency',q.screw.sizingEfficiency,'holdCurrentEfficiencyBound_A',NaN, ...
 'continuousStaticEfficiencyBoundPassed',NaN,'continuousStaticTorqueBoundPassed',NaN, ...
 'normalBalanceResidual_N',NaN,'downstreamForceBalanceResidual_N',NaN, ...
 'actuatedForceBalanceResidual_N',NaN,'rotorForceBalanceResidual_N',NaN,'actualDuration_s',NaN, ...
 'minTravel_mm',NaN,'maxTravel_mm',NaN,'failureReason','','executionSeconds',elapsed,'executionError',exception,'nativeTraceFile','');
if isfield(q,'springs')
 row.springPreload_N=q.springs.commonPreload_N;
 if isfield(q.springs,'sensingCell')
  row.loadCellStiffness_N_m=q.springs.sensingCell.stiffness_N_m;
  row.masslessSeriesComplianceApproximation=q.springs.sensingCell.masslessSeriesApproximation;
 end
end
if isfield(q.inertial,'lowerLoadInverse')
 c=q.inertial.lowerLoadInverse;
 if strcmp(q.actuator.kind,'leadscrew'),c=q.d.screwLowerLoadInverse;end
 row.lowerInverseEnabled=c.enabled;row.lowerInverseCutoff_Hz=c.cutoff_Hz;
 row.identifiedInverseMovingMass_kg=c.identifiedMassB_kg;
 row.identifiedInverseForceConstant_N_A=c.identifiedForceConstant_N_A;
end
if strcmp(q.actuator.kind,'voice_coil')
 row.controller=['coil_' q.coil.strategy];row.activeFeedbackFilterKind='bypass';
 if q.inertial.feedbackFilter.enabled,row.activeFeedbackFilterKind='lowpass';end
 if isfield(q.inertial,'accelerationPrediction_s')
  row.accelerationPrediction_s=q.inertial.accelerationPrediction_s;
  row.accelerationDerivativeFilter_s=q.inertial.accelerationDerivativeFilter_s;
  if isfield(q.inertial,'accelerationFeedforwardFilter_s')
   row.accelerationFeedforwardFilter_s=q.inertial.accelerationFeedforwardFilter_s;
  end
 end
elseif strcmp(q.screw.feedbackFilter.kind,'common')
 row.activeFeedbackFilterKind='bypass';
 if q.inertial.feedbackFilter.enabled,row.activeFeedbackFilterKind='lowpass';end
end
if strcmp(row.activeFeedbackFilterKind,'lowpass')
 row.activeFeedbackFilterCutoff_Hz=q.inertial.feedbackFilter.cutoff_Hz;
 if strcmp(q.actuator.kind,'leadscrew')&&strcmp(q.screw.feedbackFilter.kind,'lowpass')
  row.activeFeedbackFilterCutoff_Hz=q.screw.feedbackFilter.cutoff_Hz;
 end
end
if isempty(r),row.failureReason='preparation or simulation error';return;end
q=r.parameters;t=r.time;d=r.data;n=r.normalContact.data;error=n(:,1)-q.force.reference;
row.peakNormalForceError_N=max(abs(error));row.rmsNormalForceError_N=sqrt(mean(error.^2));
row.completed=t(end)>=q.sim.stopTime-q.sim.step/2;row.contactLoss=any(d(:,18)<.5);row.strokeStop=any(d(:,21)>.5);
row.peakCurrent_A=max(abs(d(:,14)));row.rmsCurrent_A=sqrt(mean(d(:,14).^2));row.peakVoltage_V=max(abs(d(:,15)));
row.peakMotorSpeed_rad_s=max(abs(d(:,17)));row.currentSaturationFraction=mean(d(:,22)>.5);row.voltageSaturationFraction=mean(d(:,23)>.5);
if strcmp(q.actuator.kind,'voice_coil'),limit=q.coil.peakCurrent;hold=q.d.coilHoldCurrent;resistance=q.coil.resistance;
else,limit=q.screw.peakCurrent;hold=q.d.screwHoldCurrent;resistance=q.screw.resistance;end
row.overCurrent=row.peakCurrent_A>limit*(1+1e-4);row.holdCurrent_A=hold;row.holdCopperLoss_W=hold^2*resistance;
row.physicalCurrentAtLimitFraction=mean(abs(d(:,14))>=limit*(1-1e-6));
if strcmp(q.actuator.kind,'voice_coil') || strcmp(q.screw.controlStrategy,'current')
 row.currentCommandAtLimitFraction=mean(abs(d(:,33))>=limit*(1-1e-6));
end
if strcmp(q.actuator.kind,'leadscrew')
 row.continuousStaticCatalogPassed=abs(q.d.screwHoldCurrent)<=q.screw.continuousCurrent_A;
 row.holdCurrentEfficiencyBound_A=abs(q.d.screwHoldCurrent)/q.screw.sizingEfficiency;
 row.continuousStaticEfficiencyBoundPassed=row.holdCurrentEfficiencyBound_A<=q.screw.continuousCurrent_A;
 row.continuousStaticTorqueBoundPassed=abs(q.d.holdForce*q.d.ell)/q.screw.sizingEfficiency<=q.screw.continuousTorque_Nm;
else,row.sizingEfficiency=NaN;end
row.minTravel_mm=min(d(:,2))*1000;row.maxTravel_mm=max(d(:,2))*1000;
slope=d(:,29)/q.surface.scanSpeed;
row.normalBalanceResidual_N=max(abs(d(:,1).*sqrt(1+slope.^2)./(1-q.contact.mu*slope)-n(:,1)));
contact=d(:,18)>.5;
row.downstreamForceBalanceResidual_N=max(abs(d(contact,1)-d(contact,24)-q.mechanics.mS*(q.d.g+d(contact,30))));
row.actuatedForceBalanceResidual_N=max(abs(d(contact,1)-d(contact,13)-q.mechanics.mB*(q.d.g+d(contact,12))-q.mechanics.mS*(q.d.g+d(contact,30))));
assert(max([row.normalBalanceResidual_N,row.downstreamForceBalanceResidual_N,row.actuatedForceBalanceResidual_N])<1e-8, ...
 'Native force reconstruction/free-body balance failed.');
if strcmp(q.actuator.kind,'leadscrew')
 omega=d(:,17);friction=q.screw.coulombTorque*tanh(omega/q.screw.frictionSpeed)+q.screw.viscousTorque*omega;
 torque=q.screw.torqueConstant*d(:,14)-friction;
 row.rotorForceBalanceResidual_N=max(abs(torque/q.d.ell-d(:,13)-q.screw.rotorInertia/q.d.ell^2*(d(:,11)-d(:,12))));
 assert(row.rotorForceBalanceResidual_N<1e-7,'Native rotor/transmitted-force balance failed: residual %.12g N.',row.rotorForceBalanceResidual_N);
end
row.actualDuration_s=t(end);row.passed=row.completed && row.peakNormalForceError_N<=q.force.tolerance && ...
 ~row.contactLoss && ~row.strokeStop && ~row.overCurrent;
reasons={};if ~row.completed,reasons{end+1}='incomplete';end
if row.contactLoss,reasons{end+1}='contact loss';end;if row.strokeStop,reasons{end+1}='travel stop';end
if row.overCurrent,reasons{end+1}='overcurrent';end;if row.peakNormalForceError_N>q.force.tolerance,reasons{end+1}='normal force error';end
if isempty(reasons),row.failureReason='pass';else,row.failureReason=strjoin(reasons,', ');end
end

function strategy=choose_strategy(kd)
if kd>0,strategy='pid';else,strategy='pi';end
end
function from(sys,name,tag,pos)
block('simulink/Signal Routing/From',sys,name,pos,'GotoTag',tag);
end
function block(library,sys,name,pos,varargin)
add_block(library,[sys '/' name],'Position',pos,'FontSize','11',varargin{:});
end
function link(sys,source,dest)
add_line(sys,source,dest,'autorouting','on');
end
function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end
function s=defaults(s,d)
names=fieldnames(d);for j=1:numel(names),if ~isfield(s,names{j}),s.(names{j})=d.(names{j});end,end
end
function write_text(path,value)
fid=fopen(path,'w');assert(fid>0);fprintf(fid,'%s\n',value);fclose(fid);
end
function write_json(path,value)
write_text([path '.pending'],jsonencode(value,'PrettyPrint',true));movefile([path '.pending'],path,'f');
end
function digest=sha256(path)
quote=char(39);escaped=strrep(path,quote,[quote char(34) quote char(34) quote]);
[status,out]=system(['sha256sum -- ' quote escaped quote]);assert(status==0);digest=out(1:64);
end
function finish(mdl,storage,record,threads)
try,set_param(mdl,'FastRestart','off');close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
try,maxNumCompThreads(threads);catch,end
end
