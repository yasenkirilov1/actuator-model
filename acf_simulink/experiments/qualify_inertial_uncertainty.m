function report=qualify_inertial_uncertainty(sourceFolder,outputFolder,phase,candidateFile,simulationMode,priorJointFile,storageMode)
%QUALIFY_INERTIAL_UNCERTAINTY Native IMU/force calibration and timing study.
% Private instrument gains express residual calibration after gravity tare.
% Physical force, mass, robot motion and the frozen canonical model are never
% replaced by a script plant or supplied as controller truth.
% Optional seventh argument 'tmpfs' changes only private SDI storage, after
% checking the retained full-native disk/tmpfs proof for this source/candidate.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
project=fileparts(fileparts(mfilename('fullpath')));
if nargin<2,outputFolder=fullfile(project,'validation');end
if nargin<3,phase='sensitivity';end
if nargin<4,candidateFile=fullfile(sourceFolder,'normal_contact_trial_candidate.mat');end
if nargin<5,simulationMode='normal';end
if nargin<6,priorJointFile='';end
if nargin<7,storageMode='disk';end
simulationMode=validatestring(simulationMode,{'normal','accelerator'});
storageMode=validatestring(storageMode,{'disk','tmpfs'});
assert(isfolder(sourceFolder) && isfile(candidateFile),'A frozen inertial model and selected candidate are required.');
stem=['inertial_uncertainty_' phase '_20260930'];
resultFile=fullfile(outputFolder,[stem '_results.json']);
assert(~isfile(resultFile),'Preserve prior evidence: choose a fresh output folder.');
maxNumCompThreads(2);
workerFolder=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','helpers','cache','code','sdi'}
 folder=fullfile(workerFolder,name{1});if ~isfolder(folder),mkdir(folder);end
end
if ~isfolder(outputFolder),mkdir(outputFolder);end
traceFolder=fullfile(outputFolder,[stem '_native_traces']);mkdir(traceFolder);
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
sourceHash=sha256(sourceModel);candidateHash=sha256(candidateFile);
modelFile=fullfile(workerFolder,'acf_force_control_normal_trial.slx');copyfile(sourceModel,modelFile);
% Immutable freezes may be read-only. Change only the new private copy so
% adding native residual instruments and recording its mode can be saved.
assert(fileattrib(modelFile,'+w','u'),'Cannot make the private model writable.');
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile(candidateFile,fullfile(workerFolder,'candidate.mat'));
assert(strcmp(sha256(modelFile),sourceHash) && ...
 strcmp(sha256(fullfile(workerFolder,'candidate.mat')),candidateHash), ...
 'Frozen input changed while the private snapshot was copied.');
recipeFile=fullfile(workerFolder,'qualify_inertial_uncertainty.m');
copyfile([mfilename('fullpath') '.m'],recipeFile);
addpath(fullfile(workerFolder,'helpers'),'-begin');clear acf_prepare acf_run acf_metrics;
files=dir(fullfile(workerFolder,'helpers','*.m'));helperSources=struct([]);
for j=1:numel(files)
 one=struct('file',files(j).name,'sha256',sha256(fullfile(files(j).folder,files(j).name)));
 assert(strcmp(one.sha256,sha256(fullfile(sourceFolder,'helpers',files(j).name))), ...
  'Frozen helper changed while the private snapshot was copied.');
 if isempty(helperSources),helperSources=one;else,helperSources(end+1)=one;end %#ok<AGROW>
end
saved=load(fullfile(workerFolder,'candidate.mat'));
if isfield(saved,'p'),p=saved.p;else,p=saved.pSelected;end
assert(isfield(p,'accel') && p.accel.enabled && isfield(p,'inertial'),'The candidate lacks inertial feedback.');
assert(strcmp(p.actuator.kind,'voice_coil') && p.mechanics.mB==.6);
assert(strcmp(p.sensor.positionTarget,'tool_relative_A') && p.robotPrediction.enabled);
p.sim.stopTime=4;p.sim.evaluateFrom=0;p.sim.step=min(50e-6,p.sim.step);
p.robot.forceGate.filterTime=.1;p.robot.forceGate.dwellTime=.1;
if ~isfield(p.inertial,'accelerationPrediction_s'),p.inertial.accelerationPrediction_s=0;end
if ~isfield(p.inertial,'accelerationDerivativeFilter_s'),p.inertial.accelerationDerivativeFilter_s=.0003;end
if ~isfield(p.inertial,'accelerationFeedforwardFilter_s'),p.inertial.accelerationFeedforwardFilter_s=0;end
p.uncertainty=struct('forceScaleZ',0,'forceScaleX',0,'positionScale',0, ...
 'forceFrameAngle_rad',0,'accelFrameAngle_rad',0,'forceCrossZFromX',0,'forceCrossXFromZ',0, ...
 'accelBiasX_m_s2',0,'massB_model_kg',.6,'forceConstant_model_N_A',p.coil.forceConstant, ...
 'recenterGain',min(p.robot.recenterGain,p.robot.forceGate.maxDelayGainProduct/ ...
 (p.robotPrediction.delay+p.robotPrediction.responseTime+p.robot.forceGate.dwellTime+p.robot.forceGate.filterTime)));
p=acf_prepare(p);
sdiFolder=fullfile(workerFolder,'sdi');storageProof=struct([]);
if strcmp(storageMode,'tmpfs')
 proofFile=fullfile(project,'validation','normal_mass_controller_tmpfs_sdi_pilot_20260930_v8_results.json');
 assert(isfile(proofFile),'A retained storage-only native proof is required.');
 proof=jsondecode(fileread(proofFile));
 assert(strcmp(proof.status,'complete') && proof.allFourFullNativeHistoriesBitwiseIdentical && ...
  proof.sourceModelUnchanged && proof.privateModelUnchanged && ...
  strcmp(proof.modelSha256,sourceHash) && strcmp(proof.candidateSha256,candidateHash), ...
  'Storage proof does not bind the requested native source/candidate.');
 for helperIndex=1:numel(helperSources)
  match=find(strcmp({proof.helperSources.file},helperSources(helperIndex).file));
  assert(numel(match)==1 && strcmp(proof.helperSources(match).sha256,helperSources(helperIndex).sha256), ...
   'Storage proof helper hashes do not match the private frozen snapshot.');
 end
 assert(isfolder('/dev/shm'),'The requested tmpfs filesystem is unavailable.');
 [~,privateName]=fileparts(workerFolder);
 sdiFolder=fullfile('/dev/shm',['inertial-uncertainty-' privateName]);
 assert(~isfolder(sdiFolder),'Preserve any existing temporary SDI data.');
 mkdir(sdiFolder);
 storageProof=struct('file',proofFile,'sha256',sha256(proofFile), ...
  'nativeHistoriesCompared',proof.executedNativeComparisons,'allBitwiseIdentical',true, ...
  'helperHashesVerified',true, ...
  'scope','Storage-only execution change; native model, parameters, solver and metrics are unchanged.');
end
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(sdiFolder);Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,sdiFolder, ...
 strcmp(storageMode,'tmpfs'),fullfile(outputFolder,[stem '_storage_cleanup.json'])));
assert(~bdIsLoaded(mdl));load_system(modelFile);
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
instrumented=~strcmp(phase,'proof_reference');
if instrumented,add_residual_instruments(mdl);end
set_param(mdl,'SimulationMode',simulationMode);save_system(mdl,modelFile);
privateHash=sha256(modelFile);
controller=coil_controller_path(mdl);
gainContract=struct('movingMassFF',get_param(find_unique(controller,'Measured acceleration current feedforward'),'Gain'), ...
 'holdingCurrent',get_param(find_unique(controller,'Flat holding current'),'Value'), ...
 'sharedIdentifiedMassZ',get_param(find_unique(mdl,'Downstream Z inertia'),'Gain'), ...
 'sharedIdentifiedMassX',get_param(find_unique(mdl,'Downstream X inertia'),'Gain'));
if isfield(p.inertial,'lowerLoadInverse')
 gainContract.inverseDownstreamMass=get_param(find_unique(controller,'Known downstream S inertia'),'Gain');
 gainContract.inverseNumerator=get_param(find_unique(controller,'Proper identified lower-link inverse'),'Numerator');
 gainContract.inverseDenominator=get_param(find_unique(controller,'Proper identified lower-link inverse'),'Denominator');
 assert(strcmp(gainContract.inverseDownstreamMass,'p.sensor.distalMass_model_kg'), ...
  'The lower inverse uses actual instead of identified S mass.');
end
if isfield(p,'acquisition') && p.acquisition.enabled
 gainContract.actualForceAA=get_param([mdl '/Force measurement/Engineered common AA and EMA'],'Numerator');
 gainContract.actualAccelerationAA=get_param([mdl '/S accelerometer and ADC/Z engineered common AA and EMA'],'Numerator');
 gainContract.identifiedRobotFrameAA=get_param([mdl '/Contact force measurement/Head motion estimator/Robot velocity common AA and EMA'],'Numerator');
 gainContract.actualRobotTransportClock=get_param(find_unique(mdl,'Robot command samples'),'SampleTime');
end
fromBlocks=find_system(controller,'MatchFilter',@Simulink.match.allVariants,'BlockType','From');
if getSimulinkBlockHandle([mdl '/Contact force measurement'])~=-1
 fromBlocks=[fromBlocks;find_system([mdl '/Contact force measurement'], ...
  'MatchFilter',@Simulink.match.allVariants,'BlockType','From')];
end
tags=cellfun(@(block)get_param(block,'GotoTag'),fromBlocks,'UniformOutput',false);
assert(~any(ismember(tags,{'plant_log','surface_motion','normal_contact_N', ...
 'lateral_contact_N','actual_contact_slope','physical_S_acceleration_z','physical_S_acceleration_x'})), ...
 'Physical contact/geometry/acceleration truth enters the controller.');
configs=make_cases(p,phase,priorJointFile);rows=struct([]);plotTraces=struct([]);worst=[];worstError=-inf;
diskBackedWorst=ismember(phase,{'budget_repeated','calibrated_static_long'});
if ~isempty(priorJointFile)
 prior=load(priorJointFile,'report');
 assert(strcmp(prior.report.sourceModelSha256,sourceHash) && ...
  strcmp(prior.report.candidateSha256,candidateHash), ...
  'Long-run cases must reuse the same frozen source and selected candidate as their joint cohort.');
end
started=tic;lastCompileKey=[];
report=struct('date','2026-09-30','status','running','phase',phase, ...
 'recipeVersion','native_inertial_uncertainty_v8','plannedCases',numel(configs), ...
 'sourceModel',sourceModel,'sourceModelSha256',sourceHash,'privateModel',modelFile, ...
 'privateModelSha256',privateHash,'candidateFile',candidateFile,'candidateSha256',candidateHash, ...
 'helperSources',helperSources,'experimentSnapshot',recipeFile,'experimentSha256',sha256(recipeFile), ...
 'priorJointFile',priorJointFile, ...
 'matlabVersion',version,'simulationMode',simulationMode,'residualInstrumentationEnabled',instrumented, ...
 'storageMode',storageMode,'ownedSDIStorage',sdiFolder,'storageOnlyProof',storageProof, ...
 'diskBackedWorstHistory',diskBackedWorst, ...
 'memoryAtStart',process_memory(), ...
 'baseParameters',p,'controllerFromTags',{tags},'rows',rows, ...
 'nativeGainContract',gainContract, ...
 'scope','Actual normal pressing force +/-0.5 N over every native sample including startup, sharp 1 mm x10 mm bump at100 mm/s. Finite calibration/timing test grid.', ...
 'sensing','B-S multiaxis force plus S accelerometer plus S-A position; known static tare and issued robot commands only. Finite noise/filter/delay/ADC model, not perfect accelerometry.', ...
 'actualVsIdentified','Physical S stays unchanged for identified mass tests; identified S is used in inertia reconstruction, acceleration FF and holding-current FF. Physical current starts at its actual equilibrium. Robot predictor and recenter gain remain fixed at the baseline identified robot settings when actual delay/response change.', ...
 'residualCalibration','Force scale acts on calibrated, tared absolute cell signals before sampling/filtering, with consistent initial states; bias is drift after tare. Position scale acts around its calibrated initial zero. Acceleration gravity tare is recalibrated before each fixed-orientation run, retaining dynamic scale error and residual drift.', ...
 'gateMetric','Ready fraction and closures use the previous-held mean/rate gate on the native grid, combined with the fast variation gate when enabled. Previously queued robot motion can persist after either gate closes.', ...
 'matchedAcquisitionTarget','Timing phases are identified native response/phase acceptance targets, not exact manufacturer part transfers. Transport0 is zero additional whole-sample transport at a common acquisition clock, with analog/group delay retained in the stated filter model. Position transport matches force and acceleration. ATI analog response and mounted ADXL response require measurement; an AD7606-style synchronous analog acquisition can remove avoidable software buffering but does not remove analog sensor phase.', ...
 'alignmentConvention','Z is away from the workpiece, X is scanning direction. Positive residual sensor rotation maps Z to cos(angle)*Z+sin(angle)*X and X to cos(angle)*X-sin(angle)*Z. Force cross-talk adds independent X-to-Z/Z-to-X coefficients before diagonal scale. Force and accelerometer residual angles are relative to the flange frame used for the causal normal estimate. Gravity tare is recorded in each sensor frame, so fixed gravity is removed while dynamic frame error remains.', ...
 'qualificationLimit','One-factor sensitivities are not combined tolerances. Joint corner cases and repeated runs have their own explicit scope. No changing orientation, tool rotation, unknown surface preview, sanding vibration, pad pressure distribution or actual robot impedance is qualified.');
if strcmp(phase,'calibrated_static_long')
 refinementGuards=[configs.staticCalibrationGuard];
 sourceBinding=struct('sourceModelSha256',sourceHash,'privateModelSha256',privateHash, ...
  'candidateSha256',candidateHash,'helperSources',helperSources,'experimentSha256',report.experimentSha256);
 preparedFile=fullfile(outputFolder,[stem '_prepared_configs.mat']);
 save(preparedFile,'configs','refinementGuards','sourceBinding','-v7');
 report.preparedConfigurationsFile=preparedFile;report.preparedConfigurationsSha256=sha256(preparedFile);
 report.staticCalibrationGuard=refinementGuards;
 report.calibrationRefinementScope='Sixteen static residuals are halved from the exact four executed original30s cases. Noise, actual/identified timing and AA, robot, mechanics and100us current response are unchanged. Assembled calibration targets, not manufacturer noise improvements or full-grid qualification.';
end
for j=1:numel(configs)
 config=configs(j);q=config.p;
 compileKey=[q.sim.step,q.coil.currentResponse,q.sensor.filterTime,q.sensor.delay, ...
  q.sensor.strokeDelay,q.accel.filterTime,q.accel.delay,q.robot.delay,q.robotPrediction.delay,q.sim.stopTime];
 if isfield(q,'acquisition') && q.acquisition.enabled
  c=q.acquisition;
  compileKey=[compileKey,c.rawSampleTime_s,c.pipelineDelay_s,c.forceDelayOffset_s, ...
   c.lateralDelayOffset_s,c.accelerationDelayOffset_s,c.positionDelayOffset_s]; %#ok<AGROW>
 end
 if ~isequal(lastCompileKey,compileKey)
  set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');lastCompileKey=compileKey;
 end
 timer=tic;r=[];rejectedNative=[];nativeReturned=false;
 try
  quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  nativeReturned=~isempty(r);
  assert(isfield(r,'normalContact') && size(r.normalContact.data,2)==8,'Normal native log is missing.');
  assert(isequal(r.time,r.normalContact.time,r.accelerometer.time,r.motionObserver.time), ...
   'Native sensor/contact logging grids differ.');
  row=measure_run(q,r,j,config,toc(timer),'');
 catch failure
  exception=getReport(failure,'extended','hyperlinks','off');rejectedNative=r;r=[];
  row=measure_run(q,r,j,config,toc(timer),exception);
  fprintf('INERTIAL_UNCERTAINTY_REJECTION %d %s\n',j,failure.message);
 end
 row.nativeReturned=nativeReturned;
 if ~isempty(rejectedNative)
  % A returned history that fails a contract/balance check is retained apart
  % from validated native histories and cannot become a passing case.
  row.nativeTrace=fullfile(traceFolder,sprintf('rejected_native_%04d.mat',j));
  save(row.nativeTrace,'rejectedNative','row','-v7');clear rejectedNative;
 end
 if ~isempty(r)
  % Every failed native history is retained; finite plot excerpts are never scored.
  if ~row.passed || strcmp(config.scenario,'baseline') || startsWith(phase,'repeated') || ...
    diskBackedWorst || ...
    strcmp(phase,'smoke') || startsWith(phase,'matched_acquisition') || startsWith(phase,'proof_')
   row.nativeTrace=fullfile(traceFolder,sprintf('case_%04d.mat',j));
   save(row.nativeTrace,'r','row','-v7');
  end
  if row.peakNormalForceError_N>worstError
   worstError=row.peakNormalForceError_N;worst=r;worstRow=row;
   if diskBackedWorst
    % Save the full selected history now, then release it before the next
    % 30-second simulation rather than keeping a second large native record.
    save_worst();worst=[];
   end
  end
  trace=plot_excerpt(r,row);
  if isempty(plotTraces),plotTraces=trace;else,plotTraces(end+1)=trace;end %#ok<AGROW>
 end
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 clear r;checkpoint('running');
 if j==1 || mod(j,4)==0
  fprintf('INERTIAL_UNCERTAINTY_PROGRESS %s %d/%d,%d pass,%d rejected; %s %.9g N,elapsed%.1fs\n', ...
   phase,j,numel(configs),sum([rows.passed]),sum(~[rows.executed]),config.scenario,row.peakNormalForceError_N,toc(started));
 end
 if mod(j,8)==0 || startsWith(phase,'budget_') || diskBackedWorst,Simulink.sdi.clear;end
 if isfile(fullfile(workerFolder,'stop_after_case'))
  checkpoint('stopped_by_coordinator');save_worst();return;
 end
end
report.sourceStillMatches=strcmp(sha256(sourceModel),sourceHash);
report.privateSourceUnmodified=strcmp(sha256(modelFile),privateHash);
report.sourceHelpersStillMatch=true;
for j=1:numel(helperSources)
 report.sourceHelpersStillMatch=report.sourceHelpersStillMatch && strcmp(helperSources(j).sha256, ...
  sha256(fullfile(sourceFolder,'helpers',helperSources(j).file)));
end
assert(report.sourceStillMatches && report.privateSourceUnmodified && report.sourceHelpersStillMatch, ...
 'Frozen input changed during qualification.');
report.scenarioRanking=rank_scenarios(rows);save_worst();checkpoint('complete');
if diskBackedWorst,save_worst();end
fprintf('INERTIAL_UNCERTAINTY_COMPLETE %s %d/%d executed,%d passed,%d rejected,worst%.9g N.\n', ...
 phase,report.executed,report.plannedCases,report.passed,report.rejectedBeforeMetrics,worstError);

 function checkpoint(status)
  report.status=status;report.rows=rows;report.attempts=numel(rows);
  report.executed=sum([rows.executed]);report.validatedNativeHistories=report.executed;
  report.nativeReturned=sum([rows.nativeReturned]);report.failedBeforeNativeReturn=sum(~[rows.nativeReturned]);
  report.passed=sum([rows.passed]);report.failedNative=report.executed-report.passed;
  report.rejectedBeforeMetrics=sum(~[rows.executed]);report.elapsedSeconds=toc(started);
  report.worstNormalForceError_N=worstError;
  report.memoryAtCheckpoint=process_memory();
  report.peakResidentMemory_kB=report.memoryAtCheckpoint.peakResident_kB;
  fid=fopen(resultFile,'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  writetable(struct2table(rows,'AsArray',true),fullfile(outputFolder,[stem '_rows.csv']));
  save(fullfile(outputFolder,[stem '_results.mat']),'report','configs','-v7');
  if isscalar(rows) || mod(numel(rows),8)==0 || ~strcmp(status,'running')
   save(fullfile(outputFolder,[stem '_plot_traces.mat']),'plotTraces','-v7');
  end
 end
 function save_worst()
  worstFile=fullfile(outputFolder,[stem '_worst_native_trace.mat']);
  if ~isempty(worst)
   report.worstCaseRow=worstRow;report.worstCaseParameters=worst.parameters;
   save(worstFile,'worst','worstRow','report','-v7');
  elseif diskBackedWorst && isfile(worstFile)
   % Append only the small finalized metadata; the saved native history stays
   % on disk and is never loaded again merely to update cohort status.
   save(worstFile,'report','-append');
  end
 end
end

function configs=make_cases(p,phase,priorJointFile)
configs=struct([]);keys={};
switch phase
 case 'smoke'
  add(physical_case(p,2,50,90,.3,1729),'baseline','none',0);
 case {'proof_reference','proof_normal','proof_accelerator'}
  add(physical_case(p,2,50,90,.3,1729),'baseline','none',0);
  add(physical_case(p,.1,5,0,.3,29),'baseline','none',0);
 case {'sensitivity','core_sensitivity'}
  % Each main sensitivity uses the same two heavy/noisy endpoint records.
  for seed=[7 1729]
   q0=physical_case(p,2,50,90,.3,seed);add(q0,'baseline','none',0);
   for value=[-.05 -.02 .02 .05]
    q=q0;q.accel.bias=value;add(q,'accelerometer_bias','accel.bias',value);
   end
   for value=[-.01 -.005 .005 .01]
    q=q0;q.accel.scaleError=value;add(q,'accelerometer_scale','accel.scaleError',value);
   end
   for value=[-.1 -.05 -.02 .02 .05 .1]
    q=q0;q.sensor.bias=value;add(q,'axial_force_bias','sensor.bias',value);
   end
   for value=[-.002 -.001 .001 .002]
    q=q0;q.uncertainty.forceScaleZ=value;q.uncertainty.forceScaleX=value;
    add(q,'force_vector_scale','uncertainty.forceScaleZ/X',value);
   end
   if strcmp(phase,'core_sensitivity')
    for value=[.03 .05 .1]
     q=q0;q.sensor.noiseRms=value;q.contact.lateralNoiseRms_N=value;
     add(q,'force_vector_noise','forceOutputNoiseRms_N',value);
    end
    for value=[.02 .03 .05]
     q=q0;q.accel.noiseRms=value;add(q,'accelerometer_noise','accel.noiseRms',value);
    end
   end
   for value=[-.01 -.005 .005 .01]
    q=q0;q.sensor.distalMass_model_kg=q.mechanics.mS*(1+value);
    add(q,'identified_S_mass','sensor.distalMass_model_kg relative',value);
   end
   if strcmp(phase,'sensitivity')
    for value=[200e-6 300e-6 400e-6]
     q=q0;q.sensor.filterTime=value;q.accel.filterTime=value;
     add(q,'matched_filter_response','sensor/accel.filterTime',value);
    end
    for value=[0 100e-6 200e-6]
     q=q0;q.accel.delay=value;add(q,'acceleration_channel_delay','accel.delay',value);
    end
    for value=[-.5 -.25 .25 .5]
     q=q0;q.accel.filterTime=q0.accel.filterTime*(1+value);
     add(q,'acceleration_filter_mismatch','accel.filterTime relative',value);
    end
   end
   for value=[1e-6 3e-6]
    q=q0;q.sensor.strokeNoiseRms=value;add(q,'position_noise','sensor.strokeNoiseRms',value);
   end
   for value=[-.01 -.001 .001 .01]
    q=q0;q.uncertainty.positionScale=value;add(q,'position_scale','uncertainty.positionScale',value);
   end
   for value=[.06 .08 .10 .12]
    q=q0;q.robot.delay=value;add(q,'robot_actual_delay','robot.delay',value);
   end
   for value=[.04 .06 .10]
    q=q0;q.robot.responseTime=value;add(q,'robot_actual_response','robot.responseTime',value);
   end
  end
  % Light tool low-force cases bound startup and calibration behavior.
  for seed=[7 1729]
   q0=physical_case(p,.1,5,0,.3,seed);add(q0,'baseline','none',0);
   for sign=[-1 1]
    q=q0;q.accel.bias=sign*.05;add(q,'light_accelerometer_bias','accel.bias',q.accel.bias);
    q=q0;q.sensor.bias=sign*.1;add(q,'light_force_bias','sensor.bias',q.sensor.bias);
    q=q0;q.sensor.distalMass_model_kg=q.mechanics.mS*(1+sign*.01);
    add(q,'light_identified_S_mass','sensor.distalMass_model_kg relative',sign*.01);
    q=q0;q.uncertainty.positionScale=sign*.01;add(q,'light_position_scale','uncertainty.positionScale',sign*.01);
   end
  end
 case 'joint'
  % This is an explicit engineering tolerance proposal, not a claimed sensor
  % factory specification: biases .02 N/.02 m/s2, scale .1%/.5%, mass .5%,
  % position3 um/.1%, differential filter25%, correctly matched100 us delay,
  % frame alignment .1 deg and cross-talk .1%. This is a bounded corner grid.
  for mass=[.1 2]
   if mass==.1,reference=5;else,reference=50;end
   for angle=[0 90]
    for seed=[7 29 1729]
     q0=physical_case(p,mass,reference,angle,.3,seed);
     add(q0,'joint_reference','none',0);
     for sign=[-1 1]
      q=q0;q.sensor.bias=sign*.02;q.contact.lateralBias_N=sign*.02;
      q.accel.bias=sign*.02;q.uncertainty.accelBiasX_m_s2=sign*.02;
      q.accel.scaleError=sign*.005;
      q.uncertainty.forceScaleZ=sign*.001;q.uncertainty.forceScaleX=sign*.001;
      q.sensor.distalMass_model_kg=mass*(1+sign*.005);
      q.uncertainty.massB_model_kg=.6*(1+sign*.005);
      q.sensor.strokeNoiseRms=3e-6;q.uncertainty.positionScale=sign*.001;
      q.uncertainty.forceFrameAngle_rad=sign*.1*pi/180;
      q.uncertainty.accelFrameAngle_rad=-sign*.1*pi/180;
      q.uncertainty.forceCrossZFromX=sign*.001;
      q.uncertainty.forceCrossXFromZ=sign*.001;
      q.accel.filterTime=q0.accel.filterTime*(1+sign*.25);
      add(q,'joint_sensor_same_sign','joint tolerance sign',sign);
      q.accel.scaleError=-sign*.005;
      q.sensor.distalMass_model_kg=mass*(1-sign*.005);
      q.accel.filterTime=q0.accel.filterTime*(1-sign*.25);
      add(q,'joint_sensor_opposed_dynamic','joint tolerance sign',sign);
     end
    end
   end
  end
 case 'alignment'
  for seed=[29 1729]
   q0=physical_case(p,2,50,90,.3,seed);add(q0,'alignment_reference','none',0);
   for value=[-.25 -.1 .1 .25]
    q=q0;q.uncertainty.forceFrameAngle_rad=value*pi/180;
    add(q,'force_frame_alignment','forceFrameAngle_deg',value);
    q=q0;q.uncertainty.accelFrameAngle_rad=value*pi/180;
    add(q,'accelerometer_frame_alignment','accelFrameAngle_deg',value);
   end
   for value=[-.002 -.001 .001 .002]
    q=q0;q.uncertainty.forceCrossZFromX=value;
    add(q,'force_cross_X_to_Z','forceCrossZFromX',value);
    q=q0;q.uncertainty.forceCrossXFromZ=value;
    add(q,'force_cross_Z_to_X','forceCrossXFromZ',value);
   end
  end
 case 'engineered_sensing'
  assert(isfield(p,'acquisition') && p.acquisition.enabled, ...
   'This phase requires the frozen V8 actual anti-alias acquisition.');
  % Relative sensor-frame calibration and actual sensing-transfer mismatch.
  % Issued-motion filtering/pipeline stays at its independently identified
  % nominal response for all these tests; no physical signal corrects it.
  for seed=[29 1729]
   q0=physical_case(p,2,50,90,.3,seed);add(q0,'engineered_reference','none',0);
   for sign=[-1 1]
    q=q0;q.uncertainty.forceFrameAngle_rad=sign*.1*pi/180;
    add(q,'engineered_force_frame','forceFrameAngle_deg',sign*.1);
    q=q0;q.uncertainty.accelFrameAngle_rad=sign*.1*pi/180;
    add(q,'engineered_accel_frame','accelFrameAngle_deg',sign*.1);
    q=q0;q.uncertainty.forceCrossZFromX=sign*.001;
    add(q,'engineered_cross_X_to_Z','forceCrossZFromX',sign*.001);
    q=q0;q.uncertainty.forceCrossXFromZ=sign*.001;
    add(q,'engineered_cross_Z_to_X','forceCrossXFromZ',sign*.001);
    q=q0;q.acquisition.antiAliasCutoff_Hz=q0.acquisition.antiAliasCutoff_Hz*(1+sign*.05);
    add(q,'engineered_common_AA','actual antiAliasCutoff relative',sign*.05);
    q=q0;q.acquisition.accelerationAntiAliasScale=1+sign*.05;
    add(q,'engineered_accel_AA','actual accelerationAntiAliasScale relative',sign*.05);
    q=q0;q.acquisition.forceAntiAliasScale=1+sign*.05;
    add(q,'engineered_force_AA','actual forceAntiAliasScale relative',sign*.05);
    q=q0;q.acquisition.positionAntiAliasScale=1+sign*.05;
    add(q,'engineered_position_AA','actual positionAntiAliasScale relative',sign*.05);
    q=q0;q.acquisition.pipelineDelay_s=q0.acquisition.pipelineDelay_s+sign*10e-6;
    add(q,'engineered_common_transport','actual pipelineDelay residual_s',sign*10e-6);
    q=q0;q.acquisition.accelerationDelayOffset_s=sign*10e-6;
    add(q,'engineered_accel_transport','actual accelerationDelayOffset_s',sign*10e-6);
   end
  end
  assert(numel(configs)==42,'The bounded actual-sensing OAT cohort changed.');
 case 'robot_fine_delay'
  assert(isfield(p,'acquisition') && p.acquisition.enabled && ...
   abs(p.d.actualRobotTransportSampleTime_s-p.force.sampleTime)<1e-12, ...
   'Fine actual robot transport must be native; do not change the sender/predictor100Hz clock.');
  for seed=[29 1729]
   for value=[-.01 -.005 .005 .01]
    q=physical_case(p,2,50,90,.3,seed);q.robot.delay=p.robotPrediction.delay+value;
    add(q,'robot_actual_fine_delay','actual delay residual_s',value);
   end
   for value=[-.005 .005]
    q=physical_case(p,2,50,90,.3,seed);q.robot.responseTime=p.robotPrediction.responseTime+value;
    add(q,'robot_actual_fine_response','actual response residual_s',value);
   end
  end
  for value=[-.005 .005]
   q=physical_case(p,.1,5,0,.3,29);q.robot.delay=p.robotPrediction.delay+value;
   add(q,'light_robot_actual_fine_delay','actual delay residual_s',value);
  end
  assert(numel(configs)==14,'The bounded native robot-delay cohort changed.');
 case 'budget_joint'
  assert(isfield(p,'acquisition') && p.acquisition.enabled, ...
   'The final joint budget requires actual V8 anti-alias sensing and native fine robot transport.');
  % Ten deterministic corners in26 dimensions: five explicit sign patterns
  % and their antipodes. This is a finite tolerance sample, not an exhaustive
  % hypercube or a proof that every combination inside the budget passes.
  pattern=ones(5,26);
  for axis=1:26
   for code=1:4,pattern(code+1,axis)=1-2*mod(floor((axis-1)/2^(code-1)),2);end
  end
  endpoints=[2,50,90;.1,5,0];
  for endpoint=1:2
   one=endpoints(endpoint,:);
   for seed=[7 29 1729]
    for corner=1:10
     s=pattern(ceil(corner/2),:);if mod(corner,2)==0,s=-s;end
     q=physical_case(p,one(1),one(2),one(3),.3,seed);
     q.sensor.bias=s(1)*.02;q.contact.lateralBias_N=s(2)*.02;
     q.uncertainty.forceScaleZ=s(3)*.001;q.uncertainty.forceScaleX=s(4)*.001;
     q.uncertainty.forceFrameAngle_rad=s(5)*.1*pi/180;
     q.uncertainty.accelFrameAngle_rad=s(6)*.1*pi/180;
     q.uncertainty.forceCrossZFromX=s(7)*.001;q.uncertainty.forceCrossXFromZ=s(8)*.001;
     q.accel.bias=s(9)*.01;q.uncertainty.accelBiasX_m_s2=s(10)*.01;
     q.accel.scaleError=s(11)*.005;
     q.sensor.distalMass_model_kg=q.mechanics.mS*(1+s(12)*.005);
     q.uncertainty.massB_model_kg=q.mechanics.mB*(1+s(13)*.005);
     q.uncertainty.forceConstant_model_N_A=q.coil.forceConstant*(1+s(14)*.005);
     q.sensor.strokeNoiseRms=1e-6;q.uncertainty.positionScale=s(15)*.001;
     q.acquisition.pipelineDelay_s=p.acquisition.pipelineDelay_s+s(16)*10e-6;
     q.acquisition.accelerationDelayOffset_s=s(17)*10e-6;
     q.acquisition.lateralDelayOffset_s=s(18)*10e-6;
     q.acquisition.positionDelayOffset_s=s(19)*10e-6;
     q.robot.delay=p.robotPrediction.delay+s(20)*.005;
     q.robot.responseTime=p.robotPrediction.responseTime+s(21)*.005;
     q.acquisition.antiAliasCutoff_Hz=p.acquisition.antiAliasCutoff_Hz*(1+s(22)*.05);
     % Force is the reference channel for the common actual AA drift.
     % Other channels may each differ +/-5% from force; their pairwise
     % difference can therefore reach10% of the common cutoff.
     q.acquisition.forceAntiAliasScale=1;
     q.acquisition.lateralAntiAliasScale=1+s(23)*.05;
     q.acquisition.accelerationAntiAliasScale=1+s(24)*.05;
     q.acquisition.positionAntiAliasScale=1+s(25)*.05;
     % Last sign independently tests residual issued scan-speed scale0.1%.
     q.contact.commandedScanSpeed_m_s=q.surface.scanSpeed*(1+s(26)*.001);
     q.uncertainty.jointCorner=corner;q.uncertainty.jointSigns=s;
     add(q,'budget_joint_corner','deterministic corner index',corner);
    end
   end
  end
  assert(numel(configs)==60,'The bounded joint tolerance cohort changed.');
 case 'budget_repeated'
  assert(isfile(priorJointFile),'Retained final joint results are required.');
  previous=load(priorJointFile,'configs','report');
  assert(strcmp(previous.report.phase,'budget_joint') && strcmp(previous.report.status,'complete'), ...
   'Choose worst executed corners from a completed joint cohort.');
  rows=previous.report.rows;
  endpointSeeds=[2 7;2 29;2 1729;.1 29];
  for selection=1:size(endpointSeeds,1)
    mass=endpointSeeds(selection,1);seed=endpointSeeds(selection,2);
    ids=find([rows.massS_actual_kg]==mass & [rows.seed]==seed & [rows.executed]);
    assert(numel(ids)==10,'Expected all ten deterministic joint corners for each endpoint/seed.');
    [~,worst]=max([rows(ids).peakNormalForceError_N]);id=ids(worst);
    q=previous.configs(id).p;q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=30;
    q.uncertainty.priorJointCaseId=id;
    add(q,'budget_worst_corner_repeated_30s','prior joint case id',id);
  end
 assert(numel(configs)==4,'The bounded joint-corner long-run cohort changed.');
 case 'calibrated_static_long'
  previous=load(priorJointFile,'configs','report');
  assert(strcmp(previous.report.phase,'budget_repeated') && strcmp(previous.report.status,'complete') && ...
   numel(previous.configs)==4,'Use the four completed original30s cases unchanged.');
  for selection=1:numel(previous.configs)
   before=previous.configs(selection).p;q=before;
   q.sensor.bias=before.sensor.bias*.5;
   q.contact.lateralBias_N=before.contact.lateralBias_N*.5;
   fields={'forceScaleZ','forceScaleX','forceFrameAngle_rad','accelFrameAngle_rad', ...
    'forceCrossZFromX','forceCrossXFromZ','accelBiasX_m_s2','positionScale'};
   for field=fields,q.uncertainty.(field{1})=before.uncertainty.(field{1})*.5;end
   q.accel.bias=before.accel.bias*.5;q.accel.scaleError=before.accel.scaleError*.5;
   q.sensor.distalMassCalibrationScale=1+.5*(before.sensor.distalMassCalibrationScale-1);
   q.sensor.distalMass_model_kg=q.mechanics.mS*q.sensor.distalMassCalibrationScale;
   q.uncertainty.massB_model_kg=q.mechanics.mB+.5*(before.uncertainty.massB_model_kg-q.mechanics.mB);
   q.uncertainty.forceConstant_model_N_A=q.coil.forceConstant+ ...
    .5*(before.uncertainty.forceConstant_model_N_A-q.coil.forceConstant);
   q.contact.commandedScanSpeed_m_s=q.surface.scanSpeed+ ...
    .5*(before.contact.commandedScanSpeed_m_s-q.surface.scanSpeed);
   add(q,'calibrated_static_worst_corner_repeated_30s','prior joint case id',q.uncertainty.priorJointCaseId);
   configs(end).staticCalibrationGuard=static_calibration_guard(before,configs(end).p);
  end
  assert(numel(configs)==4,'The four paired calibration-refinement cases changed.');
 case 'scan_speed'
  % Physical scan stays exactly100 mm/s. Only the command/identified velocity
  % used for the causal surface-normal direction has residual scale error.
  for value=[-.05 -.03 -.01 .01 .03 .05]
   q=physical_case(p,2,50,90,.3,29);
   q.contact.commandedScanSpeed_m_s=q.surface.scanSpeed*(1+value);
   add(q,'identified_scan_speed','commandedScanSpeed relative',value);
  end
  for value=[-.05 .05]
   q=physical_case(p,.1,5,0,.3,29);
   q.contact.commandedScanSpeed_m_s=q.surface.scanSpeed*(1+value);
   add(q,'light_identified_scan_speed','commandedScanSpeed relative',value);
  end
 case 'holder_mass'
  for angle=[0 90]
   for value=[-.01 -.005 .005 .01]
    q=physical_case(p,2,50,angle,.3,29);
    q.uncertainty.massB_model_kg=q.mechanics.mB*(1+value);
    add(q,'identified_B_mass','massB_model_kg relative',value);
   end
  end
 case {'matched_acquisition','matched_acquisition_remainder'}
  assert(isfield(p.d,'accelerationFeedforwardFilter'),'V7 native FF command filter required.');
  assert(p.inertial.forceKp==.06 && p.inertial.forceKi==20 && ...
   p.inertial.feedbackFilter.enabled && p.inertial.feedbackFilter.cutoff_Hz==1000 && ...
   p.inertial.feedbackFilter.damping==1.5 && ~p.inertial.currentNotch.enabled && ...
   strcmp(p.contact.geometryMode,'measured_drag_ratio') && p.contact.dragRatioFilter_s==.005 && ...
   p.inertial.guideVelocityPrediction_s==400e-6 && p.coil.currentResponse==100e-6 && ...
   p.sensor.sampleTime==100e-6 && p.force.sampleTime==100e-6, ...
   'Use the explicit common V7 controller, not defaults from the frozen input candidate.');
  timings=[100e-6,100e-6;100e-6,0;150e-6,0;200e-6,0];
  endpoints=[2,50,90,0;2,50,90,7;2,50,90,29;2,50,90,1729;.1,5,90,7];
  dt=p.force.sampleTime;
  for timing=1:size(timings,1)
   tau=timings(timing,1);transport=timings(timing,2);
   measuredGroupDelay=dt/expm1(dt/tau);
   for shaped=[0 1]
    for endpoint=1:size(endpoints,1)
     one=endpoints(endpoint,:);q=physical_case(p,one(1),one(2),one(3),.3,one(4));
     q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=10;
     q.sensor.filterTime=tau;q.accel.filterTime=tau;
     q.sensor.delay=transport;q.accel.delay=transport;q.sensor.strokeDelay=transport;
     q.contact.lateralDelaySamples=round(transport/q.sensor.sampleTime);
     q.accel.range_g=50;q.accel.quantum=.00468;
     if one(4)~=0,q.accel.noiseRms=.0123*sqrt(100e-6/tau);end
     q.inertial.accelerationFeedforwardFilter_s=shaped*.001;
     q.inertial.accelerationPrediction_s=0;
     if shaped
      ffGroupDelay=dt/expm1(dt/.001);
      q.inertial.accelerationPrediction_s=ffGroupDelay+measuredGroupDelay+transport+ ...
       q.coil.currentResponse+50e-6;
     end
     q.uncertainty.desiredJerkPrediction_s=shaped*50e-6;
     if shaped,scenario='matched_acquisition_shaped';else,scenario='matched_acquisition_unshaped';end
     add(q,scenario,'timing target index',timing);
    end
   end
  end
  if strcmp(phase,'matched_acquisition_remainder')
   % Recovery was scheduled from the last printed32-case progress message.
   % The final preserved checkpoint had34 cases, so original33:34 are
   % explicitly retained duplicate benchmarks, not new test-grid coverage.
   % Execute original33:40 with identical noise/timing in a private process.
   assert(numel(configs)==40,'The retained primary cohort contract changed.');
   configs=configs(33:40);
  end
 case 'repeated'
  for mass=[.1 2]
   if mass==.1,reference=5;angle=0;else,reference=50;angle=90;end
   for seed=[29 1729]
    q=physical_case(p,mass,reference,angle,.3,seed);
    q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=30;
    add(q,'repeated_baseline_30s','stopTime',30);
   end
  end
 case 'repeated_screen'
  q=physical_case(p,2,50,90,.3,29);
  q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=10;
  add(q,'provisional_repeated_10s','stopTime',10);
 case 'repeated_screen_seeds'
  for seed=[7 29 1729]
   q=physical_case(p,2,50,90,.3,seed);
   q.surface.kind='repeated';q.surface.repeatPeriod=1.2;q.sim.stopTime=10;
   add(q,'provisional_repeated_10s','stopTime',10);
  end
 otherwise,error('Unknown phase: %s',phase);
end

 function add(q,scenario,parameter,value)
  % Delivery helpers express calibration using scales so changing the
  % physical tool mass remains a single-field operation. Keep each requested
  % identified mass consistent across that API and legacy frozen helpers.
  if isfield(q.sensor,'distalMassCalibrationScale')
   q.sensor.distalMassCalibrationScale=q.sensor.distalMass_model_kg/q.mechanics.mS;
  end
  if isfield(q.inertial,'holderMassCalibrationScale')
   q.inertial.holderMassCalibrationScale=q.uncertainty.massB_model_kg/q.mechanics.mB;
  end
  if isfield(q.inertial,'forceConstantCalibrationScale')
   q.inertial.forceConstantCalibrationScale=q.uncertainty.forceConstant_model_N_A/q.coil.forceConstant;
  end
  if isfield(q.inertial,'lowerLoadInverse')
   % The inverse is an identified controller model. Do not silently retune
   % its k/c from a physical plant change; B/Kf weighing errors apply to all
   % controller force terms consistently, including this proper inverse.
   q.inertial.lowerLoadInverse.identifiedMassB_kg=q.uncertainty.massB_model_kg;
   q.inertial.lowerLoadInverse.identifiedForceConstant_N_A=q.uncertainty.forceConstant_model_N_A;
  end
  % Store the actual prepared acquisition clocks/delays as well as the
  % requested calibration. The run helper prepares the same structure again.
  % Actual AA/transport changes must not alter the identified frame response.
  q=acf_prepare(q);
  values=[q.mechanics.mS,q.force.reference,q.mechanics.angle,q.contact.mu,q.sensor.seed, ...
   q.sensor.bias,q.contact.lateralBias_N,q.sensor.noiseRms,q.sensor.strokeNoiseRms, ...
   q.sensor.distalMass_model_kg,q.sensor.filterTime,q.sensor.delay,q.sensor.strokeDelay, ...
   q.accel.bias,q.uncertainty.accelBiasX_m_s2,q.accel.scaleError,q.accel.filterTime,q.accel.delay,q.accel.noiseRms, ...
   q.inertial.accelerationFeedforwardFilter_s,q.inertial.accelerationPrediction_s, ...
   q.uncertainty.forceScaleZ,q.uncertainty.forceScaleX,q.uncertainty.positionScale, ...
   q.uncertainty.forceFrameAngle_rad,q.uncertainty.accelFrameAngle_rad, ...
   q.uncertainty.forceCrossZFromX,q.uncertainty.forceCrossXFromZ, ...
   q.uncertainty.massB_model_kg,q.uncertainty.forceConstant_model_N_A,q.contact.commandedScanSpeed_m_s, ...
   q.robot.delay,q.robot.responseTime,q.sim.stopTime];
  if isfield(q,'acquisition') && q.acquisition.enabled
   c=q.acquisition;
   values=[values,c.antiAliasCutoff_Hz,c.antiAliasDamping,c.emaTime_s,c.pipelineDelay_s, ...
    c.forceDelayOffset_s,c.lateralDelayOffset_s,c.accelerationDelayOffset_s,c.positionDelayOffset_s, ...
    c.forceAntiAliasScale,c.lateralAntiAliasScale,c.accelerationAntiAliasScale,c.positionAntiAliasScale]; %#ok<AGROW>
  end
  key=[sprintf('%.17g,',values),q.surface.kind];
  if any(strcmp(keys,key)),return;end
  caseDefinition=struct('p',q,'scenario',scenario,'parameter',parameter,'value',value,'key',key, ...
   'staticCalibrationGuard',struct([]));
  keys{end+1}=key;
  if isempty(configs),configs=caseDefinition;else,configs(end+1)=caseDefinition;end
 end
end

function q=physical_case(p,mass,reference,angle,mu,seed)
q=p;q.mechanics.mS=mass;q.mechanics.mB=.6;
q.sensor.distalMass_kg=mass;q.sensor.distalMass_model_kg=mass;
q.force.reference=reference;q.mechanics.angle=angle*pi/180;q.contact.mu=mu;
q.sensor.seed=seed;if seed==0,q.sensor.seed=29;end
q.sensor.noiseRms=.02;q.sensor.strokeNoiseRms=1e-6;
q.sensor.bias=0;q.contact.lateralNoiseRms_N=.02;q.contact.lateralBias_N=0;
q.accel.bias=0;q.accel.scaleError=0;
q.uncertainty.forceScaleZ=0;q.uncertainty.forceScaleX=0;q.uncertainty.positionScale=0;
q.uncertainty.accelBiasX_m_s2=0;
q.uncertainty.forceFrameAngle_rad=0;q.uncertainty.accelFrameAngle_rad=0;
q.uncertainty.forceCrossZFromX=0;q.uncertainty.forceCrossXFromZ=0;
q.uncertainty.massB_model_kg=.6;
q.contact.commandedScanSpeed_m_s=q.surface.scanSpeed;
if seed==0
 q.sensor.noiseRms=0;q.sensor.strokeNoiseRms=0;
 q.contact.lateralNoiseRms_N=0;q.accel.noiseRms=0;
end
end

function guard=static_calibration_guard(before,after)
primitive={'sensor.bias','contact.lateralBias_N','uncertainty.forceScaleZ','uncertainty.forceScaleX', ...
 'uncertainty.forceFrameAngle_rad','uncertainty.accelFrameAngle_rad', ...
 'uncertainty.forceCrossZFromX','uncertainty.forceCrossXFromZ', ...
 'accel.bias','uncertainty.accelBiasX_m_s2','accel.scaleError', ...
 'sensor.distalMassCalibrationScale','uncertainty.massB_model_kg', ...
 'uncertainty.forceConstant_model_N_A','uncertainty.positionScale','contact.commandedScanSpeed_m_s'};
derived={'sensor.distalMass_model_kg','inertial.holderMassCalibrationScale', ...
 'inertial.forceConstantCalibrationScale','inertial.lowerLoadInverse.identifiedMassB_kg', ...
 'inertial.lowerLoadInverse.identifiedForceConstant_N_A','inertial.lowerLoadInverse.numerator', ...
 'inertial.lowerLoadInverse.denominator'};
changed=parameter_changes(before,after,'');
coefficientChanges=startsWith(changed,'d.coilControl.');
assert(all(ismember(changed,[primitive,derived]) | coefficientChanges), ...
 'Calibration refinement changed an unapproved primitive or derived parameter.');
assert(numel(primitive)==16 && all(ismember(primitive,changed)), ...
 'Exactly sixteen declared static calibration residuals must be halved.');
beforeDerived=before.d;afterDerived=after.d;
beforeDerived=rmfield(beforeDerived,'coilControl');afterDerived=rmfield(afterDerived,'coilControl');
assert(isequaln(beforeDerived,afterDerived), ...
 'Physical startup, plant/current, sensor response, timing and observer coefficients changed.');
guard=struct('passed',true,'staticResidualCount',16,'primitiveAllowedPaths',{primitive}, ...
 'derivedAllowedPaths',{[derived,{'d.coilControl.* (identified-mass coefficients for inactive alternative)'}]}, ...
 'actualChangedPaths',{changed}, ...
 'physicalAndTimingCoefficientsIdentical',true,'originalSeed',before.sensor.seed, ...
 'priorJointCaseId',before.uncertainty.priorJointCaseId);
end

function changed=parameter_changes(before,after,prefix)
changed={};if isequaln(before,after),return;end
if isstruct(before) && isstruct(after) && isscalar(before) && isscalar(after)
 names=union(fieldnames(before),fieldnames(after));
 for index=1:numel(names)
  name=names{index};path=name;if ~isempty(prefix),path=[prefix '.' name];end
  if ~isfield(before,name) || ~isfield(after,name)
   changed{end+1}=path; %#ok<AGROW>
  else
   changed=[changed,parameter_changes(before.(name),after.(name),path)]; %#ok<AGROW>
  end
 end
else
 changed={prefix};
end
end

function add_residual_instruments(mdl)
axial=[mdl '/Force measurement'];lateral=[mdl '/Lateral B-S force measurement'];
position=[mdl '/Relative position measurement'];
insert_gain(axial,'Tared cell force/1','Sensor sampling/1','Residual axial calibration', ...
 '1+p.uncertainty.forceScaleZ',[45 520 255 575]);
insert_gain(lateral,'Lateral reaction/1','Sensor sampling/1','Residual lateral calibration', ...
 '1+p.uncertainty.forceScaleX',[20 860 275 915]);
add_force_frame_mix(axial,lateral);
initialZ='p.force.reference*(1+p.uncertainty.forceScaleZ)*(cos(p.uncertainty.forceFrameAngle_rad)-p.contact.mu*(sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX))';
initialX='p.force.reference*(1+p.uncertainty.forceScaleX)*(-p.contact.mu*cos(p.uncertainty.forceFrameAngle_rad)-sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ)';
set_param([axial '/Previous filtered force'],'InitialCondition',initialZ);
set_param([axial '/Sensor sample delay'],'InitialCondition',[initialZ '+p.sensor.bias']);
set_param([lateral '/Filter memory'],'InitialCondition',initialX);
set_param([lateral '/Sensor sample delay'],'InitialCondition',[initialX '+p.contact.lateralBias_N']);
% The lateral causal lead is inactive in the selected v2 baseline; keep its
% initial state consistent should a selected candidate enable it.
set_param([lateral '/Previous measured lateral force'],'InitialCondition',[initialX '+p.contact.lateralBias_N']);
if getSimulinkBlockHandle([axial '/Engineered common AA and EMA'])~=-1
 set_param([axial '/Engineered common AA and EMA'],'InitialStates', ...
  ['(' initialZ ')*p.d.acquisition.responses.force.steadyStatePerUnitInput']);
 set_param([lateral '/Engineered common AA and EMA'],'InitialStates', ...
  ['(' initialX ')*p.d.acquisition.responses.lateral.steadyStatePerUnitInput']);
end
delete_line(position,'Select position instrument/1','Position sampling/1');
add_block('simulink/Math Operations/Bias',[position '/Position calibrated zero'], ...
 'Position',[25 700 275 755],'Bias','-p.d.positionMeasurementInitial');
add_block('simulink/Math Operations/Gain',[position '/Residual position calibration'], ...
 'Position',[370 700 625 755],'Gain','1+p.uncertainty.positionScale');
add_block('simulink/Math Operations/Bias',[position '/Position calibrated origin'], ...
 'Position',[730 700 980 755],'Bias','p.d.positionMeasurementInitial');
add_line(position,'Select position instrument/1','Position calibrated zero/1','autorouting','on');
add_line(position,'Position calibrated zero/1','Residual position calibration/1','autorouting','on');
add_line(position,'Residual position calibration/1','Position calibrated origin/1','autorouting','on');
add_line(position,'Position calibrated origin/1','Position sampling/1','autorouting','on');
imu=[mdl '/S accelerometer and ADC'];
add_accel_frame_mix(imu);
set_param([imu '/X drift after tare'],'Value','p.uncertainty.accelBiasX_m_s2');
set_param([imu '/X known channel delay'],'InitialCondition','p.uncertainty.accelBiasX_m_s2');
controller=coil_controller_path(mdl);
feedforward=find_unique(controller,'Measured acceleration current feedforward');
set_param(feedforward,'Gain', ...
 '-p.inertial.accelerationFeedforwardScale*(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)/p.uncertainty.forceConstant_model_N_A');
set_param(find_unique(controller,'Flat holding current'),'Value', ...
 '(p.force.reference-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A');
set_param([mdl '/Recenter travel/Recenter gain'],'Gain','p.uncertainty.recenterGain');
end

function add_force_frame_mix(axial,lateral)
add_block('simulink/Signal Routing/Goto',[axial '/Uncalibrated axial cell for sensor matrix'], ...
 'Position',[350 800 650 830],'GotoTag','uncertainty_uncalibrated_cell_z','TagVisibility','global');
add_line(axial,'Tared cell force/1','Uncalibrated axial cell for sensor matrix/1','autorouting','on');
add_block('simulink/Signal Routing/From',[axial '/Uncalibrated lateral cell for sensor matrix'], ...
 'Position',[30 680 315 710],'GotoTag','lateral_contact_N');
insert_frame_mix(axial,'Tared cell force/1','Uncalibrated lateral cell for sensor matrix/1', ...
 'Residual axial calibration/1','Force Z frame','cos(p.uncertainty.forceFrameAngle_rad)', ...
 'sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX',[30 950]);
add_block('simulink/Signal Routing/From',[lateral '/Uncalibrated axial cell for sensor matrix'], ...
 'Position',[30 970 315 1000],'GotoTag','uncertainty_uncalibrated_cell_z');
insert_frame_mix(lateral,'Lateral reaction/1','Uncalibrated axial cell for sensor matrix/1', ...
 'Residual lateral calibration/1','Force X frame','cos(p.uncertainty.forceFrameAngle_rad)', ...
 '-sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ',[30 1080]);
end

function add_accel_frame_mix(sys)
delta='p.uncertainty.accelFrameAngle_rad';gZ='p.d.g';gX='p.mechanics.gravity*sin(p.mechanics.angle)';
insert_frame_mix(sys,'Z specific acceleration/1','X specific acceleration/1','Z sensor scale/1', ...
 'Acceleration Z frame',['cos(' delta ')'],['sin(' delta ')'],[20 1280]);
insert_frame_mix(sys,'X specific acceleration/1','Z specific acceleration/1','X sensor scale/1', ...
 'Acceleration X frame',['cos(' delta ')'],['-sin(' delta ')'],[20 1560]);
rotatedZ=['cos(' delta ')*(' gZ ')+sin(' delta ')*(' gX ')'];
rotatedX=['cos(' delta ')*(' gX ')-sin(' delta ')*(' gZ ')'];
axes={'Z','X'};gravities={rotatedZ,rotatedX};
for j=1:2
 scaled=['(1+p.accel.scaleError)*(' gravities{j} ')'];
 set_param([sys '/' axes{j} ' filter memory'],'InitialCondition',scaled);
 set_param([sys '/' axes{j} ' known static tare'],'Value',['round((' scaled ')/p.accel.quantum)*p.accel.quantum']);
 if getSimulinkBlockHandle([sys '/' axes{j} ' engineered common AA and EMA'])~=-1
  set_param([sys '/' axes{j} ' engineered common AA and EMA'],'InitialStates', ...
   ['(' scaled ')*p.d.acquisition.responses.acceleration.steadyStatePerUnitInput']);
 end
end
end

function insert_frame_mix(sys,ownSource,crossSource,destination,prefix,ownGain,crossGain,xy)
x=xy(1);y=xy(2);delete_line(sys,ownSource,destination);
add_block('simulink/Math Operations/Gain',[sys '/' prefix ' diagonal'], ...
 'Position',[x y x+250 y+55],'Gain',ownGain);
add_block('simulink/Math Operations/Gain',[sys '/' prefix ' cross axis'], ...
 'Position',[x y+105 x+250 y+160],'Gain',crossGain);
add_block('simulink/Math Operations/Sum',[sys '/' prefix ' calibrated sum'], ...
 'Position',[x+355 y+25 x+395 y+115],'Inputs','++');
add_line(sys,ownSource,[prefix ' diagonal/1'],'autorouting','on');
add_line(sys,crossSource,[prefix ' cross axis/1'],'autorouting','on');
add_line(sys,[prefix ' diagonal/1'],[prefix ' calibrated sum/1'],'autorouting','on');
add_line(sys,[prefix ' cross axis/1'],[prefix ' calibrated sum/2'],'autorouting','on');
add_line(sys,[prefix ' calibrated sum/1'],destination,'autorouting','on');
end

function controller=coil_controller_path(mdl)
controller=[mdl '/Force controller/Coil PI and PID'];
if getSimulinkBlockHandle(controller)==-1,controller=[mdl '/Force controller/Voice coil inertial control'];end
assert(getSimulinkBlockHandle(controller)~=-1,'Unsupported native coil controller contract.');
end

function path=find_unique(parent,name)
paths=find_system(parent,'MatchFilter',@Simulink.match.allVariants,'Name',name);
assert(isscalar(paths),'Expected one native block named %s.',name);path=paths{1};
end

function insert_gain(sys,source,destination,name,gain,xy)
delete_line(sys,source,destination);
add_block('simulink/Math Operations/Gain',[sys '/' name],'Position',xy,'Gain',gain);
add_line(sys,source,[name '/1'],'autorouting','on');
add_line(sys,[name '/1'],destination,'autorouting','on');
end

function row=measure_run(q,r,id,config,runtime,exception)
variationEnabled=isfield(q.robot.forceGate,'variation') && q.robot.forceGate.variation.enabled;
row=struct('id',id,'scenario',config.scenario,'parameter',config.parameter,'value',config.value, ...
 'massS_actual_kg',q.mechanics.mS,'massS_identified_kg',q.sensor.distalMass_model_kg, ...
 'massB_actual_kg',q.mechanics.mB,'massB_identified_kg',q.uncertainty.massB_model_kg, ...
 'lowerRigid',logical(q.mechanics.lowerRigid),'upperRigid',logical(q.mechanics.upperRigid), ...
 'lowerLinkStiffness_N_m',q.mechanics.k1,'lowerLinkDamping_N_s_m',q.mechanics.c1, ...
 'reference_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi,'mu',q.contact.mu,'seed',q.sensor.seed, ...
 'physicalScanSpeed_m_s',q.surface.scanSpeed,'identifiedScanSpeed_m_s',q.contact.commandedScanSpeed_m_s, ...
 'forceKp_A_N',q.inertial.forceKp,'forceKi_A_Ns',q.inertial.forceKi,'forceKd_A_s_N',q.inertial.forceKd, ...
 'accelerationPrediction_s',q.inertial.accelerationPrediction_s, ...
 'accelerationFFfilter_s',q.inertial.accelerationFeedforwardFilter_s, ...
 'measurementFilterDcDelay_s',q.sensor.sampleTime/expm1(q.sensor.sampleTime/q.sensor.filterTime), ...
 'measurementTotalDcDelay_s',q.sensor.delay+q.sensor.sampleTime/expm1(q.sensor.sampleTime/q.sensor.filterTime), ...
 'forceBiasZ_N',q.sensor.bias,'forceBiasX_N',q.contact.lateralBias_N, ...
 'forceScaleZ',q.uncertainty.forceScaleZ,'forceScaleX',q.uncertainty.forceScaleX, ...
 'forceFrameAngle_deg',q.uncertainty.forceFrameAngle_rad*180/pi, ...
 'accelFrameAngle_deg',q.uncertainty.accelFrameAngle_rad*180/pi, ...
 'forceCrossZFromX',q.uncertainty.forceCrossZFromX,'forceCrossXFromZ',q.uncertainty.forceCrossXFromZ, ...
 'forceNoiseRms_N',q.sensor.noiseRms,'forceFilterTime_s',q.sensor.filterTime,'forceDelay_s',q.sensor.delay, ...
 'lateralForceDelay_s',q.contact.lateralDelaySamples*q.sensor.sampleTime, ...
 'accelBiasZ_m_s2',q.accel.bias,'accelBiasX_m_s2',q.uncertainty.accelBiasX_m_s2, ...
 'accelScaleError',q.accel.scaleError,'accelNoiseRms_m_s2',q.accel.noiseRms, ...
 'accelRange_g',q.accel.range_g,'accelQuantum_m_s2',q.accel.quantum, ...
 'accelFilterTime_s',q.accel.filterTime,'accelDelay_s',q.accel.delay, ...
 'positionNoiseRms_m',q.sensor.strokeNoiseRms,'positionScale',q.uncertainty.positionScale, ...
 'positionDelay_s',q.sensor.strokeDelay,'robotDelay_actual_s',q.robot.delay, ...
 'robotResponse_actual_s',q.robot.responseTime,'robotDelay_identified_s',q.robotPrediction.delay, ...
 'robotResponse_identified_s',q.robotPrediction.responseTime,'recenterGain_per_s',q.uncertainty.recenterGain, ...
 'surfaceKind',q.surface.kind,'stopTime_s',q.sim.stopTime,'simulationStep_s',q.sim.step, ...
 'executed',~isempty(r),'nativeReturned',false,'passed',false,'completed',false,'lostContact',false,'strokeStop',false, ...
 'runtime_s',runtime,'endTime_s',NaN,'nativeSamples',0,'peakNormalForceError_N',NaN, ...
 'rmsNormalForceError_N',NaN,'peakNormalForceErrorTime_s',NaN,'peakBeforeBumpError_N',NaN, ...
 'peakDuringBumpError_N',NaN,'peakAfterBumpError_N',NaN,'meanSettledNormalError_N',NaN, ...
 'peakNormalEstimateError_N',NaN,'rmsNormalEstimateError_N',NaN,'peakSlopeEstimateError',NaN, ...
 'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN,'voltageClipFraction',NaN, ...
 'effortClipFraction',NaN,'minTravel_mm',NaN,'maxTravel_mm',NaN,'gateReadyFraction',NaN, ...
 'gateClosures',NaN,'meanRateGateReadyFraction',NaN,'variationGateEnabled',logical(variationEnabled), ...
 'variationGateReadyFraction',NaN,'peakMeasuredForceVariationRms_N',NaN, ...
 'peakRobotSpeedWhileGateClosed_m_s',NaN,'normalBalanceResidual_N',NaN, ...
 'axialForceBalanceResidual_N',NaN,'lowerLinkBalanceResidual_N',NaN, ...
 'rigidAxialBalanceResidual_N',NaN,'nativeTrace','','reason',exception,'axialDiagnosticReason','');
row.initialPhysicalCurrent_A=NaN;row.actualEquilibriumCurrent_A=NaN;
row.lowerInverseEnabled=false;row.inverseMassB_identified_kg=NaN;
row.inverseKf_identified_N_A=NaN;row.inverseK_identified_N_m=NaN;
row.inverseC_identified_N_s_m=NaN;
row.packingPassed=true;row.configuredCommonPreloadPassed=true;row.commonPreload_N=NaN;
row.feasibleCommonPreloadMin_N=NaN;row.feasibleCommonPreloadMax_N=NaN;
row.combinedFreeLengthMax_mm=0;row.combinedInstalledLengthMax_mm=0;
row.forceConstant_actual_N_A=q.coil.forceConstant;
row.forceConstant_identified_N_A=q.uncertainty.forceConstant_model_N_A;
row.engineeredAcquisitionEnabled=false;
row.actualAAcutoff_Hz=NaN;row.actualAAfilterDamping=NaN;
row.identifiedAAcutoff_Hz=NaN;row.actualEmaTime_s=NaN;row.identifiedEmaTime_s=NaN;
row.actualPipelineDelay_s=NaN;row.identifiedPipelineDelay_s=NaN;
row.forceAAchannelScale=NaN;row.lateralAAchannelScale=NaN;
row.accelAAchannelScale=NaN;row.positionAAchannelScale=NaN;
row.forceFilterDcDelay_s=NaN;row.accelFilterDcDelay_s=NaN;
row.positionFilterDcDelay_s=NaN;row.identifiedFrameFilterDcDelay_s=NaN;
if isfield(q,'acquisition') && q.acquisition.enabled
 c=q.acquisition;row.engineeredAcquisitionEnabled=true;
 row.actualAAcutoff_Hz=c.antiAliasCutoff_Hz;row.actualAAfilterDamping=c.antiAliasDamping;
 row.identifiedAAcutoff_Hz=c.identifiedAntiAliasCutoff_Hz;
 row.actualEmaTime_s=c.emaTime_s;row.identifiedEmaTime_s=c.identifiedEmaTime_s;
 row.actualPipelineDelay_s=c.pipelineDelay_s;row.identifiedPipelineDelay_s=c.identifiedPipelineDelay_s;
 row.forceAAchannelScale=c.forceAntiAliasScale;row.lateralAAchannelScale=c.lateralAntiAliasScale;
 row.accelAAchannelScale=c.accelerationAntiAliasScale;row.positionAAchannelScale=c.positionAntiAliasScale;
 h=c.rawSampleTime_s;responses=q.d.acquisition.responses;
 row.forceFilterDcDelay_s=transfer_dc_delay(responses.force,h);
 row.accelFilterDcDelay_s=transfer_dc_delay(responses.acceleration,h);
 row.positionFilterDcDelay_s=transfer_dc_delay(responses.position,h);
 row.identifiedFrameFilterDcDelay_s=transfer_dc_delay(responses.frame,h);
 row.measurementFilterDcDelay_s=row.forceFilterDcDelay_s;
 row.measurementTotalDcDelay_s=q.sensor.delay+row.forceFilterDcDelay_s;
end
if isempty(r),return;end
assert(abs(r.parameters.sensor.distalMass_model_kg-q.sensor.distalMass_model_kg)<1e-12, ...
 'Prepared helpers changed the requested identified S mass.');
assert(r.parameters.mechanics.mS==q.mechanics.mS && r.parameters.mechanics.mB==q.mechanics.mB, ...
  'A calibration test changed an actual physical mass.');
if row.engineeredAcquisitionEnabled
 c=r.parameters.acquisition;
 assert(c.identifiedAntiAliasCutoff_Hz==q.acquisition.identifiedAntiAliasCutoff_Hz && ...
  c.identifiedEmaTime_s==q.acquisition.identifiedEmaTime_s && ...
  c.identifiedPipelineDelay_s==q.acquisition.identifiedPipelineDelay_s, ...
  'Actual sensing mismatch silently retuned the identified issued-motion frame.');
 assert(abs(r.parameters.d.actualRobotTransportSampleTime_s-q.force.sampleTime)<1e-12, ...
  'The actual robot transport does not use the native fine clock.');
end
if isfield(r.parameters.d,'identifiedHolderMass_kg')
 assert(abs(r.parameters.d.identifiedHolderMass_kg-q.uncertainty.massB_model_kg)<1e-12, ...
  'Prepared helpers changed the requested identified B mass.');
end
if isfield(r.parameters.d,'coilForceConstant_model_N_A')
 assert(abs(r.parameters.d.coilForceConstant_model_N_A-q.uncertainty.forceConstant_model_N_A)<1e-12, ...
  'Prepared helpers changed the requested identified force constant.');
end
row.initialPhysicalCurrent_A=r.data(1,14);
row.actualEquilibriumCurrent_A=r.parameters.d.coilHoldCurrent;
assert(abs(row.initialPhysicalCurrent_A-row.actualEquilibriumCurrent_A)<1e-10, ...
 'Calibration changed the actual physical starting current.');
if isfield(r.parameters.inertial,'lowerLoadInverse')
 c=r.parameters.inertial.lowerLoadInverse;
 row.lowerInverseEnabled=logical(c.enabled);row.inverseMassB_identified_kg=c.identifiedMassB_kg;
 row.inverseKf_identified_N_A=c.identifiedForceConstant_N_A;
 row.inverseK_identified_N_m=c.identifiedStiffness_N_m;
 row.inverseC_identified_N_s_m=c.identifiedDamping_Ns_m;
 assert(abs(c.identifiedMassB_kg-q.uncertainty.massB_model_kg)<1e-12 && ...
  abs(c.identifiedForceConstant_N_A-q.uncertainty.forceConstant_model_N_A)<1e-12, ...
  'The inverse mass/force constant differs from the other identified FF terms.');
 assert(c.identifiedStiffness_N_m==q.inertial.lowerLoadInverse.identifiedStiffness_N_m && ...
  c.identifiedDamping_Ns_m==q.inertial.lowerLoadInverse.identifiedDamping_Ns_m, ...
  'Preparing the controller changed its identified spring/damper model.');
 assert(isfield(r,'lowerInverse') && isequal(r.time,r.lowerInverse.time) && size(r.lowerInverse.data,2)==4, ...
  'The native lower-load inverse audit is missing.');
 assert(max(abs(r.lowerInverse.data(:,3)-r.lowerInverse.data(:,1)+r.lowerInverse.data(:,2)))<1e-10, ...
  'Native desired lower-load subtraction failed.');
end
normal=r.normalContact.data;error=normal(:,1)-q.force.reference;metrics=r.metrics;
[row.peakNormalForceError_N,which]=max(abs(error));row.peakNormalForceErrorTime_s=r.time(which);
row.rmsNormalForceError_N=sqrt(mean(error.^2));row.completed=metrics.completed;
row.lostContact=metrics.lostContact;row.strokeStop=metrics.strokeStopHit;
row.passed=metrics.completed && row.peakNormalForceError_N<=q.force.tolerance && ...
 ~metrics.lostContact && ~metrics.strokeStopHit && ~metrics.overCurrent && metrics.impactCount==0;
if isfield(r.parameters,'springs')
 [row.packingPassed,row.feasibleCommonPreloadMin_N,row.feasibleCommonPreloadMax_N, ...
  row.combinedFreeLengthMax_mm,row.combinedInstalledLengthMax_mm, ...
  row.configuredCommonPreloadPassed]=configured_packing(r);
 row.commonPreload_N=r.parameters.springs.commonPreload_N;
 row.passed=row.passed && row.packingPassed;
end
row.endTime_s=r.time(end);row.nativeSamples=numel(r.time);
start=q.surface.startTime;stop=start+r.parameters.d.duration;
row.peakBeforeBumpError_N=masked_peak(error,r.time<start);
row.peakDuringBumpError_N=masked_peak(error,r.time>=start & r.time<=stop);
row.peakAfterBumpError_N=masked_peak(error,r.time>stop);
settled=r.time>max(stop+.5,q.sim.stopTime-.5);
if any(settled),row.meanSettledNormalError_N=mean(error(settled));end
row.peakNormalEstimateError_N=max(abs(normal(:,4)-normal(:,1)));
row.rmsNormalEstimateError_N=sqrt(mean((normal(:,4)-normal(:,1)).^2));
row.peakSlopeEstimateError=max(abs(normal(:,5)-normal(:,3)));
row.peakCurrent_A=metrics.peakCurrent_A;row.rmsCurrent_A=metrics.rmsCurrent_A;
row.peakVoltage_V=metrics.peakVoltage_V;row.voltageClipFraction=metrics.voltageSaturationFraction;
row.effortClipFraction=metrics.effortSaturationFraction;row.minTravel_mm=metrics.minTravel_mm;row.maxTravel_mm=metrics.maxTravel_mm;
actualSlope=r.data(:,29)/q.surface.scanSpeed;
expectedNormal=r.data(:,1).*sqrt(1+actualSlope.^2)./(1-q.contact.mu*actualSlope);
row.normalBalanceResidual_N=max(abs(expectedNormal-normal(:,1)));
row.axialForceBalanceResidual_N=max(abs(r.data(:,1)-r.data(:,13)- ...
 q.mechanics.mB*(r.parameters.d.g+r.data(:,12))- ...
 q.mechanics.mS*(r.parameters.d.g+r.data(:,30))));
row.lowerLinkBalanceResidual_N=max(abs(r.data(:,1)-r.data(:,24)- ...
 q.mechanics.mS*(r.parameters.d.g+r.data(:,30))));
if q.mechanics.lowerRigid && q.mechanics.upperRigid
 row.rigidAxialBalanceResidual_N=max(abs(r.data(:,1)-r.data(:,13)- ...
  (q.mechanics.mB+q.mechanics.mS)*(r.parameters.d.g+r.data(:,30))));
end
assert(max([row.normalBalanceResidual_N,row.axialForceBalanceResidual_N,row.lowerLinkBalanceResidual_N])<1e-8, ...
 'Native force balance failed.');
if isfield(r,'robotGate')
 row.meanRateGateReadyFraction=mean(r.robotGate.ready);
 gateOnNative=interp1(r.robotGate.time,r.robotGate.ready,r.time,'previous','extrap');
 if isfield(r,'forceVariation')
  row.variationGateReadyFraction=mean(r.forceVariation.data(:,5));
  row.peakMeasuredForceVariationRms_N=max(r.forceVariation.data(:,2));
  if variationEnabled
   variationOnNative=interp1(r.forceVariation.time,r.forceVariation.data(:,5),r.time,'previous','extrap');
   gateOnNative=min(gateOnNative,variationOnNative);
  end
 end
 row.gateReadyFraction=mean(gateOnNative);
 row.gateClosures=sum(diff(gateOnNative)<0);
 stopped=gateOnNative<.5;
 if any(stopped),row.peakRobotSpeedWhileGateClosed_m_s=max(abs(r.data(stopped,27)));end
end
row.axialDiagnosticReason=metrics.reason;
reasons={};if row.peakNormalForceError_N>q.force.tolerance,reasons{end+1}='normal force error';end
if metrics.lostContact,reasons{end+1}='contact loss';end
if metrics.strokeStopHit,reasons{end+1}='travel stop';end
if metrics.overCurrent,reasons{end+1}='overcurrent';end
if ~metrics.completed,reasons{end+1}='incomplete';end
if ~row.packingPassed,reasons{end+1}='configured spring preload/packing';end
if isempty(reasons),row.reason='pass';else,row.reason=strjoin(reasons,', ');end
end

function [passed,low,high,totalFree,totalInstalled,configuredPassed]=configured_packing(r)
% Native displacement extrema bound both opposed springs' actual fixed
% preload. Do not substitute a newly feasible preload after seeing a run.
p=r.parameters;d=r.data;pack=p.springs;low=0;high=inf;totalFree=0;totalInstalled=0;
links={'lower','upper'};rigid=[p.mechanics.lowerRigid,p.mechanics.upperRigid];
delta={d(:,7)-d(:,6),d(:,5)-d(:,26)};offset=[p.d.q10,p.d.q20];
stiffness=[p.mechanics.k1,p.mechanics.k2];excursionPass=true;
for j=1:2
 if rigid(j),continue;end
 s=pack.(links{j});ks=s.rate_N_m;shift=offset(j)/stiffness(j)+delta{j};
 compressionOffset=max(abs(shift));
 low=max(low,ks*(pack.unseatingClearance_m+compressionOffset));
 maximumCompression=min(s.freeLength_m-s.freeLengthTolerance_m-s.minWorkingLength_m-pack.workingClearance_m, ...
  s.forceEnvelope_N/ks);
 high=min(high,ks*(maximumCompression-compressionOffset));
 excursionPass=excursionPass && max(shift)-min(shift)<=s.maxCyclicStroke_m+1e-12;
 totalFree=totalFree+2000*(s.freeLength_m+s.freeLengthTolerance_m);
 totalInstalled=totalInstalled+2000*(s.freeLength_m+s.freeLengthTolerance_m-pack.commonPreload_N/ks);
end
configuredPassed=pack.commonPreload_N>=low-1e-9 && pack.commonPreload_N<=high+1e-9;
passed=low<=high && configuredPassed && excursionPass && ...
 totalFree<=1000*pack.maxCombinedLength_m+1e-9 && ...
 totalInstalled<=1000*pack.maxCombinedLength_m+1e-9;
end

function ranking=rank_scenarios(rows)
ranking=struct([]);scenarios=unique({rows.scenario},'stable');
for j=1:numel(scenarios)
 cohort=rows(strcmp({rows.scenario},scenarios{j}));valid=cohort([cohort.executed]);
 score=NaN;if ~isempty(valid),score=max([valid.peakNormalForceError_N]);end
 one=struct('scenario',scenarios{j},'attempts',numel(cohort),'nativeHistories',numel(valid), ...
  'passedCases',sum([cohort.passed]),'worstNormalForceError_N',score,'minimumForceMargin_N',.5-score,'rowIds',[cohort.id]);
 if isempty(ranking),ranking=one;else,ranking(end+1)=one;end %#ok<AGROW>
end
end

function value=masked_peak(values,mask)
if any(mask),value=max(abs(values(mask)));else,value=NaN;end
end

function delay=transfer_dc_delay(response,dt)
b=response.numerator;a=response.denominator;
delay=dt*(sum((0:numel(b)-1).*b)/sum(b)-sum((0:numel(a)-1).*a)/sum(a));
end

function trace=plot_excerpt(r,row)
stride=max(1,round(max(250e-6,r.parameters.sim.stopTime/10000)/r.parameters.sim.step));
take=1:stride:numel(r.time);
forceBins=nan(stride,numel(take));forceBins(1:numel(r.time))=r.normalContact.data(:,1);
trace=struct('row',row,'plotStep_s',stride*r.parameters.sim.step,'time_s',r.time(take), ...
 'normalForceMinimum_N',min(forceBins,[],1,'omitnan')', ...
 'normalForceMaximum_N',max(forceBins,[],1,'omitnan')', ...
 'forceEnvelopeConvention','All native samples within each plot bin; normalLog and other diagnostics use strided samples. Envelope retains every native force peak.', ...
 'normalLog',r.normalContact.data(take,:),'accelerometerLog',r.accelerometer.data(take,:), ...
 'motionObserver',r.motionObserver.data(take,:),'current_A',r.data(take,14),'voltage_V',r.data(take,15), ...
 'travel_m',r.data(take,2),'robotVelocity_m_s',r.data(take,27), ...
 'gateReady',interp1(r.robotGate.time,r.robotGate.ready,r.time(take),'previous','extrap'));
end

function result=sha256(file)
if ~isfile(file),file=[file '.m'];end
assert(isfile(file),'Hash input missing.');single=char(39);
escaped=strrep(file,single,[single,char(34),single,char(34),single]);
[status,output]=system(['sha256sum -- ',single,escaped,single]);
assert(status==0,'Hash utility failed.');result=regexp(output,'[0-9a-f]{64}','match','once');
assert(~isempty(result),'Invalid SHA-256 output.');
end

function snapshot=process_memory()
% Linux VmHWM is the kernel's process-lifetime peak RSS, not a polling estimate.
snapshot=struct('resident_kB',NaN,'peakResident_kB',NaN,'swap_kB',NaN, ...
 'hostAvailable_kB',NaN,'hostSwapFree_kB',NaN, ...
 'hostMemoryConvention','Host free memory is a checkpoint reading, not a continuous minimum.');
if isfile('/proc/self/status')
 status=fileread('/proc/self/status');
 for pair={{'VmRSS','resident_kB'},{'VmHWM','peakResident_kB'},{'VmSwap','swap_kB'}}
  match=regexp(status,[pair{1}{1} ':\s+(\d+)\s+kB'],'tokens','once');
  if ~isempty(match),snapshot.(pair{1}{2})=str2double(match{1});end
 end
end
if isfile('/proc/meminfo')
 status=fileread('/proc/meminfo');
 for pair={{'MemAvailable','hostAvailable_kB'},{'SwapFree','hostSwapFree_kB'}}
  match=regexp(status,[pair{1}{1} ':\s+(\d+)\s+kB'],'tokens','once');
  if ~isempty(match),snapshot.(pair{1}{2})=str2double(match{1});end
 end
end
end

function finish(mdl,previousStorage,previousRecord,ownedStorage,removeTemporaryStorage,auditFile)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
if strcmp(Simulink.sdi.getStorageLocation,ownedStorage),Simulink.sdi.clear;end
Simulink.sdi.setStorageLocation(previousStorage);Simulink.sdi.setRecordData(previousRecord);
removed=false;cleanupMessage='Persistent private SDI folder retained.';
if removeTemporaryStorage
 assert(startsWith(ownedStorage,'/dev/shm/inertial-uncertainty-'), ...
  'Cleanup must only remove this process''s freshly created SDI folder.');
 if isfolder(ownedStorage)
  [removed,cleanupMessage]=rmdir(ownedStorage,'s');
 else
  removed=true;cleanupMessage='Private temporary SDI folder was already removed.';
 end
end
audit=struct('ownedStorage',ownedStorage,'storagePreferencesRestored', ...
 strcmp(Simulink.sdi.getStorageLocation,previousStorage) && ...
 isequal(Simulink.sdi.getRecordData,previousRecord), ...
 'temporaryStorageRequested',removeTemporaryStorage,'temporaryStorageRemoved',removed, ...
 'message',cleanupMessage);
fid=fopen(auditFile,'w');if fid~=-1,fprintf(fid,'%s',jsonencode(audit,PrettyPrint=true));fclose(fid);end
end
