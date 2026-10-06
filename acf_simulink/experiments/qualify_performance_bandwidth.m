function report=qualify_performance_bandwidth(projectFolder,outputFolder,workerFolder)
%QUALIFY_PERFORMANCE_BANDWIDTH Final two-sensor native performance diagnostics.
% Runs an isolated copied SLX/helpers; does not save or change the shared model.
% Source metrics use all native samples. Plot excerpts are explicitly separate.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
if nargin<1,projectFolder=fileparts(fileparts(mfilename('fullpath')));end
if nargin<2,outputFolder=fullfile(projectFolder,'validation');end
if nargin<3,workerFolder=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));end
maxNumCompThreads(2);
stem='performance_bandwidth_20260930';
resultFile=fullfile(outputFolder,[stem '_results.json']);
assert(~isfile(resultFile),'Preserve existing evidence and use a new output directory.');
for name={'','helpers','cache','code','sdi','prefs'}
 folder=fullfile(workerFolder,name{1});if ~isfolder(folder),mkdir(folder);end
end
if ~isfolder(outputFolder),mkdir(outputFolder);end
sourceModel=fullfile(projectFolder,'generated','acf_force_control_simple.slx');
candidateFile=fullfile(projectFolder,'validation','final_two_sensor_candidate.mat');
modelHash=sha256(sourceModel);candidateHash=sha256(candidateFile);
modelFile=fullfile(workerFolder,'acf_force_control_simple.slx');copyfile(sourceModel,modelFile);
copyfile(fullfile(projectFolder,'acf_*.m'),fullfile(workerFolder,'helpers'));
copyfile(candidateFile,fullfile(workerFolder,'candidate.mat'));
addpath(fullfile(workerFolder,'helpers'),'-begin');clear acf_prepare acf_run;
sourceFiles=dir(fullfile(workerFolder,'helpers','*.m'));sources=struct([]);
for j=1:numel(sourceFiles)
 row=struct('file',sourceFiles(j).name,'sha256',sha256(fullfile(sourceFiles(j).folder,sourceFiles(j).name)));
 if isempty(sources),sources=row;else,sources(end+1)=row;end %#ok<AGROW>
end
saved=load(fullfile(workerFolder,'candidate.mat'),'p');p=saved.p;
p.sim.stopTime=4;p.sim.evaluateFrom=0;p.sim.step=50e-6;
p.robot.forceGate.filterTime=.1;p.robot.forceGate.dwellTime=.1;
assert(p.mechanics.lowerRigid && p.mechanics.upperRigid && p.mechanics.mB==.6 && p.mechanics.mS==1.4);
assert(p.sensor.distalMass_kg==1.4 && p.sensor.distalMass_model_kg==1.4);
assert(strcmp(p.sensor.positionTarget,'tool_relative_A') && p.robotPrediction.enabled);
assert(p.coil.directCoulombCorrection && p.force.sampleTime==100e-6);
assert(p.coil.voltage==24 && p.coil.peakCurrent==3.5);
p=acf_prepare(p);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(workerFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_simple';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,fullfile(workerFolder,'sdi'))); %#ok<NASGU>
regulator=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil/Current regulator'];
assert(getSimulinkBlockHandle(regulator)~=-1,'Expected native current regulator missing.');
normalRegulator=get_param(regulator,'Gain');
configs=struct([]);points=[50 60;50 90;5 0;5 90];
% Speed changes only: force/position sensors and all control models stay fixed.
for speed=[50 75 100 125 150 200]*1e-3
 for point=1:4
  for seed=[7 29]
   q=p;q.surface.scanSpeed=speed;q.force.reference=points(point,1);
   q.mechanics.angle=points(point,2)*pi/180;q.sensor.seed=seed;
   add_config(q,'scan_speed',q.coil.currentResponse,false);
  end
 end
end
% Sensor-noise ablation does not retune covariance weights or controller gains.
noises=[0 0;0 1e-6;.02 0;.02 1e-6];
for speed=[100 125]*1e-3
 for point=1:2
  for seed=[7 29]
   for noise=1:size(noises,1)
    q=p;q.surface.scanSpeed=speed;q.force.reference=points(point,1);
    q.mechanics.angle=points(point,2)*pi/180;q.sensor.seed=seed;
    q.sensor.noiseRms=noises(noise,1);q.sensor.strokeNoiseRms=noises(noise,2);
    add_config(q,'sensor_noise_ablation',q.coil.currentResponse,false);
   end
  end
 end
end
% Identified response: recompute observer and state feedback for actual tau.
% All these cases use 25 us integration, including a 100 us baseline.
for tau=[50 100 200]*1e-6
 for speed=[100 125]*1e-3
  for point=1:2
   for seed=[7 29]
    q=p;q.sim.step=25e-6;q.coil.currentResponse=tau;q.surface.scanSpeed=speed;
    q.force.reference=points(point,1);q.mechanics.angle=points(point,2)*pi/180;q.sensor.seed=seed;
    add_config(q,'identified_current_response',tau,false);
   end
  end
 end
end
% Mismatched response: only the native inner-current regulator changes.
% The outer observer/controller still assume the original 100 us response.
for tau=[50 200]*1e-6
 for speed=[100 125]*1e-3
  for point=1:2
   for seed=[7 29]
    q=p;q.sim.step=25e-6;q.surface.scanSpeed=speed;q.force.reference=points(point,1);
    q.mechanics.angle=points(point,2)*pi/180;q.sensor.seed=seed;
    add_config(q,'unidentified_current_response',tau,true);
   end
  end
 end
end
% Known delay/filter variants each change one channel and recompute observer.
% The force channel keeps one 100 us delay sample for causal Coulomb correction.
for scenario={'known_position_delay_zero','known_force_filter_50us'}
 for speed=[100 125]*1e-3
  for point=1:2
   for seed=[7 29]
    q=p;q.surface.scanSpeed=speed;q.force.reference=points(point,1);
    q.mechanics.angle=points(point,2)*pi/180;q.sensor.seed=seed;
    if strcmp(scenario{1},'known_position_delay_zero'),q.sensor.strokeDelay=0;
    else,q.sensor.filterTime=50e-6;end
    add_config(q,scenario{1},q.coil.currentResponse,false);
   end
  end
 end
end
rows=struct([]);plotTraces=struct([]);started=tic;lastCompileKey=[];
report=struct('date','2026-09-30','status','running','recipeVersion','final_two_sensor_bandwidth_v1', ...
 'scope','136 finite diagnostic cases, axial contact force over the entire 0-4 s recording. Not full-range hardware qualification.', ...
 'modelFile',sourceModel,'modelSha256',modelHash,'privateModel',modelFile, ...
 'candidateFile',candidateFile,'candidateSha256',candidateHash,'helperSources',sources, ...
 'experimentSha256',sha256(mfilename('fullpath')),'matlabVersion',version, ...
 'baseParameters',p,'plannedCases',numel(configs),'rows',rows, ...
 'physicalSensing','B-S cell with full 1.4 kg downstream, exact known weight tare; S-A position. No current readback, no measured acceleration, no surface preview.', ...
 'voltageInterpretation','Supply stays 24 V. Configured response is active inner-current bandwidth, independent of voltage until voltage clipping. Identified variants recompute outer model; unidentified variants change only the native regulator gain.', ...
 'noiseInterpretation','Independent post-filter sample noise; covariance weights stay .02 N and 1 um even in ablations. Seed is common but simulation trajectories and random sampling can differ.', ...
 'metricInterpretation','Metrics use every native step and include initialization, flat segments and bump. Estimate metrics use all logged 100 us observer samples. Plot excerpts are explicitly decimated.');
for j=1:numel(configs)
 config=configs(j);q=config.p;
 compileKey=[q.sim.step,q.coil.currentResponse,config.actualTau, ...
  q.sensor.filterTime,q.sensor.delay,q.sensor.strokeDelay,config.frozenOuter];
 if ~isequal(lastCompileKey,compileKey)
  set_param(mdl,'FastRestart','off');
  if config.frozenOuter,set_param(regulator,'Gain',sprintf('p.coil.inductance/%.17g',config.actualTau));
  else,set_param(regulator,'Gain',normalRegulator);end
  set_param(mdl,'FastRestart','on');lastCompileKey=compileKey;
 end
 timer=tic;r=[];exception='';
 try
  quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
 catch failure
  exception=getReport(failure,'extended','hyperlinks','off');
  fprintf('PERFORMANCE_SETUP_FAILURE %d %s\n',j,failure.message);
 end
 row=measure_run(q,r,j,config,toc(timer),exception);
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 if ~isempty(r)
  selected=(q.force.reference==50 && round(q.mechanics.angle*180/pi)==60 && q.sensor.seed==29);
  capture=selected && ((strcmp(config.scenario,'scan_speed') && any(q.surface.scanSpeed==[.1 .2])) || ...
   (strcmp(config.scenario,'sensor_noise_ablation') && q.surface.scanSpeed==.1 && q.sensor.noiseRms==0 && q.sensor.strokeNoiseRms==0) || ...
   (strcmp(config.scenario,'identified_current_response') && q.surface.scanSpeed==.1) || ...
   (strcmp(config.scenario,'unidentified_current_response') && q.surface.scanSpeed==.1) || ...
   (startsWith(config.scenario,'known_') && q.surface.scanSpeed==.1));
  if capture
   trace=plot_excerpt(r,row);if isempty(plotTraces),plotTraces=trace;else,plotTraces(end+1)=trace;end %#ok<AGROW>
   if strcmp(config.scenario,'scan_speed') || (strcmp(config.scenario,'sensor_noise_ablation') && q.sensor.noiseRms==0 && q.sensor.strokeNoiseRms==0)
    save(fullfile(outputFolder,sprintf('%s_native_trace_%03d.mat',stem,j)),'r','row','-v7');
   end
  end
 end
 clear r;
 checkpoint('running');
 if mod(j,4)==0 || j==1
  fprintf('PERFORMANCE_PROGRESS %d/%d, passed %d, last %s %.9g N at %.1f mm/s, elapsed %.1f s.\n', ...
   j,numel(configs),sum([rows.passed]),row.scenario,row.peakTrueForceError_N,1e3*q.surface.scanSpeed,toc(started));
 end
 if mod(j,8)==0,Simulink.sdi.clear;end
end
assert(strcmp(sha256(sourceModel),modelHash),'Shared native model changed during this experiment.');
report.nativeSourceUnmodified=true;checkpoint('complete');
fprintf('PERFORMANCE_COMPLETE %d/%d executed, %d passed.\n',report.executed,report.plannedCases,report.passed);

 function add_config(q,scenario,actualTau,frozenOuter)
  row=struct('p',q,'scenario',scenario,'actualTau',actualTau,'frozenOuter',frozenOuter);
  if isempty(configs),configs=row;else,configs(end+1)=row;end %#ok<AGROW>
 end
 function checkpoint(status)
  report.status=status;report.rows=rows;report.executed=sum([rows.executed]);
  report.passed=sum([rows.passed]);report.rejectedBeforeMetrics=sum(~[rows.executed]);
  report.elapsedSeconds=toc(started);
  fid=fopen(resultFile,'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  writetable(struct2table(rows),fullfile(outputFolder,[stem '_rows.csv']));
  save(fullfile(outputFolder,[stem '_results.mat']),'report','configs','-v7');
  save(fullfile(outputFolder,[stem '_plot_traces.mat']),'plotTraces','-v7');
 end
end

function row=measure_run(q,r,id,config,runtime,exception)
row=struct('id',id,'scenario',config.scenario,'scanSpeed_mm_s',q.surface.scanSpeed*1000, ...
 'reference_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi,'seed',q.sensor.seed, ...
 'forceNoiseRms_N',q.sensor.noiseRms,'positionNoiseRms_m',q.sensor.strokeNoiseRms, ...
 'forceFilterTime_s',q.sensor.filterTime,'forceDelay_s',q.sensor.delay,'positionDelay_s',q.sensor.strokeDelay, ...
 'actualCurrentResponse_s',config.actualTau,'outerModelCurrentResponse_s',q.coil.currentResponse, ...
 'outerModelFrozen',config.frozenOuter,'simulationStep_s',q.sim.step,'voltageLimit_V',q.coil.voltage, ...
 'currentLimit_A',q.coil.peakCurrent,'executed',~isempty(r),'passed',false,'completed',false, ...
 'lostContact',false,'strokeStop',false,'runtime_s',runtime,'endTime_s',NaN,'nativeSamples',0, ...
 'peakTrueForceError_N',NaN,'rmsTrueForceError_N',NaN,'peakTrueForceErrorTime_s',NaN, ...
 'peakBeforeBumpError_N',NaN,'peakDuringBumpError_N',NaN,'peakAfterBumpError_N',NaN, ...
 'peakEstimateTrackingError_N',NaN,'peakEstimateForceError_N',NaN,'rmsEstimateForceError_N',NaN, ...
 'peakEstimatedSurfaceAccelerationError_m_s2',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN, ...
 'peakVoltage_V',NaN,'voltageClipFraction',NaN,'effortClipFraction',NaN, ...
 'peakSurfaceAcceleration_m_s2',NaN,'peakAnalyticSurfaceJerk_m_s3',NaN, ...
 'gateReadyFraction',NaN,'reason',exception);
if isempty(r),return;end
metrics=r.metrics;row.passed=metrics.passed;row.completed=metrics.completed;
row.lostContact=metrics.lostContact;row.strokeStop=metrics.strokeStopHit;
row.endTime_s=r.time(end);row.nativeSamples=numel(r.time);row.reason=metrics.reason;
error=r.data(:,1)-q.force.reference;[row.peakTrueForceError_N,which]=max(abs(error));
row.peakTrueForceErrorTime_s=r.time(which);row.rmsTrueForceError_N=sqrt(mean(error.^2));
start=q.surface.startTime;stop=start+r.parameters.d.duration;
row.peakBeforeBumpError_N=masked_peak(error,r.time<start);
row.peakDuringBumpError_N=masked_peak(error,r.time>=start & r.time<=stop);
row.peakAfterBumpError_N=masked_peak(error,r.time>stop);
row.peakCurrent_A=metrics.peakCurrent_A;row.rmsCurrent_A=metrics.rmsCurrent_A;
row.peakVoltage_V=metrics.peakVoltage_V;row.voltageClipFraction=metrics.voltageSaturationFraction;
row.effortClipFraction=metrics.effortSaturationFraction;
row.peakSurfaceAcceleration_m_s2=max(abs(r.data(:,30)));
% Sharp bump h=64 H s^3 (1-s)^3; jerk maximum magnitude occurs at endpoints.
row.peakAnalyticSurfaceJerk_m_s3=384*q.surface.height/r.parameters.d.duration^3;
if isfield(r,'robotGate'),row.gateReadyFraction=mean(r.robotGate.ready);end
if isfield(r,'observerStates')
 states=r.observerStates.data;time=r.observerStates.time;
 estimate=q.force.reference+states*r.parameters.d.coilControl.force';
 if q.coil.directCoulombCorrection && q.mechanics.lowerRigid
  estimate=estimate-q.coil.coulombForce*tanh(states(:,6)/q.coil.frictionSpeed);
 end
 actual=interp1(r.time,r.data(:,1),time,'linear');
 row.peakEstimateTrackingError_N=max(abs(estimate-q.force.reference));
 row.peakEstimateForceError_N=max(abs(estimate-actual));
 row.rmsEstimateForceError_N=sqrt(mean((estimate-actual).^2));
 actualAcceleration=interp1(r.time,r.data(:,30),time,'linear');
 row.peakEstimatedSurfaceAccelerationError_m_s2=max(abs(states(:,8)-actualAcceleration));
end
end

function value=masked_peak(values,mask)
if any(mask),value=max(abs(values(mask)));else,value=NaN;end
end

function trace=plot_excerpt(r,row)
step=round(100e-6/r.parameters.sim.step);
take=find(r.time>=1.95 & r.time<=2.3);take=take(1:step:end);
states=r.observerStates;which=states.time>=1.95 & states.time<=2.3;
estimate=r.parameters.force.reference+states.data(which,:)*r.parameters.d.coilControl.force';
estimate=estimate-r.parameters.coil.coulombForce*tanh(states.data(which,6)/r.parameters.coil.frictionSpeed);
trace=struct('row',row,'plotStep_s',100e-6,'time_s',r.time(take), ...
 'forceError_N',r.data(take,1)-r.parameters.force.reference,'current_A',r.data(take,14), ...
 'voltage_V',r.data(take,15),'surfaceAcceleration_m_s2',r.data(take,30), ...
 'observerTime_s',states.time(which),'estimatedForceError_N',estimate-r.parameters.force.reference, ...
 'estimatedSurfaceAcceleration_m_s2',states.data(which,8));
end

function result=sha256(file)
if ~endsWith(file,'.m') && ~isfile(file),file=[file '.m'];end
fid=fopen(file,'rb');assert(fid>=0);bytes=fread(fid,inf,'*uint8');fclose(fid);
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
result=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end

function finish(mdl,previousStorage,previousRecord,ownedStorage)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
if strcmp(Simulink.sdi.getStorageLocation,ownedStorage),Simulink.sdi.clear;end
Simulink.sdi.setStorageLocation(previousStorage);Simulink.sdi.setRecordData(previousRecord);
end
