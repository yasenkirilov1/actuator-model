function report=qualify_inertial_actuators_final(sourceFolder,outputFolder,options)
%QUALIFY_INERTIAL_ACTUATORS_FINAL Run the reviewed public native donor unchanged.
% Eight paired histories plus a 5-us screw refinement of the existing 10-us
% heavy noisy row. No graph grafting, source edits, or controller tuning.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
if nargin<3,options=struct;end
options=defaults(options,struct('tag','actual_acquisition_v8_20260930', ...
 'simulationMode','accelerator','stopTime_s',4,'baseStep_s',10e-6, ...
 'refinedStep_s',5e-6,'refine',true));
assert(ismember(options.simulationMode,{'normal','accelerator'}));
modelSource=fullfile(sourceFolder,'generated','acf_force_control_simple.slx');
candidateSource=fullfile(sourceFolder,'delivery_candidate.mat');
assert(isfile(modelSource)&&isfile(candidateSource),'Wait for the immutable combined donor.');
if ~isfolder(outputFolder),mkdir(outputFolder);end
stem=['inertial_actuators_final_' options.tag];resultFile=fullfile(outputFolder,[stem '.json']);
assert(~isfile(resultFile),'Retained results are immutable; choose a new tag.');
worker=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','helpers','cache','code','sdi'}
 path=fullfile(worker,name{1});if ~isfolder(path),mkdir(path);end
end
modelFile=fullfile(worker,'acf_force_control_simple.slx');
copyfile(modelSource,modelFile);copyfile(candidateSource,fullfile(worker,'candidate.mat'));
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(worker,'helpers'));
recipeFile=fullfile(worker,'recipe_snapshot.m');copyfile([mfilename('fullpath') '.m'],recipeFile);
modelSHA=sha256(modelSource);candidateSHA=sha256(candidateSource);
assert(strcmp(modelSHA,sha256(modelFile))&&strcmp(candidateSHA,sha256(fullfile(worker,'candidate.mat'))));
helperFiles=dir(fullfile(worker,'helpers','*.m'));helperHashes=struct([]);
for j=1:numel(helperFiles)
 helperHashes=append(helperHashes,struct('file',helperFiles(j).name, ...
  'sha256',sha256(fullfile(helperFiles(j).folder,helperFiles(j).name))));
end
addpath(fullfile(worker,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator acf_select_controller;
oldThreads=maxNumCompThreads;maxNumCompThreads(2);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(worker,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_simple';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,worker,oldStorage,oldRecord,oldThreads)); %#ok<NASGU>
set_param(mdl,'FastRestart','off','SimulationMode',options.simulationMode,'AlgebraicLoopMsg','error');
saved=load(fullfile(worker,'candidate.mat'),'p');base=saved.p;
assert(base.acquisition.enabled&&base.accel.enabled&&base.robotPrediction.enabled);
assert(strcmp(base.sensor.positionTarget,'tool_relative_A'));
assert(base.mechanics.upperRigid&&~base.mechanics.lowerRigid&&base.mechanics.mB==.6);
assert(base.inertial.lowerLoadInverse.enabled&&strcmp(base.screw.controlStrategy,'current'));
assert(base.robot.forceGate.enabled&&base.robot.forceGate.variation.enabled);
assert(base.force.sampleTime==100e-6&&base.acquisition.rawSampleTime_s==10e-6);
assert(base.coil.currentResponse==100e-6);
assert(base.surface.height==.001&&base.surface.width==.010&&base.surface.scanSpeed==.100);
assert(base.sensor.distalMassCalibrationScale==1&&base.inertial.holderMassCalibrationScale==1);
base.sim.stopTime=options.stopTime_s;base.sim.evaluateFrom=0;base.sim.step=options.baseStep_s;
configs=struct([]);conditions=[2 50 90 0;2 50 90 1729;2 5 0 1729;.1 50 90 1729];
for actuator={'voice_coil','leadscrew'}
 for j=1:size(conditions,1)
  q=physical_case(base,conditions(j,:));q.actuator.kind=actuator{1};
  configs=append(configs,struct('p',q,'purpose','paired_actual_acquisition','seedRequested',conditions(j,4)));
 end
end
if options.refine
 q=configs(6).p;q.sim.step=options.refinedStep_s;
 configs=append(configs,struct('p',q,'purpose','heavy_noisy_screw_solver_refinement','seedRequested',1729));
end
report=struct('status','prepared','date','2026-09-30','options',options, ...
 'sourceFolder',sourceFolder,'sourceModelSHA256',modelSHA,'candidateSHA256',candidateSHA, ...
 'privateWorkerFolder',worker,'privateModelFile',modelFile,'privateModelSHA256',sha256(modelFile), ...
 'recipeFile',recipeFile,'recipeSHA256',sha256(recipeFile),'helperHashes',helperHashes, ...
 'plannedCases',numel(configs),'attemptedCases',0,'executedCases',0,'passedCases',0,'setupRejections',0, ...
 'metric','max(abs(true native workpiece normal force - reference)) at every native timestamp including startup; no contact/travel stops or overcurrent', ...
 'recordingNote','One sharp 1mm-by-10mm event at100mm/s in each full4s recording; not a repeat test.', ...
 'controllerNote','Reviewed native current controllers and causal common sensing; no new topology, sensor, preview, or gain tuning.', ...
 'coilProvision',base.coil,'screwProvision',base.screw,'commonInertialProvision',base.inertial, ...
 'acquisitionProvision',base.acquisition,'sensorProvision',base.sensor,'accelProvision',base.accel, ...
 'robotProvision',base.robot,'springProvision',base.springs,'algebraicLoopCount',0,'rows',struct([]));
save(fullfile(outputFolder,[stem '_configurations.mat']),'configs','options','-v7');
rows=struct([]);started=tic;compileKey='';
for j=1:numel(configs)
 q=configs(j).p;r=[];exception='';timer=tic;
 key=sprintf('%s_%g',q.actuator.kind,q.sim.step);
 if ~strcmp(key,compileKey),set_param(mdl,'FastRestart','off');compileKey='';end
 try
  q=acf_prepare(q);mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',q);
  q=acf_select_actuator(mdl,q.actuator.kind);
  if isempty(compileKey)
   set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
   set_param(mdl,'SimulationCommand','update');
   assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)),'Native donor has an algebraic loop.');
   set_param(mdl,'FastRestart','on');compileKey=key;
  end
  evalc('r=acf_run(mdl,q,false);');
 catch err
  exception=getReport(err,'extended','hyperlinks','off');
  set_param(mdl,'FastRestart','off');compileKey='';
 end
 try
  row=measure(q,r,j,configs(j),toc(timer),exception);
 catch validationError
  % Keep the native output even when its logger/physics contract is invalid.
  % Null metrics prevent a verification failure from masquerading as a pass.
  exception=getReport(validationError,'extended','hyperlinks','off');
  row=measure(q,[],j,configs(j),toc(timer),exception);
  row.executed=~isempty(r);row.failureReason='native verification error';
 end
 if ~isempty(r)
  row.nativeTraceFile=fullfile(outputFolder,sprintf('%s_native_%04d.mat',stem,j));
  save(row.nativeTraceFile,'r','row','-v7');
 end
 rows=append(rows,row);report.rows=rows;report.attemptedCases=numel(rows);
 report.executedCases=sum([rows.executed]);report.passedCases=sum([rows.passed]);
 report.setupRejections=sum(~[rows.executed]);report.elapsedSeconds=toc(started);
 report.status='running';checkpoint();clear r;
 fprintf('FINAL_ACTUATORS %d/%d %s S%.1f F%.0f angle%.0f seed%d step%.1fus: %.9gN pass%d I%.6g V%.6g\n', ...
  j,numel(configs),row.actuator,row.massS_kg,row.referenceForce_N,row.angle_deg, ...
  row.seedRequested,row.step_s*1e6,row.peakNormalForceError_N,row.passed,row.peakCurrent_A,row.peakVoltage_V);
 Simulink.sdi.clear;
 if isfile(fullfile(worker,'stop_after_case')),report.status='stopped_by_coordinator';break;end
end
set_param(mdl,'FastRestart','off');
assert(strcmp(modelSHA,sha256(modelSource))&&strcmp(modelSHA,sha256(modelFile)));
assert(strcmp(candidateSHA,sha256(candidateSource)),'Frozen candidate changed during experiment.');
if options.refine&&numel(rows)==9&&rows(6).executed&&rows(9).executed
 report.solverRefinement=struct('baselineCaseIndex',6,'refinedCaseIndex',9, ...
  'baselineStep_s',rows(6).step_s,'refinedStep_s',rows(9).step_s, ...
  'baselinePeakError_N',rows(6).peakNormalForceError_N, ...
  'refinedPeakError_N',rows(9).peakNormalForceError_N, ...
  'peakErrorChange_N',rows(9).peakNormalForceError_N-rows(6).peakNormalForceError_N, ...
  'baselinePassed',rows(6).passed,'refinedPassed',rows(9).passed);
end
if ~strcmp(report.status,'stopped_by_coordinator'),report.status='complete';end
checkpoint();fprintf('FINAL_ACTUATORS_COMPLETE: %d executed/%d attempted, %d pass, %d setup rejects.\n', ...
 report.executedCases,report.attemptedCases,report.passedCases,report.setupRejections);

 function checkpoint()
  write_json(resultFile,report);
  if ~isempty(rows),writetable(struct2table(rows,'AsArray',true),fullfile(outputFolder,[stem '_cases.csv']));end
  save(fullfile(outputFolder,[stem '_checkpoint.mat']),'report','configs','rows','-v7');
 end
end

function q=physical_case(base,values)
q=base;q.mechanics.mS=values(1);q.force.reference=values(2);q.mechanics.angle=values(3)*pi/180;
q.contact.mu=.3;q.surface.kind='sharp_bump';q.sensor.seed=values(4);
if q.sensor.seed==0
 q.sensor.seed=29;q.sensor.noiseRms=0;q.sensor.strokeNoiseRms=0;
 q.contact.lateralNoiseRms_N=0;q.accel.noiseRms=0;
end
end

function row=measure(q,r,index,config,elapsed,exception)
row=struct('caseIndex',index,'purpose',config.purpose,'actuator',q.actuator.kind, ...
 'massS_kg',q.mechanics.mS,'massB_kg',q.mechanics.mB,'referenceForce_N',q.force.reference, ...
 'angle_deg',q.mechanics.angle*180/pi,'seedRequested',config.seedRequested,'seed',q.sensor.seed, ...
 'step_s',q.sim.step,'requestedDuration_s',q.sim.stopTime,'forceNoiseRms_N',q.sensor.noiseRms, ...
 'positionNoiseRms_m',q.sensor.strokeNoiseRms,'accelNoiseRms_m_s2',q.accel.noiseRms, ...
 'rawAcquisitionPeriod_s',q.acquisition.rawSampleTime_s,'forceControllerPeriod_s',q.force.sampleTime, ...
 'pipelineDelay_s',q.acquisition.pipelineDelay_s,'forceVarianceGateEnabled',q.robot.forceGate.variation.enabled, ...
 'effectiveLowerLinkStiffness_N_m',q.mechanics.k1,'effectiveLowerLinkDamping_Ns_m',q.mechanics.c1, ...
 'coilForceKp_A_N',q.inertial.forceKp,'coilForceKi_A_N_s',q.inertial.forceKi, ...
 'screwForceKp_A_N',q.screw.currentForceKp,'screwForceKi_A_N_s',q.screw.currentForceKi, ...
 'screwPitch_m',q.screw.lead,'rotorInertia_kg_m2',q.screw.rotorInertia, ...
 'executed',~isempty(r),'passed',false,'completed',false,'contactLoss',false,'strokeStop',false,'overCurrent',false, ...
 'peakNormalForceError_N',NaN,'rmsNormalForceError_N',NaN,'peakErrorTime_s',NaN,'nativeSamples',NaN, ...
 'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN,'currentClipFraction',NaN,'voltageClipFraction',NaN, ...
 'holdCurrent_A',NaN,'holdCopperLoss_W',NaN,'continuousStaticCatalogPassed',NaN, ...
 'normalBalanceResidual_N',NaN,'downstreamForceBalanceResidual_N',NaN, ...
 'actuatedForceBalanceResidual_N',NaN,'rotorForceBalanceResidual_N',NaN,'physicsPassed',false, ...
 'actualDuration_s',NaN,'minTravel_mm',NaN,'maxTravel_mm',NaN, ...
 'forceVarianceReadyFraction',NaN,'inverseEffectiveMass_kg',NaN,'inverseForceConstant_N_A',NaN, ...
 'springPackingPassed',false,'combinedFreeSpringLength_mm',NaN,'combinedInstalledSpringLength_mm',NaN, ...
 'minimumIndividualSpringCompression_mm',NaN,'peakIndividualSpringForce_N',NaN, ...
 'failureReason','','executionError',exception,'executionSeconds',elapsed,'nativeTraceFile','');
if isempty(r),row.failureReason='preparation or simulation error';return;end
q=r.parameters;t=r.time;d=r.data;n=r.normalContact.data;
assert(isfield(r,'axialContact')&&isequal(t,r.normalContact.time,r.axialContact.time));
assert(isequal(d(:,1),n(:,1)),'Public force channel must be native workpiece normal force.');
assert(isequal(t,r.accelerometer.time,r.motionObserver.time,r.lowerInverse.time,r.forceVariation.time));
axial=r.axialContact.force_N;error=n(:,1)-q.force.reference;
[row.peakNormalForceError_N,at]=max(abs(error));row.peakErrorTime_s=t(at);
row.rmsNormalForceError_N=sqrt(mean(error.^2));row.nativeSamples=numel(t);
row.completed=t(end)>=q.sim.stopTime-q.sim.step/2;row.actualDuration_s=t(end);
row.contactLoss=any(d(:,18)<.5);row.strokeStop=any(d(:,21)>.5);
row.peakCurrent_A=max(abs(d(:,14)));row.rmsCurrent_A=sqrt(mean(d(:,14).^2));
row.peakVoltage_V=max(abs(d(:,15)));row.currentClipFraction=mean(d(:,22)>.5);row.voltageClipFraction=mean(d(:,23)>.5);
if strcmp(q.actuator.kind,'voice_coil')
 limit=q.coil.peakCurrent;hold=q.d.coilHoldCurrent;resistance=q.coil.resistance;c=q.inertial.lowerLoadInverse;
else
 limit=q.screw.peakCurrent;hold=q.d.screwHoldCurrent;resistance=q.screw.resistance;c=q.d.screwLowerLoadInverse;
 row.continuousStaticCatalogPassed=abs(hold)<=q.screw.continuousCurrent_A;
end
row.overCurrent=row.peakCurrent_A>limit*(1+1e-4);row.holdCurrent_A=hold;row.holdCopperLoss_W=hold^2*resistance;
row.inverseEffectiveMass_kg=c.identifiedMassB_kg;row.inverseForceConstant_N_A=c.identifiedForceConstant_N_A;
row.minTravel_mm=min(d(:,2))*1000;row.maxTravel_mm=max(d(:,2))*1000;
slope=d(:,29)/q.surface.scanSpeed;
row.normalBalanceResidual_N=max(abs(axial.*sqrt(1+slope.^2)./(1-q.contact.mu*slope)-n(:,1)));
contact=d(:,18)>.5;
row.downstreamForceBalanceResidual_N=max(abs(axial(contact)-d(contact,24)-q.mechanics.mS*(q.d.g+d(contact,30))));
row.actuatedForceBalanceResidual_N=max(abs(axial(contact)-d(contact,13)-q.mechanics.mB*(q.d.g+d(contact,12))-q.mechanics.mS*(q.d.g+d(contact,30))));
row.physicsPassed=max([row.normalBalanceResidual_N,row.downstreamForceBalanceResidual_N,row.actuatedForceBalanceResidual_N])<1e-8;
if strcmp(q.actuator.kind,'leadscrew')
 omega=d(:,17);friction=q.screw.coulombTorque*tanh(omega/q.screw.frictionSpeed)+q.screw.viscousTorque*omega;
 torque=q.screw.torqueConstant*d(:,14)-friction;
 row.rotorForceBalanceResidual_N=max(abs(torque/q.d.ell-d(:,13)-q.screw.rotorInertia/q.d.ell^2*(d(:,11)-d(:,12))));
 row.physicsPassed=row.physicsPassed&&row.rotorForceBalanceResidual_N<1e-7;
end
row.forceVarianceReadyFraction=mean(r.forceVariation.data(:,5)>.5);
assert(isfield(r.metrics,'springSizing'),'Public native metrics must verify selected spring packing.');
packing=r.metrics.springSizing;row.springPackingPassed=packing.passed;
row.combinedFreeSpringLength_mm=packing.combinedFreeLengthMax_mm;
row.combinedInstalledSpringLength_mm=packing.combinedInstalledLengthMax_mm;
row.minimumIndividualSpringCompression_mm=packing.lower.minCompression_mm;
row.peakIndividualSpringForce_N=packing.lower.peakIndividualForce_N;
row.passed=row.completed&&row.peakNormalForceError_N<=q.force.tolerance&& ...
 ~row.contactLoss&&~row.strokeStop&&~row.overCurrent&&row.physicsPassed&&row.springPackingPassed;
reason={};if ~row.completed,reason{end+1}='incomplete';end
if row.contactLoss,reason{end+1}='contact loss';end;if row.strokeStop,reason{end+1}='travel stop';end
if row.overCurrent,reason{end+1}='overcurrent';end;if ~row.physicsPassed,reason{end+1}='force balance failed';end
if ~row.springPackingPassed,reason{end+1}='spring packing or working range';end
if row.peakNormalForceError_N>q.force.tolerance,reason{end+1}='normal force error';end
if isempty(reason),row.failureReason='pass';else,row.failureReason=strjoin(reason,', ');end
end

function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end
function s=defaults(s,d)
names=fieldnames(d);for j=1:numel(names),if ~isfield(s,names{j}),s.(names{j})=d.(names{j});end,end
end
function write_json(path,value)
fid=fopen([path '.pending'],'w');assert(fid>=0);fwrite(fid,jsonencode(value,'PrettyPrint',true),'char');fclose(fid);
movefile([path '.pending'],path,'f');
end
function value=sha256(path)
quote=char(39);escaped=strrep(path,quote,[quote char(34) quote char(34) quote]);
[status,out]=system(['sha256sum -- ' quote escaped quote]);assert(status==0);value=out(1:64);
end
function finish(mdl,worker,storage,record,threads)
try,if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end;catch,end
try
 if strcmp(Simulink.sdi.getStorageLocation,fullfile(worker,'sdi')),Simulink.sdi.clear;end
 Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);
catch,end
try,maxNumCompThreads(threads);catch,end
end
