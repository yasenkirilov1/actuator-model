function report=benchmark_inertial_native_runner(sourceFolder,candidateFile,outputFolder,options)
%BENCHMARK_INERTIAL_NATIVE_RUNNER Verify seeds/logs before accelerating sweeps.
% Fixed native topology and known mass. The full histories from ordinary
% native simulations are the reference for FastRestart, omitted duplicate SDI
% recordings, and Accelerator. All copied-model changes are private.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch session.');
project=fileparts(fileparts(mfilename('fullpath')));
if nargin<2 || isempty(candidateFile),candidateFile=fullfile(sourceFolder,'normal_contact_trial_candidate.mat');end
if nargin<3 || isempty(outputFolder),outputFolder=fullfile(project,'validation');end
if nargin<4,options=struct;end
options=defaults(options,struct('massS_kg',2,'simulationStep_s',NaN, ...
 'tryAccelerator',true,'includeSDIOff',false,'tag',''));
stem=['normal_mass_controller_runner_20260930' options.tag];
resultFile=fullfile(outputFolder,[stem '_results.json']);assert(~isfile(resultFile));
if ~isfolder(outputFolder),mkdir(outputFolder);end
workerFolder=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','helpers','cache','code','sdi','reference'}
 folder=fullfile(workerFolder,name{1});if ~isfolder(folder),mkdir(folder);end
end
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
modelHash=sha256(sourceModel);modelFile=fullfile(workerFolder,'acf_force_control_normal_trial.slx');
candidateHash=sha256(candidateFile);
copyfile(sourceModel,modelFile);
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile(candidateFile,fullfile(workerFolder,'candidate.mat'));
assert(strcmp(candidateHash,sha256(fullfile(workerFolder,'candidate.mat'))), ...
 'Candidate changed during copy; preserve a single input version.');
recipeSnapshot=fullfile(workerFolder,'benchmark_inertial_native_runner.m');
copyfile([mfilename('fullpath') '.m'],recipeSnapshot);
addpath(fullfile(workerFolder,'helpers'),'-begin');
clear acf_prepare acf_run acf_select_actuator acf_select_controller;
files=dir(fullfile(workerFolder,'helpers','*.m'));sources=struct([]);
for j=1:numel(files)
 sources=append(sources,struct('file',files(j).name, ...
  'sha256',sha256(fullfile(files(j).folder,files(j).name))));
end
maxNumCompThreads(2);
saved=load(fullfile(workerFolder,'candidate.mat'),'p');p=saved.p;
p.sim.stopTime=4;p.sim.evaluateFrom=0;
if isfinite(options.simulationStep_s),p.sim.step=options.simulationStep_s;end
p.mechanics.mS=options.massS_kg;p.sensor.distalMass_kg=options.massS_kg;
p.sensor.distalMass_model_kg=options.massS_kg;
p=acf_prepare(p);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
ownedStorage=fullfile(workerFolder,'sdi');Simulink.sdi.setStorageLocation(ownedStorage);
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
mdl='acf_force_control_normal_trial';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,ownedStorage)); %#ok<NASGU>
% Case3 repeats case1 after a different seed. Exact paired histories establish
% reset/reseed behavior, not merely different summary scores.
conditions=[50 90 7;50 90 29;50 90 7;5 0 7;50 90 1729];
modes=struct('name',{'normal_recompile_SDI_on','normal_FastRestart_SDI_on', ...
 'accelerator_FastRestart_SDI_on'}, ...
 'simulationMode',{'normal','normal','accelerator'}, ...
 'fastRestart',{'off','on','on'},'recordSDI',{true,true,true});
if ~options.tryAccelerator,modes=modes(1:2);end
if options.includeSDIOff
 modes(end+1)=struct('name','normal_FastRestart_SDI_off', ...
  'simulationMode','normal','fastRestart','on','recordSDI',false);
 if options.tryAccelerator
  modes(end+1)=struct('name','accelerator_FastRestart_SDI_off', ...
   'simulationMode','accelerator','fastRestart','on','recordSDI',false);
 end
end
rows=struct([]);started=tic;expectedFiles=cell(size(conditions,1),1);
report=struct('date','2026-09-30','status','running','recipeVersion','native_runner_parity_v2', ...
 'sourceModel',sourceModel,'modelSha256',modelHash,'candidateFile',candidateFile, ...
 'candidateSha256',candidateHash,'privateModel',modelFile, ...
 'experimentSnapshot',recipeSnapshot,'experimentSha256',sha256(recipeSnapshot), ...
 'helperSources',sources, ...
 'matlabVersion',version,'baseParameters',p,'options',options,'conditions',conditions, ...
 'plannedAttempts',numel(modes)*size(conditions,1), ...
 'scope','Fixed-mass/fixed-topology full native histories, force/orientation/seed changes. Sampling and notch remain fixed; no hardware qualification.', ...
 'parityChannels',{{'time','original35','normal8','accelerometer6','motion2', ...
 'commandPrediction_if_available','robotGate_if_available','forceVariation_if_available', ...
 'accelerationFeedforward_if_available','measuredDrag_if_available', ...
 'all_additional_native_time_data_logs_if_available'}}, ...
 'strictGate','Each faster path must honor requested seeds and preserve every manual full-rate channel. Exact equality is reported separately from small numeric differences.', ...
 'rows',rows);
for modeIndex=1:numel(modes)
 mode=modes(modeIndex);
 set_param(mdl,'FastRestart','off','SimulationMode',mode.simulationMode);
 Simulink.sdi.clear;Simulink.sdi.setRecordData(mode.recordSDI);
 set_param(mdl,'FastRestart',mode.fastRestart);
 for caseIndex=1:size(conditions,1)
  q=p;q.force.reference=conditions(caseIndex,1);
  q.mechanics.angle=conditions(caseIndex,2)*pi/180;q.sensor.seed=conditions(caseIndex,3);
  q.contact.mu=.3;q=acf_prepare(q);timer=tic;r=[];nativeReturned=false;exception='';nativeRuntime=NaN;
  try
   quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
   nativeRuntime=toc(timer);
   nativeReturned=true;
   assert(isequal(r.time,r.normalContact.time,r.accelerometer.time,r.motionObserver.time));
   if modeIndex==1
    expectedFiles{caseIndex}=fullfile(workerFolder,'reference',sprintf('case_%02d.mat',caseIndex));
    save(expectedFiles{caseIndex},'r','q','-v7');
    parity=struct('exact',true,'maximumAbsoluteDifference',0,'channelDifferences',struct);
   else
    loaded=load(expectedFiles{caseIndex},'r');parity=compare_histories(loaded.r,r);clear loaded;
   end
  catch failure
   exception=getReport(failure,'extended','hyperlinks','off');
   parity=struct('exact',false,'maximumAbsoluteDifference',NaN,'channelDifferences',struct);
  end
  runtime=toc(timer);duration=NaN;samples=0;normalPeak=NaN;
  if ~isempty(r)
   duration=r.time(end);samples=numel(r.time);
   normalPeak=max(abs(r.normalContact.data(:,1)-q.force.reference));
  end
  row=struct('mode',mode.name,'modeIndex',modeIndex,'caseIndex',caseIndex, ...
   'force_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi,'seed',q.sensor.seed, ...
   'massS_kg',q.mechanics.mS,'simulationStep_s',q.sim.step,'recordSDI',mode.recordSDI, ...
   'simulationMode',mode.simulationMode,'fastRestart',mode.fastRestart, ...
   'nativeReturned',nativeReturned,'completed',duration>=q.sim.stopTime-q.sim.step, ...
   'duration_s',duration,'samples',samples,'runtimeIncludingValidation_s',runtime, ...
   'nativeSimulationAndMetrics_s',nativeRuntime, ...
   'validationAndReferenceIO_s',runtime-nativeRuntime, ...
   'peakNormalForceError_N',normalPeak,'exactManualHistoryParity',parity.exact, ...
   'maximumAbsoluteDifference',parity.maximumAbsoluteDifference, ...
   'channelDifferences',parity.channelDifferences,'SDIBytesAfterRun',folder_bytes(ownedStorage), ...
   'reason',exception);
  rows=append(rows,row);checkpoint('running');
  fprintf('RUNNER_PARITY %s %d/%d: seed%d F%.0f angle%.0f,%.3fs,exact%d,maxdiff%.3g,SDI%dbytes.\n', ...
   mode.name,caseIndex,size(conditions,1),q.sensor.seed,q.force.reference, ...
   q.mechanics.angle*180/pi,runtime,row.exactManualHistoryParity, ...
   row.maximumAbsoluteDifference,row.SDIBytesAfterRun);
  if ~isempty(exception)
   fprintf('RUNNER_MODE_REJECTED %s: %s\n',mode.name,exception);
   break;
  end
  Simulink.sdi.clear;
 end
 set_param(mdl,'FastRestart','off');
end
first=load(expectedFiles{1},'r');repeat=load(expectedFiles{3},'r');
report.referenceRepeatedSeedParity=compare_histories(first.r,repeat.r);
other=load(expectedFiles{2},'r');
report.differentSeedsChangeManualHistories=~isequaln(first.r.accelerometer.data,other.r.accelerometer.data);
report.referenceNativeTraceFiles=expectedFiles;
report.referenceSeed7PeakNormal_N=max(abs(first.r.normalContact.data(:,1)-50));
report.referenceSeed29PeakNormal_N=max(abs(other.r.normalContact.data(:,1)-50));
clear first repeat other;
report.modeSummaries=summarize_modes(rows,modes,size(conditions,1));
report.sourceModelUnchanged=strcmp(sha256(sourceModel),modelHash);
report.privateFrozenModelUnchanged=strcmp(sha256(modelFile),modelHash);
assert(report.sourceModelUnchanged && report.privateFrozenModelUnchanged);
checkpoint('complete');

 function checkpoint(status)
  report.status=status;report.rows=rows;report.attempts=numel(rows);
  report.nativeReturns=sum([rows.nativeReturned]);
  report.setupFailures=sum(~[rows.nativeReturned]);report.elapsedSeconds=toc(started);
  fid=fopen(resultFile,'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  save(fullfile(outputFolder,[stem '_results.mat']),'report','-v7');
 end
end

function parity=compare_histories(a,b)
fields={'time','data','normalContact','accelerometer','motionObserver', ...
 'commandPrediction','robotGate','forceVariation','accelerationFeedforward','measuredDrag'};
% New native controller logs must be covered without silently dropping a
% source-version-specific channel, such as the proper lower-load inverse.
names=union(fieldnames(a),fieldnames(b),'stable');
for j=1:numel(names)
 name=names{j};sample=[];
 if isfield(a,name),sample=a.(name);elseif isfield(b,name),sample=b.(name);end
 if isstruct(sample) && isfield(sample,'time') && isfield(sample,'data') && ...
   ~ismember(name,fields)
  fields{end+1}=name; %#ok<AGROW>
 end
end
differences=struct;maximum=0;exact=true;
for j=1:numel(fields)
 name=fields{j};presentA=isfield(a,name);presentB=isfield(b,name);
 assert(presentA==presentB,'Manual channel availability changed.');if ~presentA,continue;end
 x=a.(name);y=b.(name);
 if isstruct(x)
  nested=fieldnames(x);difference=0;
  for k=1:numel(nested)
   one=nested{k};
   if isnumeric(x.(one)) || islogical(x.(one))
    [same,error]=compare_arrays(x.(one),y.(one));exact=exact && same;
    difference=max(difference,error);
   else
    assert(isequaln(x.(one),y.(one)),'Manual signal metadata changed.');
   end
  end
 else
  [same,difference]=compare_arrays(x,y);exact=exact && same;
 end
 differences.(name)=difference;maximum=max(maximum,difference);
end
parity=struct('exact',exact,'maximumAbsoluteDifference',maximum,'channelDifferences',differences);
end

function [same,difference]=compare_arrays(x,y)
assert(isequal(size(x),size(y)),'Manual history dimensions changed.');
same=isequaln(x,y);delta=abs(double(x(:))-double(y(:)));
if isempty(delta),difference=0;else,difference=max(delta);end
end

function summaries=summarize_modes(rows,modes,expectedCount)
summaries=struct([]);
baseline=rows([rows.modeIndex]==1);baselineWarm=median([baseline(2:end).runtimeIncludingValidation_s]);
baselineNativeWarm=median([baseline(2:end).nativeSimulationAndMetrics_s]);
for j=1:numel(modes)
 cohort=rows([rows.modeIndex]==j);runtimes=[cohort.runtimeIncludingValidation_s];
 if numel(runtimes)>1,warm=median(runtimes(2:end));else,warm=NaN;end
 nativeTimes=[cohort.nativeSimulationAndMetrics_s];
 if numel(nativeTimes)>1,nativeWarm=median(nativeTimes(2:end));else,nativeWarm=NaN;end
 one=struct('mode',modes(j).name,'attempts',numel(cohort), ...
  'nativeReturns',sum([cohort.nativeReturned]),'exactPairedCases',sum([cohort.exactManualHistoryParity]), ...
  'allExpectedCasesCompleted',numel(cohort)==expectedCount && all([cohort.completed]), ...
  'allManualHistoriesExact',numel(cohort)==expectedCount && all([cohort.exactManualHistoryParity]), ...
  'firstCaseRuntime_s',runtimes(1),'medianWarmRuntime_s',warm, ...
  'firstCaseNativeSimulationAndMetrics_s',nativeTimes(1), ...
  'medianWarmNativeSimulationAndMetrics_s',nativeWarm, ...
  'nativeWarmSpeedupVersusOrdinaryNative',baselineNativeWarm/nativeWarm, ...
  'warmSpeedupVersusOrdinaryNative',baselineWarm/warm, ...
  'maximumManualHistoryDifference',max([cohort.maximumAbsoluteDifference]), ...
  'maximumSDIBytesAfterRun',max([cohort.SDIBytesAfterRun]));
 summaries=append(summaries,one);
end
end

function bytes=folder_bytes(folder)
files=dir(fullfile(folder,'**','*'));bytes=sum([files(~[files.isdir]).bytes]);
end

function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end

function s=defaults(s,d)
names=fieldnames(d);for j=1:numel(names),if ~isfield(s,names{j}),s.(names{j})=d.(names{j});end,end
end

function result=sha256(file)
assert(isfile(file));single=char(39);
escaped=strrep(file,single,[single,char(34),single,char(34),single]);
[status,output]=system(['sha256sum -- ',single,escaped,single]);assert(status==0);
result=regexp(output,'[0-9a-f]{64}','match','once');assert(~isempty(result));
end

function finish(mdl,previousStorage,previousRecord,ownedStorage)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
if strcmp(Simulink.sdi.getStorageLocation,ownedStorage),Simulink.sdi.clear;end
Simulink.sdi.setStorageLocation(previousStorage);Simulink.sdi.setRecordData(previousRecord);
end
