function report=replay_mechanical_native(buildFolder,configsFile,recordingMat,outputFolder)
% Replay exactly recorded full external actions in an independent native Run.
% Works for learned, untrained, baseline and physical-failure recordings.
assert(~usejava('desktop'));maxNumCompThreads(2);
assert(~isfolder(outputFolder),'Preserve prior native replay.');mkdir(outputFolder);
addpath(fullfile(buildFolder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'),'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_rl_replay_sdi'),'w');fprintf(fid,'Owned independent RL replay.\n');fclose(fid);
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
modelFile=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),modelFile);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
recording=load(recordingMat);bank=load(configsFile,'configs');p=bank.configs(recording.scenario+1).p;
if isfield(recording,'seed')&&recording.seed>=0,p.sensor.seed=double(recording.seed);end
if isfield(recording,'lowerTravelLimit_m')&&isfinite(recording.lowerTravelLimit_m),p.mechanics.travelMin=recording.lowerTravelLimit_m;end
p.sim.stopTime=recording.time(end);p=acf_prepare(p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
set_param(mdl,'StopTime',num2str(recording.time(end),17),'FixedStep',num2str(p.sim.step,17), ...
 'SimulationMode','normal','SignalLogging','off','SaveFormat','Dataset','SaveOutput','on', ...
 'SaveTime','on','ReturnWorkspaceOutputs','on','LimitDataPoints','off');
% A ZOH preserves command requests exactly; no interpolated future action.
dataset=Simulink.SimulationData.Dataset;
for j=1:3
 ts=timeseries(recording.inputs(:,j),recording.time(:));ts=setinterpmethod(ts,'zoh');dataset=dataset.addElement(ts);
end
in=Simulink.SimulationInput(mdl);in=in.setExternalInput(dataset);timer=tic;out=sim(in);wall=toc(timer);
nativeTime=out.yout.get(2).Values.Time;truth=rows(out.yout.get(2).Values,27);
observationTime=out.yout.get(1).Values.Time;observation=rows(out.yout.get(1).Values,18);
comparatorTime=out.yout.get(3).Values.Time;comparator=rows(out.yout.get(3).Values,3);
save(fullfile(outputFolder,'native_recording.mat'),'nativeTime','truth','observationTime','observation','comparatorTime','comparator','-v7');
report=struct('status','native_replay_checked','scenario',recording.scenario,'seed',recording.seed, ...
 'modelSha256',sha256(modelFile),'recordingMatSha256',sha256(recordingMat),'configsSha256',sha256(configsFile), ...
 'nativeWall_s',wall,'nativeSamples',numel(nativeTime),'cSamples',numel(recording.time), ...
 'sameSampleCount',isequal(size(truth),size(recording.truth)), ...
 'nativeFirstTime_s',nativeTime(1),'nativeLastTime_s',nativeTime(end), ...
 'nativeNormalPeak_N',max(abs(truth(:,1)-p.force.reference)), ...
 'cNormalPeak_N',max(abs(recording.truth(:,1)-p.force.reference)), ...
 'nativeContactLost',any(truth(:,19)<.5),'nativeTravelStop',any(truth(:,22)>.5), ...
 'sourceRecording',recording.sourceRecording,'sourceRecordingSha256',recording.sourceRecordingSha256, ...
 'algebraicLoopCount',numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));
if report.sameSampleCount
 report.maximumTimeDifference_s=max(abs(nativeTime(:)-recording.time(:)));
 report.truthPerChannelMaximumDifference=max(abs(truth-recording.truth),[],1);
 report.allTruthMaximumDifference=max(report.truthPerChannelMaximumDifference);
else,report.maximumTimeDifference_s=Inf;report.allTruthMaximumDifference=Inf;end
report.passed=report.sameSampleCount&&report.maximumTimeDifference_s<1e-12&&report.allTruthMaximumDifference<1e-8&&report.algebraicLoopCount==0;
if report.passed,report.status='independent_native_replay_passed';else,report.status='independent_native_replay_failed';end
fid=fopen(fullfile(outputFolder,'native_replay.json'),'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'replay_mechanical_native.m'));
fprintf('RL_INDEPENDENT_NATIVE_REPLAY %s\n',jsonencode(report));assert(report.passed);
end
function d=rows(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_rl_replay_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
