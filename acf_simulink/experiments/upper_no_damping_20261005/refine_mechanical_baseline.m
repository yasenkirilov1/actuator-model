function report=refine_mechanical_baseline(buildFolder,configsFile,recordingMat,outputFolder)
% Closed-loop native deterministic-controller refinement; sensor clocks fixed.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
addpath(fullfile(buildFolder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_accel_lead;
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'),'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_refine_replay_sdi'),'w');fprintf(fid,'Owned final recorded-action refinement.\n');fclose(fid);
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
modelFile=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),modelFile);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
recording=load(recordingMat);assert(all(recording.inputs(:,3)>.5),'This is deterministic-baseline closed-loop refinement only.');bank=load(configsFile,'configs');p=bank.configs(double(recording.scenario)+1).p;
if isfield(recording,'seed')&&recording.seed>=0,p.sensor.seed=double(recording.seed);end
if isfield(recording,'lowerTravelLimit_m')&&isfinite(recording.lowerTravelLimit_m),p.mechanics.travelMin=recording.lowerTravelLimit_m;end
p.sim.stopTime=recording.time(end);original=acf_prepare(p);
assert(abs(original.sim.step-1/300000)<1e-15);p.sim.step=1/600000;p=acf_prepare(p);
assert(isequaln(p.adcStudy,original.adcStudy),'Keep all noise/acquisition parameters and clocks fixed.');
assert(p.sensor.sampleTime==original.sensor.sampleTime&&p.force.sampleTime==original.force.sampleTime);
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
set_param(mdl,'StopTime',num2str(recording.time(end),17),'FixedStep',num2str(p.sim.step,17),'Solver','ode4', ...
 'SimulationMode','normal','SignalLogging','off','SaveFormat','Dataset','SaveOutput','on','SaveTime','on', ...
 'ReturnWorkspaceOutputs','on','LimitDataPoints','off');
dataset=Simulink.SimulationData.Dataset;
for j=1:3
 ts=timeseries(recording.inputs(:,j),recording.time(:));ts=setinterpmethod(ts,'zoh');dataset=dataset.addElement(ts);
end
in=Simulink.SimulationInput(mdl);in=in.setExternalInput(dataset);timer=tic;out=sim(in);wall=toc(timer);
nativeTime=out.yout.get(2).Values.Time;truth=rows(out.yout.get(2).Values,27);
observationTime=out.yout.get(1).Values.Time;observation=rows(out.yout.get(1).Values,18);
save(fullfile(outputFolder,'native_recording.mat'),'nativeTime','truth','observationTime','observation','-v7');
common=1:2:numel(nativeTime);sameCount=numel(common)==numel(recording.time);
normalError=truth(:,1)-p.force.reference;coarseError=recording.truth(:,1)-p.force.reference;
[peak,peakIndex]=max(abs(normalError));[coarsePeak,coarsePeakIndex]=max(abs(coarseError));
report=struct('status','deterministic_closed_loop_refinement_checked','closedLoopNeuralRefinement',false,'closedLoopDeterministicRefinement',true, ...
 'contract','Native deterministic PI/PID/feedforward and robot gate are recomputed on refined states at unchanged10kHz/100Hzclocks. The same external baseline-mode input remains1. No neural actor. ODE4physical/analogstep halves; fixed300kHznoise,30kHzADC,10kHzforcecontrol and100Hzrobotclockspreserved. Fullfinehistoryinclstartup/end scored.', ...
 'scenario',recording.scenario,'seed',recording.seed,'modelSha256',sha256(modelFile),'configsSha256',sha256(configsFile), ...
 'recordingMatSha256',sha256(recordingMat),'sourceRecording',recording.sourceRecording,'sourceRecordingSha256',recording.sourceRecordingSha256, ...
 'nativeWall_s',wall,'baseStep_s',original.sim.step,'refinedStep_s',p.sim.step, ...
 'rawNoiseClock_s',p.adcStudy.fineClock_s,'rawAdcSampleTime_s',p.adcStudy.outputSampleTime_s, ...
 'controllerSampleTime_s',p.force.sampleTime,'adcPreparedFieldsPreserved',true, ...
 'refinedNativeSamples',numel(nativeTime),'coarseNativeSamples',numel(recording.time),'commonSampleCountMatches',sameCount, ...
 'fineNormalPeak_N',peak,'coarseNormalPeak_N',coarsePeak,'peakDifference_N',peak-coarsePeak, ...
 'finePeakTime_s',nativeTime(peakIndex),'coarsePeakTime_s',recording.time(coarsePeakIndex), ...
 'fineNormalRms_N',sqrt(mean(normalError.^2)),'coarseNormalRms_N',sqrt(mean(coarseError.^2)), ...
 'fineVoltageClipSamples',sum(truth(:,24)>.5),'fineCurrentLimitSamples',sum(truth(:,23)>.5), ...
 'finePeakCurrent_A',max(abs(truth(:,15))),'finePeakVoltage_V',max(abs(truth(:,16))), ...
 'nativeContactLost',any(truth(:,19)<.5),'nativeTravelStop',any(truth(:,22)>.5), ...
 'algebraicLoopCount',numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));
if sameCount
 report.commonMaximumTimeDifference_s=max(abs(nativeTime(common)-recording.time(:)));
 difference=truth(common,:)-recording.truth;
 report.commonTruthPerChannelMaximumDifference=max(abs(difference),[],1);
 report.commonNormalMaximumDifference_N=report.commonTruthPerChannelMaximumDifference(1);
 report.commonMaximumTruthDifference=max(report.commonTruthPerChannelMaximumDifference);
else,report.commonMaximumTimeDifference_s=Inf;report.commonNormalMaximumDifference_N=Inf;report.commonMaximumTruthDifference=Inf;end
report.physicalForcePass=peak<=.5&&~report.nativeContactLost&&~report.nativeTravelStop;
report.executedCorrectly=sameCount&&report.commonMaximumTimeDifference_s<1e-12&&report.algebraicLoopCount==0;
write_json(fullfile(outputFolder,'deterministic_refinement.json'),report);
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'refine_mechanical_baseline.m'));
fprintf('RL_RECORDED_ACTION_REFINEMENT %s\n',jsonencode(report));assert(report.executedCorrectly);
end
function d=rows(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_refine_replay_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
