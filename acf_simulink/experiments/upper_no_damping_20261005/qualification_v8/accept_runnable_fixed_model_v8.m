function report=accept_runnable_fixed_model_v8(nativeFolder,runnableFolder,outputFolder)
% Independent native reference, direct Run, save/reopen, stops and timing.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'), ...
 'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_runnable_acceptance'),'w');fprintf(fid,'Owned runnable acceptance.\n');fclose(fid);
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
mdl='acf_force_control_fixed_upper_20261005';source='acf_rl_native';
cleanup=onCleanup(@()finish(mdl,source,oldStorage,oldRecord,sdi)); %#ok<NASGU>
modelFile=fullfile(runnableFolder,[mdl '.slx']);addpath(fullfile(runnableFolder,'helpers'),'-begin');load_system(modelFile);
mw=get_param(mdl,'ModelWorkspace');default=getVariable(mw,'pDefault');assert(isequaln(getVariable(mw,'p'),default));
assert(default.mechanics.mA==1.5&&default.mechanics.mS==1.4&&default.force.reference==20);
assert(strcmp(get_param(mdl,'LoadExternalInput'),'off'));
assert(isempty(find_system(mdl,'SearchDepth',1,'BlockType','Inport')));close_system(mdl,0);

% A copied original native graph, explicitly baseline1, uses the same full p.
addpath(fullfile(nativeFolder,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal;
sourceFile=fullfile(outputFolder,[source '.slx']);copyfile(fullfile(nativeFolder,[source '.slx']),sourceFile);load_system(sourceFile);
ref=acf_prepare(default);assert(isequaln(ref.d,default.d));assignin(get_param(source,'ModelWorkspace'),'p',ref);
set_param(source,'StopTime','2','FixedStep',num2str(ref.sim.step,17),'SimulationMode','normal', ...
 'SaveOutput','on','SaveFormat','Dataset','SignalLogging','off','ReturnWorkspaceOutputs','on','LimitDataPoints','off');
dataset=Simulink.SimulationData.Dataset;
for k=1:3,ts=timeseries([double(k==3);double(k==3)],[0;2]);ts=setinterpmethod(ts,'zoh');dataset=dataset.addElement(ts);end
in=Simulink.SimulationInput(source);in=in.setExternalInput(dataset);timer=tic;out=sim(in);referenceWall=toc(timer);
referenceTime=out.yout.get(2).Values.Time;referenceTruth=sample_rows(out.yout.get(2).Values,27);
save(fullfile(outputFolder,'original_native_baseline_reference.mat'),'referenceTime','referenceTruth','ref','-v7');
clear out;Simulink.sdi.clear;close_system(source,0);

addpath(fullfile(runnableFolder,'helpers'),'-begin');load_system(modelFile);mw=get_param(mdl,'ModelWorkspace');
timer=tic;out=sim(mdl);normalWall=toc(timer);normalTime=out.yout.get(2).Values.Time;normalTruth=sample_rows(out.yout.get(2).Values,27);
assert(isequal(normalTime,referenceTime));difference=max(abs(normalTruth-referenceTruth),[],1);
assert(max(difference)<1e-8,'Standalone graph changed selected native baseline physics or control.');
normalPeak=max(abs(normalTruth(:,1)-default.force.reference));
save(fullfile(outputFolder,'standalone_default_normal.mat'),'normalTime','normalTruth','default','-v7');
clear out;Simulink.sdi.clear;
report=struct('status','runnable_native_acceptance_running','qualificationEpisodes',0, ...
 'canonicalChanged',false,'externalInputNeeded',false,'deterministicBaselineHardSelected',true, ...
 'sourceModelSha256',sha256(fullfile(nativeFolder,[source '.slx'])), ...
 'normalReferenceWall_s',referenceWall,'standaloneNormalWall_s',normalWall, ...
 'nativeDefaultSamples',numel(normalTime),'defaultNormalForcePeakError_N',normalPeak, ...
 'referencePerChannelMaximumDifference',difference,'referenceMaximumTruthDifference',max(difference), ...
 'defaultNormalReferencePassed',true,'directRunAfterParameterEditsPassed',false, ...
 'saveReopenPassed',false,'contactStopPassed',false,'travelStopPassed',false,'acceleratorPassed',false);
write_json(fullfile(outputFolder,'acceptance_progress.json'),report);

% No preparation call: model InitFcn applies an edited tool/body/force/duration.
p=default;p.mechanics.mA=.4;p.upperObserver.identifiedMassA_kg=.4;p.mechanics.mS=.1;p.force.reference=12;
p.surface.kind='flat';p.sim.stopTime=.03;assignin(mw,'p',p);out=sim(mdl);
firstTime=out.yout.get(2).Values.Time;firstTruth=sample_rows(out.yout.get(2).Values,27);applied=getVariable(mw,'p');
assert(abs(firstTime(end)-.03)<1e-12&&applied.sensor.distalMass_kg==.1&&applied.sensor.distalMass_model_kg==.1);
assert(applied.mechanics.mA==.4&&applied.upperObserver.identifiedMassA_kg==.4&&applied.force.reference==12);
report.directRunAfterParameterEditsPassed=true;
report.editedRun=struct('actualBody_kg',.4,'identifiedBody_kg',.4,'actualTool_kg',.1,'identifiedTool_kg',.1, ...
 'targetForce_N',12,'requestedDuration_s',.03,'actualEndTime_s',firstTime(end),'normalForcePeakError_N',max(abs(firstTruth(:,1)-12)));
clear out;Simulink.sdi.clear;save_system(mdl,modelFile);close_system(mdl,0);load_system(modelFile);mw=get_param(mdl,'ModelWorkspace');
out=sim(mdl);secondTime=out.yout.get(2).Values.Time;secondTruth=sample_rows(out.yout.get(2).Values,27);
assert(isequal(firstTime,secondTime)&&isequal(firstTruth,secondTruth));report.saveReopenPassed=true;
save(fullfile(outputFolder,'edited_save_reopen.mat'),'firstTime','firstTruth','secondTime','secondTruth','applied','-v7');
clear out;Simulink.sdi.clear;write_json(fullfile(outputFolder,'acceptance_progress.json'),report);

% Deliberate out-of-envelope physical probes test the existing failure guards.
p=default;p.mechanics.mS=2;p.force.reference=50;p.mechanics.angle=pi/2;p.contact.mu=0;
p.surface.kind='sharp_bump';p.surface.startTime=.01;p.surface.height=.005;p.surface.width=.002;p.sim.stopTime=.1;
assignin(mw,'p',p);out=sim(mdl);contactTime=out.yout.get(2).Values.Time;contactTruth=sample_rows(out.yout.get(2).Values,27);
save(fullfile(outputFolder,'contact_stop.mat'),'contactTime','contactTruth','p','-v7');
report.contactStop=struct('endTime_s',contactTime(end),'requestedEnd_s',.1,'contactLost',any(contactTruth(:,19)<.5), ...
 'travelStop',any(contactTruth(:,22)>.5),'normalPeakError_N',max(abs(contactTruth(:,1)-50)));
report.contactStopPassed=report.contactStop.contactLost&&contactTime(end)<.1;
write_json(fullfile(outputFolder,'acceptance_progress.json'),report);assert(report.contactStopPassed);
clear out;Simulink.sdi.clear;
p=default;p.mechanics.mS=2;p.force.reference=50;p.mechanics.angle=pi/2;
p.surface.startTime=.01;p.sim.stopTime=.2;p.mechanics.travelMin=p.mechanics.midTravel-40e-6;
assignin(mw,'p',p);out=sim(mdl);travelTime=out.yout.get(2).Values.Time;travelTruth=sample_rows(out.yout.get(2).Values,27);
save(fullfile(outputFolder,'travel_stop.mat'),'travelTime','travelTruth','p','-v7');
report.travelStop=struct('endTime_s',travelTime(end),'requestedEnd_s',.2,'travelStop',any(travelTruth(:,22)>.5), ...
 'contactLost',any(travelTruth(:,19)<.5),'normalPeakError_N',max(abs(travelTruth(:,1)-50)));
report.travelStopPassed=report.travelStop.travelStop&&travelTime(end)<.2;
write_json(fullfile(outputFolder,'acceptance_progress.json'),report);assert(report.travelStopPassed);
clear out;Simulink.sdi.clear;

assignin(mw,'p',default);set_param(mdl,'SimulationMode','accelerator');
wall=zeros(1,2);maxDiff=zeros(1,2);
for j=1:2
 timer=tic;out=sim(mdl);wall(j)=toc(timer);acceleratorTime=out.yout.get(2).Values.Time;
 acceleratorTruth=sample_rows(out.yout.get(2).Values,27);assert(isequal(acceleratorTime,normalTime));
 maxDiff(j)=max(abs(acceleratorTruth-normalTruth),[],'all');assert(maxDiff(j)<1e-8);
 clear out;Simulink.sdi.clear;
end
report.acceleratorPassed=true;report.acceleratorColdWall_s=wall(1);report.acceleratorWarmWall_s=wall(2);
report.acceleratorMaximumTruthDifference=maxDiff;report.hostSpecificWarmSpeedup=normalWall/wall(2);
report.algebraicLoopCount=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(report.algebraicLoopCount==0);
assignin(mw,'p',default);set_param(mdl,'SimulationMode','accelerator');acf_fixed_apply(mdl);save_system(mdl,modelFile);
report.modelFile=modelFile;report.modelSha256=sha256(modelFile);report.status='private_versioned_native_runnable_model_accepted';
report.developmentOnly=true;report.fullRequirementQualified=false;
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'accept_runnable_fixed_model_v8.m'));
write_json(fullfile(outputFolder,'acceptance.json'),report);
% Ordinary Simulink-rendered diagrams provide a layout inspection artifact.
print(['-s' mdl],'-dpng',fullfile(outputFolder,'diagram_root.png'));
print(['-s' mdl '/Force controller/Coil PI and PID'],'-dpng',fullfile(outputFolder,'diagram_force_controller.png'));
print(['-s' mdl '/Contact force measurement/Head motion estimator'],'-dpng',fullfile(outputFolder,'diagram_upper_observer.png'));
fprintf('RUNNABLE_MODEL_ACCEPTED %s\n',jsonencode(report));
end
function d=sample_rows(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function finish(mdl,source,storage,record,sdi)
try,close_system(mdl,0);catch,end;try,close_system(source,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_runnable_acceptance')),try,rmdir(sdi,'s');catch,end,end
end
function write_json(file,value),fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
