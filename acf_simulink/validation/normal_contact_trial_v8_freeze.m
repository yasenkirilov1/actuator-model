function manifest=normal_contact_trial_v8_freeze(config)
% Freeze a reviewed acquisition contract after full-history native bridges.
if nargin<1,config=struct;end
state=fullfile(getenv('HOME'),'.local','state','actuator-model');
if ~isfield(config,'testedFolder'),config.testedFolder=fullfile(state,'normal-contact-trial-inertial-v8-20260930');end
if ~isfield(config,'workFolder'),config.workFolder=fullfile(state,'normal-contact-trial-inertial-v8-contract-20260930');end
assert(~usejava('desktop'));build=build_normal_contact_trial_v8(struct('workFolder',config.workFolder));
work=config.workFolder;mdl='acf_force_control_normal_trial';
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
saved=load(fullfile(config.testedFolder,'traces','normal_contact_trial_inertial_v8_pilot_02.mat'),'r');reference=saved.r;
r=run_private(work,reference.parameters);row=compare(r,reference);
save(fullfile(work,'traces','normal_contact_trial_inertial_v8_contract_identity.mat'),'r','reference','row','-v7');
assert(row.maximumDifference<1e-10,'Separating sensor/predictor response or fine robot transport changed nominal native histories.');
fprintf('V8 CONTRACT IDENTITY maxdifference %.3g\n',row.maximumDifference);Simulink.sdi.clear;
heavy=r.parameters;
saved=load(fullfile(config.testedFolder,'traces','normal_contact_trial_inertial_v8_pilot_06.mat'),'r');reference=saved.r;
q=heavy;q.mechanics.mS=.1; % No manual update of either distal sensor field.
r=run_private(work,q);massRow=compare(r,reference);
save(fullfile(work,'traces','normal_contact_trial_inertial_v8_mass_only_identity.mat'),'r','reference','massRow','-v7');
assert(massRow.maximumDifference<1e-10&&r.parameters.sensor.distalMass_kg==.1&&r.parameters.sensor.distalMass_model_kg==.1);
fprintf('V8 MASS-ONLY IDENTITY maxdifference %.3g\n',massRow.maximumDifference);Simulink.sdi.clear;
% Actual sensor tolerance stays independent of the identified command frame.
q=heavy;q.acquisition.antiAliasCutoff_Hz=.95*q.acquisition.antiAliasCutoff_Hz;q.acquisition.pipelineDelay_s=30e-6;
addpath(fullfile(work,'helpers'),'-begin');clear acf_prepare;pMismatch=acf_prepare(q);
assert(isequal(pMismatch.d.acquisition.responses.frame.numerator,heavy.d.acquisition.responses.frame.numerator));
assert(pMismatch.d.acquisition.frameDelaySamples==2&&pMismatch.d.sensorDelaySamples==3);
assert(pMismatch.d.acquisition.noiseInputScale==heavy.d.acquisition.noiseInputScale);
q=heavy;q.robot.delay=.075;pDelay=acf_prepare(q);assert(pDelay.d.robotDelaySamples==750&&pDelay.robotPrediction.delaySamples==8);
q.robot.delay=.085;pDelay=acf_prepare(q);assert(pDelay.d.robotDelaySamples==850&&pDelay.robotPrediction.delaySamples==8);
p=heavy;p.sensor.seed=7;p.surface.kind='sharp_bump';p.sim.stopTime=4;p=acf_prepare(p);
load_system(fullfile(work,[mdl '.slx']));mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);
acf_select_actuator(mdl,p.actuator.kind);set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));save_system(mdl);close_system(mdl,0);
save(fullfile(work,'normal_contact_trial_candidate.mat'),'p','-v7');
freeze=fullfile(work,'frozen-inertial-v8');mkdir(freeze);mkdir(fullfile(freeze,'helpers'));
copyfile(fullfile(work,[mdl '.slx']),fullfile(freeze,[mdl '.slx']));
copyfile(fullfile(work,'helpers','*.m'),fullfile(freeze,'helpers'));
copyfile(fullfile(work,'normal_contact_trial_candidate.mat'),fullfile(freeze,'normal_contact_trial_candidate.mat'));
copyfile(fullfile(work,'recipe_snapshot.m'),fullfile(freeze,'recipe_snapshot.m'));
proof=jsondecode(fileread(fullfile(config.testedFolder,'v8_verification.json')));assert(all([proof.pilotRows.passed]));
manifest=struct('status','READY; finite native source and endpoint qualification', ...
    'frozenFolder',freeze,'modelSHA256',sha256(fullfile(freeze,[mdl '.slx'])), ...
    'candidateSHA256',sha256(fullfile(freeze,'normal_contact_trial_candidate.mat')), ...
    'source',build,'nativeIdentity',row,'massOnlyIdentity',massRow,'endpointProof',proof, ...
    'acquisition',p.acquisition,'acquisitionCoefficients',p.d.acquisition, ...
    'controller',p.inertial,'forceGate',p.robot.forceGate,'springs',p.springs, ...
    'actualRobotTransportPeriod_s',p.d.actualRobotTransportSampleTime_s, ...
    'logging','Physical35 retains Rz in col1; normal8, accelerometer6, motion2, predictor3, drag3, variation5, accelerationFF2 and lowerInverse4 all retain native histories.');
files=dir(fullfile(freeze,'helpers','*.m'));manifest.helperHashes=struct;
for j=1:numel(files),[~,name]=fileparts(files(j).name);manifest.helperHashes.(name)=sha256(fullfile(files(j).folder,files(j).name));end
path=fullfile(work,'freeze_manifest.json');write_json(path,manifest);copyfile(path,fullfile(freeze,'freeze_manifest.json'));
root=fileparts(fileparts(mfilename('fullpath')));write_json(fullfile(root,'validation','normal_contact_trial_inertial_v8_manifest.json'),manifest);
for path={fullfile(freeze,[mdl '.slx']),fullfile(freeze,'normal_contact_trial_candidate.mat'),fullfile(freeze,'helpers','*.m'),fullfile(freeze,'recipe_snapshot.m'),fullfile(freeze,'freeze_manifest.json')},fileattrib(path{1},'-w');end
fprintf('V8 FROZEN READY %s SHA %s\n',freeze,manifest.modelSHA256);
end
function r=run_private(folder,p)
mdl='acf_force_control_normal_trial';addpath(fullfile(folder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator;
load_system(fullfile(folder,[mdl '.slx']));set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error');r=acf_run(mdl,p,false);close_system(mdl,0);
end
function row=compare(a,b)
assert(isequal(a.time,b.time));row=struct('duration_s',a.time(end),'nativeSamples',numel(a.time),'physical35Difference',max(abs(a.data-b.data),[],'all'),'maximumDifference',max(abs(a.data-b.data),[],'all'));
for name={'normalContact','accelerometer','motionObserver','commandPrediction','forceVariation','measuredDrag','accelerationFeedforward','lowerInverse'}
    assert(isequal(a.(name{1}).time,b.(name{1}).time));d=max(abs(a.(name{1}).data-b.(name{1}).data),[],'all');row.([name{1} 'Difference'])=d;row.maximumDifference=max(row.maximumDifference,d);
end
aa=[a.robotGate.ready,a.robotGate.filteredError_N,a.robotGate.forceRate_N_s,a.robotGate.stableNow];bb=[b.robotGate.ready,b.robotGate.filteredError_N,b.robotGate.forceRate_N_s,b.robotGate.stableNow];
row.gateDifference=max(abs(aa-bb),[],'all');row.maximumDifference=max(row.maximumDifference,row.gateDifference);
row.peakNormalError_N=max(abs(a.normalContact.data(:,1)-a.parameters.force.reference));
end
function write_json(file,v)
fid=fopen(file,'w');assert(fid>0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(v,PrettyPrint=true));
end
function h=sha256(file)
escaped=['''' strrep(file,'''','''"''"''') ''''];[code,out]=system(['sha256sum -- ' escaped]);assert(code==0);h=strtok(out);
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
