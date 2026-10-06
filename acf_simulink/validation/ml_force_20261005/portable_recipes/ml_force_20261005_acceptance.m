function report=ml_force_20261005_acceptance(root,study)
% Private native builder, save/reopen and numerical-inference acceptance.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,study=fullfile(root,'acf_simulink','validation','ml_force_20261005');end
folder=fullfile(study,'acceptance');assert(~isfolder(folder));mkdir(folder);
for name={'cache','code'},mkdir(fullfile(folder,name{1}));end
modelFile=fullfile(folder,'acf_force_control_adc_trial.slx');
copyfile(fullfile(study,'source','acf_force_control_adc_trial.slx'),modelFile);
addpath(fullfile(study,'source','helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'),'CodeGenFolder',fullfile(folder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_ml_force_acceptance'),'w');fprintf(fid,'Owned acceptance SDI.\n');fclose(fid);
cleanup=onCleanup(@()finish('acf_force_control_adc_trial',oldStorage,oldRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
selected=jsondecode(fileread(fullfile(study,'selected_gain.json')));
[mdl,audit]=ml_force_20261005_build(modelFile,fullfile(study,'trained_weights.mat'),selected.selectedGain);
cases=load(fullfile(study,'test','configs.mat'),'configs');p=acf_prepare(cases.configs(2).p);original=p;
p.sim.stopTime=.1;assignin(get_param(mdl,'ModelWorkspace'),'p',p);
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime','.1','SimulationMode','accelerator');
loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);assert(isempty(loops));
out=sim(mdl);a=out.get('normal_contact_log');f=out.get('ml_features_log');
firstTime=a.Time;firstData=a.Data;firstFeatureTime=f.Time;firstFeatureData=f.Data;
clear out a f;Simulink.sdi.clear;save_system(mdl,modelFile);close_system(mdl,0);
load_system(modelFile);out=sim(mdl);a=out.get('normal_contact_log');f=out.get('ml_features_log');
assert(isequal(firstTime,a.Time)&&isequal(firstData,a.Data));
assert(isequal(firstFeatureTime,f.Time)&&isequal(firstFeatureData,f.Data));
report=struct('status','passed_private_builder_save_reopen','builder',audit,'nativeNormalSamplesPerRecording',numel(a.Time), ...
 'executedShortNativeRecordings',2,'durationEach_s',.1,'algebraicLoops',numel(loops), ...
 'exactNormalHistoryAfterSaveReopen',true,'exactFeatureAndActionHistoryAfterSaveReopen',true, ...
 'sourceModelSha256','b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5', ...
 'scope','Software acceptance only, separate from qualification. No canonical model promotion.');
assignin(get_param(mdl,'ModelWorkspace'),'p',original);set_param(mdl,'StopTime',num2str(original.sim.stopTime,17));save_system(mdl,modelFile);
fid=fopen(fullfile(folder,'results.json'),'w');assert(fid~=-1);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'ml_force_20261005_acceptance.m'));
end
function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_ml_force_acceptance')),try,rmdir(sdi,'s');catch,end,end
end
