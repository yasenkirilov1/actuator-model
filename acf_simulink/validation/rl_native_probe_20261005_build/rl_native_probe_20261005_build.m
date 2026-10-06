function report=rl_native_probe_20261005_build(root)
% Private code-generation feasibility probe. No learning or shared mutation.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
folder=fullfile(root,'acf_simulink','validation','rl_native_probe_20261005_build');
assert(~isfolder(folder),'Preserve prior probe.');mkdir(folder);mkdir(fullfile(folder,'cache'));mkdir(fullfile(folder,'code'));
source=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002');
modelFile=fullfile(folder,'acf_force_control_adc_trial.slx');copyfile(fullfile(source,'acf_force_control_adc_trial.slx'),modelFile);
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'),'CodeGenFolder',fullfile(folder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_rl_probe_sdi'),'w');fprintf(fid,'Owned build probe SDI.\n');fclose(fid);
mdl='acf_force_control_adc_trial';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(false);
loaded=load(fullfile(root,'acf_simulink','validation','budget_adc_hot_coil_20261002_inputs', ...
 'ADXL1002_8000_M08_30k_current100_hot32V_configs.mat'),'configs');p=acf_prepare(loaded.configs(1).p);
p.sim.stopTime=.1;assignin(get_param(mdl,'ModelWorkspace'),'p',p);
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime','.1','SimulationMode','normal');
blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace');
for k=1:numel(blocks),set_param(blocks{k},'Commented','on');end
scope=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','Scope');for k=1:numel(scope),set_param(scope{k},'Commented','on');end
set_param(mdl,'SignalLogging','off','SaveTime','off','SaveOutput','off','MatFileLogging','off');
% Root scoring port exposes native physical truth ONLY to the environment.
add_block('simulink/Signal Routing/Mux',[mdl '/RL native scoring mux'],'Inputs','3','Position',[3600 50 3605 180]);
tags={'normal_contact_N','plant_log','actual_contact_slope'};
for k=1:3
 add_block('simulink/Signal Routing/From',[mdl '/RL scoring source ' num2str(k)],'GotoTag',tags{k},'Position',[3200 45+50*k 3510 70+50*k]);
 add_line(mdl,['RL scoring source ' num2str(k) '/1'],['RL native scoring mux/' num2str(k)]);
end
add_block('simulink/Ports & Subsystems/Out1',[mdl '/rl_scoring'],'Position',[3740 95 3800 125]);add_line(mdl,'RL native scoring mux/1','rl_scoring/1');
report=struct('date','2026-10-05','status','running_private_build_probe','sourceModelSha256',sha256(fullfile(source,[mdl '.slx'])), ...
 'matlabVersion',version,'selectedReferenceParameters','hot32V calibrated source case1, ADC30k/matched8k,100us current target','attempts',struct([]));
% Try ordinary Coder first. ERT is a fallback for a reusable class interface.
for target={'grt.tlc','ert.tlc'}
 timer=tic;attempt=struct('target',target{1},'success',false,'error','','elapsed_s',NaN);
 try
  set_param(mdl,'SystemTargetFile',target{1},'TargetLang','C++','DefaultParameterBehavior','Tunable','GenerateReport','off');
  if strcmp(target{1},'ert.tlc')
   set_param(mdl,'SupportContinuousTime','on','CodeInterfacePackaging','C++ class','GenerateSampleERTMain','on');
  end
  save_system(mdl,modelFile);slbuild(mdl);attempt.success=true;
 catch failure
  attempt.error=getReport(failure,'extended','hyperlinks','off');
 end
 attempt.elapsed_s=toc(timer);if isempty(report.attempts),report.attempts=attempt;else,report.attempts(end+1)=attempt;end
 write_json(fullfile(folder,'results.json'),report);
 fprintf('RL_BUILD_ATTEMPT %s success%d elapsed%.2fs\n',target{1},attempt.success,attempt.elapsed_s);
 if attempt.success,break;end
end
if any([report.attempts.success]),report.status='private_native_build_succeeded';else,report.status='private_native_build_rejected';end
write_json(fullfile(folder,'results.json'),report);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'rl_native_probe_20261005_build.m'));
end
function write_json(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true)); %#ok<NASGU>
end
function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_rl_probe_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');d.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
