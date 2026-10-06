function manifest=prepare_actuator_probe(folder)
% Development-only paired voltage/current-response probe, no policy retuning.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
parent=fullfile(root,'acf_simulink','validation','rl_budget3h_20261005');oldFolder=fullfile(parent,'scenario_bank');
if nargin<1,folder=fullfile(parent,'native_actuator_probe_bank');end
assert(~isfolder(folder),'Preserve prior actuator studies.');mkdir(folder);
old=load(fullfile(oldFolder,'configs.mat'),'configs');base=old.configs(62).p;prior=jsondecode(fileread(fullfile(oldFolder,'manifest.json')));template=prior.cases(62);
copyfile(fullfile(oldFolder,'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
configs=struct([]);metadata=struct([]);
for tau=[100e-6 50e-6]
 for voltage=[32 36 48]
  p=base;p.coil.voltage=voltage;p.coil.currentResponse=tau;p=acf_prepare(p);
  k=numel(configs);identity=sprintf('development_V%02d_current%03dus',voltage,round(tau*1e6));
  c=struct('p',p,'identity',identity,'split','development');if isempty(configs),configs=c;else,configs(end+1)=c;end
  r=template;r.scenario=k;r.identity=identity;r.split='development';r.curriculum='actuator_limit_probe';r.usableVoltage_V=voltage;
  if isempty(metadata),metadata=r;else,metadata(end+1)=r;end
 end
end
save(fullfile(folder,'configs.mat'),'configs','-v7');
manifest=struct('status','frozen_development_not_executed','cases',metadata,'sourceModelSha256',prior.sourceModelSha256, ...
 'configsSha256',sha256(fullfile(folder,'configs.mat')),'sourceConfigsSha256',sha256(fullfile(oldFolder,'configs.mat')), ...
 'sourceScenario',61,'sourceIdentity',old.configs(62).identity,'sourceSeed',base.sensor.seed, ...
 'probeContract','Same selection61profile/seed/physical/sensor/calibrationinputs. V32/36/48with100usfixedcurrenttarget. Separate V32/36/48with50uscurrenttargetandmatchingidentifiedmotionobserverpreparation. No qualification/heldout/checkpointselectionclaims. Exactlytwo p.coilfields editedbefore nativeacf_prepare.', ...
 'learnedCheckpointSha256','48618b8d21c4f5920d870a05f5eb198c958632f51ffab6f0abfd6a8255ef0ad4');
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_actuator_probe.m'));
fprintf('RL_ACTUATOR_PROBE_BANK_FROZEN 6 cases\n');
end
function s=sha256(file)
fid=fopen(file,'r');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
s=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
