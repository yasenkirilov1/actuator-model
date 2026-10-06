function manifest=prepare_bank_current50(folder)
% Separate driver-response counterfactual; retain all scenario identities.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
parent=fullfile(root,'acf_simulink','validation','rl_budget3h_20261005');oldFolder=fullfile(parent,'scenario_bank_sustained');
if nargin<1,folder=fullfile(parent,'scenario_bank_current50');end
assert(~isfolder(folder),'Preserve every prior counterfactual bank.');mkdir(folder);
old=load(fullfile(oldFolder,'configs.mat'),'configs');configs=old.configs;assert(numel(configs)==144);
prior=jsondecode(fileread(fullfile(oldFolder,'manifest.json')));metadata=prior.cases;
copyfile(fullfile(oldFolder,'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
for k=1:numel(configs)
 p=old.configs(k).p;assert(abs(p.coil.currentResponse-100e-6)<1e-15&&p.coil.voltage==32);
 priorTau=p.coil.currentResponse;p.coil.currentResponse=50e-6;before=p;before.coil.currentResponse=priorTau;assert(isequaln(before,old.configs(k).p));
 p=acf_prepare(p);configs(k).p=p;
 assert(p.sim.step==1/300000&&p.force.sampleTime==1e-4&&p.coil.voltage==32);
 metadata(k).currentResponse_s=p.coil.currentResponse;
 metadata(k).identifiedCurrentResponse_s=p.coil.currentResponse;
end
assert(isequal({configs.identity},{old.configs.identity})&&isequal({configs.split},{old.configs.split}));
save(fullfile(folder,'configs.mat'),'configs','-v7');
manifest=prior;manifest.status='frozen_driver_response_counterfactual_not_executed';manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(oldFolder,'configs.mat'));manifest.parentManifestSha256=sha256(fullfile(oldFolder,'manifest.json'));
manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));manifest.originalScenarioIdentitiesPreservedExactly=true;
manifest.counterfactualContract='Every144case retains source physical/sensor/calibration/seed/profile/robot program and ID. Change only p.coil.currentResponse100us to50us before nativeacf_prepare, which also retunes identified issued-command/current-response observer coefficients. Usableoutput32V unchanged. This is a separate driver-response experiment; source100us training/qualificationbanks unchanged. Execute only28 already-used DEVELOPMENTselection IDs48:63,120:127,140:143. IDs64:95 copied for scenarioidentitybut NEVERexecuted in this study.';
manifest.fixedCheckpointSha256='48618b8d21c4f5920d870a05f5eb198c958632f51ffab6f0abfd6a8255ef0ad4';
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_bank_current50.m'));
fprintf('RL_CURRENT50_BANK_FROZEN 144scenarioidentities,28developmentselectiononly\n');
end
function s=sha256(file)
fid=fopen(file,'r');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
s=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
