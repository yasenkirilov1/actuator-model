function manifest=prepare_corrected_current_bank(folder)
% Preserve original six profiles/two A masses, repair observer wiring only.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
source=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005','banks','upper_20k_lower_cell_undamped');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
old=load(fullfile(source,'configs.mat'),'configs');configs=old.configs;prior=jsondecode(fileread(fullfile(source,'manifest.json')));
metadata=prior.cases;
for j=1:numel(configs)
 p=configs(j).p;assert(~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0);
 assert(p.mechanics.k1==64e6&&p.mechanics.c1==0&&p.upperObserver.enabled);
 p.upperObserver.contract='Corrected explicit selected post-limit command ports; strictly proper identified current lag; same ADC frame; no-lead physical guide-friction estimate from corrected measured-frame gap velocity. No actual current/A/actuator-force/surface truth. Identified and actual changed mechanics equal.';
 p.mechanicalStudy.controlDevelopmentLane='corrected_issued_current_observer_v5';
 p.mechanicalStudy.upperSpringLengthAndTravelUnconstrained=true;
 configs(j).p=acf_prepare(p);
 identity=['corrected_current_observer_v5_' configs(j).identity];configs(j).identity=identity;metadata(j).identity=identity;
end
assert(numel(configs)==132);
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=prior;
manifest.status='frozen_corrected_issued_current_development_not_executed';manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.controlDevelopmentLane='corrected_issued_current_observer_v5';
manifest.developmentContract=[prior.developmentContract ' Mandatory upper20k/c0/unlimited, lower externally rigid finite64MN/m sensingcell/c0. Prior grounded issued-current observer implementations are rejected evidence. This separately frozen graph repairs the current source using explicit subsystem inports from actual selected drive request, preserving six original PI/notch profiles,two A masses,11cases,seeds,sensors,current lag/friction/matched frame. Short live source harness and emittedC source evidence required before seal. No fresh-test tuning or neural training.'];
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_corrected_current_bank.m'));
fprintf('CORRECTED_CURRENT_V5_BANK %d cases\n',numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
