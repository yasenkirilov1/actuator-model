function manifest=prepare_undamped_control_bank(folder)
% A second bounded deterministic development bank; never a fresh-test retune.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005','banks','upper_rigid_lower_undamped');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
original=load(fullfile(source,'configs.mat'),'configs');cases=original.configs(1:11);
% Reference Kp, Ki, Kd, inverse cutoff(0 disables), derivative time.
profiles=[.03,1,0,100,.001;.06,5,0,100,.001;.1,20,0,100,.001; ...
 .03,1,0,200,.001;.06,5,0,200,.001;.1,20,0,600,.001; ...
 .03,1,0,0,.001;.06,5,0,0,.001;.1,20,0,0,.001; ...
 .03,1,.00003,100,.001;.06,5,.0001,200,.0005;.1,20,.0001,0,.0005];
prior=jsondecode(fileread(fullfile(source,'manifest.json')));configs=struct([]);metadata=struct([]);index=0;
for gainID=1:size(profiles,1)
 for k=1:11
  p=cases(k).p;r=profiles(gainID,:);scale=20/24.4;
  p.inertial.forceKp=r(1)*scale;p.inertial.forceKi=r(2)*scale;p.inertial.forceKd=r(3)*scale;p.inertial.derivativeFilter=r(5);
  p.coil.forceKp=p.inertial.forceKp;p.coil.forceKi=p.inertial.forceKi;p.coil.forceKd=p.inertial.forceKd;p.coil.derivativeFilter=r(5);
  p.coil.strategy='pi';if r(3)>0,p.coil.strategy='pid';end
  p.inertial.lowerLoadInverse.enabled=r(4)>0;p.inertial.lowerLoadInverse.cutoff_Hz=max(r(4),100);
  p.inertial.lowerLoadInverse.scale=1;p.mechanicalStudy.gainID=gainID;p.mechanicalStudy.controlDevelopmentLane='proper_PI_PID_lowpass_inverse_v2';
  p=acf_prepare(p);assert(p.mechanics.c1==0&&p.mechanics.c2==0&&p.mechanics.upperRigid&&p.mechanics.k1==19746);
  identity=sprintf('upper_rigid_lower_undamped_control_v2_gain%02d_dev%02d',gainID,k-1);
  row=struct('p',p,'identity',identity,'split','development');
  if isempty(configs),configs=row;else,configs(end+1)=row;end
  m=prior.cases(k);m.scenario=index;m.identity=identity;m.gainID=gainID;
  m.referenceKp_A_N=r(1);m.referenceKi_A_N_s=r(2);m.referenceKd_A_s_N=r(3);m.forceKp_A_N=p.inertial.forceKp;m.forceKi_A_N_s=p.inertial.forceKi;
  m.forceKd_A_s_N=p.inertial.forceKd;m.derivativeFilter_s=r(5);m.lowerInverseEnabled=r(4)>0;m.lowerInverseCutoff_Hz=r(4);m.controllerStrategy=p.coil.strategy;
  if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
  index=index+1;
 end
end
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=prior;manifest.status='frozen_control_development_not_executed';
manifest.cases=metadata;manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));
manifest.gainProfiles=profiles;manifest.profileColumns={'referenceKp_A_N','referenceKi_A_N_s','referenceKd_A_s_N','lowerInverseCutoff_Hz_zero_disables','derivativeFilter_s'};
manifest.developmentContract='Second bounded deterministic development lane on the SAME11 development identities/seeds and frozen budget plant. Proper PI/PID with identified lower inverse disabled or rolled off100/200/600Hz. No fresh-test tuning or learned-policy changes.';
manifest.controlDevelopmentLane='proper_PI_PID_lowpass_inverse_v2';
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_undamped_control_bank.m'));
fprintf('UPPER_UNDAMPED_CONTROL_BANK %d cases\n',numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
