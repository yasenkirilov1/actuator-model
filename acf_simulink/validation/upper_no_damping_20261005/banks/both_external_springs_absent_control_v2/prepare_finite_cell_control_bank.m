function manifest=prepare_finite_cell_control_bank(folder)
% Bounded stronger PI/lower-bandwidth development on the same physical cases.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005','banks','both_external_springs_absent');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
original=load(fullfile(source,'configs.mat'),'configs');cases=original.configs(1:11);
profiles=[.1,20,300;.1,20,500;.2,20,300;.2,20,500;.3,20,300;.3,20,500];
prior=jsondecode(fileread(fullfile(source,'manifest.json')));configs=struct([]);metadata=struct([]);index=0;
for gainID=1:size(profiles,1)
 for k=1:11
  p=cases(k).p;r=profiles(gainID,:);scale=20/24.4;
  p.inertial.forceKp=r(1)*scale;p.inertial.forceKi=r(2)*scale;p.inertial.forceKd=0;p.coil.strategy='pi';
  p.coil.forceKp=p.inertial.forceKp;p.coil.forceKi=p.inertial.forceKi;p.coil.forceKd=0;
  p.inertial.feedbackFilter.enabled=true;p.inertial.feedbackFilter.cutoff_Hz=r(3);p.inertial.feedbackFilter.damping=1.5;
  p.inertial.currentNotch.enabled=true;p.inertial.currentNotch.frequency_Hz=sqrt(64e6/.6)/(2*pi);
  p.inertial.currentNotch.zeroDamping=.01;p.inertial.currentNotch.poleDamping=.3;p.inertial.lowerLoadInverse.enabled=false;
  p.mechanicalStudy.gainID=gainID;p.mechanicalStudy.controlDevelopmentLane='finite_cell_stronger_PI_lower_bandwidth_v2';
  p=acf_prepare(p);assert(p.mechanics.c1==0&&p.mechanics.c2==0&&p.mechanics.upperRigid&&p.mechanics.k1==64e6);
  assert(~p.inertial.lowerLoadInverse.enabled&&p.adcStudy.outputRate_Hz==30000&&p.coil.currentResponse==50e-6);
  identity=sprintf('both_external_springs_absent_control_v2_gain%02d_dev%02d',gainID,k-1);
  row=struct('p',p,'identity',identity,'split','development');
  if isempty(configs),configs=row;else,configs(end+1)=row;end
  m=prior.cases(k);m.scenario=index;m.identity=identity;m.gainID=gainID;
  m.referenceKp_A_N=r(1);m.referenceKi_A_N_s=r(2);m.forceKp_A_N=p.inertial.forceKp;m.forceKi_A_N_s=p.inertial.forceKi;
  m.feedbackCutoff_Hz=r(3);m.feedbackDamping=1.5;m.notchEnabled=true;m.notchPoleDamping=.3;
  m.lowerInverseEnabled=false;m.controllerStrategy='pi';
  if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
  index=index+1;
 end
end
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=prior;manifest.status='frozen_control_development_not_executed';
manifest.cases=metadata;manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));
manifest.gainProfiles=profiles;manifest.profileColumns={'referenceKp_A_N','referenceKi_A_N_s','feedbackCutoff_Hz'};
manifest.developmentContract='Six strongerPI/lowerfeedbackbandwidth profiles on SAME11 development cases/seeds. No external springs/dampers; finite sensingcell64MN/m. Feedbackdamping1.5,1.644kHzcurrentnotch zero0.01/pole0.3, no lowerinverse. Sensors,32Vhotdrive,50uscurrent fixed. No fresh-test tuning or neural changes.';
manifest.controlDevelopmentLane='finite_cell_stronger_PI_lower_bandwidth_v2';
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_finite_cell_control_bank.m'));
fprintf('FINITE_CELL_STRONGER_PI_BANK %d cases\n',numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
