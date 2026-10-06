function manifest=prepare_accel_lead_bank(folder)
% Six bounded causal FF predictions on the same 11 development cases/seeds.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
recipe=fileparts(mfilename('fullpath'));root=fileparts(fileparts(fileparts(recipe)));
source=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005','banks','both_external_springs_absent');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
copyfile(fullfile(recipe,'acf_prepare_accel_lead.m'),fullfile(folder,'helpers','acf_prepare_accel_lead.m'));
fid=fopen(fullfile(folder,'helpers','acf_prepare.m'),'w');assert(fid>=0);
fprintf(fid,'function p=acf_prepare(p)\np=acf_adc_prepare(p);\np=acf_prepare_upper_observer(p);\np=acf_prepare_accel_lead(p);\nend\n');fclose(fid);
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_accel_lead;
original=load(fullfile(source,'configs.mat'),'configs');cases=original.configs(1:11);
profiles=[150e-6,200e-6;150e-6,400e-6;250e-6,200e-6;250e-6,400e-6;350e-6,200e-6;350e-6,400e-6];
prior=jsondecode(fileread(fullfile(source,'manifest.json')));configs=struct([]);metadata=struct([]);index=0;
for gainID=1:size(profiles,1)
 for k=1:11
  p=cases(k).p;r=profiles(gainID,:);
  p.inertial.accelerationPrediction_s=0;
  p.accelLead=struct('enabled',true,'advanceTime_s',r(1),'poleTime_s',r(2),'sampleTime_s',p.force.sampleTime);
  p.mechanicalStudy.gainID=gainID;p.mechanicalStudy.controlDevelopmentLane='finite_cell_acceleration_current_FF_lead_v1';
  p=acf_prepare(p);assert(p.mechanics.upperRigid&&p.mechanics.k1==64e6&&p.mechanics.c1==0&&p.mechanics.c2==0);
  assert(abs(p.inertial.forceKp-.03*20/24.4)<1e-12&&abs(p.inertial.forceKi-20*20/24.4)<1e-12);
  assert(p.inertial.currentNotch.enabled&&p.inertial.feedbackFilter.cutoff_Hz==1000&&~p.inertial.lowerLoadInverse.enabled);
  assert(p.adcStudy.outputRate_Hz==30000&&p.coil.currentResponse==50e-6&&p.coil.voltage==32);
  identity=sprintf('both_external_springs_absent_accel_lead_v1_gain%02d_dev%02d',gainID,k-1);
  row=struct('p',p,'identity',identity,'split','development');
  if isempty(configs),configs=row;else,configs(end+1)=row;end
  m=prior.cases(k);m.scenario=index;m.identity=identity;m.gainID=gainID;
  m.accelerationAdvanceTime_s=r(1);m.accelerationLeadPoleTime_s=r(2);m.accelerationLeadHighFrequencyGain=p.d.accelLead.highFrequencyGain;
  m.accelerationLeadNumerator=p.d.accelLead.numerator;m.accelerationLeadDenominator=p.d.accelLead.denominator;
  m.accelerationLeadClock_s=p.force.sampleTime;
  if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
  index=index+1;
 end
end
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=prior;manifest.status='frozen_control_development_not_executed';
manifest.cases=metadata;manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));
manifest.gainProfiles=profiles;manifest.profileColumns={'accelerationAdvanceTime_s','accelerationLeadPoleTime_s'};
manifest.controlDevelopmentLane='finite_cell_acceleration_current_FF_lead_v1';
manifest.developmentContract='Six proper causal current-FF acceleration leads on SAME11 development cases/seeds. Original profile1 PI/notch/feedback, finite sensingcell64MN/m, no external springs/dampers. Force reconstruction/normal estimate/sensing frame unchanged. 10kHz bilinear filter; no surface preview or future commands. Sensors,32Vhotdrive,50uscurrent fixed. No fresh-test tuning or neural changes.';
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_accel_lead_bank.m'));
fprintf('FINITE_CELL_ACCEL_LEAD_BANK %d cases\n',numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
