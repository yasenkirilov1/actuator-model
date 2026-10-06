function manifest=prepare_guide_feedforward_bank(folder)
% V5 profile1 with only algorithmic Coulomb guide FF scale/smoothing varied.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
source=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005','corrected_current_observer_v5','bank');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
old=load(fullfile(source,'configs.mat'),'configs');cases=old.configs(1:22);
prior=jsondecode(fileread(fullfile(source,'manifest.json')));configs=struct([]);metadata=struct([]);index=0;
% Coulomb scale, algorithm smoothing speed. Viscous compensation is fixed.
profiles=[0,.0002;1,.001;1,.002;1,.005;.5,.001;.5,.002];
for gainID=1:size(profiles,1)
 for k=1:22
  p=cases(k).p;assert(~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0);
  assert(p.mechanics.k1==64e6&&p.mechanics.c1==0&&p.upperObserver.enabled);
  assert(p.coil.coulombForce==.2&&p.coil.viscousFriction==2&&p.coil.frictionSpeed==.0002);
  p.guideFeedforwardStudy=struct('coulombScale',profiles(gainID,1),'smoothingSpeed_m_s',profiles(gainID,2), ...
   'physicalFrictionUnchanged',true,'observerFrictionEstimateUnchanged',true,'viscousFeedforwardUnchanged',true);
  p.mechanicalStudy.gainID=gainID;p.mechanicalStudy.controlDevelopmentLane='corrected_current_observer_guideff_v6';
  p.mechanicalStudy.upperSpringLengthAndTravelUnconstrained=true;p.mechanicalStudy.externalUpperPair.lengthAndTravelUnconstrained=true;
  p=acf_prepare(p);assert(abs(p.inertial.forceKp-.03*20/24.4)<1e-12&&abs(p.inertial.forceKi-20*20/24.4)<1e-12);
  identity=sprintf('corrected_current_observer_guideff_v6_gain%d_A%.1f_dev%02d',gainID,p.mechanics.mA,mod(k-1,11));
  row=struct('p',p,'identity',identity,'split','development');
  if isempty(configs),configs=row;else,configs(end+1)=row;end
  m=prior.cases(k);m.scenario=index;m.identity=identity;m.gainID=gainID;
  m.guideFeedforwardCoulombScale=profiles(gainID,1);m.guideFeedforwardSmoothingSpeed_m_s=profiles(gainID,2);
  m.guideFeedforwardViscousScale=1;m.physicalGuideFrictionUnchanged=true;m.observerGuideFrictionEstimateUnchanged=true;
  m.upperLengthUnconstrained=true;m.upperSpringLengthAndTravelUnconstrained=true;m.externalUpperPair.lengthAndTravelUnconstrained=true;
  if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
  index=index+1;
 end
end
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=prior;
manifest.status='frozen_guide_feedforward_development_not_executed';manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.gainProfiles=profiles;manifest.profileColumns={'guideFeedforwardCoulombScale','guideFeedforwardSmoothingSpeed_m_s'};
manifest.controlDevelopmentLane='corrected_current_observer_guideff_v6';manifest.upperLengthUnconstrained=true;
manifest.developmentContract='Mandatory upper20k/c0/unlimited, lower externally rigid finite64MN/m sensingcell/c0. CorrectedV5 liveissuedcommand predictor/sourceguard retained; base profile1PI/notch/filter. Six algorithm-only CoulombguideFF scale/smoothing profiles on SAME11cases/twoA/seeds; physicalfriction(.2N,2Ns/m,.2mm/s), observerfrictionestimate, viscousFF, forceFF, sensors/current32V/50us allunchanged. No fresh-test tuning or neuraltraining.';
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_guide_feedforward_bank.m'));
fprintf('CORRECTED_CURRENT_GUIDEFF_V6_BANK %d cases\n',numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
