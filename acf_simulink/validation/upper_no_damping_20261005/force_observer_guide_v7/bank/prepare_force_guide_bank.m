function manifest=prepare_force_guide_bank(folder)
% Two accepted force-observer parents x six controller-only guide choices.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
source=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005','bank_observer_noise_v4');
loaded=load(fullfile(source,'configs.mat'),'configs');prior=jsondecode(fileread(fullfile(source,'manifest.json')));
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
profiles=[0,.0002;1,.001;1,.002;1,.005;.5,.001;.5,.002];parents=[24,33];
% Unscored sentinel retains Coder's exact original emitted case0 parameters.
configs=loaded.configs(1);configs.p.guideFeedforwardStudy=struct('coulombScale',1,'smoothingSpeed_m_s',.0002);
configs.identity='force_observer_guide_v7_original_coder_case0_sentinel';configs.split='preparation_check';
configs.sourceNativeScenario=0;metadata=prior.cases(1);metadata.scenario=0;metadata.identity=configs.identity;
metadata.split='preparation_check';metadata.gainID=0;metadata.parentObserverProfileID=1;metadata.guideProfileID=0;
metadata.sourceNativeScenario=0;metadata.guideFeedforwardCoulombScale=1;metadata.guideFeedforwardSmoothingSpeed_m_s=.0002;
metadata.upperLengthUnconstrained=true;metadata.upperPair.lengthAndTravelUnconstrained=true;
families=struct([]);index=0;
for parentID=parents
 for guideID=1:size(profiles,1)
  familyID=numel(families)+1;
  family=struct('familyID',familyID,'parentObserverProfileID',parentID,'guideProfileID',guideID, ...
   'guideFeedforwardCoulombScale',profiles(guideID,1),'guideFeedforwardSmoothingSpeed_m_s',profiles(guideID,2));
  if isempty(families),families=family;else,families(end+1)=family;end
  for k=1:22
   sourceIndex=(parentID-1)*22+k;p=loaded.configs(sourceIndex).p;
   assert(p.mechanics.k2==20000&&p.mechanics.c2==0&&~p.mechanics.upperRigid);
   assert(p.mechanics.k1==64e6&&p.mechanics.c1==0&&p.coil.coulombForce==.2&&p.coil.viscousFriction==2&&p.coil.frictionSpeed==.0002);
   p.guideFeedforwardStudy=struct('coulombScale',profiles(guideID,1),'smoothingSpeed_m_s',profiles(guideID,2));
   p.mechanicalStudy.controlDevelopmentLane='force_observer_guide_v7';p.mechanicalStudy.gainID=familyID;
   p.mechanicalStudy.parentObserverProfileID=parentID;p.mechanicalStudy.guideProfileID=guideID;
   p.mechanicalStudy.upperSpringLengthAndTravelUnconstrained=true;
   p=acf_prepare(p);assert(isequaln(p.d,loaded.configs(sourceIndex).p.d),'Original prepared coefficients changed.');
   index=index+1;identity=sprintf('force_observer_guide_v7_parent%02d_guide%d_A%.1f_dev%02d',parentID,guideID,p.mechanics.mA,mod(k-1,11));
   row=loaded.configs(sourceIndex);row.p=p;row.identity=identity;row.sourceNativeScenario=sourceIndex-1;
   configs(end+1)=row;
   m=prior.cases(sourceIndex);m.scenario=index;m.identity=identity;m.gainID=familyID;
   m.parentObserverProfileID=parentID;m.guideProfileID=guideID;m.sourceNativeScenario=sourceIndex-1;
   m.guideFeedforwardCoulombScale=profiles(guideID,1);m.guideFeedforwardSmoothingSpeed_m_s=profiles(guideID,2);
   m.upperLengthUnconstrained=true;m.upperPair.lengthAndTravelUnconstrained=true;
   metadata(end+1)=m;
  end
 end
end
assert(numel(configs)==265&&index==264);
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=prior;
manifest.status='frozen_force_observer_guide_development_not_executed';manifest.cases=metadata;
manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));
manifest.controlDevelopmentLane='force_observer_guide_v7';manifest.families=families;manifest.guideProfiles=profiles;
manifest.developmentCaseCount=264;manifest.unscoredSentinelCount=1;manifest.upperLengthUnconstrained=true;
manifest.profileIndexContract='scenario0 is unscored originalCoderCase0 sentinel; development scenario=1+22*(familyID-1)+11*massAIndex+caseID. familyID1:6 use observerparent24,7:12 parent33. GuideIDs1:6 are OFF,full1/2/5mm/s,half1/2mm/s; massAIndex0=.4kg,1=1.5kg.';
manifest.developmentContract='Two accepted acquired-force observer parents24/33 x six controller-only Coulombcompensation scale/smoothing choices x bothA x SAME11 developmentcases/seeds. All physical and observer friction,PI/notch/filter,sensing,acquisition,gates and mechanics fixed. Originalcase0 sentinel preserves unchangedCoder parameter guard and is never a forcequalification case. No newC graph unless an exactmappedcoefficient/context reuse guard fails. No fresh-test tuning/training/promotion.';
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_force_guide_bank.m'));
fprintf('FORCE_GUIDE_V7_BANK %d development +1 sentinel\n',index);
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
