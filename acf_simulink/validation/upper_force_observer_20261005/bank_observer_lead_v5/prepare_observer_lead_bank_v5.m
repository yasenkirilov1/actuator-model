function manifest=prepare_observer_lead_bank_v5(folder)
% Eight bounded proper FF leads on each of two selected development observers.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005','bank_observer_noise_v4');
parent=jsondecode(fileread(fullfile(source,'manifest.json')));loaded=load(fullfile(source,'configs.mat'),'configs');
recipe=fileparts(mfilename('fullpath'));copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
for name={'acf_prepare.m','acf_prepare_accel_lead.m'}
 copyfile(fullfile(recipe,'accel_lead_v5',name{1}),fullfile(folder,'helpers',name{1}));
end
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_accel_lead;
selectedObservers=[24,33];leadPairs=1e-6*[150,200;150,400;250,200;250,400;350,200;350,400;350,1000;500,1000];
design='upper_20k_lower_cell_selected_observer_accel_lead_v5';configs=struct([]);metadata=struct([]);profiles=struct([]);j=0;family=0;
for observerIndex=1:numel(selectedObservers)
 observerID=selectedObservers(observerIndex);baseIndices=22*(observerID-1)+(1:22);
 for leadID=1:size(leadPairs,1)
  family=family+1;r=leadPairs(leadID,:);
  row=struct('familyID',family,'parentObserverProfileID',observerID,'leadID',leadID, ...
   'advanceTime_s',r(1),'poleTime_s',r(2),'discreteNumerator',[], ...
   'discreteDenominator',[],'highFrequencyGain',[],'maximumObserverPoleMagnitudeA04',[], ...
   'maximumObserverPoleMagnitudeA15',[]);
  for b=1:numel(baseIndices)
   base=loaded.configs(baseIndices(b));m=parent.cases(baseIndices(b));p=base.p;j=j+1;
   assert(p.inertial.accelerationPrediction_s==0);
   assert(~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0&&p.mechanics.k1==64e6&&p.mechanics.c1==0);
   p.accelLead=struct('enabled',true,'advanceTime_s',r(1),'poleTime_s',r(2),'sampleTime_s',p.force.sampleTime);
   p.mechanicalStudy.parentDesign=parent.design;p.mechanicalStudy.design=design;
   p.mechanicalStudy.parentObserverProfileID=observerID;p.mechanicalStudy.accelerationLeadFamilyID=family;
   p=acf_prepare(p);verify_parity(base.p,p);
   identity=sprintf('%s_obs%02d_lead%02d_A%.1f_dev%02d',design,observerID,leadID,p.mechanics.mA,m.caseID);
   base.p=p;base.identity=identity;
   if isempty(configs),configs=base;else,configs(end+1)=base;end
   m.scenario=j-1;m.identity=identity;m.design=design;m.gainID=family;m.leadFamilyID=family;
   m.parentObserverProfileID=observerID;m.accelerationLeadID=leadID;
   m.accelerationAdvanceTime_s=r(1);m.accelerationLeadPoleTime_s=r(2);
   m.accelerationLeadHighFrequencyGain=p.d.accelLead.highFrequencyGain;
   m.accelerationLeadNumerator=p.d.accelLead.numerator;m.accelerationLeadDenominator=p.d.accelLead.denominator;
   m.accelerationLeadClock_s=p.force.sampleTime;
   if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
   row.discreteNumerator=p.d.accelLead.numerator;row.discreteDenominator=p.d.accelLead.denominator;
   row.highFrequencyGain=p.d.accelLead.highFrequencyGain;
   if b==1,row.maximumObserverPoleMagnitudeA04=p.d.upperMotionObserver.maximumPoleMagnitude;
   elseif b==12,row.maximumObserverPoleMagnitudeA15=p.d.upperMotionObserver.maximumPoleMagnitude;end
  end
  if isempty(profiles),profiles=row;else,profiles(end+1)=row;end
 end
end
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=parent;
manifest.status='frozen_bounded_selected_observer_accel_lead_bank_not_executed';
manifest.parentDesign=parent.design;manifest.design=design;manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.observerProfiles=parent.observerProfiles(selectedObservers);manifest.leadFamilies=profiles;
manifest.gainProfiles=[[profiles.parentObserverProfileID]',[profiles.advanceTime_s]',[profiles.poleTime_s]'];
manifest.profileColumns={'parentObserverProfileID','accelerationAdvanceTime_s','accelerationLeadPoleTime_s'};
manifest.upperObserver=configs(1).p.upperObserver;manifest.mechanicalContract=configs(1).p.mechanicalStudy;
manifest.profileIndexContract='zero-based scenario = 176*observerIndex0 +22*(leadID1-1)+11*massAIndex0+caseID0. observerIndex0=0 selects parent24,1 selects parent33. massAIndex0=0 selects.4kg,1 selects1.5kg. gainID aliases leadFamilyID1=8*observerIndex0+leadID1; controlBaseGainID stays1.';
manifest.developmentContract='Eight proper causal acceleration-current-FF leads on each of parent observer24/33, chosen from previous case7 development recordings. Exact same11 cases/seeds and twoA masses. No fresh-test outcomes read, no learning, no guide/physical/sensor changes. Force reconstruction, normal estimate and measurement-frame observer unchanged. Preparing352cases is not executed qualification.';
manifest.scientificParity=struct('allExistingPhysicalSensorControllerSectionsUnchanged',true, ...
 'allExistingPreparedCoefficientsIncludingUpperObserverUnchanged',true, ...
 'forceReconstructionAndObserverMeasurementFrameUnchanged',true,'soleNewFunction','Proper causal current-FF S-acceleration lead', ...
 'createdQualificationEpisodes',0);
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_observer_lead_bank_v5.m'));
fprintf('BOUNDED_OBSERVER_ACCEL_LEAD_BANK %d cases, %d families\n',numel(configs),numel(profiles));
end
function verify_parity(before,after)
names=setdiff(fieldnames(before),{'d','mechanicalStudy'});
for j=1:numel(names),assert(isequaln(before.(names{j}),after.(names{j})),'Existing field %s changed.',names{j});end
assert(isequaln(before.d,rmfield(after.d,'accelLead')),'An existing prepared coefficient changed.');
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
