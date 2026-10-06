function manifest=prepare_observer_noise_bank_v4(folder)
% Freeze a bounded observer design bank; no force-outcome selection here.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005','bank_cellsource_v2');
parent=jsondecode(fileread(fullfile(source,'manifest.json')));loaded=load(fullfile(source,'configs.mat'),'configs');
base=loaded.configs(1:22);baseMetadata=parent.cases(1:22);
assert(all([baseMetadata.gainID]==1)&&numel(unique([baseMetadata.caseID]))==11);
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));recipe=fileparts(mfilename('fullpath'));
copyfile(fullfile(recipe,'observer_noise_v4','acf_prepare_upper_observer.m'),fullfile(folder,'helpers','acf_prepare_upper_observer.m'));
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
% Columns are force design density, acceleration design density, and the
% assumed position RMS used ONLY for the observer's measurement covariance.
% Actual conditioned sensor noise remains identical to the parent bank.
qr=[.05,.04,1e-6;.005,.004,1e-6;.0005,.0004,1e-6;.05,.004,1e-6; ...
 .005,.0004,1e-6;.005,.004,2e-6;.005,.004,5e-6; ...
 .0005,.0004,2e-6;.0005,.0004,5e-6;.05,.04,5e-6];
cutoffs=[150,400,800,1500];design='upper_20k_lower_cell_observer_noise_v4';
configs=repmat(base(1),1,size(qr,1)*numel(cutoffs)*numel(base));
metadata=repmat(baseMetadata(1),numel(configs),1);profiles=struct([]);j=0;profile=0;
for covarianceID=1:size(qr,1)
 for derivativeID=1:numel(cutoffs)
  profile=profile+1;row=struct('profileID',profile,'covarianceProfileID',covarianceID, ...
   'derivativeProfileID',derivativeID,'forceProcessDensity_N_sqrtHz',qr(covarianceID,1), ...
   'accelerationProcessDensity_m_s2_sqrtHz',qr(covarianceID,2), ...
   'positionAlgorithmRms_m',qr(covarianceID,3),'forceDerivativeCutoff_Hz',cutoffs(derivativeID), ...
   'controlBaseGainID',1,'gainA04',[],'gainA15',[],'VsPositionGainA04_s_1',[], ...
   'VsPositionGainA15_s_1',[],'maximumPoleMagnitudeA04',[],'maximumPoleMagnitudeA15',[], ...
   'derivativeHighFrequencyCorrectionGain',[]);
  for b=1:numel(base)
   j=j+1;p=base(b).p;
   assert(p.upperObserver.enabled&&~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0);
   assert(p.mechanics.k1==64e6&&p.mechanics.c1==0&&~p.inertial.lowerLoadInverse.enabled);
   p.upperObserver.forceProcessDensity_N_sqrtHz=qr(covarianceID,1);
   p.upperObserver.accelerationProcessDensity_m_s2_sqrtHz=qr(covarianceID,2);
   p.upperObserver.positionMeasurementRms_m=qr(covarianceID,3);
   p.upperObserver.forceDerivativeCutoff_Hz=cutoffs(derivativeID);
   p.upperObserver.contract='Four-state causal upper observer driven by the explicit local matched acquired axial cell deviation, identified holder inertia and a proper critically damped cell derivative. Only observer Q/R design weights and derivative cutoff vary; actual sensor noise/timing, physical mechanics and PI/notch remain unchanged.';
   p.mechanicalStudy.parentDesign=parent.design;p.mechanicalStudy.design=design;
   p.mechanicalStudy.observerNoiseProfileID=profile;
   p=acf_prepare(p);verify_scientific_parity(base(b).p,p);
   configs(j)=base(b);configs(j).p=p;
   configs(j).identity=sprintf('%s_obs%02d_A%.1f_dev%02d',design,profile,p.mechanics.mA,baseMetadata(b).caseID);
   metadata(j)=baseMetadata(b);metadata(j).scenario=j-1;metadata(j).identity=configs(j).identity;
   metadata(j).design=design;metadata(j).gainID=profile;
   % Append new fields after construction below so all metadata rows agree.
   if b==1
    row.gainA04=p.d.upperMotionObserver.gain;
    row.VsPositionGainA04_s_1=row.gainA04(2)-row.gainA04(4);
    row.maximumPoleMagnitudeA04=p.d.upperMotionObserver.maximumPoleMagnitude;
   elseif b==12
    row.gainA15=p.d.upperMotionObserver.gain;
    row.VsPositionGainA15_s_1=row.gainA15(2)-row.gainA15(4);
    row.maximumPoleMagnitudeA15=p.d.upperMotionObserver.maximumPoleMagnitude;
   end
   row.derivativeHighFrequencyCorrectionGain=p.d.upperMotionObserver.measuredForceDerivativeScale*p.d.upperMotionObserver.measuredForceDerivativeD;
  end
  if isempty(profiles),profiles=row;else,profiles(end+1)=row;end
 end
end
for j=1:numel(metadata)
 profileID=floor((j-1)/22)+1;r=profiles(profileID);
 metadata(j).observerProfileID=profileID;metadata(j).controlBaseGainID=1;
 metadata(j).covarianceProfileID=r.covarianceProfileID;metadata(j).derivativeProfileID=r.derivativeProfileID;
 metadata(j).forceProcessDensity_N_sqrtHz=r.forceProcessDensity_N_sqrtHz;
 metadata(j).accelerationProcessDensity_m_s2_sqrtHz=r.accelerationProcessDensity_m_s2_sqrtHz;
 metadata(j).positionAlgorithmRms_m=r.positionAlgorithmRms_m;
 metadata(j).forceDerivativeCutoff_Hz=r.forceDerivativeCutoff_Hz;
end
% Matching nominal weights/cutoff must reproduce every frozen v3 coefficient.
for b=1:22,assert(isequaln(configs(66+b).p.d.upperMotionObserver,base(b).p.d.upperMotionObserver));end
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=parent;
manifest.status='frozen_bounded_observer_design_bank_not_executed';manifest.parentDesign=parent.design;manifest.design=design;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.cases=metadata;manifest.observerProfiles=profiles;
manifest.gainProfiles=[];manifest.profileColumns={};manifest.controlBaseGainProfiles=parent.gainProfiles(1,:);
manifest.upperObserver=configs(1).p.upperObserver;manifest.mechanicalContract=configs(1).p.mechanicalStudy;
manifest.profileIndexContract='zero-based scenario = 22*(observerProfileID-1) + 11*massAIndex + caseID; massAIndex0=.4kg,1=1.5kg. observerProfileID = 4*(covarianceProfileID-1)+derivativeProfileID. gainID aliases observerProfileID; all use parent control gainID1.';
manifest.designWeightContract='Force/acceleration densities and positionAlgorithmRms_m are observer tuning weights; they do not change actual injected noise, quantization, calibration, sensor response, acquisition, pipeline or latency. New derivative cutoff changes only the cell-deformation term in deltaFa.';
manifest.developmentContract='Prepared 40 bounded observer profiles × the exact11 previous development cases/seeds991000+caseID × bothA masses. No fresh-test outcomes read, no training, no physical changes. Preparing880cases does not claim all have been executed. Root selects bounded endpoint probes before broader development evaluation.';
manifest.scientificParity=struct('allPhysicalSensorControllerSectionsUnchanged',true,'preparedNonObserverCoefficientsUnchanged',true, ...
 'nominal1500HzProfile4ReproducesFrozenV3All22ObserverCoefficients',true,'createdQualificationEpisodes',0);
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_observer_noise_bank_v4.m'));
fprintf('BOUNDED_OBSERVER_NOISE_BANK %d cases, %d profiles\n',numel(configs),numel(profiles));
end
function verify_scientific_parity(before,after)
names=setdiff(fieldnames(before),{'d','upperObserver','mechanicalStudy'});
for j=1:numel(names),assert(isequaln(before.(names{j}),after.(names{j})),'Physical/controller/sensor field %s changed.',names{j});end
dBefore=rmfield(before.d,'upperMotionObserver');dAfter=rmfield(after.d,'upperMotionObserver');
assert(isequaln(dBefore,dAfter),'A non-observer prepared coefficient changed.');
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
