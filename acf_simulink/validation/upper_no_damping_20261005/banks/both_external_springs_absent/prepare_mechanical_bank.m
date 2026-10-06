function manifest=prepare_mechanical_bank(folder,designName)
% Prepare bounded mechanics/controller development cases, not outcomes.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','rl_budget3h_20261005','qualification_bank_budget700');
assert(~isfolder(folder),'Preserve every earlier prepared bank.');mkdir(folder);
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
for name={'acf_prepare.m','acf_prepare_upper_observer.m'}
 copyfile(fullfile(fileparts(mfilename('fullpath')),name{1}),fullfile(folder,'helpers',name{1}));
end
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
original=load(fullfile(source,'configs.mat'),'configs');base=original.configs(1).p;
assert(base.coil.forceConstant==24.4&&base.coil.resistance==8.896&&base.coil.inductance==.004&&base.coil.voltage==32);
base.robot.nominalVelocity=0;base.robot.errorForce=0;base.robot.initialExcessForce=0;base.robot.initialPositionOffset_m=0;
base.sim.stopTime=2;base.sim.evaluateFrom=0;base.surface.kind='sharp_bump';base.surface.startTime=1;
base.surface.height=.001;base.surface.width=.01;base.surface.scanSpeed=.1;
base.contact.mu=.3;base.sensor.distalMassCalibrationScale=1;
base.inertial.forceConstantCalibrationScale=1;base.inertial.guideFrictionFeedforwardScale=1;
base.uncertainty.massB_model_kg=base.mechanics.mB;base.uncertainty.forceConstant_model_N_A=base.coil.forceConstant;
base.upperObserver=struct('enabled',false,'identifiedMassA_kg',.4,'identifiedStiffness_N_m',20000, ...
 'identifiedDamping_Ns_m',0,'identifiedCurrentResponse_s',50e-6, ...
 'forceProcessDensity_N_sqrtHz',.05,'accelerationProcessDensity_m_s2_sqrtHz',.04, ...
 'positionMeasurementRms_m',1e-6, ...
 'contract','Causal four-state ZOH observer. No plant A/current/actuator-force/surface truth. Issued current has a strictly proper identified lag and the existing ADC frame; estimated guide friction is delayed one control tick. Actual and identified mechanics match in development.');
base.mechanicalStudy=struct('design',designName,'lowerExternalSpringPresent',true,'upperExternalSpringPresent',false, ...
 'finiteLoadCellStiffness_N_m',64e6,'loadCellStructuralDamping_N_s_m',0, ...
 'cellStiffnessIsExternalSpring',false,'springPackingVerified',false, ...
 'combinedFreeSpringLength_m',0,'combinedInstalledLengthUpperBound_m',0, ...
 'packingScope','External opposed pairs only; excludes sensor cell and all frame/guide/driver dimensions. No purchased parts or structural prototype qualification.');
finiteCell=false;compliantUpper=false;
switch designName
 case 'baseline_upper_rigid_lower_damped'
  base.mechanics.upperRigid=true;base.mechanics.k1=19746;base.mechanics.c1=60;base.mechanics.c2=0;
 case 'upper_rigid_lower_undamped'
  base.mechanics.upperRigid=true;base.mechanics.k1=19746;base.mechanics.c1=0;base.mechanics.c2=0;
 case 'upper_20k_lower_cell_undamped'
  compliantUpper=true;finiteCell=true;
 case 'upper_20k_lower_19746_undamped'
  compliantUpper=true;base.mechanics.k1=19746;base.mechanics.c1=0;base.mechanics.c2=0;
 case 'both_external_springs_absent'
  finiteCell=true;base.mechanics.upperRigid=true;
 otherwise,error('Unknown mechanical design %s.',designName);
end
if compliantUpper
 base.mechanics.upperRigid=false;base.mechanics.k2=20000;base.mechanics.c2=0;
 base.upperObserver.enabled=true;
end
base.mechanics.lowerRigid=false;
if finiteCell
 base.mechanics.k1=64e6;base.mechanics.c1=0;base.mechanics.c2=0;
 base.inertial.lowerLoadInverse.enabled=false;base.mechanicalStudy.lowerExternalSpringPresent=false;
 % Do not mislabel the finite sensing element as an opposed guided spring.
 base=rmfield(base,'springs');
else
 base.inertial.lowerLoadInverse.enabled=true;
 base.springs.lower.rate_N_m=base.mechanics.k1/2;
 if compliantUpper,base.springs.upper.rate_N_m=10000;end
end
base.mechanicalStudy.upperExternalSpringPresent=compliantUpper;
pairTarget=struct('present',true,'preloadPerSpring_N',33.6,'singleSpringRate_N_m',9873, ...
 'freeLength_m',.020,'freeLengthTolerance_m',.00049,'minWorkingLength_m',.01055, ...
 'maxCyclicStrokeTarget_m',.00318,'forceEnvelopeTarget_N',68.543, ...
 'dynamicOperationQualified',false);
lowerPair=pairTarget;lowerPair.present=~finiteCell;lowerPair.singleSpringRate_N_m=base.mechanics.k1/2;
upperPair=pairTarget;upperPair.present=compliantUpper;upperPair.singleSpringRate_N_m=10000;
base.mechanicalStudy.externalLowerPair=lowerPair;base.mechanicalStudy.externalUpperPair=upperPair;
pairLength=2*(.020+.00049);
base.mechanicalStudy.combinedFreeSpringLength_m=pairLength*(~finiteCell+compliantUpper);
base.mechanicalStudy.combinedInstalledLengthUpperBound_m=base.mechanicalStudy.combinedFreeSpringLength_m;
base.mechanicalStudy.springPackingVerified=base.mechanicalStudy.combinedFreeSpringLength_m<=.100;
base.inertial.lowerLoadInverse.identifiedStiffness_N_m=base.mechanics.k1;
base.inertial.lowerLoadInverse.identifiedDamping_Ns_m=base.mechanics.c1;
base.inertial.lowerLoadInverse.identifiedMassB_kg=base.mechanics.mB;
base.inertial.lowerLoadInverse.identifiedForceConstant_N_A=base.coil.forceConstant;
base.inertial.lowerLoadInverse.scale=1;
base.inertial.lowerLoadInverse.stiffnessCalibrationScale=1;base.inertial.lowerLoadInverse.stiffnessCalibrationOffset_N_m=0;
base.inertial.lowerLoadInverse.dampingCalibrationScale=1;base.inertial.lowerLoadInverse.dampingCalibrationOffset_Ns_m=0;
if finiteCell
 % Columns: reference Kp, Ki, feedback cutoff, notch enabled, pole damping.
 profiles=[.03,20,1000,1,.3;.06,20,800,1,.3;.01,5,800,1,.3; ...
  .03,20,1000,1,.5;.06,20,800,1,.5;.01,5,800,0,.3];
elseif strcmp(designName,'baseline_upper_rigid_lower_damped')
 profiles=[.3,20,1000,0,.3];
else
 profiles=[.3,20,1000,0,.3;.1,5,1000,0,.3;.06,1,1000,0,.3];
end
cases=zeros(11,3);i=0;
for s=[.1,2],for f=[5,50],for a=[0,90],i=i+1;cases(i,:)=[s,f,a];end,end,end
cases(9:11,:)=[.5,20,30;1.1,30,45;1.4,40,60];
massA=.4;if compliantUpper,massA=[.4,1.5];end
configs=struct([]);metadata=struct([]);index=0;
for gainID=1:size(profiles,1)
 for aMass=massA
  for caseID=1:size(cases,1)
   p=base;p.mechanics.mA=aMass;p.mechanics.mS=cases(caseID,1);p.force.reference=cases(caseID,2);p.mechanics.angle=cases(caseID,3)*pi/180;
   p.upperObserver.identifiedMassA_kg=aMass;p.sensor.distalMass_kg=p.mechanics.mS;p.sensor.distalMass_model_kg=p.mechanics.mS;
   p.sensor.seed=991000+caseID-1;reference=profiles(gainID,:);scale=20/24.4;
   p.inertial.forceKp=reference(1)*scale;p.inertial.forceKi=reference(2)*scale;p.inertial.forceKd=0;
   p.coil.forceKp=p.inertial.forceKp;p.coil.forceKi=p.inertial.forceKi;p.coil.forceKd=0;
   p.inertial.feedbackFilter.enabled=true;p.inertial.feedbackFilter.cutoff_Hz=reference(3);
   if finiteCell,p.inertial.feedbackFilter.damping=1.5;else,p.inertial.feedbackFilter.damping=1/sqrt(2);end
   p.inertial.currentNotch.enabled=logical(reference(4));p.inertial.currentNotch.frequency_Hz=sqrt(64e6/.6)/(2*pi);
   p.inertial.currentNotch.zeroDamping=.01;p.inertial.currentNotch.poleDamping=reference(5);
   p.mechanicalStudy.gainID=gainID;p.mechanicalStudy.caseID=caseID-1;
   p=acf_prepare(p);assert(p.sim.step==1/300000&&p.force.sampleTime==1e-4&&p.adcStudy.outputRate_Hz==30000);
   assert(abs(p.adcStudy.d.channels.frame.cutoff_Hz-1900)<1e-9&&p.d.u0==0);
   identity=sprintf('%s_gain%d_A%g_dev%02d',designName,gainID,aMass,caseID-1);
   row=struct('p',p,'identity',identity,'split','development');
   if isempty(configs),configs=row;else,configs(end+1)=row;end
   m=struct('scenario',index,'identity',identity,'split','development','design',designName,'gainID',gainID,'caseID',caseID-1, ...
    'massS_kg',p.mechanics.mS,'identifiedMassS_kg',p.sensor.distalMass_model_kg,'massB_kg',p.mechanics.mB,'identifiedMassB_kg',p.uncertainty.massB_model_kg, ...
    'massA_kg',aMass,'identifiedMassA_kg',p.upperObserver.identifiedMassA_kg,'force_N',p.force.reference,'angle_rad',p.mechanics.angle, ...
    'mu_hidden',.3,'seed',p.sensor.seed,'offset_m',0,'duration_s',2,'bumpStart_s',1,'bumpHeight_m',.001,'bumpWidth_m',.01,'scanSpeed_m_s',.1, ...
    'actualK_N_m',p.mechanics.k1,'actualC_N_s_m',p.mechanics.c1,'upperK_N_m',p.mechanics.k2,'upperC_N_s_m',p.mechanics.c2, ...
    'identifiedK_N_m',p.inertial.lowerLoadInverse.identifiedStiffness_N_m,'identifiedC_N_s_m',p.inertial.lowerLoadInverse.identifiedDamping_Ns_m, ...
    'lowerExternalSpringPresent',~finiteCell,'upperExternalSpringPresent',compliantUpper,'upperRigid',p.mechanics.upperRigid,'lowerRigid',false, ...
    'referenceKp_A_N',reference(1),'referenceKi_A_N_s',reference(2),'forceKp_A_N',p.inertial.forceKp,'forceKi_A_N_s',p.inertial.forceKi, ...
    'feedbackCutoff_Hz',reference(3),'feedbackDamping',p.inertial.feedbackFilter.damping,'notchEnabled',logical(reference(4)),'notchPoleDamping',reference(5), ...
    'upperObserverEnabled',compliantUpper,'currentResponse_s',p.coil.currentResponse,'usableVoltage_V',32,'coilR_ohm',p.coil.resistance,'coilL_H',.004, ...
    'identifiedKf_N_A',24.4,'peakCurrent_A',3.5,'midTravel_m',.0125,'travelMin_m',0,'travelMax_m',.025,'initialTravel_m',.0125,'recenterBandRadius_m',.0075, ...
    'knownStaticLowerForce_N',p.d.q10,'knownStaticUpperForce_N',p.d.q20,'initialUpperDeviation_m',0,'gravityAlongAxis_m_s2',p.d.g, ...
    'lowerPair',lowerPair,'upperPair',upperPair);
   if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
   index=index+1;
  end
 end
end
save(fullfile(folder,'configs.mat'),'configs','-v7');prior=jsondecode(fileread(fullfile(source,'manifest.json')));
manifest=struct('status','frozen_development_not_executed','date','2026-10-05','design',designName,'cases',metadata,'sourceModelSha256',prior.sourceModelSha256, ...
 'configsSha256',sha256(fullfile(folder,'configs.mat')),'parentConfigsSha256',sha256(fullfile(source,'configs.mat')), ...
 'gainProfiles',profiles,'profileColumns',{{'referenceKp_A_N','referenceKi_A_N_s','feedbackCutoff_Hz','notchEnabled','notchPoleDamping'}}, ...
 'mechanicalContract',base.mechanicalStudy,'upperObserver',base.upperObserver, ...
 'developmentContract','Eight mass/force/orientation corners plus three interiors; 2s sharp bump at1s, zero robot axial motion/offset, known actual/identified mechanics. Gains are bounded choices selected only on development. Same991000+case noise seeds across gains/designs. No learned policy tuning or fresh qualification.');
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_mechanical_bank.m'));
fprintf('UPPER_MECHANICAL_BANK %s %d cases\n',designName,numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
