function manifest=prepare_qualification_700(folder)
% Freeze fresh nominal fullrange grid; no execution/checkpoint selection.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
parent=fullfile(root,'acf_simulink','validation','rl_budget3h_20261005');
if nargin<1,folder=fullfile(parent,'qualification_bank_700');end
assert(~isfolder(folder),'Preserve frozen qualification banks.');mkdir(folder);
source=fullfile(parent,'scenario_bank');prior=load(fullfile(source,'configs.mat'),'configs');base=prior.configs(1).p;
assert(base.coil.forceConstant==24.4&&base.coil.resistance==8.896&&base.coil.inductance==.004&&base.coil.voltage==32);
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
base.robot.nominalVelocity=0;base.robot.errorForce=0;base.robot.initialExcessForce=0;base.robot.initialPositionOffset_m=0;
base.sim.stopTime=2;base.sim.evaluateFrom=0;base.surface.kind='sharp_bump';base.surface.startTime=1;
base.surface.height=.001;base.surface.width=.01;base.surface.scanSpeed=.1;
masses=[.1 .3 .5 .7 .9 1.1 1.3 1.5 1.7 2];forces=5:5:50;angles=0:15:90;
configs=struct([]);metadata=struct([]);timer=tic;
for s=masses
 for f=forces
  for a=angles
   k=numel(configs);p=base;p.mechanics.mS=s;p.force.reference=f;p.mechanics.angle=a*pi/180;p.contact.mu=.3;
   p.sensor.seed=970000+k;p.sensor.distalMass_kg=s;p.sensor.distalMass_model_kg=s;p.sensor.distalMassCalibrationScale=1;
   p=acf_prepare(p);assert(p.sim.step==1/300000&&p.force.sampleTime==1e-4);
   identity=sprintf('qualification_nominal_%03d',k);row=struct('p',p,'identity',identity,'split','qualification');
   if isempty(configs),configs=row;else,configs(end+1)=row;end
   r=struct('scenario',k,'identity',identity,'split','qualification','curriculum','final_nominal_grid', ...
    'massS_kg',p.mechanics.mS,'identifiedMassS_kg',p.sensor.distalMass_model_kg,'massB_kg',p.mechanics.mB, ...
    'identifiedMassB_kg',p.uncertainty.massB_model_kg,'massA_kg',p.mechanics.mA,'force_N',p.force.reference, ...
    'angle_rad',p.mechanics.angle,'mu_hidden',p.contact.mu,'seed',p.sensor.seed,'offset_m',0,'duration_s',2, ...
    'bumpStart_s',1,'bumpHeight_m',.001,'bumpKind',p.surface.kind,'repeatPeriod_s',p.surface.repeatPeriod, ...
    'bumpWidth_m',.01,'scanSpeed_m_s',.1,'actualK_N_m',p.mechanics.k1,'actualC_N_s_m',p.mechanics.c1, ...
    'identifiedKf_N_A',p.uncertainty.forceConstant_model_N_A,'identifiedK_N_m',p.inertial.lowerLoadInverse.identifiedStiffness_N_m, ...
    'identifiedC_N_s_m',p.inertial.lowerLoadInverse.identifiedDamping_Ns_m,'coilR_ohm',p.coil.resistance, ...
    'coilL_H',p.coil.inductance,'usableVoltage_V',p.coil.voltage,'peakCurrent_A',p.coil.peakCurrent, ...
    'midTravel_m',p.mechanics.midTravel,'travelMin_m',p.mechanics.travelMin,'travelMax_m',p.mechanics.travelMax, ...
    'knownEquilibriumCurrent_A',(p.force.reference-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A, ...
    'knownNominalRobotVelocity_m_s',0,'robotErrorForce_N',0,'initialTravel_m',p.mechanics.midTravel,'recenterBandRadius_m',.0075);
   if isempty(metadata),metadata=r;else,metadata(end+1)=r;end
  end
 end
end
assert(numel(configs)==700);save(fullfile(folder,'configs.mat'),'configs','-v7');
old=jsondecode(fileread(fullfile(source,'manifest.json')));
manifest=struct('status','frozen_not_executed','date','2026-10-05','cases',metadata,'qualificationCases',700, ...
 'sourceModelSha256',old.sourceModelSha256,'parentConfigsSha256',sha256(fullfile(source,'configs.mat')), ...
 'configsSha256',sha256(fullfile(folder,'configs.mat')),'preparationWall_s',toc(timer), ...
 'gridContract','10S masses0.1–2kg×10force5:5:50N×7fixedangles0:15:90deg;onefreshsensorseed970000+IDpercase.2s,1mm10mmsharpbumpat1s scanned100mm/s. Purebump: axialnominalpath/positionerror/initialoffsetzero. IdentifiedSmassequalsactualknownS;othernominalknownparameterscorrect.', ...
 'qualificationContract','NEVER usedforrewardtraining orcheckpointselection. Do notexecuteuntilwinningweightsandselectiondecisionlocked. Finite nominalgrid,onefreshseedperpoint;notphysicalprototypevalidation orcalibrationboxproof.', ...
 'plantContract','Nativefrozenb370physics/acquisition;hot24.4N/A,8.896ohm,4mH,32V;30kSPSmatched8kHzADC/10kHzcontroller. NoPythonphysics.');
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_qualification_700.m'));
fprintf('RL_QUALIFICATION_GRID_FROZEN %d cases %.3fs,noexecution\n',numel(configs),manifest.preparationWall_s);
end
function s=sha256(file)
fid=fopen(file,'r');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
s=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
