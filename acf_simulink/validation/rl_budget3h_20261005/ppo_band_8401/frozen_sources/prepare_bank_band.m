function manifest=prepare_bank_band(folder)
% Preserve the frozen96-case bank exactly, append long edge/drift episodes.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
parent=fullfile(root,'acf_simulink','validation','rl_budget3h_20261005');
if nargin<1,folder=fullfile(parent,'scenario_bank_band');end
assert(~isfolder(folder),'Preserve frozen scenario banks.');mkdir(folder);
oldFolder=fullfile(parent,'scenario_bank');old=load(fullfile(oldFolder,'configs.mat'),'configs');configs=old.configs;
prior=jsondecode(fileread(fullfile(oldFolder,'manifest.json')));metadata=prior.cases;assert(numel(configs)==96);
copyfile(fullfile(oldFolder,'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
base=configs(1).p;offsets=[-.009 -.0075 -.006 .006 .0075 .009];drifts=[-.005 -.002 .002 .005];
masses=[.1 1.4 2];forces=[5 20 50];angles=[0 45 90];appended=struct([]);
for k=1:24
 p=nominal_case(base,masses(mod(k-1,3)+1),forces(mod(floor((k-1)/3),3)+1), ...
 angles(mod(floor((k-1)/9),3)+1),111000+k,offsets(mod(k-1,6)+1),drifts(floor((k-1)/6)+1));
 p.sim.stopTime=4+mod(k,3);p.surface.startTime=.65+.017*mod(29*k,110);
 [configs,metadata,appended]=append(configs,metadata,appended,p,sprintf('train_band_%02d',k),'train','band');
end
selection=[-.0087 -.004 .1 5 0; .0087 .004 2 50 90; -.0073 .002 1.4 20 45; ...
 .0073 -.002 .1 50 45; -.0063 -.005 2 5 90; .0063 .005 1.4 50 0; ...
 -.0093 .005 .1 20 90; .0093 -.005 2 20 0];
for k=1:8
 r=selection(k,:);p=nominal_case(base,r(3),r(4),r(5),121000+k,r(1),r(2));
 p.sim.stopTime=4+mod(k+1,3);p.surface.startTime=.79+.021*mod(31*k,83);
 [configs,metadata,appended]=append(configs,metadata,appended,p,sprintf('selection_band_%02d',k),'selection','band_selection');
end
assert(numel(configs)==128&&isequaln(configs(1:96),old.configs));
save(fullfile(folder,'configs.mat'),'configs','-v7');
% Re-load serialization before accepting the preserved scientific prefix.
reloaded=load(fullfile(folder,'configs.mat'),'configs');assert(isequaln(reloaded.configs(1:96),old.configs));
manifest=prior;manifest.status='frozen_not_executed';manifest.trainCases=72;manifest.selectionCases=24;manifest.holdoutCases=32;
manifest.cases=metadata;manifest.parentConfigsSha256=sha256(fullfile(oldFolder,'configs.mat'));
manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));manifest.original96PreservedExactly=true;
manifest.parentManifestSha256=sha256(fullfile(oldFolder,'manifest.json'));manifest.recenterBandRadius_m=.0075;
manifest.appendedCases=appended;
manifest.appendContract='Original96 parameter structs/IDs and freshholdout64–95 preserved exactly. Append24TRAIN IDs96–119 and8SELECTION IDs120–127,4–6s,initialoffsets near/beyond±7.5mm and issued nominalrobotdrift±2/5mm/s. No appended finalholdout inspection.';
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_bank_band.m'));
fprintf('RL_BAND_BANK_FROZEN %d cases,original96exact\n',numel(configs));
end
function p=nominal_case(base,s,f,a,seed,offset,drift)
p=base;p.mechanics.mS=s;p.force.reference=f;p.mechanics.angle=a*pi/180;p.contact.mu=.3*mod(seed,2);
p.sensor.seed=seed;p.robot.initialPositionOffset_m=offset;p.robot.initialExcessForce=0;p.robot.nominalVelocity=drift;
p.sensor.distalMass_kg=s;p.sensor.distalMass_model_kg=s;p.sensor.distalMassCalibrationScale=1;
p.sim.evaluateFrom=0;p.surface.kind='sharp_bump';p.surface.height=.001;p.surface.width=.01;p.surface.scanSpeed=.1;
end
function [configs,metadata,appended]=append(configs,metadata,appended,p,identity,split,curriculum)
p=acf_prepare(p);assert(p.sim.step==1/300000&&p.force.sampleTime==1e-4);
c=struct('p',p,'identity',identity,'split',split);configs(end+1)=c;
r=metadata(1);r.scenario=numel(configs)-1;r.identity=identity;r.split=split;r.curriculum=curriculum;
r.massS_kg=p.mechanics.mS;r.identifiedMassS_kg=p.sensor.distalMass_model_kg;r.massB_kg=p.mechanics.mB;
r.identifiedMassB_kg=p.uncertainty.massB_model_kg;r.massA_kg=p.mechanics.mA;r.force_N=p.force.reference;
r.angle_rad=p.mechanics.angle;r.mu_hidden=p.contact.mu;r.seed=p.sensor.seed;r.offset_m=p.robot.initialPositionOffset_m;
r.duration_s=p.sim.stopTime;r.bumpStart_s=p.surface.startTime;r.bumpHeight_m=p.surface.height;r.bumpKind=p.surface.kind;
r.repeatPeriod_s=p.surface.repeatPeriod;r.bumpWidth_m=p.surface.width;r.scanSpeed_m_s=p.surface.scanSpeed;
r.actualK_N_m=p.mechanics.k1;r.actualC_N_s_m=p.mechanics.c1;r.identifiedKf_N_A=p.uncertainty.forceConstant_model_N_A;
r.identifiedK_N_m=p.inertial.lowerLoadInverse.identifiedStiffness_N_m;r.identifiedC_N_s_m=p.inertial.lowerLoadInverse.identifiedDamping_Ns_m;
r.coilR_ohm=p.coil.resistance;r.coilL_H=p.coil.inductance;r.usableVoltage_V=p.coil.voltage;r.peakCurrent_A=p.coil.peakCurrent;
r.midTravel_m=p.mechanics.midTravel;r.travelMin_m=p.mechanics.travelMin;r.travelMax_m=p.mechanics.travelMax;
r.knownEquilibriumCurrent_A=(p.force.reference-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A;
metadata(end+1)=r;
a=struct('scenario',r.scenario,'identity',identity,'split',split,'initialOffset_m',r.offset_m, ...
 'initialTravel_m',p.mechanics.midTravel+p.d.u0,'knownNominalRobotVelocity_m_s',p.robot.nominalVelocity, ...
 'recenterBandRadius_m',.0075,'duration_s',r.duration_s);
assert(a.initialTravel_m>p.mechanics.travelMin&&a.initialTravel_m<p.mechanics.travelMax);
if isempty(appended),appended=a;else,appended(end+1)=a;end
end
function s=sha256(file)
fid=fopen(file,'r');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
s=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
