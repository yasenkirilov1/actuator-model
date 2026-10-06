function manifest=prepare_bank(folder)
%PREPARE_BANK Freeze whole-episode splits before reward training.
% No truth parameter, bump schedule or comparator action enters the policy.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
if nargin<1,folder=fullfile(root,'acf_simulink','validation','rl_unified_20261005','scenario_bank');end
assert(~isfolder(folder),'Preserve previous frozen scenario banks.');mkdir(folder);
source=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
nominalFile=fullfile(root,'acf_simulink','validation','adc_budget_range_20261002_part1','input_configs.mat');
nominal=load(nominalFile,'configs');base=nominal.configs(1).p;
assert(base.adcStudy.outputRate_Hz==30000&&base.adcStudy.analogCutoff_Hz==8000);
base.coil.forceConstant=24.4;base.coil.resistance=8.896;base.coil.inductance=.004;base.coil.voltage=32;
base.uncertainty.forceConstant_model_N_A=24.4;base.inertial.lowerLoadInverse.identifiedForceConstant_N_A=24.4;
for field={'forceKp','forceKi','forceKd'},base.inertial.(field{1})=base.inertial.(field{1})*20/24.4;end
base.robot.nominalVelocity=0;base.robot.initialExcessForce=0;
hotFile=fullfile(root,'acf_simulink','validation','budget_adc_hot_coil_20261002_inputs', ...
 'ADXL1002_8000_M08_30k_current100_hot32V_configs.mat');hot=load(hotFile,'configs');
configs=struct([]);metadata=struct([]);
% Training is a deliberately small RL attempt, not the historical full grid.
masses=[.1 .5 1 1.4 2];forces=[5 15 30 50];angles=[0 30 60 90];
offsets=[-1.5 1.5 -.75 .75 0 2]*1e-3;
for k=1:24
 s=masses(mod(k-1,5)+1);f=forces(mod(floor((k-1)/3),4)+1);a=angles(mod(k-1,4)+1);
 p=nominal_case(base,s,f,a,.3*mod(k,2),4100+k,offsets(mod(k-1,6)+1));
 p.sim.stopTime=.8;p.surface.startTime=.17+.013*mod(7*k,31);
 if mod(k,8)==0,p.surface.height=.0005;end
 [configs,metadata]=append_case(configs,metadata,p,sprintf('train_%02d',k),'train');
end
selection=[.2 10 15 -.0012; .8 25 75 .0012; 1.8 45 45 -.0008; 1.2 40 0 .0008; ...
 .1 50 90 .0015;2 5 0 -.0015;1.4 20 60 0;.5 35 30 .001];
for k=1:size(selection,1)
 r=selection(k,:);p=nominal_case(base,r(1),r(2),r(3),.3*mod(k,2),5100+k,r(4));
 p.sim.stopTime=.9;p.surface.startTime=.21+.027*mod(5*k,13);
 [configs,metadata]=append_case(configs,metadata,p,sprintf('selection_%02d',k),'selection');
end
holdout=[.1 5 0 -.0015;2 50 90 .0015;1 25 45 .001;1.4 20 60 -.001; ...
 .5 35 30 .0015;2 5 90 -.0015;.1 50 0 0;1.4 50 90 0];
for k=1:size(holdout,1)
 r=holdout(k,:);p=nominal_case(base,r(1),r(2),r(3),.3*mod(k,2),6100+k,r(4));
 p.sim.stopTime=4;p.surface.startTime=2;
 [configs,metadata]=append_case(configs,metadata,p,sprintf('holdout_nominal_%02d',k),'holdout');
end
for k=1:4
 p=hot.configs(2*k-1).p;p.sensor.seed=6200+k;
 p.robot.initialExcessForce=0;p.robot.initialPositionOffset_m=(-1)^k*.001;
 p.sim.stopTime=4;p.sim.evaluateFrom=0;p.surface.startTime=2;
 [configs,metadata]=append_case(configs,metadata,p,sprintf('holdout_stress_%02d',k),'holdout');
end
save(fullfile(folder,'configs.mat'),'configs','-v7');
manifest=struct('status','frozen_not_executed','date','2026-10-05','trainCases',24,'selectionCases',8,'holdoutCases',12, ...
 'sourceModelSha256',sha256(fullfile(source,'acf_force_control_adc_trial.slx')), ...
 'nominalInputsSha256',sha256(nominalFile),'hotInputsSha256',sha256(hotFile), ...
 'configsSha256',sha256(fullfile(folder,'configs.mat')),'cases',metadata, ...
 'splitContract','Complete distinct episodes and seeds. Training resets may choose new sensor seeds. No heldout selection or retuning.', ...
 'plantContract','Prepared parameters for the same native physical graph; hot 24.4N/A,8.896ohm,4mH coil and32V usable output; ADS131M08 30kSPS/8kHz chain. Actual truth/bump timing excluded from actor context.');
write_json(fullfile(folder,'manifest.json'),manifest);
save(fullfile(folder,'base_and_hot.mat'),'base','hot','-v7');
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_bank.m'));
fprintf('RL_BANK_FROZEN %d cases\n',numel(configs));
end

function p=nominal_case(base,s,f,a,mu,seed,offset)
p=base;p.mechanics.mS=s;p.force.reference=f;p.mechanics.angle=a*pi/180;p.contact.mu=mu;
p.sensor.seed=seed;p.robot.initialPositionOffset_m=offset;
p.sensor.distalMass_kg=s;p.sensor.distalMass_model_kg=s;p.sensor.distalMassCalibrationScale=1;
p.sim.evaluateFrom=0;
end

function [configs,metadata]=append_case(configs,metadata,p,identity,split)
p=acf_prepare(p);assert(p.sim.step==1/300000&&p.force.sampleTime==1e-4);
c=struct('p',p,'identity',identity,'split',split);
if isempty(configs),configs=c;else,configs(end+1)=c;end
row=struct('scenario',numel(configs)-1,'identity',identity,'split',split, ...
 'massS_kg',p.mechanics.mS,'identifiedMassS_kg',p.sensor.distalMass_model_kg, ...
 'massB_kg',p.mechanics.mB,'massA_kg',p.mechanics.mA,'force_N',p.force.reference, ...
 'angle_rad',p.mechanics.angle,'mu_hidden',p.contact.mu,'seed',p.sensor.seed, ...
 'offset_m',p.robot.initialPositionOffset_m,'duration_s',p.sim.stopTime, ...
 'bumpStart_s',p.surface.startTime,'bumpHeight_m',p.surface.height, ...
 'bumpWidth_m',p.surface.width,'scanSpeed_m_s',p.surface.scanSpeed, ...
 'identifiedKf_N_A',p.uncertainty.forceConstant_model_N_A, ...
 'identifiedK_N_m',p.inertial.lowerLoadInverse.identifiedStiffness_N_m, ...
 'identifiedC_N_s_m',p.inertial.lowerLoadInverse.identifiedDamping_Ns_m, ...
 'coilR_ohm',p.coil.resistance,'coilL_H',p.coil.inductance,'usableVoltage_V',p.coil.voltage, ...
 'peakCurrent_A',p.coil.peakCurrent,'midTravel_m',p.mechanics.midTravel, ...
 'travelMin_m',p.mechanics.travelMin,'travelMax_m',p.mechanics.travelMax, ...
 'knownEquilibriumCurrent_A',(p.force.reference-(p.mechanics.mB+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A);
if isempty(metadata),metadata=row;else,metadata(end+1)=row;end
end

function s=sha256(file)
fid=fopen(file,'r');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
s=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end

function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
