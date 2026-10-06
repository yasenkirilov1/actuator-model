function manifest=prepare_bank(folder)
%PREPARE_BANK Freeze distinct native episode banks before three-hour RL.
% Hidden schedules and actual calibration errors never enter actor context.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
if nargin<1,folder=fullfile(root,'acf_simulink','validation','rl_budget3h_20261005','scenario_bank');end
assert(~isfolder(folder),'Preserve frozen banks.');mkdir(folder);
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
masses=[.1 .5 1 1.4 2 .8];forces=[5 15 30 50 20 40];angles=[0 30 60 90 15 75];
offsets=[-.003 .003 -.0015 .0015 0 -.00075 .00075 .002]*1;
heights=[0 .0001 .0003];
for k=1:12
 p=nominal_case(base,masses(mod(k-1,6)+1),forces(mod(3*k+floor(k/6),6)+1), ...
  angles(mod(5*k+floor(k/6),6)+1),.3*mod(k,2),71000+k,offsets(mod(k-1,8)+1));
 p.sim.stopTime=2;p.surface.startTime=.3+.01*mod(37*k,110);p.surface.height=heights(mod(k-1,3)+1);
 [configs,metadata]=append_case(configs,metadata,p,sprintf('train_curriculum_%02d',k),'train','early');
end
heights=[.0001 .0003 .0006 .001];scales=[.8 .9 1 1.1 1.2];
for j=1:28
 k=j+12;s=masses(mod(5*j+floor(j/6),6)+1);f=forces(mod(j+floor(j/6),6)+1);a=angles(mod(3*j+floor(j/6),6)+1);
 p=nominal_case(base,s,f,a,.3*mod(j,2),72000+j,offsets(mod(3*j,8)+1));
 p=known_mechanics(p,scales(mod(j-1,5)+1),scales(mod(3*j-1,5)+1));
 p.sim.stopTime=2;p.surface.startTime=.3+.01*mod(47*j,110);p.surface.height=heights(mod(j-1,4)+1);
 [configs,metadata]=append_case(configs,metadata,p,sprintf('train_full_%02d',j),'train','full');
end
for j=1:8
 p=hot.configs(mod(j-1,7)+1).p;p.sensor.seed=73000+j;
 p.robot.initialExcessForce=0;p.robot.initialPositionOffset_m=offsets(mod(j+2,8)+1);
 p.sim.stopTime=2;p.sim.evaluateFrom=0;p.surface.startTime=.35+.011*mod(29*j,90);
 if mod(j,2)==0,p.surface.height=.0003;else,p.surface.height=.001;end
 [configs,metadata]=append_case(configs,metadata,p,sprintf('train_stress_%02d',j),'train','stress');
end
% Selection windows are separate episodes, seeds, nuisance timing and context.
selection=[.2 10 15 -.0024;.8 25 75 .0024;1.8 45 45 -.0018;1.2 40 0 .0018; ...
 .1 50 90 .002;2 5 0 -.002;1.4 20 60 -.001;.5 35 30 .001; ...
 2 50 0 0;.1 5 90 0;1.1 32 50 -.0028;.7 18 20 .0028];
for j=1:size(selection,1)
 r=selection(j,:);p=nominal_case(base,r(1),r(2),r(3),.3*mod(j,2),81000+j,r(4));
 if j>8,p=known_mechanics(p,scales(mod(j,5)+1),scales(mod(j+2,5)+1));end
 p.sim.stopTime=2;p.surface.startTime=.41+.013*mod(31*j,72);
 if j<3,p.surface.height=0;elseif j<5,p.surface.height=.0003;else,p.surface.height=.001;end
 [configs,metadata]=append_case(configs,metadata,p,sprintf('selection_nominal_%02d',j),'selection','selection');
end
for j=1:4
 p=hot.configs(mod(2*j,7)+1).p;p.sensor.seed=82000+j;p.robot.initialExcessForce=0;
 p.robot.initialPositionOffset_m=(-1)^j*.0022;p.sim.stopTime=2;p.sim.evaluateFrom=0;p.surface.startTime=.48+.093*j;
 [configs,metadata]=append_case(configs,metadata,p,sprintf('selection_stress_%02d',j),'selection','selection');
end
% Fresh held-out histories. Their timing/seeds are frozen now and never used
% in reward training, checkpoint selection, or policy retuning.
holdout=[.1 5 0 -.0026;2 50 90 .0026;.1 50 0 0;2 5 90 0; ...
 .1 5 90 .0019;2 50 0 -.0019;.1 50 90 -.0029;2 5 0 .0029; ...
 .1 20 45 -.0009;2 20 45 .0009;.5 35 30 -.0023;1.4 20 60 .0023; ...
 1 25 45 -.0013;.3 12 70 .0013;1.7 42 10 -.0017;.9 30 80 .0017; ...
 .6 47 55 -.0021;1.2 7 25 .0021;1.9 38 35 -.0011;.2 16 65 .0011; ...
 1.5 28 5 -.0027;.4 44 85 .0027;1.1 33 50 -.0006;.7 22 40 .0006];
for j=1:24
 r=holdout(j,:);p=nominal_case(base,r(1),r(2),r(3),.3*mod(j,2),91000+j,r(4));
 if j>16,p=known_mechanics(p,scales(mod(2*j,5)+1),scales(mod(3*j,5)+1));end
 p.sim.stopTime=4+mod(j,3);p.surface.startTime=.7+.017*mod(43*j,120);p.surface.height=.001;
 if mod(j,4)==0,p.surface.kind='repeated';p.surface.repeatPeriod=.73+.013*mod(j,9);end
 [configs,metadata]=append_case(configs,metadata,p,sprintf('holdout_nominal_%02d',j),'holdout','final');
end
for j=1:8
 p=hot.configs(mod(3*j,7)+1).p;p.sensor.seed=92000+j;p.robot.initialExcessForce=0;
 p.robot.initialPositionOffset_m=(-1)^j*(.001+.00019*j);p.sim.stopTime=4+mod(j,3);p.sim.evaluateFrom=0;
 p.surface.startTime=.83+.113*j;p.surface.height=.001;
 if mod(j,2)==0,p.surface.kind='repeated';p.surface.repeatPeriod=.91+.037*j;end
 [configs,metadata]=append_case(configs,metadata,p,sprintf('holdout_stress_%02d',j),'holdout','final');
end
assert(numel(configs)==96);
save(fullfile(folder,'configs.mat'),'configs','-v7');
manifest=struct('status','frozen_not_executed','date','2026-10-05','trainCases',48,'selectionCases',16,'holdoutCases',32, ...
 'sourceModelSha256',sha256(fullfile(source,'acf_force_control_adc_trial.slx')), ...
 'nominalInputsSha256',sha256(nominalFile),'hotInputsSha256',sha256(hotFile), ...
 'configsSha256',sha256(fullfile(folder,'configs.mat')),'cases',metadata, ...
 'splitContract','Distinct entire episodes, contexts and seeds; old12 heldout now development only. New32 heldout schedules/seeds never inspected during training or selection.', ...
 'plantContract','Same native Simulink physical graph; hot24.4N/A,8.896ohm,4mH/32V. Matched30kSPS/8kHz sensors;10kHz command acquisition. No physical equations in bridge/Python.', ...
 'curriculumContract','12early flat/0.1/0.3mm;28full0.1/0.3/0.6/1mm knownk/c0.8–1.2times;8calibratedstress. Alltrain/selection2s to include delayed robot consequences. Final32 cases4–6s incl repeated1mm bumps.');
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_bank.m'));
fprintf('RL_BUDGET_BANK_FROZEN %d cases\n',numel(configs));
end
function p=nominal_case(base,s,f,a,mu,seed,offset)
p=base;p.mechanics.mS=s;p.force.reference=f;p.mechanics.angle=a*pi/180;p.contact.mu=mu;
p.sensor.seed=seed;p.robot.initialPositionOffset_m=offset;
p.sensor.distalMass_kg=s;p.sensor.distalMass_model_kg=s;p.sensor.distalMassCalibrationScale=1;p.sim.evaluateFrom=0;
end
function p=known_mechanics(p,kscale,cscale)
p.mechanics.k1=p.mechanics.k1*kscale;p.mechanics.c1=p.mechanics.c1*cscale;
p.springs.lower.rate_N_m=p.mechanics.k1/2;
p.inertial.lowerLoadInverse.identifiedStiffness_N_m=p.mechanics.k1;
p.inertial.lowerLoadInverse.identifiedDamping_Ns_m=p.mechanics.c1;
end
function [configs,metadata]=append_case(configs,metadata,p,identity,split,curriculum)
p=acf_prepare(p);assert(p.sim.step==1/300000&&p.force.sampleTime==1e-4);
c=struct('p',p,'identity',identity,'split',split);if isempty(configs),configs=c;else,configs(end+1)=c;end
row=struct('scenario',numel(configs)-1,'identity',identity,'split',split,'curriculum',curriculum, ...
 'massS_kg',p.mechanics.mS,'identifiedMassS_kg',p.sensor.distalMass_model_kg, ...
 'massB_kg',p.mechanics.mB,'identifiedMassB_kg',p.uncertainty.massB_model_kg,'massA_kg',p.mechanics.mA, ...
 'force_N',p.force.reference,'angle_rad',p.mechanics.angle,'mu_hidden',p.contact.mu,'seed',p.sensor.seed, ...
 'offset_m',p.robot.initialPositionOffset_m,'duration_s',p.sim.stopTime, ...
 'bumpStart_s',p.surface.startTime,'bumpHeight_m',p.surface.height,'bumpKind',p.surface.kind,'repeatPeriod_s',p.surface.repeatPeriod, ...
 'bumpWidth_m',p.surface.width,'scanSpeed_m_s',p.surface.scanSpeed, ...
 'actualK_N_m',p.mechanics.k1,'actualC_N_s_m',p.mechanics.c1, ...
 'identifiedKf_N_A',p.uncertainty.forceConstant_model_N_A, ...
 'identifiedK_N_m',p.inertial.lowerLoadInverse.identifiedStiffness_N_m, ...
 'identifiedC_N_s_m',p.inertial.lowerLoadInverse.identifiedDamping_Ns_m, ...
 'coilR_ohm',p.coil.resistance,'coilL_H',p.coil.inductance,'usableVoltage_V',p.coil.voltage, ...
 'peakCurrent_A',p.coil.peakCurrent,'midTravel_m',p.mechanics.midTravel, ...
 'travelMin_m',p.mechanics.travelMin,'travelMax_m',p.mechanics.travelMax, ...
 'knownEquilibriumCurrent_A',(p.force.reference-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A);
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
