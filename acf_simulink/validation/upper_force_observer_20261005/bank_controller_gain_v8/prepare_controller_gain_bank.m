function manifest=prepare_controller_gain_bank(folder)
% Last bounded deterministic bank: joint PI multiplier and modal damping.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
recipe=fileparts(mfilename('fullpath'));root=fileparts(fileparts(fileparts(fileparts(recipe))));
source=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005','bank_cell_modal_v6');
parent=jsondecode(fileread(fullfile(source,'manifest.json')));loaded=load(fullfile(source,'configs.mat'),'configs');
assert(strcmp(parent.configsSha256,sha256(fullfile(source,'configs.mat'))));
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
for name={'acf_prepare.m','acf_prepare_cell_modal.m'}
 copyfile(fullfile(recipe,'helpers',name{1}),fullfile(folder,'helpers',name{1}));
end
addpath(fullfile(folder,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal;
baseIndices=22*(17-1)+(1:22);multipliers=[2/3,5/6,1,4/3];damping=[1000,1500,2000];
nominal=loaded.configs(baseIndices(1)).p;
assert(nominal.coil.forceKp==.03*20/24.4&&nominal.coil.forceKi==20*20/24.4);
design='upper_20k_lower_cell_joint_pi_modal_v8';configs=struct([]);metadata=struct([]);families=struct([]);j=0;family=0;
for multiplierID=1:numel(multipliers)
 for dampingID=1:numel(damping)
  family=family+1;gain=multipliers(multiplierID);
  row=struct('familyID',family,'jointPIGainMultiplier',gain,'multiplierID',multiplierID,'dampingID',dampingID, ...
   'referenceKp_A_N',.03*gain,'referenceKi_A_N_s',20*gain, ...
   'executingKp_A_N',nominal.coil.forceKp*gain,'executingKi_A_N_s',nominal.coil.forceKi*gain, ...
   'virtualDamping_Ns_m',damping(dampingID),'filterDamping',.2,'identifiedCurrentResponse_s',50e-6, ...
   'isExactV6ParentReference',gain==1&&damping(dampingID)==1500);
  for b=1:numel(baseIndices)
   base=loaded.configs(baseIndices(b));m=parent.cases(baseIndices(b));p=base.p;j=j+1;
   assert(~isfield(p,'accelLead')&&p.inertial.accelerationPrediction_s==0);
   assert(p.cellModal.enabled&&p.cellModal.filterDamping==.2&&p.cellModal.virtualDamping_Ns_m==1500);
   assert(~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0&&p.mechanics.k1==64e6&&p.mechanics.c1==0);
   assert(p.coil.currentResponse==50e-6&&p.coil.coulombForce==.2&&p.coil.viscousFriction==2&&p.coil.frictionSpeed==.0002);
   p.coil.forceKp=base.p.coil.forceKp*gain;p.coil.forceKi=base.p.coil.forceKi*gain;
   p.cellModal.identifiedCurrentResponse_s=50e-6;p.cellModal.virtualDamping_Ns_m=damping(dampingID);
   p.controllerGainStudy=row;
   p.mechanicalStudy.parentDesign=parent.design;p.mechanicalStudy.design=design;
   p.mechanicalStudy.parentCellModalFamilyID=17;p.mechanicalStudy.controllerGainFamilyID=family;
   p=acf_prepare(p);verify_parity(base.p,p,row.isExactV6ParentReference);
   changed=p;changed.coil.currentResponse=2*p.coil.currentResponse;changed=acf_prepare_cell_modal(changed);
   assert(isequaln(p.d.cellModal,changed.d.cellModal),'Modal controller retuned from actual current response.');
   identity=sprintf('%s_gain%02d_modal%02d_A%.1f_dev%02d',design,multiplierID,dampingID,p.mechanics.mA,m.caseID);
   base.p=p;base.identity=identity;if isempty(configs),configs=base;else,configs(end+1)=base;end
   m.scenario=j-1;m.identity=identity;m.design=design;m.gainID=family;m.controllerGainFamilyID=family;
   m.parentCellModalFamilyID=17;m.parentObserverProfileID=33;
   m.jointPIGainMultiplier=gain;m.executingForceKp_A_N=p.coil.forceKp;m.executingForceKi_A_N_s=p.coil.forceKi;
   m.referenceForceKp_A_N=.03*gain;m.referenceForceKi_A_N_s=20*gain;
   m.virtualCellDamping_Ns_m=damping(dampingID);m.cellModalFilterDamping=.2;m.identifiedModalCurrentResponse_s=50e-6;
   m.noAccelerationLead=true;m.guideFeedforwardSmoothingSpeed_m_s=.0002;m.isExactV6ParentReference=row.isExactV6ParentReference;
   if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
  end
  if isempty(families),families=row;else,families(end+1)=row;end
 end
end
assert(numel(configs)==264&&numel(families)==12&&families(8).isExactV6ParentReference);
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=parent;
manifest.status='frozen_last_bounded_joint_pi_modal_bank_not_executed';manifest.parentDesign=parent.design;manifest.design=design;manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.cellModalFamilies=parent.cellModalFamilies(17);manifest.controllerGainFamilies=families;
manifest.gainProfiles=[[families.jointPIGainMultiplier]',[families.executingKp_A_N]',[families.executingKi_A_N_s]',[families.virtualDamping_Ns_m]'];
manifest.profileColumns={'jointPIGainMultiplier','executingKp_A_N','executingKi_A_N_s','virtualDamping_Ns_m'};
manifest.upperObserver=configs(1).p.upperObserver;manifest.mechanicalContract=configs(1).p.mechanicalStudy;
manifest.exactReferenceFamilyID=8;
manifest.piGainContract=struct('nominalReferenceKp',.03,'nominalReferenceKi',20, ...
 'nominalExecutingKp',nominal.coil.forceKp,'nominalExecutingKi',nominal.coil.forceKi, ...
 'forceConstantRescale',20/24.4,'multipliers',multipliers,'bothPIParametersUseSameMultiplier',true);
manifest.profileIndexContract='zero-based scenario=22*(familyID1-1)+11*massAIndex0+caseID0; familyID1=3*(multiplierID1-1)+dampingID1; outerPIjointmultipliers2/3,5/6,1,4/3,innercv1000,1500,2000; exactV6parent17reference=family8.';
manifest.developmentContract='Last bounded deterministic tuning bank: four joint PI multipliers×three virtual modal damping targets×exact11developmentcases/seeds×bothA masses. No acceleration lead and originalguideFF.2mm/s. Physical/sensing/observer/reconstruction/robot/notch unchanged. Identifiedmodal50µs and acquiredcomplexresponse frozen.264preparedconfigs do not imply qualification; root selects fromdevelopment then locksbeforefresh.';
manifest.scientificParity=struct('allPhysicalSensingObserverRobotNotchUnchanged',true, ...
 'onlyPIGainsAndVirtualModalDampingChange',true,'exactV6ParentReferenceFamilyID',8,'exactReferenceAllPreparedCoefficientsEqual',true, ...
 'modalUsesExplicitIdentifiedCurrentResponse',true,'modalCoefficientsInvariantToActualCurrentResponsePerturbation',true, ...
 'noAccelerationLead',true,'originalGuideFeedforwardUnchanged',true,'createdQualificationEpisodes',0);
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_controller_gain_bank.m'));
fprintf('BOUNDED_CONTROLLER_GAIN_BANK %d cases, %d families; exactreferencefamily8\n',numel(configs),numel(families));
end
function verify_parity(before,after,isReference)
names=setdiff(fieldnames(before),{'d','mechanicalStudy','cellModal','coil'});
for j=1:numel(names),assert(isequaln(before.(names{j}),after.(names{j})),'Existing field %s changed.',names{j});end
assert(isequaln(rmfield(before.coil,{'forceKp','forceKi'}),rmfield(after.coil,{'forceKp','forceKi'})));
assert(isequaln(rmfield(before.cellModal,'virtualDamping_Ns_m'),rmfield(after.cellModal,{'virtualDamping_Ns_m','identifiedCurrentResponse_s'})));
assert(isequaln(rmfield(before.d,{'forceKp','forceKi','cellModal'}),rmfield(after.d,{'forceKp','forceKi','cellModal'})));
assert(isequaln(before.d.cellModal.A,after.d.cellModal.A)&&isequaln(before.d.cellModal.B,after.d.cellModal.B));
assert(isequaln(before.d.upperMotionObserver,after.d.upperMotionObserver));
if isReference
 assert(isequaln(before.coil,after.coil)&&isequaln(before.d,after.d),'Nominal reference no longer matches V6 exactly.');
end
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
