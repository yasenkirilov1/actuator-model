function manifest=prepare_cell_modal_lead_bank_v7(folder)
% Selected V6 modal parents, causal acceleration FF lead and guide FF smooth.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005','bank_cell_modal_v6');
parent=jsondecode(fileread(fullfile(source,'manifest.json')));loaded=load(fullfile(source,'configs.mat'),'configs');
assert(strcmp(parent.configsSha256,sha256(fullfile(source,'configs.mat'))));
recipe=fileparts(mfilename('fullpath'));copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
for name={'acf_prepare.m','acf_prepare_cell_modal.m','acf_prepare_accel_lead.m','install_accel_lead.m'}
 copyfile(fullfile(recipe,'cell_modal_lead_v7',name{1}),fullfile(folder,'helpers',name{1}));
end
addpath(fullfile(folder,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal acf_prepare_accel_lead;
selectedParents=[17,18];leadPairs=1e-6*[150,400;250,400;350,1000;500,1000];guideSpeeds=[.0002,.001];
design='upper_20k_lower_cell_modal_lead_guide_v7';configs=struct([]);metadata=struct([]);families=struct([]);j=0;family=0;
for parentIndex=1:numel(selectedParents)
 parentID=selectedParents(parentIndex);baseIndices=22*(parentID-1)+(1:22);
 for leadID=1:size(leadPairs,1)
  for guideID=1:numel(guideSpeeds)
   family=family+1;r=leadPairs(leadID,:);
   row=struct('familyID',family,'parentCellModalFamilyID',parentID,'parentObserverProfileID',33, ...
    'leadID',leadID,'guideID0',guideID-1,'advanceTime_s',r(1),'poleTime_s',r(2), ...
    'guideFrictionFeedforwardSpeed_m_s',guideSpeeds(guideID),'virtualDamping_Ns_m',[], ...
    'modalFilterDamping',.2,'discreteLeadNumerator',[],'discreteLeadDenominator',[], ...
    'leadHighFrequencyGain',[],'identifiedCurrentResponse_s',50e-6);
   for b=1:numel(baseIndices)
    base=loaded.configs(baseIndices(b));m=parent.cases(baseIndices(b));p=base.p;j=j+1;
    assert(p.cellModal.enabled&&p.cellModal.filterDamping==.2&&p.inertial.accelerationPrediction_s==0);
    assert(~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0&&p.mechanics.k1==64e6&&p.mechanics.c1==0);
    assert(p.coil.currentResponse==50e-6&&p.coil.coulombForce==.2&&p.coil.viscousFriction==2&&p.coil.frictionSpeed==.0002);
    p.cellModal.identifiedCurrentResponse_s=50e-6;
    p.accelLead=struct('enabled',true,'advanceTime_s',r(1),'poleTime_s',r(2),'sampleTime_s',p.force.sampleTime);
    p.cellModalLead=struct('guideFrictionFeedforwardSpeed_m_s',guideSpeeds(guideID), ...
     'contract','Controller Coulomb FF velocity scale only. Physical guide/observer friction and acquisition unchanged.');
    p.mechanicalStudy.parentDesign=parent.design;p.mechanicalStudy.design=design;
    p.mechanicalStudy.parentCellModalFamilyID=parentID;p.mechanicalStudy.cellModalLeadFamilyID=family;
    p=acf_prepare(p);verify_parity(base.p,p);
    changed=p;changed.coil.currentResponse=2*p.coil.currentResponse;changed=acf_prepare_cell_modal(changed);
    assert(isequaln(p.d.cellModal,changed.d.cellModal),'Modal coefficients retuned from actual driver response.');
    identity=sprintf('%s_modal%02d_lead%02d_guide%02d_A%.1f_dev%02d',design,parentID,leadID,guideID-1,p.mechanics.mA,m.caseID);
    base.p=p;base.identity=identity;if isempty(configs),configs=base;else,configs(end+1)=base;end
    m.scenario=j-1;m.identity=identity;m.design=design;m.gainID=family;m.cellModalLeadFamilyID=family;
    m.parentCellModalFamilyID=parentID;m.parentObserverProfileID=33;m.accelerationLeadID=leadID;
    m.accelerationAdvanceTime_s=r(1);m.accelerationLeadPoleTime_s=r(2);
    m.accelerationLeadHighFrequencyGain=p.d.accelLead.highFrequencyGain;
    m.accelerationLeadNumerator=p.d.accelLead.numerator;m.accelerationLeadDenominator=p.d.accelLead.denominator;
    m.guideFeedforwardSmoothingSpeed_m_s=guideSpeeds(guideID);m.guideFeedforwardCoulombScale=1;
    m.identifiedModalCurrentResponse_s=50e-6;
    if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
    row.virtualDamping_Ns_m=p.cellModal.virtualDamping_Ns_m;
    row.discreteLeadNumerator=p.d.accelLead.numerator;row.discreteLeadDenominator=p.d.accelLead.denominator;
    row.leadHighFrequencyGain=p.d.accelLead.highFrequencyGain;
   end
   if isempty(families),families=row;else,families(end+1)=row;end
  end
 end
end
assert(numel(configs)==352&&numel(families)==16);save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=parent;
manifest.status='frozen_bounded_modal_lead_guide_bank_not_executed';manifest.parentDesign=parent.design;manifest.design=design;manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.cellModalFamilies=parent.cellModalFamilies(selectedParents);manifest.cellModalLeadFamilies=families;
manifest.gainProfiles=[[families.parentCellModalFamilyID]',[families.advanceTime_s]',[families.poleTime_s]',[families.guideFrictionFeedforwardSpeed_m_s]'];
manifest.profileColumns={'parentCellModalFamilyID','accelerationAdvanceTime_s','accelerationLeadPoleTime_s','guideFrictionFeedforwardSpeed_m_s'};
manifest.upperObserver=configs(1).p.upperObserver;manifest.mechanicalContract=configs(1).p.mechanicalStudy;
manifest.profileIndexContract='zero-based scenario=176*parentIndex0+44*(leadID1-1)+22*guideID0+11*massAIndex0+caseID0; parentIndex0=0/1 selects V6modal17/18; guideID0=0/1 selects FFsmooth0.2/1mm/s; globalfamily=8*parentIndex0+2*(leadID1-1)+guideID0+1.';
manifest.developmentContract='Two V6 modal parents17/18 × four accepted proper causal Saccel-currentFF leads × two controller-only Coulomb FF smoothing speeds × exact11cases/seeds × bothA masses.352prepared configurations are not executed qualification. All physical/noise/observer/reconstruction/timing/baselinePI/notch/gates unchanged. No truth, preview, current sensor, learning or fresh-test tuning.';
manifest.scientificParity=struct('allExistingPhysicalSensorControllerSectionsUnchanged',true, ...
 'allExistingPreparedCoefficientsIncludingUpperObserverUnchanged',true, ...
 'nominalModalCoefficientsExactlyEqualV6Parents',true,'modalUsesExplicitIdentifiedCurrentResponse',true, ...
 'modalCoefficientsInvariantToActualCurrentResponsePerturbation',true, ...
 'forceReconstructionAndObserverMeasurementFrameUnchanged',true, ...
 'newFunctions','Proper causal current-FF Saccel lead and controller-only friction FF velocity scale', ...
 'createdQualificationEpisodes',0);
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_cell_modal_lead_bank_v7.m'));
fprintf('BOUNDED_CELL_MODAL_LEAD_BANK %d cases, %d families\n',numel(configs),numel(families));
end
function verify_parity(before,after)
names=setdiff(fieldnames(before),{'d','mechanicalStudy','cellModal'});
for j=1:numel(names),assert(isequaln(before.(names{j}),after.(names{j})),'Existing field %s changed.',names{j});end
assert(isequaln(before.cellModal,rmfield(after.cellModal,'identifiedCurrentResponse_s')));
assert(isequaln(before.d,rmfield(after.d,'accelLead')),'An existing prepared coefficient changed.');
assert(isequaln(before.d.cellModal,after.d.cellModal),'Nominal modal coefficient changed from V6 parent.');
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
