function manifest=prepare_cell_modal_bank_v6(folder,phaseFile)
% Bounded virtual damping bank on selected observers and the mandatory plant.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005','bank_observer_noise_v4');
parent=jsondecode(fileread(fullfile(source,'manifest.json')));loaded=load(fullfile(source,'configs.mat'),'configs');
phase=jsondecode(fileread(phaseFile));assert(strcmp(phase.status,'native_delivered_cell_acquisition_phase_measured'));
assert(phase.qualificationEpisodes==0&&strcmp(phase.parentConfigSha256,parent.configsSha256));carrier=phase.rows(2);
recipe=fileparts(mfilename('fullpath'));copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
for name={'acf_prepare.m','acf_prepare_cell_modal.m'}
 copyfile(fullfile(recipe,'cell_modal_v6',name{1}),fullfile(folder,'helpers',name{1}));
end
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal;
observers=[24,33];widths=[.05,.1,.2];damping=[500,1500,3000];design='upper_20k_lower_cell_active_modal_damping_v6';
configs=struct([]);metadata=struct([]);families=struct([]);j=0;family=0;
for observerIndex=1:numel(observers)
 observerID=observers(observerIndex);baseIndices=22*(observerID-1)+(1:22);
 for widthID=1:numel(widths)
  for dampingID=1:numel(damping)
   family=family+1;r=struct('familyID',family,'parentObserverProfileID',observerID, ...
    'widthID',widthID,'dampingID',dampingID,'filterDamping',widths(widthID), ...
    'virtualDamping_Ns_m',damping(dampingID),'currentBound_A',.05,'controller',struct());
   for b=1:numel(baseIndices)
    base=loaded.configs(baseIndices(b));m=parent.cases(baseIndices(b));p=base.p;j=j+1;
    assert(~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0&&p.mechanics.k1==64e6&&p.mechanics.c1==0);
    p.cellModal=struct('enabled',true,'carrier_Hz',carrier.frequency_Hz,'filterDamping',widths(widthID), ...
     'virtualDamping_Ns_m',damping(dampingID),'currentBound_A',.05, ...
     'acquiredTransferReal',carrier.measuredTransferReal,'acquiredTransferImag',carrier.measuredTransferImag, ...
     'phaseSourceSha256',sha256(phaseFile),'contract','Zero-DC proper causal resonant cell feedback, after current notch before current limits; full measurement frame/noise/physical mechanics preserved.');
    p.mechanicalStudy.parentDesign=parent.design;p.mechanicalStudy.design=design;
    p.mechanicalStudy.parentObserverProfileID=observerID;p.mechanicalStudy.cellModalFamilyID=family;
    p=acf_prepare(p);verify_parity(base.p,p);
    identity=sprintf('%s_obs%02d_modal%02d_A%.1f_dev%02d',design,observerID,family,p.mechanics.mA,m.caseID);
    base.p=p;base.identity=identity;if isempty(configs),configs=base;else,configs(end+1)=base;end
    m.scenario=j-1;m.identity=identity;m.design=design;m.gainID=family;m.cellModalFamilyID=family;
    m.parentObserverProfileID=observerID;m.cellModalWidthID=widthID;m.cellModalDampingID=dampingID;
    m.virtualCellDamping_Ns_m=damping(dampingID);m.cellModalFilterDamping=widths(widthID);
    m.modalAuxiliaryCurrentBound_A=.05;m.cellModalCarrier_Hz=carrier.frequency_Hz;
    if isempty(metadata),metadata=m;else,metadata(end+1)=m;end
    if b==1,r.controller=p.d.cellModal;end
   end
   if isempty(families),families=r;else,families(end+1)=r;end
  end
 end
end
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=parent;
manifest.status='frozen_bounded_cell_modal_control_bank_not_executed';manifest.parentDesign=parent.design;manifest.design=design;manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.observerProfiles=parent.observerProfiles(observers);manifest.cellModalFamilies=families;
manifest.gainProfiles=[[families.parentObserverProfileID]',[families.filterDamping]',[families.virtualDamping_Ns_m]'];
manifest.profileColumns={'parentObserverProfileID','cellModalFilterDamping','virtualCellDamping_Ns_m'};
manifest.upperObserver=configs(1).p.upperObserver;manifest.mechanicalContract=configs(1).p.mechanicalStudy;
manifest.phaseHarnessFile=phaseFile;manifest.phaseHarnessSha256=sha256(phaseFile);
manifest.profileIndexContract='zero-based scenario =198*observerIndex0 +22*(modalLocalFamilyID1-1)+11*massAIndex0+caseID0; observer0/1=parent24/33. localFamilyID1=3*(widthID1-1)+dampingID1; gainID=globalfamily1=9*observerIndex0+localfamily1.';
manifest.developmentContract='18 bounded active modal damping families × exact11 development cases/seeds × bothA masses. Existing noise/physical damping/friction/upper20k and lower finite64MN/m unchanged. Baseline notch/PI unchanged; new current feedback bypasses notch within±.05A and original current limit. No fresh-test tuning, preview, current sensor or training.396prepared cases do not imply execution.';
manifest.scientificParity=struct('allExistingPhysicalSensorControllerSectionsUnchanged',true, ...
 'allExistingPreparedCoefficientsIncludingUpperObserverUnchanged',true, ...
 'forceReconstructionAndObserverMeasurementFrameUnchanged',true,'soleNewFunction','Bounded phase-shaped active cell-mode current feedback', ...
 'createdQualificationEpisodes',0);
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_cell_modal_bank_v6.m'));
fprintf('BOUNDED_CELL_MODAL_BANK %d cases, %d families\n',numel(configs),numel(families));
end
function verify_parity(before,after)
names=setdiff(fieldnames(before),{'d','mechanicalStudy'});
for j=1:numel(names),assert(isequaln(before.(names{j}),after.(names{j})),'Existing field %s changed.',names{j});end
assert(isequaln(before.d,rmfield(after.d,'cellModal')),'An existing prepared coefficient changed.');
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
