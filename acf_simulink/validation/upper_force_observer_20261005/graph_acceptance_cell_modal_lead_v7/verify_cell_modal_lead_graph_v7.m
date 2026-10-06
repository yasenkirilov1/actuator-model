function report=verify_cell_modal_lead_graph_v7(buildFolder,outputFolder)
% Independent saved-copy compilation; no qualification episode is created.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
metadata=jsondecode(fileread(fullfile(buildFolder,'metadata.json')));
loadData=load(metadata.configsFile,'configs');configs=loadData.configs;assert(numel(configs)==352);
assert(strcmp(sha256(metadata.configsFile),metadata.configsSha256));
assert(strcmp(sha256(fullfile(buildFolder,'acf_rl_native.slx')),metadata.privateModelSha256));
addpath(fullfile(buildFolder,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal acf_prepare_accel_lead;
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'),'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
modelFile=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),modelFile);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
% Both modal parents, both A masses, both guide settings and lead extremes.
indices=[1,12,23,34,133,144,155,166,177,188,199,210,309,320,331,342];
head=[mdl '/Contact force measurement/Head motion estimator'];
recon=[mdl '/Contact force measurement/Contact force reconstruction'];
coil=[mdl '/Force controller/Coil PI and PID'];modal=[coil '/Cell mode damping'];
source=[recon '/Send acquired cell for modal control'];tag='cell_modal_acquired_deltaL_N';
assert(strcmp(get_param(source,'GotoTag'),tag)&&strcmp(get_param(source,'TagVisibility'),'global'));
gotos=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','Goto','GotoTag',tag);
assert(numel(gotos)==1&&strcmp(gotos{1},source));
assert_source(source,1,[recon '/Cell error to deviation']);
froms=find_system(modal,'MatchFilter',@Simulink.match.allVariants,'BlockType','From');
assert(numel(froms)==1&&strcmp(get_param(froms{1},'GotoTag'),tag));
filter=[modal '/Cell modal filter'];bound=[modal '/Damping current bound'];
assert(strcmp(get_param(filter,'BlockType'),'DiscreteStateSpace'));
for coefficient={'A','B','C','D'}
 assert(strcmp(get_param(filter,coefficient{1}),['p.d.cellModal.' coefficient{1}]));
end
assert(strcmp(get_param(filter,'InitialCondition'),'zeros(2,1)'));
assert(strcmp(get_param(filter,'SampleTime'),'p.force.sampleTime'));
assert_source(filter,1,[modal '/Acquired axial cell deviation']);
assert_source(bound,1,filter);assert_source([modal '/Damping current'],1,bound);
assert(strcmp(get_param(bound,'UpperLimit'),'p.cellModal.currentBound_A'));
assert(strcmp(get_param(bound,'LowerLimit'),'-p.cellModal.currentBound_A'));
sumBlock=[coil '/Notched command plus modal damping'];assert(strcmp(get_param(sumBlock,'Inputs'),'++'));
assert_source(sumBlock,1,[coil '/Optional structural-mode current notch']);assert_source(sumBlock,2,modal);
assert_source([coil '/Current limits'],1,sumBlock);assert_source([coil '/Current saturation error'],2,sumBlock);
assert_source([coil '/Current saturation error'],1,[coil '/Current limits']);
ff=[coil '/Measured motion feedforward'];lead=[ff '/Present-time acceleration lead for current FF'];
assert(strcmp(get_param(lead,'BlockType'),'DiscreteTransferFcn'));
assert(strcmp(get_param(lead,'Numerator'),'p.d.accelLead.numerator'));
assert(strcmp(get_param(lead,'Denominator'),'p.d.accelLead.denominator'));
assert(strcmp(get_param(lead,'SampleTime'),'p.force.sampleTime')&&strcmp(get_param(lead,'InitialStates'),'0'));
assert_source(lead,1,[ff '/Causal acceleration for current FF']);
assert_source([ff '/Measured acceleration current feedforward'],1,lead);
assert(strcmp(get_param([ff '/Guide friction velocity scale'],'Gain'),'1/p.cellModalLead.guideFrictionFeedforwardSpeed_m_s'));
nativeCustom=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','S-Function');assert(isempty(nativeCustom));
owned=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'RegExp','on','Name','^Owned.*');assert(isempty(owned));
audits=struct([]);
for index=indices
 p=acf_prepare(configs(index).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 froms=find_system(head,'MatchFilter',@Simulink.match.allVariants,'BlockType','From');
 tags=cellfun(@(block)get_param(block,'GotoTag'),froms,'UniformOutput',false);
 allowed={'command_based_robot_acceleration','command_based_robot_velocity','measured_S_acceleration_z'};
 assert(all(ismember(tags,allowed)),'Observer contains an unauthorized source tag.');
 assert_source(head,2,[mdl '/Contact force measurement/Contact force reconstruction']);
 assert_source([recon '/measured_lower_force_deviation_N'],1,[recon '/Cell error to deviation']);
 assert(strcmp(get_param([head '/Measured lower force deviation'],'Port'),'2'));
 o=p.d.upperMotionObserver;assert(o.observabilityRank==4&&o.maximumPoleMagnitude<1);
 assert(all(o.D(:,2:4)==0,'all'));
 assert(all(real(eig(o.measuredForceDerivativeA))<0)&&o.measuredForceDerivativeD>0);
 assert(p.mechanics.k2==20000&&p.mechanics.c2==0&&p.mechanics.k1==64e6&&p.mechanics.c1==0);
 assert(strcmp(get_param([head '/Cell force deviation from equilibrium'],'Bias'),'0'));
 assert(strcmp(get_param([head '/Measured holder acceleration inertia'],'Gain'),'p.uncertainty.massB_model_kg'));
 assert(strcmp(get_param([head '/Measured net actuator force deviation'],'Inputs'),'+-+'));
 assert(strcmp(get_param([head '/Head gap and velocity observer'],'C'),'eye(4)'));
 assert(strcmp(get_param([head '/Head gap and velocity observer'],'D'),'zeros(4,4)'));
 d=p.d.cellModal;assert(p.cellModal.enabled&&p.cellModal.currentBound_A==.05);
 assert(abs(p.d.accelLead.pole)<1&&abs(p.d.accelLead.steadyGain-1)<1e-12);
 assert(p.coil.frictionSpeed==.0002&&p.coil.coulombForce==.2&&p.coil.viscousFriction==2);
 assert(isequal(size(d.A),[2,2])&&isequal(size(d.B),[2,1])&&isequal(size(d.C),[1,2])&&isscalar(d.D));
 assert(all(isfinite([d.A(:);d.B(:);d.C(:);d.D(:)]))&&max(abs(eig(d.A)))<1);
 assert(abs(d.C*((eye(2)-d.A)\d.B)+d.D)<1e-12);
 assert(abs(d.normalizedPositiveDampingReal-1)<1e-12&&abs(d.normalizedPositiveDampingImag)<1e-12);
 assert(p.accelLead.enabled&&p.inertial.accelerationPrediction_s==0);
 assert(p.cellModal.identifiedCurrentResponse_s==50e-6);
 changed=p;changed.coil.currentResponse=2*p.coil.currentResponse;changed=acf_prepare_cell_modal(changed);
 assert(isequaln(p.d.cellModal,changed.d.cellModal));
 row=struct('scenario',index-1,'massA_kg',p.mechanics.mA,'algebraicLoopCount',loops, ...
  'parentObserverProfileID',p.mechanicalStudy.parentObserverProfileID,'parentModalFamilyID',p.mechanicalStudy.parentCellModalFamilyID,'modalLeadFamilyID',p.mechanicalStudy.cellModalLeadFamilyID, ...
  'observerFromTags',{tags},'observerGain',o.gain,'observerPoleMagnitude',o.maximumPoleMagnitude, ...
  'regularizedDerivativeCutoff_Hz',p.upperObserver.forceDerivativeCutoff_Hz, ...
  'guideFrictionFeedforwardSpeed_m_s',p.cellModalLead.guideFrictionFeedforwardSpeed_m_s, ...
  'accelerationAdvanceTime_s',p.accelLead.advanceTime_s,'accelerationLeadPoleTime_s',p.accelLead.poleTime_s, ...
  'identifiedModalCurrentResponse_s',p.cellModal.identifiedCurrentResponse_s,'modalFilterDamping',p.cellModal.filterDamping,'virtualCellDamping_Ns_m',p.cellModal.virtualDamping_Ns_m, ...
  'modalPoleMagnitude',d.maximumPoleMagnitude,'modalDcCurrentGain',d.dcCurrentGain,'auxiliaryBound_A',.05, ...
  'normalizedPositiveDampingReal',d.normalizedPositiveDampingReal,'normalizedPositiveDampingImag',d.normalizedPositiveDampingImag, ...
  'modalSourceTag',tag,'sourceExplicitlyGlobalAndUnique',true,'currentAfterNotchBeforeLimit',true, ...
  'antiwindupUsesTotalPrelimitRequest',true,'nativeSFunctionCount',0,'createdEpisodes',0);
 if isempty(audits),audits=row;else,audits(end+1)=row;end
 write_json(fullfile(outputFolder,'acceptance_progress.json'),struct('status','running_saved_copy_compilation','audits',audits));
end
guard=jsondecode(fileread(fullfile(buildFolder,'local_cell_source_guard.json')));
assert(guard.modalSourceVerified&&numel(guard.rows)==3);
assert(all([guard.rows.maximumAbsoluteWiringError_N]<1e-12)&&all([guard.rows.maximumModalSourceError_N]<1e-12));
phase=jsondecode(fileread(fullfile(buildFolder,'modal_positive_damping_guard.json')));
assert(strcmp(phase.status,'native_modal_positive_damping_sign_and_phase_passed')&&numel(phase.rows)==2);
assert(all(abs([phase.rows.gainRatio]-1)<.03)&&all(abs([phase.rows.phaseError_deg])<3));
assert(all([phase.rows.meanVirtualBModePower_W]<0));
emitted=jsondecode(fileread(fullfile(buildFolder,'generated_cell_source_audit.json')));
assert(emitted.modalStateInputCount==2&&emitted.antiwindupUsesTotalPrelimitCurrent);
assert(strcmp(emitted.nativeCSha256,sha256(fullfile(buildFolder,'code','acf_rl_native_grt_rtw','acf_rl_native.c'))));
assert(strcmp(sha256(fullfile(buildFolder,'acf_rl_native.slx')),metadata.privateModelSha256));
assert(strcmp(sha256(modelFile),metadata.privateModelSha256));
report=struct('status','private_modal_lead_guide_graph_acceptance_complete_no_episodes','createdEpisodes',0, ...
 'sourcePhysicalModelSha256',metadata.sourceModelSha256,'privateControllerGraphSha256',metadata.privateModelSha256, ...
 'configurationsSha256',metadata.configsSha256,'audits',audits,'algebraicLoopCount',0, ...
 'sourceNumericalGuardFile',fullfile(buildFolder,'local_cell_source_guard.json'), ...
 'positiveDampingGuardFile',fullfile(buildFolder,'modal_positive_damping_guard.json'), ...
 'emittedSourceAuditFile',fullfile(buildFolder,'generated_cell_source_audit.json'), ...
 'scope','Independent saved-copy compilation of both modal parents, both A masses, both guide FF smoothing settings and lead extremes; authorized existing acquired-cell source, stable two-state proper zero-DC modal filter, bounded post-notch pre-limit actuation, total-request antiwindup, unchanged observer measurement frame, no custom MATLAB/S-function dynamics or diagnostic-current controller input. Guards validate wiring and carrier phase only; no force qualification or hardware robustness inferred.');
write_json(fullfile(outputFolder,'acceptance.json'),report);
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'verify_cell_modal_lead_graph_v7.m'));
fprintf('CELL_MODAL_GRAPH_ACCEPTANCE %s\n',buildFolder);
end
function assert_source(destination,input,source)
ports=get_param(destination,'PortHandles');line=get_param(ports.Inport(input),'Line');assert(line>0);
assert(get_param(line,'SrcBlockHandle')==get_param(source,'Handle'));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
