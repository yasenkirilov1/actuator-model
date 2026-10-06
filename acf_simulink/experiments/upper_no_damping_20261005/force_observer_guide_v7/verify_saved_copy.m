function report=verify_saved_copy(buildFolder,outputFolder)
% Reopen a private copy and audit nonzero guide families and explicit force.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
metadata=jsondecode(fileread(fullfile(buildFolder,'metadata.json')));bank=load(metadata.configsFile,'configs');
addpath(fullfile(buildFolder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'),'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
file=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),file);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(file);cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
pSaved=getVariable(get_param(mdl,'ModelWorkspace'),'p');save_system(mdl,file);close_system(mdl,0);load_system(file);
assert(isequaln(pSaved,getVariable(get_param(mdl,'ModelWorkspace'),'p')),'Saved model workspace did not roundtrip.');
parent=[mdl '/Contact force measurement'];recon=[parent '/Contact force reconstruction'];head=[parent '/Head motion estimator'];
rh=get_param(recon,'PortHandles');hh=get_param(head,'PortHandles');
assert(get_param(get_param(hh.Inport(2),'Line'),'SrcPortHandle')==rh.Outport(3));
cellOut=get_param([recon '/measured_lower_force_deviation_N'],'PortHandles');gain=get_param([recon '/Cell error to deviation'],'PortHandles');
assert(get_param(get_param(cellOut.Inport(1),'Line'),'SrcPortHandle')==gain.Outport(1));
assert(strcmp(get_param([recon '/Cell error to deviation'],'Gain'),'-1'));
headIn=get_param([head '/Measured lower force deviation'],'PortHandles');bias=get_param([head '/Cell force deviation from equilibrium'],'PortHandles');
assert(get_param(get_param(bias.Inport(1),'Line'),'SrcPortHandle')==headIn.Outport(1));
assert(getSimulinkBlockHandle([head '/Owned selected cell source wiring log'])<0&&getSimulinkBlockHandle([recon '/Owned raw cell error wiring stimulus'])<0);
assert(isempty(find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','S-Function')));
ff=[mdl '/Force controller/Coil PI and PID/Measured motion feedforward'];
assert(strcmp(get_param([ff '/Guide friction velocity scale'],'Gain'),'1/p.guideFeedforwardStudy.smoothingSpeed_m_s'));
assert(strcmp(get_param([ff '/Guide Coulomb friction current'],'Gain'),'p.inertial.guideFrictionFeedforwardScale*p.guideFeedforwardStudy.coulombScale*p.coil.coulombForce/p.coil.forceConstant'));
rows=struct([]);
for index=[24,35,156,167]
 p=acf_prepare(bank.configs(index).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 assert(p.coil.coulombForce==.2&&p.coil.viscousFriction==2&&p.coil.frictionSpeed==.0002);
 assert(p.mechanics.k2==20000&&p.mechanics.c2==0&&p.mechanics.k1==64e6&&p.mechanics.c1==0);
 row=struct('scenario',index-1,'massA_kg',p.mechanics.mA,'parentObserverProfileID',p.mechanicalStudy.parentObserverProfileID, ...
  'guideCoulombScale',p.guideFeedforwardStudy.coulombScale,'guideSmoothingSpeed_m_s',p.guideFeedforwardStudy.smoothingSpeed_m_s,'algebraicLoopCount',loops);
 if isempty(rows),rows=row;else,rows(end+1)=row;end
end
report=struct('status','private_saved_copy_roundtrip_source_and_loop_acceptance_passed', ...
 'physicalQualificationEpisodes',0,'privateControllerGraphSha256',metadata.privateModelSha256, ...
 'sourcePhysicalModelSha256',metadata.sourceModelSha256,'workspaceSaveReopenEqual',true, ...
 'explicitAcquiredForceSourceChainVerified',true,'temporaryHarnessBlocksAbsent',true,'nativeSFunctionCount',0,'audits',rows);
fid=fopen(fullfile(outputFolder,'acceptance.json'),'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'verify_saved_copy.m'));
fprintf('FORCE_GUIDE_V7_SAVED_COPY_ACCEPTANCE\n');
end
