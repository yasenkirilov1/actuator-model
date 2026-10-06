function report=verify_force_observer_graph_cellsource_v2(buildFolder,outputFolder)
% Independent reopened private graph audit, without a physical episode.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
metadata=jsondecode(fileread(fullfile(buildFolder,'metadata.json')));
loadData=load(metadata.configsFile,'configs');configs=loadData.configs;
addpath(fullfile(buildFolder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'),'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
modelFile=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),modelFile);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
indices=1;
for j=2:numel(configs),if configs(j).p.mechanics.mA~=configs(1).p.mechanics.mA,indices(end+1)=j;break;end,end
audits=struct([]);sys=[mdl '/Contact force measurement/Head motion estimator'];
for index=indices
 p=acf_prepare(configs(index).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 froms=find_system(sys,'MatchFilter',@Simulink.match.allVariants,'BlockType','From');
 tags=cellfun(@(block)get_param(block,'GotoTag'),froms,'UniformOutput',false);
 allowed={'command_based_robot_acceleration','command_based_robot_velocity','measured_S_acceleration_z'};
 assert(all(ismember(tags,allowed)),'Observer contains an unauthorized source tag.');
 assert(~ismember('measured_force_N',tags)&&~ismember('actuator_command',tags));
 parent=[mdl '/Contact force measurement'];recon=[parent '/Contact force reconstruction'];
 hp=get_param(sys,'PortHandles');localLine=get_param(hp.Inport(2),'Line');
 assert(strcmp(get_param(get_param(localLine,'SrcBlockHandle'),'Name'),'Contact force reconstruction'));
 op=get_param([recon '/measured_lower_force_deviation_N'],'PortHandles');sourceLine=get_param(op.Inport(1),'Line');
 assert(strcmp(get_param(get_param(sourceLine,'SrcBlockHandle'),'Name'),'Cell error to deviation'));
 assert(strcmp(get_param([sys '/Measured lower force deviation'],'Port'),'2'));
 nativeCustom=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','S-Function');assert(isempty(nativeCustom));
 o=p.d.upperMotionObserver;assert(o.observabilityRank==4&&o.maximumPoleMagnitude<1);
 assert(all(o.D(:,2:4)==0,'all'));
 assert(all(real(eig(o.measuredForceDerivativeA))<0)&&o.measuredForceDerivativeD>0);
 assert(p.mechanics.k2==20000&&p.mechanics.c2==0&&p.mechanics.k1==64e6&&p.mechanics.c1==0);
 assert(strcmp(get_param([sys '/Cell force deviation from equilibrium'],'Bias'),'0'));
 assert(strcmp(get_param([sys '/Measured holder acceleration inertia'],'Gain'),'p.uncertainty.massB_model_kg'));
 assert(strcmp(get_param([sys '/Measured net actuator force deviation'],'Inputs'),'+-+'));
 assert(strcmp(get_param([sys '/Head gap and velocity observer'],'C'),'eye(4)'));
 assert(strcmp(get_param([sys '/Head gap and velocity observer'],'D'),'zeros(4,4)'));
 row=struct('scenario',index-1,'massA_kg',p.mechanics.mA,'algebraicLoopCount',loops, ...
  'observerFromTags',{tags},'nativeSFunctionCount',numel(nativeCustom),'physicalEpisodes',0, ...
  'observerGain',o.gain,'estimatorPoleMagnitude',o.maximumPoleMagnitude, ...
  'observerStateNames',{o.stateNames},'observerInputNames',{o.inputNames}, ...
  'identifiedHolderMass_kg',o.identifiedHolderMass_kg,'identifiedCellStiffness_N_m',o.identifiedCellStiffness_N_m, ...
  'regularizedDerivativeCutoff_Hz',p.upperObserver.forceDerivativeCutoff_Hz,'regularizedDerivativeDamping',1, ...
  'regularizedDerivativeHighFrequencyCorrectionGain',o.measuredForceDerivativeScale*o.measuredForceDerivativeD, ...
  'additionalAcquisitionStages',0,'issuedCurrentPredictorRemoved',true,'truthUsedByObserver',false,'explicitLocalCellSourceWire',true);
 if isempty(audits),audits=row;else,audits(end+1)=row;end
end
assert(strcmp(sha256(fullfile(buildFolder,'acf_rl_native.slx')),metadata.privateModelSha256));
guard=jsondecode(fileread(fullfile(buildFolder,'local_cell_source_guard.json')));
assert(strcmp(guard.status,'live_local_cell_source_sign_and_nonzero_guard_passed'));
assert(numel(guard.rows)==3&&all([guard.rows.maximumAbsoluteWiringError_N]<1e-12));
report=struct('status','private_graph_acceptance_complete_no_episodes','createdEpisodes',0, ...
 'sourcePhysicalModelSha256',metadata.sourceModelSha256,'privateControllerGraphSha256',metadata.privateModelSha256, ...
 'configurationsSha256',metadata.configsSha256,'audits',audits,'liveLocalSourceNumericalGuard',guard, ...
 'scope','Saved-copy compilation for both A masses, no algebraic loops, authorized existing sensor/issued-robot sources, stable observable upper estimator, proper causal measured-force derivative, no custom MATLAB/S-function dynamics. Preparation checks do not constitute force qualification or hardware measurements.');
write_json(fullfile(outputFolder,'acceptance.json'),report);copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'verify_force_observer_graph_cellsource_v2.m'));
fprintf('UPPER_FORCE_GRAPH_ACCEPTANCE %s\n',buildFolder);
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
