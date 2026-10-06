function report=verify_corrected_current_graph(buildFolder,outputFolder)
% Compile/audit a private saved graph without running any physical episode.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
metadata=jsondecode(fileread(fullfile(buildFolder,'metadata.json')));
loadData=load(metadata.configsFile,'configs');configs=loadData.configs;
addpath(fullfile(buildFolder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_accel_lead;
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'),'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
modelFile=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),modelFile);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
indices=1;
for j=2:numel(configs),if configs(j).p.mechanics.mA~=configs(1).p.mechanics.mA,indices(end+1)=j;break;end,end
audits=struct([]);
for index=indices
 p=acf_prepare(configs(index).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 froms=find_system([mdl '/Contact force measurement/Head motion estimator'],'MatchFilter',@Simulink.match.allVariants,'BlockType','From');
 tags=cellfun(@(block)get_param(block,'GotoTag'),froms,'UniformOutput',false);
 allowed={'command_based_robot_acceleration','command_based_robot_velocity','measured_S_acceleration_z','actuator_command'};
 assert(all(ismember(tags,allowed)),'Observer contains an unauthorized source tag.');
 nativeCustom=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','S-Function');assert(isempty(nativeCustom));
 row=struct('scenario',index-1,'massA_kg',p.mechanics.mA,'algebraicLoopCount',loops, ...
  'observerFromTags',{tags},'nativeSFunctionCount',numel(nativeCustom),'physicalEpisodes',0, ...
  'upperObserverEnabled',p.upperObserver.enabled,'loadedEquilibriumICIsZero',true);
 if p.upperObserver.enabled
  o=p.d.upperMotionObserver;assert(o.observabilityRank==4&&o.maximumPoleMagnitude<1);
  assert(o.forcePredictionD==0&&all(o.D(:,2:4)==0,'all'));
  row.observerGain=o.gain;row.estimatorPoleMagnitude=o.maximumPoleMagnitude;
  row.observerStateNames=o.stateNames;row.observerInputNames=o.inputNames;
  row.forcePredictionInputTag='EXPLICIT_NATIVE_INPORT2';row.issuedCommandChain=assert_issued_command_chain(mdl);
  ff=[mdl '/Force controller/Coil PI and PID/Measured motion feedforward'];
  assert(strcmp(get_param([ff '/Guide friction velocity scale'],'Gain'),'1/p.guideFeedforwardStudy.smoothingSpeed_m_s'));
  assert(strcmp(get_param([ff '/Guide Coulomb friction current'],'Gain'),'p.inertial.guideFrictionFeedforwardScale*p.guideFeedforwardStudy.coulombScale*p.coil.coulombForce/p.coil.forceConstant'));
  assert(strcmp(get_param([mdl '/Contact force measurement/Head motion estimator/Measured-frame guide velocity scale'],'Gain'),'1/p.coil.frictionSpeed'));
  row.guideFeedforwardCoulombScale=p.guideFeedforwardStudy.coulombScale;row.guideFeedforwardSmoothingSpeed_m_s=p.guideFeedforwardStudy.smoothingSpeed_m_s;
  row.observerPhysicalFrictionEstimateUnchanged=true;
  row.measurementFrameFrictionHasNoPredictionLead=true;
 else
  row.observerGain=[];row.estimatorPoleMagnitude=[];row.observerStateNames={};row.observerInputNames={};
  row.forcePredictionInputTag='';row.measurementFrameFrictionHasNoPredictionLead=true;
 end
 if isfield(p,'accelLead')&&p.accelLead.enabled
  ff=[mdl '/Force controller/Coil PI and PID/Measured motion feedforward'];
  lead=[ff '/Present-time acceleration lead for current FF'];
  assert(strcmp(get_param(lead,'BlockType'),'DiscreteTransferFcn'));
  assert(strcmp(get_param(lead,'Numerator'),'p.d.accelLead.numerator'));
  assert(strcmp(get_param(lead,'Denominator'),'p.d.accelLead.denominator'));
  assert(strcmp(get_param(lead,'SampleTime'),'p.force.sampleTime')&&strcmp(get_param(lead,'InitialStates'),'0'));
  ph=get_param(lead,'PortHandles');src=get_param(get_param(ph.Inport(1),'Line'),'SrcBlockHandle');
  assert(strcmp(get_param(src,'Name'),'Causal acceleration for current FF'));
  destinations=get_param(get_param(ph.Outport(1),'Line'),'DstBlockHandle');
  assert(numel(destinations)==1&&strcmp(get_param(destinations,'Name'),'Measured acceleration current feedforward'));
  assert(p.inertial.accelerationPrediction_s==0&&abs(p.d.accelLead.pole)<1);
  row.accelerationLeadCurrentFFOnly=true;row.accelerationLeadNumerator=p.d.accelLead.numerator;
  row.accelerationLeadDenominator=p.d.accelLead.denominator;row.accelerationLeadHighFrequencyGain=p.d.accelLead.highFrequencyGain;
 end
 if isempty(audits),audits=row;else,audits(end+1)=row;end
end
assert(strcmp(sha256(fullfile(buildFolder,'acf_rl_native.slx')),metadata.privateModelSha256));
report=struct('status','private_graph_acceptance_complete_no_episodes','createdEpisodes',0, ...
 'sourcePhysicalModelSha256',metadata.sourceModelSha256,'privateControllerGraphSha256',metadata.privateModelSha256, ...
 'configurationsSha256',metadata.configsSha256,'audits',audits, ...
 'emittedCurrentInputContract',metadata.emittedCurrentInputContract, ...
 'scope','Saved private native diagram compilation, no algebraic loops, authorized causal observer tags, stable observable upper estimator, no dynamic actor/plant MATLAB function. These are preparation checks, not force qualification or prototype measurements.');
write_json(fullfile(outputFolder,'acceptance.json'),report);copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'verify_corrected_current_graph.m'));
fprintf('UPPER_GRAPH_ACCEPTANCE %s\n',buildFolder);
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
