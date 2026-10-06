function report=build_runnable_fixed_model_v8(nativeFolder,configsFile,outputFolder)
% Versioned, directly runnable native baseline; original model stays intact.
assert(~usejava('desktop'));maxNumCompThreads(2);
assert(~isfolder(outputFolder),'Preserve every earlier runnable preparation.');mkdir(outputFolder);
helperFolder=fullfile(outputFolder,'helpers');mkdir(helperFolder);
sourceNames={'acf_prepare','acf_adc_prepare','acf_prepare_baseline_v8','acf_prepare_upper_observer', ...
 'acf_prepare_cell_modal','acf_select_actuator','acf_select_controller'};
targetNames={'acf_fixed_prepare','acf_fixed_adc_prepare','acf_fixed_prepare_baseline_v8','acf_fixed_prepare_upper_observer', ...
 'acf_fixed_prepare_cell_modal','acf_fixed_select_actuator','acf_fixed_select_controller'};
helperRows=cell(size(sourceNames));
for j=1:numel(sourceNames)
 source=fullfile(nativeFolder,'helpers',[sourceNames{j} '.m']);text=fileread(source);mapped=rename_functions(text,sourceNames,targetNames);
 assert(strcmp(rename_functions(mapped,targetNames,sourceNames),text),'Only function identifiers may change.');
 target=fullfile(helperFolder,[targetNames{j} '.m']);write_text(target,mapped);
 helperRows{j}=struct('sourceFile',source,'sourceSha256',sha256(source),'targetFile',target,'targetSha256',sha256(target),'inverseNamespaceTransformationExact',true);
end
copyfile(fullfile(fileparts(mfilename('fullpath')),'acf_fixed_apply.m'),helperFolder);
addpath(helperFolder,'-begin');
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'), ...
 'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
mdl='acf_force_control_fixed_upper_20261005';modelFile=fullfile(outputFolder,[mdl '.slx']);
copyfile(fullfile(nativeFolder,'acf_rl_native.slx'),modelFile);assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
assert(strcmp(get_param(mdl,'Name'),mdl));
bank=load(configsFile,'configs');p=bank.configs(1).p;
p.mechanics.mS=1.4;p.force.reference=20;p.mechanics.angle=pi/4;p.sensor.seed=761400;p.sim.stopTime=2;
p=acf_fixed_prepare(p);
assert(isequaln(p.d.cellModal,bank.configs(1).p.d.cellModal));
mw=get_param(mdl,'ModelWorkspace');mw.clear;assignin(mw,'p',p);assignin(mw,'pDefault',p);assignin(mw,'pCompliant',p);

% Baseline current always follows the native force controller. There is no
% external action switch or dummy action source in the standalone graph.
coil=[mdl '/Force controller/Coil PI and PID'];
remove_input_line([coil '/Optional structural-mode current notch'],1);
delete_block([coil '/Baseline or external current']);
delete_block([coil '/RL baseline mode']);delete_block([coil '/RL full current request']);
add_line(coil,'PI PID and feedforward current/1','Optional structural-mode current notch/1','autorouting','on');
robot=[mdl '/Recenter travel'];
remove_input_line([robot '/Send correction only when settled'],1);
robotRemoved={'Baseline or external robot','RL baseline mode','RL robot correction request', ...
 'RL fresh 100 Hz robot request','RL hard sign and gain envelope','RL negative recenter envelope','RL positive recenter envelope'};
for j=1:numel(robotRemoved),delete_block([robot '/' robotRemoved{j}]);end
add_line(robot,'Recenter gain/1','Send correction only when settled/1','autorouting','on');
rootRemoved={'rl_current_request_A','rl_robot_correction_m_s','rl_baseline_mode','RL input tag 1','RL input tag 2','RL input tag 3', ...
 'ADC acquisition diagnostics','Inertial sensor recordings'};
for j=1:numel(rootRemoved),delete_block([mdl '/' rootRemoved{j}]);end

% The three ordinary native diagnostic outputs replace redundant historical
% Workspace log branches; their18/27/3 channel interfaces stay unchanged.
logs=find_system(mdl,'SearchDepth',1,'IncludeCommented','on','BlockType','ToWorkspace');for j=1:numel(logs),delete_block(logs{j});end
pruned=prune_logging_leaves(mdl);
rename_block([mdl '/RL rl_observation'],'Measured signals');rename_block([mdl '/RL rl_scoring'],'Physical performance');
rename_block([mdl '/RL rl_comparator'],'Controller commands');
rename_block([mdl '/rl_observation'],'measured_signals');rename_block([mdl '/rl_scoring'],'physical_performance');
rename_block([mdl '/rl_comparator'],'controller_commands');
blocks=find_system(mdl,'LookUnderMasks','all','FollowLinks','off','IncludeCommented','on', ...
 'MatchFilter',@Simulink.match.allVariants,'Type','block');
callbackFields={'MaskInitialization','MaskDisplay','OpenFcn','InitFcn','StartFcn','StopFcn','LoadFcn','DeleteFcn','CopyFcn'};
for j=1:numel(blocks)
 for k=1:numel(callbackFields)
  try,text=get_param(blocks{j},callbackFields{k});catch,continue,end
  if ischar(text)&&~isempty(text)
   mapped=rename_functions(text,sourceNames,targetNames);if ~strcmp(text,mapped),set_param(blocks{j},callbackFields{k},mapped);end
  end
 end
 mask=Simulink.Mask.get(blocks{j});
 if ~isempty(mask)
  for k=1:numel(mask.Parameters)
   text=mask.Parameters(k).Callback;
   if ~isempty(text),mask.Parameters(k).Callback=rename_functions(text,sourceNames,targetNames);end
  end
 end
 try,set_param(blocks{j},'ForegroundColor','black','BackgroundColor','white');catch,end
end
set_param(mdl,'PostLoadFcn',"addpath(fullfile(fileparts(get_param(bdroot,'FileName')),'helpers'));", ...
 'InitFcn','acf_fixed_apply(bdroot);','PreLoadFcn','', ...
 'LoadExternalInput','off','ExternalInput','', ...
 'StopTime','2','FixedStep',num2str(p.sim.step,17),'Solver','ode4', ...
 'SaveOutput','on','SaveFormat','Dataset','OutputSaveName','yout', ...
 'SaveTime','on','TimeSaveName','tout','ReturnWorkspaceOutputs','on', ...
 'SignalLogging','off','LimitDataPoints','off','SimulationMode','normal','ScreenColor','white');
acf_fixed_apply(mdl);
assert(isempty(find_system(mdl,'SearchDepth',1,'BlockType','Inport')));
assert(isempty(find_system(mdl,'LookUnderMasks','all','BlockType','From','GotoTag','rl_baseline_mode')));
assert(isempty(find_system(mdl,'LookUnderMasks','all','BlockType','From','GotoTag','rl_external_current')));
assert(isempty(find_system(mdl,'LookUnderMasks','all','BlockType','From','GotoTag','rl_external_robot')));

systems={mdl,coil,robot,[mdl '/Contact force measurement/Head motion estimator'],[coil '/Cell mode damping']};
for j=1:numel(systems)
 Simulink.BlockDiagram.arrangeSystem(systems{j});set_param(systems{j},'ZoomFactor','FitSystem');
end
annotations=find_system(mdl,'SearchDepth',1,'FindAll','on','Type','annotation');
for j=1:numel(annotations)
 try,text=get_param(annotations(j),'Name');if contains(upper(text),'ACTIVE CONTACT FLANGE'),delete(annotations(j));end;catch,end
end
annotation=Simulink.Annotation(mdl,sprintf(['ACTIVE SANDING FLANGE | FIXED MECHANICS DEVELOPMENT MODEL\n' ...
 'Voice coil; A1.5 kg, B0.6 kg, default S1.4 kg. Upper20 N/mm, zero external damping.\n' ...
 'Lower link: rigid attachment with finite64 MN/m load-cell compliance.\n' ...
 'Native PI, measured-inertia feedforward and bounded modal damping. Edit Model Workspace p.']));
annotation.Position=[30,-130,890,-35];annotation.ForegroundColor='black';
set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
save_system(mdl,modelFile);
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'build_runnable_fixed_model_v8.m'));
report=struct('status','private_runnable_native_model_prepared_pending_acceptance', ...
 'modelFile',modelFile,'modelSha256',sha256(modelFile),'sourceModelFile',fullfile(nativeFolder,'acf_rl_native.slx'), ...
 'sourceModelSha256',sha256(fullfile(nativeFolder,'acf_rl_native.slx')),'configsSha256',sha256(configsFile), ...
 'canonicalChanged',false,'externalInputNeeded',false,'baselineHardSelected',true, ...
 'physicalPlantNative',true,'algebraicLoopCount',loops,'toolMass_kg',1.4,'holderMass_kg',.6,'bodyMass_kg',1.5, ...
 'defaultForce_N',20,'defaultOrientation_deg',45,'defaultSeed',761400, ...
 'normalForceKp_A_N',p.inertial.forceKp,'normalForceKi_A_N_s',p.inertial.forceKi, ...
 'modalVirtualDamping_Ns_m',p.cellModal.virtualDamping_Ns_m, ...
 'namespaceOnlyHelperCopies',[helperRows{:}],'deletedRootBlocks',{rootRemoved}, ...
 'deletedRobotExternalBranches',{robotRemoved},'prunedLoggingLeaves',{pruned}, ...
 'diagnosticInterfaceWidths',[18,27,3],'qualificationEpisodes',0);
write_json(fullfile(outputFolder,'preparation.json'),report);fprintf('RUNNABLE_FIXED_MODEL_PREPARED %s\n',jsonencode(report));
end
function mapped=rename_functions(text,from,to)
mapped=text;for j=1:numel(from),mapped=regexprep(mapped,['\<' from{j} '\>'],to{j});end
end
function rename_block(block,name),set_param(block,'Name',name);end
function remove_input_line(block,index)
h=get_param(block,'PortHandles');line=get_param(h.Inport(index),'Line');if line~=-1,delete_line(line);end
end
function removed=prune_logging_leaves(mdl)
removed={};types={'From','Mux','Selector','ZeroOrderHold','Gain','Constant'};
for iteration=1:100
 changed=false;lines=find_system(mdl,'SearchDepth',1,'IncludeCommented','on','FindAll','on','Type','line');
 for j=1:numel(lines)
  try,d=get_param(lines(j),'DstPortHandle');if isempty(d)||all(d<0),delete_line(lines(j));end;catch,end
 end
 blocks=find_system(mdl,'SearchDepth',1,'IncludeCommented','on','Type','block');
 for j=1:numel(blocks)
  if ~ismember(get_param(blocks{j},'BlockType'),types),continue,end
  h=get_param(blocks{j},'PortHandles');if isempty(h.Outport),continue,end
  connected=false;for k=1:numel(h.Outport),connected=connected||get_param(h.Outport(k),'Line')~=-1;end
  if ~connected,removed{end+1}=blocks{j};delete_block(blocks{j});changed=true;end %#ok<AGROW>
 end
 if ~changed,break,end
end
end
function write_text(file,value),fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',value);fclose(fid);end
function write_json(file,value),write_text(file,[jsonencode(value,'PrettyPrint',true) newline]);end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
