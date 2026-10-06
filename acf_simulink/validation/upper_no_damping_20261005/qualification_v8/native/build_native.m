function metadata=build_qualification_native_v8(folder,configsFile)
% Compile the selected saved native graph, with new default/bank parameters.
% This owns only private copies. No instrumenter or physical solver is added.
assert(~usejava('desktop'));maxNumCompThreads(2);
bankFolder=fileparts(configsFile);manifest=jsondecode(fileread(fullfile(bankFolder,'manifest.json')));
assert(strcmp(manifest.status,'qualification_parameters_frozen_not_executed'));
assert(strcmp(sha256(configsFile),manifest.configsSha256));
assert(strcmp(sha256(manifest.sourceNativeModel.path),manifest.sourceNativeModel.sha256));
lock=jsondecode(fileread(fullfile(bankFolder,'candidate_lock.json')));
assert(lock.selectionOnly&&~lock.qualificationOutcomesInspected);
assert(~isfolder(folder),'Preserve all exported native banks.');mkdir(folder);
modelFile=fullfile(folder,'acf_rl_native.slx');copyfile(manifest.sourceNativeModel.path,modelFile);
copyfile(fullfile(bankFolder,'helpers'),fullfile(folder,'helpers'));
addpath(fullfile(folder,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal acf_prepare_accel_lead;
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'),'CodeGenFolder',fullfile(folder,'code'),'createDir',true);
storage=Simulink.sdi.getStorageLocation;record=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_qualification_sdi'),'w');fprintf(fid,'Owned qualification export.\n');fclose(fid);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,storage,record,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(false);
loaded=load(configsFile,'configs');p=acf_prepare(loaded.configs(1).p);workspace=get_param(mdl,'ModelWorkspace');
assert(numel(loaded.configs)==736&&isequaln(p.d,loaded.configs(1).p.d));assignin(workspace,'p',p);
proportional=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'Name','Normal force proportional current');
integral=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'Name','Normal force integral current');
assert(numel(proportional)==1&&strcmp(get_param(proportional{1},'Gain'),'-p.inertial.forceKp'));
assert(numel(integral)==1&&strcmp(get_param(integral{1},'Gain'),'-p.inertial.forceKi'));
sourceGuard=verify_local_modal_source_v8(mdl,p,folder);
loopChecks=struct([]);checkCases=[1,700,701,725,726,732,733,736];
for j=1:numel(checkCases)
 index=checkCases(j);prepared=acf_prepare(loaded.configs(index).p);assignin(workspace,'p',prepared);
 set_param(mdl,'SimulationCommand','update');count=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(count==0);
 row=struct('scenario',index-1,'identity',loaded.configs(index).identity,'algebraicLoopCount',count);
 if isempty(loopChecks),loopChecks=row;else,loopChecks(end+1)=row;end
end
assert(isempty(find_system(mdl,'LookUnderMasks','all','FollowLinks','on','Regexp','on','Name','^Owned qualification')));
assignin(workspace,'p',p);
set_param(mdl,'SystemTargetFile','grt.tlc','TargetLang','C','DefaultParameterBehavior','Tunable', ...
 'CodeInterfacePackaging','Nonreusable function','GRTInterface','off','MatFileLogging','off', ...
 'GenerateReport','off','GenCodeOnly','on','SignalLogging','off','SaveTime','on','SaveOutput','on', ...
 'SaveFormat','Dataset','FixedStep',num2str(p.sim.step,17),'Solver','ode4','SimulationMode','normal', ...
 'StopTime','inf','LoadExternalInput','off');
save_system(mdl,modelFile);
metadata=jsondecode(fileread(fullfile(lock.sourceNativeDirectory,'metadata.json')));
metadata.status='building_qualification_native_v8';metadata.configsFile=configsFile;metadata.configsSha256=sha256(configsFile);
metadata.modelFile=modelFile;metadata.sourceModelSha256=manifest.sourceModelSha256;
metadata.selectedSourceNativeModel=manifest.sourceNativeModel;metadata.candidateLockSha256=manifest.candidateLockSha256;
metadata.contextRecipeSha256=manifest.contextRecipeSha256;metadata.selectedSourceScenario=lock.selectedScenario;
metadata.selectedSourceIdentity=lock.selectedIdentity;metadata.localSourceGuard=sourceGuard;
metadata.algebraicLoopCount=0;metadata.algebraicLoopChecks=loopChecks;
metadata.executingNormalForceControllerGains=lock.executingNormalForceControllerGains;
metadata.qualificationEpisodesExecuted=0;metadata.baselineEvaluationInputRequired=1;
metadata.compileContract='Selected saved graph copied; known-tool/context coefficients prepared privately, frozen modal identification. New default is nominal grid0, exactCoder/bankcase0 guard required. No Python physics or candidate selection.';
write_json(fullfile(folder,'metadata.json'),metadata);
timer=tic;
try,slbuild(mdl);metadata.buildElapsed_s=toc(timer);metadata.status='generated_qualification_native_c';
catch failure
 metadata.status='qualification_codegen_rejected';metadata.error=getReport(failure,'extended','hyperlinks','off');write_json(fullfile(folder,'metadata.json'),metadata);rethrow(failure);
end
metadata.privateModelSha256=sha256(modelFile);write_json(fullfile(folder,'metadata.json'),metadata);
export_native_parameter_bank_v8(mdl,loaded.configs,folder);
mapped=jsondecode(fileread(fullfile(folder,'scenarios.json')));rows=manifest.cases;
for j=1:numel(rows)
 assert(rows(j).scenario==j-1&&strcmp(rows(j).identity,mapped(j).identity));
 for name=fieldnames(mapped)',rows(j).(name{1})=mapped(j).(name{1});end
end
write_json(fullfile(folder,'scenarios.json'),rows);
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'build_native.m'));
copyfile(fullfile(fileparts(mfilename('fullpath')),'export_native_parameter_bank_v8.m'),fullfile(folder,'export_native_parameter_bank_v8.m'));
save(fullfile(folder,'prepared_configs.mat'),'p','-v7');
fprintf('QUALIFICATION_V8_NATIVE_EXPORTED %d cases,%.3fs,noqualificationepisodes\n',numel(rows),metadata.buildElapsed_s);
end

function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_qualification_sdi')),try,rmdir(sdi,'s');catch,end,end
end

function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end

function write_json(path,value)
fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
