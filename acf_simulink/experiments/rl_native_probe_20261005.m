function report=rl_native_probe_20261005(root)
% Read-only license/API/configuration probe; private copy, no RL changes.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
folder=fullfile(root,'acf_simulink','validation','rl_native_probe_20261005');
assert(~isfolder(folder),'Preserve prior probe.');mkdir(folder);mkdir(fullfile(folder,'cache'));mkdir(fullfile(folder,'code'));
source=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002');
copyfile(fullfile(source,'acf_force_control_adc_trial.slx'),fullfile(folder,'acf_force_control_adc_trial.slx'));
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
addpath(fullfile(folder,'helpers'),'-begin');
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'),'CodeGenFolder',fullfile(folder,'code'),'createDir',true);
mdl='acf_force_control_adc_trial';assert(~bdIsLoaded(mdl));load_system(fullfile(folder,[mdl '.slx']));
cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
products={'MATLAB','SIMULINK','Real-Time_Workshop','RTW_Embedded_Coder','Simulink_Compiler','MATLAB_Coder','Reinforcement_Learning_Toolbox'};
licenses=struct;
for k=1:numel(products)
 try,value=license('test',products{k});message='';catch failure,value=false;message=failure.message;end
 licenses.(matlab.lang.makeValidName(products{k}))=struct('available',logical(value),'message',message);
end
apiNames={'slbuild','rtwbuild','Simulink.BlockDiagram.buildRapidAcceleratorTarget','Simulink.BlockDiagram.modifyTunableParameters', ...
 'Simulink.Simulation','Simulink.SimulationInput','Simulink.ModelReference.ProtectedModel.create','Simulink.BlockDiagram.getInitialState'};
apis=struct([]);
for k=1:numel(apiNames)
 a=struct('name',apiNames{k},'location',which(apiNames{k}));if isempty(apis),apis=a;else,apis(end+1)=a;end
end
configNames={'SystemTargetFile','TargetLang','DefaultParameterBehavior','CodeInterfacePackaging','GenerateSampleERTMain','GRTInterface','MatFileLogging','ProdHWDeviceType','Solver','SolverType','FixedStep','StopTime','SimulationMode','SaveFinalState','SaveCompleteFinalSimState','LoadInitialState','InitialState','SaveTime','SaveOutput','ReturnWorkspaceOutputs'};
config=struct;
for k=1:numel(configNames)
 try,value=get_param(mdl,configNames{k});catch failure,value=['UNAVAILABLE: ' failure.message];end
 config.(configNames{k})=value;
end
compilers=struct('C',evalc("disp(mex.getCompilerConfigurations('C','Selected'))"),'CPP',evalc("disp(mex.getCompilerConfigurations('C++','Selected'))"));
workspace=get_param(mdl,'ModelWorkspace');p=getVariable(workspace,'p');
report=struct('date','2026-10-05','status','read_only_api_license_probe_complete','matlabVersion',version,'installedProducts',ver, ...
 'licenses',licenses,'apis',apis,'config',config,'compilers',compilers,'sourceModelSha256',sha256(fullfile(source,[mdl '.slx'])), ...
 'currentWorkspaceActuator',p.actuator,'currentForceSampleTime_s',p.force.sampleTime,'currentRobotSampleTime_s',p.robot.sampleTime, ...
 'currentPlantStep_s',p.sim.step,'currentAdcSettings',p.adcStudy,'activeLicenses',license('inuse'));
fid=fopen(fullfile(folder,'results.json'),'w');assert(fid~=-1);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
for k=1:numel(apiNames)
 fid=fopen(fullfile(folder,['help_' num2str(k) '.txt']),'w');fprintf(fid,'%s\n',evalc(['help ' apiNames{k}]));fclose(fid);
end
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'rl_native_probe_20261005.m'));
fprintf('RL_NATIVE_PROBE_COMPLETED %s\n',jsonencode(licenses));
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');d.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
