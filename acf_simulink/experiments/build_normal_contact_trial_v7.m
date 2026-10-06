function report=build_normal_contact_trial_v7(config)
%BUILD_NORMAL_CONTACT_TRIAL_V7 Optional acceleration-current FF low-pass.
if nargin<1,config=struct;end
if ~isfield(config,'sourceFolder'),config.sourceFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v6-20260930/frozen-inertial-v6';end
if ~isfield(config,'workFolder'),config.workFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v7-20260930';end
assert(~usejava('desktop') && ~strcmp(config.sourceFolder,config.workFolder));
for n={'','helpers','cache','code','sdi','traces'}
 f=fullfile(config.workFolder,n{1});if ~isfolder(f),mkdir(f);end
end
copyfile(fullfile(config.sourceFolder,'acf_force_control_normal_trial.slx'),fullfile(config.workFolder,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(config.sourceFolder,'helpers','*.m'),fullfile(config.workFolder,'helpers'));
patch_prepare(fullfile(config.workFolder,'helpers','acf_prepare.m'));
patch_runner(fullfile(config.workFolder,'helpers','acf_run.m'));
addpath(fullfile(config.workFolder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator acf_select_controller;
Simulink.fileGenControl('set','CacheFolder',fullfile(config.workFolder,'cache'), ...
 'CodeGenFolder',fullfile(config.workFolder,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(config.workFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(config.workFolder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error');
z=load(fullfile(config.sourceFolder,'normal_contact_trial_candidate.mat'),'p');p=z.p;
p.inertial.accelerationFeedforwardFilter_s=0;
p=acf_prepare(p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
page=[mdl '/Force controller/Coil PI and PID/Measured motion feedforward'];
target=[page '/Measured acceleration current feedforward'];ports=get_param(target,'PortHandles');
delete_line(get_param(ports.Inport(1),'Line'));
set_param(target,'Position',[1400 30 1600 85]);
set_param([page '/acceleration_current_A'],'Position',[1750 50 1780 64]);
add_block('simulink/Discrete/Discrete Transfer Fcn',[page '/Acceleration FF command filter'], ...
 'Position',[1110 30 1320 85],'Numerator','p.d.accelerationFeedforwardFilter.numerator', ...
 'Denominator','p.d.accelerationFeedforwardFilter.denominator','InitialStates','0', ...
 'SampleTime','p.force.sampleTime','FontSize','11');
add_line(page,'Causal acceleration for current FF/1','Acceleration FF command filter/1','autorouting','on');
add_line(page,'Acceleration FF command filter/1','Measured acceleration current feedforward/1','autorouting','on');
set_param([page '/Acceleration FF audit vector'],'Inputs','3','Position',[1200 740 1205 900]);
add_line(page,'Acceleration FF command filter/1','Acceleration FF audit vector/3','autorouting','on');
Simulink.BlockDiagram.routeLine(find_system(page,'FindAll','on','SearchDepth',1,'Type','line'));
set_param(mdl,'SimulationCommand','update');loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);assert(isempty(loops));
save_system(mdl,fullfile(config.workFolder,[mdl '.slx']));save(fullfile(config.workFolder,'normal_contact_trial_candidate.mat'),'p');
report=struct('version','inertial-v7','sourceFolder',config.sourceFolder,'workFolder',config.workFolder, ...
 'accelerationFeedforwardFilter_s',p.inertial.accelerationFeedforwardFilter_s,'algebraicLoopCount',numel(loops), ...
 'filterPath','Force controller/Coil PI and PID/Measured motion feedforward/Acceleration FF command filter', ...
 'logContract','accelerationFeedforward: measured aZ, causal predicted aZ before command filter, final filtered aZ entering current FF', ...
 'scope','Additional one-state current-FF-only present-sample EMA after causal acceleration prediction; force reconstruction, normal direction and guide velocity prediction unchanged.');
end

function patch_prepare(path)
body=fileread(path);marker="if isfield(p,'accel') && p.accel.enabled";assert(contains(body,marker));
addition=strjoin([ ...
 "if isfield(p,'inertial')"; ...
 " if ~isfield(p.inertial,'accelerationFeedforwardFilter_s'),p.inertial.accelerationFeedforwardFilter_s=0;end"; ...
 " tau=p.inertial.accelerationFeedforwardFilter_s;dt=p.force.sampleTime;assert(tau>=0);"; ...
 " if tau==0,num=[1,0];den=[1,0];groupDelay=0;else,a=exp(-dt/tau);num=[1-a,0];den=[1,-a];groupDelay=dt*a/(1-a);end"; ...
 " p.d.accelerationFeedforwardFilter=struct('numerator',num,'denominator',den,'dcGroupDelay_s',groupDelay);"; ...
 "end";string(marker)],newline);
body=strrep(body,marker,addition);fid=fopen(path,'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',body);
end

function patch_runner(path)
body=fileread(path);
old="rows(logged,2),'names',{{'measuredAz_m_s2','feedforwardAz_m_s2'}}";
new="rows(logged,3),'names',{{'measuredAz_m_s2','predictedFeedforwardAz_m_s2','filteredFeedforwardAz_m_s2'}}";
assert(contains(body,old));body=strrep(body,old,new);fid=fopen(path,'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',body);
end

function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
