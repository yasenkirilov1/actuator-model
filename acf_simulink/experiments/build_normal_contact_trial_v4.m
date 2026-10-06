function report=build_normal_contact_trial_v4(config)
%BUILD_NORMAL_CONTACT_TRIAL_V4 Add optional native feedback-only rolloff.
if nargin<1,config=struct;end
if ~isfield(config,'sourceFolder'),config.sourceFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-layout-20260930';end
if ~isfield(config,'workFolder'),config.workFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v4-20260930';end
assert(~usejava('desktop') && ~strcmp(config.sourceFolder,config.workFolder));
for n={'','helpers','cache','code','sdi','traces'}
 f=fullfile(config.workFolder,n{1});if ~isfolder(f),mkdir(f);end
end
copyfile(fullfile(config.sourceFolder,'acf_force_control_normal_trial.slx'),fullfile(config.workFolder,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(config.sourceFolder,'helpers','*.m'),fullfile(config.workFolder,'helpers'));
patch_prepare(fullfile(config.workFolder,'helpers','acf_prepare.m'));
addpath(fullfile(config.workFolder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator acf_select_controller;
Simulink.fileGenControl('set','CacheFolder',fullfile(config.workFolder,'cache'), ...
 'CodeGenFolder',fullfile(config.workFolder,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(config.workFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(config.workFolder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error');
z=load(fullfile(config.sourceFolder,'normal_contact_trial_candidate.mat'),'p');p=z.p;
p.inertial.feedbackFilter=struct('enabled',false,'cutoff_Hz',800,'damping',1/sqrt(2));
p=acf_prepare(p);mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);
coil=[mdl '/Force controller/Coil PI and PID'];feedback=[coil '/Normal force PI and PID'];
sumPath=[coil '/PI PID and feedforward current'];ports=get_param(sumPath,'PortHandles');
for j=1:numel(ports.Inport)
 h=get_param(ports.Inport(j),'Line');if h~=-1,delete_line(h);end
end
for name={'proportional_current_A','integral_current_A','derivative_current_A'}
 delete_block([feedback '/' name{1}]);
end
add_block('simulink/Math Operations/Sum',[feedback '/PI and PID correction'], ...
 'Position',[995 115 1030 455],'Inputs','+++','IconShape','rectangular','FontSize','11');
add_block('simulink/Discrete/Discrete Transfer Fcn',[feedback '/Optional feedback rolloff'], ...
 'Position',[1115 245 1395 315],'Numerator','p.d.feedbackFilter.numerator', ...
 'Denominator','p.d.feedbackFilter.denominator','InitialStates','p.d.feedbackFilter.initialStates', ...
 'SampleTime','p.force.sampleTime','FontSize','11');
add_block('simulink/Ports & Subsystems/Out1',[feedback '/feedback_current_A'], ...
 'Position',[1495 273 1525 287],'Port','1','FontSize','11');
set_param([feedback '/Normal force derivative current'],'Position',[760 360 880 410]);
add_line(feedback,'Normal force proportional current/1','PI and PID correction/1','autorouting','on');
add_line(feedback,'Normal force integral current/1','PI and PID correction/2','autorouting','on');
add_line(feedback,'Normal force derivative current/1','PI and PID correction/3','autorouting','on');
add_line(feedback,'PI and PID correction/1','Optional feedback rolloff/1','autorouting','on');
add_line(feedback,'Optional feedback rolloff/1','feedback_current_A/1','autorouting','on');
set_param(sumPath,'Inputs','+++++','Position',[760 170 790 430]);
sources={'Flat holding current/1','Normal force PI and PID/1','Measured motion feedforward/1', ...
 'Measured motion feedforward/2','Measured motion feedforward/3'};
for j=1:numel(sources),add_line(coil,sources{j},sprintf('PI PID and feedforward current/%d',j),'autorouting','on');end
for sys={feedback,coil}
 lines=find_system(sys{1},'FindAll','on','SearchDepth',1,'Type','line');Simulink.BlockDiagram.routeLine(lines);
end
set_param(mdl,'SimulationCommand','update');loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);assert(isempty(loops));
save_system(mdl,fullfile(config.workFolder,[mdl '.slx']));
save(fullfile(config.workFolder,'normal_contact_trial_candidate.mat'),'p');
report=struct('version','inertial-v4','workFolder',config.workFolder,'sourceFolder',config.sourceFolder, ...
 'feedbackFilter',p.inertial.feedbackFilter,'algebraicLoopCount',numel(loops), ...
 'filterPath','Force controller/Coil PI and PID/Normal force PI and PID/Optional feedback rolloff', ...
 'scope','Two-pole native low-pass on PI/PID current correction only; measured motion feedforward is direct.');
end

function patch_prepare(path)
source=fileread(path);marker="if isfield(p,'accel') && p.accel.enabled";assert(contains(source,marker));
addition=strjoin([ ...
 "if isfield(p,'inertial')"; ...
 " if ~isfield(p.inertial,'feedbackFilter'),p.inertial.feedbackFilter=struct('enabled',false,'cutoff_Hz',800,'damping',1/sqrt(2));end"; ...
 " c=p.inertial.feedbackFilter;dt=p.force.sampleTime;assert(c.cutoff_Hz>0&&c.cutoff_Hz<.45/dt&&c.damping>0);"; ...
 " if c.enabled,w=tan(pi*c.cutoff_Hz*dt);den=[1+2*c.damping*w+w^2,2*(w^2-1),1-2*c.damping*w+w^2];num=w^2*[1,2,1]/den(1);den=den/den(1);else,num=[1,0,0];den=[1,0,0];end"; ...
 " p.d.feedbackFilter=struct('numerator',num,'denominator',den,'initialStates',zeros(2,1));"; ...
 "end";string(marker)],newline);
source=strrep(source,marker,addition);fid=fopen(path,'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',source);
end

function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
