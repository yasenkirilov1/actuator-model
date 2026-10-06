function report=build_normal_contact_trial_v3(config)
%BUILD_NORMAL_CONTACT_TRIAL_V3 Move native sensing outside drive selection.
% Only private copies are changed. Frozen inertial-v2 inputs remain immutable.
if nargin<1,config=struct;end
project=fileparts(fileparts(fileparts(mfilename('fullpath'))));
if ~isfield(config,'sourceFolder'),config.sourceFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-imu-20260930/frozen-inertial-v2';end
if ~isfield(config,'workFolder'),config.workFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v3-20260930';end
if ~isfield(config,'outputFolder'),config.outputFolder=fullfile(project,'acf_simulink','validation');end
assert(~usejava('desktop'),'Use a private MATLAB batch session.');
assert(~strcmp(config.sourceFolder,config.workFolder));
for name={'','helpers','cache','code','sdi','traces'}
 folder=fullfile(config.workFolder,name{1});if ~isfolder(folder),mkdir(folder);end
end
copyfile(fullfile(config.sourceFolder,'acf_force_control_normal_trial.slx'),fullfile(config.workFolder,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(config.sourceFolder,'helpers','*.m'),fullfile(config.workFolder,'helpers'));
addpath(fullfile(config.workFolder,'helpers'),'-begin');
prepare_helpers(fullfile(config.workFolder,'helpers'));
clear acf_prepare acf_run acf_select_actuator acf_select_controller;
Simulink.fileGenControl('set','CacheFolder',fullfile(config.workFolder,'cache'), ...
 'CodeGenFolder',fullfile(config.workFolder,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(config.workFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(config.workFolder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error');
loaded=load(fullfile(config.sourceFolder,'normal_contact_trial_candidate.mat'),'p');p=loaded.p;
p.inertial.guideVelocityPrediction_s=0;
p.inertial.currentNotch=struct('enabled',false,'frequency_Hz',sqrt(64e6/.6)/(2*pi), ...
 'zeroDamping',.01,'poleDamping',.3);
p.coil.strategy='pi';p=acf_prepare(p);
mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);
source=[mdl '/Force controller/Voice coil inertial control'];
add_contact_measurement(mdl,source);
add_coil_controller(mdl,source);
delete_block(source);
if getSimulinkBlockHandle([mdl '/Force controller/Voice coil observer'])~=-1
 delete_block([mdl '/Force controller/Voice coil observer']);
end
other=[mdl '/Force controller/Other actuators PI'];
set_param(other,'Name','Drive PI and PID');other=[mdl '/Force controller/Drive PI and PID'];
rewrite_generic_outputs(other);
set_param([other '/Derivative gain'],'Gain','p.d.forceKd*double(strcmp(p.coil.strategy,''pid''))');
set_param(other,'VariantControl','Drive PI and PID');
replace_line(mdl,'Force error/1','Force controller/1','Contact force measurement/1');
replace_line(mdl,'Receive S-A relative position/1','Force controller/2','Contact force measurement/2');
link(mdl,'Contact force measurement/1','Force controller/1');
link(mdl,'Contact force measurement/2','Force controller/2');
delete_line(mdl,'Force error/1','Send measured_error_N/1');
link(mdl,'Contact force measurement/1','Send measured_error_N/1');
set_param([mdl '/Force controller'],'LabelModeActiveChoice','Coil PI and PID');
configure_selector(mdl);
p=acf_select_actuator(mdl,'voice_coil'); %#ok<NASGU>
set_param(mdl,'SimulationCommand','update');
loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);assert(isempty(loops));
save_system(mdl,fullfile(config.workFolder,[mdl '.slx']));
save(fullfile(config.workFolder,'normal_contact_trial_candidate.mat'),'p');
report=struct('version','inertial-v3','modelFile',fullfile(config.workFolder,[mdl '.slx']), ...
 'candidateFile',fullfile(config.workFolder,'normal_contact_trial_candidate.mat'), ...
 'sourceFolder',config.sourceFolder,'configuration',config,'algebraicLoopCount',numel(loops), ...
 'controllerStrategies',{{'pi','pid'}},'activeCoilVariant','Coil PI and PID', ...
 'otherActuatorVariant','Drive PI and PID','guideVelocityPrediction_s',0, ...
 'coilCurrentNotch',p.inertial.currentNotch,'currentNotchUnits','A; coil control branch only');
path=fullfile(config.outputFolder,'normal_contact_trial_inertial_v3_build.json');
write_text(path,jsonencode(report,PrettyPrint=true));
end

function add_contact_measurement(mdl,source)
sys=[mdl '/Contact force measurement'];blank(sys,[355 275 645 425]);
block('simulink/Ports & Subsystems/In1',sys,'raw_cell_error_N',[20 68 50 82],'Port','1');
block('simulink/Ports & Subsystems/In1',sys,'measured_head_position_m',[20 318 50 332],'Port','2');
recon=[sys '/Contact force reconstruction'];blank(recon,[150 25 470 140]);
reconNames={'Cell error to deviation','Tared axial cell force','Measured S acceleration Z', ...
 'Measured S acceleration X','Measured lateral cell force','Downstream Z inertia', ...
 'Downstream X inertia','Compensated axial reaction','Compensated lateral reaction', ...
 'Direct axial reaction','Direct lateral reaction'};
copy_blocks(source,recon,reconNames);
block('simulink/Ports & Subsystems/In1',recon,'raw_cell_error_N',[20 33 50 47],'Port','1');
block('simulink/Ports & Subsystems/Out1',recon,'axial_contact_N',[1170 48 1200 62],'Port','1');
block('simulink/Ports & Subsystems/Out1',recon,'lateral_contact_N',[1450 288 1480 302],'Port','2');
link(recon,'raw_cell_error_N/1','Cell error to deviation/1');
link(recon,'Cell error to deviation/1','Tared axial cell force/1');
link(recon,'Tared axial cell force/1','Compensated axial reaction/1');
link(recon,'Measured S acceleration Z/1','Downstream Z inertia/1');
link(recon,'Downstream Z inertia/1','Compensated axial reaction/2');
link(recon,'Measured S acceleration X/1','Downstream X inertia/1');
link(recon,'Downstream X inertia/1','Compensated lateral reaction/2');
link(recon,'Measured lateral cell force/1','Compensated lateral reaction/1');
link(recon,'Compensated axial reaction/1','Direct axial reaction/1');
link(recon,'Compensated lateral reaction/1','Direct lateral reaction/1');
link(recon,'Compensated axial reaction/1','axial_contact_N/1');
link(recon,'Compensated lateral reaction/1','lateral_contact_N/1');
motion=[sys '/Head motion estimator'];blank(motion,[150 260 470 375]);
motionNames={'Measured head coordinate deviation','Head gap coordinate sign', ...
 'Issued robot acceleration','Causal motion measurements','Head gap and velocity observer', ...
 'Head motion states','Position frame matching','Robot acceleration frame matching', ...
 'Robot acceleration frame matching delay','Issued robot velocity','Robot velocity frame matching', ...
 'Robot velocity frame matching delay','Aligned robot velocity','Head gap velocity', ...
 'Head gap displacement','Estimated head travel','Measured S acceleration Z'};
copy_blocks(source,motion,motionNames);
block('simulink/Ports & Subsystems/In1',motion,'measured_head_position_m',[20 393 50 407],'Port','1');
block('simulink/Ports & Subsystems/Out1',motion,'head_gap_velocity_m_s',[1940 428 1970 442],'Port','1');
block('simulink/Ports & Subsystems/Out1',motion,'estimated_head_travel_m',[2240 548 2270 562],'Port','2');
link(motion,'measured_head_position_m/1','Measured head coordinate deviation/1');
link(motion,'Measured head coordinate deviation/1','Head gap coordinate sign/1');
link(motion,'Head gap coordinate sign/1','Position frame matching/1');
link(motion,'Position frame matching/1','Causal motion measurements/1');
link(motion,'Issued robot acceleration/1','Robot acceleration frame matching/1');
link(motion,'Robot acceleration frame matching/1','Robot acceleration frame matching delay/1');
link(motion,'Robot acceleration frame matching delay/1','Causal motion measurements/2');
link(motion,'Measured S acceleration Z/1','Causal motion measurements/3');
link(motion,'Causal motion measurements/1','Head gap and velocity observer/1');
link(motion,'Head gap and velocity observer/1','Head motion states/1');
link(motion,'Head gap and velocity observer/1','Head gap velocity/1');
link(motion,'Head gap and velocity observer/1','Head gap displacement/1');
link(motion,'Head gap velocity/1','head_gap_velocity_m_s/1');
link(motion,'Head gap displacement/1','Estimated head travel/1');
link(motion,'Estimated head travel/1','estimated_head_travel_m/1');
link(motion,'Issued robot velocity/1','Robot velocity frame matching/1');
link(motion,'Robot velocity frame matching/1','Robot velocity frame matching delay/1');
link(motion,'Robot velocity frame matching delay/1','Aligned robot velocity/1');
goto(motion,'Aligned robot acceleration','inertial_robot_acceleration_aligned',[1300 745 1590 775]);
link(motion,'Robot acceleration frame matching delay/1','Aligned robot acceleration/1');
add_block([source '/Normal force projection'],[sys '/Normal force projection'],'Position',[565 65 890 215]);
block('simulink/Math Operations/Gain',sys,'Normal force error sign',[1000 90 1210 145],'Gain','-1');
block('simulink/Ports & Subsystems/Out1',sys,'normal_force_error_N',[1310 108 1340 122],'Port','1');
block('simulink/Ports & Subsystems/Out1',sys,'estimated_head_travel_m',[1310 333 1340 347],'Port','2');
goto(sys,'Axial force feedforward request','inertial_axial_reference_deviation_N',[995 220 1295 250]);
goto(sys,'Measured head gap velocity','inertial_head_gap_velocity',[600 450 890 480]);
link(sys,'raw_cell_error_N/1','Contact force reconstruction/1');
link(sys,'measured_head_position_m/1','Head motion estimator/1');
link(sys,'Contact force reconstruction/1','Normal force projection/1');
link(sys,'Contact force reconstruction/2','Normal force projection/2');
link(sys,'Head motion estimator/1','Normal force projection/3');
link(sys,'Head motion estimator/1','Measured head gap velocity/1');
link(sys,'Head motion estimator/2','estimated_head_travel_m/1');
link(sys,'Normal force projection/1','Normal force error sign/1');
link(sys,'Normal force error sign/1','normal_force_error_N/1');
link(sys,'Normal force projection/2','Axial force feedforward request/1');
arrange(recon);arrange(motion);arrange([sys '/Normal force projection']);
end

function add_coil_controller(mdl,source)
parent=[mdl '/Force controller'];sys=[parent '/Coil PI and PID'];blank(sys,[235 295 565 415]);
set_param(sys,'VariantControl','Coil PI and PID');
block('simulink/Ports & Subsystems/In1',sys,'error_N',[20 48 50 62],'Port','1');
block('simulink/Ports & Subsystems/In1',sys,'measured_travel_m',[20 418 50 432],'Port','2');
feedback=[sys '/Normal force PI and PID'];blank(feedback,[160 20 500 155]);
feedbackNames={'Normal force proportional current','Normal error and anti windup', ...
 'Normal force error integral','Normal force integral current','Normal force derivative current', ...
 'Normal error previous sample','Normal error sample increment','Normal error new derivative weight', ...
 'Normal error filtered derivative','Normal error derivative memory','Normal error old derivative weight'};
copy_blocks(source,feedback,feedbackNames);
block('simulink/Ports & Subsystems/In1',feedback,'error_N',[20 18 50 32],'Port','1');
block('simulink/Ports & Subsystems/In1',feedback,'anti_windup',[20 188 50 202],'Port','2');
block('simulink/Math Operations/Gain',feedback,'Error to normal deviation',[130 0 370 55],'Gain','-1');
block('simulink/Ports & Subsystems/Out1',feedback,'proportional_current_A',[1660 970 1690 984],'Port','1');
block('simulink/Ports & Subsystems/Out1',feedback,'integral_current_A',[1660 1100 1690 1114],'Port','2');
block('simulink/Ports & Subsystems/Out1',feedback,'derivative_current_A',[1660 1590 1690 1604],'Port','3');
set_param([feedback '/Normal force derivative current'],'Gain','-p.inertial.forceKd*double(strcmp(p.coil.strategy,''pid''))');
link(feedback,'error_N/1','Error to normal deviation/1');
link(feedback,'Error to normal deviation/1','Normal force proportional current/1');
link(feedback,'Error to normal deviation/1','Normal error and anti windup/1');
link(feedback,'anti_windup/1','Normal error and anti windup/2');
link(feedback,'Normal error and anti windup/1','Normal force error integral/1');
link(feedback,'Normal force error integral/1','Normal force integral current/1');
link(feedback,'Error to normal deviation/1','Normal error previous sample/1');
link(feedback,'Error to normal deviation/1','Normal error sample increment/1');
link(feedback,'Normal error previous sample/1','Normal error sample increment/2');
link(feedback,'Normal error sample increment/1','Normal error new derivative weight/1');
link(feedback,'Normal error new derivative weight/1','Normal error filtered derivative/1');
link(feedback,'Normal error filtered derivative/1','Normal error derivative memory/1');
link(feedback,'Normal error derivative memory/1','Normal error old derivative weight/1');
link(feedback,'Normal error old derivative weight/1','Normal error filtered derivative/2');
link(feedback,'Normal error filtered derivative/1','Normal force derivative current/1');
link(feedback,'Normal force proportional current/1','proportional_current_A/1');
link(feedback,'Normal force integral current/1','integral_current_A/1');
link(feedback,'Normal force derivative current/1','derivative_current_A/1');
ff=[sys '/Measured motion feedforward'];blank(ff,[160 205 500 340]);
ffNames={'Measured S acceleration Z','Measured acceleration current feedforward', ...
 'Geometry current feedforward','Guide viscous friction current','Guide friction velocity scale', ...
 'Guide friction direction','Guide Coulomb friction current','Guide friction feedforward'};
copy_blocks(source,ff,ffNames);
from(ff,'Axial force request','inertial_axial_reference_deviation_N',[20 30 340 60]);
from(ff,'Head gap velocity','inertial_head_gap_velocity',[20 330 340 360]);
from(ff,'Robot acceleration','inertial_robot_acceleration_aligned',[20 470 340 500]);
block('simulink/Math Operations/Sum',ff,'Measured head gap acceleration',[440 455 480 535],'Inputs','+-');
block('simulink/Math Operations/Gain',ff,'Identified guide prediction time',[570 455 890 510], ...
 'Gain','p.inertial.guideVelocityPrediction_s');
block('simulink/Math Operations/Sum',ff,'Predicted guide velocity',[980 315 1020 395],'Inputs','++');
block('simulink/Ports & Subsystems/Out1',ff,'acceleration_current_A',[2840 1210 2870 1224],'Port','1');
block('simulink/Ports & Subsystems/Out1',ff,'geometry_current_A',[2840 1320 2870 1334],'Port','2');
block('simulink/Ports & Subsystems/Out1',ff,'friction_current_A',[2910 1450 2940 1464],'Port','3');
link(ff,'Measured S acceleration Z/1','Measured acceleration current feedforward/1');
link(ff,'Axial force request/1','Geometry current feedforward/1');
link(ff,'Robot acceleration/1','Measured head gap acceleration/1');
link(ff,'Measured S acceleration Z/1','Measured head gap acceleration/2');
link(ff,'Measured head gap acceleration/1','Identified guide prediction time/1');
link(ff,'Head gap velocity/1','Predicted guide velocity/1');
link(ff,'Identified guide prediction time/1','Predicted guide velocity/2');
link(ff,'Predicted guide velocity/1','Guide viscous friction current/1');
link(ff,'Predicted guide velocity/1','Guide friction velocity scale/1');
link(ff,'Guide friction velocity scale/1','Guide friction direction/1');
link(ff,'Guide friction direction/1','Guide Coulomb friction current/1');
link(ff,'Guide viscous friction current/1','Guide friction feedforward/1');
link(ff,'Guide Coulomb friction current/1','Guide friction feedforward/2');
link(ff,'Measured acceleration current feedforward/1','acceleration_current_A/1');
link(ff,'Geometry current feedforward/1','geometry_current_A/1');
link(ff,'Guide friction feedforward/1','friction_current_A/1');
copy_blocks(source,sys,{'Flat holding current','PI PID and feedforward current','Current limits', ...
 'Current saturation error','Integral anti windup'});
block('simulink/Discrete/Discrete Transfer Fcn',sys,'Optional structural-mode current notch',[940 215 1240 285], ...
 'Numerator','p.d.currentNotch.numerator','Denominator','p.d.currentNotch.denominator', ...
 'InitialStates','p.d.currentNotch.initialStates','SampleTime','p.force.sampleTime');
block('simulink/Ports & Subsystems/Out1',sys,'command',[1320 238 1350 252],'Port','1');
block('simulink/Ports & Subsystems/Out1',sys,'estimated_travel_m',[1320 418 1350 432],'Port','2');
block('simulink/Ports & Subsystems/Out1',sys,'estimated_force_error_N',[1320 528 1350 542],'Port','3');
link(sys,'error_N/1','Normal force PI and PID/1');
link(sys,'Integral anti windup/1','Normal force PI and PID/2');
sources={'Flat holding current/1','Normal force PI and PID/1','Normal force PI and PID/2', ...
 'Measured motion feedforward/1','Measured motion feedforward/2','Measured motion feedforward/3','Normal force PI and PID/3'};
for j=1:numel(sources),link(sys,sources{j},sprintf('PI PID and feedforward current/%d',j));end
link(sys,'PI PID and feedforward current/1','Optional structural-mode current notch/1');
link(sys,'Optional structural-mode current notch/1','Current limits/1');
link(sys,'Current limits/1','Current saturation error/1');
link(sys,'Optional structural-mode current notch/1','Current saturation error/2');
link(sys,'Current saturation error/1','Integral anti windup/1');
link(sys,'Current limits/1','command/1');
link(sys,'measured_travel_m/1','estimated_travel_m/1');
link(sys,'error_N/1','estimated_force_error_N/1');
arrange(feedback);arrange(ff);arrange(sys);
end

function rewrite_generic_outputs(sys)
out=[sys '/estimated_travel_m'];ph=get_param(out,'PortHandles');delete_line(get_param(ph.Inport(1),'Line'));
out=[sys '/estimated_force_error_N'];ph=get_param(out,'PortHandles');delete_line(get_param(ph.Inport(1),'Line'));
link(sys,'measured_travel_m/1','estimated_travel_m/1');
link(sys,'error_N/1','estimated_force_error_N/1');
for name={'Cell error without inertial reconstruction','Cell minus target', ...
 'Fallback travel without rigid kinematics','Measured rigid travel or fallback', ...
 'Reverse rigid tool coordinate','Rigid gap reference','Use rigid tool kinematics'}
 if getSimulinkBlockHandle([sys '/' name{1}])~=-1,delete_block([sys '/' name{1}]);end
end
arrange(sys);
end

function configure_selector(mdl)
selector=[mdl '/Force controller selection'];mask=Simulink.Mask.get(selector);
mask.Parameters(1).TypeOptions={'PI','PID'};set_param(selector,'ControllerChoice','PI');
mask.Parameters(1).Enabled='on';
if isempty(mask.getParameter('AccelerationFeedforward'))
 mask.addParameter('Type','checkbox','Name','AccelerationFeedforward','Prompt','Measured acceleration feedforward', ...
  'Value','on','Callback','acf_select_controller(bdroot(gcb));');
end
end

function prepare_helpers(folder)
path=fullfile(folder,'acf_prepare.m');source=fileread(path);
marker='p.d.coilControl=coil_controller(p);';assert(contains(source,marker));
addition=strjoin([string(marker); ...
 "if isfield(p,'inertial')"; ...
 " if ~isfield(p.inertial,'guideVelocityPrediction_s'),p.inertial.guideVelocityPrediction_s=0;end"; ...
 " if ~isfield(p.inertial,'currentNotch'),p.inertial.currentNotch=struct('enabled',false,'frequency_Hz',sqrt(64e6/.6)/(2*pi),'zeroDamping',.01,'poleDamping',.3);end"; ...
 " c=p.inertial.currentNotch;dt=p.force.sampleTime;assert(c.frequency_Hz>0&&c.frequency_Hz<.45/dt&&c.zeroDamping>0&&c.poleDamping>0);"; ...
 " if c.enabled"; ...
 "  w=tan(pi*c.frequency_Hz*dt);den=[1+2*c.poleDamping*w+w^2,2*(w^2-1),1-2*c.poleDamping*w+w^2];num=[1+2*c.zeroDamping*w+w^2,2*(w^2-1),1-2*c.zeroDamping*w+w^2];num=num/den(1);den=den/den(1);"; ...
 " else,num=[1,0,0];den=[1,0,0];end"; ...
 " p.d.currentNotch=struct('numerator',num,'denominator',den,'initialStates',ones(2,1)*p.d.coilHoldCurrent/sum(den));"; ...
 "end"],newline);
write_text(path,strrep(source,marker,addition));
actuator=[ ...
 "function p=acf_select_actuator(mdl,choice)"; ...
 "mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');"; ...
 "if nargin<2,choice=get_param([mdl '/Actuator selection'],'ActuatorChoice');end"; ...
 "kinds={'leadscrew','voice_coil','ideal_position'};labels={'Leadscrew','Voice coil','Ideal position (reference)'};variants={'Leadscrew','Voice coil','Ideal position'};"; ...
 "index=find(strcmp(choice,kinds)|strcmp(choice,labels)|strcmp(choice,variants),1);assert(~isempty(index),'Unknown actuator selection.');"; ...
 "rigidRequested=p.mechanics.lowerRigid||p.mechanics.upperRigid;"; ...
 "if index~=2&&rigidRequested"; ...
 " assert(hasVariable(mw,'pCompliant'),'Select compliant attachments before using the leadscrew or ideal reference.');comparison=getVariable(mw,'pCompliant');"; ...
 " fields={'k1','k2','c1','c2','lowerRigid','upperRigid'};for j=1:numel(fields),p.mechanics.(fields{j})=comparison.mechanics.(fields{j});end"; ...
 " p.springs=comparison.springs;p.coil.control=comparison.coil.control;"; ...
 "end"; ...
 "if strcmp(p.coil.strategy,'observer'),if p.inertial.forceKd==0,p.coil.strategy='pi';else,p.coil.strategy='pid';end,end"; ...
 "p.actuator.kind=kinds{index};p=acf_prepare(p);assignin(mw,'p',p);"; ...
 "drive=[mdl '/Actuator and RABS mechanics/Actuator'];if ~strcmp(get_param(drive,'LabelModeActiveChoice'),variants{index}),set_param(drive,'LabelModeActiveChoice',variants{index});end"; ...
 "if index==2,controlChoice='Coil PI and PID';else,controlChoice='Drive PI and PID';end"; ...
 "controller=[mdl '/Force controller'];if ~strcmp(get_param(controller,'LabelModeActiveChoice'),controlChoice),set_param(controller,'LabelModeActiveChoice',controlChoice);end"; ...
 "selector=[mdl '/Force controller selection'];mask=Simulink.Mask.get(selector);"; ...
 "if strcmp(p.coil.strategy,'pid'),label='PID';else,label='PI';end"; ...
 "if ~strcmp(get_param(selector,'ControllerChoice'),label),set_param(selector,'ControllerChoice',label);end;if ~strcmp(mask.Parameters(1).Enabled,'on'),mask.Parameters(1).Enabled='on';end"; ...
 "if ~isempty(mask.getParameter('AccelerationFeedforward')),if p.inertial.accelerationFeedforwardScale==0,on='off';else,on='on';end;if ~strcmp(get_param(selector,'AccelerationFeedforward'),on),set_param(selector,'AccelerationFeedforward',on);end,end"; ...
 "links={'B to S spring and damper','R to A spring and damper'};rigid=[p.mechanics.lowerRigid,p.mechanics.upperRigid];"; ...
 "for j=1:2,source=[mdl '/Actuator and RABS mechanics/' links{j} '/Compliant or rigid link'];if getSimulinkBlockHandle(source)~=-1,if rigid(j),linkChoice='Rigid';else,linkChoice='Compliant';end;if ~strcmp(get_param(source,'LabelModeActiveChoice'),linkChoice),set_param(source,'LabelModeActiveChoice',linkChoice);end,end,end"; ...
 "selector=[mdl '/Actuator selection'];if ~strcmp(get_param(selector,'ActuatorChoice'),labels{index}),set_param(selector,'ActuatorChoice',labels{index});end"; ...
 "end" ];
write_text(fullfile(folder,'acf_select_actuator.m'),strjoin(actuator,newline));
controller=[ ...
 "function p=acf_select_controller(mdl,choice)"; ...
 "selector=[mdl '/Force controller selection'];mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');"; ...
 "if nargin<2,choice=get_param(selector,'ControllerChoice');end"; ...
 "if any(strcmpi(choice,{'pi','PI (force only)','Observer (force + position)','Observer (force + stroke)','observer'})),p.coil.strategy='pi';"; ...
 "elseif any(strcmpi(choice,{'pid','PID (force only)'})),p.coil.strategy='pid';else,error('Unknown control strategy.');end"; ...
 "p.inertial.accelerationFeedforwardScale=double(strcmp(get_param(selector,'AccelerationFeedforward'),'on'));"; ...
 "assignin(mw,'p',p);p=acf_select_actuator(mdl,p.actuator.kind);"; ...
 "end" ];
write_text(fullfile(folder,'acf_select_controller.m'),strjoin(controller,newline));
end

function copy_blocks(source,dest,names)
for j=1:numel(names),add_block([source '/' names{j}],[dest '/' names{j}]);end
end
function blank(sys,pos)
add_block('simulink/Ports & Subsystems/Subsystem',sys,'Position',pos);Simulink.SubSystem.deleteContents(sys);
set_param(sys,'BackgroundColor','white','ForegroundColor','black','FontSize','11');
end
function block(library,sys,name,pos,varargin)
add_block(library,[sys '/' name],'Position',pos,'FontSize','11',varargin{:});
end
function goto(sys,name,tag,pos)
block('simulink/Signal Routing/Goto',sys,name,pos,'GotoTag',tag,'TagVisibility','global');
end
function from(sys,name,tag,pos)
block('simulink/Signal Routing/From',sys,name,pos,'GotoTag',tag);
end
function link(sys,source,dest)
add_line(sys,source,dest,'autorouting','on');
end
function replace_line(sys,source,old,new)
delete_line(sys,source,old);link(sys,source,new);
end
function arrange(sys)
Simulink.BlockDiagram.arrangeSystem(sys,'FullLayout','true');
end
function write_text(path,source)
fid=fopen(path,'w');assert(fid~=-1);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',source);
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
