function mdl = acf_build(p,folder)
%ACF_BUILD Create an editable, native Simulink SLX model.
% MATLAB + Simulink only. No Simscape, Stateflow, compiler or control toolbox.
% Existing models are preserved; repeated builds receive a numeric suffix.
if nargin<1, p=acf_parameters; end
if nargin<2, folder=fullfile(fileparts(mfilename('fullpath')),'generated'); end
addpath(fileparts(mfilename('fullpath')));
p=acf_prepare(p);
assert(license('test','Simulink'),'MATLAB with a Simulink license is required.');
if ~exist(folder,'dir'), mkdir(folder); end
load_system('simulink');
base='acf_force_control'; mdl=base; n=1;
while bdIsLoaded(mdl) || exist(fullfile(folder,[mdl '.slx']),'file')
    mdl=sprintf('%s_%d',base,n); n=n+1;
end
new_system(mdl);
mw=get_param(mdl,'ModelWorkspace'); assignin(mw,'p',p);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17), ...
    'ReturnWorkspaceOutputs','on','SimulationMode','normal');


% Main force-feedback path.
b(mdl,'Sources/Constant','Requested force',[40 155 105 185],'Value','p.force.reference');
b(mdl,'Math Operations/Sum','Force error',[170 150 195 190],'Inputs','+-');
roleblock(mdl,'Force controller','force',[255 125 425 215],{'error_N'},{'command'}, ...
    'red',sprintf('Sampled PI + anti-windup\nCoil: amperes; screw: rad/s; ideal: gap in metres'));
roleblock(mdl,'Actuator and RABS mechanics','plant',[510 100 800 280], ...
    {'command','robot_r_v','surface_y_v_a'},{'true_Fc_N','travel_m','diagnostics'}, ...
    'magenta',sprintf(['Coupled actuator + masses, springs and dampers\n' ...
    'Rigid unilateral contact; no contact spring or damper\n' ...
    'Edit acf_plant_rhs.m for equations; acf_plant_step.m for integration']));
roleblock(mdl,'Force measurement','sensor',[905 115 1095 210],{'true_Fc_N'},{'measured_Fc_N'}, ...
    'cyan',sprintf('Compensated force channel\nSampling + filter + bias + noise + delay'));
w(mdl,'Requested force/1','Force error/1','Fref_N');
w(mdl,'Force error/1','Force controller/1');
w(mdl,'Force controller/1','Actuator and RABS mechanics/1','actuator_command');
w(mdl,'Actuator and RABS mechanics/1','Force measurement/1','true_contact_force_N');
w(mdl,'Force measurement/1','Force error/2','measured_contact_force_N');

% Slow robot loop and independent disturbance source.
roleblock(mdl,'Recenter travel','recenter',[260 400 450 510], ...
    {'travel_m','nominal_velocity'},{'requested_velocity'},'blue', ...
    sprintf('Mid-stroke - actual travel\nDeadband + slow gain + nominal robot path'));
roleblock(mdl,'Robot command delay','delay',[520 425 705 495], ...
    {'requested_velocity'},{'delayed_velocity'},'blue','Delay measured in robot command samples');
roleblock(mdl,'Robot motion','robot',[795 400 990 510], ...
    {'delayed_velocity'},{'actual_r_v'},'blue', ...
    sprintf('Velocity response + speed/acceleration limits\nPrescribed robot boundary, no robot force input'));
roleblock(mdl,'Surface and nominal path','surface',[40 655 260 765], ...
    {},{'surface_y_v_a','nominal_velocity'},'orange', ...
    sprintf('Analytic finite-width bump or smooth ramp\nThe robot does not receive surface-height feedback'));
w(mdl,'Actuator and RABS mechanics/2','Recenter travel/1','travel_m');
w(mdl,'Surface and nominal path/2','Recenter travel/2');
w(mdl,'Recenter travel/1','Robot command delay/1','requested_robot_velocity');
w(mdl,'Robot command delay/1','Robot motion/1','delayed_robot_velocity');
w(mdl,'Robot motion/1','Actuator and RABS mechanics/2','robot_position_velocity');
w(mdl,'Surface and nominal path/1','Actuator and RABS mechanics/3','surface_position_velocity_acceleration');

% Visible force-error tolerance scope. It uses true force, not sensor force.
b(mdl,'Math Operations/Sum','True force error',[1165 265 1190 305],'Inputs','+-');
w(mdl,'Requested force/1','True force error/1');
w(mdl,'Actuator and RABS mechanics/1','True force error/2');
b(mdl,'Sources/Constant','Upper tolerance',[1150 360 1215 390],'Value','p.force.tolerance');
b(mdl,'Sources/Constant','Lower tolerance',[1150 420 1215 450],'Value','-p.force.tolerance');
b(mdl,'Signal Routing/Mux','Force error and limits',[1280 260 1285 460],'Inputs','4');
w(mdl,'True force error/1','Force error and limits/1','true_force_error_N');
w(mdl,'Force error/1','Force error and limits/2','measured_force_error_N');
w(mdl,'Upper tolerance/1','Force error and limits/3');
w(mdl,'Lower tolerance/1','Force error and limits/4');
b(mdl,'Sinks/Scope','Force tolerance scope',[1350 325 1410 385]);
w(mdl,'Force error and limits/1','Force tolerance scope/1');

% Complete data logging; all vectors use the same numerical sample grid.
logblock(mdl,'Plant log','plant_log',[1160 540 1300 580]);
logblock(mdl,'Robot log','robot_log',[1160 610 1300 650]);
logblock(mdl,'Surface log','surface_log',[1160 680 1300 720]);
w(mdl,'Actuator and RABS mechanics/3','Plant log/1');
w(mdl,'Robot motion/1','Robot log/1');
w(mdl,'Surface and nominal path/1','Surface log/1');
b(mdl,'Signal Routing/Mux','Control signals',[805 635 810 805],'Inputs','5');
w(mdl,'Requested force/1','Control signals/1');
w(mdl,'Force measurement/1','Control signals/2');
w(mdl,'Force controller/1','Control signals/3');
w(mdl,'Recenter travel/1','Control signals/4');
w(mdl,'Robot command delay/1','Control signals/5');
logblock(mdl,'Control log','control_log',[900 700 1040 740]);
w(mdl,'Control signals/1','Control log/1');

note(mdl,sprintf(['ACTIVE CONTACT FLANGE | %s | %s\n' ...
    'Illustrative parameters. Use acf_run(model,p) after editing p.'], ...
    p.actuator.kind,p.surface.kind),[40 20]);
note(mdl,'FAST LOOP: true contact force is the performance measure; measured force closes the controller.',[40 315]);
note(mdl,'SLOW LOOP: flange travel requests robot correction. Delay and motion limits are explicit.',[40 565]);
note(mdl,sprintf(['No contact spring/damper. Contact reaction is solved from S motion.\n' ...
    'Loss of contact, re-impact or stroke-stop contact fails a sizing run.']),[325 875]);
note(mdl,'Scope order: true error, measured error, +0.5 N, -0.5 N (tolerance is editable).',[1100 475]);
set_param(mdl,'SimulationCommand','update');
save_system(mdl,fullfile(folder,[mdl '.slx']));
open_system(mdl); set_param(mdl,'ZoomFactor','FitSystem');
fprintf('Created %s\n',fullfile(folder,[mdl '.slx']));
end

function roleblock(mdl,name,role,pos,ins,outs,color,description)
sys=[mdl '/' name];
add_block('built-in/Subsystem',sys,'Position',pos,'ForegroundColor',color);
for j=1:numel(ins)
    add_block('built-in/Inport',[sys '/' ins{j}],'Port',num2str(j), ...
        'Position',[35 100+75*(j-1) 65 114+75*(j-1)]);
end
core=[sys '/Dynamics'];
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',core, ...
    'FunctionName','acf_sfun','Parameters',['''' role ''',p'], ...
    'Position',[250 90 465 280]);
set_param(mdl,'SimulationCommand','update');
for j=1:numel(ins), w(sys,[ins{j} '/1'],['Dynamics/' num2str(j)]); end
for j=1:numel(outs)
    add_block('built-in/Outport',[sys '/' outs{j}],'Port',num2str(j), ...
        'Position',[615 100+75*(j-1) 645 114+75*(j-1)]);
    w(sys,['Dynamics/' num2str(j)],[outs{j} '/1']);
end
note(sys,description,[35 20]);
note(sys,'Numerical step p.sim.step; controller and sensor sample periods are independent parameters.',[35 335]);
end

function b(mdl,source,name,pos,varargin)
add_block(['simulink/' source],[mdl '/' name],'Position',pos,varargin{:});
end
function w(sys,src,dst,name)
h=add_line(sys,src,dst,'autorouting','on');
if nargin>3, set_param(h,'Name',name); end
end
function logblock(mdl,name,var,pos)
b(mdl,'Sinks/To Workspace',name,pos,'VariableName',var,'SaveFormat','Timeseries', ...
    'MaxDataPoints','inf','Decimation','1');
end
function note(sys,txt,pos)
a=Simulink.Annotation(sys,txt); a.Position=pos; a.FontSize=11;
end
