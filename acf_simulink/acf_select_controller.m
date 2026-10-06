function p=acf_select_controller(mdl,choice)
% Select native normal-force feedback and optional measured-motion feedforward.
selector=[mdl '/Force controller selection'];mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
if nargin<2,choice=get_param(selector,'ControllerChoice');end
switch lower(char(choice))
 case {'feedforward pi','feedforward_pi'},p.coil.strategy='pi';p.force.feedforwardEnabled=true;
 case 'pi',p.coil.strategy='pi';p.force.feedforwardEnabled=false;
 case 'pid',p.coil.strategy='pid';p.force.feedforwardEnabled=false;
 otherwise,error('Choose Feedforward PI, PI or PID.');
end
assignin(mw,'p',p);p=acf_select_actuator(mdl,p.actuator.kind);
end
