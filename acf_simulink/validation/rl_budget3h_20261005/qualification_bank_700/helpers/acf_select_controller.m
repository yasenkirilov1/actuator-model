function p=acf_select_controller(mdl,choice)
selector=[mdl '/Force controller selection'];mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
if nargin<2,choice=get_param(selector,'ControllerChoice');end
if any(strcmpi(choice,{'pi','PI (force only)','Observer (force + position)','Observer (force + stroke)','observer'})),p.coil.strategy='pi';
elseif any(strcmpi(choice,{'pid','PID (force only)'})),p.coil.strategy='pid';else,error('Unknown control strategy.');end
p.inertial.accelerationFeedforwardScale=double(strcmp(get_param(selector,'AccelerationFeedforward'),'on'));
assignin(mw,'p',p);p=acf_select_actuator(mdl,p.actuator.kind);
end
