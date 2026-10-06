function p=acf_select_controller(mdl,choice)
%ACF_SELECT_CONTROLLER Choose the native coil observer, PI or filtered PID.
mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
labels={'Observer (force + position)','PI (force only)','PID (force only)'};
strategies={'observer','pi','pid'};
if nargin<2,choice=get_param([mdl '/Force controller selection'],'ControllerChoice');end
if strcmp(choice,'Observer (force + stroke)'),choice=labels{1};end
index=find(strcmp(choice,labels) | strcmp(choice,strategies),1);
assert(~isempty(index),'Unknown force-control strategy.');
p.coil.strategy=strategies{index};assignin(mw,'p',p);
p=acf_select_actuator(mdl,p.actuator.kind);
end
