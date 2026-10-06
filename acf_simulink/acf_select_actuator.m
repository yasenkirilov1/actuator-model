function p=acf_select_actuator(mdl,choice)
mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
if nargin<2,choice=get_param([mdl '/Actuator selection'],'ActuatorChoice');end
kinds={'leadscrew','voice_coil','ideal_position'};labels={'Leadscrew','Voice coil','Ideal position (reference)'};variants={'Leadscrew','Voice coil','Ideal position'};
index=find(strcmp(choice,kinds)|strcmp(choice,labels)|strcmp(choice,variants),1);assert(~isempty(index),'Unknown actuator selection.');
rigidRequested=p.mechanics.lowerRigid||p.mechanics.upperRigid;
if index==3&&rigidRequested
 error('The ideal-position reference requires an explicitly chosen compliant attachment preset.');
end
if strcmp(p.coil.strategy,'observer'),if p.inertial.forceKd==0,p.coil.strategy='pi';else,p.coil.strategy='pid';end,end
p.actuator.kind=kinds{index};p=acf_prepare(p);assignin(mw,'p',p);
drive=[mdl '/Actuator and RABS mechanics/Actuator'];if ~strcmp(get_param(drive,'LabelModeActiveChoice'),variants{index}),set_param(drive,'LabelModeActiveChoice',variants{index});end
if index==2,controlChoice='Coil PI and PID';elseif index==1&&isfield(p.screw,'controlStrategy')&&strcmp(p.screw.controlStrategy,'current'),controlChoice='Screw normal PI and PID';else,controlChoice='Drive PI and PID';end
controller=[mdl '/Force controller'];if ~strcmp(get_param(controller,'LabelModeActiveChoice'),controlChoice),set_param(controller,'LabelModeActiveChoice',controlChoice);end
selector=[mdl '/Force controller selection'];mask=Simulink.Mask.get(selector);
if p.force.feedforwardEnabled&&strcmp(p.coil.strategy,'pi'),label='Feedforward PI';elseif strcmp(p.coil.strategy,'pid'),label='PID';else,label='PI';end
if ~strcmp(get_param(selector,'ControllerChoice'),label),set_param(selector,'ControllerChoice',label);end;if ~strcmp(mask.Parameters(1).Enabled,'on'),mask.Parameters(1).Enabled='on';end
if ~isempty(mask.getParameter('AccelerationFeedforward')),if p.inertial.accelerationFeedforwardScale==0,on='off';else,on='on';end;if ~strcmp(get_param(selector,'AccelerationFeedforward'),on),set_param(selector,'AccelerationFeedforward',on);end,end
links={'B to S spring and damper','R to A spring and damper'};rigid=[p.mechanics.lowerRigid,p.mechanics.upperRigid];
for j=1:2,source=[mdl '/Actuator and RABS mechanics/' links{j} '/Compliant or rigid link'];if getSimulinkBlockHandle(source)~=-1,if rigid(j),linkChoice='Rigid';else,linkChoice='Compliant';end;if ~strcmp(get_param(source,'LabelModeActiveChoice'),linkChoice),set_param(source,'LabelModeActiveChoice',linkChoice);end,end,end
if index==1
 plant=[mdl '/Actuator and RABS mechanics/Actuator/Leadscrew'];
 src=[plant '/Current request source'];if strcmp(p.screw.controlStrategy,'current'),v='Current command';else,v='Speed loop';end;if ~strcmp(get_param(src,'LabelModeActiveChoice'),v),set_param(src,'LabelModeActiveChoice',v);end
 src=[plant '/Transmitted force mechanics'];if p.mechanics.lowerRigid,v='Rigid B-S reference';else,v='Finite B-S cell';end;if ~strcmp(get_param(src,'LabelModeActiveChoice'),v),set_param(src,'LabelModeActiveChoice',v);end
end
 src=[mdl '/Actuator and RABS mechanics/Boundary load selection'];if getSimulinkBlockHandle(src)>0,if index==1&&p.mechanics.upperRigid,v='Rigid A';else,v='Compliant A';end;if ~strcmp(get_param(src,'LabelModeActiveChoice'),v),set_param(src,'LabelModeActiveChoice',v);end;end
selector=[mdl '/Actuator selection'];if ~strcmp(get_param(selector,'ActuatorChoice'),labels{index}),set_param(selector,'ActuatorChoice',labels{index});end
end


