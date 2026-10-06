function p=acf_fixed_select_actuator(mdl,choice)
mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
if nargin<2,choice=get_param([mdl '/Actuator selection'],'ActuatorChoice');end
kinds={'leadscrew','voice_coil','ideal_position'};labels={'Leadscrew','Voice coil','Ideal position (reference)'};variants={'Leadscrew','Voice coil','Ideal position'};
index=find(strcmp(choice,kinds)|strcmp(choice,labels)|strcmp(choice,variants),1);assert(~isempty(index),'Unknown actuator selection.');
rigidRequested=p.mechanics.lowerRigid||p.mechanics.upperRigid;
if index~=2&&rigidRequested
 assert(hasVariable(mw,'pCompliant'),'Select compliant attachments before using the leadscrew or ideal reference.');comparison=getVariable(mw,'pCompliant');
 fields={'k1','k2','c1','c2','lowerRigid','upperRigid'};for j=1:numel(fields),p.mechanics.(fields{j})=comparison.mechanics.(fields{j});end
 p.springs=comparison.springs;p.coil.control=comparison.coil.control;
end
if strcmp(p.coil.strategy,'observer'),if p.inertial.forceKd==0,p.coil.strategy='pi';else,p.coil.strategy='pid';end,end
p.actuator.kind=kinds{index};p=acf_fixed_prepare(p);assignin(mw,'p',p);
drive=[mdl '/Actuator and RABS mechanics/Actuator'];if ~strcmp(get_param(drive,'LabelModeActiveChoice'),variants{index}),set_param(drive,'LabelModeActiveChoice',variants{index});end
if index==2,controlChoice='Coil PI and PID';else,controlChoice='Drive PI and PID';end
controller=[mdl '/Force controller'];if ~strcmp(get_param(controller,'LabelModeActiveChoice'),controlChoice),set_param(controller,'LabelModeActiveChoice',controlChoice);end
selector=[mdl '/Force controller selection'];mask=Simulink.Mask.get(selector);
if strcmp(p.coil.strategy,'pid'),label='PID';else,label='PI';end
if ~strcmp(get_param(selector,'ControllerChoice'),label),set_param(selector,'ControllerChoice',label);end;if ~strcmp(mask.Parameters(1).Enabled,'on'),mask.Parameters(1).Enabled='on';end
if ~isempty(mask.getParameter('AccelerationFeedforward')),if p.inertial.accelerationFeedforwardScale==0,on='off';else,on='on';end;if ~strcmp(get_param(selector,'AccelerationFeedforward'),on),set_param(selector,'AccelerationFeedforward',on);end,end
links={'B to S spring and damper','R to A spring and damper'};rigid=[p.mechanics.lowerRigid,p.mechanics.upperRigid];
for j=1:2,source=[mdl '/Actuator and RABS mechanics/' links{j} '/Compliant or rigid link'];if getSimulinkBlockHandle(source)~=-1,if rigid(j),linkChoice='Rigid';else,linkChoice='Compliant';end;if ~strcmp(get_param(source,'LabelModeActiveChoice'),linkChoice),set_param(source,'LabelModeActiveChoice',linkChoice);end,end,end
selector=[mdl '/Actuator selection'];if ~strcmp(get_param(selector,'ActuatorChoice'),labels{index}),set_param(selector,'ActuatorChoice',labels{index});end
end
