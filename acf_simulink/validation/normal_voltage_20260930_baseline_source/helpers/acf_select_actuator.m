function p = acf_select_actuator(mdl,choice)
%ACF_SELECT_ACTUATOR Configure a drive and its matching force-controller units.
% Used by the diagram dropdown and by acf_run; preserves experiment edits.
mw=get_param(mdl,'ModelWorkspace'); p=getVariable(mw,'p');
if nargin<2
    choice=get_param([mdl '/Actuator selection'],'ActuatorChoice');
end
kinds={'leadscrew','voice_coil','ideal_position'};
labels={'Leadscrew','Voice coil','Ideal position (reference)'};
variants={'Leadscrew','Voice coil','Ideal position'};
index=find(strcmp(choice,kinds) | strcmp(choice,labels) | strcmp(choice,variants),1);
assert(~isempty(index),'Unknown actuator selection.');
% Rigid-link comparisons currently use the coil drive. The saved model also
% carries a qualified compliant configuration for the screw/reference drive.
rigidRequested=(isfield(p.mechanics,'lowerRigid') && p.mechanics.lowerRigid) || ...
    (isfield(p.mechanics,'upperRigid') && p.mechanics.upperRigid);
if index~=2 && rigidRequested
    assert(hasVariable(mw,'pCompliant'), ...
        'Select compliant attachments before using the leadscrew or ideal reference.');
    comparison=getVariable(mw,'pCompliant');
    fields={'k1','k2','c1','c2','lowerRigid','upperRigid'};
    for j=1:numel(fields),p.mechanics.(fields{j})=comparison.mechanics.(fields{j});end
    p.springs=comparison.springs;p.coil.control=comparison.coil.control;
    fprintf('Using the compliant attachment preset for %s; masses, target and surface settings retained.\n',labels{index});
end
p.actuator.kind=kinds{index}; p=acf_prepare(p); assignin(mw,'p',p);
drive=[mdl '/Actuator and RABS mechanics/Actuator'];
if ~strcmp(get_param(drive,'LabelModeActiveChoice'),variants{index})
    set_param(drive,'LabelModeActiveChoice',variants{index});
end
controller=[mdl '/Force controller'];
if getSimulinkBlockHandle([controller '/Voice coil observer'])~=-1
    if index==2 && strcmp(p.coil.strategy,'observer')
        controlChoice='Voice coil observer';
    else
        controlChoice='Other actuators PI';
    end
    if ~strcmp(get_param(controller,'LabelModeActiveChoice'),controlChoice)
        set_param(controller,'LabelModeActiveChoice',controlChoice);
    end
end
controlSelector=[mdl '/Force controller selection'];
if getSimulinkBlockHandle(controlSelector)~=-1
    controlLabels={'Observer (force + position)','PI (force only)','PID (force only)'};
    strategies={'observer','pi','pid'};
    label=controlLabels{strcmp(strategies,p.coil.strategy)};
    mask=Simulink.Mask.get(controlSelector);
    if strcmp(label,controlLabels{1}) && ...
            ~any(strcmp(mask.Parameters(1).TypeOptions,label))
        label='Observer (force + stroke)';
    end
    if ~strcmp(get_param(controlSelector,'ControllerChoice'),label)
        set_param(controlSelector,'ControllerChoice',label);
    end
    if index==2,enabled='on';else,enabled='off';end
    if ~strcmp(mask.Parameters(1).Enabled,enabled)
        mask.Parameters(1).Enabled=enabled;
    end
end
links={'B to S spring and damper','R to A spring and damper'};
rigid=[p.mechanics.lowerRigid,p.mechanics.upperRigid];
for j=1:2
    source=[mdl '/Actuator and RABS mechanics/' links{j} '/Compliant or rigid link'];
    if getSimulinkBlockHandle(source)~=-1
        if rigid(j),linkChoice='Rigid';else,linkChoice='Compliant';end
        if ~strcmp(get_param(source,'LabelModeActiveChoice'),linkChoice)
            set_param(source,'LabelModeActiveChoice',linkChoice);
        end
    end
end
selector=[mdl '/Actuator selection'];
if ~strcmp(get_param(selector,'ActuatorChoice'),labels{index})
    set_param(selector,'ActuatorChoice',labels{index});
end
end
