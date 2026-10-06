function p=acf_parameters(kind,scenario,preset)
%ACF_PARAMETERS Read editable parameters from the native model workspace.
% Actuator changes preserve mechanics; a preset changes it explicitly.
mdl='acf_force_control_simple';
if ~bdIsLoaded(mdl)
    folder=fileparts(mfilename('fullpath'));
    filename=fullfile(folder,'generated',[mdl '.slx']);
    if ~isfile(filename),filename=fullfile(fileparts(folder),'generated',[mdl '.slx']);end
    assert(isfile(filename),'ACF:MissingModel','Restore the saved model: %s.',filename);load_system(filename);
end
mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
if nargin>=3&&~isempty(preset)
    names={'recommended','compliant','no_springs','near_contact_reference'};
    variables={'pRecommended','pCompliant','pNoSprings','pNearContact'};
    j=find(strcmp(preset,names),1);assert(~isempty(j)&&hasVariable(mw,variables{j}),'Unknown or unavailable saved preset.');
    p=getVariable(mw,variables{j});
end
p.sim.stopTime=evalin(mw,get_param(mdl,'StopTime'));p.sim.step=evalin(mw,get_param(mdl,'FixedStep'));
if nargin>=1&&~isempty(kind),p.actuator.kind=kind;end
if nargin>=2&&~isempty(scenario)
    p.surface.kind=scenario;
    switch scenario
        case 'sharp_bump',p.surface.height=.001;p.surface.width=.01;
        case 'ramp',p.surface.height=.003;p.surface.width=.2;
        case 'repeated',p.sim.stopTime=max(p.sim.stopTime,10);
    end
end
p=acf_prepare(p);
end
