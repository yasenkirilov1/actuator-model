function p = acf_parameters(kind,scenario,preset)
%ACF_PARAMETERS Read parameters from the saved/loaded Simulink model.
% No arguments preserves the current experiment. Optional actuator/scenario
% names select a test while retaining model masses, gains and drive settings.
% Sharp-bump/ramp/repeated names apply their documented example dimensions.
mdl='acf_force_control_simple';
if ~bdIsLoaded(mdl)
    filename=fullfile(fileparts(mfilename('fullpath')),'generated',[mdl '.slx']);
    assert(isfile(filename),'ACF:MissingModel','Restore the saved model: %s.',filename);
    load_system(filename);
end
mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
if nargin>=3 && ~isempty(preset)
    names={'recommended','compliant','no_springs','near_contact_reference'};
    variables={'pRecommended','pCompliant','pNoSprings','pNearContact'};
    index=find(strcmp(preset,names),1);
    assert(~isempty(index) && hasVariable(mw,variables{index}), ...
        'Unknown or unavailable saved mechanical preset.');
    p=getVariable(mw,variables{index});
end
% The model configuration is authoritative when these values are edited in UI.
p.sim.stopTime=evalin(mw,get_param(mdl,'StopTime'));
p.sim.step=evalin(mw,get_param(mdl,'FixedStep'));
if nargin>=1 && ~isempty(kind),p.actuator.kind=kind;end
if ~strcmp(p.actuator.kind,'voice_coil') && ...
        (p.mechanics.lowerRigid || p.mechanics.upperRigid)
    comparison=getVariable(mw,'pCompliant');
    fields={'k1','k2','c1','c2','lowerRigid','upperRigid'};
    for j=1:numel(fields),p.mechanics.(fields{j})=comparison.mechanics.(fields{j});end
    p.springs=comparison.springs;p.coil.control=comparison.coil.control;
end
if nargin>=2 && ~isempty(scenario)
    p.surface.kind=scenario;
    switch scenario
        case 'sharp_bump',p.surface.width=0.01;
        case 'ramp',p.surface.height=0.003;p.surface.width=0.2;
        case 'repeated',p.sim.stopTime=max(p.sim.stopTime,8);
    end
end
p=acf_prepare(p);
end
