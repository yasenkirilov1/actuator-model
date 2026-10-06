function p=acf_fixed_apply(mdl)
% Apply Model Workspace p to this versioned native development model.
mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
assert(p.mechanics.k2==20000&&p.mechanics.c2==0&&~p.mechanics.upperRigid, ...
 'This model uses the fixed upper20 N/mm spring with no external damping.');
assert(p.mechanics.k1==64e6&&p.mechanics.c1==0&&~p.mechanics.lowerRigid, ...
 'The externally rigid lower link retains its physical64 MN/m load cell.');
p=acf_fixed_select_actuator(mdl,p.actuator.kind);
set_param(mdl,'StopTime',num2str(p.sim.stopTime,17),'FixedStep',num2str(p.sim.step,17));
end
