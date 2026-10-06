function acf_viewer_selftest
%ACF_VIEWER_SELFTEST Check the export contract with a short Simulink run.
% Preserves the loaded model settings. Creates and removes only its own temporary folder.
root=tempname; mkdir(root); cleanup=onCleanup(@() rmdir(root,'s')); %#ok<NASGU>
mdl=acf_open;mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
step=get_param(mdl,'FixedStep');stop=get_param(mdl,'StopTime');
choice=get_param([mdl '/Actuator selection'],'ActuatorChoice');
modelCleanup=onCleanup(@() restore(mdl,original,step,stop,choice)); %#ok<NASGU>
p=acf_parameters('voice_coil','flat');p.sim.stopTime=10*p.sim.step;
r=acf_run(mdl,p,false);
original=r.data; expectedNames=r.names;
% Reordered input channels must be exported in canonical order.
order=numel(r.names):-1:1; r.names=r.names(order); r.data=r.data(:,order);
r.label='Test </script><script>unexpected()</script> & export';
file=acf_visualize(r,fullfile(root,'replay.html'),false);
html=fileread(file);
part=regexp(html,'<script id="acf-data" type="application/json">(.*?)</script>','tokens','once');
assert(~isempty(part),'No replay data script found.');
assert(isempty(strfind(part{1},'</script>')),'A label escaped the data container.');
bundle=jsondecode(part{1}); one=bundle.runs(1);
assert(strcmp(bundle.schema,'acf-replay-v1'));
assert(isequal(size(one.data),size(original)),'Export dropped samples or channels.');
assert(max(abs(one.data(:)-original(:)))<1e-12,'Signal order or values changed.');
assert(max(abs(one.time(:)-r.time(:)))<1e-12,'Time samples changed.');
assert(isequal(one.names(:),expectedNames(:)),'Signal names changed.');
assert(strcmp(one.label,r.label),'Label did not round-trip through JSON.');
bad=r; bad.time(2)=bad.time(1); rejected=false;
try
    acf_visualize(bad,fullfile(root,'invalid.html'),false);
catch err
    rejected=strcmp(err.identifier,'ACF:Viewer:Time');
end
assert(rejected,'Non-increasing time must be rejected.');
acf_visualize({r,r},fullfile(root,'two_runs.html'),false);
fprintf('ACF viewer export checks passed: signal order, all samples, label escaping, time validation and multiple runs.\n');
end

function restore(mdl,p,step,stop,choice)
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
acf_select_actuator(mdl,choice);
set_param(mdl,'FixedStep',step,'StopTime',stop);
end
