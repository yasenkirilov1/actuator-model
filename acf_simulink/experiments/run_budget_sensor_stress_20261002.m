function run_budget_sensor_stress_20261002(project)
% Execute only frozen inputs; their preparation manifest remains immutable.
validation=fullfile(project,'acf_simulink','validation');
source=fullfile(validation,'adc_acquisition_trial_20261002');
inputs=fullfile(validation,'budget_sensor_stress_20261002_inputs');
manifest=jsondecode(fileread(fullfile(inputs,'manifest.json')));
assert(strcmp(manifest.status,'prepared_inputs_not_executed'));
order=[1 2 7 3 4 5 6];
for k=order
 group=manifest.groups(k);
 assert(isfile(group.file));
 destination=fullfile(validation,['budget_sensor_stress_20261002_' group.name]);
 qualify_adc_acquisition_20261002(source,destination,'budget_sensor_joint_stress', ...
  struct('customCasesFile',group.file,'useFastRestart',false));
end
end
