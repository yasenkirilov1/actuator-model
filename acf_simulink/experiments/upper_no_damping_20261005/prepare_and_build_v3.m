function prepare_and_build_v3(root,designNames)
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));end
recipe=fullfile(root,'acf_simulink','experiments','upper_no_damping_20261005');addpath(recipe,'-begin');
if nargin<2,designNames={'baseline_upper_rigid_lower_damped','upper_rigid_lower_undamped', ...
 'upper_20k_lower_cell_undamped','upper_20k_lower_19746_undamped','both_external_springs_absent'};end
validation=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005');
for k=1:numel(designNames)
 design=designNames{k};folder=fullfile(validation,'banks',design);build=fullfile(validation,'native',design);
 prepare_mechanical_bank(folder,design);metadata=build_mechanical_native_v3(build,fullfile(folder,'configs.mat'));
 assert(strcmp(metadata.status,'generated_native_c'));fprintf('UPPER_NATIVE_GRAPH_READY %s\n',build);
end
end
