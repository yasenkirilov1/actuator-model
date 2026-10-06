function prepare_and_build(root)
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));end
recipe=fullfile(root,'acf_simulink','experiments','upper_force_observer_20261005');addpath(recipe,'-begin');
validation=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005');
folder=fullfile(validation,'bank');build=fullfile(validation,'native');
prepare_force_observer_bank(folder);
metadata=build_upper_force_native(build,fullfile(folder,'configs.mat'));
assert(strcmp(metadata.status,'generated_native_c'));
fprintf('UPPER_FORCE_NATIVE_GRAPH_READY %s\n',build);
end
