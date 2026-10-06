function prepare_and_build_cellsource_v2(root)
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));end
recipe=fullfile(root,'acf_simulink','experiments','upper_force_observer_20261005');addpath(recipe,'-begin');
validation=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005');
folder=fullfile(validation,'bank_cellsource_v2');build=fullfile(validation,'native_cellsource_v2');
prepare_cellsource_force_bank(folder);
metadata=build_upper_force_native_cellsource_v2(build,fullfile(folder,'configs.mat'));
assert(strcmp(metadata.status,'generated_native_c'));
fprintf('LOCAL_CELL_FORCE_NATIVE_GRAPH_READY %s\n',build);
end
