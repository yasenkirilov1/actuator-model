function report=run_original_long(storageMode)
% Relative-path rerun of the exact recorded recipe. Never use the instrumented
% inspection models as recipe inputs: the recipe adds residual instruments.
assert(~usejava('desktop'),'Run with matlab -batch, not the user desktop.');
if nargin<1,storageMode='tmpfs';end
here=fileparts(mfilename('fullpath'));
privateRoot=fullfile(getenv('HOME'),'.local','state','actuator-model');
if ~isfolder(privateRoot),mkdir(privateRoot);end
recipe=fullfile(here,'original','experiments');
addpath(recipe,'-begin');clear qualify_inertial_uncertainty;
out=fullfile(here,'rerun_output','original');
report=qualify_inertial_uncertainty(fullfile(here,'source'),out,'budget_repeated', ...
 fullfile(here,'source','normal_contact_trial_candidate.mat'),'accelerator', ...
 fullfile(here,'data','inertial_uncertainty_budget_joint_20260930_results.mat'),storageMode);
end
