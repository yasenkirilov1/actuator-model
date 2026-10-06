% Execute only after this worker's full-grid MATLAB process has exited.
addpath('/home/tommie/Projects/actuator-model/acf_simulink/experiments');
addpath('/home/tommie/Projects/actuator-model/acf_simulink/validation');
source='/home/tommie/.local/state/actuator-model/inertial-delivery-frozen-actual-acquisition-actuator-donor-v8-20260930';
output='/home/tommie/Projects/actuator-model/acf_simulink/validation';
options=struct('tag','actual_acquisition_v8_20260930', ...
 'simulationMode','accelerator','stopTime_s',4,'baseStep_s',10e-6, ...
 'refinedStep_s',5e-6,'refine',true);
report=qualify_inertial_actuators_final(source,output,options);
assert(strcmp(report.sourceModelSHA256,'7fa60a844f8d7d0b0b1d15d4afcdc901c6d7cbe26a463e41cb2e0b78e8d8e0f8'));
assert(strcmp(report.candidateSHA256,'65b97e2e4703b69f2a47a8d8d819fc22f82c94daf115e24d75f1dd84156b305f'));
reportFile=fullfile(output,['inertial_actuators_final_' options.tag '.json']);
replayFile=fullfile(output,['inertial_actuators_final_' options.tag '_replay.json']);
inertial_actuators_export_final_v8(reportFile,replayFile);
