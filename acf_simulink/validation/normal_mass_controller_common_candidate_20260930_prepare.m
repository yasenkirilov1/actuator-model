function evidence=normal_mass_controller_common_candidate_20260930_prepare()
% Freeze a provisional common controller without changing any native model.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
folder=fileparts(mfilename('fullpath'));
sourceFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v7-20260930/frozen-inertial-v7';
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
inputFile=fullfile(folder,'normal_mass_controller_inertial_band_limited_acceleration_feedforward_20260930_v7_finite_candidate.mat');
candidateFile=fullfile(folder,'normal_mass_controller_common_provisional_20260930_v7_candidate.mat');
manifestFile=fullfile(folder,'normal_mass_controller_common_provisional_20260930_v7_manifest.json');
assert(~isfile(candidateFile) && ~isfile(manifestFile), ...
 'Keep prior evidence immutable; create a new version when the contract changes.');
oldPath=path;cleanup=onCleanup(@()path(oldPath)); %#ok<NASGU>
addpath(fullfile(sourceFolder,'helpers'),'-begin');clear acf_prepare;
inputHash=sha256(inputFile);saved=load(inputFile,'p');p=saved.p;
assert(p.force.sampleTime==100e-6 && p.coil.currentResponse==100e-6);
assert(p.mechanics.mB==.6 && p.mechanics.k1==64e6 && p.mechanics.c1==0);
assert(~p.mechanics.lowerRigid && p.mechanics.upperRigid);
assert(p.sensor.filterTime==100e-6 && p.accel.filterTime==100e-6 && ...
 p.sensor.delay==100e-6 && p.accel.delay==100e-6);
p.inertial.forceKp=.06;p.inertial.forceKi=20;p.inertial.forceKd=0;
p.inertial.feedbackFilter=struct('enabled',true,'cutoff_Hz',1000,'damping',1.5);
p.inertial.currentNotch.enabled=false;
p.contact.geometryMode='measured_drag_ratio';p.contact.dragRatioFilter_s=.005;
p.contact.dragRatioMinForce_N=1;p.inertial.guideVelocityPrediction_s=400e-6;
p.inertial.accelerationFeedforwardFilter_s=.001;
p.inertial.accelerationDerivativeFilter_s=300e-6;
dt=p.force.sampleTime;
commandDCdelay=dt/(exp(dt/p.inertial.accelerationFeedforwardFilter_s)-1);
sensorDCdelay=p.sensor.sampleTime/(exp(p.sensor.sampleTime/p.sensor.filterTime)-1);
p.inertial.accelerationPrediction_s=commandDCdelay+sensorDCdelay+ ...
 p.sensor.delay+p.coil.currentResponse+dt/2;
p.robot.forceGate.variation.enabled=false;
p.mechanics.mS=1.4;p.sensor.distalMass_kg=1.4;p.sensor.distalMass_model_kg=1.4;
p.force.reference=20;p.mechanics.angle=0;p.contact.mu=0;p.sensor.seed=29;
p.coil.strategy='pi';p.sim.step=10e-6;p.sim.stopTime=4;p.sim.evaluateFrom=0;
p.sensor.noiseRms=.02;p.sensor.strokeNoiseRms=1e-6;
p.contact.lateralNoiseRms_N=.02;p.accel.noiseRms=.0123;p.accel.nominalNoiseRms=.0123;
p=acf_prepare(p);
assert(abs(p.inertial.accelerationPrediction_s-.0012590308651644367)<1e-15);
evidence=struct('status','provisional_not_qualified', ...
 'sourceModel',sourceModel,'modelSha256',sha256(sourceModel), ...
 'sourcePrepare',fullfile(sourceFolder,'helpers','acf_prepare.m'), ...
 'sourcePrepareSha256',sha256(fullfile(sourceFolder,'helpers','acf_prepare.m')), ...
 'inputCandidate',inputFile,'inputSha256',inputHash, ...
 'candidateFile',candidateFile,'parameters',p, ...
 'reason','Common acceleration-FF shaping preserves nominal64MN/m endpoint passes and improves the independently tested110MN/m endpoint cohort. Full mass/force/orientation grid, long-run, calibration, phase and physical validation remain required.', ...
 'endpointEvidence',fullfile(folder,'normal_mass_controller_inertial_band_limited_acceleration_feedforward_20260930_v7_finite_results.json'), ...
 'structuralEvidence',fullfile(folder,'inertial_mechanics_structural_cell_rate_pilot_20260930_v7_110M_FF1ms_results.json'), ...
 'physicalQualification',false,'knownToolMassRequiresPrepare',true);
assert(strcmp(inputHash,sha256(inputFile)),'Input candidate changed during preparation.');
save(candidateFile,'p','evidence','-v7');evidence.candidateSha256=sha256(candidateFile);
evidence.preparationScriptSha256=sha256([mfilename('fullpath') '.m']);
fid=fopen(manifestFile,'w');assert(fid~=-1);
fprintf(fid,'%s',jsonencode(evidence,PrettyPrint=true));fclose(fid);
fprintf('COMMON_PROVISIONAL_CANDIDATE %s, acceleration prediction %.15g s.\n', ...
 candidateFile,p.inertial.accelerationPrediction_s);
end

function hash=sha256(file)
assert(isfile(file));quote=char(39);
escaped=strrep(file,quote,[quote,char(34),quote,char(34),quote]);
[status,result]=system(['sha256sum -- ',quote,escaped,quote]);assert(status==0);
hash=regexp(result,'[0-9a-f]{64}','match','once');assert(~isempty(hash));
end
