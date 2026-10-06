function report = build_normal_contact_trial(config)
%BUILD_NORMAL_CONTACT_TRIAL Build/run an isolated native sanding-force trial.
% The plant knows surface geometry. The controller receives only the modeled
% B-S force components, S-relative-to-A position, measured S acceleration
% when enabled, and robot commands already issued.
% Every model/helper mutation is confined to config.workFolder. The saved
% canonical model, its helper files, and older qualification data are untouched.
% Sensor specifications are modeled design targets, not hardware qualification.
if nargin<1,config=struct;end
project=fileparts(fileparts(fileparts(mfilename('fullpath'))));
config=defaults(config,struct( ...
 'workFolder',fullfile(prefdir,'normal-contact-trial-20260930'), ...
 'outputFolder',fullfile(project,'acf_simulink','validation'), ...
 'sourceModel',fullfile(project,'acf_simulink','generated','acf_force_control_simple.slx'), ...
 'sourceHelpers',fullfile(project,'acf_simulink'), ...
 'candidateFile',fullfile(project,'acf_simulink','validation','final_two_sensor_candidate.mat'), ...
 'runCases',struct([]),'sensorDesign','approved_multi_axis', ...
 'lateralNoiseRms_N',.02,'lateralBias_N',0,'slopeFilterTime_s',0, ...
 'normalIntegralScale',1,'geometryFeedforwardScale',1, ...
 'lateralPredictionTime_s',0,'lateralDerivativeFilterTime_s',.0003, ...
 'accelerationEnabled',false,'accelFilterTime_s',.0002,'forceFilterTime_s',.0002, ...
 'accelDelay_s',.0001,'accelNoiseRms_m_s2',.012,'accelBias_m_s2',0, ...
 'accelScaleError',0,'accelRange_g',8,'accelQuantum_m_s2',.00187038, ...
 'inertialForceKp_A_N',.03,'inertialForceKi_A_Ns',20,'inertialForceKd_A_s_N',0, ...
 'inertialDerivativeFilter_s',.0003,'positionBandwidth_Hz',40, ...
 'accelerationFeedforwardScale',1,'guideFrictionFeedforwardScale',1));
assert(~usejava('desktop'),'Use a private MATLAB batch session, not the user desktop.');
assert(any(strcmp(config.sensorDesign,{'conditional_multi_axis','approved_multi_axis'})), ...
 'This trial requires the approved multi-axis B-S force sensor.');
assert(~strcmp(config.workFolder,fileparts(config.sourceModel)), ...
 'The isolated work folder must differ from the authoritative model folder.');
maxNumCompThreads(2);
for suffix={'','helpers','cache','code','sdi','traces'}
 folder=fullfile(config.workFolder,suffix{1});if ~isfolder(folder),mkdir(folder);end
end
if ~isfolder(config.outputFolder),mkdir(config.outputFolder);end
files=dir(fullfile(config.sourceHelpers,'*.m'));
for j=1:numel(files),copyfile(fullfile(files(j).folder,files(j).name), ...
 fullfile(config.workFolder,'helpers',files(j).name));end
modelFile=fullfile(config.workFolder,'acf_force_control_normal_trial.slx');
copyfile(config.sourceModel,modelFile);addpath(fullfile(config.workFolder,'helpers'),'-begin');
if config.accelerationEnabled
 prepare_inertial_helpers(fullfile(config.workFolder,'helpers'));
 clear acf_prepare acf_select_actuator;
end
Simulink.fileGenControl('set','CacheFolder',fullfile(config.workFolder,'cache'), ...
 'CodeGenFolder',fullfile(config.workFolder,'code'),'createDir',true);
originalSDI=Simulink.sdi.getStorageLocation;
originalRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(config.workFolder,'sdi'));
Simulink.sdi.setRecordData(true);
load_system(modelFile);mdl='acf_force_control_normal_trial';
cleanup=onCleanup(@()finish(mdl,originalSDI,originalRecord)); %#ok<NASGU>
loaded=load(config.candidateFile,'p');p=loaded.p;
p=contact_parameters(p,config,struct('massS_kg',1.4,'force_N',50,'angle_deg',90,'seed',29,'mu',0,'clean',false));
mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);
set_param(mdl,'FastRestart','off');
add_normal_plant(mdl);
add_lateral_sensor(mdl);
add_predicted_velocity(mdl);
if config.accelerationEnabled
 add_accelerometer(mdl);
 add_inertial_control(mdl);
 add_inertial_logs(mdl);
else
 add_normal_control(mdl);
end
add_normal_log(mdl);
save_system(mdl,modelFile);
save(fullfile(config.workFolder,'normal_contact_trial_candidate.mat'),'p');
report=struct('date','2026-09-30','status','built','modelFile',modelFile, ...
 'sourceModel',config.sourceModel,'candidateFile',config.candidateFile, ...
 'sensorDesign',config.sensorDesign,'configuration',config, ...
 'forceMetric','Surface-normal pressing force N, throughout the recording', ...
 'controllerInputs',{{'B-S axial tared force','B-S lateral tared force', ...
 'S-relative-to-A position','issued axial and tangential robot commands'}}, ...
 'noPreview',true,'rows',struct([]));
report.normalLogNames={'normalForce_N','lateralForce_N','trueSlope', ...
 'estimatedNormalForce_N','estimatedSlope','requestedAxialForce_N', ...
 'measuredLateralForce_N','predictedRobotVelocity_m_s'};
if config.accelerationEnabled
 report.controllerInputs=[report.controllerInputs,{'S axial and lateral acceleration'}];
 report.accelerometerLogNames={'measuredAccelerationZ_m_s2','measuredAccelerationX_m_s2', ...
  'physicalAccelerationZ_m_s2','physicalAccelerationX_m_s2', ...
  'reconstructedAxialContact_N','reconstructedLateralContact_N'};
 report.motionObserverLogNames={'headGapDeviation_m','headGapVelocity_m_s'};
 report.accelerometerResponse='Identified first-order filter and acquisition-delay envelope';
end
started=tic;
if ~isempty(config.runCases),set_param(mdl,'FastRestart','on');end
for j=1:numel(config.runCases)
 one=config.runCases(j);q=contact_parameters(p,config,one);
 quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
 % The helper retains all 35 original physical/control channels. Capture the
 % extra native log without altering that established schema.
 assert(isequal(r.time,r.normalContact.time), ...
  'Normal qualification must retain the full native physical sample grid.');
 if config.accelerationEnabled
  assert(isequal(r.time,r.accelerometer.time) && isequal(r.time,r.motionObserver.time), ...
   'Inertial qualification must retain the full native sensor and motion sample grids.');
 end
 slope=r.data(:,29)/q.surface.scanSpeed;
 predictedNormal=r.data(:,1).*sqrt(1+slope.^2)./(1-q.contact.mu*slope);
 predictedLateral=-predictedNormal.*(slope+q.contact.mu)./sqrt(1+slope.^2);
 balanceResidual=max(abs(predictedNormal-r.normalContact.data(:,1)));
 lateralResidual=max(abs(predictedLateral-r.normalContact.data(:,2)));
 assert(balanceResidual<1e-9 && lateralResidual<1e-9, ...
  'Native normal/lateral reaction must satisfy the contact-force vector balance.');
 assert(abs(r.normalContact.data(1,1)-q.force.reference)<1e-9, ...
  'Qualification includes the correct initial normal reaction at t=0.');
 e=r.normalContact.data(:,1)-q.force.reference;
 row=struct('caseIndex',j,'massS_kg',q.mechanics.mS,'massB_kg',q.mechanics.mB, ...
  'force_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi, ...
  'seed',q.sensor.seed,'clean',q.sensor.noiseRms==0,'mu',q.contact.mu, ...
  'voltage_V',q.coil.voltage,'peakCurrentLimit_A',q.coil.peakCurrent, ...
  'peakNormalForceError_N',max(abs(e)), ...
  'peakAxialForceError_N',r.metrics.peakAbsForceError_N, ...
  'contactLoss',any(r.data(:,18)<.5),'strokeStop',any(r.data(:,21)>.5), ...
  'peakCurrent_A',max(abs(r.data(:,14))),'peakVoltage_V',max(abs(r.data(:,15))), ...
  'currentClipped',any(r.data(:,22)>.5),'voltageClipped',any(r.data(:,23)>.5), ...
  'minTravel_m',min(r.data(:,2)),'maxTravel_m',max(r.data(:,2)), ...
  'maxNormalEstimateError_N',max(abs(r.normalContact.data(:,4)-r.normalContact.data(:,1))), ...
  'maxSlopeEstimateError',max(abs(r.normalContact.data(:,5)-r.normalContact.data(:,3))), ...
  'normalBalanceResidual_N',balanceResidual,'lateralBalanceResidual_N',lateralResidual, ...
  'actualDuration_s',r.time(end),'normalIntegralScale',q.contact.normalIntegralScale, ...
  'geometryFeedforwardScale',q.contact.geometryFeedforwardScale, ...
  'lateralPredictionTime_s',q.contact.lateralPredictionTime_s);
 if config.accelerationEnabled
  row.forceKp_A_N=q.inertial.forceKp;row.forceKi_A_Ns=q.inertial.forceKi;
  row.forceKd_A_s_N=q.inertial.forceKd;
  row.accelerationFeedforwardScale=q.inertial.accelerationFeedforwardScale;
  row.guideFrictionFeedforwardScale=q.inertial.guideFrictionFeedforwardScale;
  row.accelFilterTime_s=q.accel.filterTime;row.accelDelay_s=q.accel.delay;
  row.accelNoiseRms_m_s2=q.accel.noiseRms;row.accelBias_m_s2=q.accel.bias;
  row.accelScaleError=q.accel.scaleError;row.forceFilterTime_s=q.sensor.filterTime;
 end
 row.pass=row.peakNormalForceError_N<=q.force.tolerance && ...
  ~row.contactLoss && ~row.strokeStop && r.time(end)>=q.sim.stopTime-q.sim.step;
 traceFile=fullfile(config.workFolder,'traces',sprintf('normal_contact_trial_%03d.mat',j));
 row.traceFile=traceFile;
 report.rows=append(report.rows,row);report.elapsedSeconds=toc(started);
 save(traceFile,'r','row');
 fprintf('NORMAL_CONTACT_TRIAL %d/%d: S %.3g kg, %.0f N %.0f deg, mu %.2g, seed %d: %.9g N, pass %d.\n', ...
  j,numel(config.runCases),row.massS_kg,row.force_N,row.angle_deg,row.mu,row.seed,row.peakNormalForceError_N,row.pass);
 checkpoint(report,config.outputFolder);
 Simulink.sdi.clear;
end
if ~isempty(config.runCases),report.status='complete';end
report.elapsedSeconds=toc(started);
checkpoint(report,config.outputFolder);
end

function p=contact_parameters(p,config,one)
one=defaults(one,struct('massS_kg',1.4,'force_N',50,'angle_deg',90,'seed',29,'mu',0,'clean',false));
p.mechanics.mS=one.massS_kg;p.mechanics.mB=.6;
p.sensor.distalMass_kg=one.massS_kg;p.sensor.distalMass_model_kg=one.massS_kg;
p.force.reference=one.force_N;p.mechanics.angle=one.angle_deg*pi/180;
p.sensor.seed=one.seed;
p.sensor.noiseRms=.02;p.sensor.strokeNoiseRms=1e-6;
if one.clean,p.sensor.noiseRms=0;p.sensor.strokeNoiseRms=0;end
p.contact=struct('mu',one.mu,'lateralNoiseRms_N',config.lateralNoiseRms_N, ...
 'lateralBias_N',config.lateralBias_N,'slopeFilterTime_s',config.slopeFilterTime_s, ...
 'normalIntegralScale',config.normalIntegralScale,'geometryFeedforwardScale',config.geometryFeedforwardScale, ...
 'lateralPredictionTime_s',config.lateralPredictionTime_s, ...
 'lateralDerivativeFilterTime_s',config.lateralDerivativeFilterTime_s);
p.contact.commandedScanSpeed_m_s=p.surface.scanSpeed;
if one.clean,p.contact.lateralNoiseRms_N=0;end
if isfield(one,'voltage_V'),p.coil.voltage=one.voltage_V;end
if isfield(one,'normalIntegralScale'),p.contact.normalIntegralScale=one.normalIntegralScale;end
if isfield(one,'geometryFeedforwardScale'),p.contact.geometryFeedforwardScale=one.geometryFeedforwardScale;end
if isfield(one,'lateralPredictionTime_s'),p.contact.lateralPredictionTime_s=one.lateralPredictionTime_s;end
assert(one.mu>=0 && one.mu<=.6,'The conditional contact-friction study uses 0 <= mu <= 0.6.');
assert(p.contact.lateralPredictionTime_s>=0 && p.contact.lateralPredictionTime_s<=.0004 && ...
 p.contact.lateralDerivativeFilterTime_s>0,'Use a bounded, causal lateral sensor lead.');
if config.accelerationEnabled
 p.sensor.filterTime=config.forceFilterTime_s;
 p.accel=struct('enabled',true,'filterTime',config.accelFilterTime_s, ...
  'delay',config.accelDelay_s,'noiseRms',config.accelNoiseRms_m_s2, ...
  'bias',config.accelBias_m_s2,'scaleError',config.accelScaleError, ...
  'range_g',config.accelRange_g,'quantum',config.accelQuantum_m_s2);
 p.inertial=struct('forceKp',config.inertialForceKp_A_N,'forceKi',config.inertialForceKi_A_Ns, ...
  'forceKd',config.inertialForceKd_A_s_N,'derivativeFilter',config.inertialDerivativeFilter_s, ...
  'positionBandwidth_Hz',config.positionBandwidth_Hz, ...
  'accelerationFeedforwardScale',config.accelerationFeedforwardScale, ...
  'guideFrictionFeedforwardScale',config.guideFrictionFeedforwardScale);
 mappings={'forceKp','forceKp';'forceKi','forceKi';'forceKd','forceKd'; ...
  'positionBandwidth_Hz','positionBandwidth_Hz'; ...
  'accelerationFeedforwardScale','accelerationFeedforwardScale'; ...
  'guideFrictionFeedforwardScale','guideFrictionFeedforwardScale'};
 for j=1:size(mappings,1),if isfield(one,mappings{j,1}),p.inertial.(mappings{j,2})=one.(mappings{j,1});end,end
 mappings={'accelFilterTime_s','filterTime';'accelDelay_s','delay'; ...
  'accelNoiseRms_m_s2','noiseRms';'accelBias_m_s2','bias';'accelScaleError','scaleError'};
 for j=1:size(mappings,1),if isfield(one,mappings{j,1}),p.accel.(mappings{j,2})=one.(mappings{j,1});end,end
 if one.clean,p.accel.noiseRms=0;end
 if isfield(one,'forceFilterTime_s'),p.sensor.filterTime=one.forceFilterTime_s;end
end
p=acf_prepare(p);
p.contact.lateralDelaySamples=p.d.sensorDelaySamples;
end

function prepare_inertial_helpers(folder)
% This coefficient calculation is for a native 2-state motion observer only.
% Contact reconstruction and the PI/PID law remain ordinary diagram blocks.
path=fullfile(folder,'acf_prepare.m');source=fileread(path);
marker='p.d.coilControl=coil_controller(p);';assert(contains(source,marker));
addition=[marker newline ...
 'if isfield(p,''accel'') && p.accel.enabled' newline ...
 ' dt=p.force.sampleTime;rho=exp(-2*pi*p.inertial.positionBandwidth_Hz*dt);' newline ...
 ' Fm=[1,dt;0,1];Lm=[1-rho^2;(1-rho)^2/dt];Jm=eye(2)-Lm*[1,0];Ga=[dt^2/2;dt];' newline ...
 ' p.d.toolMotionObserver=struct(''A'',Fm*Jm,''B'',[Fm*Lm,Ga,-Ga],''C'',Jm,''D'',[Lm,zeros(2,2)]);' newline ...
 ' p.accel.delaySamples=round(p.accel.delay/p.sensor.sampleTime);' newline ...
 ' assert(abs(p.accel.delay/p.sensor.sampleTime-p.accel.delaySamples)<1e-9);' newline ...
 'end'];
source=strrep(source,marker,addition);write_text(path,source);
path=fullfile(folder,'acf_select_actuator.m');source=fileread(path);
source=strrep(source,'Voice coil observer','Voice coil inertial control');write_text(path,source);
end

function add_normal_plant(mdl)
sys=[mdl '/Surface contact force'];blank(sys,[1280 1870 1650 1990]);
% Explicit root-to-plant ports keep the physical reaction/profile contract
% independent of the controller's local force-estimate tags.
block('simulink/Ports & Subsystems/In1',sys,'Axial reaction',[20 53 50 67],'Port','1');
block('simulink/Ports & Subsystems/In1',sys,'Physical surface velocity',[20 153 50 167],'Port','2');
block('simulink/Signal Routing/Selector',sys,'Velocity component',[210 135 350 185], ...
 'InputPortWidth','3','IndexOptions','Index vector (dialog)','Indices','2','OutputSizes','1');
block('simulink/Math Operations/Gain',sys,'Surface slope',[390 135 535 185],'Gain','1/p.surface.scanSpeed');
link(sys,'Physical surface velocity/1','Velocity component/1');link(sys,'Velocity component/1','Surface slope/1');
root_norm(sys,'Surface slope',[590 130]);
block('simulink/Math Operations/Gain',sys,'Friction times slope',[590 280 765 330],'Gain','p.contact.mu');
block('simulink/Sources/Constant',sys,'Unity',[590 365 690 405],'Value','1');
block('simulink/Math Operations/Sum',sys,'Axial reaction factor',[815 285 855 355],'Inputs','+-');
block('simulink/Math Operations/Product',sys,'Normal reaction',[1050 35 1170 110],'Inputs','**/');
link(sys,'Surface slope/1','Friction times slope/1');link(sys,'Unity/1','Axial reaction factor/1');
link(sys,'Friction times slope/1','Axial reaction factor/2');
link(sys,'Axial reaction/1','Normal reaction/1');link(sys,'Normalizer/1','Normal reaction/2');
link(sys,'Axial reaction factor/1','Normal reaction/3');
block('simulink/Math Operations/Bias',sys,'Slope plus friction',[820 450 1025 500],'Bias','p.contact.mu');
block('simulink/Math Operations/Product',sys,'Lateral reaction',[1250 420 1385 495],'Inputs','**/');
block('simulink/Math Operations/Gain',sys,'Lateral sign',[1465 435 1585 485],'Gain','-1');
link(sys,'Surface slope/1','Slope plus friction/1');link(sys,'Normal reaction/1','Lateral reaction/1');
link(sys,'Slope plus friction/1','Lateral reaction/2');link(sys,'Normalizer/1','Lateral reaction/3');
link(sys,'Lateral reaction/1','Lateral sign/1');
goto(sys,'Normal force','normal_contact_N',[1250 45 1400 75]);link(sys,'Normal reaction/1','Normal force/1');
goto(sys,'Lateral force','lateral_contact_N',[1660 435 1810 465]);link(sys,'Lateral sign/1','Lateral force/1');
goto(sys,'Actual slope','actual_contact_slope',[840 210 990 240]);link(sys,'Surface slope/1','Actual slope/1');
link(mdl,'Actuator and RABS mechanics/1','Surface contact force/1');
link(mdl,'Surface and nominal path/1','Surface contact force/2');
end

function add_lateral_sensor(mdl)
sys=[mdl '/Lateral B-S force measurement'];blank(sys,[1760 1870 2110 1990]);
from(sys,'Lateral reaction','lateral_contact_N',[20 45 175 75]);
block('simulink/Discrete/Zero-Order Hold',sys,'Sensor sampling',[235 35 405 85],'SampleTime','p.sensor.sampleTime');
block('simulink/Math Operations/Gain',sys,'New sample weight',[475 35 690 85], ...
 'Gain','1-exp(-p.sensor.sampleTime/max(p.sensor.filterTime,realmin))*double(p.sensor.filterTime>0)');
block('simulink/Math Operations/Sum',sys,'Sensor low pass',[780 35 820 105],'Inputs','++');
block('simulink/Discrete/Unit Delay',sys,'Filter memory',[955 35 1115 85], ...
 'SampleTime','p.sensor.sampleTime','InitialCondition','-p.force.reference*p.contact.mu');
block('simulink/Math Operations/Gain',sys,'Old sample weight',[1165 35 1380 85], ...
 'Gain','exp(-p.sensor.sampleTime/max(p.sensor.filterTime,realmin))*double(p.sensor.filterTime>0)');
from(sys,'Previous filtered force','normal_lateral_filter_local',[20 145 220 175]);
goto(sys,'Filtered force memory','normal_lateral_filter_local',[1450 45 1650 75],false);
link(sys,'Lateral reaction/1','Sensor sampling/1');link(sys,'Sensor sampling/1','New sample weight/1');
link(sys,'New sample weight/1','Sensor low pass/1');link(sys,'Sensor low pass/1','Filter memory/1');
link(sys,'Filter memory/1','Old sample weight/1');link(sys,'Old sample weight/1','Filtered force memory/1');
link(sys,'Previous filtered force/1','Sensor low pass/2');
block('simulink/Sources/Uniform Random Number',sys,'Independent lateral noise',[20 270 240 320], ...
 'Minimum','-sqrt(3)','Maximum','sqrt(3)','Seed','p.sensor.seed+197','SampleTime','p.sensor.sampleTime');
block('simulink/Math Operations/Gain',sys,'Noise RMS',[310 270 500 320],'Gain','p.contact.lateralNoiseRms_N');
block('simulink/Sources/Constant',sys,'Known tare residual bias',[535 390 735 430],'Value','p.contact.lateralBias_N');
block('simulink/Math Operations/Sum',sys,'Noisy tared lateral force',[780 260 820 350],'Inputs','+++');
block('simulink/Discrete/Delay',sys,'Sensor sample delay',[955 270 1150 330], ...
 'DelayLength','p.contact.lateralDelaySamples','InitialCondition','-p.force.reference*p.contact.mu', ...
 'SampleTime','p.sensor.sampleTime');
goto(sys,'Measured lateral force','measured_lateral_contact_N',[1230 280 1430 310]);
link(sys,'Independent lateral noise/1','Noise RMS/1');link(sys,'Sensor low pass/1','Noisy tared lateral force/1');
link(sys,'Noise RMS/1','Noisy tared lateral force/2');link(sys,'Known tare residual bias/1','Noisy tared lateral force/3');
link(sys,'Noisy tared lateral force/1','Sensor sample delay/1');link(sys,'Sensor sample delay/1','Measured lateral force/1');
% A bounded lead uses only present/past measured force. It can compensate an
% identified channel delay without any surface/drag model in the controller.
block('simulink/Discrete/Unit Delay',sys,'Previous measured lateral force',[20 640 265 690], ...
 'SampleTime','p.sensor.sampleTime','InitialCondition','-p.force.reference*p.contact.mu');
block('simulink/Math Operations/Sum',sys,'Measured lateral force increment',[340 625 380 705],'Inputs','+-');
block('simulink/Math Operations/Gain',sys,'Filtered derivative new sample',[465 625 720 680], ...
 'Gain','(1-exp(-p.sensor.sampleTime/p.contact.lateralDerivativeFilterTime_s))/p.sensor.sampleTime');
block('simulink/Math Operations/Sum',sys,'Filtered lateral force derivative',[800 620 840 700],'Inputs','++');
block('simulink/Discrete/Unit Delay',sys,'Previous lateral derivative',[965 620 1195 680], ...
 'SampleTime','p.sensor.sampleTime','InitialCondition','0');
block('simulink/Math Operations/Gain',sys,'Filtered derivative memory',[965 765 1210 820], ...
 'Gain','exp(-p.sensor.sampleTime/p.contact.lateralDerivativeFilterTime_s)');
block('simulink/Math Operations/Gain',sys,'Identified lateral latency',[1310 620 1520 680], ...
 'Gain','p.contact.lateralPredictionTime_s');
block('simulink/Math Operations/Sum',sys,'Present lateral force estimate',[1600 625 1640 705],'Inputs','++');
goto(sys,'Aligned lateral force','predicted_lateral_contact_N',[1730 640 1950 670]);
link(sys,'Sensor sample delay/1','Previous measured lateral force/1');
link(sys,'Sensor sample delay/1','Measured lateral force increment/1');
link(sys,'Previous measured lateral force/1','Measured lateral force increment/2');
link(sys,'Measured lateral force increment/1','Filtered derivative new sample/1');
link(sys,'Filtered derivative new sample/1','Filtered lateral force derivative/1');
link(sys,'Filtered lateral force derivative/1','Previous lateral derivative/1');
link(sys,'Previous lateral derivative/1','Filtered derivative memory/1');
link(sys,'Filtered derivative memory/1','Filtered lateral force derivative/2');
link(sys,'Filtered lateral force derivative/1','Identified lateral latency/1');
link(sys,'Sensor sample delay/1','Present lateral force estimate/1');
link(sys,'Identified lateral latency/1','Present lateral force estimate/2');
link(sys,'Present lateral force estimate/1','Aligned lateral force/1');
end

function add_predicted_velocity(mdl)
block('simulink/Signal Routing/Selector',mdl,'Predicted robot velocity',[905 1720 1110 1780], ...
 'InputPortWidth','3','IndexOptions','Index vector (dialog)','Indices','2','OutputSizes','1');
block('simulink/Discrete/Zero-Order Hold',mdl,'Predicted velocity sampling',[1175 1720 1375 1770], ...
 'SampleTime','p.force.sampleTime');
goto(mdl,'Issued-command predicted velocity','command_based_robot_velocity',[1465 1735 1665 1765]);
link(mdl,'Issued-command robot predictor/1','Predicted robot velocity/1');
link(mdl,'Predicted robot velocity/1','Predicted velocity sampling/1');
link(mdl,'Predicted velocity sampling/1','Issued-command predicted velocity/1');
end

function add_accelerometer(mdl)
sys=[mdl '/S accelerometer and ADC'];blank(sys,[1280 2080 1650 2200]);
block('simulink/Ports & Subsystems/In1',sys,'Physical S motion',[20 38 50 52],'Port','1');
block('simulink/Signal Routing/Selector',sys,'Physical S acceleration Z',[110 20 325 70], ...
 'InputPortWidth','3','IndexOptions','Index vector (dialog)','Indices','3','OutputSizes','1');
link(sys,'Physical S motion/1','Physical S acceleration Z/1');
block('simulink/Sources/Constant',sys,'Physical S acceleration X',[110 500 325 550],'Value','0');
for j=1:2
 axes={'Z','X'};axis=axes{j};y=40+(j-1)*480;
 g='p.d.g';if j==2,g='p.mechanics.gravity*sin(p.mechanics.angle)';end
 gravity=[axis ' gravity'];proper=[axis ' specific acceleration'];scaled=[axis ' sensor scale'];
 sampled=[axis ' sensor sampling'];noise=[axis ' raw noise'];gain=[axis ' raw noise RMS'];
 total=[axis ' noisy sensor'];fresh=[axis ' new sample weight'];memory=[axis ' filter memory'];
 previous=[axis ' old sample weight'];filtered=[axis ' identified sensor response'];
 quantum=[axis ' ADC quantization'];tare=[axis ' known static tare'];dynamic=[axis ' tared acceleration'];
 bias=[axis ' drift after tare'];biased=[axis ' unknown drift'];delayed=[axis ' known channel delay'];
 block('simulink/Sources/Constant',sys,gravity,[110 y+95 325 y+135],'Value',g);
 block('simulink/Math Operations/Sum',sys,proper,[385 y-5 425 y+75],'Inputs','++');
 block('simulink/Math Operations/Gain',sys,scaled,[485 y 680 y+50],'Gain','1+p.accel.scaleError');
 block('simulink/Discrete/Zero-Order Hold',sys,sampled,[750 y 930 y+50],'SampleTime','p.sensor.sampleTime');
 block('simulink/Sources/Random Number',sys,noise,[485 y+130 680 y+180], ...
  'Mean','0','Variance','1','Seed',sprintf('p.sensor.seed+%d',239+97*(j-1)), ...
  'SampleTime','p.sensor.sampleTime');
 block('simulink/Math Operations/Gain',sys,gain,[750 y+130 980 y+180], ...
  'Gain','p.accel.noiseRms*sqrt((1+exp(-p.sensor.sampleTime/p.accel.filterTime))/(1-exp(-p.sensor.sampleTime/p.accel.filterTime)))');
 block('simulink/Math Operations/Sum',sys,total,[1040 y-5 1080 y+75],'Inputs','++');
 block('simulink/Math Operations/Gain',sys,fresh,[1160 y 1365 y+50], ...
  'Gain','1-exp(-p.sensor.sampleTime/p.accel.filterTime)');
 block('simulink/Math Operations/Sum',sys,filtered,[1440 y-5 1480 y+75],'Inputs','++');
 block('simulink/Discrete/Unit Delay',sys,memory,[1540 y+160 1760 y+210], ...
  'SampleTime','p.sensor.sampleTime','InitialCondition',['(1+p.accel.scaleError)*(' g ')']);
 block('simulink/Math Operations/Gain',sys,previous,[1170 y+160 1390 y+210], ...
  'Gain','exp(-p.sensor.sampleTime/p.accel.filterTime)');
 block('simulink/Discontinuities/Saturation',sys,[axis ' acceleration range'],[1550 y 1730 y+50], ...
  'UpperLimit','p.accel.range_g*p.mechanics.gravity','LowerLimit','-p.accel.range_g*p.mechanics.gravity');
 block('simulink/Discontinuities/Quantizer',sys,quantum,[1800 y 2000 y+50], ...
  'QuantizationInterval','p.accel.quantum');
 block('simulink/Sources/Constant',sys,tare,[1810 y+110 2055 y+160], ...
  'Value',['round((1+p.accel.scaleError)*(' g ')/p.accel.quantum)*p.accel.quantum']);
 block('simulink/Math Operations/Sum',sys,dynamic,[2120 y-5 2160 y+75],'Inputs','+-');
 block('simulink/Sources/Constant',sys,bias,[2120 y+140 2320 y+180],'Value','p.accel.bias');
 block('simulink/Math Operations/Sum',sys,biased,[2400 y-5 2440 y+75],'Inputs','++');
 block('simulink/Discrete/Delay',sys,delayed,[2520 y 2745 y+60], ...
  'DelayLength','p.accel.delaySamples','InitialCondition','p.accel.bias','SampleTime','p.sensor.sampleTime');
 goto(sys,[axis ' measured acceleration'],['measured_S_acceleration_' lower(axis)],[2830 y+10 3070 y+40]);
 source=['Physical S acceleration ' axis];
 goto(sys,[axis ' physical acceleration'],['physical_S_acceleration_' lower(axis)],[20 y+235 285 y+265]);
 link(sys,[source '/1'],[axis ' physical acceleration/1']);
 link(sys,[source '/1'],[proper '/1']);link(sys,[gravity '/1'],[proper '/2']);
 link(sys,[proper '/1'],[scaled '/1']);link(sys,[scaled '/1'],[sampled '/1']);
 link(sys,[sampled '/1'],[total '/1']);link(sys,[noise '/1'],[gain '/1']);
 link(sys,[gain '/1'],[total '/2']);link(sys,[total '/1'],[fresh '/1']);
 link(sys,[fresh '/1'],[filtered '/1']);link(sys,[filtered '/1'],[memory '/1']);
 link(sys,[memory '/1'],[previous '/1']);link(sys,[previous '/1'],[filtered '/2']);
 link(sys,[filtered '/1'],[axis ' acceleration range/1']);
 link(sys,[axis ' acceleration range/1'],[quantum '/1']);link(sys,[quantum '/1'],[dynamic '/1']);
 link(sys,[tare '/1'],[dynamic '/2']);link(sys,[dynamic '/1'],[biased '/1']);
 link(sys,[bias '/1'],[biased '/2']);link(sys,[biased '/1'],[delayed '/1']);
 link(sys,[delayed '/1'],[axis ' measured acceleration/1']);
end
link(mdl,'Surface and nominal path/1','S accelerometer and ADC/1');
end

function add_inertial_control(mdl)
controller=[mdl '/Force controller'];sys=[controller '/Voice coil inertial control'];
blank(sys,[235 685 565 805]);set_param(sys,'VariantControl','Voice coil inertial control');
block('simulink/Ports & Subsystems/In1',sys,'error_N',[20 33 50 47],'Port','1');
block('simulink/Ports & Subsystems/In1',sys,'measured_travel_m',[20 393 50 407],'Port','2');
block('simulink/Math Operations/Gain',sys,'Cell error to deviation',[120 20 320 70],'Gain','-1');
block('simulink/Math Operations/Bias',sys,'Tared axial cell force',[385 20 610 70],'Bias','p.force.reference');
from(sys,'Measured S acceleration Z','measured_S_acceleration_z',[20 170 265 200]);
from(sys,'Measured S acceleration X','measured_S_acceleration_x',[20 285 265 315]);
from(sys,'Measured lateral cell force','measured_lateral_contact_N',[390 285 635 315]);
block('simulink/Math Operations/Gain',sys,'Downstream Z inertia',[385 155 610 210],'Gain','p.sensor.distalMass_model_kg');
block('simulink/Math Operations/Gain',sys,'Downstream X inertia',[705 275 935 330],'Gain','p.sensor.distalMass_model_kg');
block('simulink/Math Operations/Sum',sys,'Compensated axial reaction',[710 15 750 95],'Inputs','++');
block('simulink/Math Operations/Sum',sys,'Compensated lateral reaction',[1010 265 1050 345],'Inputs','++');
goto(sys,'Direct axial reaction','inertial_axial_contact_N',[850 35 1070 65]);
goto(sys,'Direct lateral reaction','inertial_lateral_contact_N',[1130 280 1350 310]);
link(sys,'error_N/1','Cell error to deviation/1');link(sys,'Cell error to deviation/1','Tared axial cell force/1');
link(sys,'Tared axial cell force/1','Compensated axial reaction/1');
link(sys,'Measured S acceleration Z/1','Downstream Z inertia/1');link(sys,'Downstream Z inertia/1','Compensated axial reaction/2');
link(sys,'Measured S acceleration X/1','Downstream X inertia/1');link(sys,'Downstream X inertia/1','Compensated lateral reaction/2');
link(sys,'Measured lateral cell force/1','Compensated lateral reaction/1');
link(sys,'Compensated axial reaction/1','Direct axial reaction/1');link(sys,'Compensated lateral reaction/1','Direct lateral reaction/1');
block('simulink/Math Operations/Bias',sys,'Measured head coordinate deviation',[120 375 390 425], ...
 'Bias','p.d.positionMeasurementBias');
block('simulink/Math Operations/Gain',sys,'Head gap coordinate sign',[465 375 685 425],'Gain','-1');
from(sys,'Issued robot acceleration','command_based_robot_acceleration',[465 485 720 515]);
block('simulink/Signal Routing/Mux',sys,'Causal motion measurements',[805 370 810 595],'Inputs','3');
block('simulink/Discrete/Discrete State-Space',sys,'Head gap and velocity observer',[900 410 1215 530], ...
 'A','p.d.toolMotionObserver.A','B','p.d.toolMotionObserver.B','C','p.d.toolMotionObserver.C', ...
 'D','p.d.toolMotionObserver.D','InitialCondition','zeros(2,1)','SampleTime','p.force.sampleTime');
goto(sys,'Head motion states','inertial_head_motion_states',[1300 450 1540 480]);
link(sys,'measured_travel_m/1','Measured head coordinate deviation/1');
link(sys,'Measured head coordinate deviation/1','Head gap coordinate sign/1');
positionFiltered=add_alignment_filter(sys,'Head gap coordinate sign/1','Position frame matching',[20 660],false);
robotAccelerationFiltered=add_alignment_filter(sys,'Issued robot acceleration/1','Robot acceleration frame matching',[1600 660],true);
from(sys,'Issued robot velocity','command_based_robot_velocity',[20 2100 285 2130]);
robotVelocityFiltered=add_alignment_filter(sys,'Issued robot velocity/1','Robot velocity frame matching',[385 2080],true);
goto(sys,'Aligned robot velocity','inertial_robot_velocity_aligned',[1180 2085 1440 2115]);
link(sys,robotVelocityFiltered,'Aligned robot velocity/1');
link(sys,positionFiltered,'Causal motion measurements/1');
link(sys,robotAccelerationFiltered,'Causal motion measurements/2');
link(sys,'Measured S acceleration Z/1','Causal motion measurements/3');
link(sys,'Causal motion measurements/1','Head gap and velocity observer/1');link(sys,'Head gap and velocity observer/1','Head motion states/1');
block('simulink/Signal Routing/Selector',sys,'Head gap velocity',[1640 410 1840 465], ...
 'InputPortWidth','2','IndexOptions','Index vector (dialog)','Indices','2','OutputSizes','1');
block('simulink/Signal Routing/Selector',sys,'Head gap displacement',[1640 530 1840 585], ...
 'InputPortWidth','2','IndexOptions','Index vector (dialog)','Indices','1','OutputSizes','1');
block('simulink/Math Operations/Bias',sys,'Estimated head travel',[1920 530 2160 585], ...
 'Bias','p.mechanics.midTravel+p.d.u0');
block('simulink/Ports & Subsystems/Out1',sys,'estimated_travel_m',[2240 548 2270 562],'Port','2');
link(sys,'Head gap and velocity observer/1','Head gap velocity/1');link(sys,'Head gap and velocity observer/1','Head gap displacement/1');
link(sys,'Head gap displacement/1','Estimated head travel/1');link(sys,'Estimated head travel/1','estimated_travel_m/1');
projection=[sys '/Normal force projection'];blank(projection,[720 760 1070 900]);
add_inertial_projection(projection);
link(sys,'Compensated axial reaction/1','Normal force projection/1');
link(sys,'Compensated lateral reaction/1','Normal force projection/2');
link(sys,'Head gap velocity/1','Normal force projection/3');
block('simulink/Math Operations/Gain',sys,'Contact error for robot gate',[1200 760 1460 815],'Gain','-1');
block('simulink/Ports & Subsystems/Out1',sys,'estimated_force_error_N',[1540 778 1570 792],'Port','3');
link(sys,'Normal force projection/1','Contact error for robot gate/1');link(sys,'Contact error for robot gate/1','estimated_force_error_N/1');
block('simulink/Math Operations/Gain',sys,'Normal force proportional current',[1150 950 1450 1010],'Gain','-p.inertial.forceKp');
block('simulink/Math Operations/Sum',sys,'Normal error and anti windup',[460 975 500 1055],'Inputs','++');
block('simulink/Discrete/Discrete-Time Integrator',sys,'Normal force error integral',[585 985 880 1040], ...
 'SampleTime','p.force.sampleTime','InitialCondition','0');
block('simulink/Math Operations/Gain',sys,'Normal force integral current',[955 1080 1225 1135],'Gain','-p.inertial.forceKi');
block('simulink/Sources/Constant',sys,'Flat holding current',[1490 930 1720 975],'Value','p.d.coilHoldCurrent');
block('simulink/Math Operations/Gain',sys,'Measured acceleration current feedforward',[1150 1190 1490 1250], ...
 'Gain','-p.inertial.accelerationFeedforwardScale*(p.mechanics.mB+p.mechanics.mS)/p.coil.forceConstant');
block('simulink/Math Operations/Gain',sys,'Geometry current feedforward',[1150 1300 1490 1360], ...
 'Gain','p.contact.geometryFeedforwardScale/p.coil.forceConstant');
block('simulink/Math Operations/Gain',sys,'Guide viscous friction current',[1660 1280 1950 1335], ...
 'Gain','p.inertial.guideFrictionFeedforwardScale*p.coil.viscousFriction/p.coil.forceConstant');
block('simulink/Math Operations/Gain',sys,'Guide friction velocity scale',[1660 1390 1950 1445],'Gain','1/p.coil.frictionSpeed');
block('simulink/Math Operations/Trigonometric Function',sys,'Guide friction direction',[2020 1390 2270 1445],'Operator','tanh');
block('simulink/Math Operations/Gain',sys,'Guide Coulomb friction current',[2340 1390 2640 1445], ...
 'Gain','p.inertial.guideFrictionFeedforwardScale*p.coil.coulombForce/p.coil.forceConstant');
block('simulink/Math Operations/Sum',sys,'Guide friction feedforward',[2730 1275 2770 1355],'Inputs','++');
derivative=add_filtered_derivative(sys,'Normal force projection/1','Normal error','p.inertial.derivativeFilter','0',[20 1570]);
block('simulink/Math Operations/Gain',sys,'Normal force derivative current',[1220 1570 1530 1625],'Gain','-p.inertial.forceKd');
link(sys,derivative,'Normal force derivative current/1');
block('simulink/Math Operations/Sum',sys,'PI PID and feedforward current',[2900 955 2940 1485],'Inputs','+++++++');
block('simulink/Discontinuities/Saturation',sys,'Current limits',[3040 1165 3270 1225], ...
 'UpperLimit','p.coil.peakCurrent','LowerLimit','-p.coil.peakCurrent');
block('simulink/Ports & Subsystems/Out1',sys,'command',[3350 1188 3380 1202],'Port','1');
block('simulink/Math Operations/Sum',sys,'Current saturation error',[3160 1510 3200 1590],'Inputs','+-');
block('simulink/Math Operations/Gain',sys,'Integral anti windup',[2840 1670 3150 1725], ...
 'Gain','-p.force.antiWindupRate/p.inertial.forceKi');
link(sys,'Normal force projection/1','Normal force proportional current/1');
link(sys,'Normal force projection/1','Normal error and anti windup/1');
link(sys,'Normal error and anti windup/1','Normal force error integral/1');
link(sys,'Normal force error integral/1','Normal force integral current/1');
link(sys,'Measured S acceleration Z/1','Measured acceleration current feedforward/1');
link(sys,'Normal force projection/2','Geometry current feedforward/1');
link(sys,'Head gap velocity/1','Guide viscous friction current/1');
link(sys,'Head gap velocity/1','Guide friction velocity scale/1');
link(sys,'Guide friction velocity scale/1','Guide friction direction/1');
link(sys,'Guide friction direction/1','Guide Coulomb friction current/1');
link(sys,'Guide viscous friction current/1','Guide friction feedforward/1');
link(sys,'Guide Coulomb friction current/1','Guide friction feedforward/2');
sources={'Flat holding current/1','Normal force proportional current/1','Normal force integral current/1', ...
 'Measured acceleration current feedforward/1','Geometry current feedforward/1','Guide friction feedforward/1', ...
 'Normal force derivative current/1'};
for j=1:numel(sources),link(sys,sources{j},sprintf('PI PID and feedforward current/%d',j));end
link(sys,'PI PID and feedforward current/1','Current limits/1');link(sys,'Current limits/1','command/1');
link(sys,'Current limits/1','Current saturation error/1');link(sys,'PI PID and feedforward current/1','Current saturation error/2');
link(sys,'Current saturation error/1','Integral anti windup/1');link(sys,'Integral anti windup/1','Normal error and anti windup/2');
set_param(controller,'LabelModeActiveChoice','Voice coil inertial control');
end

function add_inertial_projection(sys)
for j=1:3
 names={'Compensated axial force','Compensated lateral force','Head gap velocity'};
 block('simulink/Ports & Subsystems/In1',sys,names{j},[20 43+110*(j-1) 50 57+110*(j-1)],'Port',num2str(j));
end
from(sys,'Issued robot velocity','inertial_robot_velocity_aligned',[20 385 240 415]);
block('simulink/Math Operations/Sum',sys,'Estimated S velocity',[305 260 345 340],'Inputs','+-');
block('simulink/Math Operations/Gain',sys,'Causal surface direction',[430 265 690 325], ...
 'Gain','1/p.contact.commandedScanSpeed_m_s');
link(sys,'Issued robot velocity/1','Estimated S velocity/1');link(sys,'Head gap velocity/1','Estimated S velocity/2');
link(sys,'Estimated S velocity/1','Causal surface direction/1');root_norm(sys,'Causal surface direction',[770 260]);
block('simulink/Math Operations/Product',sys,'Lateral projection',[775 115 1030 180],'Inputs','**');
block('simulink/Math Operations/Sum',sys,'Normal projection numerator',[1110 20 1150 100],'Inputs','+-');
block('simulink/Math Operations/Product',sys,'Measured normal force',[1440 20 1680 95],'Inputs','*/');
block('simulink/Math Operations/Bias',sys,'Measured normal force deviation',[1770 20 2060 75],'Bias','-p.force.reference');
block('simulink/Ports & Subsystems/Out1',sys,'normal_force_deviation_N',[2150 38 2180 52],'Port','1');
link(sys,'Compensated lateral force/1','Lateral projection/1');link(sys,'Causal surface direction/1','Lateral projection/2');
link(sys,'Compensated axial force/1','Normal projection numerator/1');link(sys,'Lateral projection/1','Normal projection numerator/2');
link(sys,'Normal projection numerator/1','Measured normal force/1');link(sys,'Normalizer/1','Measured normal force/2');
link(sys,'Measured normal force/1','Measured normal force deviation/1');link(sys,'Measured normal force deviation/1','normal_force_deviation_N/1');
block('simulink/Math Operations/Gain',sys,'Normal reference projection',[1450 420 1690 475],'Gain','p.force.reference');
block('simulink/Math Operations/Sum',sys,'Requested axial force',[1770 415 1810 495],'Inputs','++');
block('simulink/Math Operations/Bias',sys,'Axial reference deviation',[1900 420 2180 475],'Bias','-p.force.reference');
block('simulink/Ports & Subsystems/Out1',sys,'axial_reference_deviation_N',[2260 438 2290 452],'Port','2');
link(sys,'Normalizer/1','Normal reference projection/1');link(sys,'Normal reference projection/1','Requested axial force/1');
link(sys,'Lateral projection/1','Requested axial force/2');link(sys,'Requested axial force/1','Axial reference deviation/1');
link(sys,'Axial reference deviation/1','axial_reference_deviation_N/1');
goto(sys,'Measured normal reaction','estimated_normal_contact_N',[1770 140 2020 170]);
goto(sys,'Estimated contact slope','estimated_contact_slope',[430 515 690 545]);
goto(sys,'Requested axial reaction','requested_axial_contact_N',[1880 550 2130 580]);
link(sys,'Measured normal force/1','Measured normal reaction/1');link(sys,'Causal surface direction/1','Estimated contact slope/1');
link(sys,'Requested axial force/1','Requested axial reaction/1');
end

function output=add_filtered_derivative(sys,input,prefix,timeConstant,initial,xy)
x=xy(1);y=xy(2);prior=[prefix ' previous sample'];delta=[prefix ' sample increment'];
new=[prefix ' new derivative weight'];sum=[prefix ' filtered derivative'];
memory=[prefix ' derivative memory'];old=[prefix ' old derivative weight'];
block('simulink/Discrete/Unit Delay',sys,prior,[x y+110 x+235 y+165], ...
 'SampleTime','p.force.sampleTime','InitialCondition',initial);
block('simulink/Math Operations/Sum',sys,delta,[x+305 y x+345 y+80],'Inputs','+-');
block('simulink/Math Operations/Gain',sys,new,[x+415 y x+660 y+55], ...
 'Gain',['(1-exp(-p.force.sampleTime/' timeConstant '))/p.force.sampleTime']);
block('simulink/Math Operations/Sum',sys,sum,[x+740 y x+780 y+80],'Inputs','++');
block('simulink/Discrete/Unit Delay',sys,memory,[x+860 y+110 x+1100 y+165], ...
 'SampleTime','p.force.sampleTime','InitialCondition','0');
block('simulink/Math Operations/Gain',sys,old,[x+415 y+110 x+660 y+165], ...
 'Gain',['exp(-p.force.sampleTime/' timeConstant ')']);
link(sys,input,[prior '/1']);link(sys,input,[delta '/1']);link(sys,[prior '/1'],[delta '/2']);
link(sys,[delta '/1'],[new '/1']);link(sys,[new '/1'],[sum '/1']);
link(sys,[sum '/1'],[memory '/1']);link(sys,[memory '/1'],[old '/1']);link(sys,[old '/1'],[sum '/2']);
output=[sum '/1'];
end

function output=add_alignment_filter(sys,input,name,xy,withDelay)
x=xy(1);y=xy(2);
% Current/past-only scalar low pass, identical to the force-channel recursion.
block('simulink/Discrete/Discrete State-Space',sys,name,[x y x+340 y+70], ...
 'A','exp(-p.sensor.sampleTime/p.sensor.filterTime)', ...
 'B','1-exp(-p.sensor.sampleTime/p.sensor.filterTime)', ...
 'C','exp(-p.sensor.sampleTime/p.sensor.filterTime)', ...
 'D','1-exp(-p.sensor.sampleTime/p.sensor.filterTime)', ...
 'InitialCondition','0','SampleTime','p.sensor.sampleTime');
link(sys,input,[name '/1']);output=[name '/1'];
if withDelay
 delay=[name ' delay'];block('simulink/Discrete/Delay',sys,delay,[x+415 y x+705 y+70], ...
  'DelayLength','p.d.sensorDelaySamples','InitialCondition','0','SampleTime','p.sensor.sampleTime');
 link(sys,output,[delay '/1']);output=[delay '/1'];
end
end

function add_inertial_logs(mdl)
sys=[mdl '/Inertial sensor recordings'];blank(sys,[1760 2080 2110 2200]);
tags={'measured_S_acceleration_z','measured_S_acceleration_x', ...
 'physical_S_acceleration_z','physical_S_acceleration_x', ...
 'inertial_axial_contact_N','inertial_lateral_contact_N'};
block('simulink/Signal Routing/Mux',sys,'Acceleration and reconstructed force',[735 30 740 625],'Inputs','6');
for j=1:6
 name=sprintf('Measured channel %d',j);hold=sprintf('Record every physical step %d',j);
 from(sys,name,tags{j},[20 40+100*(j-1) 310 70+100*(j-1)]);
 block('simulink/Discrete/Zero-Order Hold',sys,hold,[395 30+100*(j-1) 665 85+100*(j-1)],'SampleTime','p.sim.step');
 link(sys,[name '/1'],[hold '/1']);link(sys,[hold '/1'],sprintf('Acceleration and reconstructed force/%d',j));
end
block('simulink/Sinks/To Workspace',sys,'Acceleration validation log',[820 300 1130 360], ...
 'VariableName','accelerometer_log','SaveFormat','Timeseries','MaxDataPoints','inf','SampleTime','p.sim.step');
link(sys,'Acceleration and reconstructed force/1','Acceleration validation log/1');
from(sys,'Head motion observer','inertial_head_motion_states',[20 755 310 785]);
block('simulink/Discrete/Zero-Order Hold',sys,'Head motion recording clock',[395 740 695 800],'SampleTime','p.sim.step');
block('simulink/Sinks/To Workspace',sys,'Motion observer validation log',[820 740 1130 800], ...
 'VariableName','inertial_motion_log','SaveFormat','Timeseries','MaxDataPoints','inf','SampleTime','p.sim.step');
link(sys,'Head motion observer/1','Head motion recording clock/1');
link(sys,'Head motion recording clock/1','Motion observer validation log/1');
end

function add_normal_control(mdl)
parent=[mdl '/Force controller/Voice coil observer'];
sys=[parent '/Normal force feedback'];blank(sys,[1090 1860 1440 1990]);
block('simulink/Ports & Subsystems/In1',sys,'Estimated axial deviation',[20 38 50 52],'Port','1');
block('simulink/Ports & Subsystems/In1',sys,'Estimated gap velocity',[20 168 50 182],'Port','2');
from(sys,'Predicted robot velocity','command_based_robot_velocity',[20 265 220 295]);
from(sys,'Measured lateral force','predicted_lateral_contact_N',[20 505 220 535]);
block('simulink/Math Operations/Bias',sys,'Estimated axial reaction',[130 20 360 75],'Bias','p.force.reference');
block('simulink/Math Operations/Sum',sys,'Estimated tool velocity',[285 155 325 235],'Inputs','+-');
block('simulink/Math Operations/Gain',sys,'Causal slope estimate',[415 155 605 210],'Gain','1/p.surface.scanSpeed');
link(sys,'Estimated axial deviation/1','Estimated axial reaction/1');
link(sys,'Predicted robot velocity/1','Estimated tool velocity/1');
link(sys,'Estimated gap velocity/1','Estimated tool velocity/2');
link(sys,'Estimated tool velocity/1','Causal slope estimate/1');
root_norm(sys,'Causal slope estimate',[675 150]);
block('simulink/Math Operations/Product',sys,'Slope times lateral force',[675 485 890 550],'Inputs','**');
link(sys,'Causal slope estimate/1','Slope times lateral force/1');
link(sys,'Measured lateral force/1','Slope times lateral force/2');
block('simulink/Math Operations/Sum',sys,'Normal force numerator',[960 25 1000 105],'Inputs','+-');
block('simulink/Math Operations/Product',sys,'Estimated normal force',[1120 25 1330 105],'Inputs','*/');
block('simulink/Math Operations/Bias',sys,'Normal force deviation',[1420 25 1625 80],'Bias','-p.force.reference');
block('simulink/Math Operations/Gain',sys,'Integral force scale',[1710 25 1870 80],'Gain','p.contact.normalIntegralScale');
link(sys,'Estimated axial reaction/1','Normal force numerator/1');
link(sys,'Slope times lateral force/1','Normal force numerator/2');
link(sys,'Normal force numerator/1','Estimated normal force/1');link(sys,'Normalizer/1','Estimated normal force/2');
link(sys,'Estimated normal force/1','Normal force deviation/1');link(sys,'Normal force deviation/1','Integral force scale/1');
block('simulink/Ports & Subsystems/Out1',sys,'Integral force deviation',[1940 43 1970 57],'Port','1');
link(sys,'Integral force scale/1','Integral force deviation/1');
block('simulink/Math Operations/Gain',sys,'Reference normal projection',[1120 280 1330 330],'Gain','p.force.reference');
block('simulink/Math Operations/Sum',sys,'Requested axial contact force',[1420 275 1460 355],'Inputs','++');
block('simulink/Math Operations/Bias',sys,'Axial reference change',[1540 275 1750 330],'Bias','-p.force.reference');
block('simulink/Math Operations/Gain',sys,'Geometry current correction',[1810 275 2080 330], ...
 'Gain','p.contact.geometryFeedforwardScale*(1-p.d.coilControl.feedback(7))/p.coil.forceConstant');
block('simulink/Ports & Subsystems/Out1',sys,'Geometry current_A',[2160 293 2190 307],'Port','2');
link(sys,'Normalizer/1','Reference normal projection/1');link(sys,'Reference normal projection/1','Requested axial contact force/1');
link(sys,'Slope times lateral force/1','Requested axial contact force/2');
link(sys,'Requested axial contact force/1','Axial reference change/1');
link(sys,'Axial reference change/1','Geometry current correction/1');link(sys,'Geometry current correction/1','Geometry current_A/1');
goto(sys,'Estimated normal reaction','estimated_normal_contact_N',[1450 130 1675 160]);
goto(sys,'Estimated normal slope','estimated_contact_slope',[760 385 970 415]);
goto(sys,'Requested axial reaction','requested_axial_contact_N',[1550 400 1780 430]);
link(sys,'Estimated normal force/1','Estimated normal reaction/1');
link(sys,'Causal slope estimate/1','Estimated normal slope/1');
link(sys,'Requested axial contact force/1','Requested axial reaction/1');
delete_line(parent,'Physical estimated force deviation/1','Integral and anti windup/1');
delete_line(parent,'Physical estimated force deviation/1','Send contact error estimate/1');
link(parent,'Physical estimated force deviation/1','Normal force feedback/1');
link(parent,'Estimated gap velocity/1','Normal force feedback/2');
link(parent,'Normal force feedback/1','Integral and anti windup/1');
link(parent,'Normal force feedback/1','Send contact error estimate/1');
set_param([parent '/Coil current request'],'Inputs','+++++');
link(parent,'Normal force feedback/2','Coil current request/5');
end

function add_normal_log(mdl)
tags={'normal_contact_N','lateral_contact_N','actual_contact_slope', ...
 'estimated_normal_contact_N','estimated_contact_slope','requested_axial_contact_N', ...
 'measured_lateral_contact_N','command_based_robot_velocity'};
block('simulink/Signal Routing/Mux',mdl,'Surface normal diagnostics',[2520 1870 2525 2550],'Inputs','8');
for j=1:numel(tags)
 from(mdl,sprintf('Normal diagnostic %d',j),tags{j},[1970 1875+80*(j-1) 2200 1905+80*(j-1)]);
 block('simulink/Discrete/Zero-Order Hold',mdl,sprintf('Normal logging sample %d',j), ...
  [2280 1865+80*(j-1) 2460 1915+80*(j-1)],'SampleTime','p.sim.step');
 link(mdl,sprintf('Normal diagnostic %d/1',j),sprintf('Normal logging sample %d/1',j));
 link(mdl,sprintf('Normal logging sample %d/1',j),sprintf('Surface normal diagnostics/%d',j));
end
block('simulink/Sinks/To Workspace',mdl,'Surface normal log',[2630 2175 2830 2230], ...
 'VariableName','normal_contact_log','SaveFormat','Timeseries','MaxDataPoints','inf','SampleTime','p.sim.step');
link(mdl,'Surface normal diagnostics/1','Surface normal log/1');
% Expose the native log in the private copied helper without modifying the
% authoritative helper or the established 35-channel physical log schema.
helper=fullfile(fileparts(get_param(mdl,'FileName')),'helpers','acf_run.m');
source=fileread(helper);marker='r.parameters=p; r.names=acf_signals; r.metrics=acf_metrics(r);';
assert(contains(source,marker));
existingStart=strfind(source,'nativeNormal=out.get(''normal_contact_log'');');
if ~isempty(existingStart)
 existingEnd=strfind(source,'if any(strcmp(who(out),''robot_prediction_observer_log''))');
 assert(numel(existingStart)==1 && numel(existingEnd)==1 && existingEnd>existingStart);
 source=[source(1:existingStart-1),source(existingEnd:end)];
end
source=strrep(source,marker,[marker newline ...
 'nativeNormal=out.get(''normal_contact_log'');' newline ...
 'r.normalContact=struct(''time'',nativeNormal.Time(:),''data'',rows(nativeNormal,8),' ...
 '''names'',{{''normalForce_N'',''lateralForce_N'',''trueSlope'',''estimatedNormalForce_N'',' ...
 '''estimatedSlope'',''requestedAxialForce_N'',''measuredLateralForce_N'',''predictedRobotVelocity_m_s''}});' newline ...
 'r.metrics.peakAbsNormalForceError_N=max(abs(r.normalContact.data(:,1)-p.force.reference));' newline ...
 'r.metrics.normalPass=r.metrics.peakAbsNormalForceError_N<=p.force.tolerance && ' ...
 '~any(r.data(:,18)<.5) && ~any(r.data(:,21)>.5) && r.time(end)>=p.sim.stopTime-p.sim.step;' newline ...
 'if any(strcmp(who(out),''accelerometer_log''))' newline ...
 ' inertialLog=out.get(''accelerometer_log'');r.accelerometer=struct(''time'',inertialLog.Time(:),''data'',rows(inertialLog,6));' newline ...
 ' motionLog=out.get(''inertial_motion_log'');r.motionObserver=struct(''time'',motionLog.Time(:),''data'',rows(motionLog,2));' newline ...
 'end']);
fid=fopen(helper,'w');assert(fid>0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',source); %#ok<NASGU>
clear acf_run;
end

function root_norm(sys,source,xy)
x=xy(1);y=xy(2);
block('simulink/Math Operations/Math Function',sys,'Squared slope',[x y x+145 y+55],'Operator','square');
block('simulink/Math Operations/Bias',sys,'Squared slope plus one',[x+195 y x+400 y+55],'Bias','1');
block('simulink/Math Operations/Math Function',sys,'Normalizer',[x+450 y x+610 y+55],'Operator','sqrt');
link(sys,[source '/1'],'Squared slope/1');link(sys,'Squared slope/1','Squared slope plus one/1');
link(sys,'Squared slope plus one/1','Normalizer/1');
end

function blank(sys,position)
add_block('simulink/Ports & Subsystems/Subsystem',sys,'Position',position);
Simulink.SubSystem.deleteContents(sys);
end
function block(library,sys,name,position,varargin)
add_block(library,[sys '/' name],'Position',position,varargin{:});
end
function from(sys,name,tag,position)
block('simulink/Signal Routing/From',sys,name,position,'GotoTag',tag);
end
function goto(sys,name,tag,position,globalTag)
if nargin<5,globalTag=true;end
visibility='local';if globalTag,visibility='global';end
block('simulink/Signal Routing/Goto',sys,name,position,'GotoTag',tag,'TagVisibility',visibility);
end
function link(sys,source,destination)
add_line(sys,source,destination,'autorouting','on');
end
function s=defaults(s,d)
names=fieldnames(d);for j=1:numel(names),if ~isfield(s,names{j}),s.(names{j})=d.(names{j});end,end
end
function write_text(path,source)
fid=fopen(path,'w');assert(fid>0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',source); %#ok<NASGU>
end
function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end
function d=rows(ts,width)
n=numel(ts.Time);d=ts.Data;
if size(d,1)~=n,d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function checkpoint(report,output)
save(fullfile(output,'normal_contact_trial_results.mat'),'report');
fid=fopen(fullfile(output,'normal_contact_trial_results.json'),'w');assert(fid>0);
cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',jsonencode(report)); %#ok<NASGU>
end
function finish(mdl,location,record)
try,set_param(mdl,'FastRestart','off');catch,end
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(location);Simulink.sdi.setRecordData(record);catch,end
end
