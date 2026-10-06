function report=prepare_budget_sensor_stress_20261002(root,outputFolder)
%PREPARE_BUDGET_SENSOR_STRESS_20261002 Preserve recorded joint residuals.
% Input preparation only. The root native runner executes these49 cases.
% Full actual force/acceleration timing residuals and calibration are retained.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,outputFolder=fullfile(root,'acf_simulink','validation','budget_sensor_stress_20261002_inputs');end
assert(~isfile(fullfile(outputFolder,'manifest.json')),'Preserve earlier prepared inputs.');
if ~isfolder(outputFolder),mkdir(outputFolder);end
validation=fullfile(root,'acf_simulink','validation');
source=fullfile(validation,'adc_acquisition_trial_20261002');
modelFile=fullfile(source,'acf_force_control_adc_trial.slx');modelSha=sha256(modelFile);
assert(strcmp(modelSha,'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
longFile=fullfile(validation,'inertial_uncertainty_v8_calibrated_static_long', ...
 'inertial_uncertainty_calibrated_static_long_20260930_prepared_configs.mat');
mixedFile=fullfile(validation,'inertial_mechanics_calibrated_static_mixed_source_20260930', ...
 'prepared_static_configurations.mat');
longSha=sha256(longFile);mixedSha=sha256(mixedFile);
assert(strcmp(longSha,'be9e5f65ff211ee7a1d086574da499a74dcfde5f5168a4aad585d9c7a3701a7a'));
assert(strcmp(mixedSha,'fe2376a79b1084096c3b808bf6cdda4aa68431671cb5dac1b3ab35d9c359726c'));
long=load(longFile,'configs');mixed=load(mixedFile,'configs');
assert(numel(long.configs)==4&&numel(mixed.configs)==3);
originalCases=cell(1,7);sourceFiles=cell(1,7);sourceHashes=cell(1,7);sourceIndices=zeros(1,7);
for k=1:7
 if k<=4,originalCases{k}=long.configs(k).p;sourceFiles{k}=longFile;sourceHashes{k}=longSha;sourceIndices(k)=k;
 else,originalCases{k}=mixed.configs(k-4).p;sourceFiles{k}=mixedFile;sourceHashes{k}=mixedSha;sourceIndices(k)=k-4;end
end
addpath(fullfile(source,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
g=9.80665;enbw=.06354253123515036/(2*10e-6);
referenceDensity=.014/g/sqrt(enbw)*1e6;
defs={ ...
 'ADXL354C_1900_L0_current100_24V','analog354',1900,33.75,8,0,100,24; ...
 'ADXL354C_1900_L0_current50_36V','analog354',1900,33.75,8,0,50,36; ...
 'ADXL354C_1900_L50_current50_36V','analog354',1900,33.75,8,50,50,36; ...
 'digital70_5100_L80_current100_24V','digital_envelope',5100,70,16,80,100,24; ...
 'digital70_5100_L80_current50_36V','digital_envelope',5100,70,16,80,50,36; ...
 'ADXL1002_reference_8000_L0_current100_24V','analog1002',8000,referenceDensity,50,0,100,24; ...
 'ADXL354C_1900_L0_current50_hot32V','analog354_hot_winding',1900,33.75,8,0,50,32};
groups=struct([]);rows=struct([]);
for groupIndex=1:size(defs,1)
 name=defs{groupIndex,1};kind=defs{groupIndex,2};cutoff=defs{groupIndex,3};
 density=defs{groupIndex,4};sensorRange=defs{groupIndex,5};latency_us=defs{groupIndex,6};
 current_us=defs{groupIndex,7};voltage=defs{groupIndex,8};configs=struct([]);
 for caseIndex=1:7
  before=originalCases{caseIndex};p=before;
  assert(p.acquisition.enabled&&p.accel.enabled&&isfield(p,'uncertainty'));
  if caseIndex<=4,assert(p.sim.stopTime==30);else,assert(p.sim.stopTime==4);end
  actualCommonBefore=p.acquisition.pipelineDelay_s;
  actualOffsetsBefore=[p.acquisition.forceDelayOffset_s,p.acquisition.lateralDelayOffset_s, ...
   p.acquisition.accelerationDelayOffset_s,p.acquisition.positionDelayOffset_s];
  p.sim.step=1/300000;p.sim.evaluateFrom=0;
  p.accel.range_g=sensorRange;
  if strcmp(kind,'analog1002'),p.accel.noiseRms=before.accel.noiseRms;
  else,p.accel.noiseRms=density*1e-6*g*sqrt(enbw);end
  p.accel.nominalNoiseRms=p.accel.noiseRms;
  p.coil.currentResponse=current_us*1e-6;p.coil.voltage=voltage;
  forceQ=320/2^24;forceTotal=21.31e-6/(1.2/160);
  if startsWith(kind,'analog354'),sensitivity=.1;
  elseif strcmp(kind,'analog1002'),sensitivity=.5*.04;
  else,sensitivity=NaN;end
  if isfinite(sensitivity)
   accQ=2.4/2^24/sensitivity*g;accTotal=21.31e-6/sensitivity*g;adcRange=1.2/sensitivity;
  else,accQ=32*g/2^16;accTotal=0;adcRange=sensorRange;end
  p.adcStudy=struct('fineClock_s',1/300000,'outputRate_Hz',30000,'converter','sinc3', ...
   'modulatorRate_Hz',3840000,'analogCutoff_Hz',cutoff,'identifiedAnalogCutoff_Hz',cutoff, ...
   'analogDamping',1/sqrt(2),'converterAnalogOrder',0,'converterAnalogCutoff_Hz',16000, ...
   'emaTime_s',50e-6,'identifiedEmaTime_s',50e-6, ...
   'pipelineDelay_s',actualCommonBefore+latency_us*1e-6, ...
   'identifiedPipelineDelay_s',20e-6+latency_us*1e-6, ...
   'forceQuantum_N',forceQ,'accelerationQuantum_m_s2',accQ, ...
   'forceAdcNoiseRms_N',sqrt(max(0,forceTotal^2-forceQ^2/12)), ...
   'accelerationAdcNoiseRms_m_s2',sqrt(max(0,accTotal^2-accQ^2/12)), ...
   'forceAdcRange_N',160,'accelerationAdcRange_g',adcRange, ...
   'adcNoiseCorrelationTime_s',0,'accelerationExtraDelay_s',0);
  hot=contains(kind,'hot_winding');oldK=before.coil.forceConstant;
  oldIdentifiedK=before.uncertainty.forceConstant_model_N_A;forceGainScale=1;
  if hot
   forceGainScale=20/24.4;
   p.coil.forceConstant=24.4;p.coil.resistance=6.4*1.39;p.coil.inductance=.004;
   p.uncertainty.forceConstant_model_N_A=p.coil.forceConstant*oldIdentifiedK/oldK;
   p.inertial.forceConstantCalibrationScale=oldIdentifiedK/oldK;
   p.inertial.lowerLoadInverse.identifiedForceConstant_N_A=p.uncertainty.forceConstant_model_N_A;
   for field={'forceKp','forceKi','forceKd'},p.inertial.(field{1})=before.inertial.(field{1})*forceGainScale;end
  end
  scope='Required engineered complete-response envelope; no measured manufacturer full transfer or physical sensor qualification.';
  if strcmp(kind,'digital_envelope')
   scope=[scope ' Digital noise/range/latency target only; no ICM clock, FIR, actual sample interface or external ADC path is asserted.'];
  elseif strcmp(kind,'analog1002')
   scope=[scope ' ADXL1002 reference at an8kHz complete response; that assembled response remains an engineering assumption.'];
  else
   scope=[scope ' ADXL354C noise/range anchored at1.9kHz; extra phase and nominal response are assumed, while the retained actual AA and timing residuals remain.'];
  end
  if current_us==50,scope=[scope ' Separate50us current-bandwidth tradeoff, not the original100us drive.'];end
  if hot
   scope=[scope ' New native hot-winding dynamic simulation inputs: Kf24.4N/A,L4mH,identified operatingR8.896ohm (6.4ohm at25C times1.39),32V usable output,50us current target. No time-varying thermal state or supplier hot-force/driver-phase guarantee.'];
  end
  p.budgetAccelerometer=struct('profile',name,'density_ug_sqrtHz',density,'sensorRange_g',sensorRange, ...
   'analogSensitivity_V_g',sensitivity,'matchedNominalCompleteResponseCutoff_Hz',cutoff, ...
   'commonExtraLatency_s',latency_us*1e-6,'actualManufacturerCompleteTransferRepresented',false, ...
   'partQualified',false,'originalStressCaseIndex',caseIndex,'scope',scope);
  p=acf_prepare(p);
  assert(isequaln(p.mechanics,before.mechanics));
  expectedUncertainty=before.uncertainty;
  if hot,expectedUncertainty.forceConstant_model_N_A=p.uncertainty.forceConstant_model_N_A;end
  assert(isequaln(p.uncertainty,expectedUncertainty),'A retained calibration or robot identification residual changed.');
  assert(p.sensor.bias==before.sensor.bias&&p.contact.lateralBias_N==before.contact.lateralBias_N);
  assert(p.accel.bias==before.accel.bias&&p.accel.scaleError==before.accel.scaleError);
  assert(p.sensor.distalMassCalibrationScale==before.sensor.distalMassCalibrationScale);
  assert(p.sensor.distalMass_model_kg==before.sensor.distalMass_model_kg);
  assert(p.inertial.lowerLoadInverse.identifiedStiffness_N_m==before.inertial.lowerLoadInverse.identifiedStiffness_N_m);
  assert(p.inertial.lowerLoadInverse.identifiedDamping_Ns_m==before.inertial.lowerLoadInverse.identifiedDamping_Ns_m);
  expectedCoil=before.coil;
  expectedCoil.currentResponse=current_us*1e-6;expectedCoil.voltage=voltage;
  if hot
   expectedCoil.forceConstant=24.4;expectedCoil.resistance=6.4*1.39;expectedCoil.inductance=.004;
  end
  assert(isequaln(p.coil,expectedCoil),'A current limit, travel assumption or other coil setting changed.');
  afterOffsets=[p.acquisition.forceDelayOffset_s,p.acquisition.lateralDelayOffset_s, ...
   p.acquisition.accelerationDelayOffset_s,p.acquisition.positionDelayOffset_s];
  assert(isequal(actualOffsetsBefore,afterOffsets));
  channelNames={'force','lateral','acceleration','position'};actualPipelines=zeros(1,4);
  for channelIndex=1:4
   actualPipelines(channelIndex)=p.adcStudy.d.channels.(channelNames{channelIndex}).pipelineDelay_s;
   assert(abs(actualPipelines(channelIndex)-(actualCommonBefore+actualOffsetsBefore(channelIndex)+latency_us*1e-6))<1e-14);
  end
  assert(abs(p.adcStudy.d.channels.frame.pipelineDelay_s-(20e-6+latency_us*1e-6))<1e-14);
  if hot,assert(abs(p.uncertainty.forceConstant_model_N_A/p.coil.forceConstant-oldIdentifiedK/oldK)<1e-14);end
  config=struct('p',p,'scenario','budget_sensor_simultaneous_stress','seedRequested',before.sensor.seed, ...
   'group',name,'retainedParameterSource',sourceFiles{caseIndex}, ...
   'retainedParameterSha256',sourceHashes{caseIndex},'retainedParameterIndex',sourceIndices(caseIndex));
  if isempty(configs),configs=config;else,configs(end+1)=config;end %#ok<AGROW>
  row=struct('group',name,'caseIndex',caseIndex,'sourceFile',sourceFiles{caseIndex}, ...
   'sourceSha256',sourceHashes{caseIndex},'sourceIndex',sourceIndices(caseIndex), ...
   'duration_s',p.sim.stopTime,'massS_kg',p.mechanics.mS,'reference_N',p.force.reference, ...
   'angle_deg',p.mechanics.angle*180/pi,'seed',p.sensor.seed, ...
   'density_ug_sqrtHz',density,'referenceAccelerationRMS_m_s2',p.accel.noiseRms, ...
   'commonExtraLatency_us',latency_us,'originalActualCommonPipeline_us',actualCommonBefore*1e6, ...
   'originalActualChannelOffsets_us',actualOffsetsBefore*1e6, ...
   'actualPipelinesForceLateralAccelerationPosition_us',actualPipelines*1e6, ...
   'identifiedFramePipeline_us',p.adcStudy.d.channels.frame.pipelineDelay_s*1e6, ...
   'currentResponse_us',p.coil.currentResponse*1e6,'usableVoltage_V',p.coil.voltage, ...
   'actualKf_N_A',p.coil.forceConstant,'identifiedKf_N_A',p.uncertainty.forceConstant_model_N_A, ...
   'actualResistance_ohm',p.coil.resistance,'actualInductance_H',p.coil.inductance, ...
   'forceFeedbackGainScale',forceGainScale,'scope',scope,'executed',false);
  if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 end
 file=fullfile(outputFolder,[name '_configs.mat']);save(file,'configs','-v7');
 group=struct('name',name,'file',file,'sha256',sha256(file),'cases',7, ...
  'longCases',4,'mixedCases',3,'totalRecordingDuration_s',132,'kind',kind, ...
  'scope',scope,'executed',false);
 if isempty(groups),groups=group;else,groups(end+1)=group;end %#ok<AGROW>
end
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'prepare_budget_sensor_stress_20261002.m'));
report=struct('status','prepared_inputs_not_executed','date','2026-10-02','sourceModelFile',modelFile, ...
 'sourceModelSha256',modelSha,'retainedLongParametersFile',longFile,'retainedLongParametersSha256',longSha, ...
 'retainedMixedParametersFile',mixedFile,'retainedMixedParametersSha256',mixedSha, ...
 'recipeSha256',sha256(fullfile(outputFolder,'prepare_budget_sensor_stress_20261002.m')), ...
 'plannedGroups',7,'plannedCases',49,'executedNativeCases',0,'groups',groups,'rows',rows, ...
 'guardSummary',struct('allStaticResidualsPreservedExceptExplicitRealKfUnitConversion',true, ...
 'actualAndIdentifiedMassesAndMechanicsPreserved',true,'actualChannelTimingOffsetsPreserved',true, ...
 'actualCommonPipelineResidualPreserved',true,'extraCommonLatencyAddedToKnownAndActualPaths',true, ...
 'noPhysicalCaseReset',true,'nativeSourceUnchanged',strcmp(sha256(modelFile),modelSha)), ...
 'claim','Prepared native experiment inputs only. Root executions qualify finite modeled stress histories; unknown assembled part transfer remains an acceptance envelope. Hot-winding cases additionally model real electrical constants and voltage clipping, with fixed identified hot resistance and unchanged mechanical/current/travel assumptions.');
assert(all(structfun(@(x)logical(x),report.guardSummary)));
write_json(fullfile(outputFolder,'manifest.json'),report);
end

function write_json(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end

function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
digest=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
