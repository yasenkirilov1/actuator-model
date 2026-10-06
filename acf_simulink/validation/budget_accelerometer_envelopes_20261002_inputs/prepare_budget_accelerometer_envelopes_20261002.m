function report=prepare_budget_accelerometer_envelopes_20261002(root,outputFolder)
%PREPARE_BUDGET_ACCELEROMETER_ENVELOPES_20261002 Native experiment inputs.
% These are required engineered noise/range/response envelopes, not exact
% unpublished manufacturer transfer functions or hardware qualifications.
% The root qualification runner executes and scores the native histories.
assert(~usejava('desktop'),'Use a private MATLAB -batch process.');maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,outputFolder=fullfile(root,'acf_simulink','validation','budget_accelerometer_envelopes_20261002_inputs');end
assert(~isfile(fullfile(outputFolder,'configs.mat')),'Preserve previously prepared budget inputs.');
if ~isfolder(outputFolder),mkdir(outputFolder);end
source=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002');
modelFile=fullfile(source,'acf_force_control_adc_trial.slx');
sourceSha=sha256(modelFile);
assert(strcmp(sourceSha,'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
candidateFile=fullfile(source,'adc_trial_candidate.mat');candidateSha=sha256(candidateFile);
assert(strcmp(candidateSha,'1afe7ee7650360ecb4fb2b1e11a69381332bc5d29cbfda0eb2632449d486ff25'));
addpath(fullfile(source,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
saved=load(candidateFile,'p');base=saved.p;
assert(isfield(base,'uncertainty')&&base.acquisition.enabled&&base.accel.enabled);
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.sim.step=1/300000;
referenceENBW_Hz=.06354253123515036/(2*10e-6);standardGravity=9.80665;
referenceDensity_ug_sqrtHz=.014/(standardGravity*sqrt(referenceENBW_Hz))*1e6;
configs=struct([]);planRows=struct([]);optionalConfigs=struct([]);optionalRows=struct([]);
% Primary ADXL354C range/noise anchor. Complete matched analog response and
% common extra latency still require assembled measurements.
for extra_us=[0 50 100]
 for point=1:2
  for seed=[7 29]
   add('ADXL354C_primary',33.75,8,1900,extra_us,point,seed,.1, ...
    'ADXL354C noise/range anchored; assumed engineered matched complete response, not measured or manufacturer full transfer.');
  end
 end
end
% Separate faster-current driver tradeoff on the scientific reference coil.
% Neither the higher voltage nor50us current response is hidden in a sensor
% result, and this is not a hot real-winding hardware guarantee.
for extra_us=[0 50]
 for point=1:2
  for seed=[7 29]
   add('ADXL354C_primary_faster_current_driver_tradeoff',33.75,8,1900,extra_us,point,seed,.1, ...
    'ADXL354C anchored primary envelope with a separate scientific-coil driver tradeoff:50us current response and36V. Not the original100us/24V drive or a hot real-coil hardware qualification.');
  end
 end
end
% Wider complete responses are counterfactual sensitivity points. A pass at
% these cutoffs cannot be attributed to the ADXL354C part.
for cutoff=[3000 5100]
 for point=1:2
  for seed=[7 29]
   add('ADXL354C_counterfactual_bandwidth',33.75,8,cutoff,0,point,seed,.1, ...
    'Counterfactual faster complete matched response with ADXL354C noise/range only. Not a realizable-part or purchase claim.');
  end
 end
end
% Digital IMU noise/range envelope: keep the shared matched response as an
% acceptance requirement. It is not an ICM ADC, ODR, clock or digital FIR.
for density=[65 70]
 for extra_us=[80 120]
  for seed=[7 29]
   add('ICM_class_digital_required_envelope',density,16,5100,extra_us,1,seed,NaN, ...
    'Digital IMU noise/range/latency feasibility envelope only. Shared matched acquisition is an engineered target; no actual ICM clock, internal FIR or external-ADC connection is represented.');
  end
 end
end
% Separate optional optimistic latency comparator. The real 4k-output
% digital converter/FIR is deliberately not asserted by this common30k model.
mainConfigs=configs;mainRows=planRows;configs=struct([]);planRows=struct([]);
for point=1:2
 for seed=[7 29]
  add('ADXL355_optimistic_latency_comparator',33.75,8,1900,630,point,seed,NaN, ...
   'Optimistic digital latency-only comparator: common30k frame,1.9k matched envelope and630us extra delay. Does not implement the real ADXL355 4k output/FIR or qualify the part.');
 end
end
optionalConfigs=configs;optionalRows=planRows;configs=mainConfigs;planRows=mainRows;
assert(numel(configs)==36&&numel(optionalConfigs)==4);
save(fullfile(outputFolder,'configs.mat'),'configs','-v7');
configs=optionalConfigs;save(fullfile(outputFolder,'optional_adxl355_configs.mat'),'configs','-v7');configs=mainConfigs;
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'prepare_budget_accelerometer_envelopes_20261002.m'));
report=struct('status','prepared_inputs_not_executed','date','2026-10-02', ...
 'sourceModelFile',modelFile,'sourceModelSha256',sourceSha, ...
 'sourceCandidateFile',candidateFile,'sourceCandidateSha256',candidateSha, ...
 'recipeSha256',sha256(fullfile(outputFolder,'prepare_budget_accelerometer_envelopes_20261002.m')), ...
 'configsFile',fullfile(outputFolder,'configs.mat'),'configsSha256',sha256(fullfile(outputFolder,'configs.mat')), ...
 'plannedNativeCases',36,'plannedOptionalCases',4,'executedNativeCases',0, ...
 'referenceNoiseENBW_Hz',referenceENBW_Hz,'referenceAccelerationDensity_ug_sqrtHz',referenceDensity_ug_sqrtHz, ...
 'referenceAccelerationRMS_m_s2',.014,'standardGravity_m_s2',standardGravity, ...
 'rows',planRows,'optionalRows',optionalRows, ...
 'optionalConfigsFile',fullfile(outputFolder,'optional_adxl355_configs.mat'), ...
 'optionalConfigsSha256',sha256(fullfile(outputFolder,'optional_adxl355_configs.mat')), ...
 'claim','Required engineered performance envelopes on the unchanged native model. Manufacturer noise/range anchors are not complete measured transfer functions. Only separately returned native recordings establish finite model feasibility; no physical part is qualified.', ...
 'sources',{{'https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf', ...
 'https://www.ti.com/lit/ds/symlink/ads131m08.pdf'}}, ...
 'guards',struct('sourceModelUnchanged',strcmp(sha256(modelFile),sourceSha), ...
 'freshActualAndIdentifiedResponses',true,'nativeCalibrationDefaultsIncluded',true, ...
 'noiseDensityFixedAcrossResponseSweep',true,'matchedCommonLatencyActualAndIdentified',true, ...
 'allStartupIncluded',all(arrayfun(@(x)x.p.sim.evaluateFrom==0,configs)), ...
 'noControllerRetuning',all(arrayfun(@(x)x.p.inertial.forceKp==base.inertial.forceKp&&x.p.inertial.forceKi==base.inertial.forceKi,configs))));
assert(all(structfun(@(x)logical(x),report.guards)));
write_json(fullfile(outputFolder,'manifest.json'),report);

 function add(profile,density,range_g,cutoff,extra_us,point,seed,sensitivity,scope)
  p=base;if point==1,p.mechanics.mS=2;p.force.reference=50;p.mechanics.angle=pi/2;
  else,p.mechanics.mS=.1;p.force.reference=5;p.mechanics.angle=0;end
  p.contact.mu=.3;p.sensor.seed=seed;p.coil.strategy='pi';
  p.sensor.distalMassCalibrationScale=1;p.sensor.distalMass_kg=p.mechanics.mS;p.sensor.distalMass_model_kg=p.mechanics.mS;
  p.sensor.noiseRms=.02;p.contact.lateralNoiseRms_N=.02;p.sensor.strokeNoiseRms=1e-6;
  if contains(profile,'faster_current_driver_tradeoff')
   p.coil.currentResponse=50e-6;p.coil.voltage=36;
  end
  p.accel.range_g=range_g;p.accel.noiseRms=density*1e-6*standardGravity*sqrt(referenceENBW_Hz);
  p.accel.nominalNoiseRms=p.accel.noiseRms;
  forceQ=320/2^24;forceTotal=21.31e-6/(1.2/160);
  if isfinite(sensitivity)
   accQ=2.4/2^24/sensitivity*standardGravity;
   accTotal=21.31e-6/sensitivity*standardGravity;adcRange=1.2/sensitivity;
  else
   if contains(profile,'ADXL355'),accQ=16*standardGravity/2^20;else,accQ=32*standardGravity/2^16;end
   accTotal=0;adcRange=range_g;
  end
  p.adcStudy=struct('fineClock_s',1/300000,'outputRate_Hz',30000,'converter','sinc3', ...
   'modulatorRate_Hz',3840000,'analogCutoff_Hz',cutoff,'identifiedAnalogCutoff_Hz',cutoff, ...
   'analogDamping',1/sqrt(2),'converterAnalogOrder',0,'converterAnalogCutoff_Hz',16000, ...
   'emaTime_s',50e-6,'identifiedEmaTime_s',50e-6, ...
   'pipelineDelay_s',20e-6+extra_us*1e-6,'identifiedPipelineDelay_s',20e-6+extra_us*1e-6, ...
   'forceQuantum_N',forceQ,'accelerationQuantum_m_s2',accQ, ...
   'forceAdcNoiseRms_N',sqrt(max(0,forceTotal^2-forceQ^2/12)), ...
   'accelerationAdcNoiseRms_m_s2',sqrt(max(0,accTotal^2-accQ^2/12)), ...
   'forceAdcRange_N',160,'accelerationAdcRange_g',adcRange, ...
   'adcNoiseCorrelationTime_s',0,'accelerationExtraDelay_s',0);
  p.budgetAccelerometer=struct('profile',profile,'density_ug_sqrtHz',density, ...
   'sensorRange_g',range_g,'analogSensitivity_V_g',sensitivity, ...
   'matchedCompleteResponseCutoff_Hz',cutoff,'matchedCommonExtraLatency_s',extra_us*1e-6, ...
   'actualManufacturerCompleteTransferRepresented',false,'partQualified',false,'scope',scope);
  p=acf_prepare(p);
  assert(p.acquisition.enabled&&p.accel.enabled&&p.adcStudy.d.converter.oversamplingRatio==128);
  assert(abs(p.adcStudy.d.accelerationNoiseDensity_m_s2_sqrtHz-density*1e-6*standardGravity)<1e-14);
  assert(p.adcStudy.d.channels.force.pipelineDelay_s==p.adcStudy.d.channels.frame.pipelineDelay_s);
  group=sprintf('%s_d%g_fc%g_L%gus',profile,density,cutoff,extra_us);
  config=struct('p',p,'scenario',profile,'seedRequested',seed,'group',group, ...
   'retainedParameterSource',candidateFile,'retainedParameterSha256',candidateSha,'retainedParameterIndex',1);
  if isempty(configs),configs=config;else,configs(end+1)=config;end
  row=struct('id',numel(configs),'group',group,'profile',profile,'massS_kg',p.mechanics.mS, ...
   'reference_N',p.force.reference,'angle_deg',p.mechanics.angle*180/pi,'mu',p.contact.mu,'seed',seed, ...
   'density_ug_sqrtHz',density,'referenceConditionedAccelerationRMS_m_s2',p.accel.noiseRms, ...
   'sensorRange_g',range_g,'accelerationQuantum_m_s2',accQ, ...
   'totalConverterAccelerationNoiseRMS_m_s2',accTotal, ...
   'additiveConverterAccelerationNoiseRMS_m_s2',p.adcStudy.accelerationAdcNoiseRms_m_s2, ...
   'matchedCompleteResponseCutoff_Hz',cutoff,'matchedCommonExtraLatency_us',extra_us, ...
   'totalPhysicalPipeline_us',p.adcStudy.pipelineDelay_s*1e6, ...
   'currentResponse_us',p.coil.currentResponse*1e6,'driveVoltage_V',p.coil.voltage, ...
   'scope',scope,'executed',false);
  if isempty(planRows),planRows=row;else,planRows(end+1)=row;end
 end
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
