function report=build_adc_acquisition_trial_20261002(root,outputFolder,runSmoke,instrumentedModelFile)
%BUILD_ADC_ACQUISITION_TRIAL_20261002 Private native analog/ADC study fork.
% Does not modify the canonical diagram, helpers, desktop or delivered ZIP.
% Continuous analog conditioning precedes ADC sampling. SINC3 is represented
% by the exact manufacturer kernel aggregated into a declared fine clock.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,outputFolder=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002');end
if nargin<3,runSmoke=true;end
if nargin<4,instrumentedModelFile=fullfile(root,'acf_simulink','validation','inertial_uncertainty_v8_portable_inputs','models','calibrated_long_instrumented.slx');end
assert(~isfile(fullfile(outputFolder,'acf_force_control_adc_trial.slx')),'Preserve an existing frozen ADC trial.');
maxNumCompThreads(2);
source=fullfile(root,'acf_simulink','validation','final_grid','immutable_execution_source','scientific_source');
scientificModel=fullfile(source,'acf_force_control_normal_trial.slx');
sourceModel=instrumentedModelFile;
candidate=fullfile(source,'inertial_final_candidate_20260930.mat');
assert(strcmp(sha256(scientificModel),'73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643'));
assert(strcmp(sha256(sourceModel),'a6e461f018c9c1d5c784db2ae43781dddfa5b0fe8082caf42c49bb8d270c5009'));
assert(strcmp(sha256(candidate),'d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7'));
helpers=fullfile(outputFolder,'helpers');if ~isfolder(helpers),mkdir(helpers);end
copyfile(fullfile(source,'helpers','*.m'),helpers);
baseText=fileread(fullfile(helpers,'acf_prepare.m'));
baseText=strrep(baseText,'function p = acf_prepare(p)','function p = acf_prepare_baseline_v8(p)');
write_text(fullfile(helpers,'acf_prepare_baseline_v8.m'),baseText);
write_text(fullfile(helpers,'acf_prepare.m'),sprintf('function p=acf_prepare(p)\n%% Private ADC-study dispatcher; frozen physical preparer is retained.\np=acf_adc_prepare(p);\nend\n'));
assert(isfile(fullfile(helpers,'acf_adc_prepare.m')),'The private acquisition preparer must be present.');
copyfile(candidate,fullfile(outputFolder,'baseline_candidate.mat'));
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'build_adc_acquisition_trial_20261002.m'));
addpath(helpers,'-begin');clear acf_prepare acf_prepare_baseline_v8 acf_adc_prepare acf_run;
privateState=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','cache','code','sdi'},mkdir(fullfile(privateState,name{1}));end
Simulink.fileGenControl('set','CacheFolder',fullfile(privateState,'cache'), ...
 'CodeGenFolder',fullfile(privateState,'code'),'createDir',true);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
sdiFolder=fullfile('/dev/shm',['acf-adc-build-' char(java.util.UUID.randomUUID)]);
mkdir(sdiFolder);Simulink.sdi.setStorageLocation(sdiFolder);Simulink.sdi.setRecordData(true);
original='acf_force_control_normal_trial';mdl='acf_force_control_adc_trial';
cleanup=onCleanup(@()finish(mdl,original,previousStorage,previousRecord,sdiFolder)); %#ok<NASGU>
assert(~bdIsLoaded(original)&&~bdIsLoaded(mdl));
privateDonor=fullfile(privateState,[original '.slx']);copyfile(sourceModel,privateDonor);
load_system(privateDonor);oldSnapshot=physical_snapshot(original);
modelFile=fullfile(outputFolder,[mdl '.slx']);save_system(original,modelFile);
if bdIsLoaded(original),close_system(original,0);end
if bdIsLoaded(mdl),close_system(mdl,0);end
load_system(modelFile);
saved=load(candidate);if isfield(saved,'p'),p=saved.p;else,p=saved.pSelected;end
p.uncertainty=struct('forceScaleZ',0,'forceScaleX',0,'positionScale',0, ...
 'forceFrameAngle_rad',0,'accelFrameAngle_rad',0,'forceCrossZFromX',0,'forceCrossXFromZ',0, ...
 'accelBiasX_m_s2',0,'massB_model_kg',p.mechanics.mB,'forceConstant_model_N_A',p.coil.forceConstant, ...
 'recenterGain',min(p.robot.recenterGain,p.robot.forceGate.maxDelayGainProduct/ ...
 (p.robotPrediction.delay+p.robotPrediction.responseTime+p.robot.forceGate.dwellTime+p.robot.forceGate.filterTime)));
p.adcStudy=struct('fineClock_s',1/300000,'outputRate_Hz',100000,'converter','sar', ...
 'modulatorRate_Hz',3840000,'pipelineDelay_s',20e-6,'identifiedPipelineDelay_s',20e-6, ...
 'forceQuantum_N',320/2^16,'accelerationQuantum_m_s2',5/2^16/.04*9.80665, ...
 'forceAdcNoiseRms_N',sqrt(.003225558960381975^2-(320/2^16)^2/12), ...
 'accelerationAdcNoiseRms_m_s2',sqrt(.012356221788605425^2-(5/2^16/.04*9.80665)^2/12));
p.sim.step=p.adcStudy.fineClock_s;p.sim.stopTime=.25;p.sim.evaluateFrom=0;
p=acf_prepare(p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
% Axial/lateral force: bypass the legacy input hold in the active analog path.
force=[mdl '/Force measurement'];lateral=[mdl '/Lateral B-S force measurement'];
wire_continuous_input(force,'Sensor sampling','Raw noisy force sample',1);
wire_continuous_input(lateral,'Sensor sampling','Raw noisy force sample',1);
set_param([force '/Sensor noise RMS'],'Gain','p.adcStudy.d.forceNoiseInputRms_N');
set_param([lateral '/Noise RMS'],'Gain','p.adcStudy.d.lateralNoiseInputRms_N');
set_param([force '/Raw output-noise normalization'],'Gain','1');
set_param([lateral '/Raw output-noise normalization'],'Gain','1');
add_tone(force,'Raw noisy force sample','Force analog vibration probe', ...
 'p.adcStudy.forceAnalogToneAmplitude_N');
add_tone(lateral,'Raw noisy force sample','Lateral analog vibration probe', ...
 'p.adcStudy.lateralAnalogToneAmplitude_N');
forceInitial=initial_input(force,'Engineered common AA and EMA','force');
lateralInitial=initial_input(lateral,'Engineered common AA and EMA','lateral');
make_chain(force,'Engineered common AA and EMA','force',forceInitial,'force', ...
 'p.adcStudy.d.forceGravityOffset_N','force_adc_prequant','force_adc_clip');
make_chain(lateral,'Engineered common AA and EMA','lateral',lateralInitial,'force', ...
 'p.adcStudy.d.lateralGravityOffset_N','lateral_adc_prequant','lateral_adc_clip');
replace_delay(force,'Sensor sample delay','force', ...
 quantized_force_initial(forceInitial,'p.adcStudy.d.forceGravityOffset_N','p.sensor.bias'));
replace_delay(lateral,'Sensor sample delay','lateral', ...
 quantized_force_initial(lateralInitial,'p.adcStudy.d.lateralGravityOffset_N','p.contact.lateralBias_N'));
% Accelerometer front-end: specific acceleration includes gravity; measured
% static tare uses the actual ADC quantum. Drift remains after static tare.
imu=[mdl '/S accelerometer and ADC'];
axes={'Z','X'};gravities={'p.d.g','p.mechanics.gravity*sin(p.mechanics.angle)'};
for k=1:2
 axis=axes{k};wire_continuous_input(imu,[axis ' sensor sampling'],[axis ' noisy sensor'],1);
 set_param([imu '/' axis ' raw noise RMS'],'Gain','p.adcStudy.d.accelerationNoiseInputRms_m_s2');
 add_tone(imu,[axis ' noisy sensor'],[axis ' analog vibration probe'], ...
  ['p.adcStudy.acceleration' axis 'AnalogToneAmplitude_m_s2']);
 initial=initial_input(imu,[axis ' engineered common AA and EMA'],'acceleration');
 make_chain(imu,[axis ' engineered common AA and EMA'],'acceleration',initial,'acceleration','0', ...
  ['acceleration_' lower(axis) '_adc_prequant'],['acceleration_' lower(axis) '_adc_clip']);
 % The replacement chain owns range and quantization before its EMA. These
 % historical blocks remain identity paths to preserve the saved topology.
 set_param([imu '/' axis ' ADC quantization'],'QuantizationInterval','1e-15');
 set_param([imu '/' axis ' acceleration range'],'UpperLimit','inf','LowerLimit','-inf');
 set_param([imu '/' axis ' known static tare'],'Value', ...
  ['round((' initial ')/p.accel.quantum)*p.accel.quantum']);
 if strcmp(axis,'X'),delayInitial='p.uncertainty.accelBiasX_m_s2';else,delayInitial='p.accel.bias';end
 replace_delay(imu,[axis ' known channel delay'],'acceleration',delayInitial);
end
% Position is part of the same timestamped acquisition frame. Its ADC noise
% is not charged to the force ADC; the retained position sensor PSD is used.
position=[mdl '/Relative position measurement'];
wire_continuous_input(position,'Position sampling','Sensed position',1);
set_param([position '/Position noise RMS'],'Gain','p.adcStudy.d.positionNoiseInputRms_m');
if getSimulinkBlockHandle([position '/Raw position-noise normalization'])~=-1
 set_param([position '/Raw position-noise normalization'],'Gain','1');
end
replace_delay(position,'Measurement delay','position','p.d.positionMeasurementInitial',true);
motion=[mdl '/Contact force measurement/Head motion estimator'];
make_chain(motion,'Position common AA and EMA','position','0','none','0','','');
make_chain(motion,'Robot velocity common AA and EMA','frame','0','none','0','','');
make_chain(motion,'Robot acceleration common AA and EMA','frame','0','none','0','','');
replace_delay(motion,'Robot velocity frame matching delay','frame','0');
replace_delay(motion,'Robot acceleration frame matching delay','frame','0');
% The position's physical transport was removed before the analog filter;
% place the exact transport after its active acquisition-frame selector.
insert_position_pipeline(motion);
add_adc_log(mdl);
assert(isequal(oldSnapshot,physical_snapshot(mdl)),'The acquisition fork changed the physical plant or controller.');
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
save_system(mdl,modelFile);
assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)),'Acquisition fork introduces an algebraic loop.');
software=struct('physicalPlantAndControllerUnchanged',true, ...
 'analogFiltersBeforeOutputSampling',true,'rawNoiseClockIndependentOfADCOutput',true, ...
 'converterNoiseSeparateFromSensorPSD',true,'gravityAndADCQuantumAwareStaticTare',true, ...
 'exactChosenOutputRates',true,'issuedMotionUsesIdentifiedAcquisitionOnly',true, ...
 'canonicalModelUntouched',true,'smokeExecuted',false);
if runSmoke
 t=tic;r=acf_run(mdl,p,false);software.smokeExecuted=true;
 software.smokeElapsed_s=toc(t);software.smokeNativeSamples=numel(r.time);
 software.smokeFinite=all(isfinite(r.data(:)))&&all(isfinite(r.normalContact.data(:)));
 software.smokeNormalForcePeakError_N=max(abs(r.normalContact.data(:,1)-p.force.reference));
 software.smokeForceBalanceMaximumResidual_N=max(abs(r.accelerometer.data(:,5)- ...
  (r.normalContact.data(:,4).*sqrt(1+r.normalContact.data(:,5).^2)+ ...
   r.normalContact.data(:,5).*r.accelerometer.data(:,6))));
 assert(software.smokeFinite&&software.smokeNativeSamples==round(p.sim.stopTime/p.sim.step)+1);
 save(fullfile(outputFolder,'smoke_native_trace.mat'),'r','-v7.3');clear r;
 Simulink.sdi.clear;
end
save(fullfile(outputFolder,'adc_trial_candidate.mat'),'p');
save_system(mdl,modelFile);modelHash=sha256(modelFile);
contract=struct('status','frozen_private_native_acquisition_source', ...
 'date','2026-10-02','modelFile',modelFile,'modelSha256',modelHash, ...
 'scientificSourceModelFile',scientificModel,'scientificSourceModelSha256',sha256(scientificModel), ...
 'instrumentedDonorModelFile',sourceModel,'instrumentedDonorModelSha256',sha256(sourceModel), ...
 'scientificCandidateFile',candidate,'scientificCandidateSha256',sha256(candidate), ...
 'recipeFile',fullfile(outputFolder,'build_adc_acquisition_trial_20261002.m'), ...
 'recipeSha256',sha256(fullfile(outputFolder,'build_adc_acquisition_trial_20261002.m')), ...
 'candidateFile',fullfile(outputFolder,'adc_trial_candidate.mat'), ...
 'candidateSha256',sha256(fullfile(outputFolder,'adc_trial_candidate.mat')), ...
 'adcStudy',p.adcStudy,'softwareChecks',software, ...
 'scope','Source and smoke software acceptance only; no sharp-bump ADC qualification is claimed. Each experiment must bind prepared parameters, converter clock/response and full-native force scoring.');
write_json(fullfile(outputFolder,'acquisition_contract.json'),contract);
files=dir(fullfile(helpers,'*.m'));manifest=struct([]);
for k=1:numel(files)
 one=struct('file',fullfile('helpers',files(k).name),'sha256',sha256(fullfile(helpers,files(k).name)),'bytes',files(k).bytes);
 if isempty(manifest),manifest=one;else,manifest(end+1)=one;end %#ok<AGROW>
end

function initial=initial_input(sys,name,channel)
old=get_param([sys '/' name],'InitialStates');
suffix=['*p.d.acquisition.responses.' channel '.steadyStatePerUnitInput'];
assert(endsWith(old,suffix),'Unrecognized loaded-equilibrium acquisition initialization.');
initial=extractBefore(old,strlength(old)-strlength(suffix)+1);initial=char(initial);
end
write_json(fullfile(outputFolder,'helper_manifest.json'),struct('status','frozen','files',manifest));
report=contract;
end

function make_chain(sys,blockName,channel,initial,kind,offset,prequantTag,clipTag)
old=[sys '/' blockName];ph=get_param(old,'PortHandles');
inputLine=get_param(ph.Inport(1),'Line');source=get_param(inputLine,'SrcPortHandle');
outputLine=get_param(ph.Outport(1),'Line');destinations=get_param(outputLine,'DstPortHandle');
position=get_param(old,'Position');
delete_line(sys,source,ph.Inport(1));
for target=destinations(:)',delete_line(sys,ph.Outport(1),target);end
delete_block(old);
sub=add_block('built-in/Subsystem',old,'Position',position); %#ok<NASGU>
add_block('built-in/Inport',[old '/analog input'],'Position',[30 93 60 107]);
add_block('built-in/Outport',[old '/conditioned output'],'Position',[2110 93 2140 107]);
coefficient=['p.adcStudy.d.channels.' channel];
add_block('simulink/Continuous/State-Space',[old '/Continuous analog front end'], ...
 'Position',[115 60 325 140],'A',[coefficient '.analogA'],'B',[coefficient '.analogB'], ...
 'C',[coefficient '.analogC'],'D',[coefficient '.analogD'], ...
 'X0',['(' initial ')*' coefficient '.analogSteadyStatePerUnitInput']);
add_block('simulink/Continuous/State-Space',[old '/ADC analog input filter'], ...
 'Position',[380 60 575 140],'A','p.adcStudy.d.converterAnalog.A','B','p.adcStudy.d.converterAnalog.B', ...
 'C','p.adcStudy.d.converterAnalog.C','D','p.adcStudy.d.converterAnalog.D', ...
 'X0',['(' initial ')*p.adcStudy.d.converterAnalog.steadyStatePerUnitInput']);
add_block('simulink/Discrete/Discrete Transfer Fcn',[old '/Native converter impulse response'], ...
 'Position',[625 60 860 140],'Numerator','p.adcStudy.d.converter.numerator', ...
 'Denominator','p.adcStudy.d.converter.denominator', ...
 'InitialStates',['(' initial ')*p.adcStudy.d.converter.steadyStatePerUnitInput'], ...
 'SampleTime','p.adcStudy.fineClock_s');
add_block('simulink/Discrete/Zero-Order Hold',[old '/Actual ADC output samples'], ...
 'Position',[910 65 1080 135],'SampleTime','p.adcStudy.outputSampleTime_s');
last='Actual ADC output samples/1';emaInitial=initial;
if ~strcmp(kind,'none')
 add_block('simulink/Sources/Random Number',[old '/Independent converter output noise'], ...
  'Position',[885 230 1080 290],'Mean','0','Variance','1', ...
  'Seed',['p.sensor.seed+' num2str(sum(double(prequantTag))+701)],'SampleTime','p.adcStudy.outputSampleTime_s');
 if strcmp(kind,'force')
  noise='p.adcStudy.forceAdcNoiseRms_N';quant='max(p.adcStudy.forceQuantum_N,1e-15)';
  range='p.adcStudy.forceAdcRange_N';
 else
  noise='p.adcStudy.accelerationAdcNoiseRms_m_s2';quant='max(p.adcStudy.accelerationQuantum_m_s2,1e-15)';
  range='min(p.adcStudy.accelerationAdcRange_g,p.accel.range_g)*p.mechanics.gravity';
 end
 add_block('simulink/Math Operations/Gain',[old '/Converter output noise RMS'], ...
  'Position',[1130 230 1315 290],'Gain',noise);
 add_block('simulink/Discrete/Discrete Transfer Fcn',[old '/ADC output noise correlation'], ...
  'Position',[1110 455 1390 515],'Numerator','p.adcStudy.d.converterNoise.numerator', ...
  'Denominator','p.adcStudy.d.converterNoise.denominator','InitialStates','0', ...
  'SampleTime','p.adcStudy.outputSampleTime_s');
 add_block('simulink/Math Operations/Sum',[old '/ADC input and output noise'],'Position',[1170 70 1210 140],'Inputs','++');
 add_line(old,last,'ADC input and output noise/1');
 add_line(old,'Independent converter output noise/1','Converter output noise RMS/1');
 add_line(old,'Converter output noise RMS/1','ADC output noise correlation/1');
 add_line(old,'ADC output noise correlation/1','ADC input and output noise/2');
 add_block('simulink/Math Operations/Bias',[old '/Restore physical static offset'],'Position',[1260 65 1445 135],'Bias',offset);
 add_line(old,'ADC input and output noise/1','Restore physical static offset/1');
 add_block('simulink/Discontinuities/Saturation',[old '/Actual ADC input range'],'Position',[1495 65 1645 135], ...
  'UpperLimit',range,'LowerLimit',['-(' range ')']);
 add_block('simulink/Discontinuities/Quantizer',[old '/Actual converter quantization'],'Position',[1695 65 1870 135], ...
  'QuantizationInterval',quant);
 add_line(old,'Restore physical static offset/1','Actual ADC input range/1');
 add_line(old,'Actual ADC input range/1','Actual converter quantization/1');
 add_block('simulink/Math Operations/Bias',[old '/Known quantized static offset'],'Position',[1915 65 2100 135], ...
  'Bias',['-round((' offset ')/(' quant '))*(' quant ')']);
 add_line(old,'Actual converter quantization/1','Known quantized static offset/1');
 add_block('simulink/Signal Routing/Goto',[old '/ADC input diagnostic'],'Position',[1350 165 1490 195], ...
  'GotoTag',prequantTag,'TagVisibility','global');
 add_line(old,'Restore physical static offset/1','ADC input diagnostic/1');
 add_block('simulink/Math Operations/Sum',[old '/ADC range difference'],'Position',[1620 350 1660 420],'Inputs','+-');
 add_block('simulink/Math Operations/Abs',[old '/ADC clipping magnitude'],'Position',[1740 350 1900 420]);
 add_block('simulink/Signal Routing/Goto',[old '/ADC clipping diagnostic'],'Position',[1990 370 2130 400], ...
  'GotoTag',clipTag,'TagVisibility','global');
 add_line(old,'Restore physical static offset/1','ADC range difference/1');
 add_line(old,'Actual ADC input range/1','ADC range difference/2');
 add_line(old,'ADC range difference/1','ADC clipping magnitude/1');
 add_line(old,'ADC clipping magnitude/1','ADC clipping diagnostic/1');
 last='Known quantized static offset/1';
 emaInitial=['round(((' initial ')+(' offset '))/(' quant '))*(' quant ')-round((' offset ')/(' quant '))*(' quant ')'];
end
set_param([old '/conditioned output'],'Position',[2540 93 2570 107]);
add_block('simulink/Discrete/Discrete Transfer Fcn',[old '/Fresh per-conversion EMA'], ...
 'Position',[2240 60 2470 140],'Numerator',[coefficient '.emaNumerator'], ...
 'Denominator',[coefficient '.emaDenominator'],'InitialStates',['(' emaInitial ')*' coefficient '.emaSteadyStatePerUnitInput'], ...
 'SampleTime','p.adcStudy.outputSampleTime_s');
add_line(old,'analog input/1','Continuous analog front end/1');
add_line(old,'Continuous analog front end/1','ADC analog input filter/1');
add_line(old,'ADC analog input filter/1','Native converter impulse response/1');
add_line(old,'Native converter impulse response/1','Actual ADC output samples/1');
add_line(old,last,'Fresh per-conversion EMA/1');add_line(old,'Fresh per-conversion EMA/1','conditioned output/1');
new=get_param(old,'PortHandles');add_line(sys,source,new.Inport(1),'autorouting','on');
for target=destinations(:)',add_line(sys,new.Outport(1),target,'autorouting','on');end
end

function wire_continuous_input(sys,sampling,sumName,port)
sample=get_param([sys '/' sampling],'PortHandles');input=get_param(sample.Inport(1),'Line');
source=get_param(input,'SrcPortHandle');destination=get_param([sys '/' sumName],'PortHandles');
delete_line(sys,sample.Outport(1),destination.Inport(port));
add_line(sys,source,destination.Inport(port),'autorouting','on');
end

function replace_delay(sys,name,channel,initial,identity)
if nargin<5,identity=false;end
path=[sys '/' name];ph=get_param(path,'PortHandles');
source=get_param(get_param(ph.Inport(1),'Line'),'SrcPortHandle');
targets=get_param(get_param(ph.Outport(1),'Line'),'DstPortHandle');position=get_param(path,'Position');
delete_line(sys,source,ph.Inport(1));
for target=targets(:)',delete_line(sys,ph.Outport(1),target);end
delete_block(path);
if identity
 add_block('simulink/Math Operations/Gain',path,'Position',position,'Gain','1');
else
 add_block('simulink/Continuous/Transport Delay',path,'Position',position, ...
  'DelayTime',['p.adcStudy.d.channels.' channel '.pipelineDelay_s'],'InitialOutput',initial);
end
new=get_param(path,'PortHandles');add_line(sys,source,new.Inport(1),'autorouting','on');
for target=targets(:)',add_line(sys,new.Outport(1),target,'autorouting','on');end
end

function insert_position_pipeline(sys)
delete_line(sys,'Position acquisition frame/1','Fresh 10 kHz position frame/1');
add_block('simulink/Continuous/Transport Delay',[sys '/Position ADC delivery pipeline'], ...
 'Position',[1060 540 1280 600],'DelayTime','p.adcStudy.d.channels.position.pipelineDelay_s','InitialOutput','0');
add_line(sys,'Position acquisition frame/1','Position ADC delivery pipeline/1','autorouting','on');
add_line(sys,'Position ADC delivery pipeline/1','Fresh 10 kHz position frame/1','autorouting','on');
end

function add_tone(sys,target,name,amplitude)
path=[sys '/' name];add_block('simulink/Sources/Sine Wave',path, ...
 'Position',[30 1700 245 1760],'Amplitude',amplitude,'Frequency','2*pi*p.adcStudy.analogToneFrequency_Hz', ...
 'Phase','p.adcStudy.analogTonePhase_rad','SampleTime','0');
set_param([sys '/' target],'Inputs','+++');add_line(sys,[name '/1'],[target '/3'],'autorouting','on');
end

function add_adc_log(mdl)
sys=[mdl '/ADC acquisition diagnostics'];add_block('built-in/Subsystem',sys,'Position',[1110 1200 1450 1320]);
tags={'force_adc_prequant','lateral_adc_prequant','acceleration_z_adc_prequant','acceleration_x_adc_prequant', ...
 'force_adc_clip','lateral_adc_clip','acceleration_z_adc_clip','acceleration_x_adc_clip'};
add_block('simulink/Signal Routing/Mux',[sys '/ADC native conversion channels'],'Position',[530 60 535 530],'Inputs','8');
for k=1:numel(tags)
 add_block('simulink/Signal Routing/From',[sys '/Channel ' num2str(k)], ...
  'Position',[55 40+k*55 345 70+k*55],'GotoTag',tags{k});
 add_line(sys,['Channel ' num2str(k) '/1'],['ADC native conversion channels/' num2str(k)]);
end
add_block('simulink/Sinks/To Workspace',[sys '/All ADC conversion samples'],'Position',[650 240 925 340], ...
 'VariableName','adc_acquisition_log','SaveFormat','Timeseries','MaxDataPoints','inf', ...
 'SampleTime','p.adcStudy.outputSampleTime_s');
add_line(sys,'ADC native conversion channels/1','All ADC conversion samples/1');
end

function value=quantized_force_initial(initial,offset,bias)
q='max(p.adcStudy.forceQuantum_N,1e-15)';
value=['round(((' initial ')+(' offset '))/(' q '))*(' q ')-round((' offset ')/(' q '))*(' q ')+(' bias ')'];
end

function snapshot=physical_snapshot(mdl)
paths={'Actuator and RABS mechanics','Force controller','Surface path','Robot command delay', ...
 'Robot motion','Recenter travel','Command-based robot prediction','Robot force stability gate'};
snapshot=struct([]);
for k=1:numel(paths)
 root=[mdl '/' paths{k}];if getSimulinkBlockHandle(root)==-1,continue;end
 blocks=find_system(root,'LookUnderMasks','all','MatchFilter',@Simulink.match.allVariants,'Type','Block');
 for b=1:numel(blocks)
  one=struct('path',erase(blocks{b},[mdl '/']),'type',get_param(blocks{b},'BlockType'),'parameters',struct);
  dialog=get_param(blocks{b},'DialogParameters');
  if isstruct(dialog)
   for name=fieldnames(dialog)',one.parameters.(name{1})=get_param(blocks{b},name{1});end
  end
  if isempty(snapshot),snapshot=one;else,snapshot(end+1)=one;end %#ok<AGROW>
 end
end
assert(~isempty(snapshot),'No native physical/controller blocks found.');
end

function finish(mdl,original,storage,record,sdi)
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
for name={mdl,original},try,if bdIsLoaded(name{1}),close_system(name{1},0);end,catch,end,end
if isfolder(sdi),try,rmdir(sdi,'s');catch,end,end
end

function write_text(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',value);
end

function write_json(path,value)
write_text(path,[jsonencode(value,'PrettyPrint',true) newline]);
end

function hash=sha256(path)
assert(usejava('jvm'),'Build uses MATLAB JVM SHA256 and UUID only, not the desktop.');
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
digest=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
