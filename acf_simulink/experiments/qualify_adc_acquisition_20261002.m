function report=qualify_adc_acquisition_20261002(sourceFolder,outputFolder,phase,options)
%QUALIFY_ADC_ACQUISITION_20261002 Executed native bandwidth/rate comparisons.
% Private, frozen plant/controller; all force and guard metrics use EVERY
% native physical sample, including startup. A compact logger changes only
% observation sinks, never the physical or feedback graph.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
if nargin<4,options=struct;end
options=with_defaults(options,struct('cutoffs_Hz',[500 1000 2000 3000 5100 8000], ...
 'sarRates_Hz',[100000 50000 20000 10000],'sincRates_Hz',[30000 15000 7500], ...
 'seeds',[0 7],'partitionIndex',1,'partitionCount',1,'maximumCases',inf, ...
 'selectedConverter','sinc3','selectedRate_Hz',30000,'selectedCutoff_Hz',5100, ...
 'fineClock_s',NaN,'solverStep_s',NaN,'keepAllTraces',false,'loggingParity',false, ...
 'adcNoiseMultiplier',1,'adcNoiseCorrelationTime_s',0,'pipelineExtra_s',0, ...
 'accelerationExtraDelay_s',0,'forceValues_N',5:5:50,'angleValues_deg',0:15:90, ...
 'muValues',[0 .3],'massValues_kg',[.1 2],'gridSeeds',[0 7 29 1729 104729], ...
 'toneFrequencies_Hz',[19000 49000 99000],'toneAmplitude_m_s2',.2, ...
 'customCasesFile','','useFastRestart',false,'probeStopTime_s',.5));
assert(isfolder(sourceFolder)&&~isfile(fullfile(outputFolder,'results.json')));
if ~isfolder(outputFolder),mkdir(outputFolder);end
project=fileparts(fileparts(mfilename('fullpath')));
contract=jsondecode(fileread(fullfile(sourceFolder,'acquisition_contract.json')));
assert(strcmp(contract.status,'frozen_private_native_acquisition_source'));
modelSource=fullfile(sourceFolder,'acf_force_control_adc_trial.slx');
assert(strcmp(sha256(modelSource),contract.modelSha256));
worker=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','helpers','cache','code','sdi'},mkdir(fullfile(worker,name{1}));end
modelFile=fullfile(worker,'acf_force_control_adc_trial.slx');copyfile(modelSource,modelFile);
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(worker,'helpers'));
copyfile(fullfile(sourceFolder,'baseline_candidate.mat'),fullfile(worker,'baseline_candidate.mat'));
copyfile([mfilename('fullpath') '.m'],fullfile(worker,'qualify_adc_acquisition_20261002.m'));
addpath(fullfile(worker,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_run;
maxNumCompThreads(2);
saved=load(fullfile(worker,'baseline_candidate.mat'),'p');base=saved.p;
base.sim.stopTime=4;base.sim.evaluateFrom=0;
base.robot.forceGate.filterTime=.1;base.robot.forceGate.dwellTime=.1;
base.uncertainty=struct('forceScaleZ',0,'forceScaleX',0,'positionScale',0, ...
 'forceFrameAngle_rad',0,'accelFrameAngle_rad',0,'forceCrossZFromX',0,'forceCrossXFromZ',0, ...
 'accelBiasX_m_s2',0,'massB_model_kg',base.mechanics.mB,'forceConstant_model_N_A',base.coil.forceConstant, ...
 'recenterGain',min(base.robot.recenterGain,base.robot.forceGate.maxDelayGainProduct/ ...
 (base.robotPrediction.delay+base.robotPrediction.responseTime+base.robot.forceGate.dwellTime+base.robot.forceGate.filterTime)));
assert(base.force.sampleTime==100e-6&&base.mechanics.upperRigid&&base.mechanics.mB==.6);
if isempty(options.customCasesFile)
 configs=make_configs(base,phase,options,project);
else
 custom=load(options.customCasesFile,'configs');configs=custom.configs;
 copyfile(options.customCasesFile,fullfile(worker,'custom_input_configs.mat'));
end
planned=numel(configs);
ids=find(mod(0:planned-1,options.partitionCount)==options.partitionIndex-1);
ids=ids(1:min(numel(ids),options.maximumCases));configs=configs(ids);
save(fullfile(outputFolder,'input_configs.mat'),'configs','ids','options','-v7');
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
sdi=tempname('/dev/shm');mkdir(sdi);
fid=fopen(fullfile(sdi,'owned_acf_adc_sdi'),'w');assert(fid~=-1);fprintf(fid,'Owned transient acquisition experiment SDI.\n');fclose(fid);
mdl='acf_force_control_adc_trial';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
set_param(mdl,'SimulationMode','accelerator');
add_compact_logger(mdl);
parity=struct('executed',false);
if options.loggingParity
 q=acf_prepare(configs(1).p);q.sim.stopTime=.25;
 prepare_model(mdl,q);set_historical_logging(mdl,true);
 out=sim(mdl);a=out.get('adc_qualification_log');
 full=out.get('normal_contact_log');plant=out.get('plant_log');surface=out.get('surface_log');
 plantData=tsrows(plant,25);normalData=tsrows(full,8);surfaceData=tsrows(surface,3);
 expected=[plantData(:, [1 2 14 15 18 21 22 23 20 6 7 13 12 24]), ...
  normalData(:,[1 4 2 3]),surfaceData(:,3)];
 parity=struct('executed',true,'samples',numel(a.Time),'exactCompactValues',isequal(tsrows(a,19),expected), ...
  'exactSampleTimes',isequal(a.Time,full.Time,plant.Time,surface.Time));
 assert(parity.exactCompactValues&&parity.exactSampleTimes);
 clear out a full plant surface expected plantData normalData surfaceData;Simulink.sdi.clear;
end
set_historical_logging(mdl,false);save_system(mdl,modelFile);
helperFiles=dir(fullfile(worker,'helpers','*.m'));helpers=struct([]);
for j=1:numel(helperFiles)
 helpers=append(helpers,struct('file',helperFiles(j).name, ...
  'sha256',sha256(fullfile(helperFiles(j).folder,helperFiles(j).name))));
end
report=struct('status','running','date','2026-10-02','phase',phase, ...
 'scope','Native private acquisition experiment; full normal-force history including startup. Finite cases, no physical sanding validation.', ...
 'frozenAcquisitionModelSha256',contract.modelSha256,'privateLoggingModelSha256',sha256(modelFile), ...
 'acquisitionContractSha256',sha256(fullfile(sourceFolder,'acquisition_contract.json')), ...
 'baselineCandidateSha256',sha256(fullfile(worker,'baseline_candidate.mat')), ...
 'recipeSha256',sha256(fullfile(worker,'qualify_adc_acquisition_20261002.m')), ...
 'helpers',helpers,'matlabVersion',version,'workerFolder',worker, ...
 'plannedUnpartitionedCases',planned,'selectedCaseIds',ids,'plannedCases',numel(configs), ...
 'loggingParity',parity,'compactColumns',{{'Rz_N','travel_m','I_A','V_V','contact','stop', ...
  'effortClip','voltageClip','impactImpulse_Ns','xB_m','xS_m','Fa_N','aB_m_s2', ...
  'lowerForce_N','N_N','Nestimate_N','Rx_N','slope','aS_m_s2'}},'rows',struct([]));
rows=struct([]);prepared=cell(1,numel(configs));plotTraces=struct([]);started=tic;lastKey='';
worst=-inf;bestWorstFile=fullfile(outputFolder,'worst_native.mat');
groups=struct;write_report('running');
for j=1:numel(configs)
 config=configs(j);timer=tic;out=[];trace=[];exception='';nativeReturned=false;
 row=blank_row(ids(j),config);
 try
  q=acf_prepare(config.p);prepared{j}=q;
  key=jsonencode(struct('h',q.adcStudy.fineClock_s,'dt',q.sim.step,'out',q.adcStudy.outputSampleTime_s,'seed',q.sensor.seed, ...
   'converter',q.adcStudy.converter,'firLength',numel(q.adcStudy.d.converter.numerator), ...
   'analogOrder',q.adcStudy.converterAnalogOrder,'delay',q.adcStudy.pipelineDelay_s));
  if options.useFastRestart
   if ~strcmp(lastKey,key),set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');lastKey=key;end
  else,set_param(mdl,'FastRestart','off');end
  prepare_model(mdl,q);out=sim(mdl);nativeReturned=true;
  log=out.get('adc_qualification_log');t=log.Time(:);a=tsrows(log,19);
  assert(numel(t)==size(a,1)&&all(isfinite(a(:)))&&t(1)==0);
  assert(all(abs(diff(t)-q.sim.step)<max(1e-12,q.sim.step*1e-6)),'A native physical force sample was omitted.');
  expectedCount=round(t(end)/q.sim.step)+1;assert(numel(t)==expectedCount);
  row=score(q,t,a,row);row.executed=true;
  assert(row.projectionBalanceResidual_N<1e-8 && row.rigidAxialBalanceResidual_N<1e-8 ...
   && row.lowerLinkBalanceResidual_N<1e-8,'Native force balance or logger contract failed.');
  adc=out.get('adc_acquisition_log');adcTime=adc.Time(:);adcData=tsrows(adc,8);
  assert(abs(numel(adcTime)-(floor((t(end)+1e-10)/q.adcStudy.outputSampleTime_s)+1))<=1);
  row.adcOutputFrames=numel(adcTime);row.anyAdcClipping=any(any(adcData(:,5:8)>.5));
  row.peakPrequantizedAcceleration_m_s2=max(max(abs(adcData(:,3:4))));
  row.passed=row.passed&&~row.anyAdcClipping;row.designMarginPassed=row.passed&&row.peakNormalForceError_N<=.45;
  row.hardware=q.adcStudy;row.hardware=rmfield(row.hardware,'d');
  row.converterGroupDelay_s=q.adcStudy.d.converter.exactGroupDelay_s;
  row.converterApproximationErrorThrough5k=q.adcStudy.d.converter.responseApproximation.maximumComplexErrorThrough5k;
  row.analogGroupDelay_s=q.adcStudy.d.channels.force.analogDCGroupDelay_s;
  row.converterAnalogGroupDelay_s=q.adcStudy.d.converterAnalog.DCGroupDelay_s;
  row.emaGroupDelay_s=q.adcStudy.d.channels.force.emaDCGroupDelay_s;
  row.parametersFile='prepared_configs.mat';row.parametersIndex=j;
  trace=struct('time',t,'data',a,'parameters',q,'columns',{report.compactColumns},'row',row, ...
   'adcTime',adcTime,'adcData',adcData);
  groupName=matlab.lang.makeValidName(config.group);
  if ~isfield(groups,groupName)||row.peakNormalForceError_N>groups.(groupName)
   groups.(groupName)=row.peakNormalForceError_N;
   save(fullfile(outputFolder,['group_' groupName '_worst_native.mat']),'trace','-v7.3');
  end
  if row.peakNormalForceError_N>worst
   worst=row.peakNormalForceError_N;save(bestWorstFile,'trace','-v7.3');
  end
  if options.keepAllTraces||~row.passed
   save(fullfile(outputFolder,sprintf('case_%04d_native.mat',ids(j))),'trace','-v7.3');
  end
  plotTraces=append(plotTraces,excerpt(trace));
 catch failure
  exception=getReport(failure,'extended','hyperlinks','off');
  row.reason=exception;row.executed=false;
  if nativeReturned,save(fullfile(outputFolder,sprintf('case_%04d_unvalidated.mat',ids(j))),'out','config','exception','-v7.3');end
 end
 row.nativeReturned=nativeReturned;row.runtime_s=toc(timer);rows=append(rows,row);
 fid=fopen(fullfile(outputFolder,'rows.jsonl'),'a');assert(fid~=-1);fprintf(fid,'%s\n',jsonencode(row));fclose(fid);
 clear out trace t a log adc adcTime adcData;Simulink.sdi.clear;
 save(fullfile(outputFolder,'prepared_configs.mat'),'prepared','-v7.3');
 save(fullfile(outputFolder,'plot_traces.mat'),'plotTraces','-v7.3');
 write_report('running');
 fprintf('ADC_NATIVE_PROGRESS %s %d/%d valid%d pass%d peak%.9gN time%.1fs\n', ...
  phase,j,numel(configs),sum([rows.executed]),sum([rows.passed]),row.peakNormalForceError_N,toc(started));
 if ~row.executed,write_report('software_rejection');error('acf:adc:NativeContract','Case%d: %s',ids(j),exception);end
 if isfile(fullfile(worker,'stop_after_case')),write_report('stopped_by_coordinator');return;end
end
assert(strcmp(sha256(modelSource),contract.modelSha256),'Frozen acquisition source changed.');
write_report('complete');

 function write_report(status)
  report.status=status;report.rows=rows;report.attempts=numel(rows);
  report.elapsed_s=toc(started);report.worstNativeFile=bestWorstFile;
  if isempty(rows)
   report.executed=0;report.passed=0;report.rejected=0;report.worstNormalForceError_N=NaN;report.nativeSamples=0;
  else
   report.executed=sum([rows.executed]);report.passed=sum([rows.passed]);report.rejected=sum(~[rows.executed]);
   report.worstNormalForceError_N=max([rows.peakNormalForceError_N]);report.nativeSamples=sum([rows.nativeSamples]);
  end
  fid=fopen(fullfile(outputFolder,'results.json'),'w');assert(fid~=-1);
  fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
 end
end

function configs=make_configs(base,phase,o,project)
configs=struct([]);
switch phase
 case {'sar_screen','sinc_screen'}
  if strcmp(phase,'sar_screen'),converter='sar';rates=o.sarRates_Hz;else,converter='sinc3';rates=o.sincRates_Hz;end
  for rate=rates
   for fc=o.cutoffs_Hz
    for point=1:2
     if point==1,m=2;f=50;angle=90;else,m=.1;f=5;angle=0;end
     for seed=o.seeds,add(physical_case(base,m,f,angle,.3,seed),converter,rate,fc,seed,'nominal_endpoint');end
    end
   end
  end
 case 'range_grid'
  for m=o.massValues_kg
   for f=o.forceValues_N
    for angle=o.angleValues_deg
     for mu=o.muValues
      for seed=o.gridSeeds,add(physical_case(base,m,f,angle,mu,seed),o.selectedConverter,o.selectedRate_Hz,o.selectedCutoff_Hz,seed,'nominal_range_grid');end
     end
    end
   end
  end
 case {'calibrated_long','calibrated_mixed'}
  if strcmp(phase,'calibrated_long')
   file=fullfile(project,'validation','inertial_uncertainty_v8_calibrated_static_long','inertial_uncertainty_calibrated_static_long_20260930_prepared_configs.mat');
   expected='be9e5f65ff211ee7a1d086574da499a74dcfde5f5168a4aad585d9c7a3701a7a';
  else
   file=fullfile(project,'validation','inertial_mechanics_calibrated_static_mixed_source_20260930','prepared_static_configurations.mat');
   expected='fe2376a79b1084096c3b808bf6cdda4aa68431671cb5dac1b3ab35d9c359726c';
  end
  assert(strcmp(sha256(file),expected));retained=load(file,'configs');
  for k=1:numel(retained.configs)
   q=retained.configs(k).p;add(q,o.selectedConverter,o.selectedRate_Hz,o.selectedCutoff_Hz,q.sensor.seed,phase);
   configs(end).retainedParameterSource=file;configs(end).retainedParameterSha256=expected;configs(end).retainedParameterIndex=k;
  end
 case {'noise_latency','alias_probe','refinement'}
  for seed=o.seeds
   q=physical_case(base,2,50,90,.3,seed);
   if strcmp(phase,'alias_probe')
    q.sim.stopTime=o.probeStopTime_s;
    for frequency=o.toneFrequencies_Hz
     add(q,o.selectedConverter,o.selectedRate_Hz,o.selectedCutoff_Hz,seed,phase);
     configs(end).p.adcStudy.analogToneFrequency_Hz=frequency;
     configs(end).p.adcStudy.accelerationZAnalogToneAmplitude_m_s2=o.toneAmplitude_m_s2;
     configs(end).group=[configs(end).group sprintf('_tone_%g',frequency)];
    end
   else,add(q,o.selectedConverter,o.selectedRate_Hz,o.selectedCutoff_Hz,seed,phase);end
  end
 otherwise,error('Unknown ADC experiment phase.');
end
 function add(q,converter,rate,fc,seed,scenario)
  q.adcStudy=hardware(converter,rate,fc,o);
  q.adcStudy.pipelineDelay_s=q.adcStudy.pipelineDelay_s+q.acquisition.pipelineDelay_s-20e-6;
  q.adcStudy.identifiedPipelineDelay_s=20e-6+q.acquisition.identifiedPipelineDelay_s-20e-6;
  q.sim.step=q.adcStudy.fineClock_s;
  if isfinite(o.solverStep_s),q.sim.step=o.solverStep_s;end
  if seed==0,q.adcStudy.forceAdcNoiseRms_N=0;q.adcStudy.accelerationAdcNoiseRms_m_s2=0;end
  name=sprintf('%s_%gHz_fc%gHz',converter,rate,fc);
  config=struct('p',q,'scenario',scenario,'seedRequested',seed,'group',name, ...
   'retainedParameterSource','','retainedParameterSha256','','retainedParameterIndex',0);
  configs=append(configs,config);
 end
end

function s=hardware(converter,rate,fc,o)
g=9.80665;
if strcmp(converter,'sar')
 h=10e-6;forceQ=320/2^16;accQ=5/2^16/.04*g;
 forceTotal=.003225558960381975;accTotal=.012356221788605425;order=3;range=62.5;
else
 h=1/300000;osr=3840000/rate;
 noiseTable=[128 21.31;256 10.68;512 7.56];idx=find(noiseTable(:,1)==osr,1);assert(~isempty(idx));
 voltageNoise=noiseTable(idx,2)*1e-6;forceQ=320/2^24;accQ=2.4/2^24/(.5*.04/g);
 forceTotal=voltageNoise/(1.2/160);accTotal=voltageNoise/(.5*.04/g);order=0;range=60;
end
if isfinite(o.fineClock_s),h=o.fineClock_s;end
s=struct('converter',converter,'outputRate_Hz',rate,'modulatorRate_Hz',3840000, ...
 'fineClock_s',h,'analogCutoff_Hz',fc,'identifiedAnalogCutoff_Hz',fc, ...
 'converterAnalogOrder',order,'converterAnalogCutoff_Hz',16000, ...
 'pipelineDelay_s',20e-6+o.pipelineExtra_s,'identifiedPipelineDelay_s',20e-6, ...
 'forceQuantum_N',forceQ,'accelerationQuantum_m_s2',accQ, ...
 'forceAdcNoiseRms_N',o.adcNoiseMultiplier*sqrt(max(0,forceTotal^2-forceQ^2/12)), ...
 'accelerationAdcNoiseRms_m_s2',o.adcNoiseMultiplier*sqrt(max(0,accTotal^2-accQ^2/12)), ...
 'forceAdcRange_N',160,'accelerationAdcRange_g',range, ...
 'adcNoiseCorrelationTime_s',o.adcNoiseCorrelationTime_s, ...
 'accelerationExtraDelay_s',o.accelerationExtraDelay_s);
end

function q=physical_case(p,m,f,angle,mu,seed)
q=p;q.mechanics.mS=m;q.mechanics.mB=.6;q.force.reference=f;
q.mechanics.angle=angle*pi/180;q.contact.mu=mu;q.coil.strategy='pi';
q.sensor.distalMass_kg=m;q.sensor.distalMass_model_kg=m;q.sensor.distalMassCalibrationScale=1;
q.sensor.seed=seed;if seed==0,q.sensor.seed=29;end
q.sensor.noiseRms=.02;q.contact.lateralNoiseRms_N=.02;q.sensor.strokeNoiseRms=1e-6;q.accel.noiseRms=.014;
if seed==0,q.sensor.noiseRms=0;q.contact.lateralNoiseRms_N=0;q.sensor.strokeNoiseRms=0;q.accel.noiseRms=0;end
end

function add_compact_logger(mdl)
roots={'plant_log','normal_contact_log','surface_log'};
indices={'[1 2 14 15 18 21 22 23 20 6 7 13 12 24]','[1 4 2 3]','3'};widths=[25 8 3];
add_block('simulink/Signal Routing/Mux',[mdl '/ADC qualification mux'],'Inputs','3','Position',[3900 100 3905 245]);
for k=1:3
 blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace','VariableName',roots{k});assert(numel(blocks)==1);
 ph=get_param(blocks{1},'PortHandles');ln=get_param(ph.Inport(1),'Line');src=get_param(ln,'SrcPortHandle');
 name=[mdl '/ADC qualification selector ' num2str(k)];
 add_block('simulink/Signal Routing/Selector',name,'InputPortWidth',num2str(widths(k)), ...
  'Indices',indices{k},'Position',[3550 100+90*k 3800 150+90*k]);
 sh=get_param(name,'PortHandles');add_line(mdl,src,sh.Inport(1),'autorouting','on');
 add_line(mdl,['ADC qualification selector ' num2str(k) '/1'],['ADC qualification mux/' num2str(k)],'autorouting','on');
end
add_block('simulink/Sinks/To Workspace',[mdl '/ADC full native qualification log'], ...
 'VariableName','adc_qualification_log','SaveFormat','Timeseries','MaxDataPoints','inf', ...
 'Decimation','1','SampleTime','-1','Position',[4070 125 4330 195]);
add_line(mdl,'ADC qualification mux/1','ADC full native qualification log/1','autorouting','on');
end

function set_historical_logging(mdl,enabled)
blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace');
for k=1:numel(blocks)
 if ismember(get_param(blocks{k},'VariableName'),{'adc_qualification_log','adc_acquisition_log'}),continue;end
 if enabled,comment='off';else,comment='on';end
 set_param(blocks{k},'Commented',comment);
end
end

function prepare_model(mdl,q)
assignin(get_param(mdl,'ModelWorkspace'),'p',q);
set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
end

function row=blank_row(id,c)
p=c.p;
row=struct('id',id,'group',c.group,'scenario',c.scenario,'massS_kg',p.mechanics.mS, ...
 'reference_N',p.force.reference,'angle_deg',p.mechanics.angle*180/pi,'mu',p.contact.mu, ...
 'seedRequested',c.seedRequested,'executed',false,'nativeReturned',false,'passed',false, ...
 'designMarginPassed',false,'nativeSamples',0,'endTime_s',NaN,'completed',false, ...
 'peakNormalForceError_N',NaN,'peakTime_s',NaN,'rmsNormalForceError_N',NaN, ...
 'peakStartupError_N',NaN,'peakBumpError_N',NaN,'peakFlatError_N',NaN, ...
 'minimumMargin_N',NaN,'peakCurrent_A',NaN,'peakVoltage_V',NaN,'lostContact',false, ...
 'strokeStop',false,'impact',false,'overCurrent',false,'springPackingPassed',false, ...
 'projectionBalanceResidual_N',NaN,'rigidAxialBalanceResidual_N',NaN, ...
 'lowerLinkBalanceResidual_N',NaN,'minimumPreload_N',NaN,'maximumPreload_N',NaN, ...
 'combinedFreeLength_mm',NaN,'combinedInstalledLength_mm',NaN,'voltageClipFraction',NaN, ...
 'runtime_s',NaN,'reason','','hardware',struct,'converterGroupDelay_s',NaN, ...
 'converterApproximationErrorThrough5k',NaN,'analogGroupDelay_s',NaN, ...
 'converterAnalogGroupDelay_s',NaN,'emaGroupDelay_s',NaN, ...
 'parametersFile','','parametersIndex',0,'adcOutputFrames',0,'anyAdcClipping',false, ...
 'peakPrequantizedAcceleration_m_s2',NaN);
end

function row=score(p,t,a,row)
N=a(:,15);err=N-p.force.reference;[peak,index]=max(abs(err));
row.nativeSamples=numel(t);row.endTime_s=t(end);row.completed=t(end)>=p.sim.stopTime-p.sim.step/2;
row.peakNormalForceError_N=peak;row.peakTime_s=t(index);row.minimumMargin_N=.5-peak;
row.rmsNormalForceError_N=sqrt(mean(err.^2));row.peakStartupError_N=masked_peak(err,t<=.25);
row.peakBumpError_N=masked_peak(err,t>=p.surface.startTime-.05 & t<=p.surface.startTime+p.d.duration+.2);
row.peakFlatError_N=masked_peak(err,t>.25 & t<p.surface.startTime-.05);
row.peakCurrent_A=max(abs(a(:,3)));row.peakVoltage_V=max(abs(a(:,4)));
row.lostContact=any(a(:,5)<.5);row.strokeStop=any(a(:,6)>.5);row.impact=any(abs(a(:,9))>1e-10);
row.overCurrent=row.peakCurrent_A>p.coil.peakCurrent*(1+1e-4);row.voltageClipFraction=mean(a(:,8)>.5);
attached=a(:,5)>.5 & a(:,6)<.5;
Rz=a(:,1);Rx=a(:,17);slope=a(:,18);Fa=a(:,12);aB=a(:,13);aS=a(:,19);
row.projectionBalanceResidual_N=max(abs(N-(Rz-slope.*Rx)./sqrt(1+slope.^2)));
row.rigidAxialBalanceResidual_N=masked_peak(Rz-Fa-p.mechanics.mB*(p.d.g+aB)-p.mechanics.mS*(p.d.g+aS),attached);
row.lowerLinkBalanceResidual_N=masked_peak(Rz-a(:,14)-p.mechanics.mS*(p.d.g+aS),attached);
q=p.d.q10/p.mechanics.k1+a(:,11)-a(:,10);travel=max(abs(q));
spring=p.springs.lower;pack=p.springs;
lowerMin=spring.rate_N_m*(travel+pack.unseatingClearance_m);
compression=min(spring.freeLength_m-spring.freeLengthTolerance_m-spring.minWorkingLength_m ...
 -pack.workingClearance_m,spring.forceEnvelope_N/spring.rate_N_m);
lowerMax=spring.rate_N_m*(compression-travel);
row.minimumPreload_N=lowerMin;row.maximumPreload_N=lowerMax;
row.combinedFreeLength_mm=2000*(spring.freeLength_m+spring.freeLengthTolerance_m);
row.combinedInstalledLength_mm=2000*(spring.freeLength_m+spring.freeLengthTolerance_m-p.springs.commonPreload_N/spring.rate_N_m);
row.springPackingPassed=p.springs.commonPreload_N>=lowerMin && p.springs.commonPreload_N<=lowerMax ...
 && max(q)-min(q)<=spring.maxCyclicStroke_m+1e-12 ...
 && row.combinedFreeLength_mm<=100 && row.combinedInstalledLength_mm<=100;
row.passed=row.completed&&peak<=.5&&~row.lostContact&&~row.strokeStop&&~row.impact&&~row.overCurrent&&row.springPackingPassed;
row.designMarginPassed=row.passed&&peak<=.45;
if row.passed,row.reason='Full native normal-force history passed';else,row.reason='Native physical or force requirement failed';end
end

function e=excerpt(trace)
t=trace.time;around=t>=1.95&t<=2.4;indices=find(around);stride=max(1,round(.0001/trace.parameters.sim.step));
indices=unique([indices(1:stride:end);find(t==trace.row.peakTime_s)]);
e=struct('row',trace.row,'time',t(indices),'data',trace.data(indices,:));
end

function a=tsrows(ts,n)
a=ts.Data;t=numel(ts.Time);if size(a,1)~=t,a=permute(a,[ndims(a),1:ndims(a)-1]);end;a=reshape(a,t,n);
end
function value=masked_peak(a,mask)
if any(mask),value=max(abs(a(mask)));else,value=NaN;end
end
function s=with_defaults(s,d)
for n=fieldnames(d)',if ~isfield(s,n{1}),s.(n{1})=d.(n{1});end,end
end
function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end
function finish(mdl,storage,record,sdi)
try,set_param(mdl,'FastRestart','off');close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_acf_adc_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
d=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');d.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
