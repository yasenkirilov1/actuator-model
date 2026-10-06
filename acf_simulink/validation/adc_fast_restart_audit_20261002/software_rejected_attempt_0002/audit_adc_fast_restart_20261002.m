function report=audit_adc_fast_restart_20261002(sourceFolder,outputFolder)
%AUDIT_ADC_FAST_RESTART_20261002 Native cold/warm parameter-causality audit.
% This is a software execution audit, not additional hardware qualification.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
assert(isfolder(sourceFolder)&&~isfile(fullfile(outputFolder,'results.json')));
if ~isfolder(outputFolder),mkdir(outputFolder);end
worker=fullfile(outputFolder,'private_worker');
for name={'','helpers','cache','code'},mkdir(fullfile(worker,name{1}));end
contract=jsondecode(fileread(fullfile(sourceFolder,'acquisition_contract.json')));
helperManifest=jsondecode(fileread(fullfile(sourceFolder,'helper_manifest.json')));
modelSource=fullfile(sourceFolder,'acf_force_control_adc_trial.slx');
assert(strcmp(contract.modelSha256,'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
assert(strcmp(sha256(modelSource),contract.modelSha256));
assert(numel(helperManifest.files)==18);
for k=1:numel(helperManifest.files)
 item=helperManifest.files(k);assert(strcmp(sha256(fullfile(sourceFolder,item.file)),item.sha256));
 copyfile(fullfile(sourceFolder,item.file),fullfile(worker,item.file));
 assert(strcmp(sha256(fullfile(worker,item.file)),item.sha256));
end
copyfile(modelSource,fullfile(worker,'acf_force_control_adc_trial.slx'));
copyfile(fullfile(sourceFolder,'baseline_candidate.mat'),fullfile(worker,'baseline_candidate.mat'));
assert(strcmp(sha256(fullfile(worker,'baseline_candidate.mat')), ...
 'd8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7'));
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'audit_adc_fast_restart_20261002.m'));
copyfile(fullfile(sourceFolder,'helper_manifest.json'),fullfile(outputFolder,'frozen_helper_manifest.json'));
addpath(fullfile(worker,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
maxNumCompThreads(2);
saved=load(fullfile(worker,'baseline_candidate.mat'),'p');base=saved.p;
base.sim.evaluateFrom=0;base.robot.forceGate.filterTime=.1;base.robot.forceGate.dwellTime=.1;
base.uncertainty=struct('forceScaleZ',0,'forceScaleX',0,'positionScale',0, ...
 'forceFrameAngle_rad',0,'accelFrameAngle_rad',0,'forceCrossZFromX',0,'forceCrossXFromZ',0, ...
 'accelBiasX_m_s2',0,'massB_model_kg',base.mechanics.mB,'forceConstant_model_N_A',base.coil.forceConstant, ...
 'recenterGain',min(base.robot.recenterGain,base.robot.forceGate.maxDelayGainProduct/ ...
 (base.robotPrediction.delay+base.robotPrediction.responseTime+base.robot.forceGate.dwellTime+base.robot.forceGate.filterTime)));
assert(base.force.sampleTime==100e-6&&base.mechanics.upperRigid&&base.mechanics.mB==.6);
names={'heavy7_first_short','heavy29_warm_short','heavy29_cold_short', ...
 'heavy7_cold_long','heavy7_first_long','light7_warm_long','light7_cold_long', ...
 'heavy7_fc500_first_long','heavy7_fc5100_warm_long'};
mass=[2 2 2 2 2 .1 .1 2 2];force=[50 50 50 50 50 5 5 50 50];
angle=[90 90 90 90 90 0 0 90 90];seed=[7 29 29 7 7 7 7 7 7];
cutoff=[5100 5100 5100 5100 5100 5100 5100 500 5100];
duration=[.25 .25 .25 4 4 4 4 4 4];mode={'first','warm','cold','cold','first','warm','cold','first','warm'};
configs=struct([]);
for k=1:numel(names)
 q=physical_case(base,mass(k),force(k),angle(k),seed(k));
 q.sim.step=1/300000;q.sim.stopTime=duration(k);q.adcStudy=hardware(cutoff(k));q=acf_prepare(q);
 configs=append(configs,struct('name',names{k},'mode',mode{k},'p',q));
end
assert(isequal(shape(configs(1).p),shape(configs(2).p)));
assert(isequal(shape(configs(5).p),shape(configs(6).p)));
assert(isequal(shape(configs(8).p),shape(configs(9).p)));
save(fullfile(outputFolder,'prepared_configs.mat'),'configs','-v7.3');
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
sdi=tempname('/dev/shm');mkdir(sdi);write_text(fullfile(sdi,'owned_acf_adc_fast_restart_sdi'),'Owned transient audit storage.');
mdl='acf_force_control_adc_trial';assert(~bdIsLoaded(mdl));
modelFile=fullfile(worker,[mdl '.slx']);load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
set_param(mdl,'SimulationMode','accelerator');add_compact_logger(mdl);
logs=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace');
for k=1:numel(logs)
 if ~ismember(get_param(logs{k},'VariableName'),{'adc_qualification_log','adc_acquisition_log'})
  set_param(logs{k},'Commented','on');
 end
end
save_system(mdl,modelFile);
columns={'Rz_N','travel_m','I_A','V_V','contact','stop','effortClip','voltageClip', ...
 'impactImpulse_Ns','xB_m','xS_m','Fa_N','aB_m_s2','lowerForce_N','N_N','Nestimate_N','Rx_N','slope','aS_m_s2'};
rows=struct([]);comparisons=struct([]);started=tic;
report=struct('status','running','date','2026-10-02','scope','Software cold/warm execution parity audit; full native histories including startup.', ...
 'sourceModelSha256',contract.modelSha256,'privateLoggingModelSha256',sha256(modelFile), ...
 'helperManifestSha256',sha256(fullfile(sourceFolder,'helper_manifest.json')), ...
 'recipeSha256',sha256(fullfile(outputFolder,'audit_adc_fast_restart_20261002.m')), ...
 'matlabVersion',version,'simulationMode','accelerator','controllerSampleTime_s',base.force.sampleTime, ...
 'fineClock_s',1/300000,'adcOutputRate_Hz',30000,'compactColumns',{columns}, ...
 'seedBlocks',seed_metadata(mdl),'plannedRuns',numel(configs),'rows',rows,'comparisons',comparisons);
write_report('running');
try
 for k=1:numel(configs)
  c=configs(k);q=c.p;
  if ~strcmp(c.mode,'warm')
   set_param(mdl,'FastRestart','off');
   if strcmp(c.mode,'first'),set_param(mdl,'FastRestart','on');end
  else,assert(strcmp(get_param(mdl,'FastRestart'),'on'));end
  assignin(get_param(mdl,'ModelWorkspace'),'p',q);
  set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
  lastwarn('');timer=tic;transcript=evalc('out=sim(mdl);');elapsed=toc(timer);
  [warningMessage,warningId]=lastwarn;
  log=out.get('adc_qualification_log');adc=out.get('adc_acquisition_log');
  t=log.Time(:);data=tsrows(log,19);adcTime=adc.Time(:);adcData=tsrows(adc,8);
  assert(t(1)==0&&all(isfinite(data(:)))&&all(isfinite(adcData(:))));
  assert(numel(t)==round(t(end)/q.sim.step)+1&&all(abs(diff(t)-q.sim.step)<1e-12));
  assert(t(end)>=q.sim.stopTime-q.sim.step/2,'A guard stopped the audit run early.');
  errorN=data(:,15)-q.force.reference;[peak,index]=max(abs(errorN));
  projection=max(abs(data(:,15)-(data(:,1)-data(:,18).*data(:,17))./sqrt(1+data(:,18).^2)));
  attached=data(:,5)>.5&data(:,6)<.5;
  axial=max(abs(data(attached,1)-data(attached,12)-q.mechanics.mB*(q.d.g+data(attached,13))-q.mechanics.mS*(q.d.g+data(attached,19))));
  lower=max(abs(data(attached,1)-data(attached,14)-q.mechanics.mS*(q.d.g+data(attached,19))));
  assert(max([projection axial lower])<1e-8);
  row=struct('name',c.name,'mode',c.mode,'seed',q.sensor.seed,'massS_kg',q.mechanics.mS, ...
   'reference_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi,'cutoff_Hz',q.adcStudy.analogCutoff_Hz, ...
   'nativeSamples',numel(t),'adcOutputFrames',numel(adcTime),'endTime_s',t(end),'elapsed_s',elapsed, ...
   'peakNormalForceError_N',peak,'peakTime_s',t(index),'startupPeak_N',max(abs(errorN(t<=.25))), ...
   'projectionResidual_N',projection,'axialResidual_N',axial,'lowerResidual_N',lower, ...
   'lastWarningId',warningId,'lastWarningMessage',warningMessage,'transcriptFile',[c.name '_transcript.txt']);
  trace=struct('time',t,'data',data,'adcTime',adcTime,'adcData',adcData,'parameters',q,'row',row,'columns',{columns});
  write_text(fullfile(outputFolder,row.transcriptFile),transcript);
  save(fullfile(outputFolder,[c.name '_native.mat']),'trace','-v7.3');
  row.traceFile=[c.name '_native.mat'];row.traceSha256=sha256(fullfile(outputFolder,row.traceFile));
  row.seedRuntimeMetadata=seed_runtime_metadata(mdl);rows=append(rows,row);
  clear out log adc t data adcTime adcData trace transcript errorN attached;Simulink.sdi.clear;
  if k==3
   comparisons=append(comparisons,compare_histories(outputFolder,names{2},names{3},'seed_warm29_vs_cold29'));
   comparisons=append(comparisons,compare_histories(outputFolder,names{1},names{3},'seed7_vs_cold29_contrast'));
  elseif k==7
   comparisons=append(comparisons,compare_histories(outputFolder,names{6},names{7},'physical_warmLight_vs_coldLight'));
   comparisons=append(comparisons,compare_histories(outputFolder,names{5},names{4},'heavy_first_vs_cold_repeat'));
  elseif k==9
   comparisons=append(comparisons,compare_histories(outputFolder,names{9},names{4},'cutoff_warm5100_vs_cold5100'));
   comparisons=append(comparisons,compare_histories(outputFolder,names{8},names{4},'cutoff500_vs_cold5100_contrast'));
  end
  write_report('running');fprintf('ADC_FAST_RESTART_AUDIT %d/%d %s %.9gN %.1fs\n',k,numel(configs),c.name,peak,toc(started));
 end
 assert(strcmp(sha256(modelSource),contract.modelSha256));
 for k=1:numel(helperManifest.files)
  item=helperManifest.files(k);assert(strcmp(sha256(fullfile(sourceFolder,item.file)),item.sha256));
  assert(strcmp(sha256(fullfile(worker,item.file)),item.sha256));
 end
 delete(cleanup);report.preferencesRestored=strcmp(Simulink.sdi.getStorageLocation,previousStorage)&&isequal(Simulink.sdi.getRecordData,previousRecord);
 assert(report.preferencesRestored);write_report('complete');
catch failure
 report.failure=getReport(failure,'extended','hyperlinks','off');write_report('software_rejection');rethrow(failure);
end
 function write_report(status)
  report.status=status;report.rows=rows;report.comparisons=comparisons;report.executedRuns=numel(rows);report.elapsed_s=toc(started);
  write_text(fullfile(outputFolder,'results.json'),jsonencode(report,'PrettyPrint',true));
 end
end

function q=physical_case(q,m,f,angle,seed)
q.mechanics.mS=m;q.mechanics.mB=.6;q.force.reference=f;q.mechanics.angle=angle*pi/180;q.contact.mu=.3;q.coil.strategy='pi';
q.sensor.distalMass_kg=m;q.sensor.distalMass_model_kg=m;q.sensor.distalMassCalibrationScale=1;q.sensor.seed=seed;
q.sensor.noiseRms=.02;q.contact.lateralNoiseRms_N=.02;q.sensor.strokeNoiseRms=1e-6;q.accel.noiseRms=.014;
end
function s=hardware(fc)
g=9.80665;forceQ=320/2^24;accQ=2.4/2^24/(.5*.04/g);v=21.31e-6;
s=struct('converter','sinc3','outputRate_Hz',30000,'modulatorRate_Hz',3840000, ...
 'fineClock_s',1/300000,'analogCutoff_Hz',fc,'identifiedAnalogCutoff_Hz',fc, ...
 'converterAnalogOrder',0,'converterAnalogCutoff_Hz',16000,'pipelineDelay_s',20e-6,'identifiedPipelineDelay_s',20e-6, ...
 'forceQuantum_N',forceQ,'accelerationQuantum_m_s2',accQ, ...
 'forceAdcNoiseRms_N',sqrt((v/(1.2/160))^2-forceQ^2/12), ...
 'accelerationAdcNoiseRms_m_s2',sqrt((v/(.5*.04/g))^2-accQ^2/12), ...
 'forceAdcRange_N',160,'accelerationAdcRange_g',60,'adcNoiseCorrelationTime_s',0,'accelerationExtraDelay_s',0);
end
function add_compact_logger(mdl)
roots={'plant_log','normal_contact_log','surface_log'};
indices={'[1 2 14 15 18 21 22 23 20 6 7 13 12 24]','[1 4 2 3]','3'};widths=[25 8 3];
add_block('simulink/Signal Routing/Mux',[mdl '/ADC qualification mux'],'Inputs','3','Position',[3900 100 3905 245]);
for k=1:3
 blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace','VariableName',roots{k});assert(numel(blocks)==1);
 ph=get_param(blocks{1},'PortHandles');src=get_param(get_param(ph.Inport(1),'Line'),'SrcPortHandle');
 name=[mdl '/ADC qualification selector ' num2str(k)];
 add_block('simulink/Signal Routing/Selector',name,'InputPortWidth',num2str(widths(k)), ...
  'Indices',indices{k},'Position',[3550 100+90*k 3800 150+90*k]);
 sh=get_param(name,'PortHandles');add_line(mdl,src,sh.Inport(1),'autorouting','on');
 add_line(mdl,['ADC qualification selector ' num2str(k) '/1'],['ADC qualification mux/' num2str(k)],'autorouting','on');
end
add_block('simulink/Sinks/To Workspace',[mdl '/ADC full native qualification log'],'VariableName','adc_qualification_log', ...
 'SaveFormat','Timeseries','MaxDataPoints','inf','Decimation','1','SampleTime','-1','Position',[4070 125 4330 195]);
add_line(mdl,'ADC qualification mux/1','ADC full native qualification log/1','autorouting','on');
end
function metadata=seed_metadata(mdl)
metadata=struct([]);blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'Type','Block');
for k=1:numel(blocks)
 dp=get_param(blocks{k},'DialogParameters');
 if isstruct(dp)&&isfield(dp,'Seed')
  metadata=append(metadata,struct('path',blocks{k},'blockType',get_param(blocks{k},'BlockType'), ...
   'seedExpression',get_param(blocks{k},'Seed'),'seedAttributes',{dp.Seed.Attributes}));
 end
end
end
function metadata=seed_runtime_metadata(mdl)
metadata=seed_metadata(mdl);
for k=1:numel(metadata)
 metadata(k).runtimeAvailable=false;metadata(k).dialogRuntimeParameters=struct([]);metadata(k).runtimeInspectionMessage='';
 try
  rto=get_param(metadata(k).path,'RuntimeObject');
  if isempty(rto),continue;end
  values=struct([]);
  for j=1:rto.NumDialogPrms
   prm=rto.DialogPrm(j);
   if isnumeric(prm.Data)&&numel(prm.Data)<=16,values=append(values,struct('index',j,'name',prm.Name,'data',prm.Data));end
  end
  metadata(k).runtimeAvailable=true;metadata(k).dialogRuntimeParameters=values;
 catch failure,metadata(k).runtimeInspectionMessage=failure.message;end
end
end
function result=compare_histories(folder,first,second,name)
a=load(fullfile(folder,[first '_native.mat']),'trace');b=load(fullfile(folder,[second '_native.mat']),'trace');
a=a.trace;b=b.trace;
sameT=isequal(a.time,b.time);sameAdcT=isequal(a.adcTime,b.adcTime);
assert(sameT&&sameAdcT,'Audit comparison clocks/counts differ.');
delta=a.data-b.data;adcDelta=a.adcData-b.adcData;
result=struct('name',name,'first',first,'second',second,'exactNativeTimes',sameT,'exactAdcTimes',sameAdcT, ...
 'exactNativeValues',isequal(a.data,b.data),'exactAdcValues',isequal(a.adcData,b.adcData), ...
 'nativeSamples',numel(a.time),'adcOutputFrames',numel(a.adcTime), ...
 'maximumAbsNativeDifferenceByColumn',max(abs(delta),[],1),'changedNativeSamplesByColumn',sum(delta~=0,1), ...
 'maximumAbsAdcDifferenceByColumn',max(abs(adcDelta),[],1),'changedAdcFramesByColumn',sum(adcDelta~=0,1), ...
 'exactNativeStartupValues',isequal(a.data(a.time<=.25,:),b.data(b.time<=.25,:)), ...
 'firstNativeData',a.data(1,:),'secondNativeData',b.data(1,:), ...
 'requestedParameterDifferences',{changed_leaves(a.parameters,b.parameters,'p')});
fprintf('ADC_FAST_RESTART_COMPARE %s nativeExact%d adcExact%d maxForceDelta%.9g\n',name,result.exactNativeValues,result.exactAdcValues,result.maximumAbsNativeDifferenceByColumn(15));
end
function differences=changed_leaves(a,b,prefix)
differences={};
if isstruct(a)&&isscalar(a)&&isstruct(b)&&isscalar(b)
 names=union(fieldnames(a),fieldnames(b));
 for k=1:numel(names)
  n=names{k};if ~isfield(a,n)||~isfield(b,n),differences{end+1}=[prefix '.' n]; %#ok<AGROW>
  else,differences=[differences changed_leaves(a.(n),b.(n),[prefix '.' n])];end %#ok<AGROW>
 end
elseif ~isequaln(a,b),differences={prefix};end
end
function out=shape(s)
if isstruct(s)&&isscalar(s)
 out=struct;names=fieldnames(s);for k=1:numel(names),out.(names{k})=shape(s.(names{k}));end
else,out=struct('class',class(s),'size',size(s));end
end
function a=tsrows(ts,n)
a=ts.Data;t=numel(ts.Time);if size(a,1)~=t,a=permute(a,[ndims(a),1:ndims(a)-1]);end;a=reshape(a,t,n);
end
function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end
function write_text(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s',value); %#ok<NASGU>
end
function finish(mdl,storage,record,sdi)
try,set_param(mdl,'FastRestart','off');close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_acf_adc_fast_restart_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
d=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');d.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
