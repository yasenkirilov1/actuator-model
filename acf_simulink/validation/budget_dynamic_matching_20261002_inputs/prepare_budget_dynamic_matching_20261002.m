function report=prepare_budget_dynamic_matching_20261002(root,outputFolder)
%PREPARE_BUDGET_DYNAMIC_MATCHING_20261002 Prepare paired dynamic-match targets.
% No simulations. Retain every static, mechanical, robot and drive parameter.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,outputFolder=fullfile(root,'acf_simulink','validation','budget_dynamic_matching_20261002_inputs');end
assert(~isfile(fullfile(outputFolder,'manifest.json')),'Preserve already frozen inputs.');
if ~isfolder(outputFolder),mkdir(outputFolder);end
validation=fullfile(root,'acf_simulink','validation');
native=fullfile(validation,'adc_acquisition_trial_20261002');
modelFile=fullfile(native,'acf_force_control_adc_trial.slx');modelSha=sha256(modelFile);
assert(strcmp(modelSha,'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
retained=fullfile(validation,'budget_sensor_stress_20261002_inputs');
names={'ADXL354C_1900_L0_current50_36V','ADXL354C_1900_L50_current50_36V'};
expectedHashes={'ec3af2bb47f59778fd73fb6b0214716889ef665448e8c512e7b9c85346bd0506', ...
 'effa773a1c1a29cea589c46d0ecec51d30804fa2b395c1b4239bbc98006735ff'};
addpath(fullfile(native,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
channels={'force','lateral','acceleration','position'};groups=struct([]);rows=struct([]);
for groupIndex=1:2
 sourceFile=fullfile(retained,[names{groupIndex} '_configs.mat']);sourceSha=sha256(sourceFile);
 assert(strcmp(sourceSha,expectedHashes{groupIndex}));loaded=load(sourceFile,'configs');
 assert(numel(loaded.configs)==7);name=[names{groupIndex} '_dynamic_match'];configs=loaded.configs;
 for k=1:7
  before=loaded.configs(k).p;p=before;changes=struct([]);optionalEmaFields={};
  assert(abs(p.coil.currentResponse-50e-6)<1e-15&&p.coil.voltage==36);
  assert(p.adcStudy.analogCutoff_Hz==1900&&p.adcStudy.outputRate_Hz==30000);
  assert(p.adcStudy.identifiedAnalogCutoff_Hz==1900&&p.accel.range_g==8);
  if k<=4,assert(p.sim.stopTime==30);else,assert(p.sim.stopTime==4);end
  for j=1:4
   field=[channels{j} 'AntiAliasScale'];
   [p,changes]=change(p,changes,field,1,.1);
   field=[channels{j} 'EmaScale'];
   if isfield(p.acquisition,field)
    [p,changes]=change(p,changes,field,1,.1);optionalEmaFields{end+1}=field; %#ok<AGROW>
   end
   [p,changes]=change(p,changes,[channels{j} 'DelayOffset_s'],0,.2);
  end
  [p,changes]=change(p,changes,'antiAliasCutoff_Hz',before.acquisition.identifiedAntiAliasCutoff_Hz,.1);
  [p,changes]=change(p,changes,'emaTime_s',before.acquisition.identifiedEmaTime_s,.1);
  expectedAcquisition=p.acquisition;
  p=acf_prepare(p);
  assert(isequaln(p.acquisition,expectedAcquisition),'An acquisition field outside the requested targets changed.');
  assert(p.acquisition.pipelineDelay_s==before.acquisition.pipelineDelay_s);
  assert(p.adcStudy.pipelineDelay_s==before.adcStudy.pipelineDelay_s);
  assert(p.acquisition.identifiedPipelineDelay_s==before.acquisition.identifiedPipelineDelay_s);
  assert(p.adcStudy.identifiedPipelineDelay_s==before.adcStudy.identifiedPipelineDelay_s);
  expectedStudy=before.adcStudy;actualStudy=p.adcStudy;
  expectedStudy=rmfield(expectedStudy,'d');actualStudy=rmfield(actualStudy,'d');
  assert(isequaln(actualStudy,expectedStudy),'ADC noise, density, range, converter, latency or nominal response changed.');
  originalFields=fieldnames(before);actualFields=fieldnames(p);
  assert(isequal(sort(originalFields),sort(actualFields)));
  for j=1:numel(originalFields)
   field=originalFields{j};
   if ~any(strcmp(field,{'acquisition','adcStudy','d'}))
    assert(isequaln(p.(field),before.(field)),['Unrequested parameter change under p.' field]);
   end
  end
  actualPipelines=zeros(1,4);originalPipelines=zeros(1,4);
  for j=1:4
   channel=channels{j};
   originalPipelines(j)=before.adcStudy.d.channels.(channel).pipelineDelay_s;
   actualPipelines(j)=p.adcStudy.d.channels.(channel).pipelineDelay_s;
   expected=p.adcStudy.pipelineDelay_s+p.acquisition.([channel 'DelayOffset_s']);
   if strcmp(channel,'acceleration'),expected=expected+p.adcStudy.accelerationExtraDelay_s;end
   assert(abs(actualPipelines(j)-expected)<1e-14);
   assert(isequaln(p.adcStudy.d.channels.frame,before.adcStudy.d.channels.frame));
  end
  configs(k).p=p;configs(k).scenario='budget_dynamic_matching_stress';configs(k).group=name;
  configs(k).retainedParameterSource=sourceFile;configs(k).retainedParameterSha256=sourceSha;
  configs(k).retainedParameterIndex=k;
  scope=['Tighter assembled dynamic-matching acceptance target: AA and EMA residuals contracted by 0.1; ' ...
   'relative delivery offsets contracted by 0.2. Common actual pipeline residual, static calibration, ' ...
   'robot timing, mechanics, noise PSD, sensor range and 50 us / 36 V drive remain. ' ...
   'This is neither perfect sensing nor measured manufacturer part qualification.'];
  row=struct('group',name,'caseIndex',k,'sourceFile',sourceFile,'sourceSha256',sourceSha, ...
   'seed',p.sensor.seed,'duration_s',p.sim.stopTime,'changes',changes, ...
   'optionalPerChannelEmaFieldsFound',{optionalEmaFields}, ...
   'actualCommonPipeline_us',p.adcStudy.pipelineDelay_s*1e6, ...
   'identifiedCommonPipeline_us',p.adcStudy.identifiedPipelineDelay_s*1e6, ...
   'originalActualChannelPipelines_us',originalPipelines*1e6, ...
   'newActualChannelPipelines_us',actualPipelines*1e6, ...
   'identifiedFrameUnchanged',true,'scope',scope,'executed',false);
  if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 end
 file=fullfile(outputFolder,[name '_configs.mat']);save(file,'configs','-v7');
 group=struct('name',name,'file',file,'sha256',sha256(file),'cases',7,'longCases',4,'mixedCases',3, ...
  'totalRecordingDuration_s',132,'sourceFile',sourceFile,'sourceSha256',sourceSha, ...
  'primary',groupIndex==1,'scope',scope,'executed',false);
 if isempty(groups),groups=group;else,groups(end+1)=group;end %#ok<AGROW>
end
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'prepare_budget_dynamic_matching_20261002.m'));
report=struct('date','2026-10-02','status','prepared_inputs_not_executed','sourceModelFile',modelFile, ...
 'sourceModelSha256',modelSha,'recipeSha256',sha256(fullfile(outputFolder,'prepare_budget_dynamic_matching_20261002.m')), ...
 'plannedGroups',2,'plannedCases',14,'primaryCases',7,'executedNativeCases',0,'groups',groups,'rows',rows, ...
 'guardSummary',struct('onlyRequestedInputFieldsChanged',true,'commonActualPipelineResidualPreserved',true, ...
 'allIdentifiedDynamicsPreserved',true,'allStaticCalibrationAndRobotResidualsPreserved',true, ...
 'mechanicsDriveNoiseAndRangePreserved',true,'sourceModelUnchanged',strcmp(sha256(modelFile),modelSha)), ...
 'claim','Prepared finite paired native-input comparisons only; actual execution is required. Tighter dynamic matching does not establish better static calibration or physical part qualification.');
assert(all(structfun(@(x)logical(x),report.guardSummary)));write_json(fullfile(outputFolder,'manifest.json'),report);
end

function [p,changes]=change(p,changes,field,target,factor)
original=p.acquisition.(field);updated=target+factor*(original-target);p.acquisition.(field)=updated;
record=struct('field',['p.acquisition.' field],'original',original,'updated',updated, ...
 'delta',updated-original,'target',target,'residualMultiplier',factor);
if isempty(changes),changes=record;else,changes(end+1)=record;end
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
