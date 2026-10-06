function report=normal_contact_trial_delivery_sensor_metadata(work)
% Refresh descriptive sensor metadata with zero new native simulations.
% Execute in an isolated batch only after an existing mesh worker has exited.
if nargin<1,work=fullfile(getenv('HOME'),'.local','state','actuator-model','inertial-delivery-private-v8-layoutd-20260930');end
assert(~usejava('desktop'));mdl='acf_force_control_simple';
filename=fullfile(work,'generated',[mdl '.slx']);
original=fullfile(work,'generated','acf_force_control_simple_before_sensor_metadata.slx');
reviewedSHA='4fe56e2505f10e9adef86b29a7615583ec2b6dc6be37f1cf051d3a753fef0a44';
if isfile(original)
 assert(strcmp(sha256(original),reviewedSHA),'The reviewed original was altered.');
else
 assert(strcmp(sha256(filename),reviewedSHA),'Use the exact reviewed private model.');
 copyfile(filename,original);
end
if bdIsLoaded(mdl),assert(strcmp(get_param(mdl,'FileName'),filename),'Close your own loaded model before this metadata-only step.');end
oldPath=path;addpath(fullfile(work,'helpers'),'-begin');clear acf_prepare;
cleanup=onCleanup(@()finish(mdl,oldPath)); %#ok<NASGU>
load_system(filename);mw=get_param(mdl,'ModelWorkspace');
report=struct('status','running','newNativeSimulations',0,'sourceModelSHA256',sha256(original), ...
 'variables',{struct([])},'allParametersExceptSensorDesignIdentical',true);
for name={'p','pRecommended','pNoSprings','pCompliant','pNearContact'}
 if ~hasVariable(mw,name{1}),continue;end
 before=getVariable(mw,name{1});after=acf_prepare(before);
 physicalBefore=before;if isfield(physicalBefore,'sensorDesign'),physicalBefore=rmfield(physicalBefore,'sensorDesign');end
 physicalAfter=rmfield(after,'sensorDesign');
 assert(isequal(physicalBefore,physicalAfter), ...
  'The metadata refresh changed a physical/controller parameter in %s.',name{1});
 assert(after.sensorDesign.modeledRMS_m_s2==after.accel.noiseRms);
 assert(after.sensorDesign.commonAcquisitionDelay_s==after.sensor.delay);
 assignin(mw,name{1},after);
 row=struct('variable',name{1},'modeledNoiseRMS_m_s2',after.accel.noiseRms, ...
  'nominalNoiseRMS_m_s2',after.accel.nominalNoiseRms,'rawSampleTime_s',after.acquisition.rawSampleTime_s, ...
  'antiAliasCutoff_Hz',after.acquisition.antiAliasCutoff_Hz,'EMA_s',after.acquisition.emaTime_s, ...
  'responseGroupDelay_s',after.sensorDesign.responseGroupDelay_s,'pipeline_s',after.sensor.delay);
 if isempty(report.variables),report.variables=row;else,report.variables(end+1)=row;end %#ok<AGROW>
end
blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'LookUnderMasks','all','Type','block');
for k=1:numel(blocks)
 if strcmp(blocks{k},mdl),continue;end
 parameters=get_param(blocks{k},'ObjectParameters');
 for name=fieldnames(parameters)'
  try,value=get_param(blocks{k},name{1});catch,continue;end
  if ischar(value)||isstring(value),assert(~any(contains(string(value),'p.sensorDesign'),'all'),'Native block reads descriptive metadata.');end
 end
end
save_system(mdl);p=getVariable(mw,'pRecommended');save(fullfile(work,'delivery_candidate.mat'),'p','-v7');
close_system(mdl,0);
% The saved native system XML must remain byte-for-byte identical.
check=tempname(work);mkdir(check);report.xmlComparisonFolder=check;
unzip(original,fullfile(check,'before'));unzip(filename,fullfile(check,'after'));
files=dir(fullfile(check,'before','simulink','systems','*.xml'));
for k=1:numel(files)
 assert(strcmp(sha256(fullfile(check,'before','simulink','systems',files(k).name)), ...
  sha256(fullfile(check,'after','simulink','systems',files(k).name))),'Native graph XML changed.');
end
report.nativeSystemXMLIdentical=true;report.nativeSystemXMLCount=numel(files);
report.modelSHA256=sha256(filename);report.candidateSHA256=sha256(fullfile(work,'delivery_candidate.mat'));
report.preparationSHA256=sha256(fullfile(work,'helpers','acf_prepare.m'));report.status='complete';
fid=fopen(fullfile(work,'sensor_metadata_bridge.json'),'w');assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('DELIVERY SENSOR METADATA REFRESHED; ZERO NATIVE SIMULATIONS\n');
end
function value=sha256(file)
assert(~contains(string(file),"'"),'Unexpected quote in the task-owned filename.');
[status,output]=system(sprintf('sha256sum -- ''%s''',file));assert(status==0);
value=strtok(output);assert(~isempty(regexp(value,'^[0-9a-f]{64}$','once')));
end
function finish(mdl,oldPath)
if bdIsLoaded(mdl),close_system(mdl,0);end
path(oldPath);
end
