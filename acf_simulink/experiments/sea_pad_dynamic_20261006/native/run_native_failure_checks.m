function report=run_native_failure_checks(root)
% Deliberate out-of-envelope software probes; no controller qualification.
assert(~usejava('desktop'));maxNumCompThreads(2);
here=fullfile(root,'acf_simulink','experiments','sea_pad_dynamic_20261006','native');addpath(here);
out=fullfile(root,'acf_simulink','validation','sea_pad_dynamic_20261006','native');
contract=fullfile(fileparts(here),'study_contract.json');
temp=fullfile(out,'owned_failure_temporary');mkdir(temp);fid=fopen(fullfile(temp,'owned_sea_pad_failures'),'w');fprintf(fid,'Owned native failure checks\n');fclose(fid);
storage=Simulink.sdi.getStorageLocation;record=Simulink.sdi.getRecordData;
sdi=fullfile(temp,'sdi');mkdir(sdi);Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
modelFile=fullfile(root,'acf_simulink','generated','sea_pad_dynamic_20261006','acf_sea_pad_research_20261006.slx');
[~,mdl]=fileparts(modelFile);assert(~bdIsLoaded(mdl));
cleanup=onCleanup(@()finish(storage,record,temp,mdl)); %#ok<NASGU>
Simulink.fileGenControl('set','CacheFolder',fullfile(temp,'cache'), ...
 'CodeGenFolder',fullfile(temp,'code'),'createDir',true);
modelBefore=sha256(modelFile);load_system(modelFile);
mw=get_param(mdl,'ModelWorkspace');rows=struct([]);
for j=1:4
 options=struct('stopTime',.2,'mS',2,'gAxis',0,'Fref',5,'kPad',50000,'cPad',0,'padCompressionLimit',.0025);
 switch j
  case 1,name='whole_contact_loss';command=-3.5;
  case 2,name='configured_flange_travel_stop';command=3.5;options.toolTravelLimit=.003;
  case 3,name='configured_series_spring_working_stop';command=3.5;options.sea=1;options.Fref=20;options.springTravelLimit=.0041;
  case 4,name='configured_pad_compression_stop';command=3.5;options.padCompressionLimit=.00011;
 end
 p=sea_pad_native_params(contract,options);assignin(mw,'p',p);
 replayInputs=[0 command 0;p.stopTime command 0];assignin(mw,'replayInputs',replayInputs);assignin('base','replayInputs',replayInputs);
 set_param(mdl,'FixedStep',sprintf('%.17g',p.dt),'StopTime','p.stopTime','Decimation','1');
 result=sim(mdl);ts=result.yout{1}.Values;data=matrix(ts,24);time=ts.Time;
 assert(numel(time)>1&&time(end)<p.stopTime,'Physical stop did not halt the native recording.');
 force=data(:,10);u=data(:,1)-data(:,3);
 switch j
  case 1,flag=any(force<=0);
  case 2,flag=any(abs(u)>p.toolTravelLimit);
  case 3,flag=any(abs(data(:,7)-u)>p.springTravelLimit);
  case 4,flag=any(data(:,14)>p.padCompressionLimit);
 end
 assert(flag,'Ended recording lacks the corresponding physical failure flag.');
 row=struct('identity',name,'status','deliberate_native_failure_probe_executed', ...
  'forceQualified',false,'runMarkedFailed',true,'expectedFailureObserved',flag, ...
  'requestedEnd_s',p.stopTime,'actualEnd_s',time(end),'nativeOutputPoints',numel(time), ...
  'minimumTotalForce_N',min(force),'maximumAbsoluteFlangeTravel_m',max(abs(u)), ...
  'maximumPadCompression_m',max(data(:,14)), ...
  'maximumSeriesDeflection_m',max(abs(data(:,7)-u)), ...
  'configuredTravelLimit_m',p.toolTravelLimit,'configuredSpringLimit_m',p.springTravelLimit, ...
  'configuredPadCompressionLimit_m',p.padCompressionLimit);
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 fprintf('NATIVE_FAILURE_PROBE %s\n',jsonencode(row));clear data result ts;Simulink.sdi.clear;
end
close_system(mdl,0);modelAfter=sha256(modelFile);assert(strcmp(modelBefore,modelAfter));
report=struct('status','native_failure_stops_executed', ...
 'forceQualification',false,'cases',rows,'allExpectedStopsObserved',all([rows.expectedFailureObserved]), ...
 'savedModelUnchanged',true,'savedModelBeforeSha256',modelBefore, ...
 'savedModelAfterSha256',modelAfter);write_json(fullfile(out,'failure_checks.json'),report);
end
function finish(storage,record,temp,mdl)
try,if bdIsLoaded(mdl),close_system(mdl,0);end,catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(temp,'owned_sea_pad_failures')),try,rmdir(temp,'s');catch,end,end
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);
digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(typecast(bytes,'int8'));
h=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
function d=matrix(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end;d=reshape(d,n,width);
end
function write_json(file,value),fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);end
