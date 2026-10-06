function report=benchmark_inertial_tmpfs_storage(sourceFolder,candidateFile,referenceFile,outputFolder)
%BENCHMARK_INERTIAL_TMPFS_STORAGE Paired transient-storage proof only.
% The frozen native model, candidate, helpers, solver and control are fixed.
% Reference is a retained heavy, noisy10s native recording. Four executions
% repeat that exact condition: disk cold/warm, tmpfs cold/warm. Active mesh
% workers and their files are never touched.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
assert(isfolder('/dev/shm') && isfile(referenceFile));
stem='normal_mass_controller_tmpfs_sdi_pilot_20260930_v8';
if ~isfolder(outputFolder),mkdir(outputFolder);end
resultFile=fullfile(outputFolder,[stem '_results.json']);assert(~isfile(resultFile));
worker=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for folder={'','helpers','cache','code','sdi'}
 path=fullfile(worker,folder{1});mkdir(path);
end
tmpfsFolder=tempname('/dev/shm');mkdir(tmpfsFolder);
marker=fullfile(tmpfsFolder,'owned_acf_sdi_storage');
fid=fopen(marker,'w');assert(fid~=-1);fprintf(fid,'Private SDI-only pilot; owned by this isolated batch.\n');fclose(fid);
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
modelHash=sha256(sourceModel);candidateHash=sha256(candidateFile);
assert(strcmp(modelHash,'73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643'));
assert(strcmp(candidateHash,'d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7'));
modelFile=fullfile(worker,'acf_force_control_normal_trial.slx');copyfile(sourceModel,modelFile);
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(worker,'helpers'));
copyfile(candidateFile,fullfile(worker,'candidate.mat'));
assert(strcmp(candidateHash,sha256(fullfile(worker,'candidate.mat'))));
snapshot=fullfile(worker,'benchmark_inertial_tmpfs_storage.m');copyfile([mfilename('fullpath') '.m'],snapshot);
addpath(fullfile(worker,'helpers'),'-begin');
clear acf_prepare acf_run acf_select_actuator acf_metrics;
maxNumCompThreads(2);
saved=load(fullfile(worker,'candidate.mat'),'p');p=saved.p;
reference=load(referenceFile,'r');expected=reference.r;clear reference;
rp=expected.parameters;
assert(rp.coil.currentResponse==100e-6 && rp.coil.voltage==24 && rp.sensor.seed==7);
assert(rp.mechanics.mS==2 && rp.force.reference==50 && rp.contact.mu==.3);
assert(rp.sim.stopTime==10 && strcmp(rp.surface.kind,'repeated'));
% Only match the retained condition; the final20mm packing metadata stays.
p.mechanics.mS=rp.mechanics.mS;p.mechanics.angle=rp.mechanics.angle;
p.sensor.distalMass_kg=p.mechanics.mS;p.sensor.distalMass_model_kg=p.mechanics.mS;
p.force.reference=rp.force.reference;p.contact.mu=rp.contact.mu;p.sensor.seed=rp.sensor.seed;
p.sim=rp.sim;p.surface=rp.surface;p.coil.strategy='pi';p=acf_prepare(p);
assert(p.springs.lower.freeLength_m==.020 && p.springs.commonPreload_N==33.6);
assert(p.acquisition.enabled && p.robot.forceGate.variation.enabled);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
mdl='acf_force_control_normal_trial';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,tmpfsFolder,marker)); %#ok<NASGU>
set_param(mdl,'SimulationMode','accelerator');Simulink.sdi.setRecordData(true);
helperFiles=dir(fullfile(worker,'helpers','*.m'));helperSources=struct([]);
for j=1:numel(helperFiles)
 one=struct('file',helperFiles(j).name,'sha256',sha256(fullfile(helperFiles(j).folder,helperFiles(j).name)));
 if isempty(helperSources),helperSources=one;else,helperSources(end+1)=one;end %#ok<AGROW>
end
rows=struct([]);started=tic;
report=struct('status','running','date','2026-09-30', ...
 'scope','Execution-path benchmark only: same heavy noisy10s reference, unchanged native plant/control/actual acquisition, temporary SDI disk versus tmpfs. Wall timing measured while four independent mesh workers continue.', ...
 'modelSha256',modelHash,'candidateSha256',candidateHash,'referenceFile',referenceFile, ...
 'referenceSha256',sha256(referenceFile),'privateModel',modelFile,'helperSources',helperSources, ...
 'experimentSnapshot',snapshot,'experimentSha256',sha256(snapshot),'matlabVersion',version, ...
 'baseParameters',p,'diskStorage',fullfile(worker,'sdi'),'tmpfsStorage',tmpfsFolder, ...
 'plannedNativeExecutions',4,'rows',rows);
for backend=1:2
 if backend==1,storage=fullfile(worker,'sdi');label='disk';else,storage=tmpfsFolder;label='tmpfs';end
 set_param(mdl,'FastRestart','off');Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);
 set_param(mdl,'FastRestart','on');
 for repetition=1:2
  before=process_io;timer=tic;r=[];failureText='';
  try
   quiet=evalc('r=acf_run(mdl,p,false);'); %#ok<NASGU>
   nativeRuntime=toc(timer);after=process_io;
   parity=compare_manual(expected,r);
   assert(parity.exact && numel(r.time)==1000001,'All full native histories must match exactly.');
   peak=max(abs(r.normalContact.data(:,1)-50));
   assert(r.time(end)==10 && peak<=.5 && ~r.metrics.lostContact && ~r.metrics.strokeStopHit);
  catch failure
   failureText=getReport(failure,'extended','hyperlinks','off');
   if ~exist('nativeRuntime','var'),nativeRuntime=toc(timer);end
   after=process_io;parity=struct('exact',false,'maximumDifference',NaN,'channels',struct);
   peak=NaN;
  end
  files=dir(fullfile(storage,'**','*'));storageBytes=sum([files(~[files.isdir]).bytes]);
  row=struct('backend',label,'repetition',repetition,'cold',repetition==1, ...
   'nativeSimulationAndMetrics_s',nativeRuntime,'runtimeIncludingParity_s',toc(timer), ...
   'SDILogicalBytes',storageBytes,'diskReadBytes',after.read_bytes-before.read_bytes, ...
   'diskWriteBytes',after.write_bytes-before.write_bytes,'exactFullNativeParity',parity.exact, ...
   'maximumManualHistoryDifference',parity.maximumDifference,'channelDifferences',parity.channels, ...
   'peakNormalForceError_N',peak,'reason',failureText);
  if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
  report.rows=rows;report.executedNativeComparisons=numel(rows);report.elapsedSeconds=toc(started);
  write_report(resultFile,report);
  fprintf('SDI_STORAGE_PILOT %s repetition%d native%.4fs parity%d SDI%.1fMiB read%.1fMiB write%.1fMiB\n', ...
   label,repetition,nativeRuntime,parity.exact,storageBytes/2^20,row.diskReadBytes/2^20,row.diskWriteBytes/2^20);
  if ~isempty(failureText),error('acf:tmpfsPilot:Failed','%s',failureText);end
  if backend==2 && repetition==1
   save(fullfile(outputFolder,[stem '_tmpfs_native_trace.mat']),'r','row','-v7');
  end
  clear r nativeRuntime;Simulink.sdi.clear;
 end
end
diskWarm=rows(2).nativeSimulationAndMetrics_s;tmpfsWarm=rows(4).nativeSimulationAndMetrics_s;
report.warmNativeSpeedup=diskWarm/tmpfsWarm;
report.allFourFullNativeHistoriesBitwiseIdentical=all([rows.exactFullNativeParity]);
report.sourceModelUnchanged=strcmp(modelHash,sha256(sourceModel));
report.privateModelUnchanged=strcmp(modelHash,sha256(modelFile));
assert(report.sourceModelUnchanged && report.privateModelUnchanged);
report.status='complete';report.elapsedSeconds=toc(started);write_report(resultFile,report);
save(fullfile(outputFolder,[stem '_results.mat']),'report','-v7');
end

function parity=compare_manual(a,b)
names={'time','data'};allNames=union(fieldnames(a),fieldnames(b),'stable');
for j=1:numel(allNames)
 name=allNames{j};if isfield(a,name),sample=a.(name);else,sample=b.(name);end
 if isstruct(sample) && isfield(sample,'time'),names{end+1}=name;end %#ok<AGROW>
end
exact=true;maximum=0;channels=struct;
for j=1:numel(names)
 name=names{j};assert(isfield(a,name) && isfield(b,name));x=a.(name);y=b.(name);peak=0;
 if isstruct(x)
  fields=fieldnames(x);assert(isequal(fields,fieldnames(y)));
  for k=1:numel(fields)
   one=fields{k};assert(isequal(size(x.(one)),size(y.(one))));
   exact=exact && isequaln(x.(one),y.(one));
   if isnumeric(x.(one)) || islogical(x.(one))
    delta=abs(double(x.(one)(:))-double(y.(one)(:)));if ~isempty(delta),peak=max(peak,max(delta));end
   end
  end
 else
  assert(isequal(size(x),size(y)));exact=exact && isequaln(x,y);
  delta=abs(double(x(:))-double(y(:)));if ~isempty(delta),peak=max(delta);end
 end
 channels.(name)=peak;maximum=max(maximum,peak);
end
parity=struct('exact',exact,'maximumDifference',maximum,'channels',channels);
end

function io=process_io
contents=fileread('/proc/self/io');io=struct;
for name={'read_bytes','write_bytes'}
 token=regexp(contents,[name{1} ':\s+(\d+)'],'tokens','once');io.(name{1})=str2double(token{1});
end
end

function write_report(file,report)
fid=fopen(file,'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
end

function value=sha256(file)
single=char(39);escaped=strrep(file,single,[single,char(34),single,char(34),single]);
[status,output]=system(['sha256sum -- ',single,escaped,single]);assert(status==0);
value=regexp(output,'[0-9a-f]{64}','match','once');assert(~isempty(value));
end

function finish(mdl,previousStorage,previousRecord,tmpfsFolder,marker)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(previousStorage);Simulink.sdi.setRecordData(previousRecord);
assert(startsWith(tmpfsFolder,'/dev/shm/') && isfile(marker),'Delete only this process-owned temporary SDI directory.');
rmdir(tmpfsFolder,'s');
end
