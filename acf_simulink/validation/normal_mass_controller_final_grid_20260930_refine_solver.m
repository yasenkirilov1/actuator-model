function report=normal_mass_controller_final_grid_20260930_refine_solver(sourceFolder,candidateFile,worstTraceFile,outputFolder,steps)
% Final-source numerical check, held until the complete mesh worst is known.
% Two native reruns change only ode4 integration10 ->5 ->2.5us. Actual100k
% acquisition,10k control, random seeds/conditioning and all physical and
% controller parameters remain fixed. Full fine grids, not decimated scores.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
assert(isfile(worstTraceFile) && isfolder('/dev/shm'));
if nargin<4,outputFolder=fileparts(mfilename('fullpath'));end
if nargin<5,steps=[5e-6 2.5e-6];end
assert(~isempty(steps) && numel(unique(steps))==numel(steps) && all(ismember(steps,[5e-6 2.5e-6])));
stem='normal_mass_controller_final_grid_20260930_solver_refinement_v8';
if isscalar(steps),stem=[stem sprintf('_step_%gns',steps*1e9)];end
resultFile=fullfile(outputFolder,[stem '_results.json']);assert(~isfile(resultFile));
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
modelHash=sha256(sourceModel);candidateHash=sha256(candidateFile);
assert(strcmp(modelHash,'73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643'));
assert(strcmp(candidateHash,'d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7'));
reference=load(worstTraceFile,'worst','worstRow');
assert(isfield(reference,'worst'));baseline=reference.worst;baselineRow=reference.worstRow;clear reference;
p=baseline.parameters;
assert(p.sim.step==10e-6 && p.sim.stopTime==4 && p.sim.evaluateFrom==0 && ...
 p.acquisition.enabled && p.acquisition.rawSampleTime_s==10e-6 && p.force.sampleTime==100e-6);
assert(p.springs.lower.freeLength_m==.020 && p.springs.commonPreload_N==33.6 && ...
 p.coil.currentResponse==100e-6 && p.robot.forceGate.variation.enabled);
worker=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','helpers','cache','code'}
 mkdir(fullfile(worker,name{1}));
end
modelFile=fullfile(worker,'acf_force_control_normal_trial.slx');copyfile(sourceModel,modelFile);
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(worker,'helpers'));
copyfile(candidateFile,fullfile(worker,'candidate.mat'));
snapshot=fullfile(worker,[mfilename '.m']);copyfile([mfilename('fullpath') '.m'],snapshot);
addpath(fullfile(worker,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator acf_metrics;
maxNumCompThreads(2);
ownedStorage=tempname('/dev/shm');mkdir(ownedStorage);
fid=fopen(fullfile(ownedStorage,'owned_acf_solver_sdi'),'w');assert(fid~=-1);fclose(fid);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
mdl='acf_force_control_normal_trial';assert(~bdIsLoaded(mdl));
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,ownedStorage));
Simulink.sdi.setStorageLocation(ownedStorage);Simulink.sdi.setRecordData(true);
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
load_system(modelFile);set_param(mdl,'SimulationMode','accelerator');
helpers=dir(fullfile(worker,'helpers','*.m'));helperSources=struct([]);
for j=1:numel(helpers)
 one=struct('file',helpers(j).name,'sha256',sha256(fullfile(helpers(j).folder,helpers(j).name)));
 if isempty(helperSources),helperSources=one;else,helperSources(end+1)=one;end %#ok<AGROW>
end
rows=struct([]);started=tic;
report=struct('status','running','date','2026-09-30','plannedNativeReruns',numel(steps), ...
 'integrationSteps_s',steps, ...
 'scope','Integration refinement of the selected finite-grid worst only, with the actual final V8 acquisition/control unchanged. Numerical convergence evidence, not an uncertainty or hardware qualification.', ...
 'modelSha256',modelHash,'candidateSha256',candidateHash,'sourceModel',sourceModel, ...
 'privateModel',modelFile,'helperSources',helperSources,'experimentSnapshot',snapshot, ...
 'experimentSha256',sha256(snapshot),'baselineTrace',worstTraceFile, ...
 'baselineTraceSha256',sha256(worstTraceFile),'baselineRow',baselineRow, ...
 'baselineParameters',p,'matlabVersion',version,'rows',rows);
for step=steps
 trace=fullfile(outputFolder,sprintf('%s_step_%gns_native.mat',stem,step*1e9));
 assert(~isfile(trace),'Preserve an existing returned native refinement trace.');
 q=p;q.sim.step=step;q=acf_prepare(q);
 assert(isequaln(q.acquisition,p.acquisition) && q.force.sampleTime==p.force.sampleTime && ...
  q.sensor.sampleTime==p.sensor.sampleTime && q.sensor.seed==p.sensor.seed && ...
  q.sensor.noiseRms==p.sensor.noiseRms && q.accel.noiseRms==p.accel.noiseRms);
 set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');timer=tic;
 r=struct;quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
 save(trace,'r','-v7'); % Preserve any returned native history before the audit.
 assert(numel(r.time)==round(4/step)+1 && r.time(end)==4);
 ratio=round(p.sim.step/step);indices=1:ratio:numel(r.time);
 assert(isequal(r.time(indices),baseline.time));
 fields=fieldnames(baseline);differences=struct;channels=size(baseline.data,2);
 differences.originalPhysicalAndControl=max(abs(r.data(indices,:)-baseline.data),[],'all');
 for j=1:numel(fields)
  name=fields{j};old=baseline.(name);
  if isstruct(old) && isfield(old,'time') && isfield(old,'data')
   now=r.(name);assert(isequal(now.time,r.time) && size(now.data,2)==size(old.data,2));
   differences.(name)=max(abs(now.data(indices,:)-old.data),[],'all');
   channels=channels+size(old.data,2);
  end
 end
 gateNames={'ready','filteredError_N','forceRate_N_s','stableNow'};
 assert(isequal(r.robotGate.time,baseline.robotGate.time));
 for j=1:numel(gateNames)
  name=gateNames{j};old=baseline.robotGate.(name);now=r.robotGate.(name);
  assert(numel(now)==numel(r.robotGate.time) && size(old,2)==1);
  differences.robotGate.(name)=max(abs(now-old));
 end
 channels=channels+numel(gateNames);
 assert(channels==72,'The final native logger contract has72 physical/control/measurement channels.');
 normal=r.normalContact.data(:,1);error=normal-q.force.reference;
 [peak,which]=max(abs(error));
 [packed,low,high,totalFree,totalInstalled]=spring_packing(r);
 row=struct('integrationStep_s',step,'nativeSamples',numel(r.time), ...
  'runtime_s',toc(timer),'peakNormalForceError_N',peak,'normalPeakTime_s',r.time(which), ...
  'peakScoreDifferenceFrom10us_N',peak-baselineRow.peakNormalForceError_N, ...
  'rmsNormalForceError_N',sqrt(mean(error.^2)), ...
  'peakNormalEstimationError_N',max(abs(r.normalContact.data(:,4)-normal)), ...
  'peakCurrent_A',max(abs(r.data(:,14))),'peakVoltage_V',max(abs(r.data(:,15))), ...
  'voltageClipFraction',mean(r.data(:,23)>0),'currentClipFraction',mean(r.data(:,22)>0), ...
  'lostContact',r.metrics.lostContact,'strokeStop',r.metrics.strokeStopHit, ...
  'springPackingPassed',packed,'minimumCommonSpringPreload_N',low, ...
  'maximumCommonSpringPreload_N',high,'combinedFreeSpringLength_mm',totalFree, ...
  'combinedInstalledSpringLength_mm',totalInstalled,'coarseGridChannelDifferences',differences, ...
  'nativeChannelsCompared',channels,'passed',peak<=.5 && packed && ...
   ~r.metrics.lostContact && ~r.metrics.strokeStopHit);
 save(trace,'row','-append');row.nativeTrace=trace;row.nativeTraceSha256=sha256(trace);
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 report.rows=rows;report.executedNativeReruns=numel(rows);report.elapsedSeconds=toc(started);
 write_report(resultFile,report);
 fprintf('FINAL_SOLVER_REFINEMENT %.3gus samples%d peak%.12gN delta%.9gN pass%d\n', ...
  step*1e6,row.nativeSamples,peak,row.peakScoreDifferenceFrom10us_N,row.passed);
 clear r;Simulink.sdi.clear;
end
report.status='complete';report.sourceModelUnchanged=strcmp(sha256(sourceModel),modelHash);
report.privateModelUnchanged=strcmp(sha256(modelFile),modelHash);
assert(report.sourceModelUnchanged && report.privateModelUnchanged);
report.elapsedSeconds=toc(started);write_report(resultFile,report);
save(fullfile(outputFolder,[stem '_results.mat']),'report','-v7');
end

function [passed,low,high,totalFree,totalInstalled]=spring_packing(r)
p=r.parameters;d=r.data;pack=p.springs;low=0;high=inf;totalFree=0;
links={'lower','upper'};rigid=[p.mechanics.lowerRigid,p.mechanics.upperRigid];
delta={d(:,7)-d(:,6),d(:,5)-d(:,26)};offset=[p.d.q10,p.d.q20];
stiffness=[p.mechanics.k1,p.mechanics.k2];excursionPass=true;
for j=1:2
 if rigid(j),continue;end
 spring=pack.(links{j});rate=spring.rate_N_m;
 shift=offset(j)/stiffness(j)+delta{j};largest=max(abs(shift));
 low=max(low,rate*(pack.unseatingClearance_m+largest));
 compression=min(spring.freeLength_m-spring.freeLengthTolerance_m- ...
  spring.minWorkingLength_m-pack.workingClearance_m,spring.forceEnvelope_N/rate);
 high=min(high,rate*(compression-largest));
 excursionPass=excursionPass && max(shift)-min(shift)<=spring.maxCyclicStroke_m+1e-12;
 totalFree=totalFree+2000*(spring.freeLength_m+spring.freeLengthTolerance_m);
end
common=pack.commonPreload_N;totalInstalled=0;
for j=1:2
 if rigid(j),continue;end
 spring=pack.(links{j});
 totalInstalled=totalInstalled+2000*(spring.freeLength_m+spring.freeLengthTolerance_m-common/spring.rate_N_m);
end
passed=low<=high && excursionPass && common>=low-1e-12 && common<=high+1e-12 && ...
 totalFree<=1000*pack.maxCombinedLength_m+1e-9 && totalInstalled<=1000*pack.maxCombinedLength_m+1e-9;
end

function write_report(file,report)
fid=fopen(file,'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
end

function result=sha256(file)
single=char(39);escaped=strrep(file,single,[single,char(34),single,char(34),single]);
[status,output]=system(['sha256sum -- ',single,escaped,single]);assert(status==0);
result=regexp(output,'[0-9a-f]{64}','match','once');assert(~isempty(result));
end

function finish(mdl,oldStorage,oldRecord,ownedStorage)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldStorage);Simulink.sdi.setRecordData(oldRecord);
assert(startsWith(ownedStorage,'/dev/shm/') && isfile(fullfile(ownedStorage,'owned_acf_solver_sdi')));
rmdir(ownedStorage,'s');
end
