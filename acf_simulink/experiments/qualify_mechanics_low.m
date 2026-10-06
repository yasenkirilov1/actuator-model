function report = qualify_mechanics_low(stateRoot,outputFolder)
%QUALIFY_MECHANICS_LOW Full-range native Simulink study of softer spring pairs.
% This experiment uses an immutable SLX/helpers snapshot and never saves it.
% A preload changes only the opposed-pair geometry screen, not plant physics.
root=fileparts(fileparts(mfilename('fullpath')));
if nargin<1,stateRoot='/home/tommie/.local/state/actuator-model/qualification-20260930';end
if nargin<2,outputFolder=fullfile(root,'validation');end
if ~isfolder(outputFolder),mkdir(outputFolder);end
owned=fullfile(stateRoot,'mechanics-low-worker');
if ~isfolder(owned),mkdir(owned);end
addpath(fullfile(stateRoot,'helpers'),'-begin');
if isfolder(fullfile(stateRoot,'helpers-v2'))
 addpath(fullfile(stateRoot,'helpers-v2'),'-begin');
 clear acf_prepare;
end
maxNumCompThreads(1);
Simulink.fileGenControl('set','CacheFolder',fullfile(owned,'cache'), ...
    'CodeGenFolder',fullfile(owned,'code'),'createDir',true);
storage=fullfile(owned,'sdi');if ~isfolder(storage),mkdir(storage);end
Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(true);
modelFile=fullfile(stateRoot,'acf_force_control_simple.slx');
assert(~bdIsLoaded('acf_force_control_simple'), ...
 'Run this snapshot study in a fresh MATLAB -batch session; it owns that model and session storage.');
load_system(modelFile);mdl='acf_force_control_simple';
cleanup=onCleanup(@() finish_owned_model(mdl)); %#ok<NASGU>
base=load(fullfile(stateRoot,'base_parameters.mat'));p=base.p;
catalog=jsondecode(fileread(fullfile(stateRoot,'spring_catalog.json')));
p.actuator.kind='voice_coil';p.mechanics.lowerRigid=false;p.mechanics.upperRigid=false;
p.robot.forceGate.enabled=true;p.mechanics.mB=.6;p.mechanics.mS=1.4;
p.surface.kind='sharp_bump';p.surface.height=.001;p.surface.width=.010;p.surface.scanSpeed=.100;
p.sim.stopTime=4;p.sim.evaluateFrom=0;p.sensor.noiseRms=0;p.sensor.seed=1729;
damping=[0 5 15 30 60];preloads=[10 20 28 30 32];
corners=[5 0;5 90;50 0;50 90];configs=struct([]);
for lower=1:2
 for upper=1:4
  for c1=damping
   for c2=damping
    x=struct('id',numel(configs)+1,'lowerIndex',lower,'upperIndex',upper, ...
      'lowerPart',catalog.springs(lower).part,'upperPart',catalog.springs(upper).part, ...
      'k1_N_m',2*catalog.springs(lower).rate_N_m,'k2_N_m',2*catalog.springs(upper).rate_N_m, ...
      'c1_Ns_m',c1,'c2_Ns_m',c2);
    if isempty(configs),configs=x;else,configs(end+1)=x;end %#ok<AGROW>
   end
  end
 end
end
checkpointFile=fullfile(outputFolder,'full_range_mechanics_low_checkpoint.mat');
rows=struct([]);screens={};done=false(numel(configs),size(corners,1));elapsedBefore=0;
originalSynthesisFailures=struct([]);
executionInterruption=struct([]);
if isfile(checkpointFile)
 try
  checkpoint=load(checkpointFile);
 catch err
  progressFile=fullfile(outputFolder,'full_range_mechanics_low_progress.json');
  recovered=jsondecode(fileread(progressFile));
  copyfile(checkpointFile,fullfile(owned,'interrupted_checkpoint.mat'));
  copyfile(progressFile,fullfile(owned,'recovered_progress_rows.json'));
  recoveredRows=recovered.rows(:)';recoveredDone=false(size(done));recoveredScreens=cell(1,numel(recoveredRows));
  for j=1:numel(recoveredRows)
   id=recoveredRows(j).configId;corner=recoveredRows(j).cornerId;
   assert(id>=1 && id<=numel(configs) && corner>=1 && corner<=size(corners,1));
   assert(~recoveredDone(id,corner),'Duplicate recovered case.');
   assert(recoveredRows(j).k1_N_m==configs(id).k1_N_m && recoveredRows(j).k2_N_m==configs(id).k2_N_m && ...
    recoveredRows(j).c1_Ns_m==configs(id).c1_Ns_m && recoveredRows(j).c2_Ns_m==configs(id).c2_Ns_m && ...
    recoveredRows(j).force_N==corners(corner,1) && abs(recoveredRows(j).orientation_deg-corners(corner,2))<1e-9, ...
    'Recovered progress does not match the requested grid.');
   recoveredRows(j).geometryPassed=logical(recoveredRows(j).geometryPassed(:)');
   recoveredRows(j).geometryReason=reshape(cellstr(string(recoveredRows(j).geometryReason)),1,[]);
   recoveredDone(recoveredRows(j).configId,recoveredRows(j).cornerId)=true;
   one=cell(1,numel(preloads));
   for k=1:numel(preloads)
    one{k}=struct('commonPreload_N',preloads(k),'passed',recoveredRows(j).geometryPassed(k), ...
     'reasons',{{recoveredRows(j).geometryReason{k}}},'sizing',[], ...
     'detailUnavailable',true,'source','Recovered intact progress JSON; exact sizing detail was lost in interrupted MAT write.');
   end
   recoveredScreens{j}=one;
  end
  failures=struct([]);
  for corner=1:2
   for id=[1 101 126]
    x=struct('configId',id,'cornerId',corner,'force_N',5,'orientation_deg',corners(corner,2), ...
     'exceptionMessage','Coil observer covariance did not converge.', ...
     'evidence','Captured private batch console before helpers-v2 migration; exact original exception row was in the interrupted checkpoint.');
    if isempty(failures),failures=x;else,failures(end+1)=x;end %#ok<AGROW>
   end
  end
  executionInterruption=struct('exitStatus',143,'signal','SIGTERM','cause','undetermined external execution interruption', ...
   'recoveredRows',numel(recoveredRows),'recoveredSource',fullfile(owned,'recovered_progress_rows.json'), ...
   'interruptedCheckpoint',fullfile(owned,'interrupted_checkpoint.mat'), ...
   'geometryRecovery','Actual preload pass flags and rejection reasons preserved; detailed sizing structs unavailable for recovered rows.', ...
   'loadError',err.message);
  checkpoint=struct('configs',configs,'corners',corners,'rows',recoveredRows,'screens',{recoveredScreens}, ...
   'done',recoveredDone,'elapsedSeconds',recovered.elapsedSeconds,'originalSynthesisFailures',failures, ...
   'executionInterruption',executionInterruption);
  fprintf('MECHANICS_LOW_RECOVERY: recovered %d actual rows from intact JSON; detailed earlier sizing structs unavailable.\n',numel(recoveredRows));
 end
 assert(isequal(checkpoint.configs,configs) && isequal(checkpoint.corners,corners), ...
   'Checkpoint belongs to a different experiment grid.');
 rows=checkpoint.rows;screens=checkpoint.screens;done=checkpoint.done;elapsedBefore=checkpoint.elapsedSeconds;
 if isfield(checkpoint,'originalSynthesisFailures'),originalSynthesisFailures=checkpoint.originalSynthesisFailures;end
 if isfield(checkpoint,'executionInterruption'),executionInterruption=checkpoint.executionInterruption;end
 if ~isfield(rows,'synthesisHelperPath')
  for j=1:numel(rows),rows(j).synthesisHelperPath=fullfile(stateRoot,'helpers','acf_prepare.m');end
 end
 if contains(which('acf_prepare'),'helpers-v2')
  failed=find(~[rows.executed]);
  if ~isempty(failed)
   if isempty(originalSynthesisFailures),originalSynthesisFailures=rows(failed);
   else,originalSynthesisFailures=[originalSynthesisFailures,rows(failed)];end
   for j=failed,done(rows(j).configId,rows(j).cornerId)=false;end
   rows(failed)=[];screens(failed)=[];
   fprintf('MECHANICS_LOW_RETRY: %d earlier synthesis failures queued with revised equation-equivalent solver.\n',numel(failed));
  end
 end
end
started=tic;delayBlock=[mdl '/Force measurement/Sensor sample delay'];
set_param(mdl,'FastRestart','off');set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples');
for corner=1:size(corners,1)
 set_param(mdl,'FastRestart','on');
 for id=1:numel(configs)
  if done(id,corner),continue;end
  q=configure(p,configs(id),corners(corner,:),catalog);
  [row,geometry]=execute(mdl,q,configs(id),corner,preloads,false);
  if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
  screens{end+1}=geometry;done(id,corner)=true; %#ok<AGROW>
  count=sum(done,'all');
  if mod(count,25)==0 || id==numel(configs)
   elapsedSeconds=elapsedBefore+toc(started); %#ok<NASGU>
   pending=[checkpointFile '.pending.mat'];
   save(pending,'rows','screens','done','configs','corners','preloads','elapsedSeconds','originalSynthesisFailures','executionInterruption','-v7');
   [ok,message]=movefile(pending,checkpointFile,'f');assert(ok,message);
   progress=struct('completedNativeSimulations',count,'totalNativeSimulations',numel(done), ...
     'elapsedSeconds',elapsedSeconds,'lastForce_N',corners(corner,1), ...
     'lastOrientation_deg',corners(corner,2),'rows',rows);
   write_json(fullfile(outputFolder,'full_range_mechanics_low_progress.json'),progress);
   fprintf('MECHANICS_LOW %d/%d nominal simulations, %.1f s, corner %g N / %g deg.\n', ...
     count,numel(done),elapsedSeconds,corners(corner,1),corners(corner,2));
   Simulink.sdi.clear;
  end
 end
 set_param(mdl,'FastRestart','off');
 cornerRows=rows([rows.cornerId]==corner);
 fprintf('MECHANICS_LOW_CORNER %g N / %g deg: numerical %d/%d, geometry %d/%d.\n', ...
   corners(corner,1),corners(corner,2),sum([cornerRows.physicalPassed]),numel(cornerRows), ...
   sum(arrayfun(@(a)any(a.geometryPassed),cornerRows)),numel(cornerRows));
end
summary=summarize(configs,rows,preloads);
feasible=find([summary.fullRangePassed]);
if isempty(feasible)
 pool=find([summary.physicalFullRangePassed]);
 if isempty(pool),pool=find([summary.executedCorners]==size(corners,1));end
 selectionBasis='No low-stiffness configuration passed physical and spring geometry requirements at every corner; stress the numerically best configurations as diagnostic evidence.';
else
 pool=feasible;selectionBasis='Smallest nominal worst-corner error among configurations passing physical and geometry requirements at one common preload across all four corners.';
end
if isempty(pool),chosen=[];
else,[~,order]=sort([summary(pool).worstError_N]);chosen=pool(order(1:min(8,numel(order))));end
stressChosen=chosen;
stressDescription='0.02 N RMS force noise seed 7 plus an extra unmodeled 0.2 ms force measurement delay; nominal stroke noise 1 micrometer RMS and calibrated 0.2 ms stroke delay.';
if isempty(feasible)
 stressChosen=[];
 stressDescription='Not executed: all low-rate designs fail the nominal four-corner spring geometry screen, so extra stress qualification would not select a realizable candidate.';
 selectionBasis='No low-stiffness configuration passes both physics and spring geometry at every corner. Preserve numerically best ordinary histories as diagnostic evidence; no extra stress qualification is needed for rejected geometry.';
end
write_json(fullfile(outputFolder,'full_range_mechanics_low_nominal_summary.json'), ...
 struct('configurations',summary,'nominalRows',rows,'selectionBasis',selectionBasis,'stressConfigIds',stressChosen));
fprintf('MECHANICS_LOW_NOMINAL_COMPLETE: %d/%d full-range feasible; stress ids %s.\n', ...
 numel(feasible),numel(configs),mat2str(stressChosen));
stressRows=struct([]);stressScreens={};selectedRuns={};
set_param(mdl,'FastRestart','off');set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');
for corner=1:size(corners,1)
 set_param(mdl,'FastRestart','on');
 for id=stressChosen
  q=configure(p,configs(id),corners(corner,:),catalog);q.sensor.noiseRms=.02;q.sensor.seed=7;
  [row,geometry,r]=execute(mdl,q,configs(id),corner,preloads,true);
  if isempty(stressRows),stressRows=row;else,stressRows(end+1)=row;end %#ok<AGROW>
  stressScreens{end+1}=geometry; %#ok<AGROW>
  if ~isempty(r),selectedRuns{end+1}=r;end %#ok<AGROW>
  fprintf('MECHANICS_LOW_STRESS #%d / corner%d: error %.6g N, physical %d.\n', ...
    id,corner,row.peakAbsForceError_N,row.physicalPassed);
  Simulink.sdi.clear;
 end
 set_param(mdl,'FastRestart','off');
end
stressSummary=summarize(configs(stressChosen),stressRows,preloads);
% Retain all four ordinary clean histories for the best diagnostic candidate.
ordinaryMatch=true;
if ~isempty(chosen)
 set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples');id=chosen(1);
 for corner=1:size(corners,1)
  q=configure(p,configs(id),corners(corner,:),catalog);
  [row,~,r]=execute(mdl,q,configs(id),corner,preloads,false);
  old=rows([rows.configId]==id & [rows.cornerId]==corner);
  ordinaryMatch=ordinaryMatch && row.executed==old.executed;
  if row.executed
   ordinaryMatch=ordinaryMatch && abs(row.peakAbsForceError_N-old.peakAbsForceError_N)<1e-8;
   selectedRuns{end+1}=r; %#ok<AGROW>
  end
  Simulink.sdi.clear;
 end
end
report=struct('date','2026-09-30','matlabVersion',version,'study','Lower-stiffness compliant pairs', ...
 'sourceModel',modelFile,'sourceHelpers',fullfile(stateRoot,'helpers'), ...
 'sourcePreparationHelper',which('acf_prepare'),'originalSynthesisFailures',originalSynthesisFailures, ...
 'solverMigrationEvidence',fullfile(outputFolder,'riccati_solver_regression_results.json'), ...
 'executionInterruption',executionInterruption, ...
 'parameters',p,'catalogue',catalog,'springLowerIndices',[1 2],'springUpperIndices',1:4, ...
 'dampingValues_Ns_m',damping,'preloadValues_N',preloads,'forceOrientationCorners',corners, ...
 'configurations',numel(configs),'nominalSimulations',numel(rows),'stressSimulations',numel(stressRows), ...
 'ordinaryVerificationSimulations',4*double(~isempty(chosen)), ...
 'fullRangeNominalPassedConfigurations',numel(feasible),'summary',summary,'rows',rows, ...
 'geometryScreens',{screens},'stressConfigIds',stressChosen,'diagnosticNominalConfigIds',chosen,'stressSummary',stressSummary, ...
 'stressRows',stressRows,'stressGeometryScreens',{stressScreens}, ...
 'selectionBasis',selectionBasis,'selectedOrdinaryExecutionMatch',ordinaryMatch, ...
 'ordinaryComparisonTolerance_N',1e-8, ...
 'controllerPolicy','Fixed observer/state-feedback design weights, coefficients recalculated from each mechanical configuration. Force feedback and magnet stroke only; no surface preview.', ...
 'geometryPolicy','Same physical histories post-screened at common preload 10, 20, 28, 30, and 32 N per spring. A single preload must satisfy all four corners. Includes catalogue free-length tolerance, 0.25 mm unseating clearance, and 0.5 mm working-length clearance.', ...
 'stressTest',stressDescription, ...
 'scope','Four force/orientation corners are tested. Corner results alone do not establish continuous-domain robustness; intermediate force/orientation tests and physical validation are separate.', ...
 'elapsedSeconds',elapsedBefore+toc(started));
write_json(fullfile(outputFolder,'full_range_mechanics_low_results.json'),report);
writetable(rows_table(rows),fullfile(outputFolder,'full_range_mechanics_low_results.csv'));
if ~isempty(stressRows),writetable(rows_table(stressRows),fullfile(outputFolder,'full_range_mechanics_low_stress.csv'));end
save(fullfile(outputFolder,'full_range_mechanics_low_selected_runs.mat'),'selectedRuns','report','-v7');
fprintf('MECHANICS_LOW_COMPLETE: nominal full-range %d/%d, elapsed %.1f s, ordinary match %d.\n', ...
 numel(feasible),numel(configs),report.elapsedSeconds,ordinaryMatch);
end

function q=configure(p,c,corner,catalog)
q=p;q.force.reference=corner(1);q.mechanics.angle=corner(2)*pi/180;
q.mechanics.k1=c.k1_N_m;q.mechanics.k2=c.k2_N_m;q.mechanics.c1=c.c1_Ns_m;q.mechanics.c2=c.c2_Ns_m;
q.springs.lower=catalog.springs(c.lowerIndex);q.springs.upper=catalog.springs(c.upperIndex);
q.springs.commonPreload_N=10;
end

function [row,geometry,r]=execute(mdl,p,c,corner,preloads,isStress)
row=struct('configId',c.id,'cornerId',corner,'force_N',p.force.reference, ...
 'orientation_deg',p.mechanics.angle*180/pi,'lowerPart',c.lowerPart,'upperPart',c.upperPart, ...
 'k1_N_m',c.k1_N_m,'k2_N_m',c.k2_N_m,'c1_Ns_m',c.c1_Ns_m,'c2_Ns_m',c.c2_Ns_m, ...
 'stress',isStress,'executed',false,'physicalPassed',false,'physicalReason','not executed', ...
 'exceptionIdentifier','','exceptionMessage','','peakAbsForceError_N',NaN,'rmsForceError_N',NaN, ...
 'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN,'peakAxialForce_N',NaN, ...
 'minTravel_mm',NaN,'maxTravel_mm',NaN,'peakGapAcceleration_m_s2',NaN,'completed',false, ...
 'lostContact',false,'strokeStopHit',false,'overCurrent',false,'effortSaturationFraction',NaN, ...
 'voltageSaturationFraction',NaN,'endTime_s',NaN,'geometryPassed',false(size(preloads)), ...
 'geometryReason',{{}},'freeLengthMax_mm',NaN,'installedLengthMax_mm',NaN, ...
 'peakDamperForce_N',NaN,'synthesisHelperPath',which('acf_prepare'));
geometry=cell(1,numel(preloads));r=[];
pack=p.springs;physical=rmfield(p,'springs');
try
 quiet=evalc('r=acf_run(mdl,physical,false);'); %#ok<NASGU>
 m=r.metrics;row.executed=true;row.physicalPassed=m.passed;row.physicalReason=m.reason;
 fields={'peakAbsForceError_N','rmsForceError_N','peakCurrent_A','rmsCurrent_A','peakVoltage_V', ...
   'peakAxialForce_N','minTravel_mm','maxTravel_mm','peakGapAcceleration_m_s2','completed', ...
   'lostContact','strokeStopHit','overCurrent','effortSaturationFraction','voltageSaturationFraction'};
 for k=1:numel(fields),row.(fields{k})=m.(fields{k});end
 row.endTime_s=r.time(end);
 for j=1:numel(preloads)
  r.parameters.springs=pack;r.parameters.springs.commonPreload_N=preloads(j);
  checked=acf_metrics(r);s=checked.springSizing;
  reasons=geometry_reasons(s,pack);
  geometry{j}=struct('commonPreload_N',preloads(j),'passed',s.passed,'reasons',{reasons},'sizing',s);
  row.geometryPassed(j)=s.passed;row.geometryReason{j}=strjoin(reasons,'; ');
 end
 row.freeLengthMax_mm=geometry{1}.sizing.combinedFreeLengthMax_mm;
 row.installedLengthMax_mm=max(cellfun(@(x)x.sizing.combinedInstalledLengthMax_mm,geometry));
 row.peakDamperForce_N=max(geometry{1}.sizing.lower.peakDamperForce_N,geometry{1}.sizing.upper.peakDamperForce_N);
 r.parameters.springs=pack;r.metrics=acf_metrics(r);
 r.label=sprintf('Low-spring candidate #%d / %g N / %g deg / %s',c.id,p.force.reference,p.mechanics.angle*180/pi,ternary(isStress,'noise + delay','clean'));
catch err
 row.exceptionIdentifier=err.identifier;row.exceptionMessage=err.message;
 row.physicalReason='preparation or simulation exception';
 row.geometryReason=repmat({'not screened: no complete logged result'},size(preloads));
 warning('qualify_mechanics_low:CaseException','Case #%d corner %d: %s',c.id,corner,err.message);
end
end

function reasons=geometry_reasons(s,pack)
reasons={};links={'lower','upper'};
for j=1:2
 x=s.(links{j});part=pack.(links{j});
 if x.minCompression_mm<1000*pack.unseatingClearance_m,reasons{end+1}=[links{j} ': spring unseats'];end %#ok<AGROW>
 if x.minIndividualLength_mm<1000*(part.minWorkingLength_m+pack.workingClearance_m),reasons{end+1}=[links{j} ': below minimum working length'];end %#ok<AGROW>
 if x.cyclicExcursion_mm>1000*part.maxCyclicStroke_m,reasons{end+1}=[links{j} ': cyclic stroke exceeds catalogue'];end %#ok<AGROW>
 if x.peakIndividualForce_N>part.forceEnvelope_N,reasons{end+1}=[links{j} ': individual force exceeds envelope'];end %#ok<AGROW>
end
if s.combinedFreeLengthMax_mm>s.limit_mm+1e-9,reasons{end+1}='combined free spring length exceeds 100 mm';end
if s.combinedInstalledLengthMax_mm>s.limit_mm+1e-9,reasons{end+1}='combined installed spring length exceeds 100 mm';end
if isempty(reasons),reasons={'pass'};end
end

function summary=summarize(configs,rows,preloads)
summary=struct([]);
for j=1:numel(configs)
 c=configs(j);a=rows([rows.configId]==c.id);
 geometry=true(size(preloads));
 if isempty(a),geometry(:)=false;
 else,for k=1:numel(a),geometry=geometry & a(k).geometryPassed;end,end
 executed=sum([a.executed]);allCorners=numel(a)==4 && executed==4;
 physical=allCorners && all([a.physicalPassed]);
 if allCorners,[worst,worstIndex]=max([a.peakAbsForceError_N]);worstCorner=a(worstIndex).cornerId;
 else,worst=Inf;worstCorner=NaN;end
 x=c;x.executedCorners=executed;x.physicalFullRangePassed=physical;
 x.geometryFullRangePassed=allCorners && any(geometry);x.feasiblePreloads_N=preloads(geometry);
 x.fullRangePassed=physical && x.geometryFullRangePassed;x.worstError_N=worst;x.worstCornerId=worstCorner;
 if isempty(a),x.peakCurrent_A=NaN;x.peakVoltage_V=NaN;
 else,x.peakCurrent_A=max([a.peakCurrent_A]);x.peakVoltage_V=max([a.peakVoltage_V]);end
 if isempty(summary),summary=x;else,summary(end+1)=x;end %#ok<AGROW>
end
end

function table=rows_table(rows)
flat=rows;
for k=1:numel(flat)
 flat(k).geometryPassed=mat2str(flat(k).geometryPassed);
 flat(k).geometryReason=strjoin(flat(k).geometryReason,' | ');
end
table=struct2table(flat);
end

function write_json(file,value)
pending=[file '.pending'];fid=fopen(pending,'w');assert(fid>=0,'Cannot create result file.');
try,fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));fclose(fid);
catch err,fclose(fid);rethrow(err);end
[ok,message]=movefile(pending,file,'f');assert(ok,message);
end

function finish_owned_model(mdl)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;
end

function value=ternary(condition,a,b)
if condition,value=a;else,value=b;end
end
