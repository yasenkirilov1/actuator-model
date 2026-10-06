function report=qualify_mechanics_high(snapshotRoot,outputFolder)
%QUALIFY_MECHANICS_HIGH Native Simulink full-range spring/damper study.
% Isolated immutable model and helpers; no model save or alternative physics.
oldThreads=maxNumCompThreads(2);
if nargin<1,snapshotRoot='/home/tommie/.local/state/actuator-model/qualification-20260930';end
if nargin<2,outputFolder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'validation');end
if ~isfolder(outputFolder),mkdir(outputFolder);end
addpath(fullfile(snapshotRoot,'helpers'),'-begin');
addpath(fullfile(snapshotRoot,'helpers-v2'),'-begin');clear acf_prepare;
stateFolder=fullfile(snapshotRoot,'mechanics_high');if ~isfolder(stateFolder),mkdir(stateFolder);end
Simulink.fileGenControl('set','CacheFolder',fullfile(stateFolder,'cache'), ...
 'CodeGenFolder',fullfile(stateFolder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(stateFolder,'sdi'));Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@() owned_cleanup(oldStorage,oldRecord,oldThreads)); %#ok<NASGU>
loaded=load(fullfile(snapshotRoot,'base_parameters.mat'),'p');base=loaded.p;
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.robot.forceGate.enabled=true;
base.mechanics.lowerRigid=false;base.mechanics.upperRigid=false;
catalog=jsondecode(fileread(fullfile(snapshotRoot,'spring_catalog.json')));
mdl='acf_force_control_simple';load_system(fullfile(snapshotRoot,[mdl '.slx']));
delayBlock=[mdl '/Force measurement/Sensor sample delay'];set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples');
damping=[0 5 15 30 60];preloads=[10 20 28 30 32 34];corners=[5 0;5 90;50 0;50 90];
configs=cell(200,1);designs=struct([]);number=0;
for lower=3:4
 for upper=1:4
  for c1=damping
   for c2=damping
    number=number+1;p=base;p.springs.lower=catalog.springs(lower);p.springs.upper=catalog.springs(upper);
    p.mechanics.k1=2*p.springs.lower.rate_N_m;p.mechanics.k2=2*p.springs.upper.rate_N_m;
    p.mechanics.c1=c1;p.mechanics.c2=c2;p.springs.commonPreload_N=10;configs{number}=p;
    one=struct('id',number,'lowerCatalogueIndex',lower,'upperCatalogueIndex',upper, ...
     'lowerPart',p.springs.lower.part,'upperPart',p.springs.upper.part, ...
     'k1_N_m',p.mechanics.k1,'k2_N_m',p.mechanics.k2,'c1_Ns_m',c1,'c2_Ns_m',c2, ...
     'nominalTrackingPassedAllCorners',false,'geometryPassedAllCorners',false, ...
     'fullyFeasibleNominal',false,'worstNominalError_N',NaN,'worstNominalCurrent_A',NaN, ...
     'worstNominalVoltage_V',NaN,'minimumTravel_mm',NaN,'maximumTravel_mm',NaN, ...
     'feasiblePreloadMask',false(1,numel(preloads)),'feasiblePreloads_N','', ...
     'nominalCommonPreloadMin_N',NaN,'nominalCommonPreloadMax_N',NaN,'nominalCommonPreloadWindow_N',NaN, ... ...
     'stressTested',false,'stressTrackingPassedAllCorners',false,'stressGeometryPassedAllCorners',false, ...
     'fullyFeasibleStress',false,'worstStressError_N',NaN,'worstStressCurrent_A',NaN, ...
     'worstStressVoltage_V',NaN,'stressFeasiblePreloadMask',false(1,numel(preloads)),'stressFeasiblePreloads_N','', ...
     'stressCommonPreloadMin_N',NaN,'stressCommonPreloadMax_N',NaN,'stressCommonPreloadWindow_N',NaN, ...
     'recommendedCommonPreload_N',NaN);
    if isempty(designs),designs=one;else,designs(end+1)=one;end %#ok<AGROW>
   end
  end
 end
end
assert(number==200);started=tic;cases=struct([]);screenCases=struct([]);stressCases=struct([]);selectedRuns={};
checkpoint=fullfile(outputFolder,'full_range_mechanics_high_checkpoint.mat');
if isfile(checkpoint)
 try
  previous=load(checkpoint,'cases','screenCases','stressCases','legacySynthesisRejections');
  cases=regrid_geometry(previous.cases,preloads,catalog,fullfile(snapshotRoot,'helpers','acf_prepare.m'));
  screenCases=regrid_geometry(previous.screenCases,preloads,catalog,fullfile(snapshotRoot,'helpers','acf_prepare.m'));
  if isfield(previous,'stressCases'),stressCases=regrid_geometry(previous.stressCases,preloads,catalog,fullfile(snapshotRoot,'helpers','acf_prepare.m'));end
  fprintf('HIGH_RESUME %d nominal cases recovered; additional preload checked from exact recorded compression extrema.\n',numel(cases));
 catch ME
  fprintf('HIGH_CHECKPOINT_REJECTED: %s\n',ME.message);
  movefile(checkpoint,fullfile(stateFolder,'interrupted_checkpoint.mat'),'f');
 end
end
% Early physical/packing screen of the strongest compact pair, before full batches.
legacySynthesisRejections=struct([]);
if exist('previous','var') && isfield(previous,'legacySynthesisRejections'),legacySynthesisRejections=previous.legacySynthesisRejections;end
if ~isempty(cases)
 retry=contains({cases.executionError},'covariance did not converge');
 if any(retry)
  if isempty(legacySynthesisRejections),legacySynthesisRejections=cases(retry);else,legacySynthesisRejections=[legacySynthesisRejections,cases(retry)];end
 end
 cases=cases(~retry);
 fprintf('HIGH_MIGRATION %d prior covariance rejections will be rerun with validated doubling coefficients.\n',sum(retry));
end
screenIds=find([designs.lowerCatalogueIndex]==4 & [designs.upperCatalogueIndex]==4 & ...
 ([designs.c1_Ns_m]==30 & [designs.c2_Ns_m]==5 | [designs.c1_Ns_m]==0 & [designs.c2_Ns_m]==0));
for id=screenIds
 if ~isempty(screenCases) && any([screenCases.id]==id),continue;end
 p=configs{id};p.force.reference=50;p.mechanics.angle=pi/2;
 [row,~]=execute_case(mdl,p,id,4,'screen',preloads);
 if isempty(screenCases),screenCases=row;else,screenCases(end+1)=row;end %#ok<AGROW>
 fprintf('HIGH_SCREEN id %d k %.3g/%.3g c %.3g/%.3g: error %.8g N, tracking %d, preload mask %s, I %.6g A V %.6g V\n', ...
 id,p.mechanics.k1,p.mechanics.k2,p.mechanics.c1,p.mechanics.c2,row.peakAbsForceError_N,row.trackingPassed, ...
 mat2str(row.feasiblePreloadMask),row.peakCurrent_A,row.peakVoltage_V);
end
set_param(mdl,'FastRestart','off');Simulink.sdi.clear;
for corner=1:4
 set_param(mdl,'FastRestart','off');set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples');
 set_param(mdl,'FastRestart','on');
 for id=1:200
  if ~isempty(cases) && any([cases.id]==id & [cases.cornerId]==corner),continue;end
  p=configs{id};p.force.reference=corners(corner,1);p.mechanics.angle=corners(corner,2)*pi/180;
  [row,~]=execute_case(mdl,p,id,corner,'nominal',preloads);
  if isempty(cases),cases=row;else,cases(end+1)=row;end %#ok<AGROW>
  if mod(id,20)==0
   save([checkpoint '.pending'],'cases','screenCases','designs','configs','corners','preloads','damping','legacySynthesisRejections','-v7.3');
   movefile([checkpoint '.pending'],checkpoint,'f');
   Simulink.sdi.clear;
   fprintf('HIGH_SWEEP corner %d/4 (%g N %g deg), %d/200; elapsed %.1f s; physical passes %d/%d\n', ...
    corner,corners(corner,1),corners(corner,2),id,toc(started),sum([cases.trackingPassed]),numel(cases));
  end
 end
 set_param(mdl,'FastRestart','off');
end
for id=1:200
 rows=cases([cases.id]==id);assert(numel(rows)==4);
 designs(id).nominalTrackingPassedAllCorners=all([rows.trackingPassed]);
 mask=all(vertcat(rows.feasiblePreloadMask),1);designs(id).feasiblePreloadMask=mask;
 designs(id).feasiblePreloads_N=strtrim(sprintf('%g ',preloads(mask)));
 designs(id).geometryPassedAllCorners=any(mask);
 [low,high]=preload_window(rows,catalog,base.springs);
 designs(id).nominalCommonPreloadMin_N=low;designs(id).nominalCommonPreloadMax_N=high;
 designs(id).nominalCommonPreloadWindow_N=max(0,high-low);
 designs(id).fullyFeasibleNominal=designs(id).nominalTrackingPassedAllCorners && any(mask);
 designs(id).worstNominalError_N=finite_max([rows.peakAbsForceError_N]);
 designs(id).worstNominalCurrent_A=finite_max([rows.peakCurrent_A]);
 designs(id).worstNominalVoltage_V=finite_max([rows.peakVoltage_V]);
 designs(id).minimumTravel_mm=finite_min([rows.minTravel_mm]);designs(id).maximumTravel_mm=finite_max([rows.maxTravel_mm]);
end
valid=find([designs.fullyFeasibleNominal]);candidateIds=valid;
fprintf('HIGH_NOMINAL_COMPLETE %d/%d fully feasible across four corners, %d physical tracking passes, %d packing passes.\n', ...
 numel(valid),200,sum([designs.nominalTrackingPassedAllCorners]),sum([designs.geometryPassedAllCorners]));
% Extra force delay is deliberately absent from the estimator configuration.
for corner=1:4
 set_param(mdl,'FastRestart','off');set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');
 set_param(mdl,'FastRestart','on');
 for id=candidateIds
  if ~isempty(stressCases) && any([stressCases.id]==id & [stressCases.cornerId]==corner),continue;end
  p=configs{id};p.force.reference=corners(corner,1);p.mechanics.angle=corners(corner,2)*pi/180;
  p.sensor.noiseRms=.02;p.sensor.seed=7;
  [row,~]=execute_case(mdl,p,id,corner,'stress',preloads);
  if isempty(stressCases),stressCases=row;else,stressCases(end+1)=row;end %#ok<AGROW>
  fprintf('HIGH_STRESS id %d corner %d: error %.8g N tracking %d packing %s\n',id,corner, ...
   row.peakAbsForceError_N,row.trackingPassed,mat2str(row.feasiblePreloadMask));
 end
 set_param(mdl,'FastRestart','off');Simulink.sdi.clear;
end
for id=candidateIds
 rows=stressCases([stressCases.id]==id);designs(id).stressTested=true;
 designs(id).stressTrackingPassedAllCorners=all([rows.trackingPassed]);
 mask=designs(id).feasiblePreloadMask & all(vertcat(rows.feasiblePreloadMask),1);
 designs(id).stressFeasiblePreloadMask=mask;designs(id).stressFeasiblePreloads_N=strtrim(sprintf('%g ',preloads(mask)));
 designs(id).stressGeometryPassedAllCorners=any(mask);
 nominal=cases([cases.id]==id);[low,high]=preload_window([nominal,rows],catalog,base.springs);
 designs(id).stressCommonPreloadMin_N=low;designs(id).stressCommonPreloadMax_N=high;
 designs(id).stressCommonPreloadWindow_N=max(0,high-low);
 if high>=low
  rounded=round((low+high)/2);
  if rounded>=low && rounded<=high,designs(id).recommendedCommonPreload_N=rounded;else,designs(id).recommendedCommonPreload_N=(low+high)/2;end
 end
 designs(id).fullyFeasibleStress=designs(id).stressTrackingPassedAllCorners && any(mask);
 designs(id).worstStressError_N=finite_max([rows.peakAbsForceError_N]);
 designs(id).worstStressCurrent_A=finite_max([rows.peakCurrent_A]);designs(id).worstStressVoltage_V=finite_max([rows.peakVoltage_V]);
end
valid=find([designs.fullyFeasibleStress]);selected=struct([]);selectedParameters=struct([]);
if ~isempty(valid)
 [~,j]=min([designs(valid).worstStressError_N]);selected=designs(valid(j));selectedParameters=configs{selected.id};
 selectedParameters.springs.commonPreload_N=selected.recommendedCommonPreload_N;
 for phase=1:2
  set_param(mdl,'FastRestart','off');
  if phase==1,set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples');else,set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');end
  for corner=1:4
   p=selectedParameters;p.force.reference=corners(corner,1);p.mechanics.angle=corners(corner,2)*pi/180;
   if phase==2,p.sensor.noiseRms=.02;p.sensor.seed=7;end
   [row,r]=execute_case(mdl,p,selected.id,corner,'selected-ordinary',preloads);
   if phase==1,expected=cases([cases.id]==selected.id & [cases.cornerId]==corner);
   else,expected=stressCases([stressCases.id]==selected.id & [stressCases.cornerId]==corner);end
   matchTolerance=1e-9;
   if ~strcmp(row.coefficientHelperSHA256,expected.coefficientHelperSHA256),matchTolerance=1e-7;end
   assert(numel(expected)==1 && abs(row.peakAbsForceError_N-expected.peakAbsForceError_N)<matchTolerance, ...
    'Fast Restart and ordinary selected histories differ.');
   r.label=sprintf('Selected high-rate design #%d / %g N / %g deg / phase %d',selected.id,p.force.reference,p.mechanics.angle*180/pi,phase);
   selectedRuns{end+1}=r; %#ok<AGROW>
  end
 end
end
report=struct('date','2026-09-30','matlabVersion',version,'modelSnapshot',fullfile(snapshotRoot,[mdl '.slx']), ...
 'helperSnapshot',fullfile(snapshotRoot,'helpers'),'coefficientHelperOverride',fullfile(snapshotRoot,'helpers-v2','acf_prepare.m'), ...
 'initialCoefficientHelperSHA256',file_sha256(fullfile(snapshotRoot,'helpers','acf_prepare.m')), ...
 'finalCoefficientHelperSHA256',file_sha256(fullfile(snapshotRoot,'helpers-v2','acf_prepare.m')), ...
 'coefficientMigrationEvidence','validation/riccati_solver_regression_results.json; five native noisy regressions changed true force by at most 2.96e-9 N, with unchanged physical equations and controller objective.', ...
 'legacySynthesisRejections',legacySynthesisRejections,'experiment',mfilename('fullpath'), ...
 'modelSHA256',file_sha256(fullfile(snapshotRoot,[mdl '.slx'])),'parametersSHA256',file_sha256(fullfile(snapshotRoot,'base_parameters.mat')), ...
 'configurations',200,'nominalAttempts',numel(cases),'stressAttempts',numel(stressCases), ...
 'nominalSimulations',sum(cellfun(@isempty,{cases.executionError})), ...
 'stressSimulations',sum(cellfun(@isempty,{stressCases.executionError})), ...
 'nominalExecutionRejections',sum(~cellfun(@isempty,{cases.executionError})), ...
 'screenSimulations',numel(screenCases),'selectedOrdinarySimulations',numel(selectedRuns), ...
 'nominalTrackingPassedDesigns',sum([designs.nominalTrackingPassedAllCorners]), ...
 'nominalPackingPassedDesigns',sum([designs.geometryPassedAllCorners]), ...
 'nominalFullyFeasibleDesigns',sum([designs.fullyFeasibleNominal]),'stressFullyFeasibleDesigns',sum([designs.fullyFeasibleStress]), ...
 'corners_N_deg',corners,'commonPreloads_N',preloads,'dampingValues_Ns_m',damping, ...
 'catalogue',catalog,'baseParameters',base,'selected',selected,'selectedParameters',selectedParameters, ...
 'designs',designs,'cases',cases,'stressCases',stressCases,'screenCases',screenCases, ...
 'selectionRule','Stress-test every nominally fully feasible design across all four corners; select the smallest worst-corner stress error. Recommend a whole-newton common preload near the middle of the exact common-preload interval that passes all clean and stress recordings.', ...
 'controllerPolicy','Fixed controller design weights; native observer and integral state-feedback coefficients recalculated for each spring/damper design.', ...
 'physicalTrackingDefinition','Run completed, true force within +/-0.5 N at every logged 0.1 ms solver sample, no contact loss, no travel stop and no overcurrent; spring geometry is screened separately.', ...
 'packingDefinition','One common preload works in all four force/angle corners, including all recorded transients, tolerances, minimum working length and unseating clearance, cyclic stroke, catalogue force envelope, and combined free/installed length <=100 mm.', ...
 'geometryMethod','Opposed-pair common preload leaves net link preload and physical equations unchanged; acf_metrics is rerun for the same recorded samples at each common preload. Continuous feasible preload intervals follow from the same exact recorded extrema and catalogue constraints. If a checkpoint is extended to an additional preload, exact affine translation of recorded compression extrema is mathematically identical to screening the samples again.', ...
 'stressTest','0.02 N RMS force noise, seed 7, and an extra unmodeled 0.2 ms force-sample delay; nominal 1 micrometer RMS magnet noise with calibrated 0.2 ms magnet delay.', ...
 'scopeLimit','Four force/orientation corners only. This sweep is not by itself an interior grid qualification or hardware validation.', ...
 'fastRestart',true,'selectedOrdinaryExecutionMatch',~isempty(selectedRuns), ...
 'checkpointReuse','Completed cases at unchanged frozen model and controller settings are reused; preload additions are exact geometry post-screens of those same recorded samples.', ...
 'elapsedTimeScope','Elapsed time for this invocation; previously completed checkpoint simulations are excluded.', ...
 'elapsedSeconds',toc(started));
write_json(fullfile(outputFolder,'full_range_mechanics_high_results.json'),report);
writetable(struct2table(designs),fullfile(outputFolder,'full_range_mechanics_high_designs.csv'));
write_case_csv(fullfile(outputFolder,'full_range_mechanics_high_cases.csv'),cases);
if ~isempty(stressCases),write_case_csv(fullfile(outputFolder,'full_range_mechanics_high_stress_cases.csv'),stressCases);end
save(fullfile(outputFolder,'full_range_mechanics_high_selected_runs.mat'),'report','selectedRuns','-v7');
save([checkpoint '.pending'],'cases','screenCases','stressCases','designs','configs','corners','preloads','damping','legacySynthesisRejections','report','-v7.3');
movefile([checkpoint '.pending'],checkpoint,'f');
fprintf('HIGH_SWEEP_COMPLETE elapsed %.1f s; nominal fully feasible %d, stress fully feasible %d.\n', ...
 toc(started),report.nominalFullyFeasibleDesigns,report.stressFullyFeasibleDesigns);
if ~isempty(selected),fprintf('HIGH_SELECTED %s\n',jsonencode(selected));end
end

function [row,r]=execute_case(mdl,p,id,corner,phase,preloads)
r=[];g=struct([]);mask=false(1,numel(preloads));failure='';
try
 quiet=evalc('r=acf_run(mdl,p,false);'); %#ok<NASGU>
 physical=r;physical.parameters=rmfield(physical.parameters,'springs');m=acf_metrics(physical);
 for k=1:numel(preloads)
  z=r;z.parameters.springs.commonPreload_N=preloads(k);s=acf_metrics(z);s=s.springSizing;
  one=struct('commonPreload_N',preloads(k),'passed',s.passed,'sizing',s);
  if isempty(g),g=one;else,g(end+1)=one;end %#ok<AGROW>
  mask(k)=s.passed;
 end
catch ME
 failure=getReport(ME,'basic','hyperlinks','off');
 m=struct('passed',false,'peakAbsForceError_N',NaN,'rmsForceError_N',NaN,'fractionWithinTolerance',NaN, ...
  'timeOutsideTolerance_s',NaN,'completed',false,'lostContact',false,'strokeStopHit',false,'overCurrent',false, ...
  'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN,'minTravel_mm',NaN,'maxTravel_mm',NaN, ...
  'peakGapAcceleration_m_s2',NaN,'peakGapSpeed_m_s',NaN,'peakAxialForce_N',NaN,'reason',ME.message);
 fprintf('HIGH_CASE_ERROR id %d corner %d: %s\n',id,corner,ME.message);
end
row=struct('id',id,'cornerId',corner,'phase',phase,'referenceForce_N',p.force.reference,'angle_deg',p.mechanics.angle*180/pi, ...
 'k1_N_m',p.mechanics.k1,'k2_N_m',p.mechanics.k2,'c1_Ns_m',p.mechanics.c1,'c2_Ns_m',p.mechanics.c2, ...
 'trackingPassed',m.passed,'completed',m.completed,'lostContact',m.lostContact,'strokeStopHit',m.strokeStopHit,'overCurrent',m.overCurrent, ...
 'peakAbsForceError_N',m.peakAbsForceError_N,'rmsForceError_N',m.rmsForceError_N,'fractionWithinTolerance',m.fractionWithinTolerance, ...
 'timeOutsideTolerance_s',m.timeOutsideTolerance_s,'peakCurrent_A',m.peakCurrent_A,'rmsCurrent_A',m.rmsCurrent_A,'peakVoltage_V',m.peakVoltage_V, ...
 'minTravel_mm',m.minTravel_mm,'maxTravel_mm',m.maxTravel_mm,'peakGapAcceleration_m_s2',m.peakGapAcceleration_m_s2, ...
 'peakGapSpeed_m_s',m.peakGapSpeed_m_s,'peakAxialForce_N',m.peakAxialForce_N,'physicalReason',m.reason, ...
 'feasiblePreloadMask',mask,'geometryByPreload',g,'executionError',failure, ...
 'coefficientHelperPath',which('acf_prepare'),'coefficientHelperSHA256',file_sha256(which('acf_prepare')));
end
function [low,high]=preload_window(rows,catalog,pack)
% Exact feasible common-preload interval for all recorded pair extrema.
low=0;high=inf;
for n=1:numel(rows)
 if isempty(rows(n).geometryByPreload),low=NaN;high=NaN;return;end
 origin=rows(n).geometryByPreload(1);sizing=origin.sizing;
 if sizing.combinedFreeLengthMax_mm>1000*pack.maxCombinedLength_m,low=NaN;high=NaN;return;end
 links={'lower','upper'};
 for j=1:2
  one=sizing.(links{j});spec=catalog.springs(strcmp({catalog.springs.part},one.part));
  bias=max(0,spec.rate_N_m*one.maxCompression_mm/1000-origin.commonPreload_N);
  low=max(low,bias+spec.rate_N_m*pack.unseatingClearance_m);
  available=min(spec.forceEnvelope_N,spec.rate_N_m*(spec.freeLength_m-spec.freeLengthTolerance_m- ...
   spec.minWorkingLength_m-pack.workingClearance_m));
  high=min(high,available-bias);
  if one.cyclicExcursion_mm>1000*spec.maxCyclicStroke_m,low=NaN;high=NaN;return;end
 end
end
end
function value=finite_max(values)
if all(isfinite(values)),value=max(values);else,value=NaN;end
end
function value=finite_min(values)
if all(isfinite(values)),value=min(values);else,value=NaN;end
end
function write_case_csv(path,rows)
flat=rmfield(rows,'geometryByPreload');writetable(struct2table(flat),path);
end
function write_json(path,value)
fid=fopen(path,'w');assert(fid>=0);closer=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(value,PrettyPrint=true));
end
function value=file_sha256(path)
% Batch MATLAB has no Java runtime on this host; use the system SHA utility.
persistent previousPath previousValue
if isequal(path,previousPath),value=previousValue;return;end
quoted=[char(39),strrep(path,char(39),char([39 34 39 34 39])),char(39)];
[status,result]=system(['sha256sum -- ' quoted]);assert(status==0,'Unable to hash source file.');
value=regexp(result,'^[0-9a-f]{64}','match','once');assert(~isempty(value));
previousPath=path;previousValue=value;
end
function rows=regrid_geometry(rows,preloads,catalog,legacyHelperPath)
% Exact affine translation of pair compression, preserving observed extrema.
for n=1:numel(rows)
 if ~isfield(rows,'coefficientHelperPath') || isempty(rows(n).coefficientHelperPath)
  rows(n).coefficientHelperPath=legacyHelperPath;rows(n).coefficientHelperSHA256=file_sha256(legacyHelperPath);
 end
 old=rows(n).geometryByPreload;updated=struct([]);mask=false(1,numel(preloads));
 for k=1:numel(preloads)
  if isempty(old),continue;end
  j=find([old.commonPreload_N]==preloads(k),1);
  if isempty(j)
   one=old(1);difference=preloads(k)-one.commonPreload_N;one.commonPreload_N=preloads(k);
   sizing=one.sizing;installedChange=0;
   links={'lower','upper'};
   for q=1:2
    spec=catalog.springs(strcmp({catalog.springs.part},sizing.(links{q}).part));
    delta=1000*difference/spec.rate_N_m;a=sizing.(links{q});
    a.minCompression_mm=a.minCompression_mm+delta;a.maxCompression_mm=a.maxCompression_mm+delta;
    a.minIndividualLength_mm=a.minIndividualLength_mm-delta;a.maxIndividualLength_mm=a.maxIndividualLength_mm-delta;
    a.peakIndividualForce_N=a.peakIndividualForce_N+difference;
    a.passed=a.minCompression_mm>=.25 && a.minIndividualLength_mm>=1000*spec.minWorkingLength_m+.5 && ...
     a.cyclicExcursion_mm<=1000*spec.maxCyclicStroke_m && a.peakIndividualForce_N<=spec.forceEnvelope_N;
    sizing.(links{q})=a;installedChange=installedChange+2*delta;
   end
   sizing.combinedInstalledLengthMax_mm=sizing.combinedInstalledLengthMax_mm-installedChange;
   sizing.passed=sizing.lower.passed && sizing.upper.passed && sizing.combinedFreeLengthMax_mm<=100+1e-9 && ...
    sizing.combinedInstalledLengthMax_mm<=100+1e-9;one.sizing=sizing;one.passed=sizing.passed;
  else,one=old(j);end
  if isempty(updated),updated=one;else,updated(end+1)=one;end %#ok<AGROW>
  mask(k)=one.passed;
 end
 rows(n).feasiblePreloadMask=mask;rows(n).geometryByPreload=updated;
end
end
function owned_cleanup(oldStorage,oldRecord,oldThreads)
try
 if bdIsLoaded('acf_force_control_simple'),set_param('acf_force_control_simple','FastRestart','off');close_system('acf_force_control_simple',0);end
 Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldStorage);Simulink.sdi.setRecordData(oldRecord);
 maxNumCompThreads(oldThreads);
catch ME
 fprintf('OWNED_CLEANUP: %s\n',ME.message);
end
end
