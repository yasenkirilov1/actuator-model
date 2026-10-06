function report=run_matrix_native(root,temp,out)
% Native physical command replay. Controller/acquisition remain external.
assert(~usejava('desktop'));maxNumCompThreads(2);
assert(isfile(fullfile(temp,'owned_matrix_native')));
here=fullfile(root,'acf_simulink','experiments','current_model_matrix_20261006','native');addpath(here);
base=fullfile(root,'acf_simulink','experiments','sea_pad_dynamic_20261006','native');addpath(base);
assert(~isempty(which('load_system'))&&~isempty(ver('simulink')),'Simulink installation required.');
manifest=jsondecode(fileread(fullfile(temp,'manifest.json')));assert(numel(manifest.cases)<=4);
baseContract=fullfile(fileparts(base),'study_contract.json');
sources={fullfile(here,'matrix_native_build.m'),fullfile(base,'sea_pad_native_params.m'), ...
 fullfile(base,'sea_pad_native_from_engine.m'),baseContract,manifest.matrix_contract, ...
 fullfile(fileparts(here),'engine.c'),fullfile(fileparts(here),'engine_api.py')};
hashes=cellfun(@sha256,sources,'UniformOutput',false);
assert(strcmp(sha256(manifest.matrix_contract),manifest.matrix_contract_sha256));
assert(strcmp(hashes{6},manifest.engine_source_sha256));
assert(strcmp(hashes{7},manifest.api_source_sha256));
storage=Simulink.sdi.getStorageLocation;record=Simulink.sdi.getRecordData;
sdi=fullfile(temp,'sdi');mkdir(sdi);Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
modelFile=fullfile(temp,'acf_matrix_native_private_20261006.slx');[~,mdl]=fileparts(modelFile);
cleanup=onCleanup(@()finish(storage,record,mdl)); %#ok<NASGU>
p=translate(baseContract,manifest.cases(1).config);
prepared=matrix_native_build(p,modelFile,temp);load_system(modelFile);
mw=get_param(mdl,'ModelWorkspace');rows=struct([]);refinements=struct([]);nativePoints=0;sidePoints=0;refinementPoints=0;
tempPeak=treebytes(temp);resultFile=fullfile(out,'native_results.json');
for j=1:numel(manifest.cases)
 c=manifest.cases(j);p=translate(baseContract,c.config);
 commands=readmatrix(c.commands_csv);assert(size(commands,2)==2);
 assert(max(abs(commands(:,1)-(0:size(commands,1)-1)'*p.controlStep))<1e-10);
 replayInputs=[commands zeros(size(commands,1),1)];
 assignin(mw,'p',p);assignin(mw,'replayInputs',replayInputs);assignin('base','replayInputs',replayInputs);
 set_param(mdl,'FixedStep',sprintf('%.17g',p.dt),'StopTime','p.stopTime','Decimation','1');
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 result=sim(mdl);ts=result.yout{1}.Values;time=ts.Time;data=matrix(ts,24);
 assert(numel(time)>1&&max(abs(time-(0:numel(time)-1)'*p.dt))<1e-10);
 tempPeak=max(tempPeak,treebytes(temp));assert(tempPeak<=manifest.temporary_cap_bytes,'Temporary allocation exceeded.');
 assert(strcmp(sha256(c.reference_binary),c.reference_sha256));
 expected=read_binary(c.reference_binary,1);assert(numel(expected)>=numel(time));
 forceDifference=max(abs(data(:,10)-expected(1:numel(time))));
 guard=manifest.native_force_guard_N;if p.padDiameter>0,guard=manifest.footprint_guard_N;end
 assert(strcmp(sha256(c.reference_states_binary),c.reference_states_sha256));
 stateReference=read_binary(c.reference_states_binary,9);indices=1:c.reference_state_stride:numel(time);
 stateError=max(abs(data(indices,1:9)-stateReference(1:numel(indices),:)),[],1);
 scores=score(time,data,p);balancePeak=force_balance(data,p);assert(balancePeak<1e-8);
 events=side_events(time,data,p);eventDifference=0;
 for k=1:numel(events)
  reference=c.regular_C_rigid_side_events(k);
  eventDifference=max(eventDifference,max(abs([events(k).left_force_N-reference.left_force_N,events(k).right_force_N-reference.right_force_N])));
 end
 forceDifference=max(forceDifference,eventDifference);
 original=readmatrix(c.original_commands_csv);originalForceDifference=NaN;
 if c.original_commands_columns==4
  originalForceDifference=max(abs(data(indices,10)-original(1:numel(indices),4)));
 end
 nativePoints=nativePoints+numel(time);sidePoints=sidePoints+numel(events);
 traceIndices=unique([1:round(.002/p.dt):numel(time),numel(time)]);
 trace=[time(traceIndices),data(traceIndices,10),data(traceIndices,16),data(traceIndices,22),data(traceIndices,9), ...
  data(traceIndices,1)-data(traceIndices,3),data(traceIndices,14),data(traceIndices,12),data(traceIndices,17)];
 traceNames={'time_s','total_axial_force_N','cell_raw_N','S_acceleration_m_s2','current_A','flange_travel_m','max_pad_compression_m','active_fraction','voltage_request_V'};
 traceFile=fullfile(temp,[c.identity '_native_500Hz.csv']);write_csv(traceFile,traceNames,trace);
 traceHash=sha256(traceFile);gzip(traceFile,out);traceArchive=fullfile(out,[c.identity '_native_500Hz.csv.gz']);delete(traceFile);
 row=struct('identity',c.identity,'group',c.group,'status','executed_native_physical_command_replay', ...
  'controllerIndependentlyVerified',false,'scoredEveryNativeEndpoint',true, ...
  'nativeEndpointCount',numel(time),'extraLeftLimitCount',numel(events), ...
  'nativeScoredValueCount',numel(time)+numel(events),'rigidSideEvents',events, ...
  'forceParityPassed',forceDifference<=guard,'native_vs_regular_C_force_peak_difference_N',forceDifference, ...
  'native_vs_regular_C_side_event_force_peak_difference_N',eventDifference,'predeclaredForceGuard_N',guard, ...
  'originalSplitEventForceGuard_N',guard,'originalSplitEventForceParityAtControllerTicksPassed',originalForceDifference<=guard, ...
  'native_vs_original_split_event_force_peak_difference_at_10k_N',originalForceDifference, ...
  'stateParityComponentErrors',stateError,'stateParitySampleRate_Hz',1/p.controlStep, ...
  'forceBalancePeak_N',balancePeak,'algebraicLoopCount',loops,'config',c.config,'configSha256',c.config_sha256, ...
  'nativeScores',scores,'originalMetrics',c.original_metrics,'commandCapture',c.command_capture, ...
  'commandInputsSha256',sha256(c.commands_csv),'originalCommandsUncompressedSha256',c.commands_uncompressed_sha256, ...
  'traceArchive',traceArchive,'traceUncompressedSha256',traceHash,'traceArchiveSha256',sha256(traceArchive), ...
  'regularReferenceSha256',c.reference_sha256,'regularStateReferenceSha256',c.reference_states_sha256);
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 fprintf('MATRIX_NATIVE_CASE %s\n',jsonencode(row));
 baseForce=data(:,10);baseTime=time;clear result ts data expected stateReference trace;Simulink.sdi.clear;
 delete(c.reference_binary);delete(c.reference_states_binary);
 if isfield(c,'refinement')&&c.refinement
  set_param(mdl,'FixedStep',sprintf('%.17g',p.dt/2),'Decimation','1');
  refined=sim(mdl);rts=refined.yout{1}.Values;rTime=rts.Time;rData=matrix(rts,24);
  assert(max(abs(rTime-(0:numel(rTime)-1)'*p.dt/2))<1e-10);
  rScores=score(rTime,rData,p);rEvents=side_events(rTime,rData,p);rBalance=force_balance(rData,p);assert(rBalance<1e-8);
  shared=min(numel(baseTime),ceil(numel(rTime)/2));
  assert(max(abs(rTime(1:2:2*shared-1)-baseTime(1:shared)))<1e-10);
  refinement=struct('identity',c.identity,'status','native_half_step_refinement_executed', ...
   'baseStep_s',p.dt,'refinementStep_s',p.dt/2,'controllerCommandsUnchanged',true, ...
   'scoredEveryRefinementEndpoint',true,'refinementEndpointCount',numel(rTime), ...
   'extraLeftLimitCount',numel(rEvents),'refinementScoredValueCount',numel(rTime)+numel(rEvents), ...
   'matchedBaseTimestamps',shared,'matchedForceWaveformPeakDifference_N',max(abs(baseForce(1:shared)-rData(1:2:2*shared-1,10))), ...
   'basePeakErrorIncludingSideLimits_N',scores.forcePeakErrorIncludingSideLimits_N, ...
   'refinedPeakErrorIncludingSideLimits_N',rScores.forcePeakErrorIncludingSideLimits_N, ...
   'refinedScores',rScores,'refinedSideEvents',rEvents,'forceBalancePeak_N',rBalance);
  if isempty(refinements),refinements=refinement;else,refinements(end+1)=refinement;end %#ok<AGROW>
  refinementPoints=refinementPoints+numel(rTime)+numel(rEvents);
  tempPeak=max(tempPeak,treebytes(temp));assert(tempPeak<=manifest.temporary_cap_bytes);
  fprintf('MATRIX_NATIVE_REFINEMENT %s\n',jsonencode(refinement));
  clear refined rts rTime rData;Simulink.sdi.clear;
 end
 clear baseForce baseTime;
 report=assemble(rows,refinements,nativePoints,sidePoints,refinementPoints,tempPeak,manifest,prepared,sources,hashes);
 write_json(resultFile,report);
end
% Save/reopen the exact final private graph without another physical recording.
set_param(mdl,'FixedStep',sprintf('%.17g',p.dt),'Decimation','1');save_system(mdl,modelFile);
close_system(mdl,0);savedHash=sha256(modelFile);load_system(modelFile);
set_param(mdl,'SimulationCommand','update');savedLoops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(savedLoops==0);
mw=get_param(mdl,'ModelWorkspace');reopenedP=mw.getVariable('p');assert(isequaln(p,reopenedP));close_system(mdl,0);
for j=1:numel(sources),assert(strcmp(hashes{j},sha256(sources{j})),'Replay source changed during batch.');end
report=assemble(rows,refinements,nativePoints,sidePoints,refinementPoints,tempPeak,manifest,prepared,sources,hashes);
report.status='native_current_matrix_command_replays_complete';report.modelSavedReopened=true;
report.temporarySavedModelSha256=savedHash;report.savedReopenedAlgebraicLoopCount=savedLoops;
report.savedModelRetained=false;report.temporaryReferenceArraysReleased=true;
write_json(resultFile,report);fprintf('MATRIX_NATIVE_DONE cases=%d endpoints=%d extraLeftLimits=%d refinementValues=%d\n',numel(rows),nativePoints,sidePoints,refinementPoints);
end

function p=translate(contract,c)
p=sea_pad_native_from_engine(contract,c);p.rigidWide=p.rigidPoint&&p.padDiameter>0;p.eventEpsilon=1e-12;
end

function scores=score(time,data,p)
events=side_events(time,data,p);sideForces=[];if ~isempty(events),sideForces=[events.left_force_N];end
allForce=[data(:,10);sideForces(:)];peak=max(abs(allForce-p.Fref));u=data(:,1)-data(:,3);
stops=struct('contactLoss',any(allForce<=0),'flangeTravel',any(abs(u)>p.toolTravelLimit), ...
 'nutTravel',p.sea&&any(abs(data(:,7))>p.nutTravelLimit), ...
 'springTravel',p.sea&&any(abs(data(:,7)-u)>p.springTravelLimit), ...
 'padWorkingCompression',any(data(:,14)>p.padCompressionLimit),'nonfinite',any(~isfinite(data(:))));
physicalFailed=stops.contactLoss||stops.flangeTravel||stops.nutTravel||stops.springTravel||stops.padWorkingCompression||stops.nonfinite;
completed=abs(time(end)-p.stopTime)<1e-10;
scores=struct('actualEnd_s',time(end),'completed',completed,'physicalFailed',physicalFailed,'physicalStops',stops, ...
 'forcePeakErrorIncludingSideLimits_N',peak,'forcePeakErrorEndpointsOnly_N',max(abs(data(:,10)-p.Fref)), ...
 'forceRequirementMetInReplay',completed&&~physicalFailed&&peak<=.5, ...
 'minimumTotalAxialForce_N',min(allForce),'maximumTotalAxialForce_N',max(allForce), ...
 'maximumAbsoluteFlangeTravel_m',max(abs(u)),'maximumAbsoluteNutTravel_m',max(abs(data(:,7))), ...
 'maximumAbsoluteSpringDeflection_m',p.sea*max(abs(data(:,7)-u)), ...
 'maximumPadCompression_m',max(data(:,14)),'minimumActiveFraction',min(data(:,12)), ...
 'partialSoftPadContactObserved',~p.rigidPoint&&any(data(:,12)<1-1e-10), ...
 'currentPeak_A',max(abs(data(:,9))),'currentRmsEveryEndpoint_A',sqrt(mean(data(:,9).^2)), ...
 'hotCopperLossAverageEndpoint_W',p.R*mean(data(:,9).^2),'voltagePeakRequest_V',max(abs(data(:,17))), ...
 'cellRawPeak_N',max(abs(data(:,16))),'specificAccelerationPeak_m_s2',max(abs(data(:,22)+p.gAxis)), ...
 'sensorRangesMet',max(abs(data(:,16)))<=100&&max(abs(data(:,22)+p.gAxis))<=8*9.80665, ...
 'currentPeakRangeMet',max(abs(data(:,9)))<=p.Imax+1e-6);
end

function events=side_events(time,data,p)
events=struct([]);if ~p.rigidWide,return;end
aEdge=-24*p.bumpHeight*(p.scanSpeed/p.bumpWidth)^2;
for k=1:2
 sign=2*k-3;eventTime=p.bumpTime+(p.bumpWidth+sign*p.padDiameter)/(2*p.scanSpeed);
 index=round(eventTime/(time(2)-time(1)))+1;if index>numel(time),continue;end
 assert(abs(time(index)-eventTime)<1e-10);
 if k==1,name='enter_plateau';aLeft=aEdge;aRight=0;else,name='leave_plateau';aLeft=0;aRight=aEdge;end
 cell=data(index,16);left=cell+p.mS*(aLeft+p.gAxis);right=cell+p.mS*(aRight+p.gAxis);
 assert(abs(data(index,10)-right)<1e-8,'Native event endpoint must use right-continuous acceleration.');
 event=struct('transition',name,'time_s',eventTime,'left_acceleration_m_s2',aLeft,'right_acceleration_m_s2',aRight, ...
  'cell_at_event_N',cell,'left_force_N',left,'right_force_N',right, ...
  'left_force_error_N',abs(left-p.Fref),'right_force_error_N',abs(right-p.Fref));
 if isempty(events),events=event;else,events(end+1)=event;end %#ok<AGROW>
end
end

function peak=force_balance(data,p)
residualS=max(abs(data(:,10)-data(:,16)-p.mS*(data(:,22)+p.gAxis)));
residualB=max(abs(p.mB*(data(:,21)+p.gAxis)-(data(:,16)-data(:,23))));
if p.sea
 guide=p.guideCoulomb*tanh((data(:,2)-data(:,4))/p.guideSmooth)+p.guideViscous*(data(:,2)-data(:,4));
 spring=data(:,23)+guide;omega=data(:,8)/p.ell;
 motorForce=(p.Kt*data(:,9)-p.shaftCoulomb*tanh(omega/p.shaftSmooth)-p.shaftViscous*omega)/p.ell;
 lhsA=(p.mA+p.mNut)*data(:,20)-p.mNut*data(:,24);
 lhsQ=-p.mNut*data(:,20)+(p.mNut+p.Jtotal/p.ell^2)*data(:,24);
 residualA=max(abs(lhsA-(data(:,23)-data(:,15)-(p.mA+p.mNut)*p.gAxis)));
 residualQ=max(abs(lhsQ-(motorForce-spring+p.mNut*p.gAxis)));
else
 residualA=max(abs(p.mA*(data(:,20)+p.gAxis)-(data(:,23)-data(:,15))));residualQ=0;
end
peak=max([residualS,residualB,residualA,residualQ]);
end

function report=assemble(rows,refinements,points,sidePoints,refinementPoints,tempPeak,manifest,prepared,sources,hashes)
bindings=struct([]);for j=1:numel(sources),bindings(j).path=sources{j};bindings(j).sha256=hashes{j};end
report=struct('status','native_matrix_batch_in_progress','controllerIndependentlyVerified',false, ...
 'qualification',false,'forceMetric','total axial contact force; includes every native endpoint and explicit rigid left limits', ...
 'physicalPlantNative',true,'hiddenMatlabPlantSolver',false,'sourceBindings',bindings, ...
 'nativePlantPrepared',prepared,'matrixContractSha256',manifest.matrix_contract_sha256, ...
 'sourceSelectionSha256',manifest.source_selection_sha256,'engineBuild',manifest.engine_build,'cases',rows,'refinements',refinements, ...
 'totalNativeEndpointsScored',points,'totalExtraRigidLeftLimitsScored',sidePoints, ...
 'totalNativeScoredValues',points+sidePoints,'totalRefinementScoredValues',refinementPoints, ...
 'nativeVsRegularCAllGuardsPassed',all([rows.forceParityPassed]), ...
 'nativeVsOriginalSplitEventAllGuardsPassed',all([rows.originalSplitEventForceParityAtControllerTicksPassed]), ...
 'observedTemporaryFileBytesPeak',tempPeak,'temporaryAllocationBytes',manifest.temporary_cap_bytes, ...
 'leftLimitsReuseExistingEventTimestamp',true,'strictFailedGuardsPreserved',true);
end

function finish(storage,record,mdl)
try,if bdIsLoaded(mdl),close_system(mdl,0);end,catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
end
function d=matrix(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end;d=reshape(d,n,width);
end
function d=read_binary(file,width)
fid=fopen(file,'rb');assert(fid>=0);d=fread(fid,[width Inf],'double')';fclose(fid);
end
function write_json(file,value),fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);end
function write_csv(file,names,data)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',strjoin(names,','));fclose(fid);writematrix(data,file,'WriteMode','append');
end
function n=treebytes(folder)
items=dir(fullfile(folder,'**','*'));n=sum([items(~[items.isdir]).bytes]);
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
