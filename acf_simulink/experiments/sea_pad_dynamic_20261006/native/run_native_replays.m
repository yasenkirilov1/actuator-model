function report=run_native_replays(root,manifestName,resultName,runRefinement)
% Independent native physical-plant replay; controller commands are recorded.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<2,manifestName='replay_manifest.json';end
if nargin<3,resultName='replay_results.json';end
if nargin<4,runRefinement=true;end
here=fullfile(root,'acf_simulink','experiments','sea_pad_dynamic_20261006','native');addpath(here);
out=fullfile(root,'acf_simulink','validation','sea_pad_dynamic_20261006','native');
contract=fullfile(fileparts(here),'study_contract.json');manifestFile=fullfile(out,manifestName);
manifest=jsondecode(fileread(manifestFile));assert(numel(manifest.cases)<=3);
temp=fullfile(out,'owned_temporary');assert(isfile(fullfile(temp,'owned_sea_pad_native')));
storage=Simulink.sdi.getStorageLocation;record=Simulink.sdi.getRecordData;
sdi=fullfile(temp,'sdi');if ~isfolder(sdi),mkdir(sdi);end
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@()finish(storage,record,temp)); %#ok<NASGU>
modelFile=fullfile(root,'acf_simulink','generated','sea_pad_dynamic_20261006','acf_sea_pad_research_20261006.slx');
builderHash=sha256(fullfile(here,'sea_pad_native_build.m'));paramsHash=sha256(fullfile(here,'sea_pad_native_params.m'));
translatorHash=sha256(fullfile(here,'sea_pad_native_from_engine.m'));contractHash=sha256(contract);
p=sea_pad_native_from_engine(contract,manifest.cases(1).config);
prepared=sea_pad_native_build(p,modelFile,temp);[~,mdl]=fileparts(modelFile);load_system(modelFile);
mw=get_param(mdl,'ModelWorkspace');rows=struct([]);tempPeak=treebytes(temp);totalPoints=0;
for j=1:numel(manifest.cases)
 c=manifest.cases(j);p=sea_pad_native_from_engine(contract,c.config);
 commands=readmatrix(c.commands_csv);assert(size(commands,2)==2);
 assert(max(abs(commands(:,1)-(0:size(commands,1)-1)'*p.controlStep))<1e-10);
 replayInputs=[commands zeros(size(commands,1),1)];
 assignin(mw,'p',p);assignin(mw,'replayInputs',replayInputs);assignin('base','replayInputs',replayInputs);
 set_param(mdl,'FixedStep',sprintf('%.17g',p.dt),'StopTime','p.stopTime','Decimation','1');
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 result=sim(mdl);ts=result.yout{1}.Values;data=matrix(ts,numel(prepared.outputNames));time=ts.Time;
 tempPeak=max(tempPeak,treebytes(temp));assert(size(data,1)>1);
 assert(strcmp(sha256(c.reference_binary),c.reference_sha256));
 expected=read_binary(c.reference_binary,1);assert(numel(expected)==numel(time),'Native replay stopped or output counts differ.');
 assert(max(abs(time-(0:numel(time)-1)'*p.dt))<1e-10);
 forceDifference=max(abs(data(:,10)-expected));
 guard=manifest.native_force_guard_N;if p.padDiameter>0,guard=manifest.footprint_guard_N;end
 stateReference=read_binary(c.reference_states_binary,9);
 stateIndices=1:c.reference_state_stride:numel(time);assert(numel(stateIndices)==size(stateReference,1));
 stateError=max(abs(data(stateIndices,1:9)-stateReference),[],1);
 inertiaResidual=max(abs(data(:,10)-data(:,16)-p.mS*(data(:,22)+p.gAxis)));
 holderResidual=max(abs(p.mB*(data(:,21)+p.gAxis)-(data(:,16)-data(:,23))));
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
 balancePeak=max([inertiaResidual holderResidual residualA residualQ]);
 assert(balancePeak<1e-8,'Native physical force balance rejected.');
 scored=numel(time);totalPoints=totalPoints+scored;
 stride=max(1,round(.002/p.dt));trace=[time(1:stride:end),data(1:stride:end,:)];
 traceFile=fullfile(out,[c.identity '_native_500Hz.csv']);
 write_csv(traceFile,[{'time_s'} prepared.outputNames],trace);
 row=struct('identity',c.identity,'status','executed_native_physical_replay', ...
  'controllerIndependentlyVerified',false,'config',c.config,'scoredEveryBaseStep',true, ...
  'scoredPoints',scored,'algebraicLoopCount',loops,'nativeForcePeakError_N',max(abs(data(:,10)-p.Fref)), ...
  'native_vs_regular_C_force_peak_difference_N',forceDifference,'predeclared_force_guard_N',guard, ...
  'forceParityPassed',forceDifference<=guard,'stateParityComponentErrors',stateError, ...
  'stateParitySampleRate_Hz',1/p.controlStep,'forceBalancePeak_N',balancePeak, ...
  'minimumTotalAxialForce_N',min(data(:,10)),'maximumTotalAxialForce_N',max(data(:,10)), ...
  'minimumActiveFootprintFraction',min(data(:,12)),'maximumActiveFootprintFraction',max(data(:,12)), ...
  'partialContactObserved',any(data(:,12)<1-1e-10),'wholeContactLossObserved',any(data(:,10)<=0), ...
  'traceFile',traceFile,'commandsSha256',sha256(c.commands_csv), ...
  'sourceEngineSha256',manifest.engine_source_sha256);
 if isfield(c,'original_split_event_commands_csv')
  original=readmatrix(c.original_split_event_commands_csv);
  assert(strcmp(sha256(c.original_split_event_commands_csv),c.original_split_event_commands_sha256));
  originalForceDifference=max(abs(data(stateIndices,10)-original(:,4)));
  row.native_vs_original_split_event_force_peak_difference_at_10k_N=originalForceDifference;
  row.originalSplitEventForceGuard_N=guard;
  row.originalSplitEventForceParityAtControllerTicksPassed=originalForceDifference<=guard;
  row.regular_vs_original_split_event_force_peak_difference_at_10k_N=c.split_event_vs_regular_grid_force_peak_difference_at_10k_N;
  row.originalSplitEventMetrics=c.original_split_event_metrics;
 end
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 fprintf('NATIVE_REPLAY_CASE %s\n',jsonencode(row));
 clear result ts data expected stateReference trace;Simulink.sdi.clear;
 delete(c.reference_binary);delete(c.reference_states_binary);
end
% One matched solver-refinement waveform uses the same physical command log.
% Save every second refinement output: compare at all original base timestamps.
if runRefinement
c=manifest.cases(1);p=sea_pad_native_from_engine(contract,c.config);commands=readmatrix(c.commands_csv);
replayInputs=[commands zeros(size(commands,1),1)];assignin(mw,'p',p);assignin(mw,'replayInputs',replayInputs);assignin('base','replayInputs',replayInputs);
set_param(mdl,'FixedStep',sprintf('%.17g',p.dt),'Decimation','1');base=sim(mdl);
baseData=matrix(base.yout{1}.Values,numel(prepared.outputNames));baseForce=baseData(:,10);baseTime=base.yout{1}.Values.Time;
clear baseData base;Simulink.sdi.clear;
set_param(mdl,'FixedStep',sprintf('%.17g',p.dt/2),'Decimation','2');refined=sim(mdl);
refinedData=matrix(refined.yout{1}.Values,numel(prepared.outputNames));refinedTime=refined.yout{1}.Values.Time;
assert(numel(refinedTime)==numel(baseTime)&&max(abs(refinedTime-baseTime))<1e-10);
refinement=struct('identity',c.identity,'status','matched_native_refinement_executed', ...
 'baseStep_s',p.dt,'refinementStep_s',p.dt/2,'matchedRecordedBasePoints',numel(baseTime), ...
 'allRefinementStepsSaved',false,'controllerCommandsUnchanged',true, ...
 'forcePeakDifference_N',max(abs(baseForce-refinedData(:,10))), ...
 'baseForcePeakError_N',max(abs(baseForce-p.Fref)), ...
 'refinementRecordedForcePeakError_N',max(abs(refinedData(:,10)-p.Fref)));
tempPeak=max(tempPeak,treebytes(temp));clear refined refinedData baseForce;Simulink.sdi.clear;
else
 refinement=struct('status','not_run_for_bounded_two_case_append', ...
  'reason','Previously executed refinement evidence is retained; no extra simulations were requested.');
end
% Save the first declared replay as the beginner example, without another run.
c=manifest.cases(1);p=sea_pad_native_from_engine(contract,c.config);commands=readmatrix(c.commands_csv);
replayInputs=[commands zeros(size(commands,1),1)];assignin(mw,'p',p);
assignin(mw,'replayInputs',replayInputs);assignin('base','replayInputs',replayInputs);
set_param(mdl,'FixedStep',sprintf('%.17g',p.dt),'Decimation','1');save_system(mdl,modelFile);
close_system(mdl,0);load_system(modelFile);set_param(mdl,'SimulationCommand','update');
diagramFile=fullfile(fileparts(modelFile),'native-block-diagram.pdf');
try,set_param(mdl,'PaperOrientation','landscape');print(['-s' mdl],'-dpdf',diagramFile);diagramStatus='exported_native_vector_pdf';
catch failure,diagramStatus=['export_rejected: ' failure.message];end
close_system(mdl,0);
assert(strcmp(builderHash,sha256(fullfile(here,'sea_pad_native_build.m')))&&strcmp(paramsHash,sha256(fullfile(here,'sea_pad_native_params.m')))&&strcmp(translatorHash,sha256(fullfile(here,'sea_pad_native_from_engine.m'))),'Native source changed while executing.');
report=struct('status','native_plant_replay_and_refinement_executed', ...
 'controllerIndependentlyVerified',false,'forceMetric','total axial pad force; no local pressure score', ...
 'cases',rows,'totalNativeBasePointsScored',totalPoints,'refinement',refinement, ...
 'forceParityAllPassed',all([rows.forceParityPassed]),'modelSavedReopened',true, ...
 'modelFile',modelFile,'modelSha256',sha256(modelFile),'contractAtStartSha256',contractHash, ...
 'contractAtEndSha256',sha256(contract),'contractUnchangedDuringReplay',strcmp(contractHash,sha256(contract)), ...
 'nativeBuilderSha256',builderHash,'nativeParamsSha256',paramsHash, ...
 'nativeEngineTranslatorSha256',translatorHash,'nativeDiagramFile',diagramFile,'nativeDiagramStatus',diagramStatus, ...
 'sourceEngineSha256',manifest.engine_source_sha256,'observedTemporaryBytes',tempPeak, ...
 'temporaryReferenceArraysReleased',true,'physicalPlantNative',true, ...
 'savedDefaultCaseIdentity',manifest.cases(1).identity);
if isfield(rows,'originalSplitEventForceParityAtControllerTicksPassed')
 report.originalSplitEventForceParityAtControllerTicksAllPassed=all([rows.originalSplitEventForceParityAtControllerTicksPassed]);
 report.originalSplitEventControllerTickCount=sum([rows.scoredPoints]-1)*p.dt/p.controlStep+numel(rows);
end
write_json(fullfile(out,resultName),report);fprintf('NATIVE_REPLAY_DONE %s\n',jsonencode(report));
end
function finish(storage,record,temp)
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(temp,'owned_sea_pad_native')),try,rmdir(temp,'s');catch,end,end
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
