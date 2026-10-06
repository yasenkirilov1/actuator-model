function report=verify_local_cell_source(mdl,p,folder)
% Numerically prove the installed observer consumes the acquired local cell.
% Temporary raw-cell-error perturbation is a wiring harness, not a plant run.
parent=[mdl '/Contact force measurement'];sys=[parent '/Head motion estimator'];
recon=[parent '/Contact force reconstruction'];
assert(strcmp(get_param([recon '/Cell error to deviation'],'Gain'),'-1'));
target=get_param([recon '/Cell error to deviation'],'PortHandles');inputLine=get_param(target.Inport(1),'Line');
sourcePort=get_param(inputLine,'SrcPortHandle');delete_line(recon,sourcePort,target.Inport(1));
stim=[recon '/Owned raw cell error wiring stimulus'];
log=[sys '/Owned selected cell source wiring log'];
add_block('simulink/Sources/Constant',stim,'Value','0','SampleTime','p.force.sampleTime','Position',[100 -350 400 -290]);
stimPorts=get_param(stim,'PortHandles');add_line(recon,stimPorts.Outport(1),target.Inport(1),'autorouting','on');
add_block('simulink/Sinks/To Workspace',log,'VariableName','owned_cell_source_guard', ...
 'SaveFormat','Timeseries','MaxDataPoints','inf','SampleTime','p.sim.step','Position',[2100 1450 2450 1520]);
add_line(sys,'Cell force deviation from equilibrium/1','Owned selected cell source wiring log/1','autorouting','on');
oldRecord=Simulink.sdi.getRecordData;Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@()restore(recon,stim,log,target.Inport(1),sourcePort,oldRecord)); %#ok<NASGU>
rows=struct([]);
for rawError=[-1,0,1]
 set_param(stim,'Value',num2str(rawError,17));
 input=[0,0,0,1;3*p.force.sampleTime,0,0,1];
 assignin('base','owned_cell_source_guard_input',input);
 output=sim(mdl,'StopTime',num2str(3*p.force.sampleTime,17),'SimulationMode','normal', ...
  'LoadExternalInput','on','ExternalInput','owned_cell_source_guard_input','ReturnWorkspaceOutputs','on');
 data=output.owned_cell_source_guard;actual=double(data.Data(:));times=double(data.Time(:));expected=-rawError;
 fresh=times>=p.force.sampleTime-1e-12;
 if isempty(actual),maxError=inf;first=[];last=[];
 else,maxError=max(abs(actual(fresh)-expected));first=actual(1);last=actual(end);end
 row=struct('injectedRawCellError_N',rawError,'expectedSelectedDeltaL_N',expected, ...
  'sampleCount',numel(actual),'firstSelectedDeltaL_N',first,'lastSelectedDeltaL_N',last, ...
  'checkedFromTime_s',p.force.sampleTime,'maximumAbsoluteWiringError_N',maxError, ...
  'actualTimes_s',times,'actualSelectedDeltaL_N',actual);
 if isempty(rows),rows=row;else,rows(end+1)=row;end
 evidence=struct('status','running_local_source_guard','rows',rows,'privateRecordingEnabled',true);
 fid=fopen(fullfile(folder,'local_cell_source_guard_progress.json'),'w');assert(fid>=0);
 fprintf(fid,'%s\n',jsonencode(evidence,'PrettyPrint',true));fclose(fid);
 assert(~isempty(actual)&&any(fresh)&&all(isfinite(actual))&&maxError<1e-12, ...
  'Installed upper observer local acquired-cell source failed its nonzero perturbation/sign guard.');
end
report=struct('status','live_local_cell_source_sign_and_nonzero_guard_passed', ...
 'plantQualificationEpisodes',0,'shortWiringHarnessRuns',3,'durationPerHarness_s',3*p.force.sampleTime, ...
 'source','Actual installed Contact force reconstruction/Cell error to deviation output via explicit subsystem connection', ...
 'selectedSink','Actual installed Head motion estimator/Cell force deviation from equilibrium driving observer', ...
 'sourceTagAmbiguity',false,'rows',rows, ...
 'scope','Temporary raw acquired-cell-error injection verifies active source/sign and zero equilibrium, then restores every original model connection before native export. These tiny wiring-harness runs are excluded from physical force qualification.');
fid=fopen(fullfile(folder,'local_cell_source_guard.json'),'w');assert(fid>=0);
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
end
function restore(recon,stim,log,inputPort,sourcePort,oldRecord)
if getSimulinkBlockHandle(log)>0,delete_block(log);end
if getSimulinkBlockHandle(stim)>0,delete_block(stim);end
currentLine=get_param(inputPort,'Line');if currentLine~=-1,delete_line(currentLine);end
add_line(recon,sourcePort,inputPort,'autorouting','on');
evalin('base','clear owned_cell_source_guard_input');
Simulink.sdi.clear;Simulink.sdi.setRecordData(oldRecord);
end
