function report=verify_issued_command_source(mdl,p,folder)
% Tiny wiring harness: actual selected post-limit commands reach current lag.
assert_issued_command_chain(mdl);
sys=[mdl '/Contact force measurement/Head motion estimator'];
names={[mdl '/Owned selected issued-current wiring log'],[sys '/Owned current-lag input wiring log'],[sys '/Owned predicted-current response wiring log']};
variables={'owned_selected_issued_current','owned_current_lag_input','owned_predicted_current_response'};
sources={[mdl '/Force controller'],[sys '/Issued post-limit current request'],[sys '/Identified causal current lag']};
oldRecord=Simulink.sdi.getRecordData;Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@()restore(names,oldRecord)); %#ok<NASGU>
for j=1:3
 add_block('simulink/Sinks/To Workspace',names{j},'VariableName',variables{j}, ...
  'SaveFormat','Timeseries','MaxDataPoints','inf','SampleTime','p.sim.step','Position',[20 -600-100*j 400 -550-100*j]);
 src=get_param(sources{j},'PortHandles');dst=get_param(names{j},'PortHandles');
 add_line(get_param(names{j},'Parent'),src.Outport(1),dst.Inport(1),'autorouting','on');
end
holdCurrent=p.d.upperMotionObserver.forcePredictionInitial_A;rows=struct([]);
for offset=[0,.02,-.02]
 times=[0;3*p.force.sampleTime];values=[holdCurrent+offset,0,0;holdCurrent+offset,0,0];
 dataset=Simulink.SimulationData.Dataset;
 for j=1:3
  ts=timeseries(values(:,j),times);ts=setinterpmethod(ts,'zoh');dataset=dataset.addElement(ts);
 end
 in=Simulink.SimulationInput(mdl);in=in.setExternalInput(dataset);
 in=in.setModelParameter('StopTime',num2str(times(end),17),'SimulationMode','normal','ReturnWorkspaceOutputs','on');
 output=sim(in);source=output.owned_selected_issued_current;input=output.owned_current_lag_input;prediction=output.owned_predicted_current_response;
 sourceValues=double(source.Data(:));inputValues=double(input.Data(:));predictedValues=double(prediction.Data(:));t=double(source.Time(:));
 assert(~isempty(t)&&isequal(source.Time,input.Time)&&isequal(source.Time,prediction.Time));
 maximumError=max(abs(sourceValues-inputValues));assert(maximumError<1e-12&&all(isfinite([sourceValues;inputValues;predictedValues])));
 row=struct('requestedOffset_A',offset,'knownHoldingCurrent_A',holdCurrent,'nativeSamples',numel(t), ...
  'maximumSourceToLagInputDifference_A',maximumError,'lastSelectedIssuedRequest_A',sourceValues(end), ...
  'lastPredictedCurrent_A',predictedValues(end),'times_s',t,'selectedIssuedRequest_A',sourceValues, ...
  'lagInput_A',inputValues,'predictedCurrent_A',predictedValues);
 if isempty(rows),rows=row;else,rows(end+1)=row;end
end
assert(max(abs(rows(1).selectedIssuedRequest_A-holdCurrent))<1e-10);
assert(rows(2).lastSelectedIssuedRequest_A-rows(1).lastSelectedIssuedRequest_A>1e-5);
assert(rows(3).lastSelectedIssuedRequest_A-rows(1).lastSelectedIssuedRequest_A< -1e-5);
assert(rows(2).lastPredictedCurrent_A-rows(1).lastPredictedCurrent_A>1e-6);
assert(rows(3).lastPredictedCurrent_A-rows(1).lastPredictedCurrent_A< -1e-6);
report=struct('status','live_selected_command_to_current_lag_guard_passed','physicalQualificationEpisodes',0, ...
 'shortWiringHarnessRuns',3,'durationPerHarness_s',3*p.force.sampleTime,'rows',rows, ...
 'source','Actual selected Force controller/1 after native current notch/limits; baseline selector external mode0', ...
 'sink','Explicit Head motion estimator/Inport2 feeding Identified causal current lag/1', ...
 'inputEqualityTolerance_A',1e-12,'currentPredictionRespondsToBothSigns',true, ...
 'scope','No source wire substitution: small actual external current requests exercise selected native driver-command path. Temporary loggers and private recording preference are removed before saved export. No plant-truth input enters observer. Harness runs are not physical force qualification.');
fid=fopen(fullfile(folder,'live_issued_command_guard.json'),'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
end
function restore(names,oldRecord)
for j=1:numel(names),if getSimulinkBlockHandle(names{j})>0,delete_block(names{j});end,end
Simulink.sdi.clear;Simulink.sdi.setRecordData(oldRecord);
end
