function report=verify_local_modal_source_v8(mdl,p,folder)
% Three tiny local-cell wiring checks, never qualification recordings.
parent=[mdl '/Contact force measurement'];head=[parent '/Head motion estimator'];
recon=[parent '/Contact force reconstruction'];modal=[mdl '/Force controller/Coil PI and PID/Cell mode damping'];
target=get_param([recon '/Cell error to deviation'],'PortHandles');source=get_param(get_param(target.Inport(1),'Line'),'SrcPortHandle');
assert(strcmp(get_param([recon '/Cell error to deviation'],'Gain'),'-1'));
stim=[recon '/Owned qualification raw cell stimulus'];
logs={[head '/Owned qualification upper cell log'],[modal '/Owned qualification modal cell log']};
variables={'owned_qualification_upper_cell','owned_qualification_modal_cell'};
oldRecord=Simulink.sdi.getRecordData;
settings=struct('StopTime',get_param(mdl,'StopTime'),'LoadExternalInput',get_param(mdl,'LoadExternalInput'), ...
 'ExternalInput',get_param(mdl,'ExternalInput'),'ReturnWorkspaceOutputs',get_param(mdl,'ReturnWorkspaceOutputs'));
cleanup=onCleanup(@()restore(mdl,recon,stim,logs,target.Inport(1),source,settings,oldRecord)); %#ok<NASGU>
delete_line(recon,source,target.Inport(1));
add_block('simulink/Sources/Constant',stim,'Value','0','SampleTime','p.force.sampleTime','Position',[100 -350 400 -290]);
ports=get_param(stim,'PortHandles');add_line(recon,ports.Outport(1),target.Inport(1),'autorouting','on');
origins={[head '/Cell force deviation from equilibrium'],[modal '/Acquired axial cell deviation']};
for j=1:2
 add_block('simulink/Sinks/To Workspace',logs{j},'VariableName',variables{j}, ...
  'SaveFormat','Timeseries','MaxDataPoints','inf','SampleTime','p.sim.step','Position',[2100 1450 2450 1520]);
 out=get_param(origins{j},'PortHandles');in=get_param(logs{j},'PortHandles');
 existing=get_param(in.Inport(1),'Line');if existing~=-1,delete_line(existing);end
 add_line(get_param(logs{j},'Parent'),out.Outport(1),in.Inport(1),'autorouting','on');
 assert(get_param(get_param(in.Inport(1),'Line'),'SrcPortHandle')==out.Outport(1));
end
Simulink.sdi.setRecordData(true);rows=struct([]);
for raw=[-1,0,1]
 set_param(stim,'Value',num2str(raw,17));
 actions=Simulink.SimulationData.Dataset;
 for j=1:3
  value=double(j==3);ts=timeseries([value;value],[0;3*p.force.sampleTime]);ts=setinterpmethod(ts,'zoh');actions=actions.addElement(ts);
 end
 input=Simulink.SimulationInput(mdl);input=input.setExternalInput(actions);
 input=input.setModelParameter('StopTime',num2str(3*p.force.sampleTime,17),'SimulationMode','normal','ReturnWorkspaceOutputs','on');
 output=sim(input);upper=output.(variables{1});cellMode=output.(variables{2});
 times=double(upper.Time(:));headValues=double(upper.Data(:));modalValues=double(cellMode.Data(:));
 assert(isequal(times,double(cellMode.Time(:))));fresh=times>=p.force.sampleTime-1e-12;expected=-raw;
 assert(any(fresh)&&all(isfinite([headValues;modalValues])));
 upperError=max(abs(headValues(fresh)-expected));modalError=max(abs(modalValues(fresh)-expected));
 assert(upperError<1e-12&&modalError<1e-12);
 row=struct('rawCellError_N',raw,'expectedAcquiredDeltaL_N',expected,'sampleCount',numel(times), ...
  'upperSourceMaximumError_N',upperError,'modalSourceMaximumError_N',modalError, ...
  'times_s',times,'upperSource_N',headValues,'modalSource_N',modalValues);
 if isempty(rows),rows=row;else,rows(end+1)=row;end
 Simulink.sdi.clear;
end
report=struct('status','live_local_cell_and_modal_source_guard_passed','plantQualificationEpisodes',0, ...
 'shortWiringHarnessRuns',3,'durationPerHarness_s',3*p.force.sampleTime,'baselineExternalInput',1, ...
 'rows',rows,'source','Actual local reconstructed acquired-cell deviation, explicit head inport and scoped modal branch', ...
 'scope','Three synthetic source/sign tests. Original source, blocks, parameters and preferences restored before export. No force pass inferred.');
fid=fopen(fullfile(folder,'local_cell_source_guard.json'),'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
end

function restore(mdl,recon,stim,logs,inputPort,source,settings,record)
for j=1:numel(logs),if getSimulinkBlockHandle(logs{j})>0,delete_block(logs{j});end,end
if getSimulinkBlockHandle(stim)>0,delete_block(stim);end
line=get_param(inputPort,'Line');if line~=-1,delete_line(line);end
add_line(recon,source,inputPort,'autorouting','on');
for name=fieldnames(settings)',set_param(mdl,name{1},settings.(name{1}));end
Simulink.sdi.clear;Simulink.sdi.setRecordData(record);
end
