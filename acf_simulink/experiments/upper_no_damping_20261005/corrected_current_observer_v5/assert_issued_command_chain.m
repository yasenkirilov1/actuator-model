function report=assert_issued_command_chain(mdl)
% Source continuity at every subsystem boundary; no tag visibility inference.
parent=[mdl '/Contact force measurement'];sys=[parent '/Head motion estimator'];
rootController=get_param([mdl '/Force controller'],'PortHandles');plant=get_param([mdl '/Actuator and RABS mechanics'],'PortHandles');
measurement=get_param(parent,'PortHandles');head=get_param(sys,'PortHandles');
parentInput=get_param([parent '/Issued selected actuator request'],'PortHandles');
headInput=get_param([sys '/Issued post-limit current request'],'PortHandles');
lag=get_param([sys '/Identified causal current lag'],'PortHandles');
assert(strcmp(get_param([sys '/Issued post-limit current request'],'BlockType'),'Inport'));
assert(numel(measurement.Inport)==3&&numel(head.Inport)==2);
assert(get_param(get_param(plant.Inport(1),'Line'),'SrcPortHandle')==rootController.Outport(1));
assert(get_param(get_param(measurement.Inport(3),'Line'),'SrcPortHandle')==rootController.Outport(1));
assert(get_param(get_param(head.Inport(2),'Line'),'SrcPortHandle')==parentInput.Outport(1));
assert(get_param(get_param(lag.Inport(1),'Line'),'SrcPortHandle')==headInput.Outport(1));
assert(strcmp(get_param([mdl '/Force controller'],'LabelModeActiveChoice'),'Coil PI and PID'));
report=struct('status','explicit_selected_command_chain_verified','source',[mdl '/Force controller/1'], ...
 'sameSourceAsActualPlantCommand',true,'measurementSubsystemInputPort',3,'headEstimatorInputPort',2, ...
 'sink',[sys '/Identified causal current lag/1'],'rootLocalTagUsed',false, ...
 'physicalPlantTruthUsed',false,'activeController','Coil PI and PID');
end
