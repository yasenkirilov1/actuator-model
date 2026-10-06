function report=normal_contact_trial_delivery_verify(config)
% Native delivery source equality, known-mass edits and selector contracts.
if nargin<1,config=struct;end
if ~isfield(config,'workFolder'),config.workFolder='/home/tommie/.local/state/actuator-model/inertial-delivery-private-20260930';end
if ~isfield(config,'sourceFolder'),config.sourceFolder='/home/tommie/.local/state/actuator-model/inertial-actuators-frozen-guarded-screw-extension-v7-20260930';end
assert(~usejava('desktop'));
work=config.workFolder;target='acf_force_control_simple';
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@()finish(target,oldSDI,oldRecord)); %#ok<NASGU>
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
report=struct('status','running','workFolder',work,'sourceFolder',config.sourceFolder, ...
    'sourceParityPassed',false,'massEditPassed',false,'nativeStructurePassed',false, ...
    'saveReopenPassed',false,'parityRows',struct([]),'massRows',struct([]));
for j=1:4
    file=sprintf('/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v7-20260930/traces/normal_contact_trial_inertial_v7_identity_%02d.mat',j);
    saved=load(file,'r');reference=saved.r;q=reference.parameters;
    r=delivery_run(work,q);row=compare(r,reference);
    row.caseIndex=j;row.sourceTrace=file;
    save(fullfile(work,'traces',sprintf('delivery_identity_%02d.mat',j)),'r','row','reference');
    disp(row);
    assert(row.maximumDifference<1e-9,'Native identity case differs.');
    report.parityRows=append(report.parityRows,row);
    checkpoint(work,report);Simulink.sdi.clear;
    fprintf('DELIVERY IDENTITY %d maxdiff %.3g\n',j,row.maximumDifference);
end
report.sourceParityPassed=true;
saved=load('/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v7-20260930/traces/normal_contact_trial_inertial_v7_pilot_02.mat','r');
base=saved.r.parameters;
for j=1:3
    masses=[.1,1.4,2];q=base;q.mechanics.mS=masses(j);
    % Leave the previous distal fields untouched on the delivered API.
    stale=[q.sensor.distalMass_kg,q.sensor.distalMass_model_kg];
    qReference=q;qReference.sensor.distalMass_kg=q.mechanics.mS;qReference.sensor.distalMass_model_kg=q.mechanics.mS;
    reference=source_run('/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v7-20260930/frozen-inertial-v7',qReference);r=delivery_run(work,q);
    row=compare(r,reference);row.massS_kg=q.mechanics.mS;row.staleInputMasses_kg=stale;
    row.actualMass_kg=r.parameters.sensor.distalMass_kg;
    row.identifiedMass_kg=r.parameters.sensor.distalMass_model_kg;
    row.distalScale=r.parameters.sensor.distalMassCalibrationScale;
    row.holderScale=r.parameters.inertial.holderMassCalibrationScale;
    row.forceConstantScale=r.parameters.inertial.forceConstantCalibrationScale;
    row.identifiedMovingMass_kg=r.parameters.d.identifiedMovingMass_kg;
    assert(row.actualMass_kg==q.mechanics.mS&&row.identifiedMass_kg==q.mechanics.mS&&row.distalScale==1);
    assert(row.identifiedMovingMass_kg==q.mechanics.mB+q.mechanics.mS&&row.maximumDifference<1e-9);
    report.massRows=append(report.massRows,row);
    save(fullfile(work,'traces',sprintf('delivery_mass_%02d.mat',j)),'r','reference','row');
    checkpoint(work,report);Simulink.sdi.clear;
    fprintf('DELIVERY MASS %.1f kg maxdiff %.3g\n',masses(j),row.maximumDifference);
end
report.massEditPassed=true;
load_delivery(work);mw=get_param(target,'ModelWorkspace');p=getVariable(mw,'p');
original=p;known=acf_prepare(p);
p.sensor.distalMassCalibrationScale=.995;p.inertial.holderMassCalibrationScale=1.01;p.inertial.forceConstantCalibrationScale=1.02;
identified=acf_prepare(p);
report.calibration=struct('actualMass_kg',identified.sensor.distalMass_kg, ...
    'identifiedMass_kg',identified.sensor.distalMass_model_kg, ...
    'actualHoldingCurrent_A',identified.d.coilHoldCurrent, ...
    'identifiedHoldingCurrent_A',identified.d.coilHoldingCurrentCommand_A, ...
    'physicalEquilibriumChange_A',identified.d.coilHoldCurrent-known.d.coilHoldCurrent, ...
    'feedbackGainChange',identified.inertial.forceKp-known.inertial.forceKp);
assert(report.calibration.physicalEquilibriumChange_A==0&&report.calibration.feedbackGainChange==0);
assert(identified.sensor.distalMass_kg==original.mechanics.mS);
% Controller and actuator selection keep the explicit mechanics.
assignin(mw,'p',original);before=original.mechanics;
p=acf_select_controller(target,'pid');assert(strcmp(p.coil.strategy,'pid'));set_param(target,'SimulationCommand','update');
p=acf_select_controller(target,'pi');assert(strcmp(p.coil.strategy,'pi'));set_param(target,'SimulationCommand','update');
p=acf_select_actuator(target,'leadscrew');assert(isequal(p.mechanics,before));set_param(target,'SimulationCommand','update');
assert(strcmp(get_param([target '/Force controller'],'LabelModeActiveChoice'),'Screw normal PI and PID'));
rejected=false;
try,acf_select_actuator(target,'ideal_position');catch err,rejected=contains(err.message,'compliant');end
assert(rejected,'Unsupported ideal topology was silently replaced.');
p=acf_select_actuator(target,'voice_coil');assert(isequal(p.mechanics,before));set_param(target,'SimulationCommand','update');
report.selectors=struct('piCompiled',true,'pidCompiled',true,'screwCompiled',true, ...
    'actuatorPreservedMechanics',true,'idealRigidExplicitlyRejected',rejected);
for name={'p','pRecommended','pNoSprings','pCompliant','pNearContact'}
    if hasVariable(mw,name{1})
        q=getVariable(mw,name{1});assert(~isfield(q.coil,'control')&&~isfield(q.d,'coilControl'));
    end
end
loop=Simulink.BlockDiagram.getAlgebraicLoops(target);assert(isempty(loop));
report.nativeStructurePassed=true;
report.forceScopeTag=get_param([target '/Read workpiece normal force'],'GotoTag');assert(strcmp(report.forceScopeTag,'normal_contact_N'));
close_system(target,0);load_delivery(work);set_param(target,'SimulationCommand','update');
reopened=getVariable(get_param(target,'ModelWorkspace'),'p');
assert(~isfield(reopened.coil,'control')&&~isfield(reopened.d,'coilControl'));
report.saveReopenPassed=true;report.status='complete';checkpoint(work,report);
fprintf('DELIVERY API AND SOURCE VERIFICATION COMPLETE\n');
end

function r=delivery_run(folder,p)
load_delivery(folder);r=acf_run('acf_force_control_simple',p,false);close_system('acf_force_control_simple',0);
end
function load_delivery(folder)
addpath(fullfile(folder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator acf_select_controller acf_parameters;
load_system(fullfile(folder,'generated','acf_force_control_simple.slx'));set_param('acf_force_control_simple','AlgebraicLoopMsg','error','FastRestart','off');
end
function r=source_run(folder,p)
addpath(fullfile(folder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator acf_select_controller acf_parameters;
load_system(fullfile(folder,'acf_force_control_normal_trial.slx'));
set_param('acf_force_control_normal_trial','AlgebraicLoopMsg','error','FastRestart','off');
r=acf_run('acf_force_control_normal_trial',p,false);close_system('acf_force_control_normal_trial',0);
end
function row=compare(a,b)
assert(isequal(a.time,b.time));ax=a.data;bx=b.data;
if isfield(a,'axialContact'),ax(:,1)=a.axialContact.force_N;end
if isfield(b,'axialContact'),bx(:,1)=b.axialContact.force_N;end
row=struct('duration_s',a.time(end),'nativeSamples',numel(a.time), ...
    'peakNormalError_N',max(abs(a.normalContact.data(:,1)-a.parameters.force.reference)), ...
    'physical35Difference',difference(ax,bx),'normal8Difference',difference(a.normalContact.data,b.normalContact.data), ...
    'acceleration6Difference',difference(a.accelerometer.data,b.accelerometer.data), ...
    'motion2Difference',difference(a.motionObserver.data,b.motionObserver.data), ...
    'command3Difference',difference(a.commandPrediction.data,b.commandPrediction.data), ...
    'gate4Difference',difference(gate(a),gate(b)), ...
    'variation5Difference',difference(a.forceVariation.data,b.forceVariation.data), ...
    'drag3Difference',difference(a.measuredDrag.data,b.measuredDrag.data), ...
    'accelerationFF3Difference',difference(a.accelerationFeedforward.data,b.accelerationFeedforward.data));
row.maximumDifference=max([row.physical35Difference,row.normal8Difference,row.acceleration6Difference,row.motion2Difference,row.command3Difference,row.gate4Difference,row.variation5Difference,row.drag3Difference,row.accelerationFF3Difference]);
end
function values=gate(r),values=[r.robotGate.ready,r.robotGate.filteredError_N,r.robotGate.forceRate_N_s,r.robotGate.stableNow];end
function d=difference(a,b),d=max(abs(a-b),[],'all');end
function a=append(a,row),if isempty(a),a=row;else,a(end+1)=row;end,end
function checkpoint(work,report)
save(fullfile(work,'delivery_verification.mat'),'report');
fid=fopen(fullfile(work,'delivery_verification.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
if bdIsLoaded('acf_force_control_normal_trial'),set_param('acf_force_control_normal_trial','FastRestart','off');close_system('acf_force_control_normal_trial',0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
