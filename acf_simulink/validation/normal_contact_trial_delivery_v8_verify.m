function report=normal_contact_trial_delivery_v8_verify(config)
% Bind the assembled public workflow to the immutable qualified V8 physics.
if nargin<1,config=struct;end
state=fullfile(getenv('HOME'),'.local','state','actuator-model');
if ~isfield(config,'workFolder'),config.workFolder=fullfile(state,'inertial-delivery-private-v8d-20260930');end
if ~isfield(config,'testedFolder'),config.testedFolder=fullfile(state,'normal-contact-trial-inertial-v8-20260930');end
assert(~usejava('desktop'));work=config.workFolder;mdl='acf_force_control_simple';
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
report=struct('status','running','sourceParityPassed',false,'massEditPassed',false,'nativeStructurePassed',false,'saveReopenPassed',false,'parityRows',struct([]));
for index=[2,6,9]
    saved=load(fullfile(config.testedFolder,'traces',sprintf('normal_contact_trial_inertial_v8_pilot_%02d.mat',index)),'r');reference=saved.r;
    load_delivery(work);r=acf_run(mdl,reference.parameters,false);close_system(mdl,0);
    row=compare(r,reference);row.caseIndex=index;report.parityRows=append(report.parityRows,row);
    save(fullfile(work,'traces',sprintf('delivery_v8_identity_%02d.mat',index)),'r','reference','row','-v7');
    checkpoint(work,report);assert(row.maximumDifference<1e-9,'The assembled delivery changes the frozen V8 native history.');
    fprintf('DELIVERY V8 IDENTITY %d maxdiff %.3g\n',index,row.maximumDifference);Simulink.sdi.clear;
end
report.sourceParityPassed=true;
load_delivery(work);mw=get_param(mdl,'ModelWorkspace');p=getVariable(mw,'p');
for mass=[.1,1.4,2]
    q=p;q.mechanics.mS=mass;q=acf_prepare(q);
    assert(q.sensor.distalMass_kg==mass&&q.sensor.distalMass_model_kg==mass&&q.sensor.distalMassCalibrationScale==1);
end
report.massEditPassed=true;
q=p;q.mechanics.k1=1.1*q.mechanics.k1;q.springs.lower.rate_N_m=q.mechanics.k1/2;q.mechanics.c1=1.1*q.mechanics.c1;q=acf_prepare(q);
assert(q.inertial.lowerLoadInverse.identifiedStiffness_N_m==q.mechanics.k1&&q.inertial.lowerLoadInverse.identifiedDamping_Ns_m==q.mechanics.c1);
q=p;q.inertial.holderMassCalibrationScale=.995;q.inertial.forceConstantCalibrationScale=1.005;q=acf_prepare(q);
assert(q.d.coilHoldCurrent==p.d.coilHoldCurrent&&q.inertial.lowerLoadInverse.identifiedMassB_kg==p.mechanics.mB*.995);
assert(q.inertial.lowerLoadInverse.identifiedForceConstant_N_A==p.coil.forceConstant*1.005);
before=p.mechanics;
for strategy={'PI','PID','Feedforward PI'}
    selected=acf_select_controller(mdl,strategy{1});set_param(mdl,'SimulationCommand','update');
    assert(isequal(selected.mechanics,before));
    assert(selected.force.feedforwardEnabled==strcmp(strategy{1},'Feedforward PI'));
end
selected=acf_select_actuator(mdl,'leadscrew');assert(isequal(selected.mechanics,before));set_param(mdl,'SimulationCommand','update');
assert(strcmp(get_param([mdl '/Force controller'],'LabelModeActiveChoice'),'Screw normal PI and PID'));
assert(abs(selected.d.screwLowerLoadInverse.identifiedMassB_kg-(selected.d.identifiedHolderMass_kg+selected.screw.rotorInertia/selected.d.ell^2))<1e-12);
rejected=false;try,acf_select_actuator(mdl,'ideal_position');catch err,rejected=contains(err.message,'compliant');end
assert(rejected);acf_select_actuator(mdl,'voice_coil');acf_select_controller(mdl,'Feedforward PI');
for variable={'p','pRecommended','pNoSprings','pCompliant','pNearContact'}
    if hasVariable(mw,variable{1}),one=getVariable(mw,variable{1});assert(~isfield(one.coil,'control')&&~isfield(one.d,'coilControl'));end
end
one=getVariable(mw,'pNoSprings');assert(~isfield(one,'springs')&&one.mechanics.k1==64e6&&one.mechanics.c1==0&&~one.inertial.lowerLoadInverse.enabled);
assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));report.nativeStructurePassed=true;
assert(strcmp(get_param([mdl '/Read workpiece normal force'],'GotoTag'),'normal_contact_N'));
close_system(mdl,0);load_delivery(work);set_param(mdl,'SimulationCommand','update');report.saveReopenPassed=true;
report.status='complete';checkpoint(work,report);fprintf('DELIVERY V8 SOURCE AND API VERIFICATION COMPLETE\n');
end
function load_delivery(folder)
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator acf_select_controller;
load_system(fullfile(folder,'generated','acf_force_control_simple.slx'));set_param('acf_force_control_simple','FastRestart','off','AlgebraicLoopMsg','error');
end
function row=compare(a,b)
assert(isequal(a.time,b.time));ax=a.data;if isfield(a,'axialContact'),ax(:,1)=a.axialContact.force_N;end
row=struct('duration_s',a.time(end),'nativeSamples',numel(a.time),'physical35Difference',max(abs(ax-b.data),[],'all'),'maximumDifference',max(abs(ax-b.data),[],'all'));
for name={'normalContact','accelerometer','motionObserver','commandPrediction','forceVariation','measuredDrag','accelerationFeedforward','lowerInverse'}
    assert(isequal(a.(name{1}).time,b.(name{1}).time));d=max(abs(a.(name{1}).data-b.(name{1}).data),[],'all');row.([name{1} 'Difference'])=d;row.maximumDifference=max(row.maximumDifference,d);
end
row.peakNormalError_N=max(abs(a.normalContact.data(:,1)-a.parameters.force.reference));
end
function a=append(a,row),if isempty(a),a=row;else,a(end+1)=row;end,end
function checkpoint(work,report)
save(fullfile(work,'delivery_verification.mat'),'report');fid=fopen(fullfile(work,'delivery_verification.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
