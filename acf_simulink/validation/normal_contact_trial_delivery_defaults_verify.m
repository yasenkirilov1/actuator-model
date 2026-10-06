function report=normal_contact_trial_delivery_defaults_verify(config)
% Native checks that public scenario edits cannot leave derived measurements stale.
if nargin<1,config=struct;end
if ~isfield(config,'workFolder'),config.workFolder=fullfile(getenv('HOME'),'.local','state','actuator-model','inertial-delivery-private-v8-simpleb-20260930');end
if ~isfield(config,'sourceFolder'),config.sourceFolder=fullfile(getenv('HOME'),'.local','state','actuator-model','normal-contact-trial-inertial-v8-contract-20260930','frozen-inertial-v8');end
assert(~usejava('desktop'));work=config.workFolder;mdl='acf_force_control_simple';
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
cleanup=onCleanup(@()finish(oldSDI,oldRecord)); %#ok<NASGU>
load_delivery(work);base=getVariable(get_param(mdl,'ModelWorkspace'),'p');close_system(mdl,0);
report=struct('status','running','scanEditPassed',false,'delayEditPassed',false,'slowToolExamplePassed',false,'rows',struct([]));
for j=1:3
    p=base;p.surface.kind='sharp_bump';p.surface.height=.001;p.surface.width=.01;p.surface.startTime=2;
    p.sim.stopTime=4;p.sim.evaluateFrom=0;
    if j==1,p.surface.scanSpeed=.02;name='scan_only';
    elseif j==2,p.sensor.delay=2e-4;name='delay_only';
    else,p.mechanics.mS=1.4;p.surface.kind='bump';p.surface.scanSpeed=.02;p.surface.height=.001;p.surface.width=.02;name='slow_tool_B_plus_S_2kg';end
    ref=p;ref.sensor.distalMass_kg=p.mechanics.mS;ref.sensor.distalMass_model_kg=p.mechanics.mS;
    ref.contact.commandedScanSpeed_m_s=p.surface.scanSpeed;
    ref.accel.delay=p.sensor.delay;ref.sensor.strokeDelay=p.sensor.delay;
    ref.contact.lateralDelaySamples=round(p.sensor.delay/p.sensor.sampleTime);
    ref.acquisition.pipelineDelay_s=p.sensor.delay;
    ref.acquisition.identifiedPipelineDelay_s=p.sensor.delay;
    reference=source_run(config.sourceFolder,ref);r=delivery_run(work,p);
    row=compare(r,reference);row.name=name;
    row.actualScanSpeed_m_s=r.parameters.surface.scanSpeed;row.issuedScanSpeed_m_s=r.parameters.contact.commandedScanSpeed_m_s;
    row.masterSensorDelay_s=r.parameters.sensor.delay;row.accelerationDelay_s=r.parameters.accel.delay;
    row.positionDelay_s=r.parameters.sensor.strokeDelay;row.lateralDelaySamples=r.parameters.contact.lateralDelaySamples;
    row.identifiedMotionFrameDelay_s=r.parameters.acquisition.identifiedPipelineDelay_s;
    row.actualDistalMass_kg=r.parameters.sensor.distalMass_kg;row.identifiedDistalMass_kg=r.parameters.sensor.distalMass_model_kg;
    assert(row.maximumDifference<1e-9&&row.actualScanSpeed_m_s==row.issuedScanSpeed_m_s);
    assert(row.masterSensorDelay_s==row.accelerationDelay_s&&row.masterSensorDelay_s==row.positionDelay_s);
    assert(row.lateralDelaySamples*r.parameters.sensor.sampleTime==row.masterSensorDelay_s);
    assert(row.masterSensorDelay_s==row.identifiedMotionFrameDelay_s);
    if j==3,assert(r.parameters.mechanics.mB+r.parameters.mechanics.mS==2&&row.actualDistalMass_kg==1.4&&row.identifiedDistalMass_kg==1.4);end
    report.rows=append(report.rows,row);save(fullfile(work,'traces',['delivery_defaults_' name '.mat']),'r','reference','row','-v7');
    if j==1,report.scanEditPassed=true;elseif j==2,report.delayEditPassed=true;else,report.slowToolExamplePassed=true;end
    checkpoint(work,report);Simulink.sdi.clear;fprintf('DELIVERY DEFAULT %s maxdiff %.3g\n',name,row.maximumDifference);
end
% Deliberate legacy mismatches become explicit scales/offsets, rather than vanish.
load_delivery(work);p=base;p.sensor=rmfield(p.sensor,{'distalMassCalibrationScale','lateralDelayOffsetSamples','strokeDelayOffset_s'});
p.contact=rmfield(p.contact,'commandedScanSpeedScale');p.accel=rmfield(p.accel,'delayOffset_s');
if isfield(p.acquisition,'motionFrameDelayOffset_s'),p.acquisition=rmfield(p.acquisition,'motionFrameDelayOffset_s');end
p.sensor.distalMass_model_kg=.99*p.sensor.distalMass_kg;p.contact.commandedScanSpeed_m_s=1.02*p.surface.scanSpeed;
p.accel.delay=p.sensor.delay+1e-4;p.sensor.strokeDelay=p.sensor.delay+1e-4;p.contact.lateralDelaySamples=p.d.sensorDelaySamples+1;
p.acquisition.identifiedPipelineDelay_s=p.sensor.delay+3e-5;
q=acf_prepare(p);
assert(abs(q.sensor.distalMassCalibrationScale-.99)<1e-12&&abs(q.contact.commandedScanSpeedScale-1.02)<1e-12);
assert(abs(q.accel.delayOffset_s-1e-4)<1e-12&&abs(q.sensor.strokeDelayOffset_s-1e-4)<1e-12&&q.sensor.lateralDelayOffsetSamples==1);
assert(abs(q.acquisition.motionFrameDelayOffset_s-3e-5)<1e-12);
report.intentionalLegacyMismatchPreserved=true;report.status='complete';checkpoint(work,report);
fprintf('DELIVERY DERIVED DEFAULTS VERIFICATION COMPLETE\n');
end
function r=delivery_run(folder,p)
load_delivery(folder);r=acf_run('acf_force_control_simple',p,false);close_system('acf_force_control_simple',0);
end
function load_delivery(folder)
addpath(fullfile(folder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator acf_select_controller;
load_system(fullfile(folder,'generated','acf_force_control_simple.slx'));set_param('acf_force_control_simple','AlgebraicLoopMsg','error','FastRestart','off');
end
function r=source_run(folder,p)
% The frozen helper still prepares an archived, inactive observer. Retain its
% required fields in the reference only; the delivery deleted that synthesis.
saved=load(fullfile(folder,'normal_contact_trial_candidate.mat'),'p');
p.coil.control=saved.p.coil.control;
p.robotPrediction.residualTime=saved.p.robotPrediction.residualTime;
p.robotPrediction.residualJerkPower=saved.p.robotPrediction.residualJerkPower;
addpath(fullfile(folder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator;
load_system(fullfile(folder,'acf_force_control_normal_trial.slx'));set_param('acf_force_control_normal_trial','AlgebraicLoopMsg','error','FastRestart','off');
r=acf_run('acf_force_control_normal_trial',p,false);close_system('acf_force_control_normal_trial',0);
end
function row=compare(a,b)
assert(isequal(a.time,b.time));ax=a.data;bx=b.data;if isfield(a,'axialContact'),ax(:,1)=a.axialContact.force_N;end
if isfield(b,'axialContact'),bx(:,1)=b.axialContact.force_N;end
row=struct('duration_s',a.time(end),'maximumDifference',max(abs(ax-bx),[],'all'),'peakNormalError_N',max(abs(a.normalContact.data(:,1)-a.parameters.force.reference)));
names={'normalContact','accelerometer','motionObserver','commandPrediction','forceVariation','measuredDrag','accelerationFeedforward','lowerInverse'};
for j=1:numel(names),if isfield(a,names{j})&&isfield(b,names{j}),d=max(abs(a.(names{j}).data-b.(names{j}).data),[],'all');row.([names{j} 'Difference'])=d;row.maximumDifference=max(row.maximumDifference,d);end,end
end
function a=append(a,row),if isempty(a),a=row;else,a(end+1)=row;end,end
function checkpoint(work,report)
save(fullfile(work,'delivery_defaults_verification.mat'),'report');fid=fopen(fullfile(work,'delivery_defaults_verification.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
end
function finish(oldSDI,oldRecord)
for mdl={'acf_force_control_simple','acf_force_control_normal_trial'},if bdIsLoaded(mdl{1}),set_param(mdl{1},'FastRestart','off');close_system(mdl{1},0);end,end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
