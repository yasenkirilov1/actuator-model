function report=normal_contact_trial_v8_verify(config)
% Retained source pairs and a small native actual-acquisition endpoint cohort.
if nargin<1,config=struct;end
if ~isfield(config,'workFolder'),config.workFolder=fullfile(getenv('HOME'),'.local','state','actuator-model','normal-contact-trial-inertial-v8-20260930');end
if ~isfield(config,'sourceFolder'),config.sourceFolder=fullfile(getenv('HOME'),'.local','state','actuator-model','inertial-mechanics-selected-lower-series-20260930','lower_series_v1');end
if ~isfield(config,'runIdentity'),config.runIdentity=true;end
if ~isfield(config,'runPilot'),config.runPilot=true;end
assert(~usejava('desktop'));mdl='acf_force_control_normal_trial';
work=config.workFolder;loaded=load(fullfile(work,'normal_contact_trial_candidate.mat'),'p');base=loaded.p;
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
report=struct('status','running','sourceFolder',config.sourceFolder,'workFolder',work, ...
    'identityPassed',false,'identityRows',struct([]),'pilotRows',struct([]));
if ~config.runIdentity&&isfile(fullfile(work,'v8_verification.json'))
    earlier=jsondecode(fileread(fullfile(work,'v8_verification.json')));
    if ~isempty(earlier.identityRows)
        report.identityRows=earlier.identityRows;
        report.identityPassed=all([earlier.identityRows.maximumDifference]<1e-10);
    end
end
if config.runIdentity
    specs=[2,50,90,0;.1,5,0,0;2,50,90,29;.1,5,0,1729];
    for j=1:size(specs,1)
        q=case_parameters(base,specs(j,:),false);q.sim.stopTime=4;
        reference=run_private(config.sourceFolder,q);r=run_private(work,q);
        row=compare(r,reference);row.caseIndex=j;row.spec=specs(j,:);
        report.identityRows=append(report.identityRows,row);
        save(fullfile(work,'traces',sprintf('normal_contact_trial_inertial_v8_identity_%02d.mat',j)),'r','reference','row','-v7');
        checkpoint(work,report);assert(row.maximumDifference<1e-10,'The disabled V8 acquisition changed its frozen source.');
        Simulink.sdi.clear;fprintf('V8 IDENTITY %d maxdifference %.3g\n',j,row.maximumDifference);
    end
    report.identityPassed=true;checkpoint(work,report);
end
if config.runPilot
    % Eight mass/force/orientation endpoints, plus a first clean heavy check.
    specs=[2,50,90,0;2,50,90,29;2,50,0,29;2,5,90,29;2,5,0,29; ...
        .1,50,90,29;.1,50,0,29;.1,5,90,29;.1,5,0,29];
    if isfield(config,'pilotSpecs'),specs=config.pilotSpecs;end
    for j=1:size(specs,1)
        q=case_parameters(base,specs(j,:),true);q.sim.stopTime=10;q.surface.kind='repeated';
        r=run_private(work,q);row=score(r);row.caseIndex=j;row.spec=specs(j,:);
        report.pilotRows=append(report.pilotRows,row);
        save(fullfile(work,'traces',sprintf('normal_contact_trial_inertial_v8_pilot_%02d.mat',j)),'r','row','-v7');
        checkpoint(work,report);Simulink.sdi.clear;
        fprintf('V8 ACTUAL AA %d S%.1f F%.0f angle%.0f seed%d: %.6f N pass%d\n',j,specs(j,:),row.peakNormalError_N,row.passed);
    end
end
report.status='complete';checkpoint(work,report);fprintf('V8 SOURCE AND ACQUISITION PROBE COMPLETE\n');
end

function q=case_parameters(q,spec,enabled)
q.mechanics.mS=spec(1);q.force.reference=spec(2);q.mechanics.angle=spec(3)*pi/180;
q.sensor.distalMass_kg=spec(1);q.sensor.distalMass_model_kg=spec(1);
q.sensor.seed=max(1,spec(4));q.acquisition.enabled=enabled;
q.surface.kind='sharp_bump';q.surface.height=.001;q.surface.width=.01;q.surface.scanSpeed=.1;
q.contact.commandedScanSpeed_m_s=.1;q.contact.mu=.3;q.sim.evaluateFrom=0;
if enabled
    q.accel.noiseRms=.014;q.accel.nominalNoiseRms=.014;
    q.robot.forceGate.enabled=true;q.robot.forceGate.variation.enabled=true;
end
if spec(4)==0,q.sensor.noiseRms=0;q.contact.lateralNoiseRms_N=0;q.accel.noiseRms=0;q.sensor.strokeNoiseRms=0;end
end
function r=run_private(folder,p)
mdl='acf_force_control_normal_trial';if bdIsLoaded(mdl),close_system(mdl,0);end
addpath(fullfile(folder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator;
load_system(fullfile(folder,[mdl '.slx']));set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error');
r=acf_run(mdl,p,false);close_system(mdl,0);
end
function row=compare(a,b)
assert(isequal(a.time,b.time));names={'normalContact','accelerometer','motionObserver','commandPrediction','forceVariation','measuredDrag','accelerationFeedforward','lowerInverse'};
row=struct('duration_s',a.time(end),'physical35Difference',difference(a.data,b.data),'maximumDifference',difference(a.data,b.data));
for j=1:numel(names)
    assert(isequal(a.(names{j}).time,b.(names{j}).time));d=difference(a.(names{j}).data,b.(names{j}).data);
    row.([names{j} 'Difference'])=d;row.maximumDifference=max(row.maximumDifference,d);
end
d=difference(gate(a),gate(b));row.gateDifference=d;row.maximumDifference=max(row.maximumDifference,d);
row.peakNormalError_N=max(abs(a.normalContact.data(:,1)-a.parameters.force.reference));
end
function row=score(r)
p=r.parameters;n=r.normalContact.data;t=r.time;d=r.data;
assert(isequal(t,r.normalContact.time,r.accelerometer.time,r.motionObserver.time,r.lowerInverse.time));
assert(abs(n(1,1)-p.force.reference)<1e-9,'Loaded contact startup is incorrect.');
balance=max(abs(n(:,1)-(d(:,1)-n(:,3).*n(:,2))./sqrt(1+n(:,3).^2)));
assert(balance<1e-9,'Native contact vector balance is incorrect.');
e=n(:,1)-p.force.reference;[peak,idx]=max(abs(e));
flat=t>.5&t<1.9;
row=struct('duration_s',t(end),'samples',numel(t),'completed',t(end)>=p.sim.stopTime-p.sim.step, ...
    'peakNormalError_N',peak,'peakTime_s',t(idx),'normalVectorResidual_N',balance, ...
    'lostContact',any(d(:,18)<.5),'travelStop',any(d(:,21)>.5), ...
    'peakCurrent_A',max(abs(d(:,14))),'currentClipped',any(d(:,22)>.5),'voltageClipped',any(d(:,23)>.5), ...
    'springPackingPassed',r.metrics.springSizing.passed,'springPreload_N',p.springs.commonPreload_N, ...
    'combinedFreeLength_mm',r.metrics.springSizing.combinedFreeLengthMax_mm, ...
    'combinedInstalledLength_mm',r.metrics.springSizing.combinedInstalledLengthMax_mm, ...
    'flatAccelerationNoiseRms_m_s2',std(r.accelerometer.data(flat,1)), ...
    'flatLateralNoiseRms_N',std(n(flat,7)),'flatNormalEstimateNoiseRms_N',std(n(flat,4)), ...
    'range_g',p.accel.range_g,'quantum_m_s2',p.accel.quantum,'pipeline_s',p.acquisition.pipelineDelay_s, ...
    'rawPeriod_s',p.sensor.sampleTime,'controllerPeriod_s',p.force.sampleTime);
assert(p.springs.commonPreload_N==33.6,'Check the actual stored spring preload.');
row.passed=row.completed&&peak<=p.force.tolerance&&~row.lostContact&&~row.travelStop&&row.peakCurrent_A<=p.coil.peakCurrent*(1+1e-4)&&row.springPackingPassed;
end
function x=gate(r),x=[r.robotGate.ready,r.robotGate.filteredError_N,r.robotGate.forceRate_N_s,r.robotGate.stableNow];end
function d=difference(a,b),d=max(abs(a-b),[],'all');end
function a=append(a,row),if isempty(a),a=row;else,a(end+1)=row;end,end
function checkpoint(work,report)
save(fullfile(work,'v8_verification.mat'),'report');fid=fopen(fullfile(work,'v8_verification.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
