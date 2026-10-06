function report=normal_contact_trial_delivery_acceptance(config)
% Final source bridge, direct native Run and physical failure-stop checks.
if nargin<1,config=struct;end
state=fullfile(getenv('HOME'),'.local','state','actuator-model');
if ~isfield(config,'workFolder'),config.workFolder=fullfile(state,'inertial-delivery-private-v8-layoutd-20260930');end
if ~isfield(config,'sourceTrace'),config.sourceTrace=fullfile(state,'normal-contact-trial-inertial-v8-20260930','traces','normal_contact_trial_inertial_v8_pilot_02.mat');end
if ~isfield(config,'resume'),config.resume=false;end
assert(~usejava('desktop'));work=config.workFolder;mdl='acf_force_control_simple';
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
addpath(fullfile(work,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator acf_select_controller acf_parameters;
load_system(fullfile(work,'generated',[mdl '.slx']));mw=get_param(mdl,'ModelWorkspace');base=getVariable(mw,'pRecommended');
report=struct('status','running','sourceBridgePassed',false,'directRunPassed',false, ...
 'guideSlowExamplePassed',false,'contactStopPassed',false,'travelStopPassed',false);
if config.resume&&isfile(fullfile(work,'delivery_acceptance.mat'))
 saved=load(fullfile(work,'delivery_acceptance.mat'),'report');report=saved.report;
end
if isfield(config,'redoSourceBridge')&&config.redoSourceBridge,report.sourceBridgePassed=false;end
if ~isfield(report,'noSpringsPresetPassed'),report.noSpringsPresetPassed=false;end

if ~report.sourceBridgePassed
saved=load(config.sourceTrace,'r');reference=saved.r;p=reference.parameters;
% Use the final approximate spring dimensions in the same retained physical
% case. The packing metadata must not alter any native dynamics.
p.springs=base.springs;r=acf_run(mdl,p,false);
row=compare(r,reference);row.finalSpringFreeLength_m=p.springs.lower.freeLength_m;
assert(row.maximumDifference<1e-9);report.sourceBridge=row;report.sourceBridgePassed=true;
save(fullfile(work,'traces','delivery_final_source_bridge.mat'),'r','reference','row','-v7');
checkpoint(work,report);clear r reference saved;Simulink.sdi.clear;
end

% sim(mdl) runs the native InitFcn just as the Simulink Run button does. Edit
% the workspace structure without calling prepare or a selector beforehand.
if ~report.directRunPassed
p=base;p.mechanics.mS=1.4;p.surface.scanSpeed=.020;p.force.reference=20;
assignin(mw,'p',p);set_param(mdl,'StopTime','.2','FixedStep',num2str(p.sim.step,17));
out=sim(mdl);applied=getVariable(mw,'p');normal=out.get('normal_contact_log');
normalData=native_rows(normal,8);
assert(applied.sensor.distalMass_kg==1.4&&applied.sensor.distalMass_model_kg==1.4);
assert(applied.contact.commandedScanSpeed_m_s==.020);
assert(normal.Time(end)>=.2-p.sim.step/2);
assert(max(abs(normalData(:,1)-20))<=p.force.tolerance);
report.directRun=struct('duration_s',normal.Time(end),'actualToolMass_kg',applied.sensor.distalMass_kg, ...
 'identifiedToolMass_kg',applied.sensor.distalMass_model_kg,'issuedScanSpeed_m_s',applied.contact.commandedScanSpeed_m_s, ...
 'peakNormalError_N',max(abs(normalData(:,1)-20)));report.directRunPassed=true;
save(fullfile(work,'traces','delivery_direct_native_run.mat'),'out','applied','-v7');
checkpoint(work,report);clear out normal normalData applied;Simulink.sdi.clear;
end

% Exact example printed in GETTING_STARTED: B+S=2 kg, 1 mm/60 mm/20 mm/s,
% target20 N, vertical flange, eight seconds including the entire slow bump.
if ~report.guideSlowExamplePassed
p=base;p.mechanics.mS=2-p.mechanics.mB;p.mechanics.angle=0;p.force.reference=20;
p.surface.kind='bump';p.surface.height=.001;p.surface.width=.060;p.surface.scanSpeed=.020;p.surface.startTime=2;
p.sim.stopTime=8;r=acf_run(mdl,p,false);assert(r.metrics.passed&&r.metrics.completed);
report.guideSlowExample=r.metrics;report.guideSlowExamplePassed=true;
save(fullfile(work,'traces','delivery_guide_slow_2kg.mat'),'r','-v7');checkpoint(work,report);clear r;Simulink.sdi.clear;
end

% A larger sharp bump exceeds this loaded actuator's tracking capability.
% Its starting equilibrium remains valid. Retain startup and terminal samples.
if ~report.contactStopPassed
p=base;p.force.reference=50;p.mechanics.angle=pi/2;p.contact.mu=0;p.surface.kind='sharp_bump';
p.surface.startTime=.05;p.surface.height=.005;p.surface.width=.01;p.surface.scanSpeed=.1;p.sim.stopTime=.3;
% Retain the deterministic replay of the first completed guard probe, which
% missed force tracking while remaining in contact. It is not qualification.
if ~isfile(fullfile(work,'traces','delivery_contact_probe_no_loss_replay.mat'))
 r=acf_run(mdl,p,false);save(fullfile(work,'traces','delivery_contact_probe_no_loss_replay.mat'),'r','-v7');
 report.contactProbeWithoutLoss=r.metrics;checkpoint(work,report);clear r;Simulink.sdi.clear;
end
p.surface.width=.002;
r=acf_run(mdl,p,false);
save(fullfile(work,'traces','delivery_contact_stop.mat'),'r','-v7');
assert(r.metrics.lostContact&&~r.metrics.completed&&~r.metrics.passed);
report.contactStop=r.metrics;report.contactStopPassed=true;
checkpoint(work,report);clear r;Simulink.sdi.clear;
end

% An experimental lower travel limit sits just below loaded initial travel.
if ~report.travelStopPassed
p=base;p.force.reference=50;p.mechanics.angle=pi/2;p.surface.kind='sharp_bump';
p.surface.startTime=.1;p.surface.height=.001;p.surface.width=.01;p.surface.scanSpeed=.1;p.sim.stopTime=.4;
p=acf_prepare(p);initialTravel=p.mechanics.midTravel+p.d.u0;
p.mechanics.travelMin=initialTravel-40e-6;
assert(p.mechanics.travelMin<p.mechanics.midTravel);
r=acf_run(mdl,p,false);save(fullfile(work,'traces','delivery_travel_stop.mat'),'r','-v7');
assert(r.metrics.strokeStopHit&&~r.metrics.completed&&~r.metrics.passed);
report.travelStop=r.metrics;report.travelStopPassed=true;
checkpoint(work,report);clear r;Simulink.sdi.clear;
end

if ~report.noSpringsPresetPassed
p=getVariable(mw,'pNoSprings');p.surface.kind='flat';p.sim.stopTime=.2;
r=acf_run(mdl,p,false);
assert(r.metrics.completed&&~r.metrics.lostContact);
assert(~r.parameters.mechanics.lowerRigid&&r.parameters.mechanics.upperRigid&&r.parameters.mechanics.k1==64e6);
assert(~r.parameters.mechanics.externalLowerSpringPair&&~r.parameters.mechanics.externalUpperSpringPair);
assert(~isfield(r.parameters,'springs')&&~isfield(r.metrics,'springSizing')&&~r.parameters.inertial.lowerLoadInverse.enabled);
report.noSpringsPreset=r.metrics;report.noSpringsPresetPassed=true;
save(fullfile(work,'traces','delivery_no_springs_smoke.mat'),'r','-v7');checkpoint(work,report);clear r;Simulink.sdi.clear;
end

assignin(mw,'p',base);base=acf_select_actuator(mdl,'voice_coil');acf_select_controller(mdl,'Feedforward PI');
set_param(mdl,'StopTime',num2str(base.sim.stopTime,17),'FixedStep',num2str(base.sim.step,17));
set_param(mdl,'SimulationCommand','update');assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));
save_system(mdl);close_system(mdl,0);load_system(fullfile(work,'generated',[mdl '.slx']));set_param(mdl,'SimulationCommand','update');
report.saveReopenPassed=true;report.status='complete';checkpoint(work,report);
fprintf('DELIVERY FINAL ACCEPTANCE COMPLETE\n');
end
function row=compare(a,b)
assert(isequal(a.time,b.time));physical=a.data;physical(:,1)=a.axialContact.force_N;
row=struct('duration_s',a.time(end),'nativeSamples',numel(a.time),'physical35Difference',max(abs(physical-b.data),[],'all'), ...
 'maximumDifference',max(abs(physical-b.data),[],'all'),'peakNormalError_N',a.metrics.peakAbsForceError_N);
for name={'normalContact','accelerometer','motionObserver','commandPrediction','forceVariation','measuredDrag','accelerationFeedforward','lowerInverse'}
 assert(isequal(a.(name{1}).time,b.(name{1}).time));d=max(abs(a.(name{1}).data-b.(name{1}).data),[],'all');
 row.([name{1} 'Difference'])=d;row.maximumDifference=max(row.maximumDifference,d);
end
assert(isequal(a.robotGate.time,b.robotGate.time));
gateA=[a.robotGate.ready,a.robotGate.filteredError_N,a.robotGate.forceRate_N_s,a.robotGate.stableNow];
gateB=[b.robotGate.ready,b.robotGate.filteredError_N,b.robotGate.forceRate_N_s,b.robotGate.stableNow];
d=max(abs(gateA-gateB),[],'all');row.robotGateDifference=d;row.maximumDifference=max(row.maximumDifference,d);
end
function checkpoint(work,report)
save(fullfile(work,'delivery_acceptance.mat'),'report');fid=fopen(fullfile(work,'delivery_acceptance.json'),'w');
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
end
function d=native_rows(logged,width)
n=numel(logged.Time);d=logged.Data;
if size(d,1)~=n,d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
