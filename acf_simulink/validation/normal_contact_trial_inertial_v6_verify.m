function report=normal_contact_trial_inertial_v6_verify()
% Native default equivalence and first measured-drag-ratio pilots.
work='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v6-20260930';
folder=fileparts(mfilename('fullpath'));assert(~usejava('desktop'));
addpath(fullfile(work,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator;
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(work,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
set_param(mdl,'AlgebraicLoopMsg','error','FastRestart','off');
report=struct('version','inertial-v6','workFolder',work,'status','running', ...
 'scope','Four default direct-lateral full-history equivalents; two native heavy64MN/m measured-drag-ratio pilots with LP1000Hz and no total-current notch.', ...
 'identityRows',struct([]),'pilotRows',struct([]));
for j=1:4
 z=load(sprintf('/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v5-20260930/traces/normal_contact_trial_inertial_v5_equivalence_%02d.mat',j));
 ref=z.r;q=ref.parameters;q.contact.geometryMode='direct_lateral';q.contact.dragRatioFilter_s=.005;q.contact.dragRatioMinForce_N=1;
 r=acf_run(mdl,q,false);assert(isequal(r.time,ref.time,r.measuredDrag.time));
 row=struct('caseIndex',j,'massS_kg',q.mechanics.mS,'force_N',q.force.reference, ...
  'peakNormalError_N',max(abs(r.normalContact.data(:,1)-q.force.reference)), ...
  'original35Difference',difference(r.data,ref.data), ...
  'normal8Difference',difference(r.normalContact.data,ref.normalContact.data), ...
  'accelerometer6Difference',difference(r.accelerometer.data,ref.accelerometer.data), ...
  'motion2Difference',difference(r.motionObserver.data,ref.motionObserver.data), ...
  'commandPrediction3Difference',difference(r.commandPrediction.data,ref.commandPrediction.data), ...
  'robotGate4Difference',difference(gate_matrix(r),gate_matrix(ref)), ...
  'forceVariation5Difference',difference(r.forceVariation.data,ref.forceVariation.data), ...
  'accelerationFF2Difference',difference(r.accelerationFeedforward.data,ref.accelerationFeedforward.data), ...
  'traceFile',fullfile(work,'traces',sprintf('normal_contact_trial_inertial_v6_identity_%02d.mat',j)));
 assert(max([row.original35Difference,row.normal8Difference,row.accelerometer6Difference,row.motion2Difference,row.commandPrediction3Difference,row.robotGate4Difference,row.forceVariation5Difference,row.accelerationFF2Difference])<1e-9);
 report.identityRows=append(report.identityRows,row);save(row.traceFile,'r','row');Simulink.sdi.clear;
 fprintf('V6 IDENTITY %d: physicaldiff%.3g normaldiff%.3g.\n',j,row.original35Difference,row.normal8Difference);
end
q.mechanics.mS=2;q.mechanics.mB=.6;q.mechanics.mA=.4;q.mechanics.angle=pi/2;
q.mechanics.lowerRigid=false;q.mechanics.upperRigid=true;q.mechanics.k1=64e6;q.mechanics.c1=0;q.mechanics.k2=0;q.mechanics.c2=0;
if isfield(q,'springs'),q=rmfield(q,'springs');end
q.sensor.distalMass_kg=2;q.sensor.distalMass_model_kg=2;q.force.reference=50;q.contact.mu=.3;
q.inertial.forceKp=.03;q.inertial.forceKi=20;q.inertial.forceKd=0;q.coil.strategy='pi';
q.inertial.feedbackFilter=struct('enabled',true,'cutoff_Hz',1000,'damping',1/sqrt(2));
q.inertial.currentNotch.enabled=false;q.inertial.guideVelocityPrediction_s=.0004;
q.inertial.accelerationPrediction_s=0;q.inertial.accelerationDerivativeFilter_s=.0003;
q.inertial.accelerationFeedforwardScale=1;q.inertial.guideFrictionFeedforwardScale=1;q.contact.geometryFeedforwardScale=1;
q.contact.geometryMode='measured_drag_ratio';q.contact.dragRatioFilter_s=.005;q.contact.dragRatioMinForce_N=1;
q.sensor.filterTime=.0001;q.sensor.delay=.0001;q.sensor.strokeDelay=.0001;
q.accel.filterTime=.0001;q.accel.delay=.0001;q.accel.quantum=.00468;q.accel.range_g=50;
q.coil.currentResponse=.0001;q.sim.step=.00001;q.sim.stopTime=4;q.surface.kind='sharp_bump';
q.robot.forceGate.variation.enabled=false;
for j=1:2
 q.sensor.seed=1729;q.sensor.noiseRms=.02;q.contact.lateralNoiseRms_N=.02;q.sensor.strokeNoiseRms=1e-6;q.accel.noiseRms=.0123;
 if j==1,q.sensor.seed=7;q.sensor.noiseRms=0;q.contact.lateralNoiseRms_N=0;q.sensor.strokeNoiseRms=0;q.accel.noiseRms=0;end
 q=acf_prepare(q);q.contact.lateralDelaySamples=q.d.sensorDelaySamples;r=acf_run(mdl,q,false);
 assert(isequal(r.time,r.measuredDrag.time));s=r.normalContact.data(:,5);c=sqrt(1+s.^2);
 expectedRz=q.force.reference*(1-r.measuredDrag.data(:,1).*s)./c;
 stride=round(q.force.sampleTime/q.sim.step);ix=(1:stride:numel(r.time))';
 rz=r.accelerometer.data(ix,5);rx=r.accelerometer.data(ix,6);slope=s(ix);norm=r.normalContact.data(ix,4);
 raw=min(1,max(-1,(-rx-slope.*rz)./max(q.contact.dragRatioMinForce_N,norm.*sqrt(1+slope.^2))));
 valid=norm>q.contact.dragRatioMinForce_N;a=exp(-q.force.sampleTime/q.contact.dragRatioFilter_s);
 estimated=zeros(size(raw));memory=0;
 for k=1:numel(raw)
  if valid(k),estimated(k)=a*memory+(1-a)*raw(k);else,estimated(k)=memory;end
  memory=estimated(k);
 end
 truthSlope=r.data(:,29)/q.surface.scanSpeed;
 nTrue=r.data(:,1).*sqrt(1+truthSlope.^2)./(1-q.contact.mu*truthSlope);
 row=struct('caseIndex',j,'clean',j==1,'seed',q.sensor.seed,'duration_s',r.time(end),'nativeSamples',numel(r.time), ...
  'peakNormalError_N',max(abs(r.normalContact.data(:,1)-q.force.reference)), ...
  'normalBalanceResidual_N',difference(nTrue,r.normalContact.data(:,1)), ...
  'dragRawResidual',difference(raw,r.measuredDrag.data(ix,2)), ...
  'dragEmaResidual',difference(estimated,r.measuredDrag.data(ix,1)), ...
  'dragGuardMismatchCount',nnz(valid~=(r.measuredDrag.data(ix,3)>.5)), ...
  'requestedRzResidual_N',difference(expectedRz,r.normalContact.data(:,6)), ...
  'minEstimatedDragRatio',min(r.measuredDrag.data(:,1)),'maxEstimatedDragRatio',max(r.measuredDrag.data(:,1)), ...
  'contactLoss',any(r.data(:,18)<.5),'strokeStop',any(r.data(:,21)>.5), ...
  'currentClip',any(r.data(:,22)>.5),'voltageClip',any(r.data(:,23)>.5), ...
  'peakCurrent_A',max(abs(r.data(:,14))),'peakVoltage_V',max(abs(r.data(:,15))), ...
  'traceFile',fullfile(work,'traces',sprintf('normal_contact_trial_inertial_v6_pilot_%02d.mat',j)));
 assert(row.normalBalanceResidual_N<1e-9&&row.dragRawResidual<1e-12&&row.dragEmaResidual<1e-12&&row.dragGuardMismatchCount==0&&row.requestedRzResidual_N<1e-9);
 row.normalPass=row.peakNormalError_N<=q.force.tolerance&&~row.contactLoss&&~row.strokeStop&&r.time(end)>=q.sim.stopTime-q.sim.step;
 report.pilotRows=append(report.pilotRows,row);save(row.traceFile,'r','row');Simulink.sdi.clear;
 fprintf('V6 RATIO PILOT %d clean%d: normalpeak%.9gN pass%d; EMAresidual%.3g.\n',j,j==1,row.peakNormalError_N,row.normalPass,row.dragEmaResidual);
end
report.status='complete';save(fullfile(folder,'normal_contact_trial_inertial_v6_equivalence.mat'),'report');
fid=fopen(fullfile(folder,'normal_contact_trial_inertial_v6_equivalence.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
end

function d=difference(a,b),d=max(abs(a-b),[],'all');end
function result=append(result,row),if isempty(result),result=row;else,result(end+1)=row;end,end
function values=gate_matrix(r)
values=[r.robotGate.ready,r.robotGate.filteredError_N,r.robotGate.forceRate_N_s,r.robotGate.stableNow];
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
