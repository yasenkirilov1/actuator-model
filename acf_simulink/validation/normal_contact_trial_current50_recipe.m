function report=normal_contact_trial_current50_recipe()
% Bounded private native probe of faster identified current response.
source='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v3-20260930/frozen-inertial-v3-helpers1';
work='/home/tommie/.local/state/actuator-model/normal-contact-trial-current50-20260930';
outFolder=fileparts(mfilename('fullpath'));
assert(~usejava('desktop'));
for n={'','helpers','cache','code','sdi','traces'}
 f=fullfile(work,n{1});if ~isfolder(f),mkdir(f);end
end
copyfile(fullfile(source,'acf_force_control_normal_trial.slx'),fullfile(work,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(source,'helpers','*.m'),fullfile(work,'helpers'));
addpath(fullfile(work,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator;
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(work,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
z=load(fullfile(source,'normal_contact_trial_candidate.mat'),'p');p=z.p;
p.mechanics.mS=2;p.mechanics.mB=.6;p.mechanics.mA=.4;p.mechanics.angle=pi/2;
p.mechanics.lowerRigid=false;p.mechanics.upperRigid=true;p.mechanics.k1=64e6;p.mechanics.k2=0;
p.mechanics.c1=0;p.mechanics.c2=0;if isfield(p,'springs'),p=rmfield(p,'springs');end
p.sensor.distalMass_kg=2;p.sensor.distalMass_model_kg=2;
p.sensor.structuralStiffness_N_m=64e6;p.sensor.structuralDamping_Ns_m=0;
p.coil.currentResponse=.00005;p.force.reference=50;p.coil.strategy='pi';
p.inertial.forceKp=.03;p.inertial.forceKi=20;p.inertial.forceKd=0;
p.inertial.currentNotch=struct('enabled',true,'frequency_Hz',sqrt(64e6/.6)/(2*pi),'zeroDamping',.01,'poleDamping',.5);
p.contact.mu=.3;p.accel.noiseRms=.0123;p.accel.quantum=.00468;p.accel.range_g=50;
p.accel.delay=.0001;p.sensor.delay=.0001;p.sim.step=.00001;
p.inertial.accelerationFeedforwardScale=1;p.inertial.guideFrictionFeedforwardScale=1;p.contact.geometryFeedforwardScale=1;
cases=struct([]);
for sensor=[.0001,.0002]
 for guide=[.0003,.0004]
  for noise=[0,1]
   one=struct('sensorFilter_s',sensor,'guidePrediction_s',guide,'zeroDamping',.01,'seed',1729,'clean',noise==0);
   cases=append(cases,one);
  end
 end
end
for seed=[29,1729,7]
 one=struct('sensorFilter_s',.0001,'guidePrediction_s',.0004,'zeroDamping',.001,'seed',seed,'clean',seed==7);
 cases=append(cases,one);
end
report=struct('date','2026-09-30','status','running','sourceFolder',source,'workFolder',work, ...
 'scope','Finite zero-damping64MN/m cell, S2kg/F50N/90deg/mu.3, bounded50us current-response design probe', ...
 'simulationMode','accelerator','plannedInitialCases',numel(cases),'rows',struct([]), ...
 'sensorEnvelope','Identified matched force/acceleration100 or200us response; ADXL1002-like noise/ADC/range assumptions', ...
 'forceMetric','True surface-normal force throughout every native0-4s recording');
set_param(mdl,'SimulationMode','accelerator','AlgebraicLoopMsg','error','FastRestart','on');
start=tic;allTraces=cell(numel(cases),1);
for j=1:numel(cases)
 q=parameters(p,cases(j));[row,r]=run_one(mdl,q,j,work);
 report.rows=append(report.rows,row);allTraces{j}=r;
 report.elapsedSeconds=toc(start);checkpoint(report,outFolder);
 Simulink.sdi.clear;
end
valid=find(strcmp({report.rows.status},'native_executed') & [report.rows.pass]);
if ~isempty(valid)
 [~,ranking]=min([report.rows(valid).peakNormalForceError_N]);best=valid(ranking);
 selected=allTraces{best}.parameters;
 selected.sim.step=.000005;
 set_param(mdl,'FastRestart','off');
 [row,r]=run_one(mdl,selected,numel(report.rows)+1,work);
 row.refinementOfCaseIndex=best;
 row.peakRefinementDifference_N=abs(row.peakNormalForceError_N-report.rows(best).peakNormalForceError_N);
 % Common row fields keep the retained refinement distinguishable.
 for k=1:numel(report.rows)
  report.rows(k).refinementOfCaseIndex=0;report.rows(k).peakRefinementDifference_N=0;
 end
 report.rows=append(report.rows,row);report.selectedRefinementCaseIndex=row.caseIndex;
 save(fullfile(outFolder,'normal_contact_trial_current50_candidate.mat'),'selected');
 report.candidateFile=fullfile(outFolder,'normal_contact_trial_current50_candidate.mat');
end
report.status='complete';report.elapsedSeconds=toc(start);checkpoint(report,outFolder);
end

function q=parameters(p,c)
q=p;q.accel.filterTime=c.sensorFilter_s;q.sensor.filterTime=c.sensorFilter_s;
q.inertial.guideVelocityPrediction_s=c.guidePrediction_s;
q.inertial.currentNotch.zeroDamping=c.zeroDamping;q.sensor.seed=c.seed;
q.sensor.noiseRms=.02;q.contact.lateralNoiseRms_N=.02;q.sensor.strokeNoiseRms=1e-6;q.accel.noiseRms=.0123;
if c.clean,q.sensor.noiseRms=0;q.contact.lateralNoiseRms_N=0;q.sensor.strokeNoiseRms=0;q.accel.noiseRms=0;end
q=acf_prepare(q);q.contact.lateralDelaySamples=q.d.sensorDelaySamples;
end

function [row,r]=run_one(mdl,q,j,work)
row=struct('caseIndex',j,'status','preparation_rejected','massS_kg',q.mechanics.mS,'force_N',q.force.reference, ...
 'mu',q.contact.mu,'cellStiffness_N_m',q.mechanics.k1,'cellDamping_Ns_m',q.mechanics.c1, ...
 'currentResponse_s',q.coil.currentResponse,'sensorFilter_s',q.sensor.filterTime, ...
 'guidePrediction_s',q.inertial.guideVelocityPrediction_s,'notchFrequency_Hz',q.inertial.currentNotch.frequency_Hz, ...
 'notchZeroDamping',q.inertial.currentNotch.zeroDamping,'notchPoleDamping',q.inertial.currentNotch.poleDamping, ...
 'seed',q.sensor.seed,'clean',q.sensor.noiseRms==0,'step_s',q.sim.step,'samples',0, ...
 'peakNormalForceError_N',NaN,'peakErrorTime_s',NaN,'peakCurrent_A',NaN,'peakVoltage_V',NaN, ...
 'contactLoss',false,'strokeStop',false,'currentClipped',false,'voltageClipped',false, ...
 'duration_s',0,'normalBalanceResidual_N',NaN,'pass',false,'traceFile','','rejection','');
r=[];
try
 quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
 assert(isequal(r.time,r.normalContact.time)&&isequal(r.time,r.accelerometer.time)&&isequal(r.time,r.motionObserver.time));
 slope=r.data(:,29)/q.surface.scanSpeed;
 truth=r.data(:,1).*sqrt(1+slope.^2)./(1-q.contact.mu*slope);
 row.normalBalanceResidual_N=max(abs(truth-r.normalContact.data(:,1)));
 assert(row.normalBalanceResidual_N<1e-9 && abs(r.normalContact.data(1,1)-q.force.reference)<1e-9);
 [row.peakNormalForceError_N,at]=max(abs(r.normalContact.data(:,1)-q.force.reference));
 row.peakErrorTime_s=r.time(at);row.samples=numel(r.time);row.status='native_executed';
 row.peakCurrent_A=max(abs(r.data(:,14)));row.peakVoltage_V=max(abs(r.data(:,15)));
 row.contactLoss=any(r.data(:,18)<.5);row.strokeStop=any(r.data(:,21)>.5);
 row.currentClipped=any(r.data(:,22)>.5);row.voltageClipped=any(r.data(:,23)>.5);
 row.duration_s=r.time(end);
 row.pass=row.peakNormalForceError_N<=q.force.tolerance && ~row.contactLoss && ~row.strokeStop && row.duration_s>=q.sim.stopTime-q.sim.step;
 row.traceFile=fullfile(work,'traces',sprintf('normal_contact_trial_current50_%03d.mat',j));
 save(row.traceFile,'r','row');
 fprintf('CURRENT50 %d: filter%.0fus guide%.0fus notchzz%.4g clean%d seed%d => %.9gN at%.6fs, pass%d.\n', ...
  j,q.sensor.filterTime*1e6,q.inertial.guideVelocityPrediction_s*1e6,q.inertial.currentNotch.zeroDamping,row.clean,q.sensor.seed,row.peakNormalForceError_N,row.peakErrorTime_s,row.pass);
catch err
 row.rejection=err.message;fprintf('CURRENT50_REJECTED %d: %s.\n',j,err.message);
end
end

function result=append(result,row)
if isempty(result),result=row;else,result(end+1)=row;end
end
function checkpoint(report,folder)
save(fullfile(folder,'normal_contact_trial_current50_results.mat'),'report');
fid=fopen(fullfile(folder,'normal_contact_trial_current50_results.json'),'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
