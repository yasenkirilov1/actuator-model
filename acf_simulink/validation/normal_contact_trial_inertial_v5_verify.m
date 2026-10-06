function report=normal_contact_trial_inertial_v5_verify()
% Verify identity defaults and exercise the native continuous-dwell detector.
work='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v5-20260930';
folder=fileparts(mfilename('fullpath'));assert(~usejava('desktop'));
addpath(fullfile(work,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator;
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(work,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
set_param(mdl,'AlgebraicLoopMsg','error','FastRestart','off');
report=struct('version','inertial-v5','workFolder',work,'status','running', ...
 'scope','Four complete native identity cases with variation gate disabled and acceleration FF prediction zero, then an enabled native repeated-force gate proof.', ...
 'rows',struct([]));
for j=1:4
 z=load(sprintf('/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v4-20260930/traces/normal_contact_trial_inertial_v4_equivalence_%02d.mat',j));
 ref=z.r;q=ref.parameters;
 q.robot.forceGate.variation=struct('enabled',false,'meanTime',.1,'varianceTime',.02,'rmsLimit_N',.08);
 q.inertial.accelerationPrediction_s=0;q.inertial.accelerationDerivativeFilter_s=.0003;
 r=acf_run(mdl,q,false);
 assert(isequal(r.time,ref.time,r.normalContact.time,r.accelerometer.time,r.motionObserver.time,r.forceVariation.time,r.accelerationFeedforward.time));
 row=struct('caseIndex',j,'massS_kg',q.mechanics.mS,'force_N',q.force.reference, ...
  'peakNormalError_N',max(abs(r.normalContact.data(:,1)-q.force.reference)), ...
  'original35Difference',max(abs(r.data-ref.data),[],'all'), ...
  'normal8Difference',max(abs(r.normalContact.data-ref.normalContact.data),[],'all'), ...
  'accelerometer6Difference',max(abs(r.accelerometer.data-ref.accelerometer.data),[],'all'), ...
  'motion2Difference',max(abs(r.motionObserver.data-ref.motionObserver.data),[],'all'), ...
  'commandPrediction3Difference',max(abs(r.commandPrediction.data-ref.commandPrediction.data),[],'all'), ...
  'robotGate4Difference',max(abs(gate_matrix(r)-gate_matrix(ref)),[],'all'), ...
  'accelerationPredictionIdentityDifference',max(abs(diff(r.accelerationFeedforward.data,1,2))), ...
  'traceFile',fullfile(work,'traces',sprintf('normal_contact_trial_inertial_v5_equivalence_%02d.mat',j)));
 assert(max([row.original35Difference,row.normal8Difference,row.accelerometer6Difference,row.motion2Difference,row.commandPrediction3Difference,row.robotGate4Difference])<1e-9);
 assert(row.accelerationPredictionIdentityDifference==0);
 if isempty(report.rows),report.rows=row;else,report.rows(end+1)=row;end
 save(row.traceFile,'r','row');Simulink.sdi.clear;
 fprintf('V5 IDENTITY %d: maxdiff%.3g, Nerror%.9g.\n',j,max([row.original35Difference,row.normal8Difference,row.accelerometer6Difference,row.motion2Difference,row.commandPrediction3Difference,row.robotGate4Difference]),row.peakNormalError_N);
end
set_param(mdl,'FastRestart','off');
reviewTrace=fullfile(folder,'inertial_uncertainty_finite_provisional','inertial_uncertainty_repeated_screen_20260930_native_traces','case_0001.mat');
z=load(reviewTrace);q=z.r.parameters;
q.robot.forceGate.variation=struct('enabled',true,'meanTime',.1,'varianceTime',.02,'rmsLimit_N',.08);
q.inertial.accelerationPrediction_s=0;q.inertial.accelerationDerivativeFilter_s=.0003;
r=acf_run(mdl,q,false);
assert(isequal(r.time,r.forceVariation.time));stride=round(q.force.sampleTime/q.sim.step);
indices=(1:stride:numel(r.time))';e=r.normalContact.data(indices,4)-q.force.reference;
aMean=exp(-q.force.sampleTime/.1);aVar=exp(-q.force.sampleTime/.02);
meanExpected=zeros(size(e));varianceExpected=zeros(size(e));countExpected=zeros(size(e));stableExpected=false(size(e));
oldGate=gate_matrix(r);slowIndex=floor((r.time(indices)+q.sim.step/2)/q.robot.sampleTime)+1;
heldSlow=oldGate(slowIndex,4)>.5;
previousMean=0;previousVariance=0;previousCount=0;required=1+ceil(q.robot.forceGate.dwellTime/q.force.sampleTime);
for j=1:numel(e)
 meanExpected(j)=aMean*previousMean+(1-aMean)*e(j);
 deviation=e(j)-meanExpected(j);varianceExpected(j)=aVar*previousVariance+(1-aVar)*deviation^2;
 stableExpected(j)=heldSlow(j)&&varianceExpected(j)<=.08^2;
 if stableExpected(j),countExpected(j)=min(required,previousCount+1);else,countExpected(j)=0;end
 previousMean=meanExpected(j);previousVariance=varianceExpected(j);previousCount=countExpected(j);
end
v=r.forceVariation.data(indices,:);
proof=struct('status','native_executed','duration_s',r.time(end),'nativeSamples',numel(r.time), ...
 'meanRecursionResidual_N',max(abs(v(:,1)-meanExpected)), ...
 'varianceRecursionResidual_N2',max(abs(v(:,2).^2-varianceExpected)), ...
 'fastDwellRecursionResidual_samples',max(abs(v(:,3)-countExpected)), ...
 'fastStabilityMismatchCount',nnz((v(:,4)>.5)~=stableExpected), ...
 'readyDwellMismatchCount',nnz((v(:,5)>.5)~=(countExpected>=required)), ...
 'initialMean',v(1,1),'initialRms',v(1,2),'initialCount',v(1,3), ...
 'meanCoefficient',aMean,'varianceCoefficient',aVar,'rmsLimit_N',.08,'requiredDwellSamples',required, ...
 'readyFraction',mean(r.robotGate.ready>.5),'peakNormalForceError_N',max(abs(r.normalContact.data(:,1)-q.force.reference)), ...
 'peakRmsVariation_N',max(v(:,2)),'traceFile',fullfile(work,'traces','normal_contact_trial_inertial_v5_enabled_gate.mat'));
save(fullfile(work,'traces','gate_debug.mat'),'r','proof','v','e','meanExpected','varianceExpected','countExpected','stableExpected','heldSlow','report');
fprintf('V5 GATE DEBUG: meanresidual%.9g variance%.9g count%.9g stablemismatch%d readymismatch%d.\n',proof.meanRecursionResidual_N,proof.varianceRecursionResidual_N2,proof.fastDwellRecursionResidual_samples,proof.fastStabilityMismatchCount,proof.readyDwellMismatchCount);
assert(proof.meanRecursionResidual_N<1e-10 && proof.varianceRecursionResidual_N2<1e-10);
assert(proof.fastDwellRecursionResidual_samples==0 && proof.fastStabilityMismatchCount==0 && proof.readyDwellMismatchCount==0);
assert(any(v(:,3)==0) && any(v(:,3)==required));
proof.gateContractPass=true;report.enabledGateProof=proof;report.status='complete';
save(proof.traceFile,'r','proof');
save(fullfile(folder,'normal_contact_trial_inertial_v5_equivalence.mat'),'report');
fid=fopen(fullfile(folder,'normal_contact_trial_inertial_v5_equivalence.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('V5 ENABLED GATE: Nerror%.9g, RMSmax%.4g, readyFraction%.4g, all native recursion checks passed.\n',proof.peakNormalForceError_N,proof.peakRmsVariation_N,proof.readyFraction);
end

function values=gate_matrix(r)
values=[r.robotGate.ready,r.robotGate.filteredError_N,r.robotGate.forceRate_N_s,r.robotGate.stableNow];
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
