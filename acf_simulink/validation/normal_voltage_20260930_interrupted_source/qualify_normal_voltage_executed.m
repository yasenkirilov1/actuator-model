function report=qualify_normal_voltage(sourceWorker,candidateFile,phase)
%QUALIFY_NORMAL_VOLTAGE Paired native electrical tests of normal-force control.
% Copies a build_normal_contact_trial worker and never changes that source.
% Only the private copy receives actual winding mismatch gains. MATLAB -batch.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch session.');
assert(nargin>=1 && isfolder(sourceWorker),'A completed normal trial worker is required.');
if nargin<2,candidateFile=fullfile(sourceWorker,'normal_contact_trial_candidate.mat');end
if nargin<3,phase='baseline';end
assert(any(strcmp(phase,{'baseline','final_confirmation'})),'Unknown electrical-study phase.');
maxNumCompThreads(2);project=fileparts(fileparts(fileparts(mfilename('fullpath'))));
prefix=fullfile(project,'acf_simulink','validation',['normal_voltage_20260930_' phase]);
assert(~isfile([prefix '.json']),'Retain previous evidence; do not overwrite a completed/running study.');
assert(~isfile([prefix '_stop.request']),'Remove the old study stop request before starting a new run.');
sourceModel=fullfile(sourceWorker,'acf_force_control_normal_trial.slx');
assert(isfile(sourceModel) && isfile(candidateFile),'The normal trial/candidate is not ready.');
worker=tempname;mkdir(worker);copyfile(fullfile(sourceWorker,'helpers'),fullfile(worker,'helpers'));
modelFile=fullfile(worker,'acf_force_control_normal_trial.slx');copyfile(sourceModel,modelFile);
candidateSnapshot=fullfile(worker,'selected_candidate.mat');copyfile(candidateFile,candidateSnapshot);
addpath(fullfile(worker,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator;
loaded=load(candidateSnapshot,'p');base=loaded.p;
assert(isfield(base,'contact'),'Candidate must contain the causal normal-contact controller.');
assert(base.mechanics.upperRigid);
if strcmp(phase,'baseline')
 assert(base.mechanics.lowerRigid);
else
 assert(base.mechanics.lowerRigid || (base.mechanics.k1>0 && base.mechanics.c1>=0), ...
     'Final mechanics must retain a rigid or identified finite B-S connection.');
end
assert(abs(base.mechanics.mB-.6)<1e-12 && strcmp(base.sensor.positionTarget,'tool_relative_A'));
assert(base.robot.forceGate.enabled && base.robot.forceGate.filterTime>0);
if strcmp(phase,'baseline'),assert(abs(base.robot.forceGate.filterTime-.1)<1e-12);end
assert(abs(base.coil.forceConstant-20)<1e-12 && abs(base.coil.resistance-4)<1e-12 && ...
    abs(base.coil.inductance-.003)<1e-12);
if strcmp(phase,'baseline'),assert(abs(base.coil.currentResponse-.0001)<1e-12);end
assert(abs(base.force.sampleTime-.0001)<1e-12 && base.coil.directCoulombCorrection);
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.sim.step=.00005;
if strcmp(phase,'final_confirmation'),base.sim.step=.000010;end
base.surface.kind='sharp_bump';base.surface.height=.001;base.surface.width=.01;
base.surface.scanSpeed=.1;base.surface.startTime=2;
if strcmp(phase,'baseline')
 base.sensor.noiseRms=.02;base.sensor.strokeNoiseRms=1e-6;
 base.contact.lateralNoiseRms_N=.02;base.contact.lateralBias_N=0;
 base.contact.lateralDelaySamples=1;
end
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
previousGen=Simulink.fileGenControl('getConfig');mdl='acf_force_control_normal_trial';
cleanup=onCleanup(@()finish(mdl,worker,previousStorage,previousRecord,previousGen));
mkdir(fullfile(worker,'sdi'));
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(worker,'sdi'));Simulink.sdi.setRecordData(true);
load_system(modelFile);assert(strcmp(get_param(mdl,'FileName'),modelFile));
set_param(mdl,'FastRestart','off');mw=get_param(mdl,'ModelWorkspace');
drive=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil'];
gainNames={'Coil force constant','Back EMF','Resistance','Inverse inductance','Current regulator'};
gainExpressions={'p.coil.forceConstant*p.windingActual.forceFactor', ...
 'p.coil.forceConstant*p.windingActual.forceFactor','p.coil.resistance*p.windingActual.resistanceFactor', ...
 '1/(p.coil.inductance*p.windingActual.inductanceFactor)', ...
 'p.coil.inductance*p.windingActual.inductanceFactor/(p.coil.currentResponse*p.windingActual.currentTauFactor)'};
for j=1:numel(gainNames),set_param([drive '/' gainNames{j}],'Gain',gainExpressions{j});end
configs=struct([]);
for voltage=[24 36 48]
 for angle=[60 90]
  for mu=[0 .3]
   for seed=[7 29],append('nominal_voltage',2,voltage,3.5,angle,mu,seed,1,1);end
  end
 end
end
for mass=[.1 1.4]
 for voltage=[24 48]
  for angle=[60 90]
   for mu=[0 .3]
    for seed=[7 29],append('nominal_voltage',mass,voltage,3.5,angle,mu,seed,1,1);end
   end
  end
 end
end
for mass=[2 1.4]
 for voltage=[12 18]
  for mu=[0 .3]
   for seed=[7 29],append('low_bus',mass,voltage,3.5,90,mu,seed,1,1);end
  end
 end
end
for voltage=[12 24 48]
 for mu=[0 .3]
  for seed=[7 29],append('hot_resistance',2,voltage,3.5,90,mu,seed,1,5.56/4);end
 end
end
for current=[3 5]
 for angle=[60 90]
  for mu=[0 .3]
   for seed=[7 29],append('current_limit',2,24,current,angle,mu,seed,1,1);end
  end
 end
end
for voltage=[24 48]
 for factor=[.999 1.001]
  for seed=[7 29],append('Kf_unknown',2,voltage,3.5,90,.3,seed,factor,1);end
 end
end
assert(numel(configs)==108);
if strcmp(phase,'final_confirmation')
 configs=struct([]);
 for voltage=[24 36 48]
  for angle=[60 90]
   for mu=[0 .3]
    for seed=[7 29],append('nominal_voltage',2,voltage,3.5,angle,mu,seed,1,1);end
   end
  end
 end
 for voltage=[24 48]
  for mu=[0 .3]
   for seed=[7 29],append('hot_resistance',2,voltage,3.5,90,mu,seed,1,5.56/4);end
  end
 end
 for voltage=[24 48]
  for factor=[.7 1.3]
   append('identified_inductance',2,voltage,3.5,90,.3,29,1,1,factor);
  end
 end
 for voltage=[24 48]
  for factor=[.99 1.01 .95 1.05 .9 1.1]
   for seed=[7 29],append('Kf_unknown',2,voltage,3.5,90,.3,seed,factor,1);end
  end
 end
 for current=[3 5]
  for seed=[7 29],append('current_limit',2,24,current,90,.3,seed,1,1);end
 end
 for voltage=[24 36 48]
  for resistanceFactor=[1 1.39]
   for angle=[0 90]
    for seed=[7 1729]
     append('real_coil',2,voltage,3.5,angle,.3,seed,1,resistanceFactor);
     configs(end).knownKf=24.4;configs(end).knownR=6.4;configs(end).knownL=.004;
     configs(end).massA=1.5;configs(end).currentTau=.0001;
     configs(end).coilLabel='Moticont_LVCM_070_051_01';
    end
   end
  end
 end
 assert(numel(configs)==88);
end
initialCaseCount=numel(configs);
rows=struct([]);selectedRuns={};
baselines=containers.Map('KeyType','char','ValueType','any');
fixedControllers=containers.Map('KeyType','char','ValueType','any');
started=tic;status='running';
source=struct('sourceWorker',sourceWorker,'sourceModel',sourceModel, ...
 'sourceModelSha256',checksum(sourceModel),'candidateFile',candidateFile, ...
 'candidateSha256',checksum(candidateSnapshot),'privateCandidateSnapshot',candidateSnapshot, ...
 'privateModel',modelFile, ...
 'privateModelSha256',checksum(modelFile), ...
 'studyFile',[mfilename('fullpath') '.m'],'studySha256',checksum([mfilename('fullpath') '.m']), ...
 'matlabVersion',version,'simulinkVersion',ver('simulink'));
helperFiles=dir(fullfile(worker,'helpers','*.m'));
for j=1:numel(helperFiles)
 source.helpers(j)=struct('file',helperFiles(j).name,'sha256',checksum(fullfile(helperFiles(j).folder,helperFiles(j).name)));
end
configIndex=0;
while configIndex<numel(configs)
 configIndex=configIndex+1;
 c=configs(configIndex);q=base;q.mechanics.mS=c.mass;q.sensor.distalMass_kg=c.mass;
 if strcmp(c.family,'real_coil') && configIndex>1 && ...
         ~startsWith(configs(configIndex-1).family,'real_coil')
  % Generic comparisons are complete; release their large native histories.
  baselines=containers.Map('KeyType','char','ValueType','any');
 end
 q.mechanics.mA=c.massA;q.coil.forceConstant=c.knownKf;q.coil.resistance=c.knownR;
 q.coil.inductance=c.knownL;q.coil.currentResponse=c.currentTau;
 if startsWith(c.family,'real_coil') && isfield(q,'inertial')
  gainScale=base.coil.forceConstant/c.knownKf;
  for gainName={'forceKp','forceKi','forceKd'}
   q.inertial.(gainName{1})=base.inertial.(gainName{1})*gainScale;
  end
 end
 if isfield(q,'inertial') && isfield(q.inertial,'lowerLoadInverse')
  q.inertial.lowerLoadInverse.identifiedForceConstant_N_A=c.knownKf;
 end
 q.sensor.distalMass_model_kg=c.mass;q.force.reference=50;
 q.mechanics.angle=c.angle*pi/180;q.sensor.seed=c.seed;q.contact.mu=c.mu;
 q.coil.voltage=c.voltage;q.coil.peakCurrent=c.current;
 q.windingActual=struct('forceFactor',c.forceFactor,'resistanceFactor',c.resistanceFactor, ...
 'inductanceFactor',c.inductanceFactor,'currentTauFactor',1);
 row=empty_row(c,configIndex);timer=tic;
 controllerKey=sprintf('%s_S%.17g_A%.17g_K%.17g_R%.17g_L%.17g_tau%.17g', ...
     c.coilLabel,c.mass,c.massA,c.knownKf,c.knownR,c.knownL,c.currentTau);
 baselineResistanceFactor=1;
 if startsWith(c.family,'real_coil'),baselineResistanceFactor=c.resistanceFactor;end
 baselineKey=sprintf('%s_angle%.17g_mu%.17g_seed%d_Rfactor%.17g', ...
     controllerKey,c.angle,c.mu,c.seed,baselineResistanceFactor);
 try
  set_param(mdl,'FastRestart','off');q=acf_prepare(q);row.prepared=true;
  coefficients=struct;
  if isfield(q.d,'coilControl'),coefficients.legacyObserver=q.d.coilControl;end
  if isfield(q,'inertial')
   coefficients.activeInertial=q.inertial;
   row.forceKp_A_N=q.inertial.forceKp;row.forceKi_A_N_s=q.inertial.forceKi;
   row.forceKd_A_s_N=q.inertial.forceKd;
  end
  if isfield(q.d,'currentNotch')
   coefficients.currentNotchNumerator=q.d.currentNotch.numerator;
   coefficients.currentNotchDenominator=q.d.currentNotch.denominator;
  end
  if ~isKey(fixedControllers,controllerKey),fixedControllers(controllerKey)=coefficients;end
  assert(isequal(fixedControllers(controllerKey),coefficients),'Electrical comparison changed outer coefficients.');
  assignin(mw,'p',q);acf_select_actuator(mdl,'voice_coil');
  set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime','4','FastRestart','on');
  row.simulationAttempted=true;r=[];quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  row.simulated=true;
  assert(isfield(r,'normalContact') && isequal(r.time,r.normalContact.time), ...
      'qualify_normal_voltage:InvalidNormalLog','Native normal/physical log contract differs.');
  d=r.data;t=r.time;n=r.normalContact.data;
  assert(all(isfinite(n),'all') && max(n(:,1))>1 && abs(n(1,1)-d(1,1))<1e-8, ...
      'qualify_normal_voltage:InvalidNormalLog','Invalid native normal-force initialization/log.');
  row.normalVectorBalanceResidual_N=max(abs(n(:,1)-(d(:,1)-n(:,3).*n(:,2))./sqrt(1+n(:,3).^2)));
  assert(row.normalVectorBalanceResidual_N<1e-8,'qualify_normal_voltage:InvalidNormalLog', ...
      'Native normal-force/vector balance failed.');
  err=n(:,1)-50;row.peakNormalError_N=max(abs(err));row.rmsNormalError_N=sqrt(mean(err.^2));
  row.peakAxialError_N=r.metrics.peakAbsForceError_N;
  row.lostContact=r.metrics.lostContact;row.strokeStopHit=r.metrics.strokeStopHit;
  row.overCurrent=r.metrics.overCurrent;row.completed=r.metrics.completed;
  row.passed=r.metrics.normalPass && ~row.overCurrent;
  row.reason=normal_reason(row);row.endTime_s=t(end);row.nativeSampleCount=numel(t);
  row.initialNormalForce_N=n(1,1);row.initialNormalError_N=err(1);
  row.initialCurrent_A=d(1,14);row.initialVoltage_V=d(1,15);
  row.startupNormalError_N=window_peak(err,t,0,.05);
  row.after50msNormalError_N=window_peak(err,t,.05,4);
  row.prebumpNormalError_N=window_peak(err,t,1.5,2);
  row.bumpNormalError_N=window_peak(err,t,2,2.1);
  row.recoveryNormalError_N=window_peak(err,t,2.1,2.4);
  row.finalNormalError_N=window_peak(err,t,3,4);
  row.peakCurrent_A=r.metrics.peakCurrent_A;row.rmsCurrent_A=r.metrics.rmsCurrent_A;
  row.minActuatorTravel_m=min(d(:,2));row.maxActuatorTravel_m=max(d(:,2));
  row.actuatorTravelRange_m=row.maxActuatorTravel_m-row.minActuatorTravel_m;
  row.peakActuatorGapVelocity_m_s=max(abs(d(:,3)));
  row.peakVoltage_V=r.metrics.peakVoltage_V;
  row.effortSaturationFraction=r.metrics.effortSaturationFraction;
  row.voltageSaturationFraction=r.metrics.voltageSaturationFraction;
  actualKf=c.knownKf*c.forceFactor;actualR=c.knownR*c.resistanceFactor;
  row.peakCoilForce_N=actualKf*row.peakCurrent_A;
  regulator=(q.coil.inductance*c.inductanceFactor/q.coil.currentResponse)*(min(max(d(:,33),-c.current),c.current)-d(:,14));
  resistance=actualR*d(:,14);emf=actualKf*d(:,3);raw=regulator+resistance+emf;
  row.peakRequestedVoltage_V=max(abs(raw));row.peakCommandCurrent_A=max(abs(d(:,33)));
  if startsWith(c.family,'real_coil') && c.supplyVoltage==24
   row.amplifierReserveAt4Vdrop_V=20-max(abs(raw));
   row.amplifierReserveAt5Vdrop_V=19-max(abs(raw));
   row.requestedClipFractionAt4Vdrop=mean(abs(raw)>20);
   row.requestedClipFractionAt5Vdrop=mean(abs(raw)>19);
  end
  [~,peakVoltageIndex]=max(abs(raw));row.voltagePeakTime_s=t(peakVoltageIndex);
  row.resistiveVoltageAtPeak_V=resistance(peakVoltageIndex);
  row.backEmfVoltageAtPeak_V=emf(peakVoltageIndex);
  row.regulatorVoltageAtPeak_V=regulator(peakVoltageIndex);
  row.peakResistiveVoltage_V=max(abs(resistance));row.peakBackEmfVoltage_V=max(abs(emf));
  row.peakRegulatorVoltage_V=max(abs(regulator));
  row.currentReserve_A=c.current-row.peakCurrent_A;row.voltageReserve_V=c.voltage-row.peakVoltage_V;
  row.requestedVoltageClipFraction=mean(abs(raw)>c.voltage);
  row.voltageEquationResidual_V=max(abs(min(max(raw,-c.voltage),c.voltage)-d(:,15)));
  assert(row.voltageEquationResidual_V<1e-8,'qualify_normal_voltage:InvalidDriveLog', ...
      'Native winding voltage equation mismatch.');
  row.holdCurrentActual_A=(50-(.6+c.mass)*q.mechanics.gravity*cos(q.mechanics.angle))/actualKf;
  row.holdVoltageActual_V=abs(row.holdCurrentActual_A*actualR);
  row.actualHoldFeasible=row.holdVoltageActual_V<=c.voltage && abs(row.holdCurrentActual_A)<=c.current;
  row.holdCopperLoss_W=row.holdCurrentActual_A^2*actualR;row.rmsCopperLoss_W=row.rmsCurrent_A^2*actualR;
  row.peakNormalEstimationError_N=max(abs(n(:,4)-n(:,1)));
  row.peakEstimatedNormalError_N=max(abs(n(:,4)-50));row.peakSlopeEstimationError=max(abs(n(:,5)-n(:,3)));
  row.gateReadyFraction=mean(r.robotGate.ready);
  auxiliary=capture_auxiliary(r);
  if c.voltage==24 && (strcmp(c.family,'nominal_voltage') || strcmp(c.family,'real_coil'))
   baselines(baselineKey)=struct('id',configIndex,'time',t,'data',d,'normal',n,'auxiliary',auxiliary);
  end
  b=baselines(baselineKey);row.baseline24V_id=b.id;
  row.baselineGridMatches=isequal(t,b.time);
  if row.baselineGridMatches
   row.maxAll35DifferenceFrom24V=max(abs(d-b.data),[],'all');
   row.maxNormal8DifferenceFrom24V=max(abs(n-b.normal),[],'all');
   row.maxNormalForceDifferenceFrom24V=max(abs(n(:,1)-b.normal(:,1)));
   [row.maxAllAuxiliaryDifferenceFrom24V,row.identicalAuxiliaryTo24V,auxiliaryDifferences]= ...
       compare_auxiliary(auxiliary,b.auxiliary);
   if isfield(auxiliaryDifferences,'observerStates'),row.maxStateDifferenceFrom24V=auxiliaryDifferences.observerStates;end
   if isfield(auxiliaryDifferences,'motionObserver'),row.maxMotionObserverDifferenceFrom24V=auxiliaryDifferences.motionObserver;end
   if isfield(auxiliaryDifferences,'accelerometer'),row.maxAccelerometerDifferenceFrom24V=auxiliaryDifferences.accelerometer;end
   if isfield(auxiliaryDifferences,'measuredDrag'),row.maxMeasuredDragDifferenceFrom24V=auxiliaryDifferences.measuredDrag;end
   if isfield(auxiliaryDifferences,'forceVariation'),row.maxForceVariationDifferenceFrom24V=auxiliaryDifferences.forceVariation;end
   if isfield(auxiliaryDifferences,'accelerationFeedforward'),row.maxAccelerationFeedforwardDifferenceFrom24V=auxiliaryDifferences.accelerationFeedforward;end
   if isfield(auxiliaryDifferences,'lowerInverse'),row.maxLowerInverseDifferenceFrom24V=auxiliaryDifferences.lowerInverse;end
   row.identicalPhysicalAndNormalTo24V=isequal(d,b.data) && isequal(n,b.normal);
   row.identicalAllLoggedChannelsTo24V=row.identicalPhysicalAndNormalTo24V && row.identicalAuxiliaryTo24V;
  end
  if retain_trace(c)
   r.label=sprintf('Normal voltage: %s / S=%g kg / %g V / %g A / %g deg / mu=%g / seed%d / R%g / Kf%g', ...
     c.family,c.mass,c.voltage,c.current,c.angle,c.mu,c.seed,actualR,actualKf);
   r.performanceCaseId=configIndex;r.requestedVoltage=struct('time',t,'data',raw);
   selectedRuns{end+1}=r; %#ok<AGROW>
  end
 catch e
  row.reason=[e.identifier ': ' e.message];row.exception=true;
 end
 row.runtime_s=toc(timer);if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 set_param(mdl,'FastRestart','off');clear r d t n err raw regulator resistance emf quiet b auxiliary auxiliaryDifferences;Simulink.sdi.clear;
 if strcmp(phase,'final_confirmation') && configIndex==initialCaseCount
  hot24=strcmp({rows.family},'real_coil') & [rows.Vlimit_V]==24 & ...
      [rows.actualR_ohm]>6.4 & [rows.angle_deg]==90;
  if any([rows(hot24).peakRequestedVoltage_V]>19)
   for effectiveVoltage=[19 20]
    for seed=[7 1729]
     append('real_coil_amp_drop',2,effectiveVoltage,3.5,90,.3,seed,1,1.39);
     configs(end).knownKf=24.4;configs(end).knownR=6.4;configs(end).knownL=.004;
     configs(end).massA=1.5;configs(end).currentTau=.0001;
     configs(end).coilLabel='Moticont_LVCM_070_051_01';configs(end).supplyVoltage=24;
    end
   end
  end
 end
 if mod(configIndex,8)==0 || configIndex==numel(configs)
  checkpoint();fprintf('NORMAL_VOLTAGE %d/%d native=%d pass=%d elapsed=%.1fs last=%s %.9g N\n', ...
   configIndex,numel(configs),sum([rows.simulated]),sum([rows.passed]),toc(started),row.reason,row.peakNormalError_N);
 end
 if contains(row.reason,'qualify_normal_voltage:InvalidNormalLog') || contains(row.reason,'qualify_normal_voltage:InvalidDriveLog')
  status='invalid_model_contract';checkpoint();return;
 end
 if isfile([prefix '_stop.request']),status='stopped_by_request';checkpoint();return;end
end
source.sourceModelSha256After=checksum(sourceModel);
source.sourceModelUnchangedDuringRun=strcmp(source.sourceModelSha256After,source.sourceModelSha256);
status='complete';checkpoint();

    function append(family,mass,voltage,current,angle,mu,seed,forceFactor,resistanceFactor,inductanceFactor)
     if nargin<10,inductanceFactor=1;end
     c=struct('family',family,'mass',mass,'voltage',voltage,'current',current, ...
      'angle',angle,'mu',mu,'seed',seed,'forceFactor',forceFactor,'resistanceFactor',resistanceFactor, ...
      'inductanceFactor',inductanceFactor,'knownKf',base.coil.forceConstant, ...
      'knownR',base.coil.resistance,'knownL',base.coil.inductance,'massA',base.mechanics.mA, ...
      'currentTau',base.coil.currentResponse,'coilLabel','generic_20N_A_4ohm_3mH', ...
      'supplyVoltage',voltage);
     if isempty(configs),configs=c;else,configs(end+1)=c;end
    end
    function checkpoint
     report=struct('status',status,'phase',phase,'date','2026-09-30','plannedCases',numel(configs), ...
      'initialPlannedCases',initialCaseCount, ...
      'executedConfigurations',numel(rows),'nativeSimulations',sum([rows.simulated]), ...
      'nativeSimulationAttempts',sum([rows.simulationAttempted]), ...
      'rejectedBeforeSimulation',sum(~[rows.simulationAttempted]), ...
      'preparationRejects',sum(~[rows.prepared]), ...
      'exceptionsAfterNativeAttempt',sum([rows.exception] & [rows.simulationAttempted]), ...
      'passed',sum([rows.passed]), ...
      'source',source,'parameters',base,'configurations',configs,'rows',rows, ...
      'elapsedSeconds',toc(started),'selectedTraceCaseIds',cellfun(@(r)r.performanceCaseId,selectedRuns), ...
      'metric','True surface-normal pressing force, full0-4 s, +/-0.5 N; axial error diagnostic only.', ...
      'sensorCondition',sensor_condition(base), ...
      'auxiliaryLogsCompared',{{'observerStates when active','motionObserver when present','accelerometer when present', ...
       'measuredDrag when present','forceVariation when present','accelerationFeedforward when present', ...
       'commandPrediction when present','robotGate when present','lowerInverse when present'}}, ...
      'controllerComparison','Each known mass/nominal actuator is identified/prepared with the selected candidate. At that mass/actuator, all observer, active PI/PID, lower-load inverse and notch coefficients remain bitwise fixed across voltage/current/R/Kf probes. Real Moticont nominal current-unit PI/PID gains are scaled by20/24.4 from the generic20N/A candidate to preserve force-domain feedback effort, while identified force feedforward and lower-load inverse coefficients use24.4N/A. Unknown-Kf cases retain nominal identified controller/feedforward/initial-current coefficients. All geometry-controller parameters fixed. The controller does not receive true mu or slope.', ...
      'currentDriveEquation','Vreq=(L/tau)*(clip(Icmd,Ilimit)-i)+Ractual*i+Kfactual*v_gap; V=clip(Vreq,Vbus); di/dt=(V-Ractual*i-Kfactual*v_gap)/L. Without voltage clipping, current response is independent of Vbus and Ractual.', ...
      'actualResistanceScope','R=5.56 ohm represents an assumed100degC copper-winding temperature rise using a0.0039/degC coefficient, a separate hypothetical identified hot operating condition. The native inner regulator compensates actual R. Fixed Kf,L,tau; no thermal state/PWM/hardware test, supplier hot-force guarantee or safe-temperature qualification.', ...
      'actualInductanceScope','Final phase changes physical L by +/-30%, with matched inner L/tau compensation and fixed identified tau. Outer force/position controller stays fixed; this isolates voltage/slew effects.', ...
      'unknownKfScope','Physical force/back-EMF factors are recorded per case. Outer force feedforward, controller and native default initial current retain the known nominalKf. Native inner winding compensation matches actual back EMF so identified current tau is held fixed. Full0-4s score includes any startup error: no initial force-settling samples are removed and no loaded equilibrium is reinitialized to trueKf.', ...
      'realMotorScope','Separate real_coil family uses known constantKf24.4N/A,R6.4ohm,L4mH,100us identified current response,A1.5kg,B0.6kg,S2kg,conservative25mm travel; hypothetical identified hot resistance1.39 times cold represents an assumed100degC copper-winding temperature rise. Supplier continuous-force/power ratings have distinct stated conditions. No constant-force claim is made over travel: force-versus-position variation and physical thermal response remain unqualified.', ...
      'linearAmpHeadroomScope','Ideal coil-terminal voltage cap equals supply for primary electrical cases. For real-coil24V cases, requested waveform is also compared arithmetically with19/20V available after hypothetical5/4V linear-stage drop. If hot24V demand exceeds19V, four real_coil_amp_drop cases simulate19/20V effective terminal caps at physical supply24V,50N90deg,mu.3,seeds7/1729. This is an explicit conservative envelope, not a TA11524V datasheet guarantee.', ...
      'limits','Finite point-contact2D model and independent post-filter noise realizations. B-S load-cell stiffness/damping and current notch are retained from the selected candidate. No finite toolpad, rotating sanding force, pad/surface contact compliance, contact impacts, physical robot deflection, PWM ripple, or measured hardware qualification.');
     fid=fopen([prefix '.pending.json'],'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
     movefile([prefix '.pending.json'],[prefix '.json'],'f');
     writetable(struct2table(rows),[prefix '.pending.csv'],'FileType','text');movefile([prefix '.pending.csv'],[prefix '.csv'],'f');
     if ~strcmp(status,'running'),save([prefix '_traces.mat'],'selectedRuns','-v7.3');end
    end
end

function row=empty_row(c,id)
row=struct('id',id,'family',c.family,'coilLabel',c.coilLabel,'massS_kg',c.mass,'massB_kg',.6, ...
 'massA_kg',c.massA,'force_N',50, ...
 'angle_deg',c.angle,'mu',c.mu,'seed',c.seed,'Vlimit_V',c.voltage,'Ilimit_A',c.current, ...
 'supplyVoltage_V',c.supplyVoltage, ...
 'knownKf_N_A',c.knownKf,'knownR_ohm',c.knownR,'knownL_H',c.knownL, ...
 'actualKf_N_A',c.knownKf*c.forceFactor,'actualR_ohm',c.knownR*c.resistanceFactor, ...
 'actualL_H',c.knownL*c.inductanceFactor,'currentResponse_us',c.currentTau*1e6, ...
 'feedbackGainScale',20/c.knownKf,'forceKp_A_N',NaN,'forceKi_A_N_s',NaN,'forceKd_A_s_N',NaN, ...
 'prepared',false,'simulationAttempted',false,'simulated',false,'exception',false, ...
 'passed',false,'completed',false,'reason','', ...
 'endTime_s',NaN,'nativeSampleCount',0,'runtime_s',NaN,'peakNormalError_N',NaN,'rmsNormalError_N',NaN, ...
 'peakAxialError_N',NaN,'initialNormalForce_N',NaN,'initialNormalError_N',NaN, ...
 'initialCurrent_A',NaN,'initialVoltage_V',NaN,'startupNormalError_N',NaN,'after50msNormalError_N',NaN, ...
 'prebumpNormalError_N',NaN,'bumpNormalError_N',NaN, ...
 'recoveryNormalError_N',NaN,'finalNormalError_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN, ...
 'minActuatorTravel_m',NaN,'maxActuatorTravel_m',NaN,'actuatorTravelRange_m',NaN, ...
 'peakActuatorGapVelocity_m_s',NaN,'peakCoilForce_N',NaN, ...
 'peakVoltage_V',NaN,'peakRequestedVoltage_V',NaN,'peakCommandCurrent_A',NaN, ...
 'amplifierReserveAt4Vdrop_V',NaN,'amplifierReserveAt5Vdrop_V',NaN, ...
 'requestedClipFractionAt4Vdrop',NaN,'requestedClipFractionAt5Vdrop',NaN, ...
 'voltagePeakTime_s',NaN,'resistiveVoltageAtPeak_V',NaN,'backEmfVoltageAtPeak_V',NaN, ...
 'regulatorVoltageAtPeak_V',NaN,'peakResistiveVoltage_V',NaN,'peakBackEmfVoltage_V',NaN, ...
 'peakRegulatorVoltage_V',NaN,'currentReserve_A',NaN,'voltageReserve_V',NaN, ...
 'effortSaturationFraction',NaN,'voltageSaturationFraction',NaN,'requestedVoltageClipFraction',NaN, ...
 'voltageEquationResidual_V',NaN,'holdCurrentActual_A',NaN,'holdVoltageActual_V',NaN, ...
 'actualHoldFeasible',false,'holdCopperLoss_W',NaN,'rmsCopperLoss_W',NaN, ...
 'peakNormalEstimationError_N',NaN,'peakEstimatedNormalError_N',NaN,'peakSlopeEstimationError',NaN, ...
 'normalVectorBalanceResidual_N',NaN, ...
 'gateReadyFraction',NaN,'lostContact',false,'strokeStopHit',false,'overCurrent',false, ...
 'baseline24V_id',NaN,'baselineGridMatches',false,'maxAll35DifferenceFrom24V',NaN, ...
 'maxNormal8DifferenceFrom24V',NaN,'maxNormalForceDifferenceFrom24V',NaN,'maxStateDifferenceFrom24V',NaN, ...
 'maxMotionObserverDifferenceFrom24V',NaN,'maxAccelerometerDifferenceFrom24V',NaN, ...
 'maxMeasuredDragDifferenceFrom24V',NaN,'maxForceVariationDifferenceFrom24V',NaN, ...
 'maxAccelerationFeedforwardDifferenceFrom24V',NaN, ...
 'maxLowerInverseDifferenceFrom24V',NaN, ...
 'maxAllAuxiliaryDifferenceFrom24V',NaN,'identicalAuxiliaryTo24V',false, ...
 'identicalPhysicalAndNormalTo24V',false,'identicalAllLoggedChannelsTo24V',false);
end

function logs=capture_auxiliary(r)
logs=struct;names={'observerStates','motionObserver','accelerometer', ...
 'measuredDrag','forceVariation','accelerationFeedforward','commandPrediction','lowerInverse'};
for j=1:numel(names)
 name=names{j};
 if isfield(r,name)
  if j<=3,assert(isequal(r.time,r.(name).time),'Native auxiliary and physical log grids differ.');end
  assert(all(isfinite(r.(name).data),'all'),'Native auxiliary data are non-finite.');
  logs.(name)=struct('time',r.(name).time,'data',r.(name).data);
 end
end
if isfield(r,'robotGate')
 logs.robotGate=struct('time',r.robotGate.time,'data',[r.robotGate.ready, ...
     r.robotGate.filteredError_N,r.robotGate.forceRate_N_s,r.robotGate.stableNow]);
end
end

function [peak,identical,differences]=compare_auxiliary(logs,baseline)
names=fieldnames(logs);assert(isequal(names,fieldnames(baseline)),'Auxiliary log contract changed.');
peak=0;identical=true;differences=struct;
for j=1:numel(names)
 name=names{j};a=logs.(name);b=baseline.(name);
 assert(isequal(a.time,b.time),'Paired auxiliary log grids differ.');
 differences.(name)=max(abs(a.data-b.data),[],'all');
 peak=max(peak,differences.(name));identical=identical && isequal(a.data,b.data);
end
end

function description=sensor_condition(p)
description=sprintf(['Conditional multi-axis B-S sensor (axial and lateral), S-relative-to-A position, ' ...
 'issued robot motion only. No known surface preview. Axial/lateral force noise%g/%g N RMS, ' ...
 'position noise%g um RMS, force filter%g us, axial force delay%g us, lateral delay%d samples, ' ...
 'position delay%g us, control period%g us, native solver%g us, identified current response%g us, ' ...
 'selected force-gate filter%g ms.'], ...
 p.sensor.noiseRms,p.contact.lateralNoiseRms_N,p.sensor.strokeNoiseRms*1e6, ...
 p.sensor.filterTime*1e6,p.sensor.delay*1e6,p.contact.lateralDelaySamples, ...
 p.sensor.strokeDelay*1e6,p.force.sampleTime*1e6,p.sim.step*1e6,p.coil.currentResponse*1e6, ...
 p.robot.forceGate.filterTime*1e3);
if isfield(p,'accel') && isfield(p,'inertial')
 description=[description ' The selected controller also uses a causal S accelerometer with its candidate noise/filter/delay/bias/scale/quantization parameters retained verbatim in parameters.accel; parameters.inertial records its PI/PID and position-observer settings.'];
end
end

function v=window_peak(err,t,a,b)
use=t>=a & t<=b;if any(use),v=max(abs(err(use)));else,v=NaN;end
end

function s=normal_reason(r)
reasons={};if r.peakNormalError_N>.5,reasons{end+1}='normal force error';end
if r.lostContact,reasons{end+1}='loss of contact';end
if r.strokeStopHit,reasons{end+1}='stroke stop';end
if r.overCurrent,reasons{end+1}='overcurrent';end
if ~r.completed && ~r.lostContact && ~r.strokeStopHit,reasons{end+1}='incomplete';end
if isempty(reasons),s='pass';else,s=strjoin(reasons,', ');end
end

function yes=retain_trace(c)
yes=c.mass==2 && c.angle==90 && c.seed==29 && c.mu==.3 && ( ...
 (strcmp(c.family,'nominal_voltage') && any(c.voltage==[24 48])) || ...
 (strcmp(c.family,'low_bus') && c.voltage==12) || ...
 (strcmp(c.family,'hot_resistance') && any(c.voltage==[12 24 48])) || ...
 (strcmp(c.family,'identified_inductance') && c.inductanceFactor>1 && c.voltage==24) || ...
 (strcmp(c.family,'Kf_unknown') && c.forceFactor>1.09 && any(c.voltage==[24 48])));
if strcmp(c.family,'real_coil') && c.angle==90 && c.seed==1729
 yes=any(c.voltage==[24 48]) && (c.resistanceFactor>1 || c.voltage==24);
end
if strcmp(c.family,'real_coil_amp_drop'),yes=c.voltage==19 && c.seed==1729;end
end

function finish(mdl,worker,previousStorage,previousRecord,previousGen)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
if strcmp(Simulink.sdi.getStorageLocation,fullfile(worker,'sdi')),Simulink.sdi.clear;end
Simulink.sdi.setStorageLocation(previousStorage);Simulink.sdi.setRecordData(previousRecord);
Simulink.fileGenControl('setConfig','config',previousGen);
end

function h=checksum(path)
q=char(39);path=[q,strrep(path,q,[q,char(34),q,char(34),q]),q];
[status,result]=system(['sha256sum -- ' path]);assert(status==0);h=regexp(result,'^[0-9a-f]{64}','match','once');
end
