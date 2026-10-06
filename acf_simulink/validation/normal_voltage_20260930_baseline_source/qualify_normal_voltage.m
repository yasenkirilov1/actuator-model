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
addpath(fullfile(worker,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator;
loaded=load(candidateFile,'p');base=loaded.p;
assert(isfield(base,'contact'),'Candidate must contain the causal normal-contact controller.');
assert(base.mechanics.upperRigid && base.mechanics.lowerRigid);
assert(abs(base.mechanics.mB-.6)<1e-12 && strcmp(base.sensor.positionTarget,'tool_relative_A'));
assert(base.robot.forceGate.enabled && abs(base.robot.forceGate.filterTime-.1)<1e-12);
assert(abs(base.coil.forceConstant-20)<1e-12 && abs(base.coil.currentResponse-.0001)<1e-12);
assert(abs(base.force.sampleTime-.0001)<1e-12 && base.coil.directCoulombCorrection);
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.sim.step=.00005;
base.surface.kind='sharp_bump';base.surface.height=.001;base.surface.width=.01;
base.surface.scanSpeed=.1;base.surface.startTime=2;
base.sensor.noiseRms=.02;base.sensor.strokeNoiseRms=1e-6;
base.contact.lateralNoiseRms_N=.02;base.contact.lateralBias_N=0;
base.contact.lateralDelaySamples=1;
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
 keep=strcmp({configs.family},'nominal_voltage') & [configs.mass]==2 & ...
     ismember([configs.voltage],[24 48]);configs=configs(keep);assert(numel(configs)==16);
end
rows=struct([]);selectedRuns={};baselines=cell(3,2,2,2);fixedControllers=cell(3,1);
started=tic;status='running';
source=struct('sourceWorker',sourceWorker,'sourceModel',sourceModel, ...
 'sourceModelSha256',checksum(sourceModel),'candidateFile',candidateFile, ...
 'candidateSha256',checksum(candidateFile),'privateModel',modelFile, ...
 'privateModelSha256',checksum(modelFile), ...
 'studyFile',[mfilename('fullpath') '.m'],'studySha256',checksum([mfilename('fullpath') '.m']), ...
 'matlabVersion',version,'simulinkVersion',ver('simulink'));
helperFiles=dir(fullfile(worker,'helpers','*.m'));
for j=1:numel(helperFiles)
 source.helpers(j)=struct('file',helperFiles(j).name,'sha256',checksum(fullfile(helperFiles(j).folder,helperFiles(j).name)));
end
for configIndex=1:numel(configs)
 c=configs(configIndex);q=base;q.mechanics.mS=c.mass;q.sensor.distalMass_kg=c.mass;
 q.sensor.distalMass_model_kg=c.mass;q.force.reference=50;
 q.mechanics.angle=c.angle*pi/180;q.sensor.seed=c.seed;q.contact.mu=c.mu;
 q.coil.voltage=c.voltage;q.coil.peakCurrent=c.current;
 q.windingActual=struct('forceFactor',c.forceFactor,'resistanceFactor',c.resistanceFactor, ...
 'inductanceFactor',1,'currentTauFactor',1);
 row=empty_row(c,configIndex);timer=tic;
 mi=find([.1 1.4 2]==c.mass);ai=find([60 90]==c.angle);ui=find([0 .3]==c.mu);si=find([7 29]==c.seed);
 try
  set_param(mdl,'FastRestart','off');q=acf_prepare(q);row.prepared=true;
  if isempty(fixedControllers{mi}),fixedControllers{mi}=q.d.coilControl;end
  assert(isequal(fixedControllers{mi},q.d.coilControl),'Electrical comparison changed outer coefficients.');
  assignin(mw,'p',q);acf_select_actuator(mdl,'voice_coil');
  set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime','4','FastRestart','on');
  r=[];quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  assert(isequal(r.time,r.normalContact.time),'Native normal/physical grids differ.');
  row.simulated=true;d=r.data;t=r.time;n=r.normalContact.data;
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
  row.startupNormalError_N=window_peak(err,t,0,.05);
  row.prebumpNormalError_N=window_peak(err,t,1.5,2);
  row.bumpNormalError_N=window_peak(err,t,2,2.1);
  row.recoveryNormalError_N=window_peak(err,t,2.1,2.4);
  row.finalNormalError_N=window_peak(err,t,3,4);
  row.peakCurrent_A=r.metrics.peakCurrent_A;row.rmsCurrent_A=r.metrics.rmsCurrent_A;
  row.peakVoltage_V=r.metrics.peakVoltage_V;
  row.effortSaturationFraction=r.metrics.effortSaturationFraction;
  row.voltageSaturationFraction=r.metrics.voltageSaturationFraction;
  actualKf=20*c.forceFactor;actualR=4*c.resistanceFactor;
  regulator=(.003/.0001)*(min(max(d(:,33),-c.current),c.current)-d(:,14));
  resistance=actualR*d(:,14);emf=actualKf*d(:,3);raw=regulator+resistance+emf;
  row.peakRequestedVoltage_V=max(abs(raw));row.peakCommandCurrent_A=max(abs(d(:,33)));
  [~,peakVoltageIndex]=max(abs(raw));row.voltagePeakTime_s=t(peakVoltageIndex);
  row.resistiveVoltageAtPeak_V=resistance(peakVoltageIndex);
  row.backEmfVoltageAtPeak_V=emf(peakVoltageIndex);
  row.regulatorVoltageAtPeak_V=regulator(peakVoltageIndex);
  row.peakResistiveVoltage_V=max(abs(resistance));row.peakBackEmfVoltage_V=max(abs(emf));
  row.peakRegulatorVoltage_V=max(abs(regulator));
  row.currentReserve_A=c.current-row.peakCurrent_A;row.voltageReserve_V=c.voltage-row.peakVoltage_V;
  row.requestedVoltageClipFraction=mean(abs(raw)>c.voltage);
  row.voltageEquationResidual_V=max(abs(min(max(raw,-c.voltage),c.voltage)-d(:,15)));
  assert(row.voltageEquationResidual_V<1e-8,'Native winding voltage equation mismatch.');
  row.holdCurrentActual_A=(50-(.6+c.mass)*q.mechanics.gravity*cos(q.mechanics.angle))/actualKf;
  row.holdVoltageActual_V=abs(row.holdCurrentActual_A*actualR);
  row.actualHoldFeasible=row.holdVoltageActual_V<=c.voltage && abs(row.holdCurrentActual_A)<=c.current;
  row.holdCopperLoss_W=row.holdCurrentActual_A^2*actualR;row.rmsCopperLoss_W=row.rmsCurrent_A^2*actualR;
  row.peakNormalEstimationError_N=max(abs(n(:,4)-n(:,1)));
  row.peakEstimatedNormalError_N=max(abs(n(:,4)-50));row.peakSlopeEstimationError=max(abs(n(:,5)-n(:,3)));
  row.gateReadyFraction=mean(r.robotGate.ready);
  if strcmp(c.family,'nominal_voltage') && c.voltage==24
   baselines{mi,ai,ui,si}=struct('id',configIndex,'time',t,'data',d,'normal',n,'states',r.observerStates.data);
  end
  b=baselines{mi,ai,ui,si};row.baseline24V_id=b.id;
  row.baselineGridMatches=isequal(t,b.time);
  if row.baselineGridMatches
   row.maxAll35DifferenceFrom24V=max(abs(d-b.data),[],'all');
   row.maxNormal8DifferenceFrom24V=max(abs(n-b.normal),[],'all');
   row.maxNormalForceDifferenceFrom24V=max(abs(n(:,1)-b.normal(:,1)));
   row.maxStateDifferenceFrom24V=max(abs(r.observerStates.data-b.states),[],'all');
   row.identicalPhysicalAndNormalTo24V=isequal(d,b.data) && isequal(n,b.normal);
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
 set_param(mdl,'FastRestart','off');clear r d t n err raw regulator resistance emf quiet b;Simulink.sdi.clear;
 if mod(configIndex,8)==0 || configIndex==numel(configs)
  checkpoint();fprintf('NORMAL_VOLTAGE %d/%d native=%d pass=%d elapsed=%.1fs last=%s %.9g N\n', ...
   configIndex,numel(configs),sum([rows.simulated]),sum([rows.passed]),toc(started),row.reason,row.peakNormalError_N);
 end
 if contains(row.reason,'qualify_normal_voltage:InvalidNormalLog')
  status='invalid_model_contract';checkpoint();return;
 end
 if isfile([prefix '_stop.request']),status='stopped_by_request';checkpoint();return;end
end
source.sourceModelSha256After=checksum(sourceModel);
source.sourceModelUnchangedDuringRun=strcmp(source.sourceModelSha256After,source.sourceModelSha256);
status='complete';checkpoint();

    function append(family,mass,voltage,current,angle,mu,seed,forceFactor,resistanceFactor)
     c=struct('family',family,'mass',mass,'voltage',voltage,'current',current, ...
      'angle',angle,'mu',mu,'seed',seed,'forceFactor',forceFactor,'resistanceFactor',resistanceFactor);
     if isempty(configs),configs=c;else,configs(end+1)=c;end
    end
    function checkpoint
     report=struct('status',status,'phase',phase,'date','2026-09-30','plannedCases',numel(configs), ...
      'executedConfigurations',numel(rows),'nativeSimulations',sum([rows.simulated]), ...
      'rejectedBeforeSimulation',sum(~[rows.simulated]),'passed',sum([rows.passed]), ...
      'source',source,'parameters',base,'configurations',configs,'rows',rows, ...
      'elapsedSeconds',toc(started),'selectedTraceCaseIds',cellfun(@(r)r.performanceCaseId,selectedRuns), ...
      'metric','True surface-normal pressing force, full0-4 s, +/-0.5 N; axial error diagnostic only.', ...
      'sensorCondition','Conditional multi-axis B-S sensor (axial and lateral), S-relative-to-A position, issued robot motion only. No known surface preview. Force channels0.02 N RMS, position1 um RMS, known100 us sensor delay/filter, 10 kHz control,50 us native solver.', ...
      'controllerComparison','Each known mass is identified/prepared with the candidate weights. At that mass, all observer/feedback coefficients remain bitwise fixed across voltage/current/R/Kf probes. All geometry-controller parameters fixed. The controller does not receive true mu or slope.', ...
      'currentDriveEquation','Vreq=(L/tau)*(clip(Icmd,Ilimit)-i)+Ractual*i+Kfactual*v_gap; V=clip(Vreq,Vbus); di/dt=(V-Ractual*i-Kfactual*v_gap)/L. Without voltage clipping, current response is independent of Vbus and Ractual.', ...
      'actualResistanceScope','R=5.56 ohm is a separate hypothetical identified hot operating condition. The native inner regulator compensates actual R. Fixed Kf,L,tau; no thermal state/PWM/hardware test.', ...
      'unknownKfScope','Physical Kf/back-EMF gains +/-0.1%, observer/controller and initial equilibrium retain nominalKf20. Full-run score includes startup. These mismatch probes are not new calibrated equilibria.', ...
      'limits','Finite point-contact2D model and independent post-filter noise realizations. No finite toolpad/rotating sanding force/contact compliance/contact impacts/physical robot deflection. Multi-axis sensor architecture is conditional; no hardware qualification.');
     fid=fopen([prefix '.pending.json'],'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
     movefile([prefix '.pending.json'],[prefix '.json'],'f');
     writetable(struct2table(rows),[prefix '.pending.csv'],'FileType','text');movefile([prefix '.pending.csv'],[prefix '.csv'],'f');
     if ~strcmp(status,'running'),save([prefix '_traces.mat'],'selectedRuns','-v7.3');end
    end
end

function row=empty_row(c,id)
row=struct('id',id,'family',c.family,'massS_kg',c.mass,'massB_kg',.6,'force_N',50, ...
 'angle_deg',c.angle,'mu',c.mu,'seed',c.seed,'Vlimit_V',c.voltage,'Ilimit_A',c.current, ...
 'actualKf_N_A',20*c.forceFactor,'actualR_ohm',4*c.resistanceFactor, ...
 'prepared',false,'simulated',false,'exception',false,'passed',false,'completed',false,'reason','', ...
 'endTime_s',NaN,'nativeSampleCount',0,'runtime_s',NaN,'peakNormalError_N',NaN,'rmsNormalError_N',NaN, ...
 'peakAxialError_N',NaN,'startupNormalError_N',NaN,'prebumpNormalError_N',NaN,'bumpNormalError_N',NaN, ...
 'recoveryNormalError_N',NaN,'finalNormalError_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN, ...
 'peakVoltage_V',NaN,'peakRequestedVoltage_V',NaN,'peakCommandCurrent_A',NaN, ...
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
 'identicalPhysicalAndNormalTo24V',false);
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
 (strcmp(c.family,'Kf_unknown') && c.forceFactor>1 && c.voltage==48));
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
