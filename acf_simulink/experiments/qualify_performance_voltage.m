function report=qualify_performance_voltage
%QUALIFY_PERFORMANCE_VOLTAGE Isolated native voltage/current headroom study.
% Uses the final B-S load cell / S-A position controller without retuning.
% Only an unsaved private model is modified. Run with MATLAB -batch.
assert(~usejava('desktop'),'Run this study with MATLAB -batch.');
maxNumCompThreads(2);
root=fileparts(fileparts(mfilename('fullpath')));addpath(root,'-begin');
modelSource=fullfile(root,'generated','acf_force_control_simple.slx');
candidateSource=fullfile(root,'validation','final_two_sensor_candidate.mat');
prefix=fullfile(root,'validation','performance_voltage_20260930');
assert(~isfile([prefix '.json']),'Output already exists; retain it rather than overwrite.');
loaded=load(candidateSource,'p');base=loaded.p;
assert(base.mechanics.upperRigid && base.mechanics.lowerRigid);
assert(abs(base.mechanics.mB-.6)<1e-12 && abs(base.mechanics.mS-1.4)<1e-12);
assert(strcmp(base.sensor.positionTarget,'tool_relative_A'));
assert(abs(base.sensor.distalMass_kg-1.4)<1e-12);
assert(base.coil.directCoulombCorrection && strcmp(base.coil.strategy,'observer'));
assert(base.robot.forceGate.enabled && abs(base.robot.forceGate.filterTime-.1)<1e-12);
assert(abs(base.robot.forceGate.dwellTime-.1)<1e-12);
assert(abs(base.coil.forceConstant-20)<1e-12 && abs(base.coil.currentResponse-.0001)<1e-12);
assert(abs(base.force.sampleTime-.0001)<1e-12 && abs(base.sensor.sampleTime-.0001)<1e-12);
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.sim.step=.00005;
base.sensor.noiseRms=.02;base.sensor.strokeNoiseRms=1e-6;
base.surface.kind='sharp_bump';base.surface.height=.001;base.surface.width=.01;
base.surface.scanSpeed=.1;base.surface.startTime=2;base.actuator.kind='voice_coil';
base=acf_prepare(base);fixedController=base.d.coilControl;
worker=tempname;mkdir(worker);mkdir(fullfile(worker,'sdi'));
modelFile=fullfile(worker,'acf_force_control_simple.slx');copyfile(modelSource,modelFile);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
previousGen=Simulink.fileGenControl('getConfig');mdl='acf_force_control_simple';
cleanup=onCleanup(@()finish(mdl,worker,previousStorage,previousRecord,previousGen)); %#ok<NASGU>
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
    'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(worker,'sdi'));Simulink.sdi.setRecordData(true);
load_system(modelFile);assert(strcmp(get_param(mdl,'FileName'),modelFile));
set_param(mdl,'FastRestart','off');mw=get_param(mdl,'ModelWorkspace');
drive=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil'];
gainNames={'Coil force constant','Back EMF','Resistance','Inverse inductance','Current regulator'};
gainExpressions={'p.coil.forceConstant*p.windingActual.forceFactor', ...
    'p.coil.forceConstant*p.windingActual.forceFactor', ...
    'p.coil.resistance*p.windingActual.resistanceFactor', ...
    '1/(p.coil.inductance*p.windingActual.inductanceFactor)', ...
    'p.coil.inductance*p.windingActual.inductanceFactor/(p.coil.currentResponse*p.windingActual.currentTauFactor)'};
for j=1:numel(gainNames),set_param([drive '/' gainNames{j}],'Gain',gainExpressions{j});end
configs=struct([]);
% Run 24 V first to bind exact waveform comparisons to identical noisy cases.
for voltage=[24 12 18 36 48]
 for force=[5 50]
  for angle=[0 60 90]
   for seed=[7 29],append_config('voltage_nominal',voltage,3.5,force,angle,seed,1,1);end
  end
 end
end
for voltage=[24 48]
 for current=[2.6 3 5]
  for angle=[60 90]
   for seed=[7 29],append_config('current_limit',voltage,current,50,angle,seed,1,1);end
  end
 end
end
for voltage=[12 24 48]
 for forceFactor=[.999 1.001]
  for seed=[7 29],append_config('Kf_unknown',voltage,3.5,50,90,seed,forceFactor,1);end
 end
end
for actualR=[5.56 8]
 for voltage=[12 24 48]
  for angle=[0 60 90]
   for seed=[7 29],append_config('resistance_hypothetical',voltage,3.5,50,angle,seed,1,actualR/4);end
  end
 end
end
assert(numel(configs)==132);
rows=struct([]);selectedRuns={};baselines=cell(2,3,2);started=tic;status='running';
source=struct('modelFile',modelSource,'modelSha256',checksum(modelSource), ...
    'candidateFile',candidateSource,'candidateSha256',checksum(candidateSource), ...
    'studyFile',[mfilename('fullpath') '.m'],'studySha256',checksum([mfilename('fullpath') '.m']), ...
    'privateModelFile',modelFile,'privateFileSha256',checksum(modelFile), ...
    'matlabVersion',version,'simulinkVersion',ver('simulink'));
helperNames={'acf_prepare','acf_run','acf_select_actuator','acf_metrics','acf_signals'};
for j=1:numel(helperNames)
 source.helpers(j)=struct('file',which(helperNames{j}),'sha256',checksum(which(helperNames{j})));
end
for configIndex=1:numel(configs)
 c=configs(configIndex);q=base;q.coil.voltage=c.voltage;q.coil.peakCurrent=c.current;
 q.force.reference=c.force;q.mechanics.angle=c.angle*pi/180;q.sensor.seed=c.seed;
 q.windingActual=struct('forceFactor',c.forceFactor,'resistanceFactor',c.resistanceFactor, ...
    'inductanceFactor',1,'currentTauFactor',1);
 row=empty_row(c,configIndex);runTimer=tic;
 try
  set_param(mdl,'FastRestart','off');
  q=acf_prepare(q);assert(isequal(q.d.coilControl,fixedController),'Controller tuning changed.');
  row.prepared=true;assignin(mw,'p',q);acf_select_actuator(mdl,'voice_coil');
  set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime','4','FastRestart','on');
  quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  row.simulated=true;row.passed=r.metrics.passed;row.completed=r.metrics.completed;
  row.reason=r.metrics.reason;row.endTime_s=r.time(end);row.nativeSampleCount=numel(r.time);
  row.peakError_N=r.metrics.peakAbsForceError_N;row.rmsError_N=r.metrics.rmsForceError_N;
  row.peakCurrent_A=r.metrics.peakCurrent_A;row.rmsCurrent_A=r.metrics.rmsCurrent_A;
  row.peakVoltage_V=r.metrics.peakVoltage_V;row.lostContact=r.metrics.lostContact;
  row.strokeStopHit=r.metrics.strokeStopHit;row.overCurrent=r.metrics.overCurrent;
  row.effortSaturationFraction=r.metrics.effortSaturationFraction;
  row.voltageSaturationFraction=r.metrics.voltageSaturationFraction;
  d=r.data;t=r.time;err=d(:,31)-d(:,1);
  row.startupPeakError_N=window_peak(err,t,0,.05);
  row.prebumpPeakError_N=window_peak(err,t,1.5,2);
  row.bumpPeakError_N=window_peak(err,t,2,2.1);
  row.recoveryPeakError_N=window_peak(err,t,2.1,2.4);
  row.bumpAndRecoveryPeakError_N=window_peak(err,t,2,2.4);
  row.finalPeakError_N=window_peak(err,t,3,4);
  actualKf=20*c.forceFactor;actualR=4*c.resistanceFactor;
  raw=(.003/.0001)*(min(max(d(:,33),-c.current),c.current)-d(:,14))+ ...
      actualR*d(:,14)+actualKf*d(:,3);
  row.peakRequestedVoltage_V=max(abs(raw));row.peakCommandCurrent_A=max(abs(d(:,33)));
  row.currentCommandAtLimitFraction=mean(abs(d(:,33))>=c.current-1e-12);
  row.requestedVoltageClipFraction=mean(abs(raw)>c.voltage);
  row.voltageEquationResidual_V=max(abs(min(max(raw,-c.voltage),c.voltage)-d(:,15)));
  assert(row.voltageEquationResidual_V<1e-8,'Native voltage reconstruction failed.');
  row.holdCurrentActual_A=(c.force-2*q.mechanics.gravity*cos(q.mechanics.angle))/actualKf;
  row.holdVoltageActual_V=abs(row.holdCurrentActual_A*actualR);
  row.hypotheticalActualHoldFeasible=row.holdVoltageActual_V<=c.voltage && abs(row.holdCurrentActual_A)<=c.current;
  row.holdCopperLoss_W=row.holdCurrentActual_A^2*actualR;
  row.rmsCopperLoss_W=row.rmsCurrent_A^2*actualR;
  s=r.observerStates;estimate=q.force.reference+s.data*q.d.coilControl.force' ...
      -q.coil.coulombForce*tanh(s.data(:,6)/q.coil.frictionSpeed);
  truth=interp1(t,d(:,1),s.time,'linear');
  row.peakEstimatedContactError_N=max(abs(estimate-c.force));
  row.peakContactEstimationError_N=max(abs(estimate-truth));
  row.gateReadyFraction=mean(r.robotGate.ready);
  fi=find([5 50]==c.force);ai=find([0 60 90]==c.angle);si=find([7 29]==c.seed);
  if strcmp(c.family,'voltage_nominal') && c.voltage==24
   baselines{fi,ai,si}=struct('id',configIndex,'time',t,'data',d,'states',s);
  end
  b=baselines{fi,ai,si};row.baseline24V_id=b.id;
  row.baselineSampleGridMatches=isequal(t,b.time);
  if row.baselineSampleGridMatches
   row.maxAll35DifferenceFromNominal24V=max(abs(d-b.data),[],'all');
   row.maxTrueForceDifferenceFromNominal24V=max(abs(d(:,1)-b.data(:,1)));
   row.maxStateDifferenceFromNominal24V=max(abs(s.data-b.states.data),[],'all');
   row.identicalAll35ToNominal24V=isequal(d,b.data);
  end
  if retain_trace(c)
   r.label=sprintf('%s: %g V / %g A / %g N / %g deg / seed %d / Kf %g / R %g', ...
      c.family,c.voltage,c.current,c.force,c.angle,c.seed,actualKf,actualR);
   r.windingActual=q.windingActual;r.performanceCaseId=configIndex;
   r.requestedVoltage=struct('time',t,'data',raw);
   r.contactEstimate=struct('time',s.time,'force_N',estimate);
   selectedRuns{end+1}=r; %#ok<AGROW>
  end
 catch e
  row.reason=[e.identifier ': ' e.message];row.exception=true;
 end
 row.runtime_s=toc(runTimer);if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 set_param(mdl,'FastRestart','off');clear r quiet d t raw s estimate truth b;Simulink.sdi.clear;
 if mod(configIndex,12)==0 || configIndex==numel(configs)
  checkpoint();fprintf('PERFORMANCE_VOLTAGE %d/%d native=%d pass=%d elapsed=%.1fs last=%s %.9g N\n', ...
    configIndex,numel(configs),sum([rows.simulated]),sum([rows.passed]),toc(started),row.reason,row.peakError_N);
 end
end
status='complete';checkpoint();
assert(strcmp(checksum(modelSource),source.modelSha256),'Canonical model changed during study.');

    function append_config(family,voltage,current,force,angle,seed,forceFactor,resistanceFactor)
     c=struct('family',family,'voltage',voltage,'current',current,'force',force, ...
         'angle',angle,'seed',seed,'forceFactor',forceFactor,'resistanceFactor',resistanceFactor);
     if isempty(configs),configs=c;else,configs(end+1)=c;end
    end
    function checkpoint
     report=struct('status',status,'date','2026-09-30','plannedCases',numel(configs), ...
       'executedConfigurations',numel(rows),'nativeSimulations',sum([rows.simulated]), ...
       'rejectedBeforeSimulation',sum(~[rows.simulated]),'passed',sum([rows.passed]), ...
       'source',source,'parameters',base,'configurations',configs,'rows',rows, ...
       'elapsedSeconds',toc(started),'selectedTraceCaseIds',cellfun(@(r)r.performanceCaseId,selectedRuns), ...
       'scope','Final calibrated 100 ms force gate. Actual B-S load cell with 1.4 kg distal mass and S-A position, only two physical sensors; both rigid links, B+S=2 kg. Sharp bump 1 mm x 10 mm at 100 mm/s. Score true axial contact error throughout 0-4 s. 10 kHz loop, 50 us native solver, 0.02 N / 1 um RMS independent post-filter sensor noise, known 100 us delays and force filter.', ...
       'controllerFixed','All observer and feedback matrices remain bitwise identical to final candidate. Only bus/current limits and stated private winding force/resistance gains change. No control or sensor retuning.', ...
       'voltageEquation','Vreq=(L/tau)*(clip(Icmd,Ilimit)-i)+Ractual*i+Kfactual*v_gap; V=clip(Vreq,Vbus); di/dt=(V-Ractual*i-Kfactual*v_gap)/L. Without voltage clipping, di/dt=(clip(Icmd,Ilimit)-i)/tau, independent of bus voltage and R.', ...
       'unknownKfScope','Physical Kf and back EMF gain changed +/-0.1%; outer observer/controller and initial current equilibrium keep nominal Kf=20 N/A. Includes startup error; these are model mismatch probes, not new equilibrated physical qualification.', ...
       'resistanceScope','R=5.56 ohm and 8 ohm are separate hypothetical high-resistance operating conditions, not full thermal simulations. Inner regulator compensates actual R; outer force observer remains nominal. Fixed Kf, inductance and current time constant. 8 ohm is a severe stress condition, not claimed achievable copper temperature.', ...
       'limits','Finite paired cases, not continuous-range or hardware qualification. Axial force, no true local-normal or tangential force model. No thermal states/PWM/current-readback, no unknown additional sensor lag, no surface preview. Equality to 24 V applies only at same noise seed/force/orientation.');
     fid=fopen([prefix '.pending.json'],'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
     movefile([prefix '.pending.json'],[prefix '.json'],'f');
     writetable(struct2table(rows),[prefix '.pending.csv'],'FileType','text');movefile([prefix '.pending.csv'],[prefix '.csv'],'f');
     if strcmp(status,'complete'),save([prefix '_traces.mat'],'selectedRuns','-v7.3');end
    end
end

function row=empty_row(c,id)
row=struct('id',id,'family',c.family,'Vlimit_V',c.voltage,'Ilimit_A',c.current, ...
 'force_N',c.force,'angle_deg',c.angle,'seed',c.seed,'actualKf_N_A',20*c.forceFactor, ...
 'actualR_ohm',4*c.resistanceFactor,'actualL_H',.003,'actualCurrentResponse_s',.0001, ...
 'prepared',false,'simulated',false,'exception',false,'passed',false,'completed',false, ...
 'reason','','endTime_s',NaN,'nativeSampleCount',0,'runtime_s',NaN,'peakError_N',NaN, ...
 'rmsError_N',NaN,'startupPeakError_N',NaN,'prebumpPeakError_N',NaN,'bumpPeakError_N',NaN, ...
 'recoveryPeakError_N',NaN,'bumpAndRecoveryPeakError_N',NaN,'finalPeakError_N',NaN, ...
 'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN,'peakRequestedVoltage_V',NaN, ...
 'peakCommandCurrent_A',NaN,'currentCommandAtLimitFraction',NaN,'requestedVoltageClipFraction',NaN, ...
 'effortSaturationFraction',NaN,'voltageSaturationFraction',NaN,'voltageEquationResidual_V',NaN, ...
 'holdCurrentActual_A',NaN,'holdVoltageActual_V',NaN,'hypotheticalActualHoldFeasible',false, ...
 'holdCopperLoss_W',NaN,'rmsCopperLoss_W',NaN,'peakEstimatedContactError_N',NaN, ...
 'peakContactEstimationError_N',NaN,'gateReadyFraction',NaN,'lostContact',false, ...
 'strokeStopHit',false,'overCurrent',false,'baseline24V_id',NaN,'baselineSampleGridMatches',false, ...
 'maxAll35DifferenceFromNominal24V',NaN,'maxTrueForceDifferenceFromNominal24V',NaN, ...
 'maxStateDifferenceFromNominal24V',NaN,'identicalAll35ToNominal24V',false);
end

function v=window_peak(err,t,a,b)
use=t>=a & t<=b;if any(use),v=max(abs(err(use)));else,v=NaN;end
end

function yes=retain_trace(c)
yes=c.seed==29 && c.force==50 && ( ...
 (strcmp(c.family,'voltage_nominal') && c.angle==60 && c.voltage==24) || ...
 (strcmp(c.family,'voltage_nominal') && c.angle==90 && c.voltage==12) || ...
 (strcmp(c.family,'current_limit') && c.angle==90 && c.current==2.6 && c.voltage==24) || ...
 (strcmp(c.family,'Kf_unknown') && c.forceFactor>1 && c.voltage==48) || ...
 (strcmp(c.family,'resistance_hypothetical') && c.angle==90 && c.resistanceFactor==2 && any(c.voltage==[12 24 48])));
end

function finish(mdl,worker,previousStorage,previousRecord,previousGen)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
if strcmp(Simulink.sdi.getStorageLocation,fullfile(worker,'sdi')),Simulink.sdi.clear;end
Simulink.sdi.setStorageLocation(previousStorage);Simulink.sdi.setRecordData(previousRecord);
Simulink.fileGenControl('setConfig','config',previousGen);
end

function h=checksum(path)
q=char(39);path=[q,strrep(path,q,[q,char(34),q,char(34),q]),q];
[status,result]=system(['sha256sum -- ' path]);assert(status==0);
h=regexp(result,'^[0-9a-f]{64}','match','once');
end
