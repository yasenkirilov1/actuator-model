function [report,traces]=actuator_qualification_actual_sensor_winding(worker)
% Native winding qualification of the frozen B/S-cell, S-A-position candidate.
% All changes are confined to an unsaved private model copy. The outer
% observer/controller remains at Kf=20 N/A and current response=100 us.
modelFile=fullfile(worker,'acf_force_control_simple.slx');mdl='acf_force_control_simple';
candidateFile=fullfile(worker,'candidate.mat');loaded=load(candidateFile,'p');base=loaded.p;
assert(base.mechanics.upperRigid && base.mechanics.lowerRigid);
assert(abs(base.mechanics.mB-.6)<1e-12 && abs(base.mechanics.mS-1.4)<1e-12);
assert(abs(base.coil.forceConstant-20)<1e-12 && abs(base.coil.currentResponse-.0001)<1e-12);
assert(base.coil.directCoulombCorrection);
assert(strcmp(base.sensor.positionTarget,'tool_relative_A'));
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.sim.step=.00005;
base.sensor.noiseRms=.02;base.sensor.strokeNoiseRms=1e-6;
base.surface.kind='sharp_bump';base.surface.height=.001;base.surface.width=.01;
base.surface.scanSpeed=.1;base.surface.startTime=2;base.actuator.kind='voice_coil';
if ~bdIsLoaded(mdl),load_system(modelFile);end
assert(strcmp(get_param(mdl,'FileName'),modelFile));
set_param(mdl,'FastRestart','off');mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
drive=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil'];
paths=cellfun(@(n)[drive '/' n],{'Coil force constant','Back EMF','Resistance','Inverse inductance','Current regulator'},UniformOutput=false);
oldGains=cellfun(@(n)get_param(n,'Gain'),paths,UniformOutput=false);
cleanup=onCleanup(@()restore_private(mdl,mw,original,paths,oldGains)); %#ok<NASGU>
gains={'p.coil.forceConstant*p.windingActual.forceFactor', ...
 'p.coil.forceConstant*p.windingActual.forceFactor', ...
 'p.coil.resistance*p.windingActual.resistanceFactor', ...
 '1/(p.coil.inductance*p.windingActual.inductanceFactor)', ...
 'p.coil.inductance*p.windingActual.inductanceFactor/(p.coil.currentResponse*p.windingActual.currentTauFactor)'};
for j=1:numel(paths),set_param(paths{j},'Gain',gains{j});end
scenarios=struct('name',{'nominal','Kf_minus10_unknown','tau_plus25_unknown','joint_winding'}, ...
 'forceFactor',{1,.9,1,.9},'resistanceFactor',{1,1,1,1.3}, ...
 'inductanceFactor',{1,1,1,1.3},'currentTauFactor',{1,1,1.25,1.25});
% First deliver the selected 3.5 A / 24 V configuration and isolated outer
% model mismatches. Then bound the complete current/voltage matrix with
% nominal and simultaneous tolerance cases; no controller tuning is done.
configs=repmat(struct('current',NaN,'voltage',NaN,'scenario',NaN),0,1);
for s=1:4,configs(end+1)=struct('current',3.5,'voltage',24,'scenario',s);end %#ok<AGROW>
for current=[3 3.5 4]
 for voltage=[18 24 30]
  if current==3.5 && voltage==24,continue;end
  for s=[1 4],configs(end+1)=struct('current',current,'voltage',voltage,'scenario',s);end %#ok<AGROW>
 end
end
prefix=fullfile(fileparts(mfilename('fullpath')),'actuator_qualification_actual_sensor_winding');
rows=struct([]);traces={};started=tic;status='running';
sourceChecksum=checksum([mfilename('fullpath') '.m']);snapshot=jsondecode(fileread(fullfile(worker,'source_snapshot.json')));
if isfile([prefix '_checkpoint.mat'])
 previous=load([prefix '_checkpoint.mat'],'report','traces');
 assert(strcmp(previous.report.sourceStudySha256,sourceChecksum),'Resume recipe changed.');
 assert(strcmp(previous.report.sourceModelSha256,checksum(modelFile)),'Resume model changed.');
 assert(strcmp(previous.report.candidateSha256,checksum(candidateFile)),'Resume candidate changed.');
 rows=previous.report.rows;traces=previous.traces;
end
for configIndex=1:numel(configs)
 config=configs(configIndex);scenario=scenarios(config.scenario);
 q=base;q.coil.peakCurrent=config.current;q.coil.voltage=config.voltage;
 q.windingActual=rmfield(scenario,'name');
 set_param(mdl,'FastRestart','off');
 q=acf_prepare(q);assignin(mw,'p',q);acf_select_actuator(mdl,'voice_coil');
 set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime','4','FastRestart','on');
 groupStart=numel(rows)+1;
 for force=[5 50]
  for angle=[0 pi/2]
   for seed=[7 29 1729 31415]
    if ~isempty(rows) && any([rows.configIndex]==configIndex & [rows.force_N]==force & ...
      abs([rows.angle_deg]-angle*180/pi)<1e-12 & [rows.seed]==seed),continue;end
    one=q;one.force.reference=force;one.mechanics.angle=angle;one.sensor.seed=seed;
    row=empty_row(configIndex,config,scenario,force,angle,seed);row.id=numel(rows)+1;
    try
     quiet=evalc('r=acf_run(mdl,one,false);'); %#ok<NASGU>
     m=r.metrics;row.simulated=true;row.passed=m.passed;row.completed=m.completed;
     row.reason=m.reason;row.peakError_N=m.peakAbsForceError_N;row.peakCurrent_A=m.peakCurrent_A;
     row.peakVoltage_V=m.peakVoltage_V;row.currentReserve_A=config.current-m.peakCurrent_A;
     row.voltageReserve_V=config.voltage-m.peakVoltage_V;
     row.effortSaturationFraction=m.effortSaturationFraction;
     row.voltageSaturationFraction=m.voltageSaturationFraction;
     row.rmsCurrent_A=m.rmsCurrent_A;row.lostContact=m.lostContact;row.strokeStopHit=m.strokeStopHit;
     d=r.data;t=r.time;err=d(:,31)-d(:,1);duration=r.parameters.d.duration;
     startup=t<=.05;prebump=t>=1.5 & t<2;bump=t>=2 & t<=2+duration;
     bumpRecovery=t>=2 & t<=2+duration+.3;final=t>=3;
     row.startupPeakError_N=max(abs(err(startup)));
     row.prebumpPeakError_N=max(abs(err(prebump)));
     row.bumpPeakError_N=max(abs(err(bump)));
     row.bumpRecoveryPeakError_N=max(abs(err(bumpRecovery)));
     row.finalPeakError_N=max(abs(err(final)));
     row.bumpRecoveryPassed=m.completed && row.bumpRecoveryPeakError_N<=one.force.tolerance && ...
      ~m.lostContact && ~m.strokeStopHit && ~m.overCurrent && m.impactCount==0;
     Kf=one.coil.forceConstant*scenario.forceFactor;
     R=one.coil.resistance*scenario.resistanceFactor;
     L=one.coil.inductance*scenario.inductanceFactor;
     tau=one.coil.currentResponse*scenario.currentTauFactor;
     raw=(L/tau)*(min(max(d(:,33),-config.current),config.current)-d(:,14))+R*d(:,14)+Kf*d(:,3);
     row.voltageReconstructionError_V=max(abs(min(max(raw,-config.voltage),config.voltage)-d(:,15)));
     assert(row.voltageReconstructionError_V<1e-8,'Voltage reconstruction mismatch.');
     row.peakRequestedVoltage_V=max(abs(raw));row.peakCurrentCommand_A=max(abs(d(:,33)));
     row.currentCommandClipFraction=mean(abs(d(:,33))>config.current);
     row.holdCurrent_A=(force-2*one.mechanics.gravity*cos(angle))/Kf;
     row.holdCopperLoss_W=row.holdCurrent_A^2*R;row.rmsCopperLoss_W=m.rmsCurrent_A^2*R;
     if config.current==3.5 && config.voltage==24 && force==50 && angle>0
      r.label=sprintf('Actual B/S + S-A / %s / seed%d',scenario.name,seed);
      r.windingActual=scenario;traces{end+1}=r; %#ok<AGROW>
     end
    catch e
     row.reason=[e.identifier ': ' e.message];row.passed=false;
    end
    if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
    if mod(numel(rows),8)==0,checkpoint();end
   end
  end
 end
 set_param(mdl,'FastRestart','off');Simulink.sdi.clear;checkpoint();
 use=[rows.configIndex]==configIndex;
 fprintf('ACTUAL_SENSOR_WINDING config%d/%d %s I%g V%g: full%.9gN bump%.9gN pass%d/%d, %.1fs\n', ...
  configIndex,numel(configs),scenario.name,config.current,config.voltage, ...
  max([rows(use).peakError_N]),max([rows(use).bumpRecoveryPeakError_N]),sum([rows(use).passed]),sum(use),toc(started));
 if groupStart>numel(rows),fprintf('Resumed existing config%d.\n',configIndex);end
end
status='complete';checkpoint();
    function checkpoint
     report=struct('status',status,'date','2026-09-30','simulations',sum([rows.simulated]), ...
      'passedRows',sum([rows.passed]),'bumpRecoveryPassedRows',sum([rows.bumpRecoveryPassed]), ...
      'sourceModel',modelFile,'sourceModelSha256',checksum(modelFile),'sourceStudySha256',sourceChecksum, ...
      'candidateFile',candidateFile,'candidateSha256',checksum(candidateFile),'sourceSnapshot',snapshot, ...
      'rows',rows,'configs',configs,'scenarios',scenarios,'parameters',base,'elapsedSeconds',toc(started), ...
      'scope','Frozen native both-rigid candidate. Actual B/S load cell with 1.4 kg distal tool, actual S-A tool position, no measured current, no surface preview. 5/50 N x 0/90 deg x four seeds. Force RMS 0.02 N, position RMS 1 um. Known 100 us sensor delays/filter, 10 kHz controller, nominal 100 us current response. No additional unmodeled force delay.', ...
      'matrix','3/3.5/4 A x 18/24/30 V nominal and joint winding. Selected 3.5 A/24 V additionally Kf minus10 and current tau plus25 isolated tests. 320 native simulations, no gain tuning.', ...
      'actualVsOuterModel','Only private native winding Gains change. Outer observer/controller remains Kf 20 N/A and tau 100 us. R and L changes are calibrated in the inner drive, and the drive uses actual back EMF; this is not unknown proportional-current DC regulation error.', ...
      'currentDriveEquation','Vreq=(Lactual/tauactual)*(clip(Icommand,Ilimit)-i)+Ractual*i+Kfactual*v_gap; V=clip(Vreq,Vbus); di/dt=(V-Ractual*i-Kfactual*v_gap)/Lactual. In the absence of voltage clipping, di/dt=(clip(Icommand,Ilimit)-i)/tauactual.', ...
      'startupCaveat','Current integrator initial condition stays at nominalKf holding current. Unknown physical Kf changes create a startup force imbalance. Full-run pass includes startup; separately reported bump windows cannot be substituted for the full-run score.', ...
      'evaluationWindows','Full run 0-4 s; startup 0-0.05 s; prebump 1.5-2 s; bump 2-2.1 s; bump and recovery 2-2.4 s; final 3-4 s.', ...
      'limits','Independent Kf/R/L modifications are conditional tolerances, not a real winding family. No thermal state, coil position-dependent Kf, PWM/ADC/current-readback or hardware qualification. Finite force/orientation/seed samples do not prove every continuous operating point.');
     save([prefix '_checkpoint.pending.mat'],'report','traces','-v7.3');
     movefile([prefix '_checkpoint.pending.mat'],[prefix '_checkpoint.mat'],'f');
     fid=fopen([prefix '.json.pending'],'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
     movefile([prefix '.json.pending'],[prefix '.json'],'f');
     writetable(struct2table(rows),[prefix '.csv.pending'],'FileType','text');
     movefile([prefix '.csv.pending'],[prefix '.csv'],'f');
     if strcmp(status,'complete')
      save([prefix '.pending.mat'],'report','traces','-v7.3');movefile([prefix '.pending.mat'],[prefix '.mat'],'f');
     end
    end
end
function row=empty_row(configIndex,config,scenario,force,angle,seed)
row=struct('id',NaN,'configIndex',configIndex,'scenario',scenario.name,'Ilimit_A',config.current,'Vlimit_V',config.voltage, ...
 'force_N',force,'angle_deg',angle*180/pi,'seed',seed,'actualKf_N_A',20*scenario.forceFactor, ...
 'actualR_ohm',4*scenario.resistanceFactor,'actualL_mH',3*scenario.inductanceFactor, ...
 'actualCurrentResponse_ms',.1*scenario.currentTauFactor,'simulated',false,'passed',false,'completed',false, ...
 'bumpRecoveryPassed',false,'peakError_N',NaN,'startupPeakError_N',NaN,'prebumpPeakError_N',NaN, ...
 'bumpPeakError_N',NaN,'bumpRecoveryPeakError_N',NaN,'finalPeakError_N',NaN,'peakCurrent_A',NaN, ...
 'rmsCurrent_A',NaN,'peakVoltage_V',NaN,'currentReserve_A',NaN,'voltageReserve_V',NaN, ...
 'effortSaturationFraction',NaN,'voltageSaturationFraction',NaN,'peakRequestedVoltage_V',NaN, ...
 'voltageReconstructionError_V',NaN,'peakCurrentCommand_A',NaN,'currentCommandClipFraction',NaN, ...
 'holdCurrent_A',NaN,'holdCopperLoss_W',NaN,'rmsCopperLoss_W',NaN,'lostContact',false,'strokeStopHit',false,'reason','');
end
function restore_private(mdl,mw,original,paths,oldGains)
if bdIsLoaded(mdl)
 set_param(mdl,'FastRestart','off');
 for k=1:numel(paths),set_param(paths{k},'Gain',oldGains{k});end
 assignin(mw,'p',original);acf_select_actuator(mdl,original.actuator.kind);
end
end
function h=checksum(path)
q=char(39);path=[q,strrep(path,q,[q,char(34),q,char(34),q]),q];
[status,result]=system(['sha256sum -- ' path]);assert(status==0);h=regexp(result,'^[0-9a-f]{64}','match','once');
end
