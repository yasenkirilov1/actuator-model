function [report,traces]=actuator_qualification_uncalibrated_driver(snapshotFolder,candidateFile)
% Probe unknown winding resistance/back-EMF in the snapshot's proportional
% current drive. This is an additional actuator qualification experiment,
% not a change to the canonical model or to the previously completed study.
modelFile=fullfile(snapshotFolder,'acf_force_control_simple.slx');mdl='acf_force_control_simple';
base=load(candidateFile,'p');base=base.p;
assert(base.mechanics.upperRigid && ~base.mechanics.lowerRigid);
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.actuator.kind='voice_coil';
base.sensor.noiseRms=.02;base.surface.kind='sharp_bump';base.surface.height=.001;
base.surface.width=.01;base.surface.scanSpeed=.1;base.surface.startTime=2;
if ~bdIsLoaded(mdl),load_system(modelFile);end
assert(strcmp(get_param(mdl,'FileName'),modelFile));
set_param(mdl,'FastRestart','off');mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
drive=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil'];
paths=cellfun(@(n)[drive '/' n],{'Coil force constant','Back EMF','Resistance','Inverse inductance','Current regulator'},UniformOutput=false);
oldGains=cellfun(@(n)get_param(n,'Gain'),paths,UniformOutput=false);
robotFrom=[mdl '/Receive actuator_travel_m 6'];oldRobotTag=get_param(robotFrom,'GotoTag');
delayBlock=[mdl '/Force measurement/Sensor sample delay'];oldDelay=get_param(delayBlock,'DelayLength');
actualRes=[drive '/Actual resistance tolerance'];actualEMF=[drive '/Actual back EMF tolerance'];
assert(getSimulinkBlockHandle(actualRes)==-1 && getSimulinkBlockHandle(actualEMF)==-1);
cleanup=onCleanup(@()restore(mdl,mw,original,drive,paths,oldGains,robotFrom,oldRobotTag,delayBlock,oldDelay,actualRes,actualEMF)); %#ok<NASGU>
% Keep current-drive R/back-EMF feedforward at nominal values. Add actual
% winding factors only to the physical inductor-voltage subtraction paths.
delete_line(drive,'Receive resistive_voltage_V 17/1','Inductor voltage/2');
add_block('simulink/Math Operations/Gain',actualRes,'Gain','1.3','Position',[1720 285 1770 325]);
add_line(drive,'Receive resistive_voltage_V 17/1','Actual resistance tolerance/1');
add_line(drive,'Actual resistance tolerance/1','Inductor voltage/2');
delete_line(drive,'Receive back_emf_V 19/1','Inductor voltage/3');
add_block('simulink/Math Operations/Gain',actualEMF,'Gain','.9','Position',[1720 220 1770 260]);
add_line(drive,'Receive back_emf_V 19/1','Actual back EMF tolerance/1');
add_line(drive,'Actual back EMF tolerance/1','Inductor voltage/3');
gains={'p.coil.forceConstant*.9','p.coil.forceConstant','p.coil.resistance', ...
    '1/(p.coil.inductance*1.3)', ...
    'p.coil.inductance*1.3/(p.coil.currentResponse*1.25)-p.coil.resistance*.3'};
for j=1:numel(paths),set_param(paths{j},'Gain',gains{j});end
set_param(robotFrom,'GotoTag','measured_magnet_position');set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');
base=acf_prepare(base);assignin(mw,'p',base);acf_select_actuator(mdl,'voice_coil');
set_param(mdl,'FixedStep',num2str(base.sim.step,17),'StopTime','4');
rows=struct([]);traces={};started=tic;status='running';
prefix=fullfile(fileparts(mfilename('fullpath')),'actuator_qualification_uncalibrated_driver');
for current=[3.5 4]
    set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');
    for seed=[7 29 1729 31415]
        for force=[5 50]
            for angle=[0 pi/2]
                q=base;q.coil.peakCurrent=current;q.coil.voltage=24;
                q.force.reference=force;q.mechanics.angle=angle;q.sensor.seed=seed;
                actualKf=q.coil.forceConstant*.9;actualR=q.coil.resistance*1.3;
                actualL=q.coil.inductance*1.3;actualTau=q.coil.currentResponse*1.25;
                regulatorGain=actualL/actualTau-(actualR-q.coil.resistance);
                assert(regulatorGain>0);
                row=struct('id',numel(rows)+1,'Ilimit_A',current,'Vlimit_V',24,'force_N',force, ...
                    'angle_deg',angle*180/pi,'seed',seed,'actualKf_N_A',actualKf,'actualR_ohm',actualR, ...
                    'actualL_mH',actualL*1000,'actualCurrentResponse_ms',actualTau*1000, ...
                    'currentRegulatorGain_V_A',regulatorGain, ...
                    'smallSignalCurrentDcGain',regulatorGain/(regulatorGain+actualR-q.coil.resistance), ...
                    'simulated',false,'passed',false,'peakError_N',NaN,'peakCurrent_A',NaN,'peakVoltage_V',NaN, ...
                    'peakRequestedVoltage_V',NaN,'currentReserve_A',NaN,'effortSaturationFraction',NaN, ...
                    'voltageSaturationFraction',NaN,'holdCurrent_A',NaN,'holdCurrentCommand_A',NaN, ...
                    'holdCopperLoss_W',NaN,'rmsCopperLoss_W',NaN,'completed',false,'reason','');
                try
                    quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
                    m=r.metrics;row.simulated=true;row.passed=m.passed;
                    row.peakError_N=m.peakAbsForceError_N;row.peakCurrent_A=m.peakCurrent_A;
                    row.peakVoltage_V=m.peakVoltage_V;row.currentReserve_A=current-m.peakCurrent_A;
                    row.effortSaturationFraction=m.effortSaturationFraction;
                    row.voltageSaturationFraction=m.voltageSaturationFraction;row.completed=m.completed;row.reason=m.reason;
                    row.holdCurrent_A=(force-2*q.mechanics.gravity*cos(angle))/actualKf;
                    row.holdCurrentCommand_A=row.holdCurrent_A/row.smallSignalCurrentDcGain;
                    row.holdCopperLoss_W=row.holdCurrent_A^2*actualR;
                    row.rmsCopperLoss_W=m.rmsCurrent_A^2*actualR;
                    d=r.data;raw=regulatorGain*(min(max(d(:,33),-current),current)-d(:,14))+ ...
                        q.coil.resistance*d(:,14)+q.coil.forceConstant*d(:,3);
                    row.peakRequestedVoltage_V=max(abs(raw));
                    assert(max(abs(min(max(raw,-24),24)-d(:,15)))<1e-8,'Voltage reconstruction mismatch.');
                    if force==50 && angle>0,r.label=sprintf('Uncalibrated inner drive / I%g A / seed%d',current,seed);traces{end+1}=r;end %#ok<AGROW>
                catch e,row.passed=false;row.reason=e.message;end
                if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
                if mod(numel(rows),8)==0,checkpoint;end
            end
        end
    end
    set_param(mdl,'FastRestart','off');Simulink.sdi.clear;checkpoint;
    fprintf('UNCALIBRATED_CURRENT I%g: worst%.6gN, pass%d/16\n',current,max([rows(end-15:end).peakError_N]),sum([rows(end-15:end).passed]));
end
status='complete';checkpoint;
    function checkpoint
        report=struct('status',status,'date','2026-09-30','simulations',sum([rows.simulated]),'passedRows',sum([rows.passed]), ...
            'sourceModel',modelFile,'sourceModelSha256',checksum(modelFile),'sourceStudySha256',checksum([mfilename('fullpath') '.m']), ...
            'candidateFile',candidateFile,'candidateSha256',checksum(candidateFile),'rows',rows,'parameters',base,'elapsedSeconds',toc(started), ...
            'scope','Upper-rigid candidate, I3.5/4 A at 24 V; all 5/50 N and 0/90 deg corners; four seeds; 0.02 N RMS force noise; unknown extra 0.2 ms force delay; measured stroke for slow recentering.', ...
            'actualWinding','Simultaneous Kf minus 10 percent, R and L plus 30 percent, actual small-signal current response plus 25 percent. Force observer remains nominal.', ...
            'currentDriverMismatch','Unlike the calibrated-driver cohort, R/back-EMF feedforward stays nominal while physical inductor-voltage subtraction uses actual R/Kb. Native current-drive proportional gain is 6.6 V/A, yielding 0.5 ms actual no-voltage-clipping current response and 0.846154 DC current-command gain. No inner integral current regulation is modeled.', ...
            'interpretation','This is a deliberately uncalibrated proportional current-driver case. Its failure would not prove that a real well-regulated current drive fails, but would show why current accuracy and temperature compensation matter.');
        save([prefix '_checkpoint.pending.mat'],'report','traces','-v7.3');
        movefile([prefix '_checkpoint.pending.mat'],[prefix '_checkpoint.mat'],'f');
        fid=fopen([prefix '.json.pending'],'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
        movefile([prefix '.json.pending'],[prefix '.json'],'f');
        writetable(struct2table(rows),[prefix '.csv.pending'],'FileType','text');
        movefile([prefix '.csv.pending'],[prefix '.csv'],'f');
        save([prefix '.pending.mat'],'report','traces','-v7.3');
        movefile([prefix '.pending.mat'],[prefix '.mat'],'f');
    end
end
function restore(mdl,mw,original,drive,paths,oldGains,robotFrom,oldRobotTag,delayBlock,oldDelay,actualRes,actualEMF)
        set_param(mdl,'FastRestart','off');
        destination=get_param([drive '/Inductor voltage'],'PortHandles');
        added={actualRes,actualEMF};sources={'Receive resistive_voltage_V 17/1','Receive back_emf_V 19/1'};
        for port=2:3
            line=get_param(destination.Inport(port),'Line');if line~=-1,delete_line(line);end
            if getSimulinkBlockHandle(added{port-1})~=-1
                sourcePorts=get_param(added{port-1},'PortHandles');
                line=get_param(sourcePorts.Inport(1),'Line');if line~=-1,delete_line(line);end
                delete_block(added{port-1});
            end
            add_line(drive,sources{port-1},sprintf('Inductor voltage/%d',port));
        end
        for k=1:numel(paths),set_param(paths{k},'Gain',oldGains{k});end
        set_param(robotFrom,'GotoTag',oldRobotTag);set_param(delayBlock,'DelayLength',oldDelay);
        assignin(mw,'p',original);acf_select_actuator(mdl,original.actuator.kind);
end
function h=checksum(path)
q=char(39);path=[q,strrep(path,q,[q,char(34),q,char(34),q]),q];
[status,result]=system(['sha256sum -- ' path]);assert(status==0);h=regexp(result,'^[0-9a-f]{64}','match','once');
end
