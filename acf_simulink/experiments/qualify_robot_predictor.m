function report=qualify_robot_predictor(snapshotFolder,outputFolder,phase,candidateFile)
%QUALIFY_ROBOT_PREDICTOR Test issued-command robot prediction in native blocks.
% All plant trajectories come from the immutable native Simulink snapshot.
% The predictor reads our issued velocity request, never actual robot motion.
% Physical load-cell placement is modeled by Fc - distalMass*surfaceAccel,
% with an exact static gravity tare. That is a deliberately isolated assumption.
if nargin<1,snapshotFolder='/home/tommie/.local/state/actuator-model/qualification-20260930';end
if nargin<2,outputFolder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'validation');end
if nargin<3,phase='smoke';end
if nargin<4,candidateFile=fullfile(outputFolder,'rigid_sensor_qualification_candidate_upper.mat');end
maxNumCompThreads(1);
workerFolder=fullfile(snapshotFolder,'robot_predictor_worker',['phase_' phase]);
privateHelpers=fullfile(workerFolder,'helpers');
for folder={workerFolder,privateHelpers,fullfile(workerFolder,'cache'), ...
        fullfile(workerFolder,'code'),fullfile(workerFolder,'sdi'),fullfile(workerFolder,'traces'),outputFolder}
    if ~isfolder(folder{1}),mkdir(folder{1});end
end
copyfile(fullfile(snapshotFolder,'helpers-v2','*.m'),privateHelpers);
patch_observer_helper(fullfile(privateHelpers,'acf_prepare.m'));
addpath(privateHelpers,'-begin');clear acf_prepare;
candidate=load(candidateFile,'p');p=candidate.p;
if isempty(p.ideal.maxAxialForce),p.ideal.maxAxialForce=inf;end
p.sensor.distalMass_kg=0;p.sensor.distalMass_model_kg=0;
p.robotPrediction=struct('enabled',true,'delay',.08,'responseTime',.06, ...
    'maxSpeed',.02,'maxAcceleration',.5,'residualTime',.06,'residualJerkPower',.02, ...
    'boundedNominalPath',contains(phase,'bounded') || startsWith(phase,'tool'), ...
    'surfaceJerkEnabled',contains(phase,'jerk'),'surfaceSnapPower',1e6, ...
    'surfaceJerkDecayTime',0,'surfaceJerkFeedforwardGain',0);
p.sensor.positionTarget='magnet_gap';
if startsWith(phase,'tool'),p.sensor.positionTarget='tool_relative_A';end
p.sensor.currentReadbackEnabled=contains(phase,'current');
p.sensor.currentNoiseRms_A=.001;p.sensor.currentObserverNoiseRms_A=.001;
p.sensor.currentDelay=.0001;p.sensor.currentModelPower_A2_s=100;
p.robotPrediction.delaySamples=round(p.robotPrediction.delay/p.robot.sampleTime);
p=acf_prepare(p);
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
    'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
oldSDILocation=Simulink.sdi.getStorageLocation;
Simulink.sdi.setStorageLocation(fullfile(workerFolder,'sdi'));
Simulink.sdi.setRecordData(true);
privateModel=fullfile(workerFolder,'acf_robot_prediction_experiment.slx');
copyfile(fullfile(snapshotFolder,'acf_force_control_simple.slx'),privateModel);
load_system(privateModel);
mdl='acf_robot_prediction_experiment';set_param(mdl,'FastRestart','off');
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
cleanup=onCleanup(@()finish(mdl,oldSDILocation)); %#ok<NASGU>
add_sensor_placement(mdl);
add_robot_prediction(mdl);
add_bounded_normal_command(mdl);
if strcmp(p.sensor.positionTarget,'tool_relative_A'),add_toolhead_position(mdl,p.robotPrediction.surfaceJerkEnabled);end
if p.sensor.currentReadbackEnabled,add_current_readback(mdl);end
add_validation_logs(mdl);
save_system(mdl,privateModel);
experimentSource=fullfile(workerFolder,['qualify_robot_predictor_' phase '_source.m']);
copyfile([mfilename('fullpath') '.m'],experimentSource);
delayBlock=[mdl '/Force measurement/Sensor sample delay'];
configs={};labels={};unknowns=[];physicalKfFactors=[];
switch phase
 case {'tool_jerk_sanity','tool_jerk_current_sanity'}
    one=attachment(p,2);one.force.reference=50;one.mechanics.angle=pi/2;
    one.sensor.distalMass_kg=1.4;one.sensor.distalMass_model_kg=1.4;
    one.surface.kind='flat';one.robot.nominalVelocity=0;one.robot.recenterGain=0;
    one.sim.stopTime=1;one.force.sampleTime=.0001;one.sensor.sampleTime=.0001;one.sim.step=.000025;
    one.sensor.filterTime=.0001;one.sensor.delay=.0001;one.sensor.strokeDelay=.0001;
    one.sensor.noiseRms=0;one.sensor.strokeNoiseRms=0;
    one.sensor.currentNoiseRms_A=0;
    one.coil.peakCurrent=3.5;one.coil.currentResponse=.0001;
    one.coil.control.forceNoise_N=.001;one.coil.control.strokeNoise_m=.05e-6;
    one.robotPrediction.residualJerkPower=.0002;one.robotPrediction.surfaceSnapPower=1e6;
    if one.sensor.currentReadbackEnabled
      one.coil.control.forceScale_N=.16;one.coil.control.currentScale_A=.3;
      one.sensor.currentModelPower_A2_s=.01;
    end
    queue(one,'upstream_tool_relative_A_flat_initialization',0,1);
 case 'tool_jerk_current'
    best=load(fullfile(snapshotFolder,'robot_predictor_worker','phase_tool_jerk_ideal','traces','tool_jerk_ideal_012.mat'),'r');
    for snapPower=[1e6 1e7 1e8]
      for currentPower=[.01 1]
        for forceScale=[.16 .32]
          one=best.r.parameters;one.sensor.noiseRms=.02;one.sensor.strokeNoiseRms=.1e-6;
          one.coil.control.forceNoise_N=.02;one.coil.control.strokeNoise_m=.1e-6;
          one.coil.control.forceScale_N=forceScale;one.coil.control.currentScale_A=.3;
          one.sensor.currentReadbackEnabled=true;one.sensor.currentNoiseRms_A=.001;
          one.sensor.currentObserverNoiseRms_A=.001;one.sensor.currentDelay=.0001;
          one.sensor.currentModelPower_A2_s=currentPower;
          one.robotPrediction.surfaceSnapPower=snapPower;one.robotPrediction.surfaceJerkFeedforwardGain=0;
          queue(one,'upstream_tool_relative_A_explicit_drive_current_readback',0,1);
        end
      end
    end
 case 'tool_jerk_ideal'
    for damping=[60 150]
      for snapPower=[1e6 1e7 1e8]
        for jerkGain=[0 1]
          one=attachment(p,2);one.force.reference=50;one.mechanics.angle=pi/2;
          one.mechanics.c1=damping;one.sensor.distalMass_kg=1.4;one.sensor.distalMass_model_kg=1.4;
          one.force.sampleTime=.0001;one.sensor.sampleTime=.0001;one.sim.step=.000025;
          one.sensor.filterTime=.0001;one.sensor.delay=.0001;one.sensor.strokeDelay=.0001;
          one.sensor.noiseRms=0;one.sensor.strokeNoiseRms=0;
          one.coil.peakCurrent=3.5;one.coil.currentResponse=.0001;
          one.coil.control.forceNoise_N=1e-5;one.coil.control.strokeNoise_m=1e-8;
          one.robotPrediction.residualJerkPower=.0002;one.robot.nominalVelocity=.005;
          one.robotPrediction.surfaceSnapPower=snapPower;
          one.robotPrediction.surfaceJerkFeedforwardGain=jerkGain;
          queue(one,'upstream_tool_relative_A_ideal_jerk',0,1);
        end
      end
    end
 case 'tool_jerk_noise'
    best=load(fullfile(snapshotFolder,'robot_predictor_worker','phase_tool_jerk_ideal','traces','tool_jerk_ideal_012.mat'),'r');
    for forceNoise=[.001 .005 .02]
      for positionNoise=[.05 .1 .25]*1e-6
        for snapPower=[1e7 1e8]
          for jerkGain=[0 1]
            one=best.r.parameters;one.sensor.noiseRms=forceNoise;one.sensor.strokeNoiseRms=positionNoise;
            one.coil.control.forceNoise_N=forceNoise;one.coil.control.strokeNoise_m=positionNoise;
            one.robotPrediction.surfaceSnapPower=snapPower;
            one.robotPrediction.surfaceJerkFeedforwardGain=jerkGain;
            queue(one,'upstream_tool_relative_A_sensor_quality_jerk',0,1);
          end
        end
      end
    end
 case 'tool_jerk_cost'
    best=load(fullfile(snapshotFolder,'robot_predictor_worker','phase_tool_jerk_ideal','traces','tool_jerk_ideal_012.mat'),'r');
    for forceScale=[.04 .08 .16 .32]
      for currentScale=[.3 1]
        for snapPower=[1e6 1e7 1e8]
          one=best.r.parameters;one.sensor.noiseRms=.005;one.sensor.strokeNoiseRms=.1e-6;
          one.coil.control.forceNoise_N=.005;one.coil.control.strokeNoise_m=.1e-6;
          one.coil.control.forceScale_N=forceScale;one.coil.control.currentScale_A=currentScale;
          one.robotPrediction.surfaceSnapPower=snapPower;one.robotPrediction.surfaceJerkFeedforwardGain=0;
          queue(one,'upstream_tool_relative_A_noise_cost',0,1);
        end
      end
    end
 case 'tool_jerk_zero'
    for damping=[60 150]
      for rForce=[.001 .003 .01]
        for rPosition=[.05 .1]*1e-6
          for snapPower=[1e5 1e6 1e7]
            for jerkGain=[0 1]
              one=attachment(p,2);one.force.reference=50;one.mechanics.angle=pi/2;
              one.mechanics.c1=damping;one.sensor.distalMass_kg=1.4;one.sensor.distalMass_model_kg=1.4;
              one.force.sampleTime=.0001;one.sensor.sampleTime=.0001;one.sim.step=.000025;
              one.sensor.filterTime=.0001;one.sensor.delay=.0001;one.sensor.strokeDelay=.0001;
              one.sensor.noiseRms=0;one.sensor.strokeNoiseRms=0;
              one.coil.peakCurrent=3.5;one.coil.currentResponse=.0001;
              one.coil.control.forceNoise_N=rForce;one.coil.control.strokeNoise_m=rPosition;
              one.robotPrediction.residualJerkPower=.0002;one.robot.nominalVelocity=.005;
              one.robotPrediction.surfaceSnapPower=snapPower;
              one.robotPrediction.surfaceJerkFeedforwardGain=jerkGain;
              queue(one,'upstream_tool_relative_A_zero_noise_jerk',0,1);
            end
          end
        end
      end
    end
 case 'smoke'
    for mode=[2 3]
      for md=[0 1.4 2]
        if mode==2 && md>1.4,continue;end
        for enabled=[false true]
          one=attachment(p,mode);one.force.reference=50;one.mechanics.angle=pi/2;
          one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
          one.robotPrediction.enabled=enabled;one.robot.nominalVelocity=0;
          queue(one,'sharp_no_robot_motion',0,1);
        end
      end
    end
    for md=[0 1.4]
      for enabled=[false true]
        one=attachment(p,2);one.force.reference=50;one.mechanics.angle=pi/2;
        one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
        one.robotPrediction.enabled=enabled;one.robot.nominalVelocity=.005;
        queue(one,'sharp_nominal_5mm_s',0,1);
      end
    end
 case 'bandwidth'
    for mode=[2 3]
      if mode==2,md=1.4;else,md=2;end
      for strokeNoise=[0 1e-7 3e-7 1e-6]
        for jerk=[7 70 700 7000]
          for assumedStrokeNoise=[3e-7 1e-6 3e-6]
            one=attachment(p,mode);one.force.reference=50;one.mechanics.angle=pi/2;
            one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
            one.sensor.strokeNoiseRms=strokeNoise;one.coil.control.strokeNoise_m=assumedStrokeNoise;
            one.coil.control.surfaceJerkPower=jerk;
            one.robotPrediction.residualJerkPower=.0002;one.robot.nominalVelocity=.005;
            one.sensor.noiseRms=.02;one.sensor.seed=7;
            queue(one,'acceleration_bandwidth_position_noise',.0002,1);
          end
        end
      end
    end
 case 'precision_matched'
    for mode=[2 3]
      if mode==2,mdValues=1.4;else,mdValues=[1.4 2];end
      for md=mdValues
        for strokeNoise=[0 1e-7 2.5e-7]
          for jerk=[300 1000 5000]
            for assumedStrokeNoise=[3e-7 1e-6]
              one=attachment(p,mode);one.force.reference=50;one.mechanics.angle=pi/2;
              one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
              one.sensor.strokeNoiseRms=strokeNoise;one.coil.control.strokeNoise_m=assumedStrokeNoise;
              one.coil.control.surfaceJerkPower=jerk;one.coil.currentResponse=.0001;
              one.sim.step=.00005;one.sensor.delay=.0002;
              one.robotPrediction.residualJerkPower=.0002;one.robot.nominalVelocity=.005;
              one.sensor.noiseRms=.02;one.sensor.seed=7;
              queue(one,'calibrated_force_timing_fine_position_fast_current',0,1);
            end
          end
        end
      end
    end
 case 'precision'
    for mode=[2 3]
      if mode==2,mdValues=1.4;else,mdValues=[1.4 2];end
      for md=mdValues
        for strokeNoise=[0 1e-7 2.5e-7]
          for jerk=[70 300 1000 5000]
            for currentTau=[.0001 .0002 .0004]
              one=attachment(p,mode);one.force.reference=50;one.mechanics.angle=pi/2;
              one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
              one.sensor.strokeNoiseRms=strokeNoise;
              one.coil.control.strokeNoise_m=max(strokeNoise,1e-7);
              one.coil.control.surfaceJerkPower=jerk;one.coil.currentResponse=currentTau;
              one.sim.step=min(.0001,currentTau/2);
              one.robotPrediction.residualJerkPower=.0002;one.robot.nominalVelocity=.005;
              one.sensor.noiseRms=.02;one.sensor.seed=7;
              queue(one,'finer_position_and_faster_current_loop',.0002,1);
            end
          end
        end
      end
    end
 case 'prior'
    for mode=[2 3]
      for md=[1.4 2]
        if mode==2 && md>1.4,continue;end
        for tau=[.02 .06 .2]
          for power=[.0002 .002 .02 .2]
            for stress=[false true]
              one=attachment(p,mode);one.force.reference=50;one.mechanics.angle=pi/2;
              one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
              one.robotPrediction.residualTime=tau;one.robotPrediction.residualJerkPower=power;
              one.robot.nominalVelocity=.005;
              if stress,one.sensor.noiseRms=.02;one.sensor.seed=7;end
              queue(one,'residual_motion_prior',stress*.0002,1);
            end
          end
        end
      end
    end
 case 'range_bounded'
    for mode=[2 3]
      for md=[0 .1 1.4 2]
        if mode==2 && md>1.4,continue;end
        for motion=1:2
          for enabled=[false true]
            for ref=[5 50]
              for angle=[0 90]
                one=attachment(p,mode);one.force.reference=ref;one.mechanics.angle=angle*pi/180;
                one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
                one.robotPrediction.enabled=enabled;one.sensor.noiseRms=.02;one.sensor.seed=7;
                if motion==1,label='sharp_bounded_5mm_s';one.robot.nominalVelocity=.005;
                else,label='flat_offset_recovery';one.surface.kind='flat';one.sim.stopTime=6;
                     one.robot.nominalVelocity=0;one.robot.initialPositionOffset_m=-.004;end
                queue(one,label,.0002,1);
              end
            end
          end
        end
      end
    end
 case 'range'
    for mode=[2 3]
      for md=[0 .1 1.4 2]
        if mode==2 && md>1.4,continue;end
        for motion=1:3
          for enabled=[false true]
            for stress=[false true]
              for ref=[5 50]
                for angle=[0 90]
                  one=attachment(p,mode);one.force.reference=ref;one.mechanics.angle=angle*pi/180;
                  one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
                  one.robotPrediction.enabled=enabled;
                  if motion==1,label='sharp_recenter';one.robot.nominalVelocity=0;one.robot.initialPositionOffset_m=-.004;
                  elseif motion==2,label='sharp_nominal_5mm_s';one.robot.nominalVelocity=.005;
                  else,label='flat_offset_recovery';one.surface.kind='flat';one.sim.stopTime=6;one.robot.nominalVelocity=0;one.robot.initialPositionOffset_m=-.004;end
                  if stress,one.sensor.noiseRms=.02;one.sensor.seed=7;end
                  queue(one,label,stress*.0002,1);
                end
              end
            end
          end
        end
      end
    end
 case {'timing_quick','timing_bounded'}
    for md=[0 .1 1.4]
      for motion=1:2
        for actualDelay=[.08 .12 .2]
          for actualTau=[.06 .09 .12]
            for ref=[5 50]
              one=attachment(p,2);one.force.reference=ref;one.mechanics.angle=pi/2;
              one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
              one.robot.delay=actualDelay;one.robot.responseTime=actualTau;
              one.sensor.noiseRms=.02;one.sensor.seed=7;
              if motion==1,label='sharp_nominal_5mm_s_timing';one.robot.nominalVelocity=.005;
              else,label='robot_error_timing';one.surface.kind='robot_error';one.robot.errorForce=20;one.sim.stopTime=5;end
              queue(one,label,.0002,1);
            end
          end
        end
      end
    end
 case 'timing'
    for mode=[2 3]
      for md=[0 .1 1.4 2]
        if mode==2 && md>1.4,continue;end
        for motion=1:2
          for actualDelay=[.08 .12 .2]
            for actualTau=[.06 .09 .12]
              for ref=[5 50]
                one=attachment(p,mode);one.force.reference=ref;one.mechanics.angle=pi/2;
                one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
                one.robot.delay=actualDelay;one.robot.responseTime=actualTau;
                one.sensor.noiseRms=.02;one.sensor.seed=7;
                if motion==1,label='sharp_nominal_5mm_s_timing';one.robot.nominalVelocity=.005;
                else,label='robot_error_timing';one.surface.kind='robot_error';one.robot.errorForce=20;one.sim.stopTime=5;end
                queue(one,label,.0002,1);
              end
            end
          end
        end
      end
    end
 case 'robust'
    for mode=[2 3]
      for md=[0 .1 1.4 2]
        if mode==2 && md>1.4,continue;end
        for seed=[7 29 1729 31415]
          for motion=[1 2]
            for ref=[5 50]
              for angle=[0 90]
                one=attachment(p,mode);one.force.reference=ref;one.mechanics.angle=angle*pi/180;
                one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md;
                one.sensor.noiseRms=.02;one.sensor.seed=seed;
                if motion==1,label='sharp_nominal_5mm_s_seed';one.robot.nominalVelocity=.005;
                else,label='sharp_recenter_seed';one.robot.nominalVelocity=0;end
                queue(one,label,.0002,1);
              end
            end
          end
        end
      end
    end
    for mode=[2 3]
      for md=[1.4 2]
        if mode==2 && md>1.4,continue;end
        for modelRatio=[.8 1 1.2]
          for physicalKfRatio=[.9 1 1.1]
            one=attachment(p,mode);one.force.reference=50;one.mechanics.angle=pi/2;
            one.sensor.distalMass_kg=md;one.sensor.distalMass_model_kg=md*modelRatio;
            one.robot.nominalVelocity=.005;one.sensor.noiseRms=.02;one.sensor.seed=7;
            queue(one,'distal_mass_and_coil_mismatch',.0002,physicalKfRatio);
          end
        end
      end
    end
 otherwise,error('Unknown phase %s.',phase);
end
rows=struct([]);started=tic;selectedTraces={};
report=struct('date','2026-09-30','phase',phase,'status','running', ...
    'scope','Native Simulink issued-command predictor; B+S=2 kg; sharp bump 1 mm high, 10 mm wide, at 100 mm/s; true force error <=0.5 N throughout recording.', ...
    'snapshot',snapshotFolder,'candidateFile',candidateFile,'privateModel',get_param(mdl,'FileName'), ...
    'experimentSource',experimentSource,'privateObserverHelper',fullfile(privateHelpers,'acf_prepare.m'), ...
    'matlabVersion',version,'baseParameters',p,'rows',rows,'selectedTraces',{selectedTraces}, ...
    'assumptions',{{'Perfect static gravity tare for load-cell placement experiment.', ...
      'Predictor sees only issued robot velocity requests and expected delay, response, speed and acceleration limits.', ...
      'Enabled predictor assumes residual robot acceleration decays with the stated time constant; it does not measure actual motion.', ...
      'No surface-profile preview or physical robot-motion signal enters controller or predictor.', ...
      'Force, position and command timing noise are modeled assumptions, not hardware measurements.'}});
for j=1:numel(configs)
    one=configs{j};one.robotPrediction.delaySamples=round(one.robotPrediction.delay/one.robot.sampleTime);
    assert(abs(one.robotPrediction.delaySamples*one.robot.sampleTime-one.robotPrediction.delay)<1e-12);
    set_param(mdl,'FastRestart','off');
    extra=round(unknowns(j)/one.sensor.sampleTime);
    set_param(delayBlock,'DelayLength',sprintf('p.d.sensorDelaySamples+%d',extra));
    kf=physicalKfFactors(j);path=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil/'];
    set_param([path 'Coil force constant'],'Gain',sprintf('p.coil.forceConstant*%.17g',kf));
    set_param([path 'Back EMF'],'Gain',sprintf('p.coil.forceConstant*%.17g',kf));
    [row,r]=one_run(mdl,one,labels{j},unknowns(j),kf,j);
    if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
    representative=one.force.reference==50 && abs(one.mechanics.angle-pi/2)<1e-12;
    if ~isempty(r) && (strcmp(phase,'smoke') || (startsWith(phase,'tool') && (row.passed || j<=4)) || ...
            (startsWith(phase,'precision') && row.passed) || ...
            (contains(phase,'range') && representative) || ...
            (startsWith(phase,'timing') && representative && any(j==[2 18 20 36 38 54 56 72 74 90 92 108])) || ...
            (one.sensor.distalMass_kg>=1.4 && one.force.reference==50 && j<=32) || ...
            (one.sensor.distalMass_kg==0 && one.force.reference==50 && j<=16))
        path=fullfile(workerFolder,'traces',sprintf('%s_%03d.mat',phase,j));save(path,'r');
        selectedTraces{end+1}=path; %#ok<AGROW>
    end
    fprintf('ROBOT_PREDICTOR %s %d/%d mode=%d md=%.3g predictor=%d noise=%.3g delay=%.3g/%.3g error=%.6g pass=%d %s\n', ...
        phase,j,numel(configs),row.mode,row.distalMass_kg,row.predictorEnabled,row.forceNoiseRms_N, ...
        row.expectedRobotDelay_s,row.actualRobotDelay_s,row.peakForceError_N,row.passed,row.reason);
    report.rows=rows;report.selectedTraces=selectedTraces;report.elapsed_s=toc(started);
    checkpoint(report,outputFolder,phase);
    clear r;Simulink.sdi.clear;
end
report.status='complete';report.count=numel(rows);report.executedCount=sum([rows.executed]);
report.passedCount=sum([rows.passed]);report.forceOnlyPassedCount=sum([rows.forceOnlyPassed]);
report.hiddenInputCheck=check_inputs(mdl);
report.elapsed_s=toc(started);checkpoint(report,outputFolder,phase);
fprintf('ROBOT_PREDICTOR_COMPLETE %s %d runs %.1fs\n',phase,numel(rows),report.elapsed_s);

 function queue(one,label,unknown,kf)
    for field={'currentReadbackEnabled','currentNoiseRms_A','currentObserverNoiseRms_A','currentDelay','currentModelPower_A2_s'}
      if ~isfield(one.sensor,field{1}),one.sensor.(field{1})=p.sensor.(field{1});end
    end
    configs{end+1}=one;labels{end+1}=label;unknowns(end+1)=unknown;physicalKfFactors(end+1)=kf;
 end
end

function q=attachment(q,mode)
q.mechanics.lowerRigid=bitand(mode,1)>0;q.mechanics.upperRigid=bitand(mode,2)>0;
if q.mechanics.lowerRigid,q.mechanics.k1=0;q.mechanics.c1=0;end
if q.mechanics.upperRigid,q.mechanics.k2=0;q.mechanics.c2=0;end
end

function patch_observer_helper(path)
source=fileread(path);
source=replace_exact(source,'p.d.r0 = -p.robot.initialExcessForce/p.d.keq;', ...
 ['p.d.r0 = -p.robot.initialExcessForce/p.d.keq;' newline ...
  'if isfield(p.robot,''initialPositionOffset_m''),p.d.r0=p.d.r0+p.robot.initialPositionOffset_m;end']);
source=replace_exact(source,'m=p.mechanics;drive=p.coil;w=drive.control;', ...
 ['m=p.mechanics;drive=p.coil;w=drive.control;' newline ...
  'distalModel=p.sensor.distalMass_model_kg;predict=p.robotPrediction;' newline ...
  'np=9+double(predict.surfaceJerkEnabled);currentEnabled=p.sensor.currentReadbackEnabled;' newline ...
  'ni=round(p.sensor.currentDelay/p.force.sampleTime)*double(currentEnabled);ny=2+double(currentEnabled);']);
source=replace_exact(source,'p.sensor.filterTime,p.d.sensorDelaySamples,p.d.strokeDelaySamples,weights];', ...
 ['p.sensor.filterTime,p.d.sensorDelaySamples,p.d.strokeDelaySamples,weights,distalModel,' ...
  'predict.enabled,predict.residualTime,predict.residualJerkPower,' ...
  'strcmp(p.sensor.positionTarget,''tool_relative_A''),predict.surfaceJerkEnabled,' ...
  'predict.surfaceSnapPower,predict.surfaceJerkDecayTime,predict.surfaceJerkFeedforwardGain,' ...
  'currentEnabled,ni,p.sensor.currentObserverNoiseRms_A,p.sensor.currentModelPower_A2_s];']);
source=replace_exact(source,'A=zeros(9);Bu=zeros(9,1);Bf=zeros(9,1);', ...
 'A=zeros(np);Bu=zeros(np,1);Bf=zeros(np,1);');
source=replace_exact(source,'Cf=zeros(1,9);Cf([1 2 8])=[m.k1,m.c1,ms];Cu=zeros(1,9);Cu(5)=1;', ...
 ['Cf=zeros(1,np);Cf([1 2 8])=[m.k1,m.c1,ms];Cu=zeros(1,np);Cu(5)=1;' newline ...
  'if strcmp(p.sensor.positionTarget,''tool_relative_A''),Cu(:)=0;Cu([1 5])=[1 -1];end']);
source=replace_exact(source,'E=expm([A,Bu,Bf;zeros(2,11)]*ts);Ad=E(1:9,1:9);Bd=E(1:9,10:11);', ...
 ['if predict.surfaceJerkEnabled,A(8,10)=1;' ...
  'if predict.surfaceJerkDecayTime>0,A(10,10)=-1/predict.surfaceJerkDecayTime;end;end' newline ...
  'Br=A(:,9);Br(9)=0;' newline ...
  'if predict.enabled,A(9,9)=-1/predict.residualTime;else,Br(:)=0;end' newline ...
  'E=expm([A,Bu,Bf,Br;zeros(3,np+3)]*ts);Ad=E(1:np,1:np);Bd=E(1:np,np+(1:3));']);
source=replace_exact(source,'Qc(8,8)=Qc(8,8)+w.surfaceJerkPower;', ...
 ['if predict.surfaceJerkEnabled,Qc(10,10)=predict.surfaceSnapPower;' ...
  'else,Qc(8,8)=Qc(8,8)+w.surfaceJerkPower;end']);
source=replace_exact(source,'Qc(9,9)=Qc(9,9)+w.robotJerkPower;', ...
 ['if predict.enabled,robotPower=predict.residualJerkPower;else,robotPower=w.robotJerkPower;end' newline ...
  'Qc(9,9)=Qc(9,9)+robotPower;' newline ...
  'if currentEnabled,Qc(7,7)=Qc(7,7)+p.sensor.currentModelPower_A2_s;end']);
source=replace_exact(source,"V=expm([A,Qc;zeros(9),-A']*ts);Qd=V(1:9,10:18)*V(1:9,1:9)';", ...
 "V=expm([A,Qc;zeros(np),-A']*ts);Qd=V(1:np,np+(1:np))*V(1:np,1:np)';");
source=replace_exact(source,'nf=max(p.d.sensorDelaySamples-1,0);ng=p.d.strokeDelaySamples;N=10+nf+ng;', ...
 'nf=max(p.d.sensorDelaySamples-1,0);ng=p.d.strokeDelaySamples;fi=np+1;N=fi+nf+ng+ni;');
source=replace_exact(source,'Aa=zeros(N);Aa(1:9,1:9)=Ad;Ba=zeros(N,2);Ba(1:9,:)=Bd;', ...
 'Aa=zeros(N);Aa(1:np,1:np)=Ad;Ba=zeros(N,3);Ba(1:np,:)=Bd;');
source=replace_exact(source,'Qa=zeros(N);Qa(1:9,1:9)=Qd;', ...
 'Qa=zeros(N);Qa(1:np,1:np)=Qd;');
source=replace_exact(source,'filterRow=zeros(1,N);filterRow(1:9)=(1-a)*Cf;filterRow(10)=a;Aa(10,:)=filterRow;', ...
 ['CfMeasured=Cf;CfMeasured(8)=CfMeasured(8)-distalModel;' newline ...
  'filterRow=zeros(1,N);filterRow(1:np)=(1-a)*CfMeasured;filterRow(fi)=a;Aa(fi,:)=filterRow;']);
source=replace_exact(source,'else,C(1,10+nf)=1;end','else,C(1,fi+nf)=1;end');
source=replace_exact(source,'for j=1:nf,Aa(10+j,9+j)=1;end', ...
 'for j=1:nf,Aa(fi+j,fi-1+j)=1;end');
source=replace_exact(source,'firstGap=11+nf;','firstGap=fi+1+nf;');
source=replace_exact(source,'C=zeros(2,N);','C=zeros(ny,N);');
source=replace_exact(source,'if ng==0,C(2,1:9)=Cu;','if ng==0,C(2,1:np)=Cu;');
source=replace_exact(source,'Aa(firstGap,1:9)=Cu;','Aa(firstGap,1:np)=Cu;');
source=replace_exact(source,'S=diag(scales);As=S\(Aa*S);Bs=S\Ba;Cs=C*S;Qs=S\Qa/S'';', ...
 ['if currentEnabled' newline ...
  ' if ni==0,C(3,7)=1;else,firstCurrent=fi+1+nf+ng;' newline ...
  ' Aa(firstCurrent,7)=1;for j=1:ni-1,Aa(firstCurrent+j,firstCurrent+j-1)=1;end;' newline ...
  ' C(3,firstCurrent+ni-1)=1;end' newline ...
  'end' newline ...
  'S=diag(scales);As=S\(Aa*S);Bs=S\Ba;Cs=C*S;Qs=S\Qa/S'';']);
source=replace_exact(source,'scales=[1e-3,0.1,1e-3,0.1,1e-3,0.1,1,10,10,ones(1,N-9)];', ...
 ['scales=[1e-3,0.1,1e-3,0.1,1e-3,0.1,1,10,10];' newline ...
  'if np==10,scales=[scales,1e3];end;scales=[scales,ones(1,N-np)];']);
source=replace_exact(source,'if ng>0,scales(firstGap:end)=1e-3;end', ...
 'if ng>0,scales(firstGap:firstGap+ng-1)=1e-3;end');
source=replace_exact(source,'R=diag([w.forceNoise_N^2,w.strokeNoise_m^2]);', ...
 ['measurementVariances=[w.forceNoise_N^2,w.strokeNoise_m^2];' newline ...
  'if currentEnabled,measurementVariances=[measurementVariances,p.sensor.currentObserverNoiseRms_A^2];end;' newline ...
  'R=diag(measurementVariances);']);
source=replace_exact(source,'c.C=c.C(1:9,:);','c.C=c.C(1:np,:);');
source=replace_exact(source,'c.D=[S*L,zeros(N,2)];c.D=c.D(1:9,:);', ...
 'c.D=[S*L,zeros(N,3)];c.D=c.D(1:np,:);');
source=replace_exact(source,'c.feedback=zeros(1,9);','c.feedback=zeros(1,np);');
source=replace_exact(source,'c.feedback(9)=K(1:5)*arReference;c.force=Cf;', ...
 ['c.feedback(9)=K(1:5)*arReference;c.force=Cf;' newline ...
  'if predict.surfaceJerkEnabled' newline ...
  ' assert(m.upperRigid && ~m.lowerRigid,''Jerk output regulator currently supports upper-rigid/lower-compliant only.'');' newline ...
  ' Eay=[0;1;0;0;0];Aug=[Ab,Eay,zeros(5,1),Bb;zeros(1,5),0,1,0;zeros(2,8)];' newline ...
  ' Z=expm(Aug*ts);Zd=Z(1:5,1:5);Ed=Z(1:5,6:7);BdReg=Z(1:5,8);' newline ...
  ' Md=[Zd-eye(5),BdReg;Cb,0];zAy=Md\[-Ed(:,1);-ms];zJ=Md\[ts*zAy(1:5)-Ed(:,2);0];' newline ...
  ' c.feedback(8)=zAy(6)+K(1:5)*zAy(1:5);' newline ...
  ' c.feedback(10)=predict.surfaceJerkFeedforwardGain*(zJ(6)+K(1:5)*zJ(1:5));' newline ...
  ' c.regulatorAcceleration=zAy;c.regulatorJerk=zJ;' newline ...
  'end']);
fid=fopen(path,'w');assert(fid>0);fprintf(fid,'%s',source);fclose(fid);
end

function source=replace_exact(source,old,replacement)
assert(sum(strfind(source,old)>0)==1,'Coefficient patch anchor missing or ambiguous: %s',old);
source=strrep(source,old,replacement);
end

function add_sensor_placement(mdl)
force=[mdl '/Force measurement'];
add_block('simulink/Ports & Subsystems/In1',[force '/surface_acceleration_m_s2'], ...
    'Port','2','Position',[20 598 50 612]);
add_block('simulink/Math Operations/Gain',[force '/Distal moving mass'], ...
    'Gain','p.sensor.distalMass_kg','Position',[120 577 265 633]);
add_block('simulink/Math Operations/Sum',[force '/Contact force minus distal inertia'], ...
    'Inputs','+-','Position',[75 105 110 155]);
delete_line(force,'true_Fc_N/1','Sensor sampling/1');
add_line(force,'true_Fc_N/1','Contact force minus distal inertia/1','autorouting','on');
add_line(force,'surface_acceleration_m_s2/1','Distal moving mass/1','autorouting','on');
add_line(force,'Distal moving mass/1','Contact force minus distal inertia/2','autorouting','on');
add_line(force,'Contact force minus distal inertia/1','Sensor sampling/1','autorouting','on');
add_block('simulink/Signal Routing/From',[mdl '/Receive surface for physical load-cell placement'], ...
    'GotoTag','surface_motion','Position',[80 980 260 1005]);
add_block('simulink/Signal Routing/Selector',[mdl '/Physical distal-mass acceleration'], ...
    'NumberOfDimensions','1','IndexMode','One-based','IndexOptionArray',{'Index vector (dialog)'}, ...
    'Indices','3','InputPortWidth','3','Position',[310 975 415 1010]);
add_line(mdl,'Receive surface for physical load-cell placement/1','Physical distal-mass acceleration/1','autorouting','on');
add_line(mdl,'Physical distal-mass acceleration/1','Force measurement/2','autorouting','on');
end

function add_robot_prediction(mdl)
pred=[mdl '/Issued-command robot predictor'];
add_block('simulink/Ports & Subsystems/Subsystem',pred,'Position',[525 1100 855 1250]);
Simulink.SubSystem.deleteContents(pred);
add_block('simulink/Ports & Subsystems/In1',[pred '/issued_velocity_request'], ...
    'Position',[20 88 50 102]);
add_block('simulink/Discrete/Delay',[pred '/Expected command delay'], ...
    'DelayLength','p.robotPrediction.delaySamples', ...
    'InitialCondition','p.robot.nominalVelocity*double(~p.robotPrediction.boundedNominalPath)', ...
    'SampleTime','p.robot.sampleTime','Position',[100 70 285 120]);
add_block('simulink/Discontinuities/Saturation',[pred '/Expected speed limit'], ...
    'UpperLimit','p.robotPrediction.maxSpeed','LowerLimit','-p.robotPrediction.maxSpeed', ...
    'Position',[345 70 510 120]);
add_block('simulink/Math Operations/Sum',[pred '/Predicted velocity error'], ...
    'Inputs','+-','Position',[565 70 605 135]);
add_block('simulink/Math Operations/Gain',[pred '/Expected velocity response'], ...
    'Gain','1/p.robotPrediction.responseTime','Position',[660 80 850 130]);
add_block('simulink/Discontinuities/Saturation',[pred '/Expected acceleration limit'], ...
    'UpperLimit','p.robotPrediction.maxAcceleration','LowerLimit','-p.robotPrediction.maxAcceleration', ...
    'Position',[920 80 1105 130]);
add_block('simulink/Continuous/Integrator',[pred '/Predicted velocity'], ...
    'Position',[1170 80 1320 130]);
add_block('simulink/Continuous/Integrator',[pred '/Predicted relative position'], ...
    'Position',[1385 80 1585 130]);
add_block('simulink/Signal Routing/Goto',[pred '/Send predicted velocity'], ...
    'GotoTag','prediction_internal_velocity','Position',[1390 185 1585 210]);
add_block('simulink/Signal Routing/From',[pred '/Receive predicted velocity'], ...
    'GotoTag','prediction_internal_velocity','Position',[340 210 535 235]);
add_block('simulink/Signal Routing/Mux',[pred '/Predicted position velocity acceleration'], ...
    'Inputs','3','Position',[1680 75 1685 345]);
add_block('simulink/Ports & Subsystems/Out1',[pred '/predicted_r_v_a'], ...
    'Position',[1750 203 1780 217]);
connections={ ...
 'issued_velocity_request/1','Expected command delay/1'; ...
 'Expected command delay/1','Expected speed limit/1'; ...
 'Expected speed limit/1','Predicted velocity error/1'; ...
 'Receive predicted velocity/1','Predicted velocity error/2'; ...
 'Predicted velocity error/1','Expected velocity response/1'; ...
 'Expected velocity response/1','Expected acceleration limit/1'; ...
 'Expected acceleration limit/1','Predicted velocity/1'; ...
 'Predicted velocity/1','Predicted relative position/1'; ...
 'Predicted velocity/1','Send predicted velocity/1'; ...
 'Predicted relative position/1','Predicted position velocity acceleration/1'; ...
 'Predicted velocity/1','Predicted position velocity acceleration/2'; ...
 'Expected acceleration limit/1','Predicted position velocity acceleration/3'; ...
 'Predicted position velocity acceleration/1','predicted_r_v_a/1'};
for j=1:size(connections,1),add_line(pred,connections{j,1},connections{j,2},'autorouting','on');end
add_block('simulink/Signal Routing/From',[mdl '/Issued robot command for prediction'], ...
    'GotoTag','robot_velocity_request','Position',[250 1163 455 1187]);
add_line(mdl,'Issued robot command for prediction/1','Issued-command robot predictor/1','autorouting','on');
add_block('simulink/Signal Routing/Selector',[mdl '/Predicted robot acceleration'], ...
    'NumberOfDimensions','1','IndexMode','One-based','IndexOptionArray',{'Index vector (dialog)'}, ...
    'Indices','3','InputPortWidth','3','Position',[905 1135 1065 1190]);
add_line(mdl,'Issued-command robot predictor/1','Predicted robot acceleration/1','autorouting','on');
add_block('simulink/Discrete/Zero-Order Hold',[mdl '/Robot prediction sampling'], ...
    'SampleTime','p.force.sampleTime','Position',[1115 1140 1295 1190]);
add_block('simulink/Math Operations/Gain',[mdl '/Enable robot acceleration prediction'], ...
    'Gain','double(p.robotPrediction.enabled)','Position',[1350 1140 1550 1190]);
add_block('simulink/Signal Routing/Goto',[mdl '/Send command-based robot acceleration'], ...
    'GotoTag','command_based_robot_acceleration','TagVisibility','global','Position',[1610 1153 1865 1177]);
add_line(mdl,'Predicted robot acceleration/1','Robot prediction sampling/1','autorouting','on');
add_line(mdl,'Robot prediction sampling/1','Enable robot acceleration prediction/1','autorouting','on');
add_line(mdl,'Enable robot acceleration prediction/1','Send command-based robot acceleration/1','autorouting','on');
ctrl=[mdl '/Force controller/Voice coil observer'];
set_param([ctrl '/Observer inputs'],'Inputs','5');
set_param([ctrl '/State and disturbance observer'],'D','zeros(9+double(p.robotPrediction.surfaceJerkEnabled),5+double(p.sensor.currentReadbackEnabled))');
set_param([ctrl '/Corrected state estimate'],'Inputs','+++');
add_block('simulink/Signal Routing/From',[ctrl '/Receive issued-command acceleration'], ...
    'GotoTag','command_based_robot_acceleration','Position',[190 850 415 875]);
add_block('simulink/Math Operations/Gain',[ctrl '/Restore total robot acceleration'], ...
    'Gain','[zeros(8,1);1]','Multiplication','Matrix(K*u)','Position',[585 835 805 885]);
add_line(ctrl,'Receive issued-command acceleration/1','Observer inputs/5','autorouting','on');
add_line(ctrl,'Receive issued-command acceleration/1','Restore total robot acceleration/1','autorouting','on');
add_line(ctrl,'Restore total robot acceleration/1','Corrected state estimate/3','autorouting','on');
add_block('simulink/Sinks/To Workspace',[ctrl '/Observer validation log'], ...
    'VariableName','robot_prediction_observer_log','SaveFormat','Timeseries', ...
    'MaxDataPoints','inf','SampleTime','p.force.sampleTime','Position',[1110 710 1330 760]);
add_line(ctrl,'Corrected state estimate/1','Observer validation log/1','autorouting','on');
end

function add_bounded_normal_command(mdl)
path=[mdl '/Surface and nominal path'];
% Programmed commands may use a clock. The controller sees only the command
% issued now, not this clock or the future path/surface profile.
stepChanges=[1 -2 1];
for j=1:3
    add_block('simulink/Sources/Step',[path sprintf('/Normal command change %d',j)], ...
        'Time',num2str(j),'Before','0','After',num2str(stepChanges(j)), ...
        'SampleTime','p.robot.sampleTime','Position',[70 1710+75*j 180 1750+75*j]);
end
add_block('simulink/Math Operations/Sum',[path '/Bounded normal command factor'], ...
    'Inputs','+++','Position',[240 1790 275 1970]);
add_block('simulink/Math Operations/Product',[path '/Normal velocity times bounded program'], ...
    'Position',[365 1840 425 1900]);
add_block('simulink/Sources/Constant',[path '/Bounded program selected'], ...
    'Value','double(p.robotPrediction.boundedNominalPath)','Position',[385 1990 535 2030]);
add_block('simulink/Signal Routing/Switch',[path '/Bounded or constant normal program'], ...
    'Threshold','.5','Position',[650 1825 695 1960]);
delete_line(path,'Nominal robot velocity/1','Nominal path velocity/1');
for j=1:3
 add_line(path,sprintf('Normal command change %d/1',j),sprintf('Bounded normal command factor/%d',j),'autorouting','on');
end
add_line(path,'Bounded normal command factor/1','Normal velocity times bounded program/1','autorouting','on');
add_line(path,'Nominal robot velocity/1','Normal velocity times bounded program/2','autorouting','on');
add_line(path,'Normal velocity times bounded program/1','Bounded or constant normal program/1','autorouting','on');
add_line(path,'Bounded program selected/1','Bounded or constant normal program/2','autorouting','on');
add_line(path,'Nominal robot velocity/1','Bounded or constant normal program/3','autorouting','on');
add_line(path,'Bounded or constant normal program/1','Nominal path velocity/1','autorouting','on');
set_param([mdl '/Robot command delay/Robot command samples'], ...
    'InitialCondition','p.robot.nominalVelocity*double(~p.robotPrediction.boundedNominalPath)');
end

function add_toolhead_position(mdl,jerkEnabled)
sensor=[mdl '/Magnet position measurement'];ctrl=[mdl '/Force controller/Voice coil observer'];
add_block('simulink/Signal Routing/From',[mdl '/Physical bodies for tool-position sensor'], ...
 'GotoTag','plant_log','Position',[60 1400 290 1430]);
add_block('simulink/Signal Routing/Selector',[mdl '/Tool S and body A positions'], ...
 'NumberOfDimensions','1','IndexMode','One-based','IndexOptionArray',{'Index vector (dialog)'}, ...
 'Indices','[7 5]','InputPortWidth','25','Position',[340 1385 560 1445]);
add_block('simulink/Signal Routing/Demux',[mdl '/Separate tool and flange coordinates'], ...
 'Outputs','2','Position',[605 1385 610 1445]);
add_block('simulink/Math Operations/Sum',[mdl '/Physical tool position relative to A'], ...
 'Inputs','+-','Position',[680 1390 720 1440]);
add_line(mdl,'Physical bodies for tool-position sensor/1','Tool S and body A positions/1','autorouting','on');
add_line(mdl,'Tool S and body A positions/1','Separate tool and flange coordinates/1','autorouting','on');
add_line(mdl,'Separate tool and flange coordinates/1','Physical tool position relative to A/1','autorouting','on');
add_line(mdl,'Separate tool and flange coordinates/2','Physical tool position relative to A/2','autorouting','on');
delete_line(mdl,'Receive travel for magnet sensor/1','Magnet position measurement/1');
delete_block([mdl '/Receive travel for magnet sensor']);
add_line(mdl,'Physical tool position relative to A/1','Magnet position measurement/1','autorouting','on');
% These signs are essential. Raw S-A starts at -u0, not +u0.
set_param([sensor '/Measurement delay'],'InitialCondition','-p.d.u0');
set_param([ctrl '/Measured position deviation'],'Bias','p.d.u0');
set_param([sensor '/travel_m'],'Name','tool_position_relative_A_m');
set_param([sensor '/measured_travel_m'],'Name','measured_tool_position_m');
set_param(sensor,'Name','Toolhead position measurement');
set_param([mdl '/Send measured magnet position'],'GotoTag','measured_tool_position');
set_param([mdl '/Receive measured magnet position'],'GotoTag','measured_tool_position', ...
 'Name','Receive measured toolhead position');
outputSize=9+double(jerkEnabled);
if jerkEnabled
 set_param([ctrl '/State and disturbance observer'],'D','zeros(10,5+double(p.sensor.currentReadbackEnabled))');
 set_param([ctrl '/Restore total robot acceleration'],'Gain','[zeros(8,1);1;0]');
 set_param([ctrl '/Estimated gap velocity'],'InputPortWidth','10');
end
add_block('simulink/Signal Routing/Selector',[ctrl '/Estimated coil gap deviation'], ...
 'NumberOfDimensions','1','IndexMode','One-based','IndexOptionArray',{'Index vector (dialog)'}, ...
 'Indices','5','InputPortWidth',num2str(outputSize),'Position',[950 960 1160 1005]);
add_block('simulink/Math Operations/Bias',[ctrl '/Estimated physical actuator travel'], ...
 'Bias','p.mechanics.midTravel+p.d.u0','Position',[1210 960 1415 1005]);
add_block('simulink/Signal Routing/Goto',[ctrl '/Send inferred actuator travel'], ...
 'GotoTag','inferred_actuator_travel','TagVisibility','global','Position',[1460 970 1680 995]);
add_line(ctrl,'Corrected state estimate/1','Estimated coil gap deviation/1','autorouting','on');
add_line(ctrl,'Estimated coil gap deviation/1','Estimated physical actuator travel/1','autorouting','on');
add_line(ctrl,'Estimated physical actuator travel/1','Send inferred actuator travel/1','autorouting','on');
set_param([mdl '/Receive actuator_travel_m 6'],'GotoTag','inferred_actuator_travel', ...
 'Name','Receive inferred actuator travel');
end

function add_validation_logs(mdl)
add_block('simulink/Sinks/To Workspace',[mdl '/Robot predictor validation log'], ...
    'VariableName','robot_prediction_log','SaveFormat','Timeseries', ...
    'MaxDataPoints','inf','SampleTime','p.sim.step','Position',[905 1265 1135 1315]);
add_line(mdl,'Issued-command robot predictor/1','Robot predictor validation log/1','autorouting','on');
add_block('simulink/Signal Routing/From',[mdl '/Actual acceleration for validation only'], ...
    'GotoTag','robot_boundary_acceleration','Position',[1210 1265 1460 1290]);
add_block('simulink/Sinks/To Workspace',[mdl '/Actual robot acceleration validation log'], ...
    'VariableName','robot_prediction_actual_acceleration_log','SaveFormat','Timeseries', ...
    'MaxDataPoints','inf','SampleTime','p.sim.step','Position',[1550 1265 1825 1315]);
add_line(mdl,'Actual acceleration for validation only/1','Actual robot acceleration validation log/1','autorouting','on');
end

function add_current_readback(mdl)
sensor=[mdl '/Drive current readback'];ctrl=[mdl '/Force controller/Voice coil observer'];
add_block('simulink/Ports & Subsystems/Subsystem',sensor,'Position',[930 1510 1210 1640]);
Simulink.SubSystem.deleteContents(sensor);
add_block('simulink/Ports & Subsystems/In1',[sensor '/actual_winding_current_A'], ...
 'Position',[20 88 50 102]);
add_block('simulink/Discrete/Zero-Order Hold',[sensor '/Current sensor sampling'], ...
 'SampleTime','p.force.sampleTime','Position',[110 70 275 120]);
add_block('simulink/Discrete/Delay',[sensor '/Current readback delay'], ...
 'DelayLength','round(p.sensor.currentDelay/p.force.sampleTime)', ...
 'InitialCondition','p.d.coilHoldCurrent','SampleTime','p.force.sampleTime', ...
 'Position',[335 70 505 120]);
add_block('simulink/Sources/Uniform Random Number',[sensor '/Current measurement noise'], ...
 'Minimum','-sqrt(3)','Maximum','sqrt(3)', ...
 'Seed','p.sensor.seed+202','SampleTime','p.force.sampleTime','Position',[335 200 505 245]);
add_block('simulink/Math Operations/Gain',[sensor '/Current sensor RMS noise'], ...
 'Gain','p.sensor.currentNoiseRms_A','Position',[555 200 730 245]);
add_block('simulink/Math Operations/Sum',[sensor '/Delayed current plus readback noise'], ...
 'Inputs','++','Position',[565 65 610 135]);
add_block('simulink/Ports & Subsystems/Out1',[sensor '/measured_current_A'], ...
 'Position',[690 88 720 102]);
add_line(sensor,'actual_winding_current_A/1','Current sensor sampling/1','autorouting','on');
add_line(sensor,'Current sensor sampling/1','Current readback delay/1','autorouting','on');
add_line(sensor,'Current readback delay/1','Delayed current plus readback noise/1','autorouting','on');
add_line(sensor,'Current measurement noise/1','Current sensor RMS noise/1','autorouting','on');
add_line(sensor,'Current sensor RMS noise/1','Delayed current plus readback noise/2','autorouting','on');
add_line(sensor,'Delayed current plus readback noise/1','measured_current_A/1','autorouting','on');
add_block('simulink/Signal Routing/From',[mdl '/Physical winding current for readback sensor'], ...
 'GotoTag','plant_log','Position',[60 1550 315 1580]);
add_block('simulink/Signal Routing/Selector',[mdl '/Physical winding current'], ...
 'NumberOfDimensions','1','IndexMode','One-based','IndexOptionArray',{'Index vector (dialog)'}, ...
 'Indices','14','InputPortWidth','25','Position',[375 1535 635 1595]);
add_block('simulink/Signal Routing/Goto',[mdl '/Send sampled drive current readback'], ...
 'GotoTag','measured_winding_current','TagVisibility','global','Position',[1270 1538 1540 1565]);
add_line(mdl,'Physical winding current for readback sensor/1','Physical winding current/1','autorouting','on');
add_line(mdl,'Physical winding current/1','Drive current readback/1','autorouting','on');
add_line(mdl,'Drive current readback/1','Send sampled drive current readback/1','autorouting','on');
add_block('simulink/Sinks/To Workspace',[mdl '/Drive current readback validation log'], ...
 'VariableName','robot_prediction_current_readback_log','SaveFormat','Timeseries', ...
 'MaxDataPoints','inf','SampleTime','p.force.sampleTime','Position',[1270 1630 1540 1675]);
add_line(mdl,'Drive current readback/1','Drive current readback validation log/1','autorouting','on');

ports=get_param([ctrl '/Observer inputs'],'PortHandles');sources=cell(1,3);
for j=3:5
 line=get_param(ports.Inport(j),'Line');sourcePort=get_param(line,'SrcPortHandle');
 sourcePath=getfullname(get_param(sourcePort,'Parent'));
 sources{j-2}=sprintf('%s/%d',extractAfter(sourcePath,[ctrl '/']),get_param(sourcePort,'PortNumber'));
end
for j=3:5,delete_line(ctrl,sources{j-2},sprintf('Observer inputs/%d',j));end
set_param([ctrl '/Observer inputs'],'Inputs','6');
for j=4:6,add_line(ctrl,sources{j-3},sprintf('Observer inputs/%d',j),'autorouting','on');end
add_block('simulink/Signal Routing/From',[ctrl '/Receive measured drive current'], ...
 'GotoTag','measured_winding_current','Position',[80 1135 345 1165]);
add_block('simulink/Math Operations/Bias',[ctrl '/Current readback deviation'], ...
 'Bias','-p.d.coilHoldCurrent','Position',[405 1125 675 1175]);
add_line(ctrl,'Receive measured drive current/1','Current readback deviation/1','autorouting','on');
add_line(ctrl,'Current readback deviation/1','Observer inputs/3','autorouting','on');
end

function [row,r]=one_run(mdl,p,label,unknown,kf,id)
mode=double(p.mechanics.lowerRigid)+2*double(p.mechanics.upperRigid);
row=struct('id',id,'study',label,'mode',mode,'distalMass_kg',p.sensor.distalMass_kg, ...
 'distalMassModel_kg',p.sensor.distalMass_model_kg,'predictorEnabled',p.robotPrediction.enabled, ...
 'residualTime_s',p.robotPrediction.residualTime,'residualJerkPower',p.robotPrediction.residualJerkPower, ...
 'expectedRobotDelay_s',p.robotPrediction.delay,'actualRobotDelay_s',p.robot.delay, ...
 'expectedRobotResponse_s',p.robotPrediction.responseTime,'actualRobotResponse_s',p.robot.responseTime, ...
 'nominalRobotVelocity_m_s',p.robot.nominalVelocity,'surface',p.surface.kind, ...
 'boundedNominalPath',p.robotPrediction.boundedNominalPath, ...
 'initialRobotPositionOffset_m',optional(p.robot,'initialPositionOffset_m',0), ...
 'forceReference_N',p.force.reference,'angle_deg',p.mechanics.angle*180/pi, ...
 'forceNoiseRms_N',p.sensor.noiseRms,'forceUnknownDelay_s',unknown, ...
 'forceKnownDelay_s',p.sensor.delay,'forceFilter_s',p.sensor.filterTime, ...
 'strokeNoiseRms_m',p.sensor.strokeNoiseRms,'seed',p.sensor.seed,'physicalCoilForceConstantRatio',kf, ...
 'observerSurfaceJerkPower',p.coil.control.surfaceJerkPower, ...
 'observerForceNoise_N',p.coil.control.forceNoise_N,'observerStrokeNoise_m',p.coil.control.strokeNoise_m, ...
 'controllerForceScale_N',p.coil.control.forceScale_N, ...
 'controllerCurrentScale_A',p.coil.control.currentScale_A, ...
 'coilCurrentResponse_s',p.coil.currentResponse, ...
 'controllerSampleTime_s',p.force.sampleTime,'plantStep_s',p.sim.step, ...
 'positionTarget',p.sensor.positionTarget,'surfaceJerkEnabled',p.robotPrediction.surfaceJerkEnabled, ...
 'currentReadbackEnabled',p.sensor.currentReadbackEnabled,'currentReadbackNoiseRms_A',p.sensor.currentNoiseRms_A, ...
 'currentReadbackKnownDelay_s',p.sensor.currentDelay,'currentModelPower_A2_s',p.sensor.currentModelPower_A2_s, ...
 'surfaceSnapPower',p.robotPrediction.surfaceSnapPower, ...
 'surfaceJerkFeedforwardGain',p.robotPrediction.surfaceJerkFeedforwardGain, ...
 'lowerDamping_Ns_m',p.mechanics.c1,'coilPeakCurrent_A',p.coil.peakCurrent, ...
 'executed',false,'passed',false,'forceOnlyPassed',false,'completed',false, ...
 'lostContact',false,'strokeStopHit',false,'springSizingPassed',false, ...
 'peakForceError_N',NaN,'margin_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN, ...
 'travelMin_mm',NaN,'travelMax_mm',NaN,'peakActualRobotAcceleration_m_s2',NaN, ...
 'peakPredictionAccelerationError_m_s2',NaN,'rmsPredictionAccelerationError_m_s2',NaN, ...
 'peakPredictionVelocityError_mm_s',NaN,'peakPredictionRelativePositionError_mm',NaN, ...
 'peakEstimatedSurfaceAccelerationError_m_s2',NaN,'peakEstimatedRobotAccelerationError_m_s2',NaN, ...
 'peakEstimatedWindingCurrentError_A',NaN,'rmsEstimatedWindingCurrentError_A',NaN, ...
 'peakObserverPole',NaN,'endTime_s',NaN,'reason','pending');
r=[];
try
 p=acf_prepare(p);mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);
 acf_select_actuator(mdl,p.actuator.kind);
 set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
 quiet=evalc('out=sim(mdl);'); %#ok<NASGU>
 a=out.get('plant_log');b=out.get('robot_log');c=out.get('surface_log');d=out.get('control_log');
 r.time=a.Time(:);assert(isequal(r.time,b.Time(:),c.Time(:),d.Time(:)));
 r.data=[timeseries_rows(a,25),timeseries_rows(b,2),timeseries_rows(c,3),timeseries_rows(d,5)];
 r.parameters=p;r.names=acf_signals;r.metrics=acf_metrics(r);r.experiment=struct('study',label,'id',id);
 pred=out.get('robot_prediction_log');actual=out.get('robot_prediction_actual_acceleration_log');
 assert(isequal(pred.Time(:),r.time,actual.Time(:)));
 states=out.get('robot_prediction_observer_log');
 r.robotPrediction=struct('time',pred.Time(:),'data',timeseries_rows(pred,3), ...
    'actualAcceleration',timeseries_rows(actual,1),'estimatedTime',states.Time(:), ...
    'estimatedStates',timeseries_rows(states,9+double(p.robotPrediction.surfaceJerkEnabled)));
 if p.sensor.currentReadbackEnabled
   currentReadback=out.get('robot_prediction_current_readback_log');
   r.currentReadback=struct('time',currentReadback.Time(:),'data',timeseries_rows(currentReadback,1));
 end
 gate=out.get('robot_gate_log');gv=timeseries_rows(gate,4);
 r.robotGate=struct('time',gate.Time(:),'ready',gv(:,1),'filteredError_N',gv(:,2), ...
    'forceRate_N_s',gv(:,3),'stableNow',gv(:,4));
 m=r.metrics;row.executed=true;row.passed=m.passed;row.peakForceError_N=m.peakAbsForceError_N;
 row.margin_N=p.force.tolerance-m.peakAbsForceError_N;
 row.forceOnlyPassed=m.completed && m.peakAbsForceError_N<=p.force.tolerance && ...
     ~m.lostContact && ~m.strokeStopHit && ~m.overCurrent;
 row.completed=m.completed;row.lostContact=m.lostContact;row.strokeStopHit=m.strokeStopHit;
 row.springSizingPassed=m.springSizing.passed;row.reason=m.reason;
 row.peakCurrent_A=m.peakCurrent_A;row.rmsCurrent_A=m.rmsCurrent_A;row.peakVoltage_V=m.peakVoltage_V;
 row.travelMin_mm=m.minTravel_mm;row.travelMax_mm=m.maxTravel_mm;row.endTime_s=r.time(end);
 pr=r.robotPrediction;
 accelErr=pr.data(:,3)-pr.actualAcceleration;velErr=pr.data(:,2)-r.data(:,27);
 posErr=pr.data(:,1)-(r.data(:,26)-r.data(1,26));
 row.peakActualRobotAcceleration_m_s2=max(abs(pr.actualAcceleration));
 row.peakPredictionAccelerationError_m_s2=max(abs(accelErr));
 row.rmsPredictionAccelerationError_m_s2=sqrt(mean(accelErr.^2));
 row.peakPredictionVelocityError_mm_s=1000*max(abs(velErr));
 row.peakPredictionRelativePositionError_mm=1000*max(abs(posErr));
 ix=round((pr.estimatedTime-r.time(1))/p.sim.step)+1;
 assert(all(ix>=1 & ix<=numel(r.time)) && all(abs(r.time(ix)-pr.estimatedTime)<p.sim.step/2));
 row.peakEstimatedSurfaceAccelerationError_m_s2=max(abs(pr.estimatedStates(:,8)-r.data(ix,30)));
 row.peakEstimatedRobotAccelerationError_m_s2=max(abs(pr.estimatedStates(:,9)-pr.actualAcceleration(ix)));
 currentError=pr.estimatedStates(:,7)+p.d.coilHoldCurrent-r.data(ix,14);
 row.peakEstimatedWindingCurrentError_A=max(abs(currentError));row.rmsEstimatedWindingCurrentError_A=sqrt(mean(currentError.^2));
 row.peakObserverPole=max(abs(eig(p.d.coilControl.A)));
catch err
 row.reason=['simulation or preparation error: ' err.message];
 fprintf('ROBOT_PREDICTOR_ERROR %d %s\n',id,err.message);
end
end

function d=timeseries_rows(ts,width)
n=numel(ts.Time);d=ts.Data;
if size(d,1)~=n
 assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);
end

d=reshape(d,n,width);
end

function value=optional(s,name,default)
if isfield(s,name),value=s.(name);else,value=default;end
end

function proof=check_inputs(mdl)
ctrl=[mdl '/Force controller/Voice coil observer'];pred=[mdl '/Issued-command robot predictor'];
paths=find_system(ctrl,'LookUnderMasks','all','BlockType','From');
controllerTags=cellfun(@(b)get_param(b,'GotoTag'),paths,'UniformOutput',false);
paths=find_system(pred,'LookUnderMasks','all','BlockType','From');
predictionTags=cellfun(@(b)get_param(b,'GotoTag'),paths,'UniformOutput',false);
forbidden={'surface_motion','robot_motion','robot_boundary_acceleration','robot_position_m','robot_velocity_m_s','true_contact_force_N'};
assert(~any(ismember(controllerTags,forbidden)) && ~any(ismember(predictionTags,forbidden)), ...
    'Forbidden physical truth feeds controller or predictor.');
proof=struct('passed',true,'controllerFromTags',{controllerTags},'predictorFromTags',{predictionTags}, ...
 'predictorRootInputTag','robot_velocity_request','actualAccelerationLogUsedOnlyForValidation',true);
loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);
proof.algebraicLoopCount=numel(loops);assert(isempty(loops),'Prediction experiment has an algebraic loop.');
end

function checkpoint(report,folder,phase)
target=fullfile(folder,['robot_prediction_' phase '.json']);
fid=fopen([target '.tmp'],'w');assert(fid>0);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
movefile([target '.tmp'],target,'f');
writetable(struct2table(report.rows),fullfile(folder,['robot_prediction_' phase '.csv']));
save(fullfile(folder,['robot_prediction_' phase '.mat']),'report');
end

function finish(mdl,oldSDILocation)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
% Only data owned by this separate MATLAB batch process is cleared.
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDILocation);
end
