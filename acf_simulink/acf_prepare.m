function p = acf_prepare(p)
%ACF_PREPARE Validate edits and calculate equilibrium and controller coefficients.
% Reconstruction uses force, measured S acceleration, relative position and issued commands.
assert(isfield(p,'acquisition')&&p.acquisition.enabled,'The delivered model uses engineered AA acquisition. Use a frozen experiment source for the legacy response.');
assert(isfield(p,'accel')&&p.accel.enabled,'The normal-force reconstruction requires the S-mounted accelerometer.');
% Software controller defaults migrate parameter sets saved before this version.
if isfield(p,'acquisition')&&p.acquisition.enabled
    previousDelay=p.sensor.delay;previousSamples=round(previousDelay/p.sensor.sampleTime);
    if isfield(p,'d')&&isfield(p.d,'sensorDelaySamples'),previousSamples=p.d.sensorDelaySamples;previousDelay=previousSamples*p.sensor.sampleTime;end
    if isfield(p,'d')&&isfield(p.d,'preparedSensorDelay_s'),previousDelay=p.d.preparedSensorDelay_s;end
    if ~isfield(p.accel,'delayOffset_s'),p.accel.delayOffset_s=p.accel.delay-previousDelay;end
    if ~isfield(p.sensor,'strokeDelayOffset_s'),p.sensor.strokeDelayOffset_s=p.sensor.strokeDelay-previousDelay;end
    if ~isfield(p.sensor,'lateralDelayOffsetSamples'),p.sensor.lateralDelayOffsetSamples=p.contact.lateralDelaySamples-previousSamples;end
    if ~isfield(p.acquisition,'motionFrameDelayOffset_s')
        p.acquisition.motionFrameDelayOffset_s=0;
        if isfield(p.acquisition,'identifiedPipelineDelay_s'),p.acquisition.motionFrameDelayOffset_s=p.acquisition.identifiedPipelineDelay_s-previousDelay;end
    end
    p.acquisition.pipelineDelay_s=p.sensor.delay;p.acquisition.forceDelayOffset_s=0;
    p.acquisition.accelerationDelayOffset_s=p.accel.delayOffset_s;
    p.acquisition.positionDelayOffset_s=p.sensor.strokeDelayOffset_s;
    p.acquisition.lateralDelayOffset_s=p.sensor.lateralDelayOffsetSamples*p.acquisition.rawSampleTime_s;
    p.acquisition.identifiedPipelineDelay_s=p.sensor.delay+p.acquisition.motionFrameDelayOffset_s;
    assert(isfinite(p.acquisition.motionFrameDelayOffset_s)&&p.acquisition.identifiedPipelineDelay_s>=0,'Motion-frame delay must be nonnegative.');
end

if ~isfield(p,'acquisition'),p.acquisition=struct;end
defaults=struct('enabled',false,'rawSampleTime_s',1e-5,'antiAliasCutoff_Hz',5100,'antiAliasDamping',1/sqrt(2),'emaTime_s',50e-6,'pipelineDelay_s',20e-6,'forceDelayOffset_s',0,'lateralDelayOffset_s',0,'accelerationDelayOffset_s',0,'positionDelayOffset_s',0,'requireFastVariationGate',true,'identifiedAntiAliasCutoff_Hz',5100,'identifiedEmaTime_s',50e-6,'identifiedPipelineDelay_s',20e-6,'forceAntiAliasScale',1,'lateralAntiAliasScale',1,'accelerationAntiAliasScale',1,'positionAntiAliasScale',1);
for name=fieldnames(defaults)',if ~isfield(p.acquisition,name{1}),p.acquisition.(name{1})=defaults.(name{1});end,end
c=p.acquisition;assert(isscalar(c.enabled)&&any(c.enabled==[false true]));
if c.enabled
 if c.requireFastVariationGate,p.robot.forceGate.enabled=true;p.robot.forceGate.variation.enabled=true;end
 p.sensor.sampleTime=c.rawSampleTime_s;p.sensor.delay=c.pipelineDelay_s+c.forceDelayOffset_s;
 p.accel.delay=c.pipelineDelay_s+c.accelerationDelayOffset_s;p.sensor.strokeDelay=c.pipelineDelay_s+c.positionDelayOffset_s;
 p.contact.lateralDelaySamples=round((c.pipelineDelay_s+c.lateralDelayOffset_s)/p.sensor.sampleTime);
 assert(p.sim.step<=c.rawSampleTime_s && mod(round(p.force.sampleTime/p.sim.step),round(c.rawSampleTime_s/p.sim.step))==0,'Resolve raw acquisition and use integer-rate control.');
 assert(abs(p.contact.lateralDelaySamples*p.sensor.sampleTime-c.pipelineDelay_s-c.lateralDelayOffset_s)<1e-12);
end
p.d.positionSampleTime=p.force.sampleTime;if c.enabled,p.d.positionSampleTime=p.sensor.sampleTime;end
p.d.actualRobotTransportSampleTime_s=p.robot.sampleTime;if c.enabled,p.d.actualRobotTransportSampleTime_s=p.force.sampleTime;end
if ~isfield(p.sensor,'strokeNoiseRms'),p.sensor.strokeNoiseRms=1e-6;end
if ~isfield(p.sensor,'strokeDelay'),p.sensor.strokeDelay=p.force.sampleTime;end
% A B-S cell is gravity-tared before a run. Its dynamic measurement is
% Fc - distalMass*surfaceAcceleration, not the physical contact reaction.
if ~isfield(p.sensor,'distalMass_kg'),p.sensor.distalMass_kg=0;end
if ~isfield(p.sensor,'distalMass_model_kg')
    p.sensor.distalMass_model_kg=p.sensor.distalMass_kg;
end
if ~isfield(p.sensor,'positionModel'),p.sensor.positionModel=0;end
if isfield(p.sensor,'positionTarget')
    p.sensor.positionModel=double(strcmp(p.sensor.positionTarget,'tool_relative_A'));
end
assert(isscalar(p.sensor.positionModel) && any(p.sensor.positionModel==[0 1]), ...
    'positionModel is 0 for magnet gap, or 1 for tool S relative to body A.');
assert(all(isfinite([p.sensor.distalMass_kg,p.sensor.distalMass_model_kg])) && ...
    all([p.sensor.distalMass_kg,p.sensor.distalMass_model_kg]>=0), ...
    'The actual and identified distal sensor masses must be nonnegative.');
% The physical tool mass is edited only through mechanics.mS.
if ~isfield(p.sensor,'distalMassCalibrationScale')
    if p.sensor.distalMass_kg>0
        p.sensor.distalMassCalibrationScale=p.sensor.distalMass_model_kg/p.sensor.distalMass_kg;
    else,p.sensor.distalMassCalibrationScale=1;end
end
if ~isfield(p,'inertial'),error('The delivered model requires measured-force/inertial control settings.');end
if ~isfield(p.inertial,'holderMassCalibrationScale'),p.inertial.holderMassCalibrationScale=1;end
if ~isfield(p.inertial,'forceConstantCalibrationScale'),p.inertial.forceConstantCalibrationScale=1;end
scales=[p.sensor.distalMassCalibrationScale,p.inertial.holderMassCalibrationScale,p.inertial.forceConstantCalibrationScale];
assert(all(isfinite(scales))&&all(scales>0),'Identified mass and force-constant scales must be positive.');
p.sensor.distalMass_kg=p.mechanics.mS;
p.sensor.distalMass_model_kg=p.mechanics.mS*p.sensor.distalMassCalibrationScale;
if ~isfield(p.contact,'commandedScanSpeedScale')
    previousScan=p.surface.scanSpeed;
    if isfield(p,'d')&&isfield(p.d,'preparedScanSpeed_m_s'),previousScan=p.d.preparedScanSpeed_m_s;end
    if isfield(p.contact,'commandedScanSpeed_m_s'),p.contact.commandedScanSpeedScale=p.contact.commandedScanSpeed_m_s/previousScan;
    else,p.contact.commandedScanSpeedScale=1;end
end
assert(isfinite(p.contact.commandedScanSpeedScale)&&p.contact.commandedScanSpeedScale>0,'Identified scan-speed scale must be positive.');
p.contact.commandedScanSpeed_m_s=p.surface.scanSpeed*p.contact.commandedScanSpeedScale;
previousDelay=p.sensor.delay;previousDelaySamples=round(previousDelay/p.sensor.sampleTime);
if isfield(p,'d')&&isfield(p.d,'sensorDelaySamples'),previousDelaySamples=p.d.sensorDelaySamples;previousDelay=previousDelaySamples*p.sensor.sampleTime;end
if isfield(p,'d')&&isfield(p.d,'preparedSensorDelay_s'),previousDelay=p.d.preparedSensorDelay_s;end
if ~isfield(p.sensor,'lateralDelayOffsetSamples')
    if isfield(p.contact,'lateralDelaySamples'),p.sensor.lateralDelayOffsetSamples=p.contact.lateralDelaySamples-previousDelaySamples;
    else,p.sensor.lateralDelayOffsetSamples=0;end
end
if ~isfield(p.accel,'delayOffset_s'),p.accel.delayOffset_s=p.accel.delay-previousDelay;end
if ~isfield(p.sensor,'strokeDelayOffset_s'),p.sensor.strokeDelayOffset_s=p.sensor.strokeDelay-previousDelay;end
p.accel.delay=p.sensor.delay+p.accel.delayOffset_s;
p.sensor.strokeDelay=p.sensor.delay+p.sensor.strokeDelayOffset_s;
if isfield(p.coil,'control'),p.coil=rmfield(p.coil,'control');end
if isfield(p,'d')&&isfield(p.d,'coilControl'),p.d=rmfield(p.d,'coilControl');end
if ~isfield(p.force,'feedforwardEnabled'),p.force.feedforwardEnabled=true;end
assert(isscalar(p.force.feedforwardEnabled)&&any(p.force.feedforwardEnabled==[false true]));
if isfield(p.coil,'strategy')&&strcmp(p.coil.strategy,'observer'),p.coil.strategy='pi';end
if ~isfield(p.coil,'strategy'),p.coil.strategy='pi';end
if ~isfield(p.force,'derivativeFilter'),p.force.derivativeFilter=.001;end
if ~isfield(p.screw,'forceKd'),p.screw.forceKd=0;end
if ~isfield(p.ideal,'forceKd'),p.ideal.forceKd=0;end
assert(any(strcmp(p.coil.strategy,{'pi','pid'})),'Unknown PI/PID strategy.');
assert(all(isfinite([p.inertial.forceKp,p.inertial.forceKi,p.inertial.forceKd,p.inertial.derivativeFilter]))&& ...
    p.inertial.forceKp>=0&&p.inertial.forceKi>0&&p.inertial.forceKd>=0&&p.inertial.derivativeFilter>0, ...
    'Use nonnegative PI/PID gains and positive Ki/filter time; anti-windup requires Ki>0.');
obsolete={'forceKp','forceKi','forceKd','derivativeFilter','directCoulombCorrection'};
for j=1:numel(obsolete),if isfield(p.coil,obsolete{j}),p.coil=rmfield(p.coil,obsolete{j});end,end
if ~isfield(p.robot,'forceGate')
    p.robot.forceGate=struct('enabled',false,'errorBand_N',.2, ...
        'maxRate_N_s',2,'dwellTime',.1,'filterTime',.02);
end
gate=p.robot.forceGate;
if ~isfield(gate,'maxDelayGainProduct'),gate.maxDelayGainProduct=.25;end
p.robot.forceGate=gate;
assert(isscalar(gate.enabled) && any(gate.enabled==[false true]) && ...
    all(isfinite([gate.errorBand_N,gate.maxRate_N_s,gate.dwellTime,gate.filterTime])) && ...
    all([gate.errorBand_N,gate.maxRate_N_s,gate.filterTime]>0) && gate.dwellTime>=0, ...
    'Invalid robot force-settling gate settings.');
assert(isfinite(gate.maxDelayGainProduct) && gate.maxDelayGainProduct>0, ...
    'Robot delay/gain bound must be positive.');
if ~isfield(p.mechanics,'lowerRigid'),p.mechanics.lowerRigid=false;end
if ~isfield(p.mechanics,'upperRigid'),p.mechanics.upperRigid=false;end

if ~isfield(p.robot,'initialPositionOffset_m'),p.robot.initialPositionOffset_m=0;end
assert(isfinite(p.robot.initialPositionOffset_m),'Initial robot position offset must be finite.');
if ~isfield(p,'robotPrediction')
    p.robotPrediction=struct('enabled',false,'delay',p.robot.delay, ...
        'responseTime',p.robot.responseTime,'maxSpeed',p.robot.maxSpeed, ...
        'maxAcceleration',p.robot.maxAcceleration,'boundedNominalPath',false);
end
for name={'residualTime','residualJerkPower'}
    if isfield(p.robotPrediction,name{1}),p.robotPrediction=rmfield(p.robotPrediction,name{1});end
end
predict=p.robotPrediction;
assert(isscalar(predict.enabled) && any(predict.enabled==[false true]) && ...
    all(isfinite([predict.delay,predict.responseTime,predict.maxSpeed, ...
    predict.maxAcceleration])) && ...
    predict.delay>=0 && all([predict.responseTime,predict.maxSpeed, ...
    predict.maxAcceleration]>0), ...
    'Invalid command-driven robot predictor settings.');
assert(isfinite(p.sensor.strokeNoiseRms) && p.sensor.strokeNoiseRms>=0 && ...
    isfinite(p.sensor.strokeDelay) && p.sensor.strokeDelay>=0,'Invalid magnet measurement settings.');
if strcmp(p.actuator.kind,'voice_coil')
    assert(p.acquisition.enabled || abs(p.force.sampleTime-p.sensor.sampleTime)<1e-12, ...
        'Force and inertial sensing require equal sample periods.');
end

assert(any(strcmp(p.actuator.kind, {'leadscrew','voice_coil','ideal_position'})), ...
    'Unknown actuator kind.');
assert(any(strcmp(p.surface.kind, {'flat','bump','sharp_bump','ramp','repeated','robot_error'})), ...
    'Unknown surface scenario.');
if ~isfield(p.mechanics,'externalLowerSpringPair')
    p.mechanics.externalLowerSpringPair=~p.mechanics.lowerRigid;
    if ~isfield(p,'springs')&&isfield(p.sensor,'structuralCell')&&isfield(p.sensor.structuralCell,'externalSpring'),p.mechanics.externalLowerSpringPair=p.sensor.structuralCell.externalSpring;end
end
if ~isfield(p.mechanics,'externalUpperSpringPair'),p.mechanics.externalUpperSpringPair=~p.mechanics.upperRigid;end
m = p.mechanics;
assert(all([m.mA,m.mB,m.mS,p.sim.step,p.sim.stopTime, ...
    p.robot.responseTime,p.robot.maxSpeed,p.robot.maxAcceleration, ...
    p.surface.width,p.surface.scanSpeed,p.screw.lead,p.screw.rotorInertia, ...
    p.screw.resistance,p.screw.inductance,p.screw.torqueConstant, ...
    p.screw.currentResponse,p.screw.peakCurrent,p.screw.voltage, ...
    p.coil.forceConstant,p.coil.resistance,p.coil.inductance, ...
    p.coil.currentResponse,p.coil.peakCurrent,p.coil.voltage, ...
    p.ideal.velocityTime,p.ideal.maxSpeed,p.ideal.maxAcceleration] > 0), ...
    'Masses, time constants and actuator limits must be positive.');
assert(m.k1>=0 && m.k2>=0 && (m.lowerRigid || m.k1>0) && ...
    (m.upperRigid || m.k2>0),'Use lowerRigid/upperRigid for a removed spring with a rigid attachment.');
assert(~(m.lowerRigid || m.upperRigid) || strcmp(p.actuator.kind,'voice_coil') || (strcmp(p.actuator.kind,'leadscrew') && m.upperRigid), ...
    'This actuator does not support the requested rigid topology; choose an explicit compliant reference preset.');
assert(all([m.c1,m.c2,p.robot.delay,p.sensor.delay,p.sensor.filterTime, ...
    p.sensor.noiseRms,p.robot.deadband] >= 0), 'Negative damping, delay or noise.');
assert(isfinite(p.sensor.seed) && p.sensor.seed==round(p.sensor.seed) && ...
    p.sensor.seed>0 && p.sensor.seed<2147483647,'Noise seed must be an integer from 1 to 2147483646.');
assert(p.ideal.maxAxialForce>=0,'Negative ideal actuator force limit.');
assert(m.travelMin < m.midTravel && m.midTravel < m.travelMax, 'Invalid travel.');
p.d.M = m.mA+m.mB;
p.d.alpha = m.mA/p.d.M;
p.d.beta = m.mB/p.d.M;
p.d.mu = m.mA*m.mB/p.d.M;
p.d.g = m.gravity*cos(m.angle);
if m.lowerRigid && m.upperRigid,p.d.keq=inf;
elseif m.lowerRigid,p.d.keq=m.k2;
elseif m.upperRigid,p.d.keq=m.k1;
else,p.d.keq=m.k1*m.k2/(m.k1+m.k2);end
p.d.recenterGain=p.robot.recenterGain;
if gate.enabled
    lag=p.robot.delay+p.robot.responseTime+gate.dwellTime+gate.filterTime;
    p.d.recenterGain=min(p.d.recenterGain,gate.maxDelayGainProduct/max(lag,p.robot.sampleTime));
end
p.d.ell = p.screw.lead/(2*pi);
p.d.q10 = p.force.reference-m.mS*p.d.g;
p.d.q20 = p.force.reference-(p.d.M+m.mS)*p.d.g;
p.d.holdForce = p.force.reference-(m.mB+m.mS)*p.d.g;
if isfield(p,'springs')
    pack=p.springs;
    assert(isfinite(pack.maxCombinedLength_m) && pack.maxCombinedLength_m>0 && ...
        pack.maxCombinedLength_m<=0.100,'Combined spring length must not exceed 100 mm.');
    assert(isfinite(pack.commonPreload_N) && pack.commonPreload_N>0 && ...
        pack.workingClearance_m>=0 && pack.unseatingClearance_m>=0,'Invalid spring preload or clearance.');
    totalFree=0;
    for j=1:2
        links={'lower','upper'};one=pack.(links{j});stiffness=[m.k1,m.k2];
        rigid=[m.lowerRigid,m.upperRigid];if rigid(j),continue;end
        assert(all(isfinite([one.rate_N_m,one.freeLength_m,one.freeLengthTolerance_m, ...
            one.minWorkingLength_m,one.maxCyclicStroke_m,one.forceEnvelope_N])) && ...
            one.rate_N_m>0 && one.freeLength_m>one.minWorkingLength_m && ...
            one.freeLengthTolerance_m>=0 && one.minWorkingLength_m>0 && ...
            one.maxCyclicStroke_m>0 && one.forceEnvelope_N>0,'Invalid spring catalogue dimensions.');
        assert(abs(stiffness(j)-2*one.rate_N_m)<1e-8*stiffness(j), ...
            'Each guided pair has twice its single-spring rate. Update springs.lower/upper when changing k1/k2.');
        totalFree=totalFree+2*(one.freeLength_m+one.freeLengthTolerance_m);
    end
    assert(totalFree<=pack.maxCombinedLength_m+1e-12, ...
        'Combined free spring length, including tolerance, exceeds the length limit.');
end
p.d.r0 = -p.robot.initialExcessForce/p.d.keq+p.robot.initialPositionOffset_m;
p.d.u0 = p.d.r0;
p.d.z0 = p.d.alpha*p.d.u0;
p.d.positionMeasurementBias=-m.midTravel-p.d.u0;
p.d.positionMeasurementInitial=m.midTravel+p.d.u0;
if p.sensor.positionModel==1
    p.d.positionMeasurementBias=-m.midTravel+p.d.u0;
    p.d.positionMeasurementInitial=m.midTravel-p.d.u0;
end
assert(m.midTravel+p.d.u0 > m.travelMin && ...
    m.midTravel+p.d.u0 < m.travelMax, 'Initial compensation exceeds travel.');
p.d.forceTicks = ticks(p.force.sampleTime,p.sim.step,'force.sampleTime');
p.d.sensorTicks = ticks(p.sensor.sampleTime,p.sim.step,'sensor.sampleTime');
p.d.robotTicks = ticks(p.robot.sampleTime,p.sim.step,'robot.sampleTime');
p.d.sensorDelaySamples = ticks(p.sensor.delay,p.sensor.sampleTime,'sensor.delay',true);
p.contact.lateralDelaySamples=p.d.sensorDelaySamples+p.sensor.lateralDelayOffsetSamples;
assert(p.contact.lateralDelaySamples>=0&&p.contact.lateralDelaySamples==round(p.contact.lateralDelaySamples),'Lateral force delay must be a nonnegative integer number of samples.');
p.d.preparedScanSpeed_m_s=p.surface.scanSpeed;p.d.preparedSensorDelay_s=p.sensor.delay;
p.d.robotDelaySamples = ticks(p.robot.delay,p.d.actualRobotTransportSampleTime_s,'robot.delay',true);
p.robotPrediction.delaySamples=ticks(predict.delay,p.robot.sampleTime,'robotPrediction.delay',true);
p.d.strokeDelaySamples = ticks(p.sensor.strokeDelay,p.d.positionSampleTime,'sensor.strokeDelay',true);
ticks(p.sim.stopTime,p.sim.step,'sim.stopTime');
assert(p.sim.step <= min([p.screw.currentResponse,p.coil.currentResponse, ...
    p.ideal.velocityTime])/2, 'Reduce sim.step to resolve drive dynamics.');
assert(p.surface.startTime >= 0 && p.force.reference > 0 && p.force.tolerance > 0);
p.d.duration = p.surface.width/p.surface.scanSpeed;
if strcmp(p.surface.kind,'repeated')
    assert(p.surface.repeatPeriod >= p.d.duration, 'Repeated bumps overlap.');
end
p.d.identifiedHolderMass_kg=m.mB*p.inertial.holderMassCalibrationScale;
p.d.identifiedMovingMass_kg=p.d.identifiedHolderMass_kg+p.sensor.distalMass_model_kg;
p.d.coilForceConstant_model_N_A=p.coil.forceConstant*p.inertial.forceConstantCalibrationScale;
p.d.screwTorqueConstant_model_Nm_A=p.screw.torqueConstant*p.inertial.forceConstantCalibrationScale;
p.d.coilHoldingCurrentCommand_A=(p.force.reference-p.d.identifiedMovingMass_kg*p.d.g)/p.d.coilForceConstant_model_N_A;
p.d.screwHoldingCurrentCommand_A=p.d.ell*(p.force.reference-p.d.identifiedMovingMass_kg*p.d.g)/p.d.screwTorqueConstant_model_Nm_A;
if isfield(p.inertial,'lowerLoadInverse')
    c=p.inertial.lowerLoadInverse;
    if ~isfield(c,'stiffnessCalibrationScale'),if m.k1>0,c.stiffnessCalibrationScale=c.identifiedStiffness_N_m/m.k1;else,c.stiffnessCalibrationScale=1;end,end
    if ~isfield(c,'dampingCalibrationScale'),if m.c1>0,c.dampingCalibrationScale=c.identifiedDamping_Ns_m/m.c1;else,c.dampingCalibrationScale=1;end,end
    if ~isfield(c,'dampingCalibrationOffset_Ns_m'),c.dampingCalibrationOffset_Ns_m=0;if m.c1==0,c.dampingCalibrationOffset_Ns_m=c.identifiedDamping_Ns_m;end,end
    assert(all(isfinite([c.stiffnessCalibrationScale,c.dampingCalibrationScale,c.dampingCalibrationOffset_Ns_m]))&&c.stiffnessCalibrationScale>0&&c.dampingCalibrationScale>=0);
    if m.lowerRigid,c.enabled=false;else,c.identifiedStiffness_N_m=m.k1*c.stiffnessCalibrationScale;end
    c.identifiedDamping_Ns_m=m.c1*c.dampingCalibrationScale+c.dampingCalibrationOffset_Ns_m;
    c.identifiedMassB_kg=p.d.identifiedHolderMass_kg;c.identifiedForceConstant_N_A=p.d.coilForceConstant_model_N_A;
    p.inertial.lowerLoadInverse=c;
end
% Native plant initial states retain the actual holding current below.
p.d.screwHoldCurrent = p.d.ell*p.d.holdForce/p.screw.torqueConstant;
p.d.coilHoldCurrent = p.d.holdForce/p.coil.forceConstant;
p.d.screwMovingMass=p.d.mu;if m.upperRigid,p.d.screwMovingMass=m.mB;end;J = p.screw.rotorInertia+p.d.screwMovingMass*p.d.ell^2;
p.d.speedKp = 2*p.screw.speedDamping*p.screw.speedBandwidth*J/p.screw.torqueConstant;
p.d.speedKi = p.screw.speedBandwidth^2*J/p.screw.torqueConstant;
switch p.actuator.kind
    case 'leadscrew'
        p.d.forceKp = p.screw.forceKp/p.d.ell;
        p.d.forceKi = p.screw.forceKi/p.d.ell;
        p.d.forceKd = p.screw.forceKd/p.d.ell*double(strcmp(p.coil.strategy,'pid'));
        p.d.derivativeFilter = p.force.derivativeFilter;
        p.d.forceFeedforward = 0;
        p.d.commandMax = p.screw.maxLinearSpeedRequest/p.d.ell;
        p.d.commandMin = -p.d.commandMax;
        holdCurrent = p.d.screwHoldCurrent;
        availableCurrent = p.screw.peakCurrent;
        holdVoltage = abs(holdCurrent*p.screw.resistance);
        availableVoltage = p.screw.voltage;
    case 'voice_coil'
        p.d.forceKp=p.inertial.forceKp;p.d.forceKi=p.inertial.forceKi;
        p.d.forceKd=p.inertial.forceKd*double(strcmp(p.coil.strategy,'pid'));
        p.d.derivativeFilter=p.inertial.derivativeFilter;
        p.d.forceFeedforward = p.d.coilHoldCurrent;
        p.d.commandMax = p.coil.peakCurrent;
        p.d.commandMin = -p.coil.peakCurrent;
        holdCurrent = p.d.coilHoldCurrent;
        availableCurrent = p.coil.peakCurrent;
        holdVoltage = abs(holdCurrent*p.coil.resistance);
        availableVoltage = p.coil.voltage;
    otherwise
        p.d.forceKp = p.ideal.forceKp;
        p.d.forceKi = p.ideal.forceKi;
        p.d.forceKd = p.ideal.forceKd*double(strcmp(p.coil.strategy,'pid'));
        p.d.derivativeFilter = p.force.derivativeFilter;
        p.d.forceFeedforward = p.d.u0;
        p.d.commandMin = m.travelMin-m.midTravel;
        p.d.commandMax = m.travelMax-m.midTravel;
        holdCurrent = p.d.holdForce; availableCurrent = p.ideal.maxAxialForce;
        holdVoltage = 0; availableVoltage = 1;
end
assert(abs(holdCurrent) <= availableCurrent && holdVoltage <= availableVoltage, ...
    'Actuator cannot hold the starting equilibrium: increase its limits or change preload.');

if isfield(p.screw,'currentForceKp')
 if ~isfield(p.screw,'accelerationPrediction_s'),p.screw.accelerationPrediction_s=0;p.screw.accelerationDerivativeFilter_s=.0003;end
 ad=exp(-p.force.sampleTime/p.screw.accelerationDerivativeFilter_s);p.d.screwAccelerationPrediction=struct('derivativeNumerator',(1-ad)/p.force.sampleTime*[1,-1],'derivativeDenominator',[1,-ad]);
 if ~isfield(p.screw,'accelerationFeedforwardFilter_s'),p.screw.accelerationFeedforwardFilter_s=0;end;af=p.screw.accelerationFeedforwardFilter_s;dt=p.force.sampleTime;
 if af>0,a=exp(-dt/af);num=[1-a,0];den=[1,-a];else,num=[1,0];den=[1,0];end;p.d.screwAccelerationFeedforwardFilter=struct('numerator',num,'denominator',den);
 c=p.screw.currentNotch;c.frequency_Hz=sqrt(m.k1/(m.mB+p.screw.rotorInertia/p.d.ell^2))/(2*pi);
 if c.enabled
  w=tan(pi*c.frequency_Hz*p.force.sampleTime);den=[1+2*c.poleDamping*w+w^2,2*(w^2-1),1-2*c.poleDamping*w+w^2];num=[1+2*c.zeroDamping*w+w^2,2*(w^2-1),1-2*c.zeroDamping*w+w^2];num=num/den(1);den=den/den(1);
 else,num=[1,0,0];den=[1,0,0];end
 p.screw.currentNotch=c;p.d.screwCurrentNotch=struct('numerator',num,'denominator',den,'initialStates',ones(2,1)*p.d.screwHoldCurrent/sum(den));
 if ~isfield(p.screw,'feedbackFilter'),p.screw.feedbackFilter=struct('kind','common','zeroDamping',.01,'poleDamping',.3);end
 if ~isfield(p.inertial,'feedbackFilter'),p.inertial.feedbackFilter=struct('enabled',false,'cutoff_Hz',800,'damping',1/sqrt(2));end
 f=p.inertial.feedbackFilter;if strcmp(p.screw.feedbackFilter.kind,'lowpass'),f=p.screw.feedbackFilter;f.enabled=true;elseif strcmp(p.screw.feedbackFilter.kind,'bypass'),f.enabled=false;end
 if f.enabled,w=tan(pi*f.cutoff_Hz*p.force.sampleTime);den=[1+2*f.damping*w+w^2,2*(w^2-1),1-2*f.damping*w+w^2];num=w^2*[1,2,1]/den(1);den=den/den(1);else,num=[1,0,0];den=[1,0,0];end
 p.d.screwFeedbackFilter=struct('numerator',num,'denominator',den,'initialStates',zeros(2,1));
 if strcmp(p.screw.feedbackFilter.kind,'notch')
  f=p.screw.feedbackFilter;w=tan(pi*c.frequency_Hz*p.force.sampleTime);den=[1+2*f.poleDamping*w+w^2,2*(w^2-1),1-2*f.poleDamping*w+w^2];num=[1+2*f.zeroDamping*w+w^2,2*(w^2-1),1-2*f.zeroDamping*w+w^2];num=num/den(1);den=den/den(1);
  p.d.screwFeedbackFilter=struct('numerator',num,'denominator',den,'initialStates',zeros(2,1));
 end
 if strcmp(p.screw.controlStrategy,'current'),p.d.speedKp=0;p.d.speedKi=0;end
 if isfield(p.inertial,'lowerLoadInverse')
  c=p.inertial.lowerLoadInverse;a=2/p.force.sampleTime;w=a*tan(pi*c.cutoff_Hz*p.force.sampleTime);quadratic=[a*a+2*c.filterDamping*w*a+w*w,2*(w*w-a*a),a*a-2*c.filterDamping*w*a+w*w];
  effectiveB=c.identifiedMassB_kg+p.screw.rotorInertia/p.d.ell^2;kax=p.d.screwTorqueConstant_model_Nm_A/p.d.ell;num=(effectiveB/kax)*w*w*a*a*[1,-2,1];k=c.identifiedStiffness_N_m;d=c.identifiedDamping_Ns_m;
  if d>0,num=conv(num,[1,1]);den=conv([d*a+k,-d*a+k],quadratic);else,den=k*quadratic;num=[num,0];den=[den,0];end
  c.numerator=num/den(1);c.denominator=den/den(1);c.identifiedMassB_kg=effectiveB;c.identifiedForceConstant_N_A=kax;p.d.screwLowerLoadInverse=c;
 end
end

if isfield(p,'inertial')
 if ~isfield(p.inertial,'guideVelocityPrediction_s'),p.inertial.guideVelocityPrediction_s=0;end
 if ~isfield(p.inertial,'currentNotch'),p.inertial.currentNotch=struct('enabled',false,'frequency_Hz',sqrt(64e6/.6)/(2*pi),'zeroDamping',.01,'poleDamping',.3);end
 c=p.inertial.currentNotch;dt=p.force.sampleTime;assert(c.frequency_Hz>0&&c.frequency_Hz<.45/dt&&c.zeroDamping>0&&c.poleDamping>0);
 if c.enabled
  w=tan(pi*c.frequency_Hz*dt);den=[1+2*c.poleDamping*w+w^2,2*(w^2-1),1-2*c.poleDamping*w+w^2];num=[1+2*c.zeroDamping*w+w^2,2*(w^2-1),1-2*c.zeroDamping*w+w^2];num=num/den(1);den=den/den(1);
 else,num=[1,0,0];den=[1,0,0];end
 p.d.currentNotch=struct('numerator',num,'denominator',den,'initialStates',ones(2,1)*p.d.coilHoldCurrent/sum(den));
end
if isfield(p,'inertial')
 if ~isfield(p.inertial,'feedbackFilter'),p.inertial.feedbackFilter=struct('enabled',false,'cutoff_Hz',800,'damping',1/sqrt(2));end
 c=p.inertial.feedbackFilter;dt=p.force.sampleTime;assert(c.cutoff_Hz>0&&c.cutoff_Hz<.45/dt&&c.damping>0);
 if c.enabled,w=tan(pi*c.cutoff_Hz*dt);den=[1+2*c.damping*w+w^2,2*(w^2-1),1-2*c.damping*w+w^2];num=w^2*[1,2,1]/den(1);den=den/den(1);else,num=[1,0,0];den=[1,0,0];end
 p.d.feedbackFilter=struct('numerator',num,'denominator',den,'initialStates',zeros(2,1));
end
if isfield(p,'inertial')
 if ~isfield(p.robot.forceGate,'variation'),p.robot.forceGate.variation=struct('enabled',false,'meanTime',.1,'varianceTime',.02,'rmsLimit_N',.08);end
 c=p.robot.forceGate.variation;dt=p.force.sampleTime;assert(c.meanTime>0&&c.varianceTime>0&&c.rmsLimit_N>0);am=exp(-dt/c.meanTime);av=exp(-dt/c.varianceTime);
 p.d.variationGate=struct('meanNumerator',[1-am,0],'meanDenominator',[1,-am],'varianceNumerator',[1-av,0],'varianceDenominator',[1,-av],'dwellSamples',1+ceil(p.robot.forceGate.dwellTime/dt));
 if ~isfield(p.inertial,'accelerationPrediction_s'),p.inertial.accelerationPrediction_s=0;end
 if ~isfield(p.inertial,'accelerationDerivativeFilter_s'),p.inertial.accelerationDerivativeFilter_s=.0003;end
 assert(p.inertial.accelerationPrediction_s>=0&&p.inertial.accelerationDerivativeFilter_s>0);ad=exp(-dt/p.inertial.accelerationDerivativeFilter_s);
 p.d.accelerationPrediction=struct('derivativeNumerator',(1-ad)/dt*[1,-1],'derivativeDenominator',[1,-ad]);
end
if isfield(p,'contact')
 if ~isfield(p.contact,'geometryMode'),p.contact.geometryMode='direct_lateral';end
 if ~isfield(p.contact,'dragRatioFilter_s'),p.contact.dragRatioFilter_s=.005;end
 if ~isfield(p.contact,'dragRatioMinForce_N'),p.contact.dragRatioMinForce_N=1;end
 assert(any(strcmp(p.contact.geometryMode,{'direct_lateral','measured_drag_ratio'}))&&p.contact.dragRatioFilter_s>0&&p.contact.dragRatioMinForce_N>0);
 a=exp(-p.force.sampleTime/p.contact.dragRatioFilter_s);p.d.dragRatio=struct('memoryWeight',a,'inputWeight',1-a);
end
if isfield(p,'accel') && p.accel.enabled
 dt=p.force.sampleTime;rho=exp(-2*pi*p.inertial.positionBandwidth_Hz*dt);
 Fm=[1,dt;0,1];Lm=[1-rho^2;(1-rho)^2/dt];Jm=eye(2)-Lm*[1,0];Ga=[dt^2/2;dt];
 p.d.toolMotionObserver=struct('A',Fm*Jm,'B',[Fm*Lm,Ga,-Ga],'C',Jm,'D',[Lm,zeros(2,2)]);
 p.accel.delaySamples=round(p.accel.delay/p.sensor.sampleTime);
 assert(abs(p.accel.delay/p.sensor.sampleTime-p.accel.delaySamples)<1e-9);
end
if isfield(p,'inertial') && isfield(p.inertial,'lowerLoadInverse')
 c=p.inertial.lowerLoadInverse;a=2/p.force.sampleTime;w=a*tan(pi*c.cutoff_Hz*p.force.sampleTime);
 quadratic=[a*a+2*c.filterDamping*w*a+w*w,2*(w*w-a*a),a*a-2*c.filterDamping*w*a+w*w];
 num=(c.identifiedMassB_kg/c.identifiedForceConstant_N_A)*w*w*a*a*[1,-2,1];
 k=c.identifiedStiffness_N_m;d=c.identifiedDamping_Ns_m;
 if d>0,num=conv(num,[1,1]);den=conv([d*a+k,-d*a+k],quadratic);
 else,den=k*quadratic;num=[num,0];den=[den,0];end
 c.numerator=num/den(1);c.denominator=den/den(1);p.inertial.lowerLoadInverse=c;
end

c=p.acquisition;h=c.rawSampleTime_s;assert(h>0&&c.antiAliasCutoff_Hz>0&&c.antiAliasCutoff_Hz<.45/h&&c.antiAliasDamping>0&&c.emaTime_s>0&&c.pipelineDelay_s>0);
w=tan(pi*c.antiAliasCutoff_Hz*h);aaDen=[1+2*c.antiAliasDamping*w+w*w,2*(w*w-1),1-2*c.antiAliasDamping*w+w*w];
aaNum=w*w*[1,2,1]/aaDen(1);aaDen=aaDen/aaDen(1);alpha=exp(-h/c.emaTime_s);
num=conv(aaNum,[1-alpha,0]);den=conv(aaDen,[1,-alpha]);
impulse=zeros(20000,1);impulse(1)=1;response=filter(num,den,impulse);energy=sum(response.^2);assert(energy>0&&energy<1&&max(abs(roots(den)))<1);
p.d.acquisition=struct('numerator',num,'denominator',den,'steadyStatePerUnitInput',ones(3,1)/sum(den),'impulseEnergy',energy,'noiseInputScale',1/sqrt(energy),'rawSampleTime_s',h,'nominalLowFrequencyDelay_s',sum((0:3).*num)/sum(num)*h-sum((0:3).*den)/sum(den)*h);
% Actual sensor tolerance must not change the identified command-motion frame.
p.d.acquisition.responses=struct;channels={'force','lateral','acceleration','position','frame'};
for j=1:numel(channels)
 channel=channels{j};fc=c.antiAliasCutoff_Hz;te=c.emaTime_s;z=c.antiAliasDamping;
 if strcmp(channel,'frame'),fc=c.identifiedAntiAliasCutoff_Hz;te=c.identifiedEmaTime_s;z=defaults.antiAliasDamping;else,fc=fc*c.([channel 'AntiAliasScale']);end
 assert(fc>0&&fc<.45/h&&te>0);w=tan(pi*fc*h);dd=[1+2*z*w+w*w,2*(w*w-1),1-2*z*w+w*w];bb=w*w*[1,2,1]/dd(1);dd=dd/dd(1);ar=exp(-h/te);bb=conv(bb,[1-ar,0]);dd=conv(dd,[1,-ar]);
 p.d.acquisition.responses.(channel)=struct('numerator',bb,'denominator',dd,'steadyStatePerUnitInput',ones(3,1)/sum(dd),'cutoff_Hz',fc,'emaTime_s',te);
end
% Raw noise is referenced to the nominal engineering path, not actual AA tolerance.
w=tan(pi*defaults.antiAliasCutoff_Hz*h);dd=[1+2*defaults.antiAliasDamping*w+w*w,2*(w*w-1),1-2*defaults.antiAliasDamping*w+w*w];bb=w*w*[1,2,1]/dd(1);dd=dd/dd(1);ar=exp(-h/defaults.emaTime_s);bb=conv(bb,[1-ar,0]);dd=conv(dd,[1,-ar]);rr=filter(bb,dd,impulse);
p.d.acquisition.noiseReferenceImpulseEnergy=sum(rr.^2);p.d.acquisition.noiseInputScale=1/sqrt(p.d.acquisition.noiseReferenceImpulseEnergy);
p.d.acquisition.frameDelaySamples=p.d.sensorDelaySamples;
if c.enabled,p.d.acquisition.frameDelaySamples=round(c.identifiedPipelineDelay_s/p.sensor.sampleTime);assert(p.d.acquisition.frameDelaySamples>=0&&abs(p.d.acquisition.frameDelaySamples*p.sensor.sampleTime-c.identifiedPipelineDelay_s)<1e-12);end
p.d.acquisition.accelerationNoiseInputScale=sqrt((1+exp(-p.sensor.sampleTime/p.accel.filterTime))/(1-exp(-p.sensor.sampleTime/p.accel.filterTime)));
p.d.acquisition.positionNoiseInputScale=1;
if c.enabled,p.d.acquisition.accelerationNoiseInputScale=p.d.acquisition.noiseInputScale;p.d.acquisition.positionNoiseInputScale=p.d.acquisition.noiseInputScale;end
% Descriptive metadata follows the engineered response and actual noise
% parameters. No native plant or controller block reads sensorDesign.
enbw=p.d.acquisition.noiseReferenceImpulseEnergy/(2*h);
p.sensorDesign=struct('accelerometer','Two orthogonal ADXL1002 analog axes', ...
 'noiseDensity_ug_sqrtHz',25,'assumedENBW_Hz',enbw, ...
 'derivedRMS_m_s2',25e-6*9.80665*sqrt(enbw), ...
 'modeledRMS_m_s2',p.accel.noiseRms,'nominalModeledRMS_m_s2',p.accel.nominalNoiseRms, ...
 'ADCbits',18,'ADCvoltageRange_V',[0 5],'sensitivity_V_g',.04, ...
 'derivedQuantum_m_s2',5/2^18/.04*9.80665,'modeledQuantum_m_s2',p.accel.quantum, ...
 'modeledRange_g',p.accel.range_g,'rawSampleTime_s',h, ...
 'antiAliasCutoff_Hz',c.antiAliasCutoff_Hz,'antiAliasDamping',c.antiAliasDamping, ...
 'conditioningEMA_s',c.emaTime_s,'responseGroupDelay_s',p.d.acquisition.nominalLowFrequencyDelay_s, ...
 'commonAcquisitionDelay_s',p.sensor.delay,'identifiedMotionFrameDelay_s',c.identifiedPipelineDelay_s, ...
 'forceConditioning','Designed common two-pole AA and fresh EMA; actual and identified response/phase are explicit.', ...
 'interpretation','Engineered and identified acquisition target, not a guaranteed manufacturer transfer. Noise is conditioned RMS before conservative post-response ADC quantization.');
end

function n = ticks(t,h,name,allowZero)
if nargin < 4, allowZero = false; end
n = round(t/h);
assert(isfinite(t) && abs(t/h-n) < 1e-7 && (n >= 1 || (allowZero && n == 0)), ...
    '%s must be an integer multiple of its parent sample period.',name);
end
