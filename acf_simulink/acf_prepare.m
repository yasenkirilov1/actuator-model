function p = acf_prepare(p)
%ACF_PREPARE Validate edits and recalculate derived quantities.
assert(any(strcmp(p.actuator.kind, {'leadscrew','voice_coil','ideal_position'})), ...
    'Unknown actuator kind.');
assert(any(strcmp(p.surface.kind, {'flat','bump','sharp_bump','ramp','repeated','robot_error'})), ...
    'Unknown surface scenario.');
m = p.mechanics;
assert(all([m.mA,m.mB,m.mS,m.k1,m.k2,p.sim.step,p.sim.stopTime, ...
    p.robot.responseTime,p.robot.maxSpeed,p.robot.maxAcceleration, ...
    p.surface.width,p.surface.scanSpeed,p.screw.lead,p.screw.rotorInertia, ...
    p.screw.resistance,p.screw.inductance,p.screw.torqueConstant, ...
    p.screw.currentResponse,p.screw.peakCurrent,p.screw.voltage, ...
    p.coil.forceConstant,p.coil.resistance,p.coil.inductance, ...
    p.coil.currentResponse,p.coil.peakCurrent,p.coil.voltage, ...
    p.ideal.velocityTime,p.ideal.maxSpeed,p.ideal.maxAcceleration] > 0), ...
    'Masses, stiffnesses, time constants and actuator limits must be positive.');
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
p.d.keq = m.k1*m.k2/(m.k1+m.k2);
p.d.ell = p.screw.lead/(2*pi);
p.d.q10 = p.force.reference-m.mS*p.d.g;
p.d.q20 = p.force.reference-(p.d.M+m.mS)*p.d.g;
p.d.holdForce = p.force.reference-(m.mB+m.mS)*p.d.g;
p.d.r0 = -p.robot.initialExcessForce/p.d.keq;
p.d.u0 = p.d.r0;
p.d.z0 = p.d.alpha*p.d.u0;
assert(m.midTravel+p.d.u0 > m.travelMin && ...
    m.midTravel+p.d.u0 < m.travelMax, 'Initial compensation exceeds travel.');
p.d.forceTicks = ticks(p.force.sampleTime,p.sim.step,'force.sampleTime');
p.d.sensorTicks = ticks(p.sensor.sampleTime,p.sim.step,'sensor.sampleTime');
p.d.robotTicks = ticks(p.robot.sampleTime,p.sim.step,'robot.sampleTime');
p.d.sensorDelaySamples = ticks(p.sensor.delay,p.sensor.sampleTime,'sensor.delay',true);
p.d.robotDelaySamples = ticks(p.robot.delay,p.robot.sampleTime,'robot.delay',true);
ticks(p.sim.stopTime,p.sim.step,'sim.stopTime');
assert(p.sim.step <= min([p.screw.currentResponse,p.coil.currentResponse, ...
    p.ideal.velocityTime])/2, 'Reduce sim.step to resolve drive dynamics.');
assert(p.surface.startTime >= 0 && p.force.reference > 0 && p.force.tolerance > 0);
p.d.duration = p.surface.width/p.surface.scanSpeed;
if strcmp(p.surface.kind,'repeated')
    assert(p.surface.repeatPeriod >= p.d.duration, 'Repeated bumps overlap.');
end
p.d.screwHoldCurrent = p.d.ell*p.d.holdForce/p.screw.torqueConstant;
p.d.coilHoldCurrent = p.d.holdForce/p.coil.forceConstant;
J = p.screw.rotorInertia+p.d.mu*p.d.ell^2;
p.d.speedKp = 2*p.screw.speedDamping*p.screw.speedBandwidth*J/p.screw.torqueConstant;
p.d.speedKi = p.screw.speedBandwidth^2*J/p.screw.torqueConstant;
switch p.actuator.kind
    case 'leadscrew'
        p.d.forceKp = p.screw.forceKp/p.d.ell;
        p.d.forceKi = p.screw.forceKi/p.d.ell;
        p.d.forceFeedforward = 0;
        p.d.commandMax = p.screw.maxLinearSpeedRequest/p.d.ell;
        p.d.commandMin = -p.d.commandMax;
        holdCurrent = p.d.screwHoldCurrent;
        availableCurrent = p.screw.peakCurrent;
        holdVoltage = abs(holdCurrent*p.screw.resistance);
        availableVoltage = p.screw.voltage;
    case 'voice_coil'
        p.d.forceKp = p.coil.forceKp;
        p.d.forceKi = p.coil.forceKi;
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
        p.d.forceFeedforward = p.d.u0;
        p.d.commandMin = m.travelMin-m.midTravel;
        p.d.commandMax = m.travelMax-m.midTravel;
        holdCurrent = p.d.holdForce; availableCurrent = p.ideal.maxAxialForce;
        holdVoltage = 0; availableVoltage = 1;
end
assert(abs(holdCurrent) <= availableCurrent && holdVoltage <= availableVoltage, ...
    'Actuator cannot hold the starting equilibrium: increase its limits or change preload.');
end

function n = ticks(t,h,name,allowZero)
if nargin < 4, allowZero = false; end
n = round(t/h);
assert(isfinite(t) && abs(t/h-n) < 1e-7 && (n >= 1 || (allowZero && n == 0)), ...
    '%s must be an integer multiple of its parent sample period.',name);
end
