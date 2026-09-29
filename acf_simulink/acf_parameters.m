function p = acf_parameters(kind, scenario)
%ACF_PARAMETERS Editable engineering assumptions, in SI units.
% All numerical defaults are EXAMPLES, not identified hardware values.
% kind: 'leadscrew', 'voice_coil', 'ideal_position'
% scenario: 'flat', 'bump', 'sharp_bump', 'ramp', 'repeated', 'robot_error'
if nargin < 1, kind = 'leadscrew'; end
if nargin < 2, scenario = 'bump'; end

p.actuator.kind = kind;
p.sim.step = 0.0002;                  % s; numerical step, NOT controller period
p.sim.stopTime = 6;                   % s
p.sim.evaluateFrom = 0;               % no hidden settling interval
p.force.reference = 20;               % N
p.force.tolerance = 0.5;              % N
p.force.sampleTime = 0.001;           % s; try 0.01 to test 100 Hz
p.force.antiWindupRate = 30;          % 1/s; back-calculation

p.mechanics.mA = 0.4;                 % kg
p.mechanics.mB = 0.6;                 % kg
p.mechanics.mS = 0.15;                % kg, includes tool mass
p.mechanics.k1 = 10000;               % N/m, B--S spring
p.mechanics.k2 = 20000;               % N/m, R--A spring
p.mechanics.c1 = 30;                  % N s/m
p.mechanics.c2 = 40;                  % N s/m
p.mechanics.gravity = 9.81;           % m/s^2
p.mechanics.angle = 0;                % rad; UP axis relative to vertical UP
p.mechanics.travelMin = 0;           % m
p.mechanics.travelMax = 0.025;       % m; usable travel and hard stops
p.mechanics.midTravel = 0.0125;      % m

p.robot.initialExcessForce = 10;     % N, if flange were locked at mid-stroke
p.robot.sampleTime = 0.01;           % s
p.robot.delay = 0.08;                % s; requested-velocity delay
p.robot.recenterGain = 0.8;          % 1/s
p.robot.deadband = 0.0005;           % m around mid-stroke
p.robot.responseTime = 0.06;         % s; velocity response
p.robot.maxSpeed = 0.02;             % m/s along the normal axis
p.robot.maxAcceleration = 0.5;      % m/s^2
p.robot.nominalVelocity = 0;         % m/s; independent of actual surface
p.robot.errorRiseTime = 0.2;         % s, for robot_error scenario
p.robot.errorForce = 20;             % N equivalent extra penetration

p.sensor.sampleTime = 0.001;         % s
p.sensor.filterTime = 0.002;         % s, sampled first-order low-pass
p.sensor.delay = 0;                  % s; integer sensor samples required
p.sensor.bias = 0;                   % N
p.sensor.noiseRms = 0;               % N, reproducible uniform sample noise
p.sensor.seed = 1729;

p.surface.kind = scenario;
p.surface.startTime = 2;             % s
p.surface.height = 0.001;            % m
p.surface.width = 0.06;              % m along the scanning direction
p.surface.scanSpeed = 0.1;           % m/s tangential scan speed
p.surface.repeatPeriod = 1.2;        % s

% Screw: rigid kinematics u = lead/(2*pi)*theta; NO axial contact spring.
% Motor and translational inertia are coupled, not a first-order force lag.
p.screw.lead = 0.005;                % m/rev
p.screw.rotorInertia = 2e-5;         % kg m^2; motor, coupling, screw reflected here
p.screw.torqueConstant = 0.05;       % N m/A
p.screw.backEmfConstant = 0.05;      % V s/rad
p.screw.resistance = 1.0;            % ohm
p.screw.inductance = 0.001;          % H
p.screw.peakCurrent = 3;             % A, symmetric four-quadrant drive
p.screw.voltage = 24;                % V
p.screw.currentResponse = 0.001;    % s, if sufficient voltage is available
p.screw.coulombTorque = 0.004;       % N m; smooth sliding friction, NOT self-locking
p.screw.viscousTorque = 2e-5;        % N m s/rad
p.screw.frictionSpeed = 0.2;         % rad/s; smoothing width
p.screw.speedBandwidth = 150;       % rad/s, inner speed-loop design value
p.screw.speedDamping = 1;
p.screw.speedAntiWindup = 100;       % 1/s
p.screw.maxLinearSpeedRequest = 0.1;% m/s; omega request = speed/ell
p.screw.forceKp = 0.02;              % (m/s)/N, outer force-to-speed PI
p.screw.forceKi = 0.008;             % (m/s)/(N s)

% Coil: current makes force. Position still follows Newton's law.
p.coil.forceConstant = 20;           % N/A; also V s/m in an ideal reciprocal coil
p.coil.resistance = 4;               % ohm
p.coil.inductance = 0.003;           % H
p.coil.peakCurrent = 3;              % A
p.coil.voltage = 24;                 % V
p.coil.currentResponse = 0.001;      % s
p.coil.coulombForce = 0.2;           % N
p.coil.viscousFriction = 2;          % N s/m
p.coil.frictionSpeed = 0.0002;       % m/s
p.coil.forceKp = 0.04;               % A/N
p.coil.forceKi = 1.5;                % A/(N s)

% Ideal position source is a sizing reference with finite speed/acceleration.
% Default force is unlimited; set maxAxialForce to sweep a force budget.
p.ideal.positionGain = 100;          % 1/s, position error -> velocity request
p.ideal.velocityTime = 0.005;        % s
p.ideal.maxSpeed = 0.1;              % m/s
p.ideal.maxAcceleration = 20;        % m/s^2
p.ideal.maxAxialForce = inf;         % N; optional symmetric force budget
p.ideal.forceKp = 0.5/6666.6666667;  % m/N; illustrative tuning for default stiffness
p.ideal.forceKi = 120/6666.6666667;  % m/(N s)

if strcmp(scenario,'sharp_bump'), p.surface.width = 0.01; end
if strcmp(scenario,'ramp'), p.surface.height = 0.003; p.surface.width = 0.2; end
if strcmp(scenario,'repeated'), p.sim.stopTime = 8; end
p = acf_prepare(p);
end
