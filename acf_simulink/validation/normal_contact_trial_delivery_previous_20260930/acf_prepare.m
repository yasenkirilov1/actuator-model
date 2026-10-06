function p = acf_prepare(p)
%ACF_PREPARE Validate edits and calculate equilibrium and controller coefficients.
% The coil observer uses force, relative position and issued commands.
% Software controller defaults migrate parameter sets saved before this version.
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
if ~isfield(p.coil,'strategy'),p.coil.strategy='observer';end
if ~isfield(p.coil,'forceKp'),p.coil.forceKp=.12;end
if ~isfield(p.coil,'forceKi'),p.coil.forceKi=200;end
if ~isfield(p.coil,'forceKd'),p.coil.forceKd=0;end
if ~isfield(p.coil,'derivativeFilter'),p.coil.derivativeFilter=.0003;end
if ~isfield(p.force,'derivativeFilter'),p.force.derivativeFilter=.001;end
if ~isfield(p.screw,'forceKd'),p.screw.forceKd=0;end
if ~isfield(p.ideal,'forceKd'),p.ideal.forceKd=0;end
assert(any(strcmp(p.coil.strategy,{'observer','pi','pid'})) && ...
    all(isfinite([p.coil.forceKp,p.coil.forceKi,p.coil.forceKd,p.coil.derivativeFilter])) && ...
    all([p.coil.forceKp,p.coil.forceKi,p.coil.forceKd]>=0) && p.coil.derivativeFilter>0 && ...
    p.force.derivativeFilter>0,'Invalid force-controller strategy or gains.');
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
if ~isfield(p.coil,'directCoulombCorrection'),p.coil.directCoulombCorrection=false;end
if ~isfield(p.robot,'initialPositionOffset_m'),p.robot.initialPositionOffset_m=0;end
assert(isfinite(p.robot.initialPositionOffset_m),'Initial robot position offset must be finite.');
if ~isfield(p,'robotPrediction')
    p.robotPrediction=struct('enabled',false,'delay',p.robot.delay, ...
        'responseTime',p.robot.responseTime,'maxSpeed',p.robot.maxSpeed, ...
        'maxAcceleration',p.robot.maxAcceleration,'residualTime',.06, ...
        'residualJerkPower',.0002,'boundedNominalPath',false);
end
predict=p.robotPrediction;
assert(isscalar(predict.enabled) && any(predict.enabled==[false true]) && ...
    all(isfinite([predict.delay,predict.responseTime,predict.maxSpeed, ...
    predict.maxAcceleration,predict.residualTime,predict.residualJerkPower])) && ...
    predict.delay>=0 && all([predict.responseTime,predict.maxSpeed, ...
    predict.maxAcceleration,predict.residualTime,predict.residualJerkPower]>0), ...
    'Invalid command-driven robot predictor settings.');
if ~isfield(p.coil,'control')
    p.coil.control=struct('forceScale_N',0.08,'integralScale_Ns',0.02, ...
        'velocityScale_m_s',0.1,'currentScale_A',1,'surfaceJerkPower',70, ...
        'robotJerkPower',0.2,'forceNoise_N',0.035,'strokeNoise_m',1e-6, ...
        'frictionPower',0.03^2);
end
assert(isfinite(p.sensor.strokeNoiseRms) && p.sensor.strokeNoiseRms>=0 && ...
    isfinite(p.sensor.strokeDelay) && p.sensor.strokeDelay>=0,'Invalid magnet measurement settings.');
if strcmp(p.actuator.kind,'voice_coil')
    assert(abs(p.force.sampleTime-p.sensor.sampleTime)<1e-12, ...
        'The voice-coil observer requires equal force and sensor sample periods.');
end

assert(any(strcmp(p.actuator.kind, {'leadscrew','voice_coil','ideal_position'})), ...
    'Unknown actuator kind.');
assert(any(strcmp(p.surface.kind, {'flat','bump','sharp_bump','ramp','repeated','robot_error'})), ...
    'Unknown surface scenario.');
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
assert(~(m.lowerRigid || m.upperRigid) || strcmp(p.actuator.kind,'voice_coil'), ...
    'The rigid-link comparison currently supports the voice-coil drive.');
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
p.d.robotDelaySamples = ticks(p.robot.delay,p.robot.sampleTime,'robot.delay',true);
p.robotPrediction.delaySamples=ticks(predict.delay,p.robot.sampleTime,'robotPrediction.delay',true);
p.d.strokeDelaySamples = ticks(p.sensor.strokeDelay,p.force.sampleTime,'sensor.strokeDelay',true);
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
        p.d.forceKd = p.screw.forceKd/p.d.ell;
        p.d.derivativeFilter = p.force.derivativeFilter;
        p.d.forceFeedforward = 0;
        p.d.commandMax = p.screw.maxLinearSpeedRequest/p.d.ell;
        p.d.commandMin = -p.d.commandMax;
        holdCurrent = p.d.screwHoldCurrent;
        availableCurrent = p.screw.peakCurrent;
        holdVoltage = abs(holdCurrent*p.screw.resistance);
        availableVoltage = p.screw.voltage;
    case 'voice_coil'
        % Preserve gains when an older PI diagram is still loaded in MATLAB.
        p.d.forceKp=0;p.d.forceKi=0;
        if isfield(p.coil,'forceKp'),p.d.forceKp=p.coil.forceKp;end
        if isfield(p.coil,'forceKi'),p.d.forceKi=p.coil.forceKi;end
        p.d.forceKd=p.coil.forceKd*double(strcmp(p.coil.strategy,'pid'));
        p.d.derivativeFilter=p.coil.derivativeFilter;
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
        p.d.forceKd = p.ideal.forceKd;
        p.d.derivativeFilter = p.force.derivativeFilter;
        p.d.forceFeedforward = p.d.u0;
        p.d.commandMin = m.travelMin-m.midTravel;
        p.d.commandMax = m.travelMax-m.midTravel;
        holdCurrent = p.d.holdForce; availableCurrent = p.ideal.maxAxialForce;
        holdVoltage = 0; availableVoltage = 1;
end
assert(abs(holdCurrent) <= availableCurrent && holdVoltage <= availableVoltage, ...
    'Actuator cannot hold the starting equilibrium: increase its limits or change preload.');
p.d.coilControl=coil_controller(p);
end

function n = ticks(t,h,name,allowZero)
if nargin < 4, allowZero = false; end
n = round(t/h);
assert(isfinite(t) && abs(t/h-n) < 1e-7 && (n >= 1 || (allowZero && n == 0)), ...
    '%s must be an integer multiple of its parent sample period.',name);
end

function c=coil_controller(p)
% Coefficients for native Discrete State-Space and Gain blocks, not a plant solver.
% State order: lower deflection/velocity, upper deflection/velocity, gap/velocity,
% winding-current deviation, surface acceleration, robot acceleration.
persistent previousKey previousControl
m=p.mechanics;drive=p.coil;w=drive.control;predict=p.robotPrediction;
distalModel=p.sensor.distalMass_model_kg;
weights=struct2array_local(w);
assert(all(isfinite(weights) & weights>0),'Coil controller weights must be positive and finite.');
key=[m.mA,m.mB,m.mS,m.k1,m.k2,m.c1,m.c2,m.lowerRigid,m.upperRigid,drive.forceConstant, ...
    drive.viscousFriction,drive.currentResponse,p.force.sampleTime, ...
    p.sensor.filterTime,p.d.sensorDelaySamples,p.d.strokeDelaySamples, ...
    distalModel,p.sensor.positionModel,predict.enabled,predict.residualTime, ...
    predict.residualJerkPower,drive.directCoulombCorrection,weights];
if isequal(key,previousKey),c=previousControl;return;end
ma=m.mA;mb=m.mB;ms=m.mS;mu=p.d.mu;kf=drive.forceConstant;b=drive.viscousFriction;
ts=p.force.sampleTime;tau=drive.currentResponse;
A=zeros(9);Bu=zeros(9,1);Bf=zeros(9,1);
A(1,2)=1;A(3,4)=1;A(5,6)=1;
A(2,[1 2 6 7 8])=[-m.k1/mb,-m.c1/mb,-b/mb,kf/mb,1];
A(4,[3 4 6 7 9])=[-m.k2/ma,-m.c2/ma,-b/ma,kf/ma,-1];
A(6,[1 2 3 4 6 7])=[-m.k1/mb,-m.c1/mb,-m.k2/ma,-m.c2/ma,-b/mu,kf/mu];
A(7,7)=-1/tau;Bu(7)=1/tau;
Bf([2 4 6])=[-1/mb;-1/ma;-1/mu];
Cf=zeros(1,9);Cf([1 2 8])=[m.k1,m.c1,ms];Cu=zeros(1,9);Cu(5)=1;
if p.sensor.positionModel==1,Cu(:)=0;Cu([1 5])=[1 -1];end
% Rigid attachments impose kinematic constraints, not a zero-stiffness gap.
% Fixed deflection states have stable zero dynamics and no physical motion.
if m.lowerRigid
    A(1:2,:)=0;A(1,1)=-1000;A(2,2)=-1000;Bf(2)=0;
    A(6,:)=0;A(6,[3 4 6 7 8])=[-m.k2/ma,-m.c2/ma,-b/ma,kf/ma,-1];
    Bf(6)=-1/ma;Cf(:)=0;Cf([6 7 8])=[-b,kf,mb+ms];
end
if m.upperRigid
    A(3:4,:)=0;A(3,3)=-1000;A(4,4)=-1000;Bf(4)=0;
    A(6,:)=0;A(6,[1 2 6 7 9])=[-m.k1/mb,-m.c1/mb,-b/mb,kf/mb,1];Bf(6)=-1/mb;
end
if m.lowerRigid && m.upperRigid
    A(6,:)=0;A(6,[8 9])=[-1 1];Bf(6)=0;
end
% Predict acceleration from the issued robot command. State 9 estimates the
% residual; add the nominal prediction back at the observer's native output.
Br=A(:,9);Br(9)=0;
if predict.enabled,A(9,9)=-1/predict.residualTime;else,Br(:)=0;end
E=expm([A,Bu,Bf,Br;zeros(3,12)]*ts);Ad=E(1:9,1:9);Bd=E(1:9,10:12);
Qc=w.frictionPower*(Bf*Bf');Qc(8,8)=Qc(8,8)+w.surfaceJerkPower;
if predict.enabled,robotPower=predict.residualJerkPower;else,robotPower=w.robotJerkPower;end
Qc(9,9)=Qc(9,9)+robotPower;
V=expm([A,Qc;zeros(9),-A']*ts);Qd=V(1:9,10:18)*V(1:9,1:9)';
% Sensor filter update matches the native sampled filter, including configured delays.
a=exp(-ts/max(p.sensor.filterTime,realmin))*double(p.sensor.filterTime>0);
nf=max(p.d.sensorDelaySamples-1,0);ng=p.d.strokeDelaySamples;N=10+nf+ng;
Aa=zeros(N);Aa(1:9,1:9)=Ad;Ba=zeros(N,3);Ba(1:9,:)=Bd;
Qa=zeros(N);Qa(1:9,1:9)=Qd;
CfMeasured=Cf;CfMeasured(8)=CfMeasured(8)-distalModel;
filterRow=zeros(1,N);filterRow(1:9)=(1-a)*CfMeasured;filterRow(10)=a;Aa(10,:)=filterRow;
if m.lowerRigid && drive.directCoulombCorrection
    assert(p.d.sensorDelaySamples>=1, ...
        'Direct Coulomb correction requires at least one configured force-delay sample.');
    Ba(10,2)=-(1-a);
end
C=zeros(2,N);
if p.d.sensorDelaySamples==0,C(1,:)=filterRow;
else,C(1,10+nf)=1;end
for j=1:nf,Aa(10+j,9+j)=1;end
firstGap=11+nf;
if ng==0,C(2,1:9)=Cu;
else
    Aa(firstGap,1:9)=Cu;
    for j=1:ng-1,Aa(firstGap+j,firstGap+j-1)=1;end
    C(2,firstGap+ng-1)=1;
end
scales=[1e-3,0.1,1e-3,0.1,1e-3,0.1,1,10,10,ones(1,N-9)];
if ng>0,scales(firstGap:end)=1e-3;end
S=diag(scales);As=S\(Aa*S);Bs=S\Ba;Cs=C*S;Qs=S\Qa/S';
Qs=(Qs+Qs')/2+eye(N)*1e-15;
R=diag([w.forceNoise_N^2,w.strokeNoise_m^2]);
P=dare_doubling(As',Cs',Qs,R);
L=(P*Cs')/(Cs*P*Cs'+R);
c.A=As*(eye(N)-L*Cs);c.B=[As*L,Bs];
c.C=(S*(eye(N)-L*Cs));c.C=c.C(1:9,:);
c.D=[S*L,zeros(N,3)];c.D=c.D(1:9,:);
% LQI synthesis in compliant-link coordinates; no extra toolbox is required.
Ab=zeros(5);Bb=zeros(5,1);Ab(1,2)=1;Ab(3,4)=1;
Ab(2,[1 2 4 5])=[-m.k1/mb,-(m.c1+b)/mb,-b/mb,kf/mb];
Ab(4,[2 3 4 5])=[-b/ma,-m.k2/ma,-(m.c2+b)/ma,kf/ma];
Ab(5,5)=-1/tau;Bb(5)=1/tau;Cb=[m.k1,m.c1,0,0,0];
if m.lowerRigid
    Ab(1:2,:)=0;Ab(1,1)=-1000;Ab(2,2)=-1000;
    Ab(4,:)=0;Ab(4,[3 4 5])=[-m.k2/ma,-(m.c2+b)/ma,kf/ma];
    Cb=[0,0,0,-b,kf];
end
if m.upperRigid
    Ab(3:4,:)=0;Ab(3,3)=-1000;Ab(4,4)=-1000;
    Ab(2,:)=0;Ab(2,[1 2 5])=[-m.k1/mb,-(m.c1+b)/mb,kf/mb];
end
if m.lowerRigid && m.upperRigid,Cb=[0,0,0,0,kf];end
Al=[Ab,zeros(5,1);Cb,0];Bl=[Bb;0];
Q=zeros(6);Q(1:5,1:5)=(Cb'*Cb)/w.forceScale_N^2;
if ~m.lowerRigid,Q(2,2)=Q(2,2)+1/w.velocityScale_m_s^2;end
if ~m.upperRigid,Q(4,4)=Q(4,4)+1/w.velocityScale_m_s^2;end
Q(6,6)=1/w.integralScale_Ns^2;
Rc=1/w.currentScale_A^2;
T=diag([1e-3,.1,1e-3,.1,1,.02]);Ax=T\(Al*T);Bx=T\Bl;Qx=T'*Q*T;
if m.lowerRigid || m.upperRigid
    % Direct force/current coupling needs discrete synthesis at the actual rate.
    % Continuous high gains can be unstable once sampled, despite a stable CARE.
    E=expm([Ab,Bb;zeros(1,6)]*ts);
    Adl=[E(1:5,1:5),zeros(5,1);ts*Cb,1];Bdl=[E(1:5,6);0];
    Ax=T\(Adl*T);Bx=T\Bdl;Px=dare_doubling(Ax,Bx,Qx,Rc);
    Kx=(Rc+Bx'*Px*Bx)\(Bx'*Px*Ax);K=Kx/T;
    assert(max(abs(eig(Adl-Bdl*K)))<1,'Rigid-link sampled feedback is unstable.');
else
    [U,D]=eig([Ax,-(Bx/Rc)*Bx';-Qx,-Ax']);stable=real(diag(D))<0;
    assert(sum(stable)==6,'Coil state feedback could not be synthesized.');
    Us=U(:,stable);Px=real(Us(7:12,:)/Us(1:6,:));Px=(Px+Px')/2;
    K=((Bx'*Px)/Rc)/T;
end
c.feedback=zeros(1,9);c.feedback([1 2 3 4 7])=-K(1:5);c.integralGain=K(6);
ayReference=[0;0;0;0;-(ms+mb)/kf];arReference=zeros(5,1);
if ~m.lowerRigid,ayReference(1)=-ms/m.k1;end
if ~m.upperRigid,ayReference(3)=-(ms+mb)/m.k2;arReference(3)=-ma/m.k2;end
c.feedback(8)=-(ms+mb)/kf+K(1:5)*ayReference;
c.feedback(9)=K(1:5)*arReference;c.force=Cf;
assert(all(isfinite([c.A(:);c.B(:);c.C(:);c.D(:);K(:)])) && ...
    max(abs(eig(c.A)))<1 && c.integralGain>0,'Invalid coil controller coefficients.');
previousKey=key;previousControl=c;
end

function values=struct2array_local(s)
names={'forceScale_N','integralScale_Ns','velocityScale_m_s','currentScale_A', ...
    'surfaceJerkPower','robotJerkPower','forceNoise_N','strokeNoise_m','frictionPower'};
values=zeros(1,numel(names));
for j=1:numel(names),values(j)=s.(names{j});end
end

function P=dare_doubling(A,B,Q,R)
% Structure-preserving doubling solves the discrete Riccati equation.
% Each iteration doubles the horizon; lightly damped modes need ~20 iterations
% instead of tens of thousands of single-step covariance updates.
% See Lin and Xu, SIAM J. Matrix Anal. Appl., doi:10.1137/040617650.
% The residual and the caller's closed-loop pole checks reject invalid results.
originalA=A;G=(B/R)*B';P=Q;I=eye(size(A));
for iteration=1:100
    W=I+G*P;
    next=P+A'*(P*(W\A));next=(next+next')/2;
    Gn=G+A*((W\G)*A');Gn=(Gn+Gn')/2;
    An=A*(W\A);
    change=norm(next-P,'fro');P=next;G=Gn;A=An;
    if change<=1e-12*max(norm(P,'fro'),realmin),break;end
end
assert(iteration<100 && all(isfinite(P),'all'),'Discrete Riccati doubling did not converge.');
residual=P-(originalA'*P*originalA-originalA'*P*B*((R+B'*P*B)\(B'*P*originalA))+Q);
relativeResidual=norm(residual,'fro')/max(norm(P,'fro'),realmin);
assert(relativeResidual<1e-9 && min(eig(P))>=-1e-9*max(norm(P,2),1),'Invalid discrete Riccati solution.');

end
