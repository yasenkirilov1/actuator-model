function m = acf_metrics(r)
%ACF_METRICS Requirements use true force. Impact peaks are not regularized.
p=r.parameters; use=r.time>=p.sim.evaluateFrom;
assert(any(use),'No samples in the evaluation interval.');
t=r.time(use); d=r.data(use,:); err=d(:,31)-d(:,1);
m.peakAbsForceError_N=max(abs(err));
m.rmsForceError_N=sqrt(mean(err.^2));
m.fractionWithinTolerance=mean(abs(err)<=p.force.tolerance);
bad=abs(err)>p.force.tolerance;
m.timeOutsideTolerance_s=sum(double(bad(1:end-1)).*diff(t));
m.minTravel_mm=1000*min(d(:,2)); m.maxTravel_mm=1000*max(d(:,2));
m.peakGapSpeed_m_s=max(abs(d(:,3)));
m.peakGapAcceleration_m_s2=max(abs(d(:,4)));
m.peakBodyAcceleration_m_s2=max(max(abs(d(:,11:12))));
m.peakAxialForce_N=max(abs(d(:,13)));
m.rmsAxialForce_N=sqrt(mean(d(:,13).^2));
m.peakCurrent_A=max(abs(d(:,14))); m.rmsCurrent_A=sqrt(mean(d(:,14).^2));
m.peakVoltage_V=max(abs(d(:,15)));
m.peakMotorTorque_Nm=max(abs(d(:,16)));
m.rmsMotorTorque_Nm=sqrt(mean(d(:,16).^2));
m.peakMotorSpeed_rad_s=max(abs(d(:,17)));
m.lostContact=any(d(:,18)<0.5);
m.impactCount=sum(d(:,20)>1e-10);
m.totalImpactImpulse_Ns=sum(d(:,20));
m.strokeStopHit=any(d(:,21)>0.5);
m.completed=r.time(end)>=p.sim.stopTime-p.sim.step/2;
m.effortSaturationFraction=mean(d(:,22)>0.5);
m.voltageSaturationFraction=mean(d(:,23)>0.5);
m.peakRobotSpeed_m_s=max(abs(d(:,27)));
if numel(t)>1, m.peakRobotAcceleration_m_s2=max(abs(diff(d(:,27))./diff(t)));
else, m.peakRobotAcceleration_m_s2=0; end
if strcmp(p.actuator.kind,'leadscrew')
    currentLimit=p.screw.peakCurrent;
elseif strcmp(p.actuator.kind,'voice_coil')
    currentLimit=p.coil.peakCurrent;
else
    currentLimit=inf;
end
% A voltage-limited drive can be forced over its current request by back-EMF.
m.overCurrent=m.peakCurrent_A>currentLimit*(1+1e-4);
m.passed=m.completed && m.peakAbsForceError_N<=p.force.tolerance && ~m.lostContact && ...
    m.impactCount==0 && ~m.strokeStopHit && ~m.overCurrent;
if isfield(p,'springs')
    m.springSizing=spring_sizing(r);
    m.passed=m.passed && m.springSizing.passed;
end
if m.impactCount>0
    m.peakForceIncludingImpacts_N=inf; % ideal rigid impact has no finite peak
else
    m.peakForceIncludingImpacts_N=max(d(:,1));
end
lastBad=find(abs(err)>p.force.tolerance | d(:,18)<0.5 | d(:,20)>1e-10,1,'last');
if isempty(lastBad), m.finalRecoveryTime_s=0;
elseif lastBad==numel(t), m.finalRecoveryTime_s=NaN;
else, m.finalRecoveryTime_s=t(lastBad+1); end % absolute time, not elapsed settling time
eventEnd=p.surface.startTime+p.d.duration;
if strcmp(p.surface.kind,'flat'), eventEnd=0;
elseif strcmp(p.surface.kind,'robot_error'), eventEnd=p.surface.startTime+p.robot.errorRiseTime;
elseif strcmp(p.surface.kind,'repeated')
    lastStart=p.surface.startTime+max(0,floor((t(end)-p.surface.startTime)/p.surface.repeatPeriod))*p.surface.repeatPeriod;
    eventEnd=lastStart+p.d.duration;
end
if eventEnd>t(end) || isnan(m.finalRecoveryTime_s), m.recoveryAfterDisturbance_s=NaN;
else, m.recoveryAfterDisturbance_s=max(0,m.finalRecoveryTime_s-eventEnd); end
reasons={};
if m.peakAbsForceError_N>p.force.tolerance, reasons{end+1}='force error'; end
if m.lostContact, reasons{end+1}='loss of contact'; end
if m.impactCount>0, reasons{end+1}='rigid re-impact'; end
if m.strokeStopHit, reasons{end+1}='stroke stop'; end
if m.overCurrent, reasons{end+1}='overcurrent'; end
if isfield(m,'springSizing') && ~m.springSizing.passed
    reasons{end+1}='spring length or working range';
end
if ~m.completed && ~m.lostContact && ~m.strokeStopHit, reasons{end+1}='incomplete run'; end
if isempty(reasons), m.reason='pass'; else, m.reason=strjoin(reasons,', '); end
end

function result=spring_sizing(r)
% Dimension/working-range screen of guided opposed pairs; no plant integration.
% Check the entire recording, including before the force evaluation interval.
p=r.parameters;pack=p.springs;d=r.data;links={'lower','upper'};
delta={d(:,7)-d(:,6),d(:,5)-d(:,26)};
velocity={d(:,10)-d(:,9),d(:,8)-d(:,27)};
offset=[p.d.q10,p.d.q20];stiffness=[p.mechanics.k1,p.mechanics.k2];
damping=[p.mechanics.c1,p.mechanics.c2];
result=struct('passed',true,'combinedFreeLengthMax_mm',0, ...
    'combinedInstalledLengthMax_mm',0,'limit_mm',1000*pack.maxCombinedLength_m);
for j=1:2
    one=pack.(links{j});ks=one.rate_N_m;
    rigid=[isfield(p.mechanics,'lowerRigid') && p.mechanics.lowerRigid, ...
        isfield(p.mechanics,'upperRigid') && p.mechanics.upperRigid];
    if rigid(j)
        result.(links{j})=struct('part','Rigid attachment','passed',true, ...
            'minCompression_mm',0,'maxCompression_mm',0, ...
            'minIndividualLength_mm',0,'maxIndividualLength_mm',0, ...
            'cyclicExcursion_mm',0,'peakIndividualForce_N',0, ...
            'peakDamperSpeed_m_s',0,'peakDamperForce_N',0,'maxDamperPower_W',0);
        continue;
    end
    bias=offset(j)/stiffness(j);common=pack.commonPreload_N/ks;
    compression=[common+bias+delta{j},common-bias-delta{j}];
    smallest=min(compression,[],'all');largest=max(compression,[],'all');
    shortest=one.freeLength_m-one.freeLengthTolerance_m-largest;
    longest=one.freeLength_m+one.freeLengthTolerance_m-smallest;
    excursion=max(max(compression)-min(compression));
    force=ks*largest;
    pass=smallest>=pack.unseatingClearance_m && ...
        shortest>=one.minWorkingLength_m+pack.workingClearance_m && ...
        excursion<=one.maxCyclicStroke_m && force<=one.forceEnvelope_N;
    result.(links{j})=struct('part',one.part,'passed',pass, ...
        'minCompression_mm',1000*smallest,'maxCompression_mm',1000*largest, ...
        'minIndividualLength_mm',1000*shortest,'maxIndividualLength_mm',1000*longest, ...
        'cyclicExcursion_mm',1000*excursion,'peakIndividualForce_N',force, ...
        'peakDamperSpeed_m_s',max(abs(velocity{j})), ...
        'peakDamperForce_N',damping(j)*max(abs(velocity{j})), ...
        'maxDamperPower_W',damping(j)*max(velocity{j}.^2));
    result.passed=result.passed && pass;
    result.combinedFreeLengthMax_mm=result.combinedFreeLengthMax_mm+ ...
        2000*(one.freeLength_m+one.freeLengthTolerance_m);
    % Opposed springs gain/lose the same length; the sum stays constant.
    result.combinedInstalledLengthMax_mm=result.combinedInstalledLengthMax_mm+ ...
        2000*(one.freeLength_m+one.freeLengthTolerance_m-common);
end
result.passed=result.passed && result.combinedFreeLengthMax_mm<=result.limit_mm+1e-9 && ...
    result.combinedInstalledLengthMax_mm<=result.limit_mm+1e-9;
end
