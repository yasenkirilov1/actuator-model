function contract=install_upper_force_observer_lower_damped_v2(mdl,p)
% Existing measured cell/inertia balance supplies the causal upper observer.
% Private comparison only. No additional sensor or physical plant changes.
contract=struct('enabled',p.upperObserver.enabled,'nativeBlocksOnly',true);
assert(p.upperObserver.enabled&&~p.mechanics.upperRigid&&p.mechanics.c1>0);
sys=[mdl '/Contact force measurement/Head motion estimator'];
observer=[sys '/Head gap and velocity observer'];
set_param(observer,'A','p.d.upperMotionObserver.A','B','p.d.upperMotionObserver.B', ...
 'C','eye(4)','D','zeros(4,4)','InitialCondition','zeros(4,1)');
set_param([sys '/Causal motion measurements'],'Inputs','4');
% Preserve the old two-dimensional gap output while exposing corrected A.
ph=get_param(observer,'PortHandles');line=get_param(ph.Outport(1),'Line');dest=get_param(line,'DstPortHandle');delete_line(line);
add_block('simulink/Signal Routing/Selector',[sys '/Reported gap states'],'InputPortWidth','4','Indices','[3,4]','Position',[1335 305 1510 360]);
add_block('simulink/Math Operations/Gain',[sys '/Prior state correction matrix'],'Gain','p.d.upperMotionObserver.C','Multiplication','Matrix(K*u)','Position',[1050 1420 1320 1500]);
add_block('simulink/Math Operations/Gain',[sys '/Position correction gain'],'Gain','p.d.upperMotionObserver.gain','Multiplication','Matrix(K*u)','Position',[1050 1550 1320 1630]);
add_block('simulink/Math Operations/Sum',[sys '/Corrected upper motion states'],'Inputs','++','Position',[1410 1420 1630 1600]);
add_line(sys,'Head gap and velocity observer/1','Prior state correction matrix/1');
add_line(sys,'Prior state correction matrix/1','Corrected upper motion states/1');
add_line(sys,'Fresh 10 kHz position frame/1','Position correction gain/1','autorouting','on');
add_line(sys,'Position correction gain/1','Corrected upper motion states/2');
add_line(sys,'Corrected upper motion states/1','Reported gap states/1');
newph=get_param([sys '/Reported gap states'],'PortHandles');
for j=1:numel(dest),add_line(sys,newph.Outport(1),dest(j),'autorouting','on');end
% Input order: measured gap, net delta Fa, predicted aR, measured aS.
delete_line(sys,'Fresh 10 kHz robot acceleration frame/1','Causal motion measurements/2');
delete_line(sys,'Measured S acceleration Z/1','Causal motion measurements/3');
add_line(sys,'Fresh 10 kHz robot acceleration frame/1','Causal motion measurements/3','autorouting','on');
add_line(sys,'Measured S acceleration Z/1','Causal motion measurements/4','autorouting','on');
% measured_force_N is gravity-tared AXIAL B-S cell force, not normal error.
% It and measured S acceleration already have the same acquisition frame.
add_block('simulink/Signal Routing/From',[sys '/Measured tared axial cell force'],'GotoTag','measured_force_N','Position',[20 970 260 1000]);
add_block('simulink/Math Operations/Bias',[sys '/Cell force deviation from equilibrium'],'Bias','-p.force.reference','Position',[300 945 590 1025]);
add_block('simulink/Discrete/Discrete Transfer Fcn',[sys '/Cell force correction'], ...
 'Numerator','p.d.upperMotionObserver.forceBalanceNumerator','Denominator','p.d.upperMotionObserver.forceBalanceDenominator', ...
 'InitialStates','zeros(3,1)','SampleTime','p.force.sampleTime','Position',[660 1080 1000 1160]);
add_block('simulink/Math Operations/Gain',[sys '/Force correction units'], ...
 'Gain','1','Position',[1080 1080 1450 1160]);
add_block('simulink/Math Operations/Gain',[sys '/Measured holder acceleration inertia'], ...
 'Gain','p.uncertainty.massB_model_kg','Position',[1080 1240 1450 1320]);
add_block('simulink/Math Operations/Sum',[sys '/Measured net actuator force deviation'],'Inputs','+-+', ...
 'Position',[1530 940 1830 1280]);
add_line(sys,'Measured tared axial cell force/1','Cell force deviation from equilibrium/1');
add_line(sys,'Cell force deviation from equilibrium/1','Measured net actuator force deviation/1','autorouting','on');
add_line(sys,'Cell force deviation from equilibrium/1','Cell force correction/1','autorouting','on');
add_line(sys,'Cell force correction/1','Force correction units/1');
add_line(sys,'Measured S acceleration Z/1','Measured holder acceleration inertia/1','autorouting','on');
add_line(sys,'Measured holder acceleration inertia/1','Measured net actuator force deviation/2');
add_line(sys,'Force correction units/1','Measured net actuator force deviation/3');
add_line(sys,'Measured net actuator force deviation/1','Causal motion measurements/2','autorouting','on');
% Reconstruct A velocity/acceleration in that same measurement frame.
add_block('simulink/Signal Routing/Selector',[sys '/Estimated upper velocity deviation'],'InputPortWidth','4','Indices','2','Position',[1390 800 1640 855]);
add_block('simulink/Math Operations/Sum',[sys '/Estimated aligned A velocity'],'Inputs','++','Position',[1730 735 1960 845]);
add_block('simulink/Signal Routing/Goto',[sys '/Aligned A velocity'],'GotoTag','upper_estimated_A_velocity_aligned','TagVisibility','global','Position',[2050 735 2320 765]);
add_block('simulink/Math Operations/Gain',[sys '/Estimated A acceleration from upper states'],'Gain','p.d.upperMotionObserver.accelerationFromStateGain','Multiplication','Matrix(K*u)','Position',[2400 790 2690 875]);
add_block('simulink/Math Operations/Gain',[sys '/Estimated A acceleration from measured force'],'Gain','p.d.upperMotionObserver.accelerationFromForceGain','Position',[2400 955 2690 1040]);
add_block('simulink/Math Operations/Sum',[sys '/Estimated aligned A acceleration'],'Inputs','++','Position',[2770 790 3010 970]);
add_block('simulink/Signal Routing/Goto',[sys '/Aligned A acceleration'],'GotoTag','upper_estimated_A_acceleration_aligned','TagVisibility','global','Position',[3090 810 3390 840]);
add_line(sys,'Corrected upper motion states/1','Estimated upper velocity deviation/1','autorouting','on');
add_line(sys,'Fresh 10 kHz robot velocity frame/1','Estimated aligned A velocity/1','autorouting','on');
add_line(sys,'Estimated upper velocity deviation/1','Estimated aligned A velocity/2');
add_line(sys,'Estimated aligned A velocity/1','Aligned A velocity/1');
add_line(sys,'Corrected upper motion states/1','Estimated A acceleration from upper states/1','autorouting','on');
add_line(sys,'Measured net actuator force deviation/1','Estimated A acceleration from measured force/1','autorouting','on');
add_line(sys,'Estimated A acceleration from upper states/1','Estimated aligned A acceleration/1');
add_line(sys,'Estimated A acceleration from measured force/1','Estimated aligned A acceleration/2');
add_line(sys,'Estimated aligned A acceleration/1','Aligned A acceleration/1');
ff=[mdl '/Force controller/Coil PI and PID/Measured motion feedforward'];
set_param([mdl '/Contact force measurement/Normal force projection/Issued robot velocity'],'GotoTag','upper_estimated_A_velocity_aligned');
set_param([ff '/Robot acceleration'],'GotoTag','upper_estimated_A_acceleration_aligned');
contract.stateNames=p.d.upperMotionObserver.stateNames;contract.inputNames=p.d.upperMotionObserver.inputNames;
contract.observabilityRank=p.d.upperMotionObserver.observabilityRank;
contract.maximumPoleMagnitude=p.d.upperMotionObserver.maximumPoleMagnitude;contract.gain=p.d.upperMotionObserver.gain;
contract.measuredForceSource='Gravity-tared axial B-S cell, existing measured_force_N tag minus Fref';
contract.forceBalance='deltaFa = deltaL - identified_mB*aS + identified_mB * s^2/(identified_k1+identified_c1*s) * LP2 deltaL; prewarped Tustin at10kHz';
contract.derivativeCutoff_Hz=p.upperObserver.forceDerivativeCutoff_Hz;contract.derivativeDamping=p.inertial.lowerLoadInverse.filterDamping;
contract.discreteDerivativeDirectCoefficient=p.d.upperMotionObserver.forceBalanceNumerator(1);
contract.causality='Only existing matched axial cell, S acceleration, S-A position and already-issued robot prediction. No actual A/current/actuator-force/surface truth. No extra force acquisition/pipeline, equilibrium holding-force subtraction or guide-friction subtraction.';
contract.predictorLimitation='Force-balance differentiation reuses the prepared lower-inverse transfer coefficients multiplied by identifiedKf on MEASUREDdeltaL, not the existing desired-load filter output; it amplifies acquired cell noise and is approximate above the regularization band. It estimates net physical actuator force including actual drive clipping and guide friction.';
end
