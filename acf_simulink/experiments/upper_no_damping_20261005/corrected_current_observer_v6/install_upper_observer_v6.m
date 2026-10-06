function contract=install_upper_observer_v6(mdl,p)
% Add a native, causal upper-motion estimator only to private compliant banks.
contract=struct('enabled',p.upperObserver.enabled,'nativeBlocksOnly',true);
if ~p.upperObserver.enabled,return;end
sys=[mdl '/Contact force measurement/Head motion estimator'];
observer=[sys '/Head gap and velocity observer'];
set_param(observer,'A','p.d.upperMotionObserver.A','B','p.d.upperMotionObserver.B', ...
 'C','p.d.upperMotionObserver.C','D','p.d.upperMotionObserver.D','InitialCondition','zeros(4,1)');
set_param([sys '/Causal motion measurements'],'Inputs','4');
% Keep the old two-dimensional gap/state interface intact.
ph=get_param(observer,'PortHandles');line=get_param(ph.Outport(1),'Line');dest=get_param(line,'DstPortHandle');delete_line(line);
add_block('simulink/Signal Routing/Selector',[sys '/Reported gap states'],'InputPortWidth','4','Indices','[3,4]','Position',[1335 305 1510 360]);
add_line(sys,'Head gap and velocity observer/1','Reported gap states/1','autorouting','on');
newph=get_param([sys '/Reported gap states'],'PortHandles');
for j=1:numel(dest),add_line(sys,newph.Outport(1),dest(j),'autorouting','on');end
% Input order is [gap measurement, delta Fa, predicted aR, measured aS].
delete_line(sys,'Fresh 10 kHz robot acceleration frame/1','Causal motion measurements/2');
delete_line(sys,'Measured S acceleration Z/1','Causal motion measurements/3');
add_line(sys,'Fresh 10 kHz robot acceleration frame/1','Causal motion measurements/3','autorouting','on');
add_line(sys,'Measured S acceleration Z/1','Causal motion measurements/4','autorouting','on');
% Issued actions alone predict current. No actual current or plant force input.
parent=[mdl '/Contact force measurement'];
% The selected root command also drives the physical actuator. Explicit ports
% avoid the root-local actuator_command Goto scope that grounded old graphs.
add_block('simulink/Ports & Subsystems/In1',[parent '/Issued selected actuator request'],'Port','3','Position',[20 480 60 510]);
add_block('simulink/Ports & Subsystems/In1',[sys '/Issued post-limit current request'],'Port','2','Position',[20 1000 60 1030]);
add_line(mdl,'Force controller/1','Contact force measurement/3','autorouting','on');
add_line(parent,'Issued selected actuator request/1','Head motion estimator/2','autorouting','on');
add_block('simulink/Continuous/State-Space',[sys '/Identified causal current lag'], ...
 'A','p.d.upperMotionObserver.forcePredictionA','B','p.d.upperMotionObserver.forcePredictionB', ...
 'C','p.d.upperMotionObserver.forcePredictionC','D','p.d.upperMotionObserver.forcePredictionD', ...
 'InitialCondition','p.d.upperMotionObserver.forcePredictionInitial_A','Position',[300 975 570 1055]);
add_block('simulink/Math Operations/Gain',[sys '/Identified current to force'],'Gain','p.uncertainty.forceConstant_model_N_A','Position',[620 975 845 1055]);
add_block('simulink/Math Operations/Bias',[sys '/Remove identified loaded equilibrium'],'Bias','-p.d.upperMotionObserver.forcePredictionInitial_A*p.uncertainty.forceConstant_model_N_A','Position',[900 975 1150 1055]);
add_block([sys '/Robot acceleration common AA and EMA'],[sys '/Issued force common ADC frame'],'Position',[1200 975 1470 1055]);
add_block([sys '/Robot acceleration frame matching delay'],[sys '/Issued force frame delivery pipeline'],'Position',[1520 975 1760 1055]);
add_block('simulink/Discrete/Zero-Order Hold',[sys '/Fresh 10 kHz issued force frame'],'SampleTime','p.force.sampleTime','Position',[1810 975 2050 1055]);
ff=[mdl '/Force controller/Coil PI and PID/Measured motion feedforward'];
% Algorithm-only Coulomb feedforward experiment. Physical and observer guide
% friction keep their exact .2N,2Ns/m,.2mm/s identified parameters.
set_param([ff '/Guide friction velocity scale'],'Gain','1/p.guideFeedforwardStudy.smoothingSpeed_m_s');
set_param([ff '/Guide Coulomb friction current'],'Gain','p.inertial.guideFrictionFeedforwardScale*p.guideFeedforwardStudy.coulombScale*p.coil.coulombForce/p.coil.forceConstant');
% Estimate physical guide friction in the corrected measurement frame only.
% The existing 400us present-time feedforward prediction remains independent.
add_block('simulink/Math Operations/Gain',[sys '/Measured-frame guide velocity scale'],'Gain','1/p.coil.frictionSpeed','Position',[1200 1160 1470 1215]);
add_block('simulink/Math Operations/Trigonometric Function',[sys '/Measured-frame friction direction'],'Operator','tanh','Position',[1520 1150 1760 1230]);
add_block('simulink/Math Operations/Gain',[sys '/Identified measured-frame Coulomb force'],'Gain','p.coil.coulombForce','Position',[1810 1150 2050 1230]);
add_block('simulink/Math Operations/Gain',[sys '/Identified measured-frame viscous force'],'Gain','p.coil.viscousFriction','Position',[1520 1300 1760 1380]);
add_block('simulink/Math Operations/Sum',[sys '/Physical guide friction in measurement frame'],'Inputs','++','Position',[2110 1190 2340 1350]);
add_block('simulink/Math Operations/Sum',[sys '/Aligned transmitted actuator force deviation'],'Inputs','+-','Position',[2110 975 2340 1145]);
add_line(sys,'Issued post-limit current request/1','Identified causal current lag/1');
add_line(sys,'Identified causal current lag/1','Identified current to force/1');
add_line(sys,'Identified current to force/1','Remove identified loaded equilibrium/1');
add_line(sys,'Remove identified loaded equilibrium/1','Issued force common ADC frame/1');
add_line(sys,'Issued force common ADC frame/1','Issued force frame delivery pipeline/1');
add_line(sys,'Issued force frame delivery pipeline/1','Fresh 10 kHz issued force frame/1');
add_line(sys,'Head gap velocity/1','Measured-frame guide velocity scale/1','autorouting','on');
add_line(sys,'Measured-frame guide velocity scale/1','Measured-frame friction direction/1');
add_line(sys,'Measured-frame friction direction/1','Identified measured-frame Coulomb force/1');
add_line(sys,'Head gap velocity/1','Identified measured-frame viscous force/1','autorouting','on');
add_line(sys,'Identified measured-frame Coulomb force/1','Physical guide friction in measurement frame/1');
add_line(sys,'Identified measured-frame viscous force/1','Physical guide friction in measurement frame/2');
add_line(sys,'Fresh 10 kHz issued force frame/1','Aligned transmitted actuator force deviation/1');
add_line(sys,'Physical guide friction in measurement frame/1','Aligned transmitted actuator force deviation/2');
add_line(sys,'Aligned transmitted actuator force deviation/1','Causal motion measurements/2','autorouting','on');
% A motion is reconstructed in the same measurement frame before geometry/FF.
add_block('simulink/Signal Routing/Selector',[sys '/Estimated upper velocity deviation'],'InputPortWidth','4','Indices','2','Position',[1390 800 1640 855]);
add_block('simulink/Math Operations/Sum',[sys '/Estimated aligned A velocity'],'Inputs','++','Position',[1730 735 1960 845]);
add_block('simulink/Signal Routing/Goto',[sys '/Aligned A velocity'],'GotoTag','upper_estimated_A_velocity_aligned','TagVisibility','global','Position',[2050 735 2320 765]);
add_block('simulink/Math Operations/Gain',[sys '/Estimated A acceleration from upper states'],'Gain','p.d.upperMotionObserver.accelerationFromStateGain','Multiplication','Matrix(K*u)','Position',[2400 790 2690 875]);
add_block('simulink/Math Operations/Gain',[sys '/Estimated A acceleration from issued force'],'Gain','p.d.upperMotionObserver.accelerationFromForceGain','Position',[2400 955 2690 1040]);
add_block('simulink/Math Operations/Sum',[sys '/Estimated aligned A acceleration'],'Inputs','++','Position',[2770 790 3010 970]);
add_block('simulink/Signal Routing/Goto',[sys '/Aligned A acceleration'],'GotoTag','upper_estimated_A_acceleration_aligned','TagVisibility','global','Position',[3090 810 3390 840]);
add_line(sys,'Head gap and velocity observer/1','Estimated upper velocity deviation/1','autorouting','on');
add_line(sys,'Fresh 10 kHz robot velocity frame/1','Estimated aligned A velocity/1','autorouting','on');
add_line(sys,'Estimated upper velocity deviation/1','Estimated aligned A velocity/2');
add_line(sys,'Estimated aligned A velocity/1','Aligned A velocity/1');
add_line(sys,'Head gap and velocity observer/1','Estimated A acceleration from upper states/1','autorouting','on');
add_line(sys,'Aligned transmitted actuator force deviation/1','Estimated A acceleration from issued force/1','autorouting','on');
add_line(sys,'Estimated A acceleration from upper states/1','Estimated aligned A acceleration/1');
add_line(sys,'Estimated A acceleration from issued force/1','Estimated aligned A acceleration/2');
add_line(sys,'Estimated aligned A acceleration/1','Aligned A acceleration/1');
set_param([mdl '/Contact force measurement/Normal force projection/Issued robot velocity'],'GotoTag','upper_estimated_A_velocity_aligned');
set_param([ff '/Robot acceleration'],'GotoTag','upper_estimated_A_acceleration_aligned');
% Separate state prediction from measurement correction so the native block's
% direct-feedthrough flag cannot invent a delta-force/friction algebraic loop.
% This is exactly x_corrected = (I-LH)*x_prior + L*position, with the same
% Aobs/Bobs update as above; no extra measurement delay is introduced.
set_param(observer,'C','eye(4)','D','zeros(4,4)');
oph=get_param(observer,'PortHandles');oldLine=get_param(oph.Outport(1),'Line');dest=get_param(oldLine,'DstPortHandle');delete_line(oldLine);
add_block('simulink/Math Operations/Gain',[sys '/Prior state correction matrix'],'Gain','p.d.upperMotionObserver.C','Multiplication','Matrix(K*u)','Position',[1050 1420 1320 1500]);
add_block('simulink/Math Operations/Gain',[sys '/Position correction gain'],'Gain','p.d.upperMotionObserver.gain','Multiplication','Matrix(K*u)','Position',[1050 1550 1320 1630]);
add_block('simulink/Math Operations/Sum',[sys '/Corrected upper motion states'],'Inputs','++','Position',[1410 1420 1630 1600]);
add_line(sys,'Head gap and velocity observer/1','Prior state correction matrix/1');
add_line(sys,'Prior state correction matrix/1','Corrected upper motion states/1');
add_line(sys,'Fresh 10 kHz position frame/1','Position correction gain/1','autorouting','on');
add_line(sys,'Position correction gain/1','Corrected upper motion states/2');
corrected=get_param([sys '/Corrected upper motion states'],'PortHandles');
for j=1:numel(dest),add_line(sys,corrected.Outport(1),dest(j),'autorouting','on');end
contract.guideFeedforwardCoulombScale=p.guideFeedforwardStudy.coulombScale;
contract.guideFeedforwardSmoothingSpeed_m_s=p.guideFeedforwardStudy.smoothingSpeed_m_s;
contract.guideFeedforwardExperimentScope='Controller present-time Coulomb compensation only; physical/observer friction and controller viscous compensation unchanged.';
contract.issuedRequestSource='Explicit root Force controller/1 (same signal as physical Actuator and RABS mechanics/1) via Contact force measurement/Inport3 and Head motion estimator/Inport2';
contract.issuedRequestRootLocalTag=false;
contract.stateNames=p.d.upperMotionObserver.stateNames;contract.inputNames=p.d.upperMotionObserver.inputNames;
contract.observabilityRank=p.d.upperMotionObserver.observabilityRank;contract.maximumPoleMagnitude=p.d.upperMotionObserver.maximumPoleMagnitude;
contract.gain=p.d.upperMotionObserver.gain;contract.causality='Post-limit issued request only; current lag D=0; matched existing ADC frame; no-lead physical guide-friction estimate from corrected measured-frame gap velocity; no actual A/current/force/surface truth.';
contract.predictorLimitation='Identified current response is a linear lag; electrical voltage clipping and operating-point identification errors are not reproduced exactly by this observer prediction. Native physical drive still has its full electrical clipping.';
end
