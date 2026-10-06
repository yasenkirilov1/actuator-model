function contract=install_upper_observer(mdl,p)
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
add_block('simulink/Signal Routing/From',[sys '/Issued post-limit current request'],'GotoTag','actuator_command','Position',[20 1000 260 1030]);
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
add_block('simulink/Signal Routing/Goto',[ff '/Upper observer guide friction tap'],'GotoTag','upper_estimated_guide_friction_current','TagVisibility','global','Position',[1500 600 1770 630]);
add_line(ff,'Guide friction feedforward/1','Upper observer guide friction tap/1','autorouting','on');
add_block('simulink/Signal Routing/From',[sys '/Measured guide friction estimate'],'GotoTag','upper_estimated_guide_friction_current','Position',[1200 1160 1480 1190]);
add_block('simulink/Discrete/Unit Delay',[sys '/Previous guide friction estimate'],'SampleTime','p.force.sampleTime','InitialCondition','0','Position',[1520 1135 1760 1215]);
add_block('simulink/Math Operations/Gain',[sys '/Identified friction current to force'],'Gain','p.uncertainty.forceConstant_model_N_A','Position',[1810 1135 2050 1215]);
add_block('simulink/Math Operations/Sum',[sys '/Aligned transmitted actuator force deviation'],'Inputs','+-','Position',[2110 975 2340 1145]);
add_line(sys,'Issued post-limit current request/1','Identified causal current lag/1');
add_line(sys,'Identified causal current lag/1','Identified current to force/1');
add_line(sys,'Identified current to force/1','Remove identified loaded equilibrium/1');
add_line(sys,'Remove identified loaded equilibrium/1','Issued force common ADC frame/1');
add_line(sys,'Issued force common ADC frame/1','Issued force frame delivery pipeline/1');
add_line(sys,'Issued force frame delivery pipeline/1','Fresh 10 kHz issued force frame/1');
add_line(sys,'Measured guide friction estimate/1','Previous guide friction estimate/1');
add_line(sys,'Previous guide friction estimate/1','Identified friction current to force/1');
add_line(sys,'Fresh 10 kHz issued force frame/1','Aligned transmitted actuator force deviation/1');
add_line(sys,'Identified friction current to force/1','Aligned transmitted actuator force deviation/2');
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
contract.stateNames=p.d.upperMotionObserver.stateNames;contract.inputNames=p.d.upperMotionObserver.inputNames;
contract.observabilityRank=p.d.upperMotionObserver.observabilityRank;contract.maximumPoleMagnitude=p.d.upperMotionObserver.maximumPoleMagnitude;
contract.gain=p.d.upperMotionObserver.gain;contract.causality='Post-limit issued request only; current lag D=0; matched existing ADC frame; guide-friction estimate delayed one control tick; no actual A/current/force/surface truth.';
contract.predictorLimitation='Identified current response is a linear lag; electrical voltage clipping and operating-point identification errors are not reproduced exactly by this observer prediction. Native physical drive still has its full electrical clipping.';
end
