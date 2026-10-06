function ml_robot_20261005_build_policy(mdl)
sys=[mdl '/Recenter travel'];ph=get_param([sys '/Send correction only when settled'],'PortHandles');
line=get_param(ph.Inport(1),'Line');delete_line(line);
% Keep the original teacher blocks intact as a comparator and safety envelope.
add_block('simulink/Ports & Subsystems/Subsystem',[sys '/Learned robot policy'],'Position',[810 -205 1080 -85]);
ns=[sys '/Learned robot policy'];Simulink.SubSystem.deleteContents(ns);
add_block('simulink/Sources/In1',[ns '/causal_stroke_error_m'],'Position',[20 113 50 127]);
add_block('simulink/Math Operations/Gain',[ns '/Normalize distance'],'Gain','1/p.mlRobot.inputScale_m','Position',[90 95 220 145]);
add_block('simulink/Math Operations/Gain',[ns '/Hidden learned weights'],'Gain','p.mlRobot.W1','Multiplication','Matrix(K*u)','Position',[260 95 415 145]);
add_block('simulink/Math Operations/Bias',[ns '/Hidden learned biases'],'Bias','p.mlRobot.b1','Position',[455 95 605 145]);
add_block('simulink/Discontinuities/Saturation',[ns '/ReLU eight neurons'],'UpperLimit','inf','LowerLimit','0','Position',[650 95 800 145]);
add_block('simulink/Math Operations/Gain',[ns '/Output learned weights'],'Gain','p.mlRobot.W2','Multiplication','Matrix(K*u)','Position',[845 95 1000 145]);
add_block('simulink/Math Operations/Bias',[ns '/Output learned bias and zero'],'Bias','p.mlRobot.b2-p.mlRobot.zeroOutput','Position',[1045 95 1240 145]);
add_block('simulink/Math Operations/Gain',[ns '/Distance in metres'],'Gain','p.mlRobot.inputScale_m','Position',[1280 95 1445 145]);
add_block('simulink/Sinks/Out1',[ns '/learned_deadband_distance_m'],'Position',[1500 113 1530 127]);
chain={'causal_stroke_error_m','Normalize distance','Hidden learned weights','Hidden learned biases','ReLU eight neurons','Output learned weights','Output learned bias and zero','Distance in metres','learned_deadband_distance_m'};
for k=1:numel(chain)-1,add_line(ns,[chain{k} '/1'],[chain{k+1} '/1'],'autorouting','on');end
add_block('simulink/Math Operations/Gain',[sys '/Hard delay bounded gain'],'Gain','p.uncertainty.recenterGain','Position',[1110 -195 1270 -145]);
add_block('simulink/Math Operations/MinMax',[sys '/Positive teacher envelope'],'Function','max','Inputs','2','Position',[1100 -25 1215 25]);
add_block('simulink/Math Operations/MinMax',[sys '/Negative teacher envelope'],'Function','min','Inputs','2','Position',[1100 35 1215 85]);
add_block('simulink/Discontinuities/Saturation Dynamic',[sys '/Hard sign and gain envelope'],'Position',[1320 -165 1480 -55]);
add_block('simulink/Sources/Constant',[sys '/Use learned robot policy'],'Value','p.mlRobot.enabled','Position',[1550 -225 1720 -175]);
add_block('simulink/Signal Routing/Switch',[sys '/Teacher or trained robot policy'],'Threshold','.5','Position',[1760 -120 1810 -10]);
add_line(sys,'Distance from mid stroke/1','Learned robot policy/1','autorouting','on');
add_line(sys,'Learned robot policy/1','Hard delay bounded gain/1','autorouting','on');
add_line(sys,'Hard delay bounded gain/1','Hard sign and gain envelope/2','autorouting','on');
add_line(sys,'Recenter gain/1','Positive teacher envelope/1','autorouting','on');
add_line(sys,'Recenter gain/1','Negative teacher envelope/1','autorouting','on');
add_line(sys,'Hold recentering/1','Positive teacher envelope/2','autorouting','on');
add_line(sys,'Hold recentering/1','Negative teacher envelope/2','autorouting','on');
add_line(sys,'Positive teacher envelope/1','Hard sign and gain envelope/1','autorouting','on');
add_line(sys,'Negative teacher envelope/1','Hard sign and gain envelope/3','autorouting','on');
add_line(sys,'Hard sign and gain envelope/1','Teacher or trained robot policy/1','autorouting','on');
add_line(sys,'Use learned robot policy/1','Teacher or trained robot policy/2','autorouting','on');
add_line(sys,'Recenter gain/1','Teacher or trained robot policy/3','autorouting','on');
add_line(sys,'Teacher or trained robot policy/1','Send correction only when settled/1','autorouting','on');
% 100 Hz causal features and inference audit, independent of native scoring.
add_block('simulink/Signal Routing/Mux',[sys '/ML robot policy audit vector'],'Inputs','8','Position',[2030 -80 2035 310]);
add_block('simulink/Signal Attributes/Data Type Conversion',[sys '/ML force gate as double'], ...
 'OutDataTypeStr','double','Position',[1880 260 1990 310]);
add_line(sys,'Force settling gate/1','ML force gate as double/1','autorouting','on');
blocks={'Distance from mid stroke','Recenter gain','Hard sign and gain envelope','Teacher or trained robot policy', ...
 'ML force gate as double','Send correction only when settled','Robot feedback sampling','Learned robot policy'};
for k=1:8,add_line(sys,[blocks{k} '/1'],['ML robot policy audit vector/' num2str(k)],'autorouting','on');end
add_block('simulink/Sinks/To Workspace',[sys '/ML robot policy causal log'],'VariableName','ml_robot_policy_log', ...
 'SaveFormat','Timeseries','MaxDataPoints','inf','Decimation','1','SampleTime','p.robot.sampleTime','Position',[2110 20 2340 80]);
add_line(sys,'ML robot policy audit vector/1','ML robot policy causal log/1','autorouting','on');
end

