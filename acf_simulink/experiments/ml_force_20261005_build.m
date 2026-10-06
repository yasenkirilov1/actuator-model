function [mdl,audit]=ml_force_20261005_build(modelPath,weightsFile,residualGain)
%ML_FORCE_20261005_BUILD Add bounded native neural residual to private model.
% Caller owns the model and saving. No canonical source is modified.
% The sensor-lag teacher is approximate; it omits lower-link inverse changes.
if nargin<2,weightsFile='';end
if nargin<3,residualGain=0;end
handle=load_system(modelPath);mdl=get_param(handle,'Name');
assert(getSimulinkBlockHandle([mdl '/Force controller/Coil PI and PID/Learned causal current residual'])==-1);
ml=struct('W1',zeros(8,10),'b1',zeros(8,1),'W2',zeros(1,8),'b2',0, ...
 'mean',zeros(10,1),'scale',ones(10,1),'gain',residualGain,'actionLimit_A',.05);
if ~isempty(weightsFile)
 learned=load(weightsFile);fields=fieldnames(ml);
 for k=1:numel(fields),if isfield(learned,fields{k}),ml.(fields{k})=learned.(fields{k});end,end
 ml.gain=residualGain;
end
assignin(get_param(mdl,'ModelWorkspace'),'ml',ml);add_residual(mdl);
audit=struct('nativeInference',true,'residualBound_A',.05,'gain',residualGain, ...
 'truthOnlyInLogger',true,'freshClock_s',100e-6,'architecture','10-8-1 ReLU', ...
 'inputs',{{'reconstructed normal deviation','100 us normal increment','existing predicted acceleration current', ...
 '100 us acceleration-current increment','observed head gap velocity','100 us velocity increment', ...
 'estimated slope','100 us slope increment','identified S mass','force target'}}, ...
 'insertion','seventh current-sum input, before notch/current limits and antiwindup');
end

function add_residual(mdl)
path=[mdl '/Force controller/Coil PI and PID'];ff=[path '/Measured motion feedforward'];
add_block('simulink/Signal Routing/Goto',[ff '/ML causal acceleration tap'],'GotoTag','ml_ff_acc_prediction','TagVisibility','global','Position',[1320 510 1580 540]);
ph=get_param([ff '/Causal acceleration for current FF'],'PortHandles');gh=get_param([ff '/ML causal acceleration tap'],'PortHandles');add_line(ff,ph.Outport(1),gh.Inport(1),'autorouting','on');
sys=[path '/Learned causal current residual'];add_block('simulink/Ports & Subsystems/Subsystem',sys,'Position',[450 750 730 870]);Simulink.SubSystem.deleteContents(sys);
tags={'estimated_normal_contact_N','ml_ff_acc_prediction','inertial_head_gap_velocity','estimated_contact_slope'};
for k=1:4
 y=80*k;name=['Causal input ' num2str(k)];add_block('simulink/Signal Routing/From',[sys '/' name],'GotoTag',tags{k},'Position',[20 y 210 y+25]);
 add_block('simulink/Discrete/Zero-Order Hold',[sys '/Fresh frame ' num2str(k)],'SampleTime','p.force.sampleTime','Position',[245 y 340 y+25]);
 add_line(sys,[name '/1'],['Fresh frame ' num2str(k) '/1']);
 if k==1
  add_block('simulink/Math Operations/Bias',[sys '/Normal deviation'],'Bias','-p.force.reference','Position',[375 y 510 y+25]);src='Normal deviation';
 elseif k==2
  add_block('simulink/Math Operations/Gain',[sys '/Acceleration FF current'],'Gain','-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)/p.uncertainty.forceConstant_model_N_A','Position',[375 y 510 y+25]);src='Acceleration FF current';
 else
  src=['Value ' num2str(k)];add_block('simulink/Math Operations/Gain',[sys '/' src],'Gain','1','Position',[375 y 510 y+25]);
 end
 add_line(sys,['Fresh frame ' num2str(k) '/1'],[src '/1']);
 add_block('simulink/Discrete/Unit Delay',[sys '/Previous ' num2str(k)],'SampleTime','p.force.sampleTime','InitialCondition','0','Position',[555 y+30 640 y+60]);
 add_block('simulink/Math Operations/Sum',[sys '/Increment ' num2str(k)],'Inputs','+-','Position',[700 y 740 y+35]);
 add_line(sys,[src '/1'],['Previous ' num2str(k) '/1']);add_line(sys,[src '/1'],['Increment ' num2str(k) '/1']);add_line(sys,['Previous ' num2str(k) '/1'],['Increment ' num2str(k) '/2']);
end
add_block('simulink/Signal Routing/Mux',[sys '/Causal feature vector'],'Inputs','10','Position',[815 55 820 465]);
srcs={'Normal deviation','Increment 1','Acceleration FF current','Increment 2','Value 3','Increment 3','Value 4','Increment 4'};
for k=1:8,add_line(sys,[srcs{k} '/1'],['Causal feature vector/' num2str(k)]);end
add_block('simulink/Sources/Constant',[sys '/Identified tool mass'],'Value','p.sensor.distalMass_model_kg','Position',[555 415 740 440]);
add_block('simulink/Sources/Constant',[sys '/Force target'],'Value','p.force.reference','Position',[555 465 740 490]);
add_line(sys,'Identified tool mass/1','Causal feature vector/9');add_line(sys,'Force target/1','Causal feature vector/10');
add_block('simulink/Math Operations/Bias',[sys '/Training mean'],'Bias','-ml.mean','Position',[890 175 1010 210]);
add_block('simulink/Math Operations/Gain',[sys '/Training scale'],'Gain','1./ml.scale','Multiplication','Element-wise(K.*u)','Position',[1060 175 1180 210]);
add_block('simulink/Discontinuities/Saturation',[sys '/Feature bounds'],'LowerLimit','-6','UpperLimit','6','Position',[1230 175 1350 210]);
add_block('simulink/Math Operations/Gain',[sys '/Hidden weights'],'Gain','ml.W1','Multiplication','Matrix(K*u)','Position',[1400 175 1520 210]);
add_block('simulink/Math Operations/Bias',[sys '/Hidden biases'],'Bias','ml.b1','Position',[1570 175 1690 210]);
add_block('simulink/Discontinuities/Saturation',[sys '/Eight ReLU neurons'],'LowerLimit','0','UpperLimit','inf','Position',[1740 175 1860 210]);
add_block('simulink/Math Operations/Gain',[sys '/Output weights'],'Gain','ml.W2','Multiplication','Matrix(K*u)','Position',[1910 175 2030 210]);
add_block('simulink/Math Operations/Bias',[sys '/Output bias'],'Bias','ml.b2','Position',[2080 175 2200 210]);
add_block('simulink/Discontinuities/Saturation',[sys '/Residual current bound'],'LowerLimit','-ml.actionLimit_A','UpperLimit','ml.actionLimit_A','Position',[2250 175 2390 210]);
add_block('simulink/Math Operations/Gain',[sys '/Selected residual gain'],'Gain','ml.gain','Position',[2440 175 2600 210]);
add_block('simulink/Ports & Subsystems/Out1',[sys '/extra_current_A'],'Position',[2670 180 2710 200]);
chain={'Causal feature vector','Training mean','Training scale','Feature bounds','Hidden weights','Hidden biases','Eight ReLU neurons','Output weights','Output bias','Residual current bound','Selected residual gain','extra_current_A'};
for k=1:numel(chain)-1,add_line(sys,[chain{k} '/1'],[chain{k+1} '/1']);end
% Truth has only logger destinations and cannot reach the inference chain.
add_block('simulink/Signal Routing/Mux',[sys '/Training and review log'],'Inputs','6','Position',[1710 590 1715 835]);
add_line(sys,'Causal feature vector/1','Training and review log/1');add_line(sys,'Selected residual gain/1','Training and review log/2');
tags={'normal_contact_N','physical_S_acceleration_z','estimated_normal_contact_N','ml_ff_acc_prediction'};
for k=1:4
 add_block('simulink/Signal Routing/From',[sys '/Label only ' num2str(k)],'GotoTag',tags{k},'Position',[1230 610+50*k 1480 635+50*k]);
 add_block('simulink/Discrete/Zero-Order Hold',[sys '/Label frame ' num2str(k)],'SampleTime','p.force.sampleTime','Position',[1520 610+50*k 1630 635+50*k]);
 add_line(sys,['Label only ' num2str(k) '/1'],['Label frame ' num2str(k) '/1']);add_line(sys,['Label frame ' num2str(k) '/1'],['Training and review log/' num2str(k+2)]);
end
add_block('simulink/Sinks/To Workspace',[sys '/ML feature recording'],'VariableName','ml_features_log','SaveFormat','Timeseries','MaxDataPoints','inf','Decimation','1','SampleTime','p.force.sampleTime','Position',[1840 690 2060 740]);
add_line(sys,'Training and review log/1','ML feature recording/1');
set_param([path '/PI PID and feedforward current'],'Inputs','+++++++');add_line(path,'Learned causal current residual/1','PI PID and feedforward current/7','autorouting','on');
end
