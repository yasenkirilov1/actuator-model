function report=build_normal_contact_trial_v5(config)
%BUILD_NORMAL_CONTACT_TRIAL_V5 Native fast force-settling gate and optional FF lead.
if nargin<1,config=struct;end
if ~isfield(config,'sourceFolder'),config.sourceFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v4-20260930/frozen-inertial-v4';end
if ~isfield(config,'workFolder'),config.workFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v5-20260930';end
assert(~usejava('desktop') && ~strcmp(config.sourceFolder,config.workFolder));
for n={'','helpers','cache','code','sdi','traces'}
 f=fullfile(config.workFolder,n{1});if ~isfolder(f),mkdir(f);end
end
copyfile(fullfile(config.sourceFolder,'acf_force_control_normal_trial.slx'),fullfile(config.workFolder,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(config.sourceFolder,'helpers','*.m'),fullfile(config.workFolder,'helpers'));
patch_prepare(fullfile(config.workFolder,'helpers','acf_prepare.m'));
patch_runner(fullfile(config.workFolder,'helpers','acf_run.m'));
addpath(fullfile(config.workFolder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator acf_select_controller;
Simulink.fileGenControl('set','CacheFolder',fullfile(config.workFolder,'cache'), ...
 'CodeGenFolder',fullfile(config.workFolder,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(config.workFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(config.workFolder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error');
z=load(fullfile(config.sourceFolder,'normal_contact_trial_candidate.mat'),'p');p=z.p;
p.robot.forceGate.variation=struct('enabled',false,'meanTime',.1,'varianceTime',.02,'rmsLimit_N',.08);
p.inertial.accelerationPrediction_s=0;p.inertial.accelerationDerivativeFilter_s=.0003;
p=acf_prepare(p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
gate=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'Name','Force settling gate');assert(numel(gate)==1);gate=gate{1};
add_variation_gate(gate);
add_acceleration_prediction([mdl '/Force controller/Coil PI and PID/Measured motion feedforward']);
set_param(mdl,'SimulationCommand','update');loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);assert(isempty(loops));
save_system(mdl,fullfile(config.workFolder,[mdl '.slx']));save(fullfile(config.workFolder,'normal_contact_trial_candidate.mat'),'p');
report=struct('version','inertial-v5','sourceFolder',config.sourceFolder,'workFolder',config.workFolder, ...
 'variationGate',p.robot.forceGate.variation,'gateSampleTime_s',p.force.sampleTime, ...
 'dwellSamples',p.d.variationGate.dwellSamples,'meanCoefficient',p.d.variationGate.meanDenominator(2)*-1, ...
 'varianceCoefficient',p.d.variationGate.varianceDenominator(2)*-1, ...
 'accelerationPrediction_s',p.inertial.accelerationPrediction_s, ...
 'accelerationDerivativeFilter_s',p.inertial.accelerationDerivativeFilter_s,'algebraicLoopCount',numel(loops));
end

function add_variation_gate(gate)
page=[gate '/Fast force variation and dwell'];
add_block('simulink/Ports & Subsystems/Subsystem',page,'Position',[20 1030 340 1180]);
Simulink.SubSystem.deleteContents(page);
block(page,'simulink/Ports & Subsystems/In1','normal_error_N',[20 75 50 89],'Port','1');
block(page,'simulink/Ports & Subsystems/In1','held_slow_stability',[20 485 50 499],'Port','2');
block(page,'simulink/Discrete/Zero-Order Hold','Fast error sample',[110 60 235 105],'SampleTime','p.force.sampleTime');
block(page,'simulink/Math Operations/Gain','Normal force above target',[300 60 440 105],'Gain','-1');
block(page,'simulink/Discrete/Discrete Transfer Fcn','Fast normal force mean',[500 150 700 205], ...
 'Numerator','p.d.variationGate.meanNumerator','Denominator','p.d.variationGate.meanDenominator', ...
 'InitialStates','0','SampleTime','p.force.sampleTime');
block(page,'simulink/Math Operations/Sum','Normal force deviation',[770 55 805 170],'Inputs','+-','IconShape','rectangular');
block(page,'simulink/Math Operations/Math Function','Squared normal force deviation',[860 80 1000 135],'Operator','square');
block(page,'simulink/Discrete/Discrete Transfer Fcn','Fast force variance',[1060 80 1250 135], ...
 'Numerator','p.d.variationGate.varianceNumerator','Denominator','p.d.variationGate.varianceDenominator', ...
 'InitialStates','0','SampleTime','p.force.sampleTime');
block(page,'simulink/Math Operations/Math Function','RMS force variation',[1320 55 1490 110],'Operator','sqrt');
block(page,'simulink/Sources/Constant','Allowed force variance',[1090 255 1250 300],'Value','p.robot.forceGate.variation.rmsLimit_N^2');
block(page,'simulink/Logic and Bit Operations/Relational Operator','Variation acceptable',[1340 175 1390 250],'Operator','<=');
block(page,'simulink/Discrete/Zero-Order Hold','Held slow stability at fast rate',[110 470 330 515],'SampleTime','p.force.sampleTime');
block(page,'simulink/Logic and Bit Operations/Logical Operator','All force stability conditions',[470 420 520 510],'Operator','AND','Inputs','2');
block(page,'simulink/Discrete/Unit Delay','Previous fast stable count',[470 700 680 750],'SampleTime','p.force.sampleTime','InitialCondition','0');
block(page,'simulink/Math Operations/Bias','One more fast stable sample',[740 700 960 750],'Bias','1');
block(page,'simulink/Sources/Constant','Reset fast dwell',[750 540 915 580],'Value','0');
block(page,'simulink/Signal Routing/Switch','Reset dwell at every fast instability',[1040 445 1095 610],'Criteria','u2 >= Threshold','Threshold','.5');
block(page,'simulink/Discontinuities/Saturation','Cap fast stable count',[1160 470 1340 530],'UpperLimit','p.d.variationGate.dwellSamples','LowerLimit','0');
block(page,'simulink/Sources/Constant','Required fast dwell count',[1160 680 1370 725],'Value','p.d.variationGate.dwellSamples');
block(page,'simulink/Logic and Bit Operations/Relational Operator','Fast continuous dwell completed',[1440 465 1490 610],'Operator','>=');
names={'ready_fast','mean_error_N','variation_rms_N','stable_sample_count','stable_fast'};
positions={[1660 480 1690 494],[1660 170 1690 184],[1660 80 1690 94],[1660 575 1690 589],[1660 730 1690 744]};
for j=1:5,block(page,'simulink/Ports & Subsystems/Out1',names{j},positions{j},'Port',num2str(j));end
wires={ ...
 'normal_error_N/1','Fast error sample/1';'Fast error sample/1','Normal force above target/1'; ...
 'Normal force above target/1','Fast normal force mean/1';'Normal force above target/1','Normal force deviation/1'; ...
 'Fast normal force mean/1','Normal force deviation/2';'Normal force deviation/1','Squared normal force deviation/1'; ...
 'Squared normal force deviation/1','Fast force variance/1';'Fast force variance/1','RMS force variation/1'; ...
 'Fast force variance/1','Variation acceptable/1';'Allowed force variance/1','Variation acceptable/2'; ...
 'held_slow_stability/1','Held slow stability at fast rate/1';'Held slow stability at fast rate/1','All force stability conditions/1'; ...
 'Variation acceptable/1','All force stability conditions/2';'Previous fast stable count/1','One more fast stable sample/1'; ...
 'One more fast stable sample/1','Reset dwell at every fast instability/1'; ...
 'All force stability conditions/1','Reset dwell at every fast instability/2';'Reset fast dwell/1','Reset dwell at every fast instability/3'; ...
 'Reset dwell at every fast instability/1','Cap fast stable count/1';'Cap fast stable count/1','Previous fast stable count/1'; ...
 'Cap fast stable count/1','Fast continuous dwell completed/1';'Required fast dwell count/1','Fast continuous dwell completed/2'; ...
 'Fast continuous dwell completed/1','ready_fast/1';'Fast normal force mean/1','mean_error_N/1'; ...
 'RMS force variation/1','variation_rms_N/1';'Cap fast stable count/1','stable_sample_count/1'; ...
 'All force stability conditions/1','stable_fast/1'};
for j=1:size(wires,1),add_line(page,wires{j,1},wires{j,2},'autorouting','on');end
block(gate,'simulink/Discrete/Zero-Order Hold','Fast ready at robot rate',[430 1070 650 1120],'SampleTime','p.robot.sampleTime');
block(gate,'simulink/Sources/Constant','Fast variation gate enabled',[755 1035 955 1080],'Value','double(p.robot.forceGate.variation.enabled)');
block(gate,'simulink/Signal Routing/Switch','Select continuous fast dwell',[1160 910 1215 1050],'Criteria','u2 >= Threshold','Threshold','.5');
add_line(gate,'force_error_N/1','Fast force variation and dwell/1','autorouting','on');
add_line(gate,'Force stable now/1','Fast force variation and dwell/2','autorouting','on');
add_line(gate,'Fast force variation and dwell/1','Fast ready at robot rate/1','autorouting','on');
ports=get_param([gate '/Ready or bypassed'],'PortHandles');delete_line(get_param(ports.Inport(2),'Line'));
add_line(gate,'Fast ready at robot rate/1','Select continuous fast dwell/1','autorouting','on');
add_line(gate,'Fast variation gate enabled/1','Select continuous fast dwell/2','autorouting','on');
add_line(gate,'Dwell completed/1','Select continuous fast dwell/3','autorouting','on');
add_line(gate,'Select continuous fast dwell/1','Ready or bypassed/2','autorouting','on');
add_native_log(gate,'Fast force variation and dwell',[2 3 4 5 1], ...
 'force_variation_log','Native fast stability',[1900 1030]);
Simulink.BlockDiagram.routeLine(find_system(page,'FindAll','on','SearchDepth',1,'Type','line'));
end

function add_acceleration_prediction(page)
target=[page '/Measured acceleration current feedforward'];ports=get_param(target,'PortHandles');
delete_line(get_param(ports.Inport(1),'Line'));
set_param(target,'Position',[1130 30 1370 85]);
block(page,'simulink/Discrete/Discrete Transfer Fcn','Filtered measured acceleration rate',[340 95 610 150], ...
 'Numerator','p.d.accelerationPrediction.derivativeNumerator','Denominator','p.d.accelerationPrediction.derivativeDenominator', ...
 'InitialStates','0','SampleTime','p.force.sampleTime');
block(page,'simulink/Math Operations/Gain','Acceleration FF prediction time',[680 95 900 150],'Gain','p.inertial.accelerationPrediction_s');
block(page,'simulink/Math Operations/Sum','Causal acceleration for current FF',[990 30 1030 160],'Inputs','++','IconShape','rectangular');
add_line(page,'Measured S acceleration Z/1','Filtered measured acceleration rate/1','autorouting','on');
add_line(page,'Filtered measured acceleration rate/1','Acceleration FF prediction time/1','autorouting','on');
add_line(page,'Acceleration FF prediction time/1','Causal acceleration for current FF/2','autorouting','on');
add_line(page,'Measured S acceleration Z/1','Causal acceleration for current FF/1','autorouting','on');
add_line(page,'Causal acceleration for current FF/1','Measured acceleration current feedforward/1','autorouting','on');
block(page,'simulink/Signal Routing/Mux','Acceleration FF audit vector',[1200 740 1205 840],'Inputs','2');
block(page,'simulink/Discrete/Zero-Order Hold','Acceleration FF audit native rate',[1260 760 1420 810],'SampleTime','p.sim.step');
block(page,'simulink/Sinks/To Workspace','Acceleration FF native log',[1510 760 1700 810],'VariableName','acceleration_feedforward_log','SaveFormat','Timeseries');
add_line(page,'Measured S acceleration Z/1','Acceleration FF audit vector/1','autorouting','on');
add_line(page,'Causal acceleration for current FF/1','Acceleration FF audit vector/2','autorouting','on');
add_line(page,'Acceleration FF audit vector/1','Acceleration FF audit native rate/1','autorouting','on');
add_line(page,'Acceleration FF audit native rate/1','Acceleration FF native log/1','autorouting','on');
Simulink.BlockDiagram.routeLine(find_system(page,'FindAll','on','SearchDepth',1,'Type','line'));
end

function add_native_log(page,source,ports,variable,label,origin)
n=numel(ports);x=origin(1);y=origin(2);
block(page,'simulink/Signal Routing/Mux',[label ' vector'],[x+250 y x+255 y+50*n],'Inputs',num2str(n));
for j=1:n
 convert=sprintf('%s channel %d',label,j);sample=sprintf('%s sample %d',label,j);
 block(page,'simulink/Signal Attributes/Data Type Conversion',convert,[x y+50*(j-1) x+95 y+35+50*(j-1)],'OutDataTypeStr','double');
 block(page,'simulink/Discrete/Zero-Order Hold',sample,[x+135 y+50*(j-1) x+225 y+35+50*(j-1)],'SampleTime','p.sim.step');
 add_line(page,sprintf('%s/%d',source,ports(j)),[convert '/1'],'autorouting','on');
 add_line(page,[convert '/1'],[sample '/1'],'autorouting','on');
 add_line(page,[sample '/1'],sprintf('%s vector/%d',label,j),'autorouting','on');
end
block(page,'simulink/Sinks/To Workspace',[label ' log'],[x+335 y+90 x+530 y+145],'VariableName',variable,'SaveFormat','Timeseries');
add_line(page,[label ' vector/1'],[label ' log/1'],'autorouting','on');
end

function block(page,library,name,position,varargin)
add_block(library,[page '/' name],'Position',position,'FontSize','11',varargin{:});
end

function patch_prepare(path)
body=fileread(path);marker="if isfield(p,'accel') && p.accel.enabled";assert(contains(body,marker));
addition=strjoin([ ...
 "if isfield(p,'inertial')"; ...
 " if ~isfield(p.robot.forceGate,'variation'),p.robot.forceGate.variation=struct('enabled',false,'meanTime',.1,'varianceTime',.02,'rmsLimit_N',.08);end"; ...
 " c=p.robot.forceGate.variation;dt=p.force.sampleTime;assert(c.meanTime>0&&c.varianceTime>0&&c.rmsLimit_N>0);am=exp(-dt/c.meanTime);av=exp(-dt/c.varianceTime);"; ...
 " p.d.variationGate=struct('meanNumerator',[1-am,0],'meanDenominator',[1,-am],'varianceNumerator',[1-av,0],'varianceDenominator',[1,-av],'dwellSamples',1+ceil(p.robot.forceGate.dwellTime/dt));"; ...
 " if ~isfield(p.inertial,'accelerationPrediction_s'),p.inertial.accelerationPrediction_s=0;end"; ...
 " if ~isfield(p.inertial,'accelerationDerivativeFilter_s'),p.inertial.accelerationDerivativeFilter_s=.0003;end"; ...
 " assert(p.inertial.accelerationPrediction_s>=0&&p.inertial.accelerationDerivativeFilter_s>0);ad=exp(-dt/p.inertial.accelerationDerivativeFilter_s);"; ...
 " p.d.accelerationPrediction=struct('derivativeNumerator',(1-ad)/dt*[1,-1],'derivativeDenominator',[1,-ad]);"; ...
 "end";string(marker)],newline);
body=strrep(body,marker,addition);fid=fopen(path,'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',body);
end

function patch_runner(path)
body=fileread(path);marker="if any(strcmp(who(out),'robot_prediction_observer_log'))";assert(contains(body,marker));
addition=strjoin([ ...
 "if any(strcmp(who(out),'force_variation_log'))"; ...
 " logged=out.get('force_variation_log');r.forceVariation=struct('time',logged.Time(:),'data',rows(logged,5),'names',{{'meanError_N','rmsDeviation_N','stableSampleCount','stableNow','ready'}});"; ...
 " logged=out.get('acceleration_feedforward_log');r.accelerationFeedforward=struct('time',logged.Time(:),'data',rows(logged,2),'names',{{'measuredAz_m_s2','feedforwardAz_m_s2'}});"; ...
 "end";string(marker)],newline);
body=strrep(body,marker,addition);fid=fopen(path,'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',body);
end

function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
