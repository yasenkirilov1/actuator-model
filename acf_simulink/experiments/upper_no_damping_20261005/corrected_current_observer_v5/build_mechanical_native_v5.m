function metadata=build_mechanical_native_v5(folder,configsFile,stage)
%BUILD_NATIVE Export the frozen native plant with external causal actions.
% Private experiment only. The C bridge supplies actions, not plant physics.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
if nargin<1,folder=fullfile(root,'acf_simulink','validation','rl_budget3h_20261005','native_bank');end
if nargin<2,configsFile=fullfile(root,'acf_simulink','validation','rl_budget3h_20261005','scenario_bank','configs.mat');end
if nargin<3,stage='full';end
if strcmp(stage,'parameter_bank_only')
 source=fullfile(folder,'helpers');addpath(source,'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
 mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(fullfile(folder,[mdl '.slx']));cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
 loaded=load(configsFile,'configs');export_parameter_bank(mdl,loaded.configs,folder);
 metadata=jsondecode(fileread(fullfile(folder,'metadata.json')));metadata.status='native_c_and_parameter_bank';
 write_json(fullfile(folder,'metadata.json'),metadata);return;
end
assert(~isfolder(folder),'Preserve prior native export.');mkdir(folder);mkdir(fullfile(folder,'cache'));mkdir(fullfile(folder,'code'));
source=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002');
modelFile=fullfile(folder,'acf_rl_native.slx');copyfile(fullfile(source,'acf_force_control_adc_trial.slx'),modelFile);
copyfile(fullfile(fileparts(configsFile),'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'),'CodeGenFolder',fullfile(folder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_rl_native_sdi'),'w');fprintf(fid,'Owned unified-RL native export.\n');fclose(fid);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(false);
loaded=load(configsFile,'configs');p=acf_prepare(loaded.configs(1).p);
assert(abs(p.sim.step-1/300000)<1e-15&&abs(p.force.sampleTime-1e-4)<1e-15);
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
instrument(mdl);
upperContract=install_upper_observer_v5(mdl,p);
upperContract.liveIssuedCommandGuard=verify_issued_command_source(mdl,p,folder);
assert_issued_command_chain(mdl);
fixed_bank_delay(mdl,[mdl '/Robot command delay/Robot command samples'],1000);
fixed_bank_delay(mdl,[mdl '/Issued-command robot predictor/Expected command delay'],12);
blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace');
for k=1:numel(blocks),set_param(blocks{k},'Commented','on');end
blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','Scope');
for k=1:numel(blocks),set_param(blocks{k},'Commented','on');end
set_param(mdl,'SystemTargetFile','grt.tlc','TargetLang','C','DefaultParameterBehavior','Tunable', ...
 'CodeInterfacePackaging','Nonreusable function','GRTInterface','off','MatFileLogging','off', ...
 'GenerateReport','off','GenCodeOnly','on','SignalLogging','off','SaveTime','on','SaveOutput','on', ...
 'SaveFormat','Dataset','FixedStep',num2str(p.sim.step,17),'Solver','ode4','SimulationMode','normal','StopTime','inf');
save_system(mdl,modelFile);
metadata=struct('status','building','sourceModelSha256',sha256(fullfile(source,'acf_force_control_adc_trial.slx')), ...
 'upperObserverContract',upperContract,'mechanicalDesign',p.mechanicalStudy.design, ...
 'modelName',mdl,'modelFile',modelFile,'step_s',p.sim.step,'actionStep_s',p.force.sampleTime,'physicalTicksPerAction',30, ...
 'robotStep_s',p.robot.sampleTime,'matlabVersion',version,'installedProducts',ver,'configsFile',configsFile,'configsSha256',sha256(configsFile), ...
 'observationNames',{{'estimated_normal_N','inertial_axial_N','inertial_lateral_N', ...
 'measured_acceleration_z_m_s2','measured_acceleration_x_m_s2','relative_position_m', ...
 'estimated_relative_velocity_m_s','estimated_actuator_travel_m','estimated_slope', ...
 'issued_command_robot_velocity_m_s','issued_command_robot_acceleration_m_s2', ...
 'aligned_issued_command_robot_velocity_m_s','aligned_issued_command_robot_acceleration_m_s2', ...
 'issued_current_request_A','issued_robot_velocity_request_m_s','issued_robot_correction_m_s', ...
 'force_gate_ready','recenter_error_m'}}, ...
 'scoringNames',{{'true_normal_N','plant_log_25','true_surface_slope'}}, ...
 'comparatorNames',{{'baseline_current_request_A','baseline_robot_correction_m_s','known_loaded_equilibrium_current_A'}}, ...
 'causalContract','Observations use measured/estimated signals and actual already-issued commands only. Truth and disconnected baseline actions are separate scoring/comparator outputs. Actor current replaces all dynamic PI/FF actuation before current notch/limit; actor robot correction is 100Hz held and passes original gate and teacher sign/amplitude envelope.');
write_json(fullfile(folder,'metadata.json'),metadata);save(fullfile(folder,'prepared_configs.mat'),'p','loaded','-v7');
try
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);metadata.algebraicLoopCount=loops;
 timer=tic;slbuild(mdl);metadata.buildElapsed_s=toc(timer);metadata.status='generated_native_c';
catch failure
 metadata.status='codegen_rejected';metadata.error=getReport(failure,'extended','hyperlinks','off');
 write_json(fullfile(folder,'metadata.json'),metadata);rethrow(failure);
end
metadata.emittedCurrentInputContract=verify_emitted_current_input(folder);
metadata.privateModelSha256=sha256(modelFile);write_json(fullfile(folder,'metadata.json'),metadata);
export_parameter_bank(mdl,loaded.configs,folder);
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'build_native.m'));
fprintf('RL_NATIVE_EXPORT_GENERATED %.3fs\n',metadata.buildElapsed_s);
end

function fixed_bank_delay(mdl,path,maximum)
% Native circular Delay implements the identical reset-fixed queue length.
% Capacity is shared by all bank cases; delay remains an exact prepared value.
sys=get_param(path,'Parent');name=get_param(path,'Name');expression=get_param(path,'DelayLength');
set_param(path,'DelayLengthSource','Input port','DelayLengthUpperLimit',num2str(maximum), ...
 'UseCircularBuffer','on','PreventDirectFeedthrough','on','DiagnosticForDelayLength','Error');
add_block('simulink/Sources/Constant',[sys '/RL reset-fixed queue length'],'Value',expression,'Position',[20 -150 240 -120]);
add_line(sys,'RL reset-fixed queue length/1',[name '/2'],'autorouting','on');
assert(evalin(get_param(mdl,'ModelWorkspace'),expression)<=maximum);
end

function export_parameter_bank(mdl,configs,folder)
% Parse Coder's parameter declarations, then evaluate the actual native block
% expressions. Computed sparse state-space coefficients retain Coder's exact
% sparse column-major and discrete row-major layouts. No physical equation is
% reimplemented here; exact emitted case0 bytes are checked by the C bridge.
header=fileread(fullfile(folder,'code',[mdl '_grt_rtw'],[mdl '.h']));
body=regexp(header,'struct P_acf_rl_native_T_ \{(.*?)\n\};','tokens','once');assert(~isempty(body));
fields=regexp(body{1},'(\w+)\s+(\w+[^;]*);\s*/\*(.*?)\*/','tokens');assert(numel(fields)>500);
map=struct([]);counters=containers.Map;
for j=1:numel(fields)
 f=fields{j};size=regexp(f{2},'\[(\d+)\]','tokens','once');count=1;if ~isempty(size),count=str2double(size{1});end
 f{2}=regexp(f{2},'^\w+','match','once');
 ref=regexp(f{3},'Referenced by: ''([^'']+)''','tokens','once');assert(~isempty(ref));
 expr=regexp(f{3},'Expression:\s*([^\n]+)','tokens','once');
 if isempty(expr)
  path=ref{1};token=regexp(path,'^<S(\d+)>/(.*)$','tokens','once');assert(~isempty(token));
  sid=str2double(token{1});sub=regexp(header,sprintf("'\\<S%d\\>'\\s*:\\s*'([^']+)'",sid),'tokens','once');
  if isempty(sub),sub=regexp(header,sprintf("'<S%d>'\\s*:\\s*'([^']+)'",sid),'tokens','once');end
  assert(~isempty(sub));block=[sub{1} '/' token{2}];type=get_param(block,'BlockType');
  if any(strcmp(type,{'StateSpace','DiscreteStateSpace'}))
   if strcmp(type,'StateSpace')
    matrix=regexp(f{2},'_([ABCD])_(pr|p_|ir|i_|jc|j_)','tokens','once');assert(~isempty(matrix));storage=matrix{2};matrix=matrix{1};
    if startsWith(f{1},'uint')
     if any(strcmp(storage,{'ir','i_'})),kind='sparse_row';else,kind='sparse_col';end
    else,kind='sparse_value';end
   else
    if ~isKey(counters,block),counters(block)=0;end;counters(block)=counters(block)+1;
    abcd='ABCD';matrix=abcd(counters(block));kind='nonzero_value';
   end
   expression=get_param(block,matrix);
  elseif strcmp(type,'RandomNumber'),kind='sqrt';expression=get_param(block,'Variance');
  elseif strcmp(type,'DiscreteIntegrator'),kind='plain';expression=get_param(block,'SampleTime');
  else,error('Unsupported native computed coefficient %s type%s.',f{2},type);end
 else,kind='plain';expression=strtrim(expr{1});block='';end
 row=struct('type',f{1},'name',f{2},'count',count,'kind',kind,'expression',expression,'block',block);
 if isempty(map),map=row;else,map(end+1)=row;end
end
fid=fopen(fullfile(folder,'native_parameter_bank.c'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'#include "acf_rl_native.h"\nconst P_acf_rl_native_T acf_parameter_bank[%d] = {\n',numel(configs));
values=cell(numel(configs),numel(map));
for k=1:numel(configs)
 p=acf_prepare(configs(k).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);fprintf(fid,' {\n');
 for j=1:numel(map)
  m=map(j);v=evalin(get_param(mdl,'ModelWorkspace'),m.expression);
  switch m.kind
   case 'sqrt',v=sqrt(v);
   case 'sparse_value',v=v(v~=0);
   case 'nonzero_value',v=v.';v=v(v~=0);
   case 'sparse_row',[row,~,~]=find(v);v=row-1;
   case 'sparse_col',v=[0,cumsum(sum(v~=0,1))];
  end
  assert(isreal(v)&&numel(v)==m.count,'Native coefficient shape mismatch for case%d %s.',k,m.name);values{k,j}=v(:);
  fprintf(fid,'  .%s = ',m.name);
  if m.count>1,fprintf(fid,'{');end
  for n=1:numel(v)
   if n>1,fprintf(fid,',');end
   if isinf(v(n)),if v(n)<0,fprintf(fid,'-');end;fprintf(fid,'INFINITY');
   elseif isnan(v(n)),fprintf(fid,'NAN');else,fprintf(fid,'%.17g',v(n));end
  end
  if m.count>1,fprintf(fid,'}');end;fprintf(fid,',\n');
 end
 fprintf(fid,' },\n');
end
fprintf(fid,'};\nconst int acf_parameter_bank_count=%d;\n',numel(configs));
save(fullfile(folder,'parameter_mapping.mat'),'map','values','-v7');write_json(fullfile(folder,'parameter_mapping.json'),map);
scenarios=struct([]);
for k=1:numel(configs)
 p=acf_prepare(configs(k).p);
 row=struct('scenario',k-1,'identity',configs(k).identity,'identifiedMassB_kg',p.uncertainty.massB_model_kg, ...
  'knownEquilibriumCurrent_A',(p.force.reference-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A, ...
  'identifiedRobotDelay_s',p.robotPrediction.delay,'identifiedRobotResponseTime_s',p.robotPrediction.responseTime, ...
  'currentResponse_s',p.coil.currentResponse,'identifiedAntiAliasCutoff_Hz',p.adcStudy.identifiedAnalogCutoff_Hz*p.acquisition.identifiedAntiAliasCutoff_Hz/5100, ...
  'identifiedEmaTime_s',p.adcStudy.identifiedEmaTime_s*p.acquisition.identifiedEmaTime_s/50e-6, ...
  'identifiedPipelineDelay_s',p.adcStudy.d.channels.frame.pipelineDelay_s,'actualRobotDelaySamples',p.d.robotDelaySamples, ...
  'identifiedRobotDelaySamples',p.robotPrediction.delaySamples,'seed',p.sensor.seed);
 if isempty(scenarios),scenarios=row;else,scenarios(end+1)=row;end
end
write_json(fullfile(folder,'scenarios.json'),scenarios);
assignin(get_param(mdl,'ModelWorkspace'),'p',acf_prepare(configs(1).p));
end

function instrument(mdl)
names={'rl_current_request_A','rl_robot_correction_m_s','rl_baseline_mode'};
tags={'rl_external_current','rl_external_robot','rl_baseline_mode'};
for k=1:3
 add_block('simulink/Ports & Subsystems/In1',[mdl '/' names{k}],'Port',num2str(k),'SampleTime','p.force.sampleTime','Position',[3100 50+70*k 3160 80+70*k]);
 add_block('simulink/Signal Routing/Goto',[mdl '/RL input tag ' num2str(k)],'GotoTag',tags{k},'TagVisibility','global','Position',[3240 50+70*k 3500 80+70*k]);
 add_line(mdl,[names{k} '/1'],['RL input tag ' num2str(k) '/1']);
end
sys=[mdl '/Force controller/Coil PI and PID'];
tap(sys,'PI PID and feedforward current','RL baseline current tap','rl_baseline_current');
tap(sys,'Flat holding current','RL equilibrium current tap','rl_equilibrium_current');
tap(sys,'Current limits','RL actually issued current tap','rl_issued_current');
target=[sys '/Optional structural-mode current notch'];ph=get_param(target,'PortHandles');delete_line(get_param(ph.Inport(1),'Line'));
add_block('simulink/Signal Routing/From',[sys '/RL full current request'],'GotoTag','rl_external_current','Position',[1640 550 1840 580]);
add_block('simulink/Signal Routing/From',[sys '/RL baseline mode'],'GotoTag','rl_baseline_mode','Position',[1640 615 1840 645]);
add_block('simulink/Signal Routing/Switch',[sys '/Baseline or external current'],'Threshold','.5','Position',[1900 535 1950 665]);
add_line(sys,'PI PID and feedforward current/1','Baseline or external current/1');
add_line(sys,'RL baseline mode/1','Baseline or external current/2');
add_line(sys,'RL full current request/1','Baseline or external current/3');
add_line(sys,'Baseline or external current/1','Optional structural-mode current notch/1','autorouting','on');
% All existing actuator_command consumers still receive post-limit requests.
sys=[mdl '/Recenter travel'];
tap(sys,'Recenter gain','RL baseline robot tap','rl_baseline_robot');
tap(sys,'Distance from mid stroke','RL recenter error tap','rl_recenter_error');
tap(sys,'Send correction only when settled','RL issued correction tap','rl_issued_robot_correction');
tap(sys,'Force settling gate','RL force gate tap','rl_force_gate');
tap(sys,'Robot speed limits','RL actually issued total robot tap','rl_issued_robot_total');
ph=get_param([sys '/Send correction only when settled'],'PortHandles');delete_line(get_param(ph.Inport(1),'Line'));
add_block('simulink/Signal Routing/From',[sys '/RL robot correction request'],'GotoTag','rl_external_robot','Position',[850 -240 1070 -205]);
add_block('simulink/Discrete/Zero-Order Hold',[sys '/RL fresh 100 Hz robot request'],'SampleTime','p.robot.sampleTime','Position',[1120 -240 1290 -205]);
add_block('simulink/Math Operations/MinMax',[sys '/RL positive recenter envelope'],'Function','max','Inputs','2','Position',[1100 -120 1270 -70]);
add_block('simulink/Math Operations/MinMax',[sys '/RL negative recenter envelope'],'Function','min','Inputs','2','Position',[1100 -10 1270 40]);
add_block('simulink/Discontinuities/Saturation Dynamic',[sys '/RL hard sign and gain envelope'],'Position',[1350 -220 1530 -80]);
add_block('simulink/Signal Routing/From',[sys '/RL baseline mode'],'GotoTag','rl_baseline_mode','Position',[1510 -300 1710 -265]);
add_block('simulink/Signal Routing/Switch',[sys '/Baseline or external robot'],'Threshold','.5','Position',[1800 -240 1850 -120]);
add_line(sys,'RL robot correction request/1','RL fresh 100 Hz robot request/1');
add_line(sys,'RL fresh 100 Hz robot request/1','RL hard sign and gain envelope/2');
add_line(sys,'Recenter gain/1','RL positive recenter envelope/1');add_line(sys,'Hold recentering/1','RL positive recenter envelope/2');
add_line(sys,'Recenter gain/1','RL negative recenter envelope/1');add_line(sys,'Hold recentering/1','RL negative recenter envelope/2');
add_line(sys,'RL positive recenter envelope/1','RL hard sign and gain envelope/1');add_line(sys,'RL negative recenter envelope/1','RL hard sign and gain envelope/3');
add_line(sys,'Recenter gain/1','Baseline or external robot/1');add_line(sys,'RL baseline mode/1','Baseline or external robot/2');
add_line(sys,'RL hard sign and gain envelope/1','Baseline or external robot/3');
add_line(sys,'Baseline or external robot/1','Send correction only when settled/1','autorouting','on');
% Match the position observation to the acquisition frame used by the observer.
tap([mdl '/Contact force measurement/Head motion estimator'],'Fresh 10 kHz position frame','RL aligned position tap','rl_aligned_relative_position');
tags={'estimated_normal_contact_N','inertial_axial_contact_N','inertial_lateral_contact_N', ...
 'measured_S_acceleration_z','measured_S_acceleration_x','rl_aligned_relative_position', ...
 'inertial_head_gap_velocity','estimated_actuator_travel_m','estimated_contact_slope', ...
 'command_based_robot_velocity','command_based_robot_acceleration', ...
 'inertial_robot_velocity_aligned','inertial_robot_acceleration_aligned', ...
 'rl_issued_current','rl_issued_robot_total','rl_issued_robot_correction','rl_force_gate','rl_recenter_error'};
export(mdl,'rl_observation',tags,1,true);
export(mdl,'rl_scoring',{'normal_contact_N','plant_log','actual_contact_slope'},2,false);
export(mdl,'rl_comparator',{'rl_baseline_current','rl_baseline_robot','rl_equilibrium_current'},3,false);
end
function tap(sys,source,name,tag)
add_block('simulink/Signal Routing/Goto',[sys '/' name],'GotoTag',tag,'TagVisibility','global','Position',[2700 600 2920 630]);
ph=get_param([sys '/' source],'PortHandles');gh=get_param([sys '/' name],'PortHandles');add_line(sys,ph.Outport(1),gh.Inport(1),'autorouting','on');
end
function export(mdl,name,tags,port,frame)
sys=[mdl '/RL ' name];add_block('simulink/Ports & Subsystems/Subsystem',sys,'Position',[3700 50+150*port 4050 150+150*port]);Simulink.SubSystem.deleteContents(sys);
add_block('simulink/Signal Routing/Mux',[sys '/Vector'],'Inputs',num2str(numel(tags)),'Position',[730 20 735 90*numel(tags)]);
for k=1:numel(tags)
 n=['Source ' num2str(k)];add_block('simulink/Signal Routing/From',[sys '/' n],'GotoTag',tags{k},'Position',[20 90*k 280 90*k+25]);
 c=['Double ' num2str(k)];add_block('simulink/Signal Attributes/Data Type Conversion',[sys '/' c],'OutDataTypeStr','double','Position',[320 90*k 410 90*k+25]);add_line(sys,[n '/1'],[c '/1']);
 if frame
  f=['Fresh observation ' num2str(k)];add_block('simulink/Discrete/Zero-Order Hold',[sys '/' f],'SampleTime','p.force.sampleTime','Position',[480 90*k 650 90*k+25]);add_line(sys,[c '/1'],[f '/1']);c=f;
 end
 add_line(sys,[c '/1'],['Vector/' num2str(k)]);
end
add_block('simulink/Ports & Subsystems/Out1',[sys '/output'],'Position',[810 90 870 115]);add_line(sys,'Vector/1','output/1');
add_block('simulink/Ports & Subsystems/Out1',[mdl '/' name],'Port',num2str(port),'Position',[4200 100+150*port 4270 130+150*port]);add_line(mdl,['RL ' name '/1'],[name '/1']);
end
function write_json(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true)); %#ok<NASGU>
end
function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_rl_native_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');d.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
