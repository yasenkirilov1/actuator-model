function report=ml_force_20261005(root,phase)
%ML_FORCE_20261005 Bounded learned sensor-lag current residual, native plant.
% Private source only. Labels may use truth; inference never does.
assert(~usejava('desktop'),'Use isolated matlab -batch.');maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,phase='collect';end
study=fullfile(root,'acf_simulink','validation','ml_force_20261005');
source=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002');
if ~isfolder(study),mkdir(study);end
if strcmp(phase,'collect')
 assert(~isfolder(fullfile(study,'source')),'Frozen source already exists.');
 mkdir(fullfile(study,'source'));copyfile(fullfile(source,'helpers'),fullfile(study,'source','helpers'));
 copyfile(fullfile(source,'acf_force_control_adc_trial.slx'),fullfile(study,'source','acf_force_control_adc_trial.slx'));
 assert(strcmp(sha256(fullfile(study,'source','acf_force_control_adc_trial.slx')), ...
  'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
 copyfile([mfilename('fullpath') '.m'],fullfile(study,'source','ml_force_20261005.m'));
end
addpath(fullfile(study,'source','helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
worker=fullfile(study,['worker_' phase]);assert(~isfolder(worker),'Preserve previous attempts.');
for folder={'','cache','code','prefs'},mkdir(fullfile(worker,folder{1}));end
modelFile=fullfile(worker,'acf_force_control_adc_trial.slx');copyfile(fullfile(study,'source','acf_force_control_adc_trial.slx'),modelFile);
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'),'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_ml_force_sdi'),'w');fprintf(fid,'Owned ML force SDI.\n');fclose(fid);
mdl='acf_force_control_adc_trial';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
set_param(mdl,'SimulationMode','accelerator');add_compact_logger(mdl);add_residual(mdl);
ml=struct('W1',zeros(8,10),'b1',zeros(8,1),'W2',zeros(1,8),'b2',0, ...
 'mean',zeros(10,1),'scale',ones(10,1),'gain',0,'actionLimit_A',.05);
if ~strcmp(phase,'collect')
 learned=load(fullfile(study,'trained_weights.mat'));fields=fieldnames(ml);
 for k=1:numel(fields),if isfield(learned,fields{k}),ml.(fields{k})=learned.(fields{k});end,end
end
assignin(get_param(mdl,'ModelWorkspace'),'ml',ml);save_system(mdl,modelFile);
if strcmp(phase,'collect'),copyfile(modelFile,fullfile(study,'source','acf_force_control_ml_force.slx'));end
nominal=load(fullfile(root,'acf_simulink','validation','adc_budget_range_20261002_part1','input_configs.mat'),'configs');
base=nominal.configs(1).p;base.sim.stopTime=4;base.sim.evaluateFrom=0;
base.coil.forceConstant=24.4;base.coil.resistance=8.896;base.coil.inductance=.004;base.coil.voltage=32;
base.uncertainty.forceConstant_model_N_A=24.4;
base.inertial.lowerLoadInverse.identifiedForceConstant_N_A=24.4;
base.inertial.forceKp=base.inertial.forceKp*20/24.4;base.inertial.forceKi=base.inertial.forceKi*20/24.4;
base.inertial.forceKd=base.inertial.forceKd*20/24.4;base=acf_prepare(base);
% Trajectories, never adjacent-sample random splits, define each set.
train=[.1 5 0 0 101; .1 50 90 .3 103; .5 15 30 .3 107; .5 40 60 0 109; ...
 1 25 45 .3 113;1.4 10 75 0 127;2 5 90 .3 131;2 50 0 0 137];
select=[.1 25 45 .3 211;1 5 0 0 223;2 50 90 .3 227;1.4 35 60 .3 229];
test=[.1 5 90 .3 1009;.1 50 0 0 1013;.5 5 60 .3 1019;1 50 75 0 1021; ...
 1.4 20 15 .3 1031;2 5 0 0 1033;2 50 90 .3 1039;2 35 45 .3 1049];
configs=struct([]);
switch phase
 case 'collect'
  for k=1:size(train,1),add_case(make_nominal(base,train(k,:)),'train',k,0);end
  for k=1:size(select,1),add_case(make_nominal(base,select(k,:)),'selection',k,0);end
 case 'select'
  for gain=[.25 .5 1]
   for k=1:size(select,1),add_case(make_nominal(base,select(k,:)),'selection',k,gain);end
  end
 case 'test'
  selection=jsondecode(fileread(fullfile(study,'selected_gain.json')));gain=selection.selectedGain;
  for k=1:size(test,1)
   p=make_nominal(base,test(k,:));add_case(p,'holdout_nominal',k,0);add_case(p,'holdout_nominal',k,gain);
  end
  hot=load(fullfile(root,'acf_simulink','validation','budget_adc_hot_coil_20261002_inputs', ...
   'ADXL1002_8000_M08_30k_current100_hot32V_configs.mat'),'configs');
  for k=[1 3 5 7]
   p=hot.configs(k).p;p.sensor.seed=p.sensor.seed+7919;
   add_case(p,'holdout_calibrated_stress',k,0);add_case(p,'holdout_calibrated_stress',k,gain);
  end
 otherwise,error('Unknown phase.');
end
destination=fullfile(study,phase);mkdir(destination);save(fullfile(destination,'configs.mat'),'configs','-v7');
report=struct('status','running','phase',phase,'date','2026-10-05','sourceModelSha256', ...
 'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5', ...
 'privateModelSha256',sha256(modelFile),'recipeSha256',sha256([mfilename('fullpath') '.m']), ...
 'matlabVersion',version,'featureNames',{{'normal_deviation_N','normal_increment_N', ...
 'accel_ff_current_A','accel_ff_increment_A','head_gap_velocity_m_s','head_gap_velocity_increment_m_s', ...
 'estimated_slope','estimated_slope_increment','identified_tool_mass_kg','force_target_N'}}, ...
 'trainingTarget','clip(Kp*(N_est-N_true)-(B_id+S_id)/Kf_id*(aS_true-aS_ff_pred), -0.05, +0.05) A', ...
 'scope','Supervised sensor-lag compensation residual, not reward optimization. No plant truth or bump preview in inference. Finite closed-loop cases.', ...
 'architecture','10 inputs, 8 ReLU hidden units, scalar output, fixed normalization, feature clipping +/-6, current residual +/-0.05 A', ...
 'rows',struct([]));
rows=struct([]);started=tic;worst=-inf;
for j=1:numel(configs)
 c=configs(j);timer=tic;out=[];native=false;
 try
  p=acf_prepare(c.p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
  ml.gain=c.gain;assignin(get_param(mdl,'ModelWorkspace'),'ml',ml);
  set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17),'FastRestart','off');
  out=sim(mdl);native=true;log=out.get('ml_native_log');t=log.Time(:);a=tsrows(log,19);
  assert(t(1)==0&&all(isfinite(a(:)))&&all(abs(diff(t)-p.sim.step)<max(1e-12,p.sim.step*1e-6)));
  row=score(p,t,a,c);row.nativeReturned=true;row.executed=true;row.runtime_s=toc(timer);
  f=out.get('ml_features_log');ft=f.Time(:);fd=tsrows(f,15);
  assert(all(abs(diff(ft)-p.force.sampleTime)<1e-11));
  row.peakResidualCurrent_A=max(abs(fd(:,11)));row.featureFrames=numel(ft);
  row.peakAbsStroke_m=max(abs(a(:,2)));
  row.coefficientTargetRms_A=sqrt(mean(target(p,fd).^2));
  features=struct('time',ft,'data',fd,'p',p,'caseIdentity',c,'featureNames',{report.featureNames});
  save(fullfile(destination,sprintf('case_%03d_features.mat',j)),'features','-v7');
  trace=struct('time',t,'data',a,'parameters',p,'row',row);
  if row.peakNormalForceError_N>worst
   worst=row.peakNormalForceError_N;save(fullfile(destination,'worst_native.mat'),'trace','-v7.3');
  end
  if ~row.passed,save(fullfile(destination,sprintf('case_%03d_failed_native.mat',j)),'trace','-v7.3');end
  mask=(t>=1.95&t<=2.4)|t<=.05;ix=find(mask);stride=max(1,round(.00005/p.sim.step));ix=unique([ix(1:stride:end);find(t==row.peakTime_s)]);
  excerpt=struct('time',t(ix),'data',a(ix,:),'row',row,'p',p); %#ok<NASGU>
  save(fullfile(destination,sprintf('case_%03d_excerpt.mat',j)),'excerpt','-v7');
 catch failure
  row=struct('group',c.group,'id',c.id,'gain',c.gain,'executed',false,'nativeReturned',native, ...
   'passed',false,'reason',getReport(failure,'extended','hyperlinks','off'),'runtime_s',toc(timer));
  if native,save(fullfile(destination,sprintf('case_%03d_unvalidated.mat',j)),'out','c','-v7.3');end
 end
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 report.rows=rows;report.elapsed_s=toc(started);report.executed=sum([rows.executed]);report.passed=sum([rows.passed]);
 write_json(fullfile(destination,'results.json'),report);
 fprintf('ML_FORCE_PROGRESS %s %d/%d gain%.3g pass%d time%.1fs\n',phase,j,numel(configs),c.gain,row.passed,toc(started));
 clear out log t a f ft fd trace features excerpt;Simulink.sdi.clear;
 assert(row.executed,row.reason);
end
report.status='complete';write_json(fullfile(destination,'results.json'),report);
assert(strcmp(sha256(fullfile(source,'acf_force_control_adc_trial.slx')),report.sourceModelSha256));

 function add_case(p,group,id,gain)
  c=struct('p',p,'group',group,'id',id,'gain',gain);if isempty(configs),configs=c;else,configs(end+1)=c;end
 end
end

function p=make_nominal(base,x)
p=base;p.mechanics.mS=x(1);p.force.reference=x(2);p.mechanics.angle=x(3)*pi/180;p.contact.mu=x(4);p.sensor.seed=x(5);
p.sensor.distalMass_kg=x(1);p.sensor.distalMass_model_kg=x(1);p.sensor.distalMassCalibrationScale=1;
end
function u=target(p,d)
u=p.inertial.forceKp*(d(:,14)-d(:,12))-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)/p.uncertainty.forceConstant_model_N_A*(d(:,13)-d(:,15));
u=max(-.05,min(.05,u));
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
function add_compact_logger(mdl)
roots={'plant_log','normal_contact_log','surface_log'};indices={'[1 2 14 15 18 21 22 23 20 6 7 13 12 24]','[1 4 2 3]','3'};widths=[25 8 3];
add_block('simulink/Signal Routing/Mux',[mdl '/ML native mux'],'Inputs','3','Position',[3900 100 3905 245]);
for k=1:3
 blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace','VariableName',roots{k});assert(numel(blocks)==1);
 ph=get_param(blocks{1},'PortHandles');ln=get_param(ph.Inport(1),'Line');src=get_param(ln,'SrcPortHandle');
 name=[mdl '/ML native selector ' num2str(k)];add_block('simulink/Signal Routing/Selector',name,'InputPortWidth',num2str(widths(k)),'Indices',indices{k},'Position',[3550 100+90*k 3800 150+90*k]);
 sh=get_param(name,'PortHandles');add_line(mdl,src,sh.Inport(1),'autorouting','on');add_line(mdl,['ML native selector ' num2str(k) '/1'],['ML native mux/' num2str(k)],'autorouting','on');
end
add_block('simulink/Sinks/To Workspace',[mdl '/ML full native history'],'VariableName','ml_native_log','SaveFormat','Timeseries','MaxDataPoints','inf','Decimation','1','SampleTime','-1','Position',[4070 125 4330 195]);add_line(mdl,'ML native mux/1','ML full native history/1');
blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace');
for k=1:numel(blocks),if ~ismember(get_param(blocks{k},'VariableName'),{'ml_native_log','adc_acquisition_log'}),set_param(blocks{k},'Commented','on');end,end
end
function r=score(p,t,a,c)
e=a(:,15)-p.force.reference;[peak,ix]=max(abs(e));mask=a(:,5)>.5&a(:,6)<.5;
proj=max(abs(a(:,15)-(a(:,1)-a(:,18).*a(:,17))./sqrt(1+a(:,18).^2)));
rigid=max(abs(a(mask,1)-a(mask,12)-p.mechanics.mB*(p.d.g+a(mask,13))-p.mechanics.mS*(p.d.g+a(mask,19))));
lower=max(abs(a(mask,1)-a(mask,14)-p.mechanics.mS*(p.d.g+a(mask,19))));assert(max([proj rigid lower])<1e-8);
completed=t(end)>=p.sim.stopTime-p.sim.step/2;lost=any(a(:,5)<.5);stop=any(a(:,6)>.5);impact=any(abs(a(:,9))>1e-10);over=max(abs(a(:,3)))>p.coil.peakCurrent*(1+1e-4);
q=p.d.q10/p.mechanics.k1+a(:,11)-a(:,10);travel=max(abs(q));s=p.springs.lower;pack=p.springs;
minPreload=s.rate_N_m*(travel+pack.unseatingClearance_m);compression=min(s.freeLength_m-s.freeLengthTolerance_m-s.minWorkingLength_m-pack.workingClearance_m,s.forceEnvelope_N/s.rate_N_m);maxPreload=s.rate_N_m*(compression-travel);
packing=p.springs.commonPreload_N>=minPreload&&p.springs.commonPreload_N<=maxPreload&&max(q)-min(q)<=s.maxCyclicStroke_m+1e-12&&2000*(s.freeLength_m+s.freeLengthTolerance_m)<=100&&2000*(s.freeLength_m+s.freeLengthTolerance_m-p.springs.commonPreload_N/s.rate_N_m)<=100;
r=struct('group',c.group,'id',c.id,'gain',c.gain,'massS_kg',p.mechanics.mS,'reference_N',p.force.reference,'angle_deg',p.mechanics.angle*180/pi,'mu',p.contact.mu,'seed',p.sensor.seed, ...
 'passed',completed&&peak<=.5&&~lost&&~stop&&~impact&&~over&&packing,'peakNormalForceError_N',peak,'peakTime_s',t(ix),'rmsNormalForceError_N',sqrt(mean(e.^2)), ...
 'peakStartupError_N',max(abs(e(t<=.25))),'peakCurrent_A',max(abs(a(:,3))),'peakVoltage_V',max(abs(a(:,4))),'voltageClipFraction',mean(a(:,8)>.5),'nativeSamples',numel(t),'endTime_s',t(end), ...
 'completed',completed,'lostContact',lost,'strokeStop',stop,'impact',impact,'overCurrent',over,'springPackingPassed',packing,'projectionBalanceResidual_N',proj,'rigidBalanceResidual_N',rigid,'lowerBalanceResidual_N',lower,'reason','Full native recording scored, including startup');
end
function a=tsrows(ts,n)
a=ts.Data;t=numel(ts.Time);if size(a,1)~=t,a=permute(a,[ndims(a),1:ndims(a)-1]);end;a=reshape(a,t,n);
end
function finish(mdl,storage,record,sdi)
try,set_param(mdl,'FastRestart','off');close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_ml_force_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function write_json(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true)); %#ok<NASGU>
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');d.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
