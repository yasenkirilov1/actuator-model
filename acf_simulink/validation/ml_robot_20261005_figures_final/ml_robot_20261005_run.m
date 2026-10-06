function report=ml_robot_20261005_run(root,phase,outputFolder,weightsFile)
%ML_ROBOT_20261005_RUN Private causal robot-recentering imitation experiment.
% The physical V8/ADC plant and deterministic fast force policy are frozen.
% Inference consists only of Gain/Bias/ReLU/Sum blocks, followed by hard
% directional, delay-gain and speed constraints and the original force gate.
assert(~usejava('desktop'),'Run in an isolated matlab -batch process.');
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,phase='pilot';end
if nargin<3,outputFolder=fullfile(root,'acf_simulink','validation',['ml_robot_20261005_' phase]);end
if nargin<4,weightsFile='';end
assert(~isfile(fullfile(outputFolder,'results.json')),'Preserve previous experiments.');
if ~isfolder(outputFolder),mkdir(outputFolder);end
maxNumCompThreads(2);project=fullfile(root,'acf_simulink');
source=fullfile(project,'validation','adc_acquisition_trial_20261002');
sourceModel=fullfile(source,'acf_force_control_adc_trial.slx');
sourceSha=sha256(sourceModel);
assert(strcmp(sourceSha,'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
worker=fullfile(outputFolder,'private_worker');mkdir(worker);
for name={'helpers','cache','code'},mkdir(fullfile(worker,name{1}));end
copyfile(fullfile(source,'helpers','*.m'),fullfile(worker,'helpers'));
copyfile(sourceModel,fullfile(worker,'acf_force_control_adc_trial.slx'));
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'ml_robot_20261005_run.m'));
copyfile(fullfile(fileparts(mfilename('fullpath')),'ml_robot_20261005_build_policy.m'),fullfile(outputFolder,'ml_robot_20261005_build_policy.m'));
addpath(outputFolder,'-begin');
addpath(fullfile(worker,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_ml_robot_20261005'),'w');fclose(fid);
mdl='acf_force_control_adc_trial';assert(~bdIsLoaded(mdl));load_system(fullfile(worker,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
set_param(mdl,'SimulationMode','accelerator','FastRestart','off');
ml_robot_20261005_build_policy(mdl);add_compact_logger(mdl);set_historical_logging(mdl,false);
base=load(fullfile(source,'baseline_candidate.mat'),'p');base=base.p;
hotFile=fullfile(project,'validation','budget_adc_hot_coil_20261002_inputs', ...
 'ADXL1002_8000_M08_30k_current100_hot32V_configs.mat');hot=load(hotFile,'configs');
% Use exactly the retained acquisition/electrical specifications, but zero
% static identification residuals for the nominal training trajectories.
base.adcStudy=hot.configs(1).p.adcStudy;base.sim.step=base.adcStudy.fineClock_s;
base.coil=hot.configs(1).p.coil;base.inertial=hot.configs(1).p.inertial;
base.uncertainty=struct('forceScaleZ',0,'forceScaleX',0,'positionScale',0, ...
 'forceFrameAngle_rad',0,'accelFrameAngle_rad',0,'forceCrossZFromX',0,'forceCrossXFromZ',0, ...
 'accelBiasX_m_s2',0,'massB_model_kg',.6,'forceConstant_model_N_A',24.4, ...
 'recenterGain',min(base.robot.recenterGain,base.robot.forceGate.maxDelayGainProduct/ ...
 (base.robotPrediction.delay+base.robotPrediction.responseTime+base.robot.forceGate.dwellTime+base.robot.forceGate.filterTime)));
base.robot.initialExcessForce=0;base.robot.nominalVelocity=0;base.robot.initialPositionOffset_m=0;
network=struct('enabled',0,'inputScale_m',.002,'W1',[1;1;1;1;-1;-1;-1;-1], ...
 'b1',[-.25;-.5;-.75;-1;-.25;-.5;-.75;-1],'W2',[1 0 0 0 -1 0 0 0],'b2',0,'zeroOutput',0);
if ~isempty(weightsFile)
 learned=load(weightsFile,'network');network=learned.network;
 copyfile(weightsFile,fullfile(outputFolder,'network_weights.mat'));
end
configs=make_configs(base,hot.configs,phase,network);
save(fullfile(outputFolder,'input_configs.mat'),'configs','-v7');
assignin(get_param(mdl,'ModelWorkspace'),'p',acf_prepare(configs(1).p));
save_system(mdl,fullfile(worker,[mdl '.slx']));
helpers=struct([]);files=dir(fullfile(worker,'helpers','*.m'));
for j=1:numel(files),helpers=append(helpers,struct('file',files(j).name,'sha256',sha256(fullfile(files(j).folder,files(j).name))));end
report=struct('status','running','date','2026-10-05','phase',phase,'sourceModelSha256',sourceSha, ...
 'privateModelSha256',sha256(fullfile(worker,[mdl '.slx'])),'sourceHotParametersSha256',sha256(hotFile), ...
 'recipeSha256',sha256(fullfile(outputFolder,'ml_robot_20261005_run.m')), ...
 'policyBuilderSha256',sha256(fullfile(outputFolder,'ml_robot_20261005_build_policy.m')), ...
 'helpers',helpers,'matlabVersion',version,'plannedRecordings',numel(configs), ...
 'scope','Finite native closed-loop causal robot-policy imitation. No physical sanding or full residual-box qualification.', ...
 'physicalColumns',{{'Rz_N','travel_m','I_A','V_V','contact','stop','effortClip','voltageClip','impactImpulse_Ns', ...
 'xB_m','xS_m','Fa_N','aB_m_s2','lowerForce_N','N_N','Nestimate_N','Rx_N','slope','aS_m_s2', ...
 'robotPosition_m','robotVelocity_m_s','delayedRobotRequest_m_s','issuedRobotRequest_m_s'}}, ...
 'policyColumns',{{'strokeError_m','teacherCorrection_m_s','boundedNetworkCorrection_m_s','selectedCorrection_m_s', ...
 'forceGateReady','issuedCorrection_m_s','causalEstimatedTravel_m','rawNetworkDistance_m'}}, ...
 'rows',struct([]));
rows=struct([]);started=tic;write_report('running');
for j=1:numel(configs)
 c=configs(j);timer=tic;out=[];nativeReturned=false;
 row=struct('recording',j,'identity',c.identity,'split',c.split,'policy',c.policy, ...
  'massS_kg',c.p.mechanics.mS,'force_N',c.p.force.reference,'angle_deg',c.p.mechanics.angle*180/pi, ...
  'seed',c.p.sensor.seed,'offset_m',c.p.robot.initialPositionOffset_m,'actualRobotDelay_s',c.p.robot.delay, ...
  'actualRobotResponse_s',c.p.robot.responseTime,'executed',false,'passed',false,'reason','');
 try
  q=acf_prepare(c.p);
  assignin(get_param(mdl,'ModelWorkspace'),'p',q);
  set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
  out=sim(mdl);nativeReturned=true;
  log=out.get('ml_robot_native_log');t=log.Time(:);a=tsrows(log,23);
  policyLog=out.get('ml_robot_policy_log');tp=policyLog.Time(:);b=tsrows(policyLog,8);
  assert(t(1)==0&&size(a,1)==numel(t)&&all(isfinite(a(:))));
  assert(all(abs(diff(t)-q.sim.step)<max(1e-12,q.sim.step*1e-6)),'A native physical sample was omitted.');
  assert(numel(t)==round(t(end)/q.sim.step)+1);
  row=score(q,t,a,tp,b,row);
  row.executed=true;row.nativeReturned=true;row.runtime_s=toc(timer);
  trace=struct('time',t,'data',a,'policyTime',tp,'policyData',b,'parameters',q, ...
   'columns',{report.physicalColumns},'policyColumns',{report.policyColumns},'row',row);
  % Score complete native histories; retain causal training features and
  % a display excerpt for every recording. Keep all failing native histories.
  step=max(1,round(100e-6/q.sim.step));idx=unique([(1:step:numel(t))';find(t==row.peakTime_s)]);
  compact=trace;compact.time=t(idx);compact.data=a(idx,:);
  save(fullfile(outputFolder,sprintf('recording_%03d_compact.mat',j)),'compact','-v7');
  if ~row.passed || strcmp(phase,'pilot') || j==1 || j==numel(configs)
   save(fullfile(outputFolder,sprintf('recording_%03d_native.mat',j)),'trace','-v7.3');
  end
  save(fullfile(outputFolder,sprintf('recording_%03d_parameters.mat',j)),'q','-v7');
 catch failure
  row.executed=false;row.nativeReturned=nativeReturned;row.reason=getReport(failure,'extended','hyperlinks','off');
  row.runtime_s=toc(timer);
  if nativeReturned,save(fullfile(outputFolder,sprintf('recording_%03d_unvalidated.mat',j)),'out','c','-v7.3');end
 end
 rows=append(rows,row);write_report('running');
 fid=fopen(fullfile(outputFolder,'rows.jsonl'),'a');fprintf(fid,'%s\n',jsonencode(row));fclose(fid);
 fprintf('ML_ROBOT_NATIVE %s %d/%d executed%d pass%d',phase,j,numel(configs),row.executed,row.passed);
 if isfield(row,'peakNormalForceError_N'),fprintf(' peak%.9gN command%.6gmm/s gateClosedCommands%d runtime%.1fs',row.peakNormalForceError_N,row.peakIssuedCorrection_m_s*1000,row.gateClosedCommandViolations,row.runtime_s);end
 fprintf('\n');
 clear out log policyLog a b t tp trace compact q;Simulink.sdi.clear;
 if ~row.executed,write_report('software_rejection');error('acf:mlrobot:Contract','%s',row.reason);end
end
assert(strcmp(sourceSha,sha256(sourceModel)));
write_report('complete');
 function write_report(status)
  report.status=status;report.rows=rows;report.recordingsReturned=numel(rows);
  if isempty(rows),report.executed=0;report.passed=0;report.nativeSamples=0;
  else,report.executed=sum([rows.executed]);report.passed=sum([rows.passed]);
   if all([rows.executed]),report.nativeSamples=sum([rows.nativeSamples]);report.worstNormalForceError_N=max([rows.peakNormalForceError_N]);end
  end
  report.elapsed_s=toc(started);fid=fopen(fullfile(outputFolder,'results.json'),'w');
  fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
 end
end

function configs=make_configs(base,hot,phase,network)
configs=struct([]);
switch phase
 case 'pilot'
  add(base,.1,5,0,29,-.0015,'pilot_negative','pilot','teacher');
  add(base,2,50,90,7,.0015,'pilot_positive','pilot','teacher');
 case 'collect'
  rows=[.1 5 0 7 -1.5; .1 50 90 29 1.5; 1.4 25 45 7 -1; 1.4 50 0 29 1; ...
   2 5 90 7 -2; 2 50 0 29 2; .1 25 45 7 -.75; 1.4 5 90 29 .75];
  for k=1:size(rows,1),r=rows(k,:);add(base,r(1),r(2),r(3),r(4),r(5)*1e-3,sprintf('train_%02d',k),'train','teacher');end
  rows=[.1 15 30 1729 -1.25; 1.4 35 60 1729 1.25; 2 45 30 1729 -1.75; 2 10 60 1729 1.75];
  for k=1:size(rows,1),r=rows(k,:);add(base,r(1),r(2),r(3),r(4),r(5)*1e-3,sprintf('selection_%02d',k),'selection','teacher');end
 case 'heldout'
  rows=[.5 10 15 104729 -1.8;.5 40 75 104729 1.8;1 20 30 314159 -1.2;1 45 60 314159 1.2; ...
   1.8 30 15 271828 -1.6;1.8 15 75 271828 1.6; .1 5 0 104729 -.9;2 50 90 104729 .9];
  for k=1:size(rows,1)
   r=rows(k,:);q=base;
   if mod(k,2)==1,q.robot.delay=.085;q.robot.responseTime=.055;else,q.robot.delay=.075;q.robot.responseTime=.065;end
   for mode={'teacher','learned'},add(q,r(1),r(2),r(3),r(4),r(5)*1e-3,sprintf('heldout_%02d',k),'test',mode{1});end
  end
  for k=[1 4 5]
   q=hot(k).p;q.sim.stopTime=8;
   for mode={'teacher','learned'},add(q,q.mechanics.mS,q.force.reference,q.mechanics.angle*180/pi,q.sensor.seed,q.robot.initialPositionOffset_m,sprintf('calibration_%02d',k),'stress_test',mode{1},true);end
  end
 otherwise,error('Unknown phase.');
end
 function add(q,m,f,angle,seed,offset,identity,split,mode,preserve)
  if nargin<10,preserve=false;end
  if ~preserve
   q.mechanics.mS=m;q.force.reference=f;q.mechanics.angle=angle*pi/180;
   q.sensor.distalMassCalibrationScale=1;q.sensor.seed=seed;
   q.contact.mu=.3;q.sensor.bias=0;q.contact.lateralBias_N=0;q.accel.bias=0;q.accel.scaleError=0;
   q.robot.initialPositionOffset_m=offset;q.robot.initialExcessForce=0;
   q.sim.stopTime=6;q.sim.evaluateFrom=0;
  end
  q.mlRobot=network;q.mlRobot.enabled=double(strcmp(mode,'learned'));
  configs=append(configs,struct('p',q,'identity',identity,'split',split,'policy',mode));
 end
end

function add_compact_logger(mdl)
roots={'plant_log','normal_contact_log','surface_log','robot_log'};
indices={'[1 2 14 15 18 21 22 23 20 6 7 13 12 24]','[1 4 2 3]','3','[1 2]'};widths=[25 8 3 2];
add_block('simulink/Signal Routing/Mux',[mdl '/ML full native vector'],'Inputs','6','Position',[3900 100 3905 425]);
for k=1:4
 b=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace','VariableName',roots{k});assert(numel(b)==1);
 ph=get_param(b{1},'PortHandles');src=get_param(get_param(ph.Inport(1),'Line'),'SrcPortHandle');
 name=[mdl '/ML native selector ' num2str(k)];add_block('simulink/Signal Routing/Selector',name, ...
  'InputPortWidth',num2str(widths(k)),'Indices',indices{k},'Position',[3550 100+70*k 3800 145+70*k]);
 sh=get_param(name,'PortHandles');add_line(mdl,src,sh.Inport(1),'autorouting','on');
 add_line(mdl,['ML native selector ' num2str(k) '/1'],['ML full native vector/' num2str(k)],'autorouting','on');
end
for k=1:2
 if k==1,tag='delayed_robot_velocity';else,tag='robot_velocity_request';end
 name=['ML native command ' num2str(k)];add_block('simulink/Signal Routing/From',[mdl '/' name],'GotoTag',tag,'Position',[3550 500+65*k 3800 535+65*k]);
 add_line(mdl,[name '/1'],['ML full native vector/' num2str(k+4)],'autorouting','on');
end
add_block('simulink/Sinks/To Workspace',[mdl '/ML full native logger'],'VariableName','ml_robot_native_log', ...
 'SaveFormat','Timeseries','MaxDataPoints','inf','Decimation','1','SampleTime','p.sim.step','Position',[4050 225 4330 295]);
add_line(mdl,'ML full native vector/1','ML full native logger/1','autorouting','on');
end

function set_historical_logging(mdl,enabled)
blocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','ToWorkspace');
for k=1:numel(blocks)
 if ismember(get_param(blocks{k},'VariableName'),{'ml_robot_native_log','ml_robot_policy_log','adc_acquisition_log'}),continue;end
 if enabled,comment='off';else,comment='on';end
 set_param(blocks{k},'Commented',comment);
end
end

function row=score(p,t,a,tp,b,row)
err=a(:,15)-p.force.reference;[peak,idx]=max(abs(err));
row.nativeSamples=numel(t);row.endTime_s=t(end);row.completed=t(end)>=p.sim.stopTime-p.sim.step/2;
row.peakNormalForceError_N=peak;row.peakTime_s=t(idx);row.rmsNormalForceError_N=sqrt(mean(err.^2));
row.peakStartupError_N=masked_peak(err,t<=.25);
row.peakBumpError_N=masked_peak(err,t>=p.surface.startTime-.05 & t<=p.surface.startTime+p.d.duration+.2);
row.peakCurrent_A=max(abs(a(:,3)));row.peakVoltage_V=max(abs(a(:,4)));row.voltageClipFraction=mean(a(:,8)>.5);
row.lostContact=any(a(:,5)<.5);row.strokeStop=any(a(:,6)>.5);row.impact=any(abs(a(:,9))>1e-10);
row.minimumTravel_m=min(a(:,2));row.maximumTravel_m=max(a(:,2));
row.finalTravelError_m=a(end,2)-p.mechanics.midTravel;
row.peakTravelError_m=max(abs(a(:,2)-p.mechanics.midTravel));
row.meanAbsoluteTravelError_m=mean(abs(a(:,2)-p.mechanics.midTravel));
row.travelErrorIntegral_m_s=trapz(t,abs(a(:,2)-p.mechanics.midTravel));
row.peakIssuedCorrection_m_s=max(abs(b(:,6)));row.nonzeroCorrectionFrames=sum(abs(b(:,6))>1e-10);
row.policyFrames=numel(tp);row.gateClosedCommandViolations=sum(b(:,5)<.5 & abs(b(:,6))>1e-12);
row.directionEnvelopeViolations=sum(b(:,3).*b(:,2)<-1e-14 | abs(b(:,3))>abs(b(:,2))+1e-12);
row.networkInferenceResidual_m=max(abs(b(:,8)-network_distance(p.mlRobot,b(:,1))));
rawAction=b(:,8)*p.uncertainty.recenterGain;
row.rawNetworkTeacherActionRmsError_m_s=sqrt(mean((rawAction-b(:,2)).^2));
row.rawNetworkTeacherActionPeakError_m_s=max(abs(rawAction-b(:,2)));
row.boundedNetworkTeacherActionRmsError_m_s=sqrt(mean((b(:,3)-b(:,2)).^2));
row.boundedNetworkTeacherActionPeakError_m_s=max(abs(b(:,3)-b(:,2)));
row.networkEnvelopeClippedFrames=sum(abs(rawAction-b(:,3))>1e-12);
assert(row.networkInferenceResidual_m<1e-12&&row.gateClosedCommandViolations==0&&row.directionEnvelopeViolations==0);
ready=interp1(tp,b(:,5),t,'previous','extrap');closed=ready<.5;
row.pendingCommandMetricIncludesNominalPath=abs(p.robot.nominalVelocity)>0;
row.queuedCommandAfterGateCloseSamples=sum(closed & abs(a(:,22)-p.robot.nominalVelocity*double(~p.robotPrediction.boundedNominalPath))>1e-6);
row.peakQueuedCommandWhileGateClosed_m_s=masked_peak(a(:,22),closed);
row.peakRobotVelocityWhileGateClosed_m_s=masked_peak(a(:,21),closed);
row.gateClosures=sum(diff(b(:,5))<-.5);row.gateOpenFraction=mean(b(:,5)>.5);
row.projectionBalanceResidual_N=max(abs(a(:,15)-(a(:,1)-a(:,18).*a(:,17))./sqrt(1+a(:,18).^2)));
attached=a(:,5)>.5&a(:,6)<.5;
row.rigidAxialBalanceResidual_N=masked_peak(a(:,1)-a(:,12)-p.mechanics.mB*(p.d.g+a(:,13))-p.mechanics.mS*(p.d.g+a(:,19)),attached);
row.lowerLinkBalanceResidual_N=masked_peak(a(:,1)-a(:,14)-p.mechanics.mS*(p.d.g+a(:,19)),attached);
assert(max([row.projectionBalanceResidual_N,row.rigidAxialBalanceResidual_N,row.lowerLinkBalanceResidual_N])<1e-8);
q=p.d.q10/p.mechanics.k1+a(:,11)-a(:,10);travel=max(abs(q));spring=p.springs.lower;pack=p.springs;
minPreload=spring.rate_N_m*(travel+pack.unseatingClearance_m);
compression=min(spring.freeLength_m-spring.freeLengthTolerance_m-spring.minWorkingLength_m-pack.workingClearance_m,spring.forceEnvelope_N/spring.rate_N_m);
maxPreload=spring.rate_N_m*(compression-travel);
row.springPackingPassed=pack.commonPreload_N>=minPreload&&pack.commonPreload_N<=maxPreload&& ...
 max(q)-min(q)<=spring.maxCyclicStroke_m+1e-12&&2000*(spring.freeLength_m+spring.freeLengthTolerance_m)<=100;
row.passed=row.completed&&peak<=.5&&~row.lostContact&&~row.strokeStop&&~row.impact&&row.peakCurrent_A<=p.coil.peakCurrent*(1+1e-4)&&row.springPackingPassed;
row.designMarginPassed=row.passed&&peak<=.45;
if row.passed,row.reason='Complete native normal force and hard policy invariants passed';else,row.reason='Native force or physical requirement failed';end
end

function y=network_distance(n,x)
h=max(0,n.W1(:)*(x(:)'/n.inputScale_m)+n.b1(:));
y=(n.W2(:)'*h+n.b2-n.zeroOutput)'*n.inputScale_m;
end
function v=masked_peak(x,m)
if any(m),v=max(abs(x(m)));else,v=0;end
end
function a=tsrows(ts,n)
a=ts.Data;t=numel(ts.Time);if size(a,1)~=t,a=permute(a,[ndims(a),1:ndims(a)-1]);end;a=reshape(a,t,n);
end
function a=append(a,b)
if isempty(a),a=b;else,a(end+1)=b;end
end
function finish(mdl,storage,record,sdi)
try,set_param(mdl,'FastRestart','off');close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_ml_robot_20261005')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
d=java.security.MessageDigest.getInstance('SHA-256');while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');d.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
