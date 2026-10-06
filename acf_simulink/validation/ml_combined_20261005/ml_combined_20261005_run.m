function report=ml_combined_20261005_run(root,outputFolder)
%ML_COMBINED_20261005_RUN Four predeclared, new baseline/combined native pairs.
% Both trained policies and the force selection gain are fixed before tests.
assert(~usejava('desktop'),'Use an isolated matlab -batch process.');
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,outputFolder=fullfile(root,'acf_simulink','validation','ml_combined_20261005');end
assert(~isfile(fullfile(outputFolder,'results.json')),'Preserve completed attempts.');
if ~isfolder(outputFolder),mkdir(outputFolder);end
maxNumCompThreads(2);project=fullfile(root,'acf_simulink');
source=fullfile(project,'validation','adc_acquisition_trial_20261002');
sourceModel=fullfile(source,'acf_force_control_adc_trial.slx');sourceSha=sha256(sourceModel);
assert(strcmp(sourceSha,'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
robotWeights=fullfile(project,'validation','ml_robot_20261005_training','network_weights.mat');
forceWeights=fullfile(project,'validation','ml_force_20261005','trained_weights.mat');
robotSha=sha256(robotWeights);forceSha=sha256(forceWeights);
assert(strcmp(robotSha,'1fc08b808bc43986bd927532d60edf27a930bcfe6ef5a5ff4f545f391406e52e'));
assert(strcmp(forceSha,'e21982cf35ed2790e831bab0cdd45f76271bfb90b872195c9493bce094fc8b05'));
selection=jsondecode(fileread(fullfile(project,'validation','ml_force_20261005','selected_gain.json')));
assert(selection.selectedGain==1,'Do not retune gain from test data.');
worker=fullfile(outputFolder,'private_worker');mkdir(worker);
for name={'helpers','cache','code'},mkdir(fullfile(worker,name{1}));end
copyfile(fullfile(source,'helpers','*.m'),fullfile(worker,'helpers'));
modelFile=fullfile(worker,'acf_force_control_adc_trial.slx');copyfile(sourceModel,modelFile);
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'ml_combined_20261005_run.m'));
for recipe={'ml_robot_20261005_build_policy.m','ml_force_20261005_build.m'}
 copyfile(fullfile(fileparts(mfilename('fullpath')),recipe{1}),fullfile(outputFolder,recipe{1}));
end
copyfile(robotWeights,fullfile(outputFolder,'robot_weights.mat'));
copyfile(forceWeights,fullfile(outputFolder,'force_weights.mat'));
addpath(outputFolder,'-begin');addpath(fullfile(worker,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_ml_combined_20261005'),'w');fclose(fid);
mdl='acf_force_control_adc_trial';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
set_param(mdl,'SimulationMode','accelerator','FastRestart','off');
ml_robot_20261005_build_policy(mdl);
[same,forceAudit]=ml_force_20261005_build(modelFile,fullfile(outputFolder,'force_weights.mat'),selection.selectedGain);assert(strcmp(same,mdl));
ml=getVariable(get_param(mdl,'ModelWorkspace'),'ml');
add_compact_logger(mdl);set_historical_logging(mdl,false);
nominal=load(fullfile(project,'validation','adc_budget_range_20261002_part1','input_configs.mat'),'configs');
base=nominal.configs(1).p;
assert(base.coil.forceConstant==20&&base.adcStudy.outputRate_Hz==30000&&base.adcStudy.analogCutoff_Hz==8000);
base.coil.forceConstant=24.4;base.coil.resistance=8.896;base.coil.inductance=.004;base.coil.voltage=32;
base.uncertainty.forceConstant_model_N_A=24.4;base.inertial.lowerLoadInverse.identifiedForceConstant_N_A=24.4;
for field={'forceKp','forceKi','forceKd'},base.inertial.(field{1})=base.inertial.(field{1})*20/24.4;end
base.robot.nominalVelocity=0;base.robot.initialExcessForce=0;
hotFile=fullfile(project,'validation','budget_adc_hot_coil_20261002_inputs', ...
 'ADXL1002_8000_M08_30k_current100_hot32V_configs.mat');hot=load(hotFile,'configs');
learned=load(robotWeights,'network');network=learned.network;
declared=[.1 5 0 0 3101 -1.5;.0 50 90 .3 3102 1.5;1 25 45 .3 3103 1.0];declared(2,1)=2;
configs=struct([]);
for k=1:4
 if k<=3
  r=declared(k,:);p=base;p.mechanics.mS=r(1);p.force.reference=r(2);p.mechanics.angle=r(3)*pi/180;
  p.contact.mu=r(4);p.sensor.seed=r(5);p.robot.initialPositionOffset_m=r(6)*1e-3;
  p.sensor.distalMass_kg=r(1);p.sensor.distalMass_model_kg=r(1);p.sensor.distalMassCalibrationScale=1;
  if k==3,p.robot.delay=.085;p.robot.responseTime=.055;end
 else
  p=hot.configs(1).p;p.sensor.seed=3104;p.robot.initialPositionOffset_m=.0015;p.robot.initialExcessForce=0;
 end
 p.sim.stopTime=8;p.sim.evaluateFrom=0;
 for mode={'baseline','combined'}
  p.mlRobot=network;p.mlRobot.enabled=double(strcmp(mode{1},'combined'));
  configs=append(configs,struct('p',p,'identity',sprintf('combined_new_%02d',k),'split','combined_test','policy',mode{1}));
 end
end
save(fullfile(outputFolder,'input_configs.mat'),'configs','-v7');
assignin(get_param(mdl,'ModelWorkspace'),'p',acf_prepare(configs(1).p));ml.gain=0;assignin(get_param(mdl,'ModelWorkspace'),'ml',ml);
set_param(mdl,'FixedStep',num2str(configs(1).p.sim.step,17),'StopTime','8');save_system(mdl,modelFile);
helpers=struct([]);files=dir(fullfile(worker,'helpers','*.m'));
for j=1:numel(files),helpers=append(helpers,struct('file',files(j).name,'sha256',sha256(fullfile(files(j).folder,files(j).name))));end
report=struct('status','running','date','2026-10-05','phase','combined_test','sourceModelSha256',sourceSha, ...
 'privateModelSha256',sha256(modelFile),'sourceHotParametersSha256',sha256(hotFile), ...
 'recipeSha256',sha256(fullfile(outputFolder,'ml_combined_20261005_run.m')), ...
 'robotBuilderSha256',sha256(fullfile(outputFolder,'ml_robot_20261005_build_policy.m')), ...
 'forceBuilderSha256',sha256(fullfile(outputFolder,'ml_force_20261005_build.m')), ...
 'robotWeightsSha256',robotSha,'forceWeightsSha256',forceSha,'fixedForceSelectionGain',selection.selectedGain, ...
 'helpers',helpers,'matlabVersion',version,'plannedIdentities',4,'plannedRecordings',8,'forceBuilderAudit',forceAudit, ...
 'scope','Four new, predeclared eight-second native baseline/combined-policy pairs. Fixed weights/gain; no retuning and no physical sanding qualification.', ...
 'physicalColumns',{{'Rz_N','travel_m','I_A','V_V','contact','stop','effortClip','voltageClip','impactImpulse_Ns', ...
 'xB_m','xS_m','Fa_N','aB_m_s2','lowerForce_N','N_N','Nestimate_N','Rx_N','slope','aS_m_s2', ...
 'robotPosition_m','robotVelocity_m_s','delayedRobotRequest_m_s','issuedRobotRequest_m_s'}}, ...
 'policyColumns',{{'strokeError_m','teacherCorrection_m_s','boundedNetworkCorrection_m_s','selectedCorrection_m_s', ...
 'forceGateReady','issuedCorrection_m_s','causalEstimatedTravel_m','rawNetworkDistance_m'}},'rows',struct([]));
rows=struct([]);started=tic;write_report('running');
for j=1:8
 c=configs(j);timer=tic;out=[];returned=false;
 row=struct('recording',j,'identity',c.identity,'split',c.split,'policy',c.policy, ...
  'massS_kg',c.p.mechanics.mS,'force_N',c.p.force.reference,'angle_deg',c.p.mechanics.angle*180/pi, ...
  'mu',c.p.contact.mu,'seed',c.p.sensor.seed,'offset_m',c.p.robot.initialPositionOffset_m, ...
  'actualRobotDelay_s',c.p.robot.delay,'actualRobotResponse_s',c.p.robot.responseTime, ...
  'executed',false,'passed',false,'reason','');
 try
  q=acf_prepare(c.p);assignin(get_param(mdl,'ModelWorkspace'),'p',q);
  ml.gain=selection.selectedGain*double(strcmp(c.policy,'combined'));assignin(get_param(mdl,'ModelWorkspace'),'ml',ml);
  set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
  out=sim(mdl);returned=true;log=out.get('ml_robot_native_log');t=log.Time(:);a=tsrows(log,23);
  policyLog=out.get('ml_robot_policy_log');tp=policyLog.Time(:);b=tsrows(policyLog,8);
  assert(t(1)==0&&all(isfinite(a(:)))&&numel(t)==round(t(end)/q.sim.step)+1);
  assert(all(abs(diff(t)-q.sim.step)<max(1e-12,q.sim.step*1e-6)));
  row=score(q,t,a,tp,b,row);
  adc=out.get('adc_acquisition_log');ad=tsrows(adc,8);row.anyAdcClipping=any(any(ad(:,5:8)>.5));
  row.passed=row.passed&&~row.anyAdcClipping;row.designMarginPassed=row.passed&&row.peakNormalForceError_N<=.45;
  f=out.get('ml_features_log');tf=f.Time(:);fd=tsrows(f,15);
  rawFeatures=fd(:,1:10);z=max(-6,min(6,(rawFeatures-ml.mean(:)')./ml.scale(:)'));
  predicted=max(-ml.actionLimit_A,min(ml.actionLimit_A,max(0,z*ml.W1'+ml.b1(:)')*ml.W2'+ml.b2))*ml.gain;
  row.forceNetworkInferenceResidual_A=max(abs(predicted-fd(:,11)));
  assert(row.forceNetworkInferenceResidual_A<1e-10);
  row.peakLearnedCurrent_A=max(abs(fd(:,11)));row.learnedCurrentRms_A=sqrt(mean(fd(:,11).^2));
  row.learnedCurrentNonzeroFrames=sum(abs(fd(:,11))>1e-8);row.forceFeatureFrames=numel(tf);
  row.executed=true;row.nativeReturned=true;row.runtime_s=toc(timer);
  trace=struct('time',t,'data',a,'policyTime',tp,'policyData',b,'parameters',q,'forceNetwork',ml, ...
   'forceFeatureTime',tf,'forceFeatures',fd,'columns',{report.physicalColumns},'policyColumns',{report.policyColumns},'row',row);
  ix=unique([(1:max(1,round(100e-6/q.sim.step)):numel(t))';find(t==row.peakTime_s)]);
  compact=trace;compact.time=t(ix);compact.data=a(ix,:);
  save(fullfile(outputFolder,sprintf('recording_%03d_compact.mat',j)),'compact','-v7');
  if ~row.passed||j>=7,save(fullfile(outputFolder,sprintf('recording_%03d_native.mat',j)),'trace','-v7.3');end
  save(fullfile(outputFolder,sprintf('recording_%03d_parameters.mat',j)),'q','ml','-v7');
 catch failure
  row.executed=false;row.nativeReturned=returned;row.runtime_s=toc(timer);row.reason=getReport(failure,'extended','hyperlinks','off');
  if returned,save(fullfile(outputFolder,sprintf('recording_%03d_unvalidated.mat',j)),'out','c','-v7.3');end
 end
 rows=append(rows,row);write_report('running');
 fid=fopen(fullfile(outputFolder,'rows.jsonl'),'a');fprintf(fid,'%s\n',jsonencode(row));fclose(fid);
 fprintf('ML_COMBINED_NATIVE %d/8 %s executed%d pass%d',j,c.policy,row.executed,row.passed);
 if isfield(row,'peakNormalForceError_N'),fprintf(' peak%.9gN robot%.6gmm/s MLcurrent%.6gA runtime%.1fs',row.peakNormalForceError_N,row.peakIssuedCorrection_m_s*1000,row.peakLearnedCurrent_A,row.runtime_s);end
 fprintf('\n');clear out log policyLog a b t tp trace compact q f fd rawFeatures z predicted ad adc;Simulink.sdi.clear;
 if ~row.executed,write_report('software_rejection');error('acf:combined:Contract','%s',row.reason);end
end
assert(strcmp(sourceSha,sha256(sourceModel))&&strcmp(robotSha,sha256(robotWeights))&&strcmp(forceSha,sha256(forceWeights)));
write_report('complete');
 function write_report(status)
  report.status=status;report.rows=rows;report.recordingsReturned=numel(rows);
  if isempty(rows),report.executed=0;report.passed=0;report.nativeSamples=0;
  else,report.executed=sum([rows.executed]);report.passed=sum([rows.passed]);
   if all([rows.executed]),report.nativeSamples=sum([rows.nativeSamples]);report.worstNormalForceError_N=max([rows.peakNormalForceError_N]);end
  end
  report.elapsed_s=toc(started);fid=fopen(fullfile(outputFolder,'results.json'),'w');fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
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
 if ismember(get_param(blocks{k},'VariableName'),{'ml_robot_native_log','ml_robot_policy_log','adc_acquisition_log','ml_features_log'}),continue;end
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
if isfile(fullfile(sdi,'owned_ml_combined_20261005')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
d=java.security.MessageDigest.getInstance('SHA-256');while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');d.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
