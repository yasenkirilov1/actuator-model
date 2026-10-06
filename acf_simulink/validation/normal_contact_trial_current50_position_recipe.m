function proof=normal_contact_trial_current50_position_recipe()
% Native finite-cell proof of the physical S-relative-A position instrument.
assert(~usejava('desktop'));
source='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v4-20260930/frozen-inertial-v4';
work='/home/tommie/.local/state/actuator-model/normal-contact-trial-current50-position-20260930';
folder=fileparts(mfilename('fullpath'));
for n={'','helpers','cache','code','sdi'}
 f=fullfile(work,n{1});if ~isfolder(f),mkdir(f);end
end
copyfile(fullfile(source,'acf_force_control_normal_trial.slx'),fullfile(work,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(source,'helpers','*.m'),fullfile(work,'helpers'));
runner=fullfile(work,'helpers','acf_run.m');body=fileread(runner);
anchor='r.time=a.Time(:);';
replacement=sprintf('%s\npositionNative=out.get(''s_head_position_log'');\nr.positionInstrument=struct(''time'',positionNative.Time(:),''data'',rows(positionNative,4));',anchor);
assert(contains(body,anchor));body=strrep(body,anchor,replacement);
fid=fopen(runner,'w');fprintf(fid,'%s',body);fclose(fid);
addpath(fullfile(work,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator;
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(work,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(work,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
page=[mdl '/Relative position measurement'];
add_block('simulink/Signal Routing/Mux',[page '/Native instrument audit vector'],'Inputs','4','Position',[2250 300 2255 470]);
signals={'Measurement delay','Tool S relative to A','travel_m','Position noise RMS'};
for j=1:4
 sampler=sprintf('Native audit sample %d',j);
 add_block('simulink/Discrete/Zero-Order Hold',[page '/' sampler],'SampleTime','p.sim.step','Position',[2050 280+50*j 2170 315+50*j]);
 add_line(page,[signals{j} '/1'],[sampler '/1'],'autorouting','on');
 add_line(page,[sampler '/1'],sprintf('Native instrument audit vector/%d',j),'autorouting','on');
end
add_block('simulink/Sinks/To Workspace',[page '/Native instrument audit log'],'VariableName','s_head_position_log','SaveFormat','Timeseries','Position',[2310 350 2460 405]);
add_line(page,'Native instrument audit vector/1','Native instrument audit log/1','autorouting','on');
set_param(mdl,'AlgebraicLoopMsg','error','SimulationMode','accelerator','FastRestart','off');
history=load('/home/tommie/.local/state/actuator-model/normal-contact-trial-current50-20260930/traces/normal_contact_trial_current50_010.mat','r');
p=history.r.parameters;p.inertial.feedbackFilter=struct('enabled',false,'cutoff_Hz',800,'damping',1/sqrt(2));
r=acf_run(mdl,p,false);
assert(isequal(r.time,r.positionInstrument.time));
logged=r.positionInstrument.data;physicalSminusA=r.data(:,7)-r.data(:,5);
physicalBminusA=r.data(:,6)-r.data(:,5);
stride=round(p.force.sampleTime/p.sim.step);assert(abs(stride*p.sim.step-p.force.sampleTime)<1e-14);
delay=round(p.d.strokeDelaySamples);sampleIndex=floor((0:numel(r.time)-1)'/stride)*stride+1;
delayedIndex=sampleIndex-delay*stride;initial=delayedIndex<1;
expected=zeros(size(r.time));noiseless=expected;
expected(initial)=p.d.positionMeasurementInitial;noiseless(initial)=p.d.positionMeasurementInitial;
ix=delayedIndex(~initial);
noiseless(~initial)=p.mechanics.midTravel+physicalSminusA(ix);
expected(~initial)=noiseless(~initial)+logged(ix,4);
sensorNoise=logged(:,1)-noiseless;
proof=struct('date','2026-09-30','sourceFolder',source,'workFolder',work, ...
 'status','native_executed','sampleCount',numel(r.time),'duration_s',r.time(end), ...
 'cellStiffness_N_m',p.mechanics.k1,'cellDamping_Ns_m',p.mechanics.c1,'lowerRigid',p.mechanics.lowerRigid, ...
 'positionModel',p.sensor.positionModel,'sensorDefinition','S minus A plus midpoint, sampled and delayed; independent uniform position noise', ...
 'nativePositionBalanceResidual_m',max(abs(logged(:,2)-physicalSminusA)), ...
 'sampledDelayedSensorResidual_m',max(abs(logged(:,1)-expected)), ...
 'headVsMagnetPositionDifference_m',max(abs(physicalSminusA-physicalBminusA)), ...
 'sensorNoiseRms_m',sqrt(mean(sensorNoise(~initial).^2)), ...
 'sensorNoiseBound_m',max(abs(sensorNoise)), 'specifiedPositionNoiseRms_m',p.sensor.strokeNoiseRms, ...
 'sampleTime_s',p.force.sampleTime,'delaySamples',delay,'step_s',p.sim.step, ...
 'peakNormalForceError_N',max(abs(r.normalContact.data(:,1)-p.force.reference)), ...
 'allPositionSignalsAtNativeTimes',true, 'traceFile',fullfile(work,'normal_contact_trial_current50_position_trace.mat'));
assert(proof.nativePositionBalanceResidual_m<1e-13);
assert(proof.sampledDelayedSensorResidual_m<1e-12);
assert(proof.headVsMagnetPositionDifference_m>1e-8);
assert(proof.sensorNoiseBound_m<=sqrt(3)*p.sensor.strokeNoiseRms+1e-12);
proof.positionProofPass=true;
save(proof.traceFile,'r','proof','expected','noiseless');
save(fullfile(folder,'normal_contact_trial_current50_position_results.mat'),'proof');
fid=fopen(fullfile(folder,'normal_contact_trial_current50_position_results.json'),'w');fprintf(fid,'%s\n',jsonencode(proof,PrettyPrint=true));fclose(fid);
fprintf('FINITE S-A POSITION PROOF: native %.3g m, sampled/delayed %.3g m, head/magnet difference %.3g m, noise %.3g m RMS.\n',proof.nativePositionBalanceResidual_m,proof.sampledDelayedSensorResidual_m,proof.headVsMagnetPositionDifference_m,proof.sensorNoiseRms_m);
end

function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
