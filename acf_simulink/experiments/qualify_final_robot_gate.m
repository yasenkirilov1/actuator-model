function study=qualify_final_robot_gate(mdl,p)
%QUALIFY_FINAL_ROBOT_GATE Native delayed recenter tests with actual two sensors.
% Metrics use all native samples. Only 10 ms plot samples are retained here.
% Does not save the diagram. Run in an isolated batch for long recordings.
mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
cleanup=onCleanup(@()restore(mdl,mw,original)); %#ok<NASGU>
folder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'validation');
recipes=[.08 2 0 20;.08 2 1 20;3 0 0 60;3 1 0 60;3 2 0 60;3 2 1 60];
rows=struct([]);traces=struct([]);policies={'ungated','gate_only','gate_and_delay_gain_cap'};
for j=1:size(recipes,1)
 delay=recipes(j,1);policy=recipes(j,2);noise=recipes(j,3);duration=recipes(j,4);
 q=p;q.surface.kind='flat';q.sim.stopTime=duration;q.robot.delay=delay;
 q.robotPrediction.delay=delay;q.robot.nominalVelocity=0;
 q.robot.initialExcessForce=0;q.robot.initialPositionOffset_m=-.004;
 q.sensor.seed=1729;q.sensor.noiseRms=.02*noise;q.sensor.strokeNoiseRms=1e-6*noise;
 q.robot.forceGate.enabled=policy>0;q.robot.forceGate.filterTime=.1;
 if policy==1,q.robot.forceGate.maxDelayGainProduct=100;end
 r=acf_run(mdl,q,false);gate=r.robotGate;
 row=struct('id',j,'delay_s',delay,'policy',policies{policy+1}, ...
  'forceNoiseRms_N',q.sensor.noiseRms,'positionNoiseRms_m',q.sensor.strokeNoiseRms, ...
  'duration_s',duration,'initialTravelOffset_mm',-4, ...
  'effectiveRecenterGain_s_inv',r.parameters.d.recenterGain, ...
  'peakForceError_N',r.metrics.peakAbsForceError_N,'passed',r.metrics.passed, ...
  'gateReadyFraction',mean(gate.ready),'finalTravelError_mm',1000*(r.data(end,2)-q.mechanics.midTravel), ...
  'gateFilterTime_s',.1,'gateRateLimit_N_s',2,'gateDwellTime_s',.1,'metrics',r.metrics);
 take=unique([1:round(.01/q.sim.step):numel(r.time),numel(r.time)]);
 trace=struct('id',j,'delay_s',delay,'policy',policies{policy+1},'noisy',logical(noise), ...
  'time_s',r.time(take),'travelError_mm',1000*(r.data(take,2)-q.mechanics.midTravel), ...
  'forceError_N',r.data(take,1)-q.force.reference, ...
  'robotVelocityRequest_m_s',r.data(take,34),'robotVelocity_m_s',r.data(take,27), ...
  'gateTime_s',gate.time,'gateReady',gate.ready,'nativeSampleCount',numel(r.time), ...
  'metricsFromFullNativeGrid',r.metrics);
 if isempty(rows),rows=row;traces=trace;else,rows(end+1)=row;traces(end+1)=trace;end %#ok<AGROW>
 clear r gate;
 save(fullfile(folder,'final_robot_gate_plot_data.mat'),'traces','rows','-v7');
 study=struct('date','2026-09-30','status','running','mechanics','Both rigid, actual B-S load cell and S-A position', ...
  'initialTravelOffset_mm',-4,'gateParameters',p.robot.forceGate,'cases',rows);
 if j==size(recipes,1),study.status='complete';end
 fid=fopen(fullfile(folder,'final_robot_gate_results.json'),'w');assert(fid~=-1);
 fprintf(fid,'%s',jsonencode(study,PrettyPrint=true));fclose(fid);
 fprintf('FINAL_GATE_CASE %d/%d %s\n',j,size(recipes,1),row.metrics.reason);
end
end
function restore(mdl,mw,p)
assignin(mw,'p',p);acf_select_actuator(mdl,p.actuator.kind);
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
end
