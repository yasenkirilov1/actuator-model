function study=qualify_robot_gate(mdl,p)
%QUALIFY_ROBOT_GATE Exercise delayed recentering with a 4 mm initial offset.
% Metrics use every native sample. Plot histories retain one sample per 10 ms.
% The loaded model is restored; this function never saves its diagram.
mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
cleanup=onCleanup(@()acf_select_restore(mdl,mw,original)); %#ok<NASGU>
folder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'validation');
rows=struct([]);traces=struct([]);
for delay=[.08 3]
 for policy=0:2
  q=p;q.surface.kind='flat';q.sim.stopTime=60;q.robot.delay=delay;
  % acf_prepare converts this to an initial robot displacement; the initial
  % spring force remains the force-balanced holding state.
  q.robot.initialExcessForce=.004*q.mechanics.k1;
  q.robot.forceGate.enabled=policy>0;
  if policy==1,q.robot.forceGate.maxDelayGainProduct=100;end
  r=acf_run(mdl,q,false);policies={'ungated','gate_only','gate_and_delay_gain_cap'};
  late=r.time>=45;lateError=NaN;
  if any(late),lateError=1000*max(abs(r.data(late,2)-q.mechanics.midTravel));end
  row=struct('delay_s',delay,'policy',policies{policy+1}, ...
   'effectiveRecenterGain_s_inv',r.parameters.d.recenterGain, ...
   'peakForceError_N',r.metrics.peakAbsForceError_N, ...
   'lateTravelErrorMax_mm',lateError, ...
   'finalTravelError_mm',1000*abs(r.data(end,2)-q.mechanics.midTravel),'metrics',r.metrics);
  take=unique([1:100:numel(r.time),numel(r.time)]);
  trace=struct('delay_s',delay,'policy',policies{policy+1},'time_s',r.time(take), ...
   'travelError_mm',1000*(r.data(take,2)-q.mechanics.midTravel), ...
   'forceError_N',r.data(take,1)-q.force.reference, ...
   'robotVelocityRequest_m_s',r.data(take,34), ...
   'robotVelocity_m_s',r.data(take,27),'nativeSampleCount',numel(r.time), ...
   'metricsFromFullNativeGrid',r.metrics);
  if isempty(rows),rows=row;traces=trace;else,rows(end+1)=row;traces(end+1)=trace;end %#ok<AGROW>
  clear r;
  save(fullfile(folder,'selected_robot_gate_plot_data.mat'),'traces','rows','-v7');
  study=struct('date','2026-09-30','mechanics','recommended upper-rigid, lower 19.746 N/mm and 60 N s/m', ...
   'initialTravelOffset_mm',4,'gateParameters',p.robot.forceGate,'duration_s',60,'cases',rows);
  fid=fopen(fullfile(folder,'selected_robot_gate_results.json'),'w');
  assert(fid~=-1,'Cannot write gate results');fprintf(fid,'%s',jsonencode(study,PrettyPrint=true));fclose(fid);
 end
end
end

function acf_select_restore(mdl,mw,p)
assignin(mw,'p',p);acf_select_actuator(mdl,p.actuator.kind);
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
end
