function report=compare_force_controllers(snapshotFolder,outputFolder,phase,workerFolder)
%COMPARE_FORCE_CONTROLLERS Fair, feedback-only native Simulink controller study.
% Owns an independent copied model/helpers. Never saves the user's main SLX.
% The PI/PID branch uses native sampling, gains, integrator, limits, antiwindup
% and a causal, sampled, first-order filtered derivative. No surface preview.
if nargin<1,snapshotFolder='/home/tommie/.local/state/actuator-model/qualification-20260930';end
if nargin<2,outputFolder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'validation');end
if nargin<3,phase='observer';end
if strcmp(phase,'current_uncertainty')
 report=current_uncertainty_study(snapshotFolder,outputFolder);return;
end
upstream=startsWith(phase,'upstream_');
rigidCoulomb=startsWith(phase,'upstream_rigid_');
contactGate=ismember(phase,{'upstream_rigid_corrected_gate','upstream_rigid_timing_bias', ...
 'upstream_rigid_calibration_bounds'});
if nargin<4
 if contactGate,workerFolder=fullfile(snapshotFolder,'controller_upstream_rigid_gate_worker');
 elseif rigidCoulomb,workerFolder=fullfile(snapshotFolder,'controller_upstream_rigid_worker');
 elseif upstream,workerFolder=fullfile(snapshotFolder,'controller_upstream_worker');
 else,workerFolder=fullfile(snapshotFolder,'controller_worker');end
end
worker=workerFolder;
assert(~usejava('desktop'),'Run this study in an isolated MATLAB -batch process.');
if upstream,ensure_upstream_worker(snapshotFolder,worker,rigidCoulomb);end
ensure_worker(snapshotFolder,worker);
maxNumCompThreads(min(2,maxNumCompThreads));
addpath(fullfile(worker,'helpers'),'-begin');
load(fullfile(snapshotFolder,'base_parameters.mat'),'p');
catalog=jsondecode(fileread(fullfile(snapshotFolder,'spring_catalog.json')));
for name={'cache','code','sdi','traces'}
 path=fullfile(worker,name{1});if ~isfolder(path),mkdir(path);end
end
if ~isfolder(outputFolder),mkdir(outputFolder);end
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(worker,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_simple';
if ~bdIsLoaded(mdl),load_system(fullfile(worker,[mdl '.slx']));end
assert(strcmp(get_param(mdl,'FileName'),fullfile(worker,[mdl '.slx'])), ...
 'A different copy of this model is already loaded; use a new MATLAB -batch process.');
set_param(mdl,'FastRestart','off');
add_filtered_derivative(mdl);
delayBlock=[mdl '/Force measurement/Sensor sample delay'];
normalDelay='p.d.sensorDelaySamples';
cleanup=onCleanup(@()restore_model(mdl,delayBlock,normalDelay)); %#ok<NASGU>
p.coil.strategy='observer';p.coil.forceKp=0;p.coil.forceKi=0;p.coil.forceKd=0;
p.coil.derivativeFilter=.0003;
p.robot.forceGate.enabled=true;
p.force.reference=50;p.mechanics.angle=pi/2;p.sensor.noiseRms=.02;p.sensor.seed=7;
p.sensor.filterTime=.0003;p.sensor.strokeNoiseRms=1e-6;p.sensor.strokeDelay=.0002;
p.coil.currentResponse=.0004;p.sim.step=.0001;p.sim.stopTime=4;p.sim.evaluateFrom=0;
if upstream
 p.sensor.distalMass_kg=1.4;p.sensor.distalMass_model_kg=1.4;p.sensor.strokeBias=0;
 p.coil.peakCurrent=3.5;
 if isempty(p.ideal.maxAxialForce),p.ideal.maxAxialForce=inf;end
 p.robotPrediction=struct('enabled',true,'delay',.08,'responseTime',.06, ...
  'maxSpeed',.02,'maxAcceleration',.5,'residualTime',.06,'residualJerkPower',.0002, ...
  'boundedNominalPath',true,'delaySamples',8);
 p.robot.nominalVelocity=.005;
end
if rigidCoulomb
 p.sensor.positionTarget='tool_relative_A';
 p.robotPrediction.surfaceJerkEnabled=false;p.robotPrediction.surfaceSnapPower=1e6;
 p.robotPrediction.surfaceJerkDecayTime=inf;p.robotPrediction.surfaceJerkFeedforwardGain=0;
 p.coil.directCoulombCorrection=true;
 p.force.sampleTime=.0001;p.sensor.sampleTime=.0001;
 p.sensor.filterTime=.0001;p.sensor.delay=.0001;p.sensor.strokeDelay=.0001;
 p.coil.currentResponse=.0001;p.sim.step=.00005;
 p.coil.control.forceScale_N=.32;p.coil.control.forceNoise_N=.02;
 p.coil.control.surfaceJerkPower=70;p.coil.control.strokeNoise_m=1e-6;
 p=mechanics(p,3,catalog);p=acf_prepare(p);
 assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 add_native_coulomb_correction(mdl);
 if contactGate,add_native_contact_gate(mdl);end
end
rows=struct([]);started=tic;runs=0;bestScore=inf;bestRun=[];bestParameters=[];
report=struct('date','2026-09-30','phase',phase,'status','running', ...
 'scope','1 mm x 10 mm bump at 100 mm/s, B+S=2 kg, 5-50 N and 0-90 degrees, true force within 0.5 N throughout.', ...
 'selectionScope','Screen at 50 N / 90 degrees with 0.02 N force RMS and extra unmodeled 0.2 ms force delay; independently validate shortlisted controllers across four force/orientation corners and four noise seeds.', ...
 'snapshot',snapshotFolder,'privateModel',fullfile(worker,[mdl '.slx']), ...
 'matlabVersion',version,'baseParameters',p,'rows',rows);
if upstream
 report.measurementScope='Actual B/S load cell with 1.4 kg downstream, exact static gravity tare: Fc minus 1.4 times surface acceleration. Observer uses the matching measurement row; no surface or actual robot truth enters it. Issued robot commands drive a causal nominal predictor.';
 report.selectionScope='Screen at 50 N / 90 degrees on the sharp bump with a bounded +/-5 mm/s issued normal command, 0.02 N force RMS, 1 um magnet RMS, and 0.2 ms unknown force delay. Peak-current limit is 3.5 A.';
end

switch phase
 case {'upstream_rigid_timing_bias','upstream_rigid_calibration_bounds'}
  q=mechanics(p,3,catalog);sensitivityRuns={};groups=struct([]);
  report.selectionScope='Corrected Coulomb/contact-gate both-rigid controller: calibrated 100 us force/position delay; actual force noise 0.02 N and S-A position noise 1 um. Separate unmodeled +100 us force latency and +/-0.05 N force-bias tests at four corners and four noise seeds. Each group worst case repeated at 25 and 12.5 us plant steps. No controller retuning.';
  settings=[.0001 0 .0001;0 -.05 .0001;0 .05 .0001];
  if strcmp(phase,'upstream_rigid_calibration_bounds')
   settings=[0 -.01 .0001;0 .01 .0001;0 -.02 .0001;0 .02 .0001;0 0 .0002];
   report.selectionScope='Fixed corrected-Coulomb/contact-gate both-rigid controller: actual force RMS0.02 N, S-A position RMS1 um, four corners x four seeds. Separate +/-0.01 and +/-0.02 N uncalibrated force bias at known100 us latency, then matched known200 us force latency. Current/force-filter/position latency100 us. Each group worst case repeated at25 and12.5 us plant step; no control-weight retuning.';
  end
  for j=1:size(settings,1)
   extraDelay=settings(j,1);bias=settings(j,2);knownDelay=settings(j,3);
   first=numel(rows)+1;worst=[];peak=-inf;
   for ref=[50 5]
    for angle=[90 0]
     for seed=[7 29 1729 31415]
      one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
      one.sensor.seed=seed;one.sensor.bias=bias;one.sensor.delay=knownDelay;select(one,extraDelay);
      r=run_and_append(one,3,extraDelay,sprintf('rigid_known%.0fus_extra%.0fus_bias%+.2f',knownDelay*1e6,extraDelay*1e6,bias));
      assert(~isempty(r),'A rigid sensitivity case did not execute.');
      if r.metrics.peakAbsForceError_N>peak,peak=r.metrics.peakAbsForceError_N;worst=r;end
     end
    end
   end
   coarse=rows(first:end);sensitivityRuns{end+1}=worst;finePeaks=[];commonDifferences=[];
   for step=[.000025 .0000125]
    one=worst.parameters;one.sim.step=step;select(one,extraDelay);set_param(mdl,'FastRestart','off');
    r=run_and_append(one,3,extraDelay,sprintf('rigid_timing_bias%d_step%.1fus',j,step*1e6));
    assert(~isempty(r),'A rigid sensitivity fine case did not execute.');
    finePeaks(end+1)=r.metrics.peakAbsForceError_N; %#ok<AGROW>
    ix=round(worst.time/step)+1;
    if max(ix)<=numel(r.time) && max(abs(worst.time-r.time(ix)))<1e-10
     commonDifferences(end+1)=max(abs(worst.data(:,1)-r.data(ix,1))); %#ok<AGROW>
    else,commonDifferences(end+1)=NaN;end %#ok<AGROW>
    sensitivityRuns{end+1}=r; %#ok<AGROW>
   end
   entry=struct('forceKnownDelay_s',knownDelay,'extraUnknownForceDelay_s',extraDelay,'uncalibratedForceBias_N',bias, ...
    'cornerRuns',16,'allCoarseCompleted',all([coarse.completed]),'allCoarsePassed',all([coarse.passed]), ...
    'worstCornerPeakError_N',peak,'step25usError_N',finePeaks(1),'step12p5usError_N',finePeaks(2), ...
    'step25usCommonHistoryDifference_N',commonDifferences(1),'step12p5usCommonHistoryDifference_N',commonDifferences(2), ...
    'allFinePassed',all([rows(end-1:end).passed]),'peakCurrent_A',max([coarse.peakCurrent_A]), ...
    'peakVoltage_V',max([coarse.peakVoltage_V]),'worstParameters',worst.parameters);
   if isempty(groups),groups=entry;else,groups(end+1)=entry;end %#ok<AGROW>
   report.rigidSensitivities=groups;checkpoint();
  end
  save(fullfile(outputFolder,['controller_comparison_' phase '_histories.mat']),'sensitivityRuns','groups');
 case {'upstream_rigid_qualify','upstream_rigid_corrected_gate'}
  q=mechanics(p,3,catalog);q.coil.directCoulombCorrection=true;
  q.sensor.strokeNoiseRms=1e-6;qualifiedRuns={};cleanRun=[];worst=[];worstPeak=-inf;
  report.selectionScope='Both rigid, actual B/S cell, direct S-A tool position: 4 clean force/orientation corners plus 4 corners x 4 force/position-noise seeds. 10 kHz, calibrated 100 us force/position delay and filter/current response, 3.5 A; corrected direct Coulomb term. Selected worst case also 25 and 12.5 us plant steps.';
  for stress=[false true]
   if stress,seeds=[7 29 1729 31415];else,seeds=1729;end
   for ref=[5 50]
    for angle=[0 90]
     for seed=seeds
      one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
      one.sensor.noiseRms=.02*stress;one.sensor.strokeNoiseRms=1e-6*stress;one.sensor.seed=seed;
      select(one,0);
      r=run_and_append(one,3,0,'rigid_coulomb_corrected_corners');
      assert(~isempty(r),'A rigid-controller corner did not execute.');
      if ~stress && ref==50 && angle==90,cleanRun=r;end
      if r.metrics.peakAbsForceError_N>worstPeak,worstPeak=r.metrics.peakAbsForceError_N;worst=r;end
     end
    end
   end
  end
  coarseRows=rows;qualifiedRuns{end+1}=worst;finePeaks=[];commonDifferences=[];
  for step=[.000025 .0000125]
   one=worst.parameters;one.sim.step=step;select(one,0);set_param(mdl,'FastRestart','off');
   r=run_and_append(one,3,0,sprintf('rigid_coulomb_worst_step%.1fus',step*1e6));
   assert(~isempty(r),'The rigid-controller fine case did not execute.');
   finePeaks(end+1)=r.metrics.peakAbsForceError_N; %#ok<AGROW>
   ix=round(worst.time/step)+1;assert(max(abs(worst.time-r.time(ix)))<1e-10);
   commonDifferences(end+1)=max(abs(worst.data(:,1)-r.data(ix,1))); %#ok<AGROW>
   qualifiedRuns{end+1}=r; %#ok<AGROW>
  end
  entry=struct('parameters',q,'cornerRuns',20,'allCoarsePassed',all([coarseRows.passed]), ...
   'worstCornerPeakError_N',worstPeak,'step25usError_N',finePeaks(1),'step12p5usError_N',finePeaks(2), ...
   'step25usCommonHistoryDifference_N',commonDifferences(1),'step12p5usCommonHistoryDifference_N',commonDifferences(2), ...
   'allFinePassed',all([rows(end-1:end).passed]),'peakCurrent_A',max([coarseRows.peakCurrent_A]), ...
   'peakVoltage_V',max([coarseRows.peakVoltage_V]),'worstParameters',worst.parameters);
  report.rigidQualification=entry;
  if contactGate
   entry.contactGateUsesReconstructedForce=true;
   entry.coarseGateReadyFractions=[coarseRows.gateReadyFraction];
   report.rigidQualification=entry;
   save(fullfile(outputFolder,'controller_comparison_upstream_rigid_corrected_gate_histories.mat'),'qualifiedRuns','cleanRun','entry');
   p=q;save(fullfile(outputFolder,'controller_comparison_upstream_rigid_corrected_gate_candidate.mat'),'p');
  else
   save(fullfile(outputFolder,'controller_comparison_upstream_rigid_qualified_histories.mat'),'qualifiedRuns','entry');
   p=q;save(fullfile(outputFolder,'controller_comparison_upstream_rigid_candidate.mat'),'p');
  end
  one=worst.parameters;one.sim.step=.000025;select(one,0);set_param(mdl,'FastRestart','off');
  before=run_and_append(one,3,0,'rigid_coulomb_save_before');
  set_param(mdl,'FastRestart','off');save_system(mdl,fullfile(worker,[mdl '.slx']));close_system(mdl,0);
  load_system(fullfile(worker,[mdl '.slx']));select(one,0);set_param(mdl,'FastRestart','off');
  after=run_and_append(one,3,0,'rigid_coulomb_save_reopen');
  difference=max(abs(before.data-after.data),[],'all');assert(difference<1e-12,'Private save/reopen changed the native signals.');
  report.rigidQualification.saveReopenAll35SignalDifference=difference;
  report.rigidQualification.privateSavedModel=fullfile(worker,[mdl '.slx']);
  report.rigidQualification.algebraicLoopCount=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));
 case 'upstream_rigid_coulomb'
  q=mechanics(p,3,catalog);rigidRuns={};
  report.selectionScope='Both rigid, actual B/S load cell with 1.4 kg downstream, direct S-A tool position, bounded robot commands, 10 kHz control, calibrated 0.1 ms force/position delay, 0.1 ms filter/current response, 3.5 A. Compare causal estimated-Coulomb measurement/force correction enabled and disabled.';
  for correction=[false true]
   for noiseCase=1:3
    one=q;one.coil.directCoulombCorrection=correction;
    if noiseCase==1,one.sensor.noiseRms=0;one.sensor.strokeNoiseRms=0;
    elseif noiseCase==2,one.sensor.noiseRms=.02;one.sensor.strokeNoiseRms=.1e-6;
    else,one.sensor.noiseRms=.02;one.sensor.strokeNoiseRms=1e-6;end
    select(one,0);set_param(mdl,'FastRestart','off');
    r=run_and_append(one,3,0,sprintf('rigid_coulomb%d_noise%d',correction,noiseCase));
    rigidRuns{end+1}=r; %#ok<AGROW>
   end
  end
  save(fullfile(outputFolder,'controller_comparison_upstream_rigid_coulomb_histories.mat'),'rigidRuns');
 case 'upstream_verify'
  labels={'upstream_calibrated_costs','upstream_gap_precision'};
  verified=struct([]);verificationRuns={};
  for j=1:numel(labels)
   input=load(fullfile(outputFolder,['controller_comparison_' labels{j} '.mat']),'bestParameters');
   q=input.bestParameters;q.sim.step=.0001;q.sensor.noiseRms=.02;
   first=numel(rows)+1;worst=[];peak=-inf;
   for seed=[7 29 1729 31415]
    one=q;one.sensor.seed=seed;select(one,0);set_param(mdl,'FastRestart','off');
    r=run_and_append(one,2,0,['ordinary_' labels{j}]);
    assert(~isempty(r),'Selected upstream run did not execute.');
    if r.metrics.peakAbsForceError_N>peak,peak=r.metrics.peakAbsForceError_N;worst=r;end
   end
   verificationRuns{end+1}=worst;finePeaks=[];historyDifferences=[]; %#ok<AGROW>
   for step=[.00005 .000025]
    one=worst.parameters;one.sim.step=step;select(one,0);set_param(mdl,'FastRestart','off');
    r=run_and_append(one,2,0,sprintf('%s_step%.0fus',labels{j},step*1e6));
    assert(~isempty(r),'Upstream solver verification did not execute.');
    ix=round(worst.time/step)+1;assert(max(abs(worst.time-r.time(ix)))<1e-10);
    finePeaks(end+1)=r.metrics.peakAbsForceError_N; %#ok<AGROW>
    historyDifferences(end+1)=max(abs(worst.data(:,1)-r.data(ix,1))); %#ok<AGROW>
    verificationRuns{end+1}=r; %#ok<AGROW>
   end
   group=rows(first:end);entry=struct('sourcePhase',labels{j},'parameters',worst.parameters, ...
    'worstFourSeedError_N',peak,'halfStepError_N',finePeaks(1),'quarterStepError_N',finePeaks(2), ...
    'halfStepHistoryDifference_N',historyDifferences(1),'quarterStepHistoryDifference_N',historyDifferences(2), ...
    'allCompleted',all([group.completed]),'allPassed',all([group.passed]));
   if isempty(verified),verified=entry;else,verified(end+1)=entry;end
   report.upstreamVerified=verified;checkpoint();
  end
  input=load(fullfile(outputFolder,'controller_comparison_upstream_direct_pid_histories.mat'),'actualCellRuns');
  worst=input.actualCellRuns{8};
  for step=[.00005 .000025]
   one=worst.parameters;one.sim.step=step;select(one,.0002);set_param(mdl,'FastRestart','off');
   r=run_and_append(one,2,.0002,sprintf('upstream_direct_pid_step%.0fus',step*1e6));
   verificationRuns{end+1}=r; %#ok<AGROW>
  end
  save(fullfile(outputFolder,'controller_comparison_upstream_verification_histories.mat'),'verificationRuns','verified');
 case 'upstream_gap_precision'
  q=mechanics(p,2,catalog);q.sensor.delay=.0002;q.mechanics.c1=100;
  q.coil.control.forceNoise_N=.03;q.coil.control.currentScale_A=.3;
  report.selectionScope='Actual B/S load cell, 50 N / 90 degrees, bounded +/-5 mm/s normal commands; 0.02 N force RMS, 0.1 or 1 um magnet RMS, calibrated 0.2 ms force and magnet latency, no extra unknown delay. Nine-state observer and 3.5 A drive.';
  for actualStroke=[.1 1]*1e-6
   for assumedStroke=[.1 .3 1]*1e-6
    for jerk=[70 300 1000 3000]
     for forceScale=[.08 .32]
      one=q;one.sensor.strokeNoiseRms=actualStroke;
      one.coil.control.strokeNoise_m=assumedStroke;
      one.coil.control.surfaceJerkPower=jerk;one.coil.control.forceScale_N=forceScale;
      select(one,0);
      run_and_append(one,2,0,'upstream_calibrated_gap_precision');
     end
    end
   end
  end
 case 'upstream_calibrated_costs'
  q=mechanics(p,2,catalog);q.sensor.delay=.0002;
  report.selectionScope='Actual B/S load cell, 50 N / 90 degrees, bounded +/-5 mm/s normal commands; 0.02 N force RMS, 1 um magnet RMS, calibrated 0.2 ms force and magnet latency, no extra unknown delay. Nine-state observer and 3.5 A drive.';
  for forceScale=[.08 .32 1.28]
   for forceAssumed=[.01 .03 .06]
    for currentScale=[.1 .3 1]
     for damping=[60 100 150]
      for jerk=[70 700]
       one=q;one.coil.control.forceScale_N=forceScale;
       one.coil.control.forceNoise_N=forceAssumed;
       one.coil.control.currentScale_A=currentScale;
       one.coil.control.surfaceJerkPower=jerk;one.mechanics.c1=damping;
       select(one,0);
       run_and_append(one,2,0,'upstream_calibrated_cost_screen');
      end
     end
    end
   end
  end
 case 'upstream_weights'
  % Cost scales change true-force control aggressiveness; the observer's
  % measured-force row is always the physical upstream load-cell row.
  q=mechanics(p,2,catalog);q.mechanics.c1=60;
  for forceScale=[.04 .08 .16 .32 .64]
   for forceAssumed=[.003 .01 .03 .06]
    for currentScale=[.3 1 3]
     for velocityScale=[.03 .1 .3]
      one=q;one.coil.control.forceScale_N=forceScale;
      one.coil.control.forceNoise_N=forceAssumed;
      one.coil.control.currentScale_A=currentScale;
      one.coil.control.velocityScale_m_s=velocityScale;
      select(one,.0002);
      run_and_append(one,2,.0002,'upstream_cost_screen');
     end
    end
   end
  end
 case 'upstream_refine'
  prior=load(fullfile(outputFolder,'controller_comparison_upstream_weights.mat'),'report');
  ranked=prior.report.rows;ranked=ranked([ranked.completed] & [ranked.springSizingPassed]);
  [~,ix]=sort([ranked.peakForceError_N]);ranked=ranked(ix(1:min(8,numel(ix))));
  for j=1:numel(ranked)
   q=upstream_parameters(p,ranked(j),catalog);
   for damping=[60 100 150]
    for jerk=[70 300 1000]
     for integralScale=[.005 .02 .08]
      one=q;one.mechanics.c1=damping;one.coil.control.surfaceJerkPower=jerk;
      one.coil.control.integralScale_Ns=integralScale;
      select(one,.0002);
      run_and_append(one,2,.0002,sprintf('upstream_refine_source%03d',ranked(j).id));
     end
    end
   end
  end
 case 'upstream_shortlist'
  prior=load(fullfile(outputFolder,'controller_comparison_upstream_refine.mat'),'report');
  ranked=prior.report.rows;ranked=ranked([ranked.completed] & [ranked.springSizingPassed]);
  [~,ix]=sort([ranked.peakForceError_N]);ranked=ranked(ix(1:min(4,numel(ix))));
  finalistRuns={};finalists=struct([]);
  for j=1:numel(ranked)
   q=upstream_parameters(p,ranked(j),catalog);startRow=numel(rows)+1;
   worstPeak=-inf;worst=[];
   for stress=[false true]
    if stress,seeds=[7 29 1729 31415];else,seeds=1729;end
    for ref=[5 50]
     for angle=[0 90]
      for seed=seeds
       one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
       one.sensor.noiseRms=.02*stress;one.sensor.seed=seed;
       select(one,.0002*stress);
       r=run_and_append(one,2,.0002*stress,sprintf('upstream_final_source%03d',ranked(j).id));
       if ~isempty(r) && r.metrics.peakAbsForceError_N>worstPeak,worstPeak=r.metrics.peakAbsForceError_N;worst=r;end
      end
     end
    end
   end
   group=rows(startRow:end);entry=struct('sourceId',ranked(j).id,'parameters',q, ...
    'allPassed',all([group.passed]),'worstPeakError_N',worstPeak,'runs',numel(group));
   if isempty(finalists),finalists=entry;else,finalists(end+1)=entry;end
   finalistRuns{end+1}=worst; %#ok<AGROW>
   report.finalists=finalists;checkpoint();
  end
  save(fullfile(outputFolder,'controller_comparison_upstream_finalist_histories.mat'),'finalistRuns','finalists');
 case 'upstream_pid'
  q=mechanics(p,2,catalog);q.mechanics.c1=60;q.coil.strategy='pid';
  q.coil.forceKp=.64;q.coil.forceKi=300;q.coil.forceKd=.0006;q.coil.derivativeFilter=.0002;
  actualCellRuns={};
  for stress=[false true]
   for ref=[5 50]
    for angle=[0 90]
     one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
     one.sensor.noiseRms=.02*stress;one.sensor.seed=7;
     select(one,.0002*stress);
     r=run_and_append(one,2,.0002*stress,'upstream_direct_pid');
     actualCellRuns{end+1}=r; %#ok<AGROW>
    end
   end
  end
  save(fullfile(outputFolder,'controller_comparison_upstream_direct_pid_histories.mat'),'actualCellRuns');
 case 'current_limit_3p5'
  variants={};
  one=mechanics(p,2,catalog);one.mechanics.c1=60;one.coil.strategy='pid';
  one.coil.forceKp=.64;one.coil.forceKi=300;one.coil.forceKd=.0006;one.coil.derivativeFilter=.0002;one.coil.peakCurrent=3.5;
  variants{1}=one;
  one=mechanics(p,3,catalog);one.coil.strategy='observer';one.coil.peakCurrent=3.5;
  one.coil.control.surfaceJerkPower=30;one.coil.control.forceNoise_N=.07;one.coil.control.forceScale_N=.32;
  variants{2}=one;fineEvidence=struct([]);limitRuns={};
  for j=1:numel(variants)
   q=variants{j};mode=double(q.mechanics.lowerRigid)+2*double(q.mechanics.upperRigid);
   startRow=numel(rows)+1;worstCoarse=[];worstPeak=-inf;
   for stress=[false true]
    select(q,.0002*stress);
    if stress,seeds=[7 29 1729 31415];else,seeds=1729;end
    for ref=[5 50]
     for angle=[0 90]
      for seed=seeds
       one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
       one.sensor.noiseRms=.02*stress;one.sensor.seed=seed;
       r=run_and_append(one,mode,.0002*stress,sprintf('limit3p5_mode%d_%s',mode,q.coil.strategy));
       if ~isempty(r) && r.metrics.peakAbsForceError_N>worstPeak,worstPeak=r.metrics.peakAbsForceError_N;worstCoarse=r;end
      end
     end
    end
   end
   coarseRows=rows(startRow:end);assert(~isempty(worstCoarse),'No current-limit run executed.');
   fineQ=worstCoarse.parameters;
   if fineQ.sensor.noiseRms>0,extra=.0002;else,extra=0;end
   fineRuns={};peakErrors=zeros(1,2);historyDifferences=zeros(1,2);
   for k=1:2
    fineQ.sim.step=.0001/(2^k);select(fineQ,extra);set_param(mdl,'FastRestart','off');
    fine=run_and_append(fineQ,mode,extra,sprintf('limit3p5_mode%d_%s_step%.0fus',mode,q.coil.strategy,fineQ.sim.step*1e6));
    assert(~isempty(fine),'A fine current-limit run could not execute.');
    fineRuns{k}=fine;peakErrors(k)=fine.metrics.peakAbsForceError_N;
    ix=round(worstCoarse.time/fineQ.sim.step)+1;
    assert(max(abs(worstCoarse.time-fine.time(ix)))<1e-10);
    historyDifferences(k)=max(abs(worstCoarse.data(:,1)-fine.data(ix,1)));
    fprintf('CURRENT3P5_FINE mode%d %s step%.0fus peak%.9g historyDifference%.9g pass%d\n',mode,q.coil.strategy,fineQ.sim.step*1e6,peakErrors(k),historyDifferences(k),fine.metrics.passed);
   end
   commandIndex=find(strcmp(worstCoarse.names,'actuatorCommand'));assert(~isempty(commandIndex),'No actuator-command signal.');
   cmd=worstCoarse.data(:,commandIndex);
   entry=struct('mode',mode,'strategy',q.coil.strategy,'peakCurrentLimit_A',3.5, ...
    'coarseCornerRuns',20,'coarseAllPassed',all([coarseRows.passed]),'worstCoarseError_N',worstPeak, ...
    'halfStepError_N',peakErrors(1),'quarterStepError_N',peakErrors(2), ...
    'halfStepPassed',fineRuns{1}.metrics.passed,'quarterStepPassed',fineRuns{2}.metrics.passed, ...
    'halfStepHistoryDifference_N',historyDifferences(1),'quarterStepHistoryDifference_N',historyDifferences(2), ...
    'peakCommandCurrent_A',max(abs(cmd)),'commandLimitFraction',mean(abs(cmd)>=3.5-1e-10), ...
    'maxActualPeakCurrent_A',max([coarseRows.peakCurrent_A]),'maxPeakVoltage_V',max([coarseRows.peakVoltage_V]), ...
    'worstParameters',worstCoarse.parameters);
   if isempty(fineEvidence),fineEvidence=entry;else,fineEvidence(end+1)=entry;end
   limitRuns{end+1}=worstCoarse;limitRuns{end+1}=fineRuns{1};limitRuns{end+1}=fineRuns{2}; %#ok<AGROW>
   report.currentLimitEvidence=fineEvidence;checkpoint();
  end
  save(fullfile(outputFolder,'controller_comparison_current_limit_3p5_histories.mat'),'limitRuns','fineEvidence');
  report.currentLimitEvidence=fineEvidence;report.baseParameters.coil.peakCurrent=3.5;
 case 'final_clean_corners'
  input=load(fullfile(outputFolder,'controller_comparison_robust_reselect.mat'),'report');
  cleanRuns={};
  for j=1:numel(input.report.robustWinners)
   winner=input.report.robustWinners(j);q=acf_prepare(winner.parameters);
   q.sensor.noiseRms=0;q.sensor.seed=1729;q.sim.step=.0001;
   select(q,0);set_param(mdl,'FastRestart','off');
   for ref=[5 50]
    for angle=[0 90]
     one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
     r=run_and_append(one,winner.mode,0,sprintf('clean_%s_mode%d',winner.strategy,winner.mode));
     assert(~isempty(r) && r.metrics.passed,'A selected clean corner failed.');
     cleanRuns{end+1}=r; %#ok<AGROW>
    end
   end
  end
  save(fullfile(outputFolder,'controller_comparison_final_clean_histories.mat'),'cleanRuns');
 case 'verify'
  input=load(fullfile(outputFolder,'controller_comparison_robust_reselect.mat'),'report');
  robust=input.report;
  prior=load(fullfile(outputFolder,'controller_comparison_shortlist.mat'),'report');prior=prior.report;
  configs={};labels={};sourceRows={};
  settings=[2 70 .06 .04;2 30 .035 .04;0 30 .06 .08;3 30 .07 .32];
  studyNames={'shortlist_upper_rigid_064','shortlist_upper_rigid_055','shortlist_observer_044','shortlist_rigid_refine_059'};
  for j=1:size(settings,1)
   q=mechanics(p,settings(j,1),catalog);if settings(j,1)==2,q.mechanics.c1=60;end
   q.coil.control.surfaceJerkPower=settings(j,2);q.coil.control.forceNoise_N=settings(j,3);q.coil.control.forceScale_N=settings(j,4);
   configs{end+1}=q;labels{end+1}=studyNames{j}; %#ok<AGROW>
   group=prior.rows(strcmp({prior.rows.study},studyNames{j}));[~,ix]=max([group.peakForceError_N]);sourceRows{end+1}=group(ix); %#ok<AGROW>
  end
  for j=1:numel(robust.robustWinners)
   winner=robust.robustWinners(j);q=acf_prepare(winner.parameters);
   configs{end+1}=q;labels{end+1}=sprintf('reselect_%s_%03d',winner.sourcePhase,winner.sourceId); %#ok<AGROW>
   group=robust.rows(strcmp({robust.rows.study},labels{end}));[~,ix]=max([group.peakForceError_N]);sourceRows{end+1}=group(ix); %#ok<AGROW>
  end
  verified=struct([]);selectedRuns={};
  for j=1:numel(configs)
   q=configs{j};source=sourceRows{j};mode=double(q.mechanics.lowerRigid)+2*double(q.mechanics.upperRigid);
   q.force.reference=20;q.mechanics.angle=0;q.sensor.noiseRms=0;q.sensor.seed=1729;
   select(q,0);set_param(mdl,'FastRestart','off');
   clean=run_and_append(q,mode,0,['ordinary_clean_' labels{j}]);selectedRuns{end+1}=clean; %#ok<AGROW>
   q.force.reference=source.forceReference_N;q.mechanics.angle=source.angle_deg*pi/180;
   q.sensor.noiseRms=.02;q.sensor.seed=source.seed;select(q,.0002);set_param(mdl,'FastRestart','off');
   noisy=run_and_append(q,mode,.0002,['ordinary_worst_' labels{j}]);selectedRuns{end+1}=noisy; %#ok<AGROW>
   assert(~isempty(noisy) && noisy.metrics.passed,'Selected ordinary run failed.');
   restartDifference=abs(noisy.metrics.peakAbsForceError_N-source.peakForceError_N);
   assert(restartDifference<1e-9,'Fast Restart and ordinary run peak errors disagree.');
   q.sim.step=.00005;select(q,.0002);set_param(mdl,'FastRestart','off');
   fine=run_and_append(q,mode,.0002,['halfstep_worst_' labels{j}]);
   assert(~isempty(fine),'Selected half-step run could not execute.');
   selectedRuns{end+1}=fine; %#ok<AGROW>
   halfstepDifference=abs(fine.metrics.peakAbsForceError_N-noisy.metrics.peakAbsForceError_N);
   ix=round((noisy.time-fine.time(1))/q.sim.step)+1;
   assert(max(abs(noisy.time-fine.time(ix)))<1e-10,'Half-step sample grids do not match.');
   forceHistoryDifference=max(abs(noisy.data(:,1)-fine.data(ix,1)));
   % Finer recordings also catch extrema between 100 us output samples.
   % Compare the common-timestamp history separately from its recorded peak.
   fprintf('VERIFIED_SOLVER %s peakDifference %.6g historyDifference %.6g finePeak %.6g finePass%d\n', ...
    labels{j},halfstepDifference,forceHistoryDifference,fine.metrics.peakAbsForceError_N,fine.metrics.passed);
   assert(halfstepDifference<.02 && forceHistoryDifference<.002,'Selected solver comparison exceeded the numerical bounds.');
   entry=struct('label',labels{j},'mode',mode,'strategy',q.coil.strategy, ...
    'cleanPeakError_N',clean.metrics.peakAbsForceError_N,'worstPeakError_N',noisy.metrics.peakAbsForceError_N, ...
    'restartPeakDifference_N',restartDifference,'halfstepPeakDifference_N',halfstepDifference, ...
    'halfstepForceHistoryDifference_N',forceHistoryDifference,'halfstepPassed',fine.metrics.passed, ...
    'halfstepPeakError_N',fine.metrics.peakAbsForceError_N,'parameters',noisy.parameters);
   if isempty(verified),verified=entry;else,verified(end+1)=entry;end
   report.verification=verified;checkpoint();
  end
  save(fullfile(outputFolder,'controller_comparison_selected_histories.mat'),'selectedRuns','verified');
  % Save/reopen only this owned, mutable model copy and compare all 35 signals.
  sample=noisy;q=sample.parameters;select(q,.0002);set_param(mdl,'FastRestart','off');
  before=run_and_append(q,double(q.mechanics.lowerRigid)+2*double(q.mechanics.upperRigid),.0002,'before_private_save');
  save_system(mdl);close_system(mdl,0);load_system(fullfile(worker,[mdl '.slx']));
  after=run_and_append(q,double(q.mechanics.lowerRigid)+2*double(q.mechanics.upperRigid),.0002,'after_private_reopen');
  assert(isequal(before.time,after.time) && isequal(before.data,after.data),'Private saved/reopened signals changed.');
  report.privateSaveReopen35SignalsEqual=true;
  set_param(mdl,'SimulationCommand','update');
  loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);report.algebraicLoops=numel(loops);
  assert(isempty(loops),'A compared native controller introduces an algebraic loop.');
  report.pidNativeBlocks=numel(find_system([mdl '/Force controller/Other actuators PI'],'SearchDepth',1,'Type','Block'))-1;
  report.verification=verified;
 case 'robust_reselect'
  % Re-rank simple controllers by actual worst multi-seed/corner error. The
  % best single seed can conceal noise sensitivity or command saturation.
  studies={'pi_pid','upper_rigid_pi_pid','simple_refine','compliant_simple_refine'};
  allSource=struct([]);search=struct([]);
  for s=1:numel(studies)
   input=load(fullfile(outputFolder,['controller_comparison_' studies{s} '.mat']),'report');
   assert(strcmp(input.report.status,'complete'),'A screening phase is still running.');
   source=input.report.rows;
   for j=1:numel(source),source(j).sourcePhase=studies{s};end
   if isempty(allSource),allSource=source;else,allSource=[allSource source];end %#ok<AGROW>
  end
  winners=struct([]);
  for mode=[3 2 0]
   for strategy={'pi','pid'}
    eligible=find([allSource.mode]==mode & strcmp({allSource.strategy},strategy{1}) & ...
     [allSource.forceOnlyPassed] & [allSource.springSizingPassed]);
    if isempty(eligible),continue;end
    [~,order]=sort([allSource(eligible).peakForceError_N]);eligible=eligible(order);
    bound=.5;winner=[];
    for u=eligible
     source=allSource(u);q=parameters_from_row(mechanics(p,mode,catalog),source);select(q,.0002);
     label=sprintf('reselect_%s_%03d',source.sourcePhase,source.id);
     entry=struct('sourcePhase',source.sourcePhase,'sourceId',source.id,'mode',mode, ...
      'strategy',q.coil.strategy,'Kp_A_N',q.coil.forceKp,'Ki_A_Ns',q.coil.forceKi, ...
      'Kd_As_N',q.coil.forceKd,'derivativeFilter_s',q.coil.derivativeFilter, ...
      'testedCases',0,'testedAll16',false,'qualified',false,'worstObservedError_N',NaN, ...
      'pruned',false,'reason','pending');
     reject=false;
     for ref=[5 50]
      if reject,break;end
      for angle=[0 90]
       if reject,break;end
       for seed=[7 29 1729 31415]
        one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;one.sensor.seed=seed;one.sensor.noiseRms=.02;
        run_and_append(one,mode,.0002,label);last=rows(end);entry.testedCases=entry.testedCases+1;
        if isfinite(last.peakForceError_N)
         if isnan(entry.worstObservedError_N),entry.worstObservedError_N=last.peakForceError_N;
         else,entry.worstObservedError_N=max(entry.worstObservedError_N,last.peakForceError_N);end
        end
        if ~last.passed
         reject=true;entry.reason=['Rejected by an executed case: ' last.reason];break;
        elseif entry.worstObservedError_N>bound+1e-12
         reject=true;entry.pruned=true;entry.reason='An executed case already exceeds the fully tested winner for this topology and strategy.';break;
        end
       end
      end
     end
     if ~reject
      entry.testedAll16=true;entry.qualified=true;entry.reason='All four force/orientation corners and four noise seeds passed.';
      bound=entry.worstObservedError_N;winner=entry;winner.parameters=acf_prepare(q);
      fprintf('ROBUST_SIMPLE_WINNER mode%d %s Kp%.6g Ki%.6g Kd%.6g filter%.6g worst%.6g\n',mode,q.coil.strategy,q.coil.forceKp,q.coil.forceKi,q.coil.forceKd,q.coil.derivativeFilter,bound);
     end
     if isempty(search),search=entry;else,search(end+1)=entry;end
     report.candidateSearch=search;checkpoint();
    end
    if ~isempty(winner)
     if isempty(winners),winners=winner;else,winners(end+1)=winner;end
    end
   end
  end
  report.robustWinners=winners;report.candidateSearch=search;
 case 'compliant_simple_refine'
  for mode=[2 0]
   q=mechanics(p,mode,catalog);q.coil.strategy='pi';
   if mode==2,q.mechanics.c1=60;end
   select(q,.0002);
   for kp=[.32 .48 .64]
    for ki=[50 100 200 300]
     one=q;one.coil.forceKp=kp;one.coil.forceKi=ki;one.coil.forceKd=0;
     run_and_append(one,mode,.0002,'compliant_PI_refine');
    end
   end
   for kp=[.24 .32 .48 .64]
    for ki=[30 75 150 300]
     for kd=[1e-4 3e-4 6e-4]
      for filter=[.0002 .0005]
       one=q;one.coil.strategy='pid';one.coil.forceKp=kp;one.coil.forceKi=ki;
       one.coil.forceKd=kd;one.coil.derivativeFilter=filter;
       run_and_append(one,mode,.0002,'compliant_PID_refine');
      end
     end
    end
   end
  end
 case 'simple_refine'
  q=mechanics(p,3,catalog);q.coil.strategy='pi';select(q,.0002);
  for kp=[.05 .065 .08 .1 .12]
   for ki=[50 75 100 150 200]
    one=q;one.coil.forceKp=kp;one.coil.forceKi=ki;one.coil.forceKd=0;
    run_and_append(one,3,.0002,'rigid_PI_refine');
   end
  end
  for kp=[.06 .08 .1]
   for ki=[75 125 175]
    for kd=[1e-5 3e-5 5e-5]
     for filter=[.0001 .0002 .0004]
      one=q;one.coil.strategy='pid';one.coil.forceKp=kp;one.coil.forceKi=ki;
      one.coil.forceKd=kd;one.coil.derivativeFilter=filter;
      run_and_append(one,3,.0002,'rigid_PID_refine');
     end
    end
   end
  end
 case 'rigid_refine'
  q=mechanics(p,3,catalog);select(q,.0002);
  for jerk=[1 3 10 30]
   for assumedNoise=[.03 .045 .07 .1]
    for forceScale=[.02 .08 .32 1]
     one=q;one.coil.control.surfaceJerkPower=jerk;one.coil.control.forceNoise_N=assumedNoise;
     one.coil.control.forceScale_N=forceScale;
     run_and_append(one,3,.0002,'rigid_weight_refine');
    end
   end
  end
 case 'upper_rigid'
  for damping=[15 30 60]
   q=mechanics(p,2,catalog);q.mechanics.c1=damping;select(q,.0002);
   for jerk=[10 30 70 150]
    for assumedNoise=[.035 .06]
     for forceScale=[.04 .08 .16]
      one=q;one.coil.control.surfaceJerkPower=jerk;one.coil.control.forceNoise_N=assumedNoise;
      one.coil.control.forceScale_N=forceScale;
      run_and_append(one,2,.0002,'upper_rigid_weight_screen');
     end
    end
   end
  end
 case 'observer'
  for mode=[3 0]
   q=mechanics(p,mode,catalog);
   select(q,.0002);
   if mode==3,jerks=[10 30 70 150];noiseWeights=[.02 .035 .06];forceWeights=[.04 .08 .16];
   else,jerks=[30 70 150];noiseWeights=[.02 .035 .06];forceWeights=[.04 .08 .16];end
   for jerk=jerks
    for assumedNoise=noiseWeights
     for forceScale=forceWeights
      one=q;one.coil.control.surfaceJerkPower=jerk;one.coil.control.forceNoise_N=assumedNoise;
      one.coil.control.forceScale_N=forceScale;
      run_and_append(one,mode,.0002,'weight_screen');
     end
    end
   end
  end
 case {'pi_pid','upper_rigid_pi_pid'}
  if strcmp(phase,'upper_rigid_pi_pid'),modes=2;else,modes=[3 0];end
  for mode=modes
   q=mechanics(p,mode,catalog);q.coil.strategy='pi';
   if mode==2,q.mechanics.c1=60;end
   select(q,.0002);
   for kp=[.01 .02 .04 .08 .16 .32]
    for ki=[1 3 10 30 100 300]
     one=q;one.coil.forceKp=kp;one.coil.forceKi=ki;one.coil.forceKd=0;
     run_and_append(one,mode,.0002,'PI_screen');
    end
   end
   for kp=[.02 .04 .08 .16]
    for ki=[3 10 30 100]
     for kd=[3e-5 1e-4 3e-4]
      for filter=[.0002 .0005]
       one=q;one.coil.strategy='pid';one.coil.forceKp=kp;one.coil.forceKi=ki;
       one.coil.forceKd=kd;one.coil.derivativeFilter=filter;
       run_and_append(one,mode,.0002,'PID_screen');
      end
     end
    end
   end
  end
 case {'shortlist','observer_shortlist'}
  if strcmp(phase,'observer_shortlist')
   studies={'observer','rigid_refine','upper_rigid'};keep=1;
  else
   studies={'observer','rigid_refine','upper_rigid','pi_pid','upper_rigid_pi_pid','simple_refine'};keep=3;
  end
  candidates={};
  allSource=struct([]);
  for s=1:numel(studies)
   path=fullfile(outputFolder,['controller_comparison_' studies{s} '.mat']);
   if ~isfile(path),continue;end
   input=load(path,'report');assert(strcmp(input.report.status,'complete'),'A screening phase is still running.');
   source=input.report.rows;
   for j=1:numel(source),source(j).sourcePhase=studies{s};end
   if isempty(allSource),allSource=source;else,allSource=[allSource source];end %#ok<AGROW>
  end
  for mode=[3 0 2]
   for strategy={'observer','pi','pid'}
    % A candidate that already fails the screening corner cannot qualify the
    % full operating range; retain its recorded screening evidence and spend
    % multi-seed qualification on candidates that met that corner's target.
    usable=find([allSource.mode]==mode & strcmp({allSource.strategy},strategy{1}) & ...
     [allSource.executed] & [allSource.completed] & [allSource.forceOnlyPassed] & [allSource.springSizingPassed]);
    if isempty(usable),continue;end
    [~,order]=sort([allSource(usable).peakForceError_N]);usable=usable(order(1:min(keep,numel(order))));
    for u=usable
     one=mechanics(p,mode,catalog);one=parameters_from_row(one,allSource(u));
     candidates{end+1}=struct('parameters',one,'mode',mode, ...
      'sourcePhase',allSource(u).sourcePhase,'sourceId',allSource(u).id); %#ok<AGROW>
    end
   end
  end
  report.shortlistSource=cellfun(@(c)struct('phase',c.sourcePhase,'id',c.sourceId,'mode',c.mode),candidates);
  for c=1:numel(candidates)
   candidate=candidates{c};q=candidate.parameters;
   for stress=[false true]
    select(q,.0002*stress);
    if stress,seeds=[7 29 1729 31415];else,seeds=1729;end
    for ref=[5 50]
     for angle=[0 90]
      for seed=seeds
       one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
       one.sensor.seed=seed;one.sensor.noiseRms=.02*stress;
       label=sprintf('shortlist_%s_%03d',candidate.sourcePhase,candidate.sourceId);
       r=run_and_append(one,candidate.mode,.0002*stress,label);
       if ~isempty(r)
        traceFile=fullfile(worker,'traces',sprintf('%s_%04d.mat',phase,runs));save(traceFile,'r');
       end
      end
     end
    end
   end
  end
 otherwise,error('Unknown controller comparison phase: %s',phase);
end
report.status='complete';checkpoint();
fprintf('CONTROLLER_COMPARISON_COMPLETE %s: %d runs, %d pass, best %.6g N, %.1f seconds\n',phase,runs,sum([rows.passed]),bestScore,toc(started));

 function select(q,extraDelay)
  set_param(mdl,'FastRestart','off');
  assignin(get_param(mdl,'ModelWorkspace'),'p',acf_prepare(q));acf_select_actuator(mdl,'voice_coil');
  set_param(delayBlock,'DelayLength',sprintf('p.d.sensorDelaySamples+%d',round(extraDelay/q.sensor.sampleTime)));
  set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
  set_param(mdl,'FastRestart','on');
 end

 function r=run_and_append(q,mode,extraDelay,label)
  [row,r]=one_run(mdl,q,mode,extraDelay,label,runs+1);
  if isempty(rows),rows=row;else,rows(end+1)=row;end
  runs=runs+1;
  if row.completed && row.springSizingPassed && row.peakForceError_N<bestScore
   bestScore=row.peakForceError_N;bestRun=r;bestParameters=r.parameters;
   fprintf('CONTROLLER_NEW_BEST %s mode=%d score=%.6g N Kp=%.6g Ki=%.6g Kd=%.6g Qjerk=%.6g Rforce=%.6g forceScale=%.6g\n', ...
    q.coil.strategy,mode,bestScore,q.coil.forceKp,q.coil.forceKi,q.coil.forceKd, ...
    q.coil.control.surfaceJerkPower,q.coil.control.forceNoise_N,q.coil.control.forceScale_N);
  end
  if mod(runs,8)==0,checkpoint();Simulink.sdi.clear;end
 end

 function checkpoint()
  report.rows=rows;report.attemptedCases=runs;report.simulations=sum([rows.executed]);
  report.passed=sum([rows.passed]);report.elapsedSeconds=toc(started);
  report.bestScore_N=bestScore;report.bestParameters=bestParameters;
  report.bestSingleRunPeakError_N=bestScore;
  path=fullfile(outputFolder,['controller_comparison_' phase '.json']);
  fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  if ~isempty(rows),writetable(struct2table(rows),fullfile(outputFolder,['controller_comparison_' phase '.csv']));end
  save(fullfile(outputFolder,['controller_comparison_' phase '.mat']),'report','bestRun','bestParameters');
  if ~isempty(rows)
   fprintf('CONTROLLER_PROGRESS %s: %d runs, %d pass, last %.6g N, %.1f seconds\n',phase,runs,sum([rows.passed]),rows(end).peakForceError_N,toc(started));
  end
 end
end

function q=mechanics(q,mode,catalog)
q.mechanics.lowerRigid=bitand(mode,1)>0;q.mechanics.upperRigid=bitand(mode,2)>0;
q.springs.lower=catalog.springs(4);q.springs.upper=catalog.springs(4);q.springs.commonPreload_N=32;
q.mechanics.k1=2*q.springs.lower.rate_N_m;q.mechanics.k2=2*q.springs.upper.rate_N_m;
q.mechanics.c1=30;q.mechanics.c2=5;
if q.mechanics.lowerRigid,q.mechanics.k1=0;q.mechanics.c1=0;end
if q.mechanics.upperRigid,q.mechanics.k2=0;q.mechanics.c2=0;end
end

function ensure_worker(snapshot,worker)
% Mutations are confined to this private copy, including controller selection.
helperFolder=fullfile(worker,'helpers');
if ~isfolder(helperFolder),mkdir(helperFolder);copyfile(fullfile(snapshot,'helpers','*.m'),helperFolder);end
modelFile=fullfile(worker,'acf_force_control_simple.slx');
if ~isfile(modelFile),copyfile(fullfile(snapshot,'acf_force_control_simple.slx'),modelFile);end
selector=fullfile(helperFolder,'acf_select_actuator.m');source=fileread(selector);
if ~contains(source,'p.coil.strategy')
 old="if index==2,controlChoice='Voice coil observer';else,controlChoice='Other actuators PI';end";
 replacement=sprintf(['if index==2 && (~isfield(p.coil,''strategy'') || strcmp(p.coil.strategy,''observer''))\n' ...
  '        controlChoice=''Voice coil observer'';\n    else\n        controlChoice=''Other actuators PI'';\n    end']);
 assert(contains(source,old),'Private selector does not match the snapshot.');
 source=replace(source,old,replacement);fid=fopen(selector,'w');assert(fid>=0);fprintf(fid,'%s',source);fclose(fid);
end
end

function q=parameters_from_row(q,row)
q.coil.strategy=row.strategy;q.coil.forceKp=row.Kp_A_N;q.coil.forceKi=row.Ki_A_Ns;
q.coil.forceKd=row.Kd_As_N;q.coil.derivativeFilter=row.derivativeFilter_s;
q.coil.control.surfaceJerkPower=row.surfaceJerkPower;
q.coil.control.forceNoise_N=row.observerForceNoise_N;q.coil.control.forceScale_N=row.forceScale_N;
q.coil.control.integralScale_Ns=row.integralScale_Ns;
q.coil.control.currentScale_A=row.currentScale_A;
q.mechanics.c1=row.c1_Ns_m;q.mechanics.c2=row.c2_Ns_m;
end

function q=upstream_parameters(q,row,catalog)
q=mechanics(q,row.mode,catalog);q=parameters_from_row(q,row);
q.coil.control.velocityScale_m_s=row.velocityScale_m_s;
q.coil.control.strokeNoise_m=row.observerStrokeNoise_m;
end

function ensure_upstream_worker(snapshot,worker,rigidCoulomb)
% Reuse the tested predictor architecture in a separately owned copy. The
% source phase snapshot is immutable once saved; cache/SDI data are not copied.
if rigidCoulomb,origin=fullfile(snapshot,'robot_predictor_worker','phase_tool_jerk_sanity');
else,origin=fullfile(snapshot,'robot_predictor_worker','phase_timing_bounded');end
model=fullfile(worker,'acf_force_control_simple.slx');
if ~isfolder(worker),mkdir(worker);end
if ~isfolder(fullfile(worker,'helpers'))
 assert(isfolder(fullfile(origin,'helpers')),'Issued-command predictor snapshot is not available.');
 mkdir(fullfile(worker,'helpers'));copyfile(fullfile(origin,'helpers','*.m'),fullfile(worker,'helpers'));
end
if ~isfile(model)
 assert(isfile(fullfile(origin,'acf_robot_prediction_experiment.slx')),'Predictor model snapshot is not available.');
 copyfile(fullfile(origin,'acf_robot_prediction_experiment.slx'),model);
 if rigidCoulomb,name='qualify_robot_predictor_tool_jerk_sanity_source.m';
 else,name='qualify_robot_predictor_timing_bounded_source.m';end
 copyfile(fullfile(origin,name),fullfile(worker,'predictor_authorship.m'));
end
runHelper=fullfile(worker,'helpers','acf_run.m');source=fileread(runHelper);
if ~contains(source,'r.observerStates')
 old='r.parameters=p; r.names=acf_signals; r.metrics=acf_metrics(r);';
 replacement=[old newline ...
  'if any(strcmp(who(out),''robot_prediction_observer_log''))' newline ...
  ' logged=out.get(''robot_prediction_observer_log'');r.observerStates=struct(''time'',logged.Time(:),''data'',rows(logged,numel(p.d.coilControl.force)));' newline ...
  'end' newline ...
  'if any(strcmp(who(out),''robot_prediction_log''))' newline ...
  ' logged=out.get(''robot_prediction_log'');r.commandPrediction=struct(''time'',logged.Time(:),''data'',rows(logged,3));' newline ...
  'end'];
 assert(contains(source,old),'Private observer-log patch anchor is missing.');
 source=strrep(source,old,replacement);fid=fopen(runHelper,'w');assert(fid>=0);fprintf(fid,'%s',source);fclose(fid);
end
if rigidCoulomb
 helper=fullfile(worker,'helpers','acf_prepare.m');source=fileread(helper);
 if ~contains(source,'drive.directCoulombCorrection')
  old='predict.surfaceJerkDecayTime,predict.surfaceJerkFeedforwardGain];';
  assert(contains(source,old));source=strrep(source,old,'predict.surfaceJerkDecayTime,predict.surfaceJerkFeedforwardGain,drive.directCoulombCorrection];');
  old='filterRow=zeros(1,N);filterRow(1:np)=(1-a)*CfMeasured;filterRow(fi)=a;Aa(fi,:)=filterRow;';
  replacement=[old newline ...
   'if m.lowerRigid && drive.directCoulombCorrection' newline ...
   ' assert(p.d.sensorDelaySamples>=1,''Direct Coulomb correction requires at least one configured force-delay sample.'');' newline ...
   ' Ba(fi,2)=-(1-a);' newline ...
   'end'];
  assert(contains(source,old));source=strrep(source,old,replacement);
  fid=fopen(helper,'w');assert(fid>=0);fprintf(fid,'%s',source);fclose(fid);
 end
end
end

function add_native_coulomb_correction(mdl)
root=[mdl '/Force controller/Voice coil observer'];
set_param([root '/State and disturbance observer'],'D','zeros(9,5)');
set_param([root '/Restore total robot acceleration'],'Gain','[zeros(8,1);1]');
set_param([root '/Estimated gap velocity'],'InputPortWidth','9');
set_param([root '/Estimated coil gap deviation'],'InputPortWidth','9');
if getSimulinkBlockHandle([root '/Direct friction in contact force'])==-1
 add_block('simulink/Math Operations/Gain',[root '/Direct friction in contact force'], ...
  'Gain','double(p.mechanics.lowerRigid && p.coil.directCoulombCorrection)', ...
  'Position',[960 835 1170 880]);
 add_block('simulink/Math Operations/Sum',[root '/Physical estimated force deviation'], ...
  'Inputs','+-','Position',[1250 785 1285 875]);
 delete_line(root,'Estimated true force error/1','Integral and anti windup/1');
 add_line(root,'Estimated true force error/1','Physical estimated force deviation/1','autorouting','on');
 add_line(root,'Receive friction for observer/1','Direct friction in contact force/1','autorouting','on');
 add_line(root,'Direct friction in contact force/1','Physical estimated force deviation/2','autorouting','on');
 add_line(root,'Physical estimated force deviation/1','Integral and anti windup/1','autorouting','on');
end
end

function add_native_contact_gate(mdl)
% The slow loop receives the same reconstructed physical contact-force
% deviation used by the fast loop, with error sign reference minus force.
root=[mdl '/Force controller/Voice coil observer'];
if getSimulinkBlockHandle([root '/Reconstructed contact error'])==-1
 add_block('simulink/Math Operations/Gain',[root '/Reconstructed contact error'], ...
  'Gain','-1','Position',[1330 800 1435 855]);
 add_block('simulink/Signal Routing/Goto',[root '/Send reconstructed contact error'], ...
  'GotoTag','inferred_contact_error_N','TagVisibility','global','Position',[1490 810 1690 855]);
 add_line(root,'Physical estimated force deviation/1','Reconstructed contact error/1','autorouting','on');
 add_line(root,'Reconstructed contact error/1','Send reconstructed contact error/1','autorouting','on');
end
set_param([mdl '/Receive force error for robot settling'],'GotoTag','inferred_contact_error_N');
assert(strcmp(get_param([mdl '/Receive inferred actuator travel'],'GotoTag'),'inferred_actuator_travel'), ...
 'The slow recenter loop must use estimated gap, not actual actuator travel.');
end

function [row,r]=one_run(mdl,q,mode,unknownDelay,label,id)
row=struct('id',id,'study',label,'mode',mode,'strategy',q.coil.strategy, ...
 'forceReference_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi, ...
 'k1_N_m',q.mechanics.k1,'k2_N_m',q.mechanics.k2,'c1_Ns_m',q.mechanics.c1,'c2_Ns_m',q.mechanics.c2, ...
 'springPreload_N',q.springs.commonPreload_N,'Kp_A_N',q.coil.forceKp,'Ki_A_Ns',q.coil.forceKi, ...
 'Kd_As_N',q.coil.forceKd,'derivativeFilter_s',q.coil.derivativeFilter, ...
 'surfaceJerkPower',q.coil.control.surfaceJerkPower,'observerForceNoise_N',q.coil.control.forceNoise_N, ...
 'forceScale_N',q.coil.control.forceScale_N,'integralScale_Ns',q.coil.control.integralScale_Ns, ...
 'currentScale_A',q.coil.control.currentScale_A,'velocityScale_m_s',q.coil.control.velocityScale_m_s, ...
 'observerStrokeNoise_m',q.coil.control.strokeNoise_m,'forceNoiseRms_N',q.sensor.noiseRms, ...
 'strokeNoiseRms_m',q.sensor.strokeNoiseRms,'strokeKnownDelay_s',q.sensor.strokeDelay, ...
 'currentResponse_s',q.coil.currentResponse,'controllerSampleTime_s',q.force.sampleTime, ...
 'coulombCorrectionEnabled',optional_field(q.coil,'directCoulombCorrection',false), ...
 'positionTarget',optional_field(q.sensor,'positionTarget','magnet_gap'), ...
 'forceBias_N',q.sensor.bias,'gateReadyFraction',NaN,'gateReadyTransitions',NaN, ...
 'forceKnownDelay_s',q.sensor.delay,'forceUnknownDelay_s',unknownDelay,'seed',q.sensor.seed, ...
 'executed',false,'passed',false,'forceOnlyPassed',false,'completed',false,'springSizingPassed',false, ...
 'peakForceError_N',NaN,'margin_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN, ...
 'peakVoltage_V',NaN,'peakGapAcceleration_m_s2',NaN,'travelMin_mm',NaN,'travelMax_mm',NaN, ...
 'lostContact',false,'strokeStopHit',false,'freeSpringLengthMax_mm',NaN,'installedSpringLengthMax_mm',NaN,'reason','pending');
r=[];
try
 quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
 m=r.metrics;row.executed=true;row.completed=m.completed;row.passed=m.passed;
 row.peakForceError_N=m.peakAbsForceError_N;row.margin_N=q.force.tolerance-row.peakForceError_N;
 row.forceOnlyPassed=m.completed && row.peakForceError_N<=q.force.tolerance && ~m.lostContact && ~m.strokeStopHit && ~m.overCurrent;
 row.peakCurrent_A=m.peakCurrent_A;row.rmsCurrent_A=m.rmsCurrent_A;row.peakVoltage_V=m.peakVoltage_V;
 row.peakGapAcceleration_m_s2=m.peakGapAcceleration_m_s2;row.travelMin_mm=m.minTravel_mm;row.travelMax_mm=m.maxTravel_mm;
 row.lostContact=m.lostContact;row.strokeStopHit=m.strokeStopHit;row.springSizingPassed=m.springSizing.passed;
 row.freeSpringLengthMax_mm=m.springSizing.combinedFreeLengthMax_mm;
 row.installedSpringLengthMax_mm=m.springSizing.combinedInstalledLengthMax_mm;row.reason=m.reason;
 if isfield(r,'robotGate') && isfield(r.robotGate,'ready')
  ready=logical(r.robotGate.ready);row.gateReadyFraction=mean(ready);
  row.gateReadyTransitions=sum(diff(ready)>0);
 end
catch err
 row.reason=['simulation or preparation error: ' err.message];
 fprintf('CONTROLLER_ERROR %d %s\n',id,err.message);
end
end

function value=optional_field(s,name,default)
if isfield(s,name),value=s.(name);else,value=default;end
end

function add_filtered_derivative(mdl)
root=[mdl '/Force controller/Other actuators PI'];block=[root '/Filtered derivative'];
if getSimulinkBlockHandle(block)==-1
 add_block('simulink/Discrete/Discrete Transfer Fcn',block,'Position',[280 300 450 350], ...
  'Numerator','(1-exp(-p.force.sampleTime/p.coil.derivativeFilter))/p.force.sampleTime*[1 -1]', ...
  'Denominator','[1 -exp(-p.force.sampleTime/p.coil.derivativeFilter)]', ...
  'SampleTime','p.force.sampleTime','InitialStates','0');
 sumBlock=[root '/PI plus holding command'];set_param(sumBlock,'Inputs','++++');
 add_line(root,'Force controller sampling/1','Filtered derivative/1','autorouting','on');
end
% Keep the derivative filter's direct-feedthrough status constant when Kd=0.
% A separate Gain can be tuned between zero and nonzero during Fast Restart.
set_param(block,'Numerator','(1-exp(-p.force.sampleTime/p.coil.derivativeFilter))/p.force.sampleTime*[1 -1]');
gain=[root '/Derivative gain'];
if getSimulinkBlockHandle(gain)==-1
 try,delete_line(root,'Filtered derivative/1','PI plus holding command/4');catch,end
 add_block('simulink/Math Operations/Gain',gain,'Gain','p.coil.forceKd','Position',[480 307 550 343]);
 add_line(root,'Filtered derivative/1','Derivative gain/1','autorouting','on');
 add_line(root,'Derivative gain/1','PI plus holding command/4','autorouting','on');
end
end

function restore_model(mdl,delayBlock,normalDelay)
if bdIsLoaded(mdl)
 set_param(mdl,'FastRestart','off');set_param(delayBlock,'DelayLength',normalDelay);
 close_system(mdl,0);
end
Simulink.sdi.clear;
end

function report=current_uncertainty_study(snapshot,output)
% Bounded final test of model uncertainty, using the audited root snapshot.
% Physical force-gain changes affect only the native drive. Observer Kf stays20.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
maxNumCompThreads(min(2,maxNumCompThreads));
origin=fullfile(snapshot,'root-final-two-sensor-100ms');
worker=fullfile(snapshot,'controller_current_uncertainty_worker');
helpers=fullfile(worker,'helpers');model=fullfile(worker,'acf_force_control_simple.slx');
if ~isfolder(worker),mkdir(worker);end
if ~isfolder(helpers),mkdir(helpers);copyfile(fullfile(origin,'helpers','*.m'),helpers);end
if ~isfile(model),copyfile(fullfile(origin,'acf_force_control_simple.slx'),model);end
patch_uncertainty_helper(fullfile(helpers,'acf_prepare.m'));
runHelper=fullfile(helpers,'acf_run.m');source=fileread(runHelper);
source=strrep(source,'rows(logged,9)','rows(logged,numel(p.d.coilControl.force))');
fid=fopen(runHelper,'w');assert(fid>=0);fprintf(fid,'%s',source);fclose(fid);
copyfile([mfilename('fullpath') '.m'],fullfile(worker,'controller_authorship.m'));
addpath(helpers,'-begin');
for name={'cache','code','sdi','traces'},folder=fullfile(worker,name{1});if ~isfolder(folder),mkdir(folder);end,end
Simulink.fileGenControl('set','CacheFolder',fullfile(worker,'cache'), ...
 'CodeGenFolder',fullfile(worker,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(worker,'sdi'));Simulink.sdi.setRecordData(true);
input=load(fullfile(output,'final_two_sensor_candidate.mat'),'p');base=input.p;
assert(base.mechanics.lowerRigid && base.mechanics.upperRigid && base.coil.forceConstant==20);
assert(base.force.sampleTime==.0001 && base.robot.forceGate.filterTime==.1);
base.force.reference=50;base.mechanics.angle=pi/2;base.sensor.noiseRms=.02;
base.sensor.strokeNoiseRms=1e-6;base.sensor.bias=0;base.sim.step=.00005;
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.actuator.kind='voice_coil';
base.coil.control.currentModelNoisePower_A2_s=0;
base.coil.control.forceBiasEnabled=false;base.coil.control.forceBiasNoisePower_N2_s=0;
base.currentUncertaintyActual=struct('forceFactor',1,'currentTauFactor',1);
mdl='acf_force_control_simple';assert(~bdIsLoaded(mdl));load_system(model);
assert(strcmp(get_param(mdl,'FileName'),model));
set_param(mdl,'FastRestart','off');cleanup=onCleanup(@()finish_uncertainty(mdl)); %#ok<NASGU>
drive=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil'];
set_param([drive '/Coil force constant'],'Gain','p.coil.forceConstant*p.currentUncertaintyActual.forceFactor');
set_param([drive '/Back EMF'],'Gain','p.coil.forceConstant*p.currentUncertaintyActual.forceFactor');
set_param([drive '/Current regulator'],'Gain','p.coil.inductance/(p.coil.currentResponse*p.currentUncertaintyActual.currentTauFactor)');
mw=get_param(mdl,'ModelWorkspace');rows=struct([]);groups=struct([]);traces={};started=tic;
report=struct('status','running','date','2026-09-30','phase','current_model_noise', ...
 'privateModel',model,'sourceModel',fullfile(origin,'acf_force_control_simple.slx'), ...
 'scope','Actual B/S load-cell, distal1.4kg, S-A1um, force0.02N,10kHz,100us configured sensors/filter/current response; corrected Coulomb reconstruction and100ms slow contact-force gate. Fixed control weights. Physical Kf errors never enter outer model.', ...
 'baseParameters',base,'rows',rows,'groups',groups);
passedCandidate=[];passedPeak=inf;passedPower=[];passedKind='';
for kind={'current_model_noise','force_bias_state'}
 phase=kind{1};
 if strcmp(phase,'force_bias_state')
  if ~isempty(passedCandidate),break;end
  powers=[.01 .1 1];
 else,powers=[1e-5 1e-4 1e-3 .01 .1 1];end
 rows=struct([]);groups=struct([]);traces={};report.phase=phase;report.status='running';
 for power=powers
  q=base;
  if strcmp(phase,'current_model_noise'),q.coil.control.currentModelNoisePower_A2_s=power;
  else,q.coil.control.forceBiasEnabled=true;q.coil.control.forceBiasNoisePower_N2_s=power;end
  first=numel(rows)+1;worst=[];peak=-inf;
  for factor=[1 1.001 .999]
   for seed=[7 1729]
    one=q;one.sensor.seed=seed;one.currentUncertaintyActual.forceFactor=factor;
    r=run_case(one,0,'gain_uncertainty_screen');append(r,one,power,phase);
    if r.metrics.peakAbsForceError_N>peak,peak=r.metrics.peakAbsForceError_N;worst=r;end
   end
  end
  cases=rows(first:end);g=struct('kind',phase,'noisePower',power, ...
   'executedCases',6,'allPassed',all([cases.passed]),'allCompleted',all([cases.completed]), ...
   'worstPeakError_N',peak,'peakCurrent_A',max([cases.peakCurrent_A]), ...
   'peakVoltage_V',max([cases.peakVoltage_V]));
  if isempty(groups),groups=g;else,groups(end+1)=g;end %#ok<AGROW>
  trace=fullfile(worker,'traces',sprintf('%s_power%.5g.mat',phase,power));save(trace,'worst');
  traces{end+1}=trace; %#ok<AGROW>
  if g.allPassed && peak<passedPeak
   passedPeak=peak;passedCandidate=q;passedPower=power;passedKind=phase;
  end
  checkpoint();clear worst r;Simulink.sdi.clear;
 end
 report.status='complete';checkpoint();
 fprintf('CURRENT_UNCERTAINTY_PHASE_COMPLETE %s: %d runs, %d pass; acceptable groups%d\n', ...
  phase,numel(rows),sum([rows.passed]),sum([groups.allPassed]));
end
report.passedCandidate=passedCandidate;
if ~isempty(passedCandidate)
 p=passedCandidate;save(fullfile(output,'controller_current_uncertainty_candidate.mat'),'p');
 set_param(mdl,'FastRestart','off');assignin(mw,'p',acf_prepare(p));set_dimensions(p);
 set_param([mdl '/Force measurement/Sensor sample delay'],'DelayLength','p.d.sensorDelaySamples');
 save_system(mdl,model);
 report.winningPower=passedPower;report.winningKind=passedKind;
 % Bounded follow-up: remaining corners/noise seeds and explicit mismatches.
 rows=struct([]);groups=struct([]);traces={};report.phase='bounded_qualification';report.status='running';
 conditions={};
 for ref=[5 50]
  for angle=[0 90]
   if ref==50 && angle==90,continue;end
   for seed=7
    one=p;one.force.reference=ref;one.mechanics.angle=angle*pi/180;one.sensor.seed=seed;
    conditions{end+1}=struct('p',one,'extraDelay',0,'label','other_corner'); %#ok<AGROW>
   end
  end
 end
 for seed=[29 31415]
  one=p;one.sensor.seed=seed;conditions{end+1}=struct('p',one,'extraDelay',0,'label','extra_noise_seed'); %#ok<AGROW>
 end
 for massFactor=[.999 1.001]
  one=p;one.sensor.seed=7;one.sensor.distalMass_model_kg=1.4*massFactor;
  conditions{end+1}=struct('p',one,'extraDelay',0,'label','model_distal_mass_0p1pct'); %#ok<AGROW>
 end
 for bias=[-.005 .005]
  one=p;one.sensor.seed=7;one.sensor.bias=bias;
  conditions{end+1}=struct('p',one,'extraDelay',0,'label','force_bias_5mN'); %#ok<AGROW>
 end
 one=p;one.sensor.seed=7;
 conditions{end+1}=struct('p',one,'extraDelay',.0001,'label','unknown_extra_force_delay_100us');
 for tauFactor=[.99 1.01]
  one=p;one.sensor.seed=7;one.currentUncertaintyActual.currentTauFactor=tauFactor;
  conditions{end+1}=struct('p',one,'extraDelay',0,'label','actual_current_tau_1pct'); %#ok<AGROW>
 end
 assert(numel(conditions)==12);
 for j=1:numel(conditions)
  c=conditions{j};r=run_case(c.p,c.extraDelay,c.label);append(r,c.p,passedPower,passedKind);
  if j==numel(conditions),save(fullfile(worker,'traces','bounded_last.mat'),'r');end
 end
 report.status='complete';checkpoint();
end
fprintf('CURRENT_UNCERTAINTY_COMPLETE: acceptableCandidate%d, elapsed%.1fs\n',~isempty(passedCandidate),toc(started));

 function set_dimensions(q)
  root=[mdl '/Force controller/Voice coil observer'];np=9+double(q.coil.control.forceBiasEnabled);
  set_param([root '/State and disturbance observer'],'D',sprintf('zeros(%d,5)',np));
  set_param([root '/Restore total robot acceleration'],'Gain',sprintf('[zeros(8,1);1;zeros(%d,1)]',np-9));
  set_param([root '/Estimated gap velocity'],'InputPortWidth',num2str(np));
  set_param([root '/Estimated gap displacement'],'InputPortWidth',num2str(np));
 end
 function r=run_case(q,extra,label)
  set_param(mdl,'FastRestart','off');prepared=acf_prepare(q);assignin(mw,'p',prepared);set_dimensions(q);
  acf_select_actuator(mdl,'voice_coil');
  set_param([mdl '/Force measurement/Sensor sample delay'],'DelayLength', ...
   sprintf('p.d.sensorDelaySamples+%d',round(extra/q.sensor.sampleTime)));
  set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
  set_param(mdl,'FastRestart','on');
  quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  r.study=label;
 end
 function append(r,q,power,kind)
  m=r.metrics;row=struct('id',numel(rows)+1,'kind',kind,'power',power,'study',r.study, ...
   'actualKf_N_A',20*q.currentUncertaintyActual.forceFactor,'observerKf_N_A',20, ...
   'seed',q.sensor.seed,'forceReference_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi, ...
   'actualForceBias_N',q.sensor.bias,'observerDistalMass_kg',q.sensor.distalMass_model_kg, ...
   'controllerSampleTime_s',q.force.sampleTime,'forceKnownDelay_s',q.sensor.delay, ...
   'currentModelNoisePower_A2_s',q.coil.control.currentModelNoisePower_A2_s, ...
   'forceBiasStateEnabled',q.coil.control.forceBiasEnabled, ...
   'forceBiasNoisePower_N2_s',q.coil.control.forceBiasNoisePower_N2_s, ...
   'executed',true,'completed',m.completed,'passed',m.passed, ...
   'peakForceError_N',m.peakAbsForceError_N,'peakCurrent_A',m.peakCurrent_A,'peakVoltage_V',m.peakVoltage_V, ...
   'gateReadyFraction',mean(r.robotGate.ready),'lostContact',m.lostContact,'strokeStopHit',m.strokeStopHit,'reason',m.reason);
  if isempty(rows),rows=row;else,rows(end+1)=row;end
  fprintf('CURRENT_UNCERTAINTY_RUN %s power%.5g Kf%.5g seed%d: %.6f N pass%d\n', ...
   kind,power,row.actualKf_N_A,row.seed,row.peakForceError_N,row.passed);
  checkpoint();Simulink.sdi.clear;
 end
 function checkpoint()
  report.rows=rows;report.groups=groups;report.selectedTraceFiles=traces;
  report.simulations=sum([rows.executed]);report.passed=sum([rows.passed]);report.elapsedSeconds=toc(started);
  stem=fullfile(output,['controller_current_uncertainty_' report.phase]);
  fid=fopen([stem '.json'],'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  if ~isempty(rows),writetable(struct2table(rows),[stem '.csv']);end
  save([stem '.mat'],'report');
 end
end

function patch_uncertainty_helper(file)
source=fileread(file);if contains(source,'currentNoisePower'),return;end
pairs={
 'distalModel=p.sensor.distalMass_model_kg;',sprintf(['distalModel=p.sensor.distalMass_model_kg;\n' ...
 'currentNoisePower=0;if isfield(w,''currentModelNoisePower_A2_s''),currentNoisePower=w.currentModelNoisePower_A2_s;end\n' ...
 'forceBiasEnabled=isfield(w,''forceBiasEnabled'') && w.forceBiasEnabled;\n' ...
 'forceBiasPower=0;if isfield(w,''forceBiasNoisePower_N2_s''),forceBiasPower=w.forceBiasNoisePower_N2_s;end\n' ...
 'assert(isfinite(currentNoisePower) && currentNoisePower>=0 && isfinite(forceBiasPower) && forceBiasPower>=0);\n' ...
 'if forceBiasEnabled,assert(m.lowerRigid && m.upperRigid,''Force-bias augmentation is only qualified for both-rigid.'');end\n' ...
 'np=9+double(forceBiasEnabled);']);
 'predict.residualJerkPower,drive.directCoulombCorrection,weights];','predict.residualJerkPower,drive.directCoulombCorrection,weights,currentNoisePower,forceBiasEnabled,forceBiasPower];';
 'A=zeros(9);Bu=zeros(9,1);Bf=zeros(9,1);','A=zeros(np);Bu=zeros(np,1);Bf=zeros(np,1);';
 'Cf=zeros(1,9);Cf([1 2 8])=[m.k1,m.c1,ms];Cu=zeros(1,9);Cu(5)=1;','Cf=zeros(1,np);Cf([1 2 8])=[m.k1,m.c1,ms];Cu=zeros(1,np);Cu(5)=1;';
 'Br=A(:,9);Br(9)=0;',sprintf('if forceBiasEnabled,Cf(10)=1;end\nBr=A(:,9);Br(9)=0;');
 'E=expm([A,Bu,Bf,Br;zeros(3,12)]*ts);Ad=E(1:9,1:9);Bd=E(1:9,10:12);','E=expm([A,Bu,Bf,Br;zeros(3,np+3)]*ts);Ad=E(1:np,1:np);Bd=E(1:np,np+(1:3));';
 'Qc(9,9)=Qc(9,9)+robotPower;',sprintf(['Qc(9,9)=Qc(9,9)+robotPower;\n' ...
 'Qc(7,7)=Qc(7,7)+currentNoisePower;\n' ...
 'if forceBiasEnabled,Qc(10,10)=Qc(10,10)+forceBiasPower;end']);
 'V=expm([A,Qc;zeros(9),-A'']*ts);Qd=V(1:9,10:18)*V(1:9,1:9)'';','V=expm([A,Qc;zeros(np),-A'']*ts);Qd=V(1:np,np+(1:np))*V(1:np,1:np)'';';
 'nf=max(p.d.sensorDelaySamples-1,0);ng=p.d.strokeDelaySamples;N=10+nf+ng;','nf=max(p.d.sensorDelaySamples-1,0);ng=p.d.strokeDelaySamples;fi=np+1;N=fi+nf+ng;';
 'Aa=zeros(N);Aa(1:9,1:9)=Ad;Ba=zeros(N,3);Ba(1:9,:)=Bd;','Aa=zeros(N);Aa(1:np,1:np)=Ad;Ba=zeros(N,3);Ba(1:np,:)=Bd;';
 'Qa=zeros(N);Qa(1:9,1:9)=Qd;','Qa=zeros(N);Qa(1:np,1:np)=Qd;';
 'filterRow=zeros(1,N);filterRow(1:9)=(1-a)*CfMeasured;filterRow(10)=a;Aa(10,:)=filterRow;','filterRow=zeros(1,N);filterRow(1:np)=(1-a)*CfMeasured;filterRow(fi)=a;Aa(fi,:)=filterRow;';
 'Ba(10,2)=-(1-a);','Ba(fi,2)=-(1-a);';
 'else,C(1,10+nf)=1;end','else,C(1,fi+nf)=1;end';
 'for j=1:nf,Aa(10+j,9+j)=1;end','for j=1:nf,Aa(fi+j,fi-1+j)=1;end';
 'firstGap=11+nf;','firstGap=fi+1+nf;';
 'if ng==0,C(2,1:9)=Cu;','if ng==0,C(2,1:np)=Cu;';
 'Aa(firstGap,1:9)=Cu;','Aa(firstGap,1:np)=Cu;';
 'scales=[1e-3,0.1,1e-3,0.1,1e-3,0.1,1,10,10,ones(1,N-9)];',sprintf(['scales=[1e-3,0.1,1e-3,0.1,1e-3,0.1,1,10,10];\n' ...
 'if forceBiasEnabled,scales=[scales,1];end;scales=[scales,ones(1,N-np)];']);
 'c.C=c.C(1:9,:);','c.C=c.C(1:np,:);';
 'c.D=c.D(1:9,:);','c.D=c.D(1:np,:);';
 'c.feedback=zeros(1,9);','c.feedback=zeros(1,np);';
 'c.feedback(9)=K(1:5)*arReference;c.force=Cf;',sprintf(['c.feedback(9)=K(1:5)*arReference;c.force=Cf;\n' ...
 'if forceBiasEnabled,c.feedback(10)=-(1-c.feedback(7))/kf;end'])
 };
for j=1:size(pairs,1)
 assert(contains(source,pairs{j,1}),'Frozen root helper patch anchor%d is missing.',j);
 source=strrep(source,pairs{j,1},pairs{j,2});
end
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s',source);fclose(fid);
end

function finish_uncertainty(mdl)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;
end
