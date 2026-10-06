function report=qualify_rigid_sensors(snapshotFolder,outputFolder,phase,candidateFile)
%QUALIFY_RIGID_SENSORS Run native Simulink attachment and sensor experiments.
% Uses an immutable model snapshot and private disk-backed generated/SDI files.
% No alternative plant solver, user desktop, main-model save, or surface preview.
if nargin<1,snapshotFolder='/home/tommie/.local/state/actuator-model/qualification-20260930';end
if nargin<2,outputFolder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'validation');end
if nargin<3,phase='corners';end
if nargin<4,candidateFile='';end
if startsWith(phase,'actual_rigid_') || strcmp(phase,'final_rigid_dense')
 report=qualify_actual_rigid(snapshotFolder,outputFolder,phase,candidateFile);return;
end
maxNumCompThreads(2);
addpath(fullfile(snapshotFolder,'helpers'),'-begin');
load(fullfile(snapshotFolder,'base_parameters.mat'),'p');
if ~isempty(candidateFile)
 [~,~,extension]=fileparts(candidateFile);
 if strcmpi(extension,'.json')
  candidate=jsondecode(fileread(candidateFile));if isfield(candidate,'parameters'),p=candidate.parameters;else,p=candidate;end
 else
  candidate=load(candidateFile);assert(isfield(candidate,'p'),'Candidate MAT file must contain p.');p=candidate.p;
 end
end
if isempty(p.ideal.maxAxialForce),p.ideal.maxAxialForce=inf;end
p=acf_prepare(p);
catalog=jsondecode(fileread(fullfile(snapshotFolder,'spring_catalog.json')));
densePhases={'dense_selected','dense_measured_travel','dense_measured_travel_3p5A'};
workerFolder=fullfile(snapshotFolder,'rigid_sensor_worker');
if strcmp(phase,'dense_measured_travel'),workerFolder=[workerFolder '_measured'];end
if strcmp(phase,'dense_measured_travel_3p5A'),workerFolder=[workerFolder '_measured_3p5A'];end
if startsWith(phase,'physical_sensor_'),workerFolder=[workerFolder '_' phase];end
if startsWith(phase,'physical_sensor_toolA'),workerFolder=[workerFolder '_corrected'];end
folders={workerFolder,fullfile(workerFolder,'cache'),fullfile(workerFolder,'code'), ...
    fullfile(workerFolder,'sdi'),fullfile(workerFolder,'traces'),outputFolder};
for j=1:numel(folders),if ~isfolder(folders{j}),mkdir(folders{j});end,end
Simulink.fileGenControl('set','CacheFolder',folders{2},'CodeGenFolder',folders{3},'createDir',true);
originalSDILocation=Simulink.sdi.getStorageLocation;
Simulink.sdi.setStorageLocation(folders{4});Simulink.sdi.setRecordData(true);
load_system(fullfile(snapshotFolder,'acf_force_control_simple.slx'));
mdl='acf_force_control_simple';
set_param(mdl,'FastRestart','off');
delayBlock=[mdl '/Force measurement/Sensor sample delay'];
normalDelay=get_param(delayBlock,'DelayLength');
cleanup=onCleanup(@()finish(mdl,originalSDILocation)); %#ok<NASGU>
robotTravelMeasurement='Ideal plant travel supplied to the slow recentering loop; the fast voice-coil controller receives the modeled magnet measurement.';
if any(strcmp(phase,{'dense_measured_travel','dense_measured_travel_3p5A','selected_robust'})) || startsWith(phase,'physical_sensor_')
 set_param([mdl '/Receive actuator_travel_m 6'],'GotoTag','measured_magnet_position');
 robotTravelMeasurement='Modeled noisy/delayed magnet measurement supplied to both fast force control and slow robot recentering.';
end
rows=struct([]);runs=0;started=tic;previousElapsed=0;resumeRows=0;
report=struct('date','2026-09-30','phase',phase,'status','running', ...
    'recipeVersion','2026-09-30_measured_slow_v2', ...
    'scope','1 mm high by 10 mm wide sharp bump at 100 mm/s; B+S = 2 kg; true contact force error <= 0.5 N for the entire recording.', ...
    'snapshot',snapshotFolder,'candidateFile',candidateFile,'robotTravelMeasurement',robotTravelMeasurement, ...
    'matlabVersion',version,'baseParameters',p,'rows',rows);
checkpointFile=fullfile(outputFolder,['rigid_sensor_qualification_' phase '.mat']);
if any(strcmp(phase,densePhases)) && isfile(checkpointFile)
 saved=load(checkpointFile,'report');
 if strcmp(saved.report.status,'running')
  rows=saved.report.rows;runs=numel(rows);resumeRows=runs;previousElapsed=saved.report.elapsedSeconds;
  report=saved.report;report.robotTravelMeasurement=robotTravelMeasurement;
  report.resumeCount=optional(report,'resumeCount',0)+1;
  fprintf('RIGID_SENSOR_RESUME %s: %d checkpointed cases, %d remaining.\n',phase,resumeRows,350-resumeRows);
 end
end

switch phase
 case 'corners'
  % Test both rigid first so a promising simple option is available early.
  modes=[3 0 1 2];
  for mode=modes
   if mode==0,dampings=[30 5;0 5;30 0;0 0];
   elseif mode==1,dampings=[0 5;0 0];
   elseif mode==2,dampings=[30 0;0 0];
   else,dampings=[0 0];end
   for dm=1:size(dampings,1)
    for stress=[false true]
     set_param(mdl,'FastRestart','off');
     if stress,set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');
     else,set_param(delayBlock,'DelayLength',normalDelay);end
     set_param(mdl,'FastRestart','on');
     for ref=[5 50]
      for angle=[0 90]
       q=attachment(p,mode,dampings(dm,:));q.force.reference=ref;q.mechanics.angle=angle*pi/180;
       if stress,q.sensor.noiseRms=.02;q.sensor.seed=7;end
       [row,r]=one_run(mdl,q,mode,stress*.0002,'corner',runs+1);
       rows=append(rows,row);runs=runs+1;
       if ~isempty(r) && (mode==3 || (mode==2 && dm==1))
        tracePath=fullfile(folders{5},sprintf('corner_%03d.mat',runs));save(tracePath,'r');
       end
       checkpoint();
      end
     end
    end
   end
  end
  % A stiffer catalogue-envelope spring tests whether a single remaining link
  % can meet the high-force orientation without excessive spring length.
  for mode=[1 2]
   for catalogueIndex=[3 4]
    for damping=[0 5 15 30 60]
     for stress=[false true]
      set_param(mdl,'FastRestart','off');
      if stress,set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');
      else,set_param(delayBlock,'DelayLength',normalDelay);end
      set_param(mdl,'FastRestart','on');
      for ref=[5 50]
       for angle=[0 90]
        q=attachment(p,mode,[damping damping]);
        q.springs.lower=catalog.springs(catalogueIndex);q.springs.upper=catalog.springs(catalogueIndex);
        q.springs.commonPreload_N=32;
        if ~q.mechanics.lowerRigid,q.mechanics.k1=2*q.springs.lower.rate_N_m;end
        if ~q.mechanics.upperRigid,q.mechanics.k2=2*q.springs.upper.rate_N_m;end
        q.force.reference=ref;q.mechanics.angle=angle*pi/180;
        if stress,q.sensor.noiseRms=.02;q.sensor.seed=7;end
        [row,r]=one_run(mdl,q,mode,stress*.0002,'single_link_strong',runs+1);
        rows=append(rows,row);runs=runs+1;checkpoint();
       end
      end
     end
    end
   end
  end
 case 'sensing'
  q=attachment(p,3,[0 0]);
  % Worst mechanical load: 50 N at 90 degrees; baseline has the nominal
  % one-micrometer position noise and configured 0.2 ms position delay.
  q.force.reference=50;q.mechanics.angle=pi/2;
  configs={};labels={};unknowns=[];
  for noise=[0 .01 .02 .05 .1]
   one=q;one.sensor.noiseRms=noise;one.sensor.seed=7;
   for extra=[0 .0002 .0004 .0008]
    configs{end+1}=one;labels{end+1}='force_noise_delay';unknowns(end+1)=extra; %#ok<AGROW>
   end
  end
  for tau=[.0001 .0003 .0005 .001 .002]
   for seed=[7 29 1729]
    one=q;one.sensor.filterTime=tau;one.sensor.noiseRms=.02;one.sensor.seed=seed;
    configs{end+1}=one;labels{end+1}='force_filter';unknowns(end+1)=.0002; %#ok<AGROW>
   end
  end
  for noise=[0 1e-6 2e-6 5e-6 10e-6]
   for delay=[0 .0002 .0004 .0008]
    one=q;one.sensor.strokeNoiseRms=noise;one.sensor.strokeDelay=delay;one.sensor.noiseRms=.02;one.sensor.seed=7;
    configs{end+1}=one;labels{end+1}='stroke_noise_known_delay';unknowns(end+1)=.0002; %#ok<AGROW>
   end
  end
  for bias=[-.25 -.1 .1 .25]
   one=q;one.sensor.bias=bias;one.sensor.noiseRms=.02;one.sensor.seed=7;
   configs{end+1}=one;labels{end+1}='force_bias';unknowns(end+1)=.0002; %#ok<AGROW>
  end
  for rate=[1000 2000 5000 10000]
   one=q;one.force.sampleTime=1/rate;one.sensor.sampleTime=1/rate;
   one.sensor.strokeDelay=1/rate;one.sim.step=min(.0001,one.force.sampleTime/2);
   one.sensor.noiseRms=.02;one.sensor.seed=7;
   configs{end+1}=one;labels{end+1}='controller_rate';unknowns(end+1)=one.force.sampleTime; %#ok<AGROW>
  end
  % Independent seeds are actual realizations, not a statistical guarantee.
  for ref=[5 20 35 50]
   for angle=[0 30 60 90]
    for seed=[7 29 1729 31415]
     one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
     one.sensor.noiseRms=.02;one.sensor.seed=seed;
     configs{end+1}=one;labels{end+1}='range_seed';unknowns(end+1)=.0002; %#ok<AGROW>
    end
   end
  end
  for j=1:numel(configs)
   set_param(mdl,'FastRestart','off');
   one=configs{j};extra=round(unknowns(j)/one.sensor.sampleTime);
   if extra>0,set_param(delayBlock,'DelayLength',sprintf('p.d.sensorDelaySamples+%d',extra));
   else,set_param(delayBlock,'DelayLength',normalDelay);end
   [row,r]=one_run(mdl,one,3,unknowns(j),labels{j},runs+1);
   rows=append(rows,row);runs=runs+1;
   if ~isempty(r) && (strcmp(labels{j},'range_seed') || strcmp(labels{j},'force_bias'))
    tracePath=fullfile(folders{5},sprintf('sensing_%03d.mat',runs));save(tracePath,'r');
   end
   checkpoint();
  end
 case {'placement','placement_selected','physical_sensor_optimum','physical_sensor_quality','physical_sensor_toolA','physical_sensor_toolA_precision'}
  % Experiment-local native sensor blocks; the saved authoritative model is
  % untouched. Gravity is exactly tared here to isolate distal-mass dynamics.
  p.sensor.strokeBias=0;p.sensor.distalMass_kg=0;p.sensor.distalMass_model_kg=0;
  add_sensor_experiments(mdl);
  if startsWith(phase,'physical_sensor_toolA'),p.sensor.positionModel=1;end
  if optional(p.sensor,'positionModel',0)==1
   add_toolhead_position_sensor(mdl);
   report.robotTravelMeasurement='Absolute coil gap inferred from observer state 5 plus midTravel+u0 supplied to slow recentering; fast control uses the modeled S-relative-to-A position and B/S force, both zeroed at the chosen equilibrium. No separate magnet measurement.';
  end
  helperFolder=fullfile(workerFolder,'placement_helpers');if ~isfolder(helperFolder),mkdir(helperFolder);end
  source=fileread(fullfile(snapshotFolder,'helpers','acf_prepare.m'));
  old='m=p.mechanics;drive=p.coil;w=drive.control;';
  replacement=[old newline 'distalModel=0;if isfield(p.sensor,''distalMass_model_kg''),distalModel=p.sensor.distalMass_model_kg;end' newline ...
   'positionModel=0;if isfield(p.sensor,''positionModel''),positionModel=p.sensor.positionModel;end'];
  assert(contains(source,old));source=strrep(source,old,replacement);
  old='p.sensor.filterTime,p.d.sensorDelaySamples,p.d.strokeDelaySamples,weights];';
  assert(contains(source,old));source=strrep(source,old,'p.sensor.filterTime,p.d.sensorDelaySamples,p.d.strokeDelaySamples,weights,distalModel,positionModel];');
  old='Cf=zeros(1,9);Cf([1 2 8])=[m.k1,m.c1,ms];Cu=zeros(1,9);Cu(5)=1;';
  assert(contains(source,old));source=strrep(source,old,[old newline ...
   'if positionModel==1,Cu(:)=0;Cu([1 5])=[1 -1];end']);
  old='filterRow=zeros(1,N);filterRow(1:9)=(1-a)*Cf;filterRow(10)=a;Aa(10,:)=filterRow;';
  assert(contains(source,old));source=strrep(source,old, ...
   ['CfMeasured=Cf;CfMeasured(8)=CfMeasured(8)-distalModel;' newline ...
    'filterRow=zeros(1,N);filterRow(1:9)=(1-a)*CfMeasured;filterRow(10)=a;Aa(10,:)=filterRow;']);
  fid=fopen(fullfile(helperFolder,'acf_prepare.m'),'w');fprintf(fid,'%s',source);fclose(fid);
  source=fileread(fullfile(snapshotFolder,'helpers','acf_run.m'));
  marker='r.parameters=p; r.names=acf_signals; r.metrics=acf_metrics(r);';
  extra=[marker newline 'if any(strcmp(who(out),''sensor_state_log''))' newline ...
   ' states=out.get(''sensor_state_log'');r.sensorObserver=struct(''time'',states.Time(:),''data'',rows(states,9));' newline ...
   ' position=out.get(''sensor_position_log'');r.positionMeasurement=struct(''time'',position.Time(:),''data'',rows(position,1));' newline 'end'];
  assert(contains(source,marker));source=strrep(source,marker,extra);
  fid=fopen(fullfile(helperFolder,'acf_run.m'),'w');fprintf(fid,'%s',source);fclose(fid);
  addpath(helperFolder,'-begin');clear acf_prepare acf_run;
  if startsWith(phase,'physical_sensor_toolA')
   audit=p;audit=physical_sensor_audit_parameters(audit);
   set_param(delayBlock,'DelayLength',normalDelay);
   flat=[];
   quiet=evalc('flat=acf_run(mdl,audit,false);'); %#ok<NASGU>
   states=flat.sensorObserver.data;
   check=struct('forceError_N',flat.metrics.peakAbsForceError_N, ...
    'maxAbsEstimatedSurfaceAcceleration_m_s2',max(abs(states(:,8))), ...
    'maxAbsEstimatedRobotAcceleration_m_s2',max(abs(states(:,9))), ...
    'initialGapOffset_m',flat.parameters.d.u0, ...
    'initialPhysicalPositionReading_m',flat.positionMeasurement.data(1), ...
    'expectedInitialPositionReading_m',audit.mechanics.midTravel-flat.parameters.d.u0);
   report.flatInitialization=check;report.additionalInitializationSimulations=1;
   save(fullfile(outputFolder,'rigid_sensor_qualification_toolA_flat_initialization.mat'),'flat','check');
   fid=fopen(fullfile(outputFolder,'rigid_sensor_qualification_toolA_flat_initialization.json'),'w');fprintf(fid,'%s',jsonencode(check,PrettyPrint=true));fclose(fid);
   assert(check.forceError_N<1e-5 && check.maxAbsEstimatedSurfaceAcceleration_m_s2<1e-5 && ...
    check.maxAbsEstimatedRobotAcceleration_m_s2<1e-5,'Flat toolhead-sensor initialization failed.');
   fprintf('RIGID_SENSOR_FLAT_CHECK %.9g N, surface acceleration %.9g, robot acceleration %.9g.\n', ...
    check.forceError_N,check.maxAbsEstimatedSurfaceAcceleration_m_s2,check.maxAbsEstimatedRobotAcceleration_m_s2);
  end
  if startsWith(phase,'physical_sensor_')
   assert(~isempty(candidateFile),'Pass a chosen mechanics/controller candidate for the physical sensor experiments.');
   selectedMode=double(p.mechanics.lowerRigid)+2*double(p.mechanics.upperRigid);
   [configs,labels]=physical_sensor_configs(p,phase);
  else
  if strcmp(phase,'placement_selected')
   assert(~isempty(candidateFile),'Pass final candidate parameters to placement_selected.');
   selectedMode=double(p.mechanics.lowerRigid)+2*double(p.mechanics.upperRigid);
   selectedDamping=[p.mechanics.c1 p.mechanics.c2];
   distalValues=[0 .025 .05 .1 .2 .5 1.4];
   if p.mechanics.lowerRigid,distalValues=[distalValues 2];end
  else,selectedMode=3;selectedDamping=[0 0];distalValues=[0 .025 .05 .1 .2 .5 1.4 2];end
  configs={};labels={};
  for distalMass=distalValues
   for calibrated=[false true]
    if distalMass==0 && calibrated,continue;end
    for stress=[false true]
     one=attachment(p,selectedMode,selectedDamping);one.force.reference=50;one.mechanics.angle=pi/2;
     one.sensor.distalMass_kg=distalMass;
     if calibrated,one.sensor.distalMass_model_kg=distalMass;end
     if stress,one.sensor.noiseRms=.02;one.sensor.seed=7;end
     configs{end+1}=one; %#ok<AGROW>
     if calibrated,labels{end+1}='distal_mass_calibrated';else,labels{end+1}='distal_mass_unknown';end %#ok<AGROW>
    end
   end
  end
  for bias=[-20 -5 5 20]*1e-6
   one=attachment(p,selectedMode,selectedDamping);one.force.reference=50;one.mechanics.angle=pi/2;
   one.sensor.strokeBias=bias;one.sensor.noiseRms=.02;one.sensor.seed=7;
   configs{end+1}=one;labels{end+1}='stroke_bias'; %#ok<AGROW>
  end
  for extraStroke=[0 .0002 .0004 .0008]
   one=attachment(p,selectedMode,selectedDamping);one.force.reference=50;one.mechanics.angle=pi/2;
   one.sensor.strokeUnknownDelay=extraStroke;one.sensor.noiseRms=.02;one.sensor.seed=7;
   configs{end+1}=one;labels{end+1}='stroke_unknown_delay'; %#ok<AGROW>
  end
  end
  best=[];bestError=inf;
  for j=1:numel(configs)
   one=configs{j};set_param(mdl,'FastRestart','off');
   if startsWith(phase,'physical_sensor_'),unknown=0;
   elseif one.sensor.noiseRms>0,unknown=.0002;
   else,unknown=0;end
   extraForce=round(unknown/one.sensor.sampleTime);
   set_param(delayBlock,'DelayLength',sprintf('p.d.sensorDelaySamples+%d',extraForce));
   extra=0;if isfield(one.sensor,'strokeUnknownDelay'),extra=round(one.sensor.strokeUnknownDelay/one.force.sampleTime);end
   set_param([mdl '/Magnet position measurement/Measurement delay'],'DelayLength',sprintf('p.d.strokeDelaySamples+%d',extra));
   [row,r]=one_run(mdl,one,selectedMode,unknown,labels{j},runs+1);rows=append(rows,row);runs=runs+1;
   if ~isempty(r)
    if row.peakForceError_N<bestError,best=r;bestError=row.peakForceError_N;end
    if ~startsWith(phase,'physical_sensor_')
     tracePath=fullfile(folders{5},sprintf('%s_%03d.mat',phase,runs));save(tracePath,'r');
    end
   end
   checkpoint();
  end
  if startsWith(phase,'physical_sensor_') && ~isempty(best)
   report.bestError_N=bestError;report.bestMargin_N=best.parameters.force.tolerance-bestError;
   report.bestCaseParameters=best.parameters;report.bestCaseMetrics=best.metrics;
   report.scope='Actual B/S load cell with 1.4 kg rigid S downstream, exact gravity tare and a calibrated measurement model; 50 N, 90 degrees, the sharp bump. Noise/controller covariance and stated sensing/drive latency are screened; this is not full-range qualification.';
   report.loadCellAssumption='Fcell_tared = true contact force - 1.4 kg * S acceleration; acceleration appears in sensor physics only and in the observer estimated measurement equation, never as a controller truth input.';
   originalP=p;p=best.parameters;save(fullfile(outputFolder,['rigid_sensor_qualification_' phase '_candidate.mat']),'p');p=originalP;
  end
 case {'single_link_robust','selected_robust'}
  if strcmp(phase,'selected_robust')
   assert(~isempty(candidateFile),'Pass the chosen parameter file for selected_robust.');q=p;
  else
   q=attachment(p,2,[60 0]);q.springs.lower=catalog.springs(4);
   q.mechanics.k1=2*q.springs.lower.rate_N_m;q.springs.commonPreload_N=32;
  end
  configs={};labels={};unknowns=[];
  if strcmp(phase,'single_link_robust')
  for ref=[5 20 35 50]
   for angle=[0 30 60 90]
    for seed=[7 29 1729 31415]
     one=q;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
     one.sensor.noiseRms=.02;one.sensor.seed=seed;
     configs{end+1}=one;labels{end+1}='single_link_range_seed';unknowns(end+1)=.0002; %#ok<AGROW>
    end
   end
  end
  end
  for noise=[0 .01 .02 .05 .1]
   for extra=[0 .0002 .0004 .0008]
    one=q;one.force.reference=50;one.mechanics.angle=pi/2;one.sensor.noiseRms=noise;one.sensor.seed=7;
    configs{end+1}=one;labels{end+1}='single_link_force_noise_delay';unknowns(end+1)=extra; %#ok<AGROW>
   end
  end
  for stroke=[0 1e-6 2e-6 5e-6 10e-6]
   one=q;one.force.reference=50;one.mechanics.angle=pi/2;one.sensor.noiseRms=.02;one.sensor.seed=7;one.sensor.strokeNoiseRms=stroke;
   configs{end+1}=one;labels{end+1}='single_link_stroke_noise';unknowns(end+1)=.0002; %#ok<AGROW>
  end
  for bias=[-.25 -.1 -.05 .05 .1 .25]
   one=q;one.force.reference=50;one.mechanics.angle=pi/2;one.sensor.noiseRms=.02;one.sensor.seed=7;one.sensor.bias=bias;
   configs{end+1}=one;labels{end+1}='single_link_force_bias';unknowns(end+1)=.0002; %#ok<AGROW>
  end
  for mismatch=1:8
   one=q;one.force.reference=50;one.mechanics.angle=pi/2;one.sensor.noiseRms=.02;one.sensor.seed=7;
   configs{end+1}=one;labels{end+1}=sprintf('single_link_mismatch_%d',mismatch);unknowns(end+1)=.0002; %#ok<AGROW>
  end
  % Compare absolute extra force delay at 10 kHz; separate this from the
  % one-extra-sample rate screen, which changes delay when the rate changes.
  if strcmp(phase,'single_link_robust')
  for extra=[.0001 .0002 .0004]
   for seed=[7 29 1729 31415]
    one=attachment(p,3,[0 0]);one.force.reference=50;one.mechanics.angle=pi/2;
    one.force.sampleTime=.0001;one.sensor.sampleTime=.0001;one.sim.step=.00005;
    one.sensor.strokeDelay=.0001;one.sensor.noiseRms=.02;one.sensor.seed=seed;
    configs{end+1}=one;labels{end+1}='both_rigid_10khz_absolute_delay';unknowns(end+1)=extra; %#ok<AGROW>
   end
  end
  end
  for j=1:numel(configs)
   one=configs{j};set_param(mdl,'FastRestart','off');extra=round(unknowns(j)/one.sensor.sampleTime);
   set_param(delayBlock,'DelayLength',sprintf('p.d.sensorDelaySamples+%d',extra));
   restorePhysical=onCleanup(@()restore_physical(mdl)); %#ok<NASGU>
   if startsWith(labels{j},'single_link_mismatch_')
    index=str2double(extractAfter(labels{j},'single_link_mismatch_'));
    switch index
     case 1
      set_param([mdl '/Actuator and RABS mechanics/Actuator/Voice coil/Coil force constant'],'Gain','0.9*p.coil.forceConstant');
      set_param([mdl '/Actuator and RABS mechanics/Actuator/Voice coil/Back EMF'],'Gain','0.9*p.coil.forceConstant');
     case 2
      set_param([mdl '/Actuator and RABS mechanics/Actuator/Voice coil/Coil force constant'],'Gain','1.1*p.coil.forceConstant');
      set_param([mdl '/Actuator and RABS mechanics/Actuator/Voice coil/Back EMF'],'Gain','1.1*p.coil.forceConstant');
     case 3,set_param([mdl '/Actuator and RABS mechanics/Actuator/Voice coil/Current regulator'],'Gain','p.coil.inductance/(1.25*p.coil.currentResponse)');
     case 4,set_param([mdl '/Actuator and RABS mechanics/B to S spring and damper/Stiffness'],'Gain','0.9*p.mechanics.k1');
     case 5,set_param([mdl '/Actuator and RABS mechanics/B to S spring and damper/Damping'],'Gain','0.8*p.mechanics.c1');
     case 6,one.mechanics.mB=.54;one.mechanics.mS=1.26;
     case 7,one.mechanics.mB=.66;one.mechanics.mS=1.54;
     case 8,one.surface.height=.0012;
    end
   end
   mode=double(one.mechanics.lowerRigid)+2*double(one.mechanics.upperRigid);
   [row,r]=one_run(mdl,one,mode,unknowns(j),labels{j},runs+1);rows=append(rows,row);runs=runs+1;
   if ~isempty(r) && (strcmp(labels{j},'single_link_range_seed') || startsWith(labels{j},'single_link_mismatch_'))
    tracePath=fullfile(folders{5},sprintf('%s_%03d.mat',phase,runs));save(tracePath,'r');
   end
   clear restorePhysical;checkpoint();
  end
 case {'dense_selected','dense_measured_travel','dense_measured_travel_3p5A'}
  assert(~isempty(candidateFile),'Pass the final chosen parameter file for dense qualification.');
  mode=double(p.mechanics.lowerRigid)+2*double(p.mechanics.upperRigid);
  cohorts=[0 7 29 1729 31415];worst=[];worstError=-inf;
  if ~isempty(rows),worstError=max([rows.peakForceError_N]);end
  ordinal=0;
  for cohort=cohorts
   set_param(mdl,'FastRestart','off');
   if cohort==0,set_param(delayBlock,'DelayLength',normalDelay);unknown=0;
   else
    unknown=.0002;extra=round(unknown/p.sensor.sampleTime);
    assert(abs(extra*p.sensor.sampleTime-unknown)<1e-12,'Dense stress delay must be representable on the force-sensor grid.');
    set_param(delayBlock,'DelayLength',sprintf('p.d.sensorDelaySamples+%d',extra));
   end
   set_param(mdl,'FastRestart','on');
   for ref=5:5:50
    for angle=0:15:90
     ordinal=ordinal+1;if ordinal<=resumeRows,continue;end
     one=p;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
     if cohort==0,one.sensor.noiseRms=0;one.sensor.seed=1729;
     else,one.sensor.noiseRms=.02;one.sensor.seed=cohort;end
     [row,r]=one_run(mdl,one,mode,unknown,phase,runs+1);rows=append(rows,row);runs=runs+1;
     if ~isempty(r)
      if row.peakForceError_N>worstError,worst=r;worstError=row.peakForceError_N;end
      if any(ref==[5 50]) && any(angle==[0 90])
       if strcmp(phase,'dense_selected'),traceName=sprintf('dense_%03d.mat',runs);
       else,traceName=sprintf('%s_%03d.mat',phase,runs);end
       tracePath=fullfile(folders{5},traceName);save(tracePath,'r');
      end
     end
     checkpoint();
    end
   end
  end
  if isempty(worst)
   % Recover the actual worst trace if its earlier worker exited after the
   % checkpoint but before exporting the final selected history.
   [~,index]=max([rows.peakForceError_N]);one=p;one.force.reference=rows(index).forceReference_N;
   one.mechanics.angle=rows(index).angle_deg*pi/180;one.sensor.noiseRms=rows(index).forceNoiseRms_N;
   one.sensor.seed=rows(index).seed;set_param(mdl,'FastRestart','off');
   extra=round(rows(index).forceUnknownDelay_s/one.sensor.sampleTime);
   set_param(delayBlock,'DelayLength',sprintf('p.d.sensorDelaySamples+%d',extra));
   quiet=evalc('worst=acf_run(mdl,one,false);'); %#ok<NASGU>
   assert(abs(worst.metrics.peakAbsForceError_N-worstError)<1e-9,'Recovered worst-case trace differs from the recorded grid result.');
   report.trajectoryRecoverySimulations=1;
  end
  report.worstError_N=worstError;report.worstMargin_N=p.force.tolerance-worstError;
  if ~isempty(worst),report.worstCaseParameters=worst.parameters;report.worstCaseMetrics=worst.metrics;end
  report.rangeForce_N=[5 50];report.rangeAngle_deg=[0 90];report.rangeMesh=[10 7];
  report.noiseSeeds=cohorts(2:end);report.qualificationCaveat='A dense sampled range, not a proof for every continuous parameter, time-varying orientation, or real hardware.';
 otherwise,error('Unknown phase: %s',phase);
end
report.status='complete';checkpoint();
if any(strcmp(phase,densePhases))
 selectedRuns={worst};save(fullfile(outputFolder,['rigid_sensor_qualification_' phase '_traces.mat']),'selectedRuns','report','p');
elseif startsWith(phase,'physical_sensor_')
 selectedRuns={best};save(fullfile(outputFolder,['rigid_sensor_qualification_' phase '_traces.mat']),'selectedRuns','report','p');
end
fprintf('RIGID_SENSOR_COMPLETE %s: %d simulations, %d attempts, %d pass; %.1f seconds.\n', ...
 phase,sum([rows.executed]),runs,sum([rows.passed]),previousElapsed+toc(started));

 function checkpoint()
  report.rows=rows;report.attempts=runs;report.simulations=sum([rows.executed]);
  report.rejectedBeforeSimulation=sum(~[rows.executed]);report.passed=sum([rows.passed]);report.elapsedSeconds=previousElapsed+toc(started);
  path=fullfile(outputFolder,['rigid_sensor_qualification_' phase '.json']);
  fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  writetable(struct2table(rows),fullfile(outputFolder,['rigid_sensor_qualification_' phase '.csv']));
  save(fullfile(outputFolder,['rigid_sensor_qualification_' phase '.mat']),'report');
  if mod(runs,8)==0,Simulink.sdi.clear;end
  if mod(runs,8)==0 || runs==1
   fprintf('RIGID_SENSOR_PROGRESS %s: %d runs, %d pass, last %.6g N, %.1f seconds\n',phase,runs,sum([rows.passed]),rows(end).peakForceError_N,toc(started));
  end
 end
end

function q=attachment(q,mode,damping)
q.mechanics.lowerRigid=bitand(mode,1)>0;q.mechanics.upperRigid=bitand(mode,2)>0;
q.mechanics.c1=damping(1);q.mechanics.c2=damping(2);
if q.mechanics.lowerRigid,q.mechanics.k1=0;q.mechanics.c1=0;end
if q.mechanics.upperRigid,q.mechanics.k2=0;q.mechanics.c2=0;end
end

function rows=append(rows,row)
if isempty(rows),rows=row;else,rows(end+1)=row;end
end

function [row,r]=one_run(mdl,q,mode,unknownDelay,label,id)
row=struct('id',id,'study',label,'mode',mode,'lowerRigid',q.mechanics.lowerRigid,'upperRigid',q.mechanics.upperRigid, ...
 'forceReference_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi, ...
 'k1_N_m',q.mechanics.k1,'k2_N_m',q.mechanics.k2,'c1_Ns_m',q.mechanics.c1,'c2_Ns_m',q.mechanics.c2, ...
 'mA_kg',q.mechanics.mA,'mB_kg',q.mechanics.mB,'mS_kg',q.mechanics.mS, ...
 'coilForceConstant_N_A',q.coil.forceConstant,'coilResistance_ohm',q.coil.resistance,'coilInductance_H',q.coil.inductance, ...
 'coilCurrentResponse_s',q.coil.currentResponse,'coilCurrentLimit_A',q.coil.peakCurrent,'coilVoltageLimit_V',q.coil.voltage, ...
 'observerSurfaceJerkPower',q.coil.control.surfaceJerkPower,'observerForceNoise_N',q.coil.control.forceNoise_N, ...
 'observerStrokeNoise_m',q.coil.control.strokeNoise_m,'controlForceScale_N',q.coil.control.forceScale_N, ...
 'sensorPositionModel',optional(q.sensor,'positionModel',0), ...
 'springPreload_N',q.springs.commonPreload_N,'forceFilter_s',q.sensor.filterTime, ...
 'forceNoiseRms_N',q.sensor.noiseRms,'forceBias_N',q.sensor.bias,'forceKnownDelay_s',q.sensor.delay, ...
 'forceUnknownDelay_s',unknownDelay,'strokeNoiseRms_m',q.sensor.strokeNoiseRms,'strokeKnownDelay_s',q.sensor.strokeDelay, ...
 'strokeBias_m',optional(q.sensor,'strokeBias',0),'strokeUnknownDelay_s',optional(q.sensor,'strokeUnknownDelay',0), ...
 'distalMass_kg',optional(q.sensor,'distalMass_kg',0),'distalMassModel_kg',optional(q.sensor,'distalMass_model_kg',0), ...
 'controllerSampleTime_s',q.force.sampleTime,'plantStep_s',q.sim.step,'seed',q.sensor.seed, ...
 'executed',false,'passed',false,'peakForceError_N',NaN,'margin_N',NaN,'forceOnlyPassed',false, ...
 'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN,'peakForce_N',NaN,'minForce_N',NaN, ...
 'peakGapAcceleration_m_s2',NaN,'travelMin_mm',NaN,'travelMax_mm',NaN, ...
 'completed',false,'lostContact',false,'strokeStopHit',false,'springSizingPassed',false, ...
 'freeSpringLengthMax_mm',NaN,'installedSpringLengthMax_mm',NaN,'reason','pending');
r=[];
try
 quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
 m=r.metrics;row.executed=true;row.passed=m.passed;row.peakForceError_N=m.peakAbsForceError_N;
 row.margin_N=q.force.tolerance-m.peakAbsForceError_N;
 row.forceOnlyPassed=m.completed && m.peakAbsForceError_N<=q.force.tolerance && ~m.lostContact && ~m.strokeStopHit && ~m.overCurrent;
 row.peakCurrent_A=m.peakCurrent_A;row.rmsCurrent_A=m.rmsCurrent_A;row.peakVoltage_V=m.peakVoltage_V;
 row.peakForce_N=max(r.data(:,1));row.minForce_N=min(r.data(:,1));
 row.peakGapAcceleration_m_s2=m.peakGapAcceleration_m_s2;
 row.travelMin_mm=m.minTravel_mm;row.travelMax_mm=m.maxTravel_mm;
 row.completed=m.completed;row.lostContact=m.lostContact;row.strokeStopHit=m.strokeStopHit;
 row.springSizingPassed=m.springSizing.passed;row.freeSpringLengthMax_mm=m.springSizing.combinedFreeLengthMax_mm;
 row.installedSpringLengthMax_mm=m.springSizing.combinedInstalledLengthMax_mm;row.reason=m.reason;
catch err
 row.reason=['simulation or preparation error: ' err.message];
 fprintf('RIGID_SENSOR_ERROR %d: %s\n',id,err.message);
end
end

function value=optional(s,name,default)
if isfield(s,name),value=s.(name);else,value=default;end
end

function [configs,labels]=physical_sensor_configs(p,phase)
p.force.reference=50;p.mechanics.angle=pi/2;
p.sensor.distalMass_kg=1.4;p.sensor.distalMass_model_kg=1.4;
p.sensor.bias=0;p.sensor.strokeBias=0;p.sensor.seed=7;
configs={};labels={};
if strcmp(phase,'physical_sensor_optimum')
 p.coil.currentResponse=.0001;p.sensor.filterTime=.0001;
 p.sim.step=.000025;
 p.sensor.noiseRms=0;p.sensor.strokeNoiseRms=0;
 p.sensor.delay=0;p.sensor.strokeDelay=0;
 for assumedForce=[.001 .003 .01 .03]
  for assumedStroke=[.05 .1 .25]*1e-6
   for jerk=[10 70 300 1000]
    one=p;one.coil.control.forceNoise_N=assumedForce;
    one.coil.control.strokeNoise_m=assumedStroke;one.coil.control.surfaceJerkPower=jerk;
    configs{end+1}=one;labels{end+1}='physical_sensor_optimistic_observer'; %#ok<AGROW>
   end
  end
 end
elseif strcmp(phase,'physical_sensor_toolA_precision')
 p.coil.currentResponse=.0001;p.sensor.filterTime=.0001;p.sim.step=.000025;
 p.sensor.strokeNoiseRms=.1e-6;p.sensor.delay=0;p.sensor.positionModel=1;
 p.coil.control.strokeNoise_m=.1e-6;
 for forceNoise=[.001 .005]
  for positionDelay=[0 .0002]
   for assumedForce=[.001 .003 .01]
    for jerk=[300 1000]
     one=p;one.sensor.noiseRms=forceNoise;one.sensor.strokeDelay=positionDelay;
     one.coil.control.forceNoise_N=assumedForce;one.coil.control.surfaceJerkPower=jerk;
     configs{end+1}=one;labels{end+1}='physical_sensor_toolA_precision'; %#ok<AGROW>
    end
   end
  end
 end
elseif strcmp(phase,'physical_sensor_toolA')
 p.coil.currentResponse=.0001;p.sensor.filterTime=.0001;p.sim.step=.000025;
 p.sensor.noiseRms=.02;p.sensor.delay=0;p.sensor.strokeDelay=.0002;
 p.sensor.positionModel=1;
 for actualStroke=[.1 1]*1e-6
  for assumedForce=[.01 .03 .06]
   for assumedStroke=[.1 .25 1]*1e-6
    for jerk=[70 300 1000 3000]
     one=p;one.sensor.strokeNoiseRms=actualStroke;
     one.coil.control.forceNoise_N=assumedForce;one.coil.control.strokeNoise_m=assumedStroke;
     one.coil.control.surfaceJerkPower=jerk;
     configs{end+1}=one;labels{end+1}='physical_sensor_toolS_relative_A'; %#ok<AGROW>
    end
   end
  end
 end
else
 for forceNoise=[0 .001 .005 .02]
  for strokeNoise=[0 .1 1]*1e-6
   for knownDelay=[0 .0002]
    for matchedCovariance=[false true]
     one=p;one.sensor.noiseRms=forceNoise;one.sensor.strokeNoiseRms=strokeNoise;
     one.sensor.delay=knownDelay;one.sensor.strokeDelay=.0002;
     if matchedCovariance
      one.coil.control.forceNoise_N=max(.001,forceNoise);
      one.coil.control.strokeNoise_m=max(.05e-6,strokeNoise);
      label='physical_sensor_quality_matched_covariance';
     else,label='physical_sensor_quality_screened_covariance';end
     configs{end+1}=one;labels{end+1}=label; %#ok<AGROW>
    end
   end
  end
 end
end
end

function add_sensor_experiments(mdl)
force=[mdl '/Force measurement'];
add_block('simulink/Ports & Subsystems/In1',[force '/surface_acceleration_m_s2'],'Port','2','Position',[20 598 50 612]);
add_block('simulink/Math Operations/Gain',[force '/Distal moving mass'],'Gain','p.sensor.distalMass_kg','Position',[120 577 265 633]);
add_block('simulink/Math Operations/Sum',[force '/Contact force minus distal inertia'],'Inputs','+-','Position',[75 105 110 155]);
delete_line(force,'true_Fc_N/1','Sensor sampling/1');
add_line(force,'true_Fc_N/1','Contact force minus distal inertia/1','autorouting','on');
add_line(force,'surface_acceleration_m_s2/1','Distal moving mass/1','autorouting','on');
add_line(force,'Distal moving mass/1','Contact force minus distal inertia/2','autorouting','on');
add_line(force,'Contact force minus distal inertia/1','Sensor sampling/1','autorouting','on');
add_block('simulink/Signal Routing/From',[mdl '/Receive surface for load-cell experiment'],'GotoTag','surface_motion','Position',[80 980 235 1005]);
add_block('simulink/Signal Routing/Selector',[mdl '/Surface acceleration for load-cell experiment'], ...
 'NumberOfDimensions','1','IndexMode','One-based','IndexOptionArray',{'Index vector (dialog)'}, ...
 'Indices','3','InputPortWidth','3','Position',[270 975 355 1010]);
add_line(mdl,'Receive surface for load-cell experiment/1','Surface acceleration for load-cell experiment/1','autorouting','on');
add_line(mdl,'Surface acceleration for load-cell experiment/1','Force measurement/2','autorouting','on');
stroke=[mdl '/Magnet position measurement'];set_param([stroke '/Sensed position'],'Inputs','+++');
add_block('simulink/Sources/Constant',[stroke '/Position bias'],'Value','p.sensor.strokeBias','Position',[320 315 455 355]);
add_line(stroke,'Position bias/1','Sensed position/3','autorouting','on');
end

function add_toolhead_position_sensor(mdl)
% plant_log contains physical positions xA and xS at indices 5 and 7.
% Only those positions enter this instrument; no surface preview is used.
delete_line(mdl,'Receive travel for magnet sensor/1','Magnet position measurement/1');
add_block('simulink/Signal Routing/From',[mdl '/Receive positions for toolhead sensor'], ...
 'GotoTag','plant_log','Position',[80 1110 235 1135]);
add_block('simulink/Signal Routing/Selector',[mdl '/Physical A and S positions'], ...
 'NumberOfDimensions','1','IndexMode','One-based','IndexOptionArray',{'Index vector (dialog)'}, ...
 'Indices','[5 7]','InputPortWidth','25','Position',[270 1100 355 1145]);
add_block('simulink/Math Operations/Gain',[mdl '/Toolhead S relative to A'], ...
 'Gain','[-1 1]','Multiplication','Matrix(K*u)','Position',[390 1100 520 1145]);
add_block('simulink/Sources/Constant',[mdl '/Toolhead equilibrium offset'], ...
 'Value','p.mechanics.midTravel','Position',[390 1180 520 1215]);
add_block('simulink/Math Operations/Sum',[mdl '/Toolhead position sensor input'], ...
 'Inputs','++','Position',[570 1110 610 1160]);
add_line(mdl,'Receive positions for toolhead sensor/1','Physical A and S positions/1','autorouting','on');
add_line(mdl,'Physical A and S positions/1','Toolhead S relative to A/1','autorouting','on');
add_line(mdl,'Toolhead S relative to A/1','Toolhead position sensor input/1','autorouting','on');
add_line(mdl,'Toolhead equilibrium offset/1','Toolhead position sensor input/2','autorouting','on');
add_line(mdl,'Toolhead position sensor input/1','Magnet position measurement/1','autorouting','on');
set_param([mdl '/Magnet position measurement/Measurement delay'], ...
 'InitialCondition','p.mechanics.midTravel-p.d.u0');
observer=[mdl '/Force controller/Voice coil observer'];
set_param([observer '/Measured position deviation'],'Bias','-p.mechanics.midTravel+p.d.u0');
set_param([observer '/Send estimated states'],'TagVisibility','global');
add_block('simulink/Signal Routing/From',[mdl '/Receive states for inferred coil gap'], ...
 'GotoTag','estimated_states','Position',[80 1270 235 1295]);
add_block('simulink/Signal Routing/Selector',[mdl '/Estimated coil gap deviation'], ...
 'NumberOfDimensions','1','IndexMode','One-based','IndexOptionArray',{'Index vector (dialog)'}, ...
 'Indices','5','InputPortWidth','9','Position',[270 1260 355 1305]);
add_block('simulink/Math Operations/Bias',[mdl '/Estimated absolute coil gap'], ...
 'Bias','p.mechanics.midTravel+p.d.u0','Position',[390 1260 580 1305]);
add_block('simulink/Signal Routing/Goto',[mdl '/Send inferred coil gap'], ...
 'GotoTag','inferred_coil_gap','TagVisibility','global','Position',[620 1270 770 1295]);
add_line(mdl,'Receive states for inferred coil gap/1','Estimated coil gap deviation/1','autorouting','on');
add_line(mdl,'Estimated coil gap deviation/1','Estimated absolute coil gap/1','autorouting','on');
add_line(mdl,'Estimated absolute coil gap/1','Send inferred coil gap/1','autorouting','on');
set_param([mdl '/Receive actuator_travel_m 6'],'GotoTag','inferred_coil_gap');
add_block('simulink/Sinks/To Workspace',[mdl '/Toolhead observer audit log'], ...
 'VariableName','sensor_state_log','SaveFormat','Timeseries','MaxDataPoints','inf', ...
 'SampleTime','p.force.sampleTime','Position',[270 1370 430 1410]);
add_line(mdl,'Receive states for inferred coil gap/1','Toolhead observer audit log/1','autorouting','on');
add_block('simulink/Signal Routing/From',[mdl '/Receive sensed toolhead position for audit'], ...
 'GotoTag','measured_magnet_position','Position',[80 1470 235 1495]);
add_block('simulink/Sinks/To Workspace',[mdl '/Toolhead position audit log'], ...
 'VariableName','sensor_position_log','SaveFormat','Timeseries','MaxDataPoints','inf', ...
 'SampleTime','p.force.sampleTime','Position',[270 1460 430 1500]);
add_line(mdl,'Receive sensed toolhead position for audit/1','Toolhead position audit log/1','autorouting','on');
end

function p=physical_sensor_audit_parameters(p)
p.force.reference=50;p.mechanics.angle=pi/2;p.surface.kind='flat';p.sim.stopTime=.08;p.sim.evaluateFrom=0;
p.coil.currentResponse=.0001;p.sensor.filterTime=.0001;p.sim.step=.000025;
p.sensor.distalMass_kg=1.4;p.sensor.distalMass_model_kg=1.4;p.sensor.positionModel=1;
p.sensor.noiseRms=0;p.sensor.strokeNoiseRms=0;p.sensor.delay=0;p.sensor.strokeDelay=.0002;
end

function restore_physical(mdl)
% These expressions change only this experiment process's loaded snapshot.
path=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil/'];
set_param([path 'Coil force constant'],'Gain','p.coil.forceConstant');
set_param([path 'Back EMF'],'Gain','p.coil.forceConstant');
set_param([path 'Current regulator'],'Gain','p.coil.inductance/p.coil.currentResponse');
path=[mdl '/Actuator and RABS mechanics/B to S spring and damper/'];
set_param([path 'Stiffness'],'Gain','p.mechanics.k1');
set_param([path 'Damping'],'Gain','p.mechanics.c1');
end

function finish(mdl,originalSDILocation)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;
Simulink.sdi.setStorageLocation(originalSDILocation);
end

function report=qualify_actual_rigid(sourceFolder,outputFolder,phase,candidateFile)
% Frozen, corrected two-sensor candidate. Reuses its saved native model and
% exact helpers without adding an alternate plant or modifying shared files.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch worker.');
assert(~isempty(candidateFile),'The exact frozen candidate MAT is required.');
maxNumCompThreads(2);
workerFolder=fullfile(fileparts(sourceFolder),['rigid_sensor_worker_' phase]);
for name={'','helpers','cache','code','sdi','traces'}
 path=fullfile(workerFolder,name{1});if ~isfolder(path),mkdir(path);end
end
if ~isfolder(outputFolder),mkdir(outputFolder);end
modelFile=fullfile(workerFolder,'acf_force_control_simple.slx');
if ~isfile(modelFile)
 copyfile(fullfile(sourceFolder,'acf_force_control_simple.slx'),modelFile);
 copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
 copyfile(candidateFile,fullfile(workerFolder,'frozen_candidate.mat'));
end
addpath(fullfile(workerFolder,'helpers'),'-begin');clear acf_prepare acf_run;
saved=load(fullfile(workerFolder,'frozen_candidate.mat'),'p');p=acf_prepare(saved.p);
assert(p.mechanics.lowerRigid && p.mechanics.upperRigid && p.coil.directCoulombCorrection);
assert((strcmp(optional(p.sensor,'positionTarget',''),'tool_relative_A') || optional(p.sensor,'positionModel',0)==1) && p.sensor.distalMass_kg==1.4);
assert(numel(p.d.coilControl.force)==9,'This recipe qualifies the frozen nine-state controller.');
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
originalSDILocation=Simulink.sdi.getStorageLocation;
Simulink.sdi.setStorageLocation(fullfile(workerFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_simple';assert(~bdIsLoaded(mdl));load_system(modelFile);
set_param(mdl,'FastRestart','off');
% Gate on reconstructed contact error, not the upstream load-cell reading.
% The estimated gap already exists in the frozen predictor architecture.
observer=[mdl '/Force controller/Voice coil observer'];
gateFrom=[mdl '/Receive force error for robot settling'];
if ~strcmp(phase,'final_rigid_dense') && ~strcmp(get_param(gateFrom,'GotoTag'),'inferred_contact_error_N')
 assert(getSimulinkBlockHandle([observer '/Physical estimated force deviation'])~=-1);
 add_block('simulink/Math Operations/Gain',[observer '/Contact error for slow force gate'], ...
  'Gain','-1','Position',[1320 810 1440 855]);
 add_block('simulink/Signal Routing/Goto',[observer '/Send reconstructed contact error'], ...
  'GotoTag','inferred_contact_error_N','TagVisibility','global','Position',[1490 810 1690 855]);
 add_line(observer,'Physical estimated force deviation/1','Contact error for slow force gate/1','autorouting','on');
 add_line(observer,'Contact error for slow force gate/1','Send reconstructed contact error/1','autorouting','on');
 set_param(gateFrom,'GotoTag','inferred_contact_error_N');
end
if ~strcmp(phase,'final_rigid_dense')
 assert(strcmp(get_param([mdl '/Receive inferred actuator travel'],'GotoTag'),'inferred_actuator_travel'));
end
% Saving this experiment-only copy makes final replay of the gate correction
% inspectable. The source model and the user's model remain untouched.
if ~strcmp(phase,'final_rigid_dense'),save_system(mdl,modelFile);end
cleanup=onCleanup(@()finish(mdl,originalSDILocation)); %#ok<NASGU>
rows=struct([]);started=tic;previousElapsed=0;resumeRows=0;worst=[];worstError=-inf;
fineWorst=[];clean=[];
stem=['rigid_sensor_qualification_' phase];checkpointFile=fullfile(outputFolder,[stem '.mat']);
report=struct('date','2026-09-30','phase',phase,'status','running', ...
 'recipeVersion','2026-09-30_actual_BS_toolA_corrected_coulomb_v1', ...
 'scope','Native Simulink: 1 mm x 10 mm sharp bump at 100 mm/s, B+S=2 kg, true contact force within +/-0.5 N throughout the full 4 s run.', ...
 'measurementScope','Actual B/S load cell with 1.4 kg downstream and exact static gravity tare; direct tool S relative to A position. Only noisy/delayed force and position, issued commands, and calibrated actuator/robot models enter control. Estimated Coulomb friction corrects both force measurement prediction and reconstructed contact force.', ...
 'slowTravelMeasurement','Estimated absolute coil gap from the same two-sensor observer; no actual travel truth.', ...
 'sourceFolder',sourceFolder,'privateModel',modelFile,'candidateFile',candidateFile, ...
 'matlabVersion',version,'baseParameters',p,'rows',rows);
if strcmp(phase,'final_rigid_dense')
 report.recipeVersion='2026-09-30_final_native_two_sensor_gate100ms_v1';
 report.nativeSourceUnmodified=true;
 report.slowTravelMeasurement='Parent-validated native reconstruction from the permitted two sensors: tool-relative-A position and B/S force; estimated coil travel and reconstructed contact error feed recentering. Source model/helpers are copied and run unchanged.';
end
if isfile(checkpointFile)
 prior=load(checkpointFile,'report');
 if strcmp(prior.report.status,'running')
  report=prior.report;rows=report.rows;resumeRows=numel(rows);previousElapsed=report.elapsedSeconds;
  worstError=max([rows.peakForceError_N]);report.resumeCount=optional(report,'resumeCount',0)+1;
  fprintf('ACTUAL_RIGID_RESUME %s %d cases.\n',phase,resumeRows);
 else,error('A completed qualification already exists; preserve it and choose a new phase.');end
end
configs={};labels={};
switch phase
 case {'actual_rigid_dense','actual_rigid_dense_gate100ms','final_rigid_dense'}
  for seed=[0 7 29 1729 31415]
   for ref=5:5:50
    for angle=0:15:90
     one=p;one.force.reference=ref;one.mechanics.angle=angle*pi/180;
     if seed==0,one.sensor.noiseRms=0;one.sensor.strokeNoiseRms=0;one.sensor.seed=1729;
     else,one.sensor.noiseRms=.02;one.sensor.strokeNoiseRms=1e-6;one.sensor.seed=seed;end
     configs{end+1}=one;labels{end+1}='dense_full_range_actual_BS_toolA'; %#ok<AGROW>
    end
   end
  end
  report.rangeForce_N=[5 50];report.rangeAngle_deg=[0 90];report.rangeMesh=[10 7];
  report.noiseSeeds=[7 29 1729 31415];
  report.qualificationCaveat='350 sampled force/orientation/noise cases with calibrated timing and exact gravity tare; not a continuous-range or physical-hardware proof.';
 case 'actual_rigid_calibration'
  for modelMass=[1.386 1.4 1.414]
   for ref=[5 50]
    for angle=[0 90]
     for seed=[7 29 1729 31415]
      one=p;one.force.reference=ref;one.mechanics.angle=angle*pi/180;one.sensor.seed=seed;
      one.sensor.distalMass_model_kg=modelMass;
      configs{end+1}=one;labels{end+1}='modeled_distal_mass_plus_minus_1_percent'; %#ok<AGROW>
     end
    end
   end
  end
  % The plant's robot timing changes; predictor assumptions remain frozen.
  for timing=[.07 .08 .09 .08 .08 .07 .09;.06 .06 .06 .054 .066 .054 .066]
   for ref=[5 50]
    for angle=[0 90]
     for seed=[7 29 1729 31415]
      one=p;one.force.reference=ref;one.mechanics.angle=angle*pi/180;one.sensor.seed=seed;
      one.robot.delay=timing(1);one.robot.responseTime=timing(2);
      configs{end+1}=one;labels{end+1}='actual_robot_timing_frozen_predictor'; %#ok<AGROW>
     end
    end
   end
  end
  report.qualificationCaveat='Independent modeled distal-mass error and selected actual robot timing mismatches. The gravity tare stays exact; the actuator and controller assumptions stay frozen. No joint-tolerance guarantee.';
 case 'actual_rigid_mass_tight'
  for modelMass=[1.3972 1.3986 1.4014 1.4028]
   for ref=[5 50]
    for angle=[0 90]
     for seed=[7 29 1729 31415]
      one=p;one.force.reference=ref;one.mechanics.angle=angle*pi/180;one.sensor.seed=seed;
      one.sensor.distalMass_model_kg=modelMass;
      configs{end+1}=one;labels{end+1}='modeled_distal_mass_plus_minus_0p1_0p2_percent'; %#ok<AGROW>
     end
    end
   end
  end
  report.qualificationCaveat='Tighter modeled downstream-mass error only: +/-0.1% and +/-0.2%, four force/orientation corners and four noisy seeds per model. Physical mass and exact static gravity tare remain unchanged. No joint calibration-error proof.';
 otherwise,error('Unknown actual-rigid phase: %s',phase);
end
report.plannedCases=numel(configs);
set_param(mdl,'FastRestart','on');lastTiming=[];
for j=resumeRows+1:numel(configs)
 one=configs{j};timing=[one.robot.delay one.robot.responseTime];
 if ~isequal(lastTiming,timing)
  set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');lastTiming=timing;
 end
 [row,r]=one_run(mdl,one,3,0,labels{j},j);
 row.positionTarget=optional(one.sensor,'positionTarget','tool_relative_A');row.coulombCorrectionEnabled=one.coil.directCoulombCorrection;
 row.actualRobotDelay_s=one.robot.delay;row.actualRobotResponse_s=one.robot.responseTime;
 row.predictedRobotDelay_s=one.robotPrediction.delay;row.predictedRobotResponse_s=one.robotPrediction.responseTime;
 row.gateFilterTime_s=one.robot.forceGate.filterTime;row.effectiveRecenterGain_s_inv=NaN;
 row.gateReadyFraction=NaN;row.gateEverReady=false;row.gateOpeningTransitions=0;
 row.firstGateReadyTime_s=NaN;row.gateForceStableFraction=NaN;row.maxRecenterRequest_m_s=NaN;
 if ~isempty(r)
  row.effectiveRecenterGain_s_inv=r.parameters.d.recenterGain;
  if isfield(r,'robotGate')
   ready=r.robotGate.ready;row.gateReadyFraction=mean(ready);row.gateEverReady=any(ready);
   row.gateOpeningTransitions=sum(diff(double(ready))>0);
   first=find(ready,1);if ~isempty(first),row.firstGateReadyTime_s=r.robotGate.time(first);end
   row.gateForceStableFraction=mean(r.robotGate.stableNow);
  end
  nominal=one.robot.nominalVelocity*(double(r.time>=1)-2*double(r.time>=2)+double(r.time>=3));
  row.maxRecenterRequest_m_s=max(abs(r.data(:,34)-nominal));
 end
 rows=append(rows,row);
 if ~isempty(r)
  if row.peakForceError_N>worstError,worstError=row.peakForceError_N;worst=r;end
  if any(one.force.reference==[5 50]) && any(round(one.mechanics.angle*180/pi)==[0 90]) && any(strcmp(phase,{'actual_rigid_dense','actual_rigid_dense_gate100ms','final_rigid_dense'}))
   save(fullfile(workerFolder,'traces',sprintf('dense_%03d.mat',j)),'r');
  end
 end
 checkpoint_actual();
 if ~row.executed
  report.status='execution_setup_failed';checkpoint_actual();
  error('acf:qualification:SetupFailure','Case %d returned no validated native metrics: %s',j,row.reason);
 end
end
if isempty(worst) && ~isempty(rows)
 [~,ix]=max([rows.peakForceError_N]);one=configs{ix};r=[];
 quiet=evalc('r=acf_run(mdl,one,false);'); %#ok<NASGU>
 worst=r;assert(abs(worst.metrics.peakAbsForceError_N-worstError)<1e-9);
 report.trajectoryRecoverySimulations=1;
end
report.worstError_N=worstError;report.worstMargin_N=p.force.tolerance-worstError;
if ~isempty(worst),report.worstCaseParameters=worst.parameters;report.worstCaseMetrics=worst.metrics;end
if strcmp(phase,'final_rigid_dense') && ~isempty(worst)
 one=worst.parameters;one.sim.step=.000025;set_param(mdl,'FastRestart','off');
 [fineRow,fineWorst]=one_run(mdl,one,3,0,'final_force_worst_25us',numel(rows)+1);
 report.additionalFineStepSimulations=double(fineRow.executed);report.fineWorstCaseRow=fineRow;
 if ~isempty(fineWorst)
  report.fineWorstCaseMetrics=fineWorst.metrics;
  ix=round(worst.time/one.sim.step)+1;
  assert(max(abs(worst.time-fineWorst.time(ix)))<1e-10,'Fine and coarse time grids differ.');
  report.fineCommonTrueForceDifference_N=max(abs(worst.data(:,1)-fineWorst.data(ix,1)));
 end
 entry=load(fullfile(workerFolder,'traces','dense_070.mat'),'r');clean=entry.r;
 report.selectedRunRoles={'force_worst_50us','force_worst_25us','clean_50N_90deg_50us'};
 fprintf('FINAL_RIGID_FINE: %.9g N at 25 us, pass=%d.\n',fineRow.peakForceError_N,fineRow.passed);
end
report.status='complete';checkpoint_actual();
selectedRuns={worst};
if strcmp(phase,'final_rigid_dense'),selectedRuns={worst,fineWorst,clean};end
save(fullfile(outputFolder,[stem '_traces.mat']),'selectedRuns','report','p');
fprintf('ACTUAL_RIGID_COMPLETE %s: %d/%d pass, worst %.9g N, %.1f s.\n', ...
 phase,report.passed,report.simulations,worstError,report.elapsedSeconds);

 function checkpoint_actual()
  report.rows=rows;report.attempts=numel(rows);report.simulations=sum([rows.executed]);
  report.rejectedBeforeSimulation=sum(~[rows.executed]);report.passed=sum([rows.passed]);
  report.allPlannedCasesExecuted=report.simulations==report.plannedCases;
  report.gridQualificationPassed=strcmp(report.status,'complete') && report.allPlannedCasesExecuted && all([rows.passed]);
  report.elapsedSeconds=previousElapsed+toc(started);
  fid=fopen(fullfile(outputFolder,[stem '.json']),'w');assert(fid>=0);
  fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  writetable(struct2table(rows),fullfile(outputFolder,[stem '.csv']));
  save(checkpointFile,'report');
  if mod(numel(rows),8)==0,Simulink.sdi.clear;end
  if mod(numel(rows),8)==0 || numel(rows)==1
   fprintf('ACTUAL_RIGID_PROGRESS %s: %d/%d cases, %d pass, last %.7g N, %.1f s.\n', ...
    phase,numel(rows),numel(configs),report.passed,rows(end).peakForceError_N,report.elapsedSeconds);
  end
 end
end
