function report=qualify_normal_mass_controller(sourceFolder,outputFolder,phase,candidateFile)
%QUALIFY_NORMAL_MASS_CONTROLLER Private native normal-force mass/control study.
% Requires the completed conditional multi-axis prototype builder output.
% No plant replacement, canonical-model save, current/acceleration truth input,
% or known surface profile is introduced by this harness.
assert(~usejava('desktop'),'Use an isolated MATLAB -batch process.');
project=fileparts(fileparts(mfilename('fullpath')));
if nargin<2,outputFolder=fullfile(project,'validation');end
if nargin<3,phase='screen';end
if nargin<4,candidateFile=fullfile(sourceFolder,'normal_contact_trial_candidate.mat');end
assert(isfolder(sourceFolder) && isfile(candidateFile),'Wait for usable prototype/model/candidate files.');
stem=['normal_mass_controller_' phase '_20260930'];
resultFile=fullfile(outputFolder,[stem '_results.json']);
assert(~isfile(resultFile),'Preserve prior evidence; use a fresh output folder for repetitions.');
maxNumCompThreads(2);
workerFolder=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','helpers','cache','code','sdi'}
 folder=fullfile(workerFolder,name{1});if ~isfolder(folder),mkdir(folder);end
end
if ~isfolder(outputFolder),mkdir(outputFolder);end
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
modelHash=sha256(sourceModel);candidateHash=sha256(candidateFile);
modelFile=fullfile(workerFolder,'acf_force_control_normal_trial.slx');copyfile(sourceModel,modelFile);
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile(candidateFile,fullfile(workerFolder,'candidate.mat'));
recipeSource=[mfilename('fullpath') '.m'];copyfile(recipeSource,fullfile(workerFolder,'qualify_normal_mass_controller.m'));
addpath(fullfile(workerFolder,'helpers'),'-begin');clear acf_prepare acf_run acf_metrics;
files=dir(fullfile(workerFolder,'helpers','*.m'));sources=struct([]);
for j=1:numel(files)
 one=struct('file',files(j).name,'sha256',sha256(fullfile(files(j).folder,files(j).name)));
 if isempty(sources),sources=one;else,sources(end+1)=one;end %#ok<AGROW>
end
saved=load(fullfile(workerFolder,'candidate.mat'),'p');p=saved.p;
p.sim.stopTime=4;p.sim.evaluateFrom=0;p.sim.step=50e-6;
p.robot.forceGate.filterTime=.1;p.robot.forceGate.dwellTime=.1;
assert(p.mechanics.lowerRigid && p.mechanics.upperRigid && p.mechanics.mB==.6);
assert(strcmp(p.sensor.positionTarget,'tool_relative_A') && p.robotPrediction.enabled);
assert(p.coil.directCoulombCorrection && p.force.sampleTime==100e-6);
assert(isfield(p,'contact') && p.contact.lateralDelaySamples==1);
p=acf_prepare(p);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
Simulink.sdi.setStorageLocation(fullfile(workerFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,fullfile(workerFolder,'sdi'))); %#ok<NASGU>
observer=[mdl '/Force controller/Voice coil observer'];
fromBlocks=find_system(observer,'MatchFilter',@Simulink.match.allVariants,'BlockType','From');
tags=cellfun(@(block)get_param(block,'GotoTag'),fromBlocks,'UniformOutput',false);
assert(~any(ismember(tags,{'plant_log','surface_motion','normal_contact_N','lateral_contact_N','actual_contact_slope'})), ...
 'Physical geometry/contact truth enters the controller.');
configs=struct([]);keys={};
switch phase
 case 'screen'
  % All mechanical endpoint corners at the builder's current control settings.
  for mass=[.1 2]
   for reference=[5 50]
    for angle=[0 90]
     for mu=[0 .3]
      for seed=[0 29]
       q=physical_case(p,mass,reference,angle,mu,seed);
       add_config(q,'baseline_endpoint');
      end
     end
    end
   end
  end
  % Equal eight-case tuning cohort. Reuse matching baseline configurations.
  for forceScale=[.16 .32 .5]
   for jerkPower=[70 300 1000]
    for mass=[.1 2]
     if mass==.1,reference=5;else,reference=50;end
     for angle=[0 90]
      for seed=[0 29]
       q=physical_case(p,mass,reference,angle,.3,seed);
       q.coil.control.forceScale_N=forceScale;q.coil.control.surfaceJerkPower=jerkPower;
       add_config(q,'common_weight_search');
      end
     end
    end
   end
  end
 case 'confirm_endpoints'
  for mass=[.1 .5 1 1.4 2]
   for reference=[5 50]
    for angle=[0 90]
     for mu=[0 .3]
      for seed=[0 29]
       q=physical_case(p,mass,reference,angle,mu,seed);
       add_config(q,'five_mass_endpoint_confirmation');
      end
     end
    end
   end
  end
 case 'full_range'
  for mass=[.1 .5 1 1.4 2]
   for reference=5:5:50
    for angle=0:15:90
     for mu=[0 .3]
      for seed=[0 29]
       q=physical_case(p,mass,reference,angle,mu,seed);
       add_config(q,'five_mass_full_force_angle_grid');
      end
     end
    end
   end
  end
 case 'identified_current_response'
  for tau=[50e-6 100e-6 200e-6]
   for mass=[.1 2]
    for reference=[5 50]
     for angle=[0 90]
      for mu=[0 .3]
       for seed=[0 29]
        q=physical_case(p,mass,reference,angle,mu,seed);
        q.coil.currentResponse=tau;q.sim.step=25e-6;
        add_config(q,'identified_current_response');
       end
      end
     end
    end
   end
  end
 case {'response_screen','response_screen_v2'}
  for tau=[50e-6 100e-6 200e-6]
   for mass=[.1 2]
    if mass==.1,reference=5;else,reference=50;end
    for angle=[0 90]
     for seed=[0 29 1729]
      q=physical_case(p,mass,reference,angle,.3,seed);
      q.coil.currentResponse=tau;q.sim.step=25e-6;
      add_config(q,'identified_current_response_noise_screen');
     end
    end
   end
  end
 otherwise,error('Unknown phase: %s',phase);
end
rows=struct([]);plotTraces=struct([]);started=tic;lastCompileKey=[];worst=[];worstError=-inf;
report=struct('date','2026-09-30','status','running','phase',phase, ...
 'recipeVersion','conditional_multi_axis_normal_mass_v1', ...
 'scope','Surface-normal pressing force within +/-0.5 N for the entire 0-4 s run; 1 mm x10 mm bump at100 mm/s. Finite conditional simulation, not physical qualification.', ...
 'sourceModel',sourceModel,'modelSha256',modelHash,'privateModel',modelFile, ...
 'candidateFile',candidateFile,'candidateSha256',candidateHash,'helperSources',sources, ...
 'experimentSha256',sha256(mfilename('fullpath')),'matlabVersion',version, ...
 'experimentSnapshot',fullfile(workerFolder,'qualify_normal_mass_controller.m'), ...
 'baseParameters',p,'plannedCases',numel(configs),'controllerFromTags',{tags},'rows',rows, ...
 'massDefinition','S=.1–2 kg tool downstream of B-S cell; B=.6 kg fixed. A=.4 kg additional. Identified S and distal compensation change together; gravity tare stays exact.', ...
 'sensing','Conditional axial+lateral B-S force sensor and S-A position; each force axis .02 N sample RMS and known100 us filter/delay; position1 um RMS/100 us known delay;10 kHz fast controller;100 ms stable-force robot gate.', ...
 'controllerExclusions','No current readback, measured acceleration, true slope, known bump profile, friction coefficient, normal-force truth or future robot command enters control.', ...
 'qualificationLimit','Force/orientation/mass/friction/noise grid is sampled; no joint calibration tolerances, contact compliance, varying friction, changing tool orientation or full robot compliance dynamics are proven.', ...
 'metrics','Normal force scoring uses every native sample and entire recording. Original35 axial metrics are diagnostic; their axial tracking requirement is not the new requirement.');
for j=1:numel(configs)
 config=configs(j);q=config.p;
 compileKey=[q.sim.step,q.coil.currentResponse,q.sensor.filterTime,q.sensor.delay,q.sensor.strokeDelay];
 if ~isequal(lastCompileKey,compileKey)
  set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');lastCompileKey=compileKey;
 end
 timer=tic;r=[];exception='';nativeReturned=false;
 try
  quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  nativeReturned=~isempty(r);
  assert(isfield(r,'normalContact') && size(r.normalContact.data,2)==8,'Usable normal log contract missing.');
  assert(isequal(r.time,r.normalContact.time),'Normal and original native log grids differ.');
 catch failure
  exception=getReport(failure,'extended','hyperlinks','off');r=[];
  fprintf('NORMAL_MASS_SETUP_FAILURE %d %s\n',j,failure.message);
 end
 row=measure_run(q,r,j,config,toc(timer),exception);
 row.nativeReturned=nativeReturned;
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 if ~isempty(r)
  if row.peakNormalForceError_N>worstError,worstError=row.peakNormalForceError_N;worst=r;end
  % Full native trace for first case and force-worst so far; explicitly reduced
  % excerpts for corner cases are convenient for plots, never scoring.
  if j==1,save(fullfile(outputFolder,[stem '_first_native_trace.mat']),'r','row','-v7');end
  if (q.sensor.noiseRms==0 || q.mechanics.mS==2) && ...
     (j<=32 || (round(q.mechanics.angle*180/pi)==90 && q.sensor.seed==29))
   trace=plot_excerpt(r,row);
   if isempty(plotTraces),plotTraces=trace;else,plotTraces(end+1)=trace;end %#ok<AGROW>
  end
 end
 clear r;checkpoint('running');
 if mod(j,4)==0 || j==1
  fprintf('NORMAL_MASS_PROGRESS %s %d/%d, %d passed: S%.2g,F%.0f,ang%.0f,mu%.1f,noise%.3g; %.9g N,elapsed%.1fs\n', ...
   phase,j,numel(configs),sum([rows.passed]),row.massS_kg,row.reference_N,row.angle_deg,row.mu, ...
   row.forceNoiseRms_N,row.peakNormalForceError_N,toc(started));
 end
 if mod(j,8)==0,Simulink.sdi.clear;end
 if ~row.executed
  checkpoint('setup_failure');error('acf:normalStudy:ContractFailure','Case%d had no validated normal metrics: %s',j,exception);
 end
 if isfile(fullfile(workerFolder,'stop_after_case'))
  if ~isempty(worst),save(fullfile(outputFolder,[stem '_worst_native_trace.mat']),'worst','report','-v7');end
  checkpoint('stopped_by_coordinator');
  fprintf('NORMAL_MASS_STOPPED_AFTER_CASE %d/%d.\n',j,numel(configs));return;
 end
end
report.sourceUnmodified=strcmp(sha256(modelFile),modelHash);
report.upstreamSourceStillMatches=strcmp(sha256(sourceModel),modelHash);
assert(report.sourceUnmodified,'Private frozen native input changed during qualification.');
if ~isempty(worst),report.worstCaseParameters=worst.parameters;report.worstNativeMetrics=worst.metrics;end
if strcmp(phase,'screen')
 ranking=rank_weights(rows);report.weightRanking=ranking;
 if ~isempty(ranking)
  chosen=ranking(1);q=physical_case(p,1.4,20,0,0,29);
  q.coil.control.forceScale_N=chosen.forceScale_N;q.coil.control.surfaceJerkPower=chosen.surfaceJerkPower;
  q=acf_prepare(q);pSelected=q; %#ok<NASGU>
  save(fullfile(outputFolder,[stem '_selected_candidate.mat']),'pSelected','chosen');
  % Reusable candidate contract contains a p field like the native builder.
  p=q;save(fullfile(outputFolder,[stem '_candidate.mat']),'p','chosen');
  report.selectedCandidate=fullfile(outputFolder,[stem '_candidate.mat']);
  report.selectionScope='Best worst normal error on equal eight-case tuning cohort only. It must still pass all endpoint and full-range confirmation cases.';
 end
elseif ismember(phase,{'response_screen','response_screen_v2'})
 taus=unique([rows.currentResponse_s]);ranking=struct([]);
 for k=1:numel(taus)
  selected=rows([rows.currentResponse_s]==taus(k));score=max([selected.peakNormalForceError_N]);
  if ~all([selected.completed]),score=inf;end
  one=struct('currentResponse_s',taus(k),'cases',numel(selected),'passedCases',sum([selected.passed]), ...
   'worstNormalForceError_N',score,'minimumForceMargin_N',.5-score,'rowIds',[selected.id]);
  if isempty(ranking),ranking=one;else,ranking(end+1)=one;end %#ok<AGROW>
 end
 [~,order]=sort([ranking.worstNormalForceError_N]);ranking=ranking(order);report.currentResponseRanking=ranking;
 chosen=ranking(1);q=physical_case(p,1.4,20,0,0,29);q.coil.currentResponse=chosen.currentResponse_s;
 q.sim.step=min(50e-6,q.coil.currentResponse/2);q=acf_prepare(q);p=q;
 save(fullfile(outputFolder,[stem '_candidate.mat']),'p','chosen');
 report.selectedCandidate=fullfile(outputFolder,[stem '_candidate.mat']);
 report.selectionScope='Identified current response, equal12-case endpoint/noise cohort at25 us integration. Remaining masses, force/angle/friction range and calibration tolerances still require qualification.';
end
if ~isempty(worst),save(fullfile(outputFolder,[stem '_worst_native_trace.mat']),'worst','report','-v7');end
checkpoint('complete');
fprintf('NORMAL_MASS_COMPLETE %s %d/%d executed,%d passed,worst%.9g N.\n',phase,report.executed,report.plannedCases,report.passed,worstError);

 function add_config(q,scenario)
  key=sprintf('%.17g,',[q.mechanics.mS,q.force.reference,q.mechanics.angle,q.contact.mu,q.sensor.seed, ...
   q.sensor.noiseRms,q.sensor.strokeNoiseRms,q.contact.lateralNoiseRms_N, ...
   q.coil.control.forceScale_N,q.coil.control.surfaceJerkPower,q.coil.currentResponse,q.sim.step, ...
   q.contact.normalIntegralScale,q.contact.geometryFeedforwardScale]);
  if any(strcmp(keys,key)),return;end
  one=struct('p',q,'scenario',scenario,'key',key);keys{end+1}=key;
  if isempty(configs),configs=one;else,configs(end+1)=one;end %#ok<AGROW>
 end
 function checkpoint(status)
  report.status=status;report.rows=rows;report.executed=sum([rows.executed]);
  report.validatedNativeHistories=report.executed;report.nativeReturned=sum([rows.nativeReturned]);
  report.attempts=numel(rows);report.failedBeforeNativeReturn=sum(~[rows.nativeReturned]);
  report.passed=sum([rows.passed]);report.rejectedBeforeMetrics=sum(~[rows.executed]);
  report.elapsedSeconds=toc(started);report.worstNormalForceError_N=max([rows.peakNormalForceError_N]);
  fid=fopen(resultFile,'w');assert(fid~=-1);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
  writetable(struct2table(rows,'AsArray',true),fullfile(outputFolder,[stem '_rows.csv']));
  save(fullfile(outputFolder,[stem '_results.mat']),'report','configs','-v7');
  save(fullfile(outputFolder,[stem '_plot_traces.mat']),'plotTraces','-v7');
 end
end

function q=physical_case(p,mass,reference,angle,mu,seed)
q=p;q.mechanics.mS=mass;q.mechanics.mB=.6;
q.sensor.distalMass_kg=mass;q.sensor.distalMass_model_kg=mass;
q.force.reference=reference;q.mechanics.angle=angle*pi/180;q.contact.mu=mu;
q.sensor.seed=seed;if seed==0,q.sensor.seed=29;end
q.sensor.noiseRms=.02;q.sensor.strokeNoiseRms=1e-6;
q.contact.lateralNoiseRms_N=.02;q.contact.lateralDelaySamples=1;q.contact.lateralBias_N=0;
if seed==0,q.sensor.noiseRms=0;q.sensor.strokeNoiseRms=0;q.contact.lateralNoiseRms_N=0;end
end

function row=measure_run(q,r,id,config,runtime,exception)
row=struct('id',id,'scenario',config.scenario,'massS_kg',q.mechanics.mS,'massB_kg',q.mechanics.mB, ...
 'totalBS_kg',q.mechanics.mS+q.mechanics.mB,'reference_N',q.force.reference, ...
 'angle_deg',q.mechanics.angle*180/pi,'mu',q.contact.mu,'seed',q.sensor.seed, ...
 'forceNoiseRms_N',q.sensor.noiseRms,'positionNoiseRms_m',q.sensor.strokeNoiseRms, ...
 'lateralNoiseRms_N',q.contact.lateralNoiseRms_N,'forceFilterTime_s',q.sensor.filterTime, ...
 'forceDelay_s',q.sensor.delay,'positionDelay_s',q.sensor.strokeDelay, ...
 'currentResponse_s',q.coil.currentResponse,'simulationStep_s',q.sim.step, ...
 'forceScale_N',q.coil.control.forceScale_N,'surfaceJerkPower',q.coil.control.surfaceJerkPower, ...
 'normalIntegralScale',q.contact.normalIntegralScale,'geometryFeedforwardScale',q.contact.geometryFeedforwardScale, ...
 'voltageLimit_V',q.coil.voltage,'currentLimit_A',q.coil.peakCurrent, ...
 'executed',~isempty(r),'passed',false,'completed',false,'lostContact',false,'strokeStop',false, ...
 'runtime_s',runtime,'endTime_s',NaN,'nativeSamples',0,'peakNormalForceError_N',NaN, ...
 'rmsNormalForceError_N',NaN,'peakNormalForceErrorTime_s',NaN,'peakBeforeBumpError_N',NaN, ...
 'peakDuringBumpError_N',NaN,'peakAfterBumpError_N',NaN,'peakNormalEstimateTrackingError_N',NaN, ...
 'peakNormalEstimateError_N',NaN,'rmsNormalEstimateError_N',NaN,'peakSlopeEstimateError',NaN, ...
 'peakAxialDiagnosticError_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN, ...
 'voltageClipFraction',NaN,'effortClipFraction',NaN,'minTravel_mm',NaN,'maxTravel_mm',NaN, ...
 'gateReadyFraction',NaN,'normalBalanceResidual_N',NaN,'lateralBalanceResidual_N',NaN, ...
 'rigidAxialBalanceResidual_N',NaN,'axialDiagnosticReason','','reason',exception);
if isempty(r),return;end
metrics=r.metrics;normal=r.normalContact.data;error=normal(:,1)-q.force.reference;
[row.peakNormalForceError_N,which]=max(abs(error));row.peakNormalForceErrorTime_s=r.time(which);
row.rmsNormalForceError_N=sqrt(mean(error.^2));row.completed=metrics.completed;
row.lostContact=metrics.lostContact;row.strokeStop=metrics.strokeStopHit;
row.passed=metrics.completed && row.peakNormalForceError_N<=q.force.tolerance && ...
 ~metrics.lostContact && ~metrics.strokeStopHit && ~metrics.overCurrent && metrics.impactCount==0;
row.endTime_s=r.time(end);row.nativeSamples=numel(r.time);
start=q.surface.startTime;stop=start+r.parameters.d.duration;
row.peakBeforeBumpError_N=masked_peak(error,r.time<start);
row.peakDuringBumpError_N=masked_peak(error,r.time>=start & r.time<=stop);
row.peakAfterBumpError_N=masked_peak(error,r.time>stop);
row.peakNormalEstimateTrackingError_N=max(abs(normal(:,4)-q.force.reference));
row.peakNormalEstimateError_N=max(abs(normal(:,4)-normal(:,1)));
row.rmsNormalEstimateError_N=sqrt(mean((normal(:,4)-normal(:,1)).^2));
row.peakSlopeEstimateError=max(abs(normal(:,5)-normal(:,3)));
row.peakAxialDiagnosticError_N=metrics.peakAbsForceError_N;
row.peakCurrent_A=metrics.peakCurrent_A;row.rmsCurrent_A=metrics.rmsCurrent_A;
row.peakVoltage_V=metrics.peakVoltage_V;row.voltageClipFraction=metrics.voltageSaturationFraction;
row.effortClipFraction=metrics.effortSaturationFraction;
row.minTravel_mm=metrics.minTravel_mm;row.maxTravel_mm=metrics.maxTravel_mm;
actualSlope=r.data(:,29)/q.surface.scanSpeed;
expectedNormal=r.data(:,1).*sqrt(1+actualSlope.^2)./(1-q.contact.mu*actualSlope);
expectedLateral=-expectedNormal.*(actualSlope+q.contact.mu)./sqrt(1+actualSlope.^2);
row.normalBalanceResidual_N=max(abs(expectedNormal-normal(:,1)));
row.lateralBalanceResidual_N=max(abs(expectedLateral-normal(:,2)));
row.rigidAxialBalanceResidual_N=max(abs(r.data(:,1)-r.data(:,13)- ...
 (q.mechanics.mB+q.mechanics.mS)*(r.parameters.d.g+r.data(:,30))));
assert(max([row.normalBalanceResidual_N,row.lateralBalanceResidual_N,row.rigidAxialBalanceResidual_N])<1e-8, ...
 'Native normal/lateral/axial force balances failed.');
if isfield(r,'robotGate'),row.gateReadyFraction=mean(r.robotGate.ready);end
row.axialDiagnosticReason=metrics.reason;
reasons={};if row.peakNormalForceError_N>q.force.tolerance,reasons{end+1}='normal force error';end
if metrics.lostContact,reasons{end+1}='contact loss';end
if metrics.strokeStopHit,reasons{end+1}='travel stop';end
if metrics.overCurrent,reasons{end+1}='overcurrent';end
if ~metrics.completed,reasons{end+1}='incomplete';end
if isempty(reasons),row.reason='pass';else,row.reason=strjoin(reasons,', ');end
end

function rankings=rank_weights(rows)
rankings=struct([]);settings=unique([[rows.forceScale_N]',[rows.surfaceJerkPower]'],'rows');
for j=1:size(settings,1)
 use=abs([rows.forceScale_N]-settings(j,1))<1e-12 & [rows.surfaceJerkPower]==settings(j,2) & ...
  [rows.mu]==.3 & (([rows.massS_kg]==.1 & [rows.reference_N]==5) | ([rows.massS_kg]==2 & [rows.reference_N]==50));
 cohort=rows(use);if numel(cohort)~=8,continue;end
 score=max([cohort.peakNormalForceError_N]);if ~all([cohort.executed]) || ~all([cohort.completed]),score=inf;end
 one=struct('forceScale_N',settings(j,1),'surfaceJerkPower',settings(j,2), ...
  'cohortCases',numel(cohort),'passedCases',sum([cohort.passed]),'worstNormalForceError_N',score, ...
  'minimumForceMargin_N',.5-score,'rowIds',[cohort.id]);
 if isempty(rankings),rankings=one;else,rankings(end+1)=one;end %#ok<AGROW>
end
if ~isempty(rankings),[~,order]=sort([rankings.worstNormalForceError_N]);rankings=rankings(order);end
end

function value=masked_peak(values,mask)
if any(mask),value=max(abs(values(mask)));else,value=NaN;end
end

function trace=plot_excerpt(r,row)
stride=max(1,round(250e-6/r.parameters.sim.step));take=1:stride:numel(r.time);
trace=struct('row',row,'plotStep_s',stride*r.parameters.sim.step,'time_s',r.time(take), ...
 'normalLog',r.normalContact.data(take,:),'current_A',r.data(take,14),'voltage_V',r.data(take,15), ...
 'axialContactForce_N',r.data(take,1),'taredAxialCell_N',r.data(take,32));
end

function result=sha256(file)
if ~isfile(file),file=[file '.m'];end
% MATLAB on this host has no bundled JVM. Use the local hashing utility;
% single-quote escaping keeps arbitrary paths inert in the shell command.
assert(isfile(file),'Hash input missing.');single=char(39);
escaped=strrep(file,single,[single,char(34),single,char(34),single]);
[status,output]=system(['sha256sum -- ',single,escaped,single]);
assert(status==0,'Hash utility failed.');result=regexp(output,'[0-9a-f]{64}','match','once');
assert(~isempty(result),'Invalid SHA-256 output.');
end

function finish(mdl,previousStorage,previousRecord,ownedStorage)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
if strcmp(Simulink.sdi.getStorageLocation,ownedStorage),Simulink.sdi.clear;end
Simulink.sdi.setStorageLocation(previousStorage);Simulink.sdi.setRecordData(previousRecord);
end
