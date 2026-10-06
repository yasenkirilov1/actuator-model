function evidence=qualify_saved_model(mdl)
%QUALIFY_SAVED_MODEL Audit native structure, replay regression and failure stops.
% Run after the final diagram edits. No diagram is rebuilt or saved here.
root=fileparts(fileparts(mfilename('fullpath')));folder=fullfile(root,'validation');
labels=acf_signals;ix=@(name)find(strcmp(labels,name),1);
mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
cleanup=onCleanup(@()restore(mdl,mw,original)); %#ok<NASGU>
allBlocks=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'Type','Block');
types=cellfun(@(b)get_param(b,'BlockType'),allBlocks,'UniformOutput',false);
assert(~any(ismember(types,{'S-Function','MATLABFcn'})),'A hidden code block remains.');
assert(isempty(find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'MaskType','MATLAB Function')),'A MATLAB Function block remains.');
systems=[{mdl};find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','SubSystem')];
pages=struct([]);
for j=1:numel(systems)
 s=systems{j};blocks=find_system(s,'SearchDepth',1,'MatchFilter',@Simulink.match.allVariants,'Type','Block');blocks(strcmp(blocks,s))=[];
 pos=cell2mat(cellfun(@(b)get_param(b,'Position'),blocks,'UniformOutput',false));overlap=0;wireHits=0;
 for a=1:numel(blocks)
  for b=a+1:numel(blocks)
   overlap=overlap+(max(pos(a,1),pos(b,1))<min(pos(a,3),pos(b,3)) && max(pos(a,2),pos(b,2))<min(pos(a,4),pos(b,4)));
  end
 end
 lines=find_system(s,'FindAll','on','SearchDepth',1,'Type','line');
 for k=1:numel(lines)
  pts=get_param(lines(k),'Points');
  for a=1:numel(blocks)
   box=pos(a,:)+[2 2 -2 -2];hit=false;
   for t=1:size(pts,1)-1
    v=pts(t:t+1,:);
    if v(1,2)==v(2,2),hit=v(1,2)>box(2)&&v(1,2)<box(4)&&max(v(:,1))>box(1)&&min(v(:,1))<box(3);
    elseif v(1,1)==v(2,1),hit=v(1,1)>box(1)&&v(1,1)<box(3)&&max(v(:,2))>box(2)&&min(v(:,2))<box(4);end
    if hit,break;end
   end
   wireHits=wireHits+hit;
  end
 end
 row=struct('system',erase(s,[mdl '/']),'blocks',numel(blocks),'overlaps',overlap,'wireThroughBlocks',wireHits);
 if isempty(pages),pages=row;else,pages(end+1)=row;end %#ok<AGROW>
end
assert(all([pages.overlaps]==0) && all([pages.wireThroughBlocks]==0),'Layout intersections remain.');
loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);assert(isempty(loops),'An algebraic loop remains.');
inputs=struct([]);n=0;
for path={[mdl '/Force controller'],[mdl '/Recenter travel'],[mdl '/Issued-command robot predictor']}
 from=find_system(path{1},'MatchFilter',@Simulink.match.allVariants,'BlockType','From');
 for j=1:numel(from)
  tag=get_param(from{j},'GotoTag');assert(~ismember(tag,{'plant_log','surface_motion','robot_log'}),'Plant truth enters a control subsystem.');
  n=n+1;row=struct('block',erase(from{j},[mdl '/']),'tag',tag);
  if isempty(inputs),inputs=row;else,inputs(end+1)=row;end %#ok<AGROW>
 end
end
assert(strcmp(get_param([mdl '/Send plant_log'],'TagVisibility'),'global'));
assert(strcmp(get_param([mdl '/Send surface_motion'],'TagVisibility'),'global'));
load(fullfile(folder,'rigid_sensor_qualification_final_rigid_dense_traces.mat'),'selectedRuns');
golden=selectedRuns{1};q=golden.parameters;
timer=tic;actual=acf_run(mdl,q,false);elapsed=toc(timer);
assert(isequal(actual.time,golden.time),'Golden sample grid changed.');
difference=max(abs(actual.data-golden.data),[],'all');fprintf('GOLDEN_ALL35_DIFF %.17g\n',difference);assert(difference<1e-8,'Final model differs from independent qualification.');
observerDifference=max(abs(actual.observerStates.data-golden.observerStates.data),[],'all');assert(observerDifference<1e-8);
forceResidual=max(abs(actual.data(:,ix('Fc_N'))-actual.data(:,ix('actuatorAxialForce_N'))-(q.mechanics.mB+q.mechanics.mS)*(actual.parameters.d.g+actual.data(:,ix('surfaceAcceleration_m_s2')))));
positionResidual=max(abs([actual.data(:,5)-actual.data(:,26),actual.data(:,6)-actual.data(:,7),actual.data(:,7)-actual.data(:,28)]),[],'all');
assert(forceResidual<1e-8 && positionResidual<1e-8,'Rigid physical identities failed.');
actual.label='Actual B–S/S–A sensors / worst sharp bump / 50 N, 60 degrees';
actual.source='Final saved native model, calibrated nine-state controller, 100 ms force gate';
clean=selectedRuns{3};clean.label='Actual B–S/S–A sensors / clean sharp bump / 50 N, 90 degrees';
clean.source='Independent byte-identical frozen qualification source';
acf_visualize({actual,clean},fullfile(root,'generated','sharp_bump_and_spring_sweep.html'),false);
regression=struct('all35MaxDifference',difference,'observerStateMaxDifference',observerDifference, ...
 'forceIdentityResidual_N',forceResidual,'rigidPositionResidual_m',positionResidual,'fullLoggingRuntime_s',elapsed, ...
 'forcePeakError_N',actual.metrics.peakAbsForceError_N,'nativeSampleCount',numel(actual.time));
clear actual golden selectedRuns clean;
% Retuned screw settings change an unused angular-speed diagnostic in coil runs.
q=getVariable(mw,'pRecommended');retuned=acf_run(mdl,q,false);
q.screw.lead=.005;q.screw.rotorInertia=2e-5;q.screw.speedBandwidth=150;
q.screw.peakCurrent=3;q.screw.voltage=24;q.screw.forceKp=.02;q.screw.forceKi=.008;
reference=acf_run(mdl,q,false);delta=max(abs(retuned.data-reference.data),[],1);
changed=find(delta>1e-8);assert(numel(changed)<=1 && all(changed==ix('motorSpeed_rad_s')), ...
 'Retuned unused screw settings affect physical coil behavior.');
regression.unusedScrewSettingDiagnosticChanges=struct('signal',{retuned.names(changed)},'maxDifference',delta(changed));
regression.physicalCoilSignalDifference=max(delta(setdiff(1:35,ix('motorSpeed_rad_s'))));
clear retuned reference;
Simulink.sdi.clear;
cases=struct([]);
for j=1:5
 q=getVariable(mw,'pRecommended');q.surface.kind='flat';q.robot.nominalVelocity=0;q.sim.stopTime=.5;
 if j==1,q.coil.strategy='observer';label='voice_coil_observer';
 elseif j==2,q.coil.strategy='pi';label='voice_coil_pi';
 elseif j==3,q.coil.strategy='pid';label='voice_coil_pid';
 elseif j==4,q=acf_parameters('leadscrew','flat','compliant');q.robot.nominalVelocity=0;q.sim.stopTime=.5;label='leadscrew';
 else,q=acf_parameters('ideal_position','flat','compliant');q.robot.nominalVelocity=0;q.sim.stopTime=.5;label='ideal_position';end
 r=acf_run(mdl,q,false);assert(r.metrics.completed && ~r.metrics.lostContact && ~r.metrics.strokeStopHit,'An actuator/strategy smoke check failed.');
 row=struct('case',label,'sampleCount',numel(r.time),'metrics',r.metrics);
 if isempty(cases),cases=row;else,cases(end+1)=row;end %#ok<AGROW>
 clear r;Simulink.sdi.clear;
end
q=getVariable(mw,'pRecommended');q.force.reference=5;q.mechanics.angle=pi/2;
q.surface.startTime=.02;q.surface.height=.004;q.surface.width=.001;q.sim.stopTime=.2;
r=acf_run(mdl,q,false);assert(r.metrics.lostContact && ~r.metrics.completed && ~r.metrics.passed,'Contact-loss stop failed.');
contact=struct('endTime_s',r.time(end),'metrics',r.metrics);clear r;Simulink.sdi.clear;
q=getVariable(mw,'pRecommended');q.surface.kind='flat';q.sim.stopTime=2;
q.robot.initialPositionOffset_m=.0124;q.robot.nominalVelocity=.02;q.robot.recenterGain=0;
q.robot.forceGate.enabled=false;q.robotPrediction.boundedNominalPath=false;
r=acf_run(mdl,q,false);assert(r.metrics.strokeStopHit && ~r.metrics.completed && ~r.metrics.passed,'Travel-stop failure gate failed.');
stroke=struct('endTime_s',r.time(end),'metrics',r.metrics);clear r;Simulink.sdi.clear;
evidence=struct('date','2026-09-30','status','complete','modelFile',get_param(mdl,'FileName'), ...
 'nativeBlocks',numel(allBlocks),'hiddenCodeBlocks',0,'algebraicLoops',numel(loops), ...
 'maximumSubsystemDepth',max(cellfun(@(s)numel(strfind(s,'/')),systems)), ...
 'pages',pages,'controllerFromInputs',inputs,'independentGoldenRegression',regression, ...
 'actuatorStrategySmokeCases',cases,'contactLossStop',contact,'travelStop',stroke);
fid=fopen(fullfile(folder,'final_saved_model_results.json'),'w');assert(fid~=-1);
fprintf(fid,'%s',jsonencode(evidence,PrettyPrint=true));fclose(fid);
fprintf('FINAL_NATIVE_SAVED_MODEL_QUALIFIED\n');
end
function restore(mdl,mw,p)
assignin(mw,'p',p);acf_select_actuator(mdl,p.actuator.kind);
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
end
