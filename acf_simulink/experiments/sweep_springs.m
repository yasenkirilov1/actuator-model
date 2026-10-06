function [report,selectedRuns] = sweep_springs(outputFolder)
%SWEEP_SPRINGS Compact spring pairs x independent viscous damping values.
% Uses loaded native blocks with Fast Restart; restores settings without saving.
% [report,runs]=sweep_springs; acf_visualize(runs,'sweep.html',true,report);
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);
if nargin<1,outputFolder=fullfile(root,'validation');end
if ~isfolder(outputFolder),mkdir(outputFolder);end
catalog=jsondecode(fileread(fullfile(fileparts(mfilename('fullpath')),'spring_catalog.json')));
mdl=acf_open;mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
step=get_param(mdl,'FixedStep');stop=get_param(mdl,'StopTime');
choice=get_param([mdl '/Actuator selection'],'ActuatorChoice');restart=get_param(mdl,'FastRestart');
delayBlock=[mdl '/Force measurement/Sensor sample delay'];delay=get_param(delayBlock,'DelayLength');
cleanup=onCleanup(@() restore(mdl,original,step,stop,choice,restart,delayBlock,delay)); %#ok<NASGU>
set_param(mdl,'FastRestart','off');
p=acf_parameters('voice_coil','sharp_bump');p.mechanics.mB=.6;p.mechanics.mS=1.4;
p.surface.height=.001;p.surface.width=.010;p.surface.scanSpeed=.100;p.sim.stopTime=4;
p.sensor.noiseRms=0;p.sensor.seed=1729;
p.springs=struct('maxCombinedLength_m',.100,'commonPreload_N',10, ...
    'workingClearance_m',.0005,'unseatingClearance_m',.00025, ...
    'lower',catalog.springs(1),'upper',catalog.springs(1));
damping=[5 15 30 60];rows=struct([]);configs={};
total=numel(catalog.springs)^2*numel(damping)^2;started=tic;
% Keep the physical sensor-delay buffer fixed during each Fast Restart batch.
for phase=1:2
 set_param(mdl,'FastRestart','off');
 if phase==1,set_param(delayBlock,'DelayLength',delay);
 else,set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');end
 set_param(mdl,'FastRestart','on');number=0;
 for lower=1:numel(catalog.springs)
  for upper=1:numel(catalog.springs)
   p.springs.lower=catalog.springs(lower);p.springs.upper=catalog.springs(upper);
   p.mechanics.k1=2*p.springs.lower.rate_N_m;p.mechanics.k2=2*p.springs.upper.rate_N_m;
   for c1=damping
    for c2=damping
     p.mechanics.c1=c1;p.mechanics.c2=c2;q=p;
     if phase==2,q.sensor.noiseRms=.02;q.sensor.seed=7;end
     quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
     number=number+1;m=r.metrics;s=m.springSizing;
     if phase==1
      row=struct('id',number,'lowerPart',p.springs.lower.part,'upperPart',p.springs.upper.part, ...
       'k1_N_m',p.mechanics.k1,'k2_N_m',p.mechanics.k2,'c1_Ns_m',c1,'c2_Ns_m',c2, ...
       'nominalError_N',m.peakAbsForceError_N,'stressError_N',NaN, ...
       'worstError_N',m.peakAbsForceError_N,'margin_N',p.force.tolerance-m.peakAbsForceError_N, ...
       'passed',m.passed,'springSizingPassed',s.passed, ...
       'freeLengthMax_mm',s.combinedFreeLengthMax_mm,'installedLengthMax_mm',s.combinedInstalledLengthMax_mm, ...
       'peakCurrent_A',m.peakCurrent_A,'peakVoltage_V',m.peakVoltage_V, ...
       'peakDamperForce_N',max(s.lower.peakDamperForce_N,s.upper.peakDamperForce_N), ...
       'nominalReason',m.reason,'stressReason','pending');
      if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
      configs{number}=p; %#ok<AGROW>
     else
      row=rows(number);row.stressError_N=m.peakAbsForceError_N;
      row.worstError_N=max(row.nominalError_N,row.stressError_N);
      row.margin_N=p.force.tolerance-row.worstError_N;row.passed=row.passed && m.passed;
      row.springSizingPassed=row.springSizingPassed && s.passed;
      row.peakCurrent_A=max(row.peakCurrent_A,m.peakCurrent_A);row.peakVoltage_V=max(row.peakVoltage_V,m.peakVoltage_V);
      row.peakDamperForce_N=max([row.peakDamperForce_N,s.lower.peakDamperForce_N,s.upper.peakDamperForce_N]);
      row.stressReason=m.reason;rows(number)=row;
     end
     if mod(number,16)==0 || number==total
      fprintf('SPRING_SWEEP batch %d/2: %d/%d, elapsed %.1f s\n',phase,number,total,toc(started));
     end
    end
   end
  end
 end
end
set_param(mdl,'FastRestart','off');
valid=find([rows.passed]);assert(~isempty(valid),'No spring/damper candidate passed both tests.');
[~,j]=min([rows(valid).worstError_N]);selected=rows(valid(j));p=configs{selected.id};
set_param(delayBlock,'DelayLength',delay);quiet=evalc('nominal=acf_run(mdl,p,false);'); %#ok<NASGU>
noisy=p;noisy.sensor.noiseRms=.02;noisy.sensor.seed=7;
set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');quiet=evalc('stress=acf_run(mdl,noisy,false);'); %#ok<NASGU>
assert(abs(nominal.metrics.peakAbsForceError_N-selected.nominalError_N)<1e-10 && ...
    abs(stress.metrics.peakAbsForceError_N-selected.stressError_N)<1e-10, ...
    'Selected results differ between Fast Restart and ordinary execution.');
nominal.label=sprintf('Selected springs / clean force / k1 %.3g, k2 %.3g N/mm',selected.k1_N_m/1000,selected.k2_N_m/1000);
stress.label='Selected springs / 0.02 N force noise + extra 0.2 ms delay';
selectedRuns={nominal,stress};
report=struct('date','2026-09-29','matlabVersion',version,'configurations',total, ...
 'simulations',2*total,'passedConfigurations',sum([rows.passed]),'selected',selected,'rows',rows, ...
 'catalogue',catalog,'dampingValues_Ns_m',damping, ...
 'selectionRule','Smallest worst peak force error across nominal and noisy/delayed tests, with spring geometry and original electrical/travel/contact limits satisfied.', ...
 'controllerPolicy','Design weights fixed; observer and state-feedback coefficients recalculated for each spring/damper configuration.', ...
 'stressTest','0.02 N RMS force noise, seed 7, plus an extra unmodeled 0.2 ms force delay; 1 micrometer RMS magnet noise with calibrated 0.2 ms delay.', ...
 'parameters',nominal.parameters,'selectedNominalMetrics',nominal.metrics,'selectedStressMetrics',stress.metrics, ...
 'fastRestart',true,'selectedOrdinaryExecutionMatch',true,'elapsedSeconds',toc(started));
fid=fopen(fullfile(outputFolder,'spring_damper_sweep_results.json'),'w');assert(fid>=0);
fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
writetable(struct2table(rows),fullfile(outputFolder,'spring_damper_sweep_results.csv'));
save(fullfile(outputFolder,'spring_damper_selected_runs.mat'),'selectedRuns','report');
fprintf('SPRING_SWEEP_COMPLETE: %d/%d configurations passed; selected #%d, worst %.6g N.\n', ...
 sum([rows.passed]),total,selected.id,selected.worstError_N);
end

function restore(mdl,p,step,stop,choice,restart,delayBlock,delay)
set_param(mdl,'FastRestart','off');set_param(delayBlock,'DelayLength',delay);
assignin(get_param(mdl,'ModelWorkspace'),'p',p);acf_select_actuator(mdl,choice);
set_param(mdl,'FixedStep',step,'StopTime',stop,'FastRestart',restart);
end
