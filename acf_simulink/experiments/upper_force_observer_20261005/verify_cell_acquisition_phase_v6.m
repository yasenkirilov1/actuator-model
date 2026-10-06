function report=verify_cell_acquisition_phase_v6(buildFolder,outputFolder)
% Sinusoidal acquisition wiring harness, excluded from force qualification.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
metadata=jsondecode(fileread(fullfile(buildFolder,'metadata.json')));loaded=load(metadata.configsFile,'configs');
addpath(fullfile(buildFolder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
p=loaded.configs(1).p;p.adcStudy.forceAdcNoiseRms_N=0;p=acf_prepare(p);
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'),'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
modelFile=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),modelFile);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;sdi=tempname('/dev/shm');mkdir(sdi);
fid=fopen(fullfile(sdi,'owned_cell_phase_sdi'),'w');assert(fid>=0);fprintf(fid,'Owned cell-phase wiring harness.\n');fclose(fid);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
sys=[mdl '/Force measurement/Engineered common AA and EMA'];target=[sys '/Continuous analog front end'];
ports=get_param(target,'PortHandles');original=get_param(get_param(ports.Inport(1),'Line'),'SrcPortHandle');
delete_line(sys,original,ports.Inport(1));stim=[sys '/Owned raw axial cell phase stimulus'];
add_block('simulink/Sources/Sine Wave',stim,'Amplitude','.1','Bias','p.force.reference', ...
 'Frequency','1','Phase','0','SampleTime','0','Position',[20 -200 350 -140]);
source=get_param(stim,'PortHandles');add_line(sys,source.Outport(1),ports.Inport(1),'autorouting','on');
head=[mdl '/Contact force measurement/Head motion estimator'];
add_block('simulink/Sinks/To Workspace',[head '/Owned delivered cell phase log'], ...
 'VariableName','owned_delivered_cell_phase','SaveFormat','Timeseries','MaxDataPoints','inf', ...
 'SampleTime','p.force.sampleTime','Position',[2100 1700 2450 1750]);
add_line(head,'Cell force deviation from equilibrium/1','Owned delivered cell phase log/1','autorouting','on');
f0=sqrt(p.inertial.lowerLoadInverse.identifiedStiffness_N_m/p.uncertainty.massB_model_kg)/(2*pi);
rows=struct([]);duration=.03;discard=.005;
for frequency=[1300,f0,1900]
 set_param(stim,'Frequency',num2str(2*pi*frequency,17));
 input=[0,0,0,1;duration,0,0,1];assignin('base','owned_cell_phase_external_input',input);
 output=sim(mdl,'StopTime',num2str(duration,17),'SimulationMode','normal', ...
  'LoadExternalInput','on','ExternalInput','owned_cell_phase_external_input','ReturnWorkspaceOutputs','on');
 data=output.owned_delivered_cell_phase;t=double(data.Time(:));y=double(data.Data(:));use=t>=discard;
 assert(numel(t)>200&&all(isfinite(y))&&all(abs(diff(t)-p.force.sampleTime)<1e-10));
 fit=[sin(2*pi*frequency*t(use)),cos(2*pi*frequency*t(use)),ones(sum(use),1)];beta=fit\y(use);
 complexGain=(beta(1)+1i*beta(2))/.1;residual=y(use)-fit*beta;
 row=struct('frequency_Hz',frequency,'amplitudeAtRawAnalogInput_N',.1, ...
  'measuredTransferReal',real(complexGain),'measuredTransferImag',imag(complexGain), ...
  'measuredGain',abs(complexGain),'measuredPhase_deg',angle(complexGain)*180/pi, ...
  'fitResidualRms_N',sqrt(mean(residual.^2)),'fitMaximumResidual_N',max(abs(residual)), ...
  'nativeFrameSampleTime_s',p.force.sampleTime,'fittedFromTime_s',discard, ...
  'actualTimes_s',t,'actualDeliveredDeltaL_N',y,'fittedCoefficientSinCosOffset',beta);
 if isempty(rows),rows=row;else,rows(end+1)=row;end
 progress=struct('status','running_native_cell_phase_harness','qualificationEpisodes',0,'rows',rows);
 write_json(fullfile(outputFolder,'progress.json'),progress);
 assert(abs(complexGain)>.1&&abs(complexGain)<1.1&&sqrt(mean(residual.^2))<5e-5);
 Simulink.sdi.clear;
end
report=struct('status','native_delivered_cell_acquisition_phase_measured','qualificationEpisodes',0, ...
 'sinusoidalWiringHarnessCount',numel(rows),'harnessDuration_s',duration,'physicalSourceModelSha256',metadata.sourceModelSha256, ...
 'privateParentModelSha256',metadata.privateModelSha256,'parentConfigSha256',metadata.configsSha256, ...
 'source','Continuous analog front-end inlet of the actual existing axial-cell ADC chain, driven with Fref+0.1*sin(2*pi*f*t)', ...
 'sink','Actual delivered local axial deltaL at Head motion estimator/Cell force deviation from equilibrium, fresh10kHz', ...
 'harnessNoiseContract','Force analog noise bypassed by input stimulus; independent force ADC output noise zero only in harness; actual quantization retained. No candidate sensing change.', ...
 'scope','Actual native analog/sinc3/30kEMA/pipeline/10k clock-order transfer. Temporary input/log never saved into a candidate. These diagnostic wiring harnesses are not force qualification or physical hardware tests.', ...
 'rows',rows);
write_json(fullfile(outputFolder,'acquisition_phase.json'),report);copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'verify_cell_acquisition_phase_v6.m'));
fprintf('NATIVE_CELL_ACQUISITION_PHASE %.9fHz gain%.9f phase%.9fdeg\n',rows(2).frequency_Hz,rows(2).measuredGain,rows(2).measuredPhase_deg);
end
function finish(mdl,oldStorage,oldRecord,sdi)
if bdIsLoaded(mdl),close_system(mdl,0);end
evalin('base','clear owned_cell_phase_external_input');
Simulink.sdi.clear;Simulink.sdi.setRecordData(oldRecord);Simulink.sdi.setStorageLocation(oldStorage);
if isfolder(sdi)&&isfile(fullfile(sdi,'owned_cell_phase_sdi')),rmdir(sdi,'s');end
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
