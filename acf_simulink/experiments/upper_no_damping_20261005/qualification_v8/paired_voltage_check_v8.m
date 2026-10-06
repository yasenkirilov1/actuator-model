function report=paired_voltage_check_v8(buildFolder,configsFile,outputFolder)
% Independent MATLAB 32/48 V counterfactual, with no controller retuning.
assert(~usejava('desktop'));maxNumCompThreads(2);
assert(~isfolder(outputFolder),'Preserve every prior voltage check.');mkdir(outputFolder);
addpath(fullfile(buildFolder,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal;
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'), ...
 'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_voltage_check'),'w');fprintf(fid,'Owned MATLAB voltage check.\n');fclose(fid);
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
modelFile=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),modelFile);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
bank=load(configsFile,'configs');selected=bank.configs(173).p;
assert(selected.mechanics.mA==1.5&&selected.mechanics.mS==2&&selected.force.reference==50);
assert(selected.mechanics.angle==pi/2&&selected.coil.currentResponse==50e-6);
assert(selected.cellModal.identifiedCurrentResponse_s==50e-6&&selected.sensor.seed==991007);
assert(selected.coil.voltage==32);reference=acf_prepare(selected);
voltageValues=[32,48];runs=cell(1,2);rowsOut=cell(1,2);
for j=1:2
 p=selected;p.coil.voltage=voltageValues(j);p=acf_prepare(p);
 assert(isequaln(p.d.cellModal,reference.d.cellModal));
 assert(isequaln(p.adcStudy,reference.adcStudy)&&isequaln(p.sensor,reference.sensor));
 assert(p.coil.currentResponse==reference.coil.currentResponse&&p.force.sampleTime==reference.force.sampleTime);
 assert(isequaln(p.upperObserver,reference.upperObserver)&&isequaln(p.inertial,reference.inertial));
 assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 set_param(mdl,'StopTime','2','FixedStep',num2str(p.sim.step,17),'Solver','ode4', ...
  'SimulationMode','normal','SignalLogging','off','SaveFormat','Dataset','SaveOutput','on', ...
  'SaveTime','on','ReturnWorkspaceOutputs','on','LimitDataPoints','off');
 dataset=Simulink.SimulationData.Dataset;
 for k=1:3
  ts=timeseries([double(k==3);double(k==3)],[0;2]);ts=setinterpmethod(ts,'zoh');dataset=dataset.addElement(ts);
 end
 in=Simulink.SimulationInput(mdl);in=in.setExternalInput(dataset);timer=tic;out=sim(in);wall=toc(timer);
 nativeTime=out.yout.get(2).Values.Time;truth=sample_rows(out.yout.get(2).Values,27);
 observationTime=out.yout.get(1).Values.Time;observation=sample_rows(out.yout.get(1).Values,18);
 comparatorTime=out.yout.get(3).Values.Time;comparator=sample_rows(out.yout.get(3).Values,3);
 assert(numel(nativeTime)==600001&&nativeTime(1)==0&&abs(nativeTime(end)-2)<1e-12);
 historyFile=fullfile(outputFolder,sprintf('usable_%02dV_native_recording.mat',voltageValues(j)));
 save(historyFile,'nativeTime','truth','observationTime','observation','comparatorTime','comparator','p','-v7');
 [peak,index]=max(abs(truth(:,1)-p.force.reference));
 rowsOut{j}=struct('usableVoltage_V',voltageValues(j),'nativeWall_s',wall, ...
  'sampleCount',numel(nativeTime),'normalForcePeakError_N',peak,'peakTime_s',nativeTime(index), ...
  'peakCurrent_A',max(abs(truth(:,15))),'peakVoltage_V',max(abs(truth(:,16))), ...
  'voltageClipSamples',sum(truth(:,24)>.5),'currentLimitSamples',sum(truth(:,23)>.5), ...
  'contactLost',any(truth(:,19)<.5),'travelStop',any(truth(:,22)>.5), ...
  'algebraicLoopCount',numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl)), ...
  'historyFile',historyFile,'historySha256',sha256(historyFile));
 runs{j}=struct('time',nativeTime,'truth',truth);
 clear out observation comparator;Simulink.sdi.clear;
end
assert(isequal(runs{1}.time,runs{2}.time));
differences=max(abs(runs{2}.truth-runs{1}.truth),[],1);
report=struct('status','independent_matlab_paired_voltage_check_complete', ...
 'qualificationEpisodes',0,'counterfactualDiagnosticRuns',2,'controllerRetuned',false, ...
 'scenario0',172,'seed',selected.sensor.seed,'duration_s',2, ...
 'actualCurrentResponse_s',50e-6,'identifiedModalCurrentResponse_s',50e-6, ...
 'rawNoiseClock_s',selected.adcStudy.fineClock_s,'rawAdcSampleTime_s',selected.adcStudy.outputSampleTime_s, ...
 'controlSampleTime_s',selected.force.sampleTime,'physicalStep_s',selected.sim.step, ...
 'acquisitionAndModalCoefficientsUnchanged',true, ...
 'sameNativeTimeArray',true,'maximumForceWaveformDifference_N',differences(1), ...
 'maximumCurrentWaveformDifference_A',differences(15),'maximumVoltageWaveformDifference_V',differences(16), ...
 'truthPerChannelMaximumDifference',differences,'runs',[rowsOut{:}], ...
 'sourceModelFile',fullfile(buildFolder,'acf_rl_native.slx'),'sourceModelSha256',sha256(fullfile(buildFolder,'acf_rl_native.slx')), ...
 'configsFile',configsFile,'configsSha256',sha256(configsFile),'privateModelSha256',sha256(modelFile));
report.voltageIncreaseChangesForceCurrentOrVoltage=any(differences([1,15,16])>0);
write_json(fullfile(outputFolder,'paired_voltage_check.json'),report);
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'paired_voltage_check_v8.m'));
fprintf('PAIRED_VOLTAGE_CHECK %s\n',jsonencode(report));
assert(all([report.runs.algebraicLoopCount]==0));
end
function d=sample_rows(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_voltage_check')),try,rmdir(sdi,'s');catch,end,end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
