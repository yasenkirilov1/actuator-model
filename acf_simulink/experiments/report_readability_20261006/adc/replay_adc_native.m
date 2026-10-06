function report=replay_adc_native(storageRoot,outputFolder)
% Independent MATLAB replay of two actual slower-ADC native diagnostics.
assert(~usejava('desktop'));maxNumCompThreads(2);
assert(~isfolder(outputFolder));mkdir(outputFolder);
addpath(fullfile(storageRoot,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal;
old=Simulink.sdi.getStorageLocation;record=Simulink.sdi.getRecordData;
sdi=fullfile(outputFolder,'owned_sdi');mkdir(sdi);fclose(fopen(fullfile(sdi,'owned_adc_replay'),'w'));
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@()finish(old,record,sdi)); %#ok<NASGU>
specs={'sinc3_15000',5,170;'sinc3_7500',6,171};rows=struct([]);
for j=1:size(specs,1)
 group=specs{j,1};index=specs{j,2};source=specs{j,3};base=fullfile(storageRoot,group);
 own=fullfile(outputFolder,group);mkdir(own);
 Simulink.fileGenControl('set','CacheFolder',fullfile(own,'cache'),'CodeGenFolder',fullfile(own,'code'),'createDir',true);
 configsFile=fullfile(base,'bank','configs.mat');b=load(configsFile,'configs');p=acf_prepare(b.configs(index).p);
 assert(p.mechanics.mA==1.5&&p.mechanics.mS==2&&p.coil.currentResponse==50e-6&&p.coil.voltage==32);
 modelFile=fullfile(own,'acf_rl_native.slx');copyfile(fullfile(base,'acf_rl_native.slx'),modelFile);
 mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
 assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 set_param(mdl,'StopTime','2','FixedStep',num2str(p.sim.step,17),'Solver','ode4', ...
 'SimulationMode','normal','SignalLogging','off','SaveFormat','Dataset','SaveOutput','on', ...
 'SaveTime','on','ReturnWorkspaceOutputs','on','LimitDataPoints','off');
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 dataset=Simulink.SimulationData.Dataset;
 for k=1:3,ts=timeseries([double(k==3);double(k==3)],[0;2]);ts=setinterpmethod(ts,'zoh');dataset=dataset.addElement(ts);end
 in=Simulink.SimulationInput(mdl);in=in.setExternalInput(dataset);timer=tic;out=sim(in);elapsed=toc(timer);
 ts=out.yout.get(2).Values;nativeTime=ts.Time;truth=sample_rows(ts,27);
 history=fullfile(own,'native_recording.mat');save(history,'nativeTime','truth','p','-v7');
 [peak,k]=max(abs(truth(:,1)-p.force.reference));
 row=struct('group',group,'sourceScenario',source,'nativeSamples',numel(nativeTime),'nativeLastTime_s',nativeTime(end), ...
 'normalForcePeakError_N',peak,'peakErrorTime_s',nativeTime(k),'contactLoss',any(truth(:,19)<.5), ...
 'travelStop',any(truth(:,22)>.5),'algebraicLoopCount',loops,'wallTime_s',elapsed, ...
 'historyFile',history,'historySha256',sha256(history),'configsFile',configsFile,'configsSha256',sha256(configsFile), ...
 'modelSourceFile',fullfile(base,'acf_rl_native.slx'),'modelSourceSha256',sha256(fullfile(base,'acf_rl_native.slx')));
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
 close_system(mdl,0);clear out truth b;Simulink.sdi.clear;
end
report=struct('status','independent_matlab_adc_replays_executed_pending_paired_comparison', ...
 'diagnosticReplays',numel(rows),'qualificationRecordings',0,'controllerRetuned',false,'rows',rows, ...
 'recipeSha256',sha256([mfilename('fullpath') '.m']));
write_json(fullfile(outputFolder,'replay_results.json'),report);fprintf('ADC_MATLAB_REPLAYS %s\n',jsonencode(report));
end
function d=sample_rows(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end;d=reshape(d,n,width);
end
function finish(storage,record,sdi)
try,close_system('acf_rl_native',0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_adc_replay')),try,rmdir(sdi,'s');catch,end,end
end
function h=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end;h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(path,value)
fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
