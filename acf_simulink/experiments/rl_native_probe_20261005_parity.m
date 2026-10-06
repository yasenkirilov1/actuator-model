function report=rl_native_probe_20261005_parity(folder)
% Independent native MATLAB Run against the exported C context and source.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin<1,folder=fullfile(root,'acf_simulink','validation','rl_native_probe_20261005_bank');end
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'parity_cache'),'CodeGenFolder',fullfile(folder,'parity_code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_rl_parity_sdi'),'w');fprintf(fid,'Owned RL native parity.\n');fclose(fid);
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(fullfile(folder,[mdl '.slx']));
cleanup=onCleanup(@()finish({mdl,'acf_force_control_adc_trial'},oldStorage,oldRecord,sdi)); %#ok<NASGU>
bank=load(fullfile(root,'acf_simulink','validation','rl_unified_20261005','scenario_bank','configs.mat'),'configs');p=acf_prepare(bank.configs(1).p);p.sim.stopTime=.4;
assignin(get_param(mdl,'ModelWorkspace'),'p',p);set_param(mdl,'StopTime','.4','SaveFormat','Dataset','SaveOutput','on','SaveTime','on','ReturnWorkspaceOutputs','on','LimitDataPoints','off');
input=[0 p.d.coilHoldCurrent 0 1;.4 p.d.coilHoldCurrent 0 1];
in=Simulink.SimulationInput(mdl);in=in.setExternalInput(input);timer=tic;out=sim(in);wall=toc(timer);
truth=rows(out.yout.get(2).Values,27);nativeTime=out.yout.get(2).Values.Time;
c=load(fullfile(folder,'baseline_c_record.mat'));assert(isequal(size(truth),size(c.truth)),'Native truth size%s versus C%s.',mat2str(size(truth)),mat2str(size(c.truth)));
delta=abs(truth-c.truth);
report=struct('status','parity_checks_running','nativeBaselineWall_s',wall,'physicalSamples',numel(nativeTime), ...
 'timeMaximumDifference_s',max(abs(nativeTime(:)-c.time(:))),'allTruthMaximumDifference',max(delta,[],'all'), ...
 'truthPerChannelMaximumDifference',max(delta,[],1),'nativeNormalPeak_N',max(abs(truth(:,1)-p.force.reference)), ...
 'cNormalPeak_N',max(abs(c.truth(:,1)-p.force.reference)), ...
 'privateModelSha256',sha256(fullfile(folder,[mdl '.slx'])),'pMatchesBankCase',1);
observation=rows(out.yout.get(1).Values,18);obsTime=out.yout.get(1).Values.Time;
save(fullfile(folder,'baseline_native_ported.mat'),'nativeTime','truth','observation','obsTime','-v7');
write_json(fullfile(folder,'native_parity.json'),report);
Simulink.sdi.clear;clear out truth observation;
% Source graph has original fixed array queues and complete native histories.
sourceFile=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002','acf_force_control_adc_trial.slx');
original='acf_force_control_adc_trial';load_system(sourceFile);assignin(get_param(original,'ModelWorkspace'),'p',p);
set_param(original,'FixedStep',num2str(p.sim.step,17),'StopTime','.4','SimulationMode','normal','SignalLogging','off');
timer=tic;originalOut=sim(original);report.nativeOriginalWall_s=toc(timer);
originalPlant=rows(originalOut.get('plant_log'),25);normal=rows(originalOut.get('normal_contact_log'),8);
delta=abs(originalPlant-c.truth(:,2:26));report.originalPlantMaximumDifference=max(delta,[],'all');
report.originalNormalMaximumDifference_N=max(abs(normal(:,1)-c.truth(:,1)));
report.originalNormalPeak_N=max(abs(normal(:,1)-p.force.reference));
write_json(fullfile(folder,'native_parity.json'),report);
assert(report.timeMaximumDifference_s<1e-12&&report.allTruthMaximumDifference<1e-8);
assert(report.originalPlantMaximumDifference<1e-8&&report.originalNormalMaximumDifference_N<1e-8);
report.status='native_codegen_and_original_graph_baseline_parity_passed';
write_json(fullfile(folder,'native_parity.json'),report);
fprintf('RL_NATIVE_PARITY_PASSED %s\n',jsonencode(report));
end
function d=rows(ts,width)
n=numel(ts.Time);d=ts.Data;
if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function finish(models,storage,record,sdi)
for k=1:numel(models),try,close_system(models{k},0);catch,end,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_rl_parity_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function write_json(path,value)
fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true)); %#ok<NASGU>
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
