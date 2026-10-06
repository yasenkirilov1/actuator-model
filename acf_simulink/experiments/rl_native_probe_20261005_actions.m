function report=rl_native_probe_20261005_actions(folder)
% Independent native external-action and prepared-bank endpoint comparisons.
assert(~usejava('desktop'));maxNumCompThreads(2);
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
if nargin<1,folder=fullfile(root,'acf_simulink','validation','rl_native_probe_20261005_bank');end
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'parity_cache'),'CodeGenFolder',fullfile(folder,'parity_code'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_rl_actions_sdi'),'w');fprintf(fid,'Owned RL native actions.\n');fclose(fid);
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(fullfile(folder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
bank=load(fullfile(root,'acf_simulink','validation','rl_unified_20261005','scenario_bank','configs.mat'),'configs');
cases=jsondecode(fileread(fullfile(folder,'additional_c_cases.json')));report=struct('status','running_native_action_checks','cases',struct([]));
set_param(mdl,'SaveFormat','Dataset','SaveOutput','on','SaveTime','on','ReturnWorkspaceOutputs','on','LimitDataPoints','off');
for k=1:numel(cases)
 row=cases(k);p=acf_prepare(bank.configs(row.scenario+1).p);p.sim.stopTime=row.endTime;
 assignin(get_param(mdl,'ModelWorkspace'),'p',p);set_param(mdl,'StopTime',num2str(row.endTime,17));
 requests=load(fullfile(folder,[row.recordName '_requests.mat']));inputs=requests.requests;
 inputs(end+1,:)=inputs(end,:);inputs(end,1)=row.endTime;
 dataset=Simulink.SimulationData.Dataset;
 for j=1:3
  ts=timeseries(inputs(:,j+1),inputs(:,1));ts=setinterpmethod(ts,'zoh');dataset=dataset.addElement(ts);
 end
 in=Simulink.SimulationInput(mdl);in=in.setExternalInput(dataset);timer=tic;out=sim(in);wall=toc(timer);
 nativeTime=out.yout.get(2).Values.Time;truth=rows(out.yout.get(2).Values,27);
 observation=rows(out.yout.get(1).Values,18);observationTime=out.yout.get(1).Values.Time;
 c=load(fullfile(folder,[row.recordName '_c_record.mat']));assert(isequal(size(truth),size(c.truth)));
 difference=max(abs(truth-c.truth),[],1);
 check=struct('scenario',row.scenario,'mode',row.baseline,'nativeWall_s',wall,'physicalSamples',numel(nativeTime), ...
  'timeMaximumDifference_s',max(abs(nativeTime(:)-c.time(:))),'allTruthMaximumDifference',max(difference), ...
  'truthPerChannelMaximumDifference',difference,'nativeNormalPeak_N',max(abs(truth(:,1)-p.force.reference)), ...
  'cNormalPeak_N',max(abs(c.truth(:,1)-p.force.reference)),'nativeContactLost',any(truth(:,19)<.5), ...
  'nativeTravelStop',any(truth(:,22)>.5));
 if isempty(report.cases),report.cases=check;else,report.cases(end+1)=check;end
 write_json(fullfile(folder,'native_action_parity.json'),report);
 save(fullfile(folder,[row.recordName '_native_run.mat']),'nativeTime','truth','observation','observationTime','-v7');
 assert(check.timeMaximumDifference_s<1e-12&&check.allTruthMaximumDifference<1e-8);
 Simulink.sdi.clear;clear out c truth observation;
end
report.status='independent_native_external_actions_and_bank_endpoints_passed';write_json(fullfile(folder,'native_action_parity.json'),report);
fprintf('RL_NATIVE_ACTION_PARITY_PASSED %s\n',jsonencode(report));
end
function d=rows(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_rl_actions_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function write_json(path,value)
fid=fopen(path,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true)); %#ok<NASGU>
end
