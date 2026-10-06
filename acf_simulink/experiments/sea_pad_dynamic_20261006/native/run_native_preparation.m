function report=run_native_preparation(root)
assert(~usejava('desktop'));maxNumCompThreads(2);
here=fullfile(root,'acf_simulink','experiments','sea_pad_dynamic_20261006','native');addpath(here);
out=fullfile(root,'acf_simulink','validation','sea_pad_dynamic_20261006','native');
generated=fullfile(root,'acf_simulink','generated','sea_pad_dynamic_20261006');
forecast=jsondecode(fileread(fullfile(fileparts(out),'storage_forecast.json')));
assert(forecast.recorded_before_launch&&forecast.allocations.native.retained_bytes>=10485760);
contract=fullfile(fileparts(here),'study_contract.json');
temp=fullfile(out,'owned_temporary');if ~isfolder(temp),mkdir(temp);end
marker=fullfile(temp,'owned_sea_pad_native');fid=fopen(marker,'w');fprintf(fid,'Owned bounded native preparation/replay\n');fclose(fid);
storage=Simulink.sdi.getStorageLocation;record=Simulink.sdi.getRecordData;
sdi=fullfile(temp,'sdi');if ~isfolder(sdi),mkdir(sdi);end
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@()finish(storage,record,temp)); %#ok<NASGU>
p=sea_pad_native_params(contract,struct('stopTime',.02));
modelFile=fullfile(generated,'acf_sea_pad_research_20261006.slx');
report=sea_pad_native_build(p,modelFile,temp);[~,mdl]=fileparts(modelFile);load_system(modelFile);
assignin('base','replayInputs',[0 p.initial(9) 0;p.stopTime p.initial(9) 0]);
set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
simOut=sim(mdl);data=matrix(simOut.yout{1}.Values,numel(report.outputNames));t=simOut.tout;
assert(numel(t)>1&&size(data,1)==numel(t),'Native score requires executed output samples.');
forceDiff=abs(data(:,10)-p.Fref);stateDiff=abs(data(:,1:9)-p.initial);
forceError=max(forceDiff(:));stateError=max(stateDiff(:));
fprintf('NATIVE_EQUILIBRIUM_DIAGNOSTIC data=%s class=%s force=%s state=%s script=%s\n', ...
 mat2str(size(data)),class(data),mat2str(forceError),mat2str(stateError),mfilename('fullpath'));
assert(forceError<1e-8&&stateError<1e-8,'Loaded equilibrium check rejected.');
report.status='native_preparation_and_equilibrium_executed';report.equilibriumForceError_N=forceError;
report.equilibriumStateError=stateError;report.equilibriumScoredPoints=numel(t);report.algebraicLoopCount=loops;
report.modelSha256=sha256(modelFile);report.contractSha256=sha256(contract);
report.sourceSha256=struct('builder',sha256(fullfile(here,'sea_pad_native_build.m')), ...
 'parameters',sha256(fullfile(here,'sea_pad_native_params.m')));
report.nativeBlockTypes=unique(get_param(find_system(mdl,'LookUnderMasks','all','Type','block'),'BlockType'));
report.matlabFunctionBlockCount=numel(find_system(mdl,'LookUnderMasks','all','BlockType','MATLABFcn'));
report.matlabFunctionStateflowCount=numel(find_system(mdl,'LookUnderMasks','all','MaskType','MATLAB Function'));
assert(report.matlabFunctionBlockCount==0&&report.matlabFunctionStateflowCount==0);
save_system(mdl,modelFile);close_system(mdl,0);load_system(modelFile);
set_param(mdl,'SimulationCommand','update');report.savedReopened=true;close_system(mdl,0);
write_json(fullfile(out,'preparation.json'),report);
fprintf('SEA_PAD_NATIVE_PREPARED %s\n',jsonencode(report));
end
function finish(storage,record,temp)
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(temp,'owned_sea_pad_native')),try,rmdir(temp,'s');catch,end,end
end
function write_json(file,value),fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);end
function d=matrix(ts,width)
n=numel(ts.Time);d=ts.Data;if size(d,1)~=n,assert(size(d,ndims(d))==n);d=permute(d,[ndims(d),1:ndims(d)-1]);end
d=reshape(d,n,width);
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
