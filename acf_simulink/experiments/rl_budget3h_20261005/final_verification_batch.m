function final_verification_batch(planFile,outputFolder)
% Warm private batch waits for root's lockedwinner/exactrecording plan.
% No newactor forward pass. All requests replayed fromfrozennativehistories.
assert(~usejava('desktop'));maxNumCompThreads(2);
assert(~isfolder(outputFolder),'Preserve any previous final verification.');mkdir(outputFolder);
waitStart=tic;
while ~isfile(planFile)
 assert(toc(waitStart)<600,'No locked final recording plan received before timeout.');pause(1);
end
plan=jsondecode(fileread(planFile));assert(plan.winnerLocked&&numel(plan.replays)==2);
assert(strcmp(sha256(plan.checkpointFile),plan.checkpointSha256),'Finalcontrollercheckpoint changed.');
write_json(fullfile(outputFolder,'consumed_plan.json'),plan);copyfile(planFile,fullfile(outputFolder,'frozen_plan.json'));
timer=tic;reports=cell(1,3);
for k=1:2
 item=plan.replays(k);assert(strcmp(sha256(item.recordingMat),item.recordingMatSha256));
 reports{k}=replay_native(item.buildFolder,item.configsFile,item.recordingMat,fullfile(outputFolder,item.label));
 write_json(fullfile(outputFolder,'progress.json'),struct('completedParity',k,'totalWall_s',toc(timer)));
end
item=plan.replays(1);
reports{3}=refine_recorded_actions(item.buildFolder,item.configsFile,item.recordingMat,fullfile(outputFolder,'worst_recorded_actions_600k'));
report=struct('status','final_independent_verification_complete','checkpointSha256',plan.checkpointSha256, ...
 'planSha256',sha256(planFile),'reports',{reports},'wall_s',toc(timer), ...
 'scope','TwoindependentnativeMATLABreplaysofexactlockedpolicyrequests;onefine-steprecordedrequestrefinement. Nocanonicaledits,noretraining,andnofeedbackfromrefinedstatesintoneuralactor.');
write_json(fullfile(outputFolder,'summary.json'),report);copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'final_verification_batch.m'));
fprintf('RL_FINAL_INDEPENDENT_VERIFICATION_COMPLETE %s\n',jsonencode(report));
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
