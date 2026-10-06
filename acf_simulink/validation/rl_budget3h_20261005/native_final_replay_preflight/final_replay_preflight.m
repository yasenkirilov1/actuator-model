function report=final_replay_preflight(buildFolder,configsFile,outputFolder)
% Compile/clock audit only. No native episode or qualification outcome.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
addpath(fullfile(buildFolder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
Simulink.fileGenControl('set','CacheFolder',fullfile(outputFolder,'cache'),'CodeGenFolder',fullfile(outputFolder,'code'),'createDir',true);
modelFile=fullfile(outputFolder,'acf_rl_native.slx');copyfile(fullfile(buildFolder,'acf_rl_native.slx'),modelFile);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
bank=load(configsFile,'configs');p=acf_prepare(bank.configs(1).p);original=p;
assert(abs(p.sim.step-1/300000)<1e-15&&p.force.sampleTime==1e-4&&p.adcStudy.outputRate_Hz==30000);
originalClock=noise_clocks(mdl,p);p.sim.step=1/600000;p=acf_prepare(p);refinedClock=noise_clocks(mdl,p);
assert(isequaln(originalClock,refinedClock),'Noise timing/variance changed during ODE refinement.');
assert(p.adcStudy.fineClock_s==original.adcStudy.fineClock_s&&p.adcStudy.outputSampleTime_s==original.adcStudy.outputSampleTime_s);
assert(isequaln(p.adcStudy,original.adcStudy));
assert(p.mechanics.mA==.4&&p.mechanics.upperRigid,'Selected physical A must remain prescribed and0.4kg metadata.');
assignin(get_param(mdl,'ModelWorkspace'),'p',p);set_param(mdl,'FixedStep',num2str(p.sim.step,17));
set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
report=struct('status','refinement_preflight_passed_no_episodes','createdEpisodes',0,'configsSha256',sha256(configsFile), ...
 'sourceModelSha256',sha256(fullfile(buildFolder,'acf_rl_native.slx')),'baseStep_s',original.sim.step,'refinedStep_s',p.sim.step, ...
 'rawNoiseClock_s',p.adcStudy.fineClock_s,'rawAdcSampleTime_s',p.adcStudy.outputSampleTime_s,'controllerSampleTime_s',p.force.sampleTime, ...
 'noiseClocksAndVariancesPreserved',true,'adcPreparedFieldsPreserved',true,'noiseBlocks',originalClock,'algebraicLoopCount',loops, ...
 'massA_kg',p.mechanics.mA,'massB_kg',p.mechanics.mB,'massS_kg',p.mechanics.mS,'upperRigid',p.mechanics.upperRigid, ...
 'massAInterpretation','A follows prescribed actual robot motion because R–A rigid.0.4kg is knowncontext/mechanicalbookkeeping; it doesnot add freeA dynamicsor robotflexibility. B remains0.6kgassumed movingholder.');
write_json(fullfile(outputFolder,'preflight.json'),report);copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'final_replay_preflight.m'));
fprintf('RL_FINAL_REPLAY_PREFLIGHT %s\n',jsonencode(report));
end
function out=noise_clocks(mdl,p)
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
blocks=[find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','RandomNumber');find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'BlockType','UniformRandomNumber')];
out=struct([]);
for k=1:numel(blocks)
 block=blocks{k};expression=get_param(block,'SampleTime');time=evalin(get_param(mdl,'ModelWorkspace'),expression);
 r=struct('block',block,'sampleTimeExpression',expression,'sampleTime_s',time,'seedExpression',get_param(block,'Seed'));
 if strcmp(get_param(block,'BlockType'),'RandomNumber'),r.varianceExpression=get_param(block,'Variance');r.variance=evalin(get_param(mdl,'ModelWorkspace'),r.varianceExpression);
 else,r.varianceExpression='bounded_uniform_block_limits';r.variance=NaN;end
 if isempty(out),out=r;else,out(end+1)=r;end
end
end
function hash=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);cleanup=onCleanup(@()fclose(fid));digest=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
