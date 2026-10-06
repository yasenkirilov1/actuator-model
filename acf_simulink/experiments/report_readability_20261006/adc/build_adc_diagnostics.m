function summary=build_adc_diagnostics(root,storageRoot)
% Matched fixed-mechanics ADC diagnostics. No controller/modal retuning.
% Private workspace and code exports only; frozen source files are untouched.
assert(~usejava('desktop'));maxNumCompThreads(2);
assert(~isfolder(storageRoot),'Preserve previous ADC evidence.');mkdir(storageRoot);
study=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005','qualification_v8');
lockFile=fullfile(study,'candidate_lock.json');lock=jsondecode(fileread(lockFile));
source=lock.sourceNativeDirectory;sourceFile=fullfile(source,'acf_rl_native.slx');
meta=jsondecode(fileread(fullfile(source,'metadata.json')));
configsFile=meta.configsFile;sourceHash=sha256(sourceFile);configsHash=sha256(configsFile);
assert(strcmp(configsHash,meta.configsSha256));
bank=load(configsFile,'configs');selected=[165,166,168,169,170,171,172];
assert(numel(bank.configs)==264);
copyfile(fullfile(source,'helpers'),fullfile(storageRoot,'helpers'));
copyfile(fullfile(root,'acf_simulink','experiments','upper_no_damping_20261005','qualification_v8','export_native_parameter_bank_v8.m'),fullfile(storageRoot,'export_native_parameter_bank_v8.m'));
addpath(storageRoot,'-begin');addpath(fullfile(storageRoot,'helpers'),'-begin');
clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal;
storage=Simulink.sdi.getStorageLocation;record=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fclose(fopen(fullfile(sdi,'owned_adc_20261006'),'w'));
cleanup=onCleanup(@()finish(storage,record,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(false);
groups={'sinc3_30000','sinc3_15000','sinc3_7500'};rates=[30000,15000,7500];
noiseVolts=[21.31,10.68,7.56]*1e-6;exportRows=struct([]);
for g=1:3
 folder=fullfile(storageRoot,groups{g});mkdir(folder);mkdir(fullfile(folder,'bank'));
 if g==1,bits=[24,16,14,12];else,bits=24;end
 configs=struct([]);rows=struct([]);
 for bit=bits
  for k=1:numel(selected)
   srcIndex=selected(k);before=acf_prepare(bank.configs(srcIndex+1).p);p=before;
   assert(p.mechanics.mA==1.5&&p.mechanics.mB==.6&&p.mechanics.k1==64e6&&p.mechanics.c1==0&&p.mechanics.k2==20000&&p.mechanics.c2==0);
   assert(~p.mechanics.upperRigid&&~p.mechanics.lowerRigid&&p.coil.currentResponse==50e-6&&p.coil.voltage==32);
   assert(p.inertial.forceKp==lock.executingNormalForceControllerGains.forceKp_A_N);
   assert(p.inertial.forceKi==lock.executingNormalForceControllerGains.forceKi_A_N_s);
   p.adcStudy.outputRate_Hz=rates(g);
   p.adcStudy.forceQuantum_N=320/2^bit;
   p.adcStudy.accelerationQuantum_m_s2=2.4/2^bit/.1*9.80665;
   if g>1
    % OSR256/512 manufacturer values at gain1, with the 7.68MHz clock.
    % No claimed noise improvement from reducing the clock 8.192->7.68MHz.
    p.adcStudy.forceAdcNoiseRms_N=noiseVolts(g)/(1.2/160);
    p.adcStudy.accelerationAdcNoiseRms_m_s2=noiseVolts(g)/(.1/9.80665);
   end
   expectedAdc=p.adcStudy;p=acf_prepare(p);
   originalFields=fieldnames(before);assert(isequal(originalFields,fieldnames(p)));
   for n=1:numel(originalFields)
    name=originalFields{n};if any(strcmp(name,{'d','adcStudy','accel'})),continue;end
    assert(isequaln(p.(name),before.(name)),['Unrequested input changed: ' name]);
   end
   a=p.accel;b=before.accel;a.quantum=b.quantum;assert(isequaln(a,b));
   assert(isequaln(p.d.cellModal,before.d.cellModal));
   assert(isequaln(p.d.upperMotionObserver,before.d.upperMotionObserver));
   assert(p.adcStudy.fineClock_s==before.adcStudy.fineClock_s&&p.sim.step==before.sim.step&&p.force.sampleTime==before.force.sampleTime);
   adcNames=fieldnames(expectedAdc);
   for n=1:numel(adcNames)
    name=adcNames{n};if any(strcmp(name,{'d','outputSampleTime_s'})),continue;end
    assert(isequaln(p.adcStudy.(name),expectedAdc.(name)),['Unexpected ADC input change: ' name]);
   end
   cfg=bank.configs(srcIndex+1);cfg.p=p;
   cfg.identity=sprintf('adc_%s_q%d_source%d',groups{g},bit,srcIndex);
   if isempty(configs),configs=cfg;else,configs(end+1)=cfg;end %#ok<AGROW>
   q=struct('scenario',numel(configs)-1,'identity',cfg.identity,'sourceScenario',srcIndex, ...
    'sourceIdentity',bank.configs(srcIndex+1).identity,'adcGroup',groups{g},'quantizationBits',bit, ...
    'diagnosticType',ternary(bit==24,'converter_rate_noise_and_delay','quantization_only'), ...
    'massA_kg',p.mechanics.mA,'massB_kg',p.mechanics.mB,'massS_kg',p.mechanics.mS, ...
    'force_N',p.force.reference,'angle_deg',p.mechanics.angle*180/pi,'duration_s',p.sim.stopTime, ...
    'midTravel_m',p.mechanics.midTravel,'seed',p.sensor.seed,'gainID',8, ...
    'peakCurrent_A',p.coil.peakCurrent,'usableVoltage_V',p.coil.voltage, ...
    'bumpStart_s',p.surface.startTime,'recenterBandRadius_m',.0075, ...
    'outputRate_Hz',p.adcStudy.outputRate_Hz,'forceCodeStep_N',p.adcStudy.forceQuantum_N, ...
    'accelerationCodeStep_m_s2',p.adcStudy.accelerationQuantum_m_s2, ...
    'forceAdcNoiseRms_N',p.adcStudy.forceAdcNoiseRms_N, ...
    'accelerationAdcNoiseRms_m_s2',p.adcStudy.accelerationAdcNoiseRms_m_s2, ...
    'sinc3GroupDelay_s',p.adcStudy.d.converter.exactGroupDelay_s, ...
    'emaGroupDelay_s',p.adcStudy.d.channels.force.emaDCGroupDelay_s, ...
    'analogGroupDelay_s',p.adcStudy.d.channels.force.analogDCGroupDelay_s, ...
    'pipelineDelay_s',p.adcStudy.d.channels.force.pipelineDelay_s, ...
    'physicalNoiseClock_s',p.adcStudy.fineClock_s,'physicalStep_s',p.sim.step, ...
    'controlStep_s',p.force.sampleTime,'controllerRetuned',false, ...
    'modalCoefficientsAndIdentificationExactlyUnchanged',true, ...
    'bodyObserverCoefficientsExactlyUnchanged',true);
   if isempty(rows),rows=q;else,rows(end+1)=q;end %#ok<AGROW>
  end
 end
 newConfigsFile=fullfile(folder,'bank','configs.mat');save(newConfigsFile,'configs','-v7');
 modelFile=fullfile(folder,'acf_rl_native.slx');copyfile(sourceFile,modelFile);
 mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(modelFile);
 Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'),'CodeGenFolder',fullfile(folder,'code'),'createDir',true);
 p=acf_prepare(configs(1).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 set_param(mdl,'SystemTargetFile','grt.tlc','TargetLang','C','DefaultParameterBehavior','Tunable', ...
  'CodeInterfacePackaging','Nonreusable function','GRTInterface','off','MatFileLogging','off', ...
  'GenerateReport','off','GenCodeOnly','on','SignalLogging','off','SaveTime','on','SaveOutput','on', ...
  'SaveFormat','Dataset','FixedStep',num2str(p.sim.step,17),'Solver','ode4','SimulationMode','normal', ...
  'StopTime','inf','LoadExternalInput','off');
 save_system(mdl,modelFile);timer=tic;slbuild(mdl);elapsed=toc(timer);
 export_native_parameter_bank_v8(mdl,configs,folder);
 mapped=jsondecode(fileread(fullfile(folder,'scenarios.json')));
 for k=1:numel(rows)
  names=fieldnames(mapped);for n=1:numel(names),rows(k).(names{n})=mapped(k).(names{n});end
 end
 write_json(fullfile(folder,'scenarios.json'),rows);
 privateMeta=meta;privateMeta.configsFile=newConfigsFile;privateMeta.configsSha256=sha256(newConfigsFile);
 privateMeta.status='private_adc_diagnostic_export_not_executed';privateMeta.modelFile=modelFile;
 privateMeta.privateModelSha256=sha256(modelFile);privateMeta.sourceNativeDirectory=source;
 privateMeta.sourceNativeModelSha256=sourceHash;privateMeta.sourceConfigsSha256=configsHash;
 privateMeta.adcStudyScope='7 locked development corners; no candidate selection or qualification; no PI/modal retuning; private causal acquisition frame follows specified ADC.';
 privateMeta.executingControllerGains=lock.executingNormalForceControllerGains;
 privateMeta.algebraicLoopCount=loops;privateMeta.nativeExportElapsed_s=elapsed;
 write_json(fullfile(folder,'metadata.json'),privateMeta);
 exp=struct('group',groups{g},'casesPrepared',numel(configs),'rate_Hz',rates(g), ...
  'bits',bits,'folder',folder,'modelSha256',sha256(modelFile),'configsSha256',sha256(newConfigsFile), ...
  'algebraicLoopCount',loops,'nativeExportElapsed_s',elapsed);
 if isempty(exportRows),exportRows=exp;else,exportRows(end+1)=exp;end %#ok<AGROW>
 close_system(mdl,0);clear configs;Simulink.sdi.clear;
end
assert(strcmp(sourceHash,sha256(sourceFile))&&strcmp(configsHash,sha256(configsFile)));
summary=struct('status','private_adc_diagnostic_banks_prepared_not_executed','date','2026-10-06', ...
 'sourceModel',sourceFile,'sourceModelSha256',sourceHash,'sourceConfigs',configsFile,'sourceConfigsSha256',configsHash, ...
 'candidateLockFile',lockFile,'candidateLockSha256',sha256(lockFile),'selectedSourceScenarios',selected, ...
 'plannedDiagnosticRecordings',42,'qualificationRecordings',0,'controllerRetuned',false,'exports',exportRows, ...
 'sourceInputsPreserved',true,'recipeSha256',sha256([mfilename('fullpath') '.m']));
write_json(fullfile(storageRoot,'preparation_manifest.json'),summary);
fprintf('ADC_DIAGNOSTICS_PREPARED %s\n',jsonencode(summary));
end
function x=ternary(flag,a,b),if flag,x=a;else,x=b;end,end
function finish(storage,record,sdi)
try,close_system('acf_rl_native',0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_adc_20261006')),try,rmdir(sdi,'s');catch,end,end
end
function h=sha256(path)
fid=fopen(path,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(path,value)
fid=fopen(path,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
