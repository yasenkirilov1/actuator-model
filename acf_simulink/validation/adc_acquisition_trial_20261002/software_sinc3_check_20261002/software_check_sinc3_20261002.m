function report=software_check_sinc3_20261002(root)
% Native software proof, not a sharp-bump or hardware qualification.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root='/home/tommie/Projects/actuator-model';end
source=fullfile(root,'acf_simulink','validation','adc_acquisition_trial_20261002');
output=fullfile(source,'software_sinc3_check_20261002');assert(~isfolder(output));mkdir(output);
sourceHash=sha256(fullfile(source,'acf_force_control_adc_trial.slx'));
assert(strcmp(sourceHash,'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
copyfile(fullfile(source,'acf_force_control_adc_trial.slx'),fullfile(output,'acf_force_control_adc_trial.slx'));
copyfile([mfilename('fullpath') '.m'],fullfile(output,'software_check_sinc3_20261002.m'));
addpath(fullfile(source,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
private=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
mkdir(private);mkdir(fullfile(private,'cache'));mkdir(fullfile(private,'code'));
sdi=fullfile('/dev/shm',['acf-adc-sinc3-proof-' char(java.util.UUID.randomUUID)]);mkdir(sdi);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(true);
Simulink.fileGenControl('set','CacheFolder',fullfile(private,'cache'),'CodeGenFolder',fullfile(private,'code'),'createDir',true);
mdl='acf_force_control_adc_trial';cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,sdi)); %#ok<NASGU>
load_system(fullfile(output,[mdl '.slx']));saved=load(fullfile(source,'adc_trial_candidate.mat'));p=saved.p;
p.sim.stopTime=.1;p.sim.evaluateFrom=0;p.sim.step=1/300000;
p.mechanics.mS=2;p.mechanics.angle=75*pi/180;p.force.reference=50;
p.sensor.noiseRms=0;p.contact.lateralNoiseRms_N=0;p.sensor.strokeNoiseRms=0;p.accel.noiseRms=0;
p.adcStudy.converter='sinc3';p.adcStudy.converterAnalogOrder=0;
p.adcStudy.fineClock_s=1/300000;p.adcStudy.outputRate_Hz=30000;p.adcStudy.modulatorRate_Hz=3840000;
p.adcStudy.forceQuantum_N=320/2^24;p.adcStudy.accelerationQuantum_m_s2=2.4/2^24/.02*9.80665;
p.adcStudy.forceAdcNoiseRms_N=0;p.adcStudy.accelerationAdcNoiseRms_m_s2=0;
p.adcStudy.accelerationAdcRange_g=60;p=acf_prepare(p);
assignin(get_param(mdl,'ModelWorkspace'),'p',p);set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));
elapsed=tic;out=sim(mdl);elapsed=toc(elapsed);
normal=out.get('normal_contact_log');values=reshape(normal.Data,numel(normal.Time),8);
adc=out.get('adc_acquisition_log');adcValues=reshape(adc.Data,numel(adc.Time),8);
acc=out.get('accelerometer_log');accValues=reshape(acc.Data,numel(acc.Time),6);
balance=accValues(:,5)-(values(:,4).*sqrt(1+values(:,5).^2)+values(:,5).*accValues(:,6));
report=struct('status','complete_native_software_check_not_force_qualification', ...
 'modelSha256',sourceHash,'elapsed_s',elapsed,'solverStep_s',p.sim.step, ...
 'converter','ADS131M08-clock-scaled-SINC3','outputRate_Hz',p.adcStudy.outputRate_Hz, ...
 'modulatorRate_Hz',p.adcStudy.modulatorRate_Hz,'oversamplingRatio',p.adcStudy.d.converter.oversamplingRatio, ...
 'nativeForceSamples',numel(normal.Time),'nativeAdcConversionFrames',numel(adc.Time), ...
 'maximumNormalForceDeviation_N',max(abs(values(:,1)-p.force.reference)), ...
 'maximumReconstructionBalanceResidual_N',max(abs(balance)), ...
 'maximumAdcClippingMagnitude',max(max(abs(adcValues(:,5:8)))), ...
 'converterApproximation',p.adcStudy.d.converter.responseApproximation, ...
 'checks',struct('exactOSR128',p.adcStudy.d.converter.oversamplingRatio==128, ...
 'exact30001NativeForceSamples',numel(normal.Time)==30001,'exact3001AdcFrames',numel(adc.Time)==3001, ...
 'actual30000HzOutput',max(abs(diff(adc.Time)-1/30000))<1e-12, ...
 'finiteSignals',all(isfinite(values(:)))&&all(isfinite(adcValues(:))), ...
 'noAdcClipping',all(all(adcValues(:,5:8)==0)), ...
 'reconstructionBalance',max(abs(balance))<1e-10, ...
 'constantLoadedForcePreserved',max(abs(values(:,1)-p.force.reference))<.001), ...
 'scope','Quiet loaded-equilibrium native compile/clock/quantized-tare proof. Noise disabled; no bump and no control retuning. Does not qualify the ADC design or physical hardware.');
assert(all(structfun(@(x)logical(x),report.checks)));
save(fullfile(output,'native_output.mat'),'out','p','-v7.3');
write_json(fullfile(output,'results.json'),report);
assert(strcmp(sha256(fullfile(source,'acf_force_control_adc_trial.slx')),sourceHash));
end

function finish(mdl,storage,record,sdi)
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
try,if bdIsLoaded(mdl),close_system(mdl,0);end,catch,end
if isfolder(sdi),try,rmdir(sdi,'s');catch,end,end
end

function write_json(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));
end

function hash=sha256(path)
fid=fopen(path,'rb');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
digest=java.security.MessageDigest.getInstance('SHA-256');
while ~feof(fid),bytes=fread(fid,1024*1024,'*uint8');digest.update(typecast(bytes,'int8'));end
hash=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
end
