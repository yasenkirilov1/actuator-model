function report=prepare_budget_adc_hot_coil_20261002(root,outputFolder)
%PREPARE_BUDGET_ADC_HOT_COIL_20261002 New electrical inputs, no simulations.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<1,root=fileparts(fileparts(fileparts(mfilename('fullpath'))));end
if nargin<2,outputFolder=fullfile(root,'acf_simulink','validation','budget_adc_hot_coil_20261002_inputs');end
assert(~isfile(fullfile(outputFolder,'manifest.json')),'Preserve frozen inputs.');
if ~isfolder(outputFolder),mkdir(outputFolder);end
validation=fullfile(root,'acf_simulink','validation');native=fullfile(validation,'adc_acquisition_trial_20261002');
modelFile=fullfile(native,'acf_force_control_adc_trial.slx');modelSha=sha256(modelFile);
assert(strcmp(modelSha,'b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5'));
sourceFile=fullfile(validation,'budget_sensor_stress_20261002_inputs', ...
 'ADXL1002_reference_8000_L0_current100_24V_configs.mat');sourceSha=sha256(sourceFile);
assert(strcmp(sourceSha,'23339a9fcefb2829aaf8252cf97b9de3aa8e1c3907d04b8128dabf9d361d7eb7'));
loaded=load(sourceFile,'configs');configs=loaded.configs;assert(numel(configs)==7);
addpath(fullfile(native,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
name='ADXL1002_8000_M08_30k_current100_hot32V';rows=struct([]);
for k=1:7
 before=configs(k).p;p=before;changes=struct([]);
 assert(abs(before.coil.currentResponse-100e-6)<1e-15);
 assert(before.adcStudy.analogCutoff_Hz==8000&&before.adcStudy.outputRate_Hz==30000);
 assert(strcmp(before.adcStudy.converter,'sinc3'));
 assert(before.coil.forceConstant==20);
 oldRatio=before.uncertainty.forceConstant_model_N_A/before.coil.forceConstant;
 fields={'forceConstant','resistance','inductance','voltage'};values=[24.4,8.896,.004,32];
 for j=1:4
  field=fields{j};p.coil.(field)=values(j);
  changes=record(changes,['p.coil.' field],before.coil.(field),p.coil.(field));
 end
 p.uncertainty.forceConstant_model_N_A=24.4*oldRatio;
 changes=record(changes,'p.uncertainty.forceConstant_model_N_A', ...
  before.uncertainty.forceConstant_model_N_A,p.uncertainty.forceConstant_model_N_A);
 p.inertial.lowerLoadInverse.identifiedForceConstant_N_A=p.uncertainty.forceConstant_model_N_A;
 changes=record(changes,'p.inertial.lowerLoadInverse.identifiedForceConstant_N_A', ...
  before.inertial.lowerLoadInverse.identifiedForceConstant_N_A,p.inertial.lowerLoadInverse.identifiedForceConstant_N_A);
 for field={'forceKp','forceKi','forceKd'}
  p.inertial.(field{1})=before.inertial.(field{1})*20/24.4;
  changes=record(changes,['p.inertial.' field{1}],before.inertial.(field{1}),p.inertial.(field{1}));
 end
 expected=p;p=acf_prepare(p);
 assert(abs(p.uncertainty.forceConstant_model_N_A/p.coil.forceConstant-oldRatio)<1e-14);
 assert(p.coil.currentResponse==before.coil.currentResponse);
 assert(isequaln(p.coil,expected.coil)&&isequaln(p.uncertainty,expected.uncertainty));
 % The lower inverse numerator is derived from the changed identified Kf.
 oldNum=before.inertial.lowerLoadInverse.numerator;newNum=p.inertial.lowerLoadInverse.numerator;
 ratio=before.inertial.lowerLoadInverse.identifiedForceConstant_N_A/p.inertial.lowerLoadInverse.identifiedForceConstant_N_A;
 assert(all(abs(newNum-oldNum*ratio)<1e-10*max(1,max(abs(newNum)))));
 assert(isequaln(p.inertial.lowerLoadInverse.denominator,before.inertial.lowerLoadInverse.denominator));
 expected.inertial.lowerLoadInverse.numerator=newNum;
 assert(isequaln(p.inertial,expected.inertial),'An unrelated inertial parameter changed.');
 originalFields=fieldnames(before);assert(isequal(sort(originalFields),sort(fieldnames(p))));
 for j=1:numel(originalFields)
  field=originalFields{j};
  if ~any(strcmp(field,{'coil','uncertainty','inertial','d'}))
   assert(isequaln(p.(field),before.(field)),['Unrequested parameter changed under p.' field]);
  end
 end
 scope=['New native hot-winding dynamics with selected ADS131M08 30 kSPS, assumed matched 8 kHz ' ...
  'ADXL1002 complete-response envelope and unchanged 100 us current target. Kf24.4 N/A, ' ...
  'R8.896 ohm (6.4 ohm at25C times1.39), L4 mH,32V usable bipolar output. ' ...
  'Identified Kf residual ratio retained; current-based PI/PID scaled20/24.4. ' ...
  'All acquisition, calibration, robot, mechanics, noise, range, travel and current limits unchanged. ' ...
  'No time-varying thermal state, hot-force supplier guarantee, full manufacturer sensor transfer ' ...
  'or assembled driver phase qualification. Prepared inputs require actual native execution.'];
 configs(k).p=p;configs(k).scenario='selected_budget_adc_hot_winding_stress';configs(k).group=name;
 configs(k).retainedParameterSource=sourceFile;configs(k).retainedParameterSha256=sourceSha;configs(k).retainedParameterIndex=k;
 row=struct('caseIndex',k,'sourceFile',sourceFile,'sourceSha256',sourceSha,'seed',p.sensor.seed, ...
  'duration_s',p.sim.stopTime,'massS_kg',p.mechanics.mS,'reference_N',p.force.reference, ...
  'changes',changes,'identifiedToActualKfRatioBefore',oldRatio, ...
  'identifiedToActualKfRatioAfter',p.uncertainty.forceConstant_model_N_A/p.coil.forceConstant, ...
  'currentResponse_s',p.coil.currentResponse,'acquisitionAllParametersUnchanged',true, ...
  'lowerInverseNumeratorBefore',oldNum,'lowerInverseNumeratorAfter',newNum, ...
  'lowerInverseNumeratorGainRatio',ratio,'scope',scope,'executed',false);
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
end
file=fullfile(outputFolder,[name '_configs.mat']);save(file,'configs','-v7');
copyfile([mfilename('fullpath') '.m'],fullfile(outputFolder,'prepare_budget_adc_hot_coil_20261002.m'));
report=struct('date','2026-10-02','status','prepared_inputs_not_executed','sourceModelFile',modelFile, ...
 'sourceModelSha256',modelSha,'sourceParametersFile',sourceFile,'sourceParametersSha256',sourceSha, ...
 'recipeSha256',sha256(fullfile(outputFolder,'prepare_budget_adc_hot_coil_20261002.m')), ...
 'file',file,'sha256',sha256(file),'plannedCases',7,'longCases',4,'mixedCases',3, ...
 'totalRecordingDuration_s',132,'executedNativeCases',0,'rows',rows,'scope',scope, ...
 'guardSummary',struct('onlyRequestedElectricalAndKfGainInputsChanged',true, ...
 'currentResponseUnchanged',true,'identifiedToActualKfRatioPreserved',true, ...
 'allAcquisitionInputsAndTimingPreserved',true,'staticCalibrationRobotMechanicsAndLimitsPreserved',true, ...
 'nativeSourceUnchanged',strcmp(sha256(modelFile),modelSha)));
assert(all(structfun(@(x)logical(x),report.guardSummary)));write_json(fullfile(outputFolder,'manifest.json'),report);
end

function changes=record(changes,field,original,updated)
r=struct('field',field,'original',original,'updated',updated,'delta',updated-original);
if isempty(changes),changes=r;else,changes(end+1)=r;end
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
