function manifest=prepare_budget_bank(sourceFolder,folder,currentResponse_s,extraKnownLatency_s)
% Apply retainedADXL354/M08 envelope assumptions to frozen episode identities.
% No simulations, policy training, fresh outcomes or independent physics.
assert(~usejava('desktop'));maxNumCompThreads(2);
if nargin<3,currentResponse_s=50e-6;end
if nargin<4,extraKnownLatency_s=0;end
assert(any(abs(currentResponse_s-[50e-6 100e-6])<1e-15)&&extraKnownLatency_s>=0);
assert(~isfolder(folder),'Preserve all source and budget banks.');mkdir(folder);
original=load(fullfile(sourceFolder,'configs.mat'),'configs');configs=original.configs;prior=jsondecode(fileread(fullfile(sourceFolder,'manifest.json')));metadata=prior.cases;
copyfile(fullfile(sourceFolder,'helpers'),fullfile(folder,'helpers'));addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8;
g=9.80665;referenceENBW=.06354253123515036/(2*10e-6);density=33.75;sensitivity=.1;
noiseReference=density*1e-6*g*sqrt(referenceENBW);
accQ=2.4/2^24/sensitivity*g;accTotal=21.31e-6/sensitivity*g;accAdcNoise=sqrt(max(0,accTotal^2-accQ^2/12));
changes=struct([]);
for k=1:numel(configs)
 before=configs(k).p;p=before;
 assert(p.adcStudy.outputRate_Hz==30000&&p.adcStudy.fineClock_s==1/300000);
 assert(abs(p.coil.forceConstant-24.4)<1e-12&&abs(p.coil.resistance-8.896)<1e-12&&p.coil.voltage==32&&p.coil.inductance==.004);
 p.accel.range_g=8;p.accel.noiseRms=noiseReference;p.accel.nominalNoiseRms=noiseReference;
 p.coil.currentResponse=currentResponse_s;
 p.adcStudy.analogCutoff_Hz=1900;p.adcStudy.identifiedAnalogCutoff_Hz=1900;
 p.adcStudy.accelerationQuantum_m_s2=accQ;p.adcStudy.accelerationAdcNoiseRms_m_s2=accAdcNoise;p.adcStudy.accelerationAdcRange_g=1.2/sensitivity;
 p.adcStudy.pipelineDelay_s=before.adcStudy.pipelineDelay_s+extraKnownLatency_s;
 p.adcStudy.identifiedPipelineDelay_s=before.adcStudy.identifiedPipelineDelay_s+extraKnownLatency_s;
 p.budgetAccelerometer=struct('profile','ADXL354C_1900_M08_30k_hot32V_RLbudget', ...
 'density_ug_sqrtHz',density,'sensorRange_g',8,'analogSensitivity_V_g',sensitivity, ...
 'matchedNominalCompleteResponseCutoff_Hz',1900,'commonExtraLatency_s',extraKnownLatency_s, ...
 'actualManufacturerCompleteTransferRepresented',false,'partQualified',false, ...
 'scope','Partanchorednoise/range/scaling;assumedengineeredcomplete1900Hzmatchedenvelope,notexactmanufacturerphase/transfer. OriginalactualvsidentifiedAA/timing/staticcalibrationresidualspreserved. Addedlatencyifrequested isknownmatchedcommonlatency,notunknownstress. Nointernalsensordelayissubtracted.');
 p=acf_prepare(p);assert(isequaln(p.mechanics,before.mechanics)&&isequaln(p.uncertainty,before.uncertainty));
 expectedAcquisition=before.acquisition;
 expectedAcquisition.pipelineDelay_s=before.adcStudy.pipelineDelay_s+extraKnownLatency_s;
 expectedAcquisition.identifiedPipelineDelay_s=before.adcStudy.identifiedPipelineDelay_s+extraKnownLatency_s;
 assert(isequaln(p.acquisition,expectedAcquisition));
 assert(p.sensor.seed==before.sensor.seed&&p.force.reference==before.force.reference&&isequaln(p.robot,before.robot));
 assert(p.accel.bias==before.accel.bias&&p.accel.scaleError==before.accel.scaleError&&p.sensor.distalMass_model_kg==before.sensor.distalMass_model_kg);
 assert(abs(p.adcStudy.d.accelerationNoiseDensity_m_s2_sqrtHz-density*1e-6*g)<1e-14);
 configs(k).p=p;metadata(k).usableVoltage_V=p.coil.voltage;metadata(k).currentResponse_s=p.coil.currentResponse;
 metadata(k).identifiedCurrentResponse_s=p.coil.currentResponse;
 metadata(k).budgetAccelDensity_ug_sqrtHz=density;metadata(k).budgetAccelRange_g=8;
 metadata(k).budgetMatchedResponse_Hz=1900;metadata(k).budgetExtraKnownLatency_s=extraKnownLatency_s;
 metadata(k).sensorProfile='ADXL354C-like_matched_response_envelope';
 r=struct('scenario',k-1,'identity',configs(k).identity,'split',configs(k).split, ...
 'baseAccelerationNoiseReferenceRms_m_s2',before.accel.noiseRms,'budgetAccelerationNoiseReferenceRms_m_s2',p.accel.noiseRms, ...
 'baseAdcAccelerationQuantum_m_s2',before.adcStudy.accelerationQuantum_m_s2,'budgetAdcAccelerationQuantum_m_s2',accQ, ...
 'baseAdcAccelerationNoise_m_s2',before.adcStudy.accelerationAdcNoiseRms_m_s2,'budgetAdcAccelerationNoise_m_s2',accAdcNoise, ...
 'actualCommonPipeline_s',p.adcStudy.pipelineDelay_s,'identifiedCommonPipeline_s',p.adcStudy.identifiedPipelineDelay_s);
 if isempty(changes),changes=r;else,changes(end+1)=r;end
end
assert(isequal({configs.identity},{original.configs.identity})&&isequal({configs.split},{original.configs.split}));
save(fullfile(folder,'configs.mat'),'configs','-v7');manifest=prior;manifest.status='budget_envelope_prepared_frozen_not_executed';manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(sourceFolder,'configs.mat'));manifest.parentManifestSha256=sha256(fullfile(sourceFolder,'manifest.json'));
manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));manifest.scenarioIdentitiesAndSplitsPreserved=true;manifest.budgetTransform=changes;
manifest.budgetContract=struct('referenceRecipe','acf_simulink/experiments/prepare_budget_sensor_stress_20261002.m', ...
 'sensor','ADXL354C-like±8g','density_ug_sqrtHz',density,'sensitivity_V_g',sensitivity, ...
 'fixedReferenceENBW_Hz',referenceENBW,'referenceConditionedAccelerationNoiseRms_m_s2',noiseReference, ...
 'accelerationQuantum_m_s2',accQ,'accelerationAdcNoiseRms_m_s2',accAdcNoise,'sensorRange_g',8,'adcRange_g',1.2/sensitivity, ...
 'nominalActualAndIdentifiedCompleteResponseCutoff_Hz',1900,'extraKnownMatchedCommonLatency_s',extraKnownLatency_s, ...
 'currentResponse_s',currentResponse_s,'coilKf_N_A',24.4,'coilR_ohm',8.896,'coilL_H',.004,'usableVoltage_V',32, ...
 'rawNoiseClock_Hz',300000,'adcRate_Hz',30000,'controllerRate_Hz',10000, ...
 'qualification','Parameter/sourcepreparationonly;noepisodesorqualificationexecuted. Exactoldresponse/noise/range/quantizationmodelcontract,includingnativeactualcalibration/timingresiduals. CompleteADXL354internalmechanical/filterphaseisnotrepresentedexactly andassembledchainmustmeetthe1900Hzmatchedresponseenvelope. No waveformfuture/previewinputs.');
write_json(fullfile(folder,'manifest.json'),manifest);copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_budget_bank.m'));
fprintf('RL_BUDGET_SENSOR_BANK_FROZEN %d cases,current %.1fus,knownextra%.1fus\n',numel(configs),currentResponse_s*1e6,extraKnownLatency_s*1e6);
end
function s=sha256(file)
fid=fopen(file,'r');assert(fid>=0);bytes=fread(fid,Inf,'*uint8');fclose(fid);digest=java.security.MessageDigest.getInstance('SHA-256');digest.update(bytes);
s=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2).',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
