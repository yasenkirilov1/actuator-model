function report=check_recipe_preparation_v8(outputFolder,sourceBankFile,sourceScenario)
% Read-only-source arithmetic/parser fixture; no plant or qualification bank.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(outputFolder));mkdir(outputFolder);
recipe=fileparts(mfilename('fullpath'));loaded=load(sourceBankFile,'configs');
original=loaded.configs(sourceScenario+1).p;
assert(strcmp(loaded.configs(sourceScenario+1).split,'development')&&original.mechanics.mA==1.5);
copyfile(fullfile(fileparts(sourceBankFile),'helpers'),fullfile(outputFolder,'helpers'));
copyfile(fullfile(recipe,'acf_prepare_cell_modal.m'),fullfile(outputFolder,'helpers','acf_prepare_cell_modal.m'));
addpath(fullfile(outputFolder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer acf_prepare_cell_modal acf_prepare_accel_lead;
modal=original.d.cellModal;
id=struct('identifiedMassB_kg',.6,'identifiedStiffness_N_m',64e6,'identifiedForceConstant_N_A',24.4, ...
 'identifiedCurrentResponse_s',50e-6,'acquiredTransferReal',original.cellModal.acquiredTransferReal, ...
 'acquiredTransferImag',original.cellModal.acquiredTransferImag,'frozenPreparedCoefficients',modal);
p=original;p.qualificationModalIdentification=id;p.cellModal.identifiedCurrentResponse_s=50e-6;p=acf_prepare(p);
assert(isequaln(p.d,original.d),'Preparation adapter changes nominal prepared coefficients.');
checks=struct([]);
for j=1:3
 test=p;
 if j==1,test.coil.currentResponse=100e-6;label='actual_current100us';
 elseif j==2,test.adcStudy.analogCutoff_Hz=.95*p.adcStudy.analogCutoff_Hz;label='actual_response_minus5pct';
 else,test.uncertainty.massB_model_kg=.6015;test.inertial.lowerLoadInverse.identifiedMassB_kg=.6015;test.uncertainty.forceConstant_model_N_A=24.461;test.inertial.lowerLoadInverse.identifiedForceConstant_N_A=24.461;label='global_identified_holder_and_Kf_residual';end
 test=acf_prepare(test);exact=isequaln(test.d.cellModal,modal);assert(exact);
 row=struct('label',label,'fullModalCoefficientsExact',exact);
 if isempty(checks),checks=row;else,checks(end+1)=row;end
end
files=dir(fullfile(recipe,'*.m'));parser=struct([]);
for j=1:numel(files)
 path=fullfile(recipe,files(j).name);issues=checkcode(path,'-id');
 row=struct('file',files(j).name,'issues',issues);
 if isempty(parser),parser=row;else,parser(end+1)=row;end
end
report=struct('status','preparation_arithmetic_and_parser_fixture_checked_no_physics', ...
 'sourceBankFile',sourceBankFile,'sourceScenario',sourceScenario,'sourceIdentity',loaded.configs(sourceScenario+1).identity, ...
 'sourceConfigsSha256',sha256(sourceBankFile),'adapterSha256',sha256(fullfile(recipe,'acf_prepare_cell_modal.m')), ...
 'allNominalPreparedCoefficientsExact',true,'interventions',checks,'parserReports',parser, ...
 'qualificationEpisodes',0,'nativeParameterCasesPreparedForQualification',0, ...
 'executingNormalForceKp_A_N',p.inertial.forceKp,'executingNormalForceKi_A_N_s',p.inertial.forceKi, ...
 'scope','One already-developed controller context checks coefficient arithmetic only. It neither selects a candidate nor executes a physical trajectory. Actual parameter perturbations affect physical preparation but leave modal identification fixed.');
fid=fopen(fullfile(outputFolder,'preparation_fixture.json'),'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('QUALIFICATION_RECIPE_FIXTURE %s\n',jsonencode(report));
end

function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
