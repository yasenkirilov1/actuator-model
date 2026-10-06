function metadata=export_force_guide_native(folder,configsFile)
% Reuse accepted emitted computation, override only two mapped tunable Gains.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);mkdir(fullfile(folder,'code'));
root=fileparts(fileparts(fileparts(fileparts(fileparts(mfilename('fullpath'))))));
source=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005','native_observer_noise_v4');
originalMetadata=jsondecode(fileread(fullfile(source,'metadata.json')));
metadata=originalMetadata;metadata.status='reusing_accepted_emitted_native_graph';
metadata.parentNativeBuild=source;metadata.parentConfigsSha256=originalMetadata.configsSha256;
metadata.codegenSourceControllerGraphSha256=originalMetadata.privateModelSha256;
metadata.modelFile=fullfile(folder,'acf_rl_native.slx');metadata.configsFile=configsFile;metadata.configsSha256=sha256(configsFile);
copyfile(fullfile(source,'acf_rl_native.slx'),metadata.modelFile);
copyfile(fullfile(fileparts(configsFile),'helpers'),fullfile(folder,'helpers'));
copyfile(fullfile(source,'code','acf_rl_native_grt_rtw'),fullfile(folder,'code','acf_rl_native_grt_rtw'));
copyfile(fullfile(source,'generated_cell_source_audit.json'),fullfile(folder,'parent_generated_cell_source_audit.json'));
copyfile(fullfile(source,'local_cell_source_guard.json'),fullfile(folder,'parent_local_cell_source_guard.json'));
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'),'CodeGenFolder',fullfile(folder,'private_unused_codegen'),'createDir',true);
oldStorage=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
sdi=tempname('/dev/shm');mkdir(sdi);fid=fopen(fullfile(sdi,'owned_force_guide_v7_sdi'),'w');fprintf(fid,'Owned parameter-only export.\n');fclose(fid);
mdl='acf_rl_native';assert(~bdIsLoaded(mdl));load_system(metadata.modelFile);
cleanup=onCleanup(@()finish(mdl,oldStorage,oldRecord,sdi)); %#ok<NASGU>
Simulink.sdi.setStorageLocation(sdi);Simulink.sdi.setRecordData(false);
loaded=load(configsFile,'configs');configs=loaded.configs;p=acf_prepare(configs(1).p);
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
ff=[mdl '/Force controller/Coil PI and PID/Measured motion feedforward'];
set_param([ff '/Guide friction velocity scale'],'Gain','1/p.guideFeedforwardStudy.smoothingSpeed_m_s');
set_param([ff '/Guide Coulomb friction current'],'Gain','p.inertial.guideFrictionFeedforwardScale*p.guideFeedforwardStudy.coulombScale*p.coil.coulombForce/p.coil.forceConstant');
metadata.liveLocalCellSourceGuard=verify_local_cell_source(mdl,p,folder);
indices=[1,2,13,134,145];audits=struct([]);
for index=indices
 p=acf_prepare(configs(index).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 assert(p.d.upperMotionObserver.observabilityRank==4&&p.d.upperMotionObserver.maximumPoleMagnitude<1);
 row=struct('scenario',index-1,'sourceNativeScenario',configs(index).sourceNativeScenario,'massA_kg',p.mechanics.mA, ...
  'algebraicLoopCount',loops,'estimatorMaximumPoleMagnitude',p.d.upperMotionObserver.maximumPoleMagnitude);
 if isempty(audits),audits=row;else,audits(end+1)=row;end
end
metadata.savedGraphAudits=audits;metadata.algebraicLoopCount=0;
export_parameter_bank(mdl,configs,folder,source);
p=acf_prepare(configs(1).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
save_system(mdl,metadata.modelFile);metadata.privateModelSha256=sha256(metadata.modelFile);
metadata.nativeAlgorithmReusedExactly=true;metadata.nativeCGraphGenerated=false;
metadata.parentEmittedCSha256=sha256(fullfile(source,'code','acf_rl_native_grt_rtw','acf_rl_native.c'));
metadata.emittedCSha256=sha256(fullfile(folder,'code','acf_rl_native_grt_rtw','acf_rl_native.c'));
assert(strcmp(metadata.parentEmittedCSha256,metadata.emittedCSha256));
metadata.parameterOverrideFields={'Guidefrictionvelocityscale_Gain','GuideCoulombfrictioncurrent_Gai'};
metadata.preparationSentinelScenario=0;metadata.developmentScenarios=1:264;
metadata.status='accepted_emitted_native_graph_reused_with_verified_tunable_bank';
save(fullfile(folder,'prepared_configs.mat'),'p','loaded','-v7');write_json(fullfile(folder,'metadata.json'),metadata);
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'export_force_guide_native.m'));
fprintf('FORCE_GUIDE_V7_REUSED_NATIVE_READY %d cases\n',numel(configs));
end
function export_parameter_bank(mdl,configs,folder,source)
original=load(fullfile(source,'parameter_mapping.mat'),'map','values');map=original.map;
overrideNames={'Guidefrictionvelocityscale_Gain','GuideCoulombfrictioncurrent_Gai'};
overrideExpressions={'1/p.guideFeedforwardStudy.smoothingSpeed_m_s','p.inertial.guideFrictionFeedforwardScale*p.guideFeedforwardStudy.coulombScale*p.coil.coulombForce/p.coil.forceConstant'};
for j=1:2
 index=find(strcmp({map.name},overrideNames{j}));assert(isscalar(index)&&map(index).count==1&&strcmp(map(index).kind,'plain'));
 map(index).expression=overrideExpressions{j};
end
fid=fopen(fullfile(folder,'native_parameter_bank.c'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'#include "acf_rl_native.h"\nconst P_acf_rl_native_T acf_parameter_bank[%d] = {\n',numel(configs));
values=cell(numel(configs),numel(map));scenarios=struct([]);oldScenarios=jsondecode(fileread(fullfile(source,'scenarios.json')));
checked=0;
for k=1:numel(configs)
 p=acf_prepare(configs(k).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);fprintf(fid,' {\n');
 sourceIndex=configs(k).sourceNativeScenario+1;
 for j=1:numel(map)
  m=map(j);v=evalin(get_param(mdl,'ModelWorkspace'),m.expression);
  switch m.kind
   case 'sqrt',v=sqrt(v);
   case 'sparse_value',v=v(v~=0);
   case 'nonzero_value',v=v.';v=v(v~=0);
   case 'sparse_row',[row,~,~]=find(v);v=row-1;
   case 'sparse_col',v=[0,cumsum(sum(v~=0,1))];
  end
  assert(isreal(v)&&numel(v)==m.count,'C coefficient shape mismatch %d %s.',k,m.name);v=v(:);values{k,j}=v;
  if ~ismember(m.name,overrideNames)||k==1
   old=original.values{sourceIndex,j};assert(isequaln(v,old(:)),'Unexpected C coefficient change case%d %s.',k,m.name);checked=checked+1;
  end
  fprintf(fid,'  .%s = ',m.name);if m.count>1,fprintf(fid,'{');end
  for n=1:numel(v)
   if n>1,fprintf(fid,',');end
   if isinf(v(n)),if v(n)<0,fprintf(fid,'-');end;fprintf(fid,'INFINITY');
   elseif isnan(v(n)),fprintf(fid,'NAN');else,fprintf(fid,'%.17g',v(n));end
  end
  if m.count>1,fprintf(fid,'}');end;fprintf(fid,',\n');
 end
 fprintf(fid,' },\n');r=oldScenarios(sourceIndex);r.scenario=k-1;r.identity=configs(k).identity;
 if isempty(scenarios),scenarios=r;else,scenarios(end+1)=r;end
end
fprintf(fid,'};\nconst int acf_parameter_bank_count=%d;\n',numel(configs));
save(fullfile(folder,'parameter_mapping.mat'),'map','values','-v7');write_json(fullfile(folder,'parameter_mapping.json'),map);
write_json(fullfile(folder,'scenarios.json'),scenarios);
report=struct('status','all_reused_native_coefficients_equal_except_two_declared_controller_gains', ...
 'preparedRows',numel(configs),'exactNonOverrideFieldComparisons',checked,'originalCoderCase0AllFieldsExactlyEqual',true, ...
 'allowedOverrideFields',{overrideNames},'parentParameterMappingSha256',sha256(fullfile(source,'parameter_mapping.mat')), ...
 'contract','Source row is explicit perconfig. Every scalar/array/sparse C field exactly equals frozen source values, except two controller Coulomb compensation Gains. Sentinel row equals every originalCoderCase0 field, with no tolerance. Physical/observer friction and all mechanics/ADC/controller matrices unchanged.');
write_json(fullfile(folder,'tunable_coefficient_reuse_guard.json'),report);
end
function finish(mdl,storage,record,sdi)
try,close_system(mdl,0);catch,end
try,Simulink.sdi.clear;Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if isfile(fullfile(sdi,'owned_force_guide_v7_sdi')),try,rmdir(sdi,'s');catch,end,end
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
