function report=inertial_mechanics_run_calibrated_static_mixed_20260930(sourceFolder,outputFolder,baselineFolder)
% Three known-actual k/c mixed repeats with exactly16 static residuals halved.
% Runs the copied native model. No physical plant or sensor is implemented here.
assert(~usejava('desktop'),'Use isolated MATLAB -batch after the mesh frees RAM.');
if nargin<1
 sourceFolder='/home/tommie/.local/state/actuator-model/inertial-mechanics-mixed-corner8-20260930/frozen-calibrated-source';
end
if nargin<2,outputFolder=fileparts(mfilename('fullpath'));end
if nargin<3,baselineFolder=outputFolder;end
stem='inertial_mechanics_calibrated_static_mixed_20260930';
resultFile=fullfile(outputFolder,[stem '_results.json']);
assert(~isfile(resultFile),'Preserve prior evidence.');
manifest=jsondecode(fileread(fullfile(sourceFolder,'prepared_manifest.json')));
assert(strcmp(manifest.status,'prepared_not_executed'));
sourceModel=fullfile(sourceFolder,'acf_force_control_normal_trial.slx');
assert(strcmp(sha256(sourceModel),manifest.calibratedModelSHA256));
configFile=fullfile(sourceFolder,'joint_configs.mat');
assert(strcmp(sha256(configFile),manifest.jointConfigurationsSHA256));
loaded=load(configFile,'configs');sourceIndices=[8 18 28];configs=loaded.configs(sourceIndices);
workerFolder=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));
for name={'','helpers','cache','code','sdi'}
 mkdir(fullfile(workerFolder,name{1}));
end
copyfile(sourceModel,workerFolder);
copyfile(fullfile(sourceFolder,'helpers','*.m'),fullfile(workerFolder,'helpers'));
copyfile([mfilename('fullpath') '.m'],workerFolder);
addpath(fullfile(workerFolder,'helpers'),'-begin');
clear acf_prepare acf_run acf_metrics acf_select_actuator;
maxNumCompThreads(2);
previousStorage=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.fileGenControl('set','CacheFolder',fullfile(workerFolder,'cache'), ...
 'CodeGenFolder',fullfile(workerFolder,'code'),'createDir',true);
ownedStorage=fullfile(workerFolder,'sdi');
Simulink.sdi.setStorageLocation(ownedStorage);Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';assert(~bdIsLoaded(mdl));
load_system(fullfile(workerFolder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,previousStorage,previousRecord,ownedStorage)); %#ok<NASGU>
set_param(mdl,'SimulationMode','normal','FastRestart','on');
report=struct('status','running','date','2026-09-30','scope', ...
 'Three paired4s mixed actualk+20%/c-20% histories, with known actual inverse k/c unchanged and exactly16 static calibration residuals halved. Physical plant, PI/current, acquisition/noise, latency/robot residuals and startup remain fixed. A narrow paired cohort separate from the nominal3500 and long nominal-mechanics calibration records.', ...
 'sourceModel',sourceModel,'sourceModelSHA256',sha256(sourceModel), ...
 'baseV8NativeModelSHA256',manifest.baseV8NativeModelSHA256, ...
 'configurationSHA256',sha256(configFile),'configurationIndicesOneBased',[8 18 28], ...
 'recipeSHA256',sha256([mfilename('fullpath') '.m']),'helperSources',manifest.helperSources, ...
 'privateWorkerFolder',workerFolder,'simulationMode','normal','plannedCases',3, ...
 'executedCases',0,'preparationRejects',0,'passedCases',0,'rows',struct([]));
started=tic;
baselineFiles=cell(1,numel(configs));baselineHashes=cell(1,numel(configs));
staticGuards=cell(1,numel(configs));
try
for j=1:numel(configs)
 baselineFiles{j}=fullfile(baselineFolder,sprintf('inertial_mechanics_calibrated_mechanics_20260930_case_%02d_native.mat',j));
 baselineHashes{j}=sha256(baselineFiles{j});prior=load(baselineFiles{j},'r','row','config');
 assert(prior.row.seed==configs(j).p.sensor.seed && prior.row.passed);
 assert(isequaln(prior.r.parameters.uncertainty,configs(j).p.uncertainty));
 before=prior.r.parameters;configs(j)=prior.config;clear prior;
 assert(before.mechanics.k1==23695.2 && before.mechanics.c1==48 && ...
  before.inertial.lowerLoadInverse.identifiedStiffness_N_m==23695.2 && ...
  before.inertial.lowerLoadInverse.identifiedDamping_Ns_m==48);
 q=before;q.sensor.bias=before.sensor.bias*.5;
 q.contact.lateralBias_N=before.contact.lateralBias_N*.5;
 residualFields={'forceScaleZ','forceScaleX','forceFrameAngle_rad','accelFrameAngle_rad', ...
  'forceCrossZFromX','forceCrossXFromZ','accelBiasX_m_s2','positionScale'};
 for field=residualFields,q.uncertainty.(field{1})=before.uncertainty.(field{1})*.5;end
 q.accel.bias=before.accel.bias*.5;q.accel.scaleError=before.accel.scaleError*.5;
 q.sensor.distalMassCalibrationScale=1+.5*(before.sensor.distalMassCalibrationScale-1);
 q.sensor.distalMass_model_kg=q.mechanics.mS*q.sensor.distalMassCalibrationScale;
 q.uncertainty.massB_model_kg=q.mechanics.mB+.5*(before.uncertainty.massB_model_kg-q.mechanics.mB);
 q.uncertainty.forceConstant_model_N_A=q.coil.forceConstant+ ...
  .5*(before.uncertainty.forceConstant_model_N_A-q.coil.forceConstant);
 q.contact.commandedScanSpeed_m_s=q.surface.scanSpeed+ ...
  .5*(before.contact.commandedScanSpeed_m_s-q.surface.scanSpeed);
 if isfield(q.inertial,'holderMassCalibrationScale')
  q.inertial.holderMassCalibrationScale=q.uncertainty.massB_model_kg/q.mechanics.mB;
 end
 if isfield(q.inertial,'forceConstantCalibrationScale')
  q.inertial.forceConstantCalibrationScale=q.uncertainty.forceConstant_model_N_A/q.coil.forceConstant;
 end
 q.inertial.lowerLoadInverse.identifiedMassB_kg=q.uncertainty.massB_model_kg;
 q.inertial.lowerLoadInverse.identifiedForceConstant_N_A=q.uncertainty.forceConstant_model_N_A;
 q=acf_prepare(q);staticGuards{j}=static_calibration_guard(before,q);
 configs(j).p=q;
end
preparedConfigurationFile=fullfile(outputFolder,[stem '_configuration.mat']);
assert(~isfile(preparedConfigurationFile));
save(preparedConfigurationFile,'configs','baselineFiles','baselineHashes','staticGuards','-v7');
report.preparedConfigurationFile=preparedConfigurationFile;
report.preparedConfigurationSHA256=sha256(preparedConfigurationFile);
report.baselineNativeFiles=baselineFiles;report.baselineNativeSHA256=baselineHashes;
report.staticCalibrationGuards=staticGuards;report.onlySixteenStaticChangesVerified=true;
catch failure
 report.status='preparation_rejection';report.preparationRejects=1;
 report.exception=getReport(failure,'extended','hyperlinks','off');
 write_json(resultFile,report);rethrow(failure);
end
for j=1:numel(configs)
 config=configs(j);q=config.p;r=[];
 assert(q.mechanics.mS==2 && q.mechanics.mB==.6 && q.force.reference==50 && ...
  abs(q.mechanics.angle-pi/2)<1e-12 && q.contact.mu==.3);
 assert(q.coil.currentResponse==100e-6 && q.springs.commonPreload_N==33.6 && ...
  q.springs.lower.freeLength_m==.020 && q.acquisition.enabled && q.robot.forceGate.variation.enabled);
 identified=q.inertial.lowerLoadInverse;
 q.mechanics.k1=23695.2;q.mechanics.c1=48;q.springs.lower.rate_N_m=q.mechanics.k1/2;
 q.sim.stopTime=4;q.sim.evaluateFrom=0;q.sim.step=10e-6;
 q=acf_prepare(q);
 fields={'identifiedStiffness_N_m','identifiedDamping_Ns_m', ...
  'identifiedMassB_kg','identifiedForceConstant_N_A','numerator','denominator'};
 for f=fields,assert(isequal(q.inertial.lowerLoadInverse.(f{1}),identified.(f{1})));end
 timer=tic;
 try
  quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
  assert(numel(r.time)>=2 && size(r.data,1)==numel(r.time) && size(r.data,2)==35 && ...
   isequal(r.time,r.normalContact.time) && size(r.normalContact.data,2)==8);
  p=r.parameters;d=r.data;n=r.normalContact.data;
  assert(isequaln(p,q),'Native run changed the frozen prepared parameters.');
  for f=fields,assert(isequal(p.inertial.lowerLoadInverse.(f{1}),identified.(f{1})));end
  balance=max(abs(d(:,1)-d(:,13)-p.mechanics.mB*(p.d.g+d(:,12))-p.mechanics.mS*(p.d.g+d(:,30))));
  lowerBalance=max(abs(d(:,1)-d(:,24)-p.mechanics.mS*(p.d.g+d(:,30))));
  projection=max(abs(n(:,1)-(d(:,1)-n(:,3).*n(:,2))./sqrt(1+n(:,3).^2)));
  assert(max([balance lowerBalance projection])<1e-8,'Native force identities failed.');
  peak=max(abs(n(:,1)-q.force.reference));
  completed=r.time(end)>=q.sim.stopTime-q.sim.step/2;
  if completed,assert(numel(r.time)==400001);end
  noFailure=all(d(:,18)>.5) && all(d(:,21)<.5) && all(d(:,22)<.5) && all(d(:,23)<.5);
  [packed,low,high,free,installed,forceMin,forceMax]=packing(r);
  row=struct('caseIndex',j,'sourceConfigurationIndex',sourceIndices(j), ...
   'seed',q.sensor.seed,'executed',true,'completed',completed, ...
   'peakNormalForceError_N',peak,'forcePassed',completed && noFailure && peak<=.5+1e-9, ...
   'packingPassed',packed,'passed',completed && noFailure && peak<=.5+1e-9 && packed, ...
   'actualLowerStiffness_N_m',p.mechanics.k1,'actualLowerDamping_Ns_m',p.mechanics.c1, ...
   'identifiedLowerStiffness_N_m',p.inertial.lowerLoadInverse.identifiedStiffness_N_m, ...
   'identifiedLowerDamping_Ns_m',p.inertial.lowerLoadInverse.identifiedDamping_Ns_m, ...
   'identifiedMassB_kg',p.inertial.lowerLoadInverse.identifiedMassB_kg, ...
   'identifiedForceConstant_N_A',p.inertial.lowerLoadInverse.identifiedForceConstant_N_A, ...
   'identifiedMassS_kg',p.sensor.distalMass_model_kg, ...
   'feasibleCommonPreloadMin_N',low,'feasibleCommonPreloadMax_N',high, ...
   'configuredCommonPreload_N',p.springs.commonPreload_N, ...
   'combinedFreeSpringLength_mm',free*1000,'combinedInstalledSpringLength_mm',installed*1000, ...
   'individualSpringForceMin_N',forceMin,'individualSpringForceMax_N',forceMax, ...
   'maximumCurrent_A',max(abs(d(:,14))),'maximumVoltage_V',max(abs(d(:,15))), ...
   'forceBalanceMaxResidual_N',balance,'lowerForceBalanceMaxResidual_N',lowerBalance, ...
   'normalProjectionMaxResidual_N',projection,'uncertainty',q.uncertainty, ...
   'staticCalibrationGuard',staticGuards{j},'preparedParametersPreservedAfterRun',true,'runtimeSeconds',toc(timer));
  trace=fullfile(outputFolder,sprintf('%s_case_%02d_native.mat',stem,j));
  row.nativeTraceFile=trace;save(trace,'r','row','config','-v7');
  row.nativeTraceSHA256=sha256(trace);
 catch failure
  exception=getReport(failure,'extended','hyperlinks','off');
  if ~isempty(r),save(fullfile(outputFolder,sprintf('%s_case_%02d_unvalidated.mat',stem,j)), ...
    'r','q','config','exception','-v7');end
  report.status='preparation_or_validation_rejection';report.preparationRejects=report.preparationRejects+1;
  report.exception=exception;write_json(resultFile,report);rethrow(failure);
 end
 if isempty(report.rows),report.rows=row;else,report.rows(end+1)=row;end %#ok<AGROW>
 report.executedCases=numel(report.rows);report.passedCases=sum([report.rows.passed]);
 report.elapsedSeconds=toc(started);write_json(resultFile,report);
 fprintf('CALIBRATED_STATIC_MIXED_PROGRESS %d/3 seed%d error%.9gN force%d packing%d\n',j,q.sensor.seed,peak,row.forcePassed,packed);
 clear r;Simulink.sdi.clear;
end
report.status='complete';report.worstNormalForceError_N=max([report.rows.peakNormalForceError_N]);
report.commonPreloadIntersection_N=[max([report.rows.feasibleCommonPreloadMin_N]),min([report.rows.feasibleCommonPreloadMax_N])];
assert(strcmp(sha256(sourceModel),report.sourceModelSHA256));
write_json(resultFile,report);
end

function [passed,low,high,totalFree,totalInstalled,forceMin,forceMax]=packing(r)
p=r.parameters;s=p.springs.lower;rate=s.rate_N_m;preload=p.springs.commonPreload_N;
offset=p.d.q10/p.mechanics.k1+r.data(:,7)-r.data(:,6);
shift=max(abs(offset));
low=rate*(p.springs.unseatingClearance_m+shift);
cap=min(s.freeLength_m-s.freeLengthTolerance_m-s.minWorkingLength_m-p.springs.workingClearance_m,s.forceEnvelope_N/rate);
high=rate*(cap-shift);totalFree=2*(s.freeLength_m+s.freeLengthTolerance_m);
totalInstalled=2*(s.freeLength_m+s.freeLengthTolerance_m-preload/rate);
forceMin=preload-rate*shift;forceMax=preload+rate*shift;
passed=low<=preload+1e-9 && preload<=high+1e-9 && max(offset)-min(offset)<=s.maxCyclicStroke_m+1e-12 && ...
 totalFree<=p.springs.maxCombinedLength_m && totalInstalled<=p.springs.maxCombinedLength_m;
end
function write_json(file,data)
fid=fopen(file,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(data,'PrettyPrint',true),'char');
end
function result=sha256(path)
quote=char(39);escaped=strrep(path,quote,[quote char(34) quote char(34) quote]);
[status,out]=system(['sha256sum -- ' quote escaped quote]);assert(status==0);
result=out(1:64);assert(~isempty(regexp(result,'^[a-f0-9]{64}$','once')));
end
function finish(mdl,storage,record,owned)
try,set_param(mdl,'FastRestart','off');catch,end
try,close_system(mdl,0);catch,end
try,Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(record);catch,end
if ~strcmp(Simulink.sdi.getStorageLocation,owned),assert(isfolder(owned));end
end

function guard=static_calibration_guard(before,after)
primitive={'sensor.bias','contact.lateralBias_N','uncertainty.forceScaleZ','uncertainty.forceScaleX', ...
 'uncertainty.forceFrameAngle_rad','uncertainty.accelFrameAngle_rad', ...
 'uncertainty.forceCrossZFromX','uncertainty.forceCrossXFromZ', ...
 'accel.bias','uncertainty.accelBiasX_m_s2','accel.scaleError', ...
 'sensor.distalMassCalibrationScale','uncertainty.massB_model_kg', ...
 'uncertainty.forceConstant_model_N_A','uncertainty.positionScale','contact.commandedScanSpeed_m_s'};
derived={'sensor.distalMass_model_kg','inertial.holderMassCalibrationScale', ...
 'inertial.forceConstantCalibrationScale','inertial.lowerLoadInverse.identifiedMassB_kg', ...
 'inertial.lowerLoadInverse.identifiedForceConstant_N_A','inertial.lowerLoadInverse.numerator', ...
 'inertial.lowerLoadInverse.denominator'};
for path=primitive
 old=read_parameter(before,path{1});new=read_parameter(after,path{1});
 reference=0;
 if strcmp(path{1},'sensor.distalMassCalibrationScale'),reference=1;
 elseif strcmp(path{1},'uncertainty.massB_model_kg'),reference=before.mechanics.mB;
 elseif strcmp(path{1},'uncertainty.forceConstant_model_N_A'),reference=before.coil.forceConstant;
 elseif strcmp(path{1},'contact.commandedScanSpeed_m_s'),reference=before.surface.scanSpeed;
 end
 assert(isequaln(new,reference+.5*(old-reference)),['Residual was not exactly halved: ' path{1}]);
end
assert(after.inertial.lowerLoadInverse.identifiedStiffness_N_m==before.inertial.lowerLoadInverse.identifiedStiffness_N_m && ...
 after.inertial.lowerLoadInverse.identifiedDamping_Ns_m==before.inertial.lowerLoadInverse.identifiedDamping_Ns_m);
assert(after.sensor.distalMass_model_kg==after.mechanics.mS*after.sensor.distalMassCalibrationScale);
assert(after.inertial.lowerLoadInverse.identifiedMassB_kg==after.uncertainty.massB_model_kg && ...
 after.inertial.lowerLoadInverse.identifiedForceConstant_N_A==after.uncertainty.forceConstant_model_N_A);
if isfield(after.inertial,'holderMassCalibrationScale')
 assert(after.inertial.holderMassCalibrationScale==after.uncertainty.massB_model_kg/after.mechanics.mB);
end
if isfield(after.inertial,'forceConstantCalibrationScale')
 assert(after.inertial.forceConstantCalibrationScale==after.uncertainty.forceConstant_model_N_A/after.coil.forceConstant);
end
changed=parameter_changes(before,after,'');
coefficientChanges=startsWith(changed,'d.coilControl.');
assert(all(ismember(changed,[primitive,derived]) | coefficientChanges), ...
 'Calibration refinement changed an unapproved primitive or derived parameter.');
assert(numel(primitive)==16 && all(ismember(primitive,changed)), ...
 'Exactly sixteen declared static calibration residuals must be halved.');
beforeDerived=before.d;afterDerived=after.d;
beforeDerived=rmfield(beforeDerived,'coilControl');afterDerived=rmfield(afterDerived,'coilControl');
assert(isequaln(beforeDerived,afterDerived), ...
 'Physical startup, plant/current, sensor response, timing and observer coefficients changed.');
guard=struct('passed',true,'staticResidualCount',16,'primitiveAllowedPaths',{primitive}, ...
 'derivedAllowedPaths',{[derived,{'d.coilControl.* (identified-mass coefficients for inactive alternative)'}]}, ...
 'actualChangedPaths',{changed}, ...
 'physicalAndTimingCoefficientsIdentical',true,'originalSeed',before.sensor.seed, ...
 'jointCorner',before.uncertainty.jointCorner);
end

function changed=parameter_changes(before,after,prefix)
changed={};if isequaln(before,after),return;end
if isstruct(before) && isstruct(after) && isscalar(before) && isscalar(after)
 names=union(fieldnames(before),fieldnames(after));
 for index=1:numel(names)
  name=names{index};path=name;if ~isempty(prefix),path=[prefix '.' name];end
  if ~isfield(before,name) || ~isfield(after,name)
   changed{end+1}=path; %#ok<AGROW>
  else
   changed=[changed,parameter_changes(before.(name),after.(name),path)]; %#ok<AGROW>
  end
 end
else
 changed={prefix};
end
end

function value=read_parameter(p,path)
value=p;parts=strsplit(path,'.');for j=1:numel(parts),value=value.(parts{j});end
end

