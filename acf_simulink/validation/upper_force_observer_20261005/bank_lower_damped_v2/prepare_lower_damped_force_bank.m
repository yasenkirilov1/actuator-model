function manifest=prepare_lower_damped_force_bank(folder)
% Fixed upper20k/c0; restore lower60 damping with measured-force observer.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005','banks','upper_20k_lower_19746_undamped');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));recipe=fileparts(mfilename('fullpath'));
copyfile(fullfile(recipe,'lower_damped_v2','acf_prepare_upper_observer.m'),fullfile(folder,'helpers','acf_prepare_upper_observer.m'));
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
loaded=load(fullfile(source,'configs.mat'),'configs');configs=loaded.configs;
manifest=jsondecode(fileread(fullfile(source,'manifest.json')));metadata=manifest.cases;
design='upper_20k_lower_19746_damped_force_observer_v2';
for j=1:numel(configs)
 p=configs(j).p;
 assert(p.upperObserver.enabled&&~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0);
 assert(p.mechanics.k1==19746&&p.mechanics.c1==0&&p.inertial.lowerLoadInverse.enabled);
 p.mechanics.c1=60;p.inertial.lowerLoadInverse.identifiedDamping_Ns_m=60;
 p.inertial.lowerLoadInverse.dampingCalibrationScale=1;p.inertial.lowerLoadInverse.dampingCalibrationOffset_Ns_m=0;
 p.upperObserver.forceEstimator='matched_cell_and_holder_force_balance_discrete_lower_inverse';
 p.upperObserver.forceDerivativeCutoff_Hz=p.inertial.lowerLoadInverse.cutoff_Hz;
 p.upperObserver.contract='Four-state causal upper observer. Measured axial cell plus identified holder inertia/deformation balance; prepared discrete proper lower-link inverse coefficients multiplied by identifiedKf and driven by measured deltaL. Existing desired-load current inverse is separate. No extra ADC/delay/truth sensor. Upper20k/c0/unlimited is fixed, lower19746/c60 is the changed prototype candidate.';
 p.mechanicalStudy.design=design;p.mechanicalStudy.parentDesign=manifest.design;
 p.mechanicalStudy.upperLengthUnconstrained=true;p.mechanicalStudy.upperDesignFixed=true;
 p.mechanicalStudy.externalUpperPair.lengthAndTravelUnconstrained=true;
 p.mechanicalStudy.externalUpperPair.dynamicOperationQualified=false;
 p.mechanicalStudy.springPackingVerified=false;
 p.mechanicalStudy.combinedFreeSpringLength_m=2*(p.mechanicalStudy.externalLowerPair.freeLength_m+p.mechanicalStudy.externalLowerPair.freeLengthTolerance_m);
 p.mechanicalStudy.combinedInstalledLengthUpperBound_m=p.mechanicalStudy.combinedFreeSpringLength_m;
 p.mechanicalStudy.packingScope='Lower opposed pair only; upper length/travel unconstrained by user. Lower nominal free-length estimate does not qualify dynamic compression or purchased parts.';
 % Avoid imposing the old public combined upper+lower catalog constraint.
 if isfield(p,'springs'),p=rmfield(p,'springs');end
 p=acf_prepare(p);configs(j).p=p;
 configs(j).identity=strrep(configs(j).identity,manifest.design,design);
 metadata(j).identity=configs(j).identity;metadata(j).design=design;
 metadata(j).actualC_N_s_m=60;metadata(j).identifiedC_N_s_m=60;
 metadata(j).upperObserverForceSource=p.upperObserver.forceEstimator;
 metadata(j).forceDerivativeCutoff_Hz=p.inertial.lowerLoadInverse.cutoff_Hz;
 metadata(j).forceDerivativeDamping=p.inertial.lowerLoadInverse.filterDamping;
 metadata(j).upperLengthUnconstrained=true;metadata(j).upperDesignFixed=true;
 metadata(j).upperPair.lengthAndTravelUnconstrained=true;
end
save(fullfile(folder,'configs.mat'),'configs','-v7');
manifest.parentDesign=manifest.design;manifest.design=design;manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));
manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.status='frozen_lower_damped_force_observer_development_not_executed';
manifest.upperObserver=configs(1).p.upperObserver;manifest.mechanicalContract=configs(1).p.mechanicalStudy;
manifest.developmentContract=[manifest.developmentContract ' Upper20k/c0/unlimited fixed. Lower19746/c60, actual/identified known. Observer net force derived from existing matched cell and S acceleration with general spring/damper inverse. Same three gain profiles,11cases,twoA masses,seeds; no fresh-test tuning or training.'];
write_json(fullfile(folder,'manifest.json'),manifest);
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_lower_damped_force_bank.m'));
fprintf('UPPER_LOWER_DAMPED_FORCE_OBSERVER_BANK %d cases\n',numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
