function manifest=prepare_cellsource_force_bank(folder)
% Correct the source wire; retain the exact previous physical development bank.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_force_observer_20261005','bank');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));recipe=fileparts(mfilename('fullpath'));
copyfile(fullfile(recipe,'cellsource_v2','acf_prepare_upper_observer.m'),fullfile(folder,'helpers','acf_prepare_upper_observer.m'));
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
loaded=load(fullfile(source,'configs.mat'),'configs');configs=loaded.configs;
manifest=jsondecode(fileread(fullfile(source,'manifest.json')));metadata=manifest.cases;
design='upper_20k_lower_cell_local_force_observer_v2';
for j=1:numel(configs)
 p=configs(j).p;
 assert(p.upperObserver.enabled&&~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0);
 assert(p.mechanics.k1==64e6&&p.mechanics.c1==0&&~p.inertial.lowerLoadInverse.enabled);
 p.upperObserver.forceEstimator='local_matched_cell_and_holder_force_balance';
 p.upperObserver.contract='Four-state causal upper observer driven by explicit local acquired axial cell deltaL from Cell error to deviation, identified holder inertia and proper1.5kHz cell-deformation derivative. Explicit local subsystem wire, numerical nonzero sign guard before export; no root-local From tag. No truth/preview/additional sensor.';
 p.mechanicalStudy.design=design;p.mechanicalStudy.parentDesign=manifest.design;
 p.mechanicalStudy.upperDesignFixed=true;p.mechanicalStudy.upperLengthUnconstrained=true;
 p.mechanicalStudy.springPackingVerified=false;
 p.mechanicalStudy.packingScope='Upper20k/c0 unlimited length-travel fixed by user. No lower external spring; finite64MN/m cell retained. Prior catalog metadata inactive and not a spring packing qualification.';
 p=acf_prepare(p);configs(j).p=p;
 configs(j).identity=strrep(configs(j).identity,manifest.design,design);
 metadata(j).identity=configs(j).identity;metadata(j).design=design;
 metadata(j).upperObserverForceSource=p.upperObserver.forceEstimator;
 metadata(j).upperDesignFixed=true;metadata(j).upperLengthUnconstrained=true;
end
save(fullfile(folder,'configs.mat'),'configs','-v7');
manifest.parentDesign=manifest.design;manifest.design=design;manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));
manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.status='frozen_local_cellsource_force_observer_development_not_executed';
manifest.upperObserver=configs(1).p.upperObserver;manifest.mechanicalContract=configs(1).p.mechanicalStudy;
manifest.developmentContract=[manifest.developmentContract ' Corrected source wired explicitly from the local matched acquired cell deviation. Previous root-local tag implementation is rejected wiring evidence; this is a separate candidate with a numerical source perturbation/sign guard. Exact prior sixPI profiles,11cases,twoA masses,seeds. Mandatory upper20k/c0/unlimited and lower rigid withfinite64MN/m cell/c0. No fresh-test tuning/training.'];
write_json(fullfile(folder,'manifest.json'),manifest);
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_cellsource_force_bank.m'));
fprintf('LOCAL_CELL_FORCE_OBSERVER_BANK %d cases\n',numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
