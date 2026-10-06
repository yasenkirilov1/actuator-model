function manifest=prepare_force_observer_bank(folder)
% Exact prior development profiles/cases; change only observer force source.
assert(~usejava('desktop'));maxNumCompThreads(2);assert(~isfolder(folder));mkdir(folder);
root=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
source=fullfile(root,'acf_simulink','validation','upper_no_damping_20261005','banks','upper_20k_lower_cell_undamped');
copyfile(fullfile(source,'helpers'),fullfile(folder,'helpers'));
recipe=fileparts(mfilename('fullpath'));
for name={'acf_prepare.m','acf_prepare_upper_observer.m'}
 copyfile(fullfile(recipe,name{1}),fullfile(folder,'helpers',name{1}));
end
addpath(fullfile(folder,'helpers'),'-begin');clear acf_prepare acf_adc_prepare acf_prepare_baseline_v8 acf_prepare_upper_observer;
loaded=load(fullfile(source,'configs.mat'),'configs');configs=loaded.configs;
manifest=jsondecode(fileread(fullfile(source,'manifest.json')));metadata=manifest.cases;
design='upper_20k_lower_cell_measured_force_observer';
for j=1:numel(configs)
 p=configs(j).p;
 assert(p.upperObserver.enabled&&~p.mechanics.upperRigid&&p.mechanics.k2==20000&&p.mechanics.c2==0);
 assert(p.mechanics.k1==64e6&&p.mechanics.c1==0&&~p.inertial.lowerLoadInverse.enabled);
 p.upperObserver.forceEstimator='matched_cell_and_holder_force_balance';
 p.upperObserver.forceDerivativeCutoff_Hz=1500;
 p.upperObserver.contract='Four-state causal upper observer. Net actuator-force deviation reconstructed from matched existing axial B-S cell and S acceleration, with proper critically damped 1.5kHz cell-deformation second derivative. No extra sensor/ADC/delay or truth/current-force input. Same prior development profiles and seeds.';
 p.mechanicalStudy.design=design;p.mechanicalStudy.parentDesign=manifest.design;
 p.mechanicalStudy.upperLengthUnconstrained=true;
 p.mechanicalStudy.externalUpperPair.lengthAndTravelUnconstrained=true;
 p.mechanicalStudy.externalUpperPair.dynamicOperationQualified=false;
 p=acf_prepare(p);configs(j).p=p;
 configs(j).identity=strrep(configs(j).identity,manifest.design,design);
 metadata(j).identity=configs(j).identity;metadata(j).design=design;
 metadata(j).upperObserverForceSource='matched_cell_and_holder_force_balance';
 metadata(j).forceDerivativeCutoff_Hz=1500;metadata(j).forceDerivativeDamping=1;
 metadata(j).upperLengthUnconstrained=true;
 metadata(j).upperPair.lengthAndTravelUnconstrained=true;
end
save(fullfile(folder,'configs.mat'),'configs','-v7');
manifest.parentDesign=manifest.design;manifest.design=design;manifest.cases=metadata;
manifest.parentConfigsSha256=sha256(fullfile(source,'configs.mat'));
manifest.configsSha256=sha256(fullfile(folder,'configs.mat'));
manifest.status='frozen_force_observer_development_not_executed';
manifest.upperObserver=configs(1).p.upperObserver;
manifest.mechanicalContract=configs(1).p.mechanicalStudy;
manifest.developmentContract=[manifest.developmentContract ' Upper force source alone changed to measured-cell/inertia balance; matched current/force clipping is now observed through existing sensors. Upper spring length/travel is unconstrained by user; actuator25mm travel and physical contact guards remain. No fresh-test outcomes or training used.'];
write_json(fullfile(folder,'manifest.json'),manifest);
copyfile([mfilename('fullpath') '.m'],fullfile(folder,'prepare_force_observer_bank.m'));
fprintf('UPPER_FORCE_OBSERVER_BANK %d cases\n',numel(configs));
end
function h=sha256(file)
fid=fopen(file,'rb');assert(fid>=0);clean=onCleanup(@()fclose(fid));d=java.security.MessageDigest.getInstance('SHA-256'); %#ok<NASGU>
while ~feof(fid),b=fread(fid,1024*1024,'*uint8');d.update(typecast(b,'int8'));end
h=lower(reshape(dec2hex(typecast(d.digest(),'uint8'),2)',1,[]));
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
