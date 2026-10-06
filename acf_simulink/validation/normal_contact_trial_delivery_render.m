function report=normal_contact_trial_delivery_render(config)
% Export the saved native pages for review without opening the user's desktop.
if nargin<1,config=struct;end
if ~isfield(config,'workFolder'),config.workFolder=fullfile(getenv('HOME'),'.local','state','actuator-model','inertial-delivery-private-v8-simpleb-20260930');end
if ~isfield(config,'round'),config.round='review_round1';end
assert(~usejava('desktop'));
work=config.workFolder;folder=fullfile(work,'figures',config.round);if ~isfolder(folder),mkdir(folder);end
addpath(fullfile(work,'helpers'),'-begin');clear acf_prepare acf_select_actuator acf_select_controller;
mdl='acf_force_control_simple';load_system(fullfile(work,'generated',[mdl '.slx']));
cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
pages={mdl,[mdl '/Contact force measurement'],[mdl '/Contact force measurement/Contact force reconstruction'], ...
 [mdl '/Contact force measurement/Head motion estimator'],[mdl '/Contact force measurement/Normal force projection'], ...
 [mdl '/Force controller/Coil PI and PID'],[mdl '/Force controller/Coil PI and PID/Normal force PI and PID'], ...
 [mdl '/Force controller/Coil PI and PID/Measured motion feedforward'], ...
 [mdl '/Force controller/Coil PI and PID/Lower spring inverse feedforward'], ...
 [mdl '/Actuator and RABS mechanics'],[mdl '/Actuator and RABS mechanics/Actuator/Voice coil'], ...
 [mdl '/Recenter travel/Force settling gate'],[mdl '/S accelerometer and ADC']};
report=struct('workFolder',work,'round',config.round,'pages',struct([]));
for j=1:numel(pages)
 if getSimulinkBlockHandle(pages{j})==-1&&~strcmp(pages{j},mdl),continue;end
 filename=fullfile(folder,sprintf('page_%02d.png',j));
 print(['-s' pages{j}],'-dpng','-r120',filename);
 blocks=find_system(pages{j},'SearchDepth',1,'Type','block');rows=struct([]);
 for k=1:numel(blocks)
  if strcmp(blocks{k},pages{j}),continue;end
  one=struct('name',get_param(blocks{k},'Name'),'type',get_param(blocks{k},'BlockType'),'position',get_param(blocks{k},'Position'));
  if isempty(rows),rows=one;else,rows(end+1)=one;end %#ok<AGROW>
 end
 one=struct('path',pages{j},'image',filename,'blocks',rows);
 if isempty(report.pages),report.pages=one;else,report.pages(end+1)=one;end %#ok<AGROW>
end
fid=fopen(fullfile(folder,'pages.json'),'w');assert(fid>0);fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('DELIVERY DIAGRAMS EXPORTED %d\n',numel(report.pages));
end
