function report=normal_contact_trial_delivery_simplify(config)
% Private presentation cleanup; native behavior is checked against frozen V8.
if nargin<1,config=struct;end
state=fullfile(getenv('HOME'),'.local','state','actuator-model');
if ~isfield(config,'sourceFolder'),config.sourceFolder=fullfile(state,'inertial-delivery-private-v8d-20260930');end
if ~isfield(config,'workFolder'),config.workFolder=fullfile(state,'inertial-delivery-private-v8-simple-20260930');end
assert(~usejava('desktop'));assert(~isfolder(config.workFolder));work=config.workFolder;mdl='acf_force_control_simple';
for name={'','generated','helpers','cache','code','sdi','traces','figures'},mkdir(fullfile(work,name{1}));end
copyfile(fullfile(config.sourceFolder,'generated',[mdl '.slx']),fullfile(work,'generated'));
copyfile(fullfile(config.sourceFolder,'helpers','*.m'),fullfile(work,'helpers'));
file=fullfile(work,'helpers','acf_prepare.m');body=fileread(file);
marker='% Software controller defaults migrate parameter sets saved before this version.';
guard="assert(isfield(p,'acquisition')&&p.acquisition.enabled,'The delivered model uses engineered AA acquisition. Use a frozen experiment source for the legacy response.');"+newline+ ...
    "assert(isfield(p,'accel')&&p.accel.enabled,'The normal-force reconstruction requires the S-mounted accelerometer.');"+newline;
assert(contains(body,marker));body=strrep(body,marker,char(guard+marker));write_text(file,body);
addpath(fullfile(work,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator acf_select_controller;
Simulink.fileGenControl('set','CacheFolder',fullfile(work,'cache'),'CodeGenFolder',fullfile(work,'code'),'createDir',true);
load_system(fullfile(work,'generated',[mdl '.slx']));cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
force=[mdl '/Force measurement'];bypass_switch(force,'Select identified sensor response');
remove(force,{'Use engineered acquisition','Previous filtered force','New sample weight','Filter memory','Low pass filter','Measured force', ...
    'Receive filtered_force_N 1','Receive previous_sample_N 2','Receive noise_N 3','Receive bias_N 4', ...
    'Send filtered_force_N','Send previous_sample_N','Send noise_N','Send bias_N'});
lat=[mdl '/Lateral B-S force measurement'];bypass_switch(lat,'Select identified sensor response');
remove(lat,{'Use engineered acquisition','Filter memory','Old sample weight','New sample weight','Sensor low pass', ...
    'Previous filtered force','Filtered force memory','Noisy tared lateral force'});
accel=[mdl '/S accelerometer and ADC'];
for axis={'Z','X'}
    a=axis{1};bypass_switch(accel,[a ' select identified sensor response']);
    remove(accel,{[a ' use engineered acquisition'],[a ' filter memory'],[a ' new sample weight'],[a ' old sample weight'],[a ' identified sensor response']});
end
head=[mdl '/Contact force measurement/Head motion estimator'];
for name={'Position','Robot acceleration','Robot velocity'}
    bypass_switch(head,[name{1} ' acquisition frame']);remove(head,{[name{1} ' frame matching'],[name{1} ' use engineered acquisition']});
end
mw=get_param(mdl,'ModelWorkspace');
for variable={'p','pRecommended'}
    p=getVariable(mw,variable{1});p.springs.lower.freeLength_m=.020;p.springs.upper.freeLength_m=.020;
    p.springs.lower.part='Approximate guided spring target; confirm working envelope';p.springs.upper.part=p.springs.lower.part;
    assignin(mw,variable{1},acf_prepare(p));
end
selector=Simulink.Mask.get([mdl '/Force controller selection']);
selector.Display="disp(sprintf('Normal force control\n%s',get_param(gcb,'ControllerChoice')));";
% Keep all pages shallow and use ordinary Simulink arrangement for the chosen sensors.
pages={force,lat,accel,head,[mdl '/Contact force measurement'],[mdl '/Force controller/Coil PI and PID/Lower spring inverse feedforward']};
for j=1:numel(pages),Simulink.BlockDiagram.arrangeSystem(pages{j},FullLayout='true');end
set_param(mdl,'SimulationCommand','update');assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));
save_system(mdl);p=getVariable(mw,'p');save(fullfile(work,'delivery_candidate.mat'),'p','-v7');
report=struct('status','private simplified; pairing and visual review pending','workFolder',work, ...
    'removedLegacyBranches',true,'legacyModeExplicitlyRejected',true,'approximateSpringFreeLength_mm',20,'storedSpringPreload_N',p.springs.commonPreload_N);
write_text(fullfile(work,'simplification.json'),jsonencode(report,PrettyPrint=true));
fprintf('DELIVERY SIMPLIFIED %s\n',work);
end
function bypass_switch(sys,name)
ports=get_param([sys '/' name],'PortHandles');source=get_param(get_param(ports.Inport(1),'Line'),'SrcPortHandle');
line=get_param(ports.Outport(1),'Line');dest=get_param(line,'DstPortHandle');delete_line(line);delete_block([sys '/' name]);
for j=1:numel(dest),if dest(j)~=-1,add_line(sys,source,dest(j),'autorouting','on');end,end
end
function remove(sys,names)
for j=1:numel(names),name=[sys '/' names{j}];if getSimulinkBlockHandle(name)~=-1,delete_block(name);end,end
end
function write_text(file,body)
fid=fopen(file,'w');assert(fid>0);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',body);
end
