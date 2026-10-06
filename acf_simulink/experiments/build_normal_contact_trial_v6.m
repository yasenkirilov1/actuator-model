function report=build_normal_contact_trial_v6(config)
%BUILD_NORMAL_CONTACT_TRIAL_V6 Optional causal measured-drag geometry request.
if nargin<1,config=struct;end
if ~isfield(config,'sourceFolder'),config.sourceFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v5-20260930/frozen-inertial-v5';end
if ~isfield(config,'workFolder'),config.workFolder='/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v6-20260930';end
assert(~usejava('desktop') && ~strcmp(config.sourceFolder,config.workFolder));
for n={'','helpers','cache','code','sdi','traces'}
 f=fullfile(config.workFolder,n{1});if ~isfolder(f),mkdir(f);end
end
copyfile(fullfile(config.sourceFolder,'acf_force_control_normal_trial.slx'),fullfile(config.workFolder,'acf_force_control_normal_trial.slx'));
copyfile(fullfile(config.sourceFolder,'helpers','*.m'),fullfile(config.workFolder,'helpers'));
patch_prepare(fullfile(config.workFolder,'helpers','acf_prepare.m'));
patch_runner(fullfile(config.workFolder,'helpers','acf_run.m'));
addpath(fullfile(config.workFolder,'helpers'),'-begin');clear acf_run acf_prepare acf_select_actuator acf_select_controller;
Simulink.fileGenControl('set','CacheFolder',fullfile(config.workFolder,'cache'), ...
 'CodeGenFolder',fullfile(config.workFolder,'code'),'createDir',true);
oldSDI=Simulink.sdi.getStorageLocation;oldRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(config.workFolder,'sdi'));Simulink.sdi.setRecordData(true);
mdl='acf_force_control_normal_trial';load_system(fullfile(config.workFolder,[mdl '.slx']));
cleanup=onCleanup(@()finish(mdl,oldSDI,oldRecord)); %#ok<NASGU>
set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error');
z=load(fullfile(config.sourceFolder,'normal_contact_trial_candidate.mat'),'p');p=z.p;
p.contact.geometryMode='direct_lateral';p.contact.dragRatioFilter_s=.005;p.contact.dragRatioMinForce_N=1;
p=acf_prepare(p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
projection=[mdl '/Contact force measurement/Normal force projection'];
page=[projection '/Measured drag geometry'];
add_block('simulink/Ports & Subsystems/Subsystem',page,'Position',[1410 595 1760 765]);
Simulink.SubSystem.deleteContents(page);
add_drag_estimator(page);
inputs={'Compensated axial force/1','Compensated lateral force/1','Causal surface direction/1', ...
 'Measured normal force/1','Normalizer/1'};
for j=1:numel(inputs),add_line(projection,inputs{j},sprintf('Measured drag geometry/%d',j),'autorouting','on');end
block(projection,'simulink/Sources/Constant','Measured drag geometry selected',[1830 665 2080 710], ...
 'Value','double(strcmp(p.contact.geometryMode,''measured_drag_ratio''))');
block(projection,'simulink/Signal Routing/Switch','Select axial force geometry request',[2220 505 2280 660], ...
 'Criteria','u2 >= Threshold','Threshold','.5');
oldPorts=get_param([projection '/Requested axial force'],'PortHandles');oldLine=get_param(oldPorts.Outport(1),'Line');
destinations=get_param(oldLine,'DstPortHandle');delete_line(oldLine);
add_line(projection,'Measured drag geometry/1','Select axial force geometry request/1','autorouting','on');
add_line(projection,'Measured drag geometry selected/1','Select axial force geometry request/2','autorouting','on');
add_line(projection,'Requested axial force/1','Select axial force geometry request/3','autorouting','on');
newPorts=get_param([projection '/Select axial force geometry request'],'PortHandles');
for j=1:numel(destinations),add_line(projection,newPorts.Outport(1),destinations(j),'autorouting','on');end
add_drag_log(projection);
set_param(mdl,'SimulationCommand','update');loops=Simulink.BlockDiagram.getAlgebraicLoops(mdl);assert(isempty(loops));
save_system(mdl,fullfile(config.workFolder,[mdl '.slx']));save(fullfile(config.workFolder,'normal_contact_trial_candidate.mat'),'p');
report=struct('version','inertial-v6','sourceFolder',config.sourceFolder,'workFolder',config.workFolder, ...
 'geometryMode',p.contact.geometryMode,'dragRatioFilter_s',p.contact.dragRatioFilter_s, ...
 'minimumEstimatedNormalForce_N',p.contact.dragRatioMinForce_N,'ratioInitialCondition',0, ...
 'ratioBound',[-1 1],'algebraicLoopCount',numel(loops), ...
 'controllerInputs','Matched reconstructed force vector, causal measured direction, known force reference; no physical drag coefficient or surface truth.');
end

function add_drag_estimator(page)
names={'reconstructed_Rz_N','reconstructed_Rx_N','estimated_slope','estimated_normal_N','normalizer'};
positions={[20 75 50 89],[20 180 50 194],[20 280 50 294],[20 385 50 399],[20 685 50 699]};
for j=1:5,block(page,'simulink/Ports & Subsystems/In1',names{j},positions{j},'Port',num2str(j));end
block(page,'simulink/Math Operations/Product','Slope times axial reaction',[150 40 340 110],'Inputs','**');
block(page,'simulink/Math Operations/Sum','Tangential force numerator',[420 45 455 180],'Inputs','--','IconShape','rectangular');
block(page,'simulink/Math Operations/Product','Normal projection denominator',[150 375 345 450],'Inputs','**');
block(page,'simulink/Discontinuities/Saturation','Guard projection denominator',[420 385 640 440], ...
 'LowerLimit','p.contact.dragRatioMinForce_N','UpperLimit','inf');
block(page,'simulink/Math Operations/Product','Measured drag ratio',[720 130 910 230],'Inputs','*/');
block(page,'simulink/Discontinuities/Saturation','Bound signed drag ratio',[980 140 1190 195],'LowerLimit','-1','UpperLimit','1');
block(page,'simulink/Sources/Constant','Minimum observable normal force',[150 510 350 555],'Value','p.contact.dragRatioMinForce_N');
block(page,'simulink/Logic and Bit Operations/Relational Operator','Force sufficient for drag estimate',[440 520 495 610],'Operator','>');
block(page,'simulink/Discrete/Unit Delay','Previous filtered drag ratio',[980 700 1190 755], ...
 'SampleTime','p.force.sampleTime','InitialCondition','0');
block(page,'simulink/Signal Routing/Switch','Hold drag estimate below force guard',[1290 140 1345 310], ...
 'Criteria','u2 >= Threshold','Threshold','.5');
block(page,'simulink/Math Operations/Gain','New drag estimate weight',[1440 165 1635 220],'Gain','p.d.dragRatio.inputWeight');
block(page,'simulink/Math Operations/Gain','Previous drag estimate weight',[1440 335 1635 390],'Gain','p.d.dragRatio.memoryWeight');
block(page,'simulink/Math Operations/Sum','Filtered measured drag ratio',[1725 200 1770 365],'Inputs','++','IconShape','rectangular');
block(page,'simulink/Math Operations/Product','Estimated drag slope product',[1840 245 2050 335],'Inputs','**');
block(page,'simulink/Math Operations/Gain','Drag correction sign',[2120 255 2310 310],'Gain','-1');
block(page,'simulink/Math Operations/Bias','Axial normal-force geometry factor',[2380 255 2600 310],'Bias','1');
block(page,'simulink/Math Operations/Gain','Normal force reference',[2670 255 2850 310],'Gain','p.force.reference');
block(page,'simulink/Math Operations/Product','Requested axial force from measured drag',[2920 260 3170 380],'Inputs','*/');
block(page,'simulink/Signal Attributes/Data Type Conversion','Drag estimate valid as double',[720 560 915 605],'OutDataTypeStr','double');
block(page,'simulink/Ports & Subsystems/Out1','requested_Rz_N',[3270 288 3300 302],'Port','1');
block(page,'simulink/Ports & Subsystems/Out1','filtered_drag_ratio',[1970 110 2000 124],'Port','2');
block(page,'simulink/Ports & Subsystems/Out1','bounded_raw_drag_ratio',[1420 80 1450 94],'Port','3');
block(page,'simulink/Ports & Subsystems/Out1','drag_estimate_valid',[1070 575 1100 589],'Port','4');
wires={ ...
 'reconstructed_Rz_N/1','Slope times axial reaction/1';'estimated_slope/1','Slope times axial reaction/2'; ...
 'Slope times axial reaction/1','Tangential force numerator/1';'reconstructed_Rx_N/1','Tangential force numerator/2'; ...
 'estimated_normal_N/1','Normal projection denominator/1';'normalizer/1','Normal projection denominator/2'; ...
 'Normal projection denominator/1','Guard projection denominator/1';'Guard projection denominator/1','Measured drag ratio/2'; ...
 'Tangential force numerator/1','Measured drag ratio/1';'Measured drag ratio/1','Bound signed drag ratio/1'; ...
 'estimated_normal_N/1','Force sufficient for drag estimate/1';'Minimum observable normal force/1','Force sufficient for drag estimate/2'; ...
 'Bound signed drag ratio/1','Hold drag estimate below force guard/1';'Force sufficient for drag estimate/1','Hold drag estimate below force guard/2'; ...
 'Previous filtered drag ratio/1','Hold drag estimate below force guard/3'; ...
 'Hold drag estimate below force guard/1','New drag estimate weight/1';'Previous filtered drag ratio/1','Previous drag estimate weight/1'; ...
 'New drag estimate weight/1','Filtered measured drag ratio/1';'Previous drag estimate weight/1','Filtered measured drag ratio/2'; ...
 'Filtered measured drag ratio/1','Previous filtered drag ratio/1';'Filtered measured drag ratio/1','Estimated drag slope product/1'; ...
 'estimated_slope/1','Estimated drag slope product/2';'Estimated drag slope product/1','Drag correction sign/1'; ...
 'Drag correction sign/1','Axial normal-force geometry factor/1';'Axial normal-force geometry factor/1','Normal force reference/1'; ...
 'Normal force reference/1','Requested axial force from measured drag/1';'normalizer/1','Requested axial force from measured drag/2'; ...
 'Requested axial force from measured drag/1','requested_Rz_N/1';'Filtered measured drag ratio/1','filtered_drag_ratio/1'; ...
 'Bound signed drag ratio/1','bounded_raw_drag_ratio/1';'Force sufficient for drag estimate/1','Drag estimate valid as double/1'; ...
 'Drag estimate valid as double/1','drag_estimate_valid/1'};
for j=1:size(wires,1),add_line(page,wires{j,1},wires{j,2},'autorouting','on');end
Simulink.BlockDiagram.routeLine(find_system(page,'FindAll','on','SearchDepth',1,'Type','line'));
end

function add_drag_log(page)
block(page,'simulink/Signal Routing/Mux','Measured drag audit vector',[1910 980 1915 1120],'Inputs','3');
for j=1:3
 name=sprintf('Measured drag native sample %d',j);
 block(page,'simulink/Discrete/Zero-Order Hold',name,[1730 960+50*(j-1) 1850 995+50*(j-1)],'SampleTime','p.sim.step');
 add_line(page,sprintf('Measured drag geometry/%d',j+1),[name '/1'],'autorouting','on');
 add_line(page,[name '/1'],sprintf('Measured drag audit vector/%d',j),'autorouting','on');
end
block(page,'simulink/Sinks/To Workspace','Measured drag native log',[1980 1020 2200 1075], ...
 'VariableName','measured_drag_log','SaveFormat','Timeseries');
add_line(page,'Measured drag audit vector/1','Measured drag native log/1','autorouting','on');
end

function block(page,library,name,position,varargin)
add_block(library,[page '/' name],'Position',position,'FontSize','11',varargin{:});
end

function patch_prepare(path)
body=fileread(path);marker="if isfield(p,'accel') && p.accel.enabled";assert(contains(body,marker));
addition=strjoin([ ...
 "if isfield(p,'contact')"; ...
 " if ~isfield(p.contact,'geometryMode'),p.contact.geometryMode='direct_lateral';end"; ...
 " if ~isfield(p.contact,'dragRatioFilter_s'),p.contact.dragRatioFilter_s=.005;end"; ...
 " if ~isfield(p.contact,'dragRatioMinForce_N'),p.contact.dragRatioMinForce_N=1;end"; ...
 " assert(any(strcmp(p.contact.geometryMode,{'direct_lateral','measured_drag_ratio'}))&&p.contact.dragRatioFilter_s>0&&p.contact.dragRatioMinForce_N>0);"; ...
 " a=exp(-p.force.sampleTime/p.contact.dragRatioFilter_s);p.d.dragRatio=struct('memoryWeight',a,'inputWeight',1-a);"; ...
 "end";string(marker)],newline);
body=strrep(body,marker,addition);fid=fopen(path,'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',body);
end

function patch_runner(path)
body=fileread(path);marker="if any(strcmp(who(out),'force_variation_log'))";assert(contains(body,marker));
addition=strjoin([ ...
 "if any(strcmp(who(out),'measured_drag_log'))"; ...
 " logged=out.get('measured_drag_log');r.measuredDrag=struct('time',logged.Time(:),'data',rows(logged,3),'names',{{'filteredDragRatio','boundedRawDragRatio','estimateValid'}});"; ...
 "end";string(marker)],newline);
body=strrep(body,marker,addition);fid=fopen(path,'w');c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',body);
end

function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
