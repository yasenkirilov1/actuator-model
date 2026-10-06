function report=build_normal_contact_trial_v8(config)
% Private acquisition extension: common raw AA/EMA, synchronized causal frames.
% The frozen source and the user's desktop model are never modified.
if nargin<1,config=struct;end
root=fileparts(fileparts(mfilename('fullpath')));
if ~isfield(config,'sourceFolder'),config.sourceFolder=fullfile(getenv('HOME'),'.local','state','actuator-model','inertial-mechanics-selected-lower-series-20260930','lower_series_v1');end
if ~isfield(config,'workFolder'),config.workFolder=fullfile(getenv('HOME'),'.local','state','actuator-model','normal-contact-trial-inertial-v8-20260930');end
if ~isfield(config,'candidateFile'),config.candidateFile=fullfile(config.sourceFolder,'normal_contact_trial_candidate.mat');end
assert(~usejava('desktop'),'Use an isolated MATLAB batch process.');
assert(~isfolder(config.workFolder),'Choose a new private V8 directory.');
for name={'','helpers','cache','code','sdi','traces'},mkdir(fullfile(config.workFolder,name{1}));end
mdl='acf_force_control_normal_trial';source=fullfile(config.sourceFolder,[mdl '.slx']);target=fullfile(config.workFolder,[mdl '.slx']);
copyfile(source,target,'f');fileattrib(target,'+w');
copyfile(fullfile(config.sourceFolder,'helpers','*.m'),fullfile(config.workFolder,'helpers'),'f');
fileattrib(fullfile(config.workFolder,'helpers','*.m'),'+w');
copyfile([mfilename('fullpath') '.m'],fullfile(config.workFolder,'recipe_snapshot.m'));
patch_prepare(fullfile(config.workFolder,'helpers','acf_prepare.m'));
addpath(fullfile(config.workFolder,'helpers'),'-begin');clear acf_prepare acf_run acf_select_actuator;
loaded=load(config.candidateFile,'p');p=loaded.p;p.acquisition=acquisition_defaults;
p=acf_prepare(p);
previousSDI=Simulink.sdi.getStorageLocation;previousRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(fullfile(config.workFolder,'sdi'));Simulink.sdi.setRecordData(true);
Simulink.fileGenControl('set','CacheFolder',fullfile(config.workFolder,'cache'),'CodeGenFolder',fullfile(config.workFolder,'code'),'createDir',true);
cleanup=onCleanup(@()finish(mdl,previousSDI,previousRecord)); %#ok<NASGU>
load_system(target);
add_force_response([mdl '/Force measurement'],false);
add_force_response([mdl '/Lateral B-S force measurement'],true);
add_acceleration_response([mdl '/S accelerometer and ADC']);
add_position_clock([mdl '/Relative position measurement']);
add_frame_responses([mdl '/Contact force measurement/Head motion estimator']);
add_fine_robot_transport([mdl '/Robot command delay']);
% To Workspace does not upsample a fully discrete control vector on its own.
logPorts=get_param([mdl '/Control log'],'PortHandles');line=get_param(logPorts.Inport(1),'Line');
src=get_param(line,'SrcPortHandle');delete_line(mdl,src,logPorts.Inport(1));
add_block('simulink/Discrete/Zero-Order Hold',[mdl '/Native control history clock'],'SampleTime','p.sim.step','Position',[1570 1220 1840 1275]);
clockPorts=get_param([mdl '/Native control history clock'],'PortHandles');
add_line(mdl,src,clockPorts.Inport(1),'autorouting','on');add_line(mdl,clockPorts.Outport(1),logPorts.Inport(1),'autorouting','on');
mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);
acf_select_actuator(mdl,p.actuator.kind);
set_param(mdl,'FastRestart','off','AlgebraicLoopMsg','error','FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
set_param(mdl,'SimulationCommand','update');assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));
save_system(mdl,target);save(fullfile(config.workFolder,'normal_contact_trial_candidate.mat'),'p','-v7');
report=struct('status','built_not_qualified','sourceFolder',config.sourceFolder,'sourceModelSHA256',sha256(source), ...
    'sourceCandidateSHA256',sha256(config.candidateFile),'workFolder',config.workFolder,'modelSHA256',sha256(target), ...
    'acquisition',p.acquisition,'coefficients',p.d.acquisition, ...
    'controllerContract','Same native lower-series inverse and force controller; no added actuator prediction or profile input.', ...
    'noiseContract','Declared output RMS is preserved by raw white-noise impulse-energy normalization. ADC quantization remains after the response as a conservative option.', ...
    'frameContract','Force and S acceleration share raw response, nonzero pipeline and fresh 10 kHz latch. S-A position and issued-command motion use the same response and pipeline before the 2-state observer.');
files=dir(fullfile(config.workFolder,'helpers','*.m'));report.helperHashes=struct;
for j=1:numel(files),[~,name]=fileparts(files(j).name);report.helperHashes.(name)=sha256(fullfile(files(j).folder,files(j).name));end
write_text(fullfile(config.workFolder,'build_manifest.json'),jsonencode(report,PrettyPrint=true));
write_text(fullfile(root,'validation','normal_contact_trial_inertial_v8_build.json'),jsonencode(report,PrettyPrint=true));
fprintf('V8 PRIVATE BUILD READY %s\n',config.workFolder);
end

function c=acquisition_defaults
c=struct('enabled',false,'rawSampleTime_s',1e-5,'antiAliasCutoff_Hz',5100, ...
    'antiAliasDamping',1/sqrt(2),'emaTime_s',50e-6,'pipelineDelay_s',20e-6, ...
    'forceDelayOffset_s',0,'lateralDelayOffset_s',0,'accelerationDelayOffset_s',0,'positionDelayOffset_s',0,'requireFastVariationGate',true, ...
    'identifiedAntiAliasCutoff_Hz',5100,'identifiedEmaTime_s',50e-6,'identifiedPipelineDelay_s',20e-6, ...
    'forceAntiAliasScale',1,'lateralAntiAliasScale',1,'accelerationAntiAliasScale',1,'positionAntiAliasScale',1);
end

function patch_prepare(file)
t=fileread(file);marker='% Software controller defaults migrate parameter sets saved before this version.';
head=join([ ...
"if ~isfield(p,'acquisition'),p.acquisition=struct;end"; ...
"defaults=struct('enabled',false,'rawSampleTime_s',1e-5,'antiAliasCutoff_Hz',5100,'antiAliasDamping',1/sqrt(2),'emaTime_s',50e-6,'pipelineDelay_s',20e-6,'forceDelayOffset_s',0,'lateralDelayOffset_s',0,'accelerationDelayOffset_s',0,'positionDelayOffset_s',0,'requireFastVariationGate',true,'identifiedAntiAliasCutoff_Hz',5100,'identifiedEmaTime_s',50e-6,'identifiedPipelineDelay_s',20e-6,'forceAntiAliasScale',1,'lateralAntiAliasScale',1,'accelerationAntiAliasScale',1,'positionAntiAliasScale',1);"; ...
"for name=fieldnames(defaults)',if ~isfield(p.acquisition,name{1}),p.acquisition.(name{1})=defaults.(name{1});end,end"; ...
"c=p.acquisition;assert(isscalar(c.enabled)&&any(c.enabled==[false true]));"; ...
"if c.enabled"; ...
" if c.requireFastVariationGate,p.robot.forceGate.enabled=true;p.robot.forceGate.variation.enabled=true;end"; ...
" p.sensor.sampleTime=c.rawSampleTime_s;p.sensor.delay=c.pipelineDelay_s+c.forceDelayOffset_s;"; ...
" p.accel.delay=c.pipelineDelay_s+c.accelerationDelayOffset_s;p.sensor.strokeDelay=c.pipelineDelay_s+c.positionDelayOffset_s;"; ...
" p.contact.lateralDelaySamples=round((c.pipelineDelay_s+c.lateralDelayOffset_s)/p.sensor.sampleTime);"; ...
" assert(p.sim.step<=c.rawSampleTime_s && mod(round(p.force.sampleTime/p.sim.step),round(c.rawSampleTime_s/p.sim.step))==0,'Resolve raw acquisition and use integer-rate control.');"; ...
" assert(abs(p.contact.lateralDelaySamples*p.sensor.sampleTime-c.pipelineDelay_s-c.lateralDelayOffset_s)<1e-12);"; ...
"end"; ...
"p.d.positionSampleTime=p.force.sampleTime;if c.enabled,p.d.positionSampleTime=p.sensor.sampleTime;end"; ...
"p.d.actualRobotTransportSampleTime_s=p.robot.sampleTime;if c.enabled,p.d.actualRobotTransportSampleTime_s=p.force.sampleTime;end"; ...
],newline);
assert(contains(t,marker));t=strrep(t,marker,string(marker)+newline+head);
old="assert(abs(p.force.sampleTime-p.sensor.sampleTime)<1e-12, ...";
assert(contains(t,old));t=strrep(t,old,"assert(p.acquisition.enabled || abs(p.force.sampleTime-p.sensor.sampleTime)<1e-12, ...");
t=strrep(t,"ticks(p.sensor.strokeDelay,p.force.sampleTime,'sensor.strokeDelay',true)","ticks(p.sensor.strokeDelay,p.d.positionSampleTime,'sensor.strokeDelay',true)");
t=strrep(t,"ticks(p.robot.delay,p.robot.sampleTime,'robot.delay',true)","ticks(p.robot.delay,p.d.actualRobotTransportSampleTime_s,'robot.delay',true)");
marker="if ~isfield(p.sensor,'positionModel')";
mass=join([ ...
"if ~isfield(p.sensor,'distalMassCalibrationScale')"; ...
" if p.sensor.distalMass_kg>0,p.sensor.distalMassCalibrationScale=p.sensor.distalMass_model_kg/p.sensor.distalMass_kg;else,p.sensor.distalMassCalibrationScale=1;end"; ...
"end"; ...
"assert(isfinite(p.sensor.distalMassCalibrationScale)&&p.sensor.distalMassCalibrationScale>0);"; ...
"p.sensor.distalMass_kg=p.mechanics.mS;p.sensor.distalMass_model_kg=p.mechanics.mS*p.sensor.distalMassCalibrationScale;"; ...
],newline);
assert(contains(t,marker));t=strrep(t,marker,mass+newline+marker);
marker=newline+"end"+newline+newline+"function n = ticks";
tail=join([ ...
"c=p.acquisition;h=c.rawSampleTime_s;assert(h>0&&c.antiAliasCutoff_Hz>0&&c.antiAliasCutoff_Hz<.45/h&&c.antiAliasDamping>0&&c.emaTime_s>0&&c.pipelineDelay_s>0);"; ...
"w=tan(pi*c.antiAliasCutoff_Hz*h);aaDen=[1+2*c.antiAliasDamping*w+w*w,2*(w*w-1),1-2*c.antiAliasDamping*w+w*w];"; ...
"aaNum=w*w*[1,2,1]/aaDen(1);aaDen=aaDen/aaDen(1);alpha=exp(-h/c.emaTime_s);"; ...
"num=conv(aaNum,[1-alpha,0]);den=conv(aaDen,[1,-alpha]);"; ...
"impulse=zeros(20000,1);impulse(1)=1;response=filter(num,den,impulse);energy=sum(response.^2);assert(energy>0&&energy<1&&max(abs(roots(den)))<1);"; ...
"p.d.acquisition=struct('numerator',num,'denominator',den,'steadyStatePerUnitInput',ones(3,1)/sum(den),'impulseEnergy',energy,'noiseInputScale',1/sqrt(energy),'rawSampleTime_s',h,'nominalLowFrequencyDelay_s',sum((0:3).*num)/sum(num)*h-sum((0:3).*den)/sum(den)*h);"; ...
"% Actual sensor tolerance must not change the identified command-motion frame."; ...
"p.d.acquisition.responses=struct;channels={'force','lateral','acceleration','position','frame'};"; ...
"for j=1:numel(channels)"; ...
" channel=channels{j};fc=c.antiAliasCutoff_Hz;te=c.emaTime_s;z=c.antiAliasDamping;"; ...
" if strcmp(channel,'frame'),fc=c.identifiedAntiAliasCutoff_Hz;te=c.identifiedEmaTime_s;z=defaults.antiAliasDamping;else,fc=fc*c.([channel 'AntiAliasScale']);end"; ...
" assert(fc>0&&fc<.45/h&&te>0);w=tan(pi*fc*h);dd=[1+2*z*w+w*w,2*(w*w-1),1-2*z*w+w*w];bb=w*w*[1,2,1]/dd(1);dd=dd/dd(1);ar=exp(-h/te);bb=conv(bb,[1-ar,0]);dd=conv(dd,[1,-ar]);"; ...
" p.d.acquisition.responses.(channel)=struct('numerator',bb,'denominator',dd,'steadyStatePerUnitInput',ones(3,1)/sum(dd),'cutoff_Hz',fc,'emaTime_s',te);"; ...
"end"; ...
"% Raw noise is referenced to the nominal engineering path, not actual AA tolerance."; ...
"w=tan(pi*defaults.antiAliasCutoff_Hz*h);dd=[1+2*defaults.antiAliasDamping*w+w*w,2*(w*w-1),1-2*defaults.antiAliasDamping*w+w*w];bb=w*w*[1,2,1]/dd(1);dd=dd/dd(1);ar=exp(-h/defaults.emaTime_s);bb=conv(bb,[1-ar,0]);dd=conv(dd,[1,-ar]);rr=filter(bb,dd,impulse);"; ...
"p.d.acquisition.noiseReferenceImpulseEnergy=sum(rr.^2);p.d.acquisition.noiseInputScale=1/sqrt(p.d.acquisition.noiseReferenceImpulseEnergy);"; ...
"p.d.acquisition.frameDelaySamples=p.d.sensorDelaySamples;"; ...
"if c.enabled,p.d.acquisition.frameDelaySamples=round(c.identifiedPipelineDelay_s/p.sensor.sampleTime);assert(p.d.acquisition.frameDelaySamples>=0&&abs(p.d.acquisition.frameDelaySamples*p.sensor.sampleTime-c.identifiedPipelineDelay_s)<1e-12);end"; ...
"p.d.acquisition.accelerationNoiseInputScale=sqrt((1+exp(-p.sensor.sampleTime/p.accel.filterTime))/(1-exp(-p.sensor.sampleTime/p.accel.filterTime)));"; ...
"p.d.acquisition.positionNoiseInputScale=1;"; ...
"if c.enabled,p.d.acquisition.accelerationNoiseInputScale=p.d.acquisition.noiseInputScale;p.d.acquisition.positionNoiseInputScale=p.d.acquisition.noiseInputScale;end"; ...
],newline);
assert(contains(t,marker));t=strrep(t,marker,newline+tail+marker);
write_text(file,t);
end

function add_force_response(sys,lateral)
if lateral
    noise='Noise RMS';bias='Known tare residual bias';legacy='Noisy tared lateral force';initial='-p.force.reference*p.contact.mu';
else
    noise='Sensor noise RMS';bias='Sensor bias';legacy='Measured force';initial='p.force.reference';
end
add_block('simulink/Math Operations/Gain',[sys '/Raw output-noise normalization'],'Gain','p.d.acquisition.noiseInputScale','Position',[800 350 1030 410]);
add_block('simulink/Math Operations/Sum',[sys '/Raw noisy force sample'],'Inputs','++','Position',[1120 220 1160 280]);
response='force';if lateral,response='lateral';end
add_fast_tf(sys,'Engineered common AA and EMA',initial,[1230 200 1520 285],response);
add_block('simulink/Math Operations/Sum',[sys '/Conditioned force with tare residual'],'Inputs','++','Position',[1580 205 1620 270]);
add_block('simulink/Sources/Constant',[sys '/Use engineered acquisition'],'Value','double(p.acquisition.enabled)','Position',[1280 350 1510 390]);
add_block('simulink/Signal Routing/Switch',[sys '/Select identified sensor response'],'Criteria','u2 >= Threshold','Threshold','.5','Position',[1710 190 1760 300]);
add_line(sys,[noise '/1'],'Raw output-noise normalization/1','autorouting','on');
add_line(sys,'Sensor sampling/1','Raw noisy force sample/1','autorouting','on');
add_line(sys,'Raw output-noise normalization/1','Raw noisy force sample/2','autorouting','on');
add_line(sys,'Raw noisy force sample/1','Engineered common AA and EMA/1','autorouting','on');
add_line(sys,'Engineered common AA and EMA/1','Conditioned force with tare residual/1','autorouting','on');
add_line(sys,[bias '/1'],'Conditioned force with tare residual/2','autorouting','on');
add_line(sys,'Conditioned force with tare residual/1','Select identified sensor response/1','autorouting','on');
add_line(sys,'Use engineered acquisition/1','Select identified sensor response/2','autorouting','on');
add_line(sys,[legacy '/1'],'Select identified sensor response/3','autorouting','on');
replace_input(sys,'Sensor sample delay',1,'Select identified sensor response/1');
insert_latch(sys,'Sensor sample delay','Fresh 10 kHz force frame');
end

function add_acceleration_response(sys)
for axis={'Z','X'}
    a=axis{1};gravity='p.d.g';if strcmp(a,'X'),gravity='p.mechanics.gravity*sin(p.mechanics.angle)';end
    add_fast_tf(sys,[a ' engineered common AA and EMA'],['(1+p.accel.scaleError)*(' gravity ')'],[1410 90+390*(a=='X') 1750 175+390*(a=='X')],'acceleration');
    add_block('simulink/Sources/Constant',[sys '/' a ' use engineered acquisition'],'Value','double(p.acquisition.enabled)','Position',[1450 210+390*(a=='X') 1700 250+390*(a=='X')]);
    add_block('simulink/Signal Routing/Switch',[sys '/' a ' select identified sensor response'],'Criteria','u2 >= Threshold','Threshold','.5','Position',[1850 90+390*(a=='X') 1910 195+390*(a=='X')]);
    add_line(sys,[a ' noisy sensor/1'],[a ' engineered common AA and EMA/1'],'autorouting','on');
    add_line(sys,[a ' engineered common AA and EMA/1'],[a ' select identified sensor response/1'],'autorouting','on');
    add_line(sys,[a ' use engineered acquisition/1'],[a ' select identified sensor response/2'],'autorouting','on');
    add_line(sys,[a ' identified sensor response/1'],[a ' select identified sensor response/3'],'autorouting','on');
    replace_input(sys,[a ' acceleration range'],1,[a ' select identified sensor response/1']);
    set_param([sys '/' a ' raw noise RMS'],'Gain','p.accel.noiseRms*p.d.acquisition.accelerationNoiseInputScale');
    insert_latch(sys,[a ' known channel delay'],[a ' fresh 10 kHz acceleration frame']);
end
end

function add_position_clock(sys)
for name={'Position sampling','Position noise','Measurement delay'},set_param([sys '/' name{1}],'SampleTime','p.d.positionSampleTime');end
set_param([sys '/Position noise RMS'],'Gain','p.sensor.strokeNoiseRms*p.d.acquisition.positionNoiseInputScale');
end

function add_frame_responses(sys)
names={'Position','Robot acceleration','Robot velocity'};
for j=1:3
    old=[names{j} ' frame matching'];fast=[names{j} ' common AA and EMA'];sw=[names{j} ' acquisition frame'];
    ph=get_param([sys '/' old],'PortHandles');line=get_param(ph.Inport(1),'Line');source=get_param(line,'SrcPortHandle');
    response='frame';if j==1,response='position';end
    pos=get_param([sys '/' old],'Position');add_fast_tf(sys,fast,'0',pos+[0 300 80 330],response);
    add_block('simulink/Sources/Constant',[sys '/' names{j} ' use engineered acquisition'],'Value','double(p.acquisition.enabled)','Position',pos+[0 420 80 420]);
    add_block('simulink/Signal Routing/Switch',[sys '/' sw],'Criteria','u2 >= Threshold','Threshold','.5','Position',pos+[440 180 350 230]);
    add_line(sys,source,get_param([sys '/' fast],'PortHandles').Inport(1),'autorouting','on');
    oldDest=destinations(sys,old);disconnect_output(sys,old);
    add_line(sys,[fast '/1'],[sw '/1'],'autorouting','on');
    add_line(sys,[names{j} ' use engineered acquisition/1'],[sw '/2'],'autorouting','on');
    add_line(sys,[old '/1'],[sw '/3'],'autorouting','on');
    reconnect(sys,sw,oldDest);
    if j==1,insert_latch(sys,sw,'Fresh 10 kHz position frame');
    else
        set_param([sys '/' names{j} ' frame matching delay'],'DelayLength','p.d.acquisition.frameDelaySamples');
        insert_latch(sys,[names{j} ' frame matching delay'],['Fresh 10 kHz ' lower(names{j}) ' frame']);
    end
end
end

function add_fast_tf(sys,name,initial,pos,response)
prefix=['p.d.acquisition.responses.' response];
add_block('simulink/Discrete/Discrete Transfer Fcn',[sys '/' name], ...
    'Numerator',[prefix '.numerator'],'Denominator',[prefix '.denominator'], ...
    'InitialStates',['(' initial ')*' prefix '.steadyStatePerUnitInput'], ...
    'SampleTime','p.sensor.sampleTime','Position',pos);
end
function add_fine_robot_transport(sys)
replace_input_source_clock(sys,'Robot command samples');
set_param([sys '/Robot command samples'],'SampleTime','p.d.actualRobotTransportSampleTime_s');
end
function replace_input_source_clock(sys,name)
ports=get_param([sys '/' name],'PortHandles');line=get_param(ports.Inport(1),'Line');source=get_param(line,'SrcPortHandle');
delete_line(sys,source,ports.Inport(1));
add_block('simulink/Discrete/Zero-Order Hold',[sys '/Held issued command at transport clock'],'SampleTime','p.d.actualRobotTransportSampleTime_s','Position',[190 30 465 90]);
clock=get_param([sys '/Held issued command at transport clock'],'PortHandles');add_line(sys,source,clock.Inport(1),'autorouting','on');add_line(sys,clock.Outport(1),ports.Inport(1),'autorouting','on');
end
function insert_latch(sys,source,name)
dest=destinations(sys,source);disconnect_output(sys,source);pos=get_param([sys '/' source],'Position');
add_block('simulink/Discrete/Zero-Order Hold',[sys '/' name],'SampleTime','p.force.sampleTime','Position',pos+[250 0 400 0]);
add_line(sys,[source '/1'],[name '/1'],'autorouting','on');reconnect(sys,name,dest);
end
function dest=destinations(sys,source)
ph=get_param([sys '/' source],'PortHandles');line=get_param(ph.Outport(1),'Line');
assert(line~=-1);dest=get_param(line,'DstPortHandle');dest=dest(dest~=-1);
end
function disconnect_output(sys,source)
ph=get_param([sys '/' source],'PortHandles');line=get_param(ph.Outport(1),'Line');delete_line(line);
end
function reconnect(sys,source,dest)
ph=get_param([sys '/' source],'PortHandles');for j=1:numel(dest),add_line(sys,ph.Outport(1),dest(j),'autorouting','on');end
end
function replace_input(sys,name,port,source)
ph=get_param([sys '/' name],'PortHandles');line=get_param(ph.Inport(port),'Line');assert(line~=-1);
old=get_param(line,'SrcPortHandle');delete_line(sys,old,ph.Inport(port));
new=get_param([sys '/' extractBefore(source,'/')],'PortHandles');index=str2double(extractAfter(source,'/'));
add_line(sys,new.Outport(index),ph.Inport(port),'autorouting','on');
end
function write_text(file,t)
fid=fopen(file,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',t);
end
function hash=sha256(file)
escaped=['''' strrep(file,'''','''"''"''') ''''];[code,out]=system(['sha256sum -- ' escaped]);assert(code==0);hash=strtok(out);
end
function finish(mdl,oldSDI,oldRecord)
if bdIsLoaded(mdl),set_param(mdl,'FastRestart','off');close_system(mdl,0);end
Simulink.sdi.clear;Simulink.sdi.setStorageLocation(oldSDI);Simulink.sdi.setRecordData(oldRecord);
end
