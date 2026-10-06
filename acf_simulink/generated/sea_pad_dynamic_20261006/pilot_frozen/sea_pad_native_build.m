function summary=sea_pad_native_build(p,modelFile,cacheFolder)
% Separate research plant. All time-domain physics is ordinary Simulink blocks.
% p is an explicit, flat native replay contract prepared before simulation.
assert(~usejava('desktop'),'Run this builder in its owned batch process.');
maxNumCompThreads(2);
[folder,mdl]=fileparts(modelFile);if ~isfolder(folder),mkdir(folder);end
assert(~bdIsLoaded(mdl));
Simulink.fileGenControl('set','CacheFolder',fullfile(cacheFolder,'cache'), ...
    'CodeGenFolder',fullfile(cacheFolder,'code'),'createDir',true);
new_system(mdl);cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
mw=get_param(mdl,'ModelWorkspace');assignin(mw,'p',p);
assignin(mw,'replayInputs',[0 p.initial(9) 0;p.stopTime p.initial(9) 0]);
set_param(mdl,'Solver','ode4','SolverType','Fixed-step','FixedStep',sprintf('%.17g',p.dt), ...
    'StopTime','p.stopTime','SaveTime','on','TimeSaveName','tout','SaveOutput','on', ...
    'OutputSaveName','yout','SaveFormat','Dataset','ReturnWorkspaceOutputs','on', ...
    'SignalLogging','off','LimitDataPoints','off','ScreenColor','white', ...
    'LoadExternalInput','on','ExternalInput','replayInputs');
add_block('simulink/Sources/In1',[mdl '/Current request A'],'Port','1','Position',[25 85 55 105]);
add_block('simulink/Discrete/Zero-Order Hold',[mdl '/Fresh 10 kHz current command'],'SampleTime','p.controlStep','Position',[75 80 110 110]);
add_block('simulink/Sources/In1',[mdl '/Robot displacement m'],'Port','2','Position',[25 430 55 450]);
add_block('simulink/Sources/Clock',[mdl '/Time'],'Position',[25 270 55 290]);
ss(mdl,'Electrical drive',[130 60 320 180]);
ss(mdl,'Spring and independent screw nut',[385 60 620 210]);
ss(mdl,'A B and S dynamics',[385 350 620 535]);
ss(mdl,'Pad total force',[720 255 940 395]);
ss(mdl,'Physical stops',[720 450 940 545]);
drive([mdl '/Electrical drive']);actuator([mdl '/Spring and independent screw nut']);
bodies([mdl '/A B and S dynamics']);pad([mdl '/Pad total force']);stops([mdl '/Physical stops']);
add_block('simulink/Signal Routing/Mux',[mdl '/Physical states and totals'],'Inputs','9','Position',[1030 110 1035 350]);
add_block('simulink/Sinks/Stop Simulation',[mdl '/Stop on physical failure'],'Position',[1000 450 1130 490]);
add_block('simulink/Sinks/Out1',[mdl '/Plant outputs'],'Port','1','Position',[1110 220 1140 240]);
wire(mdl,'Current request A/1','Fresh 10 kHz current command/1');wire(mdl,'Fresh 10 kHz current command/1','Electrical drive/1');
wire(mdl,'A B and S dynamics/1','Electrical drive/2');
wire(mdl,'Spring and independent screw nut/3','Electrical drive/3');
wire(mdl,'Electrical drive/1','Spring and independent screw nut/1');
wire(mdl,'A B and S dynamics/1','Spring and independent screw nut/2');
wire(mdl,'A B and S dynamics/2','Spring and independent screw nut/3');
wire(mdl,'Robot displacement m/1','A B and S dynamics/1');
wire(mdl,'Spring and independent screw nut/1','A B and S dynamics/2');
wire(mdl,'Pad total force/1','A B and S dynamics/3');
wire(mdl,'Spring and independent screw nut/2','A B and S dynamics/4');
wire(mdl,'Pad total force/3','A B and S dynamics/5');
wire(mdl,'Time/1','Pad total force/1');wire(mdl,'A B and S dynamics/1','Pad total force/2');
wire(mdl,'A B and S dynamics/1','Physical states and totals/1');
wire(mdl,'Spring and independent screw nut/3','Physical states and totals/2');
wire(mdl,'Electrical drive/1','Physical states and totals/3');
wire(mdl,'Pad total force/2','Physical states and totals/4');
wire(mdl,'A B and S dynamics/2','Physical states and totals/5');
wire(mdl,'Electrical drive/2','Physical states and totals/6');
wire(mdl,'A B and S dynamics/3','Physical states and totals/7');
wire(mdl,'Spring and independent screw nut/1','Physical states and totals/8');
wire(mdl,'Spring and independent screw nut/4','Physical states and totals/9');
wire(mdl,'A B and S dynamics/1','Physical stops/1');
wire(mdl,'Spring and independent screw nut/3','Physical stops/2');
wire(mdl,'Pad total force/2','Physical stops/3');wire(mdl,'Physical stops/1','Stop on physical failure/1');
wire(mdl,'Physical states and totals/1','Plant outputs/1');
systems={mdl,[mdl '/Electrical drive'],[mdl '/Spring and independent screw nut'], ...
    [mdl '/A B and S dynamics'],[mdl '/Pad total force'],[mdl '/Physical stops']};
for j=1:numel(systems)
    try,Simulink.BlockDiagram.arrangeSystem(systems{j});catch,end
    set_param(systems{j},'ZoomFactor','FitSystem');
end
blocks=find_system(mdl,'LookUnderMasks','all','Type','block');
for j=1:numel(blocks),try,set_param(blocks{j},'ForegroundColor','black','BackgroundColor','white');catch,end,end
note=Simulink.Annotation(mdl,sprintf(['SERIES-ELASTIC ACTUATOR AND SOFT-PAD RESEARCH PLANT\n' ...
    'Upper 20 N/mm, no external upper damping; finite 64 MN/m sensing cell.\n' ...
    'Current commands and robot displacement are replay inputs, not an independent controller.\n' ...
    'p.sea selects coil (0) or screw + internal spring (1). Pad contact is summed locally.']));
note.Position=[20 -120 930 -35];note.ForegroundColor='black';
save_system(mdl,modelFile);
summary=struct('status','native_research_plant_prepared_not_simulated','modelFile',modelFile, ...
    'physicalPlantNative',true,'hiddenMatlabPlantSolver',false,'controllerMode','external_command_replay', ...
    'outputNames',{{'xA','vA','xB','vB','xS','vS','q','vq','i','Fc_axial','Fn_sum','active_weight','minimum_compression','maximum_compression','qUpper','qCell','Vreq','Vactual','backEMF','aA','aB','aS','net_internal_force','qdd'}}, ...
    'physicalStateNames',{{'xA','vA','xB','vB','xS','vS','q','vq','i'}}, ...
    'subsystemCount',5,'blockCount',numel(blocks));
end

function drive(s)
inp(s,'Current request',1);inp(s,'Body states',2);inp(s,'Nut states',3);
demux(s,'Body coordinates','6');demux(s,'Nut coordinates','2');
wire(s,'Body states/1','Body coordinates/1');wire(s,'Nut states/1','Nut coordinates/1');
sumblock(s,'Gap speed','+-');wire(s,'Body coordinates/2','Gap speed/1');wire(s,'Body coordinates/4','Gap speed/2');
gain(s,'Coil back EMF','p.Kf');wire(s,'Gap speed/1','Coil back EMF/1');
gain(s,'Motor back EMF','p.Ke/p.ell');wire(s,'Nut coordinates/2','Motor back EMF/1');
selector(s,'Coil or motor back EMF');wire(s,'Motor back EMF/1','Coil or motor back EMF/1');wire(s,'Coil back EMF/1','Coil or motor back EMF/3');
sat(s,'Current command limit','-p.Imax','p.Imax');wire(s,'Current request/1','Current command limit/1');
integrator(s,'Winding current','p.initial(9)');
sumblock(s,'Current response error','+-');wire(s,'Current command limit/1','Current response error/1');wire(s,'Winding current/1','Current response error/2');
gain(s,'Inductive voltage request','p.L/p.currentTau');wire(s,'Current response error/1','Inductive voltage request/1');
gain(s,'Resistive drop','p.R');wire(s,'Winding current/1','Resistive drop/1');
sumblock(s,'Requested terminal voltage','+++');wire(s,'Resistive drop/1','Requested terminal voltage/1');
wire(s,'Coil or motor back EMF/1','Requested terminal voltage/2');wire(s,'Inductive voltage request/1','Requested terminal voltage/3');
sat(s,'Usable bipolar voltage','-p.Vmax','p.Vmax');wire(s,'Requested terminal voltage/1','Usable bipolar voltage/1');
sumblock(s,'Inductor voltage','+--');wire(s,'Usable bipolar voltage/1','Inductor voltage/1');wire(s,'Resistive drop/1','Inductor voltage/2');wire(s,'Coil or motor back EMF/1','Inductor voltage/3');
gain(s,'Current derivative','1/p.L');wire(s,'Inductor voltage/1','Current derivative/1');wire(s,'Current derivative/1','Winding current/1');
outp(s,'Current',1);wire(s,'Winding current/1','Current/1');
mux(s,'Electrical diagnostics',3);wire(s,'Requested terminal voltage/1','Electrical diagnostics/1');wire(s,'Usable bipolar voltage/1','Electrical diagnostics/2');wire(s,'Coil or motor back EMF/1','Electrical diagnostics/3');
outp(s,'Voltage diagnostics',2);wire(s,'Electrical diagnostics/1','Voltage diagnostics/1');
end

function actuator(s)
inp(s,'Current',1);inp(s,'Body states',2);inp(s,'Upper and cell loads',3);
demux(s,'Body coordinates','6');wire(s,'Body states/1','Body coordinates/1');
demux(s,'Support loads','2');wire(s,'Upper and cell loads/1','Support loads/1');
sumblock(s,'A minus B travel','+-');wire(s,'Body coordinates/1','A minus B travel/1');wire(s,'Body coordinates/3','A minus B travel/2');
sumblock(s,'A minus B velocity','+-');wire(s,'Body coordinates/2','A minus B velocity/1');wire(s,'Body coordinates/4','A minus B velocity/2');
gain(s,'Smoothed guide speed','1/p.guideSmooth');wire(s,'A minus B velocity/1','Smoothed guide speed/1');
trig(s,'Guide Coulomb sign','tanh');wire(s,'Smoothed guide speed/1','Guide Coulomb sign/1');
gain(s,'Coulomb guide force','p.guideCoulomb');wire(s,'Guide Coulomb sign/1','Coulomb guide force/1');
gain(s,'Viscous guide force','p.guideViscous');wire(s,'A minus B velocity/1','Viscous guide force/1');
sumblock(s,'Guide resistance','++');wire(s,'Coulomb guide force/1','Guide resistance/1');wire(s,'Viscous guide force/1','Guide resistance/2');
gain(s,'Electromagnetic coil force','p.Kf');wire(s,'Current/1','Electromagnetic coil force/1');
sumblock(s,'Net coil force','+-');wire(s,'Electromagnetic coil force/1','Net coil force/1');wire(s,'Guide resistance/1','Net coil force/2');
integrator(s,'Independent nut position','p.initial(7)');integrator(s,'Independent nut velocity','p.initial(8)');
sumblock(s,'Internal spring deflection','+-');wire(s,'Independent nut position/1','Internal spring deflection/1');wire(s,'A minus B travel/1','Internal spring deflection/2');
gain(s,'Internal spring force','p.kSea');wire(s,'Internal spring deflection/1','Internal spring force/1');
sumblock(s,'Spring relative speed','+-');wire(s,'Independent nut velocity/1','Spring relative speed/1');wire(s,'A minus B velocity/1','Spring relative speed/2');
gain(s,'Internal optional damping','p.cSea');wire(s,'Spring relative speed/1','Internal optional damping/1');
constant(s,'Internal preload','p.preload');sumblock(s,'Series force','+++');wire(s,'Internal spring force/1','Series force/1');wire(s,'Internal optional damping/1','Series force/2');wire(s,'Internal preload/1','Series force/3');
sumblock(s,'Net series force','+-');wire(s,'Series force/1','Net series force/1');wire(s,'Guide resistance/1','Net series force/2');
gain(s,'Shaft speed','1/p.ell');wire(s,'Independent nut velocity/1','Shaft speed/1');
gain(s,'Smoothed shaft speed','1/p.shaftSmooth');wire(s,'Shaft speed/1','Smoothed shaft speed/1');
trig(s,'Shaft Coulomb sign','tanh');wire(s,'Smoothed shaft speed/1','Shaft Coulomb sign/1');
gain(s,'Shaft Coulomb torque','p.shaftCoulomb');wire(s,'Shaft Coulomb sign/1','Shaft Coulomb torque/1');
gain(s,'Shaft viscous torque','p.shaftViscous');wire(s,'Shaft speed/1','Shaft viscous torque/1');
gain(s,'Motor torque','p.Kt');wire(s,'Current/1','Motor torque/1');
sumblock(s,'Net shaft torque','+--');wire(s,'Motor torque/1','Net shaft torque/1');wire(s,'Shaft Coulomb torque/1','Net shaft torque/2');wire(s,'Shaft viscous torque/1','Net shaft torque/3');
gain(s,'Nut drive force','1/p.ell');wire(s,'Net shaft torque/1','Nut drive force/1');
constant(s,'A and nut gravity','(p.mA+p.mNut)*p.gAxis');sumblock(s,'A coupled generalized force','+--');wire(s,'Net series force/1','A coupled generalized force/1');wire(s,'Support loads/1','A coupled generalized force/2');wire(s,'A and nut gravity/1','A coupled generalized force/3');
constant(s,'Nut gravity','p.mNut*p.gAxis');sumblock(s,'Nut generalized force','+-+');wire(s,'Nut drive force/1','Nut generalized force/1');wire(s,'Series force/1','Nut generalized force/2');wire(s,'Nut gravity/1','Nut generalized force/3');
mux(s,'Coupled generalized loads',2);wire(s,'A coupled generalized force/1','Coupled generalized loads/1');wire(s,'Nut generalized force/1','Coupled generalized loads/2');
gain(s,'A nut inverse mass matrix','p.massInverse','Matrix(K*u)');wire(s,'Coupled generalized loads/1','A nut inverse mass matrix/1');
demux(s,'Coupled accelerations','2');wire(s,'A nut inverse mass matrix/1','Coupled accelerations/1');
gain(s,'Nut branch enabled','p.sea');wire(s,'Coupled accelerations/2','Nut branch enabled/1');wire(s,'Nut branch enabled/1','Independent nut velocity/1');
gain(s,'Nut position enabled','p.sea');wire(s,'Independent nut velocity/1','Nut position enabled/1');wire(s,'Nut position enabled/1','Independent nut position/1');
selector(s,'Internal actuator selector');wire(s,'Net series force/1','Internal actuator selector/1');wire(s,'Net coil force/1','Internal actuator selector/3');
outp(s,'Net actuator force',1);wire(s,'Internal actuator selector/1','Net actuator force/1');outp(s,'Coupled A acceleration',2);wire(s,'Coupled accelerations/1','Coupled A acceleration/1');
mux(s,'Nut coordinates',2);wire(s,'Independent nut position/1','Nut coordinates/1');wire(s,'Independent nut velocity/1','Nut coordinates/2');outp(s,'Nut states',3);wire(s,'Nut coordinates/1','Nut states/1');
outp(s,'Nut acceleration',4);wire(s,'Nut branch enabled/1','Nut acceleration/1');
end

function bodies(s)
inp(s,'Robot displacement',1);inp(s,'Internal net force',2);inp(s,'Pad axial force',3);inp(s,'Coupled A acceleration',4);inp(s,'Imposed S motion',5);
demux(s,'Surface kinematics','3');wire(s,'Imposed S motion/1','Surface kinematics/1');
for j=1:3
    tag=char('A'+j-1);if j==3,tag='S';end
    integrator(s,[tag ' position'],sprintf('p.initial(%d)',2*j-1));integrator(s,[tag ' velocity'],sprintf('p.initial(%d)',2*j));
    wire(s,[tag ' velocity/1'],[tag ' position/1']);
end
rigidselector(s,'Measured S position');wire(s,'Surface kinematics/1','Measured S position/1');wire(s,'S position/1','Measured S position/3');
rigidselector(s,'Measured S velocity');wire(s,'Surface kinematics/2','Measured S velocity/1');wire(s,'S velocity/1','Measured S velocity/3');
sumblock(s,'Upper spring extension','+-');wire(s,'A position/1','Upper spring extension/1');wire(s,'Robot displacement/1','Upper spring extension/2');
gain(s,'Prescribed upper spring','p.kUpper');wire(s,'Upper spring extension/1','Prescribed upper spring/1');
sumblock(s,'Finite cell extension','+-');wire(s,'Measured S position/1','Finite cell extension/1');wire(s,'B position/1','Finite cell extension/2');
gain(s,'Finite sensing cell','p.kCell');wire(s,'Finite cell extension/1','Finite sensing cell/1');
constant(s,'A gravity force','p.mA*p.gAxis');sumblock(s,'A net force','+--');wire(s,'Internal net force/1','A net force/1');wire(s,'Prescribed upper spring/1','A net force/2');wire(s,'A gravity force/1','A net force/3');
gain(s,'Coil A acceleration','1/p.mA');wire(s,'A net force/1','Coil A acceleration/1');selector(s,'A inertia selector');wire(s,'Coupled A acceleration/1','A inertia selector/1');wire(s,'Coil A acceleration/1','A inertia selector/3');wire(s,'A inertia selector/1','A velocity/1');
constant(s,'B gravity force','p.mB*p.gAxis');sumblock(s,'B net force','+--');wire(s,'Finite sensing cell/1','B net force/1');wire(s,'Internal net force/1','B net force/2');wire(s,'B gravity force/1','B net force/3');gain(s,'B inverse mass','1/p.mB');wire(s,'B net force/1','B inverse mass/1');wire(s,'B inverse mass/1','B velocity/1');
constant(s,'S gravity force','p.mS*p.gAxis');sumblock(s,'S net force','+--');wire(s,'Pad axial force/1','S net force/1');wire(s,'Finite sensing cell/1','S net force/2');wire(s,'S gravity force/1','S net force/3');gain(s,'S inverse mass','1/p.mS');wire(s,'S net force/1','S inverse mass/1');
rigidselector(s,'S acceleration selector');wire(s,'Surface kinematics/3','S acceleration selector/1');wire(s,'S inverse mass/1','S acceleration selector/3');wire(s,'S acceleration selector/1','S velocity/1');
mux(s,'Body coordinates',6);names={'A position','A velocity','B position','B velocity','Measured S position','Measured S velocity'};
for j=1:numel(names),wire(s,[names{j} '/1'],sprintf('Body coordinates/%d',j));end
outp(s,'Body states',1);wire(s,'Body coordinates/1','Body states/1');
mux(s,'Support and sensing loads',2);wire(s,'Prescribed upper spring/1','Support and sensing loads/1');wire(s,'Finite sensing cell/1','Support and sensing loads/2');outp(s,'Upper and cell loads',2);wire(s,'Support and sensing loads/1','Upper and cell loads/1');
mux(s,'Body accelerations',3);wire(s,'A inertia selector/1','Body accelerations/1');wire(s,'B inverse mass/1','Body accelerations/2');wire(s,'S acceleration selector/1','Body accelerations/3');outp(s,'Accelerations',3);wire(s,'Body accelerations/1','Accelerations/1');
end

function pad(s)
inp(s,'Time',1);inp(s,'Body states',2);demux(s,'Body coordinates','6');wire(s,'Body states/1','Body coordinates/1');
constant(s,'Bump entry time','p.bumpTime');sumblock(s,'Time from bump entry','+-');wire(s,'Time/1','Time from bump entry/1');wire(s,'Bump entry time/1','Time from bump entry/2');
gain(s,'Scanning position','p.scanSpeed');wire(s,'Time from bump entry/1','Scanning position/1');constant(s,'Pad strip offsets','p.offsets');sumblock(s,'Local positions','++');wire(s,'Scanning position/1','Local positions/1');wire(s,'Pad strip offsets/1','Local positions/2');gain(s,'Relative bump coordinate','1/p.bumpWidth');wire(s,'Local positions/1','Relative bump coordinate/1');sat(s,'Bump interval','0','1');wire(s,'Relative bump coordinate/1','Bump interval/1');
constant(s,'One','1');sumblock(s,'One minus coordinate','+-');wire(s,'One/1','One minus coordinate/1');wire(s,'Bump interval/1','One minus coordinate/2');product(s,'Coordinate envelope','**');wire(s,'Bump interval/1','Coordinate envelope/1');wire(s,'One minus coordinate/1','Coordinate envelope/2');
product(s,'Envelope squared','**');wire(s,'Coordinate envelope/1','Envelope squared/1');wire(s,'Coordinate envelope/1','Envelope squared/2');product(s,'Envelope cubed','**');wire(s,'Envelope squared/1','Envelope cubed/1');wire(s,'Coordinate envelope/1','Envelope cubed/2');gain(s,'Local height','64*p.bumpHeight');wire(s,'Envelope cubed/1','Local height/1');
gain(s,'Twice coordinate','2');wire(s,'Bump interval/1','Twice coordinate/1');sumblock(s,'Slope sign','+-');wire(s,'One/1','Slope sign/1');wire(s,'Twice coordinate/1','Slope sign/2');product(s,'Slope polynomial','**');wire(s,'Envelope squared/1','Slope polynomial/1');wire(s,'Slope sign/1','Slope polynomial/2');gain(s,'Local slope','192*p.bumpHeight/p.bumpWidth');wire(s,'Slope polynomial/1','Local slope/1');gain(s,'Local height velocity','p.scanSpeed');wire(s,'Local slope/1','Local height velocity/1');
product(s,'Slope sign squared','**');wire(s,'Slope sign/1','Slope sign squared/1');wire(s,'Slope sign/1','Slope sign squared/2');sumblock(s,'Acceleration factor','+-');wire(s,'Slope sign squared/1','Acceleration factor/1');wire(s,'Coordinate envelope/1','Acceleration factor/2');product(s,'Acceleration polynomial','**');wire(s,'Coordinate envelope/1','Acceleration polynomial/1');wire(s,'Acceleration factor/1','Acceleration polynomial/2');gain(s,'Local height acceleration','384*p.bumpHeight*(p.scanSpeed/p.bumpWidth)^2');wire(s,'Acceleration polynomial/1','Local height acceleration/1');
gain(s,'Center height','p.centerWeight','Matrix(K*u)');wire(s,'Local height/1','Center height/1');gain(s,'Center velocity','p.centerWeight','Matrix(K*u)');wire(s,'Local height velocity/1','Center velocity/1');gain(s,'Center acceleration','p.centerWeight','Matrix(K*u)');wire(s,'Local height acceleration/1','Center acceleration/1');
mux(s,'Center surface kinematics',3);wire(s,'Center height/1','Center surface kinematics/1');wire(s,'Center velocity/1','Center surface kinematics/2');wire(s,'Center acceleration/1','Center surface kinematics/3');outp(s,'Imposed S motion',3);wire(s,'Center surface kinematics/1','Imposed S motion/1');
sumblock(s,'Local compression','+-');wire(s,'Local height/1','Local compression/1');wire(s,'Body coordinates/5','Local compression/2');sumblock(s,'Local compression speed','+-');wire(s,'Local height velocity/1','Local compression speed/1');wire(s,'Body coordinates/6','Local compression speed/2');
sat(s,'Elastic positive compression','0','inf');wire(s,'Local compression/1','Elastic positive compression/1');gain(s,'Distributed elastic load','p.kPad');wire(s,'Elastic positive compression/1','Distributed elastic load/1');gain(s,'Distributed damping load','p.cPad');wire(s,'Local compression speed/1','Distributed damping load/1');sumblock(s,'Local constitutive load','++');wire(s,'Distributed elastic load/1','Local constitutive load/1');wire(s,'Distributed damping load/1','Local constitutive load/2');sat(s,'No local tensile force','0','inf');wire(s,'Local constitutive load/1','No local tensile force/1');
add_block('simulink/Logic and Bit Operations/Compare To Constant',[s '/Local contact active'],'relop','>','const','0','Position',nextpos());wire(s,'Local compression/1','Local contact active/1');
add_block('simulink/Signal Attributes/Data Type Conversion',[s '/Contact indicator'],'OutDataTypeStr','double','Position',nextpos());wire(s,'Local contact active/1','Contact indicator/1');
product(s,'Unilateral local load','**');wire(s,'No local tensile force/1','Unilateral local load/1');wire(s,'Contact indicator/1','Unilateral local load/2');
gain(s,'Weighted total axial force','p.weights','Matrix(K*u)');wire(s,'Unilateral local load/1','Weighted total axial force/1');
sumblock(s,'Rigid contact cell extension','+-');wire(s,'Center height/1','Rigid contact cell extension/1');wire(s,'Body coordinates/3','Rigid contact cell extension/2');gain(s,'Rigid contact cell load','p.kCell');wire(s,'Rigid contact cell extension/1','Rigid contact cell load/1');constant(s,'S gravity acceleration','p.gAxis');sumblock(s,'Rigid contact inertia acceleration','++');wire(s,'Center acceleration/1','Rigid contact inertia acceleration/1');wire(s,'S gravity acceleration/1','Rigid contact inertia acceleration/2');gain(s,'Rigid S inertia force','p.mS');wire(s,'Rigid contact inertia acceleration/1','Rigid S inertia force/1');sumblock(s,'Rigid contact total force','++');wire(s,'Rigid contact cell load/1','Rigid contact total force/1');wire(s,'Rigid S inertia force/1','Rigid contact total force/2');
rigidselector(s,'Imposed or compliant total force');wire(s,'Rigid contact total force/1','Imposed or compliant total force/1');wire(s,'Weighted total axial force/1','Imposed or compliant total force/3');outp(s,'Axial force',1);wire(s,'Imposed or compliant total force/1','Axial force/1');
product(s,'Slope squared','**');wire(s,'Local slope/1','Slope squared/1');wire(s,'Local slope/1','Slope squared/2');sumblock(s,'Local normal normalization','++');wire(s,'One/1','Local normal normalization/1');wire(s,'Slope squared/1','Local normal normalization/2');add_block('simulink/Math Operations/Sqrt',[s '/Local normal factor'],'Position',nextpos());wire(s,'Local normal normalization/1','Local normal factor/1');product(s,'Normal force magnitudes','**');wire(s,'Unilateral local load/1','Normal force magnitudes/1');wire(s,'Local normal factor/1','Normal force magnitudes/2');gain(s,'Sum local normal forces','p.weights','Matrix(K*u)');wire(s,'Normal force magnitudes/1','Sum local normal forces/1');gain(s,'Active footprint fraction','p.weights','Matrix(K*u)');wire(s,'Contact indicator/1','Active footprint fraction/1');
gain(s,'Center normal factor','p.centerWeight','Matrix(K*u)');wire(s,'Local normal factor/1','Center normal factor/1');product(s,'Rigid normal magnitude','**');wire(s,'Rigid contact total force/1','Rigid normal magnitude/1');wire(s,'Center normal factor/1','Rigid normal magnitude/2');rigidselector(s,'Imposed or compliant normal total');wire(s,'Rigid normal magnitude/1','Imposed or compliant normal total/1');wire(s,'Sum local normal forces/1','Imposed or compliant normal total/3');
constant(s,'Full point contact','1');rigidselector(s,'Imposed or compliant active fraction');wire(s,'Full point contact/1','Imposed or compliant active fraction/1');wire(s,'Active footprint fraction/1','Imposed or compliant active fraction/3');
add_block('simulink/Math Operations/MinMax',[s '/Minimum compression'],'Function','min','Inputs','1','Position',nextpos());wire(s,'Local compression/1','Minimum compression/1');
add_block('simulink/Math Operations/MinMax',[s '/Maximum compression'],'Function','max','Inputs','1','Position',nextpos());wire(s,'Local compression/1','Maximum compression/1');
mux(s,'Total contact diagnostics',5);wire(s,'Imposed or compliant total force/1','Total contact diagnostics/1');wire(s,'Imposed or compliant normal total/1','Total contact diagnostics/2');wire(s,'Imposed or compliant active fraction/1','Total contact diagnostics/3');wire(s,'Minimum compression/1','Total contact diagnostics/4');wire(s,'Maximum compression/1','Total contact diagnostics/5');outp(s,'Contact totals',2);wire(s,'Total contact diagnostics/1','Contact totals/1');
end

function stops(s)
inp(s,'Body states',1);inp(s,'Nut states',2);inp(s,'Contact totals',3);
demux(s,'Body coordinates','6');wire(s,'Body states/1','Body coordinates/1');demux(s,'Nut coordinates','2');wire(s,'Nut states/1','Nut coordinates/1');demux(s,'Pad diagnostics','5');wire(s,'Contact totals/1','Pad diagnostics/1');
sumblock(s,'Actuator travel','+-');wire(s,'Body coordinates/1','Actuator travel/1');wire(s,'Body coordinates/3','Actuator travel/2');absblock(s,'Absolute actuator travel');wire(s,'Actuator travel/1','Absolute actuator travel/1');constant(s,'Flange hard limit','p.toolTravelLimit');compare(s,'Flange stop','>');wire(s,'Absolute actuator travel/1','Flange stop/1');wire(s,'Flange hard limit/1','Flange stop/2');
absblock(s,'Absolute nut travel');wire(s,'Nut coordinates/1','Absolute nut travel/1');constant(s,'Nut hard limit','p.nutTravelLimit');compare(s,'Nut stop','>');wire(s,'Absolute nut travel/1','Nut stop/1');wire(s,'Nut hard limit/1','Nut stop/2');
sumblock(s,'Series spring travel','+-');wire(s,'Nut coordinates/1','Series spring travel/1');wire(s,'Actuator travel/1','Series spring travel/2');absblock(s,'Absolute spring travel');wire(s,'Series spring travel/1','Absolute spring travel/1');constant(s,'Spring working limit','p.springTravelLimit');compare(s,'Spring stop','>');wire(s,'Absolute spring travel/1','Spring stop/1');wire(s,'Spring working limit/1','Spring stop/2');
constant(s,'Zero','0');compare(s,'Whole pad contact loss','<=');wire(s,'Pad diagnostics/1','Whole pad contact loss/1');wire(s,'Zero/1','Whole pad contact loss/2');
constant(s,'Pad compression working limit','p.padCompressionLimit');compare(s,'Pad compression stop','>');wire(s,'Pad diagnostics/5','Pad compression stop/1');wire(s,'Pad compression working limit/1','Pad compression stop/2');
logic(s,'SEA working stops','OR',2);wire(s,'Nut stop/1','SEA working stops/1');wire(s,'Spring stop/1','SEA working stops/2');constant(s,'SEA enabled','p.sea');logic(s,'SEA only','AND',2);wire(s,'SEA working stops/1','SEA only/1');wire(s,'SEA enabled/1','SEA only/2');
logic(s,'Any physical failure','OR',4);wire(s,'Flange stop/1','Any physical failure/1');wire(s,'SEA only/1','Any physical failure/2');wire(s,'Whole pad contact loss/1','Any physical failure/3');wire(s,'Pad compression stop/1','Any physical failure/4');constant(s,'Stops enabled','p.enableStops');logic(s,'Stop gate','AND',2);wire(s,'Any physical failure/1','Stop gate/1');wire(s,'Stops enabled/1','Stop gate/2');outp(s,'Stop',1);wire(s,'Stop gate/1','Stop/1');
end

function ss(mdl,name,pos)
add_block('simulink/Ports & Subsystems/Subsystem',[mdl '/' name],'Position',pos);
delete_block([mdl '/' name '/In1']);delete_block([mdl '/' name '/Out1']);
end
function inp(s,name,n),add_block('simulink/Sources/In1',[s '/' name],'Port',num2str(n),'Position',nextpos());end
function outp(s,name,n),add_block('simulink/Sinks/Out1',[s '/' name],'Port',num2str(n),'Position',nextpos());end
function constant(s,name,value),add_block('simulink/Sources/Constant',[s '/' name],'Value',value,'Position',nextpos());end
function gain(s,name,value,multiplication)
if nargin<4,multiplication='Element-wise(K.*u)';end
add_block('simulink/Math Operations/Gain',[s '/' name],'Gain',value,'Multiplication',multiplication,'Position',nextpos());
end
function sumblock(s,name,inputs),add_block('simulink/Math Operations/Sum',[s '/' name],'Inputs',inputs,'Position',nextpos());end
function product(s,name,inputs),add_block('simulink/Math Operations/Product',[s '/' name],'Inputs',inputs,'Position',nextpos());end
function demux(s,name,outputs),add_block('simulink/Signal Routing/Demux',[s '/' name],'Outputs',outputs,'Position',nextpos());end
function mux(s,name,inputs),add_block('simulink/Signal Routing/Mux',[s '/' name],'Inputs',num2str(inputs),'Position',nextpos());end
function sat(s,name,lo,hi),add_block('simulink/Discontinuities/Saturation',[s '/' name],'LowerLimit',lo,'UpperLimit',hi,'Position',nextpos());end
function integrator(s,name,initial),add_block('simulink/Continuous/Integrator',[s '/' name],'InitialCondition',initial,'Position',nextpos());end
function trig(s,name,op),add_block('simulink/Math Operations/Trigonometric Function',[s '/' name],'Operator',op,'Position',nextpos());end
function absblock(s,name),add_block('simulink/Math Operations/Abs',[s '/' name],'Position',nextpos());end
function compare(s,name,op),add_block('simulink/Logic and Bit Operations/Relational Operator',[s '/' name],'Operator',op,'Position',nextpos());end
function logic(s,name,op,inputs),add_block('simulink/Logic and Bit Operations/Logical Operator',[s '/' name],'Operator',op,'Inputs',num2str(inputs),'Position',nextpos());end
function selector(s,name)
add_block('simulink/Signal Routing/Switch',[s '/' name],'Criteria','u2 >= Threshold','Threshold','0.5','Position',nextpos());
constant(s,[name ' choice'],'p.sea');wire(s,[name ' choice/1'],[name '/2']);
end
function rigidselector(s,name)
add_block('simulink/Signal Routing/Switch',[s '/' name],'Criteria','u2 >= Threshold','Threshold','0.5','Position',nextpos());
constant(s,[name ' choice'],'p.rigidPoint');wire(s,[name ' choice/1'],[name '/2']);
end
function wire(s,from,to),add_line(s,from,to,'autorouting','on');end
function p=nextpos()
persistent n;if isempty(n),n=0;end;n=n+1;x=45+mod(n-1,8)*150;y=45+floor((n-1)/8)*90;p=[x y x+100 y+40];
end
