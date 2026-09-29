function acf_sfun(block)
%ACF_SFUN Simulink adapter. Physics/controller functions remain plain MATLAB.

setup(block);

end

function setup(block)

% Dialog parameters: role (character vector), prepared parameter structure p.
block.NumDialogPrms = 2;
block.DialogPrmsTunable = {'Nontunable','Nontunable'};

role = block.DialogPrm(1).Data;
p = block.DialogPrm(2).Data;

switch role
    case 'surface'
        ins = [];
        outs = [3 1];

    case 'plant'
        ins = [1 2 3];
        outs = [1 1 25];

    case 'recenter'
        ins = [1 1];
        outs = 1;

    case 'robot'
        ins = 1;
        outs = 2;

    otherwise
        ins = 1;
        outs = 1;
end

block.NumInputPorts = numel(ins);
block.NumOutputPorts = numel(outs);

block.SetPreCompPortInfoToDefaults;

for j = 1:numel(ins)
    block.InputPort(j).Dimensions = ins(j);
    block.InputPort(j).DatatypeID = 0;
    block.InputPort(j).Complexity = 'Real';
    block.InputPort(j).SamplingMode = 'Sample';
    block.InputPort(j).DirectFeedthrough = true;
end

if strcmp(role,'plant') || strcmp(role,'robot')
    block.InputPort(1).DirectFeedthrough = false;
end

for j = 1:numel(outs)
    block.OutputPort(j).Dimensions = outs(j);
    block.OutputPort(j).DatatypeID = 0;
    block.OutputPort(j).Complexity = 'Real';
    block.OutputPort(j).SamplingMode = 'Sample';
end

block.SampleTimes = [p.sim.step 0];
block.NumContStates = 0;
block.SimStateCompliance = 'DefaultSimState';
block.SetAccelRunOnTLC(false);

block.RegBlockMethod('PostPropagationSetup', @allocate);
block.RegBlockMethod('InitializeConditions', @initialize);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Update', @update);

end

function allocate(block)
x=acf_initial(block.DialogPrm(1).Data,block.DialogPrm(2).Data);
block.NumDworks=1;
block.Dwork(1).Name='state';
block.Dwork(1).Dimensions=numel(x);
block.Dwork(1).DatatypeID=0;
block.Dwork(1).Complexity='Real';
block.Dwork(1).UsedAsDiscState=true;
end

function initialize(block)
block.Dwork(1).Data=acf_initial(block.DialogPrm(1).Data,block.DialogPrm(2).Data);
end

function outputs(block)
role=block.DialogPrm(1).Data; p=block.DialogPrm(2).Data;
x=block.Dwork(1).Data; t=block.CurrentTime;
switch role
    case 'surface'
        [y,v]=acf_surface(t,p);
        block.OutputPort(1).Data=y; block.OutputPort(2).Data=v;
    case 'plant'
        % Do not read port 1 here: a new command cannot instantly move a mass.
        y=acf_observe(x,block.InputPort(2).Data,block.InputPort(3).Data,p);
        block.OutputPort(1).Data=y(1);
        block.OutputPort(2).Data=y(2);
        block.OutputPort(3).Data=y;
    case 'robot'
        block.OutputPort(1).Data=x;
    case 'force'
        block.OutputPort(1).Data=acf_controller(t,x,block.InputPort(1).Data,p);
    case 'sensor'
        block.OutputPort(1).Data=acf_sensor(t,x,block.InputPort(1).Data,p);
    case 'recenter'
        block.OutputPort(1).Data=acf_recenter(t,x,block.InputPort(1).Data,block.InputPort(2).Data,p);
    case 'delay'
        block.OutputPort(1).Data=acf_delay(t,x,block.InputPort(1).Data,p);
end
end

function update(block)
role=block.DialogPrm(1).Data; p=block.DialogPrm(2).Data;
x=block.Dwork(1).Data; t=block.CurrentTime;
switch role
    case 'plant'
        xn=acf_plant_step(x,block.InputPort(1).Data,block.InputPort(2).Data,block.InputPort(3).Data,p);
    case 'robot'
        xn=acf_robot_step(x,block.InputPort(1).Data,p);
    case 'force'
        [~,xn]=acf_controller(t,x,block.InputPort(1).Data,p);
    case 'sensor'
        [~,xn]=acf_sensor(t,x,block.InputPort(1).Data,p);
    case 'recenter'
        [~,xn]=acf_recenter(t,x,block.InputPort(1).Data,block.InputPort(2).Data,p);
    case 'delay'
        [~,xn]=acf_delay(t,x,block.InputPort(1).Data,p);
    otherwise
        xn=x;
end
block.Dwork(1).Data=xn;
end
