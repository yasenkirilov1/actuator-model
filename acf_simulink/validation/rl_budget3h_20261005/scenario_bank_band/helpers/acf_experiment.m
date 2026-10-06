%% A slow voice-coil bump with a 2 kg tool and holder (B + S)
% Edit this experiment, then run the WHOLE file in the MATLAB Editor.
% Equations and base settings come from the saved Simulink model.
addpath(fileparts(mfilename('fullpath')));
acfExperimentModel = acf_open;
p = acf_parameters('voice_coil','bump');

%% Experiment settings (SI units)
toolheadTotal_kg = 2;               % Combined holder B and tool S
p.mechanics.mS = toolheadTotal_kg - p.mechanics.mB;
% With the example 0.6 kg holder, S is 1.4 kg. Robot-side A is additional.
p.surface.height = 0.001;           % 1 mm high
p.surface.width = 0.060;            % 60 mm along the scan direction
p.surface.scanSpeed = 0.020;        % 20 mm/s: three seconds to cross
p.sim.stopTime = 8;                 % Observe the bump and recovery
p.force.reference = 20;             % N: desired contact force
p.force.tolerance = 0.5;            % N: allowed error on either side

%% Apply these settings to the existing blocks, run and plot
r = acf_run(acfExperimentModel,p);
p = r.parameters;
disp(r.metrics);
% Optional recorded-motion replay: acf_visualize(r)
