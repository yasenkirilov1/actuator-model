# Open, run and change the contact-flange model

## Open Simulink

In MATLAB, set **Current Folder** to the `actuator-model` project folder. Paste these two lines into the **Command Window**, then press Enter:

```matlab
addpath('acf_simulink')
mdl = acf_open;
```

This opens `acf_simulink/generated/acf_force_control_simple.slx`. You can also double-click that SLX in MATLAB's file browser. Typing `simulink` opens the general Simulink start page; `acf_open` opens this particular model directly.

Double-click a subsystem to see its blocks. Use the breadcrumb/back button to return. The root diagram exposes the actuator/controller selectors, requested force, surface/robot inputs and feedback loops. Simulink's light appearance is intentional; MATLAB can remain dark.

`acf_open` preserves a model already loaded in your desktop. If you have an older unsaved copy open, save those edits or save a separate copy, then close it and call `acf_open` again to load the current disk version.

## First run

For the selected sharp-bump configuration:

```matlab
p = acf_parameters('voice_coil','sharp_bump','recommended');
p.mechanics.mS = 1.4;             % kg of tool beyond the load cell
p.force.reference = 20;          % N normal pressing force
p.mechanics.angle = deg2rad(0);  % angle from upward vertical
p.surface.scanSpeed = 0.100;     % m/s along the surface
p.sim.stopTime = 4;              % s
r = acf_run(mdl,p);
disp(r.metrics)
acf_visualize(r)
```

`r` contains recorded signals, the parameters used and the pass/fail result. The force metric uses every native sample, including startup. The HTML replay shows the recorded motion; its playback controls do not rerun the model. A failed run can mean excess force error, contact loss, a travel stop, spring packing failure or a drive limit; read `r.metrics`.

The green **Run** button in Simulink also runs the blocks, using the Model Workspace parameters. `acf_run(mdl)` additionally returns logs and opens MATLAB result plots.

## A slow bump with a 2 kg toolhead

Here **2 kg means B+S**, as requested. With B = 0.6 kg, S is 1.4 kg. A is additional. Open and run the whole [acf_experiment.m](acf_experiment.m) file, or enter:

```matlab
p = acf_parameters('voice_coil','bump','recommended');
p.mechanics.mS = 2 - p.mechanics.mB;
p.surface.height = 0.001;       % 1 mm
p.surface.width = 0.060;        % 60 mm
p.surface.scanSpeed = 0.020;    % 20 mm/s: three seconds across
p.surface.startTime = 2;
p.force.reference = 20;
p.sim.stopTime = 8;
r = acf_run(mdl,p);
acf_visualize(r)
```

Changing `mechanics.mS` updates the known downstream mass used by reconstruction. Changing `surface.scanSpeed` updates the commanded speed used by the motion observer. Deliberate identification errors use separate calibration fields, rather than stale copies of those parameters.

## Change parameters

`p` is a structure: dots select a group and a setting. Values use **SI units**.

| Setting | Meaning |
|---|---|
| `p.force.reference` | Desired surface-normal force, N |
| `p.force.tolerance` | Allowed absolute force error, N |
| `p.mechanics.mB`, `mS` | Holder and downstream tool mass, kg |
| `p.mechanics.angle` | Flange-axis angle from upward vertical, radians |
| `p.surface.height`, `width` | Bump height and scan-direction width, m |
| `p.surface.scanSpeed` | Tangential scan speed, m/s |
| `p.surface.startTime`, `repeatPeriod` | First bump and repeated-bump interval, s |
| `p.contact.mu` | Physical sanding drag ratio for the experiment |
| `p.mechanics.k1`, `c1` | Lower net pair stiffness, N/m, and damper, N·s/m |
| `p.springs.lower.rate_N_m` | Rate of one spring; must equal `k1/2` |
| `p.springs.commonPreload_N` | Preload of each opposed spring, N |
| `p.coil.forceConstant`, `resistance`, `inductance` | N/A, Ω and H |
| `p.coil.peakCurrent`, `voltage`, `currentResponse` | A, usable bipolar V, and response target, s |
| `p.sensor.noiseRms`, `strokeNoiseRms` | Conditioned force/position noise, N and m RMS |
| `p.accel.noiseRms` | Conditioned acceleration noise, m/s² RMS |
| `p.sensor.delay` | Common pipeline delay, s |
| `p.robot.delay`, `responseTime` | Actual robot delay and response, s |
| `p.robotPrediction.delay`, `responseTime` | Identified robot predictor values, s |
| `p.sim.stopTime` | Recording duration, s |

Read the current settings, edit and apply them:

```matlab
p = acf_parameters;
p.force.reference = 50;
p.mechanics.angle = deg2rad(90);
r = acf_run(mdl,p);
```

A MATLAB variable `p` and the model's saved `p` are separate copies until you apply the variable with `acf_run`. To edit in the UI, open **Model Explorer**, select this model's **Model Workspace**, open `p`, edit its fields and press Run. To apply an edited copy without running:

```matlab
mw = get_param(mdl,'ModelWorkspace');
assignin(mw,'p',p);
acf_select_actuator(mdl,p.actuator.kind);
```

The preparation helper checks consistency and recalculates `p.d`. Edit ordinary parameters rather than derived fields under `p.d`. **Ctrl+S** saves parameters with the SLX; **Save As** preserves an alternative diagram.

## Choose and tune a controller

Use the root **Force controller selection** dropdown, or:

```matlab
acf_select_controller(mdl,'Feedforward PI');
p = acf_parameters;
r = acf_run(mdl,p);
```

Choose the strategy first, then read/edit `p`; passing an older parameter copy to `acf_run` would apply that older choice again.

| Choice | Behavior |
|---|---|
| **Feedforward PI** | Selected PI plus measured acceleration, geometry, guide friction and causal lower-link compensation |
| **PI** | Reconstructed normal-force feedback with known static holding current |
| **PID** | Same feedback with a filtered derivative term |

All choices use the same sensor reconstruction. The selected coil gains are:

```matlab
p.inertial.forceKp = 0.3;          % A/N
p.inertial.forceKi = 20;           % A/(N*s)
p.inertial.forceKd = 0;            % A*s/N
p.inertial.derivativeFilter = 0.001;
```

For PID, choose `'PID'` and set a nonzero `p.inertial.forceKd`. Its derivative can amplify noise; a new gain is an experiment requiring another check. The leadscrew current controller uses `p.screw.currentForceKp`, `currentForceKi` and `currentForceKd`. Gains are actuator-specific. `coil.forceKp` and the old nine-state observer weights are obsolete and do not tune this controller.

## Choose an actuator or mechanical alternative

```matlab
acf_select_actuator(mdl,'Leadscrew');
p = acf_parameters;
r = acf_run(mdl,p);
```

Switching between coil and screw preserves the mechanics and uses the matching native current controller. Read the report before treating the small screw motor as a continuous 50 N design.

This retained leadscrew branch ties screw-nut motion to B. It is not the proposed
**series-elastic leadscrew**, which places a separate spring between the nut and
B and controls spring compression through screw position/velocity. Historical
comparisons already had a compliant B–S link, but used different mechanics and
current control. They do not rank that new spring-based actuator against a voice
coil with the prescribed compliant upper attachment. See the
[series-spring comparison scope](experiments/series_elastic_comparison_20261006/README.md).
No runnable series-elastic branch or new dynamic qualification is implied by
these static calculations.

To restore the selected mechanics, or compare rigid replacement couplers:

```matlab
p = acf_parameters('voice_coil','sharp_bump','recommended');
r = acf_run(mdl,p);

p = acf_parameters('voice_coil','sharp_bump','no_springs');
r = acf_run(mdl,p);
```

The no-springs preset retains the load cell's finite stiffness. Setting a spring rate to zero would leave a disconnected/free link and is rejected. For a zero-damper experiment, set `p.mechanics.c1 = 0`; actuator friction still exists. When changing a pair's stiffness, also set its individual spring rate to half the net value and reassess geometry/preload. Alternatives need separate controller tuning and are not qualified by the selected design's result.

## Configure the representative real coil

The broad qualification grid uses the generic 20 N/A winding. To configure
the separately checked Moticont-class 24.4 N/A winding:

```matlab
p = acf_parameters('voice_coil','sharp_bump','recommended');
p.mechanics.mA = 1.5;             % robot-side stationary assembly budget
p.coil.forceConstant = 24.4;      % N/A
p.coil.resistance = 6.4;          % ohm, cold reference
p.coil.inductance = 0.004;        % H
p.coil.voltage = 32;              % usable output; verify driver dropout at 36 V
p.coil.currentResponse = 100e-6;  % measured-response target
p.inertial.forceKp = 0.3*20/24.4;
p.inertial.forceKi = 20*20/24.4;
r = acf_run(mdl,p);
```

This explicitly preserves the force-domain PI gains. Preparation updates
identified Kf, holding current and inverse coefficients from the known
winding and calibration scales. The usable voltage and hot-load response
remain bench requirements; this configuration is a bounded real-coil study,
not qualification of every mass/force/orientation combination. Include the
moving coil in B and all tool-side fittings in S. See the report for hot
resistance, continuous force, driver cooling and sensor calibration.

## Run faster and interpret the result

For repeated runs of the same topology, choose **Accelerator** in the Simulink toolbar’s **Normal** dropdown, or enter:

```matlab
set_param(mdl,'SimulationMode','accelerator');
r = acf_run(mdl,p);
```

The first run compiles; subsequent compatible runs reuse that work. The retained runner comparison was about 4.8 times faster in warm Accelerator mode and bitwise equal to Normal mode on its recorded cases. Runtime depends on logging, edits and machine load. Keep the 10 µs solver step to resolve the 100 kHz acquisition; a larger step changes or fails that model.

Every case begins in established loaded contact at its requested force and orientation. These are tracking tests, not simulations of initial contact acquisition. Contact loss and travel stops terminate the run and mark it failed. [The report](../reports/active-contact-flange-report.md) separates nominal tracking, calibration tolerance and physical validation.
