# Active Contact Flange — Simulink model package

Unzip this folder, make it your **MATLAB Current Folder**, then run:

```matlab
acf_start
```

This builds and opens `generated/acf_force_control.slx`, runs the leadscrew
bump example, and plots the results. Existing models are preserved by giving
new builds a numeric suffix. Keep the `.m` files available on the MATLAB path
when reopening the generated model. `acf_start` adds this folder automatically.

**Needs MATLAB and Simulink.** Simscape, Stateflow and a C compiler are not
needed. Use normal simulation mode; this is a design simulation, not deployable
real-time controller code.

The package contains the model builder and its source files. **No precompiled
SLX is included because MATLAB/Simulink was unavailable during authoring.**
MATLAB source was checked with MISS_HIT. Equations and numerical behaviour were
checked in an independent Python implementation; see `VALIDATION.md`. The
builder compiles the diagram in your Simulink before saving it.

## What is included

- Fast sampled force PI, with output limits and anti-windup.
- Leadscrew, voice-coil and ideal-position actuator options.
- The R–A–B–S mechanism: upper and lower springs/dampers, separate moving
  masses, gravity and preload, with equal/opposite actuator reactions.
- **Rigid contact between S and the workpiece. No contact spring or damper.**
- A slower mid-stroke recentering loop requesting robot motion.
- Robot command delay, finite velocity response, speed and acceleration limits.
- Force sampling, filtering, delay, bias and repeatable noise.
- Smooth bumps, sharp finite-width bumps, ramps, repeated bumps, and robot
  positioning disturbances.
- True-force tolerance checks, motion/effort logs, plots and parameter sweeps.

The actuator and mechanics are integrated together inside one subsystem. This
preserves their load coupling. Their equations are in ordinary, editable
MATLAB files rather than an unexplained transfer function.

## Change a scenario or actuator

```matlab
[r, mdl, p] = acf_start('leadscrew', 'bump');

% Reuse this model with different parameters:
p.surface.width = 0.01;      % 10 mm along the scan direction
p.surface.height = 0.001;    % 1 mm high
p.surface.scanSpeed = 0.1;   % 100 mm/s scanning speed
p.robot.delay = 0.15;        % 150 ms robot command delay
r = acf_run(mdl, p);

% Swap the actuator and use its corresponding controller tuning:
p = acf_parameters('voice_coil', 'sharp_bump');
r = acf_run(mdl, p);
```

Actuators: `leadscrew`, `voice_coil`, `ideal_position`.

Scenarios: `flat`, `bump`, `sharp_bump`, `ramp`, `repeated`, `robot_error`.

Use `acf_parameters(kind,scenario)` when selecting a named preset: it sets both
the shape and its example dimensions. All masses, stiffnesses, motor constants,
limits and controller gains in this file are **illustrative assumptions**.

## Most useful parameters

| Parameter | Meaning |
|---|---|
| `force.reference`, `force.tolerance` | Desired contact force and allowed error, N |
| `force.sampleTime` | Force-controller period, s |
| `surface.height`, `surface.width`, `surface.scanSpeed` | Bump/ramp geometry and scanning speed |
| `robot.initialExcessForce` | Extra static force if the flange were locked at mid-stroke, N |
| `robot.delay`, `robot.maxAcceleration`, `robot.maxSpeed` | Coarse robot response limits |
| `robot.recenterGain`, `robot.deadband` | How strongly the flange asks to recenter |
| `mechanics.travelMin`, `travelMax`, `midTravel` | Travel limits and preferred location, m |
| `mechanics.mA`, `mB`, `mS`, `k1`, `k2`, `c1`, `c2` | Masses and elastic links |
| `screw.lead`, `rotorInertia`, `peakCurrent`, `voltage` | Leadscrew transmission and motor constraints |
| `coil.forceConstant`, `peakCurrent`, `voltage` | Coil force and drive constraints |
| `ideal.maxAcceleration`, `ideal.maxAxialForce` | Generic sizing limits for the ideal-position variant |
| `sensor.bias`, `noiseRms`, `delay`, `filterTime` | Measurement imperfections |
| `sim.step` | Numerical integration step, not the controller period |

Default controller/sensor sampling is 1 kHz, robot commands 100 Hz, and
integration 5 kHz. To investigate 100 Hz force control, set
`p.force.sampleTime = 0.01`; change the sensor period too if appropriate.
Controller update rate and closed-loop bandwidth are different quantities.
Periods must be integer multiples of `sim.step`, and delays integer multiples
of their respective sampling periods. Halve `sim.step` to check convergence
before treating a near-boundary result as a pass.

## Find required capability

Start with a chosen scenario envelope and controller tuning. Then sweep limits.
For a generic force/acceleration budget:

```matlab
cases = {acf_parameters('ideal_position','bump'), ...
         acf_parameters('ideal_position','ramp')};
[T, detail] = acf_sweep(cases, ...
    'ideal.maxAxialForce', [13 15 20 30], ...
    'ideal.maxAcceleration', [0.05 0.1 0.5 2]);
T(T.allPassed,:)
writetable(T, 'capability_sweep.csv');
```

For an actual leadscrew model:

```matlab
cases = {acf_parameters('leadscrew','bump'), ...
         acf_parameters('leadscrew','sharp_bump')};
[T, detail] = acf_sweep(cases, ...
    'screw.peakCurrent', [0.3 0.5 1 2 3], ...
    'robot.delay', [0 0.08 0.2]);
```

`allPassed` means every supplied case passed. It identifies **tested passing
configurations**, not a universal minimum. Gains remain fixed during a sweep;
a force error can come from controller lag even when the actuator has ample
force. Look at saturation fractions and motion traces before buying a larger
actuator. Faster gains also need stability checks.

For the ideal variant, acceleration limits shape the requested motion; force
limits then determine whether that acceleration is physically achievable.
An external load can drive actual acceleration beyond the requested limit if
the actuator cannot oppose it. The default ideal force budget is unlimited.

## Results and verification

`r.time`, `r.data`, `r.names` contain all logged signals; `r.metrics` contains
peak/RMS force error, travel, speed, acceleration, axial force, current and
motor torque, plus saturation and contact flags. `acf_signals.m` defines units.
The initial state is a loaded equilibrium, so evaluation starts at time zero.
`finalRecoveryTime_s` is an absolute simulation time;
`recoveryAfterDisturbance_s` measures recovery after the last surface event
ends. `NaN` means recovery was not observed or the event had not finished.

```matlab
r.metrics
signals = array2table([r.time,r.data], ...
    'VariableNames',[{'time_s'},r.names]);
writetable(signals,'acf_signals.csv');
acf_selftest(mdl,p)   % physics checks and a short Simulink/reference comparison
```

Loss of contact, an impact, a stroke stop, or overcurrent fails the run even if
the sampled finite force otherwise looks acceptable. Ideal rigid impacts have
an impulse, not a finite force peak: `peakForceIncludingImpacts_N` is `Inf` if
one occurs. The model therefore cannot certify an impact force waveform to
±0.5 N without adding real tool/workpiece compliance in a later model.

RMS current/torque are thermal-sizing inputs, not continuous ratings. No motor
thermal model, backlash, screw self-locking, robot structural modes, or raw
load-cell placement model is included. The sensor block represents an already
compensated contact-force estimate with residual error. See `MODEL_NOTES.md`
for the exact mechanics and numerical assumptions.

You can also run `acf_reference(p)` in MATLAB without Simulink; it calls the
same component functions and returns the same result structure.
