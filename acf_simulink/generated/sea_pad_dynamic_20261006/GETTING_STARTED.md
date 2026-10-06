# Open the native pad/SEA research plant

This model shows the physical plant with ordinary Simulink blocks. Its current
requests are a recorded replay from the separately tested controller. Editing
the mechanics while replaying the same requests does not retune the controller.
The canonical full-control model has not been replaced.

The saved default is a voice-coil replay with a **77 mm diameter, 5 mm-thick
pad**, assumed total pad stiffness 50 kN/m and zero pad damping. Its target
is 50 N, tool mass S is 1.4 kg (B+S is 2 kg), and orientation is 90°.
Its recorded native peak force error is 0.046075 N. These assumed pad
properties require measurement on the real pad; geometry alone does not
establish its stiffness.

In the MATLAB **Command Window**, paste:

```matlab
cd('/home/tommie/Projects/actuator-model')
modelFile = fullfile(pwd,'acf_simulink','generated', ...
    'sea_pad_dynamic_20261006','acf_sea_pad_research_20261006.slx');
load_system(modelFile)
mdl = 'acf_sea_pad_research_20261006';
mw = get_param(mdl,'ModelWorkspace');
replayInputs = evalin(mw,'replayInputs');
open_system(mdl)
```

The `replayInputs` assignment brings the saved current/robot input waveform
into the MATLAB workspace, where the model's external-input loader resolves
it. The structure `p` remains in the **Model Workspace**. View it with:

```matlab
p = evalin(mw,'p');
p.sea          % 0 = voice coil; 1 = screw driving an internal series spring
p.Fref         % loaded equilibrium force for this recorded example
p.mS           % tool mass beyond the sensing cell
p.kPad         % total pad spring stiffness, N/m
p.cPad         % total pad damping, N*s/m
p.padDiameter  % metres
```

Use the diagram's six subsystems to inspect the electrical drive, independent
screw nut/spring, A/B/S dynamics, local pad contact, stops and recordings.
The [vector diagram](native-block-diagram.svg) gives the same root layout.
All forces shown in the scored output are total axial contact forces; local
pressure is outside this research question.

Running this loaded example produces a two-second physical replay. The
properly isolated reproducibility helpers are described in the
[native study README](../../experiments/sea_pad_dynamic_20261006/native/README.md).
They score every native step in memory and clean their own SDI/build storage.
Use those helpers for experimental reruns so ordinary desktop logging does
not create large files. The source-bound results are
[in the native validation folder](../../validation/sea_pad_dynamic_20261006/native/).

Changes to actuator type require preparing its resistance, inductance, lead,
equilibrium and mass matrix together using `sea_pad_native_params` or
`sea_pad_native_from_engine`; setting only `p.sea` is insufficient. New
closed-loop comparisons require new controller command histories from the
study engine. Retained point/partial-contact cases include explicit failed
parity and numerical-convergence checks; consult the README before making
performance claims.

`pilot_frozen/` and `partial_frozen/` preserve earlier executed model versions.
They are research evidence and are not the beginner example's current default.
