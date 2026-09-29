# ACF 2D replay add-on

The viewer animates recorded results from the existing model. MATLAB exports
one self-contained HTML file and opens it in your normal browser. It works
offline; no server, account, browser extension or extra MATLAB toolbox is used.

## Install and run

If using the standalone add-on ZIP, extract its contents **into the same folder
as `acf_start.m`**. Keep the `viewer` subfolder. The included `acf_plot.m` adds a
**2D replay** menu to the existing plot window. If using the updated complete
model ZIP, everything is already included.

Make that model folder your MATLAB Current Folder and run:

```matlab
acf_start_visualizer
```

This builds/runs the **voice-coil bump** scenario and opens its replay. There is
no need to run the helper functions individually.

Already have a result?

```matlab
acf_visualize(r)
```

After `acf_start` with no output arguments, `acf_visualize` with no arguments
opens the result stored in `acfResult`. You can also choose **2D replay** from
the normal result plot's menu.

## Several scenarios in one viewer

```matlab
[runs, htmlFile, mdl] = acf_start_visualizer('voice_coil', ...
    {'bump','sharp_bump','ramp'});
```

The run selector switches between these completed simulations. Selecting a
run does **not** rerun or retune the controller. The selector labels can be
customized with `r.label`, useful for parameter-sweep candidates.

```matlab
p = acf_parameters('voice_coil','sharp_bump');
p.coil.peakCurrent = 2;
p.coil.voltage = 24;
r = acf_run(mdl,p,false);
r.label = 'Sharp bump / 2 A / 24 V';
acf_visualize(r)

% Put any collected runs into the same replay:
acf_visualize({r1,r2,r3})
```

Sweep summary tables `T` and `D` contain metrics, not motion histories. Rerun
the selected candidate with `acf_run`, using the same scenario and parameter
values, then pass its result to the viewer.

## Controls

- **Play / pause**, playback speed, and the timeline replay the recorded run.
- **Previous / next** move by approximately 1/60 s, selecting recorded samples.
- **Space** toggles playback; **left / right arrows** step when focus is outside
  a form control. Playback pauses when the browser tab is hidden.
- Click either trace to seek to that time.
- **Worst error** jumps to the largest absolute true-force error in the
  evaluation interval.
- **Next event** jumps through entries into force/drive limits, contact loss,
  rigid impacts and stroke-stop flags. It wraps after the last event.
- **Measured** adds the measured-force error trace for sensor/feedback checks.
- **Load results** opens another exported replay HTML or an `acf-replay-v1`
  JSON bundle locally. Imported HTML is parsed for data; its scripts do not run.

**RUN PASSES / RUN FAILS** describes the complete evaluation interval.
**IN BAND / OUTSIDE BAND** describes the displayed frame. Heating is not
included in these checks.

## What the drawing means

The existing R–A–B–S topology is retained: robot boundary R, upper compliant
link, actuator between A and B, lower compliant link, and tool S against a
rigid workpiece. There is no contact spring or damper. A and B use their own
logged positions. The guides and stops belong to A; the stroke marker follows B.

The scene is a **2D schematic of the 1D axial model**, with magnified movement.
It does not add tilt, lateral forces, surface-normal rotation or new dynamics.
The terrain is drawn from the logged surface-height history and scan speed.
Outside the recorded time window, the endpoint height is held for display.

The force arrows show the logged transmitted axial force `Fa` on A/B, with
opposite directions, and the workpiece contact force `Fc` on S. Arrow direction
follows the sign; arrow lengths are schematic. The tool moves separately from
the surface after contact loss. A rigid impact displays its impulse; it has no
finite peak contact force in this model.

The viewer supports `voice_coil`, `leadscrew` and `ideal_position`. For the
ideal-position source, the electrical readouts become axial force and gap
acceleration. The screw graphic indicates relative motion, not a mechanical
CAD section or a detailed winding/screw design.

The viewer uses **actual minus requested** force as the plotted signed error.
The existing force controller uses the opposite sign. Both have the same
absolute ±0.5 N requirement.

## Export, sharing and compatibility

```matlab
file = acf_visualize(r,'my_replay.html',false);
```

The final argument disables automatic browser opening. Double-click the HTML
or share that one file; the recipient does not need MATLAB to replay it.
For MATLAB Online, download the generated HTML and open it locally.

Default exports are saved in `generated/` with unique names. An explicit output
filename replaces that file. All logged samples and numeric values are exported.
The curves are drawn with an extrema-preserving display reduction; sample
selection, event detection and metrics use the full exported record. Contact
and impact flags are never interpolated. All signals are matched by name.

Use a current Chrome, Edge, Firefox or Safari. The MATLAB exporter uses standard
`jsonencode` and `web(...,'-browser')`. The complete package still requires
MATLAB and Simulink to build/run the native model. To use the existing plain
MATLAB reference implementation instead:

```matlab
acf_start_visualizer('voice_coil','bump','reference')
```

## Validation

The shipped replay script passed 31 local logic checks using the independent
Python reference model, including a smooth bump, sharp bump, ramp and a stress
case with contact loss and rigid impacts. The checks exercise playback,
seeking, separate body positions, run selection, event jumps, sensor overlay
and data validation in an in-memory DOM. MATLAB source is checked with
MISS_HIT. See `viewer/validation/VALIDATION.md` for actual results.

These checks do not replace a browser rendering test. Browser security policy
blocked opening the local HTML in the authoring environment, so real-browser
layout, file-chooser import and mobile rendering remain unverified. A
Playwright test script is supplied for a normal local development environment.

MATLAB/Simulink is unavailable in the authoring environment, so the new MATLAB
exporter and launcher have **not been executed there**. On your MATLAB, this
short export check requires no Simulink or browser:

```matlab
acf_viewer_selftest
```

Official API references:

- https://www.mathworks.com/help/matlab/ref/jsonencode.html
- https://www.mathworks.com/help/matlab/ref/web.html
