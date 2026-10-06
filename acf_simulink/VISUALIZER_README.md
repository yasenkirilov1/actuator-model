# Recorded motion and offline HTML

Run the native model, then export its recorded motion:

```matlab
addpath('acf_simulink')
mdl = acf_open;
p = acf_parameters('voice_coil','sharp_bump','recommended');
r = acf_run(mdl,p);
acf_visualize(r)
```

The generated HTML is self-contained and works offline. MATLAB opens it in
the normal browser. To export without opening a browser:

```matlab
file = acf_visualize(r,'my_replay.html',false);
```

Default filenames under `generated/` are unique. An explicit filename replaces
that file. To include several already-completed runs:

```matlab
acf_visualize({r1,r2,r3})
```

Changing playback or selecting a recording never runs the model again.

## Force and motion

The force plot/readout uses the **true surface-normal pressing force** and
its recorded native estimate. The optional cell overlay is the **tared axial
B–S cell channel**, before inertia correction. Its difference from pressing
force depends on downstream mass, acceleration and surface slope.

The schematic shows R–A–B–S axial motion, the actuator, the attachments and
relative travel. The selected upper link is rigid; the lower link contains
an opposed spring pair and damper. Tool position sensing measures S relative
to A; actuator travel is A–B motion. This is a schematic, not a CAD assembly,
a pad-pressure simulation or a model of lateral guide deflection.

## Controls

- Play/pause, speed and timeline replay recorded samples.
- Click a plot to seek. Worst error jumps to the largest original force error.
- Next event visits entries into band/drive limits, contact loss and travel stops.
- Run selector switches between supplied recordings.
- Cell reading and Estimated contact toggle the sensor/estimate overlays.
- Load results reads another exported replay locally; imported scripts do not run.

RUN PASSES / RUN FAILS describes the complete input recording, including
startup and guards. IN BAND / OUTSIDE BAND describes the displayed frame.
Heating is a separate design calculation. Contact-loss and travel-stop runs
terminate in the selected native model; historical impact recordings are
still readable.

## Long recordings

For up to20000 samples every sample is exported. Longer runs export a smaller
set of original samples: per-bin force/estimate/travel/current/voltage and
spring-motion extrema, endpoints and discrete-event transitions. The largest
force error and individual contact/stop/limit events are retained. The footer
states the original count and display policy.

Pass/fail, peak error, time outside tolerance and spring packing come from
**all original samples before display reduction**. They are not recalculated
from the sparse display. The full native data remain in `r`; save them with
`save('native_run.mat','r','-v7.3')` for later analysis. Replay does not interpolate
contact/stop states or invent simulations between frames.

The comprehensive [Markdown report](../reports/active-contact-flange-report.md)
explains the selected mechanics, controller, accuracy requirements and limits.
Its corresponding offline HTML report includes source inspection and parameter
filters. Report filters and replay controls display recorded evidence.

## Export checks

`acf_viewer_selftest` checks a short run, signal reordering, label escaping,
non-increasing timestamps and several recordings while restoring the model's
settings. `experiments/validate_public_force_helpers.m` checks native force
metrics and a long-history display reduction against retained data, including
single-sample synthetic events. Those software fixtures are not force experiments.
Use the current execution record in `validation/` for the tested scope.
