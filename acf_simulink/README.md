# Active contact flange — native Simulink model

The physical model is [acf_force_control_simple.slx](generated/acf_force_control_simple.slx). Its mechanics, sensors, controllers and robot response execute native Simulink blocks. Editable parameters `p` and named presets are saved in the **Model Workspace**. MATLAB helpers open the diagram, prepare coefficients, run it and inspect results. There is no separate MATLAB plant integrator.

Start with [GETTING_STARTED.md](GETTING_STARTED.md). The comprehensive [design report](../reports/active-contact-flange-report.md) explains the design choices, experiments, component targets and prototype limitations. [MODEL_NOTES.md](MODEL_NOTES.md) describes the equations; [REPRODUCING_RESULTS.md](REPRODUCING_RESULTS.md) describes the retained studies. MATLAB and Simulink R2026b were used; Simscape and Stateflow are unnecessary.

From the project folder in MATLAB:

```matlab
addpath('acf_simulink')
mdl = acf_open;
p = acf_parameters('voice_coil','sharp_bump','recommended');
p.sim.stopTime = 4;
r = acf_run(mdl,p);
acf_visualize(r)
```

The root **Actuator selection** dropdown chooses voice coil, leadscrew or the ideal-position reference. **Force controller selection** chooses **Feedforward PI**, **PI** or **PID**. The green Run button uses the Model Workspace. Calling `acf_run(mdl,p)` applies the supplied parameter copy, runs the same blocks and returns plots and pass/fail metrics.

## Design target

Regulate the **normal pressing force experienced by the workpiece** to within **±0.5 N throughout the recording**. The sharp test is **1 mm high × 10 mm wide at 100 mm/s**, with force targets **5–50 N**, fixed orientations **0–90°** and tool mass **S = 0.1–2 kg**. Holder B is provisionally 0.6 kg, so B+S spans 0.7–2.6 kg. A is additional robot-side structure.

The B–S multi-axis cell has all of S beyond it. A matched, synchronized accelerometer on S removes dynamic tool inertia from the cell reading. Position measures S relative to A. A two-state motion observer and a causal prediction from robot commands already issued estimate the local surface normal. No bump preview enters the controller.

## Selected design

| Item | Selected value |
|---|---|
| R–A attachment | Rigid |
| B–S compliant link | Opposed guided spring pair, net 19,746 N/m |
| Each spring | Approximately 9.873 N/mm, 20 mm free length |
| Common preload | 33.6 N per spring |
| Parallel damper | 60 N·s/m |
| Conservative combined free length | 40.98 mm including ±0.49 mm per spring |
| Force controller | Feedforward PI; P = 0.3 A/N, I = 20 A/(N·s), D = 0 |
| Feedback filter / lower-link inverse roll-off | 1 kHz / 1.5 kHz |
| Current-response target | 100 µs |
| Raw acquisition / control / robot command rates | 100 kHz / 10 kHz / 100 Hz |
| Analog response envelope / raw-clock EMA | Matched two-pole 5.1 kHz / 50 µs |
| Common pipeline provision | 20 µs |
| Conditioned force / acceleration / position noise | 0.02 N per axis / 0.014 m/s² / 1 µm RMS |

The compliant-link inverse is a proper causal filter made from native blocks; it compensates the holder/tool acceleration difference. The slow robot loop requires stable force, low force variation and a 100 ms dwell before sending new recentering commands. It also caps gain for the identified robot delay. Commands already queued still act after the gate closes.

The generic qualification drive is 20 N/A, 4 Ω, 3 mH, bipolar 3.5 A/24 V with 25 mm travel. For a realistic 24.4 N/A, 6.4 Ω, 4 mH coil, the report recommends a **36 V supply with at least 32 V usable bipolar output**, subject to hot winding, driver and continuous-force checks. Higher voltage alone does not improve an unclipped response. Catalog data and simulation passes do not establish prototype accuracy; inspect the report's calibration and mounting requirements before building.

## Recorded results and delivery

All **3,500 nominal sharp-bump cases passed**, covering the declared force, orientation, mass, drag and five clean/noisy conditions. The worst native error is **0.315811 N**. Four paired 30 s repeats with the selected tighter static calibration also pass, worst **0.415311 N**; three calibrated mechanical stress cases pass at **0.376938 N** worst. These are finite simulated cohorts. The tighter targets require assembled warm calibration and measured spring/damper identification; they are not catalog guarantees.

Open the [offline HTML report](../reports/active-contact-flange-report.html) for graphs, source inspection and the full searchable nominal grid. The [actual worst 2D replay](generated/active_contact_flange_inertial_worst_20260930.html) seeks its exact native peak. The [13-page native diagram](generated/active_flange_native_model_pages_20260930.pdf) makes the saved blocks inspectable without MATLAB. The [dated delivery ZIP](generated/active_contact_flange_inertial_20260930.zip) includes the model, guides, reports, selected native traces and reproducible frozen inputs; its manifest identifies the retained scope.

## Saved alternatives

| Preset | Purpose |
|---|---|
| `recommended` | Selected lower pair/damper, upper rigid, measured-inertia feedforward PI |
| `no_springs` | Rigid mechanical couplers with finite B–S cell stiffness; no external damping |
| `compliant` | Explicit alternative with both compliant attachments |
| `near_contact_reference` | Historical mechanical comparison preset; inspect its parameters before use |

Only the selected configuration has the final qualification scope described in the report. Presets are experiments, not equivalent guarantees. Selecting a leadscrew preserves the chosen mechanics; it uses its own motor current controller and reflected-inertia feedforward. The small reference motor can pass short dynamic tests while failing continuous thermal sizing.

The [slow-bump experiment](acf_experiment.m) uses a voice coil and **B+S = 2 kg** (S = 1.4 kg with the 0.6 kg holder), a 1 mm × 60 mm bump at 20 mm/s and 20 N. Save the SLX to keep workspace parameters and blocks together. Preserve any unsaved desktop edits before closing an older already-loaded model.
