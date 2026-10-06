# Run the fixed-mechanics sanding flange

This version follows the current mechanical choice: an upper **20 N/mm** spring with **zero external damping**, an unlimited ideal upper-spring working range, and a rigid lower attachment with the finite **64 MN/m load cell** retained. The selected controller is ordinary native Simulink PI, measured-inertia feedforward and bounded feedback around the load-cell mode. It runs directly, without an external learned-controller action input.

It is a **development baseline**. It passed the 11 controller-development cases, but the fresh tests did not satisfy ±0.5 N across the complete operating range. Keep the failed runs when comparing a change.

## Open and run

Paste this into the **MATLAB Command Window**:

```matlab
cd('/home/tommie/Projects/actuator-model/acf_simulink/generated/fixed_upper_20261005')
mdl = 'acf_force_control_fixed_upper_20261005';
open_system(mdl)
```

Press **Run** in Simulink. The default is a 2 s sharp-bump recording: 1 mm high, 10 mm wide, scanned at 100 mm/s, beginning at 1 s. The force target is 20 N, orientation 45°, robot-side body A = 1.5 kg, holder B = 0.6 kg and tool S = 1.4 kg. Thus **B+S = 2 kg**; A is additional.

The model uses Accelerator mode. The private acceptance run took 8.92 s on its first compiled run and 3.48 s warm, compared with 39.03 s in Normal mode on this computer. The complete 27-channel physical arrays matched exactly. Recompilation after an edit can take longer. These timings are a host-specific example.

The `helpers` directory must remain beside the SLX. The model loads its own namespaced preparation helpers automatically; they prepare coefficients and settings. The mechanical motion, actuator, sensors and controller run in native Simulink blocks.

## Change the tool, force or bump

The important `p` structure lives in the **Model Workspace**, which is separate from MATLAB's base workspace. You can inspect it through **Model Explorer → this model → Model Workspace**. The following commands are often easier:

```matlab
mw = get_param(mdl,'ModelWorkspace');
p = getVariable(mw,'p');
p.mechanics.mS = 2.0;          % tool only; B remains 0.6 kg
p.force.reference = 20;       % workpiece-normal pressing force, N
p.mechanics.angle = 0*pi/180;  % 0 degrees: gravity acts along the flange
assignin(mw,'p',p);
out = sim(mdl);
```

The model's initialization applies the edit and updates the known-tool inertia compensation. Do not substitute the combined B+S mass for `mS`. To restore the delivered example:

```matlab
assignin(mw,'p',getVariable(mw,'pDefault'));
out = sim(mdl);
```

For a slow bump with the original **2 kg tool+holder** example:

```matlab
p = getVariable(mw,'pDefault');
p.mechanics.mS = 1.4;
p.mechanics.angle = 0;
p.force.reference = 20;
p.surface.kind = 'bump';
p.surface.height = 0.001;
p.surface.width = 0.060;
p.surface.scanSpeed = 0.020;
p.surface.startTime = 2;
p.sim.stopTime = 8;
assignin(mw,'p',p);
out = sim(mdl);
```

This is a runnable example, not an additional qualified experiment. The delivered default and the recorded final grid use the sharp test.

To compare the lighter body assumption, change both the known actual body and its identified observer mass:

```matlab
p = getVariable(mw,'pDefault');
p.mechanics.mA = 0.4;
p.upperObserver.identifiedMassA_kg = 0.4;
assignin(mw,'p',p);
out = sim(mdl);
```

The final fresh cohorts use A = 1.5 kg. Actual and identified sensor, driver and robot timing remain separate parameters; changing a physical value does not automatically identify it.

## Select an actuator or controller

Double-click **Actuator selection** to choose **Voice coil** or **Leadscrew**. The delivered setting and all current fixed-mechanics qualification recordings use **Voice coil**. The retained leadscrew branch is an experimental comparison; it has not been qualified with the new fixed mechanics and modal controller.

Double-click **Force controller selection** for **PI** or **PID**, and its acceleration-feedforward checkbox. With zero derivative gain, choosing PID still behaves as PI. The checkbox changes acceleration feedforward; the contact-force reconstruction continues using the S accelerometer.

The executing normal-force gains are:

| Parameter | Delivered value | Units |
|---|---:|---|
| `p.inertial.forceKp` | 0.0245901639344 | A/N |
| `p.inertial.forceKi` | 16.3934426230 | A/(N·s) |
| `p.inertial.forceKd` | 0 | A·s/N |
| `p.inertial.derivativeFilter` | 0.001 | s |

For a controller experiment, edit these `p.inertial` fields and select PI or PID. The `p.coil.forceKp/Ki/Kd` aliases belong to another branch; they are not the gains executed by the normal-force controller. Keep Ki positive because the antiwindup expression uses it.

The selected cell-mode filter uses 1.644 kHz, width ζ = 0.2, virtual damping 1500 N·s/m and an auxiliary current bound of ±0.05 A. Its identified transfer is frozen independently of the actual sensors and driver. This delivered helper deliberately rejects edits that silently redesign that modal filter. A new modal design requires a separate controller candidate and fresh testing. The upper/lower mechanics are fixed in this version.

## Inspect the result

Double-click **Normal force error scope** after running. It displays true workpiece-normal error, reconstructed error and the ±0.5 N limits. The three diagnostic outputs also retain measured signals, physical performance and issued controller requests.

For an explicit plot of the workpiece force:

```matlab
x = out.yout.get(2).Values;
t = x.Time;
d = squeeze(x.Data);
if size(d,1) ~= numel(t), d = d.'; end
p = getVariable(mw,'p');
plot(t,d(:,1)); grid on
yline(p.force.reference,'k--');
yline(p.force.reference + 0.5,'r--');
yline(p.force.reference - 0.5,'r--');
xlabel('Time (s)'); ylabel('Workpiece normal force (N)');
```

Physical-performance columns 15 and 16 are actual coil current and voltage. Column 19 is contact present; column 22 is a hard travel-stop flag. Contact loss or a travel stop ends the simulation and marks a physical failure. A run that finishes can still fail the force requirement. Excursions outside ±7.5 mm of stroke center are recorded separately from hard stops.

The locked experiments use 32 V usable output and a 50 µs current-response target. A paired 48 V diagnostic helped a clipped stress case, but that case still exceeded ±0.5 N. Higher voltage is not a substitute for a qualified control/sensing design.

## Evidence

The input lock and immutable 736-case bank are under `validation/upper_no_damping_20261005/qualification_v8/`. Native C, helpers, controller coefficients and seeds are sealed. Final cohort results are in `report/fixed_qualification_results.json`; independent MATLAB checks, full failed histories and solver refinement are retained separately.

The runnable acceptance compares all 27 physical channels with the original selected native graph, verifies direct Run after parameter edits, exact save/reopen, both failure stops and Accelerator equivalence. It is software/model evidence. Installed sensor phase, the actual load-cell/guide modes, winding heating and sanding vibration still need physical measurements.
