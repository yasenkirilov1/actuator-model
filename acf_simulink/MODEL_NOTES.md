# Native model equations and control contracts

The saved SLX is the physical source. `acf_prepare` computes coefficients, loaded initial conditions and limits before simulation. Native Gain, Sum, Integrator, State-Space, Discrete Transfer Fcn, delay and saturation blocks execute the plant and controllers. The report gives the design rationale and source-backed figures; this document defines the implementation.

## Coordinates, bodies and links

R is the commanded robot reference, A the robot-side flange body, B the actuator carrier/holder and S the tool downstream of the sensing element. Positive z points away from the surface, x follows the scan. Axial gravity is `g = 9.81*cos(angle)`; angle is measured from upward vertical. B is provisionally 0.6 kg, S ranges from 0.1 to 2 kg, and A is additional.

The selected R–A connection is rigid. The B–S connection contains an opposed guided pair of **net k = 19,746 N/m** and a parallel damper **c = 60 N·s/m**, with the cell in the complete load path. The selected native plant uses k1 and c1 directly and a massless joining frame. It neglects the representative 64 MN/m cell's approximately **0.031% static series-compliance correction** and has no independent cell state. A real joining frame can introduce extra kilohertz modes. The stiff no-springs reference retains finite cell stiffness.

The load path is **B → spring pair/parallel damper → joining frame → multi-axis cell → S**. Supports must not bypass the cell. The current joining frame is massless; its real mass and mounting modes need identification. The no-springs preset uses rigid replacement couplers while retaining finite cell stiffness. Zero external damping retains internal actuator friction.

## Contact and normal pressing force

While contact remains closed, the prescribed surface height constrains S. For slope `s = dh/dx`, normal and tangent are:

```
n = (-s, 1)/sqrt(1+s^2)
t = ( 1, s)/sqrt(1+s^2)
Rcontact = N*n - mu*N*t
N = (Rz - s*Rx)/sqrt(1+s^2)
```

`N` is the pressing force used for qualification. The native contact log records N, lateral reaction, true slope, estimated N/slope, requested axial reaction, measured lateral signal and command-based robot velocity. Physical `mu` belongs to the plant; the controller estimates drag from reconstructed forces and never reads `mu` or the programmed surface profile.

The sharp bump is `h = 64*H*q^3*(1-q)^3`, for q from 0 to 1 over `T = width/scanSpeed`. At H = 1 mm and T = 0.1 s, peak downward acceleration is −2.4 m/s², peak absolute jerk is 384 m/s³ and peak speed is 34.35 mm/s. This finite smooth test does not model abrasive roughness.

If N becomes nonpositive or actuator travel reaches a stop, simulation stops and the run fails. It does not continue through contact loss, impact or free flight.

## Cell, accelerometer and matched acquisition

With all of S beyond the cell, its vector reading is:

```
Lraw = Rcontact - mS*(aS - gvector)
```

The established static gravity tare is matched to the acceleration zero:

```
Rcontact_est = Ltared + identified_mS * measured_dynamic_aS
```

No second gravity correction is added. Force and acceleration need matched complex response and timestamps; equal sample rates alone do not cancel inertia. `mechanics.mS` is the actual mass source. Identification derives from `sensor.distalMassCalibrationScale`; holder mass and force constant have separate calibration scales too.

The final acquisition graph uses 100 ksample/s raw sampling, matched two-pole 5.1 kHz response envelopes, a 50 µs EMA on that raw clock and fresh latching to the 10 kHz controller. Native nominal low-frequency filter delay is **88.92 µs**, plus **20 µs pipeline**. Nominal conditioned noise is 0.02 N RMS per force axis, 0.014 m/s² acceleration and 1 µm position. Acceleration has ±50 g clipping and a 0.00468 m/s² ideal quantum.

Noise is injected before conditioning, normalized to the nominal response. Changing an actual filter does not renormalize physical noise downward. White/bounded sample noise does not describe sanding vibration, rectification, thermal drift or calibration accuracy.

`sensor.delay` is the common pipeline setting. Channel offsets and `acquisition.motionFrameDelayOffset_s` allow deliberate mismatch. Actual filter scales are distinct from the identified response used to align robot prediction. The public model requires this acquisition graph; legacy envelopes remain only in frozen inputs.

## Motion observer and drag estimate

Position measures S relative to A. A two-state gap/velocity observer receives position, measured S acceleration and causal robot prediction. With R–A rigid, the prediction supplies A motion. Relative velocity and commanded scan speed estimate slope. `surface.scanSpeed` updates the observer's commanded speed; `contact.commandedScanSpeedScale` represents deliberate identification error.

```
mu_raw = -(Rx_est + slope_est*Rz_est)/(Rz_est - slope_est*Rx_est)
Rz_requested = N_reference*(1 - mu_est*slope_est)/sqrt(1+slope_est^2)
```

The drag estimate is valid only above a 1 N normal/denominator threshold, bounded to [−1,1], held when invalid and filtered over 5 ms. It avoids assuming a fixed sanding coefficient in the controller.

## Force controller and compliant-link compensation

Selected coil feedback is **P = 0.3 A/N, I = 20 A/(N·s), D = 0**, with a feedback-only 1 kHz two-pole filter and anti-windup. Static holding current balances the target and known gravity. Measured acceleration, normal-to-axial geometry and guide friction supply dynamic feedforward.

Guide friction is smoothed Coulomb plus viscous drag. Feedforward uses estimated S–A velocity with a 400 µs causal lead. This approximates B–A actuator velocity with a compliant B–S link; the residual remains in feedback.

Let `q = xS-xB`. The link obeys `delta_L = k*q+c*qdot`, and `aB = aS-qddot`. Net actuator force, after guide friction, is:

```
Fa_net = Rz - (mB+mS)*(g+aS) + mB*qddot
Fcoil = Fa_net + guide_friction
```

The proper causal inverse generates the last term:

```
delta_L_desired = (Rz_requested - N_reference) - identified_mS*measured_aS
I_inverse = (identified_mB/identified_Kf)
            * [s^2/(identified_k+identified_c*s)]
            * LP2(1500 Hz) * delta_L_desired
```

It uses measured signals and requested load, without truth acceleration or preview. The selected loaded B mode is about 28.9 Hz; mounting modes still require physical tests. **Feedforward PI / PI / PID** groups dynamic feedforward on/off while retaining the same normal-force reconstruction. Coil gains live in `p.inertial`, screw current gains in `p.screw`.

## Drives and robot recentering

The electrical model includes R/L, back EMF, a closed-loop current-response target, usable bipolar voltage/current limits and guide friction. Its current feedback is ideal; measured performance under the actual hot winding must establish the target.

For screw lead, `ell=lead/(2*pi)` and shaft inertia reflects as `Jax=Jrotor/ell^2`. The screw carrier inverse uses `mB+Jax` and adds `Jax*predicted_robot_acceleration`; shaft friction converts through ell. Smaller lead reduces holding current but increases reflected inertia. Short dynamic passes and continuous motor sizing are separate.

The robot receives 100 Hz commands, with nominal 80 ms delay and 60 ms response. Its predictor uses issued/pending commands. New recentering commands require filtered error within ±0.2 N, trend below 2 N/s and 100 ms dwell. A 10 kHz gate also requires variation below 0.08 N RMS, with a 100 ms mean and 20 ms variance filter; every unstable fast sample resets dwell. The gain cap includes robot delay, response and gate lag; selected effective gain is about 0.735 s⁻¹.

The gate does not cancel queued commands or guarantee stability with an incorrect predictor. Actual and identified robot dynamics remain separate. ±20–40 ms unidentified extra delay can fail despite apparently calm force.

## Logs, metrics and scope

`acf_run` maps public force to native N and retains Rz in `r.axialContact`. `acf_metrics` scores every normal-force sample, including startup, complete duration and physical guards. Plot thinning cannot change the metric. Cases start in established loaded contact at the requested force/orientation; first touch, force-step engagement and changing orientation are not qualified.
