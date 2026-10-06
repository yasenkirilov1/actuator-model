# Structural force-loop review: finite B–S cell and native V4 control

The force controller must suppress the finite cell's B-side mode while retaining ±0.5 N workpiece-normal force tracking. A total-current notch can reduce excitation at a nominal frequency while leaving the free mode weakly damped and vulnerable to stiffness changes. The strongest simple alternative supported by this review is measured friction-ratio feedforward with phase-shaped PI feedback; a small structural observer is a fallback if the remaining native margin is demonstrably modal.

This is a read-only calculation from actual frozen V4 blocks and coefficients. It introduces no model or helper changes and runs no new native simulations. The [JSON](structural_control_review.json) retains coefficient checks, exact sampled matrices, complete pole grids, calculation source, a noise-limited observer example, and source hashes. Analytical point counts are not native qualification counts.

## Physical mode and sensing

Let `q = zS - zB`, with z positive away from the workpiece. In maintained contact:

```text
mB*qdd = Kf*i - guideFriction(vA-vS+qdot)
         + mB*(g+aS) - k*q - c*qdot
Rz = k*q + c*qdot + mS*(g+aS)
aB = aS - qdd
Rz = Kf*i - guideFriction + (mB+mS)*(g+aS) - mB*qdd
```

Here g is the positive downward gravity component along the flange. The measured S acceleration corrects the distal tool's load-cell inertia, but does not measure the final `-mB*qdd` contribution from motion of B relative to S. S–A position also does not directly see that B–S deflection. The cell force does see it, so a cell-driven structural observer is possible without another position sensor.

For B = 0.6 kg and k = 64 MN/m, the constrained-contact natural frequency is 1643.745 Hz. S is prescribed by the maintained surface constraint in this model; its mass changes disturbance and measurement-noise sensitivity, but does not enter this particular B-mode frequency. A real sanding pad, workpiece compliance, rotation, or flexible tooling can introduce further modes.

The native guide model is `0.2*tanh(vGap/0.0002) + 2*vGap` N. Its tangent damping is highly dependent on operating velocity:

| Guide velocity | Tangent damping |
|---|---:|
| 0 | 1002 N·s/m |
| 0.2 mm/s | 422 N·s/m |
| 0.5 mm/s | 28.6 N·s/m |
| 1 mm/s | 2.18 N·s/m |
| 5 mm/s | 2 N·s/m |

Zero-speed linearization therefore overstates modal damping during faster parts of the bump. At 2 N·s/m the physical mode has damping ratio about 0.00016. The guide friction feedforward uses S–A kinematics, so it does not directly measure and cancel B-mode velocity.

## Actual sampled loop

The frozen source is [V4](</home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v4-20260930/frozen-inertial-v4/acf_force_control_normal_trial.slx>), SHA256 `2aaafd4da3b6f5593157f1667e84e2fb72ebdc994804f743f8e5c80d8202668b`. The finite candidate is [inertial_mechanics_feedback_candidate_20260930.mat](inertial_mechanics_feedback_candidate_20260930.mat). Recomputed 1000 Hz feedback-filter numerator and denominator match its saved coefficients exactly.

The calculation uses exact sampled continuous B mechanics and winding-current dynamics at 100 µs, the native sensor EMA followed by one sample of transport delay, the forward-Euler PI integrator, and the helper's exact bilinear LP/notch coefficients. Current and voltage saturation and antiwindup are inactive in this local calculation. Native 10 µs RK4 integration error is not reproduced.

For normalized states `x = [k*q, k*qdot/omega0, Kf*i]`, where `omega0=sqrt(k/mB)`:

```text
A = [ 0          omega0            0
     -omega0    -(b+c)/mB          omega0
      0          0               -1/tauI ]
B = [0, 0, 1/tauI]'
Rz = [1, c*omega0/k, 0]*x
```

Discretize `(A,B)` with a matrix exponential and a held issued current request. For a frozen slope s and friction coefficient µ, `N = Rz/aZ`, where `aZ=(1-µ*s)/sqrt(1+s²)`.

The native measurement response is `H(z)=(1-alpha)/(z-alpha)`, with `alpha=exp(-Ts/filterTime)`. At the nominal mode:

| Contribution | Phase |
|---|---:|
| Native force response plus transport | −80.45° |
| Continuous current regulator | −45.92° |
| Command zero-order hold | −29.59° |
| PI, Kp = 0.03 A/N and Ki = 20 A/(N·s) | −3.48° |
| Optional 1000 Hz Butterworth feedback LP | −129.73° |

The unfiltered feedback path has roughly −159° of extra phase. Adding the LP gives roughly −289°. Around the lightly damped mode this phase change can add damping; gain attenuation alone is not the whole explanation. A raw PID with the existing 300 µs derivative filter does not supply enough lead to overcome the sensing/current delay. The calculated flat-mode growth is +915/s for raw PI, +1275/s with Kd = 3e-6 A·s/N, and +1938/s with Kd = 1e-5 A·s/N. These are local structural calculations, not general statements that PID cannot work.

The native `filterTime=100 µs` identifies a **discrete EMA response**. A continuous analog 100 µs RC followed by 100 µs transport would have −105.10° at this frequency, rather than −80.45°. The installed transfer must fit the tested response; the same numeric time constant does not establish the same high-frequency phase.

## Geometry feedforward is also a feedback path

The existing actuator reference is `Rz_requested = Nref*sqrt(1+s_hat²) + s_hat*Rx_measured`. For frozen correct s, `Rx=-N*(s+µ)/sqrt(1+s²)`. Its changing force component is therefore an additional feedback term outside the PI/PID-only LP:

```text
positive force-feedback controller gain
    = Kf*C_PI*LP + s*(s+µ)/sqrt(1+s²)
```

At the largest rising slope, s = 0.34346 and µ = 0.3, the extra geometry coefficient is about 0.209 N/N. PI alone has magnitude about 0.581 before filtering, and about 0.181 after the 1000 Hz LP at the mode. The unfiltered geometry path is therefore comparable to the filtered PI path. A total-current notch covers both; the feedback-only LP does not.

This explains why a controller that appears quiet on flat contact can behave differently on ascent. The sampled calculation also reproduces the reported high-P failure: Kp = 0.06 with a Butterworth 1000 Hz LP gives an unstable approximately 1000 Hz pole on maximum ascent. Critically damped filtering changes that result.

## Measured friction-ratio feedforward

The authorized aligned force vector and causal slope estimate give:

```text
Nhat = (Rz_hat - s_hat*Rx_hat)/sqrt(1+s_hat²)
mu_raw = -(Rx_hat + s_hat*Rz_hat)/(Rz_hat - s_hat*Rx_hat)
Rz_requested = Nref*(1 - mu_hat*s_hat)/sqrt(1+s_hat²)
```

This uses measured tangential force divided by measured normal force, with the sign for positive scanning. It does not require known µ, actual force, or the programmed bump profile. Guard the denominator and update only when reconstructed normal force exceeds 1 N; hold otherwise. Clamp the estimate to [−1,1], filter it with an initially zero 5 ms EMA, and retain the full vector-projection normal-force feedback unchanged. The initial estimate has no axial-reference effect on flat contact, where s = 0.

With a correct frozen slope and common force-vector transfer, a radial modal force variation scales both components equally and cancels exactly in the ratio. The previous force-dependent geometry path disappears. Changing friction, mismatched sensor transfer, normal-direction error, tool dynamics, and division noise remain validation subjects.

An offline replay on the old retained seed29 trajectory confirms the available vectors exactly reproduce its logged normal estimate. With a 5 ms µ filter, the quiet estimated µ is 0.29998 with standard deviation 0.000226; during the fifth bump its RMS error against the simulated 0.3 is 0.000416. This is an observer-input check on a fixed trajectory, not a new force-tracking result.

## Stability and stiffness sensitivity

At nominal stiffness and minimum tangent damping, the calculated structural decay rates are:

| Strategy, Kp = 0.03 / Ki = 20 | Flat contact | Maximum ascent |
|---|---:|---:|
| Raw PI | +915/s, grows | +1550/s, grows |
| Feedback LP1000, Butterworth, original geometry | −398/s | −346/s |
| Feedback LP1000, Butterworth, radial geometry path removed | −398/s | −455/s |
| Total-current notch, zero damping 0.001 / pole damping 0.5 | −1.63/s | −2.39/s |
| Wider total-current notch, 0.001 / 1 | −1.43/s | −1.46/s |

The notch strategies suppress excitation but leave a nearly free mode. Increasing pole damping broadens attenuation; it does not necessarily damp the underlying oscillator. With the 0.001/1 notch center held fixed, only +10% stiffness gives approximately +131/s mode growth on maximum ascent.

The pattern is corroborated by the separate [native frequency-sensitivity batch](inertial_mechanics_structural_cell_frequency_sensitivity_20260930_results.json): all six cases pass at actual frequency ratios 0.8 and 0.9, while all six lose contact before the bump at 0.7, 1.1, 1.2, and 1.3. The local minimum-tangent calculation predicts the same asymmetric stable/unstable pattern. It does not predict the exact nonlinear loss-of-contact time. Ratio feedforward alone does not fix flat-contact notch detuning, because s = 0 already removes that geometry term.

The ratio-feedback LP screen evaluates k = 40–110 MN/m, tangent damping 2–1002 N·s/m, slopes 0, +0.343, −0.15, −0.343, and µ = 0.3/0.5, giving 240 frozen points per setting:

| Kp, Ki = 20 | LP1000 damping | Stable points / 240 |
|---|---:|---:|
| 0.03 | 0.707, 1, 1.2, or 1.5 | 240 for each |
| 0.06 | 0.707 | 122 |
| 0.06 | 1 | 225 |
| 0.06 | 1.2 | 236 |
| 0.06 | 1.5 | 240 |
| 0.10 | 1.5 | 193 |

Kp = 0.06 / Ki = 20 with LP1000 damping 1.5 is a bounded candidate worth testing natively if more margin is needed. Its worst calculated growth rate is −168/s across this grid. Stability is not transient force qualification. The k brackets are engineering sensitivities; no manufacturer stiffness tolerance or loaded assembly damping has been established.

## Compact structural observer, if it becomes necessary

A four-state observer can use the B–S force sensor to recover the missing mode without surface preview. Its states are mechanical deflection, relative velocity, winding current, and the combined native cell-filter/delay state. The native EMA plus one-sample delay requires one response state, rather than separate filter and delay states.

Let `(Ap,Bp,Cp)` be the sampled three-state plant above, now driven by the issued clipped current request and measured S acceleration with known gravity and guide-motion inputs. Augment:

```text
Ao = [ Ap                   0
       (1-alpha)*Cp         alpha ]
Bo = [ Bp
       0 ]
Co = [0 0 0 1]

xhat[k+1] = Ao*xhat[k] + Bo*u_issued[k]
            + L*(cell_measured[k] - Co*xhat[k])
```

The observed cell signal is calibrated and gravity-tared; initialize the corresponding equilibrium consistently. This observer models the delayed force response explicitly. Actual winding-current telemetry can improve it, but the present native proposal predicts current from issued commands. Voltage clipping, thermal force-constant changes, or a mismatched current regulator require separate validation.

Prefer a bounded relative-velocity damper, optionally compensating current lag:

```text
i_damp = -d_v*(qdot_hat + tauI*qdd_hat)/Kf
```

Place the damping contribution where it can act on the mode; a total-current notch that removes it at that frequency defeats its purpose. Preserve current clipping and antiwindup. An illustrative nominal observer and d_v = 300 N·s/m, with 100 µs current compensation and LP1000 feedback, changes calculated modal decay from −398/s to −585/s. It stays stable with its nominal 64 MN/m observer when actual k is 51.2–96 MN/m in the limited flat screen. Its force-cell-noise-only physical force RMS rises from 0.00997 to 0.01129 N. Acceleration/position noise, profile variation, actuator mismatch, and nonlinear clipping were omitted; this is not a qualified controller recommendation.

Simply adding full B-acceleration feedforward is more fragile. For
`Fa = Rz_ref - M*(g+aS) - gamma*mB*(aB-aS)`, the relative equation has effective mass `(1-gamma)*mB`. Gamma = 1 cancels modeled inertia instead of adding damping and creates a delay-sensitive ideal limit. An identified structural predictor used for controlled damping is the better fallback.

**Recommendation:** qualify measured friction-ratio feedforward with the unchanged normal-force reconstruction and a feedback-only LP, without depending on a fixed cell-mode notch. Keep P = 0.03 / I = 20 as the initial simple strategy; test P = 0.06 with LP damping 1.5 only if native results justify it. Identify the loaded cell/holder transfer and sensor/current phase before treating the prototype as qualified. Add the four-state modal damper only if the retained native evidence still shows a modal performance limit.


## Band-limited acceleration feedforward before adding an observer

Filtering the force-controller correction does not filter the direct measured-acceleration feedforward. A pure causal prediction adds noise at the cell mode. An additional actuator-only first-order command filter can suppress that noise while retaining low-frequency delay compensation. The original aligned accelerometer remains unchanged in normal-force reconstruction.

For the native filtered derivative:

```text
aDer = exp(-Ts/300us)
derivative[k] = aDer*derivative[k-1]
                + (1-aDer)*(accel[k]-accel[k-1])/Ts
predictedAccel[k] = accel[k] + T*derivative[k]
aFF = exp(-Ts/tauFF)
commandAccel[k] = aFF*commandAccel[k-1]
                  + (1-aFF)*predictedAccel[k]
T = Ts*aFF/(1-aFF) + 308.20us
```

The 308.20 µs combines the native sensor EMA's 58.20 µs DC group delay, 100 µs transport, 100 µs current response, and 50 µs command-hold contribution. Choose T using the actual discrete command-filter delay; a direct EMA has delay `Ts*aFF/(1-aFF)`, rather than exactly `tauFF`. Use explicit direct recursion or equivalent direct-feedthrough filter coefficients; a strictly proper Transfer Fcn adds another sample.

| Command filter tauFF | Prediction T | Gain at 20 Hz | Phase at 20 Hz | Gain at 1643.7 Hz | FF force-request noise RMS |
|---|---:|---:|---:|---:|---:|
| None, no prediction | 0 | 1 | 0° | 1 | 0.03217 N |
| None, pure prediction | 200 µs | 1.001 | +1.44° | 1.619 | 0.04876 N |
| 700 µs | 959.4 µs | 1.008 | +2.16° | 0.575 | 0.02654 N |
| 1000 µs | 1259.0 µs | 1.010 | +2.12° | 0.500 | 0.02430 N |
| 1500 µs | 1758.8 µs | 1.014 | +2.03° | 0.441 | 0.02229 N |

Noise calculations use the verified output-normalized acceleration RMS 0.0123 m/s² plus the 0.00468 m/s² ADC quantum, the unchanged 0.02 N post-response force noise, B+S = 2.6 kg, and the exact native sampled transfers. No extra noise attenuation is assumed in normal reconstruction. The scalar filter does not change the frozen force-feedback poles because S acceleration is an exogenous input under the maintained-contact constraint.

Including the correlated use of measured acceleration in both PI feedback and actuator feedforward is essential. In the frozen flat ratioFF loop:

```text
force_from_accel_noise = -Gp*(mS*Cfb + M*H_FF)
                        / (1 + Gp*Cfb*Hsensor)
force_from_cell_noise = -Gp*Cfb / (1 + Gp*Cfb*Hsensor)
```

Here `Cfb` includes Kf times the native PI and feedback LP; `Gp` includes the finite cell and current regulation. With P = 0.03, I = 20, and Butterworth LP1000, the calculated combined physical force RMS is:

| FF shaping | k = 64 MN/m | k = 110 MN/m |
|---|---:|---:|
| None | 0.0658 N | 0.1288 N |
| Pure 200 µs prediction | 0.1020 N | 0.2128 N |
| 700 µs filter with matched prediction | 0.0555 N | 0.0694 N |
| 1000 µs filter with matched prediction | 0.0507 N | 0.0616 N |
| 1500 µs filter with matched prediction | 0.0467 N | 0.0554 N |

These are stationary noise-transfer calculations, not native peak-force results. They omit position/slope/µ-observer noise, bias, phase mismatch, changing surface geometry, and clipping. At 100 Hz the shaped gain is about 1.15–1.17, so a reduction in RMS does not prove the bump-entry transient will improve. Start a bounded native test with 700 or 1000 µs command filtering and the matching prediction above; select from retained bump and repeated-bump results. The compact structural observer remains a fallback if this simpler shaping is insufficient.

## Modest drive and feedback changes after the V7 finite-cell screen

This addendum retains the earlier 100 µs sensor EMA plus 100 µs transport, P = 0.06 A/N, I = 20 A/(N·s), feedback LP damping 1.5, ratio feedforward, and no total-current notch. It extends the exact sampled frozen-point calculation to 20 MN/m and compares current response and feedback cutoffs. No native simulation or shared model change was made.

Lowering the feedback cutoff removes useful modal damping. It does not necessarily lower physical force noise, because the direct acceleration feedforward still drives the oscillator.

| Current response | Feedback LP | Stable frozen points, 40–110 MN/m | Worst growth rate | Heavy flat true-force RMS, 64 / 110 MN/m, direct FF |
|---|---:|---:|---:|---:|
| 100 µs | 400 Hz | 240/240 | −13.9/s | 0.1140 / 0.2790 N |
| 100 µs | 600 Hz | 240/240 | −46.5/s | 0.0801 / 0.1496 N |
| 100 µs | 800 Hz | 240/240 | −99.1/s | 0.0683 / 0.1034 N |
| 100 µs | 1000 Hz | 240/240 | −168.2/s | 0.0653 / 0.0827 N |
| 100 µs | 1200 Hz | 240/240 | −9.5/s | 0.0670 / 0.0727 N |
| 50 µs | 1000 Hz | 240/240 | −49.5/s | 0.0775 / 0.0867 N |
| 150 µs | 1000 Hz | 240/240 | −99.1/s | 0.0574 / 0.0790 N |
| 200 µs | 1000 Hz | 240/240 | −65.8/s | 0.0517 / 0.0758 N |

The grid is k = 40, 51.2, 64, 80, 96, 110 MN/m; guide tangent damping 2, 5, 50, 200, 1002 N·s/m; slopes 0, +0.34346, −0.15, −0.34346; and µ = 0.3/0.5. These are engineering sensitivities. Noise integration uses flat maintained contact, S = 2 kg, output acceleration RMS 0.0123 m/s², ADC quantum 0.00468 m/s², and force output noise RMS 0.02 N; it includes the correlated acceleration use in feedback and feedforward.

The same linear model accepts 20 MN/m mathematically, but the existing controller cannot be extrapolated to that stiffness: only 6/40 frozen points are stable for LP1000, with worst growth +716/s. LP400 and LP600 each stabilize all 40 tested points at 20 MN/m, while strongly worsening high-stiffness modal noise. There is no demonstrated uniform 20–110 MN/m controller qualification.

The stationary calculation reproduces [the retained V7 flat-force analysis](normal_mass_controller_v7_drive_shape_analysis_20260930.json) closely: current100/direct FF gives 0.06527 N calculated versus 0.06537 N native; current100/FF1ms gives 0.05355 versus 0.05411 N; current50/direct gives 0.07750 versus 0.07779 N. This supports the noise mechanism. It does not predict the bump-entry maximum or qualify ±0.5 N.

With the old timing, bounded native candidates are current150/direct FF, current150 with actuator-only FF1ms and prediction 1.30903 ms, or current200 with FF1ms and prediction 1.35903 ms. Their slower current response can add deterministic entry lag; the compensated feedforward must be scored natively. These candidates are conditional on the old sensor response and may be superseded by the separately designed acquisition path below.

### Filtered derivative sensitivity

A derivative filter of 300 µs with D = 3e−6 A·s/N retains all 240 frozen points stable and changes worst overall growth from −168/s to −188/s. D = 1e−5 leaves 230/240 stable, with +414/s growth at low stiffness on ascent. D = 3e−5 leaves only 95/240 stable, with worst +1327/s. Only the smallest derivative merits a bounded native probe.

Even that gain does not reduce nominal stationary noise: at 64 MN/m direct-FF RMS changes 0.06527 → 0.06543 N; at 110 MN/m 0.08267 → 0.07770 N; at 40 MN/m 0.07978 → 0.09052 N. Improved local decay does not imply a useful force-tracking margin.

## Engineering fast common sensor path

The companion [accelerometer review](accelerometer_physics_review.md) specifies a designed common two-pole 5.1 kHz response, followed by a fresh 50 µs EMA at 100 kHz raw acquisition, then fresh-frame decimation to the 10 kHz controller. It has approximately 89.3 µs DC delay and, at 1643.7 Hz, gain 0.884 and phase −51.4° before intrinsic sensor phase and pipeline timing. The older common native EMA100 plus100 µs delay has gain 0.726 and phase −80.4°. The new target requires acceleration output noise about 0.014 m/s², derived from 25 µg/√Hz and the actual engineered response; 0.0123 cannot be carried over unchanged.

The exact 10 µs plant and three-state common filter were interconnected and lifted over ten raw steps to the 100 µs controller. The pipeline sensitivity holds the previous command for the first 0, 2, or 5 raw steps, then applies the current command. The table is for the same 240 mechanical/geometry points above, I = 20 and feedback-filter damping 1.5.

| P and feedback LP | 0 µs pipeline | 20 µs pipeline | 50 µs pipeline |
|---|---:|---:|---:|
| P0.03, LP1000 | 240/240 | 240/240 | 240/240 |
| P0.06, LP600 | 240/240 | 240/240 | 240/240 |
| P0.06, LP800 | 240/240 | 240/240 | 240/240 |
| P0.06, LP1000 | 237/240 | 240/240 | 240/240 |
| P0.06, LP1200 | 210/240 | 227/240 | 231/240 |

P0.06/LP1000 at zero pipeline grows at three low-stiffness ascent points, with worst +95/s; a faster response is not monotonically stabilizing. P0.06/LP800 has worst growth −188, −182, and −98/s at 0/20/50 µs and is the bounded initial high-gain candidate for this engineered path. P0.03/LP1000 is a lower-gain baseline. Actual ATI filter damping and ADXL intrinsic response are not supplied as guaranteed complex transfers here, so these are design calculations, not hardware qualification.

The physical noise calculation uses the engineered analog/digital transfer and folds white noise to the 10 kHz sampling grid. With S = 2 kg and direct acceleration feedforward, calculated true-force RMS at 64/110 MN/m is 0.0760/0.0900 N for P0.06/LP800 at ideal zero pipeline and 0.0735/0.0969 N at 20 µs. P0.06/LP1000 at 20 µs gives 0.0726/0.0812 N. The engineered fast path improves available deterministic sensing phase but does not by itself lower structural noise. Native bump peaks, realistic phase mismatch, sanding vibration, and the actual loaded assembly must choose the final control setting.

## External A–B viscous damping and controller sensitivity

This addendum uses the earlier 100 µs common response plus 100 µs transport, FF1ms/prediction1.25903ms, P0.06/I20, and no total-current notch. It asks whether a total guide viscous coefficient of 120 N·s/m, including the earlier 2 N·s/m, permits more aggressive feedback filtering. The frozen grid retains 40–110 MN/m, slopes/µ above, and adds the Coulomb tangent range 0/3/48/198/1000 N·s/m to that viscous value.

| Feedback filter | Stable points / 240 | Worst growth rate |
|---|---:|---:|
| LP1000, damping0.707 | 123 | +927/s |
| LP1000, damping1.0 | 225 | +394/s |
| LP1000, damping1.2 | 236 | +21/s |
| LP1000, damping1.5 | 240 | −187/s |
| LP1200, damping1.5 | 240 | −35/s |

Additional mechanical damping does not justify damping0.707/1/1.2 across the selected stiffness range. LP1200/damping1.5 is the bounded faster-feedback candidate among these settings, with less stability reserve than LP1000/damping1.5. Its deterministic force-entry behavior still needs native scoring.

The stationary comparison explicitly includes the native 40 Hz head-motion observer, raw position-noise RMS 1 µm, its 100 µs stroke delay and alignment EMA, guide-velocity prediction400µs, and correlated use of the same accelerometer in force reconstruction, motion estimation, and acceleration/guide feedforward. On flat contact, velocity-estimation noise also changes both the normal projection and the geometry reference; that contribution is retained. At 50 N, µ0.3 and scan0.1m/s, its coefficient is `Nref*mu/vscan = 150 N·s/m`.

For the observer transfers `Hvp` from gap position to gap velocity and `Hva` from measured S acceleration to gap velocity, the actuator disturbance coefficients are:

```text
acceleration:
 -mS*Cfb - M*HFF + (Nref*mu/vscan)*(Cfb+1)*Hva
                 + b*(Hva - guidePredictionTime)

position:
 -[b + (Nref*mu/vscan)*(Cfb+1)]*Hvp*Hsensor
```

Divide their physical-force responses by `1+Gp*Cfb*Hsensor`. Position and force-cell sources are independent; acceleration is kept correlated across all its branches. This is flat minimum-Coulomb-tangent sizing at nonzero relative velocity; the Coulomb direction near zero and its saturation remain native nonlinear sensitivities.

At b120, position noise contributes guide-prediction velocity RMS 19.51 µm/s and acceleration noise 13.86 µm/s. Their direct viscous force requests have RMS 0.00234 and 0.00166 N. Position/geometry contribution to the resulting true force is about 0.005 N. Thus those paths are included without assuming that an external damper is noiseless.

With S2kg and matched FF1ms, combined calculated true-force RMS at 64/110 MN/m is 0.05324/0.04763 N for LP1000/damping1.5 and 0.05807/0.04886 N for LP1200/damping1.5. The faster filter gives little stationary-noise benefit and weaker broad stability reserve. The separate native damper sweep determines whether deterministic bump tracking and repeated-bump margin improve.

