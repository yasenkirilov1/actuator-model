# Read-only plan for the compliant-carrier actuator comparison

Status: prepared from the integrator's proposed mechanical/control contract; **zero new native executions**. Wait for the frozen source, parameter MAT and helper hashes before changing a private model. The completed finite-cell reference remains separate.

The proposed link is k = 19,746 N/m with c = 60 N·s/m, rigid A, and the common B–S force vector, S acceleration and S–A position measurements. The coil prototype adds a proper causal carrier inverse, regularized by a two-pole 1.5 kHz filter. Exact coefficients, channel time frame, gate and actual-versus-identified parameters must come from the frozen source.

Let ell = lead/(2π), Kax = Kt/ell, Jax = Jrot/ell², and let d = xS − xB have the sign for which q1 = k*d + c*d_dot plus a fixed preload. In the following balances Fc denotes the axial contact component Rz; the qualification metric remains the projected true normal force Fn. The native physical force balance is:

```text
Fc = q1 + S*(g + aS)
q1 = Fa + B*(g + aB)
aB = aS - d_double_dot
torque/ell = Fa + Jax*(aR - aB)
```

Thus the current-domain screw command should include:

```text
force_command = requested_Rz - (B+S)*g - (B+S+Jax)*measured_aS
              + (B+Jax) * carrier_inverse[delta_requested_Rz - S*measured_aS]
              + Jax*issued_robot_acceleration + estimated_internal_friction
current_command = force_command / Kax + independent_normal_force_PI_PID
```

Here `carrier_inverse = s²/(k+c*s)` times the frozen proper lowpass. Apply the dynamic requested-force variation relative to its initial equilibrium, as the coil source does, to avoid a spurious startup impulse. Preserve the source's sensor/prediction timing; do not use true surface acceleration, true slope, future commands, or future profile values.

The inverse gain is **B+Jax**, rather than B. The separate +Jax*aR term must remain. For the recorded RE25/10 mm reference Kax = 14.7026536 N/A and Jax = 0.623759 kg. Equal force-domain P=6 N/N and I=400 N/(N·s) therefore start at **P=0.408089598 A/N and I=27.205973178 A/(N·s)**. The previous finite-cell P=0.040808960 A/N was ten times smaller and must not silently serve as the equal-gain comparison.

Internal friction conversion remains exact: Coulomb = tauC/ell = 0.628319 N; viscous = bRot/ell²; transition speed = omegaEps*ell. The physical winding, current regulator, voltage clipping, rotor inertia and transmission losses remain native. Continuous rating and peak effort remain separate.

S–A velocity is not B–A velocity for this softer link. With a matched, causal cell deformation estimate d_hat, the gap velocity is:

```text
estimated_velocity_A_minus_B = estimated_velocity_A_minus_S + d_hat_dot
```

A measured-force inverse `d_hat = q1/(k+c*s)` is causal when c>0, but its derivative has direct feedthrough and can amplify force noise. A desired-cell-force estimate or the frozen common estimator may provide a different tradeoff. Reuse the source's approved estimator if supplied; otherwise label the S-motion approximation and any bounded private estimator change explicitly. Adding a B sensor would require separate qualification and is not part of the default comparison.

Proposed bounded protocol after source review:

1. Reopen the copied source and check branch selection, actual force balance and rotor balance on a short flat case.
2. Run paired coil/screw sharp cases using the exact same mechanical, sensing, gate, issued-scan and recording settings. Use nominal force-domain gains first; any independently retuned screw row must record its actual gains.
3. Include both tool-mass endpoints and force endpoints, the fixed orientation endpoints, stochastic-noise off and at least the source's selected noisy seed. Limit extra tuning to an agreed small grid triggered by a concrete failure.
4. Retain no-clipping and clipped waveform/current evidence; report all preparation rejections and stops with null force metrics when no native result exists.
5. Refine the selected difficult screw case at 25/12.5/6.25 µs if the source's sample periods permit those steps. Score every native timestamp, including startup.
6. Bind the comparison to the frozen source/helper/parameter hashes and native history files. Do not relabel prior 64 MN/m or 200 µs/8 g histories as this softer-link experiment.

Catalog sizing remains unresolved for the small RE25 reference: 50 N at 10 mm needs 3.401 A and 26.831 W cold copper loss before guide losses, versus a 1.16 A/26.3 mNm continuous catalog provision. The larger EC-i 30 static example is not a simulated substitute. A fair transient controller pass would not qualify continuous sanding, backlash, thermal duty, screw elasticity, bearings or the physical prototype.
