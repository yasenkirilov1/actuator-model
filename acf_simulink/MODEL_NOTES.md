# Mechanics and implementation notes

All translation coordinates are positive **upward**, away from the workpiece.
Positions are changes from reference geometry; constant preload forces are
included explicitly. R is a prescribed robot boundary, not another free mass.
This is a one-dimensional axial model. Contact reaction is assumed parallel
to the flange axis. Tangential friction, changing surface-normal direction,
tilt and lateral loading are outside this first model.

## R–A–B–S mechanics

The topology is R — upper spring/damper — A — actuator — B — lower
spring/damper — S — rigid workpiece contact. There is no rigid bypass from R
to S. The two elastic links are preloaded, bilateral linear elements; this
first model does not simulate a compression-only spring becoming unseated.

With `M=mA+mB`, `alpha=mA/M`, `beta=mB/M`, `mu=mA*mB/M`:

```text
z = (mA*xA + mB*xB)/M       centre of mass
u = xA - xB                 change in actuator gap
xA = z + beta*u
xB = z - alpha*u
actual travel = midTravel + u

q1 = q10 + k1*(s-xB) + c1*(vS-vB)
q2 = q20 + k2*(xA-r) + c2*(vA-vR)
M*zdd = q1-q2-M*g_normal

mA*aA = Fa-q2-mA*g_normal
mB*aB = q1-Fa-mB*g_normal
mS*aS = Fc-q1-mS*g_normal
```

The masses determine how the two actuator attachments move. They are not
assigned arbitrary halves of a commanded displacement.

`g_normal=g*cos(angle)` with angle zero for a vertical upward axis. Reference
preloads are `q10=Fref-mS*g_normal`,
`q20=Fref-(M+mS)*g_normal`, and
`Fa0=Fref-(mB+mS)*g_normal`. These give a stationary loaded equilibrium.
They should be replaced with an equilibrium reflecting the final physical
preload arrangement if that differs from this parameterization.

Initial robot error is `r0=-initialExcessForce/keq`, where
`keq=k1*k2/(k1+k2)`. The flange starts compensated with `u0=r0`, `xB0=s0=0`.
It therefore starts at the target force while displaced from mid-stroke. The
slow loop then removes the robot offset. The excess-force parameter is the
force that would occur if the actuator were instead locked at mid-stroke;
it is not an extra arbitrary force injected into the mechanics.

## Actuator choices

The screw lead is converted to `ell=lead/(2*pi)` metres/radian. Its actual
angle and speed are `theta=u/ell` and `omega=udot/ell`. With rotor inertia J:

```text
(mu + J/ell^2)*udd = (motorTorque-frictionTorque)/ell - beta*q2-alpha*q1
motorTorque = Kt*current
Fa = mu*udd + beta*q2 + alpha*q1
```

The outer force controller requests speed. An inner speed PI requests motor
current. The current drive is a finite-response current regulator constrained
by supply voltage, winding resistance/inductance, and back EMF. Current-request
limits, voltage clipping and actual current are all retained. Mechanical
back-driving remains possible; rigidity is not treated as self-locking.
Friction is smooth Coulomb plus viscous sliding friction. No gearbox/backlash
or static stiction model is implied.

For a coil, the same relative-motion equation has no reflected rotor inertia:

```text
mu*udd = Kf*current - axialFriction - beta*q2-alpha*q1
```

Its force controller requests current. The reciprocal SI back-EMF constant is
taken as Kf. Both electromagnetic drives use four-quadrant voltage limits.
Their example current regulator computes the voltage needed for the requested
current response and clips it to the supply. This captures speed-dependent
loss of available effort without imposing a universal force-lag actuator.

The ideal-position variant requests a gap, generates bounded velocity and
acceleration, and calculates the required axial force. An optional force
budget can limit what it achieves. It does not implement instantaneous position
teleportation, which would make inertial requirements meaningless.

## Rigid unilateral contact

While contact is maintained, `s=y`, `vS=ydot`, and:

```text
Fc = q1 + mS*(g_normal + yddot)
```

If the required reaction becomes negative, contact releases, `Fc=0`, and S
moves freely. If S reaches the workpiece again, its relative closing velocity
is removed in a perfectly inelastic impact. The impulse is
`max(0,mS*(ydot-vS_before))`. It is logged separately from the ordinary force.
There is no contact stiffness or damping constant anywhere in the code.

## Numerical scope

The model uses a fixed-step discrete Simulink schedule. Inside the coupled
plant, classical RK4 integrates smooth mechanics/drive states. Surface
position/velocity are extrapolated using the analytic acceleration over a
single step; the robot boundary uses its position and velocity over that step.
The robot's own motion update respects its velocity/acceleration bounds.
Sampling and command delays occur at explicit integer tick boundaries.

Contact release, inelastic re-impact and stroke-stop projection are resolved
to the numerical timestep, not an exact event time. Impacts and stroke-stop
hits invalidate the force-tolerance test. Their transient peak forces are not
predicted. Use a smaller step to assess event-time/impulse convergence if those
events become the subject of a later study. Smooth maintained-contact cases
are the useful domain for accurate initial force/acceleration sizing.

The default bump shape is `64*q^3*(1-q)^3` for `0<=q<=1`, where duration is
width/scanSpeed. Ramps use `10*q^3-15*q^4+6*q^5`. Position, velocity and
acceleration match continuously at the endpoints. A literal height step is
deliberately excluded from a finite acceleration requirement calculation.

## Simulink implementation references

The builder uses documented native model/block APIs and Level-2 MATLAB
S-functions with separate output/state-update callbacks and explicit
feedthrough declarations. The new actuator command has no instantaneous
effect on the plant's positions or contact force. This breaks the fast loop's
algebraic path; the robot position/velocity states break the slow one.

- [MathWorks: Programmatic modeling basics](https://www.mathworks.com/help/simulink/ug/approach-modeling-programmatically.html)
- [MathWorks: Write Level-2 MATLAB S-functions](https://www.mathworks.com/help/simulink/sfg/writing-level-2-matlab-s-functions.html)
- [MathWorks: Level-2 MATLAB S-Function block](https://www.mathworks.com/help/simulink/slref/level2matlabsfunction.html)

These references establish the API usage. They do not validate the ACF
parameters or substitute for executing the package in MATLAB/Simulink.
