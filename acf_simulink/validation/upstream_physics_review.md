# Upstream sensor and causal-control review

This note derives the sensor equations and reviews archived native Simulink
traces. It does **not** qualify a newly simulated closed loop. The accompanying
Python/JSON files contain estimator-only replay, keeping the archived issued
current commands fixed.

## Physical sensor definition

Take the model's positive axial direction upward and write
`g_axis = 9.81*cos(theta)`. The actual load cell is between B and S; S is
1.4 kg and lies downstream of it. The S free-body equation is

```
mS*aS = Fcontact - q1 - mS*g_axis.
```

Consequently its raw mechanical reading is
`q1 = Fcontact - mS*(g_axis+aS)`. Exact gravity tare gives

```
Fcell_tared = Fcontact - 1.4*aS.
```

This difference is physical inertia, not a load-cell error. The specified
sharp bump has maximum downward axial acceleration -2.4 m/s^2, so the
maximum difference is 3.36 N. A controller regulating only the tared cell
cannot establish the required true contact force. If 0.1 N of an error budget
is assigned to distal-inertia compensation, the acceleration error must be
at most 0.1/1.4 = 0.07143 m/s^2. Known mass and gravity tare address the static
term; they do not remove the dynamic term.

With both attachments rigid, the moving downstream mass is B+S = 2 kg.
Around a gravity-balanced operating point,

```
q_cell = Fa + 0.6*aS
Fcontact = Fa + 2*aS
Fcontact = (2/0.6)*q_cell - (1.4/0.6)*Fa.
```

Here `Fa` is net transmitted actuator force, including friction. This limit is
observable from a modeled actuator force and the B/S cell, even without a
position measurement. Its direct algebraic reconstruction amplifies cell
noise by 3.333 and actuator-force error by 2.333. That error amplification
describes this reconstruction method; it is not a universal requirement on
force-constant calibration when an independent position channel is also used.
An upstream cell before *all* 2 kg instead reads essentially actuator force
in the both-rigid limit and needs another measurement to resolve common
robot/tool acceleration.

## Position measurement frame

The existing model states contain lower deflection `delta1`, upper deflection
`delta2`, and coil-gap deviation `u`. A toolhead-to-robot-flange sensor reads

```
xS-xR = delta1+delta2-u;
```

its second derivative is `aS-aR`. Expected acceleration from issued robot
commands can be added causally, with mismatch retained as an estimated
disturbance. In the upper-rigid layout A and R coincide, so S-to-A gives the
same physical channel. S-to-B reads only `delta1`; much of that information is
already present in `q1=k1*delta1+c1*delta1dot`. A fixed-world S position channel
would provide absolute tool acceleration but requires external metrology.

Raw numerical differentiation is unsuitable: independent 1 micrometre RMS
position noise sampled at 5 kHz gives approximately
`sqrt(6)*sigma/Ts^2 = 61.2 m/s^2` noise in a central second difference. A causal
observer or appropriately filtered estimator is necessary. Encoder resolution
alone does not establish acceleration accuracy or latency.

## What the archived native traces show

The Python review first recovers the actual recorded coil-gap measurement
from the logged observer correction, then replays the exact original observer.
The maximum original-state reproduction errors were 2.28e-10 and 9.67e-13 for
`smoke_004.mat` and `precision_matched_028.mat`, respectively. Thus the timing,
signs, command inputs, and correction convention match the native evidence.

The upper-rigid clean-force trace had a 2.323 N true-contact peak and an
estimated acceleration of approximately +1.000 m/s^2 at bump end, when the
actual acceleration was zero. Its winding-current estimation error was only
about 35 microamperes during the bump. The main problem in that trace is
acceleration reconstruction lag, rather than current-state reconstruction.

For estimator-only replay of this older 5 kHz trace (0.3 ms force filtering,
0.2 ms position delay), the best reviewed force-reconstruction peaks were:

| Channel / estimator | Peak reconstruction error | RMS reconstruction error |
| --- | ---: | ---: |
| Coil gap / acceleration random walk | 0.918 N | See JSON |
| Tool-to-robot / acceleration random walk | 0.722 N | 0.273 N |
| Tool-to-robot / explicit jerk state | 0.748 N | 0.185 N |

The explicit jerk state reduced RMS but did not by itself establish a
0.5 N peak pass at these older timings. The new direct-position measurements
are constructed from archived physical positions plus explicitly rescaled
recorded independent position noise. No surface preview enters the observer.
These are not a replacement for native closed-loop tests at the proposed
sample rate, filter latency, noise, and drive limits.

## Causal jerk-state and compliance-reference remedy

The nine-state observer models acceleration as a random walk. An explicit
surface jerk state is a natural local prediction model:

```
state10 = jS
A(8,10) = 1
A(10,10) = 0
Q(8,8) = 0
Q(10,10) = snap_process_power.
```

This assumes locally persistent jerk; it does not know when a bump starts or
ends. A colored-jerk prior is another tunable prediction model. Suggested
initial snap-power tests were 1e5 through 1e8 in the model's SI-state units.

The old controller's acceleration reference is a DC equilibrium. For the
upper-rigid compliant lower link, both deflection and deflection velocity
change when acceleration changes. Using the existing five-state synthesis
matrices `Ab,Bb,Cb`, `Eay=[0;1;0;0;0]`, and the feedback convention `-K*x`,
the continuous constant-jerk output-regulator equations are

```
M = [Ab Bb; Cb 0]
[Xay; Uay] = M \ [-Eay; -mS]
[Xj;  Uj ] = M \ [ Xay;  0 ]
command = -K*x + (Uay+K*Xay)*ahat + (Uj+K*Xj)*jhat.
```

For the reviewed native upper-rigid point `k1=19746 N/m`, `c1=60 Ns/m`,
`Kf=20 N/A`, `tau_current=0.4 ms`, the old acceleration coefficient is
-10.18168 A per m/s^2. The additional compliance jerk coefficient is about
-0.00259 A per m/s^3, approaching 0.99 A at jerk 384 m/s^3. Winding-lag-only
compensation is much smaller: `-(mB+mS)*tau_current/Kf = -0.00004`, or
0.01536 A at the same jerk. The compliance term is therefore material.
Use the discrete zero-order-held output-regulator equations when native
controller synthesis is discrete.

These equations are exact for the existing control-synthesis surrogate, not
the full nonlinear physical model: actual gap viscous friction also contains
`vR-vS`, which the five-state surrogate omits. At `b=2 Ns/m` its scale on this
bump is about 0.069 N; Coulomb friction, parameter mismatch, clipping, and
measurement reconstruction remain separate disturbances.

### Identified direct-friction omission in the rigid-lower observer

The reviewed helper's lower-rigid force row contains
`Kf*i-b*u_dot+(mB+mS)*aS`, but physical actuator-derived contact force also
contains `-F_coulomb*tanh(u_dot/v_friction)`. The measurement-force row has the
same missing direct Coulomb term. In the both-rigid limit the state input for
friction is zero because gap motion is kinematic, so the term cannot enter
indirectly through mechanical states. Current friction feedforward does not
correct the observer's measurement equation.

At the reviewed 0.2 N Coulomb level and 0.6 kg B mass, the missing term can be
interpreted as approximately -0.333 m/s^2 tool acceleration. The reconstructed
contact force then biases by approximately `-(1.4/0.6)*0.2 = -0.467 N` during
nonzero gap velocity. This explains much of the half-newton reconstruction
RMS in the older both-rigid replay. It is a model/controller implementation
defect, rather than a physical impossibility result.

For a causal native fix use only the already estimated gap velocity:

```
fr_hat = F_coulomb*tanh(estimated_gap_velocity/v_friction)
Ba(force_filter_state,known_friction_input) = -(1-filter_alpha)
estimated_true_force = Cf*xhat - double(lowerRigid)*fr_hat.
```

The known-friction column is column 2 of the three known inputs (native
observer input 4 overall). With configured force delay at least one sample,
the measurement reads the previous filter state, so no instantaneous known
input feedthrough is needed. If force delay is zero, add measurement
feedthrough `Dm(1,2)=-(1-filter_alpha)` and apply the corresponding innovation
correction; using a previous-sample friction estimate avoids an algebraic
loop. Upper-rigid layouts with a compliant lower link already measure spring
force, so their force row is unaffected by this particular omission.

Estimator replay with this correction removes much of the apparent sensing
floor in archived both-rigid `precision_matched_028.mat`: reconstructed-force
peak 0.343 N for a random-walk acceleration observer (surface process power
70, assumed force noise 0.02 N, constructed S-to-R position noise 0.1
micrometre). An explicit jerk state (snap power 1e6) gave peak 0.346 N and
RMS 0.067 N with 1 micrometre position noise. Actual recorded force noise was
0.02 N and the archived force/position delays were both 0.2 ms. These improve
the earlier roughly 0.8--0.9 N reconstruction peaks and half-newton RMS, but
still use the old fixed native current commands: they justify a corrected
native closed-loop experiment, not a qualification claim.

A fully dynamic compliant reference is another causal option:

```
c1*delta_ref_dot+k1*delta_ref = -mS*ahat
v_ref = (-mS*ahat-k1*delta_ref)/c1
a_ref = (-mS*jhat-k1*v_ref)/c1
Fa_ref = -(mB+mS)*ahat+mB*a_ref.
```

It can be implemented as a native first-order filter and arithmetic blocks.
Use state feedback about `[delta_ref,v_ref,0,0,iref]`, with estimated friction
included in `iref`. Exact current-lag inversion would require another
derivative (snap) or filtering; a fast drive can instead leave that lag to
feedback. This alternative was derived here and is not a native-tested result.

## Other physical qualification limits

The preliminary PDF calls the reference a target *normal* force; the current
plant computes axial force. The maximum profile slope is 0.34346, about
18.96 degrees, and its normal-direction cosine is 0.94577. With negligible
tangential force, 50 N axial corresponds to 52.87 N local normal force and
17.17 N lateral reaction. Measuring the force vector near contact could
recover its normal-force magnitude when tangential process force is
negligible. With grinding/friction loads, a vector magnitude alone does not
separate normal and tangential force. No numerical axial-force pass qualifies
these missing lateral loads, guide moments, distributed/compliant contact,
tilt, or changing contact-normal geometry.

The spring-length rule is a sum of spring material lengths, not total flange
height. An opposed pair on fixed seats has constant installed-length sum
`2*Lfree-2*Pcommon/ks`; actuator, seats, guides, dampers, and end clearances
remain outside that sum. Dynamic contact/spring reactions must be checked
against preload and working-compression clearances on actual histories.

The electrical current-loop model is not a thermal qualification. At 20 N/A
and 4 ohm, 50 N stationary coil force means 2.5 A and 25 W cold copper loss.
Stroke-dependent force constant, coil heating, driver voltage headroom,
temperature limits, and actual moving mass require their own physical checks.
