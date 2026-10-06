# Final two-sensor physical/model review

Read-only review of the corrected **both-rigid** native controller, including
the final reconstructed-contact-force gate in
`controller_upstream_rigid_gate_worker`. Source hashes and numerical checks
are recorded in `final_two_sensor_physics_review.json`, which also preserves
the original raw-cell-gate audit. The final gate wiring is a native Gain/Goto
change; the passing observer helper is unchanged.

## Result and its scope

The reviewed fast force loop passes the model's axial +/-0.5 N requirement.
Twenty clean/noisy endpoint runs passed; the worst peak was **0.426384 N**.
The same worst case gave **0.426058 N** at 25 microsecond plant steps and
**0.426042 N** at 12.5 microsecond steps. Save/reopen reproduced all 35 recorded
signals exactly, and the native algebraic-loop count was zero. The remaining
peak-error margin is **0.073616 N**, about **14.7% of the specified tolerance**.
This is a nominal modeled pass, not hardware qualification or a broad
uncertainty margin.

The endpoint tests cover 5/50 N and 0/90 degrees, with four clean runs and
four independent noise seeds per noisy corner. Under the both-rigid,
constant-parameter equations, changing force reference and axial gravity
changes holding current by `(deltaF-2*deltaG)/Kf`; all deviations, positions,
friction, and force errors remain identical while the driver remains
unclipped. Endpoint holding-current extrema bound the interior current and
voltage demand. Thus the model permits an analytical extension to intermediate
5--50 N and 0--90 degree settings. It does not establish that real friction,
force constant, contact geometry, or sensor properties are independent of
force and orientation.

## Physical measurements and permitted inputs

The controller uses:

1. The sampled, filtered, delayed B/S load-cell signal with exact gravity tare.
2. Sampled, delayed toolhead S position relative to body A.
3. Its own clipped issued current command.
4. Coulomb friction calculated only from estimated gap velocity.
5. Expected robot acceleration from a predictor driven only by issued robot
   velocity commands.

The plant's surface acceleration enters physical load-cell construction
`Fcell_tared=Fcontact-1.4*aS`; this represents the mounting location, not an
accelerometer or preview channel supplied to the controller. Actual robot
acceleration enters the plant's upper rigid reaction and a validation log,
with no route into the controller predictor. The tool-position sensor is
constructed from physical S and A coordinates and then receives its stated
sampling, independent noise, and delay. Slow recentering uses inferred gap
state 5, not another hidden magnet sensor.

In this topology A follows the robot flange, so the permitted S-to-A channel
is equivalent to S-to-flange. The controller uses neither surface preview nor
an additional accelerometer.

## Corrected force reconstruction

For the actual 1.4 kg distal tool, exact gravity tare leaves the dynamic
inertial term:

```
Fcell_tared = Fcontact - 1.4*aS.
```

The specified bump gives a physical difference of up to **3.36 N**. When
both attachments are rigid and B+S=2 kg,

```
Fcontact = Fa + 2*(g_axis+aS)
q_cell_deviation = Fa_deviation + 0.6*aS.
```

Native history checks verify the first equation to less than 1e-10 N and
rigid attachment positions to less than 1e-10 m. The repaired observer
includes direct Coulomb friction in both its filtered force prediction and
its reconstructed contact-force error:

```
Ba(forceFilterState,knownFrictionInput) = -(1-filterAlpha)
forceErrorEstimate = Cf*xhat - Fc*tanh(estimatedGapVelocity/frictionSpeed).
```

The force measurement has one configured sample of delay, making this
correction causal without instantaneous estimated-friction feedthrough. The
native Sum subtracts estimated friction before the force-error integrator;
its sign matches the transmitted-force equation. The earlier missing term
could bias contact reconstruction by about 0.467 N at 0.2 N Coulomb friction.

Initialization also matches the measurement definitions: winding current is
`(reference-2*g_axis)/Kf`; initial S-to-A measurement is `-u0`, and observer
position normalization adds `+u0`. Exact known distal mass, gravity tare, and
zero sensor biases are assumptions of this evidence.

## Timing, noise, and drive requirements represented by the pass

The reviewed mechanical measurement channels run at **10 kHz**, each with
**known 100 microsecond delay**. Force filtering and closed current response
both have 100 microsecond time constants. The noisy runs use **0.02 N RMS
force noise** and **1 micrometre RMS position noise**, implemented as bounded
uniform white noise. These RMS values are not sensor accuracy, calibration,
resolution, synchronization, colored noise, or timing-jitter specifications.
The reviewed proof does not include nonzero force/position bias or unknown
latency.

The modeled coil has Kf=20 N/A, R=4 ohm, L=3 mH, bipolar 3.5 A current limits,
and a 24 V supply. Endpoint peaks were **2.752659 A** and **11.825937 V**;
neither current nor voltage clipping occurred. The winding-current predictor
therefore closely matches the native drive: approximately 7 microampere peak
error in the reviewed worst trace. This supports the two-sensor *outer-loop*
model without measured current readback as an observer input. A physical
current-regulated drive still needs its own electrical current feedback, and
providing drive readback to the outer observer is a useful practical option
when voltage limits, temperature, or driver mismatch invalidate the simple
current predictor. It has not been established as necessary by this pass.

The local current regulation matters: passive L/R is 0.75 ms, so the modeled
0.1 ms response requires an active current controller. At 50 N hold, cold
copper loss is approximately 25 W. Heating, stroke-dependent Kf, force
calibration, moving mass, guide friction, and thermal ratings remain physical
design and robustness checks rather than conclusions from this electrical
model.

## Slow robot gate and its remaining limitation

The final gate reads **reconstructed contact-force error**: the physical
estimated force deviation passes through Gain -1 and the global
`inferred_contact_error_N` signal into the settling gate. Its inputs remain
causal and permitted. It requires a +/-0.2 N band, filtered force rate
<=2 N/s, and 0.1 s dwell before permitting the **recenter correction**.
Programmed nominal motion continues independently of this gate.

All four clean corner cases exercise gate opening, with readiness in 86.28%
of samples and four opening transitions. The clean 50 N/90 degree history
also contains a nonzero recenter correction, so this is active slow-feedback
evidence rather than a gate-status-only test. The final native qualification
still passes all twenty corners/noise seeds, both fine-step cases, and
save/reopen.

However, all sixteen noisy corner/seed runs have zero gate readiness for the
entire four seconds, including all three reviewed worst-case histories.
Thus active slow recenter feedback is demonstrated only in the clean cases.
The nominal robot program is bounded to +/-5 mm/s with command reversals;
its predictor exactly matches the native robot dynamics in the reviewed
histories. This neither proves recenter performance with sensor noise nor
qualification against robot-model mismatch.

The original raw-cell gate was incorrectly judging the sensor's inertial
reaction rather than contact-force error. An independently reproduced
gate-only replay matches the native trend filter to floating-point precision.
Switching to corrected contact-force estimate alone still does not make the
gate open under the reviewed noise and existing 2 N/s rate limit. Usable
noise-tolerant gate settings require a further native test before claiming
active noisy recenter behavior. This limitation does not invalidate the fast
force tracking result.

Both-rigid equivalent stiffness is infinite, so legacy
`initialExcessForce/keq` and `robot.errorForce/keq` displacement inputs become
zero. Their stored 10 N/20 N parameter values do not represent executed
startup-excess or robot-error disturbances in this rigid configuration.

## Physical prototype interpretation

The present winning topology uses **rigid attachments with no modeled
springs or dampers**; it does not detach the tool or stator. The summed spring
length is zero, satisfying the requested spring-length bound. Actuator,
guides, sensor, seats, and structure still need a physical package and load
ratings. There is no passive spring design implied by this result.

The actual B/S sensor needs force reconstruction rather than direct PID on
its tared reading. Prototype measurements should preserve that sensor
location, known downstream mass, local S-to-A displacement, synchronized
timestamps, and calibrated delay. The nominal pass has insufficient margin
to treat realistic bias, current-force calibration error, latency jitter, or
unmodeled compliance as negligible. The load cell's own flexibility and
mechanical bandwidth are outside the rigid attachment equations.

Finally, the preliminary report's target normal force and this model's axial
reaction are different quantities on a sloped bump. Maximum slope is 0.34346,
or about 18.96 degrees. With negligible tangential process force, **50 N axial
corresponds to 52.87 N local normal force and 17.17 N lateral reaction**.
Consequently the numerical axial pass does not establish +/-0.5 N normal
force. A multiaxis sensor near contact can measure force magnitude when
tangential load is negligible; grinding/friction force needs a contact-normal
estimate to separate components. Guide side loads, moments, distributed
contact compliance, tilt, and changing normals remain outside the 1D model.
