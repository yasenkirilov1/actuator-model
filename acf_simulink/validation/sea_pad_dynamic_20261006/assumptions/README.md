# Actuator and soft-pad assumptions

This is a source-bound engineering input to the new dynamic study, **not an
executed simulation or hardware qualification**. It uses the first-prototype
scope: one-axis B–S force sensing plus S acceleration, sanding drag ignored,
and total axial pressing force only. No local-pressure result is generated.
The study's prescribed upper spring and rigid external B–S connection stay as
specified by the user.

`source_bound_assumptions.json` contains the reviewed values, dates, exact
manufacturer URLs, formulas and distinctions between specifications and
assumptions. `build_assumptions.py` reproduces the compact JSON and static
sizing calculations. The retained EC-i catalogue PDF is hashed in that JSON.

## Motor and screw

The adopted reference is the [maxon RE35 273757](https://www.maxongroup.com/maxon/view/product/motor/dcmotor/re/re35/273757):
48 V brushed DC, 75.8 mNm/A, 4.68 Ω, 1.29 mH, 65.4 gcm² rotor inertia,
340 g body mass, 104 mNm catalogue continuous torque and 1.42 A catalogue
continuous current. Its ordinary armature-current convention matches a
two-terminal DC model. It is NRND; it establishes a reasonable motor size,
not a purchasing recommendation.

With a 10 mm lead and 90% efficiency used **for static sizing**, 50 N requires
1.166 A and the continuous torque corresponds to 58.81 N, before additional
losses. At 5 mm those values are 0.583 A and 117.62 N. These are capacity
calculations, not predicted closed-loop errors. Including the assumed
0.5 × 10⁻⁶ kg·m² screw/coupler inertia gives 2.779 kg reflected inertia at
the **independent nut coordinate** for 10 mm lead and 11.117 kg for 5 mm.
That inertia must not be placed directly on B as if a series spring were a
rigid link.

The smaller RE25 remains a historical comparison. The
[EC-i30 539489](https://www.maxongroup.com/maxon/view/product/539489) is a
lower-inertia future option, with 55.4 mNm/A, 15.5 gcm² rotor inertia and
103 mNm continuous torque on the current product page. Its linked 2025
catalogue instead lists 104 mNm, 1.84 A and 1.37 Ω; the live page lists
103 mNm, 1.82 A and 1.39 Ω. Use one revision consistently. Maxon explains
that its EC catalogue values apply to
[block commutation](https://support.maxongroup.com/hc/en-us/articles/360006288554-Strength-of-BLDC-EC-motor-with-sinusoidal-commutation).
They cannot be silently relabelled as bus current or unchanged FOC phase
amplitude. This EC motor has **not** been simulated by this assumptions task.

A 3.5 A driver limit is an assumed transient development limit. It is not the
motor's continuous rating or a manufacturer-established safe peak duration.
Score voltage clipping and RMS heating separately. A 50 µs current-response
target also needs adequate L·di/dt headroom; it does not follow from the
passive L/R constant. Catalogue inductance is measured at 1 kHz, and
[maxon notes](https://support.maxongroup.com/hc/en-us/articles/360013761160-Motor-data-and-simulation)
that effective PWM inductance may differ. The no-load-current point at high
catalogue speed does not identify low-speed stiction or a unique
Coulomb/viscous loss split.

The [THK ball-screw FAQ](https://www.thk.com/jp/en/products/faq/ball_screw/-/0089/modal.html)
supports efficiency above 90% in both directions for ball screws. That is
not a specification for a plastic-nut or trapezoidal sliding leadscrew.
Use reversible pitch conversion and explicit passive losses in dynamics,
with separate axial support bearings. The 0.075 kg translating nut/carriage
and 0.5 × 10⁻⁶ kg·m² screw/coupler inertia remain declared engineering
assumptions. An 8 mm diameter, 100 mm long solid steel shaft alone would be
about 39.46 g and 0.316 × 10⁻⁶ kg·m²; actual thread roots, journals, coupling
and shaft length change that estimate.

## Pad properties and working range

Mirka publishes a [77 mm, 5 mm interface](https://www.mirka.com/cs-CZ/p/Interface-77---6-GRIP/?variant=94118)
and a [77 mm, nominal 10 mm interface](https://www.mirka.com/en-ca/p/Interface-77---NONE-GRIP/).
These sources establish realistic geometry. They do not establish dynamic
stiffness or damping.

The adopted representative cases use **20 N/mm at 10 mm thickness**, and
**50 or 100 N/mm at 5 mm thickness**. At 50 N, flat uniform compression would
be 2.5, 1.0 and 0.5 mm, respectively. The declared validity guard is 50%
compression of pad thickness. Beyond it, extrapolating a linear material
law is not a valid working-range result.

[Rogers PORON 4701-30, publication 17-006](https://www.rogerscorp.com/-/media/project/rogerscorp/documents/elastomeric-material-solutions/poron/english/data-sheets/17-006-poron-4701-30-very-soft.pdf)
provides a bulk-material plausibility check: its 240 kg/m³ grade has
7–35 kPa compression-force deflection at 25% compression, tested at
0.51 cm/min, and 4.78–12.7 mm stock thickness. Applying those bulk values
to an assumed full 77 mm disk gives secant stiffness of 26.08–130.39 N/mm
at 5 mm, or 13.04–65.19 N/mm at 10 mm. These are quasistatic estimates for
a different material, not measured Mirka properties or a local-pressure
evaluation.

Rogers' [ideal battery-pad curves, publication 17-388](https://www.rogerscorp.com/-/media/project/rogerscorp/documents/elastomeric-material-solutions/poron/english/data-sheets/17-388-ideal-cfd-curves-for-battery-pad-applications-poron-4701-30-very-soft.pdf)
illustrate stronger nonlinearity beyond the middle compression range. The
240 kg/m³ curve lists 21.6 kPa at 25%, 42.1 at 50%, 68.3 at 60% and 149
at 70%. It concerns thinner battery-pad construction and supplies no dynamic
damping coefficient. It is an example, not an identified sanding-pad law.

The c = **0, 40, 80 and 150 N·s/m** cases are explicit unvalidated damping
sensitivities. A constant Kelvin–Voigt coefficient can exaggerate damping
at high frequency, so retaining the zero-damping comparison matters. Fit
installed dynamic stiffness, phase and amplitude dependence before treating
any value as a prototype specification. For a finite footprint, apply
unilateral contact before summing total force; clipping a spatially averaged
height can conceal partial contact.

## Internal spring geometry

The 3.5, 5 and 10 N/mm net opposed-pair cases use 1.75, 2.5 and 5 N/mm
individual rates. The length sketch has two 50 mm free springs, nominally
installed at 25 mm each, and assumed individual limits of 8–42 mm. This
gives 100 mm combined free length, ±17 mm relative travel, and signed net
force limits of ±59.5, ±85 and ±170 N. Individual preload is 43.75, 62.5
and 125 N; the highest-rate case reaches 210 N in one spring at its limit.

Those calculations are **length and force bookkeeping**, not a verified
coil-spring design. Actual solid height, stress, fatigue, guide buckling,
seats and frame compliance remain to be checked. In particular, an 8 mm
minimum installed length cannot be claimed feasible simply from rate and
free length. These springs are internal to the nut–B actuator, without
replacing the prescribed upper spring or the rigid external B–S link.

The entire assumptions folder is bounded to 200 kB retained and 300 kB
temporary storage. It creates no experiment arrays and ran no simulations.
