# Guide Coulomb feedforward study on corrected current observer, v6

The eligible physical system and v5 observer are unchanged: ideal upper
20 N/mm spring with zero damping and unlimited length/travel, externally
rigid lower mounting retaining a 64 MN/m cell, zero external damping,
budget sensing and the hot 32 V coil. V5's explicit actual selected-command
source, matched-frame current prediction and no-lead observer friction
estimate remain active.

This bank uses original PI/notch/filter **profile 1** and varies only the
controller's present-time Coulomb guide-friction compensation. It keeps
physical friction at 0.2 N, viscous friction at 2 N·s/m and physical smoothing
speed at 0.2 mm/s. The observer's physical-friction estimate remains exactly
as identified in v5. Viscous feedforward, the 400 µs guide-velocity prediction,
acceleration/geometry feedforward and all sensors/clocks remain unchanged.

| Gain ID | Controller Coulomb scale | Controller smoothing speed |
| --- | --- | --- |
| 1 | 0 | 0.2 mm/s |
| 2 | 1 | 1 mm/s |
| 3 | 1 | 2 mm/s |
| 4 | 1 | 5 mm/s |
| 5 | 0.5 | 1 mm/s |
| 6 | 0.5 | 2 mm/s |

The new coefficient fields are separate from `p.coil` so varying compensation
cannot silently change physical guide friction. Upper spring length/travel
constraints are explicitly disabled in case metadata; inherited catalog
spring-operation targets are inapplicable to this prescribed ideal link.
Actuator hard travel and force failure metrics still apply.

All 132 prepared configurations reuse the eleven original development cases
and seeds, with A=0.4/1.5 kg for each of six choices. The ordering has 22 cases
per gain, first A=0.4 cases0:10 then A=1.5 cases0:10. The live selected-command
harness and emitted-C guards are repeated on the new private graph before
sealing; harnesses are separate from force-qualification experiments.
Parent-owned bounded evaluation initially scores cases2/5/7 for each A/gain.
No fresh test outcomes, learned checkpoints or public model are used for
tuning or changed by this lane.
