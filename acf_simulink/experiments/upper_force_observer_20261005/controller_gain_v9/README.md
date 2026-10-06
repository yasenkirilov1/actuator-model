# Final bounded live normal-PI/modal bank V9

This private bank varies the **selected normal-force PI** and bounded modal
damping on the no-lead V6 family-17 parent. Root owns development scoring and
selection. It has **12 families × 22 existing development configurations =
264 prepared cases**. The worker has executed no force-qualification episodes.

The live feedback parameters are **`p.inertial.forceKp` and
`p.inertial.forceKi`**. Their parent values are **0.02459016393442623 A/N** and
**16.393442622950822 A/(N·s)**. These correspond to the original `.03 / 20`
reference gains rescaled by `20/24.4` for the identified coil force constant.
Both live gains use the same joint multiplier:

| Joint multiplier | Live Kp, A/N | Live Ki, A/(N·s) |
| --- | ---: | ---: |
| 2/3 | 0.016393443 | 10.928961749 |
| 5/6 | 0.020491803 | 13.661202186 |
| 1 | 0.024590164 | 16.393442623 |
| 4/3 | 0.032786885 | 21.857923497 |

Each multiplier is paired with virtual modal damping **1000, 1500 or
2000 N·s/m**, width **0.2**, and the unchanged **±0.05 A** auxiliary-current
bound. Multiplier is the outer ordering and damping is the inner ordering.
**Family 8** is the complete nominal reference: multiplier 1, damping 1500.

The inactive coil/d PI aliases remain at their parent values. The selected
normal PI uses the live gains above, including the corresponding antiwindup
expression `-p.force.antiWindupRate/p.inertial.forceKi`. Its effective current
back-calculation rate remains **30/s** for every candidate.

There is no acceleration lead and the original guide feedforward smoothing
is **0.2 mm/s**. Physical stiffness/damping/friction, sensing/noise/timing,
upper observer, force reconstruction, robot commands/gates and current notch
remain unchanged. The modal design uses the frozen acquired complex transfer
and explicit **`p.cellModal.identifiedCurrentResponse_s = 50e-6`**; actual
driver-response perturbations do not retune it.

The independent emitted-parameter guard verifies all 264 rows. Only five
native fields may differ from the matched V6 case: active normal P and I,
matching antiwindup, and modal filter C and D. Every native parameter in all
22 family-8 rows exactly equals the corresponding V6 reference. Saved-copy
compilation checks every family with both A masses, with zero algebraic loops.
Carrier phase/source guards remain implementation evidence, not force passes
or broadband stability claims. The prior off-carrier damping limitation stays.

Zero-based scenario index is
`22*(familyID1−1) + 11*massAIndex0 + caseID0`.
Family ID is `3*(multiplierID1−1)+dampingID1`. A index 0/1 means 0.4/1.5 kg;
case IDs and seeds are the same eleven development cases used previously.

The final handoff binds the V9 native/model/bank/helpers/guards by hashes.
Its directory and version identify the corrected live-gain implementation;
the descriptive internal design label inherits the earlier tuning-table name.
The earlier V8 bank varied inactive PI aliases and is preserved as rejected
preparation with **zero scored episodes**, excluded from gain-performance
claims. Root selects only from development, locks the candidate before fresh
tests, and performs no further gain tuning after this bank.
