# Bounded modal damping, acceleration lead and guide compensation V7

This private candidate bank combines the two selected V6 development parents
with a proper causal S-acceleration lead used only for current feedforward,
and two controller-only Coulomb-friction smoothing settings. Root owns the
native force recordings and candidate selection. Preparation and wiring
checks create no qualification episodes.

## Changes and fixed inputs

The selected V6 parents are families **17 and 18**: observer profile **33**,
modal width **0.2**, virtual damping targets **1500 and 3000 N·s/m**, and
the existing **±0.05 A** modal current bound. The modal branch still consumes
the actual acquired axial cell deviation, follows the baseline notch and
precedes the original current limit. Antiwindup still uses the total request.

The acceleration lead is the accepted native Discrete Transfer Fcn from V5:
`(1+(tauPole+tauAdvance)s)/(1+tauPole*s)`, bilinear at 10 kHz, zero initial
state and DC gain 1. Only the current feedforward acceleration path receives
it. Force reconstruction, the normal estimate and observer measurement frame
keep the original acquired signals.

| Advance / pole | High-frequency gain |
| --- | ---: |
| 150 / 400 µs | 1.375 |
| 250 / 400 µs | 1.625 |
| 350 / 1000 µs | 1.350 |
| 500 / 1000 µs | 1.500 |

For each pair, the `Guide friction velocity scale` feedforward Gain uses
either **0.2 or 1.0 mm/s** smoothing. Coulomb compensation scale remains 1.
The physical guide retains **0.2 N Coulomb force**, **2 N·s/m viscous friction**
and **0.2 mm/s physical smoothing**. Observer friction is unchanged.

The upper spring is fixed at **20,000 N/m**, zero external damping and ideal
unlimited spring length/travel; the lower link remains a **64 MN/m** undamped
finite cell. All existing physical, acquisition/noise, observer, PI/notch,
robot and guard parameter sections and prepared coefficients match the
selected V6 parent exactly. The new lead coefficients are the sole additional
prepared coefficient section.

## Identified modal timing stays fixed

The modal helper now reads the explicit controller-design field
`p.cellModal.identifiedCurrentResponse_s = 50e-6` instead of the actual
`p.coil.currentResponse`. The stored acquired complex transfer remains frozen.
Every nominal modal coefficient is exactly equal to V6. A preparation guard
changes actual current response alone by ×2 and asserts that the modal
coefficients do not change. Future driver/sensor mismatch experiments must
preserve the identified field and acquired complex transfer.

Carrier sign/phase checks remain finite-frequency evidence. V6's off-carrier
negative-damping limitation remains: this correction does not establish
broadband stability or hardware performance. Auxiliary saturation bounds
effort, not stability. The native histories must still pass the whole-record
normal-force and physical-stop metrics.

## Bank and reproduction

The 16 families are ordered: V6 parent **17/18**, lead pair in the table order,
then guide smoothing **0.2/1.0 mm/s**. Each retains the same eleven development
cases/seeds for A masses **0.4 and 1.5 kg**: **352 prepared configurations**.

Zero-based scenario index:

`176*parentIndex0 + 44*(leadID1−1) + 22*guideID0 + 11*massAIndex0 + caseID0`.

Global family/gain ID:

`8*parentIndex0 + 2*(leadID1−1) + guideID0 + 1`.

The handoff JSON under `validation/upper_force_observer_20261005/` binds the
sealed `native_cell_modal_lead_v7`, `bank_cell_modal_lead_v7`, live-source
audit, numerical source and carrier-phase guards, and independent saved-copy
acceptance. Use the unchanged local `native_api.py` / `native_bridge.c` for
scoring. Preserve all earlier studies and write later cohorts to new private
directories. Root must lock a candidate before fresh tests are executed.
