# Proper current-feedforward acceleration leads

This private bank retains two exact frozen observer families from the bounded
v4 development bank: **24** (Q densities 0.005/0.004, algorithm position RMS
2 µm, cell-derivative cutoff 1,500 Hz) and **33** (0.0005/0.0004, 5 µm,
150 Hz). They were selected using previous development recordings. Actual
sensor noise remains unchanged, including the 1 µm position assumption.

The required mechanics stay fixed: upper spring 20,000 N/m with zero external
damping and unlimited ideal length/travel; lower rigid coupling represented
by the finite 64 MN/m B–S load cell with zero structural damping. Guide
friction, current loop, drive voltage, PI/notch and robot control stay fixed.

One ordinary native Discrete Transfer Fcn is inserted between `Causal
acceleration for current FF` and `Measured acceleration current feedforward`.
It predicts current-feedforward acceleration with the proper causal filter

$$C(s)=\frac{1+(\tau_p+\tau_a)s}{1+\tau_p s}.$$

The filter is bilinear at the existing 10 kHz clock, has zero initial state,
unit steady gain and finite high-frequency gain `1+tau_a/tau_p`. It operates
only in the S-acceleration current feedforward path. Force reconstruction,
normal estimation and the upper observer continue using the original matched
measurement frame. Existing `accelerationPrediction_s` is already zero, so
there is no duplicate acceleration predictor. No future command, surface
preview or additional sensor enters this branch.

| Lead ID | Advance, µs | Pole time, µs | High-frequency gain |
| --- | ---: | ---: | ---: |
| 1 | 150 | 200 | 1.750 |
| 2 | 150 | 400 | 1.375 |
| 3 | 250 | 200 | 2.250 |
| 4 | 250 | 400 | 1.625 |
| 5 | 350 | 200 | 2.750 |
| 6 | 350 | 400 | 1.875 |
| 7 | 350 | 1,000 | 1.350 |
| 8 | 500 | 1,000 | 1.500 |

The 352 prepared configurations are ordered as observer 24 then 33, eight
leads each, eleven A=0.4 kg cases then eleven A=1.5 kg cases per family.
The zero-based scenario index is `176*observerIndex + 22*(leadID-1) +
11*massAIndex + caseID`. Observer index 0/1 selects parent 24/33; mass index
0/1 selects A=0.4/1.5 kg. `gainID` aliases the one-based lead-family index;
`controlBaseGainID` remains the original PI profile 1. The sixteen first
high-force heavy 90° probes use indices `7+22*j`, j=0…15.

Preparation asserts every existing parameter section and every existing
prepared coefficient, including the full upper observer, remains identical
to its selected parent. Only the new acceleration-lead structure and
descriptive experiment identity are added. The copied lead preparer and
installer are byte-identical to the earlier worker's sealed implementation;
the two longer-pole settings use the same formula.

The private export runs the local cell-source numerical guard and checks the
live measured cell term in generated C. The unchanged native API/bridge checks
case 0 against Coder's parameters. A separate batch process reopens the saved
graph and verifies both selected observers and both A masses: stable proper
filters, zero algebraic loops, native blocks and the lead's two exclusive
current-feedforward connections. These implementation checks and prepared
configurations are excluded from force qualification. Root owns endpoint
recordings, full native scoring, selection and any report changes.

Frozen input bank: `validation/upper_force_observer_20261005/bank_observer_lead_v5/`.
Private native export: `validation/upper_force_observer_20261005/native_observer_lead_v5/`.
Use new output directories with `prepare_and_build_observer_lead_v5.m`,
`audit_native_lead_v5.py`, `seal_native_lead_v5.py` and
`verify_force_observer_lead_graph_v5.m`; preserve existing exports and tests.
