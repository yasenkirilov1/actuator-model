# Force observer with bounded guide compensation, v7

This lane combines force-observer parents 24 and 33 from the accepted,
explicitly wired `native_observer_noise_v4` with six controller-only Coulomb
guide compensation choices. Parent24 uses design densities 0.005 N/√Hz and
0.004 m/s²/√Hz, assumed position RMS 2 µm and a 1.5 kHz cell derivative.
Parent33 uses 0.0005/0.0004 design densities, 5 µm algorithm RMS and a 150 Hz
cell derivative. These are observer tuning weights; actual sensor noise and
acquisition remain fixed.

For each parent the guide profiles are: Coulomb compensation off; full at
1/2/5 mm/s smoothing; half at 1/2 mm/s smoothing. Viscous feedforward and the
400 µs guide-velocity prediction are unchanged. Physical guide friction is
still 0.2 N Coulomb, 2 N·s/m viscous and 0.2 mm/s smoothing. The force observer
and normal reconstruction are unchanged. Both links are fixed: ideal upper
20 N/mm, zero damping, unlimited length/travel; externally rigid lower
mounting retaining a 64 MN/m cell with zero damping. Upper catalog-operation
screens are explicitly disabled; actuator travel limits still apply.

There are 12 families × two A masses × eleven paired development cases,
giving **264 development configurations**. Scenario0 is an **unscored
preparation sentinel** that preserves every original emitted Coder case0
parameter and the existing native-bank equality guard. Development scenario
is `1 + 22*(familyID-1) + 11*massAIndex + caseID`; families1:6 use parent24 and
families7:12 parent33. The same991000+caseID seeds and physical cases are reused.

The accepted emitted C, generated headers and native context computation are
reused byte for byte. The unchanged API/bridge is relinked with the new
parameter bank; no native model code is regenerated and no runtime parameter
patch or ABI is added. Every mapped C field is checked exactly against its
source parent row, allowing only `Guidefrictionvelocityscale_Gain` and
`GuideCoulombfrictioncurrent_Gai` to differ. The sentinel must match **all**
original Coder fields. The private saved SLX changes only those two Gain
expressions to separate controller choices from physical guide parameters.

The acquired-cell source guard is rerun on the private graph, with restored
connections/loggers/preferences before export. Saved-graph updates check
zero algebraic loops for both parents and both A masses. These short source
harnesses are separate from force qualification. Parent-owned evaluation
selects endpoint indices; preparing and sealing the bank creates no force
qualification recording or public-model promotion. Older sources and seals
remain untouched.
