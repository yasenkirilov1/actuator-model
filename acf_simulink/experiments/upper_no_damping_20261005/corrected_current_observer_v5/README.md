# Corrected issued-command upper observer, v5

This private comparison uses the fixed eligible mechanics: robot–A has an
ideal 20,000 N/m spring, zero damping and unlimited spring length/travel;
B–S has no external spring or damper but retains the physical 64 MN/m
load-cell stiffness. The coil retains its 25 mm travel limit and internal
guide friction. The public model and every previous evidence file remain
unchanged.

The earlier private v3 upper observer consumed a root-local `actuator_command`
tag from inside a nested subsystem. That source was grounded in generated C,
so the resulting upper-compliant recordings are rejected implementations,
not mechanical qualification. The original files are retained for auditing.

V5 branches **the same root `Force controller/1` port that drives the physical
actuator** through explicit Contact force measurement Inport3 and Head
motion estimator Inport2. It preserves the strictly proper 50 µs identified
current lag, known holding-current initialization/subtraction, identified
force constant, matched existing acquisition frame and no-lead physical
guide-friction estimate. No actual plant current, actuator force, A motion
or surface profile enters this observer. The four-state observer and six
original PI/notch/filter profiles are unchanged. The linear current predictor
still approximates electrical clipping, and A–S gap velocity approximates
A–B guide velocity across the finite sensing cell.

Preparation reuses all 132 development configurations: six profiles × two
A masses (0.4/1.5 kg) × eleven physical cases, with identical seeds and
budget/hot-coil inputs. It adds no new physical test cohort or training.
The original profile/case ordering is retained: each gain has 22 entries,
with A=0.4 cases0:10 followed by A=1.5 cases0:10.

`verify_issued_command_source.m` runs a short wiring harness at the known
holding request and ±20 mA. External baseline-selector mode0 exercises the
actual native selected command through notch/limits. At every logged native
point, the active root drive command must equal the lag input within 1e-12 A,
and the predicted current must respond to both perturbation signs. Temporary
loggers and private SDI preferences are restored before saved export.
`assert_issued_command_chain.m` then checks each actual port connection.
`verify_emitted_current_input.m` rejects generated C if the lag input is
grounded or lacks a live selected-command variable. These harness runs do
not count as force-qualification episodes.

Run preparation/build with a private `MATLAB_PREFDIR`,
`matlab -licmode onlinelicensing -batch`, then `seal_native.py` on the new
native directory. Model/helper/configuration/C/library and guard hashes are
sealed together. Parent-owned bounded endpoint evaluation supplies the
physical results; generated scripts or successful preparation alone establish
no force pass. A selected candidate needs independent MATLAB parity and
closed-loop physical-step refinement before any promotion.
