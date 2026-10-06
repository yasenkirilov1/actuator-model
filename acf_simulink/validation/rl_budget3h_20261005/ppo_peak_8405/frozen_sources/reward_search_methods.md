# Reward-only evolutionary search

This lane optimizes one two-output policy against complete native Simulink
trajectories. It does not regress baseline actions or use supervised labels.
The external policy input is the same 40 causal measurements and identified
parameters used by PPO. The policy's eight additional basis features contain
only products of those permitted inputs. The physical graph, sensor filters,
current servo, robot gate, command limits and delayed queue are native.

The policy uses the common `UnifiedPolicy` / Puffer minGRU implementation and
normal portable checkpoint format. Initial direct coefficients follow static
force balance: negative total moving-mass acceleration compensation, positive
axial-minus-normal correction and negative measured force feedback. These
coefficients are explicitly physics initialization, not a learned result.
The first search learns 12 direct coefficients. Its minGRU readout is zero;
that first stage must not be described as learned recurrent control.

The second search initializes from the useful first-stage checkpoint and also
learns 16 paired current readout coefficients from the minGRU history. Positive
and negative state projections share gates, so paired differences cancel the
common startup of the positive minGRU state. Four input bases have memories of
0.1, 0.4, 3 and 20 ms. The full network remains trainable by PPO. In this ES
stage the robot request is a learned direct recentering coefficient; PPO can
also learn the robot head's recurrent connections.

`reward_search_native.c` runs a complete episode by calling the accepted
Simulink-generated library and the exported policy, avoiding a Python callback
at every 100 microseconds. It contains feature scaling and policy calls, but no
physical integration. Feature parity is exact. Independent Python-loop replay
checks retain the actual numerical differences from float32 tanh evaluation.

The optimizer uses antithetic proposals and standard rank-mu covariance
adaptation. It ranks whole-episode measured objectives from every physical
integration interval, including peak actual normal-force error and failure
stops. Policy observations never contain these truth-only reward signals.
The expanded study rotates stratified common 12-case minibatches drawn from
all 48 training cases. Each candidate in a generation receives the same cases
and sensor seeds. All 16 selection cases are evaluated separately; the final
32-case holdout is not used here. Minibatch objective values are not directly
comparable across generations; selection results use the same full cases.

The historical second run used physical failures, a robot progress design screen, force passes,
worst force error and mean force RMS, in that order. The progress screen applies
only to the 12 nominal zero-path selection cases with at least 0.5 mm initial
stroke error. A candidate must achieve at least half the baseline's absolute
initial-to-final stroke-error reduction. Driven stress cases are excluded from
this screen. This is an engineering comparison criterion, not a user-prescribed
numerical requirement.

All candidate coefficients and objectives are recorded. The first direct run
retained aggregate training objectives and the best candidate's metrics; its
two physically failed candidate vectors were subsequently replayed exactly.
Later runs retain every training candidate's per-case metrics. Historical
selection events retained only the winning proposal's per-case metrics even
though both the generation candidate and distribution mean were executed.
The sustained-motion run retains both proposals. The ledger labels missing
historical counters explicitly rather than assuming they completed normally.
Completed physical failures are replayed deterministically with full native
histories; these recovery recordings are counted separately from training.
Checkpoint exports are published after all files are written. A `STOP` file in
the run directory requests completion after the current generation and final
audits. A partial interrupted generation is not counted as a completed update.

The first direct search used the prior 24/8 development bank and stopped after
17 completed generations to move compute to the expanded bank. Generation 15
reduced force error by suppressing robot movement and was excluded under the
then-active center-to-zero criterion. Generation 10 retained useful robot movement and
is the warm start for the expanded search. Earlier held-out cases become
explicit development cases for this new study; fresh holdout cases remain
separate.

## User update: accepted stroke band

The user subsequently accepted positions within **±7.5 mm of stroke center**
without significant recentering punishment. This supersedes the center-to-zero
cost and baseline-relative progress screen above. Those runs are preserved as
historical experiments. Inactivity inside the accepted band is a valid result;
generation 15 is not rejected simply for suppressing unnecessary robot motion.
The next version penalizes only excursion outside the accepted band and adds
edge and programmed-drift training cases to establish when correction is needed.
The physical stop remains a failure regardless of the soft recentering target.

The 128-case band bank uses a bounded 0 / +velocity / -velocity / 0 robot
program with changes at 1, 2 and 3 seconds. These are programmed pulse/reversal
cases, not sustained drift. Any sustained-drift extension must have its own
executed-case identity and evidence.

## Current recurrent band search

The next two stages preserve the same Puffer network and evolve **55
independent coefficients**, rather than claiming to evolve all 8,101 stored
trainable neural parameters. Those coefficients control paired current-history
readouts, robot force-history readouts, a smooth robot-band feature, direct
causal measurement gains, recurrent update-gate time constants and identified
spring/damper context projections. Both outputs share the recurrent trunk.
All remaining neural weights stay fixed in this evolutionary lane; the
parallel PPO lane may train the complete network.

The robot-band feature is an explicitly disclosed neural initialization:
opposed travel projections with a negative common bias and paired readouts
produce small corrections near the center and increasing corrections near the
accepted band. It is made from ordinary trainable encoder, minGRU and readout
weights. A direct projection of already-issued total robot velocity minus
issued correction provides causal nominal-motion compensation. No future
motion, surface profile, teacher action or separate dynamic baseline controller
enters the actor.

The raw robot head maps to **±10 mm/s**, before the unchanged native
stability gate, stroke-dependent command envelope, speed cap and delayed queue.
This permits a correction to oppose a sustained 5 mm/s outward nominal motion;
the earlier ±5 mm/s raw range could at best cancel that velocity.

Selection ranks physical failures first, then force-pass count, then a shared
soft-band score, then worst force error and mean force RMS. The score is

\[
J=(e_{\max}/0.45\,\mathrm{N})^2
 +0.02\,\operatorname{mean}(e_{\mathrm{RMS}}/0.05\,\mathrm{N})^2
 +\operatorname{mean}\left[\frac{\int b(t)^2\,dt}{T(1\,\mathrm{mm})^2}\right]
 +0.5\,\operatorname{mean}(b(T)/1\,\mathrm{mm})^2,
\]

where \(b(t)=\max(|x(t)-x_{\mathrm{center}}|-7.5\,\mathrm{mm},0)\).
The integral uses the maximum physical excursion within each 100 µs interval
as a conservative scoring approximation. One complete native positive-excess
recording verifies the proxy and shows only 0.0178% additional integral. Force
peaks still inspect every native integration interval. The 0.45 N denominator
is a preferred design margin; the user's hard force requirement remains ±0.5 N.
Starting outside the soft band is penalized through excursion and recovery time,
not classified as a physical failure. Native contact loss and travel stops are
always failures.

The 144-case bank preserves the original 128 cases exactly and adds 12 training
and four selection cases with **sustained** nominal velocity from time zero.
It contains 84 training, 28 selection and 32 untouched final cases. Each
12-case evolutionary minibatch includes two early cases, four known-mechanics
cases, one calibration stress case, three bounded motion cases and two sustained
motion cases. The search rotates through every training identity; each selection
event evaluates all 28 cases. No final case is used for optimization or selection.

`reward_search_ledger.py` audits completed optimizer work, selection, initial
and final audits, native failure recovery, and checkpoint ancestry separately.
Planned generations, interrupted unscored work and replay recovery are excluded
from optimizer-update counts. Counts from warm-start ancestors are subsets of
the executed study ledger and must not be added a second time.

## Current-output refinement lane

A parallel local search initializes from the selected sustained-motion policy
and varies **41 coefficients** with initial normalized search standard
deviation 0.05. It fixes every robot-head coefficient, the two robot-band
encoder units, their gates, and all force-history gates used by the robot head.
The evolved subset contains 15 paired current readouts, three direct measured
current gains, seven physical-product gains, one current bias, 11 current-only
history gates and four identified spring/damper context projections. A random
perturbation of all 41 coefficients gives exactly identical robot outputs on
750 identical sequential synthetic observations while changing current output.
The closed-loop robot trajectory can still change because improved current
control changes the measurements presented to the fixed robot mapping.

Its rotating 12-case training batch places more weight on difficult force
conditions: three calibration stress cases, five full-sharp mechanics cases,
two bounded-motion cases and two sustained-motion cases. It still audits all
84 training cases initially and finally, and uses the same 28-case selection
score as the joint-output search. The independent run retains its own seed,
source checkpoint, parameter specification, snapshots and counters.
