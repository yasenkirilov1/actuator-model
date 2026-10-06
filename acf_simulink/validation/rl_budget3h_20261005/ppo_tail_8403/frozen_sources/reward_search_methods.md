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

Selection uses physical failures, a robot progress design screen, force passes,
worst force error and mean force RMS, in that order. The progress screen applies
only to the 12 nominal zero-path selection cases with at least 0.5 mm initial
stroke error. A candidate must achieve at least half the baseline's absolute
initial-to-final stroke-error reduction. Driven stress cases are excluded from
this screen. This is an engineering comparison criterion, not a user-prescribed
numerical requirement.

All candidate coefficients, objectives and per-case metrics are recorded.
Completed physical failures are replayed deterministically with full native
histories; these recovery recordings are counted separately from training.
Checkpoint exports are published after all files are written. A `STOP` file in
the run directory requests completion after the current generation and final
audits. A partial interrupted generation is not counted as a completed update.

The first direct search used the prior 24/8 development bank and stopped after
17 completed generations to move compute to the expanded bank. Generation 15
reduced force error by suppressing robot movement and is excluded from the
joint control recommendation. Generation 10 retained useful robot movement and
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
edge and sustained-drift training cases to establish when correction is needed.
The physical stop remains a failure regardless of the soft recentering target.
