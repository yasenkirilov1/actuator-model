# Episode-running force-peak reward acceptance

The optional `--force-running-peak-weight` defaults to zero. With weight `w`, each decision receives the additional reward

`−return_scale × w × (new_episode_peak² − previously_charged_peak²) / force_error_scale_N²`.

There is no `dt` multiplier. The first decision starts from a previously charged peak of zero and includes the initial native sample. The undiscounted increments telescope to the episode peak cost. PPO still applies its configured discount and GAE traces, so its optimization objective is not identical to whole-bank worst-case selection. The native true force is used only for reward; actor and critic observations remain unchanged.

The already-retained episode peak and step counter preserve this cost across rollout boundaries and exact resume. Resume reconstructs the native trajectory from saved requests and checks the restored episode peak. Episode completion resets the accumulator through the existing reset path.

The acceptance checks passed:

- Two full native training recordings, each with 600,001 samples beginning at time zero, matched the telescoped cost to within 1.11e−16.
- Six short-horizon episodes exercised reset and truncation. Their float32 training-return residual was at most 5.85e−10.
- A resume in the middle of an episode reproduced all subsequent rollout arrays exactly; an intentionally corrupted carried peak was rejected.
- Weight zero reproduced every collected array from frozen v5 exactly across two rollouts and episode resets.
- A deliberate contact-loss test retained all 2,350 native samples and matched the peak cost to within 1.82e−12. This is a failure-stop fixture, not a successful controller recording.
- One isolated CUDA update consumed 8,192 decisions with the new cost, a 5 ms force trace and the unchanged 300 ms robot trace. It produced finite changed parameters and included 44 eligible robot actions. This is an integration smoke test, not a selected policy or a training experiment.

`results.json` and `results_extra.json` bind the implementation hashes. The source snapshots, test scripts, requested actions and native traces remain in this evidence directory. No held-out outcome was accessed.
