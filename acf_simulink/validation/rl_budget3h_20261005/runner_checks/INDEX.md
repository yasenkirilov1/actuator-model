# Runner evidence

These are implementation checks and isolated smoke runs. Final model selection and held-out physical-model scoring belong to the root experiment, not this folder.

| Check | Recorded result |
| --- | --- |
| Causal feature cache | Bit-identical to the original 40-feature scaling. |
| Optimized next-state values | Bit-identical to independent per-tick cloned inference, including truncations with a nonzero critic. |
| Native checkpoint replay | Exact requested actions reconstruct the native observations; carried neural state, RNG and the subsequent rollout match. CUDA resume also executed successfully. |
| Failure retention | Every physical failure saves scenario, seed, exact requests and metrics; representative full native replay matched. |
| Vectorized collection | Same observations, requests, rewards, values, recurrent states and native metrics; 1.402× measured speedup on the recorded 16-environment 10 kHz comparison. |
| GPU PPO | Identical real `[4096,24,40]` batch and initial weights, with matched minibatch permutations: CPU two-thread median 0.55518 s versus CUDA 0.18397 s including transfer, a 3.018× speedup. Final parameter difference was at most 1.44e-9. |
| Deadline | A three-second smoke budget completed gracefully in 2.533 s, retaining its atomic checkpoint and final detailed rollout. |
| Multiscale credit | Disabled/equal force traces reproduce the original update exactly. A 100 ms delayed reward keeps robot credit at −0.6483 while the 20 ms force trace gives −0.00610. Critic targets and robot advantages remain unchanged. |
| Soft travel band | Cost is zero through 7.5 mm. A constant 9 mm offset produces the expected shared score 4.395 in the analytic fixture. The same 100 µs native interval-peak proxy is used across actor cadences. |
| Native band smoke | The 128-case bank and ES generation 10 warm start ran 8,192 CUDA decisions, then passed both selected boundary recordings with 0.333261 N worst error. This is a smoke result, not the final test. |
| Episode force-peak cost | Optional weight defaults to zero and preserves frozen-v5 collection bit-for-bit. Two full 600,001-sample native histories, a deliberate contact-loss fixture, reset/truncation and exact resume verify telescoping through startup and episode boundaries. One CUDA integration update checks the new reward with a 5 ms force trace. |

The historical centering-fraction guard was checked against a deliberately idle robot and worked as specified. It was superseded by the user's 7.5 mm soft-band preference and is optional only for reproducing that earlier experiment.

The current physical action mapping is `Ieq + 2*tanh(current_mean)` amperes and `0.010*tanh(robot_mean)` metres per second, followed by the unchanged native gate, sign/amplitude envelope and delayed command path. Causal observation normalization of robot quantities stays at 0.005 m/s. Exports and checkpoints record the physical mapping; exact resume rejects an incompatible mapping. A policy-only warm start retains normalized neural weights and records its new runtime mapping separately.

Use `--force-lambda-time-s .02` for the optional shorter force trace. The band objective is enabled with `--acceptable-travel-radius-m .0075 --travel-excess-weight 10 --inside-travel-weight 0`. Its running penalty uses squared excess of each native interval's peak travel; it is explicitly not an excess MSE. Selection uses the common 100 µs proxy and the shared force/band score after physical failures and force-pass count. Excursions beyond 7.5 mm are a soft preference, not a new physical failure stop.

`runner_checks_manifest.json` binds the main result files and owned source bytes. The two preparation rejections are retained separately and contribute no optimizer updates. Detailed rollout storage is bounded to the first, periodic and final rollout; completed-episode scoreboards and every failed episode's exact requests are retained.

The subsequent read-only analyses are separate from these acceptance checks. `ppo_diagnostics_v1/README.md` and its SVG compare the three completed band-aware PPO lanes against their own warm starts; each lane retained update zero under the shared selection rank. `credit_diagnostics_v1` estimates the local reconstructed-force response to randomized training current and shows why force and delayed robot actions need different credit timescales. Its proposed 5 ms force trace was not executed. Both analyses retain their source hashes and use no held-out outcomes.

After those diagnostics, `running_peak_v1` accepted the opt-in `--force-running-peak-weight` objective. Its cost increments have no time multiplier and sum to the squared maximum normal-force error over the episode, including startup. This option changes reward only. The isolated acceptance update is separate from any later root-owned training lane using this objective.
