# Inverse-initialized policy replay rejection

The 8404 attempt passed its initial 28-case selection with 0.397905 N worst force error and a shared score of 0.860423. It then executed and retained a 4,096 × 24 rollout, or 98,304 native decisions. The behavior-replay guard rejected this batch before any optimizer update.

The frozen CUDA sequence replay differed from the stored C actor by at most 0.00016056 in current latent mean, corresponding to 0.3155 mA in requested current. The action log-probability mismatch reached 0.03558, exceeding the unchanged 0.005 guard. CPU replay also failed, so this was not a CUDA-only discrepancy.

The policy's inverse initialization has actor readout weights up to 662.11. Both recurrent-state rounding and cancellation in the large readout contribute: a single step from its exact stored C state still differs in current latent mean by up to 0.00010686 on CUDA; CPU sequence versus serial replay differs by up to 0.00016785. At identical reconstructed hidden values, float32 versus float64 readout arithmetic differs by up to 0.00008426.

The guards and policy implementation were preserved. Evaluation of fixed weights in the native C path and numerical acceptability of a PPO training replay are separate checks. The later 8405 experiment starts from the earlier accepted policy and uses the new episode-peak objective. No outcome from this rejected batch was used to update weights.

`analyze.py` reproduces the diagnosis from the retained rollout, frozen policy and checkpoint. `results.json` contains exact source hashes, CPU/CUDA comparisons and the arithmetic split. The split using stored C recurrent states still calculates encoder and highway arithmetic with Torch; it is diagnostic evidence, not a complete decomposition of numerical error. It uses no held-out records or new native simulation.
