# Unified recurrent controller: runtime and interface checks

These records test software architecture, gradients, episode semantics and
portable inference with **synthetic inputs and rewards**. They are not a
flange-performance experiment or a physical-system qualification.

The runnable runtime is
`/home/tommie/Pesten/puffer4-native/.venv/bin/python`: Python 3.12.12,
PufferLib 4.0.0 pinned to `c5d3c637446047a6efbcaa74c039c5295d201ab0`,
Torch 2.13.0+cu130 and NumPy 2.5.1. CUDA is available on the RTX 3090.
Pesten and its environment were read only. The default installation has no
`pufferlib._C`, Gym or Gymnasium; its native trainer is not directly callable.

## One policy

`UnifiedPolicy(obs_dim, hidden_size=32, num_layers=2, dt=1e-4)` uses one shared
Puffer MinGRU trunk, two continuous action means and one recurrent critic.
The synthetic 32-input instance has 7,493 parameters. An ordinary Linear bias
on each genuine Puffer cell initializes its update gates to time constants
spanning 0.3–300 ms. The encoder uses tanh and the cells retain Puffer's
positive candidate and highway connections. There is no PI, teacher-action
fit, separate robot actor or residual fast controller.

The caller provides fixed scaling of current causal sensor/observer signals,
known identified parameters and command-clock state. Future bump/profile,
future motion, actual plant states and actual normal force are not actor
inputs. Training rewards may use actual normal force from the simulation.
The critic receives the same causal features and recurrent trunk as the actor.

The first normalized action requests full coil current around a known static
equilibrium offset. The second requests robot recenter correction. Physical
current limits, the 100 Hz robot clock, force-stability gate and the existing
delay-aware sign/amplitude envelope remain external. The robot likelihood
and entropy are masked at ineligible command events. A closed gate never
clears recurrent history or the already-issued command queue.

## Policy and PPO contract

Policy tensors use observations `[T,N,D]` and hidden `[L,N,H]`.
`step(obs, state, reset_before, action_mask, deterministic=False)` returns
normalized actions, sampled latent actions, log likelihood, latent entropy,
critic value, means and the next hidden state. `sequence(obs, initial_state,
reset_before)` supports arbitrary carried state and independent episode
reset masks through an associative affine prefix scan.

`PPOTrainer(policy, PPOConfig(...)).update(RolloutBatch(...))` performs real
on-policy reward PPO. Store sampled **latent requests**, their behavior
log likelihoods and values before external action mapping. The default
physical clock is 100 µs, chunk length 2,048 (204.8 ms), and gamma and lambda
are `exp(-dt/1s)` and `exp(-dt/0.1s)`. They are configurable in seconds.
Long rollouts use carried behavior states plus configurable recent-context
burn-in under updated weights; this is truncated recurrent PPO, not a claim
to backpropagate through the entire physical recording. Sequences retain
time order, and training does not shuffle individual adjacent frames.

`terminated` represents a true physical terminal such as contact loss or a
travel stop and bootstraps zero. `truncated` represents a recording time limit
and bootstraps the terminal observation with the corresponding next hidden
state **before resetting**. Both flags cut GAE between episodes. A rollout
or training-chunk boundary is neither flag. Only a new physical episode sets
`reset_before`. The caller must retain unsuccessful physical runs and rejected
software attempts and separate training, selection and untouched test identities.

## Portable CPU interface

`export_policy(policy, directory, feature_names, provenance)` writes JSON,
NPZ, a flat little-endian float32 buffer, a Torch checkpoint and C99 source.
The C weights pointer is mutable between PPO rollouts; shape is fixed at
export. All encoder, recurrent, actor and critic affine biases are exported.
There is no hidden running observation-normalization state.

The flat order is encoder weight/bias; each recurrent layer weight `[3H,H]`
and bias `[3H]`; actor weight `[2,H]` and bias `[2]`; critic weight `[1,H]`
and bias `[1]`; then two log standard deviations. JSON offsets specify every
array. The C header also declares `ACF_RL_LOG_STD_OFFSET`.

```c
void acf_unified_policy_step_many(size_t batch, const float *weights,
    const float *observations, float *states, const unsigned char *resets,
    float *latent_means, float *values);
```

C arrays are observations `[N,D]`, state `[N,L,H]`, means `[N,2]` and values
`[N]`; state updates in place. Clone state before a bootstrap-only evaluation.
The ctypes argument types, in order, are
`[c_size_t, float_pointer, float_pointer, float_pointer, byte_pointer, float_pointer, float_pointer]`,
where the byte pointer uses `c_ubyte` and the float pointer uses `c_float`.
Compile with `cc -O2 -std=c99 -fPIC -shared policy_inference.c -lm`.
`numpy_sample` handles seeded CPU Gaussian sampling and tanh likelihoods;
`numpy_evaluate_latent` reproduces Torch likelihoods. Preserve the NumPy RNG
state for resume. Deterministic native acceptance uses `tanh(latent_mean)`.

## Recorded checks

The complete numbers and source hashes are in `results.json`.

| Check | Maximum error or outcome |
|---|---:|
| CPU/CUDA serial versus affine scan, action mean | 8.39e-9 |
| CPU/CUDA carried/reset hidden state | 2.81e-6 |
| Float64 serial versus scan gradients | 7.11e-15 |
| Changing future observations changes earlier outputs | Exactly zero |
| C mean versus Torch over 1,200 frames, seven rows | 2.33e-9 |
| C critic/hidden versus Torch | 1.79e-7 |
| NumPy versus Torch action log likelihood | 4.77e-7 |
| Disabled robot head and std gradients | Exactly zero |
| Failure bootstrap under poisoned next value | Exactly unchanged |
| Time-limit terminal-observation bootstrap | Verified |
| Synthetic reward PPO on CPU and CUDA | Trunk, both actor outputs, std and critic update |

The component benchmark measured about 10.7 µs for one C policy evaluation,
331,924 policy evaluations/s at N=64, or 268,975/s including CPU sampling.
The N=64 batch tick is about 238 µs; it advances 64 simulated controllers,
so this is training throughput rather than a real-time claim for 64 physical
controllers. Single-controller C plus NumPy sampling measured 47.6 µs.
CUDA parallel forward/backward on `[2048,32,32]` measured about 3.95 million
samples/s with 0.584 GB peak memory. These component measurements exclude
the physical environment, acquisition, hard gates and command queues.

The architecture and probe sources are frozen in `frozen_sources/` and bound
by `dependency_manifest.json`. The portable probe weights are intentionally
untrained and remain marked `exported_unqualified`.

Reproduce from the repository root:

```bash
/home/tommie/Pesten/puffer4-native/.venv/bin/python \
  acf_simulink/validation/rl_puffer_probe_20261005_final/frozen_sources/policy_checks.py \
  --output acf_simulink/validation/rl_puffer_probe_20261005_replay
```

The recurrence is the actual Puffer implementation, extended for carried
state/reset handling. Background: [Puffer MinGRU design](https://puffer.ai/blog/mingru/)
and [Were RNNs All We Needed?](https://arxiv.org/abs/2410.01201).
