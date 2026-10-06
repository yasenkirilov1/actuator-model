"""Architecture, gradient, PPO-state and portable-inference probes only.

Synthetic observations/rewards are deliberately not flange experiments and
cannot establish force performance. Probe output goes into a private lane.
"""
from __future__ import annotations

import argparse
import copy
import ctypes
import hashlib
import importlib.metadata
import importlib.util
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

import numpy as np
import torch
import pufferlib

from policy import UnifiedPolicy
from policy_export import (arrays_from_policy, export_policy, flat_weights, numpy_step,
                           numpy_sample, numpy_evaluate_latent)
from ppo import PPOConfig, PPOTrainer, RolloutBatch, compute_gae


def serial(policy, observations, state, reset):
    means, values, inputs = [], [], []
    for index in range(observations.shape[0]):
        inputs.append(state)
        step = policy.step(observations[index], state, reset_before=reset[index], deterministic=True)
        means.append(step.mean)
        values.append(step.value)
        state = step.state
    return torch.stack(means), torch.stack(values), state, torch.stack(inputs)


def maximum_error(one, two):
    return float((one - two).abs().max().detach())


def architecture_check(device):
    torch.manual_seed(20261005)
    policy = UnifiedPolicy(32).to(device)
    observations = torch.randn(2048, 8, 32, device=device)
    state = torch.rand(2, 8, 32, device=device) * 0.7
    reset = torch.zeros(2048, 8, device=device, dtype=torch.bool)
    reset[0, 0] = True
    reset[17, 1] = True
    reset[811, 2:5] = True
    reset[1899, 7] = True
    with torch.no_grad():
        parallel = policy.sequence(observations, state, reset, return_state_inputs=True)
        means, values, hidden, state_inputs = serial(policy, observations, state, reset)
        errors = {"mean": maximum_error(parallel.mean, means),
                  "value": maximum_error(parallel.value, values),
                  "state": maximum_error(parallel.state, hidden),
                  "state_inputs": maximum_error(parallel.state_inputs, state_inputs)}
        chunk_state, chunk_means = state, []
        for start in range(0, 2048, 256):
            piece = policy.sequence(observations[start:start + 256], chunk_state, reset[start:start + 256])
            chunk_state = piece.state
            chunk_means.append(piece.mean)
        errors["chunked_mean"] = maximum_error(torch.cat(chunk_means), means)
        errors["chunked_state"] = maximum_error(chunk_state, hidden)
        changed_future = observations.clone()
        changed_future[1024:] *= -15
        future_test = policy.sequence(changed_future, state, reset)
        errors["future_observation_invariance"] = maximum_error(future_test.mean[:1024], parallel.mean[:1024])
    if max(errors.values()) > 3e-5 or errors["future_observation_invariance"] != 0:
        raise AssertionError(f"serial/scan/chunk/causality parity failed: {errors}")
    return {"device": device, "shape": [2048, 8, 32], "errors": errors,
            "parameters": sum(p.numel() for p in policy.parameters()),
            "genuine_module": type(policy.recurrent).__module__ + "." + type(policy.recurrent).__name__}


def gradient_check():
    torch.manual_seed(29)
    parallel_policy = UnifiedPolicy(13).double()
    serial_policy = copy.deepcopy(parallel_policy)
    one = torch.randn(43, 3, 13, dtype=torch.float64, requires_grad=True)
    two = one.detach().clone().requires_grad_(True)
    h_one = torch.rand(2, 3, 32, dtype=torch.float64, requires_grad=True)
    h_two = h_one.detach().clone().requires_grad_(True)
    reset = torch.zeros(43, 3, dtype=torch.bool)
    reset[0, 0] = True
    reset[11, 1] = True
    reset[37, 2] = True
    multiplier = torch.randn(43, 3, 2, dtype=torch.float64)
    scan = parallel_policy.sequence(one, h_one, reset)
    means, values, hidden, _ = serial(serial_policy, two, h_two, reset)
    ((scan.mean * multiplier).sum() + scan.value.square().mean() + scan.state.square().mean()).backward()
    ((means * multiplier).sum() + values.square().mean() + hidden.square().mean()).backward()
    errors = {"observation_gradient": maximum_error(one.grad, two.grad),
              "initial_state_gradient": maximum_error(h_one.grad, h_two.grad)}
    for (name, param_one), (other_name, param_two) in zip(parallel_policy.named_parameters(),
                                                       serial_policy.named_parameters()):
        assert name == other_name
        if param_one.grad is not None:
            errors[name] = maximum_error(param_one.grad, param_two.grad)
    if max(errors.values()) > 1e-9:
        raise AssertionError(f"gradient parity failed: {errors}")
    return {"float64_serial_scan_max_gradient_error": max(errors.values()), "errors": errors}


def gae_check():
    torch.manual_seed(5)
    shape = (137, 4)
    rewards, values, next_values = (torch.randn(shape, dtype=torch.float64) for _ in range(3))
    terminated, truncated = torch.zeros(shape, dtype=torch.bool), torch.zeros(shape, dtype=torch.bool)
    terminated[17, 0] = True
    terminated[91, 3] = True
    truncated[33, 1] = True
    truncated[72, 2] = True
    config = PPOConfig()
    advantage, _ = compute_gae(rewards, values, next_values, terminated, truncated,
                               config.gamma, config.gae_lambda)
    expected = torch.empty_like(advantage)
    running = torch.zeros(4, dtype=torch.float64)
    for index in reversed(range(shape[0])):
        bootstrap = (~terminated[index]).to(values.dtype)
        continuation = (~(terminated[index] | truncated[index])).to(values.dtype)
        running = rewards[index] + config.gamma * next_values[index] * bootstrap - values[index] + \
                  config.gamma * config.gae_lambda * continuation * running
        expected[index] = running
    error = maximum_error(advantage, expected)
    poisoned = next_values.clone()
    poisoned[terminated] += 1e6
    no_failure_bootstrap, _ = compute_gae(rewards, values, poisoned, terminated, truncated,
                                         config.gamma, config.gae_lambda)
    failure_error = maximum_error(advantage, no_failure_bootstrap)
    changed = next_values.clone()
    changed[truncated] += 2.0
    time_limit, _ = compute_gae(rewards, values, changed, terminated, truncated,
                               config.gamma, config.gae_lambda)
    time_error = float((time_limit - advantage)[truncated].abs().min())
    if error > 1e-11 or failure_error != 0 or abs(time_error - 2 * config.gamma) > 1e-11:
        raise AssertionError("GAE failure/time-limit bootstrap semantics failed")
    return {"serial_scan_error": error, "failure_bootstrap_invariance": failure_error,
            "time_limit_bootstrap_change": time_error, "gamma": config.gamma,
            "lambda": config.gae_lambda, "one_second_discount": config.gamma ** 10000}


def action_distribution_check():
    torch.manual_seed(3103)
    rng = np.random.default_rng(3103)
    policy = UnifiedPolicy(32)
    mean = rng.normal(0, 0.5, (113, 2)).astype(np.float32)
    log_std = policy.log_std.detach().numpy()
    mask = np.ones_like(mean)
    mask[:, 1] = 0
    mask[::100, 1] = 1
    sample = numpy_sample(mean, log_std, rng, mask)
    latent = sample["latent_actions"]
    torch_lp, torch_entropy, torch_by_action = policy.evaluate_latent(
        torch.from_numpy(mean), torch.from_numpy(latent), torch.from_numpy(mask))
    errors = {"log_prob": float(np.max(np.abs(torch_lp.detach().numpy() - sample["log_prob"]))),
              "latent_entropy": float(np.max(np.abs(torch_entropy.detach().numpy() - sample["entropy"]))),
              "by_action": float(np.max(np.abs(torch_by_action.detach().numpy() - sample["log_prob_by_action"])))}
    lp_copy, _, _ = numpy_evaluate_latent(mean, latent, log_std, mask)
    assert np.array_equal(lp_copy, sample["log_prob"])
    assert np.all(sample["actions"][mask == 0] == 0)
    assert np.all(sample["log_prob_by_action"][mask == 0] == 0)
    # Disabled robot samples cannot update the robot mean/std through the
    # actor likelihood; shared features can still learn from the current.
    policy.zero_grad(set_to_none=True)
    observations = torch.randn(5, 32)
    disabled = torch.tensor([[1.0, 0.0]]).repeat(5, 1)
    step = policy.step(observations, policy.initial_state(5), action_mask=disabled)
    (-step.log_prob.mean()).backward()
    robot_gradient = float(policy.actor.weight.grad[1].abs().max())
    robot_std_gradient = float(policy.log_std.grad[1].abs())
    if max(errors.values()) > 3e-6 or robot_gradient != 0 or robot_std_gradient != 0:
        raise AssertionError("CPU/Torch distribution or disabled-channel gradient parity failed")
    return {"errors": errors, "disabled_robot_head_gradient": robot_gradient,
            "disabled_robot_std_gradient": robot_std_gradient,
            "clock_mask_robot_events": int(mask[:, 1].sum()), "frames": len(mask)}


def optimizer_check(device):
    torch.manual_seed(1729)
    policy = UnifiedPolicy(32).to(device)
    cfg = PPOConfig(chunk_length=64, burnin_steps=17, epochs=2, minibatch_sequences=2, target_kl=0)
    trainer = PPOTrainer(policy, cfg)
    time_count, batch_size = 191, 5
    observations = torch.randn(time_count, batch_size, 32, device=device)
    state = policy.initial_state(batch_size, device)
    initial = state.clone()
    reset = torch.zeros(time_count, batch_size, dtype=torch.bool, device=device)
    reset[0] = True
    reset[77, 1] = True
    reset[114, 4] = True
    action_mask = torch.ones(time_count, batch_size, 2, device=device)
    action_mask[..., 1] = 0
    action_mask[::100, :, 1] = 1
    samples = []
    with torch.no_grad():
        for index in range(time_count):
            step = policy.step(observations[index], state, reset[index], action_mask[index])
            samples.append(step)
            state = step.state
    latent = torch.stack([sample.latent_actions for sample in samples])
    old_lp = torch.stack([sample.log_prob for sample in samples])
    old_value = torch.stack([sample.value for sample in samples])
    terminated = torch.zeros(time_count, batch_size, dtype=torch.bool, device=device)
    truncated = torch.zeros_like(terminated)
    terminated[76, 1] = True
    truncated[113, 4] = True
    # A synthetic reward is used solely to show real PPO gradients work.
    # It is neither a teacher label nor a flange-performance experiment.
    rewards = -(torch.tanh(latent[..., 0]) - 0.02).square() + \
              action_mask[..., 1] * -(torch.tanh(latent[..., 1]) - 0.1).square()
    next_values = torch.cat((old_value[1:], torch.zeros(1, batch_size, device=device)))
    next_values[113, 4] = 0.317  # final observation BEFORE time-limit reset
    rollout = RolloutBatch(observations, latent, old_lp, old_value, rewards, terminated,
                           truncated, next_values, reset, action_mask, initial)
    before = {name: param.detach().clone() for name, param in policy.named_parameters()}
    metrics = trainer.update(rollout)
    changes = {name: float((param.detach() - before[name]).abs().max())
               for name, param in policy.named_parameters()}
    if not all(changes[name] > 0 for name in ("encoder.weight", "actor.weight", "critic.weight", "log_std")):
        raise AssertionError("PPO did not update all shared-trunk/action/value paths")
    return {"synthetic_only": True, "metrics": metrics, "maximum_parameter_changes": changes}


def portable_check(output):
    torch.manual_seed(3101)
    policy = UnifiedPolicy(32)
    export_dir = output / "portable_initial_policy"
    export_manifest = export_policy(policy, export_dir,
                                    provenance={"kind": "untrained_architecture_probe", "seed": 3101})
    library = export_dir / "policy_inference.so"
    command = ["cc", "-O2", "-std=c99", "-fPIC", "-shared", str(export_dir / "policy_inference.c"),
               "-lm", "-o", str(library)]
    subprocess.run(command, check=True, capture_output=True, text=True)
    loaded = ctypes.CDLL(str(library))
    c_step = loaded.acf_unified_policy_step_many
    ptr = ctypes.POINTER(ctypes.c_float)
    byte_ptr = ctypes.POINTER(ctypes.c_ubyte)
    c_step.argtypes = [ctypes.c_size_t, ptr, ptr, ptr, byte_ptr, ptr, ptr]
    c_step.restype = None
    arrays = arrays_from_policy(policy)
    weights = flat_weights(policy)
    rng = np.random.default_rng(3101)
    count = 7
    c_state = rng.uniform(0, 0.7, (count, 2, 32)).astype(np.float32)
    numpy_state = c_state.transpose(1, 0, 2).copy()
    torch_state = torch.from_numpy(numpy_state.copy())
    c_mean, c_value = np.empty((count, 2), np.float32), np.empty(count, np.float32)
    errors = {"c_mean": 0.0, "c_value": 0.0, "c_state": 0.0,
              "numpy_mean": 0.0, "numpy_value": 0.0, "numpy_state": 0.0}
    start = time.perf_counter()
    with torch.no_grad():
        for index in range(1200):
            obs = rng.normal(0, 1, (count, 32)).astype(np.float32)
            resets = np.zeros(count, np.uint8)
            if index in (0, 13, 301, 1001):
                resets[index % count] = 1
            c_step(count, weights.ctypes.data_as(ptr), obs.ctypes.data_as(ptr),
                   c_state.ctypes.data_as(ptr), resets.ctypes.data_as(byte_ptr),
                   c_mean.ctypes.data_as(ptr), c_value.ctypes.data_as(ptr))
            n_mean, n_value, numpy_state = numpy_step(arrays, obs, numpy_state, resets)
            actual = policy.step(torch.from_numpy(obs), torch_state, torch.from_numpy(resets.astype(bool)),
                                 deterministic=True)
            torch_state = actual.state
            references = {"mean": actual.mean.numpy(), "value": actual.value.numpy(),
                          "state": actual.state.numpy()}
            for name, array in (("c_mean", c_mean), ("c_value", c_value),
                                ("c_state", c_state.transpose(1, 0, 2)),
                                ("numpy_mean", n_mean), ("numpy_value", n_value),
                                ("numpy_state", numpy_state)):
                errors[name] = max(errors[name], float(np.max(np.abs(array - references[name.split('_', 1)[1]]))))
    if max(errors.values()) > 3e-5:
        raise AssertionError(f"portable C/NumPy inference parity failed: {errors}")
    # Component-only benchmark, excluding environment, sensor and queue work.
    benchmarks = []
    for batch, frames in ((1, 3000), (64, 512), (256, 128)):
        obs = rng.normal(size=(batch, 32)).astype(np.float32)
        hidden = np.zeros((batch, 2, 32), np.float32)
        means, values = np.empty((batch, 2), np.float32), np.empty(batch, np.float32)
        elapsed = []
        for repeat in range(3):
            begin = time.perf_counter()
            for _ in range(frames):
                c_step(batch, weights.ctypes.data_as(ptr), obs.ctypes.data_as(ptr),
                       hidden.ctypes.data_as(ptr), None, means.ctypes.data_as(ptr), values.ctypes.data_as(ptr))
            elapsed.append(time.perf_counter() - begin)
        median = float(np.median(elapsed))
        benchmarks.append({"batch": batch, "frames": frames, "median_s": median,
                           "policy_evaluations_per_s": batch * frames / median,
                           "batch_tick_us": median / frames * 1e6})
    sampled_benchmarks = []
    for batch, frames in ((1, 2048), (64, 512)):
        obs = rng.normal(size=(batch, 32)).astype(np.float32)
        hidden = np.zeros((batch, 2, 32), np.float32)
        means, values = np.empty((batch, 2), np.float32), np.empty(batch, np.float32)
        mask = np.ones((batch, 2), np.float32)
        begin = time.perf_counter()
        for frame in range(frames):
            c_step(batch, weights.ctypes.data_as(ptr), obs.ctypes.data_as(ptr),
                   hidden.ctypes.data_as(ptr), None, means.ctypes.data_as(ptr), values.ctypes.data_as(ptr))
            mask[:, 1] = float(frame % 100 == 0)
            numpy_sample(means, arrays["log_std"], rng, mask)
        elapsed = time.perf_counter() - begin
        sampled_benchmarks.append({"batch": batch, "frames": frames, "elapsed_s": elapsed,
                                   "sampled_policy_evaluations_per_s": batch * frames / elapsed,
                                   "batch_tick_us": elapsed / frames * 1e6})
    return {"errors": errors, "1200_frames_7_rows_check_s": time.perf_counter() - start,
            "component_only_c_benchmarks": benchmarks, "export": export_manifest,
            "component_only_c_plus_numpy_sampling": sampled_benchmarks,
            "compiler_command": command, "library_sha256": hashlib.sha256(library.read_bytes()).hexdigest()}


def gpu_component_check():
    if not torch.cuda.is_available():
        return {"available": False}
    torch.manual_seed(3102)
    policy = UnifiedPolicy(32).cuda()
    batch, frames = 256, 256
    obs = torch.randn(batch, 32, device="cuda")
    hidden = policy.initial_state(batch, "cuda")
    mask = torch.ones(batch, 2, device="cuda")
    mask[:, 1] = 0
    with torch.no_grad():
        for _ in range(16):
            hidden = policy.step(obs, hidden, action_mask=mask).state
        torch.cuda.synchronize()
        begin = time.perf_counter()
        for _ in range(frames):
            hidden = policy.step(obs, hidden, action_mask=mask).state
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - begin
    observations = torch.randn(2048, 32, 32, device="cuda")
    hidden = policy.initial_state(32, "cuda")
    for _ in range(2):
        result = policy.sequence(observations, hidden)
        (result.mean.square().mean() + result.value.square().mean()).backward()
        policy.zero_grad(set_to_none=True)
    torch.cuda.synchronize()
    begin = time.perf_counter()
    result = policy.sequence(observations, hidden)
    (result.mean.square().mean() + result.value.square().mean()).backward()
    torch.cuda.synchronize()
    train_elapsed = time.perf_counter() - begin
    return {"available": True, "component_only": True,
            "eager_sampled_policy": {"batch": batch, "frames": frames, "elapsed_s": elapsed,
                                     "policy_evaluations_per_s": batch * frames / elapsed,
                                     "batch_tick_us": elapsed / frames * 1e6},
            "sequence_forward_backward": {"shape": [2048, 32, 32], "elapsed_s": train_elapsed,
                                          "samples_per_s": 2048 * 32 / train_elapsed},
            "max_memory_bytes": torch.cuda.max_memory_allocated()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--skip-gpu", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(2)
    torch.backends.cuda.matmul.allow_tf32 = False
    sources = {name: hashlib.sha256((Path(__file__).parent / name).read_bytes()).hexdigest()
               for name in ("policy.py", "ppo.py", "policy_export.py", "policy_checks.py")}
    report = {"status": "running", "scope": "synthetic architecture/gradient/interface probes; no flange qualification",
              "runtime": {"python": sys.version, "executable": sys.executable,
                          "platform": platform.platform(), "torch": torch.__version__,
                          "numpy": np.__version__, "pufferlib": importlib.metadata.version("pufferlib"),
                          "puffer_file": pufferlib.__file__, "cuda": torch.version.cuda,
                          "cuda_available": torch.cuda.is_available(),
                          "device": torch.cuda.get_device_name(0) if torch.cuda.is_available() else None,
                          "native_C_backend_available": importlib.util.find_spec("pufferlib._C") is not None},
              "sources": sources}
    destination = args.output / "results.json"
    destination.write_text(json.dumps(report, indent=2) + "\n")
    try:
        report["architecture_cpu"] = architecture_check("cpu")
        report["gradient"] = gradient_check()
        report["gae"] = gae_check()
        report["action_distribution"] = action_distribution_check()
        report["synthetic_ppo_cpu"] = optimizer_check("cpu")
        report["portable"] = portable_check(args.output)
        if not args.skip_gpu and torch.cuda.is_available():
            report["architecture_cuda"] = architecture_check("cuda")
            report["synthetic_ppo_cuda"] = optimizer_check("cuda")
            report["gpu_components"] = gpu_component_check()
        report["status"] = "complete"
    except Exception as error:
        report["status"] = "failed"
        report["error"] = {"type": type(error).__name__, "message": str(error)}
        destination.write_text(json.dumps(report, indent=2) + "\n")
        raise
    destination.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"status": report["status"], "result": str(destination),
                      "parameters": report["architecture_cpu"]["parameters"]}))


if __name__ == "__main__":
    main()
