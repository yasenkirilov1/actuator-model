"""Portable frozen weights and CPU C99 inference for the shared RL policy.

The compiled inference takes weights as an argument: an updated GPU learner
can copy this small flat weight array into a running CPU environment without
recompiling. The C API emits latent means; sampling and tanh are explicit in
the rollout adapter. Deterministic qualification uses tanh(mean). Physical
scaling, equilibrium offset, gates, caps and command queues remain outside.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from policy import UnifiedPolicy


def arrays_from_policy(policy: UnifiedPolicy) -> dict[str, np.ndarray]:
    arrays = {"obs_center": policy.obs_center, "obs_scale": policy.obs_scale,
              "encoder_weight": policy.encoder.weight, "encoder_bias": policy.encoder.bias}
    for index, layer in enumerate(policy.recurrent.layers):
        arrays[f"recurrent_{index}_weight"] = layer.weight
        arrays[f"recurrent_{index}_bias"] = layer.bias
    arrays.update({"actor_weight": policy.actor.weight, "actor_bias": policy.actor.bias,
                   "actor_skip_weight": policy.actor_skip.weight,
                   "actor_aux_weight": policy.actor_aux.weight,
                   "critic_weight": policy.critic.weight, "critic_bias": policy.critic.bias,
                   "log_std": policy.log_std.clamp(-8, 1)})
    return {name: tensor.detach().cpu().numpy().astype(np.float32, copy=True)
            for name, tensor in arrays.items()}


def flat_weights(policy: UnifiedPolicy) -> np.ndarray:
    return np.concatenate([array.ravel() for array in arrays_from_policy(policy).values()]).astype(np.float32)


def numpy_evaluate_latent(mean: np.ndarray, latent: np.ndarray, log_std: np.ndarray,
                          action_mask: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Same transformed likelihood and latent-entropy regularizer as Torch."""
    mean, latent = np.asarray(mean, np.float32), np.asarray(latent, np.float32)
    log_std = np.clip(np.asarray(log_std, np.float32), -8.0, 1.0)
    by_action = -0.5 * ((latent - mean) / np.exp(log_std)) ** 2 - log_std - 0.5 * np.log(2.0 * np.pi)
    log_jacobian = 2.0 * (np.log(2.0) - latent - np.logaddexp(0.0, -2.0 * latent))
    by_action = by_action - log_jacobian
    entropy_by_action = np.broadcast_to(log_std + 0.5 * (1.0 + np.log(2.0 * np.pi)), mean.shape)
    if action_mask is not None:
        mask = np.asarray(action_mask, np.float32)
        by_action = by_action * mask
        entropy_by_action = entropy_by_action * mask
    return (by_action.sum(axis=-1).astype(np.float32), entropy_by_action.sum(axis=-1).astype(np.float32),
            by_action.astype(np.float32))


def numpy_sample(mean: np.ndarray, log_std: np.ndarray, rng: np.random.Generator,
                 action_mask: np.ndarray | None = None, deterministic: bool = False) -> dict:
    """CPU rollout sample without a GPU synchronization per 100 us tick.

    rng belongs to the run and its bit-generator state must be checkpointed.
    Store latent_actions/log_prob for PPO before physical gate/limit mapping.
    Ineligible channels output zero and contribute no likelihood or entropy.
    """
    mean = np.asarray(mean, np.float32)
    latent = mean if deterministic else (mean + rng.standard_normal(mean.shape, dtype=np.float32) *
                                         np.exp(np.clip(np.asarray(log_std, np.float32), -8.0, 1.0)))
    actions = np.tanh(latent)
    if action_mask is not None:
        actions = actions * np.asarray(action_mask, np.float32)
    log_prob, entropy, by_action = numpy_evaluate_latent(mean, latent, log_std, action_mask)
    return {"actions": actions.astype(np.float32), "latent_actions": latent.astype(np.float32),
            "log_prob": log_prob, "entropy": entropy, "log_prob_by_action": by_action}


def numpy_auxiliary(observations: np.ndarray) -> np.ndarray:
    x = np.asarray(observations, np.float32)
    if x.shape[-1] != 40:
        return np.zeros((*x.shape[:-1], 8), np.float32)
    s, b = 2*x[...,19], x[...,20]
    az, ax = 10*x[...,3], 10*x[...,4]
    denominator = 2*np.maximum(25*x[...,26], 1)
    return np.stack(((s+b)*az, s*b*az, 5*x[...,1]-.5*x[...,0], b*az,
                     (s+b)*(.2*x[...,10]), (60*x[...,25])*(.02*x[...,6]),
                     (20000*x[...,24])*(.005*x[...,7]), (s+b)*ax),axis=-1)/denominator[...,None]


def numpy_step(arrays: dict[str, np.ndarray], observations: np.ndarray, state: np.ndarray,
               reset_before: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Reference CPU inference; observations [N,D], state [L,N,H]."""
    x = np.asarray(observations, dtype=np.float32)
    previous = np.asarray(state, dtype=np.float32).copy()
    if reset_before is not None:
        previous[:, np.asarray(reset_before, dtype=bool), :] = 0

    def sigmoid(value):
        return 1.0 / (1.0 + np.exp(-np.clip(value, -80.0, 80.0)))

    normalized = (x - arrays["obs_center"]) / arrays["obs_scale"]
    auxiliary = numpy_auxiliary(x)
    hidden = np.tanh(np.concatenate((normalized,auxiliary),axis=-1) @ arrays["encoder_weight"].T + arrays["encoder_bias"])
    next_state = np.empty_like(previous)
    for index in range(previous.shape[0]):
        combined = hidden @ arrays[f"recurrent_{index}_weight"].T + arrays[f"recurrent_{index}_bias"]
        candidate, gate, projection = np.split(combined, 3, axis=-1)
        update = sigmoid(gate)
        candidate = np.where(candidate >= 0, candidate + 0.5, sigmoid(candidate))
        recurrent = previous[index] + update * (candidate - previous[index])
        highway = sigmoid(projection)
        hidden = highway * recurrent + (1.0 - highway) * hidden
        next_state[index] = recurrent
    mean = (hidden @ arrays["actor_weight"].T + arrays["actor_bias"] +
            normalized @ arrays["actor_skip_weight"].T +
            auxiliary @ arrays["actor_aux_weight"].T)
    value = (hidden @ arrays["critic_weight"].T + arrays["critic_bias"]).squeeze(-1)
    return mean, value, next_state


def export_policy(policy: UnifiedPolicy, directory: str | Path,
                  feature_names: list[str] | None = None,
                  provenance: dict | None = None) -> dict:
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    if feature_names is not None and len(feature_names) != policy.obs_dim:
        raise ValueError("feature-name count must match observation dimension")
    arrays = arrays_from_policy(policy)
    offsets, position = {}, 0
    for name, array in arrays.items():
        offsets[name] = {"offset": position, "shape": list(array.shape), "count": int(array.size)}
        position += array.size
    np.savez(directory / "policy_weights.npz", **arrays)
    flat_weights(policy).tofile(directory / "policy_weights.f32")
    weights_json = {"config": policy.config(), "features": feature_names,
                    "arrays": {name: value.tolist() for name, value in arrays.items()},
                    "offsets": offsets, "float_count": position,
                    "actor_outputs": ["normalized_current_request", "normalized_robot_correction"],
                    "inference": "tanh(latent mean); no stochastic sampling in native qualification",
                    "observation_normalization": "caller SI feature scales followed by exported fixed obs_center/obs_scale buffers; no running or split-derived normalization",
                    "mean_parameterization": "actor(hidden) + trainable actor_skip(normalized_observation) + trainable actor_aux(causal identified/measured products); both output channels use one recurrent actor; physically informed force-gain initialization, no teacher labels",
                    "auxiliary_features": policy.auxiliary_features,
                    "activations": {"encoder": "tanh", "candidate": "x+0.5 for x>=0; sigmoid(x) otherwise",
                                    "update_gate": "sigmoid", "highway_gate": "sigmoid", "action": "tanh"},
                    "state_layout_torch": "[layers,batch,hidden]",
                    "state_layout_c_batch": "[batch,layers,hidden]",
                    "provenance": provenance or {}}
    (directory / "policy_weights.json").write_text(json.dumps(weights_json, indent=2) + "\n")
    hidden, layers, obs = policy.hidden_size, policy.num_layers, policy.obs_dim
    header = f'''/* Frozen shape; weights may be refreshed between PPO rollouts. */
#ifndef ACF_UNIFIED_POLICY_H
#define ACF_UNIFIED_POLICY_H
#include <stddef.h>
#define ACF_RL_OBS {obs}
#define ACF_RL_HIDDEN {hidden}
#define ACF_RL_LAYERS {layers}
#define ACF_RL_AUX 8
#define ACF_RL_ENCODER_INPUT {policy.encoder_input_dim}
#define ACF_RL_FLOAT_COUNT {position}
#define ACF_RL_LOG_STD_OFFSET {offsets['log_std']['offset']}
#ifdef __cplusplus
extern "C" {{
#endif
/* State is [layers,hidden]. Reset only at a physical episode start. */
void acf_unified_policy_step(const float *weights, const float *observation,
    float *state, int reset_before, float *latent_mean, float *value);
/* Batch state is [batch,layers,hidden], observations [batch,obs]. */
void acf_unified_policy_step_many(size_t batch, const float *weights,
    const float *observations, float *states, const unsigned char *resets,
    float *latent_means, float *values);
#ifdef __cplusplus
}}
#endif
#endif
'''
    recurrent_w = ",".join(str(offsets[f"recurrent_{i}_weight"]["offset"]) for i in range(layers))
    recurrent_b = ",".join(str(offsets[f"recurrent_{i}_bias"]["offset"]) for i in range(layers))
    source = f'''#include "policy_inference.h"
#include <math.h>
#include <string.h>
static float acf_sigmoid(float x) {{
    if (x >= 0.0f) return 1.0f / (1.0f + expf(-x));
    float e = expf(x); return e / (1.0f + e);
}}
static const size_t recurrent_w[ACF_RL_LAYERS] = {{{recurrent_w}}};
static const size_t recurrent_b[ACF_RL_LAYERS] = {{{recurrent_b}}};
void acf_unified_policy_step(const float *w, const float *observation,
    float *state, int reset_before, float *mean, float *value) {{
    float hidden[ACF_RL_HIDDEN], combined[3*ACF_RL_HIDDEN], normalized[ACF_RL_OBS];
    float aux[ACF_RL_AUX] = {{0}};
    if (reset_before) memset(state, 0, ACF_RL_LAYERS*ACF_RL_HIDDEN*sizeof(float));
    for (size_t j=0; j<ACF_RL_OBS; j++)
        normalized[j] = (observation[j]-w[{offsets['obs_center']['offset']}+j]) / w[{offsets['obs_scale']['offset']}+j];
#if ACF_RL_OBS == 40
    float s=2*observation[19], b=observation[20];
    float az=10*observation[3], ax=10*observation[4];
    float denominator=2*fmaxf(25*observation[26],1);
    aux[0]=(s+b)*az/denominator; aux[1]=s*b*az/denominator;
    aux[2]=(5*observation[1]-.5f*observation[0])/denominator;
    aux[3]=b*az/denominator; aux[4]=(s+b)*(.2f*observation[10])/denominator;
    aux[5]=(60*observation[25])*(.02f*observation[6])/denominator;
    aux[6]=(20000*observation[24])*(.005f*observation[7])/denominator;
    aux[7]=(s+b)*ax/denominator;
#endif
    for (size_t i=0; i<ACF_RL_HIDDEN; i++) {{
        float x = w[{offsets['encoder_bias']['offset']} + i];
        for (size_t j=0; j<ACF_RL_OBS; j++)
            x += w[{offsets['encoder_weight']['offset']} + i*ACF_RL_ENCODER_INPUT + j]*normalized[j];
        for (size_t j=0; j<ACF_RL_AUX; j++)
            x += w[{offsets['encoder_weight']['offset']} + i*ACF_RL_ENCODER_INPUT + ACF_RL_OBS+j]*aux[j];
        hidden[i] = tanhf(x);
    }}
    for (size_t layer=0; layer<ACF_RL_LAYERS; layer++) {{
        for (size_t i=0; i<3*ACF_RL_HIDDEN; i++) {{
            float x = w[recurrent_b[layer] + i];
            for (size_t j=0; j<ACF_RL_HIDDEN; j++)
                x += w[recurrent_w[layer] + i*ACF_RL_HIDDEN + j]*hidden[j];
            combined[i] = x;
        }}
        for (size_t i=0; i<ACF_RL_HIDDEN; i++) {{
            float candidate = combined[i] >= 0 ? combined[i]+0.5f : acf_sigmoid(combined[i]);
            float update = acf_sigmoid(combined[ACF_RL_HIDDEN+i]);
            float projection = acf_sigmoid(combined[2*ACF_RL_HIDDEN+i]);
            size_t index = layer*ACF_RL_HIDDEN+i;
            float recurrent = state[index] + update*(candidate-state[index]);
            state[index] = recurrent;
            hidden[i] = projection*recurrent + (1.0f-projection)*hidden[i];
        }}
    }}
    for (size_t i=0; i<2; i++) {{
        float x = w[{offsets['actor_bias']['offset']}+i];
        for (size_t j=0; j<ACF_RL_HIDDEN; j++)
            x += w[{offsets['actor_weight']['offset']}+i*ACF_RL_HIDDEN+j]*hidden[j];
        for (size_t j=0; j<ACF_RL_OBS; j++)
            x += w[{offsets['actor_skip_weight']['offset']}+i*ACF_RL_OBS+j]*normalized[j];
        for (size_t j=0; j<ACF_RL_AUX; j++)
            x += w[{offsets['actor_aux_weight']['offset']}+i*ACF_RL_AUX+j]*aux[j];
        mean[i] = x;
    }}
    if (value) {{
        float x = w[{offsets['critic_bias']['offset']}];
        for (size_t j=0; j<ACF_RL_HIDDEN; j++) x += w[{offsets['critic_weight']['offset']}+j]*hidden[j];
        *value = x;
    }}
}}
void acf_unified_policy_step_many(size_t batch, const float *w,
    const float *observations, float *states, const unsigned char *resets,
    float *means, float *values) {{
    for (size_t i=0; i<batch; i++) acf_unified_policy_step(w,
        observations+i*ACF_RL_OBS, states+i*ACF_RL_LAYERS*ACF_RL_HIDDEN,
        resets ? resets[i] : 0, means+2*i, values ? values+i : 0);
}}
'''
    (directory / "policy_inference.h").write_text(header)
    (directory / "policy_inference.c").write_text(source)
    torch.save({"config": policy.config(), "state_dict": policy.state_dict(),
                "features": feature_names, "provenance": provenance or {}}, directory / "policy_checkpoint.pt")
    files = ["policy_weights.npz", "policy_weights.f32", "policy_weights.json",
             "policy_inference.h", "policy_inference.c", "policy_checkpoint.pt"]
    manifest = {"status": "exported_unqualified", "config": policy.config(),
                "parameter_count": sum(parameter.numel() for parameter in policy.parameters()),
                "files": {name: hashlib.sha256((directory / name).read_bytes()).hexdigest() for name in files}}
    (directory / "policy_export_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def load_policy(checkpoint: str | Path, device="cpu") -> UnifiedPolicy:
    saved = torch.load(checkpoint, map_location=device, weights_only=False)
    policy = UnifiedPolicy(**saved["config"]).to(device)
    policy.load_state_dict(saved["state_dict"])
    return policy
