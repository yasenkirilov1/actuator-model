"""Flange-owned recurrent PPO with explicit clocks, state and bootstraps.

This is on-policy reward optimization of the one shared policy in policy.py.
It does not consume teacher actions. Physical environment work and native
qualification are separate. Tensor layout is [time,environment,...].
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import torch

from policy import UnifiedPolicy, affine_prefix


@dataclass
class PPOConfig:
    dt: float = 1e-4
    gamma_time_s: float = 1.0
    lambda_time_s: float = 0.1
    chunk_length: int = 2048
    burnin_steps: int = 256
    epochs: int = 2
    minibatch_sequences: int = 8
    learning_rate: float = 1e-5
    actor_learning_rate: float | None = None
    critic_learning_rate: float = 3e-4
    log_std_learning_rate: float | None = None
    clip_coef: float = 0.1
    value_clip: float = 0.0
    value_coef: float = 0.1
    value_huber_delta: float = 1.0
    channelwise_policy_loss: bool = True
    robot_policy_weight: float = 1.0
    entropy_coef: float = 1e-4
    max_grad_norm: float = 0.5
    target_kl: float = 0.02

    @property
    def gamma(self) -> float:
        return math.exp(-self.dt / self.gamma_time_s)

    @property
    def gae_lambda(self) -> float:
        return math.exp(-self.dt / self.lambda_time_s)

    def validate(self) -> None:
        if min(self.dt, self.gamma_time_s, self.lambda_time_s, self.learning_rate) <= 0:
            raise ValueError("clock, discount time constants and learning rate must be positive")
        if min(self.chunk_length, self.epochs, self.minibatch_sequences) < 1 or self.burnin_steps < 0:
            raise ValueError("positive chunk/epoch/minibatch counts and nonnegative burn-in are required")
        if self.critic_learning_rate <= 0 or self.value_huber_delta <= 0:
            raise ValueError("critic learning rate and Huber scale must be positive")
        for rate in (self.actor_learning_rate, self.log_std_learning_rate):
            if rate is not None and rate <= 0:
                raise ValueError("optional actor/std learning rates must be positive")


@dataclass
class RolloutBatch:
    observations: torch.Tensor
    latent_actions: torch.Tensor
    log_probs: torch.Tensor
    values: torch.Tensor
    rewards: torch.Tensor
    terminated: torch.Tensor
    truncated: torch.Tensor
    next_values: torch.Tensor
    reset_before: torch.Tensor
    action_mask: torch.Tensor
    initial_state: torch.Tensor
    valid_mask: torch.Tensor | None = None
    state_inputs: torch.Tensor | None = None
    log_probs_by_action: torch.Tensor | None = None

    def validate(self, policy: UnifiedPolicy) -> None:
        if self.observations.ndim != 3 or self.observations.shape[-1] != policy.obs_dim:
            raise ValueError("observations must have shape [T,N,obs_dim]")
        shape = self.observations.shape[:2]
        for name in ("log_probs", "values", "rewards", "terminated", "truncated",
                     "next_values", "reset_before"):
            if getattr(self, name).shape != shape:
                raise ValueError(f"{name} must have shape [T,N]")
        for name in ("latent_actions", "action_mask"):
            if getattr(self, name).shape != (*shape, 2):
                raise ValueError(f"{name} must have shape [T,N,2]")
        if self.valid_mask is not None and self.valid_mask.shape != shape:
            raise ValueError("valid_mask must have shape [T,N]")
        policy._check_state(self.initial_state, shape[1])
        if self.state_inputs is not None and self.state_inputs.shape != (
                shape[0], policy.num_layers, shape[1], policy.hidden_size):
            raise ValueError("state_inputs must have shape [T,L,N,H]")
        if self.log_probs_by_action is not None and self.log_probs_by_action.shape != (*shape, 2):
            raise ValueError("log_probs_by_action must have shape [T,N,2]")
        tensor_fields = ("observations", "latent_actions", "log_probs", "values", "rewards",
                         "next_values", "initial_state")
        if any(not torch.isfinite(getattr(self, name)).all() for name in tensor_fields):
            raise ValueError("non-finite rollout data must be retained as a rejected attempt")


def compute_gae(rewards: torch.Tensor, values: torch.Tensor, next_values: torch.Tensor,
                terminated: torch.Tensor, truncated: torch.Tensor,
                gamma: float, gae_lambda: float) -> tuple[torch.Tensor, torch.Tensor]:
    """Failure bootstraps zero; a time limit bootstraps the final observation.

    next_values[t] is V(final next observation, next recurrent state), measured
    BEFORE any auto-reset. Both kinds of episode boundary stop the GAE trace,
    so a reset episode can never contribute to the previous episode's target.
    A rollout/chunk boundary is neither flag and keeps its value bootstrap.
    """
    if not (rewards.shape == values.shape == next_values.shape ==
            terminated.shape == truncated.shape):
        raise ValueError("GAE tensors must all have the same [T,N] shape")
    bootstrap = (~terminated.bool()).to(values.dtype)
    continue_trace = (~(terminated.bool() | truncated.bool())).to(values.dtype)
    delta = rewards + gamma * next_values * bootstrap - values
    coefficient = (gamma * gae_lambda) * continue_trace
    _, reverse_advantage = affine_prefix(coefficient.flip(0), delta.flip(0))
    advantage = reverse_advantage.flip(0)
    return advantage, advantage + values


class PPOTrainer:
    def __init__(self, policy: UnifiedPolicy, config: PPOConfig | None = None) -> None:
        self.policy = policy
        self.config = config or PPOConfig(dt=policy.dt)
        self.config.validate()
        if not math.isclose(self.config.dt, policy.dt, rel_tol=0, abs_tol=1e-12):
            raise ValueError("PPO and actor must use the same physical sample period")
        self.actor_parameters = [p for n,p in policy.named_parameters()
                                 if not n.startswith("critic.") and n != "log_std"]
        self.critic_parameters = list(policy.critic.parameters())
        self.optimizer = torch.optim.Adam([
            {"params":self.actor_parameters,"lr":self.config.actor_learning_rate or self.config.learning_rate,
             "name":"actor_shared_trunk"},
            {"params":self.critic_parameters,"lr":self.config.critic_learning_rate,"name":"critic_head"},
            {"params":[policy.log_std],"lr":self.config.log_std_learning_rate or
             self.config.actor_learning_rate or self.config.learning_rate,"name":"latent_std"},
        ], eps=1e-5)
        self.update_count = 0

    def state_dict(self) -> dict:
        return {"policy": self.policy.state_dict(), "optimizer": self.optimizer.state_dict(),
                "update_count": self.update_count, "config": vars(self.config),
                "policy_config": self.policy.config()}

    def load_state_dict(self, state: dict) -> None:
        if state.get("config") != vars(self.config):
            raise ValueError("resume requires the identical clock and PPO configuration")
        self.policy.load_state_dict(state["policy"])
        self.optimizer.load_state_dict(state["optimizer"])
        self.update_count = int(state.get("update_count", 0))

    def update(self, batch: RolloutBatch) -> dict:
        batch.validate(self.policy)
        config, policy = self.config, self.policy
        time, environments = batch.observations.shape[:2]
        with torch.no_grad():
            advantages, returns = compute_gae(batch.rewards, batch.values, batch.next_values,
                                             batch.terminated, batch.truncated,
                                             config.gamma, config.gae_lambda)
            valid = (torch.ones_like(batch.values, dtype=torch.bool) if batch.valid_mask is None
                     else batch.valid_mask.bool())
            if not valid.any():
                raise ValueError("the rollout has no valid transitions")
            selected = advantages[valid]
            raw_advantages = advantages
            advantages = (advantages - selected.mean()) / (selected.std(unbiased=False) + 1e-8)
            by_channel_advantages = torch.zeros((*advantages.shape, 2),device=advantages.device,
                                                dtype=advantages.dtype)
            for channel in range(2):
                enabled = valid & (batch.action_mask[...,channel] > .5)
                if enabled.any():
                    samples = raw_advantages[enabled]
                    by_channel_advantages[...,channel] = (raw_advantages-samples.mean()) / (
                        samples.std(unbiased=False)+1e-8)
            # Snapshot under the behavior policy before any optimizer step.
            # Large rollouts may provide stored states instead. States at
            # chunk starts are carried, never replaced by zeros.
            old_replay = None
            if batch.state_inputs is None or (config.channelwise_policy_loss and
                                             batch.log_probs_by_action is None):
                old_replay = policy.sequence(batch.observations, batch.initial_state,
                                            batch.reset_before, return_state_inputs=True)
            anchors = old_replay.state_inputs if batch.state_inputs is None else batch.state_inputs
            anchors = anchors.detach()
            if batch.log_probs_by_action is not None:
                old_by_action = batch.log_probs_by_action.detach()
            elif config.channelwise_policy_loss:
                _,_,old_by_action = policy.evaluate_latent(old_replay.mean,batch.latent_actions,
                                                          batch.action_mask)
            else:
                old_by_action = None

        chunks = list(range(0, time, config.chunk_length))
        totals = {name: 0.0 for name in ("policy_loss", "value_loss", "latent_entropy",
                                       "approx_kl", "clip_fraction", "grad_norm", "critic_grad_norm",
                                       "current_policy_loss", "robot_policy_loss")}
        total_weight, optimizer_steps, stopped_early = 0, 0, False
        device = batch.observations.device
        for _ in range(config.epochs):
            chunk_order = torch.randperm(len(chunks)).tolist()
            for chunk_index in chunk_order:
                start = chunks[chunk_index]
                end = min(time, start + config.chunk_length)
                burn_start = max(0, start - config.burnin_steps)
                row_order = torch.randperm(environments, device=device)
                for offset in range(0, environments, config.minibatch_sequences):
                    rows = row_order[offset:offset + config.minibatch_sequences]
                    hidden = anchors[burn_start, :, rows].detach()
                    if burn_start < start:
                        # Reconstruct the most recent context under the
                        # updated weights; gradients start at the loss chunk.
                        with torch.no_grad():
                            hidden = policy.sequence(batch.observations[burn_start:start, rows], hidden,
                                                     batch.reset_before[burn_start:start, rows]).state.detach()
                    result = policy.sequence(batch.observations[start:end, rows], hidden,
                                             batch.reset_before[start:end, rows])
                    new_log_prob, entropy, new_by_action = policy.evaluate_latent(
                        result.mean, batch.latent_actions[start:end, rows], batch.action_mask[start:end, rows])
                    old_log_prob = batch.log_probs[start:end, rows].detach()
                    old_value = batch.values[start:end, rows].detach()
                    advantage = advantages[start:end, rows].detach()
                    target = returns[start:end, rows].detach()
                    mask = valid[start:end, rows]
                    if not mask.any():
                        continue
                    log_ratio = new_log_prob - old_log_prob
                    ratio = log_ratio.exp()
                    loss_one = -advantage * ratio
                    loss_two = -advantage * ratio.clamp(1.0 - config.clip_coef, 1.0 + config.clip_coef)
                    policy_loss = torch.maximum(loss_one, loss_two)[mask].mean()
                    entropy_mean = entropy[mask].mean()
                    approx_kl = ((ratio - 1.0) - log_ratio)[mask].mean()
                    clip_fraction = ((ratio - 1.0).abs() > config.clip_coef)[mask].float().mean()
                    channel_losses = [policy_loss, policy_loss.new_zeros(())]
                    if config.channelwise_policy_loss:
                        policy_loss, entropy_mean = new_log_prob.new_zeros(()), new_log_prob.new_zeros(())
                        channel_kls, channel_clips = [], []
                        for channel in range(2):
                            enabled = mask & (batch.action_mask[start:end,rows,channel] > .5)
                            channel_losses[channel] = new_log_prob.new_zeros(())
                            if not enabled.any():
                                continue
                            lr = new_by_action[...,channel] - old_by_action[start:end,rows,channel]
                            rr = lr.exp()
                            adv = by_channel_advantages[start:end,rows,channel].detach()
                            channel_losses[channel] = torch.maximum(-adv*rr,-adv*rr.clamp(
                                1-config.clip_coef,1+config.clip_coef))[enabled].mean()
                            weight = config.robot_policy_weight if channel else 1.0
                            policy_loss = policy_loss + weight*channel_losses[channel]
                            # Each action's entropy is normalized over its own
                            # events; robot100Hz credit is not diluted10k-fold.
                            entropy_mean = entropy_mean + weight*(policy.log_std[channel].clamp(-8,1)+
                                .5*(1+math.log(2*math.pi)))
                            channel_kls.append(((rr-1)-lr)[enabled].mean())
                            channel_clips.append(((rr-1).abs()>config.clip_coef)[enabled].float().mean())
                        approx_kl = torch.stack(channel_kls).max() if channel_kls else new_log_prob.new_zeros(())
                        clip_fraction = torch.stack(channel_clips).mean() if channel_clips else new_log_prob.new_zeros(())
                    # Reject a too-distant epoch before applying another
                    # destructive step. KL is measured on sampled requests.
                    if not torch.isfinite(approx_kl):
                        raise FloatingPointError("non-finite PPO ratio; retain rollout")
                    if config.target_kl > 0 and optimizer_steps and float(approx_kl) > config.target_kl:
                        stopped_early = True
                        break
                    value_error = torch.nn.functional.huber_loss(result.value,target,
                        reduction="none",delta=config.value_huber_delta)
                    if config.value_clip > 0:
                        clipped_value = old_value+(result.value-old_value).clamp(-config.value_clip,
                                                                               config.value_clip)
                        value_error = torch.maximum(value_error,torch.nn.functional.huber_loss(
                            clipped_value,target,reduction="none",delta=config.value_huber_delta))
                    value_loss = value_error[mask].mean()
                    loss = policy_loss + config.value_coef * value_loss - config.entropy_coef * entropy_mean
                    self.optimizer.zero_grad(set_to_none=True)
                    loss.backward()
                    grad_norm = torch.nn.utils.clip_grad_norm_(self.actor_parameters+[policy.log_std],
                                                               config.max_grad_norm)
                    critic_grad_norm = torch.nn.utils.clip_grad_norm_(self.critic_parameters,config.max_grad_norm)
                    if not torch.isfinite(grad_norm) or not torch.isfinite(critic_grad_norm):
                        self.optimizer.zero_grad(set_to_none=True)
                        raise FloatingPointError("non-finite PPO gradient; preserve this rejected rollout")
                    self.optimizer.step()
                    weight = int(mask.sum())
                    metrics = (policy_loss, value_loss, entropy_mean, approx_kl, clip_fraction, grad_norm,
                               critic_grad_norm,*channel_losses)
                    for name, metric in zip(totals, metrics):
                        totals[name] += float(metric.detach()) * weight
                    total_weight += weight
                    optimizer_steps += 1
                if stopped_early:
                    break
            if stopped_early:
                break
        self.update_count += 1
        return {**{name: value / max(1, total_weight) for name, value in totals.items()},
                "optimizer_steps": optimizer_steps, "early_stop": stopped_early,
                "update_count": self.update_count, "gamma": config.gamma, "gae_lambda": config.gae_lambda,
                "chunk_length": config.chunk_length, "burnin_steps": config.burnin_steps,
                "rollout_transitions": time * environments,
                "eligible_current_actions": int(batch.action_mask[..., 0].sum()),
                "eligible_robot_actions": int(batch.action_mask[..., 1].sum()),
                "optimizer_learning_rates": [group["lr"] for group in self.optimizer.param_groups],
                "critic_detached_shared_trunk": policy.detach_critic_trunk,
                "channelwise_policy_loss": config.channelwise_policy_loss,
                "std": policy.log_std.detach().clamp(-8, 1).exp().cpu().tolist(),
                "advantage_mean": float(selected.mean()), "advantage_std": float(selected.std(unbiased=False))}
