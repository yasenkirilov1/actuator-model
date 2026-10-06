"""Opt-in separate force/robot GAE traces on the one shared recurrent actor.

Physical plant, observations, sampled actions and reward remain unchanged.
The force channel may use shorter temporal credit; robot and critic retain
existing long-trace returns. Both action channels keep the original per-event
normalization. Disabled/equal trace settings execute the original trainer.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import torch
from ppo import PPOConfig as BasePPOConfig, PPOTrainer as BasePPOTrainer, RolloutBatch, compute_gae

BASE_PPO_SHA256 = "19890f3bfa8ad3b9592847cd3b11972f4cb13260639030582ba810d920fa8f17"

@dataclass
class PPOConfig(BasePPOConfig):
    force_lambda_time_s: float | None = None

    @property
    def force_gae_lambda(self):
        return self.gae_lambda if self.force_lambda_time_s is None else math.exp(-self.dt/self.force_lambda_time_s)

    def validate(self):
        super().validate()
        if self.force_lambda_time_s is not None and self.force_lambda_time_s<=0:
            raise ValueError("force credit time constant must be positive")
        if (self.force_lambda_time_s is not None and
                not math.isclose(self.force_lambda_time_s,self.lambda_time_s,rel_tol=0,abs_tol=1e-15)
                and not self.channelwise_policy_loss):
            raise ValueError("different force/robot credit requires channelwise policy loss")


def multiscale_advantages(batch,config):
    """Raw [T,N,2] actor advantages and unchanged long-trace critic targets.

    Each call uses the existing terminal/truncation-aware GAE function. A
    short force trace never alters queued-robot credit or value-head targets.
    """
    long,returns=compute_gae(batch.rewards,batch.values,batch.next_values,
                            batch.terminated,batch.truncated,config.gamma,config.gae_lambda)
    if config.force_lambda_time_s is None or math.isclose(
            config.force_lambda_time_s,config.lambda_time_s,rel_tol=0,abs_tol=1e-15):
        force=long
    else:
        force,_=compute_gae(batch.rewards,batch.values,batch.next_values,
                            batch.terminated,batch.truncated,config.gamma,config.force_gae_lambda)
    return torch.stack((force,long),dim=-1),returns


class PPOTrainer(BasePPOTrainer):
    def __init__(self,policy,config=None):
        super().__init__(policy,config or PPOConfig(dt=policy.dt))

    def update(self, batch: RolloutBatch) -> dict:
        config, policy = self.config, self.policy
        if config.force_lambda_time_s is None or math.isclose(
                config.force_lambda_time_s,config.lambda_time_s,rel_tol=0,abs_tol=1e-15):
            metrics=super().update(batch)
            return {**metrics,"multiscale_gae":False,"force_gae_lambda":config.gae_lambda,
                    "robot_gae_lambda":config.gae_lambda,"critic_gae_lambda":config.gae_lambda}
        batch.validate(self.policy)
        time, environments = batch.observations.shape[:2]
        with torch.no_grad():
            raw_channels,returns=multiscale_advantages(batch,config)
            advantages=raw_channels[...,1]
            valid = (torch.ones_like(batch.values, dtype=torch.bool) if batch.valid_mask is None
                     else batch.valid_mask.bool())
            if not valid.any():
                raise ValueError("the rollout has no valid transitions")
            selected = advantages[valid]
            advantages = (advantages - selected.mean()) / (selected.std(unbiased=False) + 1e-8)
            by_channel_advantages = torch.zeros((*advantages.shape, 2),device=advantages.device,
                                                dtype=advantages.dtype)
            for channel in range(2):
                enabled = valid & (batch.action_mask[...,channel] > .5)
                if enabled.any():
                    raw_channel=raw_channels[...,channel]
                    samples = raw_channel[enabled]
                    by_channel_advantages[...,channel] = (raw_channel-samples.mean()) / (
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
                    if config.target_kl > 0 and optimizer_steps and float(approx_kl.detach()) > config.target_kl:
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
                "multiscale_gae":True,"force_gae_lambda":config.force_gae_lambda,
                "force_lambda_time_s":config.force_lambda_time_s,
                "robot_gae_lambda":config.gae_lambda,"critic_gae_lambda":config.gae_lambda,
                "force_advantage_mean":float(raw_channels[...,0][valid].mean()),
                "force_advantage_std":float(raw_channels[...,0][valid].std(unbiased=False)),
                "std": policy.log_std.detach().clamp(-8, 1).exp().cpu().tolist(),
                "advantage_mean": float(selected.mean()), "advantage_std": float(selected.std(unbiased=False))}
