"""One causal, continuous-action Puffer MinGRU actor and recurrent critic.

The two outputs are normalized current request and robot correction request.
The environment owns the known equilibrium-current offset, physical limits,
robot event clock, safety gate and delayed command queue. There is no PI or
learned residual branch here. Training uses sampled actions and rewards.

Puffer's stock parallel path assumes a zero initial state. The affine scan
below extends the same genuine MinGRU layers to arbitrary initial state and
episode reset masks without resetting at rollout/chunk boundaries.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import torch
from torch import nn
from torch.nn import functional as F
from pufferlib.models import MinGRU


def affine_prefix(a: torch.Tensor, b: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Inclusive affine prefix along dimension 0, including exact zero resets.

    Each row represents h[t] = a[t]*h[t-1] + b[t]. Products/compositions are
    associative. This direct scan also permits signed b (used by GAE), unlike
    a log-space positive scan, and never takes log(0) at an episode reset.
    """
    if a.shape != b.shape or a.ndim < 1:
        raise ValueError("a and b must have equal shape with a time dimension")
    stride = 1
    while stride < a.shape[0]:
        previous_a, previous_b = a, b
        a = torch.cat((previous_a[:stride],
                       previous_a[stride:] * previous_a[:-stride]), dim=0)
        b = torch.cat((previous_b[:stride],
                       previous_b[stride:] + previous_a[stride:] * previous_b[:-stride]), dim=0)
        stride *= 2
    return a, b


@dataclass
class PolicyStep:
    actions: torch.Tensor
    latent_actions: torch.Tensor
    log_prob: torch.Tensor
    entropy: torch.Tensor
    value: torch.Tensor
    state: torch.Tensor
    mean: torch.Tensor
    log_prob_by_action: torch.Tensor


@dataclass
class PolicySequence:
    mean: torch.Tensor
    value: torch.Tensor
    state: torch.Tensor
    state_inputs: torch.Tensor | None = None


class UnifiedPolicy(nn.Module):
    """Shared 32x2 recurrent trunk, two action means, one recurrent value head.

    All tensors use time-major [T,N,...] except hidden state [L,N,H]. A reset
    mask is true BEFORE the corresponding observation, only when that row
    belongs to a new physical episode. A closed robot gate never resets it.
    """
    auxiliary_features = [
        "identified_total_mass_times_measured_az_over_2Kf",
        "identified_S_B_mass_product_times_measured_az_over_2Kf",
        "estimated_axial_minus_normal_force_over_2Kf",
        "identified_B_mass_times_measured_az_over_2Kf",
        "identified_total_mass_times_issued_robot_az_over_2Kf",
        "identified_damping_times_relative_velocity_over_2Kf",
        "identified_stiffness_times_travel_error_over_2Kf",
        "identified_total_mass_times_measured_ax_over_2Kf",
    ]
    def __init__(self, obs_dim: int, hidden_size: int = 32, num_layers: int = 2,
                 dt: float = 1e-4, min_memory_s: float = 3e-4,
                 max_memory_s: float = 0.3,
                 initial_log_std: tuple[float, float] = (-4.5, -1.5),
                 physics_init_force_gain: float = 0.1,
                 detach_critic_trunk: bool = True,
                 center_context: bool = True,
                 paired_filter_init: bool = True) -> None:
        super().__init__()
        if obs_dim < 1 or hidden_size < 1 or num_layers < 1 or dt <= 0:
            raise ValueError("positive observation/hidden/layer dimensions and dt are required")
        if not (0 < min_memory_s <= max_memory_s):
            raise ValueError("memory time constants must be positive and ordered")
        self.obs_dim, self.hidden_size, self.num_layers = obs_dim, hidden_size, num_layers
        self.dt = float(dt)
        self.min_memory_s, self.max_memory_s = float(min_memory_s), float(max_memory_s)
        self.physics_init_force_gain = float(physics_init_force_gain)
        self.detach_critic_trunk = bool(detach_critic_trunk)
        self.center_context = bool(center_context)
        self.paired_filter_init = bool(paired_filter_init)
        self.encoder_input_dim = obs_dim + len(self.auxiliary_features)
        # Fixed identified-context scaling, independent of data or split.
        # The first18 causal sensor features retain the documented SI scale.
        center, scale = torch.zeros(obs_dim), torch.ones(obs_dim)
        if obs_dim == 40 and center_context:
            center[18:] = torch.tensor([
                .55, .525, .6, .2, .5, .5, .9873, 1, .976, .8896,
                1, 1, 1, 1, .4, 1, 1, 1, 1, 1, 1, 0])
            scale[18:] = torch.tensor([
                .5, .5, .2, .2, 1, 1, .5, .5, .25, .25,
                .25, .25, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1])
        self.register_buffer("obs_center", center)
        self.register_buffer("obs_scale", scale)
        self.encoder = nn.Linear(self.encoder_input_dim, hidden_size)
        self.recurrent = MinGRU(hidden_size=hidden_size, num_layers=num_layers)
        # An ordinary Linear bias lets the genuine Puffer cells start with
        # physically meaningful update-gate time constants at the 10 kHz clock.
        # Candidate and highway biases are initially zero. No extra RNN exists.
        taus = torch.logspace(math.log10(min_memory_s), math.log10(max_memory_s), hidden_size)
        update = -torch.expm1(-dt / taus)
        gate_bias = torch.logit(update)
        for layer in self.recurrent.layers:
            layer.bias = nn.Parameter(torch.zeros(3 * hidden_size))
            with torch.no_grad():
                layer.bias[hidden_size:2 * hidden_size].copy_(gate_bias)
        self.actor = nn.Linear(hidden_size, 2)
        # This is a learned neural connection in the same actor, with no
        # independent controller or target-action labels. It keeps large
        # measured force errors out of the saturating encoder bottleneck.
        self.actor_skip = nn.Linear(obs_dim, 2, bias=False)
        self.actor_aux = nn.Linear(len(self.auxiliary_features), 2, bias=False)
        self.critic = nn.Linear(hidden_size, 1)
        self.log_std = nn.Parameter(torch.tensor(initial_log_std, dtype=torch.float32))
        nn.init.orthogonal_(self.encoder.weight, gain=0.5)
        nn.init.zeros_(self.encoder.bias)
        nn.init.zeros_(self.actor.weight)
        nn.init.zeros_(self.actor.bias)
        nn.init.zeros_(self.actor_skip.weight)
        nn.init.zeros_(self.actor_aux.weight)
        with torch.no_grad():
            # feature0=(F_est-F_target)/.5N; SI request=Ieq+2*tanh(mean).
            # Near zero, -gain*A/N feedback maps to -.25*gain here.
            self.actor_skip.weight[0, 0] = -.25 * physics_init_force_gain
        nn.init.zeros_(self.critic.weight)
        nn.init.zeros_(self.critic.bias)
        if paired_filter_init and obs_dim == 40 and hidden_size == 32:
            # Four measured/context-conditioned features, four learned
            # minGRU time constants, paired signs. Their g(0)=.5 startup
            # components cancel when a readout uses paired differences.
            # The coefficients and gates remain ordinary trainable layers.
            with torch.no_grad():
                self.encoder.weight.zero_()
                bases = [(0,.02),(obs_dim+1,1.),(obs_dim+2,1.),(obs_dim+5,1.)]
                taus = [.0001,.0004,.003,.02]
                first = self.recurrent.layers[0]
                first.weight.zero_()
                first.bias.zero_()
                for base,(feature,magnitude) in enumerate(bases):
                    for clock,tau in enumerate(taus):
                        for sign in range(2):
                            neuron=8*base+2*clock+sign
                            self.encoder.weight[neuron,feature] = magnitude*(1 if sign==0 else -1)
                            first.weight[neuron,neuron]=1
                            first.bias[hidden_size+neuron]=math.log(math.expm1(dt/tau)) if dt/tau < 50 else dt/tau
                            first.bias[2*hidden_size+neuron]=8
                for layer in self.recurrent.layers[1:]:
                    layer.weight.zero_()
                    layer.bias.zero_()
                    layer.weight[:hidden_size].copy_(torch.eye(hidden_size))
                    layer.bias[hidden_size:2*hidden_size].fill_(math.log(math.expm1(dt/.0001)) if dt/.0001 < 50 else dt/.0001)
                    layer.bias[2*hidden_size:].fill_(-8)

    def config(self) -> dict:
        return {"obs_dim": self.obs_dim, "hidden_size": self.hidden_size,
                "num_layers": self.num_layers, "dt": self.dt,
                "min_memory_s": self.min_memory_s, "max_memory_s": self.max_memory_s,
                "physics_init_force_gain": self.physics_init_force_gain,
                "detach_critic_trunk": self.detach_critic_trunk,
                "center_context": self.center_context,
                "paired_filter_init": self.paired_filter_init,
                "initial_log_std": tuple(float(v) for v in self.log_std.detach().cpu())}

    def normalize(self, observations: torch.Tensor) -> torch.Tensor:
        return (observations - self.obs_center) / self.obs_scale

    def auxiliary(self, observations: torch.Tensor) -> torch.Tensor:
        # Products of measured quantities and identified parameters. These
        # are neural basis features, not fixed feedforward coefficients.
        if self.obs_dim != 40:
            return observations.new_zeros(*observations.shape[:-1], len(self.auxiliary_features))
        s, b = 2 * observations[..., 19], observations[..., 20]
        az, ax = 10 * observations[..., 3], 10 * observations[..., 4]
        denominator = 2 * (25 * observations[..., 26]).clamp_min(1)
        return torch.stack((
            (s+b)*az, s*b*az, 5*observations[..., 1]-.5*observations[..., 0],
            b*az, (s+b)*(.2*observations[..., 10]),
            (60*observations[..., 25])*(.02*observations[..., 6]),
            (20000*observations[..., 24])*(.005*observations[..., 7]),
            (s+b)*ax,
        ), dim=-1) / denominator.unsqueeze(-1)

    def heads(self, hidden: torch.Tensor, normalized: torch.Tensor, auxiliary: torch.Tensor):
        mean = self.actor(hidden) + self.actor_skip(normalized) + self.actor_aux(auxiliary)
        critic_input = hidden.detach() if self.detach_critic_trunk else hidden
        return mean, self.critic(critic_input).squeeze(-1)

    def initial_state(self, batch_size: int, device=None, dtype=None) -> torch.Tensor:
        parameter = self.encoder.weight
        return torch.zeros(self.num_layers, batch_size, self.hidden_size,
                           device=device or parameter.device, dtype=dtype or parameter.dtype)

    def _check_state(self, state: torch.Tensor, batch_size: int) -> None:
        if state.shape != (self.num_layers, batch_size, self.hidden_size):
            raise ValueError("hidden state must have shape [num_layers,batch,hidden_size]")

    def evaluate_latent(self, mean: torch.Tensor, latent_actions: torch.Tensor,
                        action_mask: torch.Tensor | None = None
                        ) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        """Tanh-Normal likelihood of the stored *sampled latent* action.

        Store the latent request, not the externally clipped or gated action.
        Disabled channels contribute neither likelihood nor entropy. The
        robot channel is enabled only at eligible 100 Hz command events.
        Entropy is the Gaussian latent entropy, explicitly a regularizer;
        it is not advertised as exact entropy of the transformed distribution.
        """
        if mean.shape != latent_actions.shape or mean.shape[-1] != 2:
            raise ValueError("means and latents must have equal [...,2] shape")
        std = self.log_std.clamp(-8.0, 1.0).exp()
        distribution = torch.distributions.Normal(mean, std)
        # Stable log(1 - tanh(z)^2), including large |z|.
        log_jacobian = 2.0 * (math.log(2.0) - latent_actions - F.softplus(-2.0 * latent_actions))
        by_action = distribution.log_prob(latent_actions) - log_jacobian
        entropy_by_action = distribution.entropy()
        if action_mask is not None:
            mask = action_mask.to(dtype=mean.dtype)
            by_action = by_action * mask
            entropy_by_action = entropy_by_action * mask
        return by_action.sum(dim=-1), entropy_by_action.sum(dim=-1), by_action

    def step(self, observations: torch.Tensor, state: torch.Tensor,
             reset_before: torch.Tensor | None = None,
             action_mask: torch.Tensor | None = None,
             deterministic: bool = False) -> PolicyStep:
        if observations.ndim != 2 or observations.shape[-1] != self.obs_dim:
            raise ValueError("step observations must have shape [batch,obs_dim]")
        self._check_state(state, observations.shape[0])
        if reset_before is not None:
            state = torch.where(reset_before.bool()[None, :, None], torch.zeros_like(state), state)
        normalized = self.normalize(observations)
        auxiliary = self.auxiliary(observations)
        encoded = torch.tanh(self.encoder(torch.cat((normalized,auxiliary),dim=-1)))
        hidden, next_tuple = self.recurrent.forward_eval(encoded, (state,))
        mean, value = self.heads(hidden, normalized, auxiliary)
        if deterministic:
            latent = mean
        else:
            latent = (mean + torch.randn_like(mean) * self.log_std.clamp(-8.0, 1.0).exp()).detach()
        actions = torch.tanh(latent)
        if action_mask is not None:
            actions = actions * action_mask.to(actions.dtype)
        log_prob, entropy, by_action = self.evaluate_latent(mean, latent, action_mask)
        return PolicyStep(actions, latent, log_prob, entropy, value, next_tuple[0], mean, by_action)

    def sequence(self, observations: torch.Tensor, initial_state: torch.Tensor,
                 reset_before: torch.Tensor | None = None,
                 return_state_inputs: bool = False) -> PolicySequence:
        if observations.ndim != 3 or observations.shape[-1] != self.obs_dim:
            raise ValueError("sequence observations must have shape [time,batch,obs_dim]")
        time, batch = observations.shape[:2]
        if time < 1:
            raise ValueError("an empty sequence has no output")
        self._check_state(initial_state, batch)
        if reset_before is None:
            keep_previous = observations.new_ones(time, batch, 1)
        else:
            if reset_before.shape != (time, batch):
                raise ValueError("reset_before must have shape [time,batch]")
            keep_previous = (~reset_before.bool()).to(observations.dtype).unsqueeze(-1)
        normalized = self.normalize(observations)
        auxiliary = self.auxiliary(observations)
        hidden = torch.tanh(self.encoder(torch.cat((normalized,auxiliary),dim=-1)))
        final_states, all_inputs = [], []
        for index, layer in enumerate(self.recurrent.layers):
            candidate, gate, projection = layer(hidden).chunk(3, dim=-1)
            update = gate.sigmoid()
            a = (1.0 - update) * keep_previous
            b = update * self.recurrent._g(candidate)
            prefix_a, prefix_b = affine_prefix(a, b)
            recurrent = prefix_a * initial_state[index].unsqueeze(0) + prefix_b
            if return_state_inputs:
                all_inputs.append(torch.cat((initial_state[index].unsqueeze(0), recurrent[:-1]), dim=0))
            hidden = self.recurrent._highway(hidden, recurrent, projection)
            final_states.append(recurrent[-1])
        mean, value = self.heads(hidden, normalized, auxiliary)
        return PolicySequence(mean, value,
                              torch.stack(final_states, dim=0),
                              torch.stack(all_inputs, dim=1) if return_state_inputs else None)

    def deterministic(self, observations: torch.Tensor, state: torch.Tensor,
                      reset_before: torch.Tensor | None = None) -> tuple[torch.Tensor, torch.Tensor]:
        result = self.step(observations, state, reset_before=reset_before, deterministic=True)
        return result.actions, result.state
