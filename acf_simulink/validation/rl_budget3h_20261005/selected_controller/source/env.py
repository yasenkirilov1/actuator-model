"""Causal feature/action mapping and rewards for the one native RL plant.

This file contains no physical integration. NativePlant owns the generated
Simulink graph. True interval extrema are used only by rewards and scoring.
Fixed feature scaling is shared by training, selection and held-out replay.
"""
from __future__ import annotations

import ctypes as ct
from dataclasses import dataclass, asdict
import math
from pathlib import Path
import subprocess

import numpy as np

from policy_export import export_policy, flat_weights, numpy_sample


DT = 1e-4
CURRENT_SCALE_A = 2.0
ROBOT_SCALE_M_S = 0.010
FEATURE_NAMES = [
    "estimated_normal_error_over_0p5N", "inertial_axial_error_over_5N",
    "inertial_lateral_over_50N", "measured_S_acceleration_z_over_10m_s2",
    "measured_S_acceleration_x_over_10m_s2", "relative_position_over_25mm",
    "estimated_relative_velocity_over_20mm_s", "estimated_travel_error_over_5mm",
    "estimated_slope_over_0p2", "issued_robot_prediction_velocity_over_5mm_s",
    "issued_robot_prediction_acceleration_over_0p2m_s2",
    "aligned_issued_robot_prediction_velocity_over_5mm_s",
    "aligned_issued_robot_prediction_acceleration_over_0p2m_s2",
    "issued_current_error_over_2A", "issued_robot_velocity_request_over_5mm_s",
    "issued_robot_correction_over_5mm_s", "force_gate_ready",
    "causal_recenter_error_over_5mm", "force_target_over_50N",
    "identified_S_mass_over_2kg", "known_B_mass_over_1kg", "known_A_mass_over_2kg",
    "sin_fixed_orientation", "cos_fixed_orientation", "identified_stiffness_over_20kN_m",
    "identified_damping_over_60Ns_m", "identified_Kf_over_25N_A",
    "known_hot_resistance_over_10ohm", "identified_inductance_over_4mH",
    "usable_voltage_over_32V", "peak_current_limit_over_3p5A",
    "travel_range_over_25mm", "known_equilibrium_current_over_3p5A",
    "identified_current_response_over_100us", "identified_robot_delay_over_80ms",
    "identified_robot_response_over_60ms", "identified_AA_cutoff_over_8kHz",
    "known_EMA_time_over_50us", "identified_pipeline_over_20us",
    "robot_decision_clock_phase",
]


def feature_array(observations: np.ndarray, scenarios: list[dict],
                  episode_steps: np.ndarray) -> np.ndarray:
    """Compatibility entry point; collectors reuse one cached FeatureEncoder."""
    return FeatureEncoder(scenarios).encode(observations, np.arange(len(scenarios)), episode_steps)


class FeatureEncoder:
    """Cache only identified case constants; vectorize causal sensor scaling.

    No true dynamic state, future surface profile, or baseline control action is
    stored. The identifiers select known context, never an episode-time input.
    """
    def __init__(self, scenarios: list[dict]):
        self.equilibrium = np.asarray([r["knownEquilibriumCurrent_A"] for r in scenarios], np.float64)
        self.scales = np.asarray([.5,5,50,10,10,.025,.02,.005,.2,.005,.2,.005,.2,
                                  CURRENT_SCALE_A,.005,.005,1,.005], np.float64)
        self.offsets = np.zeros((len(scenarios),18), np.float64)
        self.static = np.empty((len(scenarios),21), np.float32)
        for i,row in enumerate(scenarios):
            force, equilibrium, angle = row["force_N"], self.equilibrium[i], row["angle_rad"]
            self.offsets[i,[0,1,7,13]] = [force,force,row["midTravel_m"],equilibrium]
            self.static[i] = [
            force/50, row["identifiedMassS_kg"]/2, row["identifiedMassB_kg"], row["massA_kg"]/2,
            math.sin(angle), math.cos(angle), row["identifiedK_N_m"]/20000,
            row["identifiedC_N_s_m"]/60, row["identifiedKf_N_A"]/25,
            row["coilR_ohm"]/10, row["coilL_H"]/.004, row["usableVoltage_V"]/32,
            row["peakCurrent_A"]/3.5, (row["travelMax_m"]-row["travelMin_m"])/.025,
            equilibrium/3.5, row.get("identifiedCurrentResponse_s", .0001)/.0001,
            row.get("identifiedRobotDelay_s", .08)/.08,
            row.get("identifiedRobotResponse_s", .06)/.06,
            row.get("identifiedAntiAliasCutoff_Hz", 8000)/8000,
            row.get("identifiedEmaTime_s", .00005)/.00005,
            row.get("identifiedPipelineDelay_s", .00002)/.00002,
        ]

    def encode(self, observations, identifiers, episode_steps):
        observations = np.asarray(observations, np.float64)
        identifiers = np.asarray(identifiers, np.int64)
        if observations.shape != (len(identifiers),18):
            raise ValueError("native causal observations must be [N,18]")
        out = np.empty((len(identifiers),len(FEATURE_NAMES)), np.float32)
        out[:,:18] = (observations-self.offsets[identifiers])/self.scales
        out[:,18:39] = self.static[identifiers]
        out[:,39] = 2*(np.asarray(episode_steps)%100)/99-1
        if not np.isfinite(out).all():
            raise FloatingPointError("non-finite causal observation")
        np.clip(out,-100,100,out=out)
        return out

    def actions(self, normalized, identifiers):
        result=np.asarray(normalized,np.float64).copy()
        result[:,0] = result[:,0]*CURRENT_SCALE_A+self.equilibrium[identifiers]
        result[:,1] *= ROBOT_SCALE_M_S
        return np.ascontiguousarray(result)


def action_mask(observations: np.ndarray, episode_steps: np.ndarray) -> np.ndarray:
    mask = np.ones((len(observations), 2), np.float32)
    mask[:, 1] = (observations[:, 16] > .5) & (episode_steps % 100 == 0)
    return mask


def physical_actions(normalized: np.ndarray, scenarios: list[dict]) -> np.ndarray:
    result = np.asarray(normalized, dtype=np.float64).copy()
    result[:, 0] *= CURRENT_SCALE_A
    result[:, 0] += np.asarray([s["knownEquilibriumCurrent_A"] for s in scenarios])
    result[:, 1] *= ROBOT_SCALE_M_S
    return np.ascontiguousarray(result)


@dataclass
class RewardConfig:
    force_error_scale_N: float = .5
    force_mse_weight: float = 1.0
    force_excess_weight: float = 4.0
    force_running_peak_weight: float = 0.0
    recenter_weight: float = .1
    recenter_scale_m: float = .005
    effort_weight: float = .01
    effort_scale_A: float = 3.5
    current_change_weight: float = .001
    current_change_scale_A: float = .05
    robot_change_weight: float = .001
    robot_change_scale_m_s: float = .005
    physical_failure_penalty: float = 10.0
    return_scale: float = 1.0
    acceptable_travel_radius_m: float | None = None
    travel_excess_weight: float = 10.0
    inside_travel_weight: float = 0.0

    def __post_init__(self):
        if not np.isfinite(self.force_running_peak_weight) or self.force_running_peak_weight < 0:
            raise ValueError('force running-peak weight must be finite and nonnegative')

    def contract(self) -> dict:
        return {**asdict(self), "dt_s": DT,
                "force_truth": "Native actual workpiece-normal force, every physical tick incl startup.",
                "running_reward": "Negative elapsed time times weighted native mean squared normal error, interval force peak excess squared, configured travel objective, true current squared and issued-action changes squared.",
                "force_running_peak_reward": "Optional negative weight times the increase in squared episode-running true normal-force peak, divided by force scale squared, without dt. Startup is charged on the first decision. Undiscounted increments telescope to the full episode peak squared; PPO still applies its configured discount/GAE. Episode peak and step counter carry across rollouts and checkpoints. This truth is reward-only.",
                "failure": "Contact loss/travel stop terminates with negative fixed penalty; horizon truncation bootstraps final causal observation.",
                "travel_reward": ("Legacy native mean squared centering error." if self.acceptable_travel_radius_m is None else
                    "Zero/default optional weak mean-square cost inside the acceptable band; outside it, dt times squared interval-peak excess from native min/max travel. This is an interval-peak penalty, not an excess MSE."),
                "legacy_recenter_weight_used": self.acceptable_travel_radius_m is None,
                "excluded": "No teacher action, future profile, inferred-force-only reward or sample-rate-dependent discount."}


def force_running_peak_cost(previous_peak, running_peak, cfg):
    """Undiscounted episode-maximum cost increment; never an actor input."""
    if cfg.force_running_peak_weight == 0:
        return 0.0
    if previous_peak is None or running_peak is None:
        raise ValueError('running-peak reward needs prior charged and current episode peaks')
    previous_peak = np.asarray(previous_peak)
    running_peak = np.asarray(running_peak)
    if np.any(previous_peak < 0) or np.any(running_peak < previous_peak):
        raise ValueError('episode-running peaks must be nonnegative and nondecreasing')
    return cfg.force_running_peak_weight * (running_peak**2 - previous_peak**2) / cfg.force_error_scale_N**2


def travel_cost(minimum, maximum, mean_squared_error, mid_travel, cfg):
    """Reward only: exact native interval extrema determine band excess."""
    if cfg.acceptable_travel_radius_m is None:
        return cfg.recenter_weight*np.asarray(mean_squared_error)/cfg.recenter_scale_m**2
    if cfg.acceptable_travel_radius_m<=0:raise ValueError('acceptable travel radius must be positive')
    if mid_travel is None:raise ValueError('band reward needs each case\'s known mid travel')
    peak=np.maximum(np.abs(np.asarray(minimum)-mid_travel),np.abs(np.asarray(maximum)-mid_travel))
    excess=np.maximum(peak-cfg.acceptable_travel_radius_m,0.0)
    return (cfg.inside_travel_weight*np.asarray(mean_squared_error)/cfg.recenter_scale_m**2
            +cfg.travel_excess_weight*(excess/cfg.recenter_scale_m)**2)


class CPolicy:
    """Small CPU inference with GPU-trained weights refreshed between rollouts."""
    def __init__(self, policy, directory: Path, batch: int, provenance=None):
        self.directory = Path(directory)
        export_policy(policy, self.directory, FEATURE_NAMES,
                      {**(provenance or {}),"physicalActionScales":{"currentOffset_A":CURRENT_SCALE_A,
                                                                  "robotRequest_m_s":ROBOT_SCALE_M_S}})
        subprocess.run(["cc", "-O2", "-std=c99", "-fPIC", "-shared",
                        str(self.directory/"policy_inference.c"), "-lm", "-o",
                        str(self.directory/"policy_inference.so")], check=True)
        self.library = ct.CDLL(str((self.directory/"policy_inference.so").resolve()))
        self.function = self.library.acf_unified_policy_step_many
        float_pointer = ct.POINTER(ct.c_float)
        self.function.argtypes = [ct.c_size_t, float_pointer, float_pointer,
                                  float_pointer, ct.POINTER(ct.c_ubyte),
                                  float_pointer, float_pointer]
        self.function.restype = None
        self.state = np.zeros((batch, policy.num_layers, policy.hidden_size), np.float32)
        self.refresh(policy)

    def refresh(self, policy):
        self.weights = np.ascontiguousarray(flat_weights(policy))
        self.log_std = policy.log_std.detach().clamp(-8,1).cpu().numpy().astype(np.float32)

    @staticmethod
    def pointer(array):
        return array.ctypes.data_as(ct.POINTER(ct.c_float))

    def infer(self, observations, reset_before=None, state=None):
        observations = np.ascontiguousarray(observations, np.float32)
        state = self.state if state is None else state
        if not state.flags.c_contiguous or state.shape[0] != len(observations):
            raise ValueError("C state is contiguous [N,L,H]")
        reset_before = np.ascontiguousarray(np.zeros(len(observations),np.uint8)
                                           if reset_before is None else reset_before, dtype=np.uint8)
        means = np.empty((len(observations),2), np.float32)
        values = np.empty(len(observations), np.float32)
        self.function(len(observations), self.pointer(self.weights),self.pointer(observations),
                      self.pointer(state),reset_before.ctypes.data_as(ct.POINTER(ct.c_ubyte)),
                      self.pointer(means),self.pointer(values))
        return means, values

    def sample(self, observations, rng, resets, masks, deterministic=False):
        means, values = self.infer(observations, resets)
        result = numpy_sample(means,self.log_std,rng,masks,deterministic)
        return {**result,"means":means,"values":values}
