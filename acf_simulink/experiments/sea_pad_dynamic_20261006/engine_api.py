"""Compact streaming C experiment interface for the versioned SEA/pad study.

This is a separately reviewed development model, not a replay of frozen V8.
The root-owned study_contract.json is authoritative for study assumptions.
"""
from __future__ import annotations

import ctypes as ct
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / "acf_simulink/validation/sea_pad_dynamic_20261006/engine"

FIELDS = [
    "actuator", "pad_points", "dt", "duration", "force_target", "mass_a",
    "mass_b", "mass_s", "gravity", "angle_deg", "upper_k", "cell_k",
    "pad_k", "pad_c", "pad_diameter", "pad_thickness", "pad_compression_limit",
    "bump_height", "bump_width", "scan_speed", "bump_start", "nut_mass",
    "lead", "rotor_inertia", "sea_k", "sea_c", "spring_deflection_limit",
    "nut_travel_limit", "tool_travel_limit", "force_constant", "resistance",
    "inductance", "current_response", "voltage_limit", "current_limit",
    "continuous_current", "guide_coulomb", "guide_viscous", "guide_eps",
    "shaft_coulomb", "shaft_viscous", "shaft_eps", "analog_cutoff",
    "adc_rate", "control_rate", "ema_time", "pipeline_time", "force_noise_rms",
    "accel_noise_rms", "position_noise_rms", "noise_reference_enbw",
    "force_quantum", "accel_quantum", "position_quantum", "force_bias",
    "accel_bias", "position_bias", "force_scale", "accel_scale",
    "identified_mass_s", "force_kp", "force_ki", "velocity_gain", "inertia_gain",
    "inertia_cutoff", "velocity_leak", "servo_hz", "servo_zeta",
    "servo_target_cutoff", "encoder_quantum", "noise_enabled", "seed",
    "robot_enabled", "robot_gate_rms", "robot_gate_mean", "robot_dwell",
    "robot_rate", "robot_delay", "robot_tau", "robot_max_velocity",
    "robot_gain", "robot_soft_band", "external_current", "external_frequency",
    "external_amplitude", "external_start", "external_robot_amplitude",
    "external_robot_frequency",
    "servo_velocity_feedforward", "force_feedback_cutoff",
]


class Parameters(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in FIELDS]


METRIC_FIELDS = [
    "steps", "stop_code", "stop_time", "force_peak_error", "force_rms_error",
    "force_peak_time", "force_min", "force_max", "current_peak", "current_rms",
    "voltage_peak_request", "voltage_clipped_fraction", "current_clipped_fraction",
    "tool_peak_deviation", "nut_peak_deviation", "spring_peak_deflection",
    "pad_peak_compression", "force_reconstruction_peak_error", "force_balance_peak",
    "centering_band_fraction", "robot_command_count", "runtime_s", "adc_count",
    "control_count", "force_noise_filtered_rms", "accel_noise_filtered_rms",
    "flat_peak_error", "bump_window_peak_error", "nut_velocity_peak", "rotor_speed_peak_rpm",
    "cell_raw_min", "cell_raw_max", "specific_acceleration_peak", "kinematic_acceleration_peak",
]


class Metrics(ct.Structure):
    _fields_ = [(name, ct.c_double) for name in METRIC_FIELDS]


TRACE_COLUMNS = [
    "time_s", "contact_total_axial_N", "force_estimate_N", "cell_raw_N",
    "acceleration_s_m_s2", "current_A", "current_command_A", "voltage_request_V",
    "xA_m", "vA_m_s", "xB_m", "vB_m_s", "xS_m", "vS_m_s", "q_m", "vq_m_s",
    "force_actuator_N", "robot_R_m", "robot_velocity_m_s", "pad_compression_m",
    "force_requested_N", "velocity_s_estimate_m_s", "measured_position_s_a_m",
]

STOP_CODES = {0: "completed", 1: "contact_loss", 2: "tool_travel_stop",
              3: "nut_travel_stop", 4: "spring_working_travel_stop",
              5: "pad_compression_limit", 6: "nonfinite_state"}


def compile_engine() -> ct.CDLL:
    OUT.mkdir(parents=True, exist_ok=True)
    src = HERE / "engine.c"
    digest = hashlib.sha256(src.read_bytes()).hexdigest()
    binary = OUT / f"engine_{digest[:16]}.so"
    stamp = OUT / "build.json"
    current = json.loads(stamp.read_text()) if stamp.exists() else {}
    if current.get("source_sha256") != digest or not binary.exists():
        command = ["gcc", "-O3", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC",
                   "-shared", str(src), "-lm", "-o", str(binary)]
        proc = subprocess.run(command, text=True, capture_output=True, check=False)
        if proc.returncode:
            raise RuntimeError(proc.stdout + proc.stderr)
        stamp.write_text(json.dumps({"source_sha256": digest, "compiler_command": command,
                                    "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
                                    "flags": "No fast-math; double precision."}, indent=2) + "\n")
    lib = ct.CDLL(str(binary))
    lib.engine_simulate.argtypes = [ct.POINTER(Parameters), ct.POINTER(Metrics),
                                   ct.POINTER(ct.c_double), ct.c_int, ct.c_int]
    lib.engine_simulate.restype = ct.c_int
    lib.engine_initialize.argtypes = [ct.POINTER(Parameters), ct.POINTER(ct.c_double),
                                     ct.POINTER(ct.c_double)]
    lib.engine_rhs.argtypes = [ct.POINTER(Parameters), ct.c_double,
                              ct.POINTER(ct.c_double), ct.c_double, ct.c_double,
                              ct.POINTER(ct.c_double), ct.POINTER(ct.c_double)]
    lib.engine_replay.argtypes = [ct.POINTER(Parameters), ct.POINTER(ct.c_double), ct.c_int,
                                 ct.POINTER(ct.c_double), ct.c_int]
    lib.engine_replay.restype = ct.c_int
    return lib


def make_parameters(config: dict) -> Parameters:
    missing = set(FIELDS) - set(config)
    unknown = set(config) - set(FIELDS)
    if missing or unknown:
        raise ValueError(f"Parameter names differ: missing={sorted(missing)}, unknown={sorted(unknown)}")
    return Parameters(**{name: float(config[name]) for name in FIELDS})


def simulate(lib: ct.CDLL, config: dict, trace_rate: float = 0) -> tuple[dict, np.ndarray]:
    p = make_parameters(config)
    m = Metrics()
    if trace_rate:
        stride = max(1, round(1 / (config["dt"] * trace_rate)))
        count = round(config["duration"] / config["dt"]) // stride + 4
        trace = np.empty((count, len(TRACE_COLUMNS)), dtype=np.float64)
        pointer = trace.ctypes.data_as(ct.POINTER(ct.c_double))
    else:
        stride, count, trace, pointer = 0, 0, np.empty((0, len(TRACE_COLUMNS))), None
    recorded = lib.engine_simulate(ct.byref(p), ct.byref(m), pointer, count, stride)
    if recorded < 0:
        raise ValueError(f"Engine rejected timing or parameters (code {recorded}).")
    metrics = {name: float(getattr(m, name)) for name in METRIC_FIELDS}
    metrics["stop_reason"] = STOP_CODES[round(m.stop_code)]
    metrics["force_pass"] = m.stop_code == 0 and m.force_peak_error <= 0.5
    metrics["completed"] = m.stop_code == 0
    metrics["static_current_A"] = abs((config["force_target"] -
        (config["mass_b"] + config["mass_s"] +
         (config["nut_mass"] if config["actuator"] != 0 else 0)) * config["gravity"] *
        np.cos(np.deg2rad(config["angle_deg"]))) /
        (config["force_constant"] if config["actuator"] == 0 else
         config["force_constant"] * 2*np.pi/config["lead"]))
    metrics["continuous_static_current_pass"] = (
        bool(metrics["static_current_A"] <= config["continuous_current"])
        if config["continuous_current"] > 0 else None)
    metrics["continuous_rms_current_pass"] = (
        bool(metrics["current_rms"] <= config["continuous_current"])
        if config["continuous_current"] > 0 else None)
    metrics["copper_loss_average_W"] = metrics["current_rms"]**2 * config["resistance"]
    metrics["actuator_ratings_pass"] = bool(metrics["current_peak"] <= config["current_limit"] + 1e-6 and
        (config["actuator"] == 0 or metrics["rotor_speed_peak_rpm"] <= 12000))
    metrics["sensor_ratings_pass"] = bool(max(abs(metrics["cell_raw_min"]),abs(metrics["cell_raw_max"])) <= 100 and
                                         metrics["specific_acceleration_peak"] <= 8*9.80665)
    return metrics, trace[:recorded].copy()


def write_trace(path: Path, trace: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(path, trace, delimiter=",", header=",".join(TRACE_COLUMNS), comments="", fmt="%.12g")


def physical_replay(lib: ct.CDLL, config: dict, commands: np.ndarray) -> np.ndarray:
    p=make_parameters(config)
    commands=np.ascontiguousarray(commands,dtype=np.float64)
    count=round(config["duration"]/config["dt"])+1
    result=np.empty((count,10),dtype=np.float64)
    copied=lib.engine_replay(ct.byref(p),commands.ctypes.data_as(ct.POINTER(ct.c_double)),
                             len(commands),result.ctypes.data_as(ct.POINTER(ct.c_double)),count)
    if copied<0:raise ValueError(f"Physical replay rejected inputs: code{copied}")
    return result[:copied]
