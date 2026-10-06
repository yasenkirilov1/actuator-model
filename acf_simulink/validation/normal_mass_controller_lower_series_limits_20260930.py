"""Read-only native force decomposition for the lower-series candidate.

Pass one or more retained MAT traces as arguments. All physical quantities
used here are diagnostics; this script does not run or alter the controller.
"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from scipy.io import loadmat
from scipy.signal import lfilter


FOLDER = Path(__file__).resolve().parent
STEM = Path(__file__).with_suffix("")
sources = [Path(arg).resolve() for arg in sys.argv[1:]]
if not sources:
    sources = [FOLDER / "normal_mass_controller_inertial_endpoint_confirmation_20260930_lower_series_v1_worst_native_trace.mat"]
rows = []
for index, source in enumerate(sources, 1):
    saved = loadmat(source, simplify_cells=True)
    r = saved["r"] if "r" in saved else saved["worst"]
    p = r["parameters"]
    t, d = r["time"], r["data"]
    normal, imu = r["normalContact"]["data"], r["accelerometer"]["data"]
    inverse = r["lowerInverse"]["data"]
    dt = p["force"]["sampleTime"]
    stride = round(dt / p["sim"]["step"])
    take = np.arange(0, len(t), stride)
    held = lambda value: np.repeat(value, stride)[:len(t)]
    kf = p["coil"]["forceConstant"]
    mass = p["mechanics"]["mB"] + p["mechanics"]["mS"]
    assert not p["inertial"]["currentNotch"]["enabled"] and not np.any(d[:, 21] > .5)
    error_measured = normal[take, 3] - p["force"]["reference"]
    integral = np.r_[0., np.cumsum(error_measured[:-1]) * dt]
    pi_request = -p["inertial"]["forceKp"] * error_measured - p["inertial"]["forceKi"] * integral
    feedback = p["d"]["feedbackFilter"]
    pi = held(lfilter(feedback["numerator"], feedback["denominator"], pi_request))
    a = np.exp(-dt / p["sensor"]["filterTime"])
    robot_a = lfilter([1-a], [1, -a], r["commandPrediction"]["data"][take, 2])
    delay = round(p["sensor"]["delay"] / dt)
    if delay:
        robot_a = np.r_[np.zeros(delay), robot_a[:-delay]]
    predicted_gap_v = held(r["motionObserver"]["data"][take, 1] +
                           p["inertial"]["guideVelocityPrediction_s"] * (robot_a - imu[take, 0]))
    friction = lambda velocity: p["coil"]["viscousFriction"] * velocity + p["coil"]["coulombForce"] * np.tanh(velocity / p["coil"]["frictionSpeed"])
    ff_az = r["accelerationFeedforward"]["data"][:, -1]
    command = (p["d"]["coilHoldCurrent"] + pi - mass * ff_az / kf +
               (normal[:, 5] - p["force"]["reference"]) / kf +
               friction(predicted_gap_v) / kf + inverse[:, 3])
    command_residual = float(np.max(np.abs(command - d[:, 32])))
    assert command_residual < 1e-11, command_residual
    geometry = np.sqrt(1 + normal[:, 2]**2) / (1 - p["contact"]["mu"] * normal[:, 2])
    error = normal[:, 0] - p["force"]["reference"]
    terms = {
        "currentTracking_N": geometry * kf * (d[:, 13] - d[:, 32]),
        "accelerationFeedforwardError_N": geometry * mass * (imu[:, 2] - ff_az),
        "normalProjectionFeedforward_N": geometry * (normal[:, 5] - p["force"]["reference"] / geometry),
        "guideFeedforwardError_N": geometry * (friction(predicted_gap_v) - friction(d[:, 2])),
        "filteredPIcorrection_N": geometry * kf * pi,
        "lowerLinkBInertia_N": geometry * p["mechanics"]["mB"] * (d[:, 11] - imu[:, 2]),
        "lowerLinkInverseFeedforward_N": geometry * kf * inverse[:, 3],
    }
    residual = float(np.max(np.abs(sum(terms.values()) - error)))
    assert residual < 1e-10, residual
    peak = int(np.argmax(np.abs(error)))
    start, duration = p["surface"]["startTime"], p["d"]["duration"]
    if p["surface"]["kind"] == "repeated":
        phase = (t - start) % p["surface"]["repeatPeriod"]
        bump_window = (t >= start-.05) & ((phase <= duration+.2) | (phase >= p["surface"]["repeatPeriod"]-.05))
    else:
        bump_window = (t >= start-.05) & (t <= start+duration+.2)
    flat = (t > .25) & ~bump_window
    excerpt_window = np.abs(t - t[peak]) <= .03
    columns = {"time_s": t, "normalError_N": error, **terms,
               "physicalBAcceleration_m_s2": d[:, 11], "physicalSAcceleration_m_s2": imu[:, 2],
               "measuredSAcceleration_m_s2": imu[:, 0], "finalFFAcceleration_m_s2": ff_az,
               "physicalCoilGapVelocity_m_s": d[:, 2], "predictedGuideVelocity_m_s": predicted_gap_v,
               "inverseCurrent_A": inverse[:, 3], "actualCurrent_A": d[:, 13], "commandCurrent_A": d[:, 32]}
    excerpt = STEM.with_name(STEM.name + f"_trace_{index:02d}").with_suffix(".csv")
    np.savetxt(excerpt, np.column_stack([value[excerpt_window] for value in columns.values()]),
               delimiter=",", header=",".join(columns), comments="")
    rows.append({
        "sourceTrace": str(source), "sourceTraceSha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "massS_kg": p["mechanics"]["mS"], "reference_N": p["force"]["reference"],
        "angle_deg": p["mechanics"]["angle"]*180/np.pi, "mu": p["contact"]["mu"],
        "seed": p["sensor"]["seed"], "nativeSamples": len(t), "nativeStep_s": p["sim"]["step"],
        "lowerLinkStiffness_N_m": p["mechanics"]["k1"], "lowerLinkDamping_Ns_m": p["mechanics"]["c1"],
        "normalPeakErrorSigned_N": float(error[peak]), "peakTime_s": float(t[peak]),
        "peakOccursDuringBumpOrRecovery": bool(bump_window[peak]),
        "termsAtPeak_N": {key: float(values[peak]) for key, values in terms.items()},
        "combinedLowerLinkAndInverseAtPeak_N": float(terms["lowerLinkBInertia_N"][peak] + terms["lowerLinkInverseFeedforward_N"][peak]),
        "settledFlatRmsError_N": float(np.sqrt(np.mean(error[flat]**2))),
        "settledFlatPeakError_N": float(np.max(np.abs(error[flat]))),
        "peakEstimatedNormalError_N": float(np.max(np.abs(normal[:, 3]-normal[:, 0]))),
        "peakCurrent_A": float(np.max(np.abs(d[:, 13]))), "peakVoltage_V": float(np.max(np.abs(d[:, 14]))),
        "voltageClipFraction": float(np.mean(d[:, 22] > .5)),
        "effortClipFraction": float(np.mean(np.abs(d[:, 32]) >= p["coil"]["peakCurrent"]-1e-10)),
        "commandResidual_A": command_residual, "normalDecompositionResidual_N": residual,
        "excerpt": str(excerpt),
    })

report = {
    "status": "complete_read_only_analysis", "analysisScript": str(Path(__file__).resolve()),
    "analysisSha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    "scope": "Retained full native histories for the external lower spring/damper and causal proper inverse feedforward candidate. Zero new simulations; physical truth appears only in this diagnostic decomposition.",
    "cellInterpretation": "The soft lower link represents the spring/damper assembly. Catalog cell stiffness is a massless series reference; no separately resolved high-frequency cell/mount mode is qualified.",
    "rows": rows,
}
STEM.with_suffix(".json").write_text(json.dumps(report, indent=2)+"\n")
print(json.dumps(rows, indent=2))
