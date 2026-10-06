"""Exact, read-only decomposition of retained native v7 controller traces.

The plant's true geometry and physical acceleration are diagnostic inputs
only. No simulations or controller changes are performed by this script.
"""
from pathlib import Path
import hashlib
import json

import numpy as np
from scipy.io import loadmat
from scipy.signal import lfilter, periodogram


ROOT = Path(__file__).resolve().parents[2]
VALIDATION = ROOT / "acf_simulink/validation"
STEM = Path(__file__).with_suffix("")
SOURCES = [
    ("current100_direct_FF", "band_limited_acceleration_feedforward", 1),
    ("current100_FF_LP1ms", "band_limited_acceleration_feedforward", 5),
    ("current50_direct_FF", "identified_fast_drive", 1),
    ("current50_predicted_FF", "identified_fast_drive", 2),
    ("current50_FF_LP1ms", "identified_fast_drive", 3),
]


def analyse(label, phase, trace_id):
    source = VALIDATION / (
        f"normal_mass_controller_inertial_{phase}_20260930_v7_finite_traces/"
        f"controller_{trace_id:02d}_worst_native.mat"
    )
    saved = loadmat(source, simplify_cells=True)
    r, row = saved["r"], saved["row"]
    p = r["parameters"]
    t, d = r["time"], r["data"]
    n, imu = r["normalContact"]["data"], r["accelerometer"]["data"]
    dt, h = p["force"]["sampleTime"], p["sim"]["step"]
    stride = round(dt / h)
    take = np.arange(0, len(t), stride)
    held = lambda values: np.repeat(values, stride)[:len(t)]
    kf = p["coil"]["forceConstant"]
    mass = p["mechanics"]["mB"] + p["mechanics"]["mS"]

    # This is the native PI recurrence, reconstructed for diagnosis only.
    assert not np.any(d[:, 21] > .5), "Clipping would require the antiwindup recurrence."
    e = n[take, 3] - p["force"]["reference"]
    integral = np.r_[0., np.cumsum(e[:-1]) * dt]
    raw_pi = -p["inertial"]["forceKp"] * e - p["inertial"]["forceKi"] * integral
    lp = p["d"]["feedbackFilter"]
    pi = held(lfilter(np.atleast_1d(lp["numerator"]), np.atleast_1d(lp["denominator"]), raw_pi))

    a = np.exp(-dt / p["sensor"]["filterTime"])
    robot_a = lfilter([1 - a], [1, -a], r["commandPrediction"]["data"][take, 2])
    robot_a = np.r_[0., robot_a[:-1]]
    guide_v = held(r["motionObserver"]["data"][take, 1] +
                   p["inertial"]["guideVelocityPrediction_s"] * (robot_a - imu[take, 0]))
    friction = lambda velocity: (p["coil"]["viscousFriction"] * velocity +
                                p["coil"]["coulombForce"] *
                                np.tanh(velocity / p["coil"]["frictionSpeed"]))
    az_ff = r["accelerationFeedforward"]["data"][:, -1]
    raw_current = (p["d"]["coilHoldCurrent"] + pi - mass * az_ff / kf +
                   (n[:, 5] - p["force"]["reference"]) / kf + friction(guide_v) / kf)
    command_residual = float(np.max(np.abs(raw_current - d[:, 32])))
    assert command_residual < 1e-11, command_residual
    assert not p["inertial"]["currentNotch"]["enabled"]

    geometry = np.sqrt(1 + n[:, 2] ** 2) / (1 - p["contact"]["mu"] * n[:, 2])
    error = n[:, 0] - p["force"]["reference"]
    terms = {
        "currentTracking_N": geometry * kf * (d[:, 13] - d[:, 32]),
        "accelerationFeedforwardError_N": geometry * mass * (imu[:, 2] - az_ff),
        "normalProjectionFeedforward_N": geometry * (n[:, 5] - p["force"]["reference"] / geometry),
        "instantaneousGuideFeedforward_N": geometry * (friction(guide_v) - friction(d[:, 2])),
        "filteredPIcorrection_N": geometry * kf * pi,
        "finiteCellInertia_N": geometry * p["mechanics"]["mB"] * (d[:, 11] - imu[:, 2]),
    }
    force_residual = float(np.max(np.abs(sum(terms.values()) - error)))
    assert force_residual < 1e-10, force_residual
    peak = int(np.argmax(np.abs(error)))
    entry = np.abs(t - p["surface"]["startTime"]) < .006
    entry_peak = np.where(entry)[0][np.argmax(np.abs(error[entry]))]
    flat = (t > 2.4) & (t < 3.8)
    flat_peak = np.where(flat)[0][np.argmax(np.abs(error[flat]))]

    def peak_values(index):
        window = np.abs(t - t[index]) <= .025
        frequency, power = periodogram(terms["finiteCellInertia_N"][window],
                                      fs=1/h, window="hann", detrend="linear")
        mode = (frequency > 900) & (frequency < 2200)
        maximum_mode = float(frequency[np.where(mode)[0][np.argmax(power[mode])]])
        return {
            "time_s": float(t[index]), "signedError_N": float(error[index]),
            "terms_N": {key: float(values[index]) for key, values in terms.items()},
            "localCellPeriodogramPeak_Hz": maximum_mode,
            "physicalSAcceleration_m_s2": float(imu[index, 2]),
            "measuredSAcceleration_m_s2": float(imu[index, 0]),
            "finalFFAcceleration_m_s2": float(az_ff[index]),
            "physicalBAcceleration_m_s2": float(d[index, 11]),
        }

    window = np.abs(t - t[peak]) <= .025
    columns = {"time_s": t, "normalError_N": error, **terms,
               "physicalSAcceleration_m_s2": imu[:, 2],
               "measuredSAcceleration_m_s2": imu[:, 0], "finalFFAcceleration_m_s2": az_ff,
               "physicalBAcceleration_m_s2": d[:, 11],
               "actualCurrent_A": d[:, 13], "issuedCurrent_A": d[:, 32]}
    excerpt = STEM.with_name(STEM.name + "_" + label).with_suffix(".csv")
    np.savetxt(excerpt, np.column_stack([v[window] for v in columns.values()]),
               delimiter=",", header=",".join(columns), comments="")
    return {
        "label": label, "sourceTrace": str(source),
        "sourceTraceSha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "nativeSamples": len(t), "nativeStep_s": h,
        "currentResponse_s": p["coil"]["currentResponse"],
        "ffFilter_s": p["inertial"]["accelerationFeedforwardFilter_s"],
        "ffPrediction_s": p["inertial"]["accelerationPrediction_s"],
        "massS_kg": p["mechanics"]["mS"], "seed": p["sensor"]["seed"],
        "reference_N": p["force"]["reference"], "angle_deg": row["angle_deg"],
        "mu": p["contact"]["mu"], "forceKp_A_N": p["inertial"]["forceKp"],
        "feedbackFilterDamping": p["inertial"]["feedbackFilter"]["damping"],
        "fullPeak": peak_values(peak), "entryPeak": peak_values(entry_peak),
        "settledFlatPeak": peak_values(flat_peak),
        "flatRmsError_N": float(np.sqrt(np.mean(error[flat] ** 2))),
        "flatRmsCellInertia_N": float(np.sqrt(np.mean(terms["finiteCellInertia_N"][flat] ** 2))),
        "issuedCurrentReconstructionResidual_A": command_residual,
        "normalForceDecompositionResidual_N": force_residual,
        "voltageClipFraction": float(np.mean(d[:, 22] > .5)),
        "effortClipFraction": float(np.mean(d[:, 21] > .5)),
        "excerpt": str(excerpt),
    }


traces = [analyse(*item) for item in SOURCES]
report = {
    "status": "complete_read_only_analysis",
    "analysisScript": str(Path(__file__).resolve()),
    "analysisSha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    "scope": "Exact decomposition of five retained native four-second v7 histories; zero new simulations. Each source is its controller's worst endpoint trace, so seeds may differ across rows. Physical truth is diagnostic only.",
    "interpretation": "Faster identified current response and causal acceleration pre-emphasis can raise the sampled cell/noise contribution even without voltage/current clipping. These are finite-cell/noise/phase limitations; a higher voltage does not directly change the identified current-loop response. Local periodograms are descriptive and do not prove exact closed-loop poles.",
    "traces": traces,
}
STEM.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps([{key: row[key] for key in ["label", "seed", "fullPeak", "entryPeak", "flatRmsError_N", "flatRmsCellInertia_N", "issuedCurrentReconstructionResidual_A", "normalForceDecompositionResidual_N"]} for row in traces], indent=2))
