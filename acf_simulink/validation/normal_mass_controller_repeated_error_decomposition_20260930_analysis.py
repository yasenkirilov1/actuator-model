"""Read-only native trace decomposition; physical truth is diagnostic only."""
from pathlib import Path
import hashlib
import json
import numpy as np
from scipy.io import loadmat
from scipy.signal import lfilter, lfilter_zi, periodogram

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "acf_simulink/validation/inertial_uncertainty_finite_provisional/inertial_uncertainty_repeated_screen_20260930_native_traces/case_0001.mat"
STEM = Path(__file__).with_name("normal_mass_controller_repeated_error_decomposition_20260930")
r = loadmat(SOURCE, simplify_cells=True)["r"]
p = r["parameters"]
t, d = r["time"], r["data"]
normal, imu = r["normalContact"]["data"], r["accelerometer"]["data"]
sample_stride = round(p["force"]["sampleTime"] / p["sim"]["step"])
take = np.arange(0, len(t), sample_stride)
dt, Kf, mass = p["force"]["sampleTime"], p["coil"]["forceConstant"], p["mechanics"]["mB"] + p["mechanics"]["mS"]

alpha = np.exp(-dt / p["sensor"]["filterTime"])
robot_a = lfilter([1 - alpha], [1, -alpha], r["commandPrediction"]["data"][take, 2])
robot_a = np.r_[0, robot_a[:-1]]  # Native one-sample matched delay.
guide_v = r["motionObserver"]["data"][take, 1] + p["inertial"]["guideVelocityPrediction_s"] * (robot_a - imu[take, 0])
held = lambda x: np.repeat(x, sample_stride)[:len(t)]
guide_v = held(guide_v)
friction = lambda v: p["coil"]["viscousFriction"] * v + p["coil"]["coulombForce"] * np.tanh(v / p["coil"]["frictionSpeed"])
estimated_error = normal[take, 3] - p["force"]["reference"]
assert not np.any(d[:, 21] > .5), "Anti-windup reconstruction requires no effort clipping."
integral = np.r_[0, np.cumsum(estimated_error[:-1]) * dt]
pi = held(-p["inertial"]["forceKp"] * estimated_error - p["inertial"]["forceKi"] * integral)
raw_current = p["d"]["coilHoldCurrent"] + pi - mass * imu[:, 0] / Kf + (normal[:, 5] - p["force"]["reference"]) / Kf + friction(guide_v) / Kf
notch = p["d"]["currentNotch"]
filtered_current = lfilter(notch["numerator"], notch["denominator"], raw_current[take], zi=lfilter_zi(notch["numerator"], notch["denominator"]) * p["d"]["coilHoldCurrent"])[0]
current_reconstruction_residual = float(np.max(np.abs(filtered_current - d[take, 32])))
assert current_reconstruction_residual < 1e-12

slope = normal[:, 2]
geometry = np.sqrt(1 + slope ** 2) / (1 - p["contact"]["mu"] * slope)
error = normal[:, 0] - p["force"]["reference"]
components = {
    "currentTracking_N": geometry * Kf * (d[:, 13] - d[:, 32]),
    "totalCurrentNotch_N": geometry * Kf * (d[:, 32] - raw_current),
    "accelerationFeedforwardLag_N": geometry * mass * (imu[:, 2] - imu[:, 0]),
    "normalProjectionFeedforward_N": geometry * (normal[:, 5] - p["force"]["reference"] / geometry),
    "instantaneousGuideFeedforward_N": geometry * (friction(guide_v) - friction(d[:, 2])),
    "PIcorrection_N": geometry * Kf * pi,
    "finiteCellInertia_N": geometry * p["mechanics"]["mB"] * (d[:, 11] - imu[:, 2]),
}
identity_residual = float(np.max(np.abs(sum(components.values()) - error)))
assert identity_residual < 1e-10
peak = int(np.argmax(np.abs(error)))

def rising_crossings(values, mask, interpolate):
    ids = np.where(mask[:-1] & (values[:-1] < 0) & (values[1:] >= 0))[0]
    if interpolate:
        return [float(t[k] - values[k] * (t[k + 1] - t[k]) / (values[k + 1] - values[k])) for k in ids]
    return [float(t[k + 1]) for k in ids]

bumps = []
period, duration = p["surface"]["repeatPeriod"], p["d"]["duration"]
for number, start in enumerate(np.arange(p["surface"]["startTime"], t[-1], period), 1):
    ids = np.where((t >= start - .05) & (t <= start + duration + .2))[0]
    which = ids[np.argmax(np.abs(error[ids]))]
    center_mask = np.abs(t - (start + duration / 2)) < .006
    actual_zero = rising_crossings(d[:, 2], center_mask, True)
    predicted_zero = rising_crossings(guide_v, center_mask, False)
    bumps.append({
        "number": number, "startTime_s": float(start), "peakTime_s": float(t[which]),
        "signedError_N": float(error[which]), "peakAbsError_N": float(abs(error[which])),
        "componentsAtPeak_N": {k: float(v[which]) for k, v in components.items()},
        "actualGuideRisingZero_s": actual_zero, "predictedGuideRisingZero_s": predicted_zero,
        "predictionLeadAtCenter_us": (actual_zero[0] - predicted_zero[0]) * 1e6 if actual_zero and predicted_zero else None,
    })

window = np.abs(t - t[peak]) <= .025
freq, power = periodogram(components["finiteCellInertia_N"][window], fs=1 / p["sim"]["step"], window="hann", detrend="linear")
mode = (freq > 1400) & (freq < 1900)
dominant_mode_frequency = float(freq[np.where(mode)[0][np.argmax(power[mode])]])
csv_columns = {
    "time_s": t, "normalError_N": error, **components,
    "actualCoilGapVelocity_m_s": d[:, 2], "predictedGuideFFVelocity_m_s": guide_v,
    "actualCurrent_A": d[:, 13], "issuedCurrent_A": d[:, 32], "rawPIFFCurrent_A": raw_current,
    "physicalBAcceleration_m_s2": d[:, 11], "physicalSAcceleration_m_s2": imu[:, 2], "measuredSAcceleration_m_s2": imu[:, 0],
}
np.savetxt(STEM.with_suffix(".csv"), np.column_stack([v[window] for v in csv_columns.values()]), delimiter=",", header=",".join(csv_columns), comments="")
report = {
    "sourceTrace": str(SOURCE), "sourceTraceSha256": hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    "analysisScript": str(Path(__file__).resolve()), "analysisSha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    "scope": "Independent exact decomposition of one finite-cell ten-second repeated-bump diagnostic. No additional Simulink cases; no uncertainty injected. Physical truth channels are used for diagnosis only.",
    "nativeSamples": len(t), "step_s": p["sim"]["step"], "controllerSampleTime_s": dt,
    "peakError_N": float(abs(error[peak])), "peakTime_s": float(t[peak]),
    "componentsAtPeak_N": {k: float(v[peak]) for k, v in components.items()},
    "issuedCurrentReconstructionResidual_A": current_reconstruction_residual,
    "normalForceDecompositionResidual_N": identity_residual,
    "localCellBandDominantPeriodogramFrequency_Hz": dominant_mode_frequency,
    "frequencyInterpretation": "Finite local periodogram near the failing bump, not flat-background PSD or an exact pole identification.",
    "peakPhase": "Fifth bump center/descending side, approximately one loaded-cell natural period after guide reversal; bump maxima are nonmonotonic.",
    "voltageClipFraction": float(np.mean(d[:, 22] > .5)), "effortClipFraction": float(np.mean(d[:, 21] > .5)),
    "forceReference_N": p["force"]["reference"], "massS_kg": p["mechanics"]["mS"],
    "cellStiffness_N_m": p["mechanics"]["k1"], "cellDamping_Ns_m": p["mechanics"]["c1"],
    "guidePrediction_s": p["inertial"]["guideVelocityPrediction_s"],
    "guideCoulombForce_N": p["coil"]["coulombForce"], "guideFrictionTransitionVelocity_m_s": p["coil"]["frictionSpeed"],
    "bumps": bumps,
    "interpretation": "At the failing peak, finite B-S cell inertia dominates; the instantaneous guide mismatch has already subsided. Guide reversals and PI/noise can excite the undamped loaded-cell mode. These observations motivate feedback rolloff and separately measured structural damping; they do not establish a uniquely identified causal source or a hardware qualification.",
}
STEM.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps({k: report[k] for k in ["peakError_N", "peakTime_s", "componentsAtPeak_N", "issuedCurrentReconstructionResidual_A", "normalForceDecompositionResidual_N", "localCellBandDominantPeriodogramFrequency_Hz"]}, indent=2))
