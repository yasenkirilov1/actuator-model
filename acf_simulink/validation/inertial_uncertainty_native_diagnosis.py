"""Read-only force-balance/peak diagnostics for retained native MAT histories.

Usage: python inertial_uncertainty_native_diagnosis.py TRACE.mat OUTPUT_PREFIX
Scoring always uses every native true-normal-force sample, never the plots.
This file does not simulate a plant or supply physical truth to a controller.
"""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from scipy.io import loadmat
from scipy.signal import butter, detrend, sosfiltfilt


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(trace_path, output_prefix):
    trace_path, output_prefix = Path(trace_path), Path(output_prefix)
    loaded = loadmat(trace_path, simplify_cells=True)
    r = loaded.get("r", loaded.get("worst"))
    assert r is not None, "Expected retained r or worst native history"
    p = r["parameters"]
    t, d = r["time"], r["data"]
    n, a, motion = (r[k]["data"] for k in
                    ("normalContact", "accelerometer", "motionObserver"))
    error = n[:, 0] - p["force"]["reference"]
    ix = int(np.argmax(np.abs(error)))
    dt = float(np.median(np.diff(t)))
    b, s = p["mechanics"]["mB"], p["mechanics"]["mS"]
    kf, coil = p["coil"]["forceConstant"], p["coil"]
    factor = np.sqrt(1 + n[:, 2] ** 2) / (1 - p["contact"]["mu"] * n[:, 2])
    friction = (coil["coulombForce"] * np.tanh(d[:, 2] / coil["frictionSpeed"])
                + coil["viscousFriction"] * d[:, 2])
    desired_rz = p["force"]["reference"] / factor
    ideal_rigid_current = (desired_rz - (b + s) * (p["d"]["g"] + d[:, 29])
                           + friction) / kf
    tracking = factor * kf * (d[:, 13] - d[:, 32])
    command = factor * kf * (d[:, 32] - ideal_rigid_current)
    cell = factor * b * (d[:, 11] - d[:, 29])
    window = np.abs(t - t[ix]) <= .030000001
    fft_values = np.abs(np.fft.rfft(detrend(error[window])))
    frequencies = np.fft.rfftfreq(sum(window), dt)
    band = (frequencies >= 1000) & (frequencies <= 2200)
    dominant = frequencies[band][np.argmax(fft_values[band])]
    ringing = sosfiltfilt(butter(4, [1000, 2200], btype="bandpass", fs=1 / dt,
                               output="sos"), error)
    near_peak = np.abs(t - t[ix]) <= .003000001
    gate = r["robotGate"]
    gate_index = np.clip(np.searchsorted(gate["time"], t, side="right") - 1,
                         0, len(gate["time"]) - 1)
    ready = gate["ready"][gate_index]
    mean_gate_ready = ready.copy()
    variation_enabled = p["robot"]["forceGate"].get("variation", {}).get("enabled", False)
    if variation_enabled and "forceVariation" in r:
        variation = r["forceVariation"]
        variation_index = np.clip(np.searchsorted(variation["time"], t, side="right") - 1,
                                  0, len(variation["time"]) - 1)
        ready = np.minimum(ready, variation["data"][variation_index, 4])
    starts = [p["surface"]["startTime"]]
    if p["surface"]["kind"] == "repeated":
        starts = np.arange(starts[0], t[-1], p["surface"]["repeatPeriod"])
    bumps = []
    quiet = t >= max(3.5, float(starts[0]) + p["d"]["duration"] + .2)
    for bump_index, start in enumerate(starts, 1):
        mask = (t >= start - .005) & (t <= start + p["d"]["duration"] + .005)
        peak_index = np.flatnonzero(mask)[np.argmax(np.abs(error[mask]))]
        bumps.append({"index": bump_index, "start_s": float(start), "peak_s": float(t[peak_index]),
                      "peakPhase_s": float(t[peak_index] - start),
                      "peakError_N": float(abs(error[peak_index])),
                      "estimatedErrorAtTruePeak_N": float(n[peak_index, 3] - p["force"]["reference"])})
        quiet &= ~((t >= start - .05) & (t <= start + p["d"]["duration"] + .2))
    peak = {
        "time_s": float(t[ix]), "signedNormalForceError_N": float(error[ix]),
        "normalForce_N": float(n[ix, 0]), "estimatedNormalForce_N": float(n[ix, 3]),
        "trueSlope": float(n[ix, 2]), "estimatedSlope": float(n[ix, 4]),
        "current_A": float(d[ix, 13]), "currentCommand_A": float(d[ix, 32]),
        "idealRigidCurrent_A": float(ideal_rigid_current[ix]),
        "actualCoilGapVelocity_m_s": float(d[ix, 2]),
        "actualHeadGapVelocity_m_s": float(d[ix, 7] - d[ix, 9]),
        "measuredObserverHeadGapVelocity_m_s": float(motion[ix, 1]),
        "actualAccelerationS_m_s2": float(d[ix, 29]),
        "measuredAccelerationS_m_s2": float(a[ix, 0]),
        "actualAccelerationB_m_s2": float(d[ix, 11]),
        "actualRobotVelocity_m_s": float(d[ix, 26]),
        "predictedRobotVelocity_m_s": float(n[ix, 7]),
        "robotGateReady": bool(ready[ix]),
        "meanRateRobotGateReady": bool(mean_gate_ready[ix]),
        "currentTrackingNormalError_N": float(tracking[ix]),
        "commandAndFeedforwardNormalError_N": float(command[ix]),
        "finiteCellBInertiaNormalError_N": float(cell[ix]),
        "BRelativeInertiaNormalError_N": float(cell[ix]),
    }
    result = {
        "scope": "Read-only exact force decomposition using actual B-S relative inertia. Full native true normal force history is scored. Calibration injections, if any, are defined in the source run, not this diagnostic.",
        "nativeTrace": str(trace_path.resolve()),
        "nativeTraceSha256": sha256(trace_path),
        "nativeSamples": len(t), "step_s": dt, "endTime_s": float(t[-1]),
        "parameters": {"massB_kg": float(b), "massS_kg": float(s),
                       "Kf_N_A": float(kf), "seed": int(p["sensor"]["seed"]),
                       "k1_N_m": float(p["mechanics"]["k1"]),
                       "c1_N_s_m": float(p["mechanics"]["c1"])},
        "peak": peak,
        "decompositionIdentityResidual_N": float(np.max(np.abs(error-tracking-command-cell))),
        "peakWindow_s": [float(t[window][0]), float(t[window][-1])],
        "peakWindowDominantForceFrequencyIn1000To2200Hz_Hz": float(dominant),
        "diagnosticRingingBand_Hz": [1000, 2200],
        "diagnosticBandAmplitudeWithin3ms_N": float(np.max(np.abs(ringing[near_peak]))),
        "diagnosticBandPeakToPeakWithin3ms_N": float(np.ptp(ringing[near_peak])),
        "diagnosticBandConvention": "Zero-phase offline 4th-order bandpass; diagnostic only, never a controller input or qualification score.",
        "BRelativeInertiaConvention": "B*(aB-aS) measures holder inertia relative to S. In a lower-series spring candidate it is the external compliant-link contribution; the legacy finiteCellBInertia key is retained for compatibility. The proper inverse intentionally commands extra current to counteract this term, so the individual term is not an independent unavoidable error.",
        "bumps": bumps,
    }
    if np.any(quiet):
        estimate_error = n[:, 3] - n[:, 0]
        result["quietFlatWindows"] = {
            "convention": "t>=3.5s; excludes 50ms before each bump through 200ms after its duration. Offline read-only summary.",
            "samples": int(sum(quiet)),
            "trueErrorMean_N": float(np.mean(error[quiet])),
            "trueErrorRms_N": float(np.sqrt(np.mean(error[quiet] ** 2))),
            "estimatedErrorMean_N": float(np.mean((n[:, 3] - p["force"]["reference"])[quiet])),
            "estimatedMinusTrueMean_N": float(np.mean(estimate_error[quiet])),
            "estimatedMinusTrueRms_N": float(np.sqrt(np.mean(estimate_error[quiet] ** 2))),
            "estimatedMinusTrueP95Absolute_N": float(np.percentile(np.abs(estimate_error[quiet]), 95)),
            "combinedGateReadyFraction": float(np.mean(ready[quiet])),
        }
    if variation_enabled and "forceVariation" in r:
        vi = int(variation_index[ix])
        result["peak"]["nativeVariationGate"] = {
            "loggedTime_s": float(variation["time"][vi]),
            "data": [float(value) for value in variation["data"][vi]],
            "schema": list(variation["names"]),
        }
    if len(starts) > 1:
        last_start = float(starts[np.searchsorted(starts, t[ix], side="right") - 1])
        phase = float(t[ix] - last_start)
        first_ix = int(np.argmin(np.abs(t - (float(starts[0]) + phase))))
        result["samePhaseFirstVersusWorstBump"] = {
            "worstBumpIndex": int(np.searchsorted(starts, t[ix], side="right")),
            "phaseAfterEntry_s": phase,
            "firstBumpTime_s": float(t[first_ix]),
            "firstTrueError_N": float(error[first_ix]),
            "firstEstimatedError_N": float(n[first_ix, 3] - p["force"]["reference"]),
            "firstEstimatedMinusTrue_N": float(n[first_ix, 3] - n[first_ix, 0]),
            "worstEstimatedMinusTrue_N": float(n[ix, 3] - n[ix, 0]),
            "firstMeasuredAccelZ_m_s2": float(a[first_ix, 0]),
            "worstMeasuredAccelZ_m_s2": float(a[ix, 0]),
            "firstActualAccelZ_m_s2": float(d[first_ix, 29]),
            "worstActualAccelZ_m_s2": float(d[ix, 29]),
            "staticCalibrationAndNoiseSettings": "Identical throughout the recording; different samples of the same seeded sensor-noise streams occur at later bumps.",
        }
    if "lowerInverse" in r:
        inverse = r["lowerInverse"]["data"]
        peak["nativeLowerInverseExtraCurrent_A"] = float(inverse[ix, 3])
        peak["nativeDesiredLowerLoadDeviation_N"] = float(inverse[ix, 2])
        low_band = (frequencies >= 20) & (frequencies <= 500)
        result["peakWindowDominantFrequencyIn20To500Hz_Hz"] = float(frequencies[low_band][np.argmax(fft_values[low_band])])
        result["finiteWindowFrequencyResolution_Hz"] = float(1 / (sum(window) * dt))
    output_prefix.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    columns = np.column_stack([t[window], error[window], n[window, 3] - p["force"]["reference"],
                               tracking[window], command[window], cell[window], d[window, 13], d[window, 32],
                               ideal_rigid_current[window], d[window, 2], d[window, 7] - d[window, 9],
                               motion[window, 1], d[window, 29], a[window, 0], d[window, 11],
                               d[window, 26], n[window, 7], ready[window], n[window, 2], n[window, 4]])
    np.savetxt(output_prefix.with_suffix(".csv"), columns, delimiter=",",
               header="time_s,normalError_N,estimatedError_N,currentTracking_N,commandAndFF_N,BcellInertia_N,current_A,command_A,idealRigidCurrent_A,coilGapVelocity_m_s,headGapVelocity_m_s,observerVelocity_m_s,Saccel_m_s2,measuredSaccel_m_s2,Baccel_m_s2,robotVelocity_m_s,predictedRobotVelocity_m_s,gateReady,trueSlope,estimatedSlope", comments="")
    print(json.dumps({"peak": peak,
                      "identityResidual_N": result["decompositionIdentityResidual_N"],
                      "dominantHighFrequency_Hz": float(dominant), "bumps": bumps}, indent=2))


if __name__ == "__main__":
    run(*sys.argv[1:])
