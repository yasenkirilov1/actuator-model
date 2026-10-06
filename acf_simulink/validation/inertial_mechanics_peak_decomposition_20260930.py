"""Inspect retained native traces; this is analysis, not a plant solver."""
from pathlib import Path
import hashlib
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat
from scipy.signal import lfilter, welch

ROOT = Path(__file__).resolve().parent
STEM = "inertial_mechanics_peak_decomposition_20260930"


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dominant_frequency(t, values, lo, hi):
    use = (t >= lo) & (t <= hi)
    frequency, density = welch(values[use], fs=1 / (t[1] - t[0]),
                               nperseg=min(16384, int(use.sum())), detrend="linear")
    band = (frequency >= 500) & (frequency <= 4000)
    selected = np.flatnonzero(band)[np.argmax(density[band])]
    return {"window_s": [lo, hi], "band_Hz": [500, 4000],
            "dominantFrequency_Hz": float(frequency[selected]),
            "frequencyBinWidth_Hz": float(frequency[1] - frequency[0]),
            "rmsAboutWindowMean_N": float(np.std(values[use])),
            "peakAbsoluteError_N": float(np.max(np.abs(values[use])))}


def analyze(path, key, label):
    r = loadmat(path, simplify_cells=True)[key]
    t, d, p = r["time"], r["data"], r["parameters"]
    n, a, m = (r[name]["data"] for name in ("normalContact", "accelerometer", "motionObserver"))
    cp = r["commandPrediction"]["data"]
    reference = float(p["force"]["reference"])
    S, B = (float(p["mechanics"][name]) for name in ("mS", "mB"))
    Kf = float(p["coil"]["forceConstant"])
    mu = float(p["contact"]["mu"])
    Ts = float(p["sensor"]["sampleTime"])
    stride = int(round(Ts / (t[1] - t[0])))
    rho = np.exp(-Ts / float(p["sensor"]["filterTime"]))
    delay = int(p["d"]["sensorDelaySamples"])
    sampled = cp[::stride]
    aligned = lfilter([1 - rho], [1, -rho], sampled, axis=0)
    if delay:
        aligned = np.vstack((np.zeros((delay, 3)), aligned[:-delay]))
    aligned = np.repeat(aligned, stride, axis=0)[:len(t)]
    raw_sampled = np.repeat(sampled, stride, axis=0)[:len(t)]
    predictor_sampling_residual = float(np.max(np.abs(raw_sampled[:, 1] - n[:, 7])))
    assert predictor_sampling_residual < 1e-12
    guide_prediction = m[:, 1] + float(p["inertial"]["guideVelocityPrediction_s"]) * (aligned[:, 2] - a[:, 0])
    b, Cc, veps = (float(p["coil"][name]) for name in ("viscousFriction", "coulombForce", "frictionSpeed"))
    friction_true = b * d[:, 2] + Cc * np.tanh(d[:, 2] / veps)
    friction_ff = float(p["inertial"]["guideFrictionFeedforwardScale"]) * (
        b * guide_prediction + Cc * np.tanh(guide_prediction / veps))
    gravity = float(p["mechanics"]["gravity"]) * np.cos(float(p["mechanics"]["angle"]))
    true_geometry_factor = (1 - mu * n[:, 2]) / np.sqrt(1 + n[:, 2] ** 2)
    axial_reference_true = reference * true_geometry_factor
    axial_ff = n[:, 5] - (B + S) * gravity - (B + S) * a[:, 0] + friction_ff
    terms = {
        "currentTracking": Kf * (d[:, 13] - d[:, 32]),
        "postNotchCurrentMinusDirectFeedforward": Kf * d[:, 32] - axial_ff,
        "geometryEstimation": n[:, 5] - axial_reference_true,
        "measuredAccelerationPhaseAndNoise": (B + S) * (d[:, 29] - a[:, 0]),
        "finiteCellHolderInertia": B * (d[:, 11] - d[:, 29]),
        "guideFrictionEstimation": friction_ff - friction_true,
    }
    axial_error = d[:, 0] - axial_reference_true
    residual = float(np.max(np.abs(sum(terms.values()) - axial_error)))
    assert residual < 1e-9
    error = n[:, 0] - reference
    peak = int(np.argmax(np.abs(error)))
    entry = (t >= 2) & (t <= 2.003)
    reversal = (t >= 2.045) & (t <= 2.055)
    recorded = {
        "label": label, "nativeTrace": str(path), "nativeTraceSHA256": sha256(path),
        "nativeSamples": len(t), "nativeStep_s": float(t[1] - t[0]),
        "peakTime_s": float(t[peak]), "peakNormalError_N": float(error[peak]),
        "trueNormalForceAtPeak_N": float(n[peak, 0]), "measuredReconstructedNormalAtPeak_N": float(n[peak, 3]),
        "normalEstimateMinusTrueAtPeak_N": float(n[peak, 3] - n[peak, 0]),
        "trueActuatorGapVelocityAtPeak_m_s": float(d[peak, 2]),
        "trueHeadGapVelocityAtPeak_m_s": float(d[peak, 26] - d[peak, 9]),
        "estimatedHeadGapVelocityAtPeak_m_s": float(m[peak, 1]),
        "causalPredictedGuideVelocityAtPeak_m_s": float(guide_prediction[peak]),
        "holderAccelerationAtPeak_m_s2": float(d[peak, 11]),
        "toolAccelerationAtPeak_m_s2": float(d[peak, 29]),
        "measuredToolAccelerationAtPeak_m_s2": float(a[peak, 0]),
        "postNotchCurrentRequestAtPeak_A": float(d[peak, 32]),
        "actualCurrentAtPeak_A": float(d[peak, 13]),
        "trueAxialErrorAtPeak_N": float(axial_error[peak]),
        "axialToNormalErrorFactorAtPeak": float(1 / true_geometry_factor[peak]),
        "axialTermsAtPeak_N": {name: float(value[peak]) for name, value in terms.items()},
        "decompositionResidualMaximum_N": residual,
        "reconstructedRawPredictorSamplingResidualMaximum_m_s": predictor_sampling_residual,
        "nominalCellMode_Hz": float(np.sqrt(p["mechanics"]["k1"] / B) / (2 * np.pi)),
        "notchFrequency_Hz": float(p["inertial"]["currentNotch"]["frequency_Hz"]),
        "entryWindowPeakError_N": float(np.max(np.abs(error[entry]))),
        "guideReversalWindowPeakError_N": float(np.max(np.abs(error[reversal]))),
        "frequencyWindows": [dominant_frequency(t, error, lo, hi) for lo, hi in ((1.2, 1.8), (2.04, 2.08), (3, 4))],
        "voltageClippedSamples": int(np.count_nonzero(d[:, 22] > .5)),
        "currentClippedSamples": int(np.count_nonzero(d[:, 21] > .5)),
    }
    return recorded, (t, error, n[:, 3] - reference, d, m, guide_prediction, terms)


results, series = [], []
for label, suffix, key in (("clean", "first", "r"), ("noise1729", "worst", "worst")):
    result, history = analyze(ROOT / f"inertial_mechanics_structural_notch_selected_trace_20260930_{suffix}_native_trace.mat", key, label)
    results.append(result)
    series.append(history)

refinement = []
for suffix in ("", "_step5us", "_step2p5us"):
    path = ROOT / f"inertial_mechanics_structural_notch_selected_trace_20260930{suffix}_results.json"
    data = json.loads(path.read_text())
    refinement.append({"resultFile": str(path), "sha256": sha256(path),
                       "step_s": data["options"]["step_s"],
                       "peakNormalErrors_N": [row["peakNormalForceError_N"] for row in data["rows"]]})

output = {"status": "provisional_candidate_not_qualified",
          "method": "Additive axial force-balance decomposition of original native logs. The displayed normal error is evaluated independently from native normalContact channel1. Derived guide feedforward follows the saved native frame-matching recurrence. Its raw predictor sampling is verified against native normalContact channel8; matched acceleration is derived from that input, not an independently logged channel. PSD identifies spectral content in each stated window, not proof of a unique physical mode.",
          "cases": results, "solverRefinement": refinement,
          "limitation": "The 4-second cases pass but repeated bumps and notch-frequency errors fail. This is diagnostic evidence, not a promoted controller."}
(ROOT / f"{STEM}.json").write_text(json.dumps(output, indent=2) + "\n")

fig, axes = plt.subplots(3, 2, figsize=(12, 9), sharex="col")
for col, limits in enumerate(((1.9998, 2.0025), (2.043, 2.057))):
    for result, (t, error, measured, d, m, guide, terms) in zip(results, series):
        use = (t >= limits[0]) & (t <= limits[1])
        axes[0, col].plot(t[use], error[use], label=result["label"] + " true")
        if result["label"] == "noise1729":
            axes[0, col].plot(t[use], measured[use], "--", alpha=.7, label="measured reconstructed")
    t, error, measured, d, m, guide, terms = series[1]
    use = (t >= limits[0]) & (t <= limits[1])
    for name in ("currentTracking", "measuredAccelerationPhaseAndNoise", "finiteCellHolderInertia", "postNotchCurrentMinusDirectFeedforward"):
        axes[1, col].plot(t[use], terms[name][use], label=name)
    axes[2, col].plot(t[use], d[use, 2] * 1000, label="true actuator gap")
    axes[2, col].plot(t[use], m[use, 1] * 1000, label="estimated head gap")
    axes[2, col].plot(t[use], guide[use] * 1000, "--", label="causal guide prediction")
    axes[0, col].axhline(.5, color="firebrick", linestyle=":")
    axes[0, col].axhline(-.5, color="firebrick", linestyle=":")
    axes[0, col].set_title("Bump entry" if col == 0 else "Guide reversal")
    axes[2, col].set_xlabel("Native time (s)")
    for row in range(3):
        axes[row, col].grid(alpha=.25)
        axes[row, col].legend(fontsize=7)
axes[0, 0].set_ylabel("Normal force error (N)")
axes[1, 0].set_ylabel("Axial balance contributions (N)")
axes[2, 0].set_ylabel("Gap velocity (mm/s)")
fig.suptitle("Provisional finite-cell controller: native peak decomposition")
fig.tight_layout()
fig.savefig(ROOT / f"{STEM}.svg")
fig.savefig(ROOT / f"{STEM}.png", dpi=160)

lines = ["# Provisional finite-cell peak decomposition", "", "These are original full-grid native recordings. The controller remains provisional because longer recordings and frequency mismatch fail.", "", "| Case | Time (s) | True normal error (N) | Reconstructed N (N) | Cell holder inertia (N) | Acceleration phase/noise (N) | Current tracking (N) |", "|---|---:|---:|---:|---:|---:|---:|"]
for result in results:
    terms = result["axialTermsAtPeak_N"]
    lines.append(f"| {result['label']} | {result['peakTime_s']:.5f} | {result['peakNormalError_N']:.6f} | {result['measuredReconstructedNormalAtPeak_N']:.6f} | {terms['finiteCellHolderInertia']:.6f} | {terms['measuredAccelerationPhaseAndNoise']:.6f} | {terms['currentTracking']:.6f} |")
lines += ["", "Both full-record peaks occur at bump entry, before guide friction reverses. The additive force-balance contributions show synchronized sensing delay and current tracking combined with finite-cell B inertia. In the reversal window, a roughly 1.65 kHz component matches the 1.644 kHz nominal cell mode within Welch-bin resolution. The noisy steady sections instead have their largest 500–4000 Hz spectral component around 1.07 kHz; that is not a measured cell resonance shift.", "", "The estimated head gap and causal guide-velocity prediction use only measured S–A position, measured S acceleration and matched past issued robot commands. The guide feedforward is derived from the saved native frame-matching coefficients; raw predictor sampling is checked against native channel8, and force decomposition closes below 1e-9 N. The matched acceleration itself was not separately logged.", "", "Native step refinement gives clean/noisy peaks of 0.365362638/0.404199265 N at 10 µs, 0.365359841/0.404200437 N at 5 µs, and 0.365376982/0.404212959 N at 2.5 µs. All six selected runs pass, and the short-case differences are under 15 µN. This does not cure the repeated-run or frequency-mismatch failures.", "", f"![Native peak decomposition]({STEM}.svg)", ""]
(ROOT / f"{STEM}.md").write_text("\n".join(lines))
print(json.dumps({"cases": results, "output": str(ROOT / f"{STEM}.json")}, indent=2))
