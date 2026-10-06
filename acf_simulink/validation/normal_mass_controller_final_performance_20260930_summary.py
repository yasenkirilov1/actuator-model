#!/usr/bin/env python3
"""Read the frozen twelve-case native drive-response comparison; no simulation."""
from pathlib import Path
import csv
import hashlib
import json
import math

HERE = Path(__file__).resolve().parent
source = HERE / "normal_mass_controller_inertial_final_performance_limits_20260930_v8_results.json"
report = json.loads(source.read_text())
assert report["status"] == "complete"
assert report["modelSha256"] == "73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643"
assert report["candidateSha256"] == "36e61547154e87d77b289d053cab40553ed849d61fa3cf278f20f902a3ad0708"
assert len(report["rows"]) == report["validatedNativeHistories"] == 12
rows = report["rows"]
assert all(r["executed"] and r["completed"] and r["passed"] and r["nativeSamples"] == 1000001 for r in rows)
compact = []
for r in rows:
    compact.append({
        "currentResponse_us": round(r["currentResponse_s"] * 1e6),
        "modeledFirstOrderBandwidth_Hz": 1 / (2 * math.pi * r["currentResponse_s"]),
        "idealBusVoltage_V": r["voltageLimit_V"], "seed": r["seedRequested"],
        "peakNormalForceError_N": r["peakNormalForceError_N"],
        "peakNormalForceErrorTime_s": r["peakNormalForceErrorTime_s"],
        "normalForceMargin_N": .5 - r["peakNormalForceError_N"],
        "settledFlatPeakNormalError_N": r["peakSettledFlatError_N"],
        "bumpAndRecoveryPeakNormalError_N": r["peakBumpPlusRecoveryError_N"],
        "rmsNormalForceError_N": r["rmsNormalForceError_N"],
        "peakNormalEstimatorError_N": r["peakNormalEstimateError_N"],
        "peakEstimatedNormalTrackingError_N": r["peakNormalEstimateTrackingError_N"],
        "peakCurrent_A": r["peakCurrent_A"], "rmsCurrent_A": r["rmsCurrent_A"],
        "peakRequestedCurrent_A": r["peakRequestedCurrent_A"],
        "peakCurrentTrackingError_A": r["peakCurrentTrackingError_A"],
        "rmsCurrentTrackingError_A": r["rmsCurrentTrackingError_A"],
        "currentTrackingAxialForceAtNormalPeak_N": r["currentTrackingAxialForceAtNormalPeak_N"],
        "peakVoltage_V": r["peakVoltage_V"],
        "voltageClippedNativeSamples": round(r["voltageClipFraction"] * r["nativeSamples"]),
        "voltageClipFraction": r["voltageClipFraction"], "effortClipFraction": r["effortClipFraction"],
        "springPackingPassed": r["springPackingPassed"], "passed": r["passed"],
    })
groups = []
for tau in [50, 100, 150]:
    values = [r for r in compact if r["currentResponse_us"] == tau]
    worst = max(values, key=lambda r: r["peakNormalForceError_N"])
    groups.append({"currentResponse_us": tau, "cases": len(values), "passed": sum(r["passed"] for r in values),
                   "worstPeakNormalForceError_N": worst["peakNormalForceError_N"],
                   "minimumMargin_N": worst["normalForceMargin_N"],
                   "maximumPeakCurrent_A": max(r["peakCurrent_A"] for r in values),
                   "maximumPeakVoltage_V": max(r["peakVoltage_V"] for r in values)})
baseline = next(g for g in groups if g["currentResponse_us"] == 100)
fast = next(g for g in groups if g["currentResponse_us"] == 50)
summary = {
    "status": "completed_bounded_alternative_drive_comparison",
    "scope": "S = 2 kg, B = 0.6 kg, target 50 N, fixed 90 degrees, Coulomb drag coefficient 0.3; seven repeated 1 mm high, 10 mm wide bumps at 100 mm/s during 10 s; noise seeds 7 and 1729. Only configured inner current response and ideal available bus vary. It is not a full-range or hardware qualification.",
    "comparisonCases": 12, "ordinaryReferenceExecutions": 1,
    "comparisonNativeSamplesScored": sum(r["nativeSamples"] for r in rows),
    "allComparisonCasesPassed": True,
    "ordinaryAcceleratorParity": report["ordinarySourceExecutionParity"],
    "modelSHA256": report["modelSha256"], "candidateSHA256": report["candidateSha256"],
    "recipeSHA256": report["experimentSha256"], "resultFile": str(source),
    "resultSHA256": hashlib.sha256(source.read_bytes()).hexdigest(),
    "configurationGroups": groups, "rows": compact,
    "fastResponseWorstErrorImprovement_N": baseline["worstPeakNormalForceError_N"] - fast["worstPeakNormalForceError_N"],
    "fastResponseWorstErrorImprovementPercent": 100 * (1 - fast["worstPeakNormalForceError_N"] / baseline["worstPeakNormalForceError_N"]),
    "busPairs": report["voltagePairs"],
    "retainedNativeTraces": report["performanceNativeTraces"],
    "selectedCurrentResponse_us": 100, "selectedIdealBusVoltage_V": 24,
    "selectionReason": "The baseline already provides useful margin. The bounded 50 us alternative lowers this cohort's worst error by only 0.0326 N while doubling modeled inner bandwidth and requesting up to 28.05 V; assembled driver/coil bandwidth and wider uncertainties remain to be verified.",
    "voltageConclusion": "At 100 us and 150 us current response, all manually retained native channels are bitwise identical between 24 V and 36 V. At 50 us, 36 V removes 42–43 voltage-clipped 10 us samples; waveform changes exist although the peak normal-force scores are unchanged to about 1e-12 N. Supply voltage and inner response were varied independently.",
    "performanceLocation": "All global force-error peaks occur within repeated bump/recovery windows; the settled-flat peaks are reported separately. This comparison mainly exposes a transient timing limit rather than flat noise or initialization.",
    "springMetadataScope": "The response probe used the prior 18.1 mm spring metadata, actual 33.6 N preload and unchanged 19,746 N/m plus 60 N s/m physical link. Final 20 mm approximate metadata is a separate force-identical candidate with its own packing audit; these probe packing lengths must not be relabeled.",
    "hardwareBoundary": "Calculated first-order bandwidth is a model parameter, not a measured driver capability. A 50 us response must be identified and verified with the selected coil inductance, current and bus headroom; a driver's rating at a different inductance does not prove it.",
}
output = HERE / "normal_mass_controller_final_performance_20260930_summary.json"
output.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
with (HERE / "normal_mass_controller_final_performance_20260930_rows.csv").open("w", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=list(compact[0])); writer.writeheader(); writer.writerows(compact)
print(json.dumps({"cases": 12, "passed": 12, "groups": groups,
                  "improvement_N": summary["fastResponseWorstErrorImprovement_N"], "selected_us": 100}))
