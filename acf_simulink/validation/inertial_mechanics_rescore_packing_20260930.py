#!/usr/bin/env python3
"""Analytically rescore spring geometry against original native extrema.

No force simulation is run or altered. The linear physical link depends on
rate, damping and net equilibrium load, while its free-length envelope is a
separate packing constraint. Original 18.1 mm results remain untouched.
"""
from hashlib import sha256
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "inertial_mechanics_lower_spring_actual_sensitivity_20260930_v8_pm20_acc_results.json"
FREE = .020
TOL = .00049
MIN_WORK = .01055
WORK_CLEAR = .0005
UNSEAT_CLEAR = .00025
ENVELOPE = 68.543
CYCLIC = .00318
PRELOAD = 33.6


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def main():
    report = json.loads(SOURCE.read_text())
    assert report["status"] == "complete" and report["executedCases"] == 72
    assert report["simulationMode"] == "accelerator"
    rows = []
    verified_trace_count = 0
    for old in report["rows"]:
        assert old["executed"] and old["completed"] and old["physicalPassed"]
        assert old["upperRigid"] and not old["lowerRigid"]
        assert old["configuredCommonPreload_N"] == PRELOAD
        assert old["identifiedLowerStiffness_N_m"] == 19746
        assert old["identifiedLowerDamping_Ns_m"] == 60
        ks = old["k1_N_m"] / 2
        # The original sole active pair computes this exact maximum from the
        # entire native recording, including its equilibrium spring offset.
        shift_max = old["feasibleCommonPreloadMin_N"] / ks - UNSEAT_CLEAR
        low = ks * (UNSEAT_CLEAR + shift_max)
        compression_upper = min(FREE-TOL-MIN_WORK-WORK_CLEAR, ENVELOPE/ks)
        high = ks * (compression_upper-shift_max)
        force_min, force_max = PRELOAD-ks*shift_max, PRELOAD+ks*shift_max
        compression_min, compression_max = force_min/ks, force_max/ks
        # Even if a successful raw trace was not retained, its full-recording
        # max(abs(dynamic deflection)) gives a rigorous sufficient range bound.
        cyclic_upper = 2 * old["peakLowerDeflection_mm"] / 1000
        cyclic_passed = cyclic_upper <= CYCLIC + 1e-12
        total_free = 2 * (FREE+TOL)
        total_installed = 2 * (FREE+TOL-PRELOAD/ks)
        passed = (low <= PRELOAD <= high and cyclic_passed and
                  total_free <= .1 and total_installed <= .1)
        trace_verified = False
        trace_hash = None
        if old["nativeTraceFile"]:
            path = Path(old["nativeTraceFile"])
            native = loadmat(path, simplify_cells=True, variable_names=["r"])["r"]
            assert len(native["time"]) == 400_001
            p, d = native["parameters"], native["data"]
            shift = p["d"]["q10"]/old["k1_N_m"] + d[:, 6]-d[:, 5]
            residual = abs(np.max(np.abs(shift))-shift_max)
            assert residual < 1e-12, (old["caseIndex"], residual)
            true_error = np.max(np.abs(native["normalContact"]["data"][:, 0]-50))
            assert true_error == old["peakNormalForceError_N"]
            assert np.ptp(shift) <= cyclic_upper + 1e-12
            trace_hash = digest(path)
            trace_verified = True
            verified_trace_count += 1
            del native, d, shift
        rows.append({
            "caseIndex": old["caseIndex"], "massS_kg": old["massS_kg"],
            "seed": old["seed"], "randomNoiseEnabled": old["forceNoiseRms_N"] > 0,
            "actualStiffness_N_m": old["k1_N_m"], "actualDamping_Ns_m": old["c1_Ns_m"],
            "identifiedStiffness_N_m": old["identifiedLowerStiffness_N_m"],
            "identifiedDamping_Ns_m": old["identifiedLowerDamping_Ns_m"],
            "peakNormalForceError_N": old["peakNormalForceError_N"],
            "original18p1mmPackingPassed": old["packingPassed"],
            "new20mmPackingPassed": passed,
            "feasibleCommonPreloadMin_N": low, "feasibleCommonPreloadMax_N": high,
            "combinedFreeLengthMax_mm": total_free * 1000,
            "combinedInstalledLengthMax_mm": total_installed * 1000,
            "minIndividualSpringForce_N": force_min, "maxIndividualSpringForce_N": force_max,
            "minIndividualCompression_mm": compression_min * 1000,
            "maxIndividualCompression_mm": compression_max * 1000,
            "minInstalledIndividualLength_mm": (FREE-TOL-compression_max) * 1000,
            "cyclicStrokeConservativeUpperBound_mm": cyclic_upper * 1000,
            "minimumNominalFreeLengthFor33p6Preload_mm":
                (MIN_WORK+WORK_CLEAR+TOL+compression_max) * 1000,
            "availableNativeTraceVerified": trace_verified,
            "nativeTraceFile": old["nativeTraceFile"], "nativeTraceSHA256": trace_hash,
        })
    assert len(rows) == 72 and verified_trace_count == 24
    assert sum(row["original18p1mmPackingPassed"] for row in rows) == 48
    assert all(row["new20mmPackingPassed"] for row in rows)
    summary = {
        "originalNativeForceExecutions": 72, "newForceExecutions": 0,
        "original18p1mmPackingPasses": 48, "original18p1mmPackingRejects": 24,
        "new20mmPackingPasses": 72, "new20mmPackingRejects": 0,
        "fullNativeRawTracesIndependentlyRechecked": verified_trace_count,
        "commonPreloadIntersectionMin_N": max(row["feasibleCommonPreloadMin_N"] for row in rows),
        "commonPreloadIntersectionMax_N": min(row["feasibleCommonPreloadMax_N"] for row in rows),
        "configuredCommonPreloadPerSpring_N": PRELOAD,
        "combinedFreeLengthMax_mm": max(row["combinedFreeLengthMax_mm"] for row in rows),
        "combinedInstalledLengthMax_mm": max(row["combinedInstalledLengthMax_mm"] for row in rows),
        "minIndividualSpringForce_N": min(row["minIndividualSpringForce_N"] for row in rows),
        "maxIndividualSpringForce_N": max(row["maxIndividualSpringForce_N"] for row in rows),
        "minIndividualCompression_mm": min(row["minIndividualCompression_mm"] for row in rows),
        "maxIndividualCompression_mm": max(row["maxIndividualCompression_mm"] for row in rows),
        "minIndividualInstalledLength_mm": min(row["minInstalledIndividualLength_mm"] for row in rows),
        "maximumRequiredNominalFreeLength_mm": max(row["minimumNominalFreeLengthFor33p6Preload_mm"] for row in rows),
        "cyclicStrokeConservativeUpperBound_mm": max(row["cyclicStrokeConservativeUpperBound_mm"] for row in rows),
        "worstOriginalNormalForceError_N": max(row["peakNormalForceError_N"] for row in rows),
    }
    output = {
        "status": "analytical_geometry_rescore_of_executed_native_histories_no_new_force_simulation",
        "date": "2026-09-30", "sourceResult": SOURCE.name, "sourceResultSHA256": digest(SOURCE),
        "sourceModel": report["sourceModel"], "sourceModelSHA256": report["sourceModelSHA256"],
        "sourceCandidateSHA256": report["candidateSHA256"],
        "sourceRecipeSHA256": report["recipeSHA256"],
        "rescoreScriptSHA256": digest(Path(__file__).resolve()),
        "geometryTarget": {"part": "approximate compact guided spring target, not an exact catalog part",
                           "nominalFreeLength_mm": 20, "freeTolerance_mm": .49,
                           "minimumWorkingLength_mm": 10.55, "workingClearance_mm": .5,
                           "unseatingClearance_mm": .25, "individualForceEnvelope_N": ENVELOPE,
                           "individualCyclicStrokeEnvelope_mm": 3.18},
        "forceHistoryPolicy": "Original true-force histories and force metrics are unchanged. Free length is a packing parameter, not a force-plant coefficient. A final parameter-source change still needs the integrator's full native source-identity proof.",
        "extremaPolicy": "Every row uses the exact maximum absolute lower-spring offset originally computed from all native samples. All24 retained packing-rejected raw histories independently reproduce that maximum. A rigorous 2*max(abs(dynamic shift)) bound certifies cyclic stroke for every row.",
        "manufacturingPolicy": "The new20mm geometry and68.543N envelope are approximate design targets requiring actual spring verification, not a manufacturer qualification of a renamed part.",
        "summary": summary, "rows": rows,
    }
    target = ROOT / "inertial_mechanics_v8_20mm_geometry_rescore_20260930.json"
    target.write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
