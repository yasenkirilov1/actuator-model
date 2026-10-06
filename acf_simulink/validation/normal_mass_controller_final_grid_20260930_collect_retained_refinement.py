"""Audit the retained 5 us native return without executing another simulation."""

import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat


BASE = Path(__file__).resolve().parent
TRACE = BASE / "normal_mass_controller_final_grid_20260930_solver_refinement_v8_step_5000ns_step_5000ns_native.mat"
BASELINE = BASE / "final_grid/part1/normal_mass_controller_inertial_full_range_mass_20260930_v8_final_part1_r2_worst_native_trace.mat"
SOURCE = Path("/home/tommie/.local/state/actuator-model/normal-contact-trial-inertial-v8-contract-20260930/frozen-inertial-v8")
WORKER = Path("/home/tommie/.local/state/actuator-model/tp0d0e04f2_a35d_45ef_85be_d04346ed0f1f")
RESULT = BASE / "normal_mass_controller_final_grid_20260930_solver_refinement_v8_step_5000ns_results.json"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def differences(a, b, prefix=""):
    if isinstance(a, dict):
        assert isinstance(b, dict) and a.keys() == b.keys(), prefix
        return [item for key in a for item in differences(a[key], b[key], f"{prefix}.{key}" if prefix else key)]
    try:
        equal = np.array_equal(a, b, equal_nan=True)
    except TypeError:
        equal = np.array_equal(a, b)
    return [] if equal else [prefix]


def spring_packing(p, data):
    pack = p["springs"]
    low, high, total_free, installed = 0.0, float("inf"), 0.0, 0.0
    excursion_pass = True
    links = [
        ("lower", "lowerRigid", "k1", "q10", data[:, 6] - data[:, 5]),
        ("upper", "upperRigid", "k2", "q20", data[:, 4] - data[:, 25]),
    ]
    for name, rigid, stiffness, offset, displacement in links:
        if p["mechanics"][rigid]:
            continue
        spring = pack[name]
        rate = spring["rate_N_m"]
        shift = p["d"][offset] / p["mechanics"][stiffness] + displacement
        largest = float(np.max(np.abs(shift)))
        low = max(low, rate * (pack["unseatingClearance_m"] + largest))
        compression = min(
            spring["freeLength_m"] - spring["freeLengthTolerance_m"]
            - spring["minWorkingLength_m"] - pack["workingClearance_m"],
            spring["forceEnvelope_N"] / rate,
        )
        high = min(high, rate * (compression - largest))
        excursion_pass &= np.ptp(shift) <= spring["maxCyclicStroke_m"] + 1e-12
        total_free += 2000 * (spring["freeLength_m"] + spring["freeLengthTolerance_m"])
        installed += 2000 * (spring["freeLength_m"] + spring["freeLengthTolerance_m"] - pack["commonPreload_N"] / rate)
    passed = bool(excursion_pass and low <= pack["commonPreload_N"] <= high
                  and total_free <= 1000 * pack["maxCombinedLength_m"]
                  and installed <= 1000 * pack["maxCombinedLength_m"])
    return passed, low, high, total_free, installed


def main():
    assert not RESULT.exists(), "Preserve the frozen refinement audit."
    model_sha = "73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643"
    candidate_sha = "d8c49dd39bbb3417c294a79ebb061efdff04d2625b91794066d7b8ad087979c7"
    assert sha(SOURCE / "acf_force_control_normal_trial.slx") == model_sha
    assert sha(WORKER / "acf_force_control_normal_trial.slx") == model_sha
    assert sha(BASE / "inertial_final_candidate_20260930.mat") == candidate_sha
    trace_sha = sha(TRACE)
    native = loadmat(TRACE, variable_names=["r"], simplify_cells=True)["r"]
    reference = loadmat(BASELINE, variable_names=["worst", "worstRow"], simplify_cells=True)
    old = reference["worst"]
    p = native["parameters"]
    changed = differences(old["parameters"], p)
    assert set(changed) == {"sim.step", "d.forceTicks", "d.sensorTicks", "d.robotTicks"}
    assert p["sim"]["step"] == 5e-6 and old["parameters"]["sim"]["step"] == 10e-6
    for tick in ["forceTicks", "sensorTicks", "robotTicks"]:
        assert p["d"][tick] == 2 * old["parameters"]["d"][tick]
    assert len(native["time"]) == 800001 and native["time"][0] == 0 and native["time"][-1] == 4
    assert np.array_equal(native["time"][::2], old["time"])
    assert native["data"].shape == (800001, 35)
    channel_differences = {"originalPhysicalAndControl": float(np.max(np.abs(native["data"][::2] - old["data"])))}
    channels = 35
    for name, log in old.items():
        if not isinstance(log, dict) or "time" not in log or "data" not in log:
            continue
        new = native[name]
        assert np.array_equal(new["time"], native["time"]), name
        assert new["data"].shape == (800001, log["data"].shape[1]), name
        channel_differences[name] = float(np.max(np.abs(new["data"][::2] - log["data"])))
        channels += log["data"].shape[1]
    assert np.array_equal(native["robotGate"]["time"], old["robotGate"]["time"])
    assert len(native["robotGate"]["time"]) == 401
    channel_differences["robotGate"] = {}
    for name in ["ready", "filteredError_N", "forceRate_N_s", "stableNow"]:
        assert native["robotGate"][name].shape == old["robotGate"][name].shape == (401,)
        channel_differences["robotGate"][name] = float(np.max(np.abs(native["robotGate"][name] - old["robotGate"][name])))
        channels += 1
    assert channels == 72
    normal = native["normalContact"]["data"][:, 0]
    error = normal - p["force"]["reference"]
    index = int(np.argmax(np.abs(error)))
    peak = float(abs(error[index]))
    packing, low, high, free, installed = spring_packing(p, native["data"])
    row = {
        "integrationStep_s": 5e-6, "nativeSamples": 800001,
        "peakNormalForceError_N": peak, "normalPeakTime_s": float(native["time"][index]),
        "peakScoreDifferenceFrom10us_N": peak - reference["worstRow"]["peakNormalForceError_N"],
        "rmsNormalForceError_N": float(np.sqrt(np.mean(error**2))),
        "peakNormalEstimationError_N": float(np.max(np.abs(native["normalContact"]["data"][:, 3] - normal))),
        "trueNormalForceDifferenceAt10usTimes_N": float(np.max(np.abs(normal[::2] - old["normalContact"]["data"][:, 0]))),
        "peakCurrent_A": float(np.max(np.abs(native["data"][:, 13]))),
        "peakVoltage_V": float(np.max(np.abs(native["data"][:, 14]))),
        "voltageClipFraction": float(np.mean(native["data"][:, 22] > 0)),
        "currentClipFraction": float(np.mean(native["data"][:, 21] > 0)),
        "lostContact": bool(native["metrics"]["lostContact"]),
        "strokeStop": bool(native["metrics"]["strokeStopHit"]),
        "springPackingPassed": packing, "minimumCommonSpringPreload_N": low,
        "maximumCommonSpringPreload_N": high, "combinedFreeSpringLength_mm": free,
        "combinedInstalledSpringLength_mm": installed,
        "nativeChannelsCompared": channels, "coarseGridChannelDifferences": channel_differences,
        "nativeTrace": str(TRACE), "nativeTraceSha256": trace_sha,
    }
    row["passed"] = bool(peak <= .5 and packing and not row["lostContact"] and not row["strokeStop"])
    assert row["passed"] and sha(TRACE) == trace_sha
    report = {
        "status": "complete", "scope": "Numerical convergence of final nominal-grid worst global3489; no retuning or hardware qualification.",
        "executedNativeReruns": 1, "nativeCasesExecutedByThisCollector": 0,
        "retainedNativeHistoriesValidated": 1, "nativeSamplesScored": 800001,
        "modelSha256": model_sha, "candidateSha256": candidate_sha,
        "baselineTrace": str(BASELINE), "baselineTraceSha256": sha(BASELINE),
        "originalNativeExperimentSnapshot": str(WORKER / "normal_mass_controller_final_grid_20260930_refine_solver.m"),
        "originalNativeExperimentSha256": sha(WORKER / "normal_mass_controller_final_grid_20260930_refine_solver.m"),
        "collectorFile": str(Path(__file__).resolve()), "collectorSha256": sha(Path(__file__)),
        "parameterChanges": changed, "rows": [row],
        "gateSamplingNote": "The68 physical/measurement/control channels have800001 fine native points. The4 robot-gate channels retain their own401-point10ms grid and are compared on that unchanged grid.",
        "recoveryNote": "Retained native return from attempt03 is scored offline. Original post-run gate-grid assertion rejection is preserved; no extra5us simulation or trace rewrite occurred. Attempt01 returned without a durable trace; attempt02 wrapper rejected before experiment execution.",
    }
    RESULT.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"result": str(RESULT), "peak_N": peak, "delta_N": row["peakScoreDifferenceFrom10us_N"], "passed": row["passed"]}))


if __name__ == "__main__":
    main()
