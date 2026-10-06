#!/usr/bin/env python3
"""Compare full retained software-proof histories, never plot excerpts."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def arrays(run):
    fields = {"time": run["time"], "data": run["data"]}
    for name in ("normalContact", "accelerometer", "motionObserver", "commandPrediction",
                 "forceVariation", "accelerationFeedforward", "measuredDrag", "lowerInverse"):
        for field in ("time", "data"):
            fields[f"{name}.{field}"] = run[name][field]
    for field in ("time", "ready", "filteredError_N", "forceRate_N_s", "stableNow"):
        fields[f"robotGate.{field}"] = run["robotGate"][field]
    return {k: np.asarray(v) for k, v in fields.items()}


def compare(left, right):
    a, b = arrays(left), arrays(right)
    assert a.keys() == b.keys()
    checks = {}
    for key in a:
        assert a[key].shape == b[key].shape, key
        checks[key] = dict(bitwiseEqual=np.array_equal(a[key], b[key]),
                           maxDifference=float(np.max(np.abs(a[key] - b[key]))))
    return dict(checks=checks, allBitwiseEqual=all(r["bitwiseEqual"] for r in checks.values()),
                allNumericallyEquivalent=max(r["maxDifference"] for r in checks.values()) < 1e-9,
                nativeSignalChannelsCompared=72,
                maxTrueNormalDifference_N=checks["normalContact.data"]["maxDifference"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("version", help="Folder prefix, for example v8 or lower_v1")
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    result = dict(scope="Two4s endpoint configurations, each native unmodified Normal/reference, "
                        "private zero-residual instruments Normal, and same instruments Accelerator. "
                        "Six native executions are paired software proofs, not six independent operating points.",
                  comparisons=[])
    reports = {}
    for kind in ("reference", "normal", "accelerator"):
        folder = root / f"inertial_uncertainty_{args.version}_proof_{kind}"
        file = folder / f"inertial_uncertainty_proof_{kind}_20260930_results.json"
        reports[kind] = json.loads(file.read_text())
        assert reports[kind]["status"] == "complete" and reports[kind]["executed"] == 2
    identities = {(r["sourceModelSha256"], r["candidateSha256"]) for r in reports.values()}
    assert len(identities) == 1
    result["sourceModelSha256"], result["candidateSha256"] = next(iter(identities))
    for case in (1, 2):
        paths = {name: Path(reports[name]["rows"][case - 1]["nativeTrace"]) for name in reports}
        runs = {name: loadmat(path, simplify_cells=True)["r"] for name, path in paths.items()}
        one = dict(case=case, physicalS_kg=runs["normal"]["parameters"]["mechanics"]["mS"],
                   target_N=runs["normal"]["parameters"]["force"]["reference"],
                   angle_rad=runs["normal"]["parameters"]["mechanics"]["angle"],
                   seed=runs["normal"]["parameters"]["sensor"]["seed"],
                   traces={k: str(v) for k, v in paths.items()}, traceHashes={k: digest(v) for k, v in paths.items()},
                   comparisons=dict(referenceToNormal=compare(runs["reference"], runs["normal"]),
                                    normalToAccelerator=compare(runs["normal"], runs["accelerator"])))
        result["comparisons"].append(one)
    result["allBitwiseEqual"] = all(c["allBitwiseEqual"] for r in result["comparisons"] for c in r["comparisons"].values())
    result["allNumericallyEquivalent"] = all(c["allNumericallyEquivalent"] for r in result["comparisons"] for c in r["comparisons"].values())
    result["nativeExecutions"] = 6
    result["distinctSoftwareProofConfigurations"] = 2
    result["passedNativeExecutions"] = sum(r["passed"] for r in reports.values())
    path = root / f"inertial_uncertainty_{args.version}_instrumentation_parity.json"
    path.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: result[k] for k in ("allBitwiseEqual", "allNumericallyEquivalent", "passedNativeExecutions")}))
    assert result["allNumericallyEquivalent"], "Software-proof histories do not agree."


if __name__ == "__main__":
    main()
