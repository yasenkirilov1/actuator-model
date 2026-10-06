"""Verify that the bounded bank preserves the already-reviewed native graph."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("build")
    parser.add_argument("--parent", required=True)
    args = parser.parse_args()
    build, parent = Path(args.build).resolve(), Path(args.parent).resolve()
    target = build / "generated_cell_source_audit.json"
    assert not target.exists(), "Preserve previous source acceptance."
    relative = "code/acf_rl_native_grt_rtw/acf_rl_native.c"
    current, original = build / relative, parent / relative
    source, before = current.read_text(), original.read_text()
    normalize = lambda value: re.sub(r"^ \* C source code generated on :[^\n]*$", "", value, flags=re.M)
    assert normalize(source) == normalize(before), "The native algorithm changed outside tunable coefficients."
    live = re.search(r"rtb_Currentlimits = \(acf_rl_native_P.Requestedforce_Value -\s*acf_rl_native_B.Sensorsampledelay\) \*\s*acf_rl_native_P.Cellerrortodeviation_Gain;", source)
    selected = re.search(r"acf_rl_native_B.Cellforcedeviationfromequilibri = rtb_Currentlimits \+\s*acf_rl_native_P.Cellforcedeviationfromequilibri;", source)
    assert live and selected, "The selected observer cell source lost its measured data term."
    guard = json.loads((build / "local_cell_source_guard.json").read_text())
    assert guard["status"] == "live_local_cell_source_sign_and_nonzero_guard_passed"
    assert all(row["maximumAbsoluteWiringError_N"] < 1e-12 for row in guard["rows"])
    metadata = json.loads((build / "metadata.json").read_text())
    manifest = json.loads((Path(metadata["configsFile"]).parent / "manifest.json").read_text())
    assert manifest["scientificParity"]["allPhysicalSensorControllerSectionsUnchanged"]
    record = {
        "status": "emitted_native_measured_cell_source_live_and_algorithm_equal_v3",
        "physicalEpisodes": 0,
        "nativeCFile": str(current), "nativeCSha256": sha(current),
        "reviewedParentCFile": str(original), "reviewedParentCSha256": sha(original),
        "nativeAlgorithmIdenticalApartFromGeneratedTimestamp": True,
        "localRawDeviationAssignment": live.group(0),
        "selectedObserverDeviationAssignment": selected.group(0),
        "cellErrorToDeviationGain": -1, "additionalSelectedBias_N": 0,
        "measurementDirection": "selected deltaL = acquired gravity-tared axial cell - Fref",
        "actualSensorsMechanicsControllerAndTimingUnchanged": True,
        "allowedDifferences": "Tunable observer Q/R design coefficients and proper cell-derivative cutoff only.",
        "guardFile": str(build / "local_cell_source_guard.json"),
        "guardSha256": sha(build / "local_cell_source_guard.json"),
        "scope": "Live-source and native algorithm proof; excludes force qualification.",
    }
    target.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({key: value for key, value in record.items() if "Assignment" not in key}, indent=2))


if __name__ == "__main__":
    main()
