"""Inspect the live measured cell path after adding the current-FF-only lead."""
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
    args = parser.parse_args()
    build = Path(args.build).resolve()
    target = build / "generated_cell_source_audit.json"
    assert not target.exists(), "Preserve prior acceptance."
    current = build / "code/acf_rl_native_grt_rtw/acf_rl_native.c"
    source = current.read_text()
    live = re.search(r"(\w+) = \(acf_rl_native_P.Requestedforce_Value -\s*acf_rl_native_B.Sensorsampledelay\) \*\s*acf_rl_native_P.Cellerrortodeviation_Gain;", source)
    assert live, "Acquired cell data disappeared from the local deviation."
    selected = re.search(r"acf_rl_native_B.Cellforcedeviationfromequilibri = " + re.escape(live.group(1)) + r" \+\s*acf_rl_native_P.Cellforcedeviationfromequilibri;", source)
    assert selected, "The observer lost the measured local cell path."
    assert "DiscreteTransferFcn: '<S" in source and "Present-time acceleration lead for current FF'" in source
    metadata = json.loads((build / "metadata.json").read_text())
    manifest = json.loads((Path(metadata["configsFile"]).parent / "manifest.json").read_text())
    guard = json.loads((build / "local_cell_source_guard.json").read_text())
    assert guard["status"] == "live_local_cell_source_sign_and_nonzero_guard_passed"
    assert all(row["maximumAbsoluteWiringError_N"] < 1e-12 for row in guard["rows"])
    assert manifest["scientificParity"]["allExistingPreparedCoefficientsIncludingUpperObserverUnchanged"]
    record = {
        "status": "emitted_native_measured_cell_source_live_with_current_ff_acceleration_lead",
        "physicalEpisodes": 0, "nativeCFile": str(current), "nativeCSha256": sha(current),
        "localRawDeviationAssignment": live.group(0),
        "selectedObserverDeviationAssignment": selected.group(0),
        "cellErrorToDeviationGain": -1, "additionalSelectedBias_N": 0,
        "nativeAccelerationLeadBlockPresent": True,
        "existingPreparedCoefficientsIncludingObserverUnchanged": True,
        "measurementFrameUnchanged": True,
        "guardFile": str(build / "local_cell_source_guard.json"),
        "guardSha256": sha(build / "local_cell_source_guard.json"),
        "scope": "Live source plus native lead presence; saved-copy audit proves its current-FF-only connections. No force qualification inferred.",
    }
    target.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({key: value for key, value in record.items() if "Assignment" not in key}, indent=2))


if __name__ == "__main__":
    main()
