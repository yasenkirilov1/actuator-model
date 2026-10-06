"""Compile and freeze a current-FF lead bank; no physical qualification runs."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from native_api import NativePlant


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("build")
    args = parser.parse_args()
    build = Path(args.build).resolve()
    seal_path = build / "file_hashes.json"
    assert not seal_path.exists(), "Preserve earlier native seal."
    plant = NativePlant(build, rebuild=not (build / "libacf_native.so").exists())
    metadata = plant.metadata
    bank = Path(metadata["configsFile"]).parent
    manifest = json.loads((bank / "manifest.json").read_text())
    assert metadata["configsSha256"] == sha(bank / "configs.mat") == manifest["configsSha256"]
    assert metadata["sourceModelSha256"] == manifest["sourceModelSha256"]
    assert metadata["algebraicLoopCount"] == 0
    guard = json.loads((build / "local_cell_source_guard.json").read_text())
    assert guard["status"] == "live_local_cell_source_sign_and_nonzero_guard_passed"
    assert len(guard["rows"]) == 3 and all(row["maximumAbsoluteWiringError_N"] < 1e-12 for row in guard["rows"])
    source_audit = json.loads((build / "generated_cell_source_audit.json").read_text())
    assert source_audit["nativeAccelerationLeadBlockPresent"]
    assert source_audit["nativeCSha256"] == sha(build / "code/acf_rl_native_grt_rtw/acf_rl_native.c")
    assert len(plant.scenarios) == len(manifest["cases"]) == 352
    assert len(manifest["leadFamilies"]) == 16 and len(manifest["observerProfiles"]) == 2
    assert manifest["scientificParity"]["allExistingPhysicalSensorControllerSectionsUnchanged"]
    assert manifest["scientificParity"]["allExistingPreparedCoefficientsIncludingUpperObserverUnchanged"]
    assert metadata["step_s"] == 1 / 300000 and metadata["actionStep_s"] == 1e-4
    files = [build / name for name in ["acf_rl_native.slx", "metadata.json", "prepared_configs.mat",
             "native_parameter_bank.c", "parameter_mapping.json", "parameter_mapping.mat", "scenarios.json",
             "libacf_native.so", "bridge_build.json", "acf_rl_native_context.c", "native_bank_aux.h", "build_native.m",
             "local_cell_source_guard.json", "generated_cell_source_audit.json"]]
    files += [bank / "configs.mat", bank / "manifest.json"]
    recipe = Path(__file__).parent
    files += [recipe / "observer_noise_v4/acf_prepare_upper_observer.m"]
    files += sorted((recipe / "accel_lead_v5").glob("*.m"))
    files += sorted((build / "helpers").glob("*.m"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.c"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.h"))
    files += [recipe / name for name in ["native_api.py", "native_bridge.c", "build_upper_force_native_lead_v5.m",
              "install_upper_force_observer_cellsource_v2.m", "prepare_observer_lead_bank_v5.m",
              "prepare_and_build_observer_lead_v5.m", "audit_native_lead_v5.py", "seal_native_lead_v5.py",
              "verify_force_observer_lead_graph_v5.m", "verify_local_cell_source.m"]]
    record = {"status": "native_graph_and_parameter_bank_frozen_not_executed", "createdEpisodes": 0,
              "design": manifest["design"], "caseCount": len(manifest["cases"]),
              "leadFamilyCount": len(manifest["leadFamilies"]), "selectedParentObserverProfileIDs": [24, 33],
              "scientificParity": manifest["scientificParity"], "nativeBankCase0MatchesCoder": True,
              "sourceModelSha256": metadata["sourceModelSha256"], "configsSha256": metadata["configsSha256"],
              "sharedLibrarySha256": sha(build / "libacf_native.so"),
              "files": [{"file": str(path), "sha256": sha(path)} for path in files]}
    seal_path.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({key: value for key, value in record.items() if key != "files"}, indent=2))


if __name__ == "__main__":
    main()
