"""Compile/freeze the bounded active-damping controller; no force episodes."""
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
    seal = build / "file_hashes.json"
    assert not seal.exists(), "Preserve earlier native seal."
    plant = NativePlant(build, rebuild=not (build / "libacf_native.so").exists())
    metadata = plant.metadata;bank = Path(metadata["configsFile"]).parent
    manifest = json.loads((bank / "manifest.json").read_text())
    assert metadata["configsSha256"] == sha(bank / "configs.mat") == manifest["configsSha256"]
    assert metadata["sourceModelSha256"] == manifest["sourceModelSha256"] and metadata["algebraicLoopCount"] == 0
    assert len(plant.scenarios) == len(manifest["cases"]) == 264 and len(manifest["controllerGainFamilies"]) == 12
    guard = json.loads((build / "local_cell_source_guard.json").read_text())
    assert guard["modalSourceVerified"] and len(guard["rows"]) == 3
    assert all(r["maximumAbsoluteWiringError_N"] < 1e-12 and r["maximumModalSourceError_N"] < 1e-12 for r in guard["rows"])
    phase = json.loads((build / "modal_positive_damping_guard.json").read_text())
    assert phase["status"] == "native_modal_positive_damping_sign_and_phase_passed" and len(phase["rows"]) == 3
    assert all(abs(r["gainRatio"]-1) < .03 and abs(r["phaseError_deg"]) < 3 and r["meanVirtualBModePower_W"] < 0 for r in phase["rows"])
    source = json.loads((build / "generated_cell_source_audit.json").read_text())
    assert source["modalStateInputCount"] == 2 and source["antiwindupUsesTotalPrelimitCurrent"]
    assert source["nativeCSha256"] == sha(build / "code/acf_rl_native_grt_rtw/acf_rl_native.c")
    assert manifest["scientificParity"]["allPhysicalSensingObserverRobotNotchUnchanged"]
    assert manifest["scientificParity"]["exactReferenceAllPreparedCoefficientsEqual"]
    assert manifest["scientificParity"]["modalCoefficientsInvariantToActualCurrentResponsePerturbation"]
    assert metadata["step_s"] == 1/300000 and metadata["actionStep_s"] == 1e-4
    parity = json.loads((build / "native_reference_parameter_parity.json").read_text())
    assert parity["exactReferenceRows"] == 22 and parity["allNativeFieldsExactlyEqual"]
    files = [build / name for name in ["acf_rl_native.slx", "metadata.json", "prepared_configs.mat", "native_parameter_bank.c",
             "parameter_mapping.json", "parameter_mapping.mat", "scenarios.json", "libacf_native.so", "bridge_build.json",
             "acf_rl_native_context.c", "native_bank_aux.h", "build_native.m", "local_cell_source_guard.json",
             "modal_positive_damping_guard.json", "generated_cell_source_audit.json", "native_reference_parameter_parity.json"]]
    files += [bank / "configs.mat", bank / "manifest.json", Path(manifest["phaseHarnessFile"])]
    files += sorted((build / "helpers").glob("*.m"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.c")) + sorted((build / "code/acf_rl_native_grt_rtw").glob("*.h"))
    recipe = Path(__file__).parent
    files += sorted((recipe / "helpers").glob("*.m"))
    files += [recipe / name for name in ["native_api.py", "native_bridge.c",
              "build_upper_force_native_controller_gain_v8.m", "install_upper_force_observer_cellsource_v2.m", "install_cell_modal_v6.m",
              "prepare_controller_gain_bank.m", "prepare_and_build.m", "verify_modal_cell_source_v6.m",
              "verify_modal_positive_damping_v8.m", "verify_controller_gain_graph.m",
              "audit_native_controller_gain_v8.py", "seal_native_controller_gain_v8.py", "verify_native_reference_parity.py"]]
    record = {"status": "native_graph_and_parameter_bank_frozen_not_executed", "qualificationEpisodes": 0,
              "design": manifest["design"], "caseCount": 264, "controllerGainFamilyCount": 12, "exactReferenceFamilyID": 8,
              "selectedParentObserverProfileIDs": [33], "selectedParentModalFamilyID": 17, "nativeBankCase0MatchesCoder": True,
              "scientificParity": manifest["scientificParity"], "sourceModelSha256": metadata["sourceModelSha256"],
              "configsSha256": metadata["configsSha256"], "sharedLibrarySha256": sha(build / "libacf_native.so"),
              "files": [{"file": str(path), "sha256": sha(path)} for path in files]}
    seal.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({key: value for key, value in record.items() if key != "files"}, indent=2))


if __name__ == "__main__":
    main()
