"""Seal reused emitted computation and a verified controller-only bank."""
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
    assert not (build / "file_hashes.json").exists()
    plant = NativePlant(build, rebuild=not (build / "libacf_native.so").exists())
    metadata = plant.metadata
    parent = Path(metadata["parentNativeBuild"])
    bank = Path(metadata["configsFile"]).parent
    manifest = json.loads((bank / "manifest.json").read_text())
    coefficient = json.loads((build / "tunable_coefficient_reuse_guard.json").read_text())
    local = json.loads((build / "local_cell_source_guard.json").read_text())
    assert metadata["configsSha256"] == manifest["configsSha256"] == sha(bank / "configs.mat")
    assert metadata["sourceModelSha256"] == manifest["sourceModelSha256"]
    assert metadata["nativeAlgorithmReusedExactly"] and not metadata["nativeCGraphGenerated"]
    assert metadata["algebraicLoopCount"] == 0
    assert coefficient["originalCoderCase0AllFieldsExactlyEqual"]
    assert coefficient["allowedOverrideFields"] == ["Guidefrictionvelocityscale_Gain", "GuideCoulombfrictioncurrent_Gai"]
    assert local["status"] == "live_local_cell_source_sign_and_nonzero_guard_passed"
    assert all(r["maximumAbsoluteWiringError_N"] < 1e-12 for r in local["rows"])
    assert len(plant.scenarios) == 265 and plant.scenarios[0]["split"] == "preparation_check"
    assert all(s["split"] == "development" and s["upperLengthUnconstrained"] for s in plant.scenarios[1:])
    code_files = sorted((build / "code/acf_rl_native_grt_rtw").glob("*.c")) + sorted((build / "code/acf_rl_native_grt_rtw").glob("*.h"))
    assert all(p.read_bytes() == (parent / "code/acf_rl_native_grt_rtw" / p.name).read_bytes() for p in code_files)
    assert (build / "acf_rl_native_context.c").read_bytes() == (parent / "acf_rl_native_context.c").read_bytes()
    source_bridge = parent / "bridge_build.json"
    parent_bridge = json.loads(source_bridge.read_text())
    bridge = json.loads((build / "bridge_build.json").read_text())
    assert bridge["emitted_c_sha256"] == parent_bridge["emitted_c_sha256"]
    assert bridge["adapted_c_sha256"] == parent_bridge["adapted_c_sha256"]
    assert bridge["native_computation_recovered_exactly"]
    names = ["acf_rl_native.slx", "metadata.json", "prepared_configs.mat", "native_parameter_bank.c",
             "parameter_mapping.json", "parameter_mapping.mat", "scenarios.json", "libacf_native.so",
             "bridge_build.json", "acf_rl_native_context.c", "native_bank_aux.h", "tunable_coefficient_reuse_guard.json",
             "local_cell_source_guard.json", "parent_generated_cell_source_audit.json", "parent_local_cell_source_guard.json",
             "export_force_guide_native.m"]
    files = [build / name for name in names] + code_files
    files += sorted((build / "helpers").glob("*.m"))
    files += [bank / "configs.mat", bank / "manifest.json", bank / "prepare_force_guide_bank.m"]
    files += [Path(__file__).with_name(name) for name in ["native_api.py", "native_bridge.c", "verify_local_cell_source.m", "prepare_force_guide_bank.m", "export_force_guide_native.m", "seal_native.py"]]
    record = {
        "status": "reused_native_algorithm_and_controller_only_parameter_bank_frozen",
        "createdForceQualificationEpisodes": 0,
        "sourceWiringHarnessRuns": 3,
        "developmentCases": 264,
        "unscoredPreparationCheckSentinels": 1,
        "parentNativeBuild": str(parent),
        "nativeCAndContextComputationByteIdenticalToParent": True,
        "nativeBankCase0MatchesOriginalCoder": True,
        "sourceModelSha256": metadata["sourceModelSha256"],
        "privateControllerGraphSha256": metadata["privateModelSha256"],
        "configsSha256": metadata["configsSha256"],
        "sharedLibrarySha256": sha(build / "libacf_native.so"),
        "contextSha256": bridge["adapted_c_sha256"],
        "files": [{"file": str(p), "sha256": sha(p)} for p in files],
    }
    (build / "file_hashes.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({k: v for k, v in record.items() if k != "files"}, indent=2))


if __name__ == "__main__":
    main()
