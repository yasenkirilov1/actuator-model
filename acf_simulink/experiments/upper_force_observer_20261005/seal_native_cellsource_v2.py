"""Compile and freeze the emitted native observer candidate; no episodes."""
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
    assert len(plant.scenarios) == len(manifest["cases"]) == 132
    assert metadata["step_s"] == 1 / 300000 and metadata["actionStep_s"] == 1e-4
    files = [build / name for name in ["acf_rl_native.slx", "metadata.json", "prepared_configs.mat",
             "native_parameter_bank.c", "parameter_mapping.json", "parameter_mapping.mat", "scenarios.json",
             "libacf_native.so", "bridge_build.json", "acf_rl_native_context.c", "native_bank_aux.h", "build_native.m"]]
    files += [bank / "configs.mat", bank / "manifest.json", build / "local_cell_source_guard.json"]
    files += [Path(__file__).with_name("cellsource_v2") / "acf_prepare_upper_observer.m"]
    files += sorted((build / "helpers").glob("*.m"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.c"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.h"))
    files += [Path(__file__).with_name(name) for name in ["native_api.py", "native_bridge.c",
              "build_upper_force_native_cellsource_v2.m", "install_upper_force_observer_cellsource_v2.m",
              "prepare_cellsource_force_bank.m", "verify_force_observer_graph_cellsource_v2.m", "verify_local_cell_source.m"]]
    record = {"status": "native_graph_and_parameter_bank_frozen_not_executed", "createdEpisodes": 0,
              "design": manifest["design"], "caseCount": len(manifest["cases"]),
              "nativeBankCase0MatchesCoder": True, "sourceModelSha256": metadata["sourceModelSha256"],
              "configsSha256": metadata["configsSha256"], "sharedLibrarySha256": sha(build / "libacf_native.so"),
              "files": [{"file": str(p), "sha256": sha(p)} for p in files]}
    seal_path.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({k: v for k, v in record.items() if k != "files"}, indent=2))


if __name__ == "__main__":
    main()
