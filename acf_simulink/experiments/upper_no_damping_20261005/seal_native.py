"""Compile/seal a private emitted Simulink graph, without running an episode."""
from __future__ import annotations
import argparse
import hashlib
import json
import re
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
    if seal_path.exists():
        raise SystemExit("Preserve the earlier frozen native seal.")
    plant = NativePlant(build, rebuild=not (build / "libacf_native.so").exists())
    metadata = plant.metadata
    bank = Path(metadata["configsFile"]).parent
    manifest = json.loads((bank / "manifest.json").read_text())
    assert metadata["configsSha256"] == sha(bank / "configs.mat") == manifest["configsSha256"]
    assert metadata["sourceModelSha256"] == manifest["sourceModelSha256"]
    assert len(plant.scenarios) == len(manifest["cases"])
    assert metadata["step_s"] == 1 / 300000 and metadata["actionStep_s"] == 1e-4
    files = [build / name for name in ["acf_rl_native.slx", "metadata.json", "prepared_configs.mat",
             "native_parameter_bank.c", "parameter_mapping.json", "parameter_mapping.mat", "scenarios.json",
             "libacf_native.so", "bridge_build.json", "acf_rl_native_context.c", "native_bank_aux.h"]]
    files += [bank / "configs.mat", bank / "manifest.json"]
    files += sorted((build / "helpers").glob("*.m"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.c"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.h"))
    builder_file = build / "build_native.m"
    builder_text = builder_file.read_text()
    installers = re.findall(r"\w+Contract=(install_\w+)\(", builder_text)
    assert installers
    files += [Path(__file__).with_name("native_api.py"), Path(__file__).with_name("native_bridge.c"),
              builder_file]
    files += [Path(__file__).with_name(installer + ".m") for installer in installers]
    record = {"status": "native_graph_and_parameter_bank_frozen_not_executed", "createdEpisodes": 0,
              "design": manifest["design"], "caseCount": len(manifest["cases"]),
              "nativeBankCase0MatchesCoder": True, "sourceModelSha256": metadata["sourceModelSha256"],
              "configsSha256": metadata["configsSha256"], "sharedLibrarySha256": sha(build / "libacf_native.so"),
              "files": [{"file": str(p), "sha256": sha(p)} for p in files]}
    seal_path.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({k: v for k, v in record.items() if k != "files"}, indent=2))


if __name__ == "__main__":
    main()
