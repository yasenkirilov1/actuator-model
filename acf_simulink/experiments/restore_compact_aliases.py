"""Verify published compact SSD evidence; optionally restore relative aliases."""
import argparse
import hashlib
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "acf_simulink/validation/github_snapshot_20261006/compact_external_manifest.json"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--restore", action="store_true")
    args = parser.parse_args()
    data = json.loads(MANIFEST.read_text())
    verified = restored = preserved = 0
    for item in data["files"]:
        path = ROOT / item["path"]
        assert path.resolve().is_relative_to(ROOT), str(path)
        assert hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"], str(path)
        verified += 1
    for item in data["aliases"]:
        alias, mirror = ROOT / item["alias"], ROOT / item["mirror"]
        assert alias.parent.resolve().is_relative_to(ROOT), str(alias)
        assert mirror.resolve().is_relative_to(ROOT) and mirror.is_dir(), str(mirror)
        if alias.exists() or alias.is_symlink():
            preserved += 1
        elif args.restore:
            alias.parent.mkdir(parents=True, exist_ok=True)
            alias.symlink_to(os.path.relpath(mirror, alias.parent), target_is_directory=True)
            restored += 1
    print(json.dumps({"verified_files": verified, "restored_aliases": restored,
                      "existing_paths_preserved": preserved,
                      "raw_histories_restored": False}))


if __name__ == "__main__":
    main()
