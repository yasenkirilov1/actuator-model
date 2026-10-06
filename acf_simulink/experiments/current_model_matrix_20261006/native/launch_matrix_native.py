"""One private MATLAB batch with bounded temporary files and compact results."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time

sys.dont_write_bytecode = True

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
OUT = ROOT / "acf_simulink/validation/current_model_matrix_20261006/native"
CONTRACT = HERE.parent / "study_contract.json"
FORECAST = OUT.parent / "storage_forecast.json"
RETAINED_CAP = 500 * 1024
TEMP_CAP = 600 * 1024**2


def tree_bytes(folder: Path) -> int:
    return sum(path.stat().st_size for path in folder.rglob("*") if path.is_file())


def array_audit(scopes: list[str]) -> int:
    seen = set()
    total = 0
    for scope in scopes:
        for path in Path(scope).rglob("*"):
            if path.suffix not in {".mat", ".npz", ".npy", ".bin"} or not path.is_file():
                continue
            stat = path.stat()
            key = (stat.st_dev, stat.st_ino)
            if key not in seen:
                seen.add(key)
                total += stat.st_size
    return total


def matlab_quote(value: str) -> str:
    return "'" + value.replace("'", "''") + "'"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("selection", type=Path)
    parser.add_argument("--contract", type=Path, default=CONTRACT)
    parser.add_argument("--contract-sha256", required=True)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    forecast = json.loads(FORECAST.read_text())
    arrays = array_audit(forecast["shared_array_audit"]["scopes"])
    if arrays + forecast["temporary_all_workers_cap_bytes"] + forecast["incremental_retained_cap_bytes"] > forecast["shared_array_limit_bytes"]:
        raise RuntimeError("Shared array forecast exceeds the authorized5GiB.")
    free = shutil.disk_usage("/tmp").free
    if free < TEMP_CAP + 256 * 1024**2:
        raise RuntimeError("Insufficient private temporary free space.")
    available = next(int(line.split()[1]) * 1024 for line in Path("/proc/meminfo").read_text().splitlines() if line.startswith("MemAvailable:"))
    if available < 2 * 1024**3:
        raise RuntimeError("Insufficient available RAM for one private native run.")
    temp = Path(tempfile.mkdtemp(prefix="acf_matrix_native_", dir="/tmp"))
    (temp / "owned_matrix_native").write_text("Owned bounded current-matrix native worker\n")
    prefs = temp / "preferences"
    prefs.mkdir()
    env = dict(os.environ, MATLAB_PREFDIR=str(prefs), OMP_NUM_THREADS="2", OPENBLAS_NUM_THREADS="2")
    start = time.monotonic()
    audit = dict(status="launch_checks_passed", shared_retained_array_bytes=arrays,
                 global_temporary_forecast_bytes=forecast["temporary_all_workers_cap_bytes"],
                 worker_temporary_cap_bytes=TEMP_CAP, worker_retained_cap_bytes=RETAINED_CAP,
                 free_tmp_bytes=free, available_RAM_bytes=available, temporary_directory=str(temp),
                 private_preferences=True, temporary_peak_polled_bytes=0)
    try:
        prep = subprocess.run([sys.executable, str(HERE / "prepare_matrix_native.py"), str(args.selection),
                               "--contract", str(args.contract), "--contract-sha256", args.contract_sha256,
                               "--temp", str(temp)], text=True, capture_output=True, check=False, env=env)
        if prep.returncode:
            raise RuntimeError(prep.stdout + prep.stderr)
        print(prep.stdout, end="", flush=True)
        command = ("addpath(" + matlab_quote(str(HERE)) + ");run_matrix_native(" +
                   ",".join(matlab_quote(str(path)) for path in (ROOT, temp, OUT)) + ");")
        log = temp / "batch.log"
        with log.open("w") as stream:
            proc = subprocess.Popen(["matlab", "-batch", command], stdout=stream, stderr=subprocess.STDOUT, env=env)
            while proc.poll() is None:
                size = tree_bytes(temp)
                audit["temporary_peak_polled_bytes"] = max(audit["temporary_peak_polled_bytes"], size)
                if size > TEMP_CAP:
                    proc.terminate()
                    proc.wait(timeout=30)
                    raise RuntimeError("Native temporary allocation exceeded.")
                time.sleep(2)
        audit["matlab_exit_code"] = proc.returncode
        # Keep failures legible without retaining an unbounded verbose batch log.
        text = log.read_text(errors="replace")
        useful = [line for line in text.splitlines() if line.startswith("MATRIX_NATIVE") or "Error" in line or "error" in line]
        audit["batch_messages"] = [line[:1500] for line in useful[-20:]]
        if proc.returncode:
            audit["batch_failure_tail"] = text[-12000:]
            raise RuntimeError(f"Native MATLAB batch failed with exit{proc.returncode}.")
        result = json.loads((OUT / "native_results.json").read_text())
        for case in result["cases"]:
            archive = Path(case["traceArchive"])
            import gzip
            original = gzip.decompress(archive.read_bytes())
            if hashlib.sha256(original).hexdigest() != case["traceUncompressedSha256"]:
                raise RuntimeError("Native compact trace gzip roundtrip failed.")
        audit["status"] = "private_native_batch_complete"
        audit["trace_roundtrip_hashes_passed"] = True
    except Exception as error:
        audit["status"] = "private_native_batch_failed"
        audit["failure"] = str(error)
        raise
    finally:
        audit["elapsed_s"] = time.monotonic() - start
        audit["temporary_peak_polled_bytes"] = max(audit["temporary_peak_polled_bytes"], tree_bytes(temp))
        if (temp / "owned_matrix_native").is_file():
            shutil.rmtree(temp)
        audit["temporary_directory_released"] = not temp.exists()
        number = 1
        while (OUT / f"launch_attempt{number:02d}.json").exists():
            number += 1
        (OUT / f"launch_attempt{number:02d}.json").write_text(json.dumps(audit, indent=2) + "\n")
        retained = tree_bytes(OUT) + tree_bytes(HERE)
        if retained > RETAINED_CAP:
            raise RuntimeError(f"Native retained allocation exceeded: {retained}>{RETAINED_CAP}.")
    print(json.dumps(dict(status=audit["status"], retained_bytes=retained,
                          temporary_peak_bytes=audit["temporary_peak_polled_bytes"])), flush=True)


if __name__ == "__main__":
    main()
