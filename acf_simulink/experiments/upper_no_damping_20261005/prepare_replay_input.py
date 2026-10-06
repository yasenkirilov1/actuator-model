"""Convert a hash-checked retained native recording to an independent MAT replay."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.io import savemat


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--history", required=True)
    parser.add_argument("--scenario", type=int, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--expected-sha256", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    history = Path(args.history).resolve()
    output = Path(args.output).resolve()
    assert not output.exists(), "Preserve every prior replay input."
    actual = hashlib.sha256(history.read_bytes()).hexdigest()
    assert actual == args.expected_sha256
    with np.load(history) as source:
        data = {key: source[key] for key in ["time", "inputs", "truth"]}
    assert data["inputs"].shape == (len(data["time"]), 3)
    assert data["truth"].shape == (len(data["time"]), 27)
    assert np.all(data["inputs"][:, 2] > 0.5)
    data.update(scenario=args.scenario, seed=args.seed, sourceRecording=str(history), sourceRecordingSha256=actual)
    output.parent.mkdir(parents=True, exist_ok=True)
    savemat(output, data, do_compression=True)
    result = {"status": "independent_replay_input_prepared_not_executed", "scenario": args.scenario,
              "seed": args.seed, "nativeSamples": len(data["time"]), "sourceRecording": str(history),
              "sourceRecordingSha256": actual, "recordingMatSha256": hashlib.sha256(output.read_bytes()).hexdigest()}
    output.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
