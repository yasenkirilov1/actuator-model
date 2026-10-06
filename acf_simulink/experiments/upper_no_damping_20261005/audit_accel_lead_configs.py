"""Assert that all 66 lead cases preserve the paired physical/sensing inputs."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat


def differences(before: object, after: object, path: str = "p"):
    if isinstance(before, dict) and isinstance(after, dict):
        for key in sorted(set(before) | set(after)):
            if key not in before or key not in after:
                yield path + "." + key
            else:
                yield from differences(before[key], after[key], path + "." + key)
    elif isinstance(before, np.ndarray) or isinstance(after, np.ndarray):
        if not np.array_equal(np.asarray(before), np.asarray(after), equal_nan=True):
            yield path
    elif before != after:
        yield path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline")
    parser.add_argument("candidate")
    parser.add_argument("output")
    args = parser.parse_args()
    before_path, after_path, output = map(Path, [args.baseline, args.candidate, args.output])
    assert not output.exists()
    before = loadmat(before_path, simplify_cells=True)["configs"]
    after = loadmat(after_path, simplify_cells=True)["configs"]
    assert len(after) == 66
    allowed = {"p.accelLead", "p.d.accelLead", "p.mechanicalStudy.controlDevelopmentLane", "p.mechanicalStudy.gainID"}
    changed = set()
    for index, case in enumerate(after):
        case_changes = set(differences(before[index % 11]["p"], case["p"]))
        assert case_changes <= allowed, (index, case_changes - allowed)
        changed.update(case_changes)
        d = case["p"]["d"]["accelLead"]
        assert abs(d["pole"]) < 1 and abs(np.sum(d["numerator"]) / np.sum(d["denominator"]) - 1) < 1e-12
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    record = {
        "status": "all_prepared_lead_inputs_match_original_physics_and_sensing",
        "comparedPreparedCases": 66,
        "createdPhysicalEpisodes": 0,
        "baselineConfigsSha256": sha(before_path),
        "candidateConfigsSha256": sha(after_path),
        "allChangedParameterPaths": sorted(changed),
        "physicalAndSensingParametersExactlyEqual": True,
        "allSixFiltersStableAndDCGainOne": True,
        "comparison": "Exact recursive comparison against original profile1 cases0:10. No tolerance or omitted physical/sensor fields. The only changes are the declared filter and development metadata.",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
