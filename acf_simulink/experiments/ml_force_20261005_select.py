#!/usr/bin/env python3
"""Freeze physical selection before opening final held-out trajectories."""
from pathlib import Path
import json
import argparse

root = Path(__file__).resolve().parents[2]
study = root / "acf_simulink/validation/ml_force_20261005"
parser = argparse.ArgumentParser()
parser.add_argument("--study", type=Path, default=study)
study = parser.parse_args().study.resolve()
report = json.loads((study / "select/results.json").read_text())
assert report["status"] == "complete" and len(report["rows"]) == 12
assert not (study / "selected_gain.json").exists(), "Preserve frozen selection."
baseline = [r for r in json.loads((study / "collect/results.json").read_text())["rows"] if r["group"] == "selection"]
gains = []
for gain in [.25, .5, 1.]:
    rows = [r for r in report["rows"] if r["gain"] == gain]
    assert len(rows) == 4 and {r["id"] for r in rows} == {1, 2, 3, 4}
    gains.append({"gain": gain, "passed": sum(r["passed"] for r in rows),
                  "worstError_N": max(r["peakNormalForceError_N"] for r in rows),
                  "meanPeakError_N": sum(r["peakNormalForceError_N"] for r in rows)/4})
selected = min(gains, key=lambda r: (4-r["passed"], r["worstError_N"], r["meanPeakError_N"]))
base_peak = max(r["peakNormalForceError_N"] for r in baseline)
result = {"status": "selected_before_final_test", "selectedGain": selected["gain"],
          "selectionRule": "Among predeclared nonzero gains 0.25, 0.5, 1, minimize failures then worst full-native peak, then mean peak. Baseline remains an explicit comparator.",
          "baselineWorstError_N": base_peak, "gains": gains,
          "selectionSupportsImprovement": selected["worstError_N"] < base_peak,
          "allSelectionRecordingsIndependentOfTraining": True, "calibrationStressUsedForSelection": False,
          "finalHoldoutReadOrUsed": False}
(study / "selected_gain.json").write_text(json.dumps(result, indent=2)+"\n")
print(json.dumps(result, indent=2))
