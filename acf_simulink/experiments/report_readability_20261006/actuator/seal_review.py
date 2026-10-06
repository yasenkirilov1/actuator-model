"""Bind the completed bounded figure review to the exact supplied SVG bytes.

Run only after inspecting all four current PNG renders. This script verifies
source and artifact hashes; it does not perform or claim a browser/mobile check.
"""
from pathlib import Path
import csv
import hashlib
import json

ROOT = Path(__file__).resolve().parents[4]
OUT = ROOT / "acf_simulink/validation/report_readability_20261006/actuator"


def sha(p):
    h = hashlib.sha256()
    with Path(p).open("rb") as f:
        for block in iter(lambda: f.read(4*1024*1024), b""):
            h.update(block)
    return h.hexdigest()


source = OUT / "actuator_evidence.json"
data = json.loads(source.read_text())
assert data["newNativeExperiments"] == 0 and data["prototypeMeasurements"] == 0
checks = {"sourceHashes": {}, "figures": {}, "csvRows": {}}
for rel, digest in data["sourceHashes"].items():
    actual = sha(ROOT / rel)
    assert actual == digest, rel
    checks["sourceHashes"][rel] = {"sha256": actual, "passed": True}
for figure in data["figures"]:
    for artifact in figure["artifacts"]:
        assert sha(ROOT / artifact["path"]) == artifact["sha256"]
    figure["visuallyReviewed"] = True
    figure["reviewScope"] = "Static paired SVG/PNG figures; labels, axes, units, limits, native metrics and visible source qualifications reviewed."
    checks["figures"][figure["id"]] = {
        "passed": True, "visibleLabelsAndUnits": True, "observedOverlapsCorrected": True,
        "colorIndependentEncoding": "Direct values/axis labels; solid versus hatched historical bars and solid versus dashed voltage traces.",
        "artifacts": figure["artifacts"]}
for path in sorted(OUT.glob("*.csv")):
    with path.open(newline="") as f: rows = list(csv.DictReader(f))
    assert rows and len(rows[0]) > 1
    checks["csvRows"][str(path.relative_to(ROOT))] = {"rows": len(rows), "sha256": sha(path)}
assert len(checks["csvRows"]) == 7
assert len(data["historicalComparison"]["rows"]) == 4
assert data["historicalComparison"]["fullHistoriesRechecked"] == 8
assert data["fixedVoltageDiagnostics"]["identicalUnclippedNativeArrays"]
assert data["fixedVoltageDiagnostics"]["qualificationEpisodes"] == 0
assert abs(data["voiceCoilTargets"]["driverLimitedPeakForce_N"] - 85.4) < 1e-12
assert data["currentQualificationTotals"]["passed"] == 705
primary = OUT / "primary_sources_review_20261006.json"
assert data["sourceHashes"][str(primary.relative_to(ROOT))] == sha(primary)
builder = ROOT / "acf_simulink/experiments/report_readability_20261006/actuator/build_actuator_evidence.py"
data["status"] = "reviewed_complete_bounded_actuator_evidence"
data["reviewedDate"] = "2026-10-06"
data["reviewer"] = "/root/voicecoil_report_evidence"
data["rawBuildFile"] = str(source.relative_to(ROOT))
data["rawBuildSha256"] = sha(source)
data["builderFile"] = str(builder.relative_to(ROOT))
data["builderSha256"] = sha(builder)
data["checks"] = checks
data["reviewNotes"] = "The exact current four static renders were inspected. Initial rating-label and footnote overlaps were corrected and re-rendered. No new simulation or plant modification occurred. Browser interactions/mobile layout are outside this static evidence package."
reviewed = OUT / "actuator_evidence_reviewed.json"
reviewed.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")
manifest = {
    "status": "reviewed_complete", "reviewedDate": "2026-10-06",
    "newNativeRuns": 0, "reviewedEvidence": str(reviewed.relative_to(ROOT)),
    "reviewedEvidenceSha256": sha(reviewed),
    "files": [{"path": str(p.relative_to(ROOT)), "sha256": sha(p), "bytes": p.stat().st_size}
              for p in sorted(OUT.rglob("*")) if p.is_file() and p.name != "artifact_manifest.json"],
    "scope": "Only new owned actuator evidence, CSVs, source PDF/render and fourSVG candidates. Frozen sources/histories are referenced and hash-checked, not copied or modified."
}
(OUT / "artifact_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps({"status": "reviewed_complete", "sourceFilesVerified": len(checks["sourceHashes"]),
                  "figuresReviewed": len(checks["figures"]), "csvFiles": len(checks["csvRows"]),
                  "reviewedEvidenceSha256": sha(reviewed),
                  "artifactBytes": sum(f["bytes"] for f in manifest["files"])}))
