"""Revise the existing nested report using reviewed unified-RL evidence.

Preserves all 54 previously reviewed queries, report identity, presentation
revision, and protected runtime. This is a report authoring recipe, not a
simulation or an assertion that planned evaluations were executed.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "reports/prototype-design-20260930"
PREVIOUS = ROOT / "acf_simulink/validation/rl_report_20261005/previous_delivery"
REPORT_ID = "report:578d0259-115c-40f2-9264-4c57b68ab138"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--markdown", type=Path, required=True)
    parser.add_argument("--inputs", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--figure-root", type=Path)
    parser.add_argument("--complete", action="store_true")
    args = parser.parse_args()
    inputs = json.loads(args.inputs.read_text())
    original = json.loads((PREVIOUS / "02_data.json").read_text())
    current = json.loads((APP / "src/data.json").read_text())
    assert original["id"] == current["id"] == REPORT_ID
    assert current.get("authoredRevision") == original.get("authoredRevision")
    assert all(current["queries"].get(k) == v for k, v in original["queries"].items())
    assert len(original["queries"]) == 54
    queries = dict(original["queries"])
    for entry in inputs["queries"]:
        path = ROOT / entry["file"]
        document = json.loads(path.read_text())
        assert sha(path) == entry["sha256"], path
        assert document["status"] == "complete", path
        rows = document
        for field in entry.get("rowsPath", ["rows"]):
            rows = rows[field]
        assert isinstance(rows, list) and rows, path
        assert entry["id"] not in original["queries"], entry["id"]
        sources = {name: sha(ROOT / name) for name in entry.get("sourceFiles", [])}
        if args.complete or "sourceHashes" in document:
            assert document.get("sourceHashes") == sources, f"Evidence changed after preparation: {path}"
        queries[entry["id"]] = {"rows": rows, "source": {
            "label": entry["label"], "files": [entry["file"], *sources],
            "sha256": sha(path), "sourceFileSha256": sources,
            "kind": entry["kind"], "caveats": entry.get("caveats", []),
        }}

    spec = importlib.util.spec_from_file_location(
        "nested_report", Path(__file__).with_name("build_nested_ml_report_20261005.py"))
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    original_source_ids = helper.source_ids
    rl_ids = [entry["id"] for entry in inputs["queries"]]

    def source_ids(path, config):
        ids = original_source_ids(path, config)
        if len(path) > 1 and path[1] == "Learned control policy":
            return inputs.get("sectionQueries", {}).get(path[-1], rl_ids)
        if path[0] == "Recommendation" or (len(path) == 1 and path[0] == "Results"):
            return list(dict.fromkeys([q for q in ids if not q.startswith("ml_")] + rl_ids))
        if len(path) > 1 and path[1] == "Robot commands and recentering":
            return list(dict.fromkeys(ids + inputs.get("robotQueryIds", [])))
        if len(path) > 1 and path[1] == "Actuator and electrical drive":
            return list(dict.fromkeys(ids + inputs.get("actuatorQueryIds", [])))
        if len(path) > 1 and path[1] == "Acceleration measurement":
            return list(dict.fromkeys(ids + inputs.get("accelerationQueryIds", [])))
        if len(path) > 1 and path[1] == "Acquisition electronics: amplifier, ADC and timing":
            return list(dict.fromkeys(ids + inputs.get("acquisitionQueryIds", [])))
        if path[0] in ("Intro", "Question"):
            return list(dict.fromkeys(ids + inputs.get("methodQueryIds", [])))
        return ids

    helper.source_ids = source_ids
    config_path = ROOT / "acf_simulink/validation/budget_sensing_report_inputs_20261002.json"
    config = json.loads(config_path.read_text())
    config["newFigures"] = inputs["figures"]
    markdown = args.markdown.read_text()
    if args.complete:
        assert inputs["status"] == "complete"
        assert "**Study in progress.**" not in markdown
        assert "**Pending executed evidence.**" not in markdown
        assert "PLACEHOLDER" not in markdown and "Internal methods draft" not in markdown
    for figure in inputs["figures"]:
        figure_path = (args.figure_root or args.markdown.parent) / figure["file"]
        assert sha(figure_path) == figure["sha256"], f"Figure changed after preparation: {figure_path}"
    assets, sections = helper.render_content(markdown, args.figure_root or args.markdown.parent, config, queries)
    status = "complete" if args.complete else "updating"
    data = {**current, "queries": queries, "buildStatus": status,
            "report": {**current.get("report", {}), "asOf": "2026-10-05"}}
    assert data["authoredRevision"] == original["authoredRevision"]
    assert all(data["queries"][k] == v for k, v in original["queries"].items())
    (APP / "src/data.json").write_text(json.dumps(data, ensure_ascii=False, allow_nan=False) + "\n")
    args.output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "status": status, "reportId": REPORT_ID,
        "authoredRevision": data["authoredRevision"],
        "preservedOriginalQueries": len(original["queries"]),
        "markdown": str(args.markdown.resolve()), "markdownSha256": sha(args.markdown),
        "inputsSha256": sha(args.inputs), "previousDataSha256": sha(PREVIOUS / "02_data.json"),
        "queries": {k: {"rowCount": len(v["rows"]), "source": v["source"]}
                    for k, v in queries.items()},
        "figures": assets, "topLevelBlocks": [section["title"] for section in sections],
        "componentBlocks": [child["title"] for section in sections for child in section["children"]],
    }
    (args.output / "html_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"status": status, "reportId": REPORT_ID,
                      "queries": len(queries), "figures": len(assets)}))


if __name__ == "__main__":
    main()
