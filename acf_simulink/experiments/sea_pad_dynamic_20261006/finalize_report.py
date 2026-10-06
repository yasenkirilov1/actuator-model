"""Audit the completed study and optionally deliver its Markdown report to Notes.

Does not run simulations or rewrite scientific evidence. Run with --deliver
once the report and figures are reviewed; without it, require delivery parity.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
from datetime import datetime, timezone
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
RECIPE = Path(__file__).resolve().parent
STUDY = ROOT / "acf_simulink/validation/sea_pad_dynamic_20261006"
GENERATED = ROOT / "acf_simulink/generated/sea_pad_dynamic_20261006"
OUT = STUDY / "report"
REPORT = ROOT / "reports/active-contact-flange-report-design-explained-20261006.md"
NOTES = Path("/home/tommie/Documents/Notes Remote/S7 Project")


def load(path):
    return json.loads(Path(path).read_text())


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def bound(path):
    path = Path(path)
    label = str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path)
    return {"path": label, "sha256": digest(path), "bytes": path.stat().st_size}


def verify(item):
    path = Path(item["path"])
    if not path.is_absolute():
        path = ROOT / path
    assert digest(path) == item["sha256"], str(path)


def save(name, data):
    (OUT / name).write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n")


def files_under(folder):
    return {p for p in Path(folder).rglob("*") if p.is_file()}


def physical_bytes(paths):
    seen = set()
    total = 0
    for path in paths:
        stat = path.stat()
        identity = (stat.st_dev, stat.st_ino)
        if identity not in seen:
            seen.add(identity)
            total += stat.st_size
    return total


def shared_array_audit(scopes):
    seen_dirs, seen_files = set(), set()
    total, count = 0, 0
    for scope in scopes:
        for directory, dirs, names in os.walk(scope, followlinks=True):
            stat = os.stat(directory)
            identity = (stat.st_dev, stat.st_ino)
            if identity in seen_dirs:
                dirs[:] = []
                continue
            seen_dirs.add(identity)
            for name in names:
                path = Path(directory) / name
                if path.suffix.lower() not in {".mat", ".npz", ".npy", ".bin"}:
                    continue
                stat = path.stat()
                identity = (stat.st_dev, stat.st_ino)
                if identity not in seen_files:
                    seen_files.add(identity)
                    count += 1
                    total += stat.st_size
    return {"scopes": scopes, "count": count, "bytes": total,
            "extensions": [".mat", ".npz", ".npy", ".bin"],
            "symlinks_and_hardlinks_deduplicated": True}


def grid_summary(name):
    path = STUDY / "evaluation" / f"{name}_pad5mm_k50_c0.json"
    data = load(path)
    rows = data["records"]
    assert data["executed_count"] == data["pass_count"] == len(rows) == 54
    assert data["physical_stop_count"] == data["preparation_rejected_count"] == 0
    identities = set()
    for row in rows:
        p, m = row["config"], row["metrics"]
        assert m["force_pass"] and m["completed"]
        assert m["sensor_ratings_pass"] and m["actuator_ratings_pass"]
        assert p["pad_thickness"] == .005 and p["pad_diameter"] == .077
        assert p["pad_k"] == 50000 and p["pad_c"] == 0
        assert p["upper_k"] == 20000 and p["cell_k"] == 64000000
        assert p["mass_a"] == 1.5 and p["mass_b"] == .6
        assert p["robot_enabled"] == 0 and p["duration"] == 2
        identities.add((p["mass_s"], p["force_target"], p["angle_deg"], p["seed"]))
    expected = {(s, f, a, n) for s in [.1, 1.4, 2] for f in [5, 20, 50]
                for a in [0, 45, 90] for n in [260651, 260652]}
    assert identities == expected
    metrics = [r["metrics"] for r in rows]
    worst = max(m["force_peak_error"] for m in metrics)
    assert abs(worst - data["worst_error_N"]) < 1e-12
    return {"source": bound(path), "executed": 54, "passes": 54,
            "worst_error_N": worst,
            "maximum_average_hot_winding_loss_W": max(m["copper_loss_average_W"] for m in metrics),
            "maximum_current_rms_A": max(m["current_rms"] for m in metrics),
            "maximum_voltage_request_V": max(m["voltage_peak_request"] for m in metrics),
            "maximum_pad_compression_m": max(m["pad_peak_compression"] for m in metrics),
            "raw_cell_min_N": min(m["cell_raw_min"] for m in metrics),
            "raw_cell_max_N": max(m["cell_raw_max"] for m in metrics),
            "gains": data["candidate_lock"]["gains"],
            "locked_before_evaluation": data["candidate_lock"]["locked_before_evaluation"]}


def main(deliver):
    checked_utc = datetime.now(timezone.utc).isoformat()
    protected = load(OUT / "protected_inputs_before.json")["files"]
    for item in protected:
        verify(item)
    engine_manifest = load(STUDY / "engine/evidence_manifest.json")
    verify({"path": engine_manifest["summary_path"], "sha256": engine_manifest["summary_sha256"]})
    for item in engine_manifest["sources"]:
        verify(item)
    summary = load(STUDY / "engine/final_summary.json")
    assert summary["all_primary_recordings"]["executed_record_count"] == 1975
    assert summary["all_primary_recordings"]["scored_integration_endpoint_count"] == 1403759763
    assert summary["categories"]["primary_development"]["executed_record_count"] == 1049
    assert summary["categories"]["primary_evaluation_and_diagnostics"]["executed_record_count"] == 926
    assert summary["categories"]["observation_only_instrumentation_reruns"]["executed_record_count"] == 216
    assert sum(summary["known_presentation_reruns"].values()) == 26
    assert digest(RECIPE / "engine.c") == summary["current_engine_source_sha256"]
    assert digest(RECIPE / "engine_api.py") == summary["current_api_sha256"]
    assert digest(RECIPE / "study_contract.json") == summary["base_contract_sha256"]
    extension = load(RECIPE / "pad5mm_contract.json")
    assert extension["base_contract_sha256"] == digest(RECIPE / "study_contract.json")

    model_manifest = load(GENERATED / "model_manifest.json")
    for path, item in model_manifest["files"].items():
        verify({"path": path, **item})
    native = load(STUDY / "native/validation_summary.json")
    assert native["fullNativeBaseSamplesScored"] == 7200009
    assert native["pad5mmBothNativeRegularAndOriginalSplitEventGuardsPassed"]
    assert not native["knownStrictGuardFailure"]["passed"]
    assert not native["scientificModelOrFullControllerQualified"]
    archives = load(STUDY / "native/trace_archive_manifest.json")["entries"]
    for item in archives:
        verify({"path": item["archive_path"], "sha256": item["archive_sha256"]})
        assert hashlib.sha256(gzip.decompress((ROOT / item["archive_path"]).read_bytes())).hexdigest() == item["uncompressed_sha256"]

    figures_manifest = load(OUT / "pad5mm_figure_manifest.json")
    for item in figures_manifest["figures"]:
        verify(item["figure"])
        for source in item["sources"]:
            verify(source)
    old_figures = load(OUT / "figure_manifest.json")["figures"]
    for item in old_figures:
        verify({"path": item["figure"], "sha256": item["sha256"]})
        for source in item.get("sources", []):
            verify(source)
        for source in item.get("trace_sources", []):
            path = ROOT / source["path"]
            content = path.read_bytes() if path.exists() else gzip.decompress(Path(str(path)+".gz").read_bytes())
            assert hashlib.sha256(content).hexdigest() == source["sha256"]

    report = REPORT.read_text()
    assert "How do soft pads influence our results?" in report
    image_paths = [REPORT.parent / p for p in re.findall(r"!\[[^\]]*\]\(([^)]+)\)", report)]
    assert len(image_paths) == 7
    for path in image_paths:
        ET.parse(path)
    links = re.findall(r"\[[^\]]*\]\(([^)]+)\)", report)
    local_links = [p for p in links if not p.startswith(("https://", "http://", "#"))]
    for value in local_links:
        path = Path(value) if value.startswith("/") else REPORT.parent / value
        assert path.exists(), value

    notes_report = NOTES / "active-contact-flange-report.md"
    previous_notes_hash = digest(OUT / "previous_report/Notes.md")
    if deliver:
        permitted_notes_hashes = {previous_notes_hash, digest(REPORT)}
        prior_delivery = OUT / "markdown_delivery_manifest.json"
        if prior_delivery.exists():
            permitted_notes_hashes.add(load(prior_delivery)["notes_report"]["sha256"])
        assert digest(notes_report) in permitted_notes_hashes, "Notes report changed independently; preserve and reconcile it."
        for path in image_paths:
            destination = NOTES / "figures" / path.name
            if destination.exists():
                assert digest(destination) == digest(path), f"Preserve independently changed Notes figure: {destination}"
        (NOTES / "figures").mkdir(exist_ok=True)
        for path in image_paths:
            (NOTES / "figures" / path.name).write_bytes(path.read_bytes())
        temporary = NOTES / ".acf-sea-pad-report-delivery.tmp"
        temporary.write_bytes(REPORT.read_bytes())
        temporary.replace(notes_report)
    assert digest(notes_report) == digest(REPORT)
    for path in image_paths:
        assert digest(NOTES / "figures" / path.name) == digest(path)

    sources = [RECIPE / name for name in ["study_contract.json", "pad5mm_contract.json", "engine.c", "engine_api.py", "README.md", "build_pad5mm_figures.py", "build_report_figures.py", "finalize_report.py"]]
    sources += [STUDY / name for name in ["engine/final_summary.json", "engine/evidence_manifest.json", "native/validation_summary.json", "native/trace_archive_manifest.json", "assumptions/source_bound_assumptions.json", "evaluation/conservative_material_sensitivity.json", "evaluation/pad5mm_exact_width_pairs.json", "evaluation/pad5mm_k20_guard_corners.json", "evaluation/voltage_limit_pairs.json", "evaluation/closed_loop_numerical_refinement.json", "report/pad5mm_figure_manifest.json", "report/figure_manifest.json", "storage_forecast.json"]]
    sources += [GENERATED / "model_manifest.json", ROOT / "acf_simulink/validation/report_readability_20261006/report/report_inputs_design_explained.json"]
    save("report_inputs.json", {
        "status": "completed_source_bound_markdown_report", "checked_utc": checked_utc,
        "scope": summary["scope"], "sources": [bound(p) for p in sources],
        "selected_5mm_grids": {name: grid_summary(name) for name in ["coil", "sea"]},
        "primary_recordings": summary["all_primary_recordings"],
        "native_base_samples_scored": native["fullNativeBaseSamplesScored"],
        "qualification": False,
        "metadata_notes": [
            "Native pad5mmContractSha256 records the inherited base contract. Both base and actual5mm extension are separately bound here.",
            "Extension allocation text was superseded by later redistribution inside the unchanged32MiB cap; final storage_forecast.json governs accounting.",
            "Earlier10mm figure plaintext traces are retained losslessly as gzip; their original content hashes still pass.",
            "Historical reference and ADC evidence remains separate; no renewed qualification or newADC sweep is claimed."]})
    save("markdown_delivery_manifest.json", {
        "status": "delivered_markdown_and_svg", "checked_utc": checked_utc,
        "report": bound(REPORT), "notes_report": bound(notes_report),
        "previous_notes_report_sha256": previous_notes_hash,
        "figures": [{"source": bound(p), "delivery": bound(NOTES / "figures" / p.name)} for p in image_paths],
        "html_updated": False, "protected_inputs_verified": len(protected)})

    recipe_files, study_files, generated_files = files_under(RECIPE), files_under(STUDY), files_under(GENERATED)
    engine_files = set().union(*(files_under(STUDY / n) for n in ["engine", "development", "evaluation"]))
    engine_files |= {p for p in recipe_files if p.parent == RECIPE and p.name.startswith(("engine", "runner"))}
    engine_files |= files_under(RECIPE / "__pycache__")
    native_files = files_under(STUDY / "native") | files_under(RECIPE / "native") | generated_files
    extra_reports = {REPORT, ROOT / "AGENTS.md", notes_report, *image_paths}
    extra_reports |= {NOTES / "figures" / p.name for p in image_paths}
    extra_reports |= {ROOT / p["figure"] for p in old_figures}
    root_files = (recipe_files | study_files | generated_files | extra_reports) - engine_files - native_files
    forecast = load(STUDY / "storage_forecast.json")
    sizes = {"engine": physical_bytes(engine_files), "native": physical_bytes(native_files), "root_report_assumptions": physical_bytes(root_files)}
    for name, size in sizes.items():
        assert size <= forecast["allocations"][name]["retained_bytes"], (name, size)
    total = physical_bytes(engine_files | native_files | root_files)
    assert total <= forecast["retained_total_limit_bytes"]
    array_scopes = load(ROOT / "acf_simulink/validation/storage_cleanup_20261006/bulk_plan.json")["scope"]
    array_audit = shared_array_audit(array_scopes)
    assert array_audit["bytes"] <= forecast["shared_array_budget_bytes"]
    study_arrays = [str(p.relative_to(ROOT)) for p in study_files if p.suffix.lower() in {".mat", ".npz", ".npy", ".bin"}]
    assert not study_arrays
    save("completion_checks.json", {
        "status": "completed_report_delivery_and_preservation_audit", "checked_utc": checked_utc,
        "protected_inputs": protected, "all_protected_hashes_passed": True,
        "engine_source_files_verified": len(engine_manifest["sources"]),
        "native_model_manifest_files_verified": len(model_manifest["files"]),
        "native_trace_roundtrips_verified": len(archives),
        "local_markdown_links_verified": len(local_links), "svg_figures_xml_checked": len(image_paths),
        "new_figures_visually_reviewed": True, "native_worker_temporary_cleanup": load(STUDY / "native/storage_completion.json"),
        "retained_bytes_by_owner_before_this_small_audit": sizes,
        "retained_total_bytes_before_this_small_audit": total,
        "retained_limit_bytes": forecast["retained_total_limit_bytes"],
        "conservative_accounting": "Includes complete report/AGENTS and all seven deliveredSVGs, even unchanged reference figures; does not count unrelated old reports.",
        "shared_array_audit": array_audit, "shared_array_limit_bytes": forecast["shared_array_budget_bytes"],
        "new_study_bulk_arrays": study_arrays,
        "report_inputs": bound(OUT / "report_inputs.json"),
        "markdown_delivery_manifest": bound(OUT / "markdown_delivery_manifest.json"),
        "report_word_count_including_tables": len(report.split())})
    # Recount after the small audit file itself is present, avoiding a self-hash.
    final_root = root_files | files_under(OUT)
    final_total = physical_bytes(engine_files | native_files | final_root)
    assert physical_bytes(final_root) <= forecast["allocations"]["root_report_assumptions"]["retained_bytes"]
    assert final_total <= forecast["retained_total_limit_bytes"]
    print(json.dumps({"status": "passed", "report_delivered": str(notes_report),
                      "retained_study_MiB": round(final_total / 2**20, 3),
                      "shared_array_GiB": round(array_audit["bytes"] / 2**30, 3),
                      "protected_files": len(protected), "native_trace_roundtrips": len(archives)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--deliver", action="store_true")
    main(parser.parse_args().deliver)
