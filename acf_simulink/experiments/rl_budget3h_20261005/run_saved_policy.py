"""Reproduce a locked controller without choosing weights or changing physics.

The selected_controller package must already contain the reviewed checkpoint,
export hashes, frozen source and cohort manifest with selection reviews. This
wrapper dispatches its frozen final_evaluate.py using only the manifest's banks
and native binaries, then writes into a new reproduction directory.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[3]
STUDY = ROOT / "acf_simulink/validation/rl_budget3h_20261005"
PYTHON = Path("/home/tommie/Pesten/puffer4-native/.venv/bin/python")
COHORTS = ("holdout32", "nominal700", "long4")


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read(path: Path):
    if not path.is_file():
        raise ValueError(f"The locked package is incomplete: {path}")
    return json.loads(path.read_text())


def check_files(directory: Path, hashes: dict, required: tuple[str, ...]):
    if not isinstance(hashes, dict) or not all(name in hashes for name in required):
        raise ValueError(f"The package manifest does not bind its required files: {directory}")
    for name, expected in hashes.items():
        path = directory / name
        if path.resolve().parent != directory.resolve() or not path.is_file():
            raise ValueError(f"Invalid or missing frozen file: {path}")
        if sha(path) != expected:
            raise ValueError(f"A locked file has changed: {path}")


def study_path(study: Path, relative: str) -> Path:
    """Resolve an explicitly locked path; no implicit bank-name fallback."""
    supplied = Path(relative)
    path = (study / supplied).resolve()
    if supplied.is_absolute() or path == study or study not in path.parents:
        raise ValueError(f"A cohort path must remain relative to the study: {relative}")
    return path


def check_hash(path: Path, expected: str, label: str):
    if not path.is_file() or sha(path) != expected:
        raise ValueError(f"The locked cohort {label} is missing or changed: {path}")


def plan_run(study: Path, cohort: str, output: Path | None = None,
             workers: int | None = None, python: Path = PYTHON):
    """Validate the saved package and return the exact subprocess command."""
    study = study.resolve()
    selected = study / "selected_controller"
    source = selected / "source"
    export = read(selected / "policy_export_manifest.json")
    check_files(selected, export.get("files"), (
        "policy_checkpoint.pt", "policy_inference.c", "policy_inference.h",
        "policy_weights.f32", "policy_weights.json", "policy_weights.npz",
    ))
    frozen = read(source / "frozen_source_manifest.json")
    check_files(source, frozen.get("files"), (
        "final_evaluate.py", "policy.py", "policy_export.py", "env.py",
        "reward_search.py", "reward_search_native.c",
    ))

    checkpoint = selected / "policy_checkpoint.pt"
    cohorts = read(selected / "cohort_manifest.json")
    if cohorts.get("selectedCheckpointSha256") != sha(checkpoint):
        raise ValueError("The cohort manifest is bound to different controller weights")
    settings = cohorts["cohorts"][cohort]
    if settings["evaluationSplit"] not in ("holdout", "qualification"):
        raise ValueError("The cohort manifest specifies an unsupported evaluation split")
    if int(settings["cases"]) <= 0 or not isinstance(settings["aggregateOnly"], bool):
        raise ValueError("The cohort manifest has invalid case or storage settings")
    bank = study_path(study, settings["bank"])
    native = study_path(study, settings["nativeBuild"])
    review_path = study_path(study, settings["selectionReview"])
    check_hash(bank, settings["bankSha256"], "bank")
    check_hash(review_path, settings["selectionReviewSha256"], "selection review")
    for name, field in (
        ("libacf_native.so", "nativeLibrarySha256"),
        ("metadata.json", "nativeMetadataSha256"),
        ("scenarios.json", "nativeScenariosSha256"),
        ("bridge_build.json", "nativeBridgeBuildSha256"),
    ):
        check_hash(native / name, settings[field], name)
    review = read(review_path)
    scenarios = read(bank)
    metadata = read(native / "metadata.json")
    expected = {
        "selectedCheckpointSha256": sha(checkpoint),
        "evaluationBankSha256": sha(bank),
        "sourceModelSha256": metadata["sourceModelSha256"],
        "selectionOnly": True,
    }
    if any(review.get(name) != value for name, value in expected.items()):
        raise ValueError(f"The selection review does not match the locked package: {review_path}")
    if scenarios["sourceModelSha256"] != metadata["sourceModelSha256"]:
        raise ValueError("The evaluation bank and native model use different physical sources")
    if len([row for row in scenarios["cases"] if row["split"] == settings["evaluationSplit"]]) != settings["cases"]:
        raise ValueError(f"The bank does not contain the complete {cohort} cohort")

    selection_bank = study_path(study, cohorts["selectionBank"])
    check_hash(selection_bank, cohorts["selectionBankSha256"], "selection bank")
    selection_cases = read(selection_bank)
    if selection_cases["sourceModelSha256"] != metadata["sourceModelSha256"]:
        raise ValueError("Selection and evaluation banks use different physical sources")
    selection_ids = [i for i, row in enumerate(selection_cases["cases"])
                     if row["split"] == "selection"]
    if sorted(review.get("selectionIDs", [])) != selection_ids:
        raise ValueError("The review does not cover every development selection identity")
    if settings["evaluationSplit"] == "qualification" and review.get("selectionBankSha256") != sha(selection_bank):
        raise ValueError("The fresh qualification review does not bind the reviewed selection bank")
    if settings["evaluationSplit"] == "holdout" and sha(bank) != sha(selection_bank):
        raise ValueError("The holdout must use its reviewed development bank")
    for evidence in review.get("selectionEvidence", []):
        path = Path(evidence["file"])
        if not path.is_file() or sha(path) != evidence["sha256"]:
            raise ValueError(f"Reviewed selection evidence has changed: {path}")
    if not python.is_file():
        raise ValueError(f"The training runtime is unavailable: {python}")

    if workers is None:
        workers = settings["workers"]
    if not 1 <= workers <= 4:
        raise ValueError("workers must be between 1 and 4")
    if output is None:
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        output = study / "reproductions" / f"{cohort}_{stamp}"
    output = output.resolve()
    if output.exists():
        raise ValueError(f"Use a new output directory; this one already exists: {output}")
    if output == study or output == selected or selected in output.parents:
        raise ValueError("A reproduction cannot write into the locked controller package")

    command = [str(python), str(source / "final_evaluate.py"),
               "--checkpoint", str(checkpoint), "--policy-source-dir", str(source),
               "--native-build", str(native), "--bank", str(bank),
               "--selection-review", str(review_path), "--output", str(output),
               "--baseline", "--run-locked-holdout", "--workers", str(workers)]
    if settings["evaluationSplit"] == "qualification":
        command += ["--evaluation-split", "qualification", "--selection-bank", str(selection_bank)]
    if settings["aggregateOnly"]:
        command += ["--aggregate-only"]
    return command, output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cohort", choices=COHORTS, default="holdout32")
    parser.add_argument("--output", type=Path, help="New output directory; the default is timestamped")
    parser.add_argument("--workers", type=int, choices=range(1, 5),
                        help="Independent native processes; default from the locked cohort manifest")
    parser.add_argument("--dry-run", action="store_true", help="Validate and print the command without simulating")
    args = parser.parse_args()
    try:
        command, output = plan_run(STUDY, args.cohort, args.output, args.workers)
    except (ValueError, KeyError, json.JSONDecodeError) as error:
        parser.exit(2, f"Saved controller is not ready: {error}\n")
    print(f"Reproduction output: {output}", flush=True)
    print(shlex.join(command), flush=True)
    if args.dry_run:
        return
    result = subprocess.run(command, cwd=ROOT)
    if result.returncode:
        parser.exit(result.returncode, f"Evaluation stopped; inspect preserved results at {output}\n")
    document = read(output / "results.json")
    if document.get("status") != "complete" or not document.get("lockedInputsUnchanged"):
        parser.exit(1, f"Evaluation is incomplete; inspect preserved results at {output}\n")
    print(f"Completed {document['completedRecordings']} paired-controller recordings. "
          f"Results: {output / 'results.json'}", flush=True)


if __name__ == "__main__":
    main()
