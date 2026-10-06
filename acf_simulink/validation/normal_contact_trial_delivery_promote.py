"""Promote the accepted delivery graph and exactly four owned helpers.

This performs the filesystem-only promotion gates in promote_inertial_delivery.m.
All native runs were completed beforehand. No MATLAB process is started here.
"""

import hashlib
import json
import shutil
from datetime import datetime, timezone
from pathlib import Path


def sha256(file):
    digest = hashlib.sha256()
    with file.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_receipt(work, name, flags):
    data = json.loads((work / name).read_text())
    assert data["status"] == "complete"
    assert all(data[key] is True for key in flags)
    return data


def main():
    root = Path(__file__).resolve().parents[1]
    work = Path.home() / ".local/state/actuator-model/inertial-delivery-private-v8-layoutd-20260930"
    model = work / "generated/acf_force_control_simple.slx"
    reviewed = "48287e6a17bc9d3144e3d6a77c59ecc601ba4fa26a7ec748173dc89d9bf718de"
    assert sha256(model) == reviewed
    owned = ["acf_prepare.m", "acf_select_actuator.m", "acf_select_controller.m", "acf_parameters.m"]
    receipts = {
        "delivery_verification.json": ["sourceParityPassed", "massEditPassed", "nativeStructurePassed", "saveReopenPassed"],
        "delivery_defaults_verification.json": ["scanEditPassed", "delayEditPassed", "slowToolExamplePassed"],
        "delivery_acceptance.json": ["sourceBridgePassed", "directRunPassed", "guideSlowExamplePassed", "contactStopPassed", "travelStopPassed", "noSpringsPresetPassed", "saveReopenPassed"],
        "sensor_metadata_bridge.json": ["allParametersExceptSensorDesignIdentical", "nativeSystemXMLIdentical"],
        "line_cleanup_bridge.json": ["nativeBlockParametersIdentical", "nativeSystemParametersIdentical", "connectedSourceDestinationEdgesIdentical", "modelWorkspaceIdentical"],
    }
    evidence = {name: read_receipt(work, name, flags) for name, flags in receipts.items()}
    assert evidence["line_cleanup_bridge.json"]["modelSHA256"] == reviewed
    assert evidence["line_cleanup_bridge.json"]["beforeModelSHA256"] == evidence["sensor_metadata_bridge.json"]["modelSHA256"]
    assert evidence["sensor_metadata_bridge.json"]["sourceModelSHA256"] == "4fe56e2505f10e9adef86b29a7615583ec2b6dc6be37f1cf051d3a753fef0a44"
    assert evidence["line_cleanup_bridge.json"]["destinationlessLeavesAfter"] == 0
    candidate = work / "delivery_candidate.mat"
    assert sha256(candidate) == evidence["sensor_metadata_bridge.json"]["candidateSHA256"]
    assert sha256(work / "helpers/acf_prepare.m") == evidence["sensor_metadata_bridge.json"]["preparationSHA256"]
    diagrams = root / "validation/normal_contact_trial_delivery_diagrams_20260930.json"
    diagram_data = json.loads(diagrams.read_text())
    assert diagram_data["modelSHA256"] == reviewed and len(diagram_data["pages"]) == 13
    assert sha256(root / diagram_data["artifact"]) == diagram_data["PDFSHA256"]
    for page in diagram_data["pages"]:
        assert sha256(root / page["image"]) == page["imageSHA256"]
    protected = [p for p in root.glob("*.m") if p.name not in owned] + list(root.glob("*.md"))
    protected_hashes = {str(p.relative_to(root)): sha256(p) for p in protected}
    backup = root / "validation/normal_contact_trial_delivery_previous_20260930"
    assert not backup.exists(), "Keep the prior promoted version intact"
    backup.mkdir()
    targets = [(model, root / "generated/acf_force_control_simple.slx")]
    targets.extend((work / "helpers" / name, root / name) for name in owned)
    old_files = {}
    for _, destination in targets:
        assert destination.is_file() and not destination.is_symlink()
        shutil.copy2(destination, backup / destination.name)
        old_files[destination.name] = sha256(destination)
    for source, destination in targets:
        shutil.copy2(source, destination)
        assert sha256(source) == sha256(destination)
    assert all(sha256(root / name) == value for name, value in protected_hashes.items())
    public_evidence = {}
    for name in receipts:
        destination = root / "validation" / ("normal_contact_trial_" + name.removesuffix(".json") + "_20260930.json")
        shutil.copy2(work / name, destination)
        public_evidence[name] = {"file": str(destination.relative_to(root)), "SHA256": sha256(destination)}
    public_candidate = root / "validation/normal_contact_trial_delivery_candidate_v8_20260930.mat"
    shutil.copy2(candidate, public_candidate)
    native_traces = []
    for file in sorted((work / "traces").glob("delivery_*.mat")):
        native_traces.append({"file": str(file), "bytes": file.stat().st_size, "SHA256": sha256(file)})
    manifest = {
        "status": "promoted",
        "UTC": datetime.now(timezone.utc).isoformat(),
        "model": "generated/acf_force_control_simple.slx",
        "modelSHA256": reviewed,
        "ownedHelpers": {name: sha256(root / name) for name in owned},
        "untouchedRootFiles": protected_hashes,
        "previousCanonicalBackup": str(backup.relative_to(root)),
        "previousCanonicalFiles": old_files,
        "reviewedOriginalModel": str(work / "generated/acf_force_control_simple_before_sensor_metadata.slx"),
        "reviewedOriginalHelpers": str(work / "before_sensor_metadata_helpers"),
        "metadataOnlyOriginal": str(work / "generated/acf_force_control_simple_before_line_cleanup.slx"),
        "privateDeliveryWorkspace": str(work),
        "scientificSourceModelSHA256": "73bbc46529f3c28d51d17766c4b0042d278446d29b5fdb21910f93c310937643",
        "scientificFinalCandidate": "validation/inertial_final_candidate_20260930.mat",
        "scientificFinalCandidateSHA256": sha256(root / "validation/inertial_final_candidate_20260930.mat"),
        "publicPreparedCandidate": str(public_candidate.relative_to(root)),
        "publicPreparedCandidateSHA256": sha256(public_candidate),
        "evidence": public_evidence,
        "nativeTraces": native_traces,
        "diagrams": {"manifest": str(diagrams.relative_to(root)), "SHA256": sha256(diagrams), "PDF": diagram_data["artifact"], "PDFSHA256": diagram_data["PDFSHA256"]},
        "nativeSourceBridge": evidence["delivery_acceptance.json"]["sourceBridge"],
        "visualReview": "Root reviewed all13 final pages and re-reviewed cleanup pages04/07/13 before authorizing promotion.",
        "newNativeSimulationsDuringMetadataCleanupAndPromotion": 0,
        "desktopModel": "No desktop model or window was altered; reopen the saved canonical graph before using the new helpers.",
        "recipe": str(Path(__file__).relative_to(root)),
    }
    output = root / "validation/normal_contact_trial_delivery_manifest_v8_20260930.json"
    output.write_text(json.dumps(manifest, indent=2) + "\n")
    for file in backup.iterdir():
        file.chmod(0o444)
    (work / "promotion.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"status": manifest["status"], "modelSHA256": reviewed, "manifest": str(output), "backup": str(backup)}))


if __name__ == "__main__":
    main()
