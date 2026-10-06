"""Bind a development-only selected native candidate before qualification.

Invoke only after the integrator selects the deterministic candidate. This
does not inspect qualification outputs or execute any physics.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from scipy.io import loadmat


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def entry(path: Path) -> dict:
    path = path.resolve()
    assert path.is_file()
    return {"path": str(path), "sha256": sha(path)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--native-build", type=Path, required=True)
    parser.add_argument("--selected-scenario", type=int, required=True)
    parser.add_argument("--expected-config-sha256", required=True)
    parser.add_argument("--expected-library-sha256", required=True)
    parser.add_argument("--selection-evidence", type=Path, action="append", required=True)
    parser.add_argument("--context-recipe", type=Path, required=True)
    parser.add_argument("--reason", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    assert not out.exists(), "Preserve prior selection decisions."
    build = args.native_build.resolve()
    metadata = json.loads((build / "metadata.json").read_text())
    assert metadata["algebraicLoopCount"] == 0
    configs_file = Path(metadata["configsFile"]).resolve()
    assert sha(configs_file) == args.expected_config_sha256 == metadata["configsSha256"]
    assert sha(build / "libacf_native.so") == args.expected_library_sha256
    configs = loadmat(configs_file, simplify_cells=True)["configs"]
    rows = [configs] if isinstance(configs, dict) else list(configs)
    chosen = rows[args.selected_scenario]
    assert chosen["split"] == "development"
    p = chosen["p"]
    assert p["mechanics"]["mA"] == 1.5 and p["mechanics"]["mB"] == .6
    assert p["mechanics"]["k1"] == 64e6 and p["mechanics"]["c1"] == 0
    assert p["mechanics"]["k2"] == 20000 and p["mechanics"]["c2"] == 0
    assert not p["mechanics"]["lowerRigid"] and not p["mechanics"]["upperRigid"]
    assert p["cellModal"]["enabled"] and p["upperObserver"]["enabled"]
    assert p["d"]["cellModal"]["identifiedMassB_kg"] == .6
    assert p["d"]["cellModal"]["identifiedForceConstant_N_A"] == 24.4
    assert p["d"]["cellModal"]["identifiedStiffness_N_m"] == 64e6
    assert p["coil"]["currentResponse"] == 50e-6
    recipe = json.loads(args.context_recipe.read_text())
    assert recipe["qualificationEpisodesExecuted"] == 0
    assert recipe["counts"] == {"nominal700": 700, "fresh32": 32, "long4": 4, "total": 736}
    source_files = {name: entry(build / name) for name in (
        "acf_rl_native.slx", "metadata.json", "scenarios.json", "bridge_build.json",
        "libacf_native.so", "native_parameter_bank.c", "acf_rl_native_context.c",
        "generated_cell_source_audit.json", "local_cell_source_guard.json",
        "modal_positive_damping_guard.json", "file_hashes.json")}
    helpers = [entry(path) for path in sorted((build / "helpers").glob("*.m"))]
    lock = {
        "status": "locked_development_candidate_before_qualification",
        "createdUtc": datetime.now(timezone.utc).isoformat(),
        "selectionOnly": True,
        "qualificationOutcomesInspected": False,
        "noRetuningOnQualification": True,
        "selectionReason": args.reason,
        "selectedScenario": args.selected_scenario,
        "selectedIdentity": chosen["identity"],
        "executingNormalForceControllerGains": {
            "forceKp_A_N": float(p["inertial"]["forceKp"]),
            "forceKi_A_N_s": float(p["inertial"]["forceKi"]),
            "forceKd_A_s_N": float(p["inertial"]["forceKd"]),
            "parameterSource": "p.inertial.forceKp/Ki/Kd",
        },
        "sourceNativeDirectory": str(build),
        "sourceNativeModel": entry(build / "acf_rl_native.slx"),
        "sourceModelSha256": metadata["sourceModelSha256"],
        "sourceConfigs": entry(configs_file),
        "sourceFiles": source_files,
        "sourceHelpers": helpers,
        "selectionEvidence": [entry(path) for path in args.selection_evidence],
        "contextRecipe": entry(args.context_recipe),
        "recipeSource": entry(Path(__file__)),
        "frozenModalIdentification": {
            "identifiedMassB_kg": .6, "identifiedStiffness_N_m": 64e6,
            "identifiedForceConstant_N_A": 24.4, "identifiedCurrentResponse_s": 50e-6,
            "acquiredTransferReal": float(p["cellModal"]["acquiredTransferReal"]),
            "acquiredTransferImag": float(p["cellModal"]["acquiredTransferImag"]),
        },
        "contract": "One deterministic development candidate, selected before any new qualification outcome. Frozen native graph/helpers/evidence and new seed/context recipe are bound separately. Calibration residuals remain in global identified parameters; the modal filter retains its own nominal identified B/Kf/cell/current/acquisition contract. No neural controller or mechanical optimization.",
    }
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(lock, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"status": lock["status"], "output": str(out), "sha256": sha(out),
                      "selectedScenario": args.selected_scenario, "selectedIdentity": chosen["identity"]}, indent=2))


if __name__ == "__main__":
    main()
