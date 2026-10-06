"""Compile/seal the new qualification bank after saved-graph/source checks."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
from zipfile import ZipFile
import xml.etree.ElementTree as ET

from native_api import NativePlant


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def graphs(path: Path):
    result = {}
    with ZipFile(path) as archive:
        for name in archive.namelist():
            if not name.startswith("simulink/systems/") or not name.endswith(".xml"):
                continue
            root = ET.fromstring(archive.read(name))
            blocks = {}
            for block in root.findall("./Block"):
                # Layout and generated line-routing IDs are separate from the
                # native block type, parameter expression and connected edges.
                ignored = {"Position", "ZOrder", "PortCounts", "Location"}
                params = {p.attrib["Name"]: p.text or "" for p in block.findall("./P")
                          if p.attrib["Name"] not in ignored}
                blocks[block.attrib["SID"]] = (block.attrib["BlockType"], block.attrib["Name"], params)
            edges = []
            for line in root.findall("./Line"):
                source = next((p.text for p in line.findall("./P") if p.attrib["Name"] == "Src"), None)
                for item in line.iter("P"):
                    if item.attrib.get("Name") == "Dst":
                        edges.append((source, item.text))
            result[name] = (blocks, sorted(edges))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("build", type=Path)
    args = parser.parse_args()
    build = args.build.resolve()
    assert not (build / "file_hashes.json").exists(), "Preserve all native seals."
    metadata = json.loads((build / "metadata.json").read_text())
    bank = Path(metadata["configsFile"]).parent
    manifest = json.loads((bank / "manifest.json").read_text())
    lock = json.loads((bank / "candidate_lock.json").read_text())
    assert metadata["algebraicLoopCount"] == 0
    assert all(row["algebraicLoopCount"] == 0 for row in metadata["algebraicLoopChecks"])
    assert lock["selectionOnly"] and not lock["qualificationOutcomesInspected"]
    assert sha(bank / "configs.mat") == metadata["configsSha256"] == manifest["configsSha256"]
    assert sha(Path(manifest["sourceNativeModel"]["path"])) == manifest["sourceNativeModel"]["sha256"]
    assert manifest["allPreparedModalCoefficientsExactlySelected"]
    assert manifest["selectedNominalPreparedCoefficientsExact"]
    assert manifest["qualificationEpisodesExecuted"] == 0

    parent_graph = graphs(Path(manifest["sourceNativeModel"]["path"]))
    saved_graph = graphs(build / "acf_rl_native.slx")
    assert parent_graph == saved_graph, "Native blocks/parameters/edges differ from the selected source."
    scope = {"status": "saved_native_block_parameter_and_edge_graph_exactly_selected_source",
             "parentModel": manifest["sourceNativeModel"],
             "privateModelSha256": sha(build / "acf_rl_native.slx"),
             "systemCount": len(parent_graph), "allNativeBlockParametersExact": True,
             "allConnectedEdgesExact": True,
             "changedScope": "Model Workspace p, tunable coefficient/default bank, solver/code generation settings only.",
             "qualificationEpisodes": 0}
    (build / "saved_graph_scope.json").write_text(json.dumps(scope, indent=2) + "\n")

    current = build / "code/acf_rl_native_grt_rtw/acf_rl_native.c"
    source = current.read_text()
    live = re.search(r"(\w+) = \(acf_rl_native_P.Requestedforce_Value -\s*acf_rl_native_B.Sensorsampledelay\) \*\s*acf_rl_native_P.Cellerrortodeviation_Gain;", source)
    assert live
    variable = live.group(1)
    upper = re.search(r"acf_rl_native_B.Cellforcedeviationfromequilibri = " + re.escape(variable) + r"\s*\+\s*acf_rl_native_P.Cellforcedeviationfromequilibri;", source)
    direct = re.search(r"rtb_Currentlimits \+= acf_rl_native_P.Cellmodalfilter_D\*\s*" + re.escape(variable) + ";", source)
    state = re.findall(r"acf_rl_native_P.Cellmodalfilter_B\[[01]\]\*" + re.escape(variable), source)
    assert upper and direct and len(state) == 2
    assert re.search(r"rtb_Normalerrorandantiwindup = \(rtb_Currentlimits -\s*rtb_Notchedcommandplusmodaldamp\)", source)
    bound = re.search(r"/\* Saturate: '<S\d+>/Damping current bound' \*/", source)
    assert bound
    sum_block = source.index("/Notched command plus modal damping'", bound.start())
    total_limit = source.index("if (rtb_Notchedcommandplusmodaldamp >", sum_block)
    assert bound.start() < sum_block < total_limit
    source_guard = json.loads((build / "local_cell_source_guard.json").read_text())
    assert source_guard["status"] == "live_local_cell_and_modal_source_guard_passed"
    assert source_guard["plantQualificationEpisodes"] == 0
    assert all(row["upperSourceMaximumError_N"] < 1e-12 and row["modalSourceMaximumError_N"] < 1e-12
               for row in source_guard["rows"])
    mapping = json.loads((build / "parameter_mapping.json").read_text())
    assert sum(row["expression"] == "-p.inertial.forceKp" for row in mapping) == 1
    assert sum(row["expression"] == "-p.inertial.forceKi" for row in mapping) == 1
    phase = lock["sourceFiles"]["modal_positive_damping_guard.json"]
    assert sha(Path(phase["path"])) == phase["sha256"]
    audit = {"status": "emitted_cell_sources_live_selected_modal_and_executing_normal_PI_confirmed",
             "qualificationEpisodes": 0, "nativeCSha256": sha(current),
             "localRawDeviationAssignment": live.group(0), "upperDeviationAssignment": upper.group(0),
             "modalDirectFeedthroughAssignment": direct.group(0), "modalStateInputCount": len(state),
             "modalAuxiliaryBoundBeforeTotalCurrentLimit": True, "antiwindupUsesTotalPrelimitCurrent": True,
             "executingNormalForceGains": lock["executingNormalForceControllerGains"],
             "localSourceGuardSha256": sha(build / "local_cell_source_guard.json"),
             "parentPositiveDampingGuard": phase,
             "scope": "New emitted-source/graph check plus three source harnesses. Selected coefficients preserve the parent phase guard; no new carrier or force-qualification run inferred."}
    (build / "generated_cell_source_audit.json").write_text(json.dumps(audit, indent=2) + "\n")

    # NativePlant's unchanged bridge demands exact default/case0 Coder bytes.
    plant = NativePlant(build, rebuild=True, threads=1)
    assert len(plant.scenarios) == 736
    assert plant.metadata["step_s"] == 1 / 300000 and plant.metadata["actionStep_s"] == 1e-4
    counts = {name: sum(row["split"] == "qualification_" + name for row in plant.scenarios)
              for name in ("nominal700", "fresh32", "long4")}
    assert counts == {"nominal700": 700, "fresh32": 32, "long4": 4}
    assert all(row["massA_kg"] == row["identifiedMassA_kg"] == 1.5 for row in plant.scenarios)
    assert len({row["seed"] for row in plant.scenarios}) == 736
    recipe = Path(__file__).parent
    files = [build / name for name in (
        "acf_rl_native.slx", "metadata.json", "prepared_configs.mat", "scenarios.json",
        "native_parameter_bank.c", "parameter_mapping.json", "parameter_mapping.mat",
        "libacf_native.so", "bridge_build.json", "acf_rl_native_context.c", "native_bank_aux.h",
        "build_native.m", "export_native_parameter_bank_v8.m", "local_cell_source_guard.json",
        "saved_graph_scope.json", "generated_cell_source_audit.json")]
    files += [bank / name for name in ("configs.mat", "manifest.json", "candidate_lock.json",
                                      "context_recipe.json", "coefficient_guard.mat", "prepare_qualification_bank_v8.m")]
    files += sorted((build / "helpers").glob("*.m"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.c"))
    files += sorted((build / "code/acf_rl_native_grt_rtw").glob("*.h"))
    files += sorted(recipe.glob("*.m")) + sorted(recipe.glob("*.py")) + [recipe / "native_bridge.c"]
    record = {"status": "qualification_native_bank_frozen_not_executed", "qualificationEpisodes": 0,
              "candidateLockSha256": sha(bank / "candidate_lock.json"), "contextRecipeSha256": manifest["contextRecipeSha256"],
              "configsSha256": metadata["configsSha256"], "nativeBankCase0MatchesCoder": True,
              "sharedLibrarySha256": sha(build / "libacf_native.so"), "cohortCounts": counts,
              "caseCount": 736, "allA1_5kg": True, "modalCoefficientsFrozen": True,
              "sourceAuditSha256": sha(build / "generated_cell_source_audit.json"),
              "files": [{"file": str(path), "sha256": sha(path)} for path in files]}
    (build / "file_hashes.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({key: value for key, value in record.items() if key != "files"}, indent=2))


if __name__ == "__main__":
    main()
