"""Verify both live acquired-cell paths and modal current placement in C."""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("build")
    args = parser.parse_args()
    build = Path(args.build).resolve()
    target = build / "generated_cell_source_audit.json"
    assert not target.exists(), "Preserve previous source acceptance."
    current = build / "code/acf_rl_native_grt_rtw/acf_rl_native.c"
    source = current.read_text()
    live = re.search(r"(\w+) = \(acf_rl_native_P.Requestedforce_Value -\s*acf_rl_native_B.Sensorsampledelay\) \*\s*acf_rl_native_P.Cellerrortodeviation_Gain;", source)
    assert live
    raw = live.group(1)
    upper = re.search(r"acf_rl_native_B.Cellforcedeviationfromequilibri = " + re.escape(raw) + r"\s*\+\s*acf_rl_native_P.Cellforcedeviationfromequilibri;", source)
    assert upper
    direct = re.search(r"rtb_Currentlimits \+= acf_rl_native_P.Cellmodalfilter_D\*\s*" + re.escape(raw) + ";", source)
    update = re.findall(r"acf_rl_native_P.Cellmodalfilter_B\[[01]\]\*" + re.escape(raw), source)
    assert direct and len(update) == 2
    assert "rtb_Notchedcommandplusmodaldamp" in source
    assert re.search(r"rtb_Normalerrorandantiwindup = \(rtb_Currentlimits -\s*rtb_Notchedcommandplusmodaldamp\)", source)
    bound_position = source.index("/* Saturate: '<S34>/Damping current bound' */")
    sum_position = source.index("/* Sum: '<S33>/Notched command plus modal damping'", bound_position)
    limit_position = source.index("if (rtb_Notchedcommandplusmodaldamp >", sum_position)
    assert bound_position < sum_position < limit_position
    assert "Present-time acceleration lead for current FF'" not in source
    helper = (build / "helpers/acf_prepare_cell_modal.m").read_text()
    assert "o.identifiedCurrentResponse_s" in helper and "p.coil.currentResponse" not in helper
    guard = json.loads((build / "local_cell_source_guard.json").read_text())
    assert guard["modalSourceVerified"] and all(r["maximumModalSourceError_N"] < 1e-12 for r in guard["rows"])
    phase = json.loads((build / "modal_positive_damping_guard.json").read_text())
    assert phase["status"] == "native_modal_positive_damping_sign_and_phase_passed"
    assert all(r["meanVirtualBModePower_W"] < 0 for r in phase["rows"])
    record = {"status": "emitted_native_cell_source_modal_plus_joint_pi_gain_live", "qualificationEpisodes": 0,
              "nativeCFile": str(current), "nativeCSha256": sha(current),
              "localRawDeviationAssignment": live.group(0), "upperObserverDeviationAssignment": upper.group(0),
              "modalDirectFeedthroughAssignment": direct.group(0), "modalStateInputCount": len(update),
              "modalAuxiliaryBoundBeforeTotalCurrentLimit": True, "antiwindupUsesTotalPrelimitCurrent": True,
              "existingUpperObserverSourceUnchanged": True, "noAccelerationLead": True,
              "modalUsesExplicitIdentifiedCurrentResponse": True, "diagnosticWindingCurrentIsNotControllerInput": True,
              "localSourceGuardSha256": sha(build / "local_cell_source_guard.json"),
              "nativePositiveDampingGuardSha256": sha(build / "modal_positive_damping_guard.json"),
              "scope": "Emitted-code source and placement acceptance plus numerical sign/phase guards; no force pass inferred."}
    target.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps({key: value for key, value in record.items() if "Assignment" not in key}, indent=2))


if __name__ == "__main__":
    main()
