"""Verify live PI bank coefficients and exact full-native reference parity.

No controller or physical episode is executed. The independent guard reads
the actual emitted parameter-bank C, rather than trusting the recipe labels.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path: Path) -> list[dict[str, str]]:
    sections = re.findall(r" \{\n(.*?)\n \},\n", path.read_text(), flags=re.S)
    result = []
    for section in sections:
        fields = dict(re.findall(r"^  \.(\w+) = (.*),$", section, flags=re.M))
        assert len(fields) > 500
        result.append(fields)
    return result


def main() -> None:
    parser = argparse.ArgumentParser();parser.add_argument("build");args = parser.parse_args()
    build = Path(args.build).resolve();target = build / "native_reference_parameter_parity.json"
    assert not target.exists(), "Preserve previous acceptance."
    parent = build.parent / "native_cell_modal_v6"
    current_file = build / "native_parameter_bank.c";parent_file = parent / "native_parameter_bank.c"
    current = rows(current_file);baseline = rows(parent_file)
    assert len(current) == 264 and len(baseline) == 396
    metadata = json.loads((build / "metadata.json").read_text())
    manifest = json.loads((Path(metadata["configsFile"]).parent / "manifest.json").read_text())
    mapping = {x["name"]: x for x in json.loads((build / "parameter_mapping.json").read_text())}
    kp = "Normalforceproportionalcurrent_";ki = "Normalforceintegralcurrent_Gain";aw = "Integralantiwindup_Gain"
    assert mapping[kp]["expression"] == "-p.inertial.forceKp"
    assert mapping[ki]["expression"] == "-p.inertial.forceKi"
    assert mapping[aw]["expression"] == "-p.force.antiWindupRate/p.inertial.forceKi"
    allowed = {kp, ki, aw, "Cellmodalfilter_C", "Cellmodalfilter_D"}
    reference_rows = 0;records = [];maximum_aw_error = 0.0
    for index, (actual, case) in enumerate(zip(current, manifest["cases"], strict=True)):
        original = baseline[16*22 + index % 22]
        assert actual.keys() == original.keys()
        changed = {key for key in actual if actual[key] != original[key]}
        assert changed <= allowed, (index, changed-allowed)
        actual_kp = float(actual[kp]);actual_ki = float(actual[ki]);actual_aw = float(actual[aw])
        assert actual_kp == -case["executingForceKp_A_N"] and actual_ki == -case["executingForceKi_A_N_s"]
        assert actual_aw == -30/case["executingForceKi_A_N_s"]
        aw_error = abs(actual_ki*actual_aw-30);assert aw_error < 1e-12
        maximum_aw_error = max(maximum_aw_error, aw_error)
        if case["isExactV6ParentReference"]:
            assert not changed, (index, changed);reference_rows += 1
        records.append({"scenario": index, "familyID": case["gainID"], "changedNativeFields": sorted(changed),
                        "liveProportionalGain": actual_kp, "liveIntegralGain": actual_ki,
                        "liveAntiWindupGain": actual_aw, "effectiveCurrentBackCalculationRate_s_inv": actual_ki*actual_aw,
                        "exactReference": case["isExactV6ParentReference"]})
    assert reference_rows == 22
    assert len({r["liveProportionalGain"] for r in records}) == 4
    assert len({r["liveIntegralGain"] for r in records}) == 4
    record = {"status": "live_selected_normal_pi_coefficients_and_full_reference_parity_passed", "qualificationEpisodes": 0,
              "currentParameterBankFile": str(current_file), "currentParameterBankSha256": sha(current_file),
              "parentParameterBankFile": str(parent_file), "parentParameterBankSha256": sha(parent_file),
              "currentConfigsSha256": metadata["configsSha256"], "currentMappingSha256": sha(build / "parameter_mapping.json"),
              "exactReferenceRows": reference_rows, "allNativeFieldsExactlyEqual": True,
              "all264RowsOnlyAllowedLivePIAndModalFieldsDiffer": True, "uniqueExecutingProportionalGains": 4,
              "uniqueExecutingIntegralGains": 4, "maximumCurrentBackCalculationRateError_s_inv": maximum_aw_error,
              "allowedChangedNativeFields": sorted(allowed), "rows": records,
              "scope": "Independent actual emitted-bank coefficient inspection: selected normal P/I and matching antiwindup vary, all other native parameters exactly match the correct V6parent17 case, and family8 has22 complete native parameter matches. No force experiment or hardware claim."}
    target.write_text(json.dumps(record, indent=2)+"\n")
    print(json.dumps({k:v for k,v in record.items() if k != "rows"}, indent=2))


if __name__ == "__main__":
    main()
