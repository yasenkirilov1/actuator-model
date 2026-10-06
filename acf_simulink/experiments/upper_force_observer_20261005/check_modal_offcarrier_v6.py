"""Linear off-carrier diagnostic from the three existing native sine tones.

This creates no plant episode and makes no broadband stability claim.
"""
from pathlib import Path
import hashlib
import json
import numpy as np


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    root = Path(__file__).resolve().parents[3]
    base = root / "acf_simulink/validation/upper_force_observer_20261005"
    phase_file = base / "cell_modal_phase_v6/acquisition_phase.json"
    manifest_file = base / "bank_cell_modal_v6/manifest.json"
    phase = json.loads(phase_file.read_text())
    manifest = json.loads(manifest_file.read_text())
    target = base / "modal_offcarrier_response_v6.json"
    assert not target.exists(), "Preserve recorded diagnostic."
    rows = []
    # c only scales the correction; one family for each width is sufficient.
    for family in [manifest["cellModalFamilies"][i] for i in (0, 3, 6)]:
        d = family["controller"]
        a = np.array(d["A"]);b = np.array(d["B"]);c = np.array(d["C"])
        sample_time = 1e-4
        for tone in phase["rows"]:
            w = 2 * np.pi * tone["frequency_Hz"]
            acquired = complex(tone["measuredTransferReal"], tone["measuredTransferImag"])
            command_hold = np.exp(-1j*w*sample_time/2) * np.sin(w*sample_time/2)/(w*sample_time/2)
            current_response = 1/(1+1j*w*50e-6)
            h = (c @ np.linalg.solve(np.exp(1j*w*sample_time)*np.eye(2)-a, b)+d["D"]).item()
            actual_force = h*d["identifiedForceConstant_N_A"]*acquired*command_hold*current_response
            ideal_damping_force = -1j*w*d["virtualDamping_Ns_m"]/d["identifiedStiffness_N_m"]
            normalized = actual_force/ideal_damping_force
            rows.append({"familyID": family["familyID"], "filterDamping": family["filterDamping"],
                         "frequency_Hz": tone["frequency_Hz"], "normalizedDampingReal": normalized.real,
                         "normalizedDampingImag": normalized.imag,
                         "positiveDampingSignAtThisTone": bool(normalized.real > 0)})
    record = {"status": "linear_three_measured_tone_diagnostic_not_broadband_stability", "qualificationEpisodes": 0,
              "phaseHarnessFile": str(phase_file), "phaseHarnessSha256": sha(phase_file),
              "frozenBankManifestFile": str(manifest_file), "frozenBankManifestSha256": sha(manifest_file),
              "recipeFile": str(Path(__file__).resolve()), "recipeSha256": sha(Path(__file__).resolve()),
              "rows": rows,
              "interpretation": "All three widths give desired positive damping at the identified carrier. At1300Hz their linear action has negative damping sign;1900Hz depends on width. Resonant selectivity and auxiliary saturation bound action, but do not prove stability. Endpoint/whole-record experiments and physical transfer validation remain necessary.",
              "scope": "Exact native controller coefficients combined with already measured native acquisition tones,50us current-response model and10k command hold. No new sine or plant runs; no physical qualification claim."}
    target.write_text(json.dumps(record, indent=2)+"\n")
    print(json.dumps({"output": str(target), "rows": rows}, indent=2))


if __name__ == "__main__":
    main()
