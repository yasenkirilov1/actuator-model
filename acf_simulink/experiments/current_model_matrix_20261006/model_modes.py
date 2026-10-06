"""Flat-contact mechanical eigenfrequencies, independent of controller gains.

This is a small-signal mechanical calculation, not measured modes, not a
closed-loop pole analysis, and not an additional time-domain recording.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / "acf_simulink/validation/current_model_matrix_20261006/report"


def main():
    path = HERE / "initial_banks.json"
    banks = json.loads(path.read_text())["banks"]
    rows = []
    for bank in banks:
        p = bank["config"]
        for mass_s in [.1, 1.4, 2.]:
            screw = p["actuator"] != 0
            rigid = p["pad_k"] == 0
            if screw:
                # Coordinates A, B, S, q, with q=A-nut as in the contract.
                ell = p["lead"] / (2*np.pi)
                mass = np.diag([p["mass_a"]+p["nut_mass"], p["mass_b"], mass_s,
                                p["nut_mass"]+p["rotor_inertia"] / ell**2])
                mass[0, 3] = mass[3, 0] = -p["nut_mass"]
                stiffness = np.zeros((4, 4))
                sea = np.array([-1., 1., 0., 1.])
                stiffness += p["sea_k"] * np.outer(sea, sea)
            else:
                mass = np.diag([p["mass_a"], p["mass_b"], mass_s])
                stiffness = np.zeros((3, 3))
            stiffness[0, 0] += p["upper_k"]
            cell = np.zeros(len(mass)); cell[1], cell[2] = -1, 1
            stiffness += p["cell_k"] * np.outer(cell, cell)
            stiffness[2, 2] += p["pad_k"]
            if rigid:
                keep = [i for i in range(len(mass)) if i != 2]
                mass = mass[np.ix_(keep, keep)]
                stiffness = stiffness[np.ix_(keep, keep)]
            lower = np.linalg.cholesky(mass)
            left = np.linalg.solve(lower, stiffness)
            symmetric = np.linalg.solve(lower, left.T).T
            assert np.allclose(symmetric, symmetric.T)
            values, vectors = np.linalg.eigh(symmetric)
            physical_vectors = np.linalg.solve(lower.T, vectors)
            residual = stiffness @ physical_vectors - (mass @ physical_vectors) * values
            relative_residual = np.linalg.norm(residual) / np.linalg.norm(stiffness @ physical_vectors)
            assert relative_residual < 1e-10
            assert np.all(values > 0)
            rows.append({"bank": bank["id"], "mass_s_kg": mass_s,
                         "frequency_Hz": (np.sqrt(values)/(2*np.pi)).tolist(),
                         "relative_eigen_residual": float(relative_residual)})
    OUT.mkdir(exist_ok=True)
    result = {"status": "computed_flat_contact_mechanical_modes",
              "initial_banks_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "scope": "Ideal linearized mechanics at flat loaded contact. Excludes controller/current dynamics, damping, spindle and angular motion. Rigid S is imposed; its mass affects force but not the remaining free mechanical modes.",
              "records": rows}
    (OUT / "mechanical_modes.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({"cases": len(rows), "time_domain_recordings": 0}))


if __name__ == "__main__":
    main()
