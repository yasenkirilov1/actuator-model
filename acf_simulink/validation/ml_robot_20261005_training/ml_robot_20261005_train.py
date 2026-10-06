#!/usr/bin/env python3
"""Train an eight-ReLU causal robot policy on native teacher trajectories.

This is supervised imitation, not reinforcement learning. The only input is
the robot-rate causal estimate of distance from mid stroke. Gate, directional
gain envelope, delay-aware gain and physical limits remain outside learning.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.io import loadmat, savemat
from scipy.optimize import least_squares


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def forward(x: np.ndarray, theta: np.ndarray) -> np.ndarray:
    knots, coefficients = theta[:4], theta[4:]
    h = np.maximum(x[:, None] - knots, 0) - np.maximum(-x[:, None] - knots, 0)
    return h @ coefficients


def jacobian(x: np.ndarray, theta: np.ndarray) -> np.ndarray:
    knots, coefficients = theta[:4], theta[4:]
    h = np.maximum(x[:, None] - knots, 0) - np.maximum(-x[:, None] - knots, 0)
    dk = ((-1.0 * (x[:, None] > knots)) + 1.0 * (-x[:, None] > knots)) * coefficients
    return np.column_stack([dk, h])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = json.loads((args.input / "results.json").read_text())
    assert report["status"] == "complete"
    assert all(r["executed"] for r in report["rows"])
    native_sha = "b3701daabf1f7a9712511580fe20fde1f10b3d7d83b68b778909d1ff051da1d5"
    assert report["sourceModelSha256"] == native_sha
    datasets = {"train": [], "selection": []}
    sources = []
    scale = 0.002
    for row in report["rows"]:
        path = args.input / f"recording_{row['recording']:03d}_compact.mat"
        c = loadmat(path, simplify_cells=True)["compact"]
        b = np.asarray(c["policyData"])
        p = c["parameters"]
        gain = float(p["uncertainty"]["recenterGain"])
        row["effective_recenter_gain_s_1"] = gain
        x = b[:, 0] / scale
        y = b[:, 1] / (gain * scale)
        assert np.allclose(y, np.sign(x) * np.maximum(np.abs(x) - p["robot"]["deadband"] / scale, 0), atol=1e-11)
        datasets[row["split"]].append((x, y, b, row))
        sources.append({"file": str(path), "sha256": sha(path), "identity": row["identity"], "split": row["split"],
                        "native_samples_scored": row["nativeSamples"], "robot_frames": len(x),
                        "nonzero_teacher_pre_gate_frames": int(np.sum(np.abs(b[:, 1]) > 1e-10)),
                        "nonzero_issued_frames": int(np.sum(np.abs(b[:, 5]) > 1e-10))})
    assert len(datasets["train"]) == 8 and len(datasets["selection"]) == 4
    x = np.concatenate([d[0] for d in datasets["train"]])
    y = np.concatenate([d[1] for d in datasets["train"]])
    xv = np.concatenate([d[0] for d in datasets["selection"]])
    yv = np.concatenate([d[1] for d in datasets["selection"]])
    assert np.sum(np.abs(y) > 1e-10) > 100 and np.any(x < -.5) and np.any(x > .5)
    # Balance observed stroke-error bins to prevent the numerous nearly
    # recentered frames from overwhelming the active command examples.
    bins = np.linspace(float(x.min()), float(x.max()), 25)
    ids = np.clip(np.digitize(x, bins) - 1, 0, len(bins) - 2)
    counts = np.bincount(ids, minlength=len(bins) - 1)
    weights = np.sqrt(len(x) / (max(1, np.sum(counts > 0)) * counts[ids]))
    candidates = []
    fit_arrays = []
    for seed in [5, 29, 1729, 20261005]:
        rng = np.random.default_rng(seed)
        initial = np.r_[np.clip(np.array([.10, .30, .60, .95]) + rng.normal(0, .025, 4), .01, 1.1),
                        rng.normal(.25, .1, 4)]
        solved = least_squares(lambda t: weights * (forward(x, t) - y), initial,
                               jac=lambda t: weights[:, None] * jacobian(x, t),
                               bounds=(np.r_[np.zeros(4), -4*np.ones(4)], np.r_[2*np.ones(4), 4*np.ones(4)]),
                               max_nfev=2000, ftol=1e-13, xtol=1e-13, gtol=1e-13)
        pred = forward(x, solved.x)
        predv = forward(xv, solved.x)
        row = {"seed": seed, "converged": bool(solved.success), "message": solved.message,
               "function_evaluations": int(solved.nfev), "theta": solved.x.tolist(),
               "training_normalized_rmse": float(np.sqrt(np.mean((pred-y)**2))),
               "selection_normalized_rmse": float(np.sqrt(np.mean((predv-yv)**2))),
               "selection_max_distance_error_m": float(scale*np.max(np.abs(predv-yv)))}
        candidates.append(row)
        fit_arrays.append(solved.x)
    selected = min(range(len(candidates)), key=lambda i: candidates[i]["selection_normalized_rmse"])
    theta = fit_arrays[selected]
    knots, coefficients = theta[:4], theta[4:]
    network = {"enabled": 1.0, "inputScale_m": scale, "W1": np.r_[np.ones(4), -np.ones(4)][:, None],
               "b1": np.r_[-knots, -knots][:, None], "W2": np.r_[coefficients, -coefficients][None, :],
               "b2": 0.0, "zeroOutput": 0.0}
    savemat(args.output / "network_weights.mat", {"network": network}, do_compression=True)
    command_metrics = {}
    for split, data in datasets.items():
        raw_errors, bounded_errors, gate_errors, raw_all, bound_all = [], [], [], [], []
        for xx, yy, b, row in data:
            gain = row["effective_recenter_gain_s_1"]
            raw = forward(xx, theta) * scale * gain
            target = b[:, 1]
            bounded = np.maximum(np.minimum(raw, np.maximum(target, 0)), np.minimum(target, 0))
            issued = bounded * (b[:, 4] > .5)
            raw_errors.extend(raw - target)
            bounded_errors.extend(bounded - target)
            gate_errors.extend(issued - b[:, 5])
            raw_all.extend(raw);bound_all.extend(bounded)
        command_metrics[split] = {
            "raw_pre_gate_rmse_m_s": float(np.sqrt(np.mean(np.array(raw_errors)**2))),
            "raw_pre_gate_max_error_m_s": float(np.max(np.abs(raw_errors))),
            "bounded_pre_gate_rmse_m_s": float(np.sqrt(np.mean(np.array(bounded_errors)**2))),
            "bounded_pre_gate_max_error_m_s": float(np.max(np.abs(bounded_errors))),
            "issued_correction_rmse_m_s": float(np.sqrt(np.mean(np.array(gate_errors)**2))),
            "issued_correction_max_error_m_s": float(np.max(np.abs(gate_errors))),
            "safety_clipped_frames": int(np.sum(np.abs(np.array(raw_all)-np.array(bound_all)) > 1e-12))}
    summary = {"date": "2026-10-05", "status": "trained_from_native_trajectories", "learning": "supervised teacher imitation",
               "objective": "Weighted mean-square reproduction of logged pre-gate recenter correction; selection on untouched trajectories",
               "feature": "causal robot-rate estimated actuator distance from mid stroke, normalized by 2 mm",
               "features_excluded": ["future surface profile", "plant truth", "true normal force", "future commands"],
               "architecture": "1 input, 8 ReLU hidden units, 1 output; odd symmetric pairs",
               "learned_scalar_parameters": 8, "stored_affine_coefficients": 25,
               "hard_constraints": ["original slow force band/rate and dwell gate", "original 10 kHz variation and dwell gate",
                                    "same-signed action no larger than delay-bounded teacher action", "robot speed and acceleration limits"],
               "inference_updates_Hz": 100, "input_scale_m": scale,
               "training_trajectories": 8, "selection_trajectories": 4, "test_trajectories_used_in_training": 0,
               "training_frames": len(x), "selection_frames": len(xv), "training_input_range_m": [float(x.min()*scale), float(x.max()*scale)],
               "training_nonzero_target_frames": int(np.sum(np.abs(y)>1e-10)),
               "training_nonzero_issued_frames": sum(s["nonzero_issued_frames"] for s in sources if s["split"]=="train"),
               "initialization_seeds": [5,29,1729,20261005], "selected_initialization_seed": candidates[selected]["seed"],
               "candidates": candidates, "teacher_command_metrics": command_metrics, "sources": sources,
               "native_source_sha256": native_sha, "recipe_sha256": sha(Path(__file__)),
               "network_file": str(args.output / "network_weights.mat"), "network_sha256": sha(args.output / "network_weights.mat"),
               "limitations": "Imitation can reproduce the existing safe policy but does not optimize closed-loop force or recenter speed. Finite native trajectory data; no hardware qualification."}
    (args.output / "training_summary.json").write_text(json.dumps(summary, indent=2)+"\n")
    print(json.dumps({k: summary[k] for k in ["training_frames","training_nonzero_issued_frames","selected_initialization_seed","teacher_command_metrics"]}, indent=2))


if __name__ == "__main__":
    main()
