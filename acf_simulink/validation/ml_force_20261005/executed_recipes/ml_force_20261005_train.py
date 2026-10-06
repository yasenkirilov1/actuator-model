#!/usr/bin/env python3
"""Train a 10-8-1 ReLU sensor-lag current residual from native trajectories.

Numpy Adam implementation avoids any opaque inference dependency. Whole native
recordings define training and selection; untouched test recordings are not read.
"""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import numpy as np
from scipy.io import loadmat, savemat

ROOT = Path(__file__).resolve().parents[2]
STUDY = ROOT / "acf_simulink/validation/ml_force_20261005"
SEED = 20261005


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path: Path) -> dict:
    f = loadmat(path, simplify_cells=True)["features"]
    p, d = f["p"], f["data"]
    y = p["inertial"]["forceKp"] * (d[:, 13] - d[:, 11])
    y -= (p["uncertainty"]["massB_model_kg"] + p["sensor"]["distalMass_model_kg"]) / p["uncertainty"]["forceConstant_model_N_A"] * (d[:, 12] - d[:, 14])
    y = np.clip(y, -.05, .05)
    t = f["time"]
    # Every dynamic sample, plus a regular subsample of quiet samples. This is
    # sampling for training efficiency, never the qualification metric.
    mask = ((t <= .05) | ((t >= 1.98) & (t <= 2.20)) | (np.arange(len(t)) % 20 == 0))
    return {"x": d[:, :10], "y": y, "mask": mask, "group": f["caseIdentity"]["group"],
            "file": str(path), "sha256": sha(path), "nativeFrames": len(t),
            "selectedTrainingFrames": int(mask.sum())}


def predict(x, w1, b1, w2, b2):
    return np.maximum(0, x @ w1.T + b1) @ w2 + b2


def main():
    assert not (STUDY / "trained_weights.mat").exists(), "Preserve trained weights."
    files = sorted((STUDY / "collect").glob("case_*_features.mat"))
    records = [read(path) for path in files]
    train = [r for r in records if r["group"] == "train"]
    val = [r for r in records if r["group"] == "selection"]
    assert len(train) == 8 and len(val) == 4
    x = np.concatenate([r["x"][r["mask"]] for r in train])
    y = np.concatenate([r["y"][r["mask"]] for r in train])
    vx = np.concatenate([r["x"][r["mask"]] for r in val])
    vy = np.concatenate([r["y"][r["mask"]] for r in val])
    mean, scale = x.mean(axis=0), np.maximum(x.std(axis=0), 1e-7)
    x, vx = np.clip((x-mean)/scale, -6, 6), np.clip((vx-mean)/scale, -6, 6)
    output_scale = .02
    y, vy = y/output_scale, vy/output_scale
    rng = np.random.default_rng(SEED)
    w1 = rng.normal(0, np.sqrt(2/10), (8, 10)); b1 = np.zeros(8)
    w2 = rng.normal(0, .1, 8); b2 = np.array(0.)
    params = [w1, b1, w2, b2]
    moments = [np.zeros_like(p) for p in params]
    variances = [np.zeros_like(p) for p in params]
    lr, reg, batch_size, epochs = .004, 1e-4, 2048, 180
    best_loss, best, step, history = np.inf, None, 0, []
    for epoch in range(epochs):
        order = rng.permutation(len(x))
        for start in range(0, len(x), batch_size):
            ids = order[start:start+batch_size]; bx, by = x[ids], y[ids]
            z = bx @ w1.T + b1; h = np.maximum(0, z); yp = h @ w2 + b2
            dp = 2*(yp-by)/len(ids)
            dh = dp[:, None]*w2[None, :]*(z > 0)
            grads = [dh.T @ bx + 2*reg*w1, dh.sum(axis=0), h.T @ dp + 2*reg*w2, np.array(dp.sum())]
            step += 1
            for p, m, v, g in zip(params, moments, variances, grads):
                m *= .9; m += .1*g; v *= .999; v += .001*g*g
                p -= lr*(m/(1-.9**step))/(np.sqrt(v/(1-.999**step))+1e-8)
        if epoch % 5 == 0 or epoch == epochs-1:
            ty = predict(x, *params); vpred = predict(vx, *params)
            tl, vl = np.mean((ty-y)**2), np.mean((vpred-vy)**2)
            history.append({"epoch": epoch+1, "trainMse_A2": float(tl*output_scale**2), "selectionMse_A2": float(vl*output_scale**2)})
            if vl < best_loss:
                best_loss, best = vl, [p.copy() for p in params]
            print(f"ML_FORCE_TRAIN epoch{epoch+1} trainRms{np.sqrt(tl)*output_scale:.6g}A selectionRms{np.sqrt(vl)*output_scale:.6g}A", flush=True)
        if epoch in [79, 129]: lr *= .5
    w1, b1, w2, b2 = best
    weights = {"W1": w1, "b1": b1[:, None], "W2": (w2*output_scale)[None, :], "b2": float(b2*output_scale),
               "mean": mean[:, None], "scale": scale[:, None], "gain": 0., "actionLimit_A": .05}
    savemat(STUDY / "trained_weights.mat", weights, do_compression=False)
    json_weights = {k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in weights.items()}
    (STUDY / "trained_weights.json").write_text(json.dumps(json_weights, indent=2)+"\n")
    assessment = []
    for r in records:
        xx = np.clip((r["x"]-mean)/scale, -6, 6)
        pred = np.clip(predict(xx, w1, b1, w2, b2)*output_scale, -.05, .05)
        assessment.append({"file": r["file"], "group": r["group"], "sha256": r["sha256"],
                           "frames": r["nativeFrames"], "trainingSubsetFrames": r["selectedTrainingFrames"],
                           "targetRms_A": float(np.sqrt(np.mean(r["y"]**2))),
                           "predictionResidualRms_A": float(np.sqrt(np.mean((pred-r["y"])**2))),
                           "predictionPeakAbs_A": float(np.max(np.abs(pred)))})
    report = {"status": "trained_not_closed_loop_qualified", "date": "2026-10-05", "seed": SEED,
              "architecture": "10-8-1 ReLU, fixed normalization, feature clipping +/-6, current clipping +/-0.05 A",
              "parameterCount": 97, "epochs": epochs, "batchSize": batch_size, "optimizer": "Adam",
              "objective": "MSE imitation of approximate PI/total-mass inertia sensor-lag correction, not optimal-action or reward optimization",
              "target": "clip(Kp*(N_est-N_true)-(B_id+S_id)/Kf_id*(aS_true-aS_ff_pred), +/-0.05) A; omits a changed lower-link inverse response",
              "trajectorySplit": {"train": len(train), "selection": len(val), "finalTestUsedForTraining": 0},
              "trainingNativeFrames": sum(r["nativeFrames"] for r in train), "trainingFramesUsed": len(x),
              "selectionNativeFrames": sum(r["nativeFrames"] for r in val), "selectionFramesUsed": len(vx),
              "bestSelectionMse_A2": float(best_loss*output_scale**2), "history": history, "recordings": assessment,
              "weightsSha256": sha(STUDY / "trained_weights.mat"), "recipeSha256": sha(Path(__file__))}
    (STUDY / "training.json").write_text(json.dumps(report, indent=2)+"\n")


if __name__ == "__main__":
    main()
