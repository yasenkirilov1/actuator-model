#!/usr/bin/env python3
"""Audit paired native summaries and render standalone SVG experiment figures."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("collect", type=Path)
    parser.add_argument("training", type=Path)
    parser.add_argument("heldout", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    heldout = json.loads((args.heldout / "results.json").read_text())
    training = json.loads((args.training / "training_summary.json").read_text())
    collection = json.loads((args.collect / "results.json").read_text())
    assert heldout["status"] == collection["status"] == "complete"
    assert training["status"] == "trained_from_native_trajectories"
    rows = heldout["rows"]
    groups = {}
    for row in rows:
        groups.setdefault(row["identity"], {})[row["policy"]] = row
    assert all(set(g) == {"teacher", "learned"} for g in groups.values())
    assert len(rows) == len(groups)*2 and all(r["executed"] for r in rows)
    pairs = []
    for identity, group in groups.items():
        a, b = group["teacher"], group["learned"]
        # Simulation identity and environment must be paired exactly.
        for key in ["massS_kg","force_N","angle_deg","seed","offset_m","actualRobotDelay_s","actualRobotResponse_s"]:
            assert a[key] == b[key]
        pairs.append({"identity": identity, "split": a["split"], "teacher_peak_N": a["peakNormalForceError_N"],
                      "learned_peak_N": b["peakNormalForceError_N"],
                      "change_N": b["peakNormalForceError_N"]-a["peakNormalForceError_N"],
                      "teacher_rms_N": a["rmsNormalForceError_N"], "learned_rms_N": b["rmsNormalForceError_N"],
                      "teacher_final_travel_error_mm": a["finalTravelError_m"]*1000,
                      "learned_final_travel_error_mm": b["finalTravelError_m"]*1000,
                      "teacher_passed": a["passed"],"learned_passed": b["passed"],
                      "teacher_nonzero_command_frames": a["nonzeroCorrectionFrames"],
                      "learned_nonzero_command_frames": b["nonzeroCorrectionFrames"]})
    plt.rcParams.update({"font.family":"DejaVu Sans", "svg.fonttype":"none", "font.size":11,
                         "axes.spines.top":False,"axes.spines.right":False, "axes.labelcolor":"#26384c",
                         "text.color":"#26384c", "axes.edgecolor":"#c9d3df", "grid.color":"#e5ebf2"})
    blue, teal, red = "#3363a6", "#07948e", "#cf4a4a"
    x = np.arange(len(pairs))
    fig, ax = plt.subplots(figsize=(12,5.3))
    ax.bar(x-.18,[p["teacher_peak_N"] for p in pairs],.34,label="Deterministic recenter policy",color=blue)
    ax.bar(x+.18,[p["learned_peak_N"] for p in pairs],.34,label="Trained policy + hard gate and envelope",color=teal)
    ax.axhline(.5,color=red,lw=1.5,label="Required limit: 0.5 N")
    ax.axhline(.45,color="#9f7c39",lw=1,ls="--",label="Design margin screen: 0.45 N")
    ax.set_xticks(x,[p["identity"].replace("heldout_","H").replace("calibration_","C") for p in pairs])
    ax.set_ylabel("Maximum normal-force error (N)")
    ax.set_ylim(0,max(.65,max(p["learned_peak_N"] for p in pairs)*1.25))
    ax.set_title("A learned recenter policy reproduces the existing policy\nEight unseen recenter trajectories plus three calibration-stress pairs",loc="left",fontweight="bold",pad=18)
    ax.grid(axis="y",alpha=.6);ax.set_axisbelow(True);ax.legend(ncol=2,frameon=False,loc="upper left")
    fig.text(.09,.015,"The fast force controller is identical in each pair. Complete native histories include startup and the sharp bump.",fontsize=9)
    fig.tight_layout(rect=(0,.04,1,1));fig.savefig(args.output/"ml_robot_paired_results.svg");plt.close(fig)

    # Show a genuinely active unseen robot trajectory, including delayed
    # requests that survive a gate closure. Plot excerpts, not score inputs.
    candidates=[g for g in groups.values() if g["learned"]["gateClosures"]>0]
    group=max(candidates,key=lambda g:g["learned"]["peakIssuedCorrection_m_s"])
    teacher=loadmat(args.heldout/f"recording_{group['teacher']['recording']:03d}_compact.mat",simplify_cells=True)["compact"]
    learned=loadmat(args.heldout/f"recording_{group['learned']['recording']:03d}_compact.mat",simplify_cells=True)["compact"]
    fig, axes=plt.subplots(3,1,figsize=(12,8.5),sharex=True)
    for c,label,color in [(teacher,"Deterministic",blue),(learned,"Trained",teal)]:
        t=c["time"];a=c["data"];p=c["parameters"];tp=c["policyTime"];b=c["policyData"]
        axes[0].plot(t,a[:,14]-p["force"]["reference"],label=label,color=color,lw=1)
        axes[1].plot(t,(a[:,1]-p["mechanics"]["midTravel"])*1000,label=label,color=color,lw=1.6)
        axes[2].step(tp,b[:,5]*1000,where="post",label=f"{label}: issued correction",color=color,lw=1.3)
    t=learned["time"];a=learned["data"];tp=learned["policyTime"];b=learned["policyData"]
    axes[2].plot(t,a[:,22]*1000,label="Trained: total issued request",color="#805db0",lw=1.3)
    axes[2].plot(t,a[:,21]*1000,label="Delayed total request reaching robot",color="#cc8929",lw=1.3,ls="--")
    axes[0].axhline(.5,color=red,lw=1,ls="--");axes[0].axhline(-.5,color=red,lw=1,ls="--")
    axes[1].axhline(.5,color="#b4becb",lw=1,ls="--");axes[1].axhline(-.5,color="#b4becb",lw=1,ls="--")
    for ax in axes:
        ax.grid(alpha=.55);ax.set_axisbelow(True)
        ready=b[np.clip(np.searchsorted(tp,t,side="right")-1,0,len(tp)-1),4]
        ymin,ymax=ax.get_ylim();ax.fill_between(t,ymin,ymax,where=ready<.5,color="#f3d7dc",alpha=.45,step="pre",label="Gate closed" if ax is axes[2] else None)
        ax.set_ylim(ymin,ymax)
    axes[0].set_ylabel("Normal-force error (N)");axes[1].set_ylabel("Travel from centre (mm)")
    axes[2].set_ylabel("Robot velocity command (mm/s)");axes[2].set_xlabel("Time (s)")
    axes[2].legend(ncol=2,frameon=False,loc="best");axes[0].legend(frameon=False,loc="best")
    r=group['learned'];fig.suptitle(f"Active calibration-stress recentering: {r['massS_kg']:g} kg tool, {r['force_N']:g} N, {r['angle_deg']:g}°\nNew corrections stop at gate closure; the nominal path and queued requests remain",x=.075,ha="left",fontweight="bold",fontsize=14)
    fig.tight_layout(rect=(0,0,1,.93));fig.savefig(args.output/"ml_robot_active_trajectory.svg");plt.close(fig)

    # Native training examples and the learned transfer curve; no surface
    # truth is present in this data or inference.
    files=[args.collect/f"recording_{r['recording']:03d}_compact.mat" for r in collection['rows'] if r['split']=='train']
    xx=[];yy=[]
    for f in files:
        c=loadmat(f,simplify_cells=True)['compact'];b=c['policyData'];p=c['parameters']
        xx.extend(b[:,0]*1000);yy.extend(b[:,1]*1000)
    n=loadmat(args.training/'network_weights.mat',simplify_cells=True)['network'];curve=np.linspace(min(xx),max(xx),1200)
    hidden=np.maximum(np.asarray(n['W1'])[:,None]*(curve[None,:]/1000/n['inputScale_m'])+np.asarray(n['b1'])[:,None],0)
    output=(np.asarray(n['W2'])@hidden+n['b2']-n['zeroOutput'])*n['inputScale_m']*.7352941176470588*1000
    fig,ax=plt.subplots(figsize=(10,5.5));ax.scatter(xx,yy,s=5,alpha=.3,color=blue,label="Native logged teacher commands (before gate)")
    ax.plot(curve,output,color=teal,lw=2,label="Trained eight-ReLU policy (before safety envelope)")
    ax.set_xlabel("Causal estimated distance from mid stroke (mm)");ax.set_ylabel("Recenter correction (mm/s)")
    ax.set_title("A small supervised policy learns the recenter transfer\nEight training trajectories; four separate trajectories select initialization",loc="left",fontweight="bold",pad=18)
    ax.grid(alpha=.5);ax.legend(frameon=False);fig.tight_layout();fig.savefig(args.output/'ml_robot_learned_policy.svg');plt.close(fig)
    summary={"date":"2026-10-05","status":"paired_native_results_audited","learning":"supervised robot-policy imitation",
             "training_trajectories":training['training_trajectories'],"selection_trajectories":training['selection_trajectories'],
             "heldout_identities":len(groups),"heldout_recordings":len(rows),"heldout_native_samples":heldout['nativeSamples'],
             "collection_native_samples":collection['nativeSamples'],"trained_network_sha256":training['network_sha256'],
             "test_trajectories_used_in_training":0,"teacher_passes":sum(p['teacher_passed'] for p in pairs),
             "learned_passes":sum(p['learned_passed'] for p in pairs),
             "teacher_worst_error_N":max(p['teacher_peak_N'] for p in pairs),
             "learned_worst_error_N":max(p['learned_peak_N'] for p in pairs),
             "maximum_abs_paired_peak_change_N":max(abs(p['change_N']) for p in pairs),
             "gate_closed_command_violations":sum(r['gateClosedCommandViolations'] for r in rows),
             "direction_or_gain_envelope_violations":sum(r['directionEnvelopeViolations'] for r in rows),
             "maximum_network_inference_residual_m":max(r['networkInferenceResidual_m'] for r in rows),
             "maximum_raw_network_teacher_action_error_m_s":max(r['rawNetworkTeacherActionPeakError_m_s'] for r in rows if r['policy']=='learned'),
             "maximum_bounded_network_teacher_action_error_m_s":max(r['boundedNetworkTeacherActionPeakError_m_s'] for r in rows if r['policy']=='learned'),
             "raw_network_safety_clipped_frames":sum(r['networkEnvelopeClippedFrames'] for r in rows if r['policy']=='learned'),
             "paired_results":pairs,"active_plot_identity":r['identity'],
             "sources":[{"file":str(f),"sha256":sha(f)} for f in [args.heldout/'results.json',args.collect/'results.json',args.training/'training_summary.json']],
             "figures":[{"file":f.name,"sha256":sha(f)} for f in sorted(args.output.glob('*.svg'))],
             "recommendation":"Retain the transparent deterministic recenter policy. This learned policy imitates it and offers no demonstrated performance or complexity advantage.",
             "limitations":"Finite trajectory-level holdouts, partial calibration corners, and modeled delay errors. No full range requalification, online RL, or physical sanding validation."}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:summary[k] for k in ['heldout_identities','teacher_passes','learned_passes','teacher_worst_error_N','learned_worst_error_N','maximum_abs_paired_peak_change_N','gate_closed_command_violations']},indent=2))


if __name__ == '__main__':
    main()
