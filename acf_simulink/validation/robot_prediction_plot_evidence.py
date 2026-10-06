"""Render native experiment traces. Run with the project analysis Python env."""
from pathlib import Path
import json
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = Path(__file__).resolve().parent
WORKER = Path("/home/tommie/.local/state/actuator-model/qualification-20260930/robot_predictor_worker")
SOURCES = {
    "ideal": WORKER / "phase_tool_jerk_ideal/traces/tool_jerk_ideal_012.mat",
    "noise": WORKER / "phase_tool_jerk_noise/traces/tool_jerk_noise_001.mat",
    "timing": WORKER / "phase_timing_bounded/traces/timing_bounded_018.mat",
    "current": WORKER / "phase_tool_jerk_current/traces/tool_jerk_current_007.mat",
}
traces = {k: loadmat(v, simplify_cells=True)["r"] for k, v in SOURCES.items()}
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .2,
                     "savefig.facecolor": "white", "axes.spines.top": False,
                     "axes.spines.right": False})
colors = {"ideal": "#126b9a", "noise": "#c64c36"}
fig, ax = plt.subplots(4, 1, figsize=(10, 11), sharex=True, layout="constrained")
for name in ("ideal", "noise"):
    r = traces[name]; t = r["time"]; data = r["data"]
    mask = (t >= 1.99) & (t <= 2.125)
    label = "Zero physical sensor noise" if name == "ideal" else "1 mN force / 50 nm position RMS noise"
    ax[0].plot(t[mask], data[mask, 0] - 50, color=colors[name], lw=1.1, label=label)
    pred = r["robotPrediction"]; te = pred["estimatedTime"]
    emask = (te >= 1.99) & (te <= 2.125)
    ax[1].plot(te[emask], pred["estimatedStates"][emask, 7], color=colors[name], lw=1.1, label=label)
    ax[2].plot(t[mask], data[mask, 13], color=colors[name], lw=1.1, label=label)
    ax[3].plot(t[mask], data[mask, 14], color=colors[name], lw=1, label=label)
for a in ax:
    a.axvspan(2, 2.1, color="#b3cbd8", alpha=.14)
    a.axvline(2.1, color="#666", lw=.7, ls=":")
for target in (-.5, .5):
    ax[0].axhline(target, color="#222", ls="--", lw=1)
r = traces["ideal"]; mask = (r["time"] >= 1.99) & (r["time"] <= 2.125)
ax[1].plot(r["time"][mask], r["data"][mask, 29], color="#222", lw=1.3, ls="--", label="Actual surface acceleration (validation only)")
ax[0].set_ylabel("True contact force error (N)")
ax[1].set_ylabel("Surface acceleration (m/s²)")
ax[2].set_ylabel("Actual winding current (A)")
ax[3].set_ylabel("Winding voltage (V)"); ax[3].set_xlabel("Time (s)")
ax[3].axhline(24, color="#222", lw=1, ls="--", label="24 V drive limit")
ax[0].legend(loc="lower left"); ax[1].legend(loc="lower left", fontsize=8)
ax[3].legend(loc="lower right")
fig.suptitle("Upper-rigid / lower-compliant: causal surface-jerk observer\n"
             "Upstream B–S load cell, 1.4 kg distal mass, tool S relative to A position sensing", fontsize=12)
fig.supxlabel("Native Simulink sharp bump: 1 mm × 10 mm at 100 mm/s, 50 N, 90°.\n"
               "10 kHz; known 0.1 ms force and position delay; ideal run uses Qsnap=10⁸, noisy run 10⁷.\n"
               "Robot prediction is calibrated exactly in these two cases. Zero-noise pass is not noisy qualification.", fontsize=9)
for ext in ("png", "svg"):
    fig.savefig(HERE / f"robot_prediction_upstream_jerk_diagnostics.{ext}", dpi=180)
plt.close(fig)

fig, ax = plt.subplots(2, 2, figsize=(11, 6.8), sharex=True, layout="constrained")
for column, name in enumerate(("ideal", "timing")):
    r=traces[name]; t=r["time"]; data=r["data"]; pred=r["robotPrediction"]
    mask=(t>=.9)&(t<=3.5)
    ax[0,column].plot(t[mask], data[mask,26]*1000, color="#126b9a", lw=1.4, label="Actual robot velocity")
    ax[0,column].plot(t[mask], pred["data"][mask,1]*1000, color="#c64c36", ls="--", lw=1.2, label="Command-based prediction")
    ax[1,column].plot(t[mask], pred["actualAcceleration"][mask], color="#126b9a", lw=1.4)
    ax[1,column].plot(t[mask], pred["data"][mask,2], color="#c64c36", ls="--", lw=1.2)
    ax[0,column].set_title("Actual / predicted delay 80 / 80 ms; response 60 / 60 ms" if name=="ideal" else "Actual / predicted delay 200 / 80 ms; response 120 / 60 ms", fontsize=9)
    ax[0,column].legend(fontsize=8); ax[1,column].set_xlabel("Time (s)")
ax[0,0].set_ylabel("Robot normal velocity (mm/s)"); ax[1,0].set_ylabel("Robot normal acceleration (m/s²)")
fig.suptitle("Expected robot motion is a causal prediction, not a motion measurement", fontsize=12)
fig.supxlabel("Native physical motion shown only for validation; predictor receives issued velocity commands.\n"
              "Right panel is the earlier near-contact comparison plant: it demonstrates timing error, not upstream force qualification.", fontsize=9)
for ext in ("png", "svg"):
    fig.savefig(HERE / f"robot_prediction_timing_comparison.{ext}", dpi=180)
plt.close(fig)
(HERE / "robot_prediction_figures_sources.json").write_text(json.dumps({k:str(v) for k,v in SOURCES.items()}, indent=2)+"\n")

r=traces["current"]; t=r["time"]; data=r["data"]; pr=r["robotPrediction"]
te=pr["estimatedTime"]; states=pr["estimatedStates"]; ix=np.rint(te/r["parameters"]["sim"]["step"]).astype(int)
mask=(t>=1.99)&(t<=2.125); emask=(te>=1.99)&(te<=2.125)
fig,ax=plt.subplots(3,1,figsize=(10,8.8),sharex=True,layout="constrained")
ax[0].plot(t[mask],data[mask,0]-50,color="#c64c36",label="Actual contact-force error")
for target in (-.5,.5): ax[0].axhline(target,color="#222",ls="--",lw=1)
ax[1].plot(t[mask],data[mask,29],color="#222",ls="--",lw=1.3,label="Actual surface acceleration (validation only)")
ax[1].plot(te[emask],states[emask,7],color="#126b9a",label="Causal observer estimate")
currentError=states[:,6]+r["parameters"]["d"]["coilHoldCurrent"]-data[ix,13]
ax[2].plot(te[emask],currentError[emask]*1000,color="#126b9a",label="Estimated current minus actual current")
for a in ax:
    a.axvspan(2,2.1,color="#b3cbd8",alpha=.14)
    a.axvline(2.1,color="#666",lw=.7,ls=":")
    a.legend(loc="lower left",fontsize=8)
ax[0].set_ylabel("True contact force error (N)")
ax[1].set_ylabel("Surface acceleration (m/s²)")
ax[2].set_ylabel("Current-estimate error (mA)");ax[2].set_xlabel("Time (s)")
fig.suptitle("Optional current readback: best of 12 compliant-link tests still fails\n"
             "Actual peak contact-force error 0.805 N; requirement ±0.5 N",fontsize=12)
fig.supxlabel("Upstream B–S load cell / S−A position / drive-current readback, RMS noise 20 mN / 100 nm / 1 mA.\n"
              "10 kHz, 100 µs known measurement delays, 100 µs current response; matched command-based robot prediction.\n"
              "This is a tested observer limitation, not proof that current sensing cannot help another controller.",fontsize=9)
for ext in ("png","svg"):
    fig.savefig(HERE/f"robot_prediction_current_readback_negative.{ext}",dpi=180)
plt.close(fig)
