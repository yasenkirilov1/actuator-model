"""SVG report figures from executed, locked unified-policy evaluations.

Force metrics come from full native histories in the evaluation records.
Displayed waveforms use min/max bins, retaining excursions rather than
sampling one convenient point from each bin.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

BLUE = "#1768a6"
ORANGE = "#c35c13"
RED = "#b72c40"
GRAY = "#596475"
plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 10,
    "axes.spines.top": False, "axes.spines.right": False,
    "axes.labelcolor": "#26374b", "text.color": "#26374b",
    "axes.titleweight": "bold", "axes.titlesize": 12,
    "svg.fonttype": "none", "savefig.facecolor": "white",
    "figure.facecolor": "white", "axes.grid": True,
    "grid.alpha": .17, "grid.linewidth": .6,
})


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def band(ax, limit=.5):
    ax.axhspan(-limit, limit, color=BLUE, alpha=.055, zorder=0)
    for value in (-limit, limit):
        ax.axhline(value, color=RED, linewidth=.9, linestyle="--")


def envelope(ax, t, y, color, label, maximum_bins=1800):
    size=max(1, int(np.ceil(len(t)/maximum_bins)))
    start=np.arange(0,len(t),size)
    lo=np.minimum.reduceat(y,start);hi=np.maximum.reduceat(y,start)
    stop=np.minimum(start+size-1,len(t)-1)
    centers=(t[start]+t[stop])/2
    ax.fill_between(centers,lo,hi,color=color,alpha=.24,linewidth=0)
    ax.plot(centers,(lo+hi)/2,color=color,linewidth=.85,label=label)


def native(row):
    assert sha(row["nativeHistoryFile"]) == row["nativeHistorySha256"]
    with np.load(row["nativeHistoryFile"]) as source:
        return source["time"], source["truth"]


def case_title(row):
    return (f"S = {row['massS_kg']:g} kg  ·  {row['force_N']:g} N  ·  "
            f"{np.degrees(row['angle_rad']):g}°")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--final",type=Path,required=True)
    parser.add_argument("--grid",type=Path)
    parser.add_argument("--ppo",type=Path,action="append",default=[])
    parser.add_argument("--es",type=Path,action="append",default=[])
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    final=read(args.final)
    assert final["status"]=="complete" and final["lockedInputsUnchanged"]
    baseline=final["outcomes"]["baseline"]["rows"]
    selected=final["outcomes"]["selected"]["rows"]
    assert len(baseline)==len(selected)>0
    assert all(a["identity"]==b["identity"] and a["seed"]==b["seed"]
               for a,b in zip(baseline,selected,strict=True))
    assert all(r["fullNativeHistory"] for r in baseline+selected)
    args.output.mkdir(parents=True,exist_ok=True)
    figures=[]

    def save(fig,stem,description,source_paths):
        fig.savefig(args.output/(stem+".svg"),bbox_inches="tight")
        fig.savefig(args.output/(stem+".png"),bbox_inches="tight",dpi=125)
        plt.close(fig)
        figures.append({"file":stem+".svg","sha256":sha(args.output/(stem+".svg")),
                        "description":description,
                        "sourceHashes":{str(p):sha(p) for p in source_paths}})

    # Pairing uses the same physical case and noise seed for both controllers.
    x=np.array([r["peakNormalForceError_N"] for r in baseline])
    y=np.array([r["peakNormalForceError_N"] for r in selected])
    fig,axes=plt.subplots(1,2,figsize=(11.5,4.4),layout="constrained")
    lim=max(.52,float(max(x.max(),y.max()))*1.08)
    axes[0].plot([0,lim],[0,lim],color=GRAY,lw=1,ls=":",label="Equal peak error")
    points=axes[0].scatter(x,y,c=[r["massS_kg"] for r in selected],
                           cmap="viridis",s=42,edgecolor="white",linewidth=.5,zorder=3)
    axes[0].axhline(.5,color=RED,lw=1,ls="--");axes[0].axvline(.5,color=RED,lw=1,ls="--")
    axes[0].set(xlim=(0,lim),ylim=(0,lim),xlabel="PI + feedforward peak error (N)",
                ylabel="Unified learned policy peak error (N)",title="Fresh paired force tests")
    fig.colorbar(points,ax=axes[0],label="Tool mass S (kg)",shrink=.85)
    order=np.argsort(y-x)
    delta=y-x
    axes[1].bar(np.arange(len(delta)),delta[order],
                color=np.where(delta[order]<0,BLUE,ORANGE),width=.78)
    axes[1].axhline(0,color=GRAY,lw=.8)
    axes[1].set(xlabel="Paired cases, ordered by change",ylabel="Learned − baseline peak (N)",
                title=f"{int(np.sum(delta<0))}/{len(delta)} paired peaks improve")
    axes[1].set_xticks([])
    save(fig,"rl_3h_fresh_pairs","All-native force peaks in paired fresh cases.",[args.final])

    worst_index=int(np.argmax(y));worst=selected[worst_index]
    tb,db=native(baseline[worst_index]);ts,ds=native(worst)
    eb=db[:,0]-worst["force_N"];es=ds[:,0]-worst["force_N"]
    peak_time=float(ts[np.argmax(np.abs(es))])
    fig,axes=plt.subplots(2,1,figsize=(11.5,6.4),layout="constrained")
    for ax in axes:band(ax)
    envelope(axes[0],tb,eb,BLUE,"PI + feedforward")
    envelope(axes[0],ts,es,ORANGE,"Unified learned policy")
    axes[0].set(title="Worst fresh learned-policy recording — "+case_title(worst),
                xlabel="Time (s)",ylabel="Normal-force error (N)")
    axes[0].legend(loc="upper right")
    window=(max(0,peak_time-.08),min(float(ts[-1]),peak_time+.08))
    for t,e,color,label in [(tb,eb,BLUE,"PI + feedforward"),(ts,es,ORANGE,"Unified learned policy")]:
        mask=(t>=window[0])&(t<=window[1]);envelope(axes[1],t[mask],e[mask],color,label,maximum_bins=2200)
    axes[1].scatter([peak_time],[es[np.argmax(np.abs(es))]],s=30,color=ORANGE,zorder=4)
    axes[1].set(xlabel="Time (s)",ylabel="Normal-force error (N)",title=f"Peak region · learned maximum {y[worst_index]:.4f} N")
    save(fig,"rl_3h_worst_force","Complete history plus a peak-region view; extrema-preserving display bins.",[args.final])

    fig,axes=plt.subplots(2,1,figsize=(11.5,5.8),sharex=True,layout="constrained")
    for t,d,color,label in [(tb,db,BLUE,"PI + feedforward"),(ts,ds,ORANGE,"Unified learned policy")]:
        envelope(axes[0],t,d[:,14],color,label)
        envelope(axes[1],t,d[:,15],color,label)
    for v in (-3.5,3.5):axes[0].axhline(v,color=RED,ls="--",lw=.8)
    for v in (-32,32):axes[1].axhline(v,color=RED,ls="--",lw=.8)
    axes[0].set(ylabel="Coil current (A)",title="Electrical effort in the same worst force recording")
    axes[0].legend(loc="best")
    axes[1].set(ylabel="Bipolar coil voltage (V)",xlabel="Time (s)")
    save(fig,"rl_3h_drive","Current and voltage at unchanged 3.5 A / 32 V usable limits.",[args.final])

    # Choose the largest stroke usage, not a cosmetically quiet robot example.
    robot_index=max(range(len(selected)),key=lambda i:selected[i]["maxAbsTravelError_m"])
    robot=selected[robot_index]
    fig,axes=plt.subplots(2,1,figsize=(11.5,6.2),sharex=True,layout="constrained")
    axes[0].axhspan(-7.5,7.5,color=BLUE,alpha=.08,label="Accepted ±7.5 mm region")
    for value in (-12.5,12.5):axes[0].axhline(value,color=RED,ls="--",lw=.8)
    for row,color,label in [(baseline[robot_index],BLUE,"PI + feedforward"),(robot,ORANGE,"Unified learned policy")]:
        t,d=native(row);envelope(axes[0],t,(d[:,2]-.0125)*1000,color,label)
        envelope(axes[1],t,d[:,8]*1000,color,label)
    axes[0].set(ylabel="Travel from center (mm)",title="Largest fresh travel excursion — "+case_title(robot))
    axes[0].legend(loc="best",ncol=2)
    axes[1].set(ylabel="Actual robot-side velocity (mm/s)",xlabel="Time (s)")
    save(fig,"rl_3h_robot_travel","Full histories; centering inside the shaded band is optional.",[args.final])

    if args.grid:
        grid=read(args.grid);assert grid["status"]=="complete" and grid["lockedInputsUnchanged"]
        assert grid["contract"]["checkpointSha256"] == final["contract"]["checkpointSha256"]
        gr=grid["outcomes"]["selected"]["rows"]
        br=grid["outcomes"]["baseline"]["rows"]
        assert len(gr)==len(br)==700
        assert all(a["identity"]==b["identity"] and a["seed"]==b["seed"] for a,b in zip(gr,br,strict=True))
        identities={(r["massS_kg"],r["force_N"],r["angle_rad"]) for r in gr}
        assert len(identities)==700
        masses=sorted({r["massS_kg"] for r in grid["outcomes"]["selected"]["rows"]})
        forces=sorted({r["force_N"] for r in grid["outcomes"]["selected"]["rows"]})
        angles=sorted({r["angle_rad"] for r in gr})
        assert len(masses)==len(forces)==10 and len(angles)==7
        assert identities=={(m,f,a) for m in masses for f in forces for a in angles}
        matrices=[]
        for phase in ("baseline","selected"):
            rows=grid["outcomes"][phase]["rows"]
            matrices.append(np.array([[max(r["peakNormalForceError_N"] for r in rows
                    if r["massS_kg"]==mass and r["force_N"]==force) for force in forces] for mass in masses]))
        fig,axes=plt.subplots(1,2,figsize=(11.5,5.4),layout="constrained")
        high=max(.5,max(float(m.max()) for m in matrices))
        for ax,matrix,phase,title in zip(axes,matrices,("baseline","selected"),("PI + feedforward","Unified learned policy")):
            shown=ax.imshow(matrix,origin="lower",aspect="auto",cmap="viridis",vmin=0,vmax=high)
            ax.set_xticks(range(len(forces)),[f"{f:g}" for f in forces]);ax.set_yticks(range(len(masses)),[f"{m:g}" for m in masses])
            outcome=grid["outcomes"][phase]
            ax.set(title=f"{title}\n{outcome['passed']}/{outcome['recordings']} force passes",xlabel="Force target (N)",ylabel="Tool mass S (kg)")
        fig.colorbar(shown,ax=axes,label="Worst peak across seven fixed orientations (N)",shrink=.8)
        save(fig,"rl_3h_nominal_grid","Fresh nominal hot-coil grid; each heatmap cell takes the maximum across orientations.",[args.grid])

    if args.ppo or args.es:
        fig,axes=plt.subplots(1,2,figsize=(11.5,4.5),layout="constrained")
        for path in args.ppo:
            doc=read(path);rows=doc.get("selection",[])
            if rows:axes[0].plot([r["update"] for r in rows],[r["worstNormalForceError_N"] for r in rows],marker="o",ms=3,label=f"PPO {doc['seed']} ({len(doc['selectionIds'])} cases)")
        for path in args.es:
            rows=read(path)
            cohorts=sorted({r["recordings"] for r in rows})
            label=path.parent.name.removeprefix("reward_search_").replace("_"," ")+" ("+"/".join(map(str,cohorts))+" cases)"
            if rows:axes[1].plot([r["generation"] for r in rows],[r["worstNormalForceError_N"] for r in rows],marker="o",ms=3,label=label)
        for ax in axes:
            ax.axhline(.5,color=RED,lw=.9,ls="--",label="±0.5 N requirement")
            ax.set_ylabel("Worst development force error (N)")
            ax.legend(loc="best",fontsize=8)
        axes[0].set(xlabel="PPO updates",title="PPO checkpoints on each run's fixed selection set")
        axes[1].set(xlabel="Evolutionary generations",title="Reward-only weight search")
        save(fig,"rl_3h_training","Development selection only; changing cohorts are explicitly labeled.",args.ppo+args.es)

    manifest={"status":"complete","figures":figures,"worstCase":worst["identity"],
              "robotCase":robot["identity"],"nativeMetricDefinition":"Every native sample, startup and endpoint included.",
              "waveformDisplay":"Min/max bins; metric values are never computed from display bins."}
    (args.output/"figure_manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print(json.dumps({"figures":len(figures),"worstCase":worst["identity"],"robotCase":robot["identity"]}))


if __name__=="__main__":
    main()
