#!/usr/bin/env python3
"""Plot completed native winding qualification; require all actual cases."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.patches import Rectangle
folder=Path(__file__).resolve().parent
report=json.loads((folder/"actuator_qualification_final_winding.json").read_text())
assert report["status"]=="complete" and report["simulations"]==432
assert len(report["rows"])==432 and len(report["summary"])==9
current=[3,3.5,4]
voltage=[18,24,30]
mat_error=np.full((3,3),np.nan)
mat_reserve=np.full((3,3),np.nan)
mat_vreserve=np.full((3,3),np.nan)
for s in report["summary"]:
 assert s["completedSet"] and s["rows"]==48
 i=current.index(s["Ilimit_A"]);j=voltage.index(s["Vlimit_V"])
 mat_error[i,j]=s["worstError_N"]
 mat_reserve[i,j]=s["minimumCurrentReserve_A"]
 mat_vreserve[i,j]=s["minimumVoltageReserve_V"]
plt.rcParams.update({"font.family":"DejaVu Sans","font.size":11,"axes.titleweight":"bold"})
fig,axes=plt.subplots(1,3,figsize=(13.7,5.7))
fig.subplots_adjust(left=.065,right=.975,top=.80,bottom=.29,wspace=.41)
panels=[(mat_error,"Worst true force error (N)","RdYlGn_r",Normalize(0,.6)),
        (mat_reserve,"Minimum current reserve (A)","YlGn",Normalize(0,1)),
        (mat_vreserve,"Minimum voltage reserve (V)","YlGn",Normalize(0,9))]
for ax,(mat,title,cmap,norm) in zip(axes,panels):
 im=ax.imshow(mat,cmap=cmap,norm=norm,origin="lower",aspect="auto")
 ax.set_title(title,pad=12)
 ax.set_xticks(range(3),[str(v) for v in voltage]);ax.set_yticks(range(3),[str(i) for i in current])
 ax.set_xlabel("Supply voltage (V)");ax.set_ylabel("Current command limit (A)")
 for i in range(3):
  for j in range(3):
   ax.text(j,i,f"{mat[i,j]:.3f}",ha="center",va="center",fontweight="bold",color="#101820")
 ax.add_patch(Rectangle((.5,.5),1,1,fill=False,edgecolor="#0072b2",linewidth=3))
 cb=fig.colorbar(im,ax=ax,fraction=.045,pad=.03)
 if ax is axes[0]:cb.ax.axhline(.5,color="#1f2937",linewidth=1.2)
fig.suptitle("Voice-coil headroom with direct contact-force feedback",fontsize=18,fontweight="bold",y=.975)
fig.text(.5,.887,"Conditional reference: upper rigid; lower 19,746 N/m, 60 N·s/m. Actual B/S sensor assessed separately.",ha="center",fontsize=12)
fig.text(.065,.185,"Each cell summarizes 48 native Simulink runs: 5/50 N × 0/90° × four noise seeds × three winding cases.",fontsize=10.5)
fig.text(.065,.145,"Winding cases: nominal; Kf −10%; and joint Kf −10%, R/L +30%, current response +25%.",fontsize=10.5)
fig.text(.065,.105,"0.02 N RMS force noise and unknown 0.2 ms extra delay; measured magnet position also feeds slow recentering.",fontsize=10.5)
fig.text(.065,.065,"Force observer stays nominal. Inner current drive is calibrated to actual R/L/back EMF. Thermal design remains separate.",fontsize=10.5)
fig.text(.065,.025,"Blue outline: proposed 3.5 A /24 V setting. Reserve uses actual peak current and applied voltage; force target is 0.5 N.",fontsize=10.5)
for suffix in ("png","svg"):
 fig.savefig(folder/f"actuator_qualification_final_winding_matrix.{suffix}",dpi=170,facecolor="white")
print(json.dumps({"simulations":report["simulations"],"error":mat_error.tolist(),"currentReserve":mat_reserve.tolist(),"voltageReserve":mat_vreserve.tolist()},indent=2))
