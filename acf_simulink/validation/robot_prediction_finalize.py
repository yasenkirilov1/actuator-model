"""Consolidate saved native experiment rows without rerunning MATLAB."""
from pathlib import Path
from collections import Counter
import csv
import json
import numpy as np
from scipy.io import savemat

BASE=Path(__file__).resolve().parent
phases=[]; allrows=[]
for path in sorted(BASE.glob("robot_prediction_*.json")):
    data=json.loads(path.read_text())
    if "rows" not in data: continue
    rows=data["rows"];rows=[rows] if isinstance(rows,dict) else rows
    phases.append({
        "phase":data["phase"],"status":data["status"],"configurationRows":len(rows),
        "nativeExecutions":sum(row["executed"] for row in rows),
        "preparationRejected":sum(not row["executed"] for row in rows),
        "performancePasses":sum(row["passed"] for row in rows),
        "forceOnlyPasses":sum(row["forceOnlyPassed"] for row in rows),
        "sourceJson":str(path),"privateModel":data["privateModel"],
        "privateHelper":data.get("privateObserverHelper"),
        "sourceScript":data.get("experimentSource"),
        "frozenPerPhaseSource":bool(data.get("experimentSource")),
    })
    allrows += [{"phase":data["phase"],**row} for row in rows]
counts={key:sum(item[key] for item in phases) for key in
        ("configurationRows","nativeExecutions","preparationRejected","performancePasses","forceOnlyPasses")}
counts["performanceFailuresAfterExecution"]=counts["nativeExecutions"]-counts["performancePasses"]
selected={}
for phase in ("tool_jerk_ideal","tool_jerk_noise","tool_jerk_cost","tool_jerk_current"):
    rows=[row for row in allrows if row["phase"]==phase and row["executed"]]
    selected[phase]={"best":min(rows,key=lambda row:row["peakForceError_N"]),
                     "nativeExecutions":len(rows),"passes":sum(row["passed"] for row in rows)}
report={
    "date":"2026-09-30","status":"complete",
    "scope":"Issued-command robot predictor, physical B/S load-cell inertial sensing, native S-relative-A position and optional explicitly measured drive-current readback; private experiments only.",
    "persistedDistinctRows":counts,"phases":phases,
    "preparationRejectionReasons":dict(Counter(row["reason"] for row in allrows if not row["executed"])),
    "selectedUpstreamCompliantComparisons":selected,
    "attemptLedger":{
        "retainedLogNativeExecutions":846,"retainedLogRejectedRowAttempts":70,
        "retainedLogTotalRowAttempts":916,
        "extraNativeRerun":"One superseded current-readback flat sanity with high current-process uncertainty was simulated and failed from roundoff (0.839948 N). Final current sanity passes.",
        "extraRejectedRow":"One superseded current-sanity attempt rejected invalid zero-width Uniform Random Number settings before native simulation. Fixed by unit-RMS noise followed by a zero gain.",
        "nonRowConstructionRejections":3,
        "nonRowDetails":["Old driver queued precision before switch case existed.",
                         "range constructor referenced optional helper before it was present.",
                         "tool_jerk_sanity observer D initially had nine rather than ten rows."],
        "note":"Distinct row count uses final complete artifacts. Superseded attempts are separately retained in driver logs and are not additional qualification points.",
        "logs":[str(Path("/home/tommie/.local/state/actuator-model/qualification-20260930")/name)
                for name in ("robot_prediction_smoke.log","robot_prediction_driver.log","robot_prediction_driver_v2.log")],
    },
    "interpretation":[
        "The 296 persisted passes include 282 near-contact counterfactual cases, 12 noiseless upstream sharp cases, and two flat initialization checks. None is a full noisy upstream prototype qualification from this lane.",
        "Historical lower-rigid/both-rigid command-prediction baselines lacked the direct estimated Coulomb measurement/output correction. They show those tested baseline controllers failed, not that a corrected rigid mechanism is incapable.",
        "Older phases without frozenPerPhaseSource used a shared private model/helper that was subsequently overwritten. Selected trace MAT files still contain their actual prepared parameters; the native physical plant snapshot is immutable. Do not treat the final shared helper as the exact historical controller source.",
        "The actual B/S cell has 1.4 kg downstream. Its measurement matrix subtracts 1.4 kg from the true-force surface-acceleration coefficient.",
        "Tool-position measurement is physical xS-xA sampled, noisy and delayed; no surface profile, actual surface acceleration or physical robot-motion truth feeds the controller.",
        "Robot prediction uses only the issued velocity request and calibrated command delay/response/speed/acceleration limits. Knowing the command alone does not measure actual motion or remove calibration error.",
        "The optional third sensor explicitly models drive-current readback with 1 mA RMS noise and 100 us known delay. Its tested twelve-case controller grid did not pass.",
        "All plotted source MAT files come from native Simulink plant runs. Standalone PNG/SVG figures are inspectable; source paths are in robot_prediction_figures_sources.json.",
    ],
    "figures":[str(BASE/name) for name in
               ("robot_prediction_upstream_jerk_diagnostics.svg","robot_prediction_timing_comparison.svg","robot_prediction_current_readback_negative.svg")],
    "workerStopped":True,"workerPid":3079417,
    "workerStopMechanism":"Own exit sentinel /tmp/acf_robot_predictor_exit_v2 issued; process verified absent. The exec session reported exit 143 after all jobs completed.",
}
(BASE/"robot_prediction_experiment_summary.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
with (BASE/"robot_prediction_experiment_summary.csv").open("w",newline="") as file:
    writer=csv.DictWriter(file,fieldnames=phases[0].keys());writer.writeheader();writer.writerows(phases)
fields=["phase"]+sorted(set().union(*(row.keys() for row in allrows))-{ "phase" })
with (BASE/"robot_prediction_all_rows.csv").open("w",newline="") as file:
    writer=csv.DictWriter(file,fieldnames=fields);writer.writeheader();writer.writerows(allrows)
savemat(BASE/"robot_prediction_experiment_summary.mat",{
    "summary_json":json.dumps(report,allow_nan=False),"distinct_counts":counts,
    "phase_names":np.array([item["phase"] for item in phases],dtype=object),
    "phase_count_columns":np.array(["configurationRows","nativeExecutions","preparationRejected","performancePasses"],dtype=object),
    "phase_counts":np.array([[item[key] for key in ("configurationRows","nativeExecutions","preparationRejected","performancePasses")] for item in phases]),
},do_compression=True,long_field_names=True)
print(json.dumps(counts))
