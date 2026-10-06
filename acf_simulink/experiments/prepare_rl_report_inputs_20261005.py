"""Bind completed native RL evidence to inspectable report queries.

This recipe never executes or qualifies a simulation. It requires locked,
completed final evaluations and preserves all source files unchanged.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
STUDY=ROOT/"acf_simulink/validation/rl_budget3h_20261005"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def relative(path):
    return str(Path(path).resolve().relative_to(ROOT))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--final",type=Path,required=True)
    parser.add_argument("--grid",type=Path)
    parser.add_argument("--long",type=Path)
    parser.add_argument("--selection-review",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    queries=[];numbers={};scope_notes={}

    def query(name,rows,sources,label,kind,caveats=()):
        assert rows and isinstance(rows,list),name
        payload={"status":"complete","kind":kind,"rows":rows,
                 "sourceHashes":{relative(p):sha(p) for p in sources},
                 "caveats":list(caveats)}
        path=args.output/(name+".json")
        path.write_text(json.dumps(payload,indent=2,allow_nan=False)+"\n")
        queries.append({"id":name,"file":relative(path),"sha256":sha(path),
                        "sourceFiles":[relative(p) for p in sources],
                        "label":label,"kind":kind,"caveats":list(caveats)})

    contract=read(STUDY/"study_contract.json")
    band=read(STUDY/"selection_contract_band_v2.json")
    review=read(args.selection_review)
    query("rl_method_contract",[
        {"topic":"Task","value":contract["goal"]},
        {"topic":"Travel objective","value":"Zero centering cost inside ±7.5 mm; increasing soft cost outside; physical stops unchanged."},
        {"topic":"Physical stop travel","minimum_m":0,"maximum_m":.025,"midpoint_m":.0125},
        {"topic":"Unified policy","causalInputs":40,"derivedCausalProducts":8,"recurrentLayers":2,"statesPerLayer":32,"storedParameters":8101},
        {"topic":"Decision clocks","force_Hz":10000,"robot_Hz":100,"nativePhysics_Hz":300000},
        {"topic":"Action mapping","current":"Known static equilibrium + 2 A × tanh(current output)","rawRobot_m_s":.01},
        {"topic":"Inputs","value":"Timestamp-aligned force/acceleration/position observers, identified physical context, and robot commands already issued; no surface preview or plant truth."},
        {"topic":"Selection ranking","value":band["ranking"],"score":band["joint_score_formula"]},
        {"topic":"Selected checkpoint","sha256":review["selectedCheckpointSha256"]},
        {"topic":"Finite evidence","value":"Native simulation recordings; no physical prototype qualification or full uncertainty-box proof."},
    ],[STUDY/"study_contract.json",STUDY/"selection_contract_band_v2.json",args.selection_review],
          "Approved task and locked recurrent-policy methodology","method_and_user_constraints")

    parts=STUDY/"budget_parts_review_20261005.json"
    if parts.exists():
        document=read(parts)
        query("rl_budget_parts",[{"review":document}],[parts],
              "Live quantity-one chip prices and primary ADXL354C/M08 specifications","manufacturer_and_distributor_review",
              ["Chips for the user's PCB; amplifier, board, clock, assembly and shipping costs are additional.",
               "The 1.9 kHz matched response is a declared model envelope, not a complete measured ADXL354 transfer."])
        numbers["budgetParts"]=document

    budget_preparation=STUDY/"native_bank_budget50/preparation_acceptance.json"
    if budget_preparation.exists():
        query("rl_budget_sensing_contract",[{"preparation":read(budget_preparation)}],[budget_preparation],
              "Budget sensing, noise, ADC scaling and 50 µs current-response inputs","frozen_model_assumptions",
              ["This record binds preparation; only completed final results establish finite simulation outcomes."])

    ppo_ledger=STUDY/"runner_checks/ppo_final_ledger_v1/ledger.json"
    ppo=read(ppo_ledger)
    assert ppo["status"]=="complete"
    query("rl_ppo_training",ppo["attempts"],[ppo_ledger],
          "Executed PPO runs and preserved rejections","optimizer_training",
          ["Decisions used by completed updates and the 98,304 post-collection rejected decisions are separate.",
           "Different warm starts, seeds, reward functions and development banks are explicit; this is not a one-factor causal comparison."])
    numbers["ppo"]=ppo

    ledger=STUDY/"reward_search_completed_ledger.json"
    if ledger.exists():
        doc=read(ledger)
        # Keep the complete audited ledger available without guessing field names.
        query("rl_reward_search",[{"ledger":doc}],[ledger],
              "Audited episodic reward-search counts and checkpoint ancestry","reward_search_execution",
              ["Evolved directions differ from total stored neural parameters; read the ancestry and completed-stage counts.",
               "Recovery replays and unscored interrupted work are not new training generations."])
        numbers["rewardSearch"]=doc
    auxiliary=STUDY/"reward_search_auxiliary_ledger.json"
    if auxiliary.exists():
        query("rl_initialization_probes",[{"ledger":read(auxiliary)}],[auxiliary],
              "Separate physics-initialization comparisons and implementation audits","development_initialization",
              ["These are separate episode evaluations, not additional evolutionary generations or fresh qualification."])
    budget_ledger=STUDY/"reward_search_budget_ledger.json"
    if budget_ledger.exists():
        document=read(budget_ledger)
        query("rl_budget_reward_search",[{"ledger":document}],[budget_ledger],
              "Separate budget-chain screen, history attenuation and constrained search","budget_policy_execution",
              ["The final winner retains previously evolved weights with reward-selected history attenuation; the short budget CMA continuation did not outrank it.",
               "Winner development coverage is its own 20-case training screen and 28-case selection, not the alternate policy's 84-case audit."])
        numbers["budgetRewardSearch"]=document

    selected_evidence=[Path(item["file"]) for item in review.get("selectionEvidence",[])]
    query("rl_selected_candidate",[{"review":review}],[args.selection_review,*selected_evidence],
          "Selection-only review and selected weight identity","candidate_selection",
          ["Weights are selected using development evidence only, before final outcomes."])

    implementation_paths=[
        STUDY/"runner_checks/cpu_cuda_benchmark_v1/results.json",
        STUDY/"runner_checks/inverse_replay_v1/results.json",
        ROOT/"acf_simulink/validation/rl_puffer_probe_20261005_final_driver_checks/acceptance_v3/run_v2/acceptance_results.json",
        ROOT/"acf_simulink/validation/final_driver_checks/final_budget_selected_acceptance/results.json",
    ]
    query("rl_implementation_checks",[{"file":relative(p),"evidence":read(p)} for p in implementation_paths],
          implementation_paths,"GPU benchmark, replay guard and native evaluation acceptance","software_verification",
          ["Desktop timing is not a microcontroller deadline measurement; high-gain C and PyTorch arithmetic are not interchangeable."])

    for name,path,label in [("rl_fresh_tests",args.final,"Fresh 4–6 s paired tests"),
                            ("rl_nominal_grid",args.grid,"Fresh-seed nominal hot-coil grid"),
                            ("rl_long_tests",args.long,"Fresh 30 s repeated-bump recordings")]:
        if path is None:continue
        doc=read(path)
        assert doc["status"]=="complete" and doc["lockedInputsUnchanged"],path
        assert doc["contract"]["checkpointSha256"]==review["selectedCheckpointSha256"]
        rows=[]
        for controller,outcome in doc["outcomes"].items():
            rows += [{"controller":controller,**row} for row in outcome["rows"]]
        query(name,rows,[path,args.selection_review],label,"locked_native_evaluation",
              ["Every native force sample is scored, including startup and endpoint.",
               "The ±7.5 mm travel allowance is a separate soft objective; contact loss and travel stops fail a run.",
               "Finite simulated cases and noise seeds do not establish physical sanding performance."])
        numbers[name]={"status":doc["status"],"completedRecordings":doc["completedRecordings"],
                      "totalNativeSamples":doc["totalNativeSamples"],
                      "outcomes":{k:{a:b for a,b in v.items() if a!="rows"} for k,v in doc["outcomes"].items()},
                      "paired":doc.get("paired",[])}

    probe=STUDY/"native_actuator_probe_results/summary.json"
    broad=STUDY/"native_current50_results/summary.json"
    query("rl_actuator_response",read(probe)["rows"]+read(broad)["rows"],
          [probe,broad,STUDY/"native_current50_results/metric_semantics_overlay.json"],
          "Fixed earlier checkpoint: voltage and current-response development tests","development_counterfactual",
          ["These tests use frozen gen5 checkpoint 48618b8d, not necessarily the final winning weights.",
           "50 µs changes current-loop tuning and matching identified response; voltage-only pairs retain 100 µs tuning.",
           "Ignore the inherited historical recenterPassed flag; use native band excess and actual travel metrics."])
    numbers["actuatorProbe"]=read(probe)
    numbers["currentResponse"]=read(broad)

    limits=STUDY/"reward_search_budget_limits/comparison.json"
    query("rl_budget_response_limits",[{"comparison":read(limits)}],[limits],
          "Locked budget policy: current response and added sensing delay","development_sensitivity",
          ["112 new paired development recordings, no retuning; primary references are reused.",
           "Known matched latency is still harmful; a filter cutoff does not establish its complete transfer function."])
    numbers["budgetResponseLimits"]=read(limits)

    memory=ROOT/"acf_simulink/validation/rl_puffer_probe_20261005_final_driver_checks/memory_ablation_gen5/results.json"
    query("rl_memory_reliance",read(memory)["paired"],[memory],
          "Four development cases: same weights with and without carried memory","development_ablation",
          ["Resetting memory each frame is an intervention in this controller, not an independently optimized memoryless baseline.",
           "This recorded ablation uses the earlier gen5 checkpoint; final-winner results must be separately identified."])
    numbers["memory"]=read(memory)

    gate=STUDY/"reward_search_inverse_gate_figure/data_sources.json"
    query("rl_gate_tradeoff",[{"evidence":read(gate)}],[gate],
          "Stronger inverse force response can block robot commands and exhaust travel","development_failure_comparison",
          ["Full and partial inverse initialization in the same development case; not the final winning checkpoint."])

    (args.output/"report_numbers.json").write_text(json.dumps(numbers,indent=2,allow_nan=False)+"\n")
    inputs={"status":"complete","queries":queries,"figures":[],
            "robotQueryIds":["rl_method_contract","rl_fresh_tests"],
            "actuatorQueryIds":["rl_actuator_response"],
            "methodQueryIds":["rl_method_contract"],"sectionQueries":{}}
    if parts.exists():
        inputs["accelerationQueryIds"]=["rl_budget_parts","rl_budget_sensing_contract","rl_fresh_tests","rl_long_tests"]
        inputs["acquisitionQueryIds"]=inputs["accelerationQueryIds"]
    (args.output/"report_inputs.json").write_text(json.dumps(inputs,indent=2)+"\n")
    print(json.dumps({"queries":len(queries),"output":str(args.output)}))


if __name__=="__main__":
    main()
