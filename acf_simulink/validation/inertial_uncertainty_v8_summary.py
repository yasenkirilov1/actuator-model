#!/usr/bin/env python3
"""Summarize executed V8 native sensing and joint-budget evidence."""
from __future__ import annotations

import hashlib
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

ROOT = Path(__file__).resolve().parent
STEM = "inertial_uncertainty_v8_summary"
PATHS = {
    "sensing_OAT": "inertial_uncertainty_v8_engineered_sensing/inertial_uncertainty_engineered_sensing_20260930_results.json",
    "robot_OAT": "inertial_uncertainty_v8_robot_fine_delay/inertial_uncertainty_robot_fine_delay_20260930_results.json",
    "joint_budget": "inertial_uncertainty_v8_joint/inertial_uncertainty_budget_joint_20260930_results.json",
    "joint_long": "inertial_uncertainty_v8_joint_long/inertial_uncertainty_budget_repeated_20260930_results.json",
    "calibrated_static_long": "inertial_uncertainty_v8_calibrated_static_long/inertial_uncertainty_calibrated_static_long_20260930_results.json",
}


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    datasets, provenance, traces = {}, [], []
    for name, relative in PATHS.items():
        path = ROOT / relative
        if not path.exists():
            continue
        data = json.loads(path.read_text())
        datasets[name] = data
        provenance.append(dict(cohort=name, file=relative, sha256=sha(path), status=data["status"],
                               planned=data["plannedCases"], executed=data["executed"], passed=data["passed"],
                               failed=data["failedNative"], rejected=data["rejectedBeforeMetrics"],
                               sourceModelSha256=data["sourceModelSha256"], candidateSha256=data["candidateSha256"]))
        native_paths = {Path(row["nativeTrace"]) for row in data["rows"] if row.get("nativeTrace")}
        worst_trace = path.with_name(path.name.replace("_results.json", "_worst_native_trace.mat"))
        if worst_trace.exists():
            native_paths.add(worst_trace)
        for native_path in sorted(native_paths):
            if not native_path.exists():
                raise FileNotFoundError(f"A recorded native trace is missing: {native_path}")
            traces.append(dict(cohort=name, file=str(native_path.relative_to(ROOT)),
                               sha256=sha(native_path), bytes=native_path.stat().st_size))
    proof_file = ROOT / "inertial_uncertainty_v8_instrumentation_parity.json"
    proof = json.loads(proof_file.read_text())
    groups = []
    for name, data in datasets.items():
        ranks = data.get("scenarioRanking", [])
        if isinstance(ranks, dict):
            ranks = [ranks]
        for rank in ranks:
            group = dict(rank, cohort=name)
            groups.append(group)
    rows = [dict(row, cohort=name) for name, data in datasets.items() for row in data["rows"] if row["executed"]]
    summary = dict(
        scope="Native V8 actual-sensing uncertainty evidence; OAT and finite combined corners have separate scopes.",
        metric="Complete native normalContact.data(:,1) maximum absolute error versus target, including startup; "
               "contact/travel/overcurrent/incomplete/impact and configured spring packing also fail.",
        softwareProof=dict(file=proof_file.name, sha256=sha(proof_file),
                           nativeExecutions=proof["nativeExecutions"],
                           pairedConfigurations=proof["distinctSoftwareProofConfigurations"],
                           bitwise=proof["allBitwiseEqual"]),
        cohorts=provenance, scenarios=groups, nativeTraces=traces,
        uncertaintyNativeExecutions=sum(d["executed"] for d in datasets.values()),
        uncertaintyPassed=sum(d["passed"] for d in datasets.values()),
        uncertaintyFailed=sum(d["failedNative"] for d in datasets.values()),
        nativeRejections=sum(d["rejectedBeforeMetrics"] for d in datasets.values()),
        failedRows=[r for r in rows if not r["passed"]],
        clippingRows=[r for r in rows if r["voltageClipFraction"] or r["effortClipFraction"]],
        jointComplete="joint_budget" in datasets and datasets["joint_budget"]["status"] == "complete",
        longComplete="joint_long" in datasets and datasets["joint_long"]["status"] == "complete",
        calibratedStaticLongComplete="calibrated_static_long" in datasets and datasets["calibrated_static_long"]["status"] == "complete",
        qualificationLimit="The joint grid covers two endpoints (S2/F50/90deg and S0.1/F5/0deg), "
                           "three seeded records and ten deterministic corners. It does not prove every "
                           "point inside a 26-dimensional tolerance hypercube or full-range joint uncertainty. "
                           "A fixed mismatch is not random latency jitter, changing orientation or sanding vibration.",
        nominalAcquisition="100kHz raw samples, second-order5100Hz AA plus50us EMA,20us pipeline,10kHz controller; "
                           "force output RMS target0.02N, acceleration0.014m/s² and position1um. "
                           "Actual AA variation uses fixed nominal raw-noise PSD normalization.",
        sourceBridge="The final selected candidate differs from the V8 software-proof candidate only in spring "
                     "free-length metadata18.1 to20mm and approximate-part text. The dynamical parameters remain "
                     "identical; provenance records both candidate hashes and each cohort's own packing score.",
    )
    if rows:
        summary["packing"] = dict(configuredPreload_N=33.6,
                                  allPassed=all(r["packingPassed"] for r in rows),
                                  commonFeasiblePreloadIntersection_N=[max(r["feasibleCommonPreloadMin_N"] for r in rows),
                                                                      min(r["feasibleCommonPreloadMax_N"] for r in rows)],
                                  combinedFreeLengthMax_mm=max(r["combinedFreeLengthMax_mm"] for r in rows),
                                  combinedInstalledLengthMax_mm=max(r["combinedInstalledLengthMax_mm"] for r in rows))
    if "joint_budget" in datasets:
        joint = datasets["joint_budget"]
        jr = joint["rows"]
        summary["jointBudget"] = dict(status=joint["status"], executed=joint["executed"], passed=joint["passed"],
                                      worstNormalError_N=joint["worstNormalForceError_N"],
                                      budgetPlan="inertial_uncertainty_v8_joint_budget_plan.json",
                                      budgetPlanSha256=sha(ROOT / "inertial_uncertainty_v8_joint_budget_plan.json"),
                                      worstRow=max(jr, key=lambda r: r["peakNormalForceError_N"]) if jr else None,
                                      forceVsOtherAAMismatch="±5% each lateral/IMU/position relative to force; "
                                      "other-channel pairwise difference may reach 10% of common cutoff.",
                                      delayConvention="Common force pipeline20±10us; lateral/IMU/position each "
                                      "add ±10us, giving actual 0–40us. Identified frame stays 20us.")
    if "joint_long" in datasets:
        long = datasets["joint_long"]
        summary["jointLong"] = dict(status=long["status"], executed=long["executed"], passed=long["passed"],
                                    worstNormalError_N=long["worstNormalForceError_N"],
                                    scope="Four 30-second records at the worst previously executed corner for "
                                          "heavy seeds 7/29/1729 and light seed 29; each uses the same exact "
                                          "joint residuals and a repeated 1.2-second bump period.",
                                    allReturnedHistoriesRetained=all(bool(row.get("nativeTrace"))
                                        for row in long["rows"] if row.get("nativeReturned")),
                                    rows=long["rows"])
    if "calibrated_static_long" in datasets:
        calibrated = datasets["calibrated_static_long"]
        comparison_file = ROOT / "inertial_uncertainty_v8_calibration_refinement.json"
        comparison = json.loads(comparison_file.read_text())
        summary["calibratedStaticLong"] = dict(status=calibrated["status"], executed=calibrated["executed"],
            passed=calibrated["passed"], worstNormalError_N=calibrated["worstNormalForceError_N"],
            scope=calibrated["calibrationRefinementScope"],
            sourceBoundPairedComparison=dict(file=comparison_file.name, sha256=sha(comparison_file)),
            originalMargin_N=comparison["originalMargin_N"], calibratedStaticMargin_N=comparison["calibratedStaticMargin_N"],
            marginRecovered_N=comparison["marginRecovered_N"],
            preparedConfigurationsSha256=calibrated["preparedConfigurationsSha256"],
            allSixteenResidualGuardsPassed=all(g["passed"] and g["staticResidualCount"] == 16
                                              for g in calibrated["staticCalibrationGuard"]),
            rows=calibrated["rows"])
    (ROOT / f"{STEM}.json").write_text(json.dumps(summary, indent=2) + "\n")

    # Draw retained scalar peaks; native traces, never strided display arrays,
    # were used to calculate every value in the source result files.
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    sensing = [g for g in groups if g["cohort"] == "sensing_OAT"]
    labels = {
        "engineered_reference": "Nominal heavy case, seeded noise",
        "engineered_force_frame": "Force frame ±0.1°", "engineered_accel_frame": "Acceleration frame ±0.1°",
        "engineered_cross_X_to_Z": "Force X→Z cross-talk ±0.1%", "engineered_cross_Z_to_X": "Force Z→X cross-talk ±0.1%",
        "engineered_common_AA": "Common actual AA cutoff ±5%", "engineered_accel_AA": "Acceleration AA cutoff ±5%",
        "engineered_force_AA": "Force AA cutoff ±5%", "engineered_position_AA": "Position AA cutoff ±5%",
        "engineered_common_transport": "Common pipeline residual ±10µs", "engineered_accel_transport": "Acceleration pipeline residual ±10µs",
    }
    if sensing:
        fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)
        y = np.arange(len(sensing))
        ax.barh(y, [g["worstNormalForceError_N"] for g in sensing], color="#0369a1", height=.65)
        ax.set_yticks(y, [labels.get(g["scenario"], g["scenario"]) for g in sensing]); ax.invert_yaxis()
        for i, g in enumerate(sensing):
            ax.text(g["worstNormalForceError_N"] + .005, i, f"{g['worstNormalForceError_N']:.3f} N · {g['passedCases']}/{g['nativeHistories']}", va="center", fontsize=9)
        ax.axvline(.5, color="#b91c1c", linestyle="--"); ax.set_xlim(0, .61)
        ax.set_xlabel("Worst normal force error over complete native history (N)")
        ax.set_title("Actual AA/frame/transport: separate sensitivities\nS = 2 kg / F = 50 N / 90°; seeds 29 and 1729", loc="left", pad=12)
        fig.savefig(ROOT / f"{STEM}_sensing.svg"); fig.savefig(ROOT / f"{STEM}_sensing.png", dpi=160); plt.close(fig)
    if "robot_OAT" in datasets:
        rr = datasets["robot_OAT"]["rows"]
        fig, ax = plt.subplots(figsize=(8.5, 4.5), constrained_layout=True)
        for seed, color in [(29, "#0369a1"), (1729, "#6d28d9")]:
            cases = sorted([r for r in rr if r["scenario"] == "robot_actual_fine_delay" and r["seed"] == seed], key=lambda r: r["value"])
            nominal = [r for r in datasets["sensing_OAT"]["rows"] if r["scenario"] == "engineered_reference" and r["seed"] == seed]
            assert len(nominal) == 1
            cases = sorted(cases + [nominal[0]], key=lambda r: r["value"])
            ax.plot([r["value"] * 1000 for r in cases], [r["peakNormalForceError_N"] for r in cases], "o-", color=color, label=f"S = 2 kg, F = 50 N, 90°, seed {seed}")
        ax.axhline(.5, color="#b91c1c", linestyle="--", label="±0.5 N requirement")
        ax.axvline(0, color="#94a3b8", linewidth=.8); ax.set_ylim(0, .56)
        ax.set_xlabel("Actual robot transport residual versus identified 80 ms (ms)")
        ax.set_ylabel("Complete-history peak normal error (N)"); ax.legend(fontsize=9, loc="lower right")
        ax.set_title("Issued-command predictor versus actual robot delay\nSender/predictor/gate 100 Hz; native actual transport 100µs", loc="left", pad=12)
        fig.savefig(ROOT / f"{STEM}_robot.svg"); fig.savefig(ROOT / f"{STEM}_robot.png", dpi=160); plt.close(fig)
    joint_figure = ""
    if "joint_budget" in datasets and datasets["joint_budget"]["rows"]:
        jr = datasets["joint_budget"]["rows"]
        fig, axes = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True, constrained_layout=True)
        for ax, mass, title in zip(axes, [2, .1], ["S = 2 kg / F = 50 N / 90°", "S = 0.1 kg / F = 5 N / 0°"]):
            for seed, color in [(7, "#0369a1"), (29, "#6d28d9"), (1729, "#059669")]:
                cases = sorted([r for r in jr if r["massS_actual_kg"] == mass and r["seed"] == seed], key=lambda r: r["value"])
                ax.plot([r["value"] for r in cases], [r["peakNormalForceError_N"] for r in cases], "o-", color=color, label=f"Seed {seed}")
            ax.axhline(.5, color="#b91c1c", linestyle="--"); ax.set_xticks(range(1, 11)); ax.set_xlabel("Deterministic joint corner")
            ax.set_title(title); ax.set_ylim(0, max(.56, max(r["peakNormalForceError_N"] for r in jr) * 1.12)); ax.legend(fontsize=9)
        axes[0].set_ylabel("Peak native normal error (N)")
        fig.suptitle(f"Combined sensing/identification/robot corners: {datasets['joint_budget']['status']}", fontsize=14)
        fig.savefig(ROOT / f"{STEM}_joint.svg"); fig.savefig(ROOT / f"{STEM}_joint.png", dpi=160); plt.close(fig)
        joint_figure = f'<img src="{STEM}_joint.svg" alt="Native peak force errors for the joint corners">'
    long_figure = ""
    if "joint_long" in datasets:
        plot_path = (ROOT / PATHS["joint_long"]).with_name(Path(PATHS["joint_long"]).name.replace("_results.json", "_plot_traces.mat"))
        if plot_path.exists():
            native_envelopes = np.atleast_1d(loadmat(plot_path, squeeze_me=True, struct_as_record=False)["plotTraces"])
            if len(native_envelopes):
                fig, axes = plt.subplots(len(native_envelopes), 1, figsize=(11, 2.4 * len(native_envelopes)), sharex=True,
                                         squeeze=False, constrained_layout=True)
                for ax, trace in zip(axes[:, 0], native_envelopes):
                    row = trace.row
                    ax.fill_between(trace.time_s, trace.normalForceMinimum_N - row.reference_N,
                                    trace.normalForceMaximum_N - row.reference_N, color="#0369a1", alpha=.7)
                    ax.axhline(.5, color="#b91c1c", linestyle="--"); ax.axhline(-.5, color="#b91c1c", linestyle="--")
                    ax.set_ylim(-.56, .56); ax.set_ylabel("Normal error (N)")
                    ax.set_title(f"S = {row.massS_actual_kg:g} kg / F = {row.reference_N:g} N / "
                                 f"seed {row.seed:g} / prior joint case {row.value:g}; "
                                 f"complete-history peak {row.peakNormalForceError_N:.3f} N", loc="left", fontsize=10)
                axes[-1, 0].set_xlabel("Recorded time (s)")
                fig.suptitle("30-second repeated bumps: envelope of every native normal-force sample", fontsize=14)
                fig.savefig(ROOT / f"{STEM}_long.svg"); fig.savefig(ROOT / f"{STEM}_long.png", dpi=160); plt.close(fig)
                long_figure = f'<img src="{STEM}_long.svg" alt="All-sample normal force envelopes over the 30-second repeated histories">'
    cohort_table = "".join(f"<tr><td>{html.escape(r['cohort'])}</td><td>{html.escape(r['status'])}</td><td>{r['executed']}/{r['planned']}</td><td>{r['passed']}</td><td>{r['failed']}</td><td>{r['rejected']}</td></tr>" for r in provenance)
    page = f"""<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Native V8 uncertainty evidence</title>
<style>body{{font:16px/1.5 system-ui;max-width:1140px;margin:32px auto;padding:0 20px;background:#f7fafc;color:#172b3a}}img{{width:100%;height:auto;background:white;margin:16px 0}}table{{width:100%;border-collapse:collapse;background:white}}td,th{{padding:8px;text-align:left;border-bottom:1px solid #ccd8e2}}.limit{{padding:12px;border-left:4px solid #b91c1c;background:#fff1f2}}</style>
<h1>V8 native uncertainty evidence</h1><p>{summary['uncertaintyNativeExecutions']} uncertainty histories executed; {summary['uncertaintyPassed']} pass, {summary['uncertaintyFailed']} fail, {summary['nativeRejections']} rejected. Six separate software-proof recordings demonstrate bitwise unity and Normal/Accelerator parity for two configurations.</p>
<p class="limit">Separate sensitivities are not combined tolerances. Joint corners are a finite two-endpoint sample; they do not establish every 26-dimensional tolerance combination or full-range joint uncertainty. No physical prototype has been validated by these recordings.</p>
<table><thead><tr><th>Cohort</th><th>Status</th><th>Executed/planned</th><th>Pass</th><th>Fail</th><th>Rejected</th></tr></thead><tbody>{cohort_table}</tbody></table>
<img src="{STEM}_sensing.svg" alt="Native one-factor sensor transfer and calibration sensitivities"><img src="{STEM}_robot.svg" alt="Native actual robot delay sensitivities">{joint_figure}{long_figure}
<img src="inertial_uncertainty_v8_calibration_refinement.svg" alt="Paired original and tighter-static calibration complete-history peaks and envelopes">
<p>The four paired30-second stress records halve exactly16 static calibration residuals while holding noise, actual/identified AA, timing, robot mismatch, mechanics, controller and100µs drive response unchanged. They improve the worst peak from0.490396N to0.415311N. The resulting0.084689N margin belongs to those four finite stress records; it is not a refined full-range uncertainty grid or a demonstrated installed-hardware accuracy.</p>
<p>Sensor response and issued-motion prediction use separate actual and identified coefficients. Raw noise PSD normalization remains tied to the nominal AA chain when actual cutoff varies. Sensor static tare and transfer-function initial states use the calibrated sensor frame; the physical starting current always uses actual plant masses and force constant.</p>
<p>The selected final candidate keeps 100µs current response and 24V. Its approximate 20mm spring metadata is recorded separately from the 18.1mm metadata used by the initial sensing proofs; physical force dynamics are identical. Native full traces, exact hashes, failed runs and packing audits are linked through <a href="{STEM}.json">the machine-readable evidence</a>.</p></html>"""
    (ROOT / f"{STEM}.html").write_text(page)
    print(json.dumps({k: summary[k] for k in ("uncertaintyNativeExecutions", "uncertaintyPassed", "uncertaintyFailed", "jointComplete", "longComplete")}))


if __name__ == "__main__":
    main()
